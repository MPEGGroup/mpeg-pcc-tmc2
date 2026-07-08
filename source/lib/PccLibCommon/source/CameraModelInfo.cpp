/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "CameraModelInfo.h"

#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>
#include <unordered_map>
#include <algorithm>

CameraModel stringToModel( const std::string &modelStr ) {
  static const std::unordered_map<std::string, CameraModel> kMap = { { "SIMPLE_PINHOLE", CameraModel::SIMPLE_PINHOLE },
                                                                     { "PINHOLE", CameraModel::PINHOLE } };

  auto it = kMap.find( modelStr );
  if ( it != kMap.end() ) return it->second;

  throw std::runtime_error( "Unknown camera model: " + modelStr );
}

const char *modelToString( CameraModel m ) {
  switch ( m ) {
    case CameraModel::SIMPLE_PINHOLE: return "SIMPLE_PINHOLE";
    case CameraModel::PINHOLE: return "PINHOLE";
    default: return "UNKNOWN";
  }
}

size_t numParams( CameraModel model ) {
  switch ( model ) {
    case CameraModel::SIMPLE_PINHOLE: return 3;
    case CameraModel::PINHOLE: return 4;
    default: return 0;
  }
}

namespace {
std::string readString( std::ifstream &in ) {
  std::string s;
  char        c;
  while ( in.get( c ) && c != '\0' ) s.push_back( c );
  return s;
}

template <typename T>
void readBinary( std::ifstream &in, T &value ) {
  in.read( reinterpret_cast<char *>( &value ), sizeof( T ) );
}
}  // namespace

CameraModelInfo::PinholeParams CameraModelInfo::getPinholeParams( uint32_t camera_id ) const {
  const auto &intr = cameras_.at( camera_id );
  return convertToPinhole( intr );
}

bool CameraModelInfo::isBinary( const std::string &file ) { return file.find( ".bin" ) != std::string::npos; }

CameraModelInfo::PinholeParams CameraModelInfo::convertToPinhole( const CamIntrinsics &cam ) {
  PinholeParams p{};
  p.width  = cam.width;
  p.height = cam.height;

  const auto &v = cam.params;

  switch ( cam.model ) {
    case CameraModel::SIMPLE_PINHOLE:
      // [f, cx, cy]
      p.fx = v[0];
      p.fy = v[0];
      p.cx = v[1];
      p.cy = v[2];
      break;

    case CameraModel::PINHOLE:
      // [fx, fy, cx, cy]
      p.fx = v[0];
      p.fy = v[1];
      p.cx = v[2];
      p.cy = v[3];
      break;

    default: throw std::runtime_error( "pinhole conversion does not exist for the model" );
  }

  return p;
}

bool CameraModelInfo::hasSameIntrinsics() const {
  if ( cameras_.size() <= 1 ) return true;

  auto        it    = cameras_.begin();
  const auto &first = it->second;
  ++it;

  for ( ; it != cameras_.end(); ++it ) {
    const auto &cam = it->second;

    if ( cam.model != first.model || cam.width != first.width || cam.height != first.height ||
         cam.params != first.params ) {
      return false;
    }
  }

  return true;
}

void CameraModelInfo::readColmapData( const std::string &cameras_file, const std::string &images_file ) {
  images_.clear();
  cameras_.clear();

  if ( isBinary( cameras_file ) )
    readCamerasBin( cameras_file, cameras_ );
  else
    readCamerasTxt( cameras_file, cameras_ );

  if ( isBinary( images_file ) )
    readImagesBin( images_file, cameras_ );
  else
    readImagesTxt( images_file, cameras_ );
}

size_t CameraModelInfo::numCameras() const { return cameras_.size(); }

void CameraModelInfo::readCamerasTxt( const std::string &file, std::unordered_map<uint32_t, CamIntrinsics> &camMap ) {
  std::ifstream in( file );
  if ( !in ) throw std::runtime_error( "Cannot open cameras.txt" );

  std::string line;

  while ( std::getline( in, line ) ) {
    if ( line.empty() || line[0] == '#' ) continue;

    std::istringstream iss( line );

    CamIntrinsics cam;
    std::string   modelStr;

    iss >> cam.camera_id >> modelStr >> cam.width >> cam.height;

    cam.model = stringToModel( modelStr );

    size_t n = numParams( cam.model );
    cam.params.resize( n );

    for ( size_t i = 0; i < n; ++i ) iss >> cam.params[i];

    camMap[cam.camera_id] = std::move( cam );
  }
}

void CameraModelInfo::readImagesTxt( const std::string                                 &file,
                                     const std::unordered_map<uint32_t, CamIntrinsics> &camMap ) {
  std::ifstream in( file );
  if ( !in ) throw std::runtime_error( "Cannot open images.txt" );

  std::string line;

  while ( std::getline( in, line ) ) {
    if ( line.empty() || line[0] == '#' ) continue;

    std::istringstream iss( line );

    CamExtrinsics img;

    iss >> img.image_id >> img.qw >> img.qx >> img.qy >> img.qz >> img.tx >> img.ty >> img.tz >> img.camera_id >>
        img.image_name;

    if ( camMap.find( img.camera_id ) == camMap.end() )
      throw std::runtime_error( "unknown camera_id for image reference" );

    images_.push_back( std::move( img ) );

    std::getline( in, line );
  }
}

void CameraModelInfo::readCamerasBin( const std::string &file, std::unordered_map<uint32_t, CamIntrinsics> &camMap ) {
  std::ifstream in( file, std::ios::binary );
  if ( !in ) throw std::runtime_error( "Cannot open cameras.bin" );

  uint64_t num_cameras = 0;
  readBinary( in, num_cameras );

  for ( uint64_t i = 0; i < num_cameras; ++i ) {
    CamIntrinsics cam;

    uint32_t model_id;

    readBinary( in, cam.camera_id );
    readBinary( in, model_id );
    readBinary( in, cam.width );
    readBinary( in, cam.height );

    cam.model = static_cast<CameraModel>( model_id );

    size_t n = numParams( cam.model );
    cam.params.resize( n );

    for ( size_t j = 0; j < n; ++j ) readBinary( in, cam.params[j] );

    camMap[cam.camera_id] = std::move( cam );
  }
}

void CameraModelInfo::readImagesBin( const std::string                                 &file,
                                     const std::unordered_map<uint32_t, CamIntrinsics> &camMap ) {
  std::ifstream in( file, std::ios::binary );
  if ( !in ) throw std::runtime_error( "Cannot open images.bin" );

  uint64_t num_images = 0;
  readBinary( in, num_images );

  for ( uint64_t i = 0; i < num_images; ++i ) {
    CamExtrinsics img;

    readBinary( in, img.image_id );
    readBinary( in, img.qw );
    readBinary( in, img.qx );
    readBinary( in, img.qy );
    readBinary( in, img.qz );
    readBinary( in, img.tx );
    readBinary( in, img.ty );
    readBinary( in, img.tz );
    readBinary( in, img.camera_id );

    img.image_name = readString( in );

    if ( camMap.find( img.camera_id ) == camMap.end() )
      throw std::runtime_error( "unknown camera_id for image reference" );

    uint64_t num_points2D;
    readBinary( in, num_points2D );

    for ( uint64_t j = 0; j < num_points2D; ++j ) {
      double   x, y;
      uint64_t id;

      readBinary( in, x );
      readBinary( in, y );
      readBinary( in, id );
    }

    images_.push_back( std::move( img ) );
  }
}
