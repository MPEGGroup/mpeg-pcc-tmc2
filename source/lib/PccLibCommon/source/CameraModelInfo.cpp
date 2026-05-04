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

template <typename T>
static void readBinary( std::ifstream &file, T &value ) {
  file.read( reinterpret_cast<char *>( &value ), sizeof( T ) );
}

bool CameraModelInfo::isBinary( const std::string &file ) { return file.find( ".bin" ) != std::string::npos; }

bool CameraModelInfo::hasSameIntrinsics() const {
  if ( cameras_.empty() ) return false;

  auto        it    = cameras_.begin();
  const auto &first = it->second;
  ++it;

  for ( ; it != cameras_.end(); ++it ) {
    const auto &c = it->second;

    if ( c.fx != first.fx || c.fy != first.fy || c.cx != first.cx || c.cy != first.cy || c.width != first.width ||
         c.height != first.height )
      return false;
  }

  return true;
}

void CameraModelInfo::load( const std::string &cameras_file, const std::string &images_file ) {
  frames_.clear();
  cameras_.clear();

  if ( isBinary( cameras_file ) )
    loadCamerasBin( cameras_file, cameras_ );
  else
    loadCamerasTxt( cameras_file, cameras_ );

  if ( isBinary( images_file ) )
    loadImagesBin( images_file, cameras_ );
  else
    loadImagesTxt( images_file, cameras_ );
}

size_t CameraModelInfo::numCameras() const { return cameras_.size(); }

void CameraModelInfo::loadCamerasTxt( const std::string                               &file,
                                      std::unordered_map<uint32_t, CameraModelParams> &camMap ) {
  std::ifstream f( file );
  if ( !f ) throw std::runtime_error( "Cannot open cameras file: " + file );

  std::string line;

  while ( std::getline( f, line ) ) {
    if ( line.empty() || line[0] == '#' ) continue;

    std::stringstream ss( line );

    uint32_t    cam_id, width, height;
    std::string model;
    double      fx, fy, cx, cy;

    ss >> cam_id >> model >> width >> height >> fx >> fy >> cx >> cy;

    CameraModelParams cam;
    cam.camera_id = cam_id;
    cam.width     = width;
    cam.height    = height;
    cam.fx        = fx;
    cam.fy        = fy;
    cam.cx        = cx;
    cam.cy        = cy;

    camMap[cam_id] = cam;
  }
}

void CameraModelInfo::loadImagesTxt( const std::string                                     &file,
                                     const std::unordered_map<uint32_t, CameraModelParams> &camMap ) {
  std::ifstream f( file );
  if ( !f ) throw std::runtime_error( "Cannot open images file: " + file );

  std::string line;

  while ( std::getline( f, line ) ) {
    if ( line.empty() || line[0] == '#' ) continue;

    std::stringstream ss( line );

    CameraModelParams frame;

    ss >> frame.image_id >> frame.qw >> frame.qx >> frame.qy >> frame.qz >> frame.tx >> frame.ty >> frame.tz >>
        frame.camera_id >> frame.image_name;

    auto it = camMap.find( frame.camera_id );
    if ( it == camMap.end() ) throw std::runtime_error( "Camera ID mismatch in images file." );

    frame.width  = it->second.width;
    frame.height = it->second.height;
    frame.fx     = it->second.fx;
    frame.fy     = it->second.fy;
    frame.cx     = it->second.cx;
    frame.cy     = it->second.cy;

    frames_.push_back( frame );

    // Skip POINTS2D line
    std::getline( f, line );
  }
}

void CameraModelInfo::loadCamerasBin( const std::string                               &file,
                                      std::unordered_map<uint32_t, CameraModelParams> &camMap ) {
  std::ifstream f( file, std::ios::binary );
  if ( !f ) throw std::runtime_error( "Cannot open cameras bin file: " + file );

  uint64_t num_cameras;
  readBinary( f, num_cameras );

  for ( uint64_t i = 0; i < num_cameras; ++i ) {
    uint32_t cam_id;
    int32_t  model_id;
    uint64_t width, height;

    readBinary( f, cam_id );
    readBinary( f, model_id );
    readBinary( f, width );
    readBinary( f, height );

    double params[4];
    f.read( reinterpret_cast<char *>( params ), 4 * sizeof( double ) );

    CameraModelParams cam;
    cam.camera_id = cam_id;
    cam.width     = width;
    cam.height    = height;
    cam.fx        = params[0];
    cam.fy        = params[1];
    cam.cx        = params[2];
    cam.cy        = params[3];

    camMap[cam_id] = cam;
  }
}

void CameraModelInfo::loadImagesBin( const std::string                                     &file,
                                     const std::unordered_map<uint32_t, CameraModelParams> &camMap ) {
  std::ifstream f( file, std::ios::binary );
  if ( !f ) throw std::runtime_error( "Cannot open images bin file: " + file );

  uint64_t num_images;
  readBinary( f, num_images );
  CameraModelParams frame;
  double            qw, qx, qy, qz;
  double            tx, ty, tz;

  for ( uint64_t i = 0; i < num_images; ++i ) {
    CameraModelParams frame;

    readBinary( f, frame.image_id );

    double qw, qx, qy, qz;
    double tx, ty, tz;

    readBinary( f, qw );
    readBinary( f, qx );
    readBinary( f, qy );
    readBinary( f, qz );
    readBinary( f, tx );
    readBinary( f, ty );
    readBinary( f, tz );

    readBinary( f, frame.camera_id );

    frame.qw = qw;
    frame.qx = qx;
    frame.qy = qy;
    frame.qz = qz;
    frame.tx = tx;
    frame.ty = ty;
    frame.tz = tz;

    char ch;
    while ( f.get( ch ) && ch != '\0' ) frame.image_name += ch;

    uint64_t num_points2D;
    readBinary( f, num_points2D );

    f.seekg( num_points2D * ( 2 * sizeof( double ) + sizeof( int64_t ) ), std::ios::cur );

    auto it = camMap.find( frame.camera_id );
    if ( it == camMap.end() ) throw std::runtime_error( "Camera ID mismatch in images bin file." );

    frame.width  = it->second.width;
    frame.height = it->second.height;
    frame.fx     = it->second.fx;
    frame.fy     = it->second.fy;
    frame.cx     = it->second.cx;
    frame.cy     = it->second.cy;

    frames_.push_back( frame );
  }
}
