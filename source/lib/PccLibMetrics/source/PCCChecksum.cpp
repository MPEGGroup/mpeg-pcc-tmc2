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
#include "PCCCommon.h"
#include "PCCGroupOfFrames.h"
#include "PCCPointSet.h"

#include "PCCChecksum.h"

using namespace std;
using namespace pcc;

PCCChecksum::PCCChecksum() = default;
PCCChecksum::~PCCChecksum() {
  checksumsSrc_.clear();
  checksumsOrd_.clear();
  checksumsRec_.clear();
  checksumsDec_.clear();
}
void PCCChecksum::setParameters( const PCCMetricsParameters& params ) { params_ = params; }

void PCCChecksum::computeSource( PCCGroupOfFrames& groupOfFrames ) {
  for ( auto& frame : groupOfFrames ) {
    checksumsSrc_.push_back( frame.computeChecksum( true ) );
    printf( "ChecksumSrc: " );
    for ( auto& c : checksumsSrc_.back() ) { printf( "%02x", c ); }
    printf( "\n" );
    fflush( stdout );
  }
}

void PCCChecksum::computeReordered( PCCGroupOfFrames& groupOfFrames ) {
  for ( auto& frame : groupOfFrames ) {
    checksumsOrd_.push_back( frame.computeChecksum( true ) );
    printf( "ChecksumOrd: " );
    for ( auto& c : checksumsOrd_.back() ) { printf( "%02x", c ); }
    printf( "\n" );
    fflush( stdout );
  }
}
void PCCChecksum::computeReconstructed( PCCGroupOfFrames& groupOfFrames ) {
  for ( auto& frame : groupOfFrames ) {
    checksumsRec_.push_back( frame.computeChecksum() );
    printf( "ChecksumRec: " );
    for ( auto& c : checksumsRec_.back() ) { printf( "%02x", c ); }
    printf( "\n" );
    fflush( stdout );
  }
}

void PCCChecksum::computeDecoded( PCCGroupOfFrames& groupOfFrames ) {
  for ( auto& frame : groupOfFrames ) {
    checksumsDec_.push_back( frame.computeChecksum() );
    printf( "ChecksumDec: " );
    for ( auto& c : checksumsDec_.back() ) { printf( "%02x", c ); }
    printf( "\n" );
    fflush( stdout );
  }
}

void PCCChecksum::read( const std::string& compressedStreamPath ) {
  std::ifstream fin( removeFileExtension( compressedStreamPath ) + ".checksum", std::ios::in );
  if ( fin.is_open() ) {
    size_t numberOfFrames = 0;
    size_t sizeChecksum   = 0;
    fin >> numberOfFrames >> sizeChecksum;
    checksumsRec_.resize( numberOfFrames );
    for ( auto& checksum : checksumsRec_ ) {
      checksum.resize( sizeChecksum, 0 );
      for ( auto& c : checksum ) {
        uint8_t c0;
        uint8_t c1;
        fin >> c0 >> c1;
        c = ( ( c0 + ( c0 > '9' ? 9 : 0 ) ) & 0x0F ) * 16 + ( ( c1 + ( c1 > '9' ? 9 : 0 ) ) & 0x0F );
      }
      printf( "ChecksumRec: " );
      for ( auto& c : checksum ) { printf( "%02x", c ); }
      printf( "\n" );
    }
    fin.close();
  }
}

void PCCChecksum::write( const std::string& compressedStreamPath ) {
  std::ofstream fout( removeFileExtension( compressedStreamPath ) + ".checksum", std::ios::out );
  if ( !fout.is_open() ) { return; }
  fout << checksumsRec_.size() << std::endl;
  fout << ( checksumsRec_.empty() ? 0 : checksumsRec_[0].size() ) << std::endl;
  for ( auto& checksum : checksumsRec_ ) {
    for ( auto& c : checksum ) { fout << std::hex << ( c / 16 ) << std::hex << ( c % 16 ); }
    fout << std::endl;
  }
  fout.close();
}

bool PCCChecksum::compare( std::vector<std::vector<uint8_t>>& checksumsA,
                           std::vector<std::vector<uint8_t>>& checksumsB ) {
  size_t num   = ( std::min )( checksumsA.size(), checksumsB.size() );
  bool   equal = checksumsA.size() == checksumsB.size();
  for ( size_t i = 0; equal && ( i < num ); i++ ) {
    if ( checksumsA[i] != checksumsB[i] ) { equal = false; }
    printf( "PLY %4zu: [MD5:", params_.startFrameNumber_ + i );
    for ( auto& c : checksumsA[i] ) { printf( "%02x", c ); }
    printf( "," );
    for ( auto& c : checksumsB[i] ) { printf( "%02x", c ); }
    printf( ",(%s)] \n", checksumsA[i] == checksumsB[i] ? "OK" : "DIFF" );
  }
  return equal;
}

bool PCCChecksum::compareSrcRec() {
  bool equal = compare( checksumsSrc_, checksumsOrd_ );
  printf( "SEQUENCE SOURCE AND RECONSCTRUTED MD5 %s => ENCODING IS %s\n", equal ? "OK" : "DIFF",
          equal ? "LOSSLESS" : "NOT LOSSLESS" );
  fflush( stdout );
  return equal;
}

bool PCCChecksum::compareRecDec() {
  bool equal = compare( checksumsRec_, checksumsDec_ );
  printf( "SEQUENCE RECONSCTRUTED AND DECODED MD5 %s \n", equal ? "OK" : "DIFF" );
  fflush( stdout );
  return equal;
}
