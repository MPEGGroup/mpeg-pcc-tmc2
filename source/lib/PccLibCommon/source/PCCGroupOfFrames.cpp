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
#include "PCCPointSet.h"
#include "PCCGroupOfFrames.h"
#include "tbb/tbb.h"

using namespace pcc;

PCCGroupOfFrames::PCCGroupOfFrames() = default;
PCCGroupOfFrames::PCCGroupOfFrames( size_t value ) { frames_.resize( value ); }
PCCGroupOfFrames::~PCCGroupOfFrames() { frames_.clear(); }

bool PCCGroupOfFrames::load( const std::string&      uncompressedDataPath,
                             const size_t            startFrameNumber,
                             const size_t            endFrameNumber,
                             const PCCColorTransform colorTransform,
                             const bool              readNormals,
                             const size_t            nbThread ) {
  if ( endFrameNumber < startFrameNumber ) { return false; }
  const size_t frameCount = endFrameNumber - startFrameNumber;
  frames_.resize( frameCount );
  tbb::task_arena limited( static_cast<int>( nbThread ) );
  limited.execute( [&] {
    tbb::parallel_for( size_t( startFrameNumber ), endFrameNumber, [&]( const size_t frameNumber ) {
      char fileName[4096];
      sprintf( fileName, uncompressedDataPath.c_str(), frameNumber );
      auto& pointSet = frames_[frameNumber - startFrameNumber];
      pointSet.resize( 0 );
      if ( !pointSet.read( fileName, readNormals ) ) {
        std::cout << "Error: can't open " << fileName << std::endl;
        frames_.resize( frameNumber - startFrameNumber );
      } else {
        if ( colorTransform == COLOR_TRANSFORM_RGB_TO_YCBCR ) { pointSet.convertRGBToYUV(); }
      }
    } );
  } );
  return ( startFrameNumber != endFrameNumber );
}

bool PCCGroupOfFrames::write( const std::string& reconstructedDataPath,
                              size_t&            frameNumber,
                              const size_t       nbThread,
                              const bool         isAscii ) {
  bool            ret = true;
  tbb::task_arena limited( static_cast<int>( nbThread ) );
  limited.execute( [&] {
    tbb::parallel_for( size_t( 0 ), frames_.size(), [&]( const size_t i ) {
      char  fileName[4096];
      auto& pointSet = frames_[i];
      sprintf( fileName, reconstructedDataPath.c_str(), frameNumber + i );
      if ( !pointSet.write( fileName, isAscii ) ) { ret = false; }
    } );
  } );
  frameNumber += frames_.size();
  return ret;
}
