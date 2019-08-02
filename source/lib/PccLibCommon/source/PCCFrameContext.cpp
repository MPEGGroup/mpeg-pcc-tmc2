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
#include "PCCPatch.h"
#include "PCCFrameContext.h"

using namespace pcc;

PCCFrameContext::PCCFrameContext() :
  index_( 0 ),
  numMatchedPatches_( 0 ),
  surfaceThickness_( 0 ),
  width_( 0 ),
  height_( 0 ),
  MPGeoWidth_( 0 ),
  MPGeoHeight_( 0 ),
  MPAttWidth_( 0 ),
  MPAttHeight_( 0 ),
  numberOfMissedPointsPatches_( 0 ),
  totalNumberOfMissedPoints_( 0 ),
  NumberOfEddPoints_( 0 ),
  globalPatchCount_( 0 ),
  geometry3dCoordinatesBitdepth_( 10 ),
  pointLocalReconstructionNumber_( 0 ),
  losslessGeo_( false ),
  losslessGeo444_( false ),
  useMissedPointsSeparateVideo_( false ),
  pcmPatchEnabledFlag_(false),
  geometry2dNorminalBitdepth_(8)
{
}

PCCFrameContext::~PCCFrameContext() {
  pointToPixel_.clear();
  blockToPatch_.clear();
  occupancyMap_.clear();
  fullOccupancyMap_.clear();
  patches_.clear();
  srcPointCloudByPatch_.clear();
  srcPointCloudByBlock_.clear();
  recPointCloudByBlock_.clear();
  for ( auto& block : pointToPixelByBlock_ ) { block.clear(); }
  pointToPixelByBlock_.clear();
}

void PCCFrameContext::allocOneLayerData() {
  for ( auto& patch : patches_ ) { patch.allocOneLayerData(); }
}
void PCCFrameContext::printBlockToPatch( const size_t resolution ) {
  printVector( blockToPatch_, width_ / resolution, height_ / resolution, stringFormat( "blockToPatch[%d]", index_ ),
               true );
}
void PCCFrameContext::printPatch() {
  size_t index = 0;
  printf( "Patch %4lu:", patches_.size() );
  for ( auto& patch : patches_ ) {
    printf( "  Patch[%4lu]: ", index++ );
    patch.print();
  }
  fflush( stdout );
}
void PCCFrameContext::printPatchDecoder() {
  size_t index = 0;
  printf( "Patch %4lu:", patches_.size() );
  for ( auto& patch : patches_ ) {
    printf( "  Patch[%4lu]: ", index++ );
    patch.printDecoder();
  }
  fflush( stdout );
}
