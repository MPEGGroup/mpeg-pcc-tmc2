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

PCCFrameContext::PCCFrameContext() : index_(0), width_(0), height_(0) {
}

PCCFrameContext::~PCCFrameContext(){
  pointToPixel_        .clear();
  blockToPatch_        .clear();
  occupancyMap_        .clear();
  fullOccupancyMap_    .clear();
  occupancyMap_        .clear();
  patches_             .clear();
  srcPointCloudByPatch_.clear();
  srcPointCloudByBlock_.clear();
  recPointCloudByBlock_.clear();
  for( auto& block: pointToPixelByBlock_ ){
    block.clear();
  }
  pointToPixelByBlock_.clear();
  interpolate_        .clear();
  filling_            .clear();
  minD1_              .clear();
  neighbor_           .clear();
  numMatchedPatches_  = 0;  
}

void PCCFrameContext::allocOneLayerData( const size_t occupancyResolution ){
  const size_t blockCount = ( width_ * height_ ) / ( occupancyResolution * occupancyResolution );
  interpolate_.resize( blockCount, 0 );
  filling_    .resize( blockCount, 0 );
  minD1_      .resize( blockCount, 0 );
  neighbor_   .resize( blockCount, 1 );
  std::fill( interpolate_.begin(), interpolate_.end(), 0 );
  std::fill( filling_    .begin(), filling_    .end(), 0 );
  std::fill( minD1_      .begin(), minD1_      .end(), 0 );
  std::fill( neighbor_   .begin(), neighbor_   .end(), 1 );
}


template<typename T>
void printVector( std::vector<T> data, const size_t width, const size_t height, const std::string string, const bool hexa = false ) {
  if( data.size() == 0 ) { data.resize( width * height, 0 ); }
    printf("%s: %lu %lu \n",string.c_str(), width,height);
    for (size_t v0 = 0; v0 < height; ++v0) {
      for (size_t u0 = 0; u0 < width; ++u0) {
        if( hexa ) { printf("%2x",(int)(data[ v0 * width + u0 ]) ); }
        else       { printf("%d", (int)(data[ v0 * width + u0 ]) ); }
      }
      printf("\n"); fflush(stdout);
    }
}

void PCCFrameContext::printBlockToPatch( const size_t resolution ){
  printVector( blockToPatch_, width_ / resolution, height_ / resolution, string_format( "blockToPatch[%d]", index_), true );
}
void PCCFrameContext::printInterpolate( const size_t resolution ){
  printVector( interpolate_,  width_ / resolution, height_ / resolution, string_format( "interpolate [%d]", index_) );
}
void PCCFrameContext::printNeightbor( const size_t resolution ){
  printVector( neighbor_,     width_ / resolution, height_ / resolution, string_format( "neighbor    [%d]", index_) );
}
void PCCFrameContext::printMinD1( const size_t resolution ){
  printVector( minD1_,        width_ / resolution, height_ / resolution, string_format( "minD1       [%d]", index_) );
}
void PCCFrameContext::printFilling( const size_t resolution ){
  printVector( filling_,      width_ / resolution, height_ / resolution, string_format( "filling     [%d]", index_) );
}

void PCCFrameContext::printOneLayerData( const size_t resolution ){
  printInterpolate( resolution );
  printNeightbor  ( resolution );
  printMinD1      ( resolution );
  printFilling    ( resolution );
}


void PCCFrameContext::printPatch(){
  size_t index = 0;
  printf("Patch %4lu:", patches_.size());
  for( auto& patch : patches_ ) {
    printf("  Patch[%4lu]: ",index++);
    patch.print();
  }
  fflush(stdout);
}
void PCCFrameContext::printPatchDecoder(){
  size_t index = 0;
  printf("Patch %4lu:", patches_.size());
  for( auto& patch : patches_ ) {
    printf("  Patch[%4lu]: ",index++);
    patch.printDecoder();
  }
  fflush(stdout);
}

