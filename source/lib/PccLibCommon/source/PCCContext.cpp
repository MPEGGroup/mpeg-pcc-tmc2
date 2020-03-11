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
#include "PCCFrameContext.h"
#include "PCCVideo.h"
#include "PCCContext.h"

using namespace pcc;

PCCContext::PCCContext() = default;

PCCContext::~PCCContext() { atlasContexts_.clear(); }

void PCCContext::resizeAtlas( size_t size ) {
  atlasContexts_.resize( size );
  for ( int atlIdx = 0; atlIdx < size; atlIdx++ ) { atlasContexts_[atlIdx].setAtlasIndex( atlIdx ); }
}

PCCAtlasContext::PCCAtlasContext() = default;

PCCAtlasContext::~PCCAtlasContext() {
  frameContexts_.clear();
  clearVideoFrames();
  subContexts_.clear();
  unionPatch_.clear();
}

void PCCAtlasContext::resize( size_t size, size_t frameStart ) {
  frameContexts_.resize( size );
  for ( size_t i = frameStart; i < size + frameStart; i++ ) { frameContexts_[i].setIndex( i ); }
}

void PCCAtlasContext::allocOneLayerData() {
  for ( auto& frameContext : frameContexts_ ) { frameContext.allocOneLayerData(); }
}

void PCCAtlasContext::printBlockToPatch( const size_t occupancyResolution ) {
  for ( auto& frameContext : frameContexts_ ) { frameContext.printBlockToPatch( occupancyResolution ); }
}

void PCCAtlasContext::allocateVideoFrames( PCCHighLevelSyntax& syntax, size_t numFrames ) {
  // syntax elements values
  size_t attrCount     = syntax.getVps().getAttributeInformation( atlasIndex_ ).getAttributeCount();
  size_t mapCount      = syntax.getVps().getMapCountMinus1( atlasIndex_ ) + 1;
  size_t nominalWidth  = syntax.getVps().getFrameWidth( atlasIndex_ );
  size_t nominalHeight = syntax.getVps().getFrameHeight( atlasIndex_ );
  // allocating structures for occupancy map
  size_t nominalBitDepthOccupancy =
      syntax.getVps().getOccupancyInformation( atlasIndex_ ).getOccupancyNominal2DBitdepthMinus1() + 1;
  occFrames_.resize( numFrames );
  occBitdepth_.resize( numFrames, nominalBitDepthOccupancy );
  occWidth_.resize( numFrames, nominalWidth );
  occHeight_.resize( numFrames, nominalHeight );

  // allocating structures for geometry
  size_t nominalBitDepthGeometry =
      syntax.getVps().getGeometryInformation( atlasIndex_ ).getGeometryNominal2dBitdepthMinus1() + 1;
  geoFrames_.resize( mapCount );
  geoBitdepth_.resize( mapCount );
  geoWidth_.resize( mapCount );
  geoHeight_.resize( mapCount );
  for ( size_t mapIdx = 0; mapIdx < mapCount; mapIdx++ ) {
    geoFrames_[mapIdx].resize( numFrames );
    geoBitdepth_[mapIdx].resize( numFrames, nominalBitDepthGeometry );
    geoWidth_[mapIdx].resize( numFrames, nominalWidth );
    geoHeight_[mapIdx].resize( numFrames, nominalHeight );
  }
  geoAuxFrames_.resize( numFrames );

  // allocating structures for attributes
  attrFrames_.resize( attrCount );
  attrBitdepth_.resize( attrCount );
  attrWidth_.resize( attrCount );
  attrHeight_.resize( attrCount );
  for ( size_t attrIdx = 0; attrIdx < attrCount; attrIdx++ ) {
    size_t nominalBitDepthAttribute =
        syntax.getVps().getAttributeInformation( atlasIndex_ ).getAttributeNominal2dBitdepthMinus1( attrIdx ) + 1;
    size_t partitionCount =
        syntax.getVps().getAttributeInformation( atlasIndex_ ).getAttributeDimensionPartitionsMinus1( attrIdx ) + 1;
    attrFrames_[attrIdx].resize( partitionCount );
    attrBitdepth_[attrIdx].resize( partitionCount );
    attrWidth_[attrIdx].resize( partitionCount );
    attrHeight_[attrIdx].resize( partitionCount );
    for ( size_t partIdx = 0; partIdx < partitionCount; partIdx++ ) {
      attrFrames_[attrIdx][partIdx].resize( mapCount );
      attrBitdepth_[attrIdx][partIdx].resize( mapCount );
      attrWidth_[attrIdx][partIdx].resize( mapCount );
      attrHeight_[attrIdx][partIdx].resize( mapCount );
      for ( size_t mapIdx = 0; mapIdx < syntax.getVps().getMapCountMinus1( atlasIndex_ ) + 1; mapIdx++ ) {
        attrFrames_[attrIdx][partIdx][mapIdx].resize( numFrames );
        attrBitdepth_[attrIdx][partIdx][mapIdx].resize( numFrames, nominalBitDepthAttribute );
        attrWidth_[attrIdx][partIdx][mapIdx].resize( numFrames, nominalWidth );
        attrHeight_[attrIdx][partIdx][mapIdx].resize( numFrames, nominalHeight );
      }
    }
  }
  attrAuxFrames_.resize( attrCount );
  for ( size_t attrIdx = 0; attrIdx < attrCount; attrIdx++ ) {
    size_t partitionCount =
        syntax.getVps().getAttributeInformation( atlasIndex_ ).getAttributeDimensionPartitionsMinus1( attrIdx ) + 1;
    attrAuxFrames_[attrIdx].resize( partitionCount );
    for ( size_t partIdx = 0; partIdx < partitionCount; partIdx++ ) {
      attrAuxFrames_[attrIdx][partIdx].resize( numFrames );
    }
  }
}

void PCCAtlasContext::clearVideoFrames() {
  // clearing structures for occupancy map
  occFrames_.clear();
  occBitdepth_.clear();
  occWidth_.clear();
  occHeight_.clear();

  // clearing structures for geometry
  for ( size_t mapIdx = 0; mapIdx < geoFrames_.size(); mapIdx++ ) {
    geoFrames_[mapIdx].clear();
    geoBitdepth_[mapIdx].clear();
    geoWidth_[mapIdx].clear();
    geoHeight_[mapIdx].clear();
  }
  geoFrames_.clear();
  geoBitdepth_.clear();
  geoWidth_.clear();
  geoHeight_.clear();
  geoAuxFrames_.clear();

  // clearing structures for attributes
  for ( size_t attrIdx = 0; attrIdx < attrFrames_.size(); attrIdx++ ) {
    for ( size_t partIdx = 0; partIdx < attrHeight_[attrIdx].size(); partIdx++ ) {
      for ( size_t mapIdx = 0; mapIdx < attrHeight_[attrIdx][partIdx].size(); mapIdx++ ) {
        attrFrames_[attrIdx][partIdx][mapIdx].clear();
        attrBitdepth_[attrIdx][partIdx][mapIdx].clear();
        attrWidth_[attrIdx][partIdx][mapIdx].clear();
        attrHeight_[attrIdx][partIdx][mapIdx].clear();
      }
      attrFrames_[attrIdx][partIdx].clear();
      attrBitdepth_[attrIdx][partIdx].clear();
      attrWidth_[attrIdx][partIdx].clear();
      attrHeight_[attrIdx][partIdx].clear();
    }
    attrFrames_[attrIdx].clear();
    attrBitdepth_[attrIdx].clear();
    attrWidth_[attrIdx].clear();
    attrHeight_[attrIdx].clear();
  }
  attrFrames_.clear();
  attrBitdepth_.clear();
  attrWidth_.clear();
  attrHeight_.clear();
  for (auto & attrAuxFrame : attrAuxFrames_) {
    for (auto & partIdx : attrAuxFrame) {
      partIdx.clear();
    }
    attrAuxFrame.clear();
  }
  attrAuxFrames_.clear();
}