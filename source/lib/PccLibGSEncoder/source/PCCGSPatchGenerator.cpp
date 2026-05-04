/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2026, ISO/IEC
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

#include "PCCGSPatchGenerator.h"
#include "PCCGSEncoderConstant.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>

using namespace pcc;

namespace {

uint64_t mortonAddr( const int32_t x, const int32_t y, const int32_t z ) {
  uint64_t answer = g_kMortonCode256X[( x >> 16 ) & 0xFF] | g_kMortonCode256Y[( y >> 16 ) & 0xFF] |
                    g_kMortonCode256Z[( z >> 16 ) & 0xFF];
  answer          = answer << 24 | g_kMortonCode256X[( x >> 8 ) & 0xFF] | g_kMortonCode256Y[( y >> 8 ) & 0xFF] |
                    g_kMortonCode256Z[( z >> 8 ) & 0xFF];
  answer = answer << 24 | g_kMortonCode256X[x & 0xFF] | g_kMortonCode256Y[y & 0xFF] | g_kMortonCode256Z[z & 0xFF];
  return answer;
}

uint64_t mortonAddr( const PCCPoint3D &vec, int depth ) {
  int x = int( vec.x() ) >> depth;
  int y = int( vec.y() ) >> depth;
  int z = int( vec.z() ) >> depth;
  return mortonAddr( x, y, z );
}

}  // namespace

PCCGSPatchResult PCCGSPatchGenerator::generate( const PCCPointSet3 &source, const PCCGSEncoderParameters &params ) {
  patches_.clear();
  mapWidth_  = 0;
  mapHeight_ = 0;

  // Generate patches in 3D space
  generate3DPatches( source, params );

  // Generate 2D positions using CTC algorithm
  generate2DPatchesCTC( source, params );

  // Return result
  PCCGSPatchResult result;
  result.patches   = std::move( patches_ );
  result.mapWidth  = mapWidth_;
  result.mapHeight = mapHeight_;
  return result;
}

void PCCGSPatchGenerator::generate3DPatches( const PCCPointSet3 &source, const PCCGSEncoderParameters &params ) {
  std::map<PCCBox3D, std::vector<size_t>> bbox2rawPoints;
  auto                                    mpsBoxSize = double( 1 << params.geometryBitDepth2D_ );
  if ( params.raw3dOffsetBitCountExplicitModeFlag_ ) {
    mpsBoxSize = double( 1 << ( params.raw3dOffsetAxisBitCountMinus1_ + 1 ) );
    assert( 0 );
  }

  PCCBox3D inputBbox = source.computeBoundingBox();
  inputBbox.min_     = { 0, 0, 0 };
  std::cout << "input boundinBox::(min_x, min_y, min_z) = (" << inputBbox.min_.x() << ", " << inputBbox.min_.y() << ", "
            << inputBbox.min_.z() << ");" << std::endl;
  std::cout << "input boundinBox::(max_x, max_y, max_z) = (" << inputBbox.max_.x() << ", " << inputBbox.max_.y() << ", "
            << inputBbox.max_.z() << ");" << std::endl;

  // Map points to 3D bounding boxes
  for ( size_t i = 0; i < source.getPointCount(); ++i ) {
    const PCCPoint3D &pt = source[i];
    PCCBox3D          bboxRawPoints;
    for ( int j = 0; j < 3; j++ ) {
      bboxRawPoints.min_[j] = std::floor( ( pt[j] - inputBbox.min_[j] ) / mpsBoxSize ) * mpsBoxSize;
      bboxRawPoints.max_[j] = bboxRawPoints.min_[j] + ( mpsBoxSize - 1 );
    }
    bbox2rawPoints[bboxRawPoints].push_back( i );
  }

  // Initiate the raw patches from the 3D bounding boxes
  for ( auto &ele : bbox2rawPoints ) {
    const PCCBox3D      &bboxRawPoints = ele.first;
    std::vector<size_t> &rawPointsBBox = ele.second;
    std::cout << "Box = ( " << bboxRawPoints.min_.x() << ", " << bboxRawPoints.min_.y() << ", "
              << bboxRawPoints.min_.z() << ") ~ (" << bboxRawPoints.max_.x() << ", " << bboxRawPoints.max_.y() << ", "
              << bboxRawPoints.max_.z() << ") " << std::endl;

    patches_.push_back( createPatchInfo( bboxRawPoints, rawPointsBBox ) );
  }
}

void PCCGSPatchGenerator::generate2DPatchesCTC( const PCCPointSet3 &source, const PCCGSEncoderParameters &params ) {
  packRawPatches2DCTC( params );
  sortPointsInPatchMorton( source );
  for ( auto &patchInfo : patches_ ) { computePointPositionsIn2D( patchInfo ); }
}

void PCCGSPatchGenerator::packRawPatches2DCTC( const PCCGSEncoderParameters &params ) {
  bool              lowDelayEncoding = false;
  size_t            safeguard        = 0;
  size_t            occupancySizeV   = 64;
  size_t            occupancySizeU   = params.minimumImageWidth_ / params.occupancyResolution_;
  auto              maxOccupancyRow  = 0;
  size_t            height           = 64;
  size_t            width            = params.minimumImageWidth_;
  std::vector<bool> occupancyMap;
  occupancyMap.resize( occupancySizeU * occupancySizeV, false );
  std::vector<int> uStart;
  uStart.resize( occupancySizeV, 0 );

  for ( auto &patchInfo : patches_ ) {
    auto &rawPointsPatch = patchInfo.patch;
    auto  rawPointsPatchBlocks =
        static_cast<size_t>( ceil( double( rawPointsPatch.getNumberOfRawPoints() * 3 ) /
                                   ( params.occupancyResolution_ * params.occupancyResolution_ ) ) );
    auto rawPointsPatchBlocksV0 = static_cast<size_t>( ceil( double( rawPointsPatchBlocks ) / occupancySizeU ) );
    auto rawPointsPatchBlocksU0 =
        static_cast<size_t>( ceil( double( rawPointsPatchBlocks ) / rawPointsPatchBlocksV0 ) );
    rawPointsPatch.sizeU0_ = rawPointsPatchBlocksU0;
    rawPointsPatch.sizeV0_ = rawPointsPatchBlocksV0;
    rawPointsPatch.sizeV_  = rawPointsPatchBlocksV0 * params.occupancyResolution_;
    rawPointsPatch.sizeU_  = rawPointsPatchBlocksU0 * params.occupancyResolution_;

    PCCPatch patch;
    patch.setSizeU0( rawPointsPatch.sizeU0_ );
    patch.setSizeV0( rawPointsPatch.sizeV0_ );
    patch.setSizeU( rawPointsPatch.sizeU_ );
    patch.setSizeV( rawPointsPatch.sizeV_ );
    assert( patch.getSizeU0() <= occupancySizeU );
    patch.allocOccupancy( rawPointsPatch.sizeU0_ * rawPointsPatch.sizeV0_, false );

    const int16_t infiniteValue = ( std::numeric_limits<int16_t>::max )();
    rawPointsPatch.resize( rawPointsPatch.sizeU_ * rawPointsPatch.sizeV_, infiniteValue );
    std::vector<bool> &rawPointsPatchOccupancy = rawPointsPatch.occupancy_;
    rawPointsPatchOccupancy.resize( rawPointsPatch.sizeU0_ * rawPointsPatch.sizeV0_, false );

    for ( size_t v = 0; v < rawPointsPatch.sizeV_; ++v ) {
      for ( size_t u = 0; u < rawPointsPatch.sizeU_; ++u ) {
        const size_t p = v * rawPointsPatch.sizeU_ + u;
        if ( p < rawPointsPatch.getNumberOfRawPoints() * 3 ) {
          const size_t u0 = u / rawPointsPatch.occupancyResolution_;
          const size_t v0 = v / rawPointsPatch.occupancyResolution_;
          const size_t p0 = v0 * rawPointsPatch.sizeU0_ + u0;
          assert( u0 < rawPointsPatch.sizeU0_ );
          assert( v0 < rawPointsPatch.sizeV0_ );
          rawPointsPatchOccupancy[p0] = true;
          patch.setOccupancy( p0, true );
        }
      }
    }

    // Place the raw points patch in the atlas
    bool locationFound = false;
    while ( !locationFound ) {
      patch.setPatchOrientation( PATCH_ORIENTATION_DEFAULT );
      for ( int v = maxOccupancyRow; v <= ( (int64_t)occupancySizeV - (int64_t)patch.getSizeV0() ) && !locationFound;
            ++v ) {
        for ( int u = uStart[v]; u <= ( (int64_t)occupancySizeU - (int64_t)patch.getSizeU0() ) && !locationFound;
              ++u ) {
          patch.setU0( u );
          patch.setV0( v );
          if ( patch.checkFitPatchCanvas( occupancyMap, occupancySizeU, occupancySizeV, lowDelayEncoding,
                                          safeguard ) ) {
            locationFound = true;
          }
        }
      }
      if ( !locationFound ) {
        occupancySizeV *= 2;
        occupancyMap.resize( occupancySizeU * occupancySizeV );
        uStart.resize( occupancySizeV );
      }
    }

    rawPointsPatch.u0_ = patch.getU0();
    rawPointsPatch.v0_ = patch.getV0();
    for ( size_t v0 = 0; v0 < rawPointsPatch.sizeV0_; ++v0 ) {
      const size_t v = rawPointsPatch.v0_ + v0;
      for ( size_t u0 = 0; u0 < rawPointsPatch.sizeU0_; ++u0 ) {
        const size_t u = rawPointsPatch.u0_ + u0;
        if ( lowDelayEncoding ) {
          occupancyMap[v * occupancySizeU + u] = true;
        } else {
          occupancyMap[v * occupancySizeU + u] =
              occupancyMap[v * occupancySizeU + u] || rawPointsPatchOccupancy[v0 * rawPointsPatch.sizeU0_ + u0];
        }
      }
      height = std::max( height, ( patch.getV0() + patch.getSizeV0() ) * params.occupancyResolution_ );
    }

    // Update uStart
    for ( int vv = patch.getV0(); vv < ( patch.getV0() + patch.getSizeV0() ); vv++ ) {
      for ( int uu = 0; uu < ( patch.getU0() + patch.getSizeU0() ); uu++ ) {
        if ( !occupancyMap[vv * occupancySizeU + uu] ) {
          uStart[vv] = uu;
          break;
        }
        if ( uu == ( patch.getU0() + patch.getSizeU0() ) - 1 ) {
          if ( uStart[vv] < uu ) uStart[vv] = uu;
        }
      }
    }

    printf( "packRawPointsPatch: posU0V0 %zu,%zu sizeU0V0(%zux%zu) #ofpixels %zu\n", rawPointsPatch.u0_,
            rawPointsPatch.v0_, rawPointsPatch.sizeU0_, rawPointsPatch.sizeV0_,
            rawPointsPatch.getNumberOfRawPoints() * 3 );
  }

  mapWidth_  = params.minimumImageWidth_;
  mapHeight_ = ( ( height + 64 - 1 ) / 64 ) * 64;
}

void PCCGSPatchGenerator::packRawPatches2DSimple( const PCCGSEncoderParameters &params ) {
  size_t posX             = 0;
  size_t posY             = 0;
  size_t currentRowHeight = 0;

  for ( auto &patchInfo : patches_ ) {
    auto rawPointsPatchBlocks = patchInfo.patch.numberOfRawPoints_ * 3;
    auto rawPointsPatchBlocksV0 =
        static_cast<size_t>( ceil( double( rawPointsPatchBlocks ) / params.minimumImageWidth_ ) );
    auto rawPointsPatchBlocksU0 =
        static_cast<size_t>( ceil( double( rawPointsPatchBlocks ) / rawPointsPatchBlocksV0 ) );
    patchInfo.patch.sizeU0_ = rawPointsPatchBlocksU0;
    patchInfo.patch.sizeU_  = patchInfo.patch.sizeU0_;
    patchInfo.patch.sizeV0_ = rawPointsPatchBlocksV0;
    patchInfo.patch.sizeV_  = patchInfo.patch.sizeV0_;

    if ( posX + patchInfo.patch.sizeU0_ > params.minimumImageWidth_ ) {
      posX = 0;
      posY += currentRowHeight;
      currentRowHeight = 0;
    }

    patchInfo.patch.u0_ = posX;
    patchInfo.patch.v0_ = posY;

    posX += patchInfo.patch.sizeU0_;
    currentRowHeight = std::max( currentRowHeight, patchInfo.patch.sizeV0_ );

    if ( posX >= params.minimumImageWidth_ ) {
      posX = 0;
      posY += currentRowHeight;
      currentRowHeight = 0;
    }

    std::cout << "Patch packing info: u0=" << patchInfo.patch.u0_ << ", v0=" << patchInfo.patch.v0_
              << ", sizeU=" << patchInfo.patch.sizeU_ << ", sizeV=" << patchInfo.patch.sizeV_
              << ", numberOfRawPoints=" << patchInfo.patch.numberOfRawPoints_ << std::endl;
  }

  mapWidth_  = params.minimumImageWidth_;
  mapHeight_ = ( posY / 64 + 1 ) * 64;
}

void PCCGSPatchGenerator::sortPointsInPatchMorton( const PCCPointSet3 &source ) {
  auto &positions = source.getPositions();

  for ( auto &patchInfo : patches_ ) {
    auto  &rawPointsPatch  = patchInfo.patch;
    auto  &rawPointsSrcIdx = patchInfo.pointsIndicesInPatch;
    size_t numRawPoints    = rawPointsPatch.getNumberOfRawPoints();

    if ( numRawPoints != 0u ) {
      PCCPointSet3 rawPointsSet;
      rawPointsSet.resize( numRawPoints );
      for ( size_t i = 0; i < numRawPoints; i++ ) {
        size_t      pointIndex = patchInfo.pointsIndicesInPatch[i];
        const auto &pos        = positions[pointIndex];
        rawPointsSet[i]        = PCCPoint3D( pos.x(), pos.y(), pos.z() );
      }

      // Calculate Morton codes
      std::vector<std::pair<uint64_t, uint64_t>> mortonPoint;
      mortonPoint.resize( numRawPoints );
      for ( size_t i = 0; i < numRawPoints; ++i ) {
        mortonPoint[i].first  = mortonAddr( rawPointsSet[i], 0 );
        mortonPoint[i].second = i;
      }

      // Sort by Morton codes
      std::sort( mortonPoint.begin(), mortonPoint.end() );

      // Re-order indices
      auto tmpRawPointsSrcIdx = rawPointsSrcIdx;
      for ( size_t i = 0; i < numRawPoints; ++i ) {
        uint64_t idx          = mortonPoint[i].second;
        tmpRawPointsSrcIdx[i] = rawPointsSrcIdx[idx];
      }
      rawPointsSrcIdx = tmpRawPointsSrcIdx;
    }
  }
}

void PCCGSPatchGenerator::computePointPositionsIn2D( GSEncoderPatchInformation &patchInfo ) {
  size_t numberOfRawPoints = patchInfo.patch.getNumberOfRawPoints();
  for ( int comp = 0; comp < 3; comp++ ) { patchInfo.pointsPositionsIn2D[comp].resize( numberOfRawPoints ); }
  for ( size_t p = 0; p < numberOfRawPoints; ++p ) {
    for ( int comp = 0; comp < 3; comp++ ) {
      size_t offset                          = comp * numberOfRawPoints;
      size_t u                               = ( p + offset ) % patchInfo.patch.sizeU0_ + patchInfo.patch.u0_;
      size_t v                               = ( p + offset ) / patchInfo.patch.sizeU0_ + patchInfo.patch.v0_;
      patchInfo.pointsPositionsIn2D[comp][p] = { u, v };
    }
  }
}

GSEncoderPatchInformation PCCGSPatchGenerator::createPatchInfo( const PCCBox3D            &bbox,
                                                                const std::vector<size_t> &pointIndices ) {
  GSEncoderPatchInformation patchInfo;
  patchInfo.patch.frameIndex_          = 0;
  patchInfo.patch.tileIndex_           = 0;
  patchInfo.patch.occupancyResolution_ = 1;
  patchInfo.patch.isPatchInAuxVideo_   = 0;
  patchInfo.patch.sizeU_               = 0;
  patchInfo.patch.sizeV_               = 0;
  patchInfo.patch.u0_                  = 0;
  patchInfo.patch.v0_                  = 0;
  patchInfo.patch.sizeV0_              = 0;
  patchInfo.patch.sizeU0_              = 0;
  patchInfo.patch.u1_                  = size_t( bbox.min_.x() );
  patchInfo.patch.v1_                  = size_t( bbox.min_.y() );
  patchInfo.patch.d1_                  = size_t( bbox.min_.z() );
  patchInfo.patch.occupancy_.resize( 0 );
  patchInfo.patch.setNumberOfRawPoints( pointIndices.size() );
  patchInfo.pointsIndicesInPatch = pointIndices;
  return patchInfo;
}
