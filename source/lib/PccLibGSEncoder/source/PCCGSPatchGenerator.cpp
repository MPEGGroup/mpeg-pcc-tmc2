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
#include "PCCKdTree.h"
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

// Recursive zig-zag scan of a BW×BH block, offset (ox, oy), result appended to out (m76580)
void zigzag( int                               BW,
             int                               BH,
             int                               ox,
             int                               oy,
             const std::vector<int>           &order,
             const std::vector<uint16_t>      &Xs,
             const std::vector<uint16_t>      &Ys,
             const std::vector<uint16_t>      &Zs,
             int                               cnt,
             std::vector<std::pair<int, int>> &out,
             bool                              recursive ) {
  int sumDiff = 0;
  int end     = std::min( cnt + BW * BH, (int)order.size() );
  for ( int i = cnt + 1; i < end; ++i ) {
    sumDiff += std::abs( (int)Xs[order[i]] - (int)Xs[order[i - 1]] );
    sumDiff += std::abs( (int)Ys[order[i]] - (int)Ys[order[i - 1]] );
    sumDiff += std::abs( (int)Zs[order[i]] - (int)Zs[order[i - 1]] );
  }
  if ( recursive &&
       /* sumDiff > BW * BH * 25 && */ BW + BH > 16 ) {  // currently no use of sumDiff, gives worse results
    if ( BW >= BH ) {
      int half = BW / 2;
      zigzag( half, BH, ox, oy, order, Xs, Ys, Zs, cnt, out, recursive );
      zigzag( half, BH, ox + half, oy, order, Xs, Ys, Zs, cnt + half * BH, out, recursive );
    } else {
      int half = BH / 2;
      zigzag( BW, half, ox, oy, order, Xs, Ys, Zs, cnt, out, recursive );
      zigzag( BW, half, ox, oy + half, order, Xs, Ys, Zs, cnt + BW * half, out, recursive );
    }
  } else {
    for ( int r = 0; r < BH; ++r )
      for ( int c = 0; c < BW; ++c ) out.push_back( { ox + c, oy + r } );
  }
}

// Greedy insertion order minimising sum|dX|+|dY|+|dZ|, mapped to zig-zag positions (m76580)
std::vector<int> greedyOrder( const std::vector<uint16_t> &Xs,
                              const std::vector<uint16_t> &Ys,
                              const std::vector<uint16_t> &Zs,
                              int                          BW,
                              int                          BH ) {
  const int n = (int)Xs.size();
  if ( n == 0 ) return {};

  std::vector<int> order;
  order.reserve( n );

  // Greedy insertion sort
  order.push_back( 0 );
  for ( int pointIdx = 1; pointIdx < n; ++pointIdx ) {
    long long totalCostX = 0, totalCostY = 0, totalCostZ = 0;
    for ( int insertPos = 1; insertPos < (int)order.size(); ++insertPos ) {
      totalCostX += std::abs( (int)Xs[order[insertPos]] - (int)Xs[order[insertPos - 1]] );
      totalCostY += std::abs( (int)Ys[order[insertPos]] - (int)Ys[order[insertPos - 1]] );
      totalCostZ += std::abs( (int)Zs[order[insertPos]] - (int)Zs[order[insertPos - 1]] );
    }
    int       bestInsertPos = (int)order.size();
    long long bestCost      = std::numeric_limits<long long>::max();
    for ( int insertPos = 0; insertPos <= (int)order.size(); ++insertPos ) {
      long long costX = totalCostX, costY = totalCostY, costZ = totalCostZ;
      if ( insertPos == 0 ) {
        costX += std::abs( (int)Xs[order[0]] - (int)Xs[pointIdx] );
        costY += std::abs( (int)Ys[order[0]] - (int)Ys[pointIdx] );
        costZ += std::abs( (int)Zs[order[0]] - (int)Zs[pointIdx] );
      } else if ( insertPos == (int)order.size() ) {
        costX += std::abs( (int)Xs[pointIdx] - (int)Xs[order.back()] );
        costY += std::abs( (int)Ys[pointIdx] - (int)Ys[order.back()] );
        costZ += std::abs( (int)Zs[pointIdx] - (int)Zs[order.back()] );
      } else {
        costX = costX - std::abs( (int)Xs[order[insertPos]] - (int)Xs[order[insertPos - 1]] ) +
                std::abs( (int)Xs[pointIdx] - (int)Xs[order[insertPos - 1]] ) +
                std::abs( (int)Xs[order[insertPos]] - (int)Xs[pointIdx] );
        costY = costY - std::abs( (int)Ys[order[insertPos]] - (int)Ys[order[insertPos - 1]] ) +
                std::abs( (int)Ys[pointIdx] - (int)Ys[order[insertPos - 1]] ) +
                std::abs( (int)Ys[order[insertPos]] - (int)Ys[pointIdx] );
        costZ = costZ - std::abs( (int)Zs[order[insertPos]] - (int)Zs[order[insertPos - 1]] ) +
                std::abs( (int)Zs[pointIdx] - (int)Zs[order[insertPos - 1]] ) +
                std::abs( (int)Zs[order[insertPos]] - (int)Zs[pointIdx] );
      }
      long long cost = costX + costY + costZ;
      if ( cost < bestCost ) {
        bestCost      = cost;
        bestInsertPos = insertPos;
      }
    }
    order.insert( order.begin() + bestInsertPos, pointIdx );
  }

  // Map greedy order to zig-zag positions within the block
  std::vector<std::pair<int, int>> zz;
  zz.reserve( static_cast<size_t>( BW ) * static_cast<size_t>( BH ) );
  zigzag( BW, BH, 0, 0, order, Xs, Ys, Zs, 0, zz, true );

  std::vector<int> result( n, -1 );
  for ( int z = 0; z < n && z < (int)zz.size(); ++z ) {
    int blockLinear = zz[z].second * BW + zz[z].first;
    if ( blockLinear < n ) result[blockLinear] = order[z];
  }
  int fill = 0;
  for ( int k = 0; k < n; ++k ) {
    if ( result[k] == -1 ) {
      while ( fill < n && std::find( result.begin(), result.end(), fill ) != result.end() ) ++fill;
      result[k] = fill++;
    }
  }
  return result;
}

}  // namespace

PCCGSPatchResult PCCGSPatchGenerator::generate( const PCCPointSet3            &source,
                                                std::vector<uint8_t>          &lodAssignment,
                                                const PCCGSEncoderParameters  &params,
                                                const PCCAdaptationParameters &adaptationParams ) {
  patches_.clear();
  lodRegionBBoxes_.clear();
  mapWidth_  = 0;
  mapHeight_ = 0;

  // Generate patches in 3D space
  generate3DPatches( source, lodAssignment, params, adaptationParams );

  // Generate 2D positions using CTC algorithm
  if ( params.rawPatchDataArrangementMethod_ == 1 ) {
    generate2DPatchesBlockBased( source, params );
  } else if ( params.rawPatchDataArrangementMethod_ == 0 ) {
    generate2DPatchesMorton( source, params );
  } else {
    std::cerr << "Unknown rawPatchDataArrangementMethod_ " << params.rawPatchDataArrangementMethod_ << "\n";
    std::exit( -1 );
  }

  // Return result
  PCCGSPatchResult result;
  result.patches   = std::move( patches_ );
  result.mapWidth  = mapWidth_;
  mapHeight_       = std::max( mapHeight_, params.minimumImageHeight_ );
  result.mapHeight = mapHeight_;
  return result;
}

void PCCGSPatchGenerator::generate3DPatches( const PCCPointSet3            &source,
                                             std::vector<uint8_t>          &lodAssignment,
                                             const PCCGSEncoderParameters  &params,
                                             const PCCAdaptationParameters &adaptationParams ) {
  // 1) raw point segmentation by AABB grid, keyed by (lodMask, AABB) so that
  //    std::map iteration order naturally groups patches by LoD mask first.
  std::map<std::pair<uint8_t, PCCBox3D>, std::vector<size_t>> bbox2rawPoints;
  auto                                                        mpsBoxSize = double( 1 << params.geometryBitDepth2D_ );

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
    // Look up the LoD mask for this source point (0 when LoD splitting is not active).
    const uint8_t lodMask = ( !lodAssignment.empty() ) ? lodAssignment[i] : uint8_t( 0 );
    bbox2rawPoints[{ lodMask, bboxRawPoints }].push_back( i );
  }

  // Initiate the raw patches from the 3D bounding boxes
  // 2) create raw patches — one per (lodMask, AABB) bucket.
  //    Because std::map orders by key, all patches for lodMask=0x01 come first,
  //    then 0x02, then 0x04, etc., giving naturally ordered LoD groups.
  for ( auto &ele : bbox2rawPoints ) {
    const uint8_t        patchLodMask  = ele.first.first;
    const PCCBox3D      &bboxRawPoints = ele.first.second;
    std::vector<size_t> &rawPointsBBox = ele.second;
    std::cout << "Box = ( " << bboxRawPoints.min_.x() << ", " << bboxRawPoints.min_.y() << ", "
              << bboxRawPoints.min_.z() << ") ~ (" << bboxRawPoints.max_.x() << ", " << bboxRawPoints.max_.y() << ", "
              << bboxRawPoints.max_.z() << ") " << "lodMask=" << static_cast<int>( patchLodMask ) << std::endl;

    patches_.push_back( createPatchInfo( bboxRawPoints, patchLodMask, rawPointsBBox ) );
  }

  // merge patch
  if ( params.raw3dOffsetAxisBitCountMinus1_ > 0 ) {
    std::cout << "Raw patch count before merging: " << patches_.size() << std::endl;
    uint32_t RawShift        = ( params.geometryBitDepth3D_ + 1 - 1 ) - ( params.raw3dOffsetAxisBitCountMinus1_ );
    uint32_t coordResolution = 1 << RawShift;
    bool     mergeSuccess    = false;
    if ( coordResolution < mpsBoxSize ) {
      mergeSuccess = mergePatchesByAABB( source, patches_, mpsBoxSize, coordResolution );
    }
    std::cout << "Raw patch count after merging: " << patches_.size() << std::endl;
  }

  // adaptive quantization
  if ( adaptationParams.enableAdaptiveShQuantization_ ) {
    patches_ = splitPatchesAdaptiveQuantization( source, patches_ );
    std::cout << "Raw patch count after split for adaptive quantization: " << patches_.size() << std::endl;
  }

  // set local coordinate of points of patch after merge
  for ( auto &rawPatch : patches_ ) {
    const size_t numRawPointsBBox = rawPatch.patch.getNumberOfRawPoints();
    rawPatch.patch.resize( 3 * numRawPointsBBox );  // allocation of xyz coord array
    for ( auto i = 0; i < numRawPointsBBox; ++i ) {
      const PCCPoint3D rawPoints                  = source[rawPatch.pointsIndicesInPatch[i]];
      rawPatch.patch.x_[i]                        = static_cast<uint16_t>( rawPoints.x() - rawPatch.patch.u1_ );
      rawPatch.patch.x_[numRawPointsBBox + i]     = static_cast<uint16_t>( rawPoints.y() - rawPatch.patch.v1_ );
      rawPatch.patch.x_[2 * numRawPointsBBox + i] = static_cast<uint16_t>( rawPoints.z() - rawPatch.patch.d1_ );
    }
  }
}

void PCCGSPatchGenerator::generate2DPatchesMorton( const PCCPointSet3 &source, const PCCGSEncoderParameters &params ) {
  packRawPatches2DMorton( params );
  sortPointsInPatchMorton( source );
  for ( auto &patchInfo : patches_ ) { computePointPositionsIn2DMorton( patchInfo, params ); }
}

void PCCGSPatchGenerator::generate2DPatchesBlockBased( const PCCPointSet3           &source,
                                                       const PCCGSEncoderParameters &params ) {
  // m76580: pre-sort patches descending by number of raw points
  std::stable_sort( patches_.begin(), patches_.end(),
                    []( const GSEncoderPatchInformation &a, const GSEncoderPatchInformation &b ) {
                      return a.patch.getNumberOfRawPoints() > b.patch.getNumberOfRawPoints();
                    } );
  std::vector<GSEncoderPatchInformation> BlockPatches, MortonPatches;

  for ( auto &patch : patches_ ) {
    if ( patch.patch.getNumberOfRawPoints() < 64 ) {
      MortonPatches.push_back( patch );
    } else {
      BlockPatches.push_back( patch );
    }
  }

  patches_ = std::move( BlockPatches );
  AtlasOccupancyState atlasState;
  packRawPatches2DBlockBased( params, &atlasState );
  sortPointsInPatchBlockBased( source, params );

  for ( auto &patchInfo : patches_ ) { computePointPositionsIn2DBlockBased( patchInfo, params ); }
  BlockPatches = std::move( patches_ );

  patches_ = std::move( MortonPatches );
  packRawPatches2DMorton( params, &atlasState );
  sortPointsInPatchMorton( source );

  // 3-band layout for Morton patches in mixed mode (matches rawPatchDataArrangementMethod=1)
  for ( auto &patchInfo : patches_ ) { computePointPositionsIn2DBlockBased( patchInfo, params ); }
  patches_.insert( patches_.begin(), BlockPatches.begin(), BlockPatches.end() );
}

void PCCGSPatchGenerator::packRawPatches2DMorton( const PCCGSEncoderParameters &params, AtlasOccupancyState *state ) {
  bool   lowDelayEncoding = false;
  size_t safeguard        = 0;
  size_t width            = params.minimumImageWidth_;

  size_t            occupancySizeU;
  size_t            occupancySizeV;
  int               maxOccupancyRow;
  size_t            height;
  std::vector<bool> occupancyMap;
  std::vector<int>  uStart;

  if ( state != nullptr ) {
    occupancyMap    = std::move( state->occupancyMap );
    uStart          = std::move( state->uStart );
    occupancySizeU  = state->occupancySizeU;
    occupancySizeV  = state->occupancySizeV;
    maxOccupancyRow = state->maxOccupancyRow;
    height          = state->height;
  } else {
    occupancySizeU  = params.minimumImageWidth_ / params.occupancyResolution_;
    occupancySizeV  = 64;
    maxOccupancyRow = 0;
    height          = 64;
    occupancyMap.resize( occupancySizeU * occupancySizeV, false );
    uStart.resize( occupancySizeV, 0 );
  }

  // LoD: LoD row-break tracking: ensures each distinct LoD mask group starts on a
  //  fresh block row, so their atlas bounding boxes are non-overlapping bands.
  uint8_t prevLodMask = std::numeric_limits<uint8_t>::max();  // sentinel — no previous patch yet
  int     currentRow  = static_cast<int>( maxOccupancyRow );  // highest occupied block row so far

  for ( auto &patchInfo : patches_ ) {
    auto &rawPointsPatch = patchInfo.patch;

    const size_t N  = rawPointsPatch.getNumberOfRawPoints();
    const size_t oR = params.occupancyResolution_;

    if ( state != nullptr ) {
      // 3-band layout (arrangementMethod=1): ceil(N/U) rows per band × 3 bands
      size_t bandBlocks   = static_cast<size_t>( ceil( double( N ) / ( oR * oR ) ) );
      size_t bandBlocksV0 = static_cast<size_t>( ceil( double( bandBlocks ) / occupancySizeU ) );
      if ( bandBlocksV0 == 0 ) bandBlocksV0 = 1;
      size_t bandBlocksU0 = static_cast<size_t>( ceil( double( bandBlocks ) / bandBlocksV0 ) );

      rawPointsPatch.sizeU0_ = bandBlocksU0;
      rawPointsPatch.sizeV0_ = bandBlocksV0 * 3;
      rawPointsPatch.sizeU_  = bandBlocksU0 * oR;
      rawPointsPatch.sizeV_  = bandBlocksV0 * 3 * oR;
    } else {
      // sequential layout (arrangementMethod=0): N*3 pixels in one strip
      size_t rawBlocks   = static_cast<size_t>( ceil( double( N * 3 ) / ( oR * oR ) ) );
      size_t rawBlocksV0 = static_cast<size_t>( ceil( double( rawBlocks ) / occupancySizeU ) );
      if ( rawBlocksV0 == 0 ) rawBlocksV0 = 1;
      size_t rawBlocksU0 = static_cast<size_t>( ceil( double( rawBlocks ) / rawBlocksV0 ) );

      rawPointsPatch.sizeU0_ = rawBlocksU0;
      rawPointsPatch.sizeV0_ = rawBlocksV0;
      rawPointsPatch.sizeU_  = rawBlocksU0 * oR;
      rawPointsPatch.sizeV_  = rawBlocksV0 * oR;

      // LoD:When the LoD mask changes between consecutive patches, force the packer
      // to start searching below the current bottom edge of the previous group.
      if ( rawPointsPatch.getLodMask() != prevLodMask && prevLodMask != std::numeric_limits<uint8_t>::max() ) {
        maxOccupancyRow = currentRow;  // new search floor = bottom of previous group
      }
      prevLodMask = rawPointsPatch.getLodMask();
    }

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
        bool occupied = false;
        if ( state != nullptr ) {
          // 3-band: each band holds N pixels
          const size_t bandHeight = rawPointsPatch.sizeV_ / 3;
          const size_t localV     = v % bandHeight;
          occupied                = ( localV * rawPointsPatch.sizeU_ + u < N );
        } else {
          // sequential: N*3 pixels in row-major order
          occupied = ( v * rawPointsPatch.sizeU_ + u < N * 3 );
        }
        if ( occupied ) {
          const size_t u0             = u / oR;
          const size_t v0             = v / oR;
          const size_t p0             = v0 * rawPointsPatch.sizeU0_ + u0;
          rawPointsPatchOccupancy[p0] = true;
          patch.setOccupancy( p0, true );
        }
      }
    }

    // Place the raw points patch in the atlas
    int  B_place       = 1;
    bool locationFound = false;
    while ( !locationFound ) {
      patch.setPatchOrientation( PATCH_ORIENTATION_DEFAULT );
      for ( int v = ( maxOccupancyRow / B_place ) * B_place;
            v <= ( (int64_t)occupancySizeV - (int64_t)patch.getSizeV0() ) && !locationFound; v += B_place ) {
        for ( int u = ( uStart[v] / B_place ) * B_place;
              u <= ( (int64_t)occupancySizeU - (int64_t)patch.getSizeU0() ) && !locationFound; u += B_place ) {
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

    // LoD: Track the highest occupied block row across all patches in this LoD group.
    currentRow = static_cast<int>( std::max( static_cast<size_t>( currentRow ), patch.getV0() + patch.getSizeV0() ) );
    // Accumulate/expand the atlas bounding box for this LoD mask (used later to fill region coords).
    if ( rawPointsPatch.getLodMask() != 0 ) {
      auto &bb = lodRegionBBoxes_[rawPointsPatch.getLodMask()];
      if ( bb[2] == 0 && bb[3] == 0 ) {  // first insert: initialise
        bb = { rawPointsPatch.u0_, rawPointsPatch.v0_, rawPointsPatch.u0_ + rawPointsPatch.sizeU0_,
               rawPointsPatch.v0_ + rawPointsPatch.sizeV0_ };
      } else {
        bb[0] = std::min( bb[0], rawPointsPatch.u0_ );
        bb[1] = std::min( bb[1], rawPointsPatch.v0_ );
        bb[2] = std::max( bb[2], rawPointsPatch.u0_ + rawPointsPatch.sizeU0_ );
        bb[3] = std::max( bb[3], rawPointsPatch.v0_ + rawPointsPatch.sizeV0_ );
      }
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

    printf( "packRawPointsPatch M: posU0V0 %zu,%zu sizeU0V0(%zux%zu) #ofpixels %zu\n", rawPointsPatch.u0_,
            rawPointsPatch.v0_, rawPointsPatch.sizeU0_, rawPointsPatch.sizeV0_,
            rawPointsPatch.getNumberOfRawPoints() * 3 );
  }

  mapWidth_  = params.minimumImageWidth_;
  mapHeight_ = ( ( height + 64 - 1 ) / 64 ) * 64;
}

void PCCGSPatchGenerator::packRawPatches2DBlockBased( const PCCGSEncoderParameters &params,
                                                      AtlasOccupancyState          *state ) {
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

    // m76580: replicate old encoder dimension computation exactly
    const size_t N             = rawPointsPatch.getNumberOfRawPoints();
    const size_t oR            = params.occupancyResolution_;
    size_t       blocksPerBand = static_cast<size_t>( std::ceil( double( N ) / ( oR * oR ) ) );
    size_t       occSizeU      = params.minimumImageWidth_ / oR;

    size_t B2 = 8;
    while ( blocksPerBand < B2 * B2 ) {
      B2 /= 2;
      if ( B2 == 1 ) break;
    }
    int B_place = static_cast<int>( B2 );

    size_t nV_band = static_cast<size_t>( std::ceil( double( blocksPerBand ) / occSizeU ) );
    if ( nV_band % B2 != 0 ) { nV_band += B2 - ( nV_band % B2 ); }
    if ( nV_band == 0 ) nV_band = B2;

    size_t nV_blocks = nV_band * 3;
    size_t nU_blocks = static_cast<size_t>( std::ceil( double( blocksPerBand ) / nV_band ) );
    size_t sU        = nU_blocks * oR;
    sU--;
    sU /= B2;
    sU++;
    sU *= B2;

    rawPointsPatch.sizeU_  = sU;
    rawPointsPatch.sizeU0_ = sU;
    rawPointsPatch.sizeV_  = nV_blocks * oR;
    rawPointsPatch.sizeV0_ = nV_blocks * oR;

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

    // m76580: mark occupancy per pixel (band-based)
    const size_t bandHeight = rawPointsPatch.sizeV_ / 3;
    for ( size_t v = 0; v < rawPointsPatch.sizeV_; ++v ) {
      for ( size_t u = 0; u < rawPointsPatch.sizeU_; ++u ) {
        bool occupied = false;
        if ( v < bandHeight ) {
          if ( v * rawPointsPatch.sizeU_ + u < N ) occupied = true;
        } else if ( v < 2 * bandHeight ) {
          if ( ( v - bandHeight ) * rawPointsPatch.sizeU_ + u < N ) occupied = true;
        } else {
          if ( ( v - 2 * bandHeight ) * rawPointsPatch.sizeU_ + u < N ) occupied = true;
        }
        if ( occupied ) {
          const size_t u0             = u / rawPointsPatch.occupancyResolution_;
          const size_t v0             = v / rawPointsPatch.occupancyResolution_;
          const size_t p0             = v0 * rawPointsPatch.sizeU0_ + u0;
          rawPointsPatchOccupancy[p0] = true;
          patch.setOccupancy( p0, true );
        }
      }
    }

    // Place the raw points patch in the atlas
    bool locationFound = false;
    while ( !locationFound ) {
      patch.setPatchOrientation( PATCH_ORIENTATION_DEFAULT );
      for ( int v = 0; v <= ( (int64_t)occupancySizeV - (int64_t)patch.getSizeV0() ) && !locationFound; v += B_place ) {
        for ( int u = ( uStart[v] / B_place ) * B_place;
              u <= ( (int64_t)occupancySizeU - (int64_t)patch.getSizeU0() ) && !locationFound; u += B_place ) {
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

    maxOccupancyRow = std::max( maxOccupancyRow, int( patch.getV0() + patch.getSizeV0() ) );

    printf( "packRawPointsPatch B: posU0V0 %zu,%zu sizeU0V0(%zux%zu) #ofpixels %zu\n", rawPointsPatch.u0_,
            rawPointsPatch.v0_, rawPointsPatch.sizeU0_, rawPointsPatch.sizeV0_,
            rawPointsPatch.getNumberOfRawPoints() * 3 );
  }

  mapWidth_  = params.minimumImageWidth_;
  mapHeight_ = ( ( height + 64 - 1 ) / 64 ) * 64;

  if ( state != nullptr ) {
    state->occupancyMap    = std::move( occupancyMap );
    state->uStart          = std::move( uStart );
    state->occupancySizeU  = occupancySizeU;
    state->occupancySizeV  = occupancySizeV;
    state->maxOccupancyRow = maxOccupancyRow;
    state->height          = height;
  }
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

void PCCGSPatchGenerator::computePointPositionsIn2DMorton( GSEncoderPatchInformation    &patchInfo,
                                                           const PCCGSEncoderParameters &params ) {
  // sequential layout (arrangementMethod=0): X pixels, then Y pixels, then Z pixels
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

void PCCGSPatchGenerator::computePointPositionsIn2DBlockBased( GSEncoderPatchInformation    &patchInfo,
                                                               const PCCGSEncoderParameters &params ) {
  // m76580: band-based layout — comp 0 in top band, comp 1 in middle band, comp 2 in bottom band
  size_t       numberOfRawPoints = patchInfo.patch.getNumberOfRawPoints();
  const size_t sU                = patchInfo.patch.sizeU_;
  const size_t bandH             = patchInfo.patch.sizeV_ / 3;
  const size_t u0                = patchInfo.patch.u0_;
  const size_t v0                = patchInfo.patch.v0_;
  for ( int comp = 0; comp < 3; comp++ ) { patchInfo.pointsPositionsIn2D[comp].resize( numberOfRawPoints ); }
  size_t pu = 0, pv = 0;
  for ( size_t p = 0; p < numberOfRawPoints; ++p ) {
    patchInfo.pointsPositionsIn2D[0][p] = { u0 + pu, v0 + pv };
    patchInfo.pointsPositionsIn2D[1][p] = { u0 + pu, v0 + bandH + pv };
    patchInfo.pointsPositionsIn2D[2][p] = { u0 + pu, v0 + 2 * bandH + pv };
    if ( ++pu == sU ) {
      pu = 0;
      ++pv;
    }
  }
}

void PCCGSPatchGenerator::sortPointsInPatchBlockBased( const PCCPointSet3           &source,
                                                       const PCCGSEncoderParameters &params ) {
  auto &positions = source.getPositions();
  for ( auto &patchInfo : patches_ ) {
    auto  &rawPointsPatch  = patchInfo.patch;
    auto  &rawPointsSrcIdx = patchInfo.pointsIndicesInPatch;
    size_t numRawPoints    = rawPointsPatch.getNumberOfRawPoints();
    if ( numRawPoints == 0u ) continue;

    const int    B  = static_cast<int>( params.blockBasedSortBlockSize_ );
    const size_t sU = rawPointsPatch.sizeU_;  // already computed by packRawPatches2DCTC (old encoder formula)
    const size_t sV = rawPointsPatch.sizeV_;  // full height (all 3 bands); p < numRawPoints filters band 0 only

    // Copy XYZ coordinates relative to patch bbox (same as old encoder: pts_X[i] = x_[i] = pos.x() - u1_)
    const auto            u1 = static_cast<double>( rawPointsPatch.u1_ );
    const auto            v1 = static_cast<double>( rawPointsPatch.v1_ );
    const auto            d1 = static_cast<double>( rawPointsPatch.d1_ );
    std::vector<uint16_t> pts_X( numRawPoints ), pts_Y( numRawPoints ), pts_Z( numRawPoints );
    for ( size_t i = 0; i < numRawPoints; ++i ) {
      const auto &pos = positions[rawPointsSrcIdx[i]];
      pts_X[i]        = static_cast<uint16_t>( pos.x() - u1 );
      pts_Y[i]        = static_cast<uint16_t>( pos.y() - v1 );
      pts_Z[i]        = static_cast<uint16_t>( pos.z() - d1 );
    }

    // Global greedy nearest-neighbour chain starting from the point with minimum Morton code
    std::vector<size_t> globalOrder;
    globalOrder.reserve( numRawPoints );
    {
      size_t   startIdx  = 0;
      uint64_t minMorton = UINT64_MAX;
      for ( size_t i = 0; i < numRawPoints; ++i ) {
        uint64_t m = mortonAddr( PCCPoint3D( pts_X[i], pts_Y[i], pts_Z[i] ), 0 );
        if ( m < minMorton ) {
          minMorton = m;
          startIdx  = i;
        }
      }
      PCCPointSet3 rawPointsSet;
      rawPointsSet.resize( numRawPoints );
      for ( size_t i = 0; i < numRawPoints; ++i ) rawPointsSet[i] = PCCPoint3D( pts_X[i], pts_Y[i], pts_Z[i] );
      PCCKdTree kdtree( rawPointsSet );

      std::vector<bool> visited( numRawPoints, false );
      visited[startIdx] = true;
      globalOrder.push_back( startIdx );

      const size_t kMaxKdQuery = 128;
      for ( size_t step = 1; step < numRawPoints; ++step ) {
        size_t     cur   = globalOrder.back();
        PCCPoint3D curPt = rawPointsSet[cur];
        bool       found = false;
        for ( size_t k = 2; k <= kMaxKdQuery && !found; k *= 2 ) {
          size_t      kQuery = std::min( k, numRawPoints );
          PCCNNResult result;
          result.resize( kQuery );
          kdtree.search( curPt, kQuery, result );
          for ( size_t j = 0; j < result.size(); ++j ) {
            size_t idx = result.indices( j );
            if ( !visited[idx] ) {
              visited[idx] = true;
              globalOrder.push_back( idx );
              found = true;
              break;
            }
          }
        }
        if ( !found ) {
          // KD-tree exhausted — fall back to brute-force search
          double cx       = static_cast<double>( curPt[0] );
          double cy       = static_cast<double>( curPt[1] );
          double cz       = static_cast<double>( curPt[2] );
          double bestDist = std::numeric_limits<double>::max();
          size_t bestIdx  = 0;
          for ( size_t j = 0; j < numRawPoints; ++j ) {
            if ( visited[j] ) continue;
            double dx = cx - static_cast<double>( rawPointsSet[j][0] );
            double dy = cy - static_cast<double>( rawPointsSet[j][1] );
            double dz = cz - static_cast<double>( rawPointsSet[j][2] );
            if ( dx * dx + dy * dy + dz * dz < bestDist ) {
              bestDist = dx * dx + dy * dy + dz * dz;
              bestIdx  = j;
            }
          }
          visited[bestIdx] = true;
          globalOrder.push_back( bestIdx );
        }
      }
    }

    // Build atlas permutation: assign NN-chain points to B×B blocks in raster order
    std::vector<int> perm( numRawPoints, -1 );
    {
      size_t globalIdx = 0;
      for ( size_t h = 0; h < sV && globalIdx < numRawPoints; h += static_cast<size_t>( B ) ) {
        for ( size_t w = 0; w < sU && globalIdx < numRawPoints; w += static_cast<size_t>( B ) ) {
          // Collect atlas positions belonging to this block
          std::vector<size_t> blockAtlasPos;
          for ( size_t hh = 0; hh < static_cast<size_t>( B ) && h + hh < sV; ++hh )
            for ( size_t ww = 0; ww < static_cast<size_t>( B ) && w + ww < sU; ++ww ) {
              size_t p = ( h + hh ) * sU + ( w + ww );
              if ( p < numRawPoints ) blockAtlasPos.push_back( p );
            }
          if ( blockAtlasPos.empty() ) continue;

          int                   cnt = static_cast<int>( blockAtlasPos.size() );
          std::vector<uint16_t> bX( cnt ), bY( cnt ), bZ( cnt );
          for ( int k = 0; k < cnt; ++k ) {
            size_t gk   = std::min( globalIdx + static_cast<size_t>( k ), numRawPoints - 1 );
            size_t orig = globalOrder[gk];
            bX[k]       = pts_X[orig];
            bY[k]       = pts_Y[orig];
            bZ[k]       = pts_Z[orig];
          }

          int  bW    = static_cast<int>( std::min( static_cast<size_t>( B ), sU - w ) );
          int  bH    = static_cast<int>( std::min( static_cast<size_t>( B ), sV - h ) );
          auto order = greedyOrder( bX, bY, bZ, bW, bH );

          for ( int k = 0; k < cnt; ++k )
            perm[blockAtlasPos[k]] = static_cast<int>( globalIdx + static_cast<size_t>( order[k] ) );
          globalIdx += static_cast<size_t>( cnt );
        }
      }
      // Fallback: fill any still-unassigned positions
      size_t fallback = 0;
      for ( size_t p = 0; p < numRawPoints; ++p )
        if ( perm[p] < 0 ) perm[p] = static_cast<int>( std::min( fallback++, numRawPoints - 1 ) );
    }

    // Reorder pointsIndicesInPatch according to the permutation
    auto tmpSrcIdx = rawPointsSrcIdx;
    for ( size_t p = 0; p < numRawPoints; ++p )
      tmpSrcIdx[p] = rawPointsSrcIdx[globalOrder[static_cast<size_t>( perm[p] )]];
    rawPointsSrcIdx = tmpSrcIdx;
  }
}

GSEncoderPatchInformation PCCGSPatchGenerator::createPatchInfo( const PCCBox3D            &bbox,
                                                                const uint8_t              lodMask,
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
  patchInfo.patch.setLodMask( lodMask );
  patchInfo.pointsIndicesInPatch = pointIndices;
  return patchInfo;
}

static void findMinMaxInPatch( const PCCPointSet3 &source, GSEncoderPatchInformation &rawPatch ) {
  uint32_t min_x = std::numeric_limits<uint32_t>::max();
  uint32_t max_x = 0;
  uint32_t min_y = std::numeric_limits<uint32_t>::max();
  uint32_t max_y = 0;
  uint32_t min_z = std::numeric_limits<uint32_t>::max();
  uint32_t max_z = 0;

  auto update_min_max = [&]( uint32_t x, uint32_t y, uint32_t z ) {
    min_x = std::min( min_x, x );
    max_x = std::max( max_x, x );
    min_y = std::min( min_y, y );
    max_y = std::max( max_y, y );
    min_z = std::min( min_z, z );
    max_z = std::max( max_z, z );
  };

  for ( uint32_t i = 0; i < rawPatch.patch.getNumberOfRawPoints(); i++ ) {
    const PCCPoint3D &rawPoint = source[rawPatch.pointsIndicesInPatch[i]];
    update_min_max( rawPoint.x(), rawPoint.y(), rawPoint.z() );
  }
  rawPatch.m_mins[0] = min_x;
  rawPatch.m_mins[1] = min_y;
  rawPatch.m_mins[2] = min_z;
  rawPatch.m_maxs[0] = max_x;
  rawPatch.m_maxs[1] = max_y;
  rawPatch.m_maxs[2] = max_z;
}

static void findMinMaxIn2Patches( GSEncoderPatchInformation       &newPatch,
                                  const GSEncoderPatchInformation &rawPatch1,
                                  const GSEncoderPatchInformation &rawPatch2 ) {
  uint32_t min_x = ( rawPatch1.m_mins[0] < rawPatch2.m_mins[0] ) ? rawPatch1.m_mins[0] : rawPatch2.m_mins[0];
  uint32_t min_y = ( rawPatch1.m_mins[1] < rawPatch2.m_mins[1] ) ? rawPatch1.m_mins[1] : rawPatch2.m_mins[1];
  uint32_t min_z = ( rawPatch1.m_mins[2] < rawPatch2.m_mins[2] ) ? rawPatch1.m_mins[2] : rawPatch2.m_mins[2];

  uint32_t max_x = ( rawPatch1.m_maxs[0] > rawPatch2.m_maxs[0] ) ? rawPatch1.m_maxs[0] : rawPatch2.m_maxs[0];
  uint32_t max_y = ( rawPatch1.m_maxs[1] > rawPatch2.m_maxs[1] ) ? rawPatch1.m_maxs[1] : rawPatch2.m_maxs[1];
  uint32_t max_z = ( rawPatch1.m_maxs[2] > rawPatch2.m_maxs[2] ) ? rawPatch1.m_maxs[2] : rawPatch2.m_maxs[2];

  newPatch.m_mins[0] = min_x;
  newPatch.m_mins[1] = min_y;
  newPatch.m_mins[2] = min_z;
  newPatch.m_maxs[0] = max_x;
  newPatch.m_maxs[1] = max_y;
  newPatch.m_maxs[2] = max_z;
}

static bool isMergeFeasible( const GSEncoderPatchInformation &rawPatch1,
                             const GSEncoderPatchInformation &rawPatch2,
                             uint32_t                         boxSize,
                             uint32_t                         coordResolution,
                             PCCPoint3D                      &boxCoord /*[out]*/ ) {
  uint32_t min_x = ( rawPatch1.m_mins[0] < rawPatch2.m_mins[0] ) ? rawPatch1.m_mins[0] : rawPatch2.m_mins[0];
  uint32_t min_y = ( rawPatch1.m_mins[1] < rawPatch2.m_mins[1] ) ? rawPatch1.m_mins[1] : rawPatch2.m_mins[1];
  uint32_t min_z = ( rawPatch1.m_mins[2] < rawPatch2.m_mins[2] ) ? rawPatch1.m_mins[2] : rawPatch2.m_mins[2];

  uint32_t max_x = ( rawPatch1.m_maxs[0] > rawPatch2.m_maxs[0] ) ? rawPatch1.m_maxs[0] : rawPatch2.m_maxs[0];
  uint32_t max_y = ( rawPatch1.m_maxs[1] > rawPatch2.m_maxs[1] ) ? rawPatch1.m_maxs[1] : rawPatch2.m_maxs[1];
  uint32_t max_z = ( rawPatch1.m_maxs[2] > rawPatch2.m_maxs[2] ) ? rawPatch1.m_maxs[2] : rawPatch2.m_maxs[2];

  // shift min_x/y/z by 'coordPrecision' units
  const uint32_t coordResolutionM1 = coordResolution - 1;
  min_x                            = ( min_x & coordResolutionM1 ) ? ( min_x - ( min_x & coordResolutionM1 ) ) : min_x;
  min_y                            = ( min_y & coordResolutionM1 ) ? ( min_y - ( min_y & coordResolutionM1 ) ) : min_y;
  min_z                            = ( min_z & coordResolutionM1 ) ? ( min_z - ( min_z & coordResolutionM1 ) ) : min_z;

  boxCoord.x() = min_x;
  boxCoord.y() = min_y;
  boxCoord.z() = min_z;

  // Calculate the width, height, and depth of AABB
  size_t width  = (size_t)max_x - (size_t)min_x;
  size_t height = (size_t)max_y - (size_t)min_y;
  size_t depth  = (size_t)max_z - (size_t)min_z;

  // All dimensions of the AABB must be less than or equal to L(boxSize).
  return ( width < boxSize ) && ( height < boxSize ) && ( depth < boxSize ) &&
         rawPatch1.patch.getLodMask() == rawPatch2.patch.getLodMask();
}

static void mergeTwoPatches( const GSEncoderPatchInformation &p1,
                             const GSEncoderPatchInformation &p2,
                             uint32_t                         boxSize,
                             PCCPoint3D                      &newBoxCoord,
                             GSEncoderPatchInformation       &mergedPatch /*[out]*/ ) {
  mergedPatch.patch.frameIndex_          = p1.patch.frameIndex_;
  mergedPatch.patch.occupancyResolution_ = p1.patch.occupancyResolution_;
  mergedPatch.patch.isPatchInAuxVideo_   = p1.patch.isPatchInAuxVideo_;
  mergedPatch.patch.sizeU_               = 0;
  mergedPatch.patch.sizeV_               = 0;
  mergedPatch.patch.u0_                  = 0;
  mergedPatch.patch.v0_                  = 0;
  mergedPatch.patch.sizeV0_              = 0;
  mergedPatch.patch.sizeU0_              = 0;

  // 1) merge point indices
  mergedPatch.pointsIndicesInPatch.reserve( p1.patch.numberOfRawPoints_ + p2.patch.numberOfRawPoints_ );

  mergedPatch.pointsIndicesInPatch.insert( mergedPatch.pointsIndicesInPatch.end(), p1.pointsIndicesInPatch.begin(),
                                           p1.pointsIndicesInPatch.end() );
  mergedPatch.pointsIndicesInPatch.insert( mergedPatch.pointsIndicesInPatch.end(), p2.pointsIndicesInPatch.begin(),
                                           p2.pointsIndicesInPatch.end() );

  // 2) set AABB coordinates from merged patch
  mergedPatch.patch.u1_ = size_t( newBoxCoord.x() );
  mergedPatch.patch.v1_ = size_t( newBoxCoord.y() );
  mergedPatch.patch.d1_ = size_t( newBoxCoord.z() );
  mergedPatch.patch.occupancy_.resize( 0 );
  mergedPatch.patch.setNumberOfRawPoints( mergedPatch.pointsIndicesInPatch.size() );

  mergedPatch.patch.setLodMask( p1.patch.getLodMask() );

  // 3) find new min/max
  findMinMaxIn2Patches( mergedPatch, p1, p2 );
}

uint32_t PCCGSPatchGenerator::mergePatchesByAABB( const PCCPointSet3                     &source,
                                                  std::vector<GSEncoderPatchInformation> &RawPatches,
                                                  uint32_t                                patchSize,
                                                  uint32_t                                coordResolution ) {
  bool     wasMerged             = true;
  int      mergePass             = 0;
  uint32_t numMergedPatches      = 0;
  uint32_t totalNumMergedPatches = 0;

  // 1) find min/max position of each patch
  for ( auto &rawPatch : RawPatches ) { findMinMaxInPatch( source, rawPatch ); }

  std::vector<GSEncoderPatchInformation> currPatches = RawPatches;
  std::vector<bool>                      mergedFlags( currPatches.size(), false );

  std::vector<GSEncoderPatchInformation> nextPatches;

  // 2) Iteration until merge is not possible
  while ( wasMerged && ( mergePass < 2 ) ) {
    wasMerged = false;
    mergePass++;
    mergedFlags.clear();
    mergedFlags.resize( currPatches.size(), false );
    nextPatches.clear();
    nextPatches.reserve( currPatches.size() );

    uint32_t numMergedPatches = 0;
    std::cout << "--- Merge trial begin " << mergePass << " ---" << std::endl;

    for ( size_t i = 0; i < currPatches.size(); ++i ) {
      if ( mergedFlags[i] ) continue;

      GSEncoderPatchInformation &P_A    = currPatches[i];
      size_t                     best_j = (size_t)-1;
      PCCPoint3D                 newBoxCoord;

      for ( size_t j = i + 1; j < currPatches.size(); ++j ) {
        if ( mergedFlags[j] ) continue;

        const GSEncoderPatchInformation &P_B = currPatches[j];

        // Integration Probability Check
        if ( isMergeFeasible( P_A, P_B, patchSize, coordResolution, newBoxCoord /*[out]*/ ) ) {
          // P_B can be integrated with P_A.
          best_j = j;
          break;  // find the first integrable patch, merge immediately
        }
      }

      if ( best_j != (size_t)-1 ) {  // merge success
        numMergedPatches++;
        const GSEncoderPatchInformation &P_B = currPatches[best_j];
        GSEncoderPatchInformation        P_New;
        mergeTwoPatches( P_A, P_B, patchSize, newBoxCoord, P_New /*[out]*/ );
        nextPatches.push_back( P_New );
        mergedFlags[i]      = true;
        mergedFlags[best_j] = true;
        wasMerged           = true;

      } else {
        // No integrable patches found. Keep the current patch intact.
        nextPatches.push_back( P_A );
      }
    }
    if ( numMergedPatches ) {
      currPatches = std::move( nextPatches );
      totalNumMergedPatches += numMergedPatches;
    }
    std::cout << "--- Merge trial end : " << mergePass << ": " << numMergedPatches << " ---" << std::endl;
  }

  if ( totalNumMergedPatches ) { RawPatches = std::move( currPatches ); }
  std::cout << "--- Total num of merged patches : " << totalNumMergedPatches << " ---" << std::endl;

  return totalNumMergedPatches;
}

std::vector<GSEncoderPatchInformation> PCCGSPatchGenerator::splitPatchesAdaptiveQuantization(
    const PCCPointSet3                           &source,
    const std::vector<GSEncoderPatchInformation> &patches ) {
  size_t maxQuantizationGroupId = source.getMaxQuantizationGroupId();
  if ( !source.hasQuantizationGroupIds() || maxQuantizationGroupId == 0 ) { return patches; }

  const auto &quantizationGroupIds = source.getQuantizationGroupIds();

  // for each group, store list of patches
  std::vector<std::vector<std::vector<size_t>>> perGroupPointsIndicesInPatch( maxQuantizationGroupId + 1 );
  std::vector<std::vector<PCCVector3<size_t>>>  perGroupPatchUVDs( maxQuantizationGroupId + 1 );

  std::cout << "Split patches with adaptive quantization." << std::endl;
  for ( const auto &patchInfo : patches ) {
    PCCVector3<size_t> rawPatchUVD( patchInfo.patch.u1_, patchInfo.patch.v1_, patchInfo.patch.d1_ );
    const auto        &rawPointsBBox = patchInfo.pointsIndicesInPatch;
    for ( size_t g = 0; g <= maxQuantizationGroupId; ++g ) {
      perGroupPointsIndicesInPatch[g].emplace_back();
      auto &pointsIndicesInPatch = perGroupPointsIndicesInPatch[g].back();
      // Count points relevant for this group and save their indices
      for ( auto rawPointIndex : rawPointsBBox ) {
        auto quantizationGroupId = quantizationGroupIds[rawPointIndex];
        if ( quantizationGroupId == g ) { pointsIndicesInPatch.push_back( rawPointIndex ); }
      }
      if ( pointsIndicesInPatch.size() == 0 ) {
        perGroupPointsIndicesInPatch[g].pop_back();
        continue;  // Don't create empty patch
      }
      perGroupPatchUVDs[g].push_back( rawPatchUVD );
    }
  }
  std::vector<GSEncoderPatchInformation> splitPatches;
  splitPatches.reserve( patches.size() * ( maxQuantizationGroupId + 1 ) );
  // now we have per group: a list of patches with the point indices corresponding to them
  for ( size_t g = 0; g <= maxQuantizationGroupId; ++g ) {
    auto &perPatchSrcIdx = perGroupPointsIndicesInPatch[g];
    // iterate over patches (within group)
    for ( size_t p = 0; p < perPatchSrcIdx.size(); ++p ) {
      PCCVector3<size_t> &rawPatchUVD = perGroupPatchUVDs[g][p];
      PCCBox3D            bbox;
      for ( int j = 0; j < 3; j++ ) {
        bbox.min_[j] = rawPatchUVD[j];  // NOTE: bbox.max is unused
      }
      const auto               &rawPointsBBox = perPatchSrcIdx[p];
      GSEncoderPatchInformation patchInfo     = createPatchInfo( bbox, 0, rawPointsBBox );
      patchInfo.patch.shBitdepthDelta_        = source.getQuantizationGroupBitDepthDelta( g );
      splitPatches.push_back( patchInfo );
    }
  }
  return splitPatches;
}
