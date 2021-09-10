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
#include "PCCPatch.h"

using namespace pcc;

static const std::vector<uint8_t> g_orientation = {
    0, 0, 6, 0, 0, 0, 0, 6, 4, 0, 0, 5, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 7, 7, 0, 0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 4, 0, 5, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 5, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 3, 3, 0, 4, 3, 0, 0,
    5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 7, 7, 0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 7, 0, 0, 0, 0, 0, 0,
    0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 3, 3, 0, 3, 0, 0, 0, 4, 1, 0, 0, 0, 1, 0, 1, 0, 2, 3, 0, 0, 1, 2, 0, 0};
static const int8_t g_dilate[8][2] = {{1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}};

#define OCC( tl, t, tr, l, v, r, bl, b, br ) \
  ( ( tl << 7 ) | ( t << 6 ) | ( tr << 5 ) | ( l << 4 ) | ( r << 3 ) | ( bl << 2 ) | ( b << 1 ) | ( br ) )
#define ORIENTATION( p, c, w )                                                                               \
  g_orientation[OCC( p[c - w - 1], p[c - w], p[c - w + 1], p[c - 1], p[c], p[c + 1], p[c + w - 1], p[c + w], \
                     p[c + w + 1] )];

PCCPatch::PCCPatch() :
    index_( 0 ),
    frameIndex_( 0 ),
    tileIndex_( 0 ),
    u1_( 0 ),
    v1_( 0 ),
    d1_( 0 ),
    sizeD_( 0 ),
    sizeU_( 0 ),
    sizeV_( 0 ),
    u0_( 0 ),
    v0_( 0 ),
    sizeU0_( 0 ),
    sizeV0_( 0 ),
    occupancyResolution_( 0 ),
    projectionMode_( 0 ),
    levelOfDetailX_( 1 ),
    levelOfDetailY_( 1 ),
    normalAxis_( 0 ),
    tangentAxis_( 0 ),
    bitangentAxis_( 0 ),
    viewId_( 0 ),
    bestMatchIdx_( g_invalidPatchIndex ),
    refAtlasFrameIdx_( 0 ),
    predType_( 0 ),
    patchOrientation_( 0 ),
    isGlobalPatch_( false ),
    d0Count_( 0 ),
    eomCount_( 0 ),
    eomandD1Count_( 0 ),
    patchType_( I_INTRA ) {
  depth_[0].clear();
  depth_[1].clear();
  occupancy_.clear();
  depthEOM_.clear();
  depth0PCidx_.clear();
  pointLocalReconstructionModeByBlock_.clear();
  occupancyMap_.clear();
  borderPoints_.clear();
  neighboringPatches_.clear();
  depthMap_.clear();
};
PCCPatch::~PCCPatch() {
  depth_[0].clear();
  depth_[1].clear();
  occupancy_.clear();
  depthEOM_.clear();
  depth0PCidx_.clear();
  pointLocalReconstructionModeByBlock_.clear();
  occupancyMap_.clear();
  borderPoints_.clear();
  neighboringPatches_.clear();
  depthMap_.clear();
};

void PCCPatch::setViewId( size_t viewId ) {
  viewId_ = viewId;
  // now set the other variables according to the viewId
  switch ( viewId_ ) {
    case 0: setAxis( 0, 0, 2, 1, 0 ); break;
    case 1: setAxis( 0, 1, 2, 0, 0 ); break;
    case 2: setAxis( 0, 2, 0, 1, 0 ); break;
    case 3: setAxis( 0, 0, 2, 1, 1 ); break;
    case 4: setAxis( 0, 1, 2, 0, 1 ); break;
    case 5: setAxis( 0, 2, 0, 1, 1 ); break;
    case 6: setAxis( 1, 0, 2, 1, 0 ); break;
    case 7: setAxis( 1, 2, 0, 1, 0 ); break;
    case 8: setAxis( 1, 0, 2, 1, 1 ); break;
    case 9: setAxis( 1, 2, 0, 1, 1 ); break;
    case 10: setAxis( 2, 2, 0, 1, 0 ); break;
    case 11: setAxis( 2, 1, 2, 0, 0 ); break;
    case 12: setAxis( 2, 2, 0, 1, 1 ); break;
    case 13: setAxis( 2, 1, 2, 0, 1 ); break;
    case 14: setAxis( 3, 1, 2, 0, 0 ); break;
    case 15: setAxis( 3, 0, 2, 1, 0 ); break;
    case 16: setAxis( 3, 1, 2, 0, 1 ); break;
    case 17: setAxis( 3, 0, 2, 1, 1 ); break;
    default:
      std::cout << "ViewId (" << viewId_ << ") not allowed... exiting" << std::endl;
      exit( -1 );
      break;
  }
}
PCCPoint3D PCCPatch::canvasTo3D( const size_t x, const size_t y, const uint16_t depth ) const {
  PCCPoint3D point0;
  size_t     u = 0, v = 0;
  switch ( patchOrientation_ ) {
    case PATCH_ORIENTATION_DEFAULT:
      u = x - u0_ * occupancyResolution_;
      v = y - v0_ * occupancyResolution_;
      break;
    case PATCH_ORIENTATION_ROT90:
      v = ( sizeV0_ * occupancyResolution_ - 1 - ( x - u0_ * occupancyResolution_ ) );
      u = y - v0_ * occupancyResolution_;
      break;
    case PATCH_ORIENTATION_ROT180:
      u = ( sizeU0_ * occupancyResolution_ - 1 - ( x - u0_ * occupancyResolution_ ) );
      v = ( sizeV0_ * occupancyResolution_ - 1 - ( y - v0_ * occupancyResolution_ ) );
      break;
    case PATCH_ORIENTATION_ROT270:
      v = x - u0_ * occupancyResolution_;
      u = ( sizeU0_ * occupancyResolution_ - 1 - ( y - v0_ * occupancyResolution_ ) );
      break;
    case PATCH_ORIENTATION_MIRROR:
      u = ( sizeU0_ * occupancyResolution_ - 1 - ( x - u0_ * occupancyResolution_ ) );
      v = y - v0_ * occupancyResolution_;
      break;
    case PATCH_ORIENTATION_MROT90:
      v = ( sizeV0_ * occupancyResolution_ - 1 - ( x - u0_ * occupancyResolution_ ) );
      u = ( sizeU0_ * occupancyResolution_ - 1 - ( y - v0_ * occupancyResolution_ ) );
      break;
    case PATCH_ORIENTATION_MROT180:
      u = x - u0_ * occupancyResolution_;
      v = ( sizeV0_ * occupancyResolution_ - 1 - ( y - v0_ * occupancyResolution_ ) );
      break;
    case PATCH_ORIENTATION_MROT270:
      v = x - u0_ * occupancyResolution_;
      u = y - v0_ * occupancyResolution_;
      break;
    case PATCH_ORIENTATION_SWAP:  // swapAxis
      v = x - u0_ * occupancyResolution_;
      u = y - v0_ * occupancyResolution_;
      break;
    default: assert( 0 ); break;
  }
  point0[normalAxis_]    = generateNormalCoordinate( depth );
  point0[tangentAxis_]   = ( double( u ) * levelOfDetailX_ + u1_ );
  point0[bitangentAxis_] = ( double( v ) * levelOfDetailY_ + v1_ );
  return point0;
}

size_t PCCPatch::patch2Canvas( const size_t u, const size_t v, size_t canvasStride, size_t canvasHeight ) {
  size_t x, y;
  return patch2Canvas( u, v, canvasStride, canvasHeight, x, y );
}

size_t PCCPatch::patch2Canvas( const size_t u,
                               const size_t v,
                               size_t       canvasStride,
                               size_t       canvasHeight,
                               size_t&      x,
                               size_t&      y ) {
  switch ( patchOrientation_ ) {
    case PATCH_ORIENTATION_DEFAULT:
      x = u + u0_ * occupancyResolution_;
      y = v + v0_ * occupancyResolution_;
      break;
    case PATCH_ORIENTATION_ROT90:
      x = ( sizeV0_ * occupancyResolution_ - 1 - v ) + u0_ * occupancyResolution_;
      y = u + v0_ * occupancyResolution_;
      break;
    case PATCH_ORIENTATION_ROT180:
      x = ( sizeU0_ * occupancyResolution_ - 1 - u ) + u0_ * occupancyResolution_;
      y = ( sizeV0_ * occupancyResolution_ - 1 - v ) + v0_ * occupancyResolution_;
      break;
    case PATCH_ORIENTATION_ROT270:
      x = v + u0_ * occupancyResolution_;
      y = ( sizeU0_ * occupancyResolution_ - 1 - u ) + v0_ * occupancyResolution_;
      break;
    case PATCH_ORIENTATION_MIRROR:
      x = ( sizeU0_ * occupancyResolution_ - 1 - u ) + u0_ * occupancyResolution_;
      y = v + v0_ * occupancyResolution_;
      break;
    case PATCH_ORIENTATION_MROT90:
      x = ( sizeV0_ * occupancyResolution_ - 1 - v ) + u0_ * occupancyResolution_;
      y = ( sizeU0_ * occupancyResolution_ - 1 - u ) + v0_ * occupancyResolution_;
      break;
    case PATCH_ORIENTATION_MROT180:
      x = u + u0_ * occupancyResolution_;
      y = ( sizeV0_ * occupancyResolution_ - 1 - v ) + v0_ * occupancyResolution_;
      break;
    case PATCH_ORIENTATION_MROT270:
      x = v + u0_ * occupancyResolution_;
      y = u + v0_ * occupancyResolution_;
      break;
    case PATCH_ORIENTATION_SWAP:  // swapAxis
      x = v + u0_ * occupancyResolution_;
      y = u + v0_ * occupancyResolution_;
      break;
    default: assert( 0 ); break;
  }
  // checking the results are within canvas boundary (missing y check)
  if ( x >= canvasStride || y >= canvasHeight ) {
    printf(
        "patch2Canvas (x,y) is out of boundary : frame %zu, tile %zu canvassize %zux%zu : uvstart(%zu,%zu) "
        "size(%zu,%zu), uv(%zu,%zu), xy(%zu,%zu), orientation(%zu)\n",
        frameIndex_, tileIndex_, canvasStride, canvasHeight, u0_ * occupancyResolution_, v0_ * occupancyResolution_,
        sizeU0_ * occupancyResolution_, sizeV0_ * occupancyResolution_, u, v, x, y, patchOrientation_ );
    exit( 180 );
  }
  assert( x >= 0 );
  assert( y >= 0 );
  assert( x < canvasStride );
  assert( y < canvasHeight );
  return ( x + canvasStride * y );
}

int PCCPatch::patchBlock2CanvasBlock( const size_t uBlk,
                                      const size_t vBlk,
                                      size_t       canvasStrideBlk,
                                      size_t       canvasHeightBlk,
                                      const Tile   tile ) const {
  size_t x, y;
  switch ( patchOrientation_ ) {
    case PATCH_ORIENTATION_DEFAULT:
      x = uBlk + u0_;
      y = vBlk + v0_;
      break;
    case PATCH_ORIENTATION_ROT90:
      x = ( sizeV0_ - 1 - vBlk ) + u0_;
      y = uBlk + v0_;
      break;
    case PATCH_ORIENTATION_ROT180:
      x = ( sizeU0_ - 1 - uBlk ) + u0_;
      y = ( sizeV0_ - 1 - vBlk ) + v0_;
      break;
    case PATCH_ORIENTATION_ROT270:
      x = vBlk + u0_;
      y = ( sizeU0_ - 1 - uBlk ) + v0_;
      break;
    case PATCH_ORIENTATION_MIRROR:
      x = ( sizeU0_ - 1 - uBlk ) + u0_;
      y = vBlk + v0_;
      break;
    case PATCH_ORIENTATION_MROT90:
      x = ( sizeV0_ - 1 - vBlk ) + u0_;
      y = ( sizeU0_ - 1 - uBlk ) + v0_;
      break;
    case PATCH_ORIENTATION_MROT180:
      x = uBlk + u0_;
      y = ( sizeV0_ - 1 - vBlk ) + v0_;
      break;
    case PATCH_ORIENTATION_MROT270:
      x = vBlk + u0_;
      y = uBlk + v0_;
      break;
    case PATCH_ORIENTATION_SWAP:  // swapAxis
      x = vBlk + u0_;
      y = uBlk + v0_;
      break;
    default: return -1; break;
  }
  // checking the results are within canvasHeightBlk boundary (missing y check)
  if ( x >= canvasStrideBlk ) { return -1; }
  if ( y >= canvasHeightBlk ) { return -1; }
  if ( tile.minU != -1 ) {
    if ( x < tile.minU ) { return -1; }
    if ( y < tile.minV ) { return -1; }
    if ( x > tile.maxU ) { return -1; }
    if ( y > tile.maxV ) { return -1; }
  }
  return int( x + canvasStrideBlk * y );
}

bool PCCPatch::checkFitPatchCanvas( std::vector<bool> canvas,
                                    size_t            canvasStrideBlk,
                                    size_t            canvasHeightBlk,
                                    bool              bPrecedence,
                                    int               safeguard,
                                    const Tile        tile ) {
  for ( size_t v0 = 0; v0 < sizeV0_; ++v0 ) {
    for ( size_t u0 = 0; u0 < sizeU0_; ++u0 ) {
      for ( int deltaY = -safeguard; deltaY < safeguard + 1; deltaY++ ) {
        for ( int deltaX = -safeguard; deltaX < safeguard + 1; deltaX++ ) {
          int pos = patchBlock2CanvasBlock( u0 + deltaX, v0 + deltaY, canvasStrideBlk, canvasHeightBlk, tile );
          if ( pos < 0 ) {
            return false;
          } else {
            if ( bPrecedence ) {
              if ( canvas[pos] && occupancy_[u0 + sizeU0_ * v0] ) { return false; }
            } else {
              if ( canvas[pos] ) { return false; }
            }
          }
        }
      }
    }
  }
  return true;
}

bool PCCPatch::smallerRefFirst( const PCCPatch& rhs ) {
  if ( bestMatchIdx_ == -1 && rhs.getBestMatchIdx() == -1 ) {
    return gt( rhs );
  } else if ( bestMatchIdx_ == -1 || rhs.getBestMatchIdx() == -1 ) {
    return ( bestMatchIdx_ != -1 ) ? true : false;
  } else if ( bestMatchIdx_ == rhs.getBestMatchIdx() ) {
    return refAtlasFrameIdx_ < rhs.getRefAtlasFrameIndex();
  } else {
    return bestMatchIdx_ < rhs.getBestMatchIdx();
  }
}

bool PCCPatch::gt( const PCCPatch& rhs ) {
  size_t maxDim, rhsMaxDim;
  size_t minDim, rhsMinDim;
  // setting the largest dimension
  if ( sizeU0_ > sizeV0_ ) {
    maxDim = sizeU0_;
    minDim = sizeV0_;
  } else {
    maxDim = sizeV0_;
    minDim = sizeU0_;
  }
  // getting the largest dimensions for the block to be tested
  if ( rhs.sizeU0_ > rhs.sizeV0_ ) {
    rhsMaxDim = rhs.sizeU0_;
    rhsMinDim = rhs.sizeV0_;
  } else {
    rhsMaxDim = rhs.sizeV0_;
    rhsMinDim = rhs.sizeU0_;
  }
  // if the dimensions are the same, decide by the index
  return maxDim != rhsMaxDim ? ( maxDim > rhsMaxDim )
                             : ( minDim != rhsMinDim ) ? ( minDim > rhsMinDim ) : ( index_ < rhs.index_ );
}

void PCCPatch::print() const {
  printf( "Patch[%3zu] uv0 = %4zu %4zu / %4zu %4zu uvd1 = %4zu %4zu %4zu / %4zu %4zu %4zu \n", index_, u0_, v0_,
          sizeU0_, sizeV0_, u1_, v1_, d1_, sizeU_, sizeV_, sizeD_ );
}

void PCCPatch::printDecoder() const {
  printf( "Patch uv0 = %4zu %4zu / %4zu %4zu uvd1 = %4zu %4zu %4zu orientation = %4zu \n", u0_, v0_, sizeU0_, sizeV0_,
          u1_, v1_, d1_, patchOrientation_ );
}

void PCCPatch::getPatchHorizons( std::vector<int>& topHorizon,
                                 std::vector<int>& bottomHorizon,
                                 std::vector<int>& rightHorizon,
                                 std::vector<int>& leftHorizon ) {
  topHorizon.resize( sizeU0_, 0 );
  bottomHorizon.resize( sizeU0_, 0 );
  rightHorizon.resize( sizeV0_, 0 );
  leftHorizon.resize( sizeV0_, 0 );
  for ( int i = 0; i < sizeU0_; i++ ) {
    while ( !occupancy_[( sizeV0_ - 1 - topHorizon[i] ) * sizeU0_ + i] && ( topHorizon[i] < sizeV0_ - 1 ) ) {
      topHorizon[i]++;
    }
  }
  for ( int i = 0; i < sizeU0_; i++ ) {
    while ( !occupancy_[bottomHorizon[i] * sizeU0_ + i] && ( bottomHorizon[i] < sizeV0_ - 1 ) ) bottomHorizon[i]++;
  }
  for ( int i = 0; i < sizeV0_; i++ ) {
    while ( !occupancy_[i * sizeU0_ + sizeU0_ - 1 - rightHorizon[i]] && ( rightHorizon[i] < sizeU0_ - 1 ) ) {
      rightHorizon[i]++;
    }
  }
  for ( int i = 0; i < sizeV0_; i++ ) {
    while ( !occupancy_[i * sizeU0_ + leftHorizon[i]] && ( leftHorizon[i] < sizeU0_ - 1 ) ) leftHorizon[i]++;
  }
  if ( g_printDetailedInfo ) {
    std::cout << "Top Horizon :[";
    for ( int i = 0; i < sizeU0_; i++ ) { std::cout << topHorizon[i] << ","; }
    std::cout << "]" << std::endl;
    std::cout << "Bottom Horizon :[";
    for ( int i = 0; i < sizeU0_; i++ ) { std::cout << bottomHorizon[i] << ","; }
    std::cout << "]" << std::endl;
    std::cout << "Right Horizon :[";
    for ( int i = 0; i < sizeV0_; i++ ) { std::cout << rightHorizon[i] << ","; }
    std::cout << "]" << std::endl;
    std::cout << "Left Horizon :[";
    for ( int i = 0; i < sizeV0_; i++ ) { std::cout << leftHorizon[i] << ","; }
    std::cout << "]" << std::endl;
  }
}

int PCCPatch::calculateWastedSpace( std::vector<int>& horizon,
                                    std::vector<int>& topHorizon,
                                    std::vector<int>& bottomHorizon,
                                    std::vector<int>& rightHorizon,
                                    std::vector<int>& leftHorizon ) {
  int wasted_space          = 0;
  int wasted_space_external = 0;
  int wasted_space_internal = 0;
  int lambda                = 100;  //--> bias towards the upper part of the canvas
  if ( patchOrientation_ == PATCH_ORIENTATION_DEFAULT ) {
    for ( int idx = 0; idx < sizeU0_; idx++ ) {
      wasted_space_external += v0_ + bottomHorizon[idx] - horizon[u0_ + idx];
      // calculating internal wasted space --> because of new block2patch
      // restriction, this area only contains locations for the local patch
      for ( int idx2 = bottomHorizon[idx] + 1; idx2 < sizeV0_ - topHorizon[idx]; idx2++ ) {
        if ( !occupancy_[idx2 * sizeU0_ + idx] ) wasted_space_internal++;
      }
    }
    wasted_space = int( lambda * v0_ + wasted_space_external + wasted_space_internal );
  } else if ( patchOrientation_ == PATCH_ORIENTATION_ROT90 ) {
    for ( int idx = 0; idx < sizeV0_; idx++ ) {
      wasted_space_external += v0_ + leftHorizon[sizeV0_ - 1 - idx] - horizon[u0_ + idx];
      // calculating internal wasted space
      for ( int idx2 = int( sizeU0_ - 1 - rightHorizon[idx] ); idx2 >= leftHorizon[idx]; idx2-- ) {
        if ( !occupancy_[idx * sizeU0_ + idx2] ) wasted_space_internal++;
      }
    }
    wasted_space = int( lambda * v0_ + wasted_space_external + wasted_space_internal );
  } else if ( patchOrientation_ == PATCH_ORIENTATION_ROT180 ) {
    for ( int idx = 0; idx < sizeU0_; idx++ ) {
      wasted_space_external += v0_ + topHorizon[sizeU0_ - 1 - idx] - horizon[u0_ + idx];
      // calculating internal wasted space
      for ( int idx2 = int( sizeV0_ - 1 - topHorizon[idx] ); idx2 >= bottomHorizon[idx]; idx2-- ) {
        if ( !occupancy_[idx2 * sizeU0_ + idx] ) wasted_space_internal++;
      }
    }
    wasted_space = int( lambda * v0_ + wasted_space_external + wasted_space_internal );
  } else if ( patchOrientation_ == PATCH_ORIENTATION_ROT270 ) {
    for ( int idx = 0; idx < sizeV0_; idx++ ) {
      wasted_space_external += v0_ + rightHorizon[idx] - horizon[u0_ + idx];
      // calculating internal wasted space
      for ( int idx2 = leftHorizon[idx] + 1; idx2 < sizeU0_ - rightHorizon[idx]; idx2++ ) {
        if ( !occupancy_[idx * sizeU0_ + idx2] ) wasted_space_internal++;
      }
    }
    wasted_space = int( lambda * v0_ + wasted_space_external + wasted_space_internal );
  } else if ( patchOrientation_ == PATCH_ORIENTATION_MIRROR ) {
    for ( int idx = 0; idx < sizeU0_; idx++ ) {
      wasted_space_external += v0_ + bottomHorizon[sizeU0_ - 1 - idx] - horizon[u0_ + idx];
      for ( int idx2 = bottomHorizon[idx] + 1; idx2 < sizeV0_ - topHorizon[idx]; idx2++ ) {
        if ( !occupancy_[idx2 * sizeU0_ + idx] ) wasted_space_internal++;
      }
    }
    wasted_space = int( lambda * v0_ + wasted_space_external + wasted_space_internal );
  } else if ( patchOrientation_ == PATCH_ORIENTATION_MROT90 ) {
    for ( int idx = 0; idx < sizeV0_; idx++ ) {
      wasted_space_external += v0_ + rightHorizon[sizeV0_ - 1 - idx] - horizon[u0_ + idx];
      // calculating internal wasted space
      for ( int idx2 = leftHorizon[idx] + 1; idx2 < sizeU0_ - rightHorizon[idx]; idx2++ ) {
        if ( !occupancy_[idx * sizeU0_ + idx2] ) wasted_space_internal++;
      }
    }
    wasted_space = int( lambda * v0_ + wasted_space_external + wasted_space_internal );
  } else if ( patchOrientation_ == PATCH_ORIENTATION_MROT180 ) {
    for ( int idx = 0; idx < sizeU0_; idx++ ) {
      wasted_space_external += v0_ + topHorizon[idx] - horizon[u0_ + idx];
      // calculating internal wasted space
      for ( int idx2 = int( sizeV0_ - 1 - topHorizon[idx] ); idx2 >= bottomHorizon[idx]; idx2-- ) {
        if ( !occupancy_[idx2 * sizeU0_ + idx] ) wasted_space_internal++;
      }
    }
    wasted_space = int( lambda * v0_ + wasted_space_external + wasted_space_internal );
  } else if ( patchOrientation_ == PATCH_ORIENTATION_MROT270 ) {
    for ( int idx = 0; idx < sizeV0_; idx++ ) {
      wasted_space_external += v0_ + leftHorizon[idx] - horizon[u0_ + idx];
      for ( int idx2 = int( sizeU0_ - 1 - leftHorizon[idx] ); idx2 >= rightHorizon[idx]; idx2-- ) {
        if ( !occupancy_[idx * sizeU0_ + idx2] ) wasted_space_internal++;
      }
    }
    wasted_space = int( lambda * v0_ + wasted_space_external + wasted_space_internal );
  } else if ( patchOrientation_ == PATCH_ORIENTATION_SWAP ) {
    for ( int idx = 0; idx < sizeV0_; idx++ ) {
      wasted_space_external += v0_ + leftHorizon[idx] - horizon[u0_ + idx];
      for ( int idx2 = int( sizeU0_ - 1 - leftHorizon[idx] ); idx2 >= rightHorizon[idx]; idx2-- ) {
        if ( !occupancy_[idx * sizeU0_ + idx2] ) wasted_space_internal++;
      }
    }
    wasted_space = int( lambda * v0_ + wasted_space_external + wasted_space_internal );
  }
  return wasted_space;
}

bool PCCPatch::isPatchLocationAboveHorizon( std::vector<int>& horizon,
                                            std::vector<int>& topHorizon,
                                            std::vector<int>& bottomHorizon,
                                            std::vector<int>& rightHorizon,
                                            std::vector<int>& leftHorizon ) {
  if ( patchOrientation_ == PATCH_ORIENTATION_DEFAULT ) {
    for ( int idx = 0; idx < sizeU0_; idx++ ) {
      if ( v0_ + bottomHorizon[idx] < horizon[u0_ + idx] ) { return false; }
    }
  } else if ( patchOrientation_ == PATCH_ORIENTATION_ROT90 ) {
    for ( int idx = 0; idx < sizeV0_; idx++ ) {
      if ( v0_ + leftHorizon[sizeV0_ - 1 - idx] < horizon[u0_ + idx] ) { return false; }
    }
  } else if ( patchOrientation_ == PATCH_ORIENTATION_ROT180 ) {
    for ( int idx = 0; idx < sizeU0_; idx++ ) {
      if ( v0_ + topHorizon[sizeU0_ - 1 - idx] < horizon[u0_ + idx] ) { return false; }
    }
  } else if ( patchOrientation_ == PATCH_ORIENTATION_ROT270 ) {
    for ( int idx = 0; idx < sizeV0_; idx++ ) {
      if ( v0_ + rightHorizon[idx] < horizon[u0_ + idx] ) { return false; }
    }
  } else if ( patchOrientation_ == PATCH_ORIENTATION_MIRROR ) {
    for ( int idx = 0; idx < sizeU0_; idx++ ) {
      if ( v0_ + bottomHorizon[sizeU0_ - 1 - idx] < horizon[u0_ + idx] ) { return false; }
    }
  } else if ( patchOrientation_ == PATCH_ORIENTATION_MROT90 ) {
    for ( int idx = 0; idx < sizeV0_; idx++ ) {
      if ( v0_ + rightHorizon[sizeV0_ - 1 - idx] < horizon[u0_ + idx] ) { return false; }
    }
  } else if ( patchOrientation_ == PATCH_ORIENTATION_MROT180 ) {
    for ( int idx = 0; idx < sizeU0_; idx++ ) {
      if ( v0_ + topHorizon[idx] < horizon[u0_ + idx] ) { return false; }
    }
  } else if ( patchOrientation_ == PATCH_ORIENTATION_MROT270 ) {
    for ( int idx = 0; idx < sizeV0_; idx++ ) {
      if ( v0_ + leftHorizon[idx] < horizon[u0_ + idx] ) { return false; }
    }
  } else if ( patchOrientation_ == PATCH_ORIENTATION_SWAP ) {
    for ( int idx = 0; idx < sizeV0_; idx++ ) {
      if ( v0_ + leftHorizon[idx] < horizon[u0_ + idx] ) { return false; }
    }
  }
  return true;
}

void PCCPatch::updateHorizon( std::vector<int>& horizon,
                              std::vector<int>& topHorizon,
                              std::vector<int>& bottomHorizon,
                              std::vector<int>& rightHorizon,
                              std::vector<int>& leftHorizon ) {
  size_t bestU           = u0_;
  size_t bestV           = v0_;
  size_t bestOrientation = patchOrientation_;
  int    newVal;
  if ( bestOrientation == PATCH_ORIENTATION_DEFAULT ) {
    for ( int idx = 0; idx < sizeU0_; idx++ ) {
      newVal = int( bestV + sizeV0_ - 1 - topHorizon[idx] );
      if ( newVal > horizon[bestU + idx] ) horizon[bestU + idx] = newVal;
    }
  } else if ( bestOrientation == PATCH_ORIENTATION_ROT90 ) {
    for ( int idx = 0; idx < sizeV0_; idx++ ) {
      newVal = int( bestV + sizeU0_ - 1 - rightHorizon[sizeV0_ - 1 - idx] );
      if ( newVal > horizon[bestU + idx] ) horizon[bestU + idx] = newVal;
    }
  } else if ( bestOrientation == PATCH_ORIENTATION_ROT180 ) {
    for ( int idx = 0; idx < sizeU0_; idx++ ) {
      newVal = int( bestV + sizeV0_ - 1 - bottomHorizon[sizeU0_ - 1 - idx] );
      if ( newVal > horizon[bestU + idx] ) horizon[bestU + idx] = newVal;
    }
  } else if ( bestOrientation == PATCH_ORIENTATION_ROT270 ) {
    for ( int idx = 0; idx < sizeV0_; idx++ ) {
      newVal = int( bestV + sizeU0_ - 1 - leftHorizon[idx] );
      if ( newVal > horizon[bestU + idx] ) horizon[bestU + idx] = newVal;
    }
  } else if ( bestOrientation == PATCH_ORIENTATION_MIRROR ) {
    for ( int idx = 0; idx < sizeU0_; idx++ ) {
      newVal = int( bestV + sizeV0_ - 1 - topHorizon[sizeU0_ - 1 - idx] );
      if ( newVal > horizon[bestU + idx] ) horizon[bestU + idx] = newVal;
    }
  } else if ( bestOrientation == PATCH_ORIENTATION_MROT90 ) {
    for ( int idx = 0; idx < sizeV0_; idx++ ) {
      newVal = int( bestV + sizeU0_ - 1 - leftHorizon[sizeV0_ - 1 - idx] );
      if ( newVal > horizon[bestU + idx] ) horizon[bestU + idx] = newVal;
    }
  } else if ( bestOrientation == PATCH_ORIENTATION_MROT180 ) {
    for ( int idx = 0; idx < sizeU0_; idx++ ) {
      newVal = int( bestV + sizeV0_ - 1 - bottomHorizon[idx] );
      if ( newVal > horizon[bestU + idx] ) horizon[bestU + idx] = newVal;
    }
  } else if ( bestOrientation == PATCH_ORIENTATION_MROT270 ) {
    for ( int idx = 0; idx < sizeV0_; idx++ ) {
      newVal = int( bestV + sizeU0_ - 1 - rightHorizon[idx] );
      if ( newVal > horizon[bestU + idx] ) horizon[bestU + idx] = newVal;
    }
  } else if ( bestOrientation == PATCH_ORIENTATION_SWAP ) {
    for ( int idx = 0; idx < sizeV0_; idx++ ) {
      newVal = int( bestV + sizeU0_ - 1 - rightHorizon[idx] );
      if ( newVal > horizon[bestU + idx] ) horizon[bestU + idx] = newVal;
    }
  }
}

int PCCPatch::patchBlock2CanvasBlockForGPA( const size_t uBlk,
                                            const size_t vBlk,
                                            size_t       canvasStrideBlk,
                                            size_t       canvasHeightBlk ) const {
  size_t x, y;
  switch ( curGPAPatchData_.patchOrientation_ ) {
    case PATCH_ORIENTATION_DEFAULT:
      x = uBlk + curGPAPatchData_.u0_;
      y = vBlk + curGPAPatchData_.v0_;
      break;
    case PATCH_ORIENTATION_ROT90:
      x = ( curGPAPatchData_.sizeV0_ - 1 - vBlk ) + curGPAPatchData_.u0_;
      y = uBlk + curGPAPatchData_.v0_;
      break;
    case PATCH_ORIENTATION_ROT180:
      x = ( curGPAPatchData_.sizeU0_ - 1 - uBlk ) + curGPAPatchData_.u0_;
      y = ( curGPAPatchData_.sizeV0_ - 1 - vBlk ) + curGPAPatchData_.v0_;
      break;
    case PATCH_ORIENTATION_ROT270:
      x = vBlk + curGPAPatchData_.u0_;
      y = ( curGPAPatchData_.sizeU0_ - 1 - uBlk ) + curGPAPatchData_.v0_;
      break;
    case PATCH_ORIENTATION_MIRROR:
      x = ( curGPAPatchData_.sizeU0_ - 1 - uBlk ) + curGPAPatchData_.u0_;
      y = vBlk + curGPAPatchData_.v0_;
      break;
    case PATCH_ORIENTATION_MROT90:
      x = ( curGPAPatchData_.sizeV0_ - 1 - vBlk ) + curGPAPatchData_.u0_;
      y = ( curGPAPatchData_.sizeU0_ - 1 - uBlk ) + curGPAPatchData_.v0_;
      break;
    case PATCH_ORIENTATION_MROT180:
      x = uBlk + curGPAPatchData_.u0_;
      y = ( curGPAPatchData_.sizeV0_ - 1 - vBlk ) + curGPAPatchData_.v0_;
      break;
    case PATCH_ORIENTATION_MROT270:
      x = vBlk + curGPAPatchData_.u0_;
      y = uBlk + curGPAPatchData_.v0_;
      break;
    case PATCH_ORIENTATION_SWAP:  // swapAxis
      x = vBlk + curGPAPatchData_.u0_;
      y = uBlk + curGPAPatchData_.v0_;
      break;
    default: return -1; break;
  }
  // checking the results are within canvas boundary (missing y check)
  if ( x >= canvasStrideBlk ) return -1;
  if ( y >= canvasHeightBlk ) return -1;
  return int( x + canvasStrideBlk * y );
}

bool PCCPatch::checkFitPatchCanvasForGPA( std::vector<bool> canvas,
                                          size_t            canvasStrideBlk,
                                          size_t            canvasHeightBlk,
                                          bool              bPrecedence,
                                          int               safeguard ) {
  for ( size_t v0 = 0; v0 < curGPAPatchData_.sizeV0_; ++v0 ) {
    for ( size_t u0 = 0; u0 < curGPAPatchData_.sizeU0_; ++u0 ) {
      for ( int deltaY = -safeguard; deltaY < safeguard + 1; deltaY++ ) {
        for ( int deltaX = -safeguard; deltaX < safeguard + 1; deltaX++ ) {
          int pos = patchBlock2CanvasBlockForGPA( u0 + deltaX, v0 + deltaY, canvasStrideBlk, canvasHeightBlk );
          if ( pos < 0 ) {
            return false;
          } else {
            if ( bPrecedence ) {
              if ( canvas[pos] && occupancy_[u0 + sizeU0_ * v0] ) { return false; }
            } else {
              if ( canvas[pos] ) { return false; }
            }
          }
        }
      }
    }
  }

  return true;
}

void PCCPatch::allocOneLayerData() {
  pointLocalReconstructionLevel_       = 0;
  pointLocalReconstructionModeByPatch_ = 0;
  pointLocalReconstructionModeByBlock_.resize( sizeU0_ * sizeV0_, 0 );
  std::fill( pointLocalReconstructionModeByBlock_.begin(), pointLocalReconstructionModeByBlock_.end(), 0 );
}

void PCCPatch::setDepthFromGeometryVideo( const std::vector<uint16_t>& geometryVideo,
                                          const int32_t                u2,
                                          const int32_t                v2,
                                          int32_t                      width,
                                          int32_t                      height,
                                          int32_t                      occupancyPrecision,
                                          int16_t*                     depth ) {
  const int32_t x0 = ( int32_t )( u0_ * occupancyResolution_ );
  const int32_t y0 = ( int32_t )( v0_ * occupancyResolution_ );
  depth += v2 * depthMapWidth_;
  switch ( patchOrientation_ ) {
    case PATCH_ORIENTATION_DEFAULT:
      for ( int32_t j = 0, v = v2; j < occupancyPrecision; j++, v++, depth += depthMapWidth_ ) {
        int32_t y = ( v + y0 ) * width + x0;
        for ( int32_t i = 0, u = u2; i < occupancyPrecision; i++, u++ ) { depth[u] = geometryVideo[u + y]; }
      }
      break;
    case PATCH_ORIENTATION_SWAP:  // swapAxis
      for ( int32_t j = 0, v = v2; j < occupancyPrecision; j++, v++, depth += depthMapWidth_ ) {
        int32_t x = v + x0 + width * y0;
        for ( int32_t i = 0, u = u2, y = u2 * width; i < occupancyPrecision; i++, u += 1, y += width ) {
          depth[u] = geometryVideo[x + y];
        }
      }
      break;
    case PATCH_ORIENTATION_ROT90:
      for ( int32_t j = 0, v = v2; j < occupancyPrecision; j++, v++, depth += depthMapWidth_ ) {
        int32_t x = int32_t( ( sizeV0_ * occupancyResolution_ - 1 - v ) + x0 );
        for ( int32_t i = 0, u = u2; i < occupancyPrecision; i++, u++ ) {
          int32_t y = u + y0;
          depth[u]  = geometryVideo[x + width * y];
        }
      }
      break;
    case PATCH_ORIENTATION_ROT180:
      for ( int32_t j = 0, v = v2; j < occupancyPrecision; j++, v++, depth += depthMapWidth_ ) {
        int32_t y = ( int32_t )( ( sizeV0_ * occupancyResolution_ - 1 - v ) + y0 );
        for ( int32_t i = 0, u = u2; i < occupancyPrecision; i++, u++ ) {
          int32_t x = ( int32_t )( ( sizeU0_ * occupancyResolution_ - 1 - u ) + x0 );
          depth[u]  = geometryVideo[x + width * y];
        }
      }
      break;
    case PATCH_ORIENTATION_ROT270:
      for ( int32_t j = 0, v = v2; j < occupancyPrecision; j++, v++, depth += depthMapWidth_ ) {
        int32_t x = v + x0;
        for ( int32_t i = 0, u = u2; i < occupancyPrecision; i++, u++ ) {
          int32_t y = int32_t( ( sizeU0_ * occupancyResolution_ - 1 - u ) + y0 );
          depth[u]  = geometryVideo[x + width * y];
        }
      }
      break;
    case PATCH_ORIENTATION_MIRROR:
      for ( int32_t j = 0, v = v2; j < occupancyPrecision; j++, v++, depth += depthMapWidth_ ) {
        int32_t y = v + y0;
        for ( int32_t i = 0, u = u2; i < occupancyPrecision; i++, u++ ) {
          int32_t x = int32_t( sizeU0_ * occupancyResolution_ - 1 - u ) + x0;
          depth[u]  = geometryVideo[x + width * y];
        }
      }
      break;
    case PATCH_ORIENTATION_MROT90:
      for ( int32_t j = 0, v = v2; j < occupancyPrecision; j++, v++, depth += depthMapWidth_ ) {
        int32_t x = int32_t( sizeV0_ * occupancyResolution_ - 1 - v ) + x0;
        for ( int32_t i = 0, u = u2; i < occupancyPrecision; i++, u++ ) {
          int32_t y = int32_t( sizeU0_ * occupancyResolution_ - 1 - u ) + y0;
          depth[u]  = geometryVideo[x + width * y];
        }
      }
      break;
    case PATCH_ORIENTATION_MROT180:
      for ( int32_t j = 0, v = v2; j < occupancyPrecision; j++, v++, depth += depthMapWidth_ ) {
        int32_t y = int32_t( ( sizeV0_ * occupancyResolution_ - 1 - v ) + y0 );
        for ( int32_t i = 0, u = u2; i < occupancyPrecision; i++, u++ ) {
          int32_t x = u + x0;
          depth[u]  = geometryVideo[x + width * y];
        }
      }
      break;
    case PATCH_ORIENTATION_MROT270:
      for ( int32_t j = 0, v = v2; j < occupancyPrecision; j++, v++, depth += depthMapWidth_ ) {
        int32_t x = v + x0;
        for ( int32_t i = 0, u = u2; i < occupancyPrecision; i++, u++ ) {
          int32_t y = u + y0;
          depth[u]  = geometryVideo[x + width * y];
        }
      }
      break;
    default:
      assert( 0 );
      printf( "patchOrientation_ = %zu not supported \n", patchOrientation_ );
      exit( -1 );
      break;
  }
}

void PCCPatch::setLocalData( const std::vector<uint8_t>&  occupancyMapVideo,
                             const std::vector<uint16_t>& geometryVideo,
                             std::vector<size_t>&         blockToPatch,
                             const int32_t                width,
                             const int32_t                height,
                             const int32_t                occupancyPrecision,
                             const int32_t                threhold ) {
  border_         = occupancyPrecision >= 8 ? 16 : 8;
  depthMapWidth_  = sizeU0_ * occupancyResolution_ + 2 * border_;
  depthMapHeight_ = sizeV0_ * occupancyResolution_ + 2 * border_;
  depthMap_.resize( depthMapWidth_ * depthMapHeight_, 0 );
  occupancyMap_.resize( depthMapWidth_ * depthMapHeight_, 0 );
  const int32_t patchIndexPlusOne  = int32_t( indexCopy_ ) + 1;
  const auto    ocmWidth           = width / occupancyPrecision;
  const int32_t blockToPatchWidth  = width / occupancyResolution_;
  const int32_t blockToPatchHeight = height / occupancyResolution_;
  const int32_t occupancyResByPres = int32_t( occupancyResolution_ / occupancyPrecision );
  const int32_t shiftPrecision =
      occupancyPrecision == 1 ? 0 : occupancyPrecision == 2 ? 1 : occupancyPrecision == 4 ? 2 : 3;
  uint8_t* ocm   = occupancyMap_.data() + border_ + depthMapWidth_ * border_;
  int16_t* depth = depthMap_.data() + border_ + depthMapWidth_ * border_;
  for ( int32_t v0 = 0; v0 < sizeV0_; ++v0 ) {
    for ( int32_t u0 = 0; u0 < sizeU0_; ++u0 ) {
      const int32_t blockIndex = patchBlock2CanvasBlock( u0, v0, blockToPatchWidth, blockToPatchHeight );
      if ( blockToPatch[blockIndex] == patchIndexPlusOne ) {
        for ( int32_t v1 = 0; v1 < occupancyResByPres; v1++ ) {
          const int32_t v2 = int32_t( v0 * occupancyResolution_ + v1 * occupancyPrecision );
          for ( int32_t u1 = 0; u1 < occupancyResByPres; u1++ ) {
            const int32_t u2 = int32_t( u0 * occupancyResolution_ + u1 * occupancyPrecision );
            size_t        x, y;
            patch2Canvas( u2, v2, width, height, x, y );
            if ( occupancyMapVideo[( y >> shiftPrecision ) * ocmWidth + ( x >> shiftPrecision )] > threhold ) {
              setDepthFromGeometryVideo( geometryVideo, u2, v2, width, height, occupancyPrecision, depth );
              for ( int32_t j = 0, v = v2 * depthMapWidth_; j < occupancyPrecision; j++, v += depthMapWidth_ ) {
                for ( int32_t i = 0, u = u2 + v; i < occupancyPrecision; i++, u++ ) { ocm[u] = 1; }
              }
            }
          }
        }
      }
    }
  }
}

const bool PCCPatch::isBorder( size_t u, size_t v ) {  
  const int  x1 = u + border_ - 2, x2 = u + border_ + 2;
  const int  y1 = ( v + border_ - 2 ) * depthMapWidth_, y2 = ( v + border_ + 2 ) * depthMapWidth_;
  for ( int y = y1; y <= y2; y += depthMapWidth_ ) {
    for ( int x = x1; x <= x2; ++x ) {
      if ( occupancyMap_[y + x] == 0 ) { return true; }
    }
  }
  return false;
}
void PCCPatch::generateBorderPoints3D() {
  const int32_t w   = depthMapWidth_;
  boundingBox_.min_ = PCCPoint3D( ( std::numeric_limits<int16_t>::max )() );
  boundingBox_.max_ = PCCPoint3D( ( std::numeric_limits<int16_t>::min )() );
  int32_t c         = border_ + border_ * w;
  for ( int32_t v = 0; v < sizeV0_ * occupancyResolution_; v++, c += border_ << 1 ) {
    for ( int32_t u = 0; u < sizeU0_ * occupancyResolution_; u++, c++ ) {
      if ( ( occupancyMap_[c] ) &&
           ( ( !occupancyMap_[c - 1] ) || ( !occupancyMap_[c + 1] ) || ( !occupancyMap_[c - w] ) ||
             ( !occupancyMap_[c + w] ) || ( !occupancyMap_[c - 2] ) || ( !occupancyMap_[c + 2] ) ||
             ( !occupancyMap_[c - 2 * w] ) || ( !occupancyMap_[c + 2 * w] ) || ( !occupancyMap_[c + w - 1] ) ||
             ( !occupancyMap_[c + w + 1] ) || ( !occupancyMap_[c - w - 1] ) || ( !occupancyMap_[c - w + 1] ) ) ) {
        auto point = generatePoint( u, v, depthMap_[border_ + ( v + border_ ) * w + u] );
        borderPoints_.push_back( point );
        boundingBox_.add( point );
      }
    }
  }
}

void PCCPatch::filtering( const int8_t           passesCount,
                          const int8_t           filterSize,
                          const int8_t           log2Threshold,
                          std::vector<PCCPatch>& patches ) {
  const int8_t         localWindowSizeU = filterSize;
  const int32_t        localWindowSizeV = filterSize >> 1;
  PCCInt16Box3D        boundingBox      = boundingBox_;
  const int16_t        sizeX            = sizeU0_ * occupancyResolution_;
  const int16_t        sizeY            = sizeV0_ * occupancyResolution_;
  const int16_t        threshold        = log2Threshold * log2Threshold;
  const int32_t        size             = depthMapWidth_ * depthMapHeight_;
  const int16_t        undefined        = ( std::numeric_limits<int16_t>::max )();
  std::vector<int16_t> neighborDepth;
  neighborDepth.resize( size, undefined );
  boundingBox.min_ -= PCCPoint3D( 8 );
  boundingBox.max_ += PCCPoint3D( 8 );
  const int32_t shift = ( int32_t )( ( -(int32_t)v1_ + border_ ) * depthMapWidth_ - (int32_t)u1_ + border_ );
  for ( auto& i : neighboringPatches_ ) {
    for ( auto& point : patches[i].borderPoints_ ) {
      if ( boundingBox.contains( point ) ) {
        int32_t       d = generateDepth( point );
        const int32_t c = shift + point[bitangentAxis_] * depthMapWidth_ + point[tangentAxis_];
        if ( abs( d - depthMap_[c] ) <= threshold ) {
          if ( abs( d - depthMap_[c] ) < abs( neighborDepth[c] - depthMap_[c] ) ) { neighborDepth[c] = d; }
        }
      }
    }
  }
  std::vector<uint8_t> newOccupancyMap;
  newOccupancyMap.resize( size, 0 );
  for ( size_t iter = 0; iter < passesCount; iter++ ) {  // HN : OMap precision =4, passescount = 2
    uint8_t* src = iter % 2 == 0 ? occupancyMap_.data() : newOccupancyMap.data();  // iter=0, occupancyMap_.data(),
                                                                                   // iter=1, newOccupancyMap.data()
    uint8_t* dst = iter % 2 == 1 ? occupancyMap_.data() : newOccupancyMap.data();
    for ( int32_t v = 0, c = border_ * depthMapWidth_ + border_; v < sizeY; v++, c += 2 * border_ ) {
      for ( int32_t u = 0; u < sizeX; u++, c++ ) {
        if ( src[c] == 0 ) {
          dst[c] = 0;
        } else {
          int8_t numNeigblor = src[c - 1] + src[c + 1] + src[c - depthMapWidth_] + src[c + depthMapWidth_];
          if ( numNeigblor == 0 ) {
            dst[c] = 0;
          } else if ( numNeigblor == 4 ) {
            dst[c] = 1;
          } else {
            float         sumE = 0, sumP = 0;
            int32_t       count  = 0;
            const int8_t  orX    = ORIENTATION( src, c, depthMapWidth_ );
            const int8_t  orY    = ( orX + 2 ) % 8;
            const int8_t* dX     = g_dilate[orX];
            const int8_t* dY     = g_dilate[orY];
            const int32_t shiftX = dX[0] + dX[1] * depthMapWidth_;
            const int32_t shiftY = dY[0] + dY[1] * depthMapWidth_;
            const int32_t dE     = depthMap_[c - shiftX];
            const int32_t dP     = depthMap_[c];
            int32_t       cx     = c - localWindowSizeU * shiftX - localWindowSizeV * shiftY;
            int32_t       du1    = -localWindowSizeU * dX[0] + -localWindowSizeV * dY[0];
            int32_t       dv1    = -localWindowSizeU * dX[1] + -localWindowSizeV * dY[1];
            for ( int32_t dx = -localWindowSizeU; dx <= localWindowSizeU;
                  dx++, cx += shiftX, du1 += dX[0], dv1 += dX[1] ) {
              for ( int32_t dy = -localWindowSizeV, cn = cx, du = du1, dv = dv1; dy <= localWindowSizeV;
                    dy++, cn += shiftY, du += dY[0], dv += dY[1] ) {
                if ( neighborDepth[cn] != undefined ) {
                  sumP += sqrt( du * du + dv * dv + ( neighborDepth[cn] - dP ) * ( neighborDepth[cn] - dP ) );
                  sumE += sqrt( ( du + dX[0] ) * ( du + dX[0] ) + ( dv + dX[1] ) * ( dv + dX[1] ) +
                                ( neighborDepth[cn] - dE ) * ( neighborDepth[cn] - dE ) );
                  count++;
                }
              }
            }
            dst[c] = ( ( count == 0 ) || ( sumE >= sumP ) );
          }
        }
      }
    }
  }
  if ( passesCount % 2 == 1 ) { memcpy( occupancyMap_.data(), newOccupancyMap.data(), size * sizeof( uint8_t ) ); }
}

void PatchBlockFiltering::patchBorderFiltering( size_t imageWidth,
                                                size_t imageHeight,
                                                size_t occupancyResolution,
                                                size_t occupancyPrecision,
                                                size_t thresholdLossyOM,
                                                int8_t passesCount,
                                                int8_t filterSize,
                                                int8_t log2Threshold ) {
  // Generate border points
  for ( size_t patchIndex = 0; patchIndex < patches_->size(); patchIndex++ ) {
    auto& patch = patches_->at( patchIndex );
    patch.setIndexCopy( patchIndex );
    patch.setLocalData( *occupancyMapVideo_, *geometryVideo_, *blockToPatch_, (int32_t)imageWidth, (int32_t)imageHeight,
                        (int32_t)occupancyPrecision, (int32_t)thresholdLossyOM );
    patch.generateBorderPoints3D();
  }
  for ( auto& patch : *patches_ ) {
    for ( auto& other : *patches_ ) {
      if ( patch.getIndexCopy() != other.getIndexCopy() && patch.intersects( other ) ) {
        patch.getNeighboringPatches().push_back( other.getIndexCopy() );
      }
    }
  }
  // Filtering;
  for ( auto& patch : *patches_ ) { patch.filtering( passesCount, filterSize, log2Threshold, *patches_ ); }
  for ( auto& patch : *patches_ ) { patch.clearPatchBlockFilteringData(); }
}

