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

#include "PCCVideo.h"
#include "PCCImage.h"
#include "PCCPointSet.h"
#include "PCCKdTree.h"
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCGroupOfFrames.h"
#include "PCCPatch.h"
#if defined( ENABLE_TBB )
#include <tbb/tbb.h>
#endif

#include "PCCCodec.h"

using namespace pcc;

PCCCodec::PCCCodec() {}
PCCCodec::~PCCCodec() = default;

void PCCCodec::smoothPointCloudPostprocess( PCCPointSet3&                       reconstruct,
                                            const PCCColorTransform             colorTransform,
                                            const GeneratePointCloudParameters& params,
                                            std::vector<uint32_t>&              partition ) {
#ifdef CODEC_TRACE
  printChecksum( reconstruct, "smoothPointCloudPostprocess in" );
  TRACE_CODEC( "  flagGeometrySmoothing_ = %d \n", params.flagGeometrySmoothing_ );
  TRACE_CODEC( "  gridSmoothing_         = %d \n", params.gridSmoothing_ );
  TRACE_CODEC( "  gridSize_              = %zu \n", params.gridSize_ );
  TRACE_CODEC( "  thresholdSmoothing_    = %f \n", params.thresholdSmoothing_ );
  TRACE_CODEC( "  pbfEnableFlag_         = %d \n", params.pbfEnableFlag_ );
#endif
  if ( reconstruct.getPointCount() == 0 ) { return; }
  if ( params.flagGeometrySmoothing_ ) {
    if ( params.gridSmoothing_ ) {
      // reset for each GOF
      PCCInt16Box3D boundingBox;
      boundingBox.min_ = boundingBox.max_ = reconstruct[0];
      for ( int j = 0; j < reconstruct.getPointCount(); j++ ) {
        const PCCPoint3D point = reconstruct[j];
        for ( size_t k = 0; k < 3; ++k ) {
          if ( point[k] < boundingBox.min_[k] ) { boundingBox.min_[k] = floor( point[k] ); }
          if ( point[k] > boundingBox.max_[k] ) { boundingBox.max_[k] = ceil( point[k] ); }
        }
      }
      int maxSize = ( std::max )( ( std::max )( boundingBox.max_.x(), boundingBox.max_.y() ), boundingBox.max_.z() );
      const size_t w =
          ( maxSize + static_cast<int>( params.gridSize_ ) - 1 ) / ( static_cast<int>( params.gridSize_ ) );

      // identify boundary cells
      size_t           pointCount = reconstruct.getPointCount();
#if CELLID_UNORDERED_MAP
      std::unordered_map<size_t,int64_t> cellIndex;
#else
      std::vector<int> cellIndex;
      cellIndex.resize(w * w * w);
      std::fill(cellIndex.begin(), cellIndex.end(), -1);
#endif
      size_t    numBoundaryCells = 0;
      const int disth            = ( std::max )( static_cast<int>( params.gridSize_ ) / 2, 1 );
      const int th               = params.gridSize_ * w;
      for ( size_t n = 0; n < pointCount; ++n ) {
        if ( reconstruct.getBoundaryPointType( n ) == 1 ) {
          PCCVector3<int> P = reconstruct[n];
          if ( P[0] < disth || P[1] < disth || P[2] < disth || th <= P[0] + disth || th <= P[1] + disth ||
               th <= P[2] + disth ) {
            continue;
          }
          PCCVector3<int> P2 = P / params.gridSize_;
          PCCVector3<int> Q( P2[0] + ( ( P[0] % params.gridSize_ < params.gridSize_ / 2 ) ? -1 : 0 ),
                             P2[1] + ( ( P[1] % params.gridSize_ < params.gridSize_ / 2 ) ? -1 : 0 ),
                             P2[2] + ( ( P[2] % params.gridSize_ < params.gridSize_ / 2 ) ? -1 : 0 ) );
          for ( int ix = 0; ix < 2; ix++ ) {
            for ( int iy = 0; iy < 2; iy++ ) {
              for ( int iz = 0; iz < 2; iz++ ) {
#if CELLID_UNORDERED_MAP
                int64_t cellId = ((int64_t)Q[0] + ix) + ((int64_t)Q[1] + iy) * w + ((int64_t)Q[2] + iz) * w * w;
                if (cellIndex.find(cellId) == cellIndex.end()) {
#else
                int cellId = ( Q[0] + ix ) + ( Q[1] + iy ) * w + ( Q[2] + iz ) * w * w;
                if ( cellIndex[cellId] == -1 ) {
#endif
                  cellIndex[cellId] = numBoundaryCells;
                  numBoundaryCells++;
                }
              }
            }
          }
        }
      }
      geoSmoothingCenter_.resize( numBoundaryCells );
      geoSmoothingCount_.resize( numBoundaryCells );
      geoSmoothingPartition_.resize( numBoundaryCells );
      geoSmoothingDoSmooth_.resize( numBoundaryCells );
      int size = static_cast<int>( geoSmoothingCount_.size() );
      geoSmoothingCount_.resize( 0 );
      geoSmoothingCount_.resize( size, 0 );
      for ( int j = 0; j < reconstruct.getPointCount(); j++ ) {
        PCCPoint3D      point = reconstruct[j];
        PCCVector3<int> P     = point;
        if ( P[0] < disth || P[1] < disth || P[2] < disth || th <= P[0] + disth || th <= P[1] + disth ||
             th <= P[2] + disth ) {
          continue;
        }
        PCCVector3<int> P2     = point / params.gridSize_;
#if CELLID_UNORDERED_MAP
        int64_t             cellId = (int64_t)P2[0] + (int64_t)P2[1] * w + (int64_t)P2[2] * w * w;
        if (cellIndex.find(cellId) != cellIndex.end()) {
#else
        int             cellId = P2[0] + P2[1] * w + P2[2] * w * w;
        if ( cellIndex[cellId] != -1 ) {
#endif
          addGridCentroid( reconstruct[j], partition[j] + 1, geoSmoothingCount_, geoSmoothingCenter_,
                           geoSmoothingPartition_, geoSmoothingDoSmooth_, static_cast<int>( params.gridSize_ ), w,
                           cellIndex[cellId] );
        }
      }
      for ( int i = 0; i < geoSmoothingCount_.size(); i++ ) {
        if ( geoSmoothingCount_[i] != 0U ) { geoSmoothingCenter_[i] /= geoSmoothingCount_[i]; }
      }
      smoothPointCloudGrid( reconstruct, partition, params, w, cellIndex );
      cellIndex.clear();
    } else {
      if ( !params.pbfEnableFlag_ ) { smoothPointCloud( reconstruct, partition, params ); }
    }
  }
#ifdef CODEC_TRACE
  printChecksum( reconstruct, "smoothPointCloudPostprocess out" );
#endif
}

void PCCCodec::colorSmoothing( PCCPointSet3&                       reconstruct,
                               const PCCColorTransform             colorTransform,
                               const GeneratePointCloudParameters& params ) {
#if COLOR_SMOOTHING_ALIGNMENT
    PCCInt16Box3D boundingBox;
    boundingBox.min_ = boundingBox.max_ = reconstruct[0];
    for (int j = 0; j < reconstruct.getPointCount(); j++) {
        const PCCPoint3D point = reconstruct[j];
        for (size_t k = 0; k < 3; ++k) {
            if (point[k] < boundingBox.min_[k]) { boundingBox.min_[k] = floor(point[k]); }
            if (point[k] > boundingBox.max_[k]) { boundingBox.max_[k] = ceil(point[k]); }
        }
    }
    int pcMaxSize = (std::max)((std::max)(boundingBox.max_.x(), boundingBox.max_.y()), boundingBox.max_.z());
    const size_t     gridSize = params.cgridSize_;
    const size_t w =
        (pcMaxSize + static_cast<int>(gridSize) - 1) / (static_cast<int>(gridSize));
#else
  const size_t     gridSize         = params.occupancyPrecision_;
  int              pcMaxSize        = pow( 2, params.geometryBitDepth3D_ );
  const size_t     w = pcMaxSize / gridSize;
#endif
  const size_t     w3               = w * w * w;
  size_t           pointCount       = reconstruct.getPointCount();
  size_t           numBoundaryCells = 0;
  const size_t     disth            = ( std::max )( gridSize / 2, (size_t)1 );
#if CELLID_UNORDERED_MAP
  std::unordered_map<size_t,int64_t> cellIndex;
#else
  std::vector<int> cellIndex;
  cellIndex.resize( w3 );
  std::fill( cellIndex.begin(), cellIndex.end(), -1 );
#endif
  assert( params.flagColorSmoothing_ );
  TRACE_CODEC( "%s \n", "colorSmoothing" );
  TRACE_CODEC( "  geometryBitDepth3D_       = %zu \n", params.geometryBitDepth3D_ );
  TRACE_CODEC( "  thresholdColorVariation_  = %f \n", params.thresholdColorVariation_ );
  TRACE_CODEC( "  thresholdColorDifference_ = %f \n", params.thresholdColorDifference_ );
  TRACE_CODEC( "  pcMaxSize = %d \n", pcMaxSize );
  TRACE_CODEC( "  gridSize  = %zu \n", gridSize );
  TRACE_CODEC( "  w         = %zu \n", w );
  TRACE_CODEC( "  w3        = %zu \n", w3 );
  for ( size_t n = 0; n < pointCount; ++n ) {
    if ( reconstruct.getBoundaryPointType( n ) == 1 ) {
      PCCPoint3D      point = reconstruct[n];
      PCCVector3<int> P     = point;
      if ( P[0] < disth || P[1] < disth || P[2] < disth || pcMaxSize <= P[0] + disth || pcMaxSize <= P[1] + disth ||
           pcMaxSize <= P[2] + disth ) {
        continue;
      }
      PCCVector3<int> P2 = P / gridSize;
      PCCVector3<int> Q( P2[0] + ( ( P[0] % gridSize < gridSize / 2 ) ? -1 : 0 ),
                         P2[1] + ( ( P[1] % gridSize < gridSize / 2 ) ? -1 : 0 ),
                         P2[2] + ( ( P[2] % gridSize < gridSize / 2 ) ? -1 : 0 ) );
      for ( int ix = 0; ix < 2; ix++ ) {
        for ( int iy = 0; iy < 2; iy++ ) {
          for ( int iz = 0; iz < 2; iz++ ) {
#if CELLID_UNORDERED_MAP
              int64_t cellId = ((int64_t)Q[0] + ix) + ((int64_t)Q[1] + iy) * w + ((int64_t)Q[2] + iz) * w * w;
              if (cellIndex.find(cellId) == cellIndex.end()) {
#else
            int cellId = ( Q[0] + ix ) + ( Q[1] + iy ) * w + ( Q[2] + iz ) * w * w;
            if ( cellIndex[cellId] == -1 ) {
#endif
              cellIndex[cellId] = numBoundaryCells;
              numBoundaryCells++;
            }
          }
        }
      }
    }
  }
  colorSmoothingCenter_.resize( numBoundaryCells );
  colorSmoothingCount_.resize( numBoundaryCells );
  colorSmoothingPartition_.resize( numBoundaryCells );
  colorSmoothingDoSmooth_.resize( numBoundaryCells );
  std::fill( colorSmoothingCenter_.begin(), colorSmoothingCenter_.end(), 0.f );
  std::fill( colorSmoothingCount_.begin(), colorSmoothingCount_.end(), 0 );
  std::pair<size_t, size_t> initPair;
  initPair.first = initPair.second = 0;
  std::fill( colorSmoothingPartition_.begin(), colorSmoothingPartition_.end(), initPair );
  std::fill( colorSmoothingDoSmooth_.begin(), colorSmoothingDoSmooth_.end(), false );
  colorSmoothingLum_.clear();
  colorSmoothingLum_.resize( numBoundaryCells );
  for ( int k = 0; k < reconstruct.getPointCount(); k++ ) {
    PCCPoint3D      point  = reconstruct[k];
    PCCVector3<int> P2     = reconstruct[k] / gridSize;
#if CELLID_UNORDERED_MAP
    int64_t             cellId = (int64_t)P2[0] + (int64_t)P2[1] * w + (int64_t)P2[2] * w * w;
    if (cellId >= w3) {
        TRACE_CODEC(" cellId >  w3 <=>  %zu > %zu \n", cellId, w3);
    }
    else {
        if (cellIndex.find(cellId) != cellIndex.end()) {
#else
    int             cellId = P2[0] + P2[1] * w + P2[2] * w * w;
    if ( cellId >= cellIndex.size() ) {
      TRACE_CODEC( " cellId >  cellIndex.size() <=>  %zu > %zu \n", cellId, cellIndex.size() );
    } else {
      if ( cellIndex[cellId] != -1 ) {
#endif
        PCCVector3D clr                   = reconstruct.getColor16bit( k );
        auto        tilePatchIndexPlusOne = reconstruct.getPointPatchIndex( k );
        tilePatchIndexPlusOne.second      = tilePatchIndexPlusOne.second + 1;
        addGridColorCentroid(reconstruct[k], clr, tilePatchIndexPlusOne, colorSmoothingCount_, colorSmoothingCenter_,
            colorSmoothingPartition_, colorSmoothingDoSmooth_, gridSize, colorSmoothingLum_, params,
            cellIndex[cellId]);
      }
    }
  }
  smoothPointCloudColorLC( reconstruct, params, cellIndex );
  colorSmoothingCenter_.resize( 0 );
  colorSmoothingCenter_.shrink_to_fit();
  colorSmoothingCount_.resize( 0 );
  colorSmoothingCount_.shrink_to_fit();
  colorSmoothingPartition_.resize( 0 );
  colorSmoothingPartition_.shrink_to_fit();
  colorSmoothingDoSmooth_.resize( 0 );
  colorSmoothingDoSmooth_.shrink_to_fit();
  colorSmoothingLum_.resize( 0 );
  colorSmoothingLum_.shrink_to_fit();
}

int PCCCodec::getDeltaNeighbors( const PCCImageGeometry& frame,
                                 const PCCPatch&         patch,
                                 const int               xOrg,
                                 const int               yOrg,
                                 const int               neighboring,
                                 const int               threshold,
                                 const bool              projectionMode ) {
  int          deltaMax = 0;
  const double dOrg     = patch.generateNormalCoordinate( frame.getValue( 0, xOrg, yOrg ) );
#if PLR_FIX
  const int    x0 = (std::max)(0, xOrg - neighboring);
  const int    x1 = (std::min)(xOrg + neighboring, static_cast<int>(frame.getWidth()) - 1);
  const int    y0 = (std::max)(0, yOrg - neighboring);
  const int    y1 = (std::min)(yOrg + neighboring, static_cast<int>(frame.getHeight()) - 1);
#else
  const int    x0       = ( std::max )( 0, xOrg - neighboring );
  const int    x1       = ( std::min )( xOrg + neighboring, static_cast<int>( frame.getWidth() ) );
  const int    y0       = ( std::max )( 0, yOrg - neighboring );
  const int    y1       = ( std::min )( yOrg + neighboring, static_cast<int>( frame.getHeight() ) );
#endif
  for ( int x = x0; x <= x1; x += 1 ) {
    for ( int y = y0; y <= y1; y += 1 ) {
      double dLoc  = patch.generateNormalCoordinate( frame.getValue( 0, x, y ) );
      int    delta = static_cast<int>( dLoc - dOrg );
      if ( patch.getProjectionMode() == 0 ) {
        if ( delta <= threshold && delta > deltaMax ) { deltaMax = delta; }
      } else {
        if ( delta >= -threshold && delta < deltaMax ) { deltaMax = delta; }
      }
    }
  }
  deltaMax = deltaMax == 0 ? deltaMax : ( patch.getProjectionMode() == 0 ? deltaMax - 1 : deltaMax + 1 );
  return deltaMax;
}

void PCCCodec::identifyBoundaryPoints( const std::vector<uint32_t>& occupancyMap,
                                       const size_t                 x,
                                       const size_t                 y,
                                       const size_t                 imageWidth,
                                       const size_t                 imageHeight,
                                       const size_t                 pointindex,
                                       std::vector<uint32_t>&       BPflag,
                                       PCCPointSet3&                reconstruct ) {
  if ( occupancyMap[y * imageWidth + x] != 0 ) {
    if ( y > 0 && y < imageHeight - 1 ) {
      if ( occupancyMap[( y - 1 ) * imageWidth + x] == 0 || occupancyMap[( y + 1 ) * imageWidth + x] == 0 ) {
        BPflag[y * imageWidth + x] = 1;
        reconstruct.setBoundaryPointType( pointindex, static_cast<uint16_t>( 1 ) );
      }
    }
    if ( x > 0 && x < imageWidth - 1 && reconstruct.getBoundaryPointType( pointindex ) != 1 ) {
      if ( occupancyMap[y * imageWidth + ( x + 1 )] == 0 || occupancyMap[y * imageWidth + ( x - 1 )] == 0 ) {
        BPflag[y * imageWidth + x] = 1;
        reconstruct.setBoundaryPointType( pointindex, static_cast<uint16_t>( 1 ) );
      }
    }
    if ( y > 0 && y < imageHeight - 1 && x > 0 && reconstruct.getBoundaryPointType( pointindex ) != 1 ) {
      if ( occupancyMap[( y - 1 ) * imageWidth + ( x - 1 )] == 0 ||
           occupancyMap[( y + 1 ) * imageWidth + ( x - 1 )] == 0 ) {
        BPflag[y * imageWidth + x] = 1;
        reconstruct.setBoundaryPointType( pointindex, static_cast<uint16_t>( 1 ) );
      }
    }
    if ( y > 0 && y < imageHeight - 1 && x < imageWidth - 1 && reconstruct.getBoundaryPointType( pointindex ) != 1 ) {
      if ( occupancyMap[( y - 1 ) * imageWidth + ( x + 1 )] == 0 ||
           occupancyMap[( y + 1 ) * imageWidth + ( x + 1 )] == 0 ) {
        BPflag[y * imageWidth + x] = 1;
        reconstruct.setBoundaryPointType( pointindex, static_cast<uint16_t>( 1 ) );
      }
    }
    if ( y == 0 || y == imageHeight - 1 || x == 0 || x == imageWidth - 1 ) {
      BPflag[y * imageWidth + x] = 1;
      reconstruct.setBoundaryPointType( pointindex, static_cast<uint16_t>( 1 ) );
    }
  }
  ////////////// second layer
  if ( ( occupancyMap[y * imageWidth + x] != 0U ) && reconstruct.getBoundaryPointType( pointindex ) != 1 ) {
    for ( int ix = -2; ix <= 2; ++ix ) {
      for ( int iy = -2; iy <= 2; ++iy ) {
        if ( abs( int( ix ) ) > 1 || abs( int( iy ) ) > 1 ) {
          if ( ( y + iy ) >= 0 && ( y + iy ) < imageHeight && ( x + ix ) >= 0 && ( x + ix ) < imageWidth ) {
            if ( occupancyMap[( y + iy ) * imageWidth + ( x + ix )] == 0 ) {
              reconstruct.setBoundaryPointType( pointindex, static_cast<uint16_t>( 1 ) );
              ix = 4;
              iy = 4;
            }
          }
        }
      }
    }
    if ( y == 1 || y == imageHeight - 2 || x == 1 || x == imageWidth - 2 ) {
      reconstruct.setBoundaryPointType( pointindex, static_cast<uint16_t>( 1 ) );
    }
  }
}

std::vector<PCCPoint3D> PCCCodec::generatePoints( const GeneratePointCloudParameters&  params,
                                                  PCCFrameContext&                     tile,
                                                  const std::vector<PCCVideoGeometry>& videoGeometryMultiple,
                                                  const size_t                         videoFrameIndex,
                                                  const size_t                         patchIndex,
                                                  const size_t                         u,
                                                  const size_t                         v,
                                                  const size_t                         x,
                                                  const size_t                         y,
                                                  const bool                           interpolate,
                                                  const bool                           filling,
                                                  const size_t                         minD1,
                                                  const size_t                         neighbor ) {
  const auto&             patch  = tile.getPatch( patchIndex );
  auto&                   frame0 = videoGeometryMultiple[0].getFrame( videoFrameIndex );
  std::vector<PCCPoint3D> createdPoints;
  PCCPoint3D              point0;
  if ( params.pbfEnableFlag_ ) {
    point0 = patch.generatePoint( u, v, patch.getDepthMap( u, v ) );
  } else {
    point0 = patch.generatePoint( u, v, frame0.getValue( 0, x, y ) );
  }
  createdPoints.push_back( point0 );
  if ( params.singleMapPixelInterleaving_ ) {
    size_t     patchIndexPlusOne = patchIndex + 1;
    double     depth0;
    double     depth1;
    const auto imageWidth  = frame0.getWidth();
    const auto imageHeight = frame0.getHeight();
    bool       occupancyTop;
    bool       occupancyBotton;
    bool       occupancyLeft;
    bool       occupancyRight;
    if ( !params.pbfEnableFlag_ ) {
      auto& occupancyMap = tile.getOccupancyMap();
      occupancyTop       = y > 0 && ( occupancyMap[( y - 1 ) * imageWidth + x] != 0U );
      occupancyBotton    = y < ( imageHeight - 1 ) && ( occupancyMap[( y + 1 ) * imageWidth + x] != 0U );
      occupancyLeft      = x > 0 && ( occupancyMap[y * imageWidth + x - 1] != 0U );
      occupancyRight     = x < ( imageWidth - 1 ) && ( occupancyMap[y * imageWidth + x + 1] != 0U );
    } else {
      auto res        = patch.getOccupancyResolution();
      occupancyTop    = y > 0 && ( patch.getOccupancyMap( ( x ) / res, ( y - 1 ) / res ) != 0 );
      occupancyBotton = y < ( imageHeight - 1 ) && ( patch.getOccupancyMap( ( x ) / res, ( y + 1 ) / res ) != 0 );
      occupancyLeft   = x > 0 && ( patch.getOccupancyMap( ( x - 1 ) / res, ( y ) / res ) != 0 );
      occupancyRight  = x < ( imageWidth - 1 ) && ( patch.getOccupancyMap( ( x + 1 ) / res, ( y ) / res ) != 0 );
    }
    auto&        blockToPatch      = tile.getBlockToPatch();
    const size_t blockToPatchWidth = tile.getWidth() / params.occupancyResolution_;
    double       depthNeighbors[4] = {0.0, 0.0, 0.0, 0.0};
    int          count             = 0;
    double       minimumDepth      = point0[patch.getNormalAxis()];
    double       maximumDepth      = point0[patch.getNormalAxis()];
    if ( occupancyLeft ) {
      size_t Temp_u0 = ( x - 1 ) / patch.getOccupancyResolution();
      size_t Temp_v0 = ( y ) / patch.getOccupancyResolution();
      if ( blockToPatch[(Temp_v0)*blockToPatchWidth + Temp_u0] == patchIndexPlusOne ) {
        if ( patch.getProjectionMode() == 0 ) {
          depthNeighbors[0] = double( frame0.getValue( 0, x - 1, y ) + patch.getD1() );
        } else {
          depthNeighbors[0] = double( patch.getD1() - frame0.getValue( 0, x - 1, y ) );
        }
        count++;
        if ( depthNeighbors[0] < minimumDepth ) { minimumDepth = depthNeighbors[0]; }
        if ( depthNeighbors[0] > maximumDepth ) { maximumDepth = depthNeighbors[0]; }
      }
    }
    if ( occupancyRight ) {
      size_t Temp_u0 = ( x + 1 ) / patch.getOccupancyResolution();
      size_t Temp_v0 = ( y ) / patch.getOccupancyResolution();
      if ( blockToPatch[(Temp_v0)*blockToPatchWidth + Temp_u0] == patchIndexPlusOne ) {
        if ( patch.getProjectionMode() == 0 ) {
          depthNeighbors[1] = double( frame0.getValue( 0, x + 1, y ) + patch.getD1() );
        } else {
          depthNeighbors[1] = double( patch.getD1() - frame0.getValue( 0, x + 1, y ) );
        }
        count++;
        if ( depthNeighbors[1] < minimumDepth ) { minimumDepth = depthNeighbors[1]; }
        if ( depthNeighbors[1] > maximumDepth ) { maximumDepth = depthNeighbors[1]; }
      }
    }
    if ( occupancyTop ) {
      size_t Temp_u0 = ( x ) / patch.getOccupancyResolution();
      size_t Temp_v0 = ( y - 1 ) / patch.getOccupancyResolution();
      if ( blockToPatch[(Temp_v0)*blockToPatchWidth + Temp_u0] == patchIndexPlusOne ) {
        if ( patch.getProjectionMode() == 0 ) {
          depthNeighbors[2] = double( frame0.getValue( 0, x, y - 1 ) + patch.getD1() );
        } else {
          depthNeighbors[2] = double( patch.getD1() - frame0.getValue( 0, x, y - 1 ) );
        }
        count++;
        if ( depthNeighbors[2] < minimumDepth ) { minimumDepth = depthNeighbors[2]; }
        if ( depthNeighbors[2] > maximumDepth ) { maximumDepth = depthNeighbors[2]; }
      }
    }
    if ( occupancyBotton ) {
      size_t Temp_u0 = ( x ) / patch.getOccupancyResolution();
      size_t Temp_v0 = ( y + 1 ) / patch.getOccupancyResolution();
      if ( blockToPatch[(Temp_v0)*blockToPatchWidth + Temp_u0] == patchIndexPlusOne ) {
        if ( patch.getProjectionMode() == 0 ) {
          depthNeighbors[3] = double( frame0.getValue( 0, x, y + 1 ) + patch.getD1() );
        } else {
          depthNeighbors[3] = double( patch.getD1() - frame0.getValue( 0, x, y + 1 ) );
        }
        count++;
        if ( depthNeighbors[3] < minimumDepth ) { minimumDepth = depthNeighbors[3]; }
        if ( depthNeighbors[3] > maximumDepth ) { maximumDepth = depthNeighbors[3]; }
      }
    }
    if ( count == 0 ) { return createdPoints; }
    if ( ( x + y ) % 2 == 1 ) {
      depth1 = point0[patch.getNormalAxis()];
      PCCPoint3D interpolateD0( point0 );
      if ( patch.getProjectionMode() == 0 ) {
        interpolateD0[patch.getNormalAxis()] =
            round( ( std::min )( ( std::max )( minimumDepth, depth1 - params.surfaceThickness_ ), depth1 ) );
      } else {
        interpolateD0[patch.getNormalAxis()] =
            round( ( std::max )( ( std::min )( maximumDepth, depth1 + params.surfaceThickness_ ), depth1 ) );
      }
      depth0 = interpolateD0[patch.getNormalAxis()];
      createdPoints.push_back( interpolateD0 );
    } else {
      depth0 = point0[patch.getNormalAxis()];
      PCCPoint3D interpolateD1( point0 );
      if ( patch.getProjectionMode() == 0 ) {
        interpolateD1[patch.getNormalAxis()] = round( ( std::max )(
            ( std::min )( ( depthNeighbors[0] + depthNeighbors[1] + depthNeighbors[2] + depthNeighbors[3] ) / count,
                          depth0 + params.surfaceThickness_ ),
            depth0 ) );
      } else {
        interpolateD1[patch.getNormalAxis()] = round( ( std::min )(
            ( std::max )( ( depthNeighbors[0] + depthNeighbors[1] + depthNeighbors[2] + depthNeighbors[3] ) / count,
                          depth0 - params.surfaceThickness_ ),
            depth0 ) );
      }
      depth1 = interpolateD1[patch.getNormalAxis()];
      createdPoints.push_back( interpolateD1 );
    }
    size_t xmin = ( std::min )( depth0, depth1 );
    size_t xmax = ( std::max )( depth0, depth1 );
    for ( size_t step = 1; step < ( xmax - xmin ); ++step ) {
      PCCPoint3D fillPoint( point0 );
      fillPoint[patch.getNormalAxis()] = xmin + double( step );
      createdPoints.push_back( fillPoint );
    }
  } else if ( params.pointLocalReconstruction_ ) {
    int deltaDepth = 0;
    if ( interpolate ) {
      deltaDepth =
          getDeltaNeighbors( frame0, patch, x, y, neighbor, g_neighborThreshold, patch.getProjectionMode() != 0U );
    }
    if ( patch.getProjectionMode() == 0 ) {
      deltaDepth = ( std::max )( deltaDepth, static_cast<int>( minD1 ) );
    } else {
      deltaDepth = ( std::min )( deltaDepth, -static_cast<int>( minD1 ) );
    }
    if ( deltaDepth != 0 ) {
      PCCPoint3D point1( point0 );
      point1[patch.getNormalAxis()] += static_cast<double>( deltaDepth );
      createdPoints.push_back( point1 );
      if ( filling ) {
        size_t xmin = ( std::min )( point0[patch.getNormalAxis()], point1[patch.getNormalAxis()] );
        size_t xmax = ( std::max )( point0[patch.getNormalAxis()], point1[patch.getNormalAxis()] );
        for ( size_t value = xmin + 1; value < xmax; ++value ) {
          PCCPoint3D pointFill( point0 );
          pointFill[patch.getNormalAxis()] = double( value );
          createdPoints.push_back( pointFill );
        }
      }
    }
  } else {  // not singleMapPixelInterleaving_ && not pointLocalReconstruction_
    if ( params.mapCountMinus1_ > 0 ) {
      PCCPoint3D  point1( point0 );
      const auto& frame1 = params.multipleStreams_ ? videoGeometryMultiple[1].getFrame( videoFrameIndex )
                                                   : videoGeometryMultiple[0].getFrame( 1 + videoFrameIndex );
      if ( params.absoluteD1_ ) {
        point1 = patch.generatePoint( u, v, frame1.getValue( 0, x, y ) );
      } else {
        if ( patch.getProjectionMode() == 0 ) {
          point1[patch.getNormalAxis()] += frame1.getValue( 0, x, y );
        } else {
          point1[patch.getNormalAxis()] -= frame1.getValue( 0, x, y );
        }
      }
      createdPoints.push_back( point1 );
    }  // if ( params.mapCountMinus1_ > 0 ) {
  }    // fi (pointLocalReconstruction)
  return createdPoints;
}

void PCCCodec::generatePointCloud( PCCPointSet3&                       reconstruct,
                                   PCCContext&                         context,
                                   size_t                              frameIndex,
                                   size_t                              tileIndex,
                                   const GeneratePointCloudParameters& params,
                                   std::vector<uint32_t>&              partition,
                                   bool                                bDecoder ) {
  TRACE_CODEC( "generatePointCloud F = %zu start \n", frameIndex );
  auto&        tile                  = context[frameIndex].getTile( tileIndex );
  auto&        videoGeometry         = context.getVideoGeometryMultiple()[0];
  auto&        videoGeometryMultiple = context.getVideoGeometryMultiple();
  auto&        videoOccupancyMap     = context.getVideoOccupancyMap();
  auto&        patches               = tile.getPatches();
  auto&        pointToPixel          = tile.getPointToPixel();
  auto&        blockToPatch          = tile.getBlockToPatch();
  const size_t blockToPatchWidth     = tile.getWidth() / params.occupancyResolution_;
  const size_t blockToPatchHeight    = tile.getHeight() / params.occupancyResolution_;
  const size_t totalPatchCount       = patches.size();
  uint32_t     patchIndex            = 0;
  const size_t mapCount              = params.mapCountMinus1_ + 1;
  reconstruct.addColors();
  printf( "generatePointCloud pbfEnableFlag_ = %d \n", params.pbfEnableFlag_ );
  fflush( stdout );
  TRACE_CODEC( "generatePointCloud pbfEnableFlag_ = %d \n", params.pbfEnableFlag_ );
  if ( params.pbfEnableFlag_ ) {
    size_t              frameIndex = params.multipleStreams_ ? tile.getFrameIndex() : tile.getFrameIndex() * mapCount;
    PatchBlockFiltering patchBlockFiltering;
    patchBlockFiltering.setPatches( &( tile.getPatches() ) );
    patchBlockFiltering.setBlockToPatch( &( tile.getBlockToPatch() ) );
    patchBlockFiltering.setOccupancyMapEncoder( &( tile.getOccupancyMap() ) );
    patchBlockFiltering.setOccupancyMapVideo( &( videoOccupancyMap.getFrame( tile.getFrameIndex() ).getChannel( 0 ) ) );
    patchBlockFiltering.setGeometryVideo( &( videoGeometry.getFrame( frameIndex ).getChannel( 0 ) ) );
    patchBlockFiltering.patchBorderFiltering( tile.getWidth(), tile.getHeight(), params.occupancyResolution_,
                                              params.occupancyPrecision_,
                                              !params.enhancedOccupancyMapCode_ ? params.thresholdLossyOM_ : 0,
                                              params.pbfPassesCount_, params.pbfFilterSize_, params.pbfLog2Threshold_ );
    TRACE_CODEC( "%s \n", "PBF done" );
  }

  // point cloud occupancy map upscaling from video using nearest neighbor
  auto& occupancyMap = tile.getOccupancyMap();
  if ( !params.pbfEnableFlag_ ) {
    auto width  = tile.getWidth();
    auto height = tile.getHeight();
    occupancyMap.resize( width * height, 0 );
    for ( size_t v = 0; v < height; ++v ) {
      for ( size_t u = 0; u < width; ++u ) {
        occupancyMap[v * width + u] =
            videoOccupancyMap.getFrame( tile.getFrameIndex() )
                .getValue( 0, tile.getLeftTopXInFrame() / params.occupancyPrecision_ + u / params.occupancyPrecision_,
                           tile.getLeftTopYInFrame() / params.occupancyPrecision_ + v / params.occupancyPrecision_ );
      }
    }
  }
  if ( params.enableSizeQuantization_ ) {
    size_t quantizerSizeX = ( size_t( 1 ) << tile.getLog2PatchQuantizerSizeX() );
    size_t quantizerSizeY = ( size_t( 1 ) << tile.getLog2PatchQuantizerSizeY() );
    for ( size_t patchIndex = 0; patchIndex < totalPatchCount; ++patchIndex ) {
      auto&  patch             = patches[patchIndex];
      size_t patchSizeXInPixel = ( patch.getPatchSize2DXInPixel() / quantizerSizeX ) * quantizerSizeX;
      size_t patchSizeYInPixel = ( patch.getPatchSize2DYInPixel() / quantizerSizeY ) * quantizerSizeY;
      if ( tile.getLog2PatchQuantizerSizeX() == 0 ) { assert( patchSizeXInPixel == patch.getPatchSize2DXInPixel() ); }
      if ( tile.getLog2PatchQuantizerSizeY() == 0 ) { assert( patchSizeYInPixel == patch.getPatchSize2DYInPixel() ); }
      for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
        for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
          const size_t blockIndex = patch.patchBlock2CanvasBlock( u0, v0, blockToPatchWidth, blockToPatchHeight );
          if ( blockToPatch[blockIndex] == ( patchIndex + 1 ) ) {
            for ( size_t v1 = 0; v1 < patch.getOccupancyResolution(); ++v1 ) {
              const size_t v = v0 * patch.getOccupancyResolution() + v1;
              for ( size_t u1 = 0; u1 < patch.getOccupancyResolution(); ++u1 ) {
                const size_t u = u0 * patch.getOccupancyResolution() + u1;
                if ( u >= patchSizeXInPixel || v >= patchSizeYInPixel ) {
                  occupancyMap[patch.patch2Canvas( u, v, tile.getWidth(), tile.getHeight() )] = 0;
                }
              }  // u1
            }    // v1
          }      // patchidx+1==block2patch
        }        // u0
      }          // v0
    }
  }
  // partition.resize( 0 );
  pointToPixel.resize( 0 );
  reconstruct.clear();

  TRACE_CODEC( " Frame %zu in generatePointCloud \n", tile.getFrameIndex() );
  TRACE_CODEC( " params.useAdditionalPointsPatch = %d \n", params.useAdditionalPointsPatch_ );
  TRACE_CODEC( " params.enhancedOccupancyMapCode   = %d \n", params.enhancedOccupancyMapCode_ );

  bool   useRawPointsSeparateVideo = tile.getUseRawPointsSeparateVideo();
  size_t videoFrameIndex;
  TRACE_CODEC( " mapCount                       = %d \n", mapCount );
  if ( params.multipleStreams_ ) {
    videoFrameIndex = tile.getFrameIndex();
    if ( videoGeometryMultiple[0].getFrameCount() < ( videoFrameIndex + 1 ) ) { return; }
  } else {
    videoFrameIndex = tile.getFrameIndex() * mapCount;
    if ( videoGeometry.getFrameCount() < ( videoFrameIndex + mapCount ) ) { return; }
  }
  TRACE_CODEC( " videoFrameIndex(shift):frameIndex*mapCount  = %d \n", videoFrameIndex );
  const auto& frame0 = params.multipleStreams_ ? videoGeometryMultiple[0].getFrame( videoFrameIndex )
                                               : videoGeometry.getFrame( videoFrameIndex );
  const size_t          tileWidth  = tile.getWidth();
  const size_t          tileHeight = tile.getHeight();
  std::vector<uint32_t> BPflag;
  if ( !params.pbfEnableFlag_ ) { BPflag.resize( tileWidth * tileHeight, 0 ); }

  std::vector<std::vector<PCCPoint3D>> eomPointsPerPatch;
  eomPointsPerPatch.resize( totalPatchCount );
  uint32_t   index;
  const bool patchPrecedenceOrderFlag = context.getAtlasSequenceParameterSet( 0 ).getPatchPrecedenceOrderFlag();
  for ( index = 0; index < patches.size(); index++ ) {
    patchIndex                     = ( bDecoder && patchPrecedenceOrderFlag ) ? ( totalPatchCount - index - 1 ) : index;
    const size_t patchIndexPlusOne = patchIndex + 1;
    auto&        patch             = patches[patchIndex];
    PCCColor3B   color( uint8_t( 0 ) );
    TRACE_CODEC(
        "P%2lu/%2lu: 2D=(%2lu,%2lu)*(%2lu,%2lu) 3D(%4zu,%4zu,%4zu)*(%4zu,%4zu) A=(%zu,%zu,%zu) Or=%zu P=%zu => %zu "
        "AxisOfAdditionalPlane = %zu \n",
        patchIndex, totalPatchCount, patch.getU0(), patch.getV0(), patch.getSizeU0(), patch.getSizeV0(), patch.getU1(),
        patch.getV1(), patch.getD1(), patch.getSizeU0() * patch.getOccupancyResolution(),
        patch.getSizeV0() * patch.getOccupancyResolution(), patch.getNormalAxis(), patch.getTangentAxis(),
        patch.getBitangentAxis(), patch.getPatchOrientation(), patch.getProjectionMode(), reconstruct.getPointCount(),
        patch.getAxisOfAdditionalPlane() );

    while ( color[0] == color[1] || color[2] == color[1] || color[2] == color[0] ) {
      color[0] = static_cast<uint8_t>( rand() % 32 ) * 8;
      color[1] = static_cast<uint8_t>( rand() % 32 ) * 8;
      color[2] = static_cast<uint8_t>( rand() % 32 ) * 8;
    }
    for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
      for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
        const size_t blockIndex = patch.patchBlock2CanvasBlock( u0, v0, blockToPatchWidth, blockToPatchHeight );
        if ( blockToPatch[blockIndex] == patchIndexPlusOne ) {
          for ( size_t v1 = 0; v1 < patch.getOccupancyResolution(); ++v1 ) {
            const size_t v = v0 * patch.getOccupancyResolution() + v1;
            for ( size_t u1 = 0; u1 < patch.getOccupancyResolution(); ++u1 ) {
              const size_t u = u0 * patch.getOccupancyResolution() + u1;
              size_t       x;
              size_t       y;
              bool         occupancy     = false;
              size_t       canvasIndex   = patch.patch2Canvas( u, v, tileWidth, tileHeight, x, y );
              size_t       xInVideoFrame = x + tile.getLeftTopXInFrame();
              size_t       yInVideoFrame = y + tile.getLeftTopYInFrame();
              bool         isBoundary    = false;
              if ( params.pbfEnableFlag_ ) {
                occupancy = patch.getOccupancyMap( u, v ) != 0;
                if ( occupancy ) { isBoundary = patch.isBorder( u, v ); }
              } else {
                occupancy = occupancyMap[canvasIndex] != 0;
              }
              if ( !occupancy ) { continue; }
              if ( params.enhancedOccupancyMapCode_ ) {
                // D0
                PCCPoint3D point0 = patch.generatePoint( u, v, frame0.getValue( 0, xInVideoFrame, yInVideoFrame ) );
                size_t     pointIndex0;  // = reconstruct.addPoint(point0);
                if ( patch.getAxisOfAdditionalPlane() == 0 ) {
                  pointIndex0 = reconstruct.addPoint( point0 );
                } else {
                  PCCVector3D tmp;
                  inverseRotatePosition45DegreeOnAxis( patch.getAxisOfAdditionalPlane(), params.geometryBitDepth3D_,
                                                       point0, tmp );
                  pointIndex0 = reconstruct.addPoint( tmp );
                }
                reconstruct.setPointPatchIndex( pointIndex0, tileIndex, patchIndex );
                reconstruct.setColor( pointIndex0, color );
                if ( PCC_SAVE_POINT_TYPE == 1 ) { reconstruct.setType( pointIndex0, POINT_D0 ); }
                partition.push_back( uint32_t( patchIndex ) );
                pointToPixel.emplace_back( x, y, 0 );
                uint16_t    eomCode = 0;
                size_t      d1pos   = 0;
                const auto& frame0  = params.multipleStreams_ ? videoGeometryMultiple[0].getFrame( videoFrameIndex )
                                                             : videoGeometry.getFrame( videoFrameIndex );
                const auto& indx = patch.patch2Canvas( u, v, tileWidth, tileHeight, x, y );
                if ( params.mapCountMinus1_ > 0 ) {
                  const auto& frame1 = params.multipleStreams_ ? videoGeometryMultiple[1].getFrame( videoFrameIndex )
                                                               : videoGeometry.getFrame( videoFrameIndex + 1 );
                  int16_t diff = params.absoluteD1_
                                     ? ( static_cast<int16_t>( frame1.getValue( 0, xInVideoFrame, yInVideoFrame ) ) -
                                         static_cast<int16_t>( frame0.getValue( 0, xInVideoFrame, yInVideoFrame ) ) )
                                     : static_cast<int16_t>( frame1.getValue( 0, xInVideoFrame, yInVideoFrame ) );
                  assert( diff >= 0 );
                  // Convert occupancy map to eomCode
                  if ( diff == 0 ) {
                    eomCode = 0;
                  } else if ( diff == 1 ) {
                    d1pos   = 1;
                    eomCode = 1;
                  } else if ( diff > 0 ) {
                    uint16_t bits = diff - 1;
                    uint16_t symbol =
                        ( 1 << bits ) - occupancyMap[patch.patch2Canvas( u, v, tileWidth, tileHeight, x, y )];
                    eomCode = symbol | ( 1 << bits );
                    d1pos   = ( bits );
                  }
                } else {  // params.mapCountMinus1_ == 0
                  eomCode = ( 1 << params.EOMFixBitCount_ ) - occupancyMap[indx];
                }
                PCCPoint3D point1( point0 );
                if ( eomCode == 0 ) {
                  if ( !params.removeDuplicatePoints_ ) {
                    size_t pointIndex1;  // = reconstruct.addPoint(point1);
                    if ( patch.getAxisOfAdditionalPlane() == 0 ) {
                      pointIndex1 = reconstruct.addPoint( point1 );
                    } else {
                      PCCVector3D tmp;
                      inverseRotatePosition45DegreeOnAxis( patch.getAxisOfAdditionalPlane(), params.geometryBitDepth3D_,
                                                           point1, tmp );
                      pointIndex1 = reconstruct.addPoint( tmp );
                    }
                    reconstruct.setPointPatchIndex( pointIndex1, tileIndex, patchIndex );
                    reconstruct.setColor( pointIndex1, color );
                    if ( PCC_SAVE_POINT_TYPE == 1 ) { reconstruct.setType( pointIndex1, POINT_D1 ); }
                    partition.push_back( uint32_t( patchIndex ) );
                    pointToPixel.emplace_back( x, y, 1 );
                  }
                } else {  // eomCode != 0
                  uint16_t addedPointCount = 0;
                  size_t   pointIndex1     = 0;
                  for ( uint16_t i = 0; i < 10; i++ ) {
                    if ( ( eomCode & ( 1 << i ) ) != 0 ) { d1pos = i; }
                  }
                  for ( uint16_t i = 0; i < 10; i++ ) {
                    if ( ( eomCode & ( 1 << i ) ) != 0 ) {
                      uint8_t deltaDCur = ( i + 1 );
                      if ( patch.getProjectionMode() == 0 ) {
                        point1[patch.getNormalAxis()] =
                            static_cast<double>( point0[patch.getNormalAxis()] + deltaDCur );
                      } else {
                        point1[patch.getNormalAxis()] =
                            static_cast<double>( point0[patch.getNormalAxis()] - deltaDCur );
                      }
                      if ( ( eomCode == 1 || i == d1pos ) && ( params.mapCountMinus1_ > 0 ) ) {  // d1
                        if ( patch.getAxisOfAdditionalPlane() == 0 ) {
                          pointIndex1 = reconstruct.addPoint( point1 );
                        } else {
                          PCCVector3D tmp;
                          inverseRotatePosition45DegreeOnAxis( patch.getAxisOfAdditionalPlane(),
                                                               params.geometryBitDepth3D_, point1, tmp );
                          pointIndex1 = reconstruct.addPoint( tmp );
                        }
                        reconstruct.setPointPatchIndex( pointIndex1, tileIndex, patchIndex );
                        reconstruct.setColor( pointIndex1, color );
                        if ( PCC_SAVE_POINT_TYPE == 1 ) { reconstruct.setType( pointIndex1, POINT_D1 ); }
                        partition.push_back( uint32_t( patchIndex ) );
                        pointToPixel.emplace_back( x, y, 1 );
                      } else {
                        if ( patch.getAxisOfAdditionalPlane() == 0 ) {
                          eomPointsPerPatch[patchIndex].push_back( point1 );
                        } else {
                          PCCVector3D tmp;
                          inverseRotatePosition45DegreeOnAxis( patch.getAxisOfAdditionalPlane(),
                                                               params.geometryBitDepth3D_, point1, tmp );
                          eomPointsPerPatch[patchIndex].push_back( PCCPoint3D( tmp[0], tmp[1], tmp[2] ) );
                        }
                      }
                      addedPointCount++;
                    }
                  }  // for each bit of EOM code
                  if ( PCC_SAVE_POINT_TYPE == 1 ) { reconstruct.setType( pointIndex1, POINT_D1 ); }
                  // Without "Identify boundary points" & "1st Extension boundary region" as EOM code is only for
                  // lossless coding now
                }       // if (eomCode == 0)
              } else {  // not params.enhancedOccupancyMapCode_
                std::vector<PCCPoint3D> createdPoints;
                if ( params.pointLocalReconstruction_ ) {
                  auto& mode =
                      context.getPointLocalReconstructionMode( patch.getPointLocalReconstructionMode( u0, v0 ) );
                  createdPoints = generatePoints( params, tile, videoGeometryMultiple, videoFrameIndex, patchIndex, u,
                                                  v, xInVideoFrame, yInVideoFrame, mode.interpolate_, mode.filling_,
                                                  mode.minD1_, mode.neighbor_ );
                } else {
                  createdPoints = generatePoints( params, tile, videoGeometryMultiple, videoFrameIndex, patchIndex, u,
                                                  v, xInVideoFrame, yInVideoFrame );
                }
                if ( !createdPoints.empty() ) {
                  for ( size_t i = 0; i < createdPoints.size(); i++ ) {
                    if ( ( !params.removeDuplicatePoints_ ) ||
                         ( ( i == 0 ) || ( createdPoints[i] != createdPoints[0] ) ) ) {
                      size_t pointindex = 0;
                      if ( patch.getAxisOfAdditionalPlane() == 0 ) {
                        pointindex = reconstruct.addPoint( createdPoints[i] );
                        reconstruct.setPointPatchIndex( pointindex, tileIndex, patchIndex );
                      } else {
                        PCCVector3D tmp;
                        inverseRotatePosition45DegreeOnAxis( patch.getAxisOfAdditionalPlane(),
                                                             params.geometryBitDepth3D_, createdPoints[i], tmp );
                        pointindex = reconstruct.addPoint( tmp );
                        reconstruct.setPointPatchIndex( pointindex, tileIndex, patchIndex );
                      }
                      const size_t pointindex_1 = pointindex;
                      reconstruct.setColor( pointindex_1, color );
                      if ( params.pbfEnableFlag_ ) { reconstruct.setBoundaryPointType( pointindex_1, isBoundary ); }
                      if ( PCC_SAVE_POINT_TYPE == 1 ) {
                        if ( params.singleMapPixelInterleaving_ ) {
                          size_t flag;
                          flag = ( i == 0 ) ? ( x + y ) % 2 : ( i == 1 ) ? ( x + y + 1 ) % 2 : g_intermediateLayerIndex;
                          reconstruct.setType( pointindex_1, flag == 0 ? POINT_D0 : flag == 1 ? POINT_D1 : POINT_DF );
                        } else {
                          reconstruct.setType( pointindex_1, i == 0 ? POINT_D0 : i == 1 ? POINT_D1 : POINT_DF );
                        }
                      }
                      partition.push_back( uint32_t( patchIndex ) );
                      if ( params.singleMapPixelInterleaving_ ) {
                        pointToPixel.emplace_back(
                            x, y,
                            i == 0 ? ( static_cast<size_t>( x + y ) % 2 )
                                   : i == 1 ? ( static_cast<size_t>( x + y + 1 ) % 2 ) : g_intermediateLayerIndex );
                      } else if ( params.pointLocalReconstruction_ ) {
                        pointToPixel.emplace_back(
                            x, y, i == 0 ? 0 : i == 1 ? g_intermediateLayerIndex : g_intermediateLayerIndex + 1 );
                      } else {
                        pointToPixel.emplace_back( x, y, i < 2 ? i : g_intermediateLayerIndex + 1 );
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  tile.setTotalNumberOfRegularPoints( reconstruct.getPointCount() );
  printf( "frame %zu, tile %zu: regularPoints %zu\n", frameIndex, tileIndex, reconstruct.getPointCount() );
  patchIndex                         = index;
  size_t       totalEOMPointsInFrame = 0;
  PCCPointSet3 eomSavedPoints;
  if ( params.enhancedOccupancyMapCode_ ) {
    const size_t blockSize     = params.occupancyResolution_ * params.occupancyResolution_;
    size_t       numEOMPatches = tile.getEomPatches().size();
    for ( int j = 0; j < numEOMPatches; j++ ) {
      auto&  eomPatch               = tile.getEomPatch( j );
      size_t numPatchesInEOMPatches = eomPatch.memberPatches_.size();
      size_t u0Eom                  = useRawPointsSeparateVideo ? 0 : eomPatch.u0_ * params.occupancyResolution_;
      size_t v0Eom                  = useRawPointsSeparateVideo ? 0 : eomPatch.v0_ * params.occupancyResolution_;
      totalEOMPointsInFrame += eomPatch.eomCount_;
      size_t totalPointCount = 0;
      for ( size_t patchIdxInEom = 0; patchIdxInEom < numPatchesInEOMPatches; patchIdxInEom++ ) {
        size_t memberPatchIdx = ( bDecoder && context.getAtlasSequenceParameterSet( 0 ).getPatchPrecedenceOrderFlag() )
                                    ? ( totalPatchCount - eomPatch.memberPatches_[patchIdxInEom] - 1 )
                                    : eomPatch.memberPatches_[patchIdxInEom];
        size_t numberOfEOMPointsPerPatch = eomPointsPerPatch[memberPatchIdx].size();
        for ( size_t pointCount = 0; pointCount < numberOfEOMPointsPerPatch; pointCount++ ) {
          size_t currBlock                 = totalPointCount / blockSize;
          size_t nPixelInCurrentBlockCount = totalPointCount - currBlock * blockSize;
          size_t uBlock                    = currBlock % blockToPatchWidth;
          size_t vBlock                    = currBlock / blockToPatchWidth;
          size_t uu =
              uBlock * params.occupancyResolution_ + nPixelInCurrentBlockCount % params.occupancyResolution_ + u0Eom;
          size_t vv =
              vBlock * params.occupancyResolution_ + nPixelInCurrentBlockCount / params.occupancyResolution_ + v0Eom;
          PCCPoint3D point1      = eomPointsPerPatch[memberPatchIdx][pointCount];
          size_t     pointIndex1 = reconstruct.addPoint( point1 );
          reconstruct.setPointPatchIndex( pointIndex1, tileIndex, patchIndex );
          eomSavedPoints.addPoint( point1 );

          // reconstruct.setColor( pointIndex1, color );
          if ( PCC_SAVE_POINT_TYPE == 1 ) { reconstruct.setType( pointIndex1, POINT_EOM ); }
          partition.push_back( uint32_t( patchIndex ) );
          totalPointCount++;
          pointToPixel.emplace_back( uu, vv, 0 );
          if ( !params.useAuxSeperateVideo_ ) { occupancyMap[vv * tileWidth + uu] = 1; }  // occupied
        }
      }
      TRACE_CODEC( "%d eomPatch :%zu,%zu\t %zu patches, %zu points\n", j, u0Eom, v0Eom, numPatchesInEOMPatches,
                   eomPatch.eomCount_ );
    }
    tile.setTotalNumberOfEOMPoints( totalEOMPointsInFrame );
    printf( "frame %zu, tile %zu: regularPoints+eomPoints %zu\n", frameIndex, tileIndex, reconstruct.getPointCount() );
  } else {
    tile.setTotalNumberOfEOMPoints( 0 );
    printf( "frame %zu, tile %zu: regularPoints+eomPoints %zu\n", frameIndex, tileIndex, reconstruct.getPointCount() );
  }
  TRACE_CODEC( " totalEOMPointsInFrame = %zu  \n", totalEOMPointsInFrame );
  TRACE_CODEC( " point = %zu  \n", reconstruct.getPointCount() );
  if ( params.useAdditionalPointsPatch_ ) {
    auto& frameRawPoint = useRawPointsSeparateVideo
                              ? (const PCCImageGeometry&)context.getVideoRawPointsGeometry()[tile.getFrameIndex()]
                              : frame0;
    TRACE_CODEC( "%s \n", "Add points from rawPointsPatch" );

    // Add points from rawPointsPatch
    size_t numberOfRawPointsPatches = tile.getNumberOfRawPointsPatches();
    for ( int i = 0; i < numberOfRawPointsPatches; i++ ) {
      auto&      rawPointsPatch = tile.getRawPointsPatch( i );
      PCCColor3B rawPointsColor( 0, 255, 255 );
      size_t     numRawPoints = rawPointsPatch.getNumberOfRawPoints();
      rawPointsPatch.sizeV_   = rawPointsPatch.sizeV0_ * rawPointsPatch.occupancyResolution_;
      rawPointsPatch.sizeU_   = rawPointsPatch.sizeU0_ * rawPointsPatch.occupancyResolution_;
      std::vector<PCCPoint3D> rawPoints;
      rawPoints.resize( numRawPoints );
      size_t       numRawPointsAdded = 0;
      const size_t v0                = rawPointsPatch.v0_ * rawPointsPatch.occupancyResolution_;
      const size_t u0                = rawPointsPatch.u0_ * rawPointsPatch.occupancyResolution_;
      for ( size_t v = 0; v < rawPointsPatch.sizeV_; ++v ) {
        for ( size_t u = 0; u < rawPointsPatch.sizeU_; ++u ) {
          const size_t u1 = tile.getLeftTopXInFrame() + u0 + u;
          const size_t v1 = tile.getLeftTopYInFrame() + v0 + v;
          if ( numRawPointsAdded < numRawPoints ) {
            rawPoints[numRawPointsAdded][0] = double( frameRawPoint.getValue( 0, u1, v1 ) + rawPointsPatch.u1_ );
          } else if ( numRawPoints <= numRawPointsAdded && numRawPointsAdded < 2 * numRawPoints ) {
            rawPoints[numRawPointsAdded - numRawPoints][1] =
                double( frameRawPoint.getValue( 0, u1, v1 ) + rawPointsPatch.v1_ );
          } else if ( 2 * numRawPoints <= numRawPointsAdded && numRawPointsAdded < 3 * numRawPoints ) {
            rawPoints[numRawPointsAdded - 2 * numRawPoints][2] =
                double( frameRawPoint.getValue( 0, u1, v1 ) + rawPointsPatch.d1_ );
          }
          numRawPointsAdded++;
        }  // u
      }    // v
      size_t counter = 0;
      for ( size_t v = 0; v < rawPointsPatch.sizeV_; ++v ) {
        for ( size_t u = 0; u < rawPointsPatch.sizeU_; ++u ) {
          if ( counter < numRawPoints ) {
            const size_t pointIndex = reconstruct.addPoint( rawPoints[counter] );
            reconstruct.setPointPatchIndex( pointIndex, tileIndex, patchIndex );
            reconstruct.setColor( pointIndex, rawPointsColor );
            partition.push_back( uint32_t( patchIndex ) );
            pointToPixel.emplace_back( u0 + u, v0 + v, 0 );
            counter++;
          }
        }
      }
    }
    printf( "frame %zu, tile %zu: regularPoints+eomPoints+rawPoints %zu\n", frameIndex, tileIndex,
            reconstruct.getPointCount() );
  } else {
    printf( "frame %zu, tile %zu: regularPoints+eomPoints+rawPoints %zu\n", frameIndex, tileIndex,
            reconstruct.getPointCount() );
    tile.setTotalNumberOfRawPoints( 0 );
  }
  TRACE_CODEC( " totalRawPointsInFrame = %zu  \n", tile.getTotalNumberOfRawPoints() );
  TRACE_CODEC( " point = %zu  \n", reconstruct.getPointCount() );

  if ( params.flagGeometrySmoothing_ && !params.pbfEnableFlag_ ) {
    TRACE_CODEC( "%s \n", "identify first boundary layer \n" );
    if ( useRawPointsSeparateVideo ) {
      assert( ( reconstruct.getPointCount() - tile.getTotalNumberOfRawPoints() ) == pointToPixel.size() );
    } else {
      if ( tile.getPatches().size() == 0 ) {
        assert( ( tile.getTotalNumberOfRawPoints() ) == pointToPixel.size() );
      } else {
        assert( ( reconstruct.getPointCount() + tile.getTotalNumberOfRawPoints() ) == pointToPixel.size() );
      }
    }
    size_t pointCount = reconstruct.getPointCount() - tile.getTotalNumberOfRawPoints();
    for ( size_t i = 0; i < pointCount; ++i ) {
      const PCCVector3<size_t> location = pointToPixel[i];
      const size_t             x        = location[0];
      const size_t             y        = location[1];
      if ( occupancyMap[y * tileWidth + x] != 0 ) {
        identifyBoundaryPoints( occupancyMap, x, y, tileWidth, tileHeight, i, BPflag, reconstruct );
      }
    }
  }
#ifdef CODEC_TRACE
  TRACE_CODEC( " generatePointCloud create %zu points \n", reconstruct.getPointCount() );
  printChecksum( reconstruct, "generatePointCloud Out" );
#endif
}

void PCCCodec::addGridCentroid( PCCPoint3D&                     point,
                                uint32_t                        patchIdx,
                                std::vector<uint16_t>&          count,
                                std::vector<PCCVector3<float>>& centerGrid,
                                std::vector<uint32_t>&          gpartition,
                                std::vector<bool>&              doSmooth,
                                uint8_t                         gridSize,
                                uint16_t                        gridWidth,
#if CELLID_UNORDERED_MAP
                                int64_t                         cellId) {
#else
                                int                             cellId ) {
#endif
  if ( count[cellId] == 0 ) {
    gpartition[cellId] = patchIdx;
    centerGrid[cellId] = {0., 0., 0.};
    doSmooth[cellId]   = false;
  } else if ( !doSmooth[cellId] && gpartition[cellId] != patchIdx ) {
    doSmooth[cellId] = true;
  }
  centerGrid[cellId] += PCCVector3<float>( point );
  count[cellId]++;
}

bool PCCCodec::gridFiltering( const std::vector<uint32_t>&    partition,
                              PCCPointSet3&                   pointCloud,
                              PCCPoint3D&                     curPoint,
                              PCCVector3D&                    centroid,
                              int&                            count,
                              std::vector<uint16_t>&          gridCount,
                              std::vector<PCCVector3<float>>& center,
                              std::vector<bool>&              doSmooth,
                              uint8_t                         gridSize,
                              uint16_t                        gridWidth,
#if CELLID_UNORDERED_MAP
                              std::unordered_map<size_t,int64_t>& cellIndex) {
#else
                              std::vector<int>& cellIndex ) {
#endif
    
  const uint16_t  gridSizeHalf           = gridSize / 2;
  bool            otherClusterPointCount = false;
  PCCVector3<int> P                      = curPoint;
  PCCVector3<int> P2                     = P / gridSize;
  PCCVector3<int> P3                     = P - P2 * gridSize;
  PCCVector3<int> S( P2[0] + ( ( P3[0] < gridSizeHalf ) ? -1 : 0 ), P2[1] + ( ( P3[1] < gridSizeHalf ) ? -1 : 0 ),
                     P2[2] + ( ( P3[2] < gridSizeHalf ) ? -1 : 0 ) );
#if  CELLID_UNORDERED_MAP
  int64_t             idx[2][2][2];
#else
  int             idx[2][2][2];
#endif
  for ( int dz = 0; dz < 2; dz++ ) {
    for ( int dy = 0; dy < 2; dy++ ) {
      for ( int dx = 0; dx < 2; dx++ ) {
#if  CELLID_UNORDERED_MAP
        int64_t tmp = ((int64_t)S[0] + dx) + ((int64_t)S[1] + dy) * gridWidth + ((int64_t)S[2] + dz) * gridWidth * gridWidth;
#else
        int tmp         = ( S[0] + dx ) + ( S[1] + dy ) * gridWidth + ( S[2] + dz ) * gridWidth * gridWidth;
#endif
        idx[dz][dy][dx] = tmp;
        if ( doSmooth[cellIndex[tmp]] && ( gridCount[cellIndex[tmp]] != 0U ) ) { otherClusterPointCount = true; }
      }
    }
  }
  if ( !otherClusterPointCount ) { return otherClusterPointCount; }
  PCCVector3D     centroid3[2][2][2] = {};
  PCCVector3D     curVector          = P;
  int             gridSize2          = gridSize * 2;
#if  CELLID_UNORDERED_MAP
  uint64_t             gridWidth3         = gridWidth * gridWidth * gridWidth;
#else
  int             gridWidth3 = gridWidth * gridWidth * gridWidth;
#endif
  PCCVector3<int> S2                 = S * gridSize;
  PCCVector3<int> W                  = ( P - S2 - gridSizeHalf ) * 2 + 1;
  for ( int dz = 0; dz < 2; dz++ ) {
    for ( int dy = 0; dy < 2; dy++ ) {
      for ( int dx = 0; dx < 2; dx++ ) {
        if ( ( dx == 0 && dy == 0 && dz == 0 ) || ( idx[dz][dy][dx] < gridWidth3 ) ) {
          auto index            = cellIndex[idx[dz][dy][dx]];
          centroid3[dz][dy][dx] = gridCount[index] > 0 ? PCCVector3<double>( center[index] ) : curVector;
        } else {
          centroid3[dz][dy][dx] = curVector;
        }
      }
    }
  }
  PCCVector3<int> Q( gridSize2 - W[0], gridSize2 - W[1], gridSize2 - W[2] );
  count = 0;
  PCCVector3D centroid4( 0.0 );
  for ( int dz = 0, c = Q[2]; dz < 2; dz++, c = W[2] ) {
    for ( int dy = 0, b = Q[1]; dy < 2; dy++, b = W[1] ) {
      for ( int dx = 0, a = Q[0]; dx < 2; dx++, a = W[0] ) {
        centroid3[dz][dy][dx] *= a * b * c;
        centroid4 += centroid3[dz][dy][dx];
        count += a * b * c * gridCount[cellIndex[idx[dz][dy][dx]]];
      }
    }
  }
  centroid4 /= gridSize2 * gridSize2 * gridSize2;
  count /= gridSize2 * gridSize2 * gridSize2;
  centroid = centroid4 * count;
  return otherClusterPointCount;
}

void PCCCodec::smoothPointCloudGrid( PCCPointSet3&                       reconstruct,
                                     const std::vector<uint32_t>&        partition,
                                     const GeneratePointCloudParameters& params,
                                     uint16_t                            gridWidth,
#if CELLID_UNORDERED_MAP
                                     std::unordered_map<size_t, int64_t>& cellIndex) {
#else
                                     std::vector<int>& cellIndex ) {
#endif
  TRACE_CODEC( "%s \n", "smoothPointCloudGrid start" );
  const size_t pointCount = reconstruct.getPointCount();
  const int    gridSize   = static_cast<int>( params.gridSize_ );
  const int    disth      = ( std::max )( gridSize / 2, 1 );
  const int    th         = gridSize * gridWidth;
  for ( int c = 0; c < pointCount; c++ ) {
    PCCPoint3D      curPoint = reconstruct[c];
    PCCVector3<int> P        = curPoint;
    if ( P[0] < disth || P[1] < disth || P[2] < disth || th <= P[0] + disth || th <= P[1] + disth ||
         th <= P[2] + disth ) {
      continue;
    }
    PCCVector3D centroid( 0.0 );
    PCCVector3D curVector              = P;
    int         count                  = 0;
    bool        otherClusterPointCount = false;
    PCCVector3D color( 0, 0, 0 );
    if ( reconstruct.getBoundaryPointType( c ) == 1 ) {
      otherClusterPointCount =
          gridFiltering( partition, reconstruct, curPoint, centroid, count, geoSmoothingCount_, geoSmoothingCenter_,
                         geoSmoothingDoSmooth_, gridSize, gridWidth, cellIndex );
    }
    if ( otherClusterPointCount ) {
      double dist2 = ( ( curVector * count - centroid ).getNorm2() ) / static_cast<double>( count ) + 0.5;
      if ( dist2 >= ( std::max )( static_cast<int>( params.thresholdSmoothing_ ), count ) * 2 ) {
        centroid = centroid / static_cast<double>( count ) + 0.5;
        for ( size_t k = 0; k < 3; ++k ) { centroid[k] = double( int64_t( centroid[k] ) ); }
        reconstruct[c] = centroid;
        if ( PCC_SAVE_POINT_TYPE == 1 ) { reconstruct.setType( c, POINT_SMOOTH ); }
        reconstruct.setBoundaryPointType( c, static_cast<uint16_t>( 3 ) );
      }
    }
  }
  TRACE_CODEC( "%s \n", "smoothPointCloudGrid done" );
}

void PCCCodec::smoothPointCloud( PCCPointSet3&                      reconstruct,
                                 const std::vector<uint32_t>&       partition,
                                 const GeneratePointCloudParameters params ) {
  TRACE_CODEC( "%s \n", "smoothPointCloud start" );
  const size_t pointCount = reconstruct.getPointCount();
  PCCKdTree    kdtree( reconstruct );
  PCCPointSet3 temp;
  temp.resize( pointCount );
#if defined( ENABLE_TBB )
  tbb::task_arena limited( static_cast<int>( params.nbThread_ ) );
  limited.execute( [&] {
    tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) {
#else
  for ( size_t i = 0; i < pointCount; i++ ) {
#endif
      const size_t clusterindex_ = partition[i];
      PCCNNResult  result;
      kdtree.searchRadius( reconstruct[i], params.neighborCountSmoothing_, params.radius2Smoothing_, result );
      PCCVector3D centroid( 0.0 );
      bool        otherClusterPointCount = false;
      size_t      neighborCount          = 0;
      for ( size_t r = 0; r < result.count(); ++r ) {
        const double& dist2 = result.dist( r );
        ++neighborCount;
        const size_t pointindex_ = result.indices( r );
        centroid += reconstruct[pointindex_];
        otherClusterPointCount |=
            ( dist2 <= params.radius2BoundaryDetection_ ) && ( partition[pointindex_] != clusterindex_ );
      }
      if ( otherClusterPointCount ) {
        if ( reconstruct.getBoundaryPointType( i ) == 1 ) {
          reconstruct.setBoundaryPointType( i, static_cast<uint16_t>( 2 ) );
        }
        const PCCVector3D scaledPoint =
            double( neighborCount ) * PCCVector3D( reconstruct[i][0], reconstruct[i][1], reconstruct[i][2] );
        const double distToCentroid2 =
            int64_t( ( centroid - scaledPoint ).getNorm2() + ( neighborCount / 2.0 ) ) / double( neighborCount );
        for ( size_t k = 0; k < 3; ++k ) {
          centroid[k] = double( int64_t( ( centroid[k] + ( neighborCount / 2 ) ) / neighborCount ) );
        }
        if ( distToCentroid2 >= params.thresholdSmoothing_ ) {
          temp[i] = centroid;
          reconstruct.setColor( i, PCCColor3B( 255, 0, 0 ) );
          if ( PCC_SAVE_POINT_TYPE == 1 ) { reconstruct.setType( i, POINT_SMOOTH ); }
        } else {
          temp[i] = reconstruct[i];
        }
      } else {
        temp[i] = reconstruct[i];
      }
#if defined( ENABLE_TBB )
    } );
  } );
  limited.execute(
      [&] { tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) { reconstruct[i] = temp[i]; } ); } );
#else
  }
  for ( size_t i = 0; i < pointCount; i++ ) { reconstruct[i] = temp[i]; }
#endif
  TRACE_CODEC( "%s \n", "smoothPointCloud done" );
}

void PCCCodec::addGridColorCentroid( PCCPoint3D&                             point,
                                     PCCVector3D&                            color,
                                     std::pair<size_t, size_t>               tilePatchIdx,
                                     std::vector<uint16_t>&                  colorGridCount,
                                     std::vector<PCCVector3<float>>&         colorCenter,
                                     std::vector<std::pair<size_t, size_t>>& colorPartition,
                                     std::vector<bool>&                      colorDoSmooth,
                                     uint8_t                                 gridSize,
                                     std::vector<std::vector<uint16_t>>&     colorLum,
                                     const GeneratePointCloudParameters&     params,
#if CELLID_UNORDERED_MAP
                                     int64_t                                 cellId) {
#else
                                     int                                     cellId ) {
#endif
  if ( colorGridCount[cellId] == 0 ) {
    colorPartition[cellId] = tilePatchIdx;
    colorCenter[cellId]    = {0., 0., 0.};
    colorDoSmooth[cellId]  = false;
  } else if ( !colorDoSmooth[cellId] && colorPartition[cellId] != tilePatchIdx ) {
    colorDoSmooth[cellId] = true;
  }
  colorCenter[cellId] += PCCVector3<float>( color );
  colorGridCount[cellId]++;
  colorLum[cellId].push_back( uint16_t( color[0] ) );
}

bool PCCCodec::gridFilteringColor( PCCPoint3D&                         curPos,
                                   PCCVector3D&                        colorCentroid,
                                   int&                                colorCount,
                                   std::vector<uint16_t>&              colorGridCount,
                                   std::vector<PCCVector3<float>>&     colorCenter,
                                   std::vector<bool>&                  colorDoSmooth,
#if COLOR_SMOOTHING_ALIGNMENT
                                   size_t                              w,
#endif
                                   uint8_t                             gridSize,
                                   PCCVector3D&                        curPosColor,
                                   const GeneratePointCloudParameters& params,
#if CELLID_UNORDERED_MAP
                                   std::unordered_map<size_t, int64_t>& cellIndex) {
    int64_t             idx[2][2][2];
#else
                                   std::vector<int>&                   cellIndex ) {
    int             idx[2][2][2];
#endif
#if !COLOR_SMOOTHING_ALIGNMENT
  const int       w                      = pow( 2, params.geometryBitDepth3D_ ) / gridSize;
#endif
  bool            otherClusterPointCount = false;
  PCCVector3<int> P                      = curPos;
  PCCVector3<int> P2                     = P / gridSize;
  PCCVector3<int> S( P2[0] + ( ( ( P[0] % gridSize ) < gridSize / 2 ) ? -1 : 0 ),
                     P2[1] + ( ( ( P[1] % gridSize ) < gridSize / 2 ) ? -1 : 0 ),
                     P2[2] + ( ( ( P[2] % gridSize ) < gridSize / 2 ) ? -1 : 0 ) );
  for ( int dz = 0; dz < 2; dz++ ) {
    for ( int dy = 0; dy < 2; dy++ ) {
      for ( int dx = 0; dx < 2; dx++ ) {
#if CELLID_UNORDERED_MAP
        idx[dz][dy][dx] = ((int64_t)S[0] + dx) + ((int64_t)S[1] + dy) * w + ((int64_t)S[2] + dz) * w * w;
#else
        idx[dz][dy][dx] = ( S[0] + dx ) + ( S[1] + dy ) * w + ( S[2] + dz ) * w * w;
#endif
        if ( colorDoSmooth[cellIndex[idx[dz][dy][dx]]] && ( colorGridCount[cellIndex[idx[dz][dy][dx]]] != 0U ) ) {
          otherClusterPointCount = true;
        }
      }
    }
  }
  if ( !otherClusterPointCount ) { return otherClusterPointCount; }
  PCCVector3<int> S2                      = S * gridSize;
  PCCVector3<int> W                       = ( P - S2 - gridSize / 2 ) * 2 + 1;
  PCCVector3D     colorCentroid3[2][2][2] = {};
  const int       gridSize2               = gridSize * 2;
  const double    mmThresh                = params.thresholdColorVariation_ * 256.0;
  const double    yThresh                 = params.thresholdColorDifference_ * 256.0;
  double          Y0                      = 0;
  for ( int dz = 0; dz < 2; dz++ ) {
    for ( int dy = 0; dy < 2; dy++ ) {
      for ( int dx = 0; dx < 2; dx++ ) {
#if CELLID_UNORDERED_MAP
        int64_t   index = cellIndex[idx[dz][dy][dx]];
#else
        int   index = cellIndex[idx[dz][dy][dx]];
#endif
        auto& dst   = colorCentroid3[dz][dy][dx];
        if ( colorGridCount[index] > 0 ) {
          for ( size_t c = 0; c < 3; c++ ) {
            dst[c] = double( colorCenter[index][c] ) / double( colorGridCount[index] );
          }
          if ( dx == 0 && dy == 0 && dz == 0 ) {
            if ( colorGridCount[index] > 1 ) {
              double meanY   = mean( colorSmoothingLum_[index], int( colorGridCount[index] ) );
              double medianY = median( colorSmoothingLum_[index], int( colorGridCount[index] ) );
              if ( abs( meanY - medianY ) > mmThresh ) {
                colorCentroid = curPosColor;
                colorCount    = 1;
                return otherClusterPointCount;
              }
            }
          } else {
            if ( abs( Y0 - dst[0] ) > yThresh ) { dst = curPosColor; }
            if ( colorGridCount[index] > 1 ) {
              double meanY   = mean( colorSmoothingLum_[index], int( colorGridCount[index] ) );
              double medianY = median( colorSmoothingLum_[index], int( colorGridCount[index] ) );
              if ( abs( meanY - medianY ) > mmThresh ) { dst = curPosColor; }
            }
          }
        } else {
          dst = curPosColor;
        }
        if ( dx == 0 && dy == 0 && dz == 0 ) { Y0 = dst[0]; }
      }
    }
  }
  PCCVector3<int> G( gridSize2 - W[0], gridSize2 - W[1], gridSize2 - W[2] );
  PCCVector3D     colorCentroid4( 0.0 );
  for ( int dz = 0, c = G[2]; dz < 2; dz++, c = W[2] ) {
    for ( int dy = 0, b = G[1]; dy < 2; dy++, b = W[1] ) {
      for ( int dx = 0, a = G[0]; dx < 2; dx++, a = W[0] ) {
        colorCentroid3[dz][dy][dx] *= a * b * c;
        colorCentroid4 += colorCentroid3[dz][dy][dx];
      }
    }
  }
  colorCentroid4 /= gridSize2 * gridSize2 * gridSize2;
  colorCentroid = colorCentroid4;
  colorCount    = 1;
  return otherClusterPointCount;
}

void PCCCodec::smoothPointCloudColorLC( PCCPointSet3&                       reconstruct,
                                        const GeneratePointCloudParameters& params,
#if CELLID_UNORDERED_MAP
                                        std::unordered_map<size_t, int64_t>& cellIndex) {
#else
                                        std::vector<int>&                   cellIndex ) {
#endif
    
  const size_t pointCount = reconstruct.getPointCount();
#if COLOR_SMOOTHING_ALIGNMENT
  PCCInt16Box3D boundingBox;
  boundingBox.min_ = boundingBox.max_ = reconstruct[0];
  for (int j = 0; j < reconstruct.getPointCount(); j++) {
      const PCCPoint3D point = reconstruct[j];
      for (size_t k = 0; k < 3; ++k) {
          if (point[k] < boundingBox.min_[k]) { boundingBox.min_[k] = floor(point[k]); }
          if (point[k] > boundingBox.max_[k]) { boundingBox.max_[k] = ceil(point[k]); }
      }
  }
  int pcMaxSize = (std::max)((std::max)(boundingBox.max_.x(), boundingBox.max_.y()), boundingBox.max_.z());
  const size_t     gridSize = params.cgridSize_;
  const size_t w =
      (pcMaxSize + static_cast<int>(gridSize) - 1) / (static_cast<int>(gridSize));
  const int    disth = (std::max)(static_cast<int>(gridSize) / 2, 1);
#else
  const int    gridSize = params.occupancyPrecision_;
  const int    disth = (std::max)(gridSize / 2, 1);
#endif
  for ( int i = 0; i < pointCount; i++ ) {
    PCCPoint3D curPos    = reconstruct[i];
    int        x         = curPos.x();
    int        y         = curPos.y();
    int        z         = curPos.z();
    int        pcMaxSize = pow( 2, params.geometryBitDepth3D_ );
    if ( x < disth || y < disth || z < disth || pcMaxSize <= x + disth || pcMaxSize <= y + disth ||
         pcMaxSize <= z + disth ) {
      continue;
    }
    PCCVector3D colorCentroid( 0.0 );
    int         colorCount             = 0;
    bool        otherClusterPointCount = false;
    PCCVector3D curPosColor            = reconstruct.getColor16bit( i );
    if ( reconstruct.getBoundaryPointType( i ) == 1 ) {
#if COLOR_SMOOTHING_ALIGNMENT
        otherClusterPointCount =
          gridFilteringColor( curPos, colorCentroid, colorCount, colorSmoothingCount_, colorSmoothingCenter_,
                              colorSmoothingDoSmooth_, w, gridSize, curPosColor, params, cellIndex );
#else
        otherClusterPointCount =
            gridFilteringColor(curPos, colorCentroid, colorCount, colorSmoothingCount_, colorSmoothingCenter_,
                colorSmoothingDoSmooth_, gridSize, curPosColor, params, cellIndex);
#endif
    }
    if ( otherClusterPointCount ) {
      colorCentroid = ( colorCentroid + static_cast<double>( colorCount ) / 2.0 ) / static_cast<double>( colorCount );
      for ( size_t k = 0; k < 3; ++k ) { colorCentroid[k] = double( int64_t( colorCentroid[k] ) ); }
      double distToCentroid2 = 0;
      double Ycent           = colorCentroid[0];
      double Ycur            = curPosColor[0];
      distToCentroid2        = abs( Ycent - Ycur ) * 10. / 256.;
      if ( distToCentroid2 >= params.thresholdColorSmoothing_ ) {
        PCCColor16bit color16bit = colorCentroid;
        reconstruct.setColor16bit( i, color16bit );
      }
    }
  }
}

size_t PCCCodec::colorPointCloud( PCCPointSet3&                       reconstruct,
                                  PCCContext&                         context,
                                  PCCFrameContext&                    tile,
                                  const std::vector<bool>&            absoluteT1List,
                                  const size_t                        multipleStreams,
                                  const uint8_t                       attributeCount,
                                  size_t                              accTilePointCount,
                                  const GeneratePointCloudParameters& params ) {
  TRACE_CODEC( "%s \n", "colorPointCloud start" );

  if ( reconstruct.getPointCount() == 0 ) { return accTilePointCount; }
  reconstruct.fillColor();

#ifdef CODEC_TRACE
  printChecksum( reconstruct, "colorPointCloud in" );
#endif
  auto&        videoAttribute       = context.getVideoAttributesMultiple()[0];
  auto&        videoAttributeFrame1 = context.getVideoAttributesMultiple()[1];
  const size_t mapCount             = params.mapCountMinus1_ + 1;
  if ( attributeCount == 0 ) {
    for ( auto& color : reconstruct.getColors() ) {
      for ( size_t c = 0; c < 3; ++c ) { color[c] = static_cast<uint8_t>( 127 ); }
    }
  } else {
    auto&  pointToPixel      = tile.getPointToPixel();
    auto&  color16bit        = reconstruct.getColors16bit();
    bool   useAuxVideo       = tile.getUseRawPointsSeparateVideo();
    size_t numOfRawPointGeos = tile.getTotalNumberOfRawPoints();
    size_t numberOfEOMPoints = tile.getTotalNumberOfEOMPoints();
    size_t pointCount        = tile.getTotalNumberOfRegularPoints();
    if ( !useAuxVideo ) { pointCount += numOfRawPointGeos + numberOfEOMPoints; }
    TRACE_CODEC( "plt.getProfileCodecGroupIdc() = %d \n",
                 context.getVps().getProfileTierLevel().getProfileCodecGroupIdc() );

    TRACE_CODEC( "vps.getRawPatchEnabledFlag()  = %d \n", context.getVps().getAuxiliaryVideoPresentFlag( 0 ) );
    TRACE_CODEC( "useAuxVideo                   = %d \n", useAuxVideo );
    TRACE_CODEC( "numOfRawGeos                  = %zu \n", numOfRawPointGeos );
    if ( params.enhancedOccupancyMapCode_ ) {
      TRACE_CODEC( "numberOfRawPointsAndEOMColors = %zu \n", numOfRawPointGeos + numberOfEOMPoints );
      TRACE_CODEC( "numberOfEOMPoints             = %zu \n", numberOfEOMPoints );
    }
    TRACE_CODEC( "pointCount                   = %zu \n", pointCount );
    TRACE_CODEC( "reconstruct.getPointCount()  = %zu \n", reconstruct.getPointCount() );
    TRACE_CODEC( "pointToPixel size            = %zu \n", pointToPixel.size() );
    TRACE_CODEC( "pointLocalReconstruction     = %d \n", params.pointLocalReconstruction_ );
    TRACE_CODEC( "singleMapPixelInterleaving   = %d \n", params.singleMapPixelInterleaving_ );
    TRACE_CODEC( "enhancedOccupancyMapCode       = %d \n", params.enhancedOccupancyMapCode_ );
    TRACE_CODEC( "multipleStreams              = %d \n", multipleStreams );
    PCCPointSet3        target;
    PCCPointSet3        source;
    std::vector<size_t> targetIndex;
    targetIndex.resize( 0 );
    target.clear();
    source.clear();
    target.addColors16bit();
    source.addColors16bit();
    const size_t shift = params.multipleStreams_ ? tile.getFrameIndex() : tile.getFrameIndex() * mapCount;
    for ( size_t i = accTilePointCount; i < accTilePointCount + pointCount; ++i ) {
      const PCCVector3<size_t> location = pointToPixel[i - accTilePointCount];
      const size_t             x        = tile.getLeftTopXInFrame() + location[0];
      const size_t             y        = tile.getLeftTopYInFrame() + location[1];
      const size_t             f        = location[2];
      if ( params.singleMapPixelInterleaving_ ) {
        if ( ( static_cast<int>( f == 0 && ( x + y ) % 2 == 0 ) | static_cast<int>( f == 1 && ( x + y ) % 2 == 1 ) ) !=
             0 ) {
          const auto& image = videoAttribute.getFrame( shift );
          for ( size_t c = 0; c < 3; ++c ) { color16bit[i][c] = image.getValue( c, x, y ); }
          int index = source.addPoint( reconstruct[i] );
          source.setColor16bit( index, color16bit[i] );
        } else {
          target.addPoint( reconstruct[i] );
          targetIndex.push_back( i );
        }
      } else if ( multipleStreams != 0U ) {
        if ( f == 0 ) {
          const auto& image = videoAttribute.getFrame( tile.getFrameIndex() );
          for ( size_t c = 0; c < 3; ++c ) { color16bit[i][c] = image.getValue( c, x, y ); }
          size_t index = source.addPoint( reconstruct[i] );
          source.setColor16bit( index, color16bit[i] );
        } else {
          const auto& image0   = videoAttribute.getFrame( tile.getFrameIndex() );
          const auto& image1   = videoAttributeFrame1.getFrame( tile.getFrameIndex() );
          uint8_t     numBits  = image0.getDeprecatedColorFormat() == 0 ? 8 : 16;  // or 8
          double      offset   = ( 1 << ( numBits - 1 ) );
          double      maxValue = ( 1 << numBits ) - 1;
          for ( size_t c = 0; c < 3; ++c ) {
            // reconstruction
            auto value0 = static_cast<uint16_t>( image0.getValue( c, x, y ) );
            auto value1 = static_cast<uint16_t>( image1.getValue( c, x, y ) );
            if ( !absoluteT1List[f] ) {
              int32_t newValue = value1;
              newValue -= offset;
              // clipping the delta value
              if ( newValue < -offset ) {
                newValue = -offset;
              } else if ( newValue > offset - 1 ) {
                newValue = offset - 1;
              }
              newValue += value0;  // add value0
              color16bit[i][c] =
                  newValue < 0 ? 0 : ( newValue > maxValue ? maxValue : static_cast<uint16_t>( newValue ) );
              // clipping to the unsigned 16 bit range
            } else {
              color16bit[i][c] = value1;
            }
          }
          size_t index = source.addPoint( reconstruct[i] );
          source.setColor16bit( index, color16bit[i] );
        }
      } else {
        if ( f < mapCount ) {
          const auto& frame = videoAttribute.getFrame( shift + f );
          for ( size_t c = 0; c < 3; ++c ) { color16bit[i][c] = frame.getValue( c, x, y ); }
          int index = source.addPoint( reconstruct[i] );
          source.setColor16bit( index, color16bit[i] );
        } else {
          target.addPoint( reconstruct[i] );
          targetIndex.push_back( i );
        }
      }
    }  // i < accTilePointCount+ pointCount;
    if ( target.getPointCount() > 0 ) {
      source.transferColorWeight( target );
      for ( size_t i = 0; i < target.getPointCount(); ++i ) {
        reconstruct.setColor16bit( targetIndex[i], target.getColor16bit( i ) );
      }
    }

    if ( useAuxVideo ) {
      for ( const auto& color : tile.getEOMAttribute() ) { color16bit[pointCount++] = color; }
      for ( const auto& color : tile.getRawPointsAttribute() ) { color16bit[pointCount++] = color; }
    }
  }  // noAtt

#ifdef CODEC_TRACE
  printChecksum( reconstruct, "colorPointCloud out" );
  TRACE_CODEC( "%s \n", "colorPointCloud done" );
#endif

  return accTilePointCount + tile.getTotalNumberOfRegularPoints() + tile.getTotalNumberOfEOMPoints() +
         tile.getTotalNumberOfRawPoints();
}

void PCCCodec::generateRawPointsGeometryfromVideo( PCCContext& context, size_t frameIndex ) {
  TRACE_CODEC( "%s \n", "generateRawPointsGeometryfromVideo start" );
  auto& videoRawPointsGeometry = context.getVideoRawPointsGeometry();
  videoRawPointsGeometry.resize( context.size() );
  for ( size_t tileIdx = 0; tileIdx < context.getFrame( frameIndex ).getNumTilesInAtlasFrame(); tileIdx++ ) {
    auto& tile = context.getFrame( frameIndex ).getTile( tileIdx );
    generateRawPointsGeometryfromVideo( context, tile, frameIndex );
    size_t totalNumRawPoints = 0;
    for ( size_t i = 0; i < tile.getNumberOfRawPointsPatches(); i++ ) {
      totalNumRawPoints += tile.getRawPointsPatch( i ).getNumberOfRawPoints() * 3;
    }
    std::cout << "generate raw Points Video (Geometry) frame[" << frameIndex << "] tile[" << tileIdx << "] : "
              << "#rawPatches: " << tile.getNumberOfRawPointsPatches() << "#rawPoints(pixels): " << totalNumRawPoints
              << std::endl;
  }
  std::cout << "Raw Points Geometry from Video [done]" << std::endl;
  TRACE_CODEC( "%s \n", "generateRawPointsGeometryfromVideo done" );
}

void PCCCodec::generateRawPointsGeometryfromVideo( PCCContext& context, PCCFrameContext& tile, size_t frameIndex ) {
  auto&  videoRawPointsGeometry   = context.getVideoRawPointsGeometry();
  auto&  image                    = videoRawPointsGeometry.getFrame( frameIndex );
  size_t numberOfRawPointsPatches = tile.getNumberOfRawPointsPatches();
  for ( int i = 0; i < numberOfRawPointsPatches; i++ ) {
    auto&        rawPointsPatch = tile.getRawPointsPatch( i );
    const size_t v0             = rawPointsPatch.v0_ * rawPointsPatch.occupancyResolution_;
    const size_t u0             = rawPointsPatch.u0_ * rawPointsPatch.occupancyResolution_;
    rawPointsPatch.sizeV_       = rawPointsPatch.sizeV0_ * rawPointsPatch.occupancyResolution_;
    rawPointsPatch.sizeU_       = rawPointsPatch.sizeU0_ * rawPointsPatch.occupancyResolution_;
    size_t numberOfRawPoints    = rawPointsPatch.getNumberOfRawPoints();
    numberOfRawPoints *= 3;
    rawPointsPatch.resize( numberOfRawPoints );
    for ( size_t v = 0; v < rawPointsPatch.sizeV_; ++v ) {
      for ( size_t u = 0; u < rawPointsPatch.sizeU_; ++u ) {
        const size_t p = v * rawPointsPatch.sizeU_ + u;
        if ( p < numberOfRawPoints ) {
          const size_t x       = ( u0 + u );
          const size_t y       = ( v0 + v ) + context[frameIndex].getAuxTileLeftTopY( tile.getTileIndex() );
          rawPointsPatch.x_[p] = image.getValue( 0, x, y );
        }
      }
    }
  }
}

void PCCCodec::generateRawPointsAttributefromVideo( PCCContext& context, size_t frameIndex ) {
  auto& videoRawPointsAttribute = context.getVideoRawPointsAttribute();
  videoRawPointsAttribute.resize( context.size() );
  TRACE_CODEC( "%s \n", "generateRawPointsAttributefromVideo" );
  for ( size_t tileIdx = 0; tileIdx < context.getFrame( frameIndex ).getNumTilesInAtlasFrame(); tileIdx++ ) {
    auto& tile = context.getFrame( frameIndex ).getTile( tileIdx );
    generateRawPointsAttributefromVideo( context, tile, frameIndex );
    std::cout << "generate raw Points Video (Attribute) frame[" << frameIndex << "] tile[" << tileIdx << "] : "
              << " #rawPatches: " << tile.getNumberOfRawPointsPatches()
              << " #rawPoints: " << tile.getTotalNumberOfRawPoints() << " #eomPoints: " << tile.getEomPatches().size()
              << " #eomPoints: " << tile.getTotalNumberOfEOMPoints() << std::endl;
  }
  std::cout << "RawPoints Attribute [done]" << std::endl;
  TRACE_CODEC( "%s \n", "generateRawPointsAttributefromVideo done" );
}

void PCCCodec::generateRawPointsAttributefromVideo( PCCContext& context, PCCFrameContext& tile, size_t frameIndex ) {
  auto&                    videoRawPointsAttribute  = context.getVideoRawPointsAttribute();
  auto&                    image                    = videoRawPointsAttribute.getFrame( frameIndex );
  size_t                   width                    = image.getWidth();
  size_t                   numberOfEOMPoints        = tile.getTotalNumberOfEOMPoints();
  size_t                   numberOfRawPoints        = tile.getTotalNumberOfRawPoints();
  std::vector<PCCColor3B>& rawAttributes            = tile.getRawPointsAttribute();
  std::vector<PCCColor3B>& eomAttributes            = tile.getEOMAttribute();
  size_t                   numberOfRawPointsPatches = tile.getNumberOfRawPointsPatches();
  rawAttributes.resize( numberOfRawPoints );
  eomAttributes.resize( numberOfEOMPoints );
  if ( numberOfRawPoints ) {
    size_t rawPatchOffset = 0;
    for ( int i = 0; i < numberOfRawPointsPatches; i++ ) {
      int    pointIndex        = 0;
      auto&  rawPointsPatch    = tile.getRawPointsPatch( i );
      size_t numRawColorPoints = rawPointsPatch.getNumberOfRawPoints();
      rawPointsPatch.sizeV_    = rawPointsPatch.sizeV0_ * rawPointsPatch.occupancyResolution_;
      rawPointsPatch.sizeU_    = rawPointsPatch.sizeU0_ * rawPointsPatch.occupancyResolution_;
      printf( "\tgenerateRawPointsAttributeImage:: (u0,v0) %zu,%zu, (sizeU,sizeU) %zux%zu\n", rawPointsPatch.u0_,
              rawPointsPatch.v0_, rawPointsPatch.sizeU_, rawPointsPatch.sizeV_ );
      const size_t v0 = rawPointsPatch.v0_ * rawPointsPatch.occupancyResolution_;
      const size_t u0 = rawPointsPatch.u0_ * rawPointsPatch.occupancyResolution_;
      for ( size_t v = 0; v < rawPointsPatch.sizeV_; ++v ) {
        for ( size_t u = 0; u < rawPointsPatch.sizeU_; ++u ) {
          if ( pointIndex < numRawColorPoints ) {
            const size_t x = ( u0 + u );
            const size_t y = ( v0 + v ) + context[frameIndex].getAuxTileLeftTopY( tile.getTileIndex() );
            assert( x < width && y < image.getHeight() );
            rawAttributes[rawPatchOffset + pointIndex].r() = image.getValue( 0, x, y );
            rawAttributes[rawPatchOffset + pointIndex].g() = image.getValue( 1, x, y );
            rawAttributes[rawPatchOffset + pointIndex].b() = image.getValue( 2, x, y );
            pointIndex++;
          } else {
            break;
          }
        }
      }
      rawPatchOffset += numRawColorPoints;
    }
    assert( rawPatchOffset == numberOfRawPoints );
  }
  if ( numberOfEOMPoints != 0 ) {
    size_t eomPatchOffset            = 0;
    size_t nPixelInCurrentBlockCount = 0;
    size_t xx = 0, yy = 0;
    for ( auto& eomPointsPatch : tile.getEomPatches() ) {
      std::vector<PCCColor3B>& eomAttributes  = tile.getEOMAttribute();
      size_t                   patchStartPosX = eomPointsPatch.u0_ * eomPointsPatch.occupancyResolution_;
      size_t                   patchStartPosY = eomPointsPatch.v0_ * eomPointsPatch.occupancyResolution_;
      size_t                   blockSize    = eomPointsPatch.occupancyResolution_ * eomPointsPatch.occupancyResolution_;
      size_t                   widthInBlock = width / eomPointsPatch.occupancyResolution_;
      for ( size_t k = 0; k < eomPointsPatch.eomCount_; k++ ) {
        size_t nBlock = k / blockSize;
        size_t uBlock = nBlock % widthInBlock;
        size_t vBlock = nBlock / widthInBlock;
        xx            = patchStartPosX + uBlock * eomPointsPatch.occupancyResolution_ +
             ( nPixelInCurrentBlockCount % eomPointsPatch.occupancyResolution_ );
        yy = patchStartPosY + vBlock * eomPointsPatch.occupancyResolution_ +
             ( nPixelInCurrentBlockCount / eomPointsPatch.occupancyResolution_ ) +
             context[frameIndex].getAuxTileLeftTopY( tile.getTileIndex() );
        ++nPixelInCurrentBlockCount;
        if ( nPixelInCurrentBlockCount >= blockSize ) { nPixelInCurrentBlockCount = 0; }
        eomAttributes[k + eomPatchOffset].r() = image.getValue( 0, xx, yy );
        eomAttributes[k + eomPatchOffset].g() = image.getValue( 1, xx, yy );
        eomAttributes[k + eomPatchOffset].b() = image.getValue( 2, xx, yy );
      }
      eomPatchOffset += eomPointsPatch.eomCount_;
    }
  }
}

void PCCCodec::generateOccupancyMap( PCCFrameContext&      tile,
                                     PCCImageOccupancyMap& videoFrame,
                                     const size_t          occupancyPrecision,
                                     const size_t          thresholdLossyOM,
                                     const bool            enhancedOccupancyMapForDepthFlag ) {
  auto   width        = tile.getWidth();
  auto   height       = tile.getHeight();
  size_t u0           = tile.getLeftTopXInFrame() / occupancyPrecision;
  size_t v0           = tile.getLeftTopYInFrame() / occupancyPrecision;
  auto&  occupancyMap = tile.getOccupancyMap();
  occupancyMap.resize( width * height, 0 );
  for ( size_t v = 0; v < height; ++v ) {
    for ( size_t u = 0; u < width; ++u ) {
      uint8_t& pixel = videoFrame.getValue( 0, u0 + u / occupancyPrecision, v0 + v / occupancyPrecision );
      if ( !enhancedOccupancyMapForDepthFlag ) {
        occupancyMap[v * width + u] = ( pixel > thresholdLossyOM );
        pixel                       = occupancyMap[v * width + u];
      } else {
        occupancyMap[v * width + u] = pixel;
      }
    }
  }
}

void PCCCodec::generateTileBlockToPatchFromOccupancyMapVideo( PCCContext&  context,
                                                              const size_t occupancyResolution,
                                                              const size_t occupancyPrecision ) {
  for ( int fi = 0; fi < context.size(); fi++ ) {
    PCCImageOccupancyMap& occupancyImage = context.getVideoOccupancyMap().getFrame( fi );
    for ( size_t tileIdx = 0; tileIdx < context[fi].getNumTilesInAtlasFrame(); tileIdx++ ) {
      auto& tile = context[fi].getTile( tileIdx );
      generateTileBlockToPatchFromOccupancyMapVideo( context, tile, fi, occupancyImage, occupancyResolution,
                                                     occupancyPrecision );
    }
  }
}

void PCCCodec::generateTileBlockToPatchFromOccupancyMapVideo( PCCContext&           context,
                                                              PCCFrameContext&      tile,
                                                              size_t                frameIdx,
                                                              PCCImageOccupancyMap& atlasOccupancyMapImage,
                                                              const size_t          occupancyResolution,
                                                              const size_t          occupancyPrecision ) {
  auto&        patches            = tile.getPatches();
  const size_t patchCount         = patches.size();
  const size_t blockToPatchWidth  = tile.getWidth() / occupancyResolution;
  const size_t blockToPatchHeight = tile.getHeight() / occupancyResolution;
  const size_t blockCount         = blockToPatchWidth * blockToPatchHeight;
  auto&        blockToPatch       = tile.getBlockToPatch();
  blockToPatch.resize( blockCount );
  std::fill( blockToPatch.begin(), blockToPatch.end(), 0 );
  for ( size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex ) {
    auto&  patch        = patches[patchIndex];
    size_t nonZeroPixel = 0;
    for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
      for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
        const size_t blockIndex = patch.patchBlock2CanvasBlock( u0, v0, blockToPatchWidth, blockToPatchHeight );
        nonZeroPixel            = 0;
        for ( size_t v1 = 0; v1 < patch.getOccupancyResolution(); ++v1 ) {
          const size_t v = v0 * patch.getOccupancyResolution() + v1;
          for ( size_t u1 = 0; u1 < patch.getOccupancyResolution(); ++u1 ) {
            const size_t u = u0 * patch.getOccupancyResolution() + u1;
            size_t       x;
            size_t       y;
            patch.patch2Canvas( u, v, tile.getWidth(), tile.getHeight(), x, y );
            x += tile.getLeftTopXInFrame();
            y += tile.getLeftTopYInFrame();
            nonZeroPixel += static_cast<unsigned long long>(
                atlasOccupancyMapImage.getValue( 0, x / occupancyPrecision, y / occupancyPrecision ) != 0 );
          }
        }
        if ( nonZeroPixel > 0 ) { blockToPatch[blockIndex] = patchIndex + 1; }
      }
    }
  }
}

void PCCCodec::generateAtlasBlockToPatchFromOccupancyMapVideo( PCCContext&  context,
                                                               const size_t occupancyResolution,
                                                               const size_t occupancyPrecision ) {
  generateTileBlockToPatchFromOccupancyMapVideo( context, occupancyResolution, occupancyPrecision );
  for ( int fi = 0; fi < context.size(); fi++ ) {
    PCCImageOccupancyMap& occupancyImage = context.getVideoOccupancyMap().getFrame( fi );
    auto&                 atlasFrame     = context.getFrame( fi ).getTitleFrameContext();
    // construct atlasPatchData;
    generateAtlasBlockToPatchFromOccupancyMapVideo( context, context.getFrame( fi ).getTitleFrameContext(), fi,
                                                    occupancyImage, occupancyResolution, occupancyPrecision );
  }
}

void PCCCodec::generateAtlasBlockToPatchFromOccupancyMapVideo( PCCContext&           context,
                                                               PCCFrameContext&      titleFrame,
                                                               size_t                frameIdx,
                                                               PCCImageOccupancyMap& occupancyMapImage,
                                                               const size_t          occupancyResolution,
                                                               const size_t          occupancyPrecision ) {
  auto&        patches            = titleFrame.getPatches();
  const size_t patchCount         = patches.size();
  const size_t blockToPatchWidth  = titleFrame.getWidth() / occupancyResolution;
  const size_t blockToPatchHeight = titleFrame.getHeight() / occupancyResolution;
  const size_t blockCount         = blockToPatchWidth * blockToPatchHeight;
  auto&        blockToPatch       = titleFrame.getBlockToPatch();
  blockToPatch.resize( blockCount );
  std::fill( blockToPatch.begin(), blockToPatch.end(), 0 );
  for ( size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex ) {
    auto&  patch        = patches[patchIndex];
    size_t nonZeroPixel = 0;
    for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
      for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
        const size_t blockIndex = patch.patchBlock2CanvasBlock( u0, v0, blockToPatchWidth, blockToPatchHeight );
        nonZeroPixel            = 0;
        for ( size_t v1 = 0; v1 < patch.getOccupancyResolution(); ++v1 ) {
          const size_t v = v0 * patch.getOccupancyResolution() + v1;
          for ( size_t u1 = 0; u1 < patch.getOccupancyResolution(); ++u1 ) {
            const size_t u = u0 * patch.getOccupancyResolution() + u1;
            size_t       x;
            size_t       y;
            patch.patch2Canvas( u, v, titleFrame.getWidth(), titleFrame.getHeight(), x, y );
            nonZeroPixel += static_cast<unsigned long long>(
                occupancyMapImage.getValue( 0, x / occupancyPrecision, y / occupancyPrecision ) != 0 );
          }
        }
        if ( nonZeroPixel > 0 ) { blockToPatch[blockIndex] = patchIndex + 1; }
      }
    }
  }
}

void PCCCodec::generateBlockToPatchFromOccupancyMapVideo( PCCContext&  context,
                                                          const size_t occupancyResolution,
                                                          const size_t occupancyPrecision ) {
  for ( int fi = 0; fi < context.size(); fi++ ) {
    for ( size_t tileIdx = 0; tileIdx < context[fi].getNumTilesInAtlasFrame(); tileIdx++ ) {
      auto&                 tile           = context[fi].getTile( tileIdx );
      PCCImageOccupancyMap& occupancyImage = context.getVideoOccupancyMap().getFrame( fi );
      generateBlockToPatchFromOccupancyMapVideo( context, tile, fi, occupancyImage, occupancyResolution,
                                                 occupancyPrecision );
    }
  }
}

void PCCCodec::generateBlockToPatchFromOccupancyMapVideo( PCCContext&           context,
                                                          PCCFrameContext&      tile,
                                                          size_t                frameIdx,
                                                          PCCImageOccupancyMap& occupancyMapImage,
                                                          const size_t          occupancyResolution,
                                                          const size_t          occupancyPrecision ) {
  auto&        patches            = tile.getPatches();
  const size_t patchCount         = patches.size();
  const size_t blockToPatchWidth  = tile.getWidth() / occupancyResolution;
  const size_t blockToPatchHeight = tile.getHeight() / occupancyResolution;
  const size_t blockCount         = blockToPatchWidth * blockToPatchHeight;
  auto&        blockToPatch       = tile.getBlockToPatch();
  blockToPatch.resize( blockCount );
  std::fill( blockToPatch.begin(), blockToPatch.end(), 0 );
  for ( size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex ) {
    auto&  patch        = patches[patchIndex];
    size_t nonZeroPixel = 0;
    for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
      for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
        const size_t blockIndex = patch.patchBlock2CanvasBlock( u0, v0, blockToPatchWidth, blockToPatchHeight );
        nonZeroPixel            = 0;
        for ( size_t v1 = 0; v1 < patch.getOccupancyResolution(); ++v1 ) {
          const size_t v = v0 * patch.getOccupancyResolution() + v1;
          for ( size_t u1 = 0; u1 < patch.getOccupancyResolution(); ++u1 ) {
            const size_t u = u0 * patch.getOccupancyResolution() + u1;
            size_t       x;
            size_t       y;
            patch.patch2Canvas( u, v, tile.getWidth(), tile.getHeight(), x, y );
            x += tile.getLeftTopXInFrame();
            y += tile.getLeftTopYInFrame();
            nonZeroPixel += static_cast<unsigned long long>(
                occupancyMapImage.getValue( 0, x / occupancyPrecision, y / occupancyPrecision ) != 0 );
          }
        }
        if ( nonZeroPixel > 0 ) { blockToPatch[blockIndex] = patchIndex + 1; }
      }
    }
  }
}

void PCCCodec::generateAfti( PCCContext& context, size_t frameIndex,
                             AtlasFrameTileInformationRbsp& afti ) {  // encoder
  auto&partitionInfoPerFrame = context[frameIndex];
  afti.setSingleTileInAtlasFrameFlag( partitionInfoPerFrame.getNumTilesInAtlasFrame() == 1 );
  if ( partitionInfoPerFrame.getNumTilesInAtlasFrame() == 1 ) {
    if ( partitionInfoPerFrame.getTitleFrameContext().getUseRawPointsSeparateVideo() ) {
      afti.setAuxiliaryVideoTileRowWidthMinus1( context[frameIndex].getAuxVideoWidth() / 64 - 1 );
      for ( size_t ti = 0; ti < partitionInfoPerFrame.getNumTilesInAtlasFrame(); ti++ ) {
        afti.setAuxiliaryVideoTileRowHeight( ti, context[frameIndex].getAuxTileHeight( ti ) );
      }
    } else {
      afti.setAuxiliaryVideoTileRowWidthMinus1( 0 );
      for ( size_t ti = 0; ti < partitionInfoPerFrame.getNumTilesInAtlasFrame(); ti++ ) {
        afti.setAuxiliaryVideoTileRowHeight( ti, 0 );
      }
    }
    return;
  }
  afti.setUniformPartitionSpacingFlag( partitionInfoPerFrame.getUniformPartitionSpacing() );
  afti.setNumPartitionColumnsMinus1( partitionInfoPerFrame.getNumPartitionCols() - 1 );
  afti.setNumPartitionRowsMinus1( partitionInfoPerFrame.getNumPartitionRows() - 1 );
  for ( size_t col = 0; col < partitionInfoPerFrame.getNumPartitionCols(); col++ ) {
    afti.setPartitionColumnWidthMinus1( col, partitionInfoPerFrame.getPartitionWidth( col ) / 64 - 1 );
  }
  for ( size_t row = 0; row < partitionInfoPerFrame.getNumPartitionRows(); row++ ) {
    afti.setPartitionRowHeightMinus1( row, partitionInfoPerFrame.getPartitionHeight( row ) / 64 - 1 );
  }
  afti.setSinglePartitionPerTileFlag( partitionInfoPerFrame.getSinglePartitionPerTile() );
  afti.setNumTilesInAtlasFrameMinus1( partitionInfoPerFrame.getNumTilesInAtlasFrame() - 1 );
  if ( partitionInfoPerFrame.getSignalledTileId() == true ) { exit( 100 ); }
  afti.setSignalledTileIdFlag( partitionInfoPerFrame.getSignalledTileId() );
  if ( partitionInfoPerFrame.getSignalledTileId() ) {
    afti.setSignalledTileIdLengthMinus1( ceil( log2( partitionInfoPerFrame.getNumTilesInAtlasFrame() + 1 ) ) - 1 );
    for ( size_t ti = 0; ti < partitionInfoPerFrame.getNumTilesInAtlasFrame(); ti++ ) {
      afti.setTileId( ti, partitionInfoPerFrame.getTileId()[ti] );
    }
  } else {
    for ( size_t ti = 0; ti < partitionInfoPerFrame.getNumTilesInAtlasFrame(); ti++ ) { afti.setTileId( ti, ti ); }
  }
  auto&atlasFrame = context.getFrame( frameIndex );
  for ( size_t ti = 0; ti < partitionInfoPerFrame.getNumTilesInAtlasFrame(); ti++ ) {
    auto& tile = atlasFrame.getTile( ti );
    if ( tile.getPatches().size() == 0 &&
         partitionInfoPerFrame.getTitleFrameContext().getUseRawPointsSeparateVideo() ) {
      afti.setTopLeftPartitionIdx( ti, 0 );
      afti.setBottomRightPartitionColumnOffset( ti, 0 );
      afti.setBottomRightPartitionRowOffset( ti, 0 );
      printf( "enc:%zu frame %zu tile:(%zu,%zu), Raw Tile %zux%zu in Auxiliary video\n", frameIndex, ti,
              atlasFrame.getTile( ti ).getLeftTopXInFrame(), atlasFrame.getTile( ti ).getLeftTopYInFrame(),
              atlasFrame.getTile( ti ).getWidth(), atlasFrame.getTile( ti ).getHeight() );
      continue;
    }
    int    leftTopX          = tile.getLeftTopXInFrame();
    int    leftTopY          = tile.getLeftTopYInFrame();
    int    bottomRightDeltaX = tile.getWidth();
    int    bottomRightDeltaY = tile.getHeight();
    size_t numPartLeftX = 0, numPartLeftY = 0;
    while ( leftTopX > 0 ) {
      leftTopX -= partitionInfoPerFrame.getPartitionWidth( numPartLeftX );
      numPartLeftX += 1;
    }
    while ( leftTopY > 0 ) {
      leftTopY -= partitionInfoPerFrame.getPartitionHeight( numPartLeftY );
      numPartLeftY += 1;
    }
    size_t topLeftIdx = numPartLeftX + numPartLeftY * partitionInfoPerFrame.getNumPartitionCols();
    afti.setTopLeftPartitionIdx( ti, topLeftIdx );

    size_t numPartBottomX = 0, numPartBottomY = 0;
    while ( bottomRightDeltaX > 0 ) {
      bottomRightDeltaX -= partitionInfoPerFrame.getPartitionWidth( numPartLeftX + numPartBottomX );
      numPartBottomX += 1;
    }
    while ( bottomRightDeltaY > 0 ) {
      bottomRightDeltaY -= partitionInfoPerFrame.getPartitionHeight( numPartLeftY + numPartBottomY );
      numPartBottomY += 1;
    }
    assert( numPartBottomX >= 1 && numPartBottomY >= 1 );
    if ( ( numPartBottomX < 1 ) || ( numPartBottomY < 1 ) ) {
      printf(
          "enc:<error> %zu frame %zu tile:(%zu,%zu), %zux%zu -> "
          "leftIdx(%zu,%zu), bottom(%zu,%zu) : ((numPartBottomX<1) || (numPartBottomY <1))\n",
          frameIndex, ti, atlasFrame.getTile( ti ).getLeftTopXInFrame(), atlasFrame.getTile( ti ).getLeftTopYInFrame(),
          atlasFrame.getTile( ti ).getWidth(), atlasFrame.getTile( ti ).getHeight(), numPartLeftX, numPartLeftY,
          numPartBottomX, numPartBottomY );
      printf( "enc:colWidth,rowHeight\n" );
      for ( size_t ii = 0; ii < partitionInfoPerFrame.getNumPartitionWidth(); ii++ )
        printf( "\t%zu", partitionInfoPerFrame.getPartitionWidth( ii ) );
      printf( "\n" );
      for ( size_t ii = 0; ii < partitionInfoPerFrame.getNumPartitionHeight(); ii++ )
        printf( "\t%zu", partitionInfoPerFrame.getPartitionHeight( ii ) );
      printf( "\n" );
      exit( 125 );
    }

    afti.setBottomRightPartitionColumnOffset( ti, numPartBottomX - 1 );
    afti.setBottomRightPartitionRowOffset( ti, numPartBottomY - 1 );
    printf( "enc:%zu frame %zu tile:(%zu,%zu), %zux%zu -> leftIdx(%zu,%zu), bottom(%zu,%zu) -> %u,%u,%u\n", frameIndex,
            ti, atlasFrame.getTile( ti ).getLeftTopXInFrame(), atlasFrame.getTile( ti ).getLeftTopYInFrame(),
            atlasFrame.getTile( ti ).getWidth(), atlasFrame.getTile( ti ).getHeight(), numPartLeftX, numPartLeftY,
            numPartBottomX, numPartBottomY, afti.getTopLeftPartitionIdx( ti ),
            afti.getBottomRightPartitionColumnOffset( ti ), afti.getBottomRightPartitionRowOffset( ti ) );
  }
  if ( partitionInfoPerFrame.getTitleFrameContext().getUseRawPointsSeparateVideo() ) {
    afti.setAuxiliaryVideoTileRowWidthMinus1( context[frameIndex].getAuxVideoWidth() / 64 - 1 );
    for ( size_t ti = 0; ti < partitionInfoPerFrame.getNumTilesInAtlasFrame(); ti++ ) {
      afti.setAuxiliaryVideoTileRowHeight( ti, context[frameIndex].getAuxTileHeight( ti ) / 64 );
    }
    printf( "enc:%zu frame auxiliaryVideoTileRowWidthMinus1(): width(64x)%u\t height: ", frameIndex,
            afti.getAuxiliaryVideoTileRowWidthMinus1() );
    for ( size_t ti = 0; ti < partitionInfoPerFrame.getNumTilesInAtlasFrame(); ti++ )
      printf( "%u\t", afti.getAuxiliaryVideoTileRowHeight( ti ) );
    printf( "\n" );
  } else {
    afti.setAuxiliaryVideoTileRowWidthMinus1( 0 );
    for ( size_t ti = 0; ti < partitionInfoPerFrame.getNumTilesInAtlasFrame(); ti++ ) {
      afti.setAuxiliaryVideoTileRowHeight( ti, 0 );
    }
  }
  context[frameIndex].initNumTiles( context[frameIndex].getNumTilesInAtlasFrame() );
}

// sei hash
void PCCCodec::getHashPatchParams( PCCContext&                            context,
                                   size_t                                 frameIndex,
                                   size_t                                 tileIndex,
                                   std::vector<std::vector<PatchParams>>& tilePatchParams,
                                   std::vector<PatchParams>&              atlasPatchParams ) {
  auto&       tile          = context.getFrame( frameIndex ).getTile( tileIndex );
  size_t      atlIdx        = tile.getAtlIndex();
  auto&       atl           = context.getAtlasTileLayer( atlIdx );
  auto&       ath           = atl.getHeader();
  auto&       afps          = context.getAtlasFrameParameterSet( ath.getAtlasFrameParameterSetId() );
  auto&       asps          = context.getAtlasSequenceParameterSet( afps.getAtlasSequenceParameterSetId() );
  auto&       afti          = afps.getAtlasFrameTileInformationRbsp();
  size_t      topLeftColumn = afti.getTopLeftPartitionIdx( tileIndex ) % ( afti.getNumPartitionColumnsMinus1() + 1 );
  size_t      topLeftRow    = afti.getTopLeftPartitionIdx( tileIndex ) / ( afti.getNumPartitionColumnsMinus1() + 1 );
  size_t      tileOffsetX   = context[frameIndex].getPartitionPosX( topLeftColumn );
  size_t      tileOffsetY   = context[frameIndex].getPartitionPosY( topLeftRow );
  PCCTileType tileType      = ath.getType();
  size_t      patchCount    = tile.getPatches().size();
  for ( size_t patchIdx = 0; patchIdx < patchCount; patchIdx++ ) {
    PatchParams  pps;
    PCCPatchType currPatchType = getPatchType( tileType, atl.getDataUnit().getPatchMode( patchIdx ) );
    auto&        patch         = tile.getPatch( patchIdx );
    pps.patchType_             = PCCHashPatchType::PROJECTED;
    pps.patch2dPosX_           = patch.getU0();
    pps.patch2dPosY_           = patch.getV0();
    pps.patch2dSizeX_          = patch.getSizeU0();
    pps.patch2dSizeY_          = patch.getSizeV0();
    pps.patch3dOffsetU_        = patch.getU1();
    pps.patch3dOffsetV_        = patch.getV1();
    pps.patch3dOffsetD_        = patch.getD1();
    pps.patch3dRangeD_         = asps.getNormalAxisMaxDeltaValueEnabledFlag() ? patch.getSizeD() : 0;
    pps.patchOrientationIndex_ = patch.getPatchOrientation();
    pps.patchProjectionID_     = patch.getViewId();
    pps.patchInAuxVideo_       = 0;  // ajt::check
    pps.patchLoDScaleX_        = patch.getLodScaleX();
    pps.patchLoDScaleY_        = patch.getLodScaleY();
    if ( asps.getPLREnabledFlag() ) {
      pps.aspsMapCountMinus1_    = asps.getMapCountMinus1();
      pps.patchPackingBlockSize_ = size_t( 1 ) << asps.getLog2PatchPackingBlockSize();
      pps.plriMapPresentFlag_.resize( asps.getMapCountMinus1() + 1 );
      for ( size_t j = 0; j < asps.getMapCountMinus1() + 1; j++ ) {
        auto& plri                 = asps.getPLRInformation( j );
        pps.plriMapPresentFlag_[j] = plri.getMapEnabledFlag();
      }
      auto& pid = atl.getDataUnit().getPatchInformationData( patchIdx );
      if ( currPatchType == INTRA_PATCH ) {
        auto& pdu         = pid.getPatchDataUnit();
        pps.patchPLRData_ = pdu.getPLRData();
      } else if ( currPatchType == INTER_PATCH ) {
        auto& ipdu        = pid.getInterPatchDataUnit();
        pps.patchPLRData_ = ipdu.getPLRData();
      }
    }
    if ( tilePatchParams.size() != 0 ) { tilePatchParams[tileIndex].push_back( pps ); }
    pps.patch2dPosX_ += tileOffsetX;
    pps.patch2dPosY_ += tileOffsetY;
    atlasPatchParams.push_back( pps );
  }

  size_t rawPatchCount = tile.getRawPointsPatches().size();
  for ( size_t patchIdx = 0; patchIdx < rawPatchCount; patchIdx++ ) {
    PatchParams pps;
    auto&       rawPointsPatch = tile.getRawPointsPatch( patchIdx );
    pps.patchType_             = RAW;
    pps.patchRawPoints_        = rawPointsPatch.getNumberOfRawPoints();
    pps.patch2dPosX_           = rawPointsPatch.u0_;
    pps.patch2dPosY_           = rawPointsPatch.v0_;
    pps.patch2dSizeX_          = rawPointsPatch.sizeU0_;
    pps.patch2dSizeY_          = rawPointsPatch.sizeV0_;
    pps.patch3dOffsetU_        = rawPointsPatch.u1_;
    pps.patch3dOffsetV_        = rawPointsPatch.v1_;
    pps.patch3dOffsetD_        = rawPointsPatch.d1_;
    pps.patchInAuxVideo_       = rawPointsPatch.isPatchInAuxVideo_;
    if ( tilePatchParams.size() != 0 ) { tilePatchParams[tileIndex].push_back( pps ); }
    pps.patch2dPosX_ += tileOffsetX;
    pps.patch2dPosY_ += tileOffsetY;
    atlasPatchParams.push_back( pps );
  }

  size_t eomPatchCount = tile.getEomPatches().size();
  for ( size_t patchIdx = 0; patchIdx < eomPatchCount; patchIdx++ ) {
    PatchParams pps;
    auto&       eomPatch = tile.getEomPatch( patchIdx );
    assert( getPatchType( tileType, atl.getDataUnit().getPatchMode( patchCount + rawPatchCount + patchIdx ) ) ==
            EOM_PATCH );
    pps.patchType_                = EOM;
    pps.patch2dPosX_              = eomPatch.u0_;
    pps.patch2dPosY_              = eomPatch.v0_;
    pps.patch2dSizeX_             = eomPatch.sizeU_;
    pps.patch2dSizeY_             = eomPatch.sizeV_;
    pps.patchInAuxVideo_          = eomPatch.isPatchInAuxVideo_;
    pps.epduAssociatedPatchCount_ = eomPatch.memberPatches_.size();
    pps.epduAssociatedPoints_.resize( pps.epduAssociatedPatchCount_ );
    for ( int64_t i = 0; i < pps.epduAssociatedPatchCount_; i++ ) {
      pps.epduAssociatedPoints_[i] = eomPatch.eomCountPerPatch_[i];
    }
    if ( tilePatchParams.size() != 0 ) tilePatchParams[tileIndex].push_back( pps );
    pps.patch2dPosX_ += tileOffsetX;
    pps.patch2dPosY_ += tileOffsetY;
    atlasPatchParams.push_back( pps );
  }
}

void PCCCodec::getB2PHashPatchParams( PCCContext&                                     context,
                                      size_t                                          frameIndex,
                                      std::vector<std::vector<std::vector<int64_t>>>& b2pTilePatchParams,
                                      std::vector<std::vector<int64_t>>&              b2pAtlasPatchParams ) {
  size_t atlIdx                     = context[frameIndex].getTile( 0 ).getAtlIndex();
  auto&  tileHeader                 = context.getAtlasTileLayerList()[atlIdx].getHeader();
  size_t afpsIndex                  = tileHeader.getAtlasFrameParameterSetId();
  size_t aspsIndex                  = context.getAtlasFrameParameterSet( afpsIndex ).getAtlasSequenceParameterSetId();
  auto&  asps                       = context.getAtlasSequenceParameterSet( aspsIndex );
  auto&  afps                       = context.getAtlasFrameParameterSet( afpsIndex );
  auto&  afti                       = afps.getAtlasFrameTileInformationRbsp();
  size_t numTilesInPatchFrame       = context[frameIndex].getNumTilesInAtlasFrame();
  size_t patchPackingBlockSize      = size_t( 1 ) << asps.getLog2PatchPackingBlockSize();
  size_t offset                     = patchPackingBlockSize - 1;
  size_t atlasBlockToPatchMapWidth  = ( asps.getFrameWidth() + offset ) / patchPackingBlockSize;
  size_t atlasBlockToPatchMapHeight = ( asps.getFrameHeight() + offset ) / patchPackingBlockSize;

  b2pTilePatchParams.resize( numTilesInPatchFrame );
  b2pAtlasPatchParams.resize( atlasBlockToPatchMapHeight );
  for ( auto& e : b2pAtlasPatchParams ) e.resize( atlasBlockToPatchMapWidth );
  for ( size_t y = 0; y < atlasBlockToPatchMapHeight; y++ ) {
    for ( size_t x = 0; x < atlasBlockToPatchMapWidth; x++ ) { b2pAtlasPatchParams[y][x] = -1; }
  }
  size_t offsetPatch = 0;
  for ( size_t tileIdx = 0; tileIdx < numTilesInPatchFrame; tileIdx++ ) {
    auto&       tile           = context[frameIndex].getTile( tileIdx );
    size_t      atlIdx         = tile.getAtlIndex();
    auto&       ath            = context.getAtlasTileLayerList()[atlIdx].getHeader();
    auto&       atdu           = context.getAtlasTileLayerList()[atlIdx].getDataUnit();
    PCCTileType tileType       = ath.getType();
    size_t      topLeftColumn  = afti.getTopLeftPartitionIdx( tileIdx ) % ( afti.getNumPartitionColumnsMinus1() + 1 );
    size_t      topLeftRow     = afti.getTopLeftPartitionIdx( tileIdx ) / ( afti.getNumPartitionColumnsMinus1() + 1 );
    size_t      tileOffsetBlkX = context[frameIndex].getPartitionPosX( topLeftColumn ) / patchPackingBlockSize;
    size_t      tileOffsetBlkY = context[frameIndex].getPartitionPosY( topLeftRow ) / patchPackingBlockSize;
    size_t      tileBlockToPatchMapWidth  = ( tile.getWidth() + offset ) / patchPackingBlockSize;
    size_t      tileBlockToPatchMapHeight = ( tile.getHeight() + offset ) / patchPackingBlockSize;
    for ( auto& e : b2pTilePatchParams ) e.resize( tileBlockToPatchMapHeight );
    for ( size_t y = 0; y < tileBlockToPatchMapHeight; y++ ) {
      b2pTilePatchParams[tileIdx][y].resize( tileBlockToPatchMapWidth );
      for ( size_t x = 0; x < tileBlockToPatchMapWidth; x++ ) { b2pTilePatchParams[tileIdx][y][x] = -1; }
    }
    const size_t patchCount =
        tile.getPatches().size() + tile.getRawPointsPatches().size() + tile.getEomPatches().size();
    std::vector<size_t> p2pMap;
    p2pMap.resize( patchCount, 0 );
    size_t regularPatchCount = 0;
    size_t rawPatchCount     = 0;
    size_t eomPatchCount     = 0;
    for ( size_t p = 0; p < patchCount; p++ ) {
      if ( getPatchType( tileType, atdu.getPatchMode( p ) ) == RAW_PATCH ) {
        p2pMap[p] = rawPatchCount;
        rawPatchCount++;
      } else if ( getPatchType( tileType, atdu.getPatchMode( p ) ) == EOM_PATCH ) {
        p2pMap[p] = eomPatchCount;
        eomPatchCount++;
      } else if ( getPatchType( tileType, atdu.getPatchMode( p ) ) == INTRA_PATCH ||
                  getPatchType( tileType, atdu.getPatchMode( p ) ) == INTER_PATCH ||
                  getPatchType( tileType, atdu.getPatchMode( p ) ) == MERGE_PATCH ||
                  getPatchType( tileType, atdu.getPatchMode( p ) ) == SKIP_PATCH ) {
        p2pMap[p] = regularPatchCount;
        regularPatchCount++;
      }
    }
    assert( patchCount == ( regularPatchCount + rawPatchCount + eomPatchCount ) );
    assert( regularPatchCount == tile.getPatches().size() );
    assert( rawPatchCount == tile.getRawPointsPatches().size() );
    assert( eomPatchCount == tile.getEomPatches().size() );
    for ( size_t p = 0; p < patchCount; p++ ) {
      if ( getPatchType( tileType, atdu.getPatchMode( p ) ) != RAW_PATCH &&
           getPatchType( tileType, atdu.getPatchMode( p ) ) != EOM_PATCH ) {
        auto&  patch              = tile.getPatch( p2pMap[p] );
        size_t xOrg               = patch.getU0() / patchPackingBlockSize;
        size_t yOrg               = patch.getV0() / patchPackingBlockSize;
        size_t tilePatchWidthBlk  = ( patch.getSizeU0() + offset ) / patchPackingBlockSize;
        size_t tilePatchHeightBlk = ( patch.getSizeV0() + offset ) / patchPackingBlockSize;
        for ( size_t y = 0; y < tilePatchHeightBlk; y++ ) {
          for ( size_t x = 0; x < tilePatchWidthBlk; x++ ) {
            if ( ( asps.getPatchPrecedenceOrderFlag() == 0 ) ||
                 ( b2pTilePatchParams[tileIdx][yOrg + y][xOrg + x] == -1 ) ) {
              b2pTilePatchParams[tileIdx][yOrg + y][xOrg + x]                           = p;
              b2pAtlasPatchParams[tileOffsetBlkY + yOrg + y][tileOffsetBlkX + xOrg + x] = p + offsetPatch;
            }
          }
        }
      } else if ( getPatchType( tileType, atdu.getPatchMode( p ) ) == RAW_PATCH &&
                  atdu.getPatchInformationData( p ).getRawPatchDataUnit().getPatchInAuxiliaryVideoFlag() != 0 ) {
        auto&    rawPointsPatch     = tile.getRawPointsPatch( p2pMap[p] );
        uint32_t xOrg               = rawPointsPatch.u0_ / patchPackingBlockSize;
        uint32_t yOrg               = rawPointsPatch.v0_ / patchPackingBlockSize;
        uint32_t tilePatchWidthBlk  = ( rawPointsPatch.u0_ + offset ) / patchPackingBlockSize;
        uint32_t tilePatchHeightBlk = ( rawPointsPatch.v0_ + offset ) / patchPackingBlockSize;
        for ( uint32_t y = 0; y < tilePatchHeightBlk; y++ ) {
          for ( uint32_t x = 0; x < tilePatchWidthBlk; x++ ) {
            b2pTilePatchParams[tileIdx][y][x]                                         = p;
            b2pAtlasPatchParams[tileOffsetBlkY + yOrg + y][tileOffsetBlkX + xOrg + x] = p + offsetPatch;
          }
        }
      } else if ( getPatchType( tileType, atdu.getPatchMode( p ) ) == EOM_PATCH &&
                  atdu.getPatchInformationData( p ).getEomPatchDataUnit().getPatchInAuxiliaryVideoFlag() != 0 ) {
        auto&    eomPointsPatch     = tile.getEomPatch( p2pMap[p] );
        uint32_t xOrg               = eomPointsPatch.u0_ / patchPackingBlockSize;
        uint32_t yOrg               = eomPointsPatch.v0_ / patchPackingBlockSize;
        uint32_t tilePatchWidthBlk  = ( eomPointsPatch.u0_ + offset ) / patchPackingBlockSize;
        uint32_t tilePatchHeightBlk = ( eomPointsPatch.v0_ + offset ) / patchPackingBlockSize;
        for ( uint32_t y = 0; y < tilePatchHeightBlk; y++ ) {
          for ( uint32_t x = 0; x < tilePatchWidthBlk; x++ ) {
            b2pTilePatchParams[tileIdx][y][x]                                         = p;
            b2pAtlasPatchParams[tileOffsetBlkY + yOrg + y][tileOffsetBlkX + xOrg + x] = p + offsetPatch;
          }
        }
      }
    }
    offsetPatch += patchCount;
  }
}

void PCCCodec::aspsCommonByteString( std::vector<uint8_t>& stringByte, AtlasSequenceParameterSetRbsp& asps ) {
  uint8_t val = asps.getFrameWidth() & 0xFF;
  stringByte.push_back( val );
  val = ( size_t( asps.getFrameWidth() ) >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = ( size_t( asps.getFrameWidth() ) >> 16 ) & 0xFF;
  stringByte.push_back( val );
  val = ( size_t( asps.getFrameWidth() ) >> 24 ) & 0xFF;
  stringByte.push_back( val );
  val = asps.getFrameHeight() & 0xFF;
  stringByte.push_back( val );
  val = ( asps.getFrameHeight() >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = ( size_t( asps.getFrameHeight() ) >> 16 ) & 0xFF;
  stringByte.push_back( val );
  val = ( size_t( asps.getFrameHeight() ) >> 24 ) & 0xFF;
  stringByte.push_back( val );
  val = asps.getGeometry3dBitdepthMinus1() & 0xFF;
  stringByte.push_back( val );
  val = asps.getGeometry2dBitdepthMinus1() & 0xFF;
  stringByte.push_back( val );
  val = asps.getMapCountMinus1() & 0xFF;
  stringByte.push_back( val );
  val = asps.getMaxNumberProjectionsMinus1() & 0xFF;
  stringByte.push_back( val );
  val = uint8_t( asps.getPatchPrecedenceOrderFlag() ) & 0xFF;
  stringByte.push_back( val );
}

void PCCCodec::aspsApplicationByteString( std::vector<uint8_t>&          stringByte,
                                          AtlasSequenceParameterSetRbsp& asps,
                                          AtlasFrameParameterSetRbsp&    afps ) {
  uint8_t val;
  if ( asps.getPixelDeinterleavingFlag() ) {
    for ( int j = 0; j <= asps.getMapCountMinus1(); j++ ) {
      val = uint8_t( asps.getPixelDeinterleavingMapFlag( j ) ) & 0xFF;
      stringByte.push_back( val );
    }
  }
  val = uint8_t( asps.getRawPatchEnabledFlag() ) & 0xFF;
  stringByte.push_back( val );
  val = uint8_t( asps.getEomPatchEnabledFlag() ) & 0xFF;
  stringByte.push_back( val );
  if ( asps.getEomPatchEnabledFlag() && asps.getMapCountMinus1() == 0 ) {
    val = asps.getEomFixBitCountMinus1() & 0xFF;
    stringByte.push_back( val );
  }
  if ( asps.getAuxiliaryVideoEnabledFlag() && ( asps.getRawPatchEnabledFlag() || asps.getEomPatchEnabledFlag() ) ) {
    auto&   afti            = afps.getAtlasFrameTileInformationRbsp();
    size_t  auxVideoWidthNF = ( afti.getAuxiliaryVideoTileRowWidthMinus1() + 1 ) * 64;
    uint8_t val             = auxVideoWidthNF & 0xFF;
    stringByte.push_back( val );
    val = ( auxVideoWidthNF >> 8 ) & 0xFF;
    stringByte.push_back( val );
    size_t auxVideoHeightNF = 0;
    for ( uint32_t i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++ ) {
      auxVideoHeightNF += ( afti.getAuxiliaryVideoTileRowHeight( i ) * 64 );
    }
    val = auxVideoHeightNF & 0xFF;
    stringByte.push_back( val );
    val = ( auxVideoHeightNF >> 8 ) & 0xFF;
    stringByte.push_back( val );
  }
  val = uint8_t( asps.getPLREnabledFlag() ) & 0xFF;
  stringByte.push_back( val );
  if ( asps.getPLREnabledFlag() ) {
    for ( uint8_t i = 0; i < asps.getMapCountMinus1() + 1; i++ ) {
      val = uint8_t( asps.getPLRInformation( i ).getMapEnabledFlag() ) & 0xFF;  // plri_map_present_flag in the spec?
      stringByte.push_back( val );
      if ( asps.getPLRInformation( i ).getMapEnabledFlag() ) {
        val = asps.getPLRInformation( i ).getNumberOfModesMinus1() & 0xFF;
        stringByte.push_back( val );
        for ( int j = 0; j < asps.getPLRInformation( i ).getNumberOfModesMinus1(); j++ ) {
          val = uint8_t( asps.getPLRInformation( i ).getInterpolateFlag( j ) ) & 0xFF;
          stringByte.push_back( val );
          val = uint8_t( asps.getPLRInformation( i ).getFillingFlag( j ) ) & 0xFF;
          stringByte.push_back( val );
          val = asps.getPLRInformation( i ).getMinimumDepth( j ) & 0xFF;
          stringByte.push_back( val );
          val = asps.getPLRInformation( i ).getNeighbourMinus1( j ) & 0xFF;
          stringByte.push_back( val );
        }
        val = asps.getPLRInformation( i ).getBlockThresholdPerPatchMinus1() & 0xFF;
        stringByte.push_back( val );
      }
    }
  }
  auto& ext = asps.getAspsVpccExtension();
  val       = uint8_t( ext.getRemoveDuplicatePointEnableFlag() ) & 0xFF;
  stringByte.push_back( val );
  val = ext.getSurfaceThicknessMinus1() & 0xFF;
  stringByte.push_back( val );
  val = ( size_t( ext.getSurfaceThicknessMinus1() ) >> 8 ) & 0xFF;
  stringByte.push_back( val );
}

void PCCCodec::afpsCommonByteString( std::vector<uint8_t>& stringByte,
                                     PCCContext&           context,
                                     size_t                afpsIndex,
                                     size_t                frameIndex ) {
  auto    afps = context.getAtlasFrameParameterSet( afpsIndex );
  auto    afti = afps.getAtlasFrameTileInformationRbsp();
  uint8_t val  = afti.getNumTilesInAtlasFrameMinus1() & 0xFF;
  stringByte.push_back( val );
  std::vector<size_t> hashAuxTileHeight;
  auto                asps              = context.getAtlasSequenceParameterSet( afps.getAtlasSequenceParameterSetId() );
  size_t              prevAuxTileOffset = 0;
  if ( asps.getAuxiliaryVideoEnabledFlag() ) {
    hashAuxTileHeight.resize( afti.getNumTilesInAtlasFrameMinus1() + 1, 0 );
    for ( size_t ti = 0; ti <= afti.getNumTilesInAtlasFrameMinus1(); ti++ ) {
      hashAuxTileHeight[ti] = afti.getAuxiliaryVideoTileRowHeight( ti ) * 64;
    }
  } else {
    hashAuxTileHeight.resize( afti.getNumTilesInAtlasFrameMinus1() + 1, 0 );
  }

  for ( uint32_t i = 0; i < afti.getNumTilesInAtlasFrameMinus1() + 1; i++ ) {
    size_t topLeftColumn     = afti.getTopLeftPartitionIdx( i ) % ( afti.getNumPartitionColumnsMinus1() + 1 );
    size_t topLeftRow        = afti.getTopLeftPartitionIdx( i ) / ( afti.getNumPartitionColumnsMinus1() + 1 );
    size_t bottomRightColumn = topLeftColumn + afti.getBottomRightPartitionColumnOffset( i );
    size_t bottomRightRow    = topLeftRow + afti.getBottomRightPartitionRowOffset( i );
    size_t tileWidth         = 0;
    size_t tileHeight        = 0;
    size_t tileOffsetX       = context[frameIndex].getPartitionPosX( topLeftColumn );
    size_t tileOffsetY       = context[frameIndex].getPartitionPosY( topLeftRow );
    for ( int j = topLeftColumn; j <= bottomRightColumn; j++ ) {
      tileWidth += context[frameIndex].getPartitionWidth( j );
    }
    for ( int j = topLeftRow; j <= bottomRightRow; j++ ) { tileHeight += context[frameIndex].getPartitionHeight( j ); }
    size_t auxTileHeight = hashAuxTileHeight[i];
    size_t auxTileOffset = prevAuxTileOffset + auxTileHeight;
    prevAuxTileOffset    = auxTileOffset;
    val                  = tileOffsetX & 0xFF;
    stringByte.push_back( val );
    val = ( tileOffsetX >> 8 ) & 0xFF;
    stringByte.push_back( val );
    val = tileOffsetY & 0xFF;
    stringByte.push_back( val );
    val = ( tileOffsetY >> 8 ) & 0xFF;
    stringByte.push_back( val );
    val = tileWidth & 0xFF;
    stringByte.push_back( val );
    val = ( tileWidth >> 8 ) & 0xFF;
    stringByte.push_back( val );
    val = tileHeight & 0xFF;
    stringByte.push_back( val );
    val = ( tileHeight >> 8 ) & 0xFF;
    stringByte.push_back( val );
    val = auxTileOffset & 0xFF;
    stringByte.push_back( val );
    val = ( auxTileOffset >> 8 ) & 0xFF;
    stringByte.push_back( val );
    val = auxTileHeight & 0xFF;
    stringByte.push_back( val );
    val = ( auxTileHeight >> 8 ) & 0xFF;
    stringByte.push_back( val );
    if ( afps.getAtlasFrameTileInformationRbsp().getSignalledTileIdFlag() ) {
      val = afps.getAtlasFrameTileInformationRbsp().getTileId( i ) & 0xFF;  //
      stringByte.push_back( val );
      val = ( afps.getAtlasFrameTileInformationRbsp().getTileId( i ) >> 8 ) & 0xFF;
      stringByte.push_back( val );
    } else {
      val = i & 0xFF;
      stringByte.push_back( val );
      val = ( i >> 8 ) & 0xFF;
      stringByte.push_back( val );
    }
  }
}

void PCCCodec::afpsApplicationByteString( std::vector<uint8_t>&          stringByte,
                                          AtlasSequenceParameterSetRbsp& asps,
                                          AtlasFrameParameterSetRbsp&    afps ) {}

void PCCCodec::atlasPatchCommonByteString( std::vector<uint8_t>&     stringByte,
                                           size_t                    p,
                                           std::vector<PatchParams>& atlasPatchParams ) {
  uint8_t val = atlasPatchParams[p].patchType_ & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[p].patch2dPosX_ & 0xFF;
  stringByte.push_back( val );
  val = ( atlasPatchParams[p].patch2dPosX_ >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[p].patch2dPosY_ & 0xFF;
  stringByte.push_back( val );
  val = ( atlasPatchParams[p].patch2dPosY_ >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[p].patch2dSizeX_ & 0xFF;
  stringByte.push_back( val );
  val = ( atlasPatchParams[p].patch2dSizeX_ >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[p].patch2dSizeY_ & 0xFF;
  stringByte.push_back( val );
  val = ( atlasPatchParams[p].patch2dSizeY_ >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[p].patch3dOffsetU_ & 0xFF;
  stringByte.push_back( val );
  val = ( atlasPatchParams[p].patch3dOffsetU_ >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[p].patch3dOffsetV_ & 0xFF;
  stringByte.push_back( val );
  val = ( atlasPatchParams[p].patch3dOffsetV_ >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[p].patch3dOffsetD_ & 0xFF;
  stringByte.push_back( val );
  val = ( atlasPatchParams[p].patch3dOffsetD_ >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[p].patch3dRangeD_ & 0xFF;
  stringByte.push_back( val );
  val = ( atlasPatchParams[p].patch3dRangeD_ >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[p].patchProjectionID_ & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[p].patchOrientationIndex_ & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[p].patchLoDScaleX_ & 0xFF;
  stringByte.push_back( val );
  val = ( atlasPatchParams[p].patchLoDScaleX_ >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[p].patchLoDScaleY_ & 0xFF;
  stringByte.push_back( val );
  val = ( atlasPatchParams[p].patchLoDScaleY_ >> 8 ) & 0xFF;
  stringByte.push_back( val );
};

void PCCCodec::atlasPatchApplicationByteString( std::vector<uint8_t>&     stringByte,
                                                size_t                    p,
                                                std::vector<PatchParams>& atlasPatchParams ) {
  uint8_t val = atlasPatchParams[p].patchInAuxVideo_ & 0xFF;
  stringByte.push_back( val );
  if ( atlasPatchParams[p].patchType_ == RAW ) {
    val = atlasPatchParams[p].patchRawPoints_ & 0xFF;
    stringByte.push_back( val );
    val = ( atlasPatchParams[p].patchRawPoints_ >> 8 ) & 0xFF;
    stringByte.push_back( val );
  } else if ( atlasPatchParams[p].patchType_ == PROJECTED ) {
    size_t bSize = atlasPatchParams[p].patchPackingBlockSize_;
    for ( int m = 0; m < atlasPatchParams[p].aspsMapCountMinus1_ + 1; m++ ) {
      if ( atlasPatchParams[p].plriMapPresentFlag_[m] == 1 ) {
        auto&        plrd     = atlasPatchParams[p].patchPLRData_;
        const size_t blockCnt = plrd.getBlockToPatchMapWidth() * plrd.getBlockToPatchMapHeight();
        if ( plrd.getLevelFlag() == 0 ) {
          for ( int j = 0; j < blockCnt; j++ ) {
            val = plrd.getBlockModeMinus1( j ) & 0xFF;
            stringByte.push_back( val );
            val = (uint8_t)( (int)plrd.getBlockModeMinus1( j ) >> 8 ) & 0xFF;
            stringByte.push_back( val );
          }
        } else {
          val = plrd.getModeMinus1() & 0xFF;
          stringByte.push_back( val );
          val = (uint8_t)( (int)plrd.getModeMinus1() >> 8 ) & 0xFF;
          stringByte.push_back( val );
        }
      }
    }
  } else if ( atlasPatchParams[p].patchType_ == EOM ) {
    val = atlasPatchParams[p].epduAssociatedPatchCount_ & 0xFF;
    stringByte.push_back( val );
    val = ( atlasPatchParams[p].epduAssociatedPatchCount_ >> 8 ) & 0xFF;
    stringByte.push_back( val );
    for ( int i = 0; i < atlasPatchParams[p].epduAssociatedPatchCount_; i++ ) {
      val = atlasPatchParams[p].epduAssociatedPoints_[i] & 0xFF;
      stringByte.push_back( val );
      val = ( atlasPatchParams[p].epduAssociatedPoints_[i] >> 8 ) & 0xFF;
      stringByte.push_back( val );
    }
  }
};

void PCCCodec::tilePatchCommonByteString( std::vector<uint8_t>&                  stringByte,
                                          size_t                                 tileId,
                                          size_t                                 p,
                                          std::vector<std::vector<PatchParams>>& tilePatchParams ) {
  uint8_t val = tilePatchParams[tileId][p].patchType_ & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[tileId][p].patch2dPosX_ & 0xFF;
  stringByte.push_back( val );
  val = ( tilePatchParams[tileId][p].patch2dPosX_ >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[tileId][p].patch2dPosY_ & 0xFF;
  stringByte.push_back( val );
  val = ( tilePatchParams[tileId][p].patch2dPosY_ >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[tileId][p].patch2dSizeX_ & 0xFF;
  stringByte.push_back( val );
  val = ( tilePatchParams[tileId][p].patch2dSizeX_ >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[tileId][p].patch2dSizeY_ & 0xFF;
  stringByte.push_back( val );
  val = ( tilePatchParams[tileId][p].patch2dSizeY_ >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[tileId][p].patch3dOffsetU_ & 0xFF;
  stringByte.push_back( val );
  val = ( tilePatchParams[tileId][p].patch3dOffsetU_ >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[tileId][p].patch3dOffsetV_ & 0xFF;
  stringByte.push_back( val );
  val = ( tilePatchParams[tileId][p].patch3dOffsetV_ >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[tileId][p].patch3dOffsetD_ & 0xFF;
  stringByte.push_back( val );
  val = ( tilePatchParams[tileId][p].patch3dOffsetD_ >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[tileId][p].patch3dRangeD_ & 0xFF;
  stringByte.push_back( val );
  val = ( tilePatchParams[tileId][p].patch3dRangeD_ >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[tileId][p].patchProjectionID_ & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[tileId][p].patchOrientationIndex_ & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[tileId][p].patchLoDScaleX_ & 0xFF;
  stringByte.push_back( val );
  val = ( tilePatchParams[tileId][p].patchLoDScaleX_ >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[tileId][p].patchLoDScaleY_ & 0xFF;
  stringByte.push_back( val );
  val = ( tilePatchParams[tileId][p].patchLoDScaleY_ >> 8 ) & 0xFF;
  stringByte.push_back( val );
};

void PCCCodec::tilePatchApplicationByteString( std::vector<uint8_t>&                  stringByte,
                                               size_t                                 tileId,
                                               size_t                                 p,
                                               std::vector<std::vector<PatchParams>>& tilePatchParams ) {
  uint8_t val = tilePatchParams[tileId][p].patchInAuxVideo_ & 0xFF;
  stringByte.push_back( val );
  if ( tilePatchParams[tileId][p].patchType_ == RAW ) {
    val = tilePatchParams[tileId][p].patchRawPoints_ & 0xFF;
    stringByte.push_back( val );
    val = ( tilePatchParams[tileId][p].patchRawPoints_ >> 8 ) & 0xFF;
    stringByte.push_back( val );
  } else if ( tilePatchParams[tileId][p].patchType_ == PROJECTED ) {
    size_t bSize    = tilePatchParams[tileId][p].patchPackingBlockSize_;
    size_t blockCnt = ( ( tilePatchParams[tileId][p].patch2dSizeX_ + bSize - 1 ) / bSize ) *
                      ( ( tilePatchParams[tileId][p].patch2dSizeY_ + bSize - 1 ) / bSize );
    for ( int m = 0; m < tilePatchParams[tileId][p].aspsMapCountMinus1_ + 1; m++ ) {
      if ( tilePatchParams[tileId][p].plriMapPresentFlag_[m] == 1 ) {
        if ( tilePatchParams[tileId][p].patchPLRData_.getLevelFlag() == 0 ) {
          for ( int j = 0; j < blockCnt; j++ ) {
            val = tilePatchParams[tileId][p].patchPLRData_.getBlockModeMinus1( j ) & 0xFF;
            stringByte.push_back( val );
            val = (uint8_t)( (int)tilePatchParams[tileId][p].patchPLRData_.getBlockModeMinus1( j ) >> 8 ) & 0xFF;
            stringByte.push_back( val );
          }
        } else {
          val = tilePatchParams[tileId][p].patchPLRData_.getModeMinus1() & 0xFF;
          stringByte.push_back( val );
          val = (uint8_t)( (int)tilePatchParams[tileId][p].patchPLRData_.getModeMinus1() >> 8 ) & 0xFF;
          stringByte.push_back( val );
        }
      }
    }
  } else if ( tilePatchParams[tileId][p].patchType_ == EOM ) {
    // val = tilePatchParams[ tileId ][ p ].patchEomPatchCount[ p ] & 0xFF; // ajt:: this needs to be checked
    // stringByte.push_back( val );
    // val = (tilePatchParams[ tileId ][ p ].patchEomPatchCount[ p ] >> 8 ) & 0xFF;
    // stringByte.push_back( val );
    val = tilePatchParams[tileId][p].epduAssociatedPatchCount_ & 0xFF;
    stringByte.push_back( val );
    val = ( tilePatchParams[tileId][p].epduAssociatedPatchCount_ >> 8 ) & 0xFF;
    stringByte.push_back( val );
    for ( int i = 0; i < tilePatchParams[tileId][p].epduAssociatedPatchCount_; i++ ) {
      val = tilePatchParams[tileId][p].epduAssociatedPoints_[i] & 0xFF;
      stringByte.push_back( val );
      val = ( tilePatchParams[tileId][p].epduAssociatedPoints_[i] >> 8 ) & 0xFF;
      stringByte.push_back( val );
    }
  }
};
void PCCCodec::atlasBlockToPatchByteString( std::vector<uint8_t>&             stringByte,
                                            std::vector<std::vector<int64_t>> atlasB2p ) {
  uint8_t b2pVal;
  for ( size_t y = 0; y < atlasB2p.size(); y++ ) {
    for ( size_t x = 0; x < atlasB2p[y].size(); x++ ) {
      b2pVal = ( atlasB2p[y][x] == -1 ) ? 0xFFFF : atlasB2p[y][x];
      stringByte.push_back( b2pVal & 0xFF );
      stringByte.push_back( ( size_t( b2pVal ) >> 8 ) & 0xFF );
    }
  }
}

void PCCCodec::tileBlockToPatchByteString( std::vector<uint8_t>&                          stringByte,
                                           size_t                                         tileId,
                                           std::vector<std::vector<std::vector<int64_t>>> tileB2p ) {
  uint8_t b2pVal;
  for ( size_t y = 0; y < tileB2p[tileId].size(); y++ ) {
    for ( size_t x = 0; x < tileB2p[tileId][y].size(); x++ ) {
      b2pVal = ( tileB2p[tileId][y][x] == -1 ) ? 0xFFFF : tileB2p[tileId][y][x];
      stringByte.push_back( b2pVal & 0xFF );
      stringByte.push_back( ( size_t( b2pVal ) >> 8 ) & 0xFF );
    }
  }
}

void PCCCodec::inverseRotatePosition45DegreeOnAxis( size_t axis, size_t lod, PCCPoint3D input, PCCVector3D& output ) {
  size_t s = ( 1u << ( lod - 1 ) ) - 1;
  output   = input;
  if ( axis == 1 ) {  // projection plane is defined by Y Axis.
    output.x() = input.x() - input.z() + s;
    output.x() /= 2.0;
    output.z() = input.x() + input.z() - s;
    output.z() /= 2.0;
  }
  if ( axis == 2 ) {  // projection plane is defined by X Axis.
    output.z() = input.z() - input.y() + s;
    output.z() /= 2.0;
    output.y() = input.z() + input.y() - s;
    output.y() /= 2.0;
  }
  if ( axis == 3 ) {  // projection plane is defined by Z Axis.
    output.y() = input.y() - input.x() + s;
    output.y() /= 2.0;
    output.x() = input.y() + input.x() - s;
    output.x() /= 2.0;
  }
}

#ifdef CODEC_TRACE
void PCCCodec::printChecksum( PCCPointSet3& ePointcloud, std::string eString ) {
  auto checksum = ePointcloud.computeChecksum();
  TRACE_CODEC( "Checksum %s: ", eString.c_str() );
  for ( auto& c : checksum ) { TRACE_CODEC( "%02x", c ); }
  TRACE_CODEC( "%s\n", "" );
  printf( "Checksum %s: ", eString.c_str() );
  for ( auto& c : checksum ) { printf( "%02x", c ); }
  printf( "\n" );
  fflush( stdout );
}
#endif
