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
#include "tbb/tbb.h"
#include "PCCKdTree.h"
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCGroupOfFrames.h"
#include "PCCPatch.h"

#include "PCCCodec.h"

using namespace pcc;

PCCCodec::PCCCodec() {
#ifdef BITSTREAM_TRACE
  trace_     = false;
  traceFile_ = NULL;
#endif
}
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
      std::vector<int> cellIndex;
      cellIndex.resize( w * w * w );
      std::fill( cellIndex.begin(), cellIndex.end(), -1 );
      size_t    numBoundaryCells = 0;
      const int disth            = ( std::max )( static_cast<int>( params.gridSize_ ) / 2, 1 );
      const int th               = params.gridSize_ * w;
      for ( size_t n = 0; n < pointCount; ++n ) {
        if ( reconstruct.getBoundaryPointType( n ) == 1 ) {
          PCCPoint3D point = reconstruct[n];
          int        x     = point.x();
          int        y     = point.y();
          int        z     = point.z();

          if ( x < disth || y < disth || z < disth || th <= x + disth || th <= y + disth || th <= z + disth ) {
            continue;
          }

          int x2 = x / params.gridSize_;
          int y2 = y / params.gridSize_;
          int z2 = z / params.gridSize_;

          int x3 = x % params.gridSize_;
          int y3 = y % params.gridSize_;
          int z3 = z % params.gridSize_;

          int qx = x2 + ( ( x3 < params.gridSize_ / 2 ) ? -1 : 0 );
          int qy = y2 + ( ( y3 < params.gridSize_ / 2 ) ? -1 : 0 );
          int qz = z2 + ( ( z3 < params.gridSize_ / 2 ) ? -1 : 0 );

          for ( int ix = 0; ix < 2; ix++ ) {
            for ( int iy = 0; iy < 2; iy++ ) {
              for ( int iz = 0; iz < 2; iz++ ) {
                int x4     = qx + ix;
                int y4     = qy + iy;
                int z4     = qz + iz;
                int cellId = x4 + y4 * w + z4 * w * w;
                if ( cellIndex[cellId] == -1 ) {
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
        PCCPoint3D point  = reconstruct[j];
        int        x2     = point.x() / params.gridSize_;
        int        y2     = point.y() / params.gridSize_;
        int        z2     = point.z() / params.gridSize_;
        int        cellId = x2 + y2 * w + z2 * w * w;
        if ( cellIndex[cellId] != -1 ) {
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
  const size_t     gridSize         = params.occupancyPrecision_;
  int              pcMaxSize        = pow( 2, params.geometryBitDepth3D_ );
  const size_t     w                = pcMaxSize / gridSize;
  const size_t     w3               = w * w * w;
  size_t           pointCount       = reconstruct.getPointCount();
  size_t           numBoundaryCells = 0;
  const size_t     disth            = ( std::max )( gridSize / 2, (size_t)1 );
  std::vector<int> cellIndex;
  cellIndex.resize( w3 );
  std::fill( cellIndex.begin(), cellIndex.end(), -1 );
  assert( params.flagColorSmoothing_ );
  TRACE_CODEC( "colorSmoothing \n" );
  TRACE_CODEC( "  geometryBitDepth3D_       = %zu \n", params.geometryBitDepth3D_ );
  TRACE_CODEC( "  thresholdColorVariation_  = %f \n", params.thresholdColorVariation_ );
  TRACE_CODEC( "  thresholdColorDifference_ = %f \n", params.thresholdColorDifference_ );
  TRACE_CODEC( "  pcMaxSize = %d \n", pcMaxSize );
  TRACE_CODEC( "  gridSize  = %zu \n", gridSize );
  TRACE_CODEC( "  w         = %zu \n", w );
  TRACE_CODEC( "  w3        = %zu \n", w3 );
  for ( size_t n = 0; n < pointCount; ++n ) {
    if ( reconstruct.getBoundaryPointType( n ) == 1 ) {
      PCCPoint3D point = reconstruct[n];
      int        x     = point.x();
      int        y     = point.y();
      int        z     = point.z();
      if ( x < disth || y < disth || z < disth || pcMaxSize <= x + disth || pcMaxSize <= y + disth ||
           pcMaxSize <= z + disth ) {
        continue;
      }
      int x2 = x / gridSize;
      int y2 = y / gridSize;
      int z2 = z / gridSize;
      int x3 = x % gridSize;
      int y3 = y % gridSize;
      int z3 = z % gridSize;
      int qx = x2 + ( ( x3 < gridSize / 2 ) ? -1 : 0 );
      int qy = y2 + ( ( y3 < gridSize / 2 ) ? -1 : 0 );
      int qz = z2 + ( ( z3 < gridSize / 2 ) ? -1 : 0 );
      for ( int ix = 0; ix < 2; ix++ ) {
        for ( int iy = 0; iy < 2; iy++ ) {
          for ( int iz = 0; iz < 2; iz++ ) {
            int x4     = qx + ix;
            int y4     = qy + iy;
            int z4     = qz + iz;
            int cellId = x4 + y4 * w + z4 * w * w;
            if ( cellIndex[cellId] == -1 ) {
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
  std::fill( colorSmoothingCenter_.begin(), colorSmoothingCenter_.end(), 0 );
  std::fill( colorSmoothingCount_.begin(), colorSmoothingCount_.end(), 0 );
  std::pair<size_t, size_t> initPair;
  initPair.first = initPair.second = 0;
  std::fill( colorSmoothingPartition_.begin(), colorSmoothingPartition_.end(), initPair );
  std::fill( colorSmoothingDoSmooth_.begin(), colorSmoothingDoSmooth_.end(), 0 );
  colorSmoothingLum_.clear();
  colorSmoothingLum_.resize( numBoundaryCells );
  for ( int k = 0; k < reconstruct.getPointCount(); k++ ) {
    PCCPoint3D point  = reconstruct[k];
    int        x2     = point.x() / gridSize;
    int        y2     = point.y() / gridSize;
    int        z2     = point.z() / gridSize;
    int        cellId = x2 + y2 * w + z2 * w * w;
    if ( cellId >= cellIndex.size() ) {
      TRACE_CODEC( " cellId >  cellIndex.size() <=>  %zu > %zu \n", cellId, cellIndex.size() );
    } else {
      if ( cellIndex[cellId] != -1 ) {
        PCCColor16bit color16bit = reconstruct.getColor16bit( k );
        PCCVector3D   clr;
        for ( size_t c = 0; c < 3; ++c ) { clr[c] = double( color16bit[c] ); }
        auto tilePatchIndexPlusOne   = reconstruct.getPointPatchIndex( k );
        tilePatchIndexPlusOne.second = tilePatchIndexPlusOne.second + 1;
        addGridColorCentroid( reconstruct[k], clr, tilePatchIndexPlusOne, colorSmoothingCount_, colorSmoothingCenter_,
                              colorSmoothingPartition_, colorSmoothingDoSmooth_, gridSize, colorSmoothingLum_, params,
                              cellIndex[cellId] );
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
  int    deltaMax = 0;
  double dOrg     = patch.generateNormalCoordinate( frame.getValue( 0, xOrg, yOrg ) );
  for ( int x = ( std::max )( 0, xOrg - neighboring );
        x <= ( std::min )( xOrg + neighboring, static_cast<int>( frame.getWidth() ) ); x += 1 ) {
    for ( int y = ( std::max )( 0, yOrg - neighboring );
          y <= ( std::min )( yOrg + neighboring, static_cast<int>( frame.getHeight() ) ); y += 1 ) {
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
  const auto& patch  = tile.getPatch( patchIndex );
  auto&       frame0 = videoGeometryMultiple[0].getFrame( videoFrameIndex );
  // params.multipleStreams_ ? videoGeometryMultiple[0].getFrame(
  // videoFrameIndex ) : videoGeometry.getFrame(
  // videoFrameIndex );
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
      occupancyTop = y > 0 && ( patch.getOccupancyMap( ( x ) / patch.getOccupancyResolution(),
                                                       ( y - 1 ) / patch.getOccupancyResolution() ) != 0 );
      occupancyBotton =
          y < ( imageHeight - 1 ) && ( patch.getOccupancyMap( ( x ) / patch.getOccupancyResolution(),
                                                              ( y + 1 ) / patch.getOccupancyResolution() ) != 0 );
      occupancyLeft = x > 0 && ( patch.getOccupancyMap( ( x - 1 ) / patch.getOccupancyResolution(),
                                                        ( y ) / patch.getOccupancyResolution() ) != 0 );
      occupancyRight =
          x < ( imageWidth - 1 ) && ( patch.getOccupancyMap( ( x + 1 ) / patch.getOccupancyResolution(),
                                                             ( y ) / patch.getOccupancyResolution() ) != 0 );
    }
    auto&        blockToPatch      = tile.getBlockToPatch();
    const size_t blockToPatchWidth = tile.getWidth() / params.occupancyResolution_;
    double       DepthNeighbors[4] = {0};
    int          count             = 0;
    double       minimumDepth      = point0[patch.getNormalAxis()];
    double       maximumDepth      = point0[patch.getNormalAxis()];
    if ( occupancyLeft ) {
      size_t Temp_u0 = ( x - 1 ) / patch.getOccupancyResolution();
      size_t Temp_v0 = ( y ) / patch.getOccupancyResolution();
      if ( blockToPatch[(Temp_v0)*blockToPatchWidth + Temp_u0] == patchIndexPlusOne ) {
        if ( patch.getProjectionMode() == 0 ) {
          DepthNeighbors[0] = double( frame0.getValue( 0, x - 1, y ) + patch.getD1() );
        } else {
          DepthNeighbors[0] = double( patch.getD1() - frame0.getValue( 0, x - 1, y ) );
        }
        count++;
        if ( DepthNeighbors[0] < minimumDepth ) { minimumDepth = DepthNeighbors[0]; }
        if ( DepthNeighbors[0] > maximumDepth ) { maximumDepth = DepthNeighbors[0]; }
      }
    }
    if ( occupancyRight ) {
      size_t Temp_u0 = ( x + 1 ) / patch.getOccupancyResolution();
      size_t Temp_v0 = ( y ) / patch.getOccupancyResolution();
      if ( blockToPatch[(Temp_v0)*blockToPatchWidth + Temp_u0] == patchIndexPlusOne ) {
        if ( patch.getProjectionMode() == 0 ) {
          DepthNeighbors[1] = double( frame0.getValue( 0, x + 1, y ) + patch.getD1() );
        } else {
          DepthNeighbors[1] = double( patch.getD1() - frame0.getValue( 0, x + 1, y ) );
        }
        count++;
        if ( DepthNeighbors[1] < minimumDepth ) { minimumDepth = DepthNeighbors[1]; }
        if ( DepthNeighbors[1] > maximumDepth ) { maximumDepth = DepthNeighbors[1]; }
      }
    }
    if ( occupancyTop ) {
      size_t Temp_u0 = ( x ) / patch.getOccupancyResolution();
      size_t Temp_v0 = ( y - 1 ) / patch.getOccupancyResolution();
      if ( blockToPatch[(Temp_v0)*blockToPatchWidth + Temp_u0] == patchIndexPlusOne ) {
        if ( patch.getProjectionMode() == 0 ) {
          DepthNeighbors[2] = double( frame0.getValue( 0, x, y - 1 ) + patch.getD1() );
        } else {
          DepthNeighbors[2] = double( patch.getD1() - frame0.getValue( 0, x, y - 1 ) );
        }
        count++;
        if ( DepthNeighbors[2] < minimumDepth ) { minimumDepth = DepthNeighbors[2]; }
        if ( DepthNeighbors[2] > maximumDepth ) { maximumDepth = DepthNeighbors[2]; }
      }
    }
    if ( occupancyBotton ) {
      size_t Temp_u0 = ( x ) / patch.getOccupancyResolution();
      size_t Temp_v0 = ( y + 1 ) / patch.getOccupancyResolution();
      if ( blockToPatch[(Temp_v0)*blockToPatchWidth + Temp_u0] == patchIndexPlusOne ) {
        if ( patch.getProjectionMode() == 0 ) {
          DepthNeighbors[3] = double( frame0.getValue( 0, x, y + 1 ) + patch.getD1() );
        } else {
          DepthNeighbors[3] = double( patch.getD1() - frame0.getValue( 0, x, y + 1 ) );
        }
        count++;
        if ( DepthNeighbors[3] < minimumDepth ) { minimumDepth = DepthNeighbors[3]; }
        if ( DepthNeighbors[3] > maximumDepth ) { maximumDepth = DepthNeighbors[3]; }
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
            ( std::min )( ( DepthNeighbors[0] + DepthNeighbors[1] + DepthNeighbors[2] + DepthNeighbors[3] ) / count,
                          depth0 + params.surfaceThickness_ ),
            depth0 ) );
      } else {
        interpolateD1[patch.getNormalAxis()] = round( ( std::min )(
            ( std::max )( ( DepthNeighbors[0] + DepthNeighbors[1] + DepthNeighbors[2] + DepthNeighbors[3] ) / count,
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
          getDeltaNeighbors( frame0, patch, x, y, neighbor, NeighborThreshold, patch.getProjectionMode() != 0U );
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
  reconstruct.addColors();

  printf( "generatePointCloud pbfEnableFlag_ = %d \n", params.pbfEnableFlag_ );
  TRACE_CODEC( "generatePointCloud pbfEnableFlag_ = %d \n", params.pbfEnableFlag_ );
  if ( params.pbfEnableFlag_ ) {
    PatchBlockFiltering patchBlockFiltering;
    patchBlockFiltering.setPatches( &( tile.getPatches() ) );
    patchBlockFiltering.setBlockToPatch( &( tile.getBlockToPatch() ) );
    patchBlockFiltering.setOccupancyMapEncoder( &( tile.getOccupancyMap() ) );
    patchBlockFiltering.setOccupancyMapVideo(
        &( context.getVideoOccupancyMap().getFrame( tile.getFrameIndex() ).getChannel( 0 ) ) );
    patchBlockFiltering.setGeometryVideo(
        &( videoGeometry.getFrame( tile.getFrameIndex() * ( params.mapCountMinus1_ + 1 ) ).getChannel( 0 ) ) );
    patchBlockFiltering.patchBorderFiltering( tile.getWidth(), tile.getHeight(), params.occupancyResolution_,
                                              params.occupancyPrecision_,
                                              !params.enhancedOccupancyMapCode_ ? params.thresholdLossyOM_ : 0,
                                              params.pbfPassesCount_, params.pbfFilterSize_, params.pbfLog2Threshold_ );
    printf( "PBF done \n" );
    TRACE_CODEC( "PBF done \n" );
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
      size_t nonZeroPixel      = 0;
      size_t patchSizeXInPixel = ( patch.getPatchSize2DXInPixel() / quantizerSizeX ) * quantizerSizeX;
      size_t patchSizeYInPixel = ( patch.getPatchSize2DYInPixel() / quantizerSizeY ) * quantizerSizeY;
      if ( tile.getLog2PatchQuantizerSizeX() == 0 ) { assert( patchSizeXInPixel == patch.getPatchSize2DXInPixel() ); }
      if ( tile.getLog2PatchQuantizerSizeY() == 0 ) { assert( patchSizeYInPixel == patch.getPatchSize2DYInPixel() ); }
      for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
        for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
          const size_t blockIndex = patch.patchBlock2CanvasBlock( u0, v0, blockToPatchWidth, blockToPatchHeight );
          if ( blockToPatch[blockIndex] == ( patchIndex + 1 ) ) {
            nonZeroPixel = 0;
            for ( size_t v1 = 0; v1 < patch.getOccupancyResolution(); ++v1 ) {
              const size_t v = v0 * patch.getOccupancyResolution() + v1;
              for ( size_t u1 = 0; u1 < patch.getOccupancyResolution(); ++u1 ) {
                const size_t u = u0 * patch.getOccupancyResolution() + u1;
                if ( u >= patchSizeXInPixel || v >= patchSizeYInPixel ) {
                  size_t x;
                  size_t y;
                  occupancyMap[patch.patch2Canvas( u, v, tile.getWidth(), tile.getHeight(), x, y )] = 0;
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

  bool         useRawPointsSeparateVideo = tile.getUseRawPointsSeparateVideo();
  size_t       videoFrameIndex;
  const size_t mapCount = params.mapCountMinus1_ + 1;
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
  uint32_t index;

  for ( index = 0; index < patches.size(); index++ ) {
    patchIndex = ( bDecoder && context.getAtlasSequenceParameterSet( 0 ).getPatchPrecedenceOrderFlag() )
                     ? ( totalPatchCount - index - 1 )
                     : index;
    const size_t patchIndexPlusOne = patchIndex + 1;
    auto&        patch             = patches[patchIndex];
    PCCColor3B   color( uint8_t( 0 ) );
    TRACE_CODEC(
        "P%2lu/%2lu: 2D=(%2lu,%2lu)*(%2lu,%2lu) 3D(%4zu,%4zu,%4zu)*(%4zu,%4zu) "
        "A=(%zu,%zu,%zu) Or=%zu P=%zu => %zu "
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

              bool   occupancy     = false;
              size_t canvasIndex   = patch.patch2Canvas( u, v, tileWidth, tileHeight, x, y );
              size_t xInVideoFrame = x + tile.getLeftTopXInFrame();
              size_t yInVideoFrame = y + tile.getLeftTopYInFrame();

              if ( params.pbfEnableFlag_ ) {
                occupancy = patch.getOccupancyMap( u, v ) != 0;
              } else {
                occupancy = occupancyMap[canvasIndex] != 0;
              }
              if ( !occupancy ) { continue; }
              // TRACE_CODEC( "B %4zu ci %9zu  ocm %1lu xy = %4zu %4zu
              // \n",blockIndex, canvasIndex, occupancy, x,y );
              if ( params.enhancedOccupancyMapCode_ ) {
                // D0
                PCCPoint3D point0 = patch.generatePoint( u, v, frame0.getValue( 0, xInVideoFrame, yInVideoFrame ) );
                size_t     pointIndex0;  // = reconstruct.addPoint(point0);
                if ( patch.getAxisOfAdditionalPlane() == 0 ) {
                  pointIndex0 = reconstruct.addPoint( point0 );
                } else {
                  PCCVector3D tmp;
                  PCCPatch::InverseRotatePosition45DegreeOnAxis( patch.getAxisOfAdditionalPlane(),
                                                                 params.geometryBitDepth3D_, point0, tmp );
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
                      PCCPatch::InverseRotatePosition45DegreeOnAxis( patch.getAxisOfAdditionalPlane(),
                                                                     params.geometryBitDepth3D_, point1, tmp );
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
                        // pointIndex1 = reconstruct.addPoint( point1 );
                        if ( patch.getAxisOfAdditionalPlane() == 0 ) {
                          pointIndex1 = reconstruct.addPoint( point1 );
                        } else {
                          PCCVector3D tmp;
                          PCCPatch::InverseRotatePosition45DegreeOnAxis( patch.getAxisOfAdditionalPlane(),
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
                          PCCPatch::InverseRotatePosition45DegreeOnAxis( patch.getAxisOfAdditionalPlane(),
                                                                         params.geometryBitDepth3D_, point1, tmp );
                          eomPointsPerPatch[patchIndex].push_back( PCCPoint3D( tmp[0], tmp[1], tmp[2] ) );
                        }
                      }
                      addedPointCount++;
                    }
                  }  // for each bit of EOM code
                  if ( PCC_SAVE_POINT_TYPE == 1 ) { reconstruct.setType( pointIndex1, POINT_D1 ); }
                  // Without "Identify boundary points" & "1st Extension
                  // boundary region" as EOM code is only for
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
                        PCCPatch::InverseRotatePosition45DegreeOnAxis(
                            patch.getAxisOfAdditionalPlane(), params.geometryBitDepth3D_, createdPoints[i], tmp );
                        pointindex = reconstruct.addPoint( tmp );
                        reconstruct.setPointPatchIndex( pointindex, tileIndex, patchIndex );
                      }
                      const size_t pointindex_1 = pointindex;
                      reconstruct.setColor( pointindex_1, color );
                      if ( PCC_SAVE_POINT_TYPE == 1 ) {
                        if ( params.singleMapPixelInterleaving_ ) {
                          size_t flag;
                          flag = ( i == 0 ) ? ( x + y ) % 2 : ( i == 1 ) ? ( x + y + 1 ) % 2 : IntermediateLayerIndex;
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
                                   : i == 1 ? ( static_cast<size_t>( x + y + 1 ) % 2 ) : IntermediateLayerIndex );
                      } else if ( params.pointLocalReconstruction_ ) {
                        pointToPixel.emplace_back(
                            x, y, i == 0 ? 0 : i == 1 ? IntermediateLayerIndex : IntermediateLayerIndex + 1 );
                      } else {
                        pointToPixel.emplace_back( x, y, i < 2 ? i : IntermediateLayerIndex + 1 );
                      }
                    }
                  }
                }
              }  // fi (params.enhancedOccupancyMapCode_)
            }
          }
        }
      }
    }
  }
  tile.setTotalNumberOfRegularPoints( reconstruct.getPointCount() );
#if 1
  printf("frame %zu, tile %zu: regularPoints %zu\n", frameIndex, tileIndex, reconstruct.getPointCount()  );
#endif
  patchIndex                         = index;
  size_t       totalEOMPointsInFrame = 0;
  PCCPointSet3 eomSavedPoints;
  if ( params.enhancedOccupancyMapCode_ ) {
    const size_t blockSize       = params.occupancyResolution_ * params.occupancyResolution_;
//    size_t       totalPatchCount = patchCount;
    size_t       numEOMPatches   = tile.getEomPatches().size();
    for ( int j = 0; j < numEOMPatches; j++ ) {
      auto&  eomPatch               = tile.getEomPatches( j );
      size_t numPatchesInEOMPatches = eomPatch.memberPatches.size();
      size_t u0Eom                  = useRawPointsSeparateVideo ? 0 : eomPatch.u0_ * params.occupancyResolution_;
      size_t v0Eom                  = useRawPointsSeparateVideo ? 0 : eomPatch.v0_ * params.occupancyResolution_;
      totalEOMPointsInFrame += eomPatch.eomCount_;
      size_t totalPointCount = 0;
      for ( size_t patchIdxInEom = 0; patchIdxInEom < numPatchesInEOMPatches; patchIdxInEom++ ) {
        size_t memberPatchIdx = ( bDecoder && context.getAtlasSequenceParameterSet( 0 ).getPatchPrecedenceOrderFlag() )
        ? ( totalPatchCount - eomPatch.memberPatches[patchIdxInEom] - 1 )
                                    : eomPatch.memberPatches[patchIdxInEom];
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
#if TILETYPE1_RAWAUXVIDEO_BUGFIX2
          if(!params.useAuxSeperateVideo_)
#endif
          occupancyMap[vv * tileWidth + uu] = 1;  // occupied
        }
      }
      TRACE_CODEC( "%d eomPatch :%zu,%zu\t %zu patches, %zu points\n", j, u0Eom, v0Eom, numPatchesInEOMPatches,
                   eomPatch.eomCount_ );
    }
    tile.setTotalNumberOfEOMPoints( totalEOMPointsInFrame );
  }
#if 1
  printf("frame %zu, tile %zu: regularPoints+eomPoints %zu\n", frameIndex, tileIndex, reconstruct.getPointCount()  );
#endif
  TRACE_CODEC( " totalEOMPointsInFrame = %zu  \n", totalEOMPointsInFrame );
  TRACE_CODEC( " point = %zu  \n", reconstruct.getPointCount() );
  if ( params.useAdditionalPointsPatch_ ) {
    if ( useRawPointsSeparateVideo ) {
      PCCColor3B rawPointsColor( uint8_t( 0 ) );
      rawPointsColor[0] = 0;
      rawPointsColor[1] = 255;
      rawPointsColor[2] = 255;
      // Add point GPS from rawPointsPatch without inserting to pointToPixel
      size_t numberOfRawPointsPatches = tile.getNumberOfRawPointsPatches();
      for ( int j = 0; j < numberOfRawPointsPatches; j++ ) {
        auto&  rawPointsPatch = tile.getRawPointsPatch( j );
        size_t numRawPoints   = rawPointsPatch.getNumberOfRawPoints();
        for ( int i = 0; i < numRawPoints; i++ ) {
          PCCVector3D point0;
          point0[0]               = rawPointsPatch.x_[i] + rawPointsPatch.u1_;
          point0[1]               = rawPointsPatch.x_[i + numRawPoints] + rawPointsPatch.v1_;
          point0[2]               = rawPointsPatch.x_[i + 2 * numRawPoints] + rawPointsPatch.d1_;
          const size_t pointIndex = reconstruct.addPoint( point0 );
          reconstruct.setPointPatchIndex( pointIndex, tileIndex, patchIndex );
          reconstruct.setColor( pointIndex, rawPointsColor );
          partition.push_back( uint32_t( patchIndex ) );
        }
        rawPointsPatch.resizeColor( numRawPoints );
      }
    } else {  // else useRawPointsSeparateVideo
      TRACE_CODEC( " Add points from rawPointsPatch \n" );
      // Add points from rawPointsPatch
      size_t numberOfRawPointsPatches = tile.getNumberOfRawPointsPatches();
      for ( int i = 0; i < numberOfRawPointsPatches; i++ ) {
        auto& rawPointsPatch = tile.getRawPointsPatch( i );
        PCCColor3B rawPointsColor( uint8_t( 0 ) );
        rawPointsColor[0]     = 0;
        rawPointsColor[1]     = 255;
        rawPointsColor[2]     = 255;
        size_t numRawPoints   = rawPointsPatch.getNumberOfRawPoints();
        size_t ores           = rawPointsPatch.occupancyResolution_;
        rawPointsPatch.sizeV_ = rawPointsPatch.sizeV0_ * ores;
        rawPointsPatch.sizeU_ = rawPointsPatch.sizeU0_ * ores;

        std::vector<PCCPoint3D> rawPoints;
        rawPoints.resize( numRawPoints );
        size_t       numRawPointsAdded{0};
        const size_t v0 = rawPointsPatch.v0_ * rawPointsPatch.occupancyResolution_;
        const size_t u0 = rawPointsPatch.u0_ * rawPointsPatch.occupancyResolution_;
        for ( size_t v = 0; v < rawPointsPatch.sizeV_; ++v ) {
          for ( size_t u = 0; u < rawPointsPatch.sizeU_; ++u ) {
            const size_t x = ( u0 + u );
            const size_t y = ( v0 + v );
            if ( numRawPointsAdded < numRawPoints ) {
              rawPoints[numRawPointsAdded][0] =
                  double( frame0.getValue( 0, tile.getLeftTopXInFrame() + x, tile.getLeftTopYInFrame() + y ) +
                          rawPointsPatch.u1_ );
            } else if ( numRawPoints <= numRawPointsAdded && numRawPointsAdded < 2 * numRawPoints ) {
              rawPoints[numRawPointsAdded - numRawPoints][1] =
                  double( frame0.getValue( 0, tile.getLeftTopXInFrame() + x, tile.getLeftTopYInFrame() + y ) +
                          rawPointsPatch.v1_ );
            } else if ( 2 * numRawPoints <= numRawPointsAdded && numRawPointsAdded < 3 * numRawPoints ) {
              rawPoints[numRawPointsAdded - 2 * numRawPoints][2] =
                  double( frame0.getValue( 0, tile.getLeftTopXInFrame() + x, tile.getLeftTopYInFrame() + y ) +
                          rawPointsPatch.d1_ );
            }
            numRawPointsAdded++;
          }  // u
        }    // v
        size_t counter{0};
        for ( size_t v = 0; v < rawPointsPatch.sizeV_; ++v ) {
          for ( size_t u = 0; u < rawPointsPatch.sizeU_; ++u ) {
            const size_t x = ( u0 + u );
            const size_t y = ( v0 + v );
            if ( counter < numRawPoints ) {
              const size_t pointIndex = reconstruct.addPoint( rawPoints[counter] );
              reconstruct.setPointPatchIndex( pointIndex, tileIndex, patchIndex );
              reconstruct.setColor( pointIndex, rawPointsColor );
              partition.push_back( uint32_t( patchIndex ) );
              pointToPixel.emplace_back( x, y, 0 );
              counter++;
            }
          }
        }
      }
    }  // fi :useRawPointsSeparateVideo
  }    // fi : useAdditionalPointsPatch
#if 1
    printf("frame %zu, tile %zu: regularPoints+eomPoints+rawPoints %zu\n", frameIndex, tileIndex, reconstruct.getPointCount()  );
#endif
  if ( params.flagGeometrySmoothing_ && !params.pbfEnableFlag_ ) {
    TRACE_CODEC( " identify first boundary layer \n" );
    // identify first boundary layer
    if ( useRawPointsSeparateVideo ) {
      assert( ( reconstruct.getPointCount() - tile.getTotalNumberOfRawPoints() ) == pointToPixel.size() );
    } else {
      assert( ( reconstruct.getPointCount() + tile.getTotalNumberOfRawPoints() ) == pointToPixel.size() );
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
                                int                             cellId ) {
  if ( count[cellId] == 0 ) {
    gpartition[cellId] = patchIdx;

    centerGrid[cellId][0] = 0.;
    centerGrid[cellId][1] = 0.;
    centerGrid[cellId][2] = 0.;
    doSmooth[cellId]      = false;
  } else if ( !doSmooth[cellId] && gpartition[cellId] != patchIdx ) {
    doSmooth[cellId] = true;
  }
  centerGrid[cellId][0] += static_cast<float>( point[0] );
  centerGrid[cellId][1] += static_cast<float>( point[1] );
  centerGrid[cellId][2] += static_cast<float>( point[2] );
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
                              std::vector<int>&               cellIndex ) {
  uint16_t gridSizeHalf           = gridSize / 2;
  bool     otherClusterPointCount = false;
  int      x                      = curPoint.x();
  int      y                      = curPoint.y();
  int      z                      = curPoint.z();
  int      x2                     = x / gridSize;
  int      y2                     = y / gridSize;
  int      z2                     = z / gridSize;
  int      x3                     = x - x2 * gridSize;
  int      y3                     = y - y2 * gridSize;
  int      z3                     = z - z2 * gridSize;
  int      sx                     = x2 + ( ( x3 < gridSizeHalf ) ? -1 : 0 );
  int      sy                     = y2 + ( ( y3 < gridSizeHalf ) ? -1 : 0 );
  int      sz                     = z2 + ( ( z3 < gridSizeHalf ) ? -1 : 0 );
  int      idx[2][2][2];
  for ( int dz = 0; dz < 2; dz++ ) {
    int z4 = sz + dz;
    for ( int dy = 0; dy < 2; dy++ ) {
      int y4 = sy + dy;
      for ( int dx = 0; dx < 2; dx++ ) {
        int x4          = sx + dx;
        int tmp         = x4 + y4 * gridWidth + z4 * gridWidth * gridWidth;
        idx[dz][dy][dx] = tmp;
        if ( doSmooth[cellIndex[tmp]] && ( gridCount[cellIndex[tmp]] != 0U ) ) { otherClusterPointCount = true; }
      }
    }
  }

  if ( !otherClusterPointCount ) { return otherClusterPointCount; }
  PCCVector3D centroid3[2][2][2] = {};
  PCCVector3D curVector( x, y, z );
  int         gridSize2 = gridSize * 2;

  int gridWidth3 = gridWidth * gridWidth * gridWidth;
  int sx2        = sx * gridSize;
  int sy2        = sy * gridSize;
  int sz2        = sz * gridSize;
  int wx         = ( x - sx2 - gridSizeHalf ) * 2 + 1;
  int wy         = ( y - sy2 - gridSizeHalf ) * 2 + 1;
  int wz         = ( z - sz2 - gridSizeHalf ) * 2 + 1;

  centroid3[0][0][0][0] =
      gridCount[cellIndex[idx[0][0][0]]] > 0 ? static_cast<double>( center[cellIndex[idx[0][0][0]]][0] ) : curVector[0];
  centroid3[0][0][0][1] =
      gridCount[cellIndex[idx[0][0][0]]] > 0 ? static_cast<double>( center[cellIndex[idx[0][0][0]]][1] ) : curVector[1];
  centroid3[0][0][0][2] =
      gridCount[cellIndex[idx[0][0][0]]] > 0 ? static_cast<double>( center[cellIndex[idx[0][0][0]]][2] ) : curVector[2];

  centroid3[0][0][1] = curVector;
  if ( idx[0][0][1] < gridWidth3 ) {
    centroid3[0][0][1][0] = gridCount[cellIndex[idx[0][0][1]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[0][0][1]]][0] )
                                : curVector[0];
    centroid3[0][0][1][1] = gridCount[cellIndex[idx[0][0][1]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[0][0][1]]][1] )
                                : curVector[1];
    centroid3[0][0][1][2] = gridCount[cellIndex[idx[0][0][1]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[0][0][1]]][2] )
                                : curVector[2];
  }

  centroid3[0][1][0] = curVector;
  if ( idx[0][1][0] < gridWidth3 ) {
    centroid3[0][1][0][0] = gridCount[cellIndex[idx[0][1][0]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[0][1][0]]][0] )
                                : curVector[0];
    centroid3[0][1][0][1] = gridCount[cellIndex[idx[0][1][0]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[0][1][0]]][1] )
                                : curVector[1];
    centroid3[0][1][0][2] = gridCount[cellIndex[idx[0][1][0]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[0][1][0]]][2] )
                                : curVector[2];
  }

  centroid3[0][1][1] = curVector;
  if ( idx[0][1][1] < gridWidth3 ) {
    centroid3[0][1][1][0] = gridCount[cellIndex[idx[0][1][1]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[0][1][1]]][0] )
                                : curVector[0];
    centroid3[0][1][1][1] = gridCount[cellIndex[idx[0][1][1]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[0][1][1]]][1] )
                                : curVector[1];
    centroid3[0][1][1][2] = gridCount[cellIndex[idx[0][1][1]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[0][1][1]]][2] )
                                : curVector[2];
  }

  centroid3[1][0][0] = curVector;
  if ( idx[1][0][0] < gridWidth3 ) {
    centroid3[1][0][0][0] = gridCount[cellIndex[idx[1][0][0]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[1][0][0]]][0] )
                                : curVector[0];
    centroid3[1][0][0][1] = gridCount[cellIndex[idx[1][0][0]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[1][0][0]]][1] )
                                : curVector[1];
    centroid3[1][0][0][2] = gridCount[cellIndex[idx[1][0][0]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[1][0][0]]][2] )
                                : curVector[2];
  }

  centroid3[1][0][1] = curVector;
  if ( idx[1][0][1] < gridWidth3 ) {
    centroid3[1][0][1][0] = gridCount[cellIndex[idx[1][0][1]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[1][0][1]]][0] )
                                : curVector[0];
    centroid3[1][0][1][1] = gridCount[cellIndex[idx[1][0][1]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[1][0][1]]][1] )
                                : curVector[1];
    centroid3[1][0][1][2] = gridCount[cellIndex[idx[1][0][1]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[1][0][1]]][2] )
                                : curVector[2];
  }

  centroid3[1][1][0] = curVector;
  if ( idx[1][1][0] < gridWidth3 ) {
    centroid3[1][1][0][0] = gridCount[cellIndex[idx[1][1][0]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[1][1][0]]][0] )
                                : curVector[0];
    centroid3[1][1][0][1] = gridCount[cellIndex[idx[1][1][0]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[1][1][0]]][1] )
                                : curVector[1];
    centroid3[1][1][0][2] = gridCount[cellIndex[idx[1][1][0]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[1][1][0]]][2] )
                                : curVector[2];
  }

  centroid3[1][1][1] = curVector;
  if ( idx[1][1][1] < gridWidth3 ) {
    centroid3[1][1][1][0] = gridCount[cellIndex[idx[1][1][1]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[1][1][1]]][0] )
                                : curVector[0];
    centroid3[1][1][1][1] = gridCount[cellIndex[idx[1][1][1]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[1][1][1]]][1] )
                                : curVector[1];
    centroid3[1][1][1][2] = gridCount[cellIndex[idx[1][1][1]]] > 0
                                ? static_cast<double>( center[cellIndex[idx[1][1][1]]][2] )
                                : curVector[2];
  }

  int gridSizeWx     = ( gridSize2 - wx );
  int gridSizeWy     = ( gridSize2 - wy );
  int gridSizeWz     = ( gridSize2 - wz );
  centroid3[0][0][0] = gridSizeWx * gridSizeWy * gridSizeWz * centroid3[0][0][0];
  centroid3[0][0][1] = (wx)*gridSizeWy * gridSizeWz * centroid3[0][0][1];
  centroid3[0][1][0] = gridSizeWx * (wy)*gridSizeWz * centroid3[0][1][0];
  centroid3[0][1][1] = ( wx ) * (wy)*gridSizeWz * centroid3[0][1][1];
  centroid3[1][0][0] = gridSizeWx * gridSizeWy * (wz)*centroid3[1][0][0];
  centroid3[1][0][1] = (wx)*gridSizeWy * (wz)*centroid3[1][0][1];
  centroid3[1][1][0] = gridSizeWx * ( wy ) * (wz)*centroid3[1][1][0];
  centroid3[1][1][1] = ( wx ) * ( wy ) * (wz)*centroid3[1][1][1];

  PCCVector3D centroid4;
  centroid4 = centroid3[0][0][0] + centroid3[0][0][1] + centroid3[0][1][0] + centroid3[0][1][1] + centroid3[1][0][0] +
              centroid3[1][0][1] + centroid3[1][1][0] + centroid3[1][1][1];

  count = gridSizeWx * gridSizeWy * gridSizeWz * gridCount[cellIndex[idx[0][0][0]]];
  count += (wx)*gridSizeWy * gridSizeWz * gridCount[cellIndex[idx[0][0][1]]];
  count += gridSizeWx * (wy)*gridSizeWz * gridCount[cellIndex[idx[0][1][0]]];
  count += ( wx ) * (wy)*gridSizeWz * gridCount[cellIndex[idx[0][1][1]]];
  count += gridSizeWx * gridSizeWy * (wz)*gridCount[cellIndex[idx[1][0][0]]];
  count += (wx)*gridSizeWy * (wz)*gridCount[cellIndex[idx[1][0][1]]];
  count += gridSizeWx * ( wy ) * (wz)*gridCount[cellIndex[idx[1][1][0]]];
  count += ( wx ) * ( wy ) * (wz)*gridCount[cellIndex[idx[1][1][1]]];
  centroid4 /= gridSize2 * gridSize2 * gridSize2;
  count /= gridSize2 * gridSize2 * gridSize2;
  centroid = centroid4 * count;
  //  centroid = (centroid4 * count) / (gridSize2 * gridSize2 * gridSize2 *
  // gridSize2 * gridSize2 * gridSize2);

  return otherClusterPointCount;
}

bool PCCCodec::gridFilteringTransfer( const std::vector<uint32_t>& partition,
                                      PCCPointSet3&                pointCloud,
                                      PCCPoint3D&                  curPoint,
                                      PCCVector3D&                 centroid,
                                      int&                         count,
                                      std::vector<int>&            gridCount,
                                      std::vector<PCCVector3D>&    center,
                                      std::vector<bool>&           doSmooth,
                                      int                          gridSize,
                                      int                          gridWidth,
                                      std::vector<PCCVector3D>&    colorGrid,
                                      PCCVector3D&                 color ) {
  bool otherClusterPointCount = false;
  int  x                      = curPoint.x();
  int  y                      = curPoint.y();
  int  z                      = curPoint.z();
  int  x2                     = x / gridSize;
  int  y2                     = y / gridSize;
  int  z2                     = z / gridSize;
  int  sx                     = x2 + ( ( ( x - x2 * gridSize ) < gridSize / 2 ) ? -1 : 0 );
  int  sy                     = y2 + ( ( ( y - y2 * gridSize ) < gridSize / 2 ) ? -1 : 0 );
  int  sz                     = z2 + ( ( ( z - z2 * gridSize ) < gridSize / 2 ) ? -1 : 0 );
  int  idx[2][2][2];
  for ( int dz = 0; dz < 2; dz++ ) {
    int z4 = sz + dz;
    for ( int dy = 0; dy < 2; dy++ ) {
      int y4 = sy + dy;
      for ( int dx = 0; dx < 2; dx++ ) {
        int x4          = sx + dx;
        int tmp         = x4 + y4 * gridWidth + z4 * gridWidth * gridWidth;
        idx[dz][dy][dx] = tmp;
        if ( doSmooth[tmp] && ( gridCount[tmp] != 0 ) ) { otherClusterPointCount = true; }
      }
    }
  }
  if ( !otherClusterPointCount ) { return otherClusterPointCount; }
  PCCVector3D centroid3[2][2][2] = {};
  PCCVector3D curVector( x, y, z );
  int         gridSize2 = gridSize * 2;
  int         sx2       = sx * gridSize;
  int         sy2       = sy * gridSize;
  int         sz2       = sz * gridSize;
  int         wx        = ( x - sx2 - gridSize / 2 ) * 2 + 1;
  int         wy        = ( y - sy2 - gridSize / 2 ) * 2 + 1;
  int         wz        = ( z - sz2 - gridSize / 2 ) * 2 + 1;

  centroid3[0][0][0] = gridCount[idx[0][0][0]] > 0 ? center[idx[0][0][0]] : curVector;
  centroid3[0][0][1] = gridCount[idx[0][0][1]] > 0 ? center[idx[0][0][1]] : curVector;
  centroid3[0][1][0] = gridCount[idx[0][1][0]] > 0 ? center[idx[0][1][0]] : curVector;
  centroid3[0][1][1] = gridCount[idx[0][1][1]] > 0 ? center[idx[0][1][1]] : curVector;
  centroid3[1][0][0] = gridCount[idx[1][0][0]] > 0 ? center[idx[1][0][0]] : curVector;
  centroid3[1][0][1] = gridCount[idx[1][0][1]] > 0 ? center[idx[1][0][1]] : curVector;
  centroid3[1][1][0] = gridCount[idx[1][1][0]] > 0 ? center[idx[1][1][0]] : curVector;
  centroid3[1][1][1] = gridCount[idx[1][1][1]] > 0 ? center[idx[1][1][1]] : curVector;

  int gridSizeWx = ( gridSize2 - wx );
  int gridSizeWy = ( gridSize2 - wy );
  int gridSizeWz = ( gridSize2 - wz );

  centroid3[0][0][0] = gridSizeWx * gridSizeWy * gridSizeWz * centroid3[0][0][0];
  centroid3[0][0][1] = (wx)*gridSizeWy * gridSizeWz * centroid3[0][0][1];
  centroid3[0][1][0] = gridSizeWx * (wy)*gridSizeWz * centroid3[0][1][0];
  centroid3[0][1][1] = ( wx ) * (wy)*gridSizeWz * centroid3[0][1][1];
  centroid3[1][0][0] = gridSizeWx * gridSizeWy * (wz)*centroid3[1][0][0];
  centroid3[1][0][1] = (wx)*gridSizeWy * (wz)*centroid3[1][0][1];
  centroid3[1][1][0] = gridSizeWx * ( wy ) * (wz)*centroid3[1][1][0];
  centroid3[1][1][1] = ( wx ) * ( wy ) * (wz)*centroid3[1][1][1];

  PCCVector3D centroid4;
  centroid4 = centroid3[0][0][0] + centroid3[0][0][1] + centroid3[0][1][0] + centroid3[0][1][1] + centroid3[1][0][0] +
              centroid3[1][0][1] + centroid3[1][1][0] + centroid3[1][1][1];

  count = gridSizeWx * gridSizeWy * gridSizeWz * gridCount[idx[0][0][0]];
  count += (wx)*gridSizeWy * gridSizeWz * gridCount[idx[0][0][1]];
  count += gridSizeWx * (wy)*gridSizeWz * gridCount[idx[0][1][0]];
  count += ( wx ) * (wy)*gridSizeWz * gridCount[idx[0][1][1]];
  count += gridSizeWx * gridSizeWy * (wz)*gridCount[idx[1][0][0]];
  count += (wx)*gridSizeWy * (wz)*gridCount[idx[1][0][1]];
  count += gridSizeWx * ( wy ) * (wz)*gridCount[idx[1][1][0]];
  count += ( wx ) * ( wy ) * (wz)*gridCount[idx[1][1][1]];
  count /= gridSize2 * gridSize2 * gridSize2;
  centroid4 /= gridSize2 * gridSize2 * gridSize2;
  centroid = centroid4 * count;
  // centroid = (centroid4 * count) / (gridSize2 * gridSize2 * gridSize2 *
  // gridSize2 * gridSize2 * gridSize2);

  return otherClusterPointCount;
}

void PCCCodec::smoothPointCloudGrid( PCCPointSet3&                       reconstruct,
                                     const std::vector<uint32_t>&        partition,
                                     const GeneratePointCloudParameters& params,
                                     uint16_t                            gridWidth,
                                     std::vector<int>&                   cellIndex ) {
  TRACE_CODEC( " smoothPointCloudGrid start \n" );
  const size_t pointCount = reconstruct.getPointCount();
  const int    gridSize   = static_cast<int>( params.gridSize_ );
  const int    disth      = ( std::max )( gridSize / 2, 1 );
  const int    th         = gridSize * gridWidth;
  for ( int c = 0; c < pointCount; c++ ) {
    PCCPoint3D curPoint = reconstruct[c];
    int        x        = static_cast<int>( curPoint.x() );
    int        y        = static_cast<int>( curPoint.y() );
    int        z        = static_cast<int>( curPoint.z() );
    if ( x < disth || y < disth || z < disth || th <= x + disth || th <= y + disth || th <= z + disth ) { continue; }
    PCCVector3D centroid( 0.0 );
    PCCVector3D curVector( x, y, z );
    int         count                  = 0;
    bool        otherClusterPointCount = false;
    PCCVector3D color( 0, 0, 0 );
    if ( reconstruct.getBoundaryPointType( c ) == 1 ) {
      otherClusterPointCount =
          gridFiltering( partition, reconstruct, curPoint, centroid, count, geoSmoothingCount_, geoSmoothingCenter_,
                         geoSmoothingDoSmooth_, gridSize, gridWidth, cellIndex );
    }
    if ( otherClusterPointCount ) {
      // double dist2 = ( ( curVector * count - centroid ).getNorm2() +
      // (double)count / 2.0 ) / (double)count;
      double dist2 = ( ( curVector * count - centroid ).getNorm2() ) / static_cast<double>( count ) + 0.5;
      if ( dist2 >= ( std::max )( static_cast<int>( params.thresholdSmoothing_ ), count ) * 2 ) {
        centroid = centroid / static_cast<double>( count ) + 0.5;
        for ( size_t k = 0; k < 3; ++k ) {
          centroid[k]       = double( int64_t( centroid[k] ) );
          reconstruct[c][k] = centroid[k];
        }

        if ( PCC_SAVE_POINT_TYPE == 1 ) { reconstruct.setType( c, POINT_SMOOTH ); }
        reconstruct.setBoundaryPointType( c, static_cast<uint16_t>( 3 ) );
      }
    }
  }
  TRACE_CODEC( " smoothPointCloudGrid done \n" );
}

void PCCCodec::smoothPointCloud( PCCPointSet3&                      reconstruct,
                                 const std::vector<uint32_t>&       partition,
                                 const GeneratePointCloudParameters params ) {
  TRACE_CODEC( " smoothPointCloud start \n" );
  const size_t pointCount = reconstruct.getPointCount();
  PCCKdTree    kdtree( reconstruct );
  PCCPointSet3 temp;
  temp.resize( pointCount );
  tbb::task_arena limited( static_cast<int>( params.nbThread_ ) );
  limited.execute( [&] {
    tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) {
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
          temp[i][0] = centroid[0];
          temp[i][1] = centroid[1];
          temp[i][2] = centroid[2];
          reconstruct.setColor( i, PCCColor3B( 255, 0, 0 ) );
          if ( PCC_SAVE_POINT_TYPE == 1 ) { reconstruct.setType( i, POINT_SMOOTH ); }
        } else {
          temp[i] = reconstruct[i];
        }
      } else {
        temp[i] = reconstruct[i];
      }
    } );
  } );
  limited.execute(
      [&] { tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) { reconstruct[i] = temp[i]; } ); } );
  TRACE_CODEC( " smoothPointCloud done \n" );
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
                                     int                                     cellId ) {
  if ( colorGridCount[cellId] == 0 ) {
    colorPartition[cellId] = tilePatchIdx;
    colorCenter[cellId][0] = 0.;
    colorCenter[cellId][1] = 0.;
    colorCenter[cellId][2] = 0.;
    colorDoSmooth[cellId]  = false;
  } else if ( !colorDoSmooth[cellId] && colorPartition[cellId] != tilePatchIdx ) {
    colorDoSmooth[cellId] = true;
  }
  colorCenter[cellId][0] += static_cast<float>( color[0] );
  colorCenter[cellId][1] += static_cast<float>( color[1] );
  colorCenter[cellId][2] += static_cast<float>( color[2] );
  colorGridCount[cellId]++;
  colorLum[cellId].push_back( uint16_t( color[0] ) );
}

bool PCCCodec::gridFilteringColor( PCCPoint3D&                         curPos,
                                   PCCVector3D&                        colorCentroid,
                                   int&                                colorCount,
                                   std::vector<uint16_t>&              colorGridCount,
                                   std::vector<PCCVector3<float>>&     colorCenter,
                                   std::vector<bool>&                  colorDoSmooth,
                                   uint8_t                             gridSize,
                                   PCCVector3D&                        curPosColor,
                                   const GeneratePointCloudParameters& params,
                                   std::vector<int>&                   cellIndex ) {
  const int w                      = pow( 2, params.geometryBitDepth3D_ ) / gridSize;
  bool      otherClusterPointCount = false;
  int       x                      = curPos.x();
  int       y                      = curPos.y();
  int       z                      = curPos.z();
  int       x2                     = x / gridSize;
  int       y2                     = y / gridSize;
  int       z2                     = z / gridSize;
  int       x3                     = x % gridSize;
  int       y3                     = y % gridSize;
  int       z3                     = z % gridSize;
  int       sx                     = x2 + ( ( x3 < gridSize / 2 ) ? -1 : 0 );
  int       sy                     = y2 + ( ( y3 < gridSize / 2 ) ? -1 : 0 );
  int       sz                     = z2 + ( ( z3 < gridSize / 2 ) ? -1 : 0 );
  int       sx2                    = sx * gridSize;
  int       sy2                    = sy * gridSize;
  int       sz2                    = sz * gridSize;
  int       wx                     = ( x - sx2 - gridSize / 2 ) * 2 + 1;
  int       wy                     = ( y - sy2 - gridSize / 2 ) * 2 + 1;
  int       wz                     = ( z - sz2 - gridSize / 2 ) * 2 + 1;
  int       idx[2][2][2];
  for ( int dz = 0; dz < 2; dz++ ) {
    for ( int dy = 0; dy < 2; dy++ ) {
      for ( int dx = 0; dx < 2; dx++ ) {
        int x3          = sx + dx;
        int y3          = sy + dy;
        int z3          = sz + dz;
        int tmp         = x3 + y3 * w + z3 * w * w;
        idx[dz][dy][dx] = tmp;
        if ( colorDoSmooth[cellIndex[tmp]] && ( colorGridCount[cellIndex[tmp]] != 0U ) ) {
          otherClusterPointCount = true;
        }
      }
    }
  }
  if ( !otherClusterPointCount ) { return otherClusterPointCount; }
  int         cnt0;
  PCCVector3D colorCentroid3[2][2][2] = {};
  int         gridSize2               = gridSize * 2;
  double      mmThresh                = params.thresholdColorVariation_ * 256.0;
  double      yThresh                 = params.thresholdColorDifference_ * 256.0;
  if ( colorGridCount[cellIndex[idx[0][0][0]]] > 0 ) {
    colorCentroid3[0][0][0][0] =
        double( colorCenter[cellIndex[idx[0][0][0]]][0] ) / double( colorGridCount[cellIndex[idx[0][0][0]]] );
    colorCentroid3[0][0][0][1] =
        double( colorCenter[cellIndex[idx[0][0][0]]][1] ) / double( colorGridCount[cellIndex[idx[0][0][0]]] );
    colorCentroid3[0][0][0][2] =
        double( colorCenter[cellIndex[idx[0][0][0]]][2] ) / double( colorGridCount[cellIndex[idx[0][0][0]]] );
    cnt0 = colorGridCount[cellIndex[idx[0][0][0]]];
    if ( colorGridCount[cellIndex[idx[0][0][0]]] > 1 ) {
      double meanY =
          mean( colorSmoothingLum_[cellIndex[idx[0][0][0]]], int( colorGridCount[cellIndex[idx[0][0][0]]] ) );
      double medianY =
          median( colorSmoothingLum_[cellIndex[idx[0][0][0]]], int( colorGridCount[cellIndex[idx[0][0][0]]] ) );
      if ( abs( meanY - medianY ) > mmThresh ) {
        colorCentroid = curPosColor;
        colorCount    = 1;
        return otherClusterPointCount;
      }
    }
  } else {
    colorCentroid3[0][0][0] = curPosColor;
    cnt0                    = 1;
  }

  double Y0 = colorCentroid3[0][0][0][0];

  if ( colorGridCount[cellIndex[idx[0][0][1]]] > 0 ) {
    colorCentroid3[0][0][1][0] =
        double( colorCenter[cellIndex[idx[0][0][1]]][0] ) / double( colorGridCount[cellIndex[idx[0][0][1]]] );
    colorCentroid3[0][0][1][1] =
        double( colorCenter[cellIndex[idx[0][0][1]]][1] ) / double( colorGridCount[cellIndex[idx[0][0][1]]] );
    colorCentroid3[0][0][1][2] =
        double( colorCenter[cellIndex[idx[0][0][1]]][2] ) / double( colorGridCount[cellIndex[idx[0][0][1]]] );
    double Y1 = colorCentroid3[0][0][1][0];
    if ( abs( Y0 - Y1 ) > yThresh ) { colorCentroid3[0][0][1] = curPosColor; }
    if ( colorGridCount[cellIndex[idx[0][0][1]]] > 1 ) {
      double meanY =
          mean( colorSmoothingLum_[cellIndex[idx[0][0][1]]], int( colorGridCount[cellIndex[idx[0][0][1]]] ) );
      double medianY =
          median( colorSmoothingLum_[cellIndex[idx[0][0][1]]], int( colorGridCount[cellIndex[idx[0][0][1]]] ) );
      if ( abs( meanY - medianY ) > mmThresh ) { colorCentroid3[0][0][1] = curPosColor; }
    }
  } else {
    colorCentroid3[0][0][1] = curPosColor;
  }

  if ( colorGridCount[cellIndex[idx[0][1][0]]] > 0 ) {
    colorCentroid3[0][1][0][0] =
        double( colorCenter[cellIndex[idx[0][1][0]]][0] ) / double( colorGridCount[cellIndex[idx[0][1][0]]] );
    colorCentroid3[0][1][0][1] =
        double( colorCenter[cellIndex[idx[0][1][0]]][1] ) / double( colorGridCount[cellIndex[idx[0][1][0]]] );
    colorCentroid3[0][1][0][2] =
        double( colorCenter[cellIndex[idx[0][1][0]]][2] ) / double( colorGridCount[cellIndex[idx[0][1][0]]] );
    double Y2 = colorCentroid3[0][1][0][0];

    if ( abs( Y0 - Y2 ) > yThresh ) { colorCentroid3[0][1][0] = curPosColor; }
    if ( colorGridCount[cellIndex[idx[0][1][0]]] > 1 ) {
      double meanY =
          mean( colorSmoothingLum_[cellIndex[idx[0][1][0]]], int( colorGridCount[cellIndex[idx[0][1][0]]] ) );
      double medianY =
          median( colorSmoothingLum_[cellIndex[idx[0][1][0]]], int( colorGridCount[cellIndex[idx[0][1][0]]] ) );
      if ( abs( meanY - medianY ) > mmThresh ) { colorCentroid3[0][1][0] = curPosColor; }
    }
  } else {
    colorCentroid3[0][1][0] = curPosColor;
  }

  if ( colorGridCount[cellIndex[idx[0][1][1]]] > 0 ) {
    colorCentroid3[0][1][1][0] =
        double( colorCenter[cellIndex[idx[0][1][1]]][0] ) / double( colorGridCount[cellIndex[idx[0][1][1]]] );
    colorCentroid3[0][1][1][1] =
        double( colorCenter[cellIndex[idx[0][1][1]]][1] ) / double( colorGridCount[cellIndex[idx[0][1][1]]] );
    colorCentroid3[0][1][1][2] =
        double( colorCenter[cellIndex[idx[0][1][1]]][2] ) / double( colorGridCount[cellIndex[idx[0][1][1]]] );

    double Y3 = colorCentroid3[0][1][1][0];

    if ( abs( Y0 - Y3 ) > yThresh ) { colorCentroid3[0][1][1] = curPosColor; }
    if ( colorGridCount[cellIndex[idx[0][1][1]]] > 1 ) {
      double meanY =
          mean( colorSmoothingLum_[cellIndex[idx[0][1][1]]], int( colorGridCount[cellIndex[idx[0][1][1]]] ) );
      double medianY =
          median( colorSmoothingLum_[cellIndex[idx[0][1][1]]], int( colorGridCount[cellIndex[idx[0][1][1]]] ) );
      if ( abs( meanY - medianY ) > mmThresh ) { colorCentroid3[0][1][1] = curPosColor; }
    }
  } else {
    colorCentroid3[0][1][1] = curPosColor;
  }

  if ( colorGridCount[cellIndex[idx[1][0][0]]] > 0 ) {
    colorCentroid3[1][0][0][0] =
        double( colorCenter[cellIndex[idx[1][0][0]]][0] ) / double( colorGridCount[cellIndex[idx[1][0][0]]] );
    colorCentroid3[1][0][0][1] =
        double( colorCenter[cellIndex[idx[1][0][0]]][1] ) / double( colorGridCount[cellIndex[idx[1][0][0]]] );
    colorCentroid3[1][0][0][2] =
        double( colorCenter[cellIndex[idx[1][0][0]]][2] ) / double( colorGridCount[cellIndex[idx[1][0][0]]] );
    double Y4 = colorCentroid3[1][0][0][0];

    if ( abs( Y0 - Y4 ) > yThresh ) { colorCentroid3[1][0][0] = curPosColor; }
    if ( colorGridCount[cellIndex[idx[1][0][0]]] > 1 ) {
      double meanY =
          mean( colorSmoothingLum_[cellIndex[idx[1][0][0]]], int( colorGridCount[cellIndex[idx[1][0][0]]] ) );
      double medianY =
          median( colorSmoothingLum_[cellIndex[idx[1][0][0]]], int( colorGridCount[cellIndex[idx[1][0][0]]] ) );
      if ( abs( meanY - medianY ) > mmThresh ) { colorCentroid3[1][0][0] = curPosColor; }
    }
  } else {
    colorCentroid3[1][0][0] = curPosColor;
  }

  if ( colorGridCount[cellIndex[idx[1][0][1]]] > 0 ) {
    colorCentroid3[1][0][1][0] =
        double( colorCenter[cellIndex[idx[1][0][1]]][0] ) / double( colorGridCount[cellIndex[idx[1][0][1]]] );
    colorCentroid3[1][0][1][1] =
        double( colorCenter[cellIndex[idx[1][0][1]]][1] ) / double( colorGridCount[cellIndex[idx[1][0][1]]] );
    colorCentroid3[1][0][1][2] =
        double( colorCenter[cellIndex[idx[1][0][1]]][2] ) / double( colorGridCount[cellIndex[idx[1][0][1]]] );
    double Y5 = colorCentroid3[1][0][1][0];

    if ( abs( Y0 - Y5 ) > yThresh ) { colorCentroid3[1][0][1] = curPosColor; }
    if ( colorGridCount[cellIndex[idx[1][0][1]]] > 1 ) {
      double meanY =
          mean( colorSmoothingLum_[cellIndex[idx[1][0][1]]], int( colorGridCount[cellIndex[idx[1][0][1]]] ) );
      double medianY =
          median( colorSmoothingLum_[cellIndex[idx[1][0][1]]], int( colorGridCount[cellIndex[idx[1][0][1]]] ) );
      if ( abs( meanY - medianY ) > mmThresh ) { colorCentroid3[1][0][1] = curPosColor; }
    }
  } else {
    colorCentroid3[1][0][1] = curPosColor;
  }

  if ( colorGridCount[cellIndex[idx[1][1][0]]] > 0 ) {
    colorCentroid3[1][1][0][0] =
        double( colorCenter[cellIndex[idx[1][1][0]]][0] ) / double( colorGridCount[cellIndex[idx[1][1][0]]] );
    colorCentroid3[1][1][0][1] =
        double( colorCenter[cellIndex[idx[1][1][0]]][1] ) / double( colorGridCount[cellIndex[idx[1][1][0]]] );
    colorCentroid3[1][1][0][2] =
        double( colorCenter[cellIndex[idx[1][1][0]]][2] ) / double( colorGridCount[cellIndex[idx[1][1][0]]] );
    double Y6 = colorCentroid3[1][1][0][0];

    if ( abs( Y0 - Y6 ) > yThresh ) { colorCentroid3[1][1][0] = curPosColor; }
    if ( colorGridCount[cellIndex[idx[1][1][0]]] > 1 ) {
      double meanY =
          mean( colorSmoothingLum_[cellIndex[idx[1][1][0]]], int( colorGridCount[cellIndex[idx[1][1][0]]] ) );
      double medianY =
          median( colorSmoothingLum_[cellIndex[idx[1][1][0]]], int( colorGridCount[cellIndex[idx[1][1][0]]] ) );
      if ( abs( meanY - medianY ) > mmThresh ) { colorCentroid3[1][1][0] = curPosColor; }
    }
  } else {
    colorCentroid3[1][1][0] = curPosColor;
  }

  if ( colorGridCount[cellIndex[idx[1][1][1]]] > 0 ) {
    colorCentroid3[1][1][1][0] =
        double( colorCenter[cellIndex[idx[1][1][1]]][0] ) / double( colorGridCount[cellIndex[idx[1][1][1]]] );
    colorCentroid3[1][1][1][1] =
        double( colorCenter[cellIndex[idx[1][1][1]]][1] ) / double( colorGridCount[cellIndex[idx[1][1][1]]] );
    colorCentroid3[1][1][1][2] =
        double( colorCenter[cellIndex[idx[1][1][1]]][2] ) / double( colorGridCount[cellIndex[idx[1][1][1]]] );
    double Y7 = colorCentroid3[1][1][1][0];

    if ( abs( Y0 - Y7 ) > yThresh ) { colorCentroid3[1][1][1] = curPosColor; }
    if ( colorGridCount[cellIndex[idx[1][1][1]]] > 1 ) {
      double meanY =
          mean( colorSmoothingLum_[cellIndex[idx[1][1][1]]], int( colorGridCount[cellIndex[idx[1][1][1]]] ) );
      double medianY =
          median( colorSmoothingLum_[cellIndex[idx[1][1][1]]], int( colorGridCount[cellIndex[idx[1][1][1]]] ) );
      if ( abs( meanY - medianY ) > mmThresh ) { colorCentroid3[1][1][1] = curPosColor; }
    }
  } else {
    colorCentroid3[1][1][1] = curPosColor;
  }
#if 1
  int gridSizeWx = ( gridSize2 - wx );
  int gridSizeWy = ( gridSize2 - wy );
  int gridSizeWz = ( gridSize2 - wz );

  colorCentroid3[0][0][0] = gridSizeWx * gridSizeWy * gridSizeWz * colorCentroid3[0][0][0];
  colorCentroid3[0][0][1] = (wx)*gridSizeWy * gridSizeWz * colorCentroid3[0][0][1];
  colorCentroid3[0][1][0] = gridSizeWx * (wy)*gridSizeWz * colorCentroid3[0][1][0];
  colorCentroid3[0][1][1] = ( wx ) * (wy)*gridSizeWz * colorCentroid3[0][1][1];
  colorCentroid3[1][0][0] = gridSizeWx * gridSizeWy * (wz)*colorCentroid3[1][0][0];
  colorCentroid3[1][0][1] = (wx)*gridSizeWy * (wz)*colorCentroid3[1][0][1];
  colorCentroid3[1][1][0] = gridSizeWx * ( wy ) * (wz)*colorCentroid3[1][1][0];
#else
  colorCentroid3[0][0][0] = ( gridSize2 - wx ) * ( gridSize2 - wy ) * ( gridSize2 - wz ) * colorCentroid3[0][0][0];
  colorCentroid3[0][0][1] = ( wx ) * ( gridSize2 - wy ) * ( gridSize2 - wz ) * colorCentroid3[0][0][1];
  colorCentroid3[0][1][0] = ( gridSize2 - wx ) * ( wy ) * ( gridSize2 - wz ) * colorCentroid3[0][1][0];
  colorCentroid3[0][1][1] = ( wx ) * ( wy ) * ( gridSize2 - wz ) * colorCentroid3[0][1][1];
  colorCentroid3[1][0][0] = ( gridSize2 - wx ) * ( gridSize2 - wy ) * (wz)*colorCentroid3[1][0][0];
  colorCentroid3[1][0][1] = ( wx ) * ( gridSize2 - wy ) * (wz)*colorCentroid3[1][0][1];
  colorCentroid3[1][1][0] = ( gridSize2 - wx ) * ( wy ) * (wz)*colorCentroid3[1][1][0];
  colorCentroid3[1][1][1] = ( wx ) * ( wy ) * (wz)*colorCentroid3[1][1][1];
#endif
  PCCVector3D colorCentroid4;
  colorCentroid4 = colorCentroid3[0][0][0] + colorCentroid3[0][0][1] + colorCentroid3[0][1][0] +
                   colorCentroid3[0][1][1] + colorCentroid3[1][0][0] + colorCentroid3[1][0][1] +
                   colorCentroid3[1][1][0] + colorCentroid3[1][1][1];
  colorCentroid4 /= gridSize2 * gridSize2 * gridSize2;
  colorCentroid = colorCentroid4;
  colorCount    = 1;
  return otherClusterPointCount;
}

void PCCCodec::smoothPointCloudColorLC( PCCPointSet3&                       reconstruct,
                                        const GeneratePointCloudParameters& params,
                                        std::vector<int>&                   cellIndex ) {
  const size_t pointCount = reconstruct.getPointCount();
  const int    gridSize   = params.occupancyPrecision_;
  const int    disth      = ( std::max )( gridSize / 2, 1 );
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
    PCCVector3D   colorCentroid( 0.0 );
    int           colorCount             = 0;
    bool          otherClusterPointCount = false;
    PCCColor16bit color16bit             = reconstruct.getColor16bit( i );
    PCCVector3D   curPosColor( 0.0 );
    curPosColor[0] = double( color16bit[0] );
    curPosColor[1] = double( color16bit[1] );
    curPosColor[2] = double( color16bit[2] );
    if ( reconstruct.getBoundaryPointType( i ) == 1 ) {
      otherClusterPointCount =
          gridFilteringColor( curPos, colorCentroid, colorCount, colorSmoothingCount_, colorSmoothingCenter_,
                              colorSmoothingDoSmooth_, gridSize, curPosColor, params, cellIndex );
    }
    if ( otherClusterPointCount ) {
      colorCentroid = ( colorCentroid + static_cast<double>( colorCount ) / 2.0 ) / static_cast<double>( colorCount );
      for ( size_t k = 0; k < 3; ++k ) { colorCentroid[k] = double( int64_t( colorCentroid[k] ) ); }
      double distToCentroid2 = 0;

      double Ycent = colorCentroid[0];
      double Ycur  = curPosColor[0];

      distToCentroid2 = abs( Ycent - Ycur ) * 10. / 256.;

      if ( distToCentroid2 >= params.thresholdColorSmoothing_ ) {
        PCCColor16bit color16bit;
        color16bit[0] = uint16_t( colorCentroid[0] );
        color16bit[1] = uint16_t( colorCentroid[1] );
        color16bit[2] = uint16_t( colorCentroid[2] );

        reconstruct.setColor16bit( i, color16bit );
      }
    }
  }
}

//void PCCCodec::createSpecificLayerReconstruct( const PCCPointSet3&                 reconstruct,
//                                               const std::vector<uint32_t>&        partition,
//                                               PCCFrameContext&                    frame,
//                                               const GeneratePointCloudParameters& params,
//                                               const size_t                        frameCount,
//                                               PCCPointSet3&                       subReconstruct,
//                                               std::vector<uint32_t>&              subPartition,
//                                               std::vector<size_t>&                subReconstructIndex ) {
//  subReconstruct.clear();
//  subPartition.clear();
//  subReconstructIndex.clear();
//  auto&        pointToPixel = frame.getPointToPixel();
//  const size_t pointCount   = reconstruct.getPointCount();
//  if ( ( pointCount == 0U ) || !reconstruct.hasColors() ) { return; }
//  for ( size_t i = 0; i < pointCount; ++i ) {
//    const PCCVector3<size_t> location = pointToPixel[i];
//    const size_t             f        = location[2];
//    if ( f == frameCount ) {
//      subReconstruct.addPoint( reconstruct[i] );
//      subPartition.push_back( partition[i] );
//      subReconstructIndex.push_back( i );
//    }
//  }
//  subReconstruct.addColors();
//}

//void PCCCodec::createSubReconstruct( const PCCPointSet3&                 reconstruct,
//                                     const std::vector<uint32_t>&        partition,
//                                     PCCFrameContext&                    frame,
//                                     const GeneratePointCloudParameters& params,
//                                     const size_t                        frameCount,
//                                     PCCPointSet3&                       subReconstruct,
//                                     std::vector<uint32_t>&              subPartition,
//                                     std::vector<size_t>&                subReconstructIndex ) {
//  subReconstruct.clear();
//  subPartition.clear();
//  subReconstructIndex.clear();
//  auto&        pointToPixel = frame.getPointToPixel();
//  const size_t pointCount   = reconstruct.getPointCount();
//  if ( ( pointCount == 0U ) || !reconstruct.hasColors() ) { return; }
//  for ( size_t i = 0; i < pointCount; ++i ) {
//    const PCCVector3<size_t> location = pointToPixel[i];
//    const size_t             f        = location[2];
//    if ( f < frameCount ) {
//      subReconstruct.addPoint( reconstruct[i] );
//      subReconstruct.setType( frameCount, POINT_UNSET );
//      subPartition.push_back( partition[i] );
//      subReconstructIndex.push_back( i );
//    }
//  }
//  subReconstruct.addColors();
//}
//
//void PCCCodec::updateReconstruct( PCCPointSet3&              reconstruct,
//                                  const PCCPointSet3&        subReconstruct,
//                                  const std::vector<size_t>& subReconstructIndex ) {
//  if ( subReconstruct.getPointCount() > 0 ) {
//    for ( size_t i = 0; i < subReconstruct.getPointCount(); ++i ) {
//      reconstruct[subReconstructIndex[i]] = subReconstruct[i];
//    }
//  }
//}

size_t PCCCodec::colorPointCloud( PCCPointSet3&                       reconstruct,
                                  PCCContext&                         context,
                                  PCCFrameContext&                    tile,
                                  const std::vector<bool>&            absoluteT1List,
                                  const size_t                        multipleStreams,
                                  const uint8_t                       attributeCount,
                                  size_t                              accTilePointCount,
                                  const GeneratePointCloudParameters& params ) {
  TRACE_CODEC( "colorPointCloud start \n" );

  if ( reconstruct.getPointCount() == 0 ) return accTilePointCount;

#ifdef CODEC_TRACE
  printChecksum( reconstruct, "colorPointCloud in" );
#endif
  if ( reconstruct.getPointCount() == 0 ) return false;
  auto&        videoTexture       = context.getVideoTextureMultiple()[0];
  auto&        videoTextureFrame1 = context.getVideoTextureMultiple()[1];
  const size_t mapCount           = params.mapCountMinus1_ + 1;
  size_t       numOfRawPointGeos  = 0;
  size_t       numberOfEOMPoints  = 0;
  if ( attributeCount == 0 ) {
    for ( auto& color : reconstruct.getColors() ) {
      for ( size_t c = 0; c < 3; ++c ) { color[c] = static_cast<uint8_t>( 127 ); }
    }
  } else {
    auto& pointToPixel = tile.getPointToPixel();
    auto& color16bit   = reconstruct.getColors16bit();
    bool  useAuxVideo  = tile.getUseRawPointsSeparateVideo();
    numOfRawPointGeos  = tile.getTotalNumberOfRawPoints();
    numberOfEOMPoints  = tile.getTotalNumberOfEOMPoints();
    size_t pointCount  = tile.getTotalNumberOfRegularPoints();

    if ( !useAuxVideo ) {
      pointCount += numOfRawPointGeos + numberOfEOMPoints;
    }
    TRACE_CODEC( "plt.getProfileCodecGroupIdc() = %d \n",
                 context.getVps().getProfileTierLevel().getProfileCodecGroupIdc() );

    TRACE_CODEC( "vps.getRawPatchEnabledFlag()  = %d \n", context.getVps().getAuxiliaryVideoPresentFlag( 0 ) );
    TRACE_CODEC( "useAuxVideo                   = %d \n", useAuxVideo );
    TRACE_CODEC( "numOfRawGeos                  = %zu \n", numOfRawPointGeos );
    if ( params.enhancedOccupancyMapCode_ ) {
      TRACE_CODEC( "numberOfRawPointsAndEOMColors      = %zu \n", numOfRawPointGeos + numberOfEOMPoints );
      TRACE_CODEC( "numberOfEOMPoints            = %zu \n", numberOfEOMPoints );
    }
    TRACE_CODEC( "pointCount                   = %zu \n", pointCount );
    TRACE_CODEC( "pointToPixel size            = %zu \n", pointToPixel.size() );
    TRACE_CODEC( "pointLocalReconstruction     = %d \n", params.pointLocalReconstruction_ );
    TRACE_CODEC( "singleLayerPixelInterleaving = %d \n", params.singleMapPixelInterleaving_ );
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
          const auto& image = videoTexture.getFrame( shift );
          for ( size_t c = 0; c < 3; ++c ) { color16bit[i][c] = image.getValue( c, x, y ); }
          int index = source.addPoint( reconstruct[i] );
          source.setColor16bit( index, color16bit[i] );
        } else {
          target.addPoint( reconstruct[i] );
          targetIndex.push_back( i );
        }
      } else if ( multipleStreams != 0U ) {
        if ( f == 0 ) {
          const auto& image = videoTexture.getFrame( tile.getFrameIndex() );
          for ( size_t c = 0; c < 3; ++c ) { color16bit[i][c] = image.getValue( c, x, y ); }
          size_t index = source.addPoint( reconstruct[i] );
          source.setColor16bit( index, color16bit[i] );
        } else {
          const auto& image0   = videoTexture.getFrame( tile.getFrameIndex() );
          const auto& image1   = videoTextureFrame1.getFrame( tile.getFrameIndex() );
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
          const auto& frame = videoTexture.getFrame( shift + f );
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
      std::vector<PCCColor3B>& rawTextures = tile.getRawPointsTextures();
      std::vector<PCCColor3B>& eomTextures = tile.getEOMTextures();
      for ( size_t i = 0; i < numberOfEOMPoints; ++i ) {
        color16bit[accTilePointCount + pointCount + i].r() = (uint16_t)eomTextures[i].r();
        color16bit[accTilePointCount + pointCount + i].g() = (uint16_t)eomTextures[i].g();
        color16bit[accTilePointCount + pointCount + i].b() = (uint16_t)eomTextures[i].b();
      }
      for ( size_t i = 0; i < numOfRawPointGeos; ++i ) {
        color16bit[accTilePointCount + pointCount + numberOfEOMPoints + i].r() = (uint16_t)rawTextures[i].r();
        color16bit[accTilePointCount + pointCount + numberOfEOMPoints + i].g() = (uint16_t)rawTextures[i].g();
        color16bit[accTilePointCount + pointCount + numberOfEOMPoints + i].b() = (uint16_t)rawTextures[i].b();
      }
    }
  }  // noAtt

#ifdef CODEC_TRACE
  printChecksum( reconstruct, "colorPointCloud out" );
  TRACE_CODEC( "colorPointCloud done \n" );
#endif

  return accTilePointCount + tile.getTotalNumberOfRegularPoints() + tile.getTotalNumberOfEOMPoints() +
         tile.getTotalNumberOfRawPoints();
}

void PCCCodec::generateRawPointsGeometryfromVideo( PCCContext& context, size_t frameIndex ) {
  TRACE_CODEC( " generateRawPointsGeometryfromVideo start \n" );
  auto& videoRawPointsGeometry = context.getVideoRawPointsGeometry();
  videoRawPointsGeometry.resize( context.size() );
  for ( size_t tileIdx = 0; tileIdx < context.getFrame( frameIndex ).getNumTilesInAtlasFrame(); tileIdx++ ) {
    auto& tile = context.getFrame( frameIndex ).getTile( tileIdx );
    generateRawPointsGeometryfromVideo( context, tile, frameIndex );
    size_t totalNumRawPoints = 0;
    for ( size_t i = 0; i < tile.getNumberOfRawPointsPatches(); i++ ) {
      totalNumRawPoints += tile.getRawPointsPatch( i ).sizeX();
    }
    std::cout << "generate raw Points Video (Geometry) frame[" << frameIndex << "] tile[" << tileIdx << "] : "
              << "#rawPatches: " << tile.getNumberOfRawPointsPatches() << "#rawPoints(pixels): " << totalNumRawPoints
              << std::endl;
  }
  std::cout << "Raw Points Geometry from Video [done]" << std::endl;
  TRACE_CODEC( " generateRawPointsGeometryfromVideo done \n" );
}

void PCCCodec::generateRawPointsGeometryfromVideo( PCCContext& context, PCCFrameContext& tile, size_t frameIndex ) {
  auto&  videoRawPointsGeometry   = context.getVideoRawPointsGeometry();
  auto&  image                    = videoRawPointsGeometry.getFrame( frameIndex );
  size_t numberOfRawPointsPatches = tile.getNumberOfRawPointsPatches();
  bool   isAuxiliarygeometrys444  = false;  // yo- use geo auxiliary codecID

  //jkei: it will contaminate the source. the reconstructed vaules should be saved in the different location.
  for ( int i = 0; i < numberOfRawPointsPatches; i++ ) {
    auto&        rawPointsPatch = tile.getRawPointsPatch( i );
    const size_t v0             = rawPointsPatch.v0_ * rawPointsPatch.occupancyResolution_;
    const size_t u0             = rawPointsPatch.u0_ * rawPointsPatch.occupancyResolution_;
    rawPointsPatch.sizeV_       = rawPointsPatch.sizeV0_ * rawPointsPatch.occupancyResolution_;
    rawPointsPatch.sizeU_       = rawPointsPatch.sizeU0_ * rawPointsPatch.occupancyResolution_;
    size_t numberOfRawPoints    = rawPointsPatch.getNumberOfRawPoints();
    if ( !isAuxiliarygeometrys444 ) { numberOfRawPoints *= 3; }
    rawPointsPatch.resize( numberOfRawPoints );
    for ( size_t v = 0; v < rawPointsPatch.sizeV_; ++v ) {
      for ( size_t u = 0; u < rawPointsPatch.sizeU_; ++u ) {
        const size_t p = v * rawPointsPatch.sizeU_ + u;
        if ( p < numberOfRawPoints ) {
          const size_t x = ( u0 + u );
          const size_t y = ( v0 + v ) + context.getAuxTileLeftTopY( tile.getTileIndex() );
          if ( isAuxiliarygeometrys444 ) {
            rawPointsPatch.x_[p] = image.getValue( 0, x, y );
            rawPointsPatch.y_[p] = image.getValue( 1, x, y );
            rawPointsPatch.z_[p] = image.getValue( 2, x, y );
          } else {
            rawPointsPatch.x_[p] = image.getValue( 0, x, y );
          }
        }
      }
    }
  }
}

void PCCCodec::generateRawPointsTexturefromVideo( PCCContext& context, size_t frameIndex ) {
  auto& videoRawPointsTexture = context.getVideoRawPointsTexture();
  videoRawPointsTexture.resize( context.size() );
  TRACE_CODEC( "generateRawPointsTexturefromVideo \n" );
  for ( size_t tileIdx = 0; tileIdx < context.getFrame( frameIndex ).getNumTilesInAtlasFrame(); tileIdx++ ) {
    auto& tile = context.getFrame( frameIndex ).getTile( tileIdx );
    generateRawPointsTexturefromVideo( context, tile, frameIndex );

    std::cout << "generate raw Points Video (Texture) frame[" << frameIndex << "] tile[" << tileIdx << "] : "
              << " #rawPatches: " << tile.getNumberOfRawPointsPatches()
              << " #rawPoints: " << tile.getTotalNumberOfRawPoints() << " #eomPoints: " << tile.getEomPatches().size()
              << " #eomPoints: " << tile.getTotalNumberOfEOMPoints() << std::endl;
  }
  std::cout << "RawPoints Texture [done]" << std::endl;
  TRACE_CODEC( "generateRawPointsTexturefromVideo done \n" );
}

void PCCCodec::generateRawPointsTexturefromVideo( PCCContext& context, PCCFrameContext& tile, size_t frameIndex ) {
  auto&  videoRawPointsTexture = context.getVideoRawPointsTexture();
  auto&  image                 = videoRawPointsTexture.getFrame( frameIndex );
  size_t width                 = image.getWidth();
  //  size_t height                = image.getHeight();

  size_t                   numberOfEOMPoints = tile.getTotalNumberOfEOMPoints();
  size_t                   numberOfRawPoints = tile.getTotalNumberOfRawPoints();
  std::vector<PCCColor3B>& rawTextures       = tile.getRawPointsTextures();
  std::vector<PCCColor3B>& eomTextures       = tile.getEOMTextures();
  rawTextures.resize( numberOfRawPoints );
  eomTextures.resize( numberOfEOMPoints );
  size_t numberOfRawPointsPatches = tile.getNumberOfRawPointsPatches();

  if ( numberOfRawPoints ) {
    // jkei: this part is different frome the spec TODO: update the spec
    size_t rawPatchOffset = 0;
    for ( int i = 0; i < numberOfRawPointsPatches; i++ ) {
      int    pointIndex        = 0;
      auto&  rawPointsPatch    = tile.getRawPointsPatch( i );
      size_t imageWidthInBlock = width / rawPointsPatch.occupancyResolution_;
      size_t numRawColorPoints = rawPointsPatch.getNumberOfRawPoints();
      auto   rawPointsPatchBlocks =
          static_cast<size_t>( ceil( double( numRawColorPoints ) /
                                     ( rawPointsPatch.occupancyResolution_ * rawPointsPatch.occupancyResolution_ ) ) );
      size_t rawPointsPatchColorSizeV0 =
          static_cast<size_t>( ceil( double( rawPointsPatchBlocks ) / imageWidthInBlock ) );
      size_t rawPointsPatchColorSizeU0 =
          static_cast<size_t>( ceil( double( rawPointsPatchBlocks ) / rawPointsPatchColorSizeV0 ) );
      printf(
          "\tgenerateRawPointsTextureImage:: (u0,v0) %zu,%zu, (sizeU,sizeU:geo) %zux%zu, (sizeU,sizeU:text) %zux%zu\n",
          rawPointsPatch.u0_, rawPointsPatch.v0_, rawPointsPatch.sizeU_, rawPointsPatch.sizeV_,
          rawPointsPatchColorSizeU0 * rawPointsPatch.occupancyResolution_, rawPointsPatch.occupancyResolution_ );
      const size_t v0 = rawPointsPatch.v0_ * rawPointsPatch.occupancyResolution_;
      const size_t u0 = rawPointsPatch.u0_ * rawPointsPatch.occupancyResolution_;

      for ( size_t v = 0; v < rawPointsPatchColorSizeV0 * rawPointsPatch.occupancyResolution_; ++v ) {
        for ( size_t u = 0; u < rawPointsPatchColorSizeU0 * rawPointsPatch.occupancyResolution_; ++u ) {
          if ( pointIndex < numRawColorPoints ) {
            const size_t x = ( u0 + u );
            const size_t y = ( v0 + v ) + context.getAuxTileLeftTopY( tile.getTileIndex() );
            assert( x < width && y < image.getHeight() );
            rawTextures[rawPatchOffset + pointIndex].r() = image.getValue( 0, x, y );
            rawTextures[rawPatchOffset + pointIndex].g() = image.getValue( 1, x, y );
            rawTextures[rawPatchOffset + pointIndex].b() = image.getValue( 2, x, y );
            pointIndex++;
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
      std::vector<PCCColor3B>& eomTextures    = tile.getEOMTextures();
      size_t                   patchStartPosX = eomPointsPatch.u0_ * eomPointsPatch.occupancyResolution_;
      size_t                   patchStartPosY = eomPointsPatch.v0_ * eomPointsPatch.occupancyResolution_;
      for ( size_t k = 0; k < eomPointsPatch.eomCount_; k++ ) {
        size_t nBlock = k / 256;
        size_t uBlock = nBlock % ( width / 16 );
        size_t vBlock = nBlock / ( width / 16 );
        xx            = patchStartPosX + uBlock * 16 + ( nPixelInCurrentBlockCount % 16 );
        yy            = patchStartPosY + vBlock * 16 + ( nPixelInCurrentBlockCount / 16 ) +
             context.getAuxTileLeftTopY( tile.getTileIndex() );
        ++nPixelInCurrentBlockCount;
        if ( nPixelInCurrentBlockCount >= 256 ) { nPixelInCurrentBlockCount = 0; }
        eomTextures[k + eomPatchOffset].r() = image.getValue( 0, xx, yy );
        eomTextures[k + eomPatchOffset].g() = image.getValue( 1, xx, yy );
        eomTextures[k + eomPatchOffset].b() = image.getValue( 2, xx, yy );
      }
      eomPatchOffset += eomPointsPatch.eomCount_;
    }  // eomPatches
    
   
    
  }
}
void PCCCodec::generateOccupancyMap( PCCFrameContext& tile,
                                     PCCImageOccupancyMap& videoFrame,
                                     const size_t occupancyPrecision,
                                     const size_t thresholdLossyOM,
                                     const bool   enhancedOccupancyMapForDepthFlag ) {
  auto  width        = tile.getWidth();
  auto  height       = tile.getHeight();
  auto& occupancyMap = tile.getOccupancyMap();
  occupancyMap.resize( width * height, 0 );
  for ( size_t v = 0; v < height; ++v ) {
    for ( size_t u = 0; u < width; ++u ) {
      uint8_t pixel = videoFrame.getValue( 0, tile.getLeftTopXInFrame() / occupancyPrecision + u / occupancyPrecision,
                                           tile.getLeftTopYInFrame() / occupancyPrecision + v / occupancyPrecision );
      if ( !enhancedOccupancyMapForDepthFlag ) {
        if ( pixel <= thresholdLossyOM ) {
          occupancyMap[v * width + u] = 0;
        } else {
          occupancyMap[v * width + u] = 1;
        }
      } else {
        occupancyMap[v * width + u] = pixel;
      }
    }
  }

#if MULTITILE_BUGFIX //jkei: do we need this for non - lossy occupancy case as well?
  for ( size_t yy = 0; yy < tile.getHeight(); yy++ )
    for ( size_t xx = 0; xx < tile.getWidth(); xx++ ) {
      videoFrame.setValue( 0,  (xx + tile.getLeftTopXInFrame())/occupancyPrecision, (yy + tile.getLeftTopYInFrame())/occupancyPrecision,
                           occupancyMap[ yy * width + xx ] );
    }
#else
  for ( size_t yy = 0; yy < tile.getHeight(); yy++ )
    for ( size_t xx = 0; xx < tile.getWidth(); xx++ ) {
      videoFrame.setValue( 0, xx + tile.getLeftTopXInFrame(), yy + tile.getLeftTopYInFrame(),
                           occupancyMap[( yy * occupancyPrecision ) * width + xx * occupancyPrecision] );
    }
#endif
}

void PCCCodec::generateBlockToPatchFromBoundaryBox( PCCContext&      context,
                                                    PCCFrameContext& frame,
                                                    const size_t     occupancyResolution ) {
  auto&        patches            = frame.getPatches();
  const size_t patchCount         = patches.size();
  const size_t blockToPatchWidth  = frame.getWidth() / occupancyResolution;
  const size_t blockToPatchHeight = frame.getHeight() / occupancyResolution;
  const size_t blockCount         = blockToPatchWidth * blockToPatchHeight;
  auto&        blockToPatch       = frame.getBlockToPatch();
  blockToPatch.resize( blockCount );
  std::fill( blockToPatch.begin(), blockToPatch.end(), 0 );
  for ( size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex ) {
    auto& patch = patches[patchIndex];
    for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
      for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
        const size_t blockIndex = patch.patchBlock2CanvasBlock( u0, v0, blockToPatchWidth, blockToPatchHeight );
        if ( context.getAtlasSequenceParameterSet( 0 ).getPatchPrecedenceOrderFlag() ) {
          if ( blockToPatch[blockIndex] == 0 ) { blockToPatch[blockIndex] = patchIndex + 1; }
        } else {
          blockToPatch[blockIndex] = patchIndex + 1;
        }
      }  // u0
    }    // v0
  }      // patch
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
    if ( context[fi].getNumTilesInAtlasFrame() != 1 ) {
      PCCFrameContext&      tile          = context.getFrame( fi ).getTitleFrameContext();
      PCCImageOccupancyMap& occupancyImage = context.getVideoOccupancyMap().getFrame( fi );
      generateBlockToPatchFromOccupancyMapVideo( context, tile, fi, occupancyImage, occupancyResolution,
                                                 occupancyPrecision );
    }
  }  // frame
}

void PCCCodec::generateBlockToPatchFromOccupancyMapVideo( PCCContext&           context,
                                                          PCCFrameContext&      tile,  // tile
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
          }  // u1
        }    // v1
        if ( nonZeroPixel > 0 ) { blockToPatch[blockIndex] = patchIndex + 1; }
      }  // u0
    }    // v0
  }      // patch
}

void PCCCodec::generateAfti( PCCContext& context, size_t frameIndex, AtlasFrameTileInformation& aftiLocal ) {
  // encoder
  auto& partitionInfoPerFrame = context[frameIndex];
  aftiLocal.setSingleTileInAtlasFrameFlag( partitionInfoPerFrame.getNumTilesInAtlasFrame() == 1 );
  if ( partitionInfoPerFrame.getNumTilesInAtlasFrame() == 1 ) {
    if ( partitionInfoPerFrame.getTitleFrameContext().getUseRawPointsSeparateVideo() ) {
      aftiLocal.setAuxiliaryVideoTileRowWidthMinus1( context.getAuxVideoWidth() / 64 - 1 );
      for ( size_t ti = 0; ti < partitionInfoPerFrame.getNumTilesInAtlasFrame(); ti++ ) {
        aftiLocal.setAuxiliaryVideoTileRowHeight( ti, context.getAuxTileHeight( ti ) );
      }
    } else{
      aftiLocal.setAuxiliaryVideoTileRowWidthMinus1(0 );
      for ( size_t ti = 0; ti < partitionInfoPerFrame.getNumTilesInAtlasFrame(); ti++ ) {
        aftiLocal.setAuxiliaryVideoTileRowHeight( ti, 0 );
      }
    }
    return;
  }
  aftiLocal.setUniformPartitionSpacingFlag( partitionInfoPerFrame.getUniformPartitionSpacing() );
  aftiLocal.setNumPartitionColumnsMinus1( partitionInfoPerFrame.getNumPartitionCols() - 1 );
  aftiLocal.setNumPartitionRowsMinus1( partitionInfoPerFrame.getNumPartitionRows() - 1 );
  //jkei: getPartitionWidth is pixel precision
  for ( size_t col = 0; col < partitionInfoPerFrame.getNumPartitionCols(); col++ )
    aftiLocal.setPartitionColumnWidthMinus1( col, partitionInfoPerFrame.getPartitionWidth()[col] / 64 - 1 );
  for ( size_t row = 0; row < partitionInfoPerFrame.getNumPartitionRows(); row++ )
    aftiLocal.setPartitionRowHeightMinus1( row, partitionInfoPerFrame.getPartitionHeight()[row] / 64 - 1 );

  aftiLocal.setSinglePartitionPerTileFlag( partitionInfoPerFrame.getSinglePartitionPerTile() );
  aftiLocal.setNumTilesInAtlasFrameMinus1( partitionInfoPerFrame.getNumTilesInAtlasFrame() -
                                           1 );  // partitionToTileMap.size()
#if 1
  if ( partitionInfoPerFrame.getSignalledTileId() == true ) { exit( 100 ); }
#endif
  aftiLocal.setSignalledTileIdFlag( partitionInfoPerFrame.getSignalledTileId() );
  if ( partitionInfoPerFrame.getSignalledTileId() ) {
    aftiLocal.setSignalledTileIdLengthMinus1( ceil( log2( partitionInfoPerFrame.getNumTilesInAtlasFrame() + 1 ) ) - 1 );
    for ( size_t ti = 0; ti < partitionInfoPerFrame.getNumTilesInAtlasFrame(); ti++ ) {
      aftiLocal.setTileId( ti, partitionInfoPerFrame.getTileId()[ti] );
    }
  } else{
    for ( size_t ti = 0; ti < partitionInfoPerFrame.getNumTilesInAtlasFrame(); ti++ ) {
      aftiLocal.setTileId( ti, ti );
    }

  }

  auto& atlasFrame = context.getFrame( frameIndex );
  for ( size_t ti = 0; ti < partitionInfoPerFrame.getNumTilesInAtlasFrame(); ti++ ) {
    auto& tile = atlasFrame.getTile( ti );
#if TILE_PARTITINING_BUGFIX
    if( tile.getPatches().size()==0 && partitionInfoPerFrame.getTitleFrameContext().getUseRawPointsSeparateVideo() ){
      aftiLocal.setTopLeftPartitionIdx( ti, 0 );
      aftiLocal.setBottomRightPartitionColumnOffset( ti, 0 );
      aftiLocal.setBottomRightPartitionRowOffset( ti, 0 );
#if 1
      printf( "enc:%zu frame %zu tile:(%zu,%zu), Raw Tile %zux%zu in Auxiliary video\n", frameIndex,
             ti, atlasFrame.getTile( ti ).getLeftTopXInFrame(), atlasFrame.getTile( ti ).getLeftTopYInFrame(),
             atlasFrame.getTile( ti ).getWidth(), atlasFrame.getTile( ti ).getHeight() );
#endif
      continue;
    }
#endif
    int    leftTopX          = tile.getLeftTopXInFrame();
    int    leftTopY          = tile.getLeftTopYInFrame();
    int    bottomRightDeltaX = tile.getWidth();
    int    bottomRightDeltaY = tile.getHeight();
    size_t numPartLeftX = 0, numPartLeftY = 0;
    while ( leftTopX > 0 ) {
      leftTopX -= partitionInfoPerFrame.getPartitionWidth()[numPartLeftX];
      numPartLeftX += 1;
    }
    while ( leftTopY > 0 ) {
      leftTopY -= partitionInfoPerFrame.getPartitionHeight()[numPartLeftY];
      numPartLeftY += 1;
    }
    size_t topLeftIdx = numPartLeftX + numPartLeftY * partitionInfoPerFrame.getNumPartitionCols();
    aftiLocal.setTopLeftPartitionIdx( ti, topLeftIdx );

    size_t numPartBottomX = 0, numPartBottomY = 0;
#if TILE_PARTITINING_BUGFIX //jkei: it has been okay since it was uniformsize
    while ( bottomRightDeltaX > 0 ) {
      bottomRightDeltaX -= partitionInfoPerFrame.getPartitionWidth()[ numPartLeftX + numPartBottomX];
      numPartBottomX += 1;
    }
    while ( bottomRightDeltaY > 0 ) {
      bottomRightDeltaY -= partitionInfoPerFrame.getPartitionHeight()[ numPartLeftY + numPartBottomY];
      numPartBottomY += 1;
    }
#else
    while ( bottomRightDeltaX > 0 ) {
      bottomRightDeltaX -= partitionInfoPerFrame.getPartitionWidth()[numPartBottomX];
      numPartBottomX += 1;
    }
    while ( bottomRightDeltaY > 0 ) {
      bottomRightDeltaY -= partitionInfoPerFrame.getPartitionHeight()[numPartBottomY];
      numPartBottomY += 1;
    }
#endif
    assert( numPartBottomX >= 1 && numPartBottomY >= 1 );

    if ( ( numPartBottomX < 1 ) || ( numPartBottomY < 1 ) ) {
      printf(
          "enc:<error> %zu frame %zu tile:(%zu,%zu), %zux%zu -> "
          "leftIdx(%zu,%zu), bottom(%zu,%zu) : ((numPartBottomX<1) || (numPartBottomY <1))\n",
          frameIndex, ti, atlasFrame.getTile( ti ).getLeftTopXInFrame(), atlasFrame.getTile( ti ).getLeftTopYInFrame(),
          atlasFrame.getTile( ti ).getWidth(), atlasFrame.getTile( ti ).getHeight(), numPartLeftX, numPartLeftY,
          numPartBottomX, numPartBottomY );
      printf( "enc:colWidth,rowHeight\n" );
      for ( size_t ii = 0; ii < partitionInfoPerFrame.getPartitionWidth().size(); ii++ )
        printf( "\t%zu", partitionInfoPerFrame.getPartitionWidth()[ii] );
      printf( "\n" );
      for ( size_t ii = 0; ii < partitionInfoPerFrame.getPartitionHeight().size(); ii++ )
        printf( "\t%zu", partitionInfoPerFrame.getPartitionHeight()[ii] );
      printf( "\n" );
      exit( 125 );
    }

    aftiLocal.setBottomRightPartitionColumnOffset( ti, numPartBottomX - 1 );
    aftiLocal.setBottomRightPartitionRowOffset( ti, numPartBottomY - 1 );

#if 1
    printf( "enc:%zu frame %zu tile:(%zu,%zu), %zux%zu -> leftIdx(%zu,%zu), bottom(%zu,%zu) -> %u,%u,%u\n", frameIndex,
            ti, atlasFrame.getTile( ti ).getLeftTopXInFrame(), atlasFrame.getTile( ti ).getLeftTopYInFrame(),
            atlasFrame.getTile( ti ).getWidth(), atlasFrame.getTile( ti ).getHeight(), numPartLeftX, numPartLeftY,
            numPartBottomX, numPartBottomY, aftiLocal.getTopLeftPartitionIdx( ti ),
            aftiLocal.getBottomRightPartitionColumnOffset( ti ), aftiLocal.getBottomRightPartitionRowOffset( ti ) );
#endif
  }
  if ( partitionInfoPerFrame.getTitleFrameContext().getUseRawPointsSeparateVideo() ) {
    aftiLocal.setAuxiliaryVideoTileRowWidthMinus1( context.getAuxVideoWidth() / 64 - 1 );
    for ( size_t ti = 0; ti < partitionInfoPerFrame.getNumTilesInAtlasFrame(); ti++ ) {
#if TILE_PARTITINING_BUGFIX2
      aftiLocal.setAuxiliaryVideoTileRowHeight( ti, context.getAuxTileHeight( ti )/64 );
#else
      aftiLocal.setAuxiliaryVideoTileRowHeight( ti, context.getAuxTileHeight( ti ) );
#endif
    }
#if 1
    printf( "enc:%zu frame auxiliaryVideoTileRowWidthMinus1(): width(64x)%u\t height: ", frameIndex,
            aftiLocal.getAuxiliaryVideoTileRowWidthMinus1() );
    for ( size_t ti = 0; ti < partitionInfoPerFrame.getNumTilesInAtlasFrame(); ti++ )
      printf( "%u\t", aftiLocal.getAuxiliaryVideoTileRowHeight( ti ) );
    printf( "\n" );
#endif
  } else{
       aftiLocal.setAuxiliaryVideoTileRowWidthMinus1(0 );
       for ( size_t ti = 0; ti < partitionInfoPerFrame.getNumTilesInAtlasFrame(); ti++ ) {
         aftiLocal.setAuxiliaryVideoTileRowHeight( ti, 0 );
       }
     }
}
void PCCCodec::setTilePartitionSizeAfti( PCCContext& context ) {
  // decoder
  for ( size_t afpsIdx = 0; afpsIdx < context.getAtlasFrameParameterSetList().size(); afpsIdx++ ) {
    auto& afps = context.getAtlasFrameParameterSet( afpsIdx );
    auto& asps = context.getAtlasSequenceParameterSet( afps.getAtlasFrameParameterSetId() );
    auto& afti = afps.getAtlasFrameTileInformation();

    size_t frameWidth  = asps.getFrameWidth();
    size_t frameHeight = asps.getFrameHeight();

    size_t numPartitionCols = afti.getNumPartitionColumnsMinus1() + 1;
    size_t numPartitionRows = afti.getNumPartitionRowsMinus1() + 1;
    auto&  partitionWidth   = afti.getColWidth();
    auto&  partitionHeight  = afti.getRowHeight();
    auto&  partitionPosX    = afti.getColWidth();
    auto&  partitionPosY    = afti.getRowHeight();
    partitionWidth.resize( numPartitionCols );
    partitionHeight.resize( numPartitionRows );
    partitionPosX.resize( numPartitionCols );
    partitionPosY.resize( numPartitionRows );

    if ( afti.getUniformPartitionSpacingFlag() ) {
      size_t uniformPatitionWidth  = 64 * ( afti.getPartitionColumnWidthMinus1( 0 ) + 1 );
      size_t uniformPatitionHeight = 64 * ( afti.getPartitionRowHeightMinus1( 0 ) + 1 );
      partitionPosX[0]             = 0;
      partitionWidth[0]            = uniformPatitionWidth;
      for ( size_t col = 1; col < numPartitionCols - 1; col++ ) {
        partitionPosX[col]  = partitionPosX[col - 1] + partitionWidth[col - 1];
        partitionWidth[col] = uniformPatitionWidth;
      }
      if ( numPartitionCols > 1 ) {
        partitionPosX[numPartitionCols - 1] =
            partitionPosX[numPartitionCols - 2] + partitionWidth[numPartitionCols - 2];
        partitionWidth[numPartitionCols - 1] = frameWidth - partitionPosX[numPartitionCols - 1];
      }

      partitionPosY[0]   = 0;
      partitionHeight[0] = uniformPatitionHeight;
      for ( size_t row = 1; row < numPartitionRows - 1; row++ ) {
        partitionPosY[row]   = partitionPosY[row - 1] + partitionHeight[row - 1];
        partitionHeight[row] = uniformPatitionHeight;
      }
      if ( numPartitionRows > 1 ) {
        partitionPosY[numPartitionRows - 1] =
            partitionPosY[numPartitionRows - 2] + partitionHeight[numPartitionRows - 2];
        partitionHeight[numPartitionRows - 1] = frameHeight - partitionPosY[numPartitionRows - 1];
      }
    } else {
      // jkei:we need to figure out how to set non-uniform partition spcaing
      printf( "non uniform tile partitioning\n" );
      exit( 129 );
      size_t multiPatitionWidth  = 64 * ( afti.getPartitionColumnWidthMinus1( 0 ) + 1 );
      size_t multiPatitionHeight = 64 * ( afti.getPartitionRowHeightMinus1( 0 ) + 1 );
      for ( size_t col = 0; col < numPartitionCols; col++ ) {
        partitionWidth[col] = multiPatitionWidth;  // jkei: input needs to be array
        partitionPosX[col]  = partitionPosX[col - 1] + partitionWidth[col];
      }
      for ( size_t row = 0; row < numPartitionRows; row++ ) {
        partitionHeight[row] = multiPatitionHeight;
        partitionPosY[row]   = partitionPosY[row - 1] + partitionHeight[row];
      }
    }


    if ( asps.getAuxiliaryVideoEnabledFlag() ) {
      context.setAuxVideoWidth( ( afti.getAuxiliaryVideoTileRowWidthMinus1() + 1 ) * 64 );
      context.getAuxTileLeftTopY().resize( afti.getNumTilesInAtlasFrameMinus1() + 1,
                                           0 );  // jkei: do we need it per frame?
      context.getAuxTileHeight().resize( afti.getNumTilesInAtlasFrameMinus1() + 1, 0 );
      for ( size_t ti = 0; ti <= afti.getNumTilesInAtlasFrameMinus1(); ti++ ) {
#if TILE_PARTITINING_BUGFIX2
        context.setAuxTileHeight( ti, afti.getAuxiliaryVideoTileRowHeight( ti )*64 );
        if ( ti < afti.getNumTilesInAtlasFrameMinus1() )
          context.setAuxTileLeftTopY( ti + 1,
                                      context.getAuxTileLeftTopY( ti ) + context.getAuxTileHeight( ti ) );
#else
        context.setAuxTileHeight( ti, afti.getAuxiliaryVideoTileRowHeight( ti ) );
        if ( ti < afti.getNumTilesInAtlasFrameMinus1() )
          context.setAuxTileLeftTopY( ti + 1,
                                      context.getAuxTileLeftTopY( ti ) + afti.getAuxiliaryVideoTileRowHeight( ti ) );
#endif
      }
    }
  }  // afpsIdx
}

size_t PCCCodec::setTileSizeAndLocation( PCCContext& context, size_t frameIndex, AtlasTileHeader& ath ) {
  size_t afpsIdx   = ath.getAtlasFrameParameterSetId();
  auto&  afps      = context.getAtlasFrameParameterSet( afpsIdx );
  auto&  asps      = context.getAtlasSequenceParameterSet( afps.getAtlasSequenceParameterSetId() );
  auto&  afti      = afps.getAtlasFrameTileInformation();
  size_t tileIndex = 0;

  if ( afti.getSingleTileInAtlasFrameFlag() ) {
    if ( context[frameIndex].getNumTilesInAtlasFrame() == 0 ) {
      context[frameIndex].setAtlasFrameWidth( asps.getFrameWidth() );
      context[frameIndex].setAtlasFrameHeight( asps.getFrameHeight() );
      context[frameIndex].setNumTilesInAtlasFrame( 1 );
      context[frameIndex].updatePartitionInfoPerFrame(
          frameIndex, asps.getFrameWidth(), asps.getFrameHeight(), afti.getNumTilesInAtlasFrameMinus1() + 1,
          afti.getUniformPartitionSpacingFlag(), afti.getPartitionColumnWidthMinus1( 0 ) + 1,
          afti.getPartitionRowHeightMinus1( 0 ) + 1, afti.getNumPartitionColumnsMinus1() + 1,
          afti.getNumPartitionRowsMinus1() + 1, afti.getSinglePartitionPerTileFlag(), afti.getSignalledTileIdFlag() );
    } else {
      assert( context[frameIndex].getAtlasFrameWidth() == ( asps.getFrameWidth() ) );
      assert( context[frameIndex].getAtlasFrameHeight() == ( asps.getFrameHeight() ) );
      assert( context[frameIndex].getNumTilesInAtlasFrame() == 1 );
    }
    auto& tile = context[frameIndex].getTile( 0 );
    tile.setTileIndex( tileIndex );
    tile.setLeftTopXInFrame( 0 );
    tile.setLeftTopYInFrame( 0 );
    tile.setWidth( asps.getFrameWidth() );
    tile.setHeight( asps.getFrameHeight() );
  } else {
    if ( context[frameIndex].getNumTilesInAtlasFrame() == 0 ) {
      context[frameIndex].updatePartitionInfoPerFrame(
          frameIndex, asps.getFrameWidth(), asps.getFrameHeight(), afti.getNumTilesInAtlasFrameMinus1() + 1,
          afti.getUniformPartitionSpacingFlag(), afti.getPartitionColumnWidthMinus1( 0 ) + 1,
          afti.getPartitionRowHeightMinus1( 0 ) + 1, afti.getNumPartitionColumnsMinus1() + 1,
          afti.getNumPartitionRowsMinus1() + 1, afti.getSinglePartitionPerTileFlag(), afti.getSignalledTileIdFlag() );
      context[frameIndex].initNumTiles( context[frameIndex].getNumTilesInAtlasFrame() );
    } else {
      assert( context[frameIndex].getAtlasFrameWidth() == asps.getFrameWidth() );
      assert( context[frameIndex].getAtlasFrameHeight() == asps.getFrameHeight() );
      assert( context[frameIndex].getNumTilesInAtlasFrame() == ( afti.getNumTilesInAtlasFrameMinus1() + 1 ) );
      assert( context[frameIndex].getUniformPartitionSpacing() == afti.getUniformPartitionSpacingFlag() );
      assert( context[frameIndex].getSinglePartitionPerTile() == afti.getSinglePartitionPerTileFlag() );
      assert( context[frameIndex].getNumPartitionCols() == ( afti.getNumPartitionColumnsMinus1() + 1 ) );
      assert( context[frameIndex].getNumPartitionRows() == ( afti.getNumPartitionRowsMinus1() + 1 ) );
      assert( context[frameIndex].getSignalledTileId() == afti.getSignalledTileIdFlag() );
    }

    tileIndex   = afti.getSignalledTileIdFlag() ? afti.getTileId( ath.getId() ) : ath.getId();
    auto&  tile = context[frameIndex].getTile( tileIndex );
    size_t TopLeftPartitionColumn =
        afti.getTopLeftPartitionIdx( tileIndex ) % ( afti.getNumPartitionColumnsMinus1() + 1 );
    size_t TopLeftPartitionRow = afti.getTopLeftPartitionIdx( tileIndex ) / ( afti.getNumPartitionColumnsMinus1() + 1 );
    size_t BottomRightPartitionColumn = TopLeftPartitionColumn + afti.getBottomRightPartitionColumnOffset( tileIndex );
    size_t BottomRightPartitionRow    = TopLeftPartitionRow + afti.getBottomRightPartitionRowOffset( tileIndex );

    size_t tileStartX = context[frameIndex].getPartitionPosX()[TopLeftPartitionColumn];
    size_t tileStartY = context[frameIndex].getPartitionPosY()[TopLeftPartitionRow];
    size_t tileWidth  = 0;
    size_t tileHeight = 0;
    for ( size_t j = TopLeftPartitionColumn; j <= BottomRightPartitionColumn; j++ ) {
      tileWidth += ( context[frameIndex].getPartitionWidth()[j] );
    }
    for ( size_t j = TopLeftPartitionRow; j <= BottomRightPartitionRow; j++ ) {
      tileHeight += ( context[frameIndex].getPartitionHeight()[j] );
    }
    tile.setLeftTopXInFrame( tileStartX );
    tile.setLeftTopYInFrame( tileStartY );

    if ( ( tile.getLeftTopXInFrame() + tileWidth ) >= context[frameIndex].getAtlasFrameWidth() )
      tileWidth = context[0].getAtlasFrameWidth() - tile.getLeftTopXInFrame();
    if ( ( tile.getLeftTopYInFrame() + tileHeight ) >= context[frameIndex].getAtlasFrameHeight() )
      tileHeight = context[0].getAtlasFrameHeight() - tile.getLeftTopYInFrame();

    tile.setWidth( tileWidth );
    tile.setHeight( tileHeight );

    assert( tile.getLeftTopXInFrame() < asps.getFrameWidth() );
    assert( tile.getLeftTopYInFrame() < asps.getFrameHeight() );

#if 1
    auto& atlasFrame = context[frameIndex];
    printf( "dec:%zu frame %zu tile:(%zu,%zu), %zux%zu -> leftIdx(%zu,%zu), bottom(%zu,%zu) -> %u,%u,%u\n", frameIndex,
            tileIndex, atlasFrame.getTile( tileIndex ).getLeftTopXInFrame(),
            atlasFrame.getTile( tileIndex ).getLeftTopYInFrame(), atlasFrame.getTile( tileIndex ).getWidth(),
            atlasFrame.getTile( tileIndex ).getHeight(), TopLeftPartitionColumn, TopLeftPartitionRow,
            BottomRightPartitionColumn, BottomRightPartitionRow, afti.getTopLeftPartitionIdx( tileIndex ),
            afti.getBottomRightPartitionColumnOffset( tileIndex ), afti.getBottomRightPartitionRowOffset( tileIndex ) );
#endif
  }
  return tileIndex;
}

//sei hash
void PCCCodec::getHashPatchParams( PCCContext&                            context,
                                   size_t                                 frameIndex,
                                   size_t                                 tileIndex,
                                   size_t                                 atlIndex,
                                   std::vector<std::vector<PatchParams>>& tilePatchParams,
                                   std::vector<PatchParams>&               atlasPatchParams ) {
  //jkei: it needs to be use reconstructed patch information
  //ajt:: the original version was done under the assumption of bitstream level hash value derivation and not on the reconstructed.
  //ajt:: It may be the matter of wrong interpretation. I do agree to change it to reconstructed, instead.
  auto&       atl           = context.getAtlasTileLayer( atlIndex );
  auto& atdu = atl.getDataUnit();
  auto& ath = atl.getHeader();
  auto& afps = context.getAtlasFrameParameterSet( ath.getAtlasFrameParameterSetId() );
  auto& asps = context.getAtlasSequenceParameterSet( afps.getAtlasSequenceParameterSetId() );
  auto&       afti          = afps.getAtlasFrameTileInformation();
  size_t      topLeftColumn = afti.getTopLeftPartitionIdx( tileIndex ) % ( afti.getNumPartitionColumnsMinus1() + 1 );
  size_t      topLeftRow    = afti.getTopLeftPartitionIdx( tileIndex ) / ( afti.getNumPartitionColumnsMinus1() + 1 );
  size_t      tileOffsetX   = context[frameIndex].getPartitionPosX()[topLeftColumn];
  size_t      tileOffsetY   = context[frameIndex].getPartitionPosY()[topLeftRow];
  PCCTileType tileType = ath.getType();
  auto& tile = context.getFrame( frameIndex ).getTile( tileIndex );
  size_t patchCount = tile.getPatches().size();// + tile.getRawPointsPatches().size() + tile.getEomPatches().size();
  for ( size_t patchIdx = 0; patchIdx < patchCount; patchIdx++ ) {
    PatchParams  pps( asps.getPLREnabledFlag(), asps.getMapCountMinus1() + 1 );
    auto&        patch         = tile.getPatch(patchIdx);
    assert( getPatchType( tileType, atdu.getPatchMode( patchIdx ) )!= RAW_PATCH && getPatchType( tileType, atdu.getPatchMode( patchIdx )) != EOM_PATCH );
    pps.patchType      = PCCHashPatchType::PROJECTED;
    pps.patch2dPosX    = patch.getU0();
    pps.patch2dPosY    = patch.getV0();
    pps.patch2dSizeX   = patch.getSizeU0();
    pps.patch2dSizeY   = patch.getSizeV0();
    pps.patch3dOffsetU = patch.getU1();
    pps.patch3dOffsetV = patch.getV1();
    pps.patch3dOffsetD = patch.getD1();
    if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() )
      pps.patch3dRangeD  = patch.getSizeD(); //pdu.get3dRangeD();
    else
      pps.patch3dRangeD  = 0;
    pps.patchOrientationIndex = patch.getPatchOrientation();
    pps.patchProjectionID     = patch.getProjectionMode();
    pps.patchInAuxVideo       = asps.getRawPatchEnabledFlag();  // ajt::check
    pps.patchLoDScaleX = patch.getLodScaleX();
    pps.patchLoDScaleY = patch.getLodScaleY();
    
    if(tilePatchParams.size()!=0) tilePatchParams[tileIndex].push_back( pps );
    pps.patch2dPosX += tileOffsetX;
    pps.patch2dPosY += tileOffsetY;
    atlasPatchParams.push_back( pps );
  }
  
  size_t rawPatchCount = tile.getRawPointsPatches().size();
  for ( size_t patchIdx = 0; patchIdx < rawPatchCount; patchIdx++ ) {
    PatchParams  pps( asps.getPLREnabledFlag(), asps.getMapCountMinus1() + 1 );
    auto&        rawPointsPatch         = tile.getRawPointsPatch(patchIdx);
    assert( getPatchType( tileType, atdu.getPatchMode( patchIdx ) )== RAW_PATCH  );
    pps.patchType       = RAW;
    pps.patchRawPoints  = rawPointsPatch.getNumberOfRawPoints();
    pps.patch2dPosX     = rawPointsPatch.u0_;
    pps.patch2dPosY     = rawPointsPatch.v0_;
    pps.patch2dSizeX    = rawPointsPatch.sizeU0_;
    pps.patch2dSizeY    = rawPointsPatch.sizeV0_;
    pps.patch3dOffsetU  = rawPointsPatch.u1_;
    pps.patch3dOffsetV  = rawPointsPatch.v1_;
    pps.patch3dOffsetD  = rawPointsPatch.d1_;
    pps.patchInAuxVideo = rawPointsPatch.isPatchInAuxVideo_;
    
    if(tilePatchParams.size()!=0) tilePatchParams[tileIndex].push_back( pps );
    pps.patch2dPosX += tileOffsetX;
    pps.patch2dPosY += tileOffsetY;
    atlasPatchParams.push_back( pps );
  }
  
  size_t eomPatchCount = tile.getEomPatches().size();
  for ( size_t patchIdx = 0; patchIdx < eomPatchCount; patchIdx++ ) {
    PatchParams  pps( asps.getPLREnabledFlag(), asps.getMapCountMinus1() + 1 );
    auto&        eomPatch         = tile.getEomPatches(patchIdx);
    assert( getPatchType( tileType, atdu.getPatchMode( patchIdx ) )== EOM_PATCH  );
    
    pps.patchType                = EOM;
    pps.patch2dPosX              = eomPatch.u0_;
    pps.patch2dPosY              = eomPatch.v0_;
    pps.patch2dSizeX             = eomPatch.sizeU_;
    pps.patch2dSizeY             = eomPatch.sizeV_;
    pps.patchInAuxVideo          = eomPatch.isPatchInAuxVideo_;
    
    pps.epduAssociatedPatchCount =  eomPatch.memberPatches.size();
    pps.epduAssociatedPoints.resize( pps.epduAssociatedPatchCount );
    for ( size_t i = 0; i < pps.epduAssociatedPatchCount; i++ )
      pps.epduAssociatedPoints[i] = eomPatch.eomCountPerPatch[i];

    if(tilePatchParams.size()!=0) tilePatchParams[tileIndex].push_back( pps );
    pps.patch2dPosX += tileOffsetX;
    pps.patch2dPosY += tileOffsetY;
    atlasPatchParams.push_back( pps );
    
  }

//     size_t patchCount = tile.getPatches().size() + tile.getRawPointsPatches().size() + tile.getEomPatches().size();
//     for ( size_t patchIdx = 0; patchIdx < patchCount; patchIdx++ ) {
//       PatchParams  pps( asps.getPLREnabledFlag(), asps.getMapCountMinus1() + 1 );
//       auto&        patch         = tile.getPatch(patchIdx);
//       PCCPatchType currPatchType = getPatchType( tileType, atdu.getPatchMode( patchIdx ) );
//    if ( currPatchType == INTRA_PATCH ){
//      auto& pdu = pid.getPatchDataUnit();
//      pps.patchType      = PCCHashPatchType::PROJECTED;
//      pps.patch2dPosX    = pdu.get2dPosX();
//      pps.patch2dPosY    = pdu.get2dPosY();
//      pps.patch2dSizeX   = pdu.get2dSizeXMinus1();
//      pps.patch2dSizeY   = pdu.get2dSizeYMinus1();
//      pps.patch3dOffsetU = pdu.get3dOffsetU();
//      pps.patch3dOffsetV = pdu.get3dOffsetV();
//      pps.patch3dOffsetD = pdu.get3dOffsetD();
//      if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() )
//      pps.patch3dRangeD  = patch.getSizeD(); //pdu.get3dRangeD();
//      else
//        pps.patch3dRangeD  = 0;
//      pps.patchOrientationIndex = pdu.getOrientationIndex();
//      pps.patchProjectionID     = pdu.getProjectionId();
//      pps.patchInAuxVideo       = asps.getRawPatchEnabledFlag();  // ajt::check
//      pps.patchLoDScaleX = pdu.getLodScaleXMinus1() + 1;
//      pps.patchLoDScaleY = pdu.getLodScaleYIdc();
//    }
//    else if ( currPatchType == INTER_PATCH ) {
//      //jkei: these should be reconstructed values
//      auto& ipdu = pid.getInterPatchDataUnit();
//      pps.patchType      = PROJECTED;
//      pps.patch2dPosX    = ipdu.get2dPosX();
//      pps.patch2dPosY    = ipdu.get2dPosY();
//      pps.patch2dSizeX   = ipdu.get2dDeltaSizeX();
//      pps.patch2dSizeY   = ipdu.get2dDeltaSizeY();
//      pps.patch3dOffsetU = ipdu.get3dOffsetU();
//      pps.patch3dOffsetV = ipdu.get3dOffsetV();
//      pps.patch3dOffsetD = ipdu.get3dOffsetD();
//      if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() )
//      pps.patch3dRangeD  = patch.getSizeD();
//      else
//        pps.patch3dRangeD  = 0;
//      // ajt::need to check these three below for correctness
//      pps.patchOrientationIndex = patch.getPatchOrientation();
//      pps.patchProjectionID     = patch.getProjectionMode();
//      pps.patchInAuxVideo       = asps.getRawPatchEnabledFlag(); // ajt::check
//      pps.patchLoDScaleX        = patch.getLodScaleX();
//      pps.patchLoDScaleY        = patch.getLodScaleY();
//    }
//    else if ( currPatchType == MERGE_PATCH ) {
//      //jkei: these should be reconstructed values
//      auto& mpdu         = pid.getMergePatchDataUnit();
//      pps.patchType      = PROJECTED;
//      pps.patch2dPosX    = mpdu.get2dPosX();
//      pps.patch2dPosY    = mpdu.get2dPosY();
//      pps.patch2dSizeX   = mpdu.get2dDeltaSizeX();
//      pps.patch2dSizeY   = mpdu.get2dDeltaSizeY();
//      pps.patch3dOffsetU = mpdu.get3dOffsetU();
//      pps.patch3dOffsetV = mpdu.get3dOffsetV();
//      pps.patch3dOffsetD = mpdu.get3dOffsetD();
//      pps.patch3dRangeD  = patch.getSizeD();
//
//      // ajt::need to check these three below for correctness
//      pps.patchOrientationIndex = patch.getPatchOrientation();
//      pps.patchProjectionID     = patch.getProjectionMode();
//      pps.patchInAuxVideo       = asps.getRawPatchEnabledFlag();
//      pps.patchLoDScaleX        = patch.getLodScaleX();
//      pps.patchLoDScaleY        = patch.getLodScaleY();
//    } else if ( currPatchType == SKIP_PATCH ) {
//      pps.patchType      = PROJECTED;
//      pps.patch2dPosX    = patch.getU0();
//      pps.patch2dPosY    = patch.getV0();
//      pps.patch2dSizeX   = patch.getSizeU0();
//      pps.patch2dSizeY   = patch.getSizeV0();
//      pps.patch3dOffsetU = patch.getU1();
//      pps.patch3dOffsetV = patch.getV1();
//      pps.patch3dOffsetD = patch.getD1();
//      pps.patch3dRangeD  = patch.getSizeD();
//
//      // ajt::need to check these three below for correctness
//      pps.patchOrientationIndex = patch.getPatchOrientation();
//      pps.patchProjectionID     = patch.getProjectionMode();
//      pps.patchInAuxVideo       = asps.getRawPatchEnabledFlag();
//      pps.patchLoDScaleX        = patch.getLodScaleX();
//      pps.patchLoDScaleY        = patch.getLodScaleY();
//    } else if ( currPatchType == RAW_PATCH ) {
//      auto& rpdu          = pid.getRawPatchDataUnit();
//      pps.patchType       = RAW;
//      pps.patchRawPoints  = rpdu.getRawPointsMinus1() + 1;
//      pps.patch2dPosX     = rpdu.get2dPosX();
//      pps.patch2dPosY     = rpdu.get2dPosY();
//      pps.patch2dSizeX    = rpdu.get2dSizeXMinus1();
//      pps.patch2dSizeY    = rpdu.get2dSizeYMinus1();
//      pps.patch3dOffsetU  = rpdu.get3dOffsetU();
//      pps.patch3dOffsetV  = rpdu.get3dOffsetV();
//      pps.patch3dOffsetD  = rpdu.get3dOffsetD();
//      pps.patchInAuxVideo = rpdu.getPatchInAuxiliaryVideoFlag();
//    } else if ( currPatchType == EOM_PATCH ) {
//      auto& epdu                   = pid.getEomPatchDataUnit();
//      pps.patchType                = EOM;
//      pps.patch2dPosX              = epdu.get2dPosX();
//      pps.patch2dPosY              = epdu.get2dPosY();
//      pps.patch2dSizeX             = epdu.get2dSizeXMinus1();
//      pps.patch2dSizeY             = epdu.get2dSizeYMinus1();
//      pps.patchInAuxVideo          = epdu.getPatchInAuxiliaryVideoFlag();
//      pps.epduAssociatedPatchCount = epdu.getPatchCountMinus1()+1; //  eomPatch.memberPatches.size(); jkei: why is it  eomPatch.memberPatches.size()?
//      pps.epduAssociatedPoints.resize( pps.epduAssociatedPatchCount );
//      for ( size_t i = 0; i < pps.epduAssociatedPatchCount; i++ )
//        pps.epduAssociatedPoints[i] = epdu.getPoints( i );
//    }
//    atlasPatchParams.push_back( pps );
//    if(tilePatchParams.size()!=0) tilePatchParams[tileIndex].push_back( pps );
 
}



void PCCCodec::getB2PHashPatchParams( PCCContext&                                    context,
                                          size_t                                     frameIndex,
                                          std::vector<std::vector<std::vector<int64_t>>>& b2pTilePatchParams,
                                          std::vector<std::vector<int64_t>>& b2pAtlasPatchParams ) {

  size_t atlIdx     = context[frameIndex].getTile( 0 ).getAtlIndex();
  auto&  tileHeader = context.getAtlasTileLayerList()[atlIdx].getHeader();
  size_t afpsIndex  = tileHeader.getAtlasFrameParameterSetId();
  size_t aspsIndex  = context.getAtlasFrameParameterSet( afpsIndex ).getAtlasFrameParameterSetId();
  auto&  asps       = context.getAtlasSequenceParameterSet( aspsIndex );
  auto&  afps       = context.getAtlasFrameParameterSet( afpsIndex );
  auto&  afti       = afps.getAtlasFrameTileInformation();
  
  size_t      numTilesInPatchFrame = context[frameIndex].getNumTilesInAtlasFrame();
  size_t      patchPackingBlockSize = ( 1 << asps.getLog2PatchPackingBlockSize() );
  size_t      offset                = patchPackingBlockSize - 1;

  size_t atlasBlockToPatchMapWidth  = ( asps.getFrameWidth() + offset ) / patchPackingBlockSize;
  size_t atlasBlockToPatchMapHeight = ( asps.getFrameHeight() + offset ) / patchPackingBlockSize;

  b2pTilePatchParams.resize( numTilesInPatchFrame );
  b2pAtlasPatchParams.resize( atlasBlockToPatchMapHeight );
  for ( auto& e : b2pAtlasPatchParams ) e.resize( atlasBlockToPatchMapWidth );

  for ( size_t y = 0; y < atlasBlockToPatchMapHeight; y++ ) {
    for ( size_t x = 0; x < atlasBlockToPatchMapWidth; x++ ) b2pAtlasPatchParams[y][x] = -1;
  }

  size_t offsetPatch = 0;
  
  for ( size_t tileIdx = 0; tileIdx < numTilesInPatchFrame; tileIdx++ ) {
    
    auto&       tile         = context[frameIndex].getTile( tileIdx );
    size_t      atlIdx       = tile.getAtlIndex();
    auto&       ath   = context.getAtlasTileLayerList()[atlIdx].getHeader();
    auto&       atdu = context.getAtlasTileLayerList()[atlIdx].getDataUnit();
   
    PCCTileType tileType = ath.getType();

    size_t topLeftColumn = afti.getTopLeftPartitionIdx( tileIdx ) % ( afti.getNumPartitionColumnsMinus1() + 1 );
    size_t topLeftRow    = afti.getTopLeftPartitionIdx( tileIdx ) / ( afti.getNumPartitionColumnsMinus1() + 1 );
    size_t tileOffsetBlkX = context[frameIndex].getPartitionPosX()[topLeftColumn] / patchPackingBlockSize;
    size_t tileOffsetBlkY    = context[frameIndex].getPartitionPosY()[topLeftRow] / patchPackingBlockSize;

    size_t tileBlockToPatchMapWidth  = ( tile.getWidth() + offset ) / patchPackingBlockSize;
    size_t tileBlockToPatchMapHeight = ( tile.getHeight() + offset ) / patchPackingBlockSize;
    for ( auto& e : b2pTilePatchParams ) e.resize( tileBlockToPatchMapHeight );
    for ( size_t y = 0; y < tileBlockToPatchMapHeight; y++ ) {
      b2pTilePatchParams[tileIdx][y].resize( tileBlockToPatchMapWidth );
      for ( size_t x = 0; x < tileBlockToPatchMapWidth; x++ ) b2pTilePatchParams[tileIdx][y][x] = -1;
    }
    const size_t patchCount =
        tile.getPatches().size() + tile.getRawPointsPatches().size() + tile.getEomPatches().size();
    for ( size_t p = 0; p < patchCount; p++ ) {
      if ( getPatchType( tileType, atdu.getPatchMode( p ) ) != RAW_PATCH &&
           getPatchType( tileType, atdu.getPatchMode( p ) ) != EOM_PATCH ) {
        size_t xOrg               = tile.getPatch( p ).getU0() / patchPackingBlockSize;
        size_t yOrg               = tile.getPatch( p ).getV0() / patchPackingBlockSize;
        size_t tilePatchWidthBlk  = ( tile.getPatch( p ).getSizeU0() + offset ) / patchPackingBlockSize;
        size_t tilePatchHeightBlk = ( tile.getPatch( p ).getSizeV0() + offset ) / patchPackingBlockSize;
        for ( size_t y = 0; y < tilePatchHeightBlk; y++ )
          for ( size_t x = 0; x < tilePatchWidthBlk; x++ ) {
            if ( ( asps.getPatchPrecedenceOrderFlag() == 0 ) ||
                 ( b2pTilePatchParams[tileIdx][yOrg + y][xOrg + x] == -1 ) ) {
              b2pTilePatchParams[tileIdx][yOrg + y][xOrg + x] = p;
              b2pAtlasPatchParams[tileOffsetBlkY + yOrg + y][tileOffsetBlkX + xOrg + x] = p + offsetPatch;
            }
          }
      } else if ( getPatchType( tileType, atdu.getPatchMode( p ) ) == RAW_PATCH &&
                  tile.getRawPointsPatches()[p].isPatchInAuxVideo_ == 0 ) {
        auto&    rawPointsPatch     = tile.getRawPointsPatch( p );
        uint32_t xOrg               = tile.getRawPointsPatch( p ).u0_ / patchPackingBlockSize;
        uint32_t yOrg               = tile.getRawPointsPatch( p ).v0_ / patchPackingBlockSize;
        uint32_t tilePatchWidthBlk  = ( tile.getRawPointsPatch( p ).u0_ + offset ) / patchPackingBlockSize;
        uint32_t tilePatchHeightBlk = ( tile.getRawPointsPatch( p ).v0_ + offset ) / patchPackingBlockSize;
        for ( uint32_t y = 0; y < tilePatchHeightBlk; y++ )
          for ( uint32_t x = 0; x < tilePatchWidthBlk; x++ ) { 
              b2pTilePatchParams[tileIdx][y][x] = p;
              b2pAtlasPatchParams[tileOffsetBlkY + yOrg + y][tileOffsetBlkX + xOrg + x] = p + offsetPatch;
          }
      } else if ( getPatchType( tileType, atdu.getPatchMode( p ) ) == EOM_PATCH &&
                  tile.getEomPatches()[p].isPatchInAuxVideo_ == 0 ) {
        size_t   rawPatchCount      = tile.getRawPointsPatches().size();
        auto&    eomPointsPatch     = tile.getEomPatches( p );
        uint32_t xOrg               = tile.getEomPatches( p ).u0_ / patchPackingBlockSize;
        uint32_t yOrg               = tile.getEomPatches( p ).v0_ / patchPackingBlockSize;
        uint32_t tilePatchWidthBlk  = ( tile.getEomPatches( p ).u0_ + offset ) / patchPackingBlockSize;
        uint32_t tilePatchHeightBlk = ( tile.getEomPatches( p ).v0_ + offset ) / patchPackingBlockSize;
        for ( uint32_t y = 0; y < tilePatchHeightBlk; y++ )
          for ( uint32_t x = 0; x < tilePatchWidthBlk; x++ ) { 
              b2pTilePatchParams[tileIdx][y][x] = p;
              b2pAtlasPatchParams[tileOffsetBlkY + yOrg + y][tileOffsetBlkX + xOrg + x] = p + offsetPatch;
          }
      }
    }
    offsetPatch += patchCount;
  }
}

void PCCCodec::aspsCommonByteString( std::vector<uint8_t>& stringByte, AtlasSequenceParameterSetRbsp& asps ) {

  uint8_t val  = asps.getFrameWidth() & 0xFF;
  stringByte.push_back( val );
  val = ( asps.getFrameWidth() >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = ( asps.getFrameWidth() >> 16 ) & 0xFF;
  stringByte.push_back( val );
  val = ( asps.getFrameWidth() >> 24 ) & 0xFF;
  stringByte.push_back( val );
  val = asps.getFrameHeight() & 0xFF;
  stringByte.push_back( val );
  val = ( asps.getFrameHeight() >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = ( asps.getFrameHeight() >> 16 ) & 0xFF;
  stringByte.push_back( val );
  val = ( asps.getFrameHeight() >> 24 ) & 0xFF;
  stringByte.push_back( val );
  val = asps.getGeometry3dBitdepthMinus1() & 0xFF;
  stringByte.push_back( val );
  val = asps.getGeometry2dBitdepthMinus1() & 0xFF;
  stringByte.push_back( val );
  val = asps.getMapCountMinus1() & 0xFF;
  stringByte.push_back( val );
  val = asps.getMaxNumberProjectionsMinus1() & 0xFF;
  stringByte.push_back( val );
  val = asps.getPatchPrecedenceOrderFlag() & 0xFF;
  stringByte.push_back( val );
}

void PCCCodec::aspsApplicationByteString( std::vector<uint8_t>& stringByte,
                                                    AtlasSequenceParameterSetRbsp& asps,
                                                    AtlasFrameParameterSetRbsp&    afps ) {
  uint8_t val;
  if ( asps.getPixelDeinterleavingFlag() ) {
    for ( int j = 0; j <= asps.getMapCountMinus1(); j++ ) {
      val = asps.getPixelDeinterleavingMapFlag( j ) & 0xFF;  // asps_map_pixel_deinterleaving_flag[ j ]
      stringByte.push_back( val );
    }
  }

  val = asps.getRawPatchEnabledFlag() & 0xFF;
  stringByte.push_back( val );
  val = asps.getEomPatchEnabledFlag() & 0xFF;
  stringByte.push_back( val );
  if ( asps.getEomPatchEnabledFlag() && asps.getMapCountMinus1() == 0 ) {
    val = asps.getEomFixBitCountMinus1() & 0xFF;
    stringByte.push_back( val );
  }
  if ( asps.getAuxiliaryVideoEnabledFlag() && ( asps.getRawPatchEnabledFlag() || asps.getEomPatchEnabledFlag() ) ) {
    auto&   afti            = afps.getAtlasFrameTileInformation();
    size_t  auxVideoWidthNF = ( afti.getAuxiliaryVideoTileRowWidthMinus1() + 1 ) * 64;
    uint8_t val             = auxVideoWidthNF & 0xFF;  // val = AuxVideoWidthNF & 0xFF
    stringByte.push_back( val );
    val = ( auxVideoWidthNF >> 8 ) & 0xFF;  // val = ( AuxVideoWidthNF >> 8 ) & 0xFF;
    stringByte.push_back( val );
    size_t auxVideoHeightNF = 0;
    for ( int i = 0; i <= afti.getNumTilesInAtlasFrameMinus1(); i++ )
      auxVideoHeightNF += ( afti.getAuxiliaryVideoTileRowHeight( i ) * 64 );
    val = auxVideoHeightNF & 0xFF;  // val = AuxVideoHeightNF & 0xFF
    stringByte.push_back( val );
    val = ( auxVideoHeightNF >> 8 ) & 0xFF;  //( AuxVideoHeightNF >> 8 ) & 0xFF
    stringByte.push_back( val );
  }
  val = asps.getPLREnabledFlag() & 0xFF;
  stringByte.push_back( val );
  if ( asps.getPLREnabledFlag() ) {
    for ( int i = 0; i < asps.getMapCountMinus1() + 1; i++ ) {
      val = asps.getPLRInformation( i ).getMapEnabledFlag() & 0xFF;  // plri_map_present_flag in the spec?
      stringByte.push_back( val );
      if ( asps.getPLRInformation( i ).getMapEnabledFlag() ) {
        val = asps.getPLRInformation( i ).getNumberOfModesMinus1() & 0xFF;
        stringByte.push_back( val );
        for ( int j = 0; j < asps.getPLRInformation( i ).getNumberOfModesMinus1() + 1; j++ ) {
          val = asps.getPLRInformation( i ).getInterpolateFlag( j ) & 0xFF;
          stringByte.push_back( val );
          val = asps.getPLRInformation( i ).getFillingFlag( j ) & 0xFF;
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
  val       = ext.getRemoveDuplicatePointEnableFlag() & 0xFF;
  stringByte.push_back( val );
  if ( asps.getPixelDeinterleavingFlag() || asps.getPLREnabledFlag() ) {
    val = ext.getSurfaceThicknessMinus1() & 0xFF;
    stringByte.push_back( val );
    val = ( ext.getSurfaceThicknessMinus1() >> 8 ) & 0xFF;
    stringByte.push_back( val );
  }
}


void PCCCodec::afpsCommonByteString( std::vector<uint8_t>& stringByte,
                                     PCCContext&           context,
                                     size_t                afpsIndex,
                                     size_t                frameIndex ) {
  auto    afps = context.getAtlasFrameParameterSet( afpsIndex );
  auto    afti = afps.getAtlasFrameTileInformation();
  uint8_t val  = afti.getNumTilesInAtlasFrameMinus1() & 0xFF;
  stringByte.push_back( val );

  std::vector<size_t> hashAuxTileHeight;

  auto   asps             = context.getAtlasSequenceParameterSet( afps.getAtlasFrameParameterSetId() );
  size_t frameWidth       = asps.getFrameWidth();
  size_t frameHeight      = asps.getFrameHeight();
  size_t numPartitionCols = afti.getNumPartitionColumnsMinus1() + 1;
  size_t numPartitionRows = afti.getNumPartitionRowsMinus1() + 1;

  size_t prevAuxTileOffset   = 0;
  size_t hashAuxVideoWidthNF = 0;

  if ( asps.getAuxiliaryVideoEnabledFlag() ) {
    hashAuxVideoWidthNF = ( ( afti.getAuxiliaryVideoTileRowWidthMinus1() + 1 ) * 64 );
    hashAuxTileHeight.resize( afti.getNumTilesInAtlasFrameMinus1() + 1, 0 );
    for ( size_t ti = 0; ti <= afti.getNumTilesInAtlasFrameMinus1(); ti++ ) {
      hashAuxTileHeight[ti] = afti.getAuxiliaryVideoTileRowHeight( ti ) * 64;
    }
  } else {
    hashAuxTileHeight.resize( afti.getNumTilesInAtlasFrameMinus1() + 1, 0 );
  }

  for ( int i = 0; i < afti.getNumTilesInAtlasFrameMinus1() + 1;
        i++ ) {  // ajt:: i = 1 should change to i = 0 in the spec.!

    size_t   topLeftColumn     = afti.getTopLeftPartitionIdx( i ) % ( afti.getNumPartitionColumnsMinus1() + 1 );
    size_t   topLeftRow        = afti.getTopLeftPartitionIdx( i ) / ( afti.getNumPartitionColumnsMinus1() + 1 );
    size_t   bottomRightColumn = topLeftColumn + afti.getBottomRightPartitionColumnOffset( i );
    size_t   bottomRightRow    = topLeftRow + afti.getBottomRightPartitionRowOffset( i );
 
    size_t   tileWidth         = 0;
    size_t   tileHeight        = 0;

    size_t tileOffsetX   = context[frameIndex].getPartitionPosX()[topLeftColumn];
    size_t tileOffsetY   = context[frameIndex].getPartitionPosY()[topLeftRow];

    for ( int j = topLeftColumn; j <= bottomRightColumn; j++ ) { tileWidth += context[frameIndex].getPartitionWidth()[j]; }

    for ( int j = topLeftRow; j <= bottomRightRow; j++ ) { tileHeight += context[frameIndex].getPartitionHeight()[j]; }

    size_t auxTileHeight = hashAuxTileHeight[i];
    size_t auxTileOffset = prevAuxTileOffset + auxTileHeight;
    prevAuxTileOffset    = auxTileOffset;

    val = tileOffsetX & 0xFF;
    stringByte.push_back( val );  // val = TileOffsetX[i] & 0xFF;
    val = ( tileOffsetX >> 8 ) & 0xFF;
    stringByte.push_back( val );  // val = ( TileOffsetX[i] >> 8 ) & 0xFF;
    val = tileOffsetY & 0xFF;
    stringByte.push_back( val );  // val = TileOffsetY[i] & 0xFF;
    val = ( tileOffsetY >> 8 ) & 0xFF;
    stringByte.push_back( val );  // val = ( TileOffsetY[i] >> 8 ) & 0xFF;
    val = tileWidth & 0xFF;
    stringByte.push_back( val );  // val = TileWidth[i] & 0xFF;
    val = ( tileWidth >> 8 ) & 0xFF;
    stringByte.push_back( val );  // val = ( TileWidth[i] >> 8 ) & 0xFF;
    val = tileHeight & 0xFF;
    stringByte.push_back( val );  // val = TileHeight[i] & 0xFF;
    val = ( tileHeight >> 8 ) & 0xFF;
    stringByte.push_back( val );  // val = ( TileHeight[i] >> 8 ) & 0xFF;
    val = auxTileOffset & 0xFF;
    stringByte.push_back( val );  // val = AuxTileOffset[i] & 0xFF;
    val = ( auxTileOffset >> 8 ) & 0xFF;
    stringByte.push_back( val );  // val = ( AuxTileOffset[i] >> 8 ) & 0xFF;
    val = auxTileHeight & 0xFF;
    stringByte.push_back( val );  // val = AuxTileHeight[i] & 0xFF;
    val = ( auxTileHeight >> 8 ) & 0xFF;
    stringByte.push_back( val );  // val = AuxTileHeight[i] & 0xFF;
    if ( afps.getAtlasFrameTileInformation().getSignalledTileIdFlag() ) {
      val = afps.getAtlasFrameTileInformation().getTileId( i ) & 0xFF;  //
      stringByte.push_back( val );
      val = ( afps.getAtlasFrameTileInformation().getTileId( i ) >> 8 ) & 0xFF;
      stringByte.push_back( val );
    } else {
      val = i & 0xFF;
      stringByte.push_back( val );
      val = ( i >> 8 ) & 0xFF;
      stringByte.push_back( val );
    }
  }
}

void PCCCodec::afpsApplicationByteString( std::vector<uint8_t>& stringByte, AtlasSequenceParameterSetRbsp& asps, AtlasFrameParameterSetRbsp&    afps ) {}


void PCCCodec::atlasPatchCommonByteString( std::vector<uint8_t>&     stringByte,
                                           size_t                    p,
                                           std::vector<PatchParams>& atlasPatchParams ) {
  uint8_t val = atlasPatchParams[ p ].patchType & 0xFF;
  stringByte.push_back( val );  // uint8_t val = AtlasPatchType[p] & 0xFF;
  val = atlasPatchParams[ p ].patch2dPosX & 0xFF;
  stringByte.push_back( val );
  val = ( atlasPatchParams[ p ].patch2dPosX >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[ p ].patch2dPosY & 0xFF;
  stringByte.push_back( val );
  val = ( atlasPatchParams[ p ].patch2dPosY >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[ p ].patch2dSizeX & 0xFF;
  stringByte.push_back( val );
  val = ( atlasPatchParams[p].patch2dSizeX >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[p].patch2dSizeY & 0xFF;
  stringByte.push_back( val );
  val = ( atlasPatchParams[p].patch2dSizeY >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[p].patch3dOffsetU & 0xFF;
  stringByte.push_back( val );
  val = ( atlasPatchParams[ p ].patch3dOffsetU >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[ p ].patch3dOffsetV & 0xFF;
  stringByte.push_back( val );
  val = ( atlasPatchParams[ p ].patch3dOffsetV >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[ p ].patch3dOffsetD & 0xFF;
  stringByte.push_back( val );
  val = ( atlasPatchParams[ p ].patch3dOffsetD >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[ p ].patch3dRangeD & 0xFF;
  stringByte.push_back( val );
  val = ( atlasPatchParams[ p ].patch3dRangeD >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[ p ].patchProjectionID & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[ p ].patchOrientationIndex & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[ p ].patchLoDScaleX & 0xFF;
  stringByte.push_back( val );
  val = ( atlasPatchParams[ p ].patchLoDScaleX >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = atlasPatchParams[ p ].patchLoDScaleY & 0xFF;
  stringByte.push_back( val );
  val = ( atlasPatchParams[p].patchLoDScaleY >> 8 ) & 0xFF;
  stringByte.push_back( val );
};

void PCCCodec::atlasPatchApplicationByteString( std::vector<uint8_t>& stringByte, size_t p, std::vector<PatchParams>& atlasPatchParams ) {

  //uint32_t bSize = getPatchPackingBlockSize();
  uint8_t val = atlasPatchParams[ p ].patchInAuxVideo & 0xFF;
  stringByte.push_back( val );  // AtlasPatchInAuxVideo[p] & 0xFF;
  if ( atlasPatchParams[ p ].patchType == RAW ) {
    val = atlasPatchParams[ p ].patchRawPoints & 0xFF;
    stringByte.push_back( val ) ;
    val = ( atlasPatchParams[p].patchRawPoints >> 8 ) & 0xFF;
    stringByte.push_back( val );
  } else if ( atlasPatchParams[ p ].patchType == PROJECTED ) {
    /*
    size_t blockCnt = ( ( atlasPatchParams[ p ].patch2dSizeX + bSize - 1 ) / bSize ) *
                      ( ( atlasPatchParams[ p ].patch2dSizeY + bSize - 1 ) / bSize );

    for ( int m = 0; m < mapCount + 1; m++ ) { //ajt:: need to work on it later for multiple maps!
      if ( atlasPatchParams.patchPlrdLevel== 1 ) {
        if ( atlasPatchParams.patchPlrdLevel[p][m] == 0 ) {
          for ( int j = 0; j < blockCnt; j++ ) {
            val = atlasPatchParams.patchPlrdBlockModeMinus1[p][m][j] & 0xFF;
            stringByte.push_back( val );
            val = ( atlasPatchParams.[p][m][j] >> 8 ) & 0xFF;
            stringByte.push_back( val );
          }
        } else {
          val = atlasPatchParams.patchPlrdModeMinus1[p][m] & 0xFF;
            stringByte.push_back( val );
            val = ( AtlasPlrdBlockModeMinus1[p][m] >> 8 ) & 0xFF;
            stringByte.push_back( val );
        }
      }
    }*/
  } else if ( atlasPatchParams[ p ].patchType == EOM ) {
    // val = atlasPatchParams.patchEomPatchCount[ p ] & 0xFF; // ajt:: this needs to be checked
    // stringByte.push_back( val );
    // val = (atlasPatchParams.patchEomPatchCount[ p ] >> 8 ) & 0xFF;
    // stringByte.push_back( val );
    val = atlasPatchParams[ p ].epduAssociatedPatchCount & 0xFF;
    stringByte.push_back( val );
    val = ( atlasPatchParams[ p ].epduAssociatedPatchCount >> 8 ) & 0xFF;
    stringByte.push_back( val );
    for ( int i = 0; i < atlasPatchParams[ p ].epduAssociatedPatchCount; i++ ) {
      val = atlasPatchParams[ p ].epduAssociatedPoints[ i ] & 0xFF;
      stringByte.push_back( val );
      val = ( atlasPatchParams[ p ].epduAssociatedPoints[ i ] >> 8 ) & 0xFF;
      stringByte.push_back( val );
    }
  }
};

void PCCCodec::tilePatchCommonByteString( std::vector<uint8_t>& stringByte, size_t tileId, size_t p, std::vector<std::vector<PatchParams>>& tilePatchParams ) {
  uint8_t val = tilePatchParams[ tileId ][ p ].patchType & 0xFF;
  stringByte.push_back( val );  // uint8_t val = AtlasPatchType & 0xFF;
  val = tilePatchParams[ tileId ][ p ].patch2dPosX & 0xFF;
  stringByte.push_back( val );
  val = ( tilePatchParams[ tileId ][ p ].patch2dPosX >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[ tileId ][ p ].patch2dPosY & 0xFF;
  stringByte.push_back( val );
  val = ( tilePatchParams[ tileId ][ p ].patch2dPosY >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[ tileId ][ p ].patch2dSizeX & 0xFF;
  stringByte.push_back( val );
  val = ( tilePatchParams[ tileId ][ p ].patch2dSizeX >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[ tileId ][ p ].patch2dSizeY & 0xFF;
  stringByte.push_back( val );
  val = ( tilePatchParams[ tileId ][ p ].patch2dSizeY >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[ tileId ][ p ].patch3dOffsetU & 0xFF;
  stringByte.push_back( val );
  val = ( tilePatchParams[ tileId ][ p ].patch3dOffsetU >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[ tileId ][ p ].patch3dOffsetV & 0xFF;
  stringByte.push_back( val );
  val = ( tilePatchParams[ tileId ][ p ].patch3dOffsetV >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[ tileId ][ p ].patch3dOffsetD & 0xFF;
  stringByte.push_back( val );
  val = ( tilePatchParams[ tileId ][ p ].patch3dOffsetD >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[ tileId ][ p ].patch3dRangeD & 0xFF;
  stringByte.push_back( val );
  val = ( tilePatchParams[ tileId ][ p ].patch3dRangeD >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[ tileId ][ p ].patchProjectionID & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[ tileId ][ p ].patchOrientationIndex & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[ tileId ][ p ].patchLoDScaleX & 0xFF;
  stringByte.push_back( val );
  val = ( tilePatchParams[ tileId ][ p ].patchLoDScaleX >> 8 ) & 0xFF;
  stringByte.push_back( val );
  val = tilePatchParams[ tileId ][ p ].patchLoDScaleY & 0xFF;
  stringByte.push_back( val );
  val = ( tilePatchParams[ tileId ][ p ].patchLoDScaleY >> 8 ) & 0xFF;
  stringByte.push_back( val );
  
};

void PCCCodec::tilePatchApplicationByteString( std::vector<uint8_t>& stringByte, size_t tileId, size_t p, std::vector<std::vector<PatchParams>>& tilePatchParams ) {
  //uint32_t bSize = getPatchPackingBlockSize();
  uint8_t val = tilePatchParams[ tileId ][ p ].patchInAuxVideo & 0xFF; //jkei: why is it gAtlasPatchParams_?
  stringByte.push_back( val );  // AtlasPatchInAuxVideo & 0xFF;
  if ( tilePatchParams[ tileId ][ p ].patchType == RAW ) {
    val = tilePatchParams[ tileId ][ p ].patchRawPoints & 0xFF;
    stringByte.push_back( val );
    val = ( tilePatchParams[ tileId ][ p ].patchRawPoints >> 8 ) & 0xFF;
    stringByte.push_back( val );
  } else if ( tilePatchParams[ tileId ][ p ].patchType == PROJECTED ) {
    /*size_t blockCnt = ( ( tilePatchParams[ tileId ][ p ].patch2dSizeX + bSize - 1 ) / bSize ) *
                      ( ( tilePatchParams[ tileId ][ p ].patch2dSizeY + bSize - 1 ) / bSize );

    for ( int m = 0; m < mapCount + 1; m++ ) { //ajt:: need to work on it later for multiple maps!
      if ( tilePatchParams[ tileId ][ p ].patchPlrdLevel== 1 ) {
        if ( tilePatchParams[ tileId ][ p ].patchPlrdLevel[m] == 0 ) {
          for ( int j = 0; j < blockCnt; j++ ) {
            val = tilePatchParams[ tileId ][ p ].patchPlrdBlockModeMinus1[m][j] & 0xFF;
            stringByte.push_back( val );
            val = ( tilePatchParams[ tileId ][ p ].[m][j] >> 8 ) & 0xFF;
            stringByte.push_back( val );
          }
        } else {
          val = tilePatchParams[ tileId ][ p ].patchPlrdModeMinus1[m] & 0xFF;
            stringByte.push_back( val );
            val = ( AtlasPlrdBlockModeMinus1[m] >> 8 ) & 0xFF;
            stringByte.push_back( val );
        }
      }
    }*/

  } else if ( tilePatchParams[ tileId ][ p ].patchType == EOM ) {
    // val = tilePatchParams[ tileId ][ p ].patchEomPatchCount[ p ] & 0xFF; // ajt:: this needs to be checked
    // stringByte.push_back( val );
    // val = (tilePatchParams[ tileId ][ p ].patchEomPatchCount[ p ] >> 8 ) & 0xFF;
    // stringByte.push_back( val );
    val = tilePatchParams[ tileId ][ p ].epduAssociatedPatchCount & 0xFF;
    stringByte.push_back( val );
    val = ( tilePatchParams[ tileId ][ p ].epduAssociatedPatchCount >> 8 ) & 0xFF;
    stringByte.push_back( val );
    for ( int i = 0; i < tilePatchParams[ tileId ][ p ].epduAssociatedPatchCount; i++ ) {
      val = tilePatchParams[ tileId ][ p ].epduAssociatedPoints[i] & 0xFF;
      stringByte.push_back( val );
      val = ( tilePatchParams[ tileId ][ p ].epduAssociatedPoints[i] >> 8 ) & 0xFF;
      stringByte.push_back( val );
    }
  }
};
void PCCCodec::atlasBlockToPatchByteString( std::vector<uint8_t>& stringByte,std::vector<std::vector<int64_t>> atlasB2p ) {

  uint8_t b2pVal;
  for ( size_t y = 0; y < atlasB2p.size(); y++ ) {
    for ( size_t x = 0; x < atlasB2p[y].size(); x++ ) {
      b2pVal = ( atlasB2p[y][x] == -1 ) ? 0xFFFF : atlasB2p[y][x];
      stringByte.push_back( b2pVal & 0xFF );
      stringByte.push_back( ( b2pVal >> 8 ) & 0xFF );
    }
  }
}
void PCCCodec::tileBlockToPatchByteString( std::vector<uint8_t>&                         stringByte,
                                           size_t                                        tileId,
                                           std::vector<std::vector<std::vector<int64_t>>> tileB2p ) {

  uint8_t b2pVal;
  for ( size_t y = 0; y < tileB2p[tileId].size(); y++ ) {
    for ( size_t x = 0; x < tileB2p[tileId][y].size(); x++ ) {
      b2pVal = ( tileB2p[tileId][y][x] == -1 ) ? 0xFFFF : tileB2p[tileId][y][x];
      stringByte.push_back( b2pVal & 0xFF );
      stringByte.push_back( ( b2pVal >> 8 ) & 0xFF );
    }
  }
}

#ifdef CODEC_TRACE
void PCCCodec::printChecksum( PCCPointSet3& ePointcloud, std::string eString ) {
  auto checksum = ePointcloud.computeChecksum();
  TRACE_CODEC( "Checksum %s: ", eString.c_str() );
  for ( auto& c : checksum ) { TRACE_CODEC( "%02x", c ); }
  TRACE_CODEC( "\n" );
  printf( "Checksum %s: ", eString.c_str() );
  for ( auto& c : checksum ) { printf( "%02x", c ); }
  printf( "\n" );
  fflush( stdout );
}
#endif
