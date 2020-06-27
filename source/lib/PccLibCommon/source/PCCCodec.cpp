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

void PCCCodec::generatePointCloud( PCCGroupOfFrames&                   reconstructs,
                                   PCCContext&                         context,
                                   const GeneratePointCloudParameters& params,
                                   std::vector<std::vector<uint32_t>>& partitions,
                                   bool                                bDecoder ) {
  auto& frames = context.getFrames();
#ifdef ENABLE_PAPI_PROFILING
  PAPI_PROFILING_INITIALIZE;
#endif
  for ( size_t i = 0; i < frames.size(); i++ ) {
    TRACE_CODEC( " Frame %zu / %zu \n", i, frames.size() );
    std::vector<uint32_t> partition;
    generatePointCloud( reconstructs[i], context,
                        frames[i],  // videoGeometry, videoGeometryMultiple, videoOccupancyMap,
                        params, partition, bDecoder );
    partitions.push_back( partition );
  }
#ifdef ENABLE_PAPI_PROFILING
  PAPI_PROFILING_RESULTS;
#endif
  TRACE_CODEC( "Generate point Cloud done \n" );
}

bool PCCCodec::colorPointCloud( PCCGroupOfFrames&                     reconstructs,
                                PCCContext&                           context,
                                const uint8_t                         attributeCount,
                                const PCCColorTransform               colorTransform,
                                const std::vector<std::vector<bool>>& absoluteT1List,
                                const size_t                          multipleStreams,
                                const GeneratePointCloudParameters&   params ) {
  auto& frames = context.getFrames();
  for ( size_t i = 0; i < frames.size(); i++ ) {
    if(reconstructs[i].getPointCount()==0) continue;
    for ( size_t attIdx = 0; attIdx < attributeCount; attIdx++ ) {
      colorPointCloud( reconstructs[i], context, frames[i], absoluteT1List[attIdx], multipleStreams, attributeCount,
                       params );
    }
  }
  return true;
}

void PCCCodec::smoothPointCloudPostprocess( PCCPointSet3&                       reconstruct,
                                            PCCContext&                         context,
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
  if(reconstruct.getPointCount()==0) return;
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
      const size_t w = ( maxSize + static_cast<int>( params.gridSize_ ) - 1 ) / ( static_cast<int>( params.gridSize_ ) );

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
                               PCCContext&                         context,
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
  TRACE_CODEC( "  pcMaxSize = %d \n",pcMaxSize );
  TRACE_CODEC( "  gridSize  = %zu \n",gridSize );
  TRACE_CODEC( "  w         = %zu \n",w );
  TRACE_CODEC( "  w3        = %zu \n",w3 );
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
  std::fill( colorSmoothingPartition_.begin(), colorSmoothingPartition_.end(), 0 );
  std::fill( colorSmoothingDoSmooth_.begin(), colorSmoothingDoSmooth_.end(), 0 );
  colorSmoothingLum_.clear();
  colorSmoothingLum_.resize( numBoundaryCells );
  for ( int k = 0; k < reconstruct.getPointCount(); k++ ) {
    PCCPoint3D point  = reconstruct[k];
    int        x2     = point.x() / gridSize;
    int        y2     = point.y() / gridSize;
    int        z2     = point.z() / gridSize;
    size_t     cellId = x2 + y2 * w + z2 * w * w;
    if( cellId >= cellIndex.size() ){
      TRACE_CODEC( " cellId >  cellIndex.size() <=>  %zu > %zu \n",cellId, cellIndex.size() );      
    } else {
      if ( cellIndex[cellId] != -1 ) {
        PCCColor16bit color16bit = reconstruct.getColor16bit( k );
        PCCVector3D   clr;
        for ( size_t c = 0; c < 3; ++c ) { clr[c] = double( color16bit[c] ); }
        const size_t patchIndexPlusOne = reconstruct.getPointPatchIndex( k ) + 1;
        addGridColorCentroid( reconstruct[k], clr, patchIndexPlusOne, colorSmoothingCount_, colorSmoothingCenter_,
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
                                                  PCCFrameContext&                     frame,
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
  const auto& patch  = frame.getPatch( patchIndex );
  auto&       frame0 = videoGeometryMultiple[0].getFrame( videoFrameIndex );
  // params.multipleStreams_ ? videoGeometryMultiple[0].getFrame( videoFrameIndex ) : videoGeometry.getFrame(
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
      auto& occupancyMap = frame.getOccupancyMap();
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
    auto&        blockToPatch      = frame.getBlockToPatch();
    const size_t blockToPatchWidth = frame.getWidth() / params.occupancyResolution_;
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
                                   PCCFrameContext&                    frame,
                                   const GeneratePointCloudParameters& params,
                                   std::vector<uint32_t>&              partition,
                                   bool                                bDecoder ) {
  TRACE_CODEC( "generatePointCloud F = %zu start \n", frame.getIndex() );
#ifdef CODEC_TRACE
  TRACE_CODEC( "Generate point Cloud start \n" );
  TRACE_CODEC( "  occupancyResolution_            = %zu \n", params.occupancyResolution_ );
  TRACE_CODEC( "  occupancyPrecision_             = %zu \n", params.occupancyPrecision_ );
  TRACE_CODEC( "  flagGeometrySmoothing_          = %d  \n", params.flagGeometrySmoothing_ );
  if ( params.flagGeometrySmoothing_ ) {
    TRACE_CODEC( "  gridSmoothing_                  = %d  \n", params.gridSmoothing_ );
    if ( params.gridSmoothing_ ) {
      TRACE_CODEC( "  gridSize_                       = %zu \n", params.gridSize_ );
      TRACE_CODEC( "  thresholdSmoothing_             = %f \n", params.thresholdSmoothing_ );
    }
    TRACE_CODEC( "  pbfEnableFlag_                  = %d  \n", params.gridSmoothing_ );
    if ( params.pbfEnableFlag_ ) {
      TRACE_CODEC( "    pbfEnableFlag_                = %d \n", params.pbfEnableFlag_ );
      TRACE_CODEC( "    pbfPassesCount_               = %d \n", params.pbfPassesCount_ );
      TRACE_CODEC( "    pbfFilterSize_                = %d \n", params.pbfFilterSize_ );
      TRACE_CODEC( "    pbfLog2Threshold_             = %d \n", params.pbfLog2Threshold_ );
    }
    if ( !params.flagGeometrySmoothing_ && !params.pbfEnableFlag_ ) {
      TRACE_CODEC( "  neighborCountSmoothing_         = %zu \n", params.neighborCountSmoothing_ );
      TRACE_CODEC( "  radius2Smoothing_               = %f  \n", params.radius2Smoothing_ );
      TRACE_CODEC( "  radius2BoundaryDetection_       = %f  \n", params.radius2BoundaryDetection_ );
      TRACE_CODEC( "  thresholdSmoothing_             = %f \n", params.thresholdSmoothing_ );
    }
  }
  TRACE_CODEC( "  rawPointColorFormat_            = %d  \n", params.rawPointColorFormat_ );
  TRACE_CODEC( "  nbThread_                       = %zu \n", params.nbThread_ );
  TRACE_CODEC( "  multipleStreams_                = %d  \n", params.multipleStreams_ );
  TRACE_CODEC( "  absoluteD1_                     = %d  \n", params.absoluteD1_ );
  TRACE_CODEC( "  flagColorSmoothing_             = %d  \n", params.flagColorSmoothing_ );
  if ( params.flagColorSmoothing_ ) {
    TRACE_CODEC( "  cgridSize_                      = %zu \n", params.cgridSize_ );
    TRACE_CODEC( "  thresholdColorSmoothing_        = %f  \n", params.thresholdColorSmoothing_ );
    TRACE_CODEC( "  thresholdColorDifference_       = %f  \n", params.thresholdColorDifference_ );
    TRACE_CODEC( "  thresholdColorVariation_        = %f  \n", params.thresholdColorVariation_ );
  }
  TRACE_CODEC( "  enhancedOccupancyMapCode_       = %d  \n", params.enhancedOccupancyMapCode_ );
  if ( params.enhancedOccupancyMapCode_ ) {
    TRACE_CODEC( "    EOMFixBitCount_               = %d  \n", params.EOMFixBitCount_ );
  }
  TRACE_CODEC( "  removeDuplicatePoints_          = %d  \n", params.removeDuplicatePoints_ );
  TRACE_CODEC( "  mapCountMinus1_                 = %d  \n", params.mapCountMinus1_ );
  TRACE_CODEC( "  pointLocalReconstruction_       = %d  \n", params.pointLocalReconstruction_ );
  TRACE_CODEC( "  singleLayerPixelInterleaving    = %d  \n", params.singleMapPixelInterleaving_ );
  if ( params.pointLocalReconstruction_ || params.singleMapPixelInterleaving_ ) {
    TRACE_CODEC( "    surfaceThickness              = %zu \n", params.surfaceThickness_ );
  }
  TRACE_CODEC( "  useAdditionalPointsPatch_       = %d  \n", params.useAdditionalPointsPatch_ );
#endif
  auto&        videoGeometry         = context.getVideoGeometryMultiple()[0];
  auto&        videoGeometryMultiple = context.getVideoGeometryMultiple();
  auto&        videoOccupancyMap     = context.getVideoOccupancyMap();
  auto&        patches               = frame.getPatches();
  auto&        pointToPixel          = frame.getPointToPixel();
  auto&        blockToPatch          = frame.getBlockToPatch();
  const size_t blockToPatchWidth     = frame.getWidth() / params.occupancyResolution_;
  const size_t blockToPatchHeight    = frame.getHeight() / params.occupancyResolution_;
  const size_t patchCount            = patches.size();
  uint32_t     patchIndex            = 0;
  reconstruct.addColors();

  printf( "generatePointCloud pbfEnableFlag_ = %d \n", params.pbfEnableFlag_ );
  TRACE_CODEC( "generatePointCloud pbfEnableFlag_ = %d \n", params.pbfEnableFlag_ );
  if ( params.pbfEnableFlag_ ) {
    PatchBlockFiltering patchBlockFiltering;
    patchBlockFiltering.setPatches( &( frame.getPatches() ) );
    patchBlockFiltering.setBlockToPatch( &( frame.getBlockToPatch() ) );
    patchBlockFiltering.setOccupancyMapEncoder( &( frame.getOccupancyMap() ) );
    patchBlockFiltering.setOccupancyMapVideo(
        &( context.getVideoOccupancyMap().getFrame( frame.getIndex() ).getChannel( 0 ) ) );
    patchBlockFiltering.setGeometryVideo(
        &( videoGeometry.getFrame( frame.getIndex() * ( params.mapCountMinus1_ + 1 ) ).getChannel( 0 ) ) );
    patchBlockFiltering.patchBorderFiltering( frame.getWidth(), frame.getHeight(), params.occupancyResolution_,
                                              params.occupancyPrecision_,
                                              !params.enhancedOccupancyMapCode_ ? params.thresholdLossyOM_ : 0,
                                              params.pbfPassesCount_, params.pbfFilterSize_, params.pbfLog2Threshold_ );
    printf( "PBF done \n" );
    TRACE_CODEC( "PBF done \n" );
  }

  // point cloud occupancy map upscaling from video using nearest neighbor
  auto& occupancyMap = frame.getOccupancyMap();
  if ( !params.pbfEnableFlag_ ) {
    auto width  = frame.getWidth();
    auto height = frame.getHeight();
    occupancyMap.resize( width * height, 0 );
    for ( size_t v = 0; v < height; ++v ) {
      for ( size_t u = 0; u < width; ++u ) {
        occupancyMap[v * width + u] =
            videoOccupancyMap.getFrame( frame.getIndex() )
                .getValue( 0, u / params.occupancyPrecision_, v / params.occupancyPrecision_ );
      }
    }
  }
  if ( params.enableSizeQuantization_ ) {
    size_t quantizerSizeX = ( size_t( 1 ) << frame.getLog2PatchQuantizerSizeX() );
    size_t quantizerSizeY = ( size_t( 1 ) << frame.getLog2PatchQuantizerSizeY() );
    for ( size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex ) {
      auto&  patch             = patches[patchIndex];
      size_t nonZeroPixel      = 0;
      size_t patchSizeXInPixel = ( patch.getPatchSize2DXInPixel() / quantizerSizeX ) * quantizerSizeX;
      size_t patchSizeYInPixel = ( patch.getPatchSize2DYInPixel() / quantizerSizeY ) * quantizerSizeY;
      if ( frame.getLog2PatchQuantizerSizeX() == 0 ) { assert( patchSizeXInPixel == patch.getPatchSize2DXInPixel() ); }
      if ( frame.getLog2PatchQuantizerSizeY() == 0 ) { assert( patchSizeYInPixel == patch.getPatchSize2DYInPixel() ); }
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
                  occupancyMap[patch.patch2Canvas( u, v, frame.getWidth(), frame.getHeight(), x, y )] = 0;
                }
              }  // u1
            }    // v1
          }      // patchidx+1==block2patch
        }        // u0
      }          // v0
    }
  }
  partition.resize( 0 );
  pointToPixel.resize( 0 );
  reconstruct.clear();

  TRACE_CODEC( " Frame %zu in generatePointCloud \n", frame.getIndex() );
  TRACE_CODEC( " params.useAdditionalPointsPatch = %d \n", params.useAdditionalPointsPatch_ );
  TRACE_CODEC( " params.enhancedOccupancyMapCode   = %d \n", params.enhancedOccupancyMapCode_ );

  bool         useRawPointsSeparateVideo = frame.getUseRawPointsSeparateVideo();
  size_t       videoFrameIndex;
  const size_t mapCount = params.mapCountMinus1_ + 1;
  TRACE_CODEC( " mapCount                       = %d \n", mapCount );
  if ( params.multipleStreams_ ) {
    videoFrameIndex = frame.getIndex();
    if ( videoGeometryMultiple[0].getFrameCount() < ( videoFrameIndex + 1 ) ) { return; }
  } else {
    videoFrameIndex = frame.getIndex() * mapCount;
    if ( videoGeometry.getFrameCount() < ( videoFrameIndex + mapCount ) ) { return; }
  }
  TRACE_CODEC( " videoFrameIndex:frameIndex*mapCount  = %d \n", videoFrameIndex );
  const auto& frame0 = params.multipleStreams_ ? videoGeometryMultiple[0].getFrame( videoFrameIndex )
                                               : videoGeometry.getFrame( videoFrameIndex );
  const size_t          imageWidth  = frame0.getWidth();
  const size_t          imageHeight = frame0.getHeight();
  std::vector<uint32_t> BPflag;
  if ( !params.pbfEnableFlag_ ) { BPflag.resize( imageWidth * imageHeight, 0 ); }

  std::vector<std::vector<PCCPoint3D>> eomPointsPerPatch;
  eomPointsPerPatch.resize( patchCount );
  uint32_t index;
  for ( index = 0; index < patches.size(); index++ ) {
    patchIndex = ( bDecoder && context.getAtlasSequenceParameterSet( 0 ).getPatchPrecedenceOrderFlag() )
                     ? ( patchCount - index - 1 )
                     : index;
    const size_t patchIndexPlusOne = patchIndex + 1;
    auto&        patch             = patches[patchIndex];
    PCCColor3B   color( uint8_t( 0 ) );
    TRACE_CODEC(
        "P%2lu/%2lu: 2D=(%2lu,%2lu)*(%2lu,%2lu) 3D(%4zu,%4zu,%4zu)*(%4zu,%4zu) "
        "A=(%zu,%zu,%zu) Or=%zu P=%zu => %zu "
        "AxisOfAdditionalPlane = %zu \n",
        patchIndex, patchCount, patch.getU0(), patch.getV0(), patch.getSizeU0(), patch.getSizeV0(), patch.getU1(),
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

              bool   occupancy   = false;
              size_t canvasIndex = patch.patch2Canvas( u, v, imageWidth, imageHeight, x, y );
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
                PCCPoint3D point0 = patch.generatePoint( u, v, frame0.getValue( 0, x, y ) );
                size_t     pointIndex0;  // = reconstruct.addPoint(point0);
                if ( patch.getAxisOfAdditionalPlane() == 0 ) {
                  pointIndex0 = reconstruct.addPoint( point0 );
                } else {
                  PCCVector3D tmp;
                  PCCPatch::InverseRotatePosition45DegreeOnAxis( patch.getAxisOfAdditionalPlane(),
                                                                 params.geometryBitDepth3D_, point0, tmp );
                  pointIndex0 = reconstruct.addPoint( tmp );
                }
                reconstruct.setPointPatchIndex( pointIndex0, patchIndex );
                reconstruct.setColor( pointIndex0, color );
                if ( PCC_SAVE_POINT_TYPE == 1 ) { reconstruct.setType( pointIndex0, POINT_D0 ); }
                partition.push_back( uint32_t( patchIndex ) );
                pointToPixel.emplace_back( x, y, 0 );
                uint16_t    eomCode = 0;
                size_t      d1pos   = 0;
                const auto& frame0  = params.multipleStreams_ ? videoGeometryMultiple[0].getFrame( videoFrameIndex )
                                                             : videoGeometry.getFrame( videoFrameIndex );
                const auto& indx = patch.patch2Canvas( u, v, imageWidth, imageHeight, x, y );
                if ( params.mapCountMinus1_ > 0 ) {
                  const auto& frame1 = params.multipleStreams_ ? videoGeometryMultiple[1].getFrame( videoFrameIndex )
                                                               : videoGeometry.getFrame( videoFrameIndex + 1 );
                  int16_t diff = params.absoluteD1_ ? ( static_cast<int16_t>( frame1.getValue( 0, x, y ) ) -
                                                        static_cast<int16_t>( frame0.getValue( 0, x, y ) ) )
                                                    : static_cast<int16_t>( frame1.getValue( 0, x, y ) );
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
                        ( 1 << bits ) - occupancyMap[patch.patch2Canvas( u, v, imageWidth, imageHeight, x, y )];
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
                    reconstruct.setPointPatchIndex( pointIndex1, patchIndex );
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
                        reconstruct.setPointPatchIndex( pointIndex1, patchIndex );
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
                auto& mode = context.getPointLocalReconstructionMode( patch.getPointLocalReconstructionMode( u0, v0 ) );
                auto  createdPoints = generatePoints( params, frame,  // videoGeometry,
                                                     videoGeometryMultiple, videoFrameIndex, patchIndex, u, v, x, y,
                                                     mode.interpolate_, mode.filling_, mode.minD1_, mode.neighbor_ );
                if ( !createdPoints.empty() ) {
                  for ( size_t i = 0; i < createdPoints.size(); i++ ) {
                    if ( ( !params.removeDuplicatePoints_ ) ||
                         ( ( i == 0 ) || ( createdPoints[i] != createdPoints[0] ) ) ) {
                      size_t pointindex = 0;
                      if ( patch.getAxisOfAdditionalPlane() == 0 ) {
                        pointindex = reconstruct.addPoint( createdPoints[i] );
                        reconstruct.setPointPatchIndex( pointindex, patchIndex );
                      } else {
                        PCCVector3D tmp;
                        PCCPatch::InverseRotatePosition45DegreeOnAxis(
                            patch.getAxisOfAdditionalPlane(), params.geometryBitDepth3D_, createdPoints[i], tmp );
                        pointindex = reconstruct.addPoint( tmp );
                        reconstruct.setPointPatchIndex( pointindex, patchIndex );
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

  frame.setTotalNumberOfRegularPoints( reconstruct.getPointCount() );
  patchIndex                         = index;
  size_t       totalEOMPointsInFrame = 0;
  PCCPointSet3 eomSavedPoints;
  if ( params.enhancedOccupancyMapCode_ ) {
    const size_t blockSize       = params.occupancyResolution_ * params.occupancyResolution_;
    size_t       totalPatchCount = patchCount;
    size_t       numEOMPatches   = frame.getEomPatches().size();
    for ( int j = 0; j < numEOMPatches; j++ ) {
      auto&  eomPatch               = frame.getEomPatches( j );
      size_t numPatchesInEOMPatches = eomPatch.memberPatches.size();
      size_t u0Eom                  = useRawPointsSeparateVideo ? 0 : eomPatch.u0_ * params.occupancyResolution_;
      size_t v0Eom                  = useRawPointsSeparateVideo ? 0 : eomPatch.v0_ * params.occupancyResolution_;
      totalEOMPointsInFrame += eomPatch.eomCount_;
      size_t totalPointCount = 0;
      for ( size_t patchCount = 0; patchCount < numPatchesInEOMPatches; patchCount++ ) {
        size_t memberPatchIdx = ( bDecoder && context.getAtlasSequenceParameterSet( 0 ).getPatchPrecedenceOrderFlag() )
                                    ? ( totalPatchCount - eomPatch.memberPatches[patchCount] - 1 )
                                    : eomPatch.memberPatches[patchCount];
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
          reconstruct.setPointPatchIndex( pointIndex1, patchIndex );
          eomSavedPoints.addPoint( point1 );
          // reconstruct.setColor( pointIndex1, color );
          if ( PCC_SAVE_POINT_TYPE == 1 ) { reconstruct.setType( pointIndex1, POINT_EOM ); }
          partition.push_back( uint32_t( patchIndex ) );
          totalPointCount++;
          pointToPixel.emplace_back( uu, vv, 0 );
          occupancyMap[vv * imageWidth + uu] = 1;  // occupied
        }
      }
      TRACE_CODEC( "%d eomPatch :%zu,%zu\t %zu patches, %zu points\n", j, u0Eom, v0Eom, numPatchesInEOMPatches,
                   eomPatch.eomCount_ );
    }
    frame.setTotalNumberOfEOMPoints( totalEOMPointsInFrame );
  }
  TRACE_CODEC( " totalEOMPointsInFrame = %zu  \n", totalEOMPointsInFrame );
  TRACE_CODEC( " point = %zu  \n", reconstruct.getPointCount() );
  if ( params.useAdditionalPointsPatch_ ) {
    if ( useRawPointsSeparateVideo ) {
      PCCColor3B rawPointsColor( uint8_t( 0 ) );
      rawPointsColor[0] = 0;
      rawPointsColor[1] = 255;
      rawPointsColor[2] = 255;
      // Add point GPS from rawPointsPatch without inserting to pointToPixel
      size_t numberOfRawPointsPatches = frame.getNumberOfRawPointsPatches();
      for ( int j = 0; j < numberOfRawPointsPatches; j++ ) {
        auto&  rawPointsPatch  = frame.getRawPointsPatch( j );
        size_t sizeofRawPoints = rawPointsPatch.getNumberOfRawPoints();
        if ( 0 ) {  // yo- the geometry is always coded with YUV 420 format. So
                    // the COLOURFORMAT444 is impossible
                    // without separate auxiliary video
          for ( int i = 0; i < sizeofRawPoints; i++ ) {
            PCCVector3D point0;
            point0[0]               = rawPointsPatch.x_[i] + rawPointsPatch.u1_;
            point0[1]               = rawPointsPatch.y_[i] + rawPointsPatch.v1_;
            point0[2]               = rawPointsPatch.z_[i] + rawPointsPatch.d1_;
            const size_t pointIndex = reconstruct.addPoint( point0 );
            reconstruct.setPointPatchIndex( pointIndex, patchIndex );
            reconstruct.setColor( pointIndex, rawPointsColor );
            partition.push_back( uint32_t( patchIndex ) );
          }
        } else {  // else losslessGeo444_
          for ( int i = 0; i < sizeofRawPoints; i++ ) {
            PCCVector3D point0;
            point0[0]               = rawPointsPatch.x_[i] + rawPointsPatch.u1_;
            point0[1]               = rawPointsPatch.x_[i + sizeofRawPoints] + rawPointsPatch.v1_;
            point0[2]               = rawPointsPatch.x_[i + 2 * sizeofRawPoints] + rawPointsPatch.d1_;
            const size_t pointIndex = reconstruct.addPoint( point0 );
            reconstruct.setPointPatchIndex( pointIndex, patchIndex );
            reconstruct.setColor( pointIndex, rawPointsColor );
            partition.push_back( uint32_t( patchIndex ) );
          }
        }  // fi losslessGeo444_
      }
      // secure the size of rgb in raw points patch
      size_t numofEOMSaved  = params.enhancedOccupancyMapCode_ ? frame.getTotalNumberOfEOMPoints() : 0;
      size_t numofRawcolors = frame.getTotalNumberOfRawPoints();
      auto&  rawPointsPatch = frame.getRawPointsPatch( 0 );
      rawPointsPatch.setNumberOfRawPointsColors( numofRawcolors + numofEOMSaved );
      rawPointsPatch.resizeColor( numofRawcolors + numofEOMSaved );
    } else {  // else useRawPointsSeparateVideo
      TRACE_CODEC( " Add points from rawPointsPatch \n" );
      // Add points from rawPointsPatch
      size_t numberOfRawPointsPatches = frame.getNumberOfRawPointsPatches();
      for ( int i = 0; i < numberOfRawPointsPatches; i++ ) {
        auto& rawPointsPatch = frame.getRawPointsPatch( i );

        PCCColor3B rawPointsColor( uint8_t( 0 ) );
        rawPointsColor[0]     = 0;
        rawPointsColor[1]     = 255;
        rawPointsColor[2]     = 255;
        size_t numRawPoints   = rawPointsPatch.getNumberOfRawPoints();
        size_t ores           = rawPointsPatch.occupancyResolution_;
        rawPointsPatch.sizeV_ = rawPointsPatch.sizeV0_ * ores;
        rawPointsPatch.sizeU_ = rawPointsPatch.sizeU0_ * ores;
        if ( 0 ) {  // yo- => only 420 format for geometry code => use geometry
                    // CodecId()
          for ( size_t v0 = 0; v0 < rawPointsPatch.sizeV0_; ++v0 ) {
            for ( size_t u0 = 0; u0 < rawPointsPatch.sizeU0_; ++u0 ) {
              for ( size_t v1 = 0; v1 < rawPointsPatch.occupancyResolution_; ++v1 ) {
                const size_t v = v0 * rawPointsPatch.occupancyResolution_ + v1;
                for ( size_t u1 = 0; u1 < rawPointsPatch.occupancyResolution_; ++u1 ) {
                  const size_t u         = u0 * rawPointsPatch.occupancyResolution_ + u1;
                  const size_t x         = rawPointsPatch.u0_ * rawPointsPatch.occupancyResolution_ + u;
                  const size_t y         = rawPointsPatch.v0_ * rawPointsPatch.occupancyResolution_ + v;
                  const bool   occupancy = occupancyMap[y * imageWidth + x] != 0;
                  if ( !occupancy ) { continue; }
                  PCCPoint3D point0;
                  point0[0]               = double( frame0.getValue( 0, x, y ) ) + rawPointsPatch.u1_;
                  point0[1]               = double( frame0.getValue( 1, x, y ) ) + rawPointsPatch.v1_;
                  point0[2]               = double( frame0.getValue( 2, x, y ) ) + rawPointsPatch.d1_;
                  const size_t pointIndex = reconstruct.addPoint( point0 );
                  reconstruct.setPointPatchIndex( pointIndex, patchIndex );
                  reconstruct.setColor( pointIndex, rawPointsColor );
                  for ( size_t f = 0; f < mapCount; ++f ) {
                    partition.push_back( uint32_t( patchIndex ) );
                    pointToPixel.emplace_back( x, y, f );
                  }
                }
              }
            }
          }
        } else {
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
                rawPoints[numRawPointsAdded][0] = double( frame0.getValue( 0, x, y ) + rawPointsPatch.u1_ );
              } else if ( numRawPoints <= numRawPointsAdded && numRawPointsAdded < 2 * numRawPoints ) {
                rawPoints[numRawPointsAdded - numRawPoints][1] =
                    double( frame0.getValue( 0, x, y ) + rawPointsPatch.v1_ );
              } else if ( 2 * numRawPoints <= numRawPointsAdded && numRawPointsAdded < 3 * numRawPoints ) {
                rawPoints[numRawPointsAdded - 2 * numRawPoints][2] =
                    double( frame0.getValue( 0, x, y ) + rawPointsPatch.d1_ );
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
                reconstruct.setPointPatchIndex( pointIndex, patchIndex );
                reconstruct.setColor( pointIndex, rawPointsColor );
                partition.push_back( uint32_t( patchIndex ) );
                pointToPixel.emplace_back( x, y, 0 );
                counter++;
              }
            }
          }
        }
      }
    }  // fi :useRawPointsSeparateVideo
  }    // fi : useAdditionalPointsPatch

  if ( params.flagGeometrySmoothing_ && !params.pbfEnableFlag_ ) {
    TRACE_CODEC( " identify first boundary layer \n" );
    // identify first boundary layer
    if ( useRawPointsSeparateVideo ) {
      assert( ( reconstruct.getPointCount() - frame.getTotalNumberOfRawPoints() ) == pointToPixel.size() );
    } else {
      assert( ( reconstruct.getPointCount() + frame.getTotalNumberOfRawPoints() ) == pointToPixel.size() );
    }
    size_t pointCount = reconstruct.getPointCount() - frame.getTotalNumberOfRawPoints();
    for ( size_t i = 0; i < pointCount; ++i ) {
      const PCCVector3<size_t> location = pointToPixel[i];
      const size_t             x        = location[0];
      const size_t             y        = location[1];
      if ( occupancyMap[y * imageWidth + x] != 0 ) {
        identifyBoundaryPoints( occupancyMap, x, y, imageWidth, imageHeight, i, BPflag, reconstruct );
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

void PCCCodec::addGridColorCentroid( PCCPoint3D&                         point,
                                     PCCVector3D&                        color,
                                     uint32_t                            patchIdx,
                                     std::vector<uint16_t>&              colorGridCount,
                                     std::vector<PCCVector3<float>>&     colorCenter,
                                     std::vector<uint32_t>&              colorPartition,
                                     std::vector<bool>&                  colorDoSmooth,
                                     uint8_t                             gridSize,
                                     std::vector<std::vector<uint16_t>>& colorLum,
                                     const GeneratePointCloudParameters& params,
                                     int                                 cellId ) {
  if ( colorGridCount[cellId] == 0 ) {
    colorPartition[cellId] = patchIdx;
    colorCenter[cellId][0] = 0.;
    colorCenter[cellId][1] = 0.;
    colorCenter[cellId][2] = 0.;
    colorDoSmooth[cellId]  = false;
  } else if ( !colorDoSmooth[cellId] && colorPartition[cellId] != patchIdx ) {
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

void PCCCodec::createSpecificLayerReconstruct( const PCCPointSet3&                 reconstruct,
                                               const std::vector<uint32_t>&        partition,
                                               PCCFrameContext&                    frame,
                                               const GeneratePointCloudParameters& params,
                                               const size_t                        frameCount,
                                               PCCPointSet3&                       subReconstruct,
                                               std::vector<uint32_t>&              subPartition,
                                               std::vector<size_t>&                subReconstructIndex ) {
  subReconstruct.clear();
  subPartition.clear();
  subReconstructIndex.clear();
  auto&        pointToPixel = frame.getPointToPixel();
  const size_t pointCount   = reconstruct.getPointCount();
  if ( ( pointCount == 0U ) || !reconstruct.hasColors() ) { return; }
  for ( size_t i = 0; i < pointCount; ++i ) {
    const PCCVector3<size_t> location = pointToPixel[i];
    const size_t             f        = location[2];
    if ( f == frameCount ) {
      subReconstruct.addPoint( reconstruct[i] );
      subPartition.push_back( partition[i] );
      subReconstructIndex.push_back( i );
    }
  }
  subReconstruct.addColors();
}

void PCCCodec::createSubReconstruct( const PCCPointSet3&                 reconstruct,
                                     const std::vector<uint32_t>&        partition,
                                     PCCFrameContext&                    frame,
                                     const GeneratePointCloudParameters& params,
                                     const size_t                        frameCount,
                                     PCCPointSet3&                       subReconstruct,
                                     std::vector<uint32_t>&              subPartition,
                                     std::vector<size_t>&                subReconstructIndex ) {
  subReconstruct.clear();
  subPartition.clear();
  subReconstructIndex.clear();
  auto&        pointToPixel = frame.getPointToPixel();
  const size_t pointCount   = reconstruct.getPointCount();
  if ( ( pointCount == 0U ) || !reconstruct.hasColors() ) { return; }
  for ( size_t i = 0; i < pointCount; ++i ) {
    const PCCVector3<size_t> location = pointToPixel[i];
    const size_t             f        = location[2];
    if ( f < frameCount ) {
      subReconstruct.addPoint( reconstruct[i] );
      subReconstruct.setType( frameCount, POINT_UNSET );
      subPartition.push_back( partition[i] );
      subReconstructIndex.push_back( i );
    }
  }
  subReconstruct.addColors();
}

void PCCCodec::updateReconstruct( PCCPointSet3&              reconstruct,
                                  const PCCPointSet3&        subReconstruct,
                                  const std::vector<size_t>& subReconstructIndex ) {
  if ( subReconstruct.getPointCount() > 0 ) {
    for ( size_t i = 0; i < subReconstruct.getPointCount(); ++i ) {
      reconstruct[subReconstructIndex[i]] = subReconstruct[i];
    }
  }
}

bool PCCCodec::colorPointCloud( PCCPointSet3&                       reconstruct,
                                PCCContext&                         context,
                                PCCFrameContext&                    frame,
                                const std::vector<bool>&            absoluteT1List,
                                const size_t                        multipleStreams,
                                const uint8_t                       attributeCount,
                                const GeneratePointCloudParameters& params ) {
  TRACE_CODEC( "colorPointCloud start \n" );
#ifdef CODEC_TRACE
  printChecksum( reconstruct, "colorPointCloud in" );
#endif
  if(reconstruct.getPointCount()==0) return false;
  auto&        sps                           = context.getVps();
  auto&        videoTexture                  = context.getVideoTextureMultiple()[0];
  auto&        videoTextureFrame1            = context.getVideoTextureMultiple()[1];
  const size_t mapCount                      = params.mapCountMinus1_ + 1;
  size_t       numberOfRawPointsAndEOMColors = 0;
  size_t       numOfRawGeos                  = 0;
  size_t       numberOfEOMPoints             = 0;
  if ( attributeCount == 0 ) {
    for ( auto& color : reconstruct.getColors() ) {
      for ( size_t c = 0; c < 3; ++c ) { color[c] = static_cast<uint8_t>( 127 ); }
    }
  } else {
    auto& pointToPixel = frame.getPointToPixel();
    auto& color16bit   = reconstruct.getColors16bit();
#ifdef TRACE_CODEC
    auto& color8bit = reconstruct.getColors();
    std::fill( color8bit.begin(), color8bit.end(), PCCColor3B() );
    std::fill( color16bit.begin(), color16bit.end(), PCCColor16bit() );
#endif
    bool  useRawPointsSeparateVideo = frame.getUseRawPointsSeparateVideo();
    auto& asps                      = context.getAtlasSequenceParameterSet( 0 );  // TODO: get correct value
    bool  lossyRawPointsPatch       = !asps.getRawPatchEnabledFlag() && frame.getRawPatchEnabledFlag();
    numOfRawGeos                    = frame.getTotalNumberOfRawPoints();
    numberOfEOMPoints               = frame.getTotalNumberOfEOMPoints();
    numberOfRawPointsAndEOMColors   = numOfRawGeos + numberOfEOMPoints;
    size_t pointCount               = reconstruct.getPointCount();
    if ( ( asps.getRawPatchEnabledFlag() || lossyRawPointsPatch ) && useRawPointsSeparateVideo ) {
      numOfRawGeos                  = frame.getTotalNumberOfRawPoints();
      numberOfEOMPoints             = frame.getTotalNumberOfEOMPoints();
      numberOfRawPointsAndEOMColors = numOfRawGeos + numberOfEOMPoints;
      TRACE_CODEC( " numOfRawGeos            = %d \n", numOfRawGeos );
      TRACE_CODEC( " numberOfEOMPoints       = %d \n", numberOfEOMPoints );
      TRACE_CODEC( " numberOfRawPointsAndEOMColors = %d \n", numberOfRawPointsAndEOMColors );
      if ( useRawPointsSeparateVideo && ( asps.getRawPatchEnabledFlag() || lossyRawPointsPatch ) ) {
        pointCount = reconstruct.getPointCount() - numOfRawGeos - numberOfEOMPoints;
        assert( numberOfRawPointsAndEOMColors == ( numberOfEOMPoints + numOfRawGeos ) );
        TRACE_CODEC( "    => pointCount         = %d \n", pointCount );
      }
    }
    TRACE_CODEC( "  useRawPointsSeparateVideo = %d \n", useRawPointsSeparateVideo );
    TRACE_CODEC( "  sps.getRawPatchEnabledFlag()  = %d \n", asps.getRawPatchEnabledFlag() );
    TRACE_CODEC( "  plt.getProfileCodecGroupIdc() = %d \n",
                 context.getVps().getProfileTierLevel().getProfileCodecGroupIdc() );
    TRACE_CODEC( "  lossyRawPointsPatch       = %d \n", lossyRawPointsPatch );
    if ( params.enhancedOccupancyMapCode_ ) {
      TRACE_CODEC( "  numberOfRawPointsAndEOMColors      = %zu \n", numberOfRawPointsAndEOMColors );
      TRACE_CODEC( "  numberOfEOMPoints            = %zu \n", numberOfEOMPoints );
    }
    TRACE_CODEC( "  numOfRawGeos                 = %zu \n", numOfRawGeos );
    TRACE_CODEC( "  pointCount                   = %zu \n", pointCount );
    TRACE_CODEC( "  pointToPixel size            = %zu \n", pointToPixel.size() );
    TRACE_CODEC( "  pointLocalReconstruction     = %d \n", params.pointLocalReconstruction_ );
    TRACE_CODEC( "  singleLayerPixelInterleaving = %d \n", params.singleMapPixelInterleaving_ );
    TRACE_CODEC( "  enhancedOccupancyMapCode       = %d \n", params.enhancedOccupancyMapCode_ );
    TRACE_CODEC( "  multipleStreams              = %d \n", multipleStreams );

    if ( ( pointCount == 0U ) || !reconstruct.hasColors() ) { return false; }
    PCCPointSet3        target;
    PCCPointSet3        source;
    std::vector<size_t> targetIndex;
    targetIndex.resize( 0 );
    target.clear();
    source.clear();
    target.addColors16bit();
    source.addColors16bit();
    const size_t pcFrameIndex = frame.getIndex() * mapCount;
    for ( size_t i = 0; i < pointCount; ++i ) {
      const PCCVector3<size_t> location = pointToPixel[i];
      const size_t             x        = location[0];
      const size_t             y        = location[1];
      const size_t             f        = location[2];
      if ( params.singleMapPixelInterleaving_ ) {
        if ( ( static_cast<int>( f == 0 && ( x + y ) % 2 == 0 ) | static_cast<int>( f == 1 && ( x + y ) % 2 == 1 ) ) !=
             0 ) {
          const auto& image = videoTexture.getFrame( pcFrameIndex );
          for ( size_t c = 0; c < 3; ++c ) { color16bit[i][c] = image.getValue( c, x, y ); }
          int index = source.addPoint( reconstruct[i] );
          source.setColor16bit( index, color16bit[i] );
        } else {
          target.addPoint( reconstruct[i] );
          targetIndex.push_back( i );
        }
      } else if ( multipleStreams != 0U ) {
        if ( f == 0 ) {
          const auto& image = videoTexture.getFrame( frame.getIndex() );
          for ( size_t c = 0; c < 3; ++c ) { color16bit[i][c] = image.getValue( c, x, y ); }
          size_t index = source.addPoint( reconstruct[i] );
          source.setColor16bit( index, color16bit[i] );
        } else {
          const auto& image0   = videoTexture.getFrame( frame.getIndex() );
          const auto& image1   = videoTextureFrame1.getFrame( frame.getIndex() );
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
          const auto& frame = videoTexture.getFrame( pcFrameIndex + f );
          for ( size_t c = 0; c < 3; ++c ) { color16bit[i][c] = frame.getValue( c, x, y ); }
          int index = source.addPoint( reconstruct[i] );
          source.setColor16bit( index, color16bit[i] );
#ifdef TRACE_CODEC
          if ( index < 100 ) {
            TRACE_CODEC(
                " %4zu %4zu %4zu: f = %2zu  xy = %4zu %4zu => c8 = %3zu %3zu %3zu c16 = %3zu %3zu %3zu from %3zu %3zu "
                "%3zu (i=%6zu index = %6zu)\n",
                reconstruct[i][0], reconstruct[i][1], reconstruct[i][2], f, x, y, color8bit[i][0], color8bit[i][1],
                color8bit[i][2], color16bit[i][0], color16bit[i][1], color16bit[i][2], frame.getValue( 0, x, y ),
                frame.getValue( 1, x, y ), frame.getValue( 2, x, y ), i, index );
          }
#endif
        } else {
          target.addPoint( reconstruct[i] );
          targetIndex.push_back( i );
        }
      }
    }
    if ( target.getPointCount() > 0 ) {
      source.transferColorWeight( target );
      for ( size_t i = 0; i < target.getPointCount(); ++i ) {
        reconstruct.setColor16bit( targetIndex[i], target.getColor16bit( i ) );
      }
    }
    if ( ( asps.getRawPatchEnabledFlag() || lossyRawPointsPatch ) && useRawPointsSeparateVideo ) {
      std::vector<PCCColor3B>& mpsTextures = frame.getRawPointsTextures();
      std::vector<PCCColor3B>& eomTextures = frame.getEOMTextures();
      for ( size_t i = 0; i < numberOfEOMPoints; ++i ) {
        color16bit[pointCount + i].r() = (uint16_t)eomTextures[i].r();
        color16bit[pointCount + i].g() = (uint16_t)eomTextures[i].g();
        color16bit[pointCount + i].b() = (uint16_t)eomTextures[i].b();
      }
      for ( size_t i = 0; i < numOfRawGeos; ++i ) {
        color16bit[pointCount + numberOfEOMPoints + i].r() = (uint16_t)mpsTextures[i].r();
        color16bit[pointCount + numberOfEOMPoints + i].g() = (uint16_t)mpsTextures[i].g();
        color16bit[pointCount + numberOfEOMPoints + i].b() = (uint16_t)mpsTextures[i].b();
      }
    }
  }  // noAtt
#ifdef CODEC_TRACE
  printChecksum( reconstruct, "colorPointCloud out" );
  TRACE_CODEC( "colorPointCloud done \n" );
#endif
  return true;
}

void PCCCodec::generateRawPointsGeometryfromVideo( PCCContext& context ) {
  TRACE_CODEC( " generateRawPointsGeometryfromVideo start \n" );
  const size_t gofSize                = context.size();
  auto&        videoRawPointsGeometry = context.getVideoRawPointsGeometry();
  videoRawPointsGeometry.resize( gofSize );
  for ( auto& framecontext : context.getFrames() ) {
    const size_t frameIndex = framecontext.getIndex();
    generateRawPointsGeometryfromVideo( context, framecontext, frameIndex );
    size_t totalNumRawPoints = 0;
    for ( size_t i = 0; i < framecontext.getNumberOfRawPointsPatches(); i++ ) {
      totalNumRawPoints += framecontext.getRawPointsPatch( i ).size();
    }
    std::cout << "generate raw Points Video (Geometry) frame  " << frameIndex
              << "from Video : # of raw Patches : " << framecontext.getNumberOfRawPointsPatches()
              << " total # of raw Geometry : " << totalNumRawPoints << std::endl;
  }
  std::cout << "Raw Points Geometry from Video [done]" << std::endl;
  TRACE_CODEC( " generateRawPointsGeometryfromVideo done \n" );
}

void PCCCodec::generateRawPointsGeometryfromVideo( PCCContext& context, PCCFrameContext& frame, size_t frameIndex ) {
  auto&  videoRawPointsGeometry   = context.getVideoRawPointsGeometry();
  auto&  image                    = videoRawPointsGeometry.getFrame( frameIndex );
  size_t numberOfRawPointsPatches = frame.getNumberOfRawPointsPatches();
  bool   isAuxiliarygeometrys444  = false;  // yo- use geo auxiliary codecID

  TRACE_CODEC( "generateRawPointsGeometryfromVideo \n" );
  for ( int i = 0; i < numberOfRawPointsPatches; i++ ) {
    auto&        rawPointsPatch = frame.getRawPointsPatch( i );
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
          const size_t y = ( v0 + v );
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

void PCCCodec::generateRawPointsTexturefromVideo( PCCContext& context ) {
  const size_t gofSize               = context.size();
  auto&        videoRawPointsTexture = context.getVideoRawPointsTexture();
  videoRawPointsTexture.resize( gofSize );
  TRACE_CODEC( "generateRawPointsTexturefromVideo \n" );
  for ( auto& frame : context.getFrames() ) {
    generateRawPointsTexturefromVideo( context, frame, frame.getIndex() );
    std::cout << "generate raw points (Texture) : frame " << frame.getIndex()
              << ", # of raw points Texture : " << frame.getRawPointsPatch( 0 ).size() << std::endl;
  }
  std::cout << "RawPoints Texture [done]" << std::endl;
  TRACE_CODEC( "generateRawPointsTexturefromVideo done \n" );
}

void PCCCodec::generateRawPointsTexturefromVideo( PCCContext& context, PCCFrameContext& frame, size_t frameIndex ) {
  auto&  videoRawPointsTexture = context.getVideoRawPointsTexture();
  auto&  image                 = videoRawPointsTexture.getFrame( frameIndex );
  size_t width                 = image.getWidth();
  size_t height                = image.getHeight();
  context.setRawAttWidth( width );
  context.setRawAttHeight( height );
  size_t                   numberOfEOMPoints = frame.getTotalNumberOfEOMPoints();
  size_t                   numOfRawGeos      = frame.getTotalNumberOfRawPoints();
  std::vector<PCCColor3B>& mpsTextures       = frame.getRawPointsTextures();
  std::vector<PCCColor3B>& eomTextures       = frame.getEOMTextures();
  mpsTextures.resize( numOfRawGeos );
  eomTextures.resize( numberOfEOMPoints );
  size_t heightMP  = numOfRawGeos / width + 1;
  size_t heightby8 = heightMP / 8;
  if ( heightby8 * 8 != heightMP ) { heightMP = ( heightby8 + 1 ) * 8; }
  size_t mpsV0;
  size_t maxRawPointsV0           = 0;
  size_t numberOfRawPointsPatches = frame.getNumberOfRawPointsPatches();
  for ( int i = 0; i < numberOfRawPointsPatches; i++ ) {
    auto& rawPointsPatch = frame.getRawPointsPatch( i );
    mpsV0                = rawPointsPatch.v0_ * rawPointsPatch.occupancyResolution_ + rawPointsPatch.sizeV_;
    if ( mpsV0 > maxRawPointsV0 ) { maxRawPointsV0 = mpsV0; }
  }
  heightMP            = maxRawPointsV0;
  int framePointIndex = 0;
  for ( int i = 0; i < numberOfRawPointsPatches; i++ ) {
    int          pointIndex     = 0;
    auto&        rawPointsPatch = frame.getRawPointsPatch( i );
    size_t       numRawPoints   = rawPointsPatch.getNumberOfRawPoints();
    const size_t v0             = rawPointsPatch.v0_ * rawPointsPatch.occupancyResolution_;
    const size_t u0             = rawPointsPatch.u0_ * rawPointsPatch.occupancyResolution_;
    for ( size_t v = 0; v < rawPointsPatch.sizeV_; ++v ) {
      for ( size_t u = 0; u < rawPointsPatch.sizeU_; ++u ) {
        if ( pointIndex < numRawPoints ) {
          const size_t x = ( u0 + u );
          const size_t y = ( v0 + v );
          assert( x < width && y < height );
          mpsTextures[framePointIndex].r() = image.getValue( 0, x, y );
          mpsTextures[framePointIndex].g() = image.getValue( 1, x, y );
          mpsTextures[framePointIndex].b() = image.getValue( 2, x, y );
          framePointIndex++;
          pointIndex++;
        }
      }
    }
  }
  size_t nPixelInCurrentBlockCount = 0;
  for ( size_t i = 0; i < numberOfEOMPoints; i++ ) {
    assert( ( i + numOfRawGeos ) / width < height );
    size_t xx;
    size_t yy;
    size_t nBlock = i / 256;
    size_t uBlock = nBlock % ( width / 16 );
    size_t vBlock = nBlock / ( width / 16 );
    xx            = uBlock * 16 + ( nPixelInCurrentBlockCount % 16 );
    yy            = vBlock * 16 + ( nPixelInCurrentBlockCount / 16 ) + heightMP;
    ++nPixelInCurrentBlockCount;
    if ( nPixelInCurrentBlockCount >= 256 ) { nPixelInCurrentBlockCount = 0; }
    eomTextures[i].r() = image.getValue( 0, xx, yy );
    eomTextures[i].g() = image.getValue( 1, xx, yy );
    eomTextures[i].b() = image.getValue( 2, xx, yy );
  }
}

void PCCCodec::generateOccupancyMap( PCCFrameContext&            frame,
                                     const PCCImageOccupancyMap& videoFrame,
                                     const size_t                occupancyPrecision,
                                     const size_t                thresholdLossyOM,
                                     const bool                  enhancedOccupancyMapForDepthFlag ) {
  auto  width        = frame.getWidth();
  auto  height       = frame.getHeight();
  auto& occupancyMap = frame.getOccupancyMap();
  occupancyMap.resize( width * height, 0 );
  for ( size_t v = 0; v < height; ++v ) {
    for ( size_t u = 0; u < width; ++u ) {
      uint8_t pixel = videoFrame.getValue( 0, u / occupancyPrecision, v / occupancyPrecision );
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
}

void PCCCodec::generateBlockToPatchFromBoundaryBox( PCCContext& context, const size_t occupancyResolution ) {
  for ( auto& frame : context.getFrames() ) {
    generateBlockToPatchFromBoundaryBox( context, frame, occupancyResolution );
  }
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
  size_t sizeFrames = context.getFrames().size();
  for ( int i = 0; i < sizeFrames; i++ ) {
    PCCFrameContext&      frame          = context.getFrames()[i];
    PCCImageOccupancyMap& occupancyImage = context.getVideoOccupancyMap().getFrame( i );
    generateBlockToPatchFromOccupancyMapVideo( context, frame, occupancyImage, occupancyResolution,
                                               occupancyPrecision );
  }
}

void PCCCodec::generateBlockToPatchFromOccupancyMapVideo( PCCContext&           context,
                                                          PCCFrameContext&      frame,
                                                          PCCImageOccupancyMap& occupancyMapImage,
                                                          const size_t          occupancyResolution,
                                                          const size_t          occupancyPrecision ) {
  auto&        patches            = frame.getPatches();
  const size_t patchCount         = patches.size();
  const size_t blockToPatchWidth  = frame.getWidth() / occupancyResolution;
  const size_t blockToPatchHeight = frame.getHeight() / occupancyResolution;
  const size_t blockCount         = blockToPatchWidth * blockToPatchHeight;
  auto&        blockToPatch       = frame.getBlockToPatch();
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
            patch.patch2Canvas( u, v, frame.getWidth(), frame.getHeight(), x, y );
            nonZeroPixel += static_cast<unsigned long long>(
                occupancyMapImage.getValue( 0, x / occupancyPrecision, y / occupancyPrecision ) != 0 );
          }  // u1
        }    // v1
        if ( nonZeroPixel > 0 ) { blockToPatch[blockIndex] = patchIndex + 1; }
      }  // u0
    }    // v0
  }      // patch
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
