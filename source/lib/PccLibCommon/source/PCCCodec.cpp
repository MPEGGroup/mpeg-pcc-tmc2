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
PCCCodec::~PCCCodec() {}

void PCCCodec::addGridCentroid( PCCPoint3D&               point,
                                int                       patchIdx,
                                std::vector<int>&         cnt,
                                std::vector<PCCVector3D>& center_grid,
                                std::vector<int>&         gpartition,
                                std::vector<bool>&        doSmooth,
                                int                       gridSize,
                                int                       gridWidth ) {
  const int w   = gridWidth;
  int       x   = (int)point.x() / gridSize;
  int       y   = (int)point.y() / gridSize;
  int       z   = (int)point.z() / gridSize;
  int       idx = x + y * w + z * w * w;

  if ( cnt[idx] == 0 ) {
    gpartition[idx]  = patchIdx;
    center_grid[idx] = PCCVector3D( 0, 0, 0 );
    doSmooth[idx]    = false;
  } else if ( !doSmooth[idx] && gpartition[idx] != patchIdx ) {
    doSmooth[idx] = true;
  }

  center_grid[idx] += point;
  cnt[idx]++;
}

void PCCCodec::generatePointCloud( PCCGroupOfFrames&                  reconstructs,
                                   PCCContext&                        context,
                                   const GeneratePointCloudParameters params ) {
  TRACE_CODEC( "Generate point Cloud start \n" );
  TRACE_CODEC( "  occupancyResolution_            = %lu \n", params.occupancyResolution_ );
  TRACE_CODEC( "  occupancyPrecision_             = %lu \n", params.occupancyPrecision_ );
  TRACE_CODEC( "  flagGeometrySmoothing_          = %d  \n", params.flagGeometrySmoothing_ );
  if ( params.flagGeometrySmoothing_ ) {
    TRACE_CODEC( "  gridSmoothing_                  = %d  \n", params.gridSmoothing_ );
    if ( params.gridSmoothing_ ) {
      TRACE_CODEC( "  gridSize_                       = %lu \n", params.gridSize_ );
    } else {
      TRACE_CODEC( "  neighborCountSmoothing_         = %lu \n", params.neighborCountSmoothing_ );
      TRACE_CODEC( "  radius2Smoothing_               = %f  \n", params.radius2Smoothing_ );
      TRACE_CODEC( "  radius2BoundaryDetection_       = %f  \n", params.radius2BoundaryDetection_ );
      TRACE_CODEC( "  thresholdSmoothing_             = %f  \n", params.thresholdSmoothing_ );
    }
  }
  TRACE_CODEC( "  pcmPointColorFormat_                = %d  \n", params.pcmPointColorFormat_ );
  TRACE_CODEC( "  nbThread_                       = %lu \n", params.nbThread_ );
  TRACE_CODEC( "  absoluteD1_                     = %d  \n", params.absoluteD1_ );
  TRACE_CODEC( "  surfaceThickness                = %lu \n", params.surfaceThickness_ );
  TRACE_CODEC( "  flagColorSmoothing_             = %d  \n", params.flagColorSmoothing_ );
  if ( params.flagColorSmoothing_ ) {
    TRACE_CODEC( "  gridColorSmoothing_           = %d  \n", params.gridColorSmoothing_ );
    if ( params.gridColorSmoothing_ ) {
      TRACE_CODEC( "  cgridSize_                      = %lu \n", params.cgridSize_ );
      TRACE_CODEC( "  thresholdColorSmoothing_        = %f  \n", params.thresholdColorSmoothing_ );
      TRACE_CODEC( "  thresholdColorDifference_       = %f  \n", params.thresholdColorDifference_ );
      TRACE_CODEC( "  thresholdColorVariation_        = %f  \n", params.thresholdColorVariation_ );
      TRACE_CODEC( "  thresholdLocalEntropy_          = %f  \n", params.thresholdLocalEntropy_ );
    } else {
      TRACE_CODEC( "  thresholdColorSmoothing_        = %f  \n", params.thresholdColorSmoothing_ );
      TRACE_CODEC( "  thresholdLocalEntropy_          = %f  \n", params.thresholdLocalEntropy_ );
      TRACE_CODEC( "  radius2ColorSmoothing_          = %f  \n", params.radius2ColorSmoothing_ );
      TRACE_CODEC( "  neighborCountColorSmoothing_    = %lu \n", params.neighborCountColorSmoothing_ );
    }
  }
  TRACE_CODEC( "  enhancedDeltaDepthCode_         = %d  \n", params.enhancedDeltaDepthCode_ );
  TRACE_CODEC( "  EOMFixBitCount_                 = %d  \n", params.EOMFixBitCount_ );
  TRACE_CODEC( "  EOMTexturePatch_                = %d  \n", params.EOMTexturePatch_ );
  TRACE_CODEC( "  removeDuplicatePoints_          = %d  \n", params.removeDuplicatePoints_ );
  TRACE_CODEC( "  pointLocalReconstruction_                       = %d  \n", params.pointLocalReconstruction_ );
  TRACE_CODEC( "  layerCountMinus1_               = %d  \n", params.layerCountMinus1_ );
  TRACE_CODEC( "  singleLayerPixelInterleaving    = %d  \n", params.singleLayerPixelInterleaving_ );
  TRACE_CODEC( "  useAdditionalPointsPatch_       = %d  \n", params.useAdditionalPointsPatch_ );

  auto& frames          = context.getFrames();
  auto& videoGeometry   = context.getVideoGeometry();
  auto& videoGeometryD1 = context.getVideoGeometryD1();
	auto &videoOccupancyMap = context.getVideoOccupancyMap();

#ifdef ENABLE_PAPI_PROFILING
  PAPI_PROFILING_INITIALIZE;
#endif
  for ( size_t i = 0; i < frames.size(); i++ ) {
    TRACE_CODEC( " Frame %lu / %lu \n", i, frames.size() );
    std::vector<uint32_t> partition;
    generatePointCloud( reconstructs[i], context, frames[i], videoGeometry, videoGeometryD1, videoOccupancyMap, params, partition );

    TRACE_CODEC( " generatePointCloud create %lu points \n", reconstructs[i].getPointCount() );
    if ( params.flagGeometrySmoothing_ ) {
      if ( params.gridSmoothing_ ) {
        // reset for each GOF
        PCCInt16Box3D boundingBox;
        boundingBox.min_ = boundingBox.max_ = reconstructs[i][0];
        for ( int j = 0; j < reconstructs[i].getPointCount(); j++ ) {
          const PCCPoint3D point = reconstructs[i][j];
          for ( size_t k = 0; k < 3; ++k ) {
            if ( point[k] < boundingBox.min_[k] ) { boundingBox.min_[k] = floor( point[k] ); }
            if ( point[k] > boundingBox.max_[k] ) { boundingBox.max_[k] = ceil( point[k] ); }
          }
        }

        int maxSize = ( std::max )( ( std::max )( boundingBox.max_.x(), boundingBox.max_.y() ), boundingBox.max_.z() );

        const int w = ( maxSize + (int)params.gridSize_ - 1 ) / ( (int)params.gridSize_ );
        center_grid_.resize( w * w * w );
        gcnt_.resize( w * w * w );
        gpartition_.resize( w * w * w );
        doSmooth_.resize( w * w * w );

        int size = (int)gcnt_.size();
        gcnt_.resize( 0 );
        gcnt_.resize( size, 0 );
        for ( int j = 0; j < reconstructs[i].getPointCount(); j++ ) {
          addGridCentroid( reconstructs[i][j], partition[j] + 1, gcnt_, center_grid_, gpartition_, doSmooth_,
                           (int)params.gridSize_, w );
        }
        for ( int i = 0; i < gcnt_.size(); i++ ) {
          if ( gcnt_[i] ) { center_grid_[i] /= gcnt_[i]; }
        }
        smoothPointCloudGrid( reconstructs[i], partition, params, w );
      } else {
        smoothPointCloud( reconstructs[i], partition, params );
      }
    }
  }
#ifdef ENABLE_PAPI_PROFILING
  PAPI_PROFILING_RESULTS;
#endif
  TRACE_CODEC( "Generate point Cloud done \n" );
}

bool PCCCodec::colorPointCloud( PCCGroupOfFrames&                  reconstructs,
                                PCCContext&                        context,
                                const uint8_t                      attributeCount,
                                const PCCColorTransform            colorTransform,
                                const GeneratePointCloudParameters params ) {
  TRACE_CODEC( "Color point Cloud start \n" );
  auto&     video  = context.getVideoTexture();
  auto&     frames = context.getFrames();
  const int cgrid  = params.occupancyPrecision_;
  // const int w      = SMOOTH_POINT_CLOUD_MAX_SIZE / cgrid;
  // const int w = pow( 2, params.geometry3dCoordinatesBitdepth_ ) / cgrid;
  const int w = pow( 2, params.geometryBitDepth3D_ ) / cgrid;
  if ( params.flagColorSmoothing_ && params.gridColorSmoothing_ ) {
    CS_color_center_grid_.resize( w * w * w );
    CS_color_gcnt_.resize( w * w * w );
    CS_color_gpartition_.resize( w * w * w );
    CS_color_doSmooth_.resize( w * w * w );
  }
  for ( size_t i = 0; i < frames.size(); i++ ) {
    colorPointCloud( reconstructs[i], frames[i], video, attributeCount, params );

    if ( params.flagColorSmoothing_ ) {
      if ( params.gridColorSmoothing_ ) {  // Fast Color Smoothing
        std::fill( CS_color_center_grid_.begin(), CS_color_center_grid_.end(), 0 );
        std::fill( CS_color_gcnt_.begin(), CS_color_gcnt_.end(), 0 );
        std::fill( CS_color_gpartition_.begin(), CS_color_gpartition_.end(), 0 );
        std::fill( CS_color_doSmooth_.begin(), CS_color_doSmooth_.end(), 0 );

        CS_gLum_.clear();
        CS_gLum_.resize( w * w * w );

        for ( int k = 0; k < reconstructs[i].getPointCount(); k++ ) {
          PCCColor3B  color = reconstructs[i].getColor( k );
          PCCVector3D clr;
          for ( size_t c = 0; c < 3; ++c ) { clr[c] = double( color[c] ); }
          const size_t patchIndexPlusOne = reconstructs[i].getPointPatchIndex( k ) + 1;

          addGridColorCentroid( reconstructs[i][k], clr, patchIndexPlusOne, CS_color_gcnt_, CS_color_center_grid_,
                                CS_color_gpartition_, CS_color_doSmooth_, cgrid, CS_gLum_, params );
        }
        smoothPointCloudColorLC( reconstructs[i], params );
      } else {
        smoothPointCloudColor( reconstructs[i], params );
      }
    }
    if ( colorTransform == COLOR_TRANSFORM_RGB_TO_YCBCR ) { reconstructs[i].convertYUVToRGB(); }
  }

  CS_color_center_grid_.resize(0);
  CS_color_center_grid_.shrink_to_fit();
  CS_color_gcnt_.resize(0);
  CS_color_gcnt_.shrink_to_fit();
  CS_color_gpartition_.resize(0);
  CS_color_gpartition_.shrink_to_fit();
  CS_color_doSmooth_.resize(0);
  CS_color_doSmooth_.shrink_to_fit();
  CS_gLum_.resize(0);
  CS_gLum_.shrink_to_fit();
  center_grid_.resize(0);
  center_grid_.shrink_to_fit();
  gcnt_.resize(0);
  gcnt_.shrink_to_fit();
  gpartition_.resize(0);
  gpartition_.shrink_to_fit();
  doSmooth_.resize(0);
  doSmooth_.shrink_to_fit();

  TRACE_CODEC( "Color point Cloud done \n" );
  return true;
}

int PCCCodec::getDeltaNeighbors( const PCCImageGeometry& frame,
                                 const PCCPatch&         patch,
                                 const int               xOrg,
                                 const int               yOrg,
                                 const int               neighboring,
                                 const int               threshold,
                                 const bool              projectionMode)
{
  int    deltaMax = 0;
  double dOrg     = patch.generateNormalCoordinate( frame.getValue( 0, xOrg, yOrg ));
  for ( int x = ( std::max )( 0, xOrg - neighboring ); x <= ( std::min )( xOrg + neighboring, (int)frame.getWidth() );
        x += 1 ) {
    for ( int y = ( std::max )( 0, yOrg - neighboring );
          y <= ( std::min )( yOrg + neighboring, (int)frame.getHeight() ); y += 1 ) {
      double dLoc  = patch.generateNormalCoordinate( frame.getValue( 0, x, y ));
      int    delta = (int)( dLoc - dOrg );
      if ( patch.getProjectionMode() == 0 ) {
        if ( delta <= threshold && delta > deltaMax ) { deltaMax = delta; }
      } else {
        if ( delta >= -(int)threshold && delta < deltaMax ) { deltaMax = delta; }
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
  if ( occupancyMap[y * imageWidth + x] && reconstruct.getBoundaryPointType( pointindex ) != 1 ) {
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

std::vector<PCCPoint3D> PCCCodec::generatePoints( const GeneratePointCloudParameters& params,
                                                  PCCFrameContext&                    frame,
                                                  const PCCVideoGeometry&             video,
                                                  const PCCVideoGeometry&             videoD1,
                                                  const size_t                        shift,
                                                  const size_t                        patchIndex,
                                                  const size_t                        u,
                                                  const size_t                        v,
                                                  const size_t                        x,
                                                  const size_t                        y,
                                                  const bool                          interpolate,
                                                  const bool                          filling,
                                                  const size_t                        minD1,
                                                  const size_t                        neighbor)
{
  const auto&             patch        = frame.getPatch( patchIndex );
  auto&                   frame0       = video.getFrame( shift );
  std::vector<PCCPoint3D> createdPoints;
  auto                    point0 = patch.generatePoint( u, v, frame0.getValue( 0, x, y ));
  createdPoints.push_back( point0 );
  if ( params.singleLayerPixelInterleaving_ ) {
    size_t       patchIndexPlusOne = patchIndex + 1;
    double       depth0, depth1;
    auto&        occupancyMap      = frame.getOccupancyMap();
    auto&        blockToPatch      = frame.getBlockToPatch();
    const auto   imageWidth        = frame0.getWidth();
    const auto   imageHeight       = frame0.getHeight();
    const size_t blockToPatchWidth = frame.getWidth() / params.occupancyResolution_;
    double       DepthNeighbors[4] = {0};
    int          count             = 0;
    double       minimumDepth      = point0[patch.getNormalAxis()];
    double       maximumDepth      = point0[patch.getNormalAxis()];
    if ( x > 0 && occupancyMap[y * imageWidth + x - 1] ) {
      size_t Temp_u0 = ( x - 1 ) / patch.getOccupancyResolution();
      size_t Temp_v0 = ( y ) / patch.getOccupancyResolution();
      if ( blockToPatch[(Temp_v0)*blockToPatchWidth + Temp_u0] == patchIndexPlusOne ) {
        if ( patch.getProjectionMode() == 0 ) {
//          DepthNeighbors[0] = double( frame0.getValue( 0, x - 1, y ) + patch.getD1() ) * lodScale;
          DepthNeighbors[0] = double( frame0.getValue( 0, x - 1, y ) + patch.getD1() );
        } else {
//          DepthNeighbors[0] = double( patch.getD1() - frame0.getValue( 0, x - 1, y ) ) * lodScale;
          DepthNeighbors[0] = double( patch.getD1() - frame0.getValue( 0, x - 1, y ) );
        }
        count++;
        if ( DepthNeighbors[0] < minimumDepth ) { minimumDepth = DepthNeighbors[0]; }
        if ( DepthNeighbors[0] > maximumDepth ) { maximumDepth = DepthNeighbors[0]; }
      }
    }
    if ( x < ( imageWidth - 1 ) && occupancyMap[y * imageWidth + x + 1] ) {
      size_t Temp_u0 = ( x + 1 ) / patch.getOccupancyResolution();
      size_t Temp_v0 = ( y ) / patch.getOccupancyResolution();
      if ( blockToPatch[(Temp_v0)*blockToPatchWidth + Temp_u0] == patchIndexPlusOne ) {
        if ( patch.getProjectionMode() == 0 ) {
//          DepthNeighbors[1] = double( frame0.getValue( 0, x + 1, y ) + patch.getD1() ) * lodScale;
          DepthNeighbors[1] = double( frame0.getValue( 0, x + 1, y ) + patch.getD1() );
        } else {
//          DepthNeighbors[1] = double( patch.getD1() - frame0.getValue( 0, x + 1, y ) ) * lodScale;
          DepthNeighbors[1] = double( patch.getD1() - frame0.getValue( 0, x + 1, y ) );
        }
        count++;
        if ( DepthNeighbors[1] < minimumDepth ) { minimumDepth = DepthNeighbors[1]; }
        if ( DepthNeighbors[1] > maximumDepth ) { maximumDepth = DepthNeighbors[1]; }
      }
    }
    if ( y > 0 && occupancyMap[( y - 1 ) * imageWidth + x] ) {
      size_t Temp_u0 = ( x ) / patch.getOccupancyResolution();
      size_t Temp_v0 = ( y - 1 ) / patch.getOccupancyResolution();
      if ( blockToPatch[(Temp_v0)*blockToPatchWidth + Temp_u0] == patchIndexPlusOne ) {
        if ( patch.getProjectionMode() == 0 ) {
//          DepthNeighbors[2] = double( frame0.getValue( 0, x, y - 1 ) + patch.getD1() ) * lodScale;
          DepthNeighbors[2] = double( frame0.getValue( 0, x, y - 1 ) + patch.getD1() );
        } else {
//          DepthNeighbors[2] = double( patch.getD1() - frame0.getValue( 0, x, y - 1 ) ) * lodScale;
          DepthNeighbors[2] = double( patch.getD1() - frame0.getValue( 0, x, y - 1 ) );
        }
        count++;
        if ( DepthNeighbors[2] < minimumDepth ) { minimumDepth = DepthNeighbors[2]; }
        if ( DepthNeighbors[2] > maximumDepth ) { maximumDepth = DepthNeighbors[2]; }
      }
    }
    if ( y < ( imageHeight - 1 ) && occupancyMap[( y + 1 ) * imageWidth + x] ) {
      size_t Temp_u0 = ( x ) / patch.getOccupancyResolution();
      size_t Temp_v0 = ( y + 1 ) / patch.getOccupancyResolution();
      if ( blockToPatch[(Temp_v0)*blockToPatchWidth + Temp_u0] == patchIndexPlusOne ) {
        if ( patch.getProjectionMode() == 0 ) {
//          DepthNeighbors[3] = double( frame0.getValue( 0, x, y + 1 ) + patch.getD1() ) * lodScale;
          DepthNeighbors[3] = double( frame0.getValue( 0, x, y + 1 ) + patch.getD1() );
        } else {
//          DepthNeighbors[3] = double( patch.getD1() - frame0.getValue( 0, x, y + 1 ) ) * lodScale;
          DepthNeighbors[3] = double( patch.getD1() - frame0.getValue( 0, x, y + 1 ) );
        }
        count++;
        if ( DepthNeighbors[3] < minimumDepth ) { minimumDepth = DepthNeighbors[3]; }
        if ( DepthNeighbors[3] > maximumDepth ) { maximumDepth = DepthNeighbors[3]; }
      }
    }
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
  } else {
    if ( params.pointLocalReconstruction_ ) {
      int deltaDepth = 0;
      if ( interpolate ) {
        deltaDepth =
            getDeltaNeighbors( frame0, patch, x, y, neighbor, NeighborThreshold, patch.getProjectionMode());
      }
      if ( patch.getProjectionMode() == 0 ) {
        deltaDepth = ( std::max )( deltaDepth, (int)minD1 );
      } else {
        deltaDepth = ( std::min )( deltaDepth, -(int)minD1 );
      }
      if ( deltaDepth != 0 ) {
        PCCPoint3D point1( point0 );
        point1[patch.getNormalAxis()] += (double)deltaDepth;
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
    } else {  // else (pointLocalReconstruction)
      if ( params.layerCountMinus1_ > 0 ) {
        if ( !params.absoluteD1_ ) {
          PCCPoint3D  point1( point0 );
          const auto& frame1 = videoD1.getFrame( shift );
          if ( patch.getProjectionMode() == 0 ) {
//            point1[patch.getNormalAxis()] += frame1.getValue( 0, x, y ) * lodScale;
            point1[patch.getNormalAxis()] += frame1.getValue( 0, x, y );
          } else {
//            point1[patch.getNormalAxis()] -= frame1.getValue( 0, x, y ) * lodScale;
            point1[patch.getNormalAxis()] -= frame1.getValue( 0, x, y );
          }
          createdPoints.push_back( point1 );
        } else {
          const auto& frame1 = video.getFrame( 1 + shift );
          auto        point1 = patch.generatePoint( u, v, frame1.getValue( 0, x, y ));
          createdPoints.push_back( point1 );
        }  // else !absoluteD1
      }    // if ( params.layerCountMinus1_ > 0 ) {
    }      // fi (pointLocalReconstruction)
  }        // fi (!params.singleLayerPixelInterleaving_ )
  return createdPoints;
}

void PCCCodec::generatePointCloud( PCCPointSet3&                      reconstruct,
                                   PCCContext&                        context,
                                   PCCFrameContext&                   frame,
                                   const PCCVideoGeometry&            video,
                                   const PCCVideoGeometry&            videoD1,
	                                 const PCCVideoOccupancyMap&        videoOM,
                                   const GeneratePointCloudParameters params,
                                   std::vector<uint32_t>&             partition ) {
  TRACE_CODEC( "generatePointCloud F = %lu start \n", frame.getIndex() );
  auto& patches      = frame.getPatches();
  auto& pointToPixel = frame.getPointToPixel();
  auto& blockToPatch = frame.getBlockToPatch();
	//point cloud occupancy map upscaling from video using nearest neighbor
  auto& occupancyMap = frame.getOccupancyMap();
	auto width = frame.getWidth();
	auto height = frame.getHeight();
	occupancyMap.resize(width * height, 0);
	for (size_t v = 0; v < height; ++v) {
		for (size_t u = 0; u < width; ++u) {
      occupancyMap[v * width + u] = videoOM.getFrame( frame.getIndex() )
                                        .getValue( 0, u / params.occupancyPrecision_, v / params.occupancyPrecision_ );
		}
	}

  partition.resize( 0 );
  pointToPixel.resize( 0 );
  reconstruct.clear();

  TRACE_CODEC( "Frame %lu in generatePointCloud \n", frame.getIndex() );

  bool         useMissedPointsSeparateVideo = frame.getUseMissedPointsSeparateVideo();

  TRACE_CODEC( " params.useAdditionalPointsPatch_ = %d \n", params.useAdditionalPointsPatch_ );
  TRACE_CODEC( " params.enhancedDeltaDepthCode_   = %d \n", params.enhancedDeltaDepthCode_ );
  TRACE_CODEC( " useMissedPointsSeparateVideo     = %d \n", useMissedPointsSeparateVideo );
  
  size_t       shift;
  const size_t layerCount = params.layerCountMinus1_ + 1;
  TRACE_CODEC( " layerCount                       = %d \n", layerCount );
  if ( !params.absoluteD1_ ) {
    shift = frame.getIndex();
    if ( video.getFrameCount() < ( shift + 1 ) ) { return; }
  } else {
    shift = frame.getIndex() * layerCount;
    if ( video.getFrameCount() < ( shift + layerCount ) ) { return; }
  }
  TRACE_CODEC( " shift                            = %d \n", shift );
  const auto&           frame0      = video.getFrame( shift );
  const size_t          imageWidth  = video.getWidth();
  const size_t          imageHeight = video.getHeight();
  std::vector<uint32_t> BPflag;
  BPflag.resize( imageWidth * imageHeight, 0 );

  const size_t blockToPatchWidth  = frame.getWidth() / params.occupancyResolution_;
  const size_t blockToPatchHeight = frame.getHeight() / params.occupancyResolution_;
  reconstruct.addColors();
  const size_t  patchCount    = patches.size();
  size_t        N             = 0;
  uint32_t        patchIndex{0};

  std::vector<std::vector<PCCPoint3D>> eddPointsPerPatch;
  eddPointsPerPatch.resize(patchCount);

  
  for ( patchIndex = 0; patchIndex < patchCount; ++patchIndex ) {
    const size_t patchIndexPlusOne = patchIndex + 1;
    auto&        patch             = patches[patchIndex];
    PCCColor3B   color( uint8_t( 0 ) );

    TRACE_CODEC(
        "P%2lu/%2lu: 2D=(%2lu,%2lu)*(%2lu,%2lu) 3D(%4lu,%4lu,%4lu)*(%4lu,%4lu) A=(%lu,%lu,%lu) Or=%lu P=%lu => %lu \n",
        patchIndex, patchCount, patch.getU0(), patch.getV0(), patch.getSizeU0(), patch.getSizeV0(), patch.getU1(),
        patch.getV1(), patch.getD1(), patch.getSizeU0() * patch.getOccupancyResolution(),
        patch.getSizeV0() * patch.getOccupancyResolution(), patch.getNormalAxis(), patch.getTangentAxis(),
        patch.getBitangentAxis(), patch.getPatchOrientation(), patch.getProjectionMode(), reconstruct.getPointCount() );

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
              size_t       x, y;
              const bool   occupancy = occupancyMap[patch.patch2Canvas( u, v, imageWidth, imageHeight, x, y )] != 0;
              if ( !occupancy ) { continue; }
              if ( params.enhancedDeltaDepthCode_ ) {
                // D0
                PCCPoint3D   point0      = patch.generatePoint( u, v, frame0.getValue( 0, x, y ));
                const size_t pointIndex0 = reconstruct.addPoint( point0 );
                reconstruct.setPointPatchIndex( pointIndex0, patchIndex );
                reconstruct.setColor( pointIndex0, color );
                if ( PCC_SAVE_POINT_TYPE == 1 ) { reconstruct.setType( pointIndex0, POINT_D0 ); }
                partition.push_back( uint32_t( patchIndex ) );
                pointToPixel.push_back( PCCVector3<size_t>( x, y, 0 ) );

                uint16_t eddCode = 0;
                size_t d1pos=0;
                if ( !params.absoluteD1_ ) {
                  std::cout << "EDD improvement is not implemented for absoluteD1=0" << std::endl;
                } else {
                  const auto& frame0 = video.getFrame( shift );
                  const auto& indx   = patch.patch2Canvas( u, v, imageWidth, imageHeight, x, y );
                  if ( params.layerCountMinus1_ > 0 ) {
                    const auto& frame1 = video.getFrame( shift + 1 );
                    int16_t    diff   = (int16_t)frame1.getValue( 0, x, y ) - (int16_t)frame0.getValue( 0, x, y );
                    assert( diff >= 0 );
                    // Convert occupancy map to eddCode
                    if ( diff == 0 ) {
                      eddCode = 0;
                    } else if ( diff == 1 ) {
                      d1pos=1;
                      eddCode = 1;
                    } else if ( diff < 0 ) {
                      // printf( " D0 %d D1 %d OM %d\n", frame0.getValue( 0, x, y ), frame1.getValue( 0, x, y ),
                      //  occupancyMap[patch.patch2Canvas( u, v, imageWidth, imageHeight, x, y )] );
                    } else {
                      uint16_t bits = diff - 1;
                      uint16_t symbol =
                          ( 1 << bits ) - occupancyMap[patch.patch2Canvas( u, v, imageWidth, imageHeight, x, y )];
                      eddCode = symbol | ( 1 << bits );
                      d1pos=( bits );
                    }
                  } else {  // params.layerCountMinus1_ > 0
                    N       = params.EOMFixBitCount_;
                    eddCode = ( 1 << N ) - occupancyMap[indx];
                  }
                }
                PCCPoint3D point1( point0 );
                if ( eddCode == 0 ) {
                  if ( !params.removeDuplicatePoints_ ) {
                    const size_t pointIndex1 = reconstruct.addPoint( point1 );
                    reconstruct.setPointPatchIndex( pointIndex1, patchIndex );
                    reconstruct.setColor( pointIndex1, color );
                    if ( PCC_SAVE_POINT_TYPE == 1 ) { reconstruct.setType( pointIndex1, POINT_D1 ); }
                    partition.push_back( uint32_t( patchIndex ) );
                    pointToPixel.push_back( PCCVector3<size_t>( x, y, 1 ) );
                  }
                } else {  // eddCode != 0
                  uint16_t addedPointCount = 0;
                  size_t   pointIndex1     = 0;
                  for ( uint16_t i = 0; i < 10; i++ ) {
                    if ( eddCode & ( 1 << i ) ) {
                      uint8_t deltaDCur = ( i + 1 );
                      if ( patch.getProjectionMode() == 0 ) {
                        point1[patch.getNormalAxis()] = (double)( point0[patch.getNormalAxis()] + deltaDCur );
                      } else {
                        point1[patch.getNormalAxis()] = (double)( point0[patch.getNormalAxis()] - deltaDCur );
                      }
#if BUGFIX_FIRSTEDDatT1
                      if ( eddCode==1 || i==d1pos )
#else
                      if ( addedPointCount == 0 && params.layerCountMinus1_ > 0 )
#endif
                      { //d1
                        pointIndex1 = reconstruct.addPoint( point1 );
                        reconstruct.setPointPatchIndex( pointIndex1, patchIndex );
                        reconstruct.setColor( pointIndex1, color );
                        if ( PCC_SAVE_POINT_TYPE == 1 ) { reconstruct.setType( pointIndex1, POINT_D1 ); }
                        partition.push_back( uint32_t( patchIndex ) );
                        pointToPixel.push_back( PCCVector3<size_t>( x, y, 1 ) );
                      }
                      else{
                        eddPointsPerPatch[patchIndex].push_back(point1);
                      }
                      addedPointCount++;
                    }
                  }  // for each bit of EDD code
                  if ( PCC_SAVE_POINT_TYPE == 1 ) { reconstruct.setType( pointIndex1, POINT_D1 ); }
                  // Without "Identify boundary points" & "1st Extension boundary region" as EDD code is only for
                  // lossless coding now
                }       // if (eddCode == 0)
              } else {  // not params.enhancedDeltaDepthCode_
                // TRACE_CODEC("generatePoints %4lu %4lu => %4lu %4lu => %4lu %4lu \n",u0, v0, u,v,x,y);
                // TRACE_CODEC(" patch.getPointLocalReconstructionMode( u0, v0 ) = %lu \n",
                // patch.getPointLocalReconstructionMode( u0, v0 ));

                auto& mode = context.getPointLocalReconstructionMode( patch.getPointLocalReconstructionMode( u0, v0 ) );
                auto  createdPoints =
                    generatePoints( params, frame, video, videoD1, shift, patchIndex, u, v, x, y, mode.interpolate_,
                                    mode.filling_, mode.minD1_, mode.neighbor_);
                if ( createdPoints.size() > 0 ) {
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
                        if ( params.singleLayerPixelInterleaving_ ) {
                          size_t flag;
                          flag = ( i == 0 ) ? ( x + y ) % 2 : ( i == 1 ) ? ( x + y + 1 ) % 2 : IntermediateLayerIndex;
                          reconstruct.setType( pointindex_1, flag == 0 ? POINT_D0 : flag == 1 ? POINT_D1 : POINT_DF );
                        } else {
                          reconstruct.setType( pointindex_1, i == 0 ? POINT_D0 : i == 1 ? POINT_D1 : POINT_DF );
                        }
                      }
                      partition.push_back( uint32_t( patchIndex ) );
                      if ( params.singleLayerPixelInterleaving_ ) {
                        pointToPixel.push_back( PCCVector3<size_t>(
                            x, y,
                            i == 0 ? ( ( size_t )( x + y ) % 2 )
                                   : i == 1 ? ( ( size_t )( x + y + 1 ) % 2 ) : IntermediateLayerIndex ) );
                      } else if ( params.pointLocalReconstruction_ ) {
                        pointToPixel.push_back( PCCVector3<size_t>(
                            x, y, i == 0 ? 0 : i == 1 ? IntermediateLayerIndex : IntermediateLayerIndex + 1 ) );
                      } else {
                        pointToPixel.push_back( PCCVector3<size_t>( x, y, i < 2 ? i : IntermediateLayerIndex + 1 ) );
                      }
                    }
                  }
                }
              }  // fi (params.enhancedDeltaDepthCode_)
            }
          }
        }
      }
    }
  }
   
  frame.setTotalNumberOfRegularPoints(reconstruct.getPointCount());
  
  size_t totalEddPointsInFrame = 0;
  PCCPointSet3 eddSavedPoints;
  if (  params.enhancedDeltaDepthCode_ ) {
    size_t numEddPatches = frame.getEomPatches().size();
    for ( int j = 0; j < numEddPatches; j++ ) {
      auto& eomPatch=frame.getEomPatches()[j];
      size_t numPatchesInEddPatches=eomPatch.memberPatches.size();
      size_t u0Eom = useMissedPointsSeparateVideo? 0 : eomPatch.u0_ * params.occupancyResolution_; //seperateVideoCase : 0
      size_t v0Eom = useMissedPointsSeparateVideo? 0 : eomPatch.v0_ * params.occupancyResolution_;
      totalEddPointsInFrame +=eomPatch.eddCount_;
      
      size_t totalPointCount=0;
      
      for(size_t patchCount=0; patchCount<numPatchesInEddPatches; patchCount++){
        size_t memberPatchIdx =eomPatch.memberPatches[patchCount];
        size_t numberOfEddPointsPerPatch = eddPointsPerPatch[memberPatchIdx].size();
        
        for(size_t pointCount=0; pointCount<numberOfEddPointsPerPatch ; pointCount++){
          size_t currBlock                 = totalPointCount/( params.occupancyResolution_ * params.occupancyResolution_);
          size_t nPixelInCurrentBlockCount = totalPointCount-currBlock*(params.occupancyResolution_ * params.occupancyResolution_);
          size_t uBlock                    = currBlock % blockToPatchWidth;
          size_t vBlock                    = currBlock / blockToPatchWidth;
          size_t uu                        = uBlock * params.occupancyResolution_ + nPixelInCurrentBlockCount % params.occupancyResolution_ + u0Eom;
          size_t vv                        = vBlock * params.occupancyResolution_ + nPixelInCurrentBlockCount / params.occupancyResolution_ + v0Eom;
          PCCPoint3D   point1              = eddPointsPerPatch[memberPatchIdx][pointCount];
          size_t pointIndex1               = reconstruct.addPoint( point1 );
          reconstruct.setPointPatchIndex( pointIndex1, patchIndex );
          eddSavedPoints.addPoint(point1);
          //reconstruct.setColor( pointIndex1, color );
          if ( PCC_SAVE_POINT_TYPE == 1 ) { reconstruct.setType( pointIndex1, POINT_EDD ); }
          partition.push_back( uint32_t( patchIndex ) );
          totalPointCount++;
          pointToPixel.push_back( PCCVector3<size_t>( uu, vv, 0 ) );
          occupancyMap[vv * imageWidth + uu] = 1;  // occupied
        }
      }

      TRACE_CODEC("%d eomPatch :%zu,%zu\t %zu patches, %zu points\n", j,u0Eom, v0Eom, numPatchesInEddPatches, eomPatch.eddCount_ );
    }
    frame.setTotalNumberOfEddPoints( totalEddPointsInFrame );
  }
  TRACE_CODEC( " totalEddPointsInFrame = %lu  \n", totalEddPointsInFrame );
  
  TRACE_CODEC( " point = %lu  \n", reconstruct.getPointCount() );
  if ( params.useAdditionalPointsPatch_ ) {
    TRACE_CODEC( " useAdditionalPointsPatch_ \n" );
    if ( useMissedPointsSeparateVideo ) {
      TRACE_CODEC( " useMissedPointsSeparateVideo \n" );
      PCCColor3B missedPointsColor( uint8_t( 0 ) );
      missedPointsColor[0] = 0;
      missedPointsColor[1] = 255;
      missedPointsColor[2] = 255;
      // Add point GPS from missedPointsPatch without inserting to pointToPixel
      size_t numberOfMpsPatches = frame.getNumberOfMissedPointsPatches();
      for ( int j = 0; j < numberOfMpsPatches; j++ ) {
        auto&  missedPointsPatch = frame.getMissedPointsPatch( j );
        size_t sizeofMPs         = missedPointsPatch.getNumberOfMps();
        if ( params.pcmPointColorFormat_ == COLOURFORMAT444 ) {
          for ( int i = 0; i < sizeofMPs; i++ ) {
            PCCVector3D point0;
            point0[0]               = missedPointsPatch.x_[i] + missedPointsPatch.u1_;
            point0[1]               = missedPointsPatch.y_[i] + missedPointsPatch.v1_;
            point0[2]               = missedPointsPatch.z_[i] + missedPointsPatch.d1_;
            const size_t pointIndex = reconstruct.addPoint( point0 );
            reconstruct.setPointPatchIndex( pointIndex, patchIndex );
            reconstruct.setColor( pointIndex, missedPointsColor );
            partition.push_back( uint32_t( patchIndex ) );
          }
        } else {  // else losslessGeo444_
          for ( int i = 0; i < sizeofMPs; i++ ) {
            PCCVector3D point0;
            point0[0]               = missedPointsPatch.x_[i] + missedPointsPatch.u1_;
            point0[1]               = missedPointsPatch.x_[i + sizeofMPs] + missedPointsPatch.v1_;
            point0[2]               = missedPointsPatch.x_[i + 2 * sizeofMPs] + missedPointsPatch.d1_;
            const size_t pointIndex = reconstruct.addPoint( point0 );
            reconstruct.setPointPatchIndex( pointIndex, patchIndex );
            reconstruct.setColor( pointIndex, missedPointsColor );
            partition.push_back( uint32_t( patchIndex ) );
          }
        }  // fi losslessGeo444_
      }
        // secure the size of rgb in missedpointpatch
        size_t numofEddSaved = params.enhancedDeltaDepthCode_ ? frame.getTotalNumberOfEddPoints() : 0;
        size_t numofMPcolors = frame.getTotalNumberOfMissedPoints();
        auto& missedPointsPatch = frame.getMissedPointsPatch( 0 );
        missedPointsPatch.setNumberOfMpsColors( numofMPcolors + numofEddSaved );
        missedPointsPatch.resizeColor( numofMPcolors + numofEddSaved );
    } else {  // else useMissedPointsSeparateVideo
      TRACE_CODEC( " Add points from missedPointsPatch \n" );
      // Add points from missedPointsPatch
      size_t numberOfMpsPatches = frame.getNumberOfMissedPointsPatches();
      printf( "generatepointCloud :: numberOfMpsPatches = %zu \n", numberOfMpsPatches );
      for ( int i = 0; i < numberOfMpsPatches; i++ ) {
        auto& missedPointsPatch = frame.getMissedPointsPatch( i );

        PCCColor3B missedPointsColor( uint8_t( 0 ) );
        missedPointsColor[0] = 0;
        missedPointsColor[1] = 255;
        missedPointsColor[2] = 255;
        size_t numMissedPts  = missedPointsPatch.getNumberOfMps();

        size_t ores              = missedPointsPatch.occupancyResolution_;
        missedPointsPatch.sizeV_ = missedPointsPatch.sizeV0_ * ores;
        missedPointsPatch.sizeU_ = missedPointsPatch.sizeU0_ * ores;
          if ( params.pcmPointColorFormat_ == COLOURFORMAT444 ) {
          for ( size_t v0 = 0; v0 < missedPointsPatch.sizeV0_; ++v0 ) {
            for ( size_t u0 = 0; u0 < missedPointsPatch.sizeU0_; ++u0 ) {
              for ( size_t v1 = 0; v1 < missedPointsPatch.occupancyResolution_; ++v1 ) {
                const size_t v = v0 * missedPointsPatch.occupancyResolution_ + v1;
                for ( size_t u1 = 0; u1 < missedPointsPatch.occupancyResolution_; ++u1 ) {
                  const size_t u         = u0 * missedPointsPatch.occupancyResolution_ + u1;
                  const size_t x         = missedPointsPatch.u0_ * missedPointsPatch.occupancyResolution_ + u;
                  const size_t y         = missedPointsPatch.v0_ * missedPointsPatch.occupancyResolution_ + v;
                  const bool   occupancy = occupancyMap[y * imageWidth + x] != 0;
                  if ( !occupancy ) { continue; }
                  PCCPoint3D point0;
                  point0[0] = double( frame0.getValue( 0, x, y ) ) + missedPointsPatch.u1_;
                  point0[1] = double( frame0.getValue( 1, x, y ) ) + missedPointsPatch.v1_;
                  point0[2] = double( frame0.getValue( 2, x, y ) ) + missedPointsPatch.d1_;

                  const size_t pointIndex = reconstruct.addPoint( point0 );
                  reconstruct.setPointPatchIndex( pointIndex, patchIndex );
                  reconstruct.setColor( pointIndex, missedPointsColor );
                  for ( size_t f = 0; f < layerCount; ++f ) {
                    partition.push_back( uint32_t( patchIndex ) );
                    pointToPixel.push_back( PCCVector3<size_t>( x, y, f ) );
                  }
                }
              }
            }
          }
        } else {
          std::vector<PCCPoint3D> missedPoints;
          missedPoints.resize( numMissedPts );
          size_t       numMissedPointsAdded{0};
          const size_t v0 = missedPointsPatch.v0_ * missedPointsPatch.occupancyResolution_;
          const size_t u0 = missedPointsPatch.u0_ * missedPointsPatch.occupancyResolution_;
          for ( size_t v = 0; v < missedPointsPatch.sizeV_; ++v ) {
            for ( size_t u = 0; u < missedPointsPatch.sizeU_; ++u ) {
              const size_t x = ( u0 + u );
              const size_t y = ( v0 + v );
              if ( numMissedPointsAdded < numMissedPts ) {
                missedPoints[numMissedPointsAdded][0] = double( frame0.getValue( 0, x, y ) + missedPointsPatch.u1_ );
              } else if ( numMissedPts <= numMissedPointsAdded && numMissedPointsAdded < 2 * numMissedPts ) {
                missedPoints[numMissedPointsAdded - numMissedPts][1] =
                    double( frame0.getValue( 0, x, y ) + missedPointsPatch.v1_ );
              } else if ( 2 * numMissedPts <= numMissedPointsAdded && numMissedPointsAdded < 3 * numMissedPts ) {
                missedPoints[numMissedPointsAdded - 2 * numMissedPts][2] =
                    double( frame0.getValue( 0, x, y ) + missedPointsPatch.d1_ );
              }
              numMissedPointsAdded++;
            }  // u
          }    // v
          size_t counter{0};
          for ( size_t v = 0; v < missedPointsPatch.sizeV_; ++v ) {
            for ( size_t u = 0; u < missedPointsPatch.sizeU_; ++u ) {
              const size_t x = ( u0 + u );
              const size_t y = ( v0 + v );
              if ( counter < numMissedPts ) {
                const size_t pointIndex = reconstruct.addPoint( missedPoints[counter] );
                reconstruct.setPointPatchIndex( pointIndex, patchIndex );
                reconstruct.setColor( pointIndex, missedPointsColor );
                partition.push_back( uint32_t( patchIndex ) );
                pointToPixel.push_back( PCCVector3<size_t>( x, y, 0 ) );
                counter++;
              }
            }
          }
        }
      }
    }  // fi :useMissedPointsSeparateVideo
  }    // fi : useAdditionalPointsPatch

  if ( params.flagGeometrySmoothing_ )
  {
    // identify first boundary layer
    if(useMissedPointsSeparateVideo)
      assert( (reconstruct.getPointCount() - frame.getTotalNumberOfMissedPoints() )== pointToPixel.size() );
    else
      assert( (reconstruct.getPointCount() + frame.getTotalNumberOfMissedPoints() )== pointToPixel.size() );
    size_t pointCount = reconstruct.getPointCount() - frame.getTotalNumberOfMissedPoints();
    for ( size_t i = 0; i < pointCount; ++i ) {
      const PCCVector3<size_t> location = pointToPixel[i];
      const size_t             x        = location[0];
      const size_t             y        = location[1];
      if ( occupancyMap[y * imageWidth + x] != 0 )
         identifyBoundaryPoints( occupancyMap, x, y, imageWidth, imageHeight, i, BPflag, reconstruct );
    }
  }
  TRACE_CODEC( " end point = %lu  \n", reconstruct.getPointCount() );
}

bool PCCCodec::gridFiltering( const std::vector<uint32_t>& partition,
                              PCCPointSet3&                pointCloud,
                              PCCPoint3D&                  curPoint,
                              PCCVector3D&                 centroid,
                              int&                         cnt,
                              std::vector<int>&            gcnt,
                              std::vector<PCCVector3D>&    center_grid,
                              std::vector<bool>&           doSmooth,
                              int                          grid,
                              int                          gridWidth ) {
  const int w                      = gridWidth;
  bool      otherClusterPointCount = false;

  int x = curPoint.x();
  int y = curPoint.y();
  int z = curPoint.z();

  int x2 = x / grid;
  int y2 = y / grid;
  int z2 = z / grid;

  int x3 = x % grid;
  int y3 = y % grid;
  int z3 = z % grid;

  int sx = x2 + ( ( x3 < grid / 2 ) ? -1 : 0 );
  int sy = y2 + ( ( y3 < grid / 2 ) ? -1 : 0 );
  int sz = z2 + ( ( z3 < grid / 2 ) ? -1 : 0 );

  int sx2 = sx * grid;
  int sy2 = sy * grid;
  int sz2 = sz * grid;

  int wx = ( x - sx2 - grid / 2 ) * 2 + 1;
  int wy = ( y - sy2 - grid / 2 ) * 2 + 1;
  int wz = ( z - sz2 - grid / 2 ) * 2 + 1;

  int idx[2][2][2];
  for ( int dz = 0; dz < 2; dz++ ) {
    for ( int dy = 0; dy < 2; dy++ ) {
      for ( int dx = 0; dx < 2; dx++ ) {
        int x3          = sx + dx;
        int y3          = sy + dy;
        int z3          = sz + dz;
        int tmp         = x3 + y3 * w + z3 * w * w;
        idx[dz][dy][dx] = tmp;
        if ( doSmooth[tmp] && gcnt[tmp] ) { otherClusterPointCount = true; }
      }
    }
  }

  if ( !otherClusterPointCount ) { return otherClusterPointCount; }
  PCCVector3D centroid3[2][2][2] = {};
  PCCVector3D curVector( x, y, z );
  int         grid2  = grid * 2;
  centroid3[0][0][0] = gcnt[idx[0][0][0]] > 0 ? center_grid[idx[0][0][0]] : curVector;
  centroid3[0][0][1] = gcnt[idx[0][0][1]] > 0 ? center_grid[idx[0][0][1]] : curVector;
  centroid3[0][1][0] = gcnt[idx[0][1][0]] > 0 ? center_grid[idx[0][1][0]] : curVector;
  centroid3[0][1][1] = gcnt[idx[0][1][1]] > 0 ? center_grid[idx[0][1][1]] : curVector;
  centroid3[1][0][0] = gcnt[idx[1][0][0]] > 0 ? center_grid[idx[1][0][0]] : curVector;
  centroid3[1][0][1] = gcnt[idx[1][0][1]] > 0 ? center_grid[idx[1][0][1]] : curVector;
  centroid3[1][1][0] = gcnt[idx[1][1][0]] > 0 ? center_grid[idx[1][1][0]] : curVector;
  centroid3[1][1][1] = gcnt[idx[1][1][1]] > 0 ? center_grid[idx[1][1][1]] : curVector;

  centroid3[0][0][0] = ( grid2 - wx ) * ( grid2 - wy ) * ( grid2 - wz ) * centroid3[0][0][0];
  centroid3[0][0][1] = ( wx ) * ( grid2 - wy ) * ( grid2 - wz ) * centroid3[0][0][1];
  centroid3[0][1][0] = ( grid2 - wx ) * ( wy ) * ( grid2 - wz ) * centroid3[0][1][0];
  centroid3[0][1][1] = ( wx ) * ( wy ) * ( grid2 - wz ) * centroid3[0][1][1];
  centroid3[1][0][0] = ( grid2 - wx ) * ( grid2 - wy ) * (wz)*centroid3[1][0][0];
  centroid3[1][0][1] = ( wx ) * ( grid2 - wy ) * (wz)*centroid3[1][0][1];
  centroid3[1][1][0] = ( grid2 - wx ) * ( wy ) * (wz)*centroid3[1][1][0];
  centroid3[1][1][1] = ( wx ) * ( wy ) * (wz)*centroid3[1][1][1];

  PCCVector3D centroid4;
  centroid4 = centroid3[0][0][0] + centroid3[0][0][1] + centroid3[0][1][0] + centroid3[0][1][1] + centroid3[1][0][0] +
              centroid3[1][0][1] + centroid3[1][1][0] + centroid3[1][1][1];
  centroid4 /= grid2 * grid2 * grid2;

  cnt = 0;
  cnt += ( grid2 - wx ) * ( grid2 - wy ) * ( grid2 - wz ) * gcnt[idx[0][0][0]];
  cnt += ( wx ) * ( grid2 - wy ) * ( grid2 - wz ) * gcnt[idx[0][0][1]];
  cnt += ( grid2 - wx ) * ( wy ) * ( grid2 - wz ) * gcnt[idx[0][1][0]];
  cnt += ( wx ) * ( wy ) * ( grid2 - wz ) * gcnt[idx[0][1][1]];
  cnt += ( grid2 - wx ) * ( grid2 - wy ) * (wz)*gcnt[idx[1][0][0]];
  cnt += ( wx ) * ( grid2 - wy ) * (wz)*gcnt[idx[1][0][1]];
  cnt += ( grid2 - wx ) * ( wy ) * (wz)*gcnt[idx[1][1][0]];
  cnt += ( wx ) * ( wy ) * (wz)*gcnt[idx[1][1][1]];
  cnt /= grid2 * grid2 * grid2;

  centroid = centroid4 * cnt;
  return otherClusterPointCount;
}

void PCCCodec::smoothPointCloudGrid( PCCPointSet3&                      reconstruct,
                                     const std::vector<uint32_t>&       partition,
                                     const GeneratePointCloudParameters params,
                                     int                                gridWidth ) {
  TRACE_CODEC( " smoothPointCloudGrid start \n" );
  const size_t pointCount = reconstruct.getPointCount();
  const int    gridSize   = (int)params.gridSize_;
  const int    disth      = ( std::max )( gridSize / 2, 1 );
  const int    th         = gridSize * gridWidth;
  for ( int c = 0; c < pointCount; c++ ) {
    PCCPoint3D curPoint = reconstruct[c];
    int        x        = (int)curPoint.x();
    int        y        = (int)curPoint.y();
    int        z        = (int)curPoint.z();
    if ( x < disth || y < disth || z < disth || th <= x + disth || th <= y + disth || th <= z + disth ) { continue; }
    PCCVector3D centroid( 0.0 ), curVector( x, y, z );
    int         cnt                    = 0;
    bool        otherClusterPointCount = false;
    if ( reconstruct.getBoundaryPointType( c ) == 1 ) {
      otherClusterPointCount = gridFiltering( partition, reconstruct, curPoint, centroid, cnt, gcnt_, center_grid_,
                                              doSmooth_, gridSize, gridWidth );
    }
    if ( otherClusterPointCount ) {
      double dist2 = ( ( curVector * cnt - centroid ).getNorm2() + (double)cnt / 2.0 ) / (double)cnt;
      if ( dist2 >= ( std::max )( (int)params.thresholdSmoothing_, (int)cnt ) * 2 ) {
        centroid = ( centroid + (double)cnt / 2.0 ) / (double)cnt;
        for ( size_t k = 0; k < 3; ++k ) { centroid[k] = double( int64_t( centroid[k] ) ); }
        reconstruct[c][0] = centroid[0];
        reconstruct[c][1] = centroid[1];
        reconstruct[c][2] = centroid[2];
        if ( PCC_SAVE_POINT_TYPE == 1 ) { reconstruct.setType( c, POINT_SMOOTH ); }
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
  tbb::task_arena limited( (int)params.nbThread_ );
  limited.execute( [&] {
    tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) {
      //  for (size_t i = 0; i < pointCount; ++i) {
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
        const PCCVector3D sclaedPoint =
            double( neighborCount ) * PCCVector3D( reconstruct[i][0], reconstruct[i][1], reconstruct[i][2] );
        const double distToCentroid2 =
            int64_t( ( centroid - sclaedPoint ).getNorm2() + ( neighborCount / 2.0 ) ) / double( neighborCount );
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
  limited.execute( [&] {
    tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) {
      // for (size_t i = 0; i < pointCount; ++i) {
      reconstruct[i] = temp[i];
    } );
  } );
  TRACE_CODEC( " smoothPointCloud done \n" );
}

void PCCCodec::smoothPointCloudColor( PCCPointSet3& reconstruct, const GeneratePointCloudParameters params ) {
  TRACE_CODEC( " smoothPointCloudColor start \n" );
  const size_t            pointCount = reconstruct.getPointCount();
  PCCKdTree               kdtree( reconstruct );
  std::vector<PCCColor3B> temp;
  temp.resize( pointCount );

  for ( size_t m = 0; m < pointCount; ++m ) { temp[m] = reconstruct.getColor( m ); }

  tbb::task_arena limited( (int)params.nbThread_ );
  limited.execute( [&] {
    tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) {
      //  for (size_t i = 0; i < pointCount; ++i) {

      PCCNNResult result;

      if ( reconstruct.getBoundaryPointType( i ) == 1 ) {
        kdtree.searchRadius( reconstruct[i], params.neighborCountColorSmoothing_, params.radius2ColorSmoothing_,
                             result );
        PCCVector3D centroid( 0.0 );

        size_t               neighborCount = 0;
        std::vector<uint8_t> Lum;
        bool                 otherClusterPointCount = false;
        const size_t         queryPointPatchIndex   = reconstruct.getPointPatchIndex( i );

        for ( size_t r = 0; r < result.count(); ++r ) {
          ++neighborCount;
          const size_t index = result.indices( r );
          PCCColor3B   color = reconstruct.getColor( index );
          centroid[0] += double( color[0] );
          centroid[1] += double( color[1] );
          centroid[2] += double( color[2] );

          otherClusterPointCount |= ( queryPointPatchIndex != reconstruct.getPointPatchIndex( index ) );

          double Y = 0.2126 * double( color[0] ) + 0.7152 * double( color[1] ) + 0.0722 * double( color[2] );
          Lum.push_back( uint8_t( Y ) );
        }

        PCCColor3B color;

        if ( otherClusterPointCount ) {
          for ( size_t k = 0; k < 3; ++k ) {
            centroid[k] = double( int64_t( centroid[k] + ( neighborCount / 2 ) ) / neighborCount );
          }

          // Texture characterization
          double     H               = entropy( Lum, int( neighborCount ) );
          PCCColor3B colorQP         = reconstruct.getColor( i );
          double     distToCentroid2 = 0;
          for ( size_t k = 0; k < 3; ++k ) { distToCentroid2 += abs( centroid[k] - double( colorQP[k] ) ); }
          if ( distToCentroid2 >= double( params.thresholdColorSmoothing_ ) && H < 4.5 ) {
            color[0] = uint8_t( centroid[0] );
            color[1] = uint8_t( centroid[1] );
            color[2] = uint8_t( centroid[2] );
            temp[i]  = color;
          }
        }
      }
    } );
  } );
  limited.execute( [&] {
    tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) { reconstruct.setColor( i, temp[i] ); } );
  } );
}

void PCCCodec::addGridColorCentroid( PCCPoint3D&                        point,
                                     PCCVector3D&                       color,
                                     int                                patchIdx,
                                     std::vector<int>&                  color_gcnt,
                                     std::vector<PCCVector3D>&          color_center_grid,
                                     std::vector<int>&                  color_gpartition,
                                     std::vector<bool>&                 color_doSmooth,
                                     int                                cgrid,
                                     std::vector<std::vector<uint8_t>>& CS_glum,
                                     const GeneratePointCloudParameters params ) {
  const int w = pow( 2, params.geometryBitDepth3D_ ) / cgrid;

  int x   = point.x() / cgrid;
  int y   = point.y() / cgrid;
  int z   = point.z() / cgrid;
  int idx = x + y * w + z * w * w;
  if ( color_gcnt[idx] == 0 ) {
    color_gpartition[idx]  = patchIdx;
    color_center_grid[idx] = PCCVector3D( 0, 0, 0 );
    color_doSmooth[idx]    = false;
  } else if ( !color_doSmooth[idx] && color_gpartition[idx] != patchIdx ) {
    color_doSmooth[idx] = true;
  }
  color_center_grid[idx] += color;
  color_gcnt[idx]++;

  double Y = 0.2126 * color[0] + 0.7152 * color[1] + 0.0722 * color[2];
  CS_glum[idx].push_back( uint8_t( Y ) );
}

bool PCCCodec::GridFilteringColor( PCCPoint3D&                        curPos,
                                   PCCVector3D&                       color_centroid,
                                   int&                               color_cnt,
                                   std::vector<int>&                  color_gcnt,
                                   std::vector<PCCVector3D>&          color_center_grid,
                                   std::vector<bool>&                 color_doSmooth,
                                   int                                cgrid,
                                   PCCVector3D&                       curPosColor,
                                   const GeneratePointCloudParameters params ) {
  const int w                      = pow( 2, params.geometryBitDepth3D_ ) / cgrid;
  bool      otherClusterPointCount = false;
  int       x                      = curPos.x();
  int       y                      = curPos.y();
  int       z                      = curPos.z();

  int x2 = x / cgrid;
  int y2 = y / cgrid;
  int z2 = z / cgrid;

  int x3 = x % cgrid;
  int y3 = y % cgrid;
  int z3 = z % cgrid;

  int sx = x2 + ( ( x3 < cgrid / 2 ) ? -1 : 0 );
  int sy = y2 + ( ( y3 < cgrid / 2 ) ? -1 : 0 );
  int sz = z2 + ( ( z3 < cgrid / 2 ) ? -1 : 0 );

  int sx2 = sx * cgrid;
  int sy2 = sy * cgrid;
  int sz2 = sz * cgrid;

  int wx = ( x - sx2 - cgrid / 2 ) * 2 + 1;
  int wy = ( y - sy2 - cgrid / 2 ) * 2 + 1;
  int wz = ( z - sz2 - cgrid / 2 ) * 2 + 1;

  int idx[2][2][2];
  for ( int dz = 0; dz < 2; dz++ ) {
    for ( int dy = 0; dy < 2; dy++ ) {
      for ( int dx = 0; dx < 2; dx++ ) {
        int x3          = sx + dx;
        int y3          = sy + dy;
        int z3          = sz + dz;
        int tmp         = x3 + y3 * w + z3 * w * w;
        idx[dz][dy][dx] = tmp;
        if ( color_doSmooth[tmp] && color_gcnt[tmp] ) { otherClusterPointCount = true; }
      }
    }
  }

  if ( !otherClusterPointCount ) { return otherClusterPointCount; }

  int cnt0;

  PCCVector3D color_centroid3[2][2][2] = {};
  int         cgrid2                   = cgrid * 2;

  double mmThresh = params.thresholdColorVariation_;
  double yThresh  = params.thresholdColorDifference_;

  if ( color_gcnt[idx[0][0][0]] > 0 ) {
    color_centroid3[0][0][0] = color_center_grid[idx[0][0][0]] / double( color_gcnt[idx[0][0][0]] );
    cnt0                     = color_gcnt[idx[0][0][0]];
    if ( color_gcnt[idx[0][0][0]] > 1 ) {
      double meanY   = mean( CS_gLum_[idx[0][0][0]], int( color_gcnt[idx[0][0][0]] ) );
      double medianY = median( CS_gLum_[idx[0][0][0]], int( color_gcnt[idx[0][0][0]] ) );
      if ( abs( meanY - medianY ) > mmThresh ) {
        color_centroid3[0][0][0] = curPosColor;
        cnt0                     = 1;
      }
    }
  } else {
    color_centroid3[0][0][0] = curPosColor;
    cnt0                     = 1;
  }

  double Y0 = ( 0.2126 * color_centroid3[0][0][0][0] + 0.7152 * color_centroid3[0][0][0][1] +
                0.0722 * color_centroid3[0][0][0][2] ) /
              double( cnt0 );

  if ( color_gcnt[idx[0][0][1]] > 0 ) {
    color_centroid3[0][0][1] = color_center_grid[idx[0][0][1]] / double( color_gcnt[idx[0][0][1]] );
    double Y1                = ( 0.2126 * color_centroid3[0][0][1][0] + 0.7152 * color_centroid3[0][0][1][1] +
                  0.0722 * color_centroid3[0][0][1][2] ) /
                double( color_gcnt[idx[0][0][1]] );

    if ( abs( Y0 - Y1 ) > yThresh ) { color_centroid3[0][0][1] = curPosColor; }
    if ( color_gcnt[idx[0][0][1]] > 1 ) {
      double meanY   = mean( CS_gLum_[idx[0][0][1]], int( color_gcnt[idx[0][0][1]] ) );
      double medianY = median( CS_gLum_[idx[0][0][1]], int( color_gcnt[idx[0][0][1]] ) );
      if ( abs( meanY - medianY ) > mmThresh ) { color_centroid3[0][0][1] = curPosColor; }
    }
  } else {
    color_centroid3[0][0][1] = curPosColor;
  }

  if ( color_gcnt[idx[0][1][0]] > 0 ) {
    color_centroid3[0][1][0] = color_center_grid[idx[0][1][0]] / double( color_gcnt[idx[0][1][0]] );
    double Y2                = ( 0.2126 * color_centroid3[0][1][0][0] + 0.7152 * color_centroid3[0][1][0][1] +
                  0.0722 * color_centroid3[0][1][0][2] ) /
                double( color_gcnt[idx[0][1][0]] );

    if ( abs( Y0 - Y2 ) > yThresh ) { color_centroid3[0][1][0] = curPosColor; }
    if ( color_gcnt[idx[0][1][0]] > 1 ) {
      double meanY   = mean( CS_gLum_[idx[0][1][0]], int( color_gcnt[idx[0][1][0]] ) );
      double medianY = median( CS_gLum_[idx[0][1][0]], int( color_gcnt[idx[0][1][0]] ) );
      if ( abs( meanY - medianY ) > mmThresh ) { color_centroid3[0][1][0] = curPosColor; }
    }
  } else {
    color_centroid3[0][1][0] = curPosColor;
  }

  if ( color_gcnt[idx[0][1][1]] > 0 ) {
    color_centroid3[0][1][1] = color_center_grid[idx[0][1][1]] / double( color_gcnt[idx[0][1][1]] );
    double Y3                = ( 0.2126 * color_centroid3[0][1][1][0] + 0.7152 * color_centroid3[0][1][1][1] +
                  0.0722 * color_centroid3[0][1][1][2] ) /
                double( color_gcnt[idx[0][1][1]] );

    if ( abs( Y0 - Y3 ) > yThresh ) { color_centroid3[0][1][1] = curPosColor; }
    if ( color_gcnt[idx[0][1][1]] > 1 ) {
      double meanY   = mean( CS_gLum_[idx[0][1][1]], int( color_gcnt[idx[0][1][1]] ) );
      double medianY = median( CS_gLum_[idx[0][1][1]], int( color_gcnt[idx[0][1][1]] ) );
      if ( abs( meanY - medianY ) > mmThresh ) { color_centroid3[0][1][1] = curPosColor; }
    }
  } else {
    color_centroid3[0][1][1] = curPosColor;
  }

  if ( color_gcnt[idx[1][0][0]] > 0 ) {
    color_centroid3[1][0][0] = color_center_grid[idx[1][0][0]] / double( color_gcnt[idx[1][0][0]] );
    double Y4                = ( 0.2126 * color_centroid3[1][0][0][0] + 0.7152 * color_centroid3[1][0][0][1] +
                  0.0722 * color_centroid3[1][0][0][2] ) /
                double( color_gcnt[idx[1][0][0]] );

    if ( abs( Y0 - Y4 ) > yThresh ) { color_centroid3[1][0][0] = curPosColor; }
    if ( color_gcnt[idx[1][0][0]] > 1 ) {
      double meanY   = mean( CS_gLum_[idx[1][0][0]], int( color_gcnt[idx[1][0][0]] ) );
      double medianY = median( CS_gLum_[idx[1][0][0]], int( color_gcnt[idx[1][0][0]] ) );
      if ( abs( meanY - medianY ) > mmThresh ) { color_centroid3[1][0][0] = curPosColor; }
    }
  } else {
    color_centroid3[1][0][0] = curPosColor;
  }

  if ( color_gcnt[idx[1][0][1]] > 0 ) {
    color_centroid3[1][0][1] = color_center_grid[idx[1][0][1]] / double( color_gcnt[idx[1][0][1]] );
    double Y5                = ( 0.2126 * color_centroid3[1][0][1][0] + 0.7152 * color_centroid3[1][0][1][1] +
                  0.0722 * color_centroid3[1][0][1][2] ) /
                double( color_gcnt[idx[1][0][1]] );

    if ( abs( Y0 - Y5 ) > yThresh ) { color_centroid3[1][0][1] = curPosColor; }
    if ( color_gcnt[idx[1][0][1]] > 1 ) {
      double meanY   = mean( CS_gLum_[idx[1][0][1]], int( color_gcnt[idx[1][0][1]] ) );
      double medianY = median( CS_gLum_[idx[1][0][1]], int( color_gcnt[idx[1][0][1]] ) );
      if ( abs( meanY - medianY ) > mmThresh ) { color_centroid3[1][0][1] = curPosColor; }
    }
  } else {
    color_centroid3[1][0][1] = curPosColor;
  }

  if ( color_gcnt[idx[1][1][0]] > 0 ) {
    color_centroid3[1][1][0] = color_center_grid[idx[1][1][0]] / double( color_gcnt[idx[1][1][0]] );
    double Y6                = ( 0.2126 * color_centroid3[1][1][0][0] + 0.7152 * color_centroid3[1][1][0][1] +
                  0.0722 * color_centroid3[1][1][0][2] ) /
                double( color_gcnt[idx[1][1][0]] );

    if ( abs( Y0 - Y6 ) > yThresh ) { color_centroid3[1][1][0] = curPosColor; }
    if ( color_gcnt[idx[1][1][0]] > 1 ) {
      double meanY   = mean( CS_gLum_[idx[1][1][0]], int( color_gcnt[idx[1][1][0]] ) );
      double medianY = median( CS_gLum_[idx[1][1][0]], int( color_gcnt[idx[1][1][0]] ) );
      if ( abs( meanY - medianY ) > mmThresh ) { color_centroid3[1][1][0] = curPosColor; }
    }
  } else {
    color_centroid3[1][1][0] = curPosColor;
  }

  if ( color_gcnt[idx[1][1][1]] > 0 ) {
    color_centroid3[1][1][1] = color_center_grid[idx[1][1][1]] / double( color_gcnt[idx[1][1][1]] );
    double Y7                = ( 0.2126 * color_centroid3[1][1][1][0] + 0.7152 * color_centroid3[1][1][1][1] +
                  0.0722 * color_centroid3[1][1][1][2] ) /
                double( color_gcnt[idx[1][1][1]] );

    if ( abs( Y0 - Y7 ) > yThresh ) { color_centroid3[1][1][1] = curPosColor; }
    if ( color_gcnt[idx[1][1][1]] > 1 ) {
      double meanY   = mean( CS_gLum_[idx[1][1][1]], int( color_gcnt[idx[1][1][1]] ) );
      double medianY = median( CS_gLum_[idx[1][1][1]], int( color_gcnt[idx[1][1][1]] ) );
      if ( abs( meanY - medianY ) > mmThresh ) { color_centroid3[1][1][1] = curPosColor; }
    }
  } else {
    color_centroid3[1][1][1] = curPosColor;
  }

  color_centroid3[0][0][0] = ( cgrid2 - wx ) * ( cgrid2 - wy ) * ( cgrid2 - wz ) * color_centroid3[0][0][0];
  color_centroid3[0][0][1] = ( wx ) * ( cgrid2 - wy ) * ( cgrid2 - wz ) * color_centroid3[0][0][1];
  color_centroid3[0][1][0] = ( cgrid2 - wx ) * ( wy ) * ( cgrid2 - wz ) * color_centroid3[0][1][0];
  color_centroid3[0][1][1] = ( wx ) * ( wy ) * ( cgrid2 - wz ) * color_centroid3[0][1][1];
  color_centroid3[1][0][0] = ( cgrid2 - wx ) * ( cgrid2 - wy ) * (wz)*color_centroid3[1][0][0];
  color_centroid3[1][0][1] = ( wx ) * ( cgrid2 - wy ) * (wz)*color_centroid3[1][0][1];
  color_centroid3[1][1][0] = ( cgrid2 - wx ) * ( wy ) * (wz)*color_centroid3[1][1][0];
  color_centroid3[1][1][1] = ( wx ) * ( wy ) * (wz)*color_centroid3[1][1][1];

  PCCVector3D color_centroid4;
  color_centroid4 = color_centroid3[0][0][0] + color_centroid3[0][0][1] + color_centroid3[0][1][0] +
                    color_centroid3[0][1][1] + color_centroid3[1][0][0] + color_centroid3[1][0][1] +
                    color_centroid3[1][1][0] + color_centroid3[1][1][1];
  color_centroid4 /= cgrid2 * cgrid2 * cgrid2;

  color_centroid = color_centroid4;
  color_cnt      = 1;

  return otherClusterPointCount;
}

void PCCCodec::smoothPointCloudColorLC( PCCPointSet3& reconstruct, const GeneratePointCloudParameters params ) {
  const size_t pointCount = reconstruct.getPointCount();
  const int    cgrid      = params.cgridSize_;
  const int    disth      = ( std::max )( cgrid / 2, 1 );

  for ( int i = 0; i < pointCount; i++ ) {
    PCCPoint3D curPos = reconstruct[i];
    int        x      = curPos.x();
    int        y      = curPos.y();
    int        z      = curPos.z();

    int pcMaxSize = pow( 2, params.geometryBitDepth3D_ );
    if ( x < disth || y < disth || z < disth || pcMaxSize <= x + disth || pcMaxSize <= y + disth ||
         pcMaxSize <= z + disth ) {
      continue;
    }


    PCCVector3D color_centroid( 0.0 );
    int         color_cnt = 0;

    bool otherClusterPointCount = false;

    PCCColor3B  color = reconstruct.getColor( i );
    PCCVector3D curPosColor( 0.0 );
    curPosColor[0] = double( color[0] );
    curPosColor[1] = double( color[1] );
    curPosColor[2] = double( color[2] );
    if ( reconstruct.getBoundaryPointType( i ) == 1 ) {
      otherClusterPointCount =
          GridFilteringColor( curPos, color_centroid, color_cnt, CS_color_gcnt_, CS_color_center_grid_,
                              CS_color_doSmooth_, cgrid, curPosColor, params );
    }

    if ( otherClusterPointCount ) {
      color_centroid = ( color_centroid + (double)color_cnt / 2.0 ) / (double)color_cnt;
      for ( size_t k = 0; k < 3; ++k ) { color_centroid[k] = double( int64_t( color_centroid[k] ) ); }

      double distToCentroid2 = 0;
      double Ycent           = 0.2126 * double( color_centroid[0] ) + 0.7152 * double( color_centroid[1] ) +
                     0.0722 * double( color_centroid[2] );
      double Ycur =
          0.2126 * double( curPosColor[0] ) + 0.7152 * double( curPosColor[1] ) + 0.0722 * double( curPosColor[2] );

      distToCentroid2 = abs( Ycent - Ycur ) * 10.;
      if ( distToCentroid2 >= params.thresholdColorSmoothing_ ) {
		  PCCColor3B color;
          color[0] = uint8_t( color_centroid[0] );
          color[1] = uint8_t( color_centroid[1] );
          color[2] = uint8_t( color_centroid[2] );

          reconstruct.setColor( i, color );
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
  if ( !pointCount || !reconstruct.hasColors() ) { return; }
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
  if ( !pointCount || !reconstruct.hasColors() ) { return; }
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
                                PCCFrameContext&                    frame,
                                const PCCVideoTexture&              video,
                                const uint8_t                       attributeCount,
                                const GeneratePointCloudParameters& params ) {
  TRACE_CODEC( " colorPointCloud start \n" );
  const size_t frameCount              = params.layerCountMinus1_ + 1;
  size_t       numberOfMpsAndEddColors = 0;
  size_t       numOfMPGeos             = 0;
  size_t       numberOfEddPoints       = 0;
  if ( attributeCount == 0 ) {
    for ( auto& color : reconstruct.getColors() ) {
      for ( size_t c = 0; c < 3; ++c ) { color[c] = static_cast<uint8_t>( 127 ); }
    }
  } else {
    auto& pointToPixel                 = frame.getPointToPixel();
    auto& color                        = reconstruct.getColors();
    bool  useMissedPointsSeparateVideo = frame.getUseMissedPointsSeparateVideo();
    bool  losslessAtt                  = frame.getLosslessGeo();
    bool  losslessGeo                  = frame.getLosslessGeo();
    bool  lossyMissedPointsPatch       = !losslessGeo && frame.getPcmPatchEnabledFlag();
    numOfMPGeos                        = frame.getTotalNumberOfMissedPoints();
    numberOfEddPoints                  = frame.getTotalNumberOfEddPoints();
    numberOfMpsAndEddColors            = numOfMPGeos + numberOfEddPoints;
    size_t pointCount                  = reconstruct.getPointCount();
    if ( ( losslessAtt || lossyMissedPointsPatch ) && useMissedPointsSeparateVideo ) {
      //auto& missedPointsPatch = frame.getMissedPointsPatch( 0 );

      numOfMPGeos             = frame.getTotalNumberOfMissedPoints();
      numberOfEddPoints       = frame.getTotalNumberOfEddPoints();
      numberOfMpsAndEddColors = numOfMPGeos + numberOfEddPoints;

      if ( useMissedPointsSeparateVideo && ( losslessAtt || lossyMissedPointsPatch ) ) {
        pointCount = reconstruct.getPointCount() - numOfMPGeos - numberOfEddPoints;
        assert( numberOfMpsAndEddColors == ( numberOfEddPoints + numOfMPGeos ) );        
      }
    }

    TRACE_CODEC( "useMissedPointsSeparateVideo = %d \n", useMissedPointsSeparateVideo );
    TRACE_CODEC( "losslessAtt                  = %d \n", losslessAtt );
    TRACE_CODEC( "losslessGeo                  = %d \n", losslessGeo );
    TRACE_CODEC( "lossyMissedPointsPatch       = %d \n", lossyMissedPointsPatch );
    if ( params.enhancedDeltaDepthCode_ ) {
      TRACE_CODEC( "numberOfMpsAndEddColors      = %lu \n", numberOfMpsAndEddColors );
      TRACE_CODEC( "numberOfEddPoints            = %lu \n", numberOfEddPoints );
    }
    TRACE_CODEC( "numOfMPGeos                  = %lu \n", numOfMPGeos );
    TRACE_CODEC( "pointCount                   = %lu \n", pointCount );
    TRACE_CODEC( "pointLocalReconstruction     = %d \n", params.pointLocalReconstruction_ );
    TRACE_CODEC( "singleLayerPixelInterleaving = %d \n", params.singleLayerPixelInterleaving_ );
    TRACE_CODEC( "enhancedDeltaDepthCode       = %d \n", params.enhancedDeltaDepthCode_ );

    if ( !pointCount || !reconstruct.hasColors() ) { return false; }
    PCCPointSet3        target;
    PCCPointSet3        source;
    std::vector<size_t> targetIndex;
    targetIndex.resize( 0 );
    target.clear();
    source.clear();
    target.addColors();
    source.addColors();
    const size_t shift = frame.getIndex() * frameCount;
    for ( size_t i = 0; i < pointCount; ++i ) {
      const PCCVector3<size_t> location = pointToPixel[i];
      const size_t             x        = location[0];
      const size_t             y        = location[1];
      const size_t             f        = location[2];
      if ( params.singleLayerPixelInterleaving_ ) {
        if ( ( f == 0 && ( x + y ) % 2 == 0 ) | ( f == 1 && ( x + y ) % 2 == 1 ) ) {
          const auto& frame = video.getFrame( shift );
          for ( size_t c = 0; c < 3; ++c ) { color[i][c] = frame.getValue( c, x, y ); }
          int index = source.addPoint( reconstruct[i] );
          source.setColor( index, color[i] );
        } else {
          target.addPoint( reconstruct[i] );
          targetIndex.push_back( i );
        }
      } else {
        if ( f < frameCount ) {
          const auto& frame = video.getFrame( shift + f );
          for ( size_t c = 0; c < 3; ++c ) { color[i][c] = frame.getValue( c, x, y ); }
          int index = source.addPoint( reconstruct[i] );
          source.setColor( index, color[i] );
        } else {
          target.addPoint( reconstruct[i] );
          targetIndex.push_back( i );
        }
      }
    }

    if ( target.getPointCount() > 0 ) {
      source.transfertColorWeight( target );
      for ( size_t i = 0; i < target.getPointCount(); ++i ) {
        reconstruct.setColor( targetIndex[i], target.getColor( i ) );
      }
    }
    if ( ( losslessAtt || lossyMissedPointsPatch ) && useMissedPointsSeparateVideo ) {
      //auto& missedPointsPatch = frame.getMissedPointsPatch( 0 );
      std::vector<PCCColor3B>& mpsTextures       = frame.getMpsTextures();
      std::vector<PCCColor3B>& eddTextures       = frame.getEddTextures();
      for ( size_t i = 0; i < numOfMPGeos; ++i ) {
        color[pointCount + numberOfEddPoints + i] = mpsTextures[i];
      }
      for ( size_t i = 0; i < numberOfEddPoints; ++i ) { 
        color[pointCount + i] = eddTextures[i]; 
      }
    }
  }  // noAtt
  TRACE_CODEC( " colorPointCloud done \n" );
  return true;
}

void PCCCodec::generateMissedPointsGeometryfromVideo( PCCContext& context, PCCGroupOfFrames& reconstructs ) {
  TRACE_CODEC( " generateMissedPointsGeometryfromVideo start \n" );
  auto&        sps              = context.getSps();
  const size_t gofSize          = context.size();
  auto&        videoMPsGeometry = context.getVideoMPsGeometry();
  videoMPsGeometry.resize( gofSize );
  for ( auto& framecontext : context.getFrames() ) {
    const size_t shift = framecontext.getIndex();
    framecontext.setLosslessGeo( sps.getLosslessGeo() );
    framecontext.setLosslessGeo444( sps.getLosslessGeo444() );
    generateMPsGeometryfromImage( context, framecontext, reconstructs, shift );

    size_t totalNumPCMPoints=0;
    for(size_t ii=0; ii<framecontext.getNumberOfMissedPointsPatches(); ii++)
      totalNumPCMPoints+=framecontext.getMissedPointsPatch(ii).size();
    std::cout << "generate PCM Points Video (Geometry) frame  " << shift
              << "from Video : # of PCM Patches : " << framecontext.getNumberOfMissedPointsPatches()
              << " total # of PCM Geometry : " << totalNumPCMPoints << std::endl;

  }

  std::cout << "PCM Points Geometry from Video [done]" << std::endl;
  TRACE_CODEC( " generateMissedPointsGeometryfromVideo done \n" );
}

void PCCCodec::generateMPsGeometryfromImage( PCCContext&       context,
                                             PCCFrameContext&  frame,
                                             PCCGroupOfFrames& reconstructs,
                                             size_t            frameIndex ) {
  auto&  videoMPsGeometry   = context.getVideoMPsGeometry();
  auto&  image              = videoMPsGeometry.getFrame( frameIndex );
  size_t numberOfMpsPatches = frame.getNumberOfMissedPointsPatches();
  for ( int i = 0; i < numberOfMpsPatches; i++ ) {
    auto&        missedPointsPatch = frame.getMissedPointsPatch( i );
    const size_t v0                = missedPointsPatch.v0_ * missedPointsPatch.occupancyResolution_;
    const size_t u0                = missedPointsPatch.u0_ * missedPointsPatch.occupancyResolution_;
    missedPointsPatch.sizeV_       = missedPointsPatch.sizeV0_ * missedPointsPatch.occupancyResolution_;
    missedPointsPatch.sizeU_       = missedPointsPatch.sizeU0_ * missedPointsPatch.occupancyResolution_;
    size_t numberOfMps             = missedPointsPatch.getNumberOfMps();
    if ( !frame.getLosslessGeo444() ) numberOfMps *= 3;
    missedPointsPatch.resize( numberOfMps );
    for ( size_t v = 0; v < missedPointsPatch.sizeV_; ++v ) {
      for ( size_t u = 0; u < missedPointsPatch.sizeU_; ++u ) {
        const size_t p = v * missedPointsPatch.sizeU_ + u;
        if ( p < numberOfMps ) {
          const size_t x = ( u0 + u );
          const size_t y = ( v0 + v );
          if ( frame.getLosslessGeo444() ) {
            missedPointsPatch.x_[p] = image.getValue( 0, x, y );
            missedPointsPatch.y_[p] = image.getValue( 1, x, y );
            missedPointsPatch.z_[p] = image.getValue( 2, x, y );
          } else {
            missedPointsPatch.x_[p] = image.getValue( 0, x, y );
          }
        }
      }
    }
  }
}

void PCCCodec::generateMissedPointsTexturefromVideo( PCCContext& context, PCCGroupOfFrames& reconstructs ) {
  const size_t gofSize         = context.size();
  auto&        videoMPsTexture = context.getVideoMPsTexture();
  videoMPsTexture.resize( gofSize );
  for ( auto& framecontext : context.getFrames() ) {
    const size_t shift = framecontext.getIndex();  //
    generateMPsTexturefromImage( context, framecontext, reconstructs, shift );
    std::cout << "generate Missed Points (Texture) : frame " << shift
              << ", # of Missed Points Texture : " << framecontext.getMissedPointsPatch( 0 ).size() << std::endl;
  }
  std::cout << "MissedPoints Texture [done]" << std::endl;
}

void PCCCodec::generateMPsTexturefromImage( PCCContext&       context,
                                            PCCFrameContext&  frame,
                                            PCCGroupOfFrames& reconstructs,
                                            size_t            frameIndex ) {
  auto&        videoMPsTexture   = context.getVideoMPsTexture();
  auto&        image             = videoMPsTexture.getFrame( frameIndex );
  size_t width           = image.getWidth();
  size_t height          = image.getHeight();
  context.setMPAttWidth( width );
  context.setMPAttHeight( height );
  size_t       numberOfEddPoints = frame.getTotalNumberOfEddPoints();
  size_t       numOfMPGeos       = frame.getTotalNumberOfMissedPoints();
  std::vector<PCCColor3B>& mpsTextures       = frame.getMpsTextures();
  std::vector<PCCColor3B>& eddTextures       = frame.getEddTextures();
  mpsTextures.resize( numOfMPGeos );
  eddTextures.resize( numberOfEddPoints );

  size_t heightMP = numOfMPGeos / width + 1;

  size_t heightby8 = heightMP / 8;
  if ( heightby8 * 8 != heightMP ) { heightMP = ( heightby8 + 1 ) * 8; }

  size_t mpsV0;
  size_t maxMpsV0           = 0;
  size_t numberOfMpsPatches = frame.getNumberOfMissedPointsPatches();

  for ( int i = 0; i < numberOfMpsPatches; i++ ) {
    auto&        missedPointsPatch = frame.getMissedPointsPatch( i );
    mpsV0                   = missedPointsPatch.v0_ * missedPointsPatch.occupancyResolution_ + missedPointsPatch.sizeV_;

    if ( mpsV0 > maxMpsV0 ) maxMpsV0 = mpsV0;
  }
  heightMP = maxMpsV0;

  int framePointIndex = 0;
  for ( int i = 0; i < numberOfMpsPatches; i++ ) {
    int          pointIndex   = 0;
    auto&        missedPointsPatch = frame.getMissedPointsPatch( i );
    size_t       numMps            = missedPointsPatch.getNumberOfMps();
    const size_t v0                = missedPointsPatch.v0_ * missedPointsPatch.occupancyResolution_;
    const size_t u0                = missedPointsPatch.u0_ * missedPointsPatch.occupancyResolution_;
    for ( size_t v = 0; v < missedPointsPatch.sizeV_; ++v ) {
      for ( size_t u = 0; u < missedPointsPatch.sizeU_; ++u ) {
        //const size_t p = v * missedPointsPatch.sizeU_ + u;
        if ( pointIndex < numMps ) {
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
  for ( size_t i = 0; i < numberOfEddPoints; i++ ) {
    assert( ( i + numOfMPGeos ) / width < height );
    size_t xx, yy;
    size_t nBlock = i / 256;
    size_t uBlock = nBlock % ( width / 16 );
    size_t vBlock = nBlock / ( width / 16 );
    xx            = uBlock * 16 + ( nPixelInCurrentBlockCount % 16 );
    yy            = vBlock * 16 + ( nPixelInCurrentBlockCount / 16 ) + heightMP;
    ++nPixelInCurrentBlockCount;
    if ( nPixelInCurrentBlockCount >= 256 ) nPixelInCurrentBlockCount = 0;
    eddTextures[i].r() = image.getValue( 0, xx, yy );
    eddTextures[i].g() = image.getValue( 1, xx, yy );
    eddTextures[i].b() = image.getValue( 2, xx, yy );
  }
}

void PCCCodec::generateOccupancyMap( PCCContext&  context,
                                     const size_t occupancyPrecision,
                                     const size_t thresholdLossyOM,
                                     bool         enhancedOccupancyMapForDepthFlag ) {
  for ( auto& frame : context.getFrames() ) {
    generateOccupancyMap( frame, context.getVideoOccupancyMap().getFrame( frame.getIndex() ), occupancyPrecision,
                          thresholdLossyOM, enhancedOccupancyMapForDepthFlag );
  }
}

void PCCCodec::generateOccupancyMap( PCCFrameContext&            frame,
                                     const PCCImageOccupancyMap& videoFrame,
                                     const size_t                occupancyPrecision,
                                     const size_t                thresholdLossyOM,
                                     const bool                  enhancedOccupancyMapForDepthFlag ) {
  auto width        = frame.getWidth();
  auto height       = frame.getHeight();
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

void PCCCodec::generateBlockToPatchFromOccupancyMap( PCCContext&  context, const size_t occupancyResolution ) {
  size_t sizeFrames = context.getFrames().size();
  for ( int i = 0; i < sizeFrames; i++ ) {
    PCCFrameContext& frame = context.getFrames()[i];
    generateBlockToPatchFromOccupancyMap( context, frame, i, occupancyResolution );
  }
}

void PCCCodec::generateBlockToPatchFromOccupancyMap( PCCContext&  context,
                                                     PCCFrameContext& frame,
                                                     size_t           frameIndex,
                                                     const size_t     occupancyResolution ) {
  auto&        patches            = frame.getPatches();
  const size_t patchCount         = patches.size();
  const size_t blockToPatchWidth  = frame.getWidth() / occupancyResolution;
  const size_t blockToPatchHeight = frame.getHeight() / occupancyResolution;
  const size_t blockCount         = blockToPatchWidth * blockToPatchHeight;
  auto&        blockToPatch       = frame.getBlockToPatch();
  const auto&  occupancyMap       = frame.getOccupancyMap();
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
            size_t       x, y;
            nonZeroPixel +=
                ( occupancyMap[patch.patch2Canvas( u, v, frame.getWidth(), frame.getHeight(), x, y )] != 0 );
          }  // u1
        }    // v1
        if ( nonZeroPixel > 0 ) {
          if ( ( context.getSps().getPatchPrecedenceOrderFlag() == 0 ) || blockToPatch[blockIndex] == 0 ) {
            blockToPatch[blockIndex] = patchIndex + 1;
          }
				}
      }  // u0
    }    // v0
  }      // patch
}

void PCCCodec::generateBlockToPatchFromBoundaryBox( PCCContext& context, const size_t occupancyResolution ) {
  size_t sizeFrames = context.getFrames().size();
  for ( int i = 0; i < sizeFrames; i++ ) {
    PCCFrameContext& frame = context.getFrames()[i];
    generateBlockToPatchFromBoundaryBox( context, frame, i, occupancyResolution );
  }
}

void PCCCodec::generateBlockToPatchFromBoundaryBox( PCCContext&  context,
                                                    PCCFrameContext& frame,
                                                    size_t           frameIndex,
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
        const size_t blockIndex  = patch.patchBlock2CanvasBlock( u0, v0, blockToPatchWidth, blockToPatchHeight );
        if ( ( context.getSps().getPatchPrecedenceOrderFlag() == 0 ) || blockToPatch[blockIndex] == 0 ) {
          blockToPatch[blockIndex] = patchIndex + 1;
        }
      }  // u0
    }    // v0
  }      // patch
}

void PCCCodec::generateBlockToPatchFromOccupancyMapVideo(PCCContext&  context,
	const bool   losslessGeo,
	const bool   lossyMissedPointsPatch,
	const size_t occupancyResolution,
	const size_t occupancyPrecision) {
	size_t sizeFrames = context.getFrames().size();
	for (int i = 0; i < sizeFrames; i++) {
		PCCFrameContext& frame = context.getFrames()[i];
		PCCImageOccupancyMap &occupancyImage = context.getVideoOccupancyMap().getFrame(i);
    generateBlockToPatchFromOccupancyMapVideo( context, frame, occupancyImage, i, occupancyResolution,
                                               occupancyPrecision );
	}
}

void PCCCodec::generateBlockToPatchFromOccupancyMapVideo( PCCContext&  context,
                                                          PCCFrameContext& frame, 
                                                          PCCImageOccupancyMap &occupancyMapImage,
                                                          size_t           frameIndex,
                                                          const size_t     occupancyResolution, 
                                                          const size_t occupancyPrecision) {
	auto&        patches = frame.getPatches();
	const size_t patchCount = patches.size();
	const size_t blockToPatchWidth = frame.getWidth() / occupancyResolution;
	const size_t blockToPatchHeight = frame.getHeight() / occupancyResolution;
	const size_t blockCount = blockToPatchWidth * blockToPatchHeight;
	auto&        blockToPatch = frame.getBlockToPatch();
	//const auto&  occupancyMap = frame.getOccupancyMap();
	blockToPatch.resize(blockCount);
	std::fill(blockToPatch.begin(), blockToPatch.end(), 0);
	for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
		auto&  patch = patches[patchIndex];
		size_t nonZeroPixel = 0;
		for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
			for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
				const size_t blockIndex = patch.patchBlock2CanvasBlock(u0, v0, blockToPatchWidth, blockToPatchHeight);
				nonZeroPixel = 0;
				for (size_t v1 = 0; v1 < patch.getOccupancyResolution(); ++v1) {
					const size_t v = v0 * patch.getOccupancyResolution() + v1;
					for (size_t u1 = 0; u1 < patch.getOccupancyResolution(); ++u1) {
						const size_t u = u0 * patch.getOccupancyResolution() + u1;
						size_t       x, y;
						patch.patch2Canvas(u, v, frame.getWidth(), frame.getHeight(), x, y);
						nonZeroPixel += (occupancyMapImage.getValue(0, x / occupancyPrecision, y / occupancyPrecision) != 0);
					}  // u1
				}    // v1
				if (nonZeroPixel > 0) { 
          if ( ( context.getSps().getPatchPrecedenceOrderFlag() == 0 ) || blockToPatch[blockIndex] == 0 ) {
            blockToPatch[blockIndex] = patchIndex + 1;
          }
                                }
			}  // u0
		}    // v0
	}      // patch
}
