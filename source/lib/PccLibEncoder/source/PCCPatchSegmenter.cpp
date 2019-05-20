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

#include "PCCKdTree.h"
#include "PCCNormalsGenerator.h"
#include "tbb/tbb.h"
#include "PCCPatchSegmenter.h"
#include "PCCPatch.h"

using namespace pcc;

void PCCPatchSegmenter3::setNbThread( size_t nbThread ) {
  nbThread_ = nbThread;
  if ( nbThread_ > 0 ) { tbb::task_scheduler_init init( (int)nbThread ); }
}

void PCCPatchSegmenter3::compute( const PCCPointSet3&                 geometry,
                                  const PCCPatchSegmenter3Parameters& params,
                                  std::vector<PCCPatch>&              patches,
                                  std::vector<PCCPointSet3>&          subPointCloud,
                                  float&                              distanceSrcRec ) {
  PCCVector3D* orientations     = 0;
  size_t       orientationCount = 0;
  if ( params.additionalProjectionPlaneMode == 0 ) {
    orientations     = orientations6;
    orientationCount = orientationCount6;
  } else if ( params.additionalProjectionPlaneMode == 1 ) {
    orientations     = orientations10_YAxis;
    orientationCount = 10;
  } else if ( params.additionalProjectionPlaneMode == 2 ) {
    orientations     = orientations10_XAxis;
    orientationCount = 10;
  } else if ( params.additionalProjectionPlaneMode == 3 ) {
    orientations     = orientations10_ZAxis;
    orientationCount = 10;
  } else if ( params.additionalProjectionPlaneMode == 4 ) {
    orientations     = orientations18;
    orientationCount = 18;
  }
  std::cout << "  Computing normals for original point cloud... ";
  PCCKdTree                            kdtree( geometry );
  PCCNNResult                          result;
  PCCNormalsGenerator3                 normalsGen;
  const PCCNormalsGenerator3Parameters normalsGenParams = {PCCVector3D( 0.0 ),
                                                           ( std::numeric_limits<double>::max )(),
                                                           ( std::numeric_limits<double>::max )(),
                                                           ( std::numeric_limits<double>::max )(),
                                                           ( std::numeric_limits<double>::max )(),
                                                           params.nnNormalEstimation,
                                                           params.nnNormalEstimation,
                                                           params.nnNormalEstimation,
                                                           0,
                                                           PCC_NORMALS_GENERATOR_ORIENTATION_SPANNING_TREE,
                                                           false,
                                                           false,
                                                           false};
  normalsGen.compute( geometry, kdtree, normalsGenParams, nbThread_ );
  std::cout << "[done]" << std::endl;

  std::cout << "  Computing initial segmentation... ";
  std::vector<size_t> partition;
  if ( params.additionalProjectionPlaneMode == 0 ) {
    initialSegmentation( geometry, normalsGen, orientations, orientationCount, partition, params.weightNormal );
  } else {
    initialSegmentation( geometry, normalsGen, orientations, orientationCount, partition );  // flat weight
  }
  std::cout << "[done]" << std::endl;

  std::cout << "  Refining segmentation... ";
  refineSegmentation( geometry, kdtree, normalsGen, orientations, orientationCount, params.maxNNCountRefineSegmentation,
                      params.lambdaRefineSegmentation, params.iterationCountRefineSegmentation, partition );
  std::cout << "[done]" << std::endl;

  std::cout << "  Patch segmentation... ";
  PCCPointSet3        resampled;
  std::vector<size_t> patchPartition;
  std::vector<size_t> resampledPatchPartition;
  std::vector<size_t> missedPoints;

  segmentPatches( geometry, kdtree, params.maxNNCountPatchSegmentation, params.minPointCountPerCCPatchSegmentation,
                  params.occupancyResolution, params.maxAllowedDist2MissedPointsDetection,
                  params.maxAllowedDist2MissedPointsSelection, params.surfaceThickness, params.maxAllowedDepth,
                  params.minLevel, partition, patches, patchPartition, resampledPatchPartition, missedPoints, resampled,
                  params.projectionMode, params.useEnhancedDeltaDepthCode, params.useOneLayermode, subPointCloud,
                  distanceSrcRec, params.absoluteD1, params.surfaceSeparation, params.additionalProjectionPlaneMode,
                  params.geometryBitDepth3D );
  std::cout << "[done]" << std::endl;
}

void PCCPatchSegmenter3::initialSegmentation( const PCCPointSet3&         geometry,
                                              const PCCNormalsGenerator3& normalsGen,
                                              const PCCVector3D*          orientations,
                                              const size_t                orientationCount,
                                              std::vector<size_t>&        partition ) {
  PCCVector3D axis_weight;
  axis_weight[0] = axis_weight[1] = axis_weight[2] = 1.0f;
  initialSegmentation( geometry, normalsGen, orientations, orientationCount, partition, axis_weight );
}
void PCCPatchSegmenter3::initialSegmentation( const PCCPointSet3&         geometry,
                                              const PCCNormalsGenerator3& normalsGen,
                                              const PCCVector3D*          orientations,
                                              const size_t                orientationCount,
                                              std::vector<size_t>&        partition,
                                              const PCCVector3D           axis_weight ) {
  assert( orientations );
  const size_t pointCount = geometry.getPointCount();
  partition.resize( pointCount );

  printf( "\n [weight] YZ, XZ, XY: %8.5f, %8.5f, %8.5f\n", axis_weight[0], axis_weight[1], axis_weight[2] );
  double weight_val[18];

  int i;
  for ( i = 0; i < 18; i++ ) { weight_val[i] = 1.0f; }
  weight_val[0] = weight_val[3] = axis_weight[0];
  weight_val[1] = weight_val[4] = axis_weight[1];
  weight_val[2] = weight_val[5] = axis_weight[2];

  tbb::task_arena limited( (int)nbThread_ );
  limited.execute( [&] {
    tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) {
      const PCCVector3D normal       = normalsGen.getNormal( i );
      size_t            clusterIndex = 0;
      double            bestScore    = normal * orientations[0];
      for ( size_t j = 1; j < orientationCount; ++j ) {
        // const double score = normal * orientations[j];
        const double score = normal * orientations[j] * weight_val[j];
        if ( score > bestScore ) {
          bestScore    = score;
          clusterIndex = j;
        }
      }
      partition[i] = clusterIndex;
    } );
  } );
}

void PCCPatchSegmenter3::computeAdjacencyInfo( const PCCPointSet3&               pointCloud,
                                               const PCCKdTree&                  kdtree,
                                               std::vector<std::vector<size_t>>& adj,
                                               const size_t                      maxNNCount ) {
  const size_t pointCount = pointCloud.getPointCount();
  adj.resize( pointCount );
  tbb::task_arena limited( (int)nbThread_ );
  limited.execute( [&] {
    tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) {
      PCCNNResult result;
      kdtree.search( pointCloud[i], maxNNCount, result );
      std::vector<size_t>& neighbors = adj[i];
      neighbors.resize( result.count() );
      for ( size_t j = 0; j < result.count(); ++j ) { neighbors[j] = result.indices( j ); }
    } );
  } );
}
size_t PCCPatchSegmenter3::selectFrameProjectionMode( const PCCPointSet3& points,
                                                      const size_t        surfaceThickness,
                                                      const size_t        minLevel,
                                                      const size_t        paramProjectionMode ) {
  const int16_t infiniteDepth       = ( std::numeric_limits<int16_t>::max )();
  const size_t  boxSize             = size_t( 2047 );
  size_t        frameProjectionMode = 0;
  // allocate patch cube
  boxMinDepths_.resize( 3 );
  boxMaxDepths_.resize( 3 );
  for ( auto& plan : boxMinDepths_ ) {
    plan.getSizeU() = boxSize;
    plan.getSizeV() = boxSize;
    plan.getSizeD() = boxSize;
    plan.getD1()    = ( std::numeric_limits<int16_t>::max )();
    plan.getU1()    = size_t( 0 );
    plan.getV1()    = size_t( 0 );
    plan.getDepth( 0 ).resize( plan.getSizeU() * plan.getSizeV(), infiniteDepth );
    plan.getDepth( 1 ).resize( plan.getSizeU() * plan.getSizeV(), 0 );
  }
  for ( auto& plan : boxMaxDepths_ ) {
    plan.getSizeU() = boxSize;
    plan.getSizeV() = boxSize;
    plan.getSizeD() = boxSize;
    plan.getD1()    = ( std::numeric_limits<int16_t>::max )();
    plan.getU1()    = size_t( 0 );
    plan.getV1()    = size_t( 0 );
    plan.getDepth( 0 ).resize( plan.getSizeU() * plan.getSizeV(), 0 );
    plan.getDepth( 1 ).resize( plan.getSizeU() * plan.getSizeV(), infiniteDepth );
  }
  boxMinDepths_[0].getNormalAxis()    = 0;  // min depth  projection along x axis towards (z,y) plan
  boxMinDepths_[0].getTangentAxis()   = 2;
  boxMinDepths_[0].getBitangentAxis() = 1;

  boxMinDepths_[1].getNormalAxis()    = 1;  //  min depth projection along y axis towards (z,x) plan
  boxMinDepths_[1].getTangentAxis()   = 2;
  boxMinDepths_[1].getBitangentAxis() = 0;

  boxMinDepths_[2].getNormalAxis()    = 2;  //  min depth projection along z axis towards (x,y) plan
  boxMinDepths_[2].getTangentAxis()   = 0;
  boxMinDepths_[2].getBitangentAxis() = 1;

  boxMaxDepths_[0].getNormalAxis()    = 0;  //  max depth projection along x axis towards (z,y) plan
  boxMaxDepths_[0].getTangentAxis()   = 2;
  boxMaxDepths_[0].getBitangentAxis() = 1;

  boxMaxDepths_[1].getNormalAxis()    = 1;  //  max depth projection along y axis towards (z,x) plan
  boxMaxDepths_[1].getTangentAxis()   = 2;
  boxMaxDepths_[1].getBitangentAxis() = 0;

  boxMaxDepths_[2].getNormalAxis()    = 2;  //  max depth projection along z axis towards (x,y) plan
  boxMaxDepths_[2].getTangentAxis()   = 0;
  boxMaxDepths_[2].getBitangentAxis() = 1;

  // projection keeping the minimum depth
  for ( auto& plan : boxMinDepths_ ) {
    std::fill( plan.getDepth( 0 ).begin(), plan.getDepth( 0 ).end(), infiniteDepth );
    for ( int i = 0; i < points.getPointCount(); i++ ) {
      const auto&   point = points[i];
      const int16_t d     = int16_t( round( point[plan.getNormalAxis()] ) );
      const size_t  u     = size_t( round( point[plan.getTangentAxis()] - plan.getU1() ) );
      const size_t  v     = size_t( round( point[plan.getBitangentAxis()] - plan.getV1() ) );
      assert( u >= 0 && u < plan.getSizeU() );
      assert( v >= 0 && v < plan.getSizeV() );
      const size_t p = v * plan.getSizeU() + u;
      if ( plan.getDepth( 0 )[p] > d ) {
        plan.getDepth( 0 )[p] = d;
        plan.getD1()          = ( std::min )( plan.getD1(), size_t( minLevel * ( d / minLevel ) ) );
      }
    }
  }
  // projection keeping the maximum depth
  for ( auto& plan : boxMaxDepths_ ) {
    std::fill( plan.getDepth( 0 ).begin(), plan.getDepth( 0 ).end(), 0 );
    for ( int i = 0; i < points.getPointCount(); i++ ) {
      const auto&   point = points[i];
      const int16_t d     = int16_t( round( point[plan.getNormalAxis()] ) );
      const size_t  u     = size_t( round( point[plan.getTangentAxis()] - plan.getU1() ) );
      const size_t  v     = size_t( round( point[plan.getBitangentAxis()] - plan.getV1() ) );
      assert( u >= 0 && u < plan.getSizeU() );
      assert( v >= 0 && v < plan.getSizeV() );
      const size_t p = v * plan.getSizeU() + u;
      if ( plan.getDepth( 0 )[p] < d ) {
        plan.getDepth( 0 )[p] = d;
        plan.getD1()          = ( std::max )( plan.getD1(), size_t( minLevel * ( d / minLevel ) ) );
      }
    }
  }

  // find a global criterion to apply or not the adaptive selection of the projection direction at patch level
  // compute depth1
  for ( auto& plan : boxMinDepths_ ) {
    plan.getDepth( 1 ) = plan.getDepth( 0 );
    for ( int i = 0; i < points.getPointCount(); i++ ) {
      const auto&   point = points[i];
      const int16_t d     = int16_t( round( point[plan.getNormalAxis()] ) );
      const int64_t u     = int64_t( round( point[plan.getTangentAxis()] - plan.getU1() ) );
      const int64_t v     = int64_t( round( point[plan.getBitangentAxis()] - plan.getV1() ) );
      if ( u < 0 || u >= plan.getSizeU() )
        std::cout << " u < 0 || u >= plan.getSizeU() ! (u,v,d) = (" << u << "," << v << "," << d << ")." << std::endl;
      if ( v < 0 || v >= plan.getSizeV() )
        std::cout << " u < 0 || u >= plan.getSizeV() ! (u,v,d) = (" << u << "," << v << "," << d << ")." << std::endl;
      assert( u >= 0 && u < plan.getSizeU() );
      assert( v >= 0 && v < plan.getSizeV() );
      const size_t  p      = v * plan.getSizeU() + u;
      const int16_t depth0 = plan.getDepth( 0 )[p];

      if ( depth0 < infiniteDepth && ( d - depth0 ) <= int16_t( surfaceThickness ) && d > plan.getDepth( 1 )[p] ) {
        plan.getDepth( 1 )[p] = d;
      }
    }
  }

  size_t cptD0[3];
  size_t cptD1[3];
  size_t j = 0;
  for ( auto& plan : boxMinDepths_ ) {
    cptD0[j] = 0;
    cptD1[j] = 0;
    for ( int i = 0; i < plan.getSizeU() * plan.getSizeV(); i++ ) {
      if ( plan.getDepth( 0 )[i] < infiniteDepth ) { cptD0[j]++; }
      if ( plan.getDepth( 0 )[i] < infiniteDepth && plan.getDepth( 0 )[i] != plan.getDepth( 1 )[i] ) { cptD1[j]++; }
    }
    j++;
  }

  double ratioD1D0          = double( cptD1[0] + cptD1[1] + cptD1[2] ) / double( cptD0[0] + cptD0[1] + cptD0[2] );
  double ratioD1D0Threshold = 0.8;
  if ( paramProjectionMode == 3 ) {
    ratioD1D0Threshold = 0.0;
  } else if ( paramProjectionMode == 2 ) {
    if ( ratioD1D0 < ratioD1D0Threshold ) {
      frameProjectionMode = 0;  // min for all patches
    } else {
      frameProjectionMode = 2;  // adaptive projection mode at patch level
    }
  } else if ( paramProjectionMode == 3 ) {
    frameProjectionMode = 2;  // adaptive projection mode at patch level
  }
  return frameProjectionMode;
}
void PCCPatchSegmenter3::selectPatchProjectionMode( const PCCPointSet3&  points,
                                                    size_t               frameProjectionMode,
                                                    std::vector<size_t>& connectedComponent,
                                                    PCCPatch&            patch ) {
  const size_t boxSize = size_t( 2047 );
  if ( frameProjectionMode == 0 ) {
    patch.getProjectionMode()      = 0;  // min for all patches
    patch.getFrameProjectionMode() = 0;
  } else if ( frameProjectionMode == 1 ) {
    patch.getProjectionMode()      = 1;  // max for all patches
    patch.getFrameProjectionMode() = 1;
  } else if ( ( frameProjectionMode == 2 ) || ( frameProjectionMode == 4 ) ) {
    // selection of projectionMode per patch
    patch.getFrameProjectionMode() = 2;
    uint16_t cptMinD0[3]           = {0, 0, 0};
    uint16_t cptMaxD0[3]           = {0, 0, 0};
    for ( const auto i : connectedComponent ) {
      const auto&   point = points[i];
      const int16_t d     = int16_t( round( point[patch.getNormalAxis()] ) );
      const size_t  u     = size_t( round( point[patch.getTangentAxis()] ) );
      const size_t  v     = size_t( round( point[patch.getBitangentAxis()] ) );
      assert( u >= 0 && u < patch.getSizeU() );
      assert( v >= 0 && v < patch.getSizeV() );
      assert( d >= 0 );
      const size_t p = v * boxSize + u;
      if ( boxMinDepths_[patch.getNormalAxis()].getDepth( 0 )[p] == d ) { cptMinD0[patch.getNormalAxis()]++; }
      if ( boxMaxDepths_[patch.getNormalAxis()].getDepth( 0 )[p] == d ) { cptMaxD0[patch.getNormalAxis()]++; }
    }

    // select best projection's mode
#define PROJ_MODE_IMPROVEMENT 0
#if PROJ_MODE_IMPROVEMENT == 1
    size_t patchSize = patch.getSizeU() * patch.getSizeV();
    if ( patch.getViewId() >= 3 ) {
      patch.getProjectionMode() = 1;
      if ( cptMinD0[patch.getNormalAxis()] - cptMaxD0[patch.getNormalAxis()] >= 0 ) { patch.getProjectionMode() = 0; }
    } else {
      patch.getProjectionMode() = 0;
      if ( cptMaxD0[patch.getNormalAxis()] - cptMinD0[patch.getNormalAxis()] >= 0 ) {
          patch.getProjectionMode() = 1;
        }
    }
    // std::cout << "(dir, cptMinD0, cptMinD1, projMode) = ( " <<
#else
    if ( cptMinD0[patch.getNormalAxis()] >= cptMaxD0[patch.getNormalAxis()] ) {
      patch.getProjectionMode() = 0;
    } else {
      patch.getProjectionMode() = 1;
    }
#endif
  }

  if ( patch.getProjectionMode() == 1 ) { patch.getDepth( 1 ).resize( patch.getSizeU() * patch.getSizeV(), 0 ); }
}

void PCCPatchSegmenter3::segmentPatches( const PCCPointSet3&        points,
                                         const PCCKdTree&           kdtree,
                                         const size_t               maxNNCount,
                                         const size_t               minPointCountPerCC,
                                         const size_t               occupancyResolution,
                                         const double               maxAllowedDist2MissedPointsDetection,
                                         const double               maxAllowedDist2MissedPointsSelection,
                                         const size_t               surfaceThickness,
                                         const size_t               maxAllowedDepth,
                                         const size_t               minLevel,
                                         const std::vector<size_t>& partition,
                                         std::vector<PCCPatch>&     patches,
                                         std::vector<size_t>&       patchPartition,
                                         std::vector<size_t>&       resampledPatchPartition,
                                         std::vector<size_t>        missedPoints,
                                         PCCPointSet3&              resampled,
                                         const size_t               paramProjectionMode,
                                         bool                       useEnhancedDeltaDepthCode,
                                         const bool                 createSubPointCloud,
                                         std::vector<PCCPointSet3>& subPointCloud,
                                         float&                     distanceSrcRec,
                                         const bool                 sixDirection,  // params.absoluteD1
                                         bool                       useSurfaceSeparation,
                                         const size_t               additionalProjectionAxis,
                                         const size_t               geometryBitDepth3D ) {
  const size_t pointCount = points.getPointCount();
  patchPartition.resize( pointCount, 0 );
  resampledPatchPartition.reserve( pointCount );

  PCCNNResult             result;
  const double            Threshold_Color_Error = 400;
  std::vector<PCCColor3B> frame_pcc_color;
  PCCColor3B              RGB_val;

  if ( useSurfaceSeparation ) {
    frame_pcc_color.reserve( pointCount );
    for ( size_t i = 0; i < pointCount; i++ ) {
      RGB_val = points.getColor( i );
      frame_pcc_color.push_back( RGB_val );
    }
  }

  std::cout << "\n\t Computing adjacency info... ";
  std::vector<std::vector<size_t>> adj;
  computeAdjacencyInfo( points, kdtree, adj, maxNNCount );
  std::cout << "[done]" << std::endl;

  std::cout << "\n\t Extracting patches... " << std::endl;
  std::vector<double> missedPointsDistance;
  missedPoints.resize( pointCount );
  missedPointsDistance.resize( pointCount );
  for ( size_t i = 0; i < pointCount; ++i ) {
    missedPoints[i]         = i;
    missedPointsDistance[i] = ( std::numeric_limits<double>::max )();
  }
  subPointCloud.clear();
  double meanPAB = 0.0, meanYAB = 0.0, meanUAB = 0.0, meanVAB = 0.0;
  double meanPBA = 0.0, meanYBA = 0.0, meanUBA = 0.0, meanVBA = 0.0;
  size_t testSrcNum = 0, testRecNum = 0;

  size_t frameProjectionMode = 0;
  if ( !sixDirection ) {
    if ( paramProjectionMode == 0 ) {
      frameProjectionMode = 0;  // max for all patches
    } else if ( paramProjectionMode == 1 ) {
      frameProjectionMode = 1;  // max for all patches
    } else {
      frameProjectionMode = selectFrameProjectionMode( points, surfaceThickness, minLevel, paramProjectionMode );
    }
  }
  uint16_t maxD = 1 << geometryBitDepth3D;
  while ( !missedPoints.empty() ) {
    std::vector<size_t> fifo;
    fifo.reserve( pointCount );
    std::vector<bool>                flags;
    std::vector<std::vector<size_t>> connectedComponents;
    flags.resize( pointCount, false );
    for ( const auto i : missedPoints ) { flags[i] = true; }
    connectedComponents.reserve( 256 );
    for ( const auto i : missedPoints ) {
      if ( flags[i] && missedPointsDistance[i] > maxAllowedDist2MissedPointsDetection ) {
        flags[i]                  = false;
        const size_t indexCC      = connectedComponents.size();
        const size_t clusterIndex = partition[i];
        connectedComponents.resize( indexCC + 1 );
        std::vector<size_t>& connectedComponent = connectedComponents[indexCC];
        fifo.push_back( i );
        connectedComponent.push_back( i );
        while ( !fifo.empty() ) {
          const size_t current = fifo.back();
          fifo.pop_back();
          for ( const auto n : adj[current] ) {
            if ( clusterIndex == partition[n] && flags[n] ) {
              flags[n] = false;
              fifo.push_back( n );
              connectedComponent.push_back( n );
            }
          }
        }
        if ( connectedComponent.size() < minPointCountPerCC ) {
          connectedComponents.resize( indexCC );
        } else {
          std::cout << "\t\t CC " << indexCC << " -> " << connectedComponent.size() << std::endl;
        }
      }
    }

    std::cout << " # CC " << connectedComponents.size() << std::endl;
    if ( connectedComponents.empty() ) { break; }
    bool bIsAdditionalProjectionPlane;
    for ( auto& connectedComponent : connectedComponents ) {
      const size_t patchIndex = patches.size();
      patches.resize( patchIndex + 1 );
      if ( createSubPointCloud ) { subPointCloud.resize( patchIndex + 1 ); }
      PCCPatch& patch  = patches[patchIndex];
      patch.getIndex() = patchIndex;
      patch.setPatchType( (uint8_t)P_TYPE_INTRA );
      const size_t clusterIndex        = partition[connectedComponent[0]];
      bIsAdditionalProjectionPlane     = false;
      patch.getAxisOfAdditionalPlane() = 0;
      if ( clusterIndex == 0 || clusterIndex == 3 ) {
        patch.getNormalAxis()    = 0;
        patch.getTangentAxis()   = 2;
        patch.getBitangentAxis() = 1;
      } else if ( clusterIndex == 1 || clusterIndex == 4 ) {
        patch.getNormalAxis()    = 1;
        patch.getTangentAxis()   = 2;
        patch.getBitangentAxis() = 0;
      } else if ( clusterIndex == 2 || clusterIndex == 5 ) {
        patch.getNormalAxis()    = 2;
        patch.getTangentAxis()   = 0;
        patch.getBitangentAxis() = 1;
      } else if ( additionalProjectionAxis == 1 ) {  // Y Axis
        bIsAdditionalProjectionPlane     = true;
        patch.getAxisOfAdditionalPlane() = additionalProjectionAxis;  // Y Axis
        if ( clusterIndex == 6 || clusterIndex == 8 ) {               // 6->0 : 8->3
          patch.getNormalAxis()    = 0;
          patch.getTangentAxis()   = 2;
          patch.getBitangentAxis() = 1;
        } else if ( clusterIndex == 7 || clusterIndex == 9 ) {  // 7->2 : 9->5
          patch.getNormalAxis()    = 2;
          patch.getTangentAxis()   = 0;
          patch.getBitangentAxis() = 1;
        }
      } else if ( additionalProjectionAxis == 2 ) {  // X Axis
        bIsAdditionalProjectionPlane     = true;
        patch.getAxisOfAdditionalPlane() = additionalProjectionAxis;  // X Axis
        if ( clusterIndex == 6 || clusterIndex == 8 ) {               // 6->2 : 8->5
          patch.getNormalAxis()    = 2;
          patch.getTangentAxis()   = 0;
          patch.getBitangentAxis() = 1;
        } else if ( clusterIndex == 7 || clusterIndex == 9 ) {  // 7->1 : 9->4
          patch.getNormalAxis()    = 1;
          patch.getTangentAxis()   = 2;
          patch.getBitangentAxis() = 0;
        }
      } else if ( additionalProjectionAxis == 3 ) {  // Z Axis
        bIsAdditionalProjectionPlane     = true;
        patch.getAxisOfAdditionalPlane() = additionalProjectionAxis;  // Y Zxis
        if ( clusterIndex == 6 || clusterIndex == 8 ) {               // 6->0 : 8->3
          patch.getNormalAxis()    = 1;
          patch.getTangentAxis()   = 2;
          patch.getBitangentAxis() = 0;
        } else if ( clusterIndex == 7 || clusterIndex == 9 ) {  // 7->2 : 9->1
          patch.getNormalAxis()    = 0;
          patch.getTangentAxis()   = 2;
          patch.getBitangentAxis() = 1;
        }
      } else if ( additionalProjectionAxis == 4 ) {  // All
        bIsAdditionalProjectionPlane = true;
        if ( clusterIndex == 6 || clusterIndex == 8 ) {  // 6->0 : 8->3
          patch.getAxisOfAdditionalPlane() = 1;          // Y Axis
          patch.getNormalAxis()            = 0;
          patch.getTangentAxis()           = 2;
          patch.getBitangentAxis()         = 1;
        } else if ( clusterIndex == 7 || clusterIndex == 9 ) {  // 7->2 : 9->1
          patch.getAxisOfAdditionalPlane() = 1;                 // Y Axis
          patch.getNormalAxis()            = 2;
          patch.getTangentAxis()           = 0;
          patch.getBitangentAxis()         = 1;
        } else if ( clusterIndex == 10 || clusterIndex == 12 ) {  // 6->2 : 8->5
          patch.getAxisOfAdditionalPlane() = 2;                   // X Axis
          patch.getNormalAxis()            = 2;
          patch.getTangentAxis()           = 0;
          patch.getBitangentAxis()         = 1;
        } else if ( clusterIndex == 11 || clusterIndex == 13 ) {  // 7->1 : 9->4
          patch.getAxisOfAdditionalPlane() = 2;                   // X Axis
          patch.getNormalAxis()            = 1;
          patch.getTangentAxis()           = 2;
          patch.getBitangentAxis()         = 0;
        } else if ( clusterIndex == 14 || clusterIndex == 16 ) {  // 6->0 : 8->3
          patch.getAxisOfAdditionalPlane() = 3;                   // Z Axis
          patch.getNormalAxis()            = 1;
          patch.getTangentAxis()           = 2;
          patch.getBitangentAxis()         = 0;
        } else if ( clusterIndex == 15 || clusterIndex == 17 ) {  // 7->2 : 9->1
          patch.getAxisOfAdditionalPlane() = 3;                   // Z Axis
          patch.getNormalAxis()            = 0;
          patch.getTangentAxis()           = 2;
          patch.getBitangentAxis()         = 1;
        }
      }

      patch.setViewId()       = clusterIndex;
      patch.setBestMatchIdx() = -1;

      patch.getPreGPAPatchData().initialize();
      patch.getCurGPAPatchData().initialize();

      if ( sixDirection ) {
        if ( clusterIndex <= 2 ) {
          patch.getProjectionMode() = 0;
        } else {
          patch.getProjectionMode() = 1;
        }
        // for additional projection plane
        if ( clusterIndex == 6 || clusterIndex == 7 ) { patch.getProjectionMode() = 0; }
        // for additional projection plane
        if ( clusterIndex == 10 || clusterIndex == 11 || clusterIndex == 14 || clusterIndex == 15 ) {
          patch.getProjectionMode() = 0;
        }
      }

      PCCBox3D boundingBox;
      for ( size_t k = 0; k < 3; ++k ) {
        boundingBox.min_[k] = points[connectedComponent[0]][k];
        boundingBox.max_[k] = points[connectedComponent[0]][k];
      }
      for ( const auto i : connectedComponent ) {
        patchPartition[i]   = patchIndex + 1;
        PCCPoint3D pointTmp = points[i];
        if ( bIsAdditionalProjectionPlane ) {
          auto& input = pointTmp;
          convert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, pointTmp );
        }
        const auto& point = pointTmp;
        for ( size_t k = 0; k < 3; ++k ) {
          if ( point[k] < boundingBox.min_[k] ) { boundingBox.min_[k] = floor( point[k] ); }
          if ( point[k] > boundingBox.max_[k] ) { boundingBox.max_[k] = ceil( point[k] ); }
        }
      }

      const int16_t infiniteDepth  = ( std::numeric_limits<int16_t>::max )();
      const int64_t infinitenumber = ( std::numeric_limits<int64_t>::max )();
      // patch.getSizeU() = 1 + size_t(boundingBox.max_[patch.getTangentAxis()] -
      // boundingBox.min_[patch.getTangentAxis()]); patch.getSizeV() = 1 +
      // size_t(boundingBox.max_[patch.getBitangentAxis()] - boundingBox.min_[patch.getBitangentAxis()]);
      patch.getSizeU() = 1 + size_t( round( boundingBox.max_[patch.getTangentAxis()] ) -
                                     floor( boundingBox.min_[patch.getTangentAxis()] ) );
      patch.getSizeV() = 1 + size_t( round( boundingBox.max_[patch.getBitangentAxis()] ) -
                                     floor( boundingBox.min_[patch.getBitangentAxis()] ) );
      patch.getU1()    = size_t( boundingBox.min_[patch.getTangentAxis()] );
      patch.getV1()    = size_t( boundingBox.min_[patch.getBitangentAxis()] );
      if ( !sixDirection ) {
        patch.getD1() = infiniteDepth;
      } else {
        if ( patch.getProjectionMode() == 0 ) {
          patch.getD1() = infiniteDepth;
        } else {
          patch.getD1() = 0;
        }
      }
      patch.getDepth( 0 ).resize( patch.getSizeU() * patch.getSizeV(), infiniteDepth );
      if ( useSurfaceSeparation ) {
        patch.getdepth0pccidx().resize( patch.getSizeU() * patch.getSizeV(), infinitenumber );
      }
      patch.getLod() = 0;
      if ( useEnhancedDeltaDepthCode ) {
        patch.getDepthEnhancedDeltaD().resize( patch.getSizeU() * patch.getSizeV(), 0 );
      }

      patch.getOccupancyResolution() = occupancyResolution;
      patch.getSizeU0()              = 0;
      patch.getSizeV0()              = 0;
      if ( !sixDirection ) { selectPatchProjectionMode( points, frameProjectionMode, connectedComponent, patch ); }

      for ( const auto i : connectedComponent ) {
        PCCPoint3D pointTmp = points[i];
        if ( bIsAdditionalProjectionPlane ) {
          auto& input = pointTmp;
          convert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, pointTmp );
        }
        const auto&   point = pointTmp;
        const int16_t d     = int16_t( round( point[patch.getNormalAxis()] ) );
        const size_t  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
        const size_t  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
        assert( u >= 0 && u < patch.getSizeU() );
        assert( v >= 0 && v < patch.getSizeV() );
        const size_t p = v * patch.getSizeU() + u;
        if ( patch.getProjectionMode() == 0 ) {  // min
          if ( patch.getDepth( 0 )[p] > d ) {
            patch.getDepth( 0 )[p] = d;
            if ( useSurfaceSeparation ) { patch.getdepth0pccidx()[p] = i; }
            patch.getSizeU0() = ( std::max )( patch.getSizeU0(), u / patch.getOccupancyResolution() );
            patch.getSizeV0() = ( std::max )( patch.getSizeV0(), v / patch.getOccupancyResolution() );
            patch.getD1()     = ( std::min )( patch.getD1(), size_t( d ) );
            size_t value      = patch.getD1();  // patch.getD1();
            value             = ( value / minLevel );
            patch.getD1()     = value * minLevel;
          }
        } else {  // max
          if ( !sixDirection ) {
            if ( patch.getDepth( 1 )[p] < d ) {  // getDepth(1) is used to store the max value
              patch.getDepth( 1 )[p] = d;
              patch.getDepth( 0 )[p] = d;
              if ( useSurfaceSeparation ) { patch.getdepth0pccidx()[p] = i; }
              patch.getSizeU0() = ( std::max )( patch.getSizeU0(), u / patch.getOccupancyResolution() );
              patch.getSizeV0() = ( std::max )( patch.getSizeV0(), v / patch.getOccupancyResolution() );
              patch.getD1()     = ( std::min )( patch.getD1(), size_t( d ) );
              size_t value      = patch.getD1();  // patch.getD1();
              value             = ( value / minLevel );
              patch.getD1()     = value * minLevel;
            }
          } else {
            if ( patch.getDepth( 0 )[p] == infiniteDepth ) {
              patch.getDepth( 0 )[p] = d;
              if ( useSurfaceSeparation ) patch.getdepth0pccidx()[p] = i;
              patch.getSizeU0() = ( std::max )( patch.getSizeU0(), u / patch.getOccupancyResolution() );
              patch.getSizeV0() = ( std::max )( patch.getSizeV0(), v / patch.getOccupancyResolution() );
              patch.getD1()     = ( std::max )( patch.getD1(), size_t( d ) );
              size_t value      = size_t( patch.getD1() / minLevel );
              if ( value * minLevel < patch.getD1() )
                patch.getD1() = ( 1 + value ) * minLevel;
              else
                patch.getD1() = value * minLevel;
              // std::cout<<"~~~~~~"<<value<<" -> "<<patch.getD1()<<std::endl;
            } else {
              if ( patch.getDepth( 0 )[p] < d ) {
                patch.getDepth( 0 )[p] = d;
                if ( useSurfaceSeparation ) patch.getdepth0pccidx()[p] = i;
                patch.getSizeU0() = ( std::max )( patch.getSizeU0(), u / patch.getOccupancyResolution() );
                patch.getSizeV0() = ( std::max )( patch.getSizeV0(), v / patch.getOccupancyResolution() );
                patch.getD1()     = ( std::max )( patch.getD1(), size_t( d ) );
                size_t value      = size_t( patch.getD1() / minLevel );
                if ( value * minLevel < patch.getD1() )
                  patch.getD1() = ( 1 + value ) * minLevel;
                else
                  patch.getD1() = value * minLevel;
                // std::cout<<"*******"<<value<<" -> "<<patch.getD1()<<std::endl;
              }
            }
          }
        }
      }

      ++patch.getSizeU0();
      ++patch.getSizeV0();
      patch.getOccupancy().resize( patch.getSizeU0() * patch.getSizeV0(), false );
      // filter depth
      if ( patch.getProjectionMode() == 0 ) {  // min
        std::vector<int16_t> minPerBlock;
        minPerBlock.resize( patch.getSizeU0() * patch.getSizeV0(), infiniteDepth );
        for ( int64_t v = 0; v < int64_t( patch.getSizeV() ); ++v ) {
          for ( int64_t u = 0; u < int64_t( patch.getSizeU() ); ++u ) {
            const size_t  p        = v * patch.getSizeU() + u;
            const int16_t depth0   = patch.getDepth( 0 )[p];
            const size_t  u0       = u / patch.getOccupancyResolution();
            const size_t  v0       = v / patch.getOccupancyResolution();
            const size_t  p0       = v0 * patch.getSizeU0() + u0;
            const int16_t minDepth = minPerBlock[p0];
            minPerBlock[p0]        = ( std::min )( minDepth, depth0 );
          }
        }
        for ( int64_t v = 0; v < int64_t( patch.getSizeV() ); ++v ) {
          for ( int64_t u = 0; u < int64_t( patch.getSizeU() ); ++u ) {
            const size_t  p        = v * patch.getSizeU() + u;
            const int16_t depth0   = patch.getDepth( 0 )[p];
            const size_t  u0       = u / patch.getOccupancyResolution();
            const size_t  v0       = v / patch.getOccupancyResolution();
            const size_t  p0       = v0 * patch.getSizeU0() + u0;
            const int16_t minDepth = minPerBlock[p0];
            if ( ( depth0 - minDepth > 32 ) || ( surfaceThickness + depth0 > patch.getD1() + maxAllowedDepth ) ) {
              patch.getDepth( 0 )[p] = infiniteDepth;
              if ( useSurfaceSeparation ) { patch.getdepth0pccidx()[p] = infinitenumber; }
            }
          }
        }
      } else {
        std::vector<int16_t> minPerBlock;
        std::vector<int16_t> maxPerBlock;
        maxPerBlock.resize( patch.getSizeU0() * patch.getSizeV0(), 0 );
        minPerBlock.resize( patch.getSizeU0() * patch.getSizeV0(), infiniteDepth );
        for ( int64_t v = 0; v < int64_t( patch.getSizeV() ); ++v ) {
          for ( int64_t u = 0; u < int64_t( patch.getSizeU() ); ++u ) {
            const size_t  p        = v * patch.getSizeU() + u;
            const int16_t depth0   = patch.getDepth( 0 )[p];
            const size_t  u0       = u / patch.getOccupancyResolution();
            const size_t  v0       = v / patch.getOccupancyResolution();
            const size_t  p0       = v0 * patch.getSizeU0() + u0;
            const int16_t minDepth = minPerBlock[p0];
            const int16_t maxDepth = maxPerBlock[p0];
            minPerBlock[p0]        = ( std::min )( minDepth, depth0 );
            if ( depth0 != infiniteDepth ) { maxPerBlock[p0] = ( std::max )( maxDepth, depth0 ); }
          }
        }
        for ( int64_t v = 0; v < int64_t( patch.getSizeV() ); ++v ) {
          for ( int64_t u = 0; u < int64_t( patch.getSizeU() ); ++u ) {
            const size_t  p        = v * patch.getSizeU() + u;
            const int16_t depth0   = patch.getDepth( 0 )[p];
            const size_t  u0       = u / patch.getOccupancyResolution();
            const size_t  v0       = v / patch.getOccupancyResolution();
            const size_t  p0       = v0 * patch.getSizeU0() + u0;
            const int16_t maxDepth = maxPerBlock[p0];
            if ( !sixDirection ) {
              if ( ( maxDepth - depth0 > 32 ) || ( depth0 + surfaceThickness > patch.getD1() + maxAllowedDepth ) ) {
                patch.getDepth( 0 )[p] = infiniteDepth;
                if ( useSurfaceSeparation ) { patch.getdepth0pccidx()[p] = infinitenumber; }
              }
            } else {
              if ( maxDepth < 0 ) {
                patch.getDepth( 0 )[p] = infiniteDepth;
                if ( useSurfaceSeparation ) { patch.getdepth0pccidx()[p] = infinitenumber; }
              } else {
                if ( depth0 != infiniteDepth ) {
                  int16_t tmp_a = maxDepth - depth0;
                  int16_t tmp_b = depth0 - int16_t( surfaceThickness );
                  int16_t tmp_c = int16_t( patch.getD1() ) - int16_t( maxAllowedDepth );
                  if ( ( tmp_a > 32 ) || ( tmp_b < tmp_c ) ) {
                    patch.getDepth( 0 )[p] = infiniteDepth;
                    if ( useSurfaceSeparation ) patch.getdepth0pccidx()[p] = infinitenumber;
                  }
                }
              }
            }
          }
        }
      }

      // compute d1 map
      patch.getDepth( 1 ) = patch.getDepth( 0 );
      if ( patch.getProjectionMode() == 0 ) {  // min
        if ( !useSurfaceSeparation ) {
          for ( const auto i : connectedComponent ) {
            PCCPoint3D pointTmp = points[i];
            if ( bIsAdditionalProjectionPlane ) {
              auto& input = pointTmp;
              convert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, pointTmp );
            }
            const auto& point = pointTmp;

            const int16_t d = int16_t( round( point[patch.getNormalAxis()] ) );
            const size_t  u = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
            const size_t  v = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
            assert( u >= 0 && u < patch.getSizeU() );
            assert( v >= 0 && v < patch.getSizeV() );
            const size_t  p      = v * patch.getSizeU() + u;
            const int16_t depth0 = patch.getDepth( 0 )[p];

            if ( depth0 < infiniteDepth && ( d - depth0 ) <= int16_t( surfaceThickness ) &&
                 d > patch.getDepth( 1 )[p] ) {
              patch.getDepth( 1 )[p] = d;
            }
            if ( useEnhancedDeltaDepthCode && depth0 < infiniteDepth && ( d - depth0 ) > 0 &&
                 ( d - depth0 ) <= int16_t( surfaceThickness ) ) {
              const uint16_t oldEDDCode = patch.getDepthEnhancedDeltaD()[p];
              const uint16_t deltaD     = d - depth0;
              int            comp_depth0;
              if ( !sixDirection ) {
                comp_depth0 = depth0;
              } else {
                comp_depth0 = depth0 - patch.getD1();
              }
              patch.getDepthEnhancedDeltaD()[p] |= 1 << ( deltaD - 1 );
              if ( ( comp_depth0 + patch.getDepthEnhancedDeltaD()[p] ) > maxD ) {
                patch.getDepthEnhancedDeltaD()[p] = oldEDDCode;
                std::cout << "(D0 + EDD-Code) > maxD. Data overflow observed (assume using 10bit coding). Temporary "
                             "solution: the corresponding inbetween or Depth1 point will be regarded as missing point. "
                             "To be improved if this happens a lot...\n";
              }
            }
            if ( patch.getDepth( 1 )[p] < patch.getDepth( 0 )[p] ) {
              std::cout << "compute d1 map : ERROR : proj0 and d1 < d0" << std::endl;
            }
          }
        } else {  // Surface Separation
          bool   proc_d1_select_flag    = true;
          bool   err_flag               = false;
          size_t patch_surfaceThickness = surfaceThickness;

          while ( proc_d1_select_flag ) {
            int64_t err_sum = 0;
            int32_t tot_num = 0;
            int32_t d1_num  = 0;

            if ( err_flag == true ) { proc_d1_select_flag = false; }
            for ( const auto i : connectedComponent ) {
              PCCPoint3D pointTmp = points[i];
              if ( bIsAdditionalProjectionPlane ) {
                auto& input = pointTmp;
                convert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, pointTmp );
              }
              const auto&   point = pointTmp;
              const int16_t d     = int16_t( round( point[patch.getNormalAxis()] ) );
              const size_t  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
              const size_t  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
              assert( u >= 0 && u < patch.getSizeU() );
              assert( v >= 0 && v < patch.getSizeV() );
              const size_t  p      = v * patch.getSizeU() + u;
              const int16_t depth0 = patch.getDepth( 0 )[p];
              tot_num++;

              if ( err_flag == false ) {
                if ( depth0 < infiniteDepth && ( d - depth0 ) <= int16_t( patch_surfaceThickness ) &&
                     d > patch.getDepth( 1 )[p] ) {
                  d1_num++;
                  const size_t     d0_idx   = patch.getdepth0pccidx()[p];
                  const size_t     d1_idx   = i;
                  const PCCColor3B D0_Color = frame_pcc_color[d0_idx];
                  const PCCColor3B D1_Color = frame_pcc_color[d1_idx];

                  const uint8_t r0 = D0_Color[0];
                  const uint8_t g0 = D0_Color[1];
                  const uint8_t b0 = D0_Color[2];

                  const uint8_t r1 = D1_Color[0];
                  const uint8_t g1 = D1_Color[1];
                  const uint8_t b1 = D1_Color[2];

                  int32_t delta_R = int32_t( r0 ) - int32_t( r1 );
                  int32_t delta_G = int32_t( g0 ) - int32_t( g1 );
                  int32_t delta_B = int32_t( b0 ) - int32_t( b1 );

                  int64_t delta_e = delta_R * delta_R + delta_G * delta_G + delta_B * delta_B;

                  err_sum += delta_e;
                }
              } else {
                if ( depth0 < infiniteDepth && ( d - depth0 ) <= int16_t( patch_surfaceThickness ) &&
                     d > patch.getDepth( 1 )[p] ) {
                  patch.getDepth( 1 )[p] = d;
                }
                if ( useEnhancedDeltaDepthCode && depth0 < infiniteDepth && ( d - depth0 ) > 0 &&
                     ( d - depth0 ) <= int16_t( patch_surfaceThickness ) ) {
                  const uint16_t oldEDDCode = patch.getDepthEnhancedDeltaD()[p];
                  const uint16_t deltaD     = d - depth0;
                  int            comp_depth0;
                  if ( !sixDirection ) {
                    comp_depth0 = depth0;
                  } else {
                    comp_depth0 = depth0 - patch.getD1();
                  }
                  patch.getDepthEnhancedDeltaD()[p] |= 1 << ( deltaD - 1 );
                  if ( ( comp_depth0 + patch.getDepthEnhancedDeltaD()[p] ) > maxD ) {
                    patch.getDepthEnhancedDeltaD()[p] = oldEDDCode;
                    std::cout << "(D0 + EDD-Code) > maxD. Data overflow observed (assume using 10bit coding). "
                                 "Temporary solution: the corresponding inbetween or Depth1 point will be regarded as "
                                 "missing point. To be improved if this happens a lot...\n";
                  }
                }
                if ( patch.getDepth( 1 )[p] < patch.getDepth( 0 )[p] ) {
                  std::cout << "compute d1 map : ERROR : proj0 and d1 < d0" << std::endl;
                }
              }
            }
            double avg_error;
            if ( d1_num == 0 ) {
              avg_error = 0.0;
            } else {
              avg_error = (double)err_sum / (double)d1_num;
            }
            if ( avg_error < Threshold_Color_Error ) {
              err_flag = true;
            } else {
              patch_surfaceThickness--;
              if ( patch_surfaceThickness == 0 ) { break; }
            }
          }
        }
      } else {  // else ( patch.getProjectionMode() == 0 )
        if ( !useSurfaceSeparation ) {
          for ( const auto i : connectedComponent ) {
            PCCPoint3D pointTmp = points[i];
            if ( bIsAdditionalProjectionPlane ) {
              auto& input = pointTmp;
              convert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, pointTmp );
            }
            const auto&   point = pointTmp;
            const int16_t d     = int16_t( round( point[patch.getNormalAxis()] ) );
            const size_t  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
            const size_t  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
            assert( u >= 0 && u < patch.getSizeU() );
            assert( v >= 0 && v < patch.getSizeV() );
            const size_t  p      = v * patch.getSizeU() + u;
            const int16_t depth0 = patch.getDepth( 0 )[p];
            if ( depth0 < infiniteDepth && ( depth0 - d ) <= int16_t( surfaceThickness ) &&
                 d < patch.getDepth( 1 )[p] ) {
              patch.getDepth( 1 )[p] = d;
            }
            if ( useEnhancedDeltaDepthCode && depth0 < infiniteDepth && ( depth0 - d ) > 0 &&
                 ( depth0 - d ) <= int16_t( surfaceThickness ) ) {
              const uint16_t oldEDDCode = patch.getDepthEnhancedDeltaD()[p];
              const uint16_t deltaD     = depth0 - d;
              int            comp_depth0;
              if ( !sixDirection ) {
                comp_depth0 = depth0;
              } else {
                comp_depth0 = patch.getD1() - depth0;
              }
              patch.getDepthEnhancedDeltaD()[p] |= 1 << ( deltaD - 1 );
              if ( ( !sixDirection && ( comp_depth0 - patch.getDepthEnhancedDeltaD()[p] ) < 0 ) ||
                   ( sixDirection && ( comp_depth0 + patch.getDepthEnhancedDeltaD()[p] ) > maxD ) ) {
                patch.getDepthEnhancedDeltaD()[p] = oldEDDCode;
                std::cout << "(D0 + EDD-Code) > maxD. Data overflow observed (assume using 10bit coding). Temporary "
                             "solution: the corresponding inbetween or Depth1 point will be regarded as missing point. "
                             "To be improved if this happens a lot...\n";
              }
            }
          }
        } else {  // Surface Separation
          bool   proc_d1_select_flag    = true;
          bool   err_flag               = false;
          size_t patch_surfaceThickness = surfaceThickness;

          while ( proc_d1_select_flag ) {
            int64_t err_sum = 0;
            int32_t tot_num = 0;
            int32_t d1_num  = 0;

            if ( err_flag == true ) { proc_d1_select_flag = false; }

            for ( const auto i : connectedComponent ) {
              PCCPoint3D pointTmp = points[i];
              if ( bIsAdditionalProjectionPlane ) {
                auto& input = pointTmp;
                convert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, pointTmp );
              }
              const auto&   point = pointTmp;
              const int16_t d     = int16_t( round( point[patch.getNormalAxis()] ) );
              const size_t  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
              const size_t  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
              assert( u >= 0 && u < patch.getSizeU() );
              assert( v >= 0 && v < patch.getSizeV() );
              const size_t  p      = v * patch.getSizeU() + u;
              const int16_t depth0 = patch.getDepth( 0 )[p];
              tot_num++;

              if ( err_flag == false ) {
                if ( depth0 < infiniteDepth && ( depth0 - d ) <= int16_t( patch_surfaceThickness ) &&
                     d < patch.getDepth( 1 )[p] ) {
                  d1_num++;
                  const size_t     d0_idx   = patch.getdepth0pccidx()[p];
                  const size_t     d1_idx   = i;
                  const PCCColor3B D0_Color = frame_pcc_color[d0_idx];
                  const PCCColor3B D1_Color = frame_pcc_color[d1_idx];
                  int32_t          delta_R  = int32_t( D0_Color[0] ) - int32_t( D1_Color[0] );
                  int32_t          delta_G  = int32_t( D0_Color[1] ) - int32_t( D1_Color[1] );
                  int32_t          delta_B  = int32_t( D0_Color[2] ) - int32_t( D1_Color[2] );
                  int64_t          delta_e  = delta_R * delta_R + delta_G * delta_G + delta_B * delta_B;
                  err_sum += delta_e;
                }
              } else {
                if ( depth0 < infiniteDepth && ( depth0 - d ) <= int16_t( patch_surfaceThickness ) &&
                     d < patch.getDepth( 1 )[p] ) {
                  patch.getDepth( 1 )[p] = d;
                }
                if ( useEnhancedDeltaDepthCode && depth0 < infiniteDepth && ( depth0 - d ) > 0 &&
                     ( depth0 - d ) <= int16_t( surfaceThickness ) ) {
                  const uint16_t oldEDDCode = patch.getDepthEnhancedDeltaD()[p];
                  const uint16_t deltaD     = depth0 - d;
                  int            comp_depth0;
                  if ( !sixDirection ) {
                    comp_depth0 = depth0;
                  } else {
                    comp_depth0 = patch.getD1() - depth0;
                  }
                  patch.getDepthEnhancedDeltaD()[p] |= 1 << ( deltaD - 1 );
                  if ( ( !sixDirection && ( comp_depth0 - patch.getDepthEnhancedDeltaD()[p] ) < 0 ) ||
                       ( sixDirection && ( comp_depth0 + patch.getDepthEnhancedDeltaD()[p] ) > maxD ) ) {
                    patch.getDepthEnhancedDeltaD()[p] = oldEDDCode;
                    std::cout << "(D0 + EDD-Code) > maxD. Data overflow observed (assume using 10bit coding). "
                                 "Temporary solution: the corresponding inbetween or Depth1 point will be regarded as "
                                 "missing point. To be improved if this happens a lot...\n";
                  }
                }
              }
            }
            double avg_error;
            if ( d1_num == 0 ) {
              avg_error = 0.0;
            } else {
              avg_error = (double)err_sum / (double)d1_num;
            }
            if ( avg_error < Threshold_Color_Error ) {
              err_flag = true;
            } else {
              patch_surfaceThickness--;
              if ( patch_surfaceThickness == 0 ) { break; }
            }
          }
        }
      }  // fi ( patch.getProjectionMode() == 0 )
      if ( !sixDirection && patch.getProjectionMode() == 1 ) {
        size_t quatnizedSurfaceThickness = size_t( double( surfaceThickness ) / double( minLevel ) + 0.5 ) * minLevel;
        if ( patch.getD1() < quatnizedSurfaceThickness )
          patch.getD1() = 0;
        else
          patch.getD1() = patch.getD1() - quatnizedSurfaceThickness;
      }
      patch.getSizeD() = 0;

      PCCPointSet3 rec;
      rec.resize( 0 );
      if ( !sixDirection || ( sixDirection && patch.getProjectionMode() == 0 ) ) {
        for ( size_t v = 0; v < patch.getSizeV(); ++v ) {
          for ( size_t u = 0; u < patch.getSizeU(); ++u ) {
            const size_t p      = v * patch.getSizeU() + u;
            int16_t      depth0 = patch.getDepth( 0 )[p];
            if ( depth0 < infiniteDepth ) {
              depth0 -= int16_t( patch.getD1() );
              assert( depth0 >= 0 );
              patch.getDepth( 0 )[p] = depth0;
              patch.getSizeD() = ( std::max )( patch.getSizeD(), static_cast<size_t>( depth0 ) );  // compute max depth

              const size_t u0 = u / patch.getOccupancyResolution();
              const size_t v0 = v / patch.getOccupancyResolution();
              const size_t p0 = v0 * patch.getSizeU0() + u0;
              assert( u0 >= 0 && u0 < patch.getSizeU0() );
              assert( v0 >= 0 && v0 < patch.getSizeV0() );
              patch.getOccupancy()[p0] = true;

              PCCVector3D point;
              point[patch.getNormalAxis()]    = double( depth0 ) + patch.getD1();
              point[patch.getTangentAxis()]   = double( u ) + patch.getU1();
              point[patch.getBitangentAxis()] = double( v ) + patch.getV1();

              if ( bIsAdditionalProjectionPlane ) {
                PCCVector3D point_tmp;
                auto&       input = point;
                iconvert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, point_tmp );
                resampled.addPoint( point_tmp );
                resampledPatchPartition.push_back( patchIndex );
                if ( createSubPointCloud ) { rec.addPoint( point_tmp ); }
              } else {
                resampled.addPoint( (const PCCVector3D)point );
                resampledPatchPartition.push_back( patchIndex );
                if ( createSubPointCloud ) { rec.addPoint( point ); }
              }

              patch.getDepth( 1 )[p] -= int16_t( patch.getD1() );
              if ( useEnhancedDeltaDepthCode ) {
                if ( patch.getDepthEnhancedDeltaD()[p] != 0 ) {
                  for ( uint16_t i = 0; i < surfaceThickness; i++ ) {
                    if ( patch.getDepthEnhancedDeltaD()[p] & ( 1 << i ) ) {
                      uint16_t nDeltaDCur = ( i + 1 );
                      if ( !sixDirection ) {
                        if ( patch.getProjectionMode() == 0 ) {
                          point[patch.getNormalAxis()] = double( depth0 + patch.getD1() + nDeltaDCur );
                        } else {
                          point[patch.getNormalAxis()] = double( depth0 + patch.getD1() - nDeltaDCur );
                        }
                      } else {
                        point[patch.getNormalAxis()] = double( depth0 + patch.getD1() + nDeltaDCur );
                      }
                      resampled.addPoint( point );
                      resampledPatchPartition.push_back( patchIndex );
                    }
                  }     // for each i
                }       // if( patch.getDepthEnhancedDeltaD()[p] != 0) )
              } else {  // if(useEnhancedDeltaDepthCode)
                point[patch.getNormalAxis()] = double( patch.getDepth( 1 )[p] + patch.getD1() );
                if ( bIsAdditionalProjectionPlane ) {
                  PCCVector3D point_tmp;
                  auto&       input = point;
                  iconvert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, point_tmp );
                  resampled.addPoint( point_tmp );
                  resampledPatchPartition.push_back( patchIndex );
                  assert( abs( patch.getDepth( 1 )[p] - patch.getDepth( 0 )[p] ) <= int( surfaceThickness ) );
                  if ( createSubPointCloud ) { rec.addPoint( point_tmp ); }
                } else {
                  resampled.addPoint( point );
                  resampledPatchPartition.push_back( patchIndex );
                  assert( abs( patch.getDepth( 1 )[p] - patch.getDepth( 0 )[p] ) <= int( surfaceThickness ) );
                  if ( createSubPointCloud ) { rec.addPoint( point ); }
                }
                if ( sixDirection ) {
                  int16_t depth1 = patch.getDepth( 1 )[p];
                  patch.getSizeD() =
                      ( std::max )( patch.getSizeD(), static_cast<size_t>( depth1 ) );  // compute max depth
                }
              }  // if(useEnhancedDeltaDepthCode)
            }
          }
        }
      } else if ( sixDirection ) {
        for ( size_t v = 0; v < patch.getSizeV(); ++v ) {
          for ( size_t u = 0; u < patch.getSizeU(); ++u ) {
            const size_t p        = v * patch.getSizeU() + u;
            int16_t      depth0_1 = patch.getDepth( 0 )[p];
            if ( depth0_1 < infiniteDepth ) {
              int16_t depth0 = int16_t( patch.getD1() ) - depth0_1;
              assert( depth0 >= 0 );
              patch.getDepth( 0 )[p] = depth0;
              patch.getSizeD() = ( std::max )( patch.getSizeD(), static_cast<size_t>( depth0 ) );  // compute max depth

              const size_t u0 = u / patch.getOccupancyResolution();
              const size_t v0 = v / patch.getOccupancyResolution();
              const size_t p0 = v0 * patch.getSizeU0() + u0;
              assert( u0 >= 0 && u0 < patch.getSizeU0() );
              assert( v0 >= 0 && v0 < patch.getSizeV0() );
              patch.getOccupancy()[p0] = true;

              PCCVector3D point;
              point[patch.getNormalAxis()]    = double( depth0_1 );
              point[patch.getTangentAxis()]   = double( u ) + patch.getU1();
              point[patch.getBitangentAxis()] = double( v ) + patch.getV1();
              if ( bIsAdditionalProjectionPlane ) {
                PCCVector3D point_tmp;
                auto&       input = point;
                iconvert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, point_tmp );
                resampled.addPoint( point_tmp );
                resampledPatchPartition.push_back( patchIndex );
                if ( createSubPointCloud ) { rec.addPoint( point_tmp ); }
              } else {
                resampled.addPoint( point );
                resampledPatchPartition.push_back( patchIndex );
                if ( createSubPointCloud ) { rec.addPoint( point ); }
              }
              if ( createSubPointCloud ) { rec.addPoint( point ); }

              int16_t depth1_1       = patch.getDepth( 1 )[p];
              patch.getDepth( 1 )[p] = int16_t( patch.getD1() ) - depth1_1;

              if ( useEnhancedDeltaDepthCode ) {
                if ( patch.getDepthEnhancedDeltaD()[p] != 0 ) {
                  for ( uint16_t i = 0; i < surfaceThickness; i++ ) {
                    if ( patch.getDepthEnhancedDeltaD()[p] & ( 1 << i ) ) {
                      uint16_t nDeltaDCur          = ( i + 1 );
                      point[patch.getNormalAxis()] = double( patch.getD1() - depth0 - nDeltaDCur );
                      resampled.addPoint( point );
                      resampledPatchPartition.push_back( patchIndex );
                    }
                  }     // for each i
                }       // if( patch.getDepthEnhancedDeltaD()[p] != 0) )
              } else {  // if(useEnhancedDeltaDepthCode)
                point[patch.getNormalAxis()] = double( depth1_1 );
                if ( bIsAdditionalProjectionPlane ) {
                  PCCVector3D point_tmp;
                  auto&       input = point;
                  iconvert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, point_tmp );
                  resampled.addPoint( point_tmp );
                  resampledPatchPartition.push_back( patchIndex );
                  if ( createSubPointCloud ) { rec.addPoint( point_tmp ); }
                } else {
                  resampled.addPoint( point );
                  resampledPatchPartition.push_back( patchIndex );
                  if ( createSubPointCloud ) { rec.addPoint( point ); }
                }
                assert( abs( patch.getDepth( 0 )[p] - patch.getDepth( 1 )[p] ) <= int( surfaceThickness ) );

                int16_t depth1_2 = patch.getDepth( 1 )[p];
                patch.getSizeD() =
                    ( std::max )( patch.getSizeD(), static_cast<size_t>( depth1_2 ) );  // compute max depth
              }
            }
          }
        }
      }

      size_t quantDD   = patch.getSizeD() == 0 ? 0 : ( ( patch.getSizeD() - 1 ) / minLevel + 1 );
      patch.getSizeD() = ( std::min )( quantDD * minLevel, static_cast<size_t>( maxAllowedDepth ) );

      if ( createSubPointCloud ) {
        PCCPointSet3 testSrc, testRec;
        for ( const auto i : connectedComponent ) {
          if ( bIsAdditionalProjectionPlane ) {
            PCCVector3D input;
            input.x() = points[i].x();
            input.y() = points[i].y();
            input.z() = points[i].z();
            PCCVector3D point_tmp;
            iconvert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, point_tmp );
            testSrc.addPoint( point_tmp, points.getColor( i ) );
          } else {
            testSrc.addPoint( points[i], points.getColor( i ) );
          }
        }
        for ( const auto& p : rec.getPositions() ) { testRec.addPoint( p ); }
        testSrc.transfertColorSimple( testRec );
        float distPAB, distPBA, distYAB, distYBA, distUAB, distUBA, distVAB, distVBA;
        testRec.removeDuplicate();
        testSrc.distanceGeoColor( testRec, distPAB, distPBA, distYAB, distYBA, distUAB, distUBA, distVAB, distVBA );
        meanPAB += distPAB * testSrc.getPointCount();
        meanPBA += distPBA * testRec.getPointCount();
        meanYAB += distYAB * testSrc.getPointCount();
        meanYBA += distYBA * testRec.getPointCount();
        meanUAB += distUAB * testSrc.getPointCount();
        meanUBA += distUBA * testRec.getPointCount();
        meanVAB += distVAB * testSrc.getPointCount();
        meanVBA += distVBA * testRec.getPointCount();
        testSrcNum += testSrc.getPointCount();
        testRecNum += testRec.getPointCount();

        auto& sub = subPointCloud[subPointCloud.size() - 1];
        sub.resize( 0 );
        PCCKdTree kdtreeRec( rec );
        for ( const auto i : connectedComponent ) {
          kdtreeRec.search( points[i], 1, result );
          const double dist2 = result.dist( 0 );
          if ( dist2 <= maxAllowedDist2MissedPointsSelection ) {
            if ( bIsAdditionalProjectionPlane ) {
              PCCVector3D input;
              input.x() = points[i].x();
              input.y() = points[i].y();
              input.z() = points[i].z();
              PCCVector3D point_tmp;
              iconvert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, point_tmp );
              sub.addPoint( point_tmp, points.getColor( i ) );
            } else {
              sub.addPoint( points[i], points.getColor( i ) );
            }
          }
        }
      }
      std::cout << "\t\t Patch " << patchIndex << " ->(d1,u1,v1)=( " << patch.getD1() << " , " << patch.getU1() << " , "
                << patch.getV1() << " )(dd,du,dv)=( " << patch.getSizeD() << " , " << patch.getSizeU() << " , "
                << patch.getSizeV() << " ),Normal: " << size_t( patch.getNormalAxis() )
                << " Direction: " << patch.getProjectionMode() << std::endl;
    }
    PCCKdTree kdtreeResampled( resampled );
    missedPoints.resize( 0 );
    for ( size_t i = 0; i < pointCount; ++i ) {
      kdtreeResampled.search( points[i], 1, result );
      const double dist2      = result.dist( 0 );
      missedPointsDistance[i] = dist2;
      if ( dist2 > maxAllowedDist2MissedPointsSelection ) { missedPoints.push_back( i ); }
    }
    std::cout << " # patches " << patches.size() << std::endl;
    std::cout << " # missed points " << missedPoints.size() << std::endl;
  }
  distanceSrcRec = meanYAB + meanUAB + meanVAB + meanYBA + meanUBA + meanVBA;
}

void PCCPatchSegmenter3::refineSegmentation( const PCCPointSet3&         pointCloud,
                                             const PCCKdTree&            kdtree,
                                             const PCCNormalsGenerator3& normalsGen,
                                             const PCCVector3D*          orientations,
                                             const size_t                orientationCount,
                                             const size_t                maxNNCount,
                                             const double                lambda,
                                             const size_t                iterationCount,
                                             std::vector<size_t>&        partition ) {
  assert( orientations );
  std::vector<std::vector<size_t>> adj;
  computeAdjacencyInfo( pointCloud, kdtree, adj, maxNNCount );
  const size_t                     pointCount = pointCloud.getPointCount();
  const double                     weight     = lambda / maxNNCount;
  std::vector<size_t>              tempPartition( pointCount );
  std::vector<std::vector<size_t>> scoresSmooth( pointCount, std::vector<size_t>( orientationCount ) );
  for ( size_t k = 0; k < iterationCount; ++k ) {
    tbb::task_arena limited( (int)nbThread_ );
    limited.execute( [&] {
      tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) {
        auto& scoreSmooth = scoresSmooth[i];
        std::fill( scoreSmooth.begin(), scoreSmooth.end(), 0 );
        for ( auto& neighbor : adj[i] ) { scoreSmooth[partition[neighbor]]++; }
      } );
    } );
    limited.execute( [&] {
      tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) {
        const PCCVector3D normal       = normalsGen.getNormal( i );
        size_t            clusterIndex = partition[i];
        double            bestScore    = 0.0;
        const auto&       scoreSmooth  = scoresSmooth[i];
        for ( size_t j = 0; j < orientationCount; ++j ) {
          const double scoreNormal = normal * orientations[j];
          const double score       = scoreNormal + weight * scoreSmooth[j];
          if ( score > bestScore ) {
            bestScore    = score;
            clusterIndex = j;
          }
        }
        tempPartition[i] = clusterIndex;
      } );
    } );
    swap( tempPartition, partition );
  }
  for ( auto& vector : scoresSmooth ) { vector.clear(); }
  scoresSmooth.clear();
}

float pcc::computeIOU( Rect a, Rect b ) {
  float iou              = 0.0f;
  Rect  intersec         = a & b;
  int   intersectionArea = intersec.area();
  int   unionArea        = a.area() + b.area() - intersectionArea;
  iou                    = (float)intersectionArea / unionArea;
  return iou;
}
