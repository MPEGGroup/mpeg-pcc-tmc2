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
                                  const size_t                        frameIndex,
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

  if ( params.gridBasedRefineSegmentation ) {
    std::cout << "  Refining segmentation (grid-based)... ";
    refineSegmentationGridBased( geometry, normalsGen, orientations, orientationCount,
                                 params.maxNNCountRefineSegmentation, params.lambdaRefineSegmentation,
                                 params.iterationCountRefineSegmentation, params.voxelDimensionRefineSegmentation,
                                 params.searchRadiusRefineSegmentation, partition );
  } else {
    std::cout << "  Refining segmentation... ";
    refineSegmentation( geometry, kdtree, normalsGen, orientations, orientationCount,
                        params.maxNNCountRefineSegmentation, params.lambdaRefineSegmentation,
                        params.iterationCountRefineSegmentation, partition );
  }
  std::cout << "[done]" << std::endl;

  std::cout << "  Patch segmentation... ";
  PCCPointSet3        resampled;
  std::vector<size_t> patchPartition;
  std::vector<size_t> resampledPatchPartition;
  std::vector<size_t> missedPoints;

  segmentPatches(
      geometry, frameIndex, kdtree, params.maxNNCountPatchSegmentation, params.minPointCountPerCCPatchSegmentation,
      params.occupancyResolution, params.maxAllowedDist2MissedPointsDetection,
      params.maxAllowedDist2MissedPointsSelection, params.EOMSingleLayerMode, params.EOMFixBitCount,
      params.surfaceThickness, params.maxAllowedDepth, params.minLevel, partition, patches, patchPartition,
      resampledPatchPartition, missedPoints, resampled, params.useEnhancedDeltaDepthCode, params.createSubPointCloud,
      subPointCloud, distanceSrcRec, params.absoluteD1, params.surfaceSeparation, params.additionalProjectionPlaneMode,
      params.geometryBitDepth3D, params.testLevelOfDetail, params.patchExpansion, params.highGradientSeparation,
      params.minGradient, params.minNumHighGradientPoints, normalsGen, orientations, orientationCount,
      params.enablePointCloudPartitioning, const_cast<PCCPatchSegmenter3Parameters&>( params ).roiBoundingBoxMinX,
      const_cast<PCCPatchSegmenter3Parameters&>( params ).roiBoundingBoxMaxX,
      const_cast<PCCPatchSegmenter3Parameters&>( params ).roiBoundingBoxMinY,
      const_cast<PCCPatchSegmenter3Parameters&>( params ).roiBoundingBoxMaxY,
      const_cast<PCCPatchSegmenter3Parameters&>( params ).roiBoundingBoxMinZ,
      const_cast<PCCPatchSegmenter3Parameters&>( params ).roiBoundingBoxMaxZ,
      const_cast<PCCPatchSegmenter3Parameters&>( params ).numCutsAlong1stLongestAxis,
      const_cast<PCCPatchSegmenter3Parameters&>( params ).numCutsAlong2ndLongestAxis,
      const_cast<PCCPatchSegmenter3Parameters&>( params ).numCutsAlong3rdLongestAxis );
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

void PCCPatchSegmenter3::computeAdjacencyInfoInRadius( const PCCPointSet3&               pointCloud,
                                                       const PCCKdTree&                  kdtree,
                                                       std::vector<std::vector<size_t>>& adj,
                                                       const size_t                      maxNNCount,
                                                       const size_t                      radius ) {
  const size_t pointCount = pointCloud.getPointCount();
  adj.resize( pointCount );
  tbb::task_arena limited( (int)nbThread_ );
  limited.execute( [&] {
    tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) {
      PCCNNResult result;
      kdtree.searchRadius( pointCloud[i], maxNNCount, radius, result );
      std::vector<size_t>& neighbors = adj[i];
      neighbors.resize( result.count() );
      for ( size_t j = 0; j < result.count(); ++j ) { neighbors[j] = result.indices( j ); }
    } );
  } );
}

void PCCPatchSegmenter3::computeAdjacencyInfoDist( const PCCPointSet3&               pointCloud,
                                                   const PCCKdTree&                  kdtree,
                                                   std::vector<std::vector<size_t>>& adj,
                                                   std::vector<std::vector<double>>& adjDist,
                                                   const size_t                      maxNNCount ) {
  const size_t pointCount = pointCloud.getPointCount();
  adj.resize( pointCount );
  adjDist.resize( pointCount );
  tbb::task_arena limited( (int)nbThread_ );
  limited.execute( [&] {
    tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) {
      PCCNNResult result;
      kdtree.search( pointCloud[i], maxNNCount, result );
      std::vector<size_t>& neighbors     = adj[i];
      std::vector<double>& neighborsDist = adjDist[i];
      neighbors.resize( result.count() );
      neighborsDist.resize( result.count() );
      for ( size_t j = 0; j < result.count(); ++j ) {
        neighbors[j]     = result.indices( j );
        neighborsDist[j] = result.dist( j );
      }
    } );
  } );
}

void printChunk( const std::vector<std::pair<int, int>>& chunk ) {
  std::vector<std::string> axisName{"x -> ", "y -> ", "z -> "};
  for ( size_t axis = 0; axis < 3; ++axis ) {
    std::cout << axisName[axis] << "(" << chunk[axis].first << ", " << chunk[axis].second << ")"
              << " ";
  }
  std::cout << std::endl;
}

void PCCPatchSegmenter3::quantizedPointsPatchModification( const PCCPointSet3& points,
                                                           PCCPatch&           patch,  // current patch
                                                           bool                bIsAdditionalProjectionPlane,
                                                           const double        Threshold_Color_Error,
                                                           std::vector<PCCColor3B>& frame_pcc_color,
                                                           std::vector<size_t>&     connectedComponent,
                                                           const size_t             minPointCountPerCC,
                                                           const size_t             occupancyResolution,
                                                           const double maxAllowedDist2MissedPointsDetection,
                                                           const double maxAllowedDist2MissedPointsSelection,
                                                           const bool   EOMSingleLayerMode,
                                                           const size_t EOMFixBitCount,
                                                           const size_t surfaceThickness,
                                                           const size_t maxAllowedDepth,
                                                           const size_t minLevel,
                                                           const std::vector<size_t>& partition,
                                                           std::vector<PCCPatch>&     patches,
                                                           std::vector<size_t>&       patchPartition,
                                                           std::vector<size_t>&       resampledPatchPartition,
                                                           std::vector<size_t>        missedPoints,
                                                           PCCPointSet3&              resampled,
                                                           bool                       useEnhancedDeltaDepthCode,
                                                           const bool                 createSubPointCloud,
                                                           std::vector<PCCPointSet3>& subPointCloud,
                                                           float&                     distanceSrcRec,
                                                           const bool                 absoluteD1,
                                                           bool                       useSurfaceSeparation,
                                                           const size_t               additionalProjectionAxis,
                                                           const size_t               geometryBitDepth3D,
                                                           const size_t               testLevelOfDetail,
                                                           bool                       patchExpansionEnabled,
                                                           bool                       enablePointCloudPartitioning,
                                                           std::vector<int>&          roiBoundingBoxMinX,
                                                           std::vector<int>&          roiBoundingBoxMaxX,
                                                           std::vector<int>&          roiBoundingBoxMinY,
                                                           std::vector<int>&          roiBoundingBoxMaxY,
                                                           std::vector<int>&          roiBoundingBoxMinZ,
                                                           std::vector<int>&          roiBoundingBoxMaxZ,
                                                           int                        numCutsAlong1stLongestAxis,
                                                           int                        numCutsAlong2ndLongestAxis,
                                                           int                        numCutsAlong3rdLongestAxis ) {
  //  const size_t lodScaleX = patch.getLodScaleX();
  //  const size_t lodScaleY = patch.getLodScaleY();
  //
  //  const size_t originalSizeU = patch.getSizeU();
  //  const size_t originalSizeV = patch.getSizeV();
  //
  //  const size_t afterLodSizeU = ceil( originalSizeU / (double)lodScaleX );
  //  const size_t afterLodSizeV = ceil( originalSizeV / (double)lodScaleY );
  //
  //  const size_t finalSizeU = ( std::min )( originalSizeU, afterLodSizeU );
  //  const size_t finalSizeV = ( std::min )( originalSizeV, afterLodSizeV );
  //
  //  // vector clear
  //  std::vector<int16_t>().swap( patch.getDepth( 0 ) );
  //  std::vector<int16_t>().swap( patch.getDepth( 1 ) );
  //  std::vector<int64_t>().swap( patch.getdepth0pccidx() );
  //  std::vector<int16_t>().swap( patch.getDepthEnhancedDeltaD() );
  //  std::vector<bool>().swap( patch.getOccupancy() );
  //
  //
  //  if ( patch.getProjectionMode() == 0 ) {
  //    patch.getD1() = infiniteDepth;
  //  } else {
  //    patch.getD1() = 0;
  //  }
  //
  //  patch.getDepth( 0 ).resize( finalSizeU * finalSizeV, infiniteDepth );
  //  if ( useSurfaceSeparation ) { patch.getdepth0pccidx().resize( finalSizeU * finalSizeV, infinitenumber ); }
  //    if ( useEnhancedDeltaDepthCode ) { patch.getDepthEnhancedDeltaD().resize( finalSizeU * finalSizeV, 0 ); }
  //
  //  patch.getOccupancyResolution() = occupancyResolution;
  //  patch.getSizeU0()              = 0;
  //  patch.getSizeV0()              = 0;
  //
  //  std::vector<std::vector<bool>> checkPoint( patch.getSizeV(), std::vector<bool>( patch.getSizeU(), false ) );
  //  // std::vector<size_t>            indexVector;
  //  for ( size_t v = 0; v < originalSizeV; v++ ) {
  //    for ( size_t u = 0; u < originalSizeU; u++ ) {
  //      if ( u % lodScaleX == 0 && v % lodScaleY == 0 ) {
  //        checkPoint[v][u] = true;
  //        // indexVector.push_back( p );
  //      }
  //    }
  //  }
  //
  //  for ( const auto i : connectedComponent ) {
  //    PCCPoint3D pointTmp = points[i];
  //    if ( bIsAdditionalProjectionPlane ) {
  //      auto& input = pointTmp;
  //      convert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, pointTmp );
  //    }
  //    const auto&   point = pointTmp;
  //    const int16_t d     = int16_t( round( point[patch.getNormalAxis()] ) );
  //    const size_t  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
  //    const size_t  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
  //    assert( u >= 0 && u < patch.getSizeU() );
  //    assert( v >= 0 && v < patch.getSizeV() );
  //
  //    if ( checkPoint[v][u] == true ) {
  //      const size_t nu = round( u / (double)lodScaleX );
  //      const size_t nv = round( v / (double)lodScaleY );
  //      const size_t np = nv * finalSizeU + nu;
  //
  //      if ( patch.getProjectionMode() == 0 ) {  // min
  //        if ( patch.getDepth( 0 )[np] > d ) {
  //          patch.getDepth( 0 )[np] = d;
  //          if ( useSurfaceSeparation ) { patch.getdepth0pccidx()[np] = i; }
  //          patch.getSizeU0() = ( std::max )( patch.getSizeU0(), nu / patch.getOccupancyResolution() );
  //          patch.getSizeV0() = ( std::max )( patch.getSizeV0(), nv / patch.getOccupancyResolution() );
  //          patch.getD1()     = ( std::min )( patch.getD1(), size_t( d ) );
  //          size_t value      = patch.getD1();  // patch.getD1();
  //          value             = ( value / minLevel );
  //          patch.getD1()     = value * minLevel;
  //        }
  //      } else {  // max
  //        if ( patch.getDepth( 0 )[np] == infiniteDepth ) {
  //          patch.getDepth( 0 )[np] = d;
  //          if ( useSurfaceSeparation ) patch.getdepth0pccidx()[np] = i;
  //          patch.getSizeU0() = ( std::max )( patch.getSizeU0(), nu / patch.getOccupancyResolution() );
  //          patch.getSizeV0() = ( std::max )( patch.getSizeV0(), nv / patch.getOccupancyResolution() );
  //          patch.getD1()     = ( std::max )( patch.getD1(), size_t( d ) );
  //          size_t value      = size_t( patch.getD1() / minLevel );
  //          if ( value * minLevel < patch.getD1() )
  //            patch.getD1() = ( 1 + value ) * minLevel;
  //          else
  //            patch.getD1() = value * minLevel;
  //          // std::cout<<"~~~~~~"<<value<<" -> "<<patch.getD1()<<std::endl;
  //        } else {
  //          if ( patch.getDepth( 0 )[np] < d ) {
  //            patch.getDepth( 0 )[np] = d;
  //            if ( useSurfaceSeparation ) patch.getdepth0pccidx()[np] = i;
  //            patch.getSizeU0() = ( std::max )( patch.getSizeU0(), nu / patch.getOccupancyResolution() );
  //            patch.getSizeV0() = ( std::max )( patch.getSizeV0(), nv / patch.getOccupancyResolution() );
  //            patch.getD1()     = ( std::max )( patch.getD1(), size_t( d ) );
  //            size_t value      = size_t( patch.getD1() / minLevel );
  //            if ( value * minLevel < patch.getD1() )
  //              patch.getD1() = ( 1 + value ) * minLevel;
  //            else
  //              patch.getD1() = value * minLevel;
  //            // std::cout<<"*******"<<value<<" -> "<<patch.getD1()<<std::endl;
  //          }
  //        }
  //      }
  //    }
  //  }
  //
  //  ++patch.getSizeU0();
  //  ++patch.getSizeV0();
  //  patch.getOccupancy().resize( patch.getSizeU0() * patch.getSizeV0(), false );
  //  // filter depth
  //  if ( patch.getProjectionMode() == 0 ) {  // min
  //    std::vector<int16_t> minPerBlock;
  //    minPerBlock.resize( patch.getSizeU0() * patch.getSizeV0(), infiniteDepth );
  //    for ( int64_t v = 0; v < int64_t( finalSizeV ); ++v ) {
  //      for ( int64_t u = 0; u < int64_t( finalSizeU ); ++u ) {
  //        const size_t  p        = v * finalSizeU + u;
  //        const int16_t depth0   = patch.getDepth( 0 )[p];
  //        const size_t  u0       = u / patch.getOccupancyResolution();
  //        const size_t  v0       = v / patch.getOccupancyResolution();
  //        const size_t  p0       = v0 * patch.getSizeU0() + u0;
  //        const int16_t minDepth = minPerBlock[p0];
  //        minPerBlock[p0]        = ( std::min )( minDepth, depth0 );
  //      }
  //    }
  //    for ( int64_t v = 0; v < int64_t( finalSizeV ); ++v ) {
  //      for ( int64_t u = 0; u < int64_t( finalSizeU ); ++u ) {
  //        const size_t  p        = v * finalSizeU + u;
  //        const int16_t depth0   = patch.getDepth( 0 )[p];
  //        const size_t  u0       = u / patch.getOccupancyResolution();
  //        const size_t  v0       = v / patch.getOccupancyResolution();
  //        const size_t  p0       = v0 * patch.getSizeU0() + u0;
  //        const int16_t minDepth = minPerBlock[p0];
  //        if ( ( depth0 - minDepth > 32 ) || ( surfaceThickness + depth0 > patch.getD1() + maxAllowedDepth ) ) {
  //          patch.getDepth( 0 )[p] = infiniteDepth;
  //          if ( useSurfaceSeparation ) { patch.getdepth0pccidx()[p] = infinitenumber; }
  //        }
  //      }
  //    }
  //  } else {
  //    std::vector<int16_t> minPerBlock;
  //    std::vector<int16_t> maxPerBlock;
  //    maxPerBlock.resize( patch.getSizeU0() * patch.getSizeV0(), 0 );
  //    minPerBlock.resize( patch.getSizeU0() * patch.getSizeV0(), infiniteDepth );
  //    for ( int64_t v = 0; v < int64_t( finalSizeV ); ++v ) {
  //      for ( int64_t u = 0; u < int64_t( finalSizeU ); ++u ) {
  //        const size_t  p        = v * finalSizeU + u;
  //        const int16_t depth0   = patch.getDepth( 0 )[p];
  //        const size_t  u0       = u / patch.getOccupancyResolution();
  //        const size_t  v0       = v / patch.getOccupancyResolution();
  //        const size_t  p0       = v0 * patch.getSizeU0() + u0;
  //        const int16_t minDepth = minPerBlock[p0];
  //        const int16_t maxDepth = maxPerBlock[p0];
  //        minPerBlock[p0]        = ( std::min )( minDepth, depth0 );
  //        if ( depth0 != infiniteDepth ) { maxPerBlock[p0] = ( std::max )( maxDepth, depth0 ); }
  //      }
  //    }
  //    for ( int64_t v = 0; v < int64_t( finalSizeV ); ++v ) {
  //      for ( int64_t u = 0; u < int64_t( finalSizeU ); ++u ) {
  //        const size_t  p        = v * finalSizeU + u;
  //        const int16_t depth0   = patch.getDepth( 0 )[p];
  //        const size_t  u0       = u / patch.getOccupancyResolution();
  //        const size_t  v0       = v / patch.getOccupancyResolution();
  //        const size_t  p0       = v0 * patch.getSizeU0() + u0;
  //        const int16_t maxDepth = maxPerBlock[p0];
  //        if ( maxDepth < 0 ) {
  //          patch.getDepth( 0 )[p] = infiniteDepth;
  //          if ( useSurfaceSeparation ) { patch.getdepth0pccidx()[p] = infinitenumber; }
  //        } else {
  //          if ( depth0 != infiniteDepth ) {
  //            int16_t tmp_a = maxDepth - depth0;
  //            int16_t tmp_b = depth0 - int16_t( surfaceThickness );
  //            int16_t tmp_c = int16_t( patch.getD1() ) - int16_t( maxAllowedDepth );
  //            if ( ( tmp_a > 32 ) || ( tmp_b < tmp_c ) ) {
  //              patch.getDepth( 0 )[p] = infiniteDepth;
  //              if ( useSurfaceSeparation ) patch.getdepth0pccidx()[p] = infinitenumber;
  //            }
  //          }
  //        }
  //      }
  //    }
  //  }
  //
  //  if ( EOMSingleLayerMode ) {
  //    // compute edd
  //    if ( patch.getProjectionMode() == 0 ) {  // min
  //      if ( !useSurfaceSeparation ) {
  //        for ( const auto i : connectedComponent ) {
  //          const auto&   point = points[i];
  //          const int16_t d     = int16_t( round( point[patch.getNormalAxis()] ) );
  //          const size_t  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
  //          const size_t  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
  //          assert( u >= 0 && u < patch.getSizeU() );
  //          assert( v >= 0 && v < patch.getSizeV() );
  //
  //          if ( checkPoint[v][u] == true ) {
  //            const size_t  nu     = round( u / (double)lodScaleX );
  //            const size_t  nv     = round( v / (double)lodScaleY );
  //            const size_t  np     = nv * finalSizeU + nu;
  //            const int16_t depth0 = patch.getDepth( 0 )[np];
  //            if ( useEnhancedDeltaDepthCode && depth0 < infiniteDepth && ( d - depth0 ) > 0 &&
  //                ( d - depth0 ) <= int16_t( EOMFixBitCount ) ) {
  //              const uint16_t oldEDDCode = patch.getDepthEnhancedDeltaD()[np];
  //              uint16_t       deltaD     = d - depth0;
  //              int            comp_depth0;
  //
  //              comp_depth0 = depth0 - patch.getD1();
  //
  //              patch.getDepthEnhancedDeltaD()[np] |= 1 << ( deltaD - 1 );
  //              if ( ( comp_depth0 + patch.getDepthEnhancedDeltaD()[np] ) > 1023 ) {
  //                patch.getDepthEnhancedDeltaD()[np] = oldEDDCode;
  //                std::cout
  //                << "(D0 + EDD-Code) > 1023. Data overflow observed (assume using 10bit coding). "
  //                "Temporary solution: the corresponding inbetween or Depth1 point will be regarded as "
  //                "missing point. To be improved if this happens a lot...\n";
  //              }
  //            }
  //          }
  //        }
  //      } else {  // Surface Separation
  //        bool   proc_d1_select_flag    = true;
  //        bool   err_flag               = false;
  //        size_t patch_surfaceThickness = surfaceThickness;
  //
  //        while ( proc_d1_select_flag ) {
  //          int64_t err_sum = 0;
  //          int32_t tot_num = 0;
  //          int32_t d1_num  = 0;
  //
  //          if ( err_flag == true ) { proc_d1_select_flag = false; }
  //          for ( const auto i : connectedComponent ) {
  //            const auto&   point = points[i];
  //            const int16_t d     = int16_t( round( point[patch.getNormalAxis()] ) );
  //            const size_t  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
  //            const size_t  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
  //            assert( u >= 0 && u < patch.getSizeU() );
  //            assert( v >= 0 && v < patch.getSizeV() );
  //
  //            if ( checkPoint[v][u] == true ) {
  //              const size_t  nu     = round( u / (double)lodScaleX );
  //              const size_t  nv     = round( v / (double)lodScaleY );
  //              const size_t  np     = nv * finalSizeU + nu;
  //              const int16_t depth0 = patch.getDepth( 0 )[np];
  //              tot_num++;
  //
  //              if ( err_flag ) {
  //                if ( useEnhancedDeltaDepthCode && depth0 < infiniteDepth && ( d - depth0 ) > 0 &&
  //                    ( d - depth0 ) <= int16_t( patch_surfaceThickness + EOMFixBitCount - surfaceThickness ) ) {
  //                  const uint16_t oldEDDCode = patch.getDepthEnhancedDeltaD()[np];
  //                  uint16_t       deltaD     = d - depth0;
  //                  int            comp_depth0;
  //
  //                  comp_depth0 = depth0 - patch.getD1();
  //
  //                  patch.getDepthEnhancedDeltaD()[np] |= 1 << ( deltaD - 1 );
  //                  if ( ( comp_depth0 + patch.getDepthEnhancedDeltaD()[np] ) > 1023 ) {
  //                    patch.getDepthEnhancedDeltaD()[np] = oldEDDCode;
  //                    std::cout
  //                    << "(D0 + EDD-Code) > 1023. Data overflow observed (assume using 10bit coding). "
  //                    "Temporary solution: the corresponding inbetween or Depth1 point will be regarded "
  //                    "as missing point. To be improved if this happens a lot...\n";
  //                  }
  //                }
  //              }
  //            }
  //          }
  //          double avg_error;
  //          if ( d1_num == 0 ) {
  //            avg_error = 0.0;
  //          } else {
  //            avg_error = (double)err_sum / (double)d1_num;
  //          }
  //          if ( avg_error < Threshold_Color_Error ) {
  //            err_flag = true;
  //          } else {
  //            patch_surfaceThickness--;
  //            if ( patch_surfaceThickness == 0 ) { break; }
  //          }
  //        }
  //      }
  //    } else {  // else ( patch.getProjectionMode() == 0 )
  //      if ( !useSurfaceSeparation ) {
  //        for ( const auto i : connectedComponent ) {
  //          const auto&   point = points[i];
  //          const int16_t d     = int16_t( round( point[patch.getNormalAxis()] ) );
  //          const size_t  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
  //          const size_t  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
  //          assert( u >= 0 && u < patch.getSizeU() );
  //          assert( v >= 0 && v < patch.getSizeV() );
  //
  //          if ( checkPoint[v][u] == true ) {
  //            const size_t  nu     = round( u / (double)lodScaleX );
  //            const size_t  nv     = round( v / (double)lodScaleY );
  //            const size_t  np     = nv * finalSizeU + nu;
  //            const int16_t depth0 = patch.getDepth( 0 )[np];
  //            if ( useEnhancedDeltaDepthCode && depth0 < infiniteDepth && ( depth0 - d ) > 0 &&
  //                ( depth0 - d ) <= int16_t( EOMFixBitCount ) ) {
  //              const uint16_t oldEDDCode = patch.getDepthEnhancedDeltaD()[np];
  //              uint16_t       deltaD     = depth0 - d;
  //              int            comp_depth0;
  //
  //              comp_depth0 = patch.getD1() - depth0;
  //
  //              patch.getDepthEnhancedDeltaD()[np] |= 1 << ( deltaD - 1 );
  //              if (( comp_depth0 + patch.getDepthEnhancedDeltaD()[np] ) > 1023 ) {
  //                patch.getDepthEnhancedDeltaD()[np] = oldEDDCode;
  //                std::cout
  //                << "(D0 + EDD-Code) > 1023. Data overflow observed (assume using 10bit coding). "
  //                "Temporary solution: the corresponding inbetween or Depth1 point will be regarded as "
  //                "missing point. To be improved if this happens a lot...\n";
  //              }
  //            }
  //          }
  //        }
  //      } else {  // Surface Separation
  //        bool   proc_d1_select_flag    = true;
  //        bool   err_flag               = false;
  //        size_t patch_surfaceThickness = surfaceThickness;
  //
  //        while ( proc_d1_select_flag ) {
  //          int64_t err_sum = 0;
  //          int32_t tot_num = 0;
  //          int32_t d1_num  = 0;
  //
  //          if ( err_flag == true ) { proc_d1_select_flag = false; }
  //
  //          for ( const auto i : connectedComponent ) {
  //            const auto&   point = points[i];
  //            const int16_t d     = int16_t( round( point[patch.getNormalAxis()] ) );
  //            const size_t  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
  //            const size_t  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
  //            assert( u >= 0 && u < patch.getSizeU() );
  //            assert( v >= 0 && v < patch.getSizeV() );
  //
  //            if ( checkPoint[v][u] == true ) {
  //              const size_t  nu     = round( u / (double)lodScaleX );
  //              const size_t  nv     = round( v / (double)lodScaleY );
  //              const size_t  np     = nv * finalSizeU + nu;
  //              const int16_t depth0 = patch.getDepth( 0 )[np];
  //              tot_num++;
  //
  //              if ( err_flag ) {
  //                if ( useEnhancedDeltaDepthCode && depth0 < infiniteDepth && ( depth0 - d ) > 0 &&
  //                    ( depth0 - d ) <= int16_t( EOMFixBitCount ) ) {
  //                  const uint16_t oldEDDCode = patch.getDepthEnhancedDeltaD()[np];
  //                  uint16_t       deltaD     = depth0 - d;
  //                  int            comp_depth0;
  //
  //                  comp_depth0 = patch.getD1() - depth0;
  //
  //                  patch.getDepthEnhancedDeltaD()[np] |= 1 << ( deltaD - 1 );
  //                  if ( ( comp_depth0 + patch.getDepthEnhancedDeltaD()[np] ) > 1023 ) {
  //                    patch.getDepthEnhancedDeltaD()[np] = oldEDDCode;
  //                    std::cout
  //                    << "(D0 + EDD-Code) > 1023. Data overflow observed (assume using 10bit coding). "
  //                    "Temporary solution: the corresponding inbetween or Depth1 point will be regarded "
  //                    "as missing point. To be improved if this happens a lot...\n";
  //                  }
  //                }
  //              }
  //            }
  //          }
  //          double avg_error;
  //          if ( d1_num == 0 ) {
  //            avg_error = 0.0;
  //          } else {
  //            avg_error = (double)err_sum / (double)d1_num;
  //          }
  //          if ( avg_error < Threshold_Color_Error ) {
  //            err_flag = true;
  //          } else {
  //            patch_surfaceThickness--;
  //            if ( patch_surfaceThickness == 0 ) { break; }
  //          }
  //        }
  //      }
  //    }       // fi ( patch.getProjectionMode() == 0 )
  //  } else {  // EOMSingleLayerMode
  //    // compute d1 map
  //    patch.getDepth( 1 ) = patch.getDepth( 0 );
  //    if ( patch.getProjectionMode() == 0 ) {  // min
  //      if ( !useSurfaceSeparation ) {
  //        for ( const auto i : connectedComponent ) {
  //          PCCPoint3D pointTmp = points[i];
  //          if ( bIsAdditionalProjectionPlane ) {
  //            auto& input = pointTmp;
  //            convert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, pointTmp );
  //          }
  //          const auto& point = pointTmp;
  //
  //          const int16_t d = int16_t( round( point[patch.getNormalAxis()] ) );
  //          const size_t  u = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
  //          const size_t  v = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
  //          assert( u >= 0 && u < patch.getSizeU() );
  //          assert( v >= 0 && v < patch.getSizeV() );
  //
  //          if ( checkPoint[v][u] == true ) {
  //            const size_t  nu     = round( u / (double)lodScaleX );
  //            const size_t  nv     = round( v / (double)lodScaleY );
  //            const size_t  np     = nv * finalSizeU + nu;
  //            const int16_t depth0 = patch.getDepth( 0 )[np];
  //
  //            if ( depth0 < infiniteDepth && ( d - depth0 ) <= int16_t( surfaceThickness ) &&
  //                d > patch.getDepth( 1 )[np] ) {
  //              patch.getDepth( 1 )[np] = d;
  //            }
  //            if ( useEnhancedDeltaDepthCode && depth0 < infiniteDepth && ( d - depth0 ) > 0 &&
  //                ( d - depth0 ) <= int16_t( surfaceThickness ) ) {
  //              const uint16_t oldEDDCode = patch.getDepthEnhancedDeltaD()[np];
  //              const uint16_t deltaD     = d - depth0;
  //              int            comp_depth0;
  //
  //              comp_depth0 = depth0 - patch.getD1();
  //
  //              patch.getDepthEnhancedDeltaD()[np] |= 1 << ( deltaD - 1 );
  //              if ( ( comp_depth0 + patch.getDepthEnhancedDeltaD()[np] ) > 1023 ) {
  //                patch.getDepthEnhancedDeltaD()[np] = oldEDDCode;
  //                std::cout
  //                << "(D0 + EDD-Code) > 1023. Data overflow observed (assume using 10bit coding). Temporary "
  //                "solution: the corresponding inbetween or Depth1 point will be regarded as missing point. "
  //                "To be improved if this happens a lot...\n";
  //              }
  //            }
  //            if ( patch.getDepth( 1 )[np] < patch.getDepth( 0 )[np] ) {
  //              std::cout << "3.compute d1 map : ERROR : proj0 and d1 < d0" << std::endl;
  //            }
  //          }
  //        }
  //      } else {  // Surface Separation
  //        bool   proc_d1_select_flag    = true;
  //        bool   err_flag               = false;
  //        size_t patch_surfaceThickness = surfaceThickness;
  //
  //        while ( proc_d1_select_flag ) {
  //          int64_t err_sum = 0;
  //          int32_t tot_num = 0;
  //          int32_t d1_num  = 0;
  //
  //          if ( err_flag == true ) { proc_d1_select_flag = false; }
  //          for ( const auto i : connectedComponent ) {
  //            PCCPoint3D pointTmp = points[i];
  //            if ( bIsAdditionalProjectionPlane ) {
  //              auto& input = pointTmp;
  //              convert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, pointTmp );
  //            }
  //            const auto&   point = pointTmp;
  //            const int16_t d     = int16_t( round( point[patch.getNormalAxis()] ) );
  //            const size_t  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
  //            const size_t  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
  //            assert( u >= 0 && u < patch.getSizeU() );
  //            assert( v >= 0 && v < patch.getSizeV() );
  //
  //            if ( checkPoint[v][u] == true ) {
  //              const size_t  nu     = round( u / (double)lodScaleX );
  //              const size_t  nv     = round( v / (double)lodScaleY );
  //              const size_t  np     = nv * finalSizeU + nu;
  //              const int16_t depth0 = patch.getDepth( 0 )[np];
  //
  //              tot_num++;
  //
  //              if ( err_flag == false ) {
  //                if ( depth0 < infiniteDepth && ( d - depth0 ) <= int16_t( patch_surfaceThickness ) &&
  //                    d > patch.getDepth( 1 )[np] ) {
  //                  d1_num++;
  //                  const size_t     d0_idx   = patch.getdepth0pccidx()[np];
  //                  const size_t     d1_idx   = i;
  //                  const PCCColor3B D0_Color = frame_pcc_color[d0_idx];
  //                  const PCCColor3B D1_Color = frame_pcc_color[d1_idx];
  //
  //                  const uint8_t r0 = D0_Color[0];
  //                  const uint8_t g0 = D0_Color[1];
  //                  const uint8_t b0 = D0_Color[2];
  //
  //                  const uint8_t r1 = D1_Color[0];
  //                  const uint8_t g1 = D1_Color[1];
  //                  const uint8_t b1 = D1_Color[2];
  //
  //                  int32_t delta_R = int32_t( r0 ) - int32_t( r1 );
  //                  int32_t delta_G = int32_t( g0 ) - int32_t( g1 );
  //                  int32_t delta_B = int32_t( b0 ) - int32_t( b1 );
  //
  //                  int64_t delta_e = delta_R * delta_R + delta_G * delta_G + delta_B * delta_B;
  //
  //                  err_sum += delta_e;
  //                }
  //              } else {
  //                if ( depth0 < infiniteDepth && ( d - depth0 ) <= int16_t( patch_surfaceThickness ) &&
  //                    d > patch.getDepth( 1 )[np] ) {
  //                  patch.getDepth( 1 )[np] = d;
  //                }
  //                if ( useEnhancedDeltaDepthCode && depth0 < infiniteDepth && ( d - depth0 ) > 0 &&
  //                    ( d - depth0 ) <= int16_t( patch_surfaceThickness ) ) {
  //                  const uint16_t oldEDDCode = patch.getDepthEnhancedDeltaD()[np];
  //                  const uint16_t deltaD     = d - depth0;
  //                  int            comp_depth0;
  //
  //                  comp_depth0 = depth0 - patch.getD1();
  //
  //                  patch.getDepthEnhancedDeltaD()[np] |= 1 << ( deltaD - 1 );
  //                  if ( ( comp_depth0 + patch.getDepthEnhancedDeltaD()[np] ) > 1023 ) {
  //                    patch.getDepthEnhancedDeltaD()[np] = oldEDDCode;
  //                    std::cout
  //                    << "(D0 + EDD-Code) > maxD. Data overflow observed (assume using 10bit coding). "
  //                    "Temporary solution: the corresponding inbetween or Depth1 point will be regarded as "
  //                    "missing point. To be improved if this happens a lot...\n";
  //                  }
  //                }
  //                if ( patch.getDepth( 1 )[np] < patch.getDepth( 0 )[np] ) {
  //                  std::cout << "4.compute d1 map : ERROR : proj0 and d1 < d0" << std::endl;
  //                }
  //              }
  //            }
  //          }
  //          double avg_error;
  //          if ( d1_num == 0 ) {
  //            avg_error = 0.0;
  //          } else {
  //            avg_error = (double)err_sum / (double)d1_num;
  //          }
  //          if ( avg_error < Threshold_Color_Error ) {
  //            err_flag = true;
  //          } else {
  //            patch_surfaceThickness--;
  //            if ( patch_surfaceThickness == 0 ) { break; }
  //          }
  //        }
  //      }
  //    } else {  // else ( patch.getProjectionMode() == 0 )
  //      if ( !useSurfaceSeparation ) {
  //        for ( const auto i : connectedComponent ) {
  //          PCCPoint3D pointTmp = points[i];
  //          if ( bIsAdditionalProjectionPlane ) {
  //            auto& input = pointTmp;
  //            convert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, pointTmp );
  //          }
  //          const auto&   point = pointTmp;
  //          const int16_t d     = int16_t( round( point[patch.getNormalAxis()] ) );
  //          const size_t  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
  //          const size_t  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
  //          assert( u >= 0 && u < patch.getSizeU() );
  //          assert( v >= 0 && v < patch.getSizeV() );
  //
  //          if ( checkPoint[v][u] == true ) {
  //            const size_t  nu     = round( u / (double)lodScaleX );
  //            const size_t  nv     = round( v / (double)lodScaleY );
  //            const size_t  np     = nv * finalSizeU + nu;
  //            const int16_t depth0 = patch.getDepth( 0 )[np];
  //
  //            if ( depth0 < infiniteDepth && ( depth0 - d ) <= int16_t( surfaceThickness ) &&
  //                d < patch.getDepth( 1 )[np] ) {
  //              patch.getDepth( 1 )[np] = d;
  //            }
  //            if ( useEnhancedDeltaDepthCode && depth0 < infiniteDepth && ( depth0 - d ) > 0 &&
  //                ( depth0 - d ) <= int16_t( surfaceThickness ) ) {
  //              const uint16_t oldEDDCode = patch.getDepthEnhancedDeltaD()[np];
  //              const uint16_t deltaD     = depth0 - d;
  //              int            comp_depth0;
  //
  //              comp_depth0 = patch.getD1() - depth0;
  //
  //              patch.getDepthEnhancedDeltaD()[np] |= 1 << ( deltaD - 1 );
  //              if ( ( comp_depth0 + patch.getDepthEnhancedDeltaD()[np] ) > 1023 ) {
  //                patch.getDepthEnhancedDeltaD()[np] = oldEDDCode;
  //                std::cout
  //                << "(D0 + EDD-Code) > 1023. Data overflow observed (assume using 10bit coding). Temporary "
  //                "solution: the corresponding inbetween or Depth1 point will be regarded as missing point. "
  //                "To be improved if this happens a lot...\n";
  //              }
  //            }
  //          }
  //        }
  //      } else {  // Surface Separation
  //        bool   proc_d1_select_flag    = true;
  //        bool   err_flag               = false;
  //        size_t patch_surfaceThickness = surfaceThickness;
  //
  //        while ( proc_d1_select_flag ) {
  //          int64_t err_sum = 0;
  //          int32_t tot_num = 0;
  //          int32_t d1_num  = 0;
  //
  //          if ( err_flag == true ) { proc_d1_select_flag = false; }
  //
  //          for ( const auto i : connectedComponent ) {
  //            PCCPoint3D pointTmp = points[i];
  //            if ( bIsAdditionalProjectionPlane ) {
  //              auto& input = pointTmp;
  //              convert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, pointTmp );
  //            }
  //            const auto&   point = pointTmp;
  //            const int16_t d     = int16_t( round( point[patch.getNormalAxis()] ) );
  //            const size_t  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
  //            const size_t  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
  //            assert( u >= 0 && u < patch.getSizeU() );
  //            assert( v >= 0 && v < patch.getSizeV() );
  //
  //            if ( checkPoint[v][u] == true ) {
  //              const size_t  nu     = round( u / (double)lodScaleX );
  //              const size_t  nv     = round( v / (double)lodScaleY );
  //              const size_t  np     = nv * finalSizeU + nu;
  //              const int16_t depth0 = patch.getDepth( 0 )[np];
  //              tot_num++;
  //
  //              if ( err_flag == false ) {
  //                if ( depth0 < infiniteDepth && ( depth0 - d ) <= int16_t( patch_surfaceThickness ) &&
  //                    d < patch.getDepth( 1 )[np] ) {
  //                  d1_num++;
  //                  const size_t     d0_idx   = patch.getdepth0pccidx()[np];
  //                  const size_t     d1_idx   = i;
  //                  const PCCColor3B D0_Color = frame_pcc_color[d0_idx];
  //                  const PCCColor3B D1_Color = frame_pcc_color[d1_idx];
  //                  int32_t          delta_R  = int32_t( D0_Color[0] ) - int32_t( D1_Color[0] );
  //                  int32_t          delta_G  = int32_t( D0_Color[1] ) - int32_t( D1_Color[1] );
  //                  int32_t          delta_B  = int32_t( D0_Color[2] ) - int32_t( D1_Color[2] );
  //                  int64_t          delta_e  = delta_R * delta_R + delta_G * delta_G + delta_B * delta_B;
  //                  err_sum += delta_e;
  //                }
  //              } else {
  //                if ( depth0 < infiniteDepth && ( depth0 - d ) <= int16_t( patch_surfaceThickness ) &&
  //                    d < patch.getDepth( 1 )[np] ) {
  //                  patch.getDepth( 1 )[np] = d;
  //                }
  //                if ( useEnhancedDeltaDepthCode && depth0 < infiniteDepth && ( depth0 - d ) > 0 &&
  //                    ( depth0 - d ) <= int16_t( surfaceThickness ) ) {
  //                  const uint16_t oldEDDCode = patch.getDepthEnhancedDeltaD()[np];
  //                  const uint16_t deltaD     = depth0 - d;
  //                  int            comp_depth0;
  //
  //                  comp_depth0 = patch.getD1() - depth0;
  //
  //                  patch.getDepthEnhancedDeltaD()[np] |= 1 << ( deltaD - 1 );
  //                  if ( ( comp_depth0 + patch.getDepthEnhancedDeltaD()[np] ) > 1023 ) {
  //                    patch.getDepthEnhancedDeltaD()[np] = oldEDDCode;
  //                    std::cout
  //                    << "(D0 + EDD-Code) > 1023. Data overflow observed (assume using 10bit coding). "
  //                    "Temporary solution: the corresponding inbetween or Depth1 point will be regarded as "
  //                    "missing point. To be improved if this happens a lot...\n";
  //                  }
  //                }
  //              }
  //            }
  //          }
  //          double avg_error;
  //          if ( d1_num == 0 ) {
  //            avg_error = 0.0;
  //          } else {
  //            avg_error = (double)err_sum / (double)d1_num;
  //          }
  //          if ( avg_error < Threshold_Color_Error ) {
  //            err_flag = true;
  //          } else {
  //            patch_surfaceThickness--;
  //            if ( patch_surfaceThickness == 0 ) { break; }
  //          }
  //        }
  //      }
  //    }  // fi ( patch.getProjectionMode() == 0 )
  //  }
  //
  //  patch.getSizeD() = 0;
  //
  //  patch.getSizeU() = finalSizeU;
  //  patch.getSizeV() = finalSizeV;
  //
  //  for ( size_t i = 0; i < originalSizeV; i++ ) checkPoint[i].clear();
  //  checkPoint.clear();
  //
  //  if ( patch.getProjectionMode() == 0 ) {
  //    for ( size_t v = 0; v < patch.getSizeV(); ++v ) {
  //      for ( size_t u = 0; u < patch.getSizeU(); ++u ) {
  //        const size_t p      = v * patch.getSizeU() + u;
  //        int16_t      depth0 = patch.getDepth( 0 )[p];
  //        if ( depth0 < infiniteDepth ) {
  //          depth0 -= int16_t( patch.getD1() );
  //          assert( depth0 >= 0 );
  //          patch.getDepth( 0 )[p] = depth0;
  //          patch.getSizeD() =
  //          ( std::max )( patch.getSizeD(), static_cast<size_t>( depth0 ) );  // compute max depth
  //
  //          const size_t u0 = u / patch.getOccupancyResolution();
  //          const size_t v0 = v / patch.getOccupancyResolution();
  //          const size_t p0 = v0 * patch.getSizeU0() + u0;
  //          assert( u0 >= 0 && u0 < patch.getSizeU0() );
  //          assert( v0 >= 0 && v0 < patch.getSizeV0() );
  //          patch.getOccupancy()[p0] = true;
  //          if ( !EOMSingleLayerMode ) { patch.getDepth( 1 )[p] -= int16_t( patch.getD1() ); }
  //
  //          if ( useEnhancedDeltaDepthCode ) {
  //          } else {  // if(useEnhancedDeltaDepthCode)
  //            int16_t depth1 = patch.getDepth( 1 )[p];
  //            patch.getSizeD() =
  //            ( std::max )( patch.getSizeD(), static_cast<size_t>( depth1 ) );  // compute max depth
  //          }
  //        }  // if(useEnhancedDeltaDepthCode)
  //      }
  //    }
  //  } else {
  //
  //    for ( size_t v = 0; v < patch.getSizeV(); ++v ) {
  //      for ( size_t u = 0; u < patch.getSizeU(); ++u ) {
  //        const size_t p        = v * patch.getSizeU() + u;
  //        int16_t      depth0_1 = patch.getDepth( 0 )[p];
  //        if ( depth0_1 < infiniteDepth ) {
  //          int16_t depth0 = int16_t( patch.getD1() ) - depth0_1;
  //          assert( depth0 >= 0 );
  //          patch.getDepth( 0 )[p] = depth0;
  //          patch.getSizeD() =
  //          ( std::max )( patch.getSizeD(), static_cast<size_t>( depth0 ) );  // compute max depth
  //
  //          const size_t u0 = u / patch.getOccupancyResolution();
  //          const size_t v0 = v / patch.getOccupancyResolution();
  //          const size_t p0 = v0 * patch.getSizeU0() + u0;
  //          assert( u0 >= 0 && u0 < patch.getSizeU0() );
  //          assert( v0 >= 0 && v0 < patch.getSizeV0() );
  //          patch.getOccupancy()[p0] = true;
  //          int16_t depth1_1         = 0;
  //          if ( !EOMSingleLayerMode ) {
  //            depth1_1               = patch.getDepth( 1 )[p];
  //            patch.getDepth( 1 )[p] = int16_t( patch.getD1() ) - depth1_1;
  //          }
  //
  //          if ( useEnhancedDeltaDepthCode ) {
  //          } else {
  //            assert( abs( patch.getDepth( 0 )[p] - patch.getDepth( 1 )[p] ) <= int( surfaceThickness ) );
  //
  //            int16_t depth1_2 = patch.getDepth( 1 )[p];
  //            patch.getSizeD() =
  //            ( std::max )( patch.getSizeD(), static_cast<size_t>( depth1_2 ) );  // compute max depth
  //          }
  //        }
  //      }
  //    }
  //  }
}

void PCCPatchSegmenter3::segmentPatches( const PCCPointSet3&         points,
                                         const size_t                frameIndex,
                                         const PCCKdTree&            kdtree,
                                         const size_t                maxNNCount,
                                         const size_t                minPointCountPerCC,
                                         const size_t                occupancyResolution,
                                         const double                maxAllowedDist2MissedPointsDetection,
                                         const double                maxAllowedDist2MissedPointsSelection,
                                         const bool                  EOMSingleLayerMode,
                                         const size_t                EOMFixBitCount,
                                         const size_t                surfaceThickness,
                                         const size_t                maxAllowedDepth,
                                         const size_t                minLevel,
                                         std::vector<size_t>&        partition,
                                         std::vector<PCCPatch>&      patches,
                                         std::vector<size_t>&        patchPartition,
                                         std::vector<size_t>&        resampledPatchPartition,
                                         std::vector<size_t>         missedPoints,
                                         PCCPointSet3&               resampled,
                                         bool                        useEnhancedDeltaDepthCode,
                                         const bool                  createSubPointCloud,
                                         std::vector<PCCPointSet3>&  subPointCloud,
                                         float&                      distanceSrcRec,
                                         const bool                  absoluteD1,
                                         bool                        useSurfaceSeparation,
                                         const size_t                additionalProjectionAxis,
                                         const size_t                geometryBitDepth3D,
                                         const size_t                testLevelOfDetail,
                                         bool                        patchExpansionEnabled,
                                         const bool                  highGradientSeparation,
                                         const double                minGradient,
                                         const size_t                minNumHighGradientPoints,
                                         const PCCNormalsGenerator3& normalsGen,
                                         const PCCVector3D*          orientations,
                                         const size_t                orientationCount,
                                         bool                        enablePointCloudPartitioning,
                                         std::vector<int>&           roiBoundingBoxMinX,
                                         std::vector<int>&           roiBoundingBoxMaxX,
                                         std::vector<int>&           roiBoundingBoxMinY,
                                         std::vector<int>&           roiBoundingBoxMaxY,
                                         std::vector<int>&           roiBoundingBoxMinZ,
                                         std::vector<int>&           roiBoundingBoxMaxZ,
                                         int                         numCutsAlong1stLongestAxis,
                                         int                         numCutsAlong2ndLongestAxis,
                                         int                         numCutsAlong3rdLongestAxis ) {
  const size_t pointCount = points.getPointCount();
  patchPartition.resize( pointCount, 0 );
  resampledPatchPartition.reserve( pointCount );

  PCCNNResult             result;
  const double            Threshold_Color_Error = 400;
  std::vector<PCCColor3B> frame_pcc_color;
  PCCColor3B              RGB_val;

  // jkei: do we really need this? how about points.getColor()??
#ifdef DEBUG_SURFACE_SEPARATION
  if ( useSurfaceSeparation ) 
#endif
  {
    frame_pcc_color.reserve( pointCount );
    for ( size_t i = 0; i < pointCount; i++ ) {
      RGB_val = points.getColor( i );
      frame_pcc_color.push_back( RGB_val );
    }
  }
  if ( enablePointCloudPartitioning ) assert( patchExpansionEnabled == false );

  size_t numD0Points      = 0;
  size_t numD1Points      = 0;
  size_t numEDDonlyPoints = 0;
  // size_t numRawPoints=0;
  std::cout << "\n\t Computing adjacency info... ";
  std::vector<std::vector<size_t>>              adj;
  std::vector<std::vector<double>>              adjDist;
  std::vector<bool>                             flagExp;
  int                                           numROIs;
  int                                           numChunks;
  std::vector<PCCPointSet3>                     pointsChunks;
  std::vector<std::vector<size_t>>              pointsIndexChunks;
  std::vector<size_t>                           pointCountChunks;
  std::vector<PCCKdTree>                        kdtreeChunks;
  std::vector<PCCBox3D>                         boundingBoxChunks;
  std::vector<std::vector<std::vector<size_t>>> adjChunks;
  if ( patchExpansionEnabled ) {
    computeAdjacencyInfoDist( points, kdtree, adj, adjDist, maxNNCount );
    flagExp.resize( pointCount, false );
  } else {
    if ( !enablePointCloudPartitioning ) {
      computeAdjacencyInfo( points, kdtree, adj, maxNNCount );
    } else {
      numROIs = static_cast<int>( roiBoundingBoxMinX.size() );
      std::vector<std::vector<size_t>> numCutsPerAxis;  // number of cuts per axis for each ROI
      numCutsPerAxis.resize( numROIs );
      for ( auto& nCut : numCutsPerAxis ) { nCut.resize( 3 ); }

      std::vector<std::vector<double>> cutSizePerAxis;  // size of cut per axis for each ROI
      cutSizePerAxis.resize( numROIs );
      for ( auto& szCut : cutSizePerAxis ) { szCut.resize( 3 ); }

      // cutRangesPerAxis[r][x][i]: i-th cut-range of x-th axis of r-th ROI
      std::vector<std::vector<std::vector<Range>>> cutRangesPerAxis;
      cutRangesPerAxis.resize( numROIs );
      for ( auto& cutRng : cutRangesPerAxis ) { cutRng.resize( 3 ); }

      std::vector<std::vector<Range>> chunks;  // chunks[c][x]: range of x-th axis of c-th chunk
      // cut each ROI into chunks
      // for each ROI, sort axes according to their length and cut w.r.t to
      // numCutsAlong[1st,2nd,3rd]LongestAxis
      for ( int roiIndex = 0; roiIndex < numROIs; ++roiIndex ) {
        // derive tight ROI bounding box
        int x_min, x_max, y_min, y_max, z_min, z_max;
        x_min = y_min = z_min = ( std::numeric_limits<int>::max )();
        x_max = y_max = z_max = ( std::numeric_limits<int>::min )();
        for ( int i = 0; i < pointCount; ++i ) {
          auto x = points[i][0];
          auto y = points[i][1];
          auto z = points[i][2];
          if ( ( roiBoundingBoxMinX[roiIndex] <= x && x <= roiBoundingBoxMaxX[roiIndex] ) &&
               ( roiBoundingBoxMinY[roiIndex] <= y && y <= roiBoundingBoxMaxY[roiIndex] ) &&
               ( roiBoundingBoxMinZ[roiIndex] <= z && z <= roiBoundingBoxMaxZ[roiIndex] ) ) {
            x_min = ( x < x_min ) ? x : x_min;
            y_min = ( y < y_min ) ? y : y_min;
            z_min = ( z < z_min ) ? z : z_min;
            x_max = ( x > x_max ) ? x : x_max;
            y_max = ( y > y_max ) ? y : y_max;
            z_max = ( z > z_max ) ? z : z_max;
          }
        }
        // update user-specified ROI bounding box to the actual bounding box
        roiBoundingBoxMinX[roiIndex] = x_min;
        roiBoundingBoxMaxX[roiIndex] = x_max;
        roiBoundingBoxMinY[roiIndex] = y_min;
        roiBoundingBoxMaxY[roiIndex] = y_max;
        roiBoundingBoxMinZ[roiIndex] = z_min;
        roiBoundingBoxMaxZ[roiIndex] = z_max;

        // sort x,y,z axes w.r.t the length of ROI bounding box along those axes
        std::vector<int> delta( 3 );
        delta[0]        = roiBoundingBoxMaxX[roiIndex] - roiBoundingBoxMinX[roiIndex] + 1;
        delta[1]        = roiBoundingBoxMaxY[roiIndex] - roiBoundingBoxMinY[roiIndex] + 1;
        delta[2]        = roiBoundingBoxMaxZ[roiIndex] - roiBoundingBoxMinZ[roiIndex] + 1;
        using AxisDelta = std::pair<int, int>;
        std::vector<AxisDelta> axisDelta;
        axisDelta.resize( 3 );
        for ( int k = 0; k < 3; ++k ) { axisDelta[k] = AxisDelta( {k, delta[k]} ); }

        sort( axisDelta.begin(), axisDelta.end(), []( AxisDelta a, AxisDelta b ) { return a.second > b.second; } );

        std::cout << "\n1st longest axis of ROI " << roiIndex << " is: " << axisDelta[0].first;
        std::cout << "\n2nd longest axis of ROI " << roiIndex << " is: " << axisDelta[1].first;
        std::cout << "\n3nd longest axis of ROI " << roiIndex << " is: " << axisDelta[2].first;

        // specify number of cuts per x,y,z axis
        numCutsPerAxis[roiIndex][axisDelta[0].first] = numCutsAlong1stLongestAxis;
        numCutsPerAxis[roiIndex][axisDelta[1].first] = numCutsAlong2ndLongestAxis;
        numCutsPerAxis[roiIndex][axisDelta[2].first] = numCutsAlong3rdLongestAxis;

        for ( size_t k = 0; k < 3; ++k ) {
          cutSizePerAxis[roiIndex][k] = double( delta[k] ) / double( numCutsPerAxis[roiIndex][k] );
        }

        std::vector<int> shift{roiBoundingBoxMinX[roiIndex], roiBoundingBoxMinY[roiIndex],
                               roiBoundingBoxMinZ[roiIndex]};

        for ( size_t axis = 0; axis < 3; ++axis ) {
          std::vector<Range> cutRanges;
          for ( size_t i = 0; i < numCutsPerAxis[roiIndex][axis]; ++i ) {
            Range range;
            range.first  = int( round( i * cutSizePerAxis[roiIndex][axis] ) ) + shift[axis];
            range.second = int( round( ( i + 1 ) * cutSizePerAxis[roiIndex][axis] - 1 ) ) + shift[axis];
            cutRanges.push_back( range );
          }
          cutRangesPerAxis[roiIndex][axis] = cutRanges;
        }

        for ( auto& rangeX : cutRangesPerAxis[roiIndex][0] ) {
          for ( auto& rangeY : cutRangesPerAxis[roiIndex][1] ) {
            for ( auto& rangeZ : cutRangesPerAxis[roiIndex][2] ) {
              chunks.push_back( std::vector<Range>( {rangeX, rangeY, rangeZ} ) );
            }
          }
        }
      }

      numChunks = (int)chunks.size();

      for ( int i = 0; i < numChunks; ++i ) {
        std::cout << "Chunk " << i << " : " << std::endl;
        printChunk( chunks[i] );
      }

      pointsChunks.resize( numChunks );
      pointsIndexChunks.resize( numChunks );
      pointCountChunks.resize( numChunks );
      kdtreeChunks.resize( numChunks );
      boundingBoxChunks.resize( numChunks );

      for ( size_t chunkIndex = 0; chunkIndex < numChunks; ++chunkIndex ) {
        auto& boundingBox   = boundingBoxChunks[chunkIndex];
        boundingBox.min_[0] = chunks[chunkIndex][0].first;
        boundingBox.max_[0] = chunks[chunkIndex][0].second;
        boundingBox.min_[1] = chunks[chunkIndex][1].first;
        boundingBox.max_[1] = chunks[chunkIndex][1].second;
        boundingBox.min_[2] = chunks[chunkIndex][2].first;
        boundingBox.max_[2] = chunks[chunkIndex][2].second;
      }

      for ( size_t i = 0; i < pointCount; ++i ) {
        for ( size_t chunkIndex = 0; chunkIndex < numChunks; ++chunkIndex ) {
          const auto& boundingBox = boundingBoxChunks[chunkIndex];
          if ( boundingBox.fullyContains( points[i] ) ) {
            pointsChunks[chunkIndex].addPoint( points[i] );
            pointsIndexChunks[chunkIndex].push_back( i );
          }
        }
      }

      for ( size_t chunkIndex = 0; chunkIndex < numChunks; ++chunkIndex ) {
        pointCountChunks[chunkIndex] = pointsChunks[chunkIndex].getPointCount();
      }

      for ( size_t chunkIndex = 0; chunkIndex < numChunks; ++chunkIndex ) {
        kdtreeChunks[chunkIndex].init( pointsChunks[chunkIndex] );
      }

      adjChunks.resize( numChunks );
      for ( size_t chunkIndex = 0; chunkIndex < numChunks; ++chunkIndex ) {
        std::cout << "\n\t Computing adjacency info for chunk " << chunkIndex << "... ";
        computeAdjacencyInfo( pointsChunks[chunkIndex], kdtreeChunks[chunkIndex], adjChunks[chunkIndex], maxNNCount );
        std::cout << "[done]" << std::endl;
      }
    }
  }
  std::cout << "[done]" << std::endl;

  std::cout << "\n\t Extracting patches... " << points.getPointCount() << std::endl;
  std::vector<double> missedPointsDistance;
  missedPoints.resize( pointCount );
  missedPointsDistance.resize( pointCount );
  for ( size_t i = 0; i < pointCount; ++i ) {
    missedPoints[i]         = i;
    missedPointsDistance[i] = ( std::numeric_limits<double>::max )();
  }
  std::vector<std::vector<size_t>> missedPointsChunks;
  std::vector<std::vector<double>> missedPointsDistanceChunks;
  if ( enablePointCloudPartitioning ) {
    missedPointsChunks.resize( numChunks );
    missedPointsDistanceChunks.resize( numChunks );
    for ( size_t chunkIndex = 0; chunkIndex < numChunks; ++chunkIndex ) {
      missedPointsChunks[chunkIndex].resize( pointCountChunks[chunkIndex] );
      missedPointsDistanceChunks[chunkIndex].resize( pointCountChunks[chunkIndex] );
    }

    for ( size_t chunkIndex = 0; chunkIndex < numChunks; ++chunkIndex ) {
      for ( size_t i = 0; i < pointCountChunks[chunkIndex]; ++i ) {
        missedPointsChunks[chunkIndex][i]         = i;
        missedPointsDistanceChunks[chunkIndex][i] = ( std::numeric_limits<double>::max )();
      }
    }
  }
  subPointCloud.clear();
  double   meanPAB = 0.0, meanYAB = 0.0, meanUAB = 0.0, meanVAB = 0.0;
  double   meanPBA = 0.0, meanYBA = 0.0, meanUBA = 0.0, meanVBA = 0.0;
  size_t   testSrcNum = 0, testRecNum = 0;
  size_t   numberOfEDD = 0;
  uint16_t maxD        = 1 << geometryBitDepth3D;
  while ( !missedPoints.empty() ) {
    std::vector<std::vector<size_t>> connectedComponents;
    if ( !enablePointCloudPartitioning ) {
      std::vector<size_t> fifo;
      fifo.reserve( pointCount );
      std::vector<bool> flags;
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
    } else {
      std::vector<std::vector<std::vector<size_t>>> connectedComponentsChunks( numChunks );  // CC's per chunk
      // extract connected components of chunks
      for ( size_t chunkIndex = 0; chunkIndex < numChunks; ++chunkIndex ) {
        std::cout << "\n\t Extracting connected components of chunk " << chunkIndex << "... ";
        std::vector<size_t> fifo;
        fifo.reserve( pointCountChunks[chunkIndex] );
        std::vector<bool> flags;
        flags.resize( pointCountChunks[chunkIndex], false );
        for ( const auto i : missedPointsChunks[chunkIndex] ) { flags[i] = true; }
        connectedComponentsChunks[chunkIndex].reserve( 256 );
        for ( const auto i : missedPointsChunks[chunkIndex] ) {
          if ( flags[i] && missedPointsDistanceChunks[chunkIndex][i] > maxAllowedDist2MissedPointsDetection ) {
            flags[i]                  = false;
            const size_t indexCC      = connectedComponentsChunks[chunkIndex].size();
            const size_t clusterIndex = partition[pointsIndexChunks[chunkIndex][i]];
            connectedComponentsChunks[chunkIndex].resize( indexCC + 1 );
            std::vector<size_t>& connectedComponentChunk = connectedComponentsChunks[chunkIndex][indexCC];
            fifo.push_back( i );
            connectedComponentChunk.push_back( i );
            while ( !fifo.empty() ) {
              const size_t current = fifo.back();
              fifo.pop_back();
              for ( const auto n : adjChunks[chunkIndex][current] ) {
                if ( clusterIndex == partition[pointsIndexChunks[chunkIndex][n]] && flags[n] ) {
                  flags[n] = false;
                  fifo.push_back( n );
                  connectedComponentChunk.push_back( n );
                }
              }
            }
            if ( connectedComponentChunk.size() < minPointCountPerCC ) {
              connectedComponentsChunks[chunkIndex].resize( indexCC );
            } else {
              std::cout << "\t\t CC " << indexCC << " -> " << connectedComponentChunk.size() << std::endl;
            }
          }
        }
        std::cout << "[done]" << std::endl;
      }

      std::cout << "\n\t merge connected components of all chunks... ";
      // convert connected component indexes of chunks to original indexes
      for ( int chunkIndex = 0; chunkIndex < numChunks; ++chunkIndex ) {
        for ( auto& connectedComponent : connectedComponentsChunks[chunkIndex] ) {
          for ( size_t i = 0; i < connectedComponent.size(); ++i ) {
            connectedComponent[i] = pointsIndexChunks[chunkIndex][connectedComponent[i]];
          }
        }
      }
      // merge connected components of chunks into connectedComponents
      // std::vector<std::vector<size_t>> connectedComponents;  // final connected components that include CC's of all
      // chunks
      connectedComponents.resize( 0 );
      for ( size_t chunkIndex = 0; chunkIndex < numChunks; ++chunkIndex ) {
        for ( auto& connectedComponent : connectedComponentsChunks[chunkIndex] ) {
          connectedComponents.push_back( connectedComponent );
        }
      }
      std::cout << "\n\t number of merged conncected components = " << connectedComponents.size();
      std::cout << "[done]" << std::endl;
    }
    if ( connectedComponents.empty() ) { break; }
    if ( highGradientSeparation ) {
      separateHighGradientPoints( points, additionalProjectionAxis, absoluteD1, normalsGen, orientations,
                                  orientationCount, surfaceThickness, geometryBitDepth3D, minGradient,
                                  minNumHighGradientPoints, partition, adj, connectedComponents );
    }
    if ( patchExpansionEnabled ) {
      std::sort( connectedComponents.begin(), connectedComponents.end(),
                 []( const std::vector<size_t>& a, const std::vector<size_t>& b ) {
                   return a.size() >= b.size();
                 } );  // ascending: <=, descending: >=
    }
    bool bIsAdditionalProjectionPlane;
    for ( auto& connectedComponent : connectedComponents ) {
      const size_t patchIndex = patches.size();
      patches.resize( patchIndex + 1 );
      if ( createSubPointCloud ) { subPointCloud.resize( patchIndex + 1 ); }
      PCCPatch& patch         = patches[patchIndex];
      patch.getIndex()        = patchIndex;
      size_t d0CountPerPatch  = 0;
      size_t d1CountPerPatch  = 0;
      size_t eddCountPerPatch = 0;
      patch.setEddCount( 0 );
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

      patch.setViewId() = clusterIndex;
      patch.setBestMatchIdx( InvalidPatchIndex );

      patch.getPreGPAPatchData().initialize();
      patch.getCurGPAPatchData().initialize();

      if ( clusterIndex <= 2 ) {
        patch.getProjectionMode() = 0;
      } else {
        patch.getProjectionMode() = 1;
      }

      if ( absoluteD1 ) {
        // for additional projection plane
        if ( clusterIndex == 6 || clusterIndex == 7 ) { patch.getProjectionMode() = 0; }
        // for additional projection plane
        if ( clusterIndex == 10 || clusterIndex == 11 || clusterIndex == 14 || clusterIndex == 15 ) {
          patch.getProjectionMode() = 0;
        }
      }

      if ( patchExpansionEnabled ) {
        for ( const auto i : connectedComponent ) flagExp[i] = true;  // add point

        std::vector<size_t> fifoa;
        fifoa.reserve( pointCount );
        for ( const auto i : connectedComponent ) {
          for ( size_t ac = 0; ac < adj[i].size(); ac++ ) {
            const size_t n = adj[i][ac];
            if ( flagExp[n] ) continue;  // point been added in expansion

            if ( ( clusterIndex == partition[n] ) ||  // same plane
                 ( clusterIndex + 3 == partition[n] ) || ( clusterIndex == partition[n] + 3 ) )
              continue;

            // auto colorDiff = points.getColor(i) - points.getColor(n);
            // auto colorDist = std::abs(colorDiff[0]) + std::abs(colorDiff[1]) + std::abs(colorDiff[2]);
            // if (flagExp[n] && colorDist > 63)
            //  continue;

            const double dist2 = adjDist[i][ac];  // sum of square
            if ( dist2 <= 2 ) {                   // <-- expansion distance
              fifoa.push_back( n );
              flagExp[n] = true;  // add point
            }
          }
        }
        // std::cout << "\t\t\t expand patch " << patchIndex << ": " << connectedComponent.size() << " + " <<
        // fifoa.size() << std::endl;
        if ( !fifoa.empty() ) connectedComponent.insert( connectedComponent.end(), fifoa.begin(), fifoa.end() );
      }

      PCCBox3D boundingBox;
      for ( size_t k = 0; k < 3; ++k ) {
        boundingBox.min_[k] = ( std::numeric_limits<double>::max )();
        boundingBox.max_[k] = 0;
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

      if ( enablePointCloudPartitioning ) {
        for ( int roiIndex = 0; roiIndex < numROIs; ++roiIndex ) {
          PCCBox3D roiBB;
          roiBB.min_[0] = roiBoundingBoxMinX[roiIndex];
          roiBB.max_[0] = roiBoundingBoxMaxX[roiIndex];
          roiBB.min_[1] = roiBoundingBoxMinY[roiIndex];
          roiBB.max_[1] = roiBoundingBoxMaxY[roiIndex];
          roiBB.min_[2] = roiBoundingBoxMinZ[roiIndex];
          roiBB.max_[2] = roiBoundingBoxMaxZ[roiIndex];
          if ( roiBB.fullyContains( boundingBox ) ) { patch.getRoiIndex() = roiIndex; }
        }
      }
      // patch.getSizeU() = 1 + size_t(boundingBox.max_[patch.getTangentAxis()] -
      // boundingBox.min_[patch.getTangentAxis()]); patch.getSizeV() = 1 +
      // size_t(boundingBox.max_[patch.getBitangentAxis()] - boundingBox.min_[patch.getBitangentAxis()]);
      patch.getSizeU() = 1 + size_t( round( boundingBox.max_[patch.getTangentAxis()] ) -
                                     floor( boundingBox.min_[patch.getTangentAxis()] ) );
      patch.getSizeV() = 1 + size_t( round( boundingBox.max_[patch.getBitangentAxis()] ) -
                                     floor( boundingBox.min_[patch.getBitangentAxis()] ) );
      patch.getU1()    = size_t( boundingBox.min_[patch.getTangentAxis()] );
      patch.getV1()    = size_t( boundingBox.min_[patch.getBitangentAxis()] );

      if ( patch.getProjectionMode() == 0 ) {
        patch.getD1() = infiniteDepth;
      } else {
        patch.getD1() = 0;
      }

      patch.getDepth( 0 ).resize( patch.getSizeU() * patch.getSizeV(), infiniteDepth );
#ifdef DEBUG_SURFACE_SEPARATION
      if ( useSurfaceSeparation ) 
#endif
      {
      patch.getdepth0pccidx().resize( patch.getSizeU() * patch.getSizeV(), infinitenumber );
      }

      if ( useEnhancedDeltaDepthCode ) {
        patch.getDepthEnhancedDeltaD().resize( patch.getSizeU() * patch.getSizeV(), 0 );
      }

      patch.getOccupancyResolution() = occupancyResolution;
      patch.getSizeU0()              = 0;
      patch.getSizeV0()              = 0;

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
            patch.getDepth( 0 )[p]     = d;
#ifdef DEBUG_SURFACE_SEPARATION
            if ( useSurfaceSeparation ) { patch.getdepth0pccidx()[p] = i; }
#else
            patch.getdepth0pccidx()[p] = i;
#endif
            patch.getSizeU0()          = ( std::max )( patch.getSizeU0(), u / patch.getOccupancyResolution() );
            patch.getSizeV0()          = ( std::max )( patch.getSizeV0(), v / patch.getOccupancyResolution() );
            patch.getD1()              = ( std::min )( patch.getD1(), size_t( d ) );
            size_t value               = patch.getD1();  // patch.getD1();
            value                      = ( value / minLevel );
            patch.getD1()              = value * minLevel;
          }
        } else {  // max
          if ( patch.getDepth( 0 )[p] == infiniteDepth ) {
            patch.getDepth( 0 )[p]     = d;
#ifdef DEBUG_SURFACE_SEPARATION
            if ( useSurfaceSeparation ) { patch.getdepth0pccidx()[p] = i;}
#else
            patch.getdepth0pccidx()[p] = i;
#endif
            patch.getSizeU0()          = ( std::max )( patch.getSizeU0(), u / patch.getOccupancyResolution() );
            patch.getSizeV0()          = ( std::max )( patch.getSizeV0(), v / patch.getOccupancyResolution() );
            patch.getD1()              = ( std::max )( patch.getD1(), size_t( d ) );
            size_t value               = size_t( patch.getD1() / minLevel );
            if ( value * minLevel < patch.getD1() )
              patch.getD1() = ( 1 + value ) * minLevel;
            else
              patch.getD1() = value * minLevel;
            // std::cout<<"~~~~~~"<<value<<" -> "<<patch.getD1()<<std::endl;
          } else {
            if ( patch.getDepth( 0 )[p] < d ) {
              patch.getDepth( 0 )[p]     = d;
#ifdef DEBUG_SURFACE_SEPARATION
              if ( useSurfaceSeparation ) { patch.getdepth0pccidx()[p] = i; }
#else
              patch.getdepth0pccidx()[p] = i;
#endif
              patch.getSizeU0()          = ( std::max )( patch.getSizeU0(), u / patch.getOccupancyResolution() );
              patch.getSizeV0()          = ( std::max )( patch.getSizeV0(), v / patch.getOccupancyResolution() );
              patch.getD1()              = ( std::max )( patch.getD1(), size_t( d ) );
              size_t value               = size_t( patch.getD1() / minLevel );
              if ( value * minLevel < patch.getD1() )
                patch.getD1() = ( 1 + value ) * minLevel;
              else
                patch.getD1() = value * minLevel;
              // std::cout<<"*******"<<value<<" -> "<<patch.getD1()<<std::endl;
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
              patch.getDepth( 0 )[p]     = infiniteDepth;
#ifdef DEBUG_SURFACE_SEPARATION
              if ( useSurfaceSeparation ) { patch.getdepth0pccidx()[p] = infinitenumber; }
#else
              patch.getdepth0pccidx()[p] = infinitenumber;
#endif
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

            if ( maxDepth < 0 ) {
              patch.getDepth( 0 )[p]     = infiniteDepth;
#ifdef DEBUG_SURFACE_SEPARATION
              if ( useSurfaceSeparation ) { patch.getdepth0pccidx()[p] = infinitenumber; }
#else
              patch.getdepth0pccidx()[p] = infinitenumber;
#endif
            } else {
              if ( depth0 != infiniteDepth ) {
                int16_t tmp_a = maxDepth - depth0;
                int16_t tmp_b = depth0 - int16_t( surfaceThickness );
                int16_t tmp_c = int16_t( patch.getD1() ) - int16_t( maxAllowedDepth );
                if ( ( tmp_a > 32 ) || ( tmp_b < tmp_c ) ) {
                  patch.getDepth( 0 )[p]     = infiniteDepth;
#ifdef DEBUG_SURFACE_SEPARATION
                  if ( useSurfaceSeparation ) patch.getdepth0pccidx()[p] = infinitenumber;
#else
                  patch.getdepth0pccidx()[p] = infinitenumber;
#endif
                }
              }
            }
          }
        }
      }

      if ( EOMSingleLayerMode ) {
        // compute edd
        if ( patch.getProjectionMode() == 0 ) {  // min
          if ( !useSurfaceSeparation ) {
            for ( const auto i : connectedComponent ) {
              const auto&   point = points[i];
              const int16_t d     = int16_t( round( point[patch.getNormalAxis()] ) );
              const size_t  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
              const size_t  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
              assert( u >= 0 && u < patch.getSizeU() );
              assert( v >= 0 && v < patch.getSizeV() );
              const size_t  p      = v * patch.getSizeU() + u;
              const int16_t depth0 = patch.getDepth( 0 )[p];

              if ( useEnhancedDeltaDepthCode && depth0 < infiniteDepth && ( d - depth0 ) > 0 &&
                   ( d - depth0 ) <= int16_t( EOMFixBitCount ) ) {
                const uint16_t oldEDDCode = patch.getDepthEnhancedDeltaD()[p];
                uint16_t       deltaD     = d - depth0;
                int            comp_depth0;

                comp_depth0 = depth0 - patch.getD1();

                patch.getDepthEnhancedDeltaD()[p] |= 1 << ( deltaD - 1 );
                if ( ( comp_depth0 + patch.getDepthEnhancedDeltaD()[p] ) > 1023 ) {
                  patch.getDepthEnhancedDeltaD()[p] = oldEDDCode;
                  std::cout << "(D0 + EDD-Code) > 1023. Data overflow observed (assume using 10bit coding). "
                               "Temporary solution: the corresponding inbetween or Depth1 point will be regarded as "
                               "missing point. To be improved if this happens a lot...\n";
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
                const auto&   point = points[i];
                const int16_t d     = int16_t( round( point[patch.getNormalAxis()] ) );
                const size_t  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
                const size_t  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
                assert( u >= 0 && u < patch.getSizeU() );
                assert( v >= 0 && v < patch.getSizeV() );
                const size_t  p      = v * patch.getSizeU() + u;
                const int16_t depth0 = patch.getDepth( 0 )[p];
                tot_num++;

                if ( err_flag ) {
                  if ( useEnhancedDeltaDepthCode && depth0 < infiniteDepth && ( d - depth0 ) > 0 &&
                       ( d - depth0 ) <= int16_t( patch_surfaceThickness + EOMFixBitCount - surfaceThickness ) ) {
                    const uint16_t oldEDDCode = patch.getDepthEnhancedDeltaD()[p];
                    uint16_t       deltaD     = d - depth0;
                    int            comp_depth0;

                    comp_depth0 = depth0 - patch.getD1();

                    patch.getDepthEnhancedDeltaD()[p] |= 1 << ( deltaD - 1 );
                    if ( ( comp_depth0 + patch.getDepthEnhancedDeltaD()[p] ) > 1023 ) {
                      patch.getDepthEnhancedDeltaD()[p] = oldEDDCode;
                      std::cout << "(D0 + EDD-Code) > 1023. Data overflow observed (assume using 10bit coding). "
                                   "Temporary solution: the corresponding inbetween or Depth1 point will be regarded "
                                   "as missing point. To be improved if this happens a lot...\n";
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
        } else {  // else ( patch.getProjectionMode() == 0 )
          if ( !useSurfaceSeparation ) {
            for ( const auto i : connectedComponent ) {
              const auto&   point = points[i];
              const int16_t d     = int16_t( round( point[patch.getNormalAxis()] ) );
              const size_t  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
              const size_t  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
              assert( u >= 0 && u < patch.getSizeU() );
              assert( v >= 0 && v < patch.getSizeV() );
              const size_t  p      = v * patch.getSizeU() + u;
              const int16_t depth0 = patch.getDepth( 0 )[p];
              if ( useEnhancedDeltaDepthCode && depth0 < infiniteDepth && ( depth0 - d ) > 0 &&
                   ( depth0 - d ) <= int16_t( EOMFixBitCount ) ) {
                const uint16_t oldEDDCode = patch.getDepthEnhancedDeltaD()[p];
                uint16_t       deltaD     = depth0 - d;
                int            comp_depth0;

                comp_depth0 = patch.getD1() - depth0;

                patch.getDepthEnhancedDeltaD()[p] |= 1 << ( deltaD - 1 );
                if ( ( comp_depth0 + patch.getDepthEnhancedDeltaD()[p] ) > 1023 ) {
                  patch.getDepthEnhancedDeltaD()[p] = oldEDDCode;
                  std::cout << "(D0 + EDD-Code) > 1023. Data overflow observed (assume using 10bit coding). "
                               "Temporary solution: the corresponding inbetween or Depth1 point will be regarded as "
                               "missing point. To be improved if this happens a lot...\n";
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
                const auto&   point = points[i];
                const int16_t d     = int16_t( round( point[patch.getNormalAxis()] ) );
                const size_t  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
                const size_t  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
                assert( u >= 0 && u < patch.getSizeU() );
                assert( v >= 0 && v < patch.getSizeV() );
                const size_t  p      = v * patch.getSizeU() + u;
                const int16_t depth0 = patch.getDepth( 0 )[p];
                tot_num++;

                if ( err_flag ) {
                  if ( useEnhancedDeltaDepthCode && depth0 < infiniteDepth && ( depth0 - d ) > 0 &&
                       ( depth0 - d ) <= int16_t( EOMFixBitCount ) ) {
                    const uint16_t oldEDDCode = patch.getDepthEnhancedDeltaD()[p];
                    uint16_t       deltaD     = depth0 - d;
                    int            comp_depth0;

                    comp_depth0 = patch.getD1() - depth0;

                    patch.getDepthEnhancedDeltaD()[p] |= 1 << ( deltaD - 1 );
                    if ( ( comp_depth0 + patch.getDepthEnhancedDeltaD()[p] ) > 1023 ) {
                      patch.getDepthEnhancedDeltaD()[p] = oldEDDCode;
                      std::cout << "(D0 + EDD-Code) > 1023. Data overflow observed (assume using 10bit coding). "
                                   "Temporary solution: the corresponding inbetween or Depth1 point will be regarded "
                                   "as missing point. To be improved if this happens a lot...\n";
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
        }       // fi ( patch.getProjectionMode() == 0 )
      } else {  // EOMSingleLayerMode
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
#ifndef DEBUG_MULTI_STREAMS
              bool       validD1          = false;
              bool       bColorDifference = true;
              PCCColor3B colorD1candidate;
              PCCColor3B colorD0;
              if ( depth0 < infiniteDepth ) {
                const size_t d0_idx = patch.getdepth0pccidx()[p];
                const size_t d1_idx = i;
                colorD1candidate    = frame_pcc_color[d1_idx];
                colorD0             = frame_pcc_color[d0_idx];
                bColorDifference    = ( std::abs( colorD0[0] - colorD1candidate[0] ) < 128 ) &&
                                   ( std::abs( colorD0[1] - colorD1candidate[1] ) < 128 ) &&
                                   ( std::abs( colorD0[2] - colorD1candidate[2] ) < 128 );
                validD1 = depth0 < infiniteDepth && ( d - depth0 ) <= int16_t( surfaceThickness ) &&
                          d > patch.getDepth( 1 )[p] && bColorDifference;
              }
#endif
              if ( depth0 < infiniteDepth && ( d - depth0 ) <= int16_t( surfaceThickness ) &&
                   d > patch.getDepth( 1 )[p] ) {
#ifndef DEBUG_MULTI_STREAMS
#if MULTISTREAM_UPDATE
                if ( bColorDifference )
#endif
#endif
                  patch.getDepth( 1 )[p] = d;
              }
              if ( useEnhancedDeltaDepthCode && depth0 < infiniteDepth && ( d - depth0 ) > 0 &&
                   ( d - depth0 ) <= int16_t( surfaceThickness )
#ifndef DEBUG_MULTI_STREAMS
#if MULTISTREAM_UPDATE
                   && d < patch.getDepth( 1 )[p]  // jkei:correct?
#endif
#endif
              ) {
                const uint16_t oldEDDCode = patch.getDepthEnhancedDeltaD()[p];
                const uint16_t deltaD     = d - depth0;
                int            comp_depth0;

                comp_depth0 = depth0 - patch.getD1();

                patch.getDepthEnhancedDeltaD()[p] |= 1 << ( deltaD - 1 );
                if ( ( comp_depth0 + patch.getDepthEnhancedDeltaD()[p] ) > maxD ) {
                  patch.getDepthEnhancedDeltaD()[p] = oldEDDCode;
                  std::cout
                      << "(D0 + EDD-Code) > maxD. Data overflow observed (assume using 10bit coding). Temporary "
                         "solution: the corresponding inbetween or Depth1 point will be regarded as missing point. "
                         "To be improved if this happens a lot...\n";
                }
              }
              if ( patch.getDepth( 1 )[p] < patch.getDepth( 0 )[p] ) {
                std::cout << "1.compute d1 map : ERROR : proj0 and d1 < d0" << std::endl;
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

#ifndef DEBUG_MULTI_STREAMS
                bool       validD1          = false;
                bool       bColorDifference = true;
                PCCColor3B colorD1candidate;
                PCCColor3B colorD0;
                if ( depth0 < infiniteDepth ) {
                  const size_t d0_idx = patch.getdepth0pccidx()[p];
                  const size_t d1_idx = i;
                  colorD1candidate    = frame_pcc_color[d1_idx];
                  colorD0             = frame_pcc_color[d0_idx];
                  bColorDifference    = ( std::abs( colorD0[0] - colorD1candidate[0] ) < 128 ) &&
                                     ( std::abs( colorD0[1] - colorD1candidate[1] ) < 128 ) &&
                                     ( std::abs( colorD0[2] - colorD1candidate[2] ) < 128 );
                  validD1 = depth0 < infiniteDepth && ( d - depth0 ) <= int16_t( surfaceThickness ) &&
                            d > patch.getDepth( 1 )[p];
#if MULTISTREAM_UPDATE
                  validD1 &= bColorDifference;
#endif
                }
#endif
                if ( err_flag == false ) {
#ifdef DEBUG_MULTI_STREAMS
                  if ( depth0 < infiniteDepth && ( d - depth0 ) <= int16_t( patch_surfaceThickness ) &&
                       d > patch.getDepth( 1 )[p] ) {
#else
                  if ( validD1 ) {
#endif
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
#ifdef DEBUG_MULTI_STREAMS
                  if ( depth0 < infiniteDepth && ( d - depth0 ) <= int16_t( patch_surfaceThickness ) &&
                       d > patch.getDepth( 1 )[p] ) 
#else
                  if ( validD1 ) 
#endif
                  {
                    patch.getDepth( 1 )[p] = d;
                  }
                  if ( useEnhancedDeltaDepthCode && depth0 < infiniteDepth && ( d - depth0 ) > 0 &&
                       ( d - depth0 ) <= int16_t( surfaceThickness )
#ifndef DEBUG_MULTI_STREAMS
#if MULTISTREAM_UPDATE
                       && d < patch.getDepth( 1 )[p]
#endif
#endif
                  ) {
                    const uint16_t oldEDDCode = patch.getDepthEnhancedDeltaD()[p];
                    const uint16_t deltaD     = d - depth0;
                    int            comp_depth0;

                    comp_depth0 = depth0 - patch.getD1();

                    patch.getDepthEnhancedDeltaD()[p] |= 1 << ( deltaD - 1 );
                    if ( ( comp_depth0 + patch.getDepthEnhancedDeltaD()[p] ) > 1023 ) {
                      patch.getDepthEnhancedDeltaD()[p] = oldEDDCode;
                      std::cout
                          << "(D0 + EDD-Code) > 1023. Data overflow observed (assume using 10bit coding). "
                             "Temporary solution: the corresponding inbetween or Depth1 point will be regarded as "
                             "missing point. To be improved if this happens a lot...\n";
                    }
                  }
                  if ( patch.getDepth( 1 )[p] < patch.getDepth( 0 )[p] ) {
                    std::cout << "2.compute d1 map : ERROR : proj0 and d1 < d0" << std::endl;
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
              const size_t  p                = v * patch.getSizeU() + u;
              const int16_t depth0           = patch.getDepth( 0 )[p];
              bool          validD1          = false;
              bool          bColorDifference = true;
              PCCColor3B    colorD1candidate;
              PCCColor3B    colorD0;
              if ( depth0 < infiniteDepth ) {
                const size_t d0_idx = patch.getdepth0pccidx()[p];
                const size_t d1_idx = i;
                colorD1candidate    = frame_pcc_color[d1_idx];
                colorD0             = frame_pcc_color[d0_idx];
                bColorDifference    = ( std::abs( colorD0[0] - colorD1candidate[0] ) < 128 ) &&
                                   ( std::abs( colorD0[1] - colorD1candidate[1] ) < 128 ) &&
                                   ( std::abs( colorD0[2] - colorD1candidate[2] ) < 128 );
                validD1 = depth0 < infiniteDepth && ( depth0 - d ) <= int16_t( surfaceThickness ) &&
                          d < patch.getDepth( 1 )[p] && bColorDifference;
              }
              if ( depth0 < infiniteDepth && ( depth0 - d ) <= int16_t( surfaceThickness ) &&
                   d < patch.getDepth( 1 )[p] ) {
#ifndef DEBUG_MULTI_STREAMS
#if MULTISTREAM_UPDATE
                if ( bColorDifference )
#endif
#endif
                  patch.getDepth( 1 )[p] = d;
              }
              if ( useEnhancedDeltaDepthCode && depth0 < infiniteDepth && ( depth0 - d ) > 0 &&
                   ( depth0 - d ) <= int16_t( surfaceThickness )
#ifndef DEBUG_MULTI_STREAMS
#if MULTISTREAM_UPDATE
                   && d > patch.getDepth( 1 )[p]
#endif
#endif
              ) {
                const uint16_t oldEDDCode = patch.getDepthEnhancedDeltaD()[p];
                const uint16_t deltaD     = depth0 - d;
                int            comp_depth0;

                comp_depth0 = patch.getD1() - depth0;

                patch.getDepthEnhancedDeltaD()[p] |= 1 << ( deltaD - 1 );
                if ( ( comp_depth0 + patch.getDepthEnhancedDeltaD()[p] ) > 1023 ) {
                  patch.getDepthEnhancedDeltaD()[p] = oldEDDCode;
                  std::cout
                      << "(D0 + EDD-Code) > 1023. Data overflow observed (assume using 10bit coding). Temporary "
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

#ifndef DEBUG_MULTI_STREAMS
                bool       validD1          = false;
                bool       bColorDifference = true;
                PCCColor3B colorD1candidate;
                PCCColor3B colorD0;
                if ( depth0 < infiniteDepth ) {
                  const size_t d0_idx = patch.getdepth0pccidx()[p];
                  const size_t d1_idx = i;
                  colorD1candidate    = frame_pcc_color[d1_idx];
                  colorD0             = frame_pcc_color[d0_idx];
                  bColorDifference    = ( std::abs( colorD0[0] - colorD1candidate[0] ) < 128 ) &&
                                     ( std::abs( colorD0[1] - colorD1candidate[1] ) < 128 ) &&
                                     ( std::abs( colorD0[2] - colorD1candidate[2] ) < 128 );
                  validD1 = depth0 < infiniteDepth && ( depth0 - d ) <= int16_t( patch_surfaceThickness ) &&
                            d < patch.getDepth( 1 )[p];
#if MULTISTREAM_UPDATE
                  validD1 &= bColorDifference;
#endif
                }
#endif
                if ( err_flag == false ) {
#ifdef DEBUG_MULTI_STREAMS
                  if ( depth0 < infiniteDepth && ( depth0 - d ) <= int16_t( patch_surfaceThickness ) &&
                       d < patch.getDepth( 1 )[p] ) {
#else
                  if ( validD1 ) {
#endif
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
                  if ( depth0 < infiniteDepth &&
                       ( depth0 - d ) <= int16_t( patch_surfaceThickness )  // fixed to patch_surfaceThickness

#ifndef DEBUG_MULTI_STREAMS
#if MULTISTREAM_UPDATE
                       && d < patch.getDepth( 1 )[p]
#endif
#endif
                  ) {
                    patch.getDepth( 1 )[p] = d;
                  }
                  if ( useEnhancedDeltaDepthCode && depth0 < infiniteDepth && ( depth0 - d ) > 0 &&
                       ( depth0 - d ) <= int16_t( surfaceThickness )

#ifndef DEBUG_MULTI_STREAMS
#if MULTISTREAM_UPDATE
                       && d > patch.getDepth( 1 )[p]
#endif
#endif
                  ) {
                    const uint16_t oldEDDCode = patch.getDepthEnhancedDeltaD()[p];
                    const uint16_t deltaD     = depth0 - d;
                    int            comp_depth0;

                    comp_depth0 = patch.getD1() - depth0;

                    patch.getDepthEnhancedDeltaD()[p] |= 1 << ( deltaD - 1 );
                    if ( ( comp_depth0 + patch.getDepthEnhancedDeltaD()[p] ) > 1023 ) {
                      patch.getDepthEnhancedDeltaD()[p] = oldEDDCode;
                      std::cout
                          << "(D0 + EDD-Code) > 1023. Data overflow observed (assume using 10bit coding). "
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
      }

      patch.getSizeD() = 0;

      PCCPointSet3 rec;
      rec.resize( 0 );
      if ( patch.getProjectionMode() == 0 ) {
        for ( size_t v = 0; v < patch.getSizeV(); ++v ) {
          for ( size_t u = 0; u < patch.getSizeU(); ++u ) {
            const size_t p      = v * patch.getSizeU() + u;
            int16_t      depth0 = patch.getDepth( 0 )[p];
            if ( depth0 < infiniteDepth ) {
              d0CountPerPatch++;
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
              if ( !EOMSingleLayerMode ) { patch.getDepth( 1 )[p] -= int16_t( patch.getD1() ); }

              if ( useEnhancedDeltaDepthCode ) {
                if ( patch.getDepthEnhancedDeltaD()[p] != 0 ) {
                  d1CountPerPatch++;
                  PCCVector3D pointEDD( point );
                  if ( EOMSingleLayerMode ) {
                    size_t  N       = EOMFixBitCount + 1;
                    int16_t new_EDD = 0;
                    for ( uint16_t i = 0; i < N - 1; i++ ) {
                      if ( patch.getDepthEnhancedDeltaD()[p] & ( 1 << i ) ) {
                        new_EDD |= ( 1 << i );
                        uint16_t nDeltaDCur = ( i + 1 );

                        pointEDD[patch.getNormalAxis()] = double( depth0 + patch.getD1() + nDeltaDCur );

                        if ( pointEDD[patch.getNormalAxis()] != point[patch.getNormalAxis()] ) {
                          resampled.addPoint( pointEDD );
                          resampledPatchPartition.push_back( patchIndex );
                          eddCountPerPatch++;
                        }
                      }
                    }  // for each i
                    patch.getDepthEnhancedDeltaD()[p] = new_EDD;
                  } else {
                    for ( uint16_t i = 0; i < surfaceThickness; i++ ) {
                      if ( patch.getDepthEnhancedDeltaD()[p] & ( 1 << i ) ) {
                        uint16_t nDeltaDCur = ( i + 1 );

                        point[patch.getNormalAxis()] = double( depth0 + patch.getD1() + nDeltaDCur );

                        resampled.addPoint( point );
                        resampledPatchPartition.push_back( patchIndex );
                        eddCountPerPatch++;
                      }
                    }  // for each i
                  }
                }       // if( patch.getDepthEnhancedDeltaD()[p] != 0) )
              } else {  // if(useEnhancedDeltaDepthCode)
                if ( patch.getDepth( 0 )[p] != patch.getDepth( 1 )[p] ) d1CountPerPatch++;
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

                int16_t depth1 = patch.getDepth( 1 )[p];
                patch.getSizeD() =
                    ( std::max )( patch.getSizeD(), static_cast<size_t>( depth1 ) );  // compute max depth

              }  // if(useEnhancedDeltaDepthCode)
            }
          }
        }
      } else {
        for ( size_t v = 0; v < patch.getSizeV(); ++v ) {
          for ( size_t u = 0; u < patch.getSizeU(); ++u ) {
            const size_t p        = v * patch.getSizeU() + u;
            int16_t      depth0_1 = patch.getDepth( 0 )[p];
            if ( depth0_1 < infiniteDepth ) {
              d0CountPerPatch++;
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
              int16_t depth1_1 = 0;
              if ( !EOMSingleLayerMode ) {
                depth1_1               = patch.getDepth( 1 )[p];
                patch.getDepth( 1 )[p] = int16_t( patch.getD1() ) - depth1_1;
              }

              if ( useEnhancedDeltaDepthCode ) {
                if ( patch.getDepthEnhancedDeltaD()[p] != 0 ) {
                  d1CountPerPatch++;
                  PCCVector3D pointEDD( point );
                  if ( EOMSingleLayerMode ) {
                    size_t  N       = EOMFixBitCount + 1;
                    int16_t new_EDD = 0;
                    for ( uint16_t i = 0; i < N - 1; i++ ) {
                      if ( patch.getDepthEnhancedDeltaD()[p] & ( 1 << i ) ) {
                        new_EDD |= ( 1 << i );
                        uint16_t nDeltaDCur             = ( i + 1 );
                        pointEDD[patch.getNormalAxis()] = double( patch.getD1() - ( depth0 + nDeltaDCur ) );
                        if ( pointEDD[patch.getNormalAxis()] != point[patch.getNormalAxis()] ) {
                          resampled.addPoint( pointEDD );
                          resampledPatchPartition.push_back( patchIndex );
                          eddCountPerPatch++;
                        }
                      }
                    }  // for each i
                    patch.getDepthEnhancedDeltaD()[p] = new_EDD;
                  } else {
                    for ( uint16_t i = 0; i < surfaceThickness; i++ ) {
                      if ( patch.getDepthEnhancedDeltaD()[p] & ( 1 << i ) ) {
                        uint16_t nDeltaDCur          = ( i + 1 );
                        point[patch.getNormalAxis()] = double( patch.getD1() - depth0 - nDeltaDCur );
                        resampled.addPoint( point );
                        resampledPatchPartition.push_back( patchIndex );
                        eddCountPerPatch++;
                      }
                    }   // for each i
                  }     // if( patch.getDepthEnhancedDeltaD()[p] != 0) )
                }       // if( patch.getDepthEnhancedDeltaD()[p] != 0) )
              } else {  // if(useEnhancedDeltaDepthCode)
                point[patch.getNormalAxis()] = double( depth1_1 );
                if ( patch.getDepth( 0 )[p] != patch.getDepth( 1 )[p] ) d1CountPerPatch++;
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

      if ( testLevelOfDetail > 0 ){
        quantizedPointsPatchModification(
            points,
            patch,  // current patch
            bIsAdditionalProjectionPlane, Threshold_Color_Error, frame_pcc_color, connectedComponent,
            minPointCountPerCC, occupancyResolution, maxAllowedDist2MissedPointsDetection,
            maxAllowedDist2MissedPointsSelection, EOMSingleLayerMode, EOMFixBitCount, surfaceThickness, maxAllowedDepth,
            minLevel, partition, patches, patchPartition, resampledPatchPartition, missedPoints, resampled,
            useEnhancedDeltaDepthCode, createSubPointCloud, subPointCloud, distanceSrcRec, absoluteD1,
            useSurfaceSeparation, additionalProjectionAxis, geometryBitDepth3D, testLevelOfDetail,
            patchExpansionEnabled, enablePointCloudPartitioning, roiBoundingBoxMinX, roiBoundingBoxMaxX,
            roiBoundingBoxMinY, roiBoundingBoxMaxY, roiBoundingBoxMinZ, roiBoundingBoxMaxZ, numCutsAlong1stLongestAxis,
            numCutsAlong2ndLongestAxis, numCutsAlong3rdLongestAxis );
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
        testSrc.transferColorSimple( testRec );
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
      patch.setEddandD1Count( eddCountPerPatch );
      if ( useEnhancedDeltaDepthCode ) { 
        patch.setEddCount( eddCountPerPatch - d1CountPerPatch );
        printf( "Patch %lu : EDD = %lu - %lu  = %lu \n", patchIndex, eddCountPerPatch, d1CountPerPatch,
                eddCountPerPatch - d1CountPerPatch ); 
      }
      patch.setD0Count( d0CountPerPatch );
#if 1
      numberOfEDD += eddCountPerPatch; // JR 
#else
      numberOfEDD += ( eddCountPerPatch - d1CountPerPatch );
#endif

      numD0Points += d0CountPerPatch;
      numD1Points += d1CountPerPatch;
      if ( useEnhancedDeltaDepthCode ) { 
#if 1
        numEDDonlyPoints += eddCountPerPatch;   // JR 
#else
        numEDDonlyPoints += ( eddCountPerPatch - d1CountPerPatch );  
#endif
      }
      // assert(eddPointSet.getPointCount() !=patch.getEddCount());
      std::cout << "\t\t Patch " << patchIndex << " ->(d1,u1,v1)=( " << patch.getD1() << " , " << patch.getU1() << " , "
                << patch.getV1() << " )(dd,du,dv)=( " << patch.getSizeD() << " , " << patch.getSizeU() << " , "
                << patch.getSizeV() << " ),Normal: " << size_t( patch.getNormalAxis() )
                << " Direction: " << patch.getProjectionMode() << " Edd: " << patch.getEddCount() << std::endl;
    }
    PCCKdTree kdtreeResampled( resampled );
    missedPoints.resize( 0 );
    for ( size_t i = 0; i < pointCount; ++i ) {
      kdtreeResampled.search( points[i], 1, result );
      const double dist2      = result.dist( 0 );
      missedPointsDistance[i] = dist2;
      if ( dist2 > maxAllowedDist2MissedPointsSelection ) { missedPoints.push_back( i ); }
    }
    if ( enablePointCloudPartitioning ) {
      // update missedPointsChunks using missedPoints
      std::set<size_t>                 missedPointsSet( missedPoints.begin(), missedPoints.end() );
      std::vector<std::vector<size_t>> missedPointsChunksNextIter( numChunks );
      std::vector<std::vector<size_t>> pointsIndexChunksNextIter( numChunks );

      for ( size_t chunkIndex = 0; chunkIndex < numChunks; ++chunkIndex ) {
        for ( size_t i = 0; i < pointsIndexChunks[chunkIndex].size(); ++i ) {
          if ( missedPointsSet.find( pointsIndexChunks[chunkIndex][i] ) != missedPointsSet.end() ) {
            missedPointsChunksNextIter[chunkIndex].push_back( i );
            pointsIndexChunksNextIter[chunkIndex].push_back( pointsIndexChunks[chunkIndex][i] );
            missedPointsDistanceChunks[chunkIndex][i] = missedPointsDistance[pointsIndexChunks[chunkIndex][i]];
          }
        }
        missedPointsChunks[chunkIndex].resize( 0 );
        missedPointsChunks[chunkIndex] = missedPointsChunksNextIter[chunkIndex];
        pointsIndexChunks[chunkIndex].resize( 0 );
        pointsIndexChunks[chunkIndex] = pointsIndexChunksNextIter[chunkIndex];
      }
    }
    std::cout << " # patches " << patches.size() << std::endl;
    std::cout << " # resampled " << resampled.getPointCount() << std::endl;
    std::cout << " # missed points " << missedPoints.size() << std::endl;
    if ( useEnhancedDeltaDepthCode ) { std::cout << " # EDD points " << numberOfEDD << std::endl; }
  }
  distanceSrcRec = meanYAB + meanUAB + meanVAB + meanYBA + meanUBA + meanVBA;
#if 0
      std::cout<<"frame"<<frameIndex <<" D0: "<<numD0Points<<" D1: "<<numD1Points<<" EDD: "<<numEDDonlyPoints<<std::endl;
#endif
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

void PCCPatchSegmenter3::refineSegmentationGridBased( const PCCPointSet3&         pointCloud,
                                                      const PCCNormalsGenerator3& normalsGen,
                                                      const PCCVector3D*          orientations,
                                                      const size_t                orientationCount,
                                                      const size_t                maxNNCount,
                                                      const double                lambda,
                                                      const size_t                iterationCount,
                                                      const size_t                voxDim,
                                                      const size_t                searchRadius,
                                                      std::vector<size_t>&        partition ) {
  const size_t pointCount = pointCloud.getPointCount();
  auto         geoMax     = pointCloud[0][0];
  for ( size_t i = 0; i < pointCount; i++ ) {
    const auto& pos = pointCloud[i];
    geoMax          = ( std::max )( geoMax, pos[0] );
    geoMax          = ( std::max )( geoMax, pos[1] );
    geoMax          = ( std::max )( geoMax, pos[2] );
  }
  size_t geoRange = 1;
  for ( size_t i = geoMax - 1; i; i >>= 1, geoRange <<= 1 )
    ;
  size_t voxDimShift, gridDimShift, i;
  for ( voxDimShift = 0, i = voxDim; i > 1; voxDimShift++, i >>= 1 )
    ;
  const size_t gridDim = geoRange >> voxDimShift;
  for ( gridDimShift = 0, i = gridDim; i > 1; gridDimShift++, i >>= 1 )
    ;
  const size_t gridDimShiftSqr = gridDimShift << 1;
  const size_t voxDimHalf      = voxDim >> 1;

  struct voxelType {
    std::vector<size_t> pointIndices;
    std::vector<size_t> scoreSmooth;

    size_t pointCount;
    void   init( size_t cap, size_t orientationCount ) {
      pointIndices.reserve( cap );
      scoreSmooth.resize( orientationCount, 0 );
      pointCount = 0;
    }
  };
  auto subToInd = [&]( size_t x, size_t y, size_t z ) { return x + ( y << gridDimShift ) + ( z << gridDimShiftSqr ); };

  PCCPointSet3                          gridCenters;
  std::unordered_map<size_t, voxelType> grid;
  for ( size_t i = 0; i < pointCount; i++ ) {
    const auto&  pos = pointCloud[i];
    const size_t x0  = ( ( (size_t)pos[0] + voxDimHalf ) >> voxDimShift );
    const size_t y0  = ( ( (size_t)pos[1] + voxDimHalf ) >> voxDimShift );
    const size_t z0  = ( ( (size_t)pos[2] + voxDimHalf ) >> voxDimShift );
    size_t       p   = subToInd( x0, y0, z0 );
    if ( !grid.count( p ) ) {
      grid[p].init( 64, orientationCount );
      gridCenters.addPoint( PCCVector3D( x0, y0, z0 ) );
    }
    auto& voxel = grid[p];
    voxel.pointIndices.push_back( i );
    voxel.pointCount++;
  }

  PCCKdTree                        kdtree( gridCenters );
  const size_t                     voxSearchRadius  = searchRadius >> voxDimShift;
  const size_t                     maxNeighborCount = ( std::numeric_limits<int16_t>::max )();
  std::vector<std::vector<size_t>> adj( gridCenters.getPointCount() );
  computeAdjacencyInfoInRadius( gridCenters, kdtree, adj, maxNeighborCount, voxSearchRadius );

  std::vector<size_t> tmpPartition( pointCount );
  for ( size_t n = 0; n < iterationCount; n++ ) {
    for ( auto& gridElm : grid ) {
      auto& voxel = gridElm.second;
      std::fill( voxel.scoreSmooth.begin(), voxel.scoreSmooth.end(), 0 );
      for ( auto& j : voxel.pointIndices ) { voxel.scoreSmooth[partition[j]]++; }
    }
    /*tbb::task_arena limited((int)nbThread_);
    limited.execute([&] {
      tbb::parallel_for(size_t(0), gridCenters.getPointCount(), [&](const size_t i) {*/
    for ( size_t i = 0; i < gridCenters.getPointCount(); i++ ) {
      auto&               pos = gridCenters[i];
      size_t              p   = subToInd( pos[0], pos[1], pos[2] );
      std::vector<size_t> scoreSmooth( orientationCount, 0 );
      size_t              nnPointCount = 0;
      for ( auto& j : adj[i] ) {
        auto&  pos = gridCenters[j];
        size_t q   = subToInd( pos[0], pos[1], pos[2] );
        /*std::transform(scoreSmooth.begin(), scoreSmooth.end(), grid[q].scoreSmooth.begin(),
          scoreSmooth.begin(), std::plus<size_t>());*/
        for ( size_t k = 0; k < orientationCount; k++ ) { scoreSmooth[k] += grid[q].scoreSmooth[k]; }
        nnPointCount += grid[q].pointCount;
        if ( nnPointCount >= maxNNCount ) break;
      }
      const double weight = lambda / nnPointCount;
      for ( auto& j : grid[p].pointIndices ) {
        const PCCVector3D normal       = normalsGen.getNormal( j );
        size_t            clusterIndex = partition[j];
        double            bestScore    = 0.0;
        for ( size_t k = 0; k < orientationCount; k++ ) {
          const double scoreNormal = normal * orientations[k];
          const double score       = scoreNormal + weight * scoreSmooth[k];
          if ( score > bestScore ) {
            bestScore    = score;
            clusterIndex = k;
          }
        }
        tmpPartition[j] = clusterIndex;
      }
    } /*);
   });*/
    swap( tmpPartition, partition );
  }
}

float pcc::computeIOU( Rect a, Rect b ) {
  float iou              = 0.0f;
  Rect  intersec         = a & b;
  int   intersectionArea = intersec.area();
  int   unionArea        = a.area() + b.area() - intersectionArea;
  iou                    = (float)intersectionArea / unionArea;
  return iou;
}

void PCCPatchSegmenter3::separateHighGradientPoints( const PCCPointSet3&               points,
                                                     const size_t                      additionalProjectionAxis,
                                                     const bool                        absoluteD1,
                                                     const PCCNormalsGenerator3&       normalsGen,
                                                     const PCCVector3D*                orientations,
                                                     const size_t                      orientationCount,
                                                     const size_t                      surfaceThickness,
                                                     const size_t                      geometryBitDepth3D,
                                                     const double                      minGradient,
                                                     const size_t                      minNumHighGradientPoints,
                                                     std::vector<size_t>&              partition,
                                                     std::vector<std::vector<size_t>>& adj,
                                                     std::vector<std::vector<size_t>>& connectedComponents ) {
  // detect and remove high gradient points
  std::vector<std::vector<size_t>> highGradientConnectedComponents;
  size_t                           idx = 0;
  for ( auto& connectedComponent : connectedComponents ) {
    PCCPatch patch;
    patch.getIndex() = idx++;
    std::vector<bool> isComponentRemoved;
    isComponentRemoved.resize( connectedComponent.size(), false );
    bool bIsAdditionalProjectionPlane;
    determinePatchOrientation( additionalProjectionAxis, absoluteD1, bIsAdditionalProjectionPlane, patch, partition,
                               connectedComponent );
    generatePatchD0( points, geometryBitDepth3D, bIsAdditionalProjectionPlane, patch, connectedComponent );
    calculateGradient( points, connectedComponent, normalsGen, orientations, orientationCount,
                       partition[connectedComponent[0]], surfaceThickness, geometryBitDepth3D,
                       bIsAdditionalProjectionPlane, minGradient, minNumHighGradientPoints, patch, adj,
                       highGradientConnectedComponents, isComponentRemoved );

    // remove high gradient components from CC
    std::vector<size_t> tmpConnectedComponent;
    for ( size_t i = 0; i < connectedComponent.size(); ++i ) {
      if ( !isComponentRemoved[i] ) tmpConnectedComponent.push_back( connectedComponent[i] );
    }
    swap( connectedComponent, tmpConnectedComponent );
  }

  // try to join other CC
  std::vector<size_t> patchPartition;
  patchPartition.resize( points.getPointCount(), 0 );
  for ( size_t i = 0; i < connectedComponents.size(); ++i ) {
    auto& connectedComponent = connectedComponents[i];
    for ( const auto j : connectedComponent ) patchPartition[j] = i + 1;
  }

  std::vector<size_t> highGradientCCPartition;
  highGradientCCPartition.resize( highGradientConnectedComponents.size(), 0 );
  std::vector<size_t> highGradientPartition;
  highGradientPartition.resize( highGradientConnectedComponents.size(), 0 );
  for ( size_t i = 0; i < highGradientConnectedComponents.size(); ++i ) {
    auto&       highGradientConnectedComponent = highGradientConnectedComponents[i];
    PCCVector3D normalSum( 0.0f );
    for ( const auto j : highGradientConnectedComponent ) normalSum += normalsGen.getNormal( j );

    double       bestScore    = 0.0;
    const size_t orgPartition = partition[highGradientConnectedComponent[0]];
    for ( size_t k = 0; k < orientationCount; ++k ) {
      if ( orgPartition < 6 ) {
        if ( k < 6 && ( k % 3 ) == ( orgPartition % 3 ) ) continue;
      } else {
        if ( orgPartition < 10 ) {
          if ( k >= 6 && k < 10 && ( k == orgPartition || k + 2 == orgPartition || k == orgPartition + 2 ) ) continue;
        } else if ( orgPartition < 14 ) {
          if ( k >= 10 && k < 14 && ( k == orgPartition || k + 2 == orgPartition || k == orgPartition + 2 ) ) continue;
        } else {
          if ( k >= 14 && ( k == orgPartition || k + 2 == orgPartition || k == orgPartition + 2 ) ) continue;
        }
      }
      const double score = normalSum * orientations[k];
      if ( score > bestScore ) {
        bestScore                = score;
        highGradientPartition[i] = k;
      }
    }

    for ( const auto j : highGradientConnectedComponent ) {
      for ( const auto n : adj[j] ) {
        if ( patchPartition[n] != 0 ) {
          if ( ( partition[n] < 6 && highGradientPartition[i] < 6 &&
                 ( highGradientPartition[i] % 3 ) == ( partition[n] % 3 ) ) ||
               ( partition[n] >= 6 && partition[n] < 10 && highGradientPartition[i] >= 6 &&
                 highGradientPartition[i] < 10 && ( highGradientPartition[i] % 2 ) == ( partition[n] % 2 ) ) ||
               ( partition[n] >= 10 && partition[n] < 14 && highGradientPartition[i] >= 10 &&
                 highGradientPartition[i] < 14 && ( highGradientPartition[i] % 2 ) == ( partition[n] % 2 ) ) ||
               ( partition[n] >= 14 && highGradientPartition[i] >= 14 &&
                 ( highGradientPartition[i] % 2 ) == ( partition[n] % 2 ) ) ) {
            highGradientCCPartition[i] = patchPartition[n];
            break;
          }
        }
      }
      if ( highGradientCCPartition[i] != 0 ) break;
    }
  }

  for ( size_t i = 0; i < highGradientConnectedComponents.size(); ++i ) {
    auto& highGradientConnectedComponent = highGradientConnectedComponents[i];
    if ( highGradientCCPartition[i] != 0 ) {
      const size_t newCCIdx = highGradientCCPartition[i] - 1;
      connectedComponents[newCCIdx].insert( connectedComponents[newCCIdx].end(), highGradientConnectedComponent.begin(),
                                            highGradientConnectedComponent.end() );
      const size_t newPartition = partition[connectedComponents[newCCIdx][0]];
      for ( const auto j : highGradientConnectedComponent ) { partition[j] = newPartition; }
    } else {
      for ( const auto j : highGradientConnectedComponent ) { partition[j] = highGradientPartition[i]; }
      connectedComponents.push_back( highGradientConnectedComponent );
    }
  }
}

void PCCPatchSegmenter3::determinePatchOrientation( const size_t         additionalProjectionAxis,
                                                    const bool           absoluteD1,
                                                    bool&                bIsAdditionalProjectionPlane,
                                                    PCCPatch&            patch,
                                                    std::vector<size_t>& partition,
                                                    std::vector<size_t>& connectedComponent ) {
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

  patch.setViewId() = clusterIndex;

  if ( clusterIndex <= 2 ) {
    patch.getProjectionMode() = 0;
  } else {
    patch.getProjectionMode() = 1;
  }

  if ( absoluteD1 ) {
    // for additional projection plane
    if ( clusterIndex == 6 || clusterIndex == 7 ) { patch.getProjectionMode() = 0; }
    // for additional projection plane
    if ( clusterIndex == 10 || clusterIndex == 11 || clusterIndex == 14 || clusterIndex == 15 ) {
      patch.getProjectionMode() = 0;
    }
  }
}

void PCCPatchSegmenter3::generatePatchD0( const PCCPointSet3&  points,
                                          const size_t         geometryBitDepth3D,
                                          const bool           bIsAdditionalProjectionPlane,
                                          PCCPatch&            patch,
                                          std::vector<size_t>& connectedComponent ) {
  PCCBox3D boundingBox;
  for ( size_t k = 0; k < 3; ++k ) {
    boundingBox.min_[k] = ( std::numeric_limits<double>::max )();
    boundingBox.max_[k] = 0;
  }
  for ( const auto i : connectedComponent ) {
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

  const int16_t infiniteDepth = ( std::numeric_limits<int16_t>::max )();

  patch.getSizeU() = 1 + size_t( round( boundingBox.max_[patch.getTangentAxis()] ) -
                                 floor( boundingBox.min_[patch.getTangentAxis()] ) );
  patch.getSizeV() = 1 + size_t( round( boundingBox.max_[patch.getBitangentAxis()] ) -
                                 floor( boundingBox.min_[patch.getBitangentAxis()] ) );
  patch.getU1()    = size_t( boundingBox.min_[patch.getTangentAxis()] );
  patch.getV1()    = size_t( boundingBox.min_[patch.getBitangentAxis()] );

  patch.getDepth( 0 ).resize( patch.getSizeU() * patch.getSizeV(), infiniteDepth );

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
      if ( patch.getDepth( 0 )[p] > d ) patch.getDepth( 0 )[p] = d;
    } else {  // max
      if ( patch.getDepth( 0 )[p] == infiniteDepth ) {
        patch.getDepth( 0 )[p] = d;
      } else {
        if ( patch.getDepth( 0 )[p] < d ) patch.getDepth( 0 )[p] = d;
      }
    }
  }
}

void PCCPatchSegmenter3::calculateGradient( const PCCPointSet3&               points,
                                            const std::vector<size_t>&        connectedComponent,
                                            const PCCNormalsGenerator3&       normalsGen,
                                            const PCCVector3D*                orientations,
                                            const size_t                      orientationCount,
                                            const size_t                      orgPartitionIdx,
                                            const size_t                      surfaceThickness,
                                            const size_t                      geometryBitDepth3D,
                                            const bool                        bIsAdditionalProjectionPlane,
                                            const double                      minGradient,
                                            const size_t                      minNumHighGradientPoints,
                                            PCCPatch&                         patch,
                                            std::vector<std::vector<size_t>>& adj,
                                            std::vector<std::vector<size_t>>& highGradientConnectedComponents,
                                            std::vector<bool>&                isRemoved ) {
  /* for the case that the xyz components of a normal are the same:
       sqrt( nx^2 + ny^2 + nz^2 ) = 1, where nx = ny = nz = 0.577 */
  const double normalThreshold = 0.577;

  const int16_t infiniteDepth = ( std::numeric_limits<int16_t>::max )();

  std::vector<double> Gmag;
  Gmag.resize( patch.getSizeU() * patch.getSizeV(), 0 );
  int patchWidth  = int( patch.getSizeU() );
  int patchHeight = int( patch.getSizeV() );
  for ( int v = 0; v < patchHeight; ++v ) {
    for ( int u = 0; u < patchWidth; ++u ) {
      const size_t  p      = v * patchWidth + u;
      const int16_t depth0 = patch.getDepth( 0 )[p];
      if ( depth0 < infiniteDepth ) {
        int16_t Gx = 0, Gy = 0;

        int16_t depth[8];
        depth[0] = ( u != 0 && v != 0 && patch.getDepth( 0 )[( v - 1 ) * patchWidth + u - 1] < infiniteDepth )
                       ? patch.getDepth( 0 )[( v - 1 ) * patchWidth + u - 1]
                       : depth0;
        depth[1] = ( v != 0 && patch.getDepth( 0 )[( v - 1 ) * patchWidth + u] < infiniteDepth )
                       ? patch.getDepth( 0 )[( v - 1 ) * patchWidth + u]
                       : depth0;
        depth[2] =
            ( u != patchWidth - 1 && v != 0 && patch.getDepth( 0 )[( v - 1 ) * patchWidth + u + 1] < infiniteDepth )
                ? patch.getDepth( 0 )[( v - 1 ) * patchWidth + u + 1]
                : depth0;
        depth[3] = ( u != 0 && patch.getDepth( 0 )[v * patchWidth + u - 1] < infiniteDepth )
                       ? patch.getDepth( 0 )[v * patchWidth + u - 1]
                       : depth0;
        depth[4] = ( u != patchWidth - 1 && patch.getDepth( 0 )[v * patchWidth + u + 1] < infiniteDepth )
                       ? patch.getDepth( 0 )[v * patchWidth + u + 1]
                       : depth0;
        depth[5] =
            ( u != 0 && v != patchHeight - 1 && patch.getDepth( 0 )[( v + 1 ) * patchWidth + u - 1] < infiniteDepth )
                ? patch.getDepth( 0 )[( v + 1 ) * patchWidth + u - 1]
                : depth0;
        depth[6] = ( v != patchHeight - 1 && patch.getDepth( 0 )[( v + 1 ) * patchWidth + u] < infiniteDepth )
                       ? patch.getDepth( 0 )[( v + 1 ) * patchWidth + u]
                       : depth0;
        depth[7] = ( u != patchWidth - 1 && v != patchHeight - 1 &&
                     patch.getDepth( 0 )[( v + 1 ) * patchWidth + u + 1] < infiniteDepth )
                       ? patch.getDepth( 0 )[( v + 1 ) * patchWidth + u + 1]
                       : depth0;

        /* Sobel operator for x-axis
            1  0 -1
            2  0 -2
            1  0 -1   */
        Gx += depth[0];
        Gx += 2 * depth[3];
        Gx += depth[5];
        Gx -= depth[2];
        Gx -= 2 * depth[4];
        Gx -= depth[7];

        /* Sobel operator for y-axis
            1  2  1
            0  0  0
           -1 -2 -1   */
        Gy += depth[0];
        Gy += 2 * depth[1];
        Gy += depth[2];
        Gy -= depth[5];
        Gy -= 2 * depth[6];
        Gy -= depth[7];

        Gmag[p] = sqrt( double( Gx * Gx + Gy * Gy ) );
      }
    }
  }

  std::vector<bool> highGradientMap;
  highGradientMap.resize( patch.getSizeU() * patch.getSizeV(), false );
  for ( size_t v = 0; v < patch.getSizeV(); ++v ) {
    for ( size_t u = 0; u < patch.getSizeU(); ++u ) {
      const size_t p = v * patch.getSizeU() + u;
      if ( Gmag[p] > minGradient ) { highGradientMap[p] = true; }
    }
  }

  // dilate
  std::vector<bool> tmpHighGradientMap( highGradientMap );
  for ( size_t iteration = 0; iteration < 3; ++iteration ) {
    for ( size_t v = 0; v < patch.getSizeV(); ++v ) {
      for ( size_t u = 0; u < patch.getSizeU(); ++u ) {
        const size_t p = v * patch.getSizeU() + u;
        if ( !highGradientMap[p] ) {
          size_t cnt = 0;
          if ( u > 0 && highGradientMap[p - 1] ) cnt++;
          if ( u < patch.getSizeU() - 1 && highGradientMap[p + 1] ) cnt++;
          if ( v > 0 && highGradientMap[p - patch.getSizeU()] ) cnt++;
          if ( v < patch.getSizeV() - 1 && highGradientMap[p + patch.getSizeU()] ) cnt++;
          if ( cnt >= 2 && Gmag[p] > minGradient / 2.0 ) { tmpHighGradientMap[p] = true; }
        }
      }
    }
    highGradientMap = tmpHighGradientMap;
  }

  // determine candidate points to be separated: points close to the D0 layer, or the normal does not point to the
  // projection plane
  std::vector<bool> flags;
  flags.resize( points.getPointCount(), false );
  for ( size_t i = 0; i < connectedComponent.size(); ++i ) {
    const size_t idx      = connectedComponent[i];
    PCCPoint3D   pointTmp = points[idx];
    if ( bIsAdditionalProjectionPlane ) {
      auto& input = pointTmp;
      convert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, pointTmp );
    }
    const auto&   point = pointTmp;
    const int16_t d     = int16_t( round( point[patch.getNormalAxis()] ) );
    const size_t  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
    const size_t  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
    const size_t  p     = v * patch.getSizeU() + u;
    if ( highGradientMap[p] ) {
      const double score = normalsGen.getNormal( idx ) * orientations[orgPartitionIdx];
      if ( fabs( d - patch.getDepth( 0 )[p] ) <= surfaceThickness || score <= normalThreshold ) {
        flags[idx]   = true;
        isRemoved[i] = true;
      }
    }
  }

  // connect high gradient points
  std::vector<size_t> fifo;
  for ( const auto i : connectedComponent ) {
    if ( flags[i] ) {
      double            bestScore    = 0.0;
      size_t            clusterIndex = orgPartitionIdx;
      const PCCVector3D normal       = normalsGen.getNormal( i );
      for ( size_t k = 0; k < orientationCount; ++k ) {
        const double score = normal * orientations[k];
        if ( score > bestScore ) {
          bestScore    = score;
          clusterIndex = k;
        }
      }
      if ( clusterIndex == orgPartitionIdx ) continue;
      flags[i] = false;
      fifo.push_back( i );
      std::vector<size_t> highGradientGroup;
      highGradientGroup.push_back( i );
      while ( !fifo.empty() ) {
        const size_t current = fifo.back();
        fifo.pop_back();
        for ( const auto n : adj[current] ) {
          const double score = normalsGen.getNormal( n ) * orientations[clusterIndex];
          if ( flags[n] && score > normalThreshold ) {
            flags[n] = false;
            fifo.push_back( n );
            highGradientGroup.push_back( n );
          }
        }
      }
      if ( highGradientGroup.size() > minNumHighGradientPoints ) {
        highGradientConnectedComponents.push_back( highGradientGroup );
      } else {
        for ( const auto j : highGradientGroup ) { flags[j] = true; }
      }
    }
  }

  for ( size_t i = 0; i < connectedComponent.size(); ++i ) {
    if ( flags[connectedComponent[i]] ) { isRemoved[i] = false; }
  }
}
