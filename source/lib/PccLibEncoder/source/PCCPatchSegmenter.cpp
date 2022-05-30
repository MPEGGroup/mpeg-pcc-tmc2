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
#include "PCCPatchSegmenter.h"
#include "PCCPatch.h"
#if defined( ENABLE_TBB )
#include <tbb/tbb.h>
#endif

using namespace pcc;

void PCCPatchSegmenter3::setNbThread( size_t nbThread ) {
  nbThread_ = nbThread;
#if defined( ENABLE_TBB )
  if ( nbThread_ > 0 ) { tbb::task_scheduler_init init( static_cast<int>( nbThread ) ); }
#endif
}

void PCCPatchSegmenter3::compute( const PCCPointSet3&                 geometry,
                                  const size_t                        frameIndex,
                                  const PCCPatchSegmenter3Parameters& params,
                                  std::vector<PCCPatch>&              patches,
                                  std::vector<PCCPointSet3>&          subPointCloud,
                                  float&                              distanceSrcRec ) {
  PCCVector3D* orientations     = nullptr;
  size_t       orientationCount = 0;
  if ( params.additionalProjectionPlaneMode_ == 0 ) {
    orientations     = orientations6;
    orientationCount = orientationCount6;
  } else if ( params.additionalProjectionPlaneMode_ == 1 ) {
    orientations     = orientations10_YAxis;
    orientationCount = 10;
  } else if ( params.additionalProjectionPlaneMode_ == 2 ) {
    orientations     = orientations10_XAxis;
    orientationCount = 10;
  } else if ( params.additionalProjectionPlaneMode_ == 3 ) {
    orientations     = orientations10_ZAxis;
    orientationCount = 10;
  } else if ( params.additionalProjectionPlaneMode_ == 4 ) {
    orientations     = orientations18;
    orientationCount = 18;
  }
  std::cout << std::endl << "============= FRAME " << frameIndex << " ============= " << std::endl;
  PCCPointSet3 geometryVox;
  Voxels       voxels;
  if ( params.gridBasedSegmentation_ ) {
    std::cout << "  Converting points to voxels... ";
    convertPointsToVoxels( geometry, params.geometryBitDepth3D_, params.voxelDimensionGridBasedSegmentation_,
                           geometryVox, voxels );
    std::cout << "[done]" << std::endl;
  } else {
    geometryVox = geometry;
  }
  std::cout << "  Computing normals for original point cloud... ";
  PCCKdTree            kdtree( geometryVox );
  PCCNNResult          result;
  PCCNormalsGenerator3 normalsGen;
  auto                 normalsOrientation = static_cast<PCCNormalsGeneratorOrientation>( params.normalOrientation_ );
  const PCCNormalsGenerator3Parameters normalsGenParams = {PCCVector3D( 0.0 ),
                                                           ( std::numeric_limits<double>::max )(),
                                                           ( std::numeric_limits<double>::max )(),
                                                           ( std::numeric_limits<double>::max )(),
                                                           ( std::numeric_limits<double>::max )(),
                                                           params.nnNormalEstimation_,
                                                           params.nnNormalEstimation_,
                                                           params.nnNormalEstimation_,
                                                           0,
                                                           normalsOrientation,
                                                           false,
                                                           false,
                                                           false};
  // PCC_NORMALS_GENERATOR_ORIENTATION_SPANNING_TREE,
  normalsGen.compute( geometryVox, kdtree, normalsGenParams, nbThread_ );
  std::cout << "[done]" << std::endl;

  std::cout << "  Computing initial segmentation... ";
  std::vector<size_t> partition;
  if ( params.additionalProjectionPlaneMode_ == 0 ) {
    initialSegmentation( geometryVox, normalsGen, orientations, orientationCount, partition, params.weightNormal_ );
  } else {
    initialSegmentation( geometryVox, normalsGen, orientations, orientationCount,
                         partition );  // flat weight
  }
  std::cout << "[done]" << std::endl;

  if ( params.gridBasedRefineSegmentation_ ) {
    std::cout << "  Refining segmentation (grid-based)... ";
    refineSegmentationGridBased( geometryVox, normalsGen, orientations, orientationCount,
                                 params.maxNNCountRefineSegmentation_, params.lambdaRefineSegmentation_,
                                 params.iterationCountRefineSegmentation_, params.voxelDimensionRefineSegmentation_,
                                 params.searchRadiusRefineSegmentation_, partition );
  } else {
    std::cout << "  Refining segmentation... ";
    refineSegmentation( geometryVox, kdtree, normalsGen, orientations, orientationCount,
                        params.maxNNCountRefineSegmentation_, params.lambdaRefineSegmentation_,
                        params.iterationCountRefineSegmentation_, partition );
  }
  std::cout << "[done]" << std::endl;

  if ( params.gridBasedSegmentation_ ) {
    std::cout << "  Applying voxels' data to points... ";
    applyVoxelsDataToPoints( geometry.getPointCount(), params.geometryBitDepth3D_,
                             params.voxelDimensionGridBasedSegmentation_, voxels, geometryVox, normalsGen, partition );
    std::cout << "[done]" << std::endl;
    kdtree.init( geometry );
  }
  std::cout << "  Patch segmentation... ";
  PCCPointSet3        resampled;
  std::vector<size_t> patchPartition;
  std::vector<size_t> resampledPatchPartition;
  std::vector<size_t> rawPoints;

  segmentPatches( geometry, frameIndex, kdtree, params, partition, patches, patchPartition, resampledPatchPartition,
                  rawPoints, resampled, subPointCloud, distanceSrcRec, normalsGen, orientations, orientationCount );
  std::cout << "[done]" << std::endl;
}

void PCCPatchSegmenter3::convertPointsToVoxels( const PCCPointSet3& source,
                                                size_t              geoBits,
                                                size_t              voxDim,
                                                PCCPointSet3&       sourceVox,
                                                Voxels&             voxels ) {
#define ADD_COLOR 0
  const size_t geoBits2    = geoBits << 1;
  size_t       voxDimShift = 0;
  for ( size_t i = voxDim; i > 1; ++voxDimShift, i >>= 1 ) { ; }
  const size_t voxDimHalf = voxDim >> 1;

  auto subToInd = [&]( uint64_t x, uint64_t y, uint64_t z ) { return x + ( y << geoBits ) + ( z << geoBits2 ); };

  for ( size_t i = 0; i < source.getPointCount(); ++i ) {
    const auto& pos = source[i];
    const auto  x0  = ( static_cast<uint64_t>( pos[0] ) + voxDimHalf ) >> voxDimShift;
    const auto  y0  = ( static_cast<uint64_t>( pos[1] ) + voxDimHalf ) >> voxDimShift;
    const auto  z0  = ( static_cast<uint64_t>( pos[2] ) + voxDimHalf ) >> voxDimShift;
    const auto  p   = subToInd( x0, y0, z0 );
    if ( voxels.count( p ) == 0u ) {
      voxels[p].reserve( voxDim * voxDim << 1 );
      const size_t j = sourceVox.addPoint( PCCPoint3D( x0, y0, z0 ) );
#if ADD_COLOR
      sourceVox.addColors();
      sourceVox.setColor( j, source.getColor( i ) );
#endif
    }
    voxels[p].push_back( i );
  }
}

void PCCPatchSegmenter3::applyVoxelsDataToPoints( size_t                pointCount,
                                                  size_t                geoBits,
                                                  size_t                voxDim,
                                                  Voxels&               voxels,  // const
                                                  const PCCPointSet3&   sourceVox,
                                                  PCCNormalsGenerator3& normalsGen,
                                                  std::vector<size_t>&  partitions ) {
  std::vector<size_t>      partitionsTmp( pointCount );
  std::vector<PCCVector3D> normalsTmp( pointCount );
  auto&                    normals = normalsGen.getNormals();

  const size_t geoBits2    = geoBits << 1;
  size_t       voxDimShift = 0;
  for ( size_t i = voxDim; i > 1; ++voxDimShift, i >>= 1 ) { ; }
  const size_t voxDimHalf = voxDim >> 1;

  auto subToInd = [&]( const PCCPoint3D& point ) {
    return (uint64_t)point[0] + ( (uint64_t)point[1] << geoBits ) + ( (uint64_t)point[2] << geoBits2 );
  };

  for ( size_t i = 0; i < sourceVox.getPointCount(); i++ ) {
    const auto& partition = partitions[i];
    const auto& normal    = normals[i];
    const auto& pos       = sourceVox[i];
    const auto& indices   = voxels[subToInd( pos )];
    for ( const auto& index : indices ) {
      partitionsTmp[index] = partition;
      normalsTmp[index]    = normal;
    }
  }
  swap( partitions, partitionsTmp );
  swap( normalsGen.getNormals(), normalsTmp );
}

void PCCPatchSegmenter3::initialSegmentation( const PCCPointSet3&         geometry,
                                              const PCCNormalsGenerator3& normalsGen,
                                              const PCCVector3D*          orientations,
                                              const size_t                orientationCount,
                                              std::vector<size_t>&        partition ) {
  PCCVector3D axis_weight;
  axis_weight[0] = axis_weight[1] = axis_weight[2] = 1.0F;
  initialSegmentation( geometry, normalsGen, orientations, orientationCount, partition, axis_weight );
}
void PCCPatchSegmenter3::initialSegmentation( const PCCPointSet3&         geometry,
                                              const PCCNormalsGenerator3& normalsGen,
                                              const PCCVector3D*          orientations,
                                              const size_t                orientationCount,
                                              std::vector<size_t>&        partition,
                                              const PCCVector3D&          axisWeight ) {
  assert( orientations );
  const size_t pointCount = geometry.getPointCount();
  partition.resize( pointCount );
  double weightValue[18];
  int    i;
  for ( i = 0; i < 18; i++ ) { weightValue[i] = 1.0F; }
  weightValue[0] = weightValue[3] = axisWeight[0];
  weightValue[1] = weightValue[4] = axisWeight[1];
  weightValue[2] = weightValue[5] = axisWeight[2];
#if defined( ENABLE_TBB )
  tbb::task_arena limited( static_cast<int>( nbThread_ ) );
  limited.execute( [&] {
    tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) {
#else
  for ( size_t i = 0; i < pointCount; i++ ) {
#endif
      const PCCVector3D normal       = normalsGen.getNormal( i );
      size_t            clusterIndex = 0;
      double            bestScore    = normal * orientations[0];
      for ( size_t j = 1; j < orientationCount; ++j ) {
        const double score = normal * orientations[j] * weightValue[j];
        if ( score > bestScore ) {
          bestScore    = score;
          clusterIndex = j;
        }
      }
      partition[i] = clusterIndex;
#if defined( ENABLE_TBB )
    } );
  } );
#else
  }
#endif
}

void PCCPatchSegmenter3::computeAdjacencyInfo( const PCCPointSet3&               pointCloud,
                                               const PCCKdTree&                  kdtree,
                                               std::vector<std::vector<size_t>>& adj,
                                               const size_t                      maxNNCount ) {
  const size_t pointCount = pointCloud.getPointCount();
  adj.resize( pointCount );
#if defined( ENABLE_TBB )
  tbb::task_arena limited( static_cast<int>( nbThread_ ) );
  limited.execute( [&] {
    tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) {
#else
  for ( size_t i = 0; i < pointCount; i++ ) {
#endif
      PCCNNResult result;
      kdtree.search( pointCloud[i], maxNNCount, result );
      std::vector<size_t>& neighbors = adj[i];
      neighbors.resize( result.count() );
      for ( size_t j = 0; j < result.count(); ++j ) { neighbors[j] = result.indices( j ); }
#if defined( ENABLE_TBB )
      } );
    } );
#else
  }
#endif
}

void PCCPatchSegmenter3::computeAdjacencyInfoInRadius( const PCCPointSet3&                 pointCloud,
                                                       const PCCKdTree&                    kdtree,
                                                       std::vector<std::vector<uint32_t>>& adj,
                                                       const size_t                        maxNNCount,
                                                       const size_t                        radius ) {
  const size_t pointCount = pointCloud.getPointCount();
  adj.resize( pointCount );
#if defined( ENABLE_TBB )
  tbb::task_arena limited( static_cast<int>( nbThread_ ) );
  limited.execute( [&] {
    tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) {
#else
  for ( size_t i = 0; i < pointCount; i++ ) {
#endif
      PCCNNResult result;
      kdtree.searchRadius( pointCloud[i], maxNNCount, radius, result );
      std::vector<uint32_t>& neighbors = adj[i];
      neighbors.resize( result.count() );
      for ( size_t j = 0; j < result.count(); ++j ) { neighbors[j] = result.indices( j ); }
#if defined( ENABLE_TBB )
      } );
    } );
#else
  }
#endif
}

void PCCPatchSegmenter3::computeAdjacencyInfoDist( const PCCPointSet3&               pointCloud,
                                                   const PCCKdTree&                  kdtree,
                                                   std::vector<std::vector<size_t>>& adj,
                                                   std::vector<std::vector<double>>& adjDist,
                                                   const size_t                      maxNNCount ) {
  const size_t pointCount = pointCloud.getPointCount();
  adj.resize( pointCount );
  adjDist.resize( pointCount );
#if defined( ENABLE_TBB )
  tbb::task_arena limited( static_cast<int>( nbThread_ ) );
  limited.execute( [&] {
    tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) {
#else
  for ( size_t i = 0; i < pointCount; i++ ) {
#endif
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
#if defined( ENABLE_TBB )
    } );
  } );
#else
  }
#endif
}

void printChunk( const std::vector<std::pair<int, int>>& chunk ) {
  std::vector<std::string> axisName{"x -> ", "y -> ", "z -> "};
  for ( size_t axis = 0; axis < 3; ++axis ) {
    std::cout << axisName[axis] << "(" << chunk[axis].first << ", " << chunk[axis].second << ")"
              << " ";
  }
  std::cout << std::endl;
}

void PCCPatchSegmenter3::resampledPointcloud( std::vector<size_t>& pointCount,
                                              PCCPointSet3&        resampled,
                                              std::vector<size_t>& resampledPatchPartition,
                                              PCCPatch&            patch,
                                              size_t               patchIndex,
                                              bool                 multipleMaps,
                                              size_t               surfaceThickness,
                                              size_t               EOMFixBitCount,
                                              bool                 bIsAdditionalProjectionPlane,
                                              bool                 useEnhancedOccupancyMapCode,
                                              size_t               geometryBitDepth3D,
                                              bool                 createSubPointCloud,
                                              PCCPointSet3&        rec ) {
  pointCount.resize( 3 );
  size_t  d0CountPerPatch  = 0;
  size_t  d1CountPerPatch  = 0;
  size_t  eomCountPerPatch = 0;
  int16_t projectionTypeIndication =
      ( -2 * static_cast<int16_t>( patch.getProjectionMode() ) + 1 );  // projection=0 -> 1, projection=1 -> -1
  for ( size_t v = 0; v < patch.getSizeV(); ++v ) {
    for ( size_t u = 0; u < patch.getSizeU(); ++u ) {
      const size_t p = v * patch.getSizeU() + u;
      if ( patch.getDepth( 0 )[p] < g_infiniteDepth ) {
        int16_t depth0 = patch.getDepth( 0 )[p];
        // add depth0
        const size_t u0 = u / patch.getOccupancyResolution();
        const size_t v0 = v / patch.getOccupancyResolution();
        const size_t p0 = v0 * patch.getSizeU0() + u0;
        assert( u0 < patch.getSizeU0() );
        assert( v0 < patch.getSizeV0() );
        patch.setOccupancy( p0, true );

        PCCVector3D point;
        point[patch.getNormalAxis()]    = double( depth0 );
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
        d0CountPerPatch++;

        // add EOM
        PCCVector3D pointEOM( point );
        if ( useEnhancedOccupancyMapCode && patch.getDepthEOM()[p] != 0 ) {
          size_t N = multipleMaps ? surfaceThickness : EOMFixBitCount;
          for ( uint16_t i = 0; i < N; i++ ) {
            if ( ( patch.getDepthEOM()[p] & ( 1 << i ) ) != 0 ) {
              uint16_t nDeltaDCur             = ( i + 1 );
              pointEOM[patch.getNormalAxis()] = double( depth0 + projectionTypeIndication * ( nDeltaDCur ) );
              if ( pointEOM[patch.getNormalAxis()] != point[patch.getNormalAxis()] ) {
                if ( bIsAdditionalProjectionPlane ) {
                  PCCVector3D point_tmp;
                  auto&       input = pointEOM;
                  iconvert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, point_tmp );
                  resampled.addPoint( point_tmp );
                  resampledPatchPartition.push_back( patchIndex );
                } else {
                  resampled.addPoint( pointEOM );
                  resampledPatchPartition.push_back( patchIndex );
                }
                eomCountPerPatch++;
              }
            }
          }  // N
        }

        // add depth1
        if ( !( useEnhancedOccupancyMapCode && !multipleMaps ) ) {
          int16_t depth1               = patch.getDepth( 1 )[p];
          point[patch.getNormalAxis()] = double( depth1 );
          if ( patch.getDepth( 0 )[p] != patch.getDepth( 1 )[p] ) { d1CountPerPatch++; }
          if ( bIsAdditionalProjectionPlane ) {
            PCCVector3D point_tmp;
            auto&       input = point;
            iconvert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, point_tmp );
            resampled.addPoint( point_tmp );
            resampledPatchPartition.push_back( patchIndex );
            if ( createSubPointCloud ) { rec.addPoint( point_tmp ); }
          } else if ( !useEnhancedOccupancyMapCode ||
                      pointEOM[patch.getNormalAxis()] != point[patch.getNormalAxis()] ) {
            resampled.addPoint( point );
            resampledPatchPartition.push_back( patchIndex );
            if ( createSubPointCloud ) { rec.addPoint( point ); }
          }
          assert( abs( patch.getDepth( 0 )[p] - patch.getDepth( 1 )[p] ) <= int( surfaceThickness ) );
        }
        // adjust depth(0), depth(1)
        patch.setDepth( 0, p, projectionTypeIndication * ( patch.getDepth( 0 )[p] - int16_t( patch.getD1() ) ) );
        patch.setSizeD( ( std::max )( patch.getSizeD(), static_cast<size_t>( patch.getDepth( 0 )[p] ) ) );
        if ( !( useEnhancedOccupancyMapCode && !multipleMaps ) ) {
          patch.setDepth( 1, p, projectionTypeIndication * ( patch.getDepth( 1 )[p] - int16_t( patch.getD1() ) ) );
          patch.setSizeD( ( std::max )( patch.getSizeD(), static_cast<size_t>( patch.getDepth( 1 )[p] ) ) );
        }
      }
    }
  }
  pointCount[0] = d0CountPerPatch;
  pointCount[1] = d1CountPerPatch;
  pointCount[2] = eomCountPerPatch;
}

int16_t PCCPatchSegmenter3::getPatchSurfaceThickness( const PCCPointSet3&      points,
                                                      PCCPatch&                patch,
                                                      size_t                   patchIndex,
                                                      std::vector<PCCColor3B>& frame_pcc_color,
                                                      std::vector<size_t>&     connectedComponent,
                                                      size_t                   surfaceThickness,
                                                      size_t                   projectionMode,
                                                      bool                     bIsAdditionalProjectionPlane,
                                                      size_t                   geometryBitDepth3D ) {
  const double         Threshold_Color_Error = 400;
  std::vector<size_t>  patchSurfaceThicknessList;
  std::vector<int64_t> errSumList;
  std::vector<int32_t> totalNumList;
  std::vector<int32_t> d1NumList;
  patchSurfaceThicknessList.resize( surfaceThickness, 0 );
  errSumList.resize( surfaceThickness, 0 );
  totalNumList.resize( surfaceThickness, 0 );
  d1NumList.resize( surfaceThickness, 0 );
  int16_t patch_surfaceThickness = -1;
  for ( size_t i = 0; i < surfaceThickness; i++ ) {
    patchSurfaceThicknessList[i] = surfaceThickness - i;
    errSumList[i]                = 0;
    totalNumList[i]              = 0;
    d1NumList[i]                 = 0;
  }

  for ( const auto i : connectedComponent ) {
    PCCPoint3D pointTmp = points[i];
    if ( bIsAdditionalProjectionPlane ) {
      auto& input = pointTmp;
      convert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, pointTmp );
    }
    const auto& point = pointTmp;
    const auto  d     = int16_t( round( point[patch.getNormalAxis()] ) );
    const auto  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
    const auto  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
    assert( u >= 0 && u < patch.getSizeU() );
    assert( v >= 0 && v < patch.getSizeV() );
    const size_t  p      = v * patch.getSizeU() + u;
    const int16_t depth0 = patch.getDepth( 0 )[p];

    if ( !( depth0 < g_infiniteDepth ) ) { continue; }
    bool bsimilar = colorSimilarity( frame_pcc_color[i], frame_pcc_color[patch.getDepth0PccIdx()[p]], 128 );
    for ( size_t thickness = 0; thickness < patchSurfaceThicknessList.size(); thickness++ ) {
      bool bValid =
          projectionMode != 0
              ? ( depth0 < g_infiniteDepth && ( depth0 - d ) <= int16_t( patchSurfaceThicknessList[thickness] ) &&
                  d < patch.getDepth( 1 )[p] && bsimilar )
              : ( depth0 < g_infiniteDepth && ( d - depth0 ) <= int16_t( patchSurfaceThicknessList[thickness] ) &&
                  d > patch.getDepth( 1 )[p] && bsimilar );
      if ( bValid ) {
        d1NumList[thickness]++;
        const size_t d0_idx = patch.getDepth0PccIdx()[p];
        const size_t d1_idx = i;
        errSumList[thickness] += colorDifference( frame_pcc_color[d0_idx], frame_pcc_color[d1_idx] );
      }
    }
  }  // connectcomponent

  for ( size_t i = 0; i < patchSurfaceThicknessList.size(); i++ ) {
    double avg_error =
        d1NumList[i] == 0 ? 0 : static_cast<double>( errSumList[i] ) / static_cast<double>( d1NumList[i] );
    if ( avg_error < Threshold_Color_Error ) {
      patch_surfaceThickness = patchSurfaceThicknessList[i];
      break;
    }
  }
  return patch_surfaceThickness;
}

void PCCPatchSegmenter3::segmentPatches( const PCCPointSet3&                 points,
                                         const size_t                        frameIndex,
                                         const PCCKdTree&                    kdtree,
                                         const PCCPatchSegmenter3Parameters& params,
                                         std::vector<size_t>&                partition,
                                         std::vector<PCCPatch>&              patches,
                                         std::vector<size_t>&                patchPartition,
                                         std::vector<size_t>&                resampledPatchPartition,
                                         std::vector<size_t>                 rawPoints,
                                         PCCPointSet3&                       resampled,
                                         std::vector<PCCPointSet3>&          subPointCloud,
                                         float&                              distanceSrcRec,
                                         const PCCNormalsGenerator3&         normalsGen,
                                         const PCCVector3D*                  orientations,
                                         const size_t                        orientationCount ) {
  const size_t     maxNNCount                        = params.maxNNCountPatchSegmentation_;
  const size_t     minPointCountPerCC                = params.minPointCountPerCCPatchSegmentation_;
  const size_t     occupancyResolution               = params.occupancyResolution_;
  const size_t     quantizerSizeX                    = params.quantizerSizeX_;
  const size_t     quantizerSizeY                    = params.quantizerSizeY_;
  const double     maxAllowedDist2RawPointsDetection = params.maxAllowedDist2RawPointsDetection_;
  const double     maxAllowedDist2RawPointsSelection = params.maxAllowedDist2RawPointsSelection_;
  const bool       EOMSingleLayerMode                = params.EOMSingleLayerMode_;
  const size_t     EOMFixBitCount                    = params.EOMFixBitCount_;
  const size_t     surfaceThickness                  = params.surfaceThickness_;
  const size_t     maxAllowedDepth                   = params.maxAllowedDepth_;
  const size_t     minLevel                          = params.minLevel_;
  bool             useEnhancedOccupancyMapCode       = params.useEnhancedOccupancyMapCode_;
  const bool       createSubPointCloud               = params.createSubPointCloud_;
  const bool       absoluteD1                        = params.absoluteD1_;
  bool             useSurfaceSeparation              = params.surfaceSeparation_;
  const size_t     additionalProjectionAxis          = params.additionalProjectionPlaneMode_;
  const size_t     geometryBitDepth2D                = params.geometryBitDepth2D_;
  const size_t     geometryBitDepth3D                = params.geometryBitDepth3D_;
  bool             patchExpansionEnabled             = params.patchExpansion_;
  const bool       highGradientSeparation            = params.highGradientSeparation_;
  const double     minGradient                       = params.minGradient_;
  const size_t     minNumHighGradientPoints          = params.minNumHighGradientPoints_;
  bool             enablePointCloudPartitioning      = params.enablePointCloudPartitioning_;
  std::vector<int> roiBoundingBoxMinX = const_cast<PCCPatchSegmenter3Parameters&>( params ).roiBoundingBoxMinX_;
  std::vector<int> roiBoundingBoxMaxX = const_cast<PCCPatchSegmenter3Parameters&>( params ).roiBoundingBoxMaxX_;
  std::vector<int> roiBoundingBoxMinY = const_cast<PCCPatchSegmenter3Parameters&>( params ).roiBoundingBoxMinY_;
  std::vector<int> roiBoundingBoxMaxY = const_cast<PCCPatchSegmenter3Parameters&>( params ).roiBoundingBoxMaxY_;
  std::vector<int> roiBoundingBoxMinZ = const_cast<PCCPatchSegmenter3Parameters&>( params ).roiBoundingBoxMinZ_;
  std::vector<int> roiBoundingBoxMaxZ = const_cast<PCCPatchSegmenter3Parameters&>( params ).roiBoundingBoxMaxZ_;
  int numCutsAlong1stLongestAxis      = const_cast<PCCPatchSegmenter3Parameters&>( params ).numCutsAlong1stLongestAxis_;
  int numCutsAlong2ndLongestAxis      = const_cast<PCCPatchSegmenter3Parameters&>( params ).numCutsAlong2ndLongestAxis_;
  int numCutsAlong3rdLongestAxis      = const_cast<PCCPatchSegmenter3Parameters&>( params ).numCutsAlong3rdLongestAxis_;
  const size_t pointCount             = points.getPointCount();
  patchPartition.resize( pointCount, 0 );
  resampledPatchPartition.reserve( pointCount );
  PCCNNResult             result;
  std::vector<PCCColor3B> frame_pcc_color;
  frame_pcc_color.reserve( pointCount );
  for ( size_t i = 0; i < pointCount; i++ ) { frame_pcc_color.push_back( points.getColor( i ) ); }
  if ( enablePointCloudPartitioning ) { assert( patchExpansionEnabled == false ); }

  size_t numD0Points      = 0;
  size_t numD1Points      = 0;
  size_t numEOMOnlyPoints = 0;
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

      std::vector<std::vector<Range>> chunks;
      // chunks[c][x]: range of x-th axis of c-th chunk cut each ROI into chunks for each ROI, sort axes according to
      // their length and cut w.r.t to numCutsAlong[1st,2nd,3rd]LongestAxis
      for ( int roiIndex = 0; roiIndex < numROIs; ++roiIndex ) {
        // derive tight ROI bounding box
        int x_min;
        int x_max;
        int y_min;
        int y_max;
        int z_min;
        int z_max;
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
        std::cout << "\n3nd longest axis of ROI " << roiIndex << " is: " << axisDelta[2].first << std::endl;

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

      numChunks = static_cast<int>( chunks.size() );

      for ( int i = 0; i < numChunks; ++i ) {
        std::cout << "Chunk " << i << "/" << numChunks << " : " << std::endl;
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
  std::vector<double> rawPointsDistance;
  rawPoints.resize( pointCount );
  rawPointsDistance.resize( pointCount );
  for ( size_t i = 0; i < pointCount; ++i ) {
    rawPoints[i]         = i;
    rawPointsDistance[i] = ( std::numeric_limits<double>::max )();
  }
  std::vector<std::vector<size_t>> rawPointsChunks;
  std::vector<std::vector<double>> rawPointsDistanceChunks;
  if ( enablePointCloudPartitioning ) {
    rawPointsChunks.resize( numChunks );
    rawPointsDistanceChunks.resize( numChunks );
    for ( size_t chunkIndex = 0; chunkIndex < numChunks; ++chunkIndex ) {
      rawPointsChunks[chunkIndex].resize( pointCountChunks[chunkIndex] );
      rawPointsDistanceChunks[chunkIndex].resize( pointCountChunks[chunkIndex] );
    }

    for ( size_t chunkIndex = 0; chunkIndex < numChunks; ++chunkIndex ) {
      for ( size_t i = 0; i < pointCountChunks[chunkIndex]; ++i ) {
        rawPointsChunks[chunkIndex][i]         = i;
        rawPointsDistanceChunks[chunkIndex][i] = ( std::numeric_limits<double>::max )();
      }
    }
  }
  subPointCloud.clear();
  double meanPAB     = 0.0;
  double meanYAB     = 0.0;
  double meanUAB     = 0.0;
  double meanVAB     = 0.0;
  double meanPBA     = 0.0;
  double meanYBA     = 0.0;
  double meanUBA     = 0.0;
  double meanVBA     = 0.0;
  size_t testSrcNum  = 0;
  size_t testRecNum  = 0;
  size_t numberOfEOM = 0;
  while ( !rawPoints.empty() ) {
    std::vector<std::vector<size_t>> connectedComponents;
    if ( !enablePointCloudPartitioning ) {
      std::vector<size_t> fifo;
      fifo.reserve( pointCount );
      std::vector<bool> flags;
      flags.resize( pointCount, false );
      for ( const auto i : rawPoints ) { flags[i] = true; }
      connectedComponents.reserve( 256 );
      for ( const auto i : rawPoints ) {
        if ( flags[i] && rawPointsDistance[i] > maxAllowedDist2RawPointsDetection ) {
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
      std::vector<std::vector<std::vector<size_t>>> connectedComponentsChunks( numChunks );
      for ( size_t chunkIndex = 0; chunkIndex < numChunks; ++chunkIndex ) {
        std::cout << "\n\t Extracting connected components of chunk " << chunkIndex << "... ";
        std::vector<size_t> fifo;
        fifo.reserve( pointCountChunks[chunkIndex] );
        std::vector<bool> flags;
        flags.resize( pointCountChunks[chunkIndex], false );
        for ( const auto i : rawPointsChunks[chunkIndex] ) { flags[i] = true; }
        connectedComponentsChunks[chunkIndex].reserve( 256 );
        for ( const auto i : rawPointsChunks[chunkIndex] ) {
          if ( flags[i] && rawPointsDistanceChunks[chunkIndex][i] > maxAllowedDist2RawPointsDetection ) {
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
                 []( const std::vector<size_t>& a, const std::vector<size_t>& b ) { return a.size() >= b.size(); } );
    }
    for ( auto& connectedComponent : connectedComponents ) {
      const size_t patchIndex = patches.size();
      patches.resize( patchIndex + 1 );
      if ( createSubPointCloud ) { subPointCloud.resize( patchIndex + 1 ); }
      PCCPatch& patch            = patches[patchIndex];
      size_t    d0CountPerPatch  = 0;
      size_t    d1CountPerPatch  = 0;
      size_t    eomCountPerPatch = 0;
      patch.setIndex( patchIndex );
      patch.setEOMCount( 0 );
      patch.setPatchType( static_cast<uint8_t>( P_INTRA ) );
      size_t clusterIndex                 = partition[connectedComponent[0]];
      bool   bIsAdditionalProjectionPlane = ( clusterIndex > 5 );  // false;
      if ( bIsAdditionalProjectionPlane && ( additionalProjectionAxis == 2 ) ) clusterIndex += 4;
      if ( bIsAdditionalProjectionPlane && ( additionalProjectionAxis == 3 ) ) clusterIndex += 8;
      patch.setViewId( clusterIndex );
      patch.setBestMatchIdx( g_invalidPatchIndex );
      patch.getPreGPAPatchData().initialize();
      patch.getCurGPAPatchData().initialize();
      if ( params.enablePatchSplitting_ ) {
        int16_t minU = ( std::numeric_limits<int16_t>::max )();
        int16_t minV = ( std::numeric_limits<int16_t>::max )();
        for ( const auto i : connectedComponent ) {
          PCCPoint3D pointTmp = points[i];
          if ( bIsAdditionalProjectionPlane ) {
            auto& input = pointTmp;
            convert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, pointTmp );
          }
          const auto& point = pointTmp;
          minU              = ( std::min )( minU, int16_t( round( point[patch.getTangentAxis()] ) ) );
          minV              = ( std::min )( minV, int16_t( round( point[patch.getBitangentAxis()] ) ) );
        }
        std::vector<size_t> tempCC;
        tempCC.resize( 0 );
        for ( const auto i : connectedComponent ) {
          PCCPoint3D pointTmp = points[i];
          if ( bIsAdditionalProjectionPlane ) {
            auto& input = pointTmp;
            convert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, pointTmp );
          }
          const auto& point = pointTmp;
          const auto  u     = int16_t( round( point[patch.getTangentAxis()] ) );
          const auto  v     = int16_t( round( point[patch.getBitangentAxis()] ) );
          if ( u - minU < params.maxPatchSize_ && v - minV < params.maxPatchSize_ ) { tempCC.push_back( i ); }
        }
        connectedComponent = tempCC;
        if ( connectedComponent.empty() ) { continue; }
      }

      const int16_t projectionDirectionType = -2 * patch.getProjectionMode() + 1;

      if ( patchExpansionEnabled ) {
        for ( const auto i : connectedComponent ) { flagExp[i] = true; }
        std::vector<size_t> fifoa;
        fifoa.reserve( pointCount );
        for ( const auto i : connectedComponent ) {
          for ( size_t ac = 0; ac < adj[i].size(); ++ac ) {
            const size_t n = adj[i][ac];
            if ( flagExp[n] ) { continue; }
            if ( ( clusterIndex == partition[n] ) ||  // same plane
                 ( clusterIndex + 3 == partition[n] ) || ( clusterIndex == partition[n] + 3 ) ) {
              continue;
            }
            const double dist2 = adjDist[i][ac];  // sum of square
            if ( dist2 <= 2 ) {                   // <-- expansion distance
              fifoa.push_back( n );
              flagExp[n] = true;  // add point
            }
          }
        }
        if ( !fifoa.empty() ) { connectedComponent.insert( connectedComponent.end(), fifoa.begin(), fifoa.end() ); }
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
          if ( roiBB.fullyContains( boundingBox ) ) { patch.setRoiIndex( roiIndex ); }
        }
      }
      patch.setSizeU( 1 + size_t( round( boundingBox.max_[patch.getTangentAxis()] ) -
                                  floor( boundingBox.min_[patch.getTangentAxis()] ) ) );
      patch.setSizeV( 1 + size_t( round( boundingBox.max_[patch.getBitangentAxis()] ) -
                                  floor( boundingBox.min_[patch.getBitangentAxis()] ) ) );
      patch.setU1( size_t( boundingBox.min_[patch.getTangentAxis()] ) );
      patch.setV1( size_t( boundingBox.min_[patch.getBitangentAxis()] ) );
      patch.setD1( patch.getProjectionMode() == 0 ? g_infiniteDepth : 0 );
      patch.allocDepth( 0, patch.getSizeU() * patch.getSizeV(), g_infiniteDepth );
      patch.allocDepth0PccIdx( patch.getSizeU() * patch.getSizeV(), g_infinitenumber );
      if ( useEnhancedOccupancyMapCode ) { patch.allocDepthEOM( patch.getSizeU() * patch.getSizeV(), 0 ); }
      patch.setOccupancyResolution( occupancyResolution );
      patch.setSizeU0( 0 );
      patch.setSizeV0( 0 );
      patch.setPatchSize2DXInPixel( 0 );
      patch.setPatchSize2DYInPixel( 0 );
      for ( const auto i : connectedComponent ) {
        PCCPoint3D pointTmp = points[i];
        if ( bIsAdditionalProjectionPlane ) {
          auto& input = pointTmp;
          convert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, pointTmp );
        }
        const auto& point = pointTmp;
        const auto  d     = int16_t( round( point[patch.getNormalAxis()] ) );
        const auto  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
        const auto  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
        assert( u >= 0 && u < patch.getSizeU() );
        assert( v >= 0 && v < patch.getSizeV() );
        const size_t p           = v * patch.getSizeU() + u;
        bool         bValidPoint = ( patch.getProjectionMode() == 0 )
                               ? ( patch.getDepth( 0 )[p] > d )
                               : ( ( patch.getDepth( 0 )[p] == g_infiniteDepth ) || ( patch.getDepth( 0 )[p] < d ) );
        if ( bValidPoint ) {  // min
          int16_t minD0 = patch.getD1();
          int16_t maxD0 = patch.getD1();
          patch.setDepth( 0, p, d );
          patch.setDepth0PccIdx( p, i );
          patch.setPatchSize2DXInPixel( ( std::max )( patch.getPatchSize2DXInPixel(), u ) );
          patch.setPatchSize2DYInPixel( ( std::max )( patch.getPatchSize2DYInPixel(), v ) );
          patch.setSizeU0( ( std::max )( patch.getSizeU0(), u / patch.getOccupancyResolution() ) );
          patch.setSizeV0( ( std::max )( patch.getSizeV0(), v / patch.getOccupancyResolution() ) );
          minD0 = ( std::min )( minD0, d );
          maxD0 = ( std::max )( maxD0, d );
          if ( patch.getProjectionMode() == 0 ) {
            patch.setD1( ( minD0 / minLevel ) * minLevel );
          } else {
            patch.setD1( size_t( ceil( static_cast<double>( maxD0 ) / static_cast<double>( minLevel ) ) ) * minLevel );
          }
        }
      }  // i

      patch.setPatchSize2DXInPixel( ( patch.getPatchSize2DXInPixel() + 1 ) );
      patch.setPatchSize2DYInPixel( ( patch.getPatchSize2DYInPixel() + 1 ) );
      size_t noquantizedPatchSize2DX = ( patch.getPatchSize2DXInPixel() );
      size_t noquantizedPatchSize2DY = ( patch.getPatchSize2DYInPixel() );
      if ( quantizerSizeX != 0 ) {
        patch.setPatchSize2DXInPixel(
            ceil( static_cast<double>( noquantizedPatchSize2DX ) / static_cast<double>( quantizerSizeX ) ) *
            quantizerSizeX );
      }
      if ( quantizerSizeY != 0 ) {
        patch.setPatchSize2DYInPixel(
            ceil( static_cast<double>( noquantizedPatchSize2DY ) / static_cast<double>( quantizerSizeY ) ) *
            quantizerSizeY );
      }
      patch.setSizeU0( patch.getSizeU0() + 1 );
      patch.setSizeV0( patch.getSizeV0() + 1 );
      patch.allocOccupancy( patch.getSizeU0() * patch.getSizeV0(), false );

      // filter depth
      std::vector<int16_t> peakPerBlock;
      peakPerBlock.resize( patch.getSizeU0() * patch.getSizeV0(),
                           patch.getProjectionMode() == 0 ? g_infiniteDepth : 0 );
      for ( int64_t v = 0; v < int64_t( patch.getSizeV() ); ++v ) {
        for ( int64_t u = 0; u < int64_t( patch.getSizeU() ); ++u ) {
          const size_t  p      = v * patch.getSizeU() + u;
          const int16_t depth0 = patch.getDepth( 0 )[p];
          if ( depth0 == g_infiniteDepth ) { continue; }
          const size_t u0 = u / patch.getOccupancyResolution();
          const size_t v0 = v / patch.getOccupancyResolution();
          const size_t p0 = v0 * patch.getSizeU0() + u0;
          if ( patch.getProjectionMode() == 0 ) {
            peakPerBlock[p0] = ( std::min )( peakPerBlock[p0], depth0 );
          } else {
            peakPerBlock[p0] = ( std::max )( peakPerBlock[p0], depth0 );
          }
        }  // u
      }    // v
      for ( int64_t v = 0; v < int64_t( patch.getSizeV() ); ++v ) {
        for ( int64_t u = 0; u < int64_t( patch.getSizeU() ); ++u ) {
          const size_t  p      = v * patch.getSizeU() + u;
          const int16_t depth0 = patch.getDepth( 0 )[p];
          if ( depth0 == g_infiniteDepth ) { continue; }
          const size_t u0    = u / patch.getOccupancyResolution();
          const size_t v0    = v / patch.getOccupancyResolution();
          const size_t p0    = v0 * patch.getSizeU0() + u0;
          int16_t      tmp_a = std::abs( depth0 - peakPerBlock[p0] );
          int16_t      tmp_b = int16_t( surfaceThickness ) + projectionDirectionType * depth0;
          int16_t      tmp_c = projectionDirectionType * patch.getD1() + int16_t( maxAllowedDepth );
          if ( depth0 != g_infiniteDepth ) {
            if ( ( tmp_a > 32 ) || ( tmp_b > tmp_c ) ) {
              patch.setDepth( 0, p, g_infiniteDepth );
              patch.setDepth0PccIdx( p, g_infinitenumber );
            }
          }
        }
      }

      if ( EOMSingleLayerMode ) {
        int16_t patch_surfaceThickness =
            useSurfaceSeparation ? getPatchSurfaceThickness(
                                       points, patch, patchIndex, frame_pcc_color, connectedComponent, surfaceThickness,
                                       patch.getProjectionMode(), bIsAdditionalProjectionPlane, geometryBitDepth3D )
                                 : surfaceThickness;
        if ( patch_surfaceThickness > 0 ) {
          for ( const auto i : connectedComponent ) {
            PCCPoint3D pointTmp = points[i];
            if ( bIsAdditionalProjectionPlane ) {
              auto& input = pointTmp;
              convert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, pointTmp );
            }
            const auto& point = pointTmp;
            const auto  d     = int16_t( round( point[patch.getNormalAxis()] ) );
            const auto  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
            const auto  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
            assert( u >= 0 && u < patch.getSizeU() );
            assert( v >= 0 && v < patch.getSizeV() );
            const size_t  p      = v * patch.getSizeU() + u;
            const int16_t depth0 = patch.getDepth( 0 )[p];
            const int16_t diff   = std::abs( depth0 - d );
            int16_t       eomThickness =
                useSurfaceSeparation ? ( patch_surfaceThickness + EOMFixBitCount - surfaceThickness ) : EOMFixBitCount;
            if ( depth0 < g_infiniteDepth && ( projectionDirectionType * ( d - depth0 ) ) > 0 &&
                 diff <= int16_t( eomThickness ) ) {
              uint16_t deltaD = diff;
              patch.setDepthEOM( p, patch.getDepthEOM( p ) | 1 << ( deltaD - 1 ) );
            }
          }
        }
      } else {
        // compute d1 map
        patch.setDepth( 1, patch.getDepth( 0 ) );
        int16_t patch_surfaceThickness =
            useSurfaceSeparation ? getPatchSurfaceThickness(
                                       points, patch, patchIndex, frame_pcc_color, connectedComponent, surfaceThickness,
                                       patch.getProjectionMode(), bIsAdditionalProjectionPlane, geometryBitDepth3D )
                                 : surfaceThickness;

        if ( patch_surfaceThickness > 0 ) {
          for ( const auto i : connectedComponent ) {
            PCCPoint3D pointTmp = points[i];
            if ( bIsAdditionalProjectionPlane ) {
              auto& input = pointTmp;
              convert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, pointTmp );
            }
            const auto& point = pointTmp;
            const auto  d     = int16_t( round( point[patch.getNormalAxis()] ) );
            const auto  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
            const auto  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
            assert( u >= 0 && u < patch.getSizeU() );
            assert( v >= 0 && v < patch.getSizeV() );
            const size_t  p      = v * patch.getSizeU() + u;
            const int16_t depth0 = patch.getDepth( 0 )[p];
            const int16_t deltaD = projectionDirectionType * ( d - depth0 );
            if ( !( depth0 < g_infiniteDepth ) ) { continue; }
            bool bsimilar = colorSimilarity( frame_pcc_color[i], frame_pcc_color[patch.getDepth0PccIdx()[p]], 128 );
            if ( depth0 < g_infiniteDepth && ( deltaD ) <= int16_t( patch_surfaceThickness ) && deltaD >= 0 &&
                 bsimilar ) {
              if ( projectionDirectionType * ( d - patch.getDepth( 1 )[p] ) > 0 ) { patch.setDepth( 1, p, d ); }
              if ( useEnhancedOccupancyMapCode ) {
                patch.setDepthEOM( p, patch.getDepthEOM( p ) | 1 << ( deltaD - 1 ) );
              }
            }
            if ( ( patch.getProjectionMode() == 0 && ( patch.getDepth( 1 )[p] < patch.getDepth( 0 )[p] ) ) ||
                 ( patch.getProjectionMode() == 1 && ( patch.getDepth( 1 )[p] > patch.getDepth( 0 )[p] ) ) ) {
              std::cout << "ERROR: d1(" << patch.getDepth( 1 )[p] << ") and d0(" << patch.getDepth( 0 )[p]
                        << ") for projection mode[" << patch.getProjectionMode() << "]" << std::endl;
            }
          }
        }
      }
      patch.setSizeD( 0 );
      PCCPointSet3 rec;
      rec.resize( 0 );
      std::vector<size_t> pointCount;
      pointCount.resize( 3 );
      resampledPointcloud( pointCount, resampled, resampledPatchPartition, patch, patchIndex,
                           params.mapCountMinus1_ > 0, surfaceThickness, EOMFixBitCount, bIsAdditionalProjectionPlane,
                           useEnhancedOccupancyMapCode, geometryBitDepth3D, createSubPointCloud, rec );

      d0CountPerPatch  = pointCount[0];
      d1CountPerPatch  = pointCount[1];
      eomCountPerPatch = pointCount[2];

      // note: patch.getSizeD() cannot generate maximum depth(e.g. getSizeD=255, quantDD=3, quantDD needs to be limitted
      // to satisfy the bitcount) max : (1<<std::min(geometryBitDepth3D, geometryBitDepth2D))
      patch.setSizeDPixel( patch.getSizeD() );
      patch.setSizeD(
          std::min( ( size_t )( 1 << std::min( geometryBitDepth3D, geometryBitDepth2D ) ) - 1, patch.getSizeD() ) );
      size_t bitdepthD  = std::min( geometryBitDepth3D, geometryBitDepth2D ) - std::log2( minLevel );
      size_t maxDDplus1 = 1 << bitdepthD;  // e.g. 4
      size_t quantDD    = patch.getSizeD() == 0 ? 0 : ( ( patch.getSizeD() - 1 ) / minLevel + 1 );
      quantDD           = std::min( quantDD, maxDDplus1 - 1 );          // 1,2,3,3
      patch.setSizeD( quantDD == 0 ? 0 : ( quantDD * minLevel - 1 ) );  // 63, 127, 191, 191

      if ( createSubPointCloud ) {
        PCCPointSet3 testSrc;
        PCCPointSet3 testRec;
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
        float distPAB;
        float distPBA;
        float distYAB;
        float distYBA;
        float distUAB;
        float distUBA;
        float distVAB;
        float distVBA;
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
          if ( dist2 <= maxAllowedDist2RawPointsSelection ) {
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
      if ( EOMSingleLayerMode ) { d1CountPerPatch = 0; }
      patch.setEOMandD1Count( eomCountPerPatch );
      if ( useEnhancedOccupancyMapCode ) { patch.setEOMCount( eomCountPerPatch - d1CountPerPatch ); }
      patch.setD0Count( d0CountPerPatch );
      numberOfEOM += ( eomCountPerPatch - d1CountPerPatch );
      numD0Points += d0CountPerPatch;
      numD1Points += d1CountPerPatch;
      if ( useEnhancedOccupancyMapCode ) { numEOMOnlyPoints += ( eomCountPerPatch - d1CountPerPatch ); }
      std::cout << "\t\t Patch " << patchIndex << " ->(d1,u1,v1)=( " << patch.getD1() << " , " << patch.getU1() << " , "
                << patch.getV1() << " )(dd,du,dv)=( " << patch.getSizeD() << " , " << patch.getSizeU() << " , "
                << patch.getSizeV() << " ),Normal: " << size_t( patch.getNormalAxis() )
                << " Direction: " << patch.getProjectionMode() << " EOM: " << patch.getEOMCount() << std::endl;
    }
    PCCKdTree kdtreeResampled( resampled );
    rawPoints.resize( 0 );
    for ( size_t i = 0; i < pointCount; ++i ) {
      kdtreeResampled.search( points[i], 1, result );
      const double dist2   = result.dist( 0 );
      rawPointsDistance[i] = dist2;
      if ( dist2 > maxAllowedDist2RawPointsSelection ) { rawPoints.push_back( i ); }
    }
    if ( enablePointCloudPartitioning ) {
      // update rawPointsChunks using rawPoints
      std::set<size_t>                 rawPointsSet( rawPoints.begin(), rawPoints.end() );
      std::vector<std::vector<size_t>> rawPointsChunksNextIter( numChunks );
      for ( size_t chunkIndex = 0; chunkIndex < numChunks; ++chunkIndex ) {
        for ( const auto i : rawPointsChunks[chunkIndex] ) {
          if ( rawPointsSet.find( pointsIndexChunks[chunkIndex][i] ) != rawPointsSet.end() ) {
            rawPointsChunksNextIter[chunkIndex].push_back( i );
            rawPointsDistanceChunks[chunkIndex][i] = rawPointsDistance[pointsIndexChunks[chunkIndex][i]];
          }
        }
        rawPointsChunks[chunkIndex].resize( 0 );
        rawPointsChunks[chunkIndex] = rawPointsChunksNextIter[chunkIndex];
      }
    }
    std::cout << " # patches " << patches.size() << std::endl;
    std::cout << " # resampled " << resampled.getPointCount() << std::endl;
    std::cout << " # raw points " << rawPoints.size() << std::endl;
    if ( useEnhancedOccupancyMapCode ) { std::cout << " # EOM points " << numberOfEOM << std::endl; }
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
#if defined( ENABLE_TBB )
    tbb::task_arena limited( static_cast<int>( nbThread_ ) );
    limited.execute( [&] {
      tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) {
#else
    for ( size_t i = 0; i < pointCount; i++ ) {
#endif
        auto& scoreSmooth = scoresSmooth[i];
        std::fill( scoreSmooth.begin(), scoreSmooth.end(), 0 );
        for ( auto& neighbor : adj[i] ) { ++scoreSmooth[partition[neighbor]]; }
#if defined( ENABLE_TBB )
      } );
    } );
#else
    }
#endif
#if defined( ENABLE_TBB )
    limited.execute( [&] {
      tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) {
#else
    for ( size_t i = 0; i < pointCount; i++ ) {
#endif
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
#if defined( ENABLE_TBB )
      } );
    } );
#else
  }
#endif
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
  for ( size_t i = 0; i < pointCount; ++i ) {
    const auto& pos = pointCloud[i];
    geoMax          = ( std::max )( geoMax, pos[0] );
    geoMax          = ( std::max )( geoMax, pos[1] );
    geoMax          = ( std::max )( geoMax, pos[2] );
  }
  size_t geoRange = 1;
  for ( size_t i = geoMax - 1; i != 0u; i >>= 1, geoRange <<= 1 ) { ; }
  size_t voxDimShift;
  size_t gridDimShift;
  size_t i;
  for ( voxDimShift = 0, i = voxDim; i > 1; ++voxDimShift, i >>= 1 ) { ; }
  const size_t gridDim = geoRange >> voxDimShift;
  for ( gridDimShift = 0, i = gridDim; i > 1; ++gridDimShift, i >>= 1 ) { ; }
  const size_t gridDimShiftSqr = gridDimShift << 1;
  const size_t voxDimHalf      = voxDim >> 1;

  auto subToInd = [&]( size_t x, size_t y, size_t z ) { return x + ( y << gridDimShift ) + ( z << gridDimShiftSqr ); };

  PCCPointSet3                                                        gridCenters;
  std::unordered_map<size_t, std::unique_ptr<PointIndicesOfGridCell>> gridWithPointIndices;
  std::unordered_map<size_t, std::unique_ptr<AttributeOfGridCell>>    gridWithAttributes;

  const uint32_t uiMaxNumPointsInOneVox = voxDim * voxDim * voxDim;
  for ( size_t i = 0; i < pointCount; ++i ) {
    const auto&  pos = pointCloud[i];
    const size_t x0  = ( ( static_cast<size_t>( pos[0] ) + voxDimHalf ) ) >> voxDimShift;
    const size_t y0  = ( ( static_cast<size_t>( pos[1] ) + voxDimHalf ) ) >> voxDimShift;
    const size_t z0  = ( ( static_cast<size_t>( pos[2] ) + voxDimHalf ) ) >> voxDimShift;
    size_t       p   = subToInd( x0, y0, z0 );
    if ( gridWithPointIndices.count( p ) == 0u ) {
      gridCenters.addPoint( PCCVector3D( x0, y0, z0 ) );
      gridWithPointIndices.emplace( p, std::make_unique<PointIndicesOfGridCell>( uiMaxNumPointsInOneVox ) );
      gridWithAttributes.emplace( p, std::make_unique<AttributeOfGridCell>( orientationCount ) );
    }
    gridWithPointIndices[p]->addPointIndex( (uint32_t)i );
  }

  const uint64_t uiTotalNumOfVoxs = gridCenters.getPointCount();

  // pre-processing steps [m56635]
  std::vector<PointIndicesOfGridCell*> pointIndicesOfVox;
  pointIndicesOfVox.reserve( uiTotalNumOfVoxs );

  std::vector<AttributeOfGridCell*> attributeOfVox;
  attributeOfVox.reserve( uiTotalNumOfVoxs );

  for ( size_t i = 0; i < uiTotalNumOfVoxs; ++i ) {
    auto& p = gridCenters[i];

    PointIndicesOfGridCell* pPI   = gridWithPointIndices[subToInd( p[0], p[1], p[2] )].get();
    AttributeOfGridCell*    pAttr = gridWithAttributes[subToInd( p[0], p[1], p[2] )].get();

    pAttr->setNumOfTotalPoints( pPI->getPointCount() );

    // 1st voxel classification [m56635]
    pAttr->updateScores( *( pPI->getPointIndices() ), partition );

    pointIndicesOfVox.push_back( pPI );
    attributeOfVox.push_back( pAttr );
  }

  // a step for searching adjacents voxels of each voxel within the voxSearchRadius
  PCCKdTree                          kdtree( gridCenters );
  const size_t                       voxSearchRadius  = searchRadius >> voxDimShift;
  const size_t                       maxNeighborCount = ( std::numeric_limits<int16_t>::max )();
  std::vector<std::vector<uint32_t>> adj( gridCenters.getPointCount() );

  computeAdjacencyInfoInRadius( gridCenters, kdtree, adj, maxNeighborCount, voxSearchRadius );

  // candidates for the indirect edge voxels from [m56635]
  std::vector<std::vector<uint32_t>> adjDEV( uiTotalNumOfVoxs );
  const size_t                       idvSearchRange = ( voxDim >= 4 ) ? 1 : 2;

  // pre-processing steps from m55143
  std::vector<double> weights;
  weights.reserve( uiTotalNumOfVoxs );

  // for each cell of the grid
  for ( size_t i = 0; i < uiTotalNumOfVoxs; ++i ) {
    auto& p = gridCenters[i];
    adjDEV[i].reserve( 128 );

    size_t nnPointCount  = 0;
    auto&  currentAdjOfI = adj[i];
    auto   iter          = currentAdjOfI.begin();
    for ( ; iter != currentAdjOfI.end(); ++iter ) {
      // for the 2nd voxel classification [m56635]
      auto&  q    = gridCenters[*iter];
      size_t xAbs = abs( p[0] - q[0] );
      size_t yAbs = abs( p[1] - q[1] );
      size_t zAbs = abs( p[2] - q[2] );
      if ( xAbs <= idvSearchRange && yAbs <= idvSearchRange && zAbs <= idvSearchRange ) {
        adjDEV[i].push_back( *iter );
      }
      nnPointCount += pointIndicesOfVox[*iter]->getPointCount();
      if ( nnPointCount >= maxNNCount ) { break; }
    }

    // pre-computing weights from lambda and the total number of nearest neighbors
    weights.push_back( lambda / nnPointCount );

    // removing points from the adjacent list if there is more than maxNNCount
    if ( iter != currentAdjOfI.end() ) { currentAdjOfI.erase( iter + 1, currentAdjOfI.end() ); }
  }

  std::vector<double> scores;
  scores.resize( orientationCount, 0.0 );

  ScoresVector_t scoreSmooth( orientationCount, 0 );

  size_t iter = 0;
  do {
    for ( size_t i = 0; i < uiTotalNumOfVoxs; ++i ) {
      // if the current voxel belongs to N-EV(No edge-voxel), then refining steps are skipped. [m56635]
      uint8_t edgeOfI = attributeOfVox[i]->getEdge();
      if ( edgeOfI == NO_EDGE ) { continue; }

      std::fill( scoreSmooth.begin(), scoreSmooth.end(), 0 );

      auto& currentAdjOfI = adj[i];
      for ( const auto& j : currentAdjOfI ) {
        ScoresVector_t* scoreSmoothOfAdj = attributeOfVox[j]->getScoreSmooth();
        for ( size_t k = 0; k < orientationCount; ++k ) { scoreSmooth[k] += ( *scoreSmoothOfAdj )[k]; }
      }

      // 2nd voxel classification (indirect edge-voxel)  [m56635]
      const auto& maxEleOfScoreSmooth = std::max_element( scoreSmooth.begin(), scoreSmooth.end() );
      size_t      ppiOfScoreSmooth    = std::distance( scoreSmooth.begin(), maxEleOfScoreSmooth );

      for ( auto& j : adjDEV[i] ) {
        uint8_t edgeOfAdj = attributeOfVox[j]->getEdge();
        uint8_t ppi       = attributeOfVox[j]->getPPI();
        if ( edgeOfAdj == NO_EDGE && ppi != ppiOfScoreSmooth ) { attributeOfVox[j]->updateEdge( INDIRECT_EDGE ); }
      }  // for (auto& j : adjDEV[i])

      if ( edgeOfI != M_DIRECT_EDGE ) {  // S_DIRECT_EDGE or INDIRECT_EDGE
        size_t validNumOfScores = orientationCount - std::count( scoreSmooth.begin(), scoreSmooth.end(), 0 );
        size_t voxPPI           = attributeOfVox[i]->getPPI();

        if ( validNumOfScores == 1 && scoreSmooth[voxPPI] > 0 ) { continue; }
      }

      const auto& pI = *( pointIndicesOfVox[i]->getPointIndices() );

      // for each point in a grid cell of i
      for ( const auto& j : pI ) {
        const auto& normal = normalsGen.getNormal( j );
        for ( size_t k = 0; k < orientationCount; ++k ) {
          scores[k] = normal * orientations[k] + weights[i] * scoreSmooth[k];
        }
        const auto& result = std::max_element( scores.begin(), scores.end() );
        partition[j]       = std::distance( scores.begin(), result );
      }

      attributeOfVox[i]->setUpdatedFlag();
    }  // for (size_t i = 0; i < uiTotalNumOfVoxs; ++i)

    // restarts the values of score smooth by checking to which partition points now is part of
    for ( size_t i = 0; i < uiTotalNumOfVoxs; ++i ) {
      attributeOfVox[i]->updateScores( *( pointIndicesOfVox[i]->getPointIndices() ), partition );
    }
  } while ( ++iter < iterationCount );
}

float pcc::computeIOU( Rect a, Rect b ) {
  float iou              = 0.0F;
  Rect  intersec         = a & b;
  int   intersectionArea = intersec.area();
  int   unionArea        = a.area() + b.area() - intersectionArea;
  iou                    = static_cast<float>( intersectionArea ) / unionArea;
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
    patch.setIndex( idx++ );
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
      if ( !isComponentRemoved[i] ) { tmpConnectedComponent.push_back( connectedComponent[i] ); }
    }
    swap( connectedComponent, tmpConnectedComponent );
  }

  // try to join other CC
  std::vector<size_t> patchPartition;
  patchPartition.resize( points.getPointCount(), 0 );
  for ( size_t i = 0; i < connectedComponents.size(); ++i ) {
    auto& connectedComponent = connectedComponents[i];
    for ( const auto j : connectedComponent ) { patchPartition[j] = i + 1; }
  }

  std::vector<size_t> highGradientCCPartition;
  highGradientCCPartition.resize( highGradientConnectedComponents.size(), 0 );
  std::vector<size_t> highGradientPartition;
  highGradientPartition.resize( highGradientConnectedComponents.size(), 0 );
  for ( size_t i = 0; i < highGradientConnectedComponents.size(); ++i ) {
    auto&       highGradientConnectedComponent = highGradientConnectedComponents[i];
    PCCVector3D normalSum( 0.0F );
    for ( const auto j : highGradientConnectedComponent ) { normalSum += normalsGen.getNormal( j ); }

    double       bestScore    = 0.0;
    const size_t orgPartition = partition[highGradientConnectedComponent[0]];
    for ( size_t k = 0; k < orientationCount; ++k ) {
      if ( orgPartition < 6 ) {
        if ( k < 6 && ( k % 3 ) == ( orgPartition % 3 ) ) { continue; }
      } else {
        if ( orgPartition < 10 ) {
          if ( k >= 6 && k < 10 && ( k == orgPartition || k + 2 == orgPartition || k == orgPartition + 2 ) ) {
            continue;
          }
        } else if ( orgPartition < 14 ) {
          if ( k >= 10 && k < 14 && ( k == orgPartition || k + 2 == orgPartition || k == orgPartition + 2 ) ) {
            continue;
          }
        } else {
          if ( k >= 14 && ( k == orgPartition || k + 2 == orgPartition || k == orgPartition + 2 ) ) { continue; }
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
      if ( highGradientCCPartition[i] != 0 ) { break; }
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
  size_t clusterIndex          = partition[connectedComponent[0]];
  bIsAdditionalProjectionPlane = ( clusterIndex > 5 );  // false;
  if ( bIsAdditionalProjectionPlane && ( additionalProjectionAxis == 2 ) ) clusterIndex += 4;
  if ( bIsAdditionalProjectionPlane && ( additionalProjectionAxis == 3 ) ) clusterIndex += 8;
  patch.setViewId( clusterIndex );
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

  const int16_t g_infiniteDepth = ( std::numeric_limits<int16_t>::max )();

  patch.setSizeU( 1 + size_t( round( boundingBox.max_[patch.getTangentAxis()] ) -
                              floor( boundingBox.min_[patch.getTangentAxis()] ) ) );
  patch.setSizeV( 1 + size_t( round( boundingBox.max_[patch.getBitangentAxis()] ) -
                              floor( boundingBox.min_[patch.getBitangentAxis()] ) ) );
  patch.setU1( size_t( boundingBox.min_[patch.getTangentAxis()] ) );
  patch.setV1( size_t( boundingBox.min_[patch.getBitangentAxis()] ) );
  patch.allocDepth( 0, patch.getSizeU() * patch.getSizeV(), g_infiniteDepth );

  for ( const auto i : connectedComponent ) {
    PCCPoint3D pointTmp = points[i];
    if ( bIsAdditionalProjectionPlane ) {
      auto& input = pointTmp;
      convert( patch.getAxisOfAdditionalPlane(), geometryBitDepth3D, input, pointTmp );
    }
    const auto& point = pointTmp;
    const auto  d     = int16_t( round( point[patch.getNormalAxis()] ) );
    const auto  u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
    const auto  v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
    assert( u >= 0 && u < patch.getSizeU() );
    assert( v >= 0 && v < patch.getSizeV() );
    const size_t p = v * patch.getSizeU() + u;
    if ( patch.getProjectionMode() == 0 ) {  // min
      if ( patch.getDepth( 0 )[p] > d ) { patch.setDepth( 0, p, d ); }
    } else {  // max
      if ( patch.getDepth( 0 )[p] == g_infiniteDepth ) {
        patch.setDepth( 0, p, d );
      } else {
        if ( patch.getDepth( 0 )[p] < d ) { patch.setDepth( 0, p, d ); }
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
  const double        normalThreshold = 0.577;
  const int16_t       g_infiniteDepth = ( std::numeric_limits<int16_t>::max )();
  std::vector<double> Gmag;
  Gmag.resize( patch.getSizeU() * patch.getSizeV(), 0 );
  int patchWidth  = int( patch.getSizeU() );
  int patchHeight = int( patch.getSizeV() );
  for ( int v = 0; v < patchHeight; ++v ) {
    for ( int u = 0; u < patchWidth; ++u ) {
      const size_t  p      = v * patchWidth + u;
      const int16_t depth0 = patch.getDepth( 0 )[p];
      if ( depth0 < g_infiniteDepth ) {
        int16_t Gx = 0;
        int16_t Gy = 0;
        int16_t depth[8];
        depth[0] = ( u != 0 && v != 0 && patch.getDepth( 0 )[( v - 1 ) * patchWidth + u - 1] < g_infiniteDepth )
                       ? patch.getDepth( 0 )[( v - 1 ) * patchWidth + u - 1]
                       : depth0;
        depth[1] = ( v != 0 && patch.getDepth( 0 )[( v - 1 ) * patchWidth + u] < g_infiniteDepth )
                       ? patch.getDepth( 0 )[( v - 1 ) * patchWidth + u]
                       : depth0;
        depth[2] =
            ( u != patchWidth - 1 && v != 0 && patch.getDepth( 0 )[( v - 1 ) * patchWidth + u + 1] < g_infiniteDepth )
                ? patch.getDepth( 0 )[( v - 1 ) * patchWidth + u + 1]
                : depth0;
        depth[3] = ( u != 0 && patch.getDepth( 0 )[v * patchWidth + u - 1] < g_infiniteDepth )
                       ? patch.getDepth( 0 )[v * patchWidth + u - 1]
                       : depth0;
        depth[4] = ( u != patchWidth - 1 && patch.getDepth( 0 )[v * patchWidth + u + 1] < g_infiniteDepth )
                       ? patch.getDepth( 0 )[v * patchWidth + u + 1]
                       : depth0;
        depth[5] =
            ( u != 0 && v != patchHeight - 1 && patch.getDepth( 0 )[( v + 1 ) * patchWidth + u - 1] < g_infiniteDepth )
                ? patch.getDepth( 0 )[( v + 1 ) * patchWidth + u - 1]
                : depth0;
        depth[6] = ( v != patchHeight - 1 && patch.getDepth( 0 )[( v + 1 ) * patchWidth + u] < g_infiniteDepth )
                       ? patch.getDepth( 0 )[( v + 1 ) * patchWidth + u]
                       : depth0;
        depth[7] = ( u != patchWidth - 1 && v != patchHeight - 1 &&
                     patch.getDepth( 0 )[( v + 1 ) * patchWidth + u + 1] < g_infiniteDepth )
                       ? patch.getDepth( 0 )[( v + 1 ) * patchWidth + u + 1]
                       : depth0;

        // Sobel operator for x-axis [ 1  0 -1; 2  0 -2; 1  0 -1 ]
        Gx += depth[0] - depth[2] + 2 * depth[3] - 2 * depth[4] + depth[5] - depth[7];
        // Sobel operator for y-axis [ 1  2  1; 0  0  0; -1 -2 -1 ]
        Gy += depth[0] + 2 * depth[1] + depth[2] - depth[5] - 2 * depth[6] - depth[7];
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
          if ( u > 0 && highGradientMap[p - 1] ) { cnt++; }
          if ( u < patch.getSizeU() - 1 && highGradientMap[p + 1] ) { cnt++; }
          if ( v > 0 && highGradientMap[p - patch.getSizeU()] ) { cnt++; }
          if ( v < patch.getSizeV() - 1 && highGradientMap[p + patch.getSizeU()] ) { cnt++; }
          if ( cnt >= 2 && Gmag[p] > minGradient / 2.0 ) { tmpHighGradientMap[p] = true; }
        }
      }
    }
    highGradientMap = tmpHighGradientMap;
  }

  // determine candidate points to be separated: points close to the D0 layer,
  // or the normal does not point to the
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
    const auto&  point = pointTmp;
    const auto   d     = int16_t( round( point[patch.getNormalAxis()] ) );
    const auto   u     = size_t( round( point[patch.getTangentAxis()] - patch.getU1() ) );
    const auto   v     = size_t( round( point[patch.getBitangentAxis()] - patch.getV1() ) );
    const size_t p     = v * patch.getSizeU() + u;
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
      if ( clusterIndex == orgPartitionIdx ) { continue; }
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
