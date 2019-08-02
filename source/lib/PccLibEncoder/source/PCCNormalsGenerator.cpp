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
#include "tbb/tbb.h"
#include "PCCNormalsGenerator.h"

using namespace pcc;

void PCCNormalsGenerator3::init( const size_t pointCount, const PCCNormalsGenerator3Parameters& params ) {
  normals_.resize( pointCount );
  if ( params.storeNumberOfNearestNeighborsInNormalEstimation_ ) {
    numberOfNearestNeighborsInNormalEstimation_.resize( pointCount );
  } else {
    numberOfNearestNeighborsInNormalEstimation_.resize( 0 );
  }
  if ( params.storeEigenvalues_ ) {
    eigenvalues_.resize( pointCount );
  } else {
    eigenvalues_.resize( 0 );
  }
  if ( params.storeCentroids_ ) {
    barycenters_.resize( pointCount );
  } else {
    barycenters_.resize( 0 );
  }
}
void PCCNormalsGenerator3::compute( const PCCPointSet3&                   pointCloud,
                                    const PCCKdTree&                      kdtree,
                                    const PCCNormalsGenerator3Parameters& params,
                                    const size_t                          nbThread ) {
  nbThread_ = nbThread;
  init( pointCloud.getPointCount(), params );
  computeNormals( pointCloud, kdtree, params );
  if ( params.numberOfIterationsInNormalSmoothing_ ) { smoothNormals( pointCloud, kdtree, params ); }
  orientNormals( pointCloud, kdtree, params );
}
void PCCNormalsGenerator3::computeNormal( const size_t                          index,
                                          const PCCPointSet3&                   pointCloud,
                                          const PCCKdTree&                      kdtree,
                                          const PCCNormalsGenerator3Parameters& params,
                                          PCCNNResult&                          nNResult ) {
  PCCVector3D bary( pointCloud[index][0], pointCloud[index][1], pointCloud[index][2] ), normal( 0.0 ), eigenval( 0.0 );
  PCCMatrix3D covMat, Q, D;
  kdtree.search( pointCloud[index], params.numberOfNearestNeighborsInNormalEstimation_, nNResult );
  if ( nNResult.count() > 1 ) {
    bary = 0.0;
    for ( size_t i = 0; i < nNResult.count(); ++i ) { bary += pointCloud[nNResult.indices( i )]; }
    bary /= double( nNResult.count() );
    covMat = 0.0;
    PCCVector3D pt;
    for ( size_t i = 0; i < nNResult.count(); ++i ) {
      pt = pointCloud[nNResult.indices( i )] - bary;
      covMat[0][0] += pt[0] * pt[0];
      covMat[1][1] += pt[1] * pt[1];
      covMat[2][2] += pt[2] * pt[2];
      covMat[0][1] += pt[0] * pt[1];
      covMat[0][2] += pt[0] * pt[2];
      covMat[1][2] += pt[1] * pt[2];
    }
    covMat[1][0] = covMat[0][1];
    covMat[2][0] = covMat[0][2];
    covMat[2][1] = covMat[1][2];
    covMat /= ( nNResult.count() - 1.0 );

    PCCDiagonalize( covMat, Q, D );

    D[0][0] = fabs( D[0][0] );
    D[1][1] = fabs( D[1][1] );
    D[2][2] = fabs( D[2][2] );

    if ( D[0][0] < D[1][1] && D[0][0] < D[2][2] ) {
      normal[0]   = Q[0][0];
      normal[1]   = Q[1][0];
      normal[2]   = Q[2][0];
      eigenval[0] = D[0][0];
      if ( D[1][1] < D[2][2] ) {
        eigenval[1] = D[1][1];
        eigenval[2] = D[2][2];
      } else {
        eigenval[2] = D[1][1];
        eigenval[1] = D[2][2];
      }
    } else if ( D[1][1] < D[2][2] ) {
      normal[0]   = Q[0][1];
      normal[1]   = Q[1][1];
      normal[2]   = Q[2][1];
      eigenval[0] = D[1][1];
      if ( D[0][0] < D[2][2] ) {
        eigenval[1] = D[0][0];
        eigenval[2] = D[2][2];
      } else {
        eigenval[2] = D[0][0];
        eigenval[1] = D[2][2];
      }
    } else {
      normal[0]   = Q[0][2];
      normal[1]   = Q[1][2];
      normal[2]   = Q[2][2];
      eigenval[0] = D[2][2];
      if ( D[0][0] < D[1][1] ) {
        eigenval[1] = D[0][0];
        eigenval[2] = D[1][1];
      } else {
        eigenval[2] = D[0][0];
        eigenval[1] = D[1][1];
      }
    }
  }
  if ( normal * ( params.viewPoint_ - pointCloud[index] ) < 0.0 ) {
    normals_[index] = -normal;
  } else {
    normals_[index] = normal;
  }
  if ( params.storeEigenvalues_ ) { eigenvalues_[index] = eigenval; }
  if ( params.storeCentroids_ ) { barycenters_[index] = bary; }
  if ( params.storeNumberOfNearestNeighborsInNormalEstimation_ ) {
    numberOfNearestNeighborsInNormalEstimation_[index] = uint32_t( nNResult.count() );
  }
}
void PCCNormalsGenerator3::computeNormals( const PCCPointSet3&                   pointCloud,
                                           const PCCKdTree&                      kdtree,
                                           const PCCNormalsGenerator3Parameters& params ) {
  const size_t pointCount = pointCloud.getPointCount();
  normals_.resize( pointCount );
  std::vector<size_t> subRanges;
  const size_t        chunckCount = 64;
  PCCDivideRange( 0, pointCount, chunckCount, subRanges );
  tbb::task_arena limited( (int)nbThread_ );
  limited.execute( [&] {
    tbb::parallel_for( size_t( 0 ), subRanges.size() - 1, [&]( const size_t i ) {
      const size_t start = subRanges[i];
      const size_t end   = subRanges[i + 1];
      PCCNNResult  nNResult;
      for ( size_t ptIndex = start; ptIndex < end; ++ptIndex ) {
        computeNormal( ptIndex, pointCloud, kdtree, params, nNResult );
      }
    } );
  } );
}
void PCCNormalsGenerator3::orientNormals( const PCCPointSet3&                   pointCloud,
                                          const PCCKdTree&                      kdtree,
                                          const PCCNormalsGenerator3Parameters& params ) {
  if ( params.orientationStrategy_ == PCC_NORMALS_GENERATOR_ORIENTATION_SPANNING_TREE ) {
    const size_t pointCount = pointCloud.getPointCount();
    PCCNNResult  nNResult;
    visited_.resize( pointCount );
    std::fill( visited_.begin(), visited_.end(), 0 );
    PCCNNQuery3 nNQuery  = {PCCPoint3D( 0.0 ), (float)params.radiusNormalOrientation_ * params.radiusNormalOrientation_,
                           params.numberOfNearestNeighborsInNormalOrientation_};
    PCCNNQuery3 nNQuery2 = {PCCPoint3D( 0.0 ), ( std::numeric_limits<float>::max )(),
                            params.numberOfNearestNeighborsInNormalOrientation_};
    size_t      processedPointCount = 0;
    for ( size_t ptIndex = 0; ptIndex < pointCount; ++ptIndex ) {
      if ( !visited_[ptIndex] ) {
        visited_[ptIndex] = 1;
        ++processedPointCount;
        size_t      numberOfNormals;
        PCCVector3D accumulatedNormals;
        addNeighbors( uint32_t( ptIndex ), pointCloud, kdtree, nNQuery2, nNResult, accumulatedNormals,
                      numberOfNormals );
        if ( !numberOfNormals ) {
          if ( ptIndex ) {
            accumulatedNormals = normals_[ptIndex - 1];
          } else {
            accumulatedNormals = ( params.viewPoint_ - pointCloud[ptIndex] );
          }
        }
        if ( normals_[ptIndex] * accumulatedNormals < 0.0 ) { normals_[ptIndex] = -normals_[ptIndex]; }
        while ( !edges_.empty() ) {
          PCCWeightedEdge edge = edges_.top();
          edges_.pop();
          uint32_t current = edge.end_;
          if ( !visited_[current] ) {
            visited_[current] = 1;
            ++processedPointCount;
            if ( normals_[edge.start_] * normals_[current] < 0.0 ) { normals_[current] = -normals_[current]; }
            addNeighbors( current, pointCloud, kdtree, nNQuery, nNResult, accumulatedNormals, numberOfNormals );
          }
        }
      }
    }
    size_t negNormalCount = 0;
    for ( size_t ptIndex = 0; ptIndex < pointCount; ++ptIndex ) {
      negNormalCount += ( normals_[ptIndex] * ( params.viewPoint_ - pointCloud[ptIndex] ) < 0.0 );
    }
    if ( negNormalCount > ( pointCount + 1 ) / 2 ) {
      for ( size_t ptIndex = 0; ptIndex < pointCount; ++ptIndex ) { normals_[ptIndex] = -normals_[ptIndex]; }
    }
  } else if ( params.orientationStrategy_ == PCC_NORMALS_GENERATOR_ORIENTATION_VIEW_POINT ) {
    const size_t    pointCount = pointCloud.getPointCount();
    tbb::task_arena limited( (int)nbThread_ );
    limited.execute( [&] {
      tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t ptIndex ) {
        if ( normals_[ptIndex] * ( params.viewPoint_ - pointCloud[ptIndex] ) < 0.0 ) {
          normals_[ptIndex] = -normals_[ptIndex];
        }
      } );
    } );
  }
}
void PCCNormalsGenerator3::addNeighbors( const uint32_t      current,
                                         const PCCPointSet3& pointCloud,
                                         const PCCKdTree&    kdtree,
                                         PCCNNQuery3&        nNQuery,
                                         PCCNNResult&        nNResult,
                                         PCCVector3D&        accumulatedNormals,
                                         size_t&             numberOfNormals ) {
  accumulatedNormals = 0.0;
  numberOfNormals    = 0;
  if ( nNQuery.radius > 32768.0 ) {
    kdtree.search( pointCloud[current], nNQuery.nearestNeighborCount, nNResult );
  } else {
    kdtree.searchRadius( pointCloud[current], nNQuery.nearestNeighborCount, nNQuery.radius, nNResult );
  }
  PCCWeightedEdge newEdge;
  uint32_t        index;
  for ( size_t i = 0; i < nNResult.count(); ++i ) {
    index = (uint32_t)nNResult.indices( i );
    if ( !visited_[index] ) {
      newEdge.weight_ = fabs( normals_[current] * normals_[index] );
      newEdge.end_    = index;
      newEdge.start_  = current;
      edges_.push( newEdge );
    } else if ( index != current ) {
      accumulatedNormals += normals_[index];
      ++numberOfNormals;
    }
  }
}
void PCCNormalsGenerator3::smoothNormals( const PCCPointSet3&                   pointCloud,
                                          const PCCKdTree&                      kdtree,
                                          const PCCNormalsGenerator3Parameters& params ) {
  const double        w2         = params.weightNormalSmoothing_;
  const double        w0         = ( 1 - w2 );
  const size_t        pointCount = pointCloud.getPointCount();
  PCCVector3D         n0, n1, n2;
  std::vector<size_t> subRanges;
  const size_t        chunckCount = 64;
  PCCDivideRange( 0, pointCount, chunckCount, subRanges );
  const double radius = params.radiusNormalSmoothing_ * params.radiusNormalSmoothing_;
  for ( size_t it = 0; it < params.numberOfIterationsInNormalSmoothing_; ++it ) {
    tbb::task_arena limited( (int)nbThread_ );
    limited.execute( [&] {
      tbb::parallel_for( size_t( 0 ), subRanges.size() - 1, [&]( const size_t i ) {
        const size_t start = subRanges[i];
        const size_t end   = subRanges[i + 1];
        PCCNNResult  result;
        for ( size_t ptIndex = start; ptIndex < end; ++ptIndex ) {
          kdtree.searchRadius( pointCloud[ptIndex], params.numberOfNearestNeighborsInNormalSmoothing_, radius, result );
          n0 = normals_[ptIndex];
          n1 = 0.0;
          for ( size_t i = 1; i < result.count(); ++i ) {
            n2 = normals_[result.indices( i )];
            if ( n0 * n2 < 0.0 ) {
              n1 -= n2;
            } else {
              n1 += n2;
            }
          }
          n1.normalize();
          n1 = w0 * n0 + w2 * n1;
          n1.normalize();
          normals_[ptIndex] = n1;
        }
      } );
    } );
  }
}
