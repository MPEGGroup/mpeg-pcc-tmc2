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
#include "PCCImage.h"
#if defined( ENABLE_TBB )
#include <tbb/tbb.h>
#endif

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
  if ( params.numberOfIterationsInNormalSmoothing_ != 0u ) { smoothNormals( pointCloud, kdtree, params ); }
  orientNormals( pointCloud, kdtree, params );
}
void PCCNormalsGenerator3::computeNormal( const size_t                          index,
                                          const PCCPointSet3&                   pointCloud,
                                          const PCCKdTree&                      kdtree,
                                          const PCCNormalsGenerator3Parameters& params,
                                          PCCNNResult&                          nNResult ) {
  PCCVector3D bary( pointCloud[index][0], pointCloud[index][1], pointCloud[index][2] );
  PCCVector3D normal( 0.0 );
  PCCVector3D eigenval( 0.0 );
  PCCMatrix3D covMat;
  PCCMatrix3D Q;
  PCCMatrix3D D;
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
#if defined( ENABLE_TBB )
  tbb::task_arena limited( static_cast<int>( nbThread_ ) );
  limited.execute( [&] {
    tbb::parallel_for( size_t( 0 ), subRanges.size() - 1, [&]( const size_t i ) {
#else
  for ( size_t i = 0; i < subRanges.size() - 1; i++ ) {
#endif
      const size_t start = subRanges[i];
      const size_t end   = subRanges[i + 1];
      PCCNNResult  nNResult;
      for ( size_t ptIndex = start; ptIndex < end; ++ptIndex ) {
        computeNormal( ptIndex, pointCloud, kdtree, params, nNResult );
      }
#if defined( ENABLE_TBB )
    } );
  } );
#else
  }
#endif
}
void PCCNormalsGenerator3::orientNormals( const PCCPointSet3&                   pointCloud,
                                          const PCCKdTree&                      kdtree,
                                          const PCCNormalsGenerator3Parameters& params ) {
// save mesh with normals
#ifdef DEBUG_NORMAL
  PCCPointSet3 saveNormal = pointCloud;
  saveNormal.addNormals();
  for ( int i = 0; i < saveNormal.getPointCount(); i++ ) {
    saveNormal.setNormal( i, PCCNormal3D( normals_[i][0], normals_[i][1], normals_[i][2] ) );
  }
  saveNormal.write( "normal_no_orientation.ply" );
#endif
  if ( params.orientationStrategy_ == PCC_NORMALS_GENERATOR_ORIENTATION_SPANNING_TREE ) {
    const size_t pointCount = pointCloud.getPointCount();
    PCCNNResult  nNResult;
    visited_.resize( pointCount );
    std::fill( visited_.begin(), visited_.end(), 0 );
    PCCNNQuery3 nNQuery  = {PCCPoint3D( 0.0 ),
                           static_cast<float>( params.radiusNormalOrientation_ ) * params.radiusNormalOrientation_,
                           params.numberOfNearestNeighborsInNormalOrientation_};
    PCCNNQuery3 nNQuery2 = {PCCPoint3D( 0.0 ), ( std::numeric_limits<float>::max )(),
                            params.numberOfNearestNeighborsInNormalOrientation_};
    for ( size_t ptIndex = 0; ptIndex < pointCount; ++ptIndex ) {
      if ( visited_[ptIndex] == 0u ) {
        visited_[ptIndex] = 1;
        size_t      numberOfNormals;
        PCCVector3D accumulatedNormals;
        addNeighbors( uint32_t( ptIndex ), pointCloud, kdtree, nNQuery2, nNResult, accumulatedNormals,
                      numberOfNormals );
        if ( numberOfNormals == 0u ) {
          if ( ptIndex != 0u ) {
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
          if ( visited_[current] == 0u ) {
            visited_[current] = 1;
            if ( normals_[edge.start_] * normals_[current] < 0.0 ) { normals_[current] = -normals_[current]; }
            addNeighbors( current, pointCloud, kdtree, nNQuery, nNResult, accumulatedNormals, numberOfNormals );
          }
        }
      }
    }
    size_t negNormalCount = 0;
    for ( size_t ptIndex = 0; ptIndex < pointCount; ++ptIndex ) {
      negNormalCount +=
          static_cast<unsigned long long>( normals_[ptIndex] * ( params.viewPoint_ - pointCloud[ptIndex] ) < 0.0 );
    }
    if ( negNormalCount > ( pointCount + 1 ) / 2 ) {
      for ( size_t ptIndex = 0; ptIndex < pointCount; ++ptIndex ) { normals_[ptIndex] = -normals_[ptIndex]; }
    }
#ifdef DEBUG_NORMAL
    // save mesh
    PCCPointSet3 saveNormal4 = pointCloud;
    saveNormal4.addNormals();
    for ( int i = 0; i < saveNormal4.getPointCount(); i++ ) {
      saveNormal4.setNormal( i, PCCNormal3D( normals_[i][0], normals_[i][1], normals_[i][2] ) );
    }
    saveNormal4.write( "normal_orientation_spanning_tree_final.ply" );
#endif
  } else if ( params.orientationStrategy_ == PCC_NORMALS_GENERATOR_ORIENTATION_VIEW_POINT ) {
    const size_t    pointCount = pointCloud.getPointCount();
#if defined( ENABLE_TBB )
    tbb::task_arena limited( static_cast<int>( nbThread_ ) );
    limited.execute( [&] {
      tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t ptIndex ) {
#else
  for ( size_t ptIndex = 0; ptIndex < pointCount; ptIndex++ ) {
#endif
        if ( normals_[ptIndex] * ( params.viewPoint_ - pointCloud[ptIndex] ) < 0.0 ) {
          normals_[ptIndex] = -normals_[ptIndex];
        }
#if defined( ENABLE_TBB )
    } );
  } );
#else
  }
#endif
#ifdef DEBUG_NORMAL
    // save mesh
    PCCPointSet3 saveNormal4 = pointCloud;
    saveNormal4.addNormals();
    for ( int i = 0; i < saveNormal4.getPointCount(); i++ ) {
      saveNormal4.setNormal( i, PCCNormal3D( normals_[i][0], normals_[i][1], normals_[i][2] ) );
    }
    saveNormal4.write( "normal_orientation_view_point_final.ply" );
#endif
  } else if ( params.orientationStrategy_ == PCC_NORMALS_GENERATOR_ORIENTATION_CUBEMAP_PROJECTION ) {
    const size_t pointCount = pointCloud.getPointCount();
    PCCNNResult  nNResult;
    visited_.resize( pointCount );
    std::fill( visited_.begin(), visited_.end(), 0 );
    // let's project the points to the side of the cubemap, and determine the orientation based on that
    PCCBox3D bb;
    bb.min_ = PCCVector3D( ( std::numeric_limits<int>::max )() );
    bb.max_ = PCCVector3D( -( std::numeric_limits<int>::max )() );
    for ( int i = 0; i < pointCloud.getPointCount(); i++ ) {
      bb.add( PCCVector3D( pointCloud[i][0], pointCloud[i][1], pointCloud[i][2] ) );
    }
    PCCVector3D orientations6_normal[6] = {
        PCCVector3D( -1, 0, 0 ), PCCVector3D( 0, -1, 0 ), PCCVector3D( 0, 0, -1 ),
        PCCVector3D( 1, 0, 0 ),  PCCVector3D( 0, 1, 0 ),  PCCVector3D( 0, 0, 1 ),
    };
    double planeWidth[3]  = {( bb.max_[1] - bb.min_[1] ) + 1, ( bb.max_[2] - bb.min_[2] ) + 1,
                            ( bb.max_[0] - bb.min_[0] ) + 1};
    double planeHeight[3] = {( bb.max_[2] - bb.min_[2] ) + 1, ( bb.max_[0] - bb.min_[0] ) + 1,
                             ( bb.max_[1] - bb.min_[1] ) + 1};
    // allocating projection planes
    std::vector<std::vector<int>> projection;
#ifdef DEBUG_NORMAL
    std::vector<PCCImage<uint8_t, 3>> projectionImage;
    std::vector<PCCImage<uint8_t, 3>> projectionDepth;
    std::vector<PCCImage<uint8_t, 3>> projectionDirection;
#endif
    std::vector<std::vector<int>> projectionIndex;
    projection.resize( 6 );
    projectionIndex.resize( 6 );
#ifdef DEBUG_NORMAL
    projectionImage.resize( 6 );
    projectionDepth.resize( 6 );
    projectionDirection.resize( 6 );
#endif
    for ( int j = 0; j < 3; j++ ) {
      projection[j].resize( planeWidth[j] * planeHeight[j], bb.max_[j] + 1 );
      projectionIndex[j].resize( planeWidth[j] * planeHeight[j], -1 );
#ifdef DEBUG_NORMAL
      projectionImage[j].resize( planeWidth[j], planeHeight[j], PCCCOLORFORMAT::YUV444 );
      projectionImage[j].set( 0 );
      projectionDepth[j].resize( planeWidth[j], planeHeight[j], PCCCOLORFORMAT::YUV444 );
      projectionDepth[j].set( 0 );
      projectionDirection[j].resize( planeWidth[j], planeHeight[j], PCCCOLORFORMAT::YUV444 );
      projectionDirection[j].set( 128 );
#endif
      projection[j + 3].resize( planeWidth[j] * planeHeight[j], bb.min_[j] - 1 );
      projectionIndex[j + 3].resize( planeWidth[j] * planeHeight[j], -1 );
#ifdef DEBUG_NORMAL
      projectionImage[j + 3].resize( planeWidth[j], planeHeight[j], PCCCOLORFORMAT::YUV444 );
      projectionImage[j + 3].set( 0 );
      projectionDepth[j + 3].resize( planeWidth[j], planeHeight[j], PCCCOLORFORMAT::YUV444 );
      projectionDepth[j + 3].set( 0 );
      projectionDirection[j + 3].resize( planeWidth[j], planeHeight[j], PCCCOLORFORMAT::YUV444 );
      projectionDirection[j + 3].set( 128 );
#endif
    }
    // projecting points - NOTE: we assume that the points in a surface are dense enough to cover the projection plane
    for ( int i = 0; i < pointCloud.getPointCount(); i++ ) {
      PCCPoint3D point = pointCloud[i];
      // project the point into 6 faces
      for ( int j = 0; j < 3; j++ ) {
        int w     = point[( j + 1 ) % 3] - bb.min_[( j + 1 ) % 3];
        int h     = point[( j + 2 ) % 3] - bb.min_[( j + 2 ) % 3];
        int depth = point[j];
        if ( projection[j][w + h * ( planeWidth[j] )] > depth ) {
          projection[j][w + h * ( planeWidth[j] )]      = depth;
          projectionIndex[j][w + h * ( planeWidth[j] )] = i;
#ifdef DEBUG_NORMAL
          projectionImage[j].setValue( 0, w, h, pointCloud.getColor( i )[0] );
          projectionImage[j].setValue( 1, w, h, pointCloud.getColor( i )[1] );
          projectionImage[j].setValue( 2, w, h, pointCloud.getColor( i )[2] );
          uint8_t val = 255 * ( depth - bb.min_[j] ) / ( bb.max_[j] - bb.min_[j] );
          projectionDepth[j].setValue( 0, w, h, val );
          projectionDepth[j].setValue( 1, w, h, val );
          projectionDepth[j].setValue( 2, w, h, val );
          int value = 0;
          // if ((normals_[i] * (orientations6[j]-pointCloud[i])) < 0.0)
          if ( ( normals_[i] * orientations6_normal[j] ) < 0.0 ) value = 255;
          projectionDirection[j].setValue( 0, w, h, value );
          projectionDirection[j].setValue( 1, w, h, value );
          projectionDirection[j].setValue( 2, w, h, value );
#endif
        }
        if ( projection[j + 3][w + h * ( planeWidth[j] )] < depth ) {
          projection[j + 3][w + h * ( planeWidth[j] )]      = depth;
          projectionIndex[j + 3][w + h * ( planeWidth[j] )] = i;
#ifdef DEBUG_NORMAL
          projectionImage[j + 3].setValue( 0, w, h, pointCloud.getColor( i )[0] );
          projectionImage[j + 3].setValue( 1, w, h, pointCloud.getColor( i )[1] );
          projectionImage[j + 3].setValue( 2, w, h, pointCloud.getColor( i )[2] );
          uint8_t val = 255 * ( depth - bb.min_[j] ) / ( bb.max_[j] - bb.min_[j] );
          projectionDepth[j + 3].setValue( 0, w, h, val );
          projectionDepth[j + 3].setValue( 1, w, h, val );
          projectionDepth[j + 3].setValue( 2, w, h, val );
          int value = 0;
          // if ((normals_[i] * (orientations6[j+3]-pointCloud[i])) < 0.0)
          if ( ( normals_[i] * orientations6_normal[j + 3] ) < 0.0 ) value = 255;
          projectionDirection[j + 3].setValue( 0, w, h, value );
          projectionDirection[j + 3].setValue( 1, w, h, value );
          projectionDirection[j + 3].setValue( 2, w, h, value );
#endif
        }
      }
    }
    // now check the normal direction according to the projection planes and orient them
    for ( int j = 0; j < 3; j++ ) {
#ifdef DEBUG_NORMAL
      char name[1024];
      sprintf( name, "attribute_%d_%dx%d_8bit_p444.rgb", j, planeWidth[j], planeHeight[j] );
      projectionImage[j].write( name, 1 );
      sprintf( name, "attribute_%d_%dx%d_8bit_p444.rgb", j + 3, planeWidth[j], planeHeight[j] );
      projectionImage[j + 3].write( name, 1 );
      sprintf( name, "depth_%d_%dx%d_8bit_p444.rgb", j, planeWidth[j], planeHeight[j] );
      projectionDepth[j].write( name, 1 );
      sprintf( name, "depth_%d_%dx%d_8bit_p444.rgb", j + 3, planeWidth[j], planeHeight[j] );
      projectionDepth[j + 3].write( name, 1 );
      sprintf( name, "direction_%d_%dx%d_8bit_p444.rgb", j, planeWidth[j], planeHeight[j] );
      projectionDirection[j].write( name, 1 );
      sprintf( name, "direction_%d_%dx%d_8bit_p444.rgb", j + 3, planeWidth[j], planeHeight[j] );
      projectionDirection[j + 3].write( name, 1 );
#endif
      int sizeProjectionPlane = planeHeight[j] * planeWidth[j];
      for ( int i = 0; i < sizeProjectionPlane; i++ ) {
        int index = projectionIndex[j][i];
        if ( index >= 0 ) {
          visited_[index] = 1;
          if ( ( normals_[index] * orientations6_normal[j] ) < 0.0 ) {
            normals_[index] = -normals_[index];
#ifdef DEBUG_NORMAL
            PCCPoint3D point = pointCloud[index];
            int        w     = point[( j + 1 ) % 3] - bb.min_[( j + 1 ) % 3];
            int        h     = point[( j + 2 ) % 3] - bb.min_[( j + 2 ) % 3];
            projectionDirection[j].setValue( 0, w, h, 0 );
            projectionDirection[j].setValue( 1, w, h, 0 );
            projectionDirection[j].setValue( 2, w, h, 0 );
#endif
          }
        }
        int index2 = projectionIndex[j + 3][i];
        if ( index2 >= 0 ) {
          visited_[index2] = 1;
          if ( ( normals_[index2] * orientations6_normal[j + 3] ) < 0.0 ) {
            normals_[index2] = -normals_[index2];
#ifdef DEBUG_NORMAL
            PCCPoint3D point = pointCloud[index2];
            int        w     = point[( j + 1 ) % 3] - bb.min_[( j + 1 ) % 3];
            int        h     = point[( j + 2 ) % 3] - bb.min_[( j + 2 ) % 3];
            projectionDirection[j + 3].setValue( 0, w, h, 0 );
            projectionDirection[j + 3].setValue( 1, w, h, 0 );
            projectionDirection[j + 3].setValue( 2, w, h, 0 );
#endif
          }
        }
      }
#ifdef DEBUG_NORMAL
      sprintf( name, "direction_modified_%d_%dx%d_8bit_p444.rgb", j, planeWidth[j], planeHeight[j] );
      projectionDirection[j].write( name, 1 );
      sprintf( name, "direction_modified_%d_%dx%d_8bit_p444.rgb", j + 3, planeWidth[j], planeHeight[j] );
      projectionDirection[j + 3].write( name, 1 );
#endif
    }

#ifdef DEBUG_NORMAL
    // save mesh with normals
    PCCPointSet3 saveNormal2 = pointCloud;
    saveNormal2.addNormals();
    for ( int i = 0; i < saveNormal.getPointCount(); i++ ) {
      if ( visited_[i] ) saveNormal2.setNormal( i, PCCNormal3D( normals_[i][0], normals_[i][1], normals_[i][2] ) );
    }
    saveNormal2.write( "normal_projection_orientation.ply" );
#endif
    // smooth reference normals
    PCCNNQuery3 nNQuery  = {PCCPoint3D( 0.0 ),
                           static_cast<float>( params.radiusNormalOrientation_ ) * params.radiusNormalOrientation_,
                           params.numberOfNearestNeighborsInNormalOrientation_};
    PCCNNQuery3 nNQuery2 = {PCCPoint3D( 0.0 ), ( std::numeric_limits<float>::max )(),
                            params.numberOfNearestNeighborsInNormalOrientation_};

    for ( size_t ptIndex = 0; ptIndex < pointCount; ++ptIndex ) {
      if ( visited_[ptIndex] == 1u ) {
        size_t      numberOfNormals;
        PCCVector3D accumulatedNormals;
        addNeighbors( uint32_t( ptIndex ), pointCloud, kdtree, nNQuery2, nNResult, accumulatedNormals,
                      numberOfNormals );
        if ( numberOfNormals == 0u ) {
          if ( ptIndex != 0u ) {
            accumulatedNormals = normals_[ptIndex - 1];
          } else {
            accumulatedNormals = ( params.viewPoint_ - pointCloud[ptIndex] );
          }
        }
        if ( normals_[ptIndex] * accumulatedNormals < 0.0 ) { normals_[ptIndex] = -normals_[ptIndex]; }
      }
    }
#ifdef DEBUG_NORMAL
    // save mesh
    PCCPointSet3 saveNormal3 = pointCloud;
    saveNormal3.addNormals();
    for ( int i = 0; i < saveNormal3.getPointCount(); i++ ) {
      if ( visited_[i] ) saveNormal3.setNormal( i, PCCNormal3D( normals_[i][0], normals_[i][1], normals_[i][2] ) );
    }
    saveNormal3.write( "normal_projection_orientation_smoothed.ply" );
#endif
    for ( size_t ptIndex = 0; ptIndex < pointCount; ++ptIndex ) {
      if ( visited_[ptIndex] == 0u ) {
        visited_[ptIndex] = 1;
        size_t      numberOfNormals;
        PCCVector3D accumulatedNormals;
        addNeighbors( uint32_t( ptIndex ), pointCloud, kdtree, nNQuery2, nNResult, accumulatedNormals,
                      numberOfNormals );
        if ( numberOfNormals == 0u ) {
          if ( ptIndex != 0u ) {
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
          if ( visited_[current] == 0u ) {
            visited_[current] = 1;
            if ( normals_[edge.start_] * normals_[current] < 0.0 ) { normals_[current] = -normals_[current]; }
            addNeighbors( current, pointCloud, kdtree, nNQuery, nNResult, accumulatedNormals, numberOfNormals );
          }
        }
      }
    }
#ifdef DEBUG_NORMAL
    // save mesh
    PCCPointSet3 saveNormal4 = pointCloud;
    saveNormal4.addNormals();
    for ( int i = 0; i < saveNormal4.getPointCount(); i++ ) {
      saveNormal4.setNormal( i, PCCNormal3D( normals_[i][0], normals_[i][1], normals_[i][2] ) );
    }
    saveNormal4.write( "normal_orientation_final.ply" );
#endif
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
  for ( size_t i = 0; i < nNResult.count(); ++i ) {
    uint32_t index = static_cast<uint32_t>( nNResult.indices( i ) );
    if ( visited_[index] == 0u ) {
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
  PCCVector3D         n0;
  PCCVector3D         n1;
  PCCVector3D         n2;
  std::vector<size_t> subRanges;
  const size_t        chunckCount = 64;
  PCCDivideRange( 0, pointCount, chunckCount, subRanges );
  const double radius = params.radiusNormalSmoothing_ * params.radiusNormalSmoothing_;
  for ( size_t it = 0; it < params.numberOfIterationsInNormalSmoothing_; ++it ) {
#if defined( ENABLE_TBB )
    tbb::task_arena limited( static_cast<int>( nbThread_ ) );
    limited.execute( [&] {
      tbb::parallel_for( size_t( 0 ), subRanges.size() - 1, [&]( const size_t i ) {
#else
    for ( size_t i = 0; i < subRanges.size() - 1; i++ ) {
#endif
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
#if defined( ENABLE_TBB )
      } );
    } );
#else
  }
#endif
  }
}
