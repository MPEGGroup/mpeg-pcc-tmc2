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
#include "PCCGroupOfFrames.h"
#include "PCCPointSet.h"
#include "PCCKdTree.h"
#include <tbb/tbb.h>

#include "PCCMetrics.h"

using namespace std;
using namespace pcc;

float getPSNR( float dist, float p, float factor = 1.0 ) {
  float max_energy = p * p;
  float psnr       = 10 * log10( ( factor * max_energy ) / dist );
  return psnr;
}

void convertRGBtoYUVBT709( const PCCColor3B& rgb, std::vector<float>& yuv ) {
  yuv.resize( 3 );
  yuv[0] = float( ( 0.2126 * rgb[0] + 0.7152 * rgb[1] + 0.0722 * rgb[2] ) / 255.0 );
  yuv[1] = float( ( -0.1146 * rgb[0] - 0.3854 * rgb[1] + 0.5000 * rgb[2] ) / 255.0 + 0.5000 );
  yuv[2] = float( ( 0.5000 * rgb[0] - 0.4542 * rgb[1] - 0.0458 * rgb[2] ) / 255.0 + 0.5000 );
}

QualityMetrics::QualityMetrics() :
    c2cMse_( 0.0F ),
    c2cHausdorff_( 0.0F ),
    c2cPsnr_( 0.0F ),
    c2cHausdorffPsnr_( 0.0F ),
    c2pMse_( 0.0F ),
    c2pHausdorff_( 0.0F ),
    c2pPsnr_( 0.0F ),
    c2pHausdorffPsnr_( 0.0F ),
    psnr_( 0.0F ),
    reflectanceMse_( 0.0F ),
    reflectancePsnr_( 0.0F ) {
  colorMse_[0] = colorMse_[0] = colorMse_[0] = 0.0F;
  colorPsnr_[0] = colorPsnr_[0] = colorPsnr_[0] = 0.0F;
}

void QualityMetrics::setParameters( const PCCMetricsParameters& params ) { params_ = params; }

void QualityMetrics::compute( const PCCPointSet3& pointcloudA, const PCCPointSet3& pointcloudB ) {
  double maxC2c         = ( std::numeric_limits<double>::min )();
  double maxC2p         = ( std::numeric_limits<double>::min )();
  double sseC2p         = 0;
  double sseC2c         = 0;
  double sseReflectance = 0;
  size_t num            = 0;
  double sseColor[3]    = {0.0, 0.0, 0.0};

  psnr_ = params_.resolution_;

  PCCKdTree    kdtree( pointcloudB );
  PCCNNResult  result;
  const size_t num_results_max  = 30;
  const size_t num_results_incr = 5;

  auto& normalsB = pointcloudB.getNormals();
  for ( size_t indexA = 0; indexA < pointcloudA.getPointCount(); indexA++ ) {
    // For point 'i' in A, find its nearest neighbor in B. store it in 'j'
    size_t num_results = 0;
    do {
      num_results += num_results_incr;
      kdtree.search( pointcloudA[indexA], num_results, result );
    } while ( result.dist( 0 ) == result.dist( num_results - 1 ) && num_results + num_results_incr <= num_results_max );

    // Compute point-to-point, which should be equal to sqrt( dist[0] )
    double distProjC2c = result.dist( 0 );

    // Build the list of all the points of same distances.
    std::vector<size_t> sameDistList;
    if ( params_.computeColor_ || params_.computeC2p_ ) {
      for ( size_t j = 0; j < num_results && ( fabs( result.dist( 0 ) - result.dist( j ) ) < 1e-8 ); j++ ) {
        sameDistList.push_back( result.indices( j ) );
      }
    }
    std::sort( sameDistList.begin(), sameDistList.end() );

    // Compute point-to-plane, normals in B will be used for point-to-plane
    double distProjC2p = 0.0;
    if ( params_.computeC2p_ && pointcloudB.hasNormals() && pointcloudA.hasNormals() ) {
      for ( auto& indexB : sameDistList ) {
        std::vector<double> errVector( 3 );
        for ( size_t j = 0; j < 3; j++ ) { errVector[j] = pointcloudA[indexA][j] - pointcloudB[indexB][j]; }
        double dist = pow( errVector[0] * normalsB[indexB][0] + errVector[1] * normalsB[indexB][1] +
                               errVector[2] * normalsB[indexB][2],
                           2.F );
        distProjC2p += dist;
      }
      distProjC2p /= sameDistList.size();
    }

    size_t indexB = result.indices( 0 );
    double distColor[3];
    distColor[0] = distColor[1] = distColor[2] = 0.0;
    if ( params_.computeColor_ && pointcloudA.hasColors() && pointcloudB.hasColors() ) {
      std::vector<float> yuvA;
      std::vector<float> yuvB;
      PCCColor3B         rgb;
      convertRGBtoYUVBT709( pointcloudA.getColor( indexA ), yuvA );
      if ( params_.neighborsProc_ != 0 ) {
        switch ( params_.neighborsProc_ ) {
          case 0: break;
          case 1:  // Average
          case 2:  // Weighted average
          {
            int          nbdupcumul = 0;
            unsigned int r          = 0;
            unsigned int g          = 0;
            unsigned int b          = 0;
            for ( unsigned long long i : sameDistList ) {
              int nbdup = 1;  // pointcloudB.xyz.nbdup[ indices_sameDst[n] ];
              r += nbdup * pointcloudB.getColor( i )[0];
              g += nbdup * pointcloudB.getColor( i )[1];
              b += nbdup * pointcloudB.getColor( i )[2];
              nbdupcumul += nbdup;
            }
            rgb[0] = static_cast<unsigned char>( round( static_cast<double>( r ) / nbdupcumul ) );
            rgb[1] = static_cast<unsigned char>( round( static_cast<double>( g ) / nbdupcumul ) );
            rgb[2] = static_cast<unsigned char>( round( static_cast<double>( b ) / nbdupcumul ) );
            convertRGBtoYUVBT709( rgb, yuvB );
            for ( size_t i = 0; i < 3; i++ ) { distColor[i] = pow( yuvA[i] - yuvB[i], 2.F ); }
          } break;
          case 3:  // Min
          case 4:  // Max
          {
            float  distBest  = 0;
            size_t indexBest = 0;
            for ( auto index : sameDistList ) {
              convertRGBtoYUVBT709( pointcloudB.getColor( index ), yuvB );
              float dist =
                  pow( yuvA[0] - yuvB[0], 2.F ) + pow( yuvA[1] - yuvB[1], 2.F ) + pow( yuvA[2] - yuvB[2], 2.F );
              if ( ( ( params_.neighborsProc_ == 3 ) && ( dist < distBest ) ) ||
                   ( ( params_.neighborsProc_ == 4 ) && ( dist > distBest ) ) ) {
                distBest  = dist;
                indexBest = index;
              }
            }
            convertRGBtoYUVBT709( pointcloudB.getColor( indexBest ), yuvB );
          } break;
        }
      } else {
        convertRGBtoYUVBT709( pointcloudB.getColor( indexB ), yuvB );
      }
      for ( size_t i = 0; i < 3; i++ ) { distColor[i] = pow( yuvA[i] - yuvB[i], 2.F ); }
    }

    double distReflectance = 0.0;
    if ( params_.computeReflectance_ && pointcloudA.hasReflectances() && pointcloudB.hasReflectances() ) {
      distReflectance = pow( pointcloudA.getReflectance( indexA ) - pointcloudB.getReflectance( indexB ), 2.F );
    }
    num++;

    // mean square distance
    if ( params_.computeC2c_ ) {
      sseC2c += distProjC2c;
      if ( distProjC2c > maxC2c ) { maxC2c = distProjC2c; }
    }
    if ( params_.computeC2p_ ) {
      sseC2p += distProjC2p;
      if ( distProjC2p > maxC2p ) { maxC2p = distProjC2p; }
    }
    if ( params_.computeColor_ ) {
      for ( size_t i = 0; i < 3; i++ ) { sseColor[i] += distColor[i]; }
    }
    if ( params_.computeReflectance_ && pointcloudA.hasReflectances() && pointcloudB.hasReflectances() ) {
      sseReflectance += distReflectance;
    }
  }

  if ( params_.computeC2c_ ) {
    c2cMse_  = float( sseC2c / num );
    c2cPsnr_ = getPSNR( c2cMse_, psnr_, 3 );
    if ( params_.computeHausdorff_ ) {
      c2cHausdorff_     = float( maxC2c );
      c2cHausdorffPsnr_ = getPSNR( c2cHausdorff_, psnr_, 3 );
    }
  }

  if ( params_.computeC2p_ ) {
    c2pMse_  = float( sseC2p / num );
    c2pPsnr_ = getPSNR( c2pMse_, psnr_, 3 );
    if ( params_.computeHausdorff_ ) {
      c2pHausdorff_     = float( maxC2p );
      c2pHausdorffPsnr_ = getPSNR( c2pHausdorff_, psnr_, 3 );
    }
  }
  if ( params_.computeColor_ ) {
    for ( size_t i = 0; i < 3; i++ ) {
      colorMse_[i]  = float( sseColor[i] / num );
      colorPsnr_[i] = getPSNR( colorMse_[i], 1.0 );
    }
  }
  if ( params_.computeLidar_ ) {
    reflectanceMse_  = float( sseReflectance / num );
    reflectancePsnr_ = getPSNR( float( reflectanceMse_ ), float( ( std::numeric_limits<unsigned short>::max )() ) );
  }
}

void QualityMetrics::print( char code ) {
  switch ( code ) {
    case '1':
      std::cout << "1. Use infile1 (A) as reference, loop over A, use normals "
                   "on B. (A->B)."
                << std::endl;
      break;
    case '2':
      std::cout << "2. Use infile2 (B) as reference, loop over B, use normals "
                   "on A. (B->A)."
                << std::endl;
      break;
    case 'F': std::cout << "3. Final (symmetric)." << std::endl; break;
    default:
      std::cout << "Mode not supported. " << std::endl;
      exit( -1 );
      break;
  }

  if ( params_.computeC2c_ ) {
    std::cout << "   mse" << code << "      (p2point): " << c2cMse_ << std::endl;
    std::cout << "   mse" << code << ",PSNR (p2point): " << c2cPsnr_ << std::endl;
  }
  if ( params_.computeC2p_ ) {
    std::cout << "   mse" << code << "      (p2plane): " << c2pMse_ << std::endl;
    std::cout << "   mse" << code << ",PSNR (p2plane): " << c2pPsnr_ << std::endl;
  }
  if ( params_.computeHausdorff_ ) {
    if ( params_.computeC2c_ ) {
      std::cout << "   h.       " << code << "(p2point): " << c2cHausdorff_ << std::endl;
      std::cout << "   h.,PSNR  " << code << "(p2point): " << c2cHausdorffPsnr_ << std::endl;
    }
    if ( params_.computeC2p_ ) {
      std::cout << "  h.       " << code << "(p2plane): " << c2cHausdorff_ << std::endl;
      std::cout << "  h.,PSNR  " << code << "(p2plane): " << c2cHausdorffPsnr_ << std::endl;
    }
  }
  if ( params_.computeColor_ ) {
    for ( size_t i = 0; i < 3; i++ ) {
      std::cout << "   c[" << i << "],    " << code << "         : " << colorMse_[i] << std::endl;
    }
    for ( size_t i = 0; i < 3; i++ ) {
      std::cout << "   c[" << i << "],PSNR" << code << "         : " << colorPsnr_[i] << std::endl;
    }
  }
  if ( params_.computeReflectance_ ) {
    std::cout << "   r,       " << code << "        :  " << reflectanceMse_ << std::endl;
    std::cout << "   r,PSNR   " << code << "        :  " << reflectancePsnr_ << std::endl;
  }
}
PCCMetrics::PCCMetrics() :
    sourcePoints_( 0 ),
    sourceDuplicates_( 0 ),
    reconstructPoints_( 0 ),
    reconstructDuplicates_( 0 ) {}
PCCMetrics::~PCCMetrics() = default;
void PCCMetrics::setParameters( const PCCMetricsParameters& params ) { params_ = params; }

QualityMetrics QualityMetrics::operator+( const QualityMetrics& metric ) const {
  QualityMetrics result;
  // Derive the final symmetric metric
  if ( params_.computeC2c_ ) {
    result.c2cMse_  = ( std::max )( c2cMse_, metric.c2cMse_ );
    result.c2cPsnr_ = ( std::min )( c2cPsnr_, metric.c2cPsnr_ );
  }
  if ( params_.computeC2p_ ) {
    result.c2pMse_  = ( std::max )( c2pMse_, metric.c2pMse_ );
    result.c2pPsnr_ = ( std::min )( c2pPsnr_, metric.c2pPsnr_ );
  }

  if ( params_.computeHausdorff_ ) {
    if ( params_.computeC2c_ ) {
      result.c2cHausdorff_     = ( std::max )( c2cHausdorff_, metric.c2cHausdorff_ );
      result.c2cHausdorffPsnr_ = ( std::min )( c2cHausdorffPsnr_, metric.c2cHausdorffPsnr_ );
    }
    if ( params_.computeC2p_ ) {
      result.c2pHausdorff_     = ( std::max )( c2pHausdorff_, metric.c2pHausdorff_ );
      result.c2pHausdorffPsnr_ = ( std::min )( c2pHausdorffPsnr_, metric.c2pHausdorffPsnr_ );
    }
  }
  if ( params_.computeColor_ ) {
    for ( size_t i = 0; i < 3; i++ ) {
      result.colorMse_[i]  = ( std::max )( colorMse_[i], metric.colorMse_[i] );
      result.colorPsnr_[i] = ( std::min )( colorPsnr_[i], metric.colorPsnr_[i] );
    }
  }
  if ( params_.computeReflectance_ ) {
    result.reflectanceMse_  = ( std::max )( reflectanceMse_, metric.reflectanceMse_ );
    result.reflectancePsnr_ = ( std::min )( reflectancePsnr_, metric.reflectancePsnr_ );
  }
  return result;
}

void PCCMetrics::compute( const PCCGroupOfFrames& sources,
                          const PCCGroupOfFrames& reconstructs,
                          const PCCGroupOfFrames& normals ) {
  PCCPointSet3 normalEmpty;
  if ( normals.getFrameCount() != 0 && sources.getFrameCount() != normals.getFrameCount() ) {
    params_.computeC2p_ = false;
  }
  if ( ( sources.getFrameCount() != reconstructs.getFrameCount() ) ) {
    printf(
        "Error: group of frames must have same numbers of frames. ( src = %zu "
        "rec = %zu norm = %zu ) \n",
        sources.getFrameCount(), reconstructs.getFrameCount(), normals.getFrameCount() );
    exit( -1 );
  }
  for ( size_t i = 0; i < sources.getFrameCount(); i++ ) {
    const PCCPointSet3& sourceOrg      = sources[i];
    const PCCPointSet3& reconstructOrg = reconstructs[i];
    sourcePoints_.push_back( sourceOrg.getPointCount() );
    reconstructPoints_.push_back( reconstructOrg.getPointCount() );
    PCCPointSet3 source;
    PCCPointSet3 reconstruct;
    if ( params_.dropDuplicates_ != 0 ) {
      sourceOrg.removeDuplicate( source, params_.dropDuplicates_ );
      reconstructOrg.removeDuplicate( reconstruct, params_.dropDuplicates_ );
      sourceDuplicates_.push_back( source.getPointCount() );
      reconstructDuplicates_.push_back( reconstruct.getPointCount() );
      compute( source, reconstruct, normals.getFrameCount() == 0 ? normalEmpty : normals[i] );
    } else {
      source      = sourceOrg;
      reconstruct = reconstructOrg;
      sourceDuplicates_.push_back( 0 );
      reconstructDuplicates_.push_back( 0 );
      compute( source, reconstruct, normals.getFrameCount() == 0 ? normalEmpty : normals[i] );
    }
  }
}

void PCCMetrics::compute( PCCPointSet3& source, PCCPointSet3& reconstruct, const PCCPointSet3& normalSource ) {
  if ( normalSource.getPointCount() > 0 ) {
    source.copyNormals( normalSource );
    reconstruct.scaleNormals( normalSource );
  }
  QualityMetrics q1;
  QualityMetrics q2;
  q1.setParameters( params_ );
  q1.compute( source, reconstruct );
  q2.setParameters( params_ );
  q2.compute( reconstruct, source );
  quality1_.push_back( q1 );
  quality2_.push_back( q2 );
  qualityF_.push_back( q1 + q2 );
}

void PCCMetrics::display() {
  printf( "Metrics results \n" );
  for ( size_t i = 0; i < qualityF_.size(); i++ ) {
    printf( "WARNING: %zu points with same coordinates found\n", reconstructPoints_[i] - reconstructDuplicates_[i] );
    std::cout << "Imported intrinsic resoluiton: " << params_.resolution_ << std::endl;
    std::cout << "Peak distance for PSNR: " << params_.resolution_ << std::endl;
    std::cout << "Point cloud sizes for org version, dec version, and the scaling ratio: " << sourcePoints_[i] << ", "
              << reconstructDuplicates_[i] << ", "
              << static_cast<float>( reconstructDuplicates_[i] ) / static_cast<float>( sourcePoints_[i] ) << std::endl;
    quality1_[i].print( '1' );
    quality2_[i].print( '2' );
    qualityF_[i].print( 'F' );
  }
}
