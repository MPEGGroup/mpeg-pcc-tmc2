/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2026, ISO/IEC
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
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#include "PCCCommon.h"
#include "PCCChrono.h"
#include "PCCMemory.h"
#include "PCCGs4AppEncoderParams.h"
#include "PCCBitstreamWriter.h"
#include "PCCAdaptation.h"
#include "PCCPointSet.h"
#include "PCCPointSetGS.h"
#include "PCCGSEncoder.h"
#include <program_options_lite.h>
#include <fstream>
#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>
#include <iostream>
#if defined( ENABLE_TBB )
#include <tbb/tbb.h>
#endif

#define SSVH_UNIT_SIZE_PRECISION_BYTES_MINUS1 3

using namespace std;
using namespace pcc;
using pcc::chrono::StopwatchUserTime;

bool loadPointCloudGS( const std::string &pathTemplate, uint32_t frameNumber, pcc::PCCPointSetGS &pointCloud ) {
  char fileName[512];
  snprintf( fileName, sizeof( fileName ), pathTemplate.c_str(), frameNumber );
  if ( !pointCloud.read( fileName ) ) {
    std::cout << "Error: can't open " << fileName << std::endl;
    return false;
  }
  return true;
}

void saveSampleStreamV3CUnit( const std::string &path, SampleStreamV3CUnit &ssvu, PCCLogger &logger ) {
  PCCBitstream     bitstream;
  PCCBitstreamStat bitstreamStat;
  bitstream.setLogger( logger );
  bitstream.setTrace( true );

  bitstreamStat.setHeader( bitstream.size() );
  PCCBitstreamWriter bitstreamWriter;
  size_t             headerSize = bitstreamWriter.write( ssvu, bitstream, SSVH_UNIT_SIZE_PRECISION_BYTES_MINUS1 + 1 );
  bitstreamStat.incrHeader( headerSize );
  bitstream.write( path );
  bitstreamStat.trace();
}

namespace {
std::vector<uint8_t> splitSplatsByImportanceScore( PCCPointSet3            &src,
                                                   const std::vector<int>  &splitRatios,
                                                   PCCAdaptationParameters &params,
                                                   uint32_t                 frameNumber ) {
  const size_t pointCount = src.getPointCount();
  if ( pointCount == 0 || splitRatios.empty() ) { return {}; }

  // 1. Read importance scores from the external file.
  std::vector<float> scores;
  scores.reserve( pointCount );
  {
    int         len = snprintf( nullptr, 0, params.importanceScoresPath_.c_str(), frameNumber );
    std::string scoreFilePath( len, '\0' );
    snprintf( &scoreFilePath[0], len + 1, params.importanceScoresPath_.c_str(), frameNumber );

    std::cout << "scoreFilePath: " << scoreFilePath << std::endl;

    std::ifstream file( scoreFilePath );
    if ( !file.is_open() ) {
      std::cerr << "Error: Failed to open score file: " << scoreFilePath << "\n";
      return {};
    }
    float scoreValue;
    while ( file >> scoreValue ) { scores.push_back( scoreValue ); }

    // Safety check: Ensure the number of read scores matches the total point count.
    if ( scores.size() != pointCount ) {
      std::cerr << "Warning: Score count (" << scores.size() << ") does not match point count (" << pointCount
                << "). Splitting aborted.\n";
      return {};
    }
  }

  // 2. Compute cumulative cut points (boundaries) for each LoD level.
  const int           totalWeight = std::accumulate( splitRatios.begin(), splitRatios.end(), 0 );
  std::vector<size_t> cuts;
  cuts.reserve( splitRatios.size() - 1 );
  {
    int cumulative = 0;
    for ( size_t k = 0; k + 1 < splitRatios.size(); ++k ) {
      cumulative += splitRatios[k];
      cuts.push_back(
          static_cast<size_t>( std::llround( static_cast<double>( pointCount ) * cumulative / totalWeight ) ) );
    }
  }

  // 3. Construct the index array and sort it based on importance scores in descending order.
  // Using std::sort with a lambda expression typically takes only tens of milliseconds for millions of elements.
  std::vector<size_t> permutation( pointCount );
  std::iota( permutation.begin(), permutation.end(), size_t( 0 ) );

  std::sort( permutation.begin(), permutation.end(), [&]( size_t a, size_t b ) {
    return scores[a] > scores[b];  // Descending order: higher scores first.
  } );

  // 4. Assign bitmasks sequentially based on sorted intervals, eliminating nested lookup loops and redundant counting
  // sweeps.
  std::vector<uint8_t> lodMasks( pointCount );
  std::vector<size_t>  counts( splitRatios.size(), 0 );

  size_t lastCut = 0;
  for ( size_t k = 0; k < splitRatios.size(); ++k ) {
    size_t currentCut = ( k < cuts.size() ) ? cuts[k] : pointCount;
    // k=0 is the highest-importance group (LoD0), so it maps to bit 0.
    uint8_t mask = static_cast<uint8_t>( 1u << k );

    for ( size_t idx = lastCut; idx < currentCut; ++idx ) { lodMasks[permutation[idx]] = mask; }
    counts[k] = currentCut - lastCut;  // Directly capture the count per level.
    lastCut   = currentCut;
  }

  // Print the boundary score thresholds between adjacent LoD levels.
  std::cout << "LoD boundary score thresholds:";
  for ( size_t k = 0; k < cuts.size(); ++k ) {
    size_t cutIdx = cuts[k];
    if ( cutIdx > 0 && cutIdx <= pointCount ) {
      // Because permutation is sorted descending, the last element belonging
      // to the current LoD defines the score threshold boundary.
      float thresholdScore = scores[permutation[cutIdx - 1]];
      std::cout << " Cut" << k << "(LoD" << k << "->LoD" << ( k + 1 ) << ")=" << thresholdScore;
    }
  }
  std::cout << "\n";

  // 5. Print per-level statistics for verification.
  std::cout << "LoD importance split:";
  for ( size_t k = 0; k < counts.size(); ++k ) { std::cout << " LoD" << k << "=" << counts[k]; }
  std::cout << " total=" << pointCount << "\n";

  return lodMasks;
}
}  // namespace

int main( int argc, char *argv[] ) {
  std::cout << "PccGs4AppEncoder v" << TMC2_VERSION_MAJOR << "." << TMC2_VERSION_MINOR << std::endl << std::endl;

#ifdef BITSTREAM_TRACE
  std::cout << "Bitstream trace enabled" << std::endl;
#endif

  int ret = 0;
  // this is mandatory to print floats with full precision
  std::cout.precision( std::numeric_limits<float>::max_digits10 );

  PCCApplicationParameters applicationParams;
  PCCGSEncoderParameters   encoderParams;
  PCCAdaptationParameters  adaptationParams;

  if ( !parseParameters( argc, argv, applicationParams, encoderParams, adaptationParams ) ) { return -1; }

  // Keep adaptation quantization depth aligned with 2D attribute coding depth.
  // This avoids 10-bit quantized attributes when running in 8-bit GS4 mode.
  adaptationParams.shBitDepth_       = static_cast<uint32_t>( encoderParams.attributeBitDepth2D_ );
  adaptationParams.opacityBitDepth_  = static_cast<uint32_t>( encoderParams.attributeBitDepth2D_ );
  adaptationParams.scaleBitDepth_    = static_cast<uint32_t>( encoderParams.attributeBitDepth2D_ );
  adaptationParams.rotationBitDepth_ = static_cast<uint32_t>( encoderParams.attributeBitDepth2D_ );

  if ( encoderParams.geometryBitDepth2D_ != 8 && encoderParams.geometryBitDepth2D_ != 10 ) {
    std::cerr << "Error: geometryBitDepth2D must be 8 or 10, got " << encoderParams.geometryBitDepth2D_ << std::endl;
    return -1;
  }
  if ( encoderParams.attributeBitDepth2D_ != 8 && encoderParams.attributeBitDepth2D_ != 10 ) {
    std::cerr << "Error: attributeBitDepth2D must be 8 or 10, got " << encoderParams.attributeBitDepth2D_ << std::endl;
    return -1;
  }
  if ( encoderParams.rawPatchDataArrangementMethod_ != 0 ) {
    std::cerr << "Error: rawPatchDataArrangementMethod support only 0, got "
              << encoderParams.rawPatchDataArrangementMethod_ << std::endl;
    return -1;
  }

  // Map encoder flags to adaptation parameters
  adaptationParams.gsdTransform2dPresentFlag_           = encoderParams.gsdTransform2dPresentFlag_;
  adaptationParams.dataAdaptiveShTransformEnabled_      = encoderParams.dataAdaptiveShTransformEnabled_;
  adaptationParams.gsdPcaDcFlag_                        = encoderParams.gsdPcaDcFlag_;
  adaptationParams.adaptivePcaEnabled_                  = encoderParams.adaptivePcaEnabled_;
  adaptationParams.enableShNonUniformQuantization_      = encoderParams.enableShNonUniformQuantization_;
  adaptationParams.enableOpacityNonUniformQuantization_ = encoderParams.enableOpacityNonUniformQuantization_;
  adaptationParams.encodeLinearOpacity_                 = encoderParams.encodeLinearOpacity_;
  if ( encoderParams.encodeLinearOpacity_ ) {
    // changing the default value of the opacity quantization
    adaptationParams.opacityLimitMin_ = 0.0;
    adaptationParams.opacityLimitMax_ = 1.0;
  }
  adaptationParams.enableScaleNonUniformQuantization_ = encoderParams.enableScaleNonUniformQuantization_;
  adaptationParams.shPredictionEnabled_               = encoderParams.shPredictionEnabled_;
  adaptationParams.enableScalePowerTransform_         = encoderParams.enableScalePowerTransform_;
  adaptationParams.rawPatchDataArrangementMethod_     = encoderParams.rawPatchDataArrangementMethod_;
  adaptationParams.enableGeometryPrediction_          = encoderParams.enableGeometryPrediction_;
  adaptationParams.geometryPredictionSegmentVal_      = encoderParams.geometryPredictionSegmentVal_;

#if defined( ENABLE_TBB )
  if ( applicationParams.nbThread_ > 0 ) {
    tbb::task_scheduler_init init( static_cast<int>( applicationParams.nbThread_ ) );
  }
#endif

  // remove the extension and keep folder path and base file name
  encoderParams.binaryFileBase_ = removeFileExtension( applicationParams.compressedStreamPath_ );

  if ( encoderParams.debugPath_.empty() ) {
    encoderParams.debugPath_ = getDirectoryName( applicationParams.compressedStreamPath_ );
  }
  encoderParams.geometryVideoEncoderConfig_ =
      applicationParams.configurationFolder_ + applicationParams.geometryConfig_;
  encoderParams.attributeVideoEncoderConfig_ =
      applicationParams.configurationFolder_ + applicationParams.attributeConfig_;
  encoderParams.packedVideoEncoderConfig_ = applicationParams.configurationFolder_ + applicationParams.packConfig_;

  // Timers to count elapsed wall/user time
  pcc::chrono::Stopwatch<std::chrono::steady_clock> clockWall;
  pcc::chrono::StopwatchUserTime                    clockUser;

  uint32_t startFrameNumber = static_cast<uint32_t>( applicationParams.startFrameNumber_ );
  uint32_t endFrameNumber =
      static_cast<uint32_t>( applicationParams.startFrameNumber_ + applicationParams.frameCount_ );

  PCCGSEncoder  encoderModule;
  PCCAdaptation adaptationModule;
  // results of the encoder will be stored in SSVU
  SampleStreamV3CUnit ssvu;

  PCCLogger logger;
  logger.initilalize( encoderParams.binaryFileBase_, true );

  clockWall.start();
  clockUser.start();

  for ( uint32_t frameNumber = startFrameNumber; frameNumber < endFrameNumber; ++frameNumber ) {
    // Load original point cloud as float (PCCPointSetGS)
    pcc::PCCPointSetGS originalPointCloudGS;
    if ( !loadPointCloudGS( applicationParams.uncompressedDataPath_, frameNumber, originalPointCloudGS ) ) { continue; }

    pcc::PCCPointSetGS pointCloudGS;
    if ( adaptationParams.enableAdaptiveShQuantization_ ) {
      if ( !adaptationModule.pruneAndGroupAdaptiveQuantization( originalPointCloudGS, pointCloudGS, adaptationParams,
                                                                frameNumber ) ) {
        std::cerr << "Error: adaptive SH pruning and grouping failed for frame " << frameNumber << std::endl;
        continue;
      }
    } else {
      pointCloudGS = originalPointCloudGS;
    }

    // Adapt point cloud: convert float (PCCPointSetGS) to quantized int (PCCPointSet3)
    pcc::PCCPointSet3 adaptedPointCloud;
    if ( !adaptationModule.adapt( pointCloudGS, adaptedPointCloud, adaptationParams ) ) {
      std::cerr << "Error: adaptation failed for frame " << frameNumber << std::endl;
      continue;
    }

    // print once when encoding fewer than the full 16 SH coefficient sets
    static bool shInfoPrinted = false;
    if ( !shInfoPrinted && adaptationParams.shCoefficientCount_ < 16 ) {
      std::cout << "Info: encoding with partial SH (" << adaptationParams.shCoefficientCount_
                << " coefficient sets out of 16)" << std::endl;
      shInfoPrinted = true;
    }

    // DEBUG
    std::string fileName( 512, '\0' );
    int         len =
        std::snprintf( &fileName[0], fileName.size(), applicationParams.uncompressedDataPath_.c_str(), frameNumber );
    if ( len > 0 && len < static_cast<int>( fileName.size() ) ) { fileName.resize( len ); }

    encoderParams.frameNameBase_ = removeFileExtension( getBasename( fileName ) );

    if ( encoderParams.keepIntermediateFiles_ ) {
      std::string adaptedPlyPath = encoderParams.debugPath_ + "/" + encoderParams.frameNameBase_ + ".asci.ply";
      adaptedPointCloud.write( adaptedPlyPath );

      std::string configPath =
          encoderParams.debugPath_ + "/" + encoderParams.frameNameBase_ + "_adaptation_config.json";
      adaptationParams.writeConfig( configPath );
    }

    // LoD loading (assign lod for each splats)
    std::vector<uint8_t> lodAssignment{};
    if ( encoderParams.lodSplitRatios_.size() >= 2 ) {
      lodAssignment = splitSplatsByImportanceScore( adaptedPointCloud, encoderParams.lodSplitRatios_, adaptationParams,
                                                    frameNumber );
      if ( lodAssignment.empty() ) {
        std::cerr << "Error: importance score loaded failed for frame " << frameNumber << std::endl;
        continue;
      }
    }

    // Encode adapted (quantized int) point cloud
    encoderModule.encode( adaptedPointCloud, encoderParams, adaptationParams, ssvu, lodAssignment, logger );

    // write encoded bitstream to file
    saveSampleStreamV3CUnit( applicationParams.compressedStreamPath_, ssvu, logger );
  }

  clockUser.stop();
  clockWall.stop();

  using namespace std::chrono;
  using ms            = milliseconds;
  auto totalWall      = duration_cast<ms>( clockWall.count() ).count();
  auto totalUserSelf  = duration_cast<ms>( clockUser.self.count() ).count();
  auto totalUserChild = duration_cast<ms>( clockUser.children.count() ).count();
  std::cout << "Processing time (wall): " << ( ret == 0 ? totalWall / 1000.0 : -1 ) << " s\n";
  std::cout << "Processing time (user.self): " << ( ret == 0 ? totalUserSelf / 1000.0 : -1 ) << " s\n";
  std::cout << "Processing time (user.children): " << ( ret == 0 ? totalUserChild / 1000.0 : -1 ) << " s\n";
  std::cout << "Peak memory: " << getPeakMemory() << " KB\n";
  return ret;
}
