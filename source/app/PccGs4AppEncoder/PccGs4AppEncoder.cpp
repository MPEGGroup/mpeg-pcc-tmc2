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

  // Map encoder flags to adaptation parameters
  adaptationParams.gsdTransform2dPresentFlag_      = encoderParams.gsdTransform2dPresentFlag_;
  adaptationParams.dataAdaptiveShTransformEnabled_ = encoderParams.dataAdaptiveShTransformEnabled_;
  adaptationParams.enableShNonUniformQuantization_ = encoderParams.enableShNonUniformQuantization_;
  adaptationParams.encodeLinearOpacity_            = encoderParams.encodeLinearOpacity_;
  if ( encoderParams.encodeLinearOpacity_ ) {
    // changing the default value of the opacity quantization
    adaptationParams.computedOpacityMin_ = 0.0;
    adaptationParams.computedOpacityMax_ = 1.0;
  }
  adaptationParams.enableScaleNonUniformQuantization_ = encoderParams.enableScaleNonUniformQuantization_;
  adaptationParams.enableScalePowerTransform_         = encoderParams.enableScalePowerTransform_;

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

    // Encode adapted (quantized int) point cloud
    encoderModule.encode( adaptedPointCloud, encoderParams, adaptationParams, ssvu, logger );
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
