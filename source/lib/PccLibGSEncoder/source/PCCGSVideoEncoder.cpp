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

#include "PCCGSVideoEncoder.h"
#include "PCCGSEncoderConstant.h"
#include "PCCVideoBitstream.h"
#include "PCCVirtualVideoEncoder.h"
#include <iostream>

using namespace pcc;

namespace {
constexpr size_t ATLAS_INDEX = 0;
}  // namespace

void PCCGSVideoEncoder::encode( PCCGSVideoFrames             &frames,
                                PCCHighLevelSyntax           &hls,
                                const PCCGSEncoderParameters &params ) {
  auto vps = hls.getVpsList().front();

  if ( vps.getGeometryVideoPresentFlag( ATLAS_INDEX ) ) {
    encodeVideo( frames.geometry, PCCVideoType::VIDEO_GEOMETRY, hls, params, params.geometryQP_,
                 params.geometryVideoEncoderConfig_ );
  }

  if ( vps.getAttributeVideoPresentFlag( ATLAS_INDEX ) ) {
    auto  &ai             = vps.getAttributeInformation( ATLAS_INDEX );
    size_t attributeCount = ai.getAttributeCount();

    for ( size_t attributeIdx = 0; attributeIdx < attributeCount; attributeIdx++ ) {
      auto   attributeType  = ai.getAttributeTypeId( attributeIdx );
      size_t partitionCount = ai.getAttributeDimensionPartitionsMinus1( attributeIdx );

      for ( size_t partitionIdx = 0; partitionIdx <= partitionCount; partitionIdx++ ) {
        PCCVideoType videoType = getVideoTypeByAttributeType( attributeType, partitionIdx );
        encodeVideo( frames.attributes[attributeIdx][partitionIdx], videoType, hls, params,
                     getVideoQPByAttributeType( attributeType, params ), params.attributeVideoEncoderConfig_ );
      }
    }
  }

  if ( vps.getVpsPackedVideoExtension().getPackedVideoPresentFlag( ATLAS_INDEX ) ) {
    auto &pin       = vps.getVpsPackedVideoExtension().getPackingInformation( ATLAS_INDEX );
    auto  mapsCount = pin.getPackedMapCountMinus1() + 1;

    for ( size_t packedMapIndex = 0; packedMapIndex < mapsCount; packedMapIndex++ ) {
      encodeVideo( frames.packed[packedMapIndex],
                   static_cast<PCCVideoType>( PCCVideoType::VIDEO_PACKED + packedMapIndex ), hls, params,
                   getPackedVideoQP( packedMapIndex, params ), params.packedVideoEncoderConfig_ );
    }
  }
}

void PCCGSVideoEncoder::encodeVideo( PCCVideo<uint16_t, 3>        &videoFrames,
                                     PCCVideoType                  videoType,
                                     PCCHighLevelSyntax           &hls,
                                     const PCCGSEncoderParameters &params,
                                     const int32_t                 videoQP,
                                     const std::string            &encoderConfig ) {
  PCCVideoBitstream &bitstream = hls.createVideoBitstream( videoType );
  int                nbyte     = 2;
  // Check if encoder config file exists
  FILE *file = fopen( encoderConfig.c_str(), "r" );
  if ( !file ) {
    std::cerr << "Error: encoder config file " << encoderConfig << " does not exist." << std::endl;
    exit( -1 );
  } else {
    fclose( file );
  }

  int width  = videoFrames.getWidth();
  int height = videoFrames.getHeight();

  const std::string type         = bitstream.getExtension();
  const std::string baseFileName = params.debugPath_ + "/" + params.frameNameBase_ + "_" + type;
  const std::string binFileName  = baseFileName + ".bin";
  const char       *bitdepth     = nbyte == 2 ? "10" : "8";

  const std::string srcYuvFileName = addVideoFormat( baseFileName, width, height, true, true, bitdepth );
  const std::string recYuvFileName = addVideoFormat( baseFileName + "_rec", width, height, true, true, bitdepth );

  if ( params.keepIntermediateFiles_ ) { videoFrames.write( srcYuvFileName, nbyte ); }

  // Encode video
  PCCVideoEncoderParameters encParams;
  encParams.encoderPath_                 = "";
  encParams.srcYuvFileName_              = srcYuvFileName;
  encParams.binFileName_                 = binFileName;
  encParams.recYuvFileName_              = recYuvFileName;
  encParams.encoderConfig_               = encoderConfig;
  encParams.qp_                          = videoQP;
  encParams.inputBitDepth_               = 10;
  encParams.internalBitDepth_            = 10;
  encParams.outputBitDepth_              = 10;
  encParams.use444CodecIo_               = false;
  encParams.usePccMotionEstimation_      = false;
  encParams.blockToPatchFile_            = "";
  encParams.occupancyMapFile_            = "";
  encParams.patchInfoFile_               = "";
  encParams.cuTransquantBypassFlagForce_ = false;
  encParams.transquantBypassEnable_      = false;
  encParams.inputColourSpaceConvert_     = false;
  encParams.usePccRDO_                   = false;
  encParams.shvcLayerIndex_              = 0;
  encParams.shvcRateX_                   = 0;
  encParams.shvcRateY_                   = 0;

  printf( "Encode: video size = %zu x %zu num frames = %zu \n", videoFrames.getWidth(), videoFrames.getHeight(),
          videoFrames.getFrameCount() );
  fflush( stdout );

  PCCVideo<uint16_t, 3> videoRec;
  auto                  encoder = PCCVirtualVideoEncoder<uint16_t>::create( HMLIB );
  encoder->encode( videoFrames, encParams, bitstream, videoRec );
  if ( params.keepIntermediateFiles_ ) { bitstream.write( binFileName ); }
  bitstream.byteStreamToSampleStream();
  if ( params.keepIntermediateFiles_ ) { videoRec.write( recYuvFileName, nbyte ); }
}

PCCVideoType PCCGSVideoEncoder::getVideoTypeByAttributeType( AttributeTypeId attributeType, size_t partitionIdx ) {
  PCCVideoType videoType = PCCVideoType::VIDEO_ATTRIBUTE;
  switch ( attributeType ) {
    case ATTR_TRANSPARENCY: videoType = PCCVideoType::VIDEO_OPACITY_T0; break;
    case ATTR_SPHERICAL_HARMONICS: videoType = PCCVideoType::VIDEO_SPHERICAL_HARMONIC_T0; break;
    case ATTR_SCALE: videoType = PCCVideoType::VIDEO_SCALE_T0; break;
    case ATTR_ROTATION: videoType = PCCVideoType::VIDEO_ROTATION_T0; break;
    default: break;
  }
  return static_cast<PCCVideoType>( static_cast<int>( videoType ) + static_cast<int>( partitionIdx * 64 ) );
}

int32_t PCCGSVideoEncoder::getVideoQPByAttributeType( AttributeTypeId               attributeType,
                                                      const PCCGSEncoderParameters &params ) {
  switch ( attributeType ) {
    case ATTR_TRANSPARENCY: return params.attributeOpacityQP_;
    case ATTR_SPHERICAL_HARMONICS: return params.attributeColorQP_;
    case ATTR_SCALE: return params.attributeScaleQP_;
    case ATTR_ROTATION: return params.attributeRotationQP_;
    default: break;
  }
  std::cerr << "Error: invalid attribute type." << std::endl;
  exit( -1 );
}

int32_t PCCGSVideoEncoder::getPackedVideoQP( size_t packedMapIndex, const PCCGSEncoderParameters &params ) {
  if ( params.packQP_.empty() ) {
    std::cerr << "Error: packed video requires at least one packQP value." << std::endl;
    exit( -1 );
  }
  if ( params.packQP_.size() == 1 ) {
    return params.packQP_[0];
  } else {
    assert( packedMapIndex < params.packQP_.size() );
    return params.packQP_[packedMapIndex];
  }
}
