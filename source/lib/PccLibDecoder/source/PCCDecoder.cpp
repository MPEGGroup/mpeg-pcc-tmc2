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
#include "PCCBitstream.h"
#include "PCCVideoBitstream.h"
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCPatch.h"
#include "PCCVideoDecoder.h"
#include "PCCGroupOfFrames.h"
#include "PCCBitstreamDecoder.h"
#include <tbb/tbb.h>
#include "PCCDecoder.h"

using namespace pcc;
using namespace std;

PCCDecoder::PCCDecoder() {
#ifdef ENABLE_PAPI_PROFILING
  initPapiProfiler();
#endif 
}
PCCDecoder::~PCCDecoder() {}

void PCCDecoder::setParameters( PCCDecoderParameters params ) { params_ = params; }

int PCCDecoder::decode( PCCBitstream& bitstream, PCCContext& context, PCCGroupOfFrames& reconstructs ) {
  int ret = 0;
  if ( params_.nbThread_ > 0 ) { tbb::task_scheduler_init init( (int)params_.nbThread_ ); }
  PCCBitstreamDecoder bitstreamDecoder;
#ifdef BITSTREAM_TRACE
  bitstream.setTrace( true );
  bitstream.openTrace( removeFileExtension( params_.compressedStreamPath_ ) + "_hls_decode.txt" );
#endif
  if ( !bitstreamDecoder.decode( bitstream, context ) ) { return 0; }
#ifdef BITSTREAM_TRACE
  bitstream.closeTrace();
#endif

#ifdef CODEC_TRACE
  setTrace( true );
  openTrace( stringFormat( "%s_GOF%u_patch_decode.txt", removeFileExtension( params_.compressedStreamPath_ ).c_str(),
                           context.getSps().getSequenceParameterSetId() ) );
#endif
  createPatchFrameDataStructure( context );
#ifdef CODEC_TRACE
  closeTrace();
#endif

  ret |= decode( context, reconstructs );
  return ret;
}

int PCCDecoder::decode( PCCContext& context, PCCGroupOfFrames& reconstructs ) {
  reconstructs.resize( context.size() );
  PCCVideoDecoder   videoDecoder;
  std::stringstream path;
  auto&             sps   = context.getSps();
  auto&             pdg   = context.getPatchDataGroup();
  auto&             ai    = sps.getAttributeInformation();
  auto&             oi    = sps.getOccupancyInformation();
  auto&             gi    = sps.getGeometryInformation();
  auto&             pfgps = pdg.getPatchFrameGeometryParameterSet( 0 );
  auto&             pfaps = pdg.getPatchFrameAttributeParameterSet( 0 );
  auto&             gfp   = pfgps.getGeometryFrameParams();
  auto&             afp   = pfaps.getAttributeFrameParams();

  path << removeFileExtension( params_.compressedStreamPath_ ) << "_dec_GOF" << sps.getSequenceParameterSetId() << "_";
#ifdef CODEC_TRACE
  setTrace( true );
  openTrace( stringFormat( "%s_GOF%u_codec_decode.txt", removeFileExtension( params_.compressedStreamPath_ ).c_str(),
                           sps.getSequenceParameterSetId() ) );
#endif
  bool lossyMpp = !sps.getLosslessGeo() && sps.getPcmPatchEnabledFlag();
  const size_t frameCountGeometry = sps.getLayerCountMinus1() + 1;
  const size_t frameCountTexture  = sps.getLayerCountMinus1() + 1;

  auto& videoBitstreamOM = context.getVideoBitstream( VIDEO_OCCUPANCY );
	int decodedBitDepthOM = 8;
  videoDecoder.decompress( context.getVideoOccupancyMap(), path.str(), context.size(), videoBitstreamOM,
                           params_.videoDecoderOccupancyMapPath_, context, decodedBitDepthOM, params_.keepIntermediateFiles_,
                           ( sps.getLosslessGeo() ? sps.getLosslessGeo444() : false ), false, "", "" );
	//converting the decoded bitdepth to the nominal bitdepth
	context.getVideoOccupancyMap().convertBitdepth(decodedBitDepthOM, oi.getOccupancyNominal2DBitdepthMinus1() + 1, oi.getOccupancyMSBAlignFlag());
  context.setOccupancyPrecision( sps.getFrameWidth() / context.getVideoOccupancyMap().getWidth() );
  if( !gfp.getGeometryPatchBlockFilteringEnableFlag() ) {
    generateOccupancyMap( context, context.getOccupancyPrecision(), oi.getLossyOccupancyMapCompressionThreshold(),
                          sps.getEnhancedOccupancyMapForDepthFlag() );
  }

  if ( sps.getLayerCountMinus1() > 0 && !sps.getLayerAbsoluteCodingEnabledFlag( 1 ) ) {
    if ( lossyMpp ) {
      std::cout << "ERROR! Lossy-missed-points-patch code not implemented when absoluteD_ = 0 as "
                   "of now. Exiting ..."
                << std::endl;
      std::exit( -1 );
    }
    // Compress D0
		int decodedBitDepthD0 = gi.getGeometryNominal2dBitdepthMinus1() + 1;
    auto& videoBitstreamD0 = context.getVideoBitstream( VIDEO_GEOMETRY_D0 );
    videoDecoder.decompress( context.getVideoGeometry(), path.str(), context.size(), videoBitstreamD0,
                             params_.videoDecoderPath_, context, decodedBitDepthD0,
                             params_.keepIntermediateFiles_,
                             ( sps.getLosslessGeo() ? sps.getLosslessGeo444() : false ) );
		context.getVideoGeometry().convertBitdepth(decodedBitDepthD0, gi.getGeometryNominal2dBitdepthMinus1() + 1, gi.getGeometryMSBAlignFlag());
    std::cout << "geometry D0 video ->" << videoBitstreamD0.naluSize() << " B" << std::endl;

    // Compress D1
		int decodedBitDepthD1 = gi.getGeometryNominal2dBitdepthMinus1() + 1;
    auto& videoBitstreamD1 = context.getVideoBitstream( VIDEO_GEOMETRY_D1 );
    videoDecoder.decompress( context.getVideoGeometryD1(), path.str(), context.size(), videoBitstreamD1,
                             params_.videoDecoderPath_, context, decodedBitDepthD1,
                             params_.keepIntermediateFiles_,
                             ( sps.getLosslessGeo() ? sps.getLosslessGeo444() : false ) );
		context.getVideoGeometryD1().convertBitdepth(decodedBitDepthD1, gi.getGeometryNominal2dBitdepthMinus1() + 1, gi.getGeometryMSBAlignFlag());
    std::cout << "geometry D1 video ->" << videoBitstreamD1.naluSize() << " B" << std::endl;

    std::cout << "geometry video ->" << videoBitstreamD1.naluSize() + videoBitstreamD1.naluSize() << " B" << std::endl;
  } else {
		int decodedBitDepthGeo = gi.getGeometryNominal2dBitdepthMinus1() + 1;
    auto& videoBitstream = context.getVideoBitstream( VIDEO_GEOMETRY );
    videoDecoder.decompress( context.getVideoGeometry(), path.str(), context.size() * frameCountGeometry,
                             videoBitstream, params_.videoDecoderPath_, context,
                             decodedBitDepthGeo, params_.keepIntermediateFiles_,
                             sps.getLosslessGeo() & sps.getLosslessGeo444() );
		context.getVideoGeometry().convertBitdepth(decodedBitDepthGeo, gi.getGeometryNominal2dBitdepthMinus1() + 1, gi.getGeometryMSBAlignFlag());
    std::cout << "geometry video ->" << videoBitstream.naluSize() << " B" << std::endl;
  }

  if ( sps.getPcmPatchEnabledFlag() && sps.getPcmSeparateVideoPresentFlag() ) {
		int decodedBitDepthMP = gi.getGeometryNominal2dBitdepthMinus1() + 1;
    auto& videoBitstreamMP = context.getVideoBitstream( VIDEO_GEOMETRY_MP );
    videoDecoder.decompress( context.getVideoMPsGeometry(), path.str(), context.size(), videoBitstreamMP,
                             params_.videoDecoderPath_, context,
                             decodedBitDepthMP,
                             params_.keepIntermediateFiles_ );
		context.getVideoMPsGeometry().convertBitdepth(decodedBitDepthMP, gi.getGeometryNominal2dBitdepthMinus1() + 1, gi.getGeometryMSBAlignFlag());
    generateMissedPointsGeometryfromVideo( context, reconstructs );
    std::cout << " missed points geometry -> " << videoBitstreamMP.naluSize() << " B " << endl;
  }
  if ( sps.getEnhancedOccupancyMapForDepthFlag() && !gfp.getGeometryPatchBlockFilteringEnableFlag() ) {
    generateBlockToPatchFromOccupancyMap( context, context.getOccupancyPackingBlockSize() );
  } else {
    generateBlockToPatchFromBoundaryBox( context, context.getOccupancyPackingBlockSize() );
  }
  GeneratePointCloudParameters generatePointCloudParameters;
  generatePointCloudParameters.occupancyResolution_      = context.getOccupancyPackingBlockSize();
  generatePointCloudParameters.occupancyPrecision_       = context.getOccupancyPrecision();
  generatePointCloudParameters.postprocessSmoothing_     = params_.postprocessSmoothing_;
  generatePointCloudParameters.flagGeometrySmoothing_    = gfp.getGeometrySmoothingParamsPresentFlag();
  generatePointCloudParameters.gridSmoothing_            = gfp.getGeometrySmoothingEnabledFlag();
  generatePointCloudParameters.gridSize_                 = gfp.getGeometrySmoothingGridSizeMinus2() + 2;
  generatePointCloudParameters.neighborCountSmoothing_   = 64;
  generatePointCloudParameters.radius2Smoothing_         = 64;
  generatePointCloudParameters.radius2BoundaryDetection_ = 64;
  generatePointCloudParameters.thresholdSmoothing_       = gfp.getGeometrySmoothingThreshold();
  generatePointCloudParameters.pcmPointColorFormat_      = size_t (sps.getLosslessGeo444() != 0? COLOURFORMAT444 : COLOURFORMAT420);
  generatePointCloudParameters.nbThread_                 = params_.nbThread_;
  generatePointCloudParameters.absoluteD1_ =
      sps.getLayerCountMinus1() == 0 || sps.getLayerAbsoluteCodingEnabledFlag( 1 );
  generatePointCloudParameters.surfaceThickness_ = context[0].getSurfaceThickness();
  if ( ai.getAttributeParamsEnabledFlag() ) {
    generatePointCloudParameters.thresholdColorSmoothing_  = afp.getAttributeSmoothingThreshold( 0 );
    generatePointCloudParameters.gridColorSmoothing_       = afp.getAttributeSmoothingParamsPresentFlag( 0 );
    generatePointCloudParameters.cgridSize_                = afp.getAttributeSmoothingGridSizeMinus2( 0 ) + 2;
    generatePointCloudParameters.thresholdColorDifference_ = afp.getAttributeSmoothingThresholdAttributeDifference( 0 );
    generatePointCloudParameters.thresholdColorVariation_  = afp.getAttributeSmoothingThresholdAttributeVariation( 0 );
    generatePointCloudParameters.thresholdLocalEntropy_    = afp.getAttributeSmoothingLocalEntropyThreshold( 0 );
  } else {
    generatePointCloudParameters.thresholdColorSmoothing_  = 0.;
    generatePointCloudParameters.gridColorSmoothing_       = false;
    generatePointCloudParameters.cgridSize_                = 0;
    generatePointCloudParameters.thresholdColorDifference_ = 0;
    generatePointCloudParameters.thresholdColorVariation_  = 0;
    generatePointCloudParameters.thresholdLocalEntropy_    = 0;
  }
  generatePointCloudParameters.radius2ColorSmoothing_         = 64;
  generatePointCloudParameters.neighborCountColorSmoothing_   = 64;
  generatePointCloudParameters.flagColorSmoothing_            = afp.getAttributeSmoothingParamsPresentFlag( 0 );
  generatePointCloudParameters.thresholdLossyOM_              = (size_t)oi.getLossyOccupancyMapCompressionThreshold();
  generatePointCloudParameters.removeDuplicatePoints_         = sps.getRemoveDuplicatePointEnabledFlag();
  generatePointCloudParameters.pointLocalReconstruction_      = sps.getPointLocalReconstructionEnabledFlag();
  generatePointCloudParameters.layerCountMinus1_              = sps.getLayerCountMinus1();
  generatePointCloudParameters.singleLayerPixelInterleaving_  = sps.getPixelDeinterleavingFlag();
  generatePointCloudParameters.path_                          = path.str();
  generatePointCloudParameters.useAdditionalPointsPatch_      = sps.getPcmPatchEnabledFlag();
  generatePointCloudParameters.enhancedDeltaDepthCode_        = sps.getEnhancedOccupancyMapForDepthFlag();
  generatePointCloudParameters.EOMFixBitCount_                = sps.getEOMFixBitCount();
  generatePointCloudParameters.EOMTexturePatch_ =
      generatePointCloudParameters.enhancedDeltaDepthCode_ && sps.getEOMTexturePatch();
  generatePointCloudParameters.geometry3dCoordinatesBitdepth_ = gi.getGeometry3dCoordinatesBitdepthMinus1() + 1;
  generatePointCloudParameters.geometryBitDepth3D_            = gi.getGeometry3dCoordinatesBitdepthMinus1() + 1;
  generatePointCloudParameters.pbfEnableFlag_                 = gfp.getGeometryPatchBlockFilteringEnableFlag();
  generatePointCloudParameters.pbfPassesCount_     = gfp.getGeometryPatchBlockFilteringPassesCountMinus1() + 1;
  generatePointCloudParameters.pbfFilterSize_      = gfp.getGeometryPatchBlockFilteringFilterSizeMinus1() + 1;
  generatePointCloudParameters.pbfLog2Threshold_   = gfp.getGeometryPatchBlockFilteringLog2ThresholdMinus1() + 1;
  generatePointCloudParameters.updateOccupancyMap_ = false;

  std::vector<std::vector<uint32_t>> partitions;
  generatePointCloud( reconstructs, context, generatePointCloudParameters, partitions );
  
  if ( ai.getAttributeCount() > 0 ) {
		int decodedBitdepthAttribute = ai.getAttributeNominal2dBitdepthMinus1(0) + 1;
    auto& videoBitstream = context.getVideoBitstream( VIDEO_TEXTURE );
    videoDecoder.decompress( context.getVideoTexture(), path.str(), context.size() * frameCountTexture, videoBitstream,
                             params_.videoDecoderPath_, context, decodedBitdepthAttribute,
                             params_.keepIntermediateFiles_,
                             sps.getLosslessGeo()!=0,
                             params_.patchColorSubsampling_, params_.inverseColorSpaceConversionConfig_,
                             params_.colorSpaceConversionPath_ );
		context.getVideoTexture().convertBitdepth(decodedBitdepthAttribute, ai.getAttributeNominal2dBitdepthMinus1( 0 ) + 1, ai.getAttributeMSBAlignFlag( 0 ));
    std::cout << "texture video  ->" << videoBitstream.naluSize() << " B" << std::endl;

    if ( sps.getPcmPatchEnabledFlag() && sps.getPcmSeparateVideoPresentFlag() ) {
		  int decodedBitdepthAttributeMP = ai.getAttributeNominal2dBitdepthMinus1( 0 ) + 1;
      auto& videoBitstreamMP = context.getVideoBitstream( VIDEO_TEXTURE_MP );
      videoDecoder.decompress( context.getVideoMPsTexture(), path.str(), context.size(), videoBitstreamMP,
                               params_.videoDecoderPath_, context, decodedBitdepthAttributeMP,
                               params_.keepIntermediateFiles_,
                               sps.getLosslessGeo(),
                               false,
                               params_.inverseColorSpaceConversionConfig_, params_.colorSpaceConversionPath_ );
		  context.getVideoTexture().convertBitdepth(decodedBitdepthAttributeMP, ai.getAttributeNominal2dBitdepthMinus1( 0 ) + 1, ai.getAttributeMSBAlignFlag( 0 ));
      printf( "call generateMissedPointsTexturefromVideo \n" );
      generateMissedPointsTexturefromVideo( context, reconstructs );
      std::cout << " missed points texture -> " << videoBitstreamMP.naluSize() << " B" << endl;
    }
  }
  colorPointCloud( reconstructs, context, ai.getAttributeCount(), params_.colorTransform_,
                   generatePointCloudParameters );

  //  Generate a buffer to keep unsmoothed geometry, then do geometry smoothing and transfer followed by color smoothing
  if ( generatePointCloudParameters.postprocessSmoothing_ ) {
    PCCGroupOfFrames tempFrameBuffer;
    auto&     frames = context.getFrames();
    tempFrameBuffer.resize( reconstructs.size() );
    for (size_t i = 0; i < frames.size(); i++) {
      tempFrameBuffer[i] = reconstructs[i];
    }
    smoothPointCloudPostprocess( reconstructs, context, params_.colorTransform_, generatePointCloudParameters, partitions );
    for (size_t i = 0; i < frames.size(); i++) {
      tempFrameBuffer[i].transferColors( reconstructs[i], int32_t( 0 ),
                                        sps.getLosslessGeo() == 1, 8, 1, 1, 1, 1, 0, 4, 4, 1000, 1000, 1000, 1000 ); //jkie: make it general
      //These are different attribute transfer functions
      //tempFrameBuffer[i].transferColorWeight( reconstructs[i], 0.1);
      //tempFrameBuffer[i].transferColors     ( reconstructs[i], int32_t( 0 ), sps.getLosslessGeo() == 1 );
    }
    //    This function does the color smoothing that is usually done in colorPointCloud
    colorSmoothing(reconstructs, context, params_.colorTransform_, generatePointCloudParameters);
  }

#ifdef CODEC_TRACE
  setTrace( false );
  closeTrace();
#endif
  return 0;
}

void PCCDecoder::setFrameMetadata( PCCMetadata& metadata, PatchFrameGeometryParameterSet& gfps ) {
  auto& gfp                                 = gfps.getGeometryFrameParams();
  auto& metadataEnabingFlags                = metadata.getMetadataEnabledFlags();
  metadataEnabingFlags.getMetadataEnabled() = gfps.getGeometryPatchParamsEnabledFlag();
  metadataEnabingFlags.getMetadataEnabled() = gfps.getGeometryPatchParamsEnabledFlag();

  if ( gfps.getGeometryPatchParamsEnabledFlag() ) {
    // Scale
    metadataEnabingFlags.getScaleEnabled() = gfps.getGeometryPatchScaleParamsEnabledFlag();
    if ( gfps.getGeometryPatchScaleParamsEnabledFlag() ) {
      metadataEnabingFlags.getScaleEnabled() = gfp.getGeometryScaleParamsPresentFlag();
      if ( gfp.getGeometryScaleParamsPresentFlag() ) {
        metadata.getScale()[0] = gfp.getGeometryScaleOnAxis( 0 );
        metadata.getScale()[1] = gfp.getGeometryScaleOnAxis( 1 );
        metadata.getScale()[2] = gfp.getGeometryScaleOnAxis( 2 );
      }
    }

    // Offset
    metadataEnabingFlags.getOffsetEnabled() = gfps.getGeometryPatchOffsetParamsEnabledFlag();
    if ( gfps.getGeometryPatchOffsetParamsEnabledFlag() ) {
      metadataEnabingFlags.getOffsetEnabled() = gfp.getGeometryOffsetParamsPresentFlag();
      if ( gfp.getGeometryOffsetParamsPresentFlag() ) {
        metadata.getOffset()[0] = gfp.getGeometryOffsetOnAxis( 0 );
        metadata.getOffset()[1] = gfp.getGeometryOffsetOnAxis( 1 );
        metadata.getOffset()[2] = gfp.getGeometryOffsetOnAxis( 2 );
      }
    }

    // Rotation
    metadataEnabingFlags.getRotationEnabled() = gfps.getGeometryPatchRotationParamsEnabledFlag();
    if ( gfps.getGeometryPatchRotationParamsEnabledFlag() ) {
      metadataEnabingFlags.getRotationEnabled() = gfp.getGeometryRotationParamsPresentFlag();
      if ( gfp.getGeometryRotationParamsPresentFlag() ) {
        metadata.getRotation()[0] = gfp.getGeometryRotationQuaternion( 0 );
        metadata.getRotation()[1] = gfp.getGeometryRotationQuaternion( 1 );
        metadata.getRotation()[2] = gfp.getGeometryRotationQuaternion( 2 );
        // metadata.getRotation()[3] = gfp.getGeometryRotationQuaternion( 3 );
      }
    }

    // Point size
    metadataEnabingFlags.getPointSizeEnabled() = gfps.getGeometryPatchPointSizeInfoEnabledFlag();
    if ( gfps.getGeometryPatchPointSizeInfoEnabledFlag() ) {
      metadata.getPointSizePresent() = gfp.getGeometryPointShapeInfoPresentFlag();
      if ( gfp.getGeometryPointShapeInfoPresentFlag() ) { metadata.getPointSize() = gfp.getGeometryPointSizeInfo(); }
    }

    // Point shape
    metadataEnabingFlags.getPointShapeEnabled() = gfps.getGeometryPatchPointShapeInfoEnabledFlag();
    if ( gfps.getGeometryPatchPointShapeInfoEnabledFlag() ) {
      metadata.getPointShapePresent() = gfp.getGeometryPointShapeInfoPresentFlag();
      if ( gfp.getGeometryPointShapeInfoPresentFlag() ) {
        metadata.getPointShape() = static_cast<PointShape>( gfp.getGeometryPointSizeInfo() );
      }
    }
  }
}

void PCCDecoder::setPatchMetadata( PCCMetadata& metadata, GeometryPatchParameterSet& gpps ) {
  auto& gpp                  = gpps.getGeometryPatchParams();
  auto& metadataEnabingFlags = metadata.getMetadataEnabledFlags();

  metadataEnabingFlags.getMetadataEnabled() = gpps.getGeometryPatchParamsPresentFlag();
  if ( gpps.getGeometryPatchParamsPresentFlag() ) {
    // Scale
    metadataEnabingFlags.getScaleEnabled() = gpp.getGeometryPatchScaleParamsPresentFlag();
    if ( gpp.getGeometryPatchScaleParamsPresentFlag() ) {
      metadata.getScale()[0] = gpp.getGeometryPatchScaleOnAxis( 0 );
      metadata.getScale()[1] = gpp.getGeometryPatchScaleOnAxis( 1 );
      metadata.getScale()[2] = gpp.getGeometryPatchScaleOnAxis( 2 );
    }
    // Offset
    metadataEnabingFlags.getOffsetEnabled() = gpp.getGeometryPatchOffsetParamsPresentFlag();
    if ( gpp.getGeometryPatchOffsetParamsPresentFlag() ) {
      metadata.getOffset()[0] = gpp.getGeometryPatchOffsetOnAxis( 0 );
      metadata.getOffset()[1] = gpp.getGeometryPatchOffsetOnAxis( 1 );
      metadata.getOffset()[2] = gpp.getGeometryPatchOffsetOnAxis( 2 );
    }
    // Rotation
    metadataEnabingFlags.getRotationEnabled() = gpp.getGeometryPatchRotationParamsPresentFlag();
    if ( gpp.getGeometryPatchRotationParamsPresentFlag() ) {
      metadata.getRotation()[0] = gpp.getGeometryPatchRotationQuaternion( 0 );
      metadata.getRotation()[1] = gpp.getGeometryPatchRotationQuaternion( 1 );
      metadata.getRotation()[2] = gpp.getGeometryPatchRotationQuaternion( 2 );
      // metadata.getRotation()[3] = gpp.getGeometryPatchRotationQuaternion( 3 );
    }
    // Point size
    metadataEnabingFlags.getPointSizeEnabled() = gpp.getGeometryPatchPointSizeInfoPresentFlag();
    if ( gpp.getGeometryPatchPointSizeInfoPresentFlag() ) {
      metadata.getPointSize() = gpp.getGeometryPatchPointSizeInfo();
    }
    // Point shape
    metadataEnabingFlags.getPointShapeEnabled() = gpp.getGeometryPatchPointShapeInfoPresentFlag();
    if ( gpp.getGeometryPatchPointShapeInfoPresentFlag() ) {
      metadata.getPointShape() = static_cast<PointShape>( gpp.getGeometryPatchPointShapeInfo() );
    }
  }
}

void PCCDecoder::setPointLocalReconstruction( PCCContext& context, SequenceParameterSet& sps ) {
  auto&                        plri = sps.getPointLocalReconstructionInformation();
  PointLocalReconstructionMode mode = {0, 0, 0, 1};
  context.addPointLocalReconstructionMode( mode );
  for ( size_t i = 0; i < plri.getPlrlNumberOfModesMinus1(); i++ ) {
    mode.interpolate_ = plri.getPlrlInterpolateFlag( i );
    mode.filling_     = plri.getPlrlFillingFlag( i );
    mode.minD1_       = plri.getPlrlMinimumDepth( i );
    mode.neighbor_    = plri.getPlrlNeighbourMinus1( i ) + 1;
    context.addPointLocalReconstructionMode( mode );
  }
#ifdef CODEC_TRACE
  for ( size_t i = 0; i < context.getPointLocalReconstructionModeNumber(); i++ ) {
    auto& mode = context.getPointLocalReconstructionMode( i );
    TRACE_CODEC( "Plrm[%u]: Inter = %d Fill = %d minD1 = %u neighbor = %u \n", i, mode.interpolate_, mode.filling_,
                 mode.minD1_, mode.neighbor_ );
  }
#endif
}

void PCCDecoder::setPointLocalReconstructionData( PCCFrameContext&              frame,
                                                  PCCPatch&                     patch,
                                                  PointLocalReconstructionData& plrd,
                                                  size_t                        occupancyPackingBlockSize ) {
  patch.allocOneLayerData();
  TRACE_CODEC( "WxH = %lu x %lu \n", plrd.getPlrBlockToPatchMapWidth(), plrd.getPlrBlockToPatchMapHeight() );
  patch.getPointLocalReconstructionLevel() = plrd.getPlrLevelFlag();
  TRACE_CODEC( "  LevelFlag = %d \n", plrd.getPlrLevelFlag() );
  if ( plrd.getPlrLevelFlag() ) {
    if ( plrd.getPlrPresentFlag() ) {
      patch.getPointLocalReconstructionMode() = plrd.getPlrModeMinus1() + 1;
    } else {
      patch.getPointLocalReconstructionMode() = 0;
    }
    TRACE_CODEC( "  ModePatch: Present = %d ModeMinus1 = %2d \n", plrd.getPlrPresentFlag(),
                 plrd.getPlrPresentFlag() ? (int32_t)plrd.getPlrModeMinus1() : -1 );
  } else {
    for ( size_t v0 = 0; v0 < plrd.getPlrBlockToPatchMapHeight(); ++v0 ) {
      for ( size_t u0 = 0; u0 < plrd.getPlrBlockToPatchMapWidth(); ++u0 ) {
        size_t index = v0 * plrd.getPlrBlockToPatchMapWidth() + u0;
        if ( plrd.getPlrBlockPresentFlag( index ) ) {
          patch.getPointLocalReconstructionMode( u0, v0 ) = plrd.getPlrBlockModeMinus1( index ) + 1;
        } else {
          patch.getPointLocalReconstructionMode( u0, v0 ) = 0;
        }
        TRACE_CODEC( "  Mode[%3u]: Present = %d ModeMinus1 = %2d \n", index, plrd.getPlrBlockPresentFlag( index ),
                     plrd.getPlrBlockPresentFlag( index ) ? (int32_t)plrd.getPlrBlockModeMinus1( index ) : -1 );
      }
    }
  }
#ifdef CODEC_TRACE
  for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
    for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
      TRACE_CODEC( "Block[ %2lu %2lu <=> %4lu ] / [ %2lu %2lu ]: Level = %d Present = %d mode = %lu \n", u0, v0,
                   v0 * patch.getSizeU0() + u0, patch.getSizeU0(), patch.getSizeV0(),
                   patch.getPointLocalReconstructionLevel(), plrd.getPlrBlockPresentFlag( v0 * patch.getSizeU0() + u0 ),
                   patch.getPointLocalReconstructionMode( u0, v0 ) );
    }
  }
#endif
}

void PCCDecoder::createPatchFrameDataStructure( PCCContext& context ) {
  TRACE_CODEC( "createPatchFrameDataStructure GOP start \n" );
  auto& sps  = context.getSps();
  auto& pdg  = context.getPatchDataGroup();
  auto& psps = pdg.getPatchSequenceParameterSet( 0 );
  context.setOccupancyPackingBlockSize( pow( 2, psps.getLog2PatchPackingBlockSize() ) );
  context.resize( pdg.getPatchTileGroupLayerUnitSize() );
  TRACE_CODEC( "frameCount = %u \n", pdg.getPatchTileGroupLayerUnitSize() );
  setFrameMetadata( context.getGOFLevelMetadata(), pdg.getPatchFrameGeometryParameterSet( 0 ) );
  setPointLocalReconstruction( context, sps );
  size_t indexPrevFrame = 0;
  context.setMPGeoWidth( 64 );
  context.setMPAttWidth( 0 );
  context.setMPGeoHeight( 0 );
  context.setMPAttHeight( 0 );
  for ( int i = 0; i < pdg.getPatchTileGroupLayerUnitSize(); i++ ) {
    auto& frame                          = context.getFrame( i );
    auto& frameLevelMetadataEnabledFlags = context.getGOFLevelMetadata().getLowerLevelMetadataEnabledFlags();
    frame.setIndex( i );
    frame.setWidth( sps.getFrameWidth() );
    frame.setHeight( sps.getFrameHeight() );
    frame.getFrameLevelMetadata().getMetadataEnabledFlags() = frameLevelMetadataEnabledFlags;
    frame.setLosslessGeo( sps.getLosslessGeo() );
    frame.setLosslessGeo444( sps.getLosslessGeo444() );
    frame.setSurfaceThickness( sps.getSurfaceThickness() );
    frame.setUseMissedPointsSeparateVideo( sps.getPcmSeparateVideoPresentFlag() );
    frame.setPcmPatchEnabledFlag( sps.getPcmPatchEnabledFlag() );
    createPatchFrameDataStructure( context, frame, context.getFrame( indexPrevFrame ), i );
    indexPrevFrame = i;
  }
}

void PCCDecoder::createPatchFrameDataStructure( PCCContext&      context,
                                                PCCFrameContext& frame,
                                                PCCFrameContext& preFrame,
                                                size_t           frameIndex ) {
  TRACE_CODEC( "createPatchFrameDataStructure Frame %lu \n", frame.getIndex() );
  auto&             sps            = context.getSps();
  auto&             gi             = context.getSps().getGeometryInformation();
  auto&             pdg            = context.getPatchDataGroup();
  auto&             ptglu          = pdg.getPatchTileGroupLayerUnit( frameIndex );
  auto&             ptgh           = ptglu.getPatchTileGroupHeader();
  auto&             ptgdu          = ptglu.getPatchTileGroupDataUnit();
  auto&             pfps           = pdg.getPatchFrameParameterSet( 0 );
  auto&             patches        = frame.getPatches();
  auto&             prePatches     = preFrame.getPatches();
  auto&             pcmPatches     = frame.getMissedPointsPatches();
  auto&             eomPatches     = frame.getEomPatches();
  int64_t           prevSizeU0     = 0;
  int64_t           prevSizeV0     = 0;
  int64_t           predIndex      = 0;
  const size_t      minLevel       = sps.getMinLevel();
  size_t            numPCMPatches  = 0;
  size_t            numNonPCMPatch = 0;
  size_t numEomPatch=0;
  PCCPatchFrameType patchFrameType = PCCPatchFrameType( ptgh.getType() );
  size_t            patchCount     = ptgdu.getPatchCount();
  for ( size_t i = 0; i < patchCount; i++ ) {
    if ( ( patchFrameType == PATCH_FRAME_I && PCCPatchModeI( ptgdu.getPatchMode( i ) ) == PATCH_MODE_I_PCM ) ||
         ( patchFrameType == PATCH_FRAME_P && PCCPatchModeP( ptgdu.getPatchMode( i ) ) == PATCH_MODE_P_PCM ) )
      numPCMPatches++;
    else if(( patchFrameType == PATCH_FRAME_I && PCCPatchModeI( ptgdu.getPatchMode( i ) ) == PATCH_MODE_I_EOM ) ||
            ( patchFrameType == PATCH_FRAME_P && PCCPatchModeP( ptgdu.getPatchMode( i ) ) == PATCH_MODE_P_EOM ) ){
      numEomPatch++;
    }
  }
  numNonPCMPatch = patchCount - numPCMPatches - numEomPatch;
  printf("numNonPCMPatch = %zu \n",numNonPCMPatch);
  printf("sps.getEOMTexturePatch() = %d \n",sps.getEOMTexturePatch());
  eomPatches.reserve(numEomPatch);
  patches.resize( numNonPCMPatch );
  pcmPatches.resize( numPCMPatches );
  frame.getFrameLevelMetadata().setMetadataType( METADATA_FRAME );
  frame.getFrameLevelMetadata().setIndex( frame.getIndex() );

  TRACE_CODEC( "Patches size                        = %lu \n", patches.size() );
  TRACE_CODEC( "non-regular Patches(pcm, eom)     = %lu, %lu \n",
              frame.getMissedPointsPatches().size(),
              frame.getEomPatches().size() );
  TRACE_CODEC( "OccupancyPackingBlockSize           = %d \n", context.getOccupancyPackingBlockSize() );
  TRACE_CODEC( "PatchInterPredictionEnabledFlag     = %d \n", sps.getPatchInterPredictionEnabledFlag() );
  size_t totalNumberOfMps = 0;
  size_t patchIndex       = 0;
  for ( patchIndex = 0; patchIndex < patchCount; patchIndex++ ) {
    auto& pid = ptgdu.getPatchInformationData( patchIndex );
    if ( ( ( patchFrameType == PATCH_FRAME_I ) &&
           ( ptgdu.getPatchMode( patchIndex ) == (uint8_t)PATCH_MODE_I_INTRA ) ) ||
         ( ( patchFrameType == PATCH_FRAME_P ) &&
           ( ptgdu.getPatchMode( patchIndex ) == (uint8_t)PATCH_MODE_P_INTRA ) ) ) {
      auto& patch                    = patches[patchIndex];
      patch.getOccupancyResolution() = context.getOccupancyPackingBlockSize();
      auto& pdu                      = pid.getPatchDataUnit();
      patch.getU0()                  = pdu.get2DShiftU();
      patch.getV0()                  = pdu.get2DShiftV();
      patch.getU1()                  = pdu.get3DShiftTangentAxis();
      patch.getV1()                  = pdu.get3DShiftBiTangentAxis();

      bool lodEnableFlag = pdu.getLodEnableFlag();
      if(lodEnableFlag) {
        //PatchLoDScaleX[ p ] = pdu_lod_enable_flag[ p ] ? pdu_lod_scale_x_minus1[ p ] + 1: 1
        //PatchLoDScaleY[ p ] = pdu_lod_enable_flag[ p ] ? (pdu_lod_scale_y[ p ] + (pdu_lod_scale_x_minus1[ p ] > 0) ? 1 : 2) : 1
        patch.setLodScaleX( pdu.getLodScaleXminus1() + 1);
        patch.setLodScaleY( pdu.getLodScaleY() + (patch.getLodScaleX()>1?1:2) );
      }
      else{
        patch.setLodScaleX(1);
        patch.setLodScaleY(1);
      }
      patch.getSizeD()               = ( std::min )( pdu.get3DShiftDeltaMaxNormalAxis() * minLevel, (size_t)255 );
      patch.getSizeU0()              = prevSizeU0 + pdu.get2DDeltaSizeU();
      patch.getSizeV0()              = prevSizeV0 + pdu.get2DDeltaSizeV();
      patch.getNormalAxis()          = size_t( pdu.getProjectPlane() ) % 3;
      patch.getProjectionMode()      = size_t( pdu.getProjectPlane() ) < 3 ? 0 : 1;
      patch.getPatchOrientation()    = pdu.getOrientationIndex();
      patch.getAxisOfAdditionalPlane() =
          pdu.get45DegreeProjectionPresentFlag() ? pdu.get45DegreeProjectionRotationAxis() : 0;
      TRACE_CODEC( "patch %lu / %lu: Intra \n", patchIndex, patches.size() );
      const size_t max3DCoordinate = 1 << ( gi.getGeometry3dCoordinatesBitdepthMinus1() + 1 );
      if ( patch.getProjectionMode() == 0 ||
           ( sps.getLayerCountMinus1() > 0 && !sps.getLayerAbsoluteCodingEnabledFlag( 1 ) ) ) {
        patch.getD1() = (int32_t)pdu.get3DShiftMinNormalAxis() * minLevel;
      } else {
        if ( pfps.getProjection45DegreeEnableFlag() == 0 ) {
          patch.getD1() = max3DCoordinate - (int32_t)pdu.get3DShiftMinNormalAxis() * minLevel;
        } else {
          patch.getD1() = ( max3DCoordinate << 1 ) - (int32_t)pdu.get3DShiftMinNormalAxis() * minLevel;
        }
      }
      prevSizeU0     = patch.getSizeU0();
      prevSizeV0     = patch.getSizeV0();

      if ( patch.getNormalAxis() == 0 ) {
        patch.getTangentAxis()   = 2;
        patch.getBitangentAxis() = 1;
      } else if ( patch.getNormalAxis() == 1 ) {
        patch.getTangentAxis()   = 2;
        patch.getBitangentAxis() = 0;
      } else {
        patch.getTangentAxis()   = 0;
        patch.getBitangentAxis() = 1;
      }
      TRACE_CODEC( "patch UV0 %4lu %4lu UV1 %4lu %4lu D1=%4lu S=%4lu %4lu %4lu(%4lu) P=%lu O=%lu A=%u%u%u Lod =(%zu) %lu,%lu \n",
                   patch.getU0(), patch.getV0(), patch.getU1(), patch.getV1(), patch.getD1(), patch.getSizeU0(),
                   patch.getSizeV0(), patch.getSizeD(), pdu.get3DShiftDeltaMaxNormalAxis(), patch.getProjectionMode(),
                   patch.getPatchOrientation(), patch.getNormalAxis(), patch.getTangentAxis(), patch.getBitangentAxis(),
                   (size_t)lodEnableFlag, patch.getLodScaleX(), patch.getLodScaleY() );
      auto& patchLevelMetadataEnabledFlags = frame.getFrameLevelMetadata().getLowerLevelMetadataEnabledFlags();
      auto& metadata                       = patch.getPatchLevelMetadata();
      metadata.setIndex( patchIndex );
      metadata.setMetadataType( METADATA_PATCH );
      metadata.getMetadataEnabledFlags() = patchLevelMetadataEnabledFlags;
      setPatchMetadata( metadata, pdg.getGeometryPatchParameterSet( 0 ) );
      patch.allocOneLayerData();
      if ( sps.getPointLocalReconstructionEnabledFlag() ) {
        setPointLocalReconstructionData( frame, patch, pdu.getPointLocalReconstructionData(),
                                         context.getOccupancyPackingBlockSize() );
      }
    } else if ( ( patchFrameType == PATCH_FRAME_P &&
                  PCCPatchModeP( ptgdu.getPatchMode( patchIndex ) ) == PATCH_MODE_P_INTER ) ) {
      auto& patch                    = patches[patchIndex];
      patch.getOccupancyResolution() = context.getOccupancyPackingBlockSize();
      auto& dpdu                     = pid.getDeltaPatchDataUnit();
      patch.setBestMatchIdx( ( int32_t )( dpdu.getDeltaPatchIdx() + predIndex ) );
      TRACE_CODEC( "patch %lu / %lu: Inter \n", patchIndex, patches.size() );

      TRACE_CODEC(
          "DPDU:DeltaIdx = %d ShiftUV = %ld %ld ShiftAxis = %ld %ld %ld Size = %ld %ld Idx = %ld + %ld = %zu \n",
          dpdu.getDeltaPatchIdx(), dpdu.get2DDeltaShiftU(), dpdu.get2DDeltaShiftV(), dpdu.get3DDeltaShiftTangentAxis(),
          dpdu.get3DDeltaShiftBiTangentAxis(), dpdu.get3DDeltaShiftMinNormalAxis(), dpdu.get2DDeltaSizeU(),
          dpdu.get2DDeltaSizeV(), dpdu.getDeltaPatchIdx(), predIndex, (size_t)patch.getBestMatchIdx() );

      predIndex += dpdu.getDeltaPatchIdx() + 1;
      const auto& prePatch = prePatches[patch.getBestMatchIdx()];

      TRACE_CODEC( "PrevPatch: Idx = %lu UV0 = %lu %lu  UV1 = %lu %lu Size = %lu %lu %lu  Lod = %u,%u\n", patch.getBestMatchIdx(),
                   prePatch.getU0(), prePatch.getV0(), prePatch.getU1(), prePatch.getV1(), prePatch.getSizeU0(),
                   prePatch.getSizeV0(), prePatch.getSizeD(), prePatch.getLodScaleX(), prePatch.getLodScaleY() );

      patch.getProjectionMode()        = prePatch.getProjectionMode();
      patch.getU0()                    = dpdu.get2DDeltaShiftU() + prePatch.getU0();
      patch.getV0()                    = dpdu.get2DDeltaShiftV() + prePatch.getV0();
      patch.getPatchOrientation()      = prePatch.getPatchOrientation();
      patch.getU1()                    = dpdu.get3DDeltaShiftTangentAxis() + prePatch.getU1();
      patch.getV1()                    = dpdu.get3DDeltaShiftBiTangentAxis() + prePatch.getV1();
      patch.getSizeU0()                = dpdu.get2DDeltaSizeU() + prePatch.getSizeU0();
      patch.getSizeV0()                = dpdu.get2DDeltaSizeV() + prePatch.getSizeV0();
      patch.getNormalAxis()            = prePatch.getNormalAxis();
      patch.getTangentAxis()           = prePatch.getTangentAxis();
      patch.getBitangentAxis()         = prePatch.getBitangentAxis();
      patch.getAxisOfAdditionalPlane() = prePatch.getAxisOfAdditionalPlane();
      const size_t max3DCoordinate     = 1 << ( gi.getGeometry3dCoordinatesBitdepthMinus1() + 1 );
      if ( patch.getProjectionMode() == 0 ||
           ( sps.getLayerCountMinus1() > 0 && !sps.getLayerAbsoluteCodingEnabledFlag( 1 ) ) ) {
        patch.getD1() = ( dpdu.get3DDeltaShiftMinNormalAxis() + ( prePatch.getD1() / minLevel ) ) * minLevel;
      } else {
        if ( pfps.getProjection45DegreeEnableFlag() == 0 ) {
          patch.getD1() = max3DCoordinate - ( dpdu.get3DDeltaShiftMinNormalAxis() +
                                              ( ( max3DCoordinate - prePatch.getD1() ) / minLevel ) ) *
                                                minLevel;
        } else {
          patch.getD1() =
              ( max3DCoordinate << 1 ) -
              ( dpdu.get3DDeltaShiftMinNormalAxis() + ( ( ( max3DCoordinate << 1 ) - prePatch.getD1() ) / minLevel ) ) *
                  minLevel;
        }
      }
      const int64_t delta_DD = dpdu.get3DShiftDeltaMaxNormalAxis();
      size_t        prevDD   = prePatch.getSizeD() / minLevel;
      if ( prevDD * minLevel != prePatch.getSizeD() ) { prevDD += 1; }
      patch.getSizeD() = ( std::min )( size_t( ( delta_DD + prevDD ) * minLevel ), (size_t)255 );
      patch.setLodScaleX(prePatch.getLodScaleX());
      patch.setLodScaleY(prePatch.getLodScaleY());
      prevSizeU0       = patch.getSizeU0();
      prevSizeV0       = patch.getSizeV0();

      TRACE_CODEC(
          "patch Inter UV0 %4lu %4lu UV1 %4lu %4lu D1=%4lu S=%4lu %4lu %4lu from DeltaSize = "
          "%4ld %4ld P=%lu O=%lu A=%u%u%u Lod = %lu,%lu \n",
          patch.getU0(), patch.getV0(), patch.getU1(), patch.getV1(), patch.getD1(), patch.getSizeU0(),
          patch.getSizeV0(), patch.getSizeD(), dpdu.get2DDeltaSizeU(), dpdu.get2DDeltaSizeV(),
          patch.getProjectionMode(), patch.getPatchOrientation(), patch.getNormalAxis(), patch.getTangentAxis(),
          patch.getBitangentAxis(), patch.getLodScaleX(),patch.getLodScaleY() );
      auto& patchLevelMetadataEnabledFlags         = frame.getFrameLevelMetadata().getLowerLevelMetadataEnabledFlags();
      auto& patchLevelMetadata                     = patch.getPatchLevelMetadata();
      patchLevelMetadata.getMetadataEnabledFlags() = patchLevelMetadataEnabledFlags;
      patchLevelMetadata.setIndex( patchIndex );
      patchLevelMetadata.setMetadataType( METADATA_PATCH );
      setPatchMetadata( patchLevelMetadata, pdg.getGeometryPatchParameterSet( 0 ) );
      patch.allocOneLayerData();
      if ( sps.getPointLocalReconstructionEnabledFlag() ) {
        setPointLocalReconstructionData( frame, patch, dpdu.getPointLocalReconstructionData(),
                                         context.getOccupancyPackingBlockSize() );
      }
    } else if ( ( PCCPatchFrameType( ptgh.getType() ) == PATCH_FRAME_I &&
                  PCCPatchModeI( ptgdu.getPatchMode( patchIndex ) ) == (uint8_t)PATCH_MODE_I_PCM ) ||
                ( PCCPatchFrameType( ptgh.getType() ) == PATCH_FRAME_P &&
                  PCCPatchModeP( ptgdu.getPatchMode( patchIndex ) ) == (uint8_t)PATCH_MODE_P_PCM ) ) {
      TRACE_CODEC( "patch %lu / %lu: PCM \n", patchIndex, patches.size() );

      auto& ppdu                = pid.getPCMPatchDataUnit();
      auto& missedPointsPatch   = pcmPatches[patchIndex - numNonPCMPatch];
      missedPointsPatch.u0_     = ppdu.get2DShiftU();
      missedPointsPatch.v0_     = ppdu.get2DShiftV();
      missedPointsPatch.sizeU0_ = ppdu.get2DDeltaSizeU();
      missedPointsPatch.sizeV0_ = ppdu.get2DDeltaSizeV();
      if ( ptgh.getPcm3dShiftBitCountPresentFlag() ) {
        missedPointsPatch.u1_ = ppdu.get3DShiftTangentAxis();
        missedPointsPatch.v1_ = ppdu.get3DShiftBiTangentAxis();
        missedPointsPatch.d1_ = ppdu.get3DShiftNormalAxis();
      } else {
        const size_t pcmU1V1D1Level = 1 << ( gi.getGeometryNominal2dBitdepthMinus1() );
        missedPointsPatch.u1_       = ppdu.get3DShiftTangentAxis() * pcmU1V1D1Level;
        missedPointsPatch.v1_       = ppdu.get3DShiftBiTangentAxis() * pcmU1V1D1Level;
        missedPointsPatch.d1_       = ppdu.get3DShiftNormalAxis() * pcmU1V1D1Level;
      }
      missedPointsPatch.setNumberOfMps( ppdu.getPcmPoints() );
      missedPointsPatch.occupancyResolution_ = context.getOccupancyPackingBlockSize();
      totalNumberOfMps += missedPointsPatch.getNumberOfMps();
      TRACE_CODEC( "PCM :UV = %lu %lu  size = %lu %lu  uvd1 = %lu %lu %lu numPoints = %lu ocmRes = %lu \n",
                   missedPointsPatch.u0_, missedPointsPatch.v0_, missedPointsPatch.sizeU0_, missedPointsPatch.sizeV0_,
                   missedPointsPatch.u1_, missedPointsPatch.v1_, missedPointsPatch.d1_, missedPointsPatch.numberOfMps_,
                   missedPointsPatch.occupancyResolution_ );
    } else if ( ( PCCPatchFrameType( ptgh.getType() ) == PATCH_FRAME_I &&
                  PCCPatchModeI( ptgdu.getPatchMode( patchIndex ) ) == (uint8_t)PATCH_MODE_I_EOM ) ||
                ( PCCPatchFrameType( ptgh.getType() ) == PATCH_FRAME_P &&
                  PCCPatchModeP( ptgdu.getPatchMode( patchIndex ) ) == (uint8_t)PATCH_MODE_P_EOM ) ) {
      TRACE_CODEC( "patch %lu / %lu: EOM \n", patchIndex, patchCount );
      auto& epdu            = pid.getEOMPatchDataUnit();
      auto& eomPatches        = frame.getEomPatches();
      PCCEomPatch eomPatch;
      eomPatch.u0_ =epdu.get2DShiftU();
      eomPatch.v0_ = epdu.get2DShiftV();
      eomPatch.sizeU_       = epdu.get2DDeltaSizeU();
      eomPatch.sizeV_       = epdu.get2DDeltaSizeV();
      eomPatch.memberPatches.resize(epdu.getEpduAssociatedPatchesCountMinus1()+1);
      eomPatch.eddCountPerPatch.resize(epdu.getEpduAssociatedPatchesCountMinus1()+1);
      eomPatch.eddCount_=0;
      for(size_t i=0; i<eomPatch.memberPatches.size();i++){
        eomPatch.memberPatches[i] = epdu.getEpduAssociatedPatches(i);
        eomPatch.eddCountPerPatch[i] = epdu.getEpduEomPointsPerPatch(i);
        eomPatch.eddCount_ += eomPatch.eddCountPerPatch[i];
      }
      eomPatches.push_back(eomPatch);
      TRACE_CODEC( "EOM: U0V0 %lu,%lu\tSizeU0V0 %lu,%lu\tN= %lu,%lu\n", eomPatch.u0_ , eomPatch.v0_,
                  eomPatch.sizeU_,eomPatch.sizeV_, eomPatch.memberPatches.size(), eomPatch.eddCount_);
      for(size_t i=0; i<eomPatch.memberPatches.size();i++){
        TRACE_CODEC( "%lu, %lu\n", eomPatch.memberPatches[i], eomPatch.eddCountPerPatch[i] );}
      TRACE_CODEC( "\n");

    } else if ( ( PCCPatchFrameType( ptgh.getType() ) == PATCH_FRAME_I &&
                  PCCPatchModeP( ptgdu.getPatchMode( patchIndex ) ) == (uint8_t)PATCH_MODE_I_END ) ||
                ( PCCPatchFrameType( ptgh.getType() ) == PATCH_FRAME_P &&
                  PCCPatchModeP( ptgdu.getPatchMode( patchIndex ) ) == (uint8_t)PATCH_MODE_P_END ) ) {
      break;
    } else {
      printf( "Error: unknow frame/patch type \n" );
      TRACE_CODEC( "Error: unknow frame/patch type \n" );
    }
  }
  TRACE_CODEC( "patch %lu / %lu: end \n", patches.size(), patches.size() );
  frame.setTotalNumberOfMissedPoints( totalNumberOfMps );
}
