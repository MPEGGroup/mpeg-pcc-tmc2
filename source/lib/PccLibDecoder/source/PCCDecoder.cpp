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
#include "PCCBitstreamDecoderNewSyntax.h"
#include <tbb/tbb.h>
#include "PCCDecoder.h"

using namespace pcc;
using namespace std;

PCCDecoder::PCCDecoder() {}
PCCDecoder::~PCCDecoder() {}

void PCCDecoder::setParameters( PCCDecoderParameters params ) { params_ = params; }

int PCCDecoder::decode( PCCBitstream& bitstream, PCCContext& context, PCCGroupOfFrames& reconstructs ) {
  int ret = 0;
  if ( params_.nbThread_ > 0 ) { tbb::task_scheduler_init init( (int)params_.nbThread_ ); }
#if 0
  PCCBitstreamDecoder bitstreamDecoder;
#else
  PCCBitstreamDecoderNewSyntax bitstreamDecoder;
#endif
#ifdef BITSTREAM_TRACE
  bitstream.setTrace( true );
  bitstream.openTrace( removeFileExtension( params_.compressedStreamPath_ ) + "_prev_syntax_decode.txt" );
#endif
  if ( !bitstreamDecoder.decode( bitstream, context ) ) { return 0; }
#ifdef BITSTREAM_TRACE
  bitstream.closeTrace();
#endif

  createPatchFrameDataStructure( context );

  ret |= decode( context, reconstructs );
  return ret;
}

int PCCDecoder::decode( PCCContext& context, PCCGroupOfFrames& reconstructs ) {
  printf( "decode start \n" );
  fflush( stdout );
#ifdef CODEC_TRACE
  setTrace( true );
  openTrace( removeFileExtension( params_.compressedStreamPath_ ) + "_codec_decode.txt" );
#endif
  reconstructs.resize( context.size() );
  PCCVideoDecoder   videoDecoder;
  std::stringstream path;
  auto&             sps = context.getSps();
  auto&             gps = sps.getGeometryParameterSet();
  auto&             gsp = gps.getGeometrySequenceParams();
  auto&             ops = sps.getOccupancyParameterSet();
  auto&             aps = sps.getAttributeParameterSet( 0 );
  auto&             asp = aps.getAttributeSequenceParams();
  path << removeFileExtension( params_.compressedStreamPath_ ) << "_dec_GOF" << sps.getSequenceParameterSetId() << "_";

  bool lossyMpp = !sps.getLosslessGeo() && sps.getPcmPatchEnabledFlag();
  // const size_t nbyteGeo =
  //     ( context.getLosslessGeo() || ( lossyMpp && !sps.getPcmSeparateVideoPresentFlag() ) )
  //         ? 2
  //         : 1;

  const size_t frameCountGeometry = sps.getMultipleLayerStreamsPresentFlag() ? 2 : 1;
  const size_t frameCountTexture  = sps.getMultipleLayerStreamsPresentFlag() ? 2 : 1;

  auto& videoBitstreamOM = context.getVideoBitstream( PCCVideoType::OccupancyMap );
  videoDecoder.decompress( context.getVideoOccupancyMap(), path.str(), context.size(), videoBitstreamOM, params_.videoDecoderOccupancyMapPath_,
                           context, params_.keepIntermediateFiles_, ( sps.getLosslessGeo() ? sps.getLosslessGeo444() : false ), false, "", "" );
  context.getOccupancyPrecision() = sps.getFrameWidth() / context.getVideoOccupancyMap().getWidth();
  printf( "compute OccupancyPrecision = %u \n", context.getOccupancyPrecision() );
  generateOccupancyMap( context, context.getOccupancyPrecision() );

  if ( !sps.getLayerAbsoluteCodingEnabledFlag( 1 ) ) {
    if ( lossyMpp ) {
      std::cout << "ERROR! Lossy-missed-points-patch code not implemented when absoluteD_ = 0 as of now. Exiting ..." << std::endl;
      std::exit( -1 );
    }
    // Compress D0
    auto& videoBitstreamD0 = context.getVideoBitstream( PCCVideoType::GeometryD0 );
    videoDecoder.decompress( context.getVideoGeometry(), path.str(), context.size(), videoBitstreamD0, params_.videoDecoderPath_, context,
                             params_.keepIntermediateFiles_, ( sps.getLosslessGeo() ? sps.getLosslessGeo444() : false ) );
    std::cout << "geometry D0 video ->" << videoBitstreamD0.naluSize() << " B" << std::endl;

    // Compress D1
    auto& videoBitstreamD1 = context.getVideoBitstream( PCCVideoType::GeometryD1 );
    videoDecoder.decompress( context.getVideoGeometryD1(), path.str(), context.size(), videoBitstreamD1, params_.videoDecoderPath_, context,
                             params_.keepIntermediateFiles_, ( sps.getLosslessGeo() ? sps.getLosslessGeo444() : false ) );
    std::cout << "geometry D1 video ->" << videoBitstreamD1.naluSize() << " B" << std::endl;

    std::cout << "geometry video ->" << videoBitstreamD1.naluSize() + videoBitstreamD1.naluSize() << " B" << std::endl;
  } else {
    auto& videoBitstream = context.getVideoBitstream( PCCVideoType::Geometry );
    videoDecoder.decompress( context.getVideoGeometry(), path.str(), context.size() * frameCountGeometry, videoBitstream, params_.videoDecoderPath_,
                             context, params_.keepIntermediateFiles_, sps.getLosslessGeo() & sps.getLosslessGeo444() );
    std::cout << "geometry video ->" << videoBitstream.naluSize() << " B" << std::endl;
  }

  if ( sps.getPcmPatchEnabledFlag() && sps.getPcmSeparateVideoPresentFlag() ) {
    auto& videoBitstreamMP = context.getVideoBitstream( PCCVideoType::GeometryMP );
    videoDecoder.decompress( context.getVideoMPsGeometry(), path.str(), context.size(), videoBitstreamMP, params_.videoDecoderPath_, context,
                             params_.keepIntermediateFiles_ );

    generateMissedPointsGeometryfromVideo( context, reconstructs );  // 0. geo : decode arithmetic coding part
    std::cout << " missed points geometry -> " << videoBitstreamMP.naluSize() << " B " << endl;

    // add missed point to reconstructs
    // fillMissedPoints(reconstructs, context, 0, params_.colorTransform_); //0. geo
  }
  bool useAdditionalPointsPatch = sps.getPcmPatchEnabledFlag();
  bool lossyMissedPointsPatch   = !sps.getLosslessGeo() && useAdditionalPointsPatch;
  if ( ( sps.getLosslessGeo() != 0 ) && sps.getEnhancedOccupancyMapForDepthFlag() ) {
    generateBlockToPatchFromOccupancyMap( context, sps.getLosslessGeo(), lossyMissedPointsPatch, 0, ops.getOccupancyPackingBlockSize() );
  } else {
    generateBlockToPatchFromBoundaryBox( context, sps.getLosslessGeo(), lossyMissedPointsPatch, 0, ops.getOccupancyPackingBlockSize() );
  }

  GeneratePointCloudParameters generatePointCloudParameters;
  generatePointCloudParameters.occupancyResolution_          = ops.getOccupancyPackingBlockSize();
  generatePointCloudParameters.occupancyPrecision_           = context.getOccupancyPrecision();
  generatePointCloudParameters.flagGeometrySmoothing_        = gsp.getGeometrySmoothingParamsPresentFlag();
  generatePointCloudParameters.gridSmoothing_                = gsp.getGeometrySmoothingEnabledFlag();
  generatePointCloudParameters.gridSize_                     = gsp.getGeometrySmoothingGridSize();
  generatePointCloudParameters.neighborCountSmoothing_       = asp.getAttributeSmoothingNeighbourCount();
  generatePointCloudParameters.radius2Smoothing_             = (double)asp.getAttributeSmoothingRadius();
  generatePointCloudParameters.radius2BoundaryDetection_     = (double)asp.getAttributeSmoothingRadius2BoundaryDetection();
  generatePointCloudParameters.thresholdSmoothing_           = (double)gsp.getGeometrySmoothingThreshold();
  generatePointCloudParameters.losslessGeo_                  = sps.getLosslessGeo() != 0;
  generatePointCloudParameters.losslessGeo444_               = sps.getLosslessGeo444() != 0;
  generatePointCloudParameters.nbThread_                     = params_.nbThread_;
  generatePointCloudParameters.absoluteD1_                   = sps.getLayerAbsoluteCodingEnabledFlag( 1 );
  generatePointCloudParameters.surfaceThickness              = context[0].getSurfaceThickness();
  generatePointCloudParameters.ignoreLod_                    = true;
  generatePointCloudParameters.thresholdColorSmoothing_      = (double)asp.getAttributeSmoothingThreshold();
  generatePointCloudParameters.thresholdLocalEntropy_        = (double)asp.getAttributeSmoothingThresholdLocalEntropy();
  generatePointCloudParameters.radius2ColorSmoothing_        = (double)asp.getAttributeSmoothingRadius();
  generatePointCloudParameters.neighborCountColorSmoothing_  = asp.getAttributeSmoothingNeighbourCount();
  generatePointCloudParameters.flagColorSmoothing_           = (bool)asp.getAttributeSmoothingParamsPresentFlag();
  generatePointCloudParameters.enhancedDeltaDepthCode_       = ( ( sps.getLosslessGeo() != 0 ) ? sps.getEnhancedOccupancyMapForDepthFlag() : false );
  generatePointCloudParameters.removeDuplicatePoints_        = sps.getRemoveDuplicatePointEnabledFlag();
  generatePointCloudParameters.oneLayerMode_                 = !sps.getMultipleLayerStreamsPresentFlag();
  generatePointCloudParameters.singleLayerPixelInterleaving_ = sps.getPixelDeinterleavingFlag();
  generatePointCloudParameters.path_                         = path.str();
  generatePointCloudParameters.useAdditionalPointsPatch_     = sps.getPcmPatchEnabledFlag();
  // generatePointCloudParameters.deltaCoding_                  = (params_.testLevelOfDetailSignaling_ > 0); // ignore LoD scaling for testing the
  // signaling only
  // generatePointCloudParameters.sixDirectionMode_             = context.getSixDirectionMode();
  // generatePointCloudParameters.improveEDD_                   = context.getImproveEDD();

  generatePointCloud( reconstructs, context, generatePointCloudParameters );

  if ( sps.getAttributeCount() > 0 ) {
    auto& videoBitstream = context.getVideoBitstream( PCCVideoType::Texture );
    videoDecoder.decompress( context.getVideoTexture(), path.str(), context.size() * frameCountTexture, videoBitstream, params_.videoDecoderPath_,
                             context, params_.keepIntermediateFiles_, sps.getLosslessTexture() != 0, params_.patchColorSubsampling_,
                             params_.inverseColorSpaceConversionConfig_, params_.colorSpaceConversionPath_ );
    std::cout << "texture video  ->" << videoBitstream.naluSize() << " B" << std::endl;

    if ( sps.getPcmPatchEnabledFlag() && sps.getPcmSeparateVideoPresentFlag() ) {
      auto& videoBitstreamMP = context.getVideoBitstream( PCCVideoType::TextureMP );
      videoDecoder.decompress( context.getVideoMPsTexture(), path.str(), context.size(), videoBitstreamMP, params_.videoDecoderPath_, context,
                               params_.keepIntermediateFiles_, sps.getLosslessTexture(), false, params_.inverseColorSpaceConversionConfig_,
                               params_.colorSpaceConversionPath_ );

      generateMissedPointsTexturefromVideo( context, reconstructs );
      std::cout << " missed points texture -> " << videoBitstreamMP.naluSize() << " B" << endl;
    }
  }
  colorPointCloud( reconstructs, context, sps.getAttributeCount(), params_.colorTransform_, generatePointCloudParameters );

#ifdef CODEC_TRACE
  setTrace( false );
  closeTrace();
#endif
  return 0;
}

/* THIS SECTION ADDS PATCH RELATED CALCULATIONS AND GENERATED PFDS */
/* Vlad part
void PCCDecoder::createPatchFrameDataStructure( PCCContext& context ) {
  // TODO
  size_t numPFDUs = context.getFrames().size();
  auto   ref      = context.getFrames()[0];
  auto&  sps      = context.getSps();
  auto&  psdu     = context.getPatchSequenceDataUnit();  // perhaps I need to allocate this

  RefListStruct refList;
  refList.allocate();
  refList.setNumRefEntries( 1 );      // hardcoded allow only 1 reference frame
  refList.setAbsDeltaPfocSt( 1, 0 );  // hardcoded: allowed previous farme as reference

  // 1. Create sequence of the patch frames
  for ( size_t i = 0; i < numPFDUs; i++ ) {
    auto& frame = context.getFrames()[i];

    PatchFrameHeader         pfh;
    PatchFrameDataUnit       pfdu;
    PatchFrameLayerUnit      pflu;
    PatchSequenceUnitPayload psup;

    pflu.setFrameIndex( i );

    for ( size_t k = 0; k < frame.getNumMatchedPatches(); k++ ) {
      //...
    }

    frame.getWidth()                                        = sps.getFrameWidth();
    frame.getHeight()                                       = sps.getFrameHeight();
    auto& frameLevelMetadataEnabledFlags                    = context.getGOFLevelMetadata().getLowerLevelMetadataEnabledFlags();
    frame.getFrameLevelMetadata().getMetadataEnabledFlags() = frameLevelMetadataEnabledFlags;

    if ( sps.getPcmPatchEnabledFlag() && !sps.getPcmSeparateVideoPresentFlag() ) {
      if ( !sps.getPcmSeparateVideoPresentFlag() ) {
        // ...ppdu
      }
    }
  }
}

void PCCDecoder::createPatchFrameDataStructure( PCCContext& context, PCCFrameContext& frame, PCCFrameContext& preFrame, size_t frameIndex ) {
  // TODO
}
Vlad part
*/

void PCCDecoder::setFrameMetadata( PCCMetadata& metadata, GeometryFrameParameterSet& gfps ) {
  auto& gfp                                 = gfps.getGeometryFrameParams();
  auto& metadataEnabingFlags                = metadata.getMetadataEnabledFlags();
  metadataEnabingFlags.getMetadataEnabled() = gfps.getGeometryParamsEnabledFlag();
  metadataEnabingFlags.getMetadataEnabled() = gfps.getGeometryParamsEnabledFlag();

  if ( gfps.getGeometryParamsEnabledFlag() ) {
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
        metadata.getRotation()[0] = gfp.getGeometryRotationOnAxis( 0 );
        metadata.getRotation()[1] = gfp.getGeometryRotationOnAxis( 1 );
        metadata.getRotation()[2] = gfp.getGeometryRotationOnAxis( 2 );
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
      if ( gfp.getGeometryPointShapeInfoPresentFlag() ) { metadata.getPointShape() = static_cast<PointShape>( gfp.getGeometryPointSizeInfo() ); }
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
      metadata.getRotation()[0] = gpp.getGeometryPatchRotationOnAxis( 0 );
      metadata.getRotation()[1] = gpp.getGeometryPatchRotationOnAxis( 1 );
      metadata.getRotation()[2] = gpp.getGeometryPatchRotationOnAxis( 2 );
    }

    // Point size
    metadataEnabingFlags.getPointSizeEnabled() = gpp.getGeometryPatchPointSizeInfoPresentFlag();
    if ( gpp.getGeometryPatchPointSizeInfoPresentFlag() ) { metadata.getPointSize() = gpp.getGeometryPatchPointSizeInfo(); }

    // Point shape
    metadataEnabingFlags.getPointShapeEnabled() = gpp.getGeometryPatchPointShapeInfoPresentFlag();
    if ( gpp.getGeometryPatchPointShapeInfoPresentFlag() ) {
      metadata.getPointShape() = static_cast<PointShape>( gpp.getGeometryPatchPointShapeInfo() );
    }
#ifdef CE210_MAXDEPTH_EVALUATION
    if ( metadataEnabingFlags.getMaxDepthEnabled() ) {
      currentDD = metadata.getQMaxDepthInPatch();
      // TODO: gfp.setGeometryMaxDeltaDepth(currentDD);
    }
#endif
  }
  // auto& lowerLevelMetadataEnabledFlags = metadata.getLowerLevelMetadataEnabledFlags();
  // // gfp.set...(lowerLevelMetadataEnabledFlags.getMetadataEnabled())
  // if ( lowerLevelMetadataEnabledFlags.getMetadataEnabled() ) {
  //   /* TODO: I could not locate the corespondence for these parameters*/
  //   // lowerLevelMetadataEnabledFlags.getScaleEnabled()
  //   // lowerLevelMetadataEnabledFlags.getOffsetEnabled()
  //   // lowerLevelMetadataEnabledFlags.getRotationEnabled()
  //   // lowerLevelMetadataEnabledFlags.getPointSizeEnabled()
  //   // lowerLevelMetadataEnabledFlags.getPointShapeEnabled()
  // }
}

void PCCDecoder::setPointLocalReconstruction( PCCFrameContext&          frame,
                                              PCCPatch&                 patch,
                                              PointLocalReconstruction& plr,
                                              size_t                    occupancyPackingBlockSize ) {
  auto&  blockToPatch       = frame.getBlockToPatch();
  size_t blockToPatchWidth  = frame.getWidth() / occupancyPackingBlockSize;
  size_t blockToPatchHeight = frame.getHeight() / occupancyPackingBlockSize;
  auto&  interpolateMap     = frame.getInterpolate();
  auto&  fillingMap         = frame.getFilling();
  auto&  minD1Map           = frame.getMinD1();
  auto&  neighborMap        = frame.getNeighbor();

  for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
    for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
      int pos           = patch.patchBlock2CanvasBlock( ( u0 ), ( v0 ), blockToPatchWidth, blockToPatchHeight );
      blockToPatch[pos] = plr.getBlockToPatchMap( u0, v0 );
      if ( blockToPatch[pos] > 0 ) {
        interpolateMap[pos] = plr.getModeInterpolateFlag( u0, v0 );
        if ( interpolateMap[pos] > 0 ) {
          uint32_t code    = static_cast<uint32_t>( int() - 1 );
          neighborMap[pos] = plr.getModeNeighbourMinus1( u0, v0 ) + 1;
        }
        minD1Map[pos] = plr.getModeMinimumDepthMinus1( u0, v0 ) + 1;
        if ( minD1Map[pos] > 1 || interpolateMap[pos] > 0 ) { fillingMap[pos] = plr.getModeFillingFlag( u0, v0 ); }
      }
    }
  }
}

void PCCDecoder::createPatchFrameDataStructure( PCCContext& context ) {
  TRACE_CODEC( "createPatchFrameDataStructure GOP start \n" );
  auto& sps  = context.getSps();
  auto& psdu = context.getPatchSequenceDataUnit();

  context.resize( psdu.getFrameCount() );

  // TRACE_CODEC(" PatchOrientationPresentFlag = %lu \n",
  //     psdu.getPatchFrameParameterSet( 0 ).getPatchOrientationPresentFlag() );

  TRACE_CODEC( "getFrameCount %u \n", psdu.getFrameCount() );

  setFrameMetadata( context.getGOFLevelMetadata(), psdu.getGeometryFrameParameterSet( 0 ) );

  size_t indexPrevFrame = 0;
  for ( int i = 0; i < psdu.getFrameCount(); i++ ) {
    auto& frame                                             = context.getFrame( i );
    auto& frameLevelMetadataEnabledFlags                    = context.getGOFLevelMetadata().getLowerLevelMetadataEnabledFlags();
    frame.getIndex()                                        = i;
    frame.getWidth()                                        = sps.getFrameWidth();
    frame.getHeight()                                       = sps.getFrameHeight();
    frame.getFrameLevelMetadata().getMetadataEnabledFlags() = frameLevelMetadataEnabledFlags;

    createPatchFrameDataStructure( context, frame, context.getFrame( indexPrevFrame ), i );

    if ( sps.getPcmPatchEnabledFlag() && !sps.getPcmSeparateVideoPresentFlag() ) {
      if ( !sps.getPcmSeparateVideoPresentFlag() ) {
        auto& patches           = frame.getPatches();
        auto& missedPointsPatch = frame.getMissedPointsPatch();
        if ( sps.getPcmPatchEnabledFlag() ) {
          const size_t patchIndex                = patches.size();
          PCCPatch&    dummyPatch                = patches[patchIndex - 1];
          missedPointsPatch.u0_                  = dummyPatch.getU0();
          missedPointsPatch.v0_                  = dummyPatch.getV0();
          missedPointsPatch.sizeU0_              = dummyPatch.getSizeU0();
          missedPointsPatch.sizeV0_              = dummyPatch.getSizeV0();
          missedPointsPatch.occupancyResolution_ = dummyPatch.getOccupancyResolution();
          patches.pop_back();
        }
      }
    }
    indexPrevFrame = i;
  }
}

void PCCDecoder::createPatchFrameDataStructure( PCCContext& context, PCCFrameContext& frame, PCCFrameContext& preFrame, size_t frameIndex ) {
  TRACE_CODEC( "createPatchFrameDataStructure Frame %lu \n", frame.getIndex() );
  auto& sps     = context.getSps();
  auto& psdu    = context.getPatchSequenceDataUnit();
  auto& ops     = sps.getOccupancyParameterSet();
  auto& pflu    = psdu.getPatchFrameLayerUnit( frameIndex );
  auto& pfh     = pflu.getPatchFrameHeader();
  auto& pfdu    = pflu.getPatchFrameDataUnit();
  auto& pfps    = psdu.getPatchFrameParameterSet( 0 );
  auto& patches = frame.getPatches();

  // uint32_t patchCount = pfdu.getPatchCount() + 1;
  TRACE_CODEC( "PatchCount %u \n", pfdu.getPatchCount() );
  patches.resize( (size_t)pfdu.getPatchCount() );
  TRACE_CODEC( "patches size = %lu \n", patches.size() );

  const size_t  minLevel               = sps.getMinLevel();
  const uint8_t maxBitCountForMinDepth = uint8_t( 10 - gbitCountSize[minLevel] );
  const uint8_t maxBitCountForMaxDepth = uint8_t( 9 - gbitCountSize[minLevel] );
  frame.allocOneLayerData( ops.getOccupancyPackingBlockSize() );
  frame.setSurfaceThickness( sps.getSurfaceThickness() );
  TRACE_CODEC( "OccupancyPackingBlockSize           = %d \n", ops.getOccupancyPackingBlockSize() );
  TRACE_CODEC( "PatchSequenceOrientationEnabledFlag = %d \n", sps.getPatchSequenceOrientationEnabledFlag() );
  TRACE_CODEC( "PatchOrientationPresentFlag         = %d \n", pfps.getPatchOrientationPresentFlag() );
  TRACE_CODEC( "PatchInterPredictionEnabledFlag     = %d \n", sps.getPatchInterPredictionEnabledFlag() );

  if ( ( frameIndex == 0 ) || ( !sps.getPatchInterPredictionEnabledFlag() ) ) {  // context.getDeltaCoding()

    frame.getFrameLevelMetadata().setMetadataType( METADATA_FRAME );
    frame.getFrameLevelMetadata().setIndex( frame.getIndex() );
    int64_t prevSizeU0 = 0;
    int64_t prevSizeV0 = 0;

    for ( size_t patchIndex = 0; patchIndex < pfdu.getPatchCount(); ++patchIndex ) {
      TRACE_CODEC( "patch %lu / %lu \n", patchIndex, pfdu.getPatchCount() );
      auto& pid   = pfdu.getPatchInformationData( patchIndex );
      auto& pdu   = pid.getPatchDataUnit();
      auto& patch = patches[patchIndex];

      patch.getOccupancyResolution() = ops.getOccupancyPackingBlockSize();
      patch.getU0()                  = pdu.get2DShiftU();
      patch.getV0()                  = pdu.get2DShiftV();
      patch.getU1()                  = pdu.get3DShiftTangentAxis();
      patch.getV1()                  = pdu.get3DShiftBiTangentAxis();
      patch.getLod()                 = pdu.getLod();

      TRACE_CODEC( "PatchOrientationPresentFlag = %d \n", pfps.getPatchOrientationPresentFlag() );
      if ( pfps.getPatchOrientationPresentFlag() ) {
        TRACE_CODEC( "OrientationSwapFlag = %d \n", pdu.getOrientationSwapFlag() );
        patch.getPatchOrientation() = pdu.getOrientationSwapFlag() ? PatchOrientation::SWAP : PatchOrientation::DEFAULT;
      } else {
        patch.getPatchOrientation() = PatchOrientation::DEFAULT;
      }

      if ( sps.getLayerAbsoluteCodingEnabledFlag( 1 ) ) {
        patch.getProjectionMode() = pdu.getProjectionMode();
      } else {
        patch.getProjectionMode() = 0;
      }

      TRACE_CODEC( "minLevel                      = %lu \n", minLevel );
      TRACE_CODEC( "patch.getProjectionMode()     = %lu \n", patch.getProjectionMode() );
      TRACE_CODEC( "pdu.get3DShiftTangentAxis()   = %lu \n", pdu.get3DShiftTangentAxis() );
      TRACE_CODEC( "pdu.get3DShiftBiTangentAxis() = %lu \n", pdu.get3DShiftBiTangentAxis() );
      TRACE_CODEC( "pdu.get3DShiftNormalAxis()    = %lu \n", pdu.get3DShiftNormalAxis() );
      TRACE_CODEC( "pdu.get2DDeltaSizeU()         = %ld \n", pdu.get2DDeltaSizeU() );
      TRACE_CODEC( "pdu.get2DDeltaSizeV()         = %ld \n", pdu.get2DDeltaSizeV() );

      if ( patch.getProjectionMode() == 0 ) {
        patch.getD1() = (int32_t)pdu.get3DShiftNormalAxis() * minLevel;
      } else {
        patch.getD1() = 1024 - (int32_t)pdu.get3DShiftNormalAxis() * minLevel;
      }

      const int64_t deltaSizeU0 = pdu.get2DDeltaSizeU();
      const int64_t deltaSizeV0 = pdu.get2DDeltaSizeV();

      patch.getSizeU0() = prevSizeU0 + deltaSizeU0;
      patch.getSizeV0() = prevSizeV0 + deltaSizeV0;

      prevSizeU0 = patch.getSizeU0();
      prevSizeV0 = patch.getSizeV0();
      TRACE_CODEC( "pdu.getNormalAxis()           = %lu \n", pdu.getNormalAxis() );

      patch.getNormalAxis() = pdu.getNormalAxis();
      TRACE_CODEC( "patch.getNormalAxis()         = %lu \n", patch.getNormalAxis() );

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
      TRACE_CODEC( "patch UV0 %4lu %4lu UV1 %4lu %4lu D1=%4lu S=%4lu %4lu P=%lu O=%lu A=%u%u%u \n", patch.getU0(), patch.getV0(), patch.getU1(),
                   patch.getV1(), patch.getD1(), patch.getSizeU0(), patch.getSizeV0(), patch.getProjectionMode(), patch.getPatchOrientation(),
                   patch.getNormalAxis(), patch.getTangentAxis(), patch.getBitangentAxis() );

      auto&         patchLevelMetadataEnabledFlags = frame.getFrameLevelMetadata().getLowerLevelMetadataEnabledFlags();
      auto&         metadata                       = patch.getPatchLevelMetadata();
      const uint8_t bitCountDD                     = maxBitCountForMaxDepth;
      metadata.setIndex( patchIndex );
      metadata.setMetadataType( METADATA_PATCH );
      metadata.getMetadataEnabledFlags() = patchLevelMetadataEnabledFlags;
      metadata.setbitCountQDepth( bitCountDD );

      // maxDepth reconstruction
#ifdef CE210_MAXDEPTH_EVALUATION
      patch.getSizeD() = size_t( patchLevelMetadata.getQMaxDepthInPatch() ) * minLevel;
#else
      patch.getSizeD()       = minLevel;
#endif
      setPatchMetadata( metadata, psdu.getGeometryPatchParameterSet( 0 ) );
      if ( sps.getLayerAbsoluteCodingEnabledFlag( 1 ) && !sps.getMultipleLayerStreamsPresentFlag() ) {
        setPointLocalReconstruction( frame, patch, pfdu.getPointLocalReconstruction(), ops.getOccupancyPackingBlockSize() );
      }
    }
  } else {
    auto&   prePatches = preFrame.getPatches();
    size_t  patchCount = patches.size();
    uint8_t bitCount[6];
    uint8_t F = 0, A[5] = {0, 0, 0, 0, 0};
    size_t  topNmax[6] = {0, 0, 0, 0, 0, 0};

    const size_t  minLevel               = sps.getMinLevel();
    const uint8_t maxBitCountForMinDepth = uint8_t( 10 - gbitCountSize[minLevel] );
    bitCount[4]                          = maxBitCountForMinDepth;
    const uint8_t maxBitCountForMaxDepth = uint8_t( 9 - gbitCountSize[minLevel] );
    bitCount[5]                          = maxBitCountForMaxDepth;

    frame.getFrameLevelMetadata().setMetadataType( METADATA_FRAME );
    frame.getFrameLevelMetadata().setIndex( frame.getIndex() );
    // decompressMetadata( frame.getFrameLevelMetadata(), arithmeticDecoder );

    int64_t  prevSizeU0        = 0;
    int64_t  prevSizeV0        = 0;
    uint32_t numMatchedPatches = pfdu.getMatchedPatchCount();
    if ( pfh.getInterPredictPatchBitCountFlag() ) {
      // uint8_t flag = uint8_t( DecodeUInt32( 4, arithmeticDecoder, bModel0 ) );

      // for ( int i = 0; i < 4; i++ ) {
      //   A[3 - i] = flag & 1;
      //   flag     = flag >> 1;
      // }
      // for ( int i = 0; i < 4; i++ ) {
      //   if ( A[i] ) bitCount[i] = uint8_t( DecodeUInt32( 8, arithmeticDecoder, bModel0 ) );
      // }
      A[0] = pfh.getInterPredictPatch2dShiftUBitCountFlag();
      A[1] = pfh.getInterPredictPatch2dShiftVBitCountFlag();
      A[2] = pfh.getInterPredictPatch3dShiftTangentAxisBitCountFlag();
      A[3] = pfh.getInterPredictPatch3dShiftBitangentAxisBitCountFlag();
      A[4] = pfh.getInterPredictPatchLodBitCountFlag();
      if ( A[0] ) { bitCount[0] = pfh.getInterPredictPatch2dShiftUBitCountMinus1(); }
      if ( A[1] ) { bitCount[1] = pfh.getInterPredictPatch2dShiftVBitCountMinus1(); }
      if ( A[2] ) { bitCount[2] = pfh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1(); }
      if ( A[3] ) { bitCount[3] = pfh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1(); }
      if ( A[4] ) { bitCount[4] = pfh.getInterPredictPatchLodBitCount(); }
    }
    if ( printDetailedInfo ) {
      printf( "numPatch:%d(%d),  F:%d,A[4]:%d,%d,%d,%d\n", (int)patchCount, (int)numMatchedPatches, F, A[0], A[1], A[2], A[3] );
    }

    int64_t predIndex = 0;
    for ( size_t patchIndex = 0; patchIndex < numMatchedPatches; ++patchIndex ) {
      auto& patch                    = patches[patchIndex];
      patch.getOccupancyResolution() = ops.getOccupancyPackingBlockSize();
      auto& pid                      = pfdu.getPatchInformationData( patchIndex );
      auto& dpdu                     = pid.getDeltaPatchDataUnit();

      int64_t deltaIndex      = dpdu.getDeltaPatchIdx();  // o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel0, bModelPatchIndex ) );
      patch.setBestMatchIdx() = ( size_t )( deltaIndex + predIndex );

      TRACE_CODEC( "dpdu.getDeltaPatchIdx() = %d \n", dpdu.getDeltaPatchIdx() );
      TRACE_CODEC( "patch.getBestMatchIdx() = %d \n", patch.getBestMatchIdx() );
      TRACE_CODEC( "predIndex               = %d \n", predIndex );
      TRACE_CODEC( "deltaIndex              = %d \n", deltaIndex );
      predIndex += ( deltaIndex + 1 );

      const auto&   prePatch = prePatches[patch.getBestMatchIdx()];
      const int64_t delta_U0 = dpdu.get2DDeltaShiftU();              // o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel0, bModelU0 ) );
      const int64_t delta_V0 = dpdu.get2DDeltaShiftV();              // o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel0, bModelV0 ) );
      const int64_t delta_U1 = dpdu.get3DDeltaShiftTangentAxis();    // o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel0, bModelU1 ) );
      const int64_t delta_V1 = dpdu.get3DDeltaShiftBiTangentAxis();  // o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel0, bModelV1 ) );
      const int64_t delta_D1 = dpdu.get3DDeltaShiftNormalAxis();     // o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel0, bModelD1 ) );
      const int64_t deltaSizeU0 = dpdu.get2DDeltaSizeU();  // o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel0, bModelIntSizeU0 ) );
      const int64_t deltaSizeV0 = dpdu.get2DDeltaSizeV();  // o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel0, bModelIntSizeV0 ) );

      TRACE_CODEC( "DeltaIdx = %d ShiftUV = %ld %ld ShiftAxis = %ld %ld %ld Size = %ld %ld \n", dpdu.getDeltaPatchIdx(), dpdu.get2DDeltaShiftU(),
                   dpdu.get2DDeltaShiftV(), dpdu.get3DDeltaShiftTangentAxis(), dpdu.get3DDeltaShiftBiTangentAxis(), dpdu.get3DDeltaShiftNormalAxis(),
                   dpdu.get2DDeltaSizeU(), dpdu.get2DDeltaSizeV() );

      if ( sps.getLayerAbsoluteCodingEnabledFlag( 1 ) ) {
        patch.getProjectionMode() = static_cast<size_t>( dpdu.getProjectionMode() );
      } else {
        patch.getProjectionMode() = 0;
      }

      patch.getU0()               = delta_U0 + prePatch.getU0();
      patch.getV0()               = delta_V0 + prePatch.getV0();
      patch.getPatchOrientation() = prePatch.getPatchOrientation();
      patch.getU1()               = delta_U1 + prePatch.getU1();
      patch.getV1()               = delta_V1 + prePatch.getV1();
      size_t currentD1            = 0;
      size_t prevD1               = prePatch.getD1();
      if ( patch.getProjectionMode() == 0 ) {
        prevD1        = prevD1 / minLevel;
        currentD1     = ( prevD1 - delta_D1 );
        patch.getD1() = ( delta_D1 + prevD1 ) * minLevel;
      } else {
        prevD1        = ( 1024 - prevD1 ) / minLevel;
        currentD1     = ( delta_D1 + prevD1 );
        patch.getD1() = 1024 - currentD1 * minLevel;  //(delta_D1 + prePatch.getD1()/minLevel)*minLevel;
      }

      patch.getSizeU0() = deltaSizeU0 + prePatch.getSizeU0();
      patch.getSizeV0() = deltaSizeV0 + prePatch.getSizeV0();

      // get maximum
      topNmax[0] = topNmax[0] < patch.getU0() ? patch.getU0() : topNmax[0];
      topNmax[1] = topNmax[1] < patch.getV0() ? patch.getV0() : topNmax[1];
      topNmax[2] = topNmax[2] < patch.getU1() ? patch.getU1() : topNmax[2];
      topNmax[3] = topNmax[3] < patch.getV1() ? patch.getV1() : topNmax[3];
      size_t D1  = patch.getD1() / minLevel;
      topNmax[4] = topNmax[4] < D1 ? D1 : topNmax[4];

      prevSizeU0 = patch.getSizeU0();
      prevSizeV0 = patch.getSizeV0();

      patch.getNormalAxis()    = prePatch.getNormalAxis();
      patch.getTangentAxis()   = prePatch.getTangentAxis();
      patch.getBitangentAxis() = prePatch.getBitangentAxis();

      TRACE_CODEC( "patch Inter UV0 %4lu %4lu UV1 %4lu %4lu D1=%4lu S=%4lu %4lu P=%lu O=%lu A=%u%u%u \n", patch.getU0(), patch.getV0(), patch.getU1(),
                   patch.getV1(), patch.getD1(), patch.getSizeU0(), patch.getSizeV0(), patch.getProjectionMode(), patch.getPatchOrientation(),
                   patch.getNormalAxis(), patch.getTangentAxis(), patch.getBitangentAxis() );

      if ( printDetailedInfo ) { patch.printDecoder(); }
    }

    // read info from metadata and resconstruc maxDepth
    // o3dgc::Adaptive_Bit_Model bModelDD;
    for ( size_t patchIndex = 0; patchIndex < numMatchedPatches; ++patchIndex ) {
      auto& patch                                  = patches[patchIndex];
      auto& patchLevelMetadataEnabledFlags         = frame.getFrameLevelMetadata().getLowerLevelMetadataEnabledFlags();
      auto& patchLevelMetadata                     = patch.getPatchLevelMetadata();
      patchLevelMetadata.getMetadataEnabledFlags() = patchLevelMetadataEnabledFlags;
      patchLevelMetadata.setIndex( patchIndex );
      patchLevelMetadata.setMetadataType( METADATA_PATCH );
      patchLevelMetadata.setbitCountQDepth( 0 );  // added 20190129
                                                  // decompressMetadata( patchLevelMetadata, arithmeticDecoder, bModel0, bModelDD );

#ifdef CE210_MAXDEPTH_EVALUATION
      const int64_t delta_DD = patchLevelMetadata.getQMaxDepthInPatch();
#else
      const int64_t delta_DD = 0;
#endif
      const auto& prePatch = prePatches[patch.getBestMatchIdx()];
      size_t      currentDD;
      size_t      prevDD = prePatch.getSizeD() / minLevel;
      if ( prevDD * minLevel != prePatch.getSizeD() ) prevDD += 1;
      currentDD        = ( delta_DD + prevDD ) * minLevel;
      patch.getSizeD() = currentDD;
    }

    // // Get Bitcount.
    // for ( int i = 0; i < 4; i++ ) {
    //   if ( A[i] == 0 ) { bitCount[i] = uint8_t( PCCGetNumberOfBitsInFixedLengthRepresentation( uint32_t( topNmax[i] + 1 ) ) ); }
    // }

    for ( size_t patchIndex = numMatchedPatches; patchIndex < pfdu.getPatchCount(); ++patchIndex ) {
      auto& patch                    = patches[patchIndex];
      auto& pid                      = pfdu.getPatchInformationData( patchIndex );
      auto& pdu                      = pid.getPatchDataUnit();
      patch.getOccupancyResolution() = ops.getOccupancyPackingBlockSize();
      patch.getU0()                  = pdu.get2DShiftU();  // DecodeUInt32( bitCount[0], arithmeticDecoder, bModel0 );
      patch.getV0()                  = pdu.get2DShiftV();  // DecodeUInt32( bitCount[1], arithmeticDecoder, bModel0 );
      // if ( enable_flexible_patch_flag ) {
      //   bool flexible_patch_present_flag = arithmeticDecoder.decode( orientationPatchFlagModel2 );
      //   if ( flexible_patch_present_flag ) {
      //     patch.getPatchOrientation() = arithmeticDecoder.decode( orientationPatchModel ) + 1;
      //   } else {
      //     patch.getPatchOrientation() = PatchOrientation::DEFAULT;
      //   }
      // } else {
      //   patch.getPatchOrientation() = PatchOrientation::DEFAULT;
      // }
      if ( sps.getLayerAbsoluteCodingEnabledFlag( 1 ) ) {
        patch.getProjectionMode() = static_cast<size_t>( pdu.getProjectionMode() );
      } else {
        patch.getProjectionMode() = 0;
      }
      patch.getU1() = pdu.get3DShiftTangentAxis();    // DecodeUInt32( bitCount[2], arithmeticDecoder, bModel0 );
      patch.getV1() = pdu.get3DShiftBiTangentAxis();  // DecodeUInt32( bitCount[3], arithmeticDecoder, bModel0 );
      size_t D1     = pdu.get3DShiftNormalAxis();     // DecodeUInt32( bitCount[4], arithmeticDecoder, bModel0 );

      patch.getProjectionMode() = pdu.getProjectionMode();
      if ( patch.getProjectionMode() == 0 ) {
        patch.getD1() = D1 * minLevel;
      } else {
        patch.getD1() = ( 1024 - D1 * minLevel );
      }

      if ( pfps.getPatchOrientationPresentFlag() ) {
        patch.getPatchOrientation() = pdu.getOrientationSwapFlag() ? PatchOrientation::SWAP : PatchOrientation::DEFAULT;
      } else {
        patch.getPatchOrientation() = PatchOrientation::DEFAULT;
      }

      patch.getSizeU0() = prevSizeU0 + pdu.get2DDeltaSizeU();
      patch.getSizeV0() = prevSizeV0 + pdu.get2DDeltaSizeV();
      TRACE_CODEC( "PrevSize = %ld %ld DeltaSize = %ld %ld => %lu %lu \n", prevSizeU0, prevSizeV0, pdu.get2DDeltaSizeU(), pdu.get2DDeltaSizeV(),
                   patch.getSizeU0(), patch.getSizeV0() );

      prevSizeU0 = patch.getSizeU0();
      prevSizeV0 = patch.getSizeV0();

      // // if ( bBinArithCoding ) {
      // size_t bit0 = arithmeticDecoder.decode( orientationModel2 );
      // if ( bit0 == 0 ) {  // 0
      //   patch.getNormalAxis() = 0;
      // } else {
      //   size_t bit1 = arithmeticDecoder.decode( bModel0 );
      //   if ( bit1 == 0 ) {  // 10
      //     patch.getNormalAxis() = 1;
      //   } else {  // 11
      //     patch.getNormalAxis() = 2;
      //   }
      // }
      // //  } else {
      // //    patch.getNormalAxis() = arithmeticDecoder.decode( orientationModel );
      // //  }

      patch.getNormalAxis() = pdu.getNormalAxis();

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
      auto& patchLevelMetadataEnabledFlags         = frame.getFrameLevelMetadata().getLowerLevelMetadataEnabledFlags();
      auto& patchLevelMetadata                     = patch.getPatchLevelMetadata();
      patchLevelMetadata.getMetadataEnabledFlags() = patchLevelMetadataEnabledFlags;
      patchLevelMetadata.setIndex( patchIndex );
      patchLevelMetadata.setMetadataType( METADATA_PATCH );
      patchLevelMetadata.getMetadataEnabledFlags() = patchLevelMetadataEnabledFlags;
      patchLevelMetadata.setbitCountQDepth( maxBitCountForMaxDepth );

      // decompressMetadata( patchLevelMetadata, arithmeticDecoder, bModel0, bModelDD );
      setPatchMetadata( patchLevelMetadata, psdu.getGeometryPatchParameterSet( 0 ) );

      // maxdepth reconstruction
#ifdef CE210_MAXDEPTH_EVALUATION
      size_t DD        = size_t( patchLevelMetadata.getQMaxDepthInPatch() );
      patch.getSizeD() = (DD)*minLevel;
#else
      patch.getSizeD()       = minLevel;
#endif

      TRACE_CODEC( "patch Intra UV0 %4lu %4lu UV1 %4lu %4lu D1=%4lu S=%4lu %4lu P=%lu O=%lu A=%u%u%u \n", patch.getU0(), patch.getV0(), patch.getU1(),
                   patch.getV1(), patch.getD1(), patch.getSizeU0(), patch.getSizeV0(), patch.getProjectionMode(), patch.getPatchOrientation(),
                   patch.getNormalAxis(), patch.getTangentAxis(), patch.getBitangentAxis() );
      if ( printDetailedInfo ) { patch.printDecoder(); }
    }

    if ( sps.getLayerAbsoluteCodingEnabledFlag( 1 ) && !sps.getMultipleLayerStreamsPresentFlag() ) {
      for ( size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex ) {
        setPointLocalReconstruction( frame, patches[patchIndex], pfdu.getPointLocalReconstruction(), ops.getOccupancyPackingBlockSize() );
      }
    }
    // #ifdef BITSTREAM_TRACE
    //     bitstream.trace( "OccupancyMapM42195 done \n" );
    // #endif
  }
}