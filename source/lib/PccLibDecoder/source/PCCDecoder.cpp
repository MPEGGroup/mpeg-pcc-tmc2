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


PCCDecoder::PCCDecoder(){
}
PCCDecoder::~PCCDecoder(){
}

void PCCDecoder::setParameters( PCCDecoderParameters params ) { 
  params_ = params; 
}

int PCCDecoder::decode( PCCBitstream &bitstream, PCCContext &context, PCCGroupOfFrames& reconstructs ){
  int ret = 0; 
  if( params_.nbThread_ > 0 ) {
    tbb::task_scheduler_init init( (int)params_.nbThread_ );
  }
  PCCBitstreamDecoder bitstreamDecoder;
#ifdef BITSTREAM_TRACE
    bitstream.setTrace( true );
    bitstream.openTrace( removeFileExtension( params_.compressedStreamPath_ ) + "_prev_syntax_decode.txt" );
#endif
  if (!bitstreamDecoder.decode( bitstream, context ) ) {
    return 0;
  }
#ifdef BITSTREAM_TRACE
    bitstream.closeTrace();
#endif
  ret |= decode( context, reconstructs );
  return ret;
}

int PCCDecoder::decode( PCCContext& context, PCCGroupOfFrames& reconstructs ) {
  printf("decode start \n");fflush(stdout);
#ifdef CODEC_TRACE
  setTrace( true );
  printf("setTrace done \n");fflush(stdout);
  openTrace(  removeFileExtension( params_.compressedStreamPath_ ) + "_codec_decode.txt" );
  printf("openTrace done \n");fflush(stdout);
#endif
  printf("decode trace open \n");fflush(stdout);

  reconstructs.resize( context.size() );
  PCCVideoDecoder   videoDecoder;
  std::stringstream path;
  auto&             sps = context.getSps();
  auto&             gps = sps.getGeometryParameterSet();
  auto&             gsp = gps.getGeometrySequenceParams();
  auto&             ops = sps.getOccupancyParameterSet();
  auto&             aps = sps.getAttributeParameterSet( 0 );
  auto&             asp = aps.getAttributeSequenceParams();
  path << removeFileExtension( params_.compressedStreamPath_ ) << "_dec_GOF" << sps.getSequenceParameterSetId()
       << "_";

  bool  lossyMpp = !sps.getLosslessGeo() && sps.getPcmPatchEnabledFlag();
  // const size_t nbyteGeo =
  //     ( context.getLosslessGeo() || ( lossyMpp && !sps.getPcmSeparateVideoPresentFlag() ) )
  //         ? 2
  //         : 1;

  const size_t frameCountGeometry = sps.getMultipleLayerStreamsPresentFlag() ? 2 : 1;
  const size_t frameCountTexture  = sps.getMultipleLayerStreamsPresentFlag() ? 2 : 1;

  auto& videoBitstreamOM = context.getVideoBitstream( PCCVideoType::OccupancyMap );
  videoDecoder.decompress(
      context.getVideoOccupancyMap(), path.str(), context.size(), videoBitstreamOM,
      params_.videoDecoderOccupancyMapPath_, context, params_.keepIntermediateFiles_,
      ( sps.getLosslessGeo() ? sps.getLosslessGeo444() : false ), false, "", "" );
  context.getOccupancyPrecision() = sps.getFrameWidth() / context.getVideoOccupancyMap().getWidth(); 
  printf("compute OccupancyPrecision = %lu \n", context.getOccupancyPrecision());
  generateOccupancyMap( context, context.getOccupancyPrecision() );

  if (!sps.getLayerAbsoluteCodingEnabledFlag( 0 )) {
    if (lossyMpp) {
      std::cout << "ERROR! Lossy-missed-points-patch code not implemented when absoluteD_ = 0 as of now. Exiting ..." << std::endl; std::exit(-1);
    }
    // Compress D0
    auto& videoBitstreamD0 =  context.getVideoBitstream( PCCVideoType::GeometryD0 );
    videoDecoder.decompress( context.getVideoGeometry(), path.str(), 
                             context.size(), videoBitstreamD0,
                             params_.videoDecoderPath_, context, params_.keepIntermediateFiles_,
                             (sps.getLosslessGeo()?sps.getLosslessGeo444():false) );
    std::cout << "geometry D0 video ->" << videoBitstreamD0.naluSize() << " B" << std::endl;

    // Compress D1
    auto& videoBitstreamD1 =  context.getVideoBitstream( PCCVideoType::GeometryD1 );
    videoDecoder.decompress(context.getVideoGeometryD1(), path.str(), 
                            context.size(), videoBitstreamD1, params_.videoDecoderPath_,
                            context, params_.keepIntermediateFiles_,
                            (sps.getLosslessGeo()?sps.getLosslessGeo444():false) );
    std::cout << "geometry D1 video ->" << videoBitstreamD1.naluSize() << " B" << std::endl;

    std::cout << "geometry video ->" << videoBitstreamD1.naluSize() + videoBitstreamD1.naluSize() << " B" << std::endl;
  } else {
    auto& videoBitstream =  context.getVideoBitstream( PCCVideoType::Geometry );
    videoDecoder.decompress(context.getVideoGeometry(), path.str(),
                            context.size() * frameCountGeometry, videoBitstream,
                            params_.videoDecoderPath_, context,  params_.keepIntermediateFiles_,
                            sps.getLosslessGeo() & sps.getLosslessGeo444() );
    std::cout << "geometry video ->" << videoBitstream.naluSize() << " B" << std::endl;
  }

  if( sps.getPcmPatchEnabledFlag() && sps.getPcmSeparateVideoPresentFlag()) {
    auto& videoBitstreamMP =  context.getVideoBitstream( PCCVideoType::GeometryMP ); 
    videoDecoder.decompress(context.getVideoMPsGeometry(), path.str(),
                            context.size(), videoBitstreamMP, params_.videoDecoderPath_,
                            context, params_.keepIntermediateFiles_ );

    generateMissedPointsGeometryfromVideo(context, reconstructs); //0. geo : decode arithmetic coding part
    std::cout << " missed points geometry -> " << videoBitstreamMP.naluSize() << " B "<<endl;

    //add missed point to reconstructs
    //fillMissedPoints(reconstructs, context, 0, params_.colorTransform_); //0. geo
  }
  bool useAdditionalPointsPatch = sps.getPcmPatchEnabledFlag();
  bool lossyMissedPointsPatch   = !sps.getLosslessGeo() && useAdditionalPointsPatch;
  if ( ( sps.getLosslessGeo() != 0 ) && sps.getEnhancedOccupancyMapForDepthFlag() ) {
    generateBlockToPatchFromOccupancyMap( context, sps.getLosslessGeo(), lossyMissedPointsPatch,
                                          0, ops.getOccupancyPackingBlockSize() );
  } else {
    generateBlockToPatchFromBoundaryBox( context, sps.getLosslessGeo(), lossyMissedPointsPatch,
                                         0, ops.getOccupancyPackingBlockSize() );
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
  generatePointCloudParameters.absoluteD1_                   = sps.getLayerAbsoluteCodingEnabledFlag( 0 );
  generatePointCloudParameters.surfaceThickness              = context[0].getSurfaceThickness();
  generatePointCloudParameters.ignoreLod_                    = true;
  generatePointCloudParameters.thresholdColorSmoothing_      = (double)asp.getAttributeSmoothingThreshold();
  generatePointCloudParameters.thresholdLocalEntropy_        = (double)asp.getAttributeSmoothingThresholdLocalEntropy();
  generatePointCloudParameters.radius2ColorSmoothing_        = (double)asp.getAttributeSmoothingRadius();
  generatePointCloudParameters.neighborCountColorSmoothing_  = asp.getAttributeSmoothingNeighbourCount();
  generatePointCloudParameters.flagColorSmoothing_           = (bool) asp.getAttributeSmoothingParamsPresentFlag();
  generatePointCloudParameters.enhancedDeltaDepthCode_       = ((sps.getLosslessGeo() != 0) ? sps.getEnhancedOccupancyMapForDepthFlag() : false);
  // generatePointCloudParameters.deltaCoding_                  = (params_.testLevelOfDetailSignaling_ > 0); // ignore LoD scaling for testing the signaling only
  generatePointCloudParameters.removeDuplicatePoints_        = sps.getRemoveDuplicatePointEnabledFlag();
  generatePointCloudParameters.oneLayerMode_                 = !sps.getMultipleLayerStreamsPresentFlag();
  generatePointCloudParameters.singleLayerPixelInterleaving_ = sps.getPixelDeinterleavingFlag();
  // generatePointCloudParameters.sixDirectionMode_             = context.getSixDirectionMode();
  // generatePointCloudParameters.improveEDD_                   = context.getImproveEDD();
  generatePointCloudParameters.path_                         = path.str();
  generatePointCloudParameters.useAdditionalPointsPatch_     = sps.getPcmPatchEnabledFlag();

  generatePointCloud( reconstructs, context, generatePointCloudParameters );

  if ( sps.getAttributeCount() > 0 ) {
    auto& videoBitstream = context.getVideoBitstream( PCCVideoType::Texture );
    videoDecoder.decompress( context.getVideoTexture(), path.str(), 
                             context.size() * frameCountTexture, videoBitstream,
                             params_.videoDecoderPath_, context, params_.keepIntermediateFiles_,
                             sps.getLosslessTexture() != 0, params_.patchColorSubsampling_,
                             params_.inverseColorSpaceConversionConfig_, params_.colorSpaceConversionPath_  );
    std::cout << "texture video  ->" << videoBitstream.naluSize() << " B" << std::endl;

    if( sps.getPcmPatchEnabledFlag() && sps.getPcmSeparateVideoPresentFlag()) {
      auto& videoBitstreamMP = context.getVideoBitstream( PCCVideoType::TextureMP );
      videoDecoder.decompress( context.getVideoMPsTexture(), path.str(),
                               context.size(), videoBitstreamMP, params_.videoDecoderPath_,
                               context, params_.keepIntermediateFiles_,
                               sps.getLosslessTexture(), false,
                               params_.inverseColorSpaceConversionConfig_, params_.colorSpaceConversionPath_ );

      generateMissedPointsTexturefromVideo(context, reconstructs);
      std::cout << " missed points texture -> " << videoBitstreamMP.naluSize() << " B"<<endl;
    }
  }
  colorPointCloud(reconstructs, context, sps.getAttributeCount(), params_.colorTransform_,
                  generatePointCloudParameters);
                  
#ifdef CODEC_TRACE
  setTrace( false );
  closeTrace(); 
#endif
  return 0;
}


