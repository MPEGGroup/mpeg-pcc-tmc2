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
  if (!bitstreamDecoder.decode( bitstream, context ) ) {
    return 0;
  }
  ret |= decode( context, reconstructs );
  return ret; 
}

int PCCDecoder::decode( PCCContext &context, PCCGroupOfFrames& reconstructs ){
  reconstructs.resize( context.size() );
  PCCVideoDecoder videoDecoder; 
  std::stringstream path;
  path << removeFileExtension( params_.compressedStreamPath_ ) << "_dec_GOF" << context.getIndex() << "_";

  bool lossyMpp = !context.getLosslessGeo() && context.getUseAdditionalPointsPatch();
  const size_t nbyteGeo = (context.getLosslessGeo() || (lossyMpp && !context.getUseMissedPointsSeparateVideo())) ? 2 : 1;
  const size_t frameCountGeometry = context.getOneLayerMode() ? 1 : 2;
  const size_t frameCountTexture  = context.getOneLayerMode()  ? 1 : 2;

  auto& videoBitstreamOM = context.getVideoBitstream( PCCVideoType::OccupancyMap );
  videoDecoder.decompress(context.getVideoOccupancyMap(), path.str() + "occupancy",
                          context.getWidth()/context.getOccupancyPrecision(),
                          context.getHeight()/context.getOccupancyPrecision(),
                          context.size(),
                          videoBitstreamOM,
                          params_.videoDecoderOccupancyMapPath_, context
                          ,"", "", (context.getLosslessGeo()?context.getLosslessGeo444():false), false, 1,params_.keepIntermediateFiles_ );
  generateOccupancyMap( context, context.getOccupancyPrecision() );

  if (!context.getAbsoluteD1()) {
    if (lossyMpp) {
      std::cout << "ERROR! Lossy-missed-points-patch code not implemented when absoluteD_ = 0 as of now. Exiting ..." << std::endl; std::exit(-1);
    }
    // Compress D0
    auto& videoBitstreamD0 =  context.getVideoBitstream( PCCVideoType::GeometryD0 );
    videoDecoder.decompress( context.getVideoGeometry(), path.str() + "geometryD0", context.getWidth(), context.getHeight(),
                             context.size(), videoBitstreamD0,
                             params_.videoDecoderPath_, context,
                             "", "", (context.getLosslessGeo()?context.getLosslessGeo444():false), false, nbyteGeo,
                             params_.keepIntermediateFiles_ );
    std::cout << "geometry D0 video ->" << videoBitstreamD0.naluSize() << " B" << std::endl;

    // Compress D1
    auto& videoBitstreamD1 =  context.getVideoBitstream( PCCVideoType::GeometryD1 );
    videoDecoder.decompress(context.getVideoGeometryD1(), path.str() + "geometryD1", context.getWidth(), context.getHeight(),
                            context.size(), videoBitstreamD1,
                            params_.videoDecoderPath_, context,
                            "", "", (context.getLosslessGeo()?context.getLosslessGeo444():false), false, nbyteGeo,
                            params_.keepIntermediateFiles_ );
    std::cout << "geometry D1 video ->" << videoBitstreamD1.naluSize() << " B" << std::endl;

    std::cout << "geometry video ->" << videoBitstreamD1.naluSize() + videoBitstreamD1.naluSize() << " B" << std::endl;
  } else {
    auto& videoBitstream =  context.getVideoBitstream( PCCVideoType::Geometry );
    videoDecoder.decompress(context.getVideoGeometry(), path.str() + "geometry", context.getWidth(), context.getHeight(),
                            context.size() * frameCountGeometry, videoBitstream,
                            params_.videoDecoderPath_, context,
                            "", "", context.getLosslessGeo() & context.getLosslessGeo444(), false, nbyteGeo,
                            params_.keepIntermediateFiles_);
    std::cout << "geometry video ->" << videoBitstream.naluSize() << " B" << std::endl;
  }

  if(context.getUseAdditionalPointsPatch() && context.getUseMissedPointsSeparateVideo()) {
    auto& videoBitstreamMP =  context.getVideoBitstream( PCCVideoType::GeometryMP ); 
    videoDecoder.decompress(context.getVideoMPsGeometry(), path.str() + "mps_geometry",
                            context.getMPGeoWidth(), context.getMPGeoHeight(),
                            context.size(), videoBitstreamMP,
                            params_.videoDecoderPath_, context,
                            "", "", 0,//context.getLosslessGeo() & losslessGeo444_,
                            false, 2,//patchColorSubsampling, 2.nByteGeo 10 bit coding
                            params_.keepIntermediateFiles_ );
    assert(context.getMPGeoWidth() == context.getVideoMPsGeometry().getWidth());
    assert(context.getMPGeoHeight() == context.getVideoMPsGeometry().getHeight());
    generateMissedPointsGeometryfromVideo(context, reconstructs); //0. geo : decode arithmetic coding part
    std::cout << " missed points geometry -> " << videoBitstreamMP.naluSize() << " B "<<endl;

    //add missed point to reconstructs
    //fillMissedPoints(reconstructs, context, 0, params_.colorTransform_); //0. geo
  }
  bool useAdditionalPointsPatch = context.getFrames()[0].getUseAdditionalPointsPatch();
  bool lossyMissedPointsPatch = ! context.getLosslessGeo() && useAdditionalPointsPatch;
  if( (context.getLosslessGeo() != 0 ) && context.getEnhancedDeltaDepthCode()) {
    generateBlockToPatchFromOccupancyMap( context, context.getLosslessGeo(), lossyMissedPointsPatch, 0, context.getOccupancyResolution() );
  }else{
    generateBlockToPatchFromBoundaryBox( context, context.getLosslessGeo(), lossyMissedPointsPatch, 0, context.getOccupancyResolution() );
  }

  GeneratePointCloudParameters generatePointCloudParameters;
  generatePointCloudParameters.occupancyResolution_          = context.getOccupancyResolution();
  generatePointCloudParameters.occupancyPrecision_           = context.getOccupancyPrecision();
  generatePointCloudParameters.flagGeometrySmoothing_        = (bool)context.getFlagGeometrySmoothing();
  generatePointCloudParameters.gridSmoothing_                = context.getGridSmoothing();
  generatePointCloudParameters.gridSize_                     = context.getGridSize();
  generatePointCloudParameters.neighborCountSmoothing_       = context.getNeighborCountSmoothing();
  generatePointCloudParameters.radius2Smoothing_             = (double)context.getRadius2Smoothing();
  generatePointCloudParameters.radius2BoundaryDetection_     = (double)context.getRadius2BoundaryDetection();
  generatePointCloudParameters.thresholdSmoothing_           = (double)context.getThresholdSmoothing();
  generatePointCloudParameters.losslessGeo_                  = context.getLosslessGeo() != 0;
  generatePointCloudParameters.losslessGeo444_               = context.getLosslessGeo444() != 0;
  generatePointCloudParameters.nbThread_                     = params_.nbThread_;
  generatePointCloudParameters.absoluteD1_                   = context.getAbsoluteD1();
  generatePointCloudParameters.surfaceThickness              = context[0].getSurfaceThickness();
  generatePointCloudParameters.ignoreLod_                    = true;
  generatePointCloudParameters.thresholdColorSmoothing_      = (double)context.getThresholdColorSmoothing();
  generatePointCloudParameters.thresholdLocalEntropy_        = (double)context.getThresholdLocalEntropy();
  generatePointCloudParameters.radius2ColorSmoothing_        = (double)context.getRadius2ColorSmoothing();
  generatePointCloudParameters.neighborCountColorSmoothing_  = context.getNeighborCountColorSmoothing();
  generatePointCloudParameters.flagColorSmoothing_           = (bool) context.getFlagColorSmoothing();
  generatePointCloudParameters.enhancedDeltaDepthCode_       = ((context.getLosslessGeo() != 0) ? context.getEnhancedDeltaDepthCode() : false);
  generatePointCloudParameters.deltaCoding_                  = (params_.testLevelOfDetailSignaling_ > 0); // ignore LoD scaling for testing the signaling only
  generatePointCloudParameters.removeDuplicatePoints_        = context.getRemoveDuplicatePoints();
  generatePointCloudParameters.oneLayerMode_                 = context.getOneLayerMode();
  generatePointCloudParameters.singleLayerPixelInterleaving_ = context.getSingleLayerPixelInterleaving();
  generatePointCloudParameters.sixDirectionMode_             = context.getSixDirectionMode();
  generatePointCloudParameters.improveEDD_                   = context.getImproveEDD();
  generatePointCloudParameters.path_                         = path.str();
  generatePointCloudParameters.useAdditionalPointsPatch_     = context.getUseAdditionalPointsPatch();

  generatePointCloud( reconstructs, context, generatePointCloudParameters );

  if (!context.getNoAttributes() ) {
    const size_t nbyteTexture = 1;
    auto& videoBitstream = context.getVideoBitstream( PCCVideoType::Texture );
    videoDecoder.decompress( context.getVideoTexture(),
                             path.str() + "texture",  context.getWidth(),  context.getHeight(),
                             context.size() * frameCountTexture, videoBitstream,
                             params_.videoDecoderPath_,
                             context,
                             params_.inverseColorSpaceConversionConfig_,
                             params_.colorSpaceConversionPath_,
                              context.getLosslessTexture() != 0, params_.patchColorSubsampling_, nbyteTexture,
                             params_.keepIntermediateFiles_ );
    std::cout << "texture video  ->" << videoBitstream.naluSize() << " B" << std::endl;

    if( context.getUseAdditionalPointsPatch() && context.getUseMissedPointsSeparateVideo()) {
      auto& videoBitstreamMP = context.getVideoBitstream( PCCVideoType::TextureMP );
      videoDecoder.decompress( context.getVideoMPsTexture(),
                               path.str() + "mps_texture",
                               context.getMPAttWidth(), context.getMPAttHeight(),
                               context.size(), videoBitstreamMP, //frameCount*2??
                               params_.videoDecoderPath_,
                               context,
                               params_.inverseColorSpaceConversionConfig_,
                               params_.colorSpaceConversionPath_,
                               context.getLosslessTexture(),                // && ! params_.lossyMissedPointsPatch_,
                               0, //params_.patchColorSubsampling_,
                               nbyteTexture, //nbyteTexture,
                               params_.keepIntermediateFiles_ );
      generateMissedPointsTexturefromVideo(context, reconstructs);
      std::cout << " missed points texture -> " << videoBitstreamMP.naluSize() << " B"<<endl;
    }
  }
  colorPointCloud(reconstructs, context, context.getNoAttributes() != 0, params_.colorTransform_,
                  generatePointCloudParameters);
  return 0;
}


