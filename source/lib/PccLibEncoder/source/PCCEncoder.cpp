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
#include "ArithmeticCodec.h"
#include "PCCBitstream.h"
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCPatch.h"
#include "PCCPatchSegmenter.h"
#include "PCCVideoEncoder.h"
#include "PCCSystem.h"
#include "PCCGroupOfFrames.h"
#include "PCCPointSet.h"
#include "PCCEncoderParameters.h"
#include "PCCKdTree.h"
#include <tbb/tbb.h>

#include "PCCEncoder.h"

using namespace std;
using namespace pcc;

std::string getEncoderConfig1L( std::string string ) {
  std::string sub = string.substr( 0, string.find_last_of(".") );
  std::string result = sub + "-1L.cfg";
  return result;
}

void EncodeUInt32( const uint32_t value, const uint32_t bitCount,
                   o3dgc::Arithmetic_Codec &arithmeticEncoder, o3dgc::Static_Bit_Model &bModel0) {
  uint32_t valueToEncode = PCCToLittleEndian<uint32_t>(value);
  for (uint32_t i = 0; i < bitCount; ++i) {
    arithmeticEncoder.encode(valueToEncode & 1, bModel0);
    valueToEncode >>= 1;
  }
}

PCCEncoder::PCCEncoder(){
}

PCCEncoder::~PCCEncoder(){
}

void PCCEncoder::setParameters( PCCEncoderParameters params ) { 
  params_ = params; 
}

int PCCEncoder::encode( const PCCGroupOfFrames& sources, PCCContext &context,
                        PCCBitstream &bitstream, PCCGroupOfFrames& reconstructs ){
  int ret = 0;
  size_t oneLayerModeOriginal                 = params_.oneLayerMode_;
  size_t singleLayerPixelInterleavingOriginal = params_.singleLayerPixelInterleaving_;
  if( params_.nbThread_ > 0 ) {
    tbb::task_scheduler_init init( (int)params_.nbThread_ );
  }
  printf("PCCEncoder: encode (start) \n"); fflush(stdout);
  ret |= encode( sources, context, reconstructs );
  printf("PCCEncoder: encode (done) \n"); fflush(stdout);
  printf("PCCEncoder: compress (start) \n"); fflush(stdout);
  ret |= compress( context, bitstream );
  printf("PCCEncoder: compress (done) \n"); fflush(stdout);
  params_.oneLayerMode_                = oneLayerModeOriginal;
  params_.singleLayerPixelInterleaving_= singleLayerPixelInterleavingOriginal;
  return ret; 
}

int PCCEncoder::compress( PCCContext &context, PCCBitstream &bitstream ){
  auto sizeFrameHeader = bitstream.size();
  printf("PCCEncoder: compress (start: bitstream = %lu / %lu ) \n",
         bitstream.size(), bitstream.capacity()); fflush(stdout);
  compressHeader( context, bitstream );
  printf("PCCEncoder: compress (header: bitstream = %lu / %lu ) \n",
         bitstream.size(), bitstream.capacity()); fflush(stdout);
  sizeFrameHeader = bitstream.size() - sizeFrameHeader;
  std::cout << "frame header info  ->" << sizeFrameHeader << " B " << std::endl;
  context.traceVideoBitstream();

  printf("PCCEncoder: compress (video bitstream start: bitstream = %lu / %lu ) \n",
         bitstream.size(), bitstream.capacity()); fflush(stdout);
  if( params_.useOccupancyMapVideo_ ) {
    bitstream.write( context.getVideoBitstream( PCCVideoType::OccupancyMap ) );
  }
  if (!params_.absoluteD1_) {
    bitstream.write( context.getVideoBitstream( PCCVideoType::GeometryD0 ) );
    bitstream.write( context.getVideoBitstream( PCCVideoType::GeometryD1 ) );
  } else {
    bitstream.write( context.getVideoBitstream(  PCCVideoType::Geometry ) );
  }
  if( params_.losslessGeo_ && context.getUseMissedPointsSeparateVideo()) {
    bitstream.write( context.getVideoBitstream(  PCCVideoType::GeometryMP ) );
  }
  if (!params_.noAttributes_ ) {
    bitstream.write( context.getVideoBitstream( PCCVideoType::Texture ) );
    if(params_.losslessTexture_ && context.getUseMissedPointsSeparateVideo()) {
      bitstream.write( context.getVideoBitstream( PCCVideoType::TextureMP ) );
      auto sizeMissedPointsTexture = bitstream.size();
     }
  }
  printf("PCCEncoder: compress (video bitstream done: bitstream = %lu / %lu ) \n",
         bitstream.size(), bitstream.capacity()); fflush(stdout);

  size_t sizeOccupancyMap = bitstream.size();
  compressOccupancyMap(context, bitstream);

  printf("PCCEncoder: compress (metadata done: bitstream = %lu / %lu ) \n",
         bitstream.size(),bitstream.capacity()); fflush(stdout);
  sizeOccupancyMap = ( bitstream.size() - sizeOccupancyMap )
      + context.getVideoBitstream( PCCVideoType::OccupancyMap ).naluSize();
  std::cout << " occupancy map  ->" << sizeOccupancyMap << " B " << std::endl;

  if (params_.losslessGeo_ && context.getUseMissedPointsSeparateVideo()) {
     writeMissedPointsGeometryNumber( context, bitstream );
  }
  if( !params_.noAttributes_ ) {
     if(params_.losslessTexture_ && context.getUseMissedPointsSeparateVideo())  {
       writeMissedPointsTextureNumber( context, bitstream );
    }
  }
  printf("PCCEncoder: compress (done: bitstream = %lu / %lu ) \n",
         bitstream.size(),bitstream.capacity()); fflush(stdout);
  return 0; 
}

int PCCEncoder::encode( const PCCGroupOfFrames& sources, PCCContext &context,
                        PCCGroupOfFrames& reconstructs ){
  assert( sources.size() < 256);
  if( sources.size() == 0 ) {
    return 0;
  }
  auto sizeGeoVideo = 0;
  reconstructs.resize( sources.size() );
  context.resize( sources.size() );
  auto& frames = context.getFrames();
  auto &frameLevelMetadataEnabledFlags = context.getGOFLevelMetadata().getLowerLevelMetadataEnabledFlags();
  if (frameLevelMetadataEnabledFlags.getMetadataEnabled()) {
    for (size_t i = 0; i < frames.size(); i++) {
      // Place to get/set frame-level metadata.
      PCCMetadata frameLevelMetadata;
      frames[i].getFrameLevelMetadata() = frameLevelMetadata;
      frames[i].getFrameLevelMetadata().getMetadataEnabledFlags() = frameLevelMetadataEnabledFlags;
      // Place to get/set patch metadata enabled flags (in frame level).
      PCCMetadataEnabledFlags patchLevelMetadataEnabledFlags;
      frames[i].getFrameLevelMetadata().getLowerLevelMetadataEnabledFlags() = patchLevelMetadataEnabledFlags;
    }
  }

  for(size_t i=0; i<frames.size(); i++)  {
    frames[i].setLosslessGeo(params_.losslessGeo_);
    frames[i].setLosslessGeo444(params_.losslessGeo444_);
    frames[i].setLosslessAtt(params_.losslessTexture_);
    frames[i].setEnhancedDeltaDepth(params_.enhancedDeltaDepthCode_);
    frames[i].setUseMissedPointsSeparateVideo(params_.useMissedPointsSeparateVideo_);
    frames[i].setUseOccupancyMapVideo(params_.useOccupancyMapVideo_);
  }

  PCCVideoEncoder videoEncoder;
  const size_t pointCount = sources[ 0 ].getPointCount();
  std::stringstream path;
  path << removeFileExtension( params_.compressedStreamPath_ ) << "_GOF" << context.getIndex() << "_";

  generateGeometryVideo( sources, context );

  const size_t nbFramesGeometry = params_.oneLayerMode_  ? 1 : 2;
  const size_t nbFramesTexture  = params_.oneLayerMode_  ? 1 : 2;
  resizeGeometryVideo( context );
  dilateGeometryVideo( context );
  auto& width  = context.getWidth ();
  auto& height = context.getHeight();
  width  = (uint16_t)frames[0].getWidth ();
  height = (uint16_t)frames[0].getHeight();

  if(params_.useOccupancyMapVideo_) {
    auto& videoBitstream = context.createVideoBitstream( PCCVideoType::OccupancyMap );
    generateOccupancyMapVideo(sources, context);
    auto& videoOccupancyMap = context.getVideoOccupancyMap();
    videoEncoder.compress( videoOccupancyMap, path.str() + "occupancy", params_.occupancyMapQP_, videoBitstream,
                           params_.occupancyMapVideoEncoderConfig_, params_.videoEncoderOccupancyMapPath_, context
                           , "", "", "", false, false,
                           params_.flagColorSmoothing_, params_.flagColorPreSmoothing_, 1, params_.keepIntermediateFiles_);
  }
  // Group dilation in Geometry
  if (params_.groupDilation_ && params_.absoluteD1_ && !params_.oneLayerMode_){
    auto& VideoGemetry = context.getVideoGeometry();
    if( !params_.oneLayerMode_ ) {
      for (size_t f = 0; f < frames.size(); ++f) {
        auto& frame = frames[f];
        auto& occupancyMap = frame.getOccupancyMap();
        auto  &frame1 = VideoGemetry.getFrame(2*f);
        auto  &frame2 = VideoGemetry.getFrame(2*f +1);
        uint16_t  tmp_d0, tmp_d1;
        uint32_t  tmp_avg;
        for (size_t y = 0; y < height; y++) {
          for (size_t x = 0; x < width; x++) {
            const size_t pos = y * width + x;
            if (occupancyMap[pos] == 0) {
              tmp_d0 = frame1.getValue(0, x, y);
              tmp_d1 = frame2.getValue(0, x, y);
              tmp_avg = ((uint32_t)tmp_d0 + (uint32_t)tmp_d1 + 1) >> 1;
              frame1.setValue(0, x, y, (uint16_t)tmp_avg);
              frame2.setValue(0, x, y, (uint16_t)tmp_avg);
            }
          }
        }
      }
    }
  }

  const size_t nbyteGeo = params_.losslessGeo_ ? 2 : 1;
  if (!params_.absoluteD1_) {
    // Compress geometryD0
    auto& videoBitstreamD0 = context.createVideoBitstream( PCCVideoType::GeometryD0 );
    auto& videoGeometry = context.getVideoGeometry();
    videoEncoder.compress( videoGeometry, path.str() + "geometryD0", (params_.geometryQP_-1), videoBitstreamD0,
                           params_.geometryD0Config_, params_.videoEncoderPath_, context,
                           "", "", "", params_.losslessGeo_ && params_.losslessGeo444_, false,
                           params_.flagColorSmoothing_,
                           params_.flagColorPreSmoothing_,
                           nbyteGeo, params_.keepIntermediateFiles_);
    // Form differential video geometryD1
    auto& videoGeometryD1 = context.getVideoGeometryD1();
    for (size_t f = 0; f < frames.size(); ++f) {
      auto &frame1 = videoGeometryD1.getFrame(f);
      predictGeometryFrame( frames[f], videoGeometry.getFrame(f), frame1 );
    }

    // Compress geometryD1
    auto& videoBitstreamD1 = context.createVideoBitstream( PCCVideoType::GeometryD1 );
    videoEncoder.compress( videoGeometryD1, path.str() + "geometryD1", params_.geometryQP_,
                           videoBitstreamD1,
                           params_.geometryD1Config_, params_.videoEncoderPath_, context,
                           "", "", "", params_.losslessGeo_ && params_.losslessGeo444_, false,
                           params_.flagColorSmoothing_,
                           params_.flagColorPreSmoothing_,
                           nbyteGeo, params_.keepIntermediateFiles_);

    auto sizeGeometryVideo = videoBitstreamD0.naluSize()  + videoBitstreamD1.naluSize();
    std::cout << "geometry video ->" << sizeGeometryVideo << " B ("
        << (sizeGeometryVideo * 8.0) / (2 * frames.size() * pointCount) << " bpp)"
        << std::endl;
  } else {
    auto& videoBitstream = context.createVideoBitstream( PCCVideoType::Geometry );
    auto& videoGeometry = context.getVideoGeometry();  
    videoEncoder.compress( videoGeometry, path.str() + "geometry", params_.geometryQP_,
                           videoBitstream,
                           params_.oneLayerMode_ ? getEncoderConfig1L( params_.geometryConfig_ ) : params_.geometryConfig_,
                               params_.videoEncoderPath_, context,
                               "", "", "",
                               params_.losslessGeo_ && params_.losslessGeo444_, false,
                               params_.flagColorSmoothing_,
                               params_.flagColorPreSmoothing_,
                               nbyteGeo, params_.keepIntermediateFiles_ );
  }

  if (params_.losslessGeo_ && context.getUseMissedPointsSeparateVideo()) {
    auto& videoBitstreamMP = context.createVideoBitstream( PCCVideoType::GeometryMP );
    generateMissedPointsGeometryVideo(context, reconstructs);
    auto& videoMPsGeometry = context.getVideoMPsGeometry();
    videoEncoder.compress( videoMPsGeometry, path.str() + "mps_geo", (params_.geometryQP_),
                           videoBitstreamMP,
                           params_.geometryMPConfig_, params_.videoEncoderPath_, context,
                           "", "", "", 0, //params_.losslessGeo_ && params_.losslessGeo444_=0
                           false, //patchColorSubsampling
                           params_.flagColorSmoothing_,
                           params_.flagColorPreSmoothing_,
                           2, params_.keepIntermediateFiles_); //nByteGeo=2 (10 bit coding)
  }
  buildBlockToPatch( context );

  GeneratePointCloudParameters generatePointCloudParameters;
  generatePointCloudParameters.occupancyResolution_          = params_.occupancyResolution_;
  generatePointCloudParameters.occupancyPrecision_           = params_.occupancyPrecision_;
  generatePointCloudParameters.gridSmoothing_                = params_.gridSmoothing_;
  generatePointCloudParameters.neighborCountSmoothing_       = params_.neighborCountSmoothing_;
  generatePointCloudParameters.radius2Smoothing_             = params_.radius2Smoothing_;
  generatePointCloudParameters.radius2BoundaryDetection_     = params_.radius2BoundaryDetection_;
  generatePointCloudParameters.thresholdSmoothing_           = params_.thresholdSmoothing_;
  generatePointCloudParameters.losslessGeo_                  = params_.losslessGeo_;
  generatePointCloudParameters.losslessGeo444_               = params_.losslessGeo444_;
  generatePointCloudParameters.nbThread_                     = params_.nbThread_;
  generatePointCloudParameters.absoluteD1_                   = params_.absoluteD1_;
  generatePointCloudParameters.surfaceThickness              = params_.surfaceThickness_;
  generatePointCloudParameters.ignoreLod_                    = true;
  generatePointCloudParameters.thresholdColorSmoothing_      = params_.thresholdColorSmoothing_;
  generatePointCloudParameters.thresholdLocalEntropy_        = params_.thresholdLocalEntropy_;
  generatePointCloudParameters.radius2ColorSmoothing_        = params_.radius2ColorSmoothing_;
  generatePointCloudParameters.neighborCountColorSmoothing_  = params_.neighborCountColorSmoothing_;
  generatePointCloudParameters.flagColorSmoothing_           = params_.flagColorSmoothing_;
  generatePointCloudParameters.enhancedDeltaDepthCode_       = (params_.losslessGeo_ ? params_.enhancedDeltaDepthCode_ : false);
  generatePointCloudParameters.deltaCoding_                  = (params_.testLevelOfDetailSignaling_ > 0);
  generatePointCloudParameters.removeDuplicatePoints_        = params_.removeDuplicatePoints_;
  generatePointCloudParameters.oneLayerMode_                 = params_.oneLayerMode_;
  generatePointCloudParameters.singleLayerPixelInterleaving_ = params_.singleLayerPixelInterleaving_;
  generatePointCloudParameters.sixDirectionMode_             = params_.sixDirectionMode_;
  generatePointCloudParameters.improveEDD_		             = params_.improveEDD_;
  generatePointCloudParameters.path_                         = path.str();

  for( auto& frame : context.getFrames() ) {
    const size_t width              = frame.getWidth();
    const size_t height             = frame.getHeight();
    const size_t blockToPatchWidth  = width  / params_.occupancyResolution_;
    const size_t blockToPatchHeight = height / params_.occupancyResolution_;
    const size_t blockCount         = blockToPatchWidth * blockToPatchHeight;
    auto& interpolateMap = frame.getInterpolate();
    auto& fillingMap     = frame.getFilling();
    auto& minD1Map       = frame.getMinD1();
    auto& neighborMap    = frame.getNeighbor();
    interpolateMap.resize( 0 );
    fillingMap    .resize( 0 );
    minD1Map      .resize( 0 );
    neighborMap   .resize( 0 );
    interpolateMap.resize( blockCount, (bool)0 );
    fillingMap    .resize( blockCount, (bool)0 );
    minD1Map      .resize( blockCount, 0       );
    neighborMap   .resize( blockCount, 1       );
  }
  if( params_.absoluteD1_ && params_.oneLayerMode_ && !params_.singleLayerPixelInterleaving_) {
    reconsctuctionOptimization( context, generatePointCloudParameters );
  }
  generatePointCloud( reconstructs, context, generatePointCloudParameters );

  if( !params_.noAttributes_ ) {
    generateTextureVideo(sources, reconstructs, context, params_);
    auto& videoTexture = context.getVideoTexture();
    const size_t textureFrameCount = nbFramesTexture * frames.size();
    assert(textureFrameCount == videoTexture.getFrameCount());
    if ( !(params_.losslessGeo_ && params_.textureDilationOffLossless_)) {
      for (size_t f = 0; f < textureFrameCount; ++f) {
        if(params_.textureBGFill_ == 1) {
          dilatePullPush( frames[ f / nbFramesTexture ], videoTexture.getFrame( f ) );
        } else {
          dilate( frames[ f / nbFramesTexture ], videoTexture.getFrame( f ) );
        }
        if ((params_.groupDilation_ == true) && ((f & 0x1) == 1)) {
          if( !params_.oneLayerMode_ ) {
            // Group dilation in texture
            auto & frame = frames[f / 2];
            auto& occupancyMap = frame.getOccupancyMap();
            auto& width = frame.getWidth();
            auto& height = frame.getHeight();
            auto  &frame1 = videoTexture.getFrame(f - 1);
            auto  &frame2 = videoTexture.getFrame(f);
            uint8_t  tmp_d0, tmp_d1;
            uint32_t tmp_avg;
            for (size_t y = 0; y < height; y++) {
              for (size_t x = 0; x < width; x++) {
                const size_t pos = y * width + x;
                if (occupancyMap[pos] == 0) {
                  for (size_t c = 0; c < 3; c++) {
                    tmp_d0 = frame1.getValue(c, x, y);
                    tmp_d1 = frame2.getValue(c, x, y);
                    tmp_avg = ((uint32_t)tmp_d0 + (uint32_t)tmp_d1 + 1) >> 1;
                    frame1.setValue(c, x, y, (uint8_t)tmp_avg);
                    frame2.setValue(c, x, y, (uint8_t)tmp_avg);
                  }
                }
              }
            }
          }
        }
      }
    }

    const size_t nbyteTexture = 1;
    auto& videoBitstream = context.createVideoBitstream( PCCVideoType::Texture );
    videoEncoder.compress( videoTexture,path.str() + "texture", params_.textureQP_,
                           videoBitstream,
                           params_.oneLayerMode_ ? getEncoderConfig1L( params_.textureConfig_ ) : params_.textureConfig_,
                               params_.videoEncoderPath_, context,
                               params_.colorSpaceConversionConfig_,
                               params_.inverseColorSpaceConversionConfig_,
                               params_.colorSpaceConversionPath_, params_.losslessTexture_, params_.patchColorSubsampling_,
                               params_.flagColorSmoothing_, params_.flagColorPreSmoothing_, nbyteTexture,
                               params_.keepIntermediateFiles_ );

    if(params_.losslessTexture_ && context.getUseMissedPointsSeparateVideo())  {
      auto& videoBitstreamMP = context.createVideoBitstream( PCCVideoType::TextureMP );
      generateMissedPointsTextureVideo(context, reconstructs );//1. texture
      auto& videoMPsTexture = context.getVideoMPsTexture();
      videoEncoder.compress( videoMPsTexture,path.str() + "mps_tex", params_.textureQP_,
                             videoBitstreamMP,
                             params_.textureMPConfig_,
                             params_.videoEncoderPath_, context,
                             params_.colorSpaceConversionConfig_,
                             params_.inverseColorSpaceConversionConfig_,
                             params_.colorSpaceConversionPath_, 1, //params_.losslessTexture_=1
                             false, //params_.patchColorSubsampling_,
                             params_.flagColorSmoothing_, params_.flagColorPreSmoothing_, nbyteTexture,
                             params_.keepIntermediateFiles_ );
    }
  }
  colorPointCloud(reconstructs, context, params_.noAttributes_ != 0, params_.colorTransform_, generatePointCloudParameters);
  return 0;
}

void PCCEncoder::printMap(std::vector<bool> img, const size_t sizeU, const size_t sizeV) {
  std::cout << std::endl;
  std::cout << "PrintMap size = " << sizeU << " x " << sizeV << std::endl;
  for (size_t v = 0; v < sizeV; ++v) {
    for (size_t u = 0; u < sizeU; ++u) {
      std::cout << (img[v * sizeU + u] ? 'X' : '.');
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

bool PCCEncoder::generateOccupancyMapVideo( const PCCGroupOfFrames& sources, PCCContext& context ) {
  auto& videoOccupancyMap = context.getVideoOccupancyMap();
  bool ret = true;
  videoOccupancyMap.resize(sources.size());
  for (size_t f = 0; f < sources.size(); ++f) {
    auto &contextFrame = context.getFrames()[f];
    PCCImageOccupancyMap &videoFrame = videoOccupancyMap.getFrame(f);
    ret &= generateOccupancyMapVideo(contextFrame.getWidth(), contextFrame.getHeight(), contextFrame.getOccupancyMap(), videoFrame);
  }
  return ret;
}

bool PCCEncoder::generateOccupancyMapVideo( const size_t imageWidth, const size_t imageHeight,
                                            std::vector<uint32_t> &occupancyMap,
                                            PCCImageOccupancyMap &videoFrameOccupancyMap) {
  const size_t blockSize0 = params_.occupancyResolution_ / params_.occupancyPrecision_;
  const size_t pointCount0 = blockSize0 * blockSize0;
  vector<bool> block0;
  vector<size_t> bestRuns;
  vector<size_t> runs;
  block0.resize(pointCount0);
  size_t videoFrameOccupancyMapSizeU = imageWidth/params_.occupancyPrecision_;
  size_t videoFrameOccupancyMapSizeV = imageHeight/params_.occupancyPrecision_;
  const size_t blockToPatchWidth = imageWidth / params_.occupancyResolution_;
  const size_t blockToPatchHeight = imageHeight / params_.occupancyResolution_;

  if (!params_.improveEDD_) {
  videoFrameOccupancyMap.resize(videoFrameOccupancyMapSizeU, videoFrameOccupancyMapSizeV);
  for (size_t v0 = 0; v0 < blockToPatchHeight; ++v0) {
    for (size_t u0 = 0; u0 < blockToPatchWidth; ++u0) {
      size_t fullCount = 0;
      for (size_t v1 = 0; v1 < blockSize0; ++v1) {
        const size_t v2 = v0 * params_.occupancyResolution_ + v1 * params_.occupancyPrecision_;
        for (size_t u1 = 0; u1 < blockSize0; ++u1) {
          const size_t u2 = u0 * params_.occupancyResolution_ + u1 * params_.occupancyPrecision_;
          bool isFull = false;
          for (size_t v3 = 0; v3 < params_.occupancyPrecision_ && !isFull; ++v3) {
            for (size_t u3 = 0; u3 < params_.occupancyPrecision_ && !isFull; ++u3) {
              isFull |= occupancyMap[(v2 + v3) * imageWidth + u2 + u3] == 1;
            }
          }
          block0[v1 * blockSize0 + u1] = isFull;
          fullCount += isFull;
          for (size_t v3 = 0; v3 < params_.occupancyPrecision_; ++v3) {
            for (size_t u3 = 0; u3 < params_.occupancyPrecision_; ++u3) {
              occupancyMap[(v2 + v3) * imageWidth + u2 + u3] = isFull;
            }
          }
        }
      }
      for (size_t iterBlockV=0; iterBlockV<blockSize0; iterBlockV++) {
        for (size_t iterBlockU=0; iterBlockU<blockSize0; iterBlockU++) {
          uint8_t pixel = block0[iterBlockV*blockSize0+iterBlockU];
          size_t videoFrameU = u0*blockSize0 + iterBlockU;
          size_t videoFrameV = v0*blockSize0 + iterBlockV;
          videoFrameOccupancyMap.setValue(0, videoFrameU, videoFrameV, pixel);
        }
      }
    }
  }
  } else {
	  videoFrameOccupancyMap.resize(imageWidth, imageHeight);
	  for (size_t v = 0; v < imageHeight; v++) {
		  for (size_t u = 0; u < imageWidth; u++) {
			  size_t i = v * imageWidth + u;
			  size_t symbol = occupancyMap[i];
			  if (symbol < 0) symbol = 0;
			  if (symbol > 1023) symbol = 1023;
			  videoFrameOccupancyMap.setValue(0, u, v, symbol);
		  }
	  }
  }
  return true;
}

void PCCEncoder::spatialConsistencyPack(PCCFrameContext& frame, PCCFrameContext &prevFrame, int safeguard) {
  auto& width       = frame.getWidth();
  auto& height      = frame.getHeight();
  auto& patches     = frame.getPatches();
  auto& prevPatches = prevFrame.getPatches();
  if (patches.empty()) {
    return;
  }
  std::sort( patches.begin(), patches.end() );
  int id = 0;
  size_t occupancySizeU = params_.minimumImageWidth_ / params_.occupancyResolution_;
  size_t occupancySizeV = (std::max)(params_.minimumImageHeight_ / params_.occupancyResolution_, patches[0].getSizeV0());
  vector<PCCPatch> matchedPatches, tmpPatches;
  matchedPatches.clear();
  float thresholdIOU = 0.2;

  //main loop.
  for (auto &patch : prevPatches) {
    assert(patch.getSizeU0() <= occupancySizeU);
    assert(patch.getSizeV0() <= occupancySizeV);
    id++;
    float maxIou = 0.0; int bestIdx = -1, cId = 0;
    for (auto &cpatch : patches) {
      if ((patch.getViewId() == cpatch.getViewId()) && (cpatch.getBestMatchIdx() == -1)) {
        Rect rect = Rect(patch.getU1(), patch.getV1(), patch.getSizeU(), patch.getSizeV());
        Rect crect = Rect(cpatch.getU1(), cpatch.getV1(), cpatch.getSizeU(), cpatch.getSizeV());
        float iou = computeIOU(rect, crect);
        if (iou > maxIou) {
          maxIou = iou;
          bestIdx = cId;
        }
      }//end of if (patch.viewId == cpatch.viewId).
      cId++;
    }
    if (maxIou > thresholdIOU) {
      //store the best match index
      patches[bestIdx].setBestMatchIdx() = id - 1; //the matched patch id in previous frame.
      matchedPatches.push_back(patches[bestIdx]);
    }
  }

  //generate new patch order.
  vector<PCCPatch> newOrderPatches = matchedPatches;
  for (auto patch : patches) {
    assert(patch.getSizeU0() <= occupancySizeU);
    assert(patch.getSizeV0() <= occupancySizeV);
    if (patch.getBestMatchIdx() == -1) {
      newOrderPatches.push_back(patch);
    }
  }

  frame.setNumMatchedPatches() = matchedPatches.size();
  //remove the below logs when useless.
  patches = newOrderPatches;
  for (auto &patch : patches) { occupancySizeU = (std::max)( occupancySizeU, patch.getSizeU0() + 1); }

  width  = occupancySizeU * params_.occupancyResolution_;
  height = occupancySizeV * params_.occupancyResolution_;
  size_t maxOccupancyRow{0};

  std::vector<bool> occupancyMap;
  occupancyMap.resize( occupancySizeU * occupancySizeV, false );
  for (auto &patch : patches) {
    assert(patch.getSizeU0() <= occupancySizeU);
    assert(patch.getSizeV0() <= occupancySizeV);
    bool locationFound = false;
    auto& occupancy =  patch.getOccupancy();
    while (!locationFound) {
      patch.getPatchOrientation() = 0; // only one orientation is allowed
      for (int v = 0; v <= occupancySizeV && !locationFound; ++v) {
        for (int u = 0; u <= occupancySizeU && !locationFound; ++u) {
          patch.getU0() = u;
          patch.getV0() = v;
          if (patch.checkFitPatchCanvas(occupancyMap, occupancySizeU, occupancySizeV, safeguard)) {
            locationFound = true;}
        }
      }
      if (!locationFound) {
        occupancySizeV *= 2;
        occupancyMap.resize( occupancySizeU * occupancySizeV );
      }
    }
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      const size_t v = patch.getV0() + v0;
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        const size_t u = patch.getU0() + u0;
        occupancyMap[v * occupancySizeU + u] =
            occupancyMap[v * occupancySizeU + u] || occupancy[v0 * patch.getSizeU0() + u0];
      }
    }

    height = (std::max)( height, (patch.getV0() + patch.getSizeV0()) * patch.getOccupancyResolution() );
    width  = (std::max)( width,  (patch.getU0() + patch.getSizeU0()) * patch.getOccupancyResolution() );
    maxOccupancyRow = (std::max)(maxOccupancyRow, (patch.getV0() + patch.getSizeV0()));
    // print(occupancyMap, occupancySizeU, occupancySizeV);
  }

  if (frame.getMissedPointsPatch().size() > 0 && !frame.getUseMissedPointsSeparateVideo()) {
    packMissedPointsPatch(frame, occupancyMap, width, height, occupancySizeU, occupancySizeV, maxOccupancyRow);
  } else {
    if(printDetailedInfo) {
      printMap(occupancyMap, occupancySizeU, occupancySizeV);
    }
  }
  std::cout << "actualImageSizeU " << width  << std::endl;
  std::cout << "actualImageSizeV " << height << std::endl;
}

void PCCEncoder::spatialConsistencyPackFlexible(PCCFrameContext& frame, PCCFrameContext &prevFrame, int safeguard) {
  auto& width = frame.getWidth();
  auto& height = frame.getHeight();
  auto& patches = frame.getPatches();

  auto& prevPatches = prevFrame.getPatches();
  if (patches.empty()) {
    return;
  }
  std::sort(patches.begin(), patches.end(), [](PCCPatch& a, PCCPatch& b) { return a.gt(b); });
  int id = 0;
  size_t occupancySizeU = params_.minimumImageWidth_ / params_.occupancyResolution_;
  size_t occupancySizeV = (std::max)(patches[0].getSizeU0(), patches[0].getSizeV0());
  vector<PCCPatch> matchedPatches, tmpPatches;
  matchedPatches.clear();
  float thresholdIOU = 0.2;

  //main loop.
  for (auto &patch : prevPatches) {
    assert(patch.getSizeU0() <= occupancySizeU);
    assert(patch.getSizeV0() <= occupancySizeV);
    id++;
    float maxIou = 0.0; int bestIdx = -1, cId = 0;
    for (auto &cpatch : patches) {
      if ((patch.getViewId() == cpatch.getViewId()) && (cpatch.getBestMatchIdx() == -1)) {
        Rect rect = Rect(patch.getU1(), patch.getV1(), patch.getSizeU(), patch.getSizeV());
        Rect crect = Rect(cpatch.getU1(), cpatch.getV1(), cpatch.getSizeU(), cpatch.getSizeV());
        float iou = computeIOU(rect, crect);
        if (iou > maxIou) {
          maxIou = iou;
          bestIdx = cId;
        }
      } //end of if (patch.viewId == cpatch.viewId).
      cId++;
    }

    if (maxIou > thresholdIOU) {
      //store the best match index
      patches[bestIdx].setBestMatchIdx() = id - 1; //the matched patch id in preivious frame.
      matchedPatches.push_back(patches[bestIdx]);
    }
  }

  //generate new patch order.
  vector<PCCPatch> newOrderPatches = matchedPatches;

  for (auto patch : patches) {
    assert(patch.getSizeU0() <= occupancySizeU);
    assert(patch.getSizeV0() <= occupancySizeV);
    if (patch.getBestMatchIdx() == -1) {
      newOrderPatches.push_back(patch);
    }
  }
  frame.setNumMatchedPatches() = matchedPatches.size();

  //remove the below logs when useless.
  if(printDetailedInfo) {
    std::cout << "patches.size:" << patches.size() << ",reOrderedPatches.size:" << newOrderPatches.size() << ",matchedpatches.size:" << frame.getNumMatchedPatches() << std::endl;
  }
  patches = newOrderPatches;
  if (printDetailedInfo) {
    std::cout << "Patch order:" << std::endl;
    for (auto &patch : patches) { std::cout << "Patch[" << patch.getIndex() << "]=("<<patch.getSizeU0()<<","<<patch.getSizeV0()<<")"<< std::endl; }
  }

  for (auto &patch : patches) { occupancySizeU = (std::max)(occupancySizeU, patch.getSizeU0() + 1); }

  width = occupancySizeU * params_.occupancyResolution_;
  height = occupancySizeV * params_.occupancyResolution_;
  size_t maxOccupancyRow{ 0 };

  vector<int> orientation_vertical  = { 0,1,2,3,4,6,5,7 };//favoring vertical orientation
  vector<int> orientation_horizontal = { 1,0,3,2,5,7,4,6 };//favoring horizontal orientations (that should be rotated) 
  std::vector<bool> occupancyMap;
  occupancyMap.resize(occupancySizeU * occupancySizeV, false);
  for (auto &patch : patches) {
    assert(patch.getSizeU0() <= occupancySizeU);
    assert(patch.getSizeV0() <= occupancySizeV);
    bool locationFound = false;
    auto& occupancy = patch.getOccupancy();
    while (!locationFound) {
      if (patch.getBestMatchIdx() != -1) {
        patch.getPatchOrientation() = prevPatches[patch.getBestMatchIdx()].getPatchOrientation();
        //try to place on the same position as the matched patch
        patch.getU0() = prevPatches[patch.getBestMatchIdx()].getU0();
        patch.getV0() = prevPatches[patch.getBestMatchIdx()].getV0();
        if (patch.checkFitPatchCanvas(occupancyMap, occupancySizeU, occupancySizeV)) {
          locationFound = true;
          if (printDetailedInfo) {
            std::cout << "Maintained orientation " << patch.getPatchOrientation() << " for matched patch " << patch.getIndex() << " in the same position (" << patch.getU0() << "," << patch.getV0() << ")" << std::endl;
          }
        }
        //if the patch couldn't fit, try to fit the patch in the top left position
        for (int v = 0; v <= occupancySizeV && !locationFound; ++v) {
          for (int u = 0; u <= occupancySizeU && !locationFound; ++u) {
            patch.getU0() = u;
            patch.getV0() = v;
            if (patch.checkFitPatchCanvas(occupancyMap, occupancySizeU, occupancySizeV, safeguard)) {
              locationFound = true;
              if(printDetailedInfo) {
                std::cout << "Maintained orientation " << patch.getPatchOrientation() << " for matched patch " << patch.getIndex() << " (" << u << "," << v << ")" << std::endl;
              }
            }
          }
        }
      } else {
        //best effort
        for (size_t v = 0; v < occupancySizeV && !locationFound; ++v) {
          for (size_t u = 0; u < occupancySizeU && !locationFound; ++u) {
            patch.getU0() = u;
            patch.getV0() = v;
            for (size_t orientationIdx = 0; orientationIdx < NumPatchOrientations && !locationFound; orientationIdx++) {
              if (patch.getSizeU0() > patch.getSizeV0()) {
                patch.getPatchOrientation() = orientation_horizontal[orientationIdx];
              } else {
                patch.getPatchOrientation() = orientation_vertical[orientationIdx];
              }
              if (patch.checkFitPatchCanvas(occupancyMap, occupancySizeU, occupancySizeV, safeguard)) {
                locationFound = true;
                if (printDetailedInfo) {
                  std::cout << "Orientation " << patch.getPatchOrientation() << " selected for unmatched patch " << patch.getIndex() << " (" << u << "," << v << ")" << std::endl;
                }
              }
            }
          }
        }
      }
      if (!locationFound) {
        occupancySizeV *= 2;
        occupancyMap.resize(occupancySizeU * occupancySizeV);
      }
    }
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        int coord = patch.patchBlock2CanvasBlock(u0, v0, occupancySizeU, occupancySizeV);
        occupancyMap[coord] = occupancyMap[coord] || occupancy[v0 * patch.getSizeU0() + u0];
      }
    }
    if (patch.getPatchOrientation() == 0 || patch.getPatchOrientation() == 2 || patch.getPatchOrientation() == 4 || patch.getPatchOrientation() == 6) {
      height = (std::max)(height, (patch.getV0() + patch.getSizeV0()) * patch.getOccupancyResolution());
      width = (std::max)(width, (patch.getU0() + patch.getSizeU0()) * patch.getOccupancyResolution());
      maxOccupancyRow = (std::max)(maxOccupancyRow, (patch.getV0() + patch.getSizeV0()));
    } else {
      height = (std::max)(height, (patch.getV0() + patch.getSizeU0()) * patch.getOccupancyResolution());
      width = (std::max)(width, (patch.getU0() + patch.getSizeV0()) * patch.getOccupancyResolution());
      maxOccupancyRow = (std::max)(maxOccupancyRow, (patch.getV0() + patch.getSizeU0()));
    }
  }

  if (frame.getMissedPointsPatch().size() > 0 && !frame.getUseMissedPointsSeparateVideo()) {
    packMissedPointsPatch(frame, occupancyMap, width, height, occupancySizeU, occupancySizeV, maxOccupancyRow);
  } else {
    if (printDetailedInfo){
      printMap(occupancyMap, occupancySizeU, occupancySizeV);
    }
  }
  std::cout << "actualImageSizeU " << width << std::endl;
  std::cout << "actualImageSizeV " << height << std::endl;
}

void PCCEncoder::pack( PCCFrameContext& frame, int safeguard  ) {
  auto& width   = frame.getWidth(); 
  auto& height  = frame.getHeight(); 
  auto& patches = frame.getPatches();
  if (patches.empty()) {
    return;
  }
  std::sort( patches.begin(), patches.end() );
  size_t occupancySizeU = params_.minimumImageWidth_ / params_.occupancyResolution_;
  size_t occupancySizeV = (std::max)( params_.minimumImageHeight_ / params_.occupancyResolution_, patches[0].getSizeV0());
  for (auto &patch : patches) { occupancySizeU = (std::max)( occupancySizeU, patch.getSizeU0() + 1); }

  width  = occupancySizeU * params_.occupancyResolution_;
  height = occupancySizeV * params_.occupancyResolution_;
  size_t maxOccupancyRow{ 0 };

  std::vector<bool> occupancyMap;
  occupancyMap.resize( occupancySizeU * occupancySizeV, false );
  for (auto &patch : patches) {
    assert(patch.getSizeU0() <= occupancySizeU);
    assert(patch.getSizeV0() <= occupancySizeV);
    bool locationFound = false;
    auto& occupancy =  patch.getOccupancy();
    while (!locationFound) {
      patch.getPatchOrientation() = 0; //only allowed orientation in anchor
      for (int v = 0; v <= occupancySizeV && !locationFound; ++v) {
        for (int u = 0; u <= occupancySizeU && !locationFound; ++u) {
          patch.getU0() = u;
          patch.getV0() = v;
          if (patch.checkFitPatchCanvas(occupancyMap, occupancySizeU, occupancySizeV, safeguard)) {
            locationFound = true;
          }
        }
      }
      if (!locationFound) {
        occupancySizeV *= 2;
        occupancyMap.resize( occupancySizeU * occupancySizeV );
      }
    }
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      const size_t v = patch.getV0() + v0;
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        const size_t u = patch.getU0() + u0;
        occupancyMap[v * occupancySizeU + u] = occupancyMap[v * occupancySizeU + u] || occupancy[v0 * patch.getSizeU0() + u0];
      }
    }

    height = (std::max)( height, (patch.getV0() + patch.getSizeV0()) * patch.getOccupancyResolution() );
    width  = (std::max)( width,  (patch.getU0() + patch.getSizeU0()) * patch.getOccupancyResolution() );
    maxOccupancyRow = (std::max)(maxOccupancyRow, (patch.getV0() + patch.getSizeV0()));
    // print(occupancyMap, occupancySizeU, occupancySizeV);
  }

  if (frame.getMissedPointsPatch().size() > 0 && !frame.getUseMissedPointsSeparateVideo()) {
    packMissedPointsPatch(frame, occupancyMap, width, height, occupancySizeU, occupancySizeV, maxOccupancyRow);
  } else {
    if(printDetailedInfo) {
      printMap( occupancyMap, occupancySizeU, occupancySizeV );
    }
  }
  std::cout << "actualImageSizeU " << width  << std::endl;
  std::cout << "actualImageSizeV " << height << std::endl;
}

void PCCEncoder::packFlexible(PCCFrameContext& frame, int safeguard) {
  auto& width = frame.getWidth();
  auto& height = frame.getHeight();
  auto& patches = frame.getPatches();
  //set no matched patches, since this function does not take into account the previous frame
  frame.setNumMatchedPatches() = 0;
  if (patches.empty()) {
    return;
  }
  //sorting by patch largest dimension
  std::sort(patches.begin(), patches.end(), [](PCCPatch& a, PCCPatch& b) { return a.gt(b); });
  if (printDetailedInfo) {
    std::cout << "Patch order:" << std::endl;
    for (auto &patch : patches) { std::cout << "Patch[" << patch.getIndex() << "]=("<<patch.getSizeU0()<<","<<patch.getSizeV0()<<")"<< std::endl; }
  }

  size_t occupancySizeU = params_.minimumImageWidth_ / params_.occupancyResolution_;
  size_t occupancySizeV = (std::max)(patches[0].getSizeV0(), patches[0].getSizeU0());
  for (auto &patch : patches) { occupancySizeU = (std::max)(occupancySizeU, patch.getSizeU0() + 1); }

  width = occupancySizeU * params_.occupancyResolution_;
  height = occupancySizeV * params_.occupancyResolution_;
  size_t maxOccupancyRow{ 0 };

  std::vector<bool> occupancyMap;
  vector<int> orientation_vertical = {0,1,2,3,4,6,5,7};//favoring vertical orientation 
  vector<int> orientation_horizontal = {1,0,3,2,5,7,4,6};//favoring horizontal orientations (that should be rotated) 
  occupancyMap.resize(occupancySizeU * occupancySizeV, false);
  for (auto &patch : patches) {
    assert(patch.getSizeU0() <= occupancySizeU);
    assert(patch.getSizeV0() <= occupancySizeV);
    bool locationFound = false;
    auto& occupancy = patch.getOccupancy();
    while (!locationFound) {
      for (size_t v = 0; v < occupancySizeV && !locationFound; ++v) {
        for (size_t u = 0; u < occupancySizeU && !locationFound; ++u) {
          patch.getU0() = u;
          patch.getV0() = v;
          for (size_t orientationIdx = 0; orientationIdx < NumPatchOrientations && !locationFound; orientationIdx++) {
            if(patch.getSizeU0() > patch.getSizeV0()) {
              patch.getPatchOrientation() = orientation_horizontal[orientationIdx];
            } else {
              patch.getPatchOrientation() = orientation_vertical[orientationIdx];
            }
            if (patch.checkFitPatchCanvas(occupancyMap, occupancySizeU, occupancySizeV, safeguard)) {
              locationFound = true;
              if (printDetailedInfo) {
                std::cout << "Orientation " << patch.getPatchOrientation() << " selected for patch " << patch.getIndex() <<" (" << u << ","<<v<<")" << std::endl;
              }
            }
          }
        }
      }
      if (!locationFound) {
        occupancySizeV *= 2;
        occupancyMap.resize(occupancySizeU * occupancySizeV);
      }
    }
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        int coord = patch.patchBlock2CanvasBlock(u0, v0, occupancySizeU, occupancySizeV);
        occupancyMap[coord] = occupancyMap[coord] || occupancy[v0 * patch.getSizeU0() + u0];
      }
    }

    if (patch.getPatchOrientation() == 0 || patch.getPatchOrientation() == 2 || patch.getPatchOrientation() == 4 || patch.getPatchOrientation() == 6) {
      height = (std::max)(height, (patch.getV0() + patch.getSizeV0()) * patch.getOccupancyResolution());
      width = (std::max)(width, (patch.getU0() + patch.getSizeU0()) * patch.getOccupancyResolution());
      maxOccupancyRow = (std::max)(maxOccupancyRow, (patch.getV0() + patch.getSizeV0()));
    } else {
      height = (std::max)(height, (patch.getV0() + patch.getSizeU0()) * patch.getOccupancyResolution());
      width = (std::max)(width, (patch.getU0() + patch.getSizeV0()) * patch.getOccupancyResolution());
      maxOccupancyRow = (std::max)(maxOccupancyRow, (patch.getV0() + patch.getSizeU0()));
    }
  }

  if (frame.getMissedPointsPatch().size() > 0 && !frame.getUseMissedPointsSeparateVideo()) {
    packMissedPointsPatch(frame, occupancyMap, width, height, occupancySizeU, occupancySizeV, maxOccupancyRow);
  } else {
    if (printDetailedInfo) {
      printMap(occupancyMap, occupancySizeU, occupancySizeV);
    }
  }
  std::cout << "actualImageSizeU " << width << std::endl;
  std::cout << "actualImageSizeV " << height << std::endl;
}

void PCCEncoder::packMissedPointsPatch(PCCFrameContext &frame,
                                       const std::vector<bool> &occupancyMap, size_t &width,
                                       size_t &height, size_t occupancySizeU, size_t occupancySizeV,
                                       size_t maxOccupancyRow) {
  auto& missedPointsPatch = frame.getMissedPointsPatch();
  missedPointsPatch.v0 = maxOccupancyRow;
  missedPointsPatch.u0 = 0;
  size_t missedPointsPatchBlocks =
      static_cast<size_t>(ceil(double(missedPointsPatch.size()) /
                               (params_.occupancyResolution_ * params_.occupancyResolution_)));
  size_t missedPointsPatchBlocksV =
      static_cast<size_t>(ceil(double(missedPointsPatchBlocks) / occupancySizeU));
  int occupancyRows2Add = static_cast<int>(maxOccupancyRow + missedPointsPatchBlocksV -
                                           height / params_.occupancyResolution_);
  occupancyRows2Add = occupancyRows2Add > 0 ? occupancyRows2Add : 0;
  occupancySizeV += occupancyRows2Add;
  height += occupancyRows2Add * params_.occupancyResolution_;

  vector<bool> newOccupancyMap;
  newOccupancyMap.resize(occupancySizeU * occupancySizeV);

  size_t missedPointsPatchBlocksU =
      static_cast<size_t>(ceil(double(missedPointsPatchBlocks) / missedPointsPatchBlocksV));
  missedPointsPatch.sizeV = missedPointsPatchBlocksV * params_.occupancyResolution_;
  missedPointsPatch.sizeU =
      static_cast<size_t>(ceil(double(missedPointsPatch.size()) / missedPointsPatch.sizeV));
  missedPointsPatch.sizeV0 = missedPointsPatchBlocksV;
  missedPointsPatch.sizeU0 = missedPointsPatchBlocksU;

  const int16_t infiniteValue = (std::numeric_limits<int16_t>::max)();
  missedPointsPatch.resize(missedPointsPatch.sizeU * missedPointsPatch.sizeV, infiniteValue);
  std::vector<bool> &missedPointPatchOccupancy = missedPointsPatch.occupancy;
  missedPointPatchOccupancy.resize(missedPointsPatch.sizeU0 * missedPointsPatch.sizeV0, false);

  for (size_t v = 0; v < missedPointsPatch.sizeV; ++v) {
    for (size_t u = 0; u < missedPointsPatch.sizeU; ++u) {
      const size_t p = v * missedPointsPatch.sizeU + u;
      if (missedPointsPatch.x[p] < infiniteValue) {
        const size_t u0 = u / missedPointsPatch.occupancyResolution;
        const size_t v0 = v / missedPointsPatch.occupancyResolution;
        const size_t p0 = v0 * missedPointsPatch.sizeU0 + u0;
        assert(u0 >= 0 && u0 < missedPointsPatch.sizeU0);
        assert(v0 >= 0 && v0 < missedPointsPatch.sizeV0);
        missedPointPatchOccupancy[p0] = true;
      }
    }
  }

  for (size_t v0 = 0; v0 < missedPointsPatch.sizeV0; ++v0) {
    const size_t v = missedPointsPatch.v0 + v0;
    for (size_t u0 = 0; u0 < missedPointsPatch.sizeU0; ++u0) {
      const size_t u = missedPointsPatch.u0 + u0;
      newOccupancyMap[v * occupancySizeU + u] =
          newOccupancyMap[v * occupancySizeU + u] || missedPointPatchOccupancy[v0 * missedPointsPatch.sizeU0 + u0];
    }
  }

  //printMap(newOccupancyMap, occupancySizeU, occupancySizeV);
  for (size_t v = 0; v < maxOccupancyRow; ++v) {
    for (size_t u = 0; u < occupancySizeU; ++u) {
      const size_t p = v * occupancySizeU + u;
      newOccupancyMap[p] = occupancyMap[p];
    }
  }

  printMap(newOccupancyMap, occupancySizeU, occupancySizeV);
}

bool PCCEncoder::generateGeometryVideo( const PCCPointSet3& source, PCCFrameContext& frame,
                                        const PCCPatchSegmenter3Parameters segmenterParams,
                                        PCCVideoGeometry &videoGeometry, PCCFrameContext &prevFrame ,
                                        size_t frameIndex, float& distanceSrcRec ) {
  if (!source.getPointCount()) {
    return false;
  }
  auto& patches = frame.getPatches();
  patches.reserve(256);
  PCCPatchSegmenter3 segmenter;
  segmenter.setNbThread( params_.nbThread_ );
  segmenter.compute( source, segmenterParams, patches,frame.getSrcPointCloudByPatch(), distanceSrcRec );
  auto &patchLevelMetadataEnabledFlags = frame.getFrameLevelMetadata().getLowerLevelMetadataEnabledFlags();
  if (patchLevelMetadataEnabledFlags.getMetadataEnabled()) {
    for (size_t i = 0; i < patches.size(); i++) {
      // Place to get/set patch-level metadata.
      PCCMetadata patchLevelMetadata;
      patches[i].getPatchLevelMetadata() = patchLevelMetadata;
      patches[i].getPatchLevelMetadata().getMetadataEnabledFlags() = patchLevelMetadataEnabledFlags;
    }
  }

  if (params_.losslessGeo_) {
    generateMissedPointsPatch(source, frame, segmenterParams.useEnhancedDeltaDepthCode); //useEnhancedDeltaDepthCode for EDD code
    sortMissedPointsPatch(frame);
  }

  if (params_.testLevelOfDetail_ > 0) {
    for (size_t i = 0; i < patches.size(); i++) {
      patches[i].getLod() = params_.testLevelOfDetail_;
    }
  }

  else if (params_.testLevelOfDetailSignaling_ > 0) { // generate semi-random levels of detail for testing
    srand(frame.getPatches().size());         // use a deterministic seed based on frame size
    for (size_t i = 0; i < patches.size(); i++) {
      patches[i].getLod() = rand() % params_.testLevelOfDetailSignaling_;
    }
  }

  if(params_.packingStrategy_ == 0) {
    if((frameIndex == 0) || (!params_.constrainedPack_)){
      pack( frame , params_.safeGuardDistance_ );    
    } else {
      spatialConsistencyPack(frame , prevFrame, params_.safeGuardDistance_ );
    }
  } else {
    if((frameIndex == 0) || (!params_.constrainedPack_)){
      packFlexible( frame , params_.safeGuardDistance_ );
    } else {
      spatialConsistencyPackFlexible(frame , prevFrame, params_.safeGuardDistance_ );
    }
  }

  return true;
}

void PCCEncoder::generateOccupancyMap( PCCFrameContext& frame ) {
  auto& occupancyMap     = frame.getOccupancyMap();
  auto& fullOccupancyMap = frame.getFullOccupancyMap();
  auto& width   = frame.getWidth();
  auto& height  = frame.getHeight();
  occupancyMap.resize(width * height, 0);
  if (!params_.absoluteD1_) {
    fullOccupancyMap.resize(width * height, 0);
  }
  const int16_t infiniteDepth = (std::numeric_limits<int16_t>::max)();
  for (auto &patch : frame.getPatches() ) {
    const size_t v0 = patch.getV0() * patch.getOccupancyResolution();
    const size_t u0 = patch.getU0() * patch.getOccupancyResolution();
    for (size_t v = 0; v < patch.getSizeV(); ++v) {
      for (size_t u = 0; u < patch.getSizeU(); ++u) {
        const size_t p = v * patch.getSizeU() + u;
        const int16_t d = patch.getDepth(0)[p];
        if (d < infiniteDepth) {
          size_t x;
          size_t y;
          occupancyMap[patch.patch2Canvas(u, v, width, height, x, y)] = 1;
        }
      }
    }
  }

  if(!frame.getUseMissedPointsSeparateVideo()) {
    auto& missedPointsPatch = frame.getMissedPointsPatch();
    const size_t v0 = missedPointsPatch.v0 * missedPointsPatch.occupancyResolution;
    const size_t u0 = missedPointsPatch.u0 * missedPointsPatch.occupancyResolution;
    if (missedPointsPatch.size()) {
      for (size_t v = 0; v < missedPointsPatch.sizeV; ++v) {
        for (size_t u = 0; u < missedPointsPatch.sizeU; ++u) {
          const size_t p = v * missedPointsPatch.sizeU + u;
          if (missedPointsPatch.x[p] < infiniteDepth) {
            const size_t x = (u0 + u);
            const size_t y = (v0 + v);
            assert(x < width && y < height);
            occupancyMap[x + y * width] = 1;
          }
        }
      }
    }
  }
  if (!params_.absoluteD1_) {
    fullOccupancyMap = occupancyMap;
  }
}

void PCCEncoder::modifyOccupancyMap(PCCFrameContext& frame, const PCCImageGeometry &imageRef, const PCCImageGeometry &image)
{
	auto& occupancyMap = frame.getOccupancyMap();
	auto& fullOccupancyMap = frame.getFullOccupancyMap();
	auto& width = frame.getWidth();
	auto& height = frame.getHeight();
	occupancyMap.resize(width * height, 0);
	if (!params_.absoluteD1_)
		fullOccupancyMap.resize(width * height, 0);
	const int16_t infiniteDepth = (std::numeric_limits<int16_t>::max)();
	for (auto &patch : frame.getPatches()) {
		const size_t v0 = patch.getV0() * patch.getOccupancyResolution();
		const size_t u0 = patch.getU0() * patch.getOccupancyResolution();
		for (size_t v = 0; v < patch.getSizeV(); ++v) {
			for (size_t u = 0; u < patch.getSizeU(); ++u) {
				const size_t p = v * patch.getSizeU() + u;
				const int16_t d = patch.getDepth(0)[p];
				
				const int16_t eddCode = patch.getDepthEnhancedDeltaD()[p];
								
				size_t x, y;
				auto indx = patch.patch2Canvas(u,v,width,height,x,y);
				assert(x < width && y < height);
				const size_t d0 = imageRef.getValue(0, x, y);
				const size_t d1 = image.getValue(0, x, y);
				if ((d < infiniteDepth) && (occupancyMap[indx] == 1) && ((d1 - d0) > 1)) {
					uint16_t bits = d1 - d0 - 1;
					uint16_t eddExtract = eddCode & (~((~0) << bits));
					uint16_t symbol = (((1 << bits) - 1) - eddExtract);
					occupancyMap[indx] += symbol;
				}
			}
		}
	}
	
	if (!params_.absoluteD1_)
		fullOccupancyMap = occupancyMap;

}

void PCCEncoder::generateIntraImage( PCCFrameContext& frame, const size_t depthIndex, PCCImageGeometry &image) {
  auto& width  = frame.getWidth();
  auto& height = frame.getHeight();
  image.resize( width, height );
  image.set(0);
  const int16_t infiniteDepth = (std::numeric_limits<int16_t>::max)();
  size_t maxDepth = 0;
  for (auto &patch : frame.getPatches() ) {
    const size_t v0 = patch.getV0() * patch.getOccupancyResolution();
    const size_t u0 = patch.getU0() * patch.getOccupancyResolution();
    for (size_t v = 0; v < patch.getSizeV(); ++v) {
      for (size_t u = 0; u < patch.getSizeU(); ++u) {
        const size_t p = v * patch.getSizeU() + u;
        const int16_t d = patch.getDepth(depthIndex)[p];
        if (d < infiniteDepth) {
          size_t x, y;
          patch.patch2Canvas(u, v, width, height, x, y);
          image.setValue(0, x, y, uint16_t(d));
          maxDepth = (std::max)(maxDepth, patch.getSizeD());
        }
      }
    }
  }
  std::cout << "maxDepth " << maxDepth << std::endl;
  if (maxDepth > 255) {
    std::cout << "Error: maxDepth > 255" << maxDepth << std::endl;
    exit(-1);
  }

  if(!frame.getUseMissedPointsSeparateVideo()) {
    auto& missedPointsPatch = frame.getMissedPointsPatch();
    const size_t v0 = missedPointsPatch.v0 * missedPointsPatch.occupancyResolution;
    const size_t u0 = missedPointsPatch.u0 * missedPointsPatch.occupancyResolution;
    if (missedPointsPatch.size()) {
      for (size_t v = 0; v < missedPointsPatch.sizeV; ++v) {
        for (size_t u = 0; u < missedPointsPatch.sizeU; ++u) {
          const size_t p = v * missedPointsPatch.sizeU + u;
          if (missedPointsPatch.x[p] < infiniteDepth) {
            const size_t x = (u0 + u);
            const size_t y = (v0 + v);
            assert(x < width && y < height);
            image.setValue(0, x, y, uint16_t(missedPointsPatch.x[p]));
            if (params_.losslessGeo444_) {
              image.setValue(1, x, y, uint16_t(missedPointsPatch.y[p]));
              image.setValue(2, x, y, uint16_t(missedPointsPatch.z[p]));
            }
          }
        }
      }
    }
  }
}

bool PCCEncoder::predictGeometryFrame( PCCFrameContext& frame, const PCCImageGeometry &reference, PCCImageGeometry &image) {
  assert(reference.getWidth() ==  image.getWidth());
  assert(reference.getHeight() == image.getHeight());
  const size_t refWidth  = reference.getWidth();
  const size_t refHeight = reference.getHeight();
  auto& occupancyMap = frame.getFullOccupancyMap();
  size_t occupancyResolution = params_.occupancyResolution_;

  for (size_t y = 0; y < refHeight; ++y) {
    for (size_t x = 0; x < refWidth; ++x) {
      const size_t pos1 = y * refWidth + x;
      if (occupancyMap[pos1] == 1) {
        for (size_t c = 0; c < 1; ++c) {
          //for (size_t c = 0; c < 3; ++c) {
          const uint16_t value1 = static_cast<uint16_t>(image.getValue(c, x, y));
          const uint16_t value0 = static_cast<uint16_t>(reference.getValue(c, x, y));
          size_t p = ((y / occupancyResolution) * (refWidth / occupancyResolution)) + (x / occupancyResolution);
          size_t patchIndex = frame.getBlockToPatch()[p]-1;
          auto patch = frame.getPatches()[patchIndex];

          int_least32_t delta = 0;  
          if (patch.getProjectionMode() == 0) {
            delta = (int_least32_t)value1 - (int_least32_t)value0;
            if (delta < 0) {
              delta = 0;
            }
            if (!params_.losslessGeo_ && delta > 9) {
              delta = 9;
            }
            image.setValue(c, x, y, (uint8_t)delta);
          } else {
            delta = (int_least32_t)value0 - (int_least32_t)value1;
            if (delta < 0) {
              delta = 0;
            }
            if (!params_.losslessGeo_ && delta > 9) {//no clipping for lossless coding
              delta = 9;
            }
            image.setValue(c, x, y, (uint8_t)delta);
          }
        }
      }
    }
  }
  return true;
}

void PCCEncoder::generateMissedPointsPatch(const PCCPointSet3& source, PCCFrameContext& frame, bool useEnhancedDeltaDepthCode) { //useEnhancedDeltaDepthCode for EDD
  auto& patches = frame.getPatches();
  auto& missedPointsPatch = frame.getMissedPointsPatch();
  missedPointsPatch.sizeU = 0;
  missedPointsPatch.sizeU = 0;
  missedPointsPatch.sizeV = 0;
  missedPointsPatch.u0 = 0;
  missedPointsPatch.v0 = 0;
  missedPointsPatch.sizeV0 = 0;
  missedPointsPatch.sizeU0 = 0;
  missedPointsPatch.occupancy.resize(0);
  PCCPointSet3 pointsToBeProjected;
  const int16_t infiniteDepth = (std::numeric_limits<int16_t>::max)();

  bool useOneLayerMode = params_.oneLayerMode_ || params_.singleLayerPixelInterleaving_;
  bool flag_6direction = params_.absoluteD1_ && params_.sixDirectionMode_;

  for (const auto &patch : patches) {
    for (size_t v = 0; v < patch.getSizeV(); ++v) {
      for (size_t u = 0; u < patch.getSizeU(); ++u) {
        const size_t p = v * patch.getSizeU() + u;
        const size_t depth0 = patch.getDepth(0)[p];
        if (depth0 < infiniteDepth) {
          PCCPoint3D point0;
          if (!flag_6direction) {
            point0[patch.getNormalAxis()] = double(depth0) + patch.getD1();
          } else {
            if (patch.getProjectionMode() == 0) {
              point0[patch.getNormalAxis()] = double(depth0 + patch.getD1());
            } else {
              point0[patch.getNormalAxis()] = double(patch.getD1() - depth0);
            }
          }
          point0[patch.getTangentAxis()] = double(u) + patch.getU1();
          point0[patch.getBitangentAxis()] = double(v) + patch.getV1();
          pointsToBeProjected.addPoint(point0);
          if (useEnhancedDeltaDepthCode) {
            if (patch.getDepthEnhancedDeltaD()[p] != 0) {
              PCCPoint3D point1;
              point1[patch.getTangentAxis()] = double(u) + patch.getU1();
              point1[patch.getBitangentAxis()] = double(v) + patch.getV1();
              for (uint16_t i = 0; i < 16; i++) { //surfaceThickness is not necessary here?
                if (patch.getDepthEnhancedDeltaD()[p] & (1 << i)) {
                  uint16_t nDeltaDCur = (i + 1);
                  if (!flag_6direction) {
                    if (patch.getProjectionMode() == 0) {
                      point1[patch.getNormalAxis()] = double(depth0 + patch.getD1() + nDeltaDCur);
                    } else {
                      point1[patch.getNormalAxis()] = double(depth0 + patch.getD1() - nDeltaDCur);
                    }
                  } else {
                    if (patch.getProjectionMode() == 0) {
                      point1[patch.getNormalAxis()] = double(depth0 + patch.getD1() + nDeltaDCur);
                    } else {
                      point1[patch.getNormalAxis()] = double(patch.getD1() - depth0 - nDeltaDCur);
                    }
                  }
                  pointsToBeProjected.addPoint(point1);
                }
              }//for each i
            }//if( patch.getDepthEnhancedDeltaD()[p] != 0) )
          } else {
            const size_t depth1 = patch.getDepth(1)[p];
            if (!flag_6direction) {
              if ( ( ( depth1 > depth0 ) && ( patch.getProjectionMode() == 0) ) ||
                  ( ( depth1 < depth0 ) && ( patch.getProjectionMode() == 1) ) ) {
                PCCPoint3D point1;
                point1[patch.getNormalAxis()] = double(depth1) + patch.getD1();
                point1[patch.getTangentAxis()] = double(u) + patch.getU1();
                point1[patch.getBitangentAxis()] = double(v) + patch.getV1();
                pointsToBeProjected.addPoint(point1);
              }
            } else {
              PCCPoint3D point1;
              point1[patch.getTangentAxis()] = double(u) + patch.getU1();
              point1[patch.getBitangentAxis()] = double(v) + patch.getV1();
              if (patch.getProjectionMode() == 0) {
                point1[patch.getNormalAxis()] = double(depth1) + patch.getD1();
              } else {
                point1[patch.getNormalAxis()] = double(patch.getD1()) - double(depth1);
              }
              pointsToBeProjected.addPoint(point1);
            }
          } //~if (useEnhancedDeltaDepthCode)
        }
      }
    }
  }

  PCCKdTree kdtreeMissedPoints(pointsToBeProjected);
  PCCNNResult result;
  std::vector<size_t> missedPoints;
  missedPoints.resize(0);
  for (size_t i = 0; i < source.getPointCount(); ++i) {
    kdtreeMissedPoints.search( source[i], 1, result);
    const double dist2 = result.dist(0);
    if (dist2 > 0.0) {
      missedPoints.push_back(i);
    }
  }

  missedPointsPatch.occupancyResolution = params_.occupancyResolution_;
  if (params_.losslessGeo444_){
    missedPointsPatch.resize(missedPoints.size());
    for (auto i = 0; i < missedPoints.size(); ++i) {
      const PCCPoint3D missedPoint = source[missedPoints[i]];
      missedPointsPatch.x[i] = static_cast<uint16_t>(missedPoint.x());
      missedPointsPatch.y[i] = static_cast<uint16_t>(missedPoint.y());
      missedPointsPatch.z[i] = static_cast<uint16_t>(missedPoint.z());
    }
    if(frame.getUseMissedPointsSeparateVideo()) {
      missedPointsPatch.setMPnumber(missedPoints.size());
    }
  } else {
    const size_t mps = missedPoints.size();
    missedPointsPatch.resize(3 * mps);
    if(frame.getUseMissedPointsSeparateVideo()) {
      missedPointsPatch.setMPnumber(missedPoints.size());
    }
    const int16_t infiniteValue = (std::numeric_limits<int16_t>::max)();
    for (auto i = 0; i < mps; ++i) {
      const PCCPoint3D missedPoint = source[missedPoints[i]];
      missedPointsPatch.x[i] = static_cast<uint16_t>(missedPoint.x());
      missedPointsPatch.x[mps + i] = static_cast<uint16_t>(missedPoint.y());
      missedPointsPatch.x[2 * mps + i] = static_cast<uint16_t>(missedPoint.z());
      missedPointsPatch.y[i] = infiniteValue;
      missedPointsPatch.y[mps + i] = infiniteValue;
      missedPointsPatch.y[2 * mps + i] = infiniteValue;
      missedPointsPatch.z[i] = infiniteValue;
      missedPointsPatch.z[mps + i] = infiniteValue;
      missedPointsPatch.z[2 * mps + i] = infiniteValue;
    }
  }
}

void PCCEncoder::sortMissedPointsPatch(PCCFrameContext& frame) {
  auto& missedPointsPatch = frame.getMissedPointsPatch();
  const size_t maxNeighborCount = 5;
  const size_t neighborSearchRadius = 5 * 5;
  size_t missedPointCount =
      params_.losslessGeo444_ ? missedPointsPatch.size() : missedPointsPatch.size() / 3;
  if (missedPointCount) {
    vector<size_t> sortIdx;
    sortIdx.reserve(missedPointCount);
    PCCPointSet3 missedPointSet;
    missedPointSet.resize(missedPointCount);
    for (size_t i = 0; i < missedPointCount; i++) {
      missedPointSet[i] = params_.losslessGeo444_ ?
          PCCPoint3D(missedPointsPatch.x[i], missedPointsPatch.y[i], missedPointsPatch.z[i]) :
          PCCPoint3D(missedPointsPatch.x[i], missedPointsPatch.x[i + missedPointCount],
                     missedPointsPatch.x[i + missedPointCount * 2]);
    }
    PCCKdTree kdtreeMissedPointSet(missedPointSet);
    PCCNNResult result;
    std::vector<size_t> fifo;
    fifo.reserve(missedPointCount);
    std::vector<bool> flags(missedPointCount, true);

    for (size_t i = 0; i < missedPointCount; i++) {
      if (flags[i]) {
        flags[i] = false;
        sortIdx.push_back(i);
        fifo.push_back(i);
        while (!fifo.empty()) {
          const size_t currentIdx = fifo.back();
          fifo.pop_back();
          kdtreeMissedPointSet.searchRadius(missedPointSet[currentIdx], maxNeighborCount, neighborSearchRadius, result);
          for (size_t j = 0; j < result.count(); j++) {
            size_t n = result.indices(j);
            if (flags[n]) {
              flags[n] = false;
              sortIdx.push_back(n);
              fifo.push_back(n);
            }
          }
        }
      }
    }

    for (size_t i = 0; i < missedPointCount; ++i) {
      const PCCPoint3D missedPoint = missedPointSet[sortIdx[i]];
      if (params_.losslessGeo444_) {
        missedPointsPatch.x[i] = static_cast<uint16_t>(missedPoint.x());
        missedPointsPatch.y[i] = static_cast<uint16_t>(missedPoint.y());
        missedPointsPatch.z[i] = static_cast<uint16_t>(missedPoint.z());
      } else {
        missedPointsPatch.x[i] = static_cast<uint16_t>(missedPoint.x());
        missedPointsPatch.x[i + missedPointCount] = static_cast<uint16_t>(missedPoint.y());
        missedPointsPatch.x[i + missedPointCount * 2] = static_cast<uint16_t>(missedPoint.z());
      }
    }
  }
}

void PCCEncoder::generateMissedPointsGeometryVideo(PCCContext& context, PCCGroupOfFrames& reconstructs) {
  auto& videoMPsGeometry = context.getVideoMPsGeometry();
  auto gofSize = context.getGofSize();
  size_t maxWidth=0;
  size_t maxHeight=0;
  videoMPsGeometry.resize(gofSize);
  for (auto &frame : context.getFrames() ) {
    const size_t shift = frame.getIndex();
    frame.setLosslessGeo(context.getLosslessGeo());
    frame.setLosslessGeo444(context.getLosslessGeo444());
    frame.setMPGeoWidth(context.getMPGeoWidth());
    frame.setMPGeoHeight(0);
    frame.setEnhancedDeltaDepth(context.getEnhancedDeltaDepth());
    generateMPsGeometryImage    (context, frame,videoMPsGeometry.getFrame(shift));
    cout<<"generate Missed Points (Geometry) : frame "<<shift<<", # of Missed Points Geometry : "<<frame.getMissedPointsPatch().size()<<endl;
    //for resizing for mpgeometry
    auto& MPGeoFrame = videoMPsGeometry.getFrame(shift);
    maxWidth  = (std::max)( maxWidth,  MPGeoFrame.getWidth () );
    maxHeight = (std::max)( maxHeight, MPGeoFrame.getHeight() );
  }

  //resizing for mpgeometry
  assert(maxWidth==64);
  assert(maxHeight%8==0);
  context.setMPGeoWidth(maxWidth);
  context.setMPGeoHeight(maxHeight);
  for (auto &frame : context.getFrames() ) {
    const size_t shift = frame.getIndex();
    auto& MPGeoFrame = videoMPsGeometry.getFrame(shift);
    MPGeoFrame.resize(maxWidth, maxHeight);
  }
  cout<<"MissedPoints Geometry [done]"<<endl;
}

void PCCEncoder::writeMissedPointsGeometryNumber(PCCContext& context, PCCBitstream &bitstream) {
  size_t maxHeight = 0;
  size_t MPwidth=context.getMPGeoWidth ();
  bitstream.write<size_t>(size_t( context.getMPGeoWidth () ) );
  for (auto &frame : context.getFrames() ) {
    size_t numofMPs=frame.getMissedPointsPatch().size();
    if(!context.getLosslessGeo444()) {
      numofMPs/=3;
    }
    bitstream.write<size_t>(size_t( numofMPs ) );
    size_t height = (3*numofMPs)/MPwidth+1;
    size_t heightby8= height/8;
    if(heightby8*8!=height) {
      height = (heightby8+1)*8;
    }
    maxHeight = (std::max)( maxHeight, height );
  }
  assert(maxHeight==context.getMPGeoHeight ());
}

void PCCEncoder::writeMissedPointsTextureNumber(PCCContext& context, PCCBitstream &bitstream) {
  size_t maxHeight = 0;
  size_t MPwidth=context.getMPAttWidth ();
  bitstream.write<size_t>(size_t( context.getMPAttWidth () ) );
  for (auto &frame : context.getFrames() ) {
    size_t numofMPs=frame.getMissedPointsPatch().sizeofcolor();
    bitstream.write<size_t>(size_t( numofMPs ) );
    size_t height = (3*numofMPs)/MPwidth+1;
    size_t heightby8= height/8;
    if(heightby8*8!=height) {
      height = (heightby8+1)*8;
    }
    maxHeight = (std::max)( maxHeight, height );
  }
  assert(maxHeight==context.getMPAttHeight ());
}

void PCCEncoder::generateMissedPointsTextureVideo(PCCContext& context, PCCGroupOfFrames& reconstructs ) {
  auto& videoMPsTexture = context.getVideoMPsTexture();
  auto gofSize = context.getGofSize();
  videoMPsTexture.resize(gofSize);
  size_t maxWidth=0;
  size_t maxHeight=0;
  for (auto &frame : context.getFrames() ) {
    const size_t shift = frame.getIndex();
    frame.setLosslessAtt(context.getLosslessAtt());
    frame.setMPAttWidth(context.getMPAttWidth());
    frame.setMPAttHeight(0);
    generateMPsTextureImage(context, frame, videoMPsTexture.getFrame(shift), shift, reconstructs[shift]);
    cout<<"generate Missed Points (Texture) : frame "<<shift<<", # of Missed Points Texture : "<<frame.getMissedPointsPatch().size()<<endl;
    //for resizing for mpgeometry
    auto& MPTexFrame = videoMPsTexture.getFrame(shift);
    maxWidth  = (std::max)( maxWidth,  MPTexFrame.getWidth () );
    maxHeight = (std::max)( maxHeight, MPTexFrame.getHeight() );
  }
  //resizing for mpgeometry
  assert(maxWidth==64);
  assert(maxHeight%8==0);
  context.setMPAttWidth(maxWidth);
  context.setMPAttHeight(maxHeight);
  for (auto &frame : context.getFrames() ) {
    const size_t shift = frame.getIndex();
    auto& MPTexFrame = videoMPsTexture.getFrame(shift);
    MPTexFrame.resize(maxWidth, maxHeight);
  }

  cout<<"MissedPoints Texture [done]"<<endl;
}

void PCCEncoder::generateMPsGeometryImage    (PCCContext& context, PCCFrameContext& frame, PCCImageGeometry &image) {
  bool losslessGeo444 = frame.getLosslessGeo444();
  auto& missedPointsPatch = frame.getMissedPointsPatch();
  if(!losslessGeo444) {
    assert(missedPointsPatch.size()%3==0);
  }

  const size_t numofMPs=losslessGeo444?missedPointsPatch.size():(missedPointsPatch.size()/3);
  missedPointsPatch.setMPnumber(numofMPs);
  size_t width  = frame.getMPGeoWidth();
  size_t height = (3*numofMPs)/width+1;
  //height adjust for HM
  size_t heightby8= height/8;
  if(heightby8*8!=height) {
    height = (heightby8+1)*8;
  }
  frame.setMPGeoHeight(height);
  image.resize( width, height );
  image.set(0);
  for(size_t i=0; i<numofMPs; i++) {
    if (frame.getLosslessGeo444()) {
      image.setValue(0, i%width, i/width, missedPointsPatch.x[i]);
      image.setValue(0, i%width, i/width, missedPointsPatch.y[i]);
      image.setValue(0, i%width, i/width, missedPointsPatch.z[i]);
    } else{
      image.setValue(0, i%width,                i/width,                missedPointsPatch.x[i]);
      image.setValue(0, (numofMPs + i)%width,   (numofMPs + i)/width,   missedPointsPatch.x[numofMPs + i]);
      image.setValue(0, (2*numofMPs + i)%width, (2*numofMPs + i)/width, missedPointsPatch.x[2 * numofMPs + i]);
    }
  }
}

void PCCEncoder::generateMPsTextureImage(PCCContext& context, PCCFrameContext& frame, PCCImageTexture &image, size_t shift, const PCCPointSet3& reconstruct) {
  auto& missedPointsPatch = frame.getMissedPointsPatch();
  const size_t sizeofMPcolor=missedPointsPatch.sizeofcolor();
  size_t width  = frame.getMPAttWidth();
  size_t height = sizeofMPcolor/width+1;
  size_t heightby8= height/8;
  if(heightby8*8!=height) {
    height = (heightby8+1)*8;
  }
  image.resize( width, height );
  image.set(0);

  for(size_t k=0;k<sizeofMPcolor;k++) {
    size_t xx = k%width;
    size_t yy = k/width;
    assert(yy<height);
    image.setValue(0, xx, yy, missedPointsPatch.r[k]);
    image.setValue(1, xx, yy, missedPointsPatch.g[k]);
    image.setValue(2, xx, yy, missedPointsPatch.b[k]);
  }
}

bool PCCEncoder::generateGeometryVideo( const PCCGroupOfFrames& sources, PCCContext &context) {
  bool res = true;
  PCCPatchSegmenter3Parameters segmenterParams;
  segmenterParams.nnNormalEstimation                    = params_.nnNormalEstimation_;
  segmenterParams.maxNNCountRefineSegmentation          = params_.maxNNCountRefineSegmentation_;
  segmenterParams.iterationCountRefineSegmentation      = params_.iterationCountRefineSegmentation_;
  segmenterParams.occupancyResolution                   = params_.occupancyResolution_;
  segmenterParams.minPointCountPerCCPatchSegmentation   = params_.minPointCountPerCCPatchSegmentation_;
  segmenterParams.maxNNCountPatchSegmentation           = params_.maxNNCountPatchSegmentation_;
  segmenterParams.surfaceThickness                      = params_.surfaceThickness_;
  segmenterParams.maxAllowedDepth                       = params_.maxAllowedDepth_;
  segmenterParams.maxAllowedDist2MissedPointsDetection  = params_.maxAllowedDist2MissedPointsDetection_;
  segmenterParams.maxAllowedDist2MissedPointsSelection  = params_.maxAllowedDist2MissedPointsSelection_;
  segmenterParams.lambdaRefineSegmentation              = params_.lambdaRefineSegmentation_;
  segmenterParams.projectionMode                        = params_.projectionMode_;
  segmenterParams.useEnhancedDeltaDepthCode             = params_.losslessGeo_ ? params_.enhancedDeltaDepthCode_ : false;
  segmenterParams.sixDirectionMode                      = params_.sixDirectionMode_;
  segmenterParams.absoluteD1                            = params_.absoluteD1_;
  segmenterParams.useOneLayermode                       = params_.oneLayerMode_ || params_.singleLayerPixelInterleaving_;
  segmenterParams.surfaceSeparation                     = params_.surfaceSeparation_;

  auto& videoGeometry = context.getVideoGeometry();
  auto & frames = context.getFrames();

  float sumDistanceSrcRec = 0;
  for( size_t i =0; i<frames.size();i++){
    size_t preIndex = i > 0 ? (i - 1):0;
    float distanceSrcRec = 0;
    if (!generateGeometryVideo( sources[ i ], frames[ i ], segmenterParams, videoGeometry , frames[preIndex] , i, distanceSrcRec )) {
      res = false;
      break;
    }
    sumDistanceSrcRec += distanceSrcRec;
  }
  if ( params_.oneLayerMode_ ) {
    const float distanceSrcRec = sumDistanceSrcRec / (float)frames.size();
    if( distanceSrcRec >= 250.f ) {
      params_.oneLayerMode_ = false;
      if( params_.singleLayerPixelInterleaving_ ) { params_.singleLayerPixelInterleaving_ = false; }
    }
  }
  return res;
}

template<typename... Args>
std::string string_format(const char* pFormat, Args... eArgs ) {
  size_t iSize = snprintf( NULL, 0, pFormat, eArgs... );
  std::string eBuffer;
  eBuffer.reserve( iSize + 1 );
  eBuffer.resize( iSize );
  snprintf( &eBuffer[0], iSize + 1, pFormat, eArgs... );
  return eBuffer;
}

void PCCEncoder::reconsctuctionOptimization( PCCContext& context,
                                             const GeneratePointCloudParameters params ) {
  auto& frames = context.getFrames();
  auto &videoGeometry   = context.getVideoGeometry();
  auto &videoGeometryD1 = context.getVideoGeometryD1();
  for( size_t i = 0; i < frames.size(); i++ ) {
    reconsctuctionOptimization( frames[i], videoGeometry, videoGeometryD1, params );
  }
}

void PCCEncoder::reconsctuctionOptimization( PCCFrameContext &frame,
                                             const PCCVideoGeometry &video,
                                             const PCCVideoGeometry &videoD1,
                                             const GeneratePointCloudParameters params ) {
  auto& patches         = frame.getPatches();
  auto& blockToPatch    = frame.getBlockToPatch();
  auto& occupancyMapOrg = frame.getOccupancyMap();
  std::vector<uint32_t> occupancyMap;
  occupancyMap.resize( occupancyMapOrg.size(), 0 );
  for (size_t i = 0; i < occupancyMapOrg.size(); i++) {
    occupancyMap[ i ] = occupancyMapOrg[ i ];
  }
  const size_t width              = frame.getWidth();
  const size_t height             = frame.getHeight();
  const size_t blockToPatchWidth  = width  / params_.occupancyResolution_;
  const size_t blockToPatchHeight = height / params_.occupancyResolution_;
  const size_t blockSize0         = params_.occupancyResolution_ / params_.occupancyPrecision_;
  for (size_t v0 = 0; v0 < blockToPatchHeight; ++v0) {
    for (size_t u0 = 0; u0 < blockToPatchWidth; ++u0) {
      for (size_t v1 = 0; v1 < blockSize0; ++v1) {
        const size_t v2 = v0 * params_.occupancyResolution_ + v1 * params_.occupancyPrecision_;
        for (size_t u1 = 0; u1 < blockSize0; ++u1) {
          const size_t u2 = u0 * params_.occupancyResolution_ + u1 * params_.occupancyPrecision_;
          bool isFull = false;
          for (size_t v3 = 0; v3 < params_.occupancyPrecision_ && !isFull; ++v3) {
            for (size_t u3 = 0; u3 < params_.occupancyPrecision_ && !isFull; ++u3) {
              isFull |= occupancyMap[(v2 + v3) * width + u2 + u3] == 1;
            }
          }
          for (size_t v3 = 0; v3 < params_.occupancyPrecision_; ++v3) {
            for (size_t u3 = 0; u3 < params_.occupancyPrecision_; ++u3) {
              occupancyMap[(v2 + v3) * width + u2 + u3] = isFull;
            }
          }
        }
      }
    }
  }
  size_t shift;
  const size_t layerCount = 2;
  if (!params.absoluteD1_) {
    shift = frame.getIndex();
    if (video.getFrameCount() < (shift + 1)) {
      return;
    }
  } else {
    shift = frame.getIndex() * ( params.oneLayerMode_ ?  1 : 2 );
    if (video.getFrameCount() < (shift + ( params.oneLayerMode_ ?  1 : 2 ))) {
      return;
    }
  }
  const size_t patchCount = patches.size();
  const size_t blockCount = blockToPatchWidth * blockToPatchHeight;
  blockToPatch.resize( 0 );
  blockToPatch.resize( blockCount, 0 );
  for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
    auto &patch = patches[patchIndex];
    const auto& occupancy = patch.getOccupancy();
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        if ( occupancy[v0 * patch.getSizeU0() + u0]) {
          blockToPatch[patch.patchBlock2CanvasBlock(u0, v0, blockToPatchWidth, blockToPatchHeight)] = patchIndex + 1;
        }
      }
    }
  }
  auto & interpolateMap = frame.getInterpolate();
  auto & fillingMap     = frame.getFilling();
  auto & minD1Map       = frame.getMinD1();
  auto & neighborMap    = frame.getNeighbor();
  interpolateMap.resize( blockCount, 0 );
  fillingMap    .resize( blockCount, 0 );
  minD1Map      .resize( blockCount, 0 );
  neighborMap   .resize( blockCount, 0 );

  const auto &frame0 = video.getFrame(shift);
  const size_t imageWidth  = video.getWidth();
  const size_t imageHeight = video.getHeight();
  std::vector<PCCPointSet3>& srcPointCloudByBlock = frame.getSrcPointCloudByBlock();
  for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
    const size_t patchIndexPlusOne = patchIndex + 1;
    auto &patch = patches[patchIndex];
    const double lodScale = params.ignoreLod_ ? 1.0 : double(1u << patch.getLod());
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        const size_t blockIndex = patch.patchBlock2CanvasBlock(u0, v0, blockToPatchWidth, blockToPatchHeight);
        if (blockToPatch[blockIndex] == patchIndexPlusOne) {
          auto& srcPointCloudPatch = frame.getSrcPointCloudByPatch( patch.getIndex() );
          PCCPointSet3 blockSrcPointCloud;
          const size_t xMin = u0 * patch.getOccupancyResolution() + patch.getU1();
          const size_t yMin = v0 * patch.getOccupancyResolution() + patch.getV1();
          for(size_t i=0;i<srcPointCloudPatch.getPointCount();i++) {
            if( xMin <= srcPointCloudPatch[i][patch.getTangentAxis()  ]  && srcPointCloudPatch[i][patch.getTangentAxis  ()] < xMin + patch.getOccupancyResolution() &&
                yMin <= srcPointCloudPatch[i][patch.getBitangentAxis()]  && srcPointCloudPatch[i][patch.getBitangentAxis()] < yMin + patch.getOccupancyResolution() ) {
              blockSrcPointCloud.addPoint( srcPointCloudPatch[i] );
            }
          }
          std::vector<PCCPointSet3> reconstruct;
          std::vector<float> distance;
          reconstruct.resize( 10 );
          distance   .resize( 10 );
          size_t optimizationIndex = 0, optimizationIndexMin = 0;
          for(size_t minD1 = 0; minD1<2;minD1++){
            for(size_t interpolate = 0; interpolate<2;interpolate++){
              const size_t maxNeighborLocal = interpolate == 0 ? 2 : 3;
              for(size_t neighbor = 1; neighbor<maxNeighborLocal;neighbor++){
                const size_t maxFillingLocal = params.oneLayerMode_ && ( ( interpolate == 0 && minD1 == 0 ) ||
                    ( minD1 == 1 && interpolate == 0 ) ) ? 1 : 2;
                for(size_t filling = 0; filling<maxFillingLocal;filling++){
                  for (size_t v1 = 0; v1 < patch.getOccupancyResolution(); ++v1) {
                    const size_t v = v0 * patch.getOccupancyResolution() + v1;
                    for (size_t u1 = 0; u1 < patch.getOccupancyResolution(); ++u1) {
                      const size_t u = u0 * patch.getOccupancyResolution() + u1;
                      size_t x, y;
                      const bool occupancy = occupancyMap[patch.patch2Canvas(u, v, imageWidth, imageHeight, x, y)] != 0;
                      if (!occupancy) {
                        continue;
                      }
                      auto createdPoints = generatePoints( params, frame, video, videoD1, shift,
                                                           patchIndex,
                                                           u, v,
                                                           x, y,
                                                           interpolate, filling, minD1, neighbor, lodScale );
                      if (createdPoints.size() > 0 ) {
                        for(size_t i = 0; i<createdPoints.size();i++) {
                          reconstruct[optimizationIndex].addPoint(createdPoints[i]);
                        }
                      }
                    }
                  }
                  float distancePSrcRec,  distancePRecSrc;
                  blockSrcPointCloud.distanceGeo( reconstruct[optimizationIndex], distancePSrcRec, distancePRecSrc );
                  distance[optimizationIndex] = (std::max)( distancePSrcRec, distancePRecSrc );
                  if( optimizationIndex == 0 || distance[optimizationIndexMin] > distance[optimizationIndex] ) {
                    optimizationIndexMin       = optimizationIndex;
                    interpolateMap[blockIndex] = (bool)interpolate;
                    fillingMap    [blockIndex] = (bool)filling;
                    minD1Map      [blockIndex] = minD1;
                    neighborMap   [blockIndex] = neighbor;
                  }
                  optimizationIndex++;
                }
              }
            }
          }
        } // if block is used
      }
    }
  } // patch
}

bool PCCEncoder::resizeGeometryVideo( PCCContext &context ) {
  size_t maxWidth = 0, maxHeight = 0;
  for (auto &frame : context.getFrames() ) {
    maxWidth  = (std::max)( maxWidth,  frame.getWidth () );
    maxHeight = (std::max)( maxHeight, frame.getHeight() );
  }
  for (auto &frame : context.getFrames() ) {
    frame.getWidth () = maxWidth;
    frame.getHeight() = maxHeight;
    frame.getOccupancyMap().resize( ( maxWidth  / params_.occupancyResolution_ ) * ( maxHeight / params_.occupancyResolution_ ) );
  }
  return true;
}

bool PCCEncoder::dilateGeometryVideo( PCCContext &context) {
  auto& videoGeometry = context.getVideoGeometry();
  auto& videoGeometryD1 = context.getVideoGeometryD1();
  for (auto &frame : context.getFrames() ) {
    generateOccupancyMap( frame );
    const size_t shift = videoGeometry.getFrameCount();
    if (!params_.absoluteD1_) {
      videoGeometry.resize(shift + 1);
      videoGeometryD1.resize(shift + 1);
      auto &frame1 = videoGeometry.getFrame(shift);
      generateIntraImage(frame, 0, frame1);
      auto &frame2 = videoGeometryD1.getFrame(shift);
      if (params_.enhancedDeltaDepthCode_) {
        generateIntraEnhancedDeltaDepthImage(frame, frame1, frame2);
      } else {
        generateIntraImage(frame, 1, frame2);
      }
      dilate(frame, videoGeometry.getFrame(shift));
    } else {
      const size_t nbFrames = params_.oneLayerMode_ ? 1 : 2;
      videoGeometry.resize( shift + nbFrames );
      if (params_.enhancedDeltaDepthCode_) {
        auto &frame1 = videoGeometry.getFrame(shift);
        generateIntraImage(frame, 0, frame1);
        auto &frame2 = videoGeometry.getFrame(shift + 1);
		if (params_.improveEDD_)
			generateIntraImage(frame, 1, frame2);
		else
        generateIntraEnhancedDeltaDepthImage(frame, frame1, frame2);
        dilate(frame, videoGeometry.getFrame(shift));
        dilate(frame, videoGeometry.getFrame(shift + 1));
		if (params_.improveEDD_)
			modifyOccupancyMap(frame, frame1, frame2);
      } else {
        if (params_.oneLayerMode_ && params_.singleLayerPixelInterleaving_) {
          auto &frame1 = videoGeometry.getFrame(shift);
          generateIntraImage(frame, 0, frame1);
          dilate(frame, frame1);
          PCCImageGeometry frame2;
          generateIntraImage(frame, 1, frame2);
          dilate(frame, frame2);
          for (size_t x = 0; x < frame1.getWidth(); x++) {
            for (size_t y = 0; y < frame1.getHeight(); y++) {
              if ((x + y) % 2 == 1) {
                frame1.setValue(0, x, y, frame2.getValue(0, x, y));
              }
            }
          }
        } else {
          for (size_t f = 0; f < nbFrames; ++f) {
            auto &frame1 = videoGeometry.getFrame(shift + f);
            generateIntraImage(frame, f, frame1);
            dilate(frame, videoGeometry.getFrame(shift + f));
          }
        }
      }
    }
  }
  return true;
}

template <typename T>
void PCCEncoder::dilate( PCCFrameContext& frame, PCCImage<T, 3> &image, const PCCImage<T, 3> *reference ) {
  auto occupancyMapTemp = frame.getOccupancyMap();
  const size_t pixelBlockCount = params_.occupancyResolution_ * params_.occupancyResolution_;
  const size_t occupancyMapSizeU = image.getWidth () / params_.occupancyResolution_;
  const size_t occupancyMapSizeV = image.getHeight() / params_.occupancyResolution_;
  const int64_t neighbors[4][2] = {{0, -1}, {-1, 0}, {1, 0}, {0, 1}};
  const size_t MAX_OCCUPANCY_RESOLUTION = 64;
  assert(params_.occupancyResolution_ <= MAX_OCCUPANCY_RESOLUTION);
  size_t count[MAX_OCCUPANCY_RESOLUTION][MAX_OCCUPANCY_RESOLUTION];
  PCCVector3<int32_t> values[MAX_OCCUPANCY_RESOLUTION][MAX_OCCUPANCY_RESOLUTION];

  for (size_t v1 = 0; v1 < occupancyMapSizeV; ++v1) {
    const int64_t v0 = v1 * params_.occupancyResolution_;
    for (size_t u1 = 0; u1 < occupancyMapSizeU; ++u1) {
      const int64_t u0 = u1 * params_.occupancyResolution_;
      size_t nonZeroPixelCount = 0;
      for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
        for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
          const int64_t x0 = u0 + u2;
          const int64_t y0 = v0 + v2;
          assert(x0 < int64_t(image.getWidth()) && y0 < int64_t(image.getHeight()));
          const size_t location0 = y0 * image.getWidth() + x0;
		  if (params_.improveEDD_)
			  nonZeroPixelCount += (occupancyMapTemp[location0] > 0);
		  else
          nonZeroPixelCount += (occupancyMapTemp[location0] == 1);
        }
      }
      if (!nonZeroPixelCount) {
        if (reference) {
          for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
            for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
              const size_t x0 = u0 + u2;
              const size_t y0 = v0 + v2;
              image.setValue(0, x0, y0, reference->getValue(0, x0, y0));
              image.setValue(1, x0, y0, reference->getValue(1, x0, y0));
              image.setValue(2, x0, y0, reference->getValue(2, x0, y0));
            }
          }
        } else if (u1 > 0) {
          for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
            for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
              const size_t x0 = u0 + u2;
              const size_t y0 = v0 + v2;
              assert(x0 > 0);
              const size_t x1 = x0 - 1;
              image.setValue(0, x0, y0, image.getValue(0, x1, y0));
              image.setValue(1, x0, y0, image.getValue(1, x1, y0));
              image.setValue(2, x0, y0, image.getValue(2, x1, y0));
            }
          }
        } else if (v1 > 0) {
          for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
            for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
              const size_t x0 = u0 + u2;
              const size_t y0 = v0 + v2;
              assert(y0 > 0);
              const size_t y1 = y0 - 1;
              image.setValue(0, x0, y0, image.getValue(0, x0, y1));
              image.setValue(1, x0, y0, image.getValue(1, x0, y1));
              image.setValue(2, x0, y0, image.getValue(2, x0, y1));
            }
          }
        }
        continue;
      }
      for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
        for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
          values[v2][u2] = 0;
          count[v2][u2] = 0UL;
        }
      }
      uint32_t iteration = 1;
      while (nonZeroPixelCount < pixelBlockCount) {
        for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
          for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
            const int64_t x0 = u0 + u2;
            const int64_t y0 = v0 + v2;
            assert(x0 < int64_t(image.getWidth()) && y0 < int64_t(image.getHeight()));
            const size_t location0 = y0 * image.getWidth() + x0;
            if (occupancyMapTemp[location0] == iteration) {
              for (size_t n = 0; n < 4; ++n) {
                const int64_t x1 = x0 + neighbors[n][0];
                const int64_t y1 = y0 + neighbors[n][1];
                const size_t location1 = y1 * image.getWidth() + x1;
                if (x1 >= u0 && x1 < int64_t(u0 + params_.occupancyResolution_) && y1 >= v0 &&
                    y1 < int64_t( v0 + params_.occupancyResolution_ ) && occupancyMapTemp[location1] == 0) {
                  const int64_t u3 = u2 + neighbors[n][0];
                  const int64_t v3 = v2 + neighbors[n][1];
                  assert(u3 >= 0 && u3 < int64_t( params_.occupancyResolution_ ));
                  assert(v3 >= 0 && v3 < int64_t( params_.occupancyResolution_ ));
                  for (size_t k = 0; k < 3; ++k) {
                    values[v3][u3][k] += image.getValue(k, x0, y0);
                  }
                  ++count[v3][u3];
                }
              }
            }
          }
        }
        for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
          for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
            if (count[v2][u2]) {
              ++nonZeroPixelCount;
              const size_t x0 = u0 + u2;
              const size_t y0 = v0 + v2;
              const size_t location0 = y0 * image.getWidth() + x0;
              const size_t c = count[v2][u2];
              const size_t c2 = c / 2;
              occupancyMapTemp[location0] = iteration + 1;
              for (size_t k = 0; k < 3; ++k) {
                image.setValue(k, x0, y0, T((values[v2][u2][k] + c2) / c));
              }
              values[v2][u2] = 0;
              count[v2][u2] = 0UL;
            }
          }
        }
        ++iteration;
      }
    }
  }
}

/* pull push filling algorithm */
template <typename T>
int PCCEncoder::mean4w(T p1, unsigned char w1, T p2, unsigned char w2, T p3, unsigned char w3, T p4, unsigned char w4) {
  int result = (p1*int(w1) + p2*int(w2) + p3*int(w3) + p4*int(w4)) / (int(w1) + int(w2) + int(w3) + int(w4));
  return result;
}

// Generates a weighted mipmap 
template <typename T>
void PCCEncoder::pullPushMip(PCCImage<T, 3> &image, PCCImage<T, 3> &mip, std::vector<uint32_t>& occupancyMap, std::vector<uint32_t> &mipOccupancyMap) {
  //allocate the mipmap with half the resolution
  mip.resize( ((image.getWidth()+1) / 2), ((image.getHeight()+1) / 2));
  mipOccupancyMap.resize( ((image.getWidth() + 1)/2) * ((image.getHeight() + 1) / 2), 0);
  unsigned char w1, w2, w3, w4;
  unsigned char val1, val2, val3, val4;
  int stride = image.getWidth();
  int height = image.getHeight();
  int newStride = ((image.getWidth() + 1) / 2);
  int x, y;
  for (y = 0; y < mip.getHeight(); ++y) {
    for (x = 0; x < mip.getWidth(); ++x) {
      if (occupancyMap[x * 2 + stride*(y * 2)] == 0)      w1 = 0; else w1 = 255;
      if ((x * 2 + 1 >= stride) || (occupancyMap[x * 2 + 1 + stride*(y * 2)] == 0))  w2 = 0; else w2 = 255;
      if ((y * 2 + 1 >= height) || (occupancyMap[x * 2 + stride*(y * 2 + 1)] == 0))    w3 = 0; else w3 = 255;
      if ((x * 2 + 1 >= stride) || (y * 2 + 1 >= height) || (occupancyMap[x * 2 + 1 + stride*(y * 2 + 1)] == 0))  w4 = 0; else w4 = 255;
      if (w1 + w2 + w3 + w4 > 0) {
        for (int cc = 0; cc < 3; cc++) {
          val1 = image.getValue(cc, x * 2, y * 2);
          if (x * 2 + 1 >= stride)  val2 = 0; else val2 = image.getValue(cc, x * 2 + 1, y * 2);
          if (y * 2 + 1 >= height)  val3 = 0; else val3 = image.getValue(cc, x * 2, y * 2 + 1);
          if ((x * 2 + 1 >= stride) || (y * 2 + 1 >= height)) val4 = 0; else val4 = image.getValue(cc, x * 2 + 1, y * 2 + 1);
          T newVal = mean4w(val1, w1, val2, w2, val3, w3, val4, w4);
          mip.setValue(cc, x, y, newVal);
        }
        mipOccupancyMap[x + newStride*y] = 1;
      }
    }
  }

}

// interpolate using mipmap
template <typename T>
void PCCEncoder::pullPushFill(PCCImage<T, 3> &image, PCCImage<T, 3> &mip, std::vector<uint32_t>& occupancyMap) {
  assert(((image.getWidth()+1) / 2) == mip.getWidth());
  assert(((image.getHeight()+1) / 2) == mip.getHeight());
  int stride = image.getWidth();
  int x, y, xUp, yUp;
  unsigned char w1, w2, w3, w4;
  for (yUp = 0; yUp < image.getHeight(); ++yUp) {
    y = yUp / 2;
    for (xUp = 0; xUp < image.getWidth(); ++xUp) {
      x = xUp / 2;
      if (occupancyMap[xUp + stride*yUp] == 0) {
        if ((xUp % 2 == 0) && (yUp % 2 == 0)) {
          w1 = 144;
          w2 = (x > 0 ? (unsigned char)48 : 0);
          w3 = (y > 0 ? (unsigned char)48 : 0);
          w4 = (((x > 0) && (y > 0)) ? (unsigned char)16 : 0);
          for (int cc = 0; cc < 3; cc++) {
            T val = mip.getValue(cc, x, y);
            T valLeft = (x > 0 ? mip.getValue(cc, x - 1, y) : 0);
            T valUp = (y > 0 ? mip.getValue(cc, x, y - 1) : 0);
            T valUpLeft = ((x > 0 && y > 0) ? mip.getValue(cc, x - 1, y - 1) : 0);
            T newVal = mean4w(val, w1, valLeft, w2, valUp, w3, valUpLeft, w4);
            image.setValue(cc, xUp, yUp, newVal);
          }
        } else if ((xUp % 2 == 1) && (yUp % 2 == 0)) {
          w1 = 144;
          w2 = (x < mip.getWidth() - 1 ? (unsigned char)48 : 0);
          w3 = (y > 0 ? (unsigned char)48 : 0);
          w4 = (((x < mip.getWidth() - 1) && (y > 0)) ? (unsigned char)16 : 0);
          for (int cc = 0; cc < 3; cc++) {
            T val = mip.getValue(cc, x, y);
            T valRight = (x < mip.getWidth() - 1 ? mip.getValue(cc, x + 1, y) : 0);
            T valUp = (y > 0 ? mip.getValue(cc, x, y - 1) : 0);
            T valUpRight = (((x < mip.getWidth() - 1) && (y > 0)) ? mip.getValue(cc, x + 1, y - 1) : 0);
            T newVal = mean4w(val, w1, valRight, w2, valUp, w3, valUpRight, w4);
            image.setValue(cc, xUp, yUp, newVal);
          }
        } else if ((xUp % 2 == 0) && (yUp % 2 == 1)) {
          w1 = 144;
          w2 = (x > 0 ? (unsigned char)48 : 0);
          w3 = (y < mip.getHeight() - 1 ? (unsigned char)48 : 0);
          w4 = (((x > 0) && (y < mip.getHeight() - 1)) ? (unsigned char)16 : 0);
          for (int cc = 0; cc < 3; cc++) {
            T val = mip.getValue(cc, x, y);
            T valLeft = (x > 0 ? mip.getValue(cc, x - 1, y) : 0);
            T valDown = ((y < mip.getHeight() - 1) ? mip.getValue(cc, x, y + 1) : 0);
            T valDownLeft = ((x > 0 && (y < mip.getHeight() - 1)) ? mip.getValue(cc, x - 1, y + 1) : 0);
            T newVal = mean4w(val, w1, valLeft, w2, valDown, w3, valDownLeft, w4);
            image.setValue(cc, xUp, yUp, newVal);
          }
        } else {
          w1 = 144;
          w2 = (x < mip.getWidth() - 1 ? (unsigned char)48 : 0);
          w3 = (y < mip.getHeight() - 1 ? (unsigned char)48 : 0);
          w4 = (((x < mip.getWidth() - 1) && (y < mip.getHeight() - 1)) ? (unsigned char)16 : 0);
          for (int cc = 0; cc < 3; cc++) {
            T val = mip.getValue(cc, x, y);
            T valRight = (x < mip.getWidth() - 1 ? mip.getValue(cc, x + 1, y) : 0);
            T valDown = ((y < mip.getHeight() - 1) ? mip.getValue(cc, x, y + 1) : 0);
            T valDownRight = (((x < mip.getWidth() - 1) && (y < mip.getHeight() - 1)) ? mip.getValue(cc, x + 1, y + 1) : 0);
            T newVal = mean4w(val, w1, valRight, w2, valDown, w3, valDownRight, w4);
            image.setValue(cc, xUp, yUp, newVal);
          }
        }
      }
    }
  }
}

template <typename T>
void PCCEncoder::dilatePullPush(PCCFrameContext& frame, PCCImage<T, 3> &image) {
  auto occupancyMapTemp = frame.getOccupancyMap();
  int i = 0;
  std::vector<PCCImage<T, 3>> mipVec;
  std::vector<std::vector<uint32_t>> mipOccupancyMapVec;
  int div = 2;
  int miplev = 0;

  // pull phase create the mipmap
  while (1) {
    mipVec.resize(mipVec.size() + 1);
    mipOccupancyMapVec.resize(mipOccupancyMapVec.size() + 1);
    div *= 2;
    if (miplev>0){
      pullPushMip(mipVec[miplev - 1], mipVec[miplev], mipOccupancyMapVec[miplev - 1], mipOccupancyMapVec[miplev]);
    } else {
      pullPushMip(image, mipVec[miplev], occupancyMapTemp, mipOccupancyMapVec[miplev]);
    }
    if (mipVec[miplev].getWidth() <= 4 || mipVec[miplev].getHeight() <= 4) {
      break;
    }
    ++miplev;
  }
  miplev++;
#if DEBUG_PATCH
  for (int k = 0; k<miplev; k++) {
    char buf[100]; sprintf(buf, "mip%02i.rgb", k);
    std::string filename = addVideoFormat(buf, mipVec[k].getWidth(), mipVec[k].getHeight(), false);
    mipVec[k].write(filename, 1);
  }
#endif
  // push phase: refill
  for (i = miplev - 1; i >= 0; --i) {
    if (i>0) {
      pullPushFill(mipVec[i - 1], mipVec[i], mipOccupancyMapVec[i - 1]);
    } else {
      pullPushFill(image, mipVec[i], occupancyMapTemp);
    }
  }
#if DEBUG_PATCH
  for (int k = 0; k<miplev; k++) {
    char buf[100]; sprintf(buf, "mipfill%02i.rgb", k);
    std::string filename = addVideoFormat(buf, mipVec[k].getWidth(), mipVec[k].getHeight(), false);
    mipVec[k].write(filename, 1);
  }
#endif
}

void PCCEncoder::presmoothPointCloudColor(PCCPointSet3 &reconstruct, const PCCEncoderParameters params) {
  const size_t pointCount = reconstruct.getPointCount();
  PCCKdTree kdtree(reconstruct);
  PCCNNResult result;
  std::vector<PCCColor3B> temp;
  temp.resize(pointCount);
  for (size_t m = 0; m < pointCount; ++m) {
    temp[m] = reconstruct.getColor(m);
  }
  tbb::task_arena limited((int)params.nbThread_);
  limited.execute([&] {
    tbb::parallel_for(size_t(0), pointCount, [&](const size_t i) {
      //  for (size_t i = 0; i < pointCount; ++i) {

      PCCNNResult result;
      if (reconstruct.getBoundaryPointType(i) == 2) {
        kdtree.searchRadius( reconstruct[i], params.neighborCountColorSmoothing_, params.radius2ColorSmoothing_,result);
        PCCVector3D centroid(0.0);
        size_t neighborCount = 0;
        std::vector<uint8_t> Lum;
        for (size_t r = 0; r < result.count(); ++r) {
          const double dist2 = result.dist(r);
          if (dist2 > params.radius2ColorSmoothing_) {
            break;
          }
          ++neighborCount;
          const size_t index = result.indices(r);
          PCCColor3B color = reconstruct.getColor(index);
          centroid[0] += double(color[0]);
          centroid[1] += double(color[1]);
          centroid[2] += double(color[2]);

          double Y =
              0.2126 * double(color[0]) + 0.7152 * double(color[1]) + 0.0722 * double(color[2]);
          Lum.push_back(uint8_t(Y));
        }

        PCCColor3B color;
        if (neighborCount) {
          for (size_t k = 0; k < 3; ++k) {
            centroid[k] = double(int64_t(centroid[k] + (neighborCount / 2)) / neighborCount);
          }

          // Texture characterization
          double H = entropy(Lum, int(neighborCount));
          PCCColor3B colorQP = reconstruct.getColor(i);
          double distToCentroid2 = 0;
          for (size_t k = 0; k < 3; ++k) {
            distToCentroid2 += abs(centroid[k] - double(colorQP[k]));
          }
          if (distToCentroid2 >= double(params.thresholdColorSmoothing_) &&
              H < double(params.thresholdLocalEntropy_)) {
            color[0] = uint8_t(centroid[0]);
            color[1] = uint8_t(centroid[1]);
            color[2] = uint8_t(centroid[2]);
            temp[i] = color;
          }
        }
      }
    });
  });

  limited.execute([&] {
    tbb::parallel_for(size_t(0), pointCount, [&](const size_t i) {
      // for (size_t i = 0; i < pointCount; ++i) {
      reconstruct.setColor(i, temp[i]);
    });
  });
}

bool PCCEncoder::generateTextureVideo(const PCCGroupOfFrames &sources, PCCGroupOfFrames &reconstructs, PCCContext &context, const PCCEncoderParameters params) {
  auto& frames = context.getFrames();
  auto& videoTexture = context.getVideoTexture();
  bool ret = true;
  for( size_t i = 0; i < frames.size(); i++ ) {
    auto& frame = frames[ i ];
    assert( frame.getWidth() == context.getWidth() && frame.getHeight() == context.getHeight() );
    frame.setLosslessAtt(context.getLosslessAtt());
    size_t nbFrame = params_.oneLayerMode_ ? 1 : 2 ;
    if( params_.oneLayerMode_  && !params_.singleLayerPixelInterleaving_ ) {
      // create sub reconstruct point cloud
      PCCPointSet3 subReconstruct;
      vector<size_t> subReconstructIndex;
      subReconstructIndex.clear();
      if( reconstructs[ i ].hasColors() ) { subReconstruct.addColors(); }
      size_t numPointSub = 0, numPoint = reconstructs[ i ].getPointCount();
      auto& pointToPixel = frame.getPointToPixel();
      for( size_t j = 0; j < numPoint; j++ ) {
        if( pointToPixel[j][2] < nbFrame ||
            ( !params_.oneLayerMode_  && pointToPixel[j][2] == IntermediateLayerIndex ) ) {
          numPointSub++;
          subReconstruct.addPoint( reconstructs[ i ][ j ] );
          subReconstructIndex.push_back( j );
        }
      }
      subReconstruct.resize( numPointSub );
      sources[ i ].transfertColors( subReconstruct,
                                    int32_t( params_.bestColorSearchRange_ ),
                                    params_.losslessTexture_ == 1 );

      for( size_t j = 0; j < numPointSub; j++ ) {
        reconstructs[ i ].setColor( subReconstructIndex[ j ], subReconstruct.getColor( j ) );
        subReconstruct.setBoundaryPointType( j, reconstructs[ i ].getBoundaryPointType(subReconstructIndex[ j ]));
      }
      // color pre-smoothing
      if (!params_.losslessGeo_ && params_.flagColorPreSmoothing_) {
        presmoothPointCloudColor(subReconstruct, params);
        for (size_t j = 0; j < numPointSub; j++) {
          reconstructs[i].setColor(subReconstructIndex[j], subReconstruct.getColor(j));
        }
      }
    } else {
      sources[ i ].transfertColors( reconstructs[ i ],
                                    int32_t( params_.bestColorSearchRange_ ),
                                    params_.losslessTexture_ == 1 );
      // color pre-smoothing
      if (!params_.losslessGeo_ && params_.flagColorPreSmoothing_) {
        presmoothPointCloudColor(reconstructs[i], params);
      }
    }
    ret &= generateTextureVideo( reconstructs[ i ], frame, videoTexture, nbFrame );
  }
  return ret; 
}

bool PCCEncoder::generateTextureVideo( const PCCPointSet3& reconstruct, PCCFrameContext& frame,
                                       PCCVideoTexture &video, const size_t frameCount ) {
  auto& pointToPixel = frame.getPointToPixel();
  bool useMissedPointsSeparateVideo=frame.getUseMissedPointsSeparateVideo();
  bool losslessAtt = frame.getLosslessAtt();
  auto& missedPointsPatch=frame.getMissedPointsPatch();
  size_t numOfMPGeos =missedPointsPatch.getMPnumber();
  size_t numEddSavedPoints = missedPointsPatch.numEddSavedPoints;
  size_t pointCount = reconstruct.getPointCount();
  if(useMissedPointsSeparateVideo && losslessAtt) {
    pointCount = reconstruct.getPointCount() - numOfMPGeos - numEddSavedPoints;
    if(missedPointsPatch.sizeofcolor()<(numOfMPGeos+numEddSavedPoints)) {
      missedPointsPatch.resizecolor(numOfMPGeos+numEddSavedPoints);
    }
  }
  //  const size_t pointCount = reconstruct.getPointCount();
  if (!pointCount || !reconstruct.hasColors()) {
    return false;
  }

  const size_t shift = video.getFrameCount();
  video.resize( shift + frameCount );
  for (size_t f = 0; f < frameCount; ++f) {
    auto &image = video.getFrame( f + shift );
    image.resize( frame.getWidth(), frame.getHeight() );
    image.set(0);
  }

  std::vector<bool> mapD1;
  if( !params_.oneLayerMode_) {
    mapD1.resize( frame.getWidth() * frame.getHeight(), false );
  }

  std::vector<bool> markT1;
  if( ! params_.oneLayerMode_ && params_.removeDuplicatePoints_ ) {
    const size_t size = frame.getWidth() * frame.getHeight();
    markT1.resize( size );
    for(size_t i=0;i<size;i++){  markT1[ i ] = false; }
  }

  for (size_t i = 0; i < pointCount; ++i) {
    const PCCVector3<size_t> location = pointToPixel[i];
    const PCCColor3B color = reconstruct.getColor(i);
    const size_t u = location[0];
    const size_t v = location[1];
    const size_t f = location[2];
    if (params_.oneLayerMode_ && params_.singleLayerPixelInterleaving_) {
      if ((f == 0 && ((u + v) % 2 == 0)) || (f == 1 && ((u + v) % 2 == 1))) {
        auto &image = video.getFrame(shift);
        image.setValue(0, u, v, color[0]);
        image.setValue(1, u, v, color[1]);
        image.setValue(2, u, v, color[2]);
      }
    } else {
      if( f < frameCount ) {
        auto &image = video.getFrame(f + shift);
        image.setValue(0, u, v, color[0]);
        image.setValue(1, u, v, color[1]);
        image.setValue(2, u, v, color[2]);
      }
      if (!params_.oneLayerMode_ && params_.removeDuplicatePoints_ ) {
        if( f == 0 ) {
          if( !markT1[  v * frame.getWidth() + u ] ) {
            auto &image1 = video.getFrame(1 + shift);
            image1.setValue(0, u, v, color[0]);
            image1.setValue(1, u, v, color[1]);
            image1.setValue(2, u, v, color[2]);
          }
        } else {
          markT1[  v * frame.getWidth() + u ] = true;
        }
      }
    }
  }
  if(losslessAtt && useMissedPointsSeparateVideo) {
    if(frame.getEnhancedDeltaDepth()) {
      for (size_t i = 0; i < numEddSavedPoints; ++i) {
        const PCCColor3B color = reconstruct.getColor(pointCount+i);
        missedPointsPatch.r[i] = color[0];
        missedPointsPatch.g[i] = color[1];
        missedPointsPatch.b[i] = color[2];
      }
    }
    //missed points
    for (size_t i = 0; i < numOfMPGeos; ++i) {
      const PCCColor3B color = reconstruct.getColor(pointCount+numEddSavedPoints+i);
      missedPointsPatch.r[numEddSavedPoints+i] = color[0];
      missedPointsPatch.g[numEddSavedPoints+i] = color[1];
      missedPointsPatch.b[numEddSavedPoints+i] = color[2];
    }
  }
  return true;
}

int PCCEncoder::writeMetadata( const PCCMetadata &metadata, pcc::PCCBitstream &bitstream ) {
  auto &metadataEnabledFlags = metadata.getMetadataEnabledFlags();
  if (!metadataEnabledFlags.getMetadataEnabled()) {
    return 0;
  }
  bitstream.write<uint8_t>(metadata.getMetadataPresent());
  if (metadata.getMetadataPresent()) {
    if (metadataEnabledFlags.getScaleEnabled()) {
      bitstream.write<uint8_t>(metadata.getScalePresent());
      if (metadata.getScalePresent()) {
        bitstream.write<PCCVector3U>(metadata.getScale());
      }
    }
    if (metadataEnabledFlags.getOffsetEnabled()) {
      bitstream.write<uint8_t>(metadata.getOffsetPresent());
      if (metadata.getOffsetPresent()) {
        bitstream.write<PCCVector3I>(metadata.getOffset());
      }
    }
    if (metadataEnabledFlags.getRotationEnabled()) {
      bitstream.write<uint8_t>(metadata.getRotationPresent());
      if (metadata.getRotationPresent()) {
        bitstream.write<PCCVector3I>(metadata.getRotation());
      }
    }
    if (metadataEnabledFlags.getPointSizeEnabled()) {
      bitstream.write<uint8_t>(metadata.getPointSizePresent());
      if (metadata.getPointSizePresent()) {
        bitstream.write<uint16_t>(metadata.getPointSize());
      }
    }
    if (metadataEnabledFlags.getPointShapeEnabled()) {
      bitstream.write<uint8_t>(metadata.getPointShapePresent());
      if (metadata.getPointShapePresent()) {
        bitstream.write<uint8_t>(uint8_t(metadata.getPointShape()));
      }
    }
  }

  auto &lowerLevelMetadataEnabledFlags = metadata.getLowerLevelMetadataEnabledFlags();
  bitstream.write<uint8_t>(lowerLevelMetadataEnabledFlags.getMetadataEnabled());

  if (lowerLevelMetadataEnabledFlags.getMetadataEnabled()) {
    bitstream.write<uint8_t>(lowerLevelMetadataEnabledFlags.getScaleEnabled());
    bitstream.write<uint8_t>(lowerLevelMetadataEnabledFlags.getOffsetEnabled());
    bitstream.write<uint8_t>(lowerLevelMetadataEnabledFlags.getRotationEnabled());
    bitstream.write<uint8_t>(lowerLevelMetadataEnabledFlags.getPointSizeEnabled());
    bitstream.write<uint8_t>(lowerLevelMetadataEnabledFlags.getPointShapeEnabled());
  }
  return 1;
}

int PCCEncoder::compressMetadata(const PCCMetadata &metadata, o3dgc::Arithmetic_Codec &arithmeticEncoder) {
  auto &metadataEnabingFlags = metadata.getMetadataEnabledFlags();
  if (!metadataEnabingFlags.getMetadataEnabled()) {
    return 0;
  }
  static o3dgc::Static_Bit_Model   bModel0;
  static o3dgc::Adaptive_Bit_Model bModelMetadataPresent;
  arithmeticEncoder.encode(metadata.getMetadataPresent(), bModelMetadataPresent);

  if (metadata.getMetadataPresent()) {
    if (metadataEnabingFlags.getScaleEnabled()) {
      static o3dgc::Adaptive_Bit_Model bModelScalePresent;
      arithmeticEncoder.encode(metadata.getScalePresent(), bModelScalePresent);
      if (metadata.getScalePresent()) {
        EncodeUInt32(uint32_t(metadata.getScale()[0]), 32, arithmeticEncoder, bModel0);
        EncodeUInt32(uint32_t(metadata.getScale()[1]), 32, arithmeticEncoder, bModel0);
        EncodeUInt32(uint32_t(metadata.getScale()[2]), 32, arithmeticEncoder, bModel0);
      }
    }

    if (metadataEnabingFlags.getOffsetEnabled()) {
      static o3dgc::Adaptive_Bit_Model bModelOffsetPresent;
      arithmeticEncoder.encode(metadata.getOffsetPresent(), bModelOffsetPresent);
      if (metadata.getOffsetPresent()) {
        EncodeUInt32(o3dgc::IntToUInt(metadata.getOffset()[0]), 32, arithmeticEncoder, bModel0);
        EncodeUInt32(o3dgc::IntToUInt(metadata.getOffset()[1]), 32, arithmeticEncoder, bModel0);
        EncodeUInt32(o3dgc::IntToUInt(metadata.getOffset()[2]), 32, arithmeticEncoder, bModel0);
      }
    }

    if (metadataEnabingFlags.getRotationEnabled()) {
      static o3dgc::Adaptive_Bit_Model bModelRotationPresent;
      arithmeticEncoder.encode(metadata.getRotationPresent(), bModelRotationPresent);
      if (metadata.getRotationPresent()) {
        EncodeUInt32(o3dgc::IntToUInt(metadata.getRotation()[0]), 32, arithmeticEncoder, bModel0);
        EncodeUInt32(o3dgc::IntToUInt(metadata.getRotation()[1]), 32, arithmeticEncoder, bModel0);
        EncodeUInt32(o3dgc::IntToUInt(metadata.getRotation()[2]), 32, arithmeticEncoder, bModel0);
      }
    }

    if (metadataEnabingFlags.getPointSizeEnabled()) {
      static o3dgc::Adaptive_Bit_Model bModelPointSizePresent;
      arithmeticEncoder.encode(metadata.getPointSizePresent(), bModelPointSizePresent);
      if (metadata.getPointSizePresent()) {
        EncodeUInt32(uint32_t(metadata.getPointSize()), 16, arithmeticEncoder, bModel0);
      }
    }

    if (metadataEnabingFlags.getPointShapeEnabled()) {
      static o3dgc::Adaptive_Bit_Model bModelPointShapePresent;
      arithmeticEncoder.encode(metadata.getPointShapePresent(), bModelPointShapePresent);
      if (metadata.getPointShapePresent()) {
        EncodeUInt32(uint32_t(metadata.getPointShape()), 8, arithmeticEncoder, bModel0);
      }
    }
  }

  auto &lowerLevelMetadataEnabledFlags = metadata.getLowerLevelMetadataEnabledFlags();
  static o3dgc::Adaptive_Bit_Model bModelLowerLevelMetadataEnabled;
  arithmeticEncoder.encode(lowerLevelMetadataEnabledFlags.getMetadataEnabled(), bModelLowerLevelMetadataEnabled);

  if (lowerLevelMetadataEnabledFlags.getMetadataEnabled()) {
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelScaleEnabled;
    arithmeticEncoder.encode(lowerLevelMetadataEnabledFlags.getScaleEnabled(), bModelLowerLevelScaleEnabled);

    static o3dgc::Adaptive_Bit_Model bModelLowerLevelOffsetEnabled;
    arithmeticEncoder.encode(lowerLevelMetadataEnabledFlags.getOffsetEnabled(), bModelLowerLevelOffsetEnabled);

    static o3dgc::Adaptive_Bit_Model bModelLowerLevelRotationEnabled;
    arithmeticEncoder.encode(lowerLevelMetadataEnabledFlags.getRotationEnabled(), bModelLowerLevelRotationEnabled);

    static o3dgc::Adaptive_Bit_Model bModelLowerLevelPointSizeEnabled;
    arithmeticEncoder.encode(lowerLevelMetadataEnabledFlags.getPointSizeEnabled(), bModelLowerLevelPointSizeEnabled);

    static o3dgc::Adaptive_Bit_Model bModelLowerLevelPointShapeEnabled;
    arithmeticEncoder.encode(lowerLevelMetadataEnabledFlags.getPointShapeEnabled(), bModelLowerLevelPointShapeEnabled);
  }
  return 1;
}

int PCCEncoder::compressHeader( PCCContext &context, pcc::PCCBitstream &bitstream ){
  bitstream.write<uint8_t>( (uint8_t)( context.size() ) );
  if (!context.size()) {
    return 0;
  }
  bitstream.write<uint16_t>(uint16_t( context.getWidth () ) );
  bitstream.write<uint16_t>(uint16_t( context.getHeight() ) );
  bitstream.write<uint8_t> (uint8_t(params_.occupancyResolution_));
  bitstream.write<uint8_t> (uint8_t(params_.occupancyPrecision_));
  bitstream.write<uint8_t> (uint8_t(params_.gridSmoothing_));
  bitstream.write<uint8_t> (uint8_t(params_.radius2Smoothing_));
  bitstream.write<uint8_t> (uint8_t(params_.neighborCountSmoothing_)); 
  bitstream.write<uint8_t> (uint8_t(params_.radius2BoundaryDetection_));
  bitstream.write<uint8_t> (uint8_t(params_.thresholdSmoothing_));
  bitstream.write<uint8_t> (uint8_t(params_.losslessGeo_));
  bitstream.write<uint8_t> (uint8_t(params_.losslessTexture_));
  bitstream.write<uint8_t> (uint8_t(params_.noAttributes_));
  bitstream.write<uint8_t> (uint8_t(params_.losslessGeo444_));
  bitstream.write<uint8_t> (uint8_t(params_.useMissedPointsSeparateVideo_));
  bitstream.write<uint8_t> (uint8_t(params_.useOccupancyMapVideo_));
  bitstream.write<uint8_t> (uint8_t(params_.absoluteD1_));
  if (params_.absoluteD1_) {
    bitstream.write<uint8_t>(uint8_t(params_.sixDirectionMode_));
  }
  bitstream.write<uint8_t> (uint8_t(params_.binArithCoding_));
  bitstream.write<float>(params_.modelScale_);
  bitstream.write<PCCVector3<float> >(params_.modelOrigin_);
  writeMetadata(context.getGOFLevelMetadata(), bitstream);
  bitstream.write<uint8_t>(uint8_t(params_.flagColorSmoothing_));
  if (params_.flagColorSmoothing_) {
    bitstream.write<uint8_t>(uint8_t(params_.thresholdColorSmoothing_));
    bitstream.write<double> (double(params_.thresholdLocalEntropy_));
    bitstream.write<uint8_t>(uint8_t(params_.radius2ColorSmoothing_));
    bitstream.write<uint8_t>(uint8_t(params_.neighborCountColorSmoothing_));
  }
  if (params_.losslessGeo_) {
    bitstream.write<uint8_t>(uint8_t(params_.enhancedDeltaDepthCode_));
    bitstream.write<uint8_t>(uint8_t(params_.improveEDD_));
  }
  bitstream.write<uint8_t>(uint8_t(params_.deltaCoding_));
  bitstream.write<uint8_t>(uint8_t(params_.removeDuplicatePoints_));
  bitstream.write<uint8_t>(uint8_t(params_.oneLayerMode_));
  bitstream.write<uint8_t>(uint8_t(params_.singleLayerPixelInterleaving_));
  return 1;
}

void PCCEncoder::compressPatchMetaDataM42195( PCCFrameContext &frame, PCCFrameContext &preFrame, size_t numMatchedPatches,
                                              PCCBitstream &bitstream , o3dgc::Arithmetic_Codec &arithmeticEncoder,
                                              o3dgc::Static_Bit_Model &bModel0,
                                              uint8_t enable_flexible_patch_flag) {
  auto &patches = frame.getPatches();
  auto &prePatches = preFrame.getPatches();
  size_t patchCount = patches.size();
  size_t TopNmaxU0 = 0, maxU0 = 0;
  size_t TopNmaxV0 = 0, maxV0 = 0;
  size_t TopNmaxU1 = 0, maxU1 = 0;
  size_t TopNmaxV1 = 0, maxV1 = 0;
  size_t TopNmaxD1 = 0, maxD1 = 0;
  //get the maximum u0,v0,u1,v1 and d1.
  for (size_t patchIndex = 0; patchIndex < numMatchedPatches; ++patchIndex) {
    const auto &patch = patches[patchIndex];
    TopNmaxU0 = (std::max)(TopNmaxU0, patch.getU0());
    TopNmaxV0 = (std::max)(TopNmaxV0, patch.getV0());
    TopNmaxU1 = (std::max)(TopNmaxU1, patch.getU1());
    TopNmaxV1 = (std::max)(TopNmaxV1, patch.getV1());
    TopNmaxD1 = (std::max)(TopNmaxD1, patch.getD1());
  }
  uint8_t F = 1;  //true if the maximum value comes from the latter part.
  uint8_t A[5] = { 0, 0, 0, 0, 0 };
  for (size_t patchIndex = numMatchedPatches; patchIndex < patchCount; ++patchIndex) {
    const auto &patch = patches[patchIndex];
    maxU0 = (std::max)(maxU0, patch.getU0());
    maxV0 = (std::max)(maxV0, patch.getV0());
    maxU1 = (std::max)(maxU1, patch.getU1());
    maxV1 = (std::max)(maxV1, patch.getV1());
    maxD1 = (std::max)(maxD1, patch.getD1());

    A[0] = maxU0 > TopNmaxU0 ? 1 : A[0];
    A[1] = maxV0 > TopNmaxV0 ? 1 : A[1];
    A[2] = maxU1 > TopNmaxU1 ? 1 : A[2];
    A[3] = maxV1 > TopNmaxV1 ? 1 : A[3];
    A[4] = maxD1 > TopNmaxD1 ? 1 : A[4]; 
  }
  //Generate F and A.
  if (A[0]+A[1]+A[2]+A[3]+A[4] == 0) {  F = 0; }
  printf("numMatchedPatches:%d, F:%d,A:%d,%d,%d,%d,%d\n", (int)numMatchedPatches, F, A[0], A[1], A[2], A[3], A[4]);
  //printf("topNmaxU:%d,%d,%d,%d,%d\n", TopNmaxU0, TopNmaxV0, TopNmaxU1, TopNmaxV1, TopNmaxD1);
  //printf("maxU:%d,%d,%d,%d,%d\n", maxU0, maxV0, maxU1, maxV1, maxD1);
  //size_t topNPatch = numMatchedPatches;
  uint8_t bitCount[5];
  bitCount[0] = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxU0 + 1))); 
  bitCount[0] = maxU0 > TopNmaxU0 ? bitCount[0] : uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(TopNmaxU0 + 1)));
  bitCount[1] = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxV0 + 1)));
  bitCount[1] = maxV0 > TopNmaxV0 ? bitCount[1] : uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(TopNmaxV0 + 1)));
  bitCount[2] = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxU1 + 1)));
  bitCount[2] = maxU1 > TopNmaxU1 ? bitCount[2] : uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(TopNmaxU1 + 1)));
  bitCount[3] = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxV1 + 1)));
  bitCount[3] = maxV1 > TopNmaxV1 ? bitCount[3] : uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(TopNmaxV1 + 1)));
  bitCount[4] = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxD1 + 1)));
  bitCount[4] = maxD1 > TopNmaxD1 ? bitCount[4] : uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(TopNmaxD1 + 1)));

  //printf("bitCount:%d,%d,%d,%d,%d\n",bitCount[0],bitCount[1],bitCount[2],bitCount[3],bitCount[4]);
  //write F and A into bitstream.
  uint8_t flag = F;
  for (int i = 0; i < 5; i++) {
    flag = flag << 1;
    flag += A[i];
  }

  bitstream.write<uint8_t>(flag);
  for (int i = 0; i < 5; i++) {
    if (A[i])   bitstream.write<uint8_t>(bitCount[i]);
  }

  bool bBinArithCoding = params_.binArithCoding_ && (!params_.losslessGeo_) &&
      (params_.occupancyResolution_ == 16) && (params_.occupancyPrecision_ == 4);

  o3dgc::Adaptive_Bit_Model bModelPatchIndex, bModelU0, bModelV0, bModelU1, bModelV1, bModelD1,bModelIntSizeU0,bModelIntSizeV0;
  o3dgc::Adaptive_Bit_Model bModelSizeU0, bModelSizeV0, bModelAbsoluteD1;
  o3dgc::Adaptive_Data_Model orientationModel(4);
  o3dgc::Adaptive_Bit_Model orientationModel2;
  o3dgc::Adaptive_Data_Model orientationPatchModel(NumPatchOrientations - 1 + 2);
  o3dgc::Adaptive_Bit_Model orientationPatchFlagModel2;

  const uint8_t bitCountNumPatches =
      uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(patchCount)));//numMatchedPatches <= patchCount
  EncodeUInt32(uint32_t(numMatchedPatches), bitCountNumPatches, arithmeticEncoder, bModel0);

  bool useOneLayermode = params_.oneLayerMode_ || params_.singleLayerPixelInterleaving_;
  int64_t predIndex = 0;
  for (size_t patchIndex = 0; patchIndex < numMatchedPatches; ++patchIndex) {
    const auto &patch    = patches[patchIndex];
    const auto &prepatch = prePatches[patch.getBestMatchIdx()];

    const int64_t delta_index = patch.getBestMatchIdx() - predIndex;
    predIndex += (delta_index + 1);
    const int64_t delta_u0 = static_cast<int64_t>(patch.getU0() - prepatch.getU0());
    const int64_t delta_v0 = static_cast<int64_t>(patch.getV0() - prepatch.getV0());
    const int64_t delta_u1 = static_cast<int64_t>(patch.getU1() - prepatch.getU1());
    const int64_t delta_v1 = static_cast<int64_t>(patch.getV1() - prepatch.getV1());
    const int64_t delta_d1 = static_cast<int64_t>(patch.getD1() - prepatch.getD1());
    const int64_t deltaSizeU0 = static_cast<int64_t>(patch.getSizeU0() - prepatch.getSizeU0());
    const int64_t deltaSizeV0 = static_cast<int64_t>(patch.getSizeV0() - prepatch.getSizeV0());

    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(delta_index)), 0, bModel0, bModelPatchIndex);
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(delta_u0)),    0, bModel0, bModelU0);
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(delta_v0)),    0, bModel0, bModelV0);
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(delta_u1)),    0, bModel0, bModelU1);
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(delta_v1)),    0, bModel0, bModelV1);
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(delta_d1)),    0, bModel0, bModelD1);
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(deltaSizeU0)), 0, bModel0, bModelIntSizeU0);
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(deltaSizeV0)), 0, bModel0, bModelIntSizeV0);

    if ( params_.absoluteD1_ && params_.sixDirectionMode_ ) {
      arithmeticEncoder.encode(patch.getProjectionMode(), bModel0);
    }
  }

  int64_t prevSizeU0 = patches[numMatchedPatches-1].getSizeU0();
  int64_t prevSizeV0 = patches[numMatchedPatches-1].getSizeV0();
  for (size_t patchIndex = numMatchedPatches; patchIndex < patchCount; ++patchIndex) {
    const auto &patch = patches[patchIndex];
    EncodeUInt32(uint32_t(patch.getU0()), bitCount[0], arithmeticEncoder, bModel0);
    EncodeUInt32(uint32_t(patch.getV0()), bitCount[1], arithmeticEncoder, bModel0);
    if (enable_flexible_patch_flag) {
      bool flexible_patch_present_flag = (patch.getPatchOrientation() != 0);
      if (flexible_patch_present_flag) {
        arithmeticEncoder.encode(1, orientationPatchFlagModel2);
        arithmeticEncoder.encode(patch.getPatchOrientation() - 1, orientationPatchModel);
      } else {
        arithmeticEncoder.encode(0, orientationPatchFlagModel2);
      }
    }
    EncodeUInt32(uint32_t(patch.getU1()), bitCount[2], arithmeticEncoder, bModel0);
    EncodeUInt32(uint32_t(patch.getV1()), bitCount[3], arithmeticEncoder, bModel0);
    EncodeUInt32(uint32_t(patch.getD1()), bitCount[4], arithmeticEncoder, bModel0); 
    if (!params_.absoluteD1_  && (patch.getFrameProjectionMode() == 2)) {
      const uint8_t bitCountProjDir = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(2 + 1)));
      EncodeUInt32(uint32_t(patch.getProjectionMode()), bitCountProjDir, arithmeticEncoder, bModel0);
    }
    if ( params_.absoluteD1_ && params_.sixDirectionMode_ ) {
      arithmeticEncoder.encode(patch.getProjectionMode(), bModel0);
    }
    const int64_t deltaSizeU0 = static_cast<int64_t>(patch.getSizeU0()) - prevSizeU0;
    const int64_t deltaSizeV0 = static_cast<int64_t>(patch.getSizeV0()) - prevSizeV0;
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(deltaSizeU0)), 0, bModel0, bModelSizeU0);
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(deltaSizeV0)), 0, bModel0, bModelSizeV0);
    prevSizeU0 = patch.getSizeU0();
    prevSizeV0 = patch.getSizeV0();
    if (bBinArithCoding) {
      if (patch.getNormalAxis() == 0) {
        arithmeticEncoder.encode(0, orientationModel2);
      } else if (patch.getNormalAxis() == 1) {
        arithmeticEncoder.encode(1, orientationModel2);
        arithmeticEncoder.encode(0, bModel0);
      } else {
        arithmeticEncoder.encode(1, orientationModel2);
        arithmeticEncoder.encode(1, bModel0);
      }
    } else {
      arithmeticEncoder.encode(uint32_t(patch.getNormalAxis()), orientationModel);
    }
    compressMetadata(patch.getPatchLevelMetadata(), arithmeticEncoder);
  }
}

void PCCEncoder::buildBlockToPatch( PCCContext &context ){
  size_t sizeFrames = context.getFrames().size();
  PCCFrameContext preFrame = context.getFrames()[0];
  for( int i = 0; i < sizeFrames; i++ ){
    PCCFrameContext &frame = context.getFrames()[i];
    if (params_.losslessGeo_ && !context.getUseMissedPointsSeparateVideo()) {
      auto& patches = frame.getPatches();
      auto& missedPointsPatch = frame.getMissedPointsPatch();
      const size_t patchIndex = patches.size();
      patches.resize(patchIndex + 1);
      PCCPatch &dummyPatch = patches[patchIndex];
      dummyPatch.getIndex() = patchIndex;
      dummyPatch.getU0() = missedPointsPatch.u0;
      dummyPatch.getV0() = missedPointsPatch.v0;
      dummyPatch.getSizeU0() = missedPointsPatch.sizeU0;
      dummyPatch.getSizeV0() = missedPointsPatch.sizeV0;
      dummyPatch.getU1() = 0;
      dummyPatch.getV1() = 0;
      dummyPatch.getD1() = 0;
      dummyPatch.getNormalAxis() = 0;
      dummyPatch.getTangentAxis() = 1;
      dummyPatch.getBitangentAxis() = 2;
      dummyPatch.getOccupancyResolution() = missedPointsPatch.occupancyResolution;
      dummyPatch.getOccupancy() = missedPointsPatch.occupancy;
      dummyPatch.getLod() = params_.testLevelOfDetail_;
      dummyPatch.setBestMatchIdx() = -1;
      dummyPatch.getPatchOrientation() = 0;
      buildBlockToPatch(frame, preFrame, i);
      patches.pop_back();
    } else {
      buildBlockToPatch(frame, preFrame, i);
    }
    preFrame = frame;
  }
}


void PCCEncoder::buildBlockToPatch( PCCFrameContext &frame, PCCFrameContext &preFrame, size_t frameIndex ){
  auto& patches = frame.getPatches();
  const size_t patchCount = patches.size();
  const size_t blockToPatchWidth = frame.getWidth() / params_.occupancyResolution_;
  const size_t blockToPatchHeight = frame.getHeight() / params_.occupancyResolution_;
  const size_t blockCount = blockToPatchWidth * blockToPatchHeight;
  auto& blockToPatch = frame.getBlockToPatch();
  blockToPatch.resize(0);
  blockToPatch.resize(blockCount, 0);
  auto& interpolateMap = frame.getInterpolate();
  auto& fillingMap     = frame.getFilling();
  auto& minD1Map       = frame.getMinD1();
  auto& neighborMap    = frame.getNeighbor();
  for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
    auto &patch = patches[patchIndex];
    const auto& occupancy = patch.getOccupancy();
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        if (occupancy[v0 * patch.getSizeU0() + u0]) {
          blockToPatch[patch.patchBlock2CanvasBlock(u0, v0, blockToPatchWidth, blockToPatchHeight)] = patchIndex + 1;
        }
      }
    }
  }
}

void PCCEncoder::compressOccupancyMap( PCCContext &context, PCCBitstream& bitstream ){
  size_t sizeFrames = context.getFrames().size();
  PCCFrameContext preFrame = context.getFrames()[0];
  for( int i = 0; i < sizeFrames; i++ ){
    PCCFrameContext &frame = context.getFrames()[i];
    if (params_.losslessGeo_ && !context.getUseMissedPointsSeparateVideo()) {
      auto& patches = frame.getPatches();
      auto& missedPointsPatch = frame.getMissedPointsPatch();
      const size_t patchIndex = patches.size();
      patches.resize(patchIndex + 1);
      PCCPatch &dummyPatch = patches[patchIndex];
      dummyPatch.getIndex() = patchIndex;
      dummyPatch.getU0() = missedPointsPatch.u0;
      dummyPatch.getV0() = missedPointsPatch.v0;
      dummyPatch.getSizeU0() = missedPointsPatch.sizeU0;
      dummyPatch.getSizeV0() = missedPointsPatch.sizeV0;
      dummyPatch.getU1() = 0;
      dummyPatch.getV1() = 0;
      dummyPatch.getD1() = 0;
      dummyPatch.getNormalAxis() = 0;
      dummyPatch.getTangentAxis() = 1;
      dummyPatch.getBitangentAxis() = 2;
      dummyPatch.getOccupancyResolution() = missedPointsPatch.occupancyResolution;
      dummyPatch.getOccupancy() = missedPointsPatch.occupancy;
      dummyPatch.getLod() = params_.testLevelOfDetail_;
      dummyPatch.setBestMatchIdx() = -1;
      dummyPatch.getPatchOrientation() = 0;
      compressOccupancyMap(frame, bitstream, preFrame, i);
      patches.pop_back();
    } else {
      compressOccupancyMap(frame, bitstream, preFrame, i);
    }
    preFrame = frame;
  } 
}

void PCCEncoder::compressOccupancyMap(PCCFrameContext& frame, PCCBitstream &bitstream, PCCFrameContext& preFrame, size_t frameIndex ) {
  auto& patches = frame.getPatches();
  const size_t patchCount = patches.size();
  bitstream.write<uint32_t>(uint32_t(patchCount));
  bitstream.write<uint8_t>(uint8_t(params_.maxCandidateCount_));

  if (!params_.absoluteD1_) {
    bitstream.write<uint8_t>(uint8_t(params_.surfaceThickness_));
    bitstream.write<uint8_t>(uint8_t(patches[0].getFrameProjectionMode()));
  }

  o3dgc::Adaptive_Bit_Model interpolateModel;
  o3dgc::Adaptive_Bit_Model fillingModel;
  o3dgc::Adaptive_Data_Model minD1Model(3);
  o3dgc::Adaptive_Data_Model neighborModel(3);
  o3dgc::Arithmetic_Codec arithmeticEncoder;
  o3dgc::Static_Bit_Model bModel0;
  bool bBinArithCoding = params_.binArithCoding_ && (!params_.losslessGeo_) &&
      (params_.occupancyResolution_ == 16) && (params_.occupancyPrecision_ == 4);

  bool useOneLayermode = params_.oneLayerMode_ || params_.singleLayerPixelInterleaving_;

  uint8_t enable_flexible_patch_flag = params_.packingStrategy_;
  bitstream.write<uint8_t>(enable_flexible_patch_flag);

  arithmeticEncoder.set_buffer( 0x00ffffff );
  arithmeticEncoder.start_encoder();

  if((frameIndex == 0)||(!params_.deltaCoding_)) {
    size_t maxU0 = 0;
    size_t maxV0 = 0;
    size_t maxU1 = 0;
    size_t maxV1 = 0;
    size_t maxD1 = 0;
    size_t maxLod = 0;
    for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
      const auto &patch = patches[patchIndex];
      maxU0 = (std::max)(maxU0, patch.getU0());
      maxV0 = (std::max)(maxV0, patch.getV0());
      maxU1 = (std::max)(maxU1, patch.getU1());
      maxV1 = (std::max)(maxV1, patch.getV1());
      maxD1 = (std::max)(maxD1, patch.getD1());
      maxLod = (std::max)(maxLod, patch.getLod());
    }
    const uint8_t bitCountU0  = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxU0 + 1)));
    const uint8_t bitCountV0  = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxV0 + 1)));
    const uint8_t bitCountU1  = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxU1 + 1)));
    const uint8_t bitCountV1  = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxV1 + 1)));
    const uint8_t bitCountD1  = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxD1 + 1)));
    const uint8_t bitCountLod = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxLod + 1)));

    bitstream.write<uint8_t>(bitCountU0);
    bitstream.write<uint8_t>(bitCountV0);
    bitstream.write<uint8_t>(bitCountU1);
    bitstream.write<uint8_t>(bitCountV1);
    bitstream.write<uint8_t>(bitCountD1);
    bitstream.write<uint8_t>(bitCountLod);

    compressMetadata(frame.getFrameLevelMetadata(), arithmeticEncoder);

    o3dgc::Static_Bit_Model bModel0;
    o3dgc::Adaptive_Bit_Model bModelSizeU0, bModelSizeV0, bModelAbsoluteD1;
    o3dgc::Adaptive_Data_Model orientationModel(4);
    o3dgc::Adaptive_Bit_Model orientationModel2;
    o3dgc::Adaptive_Data_Model orientationPatchModel(NumPatchOrientations - 1 + 2);
    o3dgc::Adaptive_Bit_Model orientationPatchFlagModel2;
    int64_t prevSizeU0 = 0;
    int64_t prevSizeV0 = 0;
    for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
      const auto &patch = patches[patchIndex];
      EncodeUInt32(uint32_t(patch.getU0()), bitCountU0, arithmeticEncoder, bModel0);
      EncodeUInt32(uint32_t(patch.getV0()), bitCountV0, arithmeticEncoder, bModel0);
      if (enable_flexible_patch_flag) {
        bool flexible_patch_present_flag = (patch.getPatchOrientation() != 0);
        if (flexible_patch_present_flag) {
          arithmeticEncoder.encode(1, orientationPatchFlagModel2);
          arithmeticEncoder.encode(patch.getPatchOrientation()-1, orientationPatchModel);
        } else {
          arithmeticEncoder.encode(0, orientationPatchFlagModel2);
        }
      }
      EncodeUInt32(uint32_t(patch.getU1()), bitCountU1, arithmeticEncoder, bModel0);
      EncodeUInt32(uint32_t(patch.getV1()), bitCountV1, arithmeticEncoder, bModel0);
      EncodeUInt32(uint32_t(patch.getD1()), bitCountD1, arithmeticEncoder, bModel0);
      EncodeUInt32(uint32_t(patch.getLod()), bitCountLod, arithmeticEncoder, bModel0);
      if (!params_.absoluteD1_ && (patch.getFrameProjectionMode() == 2)) {
        const uint8_t bitCountProjDir = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(2 + 1)));
        EncodeUInt32(uint32_t(patch.getProjectionMode()), bitCountProjDir, arithmeticEncoder, bModel0);
      }
      if ( params_.absoluteD1_ && params_.sixDirectionMode_ ) {
        if (patch.getProjectionMode() == 0) {
          arithmeticEncoder.encode(0, bModel0);
        } else {
          arithmeticEncoder.encode(1, bModel0);
        }
      }

      const int64_t deltaSizeU0 = static_cast<int64_t>(patch.getSizeU0()) - prevSizeU0;
      const int64_t deltaSizeV0 = static_cast<int64_t>(patch.getSizeV0()) - prevSizeV0;
      arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(deltaSizeU0)), 0, bModel0, bModelSizeU0);
      arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(deltaSizeV0)), 0, bModel0, bModelSizeV0);
      prevSizeU0 = patch.getSizeU0();
      prevSizeV0 = patch.getSizeV0();
      if (bBinArithCoding) {
        if (patch.getNormalAxis() == 0) {
          arithmeticEncoder.encode(0, orientationModel2);
        } else if (patch.getNormalAxis() == 1) {
          arithmeticEncoder.encode(1, orientationModel2);
          arithmeticEncoder.encode(0, bModel0);
        } else {
          arithmeticEncoder.encode(1, orientationModel2);
          arithmeticEncoder.encode(1, bModel0);
        }
      } else {
        arithmeticEncoder.encode(uint32_t(patch.getNormalAxis()), orientationModel);
      }
      compressMetadata(patch.getPatchLevelMetadata(), arithmeticEncoder);
    }
  } else {
    size_t numMatchedPatches = frame.getNumMatchedPatches();
    compressPatchMetaDataM42195( frame, preFrame, numMatchedPatches, bitstream, arithmeticEncoder, bModel0,
                                 enable_flexible_patch_flag);
  }

  const size_t blockToPatchWidth = frame.getWidth() / params_.occupancyResolution_;
  const size_t blockToPatchHeight = frame.getHeight() / params_.occupancyResolution_;
  const size_t blockCount = blockToPatchWidth * blockToPatchHeight;
  auto& blockToPatch = frame.getBlockToPatch();
  blockToPatch.resize(0);
  blockToPatch.resize(blockCount, 0);
  auto& interpolateMap = frame.getInterpolate();
  auto& fillingMap     = frame.getFilling();
  auto& minD1Map       = frame.getMinD1();
  auto& neighborMap    = frame.getNeighbor();
  for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
    auto &patch = patches[patchIndex];
    const auto& occupancy = patch.getOccupancy();
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        if (occupancy[v0 * patch.getSizeU0() + u0]) {
          blockToPatch[patch.patchBlock2CanvasBlock(u0, v0, blockToPatchWidth, blockToPatchHeight)] = patchIndex + 1;
        }
      }
    }
  }
  std::vector<std::vector<size_t>> candidatePatches;
  candidatePatches.resize(blockCount);
  for (int64_t patchIndex = patchCount - 1; patchIndex >= 0; --patchIndex) {  // add actual patches based on their bounding box
    auto &patch = patches[patchIndex];
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        candidatePatches[patch.patchBlock2CanvasBlock(u0, v0, blockToPatchWidth, blockToPatchHeight)].push_back(patchIndex + 1);
      }
    }
  }
  if(!params_.useOccupancyMapVideo_) {
    for (auto &candidatePatch : candidatePatches) {  // add empty as potential candidate
      candidatePatch.push_back(0);
    }
  }
  o3dgc::Adaptive_Bit_Model candidateIndexModelBit[4];
  o3dgc::Adaptive_Data_Model candidateIndexModel(uint32_t(params_.maxCandidateCount_ + 2));
  const uint32_t bitCountPatchIndex =
      PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(patchCount + (params_.useOccupancyMapVideo_?0:1)));
  for (size_t p = 0; p < blockCount; ++p) {
    const size_t patchIndex = blockToPatch[p];
    const auto &candidates = candidatePatches[p];
    if (candidates.size() == 1 || (params_.useOccupancyMapVideo_ && patchIndex==0))  {
      // empty
    } else {
      const uint32_t candidateCount = uint32_t( (std::min)(candidates.size(), params_.maxCandidateCount_));
      bool found = false;
      for (uint32_t i = 0; i < candidateCount; ++i) {
        if (candidates[i] == patchIndex) {
          found = true;
          if (bBinArithCoding) {
            if (i == 0) {
              arithmeticEncoder.encode(0, candidateIndexModelBit[0]);
            } else if (i == 1) {
              arithmeticEncoder.encode(1, candidateIndexModelBit[0]);
              arithmeticEncoder.encode(0, candidateIndexModelBit[1]);
            } else if (i == 2) {
              arithmeticEncoder.encode(1, candidateIndexModelBit[0]);
              arithmeticEncoder.encode(1, candidateIndexModelBit[1]);
              arithmeticEncoder.encode(0, candidateIndexModelBit[2]);
            } else if (i == 3) {
              arithmeticEncoder.encode(1, candidateIndexModelBit[0]);
              arithmeticEncoder.encode(1, candidateIndexModelBit[1]);
              arithmeticEncoder.encode(1, candidateIndexModelBit[2]);
              arithmeticEncoder.encode(0, candidateIndexModelBit[3]);
            }
          } else {
            arithmeticEncoder.encode(i, candidateIndexModel);
          }
          break;
        }
      }
      if (!found) {
        if (bBinArithCoding) {
          arithmeticEncoder.encode(1, candidateIndexModelBit[0]);
          arithmeticEncoder.encode(1, candidateIndexModelBit[1]);
          arithmeticEncoder.encode(1, candidateIndexModelBit[2]);
          arithmeticEncoder.encode(1, candidateIndexModelBit[3]);
        } else {
          arithmeticEncoder.encode(uint32_t(params_.maxCandidateCount_), candidateIndexModel);
        }
        EncodeUInt32(uint32_t(patchIndex), bitCountPatchIndex, arithmeticEncoder, bModel0);
      }
    }

    if( patchIndex > 0 ) {
      if( params_.absoluteD1_ && params_.oneLayerMode_ ) {
        arithmeticEncoder.encode(uint32_t(interpolateMap[ p ]), interpolateModel);
        if( interpolateMap[ p ] > 0 ) {
          arithmeticEncoder.encode(uint32_t(neighborMap[ p ] - 1), neighborModel);
        }
        arithmeticEncoder.encode(uint32_t(minD1Map[ p ] ), minD1Model);
        if( minD1Map[ p ] > 1 || interpolateMap[ p ] > 0 ) {
          arithmeticEncoder.encode(uint32_t(fillingMap[ p ] ), fillingModel);
        }
      }
    }
  }

  if(!params_.useOccupancyMapVideo_) {
    const size_t blockSize0 = params_.occupancyResolution_ / params_.occupancyPrecision_;
    const size_t pointCount0 = blockSize0 * blockSize0;
    const size_t traversalOrderCount = 4;
    std::vector<std::vector<std::pair<size_t, size_t>>> traversalOrders;
    traversalOrders.resize(traversalOrderCount);
    for (size_t k = 0; k < traversalOrderCount; ++k) {
      auto &traversalOrder = traversalOrders[k];
      traversalOrder.reserve(pointCount0);
      if (k == 0) {
        for (size_t v1 = 0; v1 < blockSize0; ++v1) {
          for (size_t u1 = 0; u1 < blockSize0; ++u1) {
            traversalOrder.push_back(std::make_pair(u1, v1));
          }
        }
      } else if (k == 1) {
        for (size_t v1 = 0; v1 < blockSize0; ++v1) {
          for (size_t u1 = 0; u1 < blockSize0; ++u1) {
            traversalOrder.push_back(std::make_pair(v1, u1));
          }
        }
      } else if (k == 2) {
        for (int64_t k = 1; k < int64_t(2 * blockSize0); ++k) {
          for (size_t u1 = (std::max)(int64_t(0), k - int64_t(blockSize0));
              int64_t(u1) < (std::min)(k, int64_t(blockSize0)); ++u1) {
            const size_t v1 = k - (u1 + 1);
            traversalOrder.push_back(std::make_pair(u1, v1));
          }
        }
      } else {
        for (int64_t k = 1; k < int64_t(2 * blockSize0); ++k) {
          for (size_t u1 = (std::max)(int64_t(0), k - int64_t(blockSize0));
              int64_t(u1) < (std::min)(k, int64_t(blockSize0)); ++u1) {
            const size_t v1 = k - (u1 + 1);
            traversalOrder.push_back(std::make_pair(blockSize0 - (1 + u1), v1));
          }
        }
      }
    }
    o3dgc::Adaptive_Bit_Model fullBlockModel, occupancyModel;
    o3dgc::Adaptive_Bit_Model traversalOrderIndexModel_Bit0;
    o3dgc::Adaptive_Bit_Model traversalOrderIndexModel_Bit1;
    o3dgc::Adaptive_Bit_Model runCountModel2;
    o3dgc::Adaptive_Bit_Model runLengthModel2[4];
    static size_t runLengthTable[16] = { 0,  1,  2,  3,  13,  7,  10,  4,  14,  9,  11,  5,  12,  8,  6, 15 };

    o3dgc::Adaptive_Data_Model traversalOrderIndexModel(uint32_t(traversalOrderCount + 1));
    o3dgc::Adaptive_Data_Model runCountModel((uint32_t)(pointCount0));
    o3dgc::Adaptive_Data_Model runLengthModel((uint32_t)(pointCount0));

    std::vector<bool> block0;
    std::vector<size_t> bestRuns;
    std::vector<size_t> runs;
    block0.resize(pointCount0);
    auto& width = frame.getWidth();
    auto& occupancyMap = frame.getOccupancyMap();
    for (size_t v0 = 0; v0 < blockToPatchHeight; ++v0) {
      for (size_t u0 = 0; u0 < blockToPatchWidth; ++u0) {
        const size_t patchIndex = blockToPatch[v0 * blockToPatchWidth + u0];
        if (patchIndex) {
          size_t fullCount = 0;
          for (size_t v1 = 0; v1 < blockSize0; ++v1) {
            const size_t v2 = v0 * params_.occupancyResolution_ + v1 * params_.occupancyPrecision_;
            for (size_t u1 = 0; u1 < blockSize0; ++u1) {
              const size_t u2 = u0 * params_.occupancyResolution_ + u1 * params_.occupancyPrecision_;
              bool isFull = false;
              for (size_t v3 = 0; v3 < params_.occupancyPrecision_ && !isFull; ++v3) {
                for (size_t u3 = 0; u3 < params_.occupancyPrecision_ && !isFull; ++u3) {
                  isFull |= occupancyMap[(v2 + v3) * width + u2 + u3] == 1;
                }
              }
              block0[v1 * blockSize0 + u1] = isFull;
              fullCount += isFull;
              for (size_t v3 = 0; v3 < params_.occupancyPrecision_; ++v3) {
                for (size_t u3 = 0; u3 < params_.occupancyPrecision_; ++u3) {
                  occupancyMap[(v2 + v3) * width + u2 + u3] = isFull;
                }
              }
            }
          }

          if (fullCount == pointCount0) {
            arithmeticEncoder.encode(true, fullBlockModel);
          } else {
            arithmeticEncoder.encode(false, fullBlockModel);
            bestRuns.clear();
            size_t bestTraversalOrderIndex = 0;
            for (size_t k = 0; k < traversalOrderCount; ++k) {
              auto &traversalOrder = traversalOrders[k];
              const auto &location0 = traversalOrder[0];
              bool occupancy0 = block0[location0.second * blockSize0 + location0.first];
              size_t runLength = 0;
              runs.clear();
              for (size_t p = 1; p < traversalOrder.size(); ++p) {
                const auto &location = traversalOrder[p];
                const bool occupancy1 = block0[location.second * blockSize0 + location.first];
                if (occupancy1 != occupancy0) {
                  runs.push_back(runLength);
                  occupancy0 = occupancy1;
                  runLength = 0;
                } else {
                  ++runLength;
                }
              }
              runs.push_back(runLength);
              if (k == 0 || runs.size() < bestRuns.size()) {
                bestRuns = runs;
                bestTraversalOrderIndex = k;
              }
            }

            assert(bestRuns.size() >= 2);
            const uint32_t runCountMinusOne = uint32_t(bestRuns.size() - 1);
            const uint32_t runCountMinusTwo = uint32_t(bestRuns.size() - 2);

            if (bBinArithCoding) {
              size_t bit1 = bestTraversalOrderIndex >> 1;
              size_t bit0 = bestTraversalOrderIndex & 0x1;
              arithmeticEncoder.encode(uint32_t(bit1), traversalOrderIndexModel_Bit1);
              arithmeticEncoder.encode(uint32_t(bit0), traversalOrderIndexModel_Bit0);
              arithmeticEncoder.ExpGolombEncode(uint32_t(runCountMinusTwo), 0, bModel0, runCountModel2);
            } else {
              arithmeticEncoder.encode(uint32_t(bestTraversalOrderIndex), traversalOrderIndexModel);
              arithmeticEncoder.encode(runCountMinusTwo, runCountModel);
            }

            const auto &location0 = traversalOrders[bestTraversalOrderIndex][0];
            bool occupancy0 = block0[location0.second * blockSize0 + location0.first];
            arithmeticEncoder.encode(occupancy0, occupancyModel);
            for (size_t r = 0; r < runCountMinusOne; ++r) {
              if (bBinArithCoding) {
                size_t runLengthIdx = runLengthTable[bestRuns[r]];
                size_t bit3 = (runLengthIdx >> 3) & 0x1;
                size_t bit2 = (runLengthIdx >> 2) & 0x1;
                size_t bit1 = (runLengthIdx >> 1) & 0x1;
                size_t bit0 = runLengthIdx & 0x1;
                arithmeticEncoder.encode(uint32_t(bit3), runLengthModel2[3]);
                arithmeticEncoder.encode(uint32_t(bit2), runLengthModel2[2]);
                arithmeticEncoder.encode(uint32_t(bit1), runLengthModel2[1]);
                arithmeticEncoder.encode(uint32_t(bit0), runLengthModel2[0]);
              } else {
                arithmeticEncoder.encode(uint32_t(bestRuns[r]), runLengthModel);
              }
            }
          }
        }
      }
    } //v0 < blockHeight
  }
  uint32_t compressedBitstreamSize = arithmeticEncoder.stop_encoder();
  bitstream.write( arithmeticEncoder.buffer(), compressedBitstreamSize );
}

void PCCEncoder::generateIntraEnhancedDeltaDepthImage(PCCFrameContext &frame, const PCCImageGeometry &imageRef, PCCImageGeometry &image) {
  size_t width = frame.getWidth();
  size_t height = frame.getHeight();
  image.resize(width, height);
  image.set(0);
  const int16_t infiniteDepth = (std::numeric_limits<int16_t>::max)();
  for (auto &patch : frame.getPatches()) {
    const size_t v0 = patch.getV0() * patch.getOccupancyResolution();
    const size_t u0 = patch.getU0() * patch.getOccupancyResolution();
    for (size_t v = 0; v < patch.getSizeV(); ++v) {
      for (size_t u = 0; u < patch.getSizeU(); ++u) {
        const size_t p = v * patch.getSizeU() + u;
        const int16_t d = patch.getDepth(0)[p];
        if (d < infiniteDepth) {
          const int16_t enhancedDeltaD = patch.getDepthEnhancedDeltaD()[p];
          size_t x, y;
          patch.patch2Canvas(u, v, width, height, x, y);
          image.setValue(0, x, y, uint16_t(enhancedDeltaD + imageRef.getValue(0, x, y)));
        }
      }
    }
  }

  if(!frame.getUseMissedPointsSeparateVideo()) {
    //missed point patch
    auto& missedPointsPatch = frame.getMissedPointsPatch();
    const size_t v0 = missedPointsPatch.v0 * missedPointsPatch.occupancyResolution;
    const size_t u0 = missedPointsPatch.u0 * missedPointsPatch.occupancyResolution;
    if (missedPointsPatch.size()) {
      for (size_t v = 0; v < missedPointsPatch.sizeV; ++v) {
        for (size_t u = 0; u < missedPointsPatch.sizeU; ++u) {
          const size_t p = v * missedPointsPatch.sizeU + u;
          if (missedPointsPatch.x[p] < infiniteDepth) {
            const size_t x = (u0 + u);
            const size_t y = (v0 + v);
            assert(x < width && y < height);
            image.setValue(0, x, y, uint16_t(missedPointsPatch.x[p]));
            if (params_.losslessGeo444_) {
              image.setValue(1, x, y, uint16_t(missedPointsPatch.y[p]));
              image.setValue(2, x, y, uint16_t(missedPointsPatch.z[p]));
            }
          }
        }
      }
    }
  }
  return;
}

