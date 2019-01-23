
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
#include <tbb/tbb.h>
#include "PCCDecoder.h"

using namespace pcc;
using namespace std;

uint32_t DecodeUInt32(const uint32_t bitCount, o3dgc::Arithmetic_Codec &arithmeticDecoder,
                      o3dgc::Static_Bit_Model &bModel0) {
  uint32_t decodedValue = 0;
  for (uint32_t i = 0; i < bitCount; ++i) {
    decodedValue += (arithmeticDecoder.decode(bModel0) << i);
  }
  return PCCFromLittleEndian<uint32_t>(decodedValue);
}

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
  if (!decompress( bitstream, context ) ) {
    return 0;
  }
  ret |= decode( context, reconstructs );
  return ret; 
}

int PCCDecoder::decompress( PCCBitstream &bitstream, PCCContext &context ) {
  if (!decompressHeader( context, bitstream ) ) {
    return 0;
  }
  if( useOccupancyMapVideo_ ) {
    bitstream.readVideoNalu( context.createVideoBitstream( PCCVideoType::OccupancyMap ) );
  }
  if (!absoluteD1_) {
     bitstream.readVideoNalu( context.createVideoBitstream( PCCVideoType::GeometryD0 ) );
     bitstream.readVideoNalu( context.createVideoBitstream( PCCVideoType::GeometryD1 ) );
  } else {
     bitstream.readVideoNalu( context.createVideoBitstream( PCCVideoType::Geometry ) );
  }

  if(losslessGeo_ && context.getUseMissedPointsSeparateVideo()) {
     bitstream.readVideoNalu( context.createVideoBitstream( PCCVideoType::GeometryMP ) );
  }
  if (!noAttributes_ ) {
    bitstream.readVideoNalu( context.createVideoBitstream( PCCVideoType::Texture ) );
    if(losslessTexture_ && context.getUseMissedPointsSeparateVideo()) {
      bitstream.readVideoNalu( context.createVideoBitstream( PCCVideoType::TextureMP ) );
      auto sizeMissedPointsTexture = bitstream.size();
    }
  }

  if( useOccupancyMapVideo_ ) {
  PCCVideoDecoder videoDecoder; 
  std::stringstream path;
  path << removeFileExtension( params_.compressedStreamPath_ ) << "_dec_GOF" << context.getIndex() << "_";
    auto& videoBitstream = context.getVideoBitstream( PCCVideoType::OccupancyMap );
    videoDecoder.decompress(context.getVideoOccupancyMap(), path.str() + "occupancy",
                            width_/occupancyPrecision_,
                            height_/occupancyPrecision_,
                            context.size(),
                            videoBitstream,
                            params_.videoDecoderOccupancyMapPath_, context
                            ,"", "", (losslessGeo_?losslessGeo444_:false), false, 1,params_.keepIntermediateFiles_
    );
  }

  decompressOccupancyMap(context, bitstream );
  if(losslessGeo_ && context.getUseMissedPointsSeparateVideo()) {
    readMissedPointsGeometryNumber( context, bitstream );
  }
  if (!noAttributes_ ) {
    if(losslessTexture_ && context.getUseMissedPointsSeparateVideo()) {
      auto sizeMissedPointsTexture = bitstream.size();
      readMissedPointsTextureNumber( context, bitstream );
    }
  }
  return 1;
}

int PCCDecoder::decode( PCCContext &context, PCCGroupOfFrames& reconstructs ){
  reconstructs.resize( context.size() );
  PCCVideoDecoder videoDecoder; 
  std::stringstream path;
  path << removeFileExtension( params_.compressedStreamPath_ ) << "_dec_GOF" << context.getIndex() << "_";

  const size_t nbyteGeo = losslessGeo_ ? 2 : 1;
  const size_t frameCountGeometry = oneLayerMode_ ? 1 : 2;
  const size_t frameCountTexture  = oneLayerMode_  ? 1 : 2;
  if (!absoluteD1_) {
    // Compress D0
    auto& videoBitstreamD0 =  context.getVideoBitstream( PCCVideoType::GeometryD0 );
    videoDecoder.decompress( context.getVideoGeometry(), path.str() + "geometryD0", width_, height_,
                             context.size(), videoBitstreamD0,
                             params_.videoDecoderPath_, context,
                             "", "", (losslessGeo_?losslessGeo444_:false), false, nbyteGeo,
                             params_.keepIntermediateFiles_ );
    std::cout << "geometry D0 video ->" << videoBitstreamD0.naluSize() << " B" << std::endl;

    // Compress D1
    auto& videoBitstreamD1 =  context.getVideoBitstream( PCCVideoType::GeometryD1 );
    videoDecoder.decompress(context.getVideoGeometryD1(), path.str() + "geometryD1", width_, height_,
                            context.size(), videoBitstreamD1,
                            params_.videoDecoderPath_, context,
                            "", "", (losslessGeo_?losslessGeo444_:false), false, nbyteGeo,
                            params_.keepIntermediateFiles_ );
    std::cout << "geometry D1 video ->" << videoBitstreamD1.naluSize() << " B" << std::endl;
    std::cout << "geometry video ->" << videoBitstreamD1.naluSize() + videoBitstreamD1.naluSize() << " B" << std::endl;
  } else {
    auto& videoBitstream =  context.getVideoBitstream( PCCVideoType::Geometry );
    videoDecoder.decompress(context.getVideoGeometry(), path.str() + "geometry", width_, height_,
                            context.size() * frameCountGeometry, videoBitstream,
                            params_.videoDecoderPath_, context,
                            "", "", losslessGeo_ & losslessGeo444_, false, nbyteGeo,
                            params_.keepIntermediateFiles_);
    std::cout << "geometry video ->" << videoBitstream.naluSize() << " B" << std::endl;
  }

  if(losslessGeo_ && context.getUseMissedPointsSeparateVideo()) {
    auto& videoBitstreamMP =  context.getVideoBitstream( PCCVideoType::Geometry );
    videoDecoder.decompress(context.getVideoMPsGeometry(), path.str() + "mps_geometry",
                            context.getMPGeoWidth(), context.getMPGeoHeight(),
                            context.size(), videoBitstreamMP,
                            params_.videoDecoderPath_, context,
                            "", "", 0,//losslessGeo_ & losslessGeo444_,
                            false, 2,//patchColorSubsampling, 2.nByteGeo 10 bit coding
                            params_.keepIntermediateFiles_ );
    assert(context.getMPGeoWidth() == context.getVideoMPsGeometry().getWidth());
    assert(context.getMPGeoHeight() == context.getVideoMPsGeometry().getHeight());
    generateMissedPointsGeometryfromVideo(context, reconstructs); //0. geo : decode arithmetic coding part
    std::cout << " missed points geometry -> " << videoBitstreamMP.naluSize() << " B "<<endl;

    //add missed point to reconstructs
    //fillMissedPoints(reconstructs, context, 0, params_.colorTransform_); //0. geo
  }

  GeneratePointCloudParameters generatePointCloudParameters;
  generatePointCloudParameters.occupancyResolution_          = occupancyResolution_;
  generatePointCloudParameters.occupancyPrecision_           = occupancyPrecision_;
  generatePointCloudParameters.gridSmoothing_                = gridSmoothing_;
  generatePointCloudParameters.neighborCountSmoothing_       = neighborCountSmoothing_;
  generatePointCloudParameters.radius2Smoothing_             = (double)radius2Smoothing_;
  generatePointCloudParameters.radius2BoundaryDetection_     = (double)radius2BoundaryDetection_;
  generatePointCloudParameters.thresholdSmoothing_           = (double)thresholdSmoothing_;
  generatePointCloudParameters.losslessGeo_                  = losslessGeo_ != 0;
  generatePointCloudParameters.losslessGeo444_               = losslessGeo444_ != 0;
  generatePointCloudParameters.nbThread_                     = params_.nbThread_;
  generatePointCloudParameters.absoluteD1_                   = absoluteD1_;
  generatePointCloudParameters.surfaceThickness              = context.getFrameContext(0).getSurfaceThickness();
  generatePointCloudParameters.ignoreLod_                    = true;
  generatePointCloudParameters.thresholdColorSmoothing_      = (double)thresholdColorSmoothing_;
  generatePointCloudParameters.thresholdLocalEntropy_        = (double)thresholdLocalEntropy_;
  generatePointCloudParameters.radius2ColorSmoothing_        = (double)radius2ColorSmoothing_;
  generatePointCloudParameters.neighborCountColorSmoothing_  = neighborCountColorSmoothing_;
  generatePointCloudParameters.flagColorSmoothing_           = (bool) flagColorSmoothing_ ;
  generatePointCloudParameters.enhancedDeltaDepthCode_       = ((losslessGeo_ != 0) ? enhancedDeltaDepthCode_ : false);
  generatePointCloudParameters.deltaCoding_                  = (params_.testLevelOfDetailSignaling_ > 0); // ignore LoD scaling for testing the signaling only
  generatePointCloudParameters.removeDuplicatePoints_        = removeDuplicatePoints_;
  generatePointCloudParameters.oneLayerMode_                 = oneLayerMode_;
  generatePointCloudParameters.singleLayerPixelInterleaving_ = singleLayerPixelInterleaving_;
  generatePointCloudParameters.sixDirectionMode_             = sixDirectionMode_;
  generatePointCloudParameters.path_                         = path.str();

  generatePointCloud( reconstructs, context, generatePointCloudParameters );

  if (!noAttributes_ ) {
    const size_t nbyteTexture = 1;
    auto& videoBitstream = context.getVideoBitstream( PCCVideoType::Texture );
    videoDecoder.decompress( context.getVideoTexture(),
                             path.str() + "texture", width_, height_,
                             context.size() * frameCountTexture, videoBitstream,
                             params_.videoDecoderPath_,
                             context,
                             params_.inverseColorSpaceConversionConfig_,
                             params_.colorSpaceConversionPath_,
                             losslessTexture_ != 0, params_.patchColorSubsampling_, nbyteTexture,
                             params_.keepIntermediateFiles_ );
    std::cout << "texture video  ->" << videoBitstream.naluSize() << " B" << std::endl;

    if(losslessTexture_ && context.getUseMissedPointsSeparateVideo()) {
      auto& videoBitstreamMP = context.getVideoBitstream( PCCVideoType::TextureMP );
      videoDecoder.decompress( context.getVideoMPsTexture(),
                               path.str() + "mps_texture",
                               context.getMPAttWidth(), context.getMPAttHeight(),
                               context.size(), videoBitstreamMP,
                               //frameCount*2??
                               params_.videoDecoderPath_,
                               context,
                               params_.inverseColorSpaceConversionConfig_,
                               params_.colorSpaceConversionPath_,
                               1, //losslessTexture_ != 0,
                               0, //params_.patchColorSubsampling_,
                               nbyteTexture, //nbyteTexture,
                               params_.keepIntermediateFiles_ );
      generateMissedPointsTexturefromVideo(context, reconstructs);
      std::cout << " missed points texture -> " << videoBitstreamMP.naluSize() << " B"<<endl;
    }
  }
  colorPointCloud(reconstructs, context, noAttributes_ != 0, params_.colorTransform_,
                  generatePointCloudParameters);
  return 0;
}

int PCCDecoder::readMetadata( PCCMetadata &metadata, PCCBitstream &bitstream ) {
  auto &metadataEnabledFlags = metadata.getMetadataEnabledFlags();
  if (!metadataEnabledFlags.getMetadataEnabled()) {
    return 0;
  }
  uint8_t  tmp;
  bitstream.read<uint8_t>(tmp);
  metadata.getMetadataPresent() = static_cast<bool>(tmp);
  if (metadata.getMetadataPresent()) {
    if (metadataEnabledFlags.getScaleEnabled()) {
      bitstream.read<uint8_t>(tmp);
      metadata.getScalePresent() = static_cast<bool>(tmp);
      if (metadata.getScalePresent()) {
        bitstream.read<PCCVector3U>(metadata.getScale());
      }
    }
    if (metadataEnabledFlags.getOffsetEnabled()) {
      bitstream.read<uint8_t>(tmp);
      metadata.getOffsetPresent() = static_cast<bool>(tmp);
      if (metadata.getOffsetPresent()) {
        bitstream.read<PCCVector3I>(metadata.getOffset());
      }
    }
    if (metadataEnabledFlags.getRotationEnabled()) {
      bitstream.read<uint8_t>(tmp);
      metadata.getRotationPresent() = static_cast<bool>(tmp);
      if (metadata.getRotationPresent()) {
        bitstream.read<PCCVector3I>(metadata.getRotation());
      }
    }
    if (metadataEnabledFlags.getPointSizeEnabled()) {
      bitstream.read<uint8_t>(tmp);
      metadata.getPointSizePresent() = static_cast<bool>(tmp);
      if (metadata.getPointSizePresent()) {
        bitstream.read<uint16_t>(metadata.getPointSize());
      }
    }
    if (metadataEnabledFlags.getPointShapeEnabled()) {
      bitstream.read<uint8_t>(tmp);
      metadata.getPointShapePresent() = static_cast<bool>(tmp);
      if (metadata.getPointShapePresent()) {
        bitstream.read<uint8_t>(tmp);
        metadata.getPointShape() = static_cast<PointShape>(tmp);
      }
    }
  }

  auto &lowerLevelMetadataEnabledFlags = metadata.getLowerLevelMetadataEnabledFlags();
  bitstream.read<uint8_t>(tmp);
  lowerLevelMetadataEnabledFlags.getMetadataEnabled() = static_cast<bool>(tmp);
  if (lowerLevelMetadataEnabledFlags.getMetadataEnabled()) {
    bitstream.read<uint8_t>(tmp);
    lowerLevelMetadataEnabledFlags.getScaleEnabled() = static_cast<bool>(tmp);
    bitstream.read<uint8_t>(tmp);
    lowerLevelMetadataEnabledFlags.getOffsetEnabled() = static_cast<bool>(tmp);
    bitstream.read<uint8_t>(tmp);
    lowerLevelMetadataEnabledFlags.getRotationEnabled() = static_cast<bool>(tmp);
    bitstream.read<uint8_t>(tmp);
    lowerLevelMetadataEnabledFlags.getPointSizeEnabled() = static_cast<bool>(tmp);
    bitstream.read<uint8_t>(tmp);
    lowerLevelMetadataEnabledFlags.getPointShapeEnabled() = static_cast<bool>(tmp);
  }
  return 1;
}

int PCCDecoder::decompressMetadata( PCCMetadata &metadata, o3dgc::Arithmetic_Codec &arithmeticDecoder) {
  auto &metadataEnabingFlags = metadata.getMetadataEnabledFlags();
  if (!metadataEnabingFlags.getMetadataEnabled()) {
    return 0;
  }
  static o3dgc::Static_Bit_Model   bModel0;
  static o3dgc::Adaptive_Bit_Model bModelMetadataPresent;
  metadata.getMetadataPresent() = arithmeticDecoder.decode(bModelMetadataPresent);
  if (metadata.getMetadataPresent()) {
    if (metadataEnabingFlags.getScaleEnabled()) {
      static o3dgc::Adaptive_Bit_Model bModelScalePresent;
      metadata.getScalePresent() = arithmeticDecoder.decode(bModelScalePresent);
      if (metadata.getScalePresent()) {
        metadata.getScale()[0] = DecodeUInt32(32, arithmeticDecoder, bModel0);
        metadata.getScale()[1] = DecodeUInt32(32, arithmeticDecoder, bModel0);
        metadata.getScale()[2] = DecodeUInt32(32, arithmeticDecoder, bModel0);
      }
    }
    if (metadataEnabingFlags.getOffsetEnabled()) {
      static o3dgc::Adaptive_Bit_Model bModelOffsetPresent;
      metadata.getOffsetPresent() = arithmeticDecoder.decode(bModelOffsetPresent);
      if (metadata.getOffsetPresent()) {
        metadata.getOffset()[0] = (int32_t)o3dgc::UIntToInt(DecodeUInt32(32, arithmeticDecoder, bModel0));
        metadata.getOffset()[1] = (int32_t)o3dgc::UIntToInt(DecodeUInt32(32, arithmeticDecoder, bModel0));
        metadata.getOffset()[2] = (int32_t)o3dgc::UIntToInt(DecodeUInt32(32, arithmeticDecoder, bModel0));
      }
    }
    if (metadataEnabingFlags.getRotationEnabled()) {
      static o3dgc::Adaptive_Bit_Model bModelRotationPresent;
      metadata.getRotationPresent() = arithmeticDecoder.decode(bModelRotationPresent);
      if (metadata.getRotationPresent()) {
        metadata.getRotation()[0] = (int32_t)o3dgc::UIntToInt(DecodeUInt32(32, arithmeticDecoder, bModel0));
        metadata.getRotation()[1] = (int32_t)o3dgc::UIntToInt(DecodeUInt32(32, arithmeticDecoder, bModel0));
        metadata.getRotation()[2] = (int32_t)o3dgc::UIntToInt(DecodeUInt32(32, arithmeticDecoder, bModel0));
      }
    }
    if (metadataEnabingFlags.getPointSizeEnabled()) {
      static o3dgc::Adaptive_Bit_Model bModelPointSizePresent;
      metadata.getPointSizePresent() = arithmeticDecoder.decode(bModelPointSizePresent);
      if (metadata.getPointSizePresent()) {
        metadata.getPointSize() = DecodeUInt32(16, arithmeticDecoder, bModel0);
      }
    }
    if (metadataEnabingFlags.getPointShapeEnabled()) {
      static o3dgc::Adaptive_Bit_Model bModelPointShapePresent;
      metadata.getPointShapePresent() = arithmeticDecoder.decode(bModelPointShapePresent);
      if (metadata.getPointShapePresent()) {
        metadata.getPointShape() = static_cast<PointShape>(DecodeUInt32(8, arithmeticDecoder, bModel0));
      }
    }
  }

  auto &lowerLevelMetadataEnabledFlags = metadata.getLowerLevelMetadataEnabledFlags();
  static o3dgc::Adaptive_Bit_Model bModelLowerLevelMetadataEnabled;
  lowerLevelMetadataEnabledFlags.getMetadataEnabled() = arithmeticDecoder.decode(bModelLowerLevelMetadataEnabled);

  if (lowerLevelMetadataEnabledFlags.getMetadataEnabled()) {
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelScaleEnabled;
    lowerLevelMetadataEnabledFlags.getScaleEnabled() = arithmeticDecoder.decode(bModelLowerLevelScaleEnabled);
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelOffsetEnabled;
    lowerLevelMetadataEnabledFlags.getOffsetEnabled() = arithmeticDecoder.decode(bModelLowerLevelOffsetEnabled);
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelRotationEnabled;
    lowerLevelMetadataEnabledFlags.getRotationEnabled() = arithmeticDecoder.decode(bModelLowerLevelRotationEnabled);
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelPointSizeEnabled;
    lowerLevelMetadataEnabledFlags.getPointSizeEnabled() = arithmeticDecoder.decode(bModelLowerLevelPointSizeEnabled);
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelPointShapeEnabled;
    lowerLevelMetadataEnabledFlags.getPointShapeEnabled() = arithmeticDecoder.decode(bModelLowerLevelPointShapeEnabled);
  }
  return 1;
}

int PCCDecoder::decompressHeader( PCCContext &context, PCCBitstream &bitstream ){
  uint8_t groupOfFramesSize;
  bitstream.read<uint8_t>( groupOfFramesSize );
  if (!groupOfFramesSize) {
    return 0;
  }
  context.resize( groupOfFramesSize );
  bitstream.read<uint16_t>( width_ );
  bitstream.read<uint16_t>( height_ );
  bitstream.read<uint8_t> ( occupancyResolution_ );
  bitstream.read<uint8_t> ( occupancyPrecision_ );
  uint8_t gridSmoothing;
  bitstream.read<uint8_t>(gridSmoothing);
  gridSmoothing_ = gridSmoothing > 0;
  bitstream.read<uint8_t> ( radius2Smoothing_ );
  bitstream.read<uint8_t> ( neighborCountSmoothing_ );
  bitstream.read<uint8_t> ( radius2BoundaryDetection_ );
  bitstream.read<uint8_t> ( thresholdSmoothing_ );
  bitstream.read<uint8_t> ( losslessGeo_ );
  bitstream.read<uint8_t> ( losslessTexture_ );
  bitstream.read<uint8_t> ( noAttributes_ );
  bitstream.read<uint8_t> ( losslessGeo444_);
  useMissedPointsSeparateVideo_=true;
  useOccupancyMapVideo_=true;
  bitstream.read<uint8_t> ( useMissedPointsSeparateVideo_);
  bitstream.read<uint8_t> ( useOccupancyMapVideo_);
  uint8_t absD1, binArithCoding,deltaCoding, sixDirection;
  bitstream.read<uint8_t> ( absD1 );
  absoluteD1_ = absD1 > 0;
  if (absoluteD1_) {
    bitstream.read<uint8_t>(sixDirection);
    sixDirectionMode_ = sixDirection > 0;
  }

  bitstream.read<uint8_t> ( binArithCoding );
  binArithCoding_ =  binArithCoding > 0;
  context.getWidth()  = width_;
  context.getHeight() = height_; 
  bitstream.read<float>(modelScale_);
  bitstream.read<PCCVector3<float> >(modelOrigin_);
  readMetadata(context.getGOFLevelMetadata(), bitstream);
  bitstream.read<uint8_t>(flagColorSmoothing_);
  if (flagColorSmoothing_) {
    bitstream.read<uint8_t>(thresholdColorSmoothing_);
    bitstream.read<double>(thresholdLocalEntropy_);
    bitstream.read<uint8_t>(radius2ColorSmoothing_);
    bitstream.read<uint8_t>(neighborCountColorSmoothing_);
  }
  enhancedDeltaDepthCode_ = false;
  if (losslessGeo_) {
    uint8_t enhancedDeltaDepthCode;
    bitstream.read<uint8_t>(enhancedDeltaDepthCode);
    enhancedDeltaDepthCode_ = enhancedDeltaDepthCode > 0;
  }
  bitstream.read<uint8_t> ( deltaCoding );
  deltaCoding_ = deltaCoding > 0;

  uint8_t removeDuplicatePoints;
  bitstream.read<uint8_t> ( removeDuplicatePoints );
  removeDuplicatePoints_ = removeDuplicatePoints > 0;

  uint8_t oneLayerMode;
  bitstream.read<uint8_t> ( oneLayerMode );
  oneLayerMode_ = oneLayerMode > 0;

  uint8_t singleLayerPixelInterleave;
  bitstream.read<uint8_t>(singleLayerPixelInterleave);
  singleLayerPixelInterleaving_ = singleLayerPixelInterleave > 0;

  context.setLosslessGeo444(losslessGeo444_);
  context.setLossless(losslessGeo_);
  context.setLosslessAtt(losslessTexture_);
  context.setMPGeoWidth(64);
  context.setMPAttWidth(64);
  context.setMPGeoHeight(0);
  context.setMPAttHeight(0);
  context.setUseMissedPointsSeparateVideo(useMissedPointsSeparateVideo_);
  context.setUseOccupancyMapVideo(useOccupancyMapVideo_);
  context.setEnhancedDeltaDepth(enhancedDeltaDepthCode_);
  auto& frames = context.getFrames();
  for(size_t i=0; i<frames.size(); i++) {
    frames[i].setLosslessGeo(losslessGeo_);
    frames[i].setLosslessGeo444(losslessGeo444_);
    frames[i].setLosslessAtt(losslessTexture_);
    frames[i].setEnhancedDeltaDepth(enhancedDeltaDepthCode_);
    frames[i].setUseMissedPointsSeparateVideo(useMissedPointsSeparateVideo_);
    frames[i].setUseOccupancyMapVideo(useOccupancyMapVideo_);
  }
  return 1;
}

void PCCDecoder::readMissedPointsGeometryNumber(PCCContext& context, PCCBitstream &bitstream) {
  size_t maxHeight = 0;
  size_t MPwidth;
  size_t numofMPs;
  bitstream.read<size_t>( MPwidth );
  for (auto &framecontext : context.getFrames() ) {
    bitstream.read<size_t>( numofMPs );
    framecontext.getMissedPointsPatch().setMPnumber(size_t(numofMPs));
    if(context.getLosslessGeo444()) {
      framecontext.getMissedPointsPatch().resize(numofMPs);
    } else {
      framecontext.getMissedPointsPatch().resize(numofMPs*3);
    }
    size_t height = (3*numofMPs)/MPwidth+1;
    size_t heightby8= height/8;
    if(heightby8*8!=height) {
      height = (heightby8+1)*8;
    }
    maxHeight = (std::max)( maxHeight, height );
  }
  context.setMPGeoWidth(size_t(MPwidth));
  context.setMPGeoHeight(size_t(maxHeight));
}

void PCCDecoder::readMissedPointsTextureNumber(PCCContext& context, PCCBitstream &bitstream) {
  size_t maxHeight = 0;
  size_t MPwidth;
  size_t numofMPs;
  bitstream.read<size_t>( MPwidth );
  for (auto &framecontext : context.getFrames() ) {
    bitstream.read<size_t>( numofMPs );
    framecontext.getMissedPointsPatch().setMPnumbercolor(size_t(numofMPs));
    framecontext.getMissedPointsPatch().resizecolor(numofMPs);
    size_t height = numofMPs/MPwidth+1;
    size_t heightby8= height/8;
    if(heightby8*8!=height) {
      height = (heightby8+1)*8;
    }
    maxHeight = (std::max)( maxHeight, height );
  }
  context.setMPAttWidth(size_t(MPwidth));
  context.setMPAttHeight(size_t(maxHeight));
}

void PCCDecoder::generateMissedPointsGeometryfromVideo(PCCContext& context, PCCGroupOfFrames& reconstructs) {
  const size_t gofSize = context.getGofSize();
  auto& videoMPsGeometry = context.getVideoMPsGeometry();
  videoMPsGeometry.resize(gofSize);
  for (auto &framecontext : context.getFrames() ) {
    const size_t shift = framecontext.getIndex();
    framecontext.setLosslessGeo(context.getLosslessGeo());
    framecontext.setLosslessGeo444(context.getLosslessGeo444());
    generateMPsGeometryfromImage(context, framecontext, reconstructs, shift);
    cout<<"generate Missed Points (Geometry) : frame "<<shift<<", # of Missed Points Geometry : "<<framecontext.getMissedPointsPatch().size()<<endl;
  }
  cout<<"MissedPoints Geometry [done]"<<endl;
}

void PCCDecoder::generateMissedPointsTexturefromVideo(PCCContext& context, PCCGroupOfFrames& reconstructs) {
  const size_t gofSize = context.getGofSize();
  auto& videoMPsTexture = context.getVideoMPsTexture();
  videoMPsTexture.resize(gofSize);
  for (auto &framecontext : context.getFrames() ) {
    const size_t shift = framecontext.getIndex(); //
    framecontext.setLosslessAtt(context.getLosslessAtt());
    generateMPsTexturefromImage(context, framecontext, reconstructs, shift);
    cout<<"generate Missed Points (Texture) : frame "<<shift<<", # of Missed Points Texture : "<<framecontext.getMissedPointsPatch().size()<<endl;
  }
  cout<<"MissedPoints Texture [done]"<<endl;
}

void PCCDecoder::generateMPsGeometryfromImage(PCCContext& context, PCCFrameContext& frame, PCCGroupOfFrames& reconstructs, size_t frameIndex) {
  auto& videoMPsGeometry = context.getVideoMPsGeometry();
  auto &image = videoMPsGeometry.getFrame(frameIndex);
  auto& missedPointsPatch = frame.getMissedPointsPatch();
  size_t width  = image.getWidth();
  bool losslessGeo444 = frame.getLosslessGeo444();
  size_t numofMPs = missedPointsPatch.getMPnumber();
  if(losslessGeo444) {
    missedPointsPatch.resize(numofMPs);
  } else {
    missedPointsPatch.resize(numofMPs*3);
  }

  for(size_t i=0; i<numofMPs; i++) {
   if (frame.getLosslessGeo444())  {
      missedPointsPatch.x[i] = image.getValue(0, i%width, i/width);
      missedPointsPatch.y[i] = image.getValue(0, i%width, i/width);
      missedPointsPatch.z[i] = image.getValue(0, i%width, i/width);
    } else {
      missedPointsPatch.x[i]               = image.getValue(0, i%width, i/width);
      missedPointsPatch.x[numofMPs + i]    = image.getValue(0, (numofMPs + i)%width, (numofMPs + i)/width);
      missedPointsPatch.x[2 * numofMPs + i]= image.getValue(0, (2*numofMPs + i)%width, (2*numofMPs + i)/width);
    }
  }
}

void PCCDecoder::generateMPsTexturefromImage(PCCContext& context, PCCFrameContext& frame, PCCGroupOfFrames& reconstructs,size_t frameIndex) {
  auto& videoMPsTexture = context.getVideoMPsTexture();
  auto &image = videoMPsTexture.getFrame(frameIndex);
  auto& missedPointsPatch = frame.getMissedPointsPatch();
  size_t width  = image.getWidth();
  size_t numofMPs = missedPointsPatch.getMPnumbercolor();
  for(size_t i=0; i<numofMPs; i++)  {
    assert(i/width<image.getHeight());
    missedPointsPatch.r[i] = image.getValue(0, i%width, i/width);
    missedPointsPatch.g[i] = image.getValue(1, i%width, i/width);
    missedPointsPatch.b[i] = image.getValue(2, i%width, i/width);
  }
}

void PCCDecoder::decompressOccupancyMap( PCCContext &context, PCCBitstream& bitstream ){

  size_t sizeFrames = context.getFrames().size();
  PCCFrameContext preFrame = context.getFrames()[0];
  for( int i = 0; i < sizeFrames; i++ ){
    PCCFrameContext &frame = context.getFrames()[i];
    frame.getWidth () = width_;
    frame.getHeight() = height_;
    auto &frameLevelMetadataEnabledFlags = context.getGOFLevelMetadata().getLowerLevelMetadataEnabledFlags();
    frame.getFrameLevelMetadata().getMetadataEnabledFlags() = frameLevelMetadataEnabledFlags;
    decompressOccupancyMap( context, frame, bitstream, preFrame, i );

    if(losslessGeo_ && !context.getUseMissedPointsSeparateVideo()) {
      if(!context.getUseMissedPointsSeparateVideo()) {
        auto&  patches = frame.getPatches();
        auto& missedPointsPatch = frame.getMissedPointsPatch();
        if (losslessGeo_) {
          const size_t patchIndex = patches.size();
          PCCPatch &dummyPatch = patches[patchIndex - 1];
          missedPointsPatch.u0 = dummyPatch.getU0();
          missedPointsPatch.v0 = dummyPatch.getV0();
          missedPointsPatch.sizeU0 = dummyPatch.getSizeU0();
          missedPointsPatch.sizeV0 = dummyPatch.getSizeV0();
          missedPointsPatch.occupancyResolution = dummyPatch.getOccupancyResolution();
          patches.pop_back();
        }
      }
    }
    preFrame = frame;
  }
}

void PCCDecoder::decompressPatchMetaDataM42195(PCCFrameContext& frame, PCCFrameContext& preFrame, PCCBitstream &bitstream ,
                                               o3dgc::Arithmetic_Codec &arithmeticDecoder, o3dgc::Static_Bit_Model &bModel0,
                                               uint32_t &compressedBitstreamSize, size_t occupancyPrecision, uint8_t enable_flexible_patch_flag) {
  auto&  patches = frame.getPatches();
  auto&  prePatches = preFrame.getPatches();
  size_t patchCount = patches.size();
  uint8_t bitCount[5];
  uint8_t F = 0,A[5];
  size_t topNmax[5] = {0,0,0,0,0};
  bitstream.read<uint8_t>( F );
  for (size_t i = 0; i < 5; i++) {
    A[4 - i] = F&1;
    F = F>>1;
  }
  F = F&1;
  for (size_t i = 0; i < 5; i++) {
    if(A[i])   bitstream.read<uint8_t>( bitCount[i] );
  }

  bitstream.read<uint32_t>(  compressedBitstreamSize );
  assert(compressedBitstreamSize + bitstream.size() <= bitstream.capacity());
  arithmeticDecoder.set_buffer(uint32_t(bitstream.capacity() - bitstream.size()),
                               bitstream.buffer() + bitstream.size());

  bool bBinArithCoding = binArithCoding_ && (!losslessGeo_) &&
      (occupancyResolution_ == 16) && (occupancyPrecision == 4);

  bool useOneLayermode = oneLayerMode_ || singleLayerPixelInterleaving_;

  arithmeticDecoder.start_decoder();
  o3dgc::Adaptive_Bit_Model bModelPatchIndex, bModelU0, bModelV0, bModelU1, bModelV1, bModelD1,bModelIntSizeU0,bModelIntSizeV0;
  o3dgc::Adaptive_Bit_Model bModelSizeU0, bModelSizeV0, bModelAbsoluteD1;
  o3dgc::Adaptive_Bit_Model orientationModel2;
  o3dgc::Adaptive_Data_Model orientationModel(4);
  o3dgc::Adaptive_Data_Model orientationPatchModel(NumPatchOrientations - 1 + 2);
  o3dgc::Adaptive_Bit_Model orientationPatchFlagModel2;
  int64_t prevSizeU0 = 0;
  int64_t prevSizeV0 = 0;
  uint32_t numMatchedPatches;
  const uint8_t bitMatchedPatchCount = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(patchCount)));
  numMatchedPatches = DecodeUInt32(bitMatchedPatchCount, arithmeticDecoder, bModel0);

  int64_t predIndex = 0;
  for (size_t patchIndex = 0; patchIndex < numMatchedPatches; ++patchIndex) {
    auto &patch = patches[patchIndex];
    patch.getOccupancyResolution() = occupancyResolution_;
    int64_t delta_index = o3dgc::UIntToInt(arithmeticDecoder.ExpGolombDecode(0, bModel0, bModelPatchIndex));
    patch.setBestMatchIdx() = (size_t)(delta_index + predIndex);
    predIndex += (delta_index+1);

    const auto &prePatch = prePatches[patch.getBestMatchIdx()];
    const int64_t delta_U0 = o3dgc::UIntToInt(arithmeticDecoder.ExpGolombDecode(0, bModel0, bModelU0));
    const int64_t delta_V0 = o3dgc::UIntToInt(arithmeticDecoder.ExpGolombDecode(0, bModel0, bModelV0));
    const int64_t delta_U1 = o3dgc::UIntToInt(arithmeticDecoder.ExpGolombDecode(0, bModel0, bModelU1));
    const int64_t delta_V1 = o3dgc::UIntToInt(arithmeticDecoder.ExpGolombDecode(0, bModel0, bModelV1));
    const int64_t delta_D1 = o3dgc::UIntToInt(arithmeticDecoder.ExpGolombDecode(0, bModel0, bModelD1));
    const int64_t deltaSizeU0 = o3dgc::UIntToInt(arithmeticDecoder.ExpGolombDecode(0, bModel0, bModelIntSizeU0));
    const int64_t deltaSizeV0 =  o3dgc::UIntToInt(arithmeticDecoder.ExpGolombDecode(0, bModel0, bModelIntSizeV0));

    if ( sixDirectionMode_ && absoluteD1_ ) {
      patch.getProjectionMode() = arithmeticDecoder.decode(bModel0);
    } else {
      patch.getProjectionMode() = 0;
    }

    patch.getU0() = delta_U0 + prePatch.getU0();
    patch.getV0() = delta_V0 + prePatch.getV0();
    patch.getPatchOrientation() = prePatch.getPatchOrientation();
    patch.getU1() = delta_U1 + prePatch.getU1();
    patch.getV1() = delta_V1 + prePatch.getV1();
    patch.getD1() = delta_D1 + prePatch.getD1();
    patch.getSizeU0() = deltaSizeU0 + prePatch.getSizeU0();
    patch.getSizeV0() = deltaSizeV0 + prePatch.getSizeV0();

    //get maximum
    topNmax[0] = topNmax[0] < patch.getU0() ? patch.getU0() : topNmax[0];
    topNmax[1] = topNmax[1] < patch.getV0() ? patch.getV0() : topNmax[1];
    topNmax[2] = topNmax[2] < patch.getU1() ? patch.getU1() : topNmax[2];
    topNmax[3] = topNmax[3] < patch.getV1() ? patch.getV1() : topNmax[3];
    topNmax[4] = topNmax[4] < patch.getD1() ? patch.getD1() : topNmax[4];

    prevSizeU0 = patch.getSizeU0();
    prevSizeV0 = patch.getSizeV0();

    patch.getNormalAxis() = prePatch.getNormalAxis();
    patch.getTangentAxis() = prePatch.getTangentAxis();
    patch.getBitangentAxis() = prePatch.getBitangentAxis();
    if (printDetailedInfo) {
      patch.print_decoder();
    }
  }

  //Get Bitcount.
  for (int i = 0; i < 5; i++) {
    if (A[i] == 0)  bitCount[i] = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(topNmax[i] + 1)));
  }

  for (size_t patchIndex = numMatchedPatches; patchIndex < patchCount; ++patchIndex) {
    auto &patch = patches[patchIndex];
    patch.getOccupancyResolution() = occupancyResolution_;
    patch.getU0() = DecodeUInt32(bitCount[0], arithmeticDecoder, bModel0);
    patch.getV0() = DecodeUInt32(bitCount[1], arithmeticDecoder, bModel0);
    if (enable_flexible_patch_flag) {
      bool flexible_patch_present_flag = arithmeticDecoder.decode(orientationPatchFlagModel2);
      if (flexible_patch_present_flag) {
        patch.getPatchOrientation() = arithmeticDecoder.decode(orientationPatchModel) + 1;
      } else {
        patch.getPatchOrientation() = 0;
      }
    } else {
      patch.getPatchOrientation() = 0;
    }
    patch.getU1() = DecodeUInt32(bitCount[2], arithmeticDecoder, bModel0);
    patch.getV1() = DecodeUInt32(bitCount[3], arithmeticDecoder, bModel0);
    patch.getD1() = DecodeUInt32(bitCount[4], arithmeticDecoder, bModel0);

    if (sixDirectionMode_ && absoluteD1_ ) {
      patch.getProjectionMode() = arithmeticDecoder.decode(bModel0);
    } else {
      patch.getProjectionMode() = 0;
    }
    const int64_t deltaSizeU0 = o3dgc::UIntToInt(arithmeticDecoder.ExpGolombDecode(0, bModel0, bModelSizeU0));
    const int64_t deltaSizeV0 = o3dgc::UIntToInt(arithmeticDecoder.ExpGolombDecode(0, bModel0, bModelSizeV0));

    patch.getSizeU0() = prevSizeU0 + deltaSizeU0;
    patch.getSizeV0() = prevSizeV0 + deltaSizeV0;

    prevSizeU0 = patch.getSizeU0();
    prevSizeV0 = patch.getSizeV0();

    if (bBinArithCoding) {
      size_t bit0 = arithmeticDecoder.decode(orientationModel2);
      if (bit0 == 0) {  // 0
        patch.getNormalAxis() = 0;
      } else {
        size_t bit1 = arithmeticDecoder.decode(bModel0);
        if (bit1 == 0) { // 10
          patch.getNormalAxis() = 1;
        } else { // 11
          patch.getNormalAxis() = 2;
        }
      }
    } else {
      patch.getNormalAxis() = arithmeticDecoder.decode(orientationModel);
    }
    if (patch.getNormalAxis() == 0) {
      patch.getTangentAxis() = 2;
      patch.getBitangentAxis() = 1;
    } else if (patch.getNormalAxis() == 1) {
      patch.getTangentAxis() = 2;
      patch.getBitangentAxis() = 0;
    } else {
      patch.getTangentAxis() = 0;
      patch.getBitangentAxis() = 1;
    }
    auto &patchLevelMetadataEnabledFlags = frame.getFrameLevelMetadata().getLowerLevelMetadataEnabledFlags();
    auto &patchLevelMetadata = patch.getPatchLevelMetadata();
    patchLevelMetadata.getMetadataEnabledFlags() = patchLevelMetadataEnabledFlags;
    decompressMetadata(patchLevelMetadata, arithmeticDecoder);
    if (printDetailedInfo) {
      patch.print_decoder();
    }
  }
}


void PCCDecoder::decompressOccupancyMap( PCCContext &context, PCCFrameContext& frame, PCCBitstream &bitstream, 
                                         PCCFrameContext& preFrame, size_t frameIndex ) {
  uint32_t patchCount = 0;
  auto&  patches = frame.getPatches();
  bitstream.read<uint32_t>( patchCount );
  patches.resize( patchCount );

  o3dgc::Adaptive_Bit_Model  interpolateModel;
  o3dgc::Adaptive_Bit_Model  fillingModel;
  o3dgc::Adaptive_Data_Model minD1Model(3);
  o3dgc::Adaptive_Data_Model neighborModel(3);

  if( useOccupancyMapVideo_ ) {
    PCCImageOccupancyMap &videoFrame = context.getVideoOccupancyMap().getFrame( frame.getIndex() );
    GenerateOccupancyMapFromVideoFrame( occupancyResolution_, occupancyPrecision_, width_, height_,
                                        frame.getOccupancyMap(), videoFrame );
  }

  size_t maxCandidateCount = 0;
  {
    uint8_t count = 0;
    bitstream.read<uint8_t>( count );
    maxCandidateCount = count;
  }
  uint8_t frameProjectionMode = 0, surfaceThickness = 4;
  if (!absoluteD1_) {
    bitstream.read<uint8_t>(surfaceThickness);
    bitstream.read<uint8_t>(frameProjectionMode);
  }
  frame.setSurfaceThickness( surfaceThickness );

  o3dgc::Arithmetic_Codec arithmeticDecoder;
  o3dgc::Static_Bit_Model bModel0;
  uint32_t compressedBitstreamSize;

  bool bBinArithCoding = binArithCoding_ && (!losslessGeo_) &&
      (occupancyResolution_ == 16) && (occupancyPrecision_ == 4);

  uint8_t enable_flexible_patch_flag;
  bitstream.read<uint8_t>(enable_flexible_patch_flag);

  if((frameIndex == 0)||(!deltaCoding_)) {
    uint8_t bitCountU0 = 0;
    uint8_t bitCountV0 = 0;
    uint8_t bitCountU1 = 0;
    uint8_t bitCountV1 = 0;
    uint8_t bitCountD1 = 0;
    uint8_t bitCountLod = 0;
    bitstream.read<uint8_t>( bitCountU0 );
    bitstream.read<uint8_t>( bitCountV0 );
    bitstream.read<uint8_t>( bitCountU1 );
    bitstream.read<uint8_t>( bitCountV1 );
    bitstream.read<uint8_t>( bitCountD1 );
    bitstream.read<uint8_t>( bitCountLod);
    bitstream.read<uint32_t>(  compressedBitstreamSize );

    assert(compressedBitstreamSize + bitstream.size() <= bitstream.capacity());
    arithmeticDecoder.set_buffer(uint32_t(bitstream.capacity() - bitstream.size()),
                                 bitstream.buffer() + bitstream.size());
    arithmeticDecoder.start_decoder();
    decompressMetadata(frame.getFrameLevelMetadata(), arithmeticDecoder);
    o3dgc::Adaptive_Bit_Model bModelSizeU0, bModelSizeV0, bModelAbsoluteD1;
    o3dgc::Adaptive_Bit_Model orientationModel2;
    o3dgc::Adaptive_Data_Model orientationModel(4);
    o3dgc::Adaptive_Data_Model orientationPatchModel(NumPatchOrientations - 1 + 2);
    o3dgc::Adaptive_Bit_Model orientationPatchFlagModel2;
    int64_t prevSizeU0 = 0;
    int64_t prevSizeV0 = 0;
    bool useOneLayermode = oneLayerMode_ || singleLayerPixelInterleaving_;
    for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
      auto &patch = patches[patchIndex];
      patch.getOccupancyResolution() = occupancyResolution_;

      patch.getU0() = DecodeUInt32(bitCountU0, arithmeticDecoder, bModel0);
      patch.getV0() = DecodeUInt32(bitCountV0, arithmeticDecoder, bModel0);
      if (enable_flexible_patch_flag) {
        bool flexible_patch_present_flag = arithmeticDecoder.decode(orientationPatchFlagModel2);
        if (flexible_patch_present_flag) {
          patch.getPatchOrientation() = arithmeticDecoder.decode(orientationPatchModel)+1;
        } else {
          patch.getPatchOrientation() = 0;
        }
      } else {
        patch.getPatchOrientation() = 0;
      }
      patch.getU1() = DecodeUInt32(bitCountU1, arithmeticDecoder, bModel0);
      patch.getV1() = DecodeUInt32(bitCountV1, arithmeticDecoder, bModel0);
      patch.getD1() = DecodeUInt32(bitCountD1, arithmeticDecoder, bModel0);
      patch.getLod() = DecodeUInt32(bitCountLod, arithmeticDecoder, bModel0);

      if (!absoluteD1_) {
        patch.getFrameProjectionMode() = frameProjectionMode;
        if (patch.getFrameProjectionMode() == 0){
          patch.getProjectionMode() = 0;
        } else if (patch.getFrameProjectionMode() == 1){
          patch.getProjectionMode() = 1;
        } else if (patch.getFrameProjectionMode() == 2) {
          patch.getProjectionMode() = 0;
          const uint8_t bitCountProjDir = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(2 + 1)));
          patch.getProjectionMode() = DecodeUInt32(bitCountProjDir, arithmeticDecoder, bModel0);
          std::cout << "patch.getProjectionMode()= " << patch.getProjectionMode() << std::endl;
        } else {
          std::cout << "This frameProjectionMode doesn't exist!" << std::endl;
        }
        std::cout << "(frameProjMode, projMode)= (" << patch.getFrameProjectionMode() << ", " << patch.getProjectionMode() << ")" << std::endl;
      } else {
        patch.getFrameProjectionMode() = 0;
        if (sixDirectionMode_) {
          patch.getProjectionMode() = arithmeticDecoder.decode(bModel0);
        } else {
          patch.getProjectionMode() = 0;
        }
      }
      const int64_t deltaSizeU0 = o3dgc::UIntToInt(arithmeticDecoder.ExpGolombDecode(0, bModel0, bModelSizeU0));
      const int64_t deltaSizeV0 = o3dgc::UIntToInt(arithmeticDecoder.ExpGolombDecode(0, bModel0, bModelSizeV0));

      patch.getSizeU0() = prevSizeU0 + deltaSizeU0;
      patch.getSizeV0() = prevSizeV0 + deltaSizeV0;

      prevSizeU0 = patch.getSizeU0();
      prevSizeV0 = patch.getSizeV0();

      if (bBinArithCoding) {
        size_t bit0 = arithmeticDecoder.decode(orientationModel2);
        if (bit0 == 0) {  // 0
          patch.getNormalAxis() = 0;
        } else {
          size_t bit1 = arithmeticDecoder.decode(bModel0);
          if (bit1 == 0) { // 10
            patch.getNormalAxis() = 1;
          } else { // 11
            patch.getNormalAxis() = 2;
          }
        }
      } else {
        patch.getNormalAxis() = arithmeticDecoder.decode(orientationModel);
      }

      if (patch.getNormalAxis() == 0) {
        patch.getTangentAxis() = 2;
        patch.getBitangentAxis() = 1;
      } else if (patch.getNormalAxis() == 1) {
        patch.getTangentAxis() = 2;
        patch.getBitangentAxis() = 0;
      } else {
        patch.getTangentAxis() = 0;
        patch.getBitangentAxis() = 1;
      }
      auto &patchLevelMetadataEnabledFlags = frame.getFrameLevelMetadata().getLowerLevelMetadataEnabledFlags();
      auto &patchLevelMetadata = patch.getPatchLevelMetadata();
      patchLevelMetadata.getMetadataEnabledFlags() = patchLevelMetadataEnabledFlags;
      decompressMetadata(patchLevelMetadata, arithmeticDecoder);

      if (printDetailedInfo) {
        patch.print_decoder();
      }
    }
  } else {
    decompressPatchMetaDataM42195( frame, preFrame, bitstream, arithmeticDecoder, bModel0, 
                                   compressedBitstreamSize, occupancyPrecision_, enable_flexible_patch_flag ) ;
  }

  const size_t blockToPatchWidth = width_ / occupancyResolution_;
  const size_t blockToPatchHeight = height_ / occupancyResolution_;
  const size_t blockCount = blockToPatchWidth * blockToPatchHeight;

  std::vector<std::vector<size_t>> candidatePatches;
  candidatePatches.resize(blockCount);
  for (int64_t patchIndex = patchCount - 1; patchIndex >= 0; --patchIndex) {
    // add actual patches based on their bounding box
    auto &patch = patches[patchIndex];
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        candidatePatches[patch.patchBlock2CanvasBlock(u0, v0, blockToPatchWidth, blockToPatchHeight)].push_back(patchIndex + 1);
      }
    }
  }
  if(! useOccupancyMapVideo_ ) {
    for (auto &candidatePatch : candidatePatches) {  // add empty as potential candidate
      candidatePatch.push_back(0);
    }
  }
  auto& blockToPatch = frame.getBlockToPatch();
  blockToPatch.resize(0);
  blockToPatch.resize(blockCount, 0);

  auto& interpolateMap = frame.getInterpolate();
  auto& fillingMap     = frame.getFilling();
  auto& minD1Map       = frame.getMinD1();
  auto& neighborMap    = frame.getNeighbor();
  interpolateMap.resize( blockCount, 0 );
  fillingMap    .resize( blockCount, 0 );
  minD1Map      .resize( blockCount, 0 );
  neighborMap   .resize( blockCount, 1 );

  o3dgc::Adaptive_Bit_Model candidateIndexModelBit[4];
  o3dgc::Adaptive_Data_Model candidateIndexModel(uint32_t(maxCandidateCount + 2));
  const uint32_t bitCountPatchIndex = PCCGetNumberOfBitsInFixedLengthRepresentation(patchCount + 1);
  if( useOccupancyMapVideo_ ) {
    auto& occupancyMap = frame.getOccupancyMap();
    for (size_t p = 0; p < blockCount; ++p) {
      blockToPatch[p] = 0;
      const auto &candidates = candidatePatches[p];
      if (candidates.size() > 0) {
        bool empty = true;
        size_t positionU = (p%blockToPatchWidth) * occupancyResolution_;
        size_t positionV = (p/blockToPatchWidth) * occupancyResolution_;
        for (size_t v=positionV; v<positionV+occupancyResolution_ && empty; ++v) {
          for (size_t u=positionU; u<positionU+occupancyResolution_ && empty; ++u) {
            if (occupancyMap[u+v*width_]) {
              empty = false;
              if (candidates.size() == 1) {
                blockToPatch[p] = candidates[0];
              } else {
                size_t candidateIndex;
                if (bBinArithCoding) {
                  size_t bit0 = arithmeticDecoder.decode(candidateIndexModelBit[0]);
                  if (bit0 == 0) {
                    candidateIndex = 0; // Codeword: 0
                  } else {
                    size_t bit1 = arithmeticDecoder.decode(candidateIndexModelBit[1]);
                    if (bit1 == 0) {
                      candidateIndex = 1; // Codeword 10
                    } else {
                      size_t bit2 = arithmeticDecoder.decode(candidateIndexModelBit[2]);
                      if (bit2 == 0) {
                        candidateIndex = 2; // Codeword 110
                      } else {
                        size_t bit3 = arithmeticDecoder.decode(candidateIndexModelBit[3]);
                        if (bit3 == 0) {
                          candidateIndex = 3; // Codeword 1110
                        } else {
                          candidateIndex = 4; // Codeword 11110
                        }
                      }
                    }
                  }
                } else {
                  candidateIndex = arithmeticDecoder.decode(candidateIndexModel);
                }

                if (candidateIndex == maxCandidateCount) {
                  blockToPatch[p] = DecodeUInt32(bitCountPatchIndex, arithmeticDecoder, bModel0);
                } else {
                  blockToPatch[p] = candidates[candidateIndex];
                }
              }
              interpolateMap[ p ] = 0;
              fillingMap    [ p ] = 0;
              minD1Map      [ p ] = 0;
              neighborMap   [ p ] = 1;
              if( blockToPatch[p] > 0 ) {
                if( absoluteD1_ && oneLayerMode_ ) {
                  interpolateMap[ p ] = arithmeticDecoder.decode( interpolateModel);
                  if( interpolateMap[ p ] > 0 ){
                    neighborMap[ p ] = arithmeticDecoder.decode( neighborModel ) + 1;
                  }
                  minD1Map[ p ] = arithmeticDecoder.decode( minD1Model);
                  if( minD1Map[ p ] > 1 || interpolateMap[ p ] > 0 ) {
                    fillingMap[ p ] = arithmeticDecoder.decode( fillingModel);
                  }
                }
              }
            }
          }
        }
      }
    }
  } else {
    for (size_t p = 0; p < blockCount; ++p) {
      const auto &candidates = candidatePatches[p];
      if (candidates.size() == 1) {
        blockToPatch[p] = candidates[0];
      } else {
        size_t candidateIndex;
        if (bBinArithCoding) {
          size_t bit0 = arithmeticDecoder.decode(candidateIndexModelBit[0]);
          if (bit0 == 0) {
            candidateIndex = 0; // Codeword: 0
          } else {
            size_t bit1 = arithmeticDecoder.decode(candidateIndexModelBit[1]);
            if (bit1 == 0) {
              candidateIndex = 1; // Codeword 10
            } else {
              size_t bit2 = arithmeticDecoder.decode(candidateIndexModelBit[2]);
              if (bit2 == 0) {
                candidateIndex = 2; // Codeword 110
              } else {
                size_t bit3 = arithmeticDecoder.decode(candidateIndexModelBit[3]);
                if (bit3 == 0) {
                  candidateIndex = 3; // Codeword 1110
                } else {
                  candidateIndex = 4; // Codeword 11110
                }
              }
            }
          }
        } else {
          candidateIndex = arithmeticDecoder.decode(candidateIndexModel);
        }
        if (candidateIndex == maxCandidateCount) {
          blockToPatch[p] = DecodeUInt32(bitCountPatchIndex, arithmeticDecoder, bModel0);
        } else {
          blockToPatch[p] = candidates[candidateIndex];
        }
      } //size=1
      interpolateMap[ p ] = 0;
      fillingMap    [ p ] = 0;
      minD1Map      [ p ] = 0;
      neighborMap   [ p ] = 1;
      if( blockToPatch[p] > 0 ) {
        if( absoluteD1_ && oneLayerMode_  ) {
          interpolateMap[ p ] = arithmeticDecoder.decode( interpolateModel);
          if( interpolateMap[ p ] > 0 ){
            neighborMap[ p ] = arithmeticDecoder.decode( neighborModel ) + 1;
          }
          minD1Map[ p ] = arithmeticDecoder.decode( minD1Model);
          if( minD1Map[ p ] > 1 || interpolateMap[ p ] > 0 ) {
            fillingMap[ p ] = arithmeticDecoder.decode( fillingModel);
          }
        }
      }
    } //blockCount
  }

  if( !useOccupancyMapVideo_ ) {
    const size_t blockSize0 = occupancyResolution_ / occupancyPrecision_;
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
          for (int64_t u1 = (std::max)(int64_t(0), k - int64_t(blockSize0));
              u1 < (std::min)(k, int64_t(blockSize0)); ++u1) {
            const size_t v1 = k - (u1 + 1);
            traversalOrder.push_back(std::make_pair(u1, v1));
          }
        }
      } else {
        for (int64_t k = 1; k < int64_t(2 * blockSize0); ++k) {
          for (int64_t u1 = (std::max)(int64_t(0), k - int64_t(blockSize0));
              u1 < (std::min)(k, int64_t(blockSize0)); ++u1) {
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
    static size_t runLengthInvTable[16] = { 0,  1,  2,  3,  7,  11,  14,  5,  13,  9,  6,  10,  12,  4,  8, 15 };
    o3dgc::Adaptive_Data_Model traversalOrderIndexModel(uint32_t(traversalOrderCount + 1));
    o3dgc::Adaptive_Data_Model runCountModel((uint32_t)(pointCount0));
    o3dgc::Adaptive_Data_Model runLengthModel((uint32_t)(pointCount0));

    std::vector<uint32_t> block0;
    std::vector<size_t> bestRuns;
    std::vector<size_t> runs;
    block0.resize(pointCount0);
    auto& occupancyMap = frame.getOccupancyMap();
    occupancyMap.resize(width_ * height_, 0);
    for (size_t v0 = 0; v0 < blockToPatchHeight; ++v0) {
      for (size_t u0 = 0; u0 < blockToPatchWidth; ++u0) {
        const size_t patchIndex = blockToPatch[v0 * blockToPatchWidth + u0];
        if (patchIndex) {
          const bool isFull = arithmeticDecoder.decode(fullBlockModel) != 0;
          if (isFull) {
            for (auto &occupancy : block0) {
              occupancy = true;
            }
          } else {
            size_t bestTraversalOrderIndex;
            if (bBinArithCoding) {
              size_t bit1 = arithmeticDecoder.decode(traversalOrderIndexModel_Bit1);
              size_t bit0 = arithmeticDecoder.decode(traversalOrderIndexModel_Bit0);
              bestTraversalOrderIndex = (bit1 << 1) + bit0;
            } else {
              bestTraversalOrderIndex = arithmeticDecoder.decode(traversalOrderIndexModel);
            }
            const auto &traversalOrder = traversalOrders[bestTraversalOrderIndex];
            int64_t runCountMinusTwo;
            if (bBinArithCoding) {
              runCountMinusTwo = arithmeticDecoder.ExpGolombDecode(0, bModel0, runCountModel2);
            } else {
              runCountMinusTwo = arithmeticDecoder.decode(runCountModel);
            }

            const size_t runCountMinusOne = runCountMinusTwo + 1;
            size_t i = 0;
            bool occupancy = arithmeticDecoder.decode(occupancyModel) != 0;
            for (size_t r = 0; r < runCountMinusOne; ++r) {
              size_t runLength;
              if (bBinArithCoding) {
                size_t bit3 = arithmeticDecoder.decode(runLengthModel2[3]);
                size_t bit2 = arithmeticDecoder.decode(runLengthModel2[2]);
                size_t bit1 = arithmeticDecoder.decode(runLengthModel2[1]);
                size_t bit0 = arithmeticDecoder.decode(runLengthModel2[0]);
                const size_t runLengthIdx = (bit3 << 3) + (bit2 << 2) + (bit1 << 1) + bit0;
                runLength = runLengthInvTable[runLengthIdx];
              } else {
                runLength = arithmeticDecoder.decode(runLengthModel);
              }
              for (size_t j = 0; j <= runLength; ++j) {
                const auto &location = traversalOrder[i++];
                block0[location.second * blockSize0 + location.first] = occupancy;
              }
              occupancy = !occupancy;
            }
            for (size_t j = i; j < pointCount0; ++j) {
              const auto &location = traversalOrder[j];
              block0[location.second * blockSize0 + location.first] = occupancy;
            }
          }
          for (size_t v1 = 0; v1 < blockSize0; ++v1) {
            const size_t v2 = v0 * occupancyResolution_ + v1 * occupancyPrecision_;
            for (size_t u1 = 0; u1 < blockSize0; ++u1) {
              const size_t u2 = u0 * occupancyResolution_ + u1 * occupancyPrecision_;
              const bool occupancy = block0[v1 * blockSize0 + u1] != 0;
              for (size_t v3 = 0; v3 < occupancyPrecision_; ++v3) {
                for (size_t u3 = 0; u3 < occupancyPrecision_; ++u3) {
                  occupancyMap[(v2 + v3) * width_ + u2 + u3] = occupancy;
                }
              }
            }
          }
        }
      }
    }
  }
  arithmeticDecoder.stop_decoder();
  bitstream += (uint64_t)compressedBitstreamSize;
}

void PCCDecoder::GenerateOccupancyMapFromVideoFrame(size_t occupancyResolution, size_t occupancyPrecision,
                                                    size_t width, size_t height,
                                                    std::vector<uint32_t> &occupancyMap,
                                                    const PCCImageOccupancyMap &videoFrame) {
  occupancyMap.resize(width * height, 0);
  for (size_t v=0; v<height; ++v) {
    for (size_t u=0; u<width; ++u) {
      occupancyMap[v*width+u] = videoFrame.getValue(0, u/occupancyPrecision, v/occupancyPrecision);
    }
  }
}


