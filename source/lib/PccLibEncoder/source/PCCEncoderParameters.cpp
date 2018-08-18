
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
#include "PCCEncoderParameters.h"

using namespace pcc ;


PCCEncoderParameters::PCCEncoderParameters() {
  uncompressedDataPath_                 = {};
  compressedStreamPath_                 = {};
  reconstructedDataPath_                = {};
  configurationFolder_                  = {};
  uncompressedDataFolder_               = {};

  startFrameNumber_                     = 0;

  frameCount_                           = 300;
  groupOfFramesSize_                    = 32;

  colorTransform_                       = COLOR_TRANSFORM_RGB_TO_YCBCR;
  colorSpaceConversionPath_             = {};

  colorSpaceConversionConfig_           = {};
  inverseColorSpaceConversionConfig_    = {};
  nnNormalEstimation_                   = 16;
  maxNNCountRefineSegmentation_         = 256;
  iterationCountRefineSegmentation_     = 100;

  occupancyResolution_                  = 16;
  minPointCountPerCCPatchSegmentation_  = 16;
  maxNNCountPatchSegmentation_          = 16;
  surfaceThickness_                     = 4;
  maxAllowedDepth_                      = 255;
  maxAllowedDist2MissedPointsDetection_ = 9.0;
  maxAllowedDist2MissedPointsSelection_ = 1.0;
  lambdaRefineSegmentation_             = 3.0;


  minimumImageWidth_                    = 1280;
  minimumImageHeight_                   = 1280;

  maxCandidateCount_                    = 4;
  occupancyPrecision_                   = 4;
  occupancyMapVideoEncoderConfig_       = {};
  occupancyMapQP_                       = 8;
  useOccupancyMapVideo_                 = true;

  neighborCountSmoothing_               = 4 * 16;
  radius2Smoothing_                     = 4.0 * 16;
  radius2BoundaryDetection_             = 4.0 * 16;
  thresholdSmoothing_                   = 64.0;
  bestColorSearchRange_                 = 0;

  videoEncoderPath_                     = {};
  videoEncoderOccupancyMapPath_         = {};
  geometryQP_                           = 28;
  textureQP_                            = 43;
  geometryConfig_                       = {};
  geometryD0Config_                     = {};
  geometryD1Config_                     = {};

  textureConfig_                        = {};

  losslessGeo_                          = false;
  losslessTexture_                      = false;
  noAttributes_                         = false;
  losslessGeo444_                       = false;

  useMissedPointsSeparateVideo_         = false; //(losslessGeo_|| losslessTexture_)? true : false;
  geometryMPConfig_                     = {};
  textureMPConfig_                      = {};
     useOccupancyMapVideo_              = (losslessGeo_|| losslessTexture_)? false : true;
  
  nbThread_                             = 1;
  keepIntermediateFiles_                = false;
  projectionMode_                       = 0;

  absoluteD1_                           = true;

  constrainedPack_                      = true;
  binArithCoding_                       = true;

  thresholdColorSmoothing_              = 10.0;
  thresholdLocalEntropy_                = 4.5;
  radius2ColorSmoothing_                = 4.0 * 16;
  neighborCountColorSmoothing_          = 4 * 16;
  flagColorSmoothing_                   = false;

  groupDilation_    		                = true;
  textureDilationOffLossless_           = true;

  enhancedDeltaDepthCode_               = losslessGeo_ ? true : false; //EDD

  patchColorSubsampling_                = false;

  deltaCoding_                          = true;
}

PCCEncoderParameters::~PCCEncoderParameters() {
}

void PCCEncoderParameters::completePath(){
  if( !uncompressedDataFolder_.empty() ){
    if( !uncompressedDataPath_.empty() ){
      uncompressedDataPath_ = uncompressedDataFolder_ + uncompressedDataPath_;
    }
  }
  if( !configurationFolder_.empty() ){
    if( !geometryConfig_  .empty() ){
      geometryConfig_   = configurationFolder_ + geometryConfig_;
    }
    if( !geometryD0Config_.empty() ){
      geometryD0Config_ = configurationFolder_ + geometryD0Config_;
    }
    if( !geometryD1Config_.empty() ){
      geometryD1Config_ = configurationFolder_ + geometryD1Config_;
    }
    if( !textureConfig_   .empty() ){
      textureConfig_    = configurationFolder_ + textureConfig_;
    }
    if( !inverseColorSpaceConversionConfig_   .empty() ) {
      inverseColorSpaceConversionConfig_    = configurationFolder_ + inverseColorSpaceConversionConfig_;
    }
    if( !colorSpaceConversionConfig_   .empty() ) {
      colorSpaceConversionConfig_    = configurationFolder_ + colorSpaceConversionConfig_;
    }
    if( !occupancyMapVideoEncoderConfig_.empty() ){
      occupancyMapVideoEncoderConfig_ = configurationFolder_ + occupancyMapVideoEncoderConfig_;
    }
    if(useMissedPointsSeparateVideo_)
    {
      if( !geometryMPConfig_.empty())
      {
        geometryMPConfig_= configurationFolder_ + geometryMPConfig_;
      }
      
      if( !textureMPConfig_.empty())
      {
        textureMPConfig_= configurationFolder_ + textureMPConfig_;
      }
    }
  }
}

void PCCEncoderParameters::print(){

  std::cout << "+ Parameters" << std::endl;
  std::cout << "\t losslessGeo                            " << losslessGeo_                          << std::endl;
  std::cout << "\t losslessTexture                        " << losslessTexture_                      << std::endl;
  std::cout << "\t noAttributes                           " << noAttributes_                         << std::endl;
  std::cout << "\t losslessGeo444                         " << losslessGeo444_                       << std::endl;
  std::cout << "\t enhancedDeltaDepthCode                 " << enhancedDeltaDepthCode_               << std::endl; //EDD
  std::cout << "\t useMissedPointsSeparateVideo           " << useMissedPointsSeparateVideo_                 <<std::endl;
  std::cout << "\t uncompressedDataPath                   " << uncompressedDataPath_                 << std::endl;
  std::cout << "\t compressedStreamPath                   " << compressedStreamPath_                 << std::endl;
  std::cout << "\t reconstructedDataPath                  " << reconstructedDataPath_                << std::endl;
  std::cout << "\t frameCount                             " << frameCount_                           << std::endl;
  std::cout << "\t startFrameNumber                       " << startFrameNumber_                     << std::endl;
  std::cout << "\t groupOfFramesSize                      " << groupOfFramesSize_                    << std::endl;
  std::cout << "\t colorTransform                         " << colorTransform_                       << std::endl;
  std::cout << "\t nbThread                               " << nbThread_                             << std::endl;
  std::cout << "\t keepIntermediateFiles                  " << keepIntermediateFiles_                << std::endl;
  std::cout << "\t absoluteD1                             " << absoluteD1_                           << std::endl;
  std::cout << "\t constrainedPack                        " << constrainedPack_                      << std::endl;
  std::cout << "\t binArithCoding                         " << binArithCoding_                       << std::endl;
  std::cout << "\t deltaCoding                            " << deltaCoding_                          << std::endl;
  std::cout << "\t segmentation" << std::endl;
  std::cout << "\t   nnNormalEstimation                   " << nnNormalEstimation_                   << std::endl;
  std::cout << "\t   maxNNCountRefineSegmentation         " << maxNNCountRefineSegmentation_         << std::endl;
  std::cout << "\t   iterationCountRefineSegmentation     " << iterationCountRefineSegmentation_     << std::endl;
  std::cout << "\t   occupancyResolution                  " << occupancyResolution_                  << std::endl;
  std::cout << "\t   minPointCountPerCCPatchSegmentation  " << minPointCountPerCCPatchSegmentation_  << std::endl;
  std::cout << "\t   maxNNCountPatchSegmentation          " << maxNNCountPatchSegmentation_          << std::endl;
  std::cout << "\t   surfaceThickness                     " << surfaceThickness_                     << std::endl;
  std::cout << "\t   maxAllowedDist2MissedPointsDetection " << maxAllowedDist2MissedPointsDetection_ << std::endl;
  std::cout << "\t   maxAllowedDist2MissedPointsSelection " << maxAllowedDist2MissedPointsSelection_ << std::endl;
  std::cout << "\t   lambdaRefineSegmentation             " << lambdaRefineSegmentation_             << std::endl;
  std::cout << "\t   maxAllowedDepth                      " << maxAllowedDepth_                      << std::endl;
  std::cout << "\t packing" << std::endl;
  std::cout << "\t   minimumImageWidth                    " << minimumImageWidth_                    << std::endl;
  std::cout << "\t   minimumImageHeight                   " << minimumImageHeight_                   << std::endl;
  std::cout << "\t video encoding" << std::endl;
  std::cout << "\t   geometryQP                           " << geometryQP_                           << std::endl;
  std::cout << "\t   textureQP                            " << textureQP_                            << std::endl;
  std::cout << "\t   colorSpaceConversionPath             " << colorSpaceConversionPath_             << std::endl;
  std::cout << "\t   videoEncoderPath                     " << videoEncoderPath_                     << std::endl;
  std::cout << "\t   videoEncoderOccupancyMapPath         " << videoEncoderOccupancyMapPath_         << std::endl;
  if ( !absoluteD1_) {
    std::cout << "\t   geometryD0Config                     " << geometryD0Config_ << std::endl;
    std::cout << "\t   geometryD1Config                     " << geometryD1Config_ << std::endl;
  }else{
    std::cout << "\t   geometryConfig                       " << geometryConfig_                       << std::endl;
  }
  std::cout << "\t   textureConfig                        " << textureConfig_                        << std::endl;
  if(useMissedPointsSeparateVideo_)
  {
    if(losslessGeo_)
      std::cout <<"\t geometryMPConfig                        "               << geometryMPConfig_<<std::endl;
    if(losslessTexture_)
      std::cout<<"\t textureMPConfig                          "               << textureMPConfig_<<std::endl;
  }
  std::cout << "\t   colorSpaceConversionConfig           " << colorSpaceConversionConfig_           << std::endl;
  std::cout << "\t   inverseColorSpaceConversionConfig    " << inverseColorSpaceConversionConfig_    << std::endl;
  std::cout << "\t dilation" << std::endl;
  std::cout << "\t   GroupDilation                        " << groupDilation_ << std::endl;
  std::cout << "\t   textureDilationOffLossless           " << textureDilationOffLossless_ << std::endl;
  std::cout << "\t occupancy map encoding " << std::endl;
  std::cout << "\t   maxCandidateCount                    " << maxCandidateCount_                    << std::endl;
  std::cout << "\t   occupancyPrecision                   " << occupancyPrecision_                   << std::endl;
  std::cout << "\t   occupancyMapVideoEncoderConfig       " << occupancyMapVideoEncoderConfig_       << std::endl;
  std::cout << "\t   occupancyMapQP                       " << occupancyMapQP_                       << std::endl;
  std::cout << "\t   useOccupancyMapVideo                 " << useOccupancyMapVideo_                 <<std::endl;
  std::cout << "\t smoothing" << std::endl;
  std::cout << "\t   neighborCountSmoothing               " << neighborCountSmoothing_               << std::endl;
  std::cout << "\t   radius2Smoothing                     " << radius2Smoothing_                     << std::endl;
  std::cout << "\t   radius2BoundaryDetection             " << radius2BoundaryDetection_             << std::endl;
  std::cout << "\t   thresholdSmoothing                   " << thresholdSmoothing_                   << std::endl;
  std::cout << "\t color smoothing" << std::endl;
  std::cout << "\t   thresholdColorSmoothing              " << thresholdColorSmoothing_ << std::endl;
  std::cout << "\t   thresholdLocalEntropy                " << thresholdLocalEntropy_ << std::endl;
  std::cout << "\t   radius2ColorSmoothing                " << radius2ColorSmoothing_ << std::endl;
  std::cout << "\t   neighborCountColorSmoothing          " << neighborCountColorSmoothing_ << std::endl;
  std::cout << "\t   flagColorSmoothing                   " << flagColorSmoothing_ << std::endl;
  std::cout << "\t coloring" << std::endl;
  std::cout << "\t   bestColorSearchRange                 " << bestColorSearchRange_                 << std::endl;
  std::cout << "\t   projectionMode                       " << projectionMode_                       << std::endl;
  std::cout << "\t   patchColorSubsampling                " << patchColorSubsampling_                << std::endl;
  std::cout << std::endl;
}

bool PCCEncoderParameters::check(){
  bool ret = true;
  if (!colorSpaceConversionPath_.empty() &&
      !inverseColorSpaceConversionConfig_.empty() &&
      !colorSpaceConversionConfig_.empty()) {
    std::cout << "Info: Using external color space conversion" << std::endl;
    if (colorTransform_ != COLOR_TRANSFORM_NONE) {
      std::cerr << "Using external color space conversion requires colorTransform = "
                    "COLOR_TRANSFORM_NONE!\n";
      colorTransform_ = COLOR_TRANSFORM_NONE;
    }
    if( !exist( colorSpaceConversionPath_) ) {
      ret = false;
      std::cerr << "colorSpaceConversionPath not exist\n";
    }
    if( !exist( colorSpaceConversionConfig_) ) {
      ret = false;
      std::cerr << "colorSpaceConversionConfig not exist\n";
    }
    if( !exist( inverseColorSpaceConversionConfig_ )) {
      ret = false;
      std::cerr << "inverseColorSpaceConversionConfig not exist\n";
    }
  } else {
    std::cout << "Info: Using internal color space conversion" << std::endl;
    colorSpaceConversionPath_ = "";
    inverseColorSpaceConversionConfig_ = "";
    colorSpaceConversionConfig_ = "";
  }

  if( compressedStreamPath_.empty() ) {
    ret = false;
    std::cerr << "compressedStreamPath not set\n";
  }
  if( uncompressedDataPath_.empty() ) {
    ret = false;
    std::cerr << "uncompressedDataPath not set\n";
  }
  if( videoEncoderPath_.empty()     ) {
    ret = false;
    std::cerr << "videoEncoderPath not set\n";
  }
  if( !exist( videoEncoderPath_  )  ) {
    ret = false;
    std::cerr << "videoEncoderPath not exist\n";
  }
  if( videoEncoderOccupancyMapPath_.empty()     ) {
    ret = false;
    std::cerr << "videoEncoderOccupancyMapPath not set\n";
  }
  if( !exist( videoEncoderOccupancyMapPath_  )  ) {
    ret = false;
    std::cerr << "videoEncoderOccupancyMapPath not exist\n";
  }

  if (absoluteD1_ && (!geometryD0Config_.empty() || !geometryD1Config_.empty()))
  {
    ret = false;
    std::cerr << "When absoultD1 is true, geometryD0Config_ and geometryD1Config_ should be empty\n";
  }

  if (absoluteD1_ && geometryConfig_.empty())
  {
    ret = false;
    std::cerr << "When absoultD1 is true, geometryConfig_ should be non-empty\n";
  }

  if (!absoluteD1_ && (geometryD0Config_.empty() || geometryD1Config_.empty()))
  {
    ret = false;
    std::cerr << "When absoultD1 is false, geometryD0Config_ and geometryD1Config_ should be non-empty\n";
  }

  if (!absoluteD1_ && !geometryConfig_.empty())
  {
    ret = false;
    std::cerr << "When absoultD1 is false, geometryConfig_ should be empty\n";
  }

  if (!losslessGeo_ && enhancedDeltaDepthCode_) { //EDD
    enhancedDeltaDepthCode_ = false;
    std::cerr << "WARNING: enhancedDeltaDepthCode is only for lossless coding mode for now. Force enhancedDeltaDepthCode=FALSE.\n";
  }

  if (enhancedDeltaDepthCode_ && surfaceThickness_ == 1) { //EDD
    std::cerr << "WARNING: EDD code doesn't bring any gain when surfaceThickness==1. Please consider to increase the value of surfaceThickness.\n";
  }

  if(geometryMPConfig_.empty() )
  {
    geometryMPConfig_=geometryConfig_;
  }
  
  if(textureMPConfig_.empty() )
  {
    textureMPConfig_=textureConfig_;
  }

  if ( occupancyMapVideoEncoderConfig_.empty() ) {
    ret = false;
    std::cerr << "occupancyMapVideoEncoderConfig not set\n";
  }

  return ret;
}
