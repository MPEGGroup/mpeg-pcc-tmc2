
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

  neighborCountSmoothing_               = 4 * 16;
  radius2Smoothing_                     = 4.0 * 16;
  radius2BoundaryDetection_             = 4.0 * 16;
  thresholdSmoothing_                   = 64.0;
  bestColorSearchRange_                 = 0;

  videoEncoderPath_                     = {};
  geometryQP_                           = 28;
  textureQP_                            = 43;
  geometryConfig_                       = {};
  textureConfig_                        = {};

  losslessGeo_                          = false;
  losslessTexture_                      = false;
  noAttributes_                         = false;

  nbThread_                             = 1;
  keepIntermediateFiles_                = false;

  absoluteD1_                           = true;

  constrainedPack_                      = true;
  binArithCoding_                       = true;
}

PCCEncoderParameters::~PCCEncoderParameters() {
}

void PCCEncoderParameters::print(){

  std::cout << "+ Parameters" << std::endl;
  std::cout << "\t losslessGeo                            " << losslessGeo_                          << std::endl;
  std::cout << "\t losslessTexture                        " << losslessTexture_                      << std::endl;
  std::cout << "\t noAttributes                           " << noAttributes_                         << std::endl;
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
  std::cout << "\t   geometryConfig                       " << geometryConfig_                       << std::endl;
  std::cout << "\t   textureConfig                        " << textureConfig_                        << std::endl;
  std::cout << "\t   colorSpaceConversionConfig           " << colorSpaceConversionConfig_           << std::endl;
  std::cout << "\t   inverseColorSpaceConversionConfig    " << inverseColorSpaceConversionConfig_    << std::endl;
  std::cout << "\t occupancy map encoding " << std::endl;
  std::cout << "\t   maxCandidateCount                    " << maxCandidateCount_                    << std::endl;
  std::cout << "\t   occupancyPrecision                   " << occupancyPrecision_                   << std::endl;
  std::cout << "\t smoothing" << std::endl;
  std::cout << "\t   neighborCountSmoothing               " << neighborCountSmoothing_               << std::endl;
  std::cout << "\t   radius2Smoothing                     " << radius2Smoothing_                     << std::endl;
  std::cout << "\t   radius2BoundaryDetection             " << radius2BoundaryDetection_             << std::endl;
  std::cout << "\t   thresholdSmoothing                   " << thresholdSmoothing_                   << std::endl;
  std::cout << "\t coloring" << std::endl;
  std::cout << "\t   bestColorSearchRange                 " << bestColorSearchRange_                 << std::endl;
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
  return ret;
}
