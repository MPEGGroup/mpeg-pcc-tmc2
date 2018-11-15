
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
#include "PCCDecoderParameters.h"

using namespace pcc;

PCCDecoderParameters::PCCDecoderParameters() {
  compressedStreamPath_              = {};
  reconstructedDataPath_             = {};
  startFrameNumber_                  = 0;
  colorTransform_                    = COLOR_TRANSFORM_RGB_TO_YCBCR;
  colorSpaceConversionPath_          = {};
  inverseColorSpaceConversionConfig_ = {};
  videoDecoderPath_                  = {};
  videoDecoderOccupancyMapPath_      = {};
  nbThread_                          = 1;
  keepIntermediateFiles_             = false;
}

PCCDecoderParameters::~PCCDecoderParameters() {
}
void PCCDecoderParameters::print() {
  std::cout << "+ Parameters" << std::endl;
  std::cout << "\t compressedStreamPath                " << compressedStreamPath_              << std::endl;
  std::cout << "\t reconstructedDataPath               " << reconstructedDataPath_             << std::endl;
  std::cout << "\t startFrameNumber                    " << startFrameNumber_                  << std::endl;
  std::cout << "\t colorTransform                      " << colorTransform_                    << std::endl;
  std::cout << "\t nbThread                            " << nbThread_                          << std::endl;
  std::cout << "\t keepIntermediateFiles               " << keepIntermediateFiles_             << std::endl;
  std::cout << "\t video encoding" << std::endl;
  std::cout << "\t   colorSpaceConversionPath          " << colorSpaceConversionPath_          << std::endl;
  std::cout << "\t   videoDecoderPath                  " << videoDecoderPath_                  << std::endl;
  std::cout << "\t   videoDecoderOccupancyMapPath      " << videoDecoderOccupancyMapPath_      << std::endl;
  std::cout << "\t   inverseColorSpaceConversionConfig " << inverseColorSpaceConversionConfig_ << std::endl;
  std::cout << "\t   patchColorSubsampling             " << patchColorSubsampling_             << std::endl;
}

void PCCDecoderParameters::completePath(){
  if ( videoDecoderOccupancyMapPath_.empty() ||
      !exist(videoDecoderOccupancyMapPath_ ) ) {
    videoDecoderOccupancyMapPath_ = videoDecoderPath_;
  }
}

bool PCCDecoderParameters::check() {
  bool ret = true;
  if (!colorSpaceConversionPath_.empty() &&
      !inverseColorSpaceConversionConfig_.empty() ) {
    std::cout << "Info: Using external color space conversion" << std::endl;
    if (colorTransform_ != COLOR_TRANSFORM_NONE) {
      std::cout << "Using external color space conversion requires colorTransform = "
          "COLOR_TRANSFORM_NONE!\n";
      colorTransform_ = COLOR_TRANSFORM_NONE;
    }
  } else {
    std::cout << "Info: Using internal color space conversion" << std::endl;
    colorSpaceConversionPath_ = "";
    inverseColorSpaceConversionConfig_ = "";
  }

  if ( compressedStreamPath_.empty() ) {
    ret = false;
    std::cerr << "compressedStreamPath not set\n";
  }
  if ( colorSpaceConversionPath_.empty() ) {
    ret = false;
    std::cerr << "colorSpaceConversionPath not set\n";
  }
  if ( inverseColorSpaceConversionConfig_.empty()) {
    ret = false;
    std::cerr << "inverseColorSpaceConversionConfig not set\n";
  }
  if( !exist( colorSpaceConversionPath_) ) {
    ret = false;
    std::cerr << "colorSpaceConversionPath not exist\n";
  }
  if( !exist( inverseColorSpaceConversionConfig_ )) {
    ret = false;
    std::cerr << "inverseColorSpaceConversionConfig not exist\n";
  }
  if( !exist( compressedStreamPath_ ) ) {
    ret = false;
    std::cerr << "compressedStreamPath not exist\n";
  }
  if( videoDecoderPath_.empty() ) {
    ret = false;
    std::cerr << "videoDecoderPath not set\n";
  }
  if( !exist( videoDecoderPath_ ) ) {
    ret = false;
    std::cerr << "videoDecoderPath not exist\n";
  }
  if ( videoDecoderOccupancyMapPath_.empty() ) {
    ret = false;
    std::cerr << "videoDecoderOccupancyMapPath not set\n";
  }
  if ( !exist(videoDecoderOccupancyMapPath_ ) ) {
    ret = false;
    std::cerr << "videoDecoderOccupancyMapPath not exist\n";
  }
  if( reconstructedDataPath_.empty() ) {
    ret = false;
    std::cerr << "reconstructedDataPath not set\n";
  }
  return ret;
}
