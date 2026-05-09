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

#ifndef PCC_GS_ENCODER_PARAMETERS_H
#define PCC_GS_ENCODER_PARAMETERS_H

#include "PCCCommon.h"
#include "PCCGSEncoderConstant.h"
#include <vector>
#include <string>
#include <iostream>

namespace pcc {

/**
 * @brief Parameters for the GS Encoder
 */
struct PCCGSEncoderParameters {
  size_t geometryQP_;
  size_t attributeQP_;
  size_t packQP_;

  // Occupancy resolution for raw points patches
  size_t occupancyResolution_;

  // Geometry parameters
  size_t geometryBitDepth3D_;
  size_t geometryBitDepth2D_;

  // Attribute parameters
  size_t attributeBitDepth2D_;

  // Minimum image width
  size_t minimumImageWidth_;

  // Raw 3D offset settings
  bool   raw3dOffsetBitCountExplicitModeFlag_;
  size_t raw3dOffsetAxisBitCountMinus1_;

  // Encoder configuration files
  std::string debugPath_;
  std::string frameNameBase_;
  std::string binaryFileBase_;
  std::string geometryVideoEncoderConfig_;
  std::string attributeVideoEncoderConfig_;
  std::string packedVideoEncoderConfig_;

  // Packing settings
  bool             packGeometry_{};
  bool             packAttributes_{};
  bool             implicitPacking_{}; // m76092
  int              geometryPackedMapIndex_{};
  int              attributeScalePackedMapIndex_{};
  int              attributeOpacityPackedMapIndex_{};
  int              attributeRotationPackedMapIndex_{};
  int              attributeColorPackedMapIndex_{};
  std::vector<int> packedMapColumnsCount_{};
  // Gaussian Splatting
  bool gsdTransform2dPresentFlag_;
  bool dataAdaptiveShTransformEnabled_;
  bool enableShNonUniformQuantization_;

  // GS SEI
  std::string gsColmapFolderPath_;

  bool keepIntermediateFiles_;

  uint8_t getPackedMapIndexByAttributeTypeId( uint8_t attrTypeId ) const {
    switch ( attrTypeId ) {
      case ATTR_TYPE_COLOR:
      case ATTR_TYPE_SPHERICAL_HARMONIC: return attributeColorPackedMapIndex_;
      case ATTR_TYPE_OPACITY: return attributeOpacityPackedMapIndex_;
      case ATTR_TYPE_SCALE: return attributeScalePackedMapIndex_;
      case ATTR_TYPE_ROTATION: return attributeRotationPackedMapIndex_;
      default: std::cerr << "Unsupported attribute type for packing: " << int( attrTypeId ) << "\n"; exit( -1 );
    }
  }

  PCCGSEncoderParameters() {
    geometryQP_                          = 6;
    attributeQP_                         = 6;
    packQP_                              = 6;
    occupancyResolution_                 = 1;
    geometryBitDepth3D_                  = 18;
    geometryBitDepth2D_                  = 10;
    attributeBitDepth2D_                 = 10;
    raw3dOffsetBitCountExplicitModeFlag_ = false;
    raw3dOffsetAxisBitCountMinus1_       = 0;
    minimumImageWidth_                   = 1024;
    packGeometry_                        = false;
    packAttributes_                      = false;
    geometryPackedMapIndex_              = 0;
    attributeScalePackedMapIndex_        = 0;
    attributeOpacityPackedMapIndex_      = 0;
    attributeRotationPackedMapIndex_     = 0;
    attributeColorPackedMapIndex_        = 0;
    debugPath_                           = "./temp";
    frameNameBase_                       = "frame";
    binaryFileBase_                      = "data";
    keepIntermediateFiles_               = false;
    gsColmapFolderPath_                  = "";
    gsdTransform2dPresentFlag_           = false;
    dataAdaptiveShTransformEnabled_      = false;
    enableShNonUniformQuantization_      = false;
  }
};

}  // namespace pcc

#endif  // PCC_GS_ENCODER_PARAMETERS_H
