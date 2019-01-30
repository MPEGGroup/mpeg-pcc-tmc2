
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
#ifndef PCCEncoderParameters_h
#define PCCEncoderParameters_h

#include "PCCCommon.h"
#include "PCCMath.h"

namespace pcc {

class PCCEncoderParameters {
 public:
  PCCEncoderParameters();
  ~PCCEncoderParameters();
  void print();
  bool check();
  void completePath();

  size_t              startFrameNumber_;

  std::string         configurationFolder_;
  std::string         uncompressedDataFolder_;

  std::string         compressedStreamPath_;
  std::string         reconstructedDataPath_;

  ColorTransform      colorTransform_;
  std::string         colorSpaceConversionPath_;
  std::string         videoEncoderPath_;
  std::string         videoEncoderOccupancyMapPath_;

  std::string         colorSpaceConversionConfig_;
  std::string         inverseColorSpaceConversionConfig_;

  size_t              nbThread_;

  size_t              frameCount_;
  size_t              groupOfFramesSize_;
  std::string         uncompressedDataPath_;

  // packing
  size_t              minimumImageWidth_;
  size_t              minimumImageHeight_;

  // video encoding
  size_t              geometryQP_;
  size_t              textureQP_;

  // segmentation
  size_t              nnNormalEstimation_;
  size_t              maxNNCountRefineSegmentation_;
  size_t              iterationCountRefineSegmentation_;
  size_t              occupancyResolution_;
  size_t              minPointCountPerCCPatchSegmentation_;
  size_t              maxNNCountPatchSegmentation_;
  size_t              surfaceThickness_;
  size_t              minLevel_;//8,16,32,64
  size_t              maxAllowedDepth_;
  double              maxAllowedDist2MissedPointsDetection_;
  double              maxAllowedDist2MissedPointsSelection_;
  double              lambdaRefineSegmentation_;

  // occupancy map encoding
  size_t              maxCandidateCount_;
  size_t              occupancyPrecision_;
  std::string         occupancyMapVideoEncoderConfig_;
  size_t              occupancyMapQP_;
  bool                useOccupancyMapVideo_;

  // smoothing
  size_t              neighborCountSmoothing_;
  double              radius2Smoothing_;
  double              radius2BoundaryDetection_;
  double              thresholdSmoothing_;
  bool                gridSmoothing_;

  // color smoothing
  double              thresholdColorSmoothing_;
  double              thresholdLocalEntropy_;
  double              radius2ColorSmoothing_;
  size_t              neighborCountColorSmoothing_;
  bool                flagColorSmoothing_;

  // color pre-smoothing
  double              thresholdColorPreSmoothing_;
  double              thresholdColorPreSmoothingLocalEntropy_;
  double              radius2ColorPreSmoothing_;
  size_t              neighborCountColorPreSmoothing_;
  bool                flagColorPreSmoothing_;

  // coloring
  size_t              bestColorSearchRange_;

  // lossless
  bool                noAttributes_;
  bool                losslessGeo_;
  bool                losslessTexture_;
  bool                losslessGeo444_;

  // missed points video
  bool                useMissedPointsSeparateVideo_;
  std::string         geometryMPConfig_;
  std::string         textureMPConfig_;

  // scale and bias
  float               modelScale_;
  PCCVector3<float>   modelOrigin_;

  // patch sampling resolution
  size_t              testLevelOfDetail_;
  size_t              testLevelOfDetailSignaling_;
  std::string         geometryConfig_;
  std::string         geometryD0Config_;
  std::string         geometryD1Config_;
  std::string         textureConfig_;
  bool                keepIntermediateFiles_;
  bool                absoluteD1_;
  bool                constrainedPack_;
  bool                binArithCoding_;
  size_t              projectionMode_;

  // dilation
  bool                groupDilation_;
  bool                textureDilationOffLossless_;

  // EDD
  bool                enhancedDeltaDepthCode_;

  // reconstruction
  bool                removeDuplicatePoints_;
  bool                oneLayerMode_;
  bool                singleLayerPixelInterleaving_;

  //visual quality
  bool                patchColorSubsampling_;
  bool                deltaCoding_;
  bool                sixDirectionMode_;
  bool                surfaceSeparation_;

  // Flexible Patch Packing
  size_t              packingStrategy_;
  size_t              textureBGFill_;
  size_t              safeGuardDistance_;

  // Improve EDD
  bool                improveEDD_;

  // Lossy Missed Points Patch
  bool lossyMissedPointsPatch_;          
  double minNormSumOfInvDist4MPSelection_;
  int lossyMppGeoQP_;

  bool useAdditionalPointsPatch_;

};

}; //~namespace

#endif /* PCCEncoderParameters_h */
