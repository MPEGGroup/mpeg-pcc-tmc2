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
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCVirtualVideoEncoder.h"
using namespace pcc;

const std::vector<PointLocalReconstructionMode> g_pointLocalReconstructionMode = {
    {false, false, 0, 1}, {true, false, 0, 1}, {true, true, 0, 1}, {true, false, 0, 2}, {true, true, 0, 2},
    {false, false, 1, 1}, {true, false, 1, 1}, {true, true, 1, 1}, {true, false, 1, 2}, {true, true, 1, 2},
};

PCCEncoderParameters::PCCEncoderParameters() {
  uncompressedDataPath_                    = {};
  compressedStreamPath_                    = {};
  reconstructedDataPath_                   = {};
  configurationFolder_                     = {};
  uncompressedDataFolder_                  = {};
  startFrameNumber_                        = 0;
  frameCount_                              = 300;
  groupOfFramesSize_                       = 32;
  colorTransform_                          = COLOR_TRANSFORM_RGB_TO_YCBCR;
  colorSpaceConversionPath_                = {};
  colorSpaceConversionConfig_              = {};
  inverseColorSpaceConversionConfig_       = {};
  nnNormalEstimation_                      = 16;
  normalOrientation_                       = 1;
  gridBasedRefineSegmentation_             = true;
  maxNNCountRefineSegmentation_            = gridBasedRefineSegmentation_ ? 1024 : 256;
  iterationCountRefineSegmentation_        = gridBasedRefineSegmentation_ ? 10 : 100;
  voxelDimensionRefineSegmentation_        = 4;
  searchRadiusRefineSegmentation_          = 192;
  occupancyResolution_                     = 16;
  enablePatchSplitting_                    = true;
  maxPatchSize_                            = 1024;
  log2QuantizerSizeX_                      = 4;
  log2QuantizerSizeY_                      = 4;
  minPointCountPerCCPatchSegmentation_     = 16;
  maxNNCountPatchSegmentation_             = 16;
  surfaceThickness_                        = 4;
  minLevel_                                = 64;  // fix value
  maxAllowedDist2RawPointsDetection_       = 9.0;
  maxAllowedDist2RawPointsSelection_       = 1.0;
  lambdaRefineSegmentation_                = 3.0;
  minimumImageWidth_                       = 1280;
  minimumImageHeight_                      = 1280;
  maxCandidateCount_                       = 4;
  occupancyPrecision_                      = 4;
  occupancyMapVideoEncoderConfig_          = {};
  occupancyMapQP_                          = 8;
  occupancyMapRefinement_                  = false;
  decodedAtlasInformationHash_             = 0;
  postprocessSmoothingFilter_              = 1;
  flagGeometrySmoothing_                   = true;
  patchExpansion_                          = false;
  gridSmoothing_                           = true;
  gridSize_                                = 8;
  neighborCountSmoothing_                  = 4 * 16;
  radius2Smoothing_                        = 4.0 * 16;
  radius2BoundaryDetection_                = 4.0 * 16;
  thresholdSmoothing_                      = 64.0;
  bestColorSearchRange_                    = 0;
  numNeighborsColorTransferFwd_            = 1;
  numNeighborsColorTransferBwd_            = 1;
  useDistWeightedAverageFwd_               = true;
  useDistWeightedAverageBwd_               = true;
  skipAvgIfIdenticalSourcePointPresentFwd_ = false;
  skipAvgIfIdenticalSourcePointPresentBwd_ = false;
  distOffsetFwd_                           = 0.0001;
  distOffsetBwd_                           = 0.0001;
  maxGeometryDist2Fwd_                     = 10000.0;
  maxGeometryDist2Bwd_                     = 10000.0;
  maxColorDist2Fwd_                        = 10000.0;
  maxColorDist2Bwd_                        = 10000.0;
  excludeColorOutlier_                     = false;
  thresholdColorOutlierDist_               = 10.0;
  videoEncoderOccupancyPath_               = {};
  videoEncoderGeometryPath_                = {};
  videoEncoderAttributePath_               = {};
  videoEncoderOccupancyCodecId_            = PCCVirtualVideoEncoder<uint8_t>::getDefaultCodecId();
  videoEncoderGeometryCodecId_             = PCCVirtualVideoEncoder<uint8_t>::getDefaultCodecId();
  videoEncoderAttributeCodecId_            = PCCVirtualVideoEncoder<uint8_t>::getDefaultCodecId();
  byteStreamVideoCoderOccupancy_           = true;
  byteStreamVideoCoderGeometry_            = true;
  byteStreamVideoCoderAttribute_           = true;
  geometryQP_                              = 28;
  textureQP_                               = 43;
  geometryConfig_                          = {};
  geometryD0Config_                        = {};
  geometryD1Config_                        = {};
  textureConfig_                           = {};
  textureT0Config_                         = {};
  textureT1Config_                         = {};
  losslessGeo_                             = false;
  noAttributes_                            = false;
  losslessGeo444_                          = false;
  useRawPointsSeparateVideo_               = false;
  geometryAuxVideoConfig_                  = {};
  textureAuxVideoConfig_                   = {};
  nbThread_                                = 1;
  keepIntermediateFiles_                   = false;

  absoluteD1_                             = true;
  absoluteT1_                             = true;
  multipleStreams_                        = false;
  qpAdjT1_                                = 0;
  qpAdjD1_                                = 0;
  constrainedPack_                        = true;
  thresholdColorSmoothing_                = 10.0;
  thresholdColorDifference_               = 10.0;
  thresholdColorVariation_                = 6.0;
  flagColorSmoothing_                     = false;
  cgridSize_                              = 4;
  thresholdColorPreSmoothing_             = 10.0;
  thresholdColorPreSmoothingLocalEntropy_ = 4.5;
  radius2ColorPreSmoothing_               = 4.0 * 16;
  neighborCountColorPreSmoothing_         = 4 * 16;
  flagColorPreSmoothing_                  = true;
  groupDilation_                          = true;
  textureDilationOffLossless_             = true;
  enhancedOccupancyMapCode_               = false;
  EOMFixBitCount_                         = 2;
  offsetLossyOM_                          = 0;
  thresholdLossyOM_                       = 0;
  prefilterLossyOM_                       = false;
  patchColorSubsampling_                  = false;
  mapCountMinus1_                         = 1;

  // reconstruction
  removeDuplicatePoints_      = true;
  pointLocalReconstruction_   = false;
  plrlNumberOfModes_          = 6;
  patchSize_                  = 9;
  singleMapPixelInterleaving_ = false;
  surfaceSeparation_          = false;

  // level of detail
  levelOfDetailX_ = 1;
  levelOfDetailY_ = 1;

  // Flexible Patch Packing
  packingStrategy_      = 1;
  textureBGFill_        = 1;
  safeGuardDistance_    = 0;
  useEightOrientations_ = false;
  lowDelayEncoding_     = false;
  geometryPadding_      = 0U;

  // lossy raw points patch
  lossyRawPointsPatch_             = false;
  minNormSumOfInvDist4MPSelection_ = 0.35;
  lossyRawPointPatchGeoQP_         = 4;
  // GPA
  globalPatchAllocation_ = 0;
  // GTP
  globalPackingStrategyGOF_       = 0;
  globalPackingStrategyReset_     = false;
  globalPackingStrategyThreshold_ = 0;
  use3dmc_                        = true;
#ifdef USE_HM_PCC_RDO
  usePccRDO_ = false;
#endif
  enhancedPP_                       = true;
  minWeightEPP_                     = 0.6;
  additionalProjectionPlaneMode_    = 0;
  partialAdditionalProjectionPlane_ = 0.00;

  // 3D and 2D bit depths
  geometry3dCoordinatesBitdepth_ = 10;
  geometryNominal2dBitdepth_     = 8;

  // Partitions and tiles
  enablePointCloudPartitioning_ = false;
  numTilesHor_                  = 2;
  tileHeightToWidthRatio_       = 1;

  // Sort raw points by Morton code
  mortonOrderSortRawPoints_     = false;
  textureRawSeparateVideoWidth_ = 128;

  // Separate high gradient points
  highGradientSeparation_   = false;
  minGradient_              = 15.0;
  minNumHighGradientPoints_ = 256;

  // Patch border filtering
  pbfEnableFlag_    = false;
  pbfPassesCount_   = 0;
  pbfFilterSize_    = 0;
  pbfLog2Threshold_ = 2;

  patchPrecedenceOrderFlag_ = false;
  maxNumRefAtlasList_       = 1;
  maxNumRefAtlasFrame_      = 1;

  log2MaxAtlasFrameOrderCntLsb_ = 10;
  tileSegmentationType_         = 0;
  numMaxTilePerFrame_           = 1;
  uniformPartitionSpacing_      = true;
  tilePartitionWidth_           = 0;
  tilePartitionHeight_          = 0;
}

PCCEncoderParameters::~PCCEncoderParameters() = default;

void PCCEncoderParameters::completePath() {
  if ( !uncompressedDataFolder_.empty() ) {
    if ( !uncompressedDataPath_.empty() ) { uncompressedDataPath_ = uncompressedDataFolder_ + uncompressedDataPath_; }
  }
  if ( !configurationFolder_.empty() ) {
    if ( !geometryConfig_.empty() ) { geometryConfig_ = configurationFolder_ + geometryConfig_; }
    if ( !geometryD0Config_.empty() ) { geometryD0Config_ = configurationFolder_ + geometryD0Config_; }
    if ( !geometryD1Config_.empty() ) { geometryD1Config_ = configurationFolder_ + geometryD1Config_; }
    if ( !textureConfig_.empty() ) { textureConfig_ = configurationFolder_ + textureConfig_; }
    if ( !textureT0Config_.empty() ) { textureT0Config_ = configurationFolder_ + textureT0Config_; }
    if ( !textureT1Config_.empty() ) { textureT1Config_ = configurationFolder_ + textureT1Config_; }
    if ( !geometryAuxVideoConfig_.empty() ) {
      geometryAuxVideoConfig_ = configurationFolder_ + geometryAuxVideoConfig_;
    }
    if ( !textureAuxVideoConfig_.empty() ) { textureAuxVideoConfig_ = configurationFolder_ + textureAuxVideoConfig_; }

    if ( !inverseColorSpaceConversionConfig_.empty() ) {
      inverseColorSpaceConversionConfig_ = configurationFolder_ + inverseColorSpaceConversionConfig_;
    }
    if ( !colorSpaceConversionConfig_.empty() ) {
      colorSpaceConversionConfig_ = configurationFolder_ + colorSpaceConversionConfig_;
    }
    if ( !occupancyMapVideoEncoderConfig_.empty() ) {
      occupancyMapVideoEncoderConfig_ = configurationFolder_ + occupancyMapVideoEncoderConfig_;
    }
    if ( useRawPointsSeparateVideo_ ) {
      if ( !geometryAuxVideoConfig_.empty() ) {
        geometryAuxVideoConfig_ = configurationFolder_ + geometryAuxVideoConfig_;
      }
      if ( !textureAuxVideoConfig_.empty() ) { textureAuxVideoConfig_ = configurationFolder_ + textureAuxVideoConfig_; }
    }
  }
}

void PCCEncoderParameters::print() {
  std::cout << "+ Parameters" << std::endl;
  std::cout << "\t losslessGeo                              " << losslessGeo_ << std::endl;
  std::cout << "\t noAttributes                             " << noAttributes_ << std::endl;
  std::cout << "\t raw Geometry Colour Plane                " << ( losslessGeo444_ ? "444" : "420" ) << std::endl;
  std::cout << "\t enhancedOccupancyMapCode                 " << enhancedOccupancyMapCode_ << std::endl;
  std::cout << "\t useRawPointsSeparateVideo                " << useRawPointsSeparateVideo_ << std::endl;
  std::cout << "\t textureRawSeparateVideoWidth             " << textureRawSeparateVideoWidth_ << std::endl;
  std::cout << "\t uncompressedDataPath                     " << uncompressedDataPath_ << std::endl;
  std::cout << "\t compressedStreamPath                     " << compressedStreamPath_ << std::endl;
  std::cout << "\t reconstructedDataPath                    " << reconstructedDataPath_ << std::endl;
  std::cout << "\t frameCount                               " << frameCount_ << std::endl;
  std::cout << "\t mapCountMinus1                           " << mapCountMinus1_ << std::endl;
  std::cout << "\t startFrameNumber                         " << startFrameNumber_ << std::endl;
  std::cout << "\t groupOfFramesSize                        " << groupOfFramesSize_ << std::endl;
  std::cout << "\t colorTransform                           " << colorTransform_ << std::endl;
  std::cout << "\t nbThread                                 " << nbThread_ << std::endl;
  std::cout << "\t keepIntermediateFiles                    " << keepIntermediateFiles_ << std::endl;
  std::cout << "\t absoluteD1                               " << absoluteD1_ << std::endl;
  std::cout << "\t multipleStreams                          " << multipleStreams_ << std::endl;
  std::cout << "\t qpD1                                     " << qpAdjD1_ << std::endl;
  std::cout << "\t absoluteT1                               " << absoluteT1_ << std::endl;
  std::cout << "\t qpT1                                     " << qpAdjT1_ << std::endl;
  std::cout << "\t constrainedPack                          " << constrainedPack_ << std::endl;
  std::cout << "\t  maxNumRefPatchList                      " << maxNumRefAtlasList_ << std::endl;
  std::cout << "\t maxNumRefIndex                           " << maxNumRefAtlasFrame_ << std::endl;
  std::cout << "\t Segmentation" << std::endl;
  std::cout << "\t   nnNormalEstimation                     " << nnNormalEstimation_ << std::endl;
  std::cout << "\t   normalOrientation                      " << normalOrientation_ << std::endl;
  std::cout << "\t   gridBasedRefineSegmentation            " << gridBasedRefineSegmentation_ << std::endl;
  std::cout << "\t   maxNNCountRefineSegmentation           " << maxNNCountRefineSegmentation_ << std::endl;
  std::cout << "\t   iterationCountRefineSegmentation       " << iterationCountRefineSegmentation_ << std::endl;
  std::cout << "\t   voxelDimensionRefineSegmentation       " << voxelDimensionRefineSegmentation_ << std::endl;
  std::cout << "\t   searchRadiusRefineSegmentation         " << searchRadiusRefineSegmentation_ << std::endl;
  std::cout << "\t   occupancyResolution                    " << occupancyResolution_ << std::endl;
  std::cout << "\t   enablePatchSplitting                   " << enablePatchSplitting_ << std::endl;
  std::cout << "\t   maxPatchSize                           " << maxPatchSize_ << std::endl;
  std::cout << "\t   quantization step for patch size       "
            << "1<<" << log2QuantizerSizeX_ << ", 1<<" << log2QuantizerSizeY_ << std::endl;
  std::cout << "\t   minPointCountPerCCPatchSegmentation    " << minPointCountPerCCPatchSegmentation_ << std::endl;
  std::cout << "\t   maxNNCountPatchSegmentation            " << maxNNCountPatchSegmentation_ << std::endl;
  std::cout << "\t   surfaceThickness                       " << surfaceThickness_ << std::endl;
  std::cout << "\t   maxAllowedDist2RawPointsDetection      " << maxAllowedDist2RawPointsDetection_ << std::endl;
  std::cout << "\t   maxAllowedDist2RawPointsSelection      " << maxAllowedDist2RawPointsSelection_ << std::endl;
  std::cout << "\t   lambdaRefineSegmentation               " << lambdaRefineSegmentation_ << std::endl;
  std::cout << "\t   depthQuantizationStep                  " << minLevel_ << std::endl;
  std::cout << "\t   highGradientSeparation                 " << highGradientSeparation_ << std::endl;
  if ( highGradientSeparation_ ) {
    std::cout << "\t     minGradient                        " << minGradient_ << std::endl;
    std::cout << "\t     minNumHighGradientPoints           " << minNumHighGradientPoints_ << std::endl;
  }
  std::cout << "\t Packing" << std::endl;
  std::cout << "\t   minimumImageWidth                      " << minimumImageWidth_ << std::endl;
  std::cout << "\t   minimumImageHeight                     " << minimumImageHeight_ << std::endl;
  std::cout << "\t   log2MaxAtlasFrameOrderCntLsb           " << log2MaxAtlasFrameOrderCntLsb_ << std::endl;
  std::cout << "\t   packingStrategy                        " << packingStrategy_ << std::endl;
  std::cout << "\t   useEightOrientations                   " << useEightOrientations_ << std::endl;
  std::cout << "\t   safeGuardDistance                      " << safeGuardDistance_ << std::endl;
  std::cout << "\t   globalPatchAllocation                  " << globalPatchAllocation_ << std::endl;
  if ( globalPatchAllocation_ == 2 ) {
    std::cout << "\t   globalPackingStrategyGOF             " << geometryD0Config_ << std::endl;
    std::cout << "\t   globalPackingStrategyReset           " << geometryD1Config_ << std::endl;
    std::cout << "\t   globalPackingStrategyThreshold       " << geometryD1Config_ << std::endl;
  }
  std::cout << "\t   patchPrecedenceOrder                   " << patchPrecedenceOrderFlag_ << std::endl;
  std::cout << "\t   lowDelayEncoding                       " << lowDelayEncoding_ << std::endl;
  std::cout << "\t   textureBGFill                          " << textureBGFill_ << std::endl;
  std::cout << "\t   geometryPadding                        " << geometryPadding_ << std::endl;
  std::cout << "\t Video encoding" << std::endl;
  std::cout << "\t   geometryQP                             " << geometryQP_ << std::endl;
  std::cout << "\t   textureQP                              " << textureQP_ << std::endl;
  std::cout << "\t   colorSpaceConversionPath               " << colorSpaceConversionPath_ << std::endl;
  std::cout << "\t   videoEncoderOccupancyPath              " << videoEncoderOccupancyPath_ << std::endl;
  std::cout << "\t   videoEncoderGeometryPath               " << videoEncoderGeometryPath_ << std::endl;
  std::cout << "\t   videoEncoderAttributePath              " << videoEncoderAttributePath_ << std::endl;
  std::cout << "\t   videoEncoderOccupancyCodecId           " << videoEncoderOccupancyCodecId_ << std::endl;
  std::cout << "\t   videoEncoderGeometryCodecId            " << videoEncoderGeometryCodecId_ << std::endl;
  std::cout << "\t   videoEncoderAttributeCodecId           " << videoEncoderAttributeCodecId_ << std::endl;
  if ( multipleStreams_ ) {
    std::cout << "\t   geometryD0Config                     " << geometryD0Config_ << std::endl;
    std::cout << "\t   geometryD1Config                     " << geometryD1Config_ << std::endl;
  } else {
    std::cout << "\t   geometryConfig                       " << geometryConfig_ << std::endl;
  }
  if ( multipleStreams_ ) {
    std::cout << "\t   textureT0Config                      " << textureT0Config_ << std::endl;
    std::cout << "\t   textureT1Config                      " << textureT1Config_ << std::endl;
  } else {
    std::cout << "\t   textureConfig                        " << textureConfig_ << std::endl;
  }
  if ( useRawPointsSeparateVideo_ ) {
    if ( losslessGeo_ ) {
      std::cout << "\t geometryAuxVideoConfig                     " << geometryAuxVideoConfig_ << std::endl;
      std::cout << "\t textureAuxVideoConfig                      " << textureAuxVideoConfig_ << std::endl;
    }
  }
  std::cout << "\t   colorSpaceConversionConfig             " << colorSpaceConversionConfig_ << std::endl;
  std::cout << "\t   inverseColorSpaceConversionConfig      " << inverseColorSpaceConversionConfig_ << std::endl;
  std::cout << "\t   apply3dMotionCompensation              " << use3dmc_ << std::endl;
  std::cout << "\t Dilation" << std::endl;
  std::cout << "\t   GroupDilation                          " << groupDilation_ << std::endl;
  std::cout << "\t   textureDilationOffLossless             " << textureDilationOffLossless_ << std::endl;
  std::cout << "\t Occupancy map encoding " << std::endl;
  std::cout << "\t   maxCandidateCount                      " << maxCandidateCount_ << std::endl;
  std::cout << "\t   occupancyPrecision                     " << occupancyPrecision_ << std::endl;
  std::cout << "\t   occupancyMapVideoEncoderConfig         " << occupancyMapVideoEncoderConfig_ << std::endl;
  std::cout << "\t   occupancyMapQP                         " << occupancyMapQP_ << std::endl;
  std::cout << "\t   EOMFixBitCount                         " << EOMFixBitCount_ << std::endl;
  std::cout << "\t   occupancyMapRefinement                 " << occupancyMapRefinement_ << std::endl;
  std::cout << "\t Lossy occupancy Map coding" << std::endl;
  std::cout << "\t   Lossy occupancy map offset             " << offsetLossyOM_ << std::endl;
  std::cout << "\t   Lossy occupancy map threshold          " << thresholdLossyOM_ << std::endl;
  std::cout << "\t   Lossy occupancy map prefilter          " << prefilterLossyOM_ << std::endl;
  std::cout << "\t   postprocessSmoothingFilter             " << postprocessSmoothingFilter_ << std::endl;
  std::cout << "\t Decoded Atlas Information Hash           " << ( decodedAtlasInformationHash_ > 0 ? 1 : 0 )
            << std::endl;
  if ( decodedAtlasInformationHash_ > 0 )
    std::cout << "\t   DecodedAtlasInformationHash Type           " << decodedAtlasInformationHash_ << std::endl;
  std::cout << "\t Geometry smoothing                       " << std::endl;
  std::cout << "\t   flagGeometrySmoothing                  " << flagGeometrySmoothing_ << std::endl;
  if ( flagGeometrySmoothing_ ) {
    std::cout << "\t   gridSmoothing                        " << gridSmoothing_ << std::endl;
    if ( gridSmoothing_ ) {
      std::cout << "\t   gridSize                           " << gridSize_ << std::endl;
      std::cout << "\t   thresholdSmoothing                 " << thresholdSmoothing_ << std::endl;
    } else {
      std::cout << "\t   neighborCountSmoothing             " << neighborCountSmoothing_ << std::endl;
      std::cout << "\t   radius2Smoothing                   " << radius2Smoothing_ << std::endl;
      std::cout << "\t   radius2BoundaryDetection           " << radius2BoundaryDetection_ << std::endl;
      std::cout << "\t   thresholdSmoothing                 " << thresholdSmoothing_ << std::endl;
    }
  }
  std::cout << "\t   patchExpansion                         " << patchExpansion_ << std::endl;
  std::cout << "\t Color smoothing" << std::endl;
  std::cout << "\t   flagColorSmoothing                     " << flagColorSmoothing_ << std::endl;
  if ( flagColorSmoothing_ ) {
    std::cout << "\t   thresholdColorSmoothing                " << thresholdColorSmoothing_ << std::endl;
    std::cout << "\t   thresholdColorDifference               " << thresholdColorDifference_ << std::endl;
    std::cout << "\t   thresholdColorVariation                " << thresholdColorVariation_ << std::endl;
    std::cout << "\t   cgridSize                              " << cgridSize_ << std::endl;
  }
  std::cout << "\t Color pre-smoothing                      " << std::endl;
  std::cout << "\t   thresholdColorPreSmoothing             " << thresholdColorSmoothing_ << std::endl;
  std::cout << "\t   thresholdColorPreSmoothingLocalEntropy " << thresholdColorPreSmoothingLocalEntropy_ << std::endl;
  std::cout << "\t   radius2ColorPreSmoothing               " << radius2ColorPreSmoothing_ << std::endl;
  std::cout << "\t   neighborCountColorPreSmoothing         " << neighborCountColorPreSmoothing_ << std::endl;
  std::cout << "\t   flagColorPreSmoothing                  " << flagColorPreSmoothing_ << std::endl;
  std::cout << "\t Coloring" << std::endl;
  std::cout << "\t   bestColorSearchRange                   " << bestColorSearchRange_ << std::endl;
  std::cout << "\t   numNeighborsColorTransferFwd           " << numNeighborsColorTransferFwd_ << std::endl;
  std::cout << "\t   numNeighborsColorTransferBwd           " << numNeighborsColorTransferBwd_ << std::endl;
  std::cout << "\t   useDistWeightedAverageFwd              " << useDistWeightedAverageFwd_ << std::endl;
  std::cout << "\t   useDistWeightedAverageBwd              " << useDistWeightedAverageBwd_ << std::endl;
  std::cout << "\t   skipAvgIfIdenticalSourcePointPresentFwd " << skipAvgIfIdenticalSourcePointPresentFwd_ << std::endl;
  std::cout << "\t   skipAvgIfIdenticalSourcePointPresentBwd " << skipAvgIfIdenticalSourcePointPresentBwd_ << std::endl;
  std::cout << "\t   distOffsetFwd                          " << distOffsetFwd_ << std::endl;
  std::cout << "\t   distOffsetBwd                          " << distOffsetBwd_ << std::endl;
  std::cout << "\t   geometryDist2ThresholdFwd              " << maxGeometryDist2Fwd_ << std::endl;
  std::cout << "\t   geometryDist2ThresholdBwd              " << maxGeometryDist2Bwd_ << std::endl;
  std::cout << "\t   maxColorDist2Fwd                       " << maxColorDist2Fwd_ << std::endl;
  std::cout << "\t   maxColorDist2Bwd                       " << maxColorDist2Bwd_ << std::endl;
  std::cout << "\t   patchColorSubsampling                  " << patchColorSubsampling_ << std::endl;
  std::cout << "\t   excludeColorOutlier                    " << excludeColorOutlier_ << std::endl;
  if ( excludeColorOutlier_ ) {
    std::cout << "\t     thresholdColorOutlierDist          " << thresholdColorOutlierDist_ << std::endl;
  }
  std::cout << "\t Reconstruction " << std::endl;
  std::cout << "\t   removeDuplicatePoints                  " << removeDuplicatePoints_ << std::endl;
  std::cout << "\t   pointLocalReconstruction               " << pointLocalReconstruction_ << std::endl;
  std::cout << "\t     plrlNumberOfModes                    " << plrlNumberOfModes_ << std::endl;
  std::cout << "\t     patchSize                            " << patchSize_ << std::endl;
  std::cout << "\t   singleLayerPixelInterleaving           " << singleMapPixelInterleaving_ << std::endl;
  std::cout << "\t surface Separation                       " << surfaceSeparation_ << std::endl;
  std::cout << "\t Lossy raw points patch                   " << std::endl;
  std::cout << "\t   lossyRawPointsPatch                    " << lossyRawPointsPatch_ << std::endl;
  std::cout << "\t   minNormSumOfInvDist4MPSelection        " << minNormSumOfInvDist4MPSelection_ << std::endl;
  std::cout << "\t   lossyRawPointPatchGeoQP                " << lossyRawPointPatchGeoQP_ << std::endl;
  std::cout << "\t raw points sorting                       " << std::endl;
  std::cout << "\t   mortonOrderSortRawPoints               " << mortonOrderSortRawPoints_ << std::endl;
  std::cout << "\t Enhanced projection plane                " << enhancedPP_ << std::endl;
  std::cout << "\t AdditionalProjectionPlane                " << std::endl;
  std::cout << "\t   additionalProjectionPlaneMode          " << additionalProjectionPlaneMode_ << std::endl;
  std::cout << "\t   partialAdditionalProjectionPlane       " << partialAdditionalProjectionPlane_ << std::endl;
  std::cout << "\t Geometry 2D and 3D bitdepths             " << std::endl;
  std::cout << "\t   geometry3dCoordinatesBitdepth          " << geometry3dCoordinatesBitdepth_ << std::endl;
  std::cout << "\t   geometryNominal2dBitdepth              " << geometryNominal2dBitdepth_ << std::endl;
  std::cout << "\t Image partitions and tiles               " << std::endl;
  std::cout << "\t tileSegmentationType                     " << tileSegmentationType_ << std::endl;
  if ( tileSegmentationType_ > 1 ) {
    std::cout << "\t   numMaxTilePerFrame                     " << numMaxTilePerFrame_ << std::endl;
    if ( numMaxTilePerFrame_ > 1 ) {
      std::cout << "\t   uniformPartitionSpacing              " << uniformPartitionSpacing_ << std::endl;
      std::cout << "\t   tilePartitionWidth                   " << tilePartitionWidth_ << std::endl;
      std::cout << "\t   tilePartitionHeight                  " << tilePartitionHeight_ << std::endl;
    }
  }
  std::cout << "\t Point cloud partitions and tiles         " << std::endl;
  std::cout << "\t   enablePointCloudPartitioning           " << enablePointCloudPartitioning_ << std::endl;
  if ( enablePointCloudPartitioning_ ) {
    auto printVector = []( std::vector<int>& vec ) {
      for ( int i = 0; i < vec.size(); ++i ) {
        std::cout << vec[i];
        if ( i < vec.size() - 1 ) { std::cout << ","; }
      }
      std::cout << std::endl;
    };
    std::cout << "\t   roiBoundingBoxMinX                     ";
    printVector( roiBoundingBoxMinX_ );
    std::cout << "\t   roiBoundingBoxMaxX                     ";
    printVector( roiBoundingBoxMaxX_ );
    std::cout << "\t   roiBoundingBoxMinY                     ";
    printVector( roiBoundingBoxMinY_ );
    std::cout << "\t   roiBoundingBoxMaxY                     ";
    printVector( roiBoundingBoxMaxY_ );
    std::cout << "\t   roiBoundingBoxMinZ                     ";
    printVector( roiBoundingBoxMinZ_ );
    std::cout << "\t   roiBoundingBoxMaxZ                     ";
    printVector( roiBoundingBoxMaxZ_ );
    std::cout << "\t   numTilesHor                            " << numTilesHor_ << std::endl;
    std::cout << "\t   tileHeightToWidthRatio                 " << tileHeightToWidthRatio_ << std::endl;
    std::cout << "\t   numCutsAlong1stLongestAxis             " << numCutsAlong1stLongestAxis_ << std::endl;
    std::cout << "\t   numCutsAlong2ndLongestAxis             " << numCutsAlong2ndLongestAxis_ << std::endl;
    std::cout << "\t   numCutsAlong3rdLongestAxis             " << numCutsAlong3rdLongestAxis_ << std::endl;
    std::vector<size_t> vecSizes = {roiBoundingBoxMinX_.size(), roiBoundingBoxMaxX_.size(), roiBoundingBoxMinY_.size(),
                                    roiBoundingBoxMaxY_.size(), roiBoundingBoxMinZ_.size(), roiBoundingBoxMaxZ_.size()};
    for ( int i = 0; i < vecSizes.size(); ++i ) {
      if ( vecSizes[i] != vecSizes[0] ) {
        std::cerr << "All the 6 arrays roiBoundingBox[Min-Max][X-Y-Z] must "
                     "have the same number of elements."
                  << std::endl;
        std::exit( EXIT_FAILURE );
      }
    }
    numROIs_ = int( roiBoundingBoxMinX_.size() );
    for ( int i = 0; i < roiBoundingBoxMinX_.size(); ++i ) {
      if ( !( roiBoundingBoxMinX_[i] < roiBoundingBoxMaxX_[i] ) ||
           !( roiBoundingBoxMinY_[i] < roiBoundingBoxMaxY_[i] ) ||
           !( roiBoundingBoxMinZ_[i] < roiBoundingBoxMaxZ_[i] ) ) {
        std::cerr << "ROI min/max values are not correct." << std::endl;
        std::exit( EXIT_FAILURE );
      }
    }
  }
  std::cout << "\t Patch block filtering" << std::endl;
  std::cout << "\t   pbfEnableFlag                          " << pbfEnableFlag_ << std::endl;
  std::cout << "\t   pbfPassesCount                         " << pbfPassesCount_ << std::endl;
  std::cout << "\t   pbfFilterSize                          " << pbfFilterSize_ << std::endl;
  std::cout << "\t   pbfLog2Threshold                       " << pbfLog2Threshold_ << std::endl;
  std::cout << std::endl;
}

bool PCCEncoderParameters::check() {
  bool ret = true;
  if ( !inverseColorSpaceConversionConfig_.empty() && !colorSpaceConversionConfig_.empty() ) {
    std::cout << "Info: Using external color space conversion" << std::endl;
    if ( colorTransform_ != COLOR_TRANSFORM_NONE ) {
      std::cerr << "Using external color space conversion requires colorTransform = "
                   "COLOR_TRANSFORM_NONE!\n";
      colorTransform_ = COLOR_TRANSFORM_NONE;
    }
    if ( !exist( colorSpaceConversionConfig_ ) ) {
      ret = false;
      std::cerr << "colorSpaceConversionConfig not exist\n";
    }
    if ( !exist( inverseColorSpaceConversionConfig_ ) ) {
      ret = false;
      std::cerr << "inverseColorSpaceConversionConfig not exist\n";
    }
  } else {
    std::cout << "Info: Using internal color space conversion" << std::endl;
    colorSpaceConversionPath_          = "";
    inverseColorSpaceConversionConfig_ = "";
    colorSpaceConversionConfig_        = "";
  }

  if ( compressedStreamPath_.empty() ) {
    ret = false;
    std::cerr << "compressedStreamPath not set\n";
  }
  if ( uncompressedDataPath_.empty() ) {
    ret = false;
    std::cerr << "uncompressedDataPath not set\n";
  }

  if ( !PCCVirtualVideoEncoder<uint8_t>::checkCodecId( videoEncoderOccupancyCodecId_ ) ||
       !PCCVirtualVideoEncoder<uint8_t>::checkCodecId( videoEncoderGeometryCodecId_ ) ||
       !PCCVirtualVideoEncoder<uint8_t>::checkCodecId( videoEncoderAttributeCodecId_ ) ) {
    std::cerr << "ERROR: CodecId is not correct" << std::endl;
    ret = false;
  }
#if defined( USE_HMAPP_VIDEO_CODEC ) || defined( USE_JMAPP_VIDEO_CODEC )
  if ( ( (int)videoEncoderOccupancyCodecId_ ) < 2 &&
       ( ( videoEncoderOccupancyPath_.empty() || !exist( videoEncoderOccupancyPath_ ) ) ) ) {
    std::cerr << "ERROR: videoEncoderOccupancyPath_ not set or not exist : " << videoEncoderOccupancyPath_ << std::endl;
    ret = false;
  }
  if ( ( (int)videoEncoderGeometryCodecId_ ) < 2 &&
       ( ( videoEncoderGeometryPath_.empty() || !exist( videoEncoderGeometryPath_ ) ) ) ) {
    std::cerr << "ERROR: videoEncoderGeometryPath not set or not exist : " << videoEncoderGeometryPath_ << std::endl;
    ret = false;
  }
  if ( ( (int)videoEncoderAttributeCodecId_ ) < 2 &&
       ( ( videoEncoderAttributePath_.empty() || !exist( videoEncoderAttributePath_ ) ) ) ) {
    std::cerr << "ERROR: videoEncoderAttributePath not set or not exist : " << videoEncoderAttributePath_ << std::endl;
    ret = false;
  }
#endif

  if ( multipleStreams_ ) {
    if ( geometryD0Config_.empty() || geometryD1Config_.empty() ) {
      geometryD0Config_ = geometryConfig_.substr( 0, geometryConfig_.find_last_of( '.' ) ) + "-D0.cfg";
      geometryD1Config_ = geometryConfig_.substr( 0, geometryConfig_.find_last_of( '.' ) ) + "-D1.cfg";
    }
  } else {
    geometryD0Config_ = {};
    geometryD1Config_ = {};
    if ( geometryConfig_.empty() ) {
      ret = false;
      std::cerr << "When multipleStreams is not true, geometryConfig_ should "
                   "be non-empty\n";
    }
  }

  if ( multipleStreams_ ) {
    if ( textureT0Config_.empty() || textureT1Config_.empty() ) {
      textureT0Config_ = textureConfig_.substr( 0, textureConfig_.find_last_of( '.' ) ) + "-T0.cfg";
      textureT1Config_ = textureConfig_.substr( 0, textureConfig_.find_last_of( '.' ) ) + "-T1.cfg";
    }
  } else {
    textureT0Config_ = {};
    textureT1Config_ = {};
    if ( textureConfig_.empty() ) {
      std::cerr << "When multipleStreams is false, textureConfig_ should be "
                   "non-empty\n";
      ret = false;
    }
  }
  if ( !multipleStreams_ && !absoluteD1_ ) {
    std::cerr << "absoluteD1_ should be true when multipleStreams_ is false\n";
    absoluteD1_ = true;
  }
  if ( normalOrientation_ > 3 ) {
    std::cerr << "WARNING: the normal orientation is out of the possible range [0;3]\n";
    normalOrientation_ = 1;
  }
  if ( !absoluteT1_ && absoluteD1_ ) {
    std::cerr << "absoluteT1 should be true when absoluteD1 is true\n";
    absoluteT1_ = 1;
  }
  if ( losslessGeo_ ) {
#ifdef USE_HM_PCC_RDO
    usePccRDO_ = false;
#endif
    pbfEnableFlag_          = false;
    occupancyMapRefinement_ = false;
    flagColorSmoothing_     = false;
    flagGeometrySmoothing_  = false;
    gridSmoothing_          = false;
    if ( lossyRawPointsPatch_ == true ) {
      std::cerr << "WARNING: lossyRawPointsPatch_ is only for lossy "
                   "coding mode for now. Force lossyRawPointsPatch_=FALSE.\n";
      lossyRawPointsPatch_ = false;
    }
    if ( pointLocalReconstruction_ ) {
      pointLocalReconstruction_ = false;
      std::cerr << "WARNING: pointLocalReconstruction_ is only for lossy "
                   "coding mode for now. Force pointLocalReconstruction_=FALSE.\n";
    }
    if ( singleMapPixelInterleaving_ ) {
      singleMapPixelInterleaving_ = false;
      std::cerr << "WARNING: singleLayerPixelInterleaving is only for lossy "
                   "coding mode for now. Force singleMapPixelInterleaving_=FALSE.\n";
    }
    if ( lossyRawPointsPatch_ ) {
      lossyRawPointsPatch_ = false;
      std::cerr << "WARNING: lossyRawPointsPatch_ is only for lossy coding "
                   "mode for now. Force lossyRawPointsPatch_=FALSE.\n";
    }
  } else {
    if ( enhancedOccupancyMapCode_ ) {
      enhancedOccupancyMapCode_ = false;
      std::cerr << "WARNING: enhancedOccupancyMapCode_ is only for lossless "
                   "coding mode for now. Force enhancedOccupancyMapCode_=FALSE.\n";
    }
  }

  if ( enhancedOccupancyMapCode_ && surfaceThickness_ == 1 ) {
    std::cerr << "WARNING: EOM code doesn't bring any gain when surfaceThickness==1."
                 "Please consider to increase the value of surfaceThickness.\n";
  }

  if ( enhancedOccupancyMapCode_ && ( thresholdLossyOM_ > 0 ) ) {
    std::cerr << "WARNING: When using enhanced occupancy map for delta depth, "
                 "thresholdOccupancyMap should be equal to 0\n";
    std::cerr << "         Forcing thresholdLossyOM_ to 0\n";
    thresholdLossyOM_ = 0;
  }

  if ( enhancedOccupancyMapCode_ && ( offsetLossyOM_ > 0 ) ) {
    std::cerr << "WARNING: When using enhanced occupancy map for delta depth, "
                 "offsetOccupancyMap should be equal to 0\n";
    std::cerr << "         Forcing offsetLossyOM_ to 0\n";
    offsetLossyOM_ = 0;
  }

  if ( enhancedOccupancyMapCode_ && ( prefilterLossyOM_ ) ) {
    std::cerr << "WARNING: When using enhanced occupancy map for delta depth, "
                 "prefilterLossyOM_ should be equal to false\n";
    std::cerr << "         Forcing prefilterLossyOM_ to false\n";
    prefilterLossyOM_ = false;
  }

  if ( useRawPointsSeparateVideo_ && !lossyRawPointsPatch_ && !losslessGeo_ ) {
    useRawPointsSeparateVideo_ = false;
    std::cerr << "WARNING: useRawPointsSeparateVideo_ is for lossy coding mode "
                 "if lossyRawPointsPatch_. Force "
                 "useRawPointsSeparateVideo_=false.\n";
  }
  if ( useRawPointsSeparateVideo_ ) {
    if ( ( textureRawSeparateVideoWidth_ % 64 ) != 0U ) {
      ret = false;
      std::cerr << "textureRawSeparateVideoWidth_ must be multiple of 64.\n";
    }
    if ( singleMapPixelInterleaving_ ) {
      ret = false;
      std::cerr << "Pixel Interleaving is built on one layer coding. Force "
                   "mapCountMinus1_ = 0.\n";
    }
    if ( geometryAuxVideoConfig_.empty() || !exist( geometryAuxVideoConfig_ ) ) {
      if ( !geometryConfig_.empty() ) {
        std::cerr << "WARNING: geometryAuxVideoConfig_ is set as geometryConfig_ : " << geometryConfig_ << std::endl;
        geometryAuxVideoConfig_ = geometryConfig_;
      } else {
        ret = false;
        std::cerr << "geometryConfig_ and geometryAuxVideoConfig_ are empty\n";
      }
    } else {
      std::cout << "geometryAuxConfig: " << geometryAuxVideoConfig_ << std::endl;
    }
    if ( textureAuxVideoConfig_.empty() || !exist( textureAuxVideoConfig_ ) ) {
      if ( !textureConfig_.empty() ) {
        std::cerr << "WARNING: textureAuxVideoConfig_ is set as textureConfig_ : " << textureConfig_ << std::endl;
        textureAuxVideoConfig_ = textureConfig_;
      } else {
        ret = false;
        std::cerr << "textureConfig_ and textureAuxVideoConfig_ are empty\n";
      }

    } else
      std::cout << "textureAuxConfig: " << textureAuxVideoConfig_ << std::endl;
  }

  if ( singleMapPixelInterleaving_ && pointLocalReconstruction_ ) {
    ret = false;
    std::cerr << "Pixel Interleaving and Point local reconstruction cna't be "
                 "use in the same time.\n";
  }

  if ( mapCountMinus1_ != 0 ) {
    if ( singleMapPixelInterleaving_ ) {
      ret = false;
      std::cerr << "Pixel Interleaving is built on one layer coding. Force "
                   "mapCountMinus1_ = 0.\n";
    }
    if ( pointLocalReconstruction_ ) {
      ret = false;
      std::cerr << "Point local reconstruction is built on one layer coding. "
                   "Force mapCountMinus1_ = 0.\n";
    }
  } else {
    if ( multipleStreams_ ) {
      std::cout << " multipleStreams is set 0 when mapCountMinus1 is 0. Force "
                   "multipleStreams_= 0."
                << std::endl;
      multipleStreams_ = false;
    }
  }

  if ( occupancyMapVideoEncoderConfig_.empty() ) {
    ret = false;
    std::cerr << "to use segmentation, you must define a segmentationDataPath \n";
  }
  if ( lossyRawPointsPatch_ ) {
    if ( !useRawPointsSeparateVideo_ ) {
      std::cerr << "Lossy raw points patch in the same video frame as the "
                   "regular patches is "
                   "not optimized as of now.\n";
    }
    if ( ( minNormSumOfInvDist4MPSelection_ < 0.0 ) || ( minNormSumOfInvDist4MPSelection_ > 1.0 ) ) {
      ret = false;
      std::cerr << "minNormSumOfInvDist4MPSelection must be between 0.0 and "
                   "1.0 (inclusive)\n";
    }
  }

  if ( tileSegmentationType_ == 0 ) {
    numMaxTilePerFrame_ = 1;
  } else if ( tileSegmentationType_ == 1 ) {
    if ( !enablePointCloudPartitioning_ ) {
      std::cerr << "enablePointCloudPartitioning should be true when tileSegmentationType=0\n";
      enablePointCloudPartitioning_ = true;
    }
  } else if ( tileSegmentationType_ == 2 ) {
    if ( numMaxTilePerFrame_ != 3 ) {
      numMaxTilePerFrame_ = 3;
      std::cerr << "current version allows numMaxTilePerFrame_=3 only\n";
    };

    if ( numMaxTilePerFrame_ > 1 ) {
      if ( tilePartitionWidth_ == 0 || tilePartitionHeight_ == 0 ) {
        std::cerr << "tilePartitionWidth/Height should be greater than 0 : set as 1x1\n";
        tilePartitionWidth_  = 1;
        tilePartitionHeight_ = 1;
      }
    }
  }
  if ( flagGeometrySmoothing_ ) {
    if ( pbfEnableFlag_ ) {
      gridSmoothing_ = false;
      if ( pbfPassesCount_ == 0 ) { pbfPassesCount_ = occupancyPrecision_ <= 2 ? 1 : occupancyPrecision_ == 4 ? 2 : 4; }
      if ( pbfFilterSize_ == 0 ) { pbfFilterSize_ = occupancyPrecision_; }
    }
    if ( gridSmoothing_ ) {
      if ( gridSize_ == 0 ) {
        ret = false;
        std::cerr << "gridSize shall be greater than 0. \n";
      }
      if ( gridSize_ % 2 == 1 ) { std::cerr << "WARNING: gridSize should be an even number\n"; }
    }
  }
  if ( flagColorSmoothing_ ) {
    if ( cgridSize_ == 0 ) {
      ret = false;
      std::cerr << "color gridSize shall be greater than 0. \n";
    }
    if ( cgridSize_ % 2 == 1 ) { std::cerr << "WARNING: color gridSize should be an even number\n"; }
  }
  if ( EOMFixBitCount_ < 1 ) {
    ret = false;
    std::cerr << "EOMFixBitCount shall be greater than 0. \n";
  }
  return ret;
}

void PCCEncoderParameters::constructAspsRefListStruct( PCCContext& context, size_t aspsIdx, size_t afpsIdx ) {
  auto& asps = context.getAtlasSequenceParameterSet( aspsIdx );
  for ( size_t list = 0; list < context.getNumOfRefAtlasFrameList(); list++ ) {
    RefListStruct refList;
    refList.setNumRefEntries( context.getMaxNumRefAtlasFrame( list ) );
    refList.allocate();
    for ( size_t i = 0; i < refList.getNumRefEntries(); i++ ) {
      int afocDiff = -1;
      if ( i == 0 )
        afocDiff = context.getRefAtlasFrame( list, i );
      else
        afocDiff = context.getRefAtlasFrame( list, i ) - context.getRefAtlasFrame( list, i - 1 );
      refList.setAbsDeltaAfocSt( i, std::abs( afocDiff ) );
      refList.setStrafEntrySignFlag( i, afocDiff < 0 ? false : !false );
      refList.setStRefAtalsFrameFlag( i, true );
    }
    asps.addRefListStruct( refList );
  }
}

void PCCEncoderParameters::initializeContext( PCCContext& context ) {
  size_t  numAtlas   = 1;
  size_t  atlasIndex = 0;
  auto&   vps        = context.getVps();
  uint8_t bitdepth3D = uint8_t( geometry3dCoordinatesBitdepth_ + ( additionalProjectionPlaneMode_ > 0 ) );

  // Allocation
  context.resizeAtlas( numAtlas );  // single atlas for V3C
  vps.setAtlasCountMinus1( numAtlas - 1 );
  vps.allocateAtlas();
  context.allocateAtlasHLS( vps.getAtlasCountMinus1() + 1 );
  context.setAtlasIndex( atlasIndex );

  // Context
  context.setLog2PatchQuantizerSizeX( log2QuantizerSizeX_ );
  context.setLog2PatchQuantizerSizeY( log2QuantizerSizeY_ );
  context.setLog2MaxAtlasFrameOrderCntLsb( log2MaxAtlasFrameOrderCntLsb_ );
  context.setOffsetLossyOM( offsetLossyOM_ );
  context.setPrefilterLossyOM( prefilterLossyOM_ );
  context.setOccupancyPrecision( occupancyPrecision_ );
  context.setModelScale( modelScale_ );
  context.setModelOrigin( modelOrigin_ );
  context.setAuxVideoWidth( textureRawSeparateVideoWidth_ );
  context.setGeometry3dCoordinatesBitdepth( bitdepth3D );
  context.setMaxNumRefAtlasFrame( maxNumRefAtlasFrame_ );
  context.setNumOfRefAtlasFrameList( maxNumRefAtlasList_ );
  context.setEnablePatchSizeQuantization(
      ( ( 1 << static_cast<int32_t>( log2QuantizerSizeX_ ) ) < static_cast<int32_t>( occupancyPrecision_ ) ) ||
      ( ( 1 << static_cast<int32_t>( log2QuantizerSizeY_ ) ) < static_cast<int32_t>( occupancyPrecision_ ) ) );
  for ( size_t list = 0; list < maxNumRefAtlasList_; list++ ) {
    context.setSizeOfRefAtlasFrameList( list, maxNumRefAtlasFrame_ );
    for ( size_t i = 0; i < maxNumRefAtlasFrame_; i++ ) {
      context.setRefAtlasFrame( list, i, static_cast<int32_t>( i + 1 ) );  // 1, 2, 3, 4
    }
  }
  size_t numPlrm = pointLocalReconstruction_
                       ? ( std::max )( static_cast<size_t>( 1 ),
                                       ( std::min )( plrlNumberOfModes_, g_pointLocalReconstructionMode.size() ) )
                       : 1;
  for ( size_t i = 0; i < numPlrm; i++ ) {
    context.addPointLocalReconstructionMode( g_pointLocalReconstructionMode[i] );
  }

  // V3C parameter set
  vps.setMapCountMinus1( atlasIndex, static_cast<uint32_t>( mapCountMinus1_ ) );
  vps.setMultipleMapStreamsPresentFlag( atlasIndex, mapCountMinus1_ != 0 && multipleStreams_ );
  vps.setAuxiliaryVideoPresentFlag( atlasIndex, useRawPointsSeparateVideo_ );
  vps.setOccupancyVideoPresentFlag( atlasIndex, true );
  vps.setGeometryVideoPresentFlag( atlasIndex, true );
  vps.setAttributeVideoPresentFlag( atlasIndex, true );
  for ( size_t i = 0; i < mapCountMinus1_ + 1; i++ ) {
    if ( i == 0 ) {
      vps.setMapAbsoluteCodingEnableFlag( atlasIndex, i, true );
      vps.setMapPredictorIndexDiff( atlasIndex, i, false );
    } else {
      vps.setMapAbsoluteCodingEnableFlag( atlasIndex, i, absoluteD1_ );
      vps.setMapPredictorIndexDiff( atlasIndex, i, false );
    }
  }

  // Atlas sequence parameter set
  auto& asps = context.addAtlasSequenceParameterSet( 0 );
  asps.setLog2PatchPackingBlockSize( std::log2( occupancyResolution_ ) );
  asps.setLog2MaxAtlasFrameOrderCntLsbMinus4( log2MaxAtlasFrameOrderCntLsb_ - 4 );
  asps.setMaxDecAtlasFrameBufferingMinus1( 0 );
  asps.setNumRefAtlasFrameListsInAsps( maxNumRefAtlasList_ );
  asps.setMapCountMinus1( mapCountMinus1_ );
  asps.setLongTermRefAtlasFramesFlag( false );
  asps.setUseEightOrientationsFlag( useEightOrientations_ );
  asps.setExtendedProjectionEnabledFlag( additionalProjectionPlaneMode_ > 0 );
  asps.setMaxNumberProjectionsMinus1( 5 + 4 * ( std::min )( additionalProjectionPlaneMode_, 3 ) );
  asps.setNormalAxisLimitsQuantizationEnabledFlag( true );
  asps.setNormalAxisMaxDeltaValueEnabledFlag( true );
  asps.setxelDeinterleavingFlag( singleMapPixelInterleaving_ );
  asps.setPatchPrecedenceOrderFlag( patchPrecedenceOrderFlag_ );
  asps.setPatchSizeQuantizerPresentFlag( context.getEnablePatchSizeQuantization() );
  asps.setEomPatchEnabledFlag( enhancedOccupancyMapCode_ );
  asps.setPLREnabledFlag( pointLocalReconstruction_ );
  asps.setVuiParametersPresentFlag( false );
  asps.setExtensionFlag( true );
  asps.setVpccExtensionFlag( true );
  asps.setExtension7Bits( 0 );
  asps.setAuxiliaryVideoEnabledFlag( useRawPointsSeparateVideo_ );
  asps.setRawPatchEnabledFlag( losslessGeo_ || lossyRawPointsPatch_ );
  if ( asps.getVpccExtensionFlag() ) {
    auto& ext = asps.getAspsVpccExtension();
    ext.setRemoveDuplicatePointEnableFlag( removeDuplicatePoints_ );
    ext.setSurfaceThicknessMinus1( surfaceThickness_ - 1 );
  }
  asps.setEomFixBitCountMinus1( EOMFixBitCount_ - 1 );
  asps.setGeometry2dBitdepthMinus1( uint8_t( geometryNominal2dBitdepth_ - 1 ) );
  asps.setGeometry3dBitdepthMinus1( bitdepth3D - 1 );
  // NOTE JR: decoder/reader must used asps value and the reconstruction must used gi value.

  // Atlas frame parameter set
  auto& afps = context.addAtlasFrameParameterSet( 0 );
  afps.setAtlasSequenceParameterSetId( 0 );
  afps.setNumRefIdxDefaultActiveMinus1( static_cast<uint8_t>(
      constrainedPack_ ? ( ( std::max )( 0, static_cast<int>( maxNumRefAtlasFrame_ ) - 1 ) ) : 0 ) );
  afps.setAdditionalLtAfocLsbLen( 4 );
  afps.setRaw3dOffsetBitCountExplicitModeFlag( false );
  afps.setExtensionFlag( true );
  afps.setExtension8Bits( 0 );
  constructAspsRefListStruct( context, 0, 0 );

  // occupancy information
  auto& oi = vps.getOccupancyInformation( atlasIndex );
  oi.setLossyOccupancyCompressionThreshold( thresholdLossyOM_ );
  oi.setOccupancy2DBitdepthMinus1( 7 );
  oi.setOccupancyMSBAlignFlag( false );
  oi.setOccupancyCodecId( videoEncoderOccupancyCodecId_ );

  // geometry information
  auto& gi = vps.getGeometryInformation( atlasIndex );
  gi.setGeometry3dCoordinatesBitdepthMinus1( bitdepth3D - 1 );
  gi.setGeometry2dBitdepthMinus1( uint8_t( geometryNominal2dBitdepth_ - 1 ) );
  gi.setGeometryMSBAlignFlag( false );
  gi.setGeometryCodecId( videoEncoderGeometryCodecId_ );

  // Attribute information
  auto& ai = vps.getAttributeInformation( atlasIndex );
  ai.setAttributeCount( noAttributes_ ? 0 : 1 );
  ai.allocate();
  if ( static_cast<int>( noAttributes_ ) == 0 ) {
    ai.setAttributeDimensionMinus1( 0, noAttributes_ ? 0 : 2 );
    ai.setAttribute2dBitdepthMinus1( 0, 7 );
  }
  for ( size_t i = 0; i < ai.getAttributeCount(); i++ ) {
    if ( absoluteT1_ == absoluteD1_ ) {
      ai.setAttributeMapAbsoluteCodingPersistenceFlag( i, false );
    } else if ( absoluteT1_ && !absoluteD1_ ) {
      ai.setAttributeMapAbsoluteCodingPersistenceFlag( i, true );
    } else {
      std::cerr << "absoluteT1_ should be true when absoluteD1_ is true\n";
      exit( 0 );
    }
  }
  for ( uint32_t i = 0; i < ai.getAttributeCount(); i++ ) {
    ai.setAttributeCodecId( i, videoEncoderAttributeCodecId_ );
  }

  // atlas video frame allocation
  context.getAtlas( atlasIndex ).allocateVideoFrames( context, 0 );

  // Profile tier level
  auto& plt = vps.getProfileTierLevel();
  plt.setProfileCodecGroupIdc( CODEC_GROUP_HEVC_MAIN10 );
  if ( losslessGeo_ ) { plt.setProfileCodecGroupIdc( CODEC_GROUP_HEVC444 ); }

  // Tiles
  auto& frames = context.getFrames();
  for ( size_t i = 0; i < frames.size(); i++ ) {
    auto& atlas = frames[i];
    auto& frame = atlas.getTitleFrameContext();
    frame.setFrameIndex( i );
    frame.setRawPatchEnabledFlag( losslessGeo_ || lossyRawPointsPatch_ );
    frame.setUseRawPointsSeparateVideo( useRawPointsSeparateVideo_ );
    frame.setGeometry3dCoordinatesBitdepth( bitdepth3D );
    frame.setGeometry2dBitdepth( geometryNominal2dBitdepth_ );
    frame.setMaxDepth( ( 1 << geometryNominal2dBitdepth_ ) - 1 );
    frame.setLog2PatchQuantizerSizeX( context.getLog2PatchQuantizerSizeX() );
    frame.setLog2PatchQuantizerSizeY( context.getLog2PatchQuantizerSizeY() );
    frame.setAtlasFrmOrderCntLsb( context.calculateAFOCLsb(
        i ) );  // ajt:: lsb and afoc values of tileFrame is set during the initilaization process
    frame.setAtlasFrmOrderCntVal( i );
    if ( i == 0 ) {
      frame.setNumRefIdxActive( 0 );
    } else {
      frame.setNumRefIdxActive( constrainedPack_ ? ( std::min )( i, maxNumRefAtlasFrame_ ) : 0 );
    }
    frame.setRefAfocList( context, 0 );
    if ( tileSegmentationType_ == 1 ) {
      size_t partitionWidthIn64  = ( minimumImageWidth_ / ( 64 * numTilesHor_ ) );
      size_t partitionHeightIn64 = ( tileHeightToWidthRatio_ * minimumImageWidth_ / ( 64 * numTilesHor_ ) );
      atlas.initPartitionInfoPerFrame( i, minimumImageWidth_, minimumImageHeight_, 0, uniformPartitionSpacing_,
                                       partitionWidthIn64, partitionHeightIn64 );
    } else {
      if ( useRawPointsSeparateVideo_ ) {
        context.getAuxTileHeight().resize( numMaxTilePerFrame_ );
        context.getAuxTileLeftTopY().resize( numMaxTilePerFrame_ );
      }
      atlas.setNumTilesInAtlasFrame( numMaxTilePerFrame_ );
      atlas.initNumTiles( numMaxTilePerFrame_ );
      atlas.initPartitionInfoPerFrame( i, minimumImageWidth_, minimumImageHeight_, numMaxTilePerFrame_,
                                       uniformPartitionSpacing_, tilePartitionWidth_, tilePartitionHeight_ );
      for ( size_t ti = 0; ti < numMaxTilePerFrame_; ti++ ) {
        auto& tile = atlas[ti];
        tile.setRawPatchEnabledFlag( losslessGeo_ || lossyRawPointsPatch_ );
        tile.setUseRawPointsSeparateVideo( useRawPointsSeparateVideo_ );
        tile.setGeometry3dCoordinatesBitdepth( bitdepth3D );
        tile.setGeometry2dBitdepth( geometryNominal2dBitdepth_ );
        tile.setMaxDepth( ( 1 << geometryNominal2dBitdepth_ ) - 1 );
        tile.setLog2PatchQuantizerSizeX( context.getLog2PatchQuantizerSizeX() );
        tile.setLog2PatchQuantizerSizeY( context.getLog2PatchQuantizerSizeY() );
        tile.setAtlasFrmOrderCntLsb( context.calculateAFOCLsb( i ) );
        tile.setAtlasFrmOrderCntVal( i );
        tile.setFrameIndex( i );
        if ( i == 0 ) {
          tile.setNumRefIdxActive( 0 );
        } else {
          tile.setNumRefIdxActive( constrainedPack_ ? ( std::min )( i, maxNumRefAtlasFrame_ ) : 0 );
        }
        tile.setGeometry2dBitdepth( geometryNominal2dBitdepth_ );
        tile.setMaxDepth( ( 1 << geometryNominal2dBitdepth_ ) - 1 );
        tile.setLog2PatchQuantizerSizeX( context.getLog2PatchQuantizerSizeX() );
        tile.setLog2PatchQuantizerSizeY( context.getLog2PatchQuantizerSizeY() );
        tile.setNumRefIdxActive( std::min( i, maxNumRefAtlasFrame_ ) );
        tile.setRefAfocList( context, 0 );
      }
    }
  }
}
