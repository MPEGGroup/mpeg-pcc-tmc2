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
using namespace pcc;

const std::vector<PointLocalReconstructionMode> g_pointLocalReconstructionMode = {
    {0, 0, 0, 1}, {1, 0, 0, 1}, {1, 1, 0, 1}, {1, 0, 0, 2}, {1, 1, 0, 2},
    {0, 0, 1, 1}, {1, 0, 1, 1}, {1, 1, 1, 1}, {1, 0, 1, 2}, {1, 1, 1, 2},
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
  gridBasedRefineSegmentation_             = true;
  maxNNCountRefineSegmentation_            = gridBasedRefineSegmentation_ ? 1024 : 256;
  iterationCountRefineSegmentation_        = gridBasedRefineSegmentation_ ? 10 : 100;
  voxelDimensionRefineSegmentation_        = 4;
  searchRadiusRefineSegmentation_          = 192;
  occupancyResolution_                     = 16;
  log2QuantizerSizeX_                      = 4;
  log2QuantizerSizeY_                      = 4;
  minPointCountPerCCPatchSegmentation_     = 16;
  maxNNCountPatchSegmentation_             = 16;
  surfaceThickness_                        = 4;
  minLevel_                                = 64;  // fix value
  maxAllowedDist2MissedPointsDetection_    = 9.0;
  maxAllowedDist2MissedPointsSelection_    = 1.0;
  lambdaRefineSegmentation_                = 3.0;
  minimumImageWidth_                       = 1280;
  minimumImageHeight_                      = 1280;
  maxCandidateCount_                       = 4;
  occupancyPrecision_                      = 4;
  occupancyMapVideoEncoderConfig_          = {};
  occupancyMapQP_                          = 8;
  occupancyMapRefinement_                  = false;
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
  videoEncoderPath_                        = {};
  videoEncoderAuxPath_                     = {};
  videoEncoderOccupancyMapPath_            = {};
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
  useMissedPointsSeparateVideo_            = false;
  geometryMPConfig_                        = {};
  textureMPConfig_                         = {};
  nbThread_                                = 1;
  keepIntermediateFiles_                   = false;

  absoluteD1_                             = true;
  absoluteT1_                             = true;
  multipleStreams_                        = false;
  qpAdjT1_                                = 0;
  qpAdjD1_                                = 0;
  constrainedPack_                        = true;
  thresholdColorSmoothing_                = 10.0;
  thresholdColorDifference_               = 100.0;
  thresholdColorVariation_                = 6.0;
  thresholdLocalEntropy_                  = 4.0;
  radius2ColorSmoothing_                  = 4.0 * 16;
  neighborCountColorSmoothing_            = 4 * 16;
  flagColorSmoothing_                     = false;
  gridColorSmoothing_                     = false;
  cgridSize_                              = 4;
  thresholdColorPreSmoothing_             = 10.0;
  thresholdColorPreSmoothingLocalEntropy_ = 4.5;
  radius2ColorPreSmoothing_               = 4.0 * 16;
  neighborCountColorPreSmoothing_         = 4 * 16;
  flagColorPreSmoothing_                  = true;
  groupDilation_                          = true;
  textureDilationOffLossless_             = true;
  enhancedDeltaDepthCode_                 = false;
  EOMFixBitCount_                         = 2;
  offsetLossyOM_                          = 0;
  thresholdLossyOM_                       = 0;
  prefilterLossyOM_                       = false;
  patchColorSubsampling_                  = false;
  deltaCoding_                            = true;
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
  geometryPadding_      = false;

  // lossy missed points patch
  lossyMissedPointsPatch_          = false;
  minNormSumOfInvDist4MPSelection_ = 0.35;
  lossyMppGeoQP_                   = 4;
  // GPA
  globalPatchAllocation_ = 0;
  // GTP
  globalPackingStrategyGOF_         = 0;
  globalPackingStrategyReset_       = false;
  globalPackingStrategyThreshold_   = 0;
  use3dmc_                          = true;
  enhancedPP_                       = true;
  minWeightEPP_                     = 0.6;
  geometry3dCoordinatesBitdepth_    = 10;
  additionalProjectionPlaneMode_    = 0;
  partialAdditionalProjectionPlane_ = 0.00;

  // 3D and 2D bit depths
  geometry3dCoordinatesBitdepth_ = 10;
  geometryNominal2dBitdepth_     = 8;

  // Partitions and tiles
  enablePointCloudPartitioning_ = false;
  numTilesHor_                  = 2;
  tileHeightToWidthRatio_       = 1;

  // Sort missed points by Morton code
  mortonOrderSortMissedPoints_ = false;
  textureMPSeparateVideoWidth_ = 64;

  // Separate high gradient points
  highGradientSeparation_   = false;
  minGradient_              = 15.0;
  minNumHighGradientPoints_ = 256;

  // Patch border filtering
  pbfEnableFlag_    = 0;
  pbfPassesCount_   = 0;
  pbfFilterSize_    = 0;
  pbfLog2Threshold_ = 2;

  patchPrecedenceOrderFlag_ = false;
  maxNumRefAtlasList_       = 1;
  maxNumRefAtlasFrame_      = 1;

  // qunatizer;
}

PCCEncoderParameters::~PCCEncoderParameters() {}

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
    if ( !inverseColorSpaceConversionConfig_.empty() ) {
      inverseColorSpaceConversionConfig_ = configurationFolder_ + inverseColorSpaceConversionConfig_;
    }
    if ( !colorSpaceConversionConfig_.empty() ) {
      colorSpaceConversionConfig_ = configurationFolder_ + colorSpaceConversionConfig_;
    }
    if ( !occupancyMapVideoEncoderConfig_.empty() ) {
      occupancyMapVideoEncoderConfig_ = configurationFolder_ + occupancyMapVideoEncoderConfig_;
    }
    if ( useMissedPointsSeparateVideo_ ) {
      if ( !geometryMPConfig_.empty() ) { geometryMPConfig_ = configurationFolder_ + geometryMPConfig_; }
      if ( !textureMPConfig_.empty() ) { textureMPConfig_ = configurationFolder_ + textureMPConfig_; }
    }
  }
}

void PCCEncoderParameters::print() {
  std::cout << "+ Parameters" << std::endl;
  std::cout << "\t losslessGeo                              " << losslessGeo_ << std::endl;
  std::cout << "\t noAttributes                             " << noAttributes_ << std::endl;
  std::cout << "\t raw Geometry Colour Plane                " << ( losslessGeo444_ ? "444" : "420" ) << std::endl;
  std::cout << "\t enhancedDeltaDepthCode                   " << enhancedDeltaDepthCode_ << std::endl;
  std::cout << "\t useMissedPointsSeparateVideo             " << useMissedPointsSeparateVideo_ << std::endl;
  std::cout << "\t textureMPSeparateVideoWidth              " << textureMPSeparateVideoWidth_ << std::endl;
  std::cout << "\t uncompressedDataPath                     " << uncompressedDataPath_ << std::endl;
  std::cout << "\t compressedStreamPath                     " << compressedStreamPath_ << std::endl;
  std::cout << "\t reconstructedDataPath                    " << reconstructedDataPath_ << std::endl;
  std::cout << "\t frameCount                               " << frameCount_ << std::endl;
  std::cout << "\t mapCountMinus1                          " << mapCountMinus1_ << std::endl;
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
  std::cout << "\t deltaCoding                              " << deltaCoding_ << std::endl;
  std::cout << "\t  maxNumRefPatchList                       " << maxNumRefAtlasList_ << std::endl;
  std::cout << "\t maxNumRefIndex                           " << maxNumRefAtlasFrame_ << std::endl;
  std::cout << "\t Segmentation" << std::endl;
  std::cout << "\t   nnNormalEstimation                     " << nnNormalEstimation_ << std::endl;
  std::cout << "\t   gridBasedRefineSegmentation            " << gridBasedRefineSegmentation_ << std::endl;
  std::cout << "\t   maxNNCountRefineSegmentation           " << maxNNCountRefineSegmentation_ << std::endl;
  std::cout << "\t   iterationCountRefineSegmentation       " << iterationCountRefineSegmentation_ << std::endl;
  std::cout << "\t   voxelDimensionRefineSegmentation       " << voxelDimensionRefineSegmentation_ << std::endl;
  std::cout << "\t   searchRadiusRefineSegmentation         " << searchRadiusRefineSegmentation_ << std::endl;
  std::cout << "\t   occupancyResolution                    " << occupancyResolution_ << std::endl;
  std::cout << "\t   quantization step for patch size       "
            << "1<<" << log2QuantizerSizeX_ << ", 1<<" << log2QuantizerSizeY_ << std::endl;
  std::cout << "\t   minPointCountPerCCPatchSegmentation    " << minPointCountPerCCPatchSegmentation_ << std::endl;
  std::cout << "\t   maxNNCountPatchSegmentation            " << maxNNCountPatchSegmentation_ << std::endl;
  std::cout << "\t   surfaceThickness                       " << surfaceThickness_ << std::endl;
  std::cout << "\t   maxAllowedDist2MissedPointsDetection " << maxAllowedDist2MissedPointsDetection_ << std::endl;
  std::cout << "\t   maxAllowedDist2MissedPointsSelection " << maxAllowedDist2MissedPointsSelection_ << std::endl;
  std::cout << "\t   lambdaRefineSegmentation               " << lambdaRefineSegmentation_ << std::endl;
  std::cout << "\t   depthQuantizationStep                  " << minLevel_ << std::endl;
  std::cout << "\t   highGradientSeparation                 " << highGradientSeparation_ << std::endl;
  if ( highGradientSeparation_ ) {
    std::cout << "\t     minGradient                          " << minGradient_ << std::endl;
    std::cout << "\t     minNumHighGradientPoints             " << minNumHighGradientPoints_ << std::endl;
  }
  std::cout << "\t Packing" << std::endl;
  std::cout << "\t   minimumImageWidth                      " << minimumImageWidth_ << std::endl;
  std::cout << "\t   minimumImageHeight                     " << minimumImageHeight_ << std::endl;
  std::cout << "\t   packingStrategy                        " << packingStrategy_ << std::endl;
  std::cout << "\t   useEightOrientations                   " << useEightOrientations_ << std::endl;
  std::cout << "\t   safeGuardDistance                      " << safeGuardDistance_ << std::endl;
  std::cout << "\t   globalPatchAllocation                  " << globalPatchAllocation_ << std::endl;
  if ( globalPatchAllocation_ == 2 ) {
    std::cout << "\t   globalPackingStrategyGOF           " << geometryD0Config_ << std::endl;
    std::cout << "\t   globalPackingStrategyReset         " << geometryD1Config_ << std::endl;
    std::cout << "\t   globalPackingStrategyThreshold     " << geometryD1Config_ << std::endl;
  }
  std::cout << "\t   patchPrecedenceOrder                   " << patchPrecedenceOrderFlag_ << std::endl;
  std::cout << "\t   lowDelayEncoding                       " << lowDelayEncoding_ << std::endl;
  std::cout << "\t   textureBGFill                          " << textureBGFill_ << std::endl;
  std::cout << "\t   geometryPadding                        " << geometryPadding_ << std::endl;
  std::cout << "\t Video encoding" << std::endl;
  std::cout << "\t   geometryQP                             " << geometryQP_ << std::endl;
  std::cout << "\t   textureQP                              " << textureQP_ << std::endl;
  std::cout << "\t   colorSpaceConversionPath               " << colorSpaceConversionPath_ << std::endl;
  std::cout << "\t   videoEncoderPath                       " << videoEncoderPath_ << std::endl;
  std::cout << "\t   videoEncoderAuxPath                    " << videoEncoderAuxPath_ << std::endl;
  std::cout << "\t   videoEncoderOccupancyMapPath           " << videoEncoderOccupancyMapPath_ << std::endl;
  if ( multipleStreams_ ) {
    std::cout << "\t   geometryD0Config                       " << geometryD0Config_ << std::endl;
    std::cout << "\t   geometryD1Config                       " << geometryD1Config_ << std::endl;
  } else {
    std::cout << "\t   geometryConfig                         " << geometryConfig_ << std::endl;
  }
  if ( multipleStreams_ ) {
    std::cout << "\t   textureT0Config                         " << textureT0Config_ << std::endl;
    std::cout << "\t   textureT1Config                         " << textureT1Config_ << std::endl;
  } else
    std::cout << "\t   textureConfig                          " << textureConfig_ << std::endl;
  if ( useMissedPointsSeparateVideo_ ) {
    if ( losslessGeo_ ) {
      std::cout << "\t geometryMPConfig                          " << geometryMPConfig_ << std::endl;
      std::cout << "\t textureMPConfig                            " << textureMPConfig_ << std::endl;
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
  std::cout << "\t Geometry smoothing                       " << std::endl;
  std::cout << "\t   flagGeometrySmoothing                  " << flagGeometrySmoothing_ << std::endl;
  if ( flagGeometrySmoothing_ ) {
    std::cout << "\t   gridSmoothing                          " << gridSmoothing_ << std::endl;
    if ( gridSmoothing_ ) {
      std::cout << "\t   gridSize                               " << gridSize_ << std::endl;
      std::cout << "\t   thresholdSmoothing                     " << thresholdSmoothing_ << std::endl;
    } else {
      std::cout << "\t   neighborCountSmoothing                 " << neighborCountSmoothing_ << std::endl;
      std::cout << "\t   radius2Smoothing                       " << radius2Smoothing_ << std::endl;
      std::cout << "\t   radius2BoundaryDetection               " << radius2BoundaryDetection_ << std::endl;
      std::cout << "\t   thresholdSmoothing                     " << thresholdSmoothing_ << std::endl;
    }
  }
  std::cout << "\t   patchExpansion                         " << patchExpansion_ << std::endl;
  std::cout << "\t Color smoothing" << std::endl;
  std::cout << "\t   flagColorSmoothing                       " << flagColorSmoothing_ << std::endl;
  if ( flagColorSmoothing_ ) {
    std::cout << "\t   gridColorSmoothing                       " << gridColorSmoothing_ << std::endl;
    if ( gridColorSmoothing_ ) {
      std::cout << "\t   thresholdColorSmoothing                " << thresholdColorSmoothing_ << std::endl;
      std::cout << "\t   thresholdColorDifference               " << thresholdColorDifference_ << std::endl;
      std::cout << "\t   thresholdColorVariation                " << thresholdColorVariation_ << std::endl;
      std::cout << "\t   thresholdLocalEntropy                  " << thresholdLocalEntropy_ << std::endl;
      std::cout << "\t   cgridSize                              " << cgridSize_ << std::endl;
    } else {
      std::cout << "\t   thresholdColorSmoothing                " << thresholdColorSmoothing_ << std::endl;
      std::cout << "\t   thresholdLocalEntropy                  " << thresholdLocalEntropy_ << std::endl;
      std::cout << "\t   radius2ColorSmoothing                  " << radius2ColorSmoothing_ << std::endl;
      std::cout << "\t   neighborCountColorSmoothing            " << neighborCountColorSmoothing_ << std::endl;
    }
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
    std::cout << "\t     thresholdColorOutlierDist            " << thresholdColorOutlierDist_ << std::endl;
  }
  std::cout << "\t Reconstruction " << std::endl;
  std::cout << "\t   removeDuplicatePoints                  " << removeDuplicatePoints_ << std::endl;
  std::cout << "\t   pointLocalReconstruction               " << pointLocalReconstruction_ << std::endl;
  std::cout << "\t     plrlNumberOfModes                    " << plrlNumberOfModes_ << std::endl;
  std::cout << "\t     patchSize                            " << patchSize_ << std::endl;
  std::cout << "\t   singleLayerPixelInterleaving           " << singleMapPixelInterleaving_ << std::endl;
  std::cout << "\t surface Separation                       " << surfaceSeparation_ << std::endl;
  std::cout << "\t Lossy missed points patch                " << std::endl;
  std::cout << "\t   lossyMissedPointsPatch                 " << lossyMissedPointsPatch_ << std::endl;
  std::cout << "\t   minNormSumOfInvDist4MPSelection        " << minNormSumOfInvDist4MPSelection_ << std::endl;
  std::cout << "\t   lossyMppGeoQP                          " << lossyMppGeoQP_ << std::endl;
  std::cout << "\t Missed points sorting                    " << std::endl;
  std::cout << "\t   mortonOrderSortMissedPoints            " << mortonOrderSortMissedPoints_ << std::endl;
  std::cout << "\t Enhanced projection plane                " << enhancedPP_ << std::endl;
  std::cout << "\t AdditionalProjectionPlane                " << std::endl;
  std::cout << "\t   additionalProjectionPlaneMode          " << additionalProjectionPlaneMode_ << std::endl;
  std::cout << "\t   partialAdditionalProjectionPlane       " << partialAdditionalProjectionPlane_ << std::endl;
  std::cout << "\t Geometry 2D and 3D bitdepths             " << std::endl;
  std::cout << "\t   geometry3dCoordinatesBitdepth          " << geometry3dCoordinatesBitdepth_ << std::endl;
  std::cout << "\t   geometryNominal2dBitdepth              " << geometryNominal2dBitdepth_ << std::endl;
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
        std::cerr << "All the 6 arrays roiBoundingBox[Min-Max][X-Y-Z] must have the same number of elements."
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
  if ( videoEncoderPath_.empty() ) {
    ret = false;
    std::cerr << "videoEncoderPath not set\n";
  }
  if ( !exist( videoEncoderPath_ ) ) {
    ret = false;
    std::cerr << "videoEncoderPath not exist\n";
  }

  if ( ( videoEncoderOccupancyMapPath_.empty() || !exist( videoEncoderOccupancyMapPath_ ) ) ) {
    std::cerr << "WARNING: videoEncoderOccupancyMapPath is set as videoEncoderPath_ : " << videoEncoderPath_
              << std::endl;
    videoEncoderOccupancyMapPath_ = videoEncoderPath_;
  }

  if ( videoEncoderAuxPath_.empty() || !exist( videoEncoderAuxPath_ ) ) {
    std::cerr << "WARNING: videoEncoderAuxPath_ is set as videoEncoderPath_ : " << videoEncoderPath_ << std::endl;
    videoEncoderAuxPath_ = videoEncoderPath_;
  }

  if ( multipleStreams_ ) {
    if ( geometryD0Config_.empty() || geometryD1Config_.empty() ) {
      // ret = false; 
      // std::cerr << "When multipleStreams is true, geometryD0Config_ and geometryD1Config_ should be non-empty\n";
      geometryD0Config_ = geometryConfig_.substr( 0, geometryConfig_.find_last_of( "." ) ) + "-D0.cfg";
      geometryD1Config_ = geometryConfig_.substr( 0, geometryConfig_.find_last_of( "." ) ) + "-D1.cfg";
    }
    geometryConfig_ = {};
  } else {
    geometryD0Config_ = {};
    geometryD1Config_ = {};
    if ( geometryConfig_.empty() ) {
      ret = false;
      std::cerr << "When multipleStreams is not true, geometryConfig_ should be non-empty\n";
    }
  }

  if ( multipleStreams_ ) {
    if ( textureT0Config_.empty() || textureT1Config_.empty() ) {
      // std::cerr << "When multipleStreams_ is true, textureT0Config_ and textureT1Config_ should be non-empty\n";
      // ret = false;      
      textureT0Config_ = textureConfig_.substr( 0, textureConfig_.find_last_of( "." ) ) + "-T0.cfg";
      textureT1Config_ = textureConfig_.substr( 0, textureConfig_.find_last_of( "." ) ) + "-T1.cfg";
    }
    textureConfig_ = {};
  } else {
    textureT0Config_ = {};
    textureT1Config_ = {};
    if ( textureConfig_.empty() ) {
      std::cerr << "When multipleStreams is false, textureConfig_ should be non-empty\n";
      ret = false;
    }
  }
  if ( !multipleStreams_ && !absoluteD1_ ) {
    std::cerr << "absoluteD1_ should be true when multipleStreams_ is false\n";
    absoluteD1_ = true;
  }
  if ( absoluteD1_ && !absoluteT1_ ) {
    std::cerr << "absoluteT1_ should be true when absoluteD1_ is true\n";
    absoluteT1_ = true;
  }

  if ( losslessGeo_ ) {
    pbfEnableFlag_ = false;
    if ( mapCountMinus1_ == 0 ) {
      // mapCountMinus1_ = 1;
      // std::cerr << "WARNING: mapCountMinus1_ is only for lossy coding mode for now. Force "
      //             "mapCountMinus1_=1.\n";
    }
    if ( pointLocalReconstruction_ ) {
      pointLocalReconstruction_ = false;
      std::cerr << "WARNING: pointLocalReconstruction_ is only for lossy coding mode for now. Force "
                   "pointLocalReconstruction_=FALSE.\n";
    }
    if ( singleMapPixelInterleaving_ ) {
      singleMapPixelInterleaving_ = false;
      std::cerr << "WARNING: singleLayerPixelInterleaving is only for lossy coding mode for now. "
                   "Force singleMapPixelInterleaving_=FALSE.\n";
    }
    if ( lossyMissedPointsPatch_ ) {
      lossyMissedPointsPatch_ = false;
      std::cerr << "WARNING: lossyMissedPointsPatch_ is only for lossy coding mode for now. "
                   "Force lossyMissedPointsPatch_=FALSE.\n";
    }
  } else {
    if ( enhancedDeltaDepthCode_ ) {
      enhancedDeltaDepthCode_ = false;
      std::cerr << "WARNING: enhancedDeltaDepthCode_ is only for lossless coding mode for now. Force "
                   "enhancedDeltaDepthCode_=FALSE.\n";
    }
  }

  if ( enhancedDeltaDepthCode_ && surfaceThickness_ == 1 ) {
    std::cerr << "WARNING: EDD code doesn't bring any gain when surfaceThickness==1. Please "
                 "consider to increase the value of surfaceThickness.\n";
  }

  if ( enhancedDeltaDepthCode_ && ( thresholdLossyOM_ > 0 ) ) {
    std::cerr
        << "WARNING: When using enhanced occupancy map for delta depth, thresholdOccupancyMap should be equal to 0\n";
    std::cerr << "         Forcing thresholdLossyOM_ to 0\n";
    thresholdLossyOM_ = 0;
  }

  if ( enhancedDeltaDepthCode_ && ( offsetLossyOM_ > 0 ) ) {
    std::cerr
        << "WARNING: When using enhanced occupancy map for delta depth, offsetOccupancyMap should be equal to 0\n";
    std::cerr << "         Forcing offsetLossyOM_ to 0\n";
    offsetLossyOM_ = 0;
  }

  if ( enhancedDeltaDepthCode_ && ( prefilterLossyOM_ == true ) ) {
    std::cerr
        << "WARNING: When using enhanced occupancy map for delta depth, prefilterLossyOM_ should be equal to false\n";
    std::cerr << "         Forcing prefilterLossyOM_ to false\n";
    prefilterLossyOM_ = false;
  }

  if ( useMissedPointsSeparateVideo_ && !lossyMissedPointsPatch_ && !losslessGeo_ ) {
    useMissedPointsSeparateVideo_ = false;
    std::cerr << "WARNING: useMissedPointsSeparateVideo_ is for lossy coding mode if lossyMissedPointsPatch_. Force "
                 "useMissedPointsSeparateVideo_=false.\n";
  }
  if ( useMissedPointsSeparateVideo_ ) {
    if ( textureMPSeparateVideoWidth_ % 64 ) {
      ret = false;
      std::cerr << "textureMPSeparateVideoWidth_ must be multiple of 64.\n";
    }
    if ( singleMapPixelInterleaving_ ) {
      ret = false;
      std::cerr << "Pixel Interleaving is built on one layer coding. Force mapCountMinus1_ = 0.\n";
    }
    if ( geometryMPConfig_.empty() || !exist( geometryMPConfig_ ) ) {
      std::cerr << "WARNING: geometryMPConfig_ is set as geometryConfig_ : " << geometryConfig_ << std::endl;
      geometryMPConfig_ = geometryConfig_;
    } else {
      std::cout << "geometryMPConfig: " << geometryMPConfig_ << std::endl;
    }
    if ( textureMPConfig_.empty() || !exist( textureMPConfig_ ) ) {
      std::cerr << "WARNING: textureMPConfig_ is set as textureConfig_ : " << textureConfig_ << std::endl;
      textureMPConfig_ = textureConfig_;
    }
  }

  if ( singleMapPixelInterleaving_ && pointLocalReconstruction_ ) {
    ret = false;
    std::cerr << "Pixel Interleaving and Point local reconstruction cna't be use in the same time.\n";
  }

  if ( mapCountMinus1_ != 0 ) {
    if ( singleMapPixelInterleaving_ ) {
      ret = false;
      std::cerr << "Pixel Interleaving is built on one layer coding. Force mapCountMinus1_ = 0.\n";
    }
    if ( pointLocalReconstruction_ ) {
      ret = false;
      std::cerr << "Point local reconstruction is built on one layer coding. Force mapCountMinus1_ = 0.\n";
    }
  }

  if ( occupancyMapVideoEncoderConfig_.empty() ) {
    ret = false;
    std::cerr << "to use segmentation, you must define a segmentationDataPath \n";
  }
  if ( lossyMissedPointsPatch_ ) {
    if ( !useMissedPointsSeparateVideo_ ) {
      std::cerr << "Lossy missed points patch in the same video frame as the regular patches is "
                   "not optimized as of now.\n";
    }
    if ( ( minNormSumOfInvDist4MPSelection_ < 0.0 ) || ( minNormSumOfInvDist4MPSelection_ > 1.0 ) ) {
      ret = false;
      std::cerr << "minNormSumOfInvDist4MPSelection must be between 0.0 and 1.0 (inclusive)\n";
    }
  }

  if ( flagGeometrySmoothing_ ) {
    if ( pbfEnableFlag_ ) {
      gridSmoothing_ = false;
      if ( !pbfPassesCount_ ) { pbfPassesCount_ = occupancyPrecision_ <= 2 ? 1 : occupancyPrecision_ == 4 ? 2 : 4; }
      if ( !pbfFilterSize_ ) { pbfFilterSize_ = occupancyPrecision_; }
    }
    if ( gridSmoothing_ ) {
      if ( gridSize_ == 0 ) {
        ret = false;
        std::cerr << "gridSize shall be greater than 0. \n";
      }
      if ( gridSize_ % 2 == 1 ) { std::cerr << "WARNING: gridSize should be an even number\n"; }
    }
  }
  if ( flagColorSmoothing_ && gridColorSmoothing_ ) {
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

void PCCEncoderParameters::constructAspsRefList( PCCContext& context, size_t aspsIdx, size_t afpsIdx ) {
  auto& asps = context.getAtlasSequenceParameterSet( aspsIdx );
  // construction of reference frame list of ASPS
  for ( size_t list = 0; list < context.getNumOfRefAtlasFrameList(); list++ ) {
    RefListStruct refList;
    refList.setNumRefEntries( context.getMaxNumRefAtlasFrame() );  //-1,-2,-3,-4
    refList.allocate();
    for ( size_t i = 0; i < refList.getNumRefEntries(); i++ ) {
      int afocDiff = context.getRefAtlasFrame( list, i );
      refList.setAbsDeltaAfocSt( i, std::abs( afocDiff ) );
      refList.setStrpfEntrySignFlag( i, afocDiff < 0 ? 0 : 1 );
      refList.setStRefAtalsFrameFlag( i, true );
    }
    asps.addRefListStruct( refList );
  }
}

void PCCEncoderParameters::initializeContext( PCCContext& context ) {
  auto& sps = context.getVps();
  sps.allocateAltas();
  size_t atlasIndex = 0;
  auto&  ai         = sps.getAttributeInformation( atlasIndex );
  auto&  oi         = sps.getOccupancyInformation( atlasIndex );
  auto&  gi         = sps.getGeometryInformation( atlasIndex );
  auto&  asps       = context.addAtlasSequenceParameterSet( 0 );
  auto&  afps       = context.addAtlasFrameParameterSet( 0 );

  context.setOccupancyPackingBlockSize( occupancyResolution_ );
  context.setLog2PatchQuantizerSizeX( log2QuantizerSizeX_ );
  context.setLog2PatchQuantizerSizeY( log2QuantizerSizeY_ );
  context.setEnablePatchSizeQuantization( ( 1 << log2QuantizerSizeX_ ) < occupancyPrecision_ ||
                                          ( 1 << log2QuantizerSizeY_ ) < occupancyPrecision_ );

  context.setMaxNumRefAtlasFrame( maxNumRefAtlasFrame_ );
  context.setNumOfRefAtlasFrameList( maxNumRefAtlasList_ );
  for ( size_t list = 0; list < maxNumRefAtlasList_; list++ ) {
    context.setSizeOfRefAtlasFrameList( list, maxNumRefAtlasFrame_ );
    for ( size_t i = 0; i < maxNumRefAtlasFrame_; i++ ) {
      context.setRefAtlasFrame( list, i, -i - 1 );  //-1, -2, -3, -4
    }
  }

  sps.setMapCountMinus1( atlasIndex, (uint32_t)mapCountMinus1_ );
  sps.setMultipleMapStreamsPresentFlag( atlasIndex, mapCountMinus1_ != 0 && multipleStreams_ );
  sps.setRawSeparateVideoPresentFlag( atlasIndex, useMissedPointsSeparateVideo_ );
  sps.setRawPatchEnabledFlag( atlasIndex, losslessGeo_ || lossyMissedPointsPatch_ );
  for ( size_t i = 0; i < mapCountMinus1_ + 1; i++ ) {
    if ( i == 0 ) {
      sps.setMapAbsoluteCodingEnableFlag( atlasIndex, i, 1 );
      sps.setMapPredictorIndexDiff( atlasIndex, i, 0 );
    } else {
      sps.setMapAbsoluteCodingEnableFlag( atlasIndex, i, absoluteD1_ );
      sps.setMapPredictorIndexDiff( atlasIndex, i, 0 );
    }
  }

  ai.setAttributeCount( noAttributes_ ? 0 : 1 );
  ai.allocate();
  if ( noAttributes_ == 0 ) {
    ai.setAttributeDimensionMinus1( 0, noAttributes_ ? 0 : 2 );
    ai.setAttributeNominal2dBitdepthMinus1( 0, 7 );
    ai.setAttributeMSBAlignFlag( false );
  }

  for ( size_t i = 0; i < ai.getAttributeCount(); i++ ) {
    for ( size_t j = 0; j <= sps.getMapCountMinus1( ATLASIDXPCC ); j++ ) {
      if ( sps.getMapAbsoluteCodingEnableFlag( ATLASIDXPCC, j ) == 0 ) {
        ai.addAttributeMapAbsoluteCodingEnabledFlag( i, (bool)absoluteT1_ );
      } else {
        ai.addAttributeMapAbsoluteCodingEnabledFlag( i, true );
      }
    }
  }

  asps.setLog2PatchPackingBlockSize( std::log2( occupancyResolution_ ) );
  asps.setLog2MaxAtlasFrameOrderCntLsbMinus4( 4 );
  asps.setMaxDecAtlasFrameBufferingMinus1( 0 );
  asps.setNumRefAtlasFrameListsInAsps( 1 );
  asps.setMapCountMinus1( mapCountMinus1_ );
  asps.setSurfaceThicknessMinus1( surfaceThickness_ - 1 );
  asps.setLongTermRefAtlasFramesFlag( 0 );
  asps.setUseEightOrientationsFlag( useEightOrientations_ );
  asps.set45DegreeProjectionPatchPresentFlag( additionalProjectionPlaneMode_ > 0 );
  asps.setNormalAxisLimitsQuantizationEnabledFlag( 1 );
  asps.setNormalAxisMaxDeltaValueEnabledFlag( 1 );
  asps.setRemoveDuplicatePointEnabledFlag( removeDuplicatePoints_ );
  asps.setPixelDeinterleavingFlag( singleMapPixelInterleaving_ );
  asps.setPatchPrecedenceOrderFlag( patchPrecedenceOrderFlag_ );
  asps.setPatchSizeQuantizerPresentFlag( context.getEnablePatchSizeQuantization() );
  asps.setEnhancedOccupancyMapForDepthFlag( enhancedDeltaDepthCode_ );
  asps.setPointLocalReconstructionEnabledFlag( pointLocalReconstruction_ );
  asps.setVuiParametersPresentFlag( 0 );
  asps.setExtensionPresentFlag( 0 );
  asps.setExtensionDataFlag( 0 );

  if ( asps.getEnhancedOccupancyMapFixBitCountMinus1() && mapCountMinus1_ == 0 ) {
    asps.setEnhancedOccupancyMapFixBitCountMinus1( EOMFixBitCount_ - 1 );
  } else {
    asps.setEnhancedOccupancyMapFixBitCountMinus1( EOMFixBitCount_ - 1 );  // default values
  }
  afps.setAtlasSequenceParameterSetId( 0 );
  afps.setAfpsNumRefIdxDefaultActiveMinus1( ( uint8_t )( std::max )( 0, (int)maxNumRefAtlasFrame_ - 1 ) );
  afps.setAfpsAdditionalLtAfocLsbLen( 4 );
  afps.setAfps2dPosXBitCountMinus1( 0 );
  afps.setAfps2dPosYBitCountMinus1( 0 );
  afps.setAfps3dPosXBitCountMinus1( 0 );
  afps.setAfps3dPosYBitCountMinus1( 0 );
  afps.setAfpsOverrideEomForDepthFlag( 0 );
  afps.setAfpsEomNumberOfPatchBitCountMinus1( 0 );
  afps.setAfpsEomMaxBitCountMinus1( 0 );
  afps.setAfpsRaw3dPosBitCountExplicitModeFlag( 0 );
  afps.setAfpsExtensionPresentFlag( 0 );
  afps.setAfpsExtensionDataFlag( 0 );

  if ( afps.getAfpsOverrideEomForDepthFlag() == 0 ) {
    afps.setAfpsEomMaxBitCountMinus1( 7 );
    afps.setAfpsEomNumberOfPatchBitCountMinus1( 7 );
  }
  for ( size_t frameIdx = 0; frameIdx < frameCount_; frameIdx++ ) {
    auto& atgl = context.addAtlasTileGroupLayer( frameIdx );
    auto& atgh = atgl.getAtlasTileGroupHeader();
    atgh.setAtghAtlasFrameParameterSetId( 0 );
#ifdef BUGFIX_45_DEGREE_PROJECTION
    if( additionalProjectionPlaneMode_ > 0 ) {
      atgh.setAtghPosMinZQuantizer( uint8_t( std::log2( minLevel_ ) ) - 1 );
    } else {
      atgh.setAtghPosMinZQuantizer( uint8_t( std::log2( minLevel_ ) ) );
    }
#else
    atgh.setAtghPosMinZQuantizer( uint8_t( std::log2( minLevel_ ) ) );
#endif
    atgh.setAtghPosDeltaMaxZQuantizer( uint8_t( std::log2( minLevel_ ) ) );
    atgh.setAtghPatchSizeXinfoQuantizer( log2QuantizerSizeX_ );
    atgh.setAtghPatchSizeYinfoQuantizer( log2QuantizerSizeY_ );
    if ( afps.getAfpsRaw3dPosBitCountExplicitModeFlag() ) {
      atgh.setAtghRaw3dPosAxisBitCountMinus1( 0 );  //
    } else {
      atgh.setAtghRaw3dPosAxisBitCountMinus1( geometry3dCoordinatesBitdepth_ - geometryNominal2dBitdepth_ - 1 );
    }
    atgh.setAtghNumRefIdxActiveOverrideFlag( 0 );

    atgh.setAtghRefAtlasFrameListSpsFlag( true );
    atgh.setAtghRefAtlasFrameListIdx( 0 );
  }

  // construction of reference frame list of ASPS
  constructAspsRefList( context, 0, 0 );

  oi.setLossyOccupancyMapCompressionThreshold( (size_t)thresholdLossyOM_ );
  oi.setOccupancyNominal2DBitdepthMinus1( 7 );
  oi.setOccupancyMSBAlignFlag( false );

  gi.setGeometry3dCoordinatesBitdepthMinus1( uint8_t( geometry3dCoordinatesBitdepth_ - 1 ) );
  gi.setGeometryNominal2dBitdepthMinus1( uint8_t( geometryNominal2dBitdepth_ - 1 ) );
  gi.setGeometryMSBAlignFlag( false );

  // deprecated
  sps.setLosslessGeo444( losslessGeo444_ );
  sps.setLosslessGeo( losslessGeo_ );
  sps.setMinLevel( minLevel_ );

  // Encoder only data
  context.setOccupancyPrecision( occupancyPrecision_ );
  context.setOccupancyPackingBlockSize( occupancyResolution_ );
  context.setModelScale( modelScale_ );
  context.setModelOrigin( modelOrigin_ );
  context.setMPGeoWidth( textureMPSeparateVideoWidth_ );
  context.setMPAttWidth( textureMPSeparateVideoWidth_ );
  context.setMPGeoHeight( 0 );
  context.setMPAttHeight( 0 );
  //multiple streams allocation
  if (multipleStreams_) {
    context.getVideoGeometryMultiple().resize(mapCountMinus1_ + 1);
    context.getVideoTextureMultiple().resize(mapCountMinus1_ + 1);
  } 
  context.setGeometry3dCoordinatesBitdepth( geometry3dCoordinatesBitdepth_ );
  size_t numPlrm =
      pointLocalReconstruction_
          ? ( std::max )( (size_t)1, ( std::min )( plrlNumberOfModes_, g_pointLocalReconstructionMode.size() ) )
          : 1;
  for ( size_t i = 0; i < numPlrm; i++ ) {
    context.addPointLocalReconstructionMode( g_pointLocalReconstructionMode[i] );
  }
  // Lossy occupancy map
  context.getOffsetLossyOM()    = offsetLossyOM_;
  context.getPrefilterLossyOM() = prefilterLossyOM_;
}
