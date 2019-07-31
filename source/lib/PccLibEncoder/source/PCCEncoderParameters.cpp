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
  minPointCountPerCCPatchSegmentation_     = 16;
  maxNNCountPatchSegmentation_             = 16;
  surfaceThickness_                        = 4;
  minLevel_                                = 64;  // fix value
  maxAllowedDepth_                         = 255;
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
  videoEncoderPath_                        = {};
  videoEncoderAuxPath_                     = {};
  videoEncoderOccupancyMapPath_            = {};
  geometryQP_                              = 28;
  textureQP_                               = 43;
  geometryConfig_                          = {};
  geometryD0Config_                        = {};
  geometryD1Config_                        = {};
  textureConfig_                           = {};
  losslessGeo_                             = false;
  losslessTexture_                         = false;
  noAttributes_                            = false;
  losslessGeo444_                          = false;
  useMissedPointsSeparateVideo_            = false;
  geometryMPConfig_                        = {};
  textureMPConfig_                         = {};
  nbThread_                                = 1;
  keepIntermediateFiles_                   = false;

  absoluteD1_                              = true;
  constrainedPack_                         = true;
  thresholdColorSmoothing_                 = 10.0;
  thresholdColorDifference_                = 100.0;
  thresholdColorVariation_                 = 6.0;
  thresholdLocalEntropy_                   = 4.0;
  radius2ColorSmoothing_                   = 4.0 * 16;
  neighborCountColorSmoothing_             = 4 * 16;
  flagColorSmoothing_                      = false;
  gridColorSmoothing_                      = false;
  cgridSize_                               = 4;
  thresholdColorPreSmoothing_              = 10.0;
  thresholdColorPreSmoothingLocalEntropy_  = 4.5;
  radius2ColorPreSmoothing_                = 4.0 * 16;
  neighborCountColorPreSmoothing_          = 4 * 16;
  flagColorPreSmoothing_                   = true;
  groupDilation_                           = true;
  textureDilationOffLossless_              = true;
  enhancedDeltaDepthCode_                  = losslessGeo_ ? true : false;
  EOMFixBitCount_                          = 2;
  EOMTexturePatch_                         = true;
  offsetLossyOM_                           = 0;
  thresholdLossyOM_                        = 0;
  prefilterLossyOM_                        = false;
  patchColorSubsampling_                   = false;
  deltaCoding_                             = true;
  layerCountMinus1_                        = 1;

  // reconstruction
  removeDuplicatePoints_        = true;
  pointLocalReconstruction_     = false;
  plrlNumberOfModes_            = 6;
  patchSize_                    = 9;
  singleLayerPixelInterleaving_ = false;
  surfaceSeparation_            = false;

  // level of detail
  testLevelOfDetail_          = 0;
  testLevelOfDetailSignaling_ = 0;

  // Flexible Patch Packing
  packingStrategy_      = 1;
  textureBGFill_        = 1;
  safeGuardDistance_    = 0;
  useEightOrientations_ = false;
  lowDelayEncoding_ = false;
	geometryPadding_ = false;

  // lossy missed points patch
  lossyMissedPointsPatch_           = false;
  minNormSumOfInvDist4MPSelection_  = 0.35;
  lossyMppGeoQP_                    = 4;
  useAdditionalPointsPatch_         = false;
  //GPA
  globalPatchAllocation_            = 0;
  //GTP
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
  mortonOrderSortMissedPoints_  = false;
}

PCCEncoderParameters::~PCCEncoderParameters() {}

void PCCEncoderParameters::completePath() {
  if ( useMissedPointsSeparateVideo_ && !lossyMissedPointsPatch_ && !losslessGeo_ ) {
    useMissedPointsSeparateVideo_ = false;
  }
  if ( !uncompressedDataFolder_.empty() ) {
    if ( !uncompressedDataPath_.empty() ) { uncompressedDataPath_ = uncompressedDataFolder_ + uncompressedDataPath_; }
  }
  if ( !configurationFolder_.empty() ) {
    if ( !geometryConfig_.empty() ) { geometryConfig_ = configurationFolder_ + geometryConfig_; }
    if ( !geometryD0Config_.empty() ) { geometryD0Config_ = configurationFolder_ + geometryD0Config_; }
    if ( !geometryD1Config_.empty() ) { geometryD1Config_ = configurationFolder_ + geometryD1Config_; }
    if ( !textureConfig_.empty() ) { textureConfig_ = configurationFolder_ + textureConfig_; }
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
  std::cout << "\t losslessTexture                          " << losslessTexture_ << std::endl;
  std::cout << "\t noAttributes                             " << noAttributes_ << std::endl;
  std::cout << "\t losslessGeo444                           " << losslessGeo444_ << std::endl;
  std::cout << "\t enhancedDeltaDepthCode                   " << enhancedDeltaDepthCode_ << std::endl;
  std::cout << "\t useMissedPointsSeparateVideo             " << useMissedPointsSeparateVideo_ << std::endl;
  std::cout << "\t uncompressedDataPath                     " << uncompressedDataPath_ << std::endl;
  std::cout << "\t compressedStreamPath                     " << compressedStreamPath_ << std::endl;
  std::cout << "\t reconstructedDataPath                    " << reconstructedDataPath_ << std::endl;
  std::cout << "\t frameCount                               " << frameCount_ << std::endl;
  std::cout << "\t layerCountMinus1                          " << layerCountMinus1_ << std::endl;
  std::cout << "\t startFrameNumber                         " << startFrameNumber_ << std::endl;
  std::cout << "\t groupOfFramesSize                        " << groupOfFramesSize_ << std::endl;
  std::cout << "\t colorTransform                           " << colorTransform_ << std::endl;
  std::cout << "\t nbThread                                 " << nbThread_ << std::endl;
  std::cout << "\t keepIntermediateFiles                    " << keepIntermediateFiles_ << std::endl;
  std::cout << "\t absoluteD1                               " << absoluteD1_ << std::endl;
  std::cout << "\t constrainedPack                          " << constrainedPack_ << std::endl;
  std::cout << "\t deltaCoding                              " << deltaCoding_ << std::endl;
  std::cout << "\t segmentation" << std::endl;
  std::cout << "\t   nnNormalEstimation                     " << nnNormalEstimation_ << std::endl;
  std::cout << "\t   gridBasedRefineSegmentation            " << gridBasedRefineSegmentation_ << std::endl;
  std::cout << "\t   maxNNCountRefineSegmentation           " << maxNNCountRefineSegmentation_ << std::endl;
  std::cout << "\t   iterationCountRefineSegmentation       " << iterationCountRefineSegmentation_ << std::endl;
  std::cout << "\t   voxelDimensionRefineSegmentation       " << voxelDimensionRefineSegmentation_ << std::endl;
  std::cout << "\t   searchRadiusRefineSegmentation         " << searchRadiusRefineSegmentation_ << std::endl;
  std::cout << "\t   occupancyResolution                    " << occupancyResolution_ << std::endl;
  std::cout << "\t   minPointCountPerCCPatchSegmentation    " << minPointCountPerCCPatchSegmentation_ << std::endl;
  std::cout << "\t   maxNNCountPatchSegmentation            " << maxNNCountPatchSegmentation_ << std::endl;
  std::cout << "\t   surfaceThickness                       " << surfaceThickness_ << std::endl;
  std::cout << "\t   maxAllowedDist2MissedPointsDetection " << maxAllowedDist2MissedPointsDetection_ << std::endl;
  std::cout << "\t   maxAllowedDist2MissedPointsSelection " << maxAllowedDist2MissedPointsSelection_ << std::endl;
  std::cout << "\t   lambdaRefineSegmentation               " << lambdaRefineSegmentation_ << std::endl;
  std::cout << "\t   depthQuantizationStep                  " << minLevel_ << std::endl;
  std::cout << "\t   maxAllowedDepth                        " << maxAllowedDepth_ << std::endl;
  std::cout << "\t packing" << std::endl;
  std::cout << "\t   minimumImageWidth                      " << minimumImageWidth_ << std::endl;
  std::cout << "\t   minimumImageHeight                     " << minimumImageHeight_ << std::endl;
  std::cout << "\t   packingStrategy                        " << packingStrategy_ << std::endl;
  std::cout << "\t   useEightOrientations                   " << useEightOrientations_ << std::endl;
  std::cout << "\t   safeGuardDistance                      " << safeGuardDistance_ << std::endl;
  std::cout << "\t   globalPatchAllocation                  " << globalPatchAllocation_ << std::endl;
  if (globalPatchAllocation_ == 2) {
	  std::cout << "\t   globalPackingStrategyGOF           " << geometryD0Config_ << std::endl;
	  std::cout << "\t   globalPackingStrategyReset         " << geometryD1Config_ << std::endl;
	  std::cout << "\t   globalPackingStrategyThreshold     " << geometryD1Config_ << std::endl;
  }
  std::cout << "\t   lowDelayEncoding                       " << lowDelayEncoding_ << std::endl;
  std::cout << "\t   textureBGFill                          " << textureBGFill_ << std::endl;
	std::cout << "\t   geometryPadding                        " << geometryPadding_ << std::endl;
  std::cout << "\t video encoding" << std::endl;
  std::cout << "\t   geometryQP                             " << geometryQP_ << std::endl;
  std::cout << "\t   textureQP                              " << textureQP_ << std::endl;
  std::cout << "\t   colorSpaceConversionPath               " << colorSpaceConversionPath_ << std::endl;
  std::cout << "\t   videoEncoderPath                       " << videoEncoderPath_ << std::endl;
  std::cout << "\t   videoEncoderAuxPath                    " << videoEncoderAuxPath_ << std::endl;
  std::cout << "\t   videoEncoderOccupancyMapPath           " << videoEncoderOccupancyMapPath_ << std::endl;
  if ( !absoluteD1_ ) {
    std::cout << "\t   geometryD0Config                       " << geometryD0Config_ << std::endl;
    std::cout << "\t   geometryD1Config                       " << geometryD1Config_ << std::endl;
  } else {
    std::cout << "\t   geometryConfig                         " << geometryConfig_ << std::endl;
  }
  std::cout << "\t   textureConfig                          " << textureConfig_ << std::endl;
  if ( useMissedPointsSeparateVideo_ ) {
    if ( losslessGeo_ ) {
      std::cout << "\t geometryMPConfig                          " << geometryMPConfig_ << std::endl;
    }
    if ( losslessTexture_ ) {
      std::cout << "\t textureMPConfig                            " << textureMPConfig_ << std::endl;
    }
  }
  std::cout << "\t   colorSpaceConversionConfig             " << colorSpaceConversionConfig_ << std::endl;
  std::cout << "\t   inverseColorSpaceConversionConfig      " << inverseColorSpaceConversionConfig_ << std::endl;
  std::cout << "\t   apply3dMotionCompensation              " << use3dmc_ << std::endl;
  std::cout << "\t dilation" << std::endl;
  std::cout << "\t   GroupDilation                          " << groupDilation_ << std::endl;
  std::cout << "\t   textureDilationOffLossless             " << textureDilationOffLossless_ << std::endl;
  std::cout << "\t occupancy map encoding " << std::endl;
  std::cout << "\t   maxCandidateCount                      " << maxCandidateCount_ << std::endl;
  std::cout << "\t   occupancyPrecision                     " << occupancyPrecision_ << std::endl;
  std::cout << "\t   occupancyMapVideoEncoderConfig         " << occupancyMapVideoEncoderConfig_ << std::endl;
  std::cout << "\t   occupancyMapQP                         " << occupancyMapQP_ << std::endl;
  std::cout << "\t   EOMFixBitCount                         " << EOMFixBitCount_ << std::endl;
  std::cout << "\t   EOMTexturePatch                        " << EOMTexturePatch_ << std::endl;
  std::cout << "\t   occupancyMapRefinement                 " << occupancyMapRefinement_ << std::endl;
  std::cout << "\t Lossy occupancy Map coding" << std::endl;
  std::cout << "\t   Lossy occupancy map offset             " << offsetLossyOM_ << std::endl;
  std::cout << "\t   Lossy occupancy map threshold          " << thresholdLossyOM_ << std::endl;
  std::cout << "\t   Lossy occupancy map prefilter          " << prefilterLossyOM_ << std::endl;
  std::cout << "\t geometry smoothing" << std::endl;
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
  std::cout << "\t color smoothing" << std::endl;
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
  std::cout << "\t color pre-smoothing                      " << std::endl;
  std::cout << "\t   thresholdColorPreSmoothing             " << thresholdColorSmoothing_ << std::endl;
  std::cout << "\t   thresholdColorPreSmoothingLocalEntropy " << thresholdColorPreSmoothingLocalEntropy_ << std::endl;
  std::cout << "\t   radius2ColorPreSmoothing               " << radius2ColorPreSmoothing_ << std::endl;
  std::cout << "\t   neighborCountColorPreSmoothing         " << neighborCountColorPreSmoothing_ << std::endl;
  std::cout << "\t   flagColorPreSmoothing                  " << flagColorPreSmoothing_ << std::endl;
  std::cout << "\t coloring" << std::endl;
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
  std::cout << "\t Reconstruction " << std::endl;
  std::cout << "\t   removeDuplicatePoints                  " << removeDuplicatePoints_ << std::endl;
  std::cout << "\t   pointLocalReconstruction               " << pointLocalReconstruction_ << std::endl;
  std::cout << "\t     plrlNumberOfModes                    " << plrlNumberOfModes_ << std::endl;
  std::cout << "\t     patchSize                            " << patchSize_ << std::endl;
  std::cout << "\t   singleLayerPixelInterleaving           " << singleLayerPixelInterleaving_ << std::endl;
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
  if (enablePointCloudPartitioning_) {
    auto printVector = [](std::vector<int> &vec) {
      for (int i = 0; i < vec.size(); ++i) {
        std::cout << vec[i];
        if (i < vec.size() - 1) {
          std::cout << ",";
        }
      }
      std::cout << std::endl;
    };
    std::cout << "\t   roiBoundingBoxMinX                     ";   printVector(roiBoundingBoxMinX_);
    std::cout << "\t   roiBoundingBoxMaxX                     ";   printVector(roiBoundingBoxMaxX_);
    std::cout << "\t   roiBoundingBoxMinY                     ";   printVector(roiBoundingBoxMinY_);
    std::cout << "\t   roiBoundingBoxMaxY                     ";   printVector(roiBoundingBoxMaxY_);
    std::cout << "\t   roiBoundingBoxMinZ                     ";   printVector(roiBoundingBoxMinZ_);
    std::cout << "\t   roiBoundingBoxMaxZ                     ";   printVector(roiBoundingBoxMaxZ_);
    std::cout << "\t   numTilesHor                            " << numTilesHor_                         << std::endl;
    std::cout << "\t   tileHeightToWidthRatio                 " << tileHeightToWidthRatio_              << std::endl;
    std::cout << "\t   numCutsAlong1stLongestAxis             " << numCutsAlong1stLongestAxis_          << std::endl;
    std::cout << "\t   numCutsAlong2ndLongestAxis             " << numCutsAlong2ndLongestAxis_          << std::endl;
    std::cout << "\t   numCutsAlong3rdLongestAxis             " << numCutsAlong3rdLongestAxis_          << std::endl;
    std::vector<size_t> vecSizes = {roiBoundingBoxMinX_.size(), roiBoundingBoxMaxX_.size(),
                                    roiBoundingBoxMinY_.size(), roiBoundingBoxMaxY_.size(),
                                    roiBoundingBoxMinZ_.size(), roiBoundingBoxMaxZ_.size()};
    for (int i = 0; i < vecSizes.size(); ++i) {
      if (vecSizes[i] != vecSizes[0]) {
        std::cerr
            << "All the 6 arrays roiBoundingBox[Min-Max][X-Y-Z] must have the same number of elements."
            << std::endl;
        std::exit(EXIT_FAILURE);
      }
    }
    numROIs_ = int(roiBoundingBoxMinX_.size());
    for (int i = 0; i < roiBoundingBoxMinX_.size(); ++i) {
      if (!(roiBoundingBoxMinX_[i] < roiBoundingBoxMaxX_[i]) ||
          !(roiBoundingBoxMinY_[i] < roiBoundingBoxMaxY_[i]) ||
          !(roiBoundingBoxMinZ_[i] < roiBoundingBoxMaxZ_[i])) {
        std::cerr << "ROI min/max values are not correct." << std::endl;
        std::exit(EXIT_FAILURE);
      }
    }
  }
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

  if ( absoluteD1_ && ( !geometryD0Config_.empty() || !geometryD1Config_.empty() ) ) {
    ret = false;
    std::cerr << "When absoultD1 is true, geometryD0Config_ and geometryD1Config_ should be empty\n";
  }
  if ( absoluteD1_ && geometryConfig_.empty() ) {
    ret = false;
    std::cerr << "When absoultD1 is true, geometryConfig_ should be non-empty\n";
  }
  if ( !absoluteD1_ && ( geometryD0Config_.empty() || geometryD1Config_.empty() ) ) {
    ret = false;
    std::cerr << "When absoultD1 is false, geometryD0Config_ and geometryD1Config_ should be non-empty\n";
  }

  if ( !absoluteD1_ && !geometryConfig_.empty() ) {
    ret = false;
    std::cerr << "When absoultD1 is false, geometryConfig_ should be empty\n";
  }


  if ( !losslessGeo_ && enhancedDeltaDepthCode_ ) {
    enhancedDeltaDepthCode_ = false;
    std::cerr << "WARNING: enhancedDeltaDepthCode is only for lossless coding mode for now. Force "
                 "enhancedDeltaDepthCode=FALSE.\n";
  }
  if ( !enhancedDeltaDepthCode_ ) {   
    std::cerr << "WARNING: EOMTexturePatch is only for enhancedDeltaDepthCode coding mode for now. Force "
                 "EOMTexturePatch=FALSE.\n";
    EOMTexturePatch_ = false; 
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

  if ( useMissedPointsSeparateVideo_ ) {
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

  if ( losslessGeo_ ) {
    if ( pointLocalReconstruction_ ) {
      pointLocalReconstruction_ = false;
      std::cerr << "WARNING: pointLocalReconstruction_ is only for lossy coding mode for now. Force "
                   "pointLocalReconstruction=FALSE.\n";
    }
    if ( singleLayerPixelInterleaving_ ) {
      singleLayerPixelInterleaving_ = false;
      std::cerr << "WARNING: singleLayerPixelInterleaving is only for lossy coding mode for now. "
                   "Force singleLayerPixelInterleaving=FALSE.\n";
    }
  }

  if ( singleLayerPixelInterleaving_ && pointLocalReconstruction_ ) {
    ret = false;
    std::cerr << "Pixel Interleaving and Point local reconstruction cna't be use in the same time.\n";
  }

  if ( layerCountMinus1_ != 0 ) {
    if ( singleLayerPixelInterleaving_ ) {
      ret = false;
      std::cerr << "Pixel Interleaving is built on one layer coding. Force layerCountMinus1_ = 0.\n";
    }
    if ( pointLocalReconstruction_ ) {
      ret = false;
      std::cerr << "Point local reconstruction is built on one layer coding. Force layerCountMinus1_ = 0.\n";
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

  if ( flagGeometrySmoothing_ && gridSmoothing_ ) {
    if ( gridSize_ == 0 ) {
      ret = false;
      std::cerr << "gridSize shall be greater than 0. \n";
    }
    if ( gridSize_ % 2 == 1 ) { std::cerr << "WARNING: gridSize should be an even number\n"; }
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

void PCCEncoderParameters::initializeContext( PCCContext& context ) {
  auto& sps   = context.getSps();
  auto& pdg   = context.getPatchDataGroup();
  auto& ai    = sps.getAttributeInformation();
  auto& oi    = sps.getOccupancyInformation();
  auto& gi    = sps.getGeometryInformation();
  auto& psps  = pdg.getPatchSequenceParameterSet( 0 );
  auto& pfgps = pdg.getPatchFrameGeometryParameterSet( 0 );
  auto& pfaps = pdg.getPatchFrameAttributeParameterSet( 0 );
  auto& gfp   = pfgps.getGeometryFrameParams();
  auto& afp   = pfaps.getAttributeFrameParams();
  auto& pfps  = pdg.getPatchFrameParameterSet( 0 );
  auto& pfti  = pfps.getPatchFrameTileInformation();

  context.setOccupancyPackingBlockSize( occupancyResolution_ );

  sps.setLayerCountMinus1( layerCountMinus1_ );
  sps.allocate();
  sps.setPcmSeparateVideoPresentFlag( useMissedPointsSeparateVideo_ );
  sps.setEnhancedOccupancyMapForDepthFlag( enhancedDeltaDepthCode_ );
  if( sps.getEnhancedOccupancyMapForDepthFlag() ){
    sps.setEOMFixBitCount( EOMFixBitCount_ );
    sps.setEOMTexturePatch( EOMTexturePatch_ );
  }
  sps.setPixelDeinterleavingFlag( singleLayerPixelInterleaving_ );
  sps.setMultipleLayerStreamsPresentFlag( layerCountMinus1_ != 0 );
  sps.setRemoveDuplicatePointEnabledFlag( removeDuplicatePoints_ );
  for ( size_t i = 0; i < layerCountMinus1_ + 1; i++ ) {
    if ( i == 0 ) {
      sps.setLayerAbsoluteCodingEnabledFlag( i, 1 );
    } else {
      sps.setLayerAbsoluteCodingEnabledFlag( i, absoluteD1_ );
    }
  }
  sps.setPcmPatchEnabledFlag( useAdditionalPointsPatch_ );
  sps.setPatchInterPredictionEnabledFlag( deltaCoding_ );
  sps.setSurfaceThickness( surfaceThickness_ );
  sps.setProjection45DegreeEnableFlag( additionalProjectionPlaneMode_ > 0 ? 1 : 0 );
	if(lowDelayEncoding_)
		sps.setPatchPrecedenceOrderFlag( 1 ); //low delay coding requires patch precedence set to 1
	else
		sps.setPatchPrecedenceOrderFlag( 0 ); 

  ai.setAttributeCount( noAttributes_ ? 0 : 1 );
  ai.allocate();
  ai.setAttributeParamsEnabledFlag( flagColorSmoothing_ );
  ai.setAttributeDimensionMinus1( 0, noAttributes_ ? 0 : 2 );
  ai.setAttributeNominal2dBitdepthMinus1( 0, 7 );

  pfps.allocate( ai.getAttributeCount() );
  pfti.setSingleTileInPatchFrameFlag( true );
  pfti.setSingleTilePerTileGroupFlag( true );

  oi.setLossyOccupancyMapCompressionThreshold( (size_t)thresholdLossyOM_ );

  gi.setGeometryParamsEnabledFlag( flagGeometrySmoothing_ );
  gi.setGeometry3dCoordinatesBitdepthMinus1( uint8_t( geometry3dCoordinatesBitdepth_ - 1 ) );
  gi.setGeometryNominal2dBitdepthMinus1( uint8_t( geometryNominal2dBitdepth_ - 1 ) );

  psps.setLog2PatchPackingBlockSize( std::log2( occupancyResolution_ ) );
  psps.setNormalAxisLimitsQuantizationEnableFlag( true );
  psps.setNormalAxisMaxDeltaValueEnableFlag( true );

  gfp.setGeometrySmoothingParamsPresentFlag( flagGeometrySmoothing_ );
  gfp.setGeometrySmoothingEnabledFlag( flagGeometrySmoothing_ );
  gfp.setGeometrySmoothingGridSizeMinus2( gridSize_ - 2 );
  gfp.setGeometrySmoothingThreshold( thresholdSmoothing_ );
  gfp.setGeometrySmoothingEnabledFlag( gridSmoothing_ );

  pfaps.setAttributeDimensionMinus1( 2 );
  afp.allocate( pfaps.getAttributeDimensionMinus1() + 1 );
  for ( size_t i = 0; i < pfaps.getAttributeDimensionMinus1() + 1; i++ ) {
    afp.setAttributeSmoothingParamsPresentFlag( i, flagColorSmoothing_ );
    afp.setAttributeSmoothingGridSizeMinus2( i, cgridSize_ - 2 );
    afp.setAttributeSmoothingThreshold( i, thresholdColorSmoothing_ );
    afp.setAttributeSmoothingThresholdAttributeDifference( i, thresholdColorDifference_ );
    afp.setAttributeSmoothingThresholdAttributeVariation( i, thresholdColorVariation_ );
    afp.setAttributeSmoothingLocalEntropyThreshold( i, thresholdLocalEntropy_ );
  }

  // deprecated
  sps.setLosslessGeo444( losslessGeo444_ );
  sps.setLosslessGeo( losslessGeo_ );
  sps.setLosslessTexture( losslessTexture_ );
  sps.setMinLevel( minLevel_ );

  // Encoder only data
  context.setOccupancyPrecision( occupancyPrecision_ );
  context.setOccupancyPackingBlockSize( occupancyResolution_ );
  context.setModelScale( modelScale_ );
  context.setModelOrigin( modelOrigin_ );
  context.setMPGeoWidth( 64 );
  context.setMPAttWidth( 64 );
  context.setMPGeoHeight( 0 );
  context.setMPAttHeight( 0 );
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
