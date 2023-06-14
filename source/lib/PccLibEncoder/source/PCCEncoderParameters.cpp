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
  gridBasedSegmentation_               = false;
  voxelDimensionGridBasedSegmentation_ = 2;
  uncompressedDataPath_                = {};
  compressedStreamPath_                = {};
  reconstructedDataPath_               = {};
  configurationFolder_                 = {};
  uncompressedDataFolder_              = {};
  startFrameNumber_                    = 0;
  frameCount_                          = 300;
  groupOfFramesSize_                   = 32;
  colorTransform_                      = COLOR_TRANSFORM_NONE;
  colorSpaceConversionPath_            = {};
  colorSpaceConversionConfig_          = {};
  inverseColorSpaceConversionConfig_   = {};
  nnNormalEstimation_                  = 16;
  normalOrientation_                   = 1;
  forcedSsvhUnitSizePrecisionBytes_    = 0;
  gridBasedRefineSegmentation_         = true;
  maxNNCountRefineSegmentation_        = gridBasedRefineSegmentation_ ? ( gridBasedSegmentation_ ? 384 : 1024 ) : 256;
  iterationCountRefineSegmentation_    = gridBasedRefineSegmentation_ ? ( gridBasedSegmentation_ ? 5 : 10 ) : 100;
  voxelDimensionRefineSegmentation_    = gridBasedSegmentation_ ? 2 : 4;
  searchRadiusRefineSegmentation_      = gridBasedSegmentation_ ? 128 : 192;
  occupancyResolution_                 = 16;
  enablePatchSplitting_                = true;
  maxPatchSize_                        = 1024;
  log2QuantizerSizeX_                  = 4;
  log2QuantizerSizeY_                  = 4;
  minPointCountPerCCPatchSegmentation_ = 16;
  maxNNCountPatchSegmentation_         = 16;
  surfaceThickness_                    = 4;
  minLevel_                            = 64;  // fix value
  maxAllowedDist2RawPointsDetection_   = 9.0;
  maxAllowedDist2RawPointsSelection_   = 1.0;
  lambdaRefineSegmentation_            = 3.0;
  minimumImageWidth_                   = 1280;
  minimumImageHeight_                  = 1280;
  maxCandidateCount_                   = 4;
  occupancyPrecision_                  = 4;
  occupancyMapConfig_                  = {};
  occupancyMapQP_                      = 8;
  occupancyMapRefinement_              = false;
  decodedAtlasInformationHash_         = 0;
  flagGeometrySmoothing_               = true;
  patchExpansion_                      = false;
  gridSmoothing_                       = true;
  gridSize_                            = 8;
  neighborCountSmoothing_              = 4 * 16;
  radius2Smoothing_                    = 4.0 * 16;
  radius2BoundaryDetection_            = 4.0 * 16;
  thresholdSmoothing_                  = 64.0;
  bestColorSearchRange_                = 0;
  numNeighborsColorTransferFwd_        = 1;
  numNeighborsColorTransferBwd_        = 1;
  useDistWeightedAverageFwd_           = true;
  useDistWeightedAverageBwd_           = true;
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
  videoEncoderInternalBitdepth_            = 10;
  byteStreamVideoCoderOccupancy_           = true;
  byteStreamVideoCoderGeometry_            = true;
  byteStreamVideoCoderAttribute_           = true;
  geometryQP_                              = 28;
  attributeQP_                             = 43;
  auxGeometryQP_                           = 0;
  auxAttributeQP_                          = 0;
  geometryConfig_                          = {};
  geometry0Config_                         = {};
  geometry1Config_                         = {};
  attributeConfig_                         = {};
  attribute0Config_                        = {};
  attribute1Config_                        = {};
  rawPointsPatch_                          = false;
  noAttributes_                            = false;
  attributeVideo444_                       = false;
  useRawPointsSeparateVideo_               = false;
  geometryAuxVideoConfig_                  = {};
  attributeAuxVideoConfig_                 = {};
  nbThread_                                = 1;
  keepIntermediateFiles_                   = false;
  absoluteD1_                              = false;
  absoluteT1_                              = false;
  multipleStreams_                         = false;
  deltaQPD0_                               = 0;
  deltaQPD1_                               = 2;
  deltaQPT0_                               = 0;
  deltaQPT1_                               = 2;
  constrainedPack_                         = true;
  thresholdColorSmoothing_                 = 10.0;
  thresholdColorDifference_                = 10.0;
  thresholdColorVariation_                 = 6.0;
  flagColorSmoothing_                      = false;
  cgridSize_                               = 4;
  thresholdColorPreSmoothing_              = 10.0;
  thresholdColorPreSmoothingLocalEntropy_  = 4.5;
  radius2ColorPreSmoothing_                = 4.0 * 16;
  neighborCountColorPreSmoothing_          = 4 * 16;
  flagColorPreSmoothing_                   = true;
  groupDilation_                           = true;
  enhancedOccupancyMapCode_                = false;
  EOMFixBitCount_                          = 2;
  offsetLossyOM_                           = 0;
  thresholdLossyOM_                        = 0;
  prefilterLossyOM_                        = false;
  patchColorSubsampling_                   = false;
  mapCountMinus1_                          = 1;

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
  attributeBGFill_      = 1;
  safeGuardDistance_    = 0;
  useEightOrientations_ = false;
  lowDelayEncoding_     = false;
  geometryPadding_      = 0U;

  // lossy raw points patch
  lossyRawPointsPatch_             = false;
  minNormSumOfInvDist4MPSelection_ = 0.35;

  // GPA
  globalPatchAllocation_ = 0;
  // GTP
  globalPackingStrategyGOF_         = 0;
  globalPackingStrategyReset_       = false;
  globalPackingStrategyThreshold_   = 0;
  use3dmc_                          = true;
  usePccRDO_                        = false;
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
  mortonOrderSortRawPoints_       = false;
  attributeRawSeparateVideoWidth_ = 128;

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
  tilePartitionWidthList_.clear();
  tilePartitionHeightList_.clear();

  // Profile tier level
  tierFlag_                 = 0;                        // Low Tier
  profileCodecGroupIdc_     = CODEC_GROUP_HEVC_MAIN10;  // HEVC Main10
  profileToolsetIdc_        = 1;                        // V-PCC Basic V-PCC Extend
  profileReconstructionIdc_ = 0;                        // Rec0, Rec1 or Rec2
  levelIdc_                 = 30;                       // Corresponds to level 1.0 in Table A.5
  avcCodecIdIndex_          = 0;                        // Index use if CMC SEI
  hevcCodecIdIndex_         = 1;                        // Index use if CMC SEI
  shvcCodecIdIndex_         = 2;                        // Index use if CMC SEI
  vvcCodecIdIndex_          = 3;                        // Index use if CMC SEI

  // Profile toolset constraints information
  oneV3CFrameOnlyFlag_                     = 0;  // V-PCC Basic
  EOMContraintFlag_                        = false;
  maxMapCountMinus1_                       = 1;
  maxAtlasCountMinus1_                     = 0;
  multipleMapStreamsConstraintFlag_        = false;
  PLRConstraintFlag_                       = false;
  attributeMaxDimensionMinus1_             = 2;
  attributeMaxDimensionPartitionsMinus1_   = 0;
  noEightOrientationsConstraintFlag_       = 0;  // Default value, does not impose a constraint
  no45DegreeProjectionPatchConstraintFlag_ = 0;  // Default value, does not impose a constraint

  // reconstuction options
  pixelDeinterleavingType_      = 0;
  pointLocalReconstructionType_ = 0;
  reconstructEomType_           = 0;
  duplicatedPointRemovalType_   = 0;
  reconstructRawType_           = 0;
  applyGeoSmoothingType_        = 1;
  applyAttrSmoothingType_       = 0;
  attrTransferFilterType_       = 1;
  applyOccupanySynthesisType_   = 0;

  // SHVC
  shvcLayerIndex_ = 8;
  shvcRateX_      = 0;
  shvcRateY_      = 0;
}

PCCEncoderParameters::~PCCEncoderParameters() = default;

void PCCEncoderParameters::completePath() {
  if ( !uncompressedDataFolder_.empty() ) {
    if ( !uncompressedDataPath_.empty() ) { uncompressedDataPath_ = uncompressedDataFolder_ + uncompressedDataPath_; }
  }
  if ( !configurationFolder_.empty() ) {
    if ( !geometryConfig_.empty() ) { geometryConfig_ = configurationFolder_ + geometryConfig_; }
    if ( !geometry0Config_.empty() ) { geometry0Config_ = configurationFolder_ + geometry0Config_; }
    if ( !geometry1Config_.empty() ) { geometry1Config_ = configurationFolder_ + geometry1Config_; }
    if ( !attributeConfig_.empty() ) { attributeConfig_ = configurationFolder_ + attributeConfig_; }
    if ( !attribute0Config_.empty() ) { attribute0Config_ = configurationFolder_ + attribute0Config_; }
    if ( !attribute1Config_.empty() ) { attribute1Config_ = configurationFolder_ + attribute1Config_; }
    if ( !geometryAuxVideoConfig_.empty() ) {
      geometryAuxVideoConfig_ = configurationFolder_ + geometryAuxVideoConfig_;
    }
    if ( !attributeAuxVideoConfig_.empty() ) {
      attributeAuxVideoConfig_ = configurationFolder_ + attributeAuxVideoConfig_;
    }

    if ( !inverseColorSpaceConversionConfig_.empty() ) {
      inverseColorSpaceConversionConfig_ = configurationFolder_ + inverseColorSpaceConversionConfig_;
    }
    if ( !colorSpaceConversionConfig_.empty() ) {
      colorSpaceConversionConfig_ = configurationFolder_ + colorSpaceConversionConfig_;
    }
    if ( !occupancyMapConfig_.empty() ) { occupancyMapConfig_ = configurationFolder_ + occupancyMapConfig_; }
    if ( useRawPointsSeparateVideo_ ) {
      if ( !geometryAuxVideoConfig_.empty() ) {
        geometryAuxVideoConfig_ = configurationFolder_ + geometryAuxVideoConfig_;
      }
      if ( !attributeAuxVideoConfig_.empty() ) {
        attributeAuxVideoConfig_ = configurationFolder_ + attributeAuxVideoConfig_;
      }
    }
  }
}

void PCCEncoderParameters::print() {
  std::cout << "+ Parameters" << std::endl;
  std::cout << "\t profileToolSet                             " << profileToolsetIdc_ << std::endl;
  std::cout << "\t profileReconstructionIdc                   " << profileReconstructionIdc_ << std::endl;
  std::cout << "\t rawPointsPatch                             " << rawPointsPatch_ << std::endl;
  std::cout << "\t noAttributes                               " << noAttributes_ << std::endl;
  std::cout << "\t attribute Colour Plane                     " << ( attributeVideo444_ ? "444" : "420" ) << std::endl;
  std::cout << "\t enhancedOccupancyMapCode                   " << enhancedOccupancyMapCode_ << std::endl;
  std::cout << "\t useRawPointsSeparateVideo                  " << useRawPointsSeparateVideo_ << std::endl;
  if ( useRawPointsSeparateVideo_ )
    std::cout << "\t attributeRawSeparateVideoWidth             " << attributeRawSeparateVideoWidth_ << std::endl;
  std::cout << "\t uncompressedDataPath                       " << uncompressedDataPath_ << std::endl;
  std::cout << "\t compressedStreamPath                       " << compressedStreamPath_ << std::endl;
  std::cout << "\t reconstructedDataPath                      " << reconstructedDataPath_ << std::endl;
  std::cout << "\t frameCount                                 " << frameCount_ << std::endl;
  std::cout << "\t mapCountMinus1                             " << mapCountMinus1_ << std::endl;
  std::cout << "\t startFrameNumber                           " << startFrameNumber_ << std::endl;
  std::cout << "\t groupOfFramesSize                          " << groupOfFramesSize_ << std::endl;
  std::cout << "\t colorTransform                             " << colorTransform_ << std::endl;
  std::cout << "\t nbThread                                   " << nbThread_ << std::endl;
  std::cout << "\t keepIntermediateFiles                      " << keepIntermediateFiles_ << std::endl;
  std::cout << "\t multipleStreams                            " << multipleStreams_ << std::endl;
  std::cout << "\t multipleStreams                            " << multipleStreams_ << std::endl;
  std::cout << "\t videoEncoderInternalBitdepth               " << videoEncoderInternalBitdepth_ << std::endl;  
  std::cout << "\t forcedSsvhUnitSizePrecisionBytes           " << forcedSsvhUnitSizePrecisionBytes_ << std::endl;
  if ( multipleStreams_ ) {
    std::cout << "\t    deltaQPD0                               " << deltaQPD0_ << std::endl;
    std::cout << "\t    deltaQPD1                               " << deltaQPD1_ << std::endl;
    std::cout << "\t    deltaQPT0                               " << deltaQPT0_ << std::endl;
    std::cout << "\t    deltaQPT1                               " << deltaQPT1_ << std::endl;
    std::cout << "\t    absoluteD1                              " << absoluteD1_ << std::endl;
    std::cout << "\t    absoluteT1                              " << absoluteT1_ << std::endl;
  }
  std::cout << "\t constrainedPack                            " << constrainedPack_ << std::endl;
  std::cout << "\t maxNumRefPatchList                         " << maxNumRefAtlasList_ << std::endl;
  std::cout << "\t maxNumRefIndex                             " << maxNumRefAtlasFrame_ << std::endl;
  std::cout << "\t Segmentation" << std::endl;
  std::cout << "\t   gridBasedSegmentation                    " << gridBasedSegmentation_ << std::endl;
  std::cout << "\t   voxelDimensionGridBasedSegmentation      " << voxelDimensionGridBasedSegmentation_ << std::endl;
  std::cout << "\t   nnNormalEstimation                       " << nnNormalEstimation_ << std::endl;
  std::cout << "\t   normalOrientation                        " << normalOrientation_ << std::endl;
  std::cout << "\t   gridBasedRefineSegmentation              " << gridBasedRefineSegmentation_ << std::endl;
  std::cout << "\t   maxNNCountRefineSegmentation             " << maxNNCountRefineSegmentation_ << std::endl;
  std::cout << "\t   iterationCountRefineSegmentation         " << iterationCountRefineSegmentation_ << std::endl;
  std::cout << "\t   voxelDimensionRefineSegmentation         " << voxelDimensionRefineSegmentation_ << std::endl;
  std::cout << "\t   searchRadiusRefineSegmentation           " << searchRadiusRefineSegmentation_ << std::endl;
  std::cout << "\t   occupancyResolution                      " << occupancyResolution_ << std::endl;
  std::cout << "\t   enablePatchSplitting                     " << enablePatchSplitting_ << std::endl;
  std::cout << "\t   maxPatchSize                             " << maxPatchSize_ << std::endl;
  std::cout << "\t   quantization step for patch size         "
            << "1<<" << log2QuantizerSizeX_ << ", 1<<" << log2QuantizerSizeY_ << std::endl;
  std::cout << "\t   minPointCountPerCCPatchSegmentation      " << minPointCountPerCCPatchSegmentation_ << std::endl;
  std::cout << "\t   maxNNCountPatchSegmentation              " << maxNNCountPatchSegmentation_ << std::endl;
  std::cout << "\t   surfaceThickness                         " << surfaceThickness_ << std::endl;
  std::cout << "\t   maxAllowedDist2RawPointsDetection        " << maxAllowedDist2RawPointsDetection_ << std::endl;
  std::cout << "\t   maxAllowedDist2RawPointsSelection        " << maxAllowedDist2RawPointsSelection_ << std::endl;
  std::cout << "\t   lambdaRefineSegmentation                 " << lambdaRefineSegmentation_ << std::endl;
  std::cout << "\t   depthQuantizationStep                    " << minLevel_ << std::endl;
  std::cout << "\t   highGradientSeparation                   " << highGradientSeparation_ << std::endl;
  if ( highGradientSeparation_ ) {
    std::cout << "\t     minGradient                          " << minGradient_ << std::endl;
    std::cout << "\t     minNumHighGradientPoints             " << minNumHighGradientPoints_ << std::endl;
  }
  std::cout << "\t Packing" << std::endl;
  std::cout << "\t   minimumImageWidth                        " << minimumImageWidth_ << std::endl;
  std::cout << "\t   minimumImageHeight                       " << minimumImageHeight_ << std::endl;
  std::cout << "\t   log2MaxAtlasFrameOrderCntLsb             " << log2MaxAtlasFrameOrderCntLsb_ << std::endl;
  std::cout << "\t   packingStrategy                          " << packingStrategy_ << std::endl;
  std::cout << "\t   useEightOrientations                     " << useEightOrientations_ << std::endl;
  std::cout << "\t   safeGuardDistance                        " << safeGuardDistance_ << std::endl;
  std::cout << "\t   globalPatchAllocation                    " << globalPatchAllocation_ << std::endl;
  if ( globalPatchAllocation_ == 2 ) {
    std::cout << "\t   globalPackingStrategyGOF               " << geometry0Config_ << std::endl;
    std::cout << "\t   globalPackingStrategyReset             " << geometry1Config_ << std::endl;
    std::cout << "\t   globalPackingStrategyThreshold         " << geometry1Config_ << std::endl;
  }
  std::cout << "\t   patchPrecedenceOrder                     " << patchPrecedenceOrderFlag_ << std::endl;
  std::cout << "\t   lowDelayEncoding                         " << lowDelayEncoding_ << std::endl;
  std::cout << "\t   attributeBGFill                          " << attributeBGFill_ << std::endl;
  std::cout << "\t   geometryPadding                          " << geometryPadding_ << std::endl;
  std::cout << "\t Video encoding" << std::endl;
  std::cout << "\t   geometryQP                               " << geometryQP_ << std::endl;
  std::cout << "\t   attributeQP                              " << attributeQP_ << std::endl;
  std::cout << "\t   usePccRDO                                " << usePccRDO_ << std::endl;
  std::cout << "\t   colorSpaceConversionPath                 " << colorSpaceConversionPath_ << std::endl;
  std::cout << "\t   videoEncoderOccupancyPath                " << videoEncoderOccupancyPath_ << std::endl;
  std::cout << "\t   videoEncoderGeometryPath                 " << videoEncoderGeometryPath_ << std::endl;
  std::cout << "\t   videoEncoderAttributePath                " << videoEncoderAttributePath_ << std::endl;
  std::cout << "\t   videoEncoderOccupancyCodecId             " << videoEncoderOccupancyCodecId_ << std::endl;
  std::cout << "\t   videoEncoderGeometryCodecId              " << videoEncoderGeometryCodecId_ << std::endl;
  std::cout << "\t   videoEncoderAttributeCodecId             " << videoEncoderAttributeCodecId_ << std::endl;
  if ( multipleStreams_ ) {
    std::cout << "\t   geometry0Config                          " << geometry0Config_ << std::endl;
    std::cout << "\t   geometry1Config                          " << geometry1Config_ << std::endl;
  } else {
    std::cout << "\t   geometryConfig                           " << geometryConfig_ << std::endl;
  }
  if ( multipleStreams_ ) {
    std::cout << "\t   attribute0Config                         " << attribute0Config_ << std::endl;
    std::cout << "\t   attribute1Config                         " << attribute1Config_ << std::endl;
  } else {
    std::cout << "\t   attributeConfig                           " << attributeConfig_ << std::endl;
  }
  if ( useRawPointsSeparateVideo_ ) {
    if ( rawPointsPatch_ ) {
      std::cout << "\t   geometryAuxVideoConfig                       " << geometryAuxVideoConfig_ << std::endl;
      std::cout << "\t   attributeAuxVideoConfig                      " << attributeAuxVideoConfig_ << std::endl;
    }
    std::cout << "\t   auxGeometryQP                                " << auxGeometryQP_ << std::endl;
    std::cout << "\t   auxAttributeQP                               " << auxAttributeQP_ << std::endl;
  }
  std::cout << "\t   colorSpaceConversionConfig               " << colorSpaceConversionConfig_ << std::endl;
  std::cout << "\t   inverseColorSpaceConversionConfig        " << inverseColorSpaceConversionConfig_ << std::endl;
  std::cout << "\t   apply3dMotionCompensation                " << use3dmc_ << std::endl;
  std::cout << "\t Dilation" << std::endl;
  std::cout << "\t   GroupDilation                            " << groupDilation_ << std::endl;
  std::cout << "\t Occupancy map encoding " << std::endl;
  std::cout << "\t   maxCandidateCount                        " << maxCandidateCount_ << std::endl;
  std::cout << "\t   occupancyPrecision                       " << occupancyPrecision_ << std::endl;
  std::cout << "\t   occupancyMapConfig           " << occupancyMapConfig_ << std::endl;
  std::cout << "\t   occupancyMapQP                           " << occupancyMapQP_ << std::endl;
  std::cout << "\t   EOMFixBitCount                           " << EOMFixBitCount_ << std::endl;
  std::cout << "\t   occupancyMapRefinement                   " << occupancyMapRefinement_ << std::endl;
  std::cout << "\t Lossy occupancy Map coding" << std::endl;
  std::cout << "\t   Lossy occupancy map offset               " << offsetLossyOM_ << std::endl;
  std::cout << "\t   Lossy occupancy map threshold            " << thresholdLossyOM_ << std::endl;
  std::cout << "\t   Lossy occupancy map prefilter            " << prefilterLossyOM_ << std::endl;
  std::cout << "\t Decoded Atlas Information Hash             " << ( decodedAtlasInformationHash_ > 0 ? 1 : 0 )
            << std::endl;
  if ( decodedAtlasInformationHash_ > 0 ) {
    std::cout << "\t   DecodedAtlasInformationHash Type             " << decodedAtlasInformationHash_ << std::endl;
  }
  std::cout << "\t Geometry smoothing                         " << std::endl;
  std::cout << "\t   flagGeometrySmoothing                    " << flagGeometrySmoothing_ << std::endl;
  if ( flagGeometrySmoothing_ ) {
    std::cout << "\t   gridSmoothing                            " << gridSmoothing_ << std::endl;
    if ( gridSmoothing_ ) {
      std::cout << "\t   gridSize                                 " << gridSize_ << std::endl;
      std::cout << "\t   thresholdSmoothing                       " << thresholdSmoothing_ << std::endl;
    } else {
      std::cout << "\t   neighborCountSmoothing                   " << neighborCountSmoothing_ << std::endl;
      std::cout << "\t   radius2Smoothing                         " << radius2Smoothing_ << std::endl;
      std::cout << "\t   radius2BoundaryDetection                 " << radius2BoundaryDetection_ << std::endl;
      std::cout << "\t   thresholdSmoothing                       " << thresholdSmoothing_ << std::endl;
    }
  }
  std::cout << "\t   patchExpansion                           " << patchExpansion_ << std::endl;
  std::cout << "\t Color smoothing" << std::endl;
  std::cout << "\t   flagColorSmoothing                       " << flagColorSmoothing_ << std::endl;
  if ( flagColorSmoothing_ ) {
    std::cout << "\t   thresholdColorSmoothing                  " << thresholdColorSmoothing_ << std::endl;
    std::cout << "\t   thresholdColorDifference                 " << thresholdColorDifference_ << std::endl;
    std::cout << "\t   thresholdColorVariation                  " << thresholdColorVariation_ << std::endl;
    std::cout << "\t   cgridSize                                " << cgridSize_ << std::endl;
  }
  std::cout << "\t Color pre-smoothing                        " << std::endl;
  std::cout << "\t   thresholdColorPreSmoothing               " << thresholdColorSmoothing_ << std::endl;
  std::cout << "\t   thresholdColorPreSmoothingLocalEntropy   " << thresholdColorPreSmoothingLocalEntropy_ << std::endl;
  std::cout << "\t   radius2ColorPreSmoothing                 " << radius2ColorPreSmoothing_ << std::endl;
  std::cout << "\t   neighborCountColorPreSmoothing           " << neighborCountColorPreSmoothing_ << std::endl;
  std::cout << "\t   flagColorPreSmoothing                    " << flagColorPreSmoothing_ << std::endl;
  std::cout << "\t Coloring" << std::endl;
  std::cout << "\t   bestColorSearchRange                     " << bestColorSearchRange_ << std::endl;
  std::cout << "\t   numNeighborsColorTransferFwd             " << numNeighborsColorTransferFwd_ << std::endl;
  std::cout << "\t   numNeighborsColorTransferBwd             " << numNeighborsColorTransferBwd_ << std::endl;
  std::cout << "\t   useDistWeightedAverageFwd                " << useDistWeightedAverageFwd_ << std::endl;
  std::cout << "\t   useDistWeightedAverageBwd                " << useDistWeightedAverageBwd_ << std::endl;
  std::cout << "\t   skipAvgIfIdenticalSourcePointPresentFwd  " << skipAvgIfIdenticalSourcePointPresentFwd_
            << std::endl;
  std::cout << "\t   skipAvgIfIdenticalSourcePointPresentBwd  " << skipAvgIfIdenticalSourcePointPresentBwd_
            << std::endl;
  std::cout << "\t   distOffsetFwd                            " << distOffsetFwd_ << std::endl;
  std::cout << "\t   distOffsetBwd                            " << distOffsetBwd_ << std::endl;
  std::cout << "\t   geometryDist2ThresholdFwd                " << maxGeometryDist2Fwd_ << std::endl;
  std::cout << "\t   geometryDist2ThresholdBwd                " << maxGeometryDist2Bwd_ << std::endl;
  std::cout << "\t   maxColorDist2Fwd                         " << maxColorDist2Fwd_ << std::endl;
  std::cout << "\t   maxColorDist2Bwd                         " << maxColorDist2Bwd_ << std::endl;
  std::cout << "\t   patchColorSubsampling                    " << patchColorSubsampling_ << std::endl;
  std::cout << "\t   excludeColorOutlier                      " << excludeColorOutlier_ << std::endl;
  if ( excludeColorOutlier_ ) {
    std::cout << "\t     thresholdColorOutlierDist            " << thresholdColorOutlierDist_ << std::endl;
  }
  std::cout << "\t Reconstruction " << std::endl;
  std::cout << "\t   removeDuplicatePoints                    " << removeDuplicatePoints_ << std::endl;
  std::cout << "\t   pointLocalReconstruction                 " << pointLocalReconstruction_ << std::endl;
  std::cout << "\t     plrlNumberOfModes                      " << plrlNumberOfModes_ << std::endl;
  std::cout << "\t     patchSize                              " << patchSize_ << std::endl;
  std::cout << "\t   singleMapPixelInterleaving               " << singleMapPixelInterleaving_ << std::endl;
  std::cout << "\t   rawPointReconstruction                   " << reconstructRawType_ << std::endl;
  std::cout << "\t   eomPointReconstruction                   " << reconstructEomType_ << std::endl;
  std::cout << "\t   attrTransferFilter                       " << attrTransferFilterType_ << std::endl;
  std::cout << "\t surface Separation                         " << surfaceSeparation_ << std::endl;
  std::cout << "\t Lossy raw points patch                     " << std::endl;
  std::cout << "\t   lossyRawPointsPatch                      " << lossyRawPointsPatch_ << std::endl;
  std::cout << "\t   minNormSumOfInvDist4MPSelection          " << minNormSumOfInvDist4MPSelection_ << std::endl;
  std::cout << "\t   lossyRawPointPatchGeoQP                  " << auxGeometryQP_ << std::endl;
  std::cout << "\t raw points sorting                         " << std::endl;
  std::cout << "\t   mortonOrderSortRawPoints                 " << mortonOrderSortRawPoints_ << std::endl;
  std::cout << "\t Enhanced projection plane                  " << enhancedPP_ << std::endl;
  std::cout << "\t AdditionalProjectionPlane                  " << std::endl;
  std::cout << "\t   additionalProjectionPlaneMode            " << additionalProjectionPlaneMode_ << std::endl;
  std::cout << "\t   partialAdditionalProjectionPlane         " << partialAdditionalProjectionPlane_ << std::endl;
  std::cout << "\t Geometry 2D and 3D bitdepths               " << std::endl;
  std::cout << "\t   geometry3dCoordinatesBitdepth            " << geometry3dCoordinatesBitdepth_ << std::endl;
  std::cout << "\t   geometryNominal2dBitdepth                " << geometryNominal2dBitdepth_ << std::endl;
  std::cout << "\t Image partitions and tiles                 " << std::endl;
  std::cout << "\t tileSegmentationType                       " << tileSegmentationType_ << std::endl;
  if ( tileSegmentationType_ > 1 ) {
    std::cout << "\t   numMaxTilePerFrame                       " << numMaxTilePerFrame_ << std::endl;
    if ( numMaxTilePerFrame_ > 1 ) {
      if ( uniformPartitionSpacing_ ) {
        std::cout << "\t   uniformPartitionSpacing                " << uniformPartitionSpacing_ << std::endl;
        std::cout << "\t   tilePartitionWidth                     " << tilePartitionWidth_ << std::endl;
        std::cout << "\t   tilePartitionHeight                    " << tilePartitionHeight_ << std::endl;
      } else {
        std::cout << "\t   tilePartitionWidthList                   ";
        for ( auto v : tilePartitionWidthList_ ) std::cout << v << "\t";
        std::cout << std::endl;
        std::cout << "\t   tilePartitionHeightList                  ";
        for ( auto v : tilePartitionHeightList_ ) std::cout << v << "\t";
        std::cout << std::endl;
      }
    }
  }
  std::cout << "\t Point cloud partitions and tiles           " << std::endl;
  std::cout << "\t   enablePointCloudPartitioning             " << enablePointCloudPartitioning_ << std::endl;
  if ( enablePointCloudPartitioning_ ) {
    auto printVector = []( std::vector<int>& vec ) {
      for ( int i = 0; i < vec.size(); ++i ) {
        std::cout << vec[i];
        if ( i < vec.size() - 1 ) { std::cout << ","; }
      }
      std::cout << std::endl;
    };
    std::cout << "\t   roiBoundingBoxMinX                       ";
    printVector( roiBoundingBoxMinX_ );
    std::cout << "\t   roiBoundingBoxMaxX                       ";
    printVector( roiBoundingBoxMaxX_ );
    std::cout << "\t   roiBoundingBoxMinY                       ";
    printVector( roiBoundingBoxMinY_ );
    std::cout << "\t   roiBoundingBoxMaxY                       ";
    printVector( roiBoundingBoxMaxY_ );
    std::cout << "\t   roiBoundingBoxMinZ                       ";
    printVector( roiBoundingBoxMinZ_ );
    std::cout << "\t   roiBoundingBoxMaxZ                       ";
    printVector( roiBoundingBoxMaxZ_ );
    std::cout << "\t   numTilesHor                              " << numTilesHor_ << std::endl;
    std::cout << "\t   tileHeightToWidthRatio                   " << tileHeightToWidthRatio_ << std::endl;
    std::cout << "\t   numCutsAlong1stLongestAxis               " << numCutsAlong1stLongestAxis_ << std::endl;
    std::cout << "\t   numCutsAlong2ndLongestAxis               " << numCutsAlong2ndLongestAxis_ << std::endl;
    std::cout << "\t   numCutsAlong3rdLongestAxis               " << numCutsAlong3rdLongestAxis_ << std::endl;
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
  std::cout << "\t   pbfEnableFlag                            " << pbfEnableFlag_ << std::endl;
  std::cout << "\t   pbfPassesCount                           " << pbfPassesCount_ << std::endl;
  std::cout << "\t   pbfFilterSize                            " << pbfFilterSize_ << std::endl;
  std::cout << "\t   pbfLog2Threshold                         " << pbfLog2Threshold_ << std::endl;

  std::cout << "\t Profile tier level" << std::endl;
  std::cout << "\t   tierFlag                                 " << tierFlag_ << std::endl;
  std::cout << "\t   profileCodecGroupIdc                     " << profileCodecGroupIdc_ << std::endl;
  std::cout << "\t   profileToolsetIdc                        " << profileToolsetIdc_ << std::endl;
  std::cout << "\t   profileReconstructionIdc                 " << profileReconstructionIdc_ << std::endl;
  std::cout << "\t      pixelDeinterleavingType             : " << pixelDeinterleavingType_ << std::endl;
  std::cout << "\t      pointLocalReconstructionType        : " << pointLocalReconstructionType_ << std::endl;
  std::cout << "\t      reconstructEomType                  : " << reconstructEomType_ << std::endl;
  std::cout << "\t      duplicatedPointRemovalType          : " << duplicatedPointRemovalType_ << std::endl;
  std::cout << "\t      reconstructRawType                  : " << reconstructRawType_ << std::endl;
  std::cout << "\t      applyGeoSmoothingType               : " << applyGeoSmoothingType_ << std::endl;
  std::cout << "\t      applyAttrSmoothingType              : " << applyAttrSmoothingType_ << std::endl;
  std::cout << "\t      applyAttrTransferFilterType         : " << attrTransferFilterType_ << std::endl;
  std::cout << "\t      applyOccupanySynthesisType          : " << applyOccupanySynthesisType_ << std::endl;

  std::cout << "\t   levelIdc                                 " << levelIdc_ << std::endl;
  std::cout << "\t   avcCodecIdIndex                          " << avcCodecIdIndex_ << std::endl;
  std::cout << "\t   hevcCodecIdIndex                         " << hevcCodecIdIndex_ << std::endl;
  std::cout << "\t   shvcCodecIdIndex                         " << shvcCodecIdIndex_ << std::endl;
  std::cout << "\t   vvcCodecIdIndex                          " << vvcCodecIdIndex_ << std::endl;

  if ( shvcRateX_ > 0 || shvcRateY_ > 0 ) {
    std::cerr << "HEVC scalable video coding (SHVC) " << std::endl;
    std::cout << "\t   shvcLayerIndex                         " << shvcLayerIndex_ << std::endl;
    std::cout << "\t   shvcRateX                              " << shvcRateX_ << std::endl;
    std::cout << "\t   shvcRateY                              " << shvcRateY_ << std::endl;
  }
  std::cout << std::endl;
}

bool PCCEncoderParameters::check() {
  bool ret = true;

  // Profile Tools set idc
  // Basic
  if ( profileToolsetIdc_ == 0 ) {
    if ( enhancedOccupancyMapCode_ == 1 ) {
      enhancedOccupancyMapCode_ = false;
      std::cerr << "enhancedOccupancyMapCode is set to 0 because profileToolsetIdc is 0. \n";
    }
    if ( multipleStreams_ == 0 && mapCountMinus1_ > 0 ) {
      multipleStreams_ = true;
      std::cerr << "multipleStreams is set to 1 because profileToolsetIdc is 0. \n";
    }
    if ( pointLocalReconstruction_ == 1 ) {
      pointLocalReconstruction_ = false;
      std::cerr << "pointLocalReconstruction is set to 0 because profileToolsetIdc is 0. \n";
    }
    if ( useEightOrientations_ == 1 ) {
      useEightOrientations_ = false;
      std::cerr << "useEightOrientations is set to 0 because profileToolsetIdc is 0. \n";
    }
    if ( additionalProjectionPlaneMode_ != 0 || partialAdditionalProjectionPlane_ != 0 ) {
      // asps_extended_projection_enabled_flag =0
      additionalProjectionPlaneMode_    = 0;
      partialAdditionalProjectionPlane_ = 0;
      std::cerr << "additionalProjectionPlaneMode and partialAdditionalProjectionPlane are set to 0 because "
                   "profileToolsetIdc is 0. \n";
    }

    // constraint options
    EOMContraintFlag_                        = true;
    multipleMapStreamsConstraintFlag_        = false;
    PLRConstraintFlag_                       = true;
    noEightOrientationsConstraintFlag_       = true;
    no45DegreeProjectionPatchConstraintFlag_ = true;
  } else {
    // constraint options
    EOMContraintFlag_                        = false;
    multipleMapStreamsConstraintFlag_        = false;
    PLRConstraintFlag_                       = false;
    noEightOrientationsConstraintFlag_       = false;
    no45DegreeProjectionPatchConstraintFlag_ = false;
  }

  // constraints by profile_toolset_constraints_information( )
  if ( oneV3CFrameOnlyFlag_ ) {
    if ( frameCount_ != 1 ) std::cerr << "frameCount is set to 1 ptci.oneV3CFrameOnlyFlag is 1. \n";
    frameCount_ = 1;
  }
  if ( EOMContraintFlag_ ) {
    if ( enhancedOccupancyMapCode_ )
      std::cerr << "enhancedOccupancyMapCode_ is set to 0 because ptci.EOMContraintFlag is 1. \n";
    enhancedOccupancyMapCode_ = false;
  }
  if ( multipleMapStreamsConstraintFlag_ ) {
    if ( multipleStreams_ )
      std::cerr << "multipleStreams_ is set to 0 because ptci.multipleMapStreamsConstraintFlag is 1. \n";
    multipleStreams_ = false;
  }
  if ( PLRConstraintFlag_ ) {
    if ( pointLocalReconstruction_ )
      std::cerr << "pointLocalReconstruction is set to 0 because ptci.PLRConstraintFlag is 1. \n";
    pointLocalReconstruction_ = false;
  }
  if ( noEightOrientationsConstraintFlag_ ) {
    if ( useEightOrientations_ )
      std::cerr << "useEightOrientations is set to 0 because ptci.noEightOrientationsConstraintFlag is 1. \n";
    useEightOrientations_ = false;
  }
  if ( no45DegreeProjectionPatchConstraintFlag_ ) {
    if ( additionalProjectionPlaneMode_ != 0 || partialAdditionalProjectionPlane_ != 0 )
      std::cerr << "additionalProjectionPlaneMode and partialAdditionalProjectionPlane are set to 0 because "
                   "ptci.no45DegreeProjectionPatchConstraintFlag is 1. \n";
    additionalProjectionPlaneMode_    = 0;
    partialAdditionalProjectionPlane_ = 0;
  }

  if ( mapCountMinus1_ > maxMapCountMinus1_ ) {
    if ( mapCountMinus1_ > maxMapCountMinus1_ )
      std::cerr << "mapCountMinus1_ are set to " << mapCountMinus1_ << " because ptci.maxMapCountMinus1_ is "
                << maxMapCountMinus1_ << ". \n";
    mapCountMinus1_ = maxMapCountMinus1_;
  }
  // Profile reconctruction idc
  // Rec0
  if ( profileReconstructionIdc_ == 0 ) {
    if ( singleMapPixelInterleaving_ ) {
      singleMapPixelInterleaving_ = false;
      std::cout << "singleMapPixelInterleaving is ignored because profileReconstructionIdc set to 0. \n";
    }
    if ( pointLocalReconstruction_ ) {
      pointLocalReconstruction_ = false;
      std::cout << "pointLocalReconstruction is ignored because profileReconstructionIdc set to 0. \n";
    }
    if ( enhancedOccupancyMapCode_ ) {
      enhancedOccupancyMapCode_ = 0;
      std::cout << "enhancedOccupancyMapCode is ignored because profileReconstructionIdc set to 0. \n";
    }
    // removeDuplicatePoints_ is removed
    if ( removeDuplicatePoints_ ) {
      removeDuplicatePoints_ = 0;
      std::cout << "removeDuplicatePoints is ignored because profileReconstructionIdc set to 0. \n";
    }
    // rawPointsPatch_ is removed
    if ( lossyRawPointsPatch_ ) {
      lossyRawPointsPatch_ = 0;
      std::cout << "lossyRawPointsPatch is ignored because profileReconstructionIdc set to 0. \n";
    }
    if ( flagGeometrySmoothing_ ) {
      flagGeometrySmoothing_ = false;
      std::cout << "flagGeometrySmoothing is ignored because profileReconstructionIdc set to 0. \n";
    }
    if ( gridSmoothing_ ) {
      gridSmoothing_ = false;
      std::cout << "gridSmoothing is ignored because profileReconstructionIdc set to 0. \n";
    }
    if ( flagColorSmoothing_ ) {
      flagColorSmoothing_ = false;
      std::cout << "flagColorSmoothing is ignored because profileReconstructionIdc set to 0. \n";
    }
    if ( attrTransferFilterType_ ) {
      attrTransferFilterType_ = 0;
      std::cout << "attrTransferFilterType is ignored because profileReconstructionIdc set to 0. \n";
    }
    if ( pbfEnableFlag_ ) {
      pbfEnableFlag_ = false;
      std::cout << "pbfEnableFlag is ignored because profileReconstructionIdc set to 0. \n";
    }

    pixelDeinterleavingType_      = 0;
    pointLocalReconstructionType_ = 0;
    reconstructEomType_           = 0;
    duplicatedPointRemovalType_   = 0;
    reconstructRawType_           = 0;
    applyGeoSmoothingType_        = 0;
    applyAttrSmoothingType_       = 0;
    attrTransferFilterType_       = 0;
    applyOccupanySynthesisType_   = 0;
  }

  // Rec1
  if ( profileReconstructionIdc_ == 1 ) {
    if ( pbfEnableFlag_ ) {
      pbfEnableFlag_ = false;
      std::cout << "pbfEnableFlag is ignored because profileReconstructionIdc set to 1. \n";
    }
    pixelDeinterleavingType_      = 1;
    pointLocalReconstructionType_ = 1;
    reconstructEomType_           = 1;
    duplicatedPointRemovalType_   = 1;
    reconstructRawType_           = 1;
    applyGeoSmoothingType_        = 1;
    applyAttrSmoothingType_       = 1;
    attrTransferFilterType_       = 1;
    applyOccupanySynthesisType_   = 0;
  }

  // Rec2
  if ( profileReconstructionIdc_ == 2 ) {
    if ( gridSmoothing_ ) {
      gridSmoothing_ = false;
      std::cout << "gridSmoothing_ is ignored because profileReconstructionIdc set to 2. \n";
    }
    pixelDeinterleavingType_      = 1;
    pointLocalReconstructionType_ = 1;
    reconstructEomType_           = 1;
    duplicatedPointRemovalType_   = 1;
    reconstructRawType_           = 1;
    applyGeoSmoothingType_        = 0;
    applyAttrSmoothingType_       = 1;
    attrTransferFilterType_       = 0;
    applyOccupanySynthesisType_   = 1;
  }

  if ( ( levelOfDetailX_ == 0 || levelOfDetailY_ == 0 ) ) {
    if ( levelOfDetailX_ == 0 ) { levelOfDetailX_ = 1; }
    if ( levelOfDetailY_ == 0 ) { levelOfDetailY_ = 1; }
    std::cerr << "levelOfDetailX and levelOfDetailY should be greater than 1, increase in these values." << std::endl;
  }
  if ( rawPointsPatch_ && ( levelOfDetailX_ > 1 || levelOfDetailY_ > 1 ) ) {
    levelOfDetailX_ = 1;
    levelOfDetailY_ = 1;
    std::cerr << "scaling is not allowed in lossless case\n";
  }
  if ( enablePointCloudPartitioning_ && patchExpansion_ ) {
    std::cerr << "Point cloud partitioning does not currently support patch expansion. \n";
  }
  if ( tileSegmentationType_ == 1 ) {
    if ( enablePointCloudPartitioning_ != 1 ) {
      enablePointCloudPartitioning_ = 1;
      std::cerr << "enablePointCloudPartitioning should be 1 when tileSegmentationType is 1.\n";
    }
  }
  if ( tileSegmentationType_ == 2 ) {
    if ( enablePointCloudPartitioning_ == 1 ) {
      enablePointCloudPartitioning_ = 0;
      std::cerr << "enablePointCloudPartitioning should be 0 when tileSegmentationType is 2.\n";
    }
  }
  if ( tileSegmentationType_ != 1 && enablePointCloudPartitioning_ && ( globalPatchAllocation_ != 0 ) ) {
    std::cerr << "Point cloud partitioning does not currently support global patch allocation. \n";
  }
  if ( static_cast<int>( patchPrecedenceOrderFlag_ ) == 0 && lowDelayEncoding_ ) {
    lowDelayEncoding_ = false;
    std::cerr << "Low delay encoding can be used only when patchPrecendenceOrder is enabled. \n";
  }
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
    if ( geometry0Config_.empty() || geometry1Config_.empty() ) {
      geometry0Config_ = geometryConfig_.substr( 0, geometryConfig_.find_last_of( '.' ) ) + "-D0.cfg";
      geometry1Config_ = geometryConfig_.substr( 0, geometryConfig_.find_last_of( '.' ) ) + "-D1.cfg";
    }
  } else {
    geometry0Config_ = {};
    geometry1Config_ = {};
    if ( geometryConfig_.empty() ) {
      ret = false;
      std::cerr << "When multipleStreams is not true, geometryConfig_ should "
                   "be non-empty\n";
    }
  }

  if ( multipleStreams_ ) {
    if ( attribute0Config_.empty() || attribute1Config_.empty() ) {
      attribute0Config_ = attributeConfig_.substr( 0, attributeConfig_.find_last_of( '.' ) ) + "-T0.cfg";
      attribute1Config_ = attributeConfig_.substr( 0, attributeConfig_.find_last_of( '.' ) ) + "-T1.cfg";
    }
  } else {
    attribute0Config_ = {};
    attribute1Config_ = {};
    if ( attributeConfig_.empty() ) {
      std::cerr << "When multipleStreams is false, attributeConfig_ should be "
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
  if ( rawPointsPatch_ ) {
    if ( pbfEnableFlag_ ) {
      pbfEnableFlag_ = false;
      std::cerr << "WARNING: pbfEnableFlag_ is only for lossy "
                   "coding mode for now. Force pbfEnableFlag_=FALSE.\n";
    }
    if ( occupancyMapRefinement_ ) {
      occupancyMapRefinement_ = false;
      std::cerr << "WARNING: occupancyMapRefinement_ is only for lossy "
                   "coding mode for now. Force pbfEnableFlag_=FALSE.\n";
    }
    if ( flagGeometrySmoothing_ ) {
      flagGeometrySmoothing_ = false;
      std::cerr << "WARNING: flagGeometrySmoothing_ is only for lossy "
                   "coding mode for now. Force flagGeometrySmoothing_=FALSE.\n";
    }
    if ( gridSmoothing_ ) {
      gridSmoothing_ = false;
      std::cerr << "WARNING: gridSmoothing_ is only for lossy "
                   "coding mode for now. Force gridSmoothing_=FALSE.\n";
    }
    if ( flagColorSmoothing_ ) {
      gridSmoothing_ = false;
      std::cerr << "WARNING: flagColorSmoothing_ is only for lossy "
                   "coding mode for now. Force flagColorSmoothing_=FALSE.\n";
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

  if ( useRawPointsSeparateVideo_ && !lossyRawPointsPatch_ && !rawPointsPatch_ ) {
    useRawPointsSeparateVideo_ = false;
    std::cerr << "WARNING: useRawPointsSeparateVideo_ is for raw point coding. These modes are not enabled, force "
                 "useRawPointsSeparateVideo_=false.\n";
  }

  if ( useRawPointsSeparateVideo_ ) {
    if ( auxAttributeQP_ == 0 ) { auxAttributeQP_ = attributeQP_; }
    if ( auxGeometryQP_ == 0 ) { auxGeometryQP_ = geometryQP_; }
    if ( lossyRawPointsPatch_ ) { auxGeometryQP_ = 4; }
    if ( ( attributeRawSeparateVideoWidth_ % 64 ) != 0U ) {
      ret = false;
      std::cerr << "attributeRawSeparateVideoWidth_ must be multiple of 64.\n";
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
    if ( attributeAuxVideoConfig_.empty() || !exist( attributeAuxVideoConfig_ ) ) {
      if ( !attributeConfig_.empty() ) {
        std::cerr << "WARNING: attributeAuxVideoConfig_ is set as attributeConfig_ : " << attributeConfig_ << std::endl;
        attributeAuxVideoConfig_ = attributeConfig_;
      } else {
        ret = false;
        std::cerr << "attributeConfig_ and attributeAuxVideoConfig_ are empty\n";
      }

    } else
      std::cout << "attributeAuxConfig: " << attributeAuxVideoConfig_ << std::endl;
  }

  if ( singleMapPixelInterleaving_ && pointLocalReconstruction_ ) {
    ret = false;
    std::cerr << "Pixel Interleaving and Point local reconstruction can not be "
                 "use in the same time.\n";
  }

  if ( globalPatchAllocation_ == 1 && constrainedPack_ == 0 ) {
    std::cout << " globalPatchAllocation_ is set 1 but constrainedPack_ is disable. Force "
                 "constrainedPack_ = 1. \n";
    constrainedPack_ = true;
  }

  if ( mapCountMinus1_ != 0 ) {
    if ( singleMapPixelInterleaving_ ) {
      ret = false;
      std::cerr << "Pixel Interleaving is built on one layer coding. "
                   "Current mapCountMinus1_ is not 0.\n";
    }
  } else {
    if ( multipleStreams_ ) {
      std::cout << " multipleStreams is set 0 when mapCountMinus1 is 0. Force "
                   "multipleStreams_= 0."
                << std::endl;
      multipleStreams_ = false;
    }
  }

  if ( occupancyMapConfig_.empty() ) {
    ret = false;
    std::cerr << "Occupancy map configuration must be defined \n";
  }
  if ( lossyRawPointsPatch_ ) {
    if ( !useRawPointsSeparateVideo_ ) {
      std::cerr
          << "Lossy raw points patch in the same video frame as the regular patches is not optimized as of now.\n";
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
    if ( tilePartitionWidth_ == 0 || tilePartitionHeight_ == 0 ) {
      std::cerr << " tilePartitionWidth_ and tilePartitionHeight_ are set as 1 (64x64 pixels)\n";
      if ( tilePartitionWidth_ == 0 ) tilePartitionWidth_ = 1;
      if ( tilePartitionHeight_ == 0 ) tilePartitionHeight_ = 1;
    }
    numMaxTilePerFrame_ = roiBoundingBoxMaxX_.size();
    std::cerr << "numMaxTilePerFrame_ is set as " << numMaxTilePerFrame_ << "\n";
    if ( roiBoundingBoxMaxX_.size() != numMaxTilePerFrame_ || roiBoundingBoxMaxY_.size() != numMaxTilePerFrame_ ||
         roiBoundingBoxMaxZ_.size() != numMaxTilePerFrame_ || roiBoundingBoxMinX_.size() != numMaxTilePerFrame_ ||
         roiBoundingBoxMinY_.size() != numMaxTilePerFrame_ || roiBoundingBoxMinZ_.size() != numMaxTilePerFrame_ ) {
      std::cerr << "roiBoundingBox need to have same number of elements\n";
      exit( -1 );
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

  if ( noAttributes_ ) {
    flagColorPreSmoothing_ = false;
    flagColorSmoothing_    = false;
    std::cerr << "Color smoothings are disable because noAttributes is true \n";
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

  // Based on profileCodecGroupIdc_, fixe the used video codecs.
  if ( profileCodecGroupIdc_ == CODEC_GROUP_HEVC_MAIN10 && attributeVideo444_ ) {
    profileCodecGroupIdc_ = CODEC_GROUP_HEVC444;
    std::cerr << "geometry/attributeVideo444 is enabled force profileCodecGroupIdc to CODEC_GROUP_HEVC444\n";
  }
  switch ( profileCodecGroupIdc_ ) {
    case CODEC_GROUP_AVC_PROGRESSIVE_HIGH:
#if defined( USE_JMAPP_VIDEO_CODEC ) && defined( USE_JMLIB_VIDEO_CODEC )
      if ( ( videoEncoderOccupancyCodecId_ != JMLIB && videoEncoderOccupancyCodecId_ != JMAPP ) ||
           ( videoEncoderGeometryCodecId_  != JMLIB && videoEncoderGeometryCodecId_ != JMAPP ) ||
           ( videoEncoderAttributeCodecId_ != JMLIB && videoEncoderAttributeCodecId_ != JMAPP ) ) {
        std::cerr << "profileCodecGroupIdc_ is CODEC_GROUP_AVC_PROGRESSIVE_HIGH force codecId. \n";
        videoEncoderOccupancyCodecId_ = JMLIB;
        videoEncoderGeometryCodecId_  = JMLIB;
        videoEncoderAttributeCodecId_ = JMLIB;
      }
#elif defined( USE_JMAPP_VIDEO_CODEC )
      if ( ( videoEncoderOccupancyCodecId_ != JMAPP ) || ( videoEncoderGeometryCodecId_ != JMAPP ) ||
           ( videoEncoderAttributeCodecId_ != JMAPP ) ) {
        std::cerr << "profileCodecGroupIdc_ is CODEC_GROUP_AVC_PROGRESSIVE_HIGH force codecId. \n";
        videoEncoderOccupancyCodecId_ = JMAPP;
        videoEncoderGeometryCodecId_  = JMAPP;
        videoEncoderAttributeCodecId_ = JMAPP;
      }
#elif defined( USE_JMLIB_VIDEO_CODEC )
      if ( ( videoEncoderOccupancyCodecId_ != JMLIB ) || ( videoEncoderGeometryCodecId_ != JMLIB ) ||
           ( videoEncoderAttributeCodecId_ != JMLIB ) ) {
        std::cerr << "profileCodecGroupIdc_ is CODEC_GROUP_AVC_PROGRESSIVE_HIGH force codecId. \n";
        videoEncoderOccupancyCodecId_ = JMLIB;
        videoEncoderGeometryCodecId_  = JMLIB;
        videoEncoderAttributeCodecId_ = JMLIB;
      }
#else
      std::cerr << "profileCodecGroupIdc_ is AVC_PROGRESSIVE_HIGH, not supported. \n";
      ret = false;
#endif
      break;
    case CODEC_GROUP_HEVC444:
    case CODEC_GROUP_HEVC_MAIN10:
#if defined( USE_HMAPP_VIDEO_CODEC ) & defined( USE_HMLIB_VIDEO_CODEC )
      if ( ( videoEncoderOccupancyCodecId_ != HMLIB && videoEncoderOccupancyCodecId_ != HMAPP ) ||
           ( videoEncoderGeometryCodecId_ != HMLIB && videoEncoderGeometryCodecId_ != HMAPP ) ||
           ( videoEncoderAttributeCodecId_ != HMLIB && videoEncoderAttributeCodecId_ != HMAPP ) ) {
        std::cerr << "profileCodecGroupIdc_ is HEVC444 or HEVCMAIN10 force codecId. \n";
        videoEncoderOccupancyCodecId_ = HMLIB;
        videoEncoderGeometryCodecId_  = HMLIB;
        videoEncoderAttributeCodecId_ = HMLIB;
      }
#elif defined( USE_HMAPP_VIDEO_CODEC )
      if ( ( videoEncoderOccupancyCodecId_ != HMAPP ) || ( videoEncoderGeometryCodecId_ != HMAPP ) ||
           ( videoEncoderAttributeCodecId_ != HMAPP ) ) {
        std::cerr << "profileCodecGroupIdc_ is HEVC444 or HEVCMAIN10 force codecId. \n";
        videoEncoderOccupancyCodecId_ = HMAPP;
        videoEncoderGeometryCodecId_  = HMAPP;
        videoEncoderAttributeCodecId_ = HMAPP;
      }
#elif defined( USE_HMAPP_VIDEO_CODEC )
      if ( ( videoEncoderOccupancyCodecId_ != HMLIB ) || ( videoEncoderGeometryCodecId_ != HMLIB ) ||
           ( videoEncoderAttributeCodecId_ != HMLIB ) ) {
        std::cerr << "profileCodecGroupIdc_ is HEVC444 or HEVCMAIN10 force codecId. \n";
        videoEncoderOccupancyCodecId_ = HMLIB;
        videoEncoderGeometryCodecId_  = HMLIB;
        videoEncoderAttributeCodecId_ = HMLIB;
      }
#else
      std::cerr << "profileCodecGroupIdc_ is HEVC444 or HEVCMAIN10, not supported. \n";
      ret = false;
#endif
      break;

    case CODEC_GROUP_VVC_MAIN10:
#if defined( USE_VTMLIB_VIDEO_CODEC ) && defined( USE_VVLIB_VIDEO_CODEC )
      if ( ( videoEncoderOccupancyCodecId_ != VTMLIB && videoEncoderOccupancyCodecId_ != VVLIB ) ||
           ( videoEncoderGeometryCodecId_ != VTMLIB && videoEncoderGeometryCodecId_ != VVLIB ) ||
           ( videoEncoderAttributeCodecId_ != VTMLIB && videoEncoderAttributeCodecId_ != VVLIB ) ) {
        std::cerr << "profileCodecGroupIdc_ is VVC_MAIN10 force codecId. \n";
        videoEncoderOccupancyCodecId_ = VTMLIB;
        videoEncoderGeometryCodecId_  = VTMLIB;
        videoEncoderAttributeCodecId_ = VTMLIB;
      }
#elif defined( USE_VTMLIB_VIDEO_CODEC )
      if ( ( videoEncoderOccupancyCodecId_ != VTMLIB ) || ( videoEncoderGeometryCodecId_ != VTMLIB ) ||
           ( videoEncoderAttributeCodecId_ != VTMLIB ) ) {
        std::cerr << "profileCodecGroupIdc_ is VVC_MAIN10 force codecId. \n";
        videoEncoderOccupancyCodecId_ = VTMLIB;
        videoEncoderGeometryCodecId_  = VTMLIB;
        videoEncoderAttributeCodecId_ = VTMLIB;
      }
#else
      std::cerr << "profileCodecGroupIdc_ is VVC_MAIN10, not supported. \n";
      ret = false;
#endif
      break;
    case CODEC_GROUP_MP4RA: break;
  }

#if defined( USE_SHMAPP_VIDEO_CODEC ) 
#if defined( USE_HMLIB_VIDEO_CODEC )
  if ( ( rawPointsPatch_ ) && ( videoEncoderOccupancyCodecId_ == SHMAPP || videoEncoderGeometryCodecId_ == SHMAPP ||
                                videoEncoderAttributeCodecId_ == SHMAPP ) ) {
    videoEncoderOccupancyCodecId_ = HMLIB;
    videoEncoderGeometryCodecId_  = HMLIB;
    videoEncoderAttributeCodecId_ = HMLIB;
    std::cerr << "SHMAPP codec not work for lossless conditions for HMLIB codecs. \n";
  }
  if ( ( videoEncoderOccupancyCodecId_ == SHMAPP || videoEncoderGeometryCodecId_ == SHMAPP ||
         videoEncoderAttributeCodecId_ == SHMAPP ) &&
       ( ( shvcRateX_ != 2 ) || ( shvcRateY_ != 2 ) ) ) {
    std::cerr << "SHMAPP codec requiered shvcRateX and shvcRateY equal to 2. \n";
    ret = false;
  }
#else
    std::cerr << "SHMAPP codec requiered USE_HMLIB_VIDEO_CODEC enable. \n";
    ret = false;
#endif
#endif
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
      refList.setStrafEntrySignFlag( i, afocDiff < 0 ? false : true );
      refList.setStRefAtalsFrameFlag( i, true );
    }
    asps.addRefListStruct( refList );
  }
}

uint8_t PCCEncoderParameters::getCodecIdIndex( PCCCodecId codecId ) {
  switch ( codecId ) {
#ifdef USE_JMAPP_VIDEO_CODEC
    case JMAPP:
#endif
#ifdef USE_JMLIB_VIDEO_CODEC
    case JMLIB: return avcCodecIdIndex_; break;
#endif
#ifdef USE_FFMPEG_VIDEO_CODEC
    case FFMPEG:
#endif
#ifdef USE_HMLIB_VIDEO_CODEC
    case HMLIB:
#endif
#ifdef USE_HMAPP_VIDEO_CODEC
    case HMAPP: return hevcCodecIdIndex_; break;
#endif
#ifdef USE_SHMAPP_VIDEO_CODEC
    case SHMAPP: return shvcCodecIdIndex_; break;
#endif
#ifdef USE_VTMLIB_VIDEO_CODEC
    case VTMLIB: return vvcCodecIdIndex_; break;
#endif
#ifdef USE_VVLIB_VIDEO_CODEC
    case VVLIB: return vvcCodecIdIndex_; break;
#endif
    default:
      printf( "Error: codec id %d not supported \n", (int)codecId );
      return 0;
      break;
  }
  return 0;
}

void PCCEncoderParameters::initializeContext( PCCContext& context ) {
  size_t  numAtlas   = 1;
  size_t  atlasIndex = 0;
  auto&   vps        = context.getVps();
  uint8_t bitdepth3D = uint8_t( geometry3dCoordinatesBitdepth_ + 1 );

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
  // context.setAuxVideoWidth( attributeRawSeparateVideoWidth_ );
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

  // V3C profile/tier/level related parameters
  auto& ptl = vps.getProfileTierLevel();
  ptl.setTierFlag( tierFlag_ );
  ptl.setProfileCodecGroupIdc( profileCodecGroupIdc_ );
  ptl.setProfileToolsetIdc( profileToolsetIdc_ );
  ptl.setProfileReconstructionIdc( profileReconstructionIdc_ );
  ptl.setLevelIdc( levelIdc_ );

  // V3C Profile toolset constraints information syntax
  auto& ptci = ptl.getProfileToolsetConstraintsInformation();
  ptci.setOneFrameOnlyFlag( oneV3CFrameOnlyFlag_ );
  ptci.setEOMContraintFlag( EOMContraintFlag_ );
  ptci.setPLRConstraintFlag( PLRConstraintFlag_ );
  ptci.setNoEightOrientationsConstraintFlag( noEightOrientationsConstraintFlag_ );
  ptci.setNo45DegreeProjectionPatchConstraintFlag( no45DegreeProjectionPatchConstraintFlag_ );

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
  asps.setPixelDeinterleavingFlag( singleMapPixelInterleaving_ );
  if ( singleMapPixelInterleaving_ ) {
    asps.allocatePixelDeinterleavingMapFlag();
    for ( size_t i = 0; i <= mapCountMinus1_; i++ )
      asps.setPixelDeinterleavingMapFlag( i, singleMapPixelInterleaving_ );
  }
  asps.setPatchPrecedenceOrderFlag( patchPrecedenceOrderFlag_ );
  asps.setPatchSizeQuantizerPresentFlag( context.getEnablePatchSizeQuantization() );
  asps.setEomPatchEnabledFlag( enhancedOccupancyMapCode_ );
  asps.setPLREnabledFlag( pointLocalReconstruction_ );
  asps.setVuiParametersPresentFlag( false );
  asps.setExtensionFlag( true );
  asps.setVpccExtensionFlag( true );
  asps.setExtension7Bits( 0 );
  asps.setAuxiliaryVideoEnabledFlag( useRawPointsSeparateVideo_ );
  asps.setRawPatchEnabledFlag( rawPointsPatch_ || lossyRawPointsPatch_ );
  if ( asps.getVpccExtensionFlag() ) {
    auto& ext = asps.getAspsVpccExtension();
    ext.setRemoveDuplicatePointEnableFlag( removeDuplicatePoints_ );
    if ( asps.getPixelDeinterleavingFlag() || asps.getPLREnabledFlag() ) {
      ext.setSurfaceThicknessMinus1( surfaceThickness_ - 1 );
    } // else keep the default value defined in the AspsVpccExtension constructor
  }
  asps.setEomFixBitCountMinus1( EOMFixBitCount_ - 1 );
  asps.setGeometry2dBitdepthMinus1( uint8_t( geometryNominal2dBitdepth_ - 1 ) );
  asps.setGeometry3dBitdepthMinus1( bitdepth3D - 1 );

  // Atlas frame parameter set
  auto& afps = context.addAtlasFrameParameterSet( 0 );
  afps.setAtlasSequenceParameterSetId( 0 );
  afps.setNumRefIdxDefaultActiveMinus1( static_cast<uint8_t>(
      constrainedPack_ ? ( ( std::max )( 0, static_cast<int>( maxNumRefAtlasFrame_ ) - 1 ) ) : 0 ) );
  afps.setAdditionalLtAfocLsbLen( asps.getLongTermRefAtlasFramesFlag() ? 4 : 0 );
  afps.setRaw3dOffsetBitCountExplicitModeFlag( false );
  afps.setExtensionFlag( true );
  afps.setExtension8Bits( 0 );
  constructAspsRefListStruct( context, 0, 0 );

  // occupancy information
  auto& oi = vps.getOccupancyInformation( atlasIndex );
  oi.setLossyOccupancyCompressionThreshold( thresholdLossyOM_ );
  oi.setOccupancy2DBitdepthMinus1( 7 );
  oi.setOccupancyMSBAlignFlag( false );
  oi.setOccupancyCodecId( getCodecIdIndex( (PCCCodecId)videoEncoderOccupancyCodecId_ ) );

  // geometry information
  auto& gi = vps.getGeometryInformation( atlasIndex );
  gi.setGeometry3dCoordinatesBitdepthMinus1( bitdepth3D - 1 );
  gi.setGeometry2dBitdepthMinus1( uint8_t( geometryNominal2dBitdepth_ - 1 ) );
  gi.setGeometryMSBAlignFlag( false );
  gi.setGeometryCodecId( getCodecIdIndex( (PCCCodecId)videoEncoderGeometryCodecId_ ) );
  gi.setAuxiliaryGeometryCodecId( gi.getGeometryCodecId() );

  // Attribute information
  auto& ai = vps.getAttributeInformation( atlasIndex );
  ai.setAttributeCount( noAttributes_ ? 0 : 1 );
  if ( !noAttributes_ ) {
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
      ai.setAttributeCodecId( i, getCodecIdIndex( (PCCCodecId)videoEncoderAttributeCodecId_ ) );
      ai.setAuxiliaryAttributeCodecId( i, ai.getAttributeCodecId( i ) );
    }
    printf( "CODEC ID SET = occupancy: %d geometry: %d attributes: %d \n", videoEncoderOccupancyCodecId_,
            videoEncoderGeometryCodecId_, videoEncoderAttributeCodecId_ );
    printf( "CODEC ID SET = occupancy: %d geometry: %d attributes[0]: %d \n", oi.getOccupancyCodecId(),
            gi.getGeometryCodecId(), ai.getAttributeCodecId( 0 ) );
  } else {
    printf( "CODEC ID SET = geometry: %d occupancy: %d \n", oi.getOccupancyCodecId(), gi.getGeometryCodecId() );
  }
  // atlas video frame allocation
  context.getAtlas( atlasIndex ).allocateVideoFrames( context, 0 );

  // Tiles
  auto& frames = context.getFrames();
  for ( size_t i = 0; i < frames.size(); i++ ) {
    auto& atlas = frames[i];
    auto& frame = atlas.getTitleFrameContext();
    frame.setFrameIndex( i );
    frame.setRawPatchEnabledFlag( rawPointsPatch_ || lossyRawPointsPatch_ );
    frame.setUseRawPointsSeparateVideo( useRawPointsSeparateVideo_ );
    frame.setGeometry3dCoordinatesBitdepth( bitdepth3D );
    frame.setGeometry2dBitdepth( geometryNominal2dBitdepth_ );
    frame.setMaxDepth( ( 1 << geometryNominal2dBitdepth_ ) - 1 );
    frame.setLog2PatchQuantizerSizeX( context.getLog2PatchQuantizerSizeX() );
    frame.setLog2PatchQuantizerSizeY( context.getLog2PatchQuantizerSizeY() );
    frame.setAtlasFrmOrderCntLsb( context.calculateAFOCLsb( i ) );
    frame.setAtlasFrmOrderCntVal( i );
    frame.setNumRefIdxActive( i == 0 ? 0 : ( constrainedPack_ ? ( std::min )( i, maxNumRefAtlasFrame_ ) : 0 ) );
    frame.setRefAfocList( context, 0 );
    if ( useRawPointsSeparateVideo_ ) {
      context[i].setAuxVideoWidth( attributeRawSeparateVideoWidth_ );
      context[i].resizeAuxTileHeight( numMaxTilePerFrame_, 0 );
      context[i].resizeAuxTileLeftTopY( numMaxTilePerFrame_, 0 );
    }
    atlas.setNumTilesInAtlasFrame( numMaxTilePerFrame_ );
    atlas.initNumTiles( numMaxTilePerFrame_ );
    atlas.initPartitionInfoPerFrame( i, minimumImageWidth_, minimumImageHeight_, numMaxTilePerFrame_,
                                     uniformPartitionSpacing_, tilePartitionWidth_, tilePartitionHeight_ );
    for ( size_t ti = 0; ti < numMaxTilePerFrame_; ti++ ) {
      auto& tile = atlas[ti];
      tile.setRawPatchEnabledFlag( frame.getRawPatchEnabledFlag() );
      tile.setUseRawPointsSeparateVideo( frame.getUseRawPointsSeparateVideo() );
      tile.setGeometry3dCoordinatesBitdepth( frame.getGeometry3dCoordinatesBitdepth() );
      tile.setGeometry2dBitdepth( frame.getGeometry2dBitdepth() );
      tile.setMaxDepth( frame.getMaxDepth() );
      tile.setLog2PatchQuantizerSizeX( frame.getLog2PatchQuantizerSizeX() );
      tile.setLog2PatchQuantizerSizeY( frame.getLog2PatchQuantizerSizeY() );
      tile.setAtlasFrmOrderCntLsb( frame.getAtlasFrmOrderCntLsb() );
      tile.setAtlasFrmOrderCntVal( frame.getAtlasFrmOrderCntVal() );
      tile.setTileIndex( ti );
      tile.setFrameIndex( frame.getFrameIndex() );
      tile.setNumRefIdxActive( frame.getNumRefIdxActive() );
      tile.setRefAfocList( context, 0 );
    }
  }
}
