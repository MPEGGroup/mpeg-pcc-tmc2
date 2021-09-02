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
class PCCContext;

class PCCEncoderParameters {
 public:
  PCCEncoderParameters();
  ~PCCEncoderParameters();
  void        print();
  bool        check();
  void        completePath();
  static void constructAspsRefListStruct( PCCContext& context, size_t aspsIdx, size_t afpsIdx );
  void        initializeContext( PCCContext& context );
  uint8_t     getCodecIdIndex( PCCCodecId codecId );

  size_t            startFrameNumber_;
  std::string       configurationFolder_;
  std::string       uncompressedDataFolder_;
  std::string       compressedStreamPath_;
  std::string       reconstructedDataPath_;
  PCCColorTransform colorTransform_;
  std::string       colorSpaceConversionPath_;
  std::string       videoEncoderOccupancyPath_;
  std::string       videoEncoderGeometryPath_;
  std::string       videoEncoderAttributePath_;
  PCCCodecId        videoEncoderOccupancyCodecId_;
  PCCCodecId        videoEncoderGeometryCodecId_;
  PCCCodecId        videoEncoderAttributeCodecId_;
  bool              byteStreamVideoCoderOccupancy_;
  bool              byteStreamVideoCoderGeometry_;
  bool              byteStreamVideoCoderAttribute_;
  bool              use3dmc_;
  bool              usePccRDO_;
  std::string       colorSpaceConversionConfig_;
  std::string       inverseColorSpaceConversionConfig_;
  size_t            nbThread_;
  size_t            frameCount_;
  size_t            groupOfFramesSize_;
  std::string       uncompressedDataPath_;
  uint32_t          forcedSsvhUnitSizePrecisionBytes_;

  // packing
  size_t minimumImageWidth_;
  size_t minimumImageHeight_;

  // video encoding
  int         geometryQP_;
  int         attributeQP_;
  int         deltaQPD0_;
  int         deltaQPD1_;
  int         deltaQPT0_;
  int         deltaQPT1_;
  int         auxGeometryQP_;
  int         auxAttributeQP_;
  std::string geometryConfig_;
  std::string geometry0Config_;
  std::string geometry1Config_;
  std::string attributeConfig_;
  std::string attribute0Config_;
  std::string attribute1Config_;
  bool        multipleStreams_;

  // segmentation
  bool   gridBasedSegmentation_;
  size_t voxelDimensionGridBasedSegmentation_;
  size_t nnNormalEstimation_;
  size_t normalOrientation_;
  bool   gridBasedRefineSegmentation_;
  size_t maxNNCountRefineSegmentation_;
  size_t iterationCountRefineSegmentation_;
  size_t voxelDimensionRefineSegmentation_;
  size_t searchRadiusRefineSegmentation_;
  size_t occupancyResolution_;
  bool   enablePatchSplitting_;
  size_t maxPatchSize_;
  size_t log2QuantizerSizeX_;
  size_t log2QuantizerSizeY_;
  size_t minPointCountPerCCPatchSegmentation_;
  size_t maxNNCountPatchSegmentation_;
  size_t surfaceThickness_;
  size_t minLevel_;  // 8,16,32,64
  double maxAllowedDist2RawPointsDetection_;
  double maxAllowedDist2RawPointsSelection_;
  double lambdaRefineSegmentation_;
  size_t mapCountMinus1_;

  // occupancy map encoding
  size_t      maxCandidateCount_;
  size_t      occupancyPrecision_;
  std::string occupancyMapConfig_;
  size_t      occupancyMapQP_;
  size_t      EOMFixBitCount_;
  bool        occupancyMapRefinement_;

  // hash
  size_t decodedAtlasInformationHash_;

  // smoothing
  size_t neighborCountSmoothing_;
  double radius2Smoothing_;
  double radius2BoundaryDetection_;
  double thresholdSmoothing_;
  bool   gridSmoothing_;
  size_t gridSize_;
  bool   flagGeometrySmoothing_;

  // Patch Expansion (m47772, CE2.12)
  bool patchExpansion_;

  // color smoothing
  double thresholdColorSmoothing_;
  double thresholdColorDifference_;
  double thresholdColorVariation_;
  size_t cgridSize_;
  bool   flagColorSmoothing_;

  // color pre-smoothing
  double thresholdColorPreSmoothing_;
  double thresholdColorPreSmoothingLocalEntropy_;
  double radius2ColorPreSmoothing_;
  size_t neighborCountColorPreSmoothing_;
  bool   flagColorPreSmoothing_;

  // coloring
  size_t bestColorSearchRange_;
  // Improved color transfer
  int    numNeighborsColorTransferFwd_;
  int    numNeighborsColorTransferBwd_;
  bool   useDistWeightedAverageFwd_;
  bool   useDistWeightedAverageBwd_;
  bool   skipAvgIfIdenticalSourcePointPresentFwd_;
  bool   skipAvgIfIdenticalSourcePointPresentBwd_;
  double distOffsetFwd_;
  double distOffsetBwd_;
  double maxGeometryDist2Fwd_;
  double maxGeometryDist2Bwd_;
  double maxColorDist2Fwd_;
  double maxColorDist2Bwd_;
  // Exclude color outliers
  bool   excludeColorOutlier_;
  double thresholdColorOutlierDist_;

  // lossless
  bool noAttributes_;
  bool rawPointsPatch_;
  bool attributeVideo444_;

  // raw points video
  bool        useRawPointsSeparateVideo_;
  std::string geometryAuxVideoConfig_;
  std::string attributeAuxVideoConfig_;

  // scale and bias
  float             modelScale_;
  PCCVector3<float> modelOrigin_;

  // patch sampling resolution
  size_t levelOfDetailX_;
  size_t levelOfDetailY_;
  bool   keepIntermediateFiles_;
  bool   absoluteD1_;
  bool   absoluteT1_;
  bool   constrainedPack_;

  // dilation
  bool groupDilation_;

  // EOM
  bool enhancedOccupancyMapCode_;

  // Lossy occupancy Map coding
  size_t offsetLossyOM_;
  size_t thresholdLossyOM_;
  bool   prefilterLossyOM_;

  // reconstruction
  bool   removeDuplicatePoints_;
  bool   pointLocalReconstruction_;
  size_t patchSize_;
  size_t plrlNumberOfModes_;
  bool   singleMapPixelInterleaving_;

  // visual quality
  bool   patchColorSubsampling_;
  bool   surfaceSeparation_;
  bool   highGradientSeparation_;
  double minGradient_;
  size_t minNumHighGradientPoints_;

  // Flexible Patch Packing
  size_t packingStrategy_;
  size_t attributeBGFill_;
  size_t safeGuardDistance_;
  bool   useEightOrientations_;

  // Lossy raw points Patch
  bool   lossyRawPointsPatch_;
  double minNormSumOfInvDist4MPSelection_;

  // GPA
  int globalPatchAllocation_;
  // GTP
  int    globalPackingStrategyGOF_;
  bool   globalPackingStrategyReset_;
  double globalPackingStrategyThreshold_;
  // low delay encoding
  bool lowDelayEncoding_;
  // 3D geometry padding
  size_t geometryPadding_;

  // EOM
  bool   enhancedPP_;
  double minWeightEPP_;

  // Additional Projection Plane
  int    additionalProjectionPlaneMode_;
  double partialAdditionalProjectionPlane_;

  // 3D and 2D bit depths
  size_t geometry3dCoordinatesBitdepth_;
  size_t geometryNominal2dBitdepth_;

  // Partitions and tiles
  bool             enablePointCloudPartitioning_;
  std::vector<int> roiBoundingBoxMinX_;
  std::vector<int> roiBoundingBoxMaxX_;
  std::vector<int> roiBoundingBoxMinY_;
  std::vector<int> roiBoundingBoxMaxY_;
  std::vector<int> roiBoundingBoxMinZ_;
  std::vector<int> roiBoundingBoxMaxZ_;
  int              numTilesHor_;
  double           tileHeightToWidthRatio_;
  int              numCutsAlong1stLongestAxis_;
  int              numCutsAlong2ndLongestAxis_;
  int              numCutsAlong3rdLongestAxis_;
  int              numROIs_;

  // Sort raw points by Morton code
  bool   mortonOrderSortRawPoints_;
  size_t attributeRawSeparateVideoWidth_;

  // Patch block filtering
  bool    pbfEnableFlag_;
  int16_t pbfPassesCount_;
  int16_t pbfFilterSize_;
  int16_t pbfLog2Threshold_;

  // re
  bool   patchPrecedenceOrderFlag_;
  size_t maxNumRefAtlasList_;
  size_t maxNumRefAtlasFrame_;

  size_t               log2MaxAtlasFrameOrderCntLsb_;
  size_t               tileSegmentationType_;
  size_t               numMaxTilePerFrame_;
  bool                 uniformPartitionSpacing_;
  size_t               tilePartitionWidth_;
  size_t               tilePartitionHeight_;
  std::vector<int32_t> tilePartitionWidthList_;
  std::vector<int32_t> tilePartitionHeightList_;

  // Profile tier level
  bool   tierFlag_;
  size_t profileCodecGroupIdc_;
  size_t profileToolsetIdc_;
  size_t profileReconstructionIdc_;
  size_t levelIdc_;
  size_t avcCodecIdIndex_;
  size_t hevcCodecIdIndex_;
  size_t shvcCodecIdIndex_;
  size_t vvcCodecIdIndex_;

  // Profile toolset constraints information
  bool   oneV3CFrameOnlyFlag_;
  bool   EOMContraintFlag_;
  size_t maxMapCountMinus1_;
  size_t maxAtlasCountMinus1_;
  bool   multipleMapStreamsConstraintFlag_;
  bool   PLRConstraintFlag_;
  size_t attributeMaxDimensionMinus1_;
  size_t attributeMaxDimensionPartitionsMinus1_;
  bool   noEightOrientationsConstraintFlag_;
  bool   no45DegreeProjectionPatchConstraintFlag_;

  // reconstruction options : 0. ignore 1. use indicated syntaxes 2. open
  size_t pixelDeinterleavingType_;
  size_t pointLocalReconstructionType_;
  size_t reconstructEomType_;
  size_t duplicatedPointRemovalType_;
  size_t reconstructRawType_;
  size_t applyGeoSmoothingType_;
  size_t applyAttrSmoothingType_;
  size_t attrTransferFilterType_;
  size_t applyOccupanySynthesisType_;

  // SHVC
  size_t shvcLayerIndex_;
  size_t shvcRateX_;
  size_t shvcRateY_;
};

};  // namespace pcc

#endif /* PCCEncoderParameters_h */
