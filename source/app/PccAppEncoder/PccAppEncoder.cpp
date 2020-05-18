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
#include "PccAppEncoder.h"

using namespace std;
using namespace pcc;
using pcc::chrono::StopwatchUserTime;

int main( int argc, char* argv[] ) {
  std::cout << "PccAppEncoder v" << TMC2_VERSION_MAJOR << "." << TMC2_VERSION_MINOR << std::endl << std::endl;

  PCCEncoderParameters encoderParams;
  PCCMetricsParameters metricsParams;
  if ( !parseParameters( argc, argv, encoderParams, metricsParams ) ) { return -1; }
  if ( encoderParams.nbThread_ > 0 ) { tbb::task_scheduler_init init( static_cast<int>( encoderParams.nbThread_ ) ); }

  // Timers to count elapsed wall/user time
  pcc::chrono::Stopwatch<std::chrono::steady_clock> clockWall;
  pcc::chrono::StopwatchUserTime                    clockUser;

  clockWall.start();
  int ret = compressVideo( encoderParams, metricsParams, clockUser );
  clockWall.stop();

  using namespace std::chrono;
  using ms       = milliseconds;
  auto totalWall = duration_cast<ms>( clockWall.count() ).count();
  std::cout << "Processing time (wall): " << ( ret == 0 ? totalWall / 1000.0 : -1 ) << " s\n";

  auto totalUserSelf = duration_cast<ms>( clockUser.self.count() ).count();
  std::cout << "Processing time (user.self): " << ( ret == 0 ? totalUserSelf / 1000.0 : -1 ) << " s\n";

  auto totalUserChild = duration_cast<ms>( clockUser.children.count() ).count();
  std::cout << "Processing time (user.children): " << ( ret == 0 ? totalUserChild / 1000.0 : -1 ) << " s\n";

  std::cout << "Peak memory: " << getPeakMemory() << " KB\n";
  return ret;
}

//---------------------------------------------------------------------------
// :: Command line / config parsing helpers

template <typename T>
static std::istream& readUInt( std::istream& in, T& val ) {
  unsigned int tmp;
  in >> tmp;
  val = T( tmp );
  return in;
}

namespace pcc {
static std::istream& operator>>( std::istream& in, PCCColorTransform& val ) { return readUInt( in, val ); }

static std::istream& operator>>( std::istream& in, std::vector<int>& vector ) {
  std::string str;
  in >> str;
  str.erase( std::remove( str.begin(), str.end(), ' ' ), str.end() );
  size_t pos = 0;
  while ( ( pos = str.find( "," ) ) != std::string::npos ) {
    vector.push_back( std::stoi( str.substr( 0, pos ) ) );
    str.erase( 0, pos + 1 );
  }
  vector.push_back( std::stoi( str.substr( 0, pos ) ) );
  return in;
}

// template <class T>
// static inline istream& operator>> (std::istream &in, T &val ) {  in >> val;
// return in; }
}  // namespace pcc

//---------------------------------------------------------------------------
// :: Command line / config parsing

bool parseParameters( int                   argc,
                      char*                 argv[],
                      PCCEncoderParameters& encoderParams,
                      PCCMetricsParameters& metricsParams ) {
  namespace po    = df::program_options_lite;
  bool print_help = false;

  // The definition of the program/config options, along with default values.
  //
  // NB: when updating the following tables:
  //      (a) please keep to 80-columns for easier reading at a glance,
  //      (b) do not vertically align values -- it breaks quickly
  //
  // clang-format off
  po::Options opts;
  opts.addOptions()
    ( "help", print_help, false, 
      "This help text" )
    ( "c,config", 
    po::parseConfigFile, 
      "Configuration file name" )
    ( "configurationFolder",
      encoderParams.configurationFolder_,
      encoderParams.configurationFolder_,
      "Folder where the configuration files are stored,use for cfg relative paths." )
    ( "uncompressedDataFolder",
      encoderParams.uncompressedDataFolder_,
      encoderParams.uncompressedDataFolder_,
      "Folder where the uncompress input data are stored, use for cfg relative paths." )

    // i/o
    ( "uncompressedDataPath",
      encoderParams.uncompressedDataPath_,
      encoderParams.uncompressedDataPath_,
      "Input pointcloud to encode. Multi-frame sequences may be represented by %04i" )
    ( "compressedStreamPath",
      encoderParams.compressedStreamPath_,
      encoderParams.compressedStreamPath_,
      "Output(encoder)/Input(decoder) compressed bitstream" )
    ( "reconstructedDataPath",
      encoderParams.reconstructedDataPath_,
      encoderParams.reconstructedDataPath_,
      "Output decoded pointcloud. Multi-frame sequences may be represented by %04i" )

    // sequence configuration
    ( "startFrameNumber",
      encoderParams.startFrameNumber_,
      encoderParams.startFrameNumber_,
      "First frame number in sequence to encode/decode" )
    ( "frameCount",
      encoderParams.frameCount_,
      encoderParams.frameCount_,
      "Number of frames to encode" )
    ( "groupOfFramesSize",
      encoderParams.groupOfFramesSize_,
      encoderParams.groupOfFramesSize_, 
      "Random access period" )

    // colour space conversion
    ( "colorTransform",
      encoderParams.colorTransform_,
      encoderParams.colorTransform_,
      "The colour transform to be applied:\n  0: none\n  1: RGB to YCbCr (Rec.709)" )
    ( "colorSpaceConversionPath",
      encoderParams.colorSpaceConversionPath_,
      encoderParams.colorSpaceConversionPath_,
      "Path to the HDRConvert. If unset, an internal color space conversion is used" )
    ( "colorSpaceConversionConfig",
      encoderParams.colorSpaceConversionConfig_,
      encoderParams.colorSpaceConversionConfig_,
      "HDRConvert configuration file used for RGB444 to YUV420 conversion" )
    ( "inverseColorSpaceConversionConfig",
      encoderParams.inverseColorSpaceConversionConfig_,
      encoderParams.inverseColorSpaceConversionConfig_,
      "HDRConvert configuration file used for YUV420 to RGB444 conversion" )

    // segmentation
    ( "nnNormalEstimation",
      encoderParams.nnNormalEstimation_,
      encoderParams.nnNormalEstimation_,
      "Number of points used for normal estimation" )
    ( "gridBasedRefineSegmentation",
      encoderParams.gridBasedRefineSegmentation_,
      encoderParams.gridBasedRefineSegmentation_,
      "Use grid-based approach for segmentation refinement" )
    ( "maxNNCountRefineSegmentation",
      encoderParams.maxNNCountRefineSegmentation_,
      encoderParams.maxNNCountRefineSegmentation_,
      "Number of nearest neighbors used during segmentation refinement" )
    ( "iterationCountRefineSegmentation",
      encoderParams.iterationCountRefineSegmentation_,
      encoderParams.iterationCountRefineSegmentation_,
      "Number of iterations performed during segmentation refinement" )
    ( "voxelDimensionRefineSegmentation",
      encoderParams.voxelDimensionRefineSegmentation_,
      encoderParams.voxelDimensionRefineSegmentation_,
      "Voxel dimension for segmentation refinement (must be a power of 2)" )
    ( "searchRadiusRefineSegmentation",
      encoderParams.searchRadiusRefineSegmentation_,
      encoderParams.searchRadiusRefineSegmentation_,
      "Search radius for segmentation refinement" )
    ( "occupancyResolution",
      encoderParams.occupancyResolution_,
      encoderParams.occupancyResolution_,
      "Resolution of packing block(a block contain only one patch)" )
    ( "enablePatchSplitting",
      encoderParams.enablePatchSplitting_,
      encoderParams.enablePatchSplitting_, 
      "Enable patch splitting" )
    ( "maxPatchSize",
      encoderParams.maxPatchSize_,
      encoderParams.maxPatchSize_, 
      "Maximum patch size for segmentation" )
    ( "log2QuantizerSizeX",
      encoderParams.log2QuantizerSizeX_,
      encoderParams.log2QuantizerSizeX_,
      "log2 of Quantization step for patch size X: 0. pixel precision 4.16 as before" )
    ( "log2QuantizerSizeY",
      encoderParams.log2QuantizerSizeY_,
      encoderParams.log2QuantizerSizeY_,
      "log2 of Quantization step for patch size Y: 0. pixel precision 4.16 as before" )
    ( "minPointCountPerCCPatchSegmentation",
      encoderParams.minPointCountPerCCPatchSegmentation_,
      encoderParams.minPointCountPerCCPatchSegmentation_,
      "Minimum number of points for a connected component to be retained as a patch" )
    ( "maxNNCountPatchSegmentation",
      encoderParams.maxNNCountPatchSegmentation_,
      encoderParams.maxNNCountPatchSegmentation_,
      "Number of nearest neighbors used during connected components extraction" )
    ( "surfaceThickness",
      encoderParams.surfaceThickness_,
      encoderParams.surfaceThickness_, 
      "Surface thickness" )
    ( "depthQuantizationStep",
      encoderParams.minLevel_,
      encoderParams.minLevel_, 
      "minimum level for patches" )
    ( "maxAllowedDist2RawPointsDetection",
      encoderParams.maxAllowedDist2RawPointsDetection_,
      encoderParams.maxAllowedDist2RawPointsDetection_,
      "Maximum distance for a point to be ignored during raw points detection" )
    ( "maxAllowedDist2RawPointsSelection",
      encoderParams.maxAllowedDist2RawPointsSelection_,
      encoderParams.maxAllowedDist2RawPointsSelection_,
      "Maximum distance for a point to be ignored during  raw points  selection" )
    ( "lambdaRefineSegmentation",
      encoderParams.lambdaRefineSegmentation_,
      encoderParams.lambdaRefineSegmentation_,
      "Controls the smoothness of the patch boundaries  during segmentation  refinement" )

    // packing
    ( "minimumImageWidth",
      encoderParams.minimumImageWidth_,
      encoderParams.minimumImageWidth_, 
      "Minimum width of packed patch frame" )
    ( "minimumImageHeight",
      encoderParams.minimumImageHeight_,
      encoderParams.minimumImageHeight_,
      "Minimum height of packed patch frame" )

    // occupancy map
    ( "maxCandidateCount",
      encoderParams.maxCandidateCount_,
      encoderParams.maxCandidateCount_,
      "Maximum nuber of candidates in list L" )
    ( "occupancyPrecision",
      encoderParams.occupancyPrecision_,
      encoderParams.occupancyPrecision_, 
      "Occupancy map B0 precision" )
    ( "occupancyMapVideoEncoderConfig",
      encoderParams.occupancyMapVideoEncoderConfig_,
      encoderParams.occupancyMapVideoEncoderConfig_,
      "Occupancy map encoder config file" )
    ( "occupancyMapQP",
      encoderParams.occupancyMapQP_,
      encoderParams.occupancyMapQP_,
      "QP for compression of occupancy map video" )

    // EOM code
    ( "enhancedOccupancyMapCode",
      encoderParams.enhancedOccupancyMapCode_,
      encoderParams.enhancedOccupancyMapCode_,
      "Use enhanced-delta-depth code" )
    ( "EOMFixBitCount",
      encoderParams.EOMFixBitCount_,
      encoderParams.EOMFixBitCount_,
      "enhanced occupancy map fixed bit count" )
    ( "occupancyMapRefinement",
      encoderParams.occupancyMapRefinement_,
      encoderParams.occupancyMapRefinement_, 
      "Use occupancy map refinement" )

    // smoothing
    ( "postprocessSmoothingFilterType",
      encoderParams.postprocessSmoothingFilter_,
      encoderParams.postprocessSmoothingFilter_,
      "Exclude geometry smoothing from attribute transfer\n" )
    ( "flagGeometrySmoothing",
      encoderParams.flagGeometrySmoothing_,
      encoderParams.flagGeometrySmoothing_, 
      "Enable geometry smoothing\n" )
    ( "neighborCountSmoothing",
      encoderParams.neighborCountSmoothing_,
      encoderParams.neighborCountSmoothing_, 
      "Neighbor count smoothing" )
    ( "radius2Smoothing",
      encoderParams.radius2Smoothing_,
      encoderParams.radius2Smoothing_, 
      "Radius to smoothing" )
    ( "radius2BoundaryDetection",
      encoderParams.radius2BoundaryDetection_,
      encoderParams.radius2BoundaryDetection_,
      "Radius to boundary detection" )
    ( "thresholdSmoothing",
      encoderParams.thresholdSmoothing_,
      encoderParams.thresholdSmoothing_, 
      "Threshold smoothing" )

    // Patch Expansion (m47772 CE2.12)
    ( "patchExpansion",
      encoderParams.patchExpansion_,
      encoderParams.patchExpansion_, 
      "Use occupancy map refinement" )

    // grid smoothing (m44705 CE2.17)
    ( "gridSmoothing",
      encoderParams.gridSmoothing_,
      encoderParams.gridSmoothing_, 
      "Enable grid smoothing" )
    ( "gridSize",
      encoderParams.gridSize_,
      encoderParams.gridSize_,
      "grid size for the smoothing" )

    // color smoothing
    ( "thresholdColorSmoothing",
      encoderParams.thresholdColorSmoothing_,
      encoderParams.thresholdColorSmoothing_, 
      "Threshold of color smoothing" )
    ( "gridColorSmoothing",
      encoderParams.gridColorSmoothing_,
      encoderParams.gridColorSmoothing_,
      "Enable low complexity color smoothing" )
    ( "cgridSize",
      encoderParams.cgridSize_,
      encoderParams.cgridSize_,
      "grid size for the color smoothing" )
    ( "thresholdColorDifference",
      encoderParams.thresholdColorDifference_,
      encoderParams.thresholdColorDifference_,
      "Threshold of color difference between cells" )
    ( "thresholdColorVariation",
      encoderParams.thresholdColorVariation_,
      encoderParams.thresholdColorVariation_,
      "Threshold of color variation in cells" )
    ( "thresholdLocalEntropy",
      encoderParams.thresholdLocalEntropy_,
      encoderParams.thresholdLocalEntropy_, 
      "Threshold of local entropy" )
    ( "radius2ColorSmoothing",
      encoderParams.radius2ColorSmoothing_,
      encoderParams.radius2ColorSmoothing_, 
      "Redius of color smoothing" )
    ( "neighborCountColorSmoothing",
      encoderParams.neighborCountColorSmoothing_,
      encoderParams.neighborCountColorSmoothing_,
      "Neighbor count for color smoothing" )
    ( "flagColorSmoothing",
      encoderParams.flagColorSmoothing_,
      encoderParams.flagColorSmoothing_, 
      "Enable color smoothing\n" )

    // color pre-smoothing
    ( "thresholdColorPreSmoothing",
      encoderParams.thresholdColorPreSmoothing_,
      encoderParams.thresholdColorPreSmoothing_,
      "Threshold of color pre-smoothing" )
    ( "thresholdColorPreSmoothingLocalEntropy",
      encoderParams.thresholdColorPreSmoothingLocalEntropy_,
      encoderParams.thresholdColorPreSmoothingLocalEntropy_,
      "Threshold of color pre-smoothing local entropy" )
    ( "radius2ColorPreSmoothing",
      encoderParams.radius2ColorPreSmoothing_,
      encoderParams.radius2ColorPreSmoothing_,
      "Redius of color pre-smoothing" )
    ( "neighborCountColorPreSmoothing",
      encoderParams.neighborCountColorPreSmoothing_,
      encoderParams.neighborCountColorPreSmoothing_,
      "Neighbor count for color pre-smoothing" )
    ( "flagColorPreSmoothing",
      encoderParams.flagColorPreSmoothing_,
      encoderParams.flagColorPreSmoothing_, 
      "Enable color pre-smoothing\n" )

    // colouring
    ( "bestColorSearchRange",
      encoderParams.bestColorSearchRange_,
      encoderParams.bestColorSearchRange_, 
      "Best color search range" )
    // Improved color transfer (m49367 CE2.17)
    ( "numNeighborsColorTransferFwd",
      encoderParams.numNeighborsColorTransferFwd_,
      encoderParams.numNeighborsColorTransferFwd_,
      "Number of neighbors creating Fwd list" )
    ( "numNeighborsColorTransferBwd",
      encoderParams.numNeighborsColorTransferBwd_,
      encoderParams.numNeighborsColorTransferBwd_,
      "Number of neighbors creating Bwd list" )
    ( "useDistWeightedAverageFwd",
      encoderParams.useDistWeightedAverageFwd_,
      encoderParams.useDistWeightedAverageFwd_,
      "Distance weighted average for Fwd list" )
    ( "useDistWeightedAverageBwd",
      encoderParams.useDistWeightedAverageBwd_,
      encoderParams.useDistWeightedAverageBwd_,
      "Distance weighted average for Bwd list" )
    ( "skipAvgIfIdenticalSourcePointPresentFwd",
      encoderParams.skipAvgIfIdenticalSourcePointPresentFwd_,
      encoderParams.skipAvgIfIdenticalSourcePointPresentFwd_,
      "Skip avgeraging if target is identical to a Fwd point" )
    ( "skipAvgIfIdenticalSourcePointPresentBwd",
      encoderParams.skipAvgIfIdenticalSourcePointPresentBwd_,
      encoderParams.skipAvgIfIdenticalSourcePointPresentBwd_,
      "Skip avgeraging if target is identical to a Bwd point" )
    ( "distOffsetFwd",
      encoderParams.distOffsetFwd_,
      encoderParams.distOffsetFwd_,
      "Distance offset to avoid infinite weight" )
    ( "distOffsetBwd",
      encoderParams.distOffsetBwd_,
      encoderParams.distOffsetBwd_,
      "Distance offset to avoid infinite weight" )
    ( "maxGeometryDist2Fwd",
      encoderParams.maxGeometryDist2Fwd_,
      encoderParams.maxGeometryDist2Fwd_,
      "Maximum allowed distance for a Fwd point" )
    ( "maxGeometryDist2Bwd",
      encoderParams.maxGeometryDist2Bwd_,
      encoderParams.maxGeometryDist2Bwd_,
      "Maximum allowed distance for a Bwd point" )
    ( "maxColorDist2Fwd",
      encoderParams.maxColorDist2Fwd_,
      encoderParams.maxColorDist2Fwd_,
      "Maximum allowed pari-wise color distance for Fwd list" )
    ( "maxColorDist2Bwd",
      encoderParams.maxColorDist2Bwd_,
      encoderParams.maxColorDist2Bwd_,
      "Maximum allowed pari-wise color distance for Bwd list" )
    ( "excludeColorOutlier",
      encoderParams.excludeColorOutlier_,
      encoderParams.excludeColorOutlier_,
      "Exclude color outliers from the NN set" )
    ( "thresholdColorOutlierDist",
      encoderParams.thresholdColorOutlierDist_,
      encoderParams.thresholdColorOutlierDist_,
      "Threshold of color distance to exclude outliers from the NN set" )

    // video encoding
    ( "videoEncoderPath",
      encoderParams.videoEncoderPath_,
      encoderParams.videoEncoderPath_, 
      "HM video encoder executable" )
    ( "videoEncoderAuxPath",
      encoderParams.videoEncoderAuxPath_,
      encoderParams.videoEncoderAuxPath_, 
      "HM video encoder executable" )
    ( "videoEncoderOccupancyMapPath",
      encoderParams.videoEncoderOccupancyMapPath_,
      encoderParams.videoEncoderOccupancyMapPath_,
      "HM lossless video encoder executable for occupancy map" )
    ( "geometryQP",
      encoderParams.geometryQP_,
      encoderParams.geometryQP_,
      "QP for compression of geometry video" )
    ( "textureQP",
      encoderParams.textureQP_,
      encoderParams.textureQP_,
      "QP for compression of texture video" )
    ( "geometryConfig",
      encoderParams.geometryConfig_,
      encoderParams.geometryConfig_,
      "HM configuration file for geometry compression" )
    ( "geometryD0Config",
      encoderParams.geometryD0Config_,
      encoderParams.geometryD0Config_,
      "HM configuration file for geometry D0 compression" )
    ( "geometryD1Config",
      encoderParams.geometryD1Config_,
      encoderParams.geometryD1Config_,
      "HM configuration file for geometry D1 compression" )
    ( "textureConfig",
      encoderParams.textureConfig_,
      encoderParams.textureConfig_,
      "HM configuration file for texture compression" )
    ( "textureT0Config",
      encoderParams.textureT0Config_,
      encoderParams.textureT0Config_,
      "HM configuration file for texture D0 compression" )
    ( "textureT1Config",
      encoderParams.textureT1Config_,
      encoderParams.textureT1Config_,
      "HM configuration file for texture D1 compression" )
    // lossless parameters
    ( "losslessGeo",
      encoderParams.losslessGeo_,
      encoderParams.losslessGeo_,
      "Enable lossless encoding of geometry\n" )
    ( "noAttributes",
      encoderParams.noAttributes_,
      encoderParams.noAttributes_, 
      "Disable encoding of attributes" )
    ( "losslessGeo444",
      encoderParams.losslessGeo444_,
      encoderParams.losslessGeo444_,
      "Use 4444 format for lossless geometry" )
    ( "useRawPointsSeparateVideo",
      encoderParams.useRawPointsSeparateVideo_,
      encoderParams.useRawPointsSeparateVideo_,
      "compress raw points with video codec" )
    ( "textureRawSeparateVideoWidth",
      encoderParams.textureRawSeparateVideoWidth_,
      encoderParams.textureRawSeparateVideoWidth_,
      "Width of the MP's texture in separate video" )
    ( "geometryMPConfig",
      encoderParams.geometryMPConfig_,
      encoderParams.geometryMPConfig_,
      "HM configuration file for raw points geometry compression" )
    ( "textureMPConfig",
      encoderParams.textureMPConfig_,
      encoderParams.textureMPConfig_,
      "HM configuration file for raw points texture compression" )

    // etc
    ( "nbThread",
      encoderParams.nbThread_,
      encoderParams.nbThread_,
      "Number of thread used for parallel processing" )
    ( "keepIntermediateFiles",
      encoderParams.keepIntermediateFiles_,
      encoderParams.keepIntermediateFiles_,
      "Keep intermediate files: RGB, YUV and bin" )
    ( "absoluteD1",
      encoderParams.absoluteD1_,
      encoderParams.absoluteD1_,
      "Absolute D1" )
    ( "absoluteT1",
      encoderParams.absoluteT1_,
      encoderParams.absoluteT1_, 
      "Absolute T1" )
    ( "multipleStreams",
      encoderParams.multipleStreams_,
    // 0. absolute 1 delta
      encoderParams.multipleStreams_,
      "number of video(geometry and attribute) streams" )
    ( "qpT1",
      encoderParams.qpAdjT1_,
    // 0. absolute 1 delta
      encoderParams.qpAdjT1_, 
      "qp adjustment for T1 0, +3, -3..." )
    ( "qpD1",
      encoderParams.qpAdjD1_,
      encoderParams.qpAdjD1_,
      "qp adjustment for D1 : 0, +3, -3..." )
    ( "constrainedPack",
      encoderParams.constrainedPack_,
      encoderParams.constrainedPack_,
      "Temporally consistent patch packing" )
    ( "levelOfDetailX",
      encoderParams.levelOfDetailX_,
      encoderParams.levelOfDetailX_,
      "levelOfDetail : X axis in 2D space (should be greater than 1)" )
    ( "levelOfDetailY",
      encoderParams.levelOfDetailY_,
      encoderParams.levelOfDetailY_,
      "levelOfDetail : Y axis in 2D space (should be greater than 1)" )
    ( "groupDilation",
      encoderParams.groupDilation_,
      encoderParams.groupDilation_, 
      "Group Dilation" )
    ( "textureDilationOffLossless",
      encoderParams.textureDilationOffLossless_,
      encoderParams.textureDilationOffLossless_, 
      "Group Dilation" )

    // Lossy occupancy map coding
    ( "offsetLossyOM",
      encoderParams.offsetLossyOM_,
      encoderParams.offsetLossyOM_,
      "Value to be assigned to non-zero occupancy map positions (default=0)\n" )
    ( "thresholdLossyOM",
      encoderParams.thresholdLossyOM_,
      encoderParams.thresholdLossyOM_,
      "Threshold for converting non-binary occupancy map to binary (default=0)\n" )
    ( "prefilterLossyOM",
      encoderParams.prefilterLossyOM_,
      encoderParams.prefilterLossyOM_,
      "Selects whether the occupany map is prefiltered before lossy compression (default=false)\n" )

    // visual quality
    ( "patchColorSubsampling",
      encoderParams.patchColorSubsampling_, false,
      "Enable per patch color sub-sampling\n" )
    ( "deltaCoding",
      encoderParams.deltaCoding_,
      encoderParams.deltaCoding_,
      "Delta meta-data coding" )
    ( "maxNumRefAtalsList",
    //
      encoderParams.maxNumRefAtlasList_,
      encoderParams.maxNumRefAtlasList_,
      "maximum Number of Reference Patch list, default: 1" )
    ( "maxNumRefAtlasFrame",
      encoderParams.maxNumRefAtlasFrame_,
      encoderParams.maxNumRefAtlasFrame_,
      "maximum Number of Reference Atlas Frame per list, default: 1" )
    ( "pointLocalReconstruction",
      encoderParams.pointLocalReconstruction_,
      encoderParams.pointLocalReconstruction_,
      "Use point local reconstruction" )
    ( "mapCountMinus1",
      encoderParams.mapCountMinus1_,
      encoderParams.mapCountMinus1_, 
      "Numbers of layers (rename to maps?)" )
    ( "singleLayerPixelInterleaving",
      encoderParams.singleMapPixelInterleaving_,
      encoderParams.singleMapPixelInterleaving_,
      "Use single layer pixel interleaving" )
    ( "removeDuplicatePoints",
      encoderParams.removeDuplicatePoints_,
      encoderParams.removeDuplicatePoints_, 
      "Remove duplicate points( " )
    ( "surfaceSeparation",
      encoderParams.surfaceSeparation_,
      encoderParams.surfaceSeparation_, 
      "surface separation" )

    // high gradient separation
    ( "highGradientSeparation",
      encoderParams.highGradientSeparation_,
      encoderParams.highGradientSeparation_,
      "Separate high gradient points from a patch" )
    ( "minGradient",
      encoderParams.minGradient_,
      encoderParams.minGradient_,
      "Minimun gradient for a point to be separated" )
    ( "minNumHighGradientPoints",
      encoderParams.minNumHighGradientPoints_,
      encoderParams.minNumHighGradientPoints_,
      "Minimum number of connected high gradient points to be separated from a patch" )

    // flexible packing + safeguard + push-pull
    ( "packingStrategy",
      encoderParams.packingStrategy_,
      encoderParams.packingStrategy_,
      "Patches packing strategy(0: anchor packing, 1(default): flexible packing, 2: tetris packing)\n" )
    ( "useEightOrientations",
      encoderParams.useEightOrientations_,
      encoderParams.useEightOrientations_,
      "Allow either 2 orientations (0(default): NULL AND SWAP), or 8 orientation (1)\n" )
    ( "safeGuardDistance",
      encoderParams.safeGuardDistance_,
      encoderParams.safeGuardDistance_,
      "Number of empty blocks that must exist between the patches (default=1)\n" )
    ( "textureBGFill",
      encoderParams.textureBGFill_,
      encoderParams.textureBGFill_,
      "Selects the background filling operation for texture only (0: patch-edge extension, "
      "1(default): smoothed push-pull algorithm), 2: harmonic background filling " )

    // lossy-raw-points patch
    ( "lossyRawPointsPatch",
      encoderParams.lossyRawPointsPatch_,
      encoderParams.lossyRawPointsPatch_,
      "Lossy raw points patch(0: no lossy raw points patch, 1: enable lossy raw points patch (default=0)\n" )
    ( "minNormSumOfInvDist4MPSelection",
      encoderParams.minNormSumOfInvDist4MPSelection_,
      encoderParams.minNormSumOfInvDist4MPSelection_,
      "Minimum normalized sum of inverse distance for raw points selection: double value between 0.0 and 1.0 (default=0.35)\n" )
    ( "lossyRawPointPatchGeoQP",
      encoderParams.lossyRawPointPatchGeoQP_,
      encoderParams.lossyRawPointPatchGeoQP_,
      "QP value for geometry in lossy raw points patch (default=4)\n" )
    ( "globalPatchAllocation",
      encoderParams.globalPatchAllocation_,
      encoderParams.globalPatchAllocation_,
      "Global temporally consistent patch allocation.(0: anchor's packing method(default), 1: gpa algorithm, 2: gtp algorithm)\n" )
    ( "globalPackingStrategyGOF",
      encoderParams.globalPackingStrategyGOF_,
      encoderParams.globalPackingStrategyGOF_,
      "Number of frames to pack globally (0:(entire GOF))\n" )
    ( "globalPackingStrategyReset",
      encoderParams.globalPackingStrategyReset_,
      encoderParams.globalPackingStrategyReset_,
      "Remove the reference to the previous frame (0(default), 1)\n" )
    ( "globalPackingStrategyThreshold",
      encoderParams.globalPackingStrategyThreshold_,
      encoderParams.globalPackingStrategyThreshold_,
      "matched patches area ratio threshold (decides if connections are valid or not, 0(default))\n" )
    ( "patchPrecedenceOrder",
      encoderParams.patchPrecedenceOrderFlag_,
      encoderParams.patchPrecedenceOrderFlag_, 
      "Order of patches\n" )
    ( "lowDelayEncoding",
      encoderParams.lowDelayEncoding_,
      encoderParams.lowDelayEncoding_,
      "Low Delay encoding (0(default): do nothing, 1: does not allow overlap of patches bounding boxes for low delay encoding)\n" )
    ( "geometryPadding",
      encoderParams.geometryPadding_,
      encoderParams.geometryPadding_,
      "Selects the background filling operation for geometry (0: anchor, 1(default): 3D geometry padding)\n" )
    ( "apply3dMotionCompensation",
      encoderParams.use3dmc_,
      encoderParams.use3dmc_,
      "Use auxilliary information for 3d motion compensation.(0: conventional video coding, 1: 3D motion compensated)\n" )
    ( "geometry3dCoordinatesBitdepth",
      encoderParams.geometry3dCoordinatesBitdepth_,
      encoderParams.geometry3dCoordinatesBitdepth_,
      "Bit depth of geomtery 3D coordinates" )
    ( "geometryNominal2dBitdepth",
      encoderParams.geometryNominal2dBitdepth_,
      encoderParams.geometryNominal2dBitdepth_, 
      "Bit depth of geometry 2D" )
    ( "nbPlrmMode",
      encoderParams.plrlNumberOfModes_,
      encoderParams.plrlNumberOfModes_, 
      "Number of PLR mode" )
    ( "patchSize",
      encoderParams.patchSize_,
      encoderParams.patchSize_,
      "Size of Patch for PLR" )
    ( "enhancedProjectionPlane",
      encoderParams.enhancedPP_,
      encoderParams.enhancedPP_,
      "Use enhanced Projection Plane(0: OFF, 1: ON)\n" )
    ( "minWeightEPP",
      encoderParams.minWeightEPP_,
      encoderParams.minWeightEPP_, 
      "Minimum value\n" )
    ( "additionalProjectionPlaneMode",
      encoderParams.additionalProjectionPlaneMode_,
      encoderParams.additionalProjectionPlaneMode_,
      "additiona Projection Plane Mode 0:none 1:Y-Axis 2:X-Axis 3:Z-Axis 4:All-Axis 5:apply to portion " )
    ( "partialAdditionalProjectionPlane",
      encoderParams.partialAdditionalProjectionPlane_,
      encoderParams.partialAdditionalProjectionPlane_,
      "The value determines the partial point cloud. It's available with only additionalProjectionPlaneMode(5)" )

    // Point cloud partitions (ROIs) and tiles (m47804 CE2.19)
    ( "enablePointCloudPartitioning",
      encoderParams.enablePointCloudPartitioning_,
      encoderParams.enablePointCloudPartitioning_,
      " " )
    ( "roiBoundingBoxMinX",
      encoderParams.roiBoundingBoxMinX_,
      encoderParams.roiBoundingBoxMinX_,
      " " )
    ( "roiBoundingBoxMaxX",
      encoderParams.roiBoundingBoxMaxX_,
      encoderParams.roiBoundingBoxMaxX_,
      " " )
    ( "roiBoundingBoxMinY",
      encoderParams.roiBoundingBoxMinY_,
      encoderParams.roiBoundingBoxMinY_, 
      " " )
    ( "roiBoundingBoxMaxY",
      encoderParams.roiBoundingBoxMaxY_,
      encoderParams.roiBoundingBoxMaxY_,
      " " )
    ( "roiBoundingBoxMinZ",
      encoderParams.roiBoundingBoxMinZ_,
      encoderParams.roiBoundingBoxMinZ_,
      " " )
    ( "roiBoundingBoxMaxZ",
      encoderParams.roiBoundingBoxMaxZ_,
      encoderParams.roiBoundingBoxMaxZ_, 
      " " )
    ( "numTilesHor",
      encoderParams.numTilesHor_,
      encoderParams.numTilesHor_,
      " " )
    ( "tileHeightToWidthRatio",
      encoderParams.tileHeightToWidthRatio_,
      encoderParams.tileHeightToWidthRatio_,
      " " )
    ( "numCutsAlong1stLongestAxis",
      encoderParams.numCutsAlong1stLongestAxis_,
      encoderParams.numCutsAlong1stLongestAxis_,
      " " )
    ( "numCutsAlong2ndLongestAxis",
      encoderParams.numCutsAlong2ndLongestAxis_,
      encoderParams.numCutsAlong2ndLongestAxis_,
      " " )
    ( "numCutsAlong3rdLongestAxis",
      encoderParams.numCutsAlong3rdLongestAxis_,
      encoderParams.numCutsAlong3rdLongestAxis_, 
      " " )

    // Sort raw points by Morton code (m49363 CE2.25)
    ( "mortonOrderSortRawPoints",
      encoderParams.mortonOrderSortRawPoints_,
      encoderParams.mortonOrderSortRawPoints_, 
      " " )

    // Patch block filtering
    ( "pbfEnableFlag",
      encoderParams.pbfEnableFlag_,
      encoderParams.pbfEnableFlag_, 
      " enable patch block filtering \n" )
    ( "pbfFilterSize",
      encoderParams.pbfFilterSize_,
      encoderParams.pbfFilterSize_, 
      "pbfFilterSize \n" )
    ( "pbfPassesCount",
      encoderParams.pbfPassesCount_,
      encoderParams.pbfPassesCount_, 
      "pbfPassesCount \n" )
    ( "pbfLog2Threshold",
      encoderParams.pbfLog2Threshold_,
      encoderParams.pbfLog2Threshold_, 
      "pbfLog2Threshold \n" );

    opts.addOptions()
    ( "computeChecksum", 
      metricsParams.computeChecksum_,
      metricsParams.computeChecksum_, 
      "Compute checksum" )
    ( "computeMetrics", 
      metricsParams.computeMetrics_,
      metricsParams.computeMetrics_, 
      "Compute metrics" )
    ( "normalDataPath", 
      metricsParams.normalDataPath_,
      metricsParams.normalDataPath_,
      "Input pointcloud to encode. Multi-frame sequences may be represented by %04i" )
    ( "resolution", 
      metricsParams.resolution_,
      metricsParams.resolution_, 
      "Specify the intrinsic resolution" )
    ( "dropdups", 
      metricsParams.dropDuplicates_, 
      metricsParams.dropDuplicates_,
      "0(detect), 1(drop), 2(average) subsequent points with same coordinates" )
    ( "neighborsProc", 
      metricsParams.neighborsProc_,
      metricsParams.neighborsProc_,
      "0(undefined), 1(average), 2(weighted average), 3(min), 4(max) neighbors with same geometric distance" );
  // clang-format on
  po::setDefaults( opts );
  po::ErrorReporter        err;
  const list<const char*>& argv_unhandled = po::scanArgv( opts, argc, (const char**)argv, err );

  for ( const auto arg : argv_unhandled ) { printf( "Unhandled argument ignored: %s \n", arg ); }

  if ( argc == 1 || print_help ) {
    po::doHelp( std::cout, opts, 78 );
    return false;
  }

  if ( ( encoderParams.levelOfDetailX_ == 0 || encoderParams.levelOfDetailY_ == 0 ) ) {
    if ( encoderParams.levelOfDetailX_ == 0 ) { encoderParams.levelOfDetailX_ = 1; }
    if ( encoderParams.levelOfDetailY_ == 0 ) { encoderParams.levelOfDetailY_ = 1; }
    std::cerr << "levelOfDetailX and levelOfDetailY should be greater than "
                 "1. levelOfDetailX="
              << encoderParams.levelOfDetailX_ << "., levelOfDetailY=" << encoderParams.levelOfDetailY_ << std::endl;
  }

  if ( encoderParams.losslessGeo_ && ( encoderParams.levelOfDetailX_ > 1 || encoderParams.levelOfDetailY_ > 1 ) ) {
    encoderParams.levelOfDetailX_ = 1;
    encoderParams.levelOfDetailY_ = 1;
    std::cerr << "scaling is not allowed in lossless case\n";
  }
  if ( encoderParams.enablePointCloudPartitioning_ && encoderParams.patchExpansion_ ) {
    std::cerr << "Point cloud partitioning does not currently support patch "
                 "expansion. \n";
  }
  if ( encoderParams.enablePointCloudPartitioning_ && ( encoderParams.globalPatchAllocation_ != 0 ) ) {
    std::cerr << "Point cloud partitioning does not currently support global "
                 "patch allocation. \n";
  }

  if ( static_cast<int>( encoderParams.patchPrecedenceOrderFlag_ ) == 0 && encoderParams.lowDelayEncoding_ ) {
    encoderParams.lowDelayEncoding_ = false;
    std::cerr << "Low Delay Encoding can be used only when "
                 "patchPrecendenceOrder is enabled. lowDelayEncoding_ is set "
                 "0\n";
  }
  encoderParams.completePath();
  encoderParams.print();
  if ( !encoderParams.check() ) { std::cerr << "Input encoder parameters not correct \n"; }
  metricsParams.completePath();
  metricsParams.print();
  if ( !metricsParams.check() ) { std::cerr << "Input metrics parameters not correct \n"; }
  metricsParams.startFrameNumber_ = encoderParams.startFrameNumber_;

  // report the current configuration (only in the absence of errors so
  // that errors/warnings are more obvious and in the same place).
  if ( err.is_errored ) { return false; }

  return true;
}

int compressVideo( const PCCEncoderParameters& encoderParams,
                   const PCCMetricsParameters& metricsParams,
                   StopwatchUserTime&          clock ) {
  const size_t startFrameNumber0        = encoderParams.startFrameNumber_;
  const size_t endFrameNumber0          = encoderParams.startFrameNumber_ + encoderParams.frameCount_;
  const size_t groupOfFramesSize0       = ( std::max )( size_t( 1 ), encoderParams.groupOfFramesSize_ );
  size_t       startFrameNumber         = startFrameNumber0;
  size_t       reconstructedFrameNumber = encoderParams.startFrameNumber_;

  std::unique_ptr<uint8_t> buffer;
  size_t                   contextIndex = 0;
  PCCEncoder               encoder;
  encoder.setParameters( encoderParams );
  std::vector<std::vector<uint8_t>> reconstructedChecksums;
  std::vector<std::vector<uint8_t>> sourceReorderChecksums;
  std::vector<std::vector<uint8_t>> reconstructedReorderChecksums;
  PCCMetrics                        metrics;
  PCCChecksum                       checksum;
  metrics.setParameters( metricsParams );
  checksum.setParameters( metricsParams );

  PCCBitstreamStat     bitstreamStat;
  SampleStreamVpccUnit ssvu;
  // Place to get/set default values for gof metadata enabled flags (in sequence
  // level).
  while ( startFrameNumber < endFrameNumber0 ) {
    size_t     endFrameNumber = min( startFrameNumber + groupOfFramesSize0, endFrameNumber0 );
    PCCContext context;
    context.setBitstreamStat( bitstreamStat );
    context.addVpccParameterSet( contextIndex );
    context.setActiveVpsId( contextIndex );

    PCCGroupOfFrames sources;
    PCCGroupOfFrames reconstructs;
    if ( !sources.load( encoderParams.uncompressedDataPath_, startFrameNumber, endFrameNumber,
                        encoderParams.colorTransform_ ) ) {
      return -1;
    }
    if ( sources.getFrameCount() < endFrameNumber - startFrameNumber ) {
      endFrameNumber = startFrameNumber + sources.getFrameCount();
    }
    clock.start();
    std::cout << "Compressing group of frames " << contextIndex << ": " << startFrameNumber << " -> " << endFrameNumber
              << "..." << std::endl;
    int ret = encoder.encode( sources, context, reconstructs );

    PCCBitstreamWriter bitstreamWriter;
#ifdef BITSTREAM_TRACE
    PCCBitstream bitstream;
    bitstream.setTrace( true );
    bitstream.openTrace( stringFormat( "%s_GOF%u_hls_encode.txt",
                                       removeFileExtension( encoderParams.compressedStreamPath_ ).c_str(),
                                       context.getVps().getVpccParameterSetId() ) );
    bitstreamWriter.setTraceFile( bitstream.getTraceFile() );
#endif
    ret |= bitstreamWriter.encode( context, ssvu );
#ifdef BITSTREAM_TRACE
    bitstreamWriter.setTraceFile( NULL );
    bitstream.closeTrace();
#endif
    clock.stop();

    PCCGroupOfFrames normals;
    bool             bRunMetric = true;
    if ( metricsParams.computeMetrics_ ) {
      if ( !metricsParams.normalDataPath_.empty() ) {
        if ( !normals.load( metricsParams.normalDataPath_, startFrameNumber, endFrameNumber, COLOR_TRANSFORM_NONE,
                            true ) ) {
          bRunMetric = false;
        }
      }
      if ( bRunMetric ) { metrics.compute( sources, reconstructs, normals ); }
    }
    if ( metricsParams.computeChecksum_ ) {
      if ( encoderParams.losslessGeo_ ) {
        checksum.computeSource( sources );
        checksum.computeReordered( reconstructs );
      }
      checksum.computeReconstructed( reconstructs );
    }
    if ( ret != 0 ) { return ret; }
    if ( !encoderParams.reconstructedDataPath_.empty() ) {
      reconstructs.write( encoderParams.reconstructedDataPath_, reconstructedFrameNumber );
    }
    sources.clear();
    reconstructs.clear();
    normals.clear();
    startFrameNumber = endFrameNumber;
    contextIndex++;
  }

  PCCBitstream bitstream;
#ifdef BITSTREAM_TRACE
  bitstream.setTrace( true );
  bitstream.openTrace( removeFileExtension( encoderParams.compressedStreamPath_ ) + "_samplestream_write.txt" );
  bitstream.setTraceFile( bitstream.getTraceFile() );
#endif
  bitstream.writeHeader();
  bitstreamStat.setHeader( bitstream.size() );
  PCCBitstreamWriter bitstreamWriter;
  size_t             headerSize = bitstreamWriter.write( ssvu, bitstream );
  bitstreamStat.incrHeader( headerSize );
  bitstream.write( encoderParams.compressedStreamPath_ );

  bitstreamStat.trace();
  std::cout << "Total bitstream size " << bitstream.size() << " B" << std::endl;

  if ( metricsParams.computeMetrics_ ) { metrics.display(); }
  bool checksumEqual = true;
  if ( metricsParams.computeChecksum_ ) {
    if ( encoderParams.losslessGeo_ ) { checksumEqual = checksum.compareSrcRec(); }
    checksum.write( encoderParams.compressedStreamPath_ );
  }
  return checksumEqual ? 0 : -1;
}
