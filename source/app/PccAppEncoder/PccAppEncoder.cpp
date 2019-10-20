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
  if ( encoderParams.nbThread_ > 0 ) { tbb::task_scheduler_init init( (int)encoderParams.nbThread_ ); }

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
    ("help", print_help, false, "This help text")
    ("c,config", po::parseConfigFile,"Configuration file name")

    ("configurationFolder",
     encoderParams.configurationFolder_,
     encoderParams.configurationFolder_,
     "Folder where the configuration files are stored, use for cfg relative paths.")

    ("uncompressedDataFolder",
     encoderParams.uncompressedDataFolder_,
     encoderParams.uncompressedDataFolder_,
     "Folder where the uncompress input data are stored, use for cfg relative paths.")

    // i/o
    ("uncompressedDataPath",
     encoderParams.uncompressedDataPath_,
     encoderParams.uncompressedDataPath_,
     "Input pointcloud to encode. Multi-frame sequences may be represented by %04i")

    ("compressedStreamPath",
     encoderParams.compressedStreamPath_,
     encoderParams.compressedStreamPath_,
     "Output(encoder)/Input(decoder) compressed bitstream")

    ("reconstructedDataPath",
     encoderParams.reconstructedDataPath_,
     encoderParams.reconstructedDataPath_,
     "Output decoded pointcloud. Multi-frame sequences may be represented by %04i")

    // sequence configuration
    ("startFrameNumber",
     encoderParams.startFrameNumber_,
     encoderParams.startFrameNumber_,
     "First frame number in sequence to encode/decode")

    ("frameCount",
     encoderParams.frameCount_,
     encoderParams.frameCount_,
     "Number of frames to encode")

    ("groupOfFramesSize",
     encoderParams.groupOfFramesSize_,
     encoderParams.groupOfFramesSize_,
     "Random access period")

    // colour space conversion
    ("colorTransform",
     encoderParams.colorTransform_,
     encoderParams.colorTransform_,
     "The colour transform to be applied:\n  0: none\n  1: RGB to YCbCr (Rec.709)")

    ("colorSpaceConversionPath",
     encoderParams.colorSpaceConversionPath_,
     encoderParams.colorSpaceConversionPath_,
     "Path to the HDRConvert. If unset, an internal color space conversion is used")

    ("colorSpaceConversionConfig",
     encoderParams.colorSpaceConversionConfig_,
     encoderParams.colorSpaceConversionConfig_,
     "HDRConvert configuration file used for RGB444 to YUV420 conversion")

    ("inverseColorSpaceConversionConfig",
     encoderParams.inverseColorSpaceConversionConfig_,
     encoderParams.inverseColorSpaceConversionConfig_,
     "HDRConvert configuration file used for YUV420 to RGB444 conversion")

    // segmentation
    ("nnNormalEstimation",
     encoderParams.nnNormalEstimation_,
     encoderParams.nnNormalEstimation_,
     "Number of points used for normal estimation")

    ("gridBasedRefineSegmentation",
     encoderParams.gridBasedRefineSegmentation_,
     encoderParams.gridBasedRefineSegmentation_,
     "Use grid-based approach for segmentation refinement")

    ("maxNNCountRefineSegmentation",
     encoderParams.maxNNCountRefineSegmentation_,
     encoderParams.maxNNCountRefineSegmentation_,
     "Number of nearest neighbors used during segmentation refinement")

    ("iterationCountRefineSegmentation",
     encoderParams.iterationCountRefineSegmentation_,
     encoderParams.iterationCountRefineSegmentation_,
     "Number of iterations performed during segmentation refinement")

    ("voxelDimensionRefineSegmentation",
     encoderParams.voxelDimensionRefineSegmentation_,
     encoderParams.voxelDimensionRefineSegmentation_,
     "Voxel dimension for segmentation refinement (must be a power of 2)")

    ("searchRadiusRefineSegmentation",
     encoderParams.searchRadiusRefineSegmentation_,
     encoderParams.searchRadiusRefineSegmentation_,
     "Search radius for segmentation refinement")
	 
    ("occupancyResolution",
     encoderParams.occupancyResolution_,
     encoderParams.occupancyResolution_,
     "Resolution T of the occupancy map")

    ("minPointCountPerCCPatchSegmentation",
     encoderParams.minPointCountPerCCPatchSegmentation_,
     encoderParams.minPointCountPerCCPatchSegmentation_,
     "Minimum number of points for a connected component to be retained as a patch")

    ("maxNNCountPatchSegmentation",
     encoderParams.maxNNCountPatchSegmentation_,
     encoderParams.maxNNCountPatchSegmentation_,
     "Number of nearest neighbors used during connected components extraction")

    ("surfaceThickness",
     encoderParams.surfaceThickness_,
     encoderParams.surfaceThickness_,
     "Surface thickness")
  
  ("depthQuantizationStep",
   encoderParams.minLevel_,
   encoderParams.minLevel_,
   "minimum level for patches")

  ("maxAllowedDist2MissedPointsDetection",
   encoderParams.maxAllowedDist2MissedPointsDetection_,
   encoderParams.maxAllowedDist2MissedPointsDetection_,
   "Maximum distance for a point to be ignored during missed point detection")

    ("maxAllowedDist2MissedPointsSelection",
     encoderParams.maxAllowedDist2MissedPointsSelection_,
     encoderParams.maxAllowedDist2MissedPointsSelection_,
     "Maximum distance for a point to be ignored during  missed points  selection")

    ("lambdaRefineSegmentation",
     encoderParams.lambdaRefineSegmentation_,
     encoderParams.lambdaRefineSegmentation_,
     "Controls the smoothness of the patch boundaries  during segmentation  refinement")

    // packing
    ("minimumImageWidth",
     encoderParams.minimumImageWidth_,
     encoderParams.minimumImageWidth_,
     "Minimum width of packed patch frame")

    ("minimumImageHeight",
     encoderParams.minimumImageHeight_,
     encoderParams.minimumImageHeight_,
     "Minimum height of packed patch frame")

    // occupancy map
    ("maxCandidateCount",
     encoderParams.maxCandidateCount_,
     encoderParams.maxCandidateCount_,
     "Maximum nuber of candidates in list L")

    ("occupancyPrecision",
    encoderParams.occupancyPrecision_,
    encoderParams.occupancyPrecision_,
    "Occupancy map B0 precision")

    ("occupancyMapVideoEncoderConfig",
     encoderParams.occupancyMapVideoEncoderConfig_,
     encoderParams.occupancyMapVideoEncoderConfig_,
     "Occupancy map encoder config file")
    
    ("occupancyMapQP",
     encoderParams.occupancyMapQP_,
     encoderParams.occupancyMapQP_,
     "QP for compression of occupancy map video")

    //EDD code
    ("enhancedDeltaDepthCode",
      encoderParams.enhancedDeltaDepthCode_,
      encoderParams.enhancedDeltaDepthCode_,
      "Use enhanced-delta-depth code")

    ("EOMFixBitCount",
     encoderParams.EOMFixBitCount_,
     encoderParams.EOMFixBitCount_,
     "enhanced occupancy map fixed bit count")

    ("EOMTexturePatch",
      encoderParams.EOMTexturePatch_,
      encoderParams.EOMTexturePatch_,
      "enhanced occupancy map texture in 1 patch")

    ("occupancyMapRefinement",
      encoderParams.occupancyMapRefinement_,
      encoderParams.occupancyMapRefinement_,
      "Use occupancy map refinement")

    // smoothing
    // NB: various parameters are of the form n * occupancyPrecision**2

    ("flagGeometrySmoothing",
      encoderParams.flagGeometrySmoothing_,
      encoderParams.flagGeometrySmoothing_,
      "Enable geometry smoothing\n")

    ("neighborCountSmoothing",
     encoderParams.neighborCountSmoothing_,
     encoderParams.neighborCountSmoothing_,
     "Neighbor count smoothing")

    ("radius2Smoothing",
     encoderParams.radius2Smoothing_,
     encoderParams.radius2Smoothing_,
     "Radius to smoothing")

    ("radius2BoundaryDetection",
     encoderParams.radius2BoundaryDetection_,
     encoderParams.radius2BoundaryDetection_,
     "Radius to boundary detection")

    ("thresholdSmoothing",
     encoderParams.thresholdSmoothing_,
     encoderParams.thresholdSmoothing_,
     "Threshold smoothing")

    // Patch Expansion (m47772 CE2.12)
	  ("patchExpansion",
  	 encoderParams.patchExpansion_,
	   encoderParams.patchExpansion_,
	   "Use occupancy map refinement")

    //grid smoothing (m44705 CE2.17)
    ("gridSmoothing",
      encoderParams.gridSmoothing_,
      encoderParams.gridSmoothing_,
      "Enable grid smoothing")
        
    ("gridSize", 
      encoderParams.gridSize_,
      encoderParams.gridSize_, 
      "grid size for the smoothing")

  // color smoothing
    ("thresholdColorSmoothing",
     encoderParams.thresholdColorSmoothing_,
     encoderParams.thresholdColorSmoothing_,
     "Threshold of color smoothing")

    ("gridColorSmoothing",
      encoderParams.gridColorSmoothing_,
      encoderParams.gridColorSmoothing_,
      "Enable low complexity color smoothing")

	("cgridSize", 
      encoderParams.cgridSize_,
      encoderParams.cgridSize_, 
      "grid size for the color smoothing")

    ("thresholdColorDifference",
     encoderParams.thresholdColorDifference_,
     encoderParams.thresholdColorDifference_,
     "Threshold of color difference between cells")

    ("thresholdColorVariation",
     encoderParams.thresholdColorVariation_,
     encoderParams.thresholdColorVariation_,
     "Threshold of color variation in cells")

    ("thresholdLocalEntropy",
     encoderParams.thresholdLocalEntropy_,
     encoderParams.thresholdLocalEntropy_,
     "Threshold of local entropy")

    ("radius2ColorSmoothing",
     encoderParams.radius2ColorSmoothing_,
     encoderParams.radius2ColorSmoothing_,
     "Redius of color smoothing")

    ("neighborCountColorSmoothing",
     encoderParams.neighborCountColorSmoothing_,
     encoderParams.neighborCountColorSmoothing_,
     "Neighbor count for color smoothing")

    ("flagColorSmoothing",
     encoderParams.flagColorSmoothing_,
     encoderParams.flagColorSmoothing_,
     "Enable color smoothing\n")

  // color pre-smoothing
    ("thresholdColorPreSmoothing",
     encoderParams.thresholdColorPreSmoothing_,
     encoderParams.thresholdColorPreSmoothing_,
     "Threshold of color pre-smoothing")

    ("thresholdColorPreSmoothingLocalEntropy",
     encoderParams.thresholdColorPreSmoothingLocalEntropy_,
     encoderParams.thresholdColorPreSmoothingLocalEntropy_,
     "Threshold of color pre-smoothing local entropy")

    ("radius2ColorPreSmoothing",
     encoderParams.radius2ColorPreSmoothing_,
     encoderParams.radius2ColorPreSmoothing_,
     "Redius of color pre-smoothing")

    ("neighborCountColorPreSmoothing",
     encoderParams.neighborCountColorPreSmoothing_,
     encoderParams.neighborCountColorPreSmoothing_,
     "Neighbor count for color pre-smoothing")

    ("flagColorPreSmoothing",
     encoderParams.flagColorPreSmoothing_,
     encoderParams.flagColorPreSmoothing_,
     "Enable color pre-smoothing\n")

    // colouring
    ("bestColorSearchRange",
     encoderParams.bestColorSearchRange_,
     encoderParams.bestColorSearchRange_,
     "Best color search range")
    // Improved color transfer (m49367 CE2.17)
    ("numNeighborsColorTransferFwd",
    encoderParams.numNeighborsColorTransferFwd_,
    encoderParams.numNeighborsColorTransferFwd_,
    "Number of neighbors creating Fwd list")

    ("numNeighborsColorTransferBwd",
    encoderParams.numNeighborsColorTransferBwd_,
    encoderParams.numNeighborsColorTransferBwd_,
    "Number of neighbors creating Bwd list")

    ("useDistWeightedAverageFwd",
    encoderParams.useDistWeightedAverageFwd_,
    encoderParams.useDistWeightedAverageFwd_,
    "Distance weighted average for Fwd list")

    ("useDistWeightedAverageBwd",
    encoderParams.useDistWeightedAverageBwd_,
    encoderParams.useDistWeightedAverageBwd_,
    "Distance weighted average for Bwd list")

    ("skipAvgIfIdenticalSourcePointPresentFwd",
    encoderParams.skipAvgIfIdenticalSourcePointPresentFwd_,
    encoderParams.skipAvgIfIdenticalSourcePointPresentFwd_,
    "Skip avgeraging if target is identical to a Fwd point")

    ("skipAvgIfIdenticalSourcePointPresentBwd",
    encoderParams.skipAvgIfIdenticalSourcePointPresentBwd_,
    encoderParams.skipAvgIfIdenticalSourcePointPresentBwd_,
    "Skip avgeraging if target is identical to a Bwd point")

    ("distOffsetFwd",
    encoderParams.distOffsetFwd_,
    encoderParams.distOffsetFwd_,
    "Distance offset to avoid infinite weight")

    ("distOffsetBwd",
    encoderParams.distOffsetBwd_,
    encoderParams.distOffsetBwd_,
    "Distance offset to avoid infinite weight")

    ("maxGeometryDist2Fwd",
    encoderParams.maxGeometryDist2Fwd_,
    encoderParams.maxGeometryDist2Fwd_,
    "Maximum allowed distance for a Fwd point")

    ("maxGeometryDist2Bwd",
    encoderParams.maxGeometryDist2Bwd_,
    encoderParams.maxGeometryDist2Bwd_,
    "Maximum allowed distance for a Bwd point")

    ("maxColorDist2Fwd",
    encoderParams.maxColorDist2Fwd_,
    encoderParams.maxColorDist2Fwd_,
    "Maximum allowed pari-wise color distance for Fwd list")

    ("maxColorDist2Bwd",
    encoderParams.maxColorDist2Bwd_,
    encoderParams.maxColorDist2Bwd_,
    "Maximum allowed pari-wise color distance for Bwd list")

    // video encoding
    ("videoEncoderPath",
     encoderParams.videoEncoderPath_,
     encoderParams.videoEncoderPath_,
     "HM video encoder executable")

    ("videoEncoderAuxPath",
      encoderParams.videoEncoderAuxPath_,
      encoderParams.videoEncoderAuxPath_,
      "HM video encoder executable")

    ("videoEncoderOccupancyMapPath",
     encoderParams.videoEncoderOccupancyMapPath_,
     encoderParams.videoEncoderOccupancyMapPath_,
     "HM lossless video encoder executable for occupancy map")

    ("geometryQP",
     encoderParams.geometryQP_,
     encoderParams.geometryQP_,
     "QP for compression of geometry video")

    ("textureQP",
     encoderParams.textureQP_,
     encoderParams.textureQP_,
     "QP for compression of texture video")

    ("geometryConfig",
     encoderParams.geometryConfig_,
     encoderParams.geometryConfig_,
     "HM configuration file for geometry compression")

    ("geometryD0Config",
     encoderParams.geometryD0Config_,
     encoderParams.geometryD0Config_,
     "HM configuration file for geometry D0 compression")

    ("geometryD1Config",
      encoderParams.geometryD1Config_,
      encoderParams.geometryD1Config_,
      "HM configuration file for geometry D1 compression")

    ("textureConfig",
     encoderParams.textureConfig_,
     encoderParams.textureConfig_,
     "HM configuration file for texture compression")

    // lossless parameters
    ("losslessGeo",
     encoderParams.losslessGeo_,
     encoderParams.losslessGeo_,
     "Enable lossless encoding of geometry\n")
    ("noAttributes",
     encoderParams.noAttributes_,
     encoderParams.noAttributes_,
     "Disable encoding of attributes")

    ("losslessGeo444",
     encoderParams.losslessGeo444_,
     encoderParams.losslessGeo444_,
     "Use 4444 format for lossless geometry")

    ("useMissedPointsSeparateVideo",
     encoderParams.useMissedPointsSeparateVideo_,
     encoderParams.useMissedPointsSeparateVideo_,
     "compress missed point with video codec")

    ("textureMPSeparateVideoWidth",
      encoderParams.textureMPSeparateVideoWidth_,
      encoderParams.textureMPSeparateVideoWidth_,
      "Width of the MP's texture in separate video")

    ("geometryMPConfig",
     encoderParams.geometryMPConfig_,
     encoderParams.geometryMPConfig_,
     "HM configuration file for missed points geometry compression")

    ("textureMPConfig",
     encoderParams.textureMPConfig_,
     encoderParams.textureMPConfig_,
     "HM configuration file for missed points texture compression")
  
    //etc
    ("nbThread",
     encoderParams.nbThread_,
     encoderParams.nbThread_,
     "Number of thread used for parallel processing")

    ("keepIntermediateFiles",
     encoderParams.keepIntermediateFiles_,
     encoderParams.keepIntermediateFiles_,
     "Keep intermediate files: RGB, YUV and bin")

    ("absoluteD1",
     encoderParams.absoluteD1_,
     encoderParams.absoluteD1_,
     "Absolute D1")

    ("constrainedPack",
     encoderParams.constrainedPack_,
     encoderParams.constrainedPack_,
     "Temporally consistent patch packing")

  ("levelOfDetailX",
   encoderParams.levelOfDetailX_,
   encoderParams.levelOfDetailX_,
   "levelOfDetail : X axis in 2D space (should be greater than 1)")
  ("levelOfDetailY",
   encoderParams.levelOfDetailY_,
   encoderParams.levelOfDetailY_,
   "levelOfDetail : Y axis in 2D space (should be greater than 1)")
  
  ("testLevelOfDetail",
   encoderParams.testLevelOfDetail_,
   encoderParams.testLevelOfDetail_,
   "testLevelOfDetail : 1.scaling in 3D space")
  
    ("groupDilation",
      encoderParams.groupDilation_,
      encoderParams.groupDilation_,
      "Group Dilation")     

    ("textureDilationOffLossless",
      encoderParams.textureDilationOffLossless_,
      encoderParams.textureDilationOffLossless_,
      "Group Dilation")

    // Lossy occupancy map coding
    ("offsetLossyOM",
      encoderParams.offsetLossyOM_,
      encoderParams.offsetLossyOM_,
      "Value to be assigned to non-zero occupancy map positions (default=0)\n")

    ("thresholdLossyOM",
      encoderParams.thresholdLossyOM_,
      encoderParams.thresholdLossyOM_,
      "Threshold for converting non-binary occupancy map to binary (default=0)\n")

    ("prefilterLossyOM",
      encoderParams.prefilterLossyOM_,
      encoderParams.prefilterLossyOM_,
      "Selects whether the occupany map is prefiltered before lossy compression (default=false)\n")

    //visual quality
    ("patchColorSubsampling", 
     encoderParams.patchColorSubsampling_, 
     false, 
     "Enable per patch color sub-sampling\n")
    ("deltaCoding",
     encoderParams.deltaCoding_,
     encoderParams.deltaCoding_,
     "Delta meta-data coding")

    ("pointLocalReconstruction",
      encoderParams.pointLocalReconstruction_,
      encoderParams.pointLocalReconstruction_,
      "Use point local reconstruction")

	  ("layerCountMinus1",
      encoderParams.layerCountMinus1_,
      encoderParams.layerCountMinus1_,
      "Numbers of layers (rename to maps?)")

    ("singleLayerPixelInterleaving",
      encoderParams.singleLayerPixelInterleaving_,
      encoderParams.singleLayerPixelInterleaving_,
      "Use single layer pixel interleaving")

    ("removeDuplicatePoints",
      encoderParams.removeDuplicatePoints_,
      encoderParams.removeDuplicatePoints_,
      "Remove duplicate points( ")

    ("surfaceSeparation",
      encoderParams.surfaceSeparation_,
      encoderParams.surfaceSeparation_,
      "surface separation")

    //flexible packing + safeguard + push-pull
    ("packingStrategy",
      encoderParams.packingStrategy_,
      encoderParams.packingStrategy_,
      "Patches packing strategy(0: anchor packing, 1(default): flexible packing, 2: tetris packing)\n")

	("useEightOrientations",
		encoderParams.useEightOrientations_,
		encoderParams.useEightOrientations_,
		"Allow either 2 orientations (0(default): NULL AND SWAP), or 8 orientation (1)\n")
			
    ("safeGuardDistance",
      encoderParams.safeGuardDistance_,
      encoderParams.safeGuardDistance_,
      "Number of empty blocks that must exist between the patches (default=1)\n")

    ("textureBGFill",
      encoderParams.textureBGFill_,
      encoderParams.textureBGFill_,
      "Selects the background filling operation for texture only (0: patch-edge extension, 1(default): smoothed push-pull algorithm), 2: harmonic background filling\n")

    //lossy-missed-points patch
    ("lossyMissedPointsPatch",
      encoderParams.lossyMissedPointsPatch_,
      encoderParams.lossyMissedPointsPatch_,
      "Lossy missed points patch(0: no lossy missed points patch, 1: enable lossy missed points patch (default=0)\n")

    ("minNormSumOfInvDist4MPSelection",
       encoderParams.minNormSumOfInvDist4MPSelection_,
       encoderParams.minNormSumOfInvDist4MPSelection_,
       "Minimum normalized sum of inverse distance for missed points selection: double value between 0.0 and 1.0 (default=0.35)\n")

    ("lossyMppGeoQP",
       encoderParams.lossyMppGeoQP_,
       encoderParams.lossyMppGeoQP_,
       "QP value for geometry in lossy missed points patch (default=4)\n")

    ("globalPatchAllocation",
      encoderParams.globalPatchAllocation_,
      encoderParams.globalPatchAllocation_,
      "Global temporally consistent patch allocation.(0: anchor's packing method(default), 1: gpa algorithm, 2: gtp algorithm)\n")

    ("globalPackingStrategyGOF",
      encoderParams.globalPackingStrategyGOF_,
      encoderParams.globalPackingStrategyGOF_,
      "Number of frames to pack globally (0:(entire GOF))\n")

    ("globalPackingStrategyReset",
      encoderParams.globalPackingStrategyReset_,
      encoderParams.globalPackingStrategyReset_,
      "Remove the reference to the previous frame (0(default), 1)\n")

    ("globalPackingStrategyThreshold",
      encoderParams.globalPackingStrategyThreshold_,
      encoderParams.globalPackingStrategyThreshold_,
      "matched patches area ratio threshold (decides if connections are valid or not, 0(default))\n")

    ("lowDelayEncoding",
	    encoderParams.lowDelayEncoding_,
	    encoderParams.lowDelayEncoding_,
	    "Low Delay encoding (0(default): do nothing, 1: does not allow overlap of patches bounding boxes for low delay encoding)\n")

    ("geometryPadding",
	    encoderParams.geometryPadding_,
		  encoderParams.geometryPadding_,
		  "Selects the background filling operation for geometry (0: anchor, 1(default): 3D geometry padding\n")

    ("apply3dMotionCompensation",
      encoderParams.use3dmc_,
      encoderParams.use3dmc_,
      "Use auxilliary information for 3d motion compensation.(0: conventional video coding, 1: 3D motion compensated)\n")

   ("geometry3dCoordinatesBitdepth", 
      encoderParams.geometry3dCoordinatesBitdepth_,
      encoderParams.geometry3dCoordinatesBitdepth_, 
      "Bit depth of geomtery 3D coordinates")

   ("geometryNominal2dBitdepth", 
      encoderParams.geometryNominal2dBitdepth_,
      encoderParams.geometryNominal2dBitdepth_, 
      "Bit depth of geometry 2D")
  
    ("nbPlrmMode",
        encoderParams.plrlNumberOfModes_,
        encoderParams.plrlNumberOfModes_,
        "Number of PLR mode")

    ("patchSize",
        encoderParams.patchSize_,
        encoderParams.patchSize_,
        "Size of Patch for PLR")

	  ("enhancedProjectionPlane",
      encoderParams.enhancedPP_,
	    encoderParams.enhancedPP_,
	    "Use enhanced Projection Plane(0: OFF, 1: ON)\n")

	  ("minWeightEPP",
      encoderParams.minWeightEPP_,
	    encoderParams.minWeightEPP_,
	    "Minimum value\n")

    ("additionalProjectionPlaneMode",
     encoderParams.additionalProjectionPlaneMode_, 
     encoderParams.additionalProjectionPlaneMode_,
     "additiona Projection Plane Mode 0:none 1:Y-Axis 2:X-Axis 3:Z-Axis 4:All-Axis 5:apply to portion ")

    ("partialAdditionalProjectionPlane",
     encoderParams.partialAdditionalProjectionPlane_, 
     encoderParams.partialAdditionalProjectionPlane_,
     "The value determines the partial point cloud. It's available with only additionalProjectionPlaneMode(5)")
    // Point cloud partitions (ROIs) and tiles (m47804 CE2.19)    
    ("enablePointCloudPartitioning", encoderParams.enablePointCloudPartitioning_, encoderParams.enablePointCloudPartitioning_, " ")
    ("roiBoundingBoxMinX"          , encoderParams.roiBoundingBoxMinX_, encoderParams.roiBoundingBoxMinX_, " ")
    ("roiBoundingBoxMaxX"          , encoderParams.roiBoundingBoxMaxX_, encoderParams.roiBoundingBoxMaxX_, " ")
    ("roiBoundingBoxMinY"          , encoderParams.roiBoundingBoxMinY_, encoderParams.roiBoundingBoxMinY_, " ")
    ("roiBoundingBoxMaxY"          , encoderParams.roiBoundingBoxMaxY_, encoderParams.roiBoundingBoxMaxY_, " ")
    ("roiBoundingBoxMinZ"          , encoderParams.roiBoundingBoxMinZ_, encoderParams.roiBoundingBoxMinZ_, " ")
    ("roiBoundingBoxMaxZ"          , encoderParams.roiBoundingBoxMaxZ_, encoderParams.roiBoundingBoxMaxZ_, " ")
    ("numTilesHor"                 , encoderParams.numTilesHor_, encoderParams.numTilesHor_, " ")
    ("tileHeightToWidthRatio"      , encoderParams.tileHeightToWidthRatio_, encoderParams.tileHeightToWidthRatio_, " ")
    ("numCutsAlong1stLongestAxis"  , encoderParams.numCutsAlong1stLongestAxis_, encoderParams.numCutsAlong1stLongestAxis_, " ")
    ("numCutsAlong2ndLongestAxis"  , encoderParams.numCutsAlong2ndLongestAxis_, encoderParams.numCutsAlong2ndLongestAxis_, " ")
    ("numCutsAlong3rdLongestAxis"  , encoderParams.numCutsAlong3rdLongestAxis_, encoderParams.numCutsAlong3rdLongestAxis_, " ")
    // Sort missed points by Morton code (m49363 CE2.25)
    ("mortonOrderSortMissedPoints" , encoderParams.mortonOrderSortMissedPoints_, encoderParams.mortonOrderSortMissedPoints_, " ")
    ;

   opts.addOptions()
     ("computeChecksum",
      metricsParams.computeChecksum_,
      metricsParams.computeChecksum_,
      "Compute checksum")

     ("computeMetrics",
       metricsParams.computeMetrics_,
       metricsParams.computeMetrics_,
      "Compute metrics")

     ("normalDataPath",
      metricsParams.normalDataPath_,
      metricsParams.normalDataPath_,
      "Input pointcloud to encode. Multi-frame sequences may be represented by %04i")

     ("resolution",
      metricsParams.resolution_,
      metricsParams.resolution_,
      "Specify the intrinsic resolution")

     ("dropdups",
      metricsParams.dropDuplicates_,
      metricsParams.dropDuplicates_,
      "0(detect), 1(drop), 2(average) subsequent points with same coordinates")

     ("neighborsProc",
      metricsParams.neighborsProc_,
      metricsParams.neighborsProc_,
      "0(undefined), 1(average), 2(weighted average), 3(min), 4(max) neighbors with same geometric distance")
      ;
  // clang-format on
  po::setDefaults( opts );
  po::ErrorReporter        err;
  const list<const char*>& argv_unhandled = po::scanArgv( opts, argc, (const char**)argv, err );

  for ( const auto arg : argv_unhandled ) { err.warn() << "Unhandled argument ignored: " << arg << "\n"; }

  if ( argc == 1 || print_help ) {
    po::doHelp( std::cout, opts, 78 );
    return false;
  }

  if ( (encoderParams.levelOfDetailX_==0 || encoderParams.levelOfDetailY_==0) )
  {
    if(encoderParams.levelOfDetailX_==0)  encoderParams.levelOfDetailX_=1;
    if(encoderParams.levelOfDetailY_==0)  encoderParams.levelOfDetailY_=1;
    err.error() << "levelOfDetailX and levelOfDetailY should be greater than 1. levelOfDetailX="<<encoderParams.levelOfDetailX_<<"., levelOfDetailY="<<encoderParams.levelOfDetailY_<<
    std::endl;
  }

  if ( encoderParams.losslessGeo_ &&
      (encoderParams.levelOfDetailX_>1 || encoderParams.levelOfDetailY_>1) ) {
    encoderParams.levelOfDetailX_= 1;
    encoderParams.levelOfDetailY_=1;
    err.error() << "scaling is not allowed in lossless case\n";
  }
  if ( encoderParams.enablePointCloudPartitioning_ && encoderParams.patchExpansion_ ) {
    err.error() << "Point cloud partitioning does not currently support patch expansion. \n";
  }
  if ( encoderParams.enablePointCloudPartitioning_ && encoderParams.globalPatchAllocation_ ) {
    err.error() << "Point cloud partitioning does not currently support global patch allocation. \n";
  }

  encoderParams.completePath();
  encoderParams.print();
  if ( !encoderParams.check() ) { err.error() << "Input encoder parameters not correct \n"; }
  metricsParams.completePath();
  metricsParams.print();
  if ( !metricsParams.check() ) { err.error() << "Input metrics parameters not correct \n"; }
  metricsParams.startFrameNumber_ = encoderParams.startFrameNumber_;

  // report the current configuration (only in the absence of errors so
  // that errors/warnings are more obvious and in the same place).
  if ( err.is_errored ) return false;

  return true;
}

int compressVideo( const PCCEncoderParameters& encoderParams,
                   const PCCMetricsParameters& metricsParams,
                   StopwatchUserTime&          clock ) {
  const size_t             startFrameNumber0        = encoderParams.startFrameNumber_;
  const size_t             endFrameNumber0          = encoderParams.startFrameNumber_ + encoderParams.frameCount_;
  const size_t             groupOfFramesSize0       = ( std::max )( size_t( 1 ), encoderParams.groupOfFramesSize_ );
  size_t                   startFrameNumber         = startFrameNumber0;
  size_t                   reconstructedFrameNumber = encoderParams.startFrameNumber_;
  PCCBitstream             bitstream;
  std::unique_ptr<uint8_t> buffer;
  size_t                   contextIndex = 0;
  PCCEncoder               encoder;
  encoder.setParameters( encoderParams );
  std::vector<std::vector<uint8_t>> reconstructedChecksums, sourceReorderChecksums, reconstructedReorderChecksums;
  PCCMetrics                        metrics;
  PCCChecksum                       checksum;
  metrics.setParameters( metricsParams );
  checksum.setParameters( metricsParams );

  // Place to get/set default values for gof metadata enabled flags (in sequence level).
  bitstream.writeHeader();
  while ( startFrameNumber < endFrameNumber0 ) {
    const size_t endFrameNumber = min( startFrameNumber + groupOfFramesSize0, endFrameNumber0 );
    PCCContext   context;
    context.addSequenceParameterSet( contextIndex );
    context.getVPCC().setSequenceParameterSetId( contextIndex );

    PCCGroupOfFrames sources, reconstructs;
    if ( !sources.load( encoderParams.uncompressedDataPath_, startFrameNumber, endFrameNumber,
                        encoderParams.colorTransform_ ) ) {
      return -1;
    }
    clock.start();

    std::cout << "Compressing group of frames " << contextIndex << ": " << startFrameNumber << " -> " << endFrameNumber
              << "..." << std::endl;
    int ret = encoder.encode( sources, context, bitstream, reconstructs );
    clock.stop();

    PCCGroupOfFrames normals;
    bool             bRunMetric = true;
    if ( metricsParams.computeMetrics_ ) {
      if ( metricsParams.normalDataPath_ != "" ) {
        if ( !normals.load( metricsParams.normalDataPath_, startFrameNumber, endFrameNumber, COLOR_TRANSFORM_NONE,
                            true ) ) {
          bRunMetric = false;
        }
      }
      if ( bRunMetric ) metrics.compute( sources, reconstructs, normals );
    }
    if ( metricsParams.computeChecksum_ ) {
      if ( encoderParams.losslessGeo_ )
      {
        checksum.computeSource( sources );
        checksum.computeReordered( reconstructs );
      }
      checksum.computeReconstructed( reconstructs );
    }
    if ( ret ) { return ret; }
    if ( !encoderParams.reconstructedDataPath_.empty() ) {
      reconstructs.write( encoderParams.reconstructedDataPath_, reconstructedFrameNumber );
    }
    sources.clear();
    reconstructs.clear();
    normals.clear();
    startFrameNumber = endFrameNumber;
    contextIndex++;
  }
  bitstream.getBitStreamStat().trace();
  std::cout << "Total bitstream size " << bitstream.size() << " B" << std::endl;
  bitstream.write( encoderParams.compressedStreamPath_ );

  if ( metricsParams.computeMetrics_ ) { metrics.display(); }
  bool checksumEqual = true;
  if ( metricsParams.computeChecksum_ ) {
    if ( encoderParams.losslessGeo_ )
    { checksumEqual = checksum.compareSrcRec(); }
    checksum.write( encoderParams.compressedStreamPath_ );
  }
  return checksumEqual ? 0 : -1;
}
