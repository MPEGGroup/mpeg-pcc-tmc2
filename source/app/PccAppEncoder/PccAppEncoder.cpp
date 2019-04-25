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

    ("maxNNCountRefineSegmentation",
     encoderParams.maxNNCountRefineSegmentation_,
     encoderParams.maxNNCountRefineSegmentation_,
     "Number of nearest neighbors used during segmentation refinement")

    ("iterationCountRefineSegmentation",
     encoderParams.iterationCountRefineSegmentation_,
     encoderParams.iterationCountRefineSegmentation_,
     "Number of iterations performed during segmentation refinement")

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
  
    ("maxAllowedDepth",
     encoderParams.maxAllowedDepth_,
     encoderParams.maxAllowedDepth_,
     "Maximum depth per patch")

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
     "todo(kmammou)")

    ("radius2Smoothing",
     encoderParams.radius2Smoothing_,
     encoderParams.radius2Smoothing_,
     "todo(kmammou)")

    ("radius2BoundaryDetection",
     encoderParams.radius2BoundaryDetection_,
     encoderParams.radius2BoundaryDetection_,
     "todo(kmammou)")

    ("thresholdSmoothing",
     encoderParams.thresholdSmoothing_,
     encoderParams.thresholdSmoothing_,
     "todo(kmammou)")

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
     "todo(kmammou)")

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

    ("losslessTexture",
     encoderParams.losslessTexture_,
     encoderParams.losslessTexture_,
     "Enable lossless encoding of texture\n")

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

    // patch sampling resolution
    ("testLevelOfDetail",
     encoderParams.testLevelOfDetail_,
     encoderParams.testLevelOfDetail_,
     "Force non-zero level of detail for testing")
    ("testLevelOfDetailSignaling",
     encoderParams.testLevelOfDetailSignaling_,
     encoderParams.testLevelOfDetailSignaling_,
     "Test the patch resolution signaling with pseudo-random values")

    ("groupDilation",
      encoderParams.groupDilation_,
      encoderParams.groupDilation_,
      "Group Dilation")     

    ("textureDilationOffLossless",
      encoderParams.textureDilationOffLossless_,
      encoderParams.textureDilationOffLossless_,
      "Group Dilation")

    //EDD code
    ("enhancedDeltaDepthCode",
      encoderParams.enhancedDeltaDepthCode_,
      encoderParams.enhancedDeltaDepthCode_,
      "Use enhanced-delta-depth code")

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

    ("projectionMode",
      encoderParams.projectionMode_,
      encoderParams.projectionMode_,
      "projectionMode - 0:min; 1:max;  2:adaptive frame & patch; 3:adaptive patch (all frames)")

    ("oneLayerMode",
      encoderParams.oneLayerMode_,
      encoderParams.oneLayerMode_,
      "Use one layer mode")

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
      "Patches packing strategy(0: anchor packing, 1(default): flexible packing with 2 orientations, 2: tetris packing)\n")

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
      "Global temporally consistent patch allocation.(0: anchor's packing method(default), 1: gpa algorithm)\n")

    ("apply3dMotionCompensation",
      encoderParams.use3dmc_,
      encoderParams.use3dmc_,
      "Use auxilliary information for 3d motion compensation.(0: conventional video coding, 1: 3D motion compensated)\n")

	("enhancedProjectionPlane",
      encoderParams.enhancedPP_,
	  encoderParams.enhancedPP_,
	 "Use enhanced Projection Plane(0: OFF, 1: ON)\n")

	("minWeightEPP",
      encoderParams.minWeightEPP_,
	  encoderParams.minWeightEPP_,
	  "Minimum value\n")

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

    if ( encoderParams.testLevelOfDetail_ > 0 ) {
      const double lodScale = 1.0 / double( 1u << encoderParams.testLevelOfDetail_ );
      for ( auto& frame : sources.getFrames() ) {
        for ( auto& point : frame.getPositions() ) { point *= lodScale; }
      }
    }

    std::cout << "Compressing group of frames " << contextIndex << ": " << startFrameNumber << " -> " << endFrameNumber
              << "..." << std::endl;
    int ret = encoder.encode( sources, context, bitstream, reconstructs );
    clock.stop();

    PCCGroupOfFrames normals;
    if ( metricsParams.computeMetrics_ ) {
      if ( metricsParams.normalDataPath_ != "" ) {
        if ( !normals.load( metricsParams.normalDataPath_, startFrameNumber, endFrameNumber, COLOR_TRANSFORM_NONE ) ) {
          return -1;
        }
      }
      metrics.compute( sources, reconstructs, normals );
    }
    if ( metricsParams.computeChecksum_ ) {
      if ( encoderParams.losslessGeo_ && encoderParams.losslessTexture_ ) {
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
    if ( encoderParams.losslessGeo_ && encoderParams.losslessTexture_ ) { checksumEqual = checksum.compareSrcRec(); }
    checksum.write( encoderParams.compressedStreamPath_ );
  }
  return checksumEqual ? 0 : -1;
}
