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

int main(int argc, char *argv[]) {
  std::cout << "PccAppEncoder v" << TMC2_VERSION_MAJOR << "." << TMC2_VERSION_MINOR << std::endl
            << std::endl;

  PCCEncoderParameters params;
  if (!parseParameters(argc, argv, params )) {
    return -1;
  }
  if( params.nbThread_ > 0 ) {
    tbb::task_scheduler_init init( (int)params.nbThread_ );
  }

  // Timers to count elapsed wall/user time
  pcc::chrono::Stopwatch<std::chrono::steady_clock> clockWall;
  pcc::chrono::StopwatchUserTime clockUser;

  clockWall.start();
  int ret = compressVideo(params, clockUser);
  clockWall.stop();

  using namespace std::chrono;
  using ms = milliseconds;
  auto totalWall = duration_cast<ms>(clockWall.count()).count();
  std::cout << "Processing time (wall): " << totalWall / 1000.0 << " s\n";

  auto totalUserSelf = duration_cast<ms>(clockUser.self.count()).count();
  std::cout << "Processing time (user.self): "
            << totalUserSelf / 1000.0 << " s\n";

  auto totalUserChild = duration_cast<ms>(clockUser.children.count()).count();
  std::cout << "Processing time (user.children): "
            << totalUserChild / 1000.0 << " s\n";

  return ret;
}

//---------------------------------------------------------------------------
// :: Command line / config parsing helpers

template <typename T>
static std::istream &readUInt(std::istream &in, T &val) {
  unsigned int tmp;
  in >> tmp;
  val = T(tmp);
  return in;
}

namespace pcc{
  static std::istream &operator>>(std::istream &in, ColorTransform &val) { return readUInt(in, val); }
}

//---------------------------------------------------------------------------
// :: Command line / config parsing

bool parseParameters(int argc, char *argv[], PCCEncoderParameters& params ) {

  namespace po = df::program_options_lite;

  bool print_help = false;

  // The definition of the program/config options, along with default values.
  //
  // NB: when updating the following tables:
  //      (a) please keep to 80-columns for easier reading at a glance,
  //      (b) do not vertically align values -- it breaks quickly
  //
  po::Options opts;
  opts.addOptions()
    ("help", print_help, false, "This help text")
    ("c,config", po::parseConfigFile,"Configuration file name")

    ("configurationFolder",
     params.configurationFolder_,
     params.configurationFolder_,
     "Folder where the configuration files are stored, use for cfg relative paths.")

    ("uncompressedDataFolder",
     params.uncompressedDataFolder_,
     params.uncompressedDataFolder_,
     "Folder where the uncompress input data are stored, use for cfg relative paths.")

    // i/o
    ("uncompressedDataPath",
     params.uncompressedDataPath_,
     params.uncompressedDataPath_,
     "Input pointcloud to encode. Multi-frame sequences may be represented by %04i")

    ("compressedStreamPath",
     params.compressedStreamPath_,
     params.compressedStreamPath_,
     "Output(encoder)/Input(decoder) compressed bitstream")

    ("reconstructedDataPath",
     params.reconstructedDataPath_,
     params.reconstructedDataPath_,
     "Output decoded pointcloud. Multi-frame sequences may be represented by %04i")

    // sequence configuration
    ("startFrameNumber",
     params.startFrameNumber_,
     params.startFrameNumber_,
     "First frame number in sequence to encode/decode")

    ("frameCount",
     params.frameCount_,
     params.frameCount_,
     "Number of frames to encode")

    ("groupOfFramesSize",
     params.groupOfFramesSize_,
     params.groupOfFramesSize_,
     "Random access period")

    // colour space conversion
    ("colorTransform",
     params.colorTransform_,
     params.colorTransform_,
     "The colour transform to be applied:\n  0: none\n  1: RGB to YCbCr (Rec.709)")

    ("colorSpaceConversionPath",
     params.colorSpaceConversionPath_,
     params.colorSpaceConversionPath_,
     "Path to the HDRConvert. If unset, an internal color space conversion is used")

    ("colorSpaceConversionConfig",
     params.colorSpaceConversionConfig_,
     params.colorSpaceConversionConfig_,
     "HDRConvert configuration file used for RGB444 to YUV420 conversion")

    ("inverseColorSpaceConversionConfig",
     params.inverseColorSpaceConversionConfig_,
     params.inverseColorSpaceConversionConfig_,
     "HDRConvert configuration file used for YUV420 to RGB444 conversion")

    // segmentation
    ("nnNormalEstimation",
     params.nnNormalEstimation_,
     params.nnNormalEstimation_,
     "Number of points used for normal estimation")

    ("maxNNCountRefineSegmentation",
     params.maxNNCountRefineSegmentation_,
     params.maxNNCountRefineSegmentation_,
     "Number of nearest neighbors used during segmentation refinement")

    ("iterationCountRefineSegmentation",
     params.iterationCountRefineSegmentation_,
     params.iterationCountRefineSegmentation_,
     "Number of iterations performed during segmentation refinement")

    ("occupancyResolution",
     params.occupancyResolution_,
     params.occupancyResolution_,
     "Resolution T of the occupancy map")

    ("minPointCountPerCCPatchSegmentation",
     params.minPointCountPerCCPatchSegmentation_,
     params.minPointCountPerCCPatchSegmentation_,
     "Minimum number of points for a connected component to be retained as a patch")

    ("maxNNCountPatchSegmentation",
     params.maxNNCountPatchSegmentation_,
     params.maxNNCountPatchSegmentation_,
     "Number of nearest neighbors used during connected components extraction")

    ("surfaceThickness",
     params.surfaceThickness_,
     params.surfaceThickness_,
     "Surface thickness Δ")

    ("maxAllowedDepth",
     params.maxAllowedDepth_,
     params.maxAllowedDepth_,
     "Maximum depth per patch")

    ("maxAllowedDist2MissedPointsDetection",
     params.maxAllowedDist2MissedPointsDetection_,
     params.maxAllowedDist2MissedPointsDetection_,
     "Maximum distance for a point to be ignored during missed point detection")

    ("maxAllowedDist2MissedPointsSelection",
     params.maxAllowedDist2MissedPointsSelection_,
     params.maxAllowedDist2MissedPointsSelection_,
     "Maximum distance for a point to be ignored during  missed points  selection")

    ("lambdaRefineSegmentation",
     params.lambdaRefineSegmentation_,
     params.lambdaRefineSegmentation_,
     "Controls the smoothness of the patch boundaries  during segmentation  refinement")

    // packing
    ("minimumImageWidth",
     params.minimumImageWidth_,
     params.minimumImageWidth_,
     "Minimum width of packed patch frame")

    ("minimumImageHeight",
     params.minimumImageHeight_,
     params.minimumImageHeight_,
     "Minimum height of packed patch frame")

    // occupancy map
    ("maxCandidateCount",
     params.maxCandidateCount_,
     params.maxCandidateCount_,
     "Maximum nuber of candidates in list L")

    ("occupancyPrecision",
    params.occupancyPrecision_,
    params.occupancyPrecision_,
    "Occupancy map B0 precision")

    ("occupancyMapVideoEncoderConfig",
     params.occupancyMapVideoEncoderConfig_,
     params.occupancyMapVideoEncoderConfig_,
     "Occupancy map encoder config file")
    
    ("occupancyMapQP",
     params.occupancyMapQP_,
     params.occupancyMapQP_,
     "QP for compression of occupancy map video")

    ("useOccupancyMapVideo",
     params.useOccupancyMapVideo_,
     params.useOccupancyMapVideo_,
     "compress occupancy map with video codec")

    // smoothing
    // NB: various parameters are of the form n * occupancyPrecision**2
    ("neighborCountSmoothing",
     params.neighborCountSmoothing_,
     params.neighborCountSmoothing_,
     "todo(kmammou)")

    ("radius2Smoothing",
     params.radius2Smoothing_,
     params.radius2Smoothing_,
     "todo(kmammou)")

    ("radius2BoundaryDetection",
     params.radius2BoundaryDetection_,
     params.radius2BoundaryDetection_,
     "todo(kmammou)")

    ("thresholdSmoothing",
     params.thresholdSmoothing_,
     params.thresholdSmoothing_,
     "todo(kmammou)")

	// color smoothing
    ("thresholdColorSmoothing",
     params.thresholdColorSmoothing_,
     params.thresholdColorSmoothing_,
     "Threshold of color smoothing")

    ("thresholdLocalEntropy",
     params.thresholdLocalEntropy_,
     params.thresholdLocalEntropy_,
     "Threshold of local entropy")

    ("radius2ColorSmoothing",
     params.radius2ColorSmoothing_,
     params.radius2ColorSmoothing_,
     "Redius of color smoothing")

    ("neighborCountColorSmoothing",
     params.neighborCountColorSmoothing_,
     params.neighborCountColorSmoothing_,
     "Neighbor count for color smoothing")

    ("flagColorSmoothing",
     params.flagColorSmoothing_,
     params.flagColorSmoothing_,
     "Enable color smoothing\n")

    // colouring
    ("bestColorSearchRange",
     params.bestColorSearchRange_,
     params.bestColorSearchRange_,
     "todo(kmammou)")

    // video encoding
    ("videoEncoderPath",
     params.videoEncoderPath_,
     params.videoEncoderPath_,
     "HM video encoder executable")

    ("videoEncoderOccupancyMapPath",
     params.videoEncoderOccupancyMapPath_,
     params.videoEncoderOccupancyMapPath_,
     "HM lossless video encoder executable for occupancy map")

    ("geometryQP",
     params.geometryQP_,
     params.geometryQP_,
     "QP for compression of geometry video")

    ("textureQP",
     params.textureQP_,
     params.textureQP_,
     "QP for compression of texture video")

    ("geometryConfig",
     params.geometryConfig_,
     params.geometryConfig_,
     "HM configuration file for geometry compression")

    ("geometryD0Config",
     params.geometryD0Config_,
     params.geometryD0Config_,
     "HM configuration file for geometry D0 compression")

    ("geometryD1Config",
      params.geometryD1Config_,
      params.geometryD1Config_,
      "HM configuration file for geometry D1 compression")

    ("textureConfig",
     params.textureConfig_,
     params.textureConfig_,
     "HM configuration file for texture compression")

    // lossless parameters
    ("losslessGeo",
     params.losslessGeo_,
     params.losslessGeo_,
     "Enable lossless encoding of geometry\n")

    ("losslessTexture",
     params.losslessTexture_,
     params.losslessTexture_,
     "Enable lossless encoding of texture\n")

    ("noAttributes",
     params.noAttributes_,
     params.noAttributes_,
     "Disable encoding of attributes")

    ("losslessGeo444",
     params.losslessGeo444_,
     params.losslessGeo444_,
     "Use 4444 format for lossless geometry")

  ("useMissedPointsSeparateVideo",
   params.useMissedPointsSeparateVideo_,
   params.useMissedPointsSeparateVideo_,
   "compress missed point with video codec")
  
  ("geometryMPConfig",
   params.geometryMPConfig_,
   params.geometryMPConfig_,
   "HM configuration file for missed points geometry compression")
  
  ("textureMPConfig",
   params.textureMPConfig_,
   params.textureMPConfig_,
   "HM configuration file for missed points texture compression")
  
//etc
    ("nbThread",
     params.nbThread_,
     params.nbThread_,
     "Number of thread used for parallel processing")

    ("keepIntermediateFiles",
     params.keepIntermediateFiles_,
     params.keepIntermediateFiles_,
     "Keep intermediate files: RGB, YUV and bin")

    ("absoluteD1",
     params.absoluteD1_,
     params.absoluteD1_,
     "Absolute D1")

    ("constrainedPack",
     params.constrainedPack_,
     params.constrainedPack_,
     "Temporally consistent patch packing")

    ("binArithCoding",
     params.binArithCoding_,
     params.binArithCoding_,
     "Binary arithmetic coding")

    // patch sampling resolution
    ("testLevelOfDetail",
     params.testLevelOfDetail_,
     (size_t)0,
     "Force non-zero level of detail for testing")
     ("testLevelOfDetailSignaling",
     params.testLevelOfDetailSignaling_,
     (size_t)0,
     "Test the patch resolution signaling with pseudo-random values")

    ("groupDilation",
      params.groupDilation_,
      params.groupDilation_,
      "Group Dilation")     

    ("textureDilationOffLossless",
      params.textureDilationOffLossless_,
      params.textureDilationOffLossless_,
      "Group Dilation")

    //EDD code
    ("enhancedDeltaDepthCode",
      params.enhancedDeltaDepthCode_,
      params.enhancedDeltaDepthCode_,
      "Use enhanced-delta-depth code")
	  
	//visual quality
    ("patchColorSubsampling", 
     params.patchColorSubsampling_, 
     false, 
     "Enable per patch color sub-sampling\n")
#if FIX_RC011
  ("deltaCoding",
#else
    ("deltaCoding_",
#endif
     params.deltaCoding_,
     params.deltaCoding_,
     "Delta meta-data coding")

      ("projectionMode",
        params.projectionMode_,
        params.projectionMode_,
        "projectionMode - 0:min; 1:max;  2:adaptive frame & patch; 3:adaptive patch (all frames)");
     ;

  po::setDefaults(opts);
  po::ErrorReporter err;
  const list<const char *> &argv_unhandled = po::scanArgv(opts, argc, (const char **)argv, err);

  for (const auto arg : argv_unhandled) {
    err.warn() << "Unhandled argument ignored: " << arg << "\n";
  }

   if(!params.losslessGeo_ && !params.losslessTexture_)
   {
     params.useOccupancyMapVideo_=true;
   }
   
   if( params.videoEncoderOccupancyMapPath_.empty() || !exist( params.videoEncoderOccupancyMapPath_)    ) {
     
     params.videoEncoderOccupancyMapPath_ = params.videoEncoderPath_;
   }
   if(params.useMissedPointsSeparateVideo_)
   {
     if(params.geometryMPConfig_.empty() || !exist( params.geometryMPConfig_) )
     {
       params.geometryMPConfig_=params.geometryConfig_;
     }
     
     if(params.textureMPConfig_.empty() || !exist( params.textureMPConfig_) )
     {
       params.textureMPConfig_=params.textureConfig_;
     }
   }
   
  if (argc == 1 || print_help) {
    po::doHelp( std::cout, opts, 78 );
    return false;
  }
  params.completePath();
  params.print();
  if( !params.check() ) {
    err.error() << "Input parameters not correct \n";
  }

  // report the current configuration (only in the absence of errors so
  // that errors/warnings are more obvious and in the same place).
  if (err.is_errored) return false;

  return true;
}

int compressVideo( const PCCEncoderParameters& params, StopwatchUserTime &clock) {
  const size_t startFrameNumber0 = params.startFrameNumber_;
  const size_t endFrameNumber0 = params.startFrameNumber_ + params.frameCount_;
  const size_t groupOfFramesSize0 = (std::max)(size_t(1), params.groupOfFramesSize_);
  size_t startFrameNumber = startFrameNumber0;
  size_t reconstructedFrameNumber = params.startFrameNumber_;
  PCCBitstream bitstream;
  std::unique_ptr<uint8_t> buffer;
  PCCBistreamPosition totalSizeIterator = bitstream.getPosition();
  size_t contextIndex = 0;
  PCCEncoder encoder;
  encoder.setParameters( params );

  // Place to get/set default values for gof metadata enabled flags (in sequence level).
  PCCMetadataEnabledFlags gofLevelMetadataEnabledFlags;
  while (startFrameNumber < endFrameNumber0) {
    const size_t endFrameNumber = min(startFrameNumber + groupOfFramesSize0, endFrameNumber0);
    PCCContext context;
    context.setIndex( contextIndex );

    context.setLosslessGeo444(params.losslessGeo444_);
    context.setLossless(params.losslessGeo_);
    context.setLosslessAtt(params.losslessTexture_);
    context.setMPGeoWidth(64);
    context.setMPAttWidth(64);
    context.setMPGeoHeight(0);
    context.setMPAttHeight(0);
    context.setEnhancedDeltaDepth(params.enhancedDeltaDepthCode_);
    context.setUseMissedPointsSeparateVideo(params.useMissedPointsSeparateVideo_);
    context.setUseOccupancyMapVideo(params.useOccupancyMapVideo_);
    
    if (gofLevelMetadataEnabledFlags.getMetadataEnabled()) {
      // Place to get/set gof-level metadata.
      PCCMetadata gofLevelMetadata;
      context.getGOFLevelMetadata() = gofLevelMetadata;
      context.getGOFLevelMetadata().getMetadataEnabledFlags() = gofLevelMetadataEnabledFlags;
      // Place to get/set frame metadata enabled flags (in gof level).
      PCCMetadataEnabledFlags frameLevelMetadataEnabledFlags;
      context.getGOFLevelMetadata().getLowerLevelMetadataEnabledFlags() = frameLevelMetadataEnabledFlags;
    }
    PCCGroupOfFrames sources, reconstructs;
    if (!sources.load( params.uncompressedDataPath_, startFrameNumber,
                       endFrameNumber, params.colorTransform_ ) ) {
      return -1;
    }
    clock.start();

    if (startFrameNumber == startFrameNumber0) {
      const size_t predictedBitstreamSize =
          10000 + 8 * params.frameCount_ * sources[0].getPointCount();
      bitstream.initialize( predictedBitstreamSize );
      bitstream.writeHeader(gofLevelMetadataEnabledFlags);
    }

    if (params.testLevelOfDetail_ > 0) {
      const double lodScale = 1.0 / double(1u << params.testLevelOfDetail_);
      for (auto& frame : sources.getFrames()) {
        for (auto& point : frame.getPositions()) {
          point *= lodScale;
        }
      }
    }

    std::cout << "Compressing group of frames " << contextIndex << ": " << startFrameNumber
              << " -> " << endFrameNumber << "..." << std::endl;
    int ret = encoder.encode( sources, context, bitstream, reconstructs );

    clock.stop();

    if (ret) {
      return ret;
    }
    if( !params.reconstructedDataPath_.empty() ) {
      reconstructs.write( params.reconstructedDataPath_, reconstructedFrameNumber );
    }
    sources.clear();
    reconstructs.clear();
    startFrameNumber = endFrameNumber;
    contextIndex++;
  }

  std::cout << "Total bitstream size " << bitstream.size() << " B" << std::endl;
  bitstream.write( params.compressedStreamPath_ );
  return 0;
}

