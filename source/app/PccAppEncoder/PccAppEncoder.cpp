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
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#include "PCCCommon.h"
#include "PCCChrono.h"
#include "PCCMemory.h"
#include "PCCEncoder.h"
#include "PCCMetrics.h"
#include "PCCChecksum.h"
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCBitstream.h"
#include "PCCGroupOfFrames.h"
#include "PCCEncoderParameters.h"
#include "PCCBitstreamWriter.h"
#include "PCCMetricsParameters.h"
#include <program_options_lite.h>
#include <tbb/tbb.h>

using namespace std;
using namespace pcc;
using pcc::chrono::StopwatchUserTime;

//---------------------------------------------------------------------------
// :: Command line / config parsing helpers

namespace pcc {
static std::istream& operator>>( std::istream& in, PCCColorTransform& val ) {
  unsigned int tmp;
  in >> tmp;
  val = PCCColorTransform( tmp );
  return in;
}
static std::istream& operator>>( std::istream& in, PCCCodecId& val ) {
  val = UNKNOWN_CODEC;
  std::string tmp;
  in >> tmp;
#ifdef USE_JMAPP_VIDEO_CODEC
  if ( tmp == "JMAPP" || tmp == "jmapp" ) { val = JMAPP; }
#endif
#ifdef USE_HMAPP_VIDEO_CODEC
  if ( tmp == "HMAPP" || tmp == "hmapp" ) { val = HMAPP; }
#endif
#ifdef USE_SHMAPP_VIDEO_CODEC
  if ( tmp == "SHMAPP" || tmp == "shmapp" || tmp == "shm" || tmp == "shm2" || tmp == "shm3" ) { val = SHMAPP; }
#endif
#ifdef USE_JMLIB_VIDEO_CODEC
  if ( tmp == "JMLIB" || tmp == "jmlib" || tmp == "jm" || tmp == "avc" ) { val = JMLIB; }
#endif
#ifdef USE_HMLIB_VIDEO_CODEC
  if ( tmp == "HMLIB" || tmp == "hmlib" || tmp == "hm" || tmp == "hevc" ) { val = HMLIB; }
#endif
#ifdef USE_VTMLIB_VIDEO_CODEC
  if ( tmp == "VTMLIB" || tmp == "vtmlib" || tmp == "vtm" || tmp == "vvc" ) { val = VTMLIB; }
#endif
#ifdef USE_FFMPEG_VIDEO_CODEC
  if ( tmp == "FFMPEG" || tmp == "ffmpeg" ) { val = FFMPEG; }
#endif
  if ( val == UNKNOWN_CODEC ) { fprintf( stderr, "PCCCodecId could not be indentified from: \"%s\" \n", tmp.c_str() ); }
  return in;
}

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
    ( "forcedSsvhUnitSizePrecisionBytes",
      encoderParams.forcedSsvhUnitSizePrecisionBytes_,
      encoderParams.forcedSsvhUnitSizePrecisionBytes_,
      "forced SSVH unit size precision bytes" )

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
    ( "gridBasedSegmentation",    // If enabled, change refineSegmentationGridBased() parameters according to m56857"
      encoderParams.gridBasedSegmentation_,
      encoderParams.gridBasedSegmentation_,
      "Voxel dimension for grid-based segmentation (GBS)" )
    ( "voxelDimensionGridBasedSegmentation",
      encoderParams.voxelDimensionGridBasedSegmentation_,
      encoderParams.voxelDimensionGridBasedSegmentation_,
      "Voxel dimension for grid-based segmentation (GBS)" )
    ( "nnNormalEstimation",
      encoderParams.nnNormalEstimation_,
      encoderParams.nnNormalEstimation_,
      "Number of points used for normal estimation" )
    ( "normalOrientation",
      encoderParams.normalOrientation_,
      encoderParams.normalOrientation_,
      "Normal orientation: 0: None 1: spanning tree, 2:view point, 3:cubemap projection" )     
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
    ( "occupancyMapConfig",
      encoderParams.occupancyMapConfig_,
      encoderParams.occupancyMapConfig_,
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

    // hash
    ( "decodedAtlasInformationHash",
      encoderParams.decodedAtlasInformationHash_,
      encoderParams.decodedAtlasInformationHash_,
      "Enable decoded atlas information hash 0. disable 1.MD5 2.CRC 3.Checksum\n" )

    // smoothing
    ( "attributeTransferFilterType",
      encoderParams.attrTransferFilterType_,
      encoderParams.attrTransferFilterType_,
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
    ( "videoEncoderOccupancyPath",
      encoderParams.videoEncoderOccupancyPath_,
      encoderParams.videoEncoderOccupancyPath_,
      "Occupancy video encoder executable path" )
    ( "videoEncoderGeometryPath",
      encoderParams.videoEncoderGeometryPath_,
      encoderParams.videoEncoderGeometryPath_, 
      "Geometry video encoder executable path" )
    ( "videoEncoderAttributePath",
      encoderParams.videoEncoderAttributePath_,
      encoderParams.videoEncoderAttributePath_, 
      "Attribute video encoder executable path" )
    ( "videoEncoderOccupancyCodecId",
      encoderParams.videoEncoderOccupancyCodecId_,
      encoderParams.videoEncoderOccupancyCodecId_,
      "Occupancy video encoder codec id" )
    ( "videoEncoderGeometryCodecId",
      encoderParams.videoEncoderGeometryCodecId_,
      encoderParams.videoEncoderGeometryCodecId_, 
      "Geometry video encoder codec id" )
    ( "videoEncoderAttributeCodecId",
      encoderParams.videoEncoderAttributeCodecId_,
      encoderParams.videoEncoderAttributeCodecId_, 
      "Attribute video encoder codec id" )
  ( "byteStreamVideoEncoderOccupancy",
    encoderParams.byteStreamVideoCoderOccupancy_,
    encoderParams.byteStreamVideoCoderOccupancy_,
    "Attribute video encoder outputs byteStream" )
  ( "byteStreamVideoEncoderGeometry",
    encoderParams.byteStreamVideoCoderGeometry_,
    encoderParams.byteStreamVideoCoderGeometry_,
    "Attribute video encoder outputs byteStream" )
  ( "byteStreamVideoEncoderAttribute",
    encoderParams.byteStreamVideoCoderAttribute_,
    encoderParams.byteStreamVideoCoderAttribute_,
    "Attribute video encoder outputs byteStream" )

    ( "geometryQP",
      encoderParams.geometryQP_,
      encoderParams.geometryQP_,
      "QP for compression of geometry video" )
    ( "attributeQP",
      encoderParams.attributeQP_,
      encoderParams.attributeQP_,
      "QP for compression of attribute video" )
    ( "auxGeometryQP",
      encoderParams.auxGeometryQP_,
      encoderParams.auxGeometryQP_,
      "QP for compression of auxiliary geometry video : default=4 for lossy raw points, geometryQP for lossless raw points" )
    ( "auxAttributeQP",
      encoderParams.auxAttributeQP_,
      encoderParams.auxAttributeQP_,
      "QP for compression of auxiliary attribute video" )
    ( "geometryConfig",
      encoderParams.geometryConfig_,
      encoderParams.geometryConfig_,
      "HM configuration file for geometry compression" )
    ( "geometry0Config",
      encoderParams.geometry0Config_,
      encoderParams.geometry0Config_,
      "HM configuration file for geometry 0 compression" )
    ( "geometry1Config",
      encoderParams.geometry1Config_,
      encoderParams.geometry1Config_,
      "HM configuration file for geometry 1 compression" )
    ( "attributeConfig",
      encoderParams.attributeConfig_,
      encoderParams.attributeConfig_,
      "HM configuration file for attribute compression" )
    ( "attribute0Config",
      encoderParams.attribute0Config_,
      encoderParams.attribute0Config_,
      "HM configuration file for attribute 0 compression" )
    ( "attribute1Config",
      encoderParams.attribute1Config_,
      encoderParams.attribute1Config_,
      "HM configuration file for attribute 1 compression" )
    ( "rawPointsPatch",
      encoderParams.rawPointsPatch_,
      encoderParams.rawPointsPatch_,
      "Enable raw points patch\n" )    
    ( "noAttributes",
      encoderParams.noAttributes_,
      encoderParams.noAttributes_, 
      "Disable encoding of attributes" )
    ( "attributeVideo444",
      encoderParams.attributeVideo444_,
      encoderParams.attributeVideo444_,
      "Use 444 format for attribute video" )
    ( "useRawPointsSeparateVideo",
      encoderParams.useRawPointsSeparateVideo_,
      encoderParams.useRawPointsSeparateVideo_,
      "compress raw points with video codec" )
    ( "attributeRawSeparateVideoWidth",
      encoderParams.attributeRawSeparateVideoWidth_,
      encoderParams.attributeRawSeparateVideoWidth_,
      "Width of the MP's attribute in separate video" )
    ( "geometryMPConfig",
      encoderParams.geometryAuxVideoConfig_,
      encoderParams.geometryAuxVideoConfig_,
      "HM configuration file for raw points geometry compression" )
    ( "attributeMPConfig",
      encoderParams.attributeAuxVideoConfig_,
      encoderParams.attributeAuxVideoConfig_,
      "HM configuration file for raw points attribute compression" )

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
    ( "deltaQPD0",
      encoderParams.deltaQPD0_,
      encoderParams.deltaQPD0_, 
      "qp adjustment for geometry0 video: 0, +3, -3..." )
    ( "deltaQPD1",
      encoderParams.deltaQPD1_,
      encoderParams.deltaQPD1_,
      "qp adjustment for geometry1 video: 0, +3, -3..." )
    ( "deltaQPT0",
      encoderParams.deltaQPT0_,
      encoderParams.deltaQPT0_, 
      "qp adjustment for attribute0 video: 0, +3, -3..." )
    ( "deltaQPT1",
      encoderParams.deltaQPT1_,
      encoderParams.deltaQPT1_, 
      "qp adjustment for attribute1 video: 0, +3, -3..." )
      
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

    // SHVC 
    ( "shvcLayerIndex",
      encoderParams.shvcLayerIndex_,
      encoderParams.shvcLayerIndex_,
     "Decode Layer ID number using SHVC codec")
    ( "shvcRateX",
      encoderParams.shvcRateX_,
      encoderParams.shvcRateX_,
      "SHVCRateX : reduce rate of each SHVC layer X axis in 2D space (should be greater than 1)")
    ( "shvcRateY",
      encoderParams.shvcRateY_,
      encoderParams.shvcRateY_,
      "SHVCRateY : reduce rate of each SHVC layer Y axis in 2D space (should be greater than 1)")

    // visual quality
    ( "patchColorSubsampling",
      encoderParams.patchColorSubsampling_, false,
      "Enable per patch color sub-sampling\n" )
    ( "maxNumRefAtalsList",
      encoderParams.maxNumRefAtlasList_,
      encoderParams.maxNumRefAtlasList_,
      "maximum Number of Reference Atlas Frame list, default: 1" )
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
    ( "singleMapPixelInterleaving",
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
      "Patches packing strategy(0: anchor packing, 1(default): flexible packing, 2: tetris packing)" )
    ( "useEightOrientations",
      encoderParams.useEightOrientations_,
      encoderParams.useEightOrientations_,
      "Allow either 2 orientations (0(default): NULL AND SWAP), or 8 orientation (1)\n" )
    ( "safeGuardDistance",
      encoderParams.safeGuardDistance_,
      encoderParams.safeGuardDistance_,
      "Number of empty blocks that must exist between the patches (default=1)\n" )
    ( "attributeBGFill",
      encoderParams.attributeBGFill_,
      encoderParams.attributeBGFill_,
      "Selects the background filling operation for attribute only (0: patch-edge extension, "
      "1(default): smoothed push-pull algorithm), 2: harmonic background filling " )

    // lossy-raw-points patch
    ( "lossyRawPointsPatch",
      encoderParams.lossyRawPointsPatch_,
      encoderParams.lossyRawPointsPatch_,
      "Lossy raw points patch(0: no lossy raw points patch, 1: enable lossy raw points patch (default=0)" )
    ( "minNormSumOfInvDist4MPSelection",
      encoderParams.minNormSumOfInvDist4MPSelection_,
      encoderParams.minNormSumOfInvDist4MPSelection_,
      "Minimum normalized sum of inverse distance for raw points selection: double value between 0.0 and 1.0 (default=0.35)" )
    ( "globalPatchAllocation",
      encoderParams.globalPatchAllocation_,
      encoderParams.globalPatchAllocation_,
      "Global temporally consistent patch allocation.(0: anchor's packing method(default), 1: gpa algorithm, 2: gtp algorithm)" )
    ( "globalPackingStrategyGOF",
      encoderParams.globalPackingStrategyGOF_,
      encoderParams.globalPackingStrategyGOF_,
      "Number of frames to pack globally (0:(entire GOF))" )
    ( "globalPackingStrategyReset",
      encoderParams.globalPackingStrategyReset_,
      encoderParams.globalPackingStrategyReset_,
      "Remove the reference to the previous frame (0(default), 1)" )
    ( "globalPackingStrategyThreshold",
      encoderParams.globalPackingStrategyThreshold_,
      encoderParams.globalPackingStrategyThreshold_,
      "matched patches area ratio threshold (decides if connections are valid or not, 0(default))" )
    ( "patchPrecedenceOrder",
      encoderParams.patchPrecedenceOrderFlag_,
      encoderParams.patchPrecedenceOrderFlag_, 
      "Order of patches" )
    ( "lowDelayEncoding",
      encoderParams.lowDelayEncoding_,
      encoderParams.lowDelayEncoding_,
      "Low Delay encoding (0(default): do nothing, 1: does not allow overlap of patches bounding boxes for low delay encoding)" )
    ( "geometryPadding",
      encoderParams.geometryPadding_,
      encoderParams.geometryPadding_,
      "Selects the background filling operation for geometry (0: anchor, 1(default): 3D geometry padding)" )
    ( "apply3dMotionCompensation",
      encoderParams.use3dmc_,
      encoderParams.use3dmc_,
      "Use auxilliary information for 3d motion compensation.(0: conventional video coding, 1: 3D motion compensated)" )
    ( "usePccRDO",
      encoderParams.usePccRDO_,
      encoderParams.usePccRDO_,
      "Use HEVC PCC RDO optimization" )
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
      "Use enhanced Projection Plane(0: OFF, 1: ON)" )
    ( "minWeightEPP",
      encoderParams.minWeightEPP_,
      encoderParams.minWeightEPP_, 
      "Minimum value" )
    ( "additionalProjectionPlaneMode",
      encoderParams.additionalProjectionPlaneMode_,
      encoderParams.additionalProjectionPlaneMode_,
      "additiona Projection Plane Mode 0:none 1:Y-Axis 2:X-Axis 3:Z-Axis 4:All-Axis 5:apply to portion " )
    ( "partialAdditionalProjectionPlane",
      encoderParams.partialAdditionalProjectionPlane_,
      encoderParams.partialAdditionalProjectionPlane_,
      "The value determines the partial point cloud. It's available with only additionalProjectionPlaneMode(5)" )
  ("numMaxTilePerFrame",
   encoderParams.numMaxTilePerFrame_,
   encoderParams.numMaxTilePerFrame_,"number of maximum tiles in a frame")
  ("uniformPartitionSpacing",
   encoderParams.uniformPartitionSpacing_,
   encoderParams.uniformPartitionSpacing_, "indictation of uniform partitioning")
  ("tilePartitionWidth",
   encoderParams.tilePartitionWidth_,
   encoderParams.tilePartitionWidth_,"uniform partition width in the unit of 64 pixels")
  ("tilePartitionHeight",
   encoderParams.tilePartitionHeight_,
   encoderParams.tilePartitionHeight_,"uniform partition height in the unit of 64 pixels")
  ("tilePartitionWidthList",
   encoderParams.tilePartitionWidthList_,
   encoderParams.tilePartitionWidthList_,"non uniform partition width in the unit of 64 pixels")
  ("tilePartitionHeightList",
   encoderParams.tilePartitionHeightList_,
   encoderParams.tilePartitionHeightList_,"non uniform partition height in the unit of 64 pixels")
  ( "tileSegmentationType",
      encoderParams.tileSegmentationType_,
      encoderParams.tileSegmentationType_,
      "tile segmentaton method : 0.no tile partition 1. 3D ROI based 2.2D Patch size based " )
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
      " enable patch block filtering " )
    ( "pbfFilterSize",
      encoderParams.pbfFilterSize_,
      encoderParams.pbfFilterSize_, 
      "pbfFilterSize " )
    ( "pbfPassesCount",
      encoderParams.pbfPassesCount_,
      encoderParams.pbfPassesCount_, 
      "pbfPassesCount " )
    ( "pbfLog2Threshold",
      encoderParams.pbfLog2Threshold_,
      encoderParams.pbfLog2Threshold_, 
      "pbfLog2Threshold " );

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

    //8.3.4.2	Profile, tier and level syntax
    opts.addOptions()
    ( "tierFlag", 
      encoderParams.tierFlag_,
      encoderParams.tierFlag_, 
      "Tier Flag" )
    ( "profileCodecGroupIdc", 
      encoderParams.profileCodecGroupIdc_,
      encoderParams.profileCodecGroupIdc_, 
      "Profile Codec Group Idc" )
    ( "profileToolsetIdc", 
      encoderParams.profileToolsetIdc_,
      encoderParams.profileToolsetIdc_, 
    "Profile Toolset Idc" )
    ( "profileReconstructionIdc", 
      encoderParams.profileReconstructionIdc_,
      encoderParams.profileReconstructionIdc_, 
      "Profile Reconstruction Idc" )
    ( "levelIdc", 
      encoderParams.levelIdc_,
      encoderParams.levelIdc_, 
      "Level Idc" )    
    ( "avcCodecIdIndex",
      encoderParams.avcCodecIdIndex_,
      encoderParams.avcCodecIdIndex_, 
      "index for avc codec " ) //ajt0526: avcCodecIdIndex/hevcCodecIdIndex/vvcCodecIdIndex when using CCM SEI and profileCodecGroupIdc beeds to correspond to mp4RA?
    ( "hevcCodecIdIndex",
      encoderParams.hevcCodecIdIndex_,
      encoderParams.hevcCodecIdIndex_, 
      "index for hevc codec " )
    ( "shvcCodecIdIndex",
      encoderParams.shvcCodecIdIndex_,
      encoderParams.shvcCodecIdIndex_, 
      "index for shvc codec " )
    ( "vvcCodecIdIndex",
      encoderParams.vvcCodecIdIndex_,
      encoderParams.vvcCodecIdIndex_, 
      "index for vvc codec " );

    //8.3.4.6	Profile toolset constraints information syntax
    opts.addOptions()
    ( "oneV3CFrameOnlyFlag", 
      encoderParams.oneV3CFrameOnlyFlag_,
      encoderParams.oneV3CFrameOnlyFlag_, 
     "One V3C Frame Only Flag" );

  // clang-format on
  po::setDefaults( opts );
  po::ErrorReporter        err;
  const list<const char*>& argv_unhandled = po::scanArgv( opts, argc, (const char**)argv, err );
  for ( const auto arg : argv_unhandled ) { printf( "Unhandled argument ignored: %s \n", arg ); }

  if ( argc == 1 || print_help ) {
    po::doHelp( std::cout, opts, 78 );
    return false;
  }
  encoderParams.completePath();
  metricsParams.completePath();
  if ( !encoderParams.check() ) {
    std::cerr << "Input encoder parameters not correct \n";
    err.is_errored = true;
  }
  if ( !metricsParams.check() ) { std::cerr << "Input metrics parameters not correct \n"; }
  encoderParams.print();
  metricsParams.print();
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
  size_t       endFrameNumber0          = encoderParams.startFrameNumber_ + encoderParams.frameCount_;
  const size_t groupOfFramesSize0       = ( std::max )( size_t( 1 ), encoderParams.groupOfFramesSize_ );
  size_t       startFrameNumber         = startFrameNumber0;
  size_t       reconstructedFrameNumber = encoderParams.startFrameNumber_;

  PCCLogger logger;
  logger.initilalize( removeFileExtension( encoderParams.compressedStreamPath_ ), true );
  std::unique_ptr<uint8_t> buffer;
  size_t                   contextIndex = 0;
  PCCEncoder               encoder;
  PCCMetrics               metrics;
  PCCChecksum              checksum;
  PCCBitstreamStat         bitstreamStat;
  SampleStreamV3CUnit      ssvu;
  encoder.setLogger( logger );
  encoder.setParameters( encoderParams );
  metrics.setParameters( metricsParams );
  checksum.setParameters( metricsParams );

  // Place to get/set default values for gof metadata enabled flags (in sequence level).
  while ( startFrameNumber < endFrameNumber0 ) {
    size_t     endFrameNumber = min( startFrameNumber + groupOfFramesSize0, endFrameNumber0 );
    PCCContext context;
    context.setBitstreamStat( bitstreamStat );
    context.addV3CParameterSet( contextIndex );
    context.setActiveVpsId( contextIndex );
    PCCGroupOfFrames sources;
    PCCGroupOfFrames reconstructs;
    clock.start();
    if ( !sources.load( encoderParams.uncompressedDataPath_, startFrameNumber, endFrameNumber,
                        encoderParams.colorTransform_, false, encoderParams.nbThread_ ) ) {
      return -1;
    }
    if ( sources.getFrameCount() < endFrameNumber - startFrameNumber ) {
      endFrameNumber  = startFrameNumber + sources.getFrameCount();
      endFrameNumber0 = endFrameNumber;
    }
    std::cout << "Compressing " << contextIndex << " frames " << startFrameNumber << " -> " << endFrameNumber << "..."
              << std::endl;
    int                ret = encoder.encode( sources, context, reconstructs );
    PCCBitstreamWriter bitstreamWriter;
#ifdef BITSTREAM_TRACE
    bitstreamWriter.setLogger( logger );
#endif
    ret |= bitstreamWriter.encode( context, ssvu );
    clock.stop();
    PCCGroupOfFrames normals;
    if ( metricsParams.computeMetrics_ ) {
      bool bRunMetric = true;
      if ( !metricsParams.normalDataPath_.empty() ) {
        if ( !normals.load( metricsParams.normalDataPath_, startFrameNumber, endFrameNumber, COLOR_TRANSFORM_NONE,
                            true ) ) {
          bRunMetric = false;
        }
      }
      if ( bRunMetric ) { metrics.compute( sources, reconstructs, normals ); }
    }
    if ( metricsParams.computeChecksum_ ) {
      if ( encoderParams.rawPointsPatch_ && encoderParams.reconstructRawType_ != 0 ) {
        checksum.computeSource( sources );
        checksum.computeReordered( reconstructs );
      }
      checksum.computeReconstructed( reconstructs );
    }
    if ( ret != 0 ) { return ret; }
    if ( !encoderParams.reconstructedDataPath_.empty() ) {
      reconstructs.write( encoderParams.reconstructedDataPath_, reconstructedFrameNumber );
    }
    normals.clear();
    sources.clear();
    reconstructs.clear();
    startFrameNumber = endFrameNumber;
    contextIndex++;
  }

  PCCBitstream bitstream;
#if defined( BITSTREAM_TRACE ) || defined( CONFORMANCE_TRACE )
  bitstream.setLogger( logger );
  bitstream.setTrace( true );
#endif

  bitstreamStat.setHeader( bitstream.size() );
  PCCBitstreamWriter bitstreamWriter;
  size_t headerSize = bitstreamWriter.write( ssvu, bitstream, encoderParams.forcedSsvhUnitSizePrecisionBytes_ );
  bitstreamStat.incrHeader( headerSize );
  bitstream.write( encoderParams.compressedStreamPath_ );
  bitstreamStat.trace();
  std::cout << "Total bitstream size " << bitstream.size() << " B" << std::endl;
  bitstream.computeMD5();

  if ( metricsParams.computeMetrics_ ) { metrics.display(); }
  bool checksumEqual = true;
  if ( metricsParams.computeChecksum_ ) {
    if ( encoderParams.rawPointsPatch_ && encoderParams.reconstructRawType_ != 0 ) {
      checksumEqual = checksum.compareSrcRec();
    }
    checksum.write( encoderParams.compressedStreamPath_ );
  }
  return checksumEqual ? 0 : -1;
}

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
  using ms            = milliseconds;
  auto totalWall      = duration_cast<ms>( clockWall.count() ).count();
  auto totalUserSelf  = duration_cast<ms>( clockUser.self.count() ).count();
  auto totalUserChild = duration_cast<ms>( clockUser.children.count() ).count();
  std::cout << "Processing time (wall): " << ( ret == 0 ? totalWall / 1000.0 : -1 ) << " s\n";
  std::cout << "Processing time (user.self): " << ( ret == 0 ? totalUserSelf / 1000.0 : -1 ) << " s\n";
  std::cout << "Processing time (user.children): " << ( ret == 0 ? totalUserChild / 1000.0 : -1 ) << " s\n";
  std::cout << "Peak memory: " << getPeakMemory() << " KB\n";
  return ret;
}
