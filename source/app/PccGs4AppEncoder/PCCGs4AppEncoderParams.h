/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2026, ISO/IEC
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

#include "PCCGSEncoder.h"
#include "PCCAdaptation.h"
#include <program_options_lite.h>
#include <set>

using namespace std;

//---------------------------------------------------------------------------
// :: Command line / config parsing helpers

namespace pcc {

static std::istream &operator>>( std::istream &in, std::vector<int> &vector ) {
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

struct PCCApplicationParameters {
  std::string configurationFolder_;
  std::string uncompressedDataPath_;
  std::string compressedStreamPath_;
  std::string geometryConfig_;
  std::string attributeConfig_;
  std::string packConfig_;
  size_t      startFrameNumber_ = 0;
  size_t      frameCount_       = 0;
  size_t      nbThread_         = 0;

  void print() const {
    std::cout << "Application parameters: \n";
    std::cout << "  configurationFolder: " << configurationFolder_ << "\n";
    std::cout << "  uncompressedDataPath: " << uncompressedDataPath_ << "\n";
    std::cout << "  compressedStreamPath: " << compressedStreamPath_ << "\n";
    std::cout << "  geometryConfig:       " << geometryConfig_ << "\n";
    std::cout << "  attributeConfig:      " << attributeConfig_ << "\n";
    std::cout << "  packConfig:           " << packConfig_ << "\n";
    std::cout << "  startFrameNumber:     " << startFrameNumber_ << "\n";
    std::cout << "  frameCount:           " << frameCount_ << "\n";
    std::cout << "  nbThread:             " << nbThread_ << "\n";
  }
};

//---------------------------------------------------------------------------
// :: Command line / config parsing

bool parseParameters( int                           argc,
                      char                         *argv[],
                      PCCApplicationParameters     &applicationParams,
                      pcc::PCCGSEncoderParameters  &encoderParams,
                      pcc::PCCAdaptationParameters &adaptationParams ) {
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
      applicationParams.configurationFolder_,
      applicationParams.configurationFolder_,
      "Folder where the configuration files are stored,use for cfg relative paths." )
    ( "uncompressedDataPath",
      applicationParams.uncompressedDataPath_,
      applicationParams.uncompressedDataPath_,
      "Input pointcloud to encode. Multi-frame sequences may be represented by %04i" )
    ( "compressedStreamPath",
      applicationParams.compressedStreamPath_,
      applicationParams.compressedStreamPath_,
      "Output(encoder) compressed bitstream" )
    ( "geometryConfig",
      applicationParams.geometryConfig_,
      applicationParams.geometryConfig_,
      "HM configuration file for geometry video compression" )
    ( "attributeConfig",
      applicationParams.attributeConfig_,
      applicationParams.attributeConfig_,
      "HM configuration file for attribute video compression" )
    ( "packConfig",
      applicationParams.packConfig_,
      applicationParams.packConfig_,
      "HM configuration file for packed video compression" )
    ( "startFrameNumber",
      applicationParams.startFrameNumber_,
      applicationParams.startFrameNumber_,
      "First frame to process in a multi-frame sequence" )
    ( "frameCount",
      applicationParams.frameCount_,
      applicationParams.frameCount_,
      "Number of frames to process from startFrameNumber (0 = all frames)" )
    ( "nbThread",
      applicationParams.nbThread_,
      applicationParams.nbThread_,
      "Number of threads to use (0 = auto)" )
    
    // Adaptation parameters
    ( "enableDuplicateMerging",
      adaptationParams.enableDuplicateMerging_,
      adaptationParams.enableDuplicateMerging_,
      "Enable merging of duplicate positions (default: 0)" )
    ("shMappingMode",
      adaptationParams.shMappingMode_,
      adaptationParams.shMappingMode_,
      "Mapping spherical harmonics AC coefficients to video using planar mode")
    ( "enableAdaptiveShQuantization",
      adaptationParams.enableAdaptiveShQuantization_,
      adaptationParams.enableAdaptiveShQuantization_,
      "Enable adaptive SH quantization" )
     ( "keepZeroShBitGaussians",
      adaptationParams.keepZeroShBitGaussians_,
      adaptationParams.keepZeroShBitGaussians_,
      "Keep Gaussians that have an adaptive bit depth of 0 (no pruning)" )
    ( "adaptiveBitshifts",
      adaptationParams.adaptiveShBitshifts_,
      adaptationParams.adaptiveShBitshifts_,
      "Comma-separated mapping from quantization group ID to SH bitshift" )
    ( "importanceScoresPath",
      adaptationParams.importanceScoresPath_,
      adaptationParams.importanceScoresPath_,
      "Path to text file containing gaussian importance scores" )
    
    // Encoder parameters
    ( "geometryQP",
      encoderParams.geometryQP_,
      encoderParams.geometryQP_,
      "QP for compression of geometry video" )
    ( "attributeScaleQP",
      encoderParams.attributeScaleQP_,
      encoderParams.attributeScaleQP_,
      "QP for compression of scale attribute video" )
    ( "attributeOpacityQP",
      encoderParams.attributeOpacityQP_,
      encoderParams.attributeOpacityQP_,
      "QP for compression of opacity attribute video" )
    ( "attributeRotationQP",
      encoderParams.attributeRotationQP_,
      encoderParams.attributeRotationQP_,
      "QP for compression of rotation attribute video" )
    ( "attributeColorQP",
      encoderParams.attributeColorQP_,
      encoderParams.attributeColorQP_,
      "QP for compression of color/SH attribute video" )
    ( "packQP",
      encoderParams.packQP_,
      encoderParams.packQP_,
      "QP for compression of packed video" )
    ( "occupancyResolution",
      encoderParams.occupancyResolution_,
      encoderParams.occupancyResolution_,
      "Occupancy resolution for raw points patches" )
    ( "geometryBitDepth3D",
      encoderParams.geometryBitDepth3D_,
      encoderParams.geometryBitDepth3D_,
      "Bit depth for 3D geometry coordinates" )
    ( "geometryBitDepth2D",
      encoderParams.geometryBitDepth2D_,
      encoderParams.geometryBitDepth2D_,
      "Bit depth for 2D geometry representation" )
    ( "attributeBitDepth2D",
      encoderParams.attributeBitDepth2D_,
      encoderParams.attributeBitDepth2D_,
      "Bit depth for 2D attribute representation" )
    ( "minimumImageWidth",
      encoderParams.minimumImageWidth_,
      encoderParams.minimumImageWidth_, 
      "Minimum width of packed patch frame" )
    ( "raw3dOffsetBitCountExplicitModeFlag",
      encoderParams.raw3dOffsetBitCountExplicitModeFlag_,
      encoderParams.raw3dOffsetBitCountExplicitModeFlag_,
      "Enable explicit mode for 3D offset bit count" )
    ( "raw3dOffsetAxisBitCountMinus1",
      encoderParams.raw3dOffsetAxisBitCountMinus1_,
      encoderParams.raw3dOffsetAxisBitCountMinus1_,
      "3D offset axis bit count minus 1 (used when explicit mode is enabled)" )
    ( "packGeometry",
      encoderParams.packGeometry_,
      encoderParams.packGeometry_,
      "Pack geometry using the packed video extension (VPS_EXT_PACKED_2)" )
    ( "packAttributes",
      encoderParams.packAttributes_,
      encoderParams.packAttributes_,
      "Pack attributes using the packed video extension (VPS_EXT_PACKED_2)" )
    ("implicitPacking", //m76092
      encoderParams.implicitPacking_,
      encoderParams.implicitPacking_,
      "Enable implicit packing (0(default): explicit mode, 1: implicit mode)")
    ("temporalPacking", //m76086
      encoderParams.temporalPacking_,
      encoderParams.temporalPacking_,
      "Temporal packing (0(default): disabled, 1: enabled)")
    ( "geometryPackedMapIndex",
      encoderParams.geometryPackedMapIndex_,
      encoderParams.geometryPackedMapIndex_,
      "Packed map index for geometry" )
    ( "attributeScalePackedMapIndex",
      encoderParams.attributeScalePackedMapIndex_,
      encoderParams.attributeScalePackedMapIndex_,
      "Packed map index for scale attribute" )
    ( "attributeOpacityPackedMapIndex",
      encoderParams.attributeOpacityPackedMapIndex_,
      encoderParams.attributeOpacityPackedMapIndex_,
      "Packed map index for opacity attribute" )
    ( "attributeRotationPackedMapIndex",
      encoderParams.attributeRotationPackedMapIndex_,
      encoderParams.attributeRotationPackedMapIndex_,
      "Packed map index for rotation attribute" )
    ( "attributeColorPackedMapIndex",
      encoderParams.attributeColorPackedMapIndex_,
      encoderParams.attributeColorPackedMapIndex_,
      "Packed map index for texture or SH attribute" )
    ( "packedMapColumnsCount",
      encoderParams.packedMapColumnsCount_,
      encoderParams.packedMapColumnsCount_,
      "Number of columns for each packed map (e.g., '4,8' for 4 columns in map 0, 8 in map 1)" )
    ( "debugPath",
      encoderParams.debugPath_,
      encoderParams.debugPath_,
      "Path for debug output files" )
    ( "GSColmapFolderPath",
      encoderParams.gsColmapFolderPath_,
      encoderParams.gsColmapFolderPath_,
      "Colmap based Base Sparse Folder containing images.txt/cameras.txt and images.bin/cameras.bin files")
    
    // Gaussian Splatting encoder parameters
    ( "gsdTransform2dPresentFlag",
      encoderParams.gsdTransform2dPresentFlag_,
      false,
      "Enable 2D transform for SH (m75038 or m75790)" )
    ( "dataAdaptiveShTransformEnabled",
      encoderParams.dataAdaptiveShTransformEnabled_,
      false,
      "Enable data adaptive SH transform (m75790)" )
    ( "adaptivePcaEnabled",
      encoderParams.adaptivePcaEnabled_,
      false,
      "Enable adaptive global/group PCA selection (m76164)" )
    ( "gsdPcaDcFlag",
      encoderParams.gsdPcaDcFlag_,
      false,
      "Enable PCA for DC component of SH (m76858)" )
    ( "enableShNonUniformQuantization",
      encoderParams.enableShNonUniformQuantization_,
      false,
      "Enable non-uniform quantization for SH (m75025)" )
    ( "enableOpacityNonUniformQuantization",
      encoderParams.enableOpacityNonUniformQuantization_,
      false,
      "Enable non-uniform quantization for opacity (m76122)" )
    ("linearOpacity",
      encoderParams.encodeLinearOpacity_,
      false,
      "Enable linear opacity encoding")
    ( "enableScaleNonUniformQuantization",
      encoderParams.enableScaleNonUniformQuantization_,
      false,
      "Enable non-uniform quantization for Scale (m76123)" )
    ( "shPredictionEnabled",
      encoderParams.shPredictionEnabled_,
      false,
      "Enable SH linear prediction (m76155)" )
    ( "enableScalePowerTransform",
      encoderParams.enableScalePowerTransform_,
      false,
      "Enable power transform + 2-segment PLS for scale (m76151)" )
    ( "enableShDimensionReduction",
      adaptationParams.enableShDimensionReduction_,
      adaptationParams.enableShDimensionReduction_,
      "Enable SH dimension reduction after 2D transform (m76859)" )
    ( "rawPatchDataArrangementMethod",
      encoderParams.rawPatchDataArrangementMethod_,
      encoderParams.rawPatchDataArrangementMethod_,
      "Raw patch data arrangement method: 0=sequential, 1=vertical planar (m76580)" )
    ( "blockBasedSortBlockSize",                 
      encoderParams.blockBasedSortBlockSize_,
      encoderParams.blockBasedSortBlockSize_,
      "Block size for raw_patch_data_arrangement_method=1 (must be power of 2, >=8, default = 64)" )


    // VUI parameters (Annex G – ISO/IEC 23090-5)
    ( "aspsVuiParametersPresentFlag",
      encoderParams.aspsVuiParametersPresentFlag_,
      encoderParams.aspsVuiParametersPresentFlag_,
      "Enable VUI parameters in ASPS (asps_vui_parameters_present_flag)" )
    ( "vuiTimingInfoPresentFlag",
      encoderParams.vuiTimingInfoPresentFlag_,
      encoderParams.vuiTimingInfoPresentFlag_,
      "VUI: timing info present flag (vui_timing_info_present_flag)" )
    ( "vuiNumUnitsInTick",
      encoderParams.vuiNumUnitsInTick_,
      encoderParams.vuiNumUnitsInTick_,
      "VUI: number of time units per clock tick (vui_num_units_in_tick)" )
    ( "vuiTimeScale",
      encoderParams.vuiTimeScale_,
      encoderParams.vuiTimeScale_,
      "VUI: time scale in Hz (vui_time_scale)" )
    ( "vuiPocProportionalToTimingFlag",
      encoderParams.vuiPocProportionalToTimingFlag_,
      encoderParams.vuiPocProportionalToTimingFlag_,
      "VUI: POC proportional to timing flag (vui_poc_proportional_to_timing_flag)" )
    ( "vuiNumTicksPocDiffOneMinus1",
      encoderParams.vuiNumTicksPocDiffOneMinus1_,
      encoderParams.vuiNumTicksPocDiffOneMinus1_,
      "VUI: num ticks poc diff one minus 1 (vui_num_ticks_poc_diff_one_minus1)" )
    ( "vuiHrdParametersPresentFlag",
      encoderParams.vuiHrdParametersPresentFlag_,
      encoderParams.vuiHrdParametersPresentFlag_,
      "VUI: HRD parameters present flag (vui_hrd_parameters_present_flag)" )
    ( "vuiUnitInMetresFlag",
      encoderParams.vuiUnitInMetresFlag_,
      encoderParams.vuiUnitInMetresFlag_,
      "VUI: coordinates expressed in metres (vui_unit_in_metres_flag)" )
    ( "vuiDisplayBoxInfoPresentFlag",
      encoderParams.vuiDisplayBoxInfoPresentFlag_,
      encoderParams.vuiDisplayBoxInfoPresentFlag_,
      "VUI: display box info present flag (vui_display_box_info_present_flag)" )
    ( "vuiDisplayBoxOriginX",
      encoderParams.vuiDisplayBoxOriginX_,
      encoderParams.vuiDisplayBoxOriginX_,
      "VUI: display box origin X (vui_display_box_origin[0])" )
    ( "vuiDisplayBoxOriginY",
      encoderParams.vuiDisplayBoxOriginY_,
      encoderParams.vuiDisplayBoxOriginY_,
      "VUI: display box origin Y (vui_display_box_origin[1])" )
    ( "vuiDisplayBoxOriginZ",
      encoderParams.vuiDisplayBoxOriginZ_,
      encoderParams.vuiDisplayBoxOriginZ_,
      "VUI: display box origin Z (vui_display_box_origin[2])" )
    ( "vuiDisplayBoxSizeX",
      encoderParams.vuiDisplayBoxSizeX_,
      encoderParams.vuiDisplayBoxSizeX_,
      "VUI: display box size X (vui_display_box_size[0])" )
    ( "vuiDisplayBoxSizeY",
      encoderParams.vuiDisplayBoxSizeY_,
      encoderParams.vuiDisplayBoxSizeY_,
      "VUI: display box size Y (vui_display_box_size[1])" )
    ( "vuiDisplayBoxSizeZ",
      encoderParams.vuiDisplayBoxSizeZ_,
      encoderParams.vuiDisplayBoxSizeZ_,
      "VUI: display box size Z (vui_display_box_size[2])" )
    ( "vuiAnchorPointPresentFlag",
      encoderParams.vuiAnchorPointPresentFlag_,
      encoderParams.vuiAnchorPointPresentFlag_,
      "VUI: anchor point present flag (vui_anchor_point_present_flag)" )
    ( "vuiAnchorPointX",
      encoderParams.vuiAnchorPointX_,
      encoderParams.vuiAnchorPointX_,
      "VUI: anchor point X integer coordinate (vui_anchor_point[0])" )
    ( "vuiAnchorPointY",
      encoderParams.vuiAnchorPointY_,
      encoderParams.vuiAnchorPointY_,
      "VUI: anchor point Y integer coordinate (vui_anchor_point[1])" )
    ( "vuiAnchorPointZ",
      encoderParams.vuiAnchorPointZ_,
      encoderParams.vuiAnchorPointZ_,
      "VUI: anchor point Z integer coordinate (vui_anchor_point[2])" )

    ( "keepIntermediateFiles",
      encoderParams.keepIntermediateFiles_,
      encoderParams.keepIntermediateFiles_,
      "Keep intermediate files: RGB, YUV and bin" )      
    ( "attributeOrder",
      encoderParams.attributeOrder_,
      encoderParams.attributeOrder_,
      "Comma-separated AttributeTypeId integers for VPS/V3C attribute order "
      "(e.g. '2,5,6,7' = opacity,SH,scale,rotation). Empty = default order." )
    ( "ptlPriorityOrderEnableFlag",
      encoderParams.ptlPriorityOrderEnableFlag_,
      false,
      "Enable ptl_priority_order_enable_flag in profile_tier_level()" );

  // clang-format on
  po::setDefaults( opts );
  po::ErrorReporter         err;
  const list<const char *> &argv_unhandled = po::scanArgv( opts, argc, (const char **)argv, err );
  for ( const auto arg : argv_unhandled ) { printf( "Unhandled argument ignored: %s \n", arg ); }

  if ( argc == 1 || print_help ) {
    po::doHelp( std::cout, opts, 78 );
    return false;
  }

  applicationParams.print();

  // Validate attributeOrder if specified
  if ( !encoderParams.attributeOrder_.empty() ) {
    static const std::set<int> validIds = { 2, 5, 6, 7 };  // TRANSPARENCY, SH, SCALE, ROTATION
    if ( encoderParams.attributeOrder_.size() != 4 ) {
      std::cerr << "Error: attributeOrder must contain exactly 4 AttributeTypeId values\n";
      return false;
    }
    std::set<int> seen;
    for ( int id : encoderParams.attributeOrder_ ) {
      if ( !validIds.count( id ) ) {
        std::cerr << "Error: unsupported AttributeTypeId in attributeOrder: " << id << "\n";
        return false;
      }
      if ( !seen.insert( id ).second ) {
        std::cerr << "Error: duplicate AttributeTypeId in attributeOrder: " << id << "\n";
        return false;
      }
    }
  }

  // report the current configuration (only in the absence of errors so
  // that errors/warnings are more obvious and in the same place).
  if ( err.is_errored ) { return false; }

  return true;
}
