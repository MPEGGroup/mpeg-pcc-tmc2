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
#ifndef PCCTMC2Common_h
#define PCCTMC2Common_h

#include <limits>
#include <assert.h>
#include <chrono>
#include <cmath>
#include <vector>
#include <list>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <string>
#include <cstring>
#include <unordered_map>
#include <sys/stat.h>
#include <memory>
#include <queue>
#include <algorithm>
#include <map>
#include "PCCConfig.h"
#if defined( WIN32 )
#include <windows.h>
#endif
#if defined( __APPLE__ ) && defined( __MACH__ )
#include <unistd.h>
#include <sys/resource.h>
#include <mach/mach.h>
#endif
#ifdef ENABLE_PAPI_PROFILING
#include <papi.h>
#include "PCCChrono.h"
#endif


namespace pcc {

// ******************************************************************* //
// Version information
// ******************************************************************* //
#define TM2_VERSION TMC2_VERSION_MAJOR "." TMC2_VERSION_MINOR

// ******************************************************************* //
// Coding tool configuration
// ******************************************************************* //
static const uint8_t PCC_SAVE_POINT_TYPE = 0;  // Save point information in reconstructed ply.


// ******************************************************************* //
// Trace modes to validate new syntax
// ******************************************************************* //
// #define BITSTREAM_TRACE
// #define CODEC_TRACE 

// ******************************************************************* //
// Common constants
// ******************************************************************* //
#define ATLASIDXPCC 0
#define POSTSMOOTHING_RGB2YUV 0

const int16_t infiniteDepth  = ( std::numeric_limits<int16_t>::max )();
const int64_t infinitenumber = ( std::numeric_limits<int64_t>::max )();

const uint32_t PCCTMC2ContainerMagicNumber = 23021981;
const uint32_t PCCTMC2ContainerVersion     = 1;
const uint32_t PCC_UNDEFINED_INDEX         = -1;
const bool     printDetailedInfo           = false;

enum PCCEndianness { PCC_BIG_ENDIAN = 0, PCC_LITTLE_ENDIAN = 1 };
enum PCCColorTransform { COLOR_TRANSFORM_NONE = 0, COLOR_TRANSFORM_RGB_TO_YCBCR = 1 };
enum PCCAxis6 {
  PCC_AXIS6_UNDEFINED = -1,
  PCC_AXIS6_X_NEAR    = 0,
  PCC_AXIS6_Y_NEAR    = 1,
  PCC_AXIS6_Z_NEAR    = 2,
  PCC_AXIS6_X_FAR     = 3,
  PCC_AXIS6_Y_FAR     = 4,
  PCC_AXIS6_Z_FAR     = 5
};
enum PCCAxis3 { PCC_AXIS3_UNDEFINED = -1, PCC_AXIS3_X = 0, PCC_AXIS3_Y = 1, PCC_AXIS3_Z = 2 };
enum PCCPointType { POINT_UNSET = 0, POINT_D0, POINT_D1, POINT_DF, POINT_SMOOTH, POINT_EDD };
enum PCCVideoType {
  VIDEO_OCCUPANCY = 0,
  VIDEO_GEOMETRY,
  VIDEO_GEOMETRY_D0,
  VIDEO_GEOMETRY_D1,
  VIDEO_GEOMETRY_RAW,
  VIDEO_TEXTURE,
  VIDEO_TEXTURE_T0,
  VIDEO_TEXTURE_T1,
  VIDEO_TEXTURE_RAW,
  NUM_VIDEO_TYPE
};
enum PCCMetadataType { METADATA_GOF = 0, METADATA_FRAME, METADATA_PATCH };
enum PCCPatchOrientation {
  PATCH_ORIENTATION_DEFAULT = 0,
  PATCH_ORIENTATION_SWAP    = 1,
  PATCH_ORIENTATION_ROT90   = 2,
  PATCH_ORIENTATION_ROT180  = 3,
  PATCH_ORIENTATION_ROT270  = 4,
  PATCH_ORIENTATION_MIRROR  = 5,
  PATCH_ORIENTATION_MROT90  = 6,
  PATCH_ORIENTATION_MROT180 = 7,
  PATCH_ORIENTATION_MROT270 = 8 //similar to SWAP, not used
};  // switched SWAP with ROT90 positions

enum VPCCUnitType {
  VPCC_VPS = 0,       // 0: Sequence parameter set
  VPCC_AD,            // 1: Patch Data Group
  VPCC_OVD,           // 2: Occupancy Video Data
  VPCC_GVD,           // 3: Geometry Video Data
  VPCC_AVD,           // 4: Attribute Video Data
  VPCC_RSVD_05,       // 05: Reserved
  VPCC_RSVD_06,       // 06: Reserved
  VPCC_RSVD_07,       // 07: Reserved
  VPCC_RSVD_08,       // 08: Reserved
  VPCC_RSVD_09,       // 09: Reserved
  VPCC_RSVD_10,       // 10: Reserved
  VPCC_RSVD_11,       // 11: Reserved
  VPCC_RSVD_12,       // 12: Reserved
  VPCC_RSVD_13,       // 13: Reserved
  VPCC_RSVD_14,       // 14: Reserved
  VPCC_RSVD_15,       // 15: Reserved
  VPCC_RSVD_16,       // 16: Reserved
  VPCC_RSVD_17,       // 17: Reserved
  VPCC_RSVD_18,       // 18: Reserved
  VPCC_RSVD_19,       // 19: Reserved
  VPCC_RSVD_20,       // 20: Reserved
  VPCC_RSVD_21,       // 21: Reserved
  VPCC_RSVD_22,       // 22: Reserved
  VPCC_RSVD_23,       // 23: Reserved
  VPCC_RSVD_24,       // 24: Reserved
  VPCC_RSVD_25,       // 25: Reserved
  VPCC_RSVD_26,       // 26: Reserved
  VPCC_RSVD_27,       // 27: Reserved
  VPCC_RSVD_28,       // 28: Reserved
  VPCC_RSVD_29,       // 29: Reserved
  VPCC_RSVD_30,       // 30: Reserved
  VPCC_RSVD_31,       // 32: Reserved
  NUM_VPCC_UNIT_TYPE  // undefined
};

enum PDGUnitType {
  PDG_PSPS = 0,    // 00: Patch sequence parameter set
  PDG_PFPS,        // 01: Patch frame parameter set
  PDG_PFGPS,       // 02: Patch frame geometry parameter set
  PDG_PFAPS,       // 03: Patch frame attribute parameter set
  PDG_GPPS,        // 04: Geometry patch parameter set
  PDG_APPS,        // 05: Attribute patch parameter set
  PDG_PTGLU,       // 06: Patch tile group layer unit
  PDG_PREFIX_SEI,  // 07: Prefix SEI message
  PDG_SUFFIX_SEI,  // 08: Suffix SEI message
  PDG_RSVD_09,     // 09: Reserved
  PDG_RSVD_10,     // 10: Reserved
  PDG_RSVD_11,     // 11: Reserved
  PDG_RSVD_12,     // 12: Reserved
  PDG_RSVD_13,     // 13: Reserved
  PDG_RSVD_14,     // 14: Reserved
  PDG_RSVD_15,     // 15: Reserved
  PDG_RSVD_16,     // 16: Reserved
  PDG_RSVD_17,     // 17: Reserved
  PDG_RSVD_18,     // 18: Reserved
  PDG_RSVD_19,     // 19: Reserved
  PDG_RSVD_20,     // 20: Reserved
  PDG_RSVD_21,     // 21: Reserved
  PDG_RSVD_22,     // 22: Reserved
  PDG_RSVD_23,     // 23: Reserved
  PDG_RSVD_24,     // 24: Reserved
  PDG_RSVD_25,     // 25: Reserved
  PDG_RSVD_26,     // 26: Reserved
  PDG_RSVD_27,     // 27: Reserved
  PDG_RSVD_28,     // 28: Reserved
  PDG_RSVD_29,     // 29: Reserved
  PDG_RSVD_30,     // 30: Reserved
  PDG_RSVD_31      // 32: Reserved
};

enum PCCCodecID { CODEC_HEVC = 0 };

enum PCCPatchFrameType { PATCH_FRAME_I = 0, PATCH_FRAME_P };

enum PCCPatchModeI { PATCH_MODE_I_INTRA = 0, PATCH_MODE_I_RAW, PATCH_MODE_I_EOM, PATCH_MODE_I_END = 14 };

enum PCCPatchModeP {
  PATCH_MODE_P_SKIP = 0,
  PATCH_MODE_P_INTRA,
  PATCH_MODE_P_INTER,
  PATCH_MODE_P_MERGE,
  PATCH_MODE_P_RAW,
  PATCH_MODE_P_EOM,
  PATCH_MODE_P_END = 14
};
enum PCCPatchType { INTRA_PATCH = 0, INTER_PATCH, MERGE_PATCH, SKIP_PATCH, RAW_PATCH, EOM_PATCH, END_PATCH, ERROR_PATCH };
enum PCCTILEGROUP { P_TILE_GRP = 0, SKIP_TILE_GRP, I_TILE_GRP };
enum { COLOURFORMAT420 = 0, COLOURFORMAT444 = 1 };

const size_t IntermediateLayerIndex = 100;
const size_t eddLayerIndex          = 10;
const size_t NeighborThreshold      = 4;
const size_t NumPatchOrientations   = 8;
const size_t gbitCountSize[]        = {
    0, 0, 1, 0, 2, 0, 0, 0,  // 0-7
    3, 0, 0, 0, 0, 0, 0, 0,  // 8-
    4, 0, 0, 0, 0, 0, 0, 0,  // 16-
    0, 0, 0, 0, 0, 0, 0, 0,  // 24-
    5, 0, 0, 0, 0, 0, 0, 0,  // 32-
    0, 0, 0, 0, 0, 0, 0, 0,  // 40
    0, 0, 0, 0, 0, 0, 0, 0,  // 48
    0, 0, 0, 0, 0, 0, 0, 0,  // 56
    6                        // 64
};

const int32_t InvalidPatchIndex = -1;
enum SeiPayloadType {
  BUFFERING_PERIOD = 0,             //  0: bufferingPeriod
  ATLAS_FRAME_TIMING,               //  1: atlasFrameTiming
  FILLER_PAYLOAD,                   //  2: fillerPayload
  USER_DATAREGISTERED_ITUTT35,      //  3: userDataRegisteredItuTT35
  USER_DATA_UNREGISTERED,           //  4: userDataUnregistered
  RECOVERY_POINT,                   //  5: recoveryPoint
  NO_DISPLAY,                       //  6: noDisplay
  TIME_CODE,                        //  7: timeCode
  REGIONAL_NESTING,                 //  8: regionalNesting
  SEI_MANIFEST,                     //  9: seiManifest
  SEI_PREFIX_INDICATION,            // 10: seiPrefixIndication
  GEOMETRY_TRANSFORMATION_PARAMS,   // 11: geometryTransformationParams
  ATTRIBUTE_TRANSFORMATION_PARAMS,  // 12: attributeTransformationParams
  ACTIVE_SUBSTREAMS,                // 13: activeSubstreams
  COMPONENT_CODEC_MAPPING,          // 14: componentCodecMapping
  VOLUMETRIC_TILING_INFO,           // 15: volumetricTilingInfo
  PRESENTATION_INFORMATION,         // 16: presentationInformation
  SMOOTHING_PARAMETERS,             // 17: smoothingParameters
  RESERVED_SEI_MESSAGE,             // 18: reservedSeiMessage
};

enum NalUnitType {  // Name, Content of NAL unit and RBSP syntax structure NAL unit type class
  NAL_TRAIL = 0,    // 0: Coded tile group of a non-TSA, non STSA trailing atlas frame atlas_tile_group_layer_rbsp() ACL
  NAL_TSA,          // 1: Coded tile group of a TSA atlas frame, atlas_tile_group_layer_rbsp() ACL
  NAL_STSA,         // 2: Coded tile group of a STSA atlas frame, atlas_tile_group_layer_rbsp() ACL
  NAL_RADL,         // 3: Coded tile group of a RADL atlas frame, atlas_tile_group_layer_rbsp() ACL
  NAL_RASL,         // 4: Coded tile group of a RASL atlas frame, atlas_tile_group_layer_rbsp() ACL
  NAL_SKIP,         // 5: Coded tile group of a skipped atlas frame, atlas_tile_group_layer_rbsp() ACL
  NAL_RSV_ACL_6,    // 6: Reserved non-IRAP ACL NAL unit types ACL
  NAL_RSV_ACL_7,    // 7: Reserved non-IRAP ACL NAL unit types ACL
  NAL_RSV_ACL_8,    // 8: Reserved non-IRAP ACL NAL unit types ACL
  NAL_RSV_ACL_9,    // 9: Reserved non-IRAP ACL NAL unit types ACL
  NAL_BLA_W_LP,     // 10: Coded tile group of a BLA atlas frame, atlas_tile_group_layer_rbsp() ACL
  NAL_BLA_W_RADL,   // 11: Coded tile group of a BLA atlas frame, atlas_tile_group_layer_rbsp() ACL
  NAL_BLA_N_LP,     // 12: Coded tile group of a BLA atlas frame, atlas_tile_group_layer_rbsp() ACL
  NAL_GBLA_W_LP,    // 13: Coded tile group of a GBLA atlas frame, atlas_tile_group_layer_rbsp() ACL
  NAL_GBLA_W_RADL,  // 14: Coded tile group of a GBLA atlas frame, atlas_tile_group_layer_rbsp() ACL
  NAL_GBLA_N_LP,    // 15: Coded tile group of a GBLA atlas frame, atlas_tile_group_layer_rbsp() ACL
  NAL_IDR_W_RADL,   // 16: Coded tile group of an IDR atlas frame, atlas_tile_group_layer_rbsp() ACL
  NAL_IDR_N_LP,     // 17: Coded tile group of an IDR atlas frame, atlas_tile_group_layer_rbsp() ACL
  NAL_GIDR_W_RADL,  // 18: Coded tile group of a GIDR atlas frame, atlas_tile_group_layer_rbsp() ACL
  NAL_GIDR_N_LP,    // 19: Coded tile group of a GIDR atlas frame, atlas_tile_group_layer_rbsp() ACL
  NAL_CRA,          // 20: Coded tile group of a CRA atlas frame, atlas_tile_group_layer_rbsp() ACL
  NAL_GCRA,         // 21: Coded tile group of a GCRA atlas frame, atlas_tile_group_layer_rbsp() ACL
  NAL_IRAP_ACL_22,  // 22: Reserved IRAP ACL NAL unit types ACL
  NAL_IRAP_ACL_23,  // 23: Reserved IRAP ACL NAL unit types ACL
  NAL_RSV_ACL_24,   // 24: Reserved non-IRAP ACL NAL unit types ACL
  NAL_RSV_ACL_25,   // 25: Reserved non-IRAP ACL NAL unit types ACL
  NAL_RSV_ACL_26,   // 26: Reserved non-IRAP ACL NAL unit types ACL
  NAL_RSV_ACL_27,   // 27: Reserved non-IRAP ACL NAL unit types ACL
  NAL_RSV_ACL_28,   // 28: Reserved non-IRAP ACL NAL unit types ACL
  NAL_RSV_ACL_29,   // 29: Reserved non-IRAP ACL NAL unit types ACL
  NAL_RSV_ACL_30,   // 30: Reserved non-IRAP ACL NAL unit types ACL
  NAL_RSV_ACL_31,   // 31: Reserved non-IRAP ACL NAL unit types ACL
  NAL_ASPS,         // 32: Atlas sequence parameter set atlas_sequence_parameter_set_rbsp() non-ACL
  NAL_AFPS,         // 33: Atlas frame parameter set atlas_frame_parameter_set_rbsp() non-ACL
  NAL_AUD,          // 34: Access unit delimiter access_unit_delimiter_rbsp() non-ACL
  NAL_VPCC_AUD,     // 35: V-PCC access unit delimiter access_unit_delimiter_rbsp() non-ACL
  NAL_EOS,          // 36: End of sequence end_of_seq_rbsp() non-ACL
  NAL_EOB,          // 37! End of bitstream end_of_atlas_substream_rbsp() non-ACL
  NAL_FD,           // 38: Filler filler_data_rbsp() non-ACL
  NAL_PREFIX_SEI,   // 39: Supplemental enhancement information sei_rbsp() non-ACL
  NAL_SUFFIX_SEI,   // 40: Supplemental enhancement information sei_rbsp() non-ACL
  NAL_RSV_NACL_41,  // 42: Reserved non-ACL NAL unit types non-ACL
  NAL_RSV_NACL_42,  // 42: Reserved non-ACL NAL unit types non-ACL
  NAL_RSV_NACL_43,  // 43: Reserved non-ACL NAL unit types non-ACL
  NAL_RSV_NACL_44,  // 44: Reserved non-ACL NAL unit types non-ACL
  NAL_RSV_NACL_45,  // 45: Reserved non-ACL NAL unit types non-ACL
  NAL_RSV_NACL_46,  // 46: Reserved non-ACL NAL unit types non-ACL
  NAL_RSV_NACL_47,  // 47: Reserved non-ACL NAL unit types non-ACL
  NAL_UNSPEC_48,    // 48: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_49,    // 49: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_50,    // 50: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_51,    // 51: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_52,    // 52: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_53,    // 53: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_54,    // 54: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_55,    // 55: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_56,    // 56: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_57,    // 57: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_58,    // 58: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_59,    // 59: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_60,    // 60: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_61,    // 61: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_62,    // 62: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_63     // 63: Unspecified non-ACL NAL unit types non-ACL
};

// ******************************************************************* //
// Static functions
// ******************************************************************* //
static inline unsigned int getMaxBit( int16_t h ) {
  int maxBit = 0;
  for ( int n = 0; n <= 16; n++ )
    if ( h & ( 1 << n ) ) maxBit = ( std::max )( maxBit, n + 1 );
  return maxBit;
}

static inline std::string toString( NalUnitType type ) {
  switch ( type ) {
    case NAL_ASPS: return std::string( "NAL_ASPS" ); break;
    case NAL_AFPS: return std::string( "NAL_AFPS" ); break;
    case NAL_AUD: return std::string( "NAL_AUD" ); break;
    case NAL_TRAIL: return std::string( "NAL_TRAIL" ); break;
    case NAL_TSA: return std::string( "NAL_TSA" ); break;
    case NAL_STSA: return std::string( "NAL_STSA" ); break;
    case NAL_RADL: return std::string( "NAL_RADL" ); break;
    case NAL_RASL: return std::string( "NAL_RASL" ); break;
    case NAL_SKIP: return std::string( "NAL_SKIP" ); break;
    case NAL_PREFIX_SEI: return std::string( "NAL_PREFIX_SEI" ); break;
    case NAL_SUFFIX_SEI: return std::string( "NAL_SUFFIX_SEI" ); break;

    default: return std::string( "others" ); break;
  }
}

static inline std::string toString( VPCCUnitType type ) {
  switch ( type ) {
    case VPCC_VPS: return std::string( "VPCC_VPS" ); break;
    case VPCC_AD: return std::string( "VPCC_AD" ); break;
    case VPCC_OVD: return std::string( "VPCC_OVD" ); break;
    case VPCC_GVD: return std::string( "VPCC_GVD" ); break;
    case VPCC_AVD: return std::string( "VPCC_AVD" ); break;
    default: return std::string( "reserved?" ); break;
  }
}
static inline std::string toString( PCCVideoType type ) {
  switch ( type ) {
    case VIDEO_OCCUPANCY: return std::string( "occupancy map video" ); break;
    case VIDEO_GEOMETRY: return std::string( "geometry video" ); break;
    case VIDEO_GEOMETRY_D0: return std::string( "geometry D0 video" ); break;
    case VIDEO_GEOMETRY_D1: return std::string( "geometry D1 video" ); break;
    case VIDEO_GEOMETRY_RAW: return std::string( "missed points geometry video" ); break;
    case VIDEO_TEXTURE: return std::string( "texture video " ); break;
    case VIDEO_TEXTURE_T0: return std::string( "texture T0 video " ); break;
    case VIDEO_TEXTURE_T1: return std::string( "texture T1 video " ); break;
    case VIDEO_TEXTURE_RAW: return std::string( "missed points texture video" ); break;
    case NUM_VIDEO_TYPE: return std::string( "not supported" ); break;
  }
  return std::string( "not supported" );
}
static bool exist( const std::string& sString ) {
  struct stat buffer;
  return ( stat( sString.c_str(), &buffer ) == 0 );
}

static inline void removeFile( const std::string string ) {
  if ( exist( string ) ) {
    if ( remove( string.c_str() ) != 0 ) { std::cout << "Could not remove the file: " << string << std::endl; }
  }
}

static inline std::string removeFileExtension( const std::string string ) {
  size_t pos = string.find_last_of( "." );
  return ( pos != std::string::npos && pos + 4 == string.length() ) ? string.substr( 0, pos ) : string;
}

static inline std::string addVideoFormat( const std::string string,
                                          const size_t      width,
                                          const size_t      height,
                                          const bool        yuv420 = true,
                                          const std::string pixel  = "8" ) {
  size_t      pos      = string.find_last_of( "." );
  std::string filename = string.substr( 0, pos ), extension = string.substr( pos );
  if ( extension == ".yuv" || extension == ".rgb" ) {
    std::stringstream result;
    result << filename << "_" << width << "x" << height << "_" << pixel << "bit_"
           << ( extension == ".yuv" && yuv420 ? "p420" : "p444" ) << extension;
    return result.str();
  }
  return string;
}

static inline PCCEndianness PCCSystemEndianness() {
  uint32_t num = 1;
  return ( *( reinterpret_cast<char*>( &num ) ) == 1 ) ? PCC_LITTLE_ENDIAN : PCC_BIG_ENDIAN;
}

template <typename T>
static inline const T PCCEndianSwap( const T u ) {
  union {
    T       u;
    uint8_t u8[sizeof( T )];
  } source, dest;
  source.u = u;
  for ( size_t k = 0; k < sizeof( T ); k++ ) dest.u8[k] = source.u8[sizeof( T ) - k - 1];
  return dest.u;
}

template <typename T>
static inline const T PCCToLittleEndian( const T u ) {
  return ( PCCSystemEndianness() == PCC_BIG_ENDIAN ) ? PCCEndianSwap( u ) : u;
}

template <typename T>
static inline const T PCCFromLittleEndian( const T u ) {
  return ( PCCSystemEndianness() == PCC_BIG_ENDIAN ) ? PCCEndianSwap( u ) : u;
}

static inline void PCCDivideRange( const size_t         start,
                                   const size_t         end,
                                   const size_t         chunckCount,
                                   std::vector<size_t>& subRanges ) {
  const size_t elementCount = end - start;
  if ( elementCount <= chunckCount ) {
    subRanges.resize( elementCount + 1 );
    for ( size_t i = start; i <= end; ++i ) { subRanges[i - start] = i; }
  } else {
    subRanges.resize( chunckCount + 1 );
    const double step = static_cast<double>( elementCount ) / ( chunckCount + 1 );
    double       pos  = static_cast<double>( start );
    for ( size_t i = 0; i < chunckCount; ++i ) {
      subRanges[i] = static_cast<size_t>( pos );
      pos += step;
    }
    subRanges[chunckCount] = end;
  }
}

static inline uint32_t getFixedLengthCodeBitsCount( uint32_t range ) {
  int count = 0;
  if ( range > 0 ) {
    range -= 1;
    while ( range > 0 ) {
      count++;
      range >>= 1;
    }
  }
  return count;
}

template <typename... Args>
std::string stringFormat( const char* pFormat, Args... eArgs ) {
  size_t      iSize = snprintf( NULL, 0, pFormat, eArgs... );
  std::string eBuffer;
  eBuffer.reserve( iSize + 1 );
  eBuffer.resize( iSize );
  snprintf( &eBuffer[0], iSize + 1, pFormat, eArgs... );
  return eBuffer;
}

template <typename T>
void printVector( std::vector<T>    data,
                  const size_t      width,
                  const size_t      height,
                  const std::string string,
                  const bool        hexa = false ) {
  if ( data.size() == 0 ) { data.resize( width * height, 0 ); }
  printf( "%s: %lu %lu \n", string.c_str(), width, height );
  for ( size_t v0 = 0; v0 < height; ++v0 ) {
    for ( size_t u0 = 0; u0 < width; ++u0 ) {
      if ( hexa ) {
        printf( "%2x", (int)( data[v0 * width + u0] ) );
      } else {
        printf( "%3d", (int)( data[v0 * width + u0] ) );
      }
    }
    printf( "\n" );
    fflush( stdout );
  }
}

static inline const std::string strUnitType( PDGUnitType unitType ) {
  switch ( unitType ) {
    case PDG_PSPS: return std::string( "PDG_PSPS " ); break;
    case PDG_PFPS: return std::string( "PDG_PFPS " ); break;
    case PDG_PFGPS: return std::string( "PDG_PFGPS" ); break;
    case PDG_PFAPS: return std::string( "PDG_PFAPS" ); break;
    case PDG_GPPS: return std::string( "PDG_GPPS " ); break;
    case PDG_APPS: return std::string( "PDG_APPS " ); break;
    case PDG_PTGLU: return std::string( "PDG_PTGLU" ); break;
    default: break;
  }
  return std::string( "ERROR" );
}

using Range = std::pair<int, int>;
struct Tile {
  int minU;
  int maxU;
  int minV;
  int maxV;
  Tile() : minU( -1 ), maxU( -1 ), minV( -1 ), maxV( -1 ){};
};

#ifdef ENABLE_PAPI_PROFILING
#define ERROR_RETURN( retval )                                              \
  fprintf( stderr, "Error %d %s:line %d: \n", retval, __FILE__, __LINE__ ); \
  exit( retval );

static void initPapiProfiler() {
  int  retval;
  char errstring[PAPI_MAX_STR_LEN];
  if ( ( retval = PAPI_library_init( PAPI_VER_CURRENT ) ) != PAPI_VER_CURRENT ) { ERROR_RETURN( retval ); }
  return;
}

static void createPapiEvent( int& EventSet ) {
  EventSet = PAPI_NULL;
  int retval, number = 0;
  if ( ( retval = PAPI_create_eventset( &EventSet ) ) != PAPI_OK ) { ERROR_RETURN( retval ); }
  if ( ( retval = PAPI_add_event( EventSet, PAPI_TOT_INS ) ) != PAPI_OK ) { ERROR_RETURN( retval ); }
  if ( ( retval = PAPI_add_event( EventSet, PAPI_TOT_CYC ) ) != PAPI_OK ) { ERROR_RETURN( retval ); }
  if ( ( retval = PAPI_add_event( EventSet, PAPI_L2_TCA ) ) != PAPI_OK ) { ERROR_RETURN( retval ); }
  if ( ( retval = PAPI_add_event( EventSet, PAPI_L3_TCA ) ) != PAPI_OK ) { ERROR_RETURN( retval ); }
  if ( ( retval = PAPI_add_event( EventSet, PAPI_L1_DCM ) ) != PAPI_OK ) { ERROR_RETURN( retval ); }
  if ( ( retval = PAPI_add_event( EventSet, PAPI_L2_DCM ) ) != PAPI_OK ) { ERROR_RETURN( retval ); }
  if ( ( retval = PAPI_list_events( EventSet, NULL, &number ) ) != PAPI_OK ) { ERROR_RETURN( retval ); }
  return;
}

#define PAPI_PROFILING_INITIALIZE                          \
  int       EventSet = PAPI_NULL;                          \
  int       retval;                                        \
  long long values[16];                                    \
  createPapiEvent( EventSet );                             \
  pcc::chrono::Stopwatch<std::chrono::steady_clock> clock; \
  clock.reset();                                           \
  clock.start();                                           \
  if ( ( retval = PAPI_start( EventSet ) ) != PAPI_OK ) { ERROR_RETURN( retval ); }

#define PAPI_PROFILING_RESULTS                                                                    \
  clock.stop();                                                                                   \
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>( clock.count() ).count(); \
  if ( ( retval = PAPI_read( EventSet, values ) ) != PAPI_OK ) { ERROR_RETURN( retval ); }        \
  printf( "PAPI: number of instructions           : %lld \n", values[0] );                        \
  printf( "PAPI: number of cycles                 : %lld \n", values[1] );                        \
  printf( "PAPI: number of L2 cache memory access : %lld \n", values[2] );                        \
  printf( "PAPI: number of L3 cache memory access : %lld \n", values[3] );                        \
  printf( "PAPI: number of L1 cache misses        : %lld \n", values[4] );                        \
  printf( "PAPI: number of L2 cache misses        : %lld \n", values[5] );                        \
  printf( "PAPI: Processing time (wall)           : %lld \n", duration );
#endif
}  // namespace pcc

#endif /* PCCTMC2Common_h */
