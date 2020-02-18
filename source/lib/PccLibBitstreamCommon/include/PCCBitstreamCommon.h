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
#ifndef PCC_BITSTREAM_COMMON_H
#define PCC_BITSTREAM_COMMON_H

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

namespace pcc {

// ******************************************************************* //
// Trace modes to validate new syntax
// ******************************************************************* //
// #define BITSTREAM_TRACE

// ******************************************************************* //
// Common constants
// ******************************************************************* //
#define ATLASIDXPCC 0

const uint32_t PCCTMC2ContainerMagicNumber = 23021981;
const uint32_t PCCTMC2ContainerVersion     = 1;
const uint32_t PCC_UNDEFINED_INDEX         = -1;
const bool     printDetailedInfo           = false;
const uint8_t MAX_NUM_ATTR_PARTITIONS = 64;
enum PCCVideoType {
  VIDEO_OCCUPANCY = 0,
  VIDEO_GEOMETRY,
  VIDEO_GEOMETRY_D0,
  VIDEO_GEOMETRY_D1,
  VIDEO_GEOMETRY_D2,
  VIDEO_GEOMETRY_D3,
  VIDEO_GEOMETRY_D4,
  VIDEO_GEOMETRY_D5,
  VIDEO_GEOMETRY_D6,
  VIDEO_GEOMETRY_D7,
  VIDEO_GEOMETRY_D8,
  VIDEO_GEOMETRY_D9,
  VIDEO_GEOMETRY_D10,
  VIDEO_GEOMETRY_D11,
  VIDEO_GEOMETRY_D12,
  VIDEO_GEOMETRY_D13,
  VIDEO_GEOMETRY_D14,
  VIDEO_GEOMETRY_D15,
  VIDEO_GEOMETRY_RAW,
  VIDEO_TEXTURE,
  VIDEO_TEXTURE_T0,
  VIDEO_TEXTURE_T1 = VIDEO_TEXTURE_T0 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_TEXTURE_T2 = VIDEO_TEXTURE_T1 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_TEXTURE_T3 = VIDEO_TEXTURE_T2 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_TEXTURE_T4 = VIDEO_TEXTURE_T3 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_TEXTURE_T5 = VIDEO_TEXTURE_T4 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_TEXTURE_T6 = VIDEO_TEXTURE_T5 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_TEXTURE_T7 = VIDEO_TEXTURE_T6 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_TEXTURE_T8 = VIDEO_TEXTURE_T7 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_TEXTURE_T9 = VIDEO_TEXTURE_T8 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_TEXTURE_T10 = VIDEO_TEXTURE_T9 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_TEXTURE_T11 = VIDEO_TEXTURE_T10 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_TEXTURE_T12 = VIDEO_TEXTURE_T11 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_TEXTURE_T13 = VIDEO_TEXTURE_T12 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_TEXTURE_T14 = VIDEO_TEXTURE_T13 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_TEXTURE_T15 = VIDEO_TEXTURE_T14 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_TEXTURE_RAW = VIDEO_TEXTURE_T15 + MAX_NUM_ATTR_PARTITIONS,
  NUM_VIDEO_TYPE = VIDEO_TEXTURE_RAW + MAX_NUM_ATTR_PARTITIONS
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


enum PCCCodecID { CODEC_HEVC = 0 };

enum PCCCodecGroup {
  CODEC_GROUP_AVC_PROGRESSIVE_HIGH = 0,
  CODEC_GROUP_HEVC_MAIN10,
  CODEC_GROUP_HEVC444,
  CODEC_GROUP_MP4RA
};
enum PCCPatchFrameType { PATCH_FRAME_I = 0, PATCH_FRAME_P };

enum PCCPatchModeI { PATCH_MODE_I_INTRA = 0, PATCH_MODE_I_RAW, PATCH_MODE_I_EOM, PATCH_MODE_I_END = 14 };

enum PCCPatchModeP { // to be removed
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
  size_t typeIndex = (size_t) type;
  if( typeIndex == (size_t)VIDEO_OCCUPANCY)         return std::string( "occupancy map video " );
  else if( typeIndex == (size_t)VIDEO_GEOMETRY)     return std::string( "geometry video " );
  else if( typeIndex >= (size_t)VIDEO_GEOMETRY_D0 && typeIndex <= (size_t)VIDEO_GEOMETRY_D15 )
       return std::string( "geometry D" )+ std::to_string (typeIndex-(size_t)VIDEO_GEOMETRY_D0 ) + std::string(" video ");
  else if( typeIndex == (size_t)VIDEO_GEOMETRY_RAW) return std::string( "raw points geometry video " );
  else if( typeIndex == (size_t)VIDEO_TEXTURE)      return std::string( "texture video " );
  else if( typeIndex >= (size_t)VIDEO_TEXTURE_T0 && typeIndex <= (size_t)VIDEO_TEXTURE_T15 )
       return std::string( "texture T" )+ std::to_string (typeIndex-(size_t)VIDEO_TEXTURE_T0 ) + std::string(" video ");
  else if( typeIndex == (size_t)VIDEO_TEXTURE_RAW)  return std::string( "raw points texture video " );
  else                                              return std::string( "not supported" );
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


}  // namespace pcc

#endif /* PCC_BITSTREAM_COMMON_H */
