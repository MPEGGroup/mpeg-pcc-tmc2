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

#define NOMINMAX
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
// #define CODEC_TRACE
// #define SEI_TRACE
// #define CONFORMANCE_TRACE

// ******************************************************************* //
// Common constants
// ******************************************************************* //
#define MAX_NUM_ATTR_PARTITIONS 64

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
  VIDEO_ATTRIBUTE,
  VIDEO_ATTRIBUTE_T0,
  VIDEO_ATTRIBUTE_T1  = VIDEO_ATTRIBUTE_T0 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_ATTRIBUTE_T2  = VIDEO_ATTRIBUTE_T1 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_ATTRIBUTE_T3  = VIDEO_ATTRIBUTE_T2 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_ATTRIBUTE_T4  = VIDEO_ATTRIBUTE_T3 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_ATTRIBUTE_T5  = VIDEO_ATTRIBUTE_T4 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_ATTRIBUTE_T6  = VIDEO_ATTRIBUTE_T5 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_ATTRIBUTE_T7  = VIDEO_ATTRIBUTE_T6 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_ATTRIBUTE_T8  = VIDEO_ATTRIBUTE_T7 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_ATTRIBUTE_T9  = VIDEO_ATTRIBUTE_T8 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_ATTRIBUTE_T10 = VIDEO_ATTRIBUTE_T9 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_ATTRIBUTE_T11 = VIDEO_ATTRIBUTE_T10 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_ATTRIBUTE_T12 = VIDEO_ATTRIBUTE_T11 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_ATTRIBUTE_T13 = VIDEO_ATTRIBUTE_T12 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_ATTRIBUTE_T14 = VIDEO_ATTRIBUTE_T13 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_ATTRIBUTE_T15 = VIDEO_ATTRIBUTE_T14 + MAX_NUM_ATTR_PARTITIONS,
  VIDEO_ATTRIBUTE_RAW = VIDEO_ATTRIBUTE_T15 + MAX_NUM_ATTR_PARTITIONS,
  NUM_VIDEO_TYPE      = VIDEO_ATTRIBUTE_RAW + MAX_NUM_ATTR_PARTITIONS
};
enum PCCMetadataType { METADATA_GOF = 0, METADATA_FRAME, METADATA_PATCH };
enum PCCPatchOrientation {
  PATCH_ORIENTATION_DEFAULT = 0,  // 0: default
  PATCH_ORIENTATION_SWAP    = 1,  // 1: swap
  PATCH_ORIENTATION_ROT90   = 2,  // 2: rotation 90
  PATCH_ORIENTATION_ROT180  = 3,  // 3: rotation 180
  PATCH_ORIENTATION_ROT270  = 4,  // 4: rotation 270
  PATCH_ORIENTATION_MIRROR  = 5,  // 5: mirror
  PATCH_ORIENTATION_MROT90  = 6,  // 6: mirror + rotation 90
  PATCH_ORIENTATION_MROT180 = 7,  // 7: mirror + rotation 180
  PATCH_ORIENTATION_MROT270 = 8   // 8: similar to SWAP, not used switched SWAP with ROT90 positions
};

enum V3CUnitType {
  V3C_VPS = 0,       //  0: Sequence parameter set
  V3C_AD,            //  1: Patch Data Group
  V3C_OVD,           //  2: Occupancy Video Data
  V3C_GVD,           //  3: Geometry Video Data
  V3C_AVD,           //  4: Attribute Video Data
  V3C_RSVD_05,       //  5: Reserved
  V3C_RSVD_06,       //  6: Reserved
  V3C_RSVD_07,       //  7: Reserved
  V3C_RSVD_08,       //  8: Reserved
  V3C_RSVD_09,       //  9: Reserved
  V3C_RSVD_10,       // 10: Reserved
  V3C_RSVD_11,       // 11: Reserved
  V3C_RSVD_12,       // 12: Reserved
  V3C_RSVD_13,       // 13: Reserved
  V3C_RSVD_14,       // 14: Reserved
  V3C_RSVD_15,       // 15: Reserved
  V3C_RSVD_16,       // 16: Reserved
  V3C_RSVD_17,       // 17: Reserved
  V3C_RSVD_18,       // 18: Reserved
  V3C_RSVD_19,       // 19: Reserved
  V3C_RSVD_20,       // 20: Reserved
  V3C_RSVD_21,       // 21: Reserved
  V3C_RSVD_22,       // 22: Reserved
  V3C_RSVD_23,       // 23: Reserved
  V3C_RSVD_24,       // 24: Reserved
  V3C_RSVD_25,       // 25: Reserved
  V3C_RSVD_26,       // 26: Reserved
  V3C_RSVD_27,       // 27: Reserved
  V3C_RSVD_28,       // 28: Reserved
  V3C_RSVD_29,       // 29: Reserved
  V3C_RSVD_30,       // 30: Reserved
  V3C_RSVD_31,       // 32: Reserved
  NUM_V3C_UNIT_TYPE  // 33: undefined
};

enum PCCCodecGroup {
  CODEC_GROUP_AVC_PROGRESSIVE_HIGH = 0,
  CODEC_GROUP_HEVC_MAIN10          = 1,
  CODEC_GROUP_HEVC444              = 2,
  CODEC_GROUP_VVC_MAIN10           = 3,
  CODEC_GROUP_MP4RA                = 127  // => CCM SEI + oi/gi/ai codec id
};

enum PCCTileType {
  P_TILE = 0,  // 0: Inter atlas tile
  I_TILE,      // 1: Intra atlas tile
  SKIP_TILE,   // 2: SKIP atlas tile
  RESERVED_3   // 3: 3 to N (N not defined?)
};

enum PCCPatchModeITile {
  I_INTRA = 0,    //  0: Non-predicted patch mode
  I_RAW,          //  1: RAW Point Patch mode
  I_EOM,          //  2: EOM Point Patch mode
  I_RESERVED_3,   //  3: I_RESERVED Reserved modes
  I_RESERVED_4,   //  4: I_RESERVED Reserved modes
  I_RESERVED_5,   //  5: I_RESERVED Reserved modes
  I_RESERVED_6,   //  6: I_RESERVED Reserved modes
  I_RESERVED_7,   //  7: I_RESERVED Reserved modes
  I_RESERVED_8,   //  8: I_RESERVED Reserved modes
  I_RESERVED_9,   //  9: I_RESERVED Reserved modes
  I_RESERVED_10,  // 10: I_RESERVED Reserved modes
  I_RESERVED_11,  // 11: I_RESERVED Reserved modes
  I_RESERVED_12,  // 12: I_RESERVED Reserved modes
  I_RESERVED_13,  // 13: I_RESERVED Reserved modes
  I_END           // 14: Patch termination mode
};

enum PCCPatchModePTile {
  P_SKIP = 0,     //  0: Patch Skip mode
  P_MERGE,        //  1: Patch Merge mode
  P_INTER,        //  2: Inter predicted Patch mode
  P_INTRA,        //  3: Non-predicted Patch mode
  P_RAW,          //  4: RAW Point Patch mode
  P_EOM,          //  5: EOM Point Patch mode
  P_RESERVED_6,   //  6: Reserved modes
  P_RESERVED_7,   //  7: Reserved modes
  P_RESERVED_8,   //  8: Reserved modes
  P_RESERVED_9,   //  9: Reserved modes
  P_RESERVED_10,  // 10: Reserved modes
  P_RESERVED_11,  // 11: Reserved modes
  P_RESERVED_12,  // 12: Reserved modes
  P_RESERVED_13,  // 13: Reserved modes
  P_END,          // 14: Patch termination mode
};

enum PCCPatchType {
  INTRA_PATCH = 0,  // 0: intra patch
  INTER_PATCH,      // 1: inter patch
  MERGE_PATCH,      // 2: merge patch
  SKIP_PATCH,       // 3: skip patch
  RAW_PATCH,        // 4: raw patch
  EOM_PATCH,        // 5: eom patch
  END_PATCH,        // 6: end patch
  ERROR_PATCH       // 7: error patch
};

enum PCCHashPatchType {
  PROJECTED = 0,  // 0: protected
  RAW,            // 1: raw
  EOM             // 2: eom
};

enum SeiPayloadType {
  BUFFERING_PERIOD                 = 0,   //  0: buffering period
  ATLAS_FRAME_TIMING               = 1,   //  1: atlas frame timing
  FILLER_PAYLOAD                   = 2,   //  2: filler payload
  USER_DATAREGISTERED_ITUTT35      = 3,   //  3: user data RegisteredItuTT35
  USER_DATA_UNREGISTERED           = 4,   //  4: user data unregistered
  RECOVERY_POINT                   = 5,   //  5: recovery point
  NO_RECONSTRUCTION                = 6,   //  6: no reconstruction
  TIME_CODE                        = 7,   //  7: time code
  SEI_MANIFEST                     = 8,   //  8: sei manifest
  SEI_PREFIX_INDICATION            = 9,   //  9: sei prefix indication
  ACTIVE_SUB_BITSTREAMS            = 10,  // 10: active subBitstreams
  COMPONENT_CODEC_MAPPING          = 11,  // 11: component Codec mapping
  SCENE_OBJECT_INFORMATION         = 12,  // 12: scene object information m52705
  OBJECT_LABEL_INFORMATION         = 13,  // 13: Object label information
  PATCH_INFORMATION                = 14,  // 14: Patch information SEI message syntax
  VOLUMETRIC_RECTANGLE_INFORMATION = 15,  // 15: Volumetric rectangle information
  ATLAS_OBJECT_INFORMATION         = 16,  // 16: atlas information
  VIEWPORT_CAMERA_PARAMETERS       = 17,  // 17: viewport camera parameters
  VIEWPORT_POSITION                = 18,  // 18: viewport position
  DECODED_ATLAS_INFORMATION_HASH   = 19,  // 19: decoded atlas information hash
  ATTRIBUTE_TRANSFORMATION_PARAMS  = 64,  // 64: attribute transformation params
  OCCUPANCY_SYNTHESIS              = 65,  // 65: occupancy synthesis
  GEOMETRY_SMOOTHING               = 66,  // 66: geometry smoothing
  ATTRIBUTE_SMOOTHING              = 67,  // 67: attribute smoothing
  RESERVED_SEI_MESSAGE             = 68,  // 68: reserved sei message
};

enum NalUnitType {
  NAL_TRAIL_N = 0,      //  0: Coded tile of a non-TSA, non STSA trailing atlas frame ACL
  NAL_TRAIL_R,          //  1: Coded tile of a non-TSA, non STSA trailing atlas frame ACL
  NAL_TSA_N,            //  2: Coded tile of a TSA atlas frame ACL
  NAL_TSA_R,            //  3: Coded tile of a TSA atlas frame ACL
  NAL_STSA_N,           //  4: Coded tile of a STSA atlas frame ACL
  NAL_STSA_R,           //  5: Coded tile of a STSA atlas frame ACL
  NAL_RADL_N,           //  6: Coded tile of a RADL atlas frame ACL
  NAL_RADL_R,           //  7: Coded tile of a RADL atlas frame ACL
  NAL_RASL_N,           //  8: Coded tile of a RASL atlas frame ACL
  NAL_RASL_R,           //  9: Coded tile of a RASL atlas frame ACL
  NAL_SKIP_N,           // 10: Coded tile of a skipped atlas frame ACL
  NAL_SKIP_R,           // 11: Coded tile of a skipped atlas frame ACL
  NAL_RSV_ACL_N12,      // 12: Reserved non-IRAP sub-layer non-reference ACL NAL unit types ACL
  NAL_RSV_ACL_N14,      // 14: Reserved non-IRAP sub-layer non-reference ACL NAL unit types ACL
  NAL_RSV_ACL_R13,      // 13: Reserved non-IRAP sub-layer reference ACL NAL unit types ACL
  NAL_RSV_ACL_R15,      // 15: Reserved non-IRAP sub-layer reference ACL NAL unit types ACL
  NAL_BLA_W_LP,         // 16: Coded tile of a BLA atlas frame ACL
  NAL_BLA_W_RADL,       // 17: Coded tile of a BLA atlas frame ACL
  NAL_BLA_N_LP,         // 18: Coded tile of a BLA atlas frame ACL
  NAL_GBLA_W_LP,        // 19: Coded tile of a GBLA atlas frame ACL
  NAL_GBLA_W_RADL,      // 20: Coded tile of a GBLA atlas frame ACL
  NAL_GBLA_N_LP,        // 21: Coded tile of a GBLA atlas frame ACL
  NAL_IDR_W_RADL,       // 22: Coded tile of an IDR atlas frame ACL
  NAL_IDR_N_LP,         // 23: Coded tile of an IDR atlas frame ACL
  NAL_GIDR_W_RADL,      // 24: Coded tile of a GIDR atlas frame ACL
  NAL_GIDR_N_LP,        // 25: Coded tile of a GIDR atlas frame ACL
  NAL_CRA,              // 26: Coded tile of a CRA atlas frame ACL
  NAL_GCRA,             // 27: Coded tile of a GCRA atlas frame ACL
  NAL_RSV_IRAP_ACL_28,  // 28: Reserved IRAP ACL NAL unit types ACL
  NAL_RSV_IRAP_ACL_29,  // 29: Reserved IRAP ACL NAL unit types ACL
  NAL_RSV_ACL_30,       // 30: Reserved non-IRAP ACL NAL unit types ACL
  NAL_RSV_ACL_31,       // 31: Reserved non-IRAP ACL NAL unit types ACL
  NAL_RSV_ACL_32,       // 32: Reserved non-IRAP ACL NAL unit types ACL
  NAL_RSV_ACL_33,       // 33: Reserved non-IRAP ACL NAL unit types ACL
  NAL_RSV_ACL_34,       // 34: Reserved non-IRAP ACL NAL unit types ACL
  NAL_RSV_ACL_35,       // 35: Reserved non-IRAP ACL NAL unit types ACL
  NAL_ASPS,             // 36: Atlas sequence parameter set non-ACL
  NAL_AFPS,             // 37: Atlas frame parameter set non-ACL
  NAL_AUD,              // 38: Access unit delimiter non-ACL
  NAL_V3C_AUD,          // 39: V3C access unit delimiter non-ACL
  NAL_EOS,              // 40: End of sequence non-ACL
  NAL_EOB,              // 41: End of bitstream non-ACL
  NAL_FD,               // 42: Filler non-ACL
  NAL_PREFIX_NSEI,      // 43: Non-essential supplemental enhancement information non-ACL
  NAL_SUFFIX_NSEI,      // 44: Non-essential supplemental enhancement information non-ACL
  NAL_PREFIX_ESEI,      // 45: Essential supplemental enhancement information non-ACL
  NAL_SUFFIX_ESEI,      // 46: Essential supplemental enhancement information non-ACL
  NAL_AAPS,             // 47: Atlas adaptation parameter set non-ACL
  NAL_RSV_NACL_48,      // 48:  Reserved non-ACL NAL unit types non-ACL
  NAL_RSV_NACL_49,      // 49: Reserved non-ACL NAL unit types non-ACL
  NAL_RSV_NACL_50,      // 50: Reserved non-ACL NAL unit types non-ACL
  NAL_RSV_NACL_51,      // 51: Reserved non-ACL NAL unit types non-ACL
  NAL_RSV_NACL_52,      // 52: Reserved non-ACL NAL unit types non-ACL
  NAL_UNSPEC_53,        // 53: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_54,        // 54: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_55,        // 55: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_56,        // 56: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_57,        // 57: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_58,        // 58: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_59,        // 59: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_60,        // 60: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_61,        // 61: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_62,        // 62: Unspecified non-ACL NAL unit types non-ACL
  NAL_UNSPEC_63         // 63: Unspecified non-ACL NAL unit types non-ACL
};

// ******************************************************************* //
// Static functions
// ******************************************************************* //
static inline PCCPatchType getPatchType( PCCTileType tileType, uint8_t patchMode ) {
  if ( tileType == SKIP_TILE ) {
    return SKIP_PATCH;
  } else if ( tileType == P_TILE ) {
    if ( patchMode == P_SKIP ) {
      return SKIP_PATCH;
    } else if ( patchMode == P_MERGE ) {
      return MERGE_PATCH;
    } else if ( patchMode == P_INTRA ) {
      return INTRA_PATCH;
    } else if ( patchMode == P_INTER ) {
      return INTER_PATCH;
    } else if ( patchMode == P_RAW ) {
      return RAW_PATCH;
    } else if ( patchMode == P_EOM ) {
      return EOM_PATCH;
    }
  } else if ( tileType == I_TILE ) {
    if ( patchMode == I_INTRA ) {
      return INTRA_PATCH;
    } else if ( patchMode == I_RAW ) {
      return RAW_PATCH;
    } else if ( patchMode == I_EOM ) {
      return EOM_PATCH;
    }
  }
  return ERROR_PATCH;
}

static inline std::string toString( NalUnitType type ) {
  switch ( type ) {
    case NAL_ASPS: return std::string( "NAL_ASPS" ); break;
    case NAL_AFPS: return std::string( "NAL_AFPS" ); break;
    case NAL_AUD: return std::string( "NAL_AUD" ); break;
    case NAL_TRAIL_N: return std::string( "NAL_TRAIL_N" ); break;
    case NAL_TRAIL_R: return std::string( "NAL_TRAIL_R" ); break;
    case NAL_TSA_N: return std::string( "NAL_TSA_N" ); break;
    case NAL_TSA_R: return std::string( "NAL_TSA_R" ); break;
    case NAL_STSA_N: return std::string( "NAL_STSA_N" ); break;
    case NAL_STSA_R: return std::string( "NAL_STSA_R" ); break;
    case NAL_RADL_N: return std::string( "NAL_RADL_N" ); break;
    case NAL_RADL_R: return std::string( "NAL_RADL_R" ); break;
    case NAL_RASL_N: return std::string( "NAL_RASL_N" ); break;
    case NAL_RASL_R: return std::string( "NAL_RASL_R" ); break;
    case NAL_SKIP_N: return std::string( "NAL_SKIP_N" ); break;
    case NAL_SKIP_R: return std::string( "NAL_SKIP_R" ); break;
    case NAL_IDR_N_LP: return std::string( "NAL_IDR_N_LP" ); break;
    case NAL_PREFIX_ESEI: return std::string( "NAL_PREFIX_ESEI" ); break;
    case NAL_PREFIX_NSEI: return std::string( "NAL_PREFIX_NSEI" ); break;
    case NAL_SUFFIX_ESEI: return std::string( "NAL_SUFFIX_ESEI" ); break;
    case NAL_SUFFIX_NSEI: return std::string( "NAL_SUFFIX_NSEI" ); break;
    default: return std::string( "others" ); break;
  }
}

static inline std::string toString( V3CUnitType type ) {
  switch ( type ) {
    case V3C_VPS: return std::string( "V3C_VPS" ); break;
    case V3C_AD: return std::string( "V3C_AD" ); break;
    case V3C_OVD: return std::string( "V3C_OVD" ); break;
    case V3C_GVD: return std::string( "V3C_GVD" ); break;
    case V3C_AVD: return std::string( "V3C_AVD" ); break;
    default: return std::string( "reserved?" ); break;
  }
}
static inline std::string toString( PCCVideoType type ) {
  size_t typeIndex = (size_t)type;
  if ( typeIndex == (size_t)VIDEO_OCCUPANCY )
    return std::string( "occupancy map video " );
  else if ( typeIndex == (size_t)VIDEO_GEOMETRY )
    return std::string( "geometry video " );
  else if ( typeIndex >= (size_t)VIDEO_GEOMETRY_D0 && typeIndex <= (size_t)VIDEO_GEOMETRY_D15 )
    return std::string( "geometry D" ) + std::to_string( typeIndex - (size_t)VIDEO_GEOMETRY_D0 ) +
           std::string( " video " );
  else if ( typeIndex == (size_t)VIDEO_GEOMETRY_RAW )
    return std::string( "raw points geometry video " );
  else if ( typeIndex == (size_t)VIDEO_ATTRIBUTE )
    return std::string( "attribute video " );
  else if ( typeIndex >= (size_t)VIDEO_ATTRIBUTE_T0 && typeIndex <= (size_t)VIDEO_ATTRIBUTE_T15 )
    return std::string( "attribute T" ) + std::to_string( typeIndex - (size_t)VIDEO_ATTRIBUTE_T0 ) +
           std::string( " video " );
  else if ( typeIndex == (size_t)VIDEO_ATTRIBUTE_RAW )
    return std::string( "raw points attribute video " );
  else
    return std::string( "not supported" );
}

static inline std::string toString( SeiPayloadType payloadType ) {
  switch ( payloadType ) {
    case BUFFERING_PERIOD: return std::string( "BUFFERING_PERIOD" ); break;
    case ATLAS_FRAME_TIMING: return std::string( "ATLAS_FRAME_TIMING" ); break;
    case FILLER_PAYLOAD: return std::string( "FILLER_PAYLOAD" ); break;
    case USER_DATAREGISTERED_ITUTT35: return std::string( "USER_DATAREGISTERED_ITUTT35" ); break;
    case USER_DATA_UNREGISTERED: return std::string( "USER_DATA_UNREGISTERED" ); break;
    case RECOVERY_POINT: return std::string( "RECOVERY_POINT" ); break;
    case NO_RECONSTRUCTION: return std::string( "NO_RECONSTRUCTION" ); break;
    case TIME_CODE: return std::string( "TIME_CODE" ); break;
    case SEI_MANIFEST: return std::string( "SEI_MANIFEST" ); break;
    case SEI_PREFIX_INDICATION: return std::string( "SEI_PREFIX_INDICATION" ); break;
    case ACTIVE_SUB_BITSTREAMS: return std::string( "ACTIVE_SUB_BITSTREAMS" ); break;
    case COMPONENT_CODEC_MAPPING: return std::string( "COMPONENT_CODEC_MAPPING" ); break;
    case SCENE_OBJECT_INFORMATION: return std::string( "SCENE_OBJECT_INFORMATION" ); break;
    case OBJECT_LABEL_INFORMATION: return std::string( "OBJECT_LABEL_INFORMATION" ); break;
    case PATCH_INFORMATION: return std::string( "PATCH_INFORMATION" ); break;
    case VOLUMETRIC_RECTANGLE_INFORMATION: return std::string( "VOLUMETRIC_RECTANGLE_INFORMATION" ); break;
    case ATLAS_OBJECT_INFORMATION: return std::string( "ATLAS_OBJECT_INFORMATION" ); break;
    case VIEWPORT_CAMERA_PARAMETERS: return std::string( "VIEWPORT_CAMERA_PARAMETERS" ); break;
    case VIEWPORT_POSITION: return std::string( "VIEWPORT_POSITION" ); break;
    case DECODED_ATLAS_INFORMATION_HASH: return std::string( "DECODED_ATLAS_INFORMATION_HASH" ); break;
    case ATTRIBUTE_TRANSFORMATION_PARAMS: return std::string( "ATTRIBUTE_TRANSFORMATION_PARAMS" ); break;
    case OCCUPANCY_SYNTHESIS: return std::string( "OCCUPANCY_SYNTHESIS" ); break;
    case GEOMETRY_SMOOTHING: return std::string( "GEOMETRY_SMOOTHING" ); break;
    case ATTRIBUTE_SMOOTHING: return std::string( "ATTRIBUTE_SMOOTHING" ); break;
    case RESERVED_SEI_MESSAGE: return std::string( "RESERVED_SEI_MESSAGE" ); break;
    default: return std::string( "others" ); break;
  }
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

#ifndef _WIN32
static char getSeparator() { return '/'; }
#else
static char getSeparator() { return '\\'; }
#endif

static char getSeparator( const std::string& eFilename ) {
  auto pos1 = eFilename.find_last_of( '/' ), pos2 = eFilename.find_last_of( '\\' );
  auto pos = ( std::max )( pos1 != std::string::npos ? pos1 : 0, pos2 != std::string::npos ? pos2 : 0 );
  return ( pos != 0 ? eFilename[pos] : getSeparator() );
}

static inline std::string removeFileExtension( const std::string string ) {
  size_t pos = string.find_last_of( "." );
  return pos != std::string::npos ? string.substr( 0, pos ) : string;
}

static inline std::string getDirectoryName( const std::string& string ) {
  auto position = string.find_last_of( getSeparator( string ) );
  if ( position != std::string::npos ) { return string.substr( 0, position ); }
  return string;
}

static inline std::string getBasename( const std::string& string ) {
  auto position = string.find_last_of( getSeparator() );
  if ( position != std::string::npos ) { return string.substr( position + 1, string.length() ); }
  return string;
}

static inline std::string addVideoFormat( const std::string filename,
                                          const size_t      width,
                                          const size_t      height,
                                          const bool        isYUV = true,
                                          const bool        is420 = true,
                                          const std::string pixel = "8" ) {
  std::stringstream result;
  result << filename << "_" << width << "x" << height << "_" << pixel << "bit_"
         << ( ( isYUV & is420 ) ? "p420" : "p444" ) << ( isYUV ? ".yuv" : ".rgb" );
  return result.str();
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

static inline int floorLog2( uint32_t x ) {
  if ( x == 0 ) {
    // note: ceilLog2() expects -1 as return value
    return -1;
  }
#ifdef __GNUC__
  return 31 - __builtin_clz( x );
#else
#ifdef _MSC_VER
  unsigned long r = 0;
  _BitScanReverse( &r, x );
  return r;
#else
  int result = 0;
  if ( x & 0xffff0000 ) {
    x >>= 16;
    result += 16;
  }
  if ( x & 0xff00 ) {
    x >>= 8;
    result += 8;
  }
  if ( x & 0xf0 ) {
    x >>= 4;
    result += 4;
  }
  if ( x & 0xc ) {
    x >>= 2;
    result += 2;
  }
  if ( x & 0x2 ) {
    x >>= 1;
    result += 1;
  }
  return result;
#endif
#endif
}

static inline int ceilLog2( uint32_t x ) { return ( x == 0 ) ? -1 : floorLog2( x - 1 ) + 1; }
}  // namespace pcc

#include "PCCLogger.h"

#endif /* PCC_BITSTREAM_COMMON_H */
