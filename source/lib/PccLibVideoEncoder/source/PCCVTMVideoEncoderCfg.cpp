/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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

/** \file     EncAppCfg.cpp
    \brief    Handle encoder configuration parameters
*/

#include "PCCCommon.h"

#ifdef USE_VTMLIB_VIDEO_CODEC
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <string>
#include <fstream>
#include <limits>

//#include "program_options_lite.h"
#include "CommonLib/Rom.h"
#include "EncoderLib/RateCtrl.h"

#include "PCCVTMLibVideoEncoderCfg.h"

#include "CommonLib/dtrace_next.h"
#include "CommonLib/ProfileLevelTier.h"

#define MACRO_TO_STRING_HELPER( val ) #val
#define MACRO_TO_STRING( val ) MACRO_TO_STRING_HELPER( val )

using namespace std;
namespace po = df::program_options_lite;

enum ExtendedProfileName  // this is used for determining profile strings, where multiple profiles map to a single
                          // profile idc with various constraint flag combinations
{ NONE,
  MAIN_10,
  MAIN_10_STILL_PICTURE,
  MAIN_10_444,
  MAIN_10_444_STILL_PICTURE,
  MULTILAYER_MAIN_10,
  MULTILAYER_MAIN_10_STILL_PICTURE,
  MULTILAYER_MAIN_10_444,
  MULTILAYER_MAIN_10_444_STILL_PICTURE,
  AUTO = -1 };

//! \ingroup EncoderApp
//! \{

// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

PCCVTMLibVideoEncoderCfg::PCCVTMLibVideoEncoderCfg() :
    m_inputColourSpaceConvert( IPCOLOURSPACE_UNCHANGED ),
    m_snrInternalColourSpace( false ),
    m_outputInternalColourSpace( false ),
    m_packedYUVMode( false )
#if EXTENSION_360_VIDEO
    ,
    m_ext360( *this )
#endif
{
  m_aidQP = NULL;
}

PCCVTMLibVideoEncoderCfg::~PCCVTMLibVideoEncoderCfg() {
  if ( m_aidQP ) { delete[] m_aidQP; }

#if ENABLE_TRACING
  tracing_uninit( g_trace_ctx );
#endif
}

void PCCVTMLibVideoEncoderCfg::create() {}

void PCCVTMLibVideoEncoderCfg::destroy() {}

std::istringstream& operator>>( std::istringstream& in, GOPEntry& entry )  // input
{
  in >> entry.m_sliceType;
  in >> entry.m_POC;
  in >> entry.m_QPOffset;
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  in >> entry.m_QPOffsetModelOffset;
  in >> entry.m_QPOffsetModelScale;
#endif
#if W0038_CQP_ADJ
  in >> entry.m_CbQPoffset;
  in >> entry.m_CrQPoffset;
#endif
  in >> entry.m_QPFactor;
  in >> entry.m_tcOffsetDiv2;
  in >> entry.m_betaOffsetDiv2;
  in >> entry.m_CbTcOffsetDiv2;
  in >> entry.m_CbBetaOffsetDiv2;
  in >> entry.m_CrTcOffsetDiv2;
  in >> entry.m_CrBetaOffsetDiv2;
  in >> entry.m_temporalId;
  in >> entry.m_numRefPicsActive0;
  in >> entry.m_numRefPics0;
  for ( int i = 0; i < entry.m_numRefPics0; i++ ) { in >> entry.m_deltaRefPics0[i]; }
  in >> entry.m_numRefPicsActive1;
  in >> entry.m_numRefPics1;
  for ( int i = 0; i < entry.m_numRefPics1; i++ ) { in >> entry.m_deltaRefPics1[i]; }

  return in;
}

bool confirmPara( bool bflag, const char* message );

static inline ChromaFormat numberToChromaFormat( const int val ) {
  switch ( val ) {
    case 400: return CHROMA_400; break;
    case 420: return CHROMA_420; break;
    case 422: return CHROMA_422; break;
    case 444: return CHROMA_444; break;
    default: return NUM_CHROMA_FORMAT;
  }
}

static const struct MapStrToProfile {
  const char*   str;
  Profile::Name value;
} strToProfile[] = {
    {"none", Profile::NONE},
    {"main_10", Profile::MAIN_10},
    {"main_10_444", Profile::MAIN_10_444},
    {"main_10_still_picture", Profile::MAIN_10_STILL_PICTURE},
    {"main_10_444_still_picture", Profile::MAIN_10_444_STILL_PICTURE},
    {"multilayer_main_10", Profile::MULTILAYER_MAIN_10},
    {"multilayer_main_10_444", Profile::MULTILAYER_MAIN_10_444},
    {"multilayer_main_10_still_picture", Profile::MULTILAYER_MAIN_10_STILL_PICTURE},
    {"multilayer_main_10_444_still_picture", Profile::MULTILAYER_MAIN_10_444_STILL_PICTURE},
};

static const struct MapStrToExtendedProfile {
  const char*         str;
  ExtendedProfileName value;
} strToExtendedProfile[] = {
    {"none", NONE},
    {"main_10", MAIN_10},
    {"main_10_444", MAIN_10_444},
    {"main_10_still_picture", MAIN_10_STILL_PICTURE},
    {"main_10_444_still_picture", MAIN_10_444_STILL_PICTURE},
    {"multilayer_main_10", MULTILAYER_MAIN_10},
    {"multilayer_main_10_444", MULTILAYER_MAIN_10_444},
    {"multilayer_main_10_still_picture", MULTILAYER_MAIN_10_STILL_PICTURE},
    {"multilayer_main_10_444_still_picture", MULTILAYER_MAIN_10_444_STILL_PICTURE},
    {"auto", AUTO},
};

static const struct MapStrToTier {
  const char* str;
  Level::Tier value;
} strToTier[] = {
    {"main", Level::MAIN},
    {"high", Level::HIGH},
};

static const struct MapStrToLevel {
  const char* str;
  Level::Name value;
} strToLevel[] = {
    {"none", Level::NONE},    {"1", Level::LEVEL1},     {"2", Level::LEVEL2},     {"2.1", Level::LEVEL2_1},
    {"3", Level::LEVEL3},     {"3.1", Level::LEVEL3_1}, {"4", Level::LEVEL4},     {"4.1", Level::LEVEL4_1},
    {"5", Level::LEVEL5},     {"5.1", Level::LEVEL5_1}, {"5.2", Level::LEVEL5_2}, {"6", Level::LEVEL6},
    {"6.1", Level::LEVEL6_1}, {"6.2", Level::LEVEL6_2}, {"6.3", Level::LEVEL6_3}, {"15.5", Level::LEVEL15_5},
};

#if U0132_TARGET_BITS_SATURATION
uint32_t g_uiMaxCpbSize[2][21] = {
    //         LEVEL1,        LEVEL2,LEVEL2_1,     LEVEL3, LEVEL3_1,      LEVEL4, LEVEL4_1,       LEVEL5,  LEVEL5_1,
    //         LEVEL5_2,    LEVEL6,  LEVEL6_1,  LEVEL6_2
    {0, 0,        0,        350000, 0,        0,        1500000,  3000000,  0,         6000000,  10000000,
     0, 12000000, 20000000, 0,      25000000, 40000000, 60000000, 60000000, 120000000, 240000000},
    {0, 0,        0,        0, 0,         0,         0,         0,         0,         0,        0,
     0, 30000000, 50000000, 0, 100000000, 160000000, 240000000, 240000000, 480000000, 800000000}};
#endif

static const struct MapStrToCostMode {
  const char* str;
  CostMode    value;
} strToCostMode[] = {{"lossy", COST_STANDARD_LOSSY},
                     {"sequence_level_lossless", COST_SEQUENCE_LEVEL_LOSSLESS},
                     {"lossless", COST_LOSSLESS_CODING},
                     {"mixed_lossless_lossy", COST_MIXED_LOSSLESS_LOSSY_CODING}};

static const struct MapStrToScalingListMode {
  const char*     str;
  ScalingListMode value;
} strToScalingListMode[] = {{"0", SCALING_LIST_OFF},           {"1", SCALING_LIST_DEFAULT},
                            {"2", SCALING_LIST_FILE_READ},     {"off", SCALING_LIST_OFF},
                            {"default", SCALING_LIST_DEFAULT}, {"file", SCALING_LIST_FILE_READ}};

template <typename T, typename P>
static std::string enumToString( P map[], uint32_t mapLen, const T val ) {
  for ( uint32_t i = 0; i < mapLen; i++ ) {
    if ( val == map[i].value ) { return map[i].str; }
  }
  return std::string();
}

template <typename T, typename P>
static istream& readStrToEnum( P map[], uint32_t mapLen, istream& in, T& val ) {
  string str;
  in >> str;

  for ( uint32_t i = 0; i < mapLen; i++ ) {
    if ( str == map[i].str ) {
      val = map[i].value;
      goto found;
    }
  }
  /* not found */
  in.setstate( ios::failbit );
found:
  return in;
}

// inline to prevent compiler warnings for "unused static function"

static inline istream& operator>>( istream& in, ExtendedProfileName& profile ) {
  return readStrToEnum( strToExtendedProfile, sizeof( strToExtendedProfile ) / sizeof( *strToExtendedProfile ), in,
                        profile );
}

namespace Level {
static inline istream& operator>>( istream& in, Tier& tier ) {
  return readStrToEnum( strToTier, sizeof( strToTier ) / sizeof( *strToTier ), in, tier );
}

static inline istream& operator>>( istream& in, Name& level ) {
  return readStrToEnum( strToLevel, sizeof( strToLevel ) / sizeof( *strToLevel ), in, level );
}
}  // namespace Level

static inline istream& operator>>( istream& in, CostMode& mode ) {
  return readStrToEnum( strToCostMode, sizeof( strToCostMode ) / sizeof( *strToCostMode ), in, mode );
}

static inline istream& operator>>( istream& in, ScalingListMode& mode ) {
  return readStrToEnum( strToScalingListMode, sizeof( strToScalingListMode ) / sizeof( *strToScalingListMode ), in,
                        mode );
}

template <class T>
struct SMultiValueInput {
  static_assert( !std::is_same<T, uint8_t>::value, "SMultiValueInput<uint8_t> is not supported" );
  static_assert( !std::is_same<T, int8_t>::value, "SMultiValueInput<int8_t> is not supported" );
  const T           minValIncl;
  const T           maxValIncl;
  const std::size_t minNumValuesIncl;
  const std::size_t maxNumValuesIncl;  // Use 0 for unlimited
  std::vector<T>    values;
  SMultiValueInput() : minValIncl( 0 ), maxValIncl( 0 ), minNumValuesIncl( 0 ), maxNumValuesIncl( 0 ), values() {}
  SMultiValueInput( std::vector<T>& defaults ) :
      minValIncl( 0 ),
      maxValIncl( 0 ),
      minNumValuesIncl( 0 ),
      maxNumValuesIncl( 0 ),
      values( defaults ) {}
  SMultiValueInput( const T&    minValue,
                    const T&    maxValue,
                    std::size_t minNumberValues = 0,
                    std::size_t maxNumberValues = 0 ) :
      minValIncl( minValue ),
      maxValIncl( maxValue ),
      minNumValuesIncl( minNumberValues ),
      maxNumValuesIncl( maxNumberValues ),
      values() {}
  SMultiValueInput( const T&       minValue,
                    const T&       maxValue,
                    std::size_t    minNumberValues,
                    std::size_t    maxNumberValues,
                    const T*       defValues,
                    const uint32_t numDefValues ) :
      minValIncl( minValue ),
      maxValIncl( maxValue ),
      minNumValuesIncl( minNumberValues ),
      maxNumValuesIncl( maxNumberValues ),
      values( defValues, defValues + numDefValues ) {}
  SMultiValueInput<T>& operator=( const std::vector<T>& userValues ) {
    values = userValues;
    return *this;
  }
  SMultiValueInput<T>& operator=( const SMultiValueInput<T>& userValues ) {
    values = userValues.values;
    return *this;
  }

  T readValue( const char*& pStr, bool& bSuccess );

  istream& readValues( std::istream& in );
};

template <class T>
static inline istream& operator>>( std::istream& in, SMultiValueInput<T>& values ) {
  return values.readValues( in );
}

template <class T>
T SMultiValueInput<T>::readValue( const char*& pStr, bool& bSuccess ) {
  T           val = T();
  std::string s( pStr );
  std::replace( s.begin(), s.end(), ',', ' ' );  // make comma separated into space separated
  std::istringstream iss( s );
  iss >> val;
  bSuccess = !iss.fail()                                   // check nothing has gone wrong
             && !( val < minValIncl || val > maxValIncl )  // check value is within range
             && (int)iss.tellg() != 0                      // check we've actually read something
             && ( iss.eof() || iss.peek() == ' ' );        // check next character is a space, or eof
  pStr += ( iss.eof() ? s.size() : (std::size_t)iss.tellg() );
  return val;
}

template <class T>
istream& SMultiValueInput<T>::readValues( std::istream& in ) {
  values.clear();
  string str;
  while ( !in.eof() ) {
    string tmp;
    in >> tmp;
    str += " " + tmp;
  }
  if ( !str.empty() ) {
    const char* pStr = str.c_str();
    // soak up any whitespace
    for ( ; isspace( *pStr ); pStr++ )
      ;

    while ( *pStr != 0 ) {
      bool bSuccess = true;
      T    val      = readValue( pStr, bSuccess );
      if ( !bSuccess ) {
        in.setstate( ios::failbit );
        break;
      }

      if ( maxNumValuesIncl != 0 && values.size() >= maxNumValuesIncl ) {
        in.setstate( ios::failbit );
        break;
      }
      values.push_back( val );
      // soak up any whitespace and up to 1 comma.
      for ( ; isspace( *pStr ); pStr++ )
        ;
      if ( *pStr == ',' ) { pStr++; }
      for ( ; isspace( *pStr ); pStr++ )
        ;
    }
  }
  if ( values.size() < minNumValuesIncl ) { in.setstate( ios::failbit ); }
  return in;
}

#if QP_SWITCHING_FOR_PARALLEL
template <class T>
static inline istream& operator>>( std::istream& in, PCCVTMLibVideoEncoderCfg::OptionalValue<T>& value ) {
  in >> std::ws;
  if ( in.eof() ) {
    value.bPresent = false;
  } else {
    in >> value.value;
    value.bPresent = true;
  }
  return in;
}
#endif

template <class T1, class T2>
static inline istream& operator>>( std::istream& in, std::map<T1, T2>& map ) {
  T1 key;
  T2 value;
  try {
    in >> key;
    in >> value;
  } catch ( ... ) { in.setstate( ios::failbit ); }

  map[key] = value;
  return in;
}

static uint32_t getMaxTileColsByLevel( Level::Name level ) {
  switch ( level ) {
    case Level::LEVEL1:
    case Level::LEVEL2:
    case Level::LEVEL2_1: return 1;
    case Level::LEVEL3: return 2;
    case Level::LEVEL3_1: return 3;
    case Level::LEVEL4:
    case Level::LEVEL4_1: return 5;
    case Level::LEVEL5:
    case Level::LEVEL5_1:
    case Level::LEVEL5_2: return 10;
    case Level::LEVEL6:
    case Level::LEVEL6_1:
    case Level::LEVEL6_2: return 20;
    case Level::LEVEL6_3: return 30;
    default: return MAX_TILE_COLS;
  }
}

static uint32_t getMaxTileRowsByLevel( Level::Name level ) {
  switch ( level ) {
    case Level::LEVEL1:
    case Level::LEVEL2:
    case Level::LEVEL2_1: return 1;
    case Level::LEVEL3: return 2;
    case Level::LEVEL3_1: return 3;
    case Level::LEVEL4:
    case Level::LEVEL4_1: return 5;
    case Level::LEVEL5:
    case Level::LEVEL5_1:
    case Level::LEVEL5_2: return 11;
    case Level::LEVEL6:
    case Level::LEVEL6_1:
    case Level::LEVEL6_2: return 22;
    case Level::LEVEL6_3: return 33;
    default: return MAX_TILES / MAX_TILE_COLS;
  }
}

static uint32_t getMaxSlicesByLevel( Level::Name level ) {
  switch ( level ) {
    case Level::LEVEL1:
    case Level::LEVEL2: return 16;
    case Level::LEVEL2_1: return 20;
    case Level::LEVEL3: return 30;
    case Level::LEVEL3_1: return 40;
    case Level::LEVEL4:
    case Level::LEVEL4_1: return 75;
    case Level::LEVEL5:
    case Level::LEVEL5_1:
    case Level::LEVEL5_2: return 200;
    case Level::LEVEL6:
    case Level::LEVEL6_1:
    case Level::LEVEL6_2: return 600;
    case Level::LEVEL6_3: return 1000;
    default: return MAX_SLICES;
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** \param  argc        number of arguments
    \param  argv        array of arguments
    \retval             true when success
 */
bool PCCVTMLibVideoEncoderCfg::parseCfg( int argc, char* argv[] ) {
  bool do_help = false;

  int                 tmpChromaFormat;
  int                 tmpInputChromaFormat;
  int                 tmpConstraintChromaFormat;
  int                 tmpWeightedPredictionMethod;
  int                 tmpFastInterSearchMode;
  int                 tmpMotionEstimationSearchMethod;
  int                 tmpDecodedPictureHashSEIMappedType;
  int                 tmpSubpicDecodedPictureHashMappedType;
  string              inputColourSpaceConvert;
  string              inputPathPrefix;
  ExtendedProfileName extendedProfile;

  // Multi-value input fields:                                // minval, maxval (incl), min_entries, max_entries (incl)
  // [, default values, number of default values]
  SMultiValueInput<uint32_t> cfgTileColumnWidth( 0, std::numeric_limits<uint32_t>::max(), 0,
                                                 std::numeric_limits<uint32_t>::max() );
  SMultiValueInput<uint32_t> cfgTileRowHeight( 0, std::numeric_limits<uint32_t>::max(), 0,
                                               std::numeric_limits<uint32_t>::max() );
  SMultiValueInput<uint32_t> cfgRectSlicePos( 0, std::numeric_limits<uint32_t>::max(), 0,
                                              std::numeric_limits<uint32_t>::max() );
  SMultiValueInput<uint32_t> cfgRasterSliceSize( 0, std::numeric_limits<uint32_t>::max(), 0,
                                                 std::numeric_limits<uint32_t>::max() );
  SMultiValueInput<int> cfg_startOfCodedInterval( std::numeric_limits<int>::min(), std::numeric_limits<int>::max(), 0,
                                                  1 << 16 );
  SMultiValueInput<int> cfg_codedPivotValue( std::numeric_limits<int>::min(), std::numeric_limits<int>::max(), 0,
                                             1 << 16 );
  SMultiValueInput<int> cfg_targetPivotValue( std::numeric_limits<int>::min(), std::numeric_limits<int>::max(), 0,
                                              1 << 16 );

  SMultiValueInput<double> cfg_adIntraLambdaModifier(
      0, std::numeric_limits<double>::max(), 0,
      MAX_TLAYER );  ///< Lambda modifier for Intra pictures, one for each temporal layer. If size>temporalLayer, then
                     ///< use [temporalLayer], else if size>0, use [size()-1], else use m_adLambdaModifier.
  SMultiValueInput<uint16_t> cfgSliceLosslessArray( 0, std::numeric_limits<uint16_t>::max(), 0, MAX_SLICES );
#if SHARP_LUMA_DELTA_QP
  const int             defaultLumaLevelTodQp_QpChangePoints[]   = {-3, -2, -1, 0, 1, 2, 3, 4, 5, 6};
  const int             defaultLumaLevelTodQp_LumaChangePoints[] = {0, 301, 367, 434, 501, 567, 634, 701, 767, 834};
  SMultiValueInput<int> cfg_lumaLeveltoDQPMappingQP( -MAX_QP, MAX_QP, 0, LUMA_LEVEL_TO_DQP_LUT_MAXSIZE,
                                                     defaultLumaLevelTodQp_QpChangePoints,
                                                     sizeof( defaultLumaLevelTodQp_QpChangePoints ) / sizeof( int ) );
  SMultiValueInput<int> cfg_lumaLeveltoDQPMappingLuma(
      0, std::numeric_limits<int>::max(), 0, LUMA_LEVEL_TO_DQP_LUT_MAXSIZE, defaultLumaLevelTodQp_LumaChangePoints,
      sizeof( defaultLumaLevelTodQp_LumaChangePoints ) / sizeof( int ) );
  uint32_t lumaLevelToDeltaQPMode;
#endif
  const int qpInVals[]  = {25, 33, 43};  // qpInVal values used to derive the chroma QP mapping table used in VTM-5.0
  const int qpOutVals[] = {25, 32, 37};  // qpOutVal values used to derive the chroma QP mapping table used in VTM-5.0
  SMultiValueInput<int> cfg_qpInValCb( MIN_QP_VALUE_FOR_16_BIT, MAX_QP, 0, MAX_NUM_QP_VALUES, qpInVals,
                                       sizeof( qpInVals ) / sizeof( int ) );
  SMultiValueInput<int> cfg_qpOutValCb( MIN_QP_VALUE_FOR_16_BIT, MAX_QP, 0, MAX_NUM_QP_VALUES, qpOutVals,
                                        sizeof( qpOutVals ) / sizeof( int ) );
  const int             zeroVector[] = {0};
  SMultiValueInput<int> cfg_qpInValCr( MIN_QP_VALUE_FOR_16_BIT, MAX_QP, 0, MAX_NUM_QP_VALUES, zeroVector, 1 );
  SMultiValueInput<int> cfg_qpOutValCr( MIN_QP_VALUE_FOR_16_BIT, MAX_QP, 0, MAX_NUM_QP_VALUES, zeroVector, 1 );
  SMultiValueInput<int> cfg_qpInValCbCr( MIN_QP_VALUE_FOR_16_BIT, MAX_QP, 0, MAX_NUM_QP_VALUES, zeroVector, 1 );
  SMultiValueInput<int> cfg_qpOutValCbCr( MIN_QP_VALUE_FOR_16_BIT, MAX_QP, 0, MAX_NUM_QP_VALUES, zeroVector, 1 );
  const int             cQpOffsets[] = {6};
  SMultiValueInput<int> cfg_cbQpOffsetList( -12, 12, 0, 6, cQpOffsets, 0 );
  SMultiValueInput<int> cfg_crQpOffsetList( -12, 12, 0, 6, cQpOffsets, 0 );
  SMultiValueInput<int> cfg_cbCrQpOffsetList( -12, 12, 0, 6, cQpOffsets, 0 );

  const uint32_t             defaultInputKneeCodes[3]  = {600, 800, 900};
  const uint32_t             defaultOutputKneeCodes[3] = {100, 250, 450};
  SMultiValueInput<uint32_t> cfg_kneeSEIInputKneePointValue( 1, 999, 0, 999, defaultInputKneeCodes,
                                                             sizeof( defaultInputKneeCodes ) / sizeof( uint32_t ) );
  SMultiValueInput<uint32_t> cfg_kneeSEIOutputKneePointValue( 0, 1000, 0, 999, defaultOutputKneeCodes,
                                                              sizeof( defaultOutputKneeCodes ) / sizeof( uint32_t ) );
  const int                  defaultPrimaryCodes[6]   = {0, 50000, 0, 0, 50000, 0};
  const int                  defaultWhitePointCode[2] = {16667, 16667};
  SMultiValueInput<int>      cfg_DisplayPrimariesCode( 0, 50000, 6, 6, defaultPrimaryCodes,
                                                  sizeof( defaultPrimaryCodes ) / sizeof( int ) );
  SMultiValueInput<int>      cfg_DisplayWhitePointCode( 0, 50000, 2, 2, defaultWhitePointCode,
                                                   sizeof( defaultWhitePointCode ) / sizeof( int ) );

  SMultiValueInput<bool>     cfg_timeCodeSeiTimeStampFlag( 0, 1, 0, MAX_TIMECODE_SEI_SETS );
  SMultiValueInput<bool>     cfg_timeCodeSeiNumUnitFieldBasedFlag( 0, 1, 0, MAX_TIMECODE_SEI_SETS );
  SMultiValueInput<int>      cfg_timeCodeSeiCountingType( 0, 6, 0, MAX_TIMECODE_SEI_SETS );
  SMultiValueInput<bool>     cfg_timeCodeSeiFullTimeStampFlag( 0, 1, 0, MAX_TIMECODE_SEI_SETS );
  SMultiValueInput<bool>     cfg_timeCodeSeiDiscontinuityFlag( 0, 1, 0, MAX_TIMECODE_SEI_SETS );
  SMultiValueInput<bool>     cfg_timeCodeSeiCntDroppedFlag( 0, 1, 0, MAX_TIMECODE_SEI_SETS );
  SMultiValueInput<int>      cfg_timeCodeSeiNumberOfFrames( 0, 511, 0, MAX_TIMECODE_SEI_SETS );
  SMultiValueInput<int>      cfg_timeCodeSeiSecondsValue( 0, 59, 0, MAX_TIMECODE_SEI_SETS );
  SMultiValueInput<int>      cfg_timeCodeSeiMinutesValue( 0, 59, 0, MAX_TIMECODE_SEI_SETS );
  SMultiValueInput<int>      cfg_timeCodeSeiHoursValue( 0, 23, 0, MAX_TIMECODE_SEI_SETS );
  SMultiValueInput<bool>     cfg_timeCodeSeiSecondsFlag( 0, 1, 0, MAX_TIMECODE_SEI_SETS );
  SMultiValueInput<bool>     cfg_timeCodeSeiMinutesFlag( 0, 1, 0, MAX_TIMECODE_SEI_SETS );
  SMultiValueInput<bool>     cfg_timeCodeSeiHoursFlag( 0, 1, 0, MAX_TIMECODE_SEI_SETS );
  SMultiValueInput<int>      cfg_timeCodeSeiTimeOffsetLength( 0, 31, 0, MAX_TIMECODE_SEI_SETS );
  SMultiValueInput<int>      cfg_timeCodeSeiTimeOffsetValue( std::numeric_limits<int>::min(),
                                                             std::numeric_limits<int>::max(), 0, MAX_TIMECODE_SEI_SETS );
  SMultiValueInput<int>      cfg_omniViewportSEIAzimuthCentre( -11796480, 11796479, 0, 15 );
  SMultiValueInput<int>      cfg_omniViewportSEIElevationCentre( -5898240, 5898240, 0, 15 );
  SMultiValueInput<int>      cfg_omniViewportSEITiltCentre( -11796480, 11796479, 0, 15 );
  SMultiValueInput<uint32_t> cfg_omniViewportSEIHorRange( 1, 23592960, 0, 15 );
  SMultiValueInput<uint32_t> cfg_omniViewportSEIVerRange( 1, 11796480, 0, 15 );
  SMultiValueInput<uint32_t> cfg_rwpSEIRwpTransformType( 0, 7, 0, std::numeric_limits<uint8_t>::max() );
  SMultiValueInput<bool>     cfg_rwpSEIRwpGuardBandFlag( 0, 1, 0, std::numeric_limits<uint8_t>::max() );
  SMultiValueInput<uint32_t> cfg_rwpSEIProjRegionWidth( 0, std::numeric_limits<uint32_t>::max(), 0,
                                                        std::numeric_limits<uint8_t>::max() );
  SMultiValueInput<uint32_t> cfg_rwpSEIProjRegionHeight( 0, std::numeric_limits<uint32_t>::max(), 0,
                                                         std::numeric_limits<uint8_t>::max() );
  SMultiValueInput<uint32_t> cfg_rwpSEIRwpSEIProjRegionTop( 0, std::numeric_limits<uint32_t>::max(), 0,
                                                            std::numeric_limits<uint8_t>::max() );
  SMultiValueInput<uint32_t> cfg_rwpSEIProjRegionLeft( 0, std::numeric_limits<uint32_t>::max(), 0,
                                                       std::numeric_limits<uint8_t>::max() );
  SMultiValueInput<uint32_t> cfg_rwpSEIPackedRegionWidth( 0, std::numeric_limits<uint16_t>::max(), 0,
                                                          std::numeric_limits<uint8_t>::max() );
  SMultiValueInput<uint32_t> cfg_rwpSEIPackedRegionHeight( 0, std::numeric_limits<uint16_t>::max(), 0,
                                                           std::numeric_limits<uint8_t>::max() );
  SMultiValueInput<uint32_t> cfg_rwpSEIPackedRegionTop( 0, std::numeric_limits<uint16_t>::max(), 0,
                                                        std::numeric_limits<uint8_t>::max() );
  SMultiValueInput<uint32_t> cfg_rwpSEIPackedRegionLeft( 0, std::numeric_limits<uint16_t>::max(), 0,
                                                         std::numeric_limits<uint8_t>::max() );
  SMultiValueInput<uint32_t> cfg_rwpSEIRwpLeftGuardBandWidth( 0, std::numeric_limits<uint8_t>::max(), 0,
                                                              std::numeric_limits<uint8_t>::max() );
  SMultiValueInput<uint32_t> cfg_rwpSEIRwpRightGuardBandWidth( 0, std::numeric_limits<uint8_t>::max(), 0,
                                                               std::numeric_limits<uint8_t>::max() );
  SMultiValueInput<uint32_t> cfg_rwpSEIRwpTopGuardBandHeight( 0, std::numeric_limits<uint8_t>::max(), 0,
                                                              std::numeric_limits<uint8_t>::max() );
  SMultiValueInput<uint32_t> cfg_rwpSEIRwpBottomGuardBandHeight( 0, std::numeric_limits<uint8_t>::max(), 0,
                                                                 std::numeric_limits<uint8_t>::max() );
  SMultiValueInput<bool>     cfg_rwpSEIRwpGuardBandNotUsedForPredFlag( 0, 1, 0, std::numeric_limits<uint8_t>::max() );
  SMultiValueInput<uint32_t> cfg_rwpSEIRwpGuardBandType( 0, 7, 0, 4 * std::numeric_limits<uint8_t>::max() );
  SMultiValueInput<uint32_t> cfg_gcmpSEIFaceIndex( 0, 5, 5, 6 );
  SMultiValueInput<uint32_t> cfg_gcmpSEIFaceRotation( 0, 3, 5, 6 );
  SMultiValueInput<double>   cfg_gcmpSEIFunctionCoeffU( 0.0, 1.0, 5, 6 );
  SMultiValueInput<uint32_t> cfg_gcmpSEIFunctionUAffectedByVFlag( 0, 1, 5, 6 );
  SMultiValueInput<double>   cfg_gcmpSEIFunctionCoeffV( 0.0, 1.0, 5, 6 );
  SMultiValueInput<uint32_t> cfg_gcmpSEIFunctionVAffectedByUFlag( 0, 1, 5, 6 );
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  const int             defaultLadfQpOffset[3]           = {1, 0, 1};
  const int             defaultLadfIntervalLowerBound[2] = {350, 833};
  SMultiValueInput<int> cfg_LadfQpOffset( -MAX_QP, MAX_QP, 2, MAX_LADF_INTERVALS, defaultLadfQpOffset, 3 );
  SMultiValueInput<int> cfg_LadfIntervalLowerBound( 0, std::numeric_limits<int>::max(), 1, MAX_LADF_INTERVALS - 1,
                                                    defaultLadfIntervalLowerBound, 2 );
#endif
  SMultiValueInput<unsigned> cfg_virtualBoundariesPosX( 0, std::numeric_limits<uint32_t>::max(), 0, 3 );
  SMultiValueInput<unsigned> cfg_virtualBoundariesPosY( 0, std::numeric_limits<uint32_t>::max(), 0, 3 );

  SMultiValueInput<uint32_t> cfg_SubProfile( 0, std::numeric_limits<uint8_t>::max(), 0,
                                             std::numeric_limits<uint8_t>::max() );
  SMultiValueInput<uint32_t> cfg_subPicCtuTopLeftX( 0, std::numeric_limits<uint32_t>::max(), 0, MAX_NUM_SUB_PICS );
  SMultiValueInput<uint32_t> cfg_subPicCtuTopLeftY( 0, std::numeric_limits<uint32_t>::max(), 0, MAX_NUM_SUB_PICS );
  SMultiValueInput<uint32_t> cfg_subPicWidth( 1, std::numeric_limits<uint32_t>::max(), 0, MAX_NUM_SUB_PICS );
  SMultiValueInput<uint32_t> cfg_subPicHeight( 1, std::numeric_limits<uint32_t>::max(), 0, MAX_NUM_SUB_PICS );
  SMultiValueInput<bool>     cfg_subPicTreatedAsPicFlag( 0, 1, 0, MAX_NUM_SUB_PICS );
  SMultiValueInput<bool>     cfg_loopFilterAcrossSubpicEnabledFlag( 0, 1, 0, MAX_NUM_SUB_PICS );
  SMultiValueInput<uint32_t> cfg_subPicId( 0, std::numeric_limits<uint16_t>::max(), 0, MAX_NUM_SUB_PICS );

  SMultiValueInput<int> cfg_sliFractions( 0, 100, 0, std::numeric_limits<int>::max() );
  SMultiValueInput<int> cfg_sliNonSubpicLayersFractions( 0, 100, 0, std::numeric_limits<int>::max() );

  SMultiValueInput<Level::Name> cfg_sliRefLevels( Level::NONE, Level::LEVEL15_5, 0, 8 * MAX_VPS_SUBLAYERS );

  int warnUnknowParameter = 0;

#if ENABLE_TRACING
  string sTracingRule;
  string sTracingFile;
  bool   bTracingChannelsList = false;
#endif
#if ENABLE_SIMD_OPT
  std::string ignore;
#endif

  bool sdr = false;

  // clang-format off
  po::Options opts;
  opts.addOptions()
  ("help",                                            do_help,                                          false, "this help text")
  ("c",    po::parseConfigFile, "configuration file name")
  ("WarnUnknowParameter,w",                           warnUnknowParameter,                                  0, "warn for unknown configuration parameters instead of failing")
  ("isSDR",                                           sdr,                                              false, "compatibility")
#if ENABLE_SIMD_OPT
  ("SIMD",                                            ignore,                                      string(""), "SIMD extension to use (SCALAR, SSE41, SSE42, AVX, AVX2, AVX512), default: the highest supported extension\n")
#endif
  // File, I/O and source parameters
  ("InputFile,i",                                     m_inputFileName,                             string(""), "Original YUV input file name")
  ("InputPathPrefix,-ipp",                            inputPathPrefix,                             string(""), "pathname to prepend to input filename")
  ("BitstreamFile,b",                                 m_bitstreamFileName,                         string(""), "Bitstream output file name")
  ("ReconFile,o",                                     m_reconFileName,                             string(""), "Reconstructed YUV output file name")
#if PATCH_BASED_MVP || PCC_ME_EXT
  ("UsePccMotionEstimation",                          m_usePCCExt,                                      false, "Use modified motion estimation for PCC content")
	  ("BlockToPatchFile",                            m_blockToPatchFileName,                      string(""), "Input block to patch file name")
	  ("OccupancyMapFile",                            m_occupancyMapFileName,                      string(""), "Input occupancy map file name")
	  ("PatchInfoFile",                               m_patchInfoFileName,                         string(""), "Input patch info file name")
#endif
  ("SourceWidth,-wdt",                                m_sourceWidth,                                       0, "Source picture width")
  ("SourceHeight,-hgt",                               m_sourceHeight,                                      0, "Source picture height")
  ("InputBitDepth",                                   m_inputBitDepth[CHANNEL_TYPE_LUMA],                   8, "Bit-depth of input file")
  ("OutputBitDepth",                                  m_outputBitDepth[CHANNEL_TYPE_LUMA],                  0, "Bit-depth of output file (default:InternalBitDepth)")
  ("MSBExtendedBitDepth",                             m_MSBExtendedBitDepth[CHANNEL_TYPE_LUMA],             0, "bit depth of luma component after addition of MSBs of value 0 (used for synthesising High Dynamic Range source material). (default:InputBitDepth)")
  ("InternalBitDepth",                                m_internalBitDepth[CHANNEL_TYPE_LUMA],                0, "Bit-depth the codec operates at. (default: MSBExtendedBitDepth). If different to MSBExtendedBitDepth, source data will be converted")
  ("InputBitDepthC",                                  m_inputBitDepth[CHANNEL_TYPE_CHROMA],                 0, "As per InputBitDepth but for chroma component. (default:InputBitDepth)")
  ("OutputBitDepthC",                                 m_outputBitDepth[CHANNEL_TYPE_CHROMA],                0, "As per OutputBitDepth but for chroma component. (default: use luma output bit-depth)")
  ("MSBExtendedBitDepthC",                            m_MSBExtendedBitDepth[CHANNEL_TYPE_CHROMA],           0, "As per MSBExtendedBitDepth but for chroma component. (default:MSBExtendedBitDepth)")
  ("ExtendedPrecision",                               m_extendedPrecisionProcessingFlag,                false, "Increased internal accuracies to support high bit depths (not valid in V1 profiles)")
#if JVET_V0054_TSRC_RICE
  ("TSRCRicePresent",                                 m_tsrcRicePresentFlag,                            false, "Indicate that TSRC Rice information is present in slice header (not valid in V1 profiles)")
#endif  
  ("HighPrecisionPredictionWeighting",                m_highPrecisionOffsetsEnabledFlag,                false, "Use high precision option for weighted prediction (not valid in V1 profiles)")
  ("InputColourSpaceConvert",                         inputColourSpaceConvert,                     string(""), "Colour space conversion to apply to input video. Permitted values are (empty string=UNCHANGED) " + getListOfColourSpaceConverts(true))
  ("SNRInternalColourSpace",                          m_snrInternalColourSpace,                         false, "If true, then no colour space conversion is applied prior to SNR, otherwise inverse of input is applied.")
  ("OutputInternalColourSpace",                       m_outputInternalColourSpace,                      false, "If true, then no colour space conversion is applied for reconstructed video, otherwise inverse of input is applied.")
  ("InputChromaFormat",                               tmpInputChromaFormat,                               420, "InputChromaFormatIDC")
  ("MSEBasedSequencePSNR",                            m_printMSEBasedSequencePSNR,                      false, "0 (default) emit sequence PSNR only as a linear average of the frame PSNRs, 1 = also emit a sequence PSNR based on an average of the frame MSEs")
  ("PrintHexPSNR",                                    m_printHexPsnr,                                   false, "0 (default) don't emit hexadecimal PSNR for each frame, 1 = also emit hexadecimal PSNR values")
  ("PrintFrameMSE",                                   m_printFrameMSE,                                  false, "0 (default) emit only bit count and PSNRs for each frame, 1 = also emit MSE values")
  ("PrintSequenceMSE",                                m_printSequenceMSE,                               false, "0 (default) emit only bit rate and PSNRs for the whole sequence, 1 = also emit MSE values")
  ("PrintMSSSIM",                                     m_printMSSSIM,                                    false, "0 (default) do not print MS-SSIM scores, 1 = print MS-SSIM scores for each frame and for the whole sequence")
  ("PrintWPSNR",                                      m_printWPSNR,                                     false, "0 (default) do not print HDR-PQ based wPSNR, 1 = print HDR-PQ based wPSNR")
  ("CabacZeroWordPaddingEnabled",                     m_cabacZeroWordPaddingEnabled,                     true, "0 do not add conforming cabac-zero-words to bit streams, 1 (default) = add cabac-zero-words as required")
  ("ChromaFormatIDC,-cf",                             tmpChromaFormat,                                      0, "ChromaFormatIDC (400|420|422|444 or set 0 (default) for same as InputChromaFormat)")
  ("ConformanceWindowMode",                           m_conformanceWindowMode,                              1, "Window conformance mode (0: no window, 1:automatic padding (default), 2:padding parameters specified, 3:conformance window parameters specified")
  ("HorizontalPadding,-pdx",                          m_sourcePadding[0],                                   0, "Horizontal source padding for conformance window mode 2")
  ("VerticalPadding,-pdy",                            m_sourcePadding[1],                                   0, "Vertical source padding for conformance window mode 2")
  ("ConfWinLeft",                                     m_confWinLeft,                                        0, "Left offset for window conformance mode 3")
  ("ConfWinRight",                                    m_confWinRight,                                       0, "Right offset for window conformance mode 3")
  ("ConfWinTop",                                      m_confWinTop,                                         0, "Top offset for window conformance mode 3")
  ("ConfWinBottom",                                   m_confWinBottom,                                      0, "Bottom offset for window conformance mode 3")
  ("AccessUnitDelimiter",                             m_AccessUnitDelimiter,                            false, "Enable Access Unit Delimiter NALUs")
  ("EnablePictureHeaderInSliceHeader",                m_enablePictureHeaderInSliceHeader,                true, "Enable Picture Header in Slice Header")
  ("FrameRate,-fr",                                   m_iFrameRate,                                         0, "Frame rate")
  ("FrameSkip,-fs",                                   m_FrameSkip,                                         0u, "Number of frames to skip at start of input YUV")
  ("TemporalSubsampleRatio,-ts",                      m_temporalSubsampleRatio,                            1u, "Temporal sub-sample ratio when reading input YUV")
  ("FramesToBeEncoded,f",                             m_framesToBeEncoded,                                  0, "Number of frames to be encoded (default=all)")
  ("ClipInputVideoToRec709Range",                     m_bClipInputVideoToRec709Range,                   false, "If true then clip input video to the Rec. 709 Range on loading when InternalBitDepth is less than MSBExtendedBitDepth")
  ("ClipOutputVideoToRec709Range",                    m_bClipOutputVideoToRec709Range,                  false, "If true then clip output video to the Rec. 709 Range on saving when OutputBitDepth is less than InternalBitDepth")
  ("PYUV",                                            m_packedYUVMode,                                  false, "If true then output 10-bit and 12-bit YUV data as 5-byte and 3-byte (respectively) packed YUV data. Ignored for interlaced output.")
  ("SummaryOutFilename",                              m_summaryOutFilename,                          string(), "Filename to use for producing summary output file. If empty, do not produce a file.")
  ("SummaryPicFilenameBase",                          m_summaryPicFilenameBase,                      string(), "Base filename to use for producing summary picture output files. The actual filenames used will have I.txt, P.txt and B.txt appended. If empty, do not produce a file.")
  ("SummaryVerboseness",                              m_summaryVerboseness,                                0u, "Specifies the level of the verboseness of the text output")
  ("Verbosity,v",                                     m_verbosity,                               (int)VTM_VERBOSE, "Specifies the level of the verboseness")

#if JVET_O0756_CONFIG_HDRMETRICS || JVET_O0756_CALCULATE_HDRMETRICS
  ( "WhitePointDeltaE1",                              m_whitePointDeltaE[0],                            100.0, "1st reference white point value")
  ( "WhitePointDeltaE2",                              m_whitePointDeltaE[1],                           1000.0, "2nd reference white point value")
  ( "WhitePointDeltaE3",                              m_whitePointDeltaE[2],                           5000.0, "3rd reference white point value")
  ( "MaxSampleValue",                                 m_maxSampleValue,                               10000.0, "Maximum sample value for floats")
  ( "InputSampleRange",                               m_sampleRange,                                        0, "Sample Range")
  ( "InputColorPrimaries",                            m_colorPrimaries,                                     1, "Input Color Primaries")
  ( "EnableTFunctionLUT",                             m_enableTFunctionLUT,                             false, "Input Color Primaries")
  ( "ChromaLocation",                                 m_chromaLocation,                                     2, "Location of Chroma Samples")
  ( "ChromaUpsampleFilter",                           m_chromaUPFilter,                                     1, "420 to 444 conversion filters")
  ( "CropOffsetLeft",                                 m_cropOffsetLeft,                                     0, "Crop Offset Left position")
  ( "CropOffsetTop",                                  m_cropOffsetTop,                                      0, "Crop Offset Top position")
  ( "CropOffsetRight",                                m_cropOffsetRight,                                    0, "Crop Offset Right position")
  ( "CropOffsetBottom",                               m_cropOffsetBottom,                                   0, "Crop Offset Bottom position")
  ( "CalculateHdrMetrics",                            m_calculateHdrMetrics,                            false, "Enable HDR metric calculation")
#endif

  //Field coding parameters
  ("FieldCoding",                                     m_isField,                                        false, "Signals if it's a field based coding")
  ("TopFieldFirst, Tff",                              m_isTopFieldFirst,                                false, "In case of field based coding, signals whether if it's a top field first or not")
  ("EfficientFieldIRAPEnabled",                       m_efficientFieldIRAPEnabled,                      true, "Enable to code fields in a specific, potentially more efficient, order.")
  ("HarmonizeGopFirstFieldCoupleEnabled",             m_harmonizeGopFirstFieldCoupleEnabled,            true, "Enables harmonization of Gop first field couple")

  // Profile and level
  ("Profile",                                         extendedProfile,              ExtendedProfileName::NONE, "Profile name to use for encoding. Use [multilayer_]main_10[_444][_still_picture], auto, or none")
  ("Level",                                           m_level,                                    Level::NONE, "Level limit to be used, eg 5.1, or none")
  ("Tier",                                            m_levelTier,                                Level::MAIN, "Tier to use for interpretation of --Level (main or high only)")
  ("FrameOnlyConstraintFlag",                         m_frameOnlyConstraintFlag,                        true, "Bitstream contains only frames")
  ("MultiLayerEnabledFlag",                           m_multiLayerEnabledFlag,                         false, "Bitstream might contain more than one layer")
  ("SubProfile",                                      cfg_SubProfile,                          cfg_SubProfile,  "Sub-profile idc")
  ("EnableDecodingCapabilityInformation",             m_DCIEnabled,                                     false, "Enables writing of Decoding Capability Information")
  ("MaxBitDepthConstraint",                           m_bitDepthConstraint,                                0u, "Bit depth to use for profile-constraint for RExt profiles. 0=automatically choose based upon other parameters")
  ("MaxChromaFormatConstraint",                       tmpConstraintChromaFormat,                            0, "Chroma-format to use for the profile-constraint for RExt profiles. 0=automatically choose based upon other parameters")

  ("GciPresentFlag",                                  m_gciPresentFlag,                                 false, "GCI field present")
  ("IntraOnlyConstraintFlag",                         m_intraOnlyConstraintFlag,                        false, "Value of intra_only_constraint_flag")
  ("AllLayersIndependentConstraintFlag",              m_allLayersIndependentConstraintFlag,             false, "Indicate that all layers are independent")
  ("OnePictureOnlyConstraintFlag",                    m_onePictureOnlyConstraintFlag,                   false, "Value of general_intra_constraint_flag. Can only be used for single frame encodings. Will be set to true for still picture profiles")
  ("MaxBitDepthConstraintIdc",                        m_maxBitDepthConstraintIdc,                          16u, "Indicate that sps_bitdepth_minus8 plus 8 shall be in the range of 0 to m_maxBitDepthConstraintIdc")
  ("MaxChromaFormatConstraintIdc",                    m_maxChromaFormatConstraintIdc,                        3, "Indicate that sps_chroma_format_idc shall be in the range of 0 to m_maxChromaFormatConstraintIdc")
  ("NoTrailConstraintFlag",                           m_noTrailConstraintFlag,                          false, "Indicate that TRAIL is deactivated")
  ("NoStsaConstraintFlag",                            m_noStsaConstraintFlag,                           false, "Indicate that STSA is deactivated")
  ("NoRaslConstraintFlag",                            m_noRaslConstraintFlag,                           false, "Indicate that RSAL is deactivated")
  ("NoRadlConstraintFlag",                            m_noRadlConstraintFlag,                           false, "Indicate that RADL is deactivated")
  ("NoIdrConstraintFlag",                             m_noIdrConstraintFlag,                            false, "Indicate that IDR is deactivated")
  ("NoCraConstraintFlag",                             m_noCraConstraintFlag,                            false, "Indicate that CRA is deactivated")
  ("NoGdrConstraintFlag",                             m_noGdrConstraintFlag,                            false, "Indicate that GDR is deactivated")
  ("NoApsConstraintFlag",                             m_noApsConstraintFlag,                            false, "Indicate that APS is deactivated")
  ("OneTilePerPicConstraintFlag",                     m_oneTilePerPicConstraintFlag,                    false, "Indicate that each picture shall contain only one tile")
  ("PicHeaderInSliceHeaderConstraintFlag",            m_picHeaderInSliceHeaderConstraintFlag,           false, "Indicate that picture header is present in slice header")
  ("OneSlicePerPicConstraintFlag",                    m_oneSlicePerPicConstraintFlag,                   false, "Indicate that each picture shall contain only one slice")
  ("NoIdrRplConstraintFlag",                          m_noIdrRplConstraintFlag,                         false, "Indicate that RPL is not present in SH of IDR slices")
  ("NoRectSliceConstraintFlag",                       m_noRectSliceConstraintFlag,                      false, "Indicate that rectagular slice is deactivated")
  ("OneSlicePerSubpicConstraintFlag",                 m_oneSlicePerSubpicConstraintFlag,                false, "Indicate that each subpicture shall contain only one slice")
  ("NoSubpicInfoConstraintFlag",                      m_noSubpicInfoConstraintFlag,                     false, "Indicate that subpicture information is not present")
  ("MaxLog2CtuSizeConstraintIdc",                     m_maxLog2CtuSizeConstraintIdc,                        8, "Indicate that Log2CtuSize shall be in the range of 0 to m_maxLog2CtuSizeConstraintIdc")
  ("NoPartitionConstraintsOverrideConstraintFlag",    m_noPartitionConstraintsOverrideConstraintFlag,   false, "Indicate that Partition Override is deactivated")
  ("MttConstraintFlag",                               m_noMttConstraintFlag,                            false, "Indicate that Mtt is deactivated")
  ("NoQtbttDualTreeIntraConstraintFlag",              m_noQtbttDualTreeIntraConstraintFlag,            false, "Indicate that Qtbtt DualTree Intra is deactivated")
  ("NoPaletteConstraintFlag",                         m_noPaletteConstraintFlag,                        false, "Indicate that PLT is deactivated")
  ("NoIbcConstraintFlag",                             m_noIbcConstraintFlag,                            false, "Indicate that IBC is deactivated")
  ("NoIspConstraintFlag",                             m_noIspConstraintFlag,                            false, "Indicate that ISP is deactivated")
  ("NoMrlConstraintFlag",                             m_noMrlConstraintFlag,                            false, "Indicate that MRL is deactivated")
  ("NoMipConstraintFlag",                             m_noMipConstraintFlag,                            false, "Indicate that MIP is deactivated")
  ("NoCclmConstraintFlag",                            m_noCclmConstraintFlag,                          false, "Indicate that CCLM is deactivated")
  ("NoRprConstraintFlag",                             m_noRprConstraintFlag,                            false, "Indicate that reference picture resampling is deactivated")
  ("NoResChangeInClvsConstraintFlag",                 m_noResChangeInClvsConstraintFlag,                false, "Indicate that the picture spatial resolution does not change within any CLVS referring to the SPS")
  ("WeightedPredictionConstraintFlag",                m_noWeightedPredictionConstraintFlag,             false, "Indicate that Weighted Prediction is deactivated")
  ("NoRefWraparoundConstraintFlag",                   m_noRefWraparoundConstraintFlag,                 false, "Indicate that Reference Wraparound is deactivated")
  ("NoTemporalMvpConstraintFlag",                     m_noTemporalMvpConstraintFlag,                   false, "Indicate that temporal MVP is deactivated")
  ("NoSbtmvpConstraintFlag",                          m_noSbtmvpConstraintFlag,                        false, "Indicate that SbTMVP is deactivated")
  ("NoAmvrConstraintFlag",                            m_noAmvrConstraintFlag,                          false, "Indicate that AMVR is deactivated")
  ("NoSmvdConstraintFlag",                            m_noSmvdConstraintFlag,                           false, "Indicate that SMVD is deactivated")
  ("NoBdofConstraintFlag",                            m_noBdofConstraintFlag,                          false, "Indicate that BIO is deactivated")
  ("NoDmvrConstraintFlag",                            m_noDmvrConstraintFlag,                           false, "Indicate that DMVR is deactivated")
  ("NoMmvdConstraintFlag",                            m_noMmvdConstraintFlag,                           false, "Indicate that MMVD is deactivated")
  ("NoAffineMotionConstraintFlag",                    m_noAffineMotionConstraintFlag,                  false, "Indicate that Affine is deactivated")
  ("NoProfConstraintFlag",                            m_noProfConstraintFlag,                           false, "Indicate that PROF is deactivated")
  ("NoBcwConstraintFlag",                             m_noBcwConstraintFlag,                           false, "Indicate that BCW is deactivated")
  ("NoCiipConstraintFlag",                            m_noCiipConstraintFlag,                          false, "Indicate that CIIP is deactivated")
  ("NoGpmConstraintFlag",                             m_noGeoConstraintFlag,                            false, "Indicate that GPM is deactivated")
  ("NoTransformSkipConstraintFlag",                   m_noTransformSkipConstraintFlag,                  false, "Indicate that Transform Skip is deactivated")
  ("NoLumaTransformSize64ConstraintFlag",             m_noLumaTransformSize64ConstraintFlag,            false, "Indicate that Luma Transform Size 64 is deactivated")
  ("NoBDPCMConstraintFlag",                           m_noBDPCMConstraintFlag,                          false, "Indicate that BDPCM is deactivated")
  ("NoMtsConstraintFlag",                             m_noMtsConstraintFlag,                           false, "Indicate that MTS is deactivated")
  ("NoLfnstConstraintFlag",                           m_noLfnstConstraintFlag,                          false, "Indicate that LFNST is deactivated")
  ("NoJointCbCrConstraintFlag",                       m_noJointCbCrConstraintFlag,                      false, "Indicate that JCCR is deactivated")
  ("NoSbtConstraintFlag",                             m_noSbtConstraintFlag,                            false, "Indicate that SBT is deactivated")
  ("NoActConstraintFlag",                             m_noActConstraintFlag,                            false, "Indicate that ACT is deactivated")
  ("NoExplicitScaleListConstraintFlag",               m_noExplicitScaleListConstraintFlag,              false, "Indicate that explicit scaling list is deactivated")
  ("NoChromaQpOffsetConstraintFlag",                  m_noChromaQpOffsetConstraintFlag,                 false, "Indicate that chroma qp offset is zero")
  ("NoDepQuantConstraintFlag",                        m_noDepQuantConstraintFlag,                      false, "Indicate that DQ is deactivated")
  ("NoSignDataHidingConstraintFlag",                  m_noSignDataHidingConstraintFlag,                false, "Indicate that SDH is deactivated")
  ("NoCuQpDeltaConstraintFlag",                       m_noCuQpDeltaConstraintFlag,                     false, "Indicate that CU QP delta is deactivated")
  ("NoSaoConstraintFlag",                             m_noSaoConstraintFlag,                           false, "Indicate that SAO is deactivated")
  ("NoAlfConstraintFlag",                             m_noAlfConstraintFlag,                           false, "Indicate that ALF is deactivated")
  ("NoCCAlfConstraintFlag",                           m_noCCAlfConstraintFlag,                          false, "Indicate that CCALF is deactivated")
  ("NoLmcsConstraintFlag",                            m_noLmcsConstraintFlag,                           false, "Indicate that LMCS is deactivated")
  ("NoLadfConstraintFlag",                            m_noLadfConstraintFlag,                          false, "Indicate that LADF is deactivated")
  ("NoVirtualBoundaryConstraintFlag",                 m_noVirtualBoundaryConstraintFlag,                false, "Indicate that virtual boundary is deactivated")

  ("CTUSize",                                         m_uiCTUSize,                                       128u, "CTUSize (specifies the CTU size if QTBT is on) [default: 128]")
  ("Log2MinCuSize",                                   m_log2MinCuSize,                                     2u, "Log2 min CU size")
  ("SubPicInfoPresentFlag",                           m_subPicInfoPresentFlag,                          false, "equal to 1 specifies that subpicture parameters are present in in the SPS RBSP syntax")
  ("NumSubPics",                                      m_numSubPics,                                        0u, "specifies the number of subpictures")
  ("SubPicSameSizeFlag",                              m_subPicSameSizeFlag,                             false, "equal to 1 specifies that all subpictures in the CLVS have the same width specified by sps_subpic_width_minus1[ 0 ] and the same height specified by sps_subpic_height_minus1[ 0 ].")
  ("SubPicCtuTopLeftX",                               cfg_subPicCtuTopLeftX,            cfg_subPicCtuTopLeftX, "specifies horizontal position of top left CTU of i-th subpicture in unit of CtbSizeY")
  ("SubPicCtuTopLeftY",                               cfg_subPicCtuTopLeftY,            cfg_subPicCtuTopLeftY, "specifies vertical position of top left CTU of i-th subpicture in unit of CtbSizeY")
  ("SubPicWidth",                                     cfg_subPicWidth,                        cfg_subPicWidth, "specifies the width of the i-th subpicture in units of CtbSizeY")
  ("SubPicHeight",                                    cfg_subPicHeight,                      cfg_subPicHeight, "specifies the height of the i-th subpicture in units of CtbSizeY")
  ("SubPicTreatedAsPicFlag",                          cfg_subPicTreatedAsPicFlag,  cfg_subPicTreatedAsPicFlag, "equal to 1 specifies that the i-th subpicture of each coded picture in the CLVS is treated as a picture in the decoding process excluding in-loop filtering operations")
  ("LoopFilterAcrossSubpicEnabledFlag",               cfg_loopFilterAcrossSubpicEnabledFlag, cfg_loopFilterAcrossSubpicEnabledFlag, "equal to 1 specifies that in-loop filtering operations may be performed across the boundaries of the i-th subpicture in each coded picture in the CLVS")
  ("SubPicIdMappingExplicitlySignalledFlag",          m_subPicIdMappingExplicitlySignalledFlag,         false, "equal to 1 specifies that the subpicture ID mapping is explicitly signalled, either in the SPS or in the PPSs")
  ("SubPicIdMappingInSpsFlag",                        m_subPicIdMappingInSpsFlag,                       false, "equal to 1 specifies that subpicture ID mapping is signalled in the SPS")
  ("SubPicIdLen",                                     m_subPicIdLen,                                       0u, "specifies the number of bits used to represent the syntax element sps_subpic_id[ i ]. ")
  ("SubPicId",                                        cfg_subPicId,                              cfg_subPicId, "specifies that subpicture ID of the i-th subpicture")
  ("SingleSlicePerSubpic",                            m_singleSlicePerSubPicFlag,                       false, "Enables setting of a single slice per sub-picture (no explicit configuration required)")
  ("EnablePartitionConstraintsOverride",              m_SplitConsOverrideEnabledFlag,                    true, "Enable partition constraints override")
  ("MinQTISlice",                                     m_uiMinQT[0],                                        8u, "MinQTISlice")
  ("MinQTLumaISlice",                                 m_uiMinQT[0],                                        8u, "MinQTLumaISlice")
  ("MinQTChromaISliceInChromaSamples",                m_uiMinQT[2],                                        4u, "MinQTChromaISliceInChromaSamples")
  ("MinQTNonISlice",                                  m_uiMinQT[1],                                        8u, "MinQTNonISlice")
  ("MaxMTTHierarchyDepth",                            m_uiMaxMTTHierarchyDepth,                            3u, "MaxMTTHierarchyDepth")
  ("MaxMTTHierarchyDepthI",                           m_uiMaxMTTHierarchyDepthI,                           3u, "MaxMTTHierarchyDepthI")
  ("MaxMTTHierarchyDepthISliceL",                     m_uiMaxMTTHierarchyDepthI,                           3u, "MaxMTTHierarchyDepthISliceL")
  ("MaxMTTHierarchyDepthISliceC",                     m_uiMaxMTTHierarchyDepthIChroma,                     3u, "MaxMTTHierarchyDepthISliceC")
  ("MaxBTLumaISlice",                                 m_uiMaxBT[0],                                       32u, "MaxBTLumaISlice")
  ("MaxBTChromaISlice",                               m_uiMaxBT[2],                                       64u, "MaxBTChromaISlice")
  ("MaxBTNonISlice",                                  m_uiMaxBT[1],                                      128u, "MaxBTNonISlice")
  ("MaxTTLumaISlice",                                 m_uiMaxTT[0],                                       32u, "MaxTTLumaISlice")
  ("MaxTTChromaISlice",                               m_uiMaxTT[2],                                       32u, "MaxTTChromaISlice")
  ("MaxTTNonISlice",                                  m_uiMaxTT[1],                                       64u, "MaxTTNonISlice")
  ("DualITree",                                       m_dualTree,                                       false, "Use separate QTBT trees for intra slice luma and chroma channel types")
  ( "LFNST",                                          m_LFNST,                                          false, "Enable LFNST (0:off, 1:on)  [default: off]" )
  ( "FastLFNST",                                      m_useFastLFNST,                                   false, "Fast methods for LFNST" )
  ("SbTMVP",                                          m_sbTmvpEnableFlag,                               false, "Enable Subblock Temporal Motion Vector Prediction (0: off, 1: on) [default: off]")
  ("MMVD",                                            m_MMVD,                                            true, "Enable Merge mode with Motion Vector Difference (0:off, 1:on)  [default: 1]")
  ("Affine",                                          m_Affine,                                         false, "Enable affine prediction (0:off, 1:on)  [default: off]")
  ("AffineType",                                      m_AffineType,                                      true,  "Enable affine type prediction (0:off, 1:on)  [default: on]" )
  ("PROF",                                            m_PROF,                                           false, "Enable Prediction refinement with optical flow for affine mode (0:off, 1:on)  [default: off]")
  ("BIO",                                             m_BIO,                                            false, "Enable bi-directional optical flow")
  ("IMV",                                             m_ImvMode,                                            1, "Adaptive MV precision Mode (IMV)\n"
                                                                                                               "\t0: disabled\n"
                                                                                                               "\t1: enabled (1/2-Pel, Full-Pel and 4-PEL)\n")
  ("IMV4PelFast",                                     m_Imv4PelFast,                                        1, "Fast 4-Pel Adaptive MV precision Mode 0:disabled, 1:enabled)  [default: 1]")
  ("LMChroma",                                        m_LMChroma,                                           1, " LMChroma prediction "
                                                                                                               "\t0:  Disable LMChroma\n"
                                                                                                               "\t1:  Enable LMChroma\n")
  ("HorCollocatedChroma",                             m_horCollocatedChromaFlag,                         true, "Specifies location of a chroma sample relatively to the luma sample in horizontal direction in the reference picture resampling\n"
                                                                                                               "\t0:  horizontally shifted by 0.5 units of luma samples\n"
                                                                                                               "\t1:  collocated (default)\n")
  ("VerCollocatedChroma",                             m_verCollocatedChromaFlag,                        false, "Specifies location of a chroma sample relatively to the luma sample in vertical direction in the cross-component linear model intra prediction and the reference picture resampling\n"
                                                                                                               "\t0:  horizontally co-sited, vertically shifted by 0.5 units of luma samples\n"
                                                                                                               "\t1:  collocated\n")
  ("MTS",                                             m_MTS,                                                0, "Multiple Transform Set (MTS)\n"
    "\t0:  Disable MTS\n"
    "\t1:  Enable only Intra MTS\n"
    "\t2:  Enable only Inter MTS\n"
    "\t3:  Enable both Intra & Inter MTS\n")
  ("MTSIntraMaxCand",                                 m_MTSIntraMaxCand,                                    3, "Number of additional candidates to test in encoder search for MTS in intra slices\n")
  ("MTSInterMaxCand",                                 m_MTSInterMaxCand,                                    4, "Number of additional candidates to test in encoder search for MTS in inter slices\n")
  ("MTSImplicit",                                     m_MTSImplicit,                                        0, "Enable implicit MTS (when explicit MTS is off)\n")
  ( "SBT",                                            m_SBT,                                            false, "Enable Sub-Block Transform for inter blocks\n" )
  ( "SBTFast64WidthTh",                               m_SBTFast64WidthTh,                                1920, "Picture width threshold for testing size-64 SBT in RDO (now for HD and above sequences)\n")
  ( "ISP",                                            m_ISP,                                            false, "Enable Intra Sub-Partitions\n" )
  ("SMVD",                                            m_SMVD,                                           false, "Enable Symmetric MVD\n")
  ("CompositeLTReference",                            m_compositeRefEnabled,                            false, "Enable Composite Long Term Reference Frame")
  ("BCW",                                             m_bcw,                                            false, "Enable Generalized Bi-prediction(Bcw)")
  ("BcwFast",                                         m_BcwFast,                                        false, "Fast methods for Generalized Bi-prediction(Bcw)\n")
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  ("LADF",                                            m_LadfEnabed,                                     false, "Luma adaptive deblocking filter QP Offset(L0414)")
  ("LadfNumIntervals",                                m_LadfNumIntervals,                                   3, "LADF number of intervals (2-5, inclusive)")
  ("LadfQpOffset",                                    cfg_LadfQpOffset,                      cfg_LadfQpOffset, "LADF QP offset")
  ("LadfIntervalLowerBound",                          cfg_LadfIntervalLowerBound,  cfg_LadfIntervalLowerBound, "LADF lower bound for 2nd lowest interval")
#endif
  ("CIIP",                                            m_ciip,                                           false, "Enable CIIP mode")
  ("Geo",                                             m_Geo,                                            false, "Enable geometric partitioning mode (0:off, 1:on)")
  ("HashME",                                          m_HashME,                                         false, "Enable hash motion estimation (0:off, 1:on)")

  ("AllowDisFracMMVD",                                m_allowDisFracMMVD,                               false, "Disable fractional MVD in MMVD mode adaptively")
  ("AffineAmvr",                                      m_AffineAmvr,                                     false, "Eanble AMVR for affine inter mode")
  ("AffineAmvrEncOpt",                                m_AffineAmvrEncOpt,                               false, "Enable encoder optimization of affine AMVR")
  ("DMVR",                                            m_DMVR,                                           false, "Decoder-side Motion Vector Refinement")
  ("MmvdDisNum",                                      m_MmvdDisNum,                                     8,     "Number of MMVD Distance Entries")
  ("ColorTransform",                                  m_useColorTrans,                                  false, "Enable the color transform")
  ("PLT",                                             m_PLTMode,                                           0u, "PLTMode (0x1:enabled, 0x0:disabled)  [default: disabled]")
  ("JointCbCr",                                       m_JointCbCrMode,                                  false, "Enable joint coding of chroma residuals (JointCbCr, 0:off, 1:on)")
  ( "IBC",                                            m_IBCMode,                                           0u, "IBCMode (0x1:enabled, 0x0:disabled)  [default: disabled]")
  ( "IBCLocalSearchRangeX",                           m_IBCLocalSearchRangeX,                            128u, "Search range of IBC local search in x direction")
  ( "IBCLocalSearchRangeY",                           m_IBCLocalSearchRangeY,                            128u, "Search range of IBC local search in y direction")
  ( "IBCHashSearch",                                  m_IBCHashSearch,                                     1u, "Hash based IBC search")
  ( "IBCHashSearchMaxCand",                           m_IBCHashSearchMaxCand,                            256u, "Max candidates for hash based IBC search")
  ( "IBCHashSearchRange4SmallBlk",                    m_IBCHashSearchRange4SmallBlk,                     256u, "Small block search range in based IBC search")
  ( "IBCFastMethod",                                  m_IBCFastMethod,                                     6u, "Fast methods for IBC")

  ("WrapAround",                                      m_wrapAround,                                     false, "Enable horizontal wrap-around motion compensation for inter prediction (0:off, 1:on)  [default: off]")
  ("WrapAroundOffset",                                m_wrapAroundOffset,                                  0u, "Offset in luma samples used for computing the horizontal wrap-around position")

  // ADD_NEW_TOOL : (encoder app) add parsing parameters here
  ( "VirtualBoundariesPresentInSPSFlag",              m_virtualBoundariesPresentFlag,                    true, "Virtual Boundary position information is signalled in SPS or PH (1:SPS, 0:PH)  [default: on]" )
  ("NumVerVirtualBoundaries",                         m_numVerVirtualBoundaries,                           0u, "Number of vertical virtual boundaries (0-3, inclusive)")
  ("NumHorVirtualBoundaries",                         m_numHorVirtualBoundaries,                           0u, "Number of horizontal virtual boundaries (0-3, inclusive)")
  ("VirtualBoundariesPosX",                           cfg_virtualBoundariesPosX,    cfg_virtualBoundariesPosX, "Locations of the vertical virtual boundaries in units of luma samples")
  ("VirtualBoundariesPosY",                           cfg_virtualBoundariesPosY,    cfg_virtualBoundariesPosY, "Locations of the horizontal virtual boundaries in units of luma samples")
  ("EncDbOpt",                                        m_encDbOpt,                                       false, "Encoder optimization with deblocking filter")
  ("LMCSEnable",                                      m_lmcsEnabled,                                    false, "Enable LMCS (luma mapping with chroma scaling")
  ("LMCSSignalType",                                  m_reshapeSignalType,                                 0u, "Input signal type: 0:SDR, 1:HDR-PQ, 2:HDR-HLG")
  ("LMCSUpdateCtrl",                                  m_updateCtrl,                                         0, "LMCS model update control: 0:RA, 1:AI, 2:LDB/LDP")
  ("LMCSAdpOption",                                   m_adpOption,                                          0, "LMCS adaptation options: 0:automatic(default),"
                                                                                                               "1: rsp both (CW66 for QP<=22), 2: rsp TID0 (for all QP),"
                                                                                                               "3: rsp inter(CW66 for QP<=22), 4: rsp inter(for all QP).")
  ("LMCSInitialCW",                                   m_initialCW,                                         0u, "LMCS initial total codeword (0~1023) when LMCSAdpOption > 0")
  ("LMCSOffset",                                      m_CSoffset,                                           0, "LMCS chroma residual scaling offset")
  ("IntraCMD",                                        m_intraCMD,                                          0u, "IntraChroma MD: 0: none, 1:fixed to default wPSNR weight")
  ("LCTUFast",                                        m_useFastLCTU,                                    false, "Fast methods for large CTU")
  ("FastMrg",                                         m_useFastMrg,                                     false, "Fast methods for inter merge")
  ("PBIntraFast",                                     m_usePbIntraFast,                                 false, "Fast assertion if the intra mode is probable")
  ("AMaxBT",                                          m_useAMaxBT,                                      false, "Adaptive maximal BT-size")
  ("E0023FastEnc",                                    m_e0023FastEnc,                                    true, "Fast encoding setting for QTBT (proposal E0023)")
  ("ContentBasedFastQtbt",                            m_contentBasedFastQtbt,                           false, "Signal based QTBT speed-up")
  ("UseNonLinearAlfLuma",                             m_useNonLinearAlfLuma,                             true, "Non-linear adaptive loop filters for Luma Channel")
  ("UseNonLinearAlfChroma",                           m_useNonLinearAlfChroma,                           true, "Non-linear adaptive loop filters for Chroma Channels")
  ("MaxNumAlfAlternativesChroma",                     m_maxNumAlfAlternativesChroma,
                                                                    (unsigned)MAX_NUM_ALF_ALTERNATIVES_CHROMA, std::string("Maximum number of alternative Chroma filters (1-") + std::to_string(MAX_NUM_ALF_ALTERNATIVES_CHROMA) + std::string (", inclusive)") )
  ("MRL",                                             m_MRL,                                            false,  "Enable MRL (multiple reference line intra prediction)")
  ("MIP",                                             m_MIP,                                             true,  "Enable MIP (matrix-based intra prediction)")
  ("FastMIP",                                         m_useFastMIP,                                     false,  "Fast encoder search for MIP (matrix-based intra prediction)")
  ("FastLocalDualTreeMode",                           m_fastLocalDualTreeMode,                              0,  "Fast intra pass coding for local dual-tree in intra coding region, 0: off, 1: use threshold, 2: one intra mode only")
  // Unit definition parameters
  ("MaxCUWidth",                                      m_uiMaxCUWidth,                                     64u)
  ("MaxCUHeight",                                     m_uiMaxCUHeight,                                    64u)
  // todo: remove defaults from MaxCUSize
  ("MaxCUSize,s",                                     m_uiMaxCUWidth,                                     64u, "Maximum CU size")
  ("MaxCUSize,s",                                     m_uiMaxCUHeight,                                    64u, "Maximum CU size")

  ("Log2MaxTbSize",                                   m_log2MaxTbSize,                                      6, "Maximum transform block size in logarithm base 2 (Default: 6)")

  // Coding structure paramters
  ("IntraPeriod,-ip",                                 m_iIntraPeriod,                                      -1, "Intra period in frames, (-1: only first frame)")
#if GDR_ENABLED
  ("GdrEnabled",                                      m_gdrEnabled,                                     false, "GDR enabled")
  ("GdrPocStart",                                     m_gdrPocStart,                                       -1, "GDR poc start")
  ("GdrPeriod",                                       m_gdrPeriod,                                         -1, "Number of frames between GDR picture to the next GDR picture")
  ("GdrInterval",                                     m_gdrInterval,                                       -1, "Number of frames from GDR picture to the recovery point picture")
  ("GdrNoHash",                                       m_gdrNoHash,                                       true, "Do not generate decode picture hash SEI messages for GDR and recovering pictures")
#endif
  ("DecodingRefreshType,-dr",                         m_iDecodingRefreshType,                               0, "Intra refresh type (0:none 1:CRA 2:IDR 3:RecPointSEI)")
  ("GOPSize,g",                                       m_iGOPSize,                                           1, "GOP size of temporal structure")
  ("DRAPPeriod",                                      m_drapPeriod,                                         0, "DRAP period in frames (0: disable Dependent RAP indication SEI messages)")
  ("ReWriteParamSets",                                m_rewriteParamSets,                           false, "Enable rewriting of Parameter sets before every (intra) random access point")
  ("IDRRefParamList",                                 m_idrRefParamList,                            false, "Enable indication of reference picture list syntax elements in slice headers of IDR pictures")
  // motion search options
  ("DisableIntraInInter",                             m_bDisableIntraPUsInInterSlices,                  false, "Flag to disable intra PUs in inter slices")
  ("FastSearch",                                      tmpMotionEstimationSearchMethod,  int(MESEARCH_DIAMOND), "0:Full search 1:Diamond 2:Selective 3:Enhanced Diamond")
  ("SearchRange,-sr",                                 m_iSearchRange,                                      96, "Motion search range")
  ("BipredSearchRange",                               m_bipredSearchRange,                                  4, "Motion search range for bipred refinement")
  ("MinSearchWindow",                                 m_minSearchWindow,                                    8, "Minimum motion search window size for the adaptive window ME")
  ("RestrictMESampling",                              m_bRestrictMESampling,                            false, "Restrict ME Sampling for selective inter motion search")
  ("ClipForBiPredMEEnabled",                          m_bClipForBiPredMeEnabled,                        false, "Enables clipping in the Bi-Pred ME. It is disabled to reduce encoder run-time")
  ("FastMEAssumingSmootherMVEnabled",                 m_bFastMEAssumingSmootherMVEnabled,                true, "Enables fast ME assuming a smoother MV.")

  ("HadamardME",                                      m_bUseHADME,                                       true, "Hadamard ME for fractional-pel")
  ("ASR",                                             m_bUseASR,                                        false, "Adaptive motion search range");
  opts.addOptions()

  // Mode decision parameters
  ("LambdaModifier0,-LM0",                            m_adLambdaModifier[ 0 ],                  ( double )1.0, "Lambda modifier for temporal layer 0. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifier1,-LM1",                            m_adLambdaModifier[ 1 ],                  ( double )1.0, "Lambda modifier for temporal layer 1. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifier2,-LM2",                            m_adLambdaModifier[ 2 ],                  ( double )1.0, "Lambda modifier for temporal layer 2. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifier3,-LM3",                            m_adLambdaModifier[ 3 ],                  ( double )1.0, "Lambda modifier for temporal layer 3. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifier4,-LM4",                            m_adLambdaModifier[ 4 ],                  ( double )1.0, "Lambda modifier for temporal layer 4. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifier5,-LM5",                            m_adLambdaModifier[ 5 ],                  ( double )1.0, "Lambda modifier for temporal layer 5. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifier6,-LM6",                            m_adLambdaModifier[ 6 ],                  ( double )1.0, "Lambda modifier for temporal layer 6. If LambdaModifierI is used, this will not affect intra pictures")
  ("LambdaModifierI,-LMI",                            cfg_adIntraLambdaModifier,    cfg_adIntraLambdaModifier, "Lambda modifiers for Intra pictures, comma separated, up to one the number of temporal layer. If entry for temporalLayer exists, then use it, else if some are specified, use the last, else use the standard LambdaModifiers.")
  ("IQPFactor,-IQF",                                  m_dIntraQpFactor,                                  -1.0, "Intra QP Factor for Lambda Computation. If negative, the default will scale lambda based on GOP size (unless LambdaFromQpEnable then IntraQPOffset is used instead)")

  /* Quantization parameters */
#if QP_SWITCHING_FOR_PARALLEL
  ("QP,q",                                            m_iQP,                                               30, "Qp value")
  ("QPIncrementFrame,-qpif",                          m_qpIncrementAtSourceFrame,   OptionalValue<uint32_t>(), "If a source file frame number is specified, the internal QP will be incremented for all POCs associated with source frames >= frame number. If empty, do not increment.")
#else
  ("QP,q",                                            m_fQP,                                             30.0, "Qp value, if value is float, QP is switched once during encoding")
#endif
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  ("IntraQPOffset",                                   m_intraQPOffset,                                      0, "Qp offset value for intra slice, typically determined based on GOP size")
  ("LambdaFromQpEnable",                              m_lambdaFromQPEnable,                             false, "Enable flag for derivation of lambda from QP")
#endif
  ("DeltaQpRD,-dqr",                                  m_uiDeltaQpRD,                                       0u, "max dQp offset for slice")
  ("MaxDeltaQP,d",                                    m_iMaxDeltaQP,                                        0, "max dQp offset for block")
  ("MaxCuDQPSubdiv,-dqd",                             m_cuQpDeltaSubdiv,                                    0, "Maximum subdiv for CU luma Qp adjustment")
  ("MaxCuChromaQpOffsetSubdiv",                       m_cuChromaQpOffsetSubdiv,                             0, "Maximum subdiv for CU chroma Qp adjustment")
  ("SliceCuChromaQpOffsetEnabled",                    m_cuChromaQpOffsetEnabled,                         true, "Enable local chroma QP offsets (slice level flag)")
  ("FastDeltaQP",                                     m_bFastDeltaQP,                                   false, "Fast Delta QP Algorithm")
#if SHARP_LUMA_DELTA_QP
  ("LumaLevelToDeltaQPMode",                          lumaLevelToDeltaQPMode,                              0u, "Luma based Delta QP 0(default): not used. 1: Based on CTU average, 2: Based on Max luma in CTU")
#if !WCG_EXT
  ("LumaLevelToDeltaQPMaxValWeight",                  m_lumaLevelToDeltaQPMapping.maxMethodWeight,        1.0, "Weight of block max luma val when LumaLevelToDeltaQPMode = 2")
#endif
  ("LumaLevelToDeltaQPMappingLuma",                   cfg_lumaLeveltoDQPMappingLuma,  cfg_lumaLeveltoDQPMappingLuma, "Luma to Delta QP Mapping - luma thresholds")
  ("LumaLevelToDeltaQPMappingDQP",                    cfg_lumaLeveltoDQPMappingQP,  cfg_lumaLeveltoDQPMappingQP, "Luma to Delta QP Mapping - DQP values")
#endif
#if JVET_V0078
  ("SmoothQPReductionEnable",                         m_smoothQPReductionEnable,                         false, "Enable QP reduction for smooth blocks according to: Clip3(SmoothQPReductionLimit, 0, SmoothQPReductionModelScale*baseQP+SmoothQPReductionModelOffset)")
  ("SmoothQPReductionThreshold",                      m_smoothQPReductionThreshold,                        3.0, "Threshold parameter for smoothness (SmoothQPReductionThreshold * number of samples in block)")
  ("SmoothQPReductionModelScale",                     m_smoothQPReductionModelScale,                      -1.0, "Scale parameter of the QP reduction model")
  ("SmoothQPReductionModelOffset",                    m_smoothQPReductionModelOffset,                     27.0, "Offset parameter of the QP reduction model")
  ("SmoothQPReductionLimit",                          m_smoothQPReductionLimit,                            -16, "Threshold parameter for controlling maximum amount of QP reduction by the QP reduction model")
  ("SmoothQPReductionPeriodicity",                    m_smoothQPReductionPeriodicity,                        1, "Periodicity parameter of the QP reduction model, 1: all frames, 0: only intra pictures, 2: every second frame, etc")
#endif
  ("UseIdentityTableForNon420Chroma",                 m_useIdentityTableForNon420Chroma,                 true, "True: Indicates that 422/444 chroma uses identity chroma QP mapping tables; False: explicit Qp table may be specified in config")
  ("SameCQPTablesForAllChroma",                       m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag,                        true, "0: Different tables for Cb, Cr and joint Cb-Cr components, 1 (default): Same tables for all three chroma components")
  ("QpInValCb",                                       cfg_qpInValCb,                            cfg_qpInValCb, "Input coordinates for the QP table for Cb component")
  ("QpOutValCb",                                      cfg_qpOutValCb,                          cfg_qpOutValCb, "Output coordinates for the QP table for Cb component")
  ("QpInValCr",                                       cfg_qpInValCr,                            cfg_qpInValCr, "Input coordinates for the QP table for Cr component")
  ("QpOutValCr",                                      cfg_qpOutValCr,                          cfg_qpOutValCr, "Output coordinates for the QP table for Cr component")
  ("QpInValCbCr",                                     cfg_qpInValCbCr,                        cfg_qpInValCbCr, "Input coordinates for the QP table for joint Cb-Cr component")
  ("QpOutValCbCr",                                    cfg_qpOutValCbCr,                      cfg_qpOutValCbCr, "Output coordinates for the QP table for joint Cb-Cr component")
  ("CbQpOffset,-cbqpofs",                             m_cbQpOffset,                                         0, "Chroma Cb QP Offset")
  ("CrQpOffset,-crqpofs",                             m_crQpOffset,                                         0, "Chroma Cr QP Offset")
  ("CbQpOffsetDualTree",                              m_cbQpOffsetDualTree,                                 0, "Chroma Cb QP Offset for dual tree")
  ("CrQpOffsetDualTree",                              m_crQpOffsetDualTree,                                 0, "Chroma Cr QP Offset for dual tree")
  ("CbCrQpOffset,-cbcrqpofs",                         m_cbCrQpOffset,                                      -1, "QP Offset for joint Cb-Cr mode")
  ("CbCrQpOffsetDualTree",                            m_cbCrQpOffsetDualTree,                               0, "QP Offset for joint Cb-Cr mode in dual tree")
#if ER_CHROMA_QP_WCG_PPS
  ("WCGPPSEnable",                                    m_wcgChromaQpControl.enabled,                     false, "1: Enable the WCG PPS chroma modulation scheme. 0 (default) disabled")
  ("WCGPPSCbQpScale",                                 m_wcgChromaQpControl.chromaCbQpScale,               1.0, "WCG PPS Chroma Cb QP Scale")
  ("WCGPPSCrQpScale",                                 m_wcgChromaQpControl.chromaCrQpScale,               1.0, "WCG PPS Chroma Cr QP Scale")
  ("WCGPPSChromaQpScale",                             m_wcgChromaQpControl.chromaQpScale,                 0.0, "WCG PPS Chroma QP Scale")
  ("WCGPPSChromaQpOffset",                            m_wcgChromaQpControl.chromaQpOffset,                0.0, "WCG PPS Chroma QP Offset")
#endif
#if W0038_CQP_ADJ
  ("SliceChromaQPOffsetPeriodicity",                  m_sliceChromaQpOffsetPeriodicity,                    0u, "Used in conjunction with Slice Cb/Cr QpOffsetIntraOrPeriodic. Use 0 (default) to disable periodic nature.")
  ("SliceCbQpOffsetIntraOrPeriodic",                  m_sliceChromaQpOffsetIntraOrPeriodic[0],              0, "Chroma Cb QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.")
  ("SliceCrQpOffsetIntraOrPeriodic",                  m_sliceChromaQpOffsetIntraOrPeriodic[1],              0, "Chroma Cr QP Offset at slice level for I slice or for periodic inter slices as defined by SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.")
#endif
  ("CbQpOffsetList",                                  cfg_cbQpOffsetList,                  cfg_cbQpOffsetList, "Chroma Cb QP offset list for local adjustment")
  ("CrQpOffsetList",                                  cfg_crQpOffsetList,                  cfg_crQpOffsetList, "Chroma Cb QP offset list for local adjustment")
  ("CbCrQpOffsetList",                                cfg_cbCrQpOffsetList,              cfg_cbCrQpOffsetList, "Chroma joint Cb-Cr QP offset list for local adjustment")

  ("AdaptiveQP,-aq",                                  m_bUseAdaptiveQP,                                 false, "QP adaptation based on a psycho-visual model")
  ("MaxQPAdaptationRange,-aqr",                       m_iQPAdaptationRange,                                 6, "QP adaptation range")
#if ENABLE_QPA
  ("PerceptQPA,-qpa",                                 m_bUsePerceptQPA,                                 false, "perceptually motivated input-adaptive QP modification (default: 0 = off, ignored if -aq is set)")
  ("WPSNR,-wpsnr",                                    m_bUseWPSNR,                                      false, "output perceptually weighted peak SNR (WPSNR) instead of PSNR")
#endif
  ("dQPFile,m",                                       m_dQPFileName,                               string(""), "dQP file name")
  ("RDOQ",                                            m_useRDOQ,                                         true)
  ("RDOQTS",                                          m_useRDOQTS,                                       true)
#if T0196_SELECTIVE_RDOQ
  ("SelectiveRDOQ",                                   m_useSelectiveRDOQ,                               false, "Enable selective RDOQ")
#endif
  ("RDpenalty",                                       m_rdPenalty,                                          0, "RD-penalty for 32x32 TU for intra in non-intra slices. 0:disabled  1:RD-penalty  2:maximum RD-penalty")

  // Deblocking filter parameters
  ("DeblockingFilterDisable",                         m_deblockingFilterDisable,                        false)
  ("DeblockingFilterOffsetInPPS",                     m_deblockingFilterOffsetInPPS,                     true)
  ("DeblockingFilterBetaOffset_div2",                 m_deblockingFilterBetaOffsetDiv2,                     0)
  ("DeblockingFilterTcOffset_div2",                   m_deblockingFilterTcOffsetDiv2,                       0)
  ("DeblockingFilterCbBetaOffset_div2",               m_deblockingFilterCbBetaOffsetDiv2,                   0)
  ("DeblockingFilterCbTcOffset_div2",                 m_deblockingFilterCbTcOffsetDiv2,                     0)
  ("DeblockingFilterCrBetaOffset_div2",               m_deblockingFilterCrBetaOffsetDiv2,                   0)
  ("DeblockingFilterCrTcOffset_div2",                 m_deblockingFilterCrTcOffsetDiv2,                     0)
#if W0038_DB_OPT
  ("DeblockingFilterMetric",                          m_deblockingFilterMetric,                             0)
#else
  ("DeblockingFilterMetric",                          m_DeblockingFilterMetric,                         false)
#endif
  // Coding tools
  ("ReconBasedCrossCPredictionEstimate",              m_reconBasedCrossCPredictionEstimate,             false, "When determining the alpha value for cross-component prediction, use the decoded residual rather than the pre-transform encoder-side residual")
  ("TransformSkip",                                   m_useTransformSkip,                               false, "Intra transform skipping")
  ("TransformSkipFast",                               m_useTransformSkipFast,                           false, "Fast encoder search for transform skipping, winner takes it all mode.")
  ("TransformSkipLog2MaxSize",                        m_log2MaxTransformSkipBlockSize,                     5U, "Specify transform-skip maximum size. Minimum 2, Maximum 5. (not valid in V1 profiles)")
  ("ChromaTS",                                        m_useChromaTS,                                    false, "Enable encoder search of chromaTS")
  ("BDPCM",                                           m_useBDPCM,                                       false, "BDPCM (0:off, 1:luma and chroma)")
  ("ISPFast",                                         m_useFastISP,                                     false, "Fast encoder search for ISP")
  ("ResidualRotation",                                m_transformSkipRotationEnabledFlag,               false, "Enable rotation of transform-skipped and transquant-bypassed TUs through 180 degrees prior to entropy coding (not valid in V1 profiles)")
  ("SingleSignificanceMapContext",                    m_transformSkipContextEnabledFlag,                false, "Enable, for transform-skipped and transquant-bypassed TUs, the selection of a single significance map context variable for all coefficients (not valid in V1 profiles)")
#if JVET_V0106_RRC_RICE
  ("ExtendedRiceRRC",                                 m_rrcRiceExtensionEnableFlag,                     false, "Enable the extention of the Golomb-Rice parameter derivation for RRC")
#endif
  ("GolombRiceParameterAdaptation",                   m_persistentRiceAdaptationEnabledFlag,            false, "Enable the adaptation of the Golomb-Rice parameter over the course of each slice")
  ("AlignCABACBeforeBypass",                          m_cabacBypassAlignmentEnabledFlag,                false, "Align the CABAC engine to a defined fraction of a bit prior to coding bypass data. Must be 1 in high bit rate profile, 0 otherwise")
  ("SAO",                                             m_bUseSAO,                                         true, "Enable Sample Adaptive Offset")
  ("TestSAODisableAtPictureLevel",                    m_bTestSAODisableAtPictureLevel,                  false, "Enables the testing of disabling SAO at the picture level after having analysed all blocks")
  ("SaoEncodingRate",                                 m_saoEncodingRate,                                 0.75, "When >0 SAO early picture termination is enabled for luma and chroma")
  ("SaoEncodingRateChroma",                           m_saoEncodingRateChroma,                            0.5, "The SAO early picture termination rate to use for chroma (when m_SaoEncodingRate is >0). If <=0, use results for luma")
  ("MaxNumOffsetsPerPic",                             m_maxNumOffsetsPerPic,                             2048, "Max number of SAO offset per picture (Default: 2048)")
  ("SAOLcuBoundary",                                  m_saoCtuBoundary,                                 false, "0: right/bottom CTU boundary areas skipped from SAO parameter estimation, 1: non-deblocked pixels are used for those areas")
  ("SAOGreedyEnc",                                    m_saoGreedyMergeEnc,                              false, "SAO greedy merge encoding algorithm")
  ("EnablePicPartitioning",                           m_picPartitionFlag,                               false, "Enable picture partitioning (0: single tile, single slice, 1: multiple tiles/slices can be used)")
  ("MixedLossyLossless",                              m_mixedLossyLossless,                                  false, "Enable encoder to encode mixed lossy/lossless coding ")
  ("SliceLosslessArray",                              cfgSliceLosslessArray, cfgSliceLosslessArray, " Lossless slice array Last lossless flag in the  list will be repeated uniformly to cover any remaining slice")
  ("TileColumnWidthArray",                            cfgTileColumnWidth,                  cfgTileColumnWidth, "Tile column widths in units of CTUs. Last column width in list will be repeated uniformly to cover any remaining picture width")
  ("TileRowHeightArray",                              cfgTileRowHeight,                      cfgTileRowHeight, "Tile row heights in units of CTUs. Last row height in list will be repeated uniformly to cover any remaining picture height")
  ("RasterScanSlices",                                m_rasterSliceFlag,                                false, "Indicates if using raster-scan or rectangular slices (0: rectangular, 1: raster-scan)")
  ("RectSlicePositions",                              cfgRectSlicePos,                        cfgRectSlicePos, "Rectangular slice positions. List containing pairs of top-left CTU RS address followed by bottom-right CTU RS address")
  ("RectSliceFixedWidth",                             m_rectSliceFixedWidth,                                0, "Fixed rectangular slice width in units of tiles (0: disable this feature and use RectSlicePositions instead)")
  ("RectSliceFixedHeight",                            m_rectSliceFixedHeight,                               0, "Fixed rectangular slice height in units of tiles (0: disable this feature and use RectSlicePositions instead)")
  ("RasterSliceSizes",                                cfgRasterSliceSize,                  cfgRasterSliceSize, "Raster-scan slice sizes in units of tiles. Last size in list will be repeated uniformly to cover any remaining tiles in the picture")
  ("DisableLoopFilterAcrossTiles",                    m_disableLFCrossTileBoundaryFlag,                 false, "Loop filtering applied across tile boundaries or not (0: filter across tile boundaries  1: do not filter across tile boundaries)")
  ("DisableLoopFilterAcrossSlices",                   m_disableLFCrossSliceBoundaryFlag,                false, "Loop filtering applied across slice boundaries or not (0: filter across slice boundaries 1: do not filter across slice boundaries)")
  ("FastUDIUseMPMEnabled",                            m_bFastUDIUseMPMEnabled,                           true, "If enabled, adapt intra direction search, accounting for MPM")
  ("FastMEForGenBLowDelayEnabled",                    m_bFastMEForGenBLowDelayEnabled,                   true, "If enabled use a fast ME for generalised B Low Delay slices")
  ("UseBLambdaForNonKeyLowDelayPictures",             m_bUseBLambdaForNonKeyLowDelayPictures,            true, "Enables use of B-Lambda for non-key low-delay pictures")
  ("IntraReferenceSmoothing",                         m_enableIntraReferenceSmoothing,                   true, "0: Disable use of intra reference smoothing (not valid in V1 profiles). 1: Enable use of intra reference smoothing (same as V1)")
  ("WeightedPredP,-wpP",                              m_useWeightedPred,                                false, "Use weighted prediction in P slices")
  ("WeightedPredB,-wpB",                              m_useWeightedBiPred,                              false, "Use weighted (bidirectional) prediction in B slices")
  ("WeightedPredMethod,-wpM",                         tmpWeightedPredictionMethod, int(WP_PER_PICTURE_WITH_SIMPLE_DC_COMBINED_COMPONENT), "Weighted prediction method")
  ("Log2ParallelMergeLevel",                          m_log2ParallelMergeLevel,                            2u, "Parallel merge estimation region")
  ("WaveFrontSynchro",                                m_entropyCodingSyncEnabledFlag,                   false, "0: entropy coding sync disabled; 1 entropy coding sync enabled")
  ("EntryPointsPresent",                              m_entryPointPresentFlag,                           true, "0: entry points is not present; 1 entry points may be present in slice header")
  ("ScalingList",                                     m_useScalingListId,                    SCALING_LIST_OFF, "0/off: no scaling list, 1/default: default scaling lists, 2/file: scaling lists specified in ScalingListFile")
  ("ScalingListFile",                                 m_scalingListFileName,                       string(""), "Scaling list file name. Use an empty string to produce help.")
  ("DisableScalingMatrixForLFNST",                    m_disableScalingMatrixForLfnstBlks,                true, "Disable scaling matrices, when enabled, for LFNST-coded blocks")
  ("DisableScalingMatrixForAlternativeColourSpace",   m_disableScalingMatrixForAlternativeColourSpace,  false, "Disable scaling matrices when the colour space is not equal to the designated colour space of scaling matrix")
  ("ScalingMatrixDesignatedColourSpace",              m_scalingMatrixDesignatedColourSpace,              true, "Indicates if the designated colour space of scaling matrices is equal to the original colour space")
  ("DepQuant",                                        m_depQuantEnabledFlag,                                          true, "Enable  dependent quantization (Default: 1)" )
  ("SignHideFlag,-SBH",                               m_signDataHidingEnabledFlag,                                    false,  "Enable sign hiding" )
  ("MaxNumMergeCand",                                 m_maxNumMergeCand,                                   5u, "Maximum number of merge candidates")
  ("MaxNumAffineMergeCand",                           m_maxNumAffineMergeCand,                             5u, "Maximum number of affine merge candidates")
  ("MaxNumGeoCand",                                   m_maxNumGeoCand,                                     5u, "Maximum number of geometric partitioning mode candidates")
  ("MaxNumIBCMergeCand",                              m_maxNumIBCMergeCand,                                6u, "Maximum number of IBC merge candidates")
    /* Misc. */
  ("SEIDecodedPictureHash,-dph",                      tmpDecodedPictureHashSEIMappedType,                   0, "Control generation of decode picture hash SEI messages\n"
                                                                                                               "\t3: checksum\n"
                                                                                                               "\t2: CRC\n"
                                                                                                               "\t1: use MD5\n"
                                                                                                               "\t0: disable")
  ("SubpicDecodedPictureHash",                        tmpSubpicDecodedPictureHashMappedType,                0, "Control generation of decode picture hash SEI messages for each subpicture\n"
                                                                                                               "\t3: checksum\n"
                                                                                                               "\t2: CRC\n"
                                                                                                               "\t1: use MD5\n"
                                                                                                               "\t0: disable")
  ("TMVPMode",                                        m_TMVPModeId,                                         1, "TMVP mode 0: TMVP disable for all slices. 1: TMVP enable for all slices (default) 2: TMVP enable for certain slices only")
  ("SliceLevelRpl",                                   m_sliceLevelRpl,                                   true, "Code reference picture lists in slice headers rather than picture header.")
  ("SliceLevelDblk",                                  m_sliceLevelDblk,                                  true, "Code deblocking filter parameters in slice headers rather than picture header.")
  ("SliceLevelSao",                                   m_sliceLevelSao,                                   true, "Code SAO parameters in slice headers rather than picture header.")
  ("SliceLevelAlf",                                   m_sliceLevelAlf,                                   true, "Code ALF parameters in slice headers rather than picture header.")
  ("SliceLevelWeightedPrediction",                    m_sliceLevelWp,                                    true, "Code weighted prediction parameters in slice headers rather than picture header.")
  ("SliceLevelDeltaQp",                               m_sliceLevelDeltaQp,                               true, "Code delta Qp in slice headers rather than picture header.")
  ("FEN",                                             tmpFastInterSearchMode,   int(FASTINTERSEARCH_DISABLED), "fast encoder setting")
  ("ECU",                                             m_bUseEarlyCU,                                    false, "Early CU setting")
  ("FDM",                                             m_useFastDecisionForMerge,                         true, "Fast decision for Merge RD Cost")
  ("ESD",                                             m_useEarlySkipDetection,                          false, "Early SKIP detection setting")
  ( "RateControl",                                    m_RCEnableRateControl,                            false, "Rate control: enable rate control" )
  ( "TargetBitrate",                                  m_RCTargetBitrate,                                    0, "Rate control: target bit-rate" )
  ( "KeepHierarchicalBit",                            m_RCKeepHierarchicalBit,                              0, "Rate control: 0: equal bit allocation; 1: fixed ratio bit allocation; 2: adaptive ratio bit allocation" )
  ( "LCULevelRateControl",                            m_RCLCULevelRC,                                    true, "Rate control: true: CTU level RC; false: picture level RC" )
  ( "RCLCUSeparateModel",                             m_RCUseLCUSeparateModel,                           true, "Rate control: use CTU level separate R-lambda model" )
  ( "InitialQP",                                      m_RCInitialQP,                                        0, "Rate control: initial QP" )
  ( "RCForceIntraQP",                                 m_RCForceIntraQP,                                 false, "Rate control: force intra QP to be equal to initial QP" )
#if U0132_TARGET_BITS_SATURATION
  ( "RCCpbSaturation",                                m_RCCpbSaturationEnabled,                         false, "Rate control: enable target bits saturation to avoid CPB overflow and underflow" )
  ( "RCCpbSize",                                      m_RCCpbSize,                                         0u, "Rate control: CPB size" )
  ( "RCInitialCpbFullness",                           m_RCInitialCpbFullness,                             0.9, "Rate control: initial CPB fullness" )
#endif
  ("CostMode",                                        m_costMode,                         COST_STANDARD_LOSSY, "Use alternative cost functions: choose between 'lossy', 'sequence_level_lossless', 'lossless' (which forces QP to " MACRO_TO_STRING(LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP) ") and 'mixed_lossless_lossy' (which used QP'=" MACRO_TO_STRING(LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME) " for pre-estimates of transquant-bypass blocks).")
  ("TSRCdisableLL",                                   m_TSRCdisableLL,                                   true, "Disable TSRC for lossless coding" )
  ("RecalculateQPAccordingToLambda",                  m_recalculateQPAccordingToLambda,                 false, "Recalculate QP values according to lambda values. Do not suggest to be enabled in all intra case")
  ("HrdParametersPresent,-hrd",                       m_hrdParametersPresentFlag,                       false, "Enable generation of hrd_parameters()")
  ("VuiParametersPresent,-vui",                       m_vuiParametersPresentFlag,                       false, "Enable generation of vui_parameters()")
  ("SamePicTimingInAllOLS",                           m_samePicTimingInAllOLS,                          true, "Indicates that the same picture timing SEI message is used in all OLS")
  ("AspectRatioInfoPresent",                          m_aspectRatioInfoPresentFlag,                     false, "Signals whether aspect_ratio_idc is present")
  ("AspectRatioIdc",                                  m_aspectRatioIdc,                                     0, "aspect_ratio_idc")
  ("SarWidth",                                        m_sarWidth,                                           0, "horizontal size of the sample aspect ratio")
  ("SarHeight",                                       m_sarHeight,                                          0, "vertical size of the sample aspect ratio")
  ("ColourDescriptionPresent",                        m_colourDescriptionPresentFlag,                   false, "Signals whether colour_primaries, transfer_characteristics and matrix_coefficients are present")
  ("ColourPrimaries",                                 m_colourPrimaries,                                    2, "Indicates chromaticity coordinates of the source primaries")
  ("TransferCharacteristics",                         m_transferCharacteristics,                            2, "Indicates the opto-electronic transfer characteristics of the source")
  ("MatrixCoefficients",                              m_matrixCoefficients,                                 2, "Describes the matrix coefficients used in deriving luma and chroma from RGB primaries")
  ("ProgressiveSource",                               m_progressiveSourceFlag,                          false, "Indicate that source is progressive")
  ("InterlacedSource",                                m_interlacedSourceFlag,                           false, "Indicate that source is interlaced")
  ("NonPackedSourceConstraintFlag",                   m_nonPackedConstraintFlag,                        false, "Indicate that source does not contain frame packing")
  ("NonProjectedConstraintFlag",                      m_nonProjectedConstraintFlag,                     false, "Indicate that the bitstream contains projection SEI messages")
  ("ChromaLocInfoPresent",                            m_chromaLocInfoPresentFlag,                       false, "Signals whether chroma_sample_loc_type_top_field and chroma_sample_loc_type_bottom_field are present")
  ("ChromaSampleLocTypeTopField",                     m_chromaSampleLocTypeTopField,                        0, "Specifies the location of chroma samples for top field")
  ("ChromaSampleLocTypeBottomField",                  m_chromaSampleLocTypeBottomField,                     0, "Specifies the location of chroma samples for bottom field")
  ("ChromaSampleLocType",                             m_chromaSampleLocType,                                0, "Specifies the location of chroma samples for progressive content")
  ("OverscanInfoPresent",                             m_overscanInfoPresentFlag,                        false, "Indicates whether conformant decoded pictures are suitable for display using overscan\n")
  ("OverscanAppropriate",                             m_overscanAppropriateFlag,                        false, "Indicates whether conformant decoded pictures are suitable for display using overscan\n")
  ("VideoFullRange",                                  m_videoFullRangeFlag,                             false, "Indicates the black level and range of luma and chroma signals");
  opts.addOptions()
  ("SEIBufferingPeriod",                              m_bufferingPeriodSEIEnabled,                      false, "Control generation of buffering period SEI messages")
  ("SEIPictureTiming",                                m_pictureTimingSEIEnabled,                        false, "Control generation of picture timing SEI messages")
  ("SEIDecodingUnitInfo",                             m_decodingUnitInfoSEIEnabled,                     false, "Control generation of decoding unit information SEI message.")
  ("SEIScalableNesting",                              m_scalableNestingSEIEnabled,                      false, "Control generation of scalable nesting SEI messages")
  ("SEIFrameFieldInfo",                               m_frameFieldInfoSEIEnabled,                       false, "Control generation of frame field information SEI messages")
  ("SEIFramePacking",                                 m_framePackingSEIEnabled,                         false, "Control generation of frame packing SEI messages")
  ("SEIFramePackingType",                             m_framePackingSEIType,                                0, "Define frame packing arrangement\n"
                                                                                                               "\t3: side by side - frames are displayed horizontally\n"
                                                                                                               "\t4: top bottom - frames are displayed vertically\n"
                                                                                                               "\t5: frame alternation - one frame is alternated with the other")
  ("SEIFramePackingId",                               m_framePackingSEIId,                                  0, "Id of frame packing SEI message for a given session")
  ("SEIFramePackingQuincunx",                         m_framePackingSEIQuincunx,                            0, "Indicate the presence of a Quincunx type video frame")
  ("SEIFramePackingInterpretation",                   m_framePackingSEIInterpretation,                      0, "Indicate the interpretation of the frame pair\n"
                                                                                                               "\t0: unspecified\n"
                                                                                                               "\t1: stereo pair, frame0 represents left view\n"
                                                                                                               "\t2: stereo pair, frame0 represents right view")
  ("SEIParameterSetsInclusionIndication",             m_parameterSetsInclusionIndicationSEIEnabled,      false, "Control generation of Parameter sets inclusion indication SEI messages")
  ("SEISelfContainedClvsFlag",                        m_selfContainedClvsFlag,                               0, "Self contained CLVS indication flag value")
  ("SEIMasteringDisplayColourVolume",                 m_masteringDisplay.colourVolumeSEIEnabled,         false, "Control generation of mastering display colour volume SEI messages")
  ("SEIMasteringDisplayMaxLuminance",                 m_masteringDisplay.maxLuminance,                  10000u, "Specifies the mastering display maximum luminance value in units of 1/10000 candela per square metre (32-bit code value)")
  ("SEIMasteringDisplayMinLuminance",                 m_masteringDisplay.minLuminance,                      0u, "Specifies the mastering display minimum luminance value in units of 1/10000 candela per square metre (32-bit code value)")
  ("SEIMasteringDisplayPrimaries",                    cfg_DisplayPrimariesCode,       cfg_DisplayPrimariesCode, "Mastering display primaries for all three colour planes in CIE xy coordinates in increments of 1/50000 (results in the ranges 0 to 50000 inclusive)")
  ("SEIMasteringDisplayWhitePoint",                   cfg_DisplayWhitePointCode,     cfg_DisplayWhitePointCode, "Mastering display white point CIE xy coordinates in normalised increments of 1/50000 (e.g. 0.333 = 16667)")
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  ("SEIPreferredTransferCharacteristics",              m_preferredTransferCharacteristics,                   -1, "Value for the preferred_transfer_characteristics field of the Alternative transfer characteristics SEI which will override the corresponding entry in the VUI. If negative, do not produce the respective SEI message")
#endif

  ("SEIErpEnabled",                                   m_erpSEIEnabled,                                   false, "Control generation of equirectangular projection SEI messages")
  ("SEIErpCancelFlag",                                m_erpSEICancelFlag,                                 true, "Indicate that equirectangular projection SEI message cancels the persistence or follows")
  ("SEIErpPersistenceFlag",                           m_erpSEIPersistenceFlag,                           false, "Specifies the persistence of the equirectangular projection SEI messages")
  ("SEIErpGuardBandFlag",                             m_erpSEIGuardBandFlag,                             false, "Indicate the existence of guard band areas in the constituent picture")
  ("SEIErpGuardBandType",                             m_erpSEIGuardBandType,                                0u, "Indicate the type of the guard band")
  ("SEIErpLeftGuardBandWidth",                        m_erpSEILeftGuardBandWidth,                           0u, "Indicate the width of the guard band on the left side of the constituent picture")
  ("SEIErpRightGuardBandWidth",                       m_erpSEIRightGuardBandWidth,                          0u, "Indicate the width of the guard band on the right side of the constituent picture")
  ("SEISphereRotationEnabled",                        m_sphereRotationSEIEnabled,                        false, "Control generation of sphere rotation SEI messages")
  ("SEISphereRotationCancelFlag",                     m_sphereRotationSEICancelFlag,                      true, "Indicate that sphere rotation SEI message cancels the persistence or follows")
  ("SEISphereRotationPersistenceFlag",                m_sphereRotationSEIPersistenceFlag,                false, "Specifies the persistence of the sphere rotation SEI messages")
  ("SEISphereRotationYaw",                            m_sphereRotationSEIYaw,                                0, "Specifies the value of the yaw rotation angle")
  ("SEISphereRotationPitch",                          m_sphereRotationSEIPitch,                              0, "Specifies the value of the pitch rotation angle")
  ("SEISphereRotationRoll",                           m_sphereRotationSEIRoll,                               0, "Specifies the value of the roll rotation angle")
  ("SEIOmniViewportEnabled",                          m_omniViewportSEIEnabled,                          false, "Control generation of omni viewport SEI messages")
  ("SEIOmniViewportId",                               m_omniViewportSEIId,                                  0u, "An identifying number that may be used to identify the purpose of the one or more recommended viewport regions")
  ("SEIOmniViewportCancelFlag",                       m_omniViewportSEICancelFlag,                        true, "Indicate that omni viewport SEI message cancels the persistence or follows")
  ("SEIOmniViewportPersistenceFlag",                  m_omniViewportSEIPersistenceFlag,                  false, "Specifies the persistence of the omni viewport SEI messages")
  ("SEIOmniViewportCntMinus1",                        m_omniViewportSEICntMinus1,                           0u, "specifies the number of recommended viewport regions minus 1")
  ("SEIOmniViewportAzimuthCentre",                    cfg_omniViewportSEIAzimuthCentre,     cfg_omniViewportSEIAzimuthCentre,     "Indicate the centre of the i-th recommended viewport region")
  ("SEIOmniViewportElevationCentre",                  cfg_omniViewportSEIElevationCentre,   cfg_omniViewportSEIElevationCentre,   "Indicate the centre of the i-th recommended viewport region")
  ("SEIOmniViewportTiltCentre",                       cfg_omniViewportSEITiltCentre,        cfg_omniViewportSEITiltCentre,        "Indicates the tilt angle of the i-th recommended viewport region")
  ("SEIOmniViewportHorRange",                         cfg_omniViewportSEIHorRange,          cfg_omniViewportSEIHorRange,          "Indicates the azimuth range of the i-th recommended viewport region")
  ("SEIOmniViewportVerRange",                         cfg_omniViewportSEIVerRange,          cfg_omniViewportSEIVerRange,          "Indicates the elevation range of the i-th recommended viewport region")
  ("SEIRwpEnabled",                                   m_rwpSEIEnabled,                          false,                                    "Controls if region-wise packing SEI message enabled")
  ("SEIRwpCancelFlag",                                m_rwpSEIRwpCancelFlag,                    true,                                    "Specifies the persistence of any previous region-wise packing SEI message in output order.")
  ("SEIRwpPersistenceFlag",                           m_rwpSEIRwpPersistenceFlag,               false,                                    "Specifies the persistence of the region-wise packing SEI message for the current layer.")
  ("SEIRwpConstituentPictureMatchingFlag",            m_rwpSEIConstituentPictureMatchingFlag,   false,                                    "Specifies the information in the SEI message apply individually to each constituent picture or to the projected picture.")
  ("SEIRwpNumPackedRegions",                          m_rwpSEINumPackedRegions,                 0,                                        "specifies the number of packed regions when constituent picture matching flag is equal to 0.")
  ("SEIRwpProjPictureWidth",                          m_rwpSEIProjPictureWidth,                 0,                                        "Specifies the width of the projected picture.")
  ("SEIRwpProjPictureHeight",                         m_rwpSEIProjPictureHeight,                0,                                        "Specifies the height of the projected picture.")
  ("SEIRwpPackedPictureWidth",                        m_rwpSEIPackedPictureWidth,               0,                                        "specifies the width of the packed picture.")
  ("SEIRwpPackedPictureHeight",                       m_rwpSEIPackedPictureHeight,              0,                                        "Specifies the height of the packed picture.")
  ("SEIRwpTransformType",                             cfg_rwpSEIRwpTransformType,               cfg_rwpSEIRwpTransformType,               "specifies the rotation and mirroring to be applied to the i-th packed region.")
  ("SEIRwpGuardBandFlag",                             cfg_rwpSEIRwpGuardBandFlag,               cfg_rwpSEIRwpGuardBandFlag,               "specifies the existence of guard band in the i-th packed region.")
  ("SEIRwpProjRegionWidth",                           cfg_rwpSEIProjRegionWidth,                cfg_rwpSEIProjRegionWidth,                "specifies the width of the i-th projected region.")
  ("SEIRwpProjRegionHeight",                          cfg_rwpSEIProjRegionHeight,               cfg_rwpSEIProjRegionHeight,               "specifies the height of the i-th projected region.")
  ("SEIRwpProjRegionTop",                             cfg_rwpSEIRwpSEIProjRegionTop,            cfg_rwpSEIRwpSEIProjRegionTop,            "specifies the top sample row of the i-th projected region.")
  ("SEIRwpProjRegionLeft",                            cfg_rwpSEIProjRegionLeft,                 cfg_rwpSEIProjRegionLeft,                 "specifies the left-most sample column of the i-th projected region.")
  ("SEIRwpPackedRegionWidth",                         cfg_rwpSEIPackedRegionWidth,              cfg_rwpSEIPackedRegionWidth,              "specifies the width of the i-th packed region.")
  ("SEIRwpPackedRegionHeight",                        cfg_rwpSEIPackedRegionHeight,             cfg_rwpSEIPackedRegionHeight,             "specifies the height of the i-th packed region.")
  ("SEIRwpPackedRegionTop",                           cfg_rwpSEIPackedRegionTop,                cfg_rwpSEIPackedRegionTop,                "specifies the top luma sample row of the i-th packed region.")
  ("SEIRwpPackedRegionLeft",                          cfg_rwpSEIPackedRegionLeft,               cfg_rwpSEIPackedRegionLeft,               "specifies the left-most luma sample column of the i-th packed region.")
  ("SEIRwpLeftGuardBandWidth",                        cfg_rwpSEIRwpLeftGuardBandWidth,          cfg_rwpSEIRwpLeftGuardBandWidth,          "specifies the width of the guard band on the left side of the i-th packed region.")
  ("SEIRwpRightGuardBandWidth",                       cfg_rwpSEIRwpRightGuardBandWidth,         cfg_rwpSEIRwpRightGuardBandWidth,         "specifies the width of the guard band on the right side of the i-th packed region.")
  ("SEIRwpTopGuardBandHeight",                        cfg_rwpSEIRwpTopGuardBandHeight,          cfg_rwpSEIRwpTopGuardBandHeight,          "specifies the height of the guard band above the i-th packed region.")
  ("SEIRwpBottomGuardBandHeight",                     cfg_rwpSEIRwpBottomGuardBandHeight,       cfg_rwpSEIRwpBottomGuardBandHeight,       "specifies the height of the guard band below the i-th packed region.")
  ("SEIRwpGuardBandNotUsedForPredFlag",               cfg_rwpSEIRwpGuardBandNotUsedForPredFlag, cfg_rwpSEIRwpGuardBandNotUsedForPredFlag, "Specifies if the guard bands is used in the inter prediction process.")
  ("SEIRwpGuardBandType",                             cfg_rwpSEIRwpGuardBandType,               cfg_rwpSEIRwpGuardBandType,               "Specifies the type of the guard bands for the i-th packed region.")
  ("SEIGcmpEnabled",                                  m_gcmpSEIEnabled,                         false,                                    "Control generation of generalized cubemap projection SEI messages")
  ("SEIGcmpCancelFlag",                               m_gcmpSEICancelFlag,                      true,                                     "Indicate that generalized cubemap projection SEI message cancels the persistence or follows")
  ("SEIGcmpPersistenceFlag",                          m_gcmpSEIPersistenceFlag,                 false,                                    "Specifies the persistence of the generalized cubemap projection SEI messages")
  ("SEIGcmpPackingType",                              m_gcmpSEIPackingType,                     0u,                                       "Specifies the packing type")
  ("SEIGcmpMappingFunctionType",                      m_gcmpSEIMappingFunctionType,             0u,                                       "Specifies the mapping function used to adjust the sample locations of the cubemap projection")
  ("SEIGcmpFaceIndex",                                cfg_gcmpSEIFaceIndex,                     cfg_gcmpSEIFaceIndex,                     "Specifies the face index for the i-th face")
  ("SEIGcmpFaceRotation",                             cfg_gcmpSEIFaceRotation,                  cfg_gcmpSEIFaceRotation,                  "Specifies the rotation to be applied to the i-th face")
  ("SEIGcmpFunctionCoeffU",                           cfg_gcmpSEIFunctionCoeffU,                cfg_gcmpSEIFunctionCoeffU,                "Specifies the coefficient used in the cubemap mapping function of the u-axis of the i-th face")
  ("SEIGcmpFunctionUAffectedByVFlag",                 cfg_gcmpSEIFunctionUAffectedByVFlag,      cfg_gcmpSEIFunctionUAffectedByVFlag,      "Specifies whether the cubemap mapping function of the u-axis refers to the v position of the sample location")
  ("SEIGcmpFunctionCoeffV",                           cfg_gcmpSEIFunctionCoeffV,                cfg_gcmpSEIFunctionCoeffV,                "Specifies the coefficient used in the cubemap mapping function of the v-axis of the i-th face")
  ("SEIGcmpFunctionVAffectedByUFlag",                 cfg_gcmpSEIFunctionVAffectedByUFlag,      cfg_gcmpSEIFunctionVAffectedByUFlag,      "Specifies whether the cubemap mapping function of the v-axis refers to the u position of the sample location")
  ("SEIGcmpGuardBandFlag",                            m_gcmpSEIGuardBandFlag,                   false,                                    "Indicate the existence of guard band areas in the picture")
  ("SEIGcmpGuardBandType",                            m_gcmpSEIGuardBandType,                   0u,                                       "Indicate the type of the guard bands")
  ("SEIGcmpGuardBandBoundaryExteriorFlag",            m_gcmpSEIGuardBandBoundaryExteriorFlag,   false,                                    "Indicate whether face boundaries contain guard bands")
  ("SEIGcmpGuardBandSamplesMinus1",                   m_gcmpSEIGuardBandSamplesMinus1,          0u,                                       "Specifies the number of guard band samples minus1 used in the cubemap projected picture")
  ("SEISubpicLevelInfoEnabled",                       m_cfgSubpictureLevelInfoSEI.m_enabled,             false,            "Control generation of Subpicture Level Information SEI messages")
  ("SEISubpicLevelInfoRefLevels",                     cfg_sliRefLevels,                                  cfg_sliRefLevels, "List of reference levels for Subpicture Level Information SEI messages")
  ("SEISubpicLevelInfoExplicitFraction",              m_cfgSubpictureLevelInfoSEI.m_explicitFraction,    false,            "Enable sending of explicit fractions in Subpicture Level Information SEI messages")
  ("SEISubpicLevelInfoNumSubpics",                    m_cfgSubpictureLevelInfoSEI.m_numSubpictures,      1,                "Number of subpictures for Subpicture Level Information SEI messages")
  ("SEIAnnotatedRegionsFileRoot,-ar",                 m_arSEIFileRoot,                                 string(""), "Annotated region SEI parameters root file name (wo num ext); only the file name base is to be added. Underscore and POC would be automatically addded to . E.g. \"-ar ar\" will search for files ar_0.txt, ar_1.txt, ...")
  ("SEISubpicLevelInfoMaxSublayers",                  m_cfgSubpictureLevelInfoSEI.m_sliMaxSublayers,               1,                    "Number of sublayers for Subpicture Level Information SEI messages")
  ("SEISubpicLevelInfoSublayerInfoPresentFlag",       m_cfgSubpictureLevelInfoSEI.m_sliSublayerInfoPresentFlag,    false,                "Enable sending of level information for all sublayers in Subpicture Level Information SEI messages")
  ("SEISubpicLevelInfoRefLevelFractions",             cfg_sliFractions,                                  cfg_sliFractions, "List of subpicture level fractions for Subpicture Level Information SEI messages")
  ("SEISubpicLevelInfoNonSubpicLayersFractions",      cfg_sliNonSubpicLayersFractions,                   cfg_sliNonSubpicLayersFractions, "List of level fractions for non-subpicture layers in Subpicture Level Information SEI messages")
  ("SEISampleAspectRatioInfo",                        m_sampleAspectRatioInfoSEIEnabled,        false, "Control generation of Sample Aspect Ratio Information SEI messages")
  ("SEISARICancelFlag",                               m_sariCancelFlag,                         false, "Indicates that Sample Aspect Ratio Information SEI message cancels the persistence or follows")
  ("SEISARIPersistenceFlag",                          m_sariPersistenceFlag,                    true, "Specifies the persistence of the Sample Aspect Ratio Information SEI message")
  ("SEISARIAspectRatioIdc",                           m_sariAspectRatioIdc,                     0, "Specifies the Sample Aspect Ratio IDC of Sample Aspect Ratio Information SEI messages")
  ("SEISARISarWidth",                                 m_sariSarWidth,                           0, "Specifies the Sample Aspect Ratio Width of Sample Aspect Ratio Information SEI messages, if extended SAR is chosen.")
  ("SEISARISarHeight",                                m_sariSarHeight,                          0, "Specifies the Sample Aspect Ratio Height of Sample Aspect Ratio Information SEI messages, if extended SAR is chosen.")
  ("MCTSEncConstraint",                               m_MCTSEncConstraint,                               false, "For MCTS, constrain motion vectors at tile boundaries")
#if ENABLE_TRACING
  ("TraceChannelsList",                               bTracingChannelsList,                              false, "List all available tracing channels")
  ("TraceRule",                                       sTracingRule,                               string( "" ), "Tracing rule (ex: \"D_CABAC:poc==8\" or \"D_REC_CB_LUMA:poc==8\")")
  ("TraceFile",                                       sTracingFile,                               string( "" ), "Tracing file")
#endif
// film grain characteristics SEI
  ("SEIFGCEnabled",                                   m_fgcSEIEnabled,                                   false, "Control generation of the film grain characteristics SEI message")
  ("SEIFGCCancelFlag",                                m_fgcSEICancelFlag,                                 true, "Specifies the persistence of any previous film grain characteristics SEI message in output order.")
  ("SEIFGCPersistenceFlag",                           m_fgcSEIPersistenceFlag,                           false, "Specifies the persistence of the film grain characteristics SEI message for the current layer.")
  ("SEIFGCModelID",                                   m_fgcSEIModelID,                                      0u, "Specifies the film grain simulation model. 0: frequency filtering; 1: auto-regression.")
  ("SEIFGCSepColourDescPresentFlag",                  m_fgcSEISepColourDescPresentFlag,                  false, "Specifies the presence of a distinct colour space description for the film grain characteristics specified in the SEI message.")
  ("SEIFGCBlendingModeID",                            m_fgcSEIBlendingModeID,                               0u, "Specifies the blending mode used to blend the simulated film grain with the decoded images. 0: additive; 1: multiplicative.")
  ("SEIFGCLog2ScaleFactor",                           m_fgcSEILog2ScaleFactor,                              0u, "Specifies a scale factor used in the film grain characterization equations.")
  ("SEIFGCCompModelPresentComp0",                     m_fgcSEICompModelPresent[0],                       false, "Specifies the presence of film grain modelling on colour component 0.")
  ("SEIFGCCompModelPresentComp1",                     m_fgcSEICompModelPresent[1],                       false, "Specifies the presence of film grain modelling on colour component 1.")
  ("SEIFGCCompModelPresentComp2",                     m_fgcSEICompModelPresent[2],                       false, "Specifies the presence of film grain modelling on colour component 2.")

// content light level SEI
  ("SEICLLEnabled",                                   m_cllSEIEnabled,                                   false, "Control generation of the content light level SEI message")
  ("SEICLLMaxContentLightLevel",                      m_cllSEIMaxContentLevel,                              0u, "When not equal to 0, specifies an upper bound on the maximum light level among all individual samples in a 4:4:4 representation "
                                                                                                                "of red, green, and blue colour primary intensities in the linear light domain for the pictures of the CLVS, "
                                                                                                                "in units of candelas per square metre.When equal to 0, no such upper bound is indicated.")
  ("SEICLLMaxPicAvgLightLevel",                       m_cllSEIMaxPicAvgLevel,                               0u, "When not equal to 0, specifies an upper bound on the maximum average light level among the samples in a 4:4:4 representation "
                                                                                                                "of red, green, and blue colour primary intensities in the linear light domain for any individual picture of the CLVS, "
                                                                                                                "in units of candelas per square metre.When equal to 0, no such upper bound is indicated.")
// ambient viewing environment SEI
  ("SEIAVEEnabled",                                   m_aveSEIEnabled,                                   false, "Control generation of the ambient viewing environment SEI message")
  ("SEIAVEAmbientIlluminance",                        m_aveSEIAmbientIlluminance,                      100000u, "Specifies the environmental illluminance of the ambient viewing environment in units of 1/10000 lux for the ambient viewing environment SEI message")
  ("SEIAVEAmbientLightX",                             m_aveSEIAmbientLightX,                            15635u, "Specifies the normalized x chromaticity coordinate of the environmental ambient light in the nominal viewing enviornment according to the CIE 1931 definition in units of 1/50000 lux for the ambient viewing enviornment SEI message")
  ("SEIAVEAmbientLightY",                             m_aveSEIAmbientLightY,                            16450u, "Specifies the normalized y chromaticity coordinate of the environmental ambient light in the nominal viewing enviornment according to the CIE 1931 definition in units of 1/50000 lux for the ambient viewing enviornment SEI message")
// content colour volume SEI
  ("SEICCVEnabled",                                   m_ccvSEIEnabled,                                   false, "Control generation of the Content Colour Volume SEI message")
  ("SEICCVCancelFlag",                                m_ccvSEICancelFlag,                                 true, "Specifies the persistence of any previous content colour volume SEI message in output order.")
  ("SEICCVPersistenceFlag",                           m_ccvSEIPersistenceFlag,                           false, "Specifies the persistence of the content colour volume SEI message for the current layer.")
  ("SEICCVPrimariesPresent",                          m_ccvSEIPrimariesPresentFlag,                       true, "Specifies whether the CCV primaries are present in the content colour volume SEI message.")
  ("m_ccvSEIPrimariesX0",                             m_ccvSEIPrimariesX[0],                             0.300, "Specifies the x coordinate of the first (green) primary for the content colour volume SEI message")
  ("m_ccvSEIPrimariesY0",                             m_ccvSEIPrimariesY[0],                             0.600, "Specifies the y coordinate of the first (green) primary for the content colour volume SEI message")
  ("m_ccvSEIPrimariesX1",                             m_ccvSEIPrimariesX[1],                             0.150, "Specifies the x coordinate of the second (blue) primary for the content colour volume SEI message")
  ("m_ccvSEIPrimariesY1",                             m_ccvSEIPrimariesY[1],                             0.060, "Specifies the y coordinate of the second (blue) primary for the content colour volume SEI message")
  ("m_ccvSEIPrimariesX2",                             m_ccvSEIPrimariesX[2],                             0.640, "Specifies the x coordinate of the third (red) primary for the content colour volume SEI message")
  ("m_ccvSEIPrimariesY2",                             m_ccvSEIPrimariesY[2],                             0.330, "Specifies the y coordinate of the third (red) primary for the content colour volume SEI message")
  ("SEICCVMinLuminanceValuePresent",                  m_ccvSEIMinLuminanceValuePresentFlag,               true, "Specifies whether the CCV min luminance value is present in the content colour volume SEI message")
  ("SEICCVMinLuminanceValue",                         m_ccvSEIMinLuminanceValue,                           0.0, "specifies the CCV min luminance value  in the content colour volume SEI message")
  ("SEICCVMaxLuminanceValuePresent",                  m_ccvSEIMaxLuminanceValuePresentFlag,               true, "Specifies whether the CCV max luminance value is present in the content colour volume SEI message")
  ("SEICCVMaxLuminanceValue",                         m_ccvSEIMaxLuminanceValue,                           0.1, "specifies the CCV max luminance value  in the content colour volume SEI message")
  ("SEICCVAvgLuminanceValuePresent",                  m_ccvSEIAvgLuminanceValuePresentFlag,               true, "Specifies whether the CCV avg luminance value is present in the content colour volume SEI message")
  ("SEICCVAvgLuminanceValue",                         m_ccvSEIAvgLuminanceValue,                          0.01, "specifies the CCV avg luminance value  in the content colour volume SEI message")

  ("DebugBitstream",                                  m_decodeBitstreams[0],             string( "" ), "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
  ("DebugPOC",                                        m_switchPOC,                                 -1, "If DebugBitstream is present, load frames up to this POC from this bitstream. Starting with DebugPOC, return to normal encoding." )
  ("DecodeBitstream1",                                m_decodeBitstreams[0],             string( "" ), "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
  ("DecodeBitstream2",                                m_decodeBitstreams[1],             string( "" ), "Assume the frames up to POC DebugPOC will be the same as in this bitstream. Load those frames from the bitstream instead of encoding them." )
  ("SwitchPOC",                                       m_switchPOC,                                 -1, "If DebugBitstream is present, load frames up to this POC from this bitstream. Starting with DebugPOC, return to normal encoding." )
  ("SwitchDQP",                                       m_switchDQP,                                  0, "delta QP applied to picture with switchPOC and subsequent pictures." )
  ("FastForwardToPOC",                                m_fastForwardToPOC,                          -1, "Get to encoding the specified POC as soon as possible by skipping temporal layers irrelevant for the specified POC." )
  ("StopAfterFFtoPOC",                                m_stopAfterFFtoPOC,                       false, "If using fast forward to POC, after the POC of interest has been hit, stop further encoding.")
  ("ForceDecodeBitstream1",                           m_forceDecodeBitstream1,                  false, "force decoding of bitstream 1 - use this only if you are realy sure about what you are doing ")
  ("DecodeBitstream2ModPOCAndType",                   m_bs2ModPOCAndType,                       false, "Modify POC and NALU-type of second input bitstream, to use second BS as closing I-slice")

  ("DebugCTU",                                        m_debugCTU,                                  -1, "If DebugBitstream is present, load frames up to this POC from this bitstream. Starting with DebugPOC-frame at CTUline containin debug CTU.")
#if JVET_V0095_ALF_SAO_TRUE_ORG
  ("AlfSaoTrueOrg",                                    m_alfSaoTrueOrg,                         false, "Using true original samples for ALF and SAO optimization when MCTF is enabled\n")
#endif
  ( "ALF",                                             m_alf,                                    true, "Adaptive Loop Filter\n" )
  ("ALFStrengthLuma",                                  m_alfStrengthLuma,                         1.0, "Adaptive Loop Filter strength for luma. The parameter scales the magnitudes of the ALF filter coefficients for luma. Valid range is 0.0 <= ALFStrengthLuma <= 1.0")
  ("ALFAllowPredefinedFilters",                        m_alfAllowPredefinedFilters,              true, "Allow use of predefined filters for ALF")
  ("CCALFStrength",                                    m_ccalfStrength,                           1.0, "Cross-component Adaptive Loop Filter strength. The parameter scales the magnitudes of the CCALF filter coefficients. Valid range is 0.0 <= CCALFStrength <= 1.0")
  ("ALFStrengthChroma",                                m_alfStrengthChroma,                       1.0, "Adaptive Loop Filter strength for chroma. The parameter scales the magnitudes of the ALF filter coefficients for chroma. Valid range is 0.0 <= ALFStrengthChroma <= 1.0")
  ("ALFStrengthTargetLuma",                            m_alfStrengthTargetLuma,                   1.0, "Adaptive Loop Filter strength target for ALF luma filter optimization. The parameter scales the auto-correlation matrix E and the cross-correlation vector y for luma. Valid range is 0.0 <= ALFStrengthTargetLuma <= 1.0")
  ("ALFStrengthTargetChroma",                          m_alfStrengthTargetChroma,                 1.0, "Adaptive Loop Filter strength target for ALF chroma filter optimization. The parameter scales the auto-correlation matrix E and the cross-correlation vector y for chroma. Valid range is 0.0 <= ALFStrengthTargetChroma <= 1.0")
  ("CCALFStrengthTarget",                              m_ccalfStrengthTarget,                     1.0, "Cross-component Adaptive Loop Filter strength target for filter optimization. The parameter scales the auto-correlation matrix E and the cross-correlation vector y. Valid range is 0.0 <= CCALFStrengthTarget <= 1.0")
  ( "CCALF",                                           m_ccalf,                                  true, "Cross-component Adaptive Loop Filter" )
  ( "CCALFQpTh",                                       m_ccalfQpThreshold,                         37, "QP threshold above which encoder reduces CCALF usage")
  ( "RPR",                                            m_rprEnabledFlag,                          true, "Reference Sample Resolution" )
  ( "ScalingRatioHor",                                m_scalingRatioHor,                          1.0, "Scaling ratio in hor direction" )
  ( "ScalingRatioVer",                                m_scalingRatioVer,                          1.0, "Scaling ratio in ver direction" )
  ( "FractionNumFrames",                              m_fractionOfFrames,                         1.0, "Encode a fraction of the specified in FramesToBeEncoded frames" )
  ( "SwitchPocPeriod",                                m_switchPocPeriod,                            0, "Switch POC period for RPR" )
  ( "UpscaledOutput",                                 m_upscaledOutput,                             0, "Output upscaled (2), decoded but in full resolution buffer (1) or decoded cropped (0, default) picture for RPR" )
  ( "MaxLayers",                                      m_maxLayers,                                  1, "Max number of layers" )
  ( "EnableOperatingPointInformation",                m_OPIEnabled,                             false, "Enables writing of Operating Point Information (OPI)" )
  ( "MaxTemporalLayer",                               m_maxTemporalLayer,                         500, "Maximum temporal layer to be signalled in OPI" )
  ( "TargetOutputLayerSet",                           m_targetOlsIdx,                             500, "Target output layer set index to be signalled in OPI" )
  ;
  opts.addOptions()
  ( "MaxSublayers",                                   m_maxSublayers,                               7, "Max number of Sublayers")
  ( "DefaultPtlDpbHrdMaxTidFlag",                     m_defaultPtlDpbHrdMaxTidFlag,              true, "specifies that the syntax elements vps_ptl_max_tid[ i ], vps_dpb_max_tid[ i ], and vps_hrd_max_tid[ i ] are not present and are inferred to be equal to the default value vps_max_sublayers_minus1")
  ( "AllIndependentLayersFlag",                       m_allIndependentLayersFlag,                true, "All layers are independent layer")
  ("AllowablePredDirection",                          m_predDirectionArray, string(""),                "prediction directions allowed for i-th temporal layer")
  ( "LayerId%d",                                      m_layerId,                    0, MAX_VPS_LAYERS, "Layer ID")
  ( "NumRefLayers%d",                                 m_numRefLayers,               0, MAX_VPS_LAYERS, "Number of direct reference layer index of i-th layer")
  ( "RefLayerIdx%d",                                  m_refLayerIdxStr,    string(""), MAX_VPS_LAYERS, "Reference layer index(es)")
  ( "EachLayerIsAnOlsFlag",                           m_eachLayerIsAnOlsFlag,                    true, "Each layer is an OLS layer flag")
  ( "OlsModeIdc",                                     m_olsModeIdc,                                 0, "Output layer set mode")
  ( "NumOutputLayerSets",                             m_numOutputLayerSets,                         1, "Number of output layer sets")
  ( "OlsOutputLayer%d",                               m_olsOutputLayerStr, string(""), MAX_VPS_LAYERS, "Output layer index of i-th OLS")
  ( "NumPTLsInVPS",                                   m_numPtlsInVps,                               1, "Number of profile_tier_level structures in VPS" )
  ( "AvoidIntraInDepLayers",                          m_avoidIntraInDepLayer,                    true, "Replaces I pictures in dependent layers with B pictures" )
  ( "MaxTidILRefPicsPlusOneLayerId%d",                m_maxTidILRefPicsPlus1Str, string(""), MAX_VPS_LAYERS, "Maximum temporal ID for inter-layer reference pictures plus 1 of i-th layer, 0 for IRAP only")
    ;

  opts.addOptions()
    ("TemporalFilter",                                m_gopBasedTemporalFilterEnabled,          false,            "Enable GOP based temporal filter. Disabled per default")
    ("TemporalFilterFutureReference",                 m_gopBasedTemporalFilterFutureReference,   true,            "Enable referencing of future frames in the GOP based temporal filter. This is typically disabled for Low Delay configurations.")
    ("TemporalFilterStrengthFrame*",                  m_gopBasedTemporalFilterStrengths, std::map<int, double>(), "Strength for every * frame in GOP based temporal filter, where * is an integer."
                                                                                                                  " E.g. --TemporalFilterStrengthFrame8 0.95 will enable GOP based temporal filter at every 8th frame with strength 0.95");
  // clang-format on

#if EXTENSION_360_VIDEO
  TExt360AppEncCfg::TExt360AppEncCfgContext ext360CfgContext;
  m_ext360.addOptions( opts, ext360CfgContext );
#endif

  for ( int i = 1; i < MAX_GOP + 1; i++ ) {
    std::ostringstream cOSS;
    cOSS << "Frame" << i;
    opts.addOptions()( cOSS.str(), m_GOPList[i - 1], GOPEntry() );
  }

  for ( int i = 0; i < MAX_NUM_OLSS; i++ ) {
    std::ostringstream cOSS1;
    cOSS1 << "LevelPTL" << i;
    opts.addOptions()( cOSS1.str(), m_levelPtl[i], Level::NONE );

    std::ostringstream cOSS2;
    cOSS2 << "OlsPTLIdx" << i;
    opts.addOptions()( cOSS2.str(), m_olsPtlIdx[i], 0 );
  }

  po::setDefaults( opts );
  po::ErrorReporter        err;
  const list<const char*>& argv_unhandled = po::scanArgv( opts, argc, (const char**)argv, err );

  m_resChangeInClvsEnabled = m_scalingRatioHor != 1.0 || m_scalingRatioVer != 1.0;
  m_resChangeInClvsEnabled = m_resChangeInClvsEnabled && m_rprEnabledFlag;
  if ( m_fractionOfFrames != 1.0 ) { m_framesToBeEncoded = int( m_framesToBeEncoded * m_fractionOfFrames ); }

  if ( m_resChangeInClvsEnabled && !m_switchPocPeriod ) {
    m_switchPocPeriod = m_iFrameRate / 2 / m_iGOPSize * m_iGOPSize;
  }

  // Check the given value of intra period and decoding refresh type. If intra period is -1, set decoding refresh type
  // to be equal to 0. And vice versa
  if ( m_iIntraPeriod == -1 ) { m_iDecodingRefreshType = 0; }
  if ( !m_iDecodingRefreshType ) { m_iIntraPeriod = -1; }

#if GDR_ENABLED
  if ( m_gdrEnabled ) {
    m_iDecodingRefreshType = 3;
    m_intraQPOffset        = 0;
    m_iGOPSize             = 1;

    m_GOPList[0].m_POC                 = 1;
    m_GOPList[0].m_sliceType           = 'B';
    m_GOPList[0].m_QPOffset            = 0;
    m_GOPList[0].m_QPOffsetModelOffset = 0;
    m_GOPList[0].m_QPOffsetModelScale  = 0;
    m_GOPList[0].m_CbQPoffset          = 0;
    m_GOPList[0].m_CrQPoffset          = 0;
    m_GOPList[0].m_QPFactor            = 1.0;
    m_GOPList[0].m_tcOffsetDiv2        = 0;
    m_GOPList[0].m_betaOffsetDiv2      = 0;
    m_GOPList[0].m_CbTcOffsetDiv2      = 0;
    m_GOPList[0].m_CbBetaOffsetDiv2    = 0;
    m_GOPList[0].m_CrTcOffsetDiv2      = 0;
    m_GOPList[0].m_CrBetaOffsetDiv2    = 0;
    m_GOPList[0].m_temporalId          = 0;

    m_GOPList[0].m_numRefPicsActive0 = 4;
    m_GOPList[0].m_numRefPics0       = 4;
    m_GOPList[0].m_deltaRefPics0[0]  = 1;
    m_GOPList[0].m_deltaRefPics0[1]  = 2;
    m_GOPList[0].m_deltaRefPics0[2]  = 3;
    m_GOPList[0].m_deltaRefPics0[3]  = 4;

    m_GOPList[0].m_numRefPicsActive1 = 4;
    m_GOPList[0].m_numRefPics1       = 4;
    m_GOPList[0].m_deltaRefPics1[0]  = 1;
    m_GOPList[0].m_deltaRefPics1[1]  = 2;
    m_GOPList[0].m_deltaRefPics1[2]  = 3;
    m_GOPList[0].m_deltaRefPics1[3]  = 4;

    m_BIO  = false;
    m_DMVR = false;
    m_SMVD = false;

    if ( m_gdrPeriod < 0 ) { m_gdrPeriod = m_iFrameRate * 2; }

    if ( m_gdrInterval < 0 ) { m_gdrInterval = m_iFrameRate; }

    if ( m_gdrPocStart < 0 ) { m_gdrPocStart = m_gdrPeriod; }

    if ( m_iIntraPeriod == -1 ) {
      m_iFrameRate = ( m_iFrameRate == 0 ) ? 30 : m_iFrameRate;
      if ( m_gdrPocStart % m_iFrameRate != 0 )
        m_iIntraPeriod = -1;
      else
        m_iIntraPeriod = m_gdrPeriod;
    }
  }
#endif

  m_bpDeltasGOPStructure = false;
  if ( m_iGOPSize == 16 ) {
    if ( ( m_GOPList[0].m_POC == 16 && m_GOPList[0].m_temporalId == 0 ) &&
         ( m_GOPList[1].m_POC == 8 && m_GOPList[1].m_temporalId == 1 ) &&
         ( m_GOPList[2].m_POC == 4 && m_GOPList[2].m_temporalId == 2 ) &&
         ( m_GOPList[3].m_POC == 2 && m_GOPList[3].m_temporalId == 3 ) &&
         ( m_GOPList[4].m_POC == 1 && m_GOPList[4].m_temporalId == 4 ) &&
         ( m_GOPList[5].m_POC == 3 && m_GOPList[5].m_temporalId == 4 ) &&
         ( m_GOPList[6].m_POC == 6 && m_GOPList[6].m_temporalId == 3 ) &&
         ( m_GOPList[7].m_POC == 5 && m_GOPList[7].m_temporalId == 4 ) &&
         ( m_GOPList[8].m_POC == 7 && m_GOPList[8].m_temporalId == 4 ) &&
         ( m_GOPList[9].m_POC == 12 && m_GOPList[9].m_temporalId == 2 ) &&
         ( m_GOPList[10].m_POC == 10 && m_GOPList[10].m_temporalId == 3 ) &&
         ( m_GOPList[11].m_POC == 9 && m_GOPList[11].m_temporalId == 4 ) &&
         ( m_GOPList[12].m_POC == 11 && m_GOPList[12].m_temporalId == 4 ) &&
         ( m_GOPList[13].m_POC == 14 && m_GOPList[13].m_temporalId == 3 ) &&
         ( m_GOPList[14].m_POC == 13 && m_GOPList[14].m_temporalId == 4 ) &&
         ( m_GOPList[15].m_POC == 15 && m_GOPList[15].m_temporalId == 4 ) ) {
      m_bpDeltasGOPStructure = true;
    }
  } else if ( m_iGOPSize == 8 ) {
    if ( ( m_GOPList[0].m_POC == 8 && m_GOPList[0].m_temporalId == 0 ) &&
         ( m_GOPList[1].m_POC == 4 && m_GOPList[1].m_temporalId == 1 ) &&
         ( m_GOPList[2].m_POC == 2 && m_GOPList[2].m_temporalId == 2 ) &&
         ( m_GOPList[3].m_POC == 1 && m_GOPList[3].m_temporalId == 3 ) &&
         ( m_GOPList[4].m_POC == 3 && m_GOPList[4].m_temporalId == 3 ) &&
         ( m_GOPList[5].m_POC == 6 && m_GOPList[5].m_temporalId == 2 ) &&
         ( m_GOPList[6].m_POC == 5 && m_GOPList[6].m_temporalId == 3 ) &&
         ( m_GOPList[7].m_POC == 7 && m_GOPList[7].m_temporalId == 3 ) ) {
      m_bpDeltasGOPStructure = true;
    }
  } else {
    m_bpDeltasGOPStructure = false;
  }
  for ( int i = 0; m_GOPList[i].m_POC != -1 && i < MAX_GOP + 1; i++ ) {
    m_RPLList0[i].m_POC = m_RPLList1[i].m_POC = m_GOPList[i].m_POC;
    m_RPLList0[i].m_temporalId = m_RPLList1[i].m_temporalId = m_GOPList[i].m_temporalId;
    m_RPLList0[i].m_refPic = m_RPLList1[i].m_refPic = m_GOPList[i].m_refPic;
    m_RPLList0[i].m_sliceType = m_RPLList1[i].m_sliceType = m_GOPList[i].m_sliceType;
    m_RPLList0[i].m_isEncoded = m_RPLList1[i].m_isEncoded = m_GOPList[i].m_isEncoded;

    m_RPLList0[i].m_numRefPicsActive          = m_GOPList[i].m_numRefPicsActive0;
    m_RPLList1[i].m_numRefPicsActive          = m_GOPList[i].m_numRefPicsActive1;
    m_RPLList0[i].m_numRefPics                = m_GOPList[i].m_numRefPics0;
    m_RPLList1[i].m_numRefPics                = m_GOPList[i].m_numRefPics1;
    m_RPLList0[i].m_ltrp_in_slice_header_flag = m_GOPList[i].m_ltrp_in_slice_header_flag;
    m_RPLList1[i].m_ltrp_in_slice_header_flag = m_GOPList[i].m_ltrp_in_slice_header_flag;
    for ( int j = 0; j < m_GOPList[i].m_numRefPics0; j++ )
      m_RPLList0[i].m_deltaRefPics[j] = m_GOPList[i].m_deltaRefPics0[j];
    for ( int j = 0; j < m_GOPList[i].m_numRefPics1; j++ )
      m_RPLList1[i].m_deltaRefPics[j] = m_GOPList[i].m_deltaRefPics1[j];
  }

  if ( m_compositeRefEnabled ) {
    for ( int i = 0; i < m_iGOPSize; i++ ) {
      m_GOPList[i].m_POC *= 2;
      m_RPLList0[i].m_POC *= 2;
      m_RPLList1[i].m_POC *= 2;
      for ( int j = 0; j < m_RPLList0[i].m_numRefPics; j++ ) { m_RPLList0[i].m_deltaRefPics[j] *= 2; }
      for ( int j = 0; j < m_RPLList1[i].m_numRefPics; j++ ) { m_RPLList1[i].m_deltaRefPics[j] *= 2; }
    }
  }

  for ( list<const char*>::const_iterator it = argv_unhandled.begin(); it != argv_unhandled.end(); it++ ) {
    msg( VTM_ERROR, "Unhandled argument ignored: `%s'\n", *it );
  }

  if ( argc == 1 || do_help ) {
    /* argc == 1: no options have been specified */
    po::doHelp( cout, opts );
    return false;
  }

  if ( err.is_errored ) {
    if ( !warnUnknowParameter ) {
      /* error report has already been printed on stderr */
      return false;
    }
  }

  g_verbosity = MsgLevel( m_verbosity );

  /*
   * Set any derived parameters
   */
#if EXTENSION_360_VIDEO
  m_inputFileWidth  = m_sourceWidth;
  m_inputFileHeight = m_sourceHeight;
  m_ext360.setMaxCUInfo( m_uiCTUSize, 1 << MIN_CU_LOG2 );
#endif

  if ( !inputPathPrefix.empty() && inputPathPrefix.back() != '/' && inputPathPrefix.back() != '\\' ) {
    inputPathPrefix += "/";
  }
  m_inputFileName = inputPathPrefix + m_inputFileName;

  if ( m_temporalSubsampleRatio < 1 ) { EXIT( "Error: TemporalSubsampleRatio must be greater than 0" ); }

  m_framesToBeEncoded     = ( m_framesToBeEncoded + m_temporalSubsampleRatio - 1 ) / m_temporalSubsampleRatio;
  m_adIntraLambdaModifier = cfg_adIntraLambdaModifier.values;
  if ( m_isField ) {
    // Frame height
    m_sourceHeightOrg = m_sourceHeight;
    // Field height
    m_sourceHeight = m_sourceHeight >> 1;
    // number of fields to encode
    m_framesToBeEncoded *= 2;
  }
  if ( m_subPicInfoPresentFlag ) {
    CHECK( m_numSubPics > MAX_NUM_SUB_PICS || m_numSubPics < 1, "Number of subpicture must be within 1 to 2^16" )
    if ( !m_subPicSameSizeFlag ) {
      CHECK( cfg_subPicCtuTopLeftX.values.size() != m_numSubPics,
             "Number of SubPicCtuTopLeftX values must be equal to NumSubPics" );
      CHECK( cfg_subPicCtuTopLeftY.values.size() != m_numSubPics,
             "Number of SubPicCtuTopLeftY values must be equal to NumSubPics" );
      CHECK( cfg_subPicWidth.values.size() != m_numSubPics,
             "Number of SubPicWidth values must be equal to NumSubPics" );
      CHECK( cfg_subPicHeight.values.size() != m_numSubPics,
             "Number of SubPicHeight values must be equal to NumSubPics" );
    } else {
      CHECK( cfg_subPicCtuTopLeftX.values.size() != 0, "Number of SubPicCtuTopLeftX values must be equal to 0" );
      CHECK( cfg_subPicCtuTopLeftY.values.size() != 0, "Number of SubPicCtuTopLeftY values must be equal to 0" );
      CHECK( cfg_subPicWidth.values.size() != 1, "Number of SubPicWidth values must be equal to 1" );
      CHECK( cfg_subPicHeight.values.size() != 1, "Number of SubPicHeight values must be equal to 1" );
    }
    CHECK( cfg_subPicTreatedAsPicFlag.values.size() != m_numSubPics,
           "Number of SubPicTreatedAsPicFlag values must be equal to NumSubPics" );
    CHECK( cfg_loopFilterAcrossSubpicEnabledFlag.values.size() != m_numSubPics,
           "Number of LoopFilterAcrossSubpicEnabledFlag values must be equal to NumSubPics" );
    if ( m_subPicIdMappingExplicitlySignalledFlag ) {
      CHECK( cfg_subPicId.values.size() != m_numSubPics, "Number of SubPicId values must be equal to NumSubPics" );
    }
    m_subPicCtuTopLeftX                 = cfg_subPicCtuTopLeftX.values;
    m_subPicCtuTopLeftY                 = cfg_subPicCtuTopLeftY.values;
    m_subPicWidth                       = cfg_subPicWidth.values;
    m_subPicHeight                      = cfg_subPicHeight.values;
    m_subPicTreatedAsPicFlag            = cfg_subPicTreatedAsPicFlag.values;
    m_loopFilterAcrossSubpicEnabledFlag = cfg_loopFilterAcrossSubpicEnabledFlag.values;
    if ( m_subPicIdMappingExplicitlySignalledFlag ) {
      for ( int i = 0; i < m_numSubPics; i++ ) { m_subPicId[i] = cfg_subPicId.values[i]; }
    }
    uint32_t tmpWidthVal  = ( m_sourceWidth + m_uiCTUSize - 1 ) / m_uiCTUSize;
    uint32_t tmpHeightVal = ( m_sourceHeight + m_uiCTUSize - 1 ) / m_uiCTUSize;
    if ( !m_subPicSameSizeFlag ) {
      for ( int i = 0; i < m_numSubPics; i++ ) {
        CHECK( m_subPicCtuTopLeftX[i] + m_subPicWidth[i] > tmpWidthVal, "Subpicture must not exceed picture boundary" );
        CHECK( m_subPicCtuTopLeftY[i] + m_subPicHeight[i] > tmpHeightVal,
               "Subpicture must not exceed picture boundary" );
      }
    } else {
      uint32_t numSubpicCols = tmpWidthVal / m_subPicWidth[0];
      CHECK( tmpWidthVal % m_subPicWidth[0] != 0, "sps_subpic_width_minus1[0] is invalid." );
      CHECK( tmpHeightVal % m_subPicHeight[0] != 0, "sps_subpic_height_minus1[0] is invalid." );
      CHECK( numSubpicCols * ( tmpHeightVal / m_subPicHeight[0] ) != m_numSubPics,
             "when sps_subpic_same_size_flag is equal to, sps_num_subpics_minus1 is invalid" );
    }
    // automatically determine subpicture ID lenght in case it is not specified
    if ( m_subPicIdLen == 0 ) {
      if ( m_subPicIdMappingExplicitlySignalledFlag ) {
        // use the heighest specified ID
        auto maxIdVal = std::max_element( m_subPicId.begin(), m_subPicId.end() );
        m_subPicIdLen = ceilLog2( *maxIdVal );
      } else {
        // use the number of subpictures
        m_subPicIdLen = ceilLog2( m_numSubPics );
      }
    }

    CHECK( m_subPicIdLen > 16, "SubPicIdLen must not exceed 16 bits" );
    CHECK( m_resChangeInClvsEnabled, "resolution change in CLVS and subpictures cannot be enabled together" );
  }

  if ( m_virtualBoundariesPresentFlag ) {
    if ( m_sourceWidth <= 8 )
      CHECK( m_numVerVirtualBoundaries != 0,
             "The number of vertical virtual boundaries shall be 0 when the picture width is less than or equal to 8" );

    if ( m_sourceHeight <= 8 )
      CHECK(
          m_numHorVirtualBoundaries != 0,
          "The number of horizontal virtual boundaries shall be 0 when the picture height is less than or equal to 8" );
  }

  if ( m_cfgSubpictureLevelInfoSEI.m_enabled ) {
    CHECK( m_numSubPics != m_cfgSubpictureLevelInfoSEI.m_numSubpictures,
           "NumSubPics must be equal to SEISubpicLevelInfoNumSubpics" );
    CHECK( m_cfgSubpictureLevelInfoSEI.m_sliMaxSublayers != m_maxSublayers,
           "SEISubpicLevelInfoMaxSublayers must be equal to vps_max_sublayers" );
    if ( m_cfgSubpictureLevelInfoSEI.m_sliSublayerInfoPresentFlag ) {
      CHECK( cfg_sliRefLevels.values.size() < m_maxSublayers,
             "when sliSublayerInfoPresentFlag = 1, the number of reference levels must be greater than or equal to "
             "sublayers" );
    }
    if ( m_cfgSubpictureLevelInfoSEI.m_explicitFraction ) {
      m_cfgSubpictureLevelInfoSEI.m_fractions = cfg_sliFractions.values;
      m_cfgSubpictureLevelInfoSEI.m_refLevels = cfg_sliRefLevels.values;
      if ( m_cfgSubpictureLevelInfoSEI.m_sliSublayerInfoPresentFlag ) {
        CHECK( (int)cfg_sliRefLevels.values.size() / m_maxSublayers * m_cfgSubpictureLevelInfoSEI.m_numSubpictures *
                       m_cfgSubpictureLevelInfoSEI.m_sliMaxSublayers !=
                   cfg_sliFractions.values.size(),
               "when sliSublayerInfoPresentFlag = 1, the number  of subpicture level fractions must be equal to the "
               "numer of subpictures times the number of reference levels times the number of sublayers" );
      } else {
        CHECK( (int)cfg_sliRefLevels.values.size() * m_cfgSubpictureLevelInfoSEI.m_numSubpictures !=
                   cfg_sliFractions.values.size(),
               "when sliSublayerInfoPresentFlag = 0, the number  of subpicture level fractions must be equal to the "
               "numer of subpictures times the number of reference levels" );
      }
    }
    m_cfgSubpictureLevelInfoSEI.m_nonSubpicLayersFraction = cfg_sliNonSubpicLayersFractions.values;
    if ( m_cfgSubpictureLevelInfoSEI.m_sliSublayerInfoPresentFlag ) {
      CHECK( (int)cfg_sliNonSubpicLayersFractions.values.size() !=
                 ( cfg_sliRefLevels.values.size() * m_cfgSubpictureLevelInfoSEI.m_numSubpictures ),
             "when sliSublayerInfoPresentFlag = 1, the number  of non-subpicture level fractions must be equal to the "
             "numer of reference levels times the number of sublayers" );
    } else {
      CHECK( (int)cfg_sliNonSubpicLayersFractions.values.size() != ( cfg_sliRefLevels.values.size() ),
             "when sliSublayerInfoPresentFlag = 0, the number  of non-subpicture level fractions must be equal to the "
             "numer of reference levels" );
    }
  }

  if ( m_costMode != COST_LOSSLESS_CODING && m_mixedLossyLossless ) {
    m_mixedLossyLossless = 0;
    msg( VTM_WARNING, "*************************************************************************\n" );
    msg( VTM_WARNING, "* Mixed lossy lossles coding cannot enable in lossy costMode *\n" );
    msg( VTM_WARNING, "* Forcely disabled  m_mixedLossyLossless *\n" );
    msg( VTM_WARNING, "*************************************************************************\n" );
  }
  if ( !m_mixedLossyLossless && cfgSliceLosslessArray.values.size() > 0 ) {
    msg( VTM_WARNING, "*************************************************************************\n" );
    msg( VTM_WARNING, "* Mixed lossy lossles coding is not enabled *\n" );
    msg( VTM_WARNING, "* ignoring the value of SliceLosslessArray *\n" );
    msg( VTM_WARNING, "*************************************************************************\n" );
  }

  if ( m_costMode == COST_LOSSLESS_CODING && m_mixedLossyLossless ) {
    m_sliceLosslessArray.resize( cfgSliceLosslessArray.values.size() );
    for ( uint32_t i = 0; i < cfgSliceLosslessArray.values.size(); i++ ) {
      m_sliceLosslessArray[i] = cfgSliceLosslessArray.values[i];
    }
  }

  if ( m_picPartitionFlag ) {
    // store tile column widths
    m_tileColumnWidth.resize( cfgTileColumnWidth.values.size() );
    for ( uint32_t i = 0; i < cfgTileColumnWidth.values.size(); i++ ) {
      m_tileColumnWidth[i] = cfgTileColumnWidth.values[i];
    }

    // store tile row heights
    m_tileRowHeight.resize( cfgTileRowHeight.values.size() );
    for ( uint32_t i = 0; i < cfgTileRowHeight.values.size(); i++ ) { m_tileRowHeight[i] = cfgTileRowHeight.values[i]; }

    // store rectangular slice positions
    if ( !m_rasterSliceFlag ) {
      m_rectSlicePos.resize( cfgRectSlicePos.values.size() );
      for ( uint32_t i = 0; i < cfgRectSlicePos.values.size(); i++ ) { m_rectSlicePos[i] = cfgRectSlicePos.values[i]; }
    }

    // store raster-scan slice sizes
    else {
      m_rasterSliceSize.resize( cfgRasterSliceSize.values.size() );
      for ( uint32_t i = 0; i < cfgRasterSliceSize.values.size(); i++ ) {
        m_rasterSliceSize[i] = cfgRasterSliceSize.values[i];
      }
    }
  } else {
    m_tileColumnWidth.clear();
    m_tileRowHeight.clear();
    m_rectSlicePos.clear();
    m_rasterSliceSize.clear();
    m_rectSliceFixedWidth  = 0;
    m_rectSliceFixedHeight = 0;
  }

  m_numSubProfile = (uint8_t)cfg_SubProfile.values.size();
  m_subProfile.resize( m_numSubProfile );
  for ( uint8_t i = 0; i < m_numSubProfile; ++i ) { m_subProfile[i] = cfg_SubProfile.values[i]; }
  /* rules for input, output and internal bitdepths as per help text */
  if ( m_MSBExtendedBitDepth[CHANNEL_TYPE_LUMA] == 0 ) {
    m_MSBExtendedBitDepth[CHANNEL_TYPE_LUMA] = m_inputBitDepth[CHANNEL_TYPE_LUMA];
  }
  if ( m_MSBExtendedBitDepth[CHANNEL_TYPE_CHROMA] == 0 ) {
    m_MSBExtendedBitDepth[CHANNEL_TYPE_CHROMA] = m_MSBExtendedBitDepth[CHANNEL_TYPE_LUMA];
  }
  if ( m_internalBitDepth[CHANNEL_TYPE_LUMA] == 0 ) {
    m_internalBitDepth[CHANNEL_TYPE_LUMA] = m_MSBExtendedBitDepth[CHANNEL_TYPE_LUMA];
  }
  m_internalBitDepth[CHANNEL_TYPE_CHROMA] = m_internalBitDepth[CHANNEL_TYPE_LUMA];
  if ( m_inputBitDepth[CHANNEL_TYPE_CHROMA] == 0 ) {
    m_inputBitDepth[CHANNEL_TYPE_CHROMA] = m_inputBitDepth[CHANNEL_TYPE_LUMA];
  }
  if ( m_outputBitDepth[CHANNEL_TYPE_LUMA] == 0 ) {
    m_outputBitDepth[CHANNEL_TYPE_LUMA] = m_internalBitDepth[CHANNEL_TYPE_LUMA];
  }
  if ( m_outputBitDepth[CHANNEL_TYPE_CHROMA] == 0 ) {
    m_outputBitDepth[CHANNEL_TYPE_CHROMA] = m_outputBitDepth[CHANNEL_TYPE_LUMA];
  }

  m_InputChromaFormatIDC = numberToChromaFormat( tmpInputChromaFormat );
  m_chromaFormatIDC =
      ( ( tmpChromaFormat == 0 ) ? ( m_InputChromaFormatIDC ) : ( numberToChromaFormat( tmpChromaFormat ) ) );
#if EXTENSION_360_VIDEO
  m_ext360.processOptions( ext360CfgContext );
#endif

  CHECK( !( tmpWeightedPredictionMethod >= 0 &&
            tmpWeightedPredictionMethod <= WP_PER_PICTURE_WITH_HISTOGRAM_AND_PER_COMPONENT_AND_CLIPPING_AND_EXTENSION ),
         "Error in cfg" );
  m_weightedPredictionMethod = WeightedPredictionMethod( tmpWeightedPredictionMethod );

  CHECK( tmpFastInterSearchMode < 0 || tmpFastInterSearchMode > FASTINTERSEARCH_MODE3, "Error in cfg" );
  m_fastInterSearchMode = FastInterSearchMode( tmpFastInterSearchMode );

  CHECK( tmpMotionEstimationSearchMethod < 0 || tmpMotionEstimationSearchMethod >= MESEARCH_NUMBER_OF_METHODS,
         "Error in cfg" );
  m_motionEstimationSearchMethod = MESearchMethod( tmpMotionEstimationSearchMethod );

  if ( extendedProfile == ExtendedProfileName::AUTO ) {
    if ( xAutoDetermineProfile() ) { EXIT( "Unable to determine profile from configured settings" ); }
  } else {
    switch ( extendedProfile ) {
      case ExtendedProfileName::NONE: m_profile = Profile::NONE; break;
      case ExtendedProfileName::MAIN_10: m_profile = Profile::MAIN_10; break;
      case ExtendedProfileName::MAIN_10_444: m_profile = Profile::MAIN_10_444; break;
      case ExtendedProfileName::MAIN_10_STILL_PICTURE: m_profile = Profile::MAIN_10_STILL_PICTURE; break;
      case ExtendedProfileName::MAIN_10_444_STILL_PICTURE: m_profile = Profile::MAIN_10_444_STILL_PICTURE; break;
      case ExtendedProfileName::MULTILAYER_MAIN_10: m_profile = Profile::MULTILAYER_MAIN_10; break;
      case ExtendedProfileName::MULTILAYER_MAIN_10_444: m_profile = Profile::MULTILAYER_MAIN_10_444; break;
      case ExtendedProfileName::MULTILAYER_MAIN_10_STILL_PICTURE:
        m_profile = Profile::MULTILAYER_MAIN_10_STILL_PICTURE;
        break;
      case ExtendedProfileName::MULTILAYER_MAIN_10_444_STILL_PICTURE:
        m_profile = Profile::MULTILAYER_MAIN_10_444_STILL_PICTURE;
        break;
      default: EXIT( "Unable to determine profile from configured settings" ); break;
    }
  }

  {
    m_chromaFormatConstraint =
        ( tmpConstraintChromaFormat == 0 ) ? m_chromaFormatIDC : numberToChromaFormat( tmpConstraintChromaFormat );
    if ( m_bitDepthConstraint == 0 ) {
      if ( m_profile != Profile::NONE ) {
        const ProfileFeatures* features = ProfileFeatures::getProfileFeatures( m_profile );
        CHECK( features->profile != m_profile, "Profile not found" );
        m_bitDepthConstraint = features->maxBitDepth;
      } else  // m_profile == Profile::NONE
      {
        m_bitDepthConstraint = 16;  // max value - unconstrained.
      }
    }
    CHECK( m_bitDepthConstraint < m_internalBitDepth[CHANNEL_TYPE_LUMA],
           "MaxBitDepthConstraint setting does not allow the specified luma bit depth to be coded." );
    CHECK( m_bitDepthConstraint < m_internalBitDepth[CHANNEL_TYPE_CHROMA],
           "MaxBitDepthConstraint setting does not allow the specified chroma bit depth to be coded." );
    CHECK( m_chromaFormatConstraint < m_chromaFormatIDC,
           "MaxChromaFormatConstraint setting does not allow the specified chroma format to be coded." );
    CHECK( m_chromaFormatConstraint >= NUM_CHROMA_FORMAT, "Bad value given for MaxChromaFormatConstraint setting." )
    CHECK( m_bitDepthConstraint < 8 || m_bitDepthConstraint > 16,
           "MaxBitDepthConstraint setting must be in the range 8 to 16 (inclusive)" );
  }

  m_inputColourSpaceConvert = stringToInputColourSpaceConvert( inputColourSpaceConvert, true );
  m_rgbFormat =
      ( m_inputColourSpaceConvert == IPCOLOURSPACE_RGBtoGBR && m_chromaFormatIDC == CHROMA_444 ) ? true : false;

  // Picture width and height must be multiples of 8 and minCuSize
  const int minResolutionMultiple = std::max( 8, 1 << m_log2MinCuSize );

  switch ( m_conformanceWindowMode ) {
    case 0: {
      // no conformance or padding
      m_confWinLeft = m_confWinRight = m_confWinTop = m_confWinBottom = 0;
      m_sourcePadding[1] = m_sourcePadding[0] = 0;
      break;
    }
    case 1: {
      // automatic padding to minimum CU size
      if ( m_sourceWidth % minResolutionMultiple ) {
        m_sourcePadding[0] = m_confWinRight =
            ( ( m_sourceWidth / minResolutionMultiple ) + 1 ) * minResolutionMultiple - m_sourceWidth;
        m_sourceWidth += m_confWinRight;
      }
      if ( m_sourceHeight % minResolutionMultiple ) {
        m_sourcePadding[1] = m_confWinBottom =
            ( ( m_sourceHeight / minResolutionMultiple ) + 1 ) * minResolutionMultiple - m_sourceHeight;
        m_sourceHeight += m_confWinBottom;
        if ( m_isField ) {
          m_sourceHeightOrg += m_confWinBottom << 1;
          m_sourcePadding[1] = m_confWinBottom << 1;
        }
      }
      if ( m_sourcePadding[0] % SPS::getWinUnitX( m_chromaFormatIDC ) != 0 ) {
        EXIT( "Error: picture width is not an integer multiple of the specified chroma subsampling" );
      }
      if ( m_sourcePadding[1] % SPS::getWinUnitY( m_chromaFormatIDC ) != 0 ) {
        EXIT( "Error: picture height is not an integer multiple of the specified chroma subsampling" );
      }
      if ( m_sourcePadding[0] ) {
        msg( VTM_INFO, "Info: Conformance window automatically enabled. Adding %i lumal pel horizontally\n",
             m_sourcePadding[0] );
      }
      if ( m_sourcePadding[1] ) {
        msg( VTM_INFO, "Info: Conformance window automatically enabled. Adding %i lumal pel vertically\n",
             m_sourcePadding[1] );
      }
      break;
    }
    case 2: {
      // padding
      m_sourceWidth += m_sourcePadding[0];
      m_sourceHeight += m_sourcePadding[1];
      m_confWinRight  = m_sourcePadding[0];
      m_confWinBottom = m_sourcePadding[1];
      break;
    }
    case 3: {
      // conformance
      if ( ( m_confWinLeft == 0 ) && ( m_confWinRight == 0 ) && ( m_confWinTop == 0 ) && ( m_confWinBottom == 0 ) ) {
        msg( VTM_ERROR, "Warning: Conformance window enabled, but all conformance window parameters set to zero\n" );
      }
      if ( ( m_sourcePadding[1] != 0 ) || ( m_sourcePadding[0] != 0 ) ) {
        msg( VTM_ERROR, "Warning: Conformance window enabled, padding parameters will be ignored\n" );
      }
      m_sourcePadding[1] = m_sourcePadding[0] = 0;
      break;
    }
  }
  CHECK( ( ( m_sourceWidth % minResolutionMultiple ) || ( m_sourceHeight % minResolutionMultiple ) ),
         "Picture width or height (after padding) is not a multiple of 8 or minCuSize, please use "
         "ConformanceWindowMode=1 for automatic adjustment or ConformanceWindowMode=2 to specify padding manually!!" );

  if ( m_conformanceWindowMode > 0 && m_subPicInfoPresentFlag ) {
    for ( int i = 0; i < m_numSubPics; i++ ) {
      CHECK( ( m_subPicCtuTopLeftX[i] * m_uiCTUSize ) >=
                 ( m_sourceWidth - m_confWinRight * SPS::getWinUnitX( m_chromaFormatIDC ) ),
             "No subpicture can be located completely outside of the conformance cropping window" );
      CHECK( ( ( m_subPicCtuTopLeftX[i] + m_subPicWidth[i] ) * m_uiCTUSize ) <=
                 ( m_confWinLeft * SPS::getWinUnitX( m_chromaFormatIDC ) ),
             "No subpicture can be located completely outside of the conformance cropping window" );
      CHECK( ( m_subPicCtuTopLeftY[i] * m_uiCTUSize ) >=
                 ( m_sourceHeight - m_confWinBottom * SPS::getWinUnitY( m_chromaFormatIDC ) ),
             "No subpicture can be located completely outside of the conformance cropping window" );
      CHECK( ( ( m_subPicCtuTopLeftY[i] + m_subPicHeight[i] ) * m_uiCTUSize ) <=
                 ( m_confWinTop * SPS::getWinUnitY( m_chromaFormatIDC ) ),
             "No subpicture can be located completely outside of the conformance cropping window" );
    }
  }

  if ( tmpDecodedPictureHashSEIMappedType < 0 || tmpDecodedPictureHashSEIMappedType >= int( NUMBER_OF_HASHTYPES ) ) {
    EXIT( "Error: bad checksum mode" );
  }
  // Need to map values to match those of the SEI message:
  if ( tmpDecodedPictureHashSEIMappedType == 0 ) {
    m_decodedPictureHashSEIType = HASHTYPE_NONE;
  } else {
    m_decodedPictureHashSEIType = HashType( tmpDecodedPictureHashSEIMappedType - 1 );
  }
  // Need to map values to match those of the SEI message:
  if ( tmpSubpicDecodedPictureHashMappedType == 0 ) {
    m_subpicDecodedPictureHashType = HASHTYPE_NONE;
  } else {
    m_subpicDecodedPictureHashType = HashType( tmpSubpicDecodedPictureHashMappedType - 1 );
  }
  // allocate slice-based dQP values
  m_aidQP = new int[m_framesToBeEncoded + m_iGOPSize + 1];
  ::memset( m_aidQP, 0, sizeof( int ) * ( m_framesToBeEncoded + m_iGOPSize + 1 ) );

#if QP_SWITCHING_FOR_PARALLEL
  if ( m_qpIncrementAtSourceFrame.bPresent ) {
    uint32_t switchingPOC = 0;
    if ( m_qpIncrementAtSourceFrame.value > m_FrameSkip ) {
      // if switch source frame (ssf) = 10, and frame skip (fs)=2 and temporal subsample ratio (tsr) =1, then
      //    for this simulation switch at POC 8 (=10-2).
      // if ssf=10, fs=2, tsr=2, then for this simulation, switch at POC 4 (=(10-2)/2): POC0=Src2, POC1=Src4, POC2=Src6,
      // POC3=Src8, POC4=Src10
      switchingPOC = ( m_qpIncrementAtSourceFrame.value - m_FrameSkip ) / m_temporalSubsampleRatio;
    }
    for ( uint32_t i = switchingPOC; i < ( m_framesToBeEncoded + m_iGOPSize + 1 ); i++ ) { m_aidQP[i] = 1; }
  }
#else
  // handling of floating-point QP values
  // if QP is not integer, sequence is split into two sections having QP and QP+1
  m_iQP = (int)( m_fQP );
  if ( m_iQP < m_fQP ) {
    int iSwitchPOC = (int)( m_framesToBeEncoded - ( m_fQP - m_iQP ) * m_framesToBeEncoded + 0.5 );

    iSwitchPOC = (int)( (double)iSwitchPOC / m_iGOPSize + 0.5 ) * m_iGOPSize;
    for ( int i = iSwitchPOC; i < m_framesToBeEncoded + m_iGOPSize + 1; i++ ) { m_aidQP[i] = 1; }
  }
#endif

#if SHARP_LUMA_DELTA_QP
  CHECK( lumaLevelToDeltaQPMode >= LUMALVL_TO_DQP_NUM_MODES, "Error in cfg" );

  m_lumaLevelToDeltaQPMapping.mode = LumaLevelToDQPMode( lumaLevelToDeltaQPMode );

  if ( m_lumaLevelToDeltaQPMapping.mode ) {
    CHECK( cfg_lumaLeveltoDQPMappingLuma.values.size() != cfg_lumaLeveltoDQPMappingQP.values.size(), "Error in cfg" );
    m_lumaLevelToDeltaQPMapping.mapping.resize( cfg_lumaLeveltoDQPMappingLuma.values.size() );
    for ( uint32_t i = 0; i < cfg_lumaLeveltoDQPMappingLuma.values.size(); i++ ) {
      m_lumaLevelToDeltaQPMapping.mapping[i] =
          std::pair<int, int>( cfg_lumaLeveltoDQPMappingLuma.values[i], cfg_lumaLeveltoDQPMappingQP.values[i] );
    }
  }
#endif

  CHECK( cfg_qpInValCb.values.size() != cfg_qpOutValCb.values.size(), "Chroma QP table for Cb is incomplete." );
  CHECK( cfg_qpInValCr.values.size() != cfg_qpOutValCr.values.size(), "Chroma QP table for Cr is incomplete." );
  CHECK( cfg_qpInValCbCr.values.size() != cfg_qpOutValCbCr.values.size(), "Chroma QP table for CbCr is incomplete." );
  if ( m_useIdentityTableForNon420Chroma && m_chromaFormatIDC != CHROMA_420 ) {
    m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag = true;

    cfg_qpInValCb.values    = {26};
    cfg_qpInValCr.values    = {26};
    cfg_qpInValCbCr.values  = {26};
    cfg_qpOutValCb.values   = {26};
    cfg_qpOutValCr.values   = {26};
    cfg_qpOutValCbCr.values = {26};
  }

  // Need to have at least 2 points in the set. Add second one if only one given
  if ( cfg_qpInValCb.values.size() == 1 ) {
    cfg_qpInValCb.values.push_back( cfg_qpInValCb.values[0] + 1 );
    cfg_qpOutValCb.values.push_back( cfg_qpOutValCb.values[0] + 1 );
  }
  if ( cfg_qpInValCr.values.size() == 1 ) {
    cfg_qpInValCr.values.push_back( cfg_qpInValCr.values[0] + 1 );
    cfg_qpOutValCr.values.push_back( cfg_qpOutValCr.values[0] + 1 );
  }
  if ( cfg_qpInValCbCr.values.size() == 1 ) {
    cfg_qpInValCbCr.values.push_back( cfg_qpInValCbCr.values[0] + 1 );
    cfg_qpOutValCbCr.values.push_back( cfg_qpOutValCbCr.values[0] + 1 );
  }

  int qpBdOffsetC = 6 * ( m_internalBitDepth[CHANNEL_TYPE_CHROMA] - 8 );
  m_chromaQpMappingTableParams.m_deltaQpInValMinus1[0].resize( cfg_qpInValCb.values.size() );
  m_chromaQpMappingTableParams.m_deltaQpOutVal[0].resize( cfg_qpOutValCb.values.size() );
  m_chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[0] = (int)cfg_qpOutValCb.values.size() - 2;
  m_chromaQpMappingTableParams.m_qpTableStartMinus26[0]    = -26 + cfg_qpInValCb.values[0];
  CHECK( m_chromaQpMappingTableParams.m_qpTableStartMinus26[0] < -26 - qpBdOffsetC ||
             m_chromaQpMappingTableParams.m_qpTableStartMinus26[0] > 36,
         "qpTableStartMinus26[0] is out of valid range of -26 -qpBdOffsetC to 36, inclusive." )
  CHECK( cfg_qpInValCb.values[0] != cfg_qpOutValCb.values[0],
         "First qpInValCb value should be equal to first qpOutValCb value" );
  for ( int i = 0; i < cfg_qpInValCb.values.size() - 1; i++ ) {
    CHECK( cfg_qpInValCb.values[i] < -qpBdOffsetC || cfg_qpInValCb.values[i] > MAX_QP,
           "Some entries cfg_qpInValCb are out of valid range of -qpBdOffsetC to 63, inclusive." );
    CHECK( cfg_qpOutValCb.values[i] < -qpBdOffsetC || cfg_qpOutValCb.values[i] > MAX_QP,
           "Some entries cfg_qpOutValCb are out of valid range of -qpBdOffsetC to 63, inclusive." );
    m_chromaQpMappingTableParams.m_deltaQpInValMinus1[0][i] = cfg_qpInValCb.values[i + 1] - cfg_qpInValCb.values[i] - 1;
    m_chromaQpMappingTableParams.m_deltaQpOutVal[0][i]      = cfg_qpOutValCb.values[i + 1] - cfg_qpOutValCb.values[i];
  }
  if ( !m_chromaQpMappingTableParams.m_sameCQPTableForAllChromaFlag ) {
    m_chromaQpMappingTableParams.m_deltaQpInValMinus1[1].resize( cfg_qpInValCr.values.size() );
    m_chromaQpMappingTableParams.m_deltaQpOutVal[1].resize( cfg_qpOutValCr.values.size() );
    m_chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[1] = (int)cfg_qpOutValCr.values.size() - 2;
    m_chromaQpMappingTableParams.m_qpTableStartMinus26[1]    = -26 + cfg_qpInValCr.values[0];
    CHECK( m_chromaQpMappingTableParams.m_qpTableStartMinus26[1] < -26 - qpBdOffsetC ||
               m_chromaQpMappingTableParams.m_qpTableStartMinus26[1] > 36,
           "qpTableStartMinus26[1] is out of valid range of -26 -qpBdOffsetC to 36, inclusive." )
    CHECK( cfg_qpInValCr.values[0] != cfg_qpOutValCr.values[0],
           "First qpInValCr value should be equal to first qpOutValCr value" );
    for ( int i = 0; i < cfg_qpInValCr.values.size() - 1; i++ ) {
      CHECK( cfg_qpInValCr.values[i] < -qpBdOffsetC || cfg_qpInValCr.values[i] > MAX_QP,
             "Some entries cfg_qpInValCr are out of valid range of -qpBdOffsetC to 63, inclusive." );
      CHECK( cfg_qpOutValCr.values[i] < -qpBdOffsetC || cfg_qpOutValCr.values[i] > MAX_QP,
             "Some entries cfg_qpOutValCr are out of valid range of -qpBdOffsetC to 63, inclusive." );
      m_chromaQpMappingTableParams.m_deltaQpInValMinus1[1][i] =
          cfg_qpInValCr.values[i + 1] - cfg_qpInValCr.values[i] - 1;
      m_chromaQpMappingTableParams.m_deltaQpOutVal[1][i] = cfg_qpOutValCr.values[i + 1] - cfg_qpOutValCr.values[i];
    }
    m_chromaQpMappingTableParams.m_deltaQpInValMinus1[2].resize( cfg_qpInValCbCr.values.size() );
    m_chromaQpMappingTableParams.m_deltaQpOutVal[2].resize( cfg_qpOutValCbCr.values.size() );
    m_chromaQpMappingTableParams.m_numPtsInCQPTableMinus1[2] = (int)cfg_qpOutValCbCr.values.size() - 2;
    m_chromaQpMappingTableParams.m_qpTableStartMinus26[2]    = -26 + cfg_qpInValCbCr.values[0];
    CHECK( m_chromaQpMappingTableParams.m_qpTableStartMinus26[2] < -26 - qpBdOffsetC ||
               m_chromaQpMappingTableParams.m_qpTableStartMinus26[2] > 36,
           "qpTableStartMinus26[2] is out of valid range of -26 -qpBdOffsetC to 36, inclusive." )
    CHECK( cfg_qpInValCbCr.values[0] != cfg_qpInValCbCr.values[0],
           "First qpInValCbCr value should be equal to first qpOutValCbCr value" );
    for ( int i = 0; i < cfg_qpInValCbCr.values.size() - 1; i++ ) {
      CHECK( cfg_qpInValCbCr.values[i] < -qpBdOffsetC || cfg_qpInValCbCr.values[i] > MAX_QP,
             "Some entries cfg_qpInValCbCr are out of valid range of -qpBdOffsetC to 63, inclusive." );
      CHECK( cfg_qpOutValCbCr.values[i] < -qpBdOffsetC || cfg_qpOutValCbCr.values[i] > MAX_QP,
             "Some entries cfg_qpOutValCbCr are out of valid range of -qpBdOffsetC to 63, inclusive." );
      m_chromaQpMappingTableParams.m_deltaQpInValMinus1[2][i] =
          cfg_qpInValCbCr.values[i + 1] - cfg_qpInValCbCr.values[i] - 1;
      m_chromaQpMappingTableParams.m_deltaQpOutVal[2][i] = cfg_qpInValCbCr.values[i + 1] - cfg_qpInValCbCr.values[i];
    }
  }

  /* Local chroma QP offsets configuration */
  CHECK( m_cuChromaQpOffsetSubdiv < 0, "MaxCuChromaQpOffsetSubdiv shall be >= 0" );
  CHECK( cfg_crQpOffsetList.values.size() != cfg_cbQpOffsetList.values.size(),
         "Chroma QP offset lists shall be the same size" );
  CHECK(
      cfg_cbCrQpOffsetList.values.size() != cfg_cbQpOffsetList.values.size() && cfg_cbCrQpOffsetList.values.size() > 0,
      "Chroma QP offset list for joint CbCr shall be either the same size as Cb and Cr or empty" );
  if ( m_cuChromaQpOffsetSubdiv > 0 && !cfg_cbQpOffsetList.values.size() ) {
    msg( VTM_WARNING, "MaxCuChromaQpOffsetSubdiv has no effect when chroma QP offset lists are empty\n" );
  }
  m_cuChromaQpOffsetList.resize( cfg_cbQpOffsetList.values.size() );
  for ( int i = 0; i < cfg_cbQpOffsetList.values.size(); i++ ) {
    m_cuChromaQpOffsetList[i].u.comp.CbOffset = cfg_cbQpOffsetList.values[i];
    m_cuChromaQpOffsetList[i].u.comp.CrOffset = cfg_crQpOffsetList.values[i];
    m_cuChromaQpOffsetList[i].u.comp.JointCbCrOffset =
        cfg_cbCrQpOffsetList.values.size() ? cfg_cbCrQpOffsetList.values[i] : 0;
  }

#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  if ( m_LadfEnabed ) {
    CHECK( m_LadfNumIntervals != cfg_LadfQpOffset.values.size(),
           "size of LadfQpOffset must be equal to LadfNumIntervals" );
    CHECK( m_LadfNumIntervals - 1 != cfg_LadfIntervalLowerBound.values.size(),
           "size of LadfIntervalLowerBound must be equal to LadfNumIntervals - 1" );
    m_LadfQpOffset              = cfg_LadfQpOffset.values;
    m_LadfIntervalLowerBound[0] = 0;
    for ( int k = 1; k < m_LadfNumIntervals; k++ ) {
      m_LadfIntervalLowerBound[k] = cfg_LadfIntervalLowerBound.values[k - 1];
    }
  }
#endif

  if ( m_chromaFormatIDC != CHROMA_420 ) {
    if ( !m_horCollocatedChromaFlag ) {
      msg( VTM_WARNING, "\nWARNING: HorCollocatedChroma is forced to 1 for chroma formats other than 4:2:0\n" );
      m_horCollocatedChromaFlag = true;
    }
    if ( !m_verCollocatedChromaFlag ) {
      msg( VTM_WARNING, "\nWARNING: VerCollocatedChroma is forced to 1 for chroma formats other than 4:2:0\n" );
      m_verCollocatedChromaFlag = true;
    }
  }
#if JVET_O0756_CONFIG_HDRMETRICS && !JVET_O0756_CALCULATE_HDRMETRICS
  if ( m_calculateHdrMetrics == true ) {
    printf(
        "Warning: Configuration enables HDR metric calculations.  However, HDR metric support was not linked when "
        "compiling the VTM.\n" );
    m_calculateHdrMetrics = false;
  }
#endif

#if GDR_ENABLED
  if ( m_gdrEnabled ) {
    m_virtualBoundariesEnabledFlag = 1;
    m_virtualBoundariesPresentFlag = 0;
  } else {
    m_virtualBoundariesEnabledFlag = 0;
  }
#else
  m_virtualBoundariesEnabledFlag = 0;
#endif

  if ( m_numVerVirtualBoundaries > 0 || m_numHorVirtualBoundaries > 0 ) m_virtualBoundariesEnabledFlag = 1;

  if ( m_virtualBoundariesEnabledFlag ) {
    CHECK( m_subPicInfoPresentFlag && m_virtualBoundariesPresentFlag != 1,
           "When subpicture signalling is present, the signalling of virtual boundaries, if present, shall be in the "
           "SPS" );

    if ( m_virtualBoundariesPresentFlag ) {
      CHECK( m_numVerVirtualBoundaries > 3,
             "Number of vertical virtual boundaries must be comprised between 0 and 3 included" );
      CHECK( m_numHorVirtualBoundaries > 3,
             "Number of horizontal virtual boundaries must be comprised between 0 and 3 included" );
      CHECK( m_numVerVirtualBoundaries != cfg_virtualBoundariesPosX.values.size(),
             "Size of VirtualBoundariesPosX must be equal to NumVerVirtualBoundaries" );
      CHECK( m_numHorVirtualBoundaries != cfg_virtualBoundariesPosY.values.size(),
             "Size of VirtualBoundariesPosY must be equal to NumHorVirtualBoundaries" );
      m_virtualBoundariesPosX = cfg_virtualBoundariesPosX.values;
      if ( m_numVerVirtualBoundaries > 1 ) { sort( m_virtualBoundariesPosX.begin(), m_virtualBoundariesPosX.end() ); }
      for ( unsigned i = 0; i < m_numVerVirtualBoundaries; i++ ) {
        CHECK( m_virtualBoundariesPosX[i] == 0 || m_virtualBoundariesPosX[i] >= m_sourceWidth,
               "The vertical virtual boundary must be within the picture" );
        CHECK( m_virtualBoundariesPosX[i] % 8, "The vertical virtual boundary must be a multiple of 8 luma samples" );
        if ( i > 0 ) {
          CHECK( m_virtualBoundariesPosX[i] - m_virtualBoundariesPosX[i - 1] < m_uiCTUSize,
                 "The distance between any two vertical virtual boundaries shall be greater than or equal to the CTU "
                 "size" );
        }
      }
      m_virtualBoundariesPosY = cfg_virtualBoundariesPosY.values;
      if ( m_numHorVirtualBoundaries > 1 ) { sort( m_virtualBoundariesPosY.begin(), m_virtualBoundariesPosY.end() ); }
      for ( unsigned i = 0; i < m_numHorVirtualBoundaries; i++ ) {
        CHECK( m_virtualBoundariesPosY[i] == 0 || m_virtualBoundariesPosY[i] >= m_sourceHeight,
               "The horizontal virtual boundary must be within the picture" );
        CHECK( m_virtualBoundariesPosY[i] % 8, "The horizontal virtual boundary must be a multiple of 8 luma samples" );
        if ( i > 0 ) {
          CHECK( m_virtualBoundariesPosY[i] - m_virtualBoundariesPosY[i - 1] < m_uiCTUSize,
                 "The distance between any two horizontal virtual boundaries shall be greater than or equal to the CTU "
                 "size" );
        }
      }
    }
  }

  if ( m_alf ) {
    CHECK( m_maxNumAlfAlternativesChroma < 1 || m_maxNumAlfAlternativesChroma > MAX_NUM_ALF_ALTERNATIVES_CHROMA,
           std::string( "The maximum number of ALF Chroma filter alternatives must be in the range (1-" ) +
               std::to_string( MAX_NUM_ALF_ALTERNATIVES_CHROMA ) + std::string( ", inclusive)" ) );
  }

  // reading external dQP description from file
  if ( !m_dQPFileName.empty() ) {
    FILE* fpt = fopen( m_dQPFileName.c_str(), "r" );
    if ( fpt ) {
      int iValue;
      int iPOC = 0;
      while ( iPOC < m_framesToBeEncoded ) {
        if ( fscanf( fpt, "%d", &iValue ) == EOF ) { break; }
        m_aidQP[iPOC] = iValue;
        iPOC++;
      }
      fclose( fpt );
    }
  }

  if ( m_masteringDisplay.colourVolumeSEIEnabled ) {
    for ( uint32_t idx = 0; idx < 6; idx++ ) {
      m_masteringDisplay.primaries[idx / 2][idx % 2] =
          uint16_t( ( cfg_DisplayPrimariesCode.values.size() > idx ) ? cfg_DisplayPrimariesCode.values[idx] : 0 );
    }
    for ( uint32_t idx = 0; idx < 2; idx++ ) {
      m_masteringDisplay.whitePoint[idx] =
          uint16_t( ( cfg_DisplayWhitePointCode.values.size() > idx ) ? cfg_DisplayWhitePointCode.values[idx] : 0 );
    }
  }
  if ( m_omniViewportSEIEnabled && !m_omniViewportSEICancelFlag ) {
    CHECK( !( m_omniViewportSEICntMinus1 >= 0 && m_omniViewportSEICntMinus1 < 16 ),
           "SEIOmniViewportCntMinus1 must be in the range of 0 to 16" );
    m_omniViewportSEIAzimuthCentre.resize( m_omniViewportSEICntMinus1 + 1 );
    m_omniViewportSEIElevationCentre.resize( m_omniViewportSEICntMinus1 + 1 );
    m_omniViewportSEITiltCentre.resize( m_omniViewportSEICntMinus1 + 1 );
    m_omniViewportSEIHorRange.resize( m_omniViewportSEICntMinus1 + 1 );
    m_omniViewportSEIVerRange.resize( m_omniViewportSEICntMinus1 + 1 );
    for ( int i = 0; i < ( m_omniViewportSEICntMinus1 + 1 ); i++ ) {
      m_omniViewportSEIAzimuthCentre[i] =
          cfg_omniViewportSEIAzimuthCentre.values.size() > i ? cfg_omniViewportSEIAzimuthCentre.values[i] : 0;
      m_omniViewportSEIElevationCentre[i] =
          cfg_omniViewportSEIElevationCentre.values.size() > i ? cfg_omniViewportSEIElevationCentre.values[i] : 0;
      m_omniViewportSEITiltCentre[i] =
          cfg_omniViewportSEITiltCentre.values.size() > i ? cfg_omniViewportSEITiltCentre.values[i] : 0;
      m_omniViewportSEIHorRange[i] =
          cfg_omniViewportSEIHorRange.values.size() > i ? cfg_omniViewportSEIHorRange.values[i] : 0;
      m_omniViewportSEIVerRange[i] =
          cfg_omniViewportSEIVerRange.values.size() > i ? cfg_omniViewportSEIVerRange.values[i] : 0;
    }
  }

  if ( !m_rwpSEIRwpCancelFlag && m_rwpSEIEnabled ) {
    CHECK( !( m_rwpSEINumPackedRegions > 0 && m_rwpSEINumPackedRegions <= std::numeric_limits<uint8_t>::max() ),
           "SEIRwpNumPackedRegions must be in the range of 1 to 255" );
    CHECK( !( cfg_rwpSEIRwpTransformType.values.size() == m_rwpSEINumPackedRegions ),
           "Number of must SEIRwpTransformType values be equal to SEIRwpNumPackedRegions" );
    CHECK( !( cfg_rwpSEIRwpGuardBandFlag.values.size() == m_rwpSEINumPackedRegions ),
           "Number of must SEIRwpGuardBandFlag values must be equal to SEIRwpNumPackedRegions" );
    CHECK( !( cfg_rwpSEIProjRegionWidth.values.size() == m_rwpSEINumPackedRegions ),
           "Number of must SEIProjRegionWidth values must be equal to SEIRwpNumPackedRegions" );
    CHECK( !( cfg_rwpSEIProjRegionHeight.values.size() == m_rwpSEINumPackedRegions ),
           "Number of must SEIProjRegionHeight values must be equal to SEIRwpNumPackedRegions" );
    CHECK( !( cfg_rwpSEIRwpSEIProjRegionTop.values.size() == m_rwpSEINumPackedRegions ),
           "Number of must SEIRwpSEIProjRegionTop values must be equal to SEIRwpNumPackedRegions" );
    CHECK( !( cfg_rwpSEIProjRegionLeft.values.size() == m_rwpSEINumPackedRegions ),
           "Number of must SEIProjRegionLeft values must be equal to SEIRwpNumPackedRegions" );
    CHECK( !( cfg_rwpSEIPackedRegionWidth.values.size() == m_rwpSEINumPackedRegions ),
           "Number of must SEIPackedRegionWidth values must be equal to SEIRwpNumPackedRegions" );
    CHECK( !( cfg_rwpSEIPackedRegionHeight.values.size() == m_rwpSEINumPackedRegions ),
           "Number of must SEIPackedRegionHeight values must be equal to SEIRwpNumPackedRegions" );
    CHECK( !( cfg_rwpSEIPackedRegionTop.values.size() == m_rwpSEINumPackedRegions ),
           "Number of must SEIPackedRegionTop values must be equal to SEIRwpNumPackedRegions" );
    CHECK( !( cfg_rwpSEIPackedRegionLeft.values.size() == m_rwpSEINumPackedRegions ),
           "Number of must SEIPackedRegionLeft values must be equal to SEIRwpNumPackedRegions" );

    m_rwpSEIRwpTransformType.resize( m_rwpSEINumPackedRegions );
    m_rwpSEIRwpGuardBandFlag.resize( m_rwpSEINumPackedRegions );
    m_rwpSEIProjRegionWidth.resize( m_rwpSEINumPackedRegions );
    m_rwpSEIProjRegionHeight.resize( m_rwpSEINumPackedRegions );
    m_rwpSEIRwpSEIProjRegionTop.resize( m_rwpSEINumPackedRegions );
    m_rwpSEIProjRegionLeft.resize( m_rwpSEINumPackedRegions );
    m_rwpSEIPackedRegionWidth.resize( m_rwpSEINumPackedRegions );
    m_rwpSEIPackedRegionHeight.resize( m_rwpSEINumPackedRegions );
    m_rwpSEIPackedRegionTop.resize( m_rwpSEINumPackedRegions );
    m_rwpSEIPackedRegionLeft.resize( m_rwpSEINumPackedRegions );
    m_rwpSEIRwpLeftGuardBandWidth.resize( m_rwpSEINumPackedRegions );
    m_rwpSEIRwpRightGuardBandWidth.resize( m_rwpSEINumPackedRegions );
    m_rwpSEIRwpTopGuardBandHeight.resize( m_rwpSEINumPackedRegions );
    m_rwpSEIRwpBottomGuardBandHeight.resize( m_rwpSEINumPackedRegions );
    m_rwpSEIRwpGuardBandNotUsedForPredFlag.resize( m_rwpSEINumPackedRegions );
    m_rwpSEIRwpGuardBandType.resize( 4 * m_rwpSEINumPackedRegions );
    for ( int i = 0; i < m_rwpSEINumPackedRegions; i++ ) {
      m_rwpSEIRwpTransformType[i] = cfg_rwpSEIRwpTransformType.values[i];
      CHECK( !( m_rwpSEIRwpTransformType[i] >= 0 && m_rwpSEIRwpTransformType[i] <= 7 ),
             "SEIRwpTransformType must be in the range of 0 to 7" );
      m_rwpSEIRwpGuardBandFlag[i]    = cfg_rwpSEIRwpGuardBandFlag.values[i];
      m_rwpSEIProjRegionWidth[i]     = cfg_rwpSEIProjRegionWidth.values[i];
      m_rwpSEIProjRegionHeight[i]    = cfg_rwpSEIProjRegionHeight.values[i];
      m_rwpSEIRwpSEIProjRegionTop[i] = cfg_rwpSEIRwpSEIProjRegionTop.values[i];
      m_rwpSEIProjRegionLeft[i]      = cfg_rwpSEIProjRegionLeft.values[i];
      m_rwpSEIPackedRegionWidth[i]   = cfg_rwpSEIPackedRegionWidth.values[i];
      m_rwpSEIPackedRegionHeight[i]  = cfg_rwpSEIPackedRegionHeight.values[i];
      m_rwpSEIPackedRegionTop[i]     = cfg_rwpSEIPackedRegionTop.values[i];
      m_rwpSEIPackedRegionLeft[i]    = cfg_rwpSEIPackedRegionLeft.values[i];
      if ( m_rwpSEIRwpGuardBandFlag[i] ) {
        m_rwpSEIRwpLeftGuardBandWidth[i]    = cfg_rwpSEIRwpLeftGuardBandWidth.values[i];
        m_rwpSEIRwpRightGuardBandWidth[i]   = cfg_rwpSEIRwpRightGuardBandWidth.values[i];
        m_rwpSEIRwpTopGuardBandHeight[i]    = cfg_rwpSEIRwpTopGuardBandHeight.values[i];
        m_rwpSEIRwpBottomGuardBandHeight[i] = cfg_rwpSEIRwpBottomGuardBandHeight.values[i];
        CHECK( !( m_rwpSEIRwpLeftGuardBandWidth[i] > 0 || m_rwpSEIRwpRightGuardBandWidth[i] > 0 ||
                  m_rwpSEIRwpTopGuardBandHeight[i] > 0 || m_rwpSEIRwpBottomGuardBandHeight[i] > 0 ),
               "At least one of the RWP guard band parameters mut be greater than zero" );
        m_rwpSEIRwpGuardBandNotUsedForPredFlag[i] = cfg_rwpSEIRwpGuardBandNotUsedForPredFlag.values[i];
        for ( int j = 0; j < 4; j++ ) {
          m_rwpSEIRwpGuardBandType[i * 4 + j] = cfg_rwpSEIRwpGuardBandType.values[i * 4 + j];
        }
      }
    }
  }
  if ( m_gcmpSEIEnabled && !m_gcmpSEICancelFlag ) {
    int numFace = m_gcmpSEIPackingType == 4 || m_gcmpSEIPackingType == 5 ? 5 : 6;
    CHECK( !( cfg_gcmpSEIFaceIndex.values.size() == numFace ),
           "Number of SEIGcmpFaceIndex must be equal to 5 when SEIGcmpPackingType is equal to 4 or 5, otherwise, it "
           "must be equal to 6" );
    CHECK( !( cfg_gcmpSEIFaceRotation.values.size() == numFace ),
           "Number of SEIGcmpFaceRotation must be equal to 5 when SEIGcmpPackingType is equal to 4 or 5, otherwise, it "
           "must be equal to 6" );
    m_gcmpSEIFaceIndex.resize( numFace );
    m_gcmpSEIFaceRotation.resize( numFace );
    if ( m_gcmpSEIMappingFunctionType == 2 ) {
      CHECK( !( cfg_gcmpSEIFunctionCoeffU.values.size() == numFace ),
             "Number of SEIGcmpFunctionCoeffU must be equal to 5 when SEIGcmpPackingType is equal to 4 or 5, "
             "otherwise, it must be equal to 6" );
      CHECK( !( cfg_gcmpSEIFunctionUAffectedByVFlag.values.size() == numFace ),
             "Number of SEIGcmpFunctionUAffectedByVFlag must be equal to 5 when SEIGcmpPackingType is equal to 4 or 5, "
             "otherwise, it must be equal to 6" );
      CHECK( !( cfg_gcmpSEIFunctionCoeffV.values.size() == numFace ),
             "Number of SEIGcmpFunctionCoeffV must be equal to 5 when SEIGcmpPackingType is equal to 4 or 5, "
             "otherwise, it must be equal to 6" );
      CHECK( !( cfg_gcmpSEIFunctionVAffectedByUFlag.values.size() == numFace ),
             "Number of SEIGcmpFunctionVAffectedByUFlag must be equal to 5 when SEIGcmpPackingType is equal to 4 or 5, "
             "otherwise, it must be equal to 6" );
      m_gcmpSEIFunctionCoeffU.resize( numFace );
      m_gcmpSEIFunctionUAffectedByVFlag.resize( numFace );
      m_gcmpSEIFunctionCoeffV.resize( numFace );
      m_gcmpSEIFunctionVAffectedByUFlag.resize( numFace );
    }
    for ( int i = 0; i < numFace; i++ ) {
      m_gcmpSEIFaceIndex[i]    = cfg_gcmpSEIFaceIndex.values[i];
      m_gcmpSEIFaceRotation[i] = cfg_gcmpSEIFaceRotation.values[i];
      if ( m_gcmpSEIMappingFunctionType == 2 ) {
        m_gcmpSEIFunctionCoeffU[i]           = cfg_gcmpSEIFunctionCoeffU.values[i];
        m_gcmpSEIFunctionUAffectedByVFlag[i] = cfg_gcmpSEIFunctionUAffectedByVFlag.values[i];
        m_gcmpSEIFunctionCoeffV[i]           = cfg_gcmpSEIFunctionCoeffV.values[i];
        m_gcmpSEIFunctionVAffectedByUFlag[i] = cfg_gcmpSEIFunctionVAffectedByUFlag.values[i];
      }
    }
  }
  m_reshapeCW.binCW.resize( 3 );
  m_reshapeCW.rspFps     = m_iFrameRate;
  m_reshapeCW.rspPicSize = m_sourceWidth * m_sourceHeight;
  m_reshapeCW.rspFpsToIp = std::max( 16, 16 * (int)( round( (double)m_iFrameRate / 16.0 ) ) );
  m_reshapeCW.rspBaseQP  = m_iQP;
  m_reshapeCW.updateCtrl = m_updateCtrl;
  m_reshapeCW.adpOption  = m_adpOption;
  m_reshapeCW.initialCW  = m_initialCW;
#if ENABLE_TRACING
  g_trace_ctx = tracing_init( sTracingFile, sTracingRule );
  if ( bTracingChannelsList && g_trace_ctx ) {
    std::string sChannelsList;
    g_trace_ctx->getChannelsList( sChannelsList );
    msg( VTM_INFO, "\n Using tracing channels:\n\n%s\n", sChannelsList.c_str() );
  }
#endif

#if ENABLE_QPA
  if ( m_bUsePerceptQPA && !m_bUseAdaptiveQP && m_dualTree &&
       ( m_cbQpOffsetDualTree != 0 || m_crQpOffsetDualTree != 0 || m_cbCrQpOffsetDualTree != 0 ) ) {
    msg( VTM_WARNING, "*************************************************************************\n" );
    msg( VTM_WARNING, "* VTM_WARNING: chroma QPA on, ignoring nonzero dual-tree chroma QP offsets! *\n" );
    msg( VTM_WARNING, "*************************************************************************\n" );
  }

#if ENABLE_QPA_SUB_CTU
#if QP_SWITCHING_FOR_PARALLEL
  if ( ( m_iQP < 38 ) && m_bUsePerceptQPA && !m_bUseAdaptiveQP && ( m_sourceWidth <= 2048 ) &&
       ( m_sourceHeight <= 1280 )
#else
  if ( ( (int)m_fQP < 38 ) && m_bUsePerceptQPA && !m_bUseAdaptiveQP && ( m_sourceWidth <= 2048 ) &&
       ( m_sourceHeight <= 1280 )
#endif
#if WCG_EXT && ER_CHROMA_QP_WCG_PPS
       && ( !m_wcgChromaQpControl.enabled )
#endif
       && ( ( 1 << ( m_log2MaxTbSize + 1 ) ) == m_uiCTUSize ) && ( m_sourceWidth > 512 || m_sourceHeight > 320 ) ) {
    m_cuQpDeltaSubdiv = 2;
  }
#else
#if QP_SWITCHING_FOR_PARALLEL
  if ( ( m_iQP < 38 ) && ( m_iGOPSize > 4 ) && m_bUsePerceptQPA && !m_bUseAdaptiveQP && ( m_sourceHeight <= 1280 ) &&
       ( m_sourceWidth <= 2048 ) )
#else
  if ( ( (int)m_fQP < 38 ) && ( m_iGOPSize > 4 ) && m_bUsePerceptQPA && !m_bUseAdaptiveQP &&
       ( m_sourceHeight <= 1280 ) && ( m_sourceWidth <= 2048 ) )
#endif
  {
    msg( VTM_WARNING, "*************************************************************************\n" );
    msg( VTM_WARNING, "* VTM_WARNING: QPA on with large CTU for <=HD sequences, limiting CTU size! *\n" );
    msg( VTM_WARNING, "*************************************************************************\n" );

    m_uiCTUSize = m_uiMaxCUWidth;
    if ( ( 1u << m_log2MaxTbSize ) > m_uiCTUSize ) m_log2MaxTbSize--;
  }
#endif
#endif  // ENABLE_QPA

  if ( m_costMode == COST_LOSSLESS_CODING ) {
    bool firstSliceLossless = false;
    if ( m_mixedLossyLossless ) {
      if ( m_sliceLosslessArray.size() > 0 ) {
        for ( uint32_t i = 0; i < m_sliceLosslessArray.size(); i++ ) {
          if ( m_sliceLosslessArray[i] == 0 ) {
            firstSliceLossless = true;
            break;
          }
        }
      }
    } else {
      firstSliceLossless = true;
    }
    if ( firstSliceLossless )  // if first slice is lossless
      m_iQP = LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP - ( ( m_internalBitDepth[CHANNEL_TYPE_LUMA] - 8 ) * 6 );
  }

  m_uiMaxCUWidth = m_uiMaxCUHeight = m_uiCTUSize;

  // check validity of input parameters
  if ( xCheckParameter() ) {
    // return check failed
    return false;
  }

  // print-out parameters
  xPrintParameter();

  return true;
}

// ====================================================================================================================
// Private member functions
// ====================================================================================================================

///< auto determine the profile to use given the other configuration settings. Returns 1 if erred. Can select profile
///< 'NONE'

int PCCVTMLibVideoEncoderCfg::xAutoDetermineProfile() {
  const int maxBitDepth = std::max(
      m_internalBitDepth[CHANNEL_TYPE_LUMA],
      m_internalBitDepth[m_chromaFormatIDC == ChromaFormat::CHROMA_400 ? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA] );
  m_profile = Profile::NONE;

  switch ( m_chromaFormatIDC ) {
    case ChromaFormat::CHROMA_400:
    case ChromaFormat::CHROMA_420:
      if ( maxBitDepth <= 10 ) {
        if ( m_level == Level::LEVEL15_5 && m_framesToBeEncoded == 1 ) {
          m_profile = m_maxLayers > 1 ? Profile::MULTILAYER_MAIN_10_STILL_PICTURE : Profile::MAIN_10_STILL_PICTURE;
        } else {
          m_profile = m_maxLayers > 1 ? Profile::MULTILAYER_MAIN_10 : Profile::MAIN_10;
        }
      }
      break;

    case ChromaFormat::CHROMA_422:
    case ChromaFormat::CHROMA_444:
      if ( maxBitDepth <= 10 ) {
        if ( m_level == Level::LEVEL15_5 && m_framesToBeEncoded == 1 ) {
          m_profile =
              m_maxLayers > 1 ? Profile::MULTILAYER_MAIN_10_444_STILL_PICTURE : Profile::MAIN_10_444_STILL_PICTURE;
        } else {
          m_profile = m_maxLayers > 1 ? Profile::MULTILAYER_MAIN_10_444 : Profile::MAIN_10_444;
        }
      }
      break;

    default: return 1;
  }

  return 0;
}

bool PCCVTMLibVideoEncoderCfg::xCheckParameter() {
  msg( VTM_NOTICE, "\n" );
  if ( m_decodedPictureHashSEIType == HASHTYPE_NONE ) {
    msg( VTM_DETAILS, "******************************************************************\n" );
    msg( VTM_DETAILS, "** VTM_WARNING: --SEIDecodedPictureHash is now disabled by default. **\n" );
    msg( VTM_DETAILS, "**          Automatic verification of decoded pictures by a     **\n" );
    msg( VTM_DETAILS, "**          decoder requires this option to be enabled.         **\n" );
    msg( VTM_DETAILS, "******************************************************************\n" );
  }
  if ( m_profile == Profile::NONE ) {
    msg( VTM_DETAILS, "***************************************************************************\n" );
    msg( VTM_DETAILS, "** VTM_WARNING: For conforming bitstreams a valid Profile value must be set! **\n" );
    msg( VTM_DETAILS, "***************************************************************************\n" );
  }
  if ( m_level == Level::NONE ) {
    msg( VTM_DETAILS, "***************************************************************************\n" );
    msg( VTM_DETAILS, "** VTM_WARNING: For conforming bitstreams a valid Level value must be set!   **\n" );
    msg( VTM_DETAILS, "***************************************************************************\n" );
  }

  bool check_failed = false; /* abort if there is a fatal configuration problem */
#define xConfirmPara( a, b ) check_failed |= confirmPara( a, b )

  if ( m_depQuantEnabledFlag ) {
    xConfirmPara( !m_useRDOQ || !m_useRDOQTS,
                  "RDOQ and RDOQTS must be equal to 1 if dependent quantization is enabled" );
    xConfirmPara( m_signDataHidingEnabledFlag, "SignHideFlag must be equal to 0 if dependent quantization is enabled" );
  }

  if ( m_wrapAround ) {
    const int minCUSize = 1 << m_log2MinCuSize;
    xConfirmPara( m_wrapAroundOffset <= m_uiCTUSize + minCUSize,
                  "Wrap-around offset must be greater than CtbSizeY + MinCbSize" );
    xConfirmPara( m_wrapAroundOffset > m_sourceWidth,
                  "Wrap-around offset must not be greater than the source picture width" );
    xConfirmPara( m_wrapAroundOffset % minCUSize != 0,
                  "Wrap-around offset must be an integer multiple of the specified minimum CU size" );
  }

#if SHARP_LUMA_DELTA_QP && ENABLE_QPA
  xConfirmPara( m_bUsePerceptQPA && m_lumaLevelToDeltaQPMapping.mode >= 2,
                "QPA and SharpDeltaQP mode 2 cannot be used together" );
  if ( m_bUsePerceptQPA && m_lumaLevelToDeltaQPMapping.mode == LUMALVL_TO_DQP_AVG_METHOD ) {
    msg( VTM_WARNING, "*********************************************************************************\n" );
    msg( VTM_WARNING, "** VTM_WARNING: Applying custom luma-based QPA with activity-based perceptual QPA! **\n" );
    msg( VTM_WARNING, "*********************************************************************************\n" );

    m_lumaLevelToDeltaQPMapping.mode = LUMALVL_TO_DQP_NUM_MODES;  // special QPA mode
  }
#endif

  xConfirmPara( m_useAMaxBT && !m_SplitConsOverrideEnabledFlag,
                "AMaxBt can only be used with PartitionConstriantsOverride enabled" );

  xConfirmPara( m_bitstreamFileName.empty(), "A bitstream file name must be specified (BitstreamFile)" );
  xConfirmPara( m_internalBitDepth[CHANNEL_TYPE_CHROMA] != m_internalBitDepth[CHANNEL_TYPE_LUMA],
                "The internalBitDepth must be the same for luma and chroma" );
  if ( m_profile != Profile::NONE ) {
    xConfirmPara( m_log2MaxTransformSkipBlockSize >= 6,
                  "Transform Skip Log2 Max Size must be less or equal to 5 for given profile." );
    xConfirmPara( m_transformSkipRotationEnabledFlag == true,
                  "UseResidualRotation must not be enabled for given profile." );
    xConfirmPara( m_transformSkipContextEnabledFlag == true,
                  "UseSingleSignificanceMapContext must not be enabled for given profile." );
#if JVET_V0106_RRC_RICE
    xConfirmPara( m_rrcRiceExtensionEnableFlag == true,
                  "Extention of the Golomb-Rice parameter derivation for RRC must not be enabled for given profile." );
#endif
    xConfirmPara( m_persistentRiceAdaptationEnabledFlag == true,
                  "GolombRiceParameterAdaption must not be enabled for given profile." );
    xConfirmPara( m_extendedPrecisionProcessingFlag == true,
                  "UseExtendedPrecision must not be enabled for given profile." );
#if JVET_V0054_TSRC_RICE
    xConfirmPara( m_tsrcRicePresentFlag == true, "TSRCRicePresent must not be enabled for given profile." );
#endif
    xConfirmPara( m_highPrecisionOffsetsEnabledFlag == true,
                  "UseHighPrecisionPredictionWeighting must not be enabled for given profile." );
    xConfirmPara( m_enableIntraReferenceSmoothing == false,
                  "EnableIntraReferenceSmoothing must be enabled for given profile." );
    xConfirmPara( m_cabacBypassAlignmentEnabledFlag, "AlignCABACBeforeBypass cannot be enabled for given profile." );
  }

  // check range of parameters
  xConfirmPara( m_inputBitDepth[CHANNEL_TYPE_LUMA] < 8, "InputBitDepth must be at least 8" );
  xConfirmPara( m_inputBitDepth[CHANNEL_TYPE_CHROMA] < 8, "InputBitDepthC must be at least 8" );

  if ( ( m_internalBitDepth[CHANNEL_TYPE_LUMA] < m_inputBitDepth[CHANNEL_TYPE_LUMA] ) ||
       ( m_internalBitDepth[CHANNEL_TYPE_CHROMA] < m_inputBitDepth[CHANNEL_TYPE_CHROMA] ) ) {
    msg( VTM_WARNING, "*****************************************************************************\n" );
    msg( VTM_WARNING, "** VTM_WARNING: InternalBitDepth is set to the lower value than InputBitDepth! **\n" );
    msg( VTM_WARNING, "**          min_qp_prime_ts_minus4 will be clipped to 0 at the low end!    **\n" );
    msg( VTM_WARNING, "*****************************************************************************\n" );
  }

#if !RExt__HIGH_BIT_DEPTH_SUPPORT
  if ( m_extendedPrecisionProcessingFlag ) {
    for ( uint32_t channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++ ) {
      xConfirmPara( ( m_internalBitDepth[channelType] > 8 ),
                    "Model is not configured to support high enough internal accuracies - enable "
                    "RExt__HIGH_BIT_DEPTH_SUPPORT to use increased precision internal data types etc..." );
    }
  } else {
    for ( uint32_t channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++ ) {
      xConfirmPara( ( m_internalBitDepth[channelType] > 12 ),
                    "Model is not configured to support high enough internal accuracies - enable "
                    "RExt__HIGH_BIT_DEPTH_SUPPORT to use increased precision internal data types etc..." );
    }
  }
#endif

  xConfirmPara( ( m_MSBExtendedBitDepth[CHANNEL_TYPE_LUMA] < m_inputBitDepth[CHANNEL_TYPE_LUMA] ),
                "MSB-extended bit depth for luma channel (--MSBExtendedBitDepth) must be greater than or equal to "
                "input bit depth for luma channel (--InputBitDepth)" );
  xConfirmPara( ( m_MSBExtendedBitDepth[CHANNEL_TYPE_CHROMA] < m_inputBitDepth[CHANNEL_TYPE_CHROMA] ),
                "MSB-extended bit depth for chroma channel (--MSBExtendedBitDepthC) must be greater than or equal to "
                "input bit depth for chroma channel (--InputBitDepthC)" );

  xConfirmPara( m_chromaFormatIDC >= NUM_CHROMA_FORMAT, "ChromaFormatIDC must be either 400, 420, 422 or 444" );
  std::string sTempIPCSC = "InputColourSpaceConvert must be empty, " + getListOfColourSpaceConverts( true );
  xConfirmPara( m_inputColourSpaceConvert >= NUMBER_INPUT_COLOUR_SPACE_CONVERSIONS, sTempIPCSC.c_str() );
  xConfirmPara( m_InputChromaFormatIDC >= NUM_CHROMA_FORMAT,
                "InputChromaFormatIDC must be either 400, 420, 422 or 444" );
  xConfirmPara( m_iFrameRate <= 0, "Frame rate must be more than 1" );
  xConfirmPara( m_framesToBeEncoded <= 0, "Total Number Of Frames encoded must be more than 0" );
  xConfirmPara( m_framesToBeEncoded < m_switchPOC, "debug POC out of range" );

  xConfirmPara( m_iGOPSize < 1, "GOP Size must be greater or equal to 1" );
  xConfirmPara( m_iGOPSize > 1 && m_iGOPSize % 2, "GOP Size must be a multiple of 2, if GOP Size is greater than 1" );
  xConfirmPara( ( m_iIntraPeriod > 0 && m_iIntraPeriod < m_iGOPSize ) || m_iIntraPeriod == 0,
                "Intra period must be more than GOP size, or -1 , not 0" );
  xConfirmPara( m_drapPeriod < 0, "DRAP period must be greater or equal to 0" );
  xConfirmPara( m_iDecodingRefreshType < 0 || m_iDecodingRefreshType > 3,
                "Decoding Refresh Type must be comprised between 0 and 3 included" );

  if ( m_isField ) {
    if ( !m_frameFieldInfoSEIEnabled ) {
      msg( VTM_WARNING, "*************************************************************************************\n" );
      msg( VTM_WARNING, "** VTM_WARNING: Frame field information SEI should be enabled for field coding!        **\n" );
      msg( VTM_WARNING, "*************************************************************************************\n" );
    }
  }
  if ( m_pictureTimingSEIEnabled && ( !m_bufferingPeriodSEIEnabled ) ) {
    msg( VTM_WARNING, "****************************************************************************\n" );
    msg( VTM_WARNING, "** VTM_WARNING: Picture Timing SEI requires Buffering Period SEI. Disabling.  **\n" );
    msg( VTM_WARNING, "****************************************************************************\n" );
    m_pictureTimingSEIEnabled = false;
  }

  xConfirmPara( m_bufferingPeriodSEIEnabled == true && m_RCCpbSize == 0,
                "RCCpbSize must be greater than zero, when buffering period SEI is enabled" );

  xConfirmPara( m_log2MaxTransformSkipBlockSize < 2, "Transform Skip Log2 Max Size must be at least 2 (4x4)" );

  xConfirmPara( m_onePictureOnlyConstraintFlag && m_framesToBeEncoded != 1,
                "When onePictureOnlyConstraintFlag is true, the number of frames to be encoded must be 1" );
  if ( m_profile != Profile::NONE ) {
    const ProfileFeatures* features = ProfileFeatures::getProfileFeatures( m_profile );
    CHECK( features->profile != m_profile, "Profile not found" );
    xConfirmPara( m_level == Level::LEVEL15_5 && !features->canUseLevel15p5, "Profile does not support level 15.5" );
  }

  xConfirmPara( m_iQP < -6 * ( m_internalBitDepth[CHANNEL_TYPE_LUMA] - 8 ) || m_iQP > MAX_QP,
                "QP exceeds supported range (-QpBDOffsety to 63)" );
#if W0038_DB_OPT
  xConfirmPara(
      m_deblockingFilterMetric != 0 && ( m_deblockingFilterDisable || m_deblockingFilterOffsetInPPS ),
      "If DeblockingFilterMetric is non-zero then both LoopFilterDisable and LoopFilterOffsetInPPS must be 0" );
#else
  xConfirmPara( m_DeblockingFilterMetric && ( m_bLoopFilterDisable || m_loopFilterOffsetInPPS ),
                "If DeblockingFilterMetric is true then both LoopFilterDisable and LoopFilterOffsetInPPS must be 0" );
#endif
  xConfirmPara( m_deblockingFilterBetaOffsetDiv2 < -12 || m_deblockingFilterBetaOffsetDiv2 > 12,
                "Loop Filter Beta Offset div. 2 exceeds supported range (-12 to 12" );
  xConfirmPara( m_deblockingFilterTcOffsetDiv2 < -12 || m_deblockingFilterTcOffsetDiv2 > 12,
                "Loop Filter Tc Offset div. 2 exceeds supported range (-12 to 12)" );
  xConfirmPara( m_deblockingFilterCbBetaOffsetDiv2 < -12 || m_deblockingFilterCbBetaOffsetDiv2 > 12,
                "Loop Filter Beta Offset div. 2 exceeds supported range (-12 to 12" );
  xConfirmPara( m_deblockingFilterCbTcOffsetDiv2 < -12 || m_deblockingFilterCbTcOffsetDiv2 > 12,
                "Loop Filter Tc Offset div. 2 exceeds supported range (-12 to 12)" );
  xConfirmPara( m_deblockingFilterCrBetaOffsetDiv2 < -12 || m_deblockingFilterCrBetaOffsetDiv2 > 12,
                "Loop Filter Beta Offset div. 2 exceeds supported range (-12 to 12" );
  xConfirmPara( m_deblockingFilterCrTcOffsetDiv2 < -12 || m_deblockingFilterCrTcOffsetDiv2 > 12,
                "Loop Filter Tc Offset div. 2 exceeds supported range (-12 to 12)" );
  xConfirmPara( m_iSearchRange < 0, "Search Range must be more than 0" );
  xConfirmPara( m_bipredSearchRange < 0, "Bi-prediction refinement search range must be more than 0" );
  xConfirmPara( m_minSearchWindow < 0,
                "Minimum motion search window size for the adaptive window ME must be greater than or equal to 0" );
  xConfirmPara( m_iMaxDeltaQP > MAX_DELTA_QP, "Absolute Delta QP exceeds supported range (0 to 7)" );
#if ENABLE_QPA
  xConfirmPara( m_bUsePerceptQPA && m_uiDeltaQpRD > 0,
                "Perceptual QPA cannot be used together with slice-level multiple-QP optimization" );
#endif
#if SHARP_LUMA_DELTA_QP
  xConfirmPara( m_lumaLevelToDeltaQPMapping.mode && m_uiDeltaQpRD > 0,
                "Luma-level-based Delta QP cannot be used together with slice level multiple-QP optimization\n" );
  xConfirmPara( m_lumaLevelToDeltaQPMapping.mode && m_RCEnableRateControl,
                "Luma-level-based Delta QP cannot be used together with rate control\n" );
#endif
  if ( m_lumaLevelToDeltaQPMapping.mode && m_lmcsEnabled ) {
    msg( VTM_WARNING,
         "For HDR-PQ, LMCS should be used mutual-exclusively with Luma-level-based Delta QP. If use LMCS, turn lumaDQP "
         "off.\n" );
    m_lumaLevelToDeltaQPMapping.mode = LUMALVL_TO_DQP_DISABLED;
  }
  if ( !m_lmcsEnabled ) {
    m_reshapeSignalType = RESHAPE_SIGNAL_NULL;
    m_intraCMD          = 0;
  }
  if ( m_lmcsEnabled && m_reshapeSignalType == RESHAPE_SIGNAL_PQ ) {
    m_intraCMD = 1;
  } else if ( m_lmcsEnabled &&
              ( m_reshapeSignalType == RESHAPE_SIGNAL_SDR || m_reshapeSignalType == RESHAPE_SIGNAL_HLG ) ) {
    m_intraCMD = 0;
  } else {
    m_lmcsEnabled = false;
  }
  if ( m_lmcsEnabled ) {
    xConfirmPara( m_updateCtrl < 0, "Min. LMCS Update Control is 0" );
    xConfirmPara( m_updateCtrl > 2, "Max. LMCS Update Control is 2" );
    xConfirmPara( m_adpOption < 0, "Min. LMCS Adaptation Option is 0" );
    xConfirmPara( m_adpOption > 4, "Max. LMCS Adaptation Option is 4" );
    xConfirmPara( m_initialCW < 0, "Min. Initial Total Codeword is 0" );
    xConfirmPara( m_initialCW > 1023, "Max. Initial Total Codeword is 1023" );
    xConfirmPara( m_CSoffset < -7, "Min. LMCS Offset value is -7" );
    xConfirmPara( m_CSoffset > 7, "Max. LMCS Offset value is 7" );
    if ( m_updateCtrl > 0 && m_adpOption > 2 ) { m_adpOption -= 2; }
  }

  xConfirmPara( m_cbQpOffset < -12, "Min. Chroma Cb QP Offset is -12" );
  xConfirmPara( m_cbQpOffset > 12, "Max. Chroma Cb QP Offset is  12" );
  xConfirmPara( m_crQpOffset < -12, "Min. Chroma Cr QP Offset is -12" );
  xConfirmPara( m_crQpOffset > 12, "Max. Chroma Cr QP Offset is  12" );
  xConfirmPara( m_cbQpOffsetDualTree < -12, "Min. Chroma Cb QP Offset for dual tree is -12" );
  xConfirmPara( m_cbQpOffsetDualTree > 12, "Max. Chroma Cb QP Offset for dual tree is  12" );
  xConfirmPara( m_crQpOffsetDualTree < -12, "Min. Chroma Cr QP Offset for dual tree is -12" );
  xConfirmPara( m_crQpOffsetDualTree > 12, "Max. Chroma Cr QP Offset for dual tree is  12" );
  if ( m_dualTree && ( m_chromaFormatIDC == CHROMA_400 ) ) {
    msg( VTM_WARNING, "****************************************************************************\n" );
    msg( VTM_WARNING, "** VTM_WARNING: --DualITree has been disabled because the chromaFormat is 400 **\n" );
    msg( VTM_WARNING, "****************************************************************************\n" );
    m_dualTree = false;
  }
  if ( m_alf ) {
    xConfirmPara( m_alfStrengthLuma < 0.0,
                  "ALFStrengthLuma is less than 0. Valid range is 0.0 <= ALFStrengthLuma <= 1.0" );
    xConfirmPara( m_alfStrengthLuma > 1.0,
                  "ALFStrengthLuma is greater than 1. Valid range is 0.0 <= ALFStrengthLuma <= 1.0" );
  }
  if ( m_ccalf ) {
    xConfirmPara( m_ccalfStrength < 0.0, "CCALFStrength is less than 0. Valid range is 0.0 <= CCALFStrength <= 1.0" );
    xConfirmPara( m_ccalfStrength > 1.0,
                  "CCALFStrength is greater than 1. Valid range is 0.0 <= CCALFStrength <= 1.0" );
  }
  if ( m_alf ) {
    xConfirmPara( m_alfStrengthChroma < 0.0,
                  "ALFStrengthChroma is less than 0. Valid range is 0.0 <= ALFStrengthChroma <= 1.0" );
    xConfirmPara( m_alfStrengthChroma > 1.0,
                  "ALFStrengthChroma is greater than 1. Valid range is 0.0 <= ALFStrengthChroma <= 1.0" );
    xConfirmPara( m_alfStrengthTargetLuma < 0.0,
                  "ALFStrengthTargetLuma is less than 0. Valid range is 0.0 <= ALFStrengthTargetLuma <= 1.0" );
    xConfirmPara( m_alfStrengthTargetLuma > 1.0,
                  "ALFStrengthTargetLuma is greater than 1. Valid range is 0.0 <= ALFStrengthTargetLuma <= 1.0" );
    xConfirmPara( m_alfStrengthTargetChroma < 0.0,
                  "ALFStrengthTargetChroma is less than 0. Valid range is 0.0 <= ALFStrengthTargetChroma <= 1.0" );
    xConfirmPara( m_alfStrengthTargetChroma > 1.0,
                  "ALFStrengthTargetChroma is greater than 1. Valid range is 0.0 <= ALFStrengthTargetChroma <= 1.0" );
  }
  if ( m_ccalf ) {
    xConfirmPara( m_ccalfStrengthTarget < 0.0,
                  "CCALFStrengthTarget is less than 0. Valid range is 0.0 <= CCALFStrengthTarget <= 1.0" );
    xConfirmPara( m_ccalfStrengthTarget > 1.0,
                  "CCALFStrengthTarget is greater than 1. Valid range is 0.0 <= CCALFStrengthTarget <= 1.0" );
  }
  if ( m_ccalf && ( m_chromaFormatIDC == CHROMA_400 ) ) {
    msg( VTM_WARNING, "****************************************************************************\n" );
    msg( VTM_WARNING, "** VTM_WARNING: --CCALF has been disabled because the chromaFormat is 400     **\n" );
    msg( VTM_WARNING, "****************************************************************************\n" );
    m_ccalf = false;
  }
  if ( m_JointCbCrMode && ( m_chromaFormatIDC == CHROMA_400 ) ) {
    msg( VTM_WARNING, "****************************************************************************\n" );
    msg( VTM_WARNING, "** VTM_WARNING: --JointCbCr has been disabled because the chromaFormat is 400 **\n" );
    msg( VTM_WARNING, "****************************************************************************\n" );
    m_JointCbCrMode = false;
  }
  if ( m_JointCbCrMode ) {
    xConfirmPara( m_cbCrQpOffset < -12, "Min. Joint Cb-Cr QP Offset is -12" );
    xConfirmPara( m_cbCrQpOffset > 12, "Max. Joint Cb-Cr QP Offset is  12" );
    xConfirmPara( m_cbCrQpOffsetDualTree < -12, "Min. Joint Cb-Cr QP Offset for dual tree is -12" );
    xConfirmPara( m_cbCrQpOffsetDualTree > 12, "Max. Joint Cb-Cr QP Offset for dual tree is  12" );
  }
  xConfirmPara( m_iQPAdaptationRange <= 0, "QP Adaptation Range must be more than 0" );
  if ( m_iDecodingRefreshType == 2 ) {
    xConfirmPara( m_iIntraPeriod > 0 && m_iIntraPeriod <= m_iGOPSize,
                  "Intra period must be larger than GOP size for periodic IDR pictures" );
  }
  xConfirmPara( m_uiMaxCUWidth > MAX_CU_SIZE, "MaxCUWith exceeds predefined MAX_CU_SIZE limit" );

  const int minCuSize = 1 << m_log2MinCuSize;
  xConfirmPara( m_uiMinQT[0] > 64, "Min Luma QT size in I slices should be smaller than or equal to 64" );
  xConfirmPara( m_uiMinQT[1] > 64, "Min Luma QT size in non-I slices should be smaller than or equal to 64" );
  xConfirmPara( m_uiMaxBT[2] > 64,
                "Maximum BT size for chroma block in I slice should be smaller than or equal to 64" );
  xConfirmPara( m_uiMaxTT[0] > 64, "Maximum TT size for luma block in I slice should be smaller than or equal to 64" );
  xConfirmPara( m_uiMaxTT[1] > 64,
                "Maximum TT size for luma block in non-I slice should be smaller than or equal to 64" );
  xConfirmPara( m_uiMaxTT[2] > 64,
                "Maximum TT size for chroma block in I slice should be smaller than or equal to 64" );
  xConfirmPara( m_uiMinQT[0] < minCuSize, "Min Luma QT size in I slices should be larger than or equal to minCuSize" );
  xConfirmPara( m_uiMinQT[1] < minCuSize,
                "Min Luma QT size in non-I slices should be larger than or equal to minCuSize" );
  xConfirmPara( ( m_sourceWidth % minCuSize ) || ( m_sourceHeight % minCuSize ),
                "Picture width or height is not a multiple of minCuSize" );
  const int minDiff = (int)floorLog2( m_uiMinQT[2] ) -
                      std::max( MIN_CU_LOG2, (int)m_log2MinCuSize -
                                                 (int)getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, m_chromaFormatIDC ) );
  xConfirmPara( minDiff < 0,
                "Min Chroma QT size in I slices is smaller than Min Luma CU size even considering color format" );
  xConfirmPara( ( m_uiMinQT[2] << (int)getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, m_chromaFormatIDC ) ) >
                    std::min( 64, (int)m_uiCTUSize ),
                "Min Chroma QT size in I slices should be smaller than or equal to CTB size or CB size after implicit "
                "split of CTB" );
  xConfirmPara( m_uiCTUSize < 32, "CTUSize must be greater than or equal to 32" );
  xConfirmPara( m_uiCTUSize > 128, "CTUSize must be less than or equal to 128" );
  xConfirmPara( m_uiCTUSize != 32 && m_uiCTUSize != 64 && m_uiCTUSize != 128,
                "CTUSize must be a power of 2 (32, 64, or 128)" );
  xConfirmPara( m_uiMaxCUWidth < 16, "Maximum partition width size should be larger than or equal to 16" );
  xConfirmPara( m_uiMaxCUHeight < 16, "Maximum partition height size should be larger than or equal to 16" );
  xConfirmPara( m_uiMaxBT[0] < m_uiMinQT[0],
                "Maximum BT size for luma block in I slice should be larger than minimum QT size" );
  xConfirmPara( m_uiMaxBT[0] > m_uiCTUSize,
                "Maximum BT size for luma block in I slice should be smaller than or equal to CTUSize" );
  xConfirmPara( m_uiMaxBT[1] < m_uiMinQT[1],
                "Maximum BT size for luma block in non I slice should be larger than minimum QT size" );
  xConfirmPara( m_uiMaxBT[1] > m_uiCTUSize,
                "Maximum BT size for luma block in non I slice should be smaller than or equal to CTUSize" );
  xConfirmPara( m_uiMaxBT[2] < ( m_uiMinQT[2] << (int)getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, m_chromaFormatIDC ) ),
                "Maximum BT size for chroma block in I slice should be larger than minimum QT size" );
  xConfirmPara( m_uiMaxBT[2] > m_uiCTUSize,
                "Maximum BT size for chroma block in I slice should be smaller than or equal to CTUSize" );
  xConfirmPara( m_uiMaxTT[0] < m_uiMinQT[0],
                "Maximum TT size for luma block in I slice should be larger than minimum QT size" );
  xConfirmPara( m_uiMaxTT[0] > m_uiCTUSize,
                "Maximum TT size for luma block in I slice should be smaller than or equal to CTUSize" );
  xConfirmPara( m_uiMaxTT[1] < m_uiMinQT[1],
                "Maximum TT size for luma block in non I slice should be larger than minimum QT size" );
  xConfirmPara( m_uiMaxTT[1] > m_uiCTUSize,
                "Maximum TT size for luma block in non I slice should be smaller than or equal to CTUSize" );
  xConfirmPara( m_uiMaxTT[2] < ( m_uiMinQT[2] << (int)getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, m_chromaFormatIDC ) ),
                "Maximum TT size for chroma block in I slice should be larger than minimum QT size" );
  xConfirmPara( m_uiMaxTT[2] > m_uiCTUSize,
                "Maximum TT size for chroma block in I slice should be smaller than or equal to CTUSize" );
  xConfirmPara( ( m_sourceWidth % ( std::max( 8u, m_log2MinCuSize ) ) ) != 0,
                "Resulting coded frame width must be a multiple of Max(8, the minimum CU size)" );
  xConfirmPara( ( m_sourceHeight % ( std::max( 8u, m_log2MinCuSize ) ) ) != 0,
                "Resulting coded frame height must be a multiple of Max(8, the minimum CU size)" );
  if ( m_uiMaxMTTHierarchyDepthI == 0 ) {
    xConfirmPara( m_uiMaxBT[0] != m_uiMinQT[0],
                  "MaxBTLumaISlice shall be equal to MinQTLumaISlice when MaxMTTHierarchyDepthISliceL is 0." );
    xConfirmPara( m_uiMaxTT[0] != m_uiMinQT[0],
                  "MaxTTLumaISlice shall be equal to MinQTLumaISlice when MaxMTTHierarchyDepthISliceL is 0." );
  }
  if ( m_uiMaxMTTHierarchyDepthIChroma == 0 ) {
    xConfirmPara(
        m_uiMaxBT[2] != ( m_uiMinQT[2] << (int)getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, m_chromaFormatIDC ) ),
        "MaxBTChromaISlice shall be equal to MinQTChromaISlice when MaxMTTHierarchyDepthISliceC is 0." );
    xConfirmPara(
        m_uiMaxTT[2] != ( m_uiMinQT[2] << (int)getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, m_chromaFormatIDC ) ),
        "MaxTTChromaISlice shall be equal to MinQTChromaISlice when MaxMTTHierarchyDepthISliceC is 0." );
  }
  if ( m_uiMaxMTTHierarchyDepth == 0 ) {
    xConfirmPara( m_uiMaxBT[1] != m_uiMinQT[1],
                  "MaxBTNonISlice shall be equal to MinQTNonISlice when MaxMTTHierarchyDepth is 0." );
    xConfirmPara( m_uiMaxTT[1] != m_uiMinQT[1],
                  "MaxTTNonISlice shall be equal to MinQTNonISlice when MaxMTTHierarchyDepth is 0." );
  }
  xConfirmPara( m_log2MaxTbSize > 6, "Log2MaxTbSize must be 6 or smaller." );
  xConfirmPara( m_log2MaxTbSize < 5, "Log2MaxTbSize must be 5 or greater." );
  xConfirmPara( m_maxNumMergeCand < 1, "MaxNumMergeCand must be 1 or greater." );
  xConfirmPara( m_maxNumMergeCand > MRG_MAX_NUM_CANDS, "MaxNumMergeCand must be no more than MRG_MAX_NUM_CANDS." );
  xConfirmPara( m_maxNumGeoCand > GEO_MAX_NUM_UNI_CANDS, "MaxNumGeoCand must be no more than GEO_MAX_NUM_UNI_CANDS." );
  xConfirmPara( m_maxNumGeoCand > m_maxNumMergeCand, "MaxNumGeoCand must be no more than MaxNumMergeCand." );
  xConfirmPara( 0 < m_maxNumGeoCand && m_maxNumGeoCand < 2,
                "MaxNumGeoCand must be no less than 2 unless MaxNumGeoCand is 0." );
  xConfirmPara( m_maxNumIBCMergeCand < 1, "MaxNumIBCMergeCand must be 1 or greater." );
  xConfirmPara( m_maxNumIBCMergeCand > IBC_MRG_MAX_NUM_CANDS,
                "MaxNumIBCMergeCand must be no more than IBC_MRG_MAX_NUM_CANDS." );
  xConfirmPara( m_maxNumAffineMergeCand < ( m_sbTmvpEnableFlag ? 1 : 0 ),
                "MaxNumAffineMergeCand must be greater than 0 when SbTMVP is enabled" );
  xConfirmPara( m_maxNumAffineMergeCand > AFFINE_MRG_MAX_NUM_CANDS,
                "MaxNumAffineMergeCand must be no more than AFFINE_MRG_MAX_NUM_CANDS." );
  if ( m_Affine == 0 ) {
    m_maxNumAffineMergeCand = m_sbTmvpEnableFlag ? 1 : 0;
    if ( m_PROF ) msg( VTM_WARNING, "PROF is forcefully disabled when Affine is off \n" );
    m_PROF = false;
  }

  xConfirmPara( m_MTS < 0 || m_MTS > 3, "MTS must be greater than 0 smaller than 4" );
  xConfirmPara( m_MTSIntraMaxCand < 0 || m_MTSIntraMaxCand > 5,
                "m_MTSIntraMaxCand must be greater than 0 and smaller than 6" );
  xConfirmPara( m_MTSInterMaxCand < 0 || m_MTSInterMaxCand > 5,
                "m_MTSInterMaxCand must be greater than 0 and smaller than 6" );
  xConfirmPara( m_MTS != 0 && m_MTSImplicit != 0, "Both explicit and implicit MTS cannot be enabled at the same time" );

  if ( m_useBDPCM ) { xConfirmPara( !m_useTransformSkip, "BDPCM cannot be used when transform skip is disabled." ); }

  if ( !m_alf ) { xConfirmPara( m_ccalf, "CCALF cannot be enabled when ALF is disabled" ); }

  xConfirmPara( m_sourceWidth % SPS::getWinUnitX( m_chromaFormatIDC ) != 0,
                "Picture width must be an integer multiple of the specified chroma subsampling" );
  xConfirmPara( m_sourceHeight % SPS::getWinUnitY( m_chromaFormatIDC ) != 0,
                "Picture height must be an integer multiple of the specified chroma subsampling" );

  xConfirmPara( m_sourcePadding[0] % SPS::getWinUnitX( m_chromaFormatIDC ) != 0,
                "Horizontal padding must be an integer multiple of the specified chroma subsampling" );
  xConfirmPara( m_sourcePadding[1] % SPS::getWinUnitY( m_chromaFormatIDC ) != 0,
                "Vertical padding must be an integer multiple of the specified chroma subsampling" );

  xConfirmPara( m_confWinLeft % SPS::getWinUnitX( m_chromaFormatIDC ) != 0,
                "Left conformance window offset must be an integer multiple of the specified chroma subsampling" );
  xConfirmPara( m_confWinRight % SPS::getWinUnitX( m_chromaFormatIDC ) != 0,
                "Right conformance window offset must be an integer multiple of the specified chroma subsampling" );
  xConfirmPara( m_confWinTop % SPS::getWinUnitY( m_chromaFormatIDC ) != 0,
                "Top conformance window offset must be an integer multiple of the specified chroma subsampling" );
  xConfirmPara( m_confWinBottom % SPS::getWinUnitY( m_chromaFormatIDC ) != 0,
                "Bottom conformance window offset must be an integer multiple of the specified chroma subsampling" );

  // max CU width and height should be power of 2
  uint32_t ui = m_uiMaxCUWidth;
  while ( ui ) {
    ui >>= 1;
    if ( ( ui & 1 ) == 1 ) { xConfirmPara( ui != 1, "Width should be 2^n" ); }
  }
  ui = m_uiMaxCUHeight;
  while ( ui ) {
    ui >>= 1;
    if ( ( ui & 1 ) == 1 ) { xConfirmPara( ui != 1, "Height should be 2^n" ); }
  }

  /* if this is an intra-only sequence, ie IntraPeriod=1, don't verify the GOP structure
   * This permits the ability to omit a GOP structure specification */
  if ( m_iIntraPeriod == 1 && m_GOPList[0].m_POC == -1 ) {
    m_GOPList[0]                    = GOPEntry();
    m_GOPList[0].m_QPFactor         = 1;
    m_GOPList[0].m_betaOffsetDiv2   = 0;
    m_GOPList[0].m_tcOffsetDiv2     = 0;
    m_GOPList[0].m_CbBetaOffsetDiv2 = 0;
    m_GOPList[0].m_CbTcOffsetDiv2   = 0;
    m_GOPList[0].m_CrBetaOffsetDiv2 = 0;
    m_GOPList[0].m_CrTcOffsetDiv2   = 0;
    m_GOPList[0].m_POC              = 1;
    m_RPLList0[0]                   = RPLEntry();
    m_RPLList1[0]                   = RPLEntry();
    m_RPLList0[0].m_POC = m_RPLList1[0].m_POC = 1;
    m_RPLList0[0].m_numRefPicsActive          = 4;
    m_GOPList[0].m_numRefPicsActive0          = 4;
  } else {
    xConfirmPara( m_intraOnlyConstraintFlag, "IntraOnlyConstraintFlag cannot be 1 for inter sequences" );
  }

  int  multipleFactor                = m_compositeRefEnabled ? 2 : 1;
  bool verifiedGOP                   = false;
  bool errorGOP                      = false;
  int  checkGOP                      = 1;
  int  numRefs                       = m_isField ? 2 : 1;
  int  refList[MAX_NUM_REF_PICS + 1] = {0};
  if ( m_isField ) { refList[1] = 1; }
  bool isOK[MAX_GOP];
  for ( int i = 0; i < MAX_GOP; i++ ) { isOK[i] = false; }
  int numOK = 0;
  xConfirmPara( m_iIntraPeriod >= 0 && ( m_iIntraPeriod % m_iGOPSize != 0 ),
                "Intra period must be a multiple of GOPSize, or -1" );

  for ( int i = 0; i < m_iGOPSize; i++ ) {
    if ( m_GOPList[i].m_POC == m_iGOPSize * multipleFactor ) {
      xConfirmPara( m_GOPList[i].m_temporalId != 0, "The last frame in each GOP must have temporal ID = 0 " );
    }
  }

  if ( ( m_iIntraPeriod != 1 ) && !m_deblockingFilterOffsetInPPS && ( !m_deblockingFilterDisable ) ) {
    for ( int i = 0; i < m_iGOPSize; i++ ) {
      xConfirmPara( ( m_GOPList[i].m_betaOffsetDiv2 + m_deblockingFilterBetaOffsetDiv2 ) < -12 ||
                        ( m_GOPList[i].m_betaOffsetDiv2 + m_deblockingFilterBetaOffsetDiv2 ) > 12,
                    "Loop Filter Beta Offset div. 2 for one of the GOP entries exceeds supported range (-12 to 12)" );
      xConfirmPara( ( m_GOPList[i].m_tcOffsetDiv2 + m_deblockingFilterTcOffsetDiv2 ) < -12 ||
                        ( m_GOPList[i].m_tcOffsetDiv2 + m_deblockingFilterTcOffsetDiv2 ) > 12,
                    "Loop Filter Tc Offset div. 2 for one of the GOP entries exceeds supported range (-12 to 12)" );
      xConfirmPara( ( m_GOPList[i].m_CbBetaOffsetDiv2 + m_deblockingFilterCbBetaOffsetDiv2 ) < -12 ||
                        ( m_GOPList[i].m_CbBetaOffsetDiv2 + m_deblockingFilterCbBetaOffsetDiv2 ) > 12,
                    "Loop Filter Beta Offset div. 2 for one of the GOP entries exceeds supported range (-12 to 12)" );
      xConfirmPara( ( m_GOPList[i].m_CbTcOffsetDiv2 + m_deblockingFilterCbTcOffsetDiv2 ) < -12 ||
                        ( m_GOPList[i].m_CbTcOffsetDiv2 + m_deblockingFilterCbTcOffsetDiv2 ) > 12,
                    "Loop Filter Tc Offset div. 2 for one of the GOP entries exceeds supported range (-12 to 12)" );
      xConfirmPara( ( m_GOPList[i].m_CrBetaOffsetDiv2 + m_deblockingFilterCrBetaOffsetDiv2 ) < -12 ||
                        ( m_GOPList[i].m_CrBetaOffsetDiv2 + m_deblockingFilterCrBetaOffsetDiv2 ) > 12,
                    "Loop Filter Beta Offset div. 2 for one of the GOP entries exceeds supported range (-12 to 12)" );
      xConfirmPara( ( m_GOPList[i].m_CrTcOffsetDiv2 + m_deblockingFilterCrTcOffsetDiv2 ) < -12 ||
                        ( m_GOPList[i].m_CrTcOffsetDiv2 + m_deblockingFilterCrTcOffsetDiv2 ) > 12,
                    "Loop Filter Tc Offset div. 2 for one of the GOP entries exceeds supported range (-12 to 12)" );
    }
  }

#if W0038_CQP_ADJ
  for ( int i = 0; i < m_iGOPSize; i++ ) {
    xConfirmPara( abs( m_GOPList[i].m_CbQPoffset ) > 12,
                  "Cb QP Offset for one of the GOP entries exceeds supported range (-12 to 12)" );
    xConfirmPara( abs( m_GOPList[i].m_CbQPoffset + m_cbQpOffset ) > 12,
                  "Cb QP Offset for one of the GOP entries, when combined with the PPS Cb offset, exceeds supported "
                  "range (-12 to 12)" );
    xConfirmPara( abs( m_GOPList[i].m_CrQPoffset ) > 12,
                  "Cr QP Offset for one of the GOP entries exceeds supported range (-12 to 12)" );
    xConfirmPara( abs( m_GOPList[i].m_CrQPoffset + m_crQpOffset ) > 12,
                  "Cr QP Offset for one of the GOP entries, when combined with the PPS Cr offset, exceeds supported "
                  "range (-12 to 12)" );
  }
  xConfirmPara( abs( m_sliceChromaQpOffsetIntraOrPeriodic[0] ) > 12,
                "Intra/periodic Cb QP Offset exceeds supported range (-12 to 12)" );
  xConfirmPara(
      abs( m_sliceChromaQpOffsetIntraOrPeriodic[0] + m_cbQpOffset ) > 12,
      "Intra/periodic Cb QP Offset, when combined with the PPS Cb offset, exceeds supported range (-12 to 12)" );
  xConfirmPara( abs( m_sliceChromaQpOffsetIntraOrPeriodic[1] ) > 12,
                "Intra/periodic Cr QP Offset exceeds supported range (-12 to 12)" );
  xConfirmPara(
      abs( m_sliceChromaQpOffsetIntraOrPeriodic[1] + m_crQpOffset ) > 12,
      "Intra/periodic Cr QP Offset, when combined with the PPS Cr offset, exceeds supported range (-12 to 12)" );
#endif

  xConfirmPara( m_maxSublayers < 1 || m_maxSublayers > 7, "MaxSublayers must be in range [1..7]" );

  xConfirmPara( m_fastLocalDualTreeMode < 0 || m_fastLocalDualTreeMode > 2,
                "FastLocalDualTreeMode must be in range [0..2]" );

  int extraRPLs = 0;
  // start looping through frames in coding order until we can verify that the GOP structure is correct.
  while ( !verifiedGOP && !errorGOP ) {
    int curGOP = ( checkGOP - 1 ) % m_iGOPSize;
    int curPOC = ( ( checkGOP - 1 ) / m_iGOPSize ) * m_iGOPSize * multipleFactor + m_RPLList0[curGOP].m_POC;
    if ( m_RPLList0[curGOP].m_POC < 0 || m_RPLList1[curGOP].m_POC < 0 ) {
      msg( VTM_WARNING, "\nError: found fewer Reference Picture Sets than GOPSize\n" );
      errorGOP = true;
    } else {
      // check that all reference pictures are available, or have a POC < 0 meaning they might be available in the next
      // GOP.
      bool beforeI = false;
      for ( int i = 0; i < m_RPLList0[curGOP].m_numRefPics; i++ ) {
        int absPOC = curPOC - m_RPLList0[curGOP].m_deltaRefPics[i];
        if ( absPOC < 0 ) {
          beforeI = true;
        } else {
          bool found = false;
          for ( int j = 0; j < numRefs; j++ ) {
            if ( refList[j] == absPOC ) {
              found = true;
              for ( int k = 0; k < m_iGOPSize; k++ ) {
                if ( absPOC % ( m_iGOPSize * multipleFactor ) ==
                     m_RPLList0[k].m_POC % ( m_iGOPSize * multipleFactor ) ) {
                  if ( m_RPLList0[k].m_temporalId == m_RPLList0[curGOP].m_temporalId ) {
                    m_RPLList0[k].m_refPic = true;
                  }
                }
              }
            }
          }
          if ( !found ) {
            msg( VTM_WARNING, "\nError: ref pic %d is not available for GOP frame %d\n",
                 m_RPLList0[curGOP].m_deltaRefPics[i], curGOP + 1 );
            errorGOP = true;
          }
        }
      }
      if ( !beforeI && !errorGOP ) {
        // all ref frames were present
        if ( !isOK[curGOP] ) {
          numOK++;
          isOK[curGOP] = true;
          if ( numOK == m_iGOPSize ) { verifiedGOP = true; }
        }
      } else {
        // create a new RPLEntry for this frame containing all the reference pictures that were available (POC > 0)
        m_RPLList0[m_iGOPSize + extraRPLs] = m_RPLList0[curGOP];
        m_RPLList1[m_iGOPSize + extraRPLs] = m_RPLList1[curGOP];
        int newRefs0                       = 0;
        for ( int i = 0; i < m_RPLList0[curGOP].m_numRefPics; i++ ) {
          int absPOC = curPOC - m_RPLList0[curGOP].m_deltaRefPics[i];
          if ( absPOC >= 0 ) {
            m_RPLList0[m_iGOPSize + extraRPLs].m_deltaRefPics[newRefs0] = m_RPLList0[curGOP].m_deltaRefPics[i];
            newRefs0++;
          }
        }
        int numPrefRefs0 = m_RPLList0[curGOP].m_numRefPicsActive;

        int newRefs1 = 0;
        for ( int i = 0; i < m_RPLList1[curGOP].m_numRefPics; i++ ) {
          int absPOC = curPOC - m_RPLList1[curGOP].m_deltaRefPics[i];
          if ( absPOC >= 0 ) {
            m_RPLList1[m_iGOPSize + extraRPLs].m_deltaRefPics[newRefs1] = m_RPLList1[curGOP].m_deltaRefPics[i];
            newRefs1++;
          }
        }
        int numPrefRefs1 = m_RPLList1[curGOP].m_numRefPicsActive;

        for ( int offset = -1; offset > -checkGOP; offset-- ) {
          // step backwards in coding order and include any extra available pictures we might find useful to replace the
          // ones with POC < 0.
          int offGOP = ( checkGOP - 1 + offset ) % m_iGOPSize;
          int offPOC =
              ( ( checkGOP - 1 + offset ) / m_iGOPSize ) * ( m_iGOPSize * multipleFactor ) + m_RPLList0[offGOP].m_POC;
          if ( offPOC >= 0 && m_RPLList0[offGOP].m_temporalId <= m_RPLList0[curGOP].m_temporalId ) {
            bool newRef = false;
            for ( int i = 0; i < ( newRefs0 + newRefs1 ); i++ ) {
              if ( refList[i] == offPOC ) { newRef = true; }
            }
            for ( int i = 0; i < newRefs0; i++ ) {
              if ( m_RPLList0[m_iGOPSize + extraRPLs].m_deltaRefPics[i] == curPOC - offPOC ) { newRef = false; }
            }
            if ( newRef ) {
              int insertPoint = newRefs0;
              // this picture can be added, find appropriate place in list and insert it.
              if ( m_RPLList0[offGOP].m_temporalId == m_RPLList0[curGOP].m_temporalId ) {
                m_RPLList0[offGOP].m_refPic = true;
              }
              for ( int j = 0; j < newRefs0; j++ ) {
                if ( m_RPLList0[m_iGOPSize + extraRPLs].m_deltaRefPics[j] > curPOC - offPOC && curPOC - offPOC > 0 ) {
                  insertPoint = j;
                  break;
                }
              }
              int prev = curPOC - offPOC;
              for ( int j = insertPoint; j < newRefs0 + 1; j++ ) {
                int newPrev = m_RPLList0[m_iGOPSize + extraRPLs].m_deltaRefPics[j];
                m_RPLList0[m_iGOPSize + extraRPLs].m_deltaRefPics[j] = prev;
                prev                                                 = newPrev;
              }
              newRefs0++;
            }
          }
          if ( newRefs0 >= numPrefRefs0 ) { break; }
        }

        for ( int offset = -1; offset > -checkGOP; offset-- ) {
          // step backwards in coding order and include any extra available pictures we might find useful to replace the
          // ones with POC < 0.
          int offGOP = ( checkGOP - 1 + offset ) % m_iGOPSize;
          int offPOC =
              ( ( checkGOP - 1 + offset ) / m_iGOPSize ) * ( m_iGOPSize * multipleFactor ) + m_RPLList1[offGOP].m_POC;
          if ( offPOC >= 0 && m_RPLList1[offGOP].m_temporalId <= m_RPLList1[curGOP].m_temporalId ) {
            bool newRef = false;
            for ( int i = 0; i < ( newRefs0 + newRefs1 ); i++ ) {
              if ( refList[i] == offPOC ) { newRef = true; }
            }
            for ( int i = 0; i < newRefs1; i++ ) {
              if ( m_RPLList1[m_iGOPSize + extraRPLs].m_deltaRefPics[i] == curPOC - offPOC ) { newRef = false; }
            }
            if ( newRef ) {
              int insertPoint = newRefs1;
              // this picture can be added, find appropriate place in list and insert it.
              if ( m_RPLList1[offGOP].m_temporalId == m_RPLList1[curGOP].m_temporalId ) {
                m_RPLList1[offGOP].m_refPic = true;
              }
              for ( int j = 0; j < newRefs1; j++ ) {
                if ( m_RPLList1[m_iGOPSize + extraRPLs].m_deltaRefPics[j] > curPOC - offPOC && curPOC - offPOC > 0 ) {
                  insertPoint = j;
                  break;
                }
              }
              int prev = curPOC - offPOC;
              for ( int j = insertPoint; j < newRefs1 + 1; j++ ) {
                int newPrev = m_RPLList1[m_iGOPSize + extraRPLs].m_deltaRefPics[j];
                m_RPLList1[m_iGOPSize + extraRPLs].m_deltaRefPics[j] = prev;
                prev                                                 = newPrev;
              }
              newRefs1++;
            }
          }
          if ( newRefs1 >= numPrefRefs1 ) { break; }
        }

        m_RPLList0[m_iGOPSize + extraRPLs].m_numRefPics       = newRefs0;
        m_RPLList0[m_iGOPSize + extraRPLs].m_numRefPicsActive = min(
            m_RPLList0[m_iGOPSize + extraRPLs].m_numRefPics, m_RPLList0[m_iGOPSize + extraRPLs].m_numRefPicsActive );
        m_RPLList1[m_iGOPSize + extraRPLs].m_numRefPics       = newRefs1;
        m_RPLList1[m_iGOPSize + extraRPLs].m_numRefPicsActive = min(
            m_RPLList1[m_iGOPSize + extraRPLs].m_numRefPics, m_RPLList1[m_iGOPSize + extraRPLs].m_numRefPicsActive );
        curGOP = m_iGOPSize + extraRPLs;
        extraRPLs++;
      }
      numRefs = 0;
      for ( int i = 0; i < m_RPLList0[curGOP].m_numRefPics; i++ ) {
        int absPOC = curPOC - m_RPLList0[curGOP].m_deltaRefPics[i];
        if ( absPOC >= 0 ) {
          refList[numRefs] = absPOC;
          numRefs++;
        }
      }
      for ( int i = 0; i < m_RPLList1[curGOP].m_numRefPics; i++ ) {
        int absPOC = curPOC - m_RPLList1[curGOP].m_deltaRefPics[i];
        if ( absPOC >= 0 ) {
          bool alreadyExist = false;
          for ( int j = 0; !alreadyExist && j < numRefs; j++ ) {
            if ( refList[j] == absPOC ) { alreadyExist = true; }
          }
          if ( !alreadyExist ) {
            refList[numRefs] = absPOC;
            numRefs++;
          }
        }
      }
      refList[numRefs] = curPOC;
      numRefs++;
    }
    checkGOP++;
  }
  xConfirmPara( errorGOP, "Invalid GOP structure given" );

  m_maxTempLayer = 1;

  for ( int i = 0; i < m_iGOPSize; i++ ) {
    if ( m_GOPList[i].m_temporalId >= m_maxTempLayer ) { m_maxTempLayer = m_GOPList[i].m_temporalId + 1; }
    xConfirmPara( m_GOPList[i].m_sliceType != 'B' && m_GOPList[i].m_sliceType != 'P' && m_GOPList[i].m_sliceType != 'I',
                  "Slice type must be equal to B or P or I" );
  }
  for ( int i = 0; i < MAX_TLAYER; i++ ) {
    m_maxNumReorderPics[i]  = 0;
    m_maxDecPicBuffering[i] = 1;
  }
  for ( int i = 0; i < m_iGOPSize; i++ ) {
    int numRefPic = m_RPLList0[i].m_numRefPics;
    for ( int tmp = 0; tmp < m_RPLList1[i].m_numRefPics; tmp++ ) {
      bool notSame = true;
      for ( int jj = 0; notSame && jj < m_RPLList0[i].m_numRefPics; jj++ ) {
        if ( m_RPLList1[i].m_deltaRefPics[tmp] == m_RPLList0[i].m_deltaRefPics[jj] ) notSame = false;
      }
      if ( notSame ) numRefPic++;
    }
    if ( numRefPic + 1 > m_maxDecPicBuffering[m_GOPList[i].m_temporalId] ) {
      m_maxDecPicBuffering[m_GOPList[i].m_temporalId] = numRefPic + 1;
    }
    int highestDecodingNumberWithLowerPOC = 0;
    for ( int j = 0; j < m_iGOPSize; j++ ) {
      if ( m_GOPList[j].m_POC <= m_GOPList[i].m_POC ) { highestDecodingNumberWithLowerPOC = j; }
    }
    int numReorder = 0;
    for ( int j = 0; j < highestDecodingNumberWithLowerPOC; j++ ) {
      if ( m_GOPList[j].m_temporalId <= m_GOPList[i].m_temporalId && m_GOPList[j].m_POC > m_GOPList[i].m_POC ) {
        numReorder++;
      }
    }
    if ( numReorder > m_maxNumReorderPics[m_GOPList[i].m_temporalId] ) {
      m_maxNumReorderPics[m_GOPList[i].m_temporalId] = numReorder;
    }
  }

  for ( int i = 0; i < MAX_TLAYER - 1; i++ ) {
    // a lower layer can not have higher value of m_maxNumReorderPics than a higher layer
    if ( m_maxNumReorderPics[i + 1] < m_maxNumReorderPics[i] ) { m_maxNumReorderPics[i + 1] = m_maxNumReorderPics[i]; }
    // the value of dpb_max_num_reorder_pics[ i ] shall be in the range of 0 to max_dec_pic_buffering[ i ] - 1,
    // inclusive
    if ( m_maxNumReorderPics[i] > m_maxDecPicBuffering[i] - 1 ) {
      m_maxDecPicBuffering[i] = m_maxNumReorderPics[i] + 1;
    }
    // a lower layer can not have higher value of m_uiMaxDecPicBuffering than a higher layer
    if ( m_maxDecPicBuffering[i + 1] < m_maxDecPicBuffering[i] ) {
      m_maxDecPicBuffering[i + 1] = m_maxDecPicBuffering[i];
    }
  }

  // the value of dpb_max_num_reorder_pics[ i ] shall be in the range of 0 to max_dec_pic_buffering[ i ] -  1, inclusive
  if ( m_maxNumReorderPics[MAX_TLAYER - 1] > m_maxDecPicBuffering[MAX_TLAYER - 1] - 1 ) {
    m_maxDecPicBuffering[MAX_TLAYER - 1] = m_maxNumReorderPics[MAX_TLAYER - 1] + 1;
  }

  if ( m_picPartitionFlag ) {
    PPS      pps;
    uint32_t colIdx, rowIdx;
    uint32_t remSize;

    pps.setPicWidthInLumaSamples( m_sourceWidth );
    pps.setPicHeightInLumaSamples( m_sourceHeight );
    pps.setLog2CtuSize( floorLog2( m_uiCTUSize ) );

    // set default tile column if not provided
    if ( m_tileColumnWidth.size() == 0 ) { m_tileColumnWidth.push_back( pps.getPicWidthInCtu() ); }
    // set default tile row if not provided
    if ( m_tileRowHeight.size() == 0 ) { m_tileRowHeight.push_back( pps.getPicHeightInCtu() ); }

    // remove any tile columns that can be specified implicitly
    while ( m_tileColumnWidth.size() > 1 && m_tileColumnWidth.end()[-1] == m_tileColumnWidth.end()[-2] ) {
      m_tileColumnWidth.pop_back();
    }

    // remove any tile rows that can be specified implicitly
    while ( m_tileRowHeight.size() > 1 && m_tileRowHeight.end()[-1] == m_tileRowHeight.end()[-2] ) {
      m_tileRowHeight.pop_back();
    }

    // setup tiles in temporary PPS structure
    remSize = pps.getPicWidthInCtu();
    for ( colIdx = 0; remSize > 0 && colIdx < m_tileColumnWidth.size(); colIdx++ ) {
      xConfirmPara( m_tileColumnWidth[colIdx] == 0, "Tile column widths cannot be equal to 0" );
      m_tileColumnWidth[colIdx] = std::min( remSize, m_tileColumnWidth[colIdx] );
      pps.addTileColumnWidth( m_tileColumnWidth[colIdx] );
      remSize -= m_tileColumnWidth[colIdx];
    }
    m_tileColumnWidth.resize( colIdx );
    pps.setNumExpTileColumns( (uint32_t)m_tileColumnWidth.size() );
    remSize = pps.getPicHeightInCtu();
    for ( rowIdx = 0; remSize > 0 && rowIdx < m_tileRowHeight.size(); rowIdx++ ) {
      xConfirmPara( m_tileRowHeight[rowIdx] == 0, "Tile row heights cannot be equal to 0" );
      m_tileRowHeight[rowIdx] = std::min( remSize, m_tileRowHeight[rowIdx] );
      pps.addTileRowHeight( m_tileRowHeight[rowIdx] );
      remSize -= m_tileRowHeight[rowIdx];
    }
    m_tileRowHeight.resize( rowIdx );
    pps.setNumExpTileRows( (uint32_t)m_tileRowHeight.size() );
    pps.initTiles();
    xConfirmPara( pps.getNumTileColumns() > getMaxTileColsByLevel( m_level ),
                  "Number of tile columns exceeds maximum number allowed according to specified level" );
    xConfirmPara( pps.getNumTileRows() > getMaxTileRowsByLevel( m_level ),
                  "Number of tile rows exceeds maximum number allowed according to specified level" );
    m_numTileCols = pps.getNumTileColumns();
    m_numTileRows = pps.getNumTileRows();

    // rectangular slices
    if ( !m_rasterSliceFlag ) {
      if ( !m_singleSlicePerSubPicFlag ) {
        uint32_t sliceIdx;
        bool     needTileIdxDelta = false;

        // generate slice list for the simplified fixed-rectangular-slice-size config option
        if ( m_rectSliceFixedWidth > 0 && m_rectSliceFixedHeight > 0 ) {
          int tileIdx = 0;
          m_rectSlicePos.clear();
          while ( tileIdx < pps.getNumTiles() ) {
            uint32_t startTileX = tileIdx % pps.getNumTileColumns();
            uint32_t startTileY = tileIdx / pps.getNumTileColumns();
            uint32_t startCtuX  = pps.getTileColumnBd( startTileX );
            uint32_t startCtuY  = pps.getTileRowBd( startTileY );
            uint32_t stopCtuX   = ( startTileX + m_rectSliceFixedWidth ) >= pps.getNumTileColumns()
                                    ? pps.getPicWidthInCtu() - 1
                                    : pps.getTileColumnBd( startTileX + m_rectSliceFixedWidth ) - 1;
            uint32_t stopCtuY = ( startTileY + m_rectSliceFixedHeight ) >= pps.getNumTileRows()
                                    ? pps.getPicHeightInCtu() - 1
                                    : pps.getTileRowBd( startTileY + m_rectSliceFixedHeight ) - 1;
            uint32_t stopTileX = pps.ctuToTileCol( stopCtuX );
            uint32_t stopTileY = pps.ctuToTileRow( stopCtuY );

            // add rectangular slice to list
            m_rectSlicePos.push_back( startCtuY * pps.getPicWidthInCtu() + startCtuX );
            m_rectSlicePos.push_back( stopCtuY * pps.getPicWidthInCtu() + stopCtuX );

            // get slice size in tiles
            uint32_t sliceWidth  = stopTileX - startTileX + 1;
            uint32_t sliceHeight = stopTileY - startTileY + 1;

            // move to next tile in raster scan order
            tileIdx += sliceWidth;
            if ( tileIdx % pps.getNumTileColumns() == 0 ) { tileIdx += ( sliceHeight - 1 ) * pps.getNumTileColumns(); }
          }
        }

        xConfirmPara( m_rectSlicePos.size() & 1,
                      "Odd number of rectangular slice positions provided. Rectangular slice positions must be "
                      "specified in pairs of (top-left / bottom-right) raster-scan CTU addresses." );

        // set default slice size if not provided
        if ( m_rectSlicePos.size() == 0 ) {
          m_rectSlicePos.push_back( 0 );
          m_rectSlicePos.push_back( pps.getPicWidthInCtu() * pps.getPicHeightInCtu() - 1 );
        }
        pps.setNumSlicesInPic( ( uint32_t )( m_rectSlicePos.size() >> 1 ) );
        xConfirmPara( pps.getNumSlicesInPic() > getMaxSlicesByLevel( m_level ),
                      "Number of rectangular slices exceeds maximum number allowed according to specified level" );
        pps.initRectSlices();

        // set slice parameters from CTU addresses
        for ( sliceIdx = 0; sliceIdx < pps.getNumSlicesInPic(); sliceIdx++ ) {
          xConfirmPara( m_rectSlicePos[2 * sliceIdx] >= pps.getPicWidthInCtu() * pps.getPicHeightInCtu(),
                        "Rectangular slice position exceeds total number of CTU in picture." );
          xConfirmPara( m_rectSlicePos[2 * sliceIdx + 1] >= pps.getPicWidthInCtu() * pps.getPicHeightInCtu(),
                        "Rectangular slice position exceeds total number of CTU in picture." );

          // map raster scan CTU address to X/Y position
          uint32_t startCtuX = m_rectSlicePos[2 * sliceIdx] % pps.getPicWidthInCtu();
          uint32_t startCtuY = m_rectSlicePos[2 * sliceIdx] / pps.getPicWidthInCtu();
          uint32_t stopCtuX  = m_rectSlicePos[2 * sliceIdx + 1] % pps.getPicWidthInCtu();
          uint32_t stopCtuY  = m_rectSlicePos[2 * sliceIdx + 1] / pps.getPicWidthInCtu();

          // get corresponding tile index
          uint32_t startTileX = pps.ctuToTileCol( startCtuX );
          uint32_t startTileY = pps.ctuToTileRow( startCtuY );
          uint32_t stopTileX  = pps.ctuToTileCol( stopCtuX );
          uint32_t stopTileY  = pps.ctuToTileRow( stopCtuY );
          uint32_t tileIdx    = startTileY * pps.getNumTileColumns() + startTileX;

          // get slice size in tiles
          uint32_t sliceWidth  = stopTileX - startTileX + 1;
          uint32_t sliceHeight = stopTileY - startTileY + 1;

          // check for slice / tile alignment
          xConfirmPara( startCtuX != pps.getTileColumnBd( startTileX ),
                        "Rectangular slice position does not align with a left tile edge." );
          xConfirmPara( stopCtuX != ( pps.getTileColumnBd( stopTileX + 1 ) - 1 ),
                        "Rectangular slice position does not align with a right tile edge." );
          if ( sliceWidth > 1 || sliceHeight > 1 ) {
            xConfirmPara( startCtuY != pps.getTileRowBd( startTileY ),
                          "Rectangular slice position does not align with a top tile edge." );
            xConfirmPara( stopCtuY != ( pps.getTileRowBd( stopTileY + 1 ) - 1 ),
                          "Rectangular slice position does not align with a bottom tile edge." );
          }

          // set slice size and tile index
          pps.setSliceWidthInTiles( sliceIdx, sliceWidth );
          pps.setSliceHeightInTiles( sliceIdx, sliceHeight );
          pps.setSliceTileIdx( sliceIdx, tileIdx );
          if ( sliceIdx > 0 && !needTileIdxDelta ) {
            uint32_t lastTileIdx = pps.getSliceTileIdx( sliceIdx - 1 );
            lastTileIdx += pps.getSliceWidthInTiles( sliceIdx - 1 );
            if ( lastTileIdx % pps.getNumTileColumns() == 0 ) {
              lastTileIdx += ( pps.getSliceHeightInTiles( sliceIdx - 1 ) - 1 ) * pps.getNumTileColumns();
            }
            if ( lastTileIdx != tileIdx ) { needTileIdxDelta = true; }
          }

          // special case for multiple slices within a single tile
          if ( sliceWidth == 1 && sliceHeight == 1 ) {
            uint32_t firstSliceIdx   = sliceIdx;
            uint32_t numSlicesInTile = 1;
            pps.setSliceHeightInCtu( sliceIdx, stopCtuY - startCtuY + 1 );

            while ( sliceIdx < pps.getNumSlicesInPic() - 1 ) {
              uint32_t nextTileIdx;
              startCtuX   = m_rectSlicePos[2 * ( sliceIdx + 1 )] % pps.getPicWidthInCtu();
              startCtuY   = m_rectSlicePos[2 * ( sliceIdx + 1 )] / pps.getPicWidthInCtu();
              stopCtuX    = m_rectSlicePos[2 * ( sliceIdx + 1 ) + 1] % pps.getPicWidthInCtu();
              stopCtuY    = m_rectSlicePos[2 * ( sliceIdx + 1 ) + 1] / pps.getPicWidthInCtu();
              startTileX  = pps.ctuToTileCol( startCtuX );
              startTileY  = pps.ctuToTileRow( startCtuY );
              stopTileX   = pps.ctuToTileCol( stopCtuX );
              stopTileY   = pps.ctuToTileRow( stopCtuY );
              nextTileIdx = startTileY * pps.getNumTileColumns() + startTileX;
              sliceWidth  = stopTileX - startTileX + 1;
              sliceHeight = stopTileY - startTileY + 1;
              if ( nextTileIdx != tileIdx || sliceWidth != 1 || sliceHeight != 1 ) { break; }
              numSlicesInTile++;
              sliceIdx++;
              pps.setSliceWidthInTiles( sliceIdx, 1 );
              pps.setSliceHeightInTiles( sliceIdx, 1 );
              pps.setSliceTileIdx( sliceIdx, tileIdx );
              pps.setSliceHeightInCtu( sliceIdx, stopCtuY - startCtuY + 1 );
            }
            pps.setNumSlicesInTile( firstSliceIdx, numSlicesInTile );
          }
        }
        pps.setTileIdxDeltaPresentFlag( needTileIdxDelta );
        m_tileIdxDeltaPresentFlag = needTileIdxDelta;

        // check rectangular slice mapping and full picture CTU coverage
        pps.initRectSliceMap( nullptr );

        // store rectangular slice parameters from temporary PPS structure
        m_numSlicesInPic = pps.getNumSlicesInPic();
        m_rectSlices.resize( pps.getNumSlicesInPic() );
        for ( sliceIdx = 0; sliceIdx < pps.getNumSlicesInPic(); sliceIdx++ ) {
          m_rectSlices[sliceIdx].setSliceWidthInTiles( pps.getSliceWidthInTiles( sliceIdx ) );
          m_rectSlices[sliceIdx].setSliceHeightInTiles( pps.getSliceHeightInTiles( sliceIdx ) );
          m_rectSlices[sliceIdx].setNumSlicesInTile( pps.getNumSlicesInTile( sliceIdx ) );
          m_rectSlices[sliceIdx].setSliceHeightInCtu( pps.getSliceHeightInCtu( sliceIdx ) );
          m_rectSlices[sliceIdx].setTileIdx( pps.getSliceTileIdx( sliceIdx ) );
        }
      }
    }
    // raster-scan slices
    else {
      uint32_t listIdx  = 0;
      uint32_t remTiles = pps.getNumTiles();

      // set default slice size if not provided
      if ( m_rasterSliceSize.size() == 0 ) { m_rasterSliceSize.push_back( remTiles ); }

      // set raster slice sizes
      while ( remTiles > 0 ) {
        // truncate if size exceeds number of remaining tiles
        if ( listIdx < m_rasterSliceSize.size() ) {
          m_rasterSliceSize[listIdx] = std::min( remTiles, m_rasterSliceSize[listIdx] );
          remTiles -= m_rasterSliceSize[listIdx];
        }
        // replicate last size uniformly as needed to cover the remainder of the picture
        else {
          m_rasterSliceSize.push_back( std::min( remTiles, m_rasterSliceSize.back() ) );
          remTiles -= m_rasterSliceSize.back();
        }
        listIdx++;
      }
      // shrink list if too many sizes were provided
      m_rasterSliceSize.resize( listIdx );

      m_numSlicesInPic = (uint32_t)m_rasterSliceSize.size();
      xConfirmPara( m_rasterSliceSize.size() > getMaxSlicesByLevel( m_level ),
                    "Number of raster-scan slices exceeds maximum number allowed according to specified level" );
    }
  } else {
    m_numTileCols    = 1;
    m_numTileRows    = 1;
    m_numSlicesInPic = 1;
  }

  if ( ( m_MCTSEncConstraint ) && ( !m_disableLFCrossTileBoundaryFlag ) ) {
    printf(
        "Warning: Constrained Encoding for Motion Constrained Tile Sets (MCTS) is enabled. Disabling filtering across "
        "tile boundaries!\n" );
    m_disableLFCrossTileBoundaryFlag = true;
  }
  if ( ( m_MCTSEncConstraint ) && ( m_TMVPModeId ) ) {
    printf( "Warning: Constrained Encoding for Motion Constrained Tile Sets (MCTS) is enabled. Disabling TMVP!\n" );
    m_TMVPModeId = 0;
  }

  if ( ( m_MCTSEncConstraint ) && ( m_alf ) ) {
    printf( "Warning: Constrained Encoding for Motion Constrained Tile Sets (MCTS) is enabled. Disabling ALF!\n" );
    m_alf = false;
  }
  if ( ( m_MCTSEncConstraint ) && ( m_BIO ) ) {
    printf( "Warning: Constrained Encoding for Motion Constrained Tile Sets (MCTS) is enabled. Disabling BIO!\n" );
    m_BIO = false;
  }

  xConfirmPara( m_sariAspectRatioIdc < 0 || m_sariAspectRatioIdc > 255,
                "SEISARISampleAspectRatioIdc must be in the range of 0 to 255" );

  if ( m_RCEnableRateControl ) {
    if ( m_RCForceIntraQP ) {
      if ( m_RCInitialQP == 0 ) {
        msg( VTM_WARNING, "\nInitial QP for rate control is not specified. Reset not to use force intra QP!" );
        m_RCForceIntraQP = false;
      }
    }
    xConfirmPara( m_uiDeltaQpRD > 0,
                  "Rate control cannot be used together with slice level multiple-QP optimization!\n" );
#if U0132_TARGET_BITS_SATURATION
    if ( ( m_RCCpbSaturationEnabled ) && ( m_level != Level::NONE ) && ( m_profile != Profile::NONE ) ) {
      uint32_t uiLevelIdx =
          ( m_level / 10 ) + ( uint32_t )( ( m_level % 10 ) / 3 );  // (m_level / 30)*3 + ((m_level % 10) / 3);
      xConfirmPara( m_RCCpbSize > g_uiMaxCpbSize[m_levelTier][uiLevelIdx],
                    "RCCpbSize should be smaller than or equal to Max CPB size according to tier and level" );
      xConfirmPara( m_RCInitialCpbFullness > 1, "RCInitialCpbFullness should be smaller than or equal to 1" );
    }
#endif
  }
#if U0132_TARGET_BITS_SATURATION
  else {
    xConfirmPara( m_RCCpbSaturationEnabled != 0, "Target bits saturation cannot be processed without Rate control" );
  }
#endif

  if ( m_framePackingSEIEnabled ) {
    xConfirmPara( m_framePackingSEIType < 3 || m_framePackingSEIType > 5,
                  "SEIFramePackingType must be in rage 3 to 5" );
  }

  if ( m_erpSEIEnabled && !m_erpSEICancelFlag ) {
    xConfirmPara( m_erpSEIGuardBandType < 0 || m_erpSEIGuardBandType > 8,
                  "SEIEquirectangularprojectionGuardBandType must be in the range of 0 to 7" );
    xConfirmPara(
        ( m_chromaFormatIDC == CHROMA_420 || m_chromaFormatIDC == CHROMA_422 ) &&
            ( m_erpSEILeftGuardBandWidth % 2 == 1 ),
        "SEIEquirectangularprojectionLeftGuardBandWidth must be an even number for 4:2:0 or 4:2:2 chroma format" );
    xConfirmPara(
        ( m_chromaFormatIDC == CHROMA_420 || m_chromaFormatIDC == CHROMA_422 ) &&
            ( m_erpSEIRightGuardBandWidth % 2 == 1 ),
        "SEIEquirectangularprojectionRightGuardBandWidth must be an even number for 4:2:0 or 4:2:2 chroma format" );
  }

  if ( m_sphereRotationSEIEnabled && !m_sphereRotationSEICancelFlag ) {
    xConfirmPara( m_sphereRotationSEIYaw < -( 180 << 16 ) || m_sphereRotationSEIYaw > ( 180 << 16 ) - 1,
                  "SEISphereRotationYaw must be in the range of -11 796 480 to 11 796 479" );
    xConfirmPara( m_sphereRotationSEIPitch < -( 90 << 16 ) || m_sphereRotationSEIYaw > ( 90 << 16 ),
                  "SEISphereRotationPitch must be in the range of -5 898 240 to 5 898 240" );
    xConfirmPara( m_sphereRotationSEIRoll < -( 180 << 16 ) || m_sphereRotationSEIYaw > ( 180 << 16 ) - 1,
                  "SEISphereRotationRoll must be in the range of -11 796 480 to 11 796 479" );
  }

  if ( m_omniViewportSEIEnabled && !m_omniViewportSEICancelFlag ) {
    xConfirmPara( m_omniViewportSEIId < 0 || m_omniViewportSEIId > 1023,
                  "SEIomniViewportId must be in the range of 0 to 1023" );
    xConfirmPara( m_omniViewportSEICntMinus1 < 0 || m_omniViewportSEICntMinus1 > 15,
                  "SEIomniViewportCntMinus1 must be in the range of 0 to 15" );
    for ( uint32_t i = 0; i <= m_omniViewportSEICntMinus1; i++ ) {
      xConfirmPara(
          m_omniViewportSEIAzimuthCentre[i] < -( 180 << 16 ) || m_omniViewportSEIAzimuthCentre[i] > ( 180 << 16 ) - 1,
          "SEIOmniViewportAzimuthCentre must be in the range of -11 796 480 to 11 796 479" );
      xConfirmPara(
          m_omniViewportSEIElevationCentre[i] < -( 90 << 16 ) || m_omniViewportSEIElevationCentre[i] > ( 90 << 16 ),
          "SEIOmniViewportSEIElevationCentre must be in the range of -5 898 240 to 5 898 240" );
      xConfirmPara(
          m_omniViewportSEITiltCentre[i] < -( 180 << 16 ) || m_omniViewportSEITiltCentre[i] > ( 180 << 16 ) - 1,
          "SEIOmniViewportTiltCentre must be in the range of -11 796 480 to 11 796 479" );
      xConfirmPara( m_omniViewportSEIHorRange[i] < 1 || m_omniViewportSEIHorRange[i] > ( 360 << 16 ),
                    "SEIOmniViewportHorRange must be in the range of 1 to 360*2^16" );
      xConfirmPara( m_omniViewportSEIVerRange[i] < 1 || m_omniViewportSEIVerRange[i] > ( 180 << 16 ),
                    "SEIOmniViewportVerRange must be in the range of 1 to 180*2^16" );
    }
  }

  if ( m_gcmpSEIEnabled && !m_gcmpSEICancelFlag ) {
    xConfirmPara( m_gcmpSEIMappingFunctionType < 0 || m_gcmpSEIMappingFunctionType > 2,
                  "SEIGcmpMappingFunctionType must be in the range of 0 to 2" );
    int numFace = m_gcmpSEIPackingType == 4 || m_gcmpSEIPackingType == 5 ? 5 : 6;
    for ( int i = 0; i < numFace; i++ ) {
      xConfirmPara( m_gcmpSEIFaceIndex[i] < 0 || m_gcmpSEIFaceIndex[i] > 5,
                    "SEIGcmpFaceIndex must be in the range of 0 to 5" );
      xConfirmPara( m_gcmpSEIFaceRotation[i] < 0 || m_gcmpSEIFaceRotation[i] > 3,
                    "SEIGcmpFaceRotation must be in the range of 0 to 3" );
      if ( m_gcmpSEIMappingFunctionType == 2 ) {
        xConfirmPara( m_gcmpSEIFunctionCoeffU[i] <= 0.0 || m_gcmpSEIFunctionCoeffU[i] > 1.0,
                      "SEIGcmpFunctionCoeffU must be in the range (0, 1]" );
        xConfirmPara( m_gcmpSEIFunctionCoeffV[i] <= 0.0 || m_gcmpSEIFunctionCoeffV[i] > 1.0,
                      "SEIGcmpFunctionCoeffV must be in the range (0, 1]" );
      }
      if ( i != 2 && ( m_gcmpSEIPackingType == 4 || m_gcmpSEIPackingType == 5 ) ) {
        if ( m_gcmpSEIFaceIndex[2] == 0 || m_gcmpSEIFaceIndex[2] == 1 ) {
          xConfirmPara( m_gcmpSEIFaceIndex[i] == 0 || m_gcmpSEIFaceIndex[i] == 1,
                        "SEIGcmpFaceIndex[i] must be in the range of 2 to 5 for i equal to 0, 1, 3, or 4 when "
                        "SEIGcmpFaceIndex[2] is equal to 0 or 1" );
          if ( m_gcmpSEIPackingType == 4 ) {
            xConfirmPara( m_gcmpSEIFaceRotation[i] != 0 && m_gcmpSEIFaceRotation[i] != 2,
                          "SEIGcmpFaceRotation[i] must be 0 or 2 for i equal to 0, 1, 3, or 4 when SEIGcmpFaceIndex[2] "
                          "is equal to 0 or 1" );
          } else {
            xConfirmPara( m_gcmpSEIFaceRotation[i] != 1 && m_gcmpSEIFaceRotation[i] != 3,
                          "SEIGcmpFaceRotation[i] must be 1 or 3 for i equal to 0, 1, 3, or 4 when SEIGcmpFaceIndex[2] "
                          "is equal to 0 or 1" );
          }
        } else if ( m_gcmpSEIFaceIndex[2] == 2 || m_gcmpSEIFaceIndex[2] == 3 ) {
          xConfirmPara( m_gcmpSEIFaceIndex[i] == 2 || m_gcmpSEIFaceIndex[i] == 3,
                        "SEIGcmpFaceIndex[i] must be 0, 1, 4 or 5 for i equal to 0, 1, 3, or 4 when "
                        "SEIGcmpFaceIndex[2] is equal to 2 or 3" );
          if ( m_gcmpSEIPackingType == 4 ) {
            if ( m_gcmpSEIFaceIndex[i] == 1 ) {
              xConfirmPara( m_gcmpSEIFaceRotation[i] != 0 && m_gcmpSEIFaceRotation[i] != 2,
                            "SEIGcmpFaceRotation[i] must be 0 or 2 when SEIGcmpFaceIndex[2] is equal to 2 or 3 and "
                            "SEIGcmpFaceIndex[i] is equal to 1" );
            } else {
              xConfirmPara( m_gcmpSEIFaceRotation[i] != 1 && m_gcmpSEIFaceRotation[i] != 3,
                            "SEIGcmpFaceRotation[i] must be 1 or 3 when SEIGcmpFaceIndex[2] is equal to 2 or 3 and "
                            "SEIGcmpFaceIndex[i] is equal to 0, 4 or 5" );
            }
          } else {
            if ( m_gcmpSEIFaceIndex[i] == 1 ) {
              xConfirmPara( m_gcmpSEIFaceRotation[i] != 1 && m_gcmpSEIFaceRotation[i] != 3,
                            "SEIGcmpFaceRotation[i] must be 1 or 3 when SEIGcmpFaceIndex[2] is equal to 2 or 3 and "
                            "SEIGcmpFaceIndex[i] is equal to 1" );
            } else {
              xConfirmPara( m_gcmpSEIFaceRotation[i] != 0 && m_gcmpSEIFaceRotation[i] != 2,
                            "SEIGcmpFaceRotation[i] must be 0 or 2 when SEIGcmpFaceIndex[2] is equal to 2 or 3 and "
                            "SEIGcmpFaceIndex[i] is equal to 0, 4 or 5" );
            }
          }
        } else if ( m_gcmpSEIFaceIndex[2] == 4 || m_gcmpSEIFaceIndex[2] == 5 ) {
          xConfirmPara( m_gcmpSEIFaceIndex[i] == 4 || m_gcmpSEIFaceIndex[i] == 5,
                        "SEIGcmpFaceIndex[i] must be in the range of 0 to 3 for i equal to 0, 1, 3, or 4 when "
                        "SEIGcmpFaceIndex[2] is equal to 4 or 5" );
          if ( m_gcmpSEIPackingType == 4 ) {
            if ( m_gcmpSEIFaceIndex[i] == 0 ) {
              xConfirmPara( m_gcmpSEIFaceRotation[i] != 0 && m_gcmpSEIFaceRotation[i] != 2,
                            "SEIGcmpFaceRotation[i] must be 0 or 2 when SEIGcmpFaceIndex[2] is equal to 4 or 5 and "
                            "SEIGcmpFaceIndex[i] is equal to 0" );
            } else {
              xConfirmPara( m_gcmpSEIFaceRotation[i] != 1 && m_gcmpSEIFaceRotation[i] != 3,
                            "SEIGcmpFaceRotation[i] must be 1 or 3 when SEIGcmpFaceIndex[2] is equal to 4 or 5 and "
                            "SEIGcmpFaceIndex[i] is equal to 1, 2 or 3" );
            }
          } else {
            if ( m_gcmpSEIFaceIndex[i] == 0 ) {
              xConfirmPara( m_gcmpSEIFaceRotation[i] != 1 && m_gcmpSEIFaceRotation[i] != 3,
                            "SEIGcmpFaceRotation[i] must be 1 or 3 when SEIGcmpFaceIndex[2] is equal to 4 or 5 and "
                            "SEIGcmpFaceIndex[i] is equal to 0" );
            } else {
              xConfirmPara( m_gcmpSEIFaceRotation[i] != 0 && m_gcmpSEIFaceRotation[i] != 2,
                            "SEIGcmpFaceRotation[i] must be 0 or 2 when SEIGcmpFaceIndex[2] is equal to 4 or 5 and "
                            "SEIGcmpFaceIndex[i] is equal to 1, 2 or 3" );
            }
          }
        }
      }
    }
    if ( m_gcmpSEIGuardBandFlag ) {
      xConfirmPara( m_gcmpSEIGuardBandSamplesMinus1 < 0 || m_gcmpSEIGuardBandSamplesMinus1 > 15,
                    "SEIGcmpGuardBandSamplesMinus1 must be in the range of 0 to 15" );
    }
  }
  xConfirmPara( m_log2ParallelMergeLevel < 2, "Log2ParallelMergeLevel should be larger than or equal to 2" );
  xConfirmPara( m_log2ParallelMergeLevel > m_uiCTUSize,
                "Log2ParallelMergeLevel should be less than or equal to CTU size" );
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  xConfirmPara( m_preferredTransferCharacteristics > 255,
                "transfer_characteristics_idc should not be greater than 255." );
#endif
  xConfirmPara( unsigned( m_ImvMode ) > 1, "ImvMode exceeds range (0 to 1)" );
  if ( m_AffineAmvr ) { xConfirmPara( !m_ImvMode, "AffineAmvr cannot be used when IMV is disabled." ); }
  xConfirmPara( m_decodeBitstreams[0] == m_bitstreamFileName,
                "Debug bitstream and the output bitstream cannot be equal.\n" );
  xConfirmPara( m_decodeBitstreams[1] == m_bitstreamFileName,
                "Decode2 bitstream and the output bitstream cannot be equal.\n" );
  xConfirmPara( unsigned( m_LMChroma ) > 1, "LMMode exceeds range (0 to 1)" );
  if ( m_gopBasedTemporalFilterEnabled ) {
    xConfirmPara( m_temporalSubsampleRatio != 1, "GOP Based Temporal Filter only support Temporal sub-sample ratio 1" );
  }
#if EXTENSION_360_VIDEO
  check_failed |= m_ext360.verifyParameters();
#endif

  xConfirmPara( m_useColorTrans && ( m_log2MaxTbSize == 6 ),
                "Log2MaxTbSize must be less than 6 when ACT is enabled, otherwise ACT needs to be disabled" );

  xConfirmPara( m_uiCTUSize <= 32 && ( m_log2MaxTbSize == 6 ),
                "Log2MaxTbSize must be less than 6 when CTU size is 32" );

#undef xConfirmPara
  return check_failed;
}

const char* profileToString( const Profile::Name profile ) {
  static const uint32_t numberOfProfiles = sizeof( strToProfile ) / sizeof( *strToProfile );

  for ( uint32_t profileIndex = 0; profileIndex < numberOfProfiles; profileIndex++ ) {
    if ( strToProfile[profileIndex].value == profile ) { return strToProfile[profileIndex].str; }
  }

  // if we get here, we didn't find this profile in the list - so there is an error
  EXIT( "VTM_ERROR: Unknown profile \"" << profile << "\" in profileToString" );
  return "";
}

void PCCVTMLibVideoEncoderCfg::xPrintParameter() {
  // msg( VTM_DETAILS, "\n" );
  msg( VTM_DETAILS, "Input          File                    : %s\n", m_inputFileName.c_str() );
  msg( VTM_DETAILS, "Bitstream      File                    : %s\n", m_bitstreamFileName.c_str() );
  msg( VTM_DETAILS, "Reconstruction File                    : %s\n", m_reconFileName.c_str() );
  msg( VTM_DETAILS, "Real     Format                        : %dx%d %gHz\n",
       m_sourceWidth - m_confWinLeft - m_confWinRight, m_sourceHeight - m_confWinTop - m_confWinBottom,
       (double)m_iFrameRate / m_temporalSubsampleRatio );
  msg( VTM_DETAILS, "Internal Format                        : %dx%d %gHz\n", m_sourceWidth, m_sourceHeight,
       (double)m_iFrameRate / m_temporalSubsampleRatio );
  msg( VTM_DETAILS, "Sequence PSNR output                   : %s\n",
       ( m_printMSEBasedSequencePSNR ? "Linear average, MSE-based" : "Linear average only" ) );
  msg( VTM_DETAILS, "Hexadecimal PSNR output                : %s\n", ( m_printHexPsnr ? "Enabled" : "Disabled" ) );
  msg( VTM_DETAILS, "Sequence MSE output                    : %s\n", ( m_printSequenceMSE ? "Enabled" : "Disabled" ) );
  msg( VTM_DETAILS, "Frame MSE output                       : %s\n", ( m_printFrameMSE ? "Enabled" : "Disabled" ) );
  msg( VTM_DETAILS, "MS-SSIM output                         : %s\n", ( m_printMSSSIM ? "Enabled" : "Disabled" ) );
  msg( VTM_DETAILS, "Cabac-zero-word-padding                : %s\n",
       ( m_cabacZeroWordPaddingEnabled ? "Enabled" : "Disabled" ) );
  if ( m_isField ) {
    msg( VTM_DETAILS, "Frame/Field                            : Field based coding\n" );
    msg( VTM_DETAILS, "Field index                            : %u - %d (%d fields)\n", m_FrameSkip,
         m_FrameSkip + m_framesToBeEncoded - 1, m_framesToBeEncoded );
    msg( VTM_DETAILS, "Field Order                            : %s field first\n",
         m_isTopFieldFirst ? "Top" : "Bottom" );

  } else {
    msg( VTM_DETAILS, "Frame/Field                            : Frame based coding\n" );
    msg( VTM_DETAILS, "Frame index                            : %u - %d (%d frames)\n", m_FrameSkip,
         m_FrameSkip + m_framesToBeEncoded - 1, m_framesToBeEncoded );
  }
  { msg( VTM_DETAILS, "Profile                                : %s\n", profileToString( m_profile ) ); }
  msg( VTM_DETAILS, "CTU size / min CU size                 : %d / %d \n", m_uiMaxCUWidth, 1 << m_log2MinCuSize );

  msg( VTM_DETAILS, "subpicture info present flag           : %s\n", m_subPicInfoPresentFlag ? "Enabled" : "Disabled" );
  if ( m_subPicInfoPresentFlag ) {
    msg( VTM_DETAILS, "number of subpictures                  : %d\n", m_numSubPics );
    msg( VTM_DETAILS, "subpicture size same flag              : %d\n", m_subPicSameSizeFlag );
    if ( m_subPicSameSizeFlag ) {
      msg( VTM_DETAILS, "[0]th subpicture size                  : [%d %d]\n", m_subPicWidth[0], m_subPicHeight[0] );
    }
    for ( int i = 0; i < m_numSubPics; i++ ) {
      if ( !m_subPicSameSizeFlag ) {
        msg( VTM_DETAILS, "[%d]th subpicture location              : [%d %d]\n", i, m_subPicCtuTopLeftX[i],
             m_subPicCtuTopLeftY[i] );
        msg( VTM_DETAILS, "[%d]th subpicture size                  : [%d %d]\n", i, m_subPicWidth[i],
             m_subPicHeight[i] );
      }
      msg( VTM_DETAILS, "[%d]th subpicture treated as picture    : %d\n", i,
           m_subPicTreatedAsPicFlag[i] ? "Enabled" : "Disabled" );
      msg( VTM_DETAILS, "loop filter across [%d]th subpicture    : %d\n", i,
           m_loopFilterAcrossSubpicEnabledFlag[i] ? "Enabled" : "Disabled" );
    }
  }

  msg( VTM_DETAILS, "subpicture ID present flag             : %s\n",
       m_subPicIdMappingExplicitlySignalledFlag ? "Enabled" : "Disabled" );
  if ( m_subPicIdMappingExplicitlySignalledFlag ) {
    msg( VTM_DETAILS, "subpicture ID signalling present flag  : %d\n", m_subPicIdMappingInSpsFlag );
    for ( int i = 0; i < m_numSubPics; i++ ) {
      msg( VTM_DETAILS, "[%d]th subpictures ID length           : %d\n", i, m_subPicIdLen );
      msg( VTM_DETAILS, "[%d]th subpictures ID                  : %d\n", i, m_subPicId[i] );
    }
  }
  msg( VTM_DETAILS, "Max TB size                            : %d \n", 1 << m_log2MaxTbSize );
  msg( VTM_DETAILS, "Motion search range                    : %d\n", m_iSearchRange );
  msg( VTM_DETAILS, "Intra period                           : %d\n", m_iIntraPeriod );
  msg( VTM_DETAILS, "Decoding refresh type                  : %d\n", m_iDecodingRefreshType );
  msg( VTM_DETAILS, "DRAP period                            : %d\n", m_drapPeriod );
#if QP_SWITCHING_FOR_PARALLEL
  if ( m_qpIncrementAtSourceFrame.bPresent ) {
    msg( VTM_DETAILS, "QP                                     : %d (incrementing internal QP at source frame %d)\n",
         m_iQP, m_qpIncrementAtSourceFrame.value );
  } else {
    msg( VTM_DETAILS, "QP                                     : %d\n", m_iQP );
  }
#else
  msg( VTM_DETAILS, "QP                                     : %5.2f\n", m_fQP );
#endif
  msg( VTM_DETAILS, "Max dQP signaling subdiv               : %d\n", m_cuQpDeltaSubdiv );

  msg( VTM_DETAILS, "Cb QP Offset (dual tree)               : %d (%d)\n", m_cbQpOffset, m_cbQpOffsetDualTree );
  msg( VTM_DETAILS, "Cr QP Offset (dual tree)               : %d (%d)\n", m_crQpOffset, m_crQpOffsetDualTree );
  msg( VTM_DETAILS, "QP adaptation                          : %d (range=%d)\n", m_bUseAdaptiveQP,
       ( m_bUseAdaptiveQP ? m_iQPAdaptationRange : 0 ) );
  msg( VTM_DETAILS, "GOP size                               : %d\n", m_iGOPSize );
  msg( VTM_DETAILS, "Input bit depth                        : (Y:%d, C:%d)\n", m_inputBitDepth[CHANNEL_TYPE_LUMA],
       m_inputBitDepth[CHANNEL_TYPE_CHROMA] );
  msg( VTM_DETAILS, "MSB-extended bit depth                 : (Y:%d, C:%d)\n", m_MSBExtendedBitDepth[CHANNEL_TYPE_LUMA],
       m_MSBExtendedBitDepth[CHANNEL_TYPE_CHROMA] );
  msg( VTM_DETAILS, "Internal bit depth                     : (Y:%d, C:%d)\n", m_internalBitDepth[CHANNEL_TYPE_LUMA],
       m_internalBitDepth[CHANNEL_TYPE_CHROMA] );
  msg( VTM_DETAILS, "Intra reference smoothing              : %s\n",
       ( m_enableIntraReferenceSmoothing ? "Enabled" : "Disabled" ) );
  if ( m_cuChromaQpOffsetList.size() > 0 ) {
    msg( VTM_DETAILS, "Chroma QP offset list                  : (" );
    for ( int i = 0; i < m_cuChromaQpOffsetList.size(); i++ ) {
      msg( VTM_DETAILS, "%d %d %d%s", m_cuChromaQpOffsetList[i].u.comp.CbOffset,
           m_cuChromaQpOffsetList[i].u.comp.CrOffset, m_cuChromaQpOffsetList[i].u.comp.JointCbCrOffset,
           ( i + 1 < m_cuChromaQpOffsetList.size() ? ", " : ")\n" ) );
    }
    msg( VTM_DETAILS, "cu_chroma_qp_offset_subdiv             : %d\n", m_cuChromaQpOffsetSubdiv );
    msg( VTM_DETAILS, "cu_chroma_qp_offset_enabled_flag       : %s\n",
         ( m_cuChromaQpOffsetEnabled ? "Enabled" : "Disabled" ) );
  } else {
    msg( VTM_DETAILS, "Chroma QP offset list                  : Disabled\n" );
  }
  msg( VTM_DETAILS, "extended_precision_processing_flag     : %s\n",
       ( m_extendedPrecisionProcessingFlag ? "Enabled" : "Disabled" ) );
#if JVET_V0054_TSRC_RICE
  msg( VTM_DETAILS, "TSRC_Rice_present_flag                 : %s\n",
       ( m_tsrcRicePresentFlag ? "Enabled" : "Disabled" ) );
#endif
  msg( VTM_DETAILS, "transform_skip_rotation_enabled_flag   : %s\n",
       ( m_transformSkipRotationEnabledFlag ? "Enabled" : "Disabled" ) );
  msg( VTM_DETAILS, "transform_skip_context_enabled_flag    : %s\n",
       ( m_transformSkipContextEnabledFlag ? "Enabled" : "Disabled" ) );
  msg( VTM_DETAILS, "high_precision_offsets_enabled_flag    : %s\n",
       ( m_highPrecisionOffsetsEnabledFlag ? "Enabled" : "Disabled" ) );
#if JVET_V0106_RRC_RICE
  msg( VTM_DETAILS, "rrc_rice_extension_flag                : %s\n",
       ( m_rrcRiceExtensionEnableFlag ? "Enabled" : "Disabled" ) );
#endif
  msg( VTM_DETAILS, "persistent_rice_adaptation_enabled_flag: %s\n",
       ( m_persistentRiceAdaptationEnabledFlag ? "Enabled" : "Disabled" ) );
  msg( VTM_DETAILS, "cabac_bypass_alignment_enabled_flag    : %s\n",
       ( m_cabacBypassAlignmentEnabledFlag ? "Enabled" : "Disabled" ) );

  switch ( m_costMode ) {
    case COST_STANDARD_LOSSY:
      msg( VTM_DETAILS, "Cost function:                         : Lossy coding (default)\n" );
      break;
    case COST_SEQUENCE_LEVEL_LOSSLESS:
      msg( VTM_DETAILS, "Cost function:                         : Sequence_level_lossless coding\n" );
      break;
    case COST_LOSSLESS_CODING:
      msg( VTM_DETAILS, "Cost function:                         : Lossless coding with fixed QP of %d\n",
           LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP );
      break;
    case COST_MIXED_LOSSLESS_LOSSY_CODING:
      msg( VTM_DETAILS,
           "Cost function:                         : Mixed_lossless_lossy coding with QP'=%d for lossless evaluation\n",
           LOSSLESS_AND_MIXED_LOSSLESS_RD_COST_TEST_QP_PRIME );
      break;
    default: msg( VTM_DETAILS, "Cost function:                         : Unknown\n" ); break;
  }

  msg( VTM_DETAILS, "RateControl                            : %d\n", m_RCEnableRateControl );
  msg( VTM_DETAILS, "WeightedPredMethod                     : %d\n", int( m_weightedPredictionMethod ) );

  if ( m_RCEnableRateControl ) {
    msg( VTM_DETAILS, "TargetBitrate                          : %d\n", m_RCTargetBitrate );
    msg( VTM_DETAILS, "KeepHierarchicalBit                    : %d\n", m_RCKeepHierarchicalBit );
    msg( VTM_DETAILS, "LCULevelRC                             : %d\n", m_RCLCULevelRC );
    msg( VTM_DETAILS, "UseLCUSeparateModel                    : %d\n", m_RCUseLCUSeparateModel );
    msg( VTM_DETAILS, "InitialQP                              : %d\n", m_RCInitialQP );
    msg( VTM_DETAILS, "ForceIntraQP                           : %d\n", m_RCForceIntraQP );
#if U0132_TARGET_BITS_SATURATION
    msg( VTM_DETAILS, "CpbSaturation                          : %d\n", m_RCCpbSaturationEnabled );
    if ( m_RCCpbSaturationEnabled ) {
      msg( VTM_DETAILS, "CpbSize                                : %d\n", m_RCCpbSize );
      msg( VTM_DETAILS, "InitalCpbFullness                      : %.2f\n", m_RCInitialCpbFullness );
    }
#endif
  }

#if GDR_ENABLED
  msg( VTM_DETAILS, "GDREnabled                             : %d\n", m_gdrEnabled );

  if ( m_gdrEnabled ) {
    msg( VTM_DETAILS, "GDR Start                              : %d\n", m_gdrPocStart );
    msg( VTM_DETAILS, "GDR Interval                           : %d\n", m_gdrInterval );
    msg( VTM_DETAILS, "GDR Period                             : %d\n", m_gdrPeriod );
  }
#endif

  msg( VTM_DETAILS, "Max Num Merge Candidates               : %d\n", m_maxNumMergeCand );
  msg( VTM_DETAILS, "Max Num Affine Merge Candidates        : %d\n", m_maxNumAffineMergeCand );
  msg( VTM_DETAILS, "Max Num Geo Merge Candidates           : %d\n", m_maxNumGeoCand );
  msg( VTM_DETAILS, "Max Num IBC Merge Candidates           : %d\n", m_maxNumIBCMergeCand );
  msg( VTM_DETAILS, "\n" );

  msg( VTM_VERBOSE, "TOOL CFG: " );
  msg( VTM_VERBOSE, "IBD:%d ",
       ( ( m_internalBitDepth[CHANNEL_TYPE_LUMA] > m_MSBExtendedBitDepth[CHANNEL_TYPE_LUMA] ) ||
         ( m_internalBitDepth[CHANNEL_TYPE_CHROMA] > m_MSBExtendedBitDepth[CHANNEL_TYPE_CHROMA] ) ) );
  msg( VTM_VERBOSE, "HAD:%d ", m_bUseHADME );
  msg( VTM_VERBOSE, "RDQ:%d ", m_useRDOQ );
  msg( VTM_VERBOSE, "RDQTS:%d ", m_useRDOQTS );
  msg( VTM_VERBOSE, "RDpenalty:%d ", m_rdPenalty );
#if SHARP_LUMA_DELTA_QP
  msg( VTM_VERBOSE, "LQP:%d ", m_lumaLevelToDeltaQPMapping.mode );
#endif
  msg( VTM_VERBOSE, "SQP:%d ", m_uiDeltaQpRD );
  msg( VTM_VERBOSE, "ASR:%d ", m_bUseASR );
  msg( VTM_VERBOSE, "MinSearchWindow:%d ", m_minSearchWindow );
  msg( VTM_VERBOSE, "RestrictMESampling:%d ", m_bRestrictMESampling );
  msg( VTM_VERBOSE, "FEN:%d ", int( m_fastInterSearchMode ) );
  msg( VTM_VERBOSE, "ECU:%d ", m_bUseEarlyCU );
  msg( VTM_VERBOSE, "FDM:%d ", m_useFastDecisionForMerge );
  msg( VTM_VERBOSE, "ESD:%d ", m_useEarlySkipDetection );
  msg( VTM_VERBOSE, "TransformSkip:%d ", m_useTransformSkip );
  msg( VTM_VERBOSE, "TransformSkipFast:%d ", m_useTransformSkipFast );
  msg( VTM_VERBOSE, "TransformSkipLog2MaxSize:%d ", m_log2MaxTransformSkipBlockSize );
  msg( VTM_VERBOSE, "ChromaTS:%d ", m_useChromaTS );
  msg( VTM_VERBOSE, "BDPCM:%d ", m_useBDPCM );
  msg( VTM_VERBOSE, "Tiles: %dx%d ", m_numTileCols, m_numTileRows );
  msg( VTM_VERBOSE, "Slices: %d ", m_numSlicesInPic );
  msg( VTM_VERBOSE, "MCTS:%d ", m_MCTSEncConstraint );
  msg( VTM_VERBOSE, "SAO:%d ", ( m_bUseSAO ) ? ( 1 ) : ( 0 ) );
  msg( VTM_VERBOSE, "ALF:%d ", m_alf ? 1 : 0 );
  msg( VTM_VERBOSE, "CCALF:%d ", m_ccalf ? 1 : 0 );

  msg( VTM_VERBOSE, "WPP:%d ", (int)m_useWeightedPred );
  msg( VTM_VERBOSE, "WPB:%d ", (int)m_useWeightedBiPred );
  msg( VTM_VERBOSE, "PME:%d ", m_log2ParallelMergeLevel );
  const int iWaveFrontSubstreams =
      m_entropyCodingSyncEnabledFlag ? ( m_sourceHeight + m_uiMaxCUHeight - 1 ) / m_uiMaxCUHeight : 1;
  msg( VTM_VERBOSE, " WaveFrontSynchro:%d WaveFrontSubstreams:%d", m_entropyCodingSyncEnabledFlag ? 1 : 0,
       iWaveFrontSubstreams );
  msg( VTM_VERBOSE, " ScalingList:%d ", m_useScalingListId );
  msg( VTM_VERBOSE, "TMVPMode:%d ", m_TMVPModeId );
  msg( VTM_VERBOSE, " DQ:%d ", m_depQuantEnabledFlag );
  msg( VTM_VERBOSE, " SignBitHidingFlag:%d ", m_signDataHidingEnabledFlag );
  msg( VTM_VERBOSE, "RecalQP:%d ", m_recalculateQPAccordingToLambda ? 1 : 0 );

  {
    msg( VTM_VERBOSE, "\nTOOL CFG: " );
    msg( VTM_VERBOSE, "LFNST:%d ", m_LFNST );
    msg( VTM_VERBOSE, "MMVD:%d ", m_MMVD );
    msg( VTM_VERBOSE, "Affine:%d ", m_Affine );
    if ( m_Affine ) { msg( VTM_VERBOSE, "AffineType:%d ", m_AffineType ); }
    msg( VTM_VERBOSE, "PROF:%d ", m_PROF );
    msg( VTM_VERBOSE, "SbTMVP:%d ", m_sbTmvpEnableFlag );
    msg( VTM_VERBOSE, "DualITree:%d ", m_dualTree );
    msg( VTM_VERBOSE, "IMV:%d ", m_ImvMode );
    msg( VTM_VERBOSE, "BIO:%d ", m_BIO );
    msg( VTM_VERBOSE, "LMChroma:%d ", m_LMChroma );
    msg( VTM_VERBOSE, "HorCollocatedChroma:%d ", m_horCollocatedChromaFlag );
    msg( VTM_VERBOSE, "VerCollocatedChroma:%d ", m_verCollocatedChromaFlag );
    msg( VTM_VERBOSE, "MTS: %1d(intra) %1d(inter) ", m_MTS & 1, ( m_MTS >> 1 ) & 1 );
    msg( VTM_VERBOSE, "SBT:%d ", m_SBT );
    msg( VTM_VERBOSE, "ISP:%d ", m_ISP );
    msg( VTM_VERBOSE, "SMVD:%d ", m_SMVD );
    msg( VTM_VERBOSE, "CompositeLTReference:%d ", m_compositeRefEnabled );
    msg( VTM_VERBOSE, "Bcw:%d ", m_bcw );
    msg( VTM_VERBOSE, "BcwFast:%d ", m_BcwFast );
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
    msg( VTM_VERBOSE, "LADF:%d ", m_LadfEnabed );
#endif
    msg( VTM_VERBOSE, "CIIP:%d ", m_ciip );
    msg( VTM_VERBOSE, "Geo:%d ", m_Geo );
    m_allowDisFracMMVD = m_MMVD ? m_allowDisFracMMVD : false;
    if ( m_MMVD ) msg( VTM_VERBOSE, "AllowDisFracMMVD:%d ", m_allowDisFracMMVD );
    msg( VTM_VERBOSE, "AffineAmvr:%d ", m_AffineAmvr );
    m_AffineAmvrEncOpt = m_AffineAmvr ? m_AffineAmvrEncOpt : false;
    msg( VTM_VERBOSE, "AffineAmvrEncOpt:%d ", m_AffineAmvrEncOpt );
    msg( VTM_VERBOSE, "DMVR:%d ", m_DMVR );
    msg( VTM_VERBOSE, "MmvdDisNum:%d ", m_MmvdDisNum );
    msg( VTM_VERBOSE, "JointCbCr:%d ", m_JointCbCrMode );
  }
  m_useColorTrans = ( m_chromaFormatIDC == CHROMA_444 ) ? m_useColorTrans : 0u;
  msg( VTM_VERBOSE, "ACT:%d ", m_useColorTrans );
  msg( VTM_VERBOSE, "PLT:%d ", m_PLTMode );
  msg( VTM_VERBOSE, "IBC:%d ", m_IBCMode );
  msg( VTM_VERBOSE, "HashME:%d ", m_HashME );
  msg( VTM_VERBOSE, "WrapAround:%d ", m_wrapAround );
  if ( m_wrapAround ) { msg( VTM_VERBOSE, "WrapAroundOffset:%d ", m_wrapAroundOffset ); }
  // ADD_NEW_TOOL (add some output indicating the usage of tools)
  msg( VTM_VERBOSE, "VirtualBoundariesEnabledFlag:%d ", m_virtualBoundariesEnabledFlag );
  msg( VTM_VERBOSE, "VirtualBoundariesPresentInSPSFlag:%d ", m_virtualBoundariesPresentFlag );
  if ( m_virtualBoundariesPresentFlag ) {
    msg( VTM_VERBOSE, "vertical virtual boundaries:[" );
    for ( unsigned i = 0; i < m_numVerVirtualBoundaries; i++ ) {
      msg( VTM_VERBOSE, " %d", m_virtualBoundariesPosX[i] );
    }
    msg( VTM_VERBOSE, " ] horizontal virtual boundaries:[" );
    for ( unsigned i = 0; i < m_numHorVirtualBoundaries; i++ ) {
      msg( VTM_VERBOSE, " %d", m_virtualBoundariesPosY[i] );
    }
    msg( VTM_VERBOSE, " ] " );
  }
  msg( VTM_VERBOSE, "Reshape:%d ", m_lmcsEnabled );
  if ( m_lmcsEnabled ) {
    msg( VTM_VERBOSE, "(Signal:%s ",
         m_reshapeSignalType == 0 ? "SDR" : ( m_reshapeSignalType == 2 ? "HDR-HLG" : "HDR-PQ" ) );
    msg( VTM_VERBOSE, "Opt:%d", m_adpOption );
    if ( m_adpOption > 0 ) { msg( VTM_VERBOSE, " CW:%d", m_initialCW ); }
    msg( VTM_VERBOSE, " CSoffset:%d", m_CSoffset );
    msg( VTM_VERBOSE, ") " );
  }
  msg( VTM_VERBOSE, "MRL:%d ", m_MRL );
  msg( VTM_VERBOSE, "MIP:%d ", m_MIP );
  msg( VTM_VERBOSE, "EncDbOpt:%d ", m_encDbOpt );
  msg( VTM_VERBOSE, "\nFAST TOOL CFG: " );
  msg( VTM_VERBOSE, "LCTUFast:%d ", m_useFastLCTU );
  msg( VTM_VERBOSE, "FastMrg:%d ", m_useFastMrg );
  msg( VTM_VERBOSE, "PBIntraFast:%d ", m_usePbIntraFast );
  if ( m_ImvMode ) msg( VTM_VERBOSE, "IMV4PelFast:%d ", m_Imv4PelFast );
  if ( m_MTS ) msg( VTM_VERBOSE, "MTSMaxCand: %1d(intra) %1d(inter) ", m_MTSIntraMaxCand, m_MTSInterMaxCand );
  if ( m_ISP ) msg( VTM_VERBOSE, "ISPFast:%d ", m_useFastISP );
  if ( m_LFNST ) msg( VTM_VERBOSE, "FastLFNST:%d ", m_useFastLFNST );
  msg( VTM_VERBOSE, "AMaxBT:%d ", m_useAMaxBT );
  msg( VTM_VERBOSE, "E0023FastEnc:%d ", m_e0023FastEnc );
  msg( VTM_VERBOSE, "ContentBasedFastQtbt:%d ", m_contentBasedFastQtbt );
  msg( VTM_VERBOSE, "UseNonLinearAlfLuma:%d ", m_useNonLinearAlfLuma );
  msg( VTM_VERBOSE, "UseNonLinearAlfChroma:%d ", m_useNonLinearAlfChroma );
  msg( VTM_VERBOSE, "MaxNumAlfAlternativesChroma:%d ", m_maxNumAlfAlternativesChroma );
  if ( m_MIP ) msg( VTM_VERBOSE, "FastMIP:%d ", m_useFastMIP );
  msg( VTM_VERBOSE, "FastLocalDualTree:%d ", m_fastLocalDualTreeMode );

  if ( m_resChangeInClvsEnabled ) {
    msg( VTM_VERBOSE, "RPR:(%1.2lfx, %1.2lfx)|%d ", m_scalingRatioHor, m_scalingRatioVer, m_switchPocPeriod );
  } else {
    msg( VTM_VERBOSE, "RPR:%d ", 0 );
  }
  msg( VTM_VERBOSE, "TemporalFilter:%d ", m_gopBasedTemporalFilterEnabled );
#if EXTENSION_360_VIDEO
  m_ext360.outputConfigurationSummary();
#endif

  msg( VTM_VERBOSE, "\n\n" );

  msg( VTM_NOTICE, "\n" );

  fflush( stdout );
}

bool PCCVTMLibVideoEncoderCfg::xHasNonZeroTemporalID() {
  for ( unsigned int i = 0; i < m_iGOPSize; i++ ) {
    if ( m_GOPList[i].m_temporalId != 0 ) { return true; }
  }
  return false;
}

bool PCCVTMLibVideoEncoderCfg::xHasLeadingPicture() {
  for ( unsigned int i = 0; i < m_iGOPSize; i++ ) {
    for ( unsigned int j = 0; j < m_GOPList[i].m_numRefPics0; j++ ) {
      if ( m_GOPList[i].m_deltaRefPics0[j] < 0 ) { return true; }
    }
    for ( unsigned int j = 0; j < m_GOPList[i].m_numRefPics1; j++ ) {
      if ( m_GOPList[i].m_deltaRefPics1[j] < 0 ) { return true; }
    }
  }
  return false;
}

bool confirmPara( bool bflag, const char* message ) {
  if ( !bflag ) { return false; }

  msg( VTM_ERROR, "Error: %s\n", message );
  return true;
}

#endif

//! \}
