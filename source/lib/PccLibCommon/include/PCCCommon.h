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
#include <unordered_map>
#include <sys/stat.h>
#include <memory>
#include <queue>
#include <algorithm>
#include <map>
#include <array>
#include "PCCConfig.h"
#include "PCCBitstreamCommon.h"
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
#undef OPTIONAL

namespace pcc {
#define COLOR_SMOOTHING_ALIGNMENT 1
#define CELLID_UNORDERED_MAP 1
#define PLR_FIX 1
// ******************************************************************* //
// Version information
// ******************************************************************* //
#define TM2_VERSION TMC2_VERSION_MAJOR "." TMC2_VERSION_MINOR

// ******************************************************************* //
// Coding tool configuration
// ******************************************************************* //
static const uint8_t PCC_SAVE_POINT_TYPE = 0;  // Save point information in reconstructed ply.
#define POSTSMOOTHING_RGB2YUV 0                // BT709 RGB to YUV convertion used for post-smoothing

// ******************************************************************* //
// Common constants
// ******************************************************************* //
enum PCCEndianness { PCC_BIG_ENDIAN = 0, PCC_LITTLE_ENDIAN = 1 };
enum PCCColorTransform { COLOR_TRANSFORM_NONE = 0, COLOR_TRANSFORM_RGB_TO_YCBCR = 1 };
enum PCCPointType { POINT_UNSET = 0, POINT_D0, POINT_D1, POINT_DF, POINT_SMOOTH, POINT_EOM, POINT_RAW };
enum { COLOURFORMAT420 = 0, COLOURFORMAT444 = 1 };
enum PCCCOLORFORMAT { UNKNOWN = 0, RGB444, YUV444, YUV420 };
enum PCCCodecId {
#ifdef USE_JMAPP_VIDEO_CODEC
  JMAPP = 0,
#endif
#ifdef USE_HMAPP_VIDEO_CODEC
  HMAPP = 1,
#endif
#ifdef USE_SHMAPP_VIDEO_CODEC
  SHMAPP = 2,
#endif
#ifdef USE_JMLIB_VIDEO_CODEC
  JMLIB = 3,
#endif
#ifdef USE_HMLIB_VIDEO_CODEC
  HMLIB = 4,
#endif
#ifdef USE_VTMLIB_VIDEO_CODEC
  VTMLIB = 5,
#endif
#ifdef USE_VVLIB_VIDEO_CODEC
  VVLIB = 6,
#endif
#ifdef USE_FFMPEG_VIDEO_CODEC
  FFMPEG = 7,
#endif
  UNKNOWN_CODEC = 255
};

// ******************************************************************* //
// Global variables
// ******************************************************************* //
const int16_t          g_infiniteDepth          = ( std::numeric_limits<int16_t>::max )();
const int64_t          g_infinitenumber         = ( std::numeric_limits<int64_t>::max )();
const int32_t          g_invalidPatchIndex      = -1;
const uint32_t         g_undefined_index        = -1;
const bool             g_printDetailedInfo      = false;
const size_t           g_intermediateLayerIndex = 100;
const size_t           g_neighborThreshold      = 4;
const std::vector<int> g_orientationVertical    = {
    PATCH_ORIENTATION_DEFAULT,  // Vertical orientation default
    PATCH_ORIENTATION_SWAP,     // Vertical orientation swap
    PATCH_ORIENTATION_ROT180,   // Vertical orientation rot180
    PATCH_ORIENTATION_MIRROR,   // Vertical orientation mirror
    PATCH_ORIENTATION_MROT180,  // Vertical orientation mrot180
    PATCH_ORIENTATION_ROT270,   // Vertical orientation rot270
    PATCH_ORIENTATION_MROT90,   // Vertical orientation mrot90
    PATCH_ORIENTATION_ROT90     // Vertical orientation rot90
};
const std::vector<int> g_orientationHorizontal = {
    PATCH_ORIENTATION_SWAP,     // Horizontal orientation swap
    PATCH_ORIENTATION_DEFAULT,  // Horizontal orientation default
    PATCH_ORIENTATION_ROT270,   // Horizontal orientation rot270
    PATCH_ORIENTATION_MROT90,   // Horizontal orientation mrot90
    PATCH_ORIENTATION_ROT90,    // Horizontal orientation rot90
    PATCH_ORIENTATION_ROT180,   // Horizontal orientation rot180
    PATCH_ORIENTATION_MIRROR,   // Horizontal orientation mirror
    PATCH_ORIENTATION_MROT180   // Horizontal orientation mrot180
};

// ******************************************************************* //
// Static functions
// ******************************************************************* //
static inline PCCEndianness PCCSystemEndianness() {
  uint32_t num = 1;
  return ( *( reinterpret_cast<char*>( &num ) ) == 1 ) ? PCC_LITTLE_ENDIAN : PCC_BIG_ENDIAN;
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
  printf( "%s: %zu %zu \n", string.c_str(), width, height );
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

static inline std::string getParameter( const std::string& config, const std::string& param ) {
  std::size_t pos = 0;
  if ( ( pos = config.find( param ) ) == std::string::npos ) {
    printf( "Can't find parameter: \"%s\" in: %s \n", param.c_str(), config.c_str() );
    exit( -1 );
  }
  pos += param.length();
  return config.substr( pos, config.find( " ", pos ) - pos );
}

static inline std::string getParameterFromConfigurationFile( const std::string& filename, const std::string& param ) {
  std::string ret = "0";
  std::cout << "getParameterFromConfigurationFile filename = " << filename << '\n';
  std::ifstream file( filename.c_str(), std::ifstream::in );
  if ( file.is_open() ) {
    do {
      std::string line;
      std::getline( file, line );
      // std::cout << "getline = " << line << '\n';
      if ( std::string::npos != line.find( param ) ) {
        if ( std::string::npos != line.find( ":" ) ) {
          ret = line.substr( line.find( ":" ) + 1 );
          std::cout << "Match = " << line << " => ret = " << ret << "\n";
          return ret;
        }
      }
    } while ( !file.eof() );
  }
  return ret;
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
