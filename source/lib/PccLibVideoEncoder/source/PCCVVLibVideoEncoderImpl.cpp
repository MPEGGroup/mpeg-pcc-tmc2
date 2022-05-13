/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
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

#include "PCCCommon.h"
#include "PCCLogger.h"

#ifdef USE_VVLIB_VIDEO_CODEC

#include <iostream>
#include <list>
#include <ostream>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <assert.h>
#include <iomanip>
#include <string>
#include <cstring>
#include <algorithm>
#include <cstdarg>

#include "PCCVVLibVideoEncoderImpl.h"
#include "vvenc/version.h"
#include "vvenc/vvenc.h"
#include "apputils/ParseArg.h"
#include "apputils/YuvFileIO.h"
#include "apputils/VVEncAppCfg.h"

using namespace pcc;

vvencMsgLevel g_verbosityEnc = VVENC_VERBOSE;

struct StoreFrame {
  void* m_videoRec;
  void* m_config;
};

void msgFnc( void*, int level, const char* fmt, va_list args ) {
  if ( g_verbosityEnc >= level ) { vfprintf( level == 1 ? stderr : stdout, fmt, args ); }
}

void msgApp( int level, const char* fmt, ... ) {
  va_list args;
  va_start( args, fmt );
  msgFnc( nullptr, level, fmt, args );
  va_end( args );
}

template <typename T>
PCCVVLibVideoEncoderImpl<T>::PCCVVLibVideoEncoderImpl() :
    m_essentialBytes( 0 ), m_totalBytes( 0 ) {
  vvenc_config_default( &m_vvenc_config );
  m_cEncAppCfg.setPresetChangeCallback( changePreset );
}

template <typename T>
PCCVVLibVideoEncoderImpl<T>::~PCCVVLibVideoEncoderImpl() {}

template <typename T>
void PCCVVLibVideoEncoderImpl<T>::encode( PCCVideo<T, 3>&    videoSrc,
                                          std::string        arguments,
                                          PCCVideoBitstream& bitstream,
                                          PCCVideo<T, 3>&    videoRec ) {
  std::ostringstream oss( std::ostringstream::binary | std::ostringstream::out );
  std::ostream&      cOutBitstream = oss;
  std::istringstream iss( arguments );
  std::string        token;
  std::vector<char*> args;
  while ( iss >> token ) {
    char* arg = new char[token.size() + 1];
    copy( token.begin(), token.end(), arg );
    arg[token.size()] = '\0';
    args.push_back( arg );
  }
  videoRec.clear();
  vvenc_set_logging_callback( nullptr, msgFnc ); 
  std::string                                 simdOpt;
  apputils::df::program_options_lite::Options opts;
  opts.addOptions()                                                     //
      ( "c", apputils::df::program_options_lite::parseConfigFile, "" )  //
      ( "SIMD", simdOpt, "" );

  apputils::df::program_options_lite::SilentReporter err;
  apputils::df::program_options_lite::scanArgv( opts, args.size(), (const char**)&args[0], err );
  vvenc_set_SIMD_extension( simdOpt.c_str() );

  if ( !parseCfg( args.size(), &args[0] ) ) { return; }
  int ret = encode( videoSrc, cOutBitstream, videoRec );
  auto buffer = oss.str();
  bitstream.resize( buffer.size() );
  std::copy( buffer.data(), buffer.data() + buffer.size(), bitstream.vector().begin() );
}

template <typename T>
bool PCCVVLibVideoEncoderImpl<T>::parseCfg( int argc, char* argv[] ) {
  vvenc_set_msg_callback( &m_vvenc_config, this, &::msgFnc );  
  std::stringstream cParserStr;
  bool ret = true;
  if ( argc ) {
    argc--;
    argv++;
  }
  int parserRes = m_cEncAppCfg.parse( argc, argv, &m_vvenc_config, cParserStr );
  if ( parserRes != 0 ) {
    if ( m_cEncAppCfg.m_showHelp ) {
      msgApp( VVENC_INFO, "vvencFFapp: %s\n", vvenc_get_enc_information( nullptr ) );
      if ( !cParserStr.str().empty() ) msgApp( VVENC_INFO, "%s", cParserStr.str().c_str() );
      return true;
    } else if ( m_cEncAppCfg.m_showVersion ) {
      msgApp( VVENC_INFO, "vvencFFapp version %s\n", vvenc_get_version() );
      if ( !cParserStr.str().empty() ) msgApp( VVENC_INFO, "%s", cParserStr.str().c_str() );
      return true;
    }
  }
  if ( parserRes >= 0 )
    g_verbosityEnc = m_vvenc_config.m_verbosity;
  else
    ret = false;

  msgApp( VVENC_INFO, "vvencFFapp: %s\n", vvenc_get_enc_information( nullptr ) );
  if ( !cParserStr.str().empty() )
    msgApp( ( parserRes < 0 ) ? VVENC_ERROR : ( ( parserRes > 0 ) ? VVENC_WARNING : VVENC_INFO ), "%s",
            cParserStr.str().c_str() );
  if ( !m_cEncAppCfg.m_additionalSettings.empty() ) {
    std::vector<std::tuple<std::string, std::string>> dict = m_cEncAppCfg.getAdditionalSettingList();
    if ( dict.empty() ) {
      msgApp( VVENC_ERROR, "Error parsing additional option string \"%s\"\n",
              m_cEncAppCfg.m_additionalSettings.c_str() );
      return false;
    }
    for ( auto& d : dict ) {
      std::string key   = std::get<0>( d );
      std::string value = std::get<1>( d );
      msgApp( VVENC_DETAILS, "additional params: set option key:'%s' value:'%s'\n", key.c_str(), value.c_str() );
      int parse_ret = vvenc_set_param( &m_vvenc_config, key.c_str(), value.c_str() );
      switch ( parse_ret ) {
        case VVENC_PARAM_BAD_NAME:
          msgApp( VVENC_ERROR, "additional params: unknown option \"%s\" \n", key.c_str() );
          ret = false;
          break;
        case VVENC_PARAM_BAD_VALUE:
          msgApp( VVENC_ERROR, "additional params: invalid value for key \"%s\": \"%s\" \n", key.c_str(),
                  value.c_str() );
          ret = false;
          break;
        default: break;
      }
    }
  }
  if ( m_vvenc_config.m_internChromaFormat < 0 || m_vvenc_config.m_internChromaFormat >= VVENC_NUM_CHROMA_FORMAT ) {
    m_vvenc_config.m_internChromaFormat = m_cEncAppCfg.m_inputFileChromaFormat;
  }
  if ( m_vvenc_config.m_RCNumPasses < 0 && ( m_vvenc_config.m_RCPass > 0 || m_vvenc_config.m_RCTargetBitrate > 0 ) ) {
    m_vvenc_config.m_RCNumPasses = 2;
  }
  if ( m_cEncAppCfg.m_decode ) { return ret; }
  if ( vvenc_init_config_parameter( &m_vvenc_config ) ) { ret = false; }
  cParserStr.str( "" );
  cParserStr.clear();
  if ( m_cEncAppCfg.checkCfg( &m_vvenc_config, cParserStr ) ) {
    msgApp( VVENC_ERROR, "%s", cParserStr.str().c_str() );
    ret = false;
  }
  return ret;
}

template <typename T>
void PCCVVLibVideoEncoderImpl<T>::outputAU( const vvencAccessUnit& au, std::ostream& bitstream ) {
  bitstream.write( reinterpret_cast<const char*>( au.payload ), au.payloadUsedSize );
  m_totalBytes += au.payloadUsedSize;
  m_essentialBytes += au.essentialBytes;
}

template <typename T>
void PCCVVLibVideoEncoderImpl<T>::outputYuv( void* ctx, vvencYUVBuffer* yuvOutBuf ) {
  StoreFrame* pStoreFrame = ctx ? (StoreFrame*)ctx : nullptr;
  if ( pStoreFrame ) {
    PCCVideo<T, 3>* pVideo  = (PCCVideo<T, 3>*)pStoreFrame->m_videoRec;
    vvenc_config*   pConfig = (vvenc_config*)pStoreFrame->m_config;
    storeRecFrame( yuvOutBuf, *pConfig, *pVideo );
  }
}

template <typename T>
int PCCVVLibVideoEncoderImpl<T>::encode( PCCVideo<T, 3>& videoSrc, std::ostream& bitstream, PCCVideo<T, 3>& videoRec ) {
  apputils::VVEncAppCfg& appCfg   = m_cEncAppCfg;
  vvenc_config&          vvencCfg = m_vvenc_config;

  // initialize encoder lib
  m_encCtx = vvenc_encoder_create();
  if ( nullptr == m_encCtx ) { return -1; }
  int iRet = vvenc_encoder_open( m_encCtx, &vvencCfg );
  if ( 0 != iRet ) {
    msgApp( VVENC_ERROR, "open encoder failed: %s\n", vvenc_get_last_error( m_encCtx ) );
    vvenc_encoder_close( m_encCtx );
    return iRet;
  }

  // get the adapted config
  vvenc_get_config( m_encCtx, &vvencCfg );
  std::stringstream css;
  css << appCfg.getAppConfigAsString( vvencCfg.m_verbosity );
  css << vvenc_get_config_as_string( &vvencCfg, vvencCfg.m_verbosity );
  css << std::endl;
  msgApp( VVENC_INFO, "%s", css.str().c_str() );
  printChromaFormat();
  if ( !strcmp( appCfg.m_inputFileName.c_str(), "-" ) ) { msgApp( VVENC_INFO, " trying to read from stdin" ); }
  StoreFrame storeFrame = { (void*)&videoRec, (void*)&vvencCfg };
  vvenc_encoder_set_RecYUVBufferCallback( m_encCtx, &storeFrame, outputYuv );

  // create buffer for input YUV pic
  vvencYUVBuffer yuvInBuf;
  vvenc_YUVBuffer_default( &yuvInBuf );
  vvenc_YUVBuffer_alloc_buffer( &yuvInBuf, vvencCfg.m_internChromaFormat, vvencCfg.m_SourceWidth,
                                vvencCfg.m_SourceHeight );

  // create sufficient memory for output data
  vvencAccessUnit au;
  vvenc_accessUnit_default( &au );
  const int auSizeScale = vvencCfg.m_internChromaFormat <= VVENC_CHROMA_420 ? 2 : 3;
  vvenc_accessUnit_alloc_payload( &au, auSizeScale * vvencCfg.m_SourceWidth * vvencCfg.m_SourceHeight + 1024 );

  // main loop
  int       framesRcvd = 0;
  const int start      = vvencCfg.m_RCPass > 0 ? vvencCfg.m_RCPass - 1 : 0;
  const int end        = vvencCfg.m_RCPass > 0 ? vvencCfg.m_RCPass : vvencCfg.m_RCNumPasses;
  for ( int pass = start; pass < end; pass++ ) {
    const int remSkipFrames = appCfg.m_FrameSkip - vvencCfg.m_leadFrames;
    if ( remSkipFrames < 0 ) {
      msgApp( VVENC_ERROR, "skip frames (%d) less than number of lead frames required (%d)\n", appCfg.m_FrameSkip,
              vvencCfg.m_leadFrames );
      vvenc_encoder_close( m_encCtx );
      vvenc_YUVBuffer_free_buffer( &yuvInBuf );
      vvenc_accessUnit_free_payload( &au );
      return -1;
    }
    int uiFrames = 0;
    if ( remSkipFrames > 0 ) { uiFrames = remSkipFrames; }

    // initialize encoder pass
    iRet = vvenc_init_pass( m_encCtx, pass, appCfg.m_RCStatsFileName.c_str() );
    if ( iRet != 0 ) {
      msgApp( VVENC_ERROR, "init pass failed: %s\n", vvenc_get_last_error( m_encCtx ) );
      vvenc_encoder_close( m_encCtx );
      vvenc_YUVBuffer_free_buffer( &yuvInBuf );
      vvenc_accessUnit_free_payload( &au );
      return -1;
    }
    // loop over input YUV data
    bool inputDone = false;
    bool encDone   = false;
    framesRcvd     = 0;
    while ( !inputDone || !encDone ) {
      // check for more input pictures
      inputDone = ( vvencCfg.m_framesToBeEncoded > 0 &&
                    framesRcvd >= ( vvencCfg.m_framesToBeEncoded + vvencCfg.m_leadFrames + vvencCfg.m_trailFrames ) ) ||
                  videoSrc.size() == uiFrames;
      // Get src YUV
      if ( !inputDone ) {
        if ( 0 != getSrcFrame( videoSrc, yuvInBuf, uiFrames ) ) {
          msgApp( VVENC_ERROR, "read input file failed. \n" );
          vvenc_encoder_close( m_encCtx );
          vvenc_YUVBuffer_free_buffer( &yuvInBuf );
          vvenc_accessUnit_free_payload( &au );
          return -1;
        }
        uiFrames++;
        if ( !inputDone ) {
          if ( vvencCfg.m_FrameRate > 0 ) {
            yuvInBuf.cts      = framesRcvd * vvencCfg.m_TicksPerSecond * vvencCfg.m_FrameScale / vvencCfg.m_FrameRate;
            yuvInBuf.ctsValid = true;
          }
          framesRcvd += 1;
        }
      }
      // encode picture
      vvencYUVBuffer* inputPacket = nullptr;
      if ( !inputDone ) { inputPacket = &yuvInBuf; }
      iRet = vvenc_encode( m_encCtx, inputPacket, &au, &encDone );
      if ( 0 != iRet ) {
        msgApp( VVENC_ERROR, "encoding failed: err code %d: %s\n", iRet, vvenc_get_last_error( m_encCtx ) );
        encDone   = true;
        inputDone = true;
      }
      // write out encoded access units
      if ( au.payloadUsedSize ) { outputAU( au, bitstream ); }
      // temporally skip frames
      if ( vvencCfg.m_temporalSubsampleRatio > 1 && !inputDone ) { uiFrames += vvencCfg.m_temporalSubsampleRatio - 1; }
    }
  }
  printRateSummary( framesRcvd - ( vvencCfg.m_leadFrames + vvencCfg.m_trailFrames ) );
  // cleanup encoder lib
  vvenc_encoder_close( m_encCtx );
  vvenc_YUVBuffer_free_buffer( &yuvInBuf );
  vvenc_accessUnit_free_payload( &au );
  return iRet;
}

template <typename T>
void PCCVVLibVideoEncoderImpl<T>::printRateSummary( int framesRcvd ) {
  vvenc_print_summary( m_encCtx );
  int    fps  = m_vvenc_config.m_FrameRate / m_vvenc_config.m_FrameScale;
  double time = (double)framesRcvd / fps * m_vvenc_config.m_temporalSubsampleRatio;
  msgApp( VVENC_DETAILS, "Bytes written to file: %u (%.3f kbps)\n", m_totalBytes, 0.008 * m_totalBytes / time );
  if ( m_vvenc_config.m_summaryVerboseness > 0 ) {
    msgApp( VVENC_DETAILS, "Bytes for SPS/PPS/APS/Slice (Incl. Annex B): %u (%.3f kbps)\n", m_essentialBytes,
            0.008 * m_essentialBytes / time );
  }
}

template <typename T>
void PCCVVLibVideoEncoderImpl<T>::printChromaFormat() {
  if ( m_vvenc_config.m_verbosity >= VVENC_DETAILS ) {
    std::stringstream ssOut;
    ssOut << std::setw( 43 ) << "Input ChromaFormat = ";
    switch ( m_cEncAppCfg.m_inputFileChromaFormat ) {
      case VVENC_CHROMA_400: ssOut << "  YUV 400"; break;
      case VVENC_CHROMA_420: ssOut << "  420"; break;
      case VVENC_CHROMA_422: ssOut << "  422"; break;
      case VVENC_CHROMA_444: ssOut << "  444"; break;
      default: msgApp( VVENC_ERROR, "invalid chroma format\n" ); return;
    }
    ssOut << std::endl;
    ssOut << std::setw( 43 ) << "Output (intern) ChromaFormat = ";
    switch ( m_vvenc_config.m_internChromaFormat ) {
      case VVENC_CHROMA_400: ssOut << "  400"; break;
      case VVENC_CHROMA_420: ssOut << "  420"; break;
      case VVENC_CHROMA_422: ssOut << "  422"; break;
      case VVENC_CHROMA_444: ssOut << "  444"; break;
      default: msgApp( VVENC_ERROR, "invalid chroma format\n" ); return;
    }
    msgApp( VVENC_DETAILS, "%s\n", ssOut.str().c_str() );
  }
}

template <typename T>
int PCCVVLibVideoEncoderImpl<T>::getSrcFrame( PCCVideo<T, 3>& videoSrc, vvencYUVBuffer& pic, int frameIndex ) {
  if ( frameIndex < videoSrc.size() ) {
    auto& image  = videoSrc.getFrame( frameIndex );
    pic.cts      = 0;
    pic.ctsValid = false;
    // printf( "Read frame %3d: m_internalBitDepth = %d m_outputBitDepth = %d size(T) = %zu \n", frameIndex,
    //         m_vvenc_config.m_internalBitDepth[0], m_vvenc_config.m_outputBitDepth[0], sizeof( T ) );
    // image.trace();
    image.get( pic.planes[0].ptr,                                                         // Y ptr
               pic.planes[1].ptr,                                                         // U ptr
               pic.planes[2].ptr,                                                         // V ptr
               pic.planes[0].width,                                                       // Y width
               pic.planes[0].height,                                                      // Y height
               pic.planes[0].stride,                                                      // Y stride
               pic.planes[1].width,                                                       // UV width
               pic.planes[1].height,                                                      // UV height
               pic.planes[1].stride,                                                      // UV stride
               m_vvenc_config.m_internalBitDepth[0] - m_vvenc_config.m_inputBitDepth[0],  // bit shift
               false );                                                                   // RGB=>GBR conversion
    return 0;
  }
  return 1;
}

template <typename T>
int PCCVVLibVideoEncoderImpl<T>::storeRecFrame( const vvencYUVBuffer* pcFrame, vvenc_config& config, PCCVideo<T, 3>& videoRec ) {
  int chromaSubsample = pcFrame->planes[0].width == pcFrame->planes[1].width ? 1 : 2;
  // printf(
  //     "storeRecFrame: write output frame %3zu size = %d %d %d / %d %d %d bit depth = %d - %d = %d in %zu "
  //     "bytes buffers ( ChromaSub. = %d )  \n",
  //     videoRec.getFrameCount(),                                                        //
  //     pcFrame->planes[0].width, pcFrame->planes[0].height, pcFrame->planes[0].stride,  //
  //     pcFrame->planes[1].width, pcFrame->planes[1].height, pcFrame->planes[1].stride,  //
  //     config.m_internalBitDepth[0], config.m_outputBitDepth[0],                        //
  //     config.m_internalBitDepth[0] - config.m_outputBitDepth[0],                       //
  //     sizeof( T ), chromaSubsample );
  // fflush( stdout );
  videoRec.resize( videoRec.getFrameCount() + 1 );
  auto&          image  = videoRec.getFrames().back();
  PCCCOLORFORMAT format = chromaSubsample > 1 ? PCCCOLORFORMAT::YUV420 : PCCCOLORFORMAT::RGB444;
  image.set( pcFrame->planes[0].ptr,                                     // Y ptr
             pcFrame->planes[1].ptr,                                     // U ptr
             pcFrame->planes[2].ptr,                                     // V ptr
             pcFrame->planes[0].width,                                   // widthY
             pcFrame->planes[0].height,                                  // heightY
             pcFrame->planes[0].stride,                                  // strideY
             pcFrame->planes[1].width,                                   // widthC
             pcFrame->planes[1].height,                                  // heightC
             pcFrame->planes[1].stride,                                  // strideC
             config.m_internalBitDepth[0] - config.m_outputBitDepth[0],  // shiftbits
             format,                                                     // format
             format == PCCCOLORFORMAT::RGB444 );                         // rgb2bgr
  // image.trace();
  return 0;
}

template class pcc::PCCVVLibVideoEncoderImpl<uint8_t>;
template class pcc::PCCVVLibVideoEncoderImpl<uint16_t>;

#endif
