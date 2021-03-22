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

/** \file     DecAppCfg.cpp
    \brief    Handle decoder configuration parameters
*/

#include "PCCCommon.h"

#ifdef USE_VTMLIB_VIDEO_CODEC
#include <cstdio>
#include <cstring>
#include <string>
#include "PCCVTMLibVideoDecoderCfg.h"
#include "Utilities/program_options_lite.h"
#include "CommonLib/ChromaFormat.h"
#include "CommonLib/dtrace_next.h"

using namespace std;
namespace po = df::program_options_lite;

//! \ingroup DecoderApp
//! \{

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/** \param argc number of arguments
    \param argv array of arguments
 */
bool PCCVTMLibVideoDecoderCfg::parseCfg( int argc, char* argv[] ) {
  bool   do_help = false;
  string cfg_TargetDecLayerIdSetFile;
  string outputColourSpaceConvert;
  int    warnUnknowParameter = 0;
#if ENABLE_TRACING
  string sTracingRule;
  string sTracingFile;
  bool   bTracingChannelsList = false;
#endif
#if ENABLE_SIMD_OPT
  std::string ignore;
#endif
  po::Options opts;
  opts.addOptions()

      ( "help", do_help, false, "this help text" )( "BitstreamFile,b", m_bitstreamFileName, string( "" ),
                                                    "bitstream input file name" )(
          "ReconFile,o", m_reconFileName, string( "" ), "reconstructed YUV output file name\n" )

          ( "OplFile,-opl", m_oplFilename, string( "" ), "opl-file name without extension for conformance testing\n" )

#if ENABLE_SIMD_OPT
              ( "SIMD", ignore, string( "" ),
                "SIMD extension to use (SCALAR, SSE41, SSE42, AVX, AVX2, AVX512), default: the highest supported "
                "extension\n" )
#endif

                  ( "WarnUnknowParameter,w", warnUnknowParameter, 0,
                    "warn for unknown configuration parameters instead of failing" )(
                      "SkipFrames,s", m_iSkipFrame, 0, "number of frames to skip before random access" )(
                      "OutputBitDepth,d", m_outputBitDepth[CHANNEL_TYPE_LUMA], 0,
                      "bit depth of YUV output luma component (default: use 0 for native depth)" )(
                      "OutputBitDepthC,d", m_outputBitDepth[CHANNEL_TYPE_CHROMA], 0,
                      "bit depth of YUV output chroma component (default: use luma output bit-depth)" )(
                      "OutputColourSpaceConvert", outputColourSpaceConvert, string( "" ),
                      "Colour space conversion to apply to input 444 video. Permitted values are (empty "
                      "string=UNCHANGED) " +
                          getListOfColourSpaceConverts( false ) )(
                      "MaxTemporalLayer,t", m_iMaxTemporalLayer, 500,
                      "Maximum Temporal Layer to be decoded. -1 to decode all layers" )(
                      "TargetOutputLayerSet,p", m_targetOlsIdx, 500, "Target output layer set index" )(
                      "SEIDecodedPictureHash,-dph", m_decodedPictureHashSEIEnabled, 1,
                      "Control handling of decoded picture hash SEI messages\n"
                      "\t1: check hash in SEI messages if available in the bitstream\n"
                      "\t0: ignore SEI message" )( "SEINoDisplay", m_decodedNoDisplaySEIEnabled, true,
                                                   "Control handling of decoded no display SEI messages" )(
                      "TarDecLayerIdSetFile,l", cfg_TargetDecLayerIdSetFile, string( "" ),
                      "targetDecLayerIdSet file name. The file should include white space separated LayerId values to "
                      "be decoded. Omitting the option or a value of -1 in the file decodes all layers." )(
                      "SEIColourRemappingInfoFilename", m_colourRemapSEIFileName, string( "" ),
                      "Colour Remapping YUV output file name. If empty, no remapping is applied (ignore SEI "
                      "message)\n" )( "SEIAnnotatedRegionsInfoFilename", m_annotatedRegionsSEIFileName, string( "" ),
                                      "Annotated regions output file name. If empty, no object information will be "
                                      "saved (ignore SEI message)\n" )(
                      "OutputDecodedSEIMessagesFilename", m_outputDecodedSEIMessagesFilename, string( "" ),
                      "When non empty, output decoded SEI messages to the indicated file. If file is '-', then output "
                      "to stdout\n" )
#if JVET_S0257_DUMP_360SEI_MESSAGE
                      ( "360DumpFile", m_outputDecoded360SEIMessagesFilename, string( "" ),
                        "When non empty, output decoded 360 SEI messages to the indicated file.\n" )
#endif
                          ( "ClipOutputVideoToRec709Range", m_bClipOutputVideoToRec709Range, false,
                            "If true then clip output video to the Rec. 709 Range on saving" )(
                              "PYUV", m_packedYUVMode, false,
                              "If true then output 10-bit and 12-bit YUV data as 5-byte and 3-byte (respectively) "
                              "packed YUV data. Ignored for interlaced output." )
#if ENABLE_TRACING
                              ( "TraceChannelsList", bTracingChannelsList, false,
                                "List all available tracing channels" )(
                                  "TraceRule", sTracingRule, string( "" ),
                                  "Tracing rule (ex: \"D_CABAC:poc==8\" or \"D_REC_CB_LUMA:poc==8\")" )(
                                  "TraceFile", sTracingFile, string( "" ), "Tracing file" )
#endif
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
                                  ( "CacheCfg", m_cacheCfgFile, string( "" ), "CacheCfg File" )
#endif
#if RExt__DECODER_DEBUG_STATISTICS
                                      ( "Stats", m_statMode, 3,
                                        "Control decoder debugging statistic output mode\n"
                                        "\t0: disable statistic\n"
                                        "\t1: enable bit statistic\n"
                                        "\t2: enable tool statistic\n"
                                        "\t3: enable bit and tool statistic\n" )
#endif
                                          ( "MCTSCheck", m_mctsCheck, false,
                                            "If enabled, the decoder checks for violations of "
                                            "mc_exact_sample_value_match_flag in Temporal MCTS " )(
                                              "targetSubPicIdx", m_targetSubPicIdx, 0,
                                              "Specify which subpicture shall be written to output, using subpic "
                                              "index, 0: disabled, subpicIdx=m_targetSubPicIdx-1 \n" )(
                                              "UpscaledOutput", m_upscaledOutput, 0, "Upscaled output for RPR" );

  po::setDefaults( opts );
  po::ErrorReporter        err;
  const list<const char*>& argv_unhandled = po::scanArgv( opts, argc, (const char**)argv, err );

  for ( list<const char*>::const_iterator it = argv_unhandled.begin(); it != argv_unhandled.end(); it++ ) {
    msg( VTM_ERROR, "Unhandled argument ignored: `%s'\n", *it );
  }

  if ( argc == 1 || do_help ) {
    po::doHelp( cout, opts );
    return false;
  }

  if ( err.is_errored ) {
    if ( !warnUnknowParameter ) {
      /* errors have already been reported to stderr */
      return false;
    }
  }

#if ENABLE_TRACING
  g_trace_ctx = tracing_init( sTracingFile, sTracingRule );
  if ( bTracingChannelsList && g_trace_ctx ) {
    std::string sChannelsList;
    g_trace_ctx->getChannelsList( sChannelsList );
    msg( INFO, "\nAvailable tracing channels:\n\n%s\n", sChannelsList.c_str() );
  }
#endif

  g_mctsDecCheckEnabled = m_mctsCheck;
  // Chroma output bit-depth
  if ( m_outputBitDepth[CHANNEL_TYPE_LUMA] != 0 && m_outputBitDepth[CHANNEL_TYPE_CHROMA] == 0 ) {
    m_outputBitDepth[CHANNEL_TYPE_CHROMA] = m_outputBitDepth[CHANNEL_TYPE_LUMA];
  }

  m_outputColourSpaceConvert = stringToInputColourSpaceConvert( outputColourSpaceConvert, false );
  if ( m_outputColourSpaceConvert >= NUMBER_INPUT_COLOUR_SPACE_CONVERSIONS ) {
    msg( VTM_ERROR, "Bad output colour space conversion string\n" );
    return false;
  }

  if ( m_bitstreamFileName.empty() ) {
    msg( VTM_ERROR, "No input file specified, aborting\n" );
    return false;
  }

  if ( !cfg_TargetDecLayerIdSetFile.empty() ) {
    FILE* targetDecLayerIdSetFile = fopen( cfg_TargetDecLayerIdSetFile.c_str(), "r" );
    if ( targetDecLayerIdSetFile ) {
      bool isLayerIdZeroIncluded = false;
      while ( !feof( targetDecLayerIdSetFile ) ) {
        int layerIdParsed = 0;
        if ( fscanf( targetDecLayerIdSetFile, "%d ", &layerIdParsed ) != 1 ) {
          if ( m_targetDecLayerIdSet.size() == 0 ) {
            msg( VTM_ERROR, "No LayerId could be parsed in file %s. Decoding all LayerIds as default.\n",
                 cfg_TargetDecLayerIdSetFile.c_str() );
          }
          break;
        }
        if ( layerIdParsed == -1 )  // The file includes a -1, which means all LayerIds are to be decoded.
        {
          m_targetDecLayerIdSet.clear();  // Empty set means decoding all layers.
          break;
        }
        if ( layerIdParsed < 0 || layerIdParsed >= MAX_NUM_LAYER_IDS ) {
          msg( VTM_ERROR, "Warning! Parsed LayerId %d is not within allowed range [0,%d]. Ignoring this value.\n",
               layerIdParsed, MAX_NUM_LAYER_IDS - 1 );
        } else {
          isLayerIdZeroIncluded = layerIdParsed == 0 ? true : isLayerIdZeroIncluded;
          m_targetDecLayerIdSet.push_back( layerIdParsed );
        }
      }
      fclose( targetDecLayerIdSetFile );
      if ( m_targetDecLayerIdSet.size() > 0 && !isLayerIdZeroIncluded ) {
        msg( VTM_ERROR, "TargetDecLayerIdSet must contain LayerId=0, aborting" );
        return false;
      }
    } else {
      msg( VTM_ERROR, "File %s could not be opened. Using all LayerIds as default.\n",
           cfg_TargetDecLayerIdSetFile.c_str() );
    }
  }
  if ( m_iMaxTemporalLayer != 500 ) {
    m_mTidExternalSet = true;
  } else {
    m_iMaxTemporalLayer = -1;
  }
  if ( m_targetOlsIdx != 500 ) {
    m_tOlsIdxTidExternalSet = true;
  } else {
    m_targetOlsIdx = -1;
  }
  return true;
}

PCCVTMLibVideoDecoderCfg::PCCVTMLibVideoDecoderCfg() :
    m_bitstreamFileName(),
    m_reconFileName(),
    m_oplFilename()

    ,
    m_iSkipFrame( 0 )
    // m_outputBitDepth array initialised below
    ,
    m_outputColourSpaceConvert( IPCOLOURSPACE_UNCHANGED ),
    m_targetOlsIdx( 0 ),
    m_iMaxTemporalLayer( -1 ),
    m_mTidExternalSet( false ),
    m_tOlsIdxTidExternalSet( false ),
    m_decodedPictureHashSEIEnabled( 0 ),
    m_decodedNoDisplaySEIEnabled( false ),
    m_colourRemapSEIFileName(),
    m_annotatedRegionsSEIFileName(),
    m_targetDecLayerIdSet(),
    m_outputDecodedSEIMessagesFilename()
#if JVET_S0257_DUMP_360SEI_MESSAGE
    ,
    m_outputDecoded360SEIMessagesFilename()
#endif
    ,
    m_bClipOutputVideoToRec709Range( false ),
    m_packedYUVMode( false ),
    m_statMode( 0 ),
    m_mctsCheck( false ) {
  for ( uint32_t channelTypeIndex = 0; channelTypeIndex < MAX_NUM_CHANNEL_TYPE; channelTypeIndex++ ) {
    m_outputBitDepth[channelTypeIndex] = 0;
  }
}

PCCVTMLibVideoDecoderCfg::~PCCVTMLibVideoDecoderCfg() {
#if ENABLE_TRACING
  tracing_uninit( g_trace_ctx );
#endif
}

//! \}
#endif
