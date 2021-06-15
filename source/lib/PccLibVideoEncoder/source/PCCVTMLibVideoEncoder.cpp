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

#ifdef USE_VTMLIB_VIDEO_CODEC
#include "PCCVideo.h"
#include "PCCVTMLibVideoEncoder.h"
#include "PCCVTMLibVideoEncoderImpl.h"
#include "PCCVTMLibVideoEncoderCfg.h"
#include "EncoderLib/EncLibCommon.h"

using namespace pcc;

template <typename T>
PCCVTMLibVideoEncoder<T>::PCCVTMLibVideoEncoder() {}
template <typename T>
PCCVTMLibVideoEncoder<T>::~PCCVTMLibVideoEncoder() {}

template <typename T>
void PCCVTMLibVideoEncoder<T>::encode( PCCVideo<T, 3>&            videoSrc,
                                       PCCVideoEncoderParameters& params,
                                       PCCVideoBitstream&         bitstream,
                                       PCCVideo<T, 3>&            videoRec ) {
  const size_t      width      = videoSrc.getWidth();
  const size_t      height     = videoSrc.getHeight();
  const size_t      frameCount = videoSrc.getFrameCount();
  std::stringstream cmd;
  cmd << "VTMEncoder";
  cmd << " -c " << params.encoderConfig_;
  cmd << " --InputFile=" << params.srcYuvFileName_;
  cmd << " --InputBitDepth=" << params.inputBitDepth_;
  cmd << " --InputChromaFormat=" << ( params.use444CodecIo_ ? "444" : "420" );
  cmd << " --OutputBitDepth=" << params.outputBitDepth_;
  cmd << " --OutputBitDepthC=" << params.outputBitDepth_;
  cmd << " --FrameRate=30";
  cmd << " --FrameSkip=0";
  cmd << " --SourceWidth=" << width;
  cmd << " --SourceHeight=" << height;
  cmd << " --ConformanceWindowMode=1 ";
  cmd << " --FramesToBeEncoded=" << frameCount;
  cmd << " --BitstreamFile=" << params.binFileName_;
  cmd << " --ReconFile=" << params.recYuvFileName_;
  cmd << " --QP=" << params.qp_;
  if ( params.internalBitDepth_ != 0 ) { cmd << " --InternalBitDepth=" << params.internalBitDepth_; }
  if ( params.usePccMotionEstimation_ ) {
    cmd << " --UsePccMotionEstimation=1"
        << " --BlockToPatchFile=" << params.blockToPatchFile_ << " --OccupancyMapFile=" << params.occupancyMapFile_
        << " --PatchInfoFile=" << params.patchInfoFile_;
  }
  if ( params.use444CodecIo_ ) { cmd << " --InputColourSpaceConvert=RGBtoGBR"; }
  std::cout << cmd.str() << std::endl;

  std::string arguments = cmd.str();

  fprintf( stdout, "\n" );
  fprintf( stdout, "VVCSoftware: VTM Encoder Version %s ", VTM_VERSION );
  fprintf( stdout, NVM_ONOS );
  fprintf( stdout, NVM_COMPILEDBY );
  fprintf( stdout, NVM_BITS );
  fprintf( stdout, "\n" );

  std::ostringstream oss( ostringstream::binary | ostringstream::out );
  std::ostream&      bitstreamFile = oss;
  EncLibCommon       encLibCommon;

  initROM();
  TComHash::initBlockSizeToIndex();

  PCCVTMLibVideoEncoderImpl<T> encoder( bitstreamFile, &encLibCommon );

  std::istringstream iss( arguments );
  std::string        token;
  std::vector<char*> args;
  while ( iss >> token ) {
    char* arg = new char[token.size() + 1];
    copy( token.begin(), token.end(), arg );
    arg[token.size()] = '\0';
    args.push_back( arg );
  }
  encoder.create();
  // parse configuration
  try {
    if ( !encoder.parseCfg( args.size(), &args[0] ) ) {
      encoder.destroy();
#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
      EnvVar::printEnvVar();
#endif
      return;
    }
  } catch ( df::program_options_lite::ParseFailure& e ) {
    std::cerr << "Error parsing option \"" << e.arg << "\" with argument \"" << e.val << "\"." << std::endl;
    return;
  }
  for ( size_t i = 0; i < args.size(); i++ ) { delete[] args[i]; }

  encoder.createLib( 0 );

  auto        startTime  = std::chrono::steady_clock::now();
  std::time_t startTime2 = std::chrono::system_clock::to_time_t( std::chrono::system_clock::now() );
  fprintf( stdout, " started @ %s", std::ctime( &startTime2 ) );
  clock_t startClock = clock();

  bool eos = false;

  while ( !eos ) {
    // read GOP
    bool keepLoop = true;
    while ( keepLoop ) {
#ifndef _DEBUG
      try {
#endif
        keepLoop = encoder.encodePrep( eos, videoSrc, arguments, videoRec );
#ifndef _DEBUG
      } catch ( Exception& e ) {
        std::cerr << e.what() << std::endl;
        return;
      } catch ( const std::bad_alloc& e ) {
        std::cout << "Memory allocation failed: " << e.what() << std::endl;
        return;
      }
#endif
    }

    // encode GOP
    keepLoop = true;
    while ( keepLoop ) {
#ifndef _DEBUG
      try {
#endif
        keepLoop = encoder.encode( videoSrc, arguments, bitstream, videoRec );
#ifndef _DEBUG
      } catch ( Exception& e ) {
        std::cerr << e.what() << std::endl;
        return;
      } catch ( const std::bad_alloc& e ) {
        std::cout << "Memory allocation failed: " << e.what() << std::endl;
        return;
      }
#endif
    }
  }

  auto buffer = oss.str();
  bitstream.resize( buffer.size() );
  std::copy( buffer.data(), buffer.data() + buffer.size(), bitstream.vector().begin() );

  clock_t     endClock = clock();
  auto        endTime  = std::chrono::steady_clock::now();
  std::time_t endTime2 = std::chrono::system_clock::to_time_t( std::chrono::system_clock::now() );
  auto        encTime  = std::chrono::duration_cast<std::chrono::milliseconds>( endTime - startTime ).count();

  printf( "\n finished @ %s", std::ctime( &endTime2 ) );
  printf( " Total Time: %12.3f sec. [user] %12.3f sec. [elapsed]\n", ( endClock - startClock ) * 1.0 / CLOCKS_PER_SEC,
          encTime / 1000.0 );

  encoder.destroyLib();
  encoder.destroy();
  destroyROM();

  printf( "\n finished @ %s", std::ctime( &endTime2 ) );

#if JVET_O0756_CALCULATE_HDRMETRICS
  printf( " Encoding Time (Total Time): %12.3f ( %12.3f ) sec. [user] %12.3f ( %12.3f ) sec. [elapsed]\n",
          ( ( endClock - startClock ) * 1.0 / CLOCKS_PER_SEC ) - ( metricTimeuser / 1000.0 ),
          ( endClock - startClock ) * 1.0 / CLOCKS_PER_SEC, encTime / 1000.0, totalTime / 1000.0 );
#else
  printf( " Total Time: %12.3f sec. [user] %12.3f sec. [elapsed]\n", ( endClock - startClock ) * 1.0 / CLOCKS_PER_SEC,
          encTime / 1000.0 );
#endif
}

template class pcc::PCCVTMLibVideoEncoder<uint8_t>;
template class pcc::PCCVTMLibVideoEncoder<uint16_t>;

#endif
