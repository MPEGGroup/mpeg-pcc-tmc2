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
#include "PCCVideo.h"
#include "program_options_lite.h"
#include "PCCVirtualVideoEncoder.h"

using namespace std;
using namespace pcc;

namespace pcc {
static std::istream& operator>>( std::istream& in, PCCCodecId& val ) {
  unsigned int tmp;
  in >> tmp;
  val = PCCCodecId( tmp );
  return in;
}
}  // namespace pcc

bool parseParameters( int          argc,
                      char*        argv[],
                      std::string& srcVideo,
                      std::string& encoderParameters,
                      size_t&      width,
                      size_t&      height,
                      size_t&      nbyte,
                      PCCCodecId&  codecId ) {
  namespace po    = df::program_options_lite;
  bool print_help = false;
  // clang-format off
  po::Options opts;
  opts.addOptions()
  ( "help,h",              print_help,        false,            "This help text" )
  ( "srcVideo,i",          srcVideo,          srcVideo,         "Source yuv file" )
  ( "codecId,c",           codecId,           codecId,          "Codec Id" )
  ( "width",               width,             width,            "Source video width" )
  ( "height",              height,            height,           "Source video height" )
  ( "nbyte,b",             nbyte,             nbyte,            "Source video nbyte" )
  ( "encoderParameters,p", encoderParameters, encoderParameters,"Encoder parameters" ) ;
  // clang-format on
  po::setDefaults( opts );
  po::ErrorReporter        err;
  const list<const char*>& argv_unhandled = po::scanArgv( opts, argc, (const char**)argv, err );
  for ( const auto arg : argv_unhandled ) {
    printf( "Unhandled argument ignored: %s \n", arg );
    fflush( stdout );
  }
  if ( argc == 1 || print_help ) {
    po::doHelp( std::cout, opts, 78 );
    return false;
  }
  printf( "parseParameters : \n" );
  printf( "  srcVideo          = %s \n", srcVideo.c_str() );
  printf( "  encoderParameters = %s \n", encoderParameters.c_str() );
  printf( "  codecId           = %zu \n", size_t( codecId ) );
  printf( "  width             = %zu \n", width );
  printf( "  height            = %zu \n", height );
  printf( "  nbyte             = %zu \n", nbyte );
  if ( argc == 1 || print_help || srcVideo.empty() || encoderParameters.empty() || width == 0 || height == 0 ||
           nbyte == 0,
       codecId == PCCCodecId::UNKNOWN_CODEC ) {
    printf( "Error parameters not correct \n" );
    return false;
  }

  // report the current configuration (only in the absence of errors so
  // that errors/warnings are more obvious and in the same place).
  if ( err.is_errored ) { return false; }
  return true;
}

int main( int argc, char* argv[] ) {
  std::cout << "PccAppVideoEncoder v" << TMC2_VERSION_MAJOR << "." << TMC2_VERSION_MINOR << std::endl << std::endl;
  std::string srcVideo, encoderParameters;
  size_t      width = 0, height = 0, nbyte = 0;
  PCCCodecId  codecId = PCCCodecId::UNKNOWN_CODEC;
  
  if ( !parseParameters( argc, argv, srcVideo, encoderParameters, width, height, nbyte, codecId ) ) {
    printf("parseParameters error \n");
    fflush(stdout);
    return -1; 
  }

  typedef uint16_t T;
  PCCVideo<T, 3> videoSrc, videoRec;
  auto encoder = PCCVirtualVideoEncoder<T>::create( codecId );
  videoSrc.read( srcVideo, width, height, PCCCOLORFORMAT::YUV420, nbyte );

  PCCVideoBitstream bitstream( VIDEO_OCCUPANCY );
  PCCVideoEncoderParameters params;
  params.encoderPath_             = "EncoderPath";
  params.srcYuvFileName_          = srcVideo;
  params.binFileName_             = removeFileExtension( srcVideo ) + ".bin";
  params.recYuvFileName_          = removeFileExtension( srcVideo ) + "_rec.yuv";
  params.encoderConfig_           = encoderParameters;
  params.qp_                      = 0;
  params.inputBitDepth_           = 8;
  params.internalBitDepth_        = 10;
  params.outputBitDepth_          = 10;
  params.use444CodecIo_           = false;
  params.usePccMotionEstimation_  = false;
  params.inputColourSpaceConvert_ = false;
  params.usePccRDO_               = false;

  encoder->encode( videoSrc, params, bitstream, videoRec );
  bitstream.write( removeFileExtension( srcVideo ) + ".bin" );
  videoRec.write( removeFileExtension( srcVideo ) + "_rec.yuv", params.outputBitDepth_ == 8 ? 1 : 2 );

  return 0;
}
