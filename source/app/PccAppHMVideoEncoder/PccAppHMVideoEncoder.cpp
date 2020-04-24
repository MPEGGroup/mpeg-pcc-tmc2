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
#include "PCCCommon.h"
#include "PCCVideo.h"

#include "program_options_lite.h"

#include "PCCVideoEncoder.h"

using namespace std;
using namespace pcc;

bool parseParameters( int          argc,
                      char*        argv[],
                      std::string& srcVideo,
                      std::string& encoderParameters,
                      size_t&      width,
                      size_t&      height,
                      size_t&      frameCount,
                      size_t&      nbyte ) {
  namespace po    = df::program_options_lite;
  bool print_help = false;
  // clang-format off
  po::Options opts;
  opts.addOptions()
  ("help", print_help, false, "This help text")
  ("srcVideo", srcVideo, srcVideo, "Source yuv file")
  ("width", width, width, "Source video width")
  ("height", height, height,"Source video height")
  ("frameCount", frameCount, frameCount, "Source video frame count")
  ("nbyte", nbyte, nbyte, "Source video nbyte")
  ("encoderParameters", encoderParameters, encoderParameters,"Hm encoder parameters");
  // clang-format on
  po::setDefaults( opts );
  po::ErrorReporter        err;
  const list<const char*>& argv_unhandled = po::scanArgv( opts, argc, (const char**)argv, err );

  printf( "parseParameters : \n" );
  printf( "  srcVideo          = %s \n", srcVideo.c_str() );
  printf( "  encoderParameters = %s \n", encoderParameters.c_str() );
  printf( "  width             = %zu \n", width );
  printf( "  height            = %zu \n", height );
  printf( "  frameCount        = %zu \n", frameCount );
  printf( "  nbyte             = %zu \n", nbyte );
  if ( argc == 1 || print_help || srcVideo.empty() || encoderParameters.empty() || width == 0 || height == 0 ||
       frameCount == 0 || nbyte == 0 ) {
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
  size_t      width = 0, height = 0, frameCount = 0, nbyte = 0;
  if ( !parseParameters( argc, argv, srcVideo, encoderParameters, width, height, frameCount, nbyte ) ) { return -1; }
  PCCVideo<uint8_t, 3> videoSrc, videoRec;
  videoSrc.read( srcVideo, width, height, PCCCOLORFORMAT::YUV420, frameCount, nbyte );
  std::string       enc = "TAppEncoderHighBitDepthStatic";
  PCCVideoBitstream bitstream( VIDEO_OCCUPANCY );
  PCCContext        context;
  PCCVideoEncoder   encoder;
  encoder.compress( videoSrc, std::string( "" ), 8, bitstream, encoderParameters, enc, context, 1, false, false, 8,
                    false, true );
  videoSrc.swap( videoRec );
  bitstream.write( removeFileExtension( srcVideo ) + ".bin" );
  videoRec.write( removeFileExtension( srcVideo ) + "_rec.yuv", nbyte );
  return 1;
}