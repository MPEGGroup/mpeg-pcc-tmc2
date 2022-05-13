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
#include "PCCVideoBitstream.h"
#include "PCCVirtualVideoDecoder.h"
#include <program_options_lite.h>

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

int main( int argc, char* argv[] ) {
  std::cout << "PccAppVideoDecoder v" << TMC2_VERSION_MAJOR << "." << TMC2_VERSION_MINOR << std::endl << std::endl;
  std::string binFile, path;
  size_t      width = 0, height = 0, nbyte = 0;
  PCCCodecId  codecId    = PCCCodecId::UNKNOWN_CODEC;
  bool        print_help = false;
  namespace po           = df::program_options_lite;
  // clang-format off
  po::Options opts;
  opts.addOptions()
  ( "help,h",    print_help, false,   "This help text" )
  ( "bin,b",     binFile,    binFile, "Video bin file" )
  ( "codecId,c", codecId,    codecId, "Codec Id" )
  ( "width",     width,      width,   "Video width" )
  ( "height",    height,     height,  "Video height" )
  ( "nbyte,n",   nbyte,      nbyte,   "Video nbyte" )
  ( "path,p",    path,       path,    "HM decoder path") ;

  // clang-format on
  po::setDefaults( opts );
  po::ErrorReporter        err;
  const list<const char*>& argv_unhandled = po::scanArgv( opts, argc, (const char**)argv, err );
  for ( const auto arg : argv_unhandled ) { printf( "Unhandled argument ignored: %s \n", arg ); }
  if ( argc == 1 || print_help ) {
    po::doHelp( std::cout, opts, 78 );
    return -1;
  }
  printf( "Parameters : \n" );
  printf( "  binFile = %s \n", binFile.c_str() );
  printf( "  codecId = %zu \n", size_t( codecId ) );
  printf( "  width   = %zu \n", width );
  printf( "  height  = %zu \n", height );
  printf( "  nbyte   = %zu \n", nbyte );
  printf( "  path    = %s \n", path.c_str() );
  if ( argc == 1 || print_help || binFile.empty() || path.empty() || width == 0 || height == 0 || nbyte == 0 ||
       codecId == PCCCodecId::UNKNOWN_CODEC ) {
    printf( "Error parameters not correct \n" );
    return -1;
  }
  PCCVideoBitstream bitstream( VIDEO_ATTRIBUTE );
  bitstream.read( binFile );
  PCCVideo<uint16_t, 3> video;
  auto                  decoder = PCCVirtualVideoDecoder<uint16_t>::create( codecId );
  decoder->decode( bitstream, video, nbyte == 1 ? 8 : 10, path, removeFileExtension( binFile ) );
  video[0].trace();
  video.write( removeFileExtension( binFile ) + "_dec.yuv", nbyte );
  return 0;
}