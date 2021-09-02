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
#include <program_options_lite.h>
#include "PCCVirtualColorConverter.h"
#ifdef USE_HDRTOOLS
#include "PCCHDRToolsLibColorConverter.h"
#else
#include "PCCHDRToolsAppColorConverter.h"
#include "PCCInternalColorConverter.h"
#endif

using namespace std;
using namespace pcc;

bool parseParameters( int          argc,
                      char*        argv[],
                      std::string& srcVideoPath,
                      std::string& dstVideoPath,
                      std::string& configFile,
                      size_t&      width,
                      size_t&      height,
                      std::string& colorFormat,
                      size_t&      inputNumBytes,
                      size_t&      outputNumBytes ) {
  namespace po    = df::program_options_lite;
  bool print_help = false;
  // clang-format off
  po::Options opts;
  opts.addOptions()
     ( "help",            print_help,     false,          "This help text" )
     ( "srcVideoPath",   srcVideoPath,   srcVideoPath,   "Source yuv file path" )
     ( "dstVideoPath",   dstVideoPath,   dstVideoPath,   "Create yuv file path" )
     ( "configFile",     configFile,     configFile,     "HDRTools configuration files" )
     ( "width",          width,          width,          "Source video width" )
     ( "height",         height,         height,         "Source video height" )
     ( "colorFormat",    colorFormat,    colorFormat,    "Source color format" )
     ( "inputNumBytes",  inputNumBytes,  inputNumBytes,  "Source video nbyte" )
     ( "outputNumBytes", outputNumBytes, outputNumBytes, "Output video nbyte" );
  // clang-format on
  po::setDefaults( opts );
  po::ErrorReporter        err;
  const list<const char*>& argv_unhandled = po::scanArgv( opts, argc, (const char**)argv, err );
  for ( const auto arg : argv_unhandled ) { printf( "Unhandled argument ignored: %s \n", arg ); }

  printf( "parseParameters : \n" );
  printf( "  srcVideoPath   = %s \n", srcVideoPath.c_str() );
  printf( "  dstVideoPath   = %s \n", dstVideoPath.c_str() );
  printf( "  configFile     = %s \n", configFile.c_str() );
  printf( "  width          = %zu \n", width );
  printf( "  height         = %zu \n", height );
  printf( "  colorFormat    = %s  \n", colorFormat.c_str() );
  printf( "  inputNumBytes  = %zu \n", inputNumBytes );
  printf( "  outputNumBytes = %zu \n", outputNumBytes );

  if ( argc == 1 || print_help || srcVideoPath.empty() || dstVideoPath.empty() || configFile.empty() ||
       colorFormat.empty() || width == 0 || height == 0 || inputNumBytes == 0 || outputNumBytes == 0 ||
       ( colorFormat != "RGB444" && colorFormat != "YUV444" && colorFormat != "YUV420" ) ) {
    printf( "Error parameters not correct \n" );
    po::doHelp( std::cout, opts, 78 );
    return false;
  }

  // report the current configuration (only in the absence of errors so
  // that errors/warnings are more obvious and in the same place).
  if ( err.is_errored ) { return false; }
  return true;
}

int main( int argc, char* argv[] ) {
  std::cout << "PccAppVideoConverter v" << TMC2_VERSION_MAJOR << "." << TMC2_VERSION_MINOR << std::endl << std::endl;
  std::string srcVideoPath, dstVideoPath, configFile, colorFormat;
  size_t      width = 0, height = 0, inputNumBytes = 0, outputNumBytes = 0;
  if ( !parseParameters( argc, argv, srcVideoPath, dstVideoPath, configFile, width, height, colorFormat, inputNumBytes,
                         outputNumBytes ) ) {
    return -1;
  }
  typedef uint16_t T;
  PCCCOLORFORMAT   format = colorFormat == "RGB444" ? RGB444 : colorFormat == "YUV444" ? YUV444 : YUV420;
  PCCVideo<T, 3>   videoSrc, videoRec;
  videoSrc.read( srcVideoPath, width, height, format, inputNumBytes );
#ifdef USE_HDRTOOLS
  PCCHDRToolsLibColorConverter<T> convert;
#else
  PCCHDRToolsAppColorConverter<T> convert;
#endif
  convert.convert( configFile, videoSrc, videoRec );
  videoRec.write( dstVideoPath, outputNumBytes );
  return 1;
}