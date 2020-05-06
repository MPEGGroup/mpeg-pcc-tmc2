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

#include <program_options_lite.h>

#define HM_DECODER
#ifdef HM_DECODER
#include "PCCHMVideoDecoder.h"
#else
#include "PCCVideoDecoder.h"
#endif

using namespace std;
using namespace pcc;

//---------------------------------------------------------------------------
// :: Command line / config parsing helpers

template <typename T>
static std::istream& readUInt( std::istream& in, T& val ) {
  unsigned int tmp;
  in >> tmp;
  val = T( tmp );
  return in;
}

namespace pcc {
static std::istream& operator>>( std::istream& in, PCCColorTransform& val ) { return readUInt( in, val ); }
}  // namespace pcc

size_t getFileSize( const std::string& file ) {
  size_t        size = 0;
  std::ifstream in( file.c_str(), ios_base::binary );
  if ( in.good() ) {
    in.seekg( 0, ios_base::end );
    size = in.tellg();
  }
  in.close();
  return size;
}

//---------------------------------------------------------------------------
// :: Command line / config parsing

bool parseParameters( int argc, char* argv[], std::string& binFile, size_t& width, size_t& height, size_t& nbyte ) {
  namespace po    = df::program_options_lite;
  bool print_help = false;
  // clang-format off
  po::Options opts;
  opts.addOptions()
  ("help", print_help, false, "This help text")
  ("bin", binFile, binFile, "Video bin file")
  ("width", width, width, "Video width")
  ("height", height, height,"Video height")
  ("nbyte", nbyte, nbyte, "Video nbyte");
  // clang-format on
  po::setDefaults( opts );
  po::ErrorReporter        err;
  const list<const char*>& argv_unhandled = po::scanArgv( opts, argc, (const char**)argv, err );

  printf( "Parameters : \n" );
  printf( "  binFile = %s \n", binFile.c_str() );
  printf( "  width   = %zu \n", width );
  printf( "  height  = %zu \n", height );
  printf( "  nbyte   = %zu \n", nbyte );
  if ( argc == 1 || print_help || binFile.empty() || width == 0 || height == 0 || nbyte == 0 ) {
    printf( "Error parameters not correct \n" );
    fflush( stdout );
    return false;
  }

  // report the current configuration (only in the absence of errors so
  // that errors/warnings are more obvious and in the same place).
  if ( err.is_errored ) { return false; }
  return true;
}

int main( int argc, char* argv[] ) {
  std::cout << "PccAppVideoDecoder v" << TMC2_VERSION_MAJOR << "." << TMC2_VERSION_MINOR << std::endl << std::endl;
  std::string binFile;
  size_t      width = 0, height = 0, nbyte = 0;
  if ( !parseParameters( argc, argv, binFile, width, height, nbyte ) ) { return -1; }
  PCCVideo<uint8_t, 3> videoDec;

  std::string yuvDecFileName = removeFileExtension( binFile ) + "_dec.yuv";
#ifdef HM_DECODER
  printf( "Read %s \n", binFile.c_str() );
  size_t        fileSize = getFileSize( binFile );
  std::ifstream fileIn( binFile, std::ios::binary );
  if ( !fileIn.good() ) {
    printf( "Can't read %s \n", binFile.c_str() );
    return false;
  }
  std::vector<uint8_t> bitstream;
  bitstream.resize( fileSize );
  fileIn.read( (char*)( bitstream.data() ), bitstream.size() );
  fileIn.close();

  std::string        s( reinterpret_cast<char*>( bitstream.data() ), bitstream.size() );
  std::istringstream iss( s );
  std::istream&      stream = iss;
  PCCHMVideoDecoder  hmDecoder;
  hmDecoder.decode( stream, nbyte == 1 ? 8 : 0, false, videoDec );

  std::ofstream fileOut( yuvDecFileName.c_str(), std::ios::binary );
  if ( !fileOut.good() ) { return false; }
  size_t size = nbyte * width * height;
  if ( sizeof( uint8_t ) == nbyte ) {
    for ( auto& image : videoDec ) {
      fileOut.write( (char*)( image.getChannel( 0 ).data() ), size );
      fileOut.write( (char*)( image.getChannel( 1 ).data() ), size / 4 );
      fileOut.write( (char*)( image.getChannel( 2 ).data() ), size / 4 );
    }
  } else {
    std::vector<uint8_t> data;
    data.resize( 6 * size / 4 );
    for ( auto& image : videoDec ) {
      auto* tmp = data.data();
      for ( size_t i = 0; i < size; ++i ) { *( tmp++ ) = image.getChannel( 0 ).data()[i]; }
      for ( size_t i = 0; i < size / 4; ++i ) { *( tmp++ ) = image.getChannel( 1 ).data()[i]; }
      for ( size_t i = 0; i < size / 4; ++i ) { *( tmp++ ) = image.getChannel( 2 ).data()[i]; }
      fileOut.write( (char*)( data.data() ), 6 * size / 4 );
    }
  }
  fileOut.close();

#else
  std::string       enc = "TAppEncoderHighBitDepthStatic";
  PCCVideoBitstream videoBitstream( VIDEO_OCCUPANCY );
  PCCContext        context;
  PCCVideoDeoder    encoder;
  encoder.compress( videoSrc, std::string( "" ), 8, videoBitstream,
                    encoderParameters,  // occupancyMapVideoEncoderConfig_,
                    enc,                // videoEncoderOccupancyMapPath_,
                    context, 1,         // nByte
                    false,              // use444CodecIo
                    false,              // use3dmv
                    8,                  // internalBitDepth
                    false,              // useConversion
                    true );
#endif

  return 1;
}