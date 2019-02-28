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
#define _CRT_SECURE_NO_WARNINGS

#include "PCCCommon.h"
#include "PCCChrono.h"
#include "PCCMemory.h"
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCBitstream.h"
#include "PCCBitstreamDecoder.h"
#include "PCCBitstreamEncoderNewSyntax.h"
#include "PCCBitstreamDecoderNewSyntax.h"
#include <program_options_lite.h>
#include <tbb/tbb.h>

using namespace std;
using namespace pcc;
using pcc::chrono::StopwatchUserTime;

int newSyntaxBitstreamInspector( const std::string&  compressedStreamPath,
                                 StopwatchUserTime& clock ) {
  PCCBitstream bitstream;
  if ( !bitstream.initialize( compressedStreamPath ) ) { return -1; }
  PCCMetadataEnabledFlags gofLevelMetadataEnabledFlags;
  if ( !bitstream.readHeader( gofLevelMetadataEnabledFlags ) ) { return -1; }
  size_t contextIndex = 0;

  while ( bitstream.size() < bitstream.capacity() ) {
    PCCContext context;
    context.getSps().setSequenceParameterSetId( contextIndex );
    if ( gofLevelMetadataEnabledFlags.getMetadataEnabled() ) {
      auto& gofLevelMetadata                     = context.getGOFLevelMetadata();
      gofLevelMetadata.getMetadataEnabledFlags() = gofLevelMetadataEnabledFlags;
    }
    clock.start();
    PCCBitstreamDecoder bitstreamDecoder;    
    if ( !bitstreamDecoder.decode( bitstream, context ) ) {
      fflush( stdout );
      return 0;
    }
    printf( "Read context %4lu : %9lu / %9lu \n", contextIndex, bitstream.size(),
            bitstream.capacity() );
    fflush( stdout );
    
    PCCBitstream bitstreamNewSyntaxEnc;
    PCCBitstreamEncoderNewSyntax bitstreamEncoderNewSyntax;

#ifdef BITSTREAM_TRACE
    bitstreamNewSyntaxEnc.setTrace( true );
    bitstreamNewSyntaxEnc.openTrace( removeFileExtension( compressedStreamPath ) + "_new_syntax_encode.txt" );
#endif

    printf( "bitstreamEncoderNewSyntax: \n" ); fflush(stdout);
    bitstreamEncoderNewSyntax.encode( context, bitstreamNewSyntaxEnc );    
    printf( "  ==> new bitstream %9lu / %9lu \n", bitstreamNewSyntaxEnc.size(),
            bitstreamNewSyntaxEnc.capacity() );

    PCCBitstream bitstreamNewSyntaxDec;
    bitstreamNewSyntaxDec.initialize( bitstreamNewSyntaxEnc );
    PCCBitstreamDecoderNewSyntax bitstreamDecoderNewSyntax;
#ifdef BITSTREAM_TRACE
    bitstreamNewSyntaxDec.setTrace( true );
    bitstreamNewSyntaxDec.openTrace( removeFileExtension( compressedStreamPath ) + "_new_syntax_decode.txt" );
#endif

    printf( "bitstreamEncoderNewSyntax: \n" ); fflush(stdout);
    bitstreamDecoderNewSyntax.decode( context, bitstreamNewSyntaxDec ); 

    printf( "Done context %4lu \n" );
    fflush( stdout );

    clock.stop();
    contextIndex++;
  }
  return 0;
}

//---------------------------------------------------------------------------
// :: Command line / config parsing helpers

template <typename T>
static std::istream &readUInt(std::istream &in, T &val) {
  unsigned int tmp;
  in >> tmp;
  val = T(tmp);
  return in;
}

namespace pcc{
  static std::istream &operator>>(std::istream &in, ColorTransform &val) { return readUInt(in, val); }
}
//---------------------------------------------------------------------------
// :: Command line / config parsing

bool parseParameters( int argc, char *argv[],
                      std::string& compressedStreamPath ) {

  namespace po = df::program_options_lite;

  bool print_help = false;
  size_t ignore;

  // The definition of the program/config options, along with default values.
  //
  // NB: when updating the following tables:
  //      (a) please keep to 80-columns for easier reading at a glance,
  //      (b) do not vertically align values -- it breaks quickly
  //
  // clang-format off
  po::Options opts;
  opts.addOptions()
    ("help", print_help, false, "This help text")

    ("compressedStreamPath",
      compressedStreamPath,
      compressedStreamPath,
     "Output(encoder)/Input(decoder) compressed bitstream")
    ;

  // clang-format on
  po::setDefaults(opts);
  po::ErrorReporter err;
  const list<const char *> &argv_unhandled = po::scanArgv(opts, argc, (const char **)argv, err);
  for (const auto arg : argv_unhandled) {
    err.warn() << "Unhandled argument ignored: " << arg << "\n";
  }

  if (argc == 1 || print_help) {
    po::doHelp( std::cout, opts, 78 );
    return false;
  }
  // report the current configuration (only in the absence of errors so
  // that errors/warnings are more obvious and in the same place).
  if (err.is_errored) return false;

  return true;
}

int main(int argc, char *argv[]) {
  std::cout << "PccAppNewSyntaxInspector v" << TMC2_VERSION_MAJOR << "." << TMC2_VERSION_MINOR << std::endl
            << std::endl;

  std::string compressedStreamPath;
  if (!parseParameters(argc, argv, compressedStreamPath )) {
    return -1;
  }

  // Timers to count elapsed wall/user time
  pcc::chrono::Stopwatch<std::chrono::steady_clock> clockWall;
  pcc::chrono::StopwatchUserTime clockUser;

  clockWall.start();
  int ret = newSyntaxBitstreamInspector( compressedStreamPath, clockUser );
  clockWall.stop();

  using namespace std::chrono;
  using ms = milliseconds;
  auto totalWall = duration_cast<ms>(clockWall.count()).count();
  std::cout << "Processing time (wall): " << ( ret == 0 ? totalWall / 1000.0 : -1 ) << " s\n";

  auto totalUserSelf = duration_cast<ms>(clockUser.self.count()).count();
  std::cout << "Processing time (user.self): "
            << ( ret == 0 ? totalUserSelf / 1000.0 : -1 ) << " s\n";

  auto totalUserChild = duration_cast<ms>(clockUser.children.count()).count();
  std::cout << "Processing time (user.children): "
            << ( ret == 0 ? totalUserChild / 1000.0 : -1 ) << " s\n";

  std::cout << "Peak memory: " << getPeakMemory() << " KB\n";
  return ret;
}
