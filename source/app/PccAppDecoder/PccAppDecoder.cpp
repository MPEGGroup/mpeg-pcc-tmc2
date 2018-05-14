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
#include "PccAppDecoder.h"

using namespace std;
using namespace pcc;

int main(int argc, char *argv[]) {
  std::cout << "PccAppDecoder v" << TMC2_VERSION_MAJOR << "." << TMC2_VERSION_MINOR << std::endl
            << std::endl;

  PCCDecoderParameters params;
  if (!ParseParameters(argc, argv, params )) {
    return -1;
  }
  if( params.nbThread_ > 0 ) {
    tbb::task_scheduler_init init( (int)params.nbThread_ );
  }

  // Timers to count elapsed wall/user time
  pcc::chrono::Stopwatch<std::chrono::steady_clock> clock_wall;
  pcc::chrono::Stopwatch<pcc::chrono::utime_inc_children_clock> clock_user;

  clock_wall.start();
  int ret = DecompressVideo( params, clock_user );
  clock_wall.stop();

  using namespace std::chrono;
  auto total_wall = duration_cast<milliseconds>(clock_wall.count()).count();
  auto total_user = duration_cast<milliseconds>(clock_user.count()).count();
  std::cout << "Processing time (wall): " << total_wall / 1000.0 << " s\n";
  std::cout << "Processing time (user): " << total_user / 1000.0 << " s\n";

  return ret;
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

static std::istream &operator>>(std::istream &in, ColorTransform &val) { return readUInt(in, val); }

//---------------------------------------------------------------------------
// :: Command line / config parsing

bool ParseParameters(int argc, char *argv[], PCCDecoderParameters& params ) {

  namespace po = df::program_options_lite;

  bool print_help = false;

  // The definition of the program/config options, along with default values.
  //
  // NB: when updating the following tables:
  //      (a) please keep to 80-columns for easier reading at a glance,
  //      (b) do not vertically align values -- it breaks quickly
  //
  po::Options opts;
  opts.addOptions()
    ("help", print_help, false, "This help text")
    ("c,config", po::parseConfigFile,"Configuration file name")

    // i/o
    ("compressedStreamPath",
     params.compressedStreamPath_,
     params.compressedStreamPath_,
     "Output(encoder)/Input(decoder) compressed bitstream")

    ("reconstructedDataPath",
     params.reconstructedDataPath_,
     params.reconstructedDataPath_,
     "Output decoded pointcloud. Multi-frame sequences may be represented by %04i")

    // sequence configuration
    ("startFrameNumber",
     params.startFrameNumber_,
     params.startFrameNumber_,
     "Fist frame number in sequence to encode/decode")

    // colour space conversion
    ("colorTransform",
     params.colorTransform_,
     params.colorTransform_,
     "The colour transform to be applied:\n"
     "  0: none\n"
     "  1: RGB to YCbCr (Rec.709)")

    ("colorSpaceConversionPath",
     params.colorSpaceConversionPath_,
     params.colorSpaceConversionPath_,
     "Path to the HDRConvert. If unset, an internal color space conversion is used")
    
    ("inverseColorSpaceConversionConfig",
     params.inverseColorSpaceConversionConfig_,
     params.inverseColorSpaceConversionConfig_,
     "HDRConvert configuration file used for YUV420 to RGB444 conversion")

    ("videoDecoderPath",
     params.videoDecoderPath_,
     params.videoDecoderPath_,
     "HM video decoder executable")

    ("nbThread",
     params.nbThread_,
     params.nbThread_,
     "Number of thread used for parallel processing")

    ("keepIntermediateFiles",
     params.keepIntermediateFiles_,
     params.keepIntermediateFiles_,
     "Keep intermediate files: RGB, YUV and bin")

 ;

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

  params.print();
  if( !params.check() ) {
    err.error() << "Input parameters are not correct \n";
  }

  // report the current configuration (only in the absence of errors so
  // that errors/warnings are more obvious and in the same place).
  if (err.is_errored) return false;

  return true;
}

int DecompressVideo(const PCCDecoderParameters &params, Stopwatch &clock) {
  PCCBitstream bitstream;
  if( ! bitstream.initialize( params.compressedStreamPath_ ) ) {
    return -1;
  }
  if( ! bitstream.readHeader() ) {
    return -1;
  }
  size_t frameNumber = params.startFrameNumber_;
  size_t contextIndex = 0;

  PCCDecoder decoder;
  decoder.setParameters( params );
  while ( bitstream.size() < bitstream.capacity()) {
    PCCGroupOfFrames reconstruct; 
    PCCContext context;
    context.setIndex( contextIndex++ );
    clock.start();
    int ret = decoder.decompress( bitstream, context, reconstruct );
    clock.stop();
    if (ret) {
      return ret;
    }
    reconstruct.write( params.reconstructedDataPath_, frameNumber );
  }
  return 0;
}
