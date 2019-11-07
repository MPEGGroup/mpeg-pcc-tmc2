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
using pcc::chrono::StopwatchUserTime;

int main( int argc, char* argv[] ) {
  std::cout << "PccAppDecoder v" << TMC2_VERSION_MAJOR << "." << TMC2_VERSION_MINOR << std::endl << std::endl;

  PCCDecoderParameters decoderParams;
  PCCMetricsParameters metricsParams;
  if ( !parseParameters( argc, argv, decoderParams, metricsParams ) ) { return -1; }
  if ( decoderParams.nbThread_ > 0 ) { tbb::task_scheduler_init init( (int)decoderParams.nbThread_ ); }

  // Timers to count elapsed wall/user time
  pcc::chrono::Stopwatch<std::chrono::steady_clock> clockWall;
  pcc::chrono::StopwatchUserTime                    clockUser;

  clockWall.start();
  int ret = decompressVideo( decoderParams, metricsParams, clockUser );
  clockWall.stop();

  using namespace std::chrono;
  using ms       = milliseconds;
  auto totalWall = duration_cast<ms>( clockWall.count() ).count();
  std::cout << "Processing time (wall): " << ( ret == 0 ? totalWall / 1000.0 : -1 ) << " s\n";

  auto totalUserSelf = duration_cast<ms>( clockUser.self.count() ).count();
  std::cout << "Processing time (user.self): " << ( ret == 0 ? totalUserSelf / 1000.0 : -1 ) << " s\n";

  auto totalUserChild = duration_cast<ms>( clockUser.children.count() ).count();
  std::cout << "Processing time (user.children): " << ( ret == 0 ? totalUserChild / 1000.0 : -1 ) << " s\n";

  std::cout << "Peak memory: " << getPeakMemory() << " KB\n";
  return ret;
}

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
//---------------------------------------------------------------------------
// :: Command line / config parsing

bool parseParameters( int                   argc,
                      char*                 argv[],
                      PCCDecoderParameters& decoderParams,
                      PCCMetricsParameters& metricsParams ) {
  namespace po = df::program_options_lite;

  bool   print_help = false;
  size_t ignore = 1024;

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
    ("c,config", po::parseConfigFile,"Configuration file name")

    // i/o
    ("compressedStreamPath",
     decoderParams.compressedStreamPath_,
     decoderParams.compressedStreamPath_,
     "Output(encoder)/Input(decoder) compressed bitstream")

    ("reconstructedDataPath",
     decoderParams.reconstructedDataPath_,
     decoderParams.reconstructedDataPath_,
     "Output decoded pointcloud. Multi-frame sequences may be represented by %04i")

    // sequence configuration
    ("startFrameNumber",
     decoderParams.startFrameNumber_,
     decoderParams.startFrameNumber_,
     "Fist frame number in sequence to encode/decode")

    // colour space conversion
    ("colorTransform",
     decoderParams.colorTransform_,
     decoderParams.colorTransform_,
     "The colour transform to be applied:\n"
     "  0: none\n"
     "  1: RGB to YCbCr (Rec.709)")

    ("colorSpaceConversionPath",
     decoderParams.colorSpaceConversionPath_,
     decoderParams.colorSpaceConversionPath_,
     "Path to the HDRConvert. If unset, an internal color space conversion is used")
    
    ("inverseColorSpaceConversionConfig",
     decoderParams.inverseColorSpaceConversionConfig_,
     decoderParams.inverseColorSpaceConversionConfig_,
     "HDRConvert configuration file used for YUV420 to RGB444 conversion")

    ("videoDecoderPath",
     decoderParams.videoDecoderPath_,
     decoderParams.videoDecoderPath_,
     "HM video decoder executable")

    ("videoDecoderOccupancyMapPath",
     decoderParams.videoDecoderOccupancyMapPath_,
     decoderParams.videoDecoderOccupancyMapPath_,
     "HM lossless video decoder executable for occupancy map")

    ("nbThread",
     decoderParams.nbThread_,
     decoderParams.nbThread_,
     "Number of thread used for parallel processing")
  
    ("postprocessSmoothingFilterType",
     decoderParams.postprocessSmoothingFilter_,
     decoderParams.postprocessSmoothingFilter_,
     "Exclude geometry smoothing from attribute transfer")
  
    ("keepIntermediateFiles",
     decoderParams.keepIntermediateFiles_,
     decoderParams.keepIntermediateFiles_,
     "Keep intermediate files: RGB, YUV and bin")

    //visual quality
    ("patchColorSubsampling", 
     decoderParams.patchColorSubsampling_,
     false, 
     "Enable per-patch color up-sampling");

  opts.addOptions()
    ("computeChecksum",
     metricsParams.computeChecksum_,
     metricsParams.computeChecksum_,
     "Compute checksum")

    ("computeMetrics",
      metricsParams.computeMetrics_,
      metricsParams.computeMetrics_,
     "Compute metrics")

    ("uncompressedDataFolder",
      metricsParams.uncompressedDataFolder_,
      metricsParams.uncompressedDataFolder_,
      "Folder where the uncompress input data are stored, use for cfg relative paths.")

    ("startFrameNumber",
     metricsParams.startFrameNumber_,
     metricsParams.startFrameNumber_,
     "Fist frame number in sequence to encode/decode")

    ("frameCount",
     metricsParams.frameCount_,
     metricsParams.frameCount_,
     "Number of frames to encode")

    ("groupOfFramesSize",
     metricsParams.groupOfFramesSize_,
     metricsParams.groupOfFramesSize_,
     "Random access period")

    ("uncompressedDataPath",
     metricsParams.uncompressedDataPath_,
     metricsParams.uncompressedDataPath_,
     "Input pointcloud to encode. Multi-frame sequences may be represented by %04i")

    ("reconstructedDataPath",
     metricsParams.reconstructedDataPath_,
     metricsParams.reconstructedDataPath_,
     "Output decoded pointcloud. Multi-frame sequences may be represented by %04i")

    ("normalDataPath",
     metricsParams.normalDataPath_,
     metricsParams.normalDataPath_,
     "Input pointcloud to encode. Multi-frame sequences may be represented by %04i")

    ("resolution",
     metricsParams.resolution_,
     metricsParams.resolution_,
     "Specify the intrinsic resolution")

    ("dropdups",
     metricsParams.dropDuplicates_,
     metricsParams.dropDuplicates_,
     "0(detect), 1(drop), 2(average) subsequent points with same coordinates")

    ("neighborsProc",
     metricsParams.neighborsProc_,
     metricsParams.neighborsProc_,
     "0(undefined), 1(average), 2(weighted average), 3(min), 4(max) neighbors with same geometric distance")

    ("nbThread",
     metricsParams.nbThread_,
     metricsParams.nbThread_,
     "Number of thread used for parallel processing")

    ("minimumImageHeight",    ignore, ignore, "Ignore parameter")
    ("flagColorPreSmoothing", ignore, ignore, "Ignore parameter")
    ("surfaceSeparation",     ignore, ignore, "Ignore parameter");

  // clang-format on
  po::setDefaults( opts );
  po::ErrorReporter        err;
  const list<const char*>& argv_unhandled = po::scanArgv( opts, argc, (const char**)argv, err );
  for ( const auto arg : argv_unhandled ) { err.warn() << "Unhandled argument ignored: " << arg << "\n"; }

  if ( argc == 1 || print_help ) {
    po::doHelp( std::cout, opts, 78 );
    return false;
  }

  decoderParams.completePath();
  decoderParams.print();
  if ( !decoderParams.check() ) { err.error() << "Input parameters are not correct \n"; }
  metricsParams.completePath();
  metricsParams.print();
  if ( !metricsParams.check( true ) ) { err.error() << "Input metrics parameters not correct \n"; }
  metricsParams.startFrameNumber_ = decoderParams.startFrameNumber_;

  // report the current configuration (only in the absence of errors so
  // that errors/warnings are more obvious and in the same place).
  if ( err.is_errored ) return false;

  return true;
}

int decompressVideo( const PCCDecoderParameters& decoderParams,
                     const PCCMetricsParameters& metricsParams,
                     StopwatchUserTime&          clock ) {
  PCCBitstream bitstream;
  PCCBitstreamStat bitstreamStat;
#ifdef BITSTREAM_TRACE
  bitstream.setTrace( true );
  bitstream.openTrace( removeFileExtension( decoderParams.compressedStreamPath_ ) + "_samplestream_read.txt" );
  bitstream.setTraceFile( bitstream.getTraceFile() );
#endif
  if ( !bitstream.initialize( decoderParams.compressedStreamPath_ ) ) { return -1; }
  if ( !bitstream.readHeader() ) { return -1; }  // JR TODO: must be remove? 
  bitstreamStat.setHeader( bitstream.size() );
  size_t      frameNumber = decoderParams.startFrameNumber_;
  PCCMetrics  metrics;
  PCCChecksum checksum;
  metrics.setParameters( metricsParams );
  checksum.setParameters( metricsParams );
  std::vector<std::vector<uint8_t>> checksumsRec, checksumsDec;
  if ( metricsParams.computeChecksum_ ) { checksum.read( decoderParams.compressedStreamPath_ ); }
  PCCDecoder decoder;
  decoder.setParameters( decoderParams );

  VpccUnitStream vpccUS;
  PCCBitstreamDecoder bitstreamDecoder;
	bitstreamDecoder.readSampleStream(bitstream, vpccUS);

  //jkei: we need to read every thing from vpccUS. I am not sure it is desirable...
  bool bMoreData = true;
  while (  bMoreData ) { //jkie : popFront() is desirable?
    PCCGroupOfFrames reconstructs;
    PCCContext       context;
    context.setBitstreamStat( bitstreamStat );
    clock.start();
    int ret = decoder.decode( vpccUS, context, reconstructs );
    clock.stop();
    if ( ret ) { return ret; }
    if ( metricsParams.computeChecksum_ ) { checksum.computeDecoded( reconstructs ); }
    if ( metricsParams.computeMetrics_ ) {
      PCCGroupOfFrames sources, normals;
      if ( !sources.load( metricsParams.uncompressedDataPath_, frameNumber, frameNumber + reconstructs.size(),
                          decoderParams.colorTransform_ ) ) {
        return -1;
      }
      if ( metricsParams.normalDataPath_ != "" ) {
        if ( !normals.load( metricsParams.normalDataPath_, frameNumber, frameNumber + reconstructs.size(),
                            COLOR_TRANSFORM_NONE, true ) ) {
          return -1;
        }
      }
      metrics.compute( sources, reconstructs, normals );
      sources.clear();
      normals.clear();
    }
    
    if ( !decoderParams.reconstructedDataPath_.empty() ) {
      reconstructs.write( decoderParams.reconstructedDataPath_, frameNumber );
    } else {
      frameNumber += reconstructs.size();
    }
    
    bMoreData= (vpccUS.getVpccUnitCount() > 0) ; //jkei: I don't feel like this is clear to understand. getVpccUnitCount() sounds very constant
  }
  bitstreamStat.trace();
  if ( metricsParams.computeMetrics_ ) { metrics.display(); }
  if ( metricsParams.computeChecksum_ ) {
    if ( !checksum.compareRecDec() ) { return -1; }
  }
  return 0;
}
