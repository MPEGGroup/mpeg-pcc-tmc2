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
#include "PCCChrono.h"
#include "PCCGroupOfFrames.h"
#include "PCCMetrics.h"
#include "PCCMetricsParameters.h"
#include <program_options_lite.h>
#include <tbb/tbb.h>

using namespace std;
using namespace pcc;
using pcc::chrono::StopwatchUserTime;

bool parseParameters( int argc, char* argv[], PCCMetricsParameters& metricsParams ) {
  namespace po      = df::program_options_lite;
  bool   print_help = false;
  size_t ignore     = 0;

  // The definition of the program/config options, along with default values.
  //
  // NB: when updating the following tables:
  //      (a) please keep to 80-columns for easier reading at a glance,
  //      (b) do not vertically align values -- it breaks quickly
  //
  // clang-format off
  po::Options opts;
  opts.addOptions()
    ( "help", print_help, false, "This help text" )
    ( "computeChecksum", 
      metricsParams.computeChecksum_, 
      metricsParams.computeChecksum_,
      "Compute checksum" )
    ( "computeMetrics", 
      metricsParams.computeMetrics_, 
      metricsParams.computeMetrics_, 
      "Compute metrics" )
    // sequence configuration
    ( "startFrameNumber", 
      metricsParams.startFrameNumber_,    
      metricsParams.startFrameNumber_,
      "Fist frame number in sequence to encode/decode" ) 
    ( "frameCount", 
      metricsParams.frameCount_,
       metricsParams.frameCount_, 
      "Number of frames to encode" ) 
    ( "uncompressedDataPath", 
      metricsParams.uncompressedDataPath_,
       metricsParams.uncompressedDataPath_,
      "Input pointcloud to encode. Multi-frame sequences may be represented by %04i" ) 
    ( "reconstructedDataPath", 
      metricsParams.reconstructedDataPath_,
      metricsParams.reconstructedDataPath_, 
      "Output decoded pointcloud. Multi-frame sequences may be represented by %04i" ) 
    ( "normalDataPath", 
      metricsParams.normalDataPath_,
      metricsParams.normalDataPath_,
      "Input pointcloud to encode. Multi-frame sequences may be represented by %04i" ) 
    ( "resolution", 
      metricsParams.resolution_, 
      metricsParams.resolution_,
      "Specify the intrinsic resolution" ) 
    ( "dropdups", 
      metricsParams.dropDuplicates_,
      metricsParams.dropDuplicates_,
      "0(detect), 1(drop), 2(average) subsequent points with same coordinates" ) 
    ( "neighborsProc", 
      metricsParams.neighborsProc_,
      metricsParams.neighborsProc_,
      "0(undefined), 1(average), 2(weighted average), 3(min), 4(max) neighbors with same geometric distance" ) 
    ( "nbThread", 
      metricsParams.nbThread_, 
      metricsParams.nbThread_,
      "Number of thread used for parallel processing" ) 
    ( "minimumImageHeight", ignore, ignore, "Ignore parameter" ) 
    ( "flagColorPreSmoothing", ignore, ignore, "Ignore parameter" ) 
    ( "surfaceSeparation", ignore, ignore, "Ignore parameter" );
  // clang-format on
  po::setDefaults( opts );
  po::ErrorReporter        err;
  const list<const char*>& argv_unhandled = po::scanArgv( opts, argc, (const char**)argv, err );
  for ( const auto arg : argv_unhandled ) { printf( "Unhandled argument ignored: %s \n", arg ); }
  if ( argc == 1 || print_help ) {
    po::doHelp( std::cout, opts, 78 );
    return false;
  }

  metricsParams.completePath();
  metricsParams.print();
  if ( !metricsParams.check( true ) ) {
    printf( "Error Input parameters are not correct \n" );
    return false;
  }

  // report the current configuration (only in the absence of errors so
  // that errors/warnings are more obvious and in the same place).
  if ( err.is_errored ) { return false; }
  return true;
}

int computeMetrics( const PCCMetricsParameters& metricsParams, StopwatchUserTime& clock ) {
  PCCMetrics metrics;
  metrics.setParameters( metricsParams );
  for ( size_t frameIndex = metricsParams.startFrameNumber_;
        frameIndex < metricsParams.startFrameNumber_ + metricsParams.frameCount_; frameIndex++ ) {
    PCCGroupOfFrames sources;
    PCCGroupOfFrames reconstructs;
    PCCGroupOfFrames normals;
    if ( !sources.load( metricsParams.uncompressedDataPath_, frameIndex, frameIndex + 1, COLOR_TRANSFORM_NONE ) ) {
      return -1;
    }
    if ( !reconstructs.load( metricsParams.reconstructedDataPath_, frameIndex, frameIndex + 1,
                             COLOR_TRANSFORM_NONE ) ) {
      return -1;
    }
    if ( !metricsParams.normalDataPath_.empty() ) {
      if ( !normals.load( metricsParams.normalDataPath_, frameIndex, frameIndex + 1, COLOR_TRANSFORM_NONE, true ) ) {
        return -1;
      }
    }
    metrics.compute( sources, reconstructs, normals );
  }
  metrics.display();
  return 0;
}

int main( int argc, char* argv[] ) {
  std::cout << "PccAppMetrics v" << TMC2_VERSION_MAJOR << "." << TMC2_VERSION_MINOR << std::endl << std::endl;

  PCCMetricsParameters metricsParams;
  if ( !parseParameters( argc, argv, metricsParams ) ) { return -1; }
  if ( metricsParams.nbThread_ > 0 ) { tbb::task_scheduler_init init( static_cast<int>( metricsParams.nbThread_ ) ); }

  // Timers to count elapsed wall/user time
  pcc::chrono::Stopwatch<std::chrono::steady_clock> clockWall;
  pcc::chrono::StopwatchUserTime                    clockUser;

  clockWall.start();
  int ret = computeMetrics( metricsParams, clockUser );
  clockWall.stop();

  using namespace std::chrono;
  using ms            = milliseconds;
  auto totalWall      = duration_cast<ms>( clockWall.count() ).count();
  auto totalUserSelf  = duration_cast<ms>( clockUser.self.count() ).count();
  auto totalUserChild = duration_cast<ms>( clockUser.children.count() ).count();
  std::cout << "metric:Processing time (wall): " << totalWall / 1000.0 << " s\n";
  std::cout << "metric:Processing time (user.self): " << totalUserSelf / 1000.0 << " s\n";
  std::cout << "metric:Processing time (user.children): " << totalUserChild / 1000.0 << " s\n";
  return ret;
}