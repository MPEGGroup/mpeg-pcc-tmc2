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
#include "PCCConformance.h"
#include "PCCConformanceParameters.h"
#include <program_options_lite.h>

using namespace pcc;
using namespace std;

bool parseParameters( int argc, char* argv[], PCCConformanceParameters& params ) {
  namespace po    = df::program_options_lite;
  bool print_help = false;
  // clang-format off
  po::Options opts;
  opts.addOptions()
     ( "help", print_help, false, "This help text")
     ( "checkConformance", 
      params.checkConformance_, 
      params.checkConformance_, 
      "Check conformance")
     ( "path", 
       params.path_, 
       params.path_, 
       "Root directory of conformance files + prefix: C:\\Test\\pcc_conformance\\Bin\\S26C03R03_")
     ( "level", 
       params.levelIdc_,
       params.levelIdc_, 
       "Level indicator") 
     //should be in multiples of 30 i.e. levelIdc of 30 corresponds to level 1, and levelIdc 105 corresponds to level 3.5 of Table A.5
     ( "fps", 
       params.fps_, 
       params.fps_, 
       "frame per second");
  // clang-format on
  po::setDefaults( opts );
  po::ErrorReporter        err;
  const list<const char*>& argv_unhandled = po::scanArgv( opts, argc, (const char**)argv, err );
  for ( const auto arg : argv_unhandled ) { printf( "Unhandled argument ignored: %s \n", arg ); }

  params.print();
  if ( !params.check() ) {
    po::doHelp( std::cout, opts, 78 );
    printf( "Error Input parameters are not correct \n" );
    return false;
  }

  // report the current configuration (only in the absence of errors so
  // that errors/warnings are more obvious and in the same place).
  if ( err.is_errored ) { return false; }
  return true;
}

int main( int argc, char* argv[] ) {
  std::cout << "PccAppConformance v" << TMC2_VERSION_MAJOR << "." << TMC2_VERSION_MINOR << std::endl << std::endl;
  PCCConformanceParameters params;
  if ( !parseParameters( argc, argv, params ) ) { return -1; }

  PCCConformance conformance;
  conformance.check( params );
  return 0;
}
