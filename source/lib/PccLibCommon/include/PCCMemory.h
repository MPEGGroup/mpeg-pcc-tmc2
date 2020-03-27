/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2018, ISO/IEC
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
 * CONTRACT, STRICT LIABILITY, OR TOR
#include "PCCCommon.h"T (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once

#include "PCCCommon.h"

namespace pcc {

// ********************************************************************* //
// ******************** Note: this value is in KB! ********************* //
// ********************************************************************* //

#if defined( WIN32 )
#include <windows.h>
#include <psapi.h>
static int getUsedMemory() {
  PROCESS_MEMORY_COUNTERS pmc;
  GetProcessMemoryInfo( GetCurrentProcess(), &pmc, sizeof( pmc ) );
  return pmc.WorkingSetSize / 1024;
}
static uint64_t getPeakMemory() {
  PROCESS_MEMORY_COUNTERS pmc;
  GetProcessMemoryInfo( GetCurrentProcess(), &pmc, sizeof( pmc ) );
  return (uint64_t)pmc.PeakWorkingSetSize / 1024;
}
#elif defined( __APPLE__ ) && defined( __MACH__ )
static inline int getUsedMemory() {
  struct mach_task_basic_info info;
  mach_msg_type_number_t      infoCount = MACH_TASK_BASIC_INFO_COUNT;
  if ( task_info( mach_task_self(), MACH_TASK_BASIC_INFO, (task_info_t)&info, &infoCount ) != KERN_SUCCESS ) {
    return 0;
  }
  return (size_t)info.resident_size;
}
static uint64_t getPeakMemory() {
  struct rusage rusage;
  getrusage( RUSAGE_SELF, &rusage );
  return (size_t)rusage.ru_maxrss / 1024;
}
#else
static int parseLine( char* pLine ) {
  int         iLen = (int)strlen( pLine );
  const char* pTmp = pLine;
  while ( *pTmp < '0' || *pTmp > '9' ) { pTmp++; }
  pLine[iLen - 3] = '\0';
  iLen            = atoi( pTmp );
  return iLen;
}
static int getUsedMemory() {
  FILE* pFile   = fopen( "/proc/self/status", "r" );
  int   iResult = 0;
  if ( pFile != NULL ) {
    char pLine[128];
    while ( fgets( pLine, 128, pFile ) != NULL ) {
      if ( strncmp( pLine, "VmSize:", 7 ) == 0 ) {
        iResult          = (int)strlen( pLine );
        const char* pTmp = pLine;
        while ( *pTmp < '0' || *pTmp > '9' ) { pTmp++; }
        pLine[iResult - 3] = '\0';
        iResult            = atoi( pTmp );
        break;
      }
    }
    fclose( pFile );
  }
  return iResult;
}
static uint64_t getPeakMemory() {
  FILE*    pFile   = fopen( "/proc/self/status", "r" );
  uint64_t iResult = 0;
  if ( pFile != NULL ) {
    char pLine[128];
    while ( fgets( pLine, 128, pFile ) != NULL ) {
      if ( strncmp( pLine, "VmPeak:", 7 ) == 0 ) {
        const char* pTmp = pLine;
        while ( *pTmp < '0' || *pTmp > '9' ) { pTmp++; }
        pLine[(int)strlen( pLine ) - 3] = '\0';
        iResult                         = atoi( pTmp );
        break;
      }
    }
    fclose( pFile );
  }
  return iResult;
}
#endif

};  // namespace pcc
