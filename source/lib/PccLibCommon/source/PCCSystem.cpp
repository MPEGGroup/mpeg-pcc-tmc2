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
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#if _WIN32
#define _UNICODE
#include <windows.h>
#endif

#include <chrono>
#include <memory>
#include "PCCSystem.h"

//===========================================================================

#if _WIN32
namespace pcc {
namespace chrono {
namespace detail {
using hundredns = std::chrono::duration<int64_t, std::ratio<1, 10000000>>;

// global state to emulate getrusage(RUSAGE_CHILDREN).
extern hundredns g_cumulative_time_children;
}  // namespace detail
}  // namespace chrono
}  // namespace pcc
#endif

//===========================================================================

#if _WIN32
int pcc::system( const char* command ) noexcept {
  // Windows doesn't directly support an equivalent of RUSAGE_CHILDREN in
  // GetProcessTimes().  To fully emulate the support is complicated.
  // However, a first order approximation may be made if a process
  // launches a child process that does not itself create any further
  // child processes.
  //
  // This is even simpler in this wrapper, since system() is synchronous:
  //  - launch process and wait for exit
  //  - gather the process times for the child
  //  - add to any existing process timers
  //
  STARTUPINFOW        si{};
  PROCESS_INFORMATION pi{};
  si.cb = sizeof( si );

  size_t                     cmd_size = strlen( command ) + 1;
  std::unique_ptr<wchar_t[]> cmd{new wchar_t[cmd_size]};
  size_t                     num_converted = 0;
  mbstowcs_s( &num_converted, cmd.get(), cmd_size, command, _TRUNCATE );

  if ( CreateProcessW( nullptr, cmd.get(), nullptr, nullptr, 0, 0, nullptr, nullptr, &si, &pi ) == 0 ) { return -1; }

  WaitForSingleObject( pi.hProcess, INFINITE );

  DWORD ret = -1;
  GetExitCodeProcess( pi.hProcess, &ret );

  FILETIME dummy;
  FILETIME userTime;
  GetProcessTimes( pi.hProcess, &dummy, &dummy, &dummy, &userTime );

  ULARGE_INTEGER val;
  val.LowPart  = userTime.dwLowDateTime;
  val.HighPart = userTime.dwHighDateTime;

  using hundredns = std::chrono::duration<int64_t, std::ratio<1, 10000000>>;
  pcc::chrono::detail::g_cumulative_time_children += hundredns( val.QuadPart );

  CloseHandle( pi.hProcess );
  CloseHandle( pi.hThread );

  return int( ret );
}
#endif

//===========================================================================
