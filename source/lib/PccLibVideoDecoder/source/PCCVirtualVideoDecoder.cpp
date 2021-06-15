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

#include "PCCVirtualVideoDecoder.h"

#include "PCCJMAppVideoDecoder.h"
#include "PCCHMAppVideoDecoder.h"
#include "PCCJMLibVideoDecoder.h"
#include "PCCHMLibVideoDecoder.h"
#include "PCCVTMLibVideoDecoder.h"
#include "PCCSHMAppVideoDecoder.h"
#ifdef USE_FFMPEG_VIDEO_CODEC
#include "PCCFFMPEGLibVideoDecoder.h"
#endif
using namespace pcc;

template <typename T>
std::shared_ptr<PCCVirtualVideoDecoder<T>> PCCVirtualVideoDecoder<T>::create( PCCCodecId codecId ) {
  printf( "PCCVirtualVideoDecoder: create codecId = %d \n", codecId );
  fflush( stdout );
  switch ( codecId ) {
#ifdef USE_JMAPP_VIDEO_CODEC
    case JMAPP: return std::make_shared<PCCJMAppVideoDecoder<T>>(); break;
#endif
#ifdef USE_HMAPP_VIDEO_CODEC
    case HMAPP: return std::make_shared<PCCHMAppVideoDecoder<T>>(); break;
#endif
#ifdef USE_SHMAPP_VIDEO_CODEC
    case SHMAPP: return std::make_shared<PCCSHMAppVideoDecoder<T>>(); break;
#endif
#ifdef USE_JMLIB_VIDEO_CODEC
    case JMLIB: return std::make_shared<PCCJMLibVideoDecoder<T>>(); break;
#endif
#ifdef USE_HMLIB_VIDEO_CODEC
    case HMLIB: return std::make_shared<PCCHMLibVideoDecoder<T>>(); break;
#endif
#ifdef USE_VTMLIB_VIDEO_CODEC
    case VTMLIB: return std::make_shared<PCCVTMLibVideoDecoder<T>>(); break;
#endif
#ifdef USE_FFMPEG_VIDEO_CODEC
    case FFMPEG: return std::make_shared<PCCFFMPEGLibVideoDecoder<T>>(); break;
#endif
    default:
      printf( "Error: codec id not supported \n" );
      exit( -1 );
      break;
  }
  return nullptr;
}

template class pcc::PCCVirtualVideoDecoder<uint8_t>;
template class pcc::PCCVirtualVideoDecoder<uint16_t>;
