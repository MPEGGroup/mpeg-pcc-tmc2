/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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

#ifdef USE_VTMLIB_VIDEO_CODEC

#include "PCCVTMLibVideoDecoder.h"
#include "PCCVTMLibVideoDecoderImpl.h"

using namespace pcc;

template <typename T>
PCCVTMLibVideoDecoder<T>::PCCVTMLibVideoDecoder() {}
template <typename T>
PCCVTMLibVideoDecoder<T>::~PCCVTMLibVideoDecoder() {}

template <typename T>
void PCCVTMLibVideoDecoder<T>::decode( PCCVideoBitstream& bitstream,
                                       PCCVideo<T, 3>&    video,
                                       size_t             outputBitDepth,
                                       const std::string& decoderPath,
                                       const std::string& fileName ) {
  // print information
  fprintf( stdout, "\n" );
  fprintf( stdout, "VVCSoftware: VTM Decoder Version %s ", VTM_VERSION );
  fprintf( stdout, NVM_ONOS );
  fprintf( stdout, NVM_COMPILEDBY );
  fprintf( stdout, NVM_BITS );
#if ENABLE_TRACING
  fprintf( stdout, "[ENABLE_TRACING] " );
#endif
  fprintf( stdout, "\n" );

  PCCVTMLibVideoDecoderImpl<T> decoder;

  // starting time
  double  dResult;
  clock_t lBefore = clock();

  // call decoding function
#ifndef _DEBUG
  try {
#endif  // !_DEBUG
    if ( 0 != decoder.decode( bitstream, outputBitDepth, video ) ) {
      printf( "\n\n***ERROR*** A decoding mismatch occured: signalled md5sum does not match\n" );
    }
#ifndef _DEBUG
  } catch ( Exception& e ) { std::cerr << e.what() << std::endl; } catch ( const std::bad_alloc& e ) {
    std::cout << "Memory allocation failed: " << e.what() << std::endl;
  }
#endif

  // ending time
  dResult = (double)( clock() - lBefore ) / CLOCKS_PER_SEC;
  printf( "\n Total Time: %12.3f sec.\n", dResult );
}

template class pcc::PCCVTMLibVideoDecoder<uint8_t>;
template class pcc::PCCVTMLibVideoDecoder<uint16_t>;

#endif
