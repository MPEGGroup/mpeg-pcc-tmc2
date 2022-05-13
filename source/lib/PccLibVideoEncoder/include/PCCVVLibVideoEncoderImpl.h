/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
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

#ifndef __PCCVVLibVideoEncoderImpl_H__
#define __PCCVVLibVideoEncoderImpl_H__

#include "PCCCommon.h"

#ifdef USE_VVLIB_VIDEO_CODEC
#include "PCCVideo.h"
#include "PCCVideoBitstream.h"

#include <list>
#include <ostream>
#include <list>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <assert.h>
#include <iomanip>

#include "vvenc/version.h"
#include "vvenc/vvenc.h"
#include "apputils/ParseArg.h"
#include "apputils/YuvFileIO.h"
#include "apputils/VVEncAppCfg.h"

namespace pcc {

/// encoder application class
template <class T>
class PCCVVLibVideoEncoderImpl {
 public:
  PCCVVLibVideoEncoderImpl();
  ~PCCVVLibVideoEncoderImpl();
  void encode( PCCVideo<T, 3>&    videoSrc,
               std::string        arguments,
               PCCVideoBitstream& bitstream,
               PCCVideo<T, 3>&    videoRec );

 private:
  bool        parseCfg( int argc, char* argv[] );
  int         encode( PCCVideo<T, 3>& videoSrc, std::ostream& bitstream, PCCVideo<T, 3>& videoRec );
  void        outputAU( const vvencAccessUnit& au, std::ostream& bitstream );
  static void outputYuv( void*, vvencYUVBuffer* );
  int         getSrcFrame( PCCVideo<T, 3>& videoSrc, vvencYUVBuffer& pic, int frameIndex );
  static int  storeRecFrame( const vvencYUVBuffer* pcFrame, vvenc_config& config, PCCVideo<T, 3>& videoRec );
  void        printRateSummary( int framesRcvd );
  void        printChromaFormat();
  static void changePreset( vvenc_config* c, vvencPresetMode preset ) {
    if ( c ) { vvenc_init_preset( c, (vvencPresetMode)preset ); }
  }
  apputils::VVEncAppCfg m_cEncAppCfg;  
  vvenc_config          m_vvenc_config;
  vvencEncoder*         m_encCtx; 
  unsigned              m_essentialBytes;
  unsigned              m_totalBytes;
};

}  // namespace pcc

#endif

#endif  //~__PCCVVLibVideoEncoderImpl_H__
