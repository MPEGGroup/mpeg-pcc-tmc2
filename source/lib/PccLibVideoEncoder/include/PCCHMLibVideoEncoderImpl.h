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

#ifndef __PCCHMLibVideoEncoderImpl_H__
#define __PCCHMLibVideoEncoderImpl_H__

#include "PCCCommon.h"

#ifdef USE_HMLIB_VIDEO_CODEC
#include "PCCVideo.h"
#include "PCCVideoBitstream.h"

#include <list>
#include <ostream>
#include "TLibEncoder/TEncTop.h"
#include "TLibCommon/AccessUnit.h"
#include "PCCHMLibVideoEncoderCfg.h"
#include <list>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <assert.h>
#include <iomanip>
#include "TLibEncoder/AnnexBwrite.h"
#if EXTENSION_360_VIDEO
#include "TAppEncHelper360/TExt360AppEncTop.h"
#endif
#include "TAppCommon/program_options_lite.h"

namespace pcc {

using namespace pcc_hm;

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder application class
template <class T>
class PCCHMLibVideoEncoderImpl : public PCCHMLibVideoEncoderCfg {
 public:
  PCCHMLibVideoEncoderImpl();

  ~PCCHMLibVideoEncoderImpl();

  Void encode( PCCVideo<T, 3>&    videoSrc,
               std::string        arguments,
               PCCVideoBitstream& bitstream,

               PCCVideo<T, 3>& videoRec );
  // #if PCC_CF_EXT
  // void setLogger( PCCLogger& logger ) { logger_ = &logger; }
  // #endif
  // protected:

  // #if PCC_CF_EXT
  // PCCLogger* logger_ = nullptr;
  // #endif

 private:
  Void xInitLibCfg();
  Void xGetBuffer( TComPicYuv*& rpcPicYuvRec );
  Void xDeleteBuffer();
  Void xWriteOutput( std::ostream&                bitstreamFile,
                     Int                          iNumEncoded,
                     const std::list<AccessUnit>& accessUnits,
                     PCCVideo<T, 3>&              videoRec );
  Void rateStatsAccum( const AccessUnit& au, const std::vector<UInt>& annexBsizes );
  Void printRateSummary();
  Void printChromaFormat();
  void xWritePicture( const TComPicYuv* pic, PCCVideo<T, 3>& video );
  void xReadPicture( TComPicYuv* pic, PCCVideo<T, 3>& video, int frameIndex );

  TEncTop               m_cTEncTop;
  TComList<TComPicYuv*> m_cListPicYuvRec;
  Int                   m_iFrameRcvd;
  UInt                  m_essentialBytes;
  UInt                  m_totalBytes;
  int                   m_outputWidth;
  int                   m_outputHeight;
};

}  // namespace pcc

#endif

#endif  //~__PCCHMLibVideoEncoderImpl_H__
