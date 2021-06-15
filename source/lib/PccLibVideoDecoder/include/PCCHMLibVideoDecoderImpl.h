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
#ifndef PCCHMlVideoDecoderImpl_h
#define PCCHMlVideoDecoderImpl_h

#include "PCCCommon.h"

#ifdef USE_HMLIB_VIDEO_CODEC

#include "PCCVideo.h"
#include "PCCVideoBitstream.h"

#include <TLibCommon/TComList.h>
#include <TLibCommon/TComPicYuv.h>
#include <TLibDecoder/AnnexBread.h>
#include <TLibDecoder/NALread.h>
#include <TLibDecoder/TDecTop.h>

namespace pcc {

template <class T>
class PCCHMLibVideoDecoderImpl {
 public:
  PCCHMLibVideoDecoderImpl();

  ~PCCHMLibVideoDecoderImpl();
  void decode( PCCVideoBitstream& bitstream, size_t outputBitDepth, PCCVideo<T, 3>& video );

 private:
  void               setVideoSize( const pcc_hm::TComSPS* sps );
  void               xWriteOutput( pcc_hm::TComList<pcc_hm::TComPic*>* pcListPic, uint32_t tId, PCCVideo<T, 3>& video );
  void               xFlushOutput( pcc_hm::TComList<pcc_hm::TComPic*>* pcListPic, PCCVideo<T, 3>& video );
  void               xWritePicture( const pcc_hm::TComPicYuv* pic, PCCVideo<T, 3>& video );
  pcc_hm::TDecTop*   m_pTDecTop;
  int                m_iPOCLastDisplay;
  int                m_iSkipFrame{};
  std::array<int, 2> m_outputBitDepth{};
  int                m_internalBitDepths;
  int                m_outputWidth;
  int                m_outputHeight;
  bool               m_bRGB2GBR;
};

};  // namespace pcc

#endif

#endif /* PCCVirtualVideoDecoder_h */
