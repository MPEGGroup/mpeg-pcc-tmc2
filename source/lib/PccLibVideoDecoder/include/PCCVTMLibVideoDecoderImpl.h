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
#ifndef PCCVirtualVideoDecoderImpl_h
#define PCCVirtualVideoDecoderImpl_h

#include "PCCCommon.h"

#ifdef USE_VTMLIB_VIDEO_CODEC

#include "PCCVideo.h"
#include "PCCVideoBitstream.h"
#include "PCCVTMLibVideoDecoderCfg.h"

#include <CommonLib/Picture.h>
#include <DecoderLib/AnnexBread.h>
#include <DecoderLib/NALread.h>
#include <DecoderLib/DecLib.h>

namespace pcc {

template <class T>
class PCCVTMLibVideoDecoderImpl : public PCCVTMLibVideoDecoderCfg {
 public:
  PCCVTMLibVideoDecoderImpl();

  ~PCCVTMLibVideoDecoderImpl();
  uint32_t decode( PCCVideoBitstream& bitstream, size_t outputBitDepth, PCCVideo<T, 3>& video );

 private:
  void   xCreateDecLib();
  void   xDestroyDecLib();
  void   setVideoSize( const SPS* sps );
  void   xWriteOutput( PicList* pcListPic, uint32_t tId, PCCVideo<T, 3>& video );
  void   xFlushOutput( PicList* pcListPic, PCCVideo<T, 3>& video, const int layerId = NOT_VALID );
  void   xWritePicture( const Picture* pic, PCCVideo<T, 3>& video );
  DecLib m_cDecLib;
  int    m_iPOCLastDisplay = -MAX_INT;
  int    m_iSkipFrame{};
  std::array<int, MAX_NUM_CHANNEL_TYPE> m_outputBitDepth{};
  int                                   m_internalBitDepths;
  int                                   m_outputWidth;
  int                                   m_outputHeight;
  bool                                  m_bRGB2GBR;
  bool                                  m_newCLVS[MAX_NUM_LAYER_IDS];
  std::ofstream                         m_seiMessageFileStream;

  SEIAnnotatedRegions::AnnotatedRegionHeader                     m_arHeader;   ///< AR header
  std::map<uint32_t, SEIAnnotatedRegions::AnnotatedRegionObject> m_arObjects;  ///< AR object pool
  std::map<uint32_t, std::string>                                m_arLabels;   ///< AR label pool

  bool xIsNaluWithinTargetDecLayerIdSet( const InputNALUnit* nalu ) const;
  bool xIsNaluWithinTargetOutputLayerIdSet( const InputNALUnit* nalu ) const;
  bool isNewPicture( std::istream* bitstreamFile, class InputByteStream* bytestream );
  bool isNewAccessUnit( bool newPicture, std::istream* bitstreamFile, class InputByteStream* bytestream );

  void xOutputAnnotatedRegions( PicList* pcListPic );
};

};  // namespace pcc

#endif

#endif /* PCCVirtualVideoDecoder_h */
