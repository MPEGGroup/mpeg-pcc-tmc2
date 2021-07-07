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

#ifndef __PCCVTMLibVideoEncoderImpl_H__
#define __PCCVTMLibVideoEncoderImpl_H__

#include "PCCCommon.h"

#ifdef USE_VTMLIB_VIDEO_CODEC
#include "PCCVideo.h"
#include "PCCVideoBitstream.h"

#include <list>
#include <ostream>
#include "EncoderLib/EncLib.h"
#include "Utilities/VideoIOYuv.h"
#include "PCCVTMLibVideoEncoderCfg.h"
#include <list>
#include <fstream>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <assert.h>
#include <iomanip>
#include "EncoderLib/EncTemporalFilter.h"
#if EXTENSION_360_VIDEO
#include "AppEncHelper360/TExt360AppEncTop.h"
#endif
#include "Utilities/program_options_lite.h"

namespace pcc {

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder application class
template <class T>
class PCCVTMLibVideoEncoderImpl : public PCCVTMLibVideoEncoderCfg, public AUWriterIf {
 public:
  PCCVTMLibVideoEncoderImpl( ostream& bitStream, EncLibCommon* encLibCommon );

  ~PCCVTMLibVideoEncoderImpl();

  bool encode( PCCVideo<T, 3>&    videoSrc,
               std::string        arguments,
               PCCVideoBitstream& bitstream,
               PCCVideo<T, 3>&    videoRec );
  bool encodePrep( bool& eos, PCCVideo<T, 3>& videoSrc, std::string arguments, PCCVideo<T, 3>& videoRec );
  void createLib( const int layerIdx );
  void destroyLib();

 private:
  void xInitLibCfg();
  void xInitLib();
  void xDestroyLib();
  void xWriteOutput( std::ostream&           bitstreamFile,
                     int                     iNumEncoded,
                     std::list<PelUnitBuf*>& recBufList,
                     PCCVideo<T, 3>&         videoRec );
  void xWritePicture( const PelUnitBuf* pic, PCCVideo<T, 3>& video );
  void xReadPicture( PelUnitBuf* pic, PCCVideo<T, 3>& video, int frameIndex );
  void printChromaFormat();
  void printRateSummary();
  void rateStatsAccum( const AccessUnit& au, const std::vector<uint32_t>& stats );
  void outputAU( const AccessUnit& au );

  EncLib                 m_cEncLib;
  std::list<PelUnitBuf*> m_recBufList;
  int                    m_iFrameRcvd;
  uint32_t               m_essentialBytes;
  uint32_t               m_totalBytes;
  int                    m_outputWidth;
  int                    m_outputHeight;
  int                    m_numEncoded;
  PelStorage*            m_trueOrgPic;
  PelStorage*            m_orgPic;
  PelStorage*            m_filteredOrgPic;
  EncTemporalFilter      m_temporalFilter;
  bool                   m_flush;
  ostream&               m_bitstream;
};

}  // namespace pcc

#endif

#endif  //~__PCCVTMLibVideoEncoderImpl_H__
