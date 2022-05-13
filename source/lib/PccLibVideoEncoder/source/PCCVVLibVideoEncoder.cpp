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
#include "PCCCommon.h"

#ifdef USE_VVLIB_VIDEO_CODEC
#include "PCCVideo.h"
#include "PCCVVLibVideoEncoder.h"
#include "PCCVVLibVideoEncoderImpl.h"
#include "PCCVVLibVideoDecoder.h"

using namespace pcc;

template <typename T>
PCCVVLibVideoEncoder<T>::PCCVVLibVideoEncoder() {}
template <typename T>
PCCVVLibVideoEncoder<T>::~PCCVVLibVideoEncoder() {}

template <typename T>
void PCCVVLibVideoEncoder<T>::encode( PCCVideo<T, 3>&            videoSrc,
                                      PCCVideoEncoderParameters& params,
                                      PCCVideoBitstream&         bitstream,
                                      PCCVideo<T, 3>&            videoRec ) {
   std::stringstream cmd;
  cmd << "./bin/Release/vvencFFapp";
  cmd << " -c " << params.encoderConfig_;
  cmd << " --InputFile=" << params.srcYuvFileName_;
  cmd << " --ReconFile=" << params.recYuvFileName_;
  cmd << " --BitstreamFile=" << params.binFileName_;
  cmd << " --Size=" << videoSrc.getWidth() << "x" << videoSrc.getHeight();
  cmd << " --InputBitDepth=" << params.inputBitDepth_;
  cmd << " --OutputBitDepth=" << params.outputBitDepth_;
  cmd << " --FramesToBeEncoded=" << videoSrc.getFrameCount();
  cmd << " --fps=30";
  cmd << " --frameskip=0";  
  cmd << " --bitrate=0";  // constant qp
  cmd << " --QP=" << params.qp_;
  if ( params.internalBitDepth_ != 0 ) { cmd << " --InternalBitDepth=" << params.internalBitDepth_; }
  cmd << " --ConformanceWindowMode=1 ";

  // if ( params.transquantBypassEnable_ != 0 ) { cmd << " --TransquantBypassEnable=1"; }
  // if ( params.cuTransquantBypassFlagForce_ != 0 ) { cmd << " --CUTransquantBypassFlagForce=1"; }
  // if ( params.inputColourSpaceConvert_ ) { cmd << " --InputColourSpaceConvert=RGBtoGBR"; }

  std::cout << cmd.str() << std::endl;

  PCCVVLibVideoEncoderImpl<T> encoder;
  encoder.encode( videoSrc, cmd.str(), bitstream, videoRec );
}

template class pcc::PCCVVLibVideoEncoder<uint8_t>;
template class pcc::PCCVVLibVideoEncoder<uint16_t>;

#endif
