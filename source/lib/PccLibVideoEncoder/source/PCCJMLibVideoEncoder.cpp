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

#ifdef USE_JMLIB_VIDEO_CODEC
#include "PCCVideo.h"
#include "PCCJMLibVideoEncoder.h"
#include "PCCJMLibVideoEncoderImpl.h"

using namespace pcc;

template <typename T>
PCCJMLibVideoEncoder<T>::PCCJMLibVideoEncoder() {}
template <typename T>
PCCJMLibVideoEncoder<T>::~PCCJMLibVideoEncoder() {}

template <typename T>
void PCCJMLibVideoEncoder<T>::encode( PCCVideo<T, 3>&            videoSrc,
                                      PCCVideoEncoderParameters& params,
                                      PCCVideoBitstream&         bitstream,
                                      PCCVideo<T, 3>&            videoRec ) {
  const size_t width      = videoSrc.getWidth();
  const size_t height     = videoSrc.getHeight();
  const size_t frameCount = videoSrc.getFrameCount();

  std::stringstream cmd;
  std::string       srcYuvFileName_ = removeFileExtension( params.srcYuvFileName_ ) + "_jmlib.bin";
  std::string       binFileName     = removeFileExtension( params.binFileName_ ) + "_jmlib.bin";
  std::string       recYuvFileName  = removeFileExtension( params.recYuvFileName_ ) + "_jmlib.bin";
  cmd << "JMEncoder";
  cmd << " -d " << params.encoderConfig_;
  if ( bitstream.type() == VIDEO_GEOMETRY && bitstream.type() != VIDEO_ATTRIBUTE ) {
    cmd << " -p QPPSlice=" << params.qp_;
    cmd << " -p QPISlice=" << params.qp_;
    cmd << " -p QPBSlice=" << params.qp_;
  }
  if ( bitstream.type() == VIDEO_ATTRIBUTE && bitstream.type() != VIDEO_GEOMETRY ) {
    cmd << " -p QPPSlice=" << params.qp_;
    cmd << " -p QPISlice=" << params.qp_;
    cmd << " -p QPBSlice=" << params.qp_;
  }
  cmd << " -p InputFile=" << srcYuvFileName_;
  cmd << " -p SourceBitDepthLuma=" << params.inputBitDepth_;
  cmd << " -p YUVFormat=" << ( params.use444CodecIo_ ? "3" : "1" );
  cmd << " -p FrameRate=30 ";
  cmd << " -p FrameSkip=0 ";
  cmd << " -p SourceWidth=" << width;
  cmd << " -p SourceHeight=" << height;
  cmd << " -p FramesToBeEncoded=" << frameCount;
  cmd << " -p OutputFile=" << binFileName;
  cmd << " -p ReconFile=" << recYuvFileName;
  cmd << " -p OutputBitDepthLuma=" << params.outputBitDepth_;
  cmd << " -p OutputBitDepthChroma=" << params.outputBitDepth_;

  std::cout << cmd.str() << std::endl;

  PCCJMLibVideoEncoderImpl<T> encoder;
  encoder.encode( videoSrc, cmd.str(), bitstream, videoRec );
}

template class pcc::PCCJMLibVideoEncoder<uint8_t>;
template class pcc::PCCJMLibVideoEncoder<uint16_t>;

#endif
