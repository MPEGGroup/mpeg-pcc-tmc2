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

#ifdef USE_HMLIB_VIDEO_CODEC
#include "PCCVideo.h"
#include "PCCHMLibVideoEncoder.h"
#include "PCCHMLibVideoEncoderImpl.h"

using namespace pcc;

template <typename T>
PCCHMLibVideoEncoder<T>::PCCHMLibVideoEncoder() {}
template <typename T>
PCCHMLibVideoEncoder<T>::~PCCHMLibVideoEncoder() {}

template <typename T>
void PCCHMLibVideoEncoder<T>::encode( PCCVideo<T, 3>&            videoSrc,
                                      PCCVideoEncoderParameters& params,
                                      PCCVideoBitstream&         bitstream,
                                      PCCVideo<T, 3>&            videoRec ) {
  const size_t      width      = videoSrc.getWidth();
  const size_t      height     = videoSrc.getHeight();
  const size_t      frameCount = videoSrc.getFrameCount();
  std::stringstream cmd;
  cmd << "HMEncoder";
  cmd << " -c " << params.encoderConfig_;
  cmd << " --InputFile=" << params.srcYuvFileName_;
  cmd << " --InputBitDepth=" << params.inputBitDepth_;
  cmd << " --InputChromaFormat=" << ( params.use444CodecIo_ ? "444" : "420" );
  cmd << " --OutputBitDepth=" << params.outputBitDepth_;
  cmd << " --OutputBitDepthC=" << params.outputBitDepth_;
  cmd << " --FrameRate=30";
  cmd << " --FrameSkip=0";
  cmd << " --SourceWidth=" << width;
  cmd << " --SourceHeight=" << height;
  cmd << " --ConformanceWindowMode=1 ";
  cmd << " --FramesToBeEncoded=" << frameCount;
  cmd << " --BitstreamFile=" << params.binFileName_;
  cmd << " --ReconFile=" << params.recYuvFileName_;
  cmd << " --QP=" << params.qp_;
  if ( params.transquantBypassEnable_ != 0 ) { cmd << " --TransquantBypassEnable=1"; }
  if ( params.cuTransquantBypassFlagForce_ != 0 ) { cmd << " --CUTransquantBypassFlagForce=1"; }
  if ( params.internalBitDepth_ != 0 ) {
    cmd << " --InternalBitDepth=" << params.internalBitDepth_;
    cmd << " --InternalBitDepthC=" << params.internalBitDepth_;
  }
#if defined( PCC_ME_EXT ) && PCC_ME_EXT
  if ( params.usePccMotionEstimation_ ) {
    cmd << " --UsePccMotionEstimation=1";
    cmd << " --BlockToPatchFile=" << params.blockToPatchFile_;
    cmd << " --OccupancyMapFile=" << params.occupancyMapFile_;
    cmd << " --PatchInfoFile=" << params.patchInfoFile_;
  }
#endif
#if defined( PCC_RDO_EXT ) && PCC_RDO_EXT
  if ( params.usePccRDO_ && !params.inputColourSpaceConvert_ ) {
    cmd << " --UsePccRDO=1";
    cmd << " --OccupancyMapFile=" << params.occupancyMapFile_;
  }
#endif

  if ( params.inputColourSpaceConvert_ ) { cmd << " --InputColourSpaceConvert=RGBtoGBR"; }
  std::cout << cmd.str() << std::endl;

  PCCHMLibVideoEncoderImpl<T> encoder;
  encoder.encode( videoSrc, cmd.str(), bitstream, videoRec );
}

template class pcc::PCCHMLibVideoEncoder<uint8_t>;
template class pcc::PCCHMLibVideoEncoder<uint16_t>;

#endif
