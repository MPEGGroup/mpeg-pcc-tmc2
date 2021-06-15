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

#include "PCCHMAppVideoEncoder.h"
#include "PCCSystem.h"

#ifdef USE_HMAPP_VIDEO_CODEC

using namespace pcc;

template <typename T>
PCCHMAppVideoEncoder<T>::PCCHMAppVideoEncoder() {}
template <typename T>
PCCHMAppVideoEncoder<T>::~PCCHMAppVideoEncoder() {}

template <typename T>
void PCCHMAppVideoEncoder<T>::encode( PCCVideo<T, 3>&            videoSrc,
                                      PCCVideoEncoderParameters& params,
                                      PCCVideoBitstream&         bitstream,
                                      PCCVideo<T, 3>&            videoRec ) {
  const size_t width          = videoSrc.getWidth();
  const size_t height         = videoSrc.getHeight();
  const size_t frameCount     = videoSrc.getFrameCount();
  std::string  srcYuvFileName = params.srcYuvFileName_;
  std::string  recYuvFileName = params.recYuvFileName_;
  std::string  binFileName    = params.binFileName_;
  srcYuvFileName.insert( srcYuvFileName.find_last_of( "." ), "_hmapp" );
  recYuvFileName.insert( recYuvFileName.find_last_of( "." ), "_hmapp" );
  binFileName.insert( binFileName.find_last_of( "." ), "_hmapp" );
  std::stringstream cmd;
  cmd << params.encoderPath_;
  cmd << " -c " << params.encoderConfig_;
  cmd << " --InputFile=" << srcYuvFileName;
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
  cmd << " --BitstreamFile=" << binFileName;
  cmd << " --ReconFile=" << recYuvFileName;
  cmd << " --QP=" << params.qp_;
  if ( params.transquantBypassEnable_ != 0 ) { cmd << " --TransquantBypassEnable=1"; }
  if ( params.cuTransquantBypassFlagForce_ != 0 ) { cmd << " --CUTransquantBypassFlagForce=1"; }
  if ( params.internalBitDepth_ != 0 ) {
    cmd << " --InternalBitDepth=" << params.internalBitDepth_;
    cmd << " --InternalBitDepthC=" << params.internalBitDepth_;
  }
  if ( params.usePccMotionEstimation_ ) {
    cmd << " --UsePccMotionEstimation=1";
    cmd << " --BlockToPatchFile=" << params.blockToPatchFile_;
    cmd << " --OccupancyMapFile=" << params.occupancyMapFile_;
    cmd << " --PatchInfoFile=" << params.patchInfoFile_;
  }
  if ( params.inputColourSpaceConvert_ ) { cmd << " --InputColourSpaceConvert=RGBtoGBR"; }

  std::cout << cmd.str() << std::endl;

  videoSrc.write( srcYuvFileName, params.inputBitDepth_ == 8 ? 1 : 2 );
  if ( pcc::system( cmd.str().c_str() ) ) {
    std::cout << "Error: can't run system command!" << std::endl;
    exit( -1 );
  }
  PCCCOLORFORMAT format = getColorFormat( params.recYuvFileName_ );
  videoRec.clear();
  videoRec.read( recYuvFileName, width, height, format, params.outputBitDepth_ == 8 ? 1 : 2 );
  bitstream.read( binFileName );
  removeFile( srcYuvFileName );
  removeFile( recYuvFileName );
  removeFile( binFileName );
}

template <typename T>
PCCCOLORFORMAT PCCHMAppVideoEncoder<T>::getColorFormat( std::string& name ) {
  if ( ( name.find( "_p444.rgb" ) ) != std::string::npos ) {
    return PCCCOLORFORMAT::RGB444;
  } else if ( ( name.find( "_p444.yuv" ) ) != std::string::npos ) {
    return PCCCOLORFORMAT::YUV444;
  } else if ( ( name.find( "_p420.yuv" ) ) != std::string::npos ) {
    return PCCCOLORFORMAT::YUV420;
  } else {
    printf( "PCCHMAppVideoEncoder can't find parameters %s \n", name.c_str() );
    exit( -1 );
  }
  return PCCCOLORFORMAT::UNKNOWN;
}

template class pcc::PCCHMAppVideoEncoder<uint8_t>;
template class pcc::PCCHMAppVideoEncoder<uint16_t>;

#endif  //~USE_HMAPP_VIDEO_CODEC
