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
#include "PCCCommon.h"

#ifdef USE_HMAPP_VIDEO_CODEC

#include "PCCHMAppVideoDecoder.h"
#include "PccHevcParser.h"
#include "PCCSystem.h"

using namespace pcc;

template <typename T>
PCCHMAppVideoDecoder<T>::PCCHMAppVideoDecoder() {}
template <typename T>
PCCHMAppVideoDecoder<T>::~PCCHMAppVideoDecoder() {}

template <typename T>
void PCCHMAppVideoDecoder<T>::decode( PCCVideoBitstream& bitstream,
                                      PCCVideo<T, 3>&    video,
                                      size_t             outputBitDepth,
                                      const std::string& decoderPath,
                                      const std::string& fileName ) {
  if ( decoderPath.empty() || !exist( decoderPath ) ) {
    std::cerr << "decoderPath not set\n";
    exit( 1 );
  }
  size_t                  width = 0, height = 0, bitDepth = 0;
  bool                    isRGB = false;
  pcc_hevc::PccHevcParser hevcParser;
  hevcParser.getVideoSize( bitstream.vector(), width, height, bitDepth, isRGB );
  printf( "hevcParser= %zu x %zu %zu bits isRGB = %d \n", width, height, bitDepth, isRGB );
  const std::string binFileName = fileName + ".bin";
  const std::string reconFile =
      addVideoFormat( fileName + "_rec", width, height, !isRGB, !isRGB, outputBitDepth == 10 ? "10" : "8" );
  bitstream.write( binFileName );
  std::stringstream cmd;
  cmd << decoderPath << " --BitstreamFile=" << binFileName << " --ReconFile=" << reconFile;
  if ( isRGB ) {
    cmd << " --OutputColourSpaceConvert=GBRtoRGB";
    if ( outputBitDepth == 8 ) { cmd << " --OutputBitDepth=8 --OutputBitDepthC=8"; }
  } else {
    if ( outputBitDepth == 8 ) { cmd << " --OutputBitDepth=8 --OutputBitDepthC=8"; }
  }
  std::cout << cmd.str() << '\n';
  if ( pcc::system( cmd.str().c_str() ) ) {
    std::cout << "Error: can't run system command!" << std::endl;
    exit( -1 );
  }
  PCCCOLORFORMAT format = isRGB ? PCCCOLORFORMAT::RGB444 : PCCCOLORFORMAT::YUV420;
  video.clear();
  video.read( reconFile, width, height, format, outputBitDepth == 8 ? 1 : 2 );
  printf( "File read size = %zu x %zu frame count = %zu \n", video.getWidth(), video.getHeight(),
          video.getFrameCount() );

  removeFile( binFileName );
  removeFile( reconFile );
}

template class pcc::PCCHMAppVideoDecoder<uint8_t>;
template class pcc::PCCHMAppVideoDecoder<uint16_t>;

#endif
