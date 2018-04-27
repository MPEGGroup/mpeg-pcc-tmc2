
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
#include "PCCBitstream.h"
#include "PCCSystem.h"
#include "PCCVideo.h"

#include "PCCVideoEncoder.h"

using namespace pcc;

PCCVideoEncoder::PCCVideoEncoder() {

}
PCCVideoEncoder::~PCCVideoEncoder() {

}
bool PCCVideoEncoder::compress( PCCVideo3B& video, const std::string &fileName,
                                const size_t qp, PCCBitstream &bitstream,
                                const std::string &encoderConfig, const std::string &encoderPath,
                                const std::string &colorSpaceConversionConfig,
                                const std::string &colorSpaceConversionPath,
                                const bool use444CodecIo) {
  auto& frames = video.getFrames();
  if (frames.empty()) {
    return false;
  }
  const size_t width  = frames[0].getWidth();
  const size_t height = frames[0].getHeight();
  const size_t depth  = frames[0].getDepth();
  if (frames[0].getChannelCount() != 3) {
    return false;
  }
  const std::string format = use444CodecIo ? "444" : "420";
  const std::string yuvFileName = addVideoFormat( fileName + ( use444CodecIo ? ".rgb" : ".yuv" ), width, height, !use444CodecIo );

  // todo: should use444CodecIo allow conversion to happen?
  if (colorSpaceConversionConfig.empty() || colorSpaceConversionPath.empty() || use444CodecIo) {
    if (use444CodecIo) {
      if (!video.write(yuvFileName)) {
        return false;
      }
    } else {
      if (!video.write420(yuvFileName)) {
        return false;
      }
    }
  } else {
    const std::string rgbFileName =  addVideoFormat( fileName + ".rgb", width, height );
    if (!video.write(rgbFileName)) {
      return false;
    }
    std::stringstream cmd;
    cmd << colorSpaceConversionPath << " -f " << colorSpaceConversionConfig << " -p SourceFile=\""
        << rgbFileName << "\" -p OutputFile=\"" << yuvFileName << "\" -p SourceWidth=" << width
        << " -p SourceHeight=" << height << " -p NumberOfFrames=" << video.getFrameCount();
    std::cout << cmd.str() << '\n';
    if (pcc::system(cmd.str().c_str())) {
      std::cout << "Error: can't run system command!" << std::endl;
      return false;
    }
  }

  const std::string binFileName = fileName + ".bin";
  const std::string recFileName = addVideoFormat( fileName + "_rec" + ( use444CodecIo ? ".rgb" : ".yuv" ), width, height, !use444CodecIo );
  std::stringstream cmd;
  cmd << encoderPath << " -c " << encoderConfig << " -i " << yuvFileName
      << " --InputBitDepth=" << depth << " --InternalBitDepth=" << depth
      << " --InputChromaFormat=" << format
      << " --FrameRate=30 --FrameSkip=0 --SourceWidth=" << width << " --SourceHeight=" << height
      << " --FramesToBeEncoded=" << frames.size() << " --BitstreamFile=" << binFileName
      << " --ReconFile=" << recFileName << " --QP=" << qp;
  std::cout << cmd.str() << '\n';
  if (pcc::system(cmd.str().c_str())) {
    std::cout << "Error: can't run system command!" << std::endl;
    return false;
  }

  std::ifstream file(binFileName, std::ios::binary | std::ios::ate);
  if (!file.good()) {
    return false;
  }
  const uint64_t fileSize = file.tellg();
  bitstream.write<uint32_t>(uint32_t(fileSize) );
  assert(bitstream.size() + fileSize < bitstream.capacity() );
  file.clear();
  file.seekg(0);
  file.read(reinterpret_cast<char *>(bitstream.buffer()) + bitstream.size(), fileSize);
  bitstream += fileSize;
  return true;
}
