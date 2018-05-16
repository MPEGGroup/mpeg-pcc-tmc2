
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
#ifndef PCCVideoEncoder_h
#define PCCVideoEncoder_h

#include "PCCCommon.h"
#include "PCCBitstream.h"
#include "PCCSystem.h"
#include "PCCVideo.h"

namespace pcc {

class PCCBitstream;
class PCCContext;
class PCCFrameContext;

class PCCVideoEncoder {
 public:
  PCCVideoEncoder();
  ~PCCVideoEncoder();
  template <typename T>
  bool compress(PCCVideo<T, 3> &video,
                const std::string &fileName,
                const size_t qp,
                PCCBitstream &bitstream,
                const std::string &encoderConfig,
                const std::string &encoderPath,
                const std::string &colorSpaceConversionConfig = "",
                const std::string &inverseColorSpaceConversionConfig = "",
                const std::string &colorSpaceConversionPath = "",
                const bool use444CodecIo = false,
                const size_t nbyte = 1,
                const bool keepIntermediateFiles = false ) {
    auto& frames = video.getFrames();
    if (frames.empty()) {
      return false;
    }
    const size_t width = frames[0].getWidth();
    const size_t height = frames[0].getHeight();
    const size_t depth = nbyte == 1 ? 8 : 10;
    const size_t frameCount = video.getFrameCount();
    if (frames[0].getChannelCount() != 3) {
      return false;
    }
    const std::string format = use444CodecIo ? "444" : "420";
    const std::string binFileName = fileName + ".bin";
    const std::string srcYuvFileName = addVideoFormat(fileName + (use444CodecIo ? ".rgb" : ".yuv"), width, height, !use444CodecIo);
    const std::string srcRgbFileName = addVideoFormat(fileName + ".rgb", width, height);
    const std::string recYuvFileName = addVideoFormat(fileName + "_rec" + (use444CodecIo ? ".rgb" : ".yuv"), width, height, !use444CodecIo);
    const std::string recRgbFileName = addVideoFormat(fileName + "_rec" + ".rgb", width, height, !use444CodecIo);

    const bool yuvVideo = colorSpaceConversionConfig.empty() || colorSpaceConversionPath.empty() || use444CodecIo;
    // todo: should use444CodecIo allow conversion to happen?
    if( yuvVideo ) {
      if (use444CodecIo) {
        if (!video.write( srcYuvFileName, nbyte)) {
          return false;
        }
      } else {
        if (!video.write420( srcYuvFileName, nbyte)) {
          return false;
        }
      }
    } else {
      if (!video.write( srcRgbFileName, nbyte )) {
        return false;
      }
      std::stringstream cmd;
      cmd << colorSpaceConversionPath << " -f " << colorSpaceConversionConfig
          << " -p SourceFile=\"" << srcRgbFileName << "\""
          << " -p OutputFile=\"" << srcYuvFileName << "\""
          << " -p SourceWidth="  << width
          << " -p SourceHeight=" << height
          << " -p NumberOfFrames=" << frameCount;
      std::cout << cmd.str() << '\n';
      if (pcc::system(cmd.str().c_str())) {
        std::cout << "Error: can't run system command!" << std::endl;
        return false;
      }
    }

    std::stringstream cmd;
    cmd << encoderPath
        << " -c " << encoderConfig
        << " -i " << srcYuvFileName
        << " --InputBitDepth=" << depth
        << " --InternalBitDepth=" << depth
        << " --InputChromaFormat=" << format
        << " --FrameRate=30 "
        << " --FrameSkip=0 "
        << " --SourceWidth=" << width
        << " --SourceHeight=" << height
        << " --FramesToBeEncoded=" << frameCount
        << " --BitstreamFile=" << binFileName
        << " --ReconFile=" << recYuvFileName
        << " --QP=" << qp;
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
    bitstream.write<uint32_t>(uint32_t(fileSize));
    assert(bitstream.size() + fileSize < bitstream.capacity());
    file.clear();
    file.seekg(0);
    file.read(reinterpret_cast<char *>(bitstream.buffer()) + bitstream.size(), fileSize);
    bitstream += fileSize;

    if ( yuvVideo ) {
      if ( use444CodecIo ) {
        video.read( recYuvFileName, width, height, frameCount, nbyte );
      } else {
        video.read420( recYuvFileName, width, height, frameCount, nbyte );
      }
    } else {
      std::stringstream cmd;
      cmd << colorSpaceConversionPath
          << " -f " << inverseColorSpaceConversionConfig
          << " -p SourceFile=\"" << recYuvFileName << "\""
          << " -p OutputFile=\"" << recRgbFileName << "\""
          << " -p SourceWidth=" << width
          << " -p SourceHeight=" << height
          << " -p NumberOfFrames=" << frameCount;
      std::cout << cmd.str() << '\n';
      if (int ret = pcc::system(cmd.str().c_str())) {
        std::cout << "Error: can't run system command!" << std::endl;
        return ret;
      }
      video.read( recRgbFileName, width, height, frameCount, nbyte);
    }
    if( !keepIntermediateFiles ) {
      removeFile( binFileName    );
      removeFile( srcYuvFileName );
      removeFile( srcRgbFileName );
      removeFile( recYuvFileName );
      removeFile( recRgbFileName );
    }
    return true;
  }
 private:

};



}; //~namespace

#endif /* PCCVideoEncoder_h */
