
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
// #ifndef PCCVideoDecoder_h
// #define PCCVideoDecoder_h

// #include "PCCCommon.h"

// namespace pcc {

// class PCCBitstream;
// template <typename T, size_t N>
// class PCCVideo;
// typedef pcc::PCCVideo<uint8_t, 3> PCCVideo3B;

// class PCCVideoDecoder {
// public:
//   PCCVideoDecoder();
//   ~PCCVideoDecoder();

//   bool decompress( PCCVideo3B& video, const std::string &fileName,
//                    const size_t width, const size_t height,
//                    const size_t frameCount, PCCBitstream &bitstream,
//                    const std::string &decoderPath,
//                    const std::string &inverseColorSpaceConversionConfig = "",
//                    const std::string &colorSpaceConversionPath = "",
//                    const bool use444CodecIo = false );
// private:
 
  
// };
// }; //~namespace

// #endif /* PCCVideoDecoder_h */

#ifndef PCCVideoDecoder_h
#define PCCVideoDecoder_h

#include "PCCCommon.h"
#include "PCCBitstream.h"
#include "PCCSystem.h"
#include "PCCVideo.h"

#include "PCCVideoDecoder.h"

namespace pcc {

class PCCBitstream;

class PCCVideoDecoder {
 public:
  PCCVideoDecoder();
  ~PCCVideoDecoder();

  template <typename T>
  bool decompress(PCCVideo<T, 3> &video,
                  const std::string &fileName,
                  const size_t width,
                  const size_t height,
                  const size_t frameCount,
                  PCCBitstream &bitstream,
                  const std::string &decoderPath,
                  const std::string &inverseColorSpaceConversionConfig = "",
                  const std::string &colorSpaceConversionPath = "",
                  const bool use444CodecIo = false,
                  const size_t nbyte = 1,
                  const bool keepIntermediateFiles = false ) {
    uint32_t compressedBitstreamSize = 0;
    bitstream.read<uint32_t>(compressedBitstreamSize);
    const std::string binFileName = fileName + ".bin";
    const std::string yuvRecFileName = addVideoFormat(fileName + "_rec" + (use444CodecIo ? ".rgb" : ".yuv"),
                                                      width, height, !use444CodecIo);
    const std::string rgbRecFileName = addVideoFormat(fileName + "_rec.rgb", width, height);
    std::ofstream file(binFileName, std::ios::binary);
    const std::string format = use444CodecIo ? "444" : "420";
    if (!file.good()) {
      return false;
    }
    file.write(reinterpret_cast<char *>(bitstream.buffer()) + bitstream.size(),
               compressedBitstreamSize);
    file.close();
    bitstream += (uint64_t)compressedBitstreamSize;
    std::stringstream cmd;
    cmd << decoderPath << " --BitstreamFile=" << binFileName << " --ReconFile=" << yuvRecFileName;
    std::cout << cmd.str() << '\n';
    if (pcc::system(cmd.str().c_str())) {
      std::cout << "Error: can't run system command!" << std::endl;
      return false;
    }
    // todo: should use444CodecIo allow conversion to happen?
    if (inverseColorSpaceConversionConfig.empty() || colorSpaceConversionPath.empty() ||
        use444CodecIo) {
      if (use444CodecIo) {
        if (!video.read(yuvRecFileName, width, height, frameCount, nbyte)) {
          return false;
        }
      } else {
        if (!video.read420(yuvRecFileName, width, height, frameCount, nbyte)) {
          return false;
        }
      }
    } else {
      std::stringstream cmd;
      cmd << colorSpaceConversionPath << " -f " << inverseColorSpaceConversionConfig
          << " -p SourceFile=\"" << yuvRecFileName << "\" -p OutputFile=\"" << rgbRecFileName
          << "\" -p SourceWidth=" << width << " -p SourceHeight=" << height
          << " -p NumberOfFrames=" << frameCount;
      std::cout << cmd.str() << '\n';
      if (pcc::system(cmd.str().c_str())) {
        std::cout << "Error: can't run system command!" << std::endl;
        return false;
      }
      if (!video.read(rgbRecFileName, width, height, frameCount, nbyte)) {
        return false;
      }
    }
    if (!keepIntermediateFiles) {
      removeFile( binFileName    );
      removeFile( yuvRecFileName );
      removeFile( rgbRecFileName );
    }
    return true;
  }
 private:

};


}; //~namespace

#endif /* PCCVideoDecoder_h */
