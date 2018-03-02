
/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * <OWNER> = Apple Inc.
 * <ORGANIZATION> = Apple Inc.
 * <YEAR> = 2017
 *
 * Copyright (c) 2017, Apple Inc.
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
 *  * Neither the name of the <ORGANIZATION> nor the names of its contributors may
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

#ifndef PCCVideo_h
#define PCCVideo_h

#include <string>

#include "PCCImage.h"
#include "PCCMisc.h"

#include "pcc_system.h"

namespace pcc {
template <typename T, size_t N>
class PCCVideo {
 public:
  PCCVideo() = default;
  PCCVideo(const PCCVideo &) = default;
  PCCVideo &operator=(const PCCVideo &rhs) = default;
  ~PCCVideo() = default;

  PCCImage<T, N> &getFrame(const size_t index) {
    assert(index < frames.size());
    return frames[index];
  }
  const PCCImage<T, N> &getFrame(const size_t index) const {
    assert(index < frames.size());
    return frames[index];
  }
  void resize(const size_t frameCount) { frames.resize(frameCount); }
  bool write(std::ofstream &outfile) const {
    for (const auto &frame : frames) {
      if (!frame.write(outfile)) {
        return false;
      }
    }
    return true;
  }
  bool write(const std::string fileName) {
    std::ofstream outfile(fileName, std::ios::binary);
    if (write(outfile)) {
      outfile.close();
      return true;
    }
    return false;
  }
  bool write420(std::ofstream &outfile) const {
    for (const auto &frame : frames) {
      if (!frame.write420(outfile)) {
        return false;
      }
    }
    return true;
  }
  bool write420(const std::string fileName) {
    std::ofstream outfile(fileName, std::ios::binary);
    if (write420(outfile)) {
      outfile.close();
      return true;
    }
    return false;
  }
  bool read(std::ifstream &infile, const size_t sizeU0, const size_t sizeV0,
            const size_t frameCount) {
    frames.resize(frameCount);
    for (auto &frame : frames) {
      if (!frame.read(infile, sizeU0, sizeV0)) {
        return false;
      }
    }
    return true;
  }
  bool read(const std::string fileName, const size_t sizeU0, const size_t sizeV0,
            const size_t frameCount) {
    std::ifstream infile(fileName, std::ios::binary);
    if (read(infile, sizeU0, sizeV0, frameCount)) {
      infile.close();
      return true;
    }
    return false;
  }
  bool read420(std::ifstream &infile, const size_t sizeU0, const size_t sizeV0,
               const size_t frameCount) {
    frames.resize(frameCount);
    for (auto &frame : frames) {
      if (!frame.read420(infile, sizeU0, sizeV0)) {
        return false;
      }
    }
    return true;
  }
  bool read420(const std::string fileName, const size_t sizeU0, const size_t sizeV0,
               const size_t frameCount) {
    std::ifstream infile(fileName, std::ios::binary);
    if (read420(infile, sizeU0, sizeV0, frameCount)) {
      infile.close();
      return true;
    }
    return false;
  }
  bool compress(const std::string &fileName, const size_t qp, PCCBitstream &bitstream,
                const std::string &encoderConfig, const std::string &encoderPath,
                const std::string &colorSpaceConversionConfig = "",
                const std::string &colorSpaceConversionPath = "") {
    if (frames.empty()) {
      return false;
    }
    const size_t width = frames[0].getWidth();
    const size_t height = frames[0].getHeight();
    const size_t depth = frames[0].getDepth();
    if (frames[0].getChannelCount() != 3) {
      return false;
    }
    const std::string yuvFileName = fileName + ".yuv";
    if (colorSpaceConversionConfig.empty() || colorSpaceConversionPath.empty()) {
      if (!write420(yuvFileName)) {
        return false;
      }
    } else {
      const std::string rgbFileName = fileName + ".rgb";
      if (!write(rgbFileName)) {
        return false;
      }
      std::stringstream cmd;
      cmd << colorSpaceConversionPath << " -f " << colorSpaceConversionConfig << " -p SourceFile=\""
          << rgbFileName << "\" -p OutputFile=\"" << yuvFileName << "\" -p SourceWidth=" << width
          << " -p SourceHeight=" << height << " -p NumberOfFrames=" << getFrameCount();
      std::cout << cmd.str() << '\n';
      if (pcc::system(cmd.str().c_str())) {
        std::cout << "Error: can't run system command!" << std::endl;
        return false;
      }
    }

    const std::string binFileName = fileName + ".bin";
    const std::string recFileName = fileName + "_rec.yuv";
    std::stringstream cmd;
    cmd << encoderPath << " -c " << encoderConfig << " -i " << yuvFileName
        << " --InputBitDepth=" << depth
        << " --InputChromaFormat=420 --FrameRate=30 --FrameSkip=0 --SourceWidth=" << width
        << " --SourceHeight=" << height << " --FramesToBeEncoded=" << frames.size()
        << " --BitstreamFile=" << binFileName << " --ReconFile=" << recFileName << " --QP=" << qp;
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
    pcc::PCCWriteToBuffer<uint32_t>(uint32_t(fileSize), bitstream.buffer, bitstream.size);
    assert(bitstream.size + fileSize < bitstream.capacity);
    file.clear();
    file.seekg(0);
    file.read(reinterpret_cast<char *>(bitstream.buffer) + bitstream.size, fileSize);
    bitstream.size += fileSize;
    return true;
  }
  bool decompress(const std::string &fileName, const size_t width, const size_t height,
                  const size_t frameCount, pcc::PCCBitstream &bitstream,
                  const std::string &decoderPath,
                  const std::string &inverseColorSpaceConversionConfig = "",
                  const std::string &colorSpaceConversionPath = "") {
    uint32_t compressedBitstreamSize = 0;
    pcc::PCCReadFromBuffer<uint32_t>(bitstream.buffer, compressedBitstreamSize, bitstream.size);
    const std::string binFileName = fileName + ".bin";
    const std::string yuvRecFileName = fileName + "_rec.yuv";
    const std::string rgbRecFileName = fileName + "_rec.rgb";
    std::ofstream file(binFileName, std::ios::binary);
    if (!file.good()) {
      return false;
    }
    file.write(reinterpret_cast<char *>(bitstream.buffer) + bitstream.size,
               compressedBitstreamSize);
    file.close();
    bitstream.size += compressedBitstreamSize;
    std::stringstream cmd;
    cmd << decoderPath << " --BitstreamFile=" << binFileName << " --ReconFile=" << yuvRecFileName;
    std::cout << cmd.str() << '\n';
    if (pcc::system(cmd.str().c_str())) {
      std::cout << "Error: can't run system command!" << std::endl;
      return false;
    }

    if (inverseColorSpaceConversionConfig.empty() || colorSpaceConversionPath.empty()) {
      if (!read420(yuvRecFileName, width, height, frameCount)) {
        return false;
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
      if (!read(rgbRecFileName, width, height, frameCount)) {
        return false;
      }
    }
    return true;
  }
  size_t getWidth() const { return frames.empty() ? 0 : frames[0].getWidth(); }
  size_t getHeight() const { return frames.empty() ? 0 : frames[0].height(); }
  size_t getFrameCount() const { return frames.size(); }

 private:
  std::vector<PCCImage<T, N> > frames;
};
}

#endif /* PCCVideo_h */
