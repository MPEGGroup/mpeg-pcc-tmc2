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

#ifndef PCCImage_h
#define PCCImage_h

#include "PCCCommon.h"

namespace pcc {
template <typename T, size_t N>
class PCCImage {
 public:
  PCCImage() : width_( 0 ), height_( 0 ) {}
  PCCImage(const PCCImage &) = default;
  PCCImage &operator=(const PCCImage &rhs) = default;
  ~PCCImage() = default;

  void clear(){
    for (auto &channel : channels_) {
      channel.clear();
    }
  }
  size_t getWidth() const { return width_; }
  size_t getHeight() const { return height_; }
  size_t getDepth() const { return sizeof(T) * 8; }
  size_t getChannelCount() const { return N; }

  void set(const T value = 0) {
    for (auto &channel : channels_) {
      for (auto &p : channel) {
        p = value;
      }
    }
  }
  void resize(const size_t sizeU0, const size_t sizeV0) {
    width_  = sizeU0;
    height_ = sizeV0;
    const size_t size = sizeU0 * sizeV0;
    for (auto &channel : channels_) {
      channel.resize(size);
    }
  }
  bool write420(std::ofstream &outfile, const size_t nbyte) const {
    if (!outfile.good()) {
      return false;
    }
    if (nbyte == 1) {
      std::vector<uint8_t> channels[N];
      for (size_t i = 0; i < N; i++) {
        channels[i].resize(channels_[i].size());
        copy(channels_[i].begin(), channels_[i].end(), channels[i].begin());
      }
      outfile.write((const char *)(channels[0].data()), width_ * height_);
      std::vector<uint8_t> chroma;
      const size_t width2 = width_ / 2;
      const size_t byteCount2 = width2;
      chroma.resize(width2);
      for (size_t c = 1; c < N; ++c) {
        const auto &channel = channels[c];
        for (size_t y = 0; y < height_; y += 2) {
          const uint8_t *const buffer1 = channel.data() + y * width_;
          const uint8_t *const buffer2 = buffer1 + width_;
          for (size_t x = 0; x < width_; x += 2) {
            const size_t x2 = x / 2;
            const uint64_t sum = buffer1[x] + buffer1[x + 1] + buffer2[x] + buffer2[x + 1];
            chroma[x2] = uint8_t((sum + 2) / 4);
          }
          outfile.write((const char *)(chroma.data()), byteCount2);
        }
      }
    } else {
      outfile.write((const char *)(channels_[0].data()), width_ * height_ * sizeof(T));
      std::vector<T> chroma;
      const size_t width2 = width_ / 2;
      const size_t byteCount2 = width2 * sizeof(T);
      chroma.resize(width2);
      for (size_t c = 1; c < N; ++c) {
        const auto &channel = channels_[c];
        for (size_t y = 0; y < height_; y += 2) {
          const T *const buffer1 = channel.data() + y * width_;
          const T *const buffer2 = buffer1 + width_;
          for (size_t x = 0; x < width_; x += 2) {
            const size_t x2 = x / 2;
            const uint64_t sum = buffer1[x] + buffer1[x + 1] + buffer2[x] + buffer2[x + 1];
            chroma[x2] = T((sum + 2) / 4);
          }
          outfile.write((const char *)(chroma.data()), byteCount2);
        }
      }
    }
    return true;
  }
  bool write(std::ofstream &outfile, const size_t nbyte) const {
    if (!outfile.good()) {
      return false;
    }
    if (nbyte == 1) {
      std::vector<uint8_t> channels[N];
      for (size_t i = 0; i < N; i++) {
        channels[i].resize(channels_[i].size());
        copy(channels_[i].begin(), channels_[i].end(), channels[i].begin());
      }
      const size_t byteCount = width_ * height_;
      for (const auto &channel : channels) {
        outfile.write((const char *)(channel.data()), byteCount);
      }
    } else {
      const size_t byteCount = width_ * height_ * sizeof(T);
      for (const auto &channel : channels_) {
        outfile.write((const char *)(channel.data()), byteCount);
      }
    }
    return true;
  }
  bool write(const std::string fileName, const size_t nbyte) const {
    std::ofstream outfile(fileName, std::ios::binary);
    if (write(outfile, nbyte)) {
      outfile.close();
      return true;
    }
    return false;
  }
  bool read420(std::ifstream &infile, const size_t sizeU0, const size_t sizeV0, const size_t nbyte) {
    if (!infile.good()) {
      return false;
    }
    resize(sizeU0, sizeV0);
    if (nbyte == 1) {
      std::vector<uint8_t> channels[N];
      for (size_t i = 0; i < N; i++) {
        channels[i].resize(channels_[i].size());
      }
      infile.read((char *)(channels[0].data()), width_ * height_);
      std::vector<uint8_t> chroma;
      const size_t width2 = width_ / 2;
      const size_t byteCount1 = width_;
      const size_t byteCount2 = width2;
      chroma.resize(width2);
      for (size_t c = 1; c < N; ++c) {
        auto &channel = channels[c];
        for (size_t y = 0; y < height_; y += 2) {
          infile.read((char *)(chroma.data()), byteCount2);
          uint8_t *const buffer1 = channel.data() + y * width_;
          uint8_t *const buffer2 = buffer1 + width_;
          for (size_t x2 = 0; x2 < width2; ++x2) {
            const size_t x = x2 * 2;
            const uint8_t value = chroma[x2];
            buffer1[x] = value;
            buffer1[x + 1] = value;
          }
          memcpy((char *)buffer2, (char *)buffer1, byteCount1);
        }
      }
      for (size_t i = 0; i < N; i++) {
        copy(channels[i].begin(), channels[i].end(), channels_[i].begin());
      }
    } else {
      infile.read((char *)(channels_[0].data()), width_ * height_ * sizeof(T));
      std::vector<T> chroma;
      const size_t width2 = width_ / 2;
      const size_t byteCount1 = width_ * sizeof(T);
      const size_t byteCount2 = width2 * sizeof(T);
      chroma.resize(width2);
      for (size_t c = 1; c < N; ++c) {
        auto &channel = channels_[c];
        for (size_t y = 0; y < height_; y += 2) {
          infile.read((char *)(chroma.data()), byteCount2);
          T *const buffer1 = channel.data() + y * width_;
          T *const buffer2 = buffer1 + width_;
          for (size_t x2 = 0; x2 < width2; ++x2) {
            const size_t x = x2 * 2;
            const T value = chroma[x2];
            buffer1[x] = value;
            buffer1[x + 1] = value;
          }
          memcpy((char *)buffer2, (char *)buffer1, byteCount1);
        }
      }
    }
    return true;
  }
  bool read(std::ifstream &infile, const size_t sizeU0, const size_t sizeV0, const size_t nbyte) {
    if (!infile.good()) {
      return false;
    }
    resize(sizeU0, sizeV0);
    if (nbyte == 1) {
      std::vector<uint8_t> channels[N];
      for (size_t i = 0; i < N; i++) {
        channels[i].resize(channels_[i].size());
      }
      const size_t byteCount = width_ * height_;
      for (auto &channel : channels) {
        infile.read((char *)(channel.data()), byteCount);
      }
      for (size_t i = 0; i < N; i++) {
        copy(channels[i].begin(), channels[i].end(), channels_[i].begin());
      }
    }
    else {
      const size_t byteCount = width_ * height_ * sizeof(T);
      for (auto &channel : channels_) {
        infile.read((char *)(channel.data()), byteCount);
      }
    }
    return true;
  }
  bool read(const std::string fileName, const size_t sizeU0, const size_t sizeV0, const size_t nbyte) {
    std::ifstream infile(fileName, std::ios::binary);
    if (read(infile, sizeU0, sizeV0, nbyte)) {
      infile.close();
      return true;
    }
    return false;
  }
  void setValue(const size_t channelIndex, const size_t u, const size_t v, const T value) {
    assert(channelIndex < N && u < width_ && v < height_);
    channels_[channelIndex][v * width_ + u] = value;
  }
  T getValue(const size_t channelIndex, const size_t u, const size_t v) const {
    assert(channelIndex < N && u < width_ && v < height_);
    return channels_[channelIndex][v * width_ + u];
  }
  T &getValue(const size_t channelIndex, const size_t u, const size_t v) {
    assert(channelIndex < N && u < width_ && v < height_);
    return channels_[channelIndex][v * width_ + u];
  }
  static const size_t MaxValue = (std::numeric_limits<T>::max)();
  static const size_t HalfMaxValue = (MaxValue + 1) / 2;

  bool write420(const std::string fileName, const size_t nbyte) const {
    std::ofstream outfile(fileName, std::ios::binary);
    if (write420(outfile, nbyte)) {
      outfile.close();
      return true;
    }
    return false;
  }  
  bool read420(const std::string fileName, const size_t sizeU0, const size_t sizeV0, const size_t nbyte) {
    std::ifstream infile(fileName, std::ios::binary);
    if (read420(infile, sizeU0, sizeV0, nbyte)) {
      infile.close();
      return true;
    }
    return false;
  } 
  bool copyBlock(size_t top, size_t left, size_t width, size_t height, PCCImage& block) {
    assert(top >= 0 && left >= 0 && (width + left) < width_ && (height + top) < height_);
    for (size_t cc = 0; cc < N; cc++) {
      for (size_t i = top; i < top + height; i++) {
        for (size_t j = left; j < left + width; j++) {
          block.setValue(cc, (j - left), (i - top), getValue(cc, j, i));
        }
      }
    }
    return true;
  }
  bool setBlock(size_t top, size_t left, PCCImage& block) {
    assert(top >= 0 && left >= 0 && (block.getWidth() + left) < width_ && (block.getHeight() + top) < height_);
    for (size_t cc = 0; cc < N; cc++) {
      for (size_t i = top; i < top + block.getHeight(); i++) {
        for (size_t j = left; j < left + block.getWidth(); j++) {
          setValue(cc, j, i, block.getValue(cc, (j - left), (i - top)));
        }
      }
    }
    return true;
  }
 private:
  size_t width_;
  size_t height_;
  std::vector<T> channels_[N];
};
}

#endif /* PCCImage_h */
