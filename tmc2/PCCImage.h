
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

#ifndef PCCImage_h
#define PCCImage_h

#include <limits>

namespace pcc {
template <typename T, size_t N>
class PCCImage {
 public:
  PCCImage() {
    width = 0;
    height = 0;
  }
  PCCImage(const PCCImage &) = default;
  PCCImage &operator=(const PCCImage &rhs) = default;
  ~PCCImage() = default;

  size_t getWidth() const { return width; }
  size_t getHeight() const { return height; }
  size_t getDepth() const { return sizeof(T) * 8; }
  size_t getChannelCount() const { return N; }

  void set(const T value = 0) {
    for (auto &channel : channels) {
      for (auto &p : channel) {
        p = value;
      }
    }
  }
  void resize(const size_t sizeU0, const size_t sizeV0) {
    width = sizeU0;
    height = sizeV0;
    const size_t size = sizeU0 * sizeV0;
    for (auto &channel : channels) {
      channel.resize(size);
    }
  }
  bool write420(std::ofstream &outfile) const {
    if (!outfile.good()) {
      return false;
    }
    outfile.write((const char *)(channels[0].data()), width * height * sizeof(T));
    std::vector<T> chroma;
    const size_t width2 = width / 2;
    const size_t byteCount2 = width2 * sizeof(T);
    chroma.resize(width2);
    for (size_t c = 1; c < N; ++c) {
      const auto &channel = channels[c];
      for (size_t y = 0; y < height; y += 2) {
        const T *const buffer1 = channel.data() + y * width;
        const T *const buffer2 = buffer1 + width;
        for (size_t x = 0; x < width; x += 2) {
          const size_t x2 = x / 2;
          const uint64_t sum = buffer1[x] + buffer1[x + 1] + buffer2[x] + buffer2[x + 1];
          chroma[x2] = T((sum + 2) / 4);
        }
        outfile.write((const char *)(chroma.data()), byteCount2);
      }
    }
    return true;
  }
  bool write(std::ofstream &outfile) const {
    if (!outfile.good()) {
      return false;
    }
    const size_t byteCount = width * height * sizeof(T);
    for (const auto &channel : channels) {
      outfile.write((const char *)(channel.data()), byteCount);
    }
    return true;
  }
  bool write(const std::string fileName) const {
    std::ofstream outfile(fileName, std::ios::binary);
    if (write(outfile)) {
      outfile.close();
      return true;
    }
    return false;
  }
  bool read420(std::ifstream &infile, const size_t sizeU0, const size_t sizeV0) {
    if (!infile.good()) {
      return false;
    }
    resize(sizeU0, sizeV0);
    infile.read((char *)(channels[0].data()), width * height * sizeof(T));
    std::vector<T> chroma;
    const size_t width2 = width / 2;
    const size_t byteCount1 = width * sizeof(T);
    const size_t byteCount2 = width2 * sizeof(T);
    chroma.resize(width2);
    for (size_t c = 1; c < N; ++c) {
      auto &channel = channels[c];
      for (size_t y = 0; y < height; y += 2) {
        infile.read((char *)(chroma.data()), byteCount2);
        T *const buffer1 = channel.data() + y * width;
        T *const buffer2 = buffer1 + width;
        for (size_t x2 = 0; x2 < width2; ++x2) {
          const size_t x = x2 * 2;
          const T value = chroma[x2];
          buffer1[x] = value;
          buffer1[x + 1] = value;
        }
        memcpy((char *)buffer2, (char *)buffer1, byteCount1);
      }
    }
    return true;
  }
  bool read(std::ifstream &infile, const size_t sizeU0, const size_t sizeV0) {
    if (!infile.good()) {
      return false;
    }
    resize(sizeU0, sizeV0);
    const size_t byteCount = width * height * sizeof(T);
    for (auto &channel : channels) {
      infile.read((char *)(channel.data()), byteCount);
    }
    return true;
  }
  bool read(const std::string fileName, const size_t sizeU0, const size_t sizeV0) {
    std::ifstream infile(fileName, std::ios::binary);
    if (read(infile, sizeU0, sizeV0)) {
      infile.close();
      return true;
    }
    return false;
  }
  void setValue(const size_t channelIndex, const size_t u, const size_t v, const T value) {
    assert(channelIndex < N && u < width && v < height);
    channels[channelIndex][v * width + u] = value;
  }
  T getValue(const size_t channelIndex, const size_t u, const size_t v) const {
    assert(channelIndex < N && u < width && v < height);
    return channels[channelIndex][v * width + u];
  }
  T &getValue(const size_t channelIndex, const size_t u, const size_t v) {
    assert(channelIndex < N && u < width && v < height);
    return channels[channelIndex][v * width + u];
  }
  static const size_t MaxValue = (std::numeric_limits<T>::max)();
  static const size_t HalfMaxValue = (MaxValue + 1) / 2;

 private:
  size_t width;
  size_t height;
  std::vector<T> channels[N];
};
}

#endif /* PCCImage_h */
