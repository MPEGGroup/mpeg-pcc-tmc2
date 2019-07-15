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

#ifndef PCCVideo_h
#define PCCVideo_h

#include <string>

#include "PCCImage.h"

namespace pcc {
template <typename T, size_t N>
class PCCVideo {
 public:
  PCCVideo() = default;
  PCCVideo(const PCCVideo &) = default;
  PCCVideo &operator=(const PCCVideo &rhs) = default;
  ~PCCVideo() = default;

  void clear() {
    for (auto &frame : frames_) {
      frame.clear();
    }
    frames_.clear();
  }
  std::vector<PCCImage<T, N> >& getFrames() { return frames_; }

  PCCImage<T, N> &getFrame(const size_t index) {
    assert(index < frames_.size());
    return frames_[index];
  }
  const PCCImage<T, N> &getFrame(const size_t index) const {
    assert(index < frames_.size());
    return frames_[index];
  }
  void resize(const size_t frameCount) { frames_.resize(frameCount); }
  bool write(std::ofstream &outfile, const size_t nbyte) const {
    for (const auto &frame : frames_) {
      if (!frame.write(outfile, nbyte)) {
        return false;
      }
    }
    return true;
  }
  bool write(const std::string fileName, const size_t nbyte) {
    std::ofstream outfile(fileName, std::ios::binary);
    if (write(outfile, nbyte)) {
      outfile.close();
      return true;
    }
    return false;
  }

  bool write420( std::ofstream& outfile, const size_t nbyte, bool convert, const size_t filter ) const {
    for ( const auto& frame : frames_ ) {
      if ( !frame.write420( outfile, nbyte, convert, filter ) ) { return false; }
    }
    return true;
  }

  bool write420( const std::string fileName, const size_t nbyte, const bool convert, const size_t filter ) {
    std::ofstream outfile( fileName, std::ios::binary );
    if ( write420( outfile, nbyte, convert, filter ) ) {
      outfile.close();
      return true;
    }
    return false;
  }

  bool write420( std::ofstream& outfile, const size_t nbyte ) const {
    for ( const auto& frame : frames_ ) {
      if ( !frame.write420( outfile, nbyte ) ) { return false; }
    }
    return true;
  }
  bool write420(const std::string fileName, const size_t nbyte) {
    std::ofstream outfile(fileName, std::ios::binary);
    if (write420(outfile, nbyte)) {
      outfile.close();
      return true;
    }
    return false;
  }
  bool read(std::ifstream &infile, const size_t sizeU0, const size_t sizeV0,
            const size_t frameCount, const size_t nbyte) {
    frames_.resize(frameCount);
    for (auto &frame : frames_) {
      if (!frame.read(infile, sizeU0, sizeV0, nbyte)) {
        return false;
      }
    }
    return true;
  }
  bool read(const std::string fileName, const size_t sizeU0, const size_t sizeV0,
            const size_t frameCount, const size_t nbyte) {
    std::ifstream infile(fileName, std::ios::binary);
    if (read(infile, sizeU0, sizeV0, frameCount, nbyte)) {
      infile.close();
      return true;
    }
    return false;
  }
  bool read420( std::ifstream& infile,
                const size_t   sizeU0,
                const size_t   sizeV0,
                const size_t   frameCount,
                const size_t   nbyte,
                const bool     convert,
                const size_t   filter ) {
    frames_.resize( frameCount );
    for ( auto& frame : frames_ ) {
      if ( !frame.read420( infile, sizeU0, sizeV0, nbyte, convert, filter ) ) { return false; }
    }
    return true;
  }
  bool read420( const std::string fileName,
                const size_t      sizeU0,
                const size_t      sizeV0,
                const size_t      frameCount,
                const size_t      nbyte,
                bool              convert,
                const int         filter ) {
    std::ifstream infile( fileName, std::ios::binary );
    if ( read420( infile, sizeU0, sizeV0, frameCount, nbyte, convert, filter ) ) {
      infile.close();
      return true;
    }
    return false;
  }

  bool read420( std::ifstream& infile,
                const size_t   sizeU0,
                const size_t   sizeV0,
                const size_t   frameCount,
                const size_t   nbyte ) {
    frames_.resize( frameCount );
    for ( auto& frame : frames_ ) {
      if ( !frame.read420( infile, sizeU0, sizeV0, nbyte ) ) { return false; }
    }
    return true;
  }
  bool read420( const std::string fileName,
                const size_t      sizeU0,
                const size_t      sizeV0,
                const size_t      frameCount,
                const size_t      nbyte ) {
    std::ifstream infile(fileName, std::ios::binary);
    if (read420(infile, sizeU0, sizeV0, frameCount, nbyte)) {
      infile.close();
      return true;
    }
    return false;
  }
  size_t getWidth() const { return frames_.empty() ? 0 : frames_[0].getWidth(); }
  size_t getHeight() const { return frames_.empty() ? 0 : frames_[0].getHeight(); }
  size_t getFrameCount() const { return frames_.size(); }

 private:
  std::vector<PCCImage<T, N> > frames_;
};

}

#endif /* PCCVideo_h */
