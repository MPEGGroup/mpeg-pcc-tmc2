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
  PCCVideo()                  = default;
  PCCVideo( const PCCVideo& ) = default;
  PCCVideo& operator=( const PCCVideo& rhs ) = default;
  ~PCCVideo()                                = default;
  void resize( const size_t frameCount ) { frames_.resize( frameCount ); }
  void clear() {
    for ( auto& frame : frames_ ) { frame.clear(); }
    frames_.clear();
  }

  typename std::vector<PCCImage<T, N> >::iterator begin() { return frames_.begin(); }
  typename std::vector<PCCImage<T, N> >::iterator end() { return frames_.end(); }
  const size_t                                    size() { return frames_.size(); }
  PCCImage<T, N>&                                 operator[]( int index ) { return frames_[index]; }
  void                                            setDeprecatedColorFormat( size_t value ) {
    for ( auto& f : frames_ ) f.setDeprecatedColorFormat( value );
  }
  std::vector<PCCImage<T, N> >& getFrames() { return frames_; }
  PCCImage<T, N>&               getFrame( const size_t index ) {
    assert( index < frames_.size() );
    return frames_[index];
  }
  const PCCImage<T, N>& getFrame( const size_t index ) const {
    assert( index < frames_.size() );
    return frames_[index];
  }
  void   swap( PCCVideo<T, N>& video ) { frames_.swap( video.frames_ ); }
  size_t getWidth() const { return frames_.empty() ? 0 : frames_[0].getWidth(); }
  size_t getHeight() const { return frames_.empty() ? 0 : frames_[0].getHeight(); }
  bool   is420() const { return getColorFormat() == PCCCOLORFORMAT::YUV420; }
  bool   is444() const {
    return getColorFormat() == PCCCOLORFORMAT::RGB444 || getColorFormat() == PCCCOLORFORMAT::YUV444;
  }
  bool isYUV() const {
    return getColorFormat() == PCCCOLORFORMAT::YUV420 || getColorFormat() == PCCCOLORFORMAT::YUV444;
  }
  PCCCOLORFORMAT getColorFormat() const {
    return frames_.empty() ? PCCCOLORFORMAT::UNKNOWN : frames_[0].getColorFormat();
  }
  size_t getFrameCount() const { return frames_.size(); }

  bool write( const std::string fileName, const size_t nbyte );
  bool read( const std::string    fileName,
             const size_t         sizeU0,
             const size_t         sizeV0,
             const PCCCOLORFORMAT format,
             const size_t         nbyte );

#if defined( WIN32 )
  bool _write( const std::string fileName, const size_t nbyte );
  bool _read( const std::string    fileName,
              const size_t         sizeU0,
              const size_t         sizeV0,
              const PCCCOLORFORMAT format,
              const size_t         nbyte );
#endif

  void        convertBitdepth( uint8_t bitdepthInput, uint8_t bitdepthOutput, bool msbAlignFlag );
  void        convertYUV420ToYUV444();
  void        convertYUV444ToYUV420();
  bool        allPixelsEqualToZero();
  std::string addFormat( std::string filename, const char* bitdepth ) {
    return addVideoFormat( filename, getWidth(), getHeight(), isYUV(), is420(), bitdepth );
  }

  void upsample( size_t rate );

 private:
  bool write( std::ofstream& outfile, const size_t nbyte );
  bool read( std::ifstream&       infile,
             const size_t         sizeU0,
             const size_t         sizeV0,
             const PCCCOLORFORMAT format,
             const size_t         nbyte );

  std::vector<PCCImage<T, N> > frames_;
};

}  // namespace pcc

#endif /* PCCVideo_h */
