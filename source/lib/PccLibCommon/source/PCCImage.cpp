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

#include "PCCImage.h"
#include "MD5.h"

using namespace pcc;

template <typename T, size_t N>
void PCCImage<T, N>::resize( const size_t sizeU0, const size_t sizeV0, PCCCOLORFORMAT format ) {
  if ( format == PCCCOLORFORMAT::UNKNOWN ) {
    printf( "ERROR: can't allocated image of unknown format \n" );
    exit( -1 );
  }
  width_  = sizeU0;
  height_ = sizeV0;
  format_ = format;
  // printf( "Image resize: %zu x %zu format = %d sizeof( T ) = %zu \n", width_, height_, format_, sizeof( T ) );
  // fflush(stdout);
  const size_t size = width_ * height_;
  if ( format_ == PCCCOLORFORMAT::YUV420 ) {
    channels_[0].resize( size, 0 );
    channels_[1].resize( size >> 2, 0 );
    channels_[2].resize( size >> 2, 0 );
  } else {
    for ( auto& channel : channels_ ) { channel.resize( size, 0 ); }
  }
}

template <typename T, size_t N>
void PCCImage<T, N>::swap( PCCImage<T, N>& image ) {
  std::swap( width_, image.width_ );
  std::swap( height_, image.height_ );
  std::swap( format_, image.format_ );
  std::swap( deprecatedColorFormat_, image.deprecatedColorFormat_ );
  for ( size_t c = 0; c < N; c++ ) { channels_[c].swap( image.channels_[c] ); }
}

template <typename T, size_t N>
void PCCImage<T, N>::convertYUV420ToYUV444() {
  PCCImage<T, 3> image;
  image.convertYUV420ToYUV444( *this );
  swap( image );
}
template <typename T, size_t N>
void PCCImage<T, N>::convertYUV444ToYUV420() {
  PCCImage<T, 3> image;
  image.convertYUV444ToYUV420( *this );
  swap( image );
}

template <typename T, size_t N>
void PCCImage<T, N>::convertRGB2BGR() {
  channels_[0].swap( channels_[1] );
  channels_[1].swap( channels_[2] );
}
template <typename T, size_t N>
void PCCImage<T, N>::convertYUV444ToYUV420( const PCCImage<T, 3>& src ) {
  if ( src.getColorFormat() != PCCCOLORFORMAT::YUV444 ) {
    printf(
        "Error: convertYUV44ToYUV420 not possible from image of format = %d "
        "!= YUV444 \n",
        (int32_t)src.getColorFormat() );
    exit( -1 );
  }
  resize( src.getWidth(), src.getHeight(), PCCCOLORFORMAT::YUV420 );
  std::copy( src.channels_[0].begin(), src.channels_[0].end(), channels_[0].begin() );
  for ( size_t c = 1; c < N; ++c ) {
    for ( size_t y = 0, y2 = 0; y < height_; y += 2, y2 += 1 ) {
      const T* const buffer1 = src.channels_[c].data() + y * width_;
      const T* const buffer2 = buffer1 + width_;
      for ( size_t x = 0, x2 = y2 * ( width_ >> 1 ); x < width_; x += 2, x2 += 1 ) {
        const uint64_t sum = buffer1[x] + buffer1[x + 1] + buffer2[x] + buffer2[x + 1];
        channels_[c][x2]   = T( ( sum + 2 ) / 4 );
      }
    }
  }
}

template <typename T, size_t N>
void PCCImage<T, N>::convertYUV420ToYUV444( const PCCImage<T, 3>& image ) {
  if ( image.getColorFormat() != PCCCOLORFORMAT::YUV420 ) {
    printf(
        "Error: convertYUV44ToYUV420 not possible from image of format = %d "
        "!= YUV420 \n",
        (int32_t)image.getColorFormat() );
    exit( -1 );
  }
  resize( image.getWidth(), image.getHeight(), PCCCOLORFORMAT::YUV444 );
  std::copy( image.channels_[0].begin(), image.channels_[0].end(), channels_[0].begin() );
  const size_t width2 = width_ / 2;
  for ( size_t c = 1; c < N; ++c ) {
    auto&    dst = channels_[c];
    const T* src = image.channels_[c].data();
    for ( size_t y = 0; y < height_; y += 2 ) {
      T* const buffer = dst.data() + y * width_;
      for ( size_t x2 = 0; x2 < width2; ++x2, src++ ) {
        const size_t x = x2 * 2;
        buffer[x]      = *src;
        buffer[x + 1]  = *src;
      }
      memcpy( (char*)( buffer + width_ ), (char*)buffer, width_ * sizeof( T ) );
    }
  }
}

template <typename T, size_t N>
bool PCCImage<T, N>::write( std::ofstream& outfile, const size_t nbyte ) {
  printf( "Image write %zux%zu T = %zu nbyte = %zu color format = %d channel size = %zu %zu %zu \n", width_, height_,
          sizeof( T ), nbyte, format_, channels_[0].size(), channels_[1].size(), channels_[2].size() );
  fflush( stdout );
  if ( nbyte == sizeof( T ) ) {
    if ( !outfile.good() ) { return false; }
    for ( const auto& channel : channels_ ) {
      outfile.write( (const char*)( channel.data() ), channel.size() * sizeof( T ) );
    }
  } else {
    assert( nbyte < sizeof( T ) );
    PCCImage<uint8_t, 3> image;
    image = *this;
    image.write( outfile, nbyte );
  }
  return true;
}

template <typename T, size_t N>
bool PCCImage<T, N>::write( const std::string fileName, const size_t nbyte ) {
  std::ofstream outfile( fileName, std::ios::binary );
  if ( write( outfile, nbyte ) ) {
    outfile.close();
    return true;
  }
  return false;
}

template <typename T, size_t N>
bool PCCImage<T, N>::read( std::ifstream&       infile,
                           const size_t         sizeU0,
                           const size_t         sizeV0,
                           const PCCCOLORFORMAT format,
                           const size_t         nbyte ) {
  // printf( " read image %zu x %zu x %zu / %zu format = %d infile.eof() = %d \n", sizeU0, sizeV0, nbyte, sizeof( T ),
  // format, infile.eof() );
  if ( !infile.good() ) { return false; }
  if ( nbyte == sizeof( T ) ) {
    resize( sizeU0, sizeV0, format );
    for ( auto& channel : channels_ ) {
      infile.read( (char*)( channel.data() ), channel.size() * sizeof( T ) );
      if ( !infile.good() ) { return false; }
    }
  } else {
    assert( nbyte < sizeof( T ) );
    PCCImage<uint8_t, 3> image;
    if ( !image.read( infile, sizeU0, sizeV0, format, nbyte ) ) { return false; }
    *this = image;
  }
  return infile.good();
}

template <typename T, size_t N>
bool PCCImage<T, N>::read( const std::string    fileName,
                           const size_t         sizeU0,
                           const size_t         sizeV0,
                           const PCCCOLORFORMAT format,
                           const size_t         nbyte ) {
  std::ifstream infile( fileName, std::ios::binary );
  if ( read( infile, sizeU0, sizeV0, format, nbyte ) ) {
    infile.close();
    return true;
  }
  return false;
}

template <typename T, size_t N>
bool PCCImage<T, N>::copyBlock( size_t top, size_t left, size_t width, size_t height, PCCImage& block ) {
  assert( top >= 0 && left >= 0 && ( width + left ) <= width_ && ( height + top ) <= height_ );
  for ( size_t cc = 0; cc < N; cc++ ) {
    for ( size_t i = top; i < top + height; i++ ) {
      for ( size_t j = left; j < left + width; j++ ) {
        block.setValue( cc, ( j - left ), ( i - top ), getValue( cc, j, i ) );
      }
    }
  }
  return true;
}
template <typename T, size_t N>
bool PCCImage<T, N>::setBlock( size_t top, size_t left, PCCImage& block ) {
  assert( top >= 0 && left >= 0 && ( block.getWidth() + left ) < width_ && ( block.getHeight() + top ) < height_ );
  for ( size_t cc = 0; cc < N; cc++ ) {
    for ( size_t i = top; i < top + block.getHeight(); i++ ) {
      for ( size_t j = left; j < left + block.getWidth(); j++ ) {
        setValue( cc, j, i, block.getValue( cc, ( j - left ), ( i - top ) ) );
      }
    }
  }
  return true;
}
template <typename T, size_t N>
void PCCImage<T, N>::copyFrom( PCCImage& image ) {
  size_t       width         = ( std::min )( width_, image.width_ );
  size_t       height        = ( std::min )( height_, image.height_ );
  size_t       subsample     = format_ == YUV420 ? 2 : 1;
  const size_t strideSrc[3]  = {image.width_, image.width_ / subsample, image.width_ / subsample};
  const size_t strideDst[3]  = {width_, width_ / subsample, width_ / subsample};
  const size_t widthComp[3]  = {width, width / subsample, width / subsample};
  const size_t heightComp[3] = {height, height / subsample, height / subsample};
  for ( size_t c = 0; c < N; c++ ) {
    T* src = image.channels_[c].data();
    T* dst = channels_[c].data();
    for ( size_t v = 0; v < heightComp[c]; ++v, src += strideSrc[c], dst += strideDst[c] ) {
      for ( size_t u = 0; u < widthComp[c]; ++u ) { dst[u] = src[u]; }
    }
  }
}
template <typename T, size_t N>
void PCCImage<T, N>::copyRawData( PCCImage& image ) {
  size_t       width         = ( std::min )( width_, image.width_ );
  size_t       height        = ( std::min )( height_, image.height_ );
  size_t       subsample     = format_ == YUV420 ? 2 : 1;
  const size_t widthComp[3]  = {width, width / subsample, width / subsample};
  const size_t heightComp[3] = {height, height / subsample, height / subsample};
  for ( size_t c = 0; c < N; c++ ) {
    size_t size = widthComp[c] * heightComp[c];
    T*     src  = image.channels_[c].data();
    T*     dst  = channels_[c].data();
    for ( size_t v = 0; v < size; ++v ) { *( dst++ ) = *( src++ ); }
  }
}

template <typename T, size_t N>
void PCCImage<T, N>::convertBitdepth( uint8_t bitdepthInput, uint8_t bitdepthOutput, bool msbAlignFlag ) {
  if ( bitdepthInput > sizeof( T ) * 8 ) {
    std::cout << "Wrong bitdepth input parameter (" << bitdepthInput << " > " << sizeof( T ) * 8 << ")" << std::endl;
    exit( -1 );
  }
  if ( bitdepthOutput > sizeof( T ) * 8 ) {
    std::cout << "Wrong bitdepth output parameter (" << bitdepthOutput << " > " << sizeof( T ) * 8 << ")" << std::endl;
    exit( -1 );
  }
  int bitDiff = (int)bitdepthInput - (int)bitdepthOutput;
  if ( bitDiff >= 0 ) {
    if ( msbAlignFlag ) {
      for ( size_t cc = 0; cc < N; cc++ ) {
        for ( size_t h = 0; h < height_; h++ ) {
          for ( size_t w = 0; w < width_; w++ ) { setValue( cc, w, h, ( getValue( cc, w, h ) >> bitDiff ) ); }
        }
      }
    } else {
      for ( size_t cc = 0; cc < N; cc++ ) {
        for ( size_t h = 0; h < height_; h++ ) {
          for ( size_t w = 0; w < width_; w++ ) {
            setValue( cc, w, h, tMin( getValue( cc, w, h ), ( T )( ( 1 << bitdepthOutput ) - 1 ) ) );
          }
        }
      }
    }
  } else {
    if ( msbAlignFlag ) {
      for ( size_t cc = 0; cc < N; cc++ ) {
        for ( size_t h = 0; h < height_; h++ ) {
          for ( size_t w = 0; w < width_; w++ ) { setValue( cc, w, h, ( getValue( cc, w, h ) << ( -bitDiff ) ) ); }
        }
      }
    } else {
      // do nothing, the vaue is correct
    }
  }
}

template <typename T, size_t N>
void PCCImage<T, N>::trace() {
  size_t maxWidth  = 16;
  size_t maxHeight = 16;
  auto*  Y         = channels_[0].data();
  auto*  U         = channels_[1].data();
  auto*  V         = channels_[2].data();
  bool   yuv444    = format_ != PCCCOLORFORMAT::YUV420;
  size_t widthC    = yuv444 ? width_ : width_ / 2;
  size_t heightC   = yuv444 ? height_ : height_ / 2;
  printf( "Picture = %4zu x %4zud C = %4zu x %4zu\n", width_, height_, widthC, heightC );
  for ( size_t j = 0; j < std::min( maxHeight, height_ ); j++ ) {
    printf( "Y %4zu: ", j );
    for ( size_t i = 0; i < std::min( maxWidth, width_ ); i++ ) { printf( "%2x ", Y[j * width_ + i] ); }
    if ( !yuv444 ) {
      if ( j < heightC ) {
        printf( "  -  U %4zu: ", j );
        for ( size_t i = 0; i < std::min( maxWidth, widthC ); i++ ) { printf( "%2x ", U[j * widthC + i] ); }
      } else {
        printf( "  -  V %4zu: ", j - heightC );
        for ( size_t i = 0; i < std::min( maxWidth, widthC ); i++ ) {
          printf( "%2x ", V[( j - heightC ) * widthC + i] );
        }
      }
    } else {
      printf( "  -  U %4zu: ", j );
      for ( size_t i = 0; i < std::min( maxWidth, widthC ); i++ ) { printf( "%2x ", U[j * widthC + i] ); }
      printf( "  -  V %4zu: ", j );
      for ( size_t i = 0; i < std::min( maxWidth, widthC ); i++ ) { printf( "%2x ", V[(j)*widthC + i] ); }
    }
    printf( "\n" );
  }
}

template <typename T, size_t N>
bool PCCImage<T, N>::allPixelsEqualToZero() {
  for ( auto& channel : channels_ ) {
    for ( auto& e : channel ) {
      if ( e != 0 ) { return false; }
    }
  }
  return true;
}

template <typename T, size_t N>
std::string PCCImage<T, N>::computeMD5( size_t channel ) {
  MD5                  md5Hash;
  std::vector<uint8_t> vector;
  vector.resize( 16 );
  md5Hash.update( (uint8_t*)( channels_[channel].data() ), channels_[channel].size() * sizeof( T ) );
  md5Hash.finalize( vector.data() );
  char result[33];
  for ( size_t i = 0; i < 16; i++ ) { sprintf( result + 2 * i, "%02x", vector[i] ); }
  return std::string( result );
}

template <typename T, size_t N>
void PCCImage<T, N>::upsample( size_t rate ) {
  for ( size_t i = rate; i > 1; i /= 2 ) {
    PCCImage<T, 3> up;
    up.resize( width_ * 2, height_ * 2, format_ );
    for ( size_t c = 0; c < N; ++c ) {
      size_t width  = format_ != YUV420 || c == 0 ? width_ * 2 : width_ * 2 / 2;
      size_t height = format_ != YUV420 || c == 0 ? height_ * 2 : height_ * 2 / 2;
      T*     src    = channels_[c].data();
      T*     dst    = up.channels_[c].data();
      for ( size_t y = 0; y < height; y += 2, dst += width * 2 ) {
        for ( size_t x = 0; x < width; x += 2, src++ ) {
          dst[x]     = *src;
          dst[x + 1] = *src;
        }
        memcpy( (char*)( dst + width ), (char*)dst, width * sizeof( T ) );
      }
    }
    swap( up );
  }
}

template class pcc::PCCImage<uint8_t, 3>;
template class pcc::PCCImage<uint16_t, 3>;