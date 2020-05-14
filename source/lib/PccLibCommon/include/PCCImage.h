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
  PCCImage() : width_( 0 ), height_( 0 ), format_( PCCCOLORFORMAT::UNKNOWN ), deprecatedColorFormat_( 0 ) {}
  PCCImage( const PCCImage& ) = default;
  PCCImage& operator=( const PCCImage& rhs ) = default;
  ~PCCImage()                                = default;
  std::vector<T>& operator[]( int index ) { return channels_[index]; }

  void swap( PCCImage<T, N>& image ) {
    std::swap( width_, image.width_ );
    std::swap( height_, image.height_ );
    std::swap( format_, image.format_ );
    std::swap( deprecatedColorFormat_, image.deprecatedColorFormat_ );
    for ( size_t c = 0; c < N; c++ ) { channels_[c].swap( image.channels_[c] ); }
  }
  template <typename FromT>
  PCCImage<T, 3>& operator=( const PCCImage<FromT, 3>& image ) {
    resize( image.getWidth(), image.getHeight(), image.getColorFormat() );
    deprecatedColorFormat_ = image.getDeprecatedColorFormat();
    for ( size_t c = 0; c < 3; ++c ) {
      size_t size = channels_[c].size();
      auto&  src  = image.getChannel( c );
      auto&  dst  = channels_[c];
      for ( size_t i = 0; i < size; ++i ) { dst[i] = static_cast<T>( src[i] ); }
    }
    return *this;
  }

  void convertYUV444ToYUV420( const PCCImage<T, 3>& image ) {
    if ( image.getColorFormat() != PCCCOLORFORMAT::YUV444 ) {
      printf( "Error: convertYUV44ToYUV420 not possible from image of format = %d != YUV444 \n",
              (int32_t)image.getColorFormat() );
      exit( -1 );
    }
    resize( image.getWidth(), image.getHeight(), PCCCOLORFORMAT::YUV420 );
    std::copy( image.channels_[0].begin(), image.channels_[0].end(), channels_[0].begin() );
    for ( size_t c = 1; c < N; ++c ) {
      const auto& channel = image.channels_[c];
      for ( size_t y = 0; y < height_; y += 2 ) {
        const T* const buffer1 = channel.data() + y * width_;
        const T* const buffer2 = buffer1 + width_;
        for ( size_t x = 0; x < width_; x += 2 ) {
          const size_t   x2  = x / 2;
          const uint64_t sum = buffer1[x] + buffer1[x + 1] + buffer2[x] + buffer2[x + 1];
          channels_[c][x2]   = T( ( sum + 2 ) / 4 );
        }
      }
    }
  }

  void convertYUV420ToYUV444( const PCCImage<T, 3>& image ) {
    if ( image.getColorFormat() != PCCCOLORFORMAT::YUV420 ) {
      printf( "Error: convertYUV44ToYUV420 not possible from image of format = %d != YUV420 \n",
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
        memcpy( (char*)( buffer + width_ ), (char*)buffer, width2 * sizeof( T ) );
      }
    }
  }

  void convertYUV420ToYUV444() {
    PCCImage<T, 3> image;
    image.convertYUV420ToYUV444( *this );
    this->swap( image );
  }
  void convertYUV444ToYUV420() {
    PCCImage<T, 3> image;
    image.convertYUV444ToYUV420( *this );
    this->swap( image );
  }
  void clear() {
    for ( auto& channel : channels_ ) { channel.clear(); }
  }
  size_t                getWidth() const { return width_; }
  size_t                getHeight() const { return height_; }
  PCCCOLORFORMAT        getColorFormat() const { return format_; }
  size_t                getDepth() const { return sizeof( T ) * 8; }
  size_t                getChannelCount() const { return N; }
  size_t                getDeprecatedColorFormat() const { return deprecatedColorFormat_; }
  void                  setDeprecatedColorFormat( size_t value ) { deprecatedColorFormat_ = value; }
  const std::vector<T>& getChannel( size_t index ) const { return channels_[index]; }
  std::vector<T>&       getChannel( size_t index ) { return channels_[index]; }
  void                  set( const T value = 0 ) {
    for ( auto& channel : channels_ ) {
      for ( auto& p : channel ) { p = value; }
    }
  }
  void resize( const size_t sizeU0, const size_t sizeV0, PCCCOLORFORMAT format ) {
    if ( format == PCCCOLORFORMAT::UNKNOWN ) {
      printf( "ERROR: can't allocated image of unknown format \n" );
      exit( -1 );
    }
    width_  = sizeU0;
    height_ = sizeV0;
    format_ = format;
    printf( "Image resize: %zu x %zu format = %d \n", width_, height_, format_ );
    const size_t size = width_ * height_;
    if ( format_ == PCCCOLORFORMAT::YUV420 ) {
      channels_[0].resize( size );
      channels_[1].resize( size >> 2 );
      channels_[2].resize( size >> 2 );
    } else {
      for ( auto& channel : channels_ ) { channel.resize( size ); }
    }
  }

  template <typename Pel>
  void set( const Pel*     Y,
            const Pel*     U,
            const Pel*     V,
            size_t         widthY,
            size_t         heightY,
            size_t         strideY,
            size_t         widthC,
            size_t         heightC,
            size_t         strideC,
            int16_t        shiftbits,
            PCCCOLORFORMAT format,
            bool           rgb2bgr ) {
    resize( widthY, heightY, format );
    const Pel*   ptr[2][3] = {{Y, U, V}, {V, Y, U}};
    const size_t width[3]  = {widthY, widthC, widthC};
    const size_t height[3] = {heightY, heightC, heightC};
    const size_t stride[3] = {strideY, strideC, strideC};
    int16_t      rounding  = 1 << ( shiftbits - 1 );
    printf( "copy image from HM to PCC: S=%d R=%d (%4zux%4zu S=%4zu C:%4zux%4zu => %4zux%4zu) bgr=%d \n", shiftbits,
            rounding, widthY, heightY, strideY, widthC, heightC, width_, height_, rgb2bgr );
    for ( size_t c = 0; c < 3; c++ ) {
      auto* src = ptr[rgb2bgr][c];
      auto* dst = channels_[c].data();
      if ( shiftbits > 0 ) {
        T minval = 0;
        T maxval = ( T )( ( 1 << ( 10 - (int)shiftbits ) ) - 1 );
        for ( size_t v = 0; v < height[c]; ++v, src += stride[c], dst += width[c] ) {
          for ( size_t u = 0; u < width[c]; ++u ) {
            dst[u] = clamp( ( T )( ( src[u] + rounding ) >> shiftbits ), minval, maxval );
          }
        }
      } else {
        for ( size_t v = 0; v < height[c]; ++v, src += stride[c], dst += width[c] ) {
          for ( size_t u = 0; u < width[c]; ++u ) { dst[u] = (T)src[u]; }
        }
      }
    }
  }

  template <typename Pel>
  void get( Pel*    Y,
            Pel*    U,
            Pel*    V,
            size_t  widthY,
            size_t  heightY,
            size_t  strideY,
            size_t  widthC,
            size_t  heightC,
            size_t  strideC,
            int16_t shiftbits,
            bool    rgb2bgr ) {
    size_t chromaSubsample = widthY / widthC;
    if ( ( chromaSubsample == 1 && format_ == PCCCOLORFORMAT::YUV420 ) ||
         ( chromaSubsample == 2 && format_ != PCCCOLORFORMAT::YUV420 ) ) {
      printf( "Error: image get not possible from image of format = %d with chromaSubsample = %zu \n", (int32_t)format_,
              chromaSubsample );
      exit( -1 );
    }
    size_t       widthChroma  = width_ / chromaSubsample;
    size_t       heightChroma = height_ / chromaSubsample;
    Pel*         ptr[2][3]    = {{Y, U, V}, {V, Y, U}};
    const size_t width[3]     = {width_, widthChroma, widthChroma};
    const size_t heightSrc[3] = {height_, heightChroma, heightChroma};
    const size_t heightDst[3] = {heightY, heightC, heightC};
    const size_t stride[3]    = {strideY, strideC, strideC};
    printf( "copy image from PCC to HM: S = %d (%4zux%4zu => %4zux%4zu S=%4zu C: %4zux%4zu ) \n", shiftbits, width_,
            height_, widthY, heightY, strideY, widthC, heightC );
    for ( size_t c = 0; c < 3; c++ ) {
      auto* src = channels_[c].data();
      auto* dst = ptr[rgb2bgr][c];
      if ( shiftbits > 0 ) {
        for ( size_t v = 0; v < heightSrc[c]; ++v, src += width[c], dst += stride[c] ) {
          for ( size_t u = 0; u < width[c]; ++u ) { dst[u] = ( Pel )( src[u] ) << shiftbits; }
        }
      } else {
        for ( size_t v = 0; v < heightSrc[c]; ++v, src += width[c], dst += stride[c] ) {
          for ( size_t u = 0; u < width[c]; ++u ) { dst[u] = (Pel)src[u]; }
        }
      }
      for ( size_t v = heightSrc[c]; v < heightDst[c]; ++v, dst += stride[c] ) {
        for ( size_t u = 0; u < width[c]; ++u ) { dst[u] = 0; }
      }
    }
  }

  bool write( std::ofstream& outfile, const size_t nbyte ) {
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

  bool write( const std::string fileName, const size_t nbyte ) {
    std::ofstream outfile( fileName, std::ios::binary );
    if ( write( outfile, nbyte ) ) {
      outfile.close();
      return true;
    }
    return false;
  }

  bool read( std::ifstream&       infile,
             const size_t         sizeU0,
             const size_t         sizeV0,
             const PCCCOLORFORMAT format,
             const size_t         nbyte ) {
    printf( " read image %zu x %zu x %zu / %zu format = %d infile.eof() = %d \n", sizeU0, sizeV0, nbyte, sizeof( T ),
            format, infile.eof() );
    if ( infile.eof() ) {
      printf( "Read image eof found return false \n" );
      return false;
    }
    if ( nbyte == sizeof( T ) ) {
      resize( sizeU0, sizeV0, format );
      for ( auto& channel : channels_ ) { infile.read( (char*)( channel.data() ), channel.size() * sizeof( T ) ); }
    } else {
      assert( nbyte < sizeof( T ) );
      PCCImage<uint8_t, 3> image;
      image.read( infile, sizeU0, sizeV0, format, nbyte );
      *this = image;
    }
    return true;
  }

  bool read( const std::string    fileName,
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

  void setValue( const size_t channelIndex, const size_t u, const size_t v, const T value ) {
    assert( channelIndex < N && u < width_ && v < height_ );
    channels_[channelIndex][v * width_ + u] = value;
  }
  T getValue( const size_t channelIndex, const size_t u, const size_t v ) const {
    assert( channelIndex < N && u < width_ && v < height_ );
    return channels_[channelIndex][v * width_ + u];
  }
  T& getValue( const size_t channelIndex, const size_t u, const size_t v ) {
    assert( channelIndex < N && u < width_ && v < height_ );
    return channels_[channelIndex][v * width_ + u];
  }

  bool copyBlock( size_t top, size_t left, size_t width, size_t height, PCCImage& block ) {
    assert( top >= 0 && left >= 0 && ( width + left ) < width_ && ( height + top ) < height_ );
    for ( size_t cc = 0; cc < N; cc++ ) {
      for ( size_t i = top; i < top + height; i++ ) {
        for ( size_t j = left; j < left + width; j++ ) {
          block.setValue( cc, ( j - left ), ( i - top ), getValue( cc, j, i ) );
        }
      }
    }
    return true;
  }
  bool setBlock( size_t top, size_t left, PCCImage& block ) {
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

  static inline T tMin( T a, T b ) { return ( ( a ) < ( b ) ) ? ( a ) : ( b ); }
  // static inline float fMin( float a, float b ) { return ( ( a ) < ( b ) ) ? ( a ) : ( b ); }
  // static inline float fMax( float a, float b ) { return ( ( a ) > ( b ) ) ? ( a ) : ( b ); }
  // static inline float fClip( float x, float low, float high ) { return fMin( fMax( x, low ), high ); }

  void convertBitdepth( uint8_t bitdepthInput, uint8_t bitdepthOutput, bool msbAlignFlag ) {
    if ( bitdepthInput > sizeof( T ) * 8 ) {
      std::cout << "Wrong bitdepth input parameter (" << bitdepthInput << " > " << sizeof( T ) * 8 << ")" << std::endl;
      exit( -1 );
    }
    if ( bitdepthOutput > sizeof( T ) * 8 ) {
      std::cout << "Wrong bitdepth output parameter (" << bitdepthOutput << " > " << sizeof( T ) * 8 << ")"
                << std::endl;
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

 private:
  T      clamp( T v, T a, T b ) const { return ( ( v < a ) ? a : ( ( v > b ) ? b : v ) ); }
  int    clamp( int v, int a, int b ) const { return ( ( v < a ) ? a : ( ( v > b ) ? b : v ) ); }
  float  clamp( float v, float a, float b ) const { return ( ( v < a ) ? a : ( ( v > b ) ? b : v ) ); }
  double clamp( double v, double a, double b ) const { return ( ( v < a ) ? a : ( ( v > b ) ? b : v ) ); }

  size_t         width_;
  size_t         height_;
  std::vector<T> channels_[N];
  PCCCOLORFORMAT format_;
  size_t         deprecatedColorFormat_;  // 0.RGB 1.YUV420 2.YUV444 16bits  // TODO JR: must be removed
};
}  // namespace pcc

#endif /* PCCImage_h */
