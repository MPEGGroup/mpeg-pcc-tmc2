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

  void resize( const size_t sizeU0, const size_t sizeV0, PCCCOLORFORMAT format );

  void clear() {
    for ( auto& channel : channels_ ) { channel.clear(); }
  }
  size_t                getWidth() const { return width_; }
  size_t                getHeight() const { return height_; }
  PCCCOLORFORMAT        getColorFormat() const { return format_; }
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
  void swap( PCCImage<T, N>& image );
  void convertRGB2BGR();
  void convertYUV420ToYUV444();
  void convertYUV444ToYUV420();
  void convertYUV444ToYUV420( const PCCImage<T, 3>& image );
  void convertYUV420ToYUV444( const PCCImage<T, 3>& image );
  void upsample( size_t rate );

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
    // printf(
    //     "copy image PCC: Shift=%d Round=%d (%4zux%4zu S=%4zu C:%4zux%4zu => "
    //     "%4zux%4zu) stride = %4zu %4zu bgr=%d sizeof(Pel) = %zu sizeof(T) = %zu \n",
    //     shiftbits, rounding, widthY, heightY, strideY, widthC, heightC, width_, height_,
    //     strideY, strideC, rgb2bgr, sizeof(Pel), sizeof(T) );
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
      printf( "Error: image get not possible from image of format = %d with  chromaSubsample = %zu \n",
              (int32_t)format_, chromaSubsample );
      exit( -1 );
    }
    size_t       widthChroma  = width_ / chromaSubsample;
    size_t       heightChroma = height_ / chromaSubsample;
    Pel*         ptr[2][3]    = {{Y, U, V}, {V, Y, U}};
    const size_t width[3]     = {width_, widthChroma, widthChroma};
    const size_t heightSrc[3] = {height_, heightChroma, heightChroma};
    const size_t heightDst[3] = {heightY, heightC, heightC};
    const size_t stride[3]    = {strideY, strideC, strideC};
    printf( "copy image from PCC: Shift = %d (%4zux%4zu => %4zux%4zu S=%4zu C: %4zux%4zu ) \n", shiftbits, width_,
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

  bool write( std::ofstream& outfile, const size_t nbyte );

  bool write( const std::string fileName, const size_t nbyte );

  bool read( std::ifstream&       infile,
             const size_t         sizeU0,
             const size_t         sizeV0,
             const PCCCOLORFORMAT format,
             const size_t         nbyte );
  bool read( const std::string    fileName,
             const size_t         sizeU0,
             const size_t         sizeV0,
             const PCCCOLORFORMAT format,
             const size_t         nbyte );

  void setValue( const size_t channelIndex, const size_t u, const size_t v, const T value ) {
    assert( channelIndex < N && u < width_ && v < height_ );
    if ( format_ == YUV420 && channelIndex != 0 ) {
      channels_[channelIndex][( v >> 1 ) * ( width_ >> 1 ) + ( u >> 1 )] = value;
    } else {
      channels_[channelIndex][v * width_ + u] = value;
    }
  }
  void setValueYuvChroma( const size_t channelIndex, const size_t u, const size_t v, const T value ) {
    channels_[channelIndex][v * ( width_ >> 1 ) + u] = value;
  }

  T getValue( const size_t channelIndex, const size_t u, const size_t v ) const {
    assert( channelIndex < N && u < width_ && v < height_ );
    if ( format_ == YUV420 && channelIndex != 0 ) {
      return channels_[channelIndex][( v >> 1 ) * ( width_ >> 1 ) + ( u >> 1 )];
    } else {
      return channels_[channelIndex][v * width_ + u];
    }
  }
  T& getValue( const size_t channelIndex, const size_t u, const size_t v ) {
    assert( channelIndex < N && u < width_ && v < height_ );
    if ( format_ == YUV420 && channelIndex != 0 ) {
      return channels_[channelIndex][( v >> 1 ) * ( width_ >> 1 ) + ( u >> 1 )];
    } else {
      return channels_[channelIndex][v * width_ + u];
    }
  }

  bool copyBlock( size_t top, size_t left, size_t width, size_t height, PCCImage& block );
  bool setBlock( size_t top, size_t left, PCCImage& block );
  void copyFrom( PCCImage& image );
  void copyRawData( PCCImage& image );

  static inline T tMin( T a, T b ) { return ( ( a ) < ( b ) ) ? ( a ) : ( b ); }
  void            convertBitdepth( uint8_t bitdepthInput, uint8_t bitdepthOutput, bool msbAlignFlag );
  void            trace();
  bool            allPixelsEqualToZero();
  std::string     computeMD5( size_t channel );

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
