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
#ifndef PCCInternalColorConverter_h
#define PCCInternalColorConverter_h

#include "PCCVirtualColorConverter.h"
#include "PCCVideo.h"

namespace pcc {

struct Filter {
  std::vector<float> data_;
  double             offset_;
  double             shift_;
};
struct Filter444to420 {
  Filter horizontal_;
  Filter vertical_;
};
struct Filter420to444 {
  Filter horizontal0_;
  Filter vertical0_;
  Filter horizontal1_;
  Filter vertical1_;
};

template <class T>
class PCCInternalColorConverter : public PCCVirtualColorConverter<T> {
 public:
  PCCInternalColorConverter();
  ~PCCInternalColorConverter();

  void convert( std::string        configuration,
                PCCVideo<T, 3>&    videoSrc,
                const std::string& externalPath = "",
                const std::string& fileName     = "" ) {
    PCCVideo<T, 3> videoDst;
    convert( configuration, videoSrc, videoDst, externalPath, fileName );
    videoSrc = videoDst;
  }

  void convert( std::string        configuration,
                PCCVideo<T, 3>&    videoSrc,
                PCCVideo<T, 3>&    videoDst,
                const std::string& externalPath = "",
                const std::string& fileName     = "" );

  void upsample( PCCVideo<T, 3>& video, size_t rate, size_t nbyte, size_t filter );
  void upsample( PCCImage<T, 3>& image, size_t rate, size_t nbyte, size_t filter );

 private:
  void extractParameters( std::string& configuration, std::string& config, int32_t& bitdepth, int32_t& filter );
  void convertRGB44ToYUV420( PCCVideo<T, 3>& videoSrc, PCCVideo<T, 3>& videoDst, size_t nbyte, size_t filter );
  void convertRGB44ToYUV420( PCCImage<T, 3>& imageSrc, PCCImage<T, 3>& imageDst, size_t nbyte, size_t filter );

  void convertRGB44ToYUV444( PCCVideo<T, 3>& videoSrc, PCCVideo<T, 3>& videoDst, size_t nbyte, size_t filter );
  void convertRGB44ToYUV444( PCCImage<T, 3>& imageSrc, PCCImage<T, 3>& imageDst, size_t nbyte, size_t filter );

  void convertYUV420ToYUV444( PCCVideo<T, 3>& videoSrc, PCCVideo<T, 3>& videoDst, size_t nbyte, size_t filter );
  void convertYUV420ToYUV444( PCCImage<T, 3>& imageSrc, PCCImage<T, 3>& imageDst, size_t nbyte, size_t filter );

  void convertYUV420ToRGB444( PCCVideo<T, 3>& videoSrc, PCCVideo<T, 3>& videoDst, size_t nbyte, size_t filter );
  void convertYUV420ToRGB444( PCCImage<T, 3>& imageSrc, PCCImage<T, 3>& imageDst, size_t nbyte, size_t filter );

  void convertYUV444ToRGB444( PCCVideo<T, 3>& videoSrc, PCCVideo<T, 3>& videoDst, size_t nbyte, size_t filter );
  void convertYUV444ToRGB444( PCCImage<T, 3>& imageSrc, PCCImage<T, 3>& imageDst, size_t nbyte, size_t filter );

  void RGBtoFloatRGB( const std::vector<T>& src, std::vector<float>& dst, const size_t nbyte ) const;

  void convertRGBToYUV( const std::vector<float>& R,
                        const std::vector<float>& G,
                        const std::vector<float>& B,
                        std::vector<float>&       Y,
                        std::vector<float>&       U,
                        std::vector<float>&       V ) const;

  T                   clamp( T v, T a, T b ) const { return ( ( v < a ) ? a : ( ( v > b ) ? b : v ) ); }
  int                 clamp( int v, int a, int b ) const { return ( ( v < a ) ? a : ( ( v > b ) ? b : v ) ); }
  float               clamp( float v, float a, float b ) const { return ( ( v < a ) ? a : ( ( v > b ) ? b : v ) ); }
  double              clamp( double v, double a, double b ) const { return ( ( v < a ) ? a : ( ( v > b ) ? b : v ) ); }
  static inline T     tMin( T a, T b ) { return ( ( a ) < ( b ) ) ? ( a ) : ( b ); }
  static inline float fMin( float a, float b ) { return ( ( a ) < ( b ) ) ? ( a ) : ( b ); }
  static inline float fMax( float a, float b ) { return ( ( a ) > ( b ) ) ? ( a ) : ( b ); }
  static inline float fClip( float x, float low, float high ) { return fMin( fMax( x, low ), high ); }

  // TODO: This currently can't handle 10-bit. A new parameter is needed.
  void floatYUVToYUV( const std::vector<float>& src, std::vector<T>& dst, const bool chroma, const size_t nbyte ) const;

  void YUVtoFloatYUV( const std::vector<T>& src,
                      std::vector<float>&   dst,
                      const bool            chroma,
                      const size_t          nbBytes ) const;

  void convertYUVToRGB( const std::vector<float>& Y,
                        const std::vector<float>& U,
                        const std::vector<float>& V,
                        std::vector<float>&       R,
                        std::vector<float>&       G,
                        std::vector<float>&       B ) const;

  void floatRGBToRGB( const std::vector<float>& src, std::vector<T>& dst, const size_t nbyte ) const;

  void downsampling( const std::vector<float>& chroma_in,
                     std::vector<float>&       chroma_out,
                     const int                 widthIn,
                     const int                 heightIn,
                     const int                 maxValue,
                     const size_t              filter ) const;

  void upsampling( const std::vector<float>& chromaIn,
                   std::vector<float>&       chromaOut,
                   const int                 widthIn,
                   const int                 heightIn,
                   const int                 maxValue,
                   const size_t              filter ) const;

  inline void copy( const std::vector<float>& src, std::vector<float>& dst ) const {
    size_t count = src.size();
    dst.resize( count );
    for ( size_t i = 0; i < count; i++ ) { dst[i] = src[i]; }
  }

  inline float downsamplingHorizontal( const Filter444to420&     filter,
                                       const std::vector<float>& im,
                                       const int                 width,
                                       const int                 height,
                                       const int                 i0,
                                       const int                 j0 ) const {
    const float scale    = 1.0f / ( (float)( 1 << ( (int)filter.horizontal_.shift_ ) ) );
    const float offset   = 0.00000000;
    const int   position = int( filter.horizontal_.data_.size() - 1 ) >> 1;
    double      value    = 0;
    for ( int j = 0; j < (int)filter.horizontal_.data_.size(); j++ ) {
      value +=
          (double)filter.horizontal_.data_[j] * (double)( im[i0 * width + clamp( j0 + j - position, 0, width - 1 )] );
    }
    return (float)( ( value + (double)offset ) * (double)scale );
  }

  inline float downsamplingVertical( const Filter444to420&     filter,
                                     const std::vector<float>& im,
                                     const int                 width,
                                     const int                 height,
                                     const int                 i0,
                                     const int                 j0 ) const {
    const float offset   = 0;
    const float scale    = 1.0f / ( (float)( 1 << ( (int)filter.vertical_.shift_ ) ) );
    const int   position = int( filter.vertical_.data_.size() - 1 ) >> 1;
    double      value    = 0;
    for ( int i = 0; i < (int)filter.vertical_.data_.size(); i++ ) {
      value +=
          (double)filter.vertical_.data_[i] * (double)( im[clamp( i0 + i - position, 0, height - 1 ) * width + j0] );
    }
    return (float)( ( value + (double)offset ) * (double)scale );
  }

  inline float upsamplingVertical0( const Filter420to444&     filter,
                                    const std::vector<float>& im,
                                    const int                 width,
                                    const int                 height,
                                    const int                 i0,
                                    const int                 j0 ) const {
    const float scale    = 1.0f / ( (float)( 1 << ( (int)filter.vertical0_.shift_ ) ) );
    const float offset   = 0.00000000;
    const int   position = int( filter.vertical0_.data_.size() + 1 ) >> 1;
    float       value    = 0;
    for ( int i = 0; i < (int)filter.vertical0_.data_.size(); i++ ) {
      value += filter.vertical0_.data_[i] * (float)( im[clamp( i0 + i - position, 0, height - 1 ) * width + j0] );
    }
    return (float)( ( value + (float)offset ) * (float)scale );
  }

  inline float upsamplingVertical1( const Filter420to444&     filter,
                                    const std::vector<float>& im,
                                    int                       width,
                                    int                       height,
                                    int                       i0,
                                    int                       j0 ) const {
    const float scale    = 1.0f / ( (float)( 1 << ( (int)filter.vertical1_.shift_ ) ) );
    const float offset   = 0.00000000;
    const int   position = int( filter.vertical1_.data_.size() + 1 ) >> 1;
    float       value    = 0;
    for ( int i = 0; i < (int)filter.vertical1_.data_.size(); i++ ) {
      value += filter.vertical1_.data_[i] * (float)( im[clamp( i0 + i - position, 0, height - 1 ) * width + j0] );
    }
    return (float)( ( value + (float)offset ) * (float)scale );
  }

  inline float upsamplingHorizontal0( const Filter420to444&     filter,
                                      const std::vector<float>& im,
                                      int                       width,
                                      int                       height,
                                      int                       i0,
                                      int                       j0 ) const {
    const float scale    = 1.0f / ( (float)( 1 << ( (int)filter.horizontal0_.shift_ ) ) );
    const float offset   = 0.00000000;
    const int   position = int( filter.horizontal0_.data_.size() + 1 ) >> 1;
    float       value    = 0;
    for ( int j = 0; j < (int)filter.horizontal0_.data_.size(); j++ ) {
      value += filter.horizontal0_.data_[j] * (float)( im[i0 * width + clamp( j0 + j - position, 0, width - 1 )] );
    }
    return (float)( ( value + (float)offset ) * (float)scale );
  }

  inline float upsamplingHorizontal1( const Filter420to444&     filter,
                                      const std::vector<float>& im,
                                      int                       width,
                                      int                       height,
                                      int                       i0,
                                      int                       j0 ) const {
    const float scale    = 1.0f / ( (float)( 1 << ( (int)filter.horizontal1_.shift_ ) ) );
    const float offset   = 0.00000000;
    const int   position = int( filter.horizontal1_.data_.size() + 1 ) >> 1;
    float       value    = 0;
    for ( int j = 0; j < (int)filter.horizontal1_.data_.size(); j++ ) {
      value += filter.horizontal1_.data_[j] * (float)( im[i0 * width + clamp( j0 + j - position, 0, width - 1 )] );
    }
    return (float)( ( value + (float)offset ) * (float)scale );
  }
};

};  // namespace pcc

#endif /* PCCInternalColorConverter_h */
