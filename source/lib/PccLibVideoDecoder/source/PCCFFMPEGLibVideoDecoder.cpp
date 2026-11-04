/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2026, ISO/IEC
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

#include "PCCFFMPEGLibVideoDecoder.h"

#ifdef USE_FFMPEG_VIDEO_CODEC

#include <iostream>
#include <vector>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/pixdesc.h>
}

using namespace pcc;

namespace {

// Copy one decoded AVFrame into the PCCVideo, converting the plane bit depth to
// outputBitDepth and storing the frame as YUV420 / YUV444.
template <typename T>
void storeDecodedFrame( const AVFrame *frame, size_t outputBitDepth, pcc::PCCVideo<T, 3> &video ) {
  const AVPixFmtDescriptor *desc     = av_pix_fmt_desc_get( static_cast<AVPixelFormat>( frame->format ) );
  const int                 bitDepth = desc != nullptr ? desc->comp[0].depth : 8;
  const bool   is420     = ( frame->format == AV_PIX_FMT_YUV420P ) || ( frame->format == AV_PIX_FMT_YUV420P10LE ) ||
                           ( frame->format == AV_PIX_FMT_YUV420P10BE );
  const int    shiftbits = static_cast<int>( bitDepth ) - static_cast<int>( outputBitDepth );
  const size_t widthY    = frame->width;
  const size_t heightY   = frame->height;
  const size_t widthC    = is420 ? widthY / 2 : widthY;
  const size_t heightC   = is420 ? heightY / 2 : heightY;

  video.resize( video.getFrameCount() + 1 );
  auto &image = video.getFrames().back();

  const pcc::PCCCOLORFORMAT format = is420 ? pcc::PCCCOLORFORMAT::YUV420 : pcc::PCCCOLORFORMAT::YUV444;

  if ( bitDepth == 8 ) {
    image.set( frame->data[0], frame->data[1], frame->data[2], widthY, heightY, frame->linesize[0], widthC, heightC,
               frame->linesize[1], static_cast<int16_t>( shiftbits ), format, false );
  } else {
    image.set( reinterpret_cast<const uint16_t *>( frame->data[0] ),
               reinterpret_cast<const uint16_t *>( frame->data[1] ),
               reinterpret_cast<const uint16_t *>( frame->data[2] ), widthY, heightY, frame->linesize[0] >> 1, widthC,
               heightC, frame->linesize[1] >> 1, static_cast<int16_t>( shiftbits ), format, false );
  }
}

}  // namespace

template <typename T>
PCCFFMPEGLibVideoDecoder<T>::PCCFFMPEGLibVideoDecoder() {}
template <typename T>
PCCFFMPEGLibVideoDecoder<T>::~PCCFFMPEGLibVideoDecoder() {}

template <typename T>
void PCCFFMPEGLibVideoDecoder<T>::decode( PCCVideoBitstream &bitstream,
                                          PCCVideo<T, 3>    &video,
                                          size_t             outputBitDepth,
                                          const std::string &decoderPath,
                                          const std::string &parameters ) {
  (void)decoderPath;
  (void)parameters;
  video.clear();

  const AVCodec *codec = avcodec_find_decoder( AV_CODEC_ID_HEVC );
  if ( codec == nullptr ) {
    std::cerr << "Error: FFMPEG HEVC decoder not available." << std::endl;
    exit( -1 );
  }

  AVCodecContext *ctx = avcodec_alloc_context3( codec );
  if ( ctx == nullptr ) {
    std::cerr << "Error: could not allocate FFMPEG decoder context." << std::endl;
    exit( -1 );
  }
  if ( avcodec_open2( ctx, codec, nullptr ) < 0 ) {
    std::cerr << "Error: could not open FFMPEG HEVC decoder." << std::endl;
    avcodec_free_context( &ctx );
    exit( -1 );
  }

  AVCodecParserContext *parser = av_parser_init( codec->id );
  if ( parser == nullptr ) {
    std::cerr << "Error: could not init FFMPEG HEVC parser." << std::endl;
    avcodec_free_context( &ctx );
    exit( -1 );
  }

  AVPacket *pkt   = av_packet_alloc();
  AVFrame  *frame = av_frame_alloc();

  auto drainFrames = [&]() {
    while ( true ) {
      int ret = avcodec_receive_frame( ctx, frame );
      if ( ret == AVERROR( EAGAIN ) || ret == AVERROR_EOF ) { break; }
      if ( ret < 0 ) {
        std::cerr << "Error: FFMPEG decoding failed." << std::endl;
        break;
      }
      storeDecodedFrame( frame, outputBitDepth, video );
      av_frame_unref( frame );
    }
  };

  const uint8_t *data = bitstream.buffer();
  size_t         size = bitstream.size();
  while ( size > 0 ) {
    uint8_t *pktData = nullptr;
    int      pktSize = 0;
    int      used = av_parser_parse2( parser, ctx, &pktData, &pktSize, data, static_cast<int>( size ), AV_NOPTS_VALUE,
                                      AV_NOPTS_VALUE, 0 );
    if ( used < 0 ) {
      std::cerr << "Error: FFMPEG bitstream parsing failed." << std::endl;
      break;
    }
    data += used;
    size -= used;
    if ( pktSize > 0 ) {
      pkt->data = pktData;
      pkt->size = pktSize;
      if ( avcodec_send_packet( ctx, pkt ) >= 0 ) { drainFrames(); }
    }
  }

  // Flush the decoder.
  avcodec_send_packet( ctx, nullptr );
  drainFrames();

  av_frame_free( &frame );
  av_packet_free( &pkt );
  av_parser_close( parser );
  avcodec_free_context( &ctx );
}

template class pcc::PCCFFMPEGLibVideoDecoder<uint8_t>;
template class pcc::PCCFFMPEGLibVideoDecoder<uint16_t>;

#endif
