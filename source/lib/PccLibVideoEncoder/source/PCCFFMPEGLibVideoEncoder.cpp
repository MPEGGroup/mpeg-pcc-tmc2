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

#include "PCCFFMPEGLibVideoEncoder.h"

#ifdef USE_FFMPEG_VIDEO_CODEC

#include "PCCFFMPEGLibVideoDecoder.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// KVZ_BIT_DEPTH selects the kvz_pixel type and must match the Kvazaar library,
// which is built with 10-bit support so Main10 content (and the lossless 10-bit
// geometry) can be encoded. FFmpeg's libkvazaar wrapper only exposes 8-bit
// yuv420p, so the native Kvazaar API is used for encoding; FFmpeg is kept solely
// for decoding the reconstructed stream.
#define KVZ_BIT_DEPTH 10
extern "C" {
#include <kvazaar.h>
}

using namespace pcc;

namespace {

// Copy one PCCImage channel (tightly packed, count = width*height samples) into
// a Kvazaar plane. Kvazaar planes are tightly packed (stride == width), so a
// straight element-wise copy into the target sample type suffices.
template <typename T>
void fillPlane( kvz_pixel *dst, const std::vector<T> &channel, size_t count ) {
  for ( size_t i = 0; i < count; ++i ) { dst[i] = static_cast<kvz_pixel>( channel[i] ); }
}

std::string trim( const std::string &s ) {
  const size_t b = s.find_first_not_of( " \t\r\n" );
  if ( b == std::string::npos ) { return ""; }
  const size_t e = s.find_last_not_of( " \t\r\n" );
  return s.substr( b, e - b + 1 );
}

// Extract the "KvazaarParams" value from an HM-style "Key : Value" config file.
// Returns an empty string when the file or the key is absent.
std::string readKvazaarParams( const std::string &cfgPath ) {
  if ( cfgPath.empty() ) { return ""; }
  std::ifstream in( cfgPath );
  if ( !in.is_open() ) { return ""; }
  std::string line;
  while ( std::getline( in, line ) ) {
    const size_t hash = line.find( '#' );
    if ( hash != std::string::npos ) { line = line.substr( 0, hash ); }
    const size_t colon = line.find( ':' );
    if ( colon == std::string::npos ) { continue; }
    if ( trim( line.substr( 0, colon ) ) == "KvazaarParams" ) { return trim( line.substr( colon + 1 ) ); }
  }
  return "";
}

// Apply a comma-separated Kvazaar parameter string ("preset=veryslow,lossless,
// no-deblock,...") to a kvz_config via the native config_parse API. Bare tokens
// (no '=') are treated as boolean flags.
void applyKvazaarParams( const kvz_api *api, kvz_config *cfg, const std::string &csv ) {
  std::stringstream ss( csv );
  std::string       token;
  while ( std::getline( ss, token, ',' ) ) {
    token = trim( token );
    if ( token.empty() ) { continue; }
    const size_t eq    = token.find( '=' );
    std::string  name  = eq == std::string::npos ? token : trim( token.substr( 0, eq ) );
    std::string  value = eq == std::string::npos ? "" : trim( token.substr( eq + 1 ) );
    if ( api->config_parse( cfg, name.c_str(), eq == std::string::npos ? nullptr : value.c_str() ) != 1 ) {
      std::cerr << "Warning: Kvazaar rejected option \"" << token << "\"." << std::endl;
    }
  }
}

}  // namespace

template <typename T>
PCCFFMPEGLibVideoEncoder<T>::PCCFFMPEGLibVideoEncoder() {}
template <typename T>
PCCFFMPEGLibVideoEncoder<T>::~PCCFFMPEGLibVideoEncoder() {}

template <typename T>
void PCCFFMPEGLibVideoEncoder<T>::encode( PCCVideo<T, 3>            &videoSrc,
                                          PCCVideoEncoderParameters &params,
                                          PCCVideoBitstream         &bitstream,
                                          PCCVideo<T, 3>            &videoRec ) {
  const int    width      = static_cast<int>( videoSrc.getWidth() );
  const int    height     = static_cast<int>( videoSrc.getHeight() );
  const size_t frameCount = videoSrc.getFrameCount();
  const int    bitDepth   = params.inputBitDepth_;
  const bool   is420      = videoSrc.is420();

  const kvz_api *api = kvz_api_get( bitDepth <= 8 ? 8 : 10 );
  if ( api == nullptr ) {
    std::cerr << "Error: could not obtain Kvazaar API." << std::endl;
    exit( -1 );
  }

  kvz_config *cfg = api->config_alloc();
  if ( cfg == nullptr || api->config_init( cfg ) != 1 ) {
    std::cerr << "Error: could not initialise Kvazaar config." << std::endl;
    exit( -1 );
  }

  // Coding tools come from the codec config file (KvazaarParams line); the I/O
  // geometry and QP are (re)asserted afterwards so a stray config value cannot
  // override them. QP is authoritative from the GS rate config (and ignored by
  // Kvazaar in lossless mode).
  const std::string cfgParams = readKvazaarParams( params.encoderConfig_ );
  applyKvazaarParams( api, cfg, cfgParams );

  cfg->width           = width;
  cfg->height          = height;
  cfg->input_bitdepth  = bitDepth;
  cfg->input_format    = is420 ? KVZ_FORMAT_P420 : KVZ_FORMAT_P444;
  cfg->framerate_num   = 30;
  cfg->framerate_denom = 1;
  cfg->qp              = params.qp_;

  std::cout << "Kvazaar encode: " << width << "x" << height << " frames = " << frameCount << " bitDepth = " << bitDepth
            << " qp = " << params.qp_ << " params = \"" << cfgParams << "\""
            << " lossless = " << cfg->lossless << std::endl;

  kvz_encoder *enc = api->encoder_open( cfg );
  if ( enc == nullptr ) {
    std::cerr << "Error: could not open Kvazaar encoder." << std::endl;
    api->config_destroy( cfg );
    exit( -1 );
  }

  const enum kvz_chroma_format csp = is420 ? KVZ_CSP_420 : KVZ_CSP_444;

  std::vector<uint8_t> &out = bitstream.vector();
  out.clear();

  auto appendChunks = [&]( kvz_data_chunk *chunk ) {
    for ( kvz_data_chunk *c = chunk; c != nullptr; c = c->next ) { out.insert( out.end(), c->data, c->data + c->len ); }
  };

  const size_t lumaCount   = static_cast<size_t>( width ) * height;
  const size_t chromaCount = is420 ? lumaCount / 4 : lumaCount;

  for ( size_t f = 0; f < frameCount; ++f ) {
    kvz_picture *pic = api->picture_alloc_csp( csp, width, height );
    if ( pic == nullptr ) {
      std::cerr << "Error: could not allocate Kvazaar picture." << std::endl;
      break;
    }
    auto &image = videoSrc.getFrame( f );
    fillPlane( pic->y, image.getChannel( 0 ), lumaCount );
    fillPlane( pic->u, image.getChannel( 1 ), chromaCount );
    fillPlane( pic->v, image.getChannel( 2 ), chromaCount );
    pic->pts = static_cast<int64_t>( f );

    kvz_data_chunk *dataOut = nullptr;
    uint32_t        lenOut  = 0;
    if ( api->encoder_encode( enc, pic, &dataOut, &lenOut, nullptr, nullptr, nullptr ) != 1 ) {
      std::cerr << "Error: Kvazaar encoding failed." << std::endl;
      api->picture_free( pic );
      break;
    }
    if ( dataOut != nullptr ) {
      appendChunks( dataOut );
      api->chunk_free( dataOut );
    }
    api->picture_free( pic );
  }

  // Flush the encoder.
  while ( true ) {
    kvz_data_chunk *dataOut = nullptr;
    uint32_t        lenOut  = 0;
    if ( api->encoder_encode( enc, nullptr, &dataOut, &lenOut, nullptr, nullptr, nullptr ) != 1 ) { break; }
    if ( dataOut == nullptr ) { break; }
    appendChunks( dataOut );
    api->chunk_free( dataOut );
  }

  api->encoder_close( enc );
  api->config_destroy( cfg );

  // Reconstruct by decoding the freshly encoded Annex-B stream with FFmpeg.
  PCCFFMPEGLibVideoDecoder<T> decoder;
  decoder.decode( bitstream, videoRec, static_cast<size_t>( params.outputBitDepth_ ) );
}

template class pcc::PCCFFMPEGLibVideoEncoder<uint8_t>;
template class pcc::PCCFFMPEGLibVideoEncoder<uint16_t>;

#endif
