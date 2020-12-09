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

#ifndef PCC_LOGGER_H
#define PCC_LOGGER_H

#include "PCCBitstreamCommon.h"

namespace pcc {

enum PCCLoggerType {
  LOG_DESCR = 0,  // *.txt	a short description of the bitstream	(mandatory)
  LOG_TRACE,      // *.trc	trace file	(optional)
  LOG_ATLAS,      // *_atl.ofl	output atlas log	(mandatory)
  LOG_TILES,      // *_atl.otl	output tile group log	(mandatory)
  LOG_FRAME,      // *.opcl	output point cloud frame log	(mandatory)
#ifdef BITSTREAM_TRACE
  LOG_STREAM,
#endif
#ifdef CODEC_TRACE
  LOG_CODEC,
  LOG_PATCH,
#endif
  LOG_ERROR,
};

static const std::string get( PCCLoggerType type ) {
  switch ( type ) {
    case LOG_DESCR: return std::string( ".txt" );
    case LOG_TRACE: return std::string( ".trc" );
    case LOG_ATLAS: return std::string( "_atl.ofl" );
    case LOG_TILES: return std::string( "_atl.otl" );
    case LOG_FRAME: return std::string( ".opcl" );
#ifdef BITSTREAM_TRACE
    case LOG_STREAM: return std::string( "_bitstream.txt" );
#endif
#ifdef CODEC_TRACE
    case LOG_CODEC: return std::string( "_codec.txt" );
    case LOG_PATCH: return std::string( "_patch.txt" );
#endif
    default: return std::string( "_unknow.txt" );
  }
}

class PCCVirtualLogger {
 public:
  PCCVirtualLogger() : file_( NULL ), disable_( false ) {}
  ~PCCVirtualLogger() { close(); }
  bool initialize( PCCLoggerType type, std::string& filename, bool encoder ) {
    return open( filename + ( encoder ? "_enc" : "_dec" ) + get( type ) );
  }
  inline bool isInitialized() { return file_ != NULL; }
  inline void disable() { disable_ = true; }
  inline void enable() { disable_ = false; }
  template <typename... Args>
  inline void trace( const char* format, Args... eArgs ) {
    if ( file_ && !disable_ ) { fprintf( file_, format, eArgs... ); }
  }
  inline void flush() {
    if ( !disable_ ) { fflush( file_ ); }
  }

 private:
  void close() {
    if ( file_ ) { fclose( file_ ); }
    file_ = NULL;
  }
  bool open( std::string name ) {
    close();
    return ( ( file_ = fopen( name.c_str(), "w+" ) ) != NULL );
  }
  FILE* file_;
  bool  disable_;
};

class PCCLogger {
 public:
  PCCLogger() : filename_( "" ), encoder_( true ) { logger_.resize( LOG_ERROR ); }
  ~PCCLogger() { logger_.clear(); }
  void initilalize( std::string filename, bool encoder ) {
    filename_ = filename;
    encoder_  = encoder;
  }
  void enable( PCCLoggerType type ) { logger_[type].enable(); }
  void disable( PCCLoggerType type ) { logger_[type].disable(); }

  template <typename... Args>
  inline void trace( PCCLoggerType type, const char* format, Args... args ) {
    if ( !logger_[type].isInitialized() ) { logger_[type].initialize( type, filename_, encoder_ ); }
    if ( logger_[type].isInitialized() ) {
      logger_[type].trace( format, args... );
      logger_[type].flush();
    }
  }
  template <typename... Args>
  inline void traceDescr( const char* format, Args... args ) {
    trace( LOG_DESCR, format, args... );
  }
  template <typename... Args>
  inline void traceTrace( const char* format, Args... args ) {
    trace( LOG_TRACE, format, args... );
  }
  template <typename... Args>
  inline void traceAtlas( const char* format, Args... args ) {
    trace( LOG_ATLAS, format, args... );
  }
  template <typename... Args>
  inline void traceTiles( const char* format, Args... args ) {
    trace( LOG_TILES, format, args... );
  }
  template <typename... Args>
  inline void traceFrame( const char* format, Args... args ) {
    trace( LOG_FRAME, format, args... );
  }
#ifdef BITSTREAM_TRACE
  template <typename... Args>
  inline void traceStream( const char* format, Args... args ) {
    trace( LOG_STREAM, format, args... );
  }
#endif
#ifdef CODEC_TRACE
  template <typename... Args>
  inline void traceCodec( const char* format, Args... args ) {
    trace( LOG_CODEC, format, args... );
  }
  template <typename... Args>
  inline void tracePatch( const char* format, Args... args ) {
    trace( LOG_PATCH, format, args... );
  }
#endif

  template <typename T>
  void traceVector( PCCLoggerType     type,
                    std::vector<T>    data,
                    const size_t      width,
                    const size_t      height,
                    const std::string string,
                    const bool        hexa = false ) {
    if ( data.size() == 0 ) { data.resize( width * height, 0 ); }
    trace( type, "%s: %zu %zu \n", string.c_str(), width, height );
    for ( size_t v0 = 0; v0 < height; ++v0 ) {
      for ( size_t u0 = 0; u0 < width; ++u0 ) { trace( type, hexa ? "%2x" : "%3d", (int)( data[v0 * width + u0] ) ); }
      trace( type, "\n" );
    }
  }

 private:
  std::vector<PCCVirtualLogger> logger_;
  std::string                   filename_;
  bool                          encoder_;
};

#ifdef BITSTREAM_TRACE
#define TRACE_BITSTREAM( fmt, ... ) bitstream.trace( fmt, ##__VA_ARGS__ );
#else
#define TRACE_BITSTREAM( fmt, ... )
#endif

#ifdef CODEC_TRACE
#define TRACE_CODEC( fmt, ... ) logger_->traceCodec( fmt, ##__VA_ARGS__ );
#define TRACE_PATCH( fmt, ... ) logger_->tracePatch( fmt, ##__VA_ARGS__ );
#else
#define TRACE_CODEC( fmt, ... ) ;
#define TRACE_PATCH( fmt, ... ) ;
#endif

}  // namespace pcc

#endif /* PCC_LOGGER_H */
