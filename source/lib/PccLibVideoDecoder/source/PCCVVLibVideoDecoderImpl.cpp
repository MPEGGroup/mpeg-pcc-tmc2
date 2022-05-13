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
#include "PCCCommon.h"
#include "PCCLogger.h"

#ifdef USE_VVLIB_VIDEO_CODEC

#include "PCCVVLibVideoDecoderImpl.h"

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <string.h>
#include <thread>

#include "vvdec/vvdec.h"
#include "vvdec/version.h"

#if defined( _WIN32 ) || defined( WIN32 ) || defined( _WIN64 ) || defined( WIN64 )
#include <io.h>
#include <fcntl.h>
#endif

using namespace pcc;

#define MAX_CODED_PICTURE_SIZE 800000

void msgFncStd( void*, int level, const char* fmt, va_list args ) {
  vfprintf( level == 1 ? stderr : stdout, fmt, args );
}

void msgFncErr( void*, int, const char* fmt, va_list args ) { vfprintf( stderr, fmt, args ); }

/**
 * \brief Retrieving of NAL unit start code
 */
static inline int retrieveNalStartCode( unsigned char* pB, int iZerosInStartcode ) {
  int info = 1;
  int i    = 0;
  for ( i = 0; i < iZerosInStartcode; i++ ) {
    if ( pB[i] != 0 ) { info = 0; }
  }
  if ( pB[i] != 1 ) { info = 0; }
  return info;
}

/**
 * \brief Reading of one Annex B NAL unit from file stream
 */
static int readBitstreamFromFile( std::istream* f, vvdecAccessUnit* pcAccessUnit, bool bLoop ) {
  uint32_t       len;
  int            info2           = 0;
  int            info3           = 0;
  int            pos             = 0;
  int            iStartCodeFound = 0;
  int            iRewind         = 0;
  unsigned char* pBuf            = pcAccessUnit->payload;
  int            curfilpos       = f->tellg();
  pcAccessUnit->payloadUsedSize  = 0;
  if ( curfilpos < 0 ) {
    if ( bLoop ) {
      f->clear();
      f->seekg( 0, f->beg );
      if ( f->bad() || f->fail() ) { return -1; }
    } else {
      return -1;
    }
  }
  // jump over possible start code
  f->read( (char*)pBuf, 5 );
  size_t extracted = f->gcount();
  if ( extracted < 4 ) {
    if ( bLoop ) {
      f->clear();
      f->seekg( 0, f->beg );
      if ( f->bad() || f->fail() ) { return -1; }
      f->read( (char*)pBuf, 5 );
      size_t extracted = f->gcount();
      if ( extracted < 4 ) { return -1; }
    } else {
      return -1;
    }
  }
  pos += 5;
  info2           = 0;
  info3           = 0;
  iStartCodeFound = 0;
  while ( !iStartCodeFound ) {
    if ( f->eof() ) {
      if ( pos > 5 ) {
        len                           = pos - 1;
        pcAccessUnit->payloadUsedSize = len;
        return len;
      } else if ( bLoop ) {
        f->clear();
        f->seekg( 0, f->beg );
      } else {
        return -1;
      }
    }
    if ( pos >= pcAccessUnit->payloadSize ) {
      int            iNewSize = pcAccessUnit->payloadSize * 2;
      unsigned char* newbuf   = (unsigned char*)malloc( sizeof( unsigned char ) * iNewSize );
      if ( newbuf == NULL ) {
        fprintf( stderr, "ERR: readBitstreamFromFile: memory re-allocation failed!\n" );
        return -1;
      }
      std::copy_n( pcAccessUnit->payload, std::min( pcAccessUnit->payloadSize, iNewSize ), newbuf );
      pcAccessUnit->payloadSize = iNewSize;
      free( pcAccessUnit->payload );
      pcAccessUnit->payload = newbuf;
      pBuf                  = pcAccessUnit->payload;
    }
    unsigned char* p = pBuf + pos;
    f->read( (char*)p, 1 );
    pos++;
    info3 = retrieveNalStartCode( &pBuf[pos - 4], 3 );
    if ( info3 != 1 ) { info2 = retrieveNalStartCode( &pBuf[pos - 3], 2 ); }
    iStartCodeFound = ( info2 == 1 || info3 == 1 );
  }

  // Here, we have found another start code (and read length of startcode bytes more than we should
  // have.  Hence, go back in the file
  iRewind = 0;
  if ( info3 == 1 )
    iRewind = -4;
  else if ( info2 == 1 )
    iRewind = -3;
  else
    fprintf( stderr, "ERR: readBitstreamFromFile: Error in next start code search \n" );
  f->seekg( iRewind, f->cur );
  if ( f->bad() || f->fail() ) {
    fprintf( stderr, "ERR: readBitstreamFromFile: Cannot seek %d in the bit stream file", iRewind );
    return -1;
  }
  // Here the Start code, the complete NALU, and the next start code is in the pBuf.
  // The size of pBuf is pos, pos+rewind are the number of bytes excluding the next
  // start code, and (pos+rewind)-startcodeprefix_len is the size of the NALU
  len                           = ( pos + iRewind );
  pcAccessUnit->payloadUsedSize = len;
  return len;
}

template <typename T>
PCCVVLibVideoDecoderImpl<T>::PCCVVLibVideoDecoderImpl() {}

template <typename T>
PCCVVLibVideoDecoderImpl<T>::~PCCVVLibVideoDecoderImpl() {}

template <typename T>
void PCCVVLibVideoDecoderImpl<T>::decode( PCCVideoBitstream& bitstream, size_t outputBitDepth, PCCVideo<T, 3>& video ) {
  std::string        s( reinterpret_cast<char*>( bitstream.buffer() ), bitstream.size() );
  std::istringstream iss( s );
  std::istream&      cInFile          = iss;
  int                iMaxFrames       = -1;
  int                iLoopCount       = 1;
  vvdecDecoder*      dec              = nullptr;
  int                iSEIHashErrCount = 0;
  int                iLoop            = 0;
  bool               bContinue        = true;
  vvdecParams        params;
  params.logLevel = VVDEC_INFO;
  vvdec_params_default( &params );
  vvdecAccessUnit* accessUnit = vvdec_accessUnit_alloc();
  vvdec_accessUnit_alloc_payload( accessUnit, MAX_CODED_PICTURE_SIZE );
  video.clear();

  while ( bContinue ) {
    vvdecFrame* pcFrame     = NULL;
    vvdecFrame* pcPrevField = NULL;
    if ( iLoopCount > 1 ) {
      std::cout << "-------------------------------------" << std::endl;
      std::cout << "begin decoder loop #" << iLoop << std::endl;
      std::cout << "-------------------------------------" << std::endl;
    }
    // initialize the decoder
    dec = vvdec_decoder_open( &params );
    if ( nullptr == dec ) {
      std::cout << "cannot init decoder" << std::endl;
      vvdec_accessUnit_free( accessUnit );
      return;
    }
    vvdec_set_logging_callback( dec, msgFncStd );
    if ( iLoop == 0 ) { std::cout << vvdec_get_dec_information( dec ) << std::endl; }
    bool bFlushDecoder                     = false;
    bool bOutputInfoWritten                = false;
    accessUnit->cts                        = 0;
    accessUnit->ctsValid                   = true;
    accessUnit->dts                        = 0;
    accessUnit->dtsValid                   = true;
    int          iComprPics                = 0;
    unsigned int uiFrames                  = 0;
    unsigned int uiBitrate                 = 0;
    bool         bTunedIn                  = false;
    unsigned int uiNoFrameAfterTuneInCount = 0;
    vvdecNalType eNalTypeSlice             = VVC_NAL_UNIT_INVALID;
    bool         bMultipleSlices           = false;
    int          iRead                     = 0;
    int          iRet                      = 0;
    do {
      iRead = readBitstreamFromFile( &cInFile, accessUnit, false );
      if ( iRead > 0 ) {
        vvdecNalType eNalType = vvdec_get_nal_unit_type( accessUnit );
        if ( eNalType == VVC_NAL_UNIT_PH ) { bMultipleSlices = true; }  // picture header indicates multiple slices
        bool bIsSlice = vvdec_is_nal_unit_slice( eNalType );
        if ( bIsSlice ) {
          if ( bMultipleSlices ) {
            if ( eNalTypeSlice == VVC_NAL_UNIT_INVALID ) {
              iComprPics++;  // set current slice type and increment pic count
              eNalTypeSlice = eNalType;
            } else {
              bIsSlice = false;  // prevent cts/dts increase if not first slice
            }
          } else {
            iComprPics++;
          }
        }
        if ( eNalTypeSlice != VVC_NAL_UNIT_INVALID && eNalType != eNalTypeSlice ) {
          eNalTypeSlice = VVC_NAL_UNIT_INVALID;  // reset slice type
        }
        if ( iMaxFrames > 0 && iComprPics >= iMaxFrames ) { iRead = -1; }

        // call decode
        iRet = vvdec_decode( dec, accessUnit, &pcFrame );
        if ( bIsSlice ) {
          accessUnit->cts++;
          accessUnit->dts++;
        }
        // check success
        if ( iRet == VVDEC_EOF ) {
          bFlushDecoder = true;
        } else if ( iRet == VVDEC_TRY_AGAIN ) {
          if ( !bMultipleSlices ) {
            if ( params.logLevel >= VVDEC_VERBOSE ) std::cout << "more data needed to tune in" << std::endl;
            if ( bTunedIn ) {
              // after the first frame is returned, the decoder must always return a frame
              if ( bIsSlice ) {
                std::cout << "vvdecapp [error]: missing output picture!" << std::endl;
                uiNoFrameAfterTuneInCount++;
              }
            }
          }
        } else if ( iRet == VVDEC_ERR_DEC_INPUT ) {
          std::string cErr           = vvdec_get_last_error( dec );
          std::string cAdditionalErr = vvdec_get_last_additional_error( dec );
          std::cout << "vvdecapp [warning]: " << cErr << " (" << vvdec_get_error_msg( iRet ) << ")";
          if ( !cAdditionalErr.empty() ) {
            std::cout << " detail: " << vvdec_get_last_additional_error( dec ) << std::endl;
          }
          std::cout << std::endl;
        } else if ( iRet != VVDEC_OK ) {
          std::string cErr           = vvdec_get_last_error( dec );
          std::string cAdditionalErr = vvdec_get_last_additional_error( dec );
          std::cout << "vvdecapp [error]: decoding failed: " << cErr << " (" << vvdec_get_error_msg( iRet ) << ")";
          if ( !cAdditionalErr.empty() ) {
            std::cout << " detail: " << vvdec_get_last_additional_error( dec ) << std::endl;
          }
          std::cout << std::endl;
          vvdec_accessUnit_free( accessUnit );
          return;
        }
        if ( NULL != pcFrame && pcFrame->ctsValid ) {
          if ( !bOutputInfoWritten ) {
            if ( params.logLevel >= VVDEC_INFO ) {
              std::cout << "vvdecapp [info]: SizeInfo: " << pcFrame->width << "x" << pcFrame->height << " ("
                        << pcFrame->bitDepth << "b)" << std::endl;
            }
            bOutputInfoWritten = true;
          }
          if ( !bTunedIn ) { bTunedIn = true; }
          uiFrames++;
          if ( pcFrame->picAttributes ) {
            uiBitrate += pcFrame->picAttributes->bits;
            (void)uiBitrate;
          }
          if ( pcFrame->frameFormat == VVDEC_FF_PROGRESSIVE ) {
            if ( 0 != storeDecodedFrame( pcFrame, outputBitDepth, video ) ) {
              std::cout << "vvdecapp [error]: write of rec. yuv failed for picture seq. " << pcFrame->sequenceNumber
                        << std::endl;
              vvdec_accessUnit_free( accessUnit );
              return;
            }
          } else {
            std::cout << "vvdecapp [error]: unsupported FrameFormat " << pcFrame->frameFormat << " for picture seq. "
                      << pcFrame->sequenceNumber << std::endl;
            vvdec_accessUnit_free( accessUnit );
            return;
          }
          // free picture memory
          if ( !pcPrevField || pcPrevField != pcFrame ) { vvdec_frame_unref( dec, pcFrame ); }
        }
      }
    } while ( iRead > 0 && !bFlushDecoder );  // end for frames

    // flush the decoder
    bFlushDecoder = true;
    while ( bFlushDecoder ) {
      vvdecFrame* pcFrame = NULL;
      // flush the decoder
      iRet = vvdec_flush( dec, &pcFrame );
      if ( iRet != VVDEC_OK && iRet != VVDEC_EOF ) {
        std::string cErr           = vvdec_get_last_error( dec );
        std::string cAdditionalErr = vvdec_get_last_additional_error( dec );
        std::cout << "vvdecapp [error]: decoding failed: " << cErr << " (" << vvdec_get_error_msg( iRet ) << ")";
        if ( !cAdditionalErr.empty() ) { std::cout << " detail: " << cAdditionalErr; }
        std::cout << std::endl;
        vvdec_accessUnit_free( accessUnit );
        return;
      }
      if ( NULL != pcFrame ) {
        uiFrames++;
        if ( pcFrame->frameFormat == VVDEC_FF_PROGRESSIVE ) {
          if ( 0 != storeDecodedFrame( pcFrame, outputBitDepth, video ) ) {
            std::cout << "vvdecapp [error]: write of rec. yuv failed for picture seq. " << pcFrame->sequenceNumber
                      << std::endl;
            vvdec_accessUnit_free( accessUnit );
            return;
          }
        } else {
          std::cout << "vvdecapp [error]: unsupported FrameFormat " << pcFrame->frameFormat << " for picture seq. "
                    << pcFrame->sequenceNumber << std::endl;
          vvdec_accessUnit_free( accessUnit );
          return;
        }
        // free picture memory
        if ( !pcPrevField || pcPrevField != pcFrame ) { vvdec_frame_unref( dec, pcFrame ); }
      } else {
        break;
      }
    };
    if ( uiNoFrameAfterTuneInCount ) {
      std::cout << "vvdecapp [error]: Decoder did not return " << uiNoFrameAfterTuneInCount
                << " pictures during decoding - came out too late" << std::endl;
    }
    iSEIHashErrCount = vvdec_get_hash_error_count( dec );
    if ( iSEIHashErrCount ) {
      std::cout << "vvdecapp [error]: MD5 checksum error ( " << iSEIHashErrCount << " errors )" << std::endl;
      vvdec_accessUnit_free( accessUnit );
      return;
    }
    if ( iComprPics && !uiFrames ) {
      std::cout << "vvdecapp [error]: read some input pictures (" << iComprPics << "), but no output was generated."
                << std::endl;
      vvdec_accessUnit_free( accessUnit );
      return;
    }
    // un-initialize the decoder
    iRet = vvdec_decoder_close( dec );
    if ( 0 != iRet ) {
      std::cout << "vvdecapp [error]: cannot uninit decoder (" << iRet << ")" << std::endl;
      vvdec_accessUnit_free( accessUnit );
      return;
    }
    if ( iLoopCount < 0 || iLoop < iLoopCount - 1 ) {
      cInFile.clear();
      cInFile.seekg( 0, cInFile.beg );
      if ( cInFile.bad() || cInFile.fail() ) {
        std::cout << "vvdecapp [error]: cannot seek to bitstream begin" << std::endl;
        vvdec_accessUnit_free( accessUnit );
        return;
      }
    }
    iLoop++;
    if ( iLoopCount >= 0 && iLoop >= iLoopCount ) { bContinue = false; }
  }
  vvdec_accessUnit_free( accessUnit );
}

template <typename T>
int PCCVVLibVideoDecoderImpl<T>::storeDecodedFrame( const vvdecFrame* pcFrame,
                                                    size_t            outputBitDepth,
                                                    PCCVideo<T, 3>&   video ) {

  int chromaSubsample = pcFrame->colorFormat == VVDEC_CF_YUV420_PLANAR ? 2 : 1;
  // printf(
  //     "storeDecodedFrame: write output frame %zu size = %d %d ( %d %d %d / %d %d %d) bit depth = %d - %zu = %zu in %zd "
  //     "bytes buffers ( ChromaSub. = %d ) bytesPerSample = %u %u %u \n",
  //     video.size(), pcFrame->width, pcFrame->height, pcFrame->planes[0].width, pcFrame->planes[0].height,
  //     pcFrame->planes[0].stride, pcFrame->planes[1].width, pcFrame->planes[1].height, pcFrame->planes[1].stride,
  //     pcFrame->bitDepth, outputBitDepth, pcFrame->bitDepth - outputBitDepth, sizeof( T ), chromaSubsample,
  //     pcFrame->planes[0].bytesPerSample, pcFrame->planes[1].bytesPerSample, pcFrame->planes[2].bytesPerSample );
  // fflush( stdout );
  video.resize( video.getFrameCount() + 1 );
  auto&          image  = video.getFrames().back();
  PCCCOLORFORMAT format = chromaSubsample > 1                              ? PCCCOLORFORMAT::YUV420
                          : pcFrame->colorFormat == VVDEC_CF_YUV444_PLANAR ? PCCCOLORFORMAT::RGB444
                                                                           : PCCCOLORFORMAT::YUV444;
  if ( pcFrame->planes[0].bytesPerSample == 1 ) {
    image.set( pcFrame->planes[0].ptr,                            // Y ptr
               pcFrame->planes[1].ptr,                            // U ptr
               pcFrame->planes[2].ptr,                            // V ptr
               pcFrame->planes[0].width,                          // widthY
               pcFrame->planes[0].height,                         // heightY
               pcFrame->planes[0].stride,                         // strideY
               pcFrame->planes[1].width,                          // widthC
               pcFrame->planes[1].height,                         // heightC
               pcFrame->planes[1].stride,                         // strideC
               pcFrame->bitDepth - outputBitDepth,                // shiftbits
               format,                                            // format
               pcFrame->colorFormat == VVDEC_CF_YUV444_PLANAR );  // rgb2bgr
  } else {
    image.set( (uint16_t*)( pcFrame->planes[0].ptr ),             // Y ptr
               (uint16_t*)( pcFrame->planes[1].ptr ),             // U ptr
               (uint16_t*)( pcFrame->planes[2].ptr ),             // V ptr
               pcFrame->planes[0].width,                          // widthY
               pcFrame->planes[0].height,                         // heightY
               pcFrame->planes[0].stride >> 1,                    // strideY
               pcFrame->planes[1].width,                          // widthC
               pcFrame->planes[1].height,                         // heightC
               pcFrame->planes[1].stride >> 1,                    // strideC
               pcFrame->bitDepth - outputBitDepth,                // shiftbits
               format,                                            // format
               pcFrame->colorFormat == VVDEC_CF_YUV444_PLANAR );  // rgb2bgr
  }
  // image.trace();
  return 0;
}

template class pcc::PCCVVLibVideoDecoderImpl<uint8_t>;
template class pcc::PCCVVLibVideoDecoderImpl<uint16_t>;

#endif
