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

#ifdef USE_JMLIB_VIDEO_CODEC

#include "PCCJMLibVideoDecoderImpl.h"

extern "C" {
#include "global.h"
#include "h264decoder.h"

void Configure( InputParameters* p_Inp, int ac, char* av[] );
int  OpenDecoder( InputParameters* p_Inp );
int  DecodeOneFrame( DecodedPicList** ppDecPicList );
int  WriteOneFrame( DecodedPicList* pDecPic, int hFileOutput0, int hFileOutput1, int bOutputAllFrames );
}

using namespace pcc;

template <typename T>
PCCJMLibVideoDecoderImpl<T>::PCCJMLibVideoDecoderImpl() {}

template <typename T>
PCCJMLibVideoDecoderImpl<T>::~PCCJMLibVideoDecoderImpl() {}

template <typename T>
void PCCJMLibVideoDecoderImpl<T>::decode( PCCVideoBitstream& bitstream,
                                          PCCVideo<T, 3>&    video,
                                          size_t             outputBitDepth,
                                          const std::string& decoderPath,
                                          const std::string& fileName ) {
  // std::string        s( reinterpret_cast<char*>( bitstream.buffer() ), bitstream.size() );
  // std::istringstream iss( s );
  // std::istream&      bitstreamFile = iss;

  std::string binName = fileName + ".bin";
  std::string decName = fileName + ".yuv";
  bitstream.write( binName );
  std::string        arguments = "JMDEC -i " + binName + " -o " + decName;
  std::istringstream aiss( arguments );
  std::string        token;
  std::vector<char*> args;
  while ( aiss >> token ) {
    char* arg = new char[token.size() + 1];
    copy( token.begin(), token.end(), arg );
    arg[token.size()] = '\0';
    args.push_back( arg );
  }
  int    argc = args.size();
  char** argv = &args[0];
  std::cout << "[JM Dec args " << argc << "]: " << arguments << std::endl;

  // === from decoder_test.ce
  // jmdecmain(argc, argv);
  int             iRet;
  DecodedPicList* pDecPicList;
  int             hFileDecOutput0 = -1, hFileDecOutput1 = -1;
  int             iFramesDecoded = 0;
  InputParameters InputParams;

  init_time();

  // get input parameters;
  Configure( &InputParams, argc, argv );

  // open decoder;
  iRet = OpenDecoder( &InputParams );
  if ( iRet != DEC_OPEN_NOERR ) {
    fprintf( stderr, "Open encoder failed: 0x%x!\n", iRet );
    return;  // failed;
  }

  // decoding;
  do {
    iRet = DecodeOneFrame( &pDecPicList );
    if ( iRet == DEC_EOS || iRet == DEC_SUCCEED ) {
      // process the decoded picture, output or display;
      WriteOneFrame( pDecPicList, hFileDecOutput0, hFileDecOutput1, 0 );
      iFramesDecoded++;
    } else {
      // error handling;
      fprintf( stderr, "Error in decoding process: 0x%x\n", iRet );
    }
  } while ( ( iRet == DEC_SUCCEED ) &&
            ( ( p_Dec->p_Inp->iDecFrmNum == 0 ) || ( iFramesDecoded < p_Dec->p_Inp->iDecFrmNum ) ) );

  FinitDecoder( &pDecPicList );
  WriteOneFrame( pDecPicList, hFileDecOutput0, hFileDecOutput1, 1 );

  int            decWidth  = pDecPicList->iWidth;
  int            decHeight = pDecPicList->iHeight;
  PCCCOLORFORMAT format    = pDecPicList->iYUVFormat == 3 ? PCCCOLORFORMAT::RGB444 : PCCCOLORFORMAT::YUV420;
  const size_t   nbyte     = pDecPicList->iBitDepth == 8 ? 1 : 2;

  // Close and quit;
  CloseDecoder();
  if ( hFileDecOutput0 >= 0 ) { close( hFileDecOutput0 ); }
  if ( hFileDecOutput1 >= 0 ) { close( hFileDecOutput1 ); }
  // end decoding

  video.clear();
  video.read( decName, decWidth, decHeight, format, nbyte );

  removeFile( binName );
  removeFile( decName );
  printf( "[ JM Dec ] %d frames are decoded: %dx%d.\n", iFramesDecoded, decWidth, decHeight );
  return;
}

template class pcc::PCCJMLibVideoDecoderImpl<uint8_t>;
template class pcc::PCCJMLibVideoDecoderImpl<uint16_t>;

#endif
