/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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

#include "PCCJMLibVideoEncoderImpl.h"

extern "C" {
#include "mbuffer.h"
#include "global.h"

void alloc_encoder( EncoderParams** p_Enc );
void init_encoder( VideoParameters* p_Vid, InputParameters* p_Inp );
void encode_sequence( VideoParameters* p_Vid, InputParameters* p_Inp );
void populate_frm_struct( VideoParameters* p_Vid,
                          InputParameters* p_Inp,
                          SeqStructure*    p_seq_struct,
                          int              num_to_populate,
                          int              init_frames_to_code );
void prepare_frame_params( VideoParameters* p_Vid, InputParameters* p_Inp, int curr_frame_to_code );
int  encode_one_frame( VideoParameters* p_Vid, InputParameters* p_Inp );
void report_frame_statistic( VideoParameters* p_Vid, InputParameters* p_Inp );
void img2buf( imgpel**       imgX,
              unsigned char* buf,
              int            size_x,
              int            size_y,
              int            symbol_size_in_bytes,
              int            crop_left,
              int            crop_right,
              int            crop_top,
              int            crop_bottom );

// void free_encoder_memory(VideoParameters *p_Vid, InputParameters *p_Inp);
void free_params( InputParameters* p_Inp );
void free_encoder( EncoderParams* p_Enc );
void Configure( VideoParameters* p_Vid, InputParameters* p_Inp, int ac, char* av[] );
}

using namespace pcc;

/// encoder application class

template <typename T>
PCCJMLibVideoEncoderImpl<T>::PCCJMLibVideoEncoderImpl() {}

template <typename T>
PCCJMLibVideoEncoderImpl<T>::~PCCJMLibVideoEncoderImpl() {}

template <typename T>
void PCCJMLibVideoEncoderImpl<T>::encode( PCCVideo<T, 3>&    videoSrc,
                                          std::string        arguments,
                                          PCCVideoBitstream& bitstream,
                                          PCCVideo<T, 3>&    videoRec ) {
  std::istringstream iss( arguments );
  std::string        token;
  std::vector<char*> args;
  while ( iss >> token ) {
    char* arg = new char[token.size() + 1];
    copy( token.begin(), token.end(), arg );
    arg[token.size()] = '\0';
    args.push_back( arg );
  }
  int    argc = args.size();
  char** argv = &args[0];
  std::cout << "[JM Enc args " << argc << "]: " << arguments << std::endl;
  const size_t   srcWidth           = videoSrc.getWidth();
  const size_t   srcHeight          = videoSrc.getHeight();
  int32_t        sourceBitDepthLuma = std::stoi( getParameter( arguments, "SourceBitDepthLuma=" ) );
  PCCCOLORFORMAT format             = videoSrc.getColorFormat();
  const size_t   nbyte              = sourceBitDepthLuma == 10 ? 2 : 1;

  init_time();
  std::cout << "[ JM Enc ]: frames " << videoSrc.getFrameCount() << std::endl;

  // copy from lencod.c main()
  alloc_encoder( &p_Enc );
  Configure( p_Enc->p_Vid, p_Enc->p_Inp, argc, argv );

  for ( size_t i = 0; i < args.size(); i++ ) { delete[] args[i]; }

  std::string srcName = p_Enc->p_Inp->input_file1.fname;
  std::string binName = p_Enc->p_Inp->outfile;
  std::string recName = p_Enc->p_Inp->ReconFile;
  videoSrc.write( srcName, nbyte );

  // init encoder
  init_encoder( p_Enc->p_Vid, p_Enc->p_Inp );

  // encode sequence
  encode_sequence( p_Enc->p_Vid, p_Enc->p_Inp );

  // terminate sequence
  free_encoder_memory( p_Enc->p_Vid, p_Enc->p_Inp );
  free_params( p_Enc->p_Inp );
  free_encoder( p_Enc );

  // === need to fill buffer
  videoRec.clear();
  videoRec.read( recName, srcWidth, srcHeight, format, nbyte );
  bitstream.read( binName );

  // Remove temp files
  removeFile( srcName );
  removeFile( recName );
  removeFile( binName );
  return;
}

template class pcc::PCCJMLibVideoEncoderImpl<uint8_t>;
template class pcc::PCCJMLibVideoEncoderImpl<uint16_t>;

#endif
