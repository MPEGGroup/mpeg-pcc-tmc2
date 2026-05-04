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
#include "PCCVirtualColorConverter.h"

#ifndef USE_HDRTOOLS

#include "PCCHDRToolsAppColorConverter.h"
#include "PCCSystem.h"

using namespace pcc;

template <typename T>
PCCHDRToolsAppColorConverter<T>::PCCHDRToolsAppColorConverter() {}
template <typename T>
PCCHDRToolsAppColorConverter<T>::~PCCHDRToolsAppColorConverter() {}

template <typename T>
void PCCHDRToolsAppColorConverter<T>::convert( std::string        configFile,
                                               PCCVideo<T, 3>    &videoSrc,
                                               PCCVideo<T, 3>    &videoDst,
                                               const std::string &externalPath,
                                               const std::string &fileName ) {
  if ( externalPath.empty() ) {
    printf(
        "PCCHDRToolsAppColorConverter error: hdrTools path must be be defined "
        "\n" );
    exit( -1 );
  }
  if ( fileName.empty() ) {
    printf(
        "PCCHDRToolsAppColorConverter error: fileName path must be be defined "
        "\n" );
    exit( -1 );
  }
  std::ifstream ifs( configFile );
  std::string   config = std::string( ( std::istreambuf_iterator<char>( ifs ) ), ( std::istreambuf_iterator<char>() ) );
  int32_t       sourceBitDepth     = std::stoi( getParameter( config, "SourceBitDepthCmp0=" ) );
  int32_t       sourceChromaFormat = std::stoi( getParameter( config, "SourceChromaFormat=" ) );
  int32_t       sourceColorSpace   = std::stoi( getParameter( config, "SourceColorSpace=" ) );
  int32_t       outputBitDepth     = std::stoi( getParameter( config, "OutputBitDepthCmp0=" ) );
  int32_t       outputChromaFormat = std::stoi( getParameter( config, "OutputChromaFormat=" ) );
  int32_t       outputColorSpace   = std::stoi( getParameter( config, "OutputColorSpace=" ) );
  const size_t  width              = videoSrc.getWidth();
  const size_t  height             = videoSrc.getHeight();
  const size_t  frameCount         = videoSrc.getFrameCount();
  const std::string sourceFile     = addVideoFormat( fileName, width, height, sourceColorSpace == 0,
                                                     sourceChromaFormat == 1, sourceBitDepth == 8 ? "8" : "10" );
  const std::string outputFile     = addVideoFormat( fileName, width, height, outputColorSpace == 0,
                                                     outputChromaFormat == 1, outputBitDepth == 8 ? "8" : "16" );
  videoSrc.write( sourceFile, sourceBitDepth == 8 ? 1 : 2 );
  std::stringstream cmd;
  cmd << externalPath << " -f " << configFile << " -p SourceFile=\"" << sourceFile << "\""
      << " -p OutputFile=\"" << outputFile << "\""
      << " -p SourceWidth=" << width << " -p SourceHeight=" << height << " -p NumberOfFrames=" << frameCount;
  std::cout << cmd.str() << '\n';
  if ( pcc::system( cmd.str().c_str() ) ) {
    std::cout << "Error: can't run system command!" << std::endl;
    exit( -1 );
  }
  PCCCOLORFORMAT format = outputChromaFormat == 1 ? PCCCOLORFORMAT::YUV420
                          : outputColorSpace == 0 ? PCCCOLORFORMAT::YUV444
                                                  : PCCCOLORFORMAT::RGB444;
  videoDst.clear();
  videoDst.read( outputFile, width, height, format, outputBitDepth == 8 ? 1 : 2 );
  removeFile( sourceFile );
  removeFile( outputFile );
}

template <typename T>
void PCCHDRToolsAppColorConverter<T>::scale( std::string        configFile,
                                             PCCVideo<T, 3>    &videoSrc,
                                             PCCVideo<T, 3>    &videoDst,
                                             int                bitDepth,
                                             int                chromaFormat,
                                             int                colorSpace,
                                             int                outputWidth,
                                             int                outputHeight,
                                             const std::string &externalPath,
                                             const std::string &fileName ) {
  if ( externalPath.empty() ) {
    printf(
        "PCCHDRToolsAppColorConverter error: hdrTools path must be be defined "
        "\n" );
    exit( -1 );
  }
  if ( fileName.empty() ) {
    printf(
        "PCCHDRToolsAppColorConverter error: fileName path must be be defined "
        "\n" );
    exit( -1 );
  }
  std::ifstream ifs( configFile );
  std::string   config = std::string( ( std::istreambuf_iterator<char>( ifs ) ), ( std::istreambuf_iterator<char>() ) );
  int32_t       sourceBitDepth     = bitDepth;      // std::stoi(getParameter(config, "SourceBitDepthCmp0="));
  int32_t       sourceChromaFormat = chromaFormat;  // std::stoi(getParameter(config, "SourceChromaFormat="));
  int32_t       sourceColorSpace   = colorSpace;    // std::stoi(getParameter(config, "SourceColorSpace="));
  int32_t       outputBitDepth     = bitDepth;      // std::stoi( getParameter( config, "OutputBitDepthCmp0=" ) );
  int32_t       outputChromaFormat = chromaFormat;  // std::stoi(getParameter(config, "OutputChromaFormat="));
  int32_t       outputColorSpace   = colorSpace;    // std::stoi(getParameter(config, "OutputColorSpace="));
  const size_t  width              = videoSrc.getWidth();
  const size_t  height             = videoSrc.getHeight();
  outputWidth                      = outputWidth == 0 ? videoSrc.getWidth() : outputWidth;
  outputHeight                     = outputHeight == 0 ? videoSrc.getHeight() : outputHeight;
  const size_t      frameCount     = videoSrc.getFrameCount();
  const std::string sourceFile     = addVideoFormat( fileName, width, height, sourceColorSpace == 0,
                                                     sourceChromaFormat == 1, sourceBitDepth == 8 ? "8" : "10" );
  const std::string outputFile     = addVideoFormat( fileName, outputWidth, outputHeight, outputColorSpace == 0,
                                                     outputChromaFormat == 1, outputBitDepth == 8 ? "8" : "16" );
  videoSrc.write( sourceFile, sourceBitDepth == 8 ? 1 : 2 );
  std::stringstream cmd;
  cmd << externalPath << " -f " << configFile << " -p SourceFile=\"" << sourceFile << "\""
      << " -p OutputFile=\"" << outputFile << "\""
      << " -p OutputWidth=" << outputWidth << " -p OutputHeight=" << outputHeight
      << " -p SourceChromaFormat=" << chromaFormat << " -p OutputChromaFormat=" << chromaFormat
      << " -p SourceColorSpace=" << colorSpace << " -p OutputColorSpace=" << colorSpace
      << " -p SourceBitDepthCmp0=" << sourceBitDepth << " -p SourceBitDepthCmp1=" << sourceBitDepth
      << " -p SourceBitDepthCmp2=" << sourceBitDepth << " -p OutputBitDepthCmp0=" << outputBitDepth
      << " -p OutputBitDepthCmp1=" << outputBitDepth << " -p OutputBitDepthCmp2=" << outputBitDepth
      << " -p SourceWidth=" << width << " -p SourceHeight=" << height << " -p NumberOfFrames=" << frameCount;
  std::cout << cmd.str() << '\n';
  if ( pcc::system( cmd.str().c_str() ) ) {
    std::cout << "Error: can't run system command!" << std::endl;
    exit( -1 );
  }
  PCCCOLORFORMAT format = outputChromaFormat == 1 ? PCCCOLORFORMAT::YUV420
                          : outputColorSpace == 0 ? PCCCOLORFORMAT::YUV444
                                                  : PCCCOLORFORMAT::RGB444;
  videoDst.clear();
  videoDst.read( outputFile, outputWidth, outputHeight, format, outputBitDepth == 8 ? 1 : 2 );
  removeFile( sourceFile );
  removeFile( outputFile );
}

template class pcc::PCCHDRToolsAppColorConverter<uint8_t>;
template class pcc::PCCHDRToolsAppColorConverter<uint16_t>;

#endif  //~USE_HDRTOOLS
