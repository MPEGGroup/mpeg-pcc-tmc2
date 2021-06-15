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

#ifdef USE_HDRTOOLS

#include "PCCHDRToolsLibColorConverterImpl.h"

using namespace pcc;

template <typename T>
PCCHDRToolsLibColorConverterImpl<T>::PCCHDRToolsLibColorConverterImpl() {
  m_oFrameStore  = NULL;
  m_iFrameStore  = NULL;
  m_nFrameStores = 7;
  for ( int index = 0; index < m_nFrameStores; index++ ) { m_pFrameStore[index] = NULL; }
  m_convertFrameStore      = NULL;
  m_colorSpaceConvert      = NULL;
  m_colorSpaceConvertMC    = NULL;
  m_convertIQuantize       = NULL;
  m_colorSpaceFrame        = NULL;
  m_convertProcess         = NULL;
  m_colorTransform         = NULL;
  m_normalizeFunction      = NULL;
  m_inputTransferFunction  = NULL;
  m_outputTransferFunction = NULL;
  m_inputFrame             = NULL;
  m_outputFrame            = NULL;
  m_convertFormatIn        = NULL;
  m_addNoise               = NULL;
  m_convertFormatOut       = NULL;
  m_scaledFrame            = NULL;
  m_frameScale             = NULL;
  m_frameFilter            = NULL;
  m_frameFilterNoise0      = NULL;
  m_frameFilterNoise1      = NULL;
  m_frameFilterNoise2      = NULL;
  m_croppedFrameStore      = NULL;
  m_srcDisplayGammaAdjust  = NULL;
  m_outDisplayGammaAdjust  = NULL;
  m_changeColorPrimaries   = false;
  m_inputFile              = NULL;
  m_outputFile             = NULL;
}
template <typename T>
PCCHDRToolsLibColorConverterImpl<T>::~PCCHDRToolsLibColorConverterImpl() {
  destroy();
}

template <typename T>
void PCCHDRToolsLibColorConverterImpl<T>::convert( std::string     configFile,
                                                   PCCVideo<T, 3>& videoSrc,
                                                   PCCVideo<T, 3>& videoDst ) {
  using hdrtoolslib::params;
  using hdrtoolslib::ZERO;
  params                         = &ccParams;
  ProjectParameters* inputParams = (ProjectParameters*)( params );
  inputParams->refresh();
  if ( !inputParams->readConfigFile( (char*)configFile.c_str() ) ) {
    printf( "Could not open configuration file: %s.\n", configFile.c_str() );
    exit( -1 );
  }
  inputParams->m_source.m_width[0]  = videoSrc.getWidth();
  inputParams->m_source.m_height[0] = videoSrc.getHeight();
  inputParams->m_numberOfFrames     = videoSrc.getFrameCount();
  inputParams->update();
  init( inputParams );
  process( inputParams, videoSrc, videoDst );
  destroy();
}

template <typename T>
void PCCHDRToolsLibColorConverterImpl<T>::init( ProjectParameters* inputParams ) {
  m_filterInFloat        = inputParams->m_filterInFloat;
  m_inputFile            = &inputParams->m_inputFile;
  m_outputFile           = &inputParams->m_outputFile;
  m_startFrame           = m_inputFile->m_startFrame;
  m_cropOffsetLeft       = inputParams->m_cropOffsetLeft;
  m_cropOffsetTop        = inputParams->m_cropOffsetTop;
  m_cropOffsetRight      = inputParams->m_cropOffsetRight;
  m_cropOffsetBottom     = inputParams->m_cropOffsetBottom;
  m_bUseChromaDeblocking = inputParams->m_bUseChromaDeblocking;
  m_bUseWienerFiltering  = inputParams->m_bUseWienerFiltering;
  m_bUse2DSepFiltering   = inputParams->m_bUse2DSepFiltering;
  m_bUseNLMeansFiltering = inputParams->m_bUseNLMeansFiltering;
  m_b2DSepMode           = inputParams->m_b2DSepMode;
  using hdrtoolslib::Frame;
  using hdrtoolslib::FrameFormat;
  using hdrtoolslib::IOFunctions;
  using hdrtoolslib::Y_COMP;
  FrameFormat* input  = &inputParams->m_source;
  FrameFormat* output = &inputParams->m_output;

  // create memory for reading the input filesource
  m_inputFile->m_videoType = hdrtoolslib::VideoFileType::VIDEO_YUV;
  m_inputFrame             = hdrtoolslib::Input::create( m_inputFile, input, inputParams );
  // Output file
  IOFunctions::openFile( m_outputFile, OPENFLAGS_WRITE, OPEN_PERMISSIONS );

  // create frame memory as necessary
  // Input. This has the same format as the Input file.
  m_inputFrame->m_colorSpace = input->m_colorSpace;
  m_iFrameStore =
      new Frame( m_inputFrame->m_width[Y_COMP], m_inputFrame->m_height[Y_COMP], m_inputFrame->m_isFloat,
                 m_inputFrame->m_colorSpace, m_inputFrame->m_colorPrimaries, m_inputFrame->m_chromaFormat,
                 m_inputFrame->m_sampleRange, m_inputFrame->m_bitDepthComp[Y_COMP], m_inputFrame->m_isInterlaced,
                 m_inputFrame->m_transferFunction, m_inputFrame->m_systemGamma );
  m_iFrameStore->clear();

  if ( m_cropOffsetLeft != 0 || m_cropOffsetTop != 0 || m_cropOffsetRight != 0 || m_cropOffsetBottom != 0 ) {
    m_cropWidth = m_width = m_inputFrame->m_width[Y_COMP] - m_cropOffsetLeft + m_cropOffsetRight;
    m_cropHeight = m_height = m_inputFrame->m_height[Y_COMP] - m_cropOffsetTop + m_cropOffsetBottom;
    m_croppedFrameStore     = new Frame(
        m_width, m_height, m_inputFrame->m_isFloat, m_inputFrame->m_colorSpace, m_inputFrame->m_colorPrimaries,
        m_inputFrame->m_chromaFormat, m_inputFrame->m_sampleRange, m_inputFrame->m_bitDepthComp[Y_COMP],
        m_inputFrame->m_isInterlaced, m_inputFrame->m_transferFunction, m_inputFrame->m_systemGamma );
    m_croppedFrameStore->clear();
  } else {
    m_cropWidth = m_width = m_inputFrame->m_width[Y_COMP];
    m_cropHeight = m_height = m_inputFrame->m_height[Y_COMP];
  }

  // Output (transfer function or denormalization). Since we don't support
  // scaling, lets reset the width and height here.
  m_outputFile->m_format.m_height[Y_COMP] = output->m_height[Y_COMP];
  m_outputFile->m_format.m_width[Y_COMP]  = output->m_width[Y_COMP];

  // Create output file
  m_outputFrame = hdrtoolslib::Output::create( m_outputFile, output );
  hdrtoolslib::ChromaFormat chromaFormat =
      ( m_inputFrame->m_colorPrimaries != output->m_colorPrimaries ||
        m_inputFrame->m_colorSpace == hdrtoolslib::CM_RGB )
          ? hdrtoolslib::CF_444
          : ( input->m_chromaFormat == hdrtoolslib::CF_400 ? hdrtoolslib::CF_400 : output->m_chromaFormat );

  // Chroma format conversion (if needed). Only difference is in chroma format
  if ( output->m_chromaFormat != m_inputFrame->m_chromaFormat ||
       ( m_inputFrame->m_chromaFormat != hdrtoolslib::CF_444 &&
         ( m_inputFrame->m_colorPrimaries != output->m_colorPrimaries ) ) ) {
    if ( m_filterInFloat == true ) {
      m_pFrameStore[0] =
          new Frame( m_width, m_height, true, m_inputFrame->m_colorSpace, m_inputFrame->m_colorPrimaries, chromaFormat,
                     m_inputFrame->m_sampleRange, m_inputFrame->m_bitDepthComp[Y_COMP], m_inputFrame->m_isInterlaced,
                     m_inputFrame->m_transferFunction, m_inputFrame->m_systemGamma );
    } else {
      m_pFrameStore[0] = new Frame( m_width, m_height, m_inputFrame->m_isFloat, m_inputFrame->m_colorSpace,
                                    m_inputFrame->m_colorPrimaries, chromaFormat, m_inputFrame->m_sampleRange,
                                    m_inputFrame->m_bitDepthComp[Y_COMP], m_inputFrame->m_isInterlaced,
                                    m_inputFrame->m_transferFunction, m_inputFrame->m_systemGamma );
    }
    m_pFrameStore[0]->clear();
  } else {
    m_pFrameStore[0] = NULL;
  }

  // Here we may convert to the output's format type (e.g. from integer to
  // float).
  m_convertFrameStore =
      new Frame( m_width, m_height, true, m_inputFrame->m_colorSpace, m_inputFrame->m_colorPrimaries, chromaFormat,
                 output->m_sampleRange, output->m_bitDepthComp[Y_COMP], output->m_isInterlaced,
                 m_inputFrame->m_transferFunction, m_inputFrame->m_systemGamma );
  m_convertFrameStore->clear();

  // Also creation of frame store for the transfer function processed images
  m_pFrameStore[3] =
      new Frame( output->m_width[Y_COMP], output->m_height[Y_COMP], true, output->m_colorSpace,
                 output->m_colorPrimaries, chromaFormat, output->m_sampleRange, output->m_bitDepthComp[Y_COMP],
                 output->m_isInterlaced, output->m_transferFunction, output->m_systemGamma );
  m_pFrameStore[3]->clear();

  // Frame store for the inversion of PQ TF
  m_pFrameStore[1] =
      new Frame( output->m_width[Y_COMP], output->m_height[Y_COMP], true, output->m_colorSpace,
                 output->m_colorPrimaries, chromaFormat, output->m_sampleRange, output->m_bitDepthComp[Y_COMP],
                 output->m_isInterlaced, output->m_transferFunction, output->m_systemGamma );
  m_pFrameStore[1]->clear();

  m_oFrameStore = new Frame( output->m_width[Y_COMP], output->m_height[Y_COMP], output->m_isFloat, output->m_colorSpace,
                             output->m_colorPrimaries, output->m_chromaFormat, output->m_sampleRange,
                             output->m_bitDepthComp[Y_COMP], output->m_isInterlaced, output->m_transferFunction,
                             output->m_systemGamma );
  m_oFrameStore->clear();

  // initiate the color transform process. Maybe we can move this in the
  // process section though.
  // This does not currently handle YCbCr to YCbCr conversion cleanly (goes
  // first to RGB which is not correct). Needs to be fixed.
  using hdrtoolslib::CM_RGB;
  using hdrtoolslib::CM_YCbCr;
  using hdrtoolslib::ColorTransform;
  if ( m_iFrameStore->m_colorSpace == CM_YCbCr ) {  // If YCbCr we need to first go to RGB
    if ( m_oFrameStore->m_colorSpace == CM_YCbCr &&
         m_iFrameStore->m_colorPrimaries == m_oFrameStore->m_colorPrimaries &&
         input->m_iConstantLuminance == output->m_iConstantLuminance ) {
      m_colorTransform = ColorTransform::create(
          m_iFrameStore->m_colorSpace, m_iFrameStore->m_colorPrimaries, m_oFrameStore->m_colorSpace,
          m_iFrameStore->m_colorPrimaries, inputParams->m_transformPrecision, inputParams->m_useHighPrecisionTransform,
          hdrtoolslib::CLT_NULL, input->m_iConstantLuminance, output->m_iConstantLuminance );
      m_changeColorPrimaries = false;
    } else {
      m_colorTransform = ColorTransform::create( m_iFrameStore->m_colorSpace, m_iFrameStore->m_colorPrimaries, CM_RGB,
                                                 m_iFrameStore->m_colorPrimaries, inputParams->m_transformPrecision,
                                                 inputParams->m_useHighPrecisionTransform, hdrtoolslib::CLT_NULL,
                                                 input->m_iConstantLuminance, 0 );
      m_changeColorPrimaries = true;
    }
    if ( m_oFrameStore->m_colorSpace != CM_YCbCr ) {
      inputParams->m_ctParams.m_max = inputParams->m_outNormalScale;
      m_colorSpaceConvert           = ColorTransform::create(
          CM_RGB, m_iFrameStore->m_colorPrimaries, m_oFrameStore->m_colorSpace, m_oFrameStore->m_colorPrimaries,
          inputParams->m_transformPrecision, inputParams->m_useHighPrecisionTransform,
          inputParams->m_closedLoopConversion, 0, output->m_iConstantLuminance, output->m_transferFunction,
          output->m_bitDepthComp[Y_COMP], output->m_sampleRange, inputParams->m_chromaDownsampleFilter,
          inputParams->m_chromaUpsampleFilter, inputParams->m_useAdaptiveDownsampling,
          inputParams->m_useAdaptiveUpsampling, inputParams->m_useMinMax, inputParams->m_closedLoopIterations,
          output->m_chromaFormat, output->m_chromaLocation, inputParams->m_filterInFloat, inputParams->m_enableTFLUTs,
          &inputParams->m_ctParams );
    } else {
      inputParams->m_ctParams.m_max = inputParams->m_outNormalScale;
      m_colorSpaceConvert           = ColorTransform::create(
          CM_RGB, m_iFrameStore->m_colorPrimaries, CM_RGB, m_oFrameStore->m_colorPrimaries,
          inputParams->m_transformPrecision, inputParams->m_useHighPrecisionTransform,
          inputParams->m_closedLoopConversion, 0, output->m_iConstantLuminance, output->m_transferFunction,
          output->m_bitDepthComp[Y_COMP], output->m_sampleRange, inputParams->m_chromaDownsampleFilter,
          inputParams->m_chromaUpsampleFilter, inputParams->m_useAdaptiveDownsampling,
          inputParams->m_useAdaptiveUpsampling, inputParams->m_useMinMax, inputParams->m_closedLoopIterations,
          output->m_chromaFormat, output->m_chromaLocation, inputParams->m_filterInFloat, inputParams->m_enableTFLUTs,
          &inputParams->m_ctParams );
      m_colorSpaceConvertMC =
          ColorTransform::create( CM_RGB, m_oFrameStore->m_colorPrimaries, CM_YCbCr, m_oFrameStore->m_colorPrimaries,
                                  inputParams->m_transformPrecision, inputParams->m_useHighPrecisionTransform,
                                  inputParams->m_closedLoopConversion, 0, output->m_iConstantLuminance );
    }

    // frame store for color format conversion
    m_pFrameStore[2] =
        new Frame( output->m_width[Y_COMP], output->m_height[Y_COMP], true, CM_RGB, m_iFrameStore->m_colorPrimaries,
                   chromaFormat, output->m_sampleRange, output->m_bitDepthComp[Y_COMP], output->m_isInterlaced,
                   m_inputFrame->m_transferFunction, m_inputFrame->m_systemGamma );
    m_pFrameStore[2]->clear();

    m_colorSpaceFrame = new Frame( output->m_width[Y_COMP], output->m_height[Y_COMP], true, output->m_colorSpace,
                                   output->m_colorPrimaries, chromaFormat, output->m_sampleRange,
                                   output->m_bitDepthComp[Y_COMP], output->m_isInterlaced, hdrtoolslib::TF_NULL, 1.0 );
    m_colorSpaceFrame->clear();
  } else if ( m_iFrameStore->m_colorSpace == hdrtoolslib::CM_ICtCp ) {  // If YCbCr we need to first go to RGB
    if ( m_oFrameStore->m_colorSpace == hdrtoolslib::CM_ICtCp &&
         m_iFrameStore->m_colorPrimaries == m_oFrameStore->m_colorPrimaries &&
         input->m_iConstantLuminance == output->m_iConstantLuminance ) {
      m_colorTransform = ColorTransform::create(
          m_iFrameStore->m_colorSpace, m_iFrameStore->m_colorPrimaries, m_oFrameStore->m_colorSpace,
          m_iFrameStore->m_colorPrimaries, inputParams->m_transformPrecision, inputParams->m_useHighPrecisionTransform,
          hdrtoolslib::CLT_NULL, input->m_iConstantLuminance, output->m_iConstantLuminance );
      m_changeColorPrimaries = false;
    } else {
      m_colorTransform = ColorTransform::create( m_iFrameStore->m_colorSpace, m_iFrameStore->m_colorPrimaries, CM_RGB,
                                                 m_iFrameStore->m_colorPrimaries, inputParams->m_transformPrecision,
                                                 inputParams->m_useHighPrecisionTransform, hdrtoolslib::CLT_NULL,
                                                 input->m_iConstantLuminance, 0 );
      m_changeColorPrimaries = true;
    }
    if ( m_oFrameStore->m_colorSpace != hdrtoolslib::CM_ICtCp ) {
      inputParams->m_ctParams.m_max = inputParams->m_outNormalScale;
      m_colorSpaceConvert           = ColorTransform::create(
          CM_RGB, m_iFrameStore->m_colorPrimaries, m_oFrameStore->m_colorSpace, m_oFrameStore->m_colorPrimaries,
          inputParams->m_transformPrecision, inputParams->m_useHighPrecisionTransform,
          inputParams->m_closedLoopConversion, 0, output->m_iConstantLuminance, output->m_transferFunction,
          output->m_bitDepthComp[Y_COMP], output->m_sampleRange, inputParams->m_chromaDownsampleFilter,
          inputParams->m_chromaUpsampleFilter, inputParams->m_useAdaptiveDownsampling,
          inputParams->m_useAdaptiveUpsampling, inputParams->m_useMinMax, inputParams->m_closedLoopIterations,
          output->m_chromaFormat, output->m_chromaLocation, inputParams->m_filterInFloat, inputParams->m_enableTFLUTs,
          &inputParams->m_ctParams );
    } else {
      inputParams->m_ctParams.m_max = inputParams->m_outNormalScale;

      m_colorSpaceConvert = ColorTransform::create(
          CM_RGB, m_iFrameStore->m_colorPrimaries, CM_RGB, m_oFrameStore->m_colorPrimaries,
          inputParams->m_transformPrecision, inputParams->m_useHighPrecisionTransform,
          inputParams->m_closedLoopConversion, 0, output->m_iConstantLuminance, output->m_transferFunction,
          output->m_bitDepthComp[Y_COMP], output->m_sampleRange, inputParams->m_chromaDownsampleFilter,
          inputParams->m_chromaUpsampleFilter, inputParams->m_useAdaptiveDownsampling,
          inputParams->m_useAdaptiveUpsampling, inputParams->m_useMinMax, inputParams->m_closedLoopIterations,
          output->m_chromaFormat, output->m_chromaLocation, inputParams->m_filterInFloat, inputParams->m_enableTFLUTs,
          &inputParams->m_ctParams );
      m_colorSpaceConvertMC = ColorTransform::create(
          CM_RGB, m_oFrameStore->m_colorPrimaries, hdrtoolslib::CM_ICtCp, m_oFrameStore->m_colorPrimaries,
          inputParams->m_transformPrecision, inputParams->m_useHighPrecisionTransform,
          inputParams->m_closedLoopConversion, 0, output->m_iConstantLuminance );
    }

    // frame store for color format conversion
    m_pFrameStore[2] =
        new Frame( output->m_width[Y_COMP], output->m_height[Y_COMP], true, CM_RGB, m_iFrameStore->m_colorPrimaries,
                   chromaFormat, output->m_sampleRange, output->m_bitDepthComp[Y_COMP], output->m_isInterlaced,
                   m_inputFrame->m_transferFunction, m_inputFrame->m_systemGamma );
    m_pFrameStore[2]->clear();

    m_colorSpaceFrame = new Frame( output->m_width[Y_COMP], output->m_height[Y_COMP], true, output->m_colorSpace,
                                   output->m_colorPrimaries, chromaFormat, output->m_sampleRange,
                                   output->m_bitDepthComp[Y_COMP], output->m_isInterlaced, hdrtoolslib::TF_NULL, 1.0 );
    m_colorSpaceFrame->clear();
  } else {
    hdrtoolslib::ColorSpace     trgColorSpace = m_oFrameStore->m_colorSpace;
    hdrtoolslib::ColorPrimaries trgPrimaries  = m_oFrameStore->m_colorPrimaries;
    if ( m_iFrameStore->m_colorPrimaries != m_oFrameStore->m_colorPrimaries &&
         m_inputFrame->m_transferFunction != hdrtoolslib::TF_NULL ) {
      trgColorSpace          = m_iFrameStore->m_colorSpace;
      trgPrimaries           = m_iFrameStore->m_colorPrimaries;
      m_changeColorPrimaries = true;
    } else {
      m_changeColorPrimaries = false;
    }
    m_colorTransform = ColorTransform::create( m_iFrameStore->m_colorSpace, m_iFrameStore->m_colorPrimaries,
                                               trgColorSpace, trgPrimaries, inputParams->m_transformPrecision,
                                               inputParams->m_useHighPrecisionTransform, hdrtoolslib::CLT_NULL );
    m_colorSpaceConvert =
        ColorTransform::create( trgColorSpace, trgPrimaries, m_oFrameStore->m_colorSpace,
                                m_oFrameStore->m_colorPrimaries, inputParams->m_transformPrecision,
                                inputParams->m_useHighPrecisionTransform, inputParams->m_closedLoopConversion );

    // frame store for color format conversion
    m_pFrameStore[2] =
        new Frame( output->m_width[Y_COMP], output->m_height[Y_COMP], true, trgColorSpace, trgPrimaries, chromaFormat,
                   output->m_sampleRange, output->m_bitDepthComp[Y_COMP], output->m_isInterlaced,
                   m_inputFrame->m_transferFunction, m_inputFrame->m_systemGamma );
    m_pFrameStore[2]->clear();
    m_colorSpaceFrame = new Frame( output->m_width[Y_COMP], output->m_height[Y_COMP], true, output->m_colorSpace,
                                   output->m_colorPrimaries, chromaFormat, output->m_sampleRange,
                                   output->m_bitDepthComp[Y_COMP], output->m_isInterlaced, hdrtoolslib::TF_NULL, 1.0 );
    m_colorSpaceFrame->clear();
  }

  // Chroma subsampling
  // We may wish to create a single convert class that uses as inputs the
  // output resolution as well the input and output chroma format, and the
  // downsampling/upsampling method. That would make the code easier to
  // handle.
  // To be done later.

  using hdrtoolslib::CF_444;
  using hdrtoolslib::ConvertColorFormat;
  if ( input->m_chromaFormat == output->m_chromaFormat && input->m_chromaFormat == CF_444 ) {
    m_convertFormatIn = hdrtoolslib::ConvertColorFormat::create(
        m_width, m_height, m_inputFrame->m_chromaFormat, output->m_chromaFormat, 0, m_inputFrame->m_chromaLocation,
        output->m_chromaLocation );
  } else if ( input->m_chromaFormat == hdrtoolslib::CF_400 ||
              output->m_chromaFormat == hdrtoolslib::CF_400 ) {  // Mono conversion
    m_convertFormatIn = hdrtoolslib::ConvertColorFormat::create(
        m_width, m_height, m_inputFrame->m_chromaFormat, output->m_chromaFormat, 0, m_inputFrame->m_chromaLocation,
        output->m_chromaLocation );

    m_convertFormatOut = hdrtoolslib::ConvertColorFormat::create( output->m_width[Y_COMP], output->m_height[Y_COMP],
                                                                  input->m_chromaFormat, output->m_chromaFormat, 0,
                                                                  output->m_chromaLocation, output->m_chromaLocation );

    m_pFrameStore[5] = new Frame( output->m_width[Y_COMP], output->m_height[Y_COMP], true, output->m_colorSpace,
                                  output->m_colorPrimaries, output->m_chromaFormat, output->m_sampleRange,
                                  output->m_bitDepthComp[Y_COMP], output->m_isInterlaced, output->m_transferFunction,
                                  output->m_systemGamma );
    m_pFrameStore[5]->clear();
  } else if ( input->m_chromaFormat != hdrtoolslib::CF_444 &&
              output->m_chromaFormat != CF_444 ) {  // If not 444, check also color space
    int inMode = ( input->m_chromaFormat < output->m_chromaFormat ) ? inputParams->m_chromaUpsampleFilter
                                                                    : inputParams->m_chromaDownsampleFilter;
    int outMode = ( output->m_chromaFormat < input->m_chromaFormat ) ? inputParams->m_chromaDownsampleFilter
                                                                     : inputParams->m_chromaUpsampleFilter;
    if ( input->m_colorPrimaries != output->m_colorPrimaries ) {
      m_convertFormatIn =
          ConvertColorFormat::create( m_width, m_height, m_inputFrame->m_chromaFormat, hdrtoolslib::CF_444, inMode,
                                      m_inputFrame->m_chromaLocation, output->m_chromaLocation );
      m_convertFormatOut = ConvertColorFormat::create( output->m_width[Y_COMP], output->m_height[Y_COMP],
                                                       hdrtoolslib::CF_444, output->m_chromaFormat, outMode,
                                                       output->m_chromaLocation, output->m_chromaLocation );
      m_pFrameStore[5]   = new Frame( output->m_width[Y_COMP], output->m_height[Y_COMP], true, output->m_colorSpace,
                                    output->m_colorPrimaries, output->m_chromaFormat, output->m_sampleRange,
                                    output->m_bitDepthComp[Y_COMP], output->m_isInterlaced, output->m_transferFunction,
                                    output->m_systemGamma );
      m_pFrameStore[5]->clear();
    } else {
      m_convertFormatIn =
          ConvertColorFormat::create( m_width, m_height, m_inputFrame->m_chromaFormat, output->m_chromaFormat, inMode,
                                      m_inputFrame->m_chromaLocation, output->m_chromaLocation );
      m_convertFormatOut = ConvertColorFormat::create( output->m_width[Y_COMP], output->m_height[Y_COMP],
                                                       m_inputFrame->m_chromaFormat, output->m_chromaFormat, outMode,
                                                       m_inputFrame->m_chromaLocation, output->m_chromaLocation );
    }
  } else if ( input->m_chromaFormat == CF_444 && output->m_chromaFormat == hdrtoolslib::CF_420 ) {
    m_convertFormatIn = ConvertColorFormat::create( m_width, m_height, m_inputFrame->m_chromaFormat,
                                                    output->m_chromaFormat, inputParams->m_chromaDownsampleFilter,
                                                    m_inputFrame->m_chromaLocation, output->m_chromaLocation,
                                                    inputParams->m_useAdaptiveDownsampling, inputParams->m_useMinMax );
    if ( input->m_colorSpace == CM_RGB ) {
      int outMode = inputParams->m_chromaDownsampleFilter;
      m_convertFormatOut =
          ConvertColorFormat::create( output->m_width[Y_COMP], output->m_height[Y_COMP], CF_444, output->m_chromaFormat,
                                      outMode, output->m_chromaLocation, output->m_chromaLocation );
      m_pFrameStore[5] = new Frame( output->m_width[Y_COMP], output->m_height[Y_COMP], true, output->m_colorSpace,
                                    output->m_colorPrimaries, output->m_chromaFormat, output->m_sampleRange,
                                    output->m_bitDepthComp[Y_COMP], output->m_isInterlaced, output->m_transferFunction,
                                    output->m_systemGamma );
    }
  } else if ( ( input->m_chromaFormat == hdrtoolslib::CF_420 || input->m_chromaFormat == hdrtoolslib::CF_422 ) &&
              output->m_chromaFormat == CF_444 ) {
    m_convertFormatIn = ConvertColorFormat::create( m_width, m_height, m_inputFrame->m_chromaFormat,
                                                    output->m_chromaFormat, inputParams->m_chromaUpsampleFilter,
                                                    m_inputFrame->m_chromaLocation, output->m_chromaLocation,
                                                    inputParams->m_useAdaptiveUpsampling, inputParams->m_useMinMax );
  }

  m_pFrameStore[4] =
      new Frame( output->m_width[Y_COMP], output->m_height[Y_COMP], true, output->m_colorSpace,
                 output->m_colorPrimaries, chromaFormat, output->m_sampleRange, output->m_bitDepthComp[Y_COMP],
                 output->m_isInterlaced, output->m_transferFunction, output->m_systemGamma );
  m_pFrameStore[4]->clear();

  m_pFrameStore[6] =
      new Frame( output->m_width[Y_COMP], output->m_height[Y_COMP], true, CM_RGB, output->m_colorPrimaries,
                 chromaFormat, output->m_sampleRange, output->m_bitDepthComp[Y_COMP], output->m_isInterlaced,
                 output->m_transferFunction, output->m_systemGamma );
  m_pFrameStore[6]->clear();

  using hdrtoolslib::TransferFunction;
  if ( ( ( input->m_colorSpace == output->m_colorSpace ) && ( input->m_colorPrimaries == output->m_colorPrimaries ) &&
         ( input->m_transferFunction == output->m_transferFunction ) ) ||
       ( ( input->m_colorSpace == CM_RGB && output->m_colorSpace == CM_YCbCr ) &&
         ( input->m_colorPrimaries == output->m_colorPrimaries ) &&
         ( input->m_transferFunction == output->m_transferFunction ) ) ) {
    m_useSingleTransferStep = true;
    m_normalizeFunction     = NULL;
    m_inputTransferFunction = hdrtoolslib::TransferFunction::create(
        hdrtoolslib::TF_NULL, true, inputParams->m_enableLegacy ? 1.0f : inputParams->m_srcNormalScale,
        input->m_systemGamma, inputParams->m_srcMinValue, inputParams->m_srcMaxValue );
    // Output transfer function picture store
    m_outputTransferFunction = hdrtoolslib::TransferFunction::create(
        hdrtoolslib::TF_NULL, true, inputParams->m_enableLegacy ? 1.0f : inputParams->m_outNormalScale,
        output->m_systemGamma, inputParams->m_outMinValue, inputParams->m_outMaxValue );
  } else {
    if ( ( input->m_iConstantLuminance != 0 || output->m_iConstantLuminance != 0 ) ||
         ( input->m_transferFunction != hdrtoolslib::TF_NULL && input->m_transferFunction != hdrtoolslib::TF_POWER &&
           ( inputParams->m_useSingleTransferStep == false ||
             ( input->m_transferFunction != hdrtoolslib::TF_PQ && input->m_transferFunction != hdrtoolslib::TF_HPQ &&
               input->m_transferFunction != hdrtoolslib::TF_HPQ2 && input->m_transferFunction != hdrtoolslib::TF_APQ &&
               input->m_transferFunction != hdrtoolslib::TF_APQS && input->m_transferFunction != hdrtoolslib::TF_MPQ &&
               input->m_transferFunction != hdrtoolslib::TF_AMPQ && input->m_transferFunction != hdrtoolslib::TF_PH &&
               input->m_transferFunction != hdrtoolslib::TF_APH &&
               input->m_transferFunction != hdrtoolslib::TF_HLG ) ) ) ) {
      m_useSingleTransferStep = false;
      m_normalizeFunction =
          TransferFunction::create( hdrtoolslib::TF_NORMAL, false, inputParams->m_srcNormalScale, input->m_systemGamma,
                                    inputParams->m_srcMinValue, inputParams->m_srcMaxValue );
      m_inputTransferFunction = TransferFunction::create(
          input->m_transferFunction, false, inputParams->m_srcNormalScale, input->m_systemGamma,
          inputParams->m_srcMinValue, inputParams->m_srcMaxValue, inputParams->m_enableTFunctionLUT );
    } else {
      m_useSingleTransferStep = true;
      m_normalizeFunction     = NULL;
      m_inputTransferFunction = TransferFunction::create(
          input->m_transferFunction, true, inputParams->m_srcNormalScale, input->m_systemGamma,
          inputParams->m_srcMinValue, inputParams->m_srcMaxValue, inputParams->m_enableTFunctionLUT );
    }
    // Output transfer function picture store
    m_outputTransferFunction =
        TransferFunction::create( output->m_transferFunction, true,
                                  ( inputParams->m_enableLegacy && output->m_transferFunction == hdrtoolslib::TF_NULL )
                                      ? 1.0f
                                      : inputParams->m_outNormalScale,
                                  output->m_systemGamma, inputParams->m_outMinValue, inputParams->m_outMaxValue,
                                  inputParams->m_enableTFunctionLUT );
  }

  // Format conversion process
  using hdrtoolslib::Convert;
  m_convertIQuantize =
      Convert::create( &m_iFrameStore->m_format, &m_convertFrameStore->m_format, &inputParams->m_cvParams );
  m_convertProcess = Convert::create( &m_pFrameStore[4]->m_format, output, &inputParams->m_cvParams );
  if ( m_bUseChromaDeblocking == true )
    m_frameFilter = hdrtoolslib::FrameFilter::create( m_width, m_height, hdrtoolslib::FT_DEBLOCK );

  m_frameScale =
      hdrtoolslib::FrameScale::create( m_width, m_height, output->m_width[Y_COMP], output->m_height[Y_COMP],
                                       &inputParams->m_fsParams, inputParams->m_chromaDownsampleFilter,
                                       output->m_chromaLocation[hdrtoolslib::FP_FRAME], inputParams->m_useMinMax );
  m_scaledFrame = new Frame( output->m_width[Y_COMP], output->m_height[Y_COMP], true, m_inputFrame->m_colorSpace,
                             m_inputFrame->m_colorPrimaries, chromaFormat, output->m_sampleRange,
                             output->m_bitDepthComp[Y_COMP], output->m_isInterlaced, hdrtoolslib::TF_NORMAL, 1.0 );

  using hdrtoolslib::FrameFilter;
  if ( m_bUseWienerFiltering == true )
    m_frameFilterNoise0 = FrameFilter::create( m_width, m_height, hdrtoolslib::FT_WIENER2D );
  if ( m_bUse2DSepFiltering == true )
    m_frameFilterNoise1 = FrameFilter::create( m_width, m_height, hdrtoolslib::FT_2DSEP, m_b2DSepMode );
  if ( m_bUseNLMeansFiltering == true )
    m_frameFilterNoise2 = FrameFilter::create( m_width, m_height, hdrtoolslib::FT_NLMEANS );

  m_srcDisplayGammaAdjust = hdrtoolslib::DisplayGammaAdjust::create(
      input->m_displayAdjustment, m_useSingleTransferStep ? inputParams->m_srcNormalScale : 1.0f,
      input->m_systemGamma );
  m_outDisplayGammaAdjust = hdrtoolslib::DisplayGammaAdjust::create(
      output->m_displayAdjustment, m_useSingleTransferStep ? inputParams->m_outNormalScale : 1.0f,
      output->m_systemGamma );

  if ( input->m_displayAdjustment == hdrtoolslib::DA_HLG ) { m_inputTransferFunction->setNormalFactor( 1.0 ); }
  if ( output->m_displayAdjustment == hdrtoolslib::DA_HLG ) { m_outputTransferFunction->setNormalFactor( 1.0 ); }
  m_addNoise =
      hdrtoolslib::AddNoise::create( inputParams->m_addNoise, inputParams->m_noiseVariance, inputParams->m_noiseMean );
}

template <typename T>
void PCCHDRToolsLibColorConverterImpl<T>::process( ProjectParameters* inputParams,
                                                   PCCVideo<T, 3>&    videoSrc,
                                                   PCCVideo<T, 3>&    videoDst ) {
  int                       frameNumber;
  int                       iCurrentFrameToProcess = 0;
  float                     fDistance0   = inputParams->m_source.m_frameRate / inputParams->m_output.m_frameRate;
  bool                      errorRead    = false;
  hdrtoolslib::Frame*       currentFrame = NULL;
  hdrtoolslib::FrameFormat* input        = &inputParams->m_source;
  videoDst.clear();
  // Now process all frames
  for ( frameNumber = 0; frameNumber < inputParams->m_numberOfFrames; frameNumber++ ) {
    iCurrentFrameToProcess = int( frameNumber * fDistance0 );
    // read frames
    m_iFrameStore->m_frameNo = frameNumber;
    if ( m_iFrameStore->m_isFloat ) {
      printf( "float input not supported \n" );
      exit( -1 );
    } else {
      if ( m_iFrameStore->m_bitDepth == 8 ) {
        for ( int8_t c = 0; c < 3; c++ ) {
          auto& image = videoSrc.getFrame( frameNumber );
          auto& src   = image.getChannel( c );
          for ( size_t i = 0; i < m_iFrameStore->m_compSize[c]; i++ ) { m_iFrameStore->m_comp[c][i] = src[i]; }
        }
      } else {
        for ( int8_t c = 0; c < 3; c++ ) {
          auto& src = videoSrc.getFrame( frameNumber ).getChannel( c );
          for ( size_t i = 0; i < m_iFrameStore->m_compSize[c]; i++ ) { m_iFrameStore->m_ui16Comp[c][i] = src[i]; }
        }
      }
    }
    // optional forced clipping of the data given their defined range. This is
    // done here without consideration of the upconversion process.
    if ( inputParams->m_forceClipping == 1 ) { m_iFrameStore->clipRange(); }
    if ( errorRead == true ) {
      break;
    } else if ( inputParams->m_silentMode == false ) {
      printf( "%05d ", frameNumber );
    }
    currentFrame = m_iFrameStore;
    if ( m_croppedFrameStore != NULL ) {
      m_croppedFrameStore->copy( m_iFrameStore, m_cropOffsetLeft, m_cropOffsetTop,
                                 m_iFrameStore->m_width[hdrtoolslib::Y_COMP] + m_cropOffsetRight,
                                 m_iFrameStore->m_height[hdrtoolslib::Y_COMP] + m_cropOffsetBottom, 0, 0 );
      currentFrame = m_croppedFrameStore;
    }
    if ( ( ( m_iFrameStore->m_chromaFormat != m_oFrameStore->m_chromaFormat ) &&
           ( m_iFrameStore->m_colorSpace != hdrtoolslib::CM_RGB ) ) ||
         ( m_iFrameStore->m_chromaFormat != hdrtoolslib::CF_444 &&
           m_iFrameStore->m_colorPrimaries != m_oFrameStore->m_colorPrimaries ) ) {
      if ( m_filterInFloat == true ) {
        // Convert to different format if needed (integer to float)
        m_convertIQuantize->process( m_pFrameStore[0], currentFrame );
        if ( m_bUseChromaDeblocking == true )  // Perform deblocking
          m_frameFilter->process( m_pFrameStore[0] );
        // Chroma conversion
        m_convertFormatIn->process( m_convertFrameStore, m_pFrameStore[0] );
      } else {
        m_convertFormatIn->process( m_pFrameStore[0], currentFrame );
        // Here perform forced clipping of the data after upconversion of the
        // chroma components
        if ( inputParams->m_forceClipping == 2 ) m_pFrameStore[0]->clipRange();
        // Convert to different format if needed (integer to float)
        m_convertIQuantize->process( m_convertFrameStore, m_pFrameStore[0] );
      }
    } else {
      // Convert to different format if needed (integer to float)
      m_convertIQuantize->process( m_convertFrameStore, currentFrame );
    }
    // Add noise
    m_addNoise->process( m_convertFrameStore );
    if ( m_bUseWienerFiltering == true ) m_frameFilterNoise0->process( m_convertFrameStore );
    if ( m_bUse2DSepFiltering == true ) m_frameFilterNoise1->process( m_convertFrameStore );
    if ( m_bUseNLMeansFiltering == true ) m_frameFilterNoise2->process( m_convertFrameStore );
    currentFrame = m_convertFrameStore;
    m_frameScale->process( m_scaledFrame, currentFrame );
    currentFrame = m_scaledFrame;
    // Now perform a color format conversion
    // Output to m_pFrameStore memory with appropriate color space conversion
    // Note that the name of "forward" may be a bit of a misnomer.
    if ( !( input->m_iConstantLuminance != 0 &&
            ( input->m_colorSpace == hdrtoolslib::CM_YCbCr || input->m_colorSpace == hdrtoolslib::CM_ICtCp ) ) ) {
      m_colorTransform->process( m_pFrameStore[2], currentFrame );
      if ( m_useSingleTransferStep == false ) {
        m_inputTransferFunction->forward( m_pFrameStore[3], m_pFrameStore[2] );
        m_srcDisplayGammaAdjust->forward( m_pFrameStore[3] );
        m_normalizeFunction->forward( m_pFrameStore[1], m_pFrameStore[3] );
      } else {
        m_inputTransferFunction->forward( m_pFrameStore[1], m_pFrameStore[2] );
        m_srcDisplayGammaAdjust->forward( m_pFrameStore[1] );
      }
    } else {
      m_colorTransform->process( m_pFrameStore[2], currentFrame );
      m_normalizeFunction->forward( m_pFrameStore[1], m_pFrameStore[2] );
    }

    if ( m_changeColorPrimaries == true ) {
      m_colorSpaceConvert->process( m_colorSpaceFrame, m_pFrameStore[1] );
      m_outDisplayGammaAdjust->inverse( m_colorSpaceFrame );
      if ( m_oFrameStore->m_colorSpace == hdrtoolslib::CM_YCbCr ||
           m_oFrameStore->m_colorSpace == hdrtoolslib::CM_ICtCp ) {
        m_outputTransferFunction->inverse( m_pFrameStore[6], m_colorSpaceFrame );
        m_colorSpaceConvertMC->process( m_pFrameStore[4], m_pFrameStore[6] );
      } else {
        m_outputTransferFunction->inverse( m_pFrameStore[4], m_colorSpaceFrame );
        m_outDisplayGammaAdjust->inverse( m_colorSpaceFrame );
      }
    } else {
      // here we apply the output transfer function (to be fixed)
      m_outDisplayGammaAdjust->inverse( m_pFrameStore[1] );
      m_outputTransferFunction->inverse( m_pFrameStore[4], m_pFrameStore[1] );
    }
    if ( ( m_iFrameStore->m_chromaFormat != hdrtoolslib::CF_444 &&
           m_oFrameStore->m_chromaFormat != hdrtoolslib::CF_444 &&
           m_iFrameStore->m_colorPrimaries != m_oFrameStore->m_colorPrimaries ) ||
         ( m_iFrameStore->m_colorSpace == hdrtoolslib::CM_RGB && m_oFrameStore->m_colorSpace != hdrtoolslib::CM_RGB &&
           m_oFrameStore->m_chromaFormat ) ) {
      m_convertFormatOut->process( m_pFrameStore[5], m_pFrameStore[4] );
      m_convertProcess->process( m_oFrameStore, m_pFrameStore[5] );
    } else {
      m_convertProcess->process( m_oFrameStore, m_pFrameStore[4] );
    }
    // frame output
    m_outputFrame->copyFrame( m_oFrameStore );
    // m_outputFrame->writeOneFrame( m_outputFile, frameNumber,
    // m_outputFile->m_fileHeader, 0 );
    if ( m_oFrameStore->m_isFloat ) {
      printf( "float input not supported \n" );
      exit( -1 );
    } else {
      videoDst.resize( videoDst.getFrameCount() + 1 );
      auto&          image = videoDst.getFrames().back();
      PCCCOLORFORMAT format =
          m_oFrameStore->m_chromaFormat == hdrtoolslib::CF_420
              ? PCCCOLORFORMAT::YUV420
              : m_oFrameStore->m_colorSpace == hdrtoolslib::CM_RGB ? PCCCOLORFORMAT::RGB444 : PCCCOLORFORMAT::YUV444;
      image.resize( m_oFrameStore->m_width[hdrtoolslib::Y_COMP], m_oFrameStore->m_height[hdrtoolslib::Y_COMP], format );
      if ( m_oFrameStore->m_bitDepth == 8 ) {
        for ( int8_t c = 0; c < 3; c++ ) {
          auto& dst = videoDst.getFrame( frameNumber ).getChannel( c );
          for ( size_t i = 0; i < dst.size(); i++ ) { dst[i] = m_oFrameStore->m_comp[c][i]; }
        }
      } else if ( m_oFrameStore->m_bitDepth > 8 ) {
        for ( int8_t c = 0; c < 3; c++ ) {
          auto& dst = videoDst.getFrame( frameNumber ).getChannel( c );
          for ( size_t i = 0; i < dst.size(); i++ ) { dst[i] = m_oFrameStore->m_ui16Comp[c][i]; }
        }
      } else {
        printf( "output format not yet supported ( frame depht = %d \n", m_oFrameStore->m_bitDepth );
        exit( -1 );
      }
    }
  }  // end for frameNumber
}

template <typename T>
void PCCHDRToolsLibColorConverterImpl<T>::destroy() {
  // destroy
  if ( m_addNoise != NULL ) {
    delete m_addNoise;
    m_addNoise = NULL;
  }
  if ( m_frameFilter != NULL ) {
    delete m_frameFilter;
    m_frameFilter = NULL;
  }
  if ( m_frameFilterNoise0 != NULL ) {
    delete m_frameFilterNoise0;
    m_frameFilterNoise0 = NULL;
  }
  if ( m_frameFilterNoise1 != NULL ) {
    delete m_frameFilterNoise1;
    m_frameFilterNoise1 = NULL;
  }
  if ( m_frameFilterNoise2 != NULL ) {
    delete m_frameFilterNoise2;
    m_frameFilterNoise2 = NULL;
  }
  if ( m_convertFormatIn != NULL ) {
    delete m_convertFormatIn;
    m_convertFormatIn = NULL;
  }
  if ( m_convertFormatOut != NULL ) {
    delete m_convertFormatOut;
    m_convertFormatOut = NULL;
  }
  if ( m_convertProcess != NULL ) {
    delete m_convertProcess;
    m_convertProcess = NULL;
  }
  if ( m_convertIQuantize != NULL ) {
    delete m_convertIQuantize;
    m_convertIQuantize = NULL;
  }
  // output frame objects
  if ( m_oFrameStore != NULL ) {
    delete m_oFrameStore;
    m_oFrameStore = NULL;
  }
  // input frame objects
  if ( m_iFrameStore != NULL ) {
    delete m_iFrameStore;
    m_iFrameStore = NULL;
  }
  // processing frame objects
  for ( int i = 0; i < m_nFrameStores; i++ ) {
    if ( m_pFrameStore[i] != NULL ) {
      delete m_pFrameStore[i];
      m_pFrameStore[i] = NULL;
    }
  }
  if ( m_frameScale != NULL ) {
    delete m_frameScale;
    m_frameScale = NULL;
  }
  if ( m_colorSpaceConvert != NULL ) {
    delete m_colorSpaceConvert;
    m_colorSpaceConvert = NULL;
  }
  if ( m_colorSpaceConvertMC != NULL ) {
    delete m_colorSpaceConvertMC;
    m_colorSpaceConvertMC = NULL;
  }
  if ( m_colorSpaceFrame != NULL ) {
    delete m_colorSpaceFrame;
    m_colorSpaceFrame = NULL;
  }
  // Cropped frame store
  if ( m_croppedFrameStore != NULL ) {
    delete m_croppedFrameStore;
    m_croppedFrameStore = NULL;
  }
  if ( m_convertFrameStore != NULL ) {
    delete m_convertFrameStore;
    m_convertFrameStore = NULL;
  }
  if ( m_scaledFrame != NULL ) {
    delete m_scaledFrame;
    m_scaledFrame = NULL;
  }
  if ( m_inputFrame != NULL ) {
    delete m_inputFrame;
    m_inputFrame = NULL;
  }
  if ( m_outputFrame != NULL ) {
    delete m_outputFrame;
    m_outputFrame = NULL;
  }
  if ( m_colorTransform != NULL ) {
    delete m_colorTransform;
    m_colorTransform = NULL;
  }
  if ( m_normalizeFunction != NULL ) {
    delete m_normalizeFunction;
    m_normalizeFunction = NULL;
  }
  if ( m_outputTransferFunction != NULL ) {
    delete m_outputTransferFunction;
    m_outputTransferFunction = NULL;
  }
  if ( m_inputTransferFunction != NULL ) {
    delete m_inputTransferFunction;
    m_inputTransferFunction = NULL;
  }
  if ( m_srcDisplayGammaAdjust != NULL ) {
    delete m_srcDisplayGammaAdjust;
    m_srcDisplayGammaAdjust = NULL;
  }
  if ( m_outDisplayGammaAdjust != NULL ) {
    delete m_outDisplayGammaAdjust;
    m_outDisplayGammaAdjust = NULL;
  }
}

template class pcc::PCCHDRToolsLibColorConverterImpl<uint8_t>;
template class pcc::PCCHDRToolsLibColorConverterImpl<uint16_t>;

#endif  //~USE_HDRTOOLS
