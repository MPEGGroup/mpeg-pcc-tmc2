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
#ifndef PCCHDRToolsLibColorConverterImpl_h
#define PCCHDRToolsLibColorConverterImpl_h

#include "PCCCommon.h"

#ifdef USE_HDRTOOLS

#if defined( WIN32 )
#undef UINT
#undef FLOAT
#undef CP_NONE
#endif

#include "Global.H"
#include "HDRConvert.H"
#include "Input.H"
#include "Output.H"
#include "Frame.H"
#include "IOFunctions.H"
#include "Convert.H"
#include "ConvertColorFormat.H"
#include "ColorTransform.H"
#include "TransferFunction.H"
#include "AddNoise.H"
#include "FrameScale.H"
#include "FrameFilter.H"
#include "DisplayGammaAdjust.H"
#include "ToneMapping.H"
#include "ProjectParameters.H"

#include "PCCVideo.h"

namespace pcc {

template <class T>
class PCCHDRToolsLibColorConverterImpl {
 public:
  PCCHDRToolsLibColorConverterImpl();
  ~PCCHDRToolsLibColorConverterImpl();
  void convert( std::string configFile, PCCVideo<T, 3>& videoSrc, PCCVideo<T, 3>& videoDst );

 private:
  void init( ProjectParameters* inputParams );
  void process( ProjectParameters* inputParams, PCCVideo<T, 3>& videoSrc, PCCVideo<T, 3>& videoDst );
  void destroy();

  int                 m_nFrameStores;
  hdrtoolslib::Frame* m_oFrameStore;                 // picture storage for output frames
  hdrtoolslib::Frame* m_iFrameStore;                 // picture storage for input frames
  hdrtoolslib::Frame* m_pFrameStore[7];              // picture storage for processing
                                                     // frames
  hdrtoolslib::Frame*          m_croppedFrameStore;  // cropped frame store
  hdrtoolslib::Frame*          m_convertFrameStore;  // pixel type conversion frame store
  hdrtoolslib::Frame*          m_colorSpaceFrame;
  hdrtoolslib::Frame*          m_scaledFrame;
  hdrtoolslib::FrameFilter*    m_frameFilter;
  hdrtoolslib::FrameFilter*    m_frameFilterNoise0;
  hdrtoolslib::FrameFilter*    m_frameFilterNoise1;
  hdrtoolslib::FrameFilter*    m_frameFilterNoise2;
  hdrtoolslib::Convert*        m_convertProcess;             // Final Conversion process
  hdrtoolslib::Convert*        m_convertIQuantize;           // Initial Conversion process
  hdrtoolslib::ColorTransform* m_colorTransform;             // Color space conversion
  hdrtoolslib::ColorTransform* m_colorSpaceConvert;          // Color space conversion
                                                             // (second)
  hdrtoolslib::ColorTransform* m_colorSpaceConvertMC;        // Color space conversion
                                                             // (final)
  hdrtoolslib::ConvertColorFormat* m_convertFormatIn;        // Input Chroma format
                                                             // conversion
  hdrtoolslib::ConvertColorFormat* m_convertFormatOut;       // Output chroma format
                                                             // conversion
  hdrtoolslib::DisplayGammaAdjust* m_srcDisplayGammaAdjust;  // Source Display
                                                             // Gamma adjustment
  hdrtoolslib::DisplayGammaAdjust* m_outDisplayGammaAdjust;  // Output Display
                                                             // Gamma adjustment
  hdrtoolslib::AddNoise*         m_addNoise;                 // Noise Addition
  hdrtoolslib::FrameScale*       m_frameScale;
  hdrtoolslib::TransferFunction* m_normalizeFunction;       // Data normalization for
                                                            // OpenEXR inputs
  hdrtoolslib::TransferFunction* m_inputTransferFunction;   // Transfer function
  hdrtoolslib::TransferFunction* m_outputTransferFunction;  // Transfer function
  bool                           m_useSingleTransferStep;
  int                            m_startFrame;
  bool                           m_filterInFloat;
  bool                           m_changeColorPrimaries;
  bool                           m_bUseChromaDeblocking;
  bool                           m_bUseWienerFiltering;
  bool                           m_bUse2DSepFiltering;
  bool                           m_bUseNLMeansFiltering;
  bool                           m_b2DSepMode;
  int                            m_cropOffsetLeft;
  int                            m_cropOffsetTop;
  int                            m_cropOffsetRight;
  int                            m_cropOffsetBottom;
  int                            m_width;
  int                            m_height;
  hdrtoolslib::IOVideo*          m_inputFile;
  hdrtoolslib::IOVideo*          m_outputFile;
  int                            m_cropWidth;
  int                            m_cropHeight;
  hdrtoolslib::Input*            m_inputFrame;   // input frames
  hdrtoolslib::Output*           m_outputFrame;  // output frames
};

};  // namespace pcc

#endif  //~USE_HDRTOOLS

#endif /* PCCHDRToolsLibColorConverterImpl_h */
