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
#ifndef PCCVideoEncoder_h
#define PCCVideoEncoder_h

#include "PCCCommon.h"

namespace pcc {

template <typename T, size_t N>
class PCCVideo;
class PCCContext;
class PCCVideoBitstream;
class PCCLogger;

class PCCVideoEncoder {
 public:
  PCCVideoEncoder();
  ~PCCVideoEncoder();
  template <typename T>
  void patchColorSubsmple( PCCVideo<T, 3>&    video,
                           PCCContext&        contexts,
                           const size_t       width,
                           const size_t       height,
                           const std::string& configColorSpace,
                           const std::string& colorSpaceConversionPath,
                           const std::string& fileName );
  template <typename T>
  bool compress( PCCVideo<T, 3>&    video,
                 const std::string& path,
                 const int          qp,
                 PCCVideoBitstream& bitstream,
                 const std::string& encoderConfig,
                 const std::string& encoderPath,
                 PCCCodecId         codecId,
                 bool               byteStreamVideoCoder,
                 PCCContext&        contexts,
                 const size_t       nbyte,
                 const bool         use444CodecIo,
                 const bool         use3dmv,
                 const bool         usePccRDO,
                 const size_t       shvcLayerIndex,
                 const size_t       shvcRateX,
                 const size_t       shvcRateY,
                 const size_t       internalBitDepth,
                 const bool         useConversion,
                 const bool         keepIntermediateFiles             = false,
                 const std::string& colorSpaceConversionConfig        = "",
                 const std::string& inverseColorSpaceConversionConfig = "",
                 const std::string& colorSpaceConversionPath          = "",
                 const size_t       downsamplingFilter                = 4,
                 const size_t       upsamplingFilter                  = 0,
                 const bool         patchColorSubsampling             = false );

  void setLogger( PCCLogger& logger ) { logger_ = &logger; }

 private:
  PCCLogger* logger_ = nullptr;
};

};  // namespace pcc

#endif /* PCCVideoEncoder_h */
