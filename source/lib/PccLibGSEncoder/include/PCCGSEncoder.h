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

#ifndef PCC_GS_ENCODER_H
#define PCC_GS_ENCODER_H

#include "PCCAdaptation.h"
#include "PCCCommon.h"
#include "PCCHighLevelSyntax.h"
#include "PCCPointSet.h"
#include "PCCGSEncoderParameters.h"
#include "PCCGSPatchInfo.h"
#include "PCCGSHlsBuilder.h"
#include "PCCGSPatchGenerator.h"
#include "PCCGSMapGenerator.h"
#include "PCCGSVideoEncoder.h"

namespace pcc {

/**
 * @brief Main GS Encoder class (Facade pattern)
 *
 * This class orchestrates the encoding process by delegating work
 * to specialized component classes:
 * - PCCGSHlsBuilder: Manages HLS initialization and updates
 * - PCCGSPatchGenerator: Generates patches from point clouds
 * - PCCGSMapGenerator: Creates geometry and attribute maps
 * - PCCGSVideoEncoder: Encodes maps into video bitstreams
 */
class PCCGSEncoder {
 public:
  PCCGSEncoder();
  ~PCCGSEncoder();

  /**
   * @brief Encode a point cloud into a V3C bitstream
   * @param inputPointCloud Input point cloud to encode
   * @param params Encoder parameters
   * @param adaptationParams Adaptation parameters
   * @param ssvu Output sample stream V3C unit
   * @param logger Logger for bitstream writing
   * @return 0 on success, non-zero on failure
   */
  int encode( const PCCPointSet3            &inputPointCloud,
              const PCCGSEncoderParameters  &params,
              const PCCAdaptationParameters &adaptationParams,
              SampleStreamV3CUnit           &ssvu,
              PCCLogger                     &logger );

 private:
  // Component classes
  PCCGSHlsBuilder     hlsBuilder_;
  PCCGSPatchGenerator patchGenerator_;
  PCCGSMapGenerator   mapGenerator_;
  PCCGSVideoEncoder   videoEncoder_;

  // State
  PCCHighLevelSyntax hls_;
  PCCBitstreamStat   bitstreamStat_;
};

}  // namespace pcc

#endif  // PCC_GS_ENCODER_H