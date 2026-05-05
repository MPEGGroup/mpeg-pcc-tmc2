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

#ifndef PCC_GS_HLS_BUILDER_H
#define PCC_GS_HLS_BUILDER_H

#include "PCCAdaptation.h"
#include "PCCCommon.h"
#include "PCCHighLevelSyntax.h"
#include "PCCGSEncoderParameters.h"
#include "PCCGSPatchInfo.h"

namespace pcc {

/**
 * @brief Builder class for High-Level Syntax (HLS) structures
 *
 * This class is responsible for initializing and updating V3C and Atlas
 * HLS structures used in the GS encoder.
 */
class PCCGSHlsBuilder {
 public:
  PCCGSHlsBuilder()  = default;
  ~PCCGSHlsBuilder() = default;

  /**
   * @brief Initialize all HLS structures
   * @param hls Reference to the high-level syntax to initialize
   * @param bitstreamStat Reference to bitstream statistics
   * @param params Encoder parameters
   * @param adaptationParams Adaptation parameters
   */
  void initialize( PCCHighLevelSyntax            &hls,
                   PCCBitstreamStat              &bitstreamStat,
                   const PCCGSEncoderParameters  &params,
                   const PCCAdaptationParameters &adaptationParams );

  bool isInitialized() const { return initialized_; }

  /**
   * @brief Add atlas frame HLS for the given patches
   * @param hls Reference to the high-level syntax
   * @param patches Vector of patch information
   * @param params Encoder parameters
   */
  void addAtlasFrame( PCCHighLevelSyntax                     &hls,
                      std::vector<GSEncoderPatchInformation> &patches,
                      const PCCGSEncoderParameters           &params );

  /**
   * @brief Update VPS based on map dimensions and packing settings
   * @param hls Reference to the high-level syntax
   * @param mapWidth Width of the generated maps
   * @param mapHeight Height of the generated maps
   * @param params Encoder parameters
   */
  void updateVps( PCCHighLevelSyntax &hls, size_t mapWidth, size_t mapHeight, const PCCGSEncoderParameters &params );

  /**
   * @brief Update VPS based on map dimensions and packing settings
   * @param hls Reference to the high-level syntax
   * @param mapWidth Width of the generated maps
   * @param mapHeight Height of the generated maps
   * @param params Encoder parameters
   */
  void updateAsps( PCCHighLevelSyntax &hls, size_t mapWidth, size_t mapHeight, const PCCGSEncoderParameters &params );

  void replaceVpsHls( PCCHighLevelSyntax &hls, const PCCGSEncoderParameters           &params );
  void replaceAtlasAspsHls( PCCHighLevelSyntax &hls, const PCCGSEncoderParameters           &params );
  void replaceAtlasAfpsHls( PCCHighLevelSyntax &hls, const PCCGSEncoderParameters           &params );
  void replaceAtlasAapsHls( PCCHighLevelSyntax &hls, const PCCAdaptationParameters &adaptationParams );
  void removeAtlasFrames(PCCHighLevelSyntax &hls );

 private:
  void initializeV3CHls( PCCHighLevelSyntax &hls, const PCCGSEncoderParameters &params );
  void initializeAtlasHls( PCCHighLevelSyntax            &hls,
                           const PCCGSEncoderParameters  &params,
                           const PCCAdaptationParameters &adaptationParams );
  void initializeAtlasAspsHls( PCCHighLevelSyntax &hls, const PCCGSEncoderParameters &params );
  void initializeAtlasAfpsHls( PCCHighLevelSyntax &hls, const PCCGSEncoderParameters &params );
  void initializeAtlasAapsHls( PCCHighLevelSyntax &hls, const PCCAdaptationParameters &adaptationParams );
  void layoutPackedMapsHls( PCCHighLevelSyntax &hls, const PCCGSEncoderParameters &params );
  void createGsCameraModelInfoSei( AtlasTileLayerRbsp &atgl, std::string path );

  uint8_t vpsParameterSetId_ = 0;
  uint8_t aspsParameterSetId_ = 0;
  uint8_t afpsParameterSetId_ = 0;
  uint8_t aapsParameterSetId_ = 0;

  size_t   frameIndex_ = 0;
  
  bool initialized_ = false;
};

}  // namespace pcc

#endif  // PCC_GS_HLS_BUILDER_H
