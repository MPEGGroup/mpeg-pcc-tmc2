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

#ifndef PCC_GS_MAP_GENERATOR_H
#define PCC_GS_MAP_GENERATOR_H

#include "PCCCommon.h"
#include "PCCPointSet.h"
#include "PCCHighLevelSyntax.h"
#include "PCCVideo.h"
#include "PCCImage.h"
#include "PCCGSEncoderParameters.h"
#include "PCCGSPatchInfo.h"

namespace pcc {

typedef pcc::PCCVideo<uint16_t, 3> PCCVideoGeometry;
typedef pcc::PCCVideo<uint16_t, 3> PCCVideoAttributes;
typedef pcc::PCCImage<uint16_t, 3> PCCVideoFrame;

/**
 * @brief Container for generated video frames
 */
struct PCCGSVideoFrames {
  PCCVideoGeometry                             geometry;
  std::vector<std::vector<PCCVideoAttributes>> attributes;  // [attributeIdx][partitionIdx]
  std::vector<PCCVideoAttributes>              packed;      // [packedMapIdx]
};

/**
 * @brief Generator class for creating geometry and attribute maps
 *
 * This class handles the generation of 2D maps from point cloud data
 * using the patch information.
 */
class PCCGSMapGenerator {
 public:
  PCCGSMapGenerator()  = default;
  ~PCCGSMapGenerator() = default;

  /**
   * @brief Generate all maps (geometry, attributes, packed)
   * @param source Input point cloud
   * @param patches Patch information
   * @param hls High-level syntax
   * @param mapWidth Width of the maps
   * @param mapHeight Height of the maps
   * @return Generated video frames
   */
  PCCGSVideoFrames generate( const PCCPointSet3                     &source,
                             const PCCGSEncoderParameters           &params,
                             std::vector<GSEncoderPatchInformation> &patches,
                             PCCHighLevelSyntax                     &hls,
                             size_t                                  mapWidth,
                             size_t                                  mapHeight );

 private:
  /**
   * @brief Generate geometry map
   */
  void generateGeometryMap( const PCCPointSet3                     &source,
                            std::vector<GSEncoderPatchInformation> &patches,
                            PCCVideoFrame                          &image,
                            const PCCGSEncoderParameters           &params,
                            size_t                                  topLeftX = 0,
                            size_t                                  topLeftY = 0 );

  /**
   * @brief Generate attribute map for a specific channel
   */
  void generateAttributeMap( std::vector<GSEncoderPatchInformation> &patches,
                             PCCVideoFrame                          &image,
                             size_t                                  componentIdx,
                             std::vector<PCCAttribute16bit>         &attribute,
                             size_t                                  topLeftX = 0,
                             size_t                                  topLeftY = 0 );

  /**
   * @brief Generate attribute maps for a region
   */
  void generateAttributeMapsForRegion( const PCCPointSet3                     &source,
                                       std::vector<GSEncoderPatchInformation> &patches,
                                       PCCVideoFrame                          &frame,
                                       AttributeTypeId                         attributeType,
                                       size_t                                  channelCount,
                                       size_t                                  partitionIdx,
                                       size_t                                  topLeftX,
                                       size_t                                  topLeftY );

  /**
   * @brief Get attribute values by type from source point cloud
   */
  std::vector<PCCAttribute16bit> getAttributeValuesByType( const PCCPointSet3 &source,
                                                           AttributeTypeId     attributeType,
                                                           size_t              partitionIdx );

  /**
   * @brief Generate packed maps and allocate video frames
   */
  void generatePackedMaps( PCCHighLevelSyntax              &hls,
                           const PCCGSEncoderParameters    &params,
                           std::vector<PCCVideoAttributes> &packedVideoFrames );

  void copyMapWithBBoxForLoD( PackedRegion &region, PCCImage<uint16_t, 3> &srcFrame, PCCImage<uint16_t, 3> &dstFrame );

  /**
   * @brief Fill geometry map with padding (m76580)
   */
  void geometryMapPadding( const PCCPointSet3                     &source,
                           const PCCGSEncoderParameters           &params,
                           std::vector<GSEncoderPatchInformation> &patches,
                           PCCVideoFrame                          &image,
                           size_t                                  topLeftX = 0,
                           size_t                                  topLeftY = 0 );
};

}  // namespace pcc

#endif  // PCC_GS_MAP_GENERATOR_H
