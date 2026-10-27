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

#ifndef PCC_GS_PATCH_GENERATOR_H
#define PCC_GS_PATCH_GENERATOR_H

#include "PCCCommon.h"
#include "PCCPointSet.h"
#include "PCCGSEncoderParameters.h"
#include "PCCAdaptation.h"
#include "PCCGSPatchInfo.h"

namespace pcc {

/**
 * @brief Result of patch generation
 */
struct PCCGSPatchResult {
  std::vector<GSEncoderPatchInformation> patches;
  size_t                                 mapWidth;
  size_t                                 mapHeight;
};

/**
 * @brief Atlas occupancy state — shared between packing functions
 */
struct AtlasOccupancyState {
  std::vector<bool> occupancyMap;
  std::vector<int>  uStart;
  size_t            occupancySizeU  = 0;
  size_t            occupancySizeV  = 0;
  int               maxOccupancyRow = 0;
  size_t            height          = 64;
};

/**
 * @brief Generator class for creating patches from point clouds
 *
 * This class handles the creation of 3D patches from point clouds,
 * packing them into 2D space, and computing point positions.
 */
class PCCGSPatchGenerator {
 public:
  PCCGSPatchGenerator()  = default;
  ~PCCGSPatchGenerator() = default;

  /**
   * @brief Generate patches from the input point cloud
   * @param source Input point cloud
   * @param params Encoder parameters
   * @return Patch generation result containing patches and map dimensions
   */
  PCCGSPatchResult generate( const PCCPointSet3            &source,
                             std::vector<uint8_t>          &lodAssignment,
                             const PCCGSEncoderParameters  &params,
                             const PCCAdaptationParameters &adaptationParams );

  std::map<uint8_t, std::array<size_t, 4>> &getLodRegionBBoxes() { return lodRegionBBoxes_; }

 private:
  /**
   * @brief Generate raw patches in 3D space by grouping points into bounding boxes
   */
  void generate3DPatches( const PCCPointSet3            &source,
                          std::vector<uint8_t>          &lodAssignment,
                          const PCCGSEncoderParameters  &params,
                          const PCCAdaptationParameters &adaptationParams );

  /**
   * @brief Generate 2D patch positions using CTC packing algorithm
   */
  void generate2DPatchesMorton( const PCCPointSet3 &source, const PCCGSEncoderParameters &params );

  /**
   * @brief Generate 2D patch positions for sortRawPointsMode==1 (m76580)
   */
  void generate2DPatchesBlockBased( const PCCPointSet3 &source, const PCCGSEncoderParameters &params );

  /**
   * @brief Pack raw patches into 2D space using Morton algorithm
   */
  void packRawPatches2DMorton( const PCCGSEncoderParameters &params, AtlasOccupancyState *state = nullptr );

  /**
   * @brief Pack raw patches into 2D space using CTC algorithm for sortRawPointsMode==1 (m76580)
   */
  void packRawPatches2DBlockBased( const PCCGSEncoderParameters &params, AtlasOccupancyState *state = nullptr );

  /**
   * @brief Pack raw patches into 2D space using simple algorithm
   */
  void packRawPatches2DSimple( const PCCGSEncoderParameters &params );

  /**
   * @brief Sort points in each patch using Morton order
   */
  void sortPointsInPatchMorton( const PCCPointSet3 &source );

  /**
   * @brief Sort points in each patch using block-based greedy order (m76580, sortRawPointsMode=1)
   */
  void sortPointsInPatchBlockBased( const PCCPointSet3 &source, const PCCGSEncoderParameters &params );

  /**
   * @brief Compute 2D positions for all points in a patch
   */
  void computePointPositionsIn2DMorton( GSEncoderPatchInformation &patchInfo, const PCCGSEncoderParameters &params );

  /**
   * @brief Compute 2D positions for all points in a patch for sortRawPointsMode==1 (m76580)
   */
  void computePointPositionsIn2DBlockBased( GSEncoderPatchInformation    &patchInfo,
                                            const PCCGSEncoderParameters &params );
  void nnsort( const PCCPointSet3 &source );
  /**
   * @brief Create patch information from bounding box and point indices
   */
  GSEncoderPatchInformation createPatchInfo( const PCCBox3D            &bbox,
                                             const uint8_t              lodMask,
                                             const std::vector<size_t> &pointIndices );

  /**
   * @brief Computes new patch list from input patches by splitting patches by quantization group
   */
  std::vector<GSEncoderPatchInformation> splitPatchesAdaptiveQuantization(
      const PCCPointSet3                           &source,
      const std::vector<GSEncoderPatchInformation> &patches );
  // Create patch information from bounding box and point indices
  uint32_t mergePatchesByAABB( const PCCPointSet3                     &source,
                               std::vector<GSEncoderPatchInformation> &RawPatches,
                               uint32_t                                patchSize,
                               uint32_t                                coordPrecision );

  std::vector<GSEncoderPatchInformation> patches_;
  size_t                                 mapWidth_{ 0 };
  size_t                                 mapHeight_{ 0 };

  std::map<uint8_t, std::array<size_t, 4>> lodRegionBBoxes_;  // key=lodMask, value={minU0,minV0,maxU0,maxV0}
};

}  // namespace pcc

#endif  // PCC_GS_PATCH_GENERATOR_H
