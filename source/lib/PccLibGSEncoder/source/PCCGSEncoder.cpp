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

#include "PCCGSEncoder.h"
#include "PCCBitstreamWriter.h"

using namespace pcc;

PCCGSEncoder::PCCGSEncoder() {}

PCCGSEncoder::~PCCGSEncoder() {}

int PCCGSEncoder::encode( const PCCPointSet3            &inputPointCloud,
                          const PCCGSEncoderParameters  &params,
                          const PCCAdaptationParameters &adaptationParams,
                          SampleStreamV3CUnit           &ssvu,
                          std::vector<uint8_t>          &lodAssignment,
                          PCCLogger                     &logger ) {
  // 1. Initialize high-level syntax
  if ( !hlsBuilder_.isInitialized() ) {
    hlsBuilder_.initialize( hls_, bitstreamStat_, params, adaptationParams );
  } else {
    hls_.clearVideoBitstream();
    hlsBuilder_.replaceVpsHls( hls_, params, adaptationParams );
    hlsBuilder_.replaceAtlasAspsHls( hls_, params );
    hlsBuilder_.replaceAtlasAfpsHls( hls_, params );
    hlsBuilder_.replaceAtlasAapsHls( hls_, adaptationParams );
    hlsBuilder_.removeAtlasFrames( hls_ );
  }
  // 2. Generate patches (3D and 2D)
  auto patchResult = patchGenerator_.generate( inputPointCloud, lodAssignment, params, adaptationParams );

  // 3. Add patch information to atlas HLS
  hlsBuilder_.addAtlasFrame( hls_, patchResult.patches, params );

  // 4. Update VPS and ASPS based on map dimensions and packing settings
  hlsBuilder_.updateVps( hls_, patchResult.mapWidth, patchResult.mapHeight, params );
  hlsBuilder_.updateAsps( hls_, patchResult.mapWidth, patchResult.mapHeight, params );

  std::cout << "Number of points: " << inputPointCloud.getPointCount() << std::endl;

  // LoD: fill packed-region bboxes from atlas packing results, then generate maps.
  // Must run after updateVps() (regions exist) and before mapGenerator_.generate() (reads lodSourceBBoxes_)
  if ( params.lodEnabled_ && ( params.packGeometry_ || params.packAttributes_ ) ) {
    fillPackingInfoForLoD( hls_, params );
  }

  // 5. Generate maps (geometry, attributes, packed)
  auto videoFrames = mapGenerator_.generate( inputPointCloud, params, patchResult.patches, hls_, patchResult.mapWidth,
                                             patchResult.mapHeight );

  // 6. Encode videos
  videoEncoder_.encode( videoFrames, hls_, params );

  // 7. Generate V3C units and finalize bitstream
  PCCBitstreamWriter bitstreamWriter;
  bitstreamWriter.setLogger( logger );
  bitstreamWriter.encode( hls_, ssvu );

  return 0;
}

// Post-packing: fill packing-information region bounding boxes from the per-LoD atlas coordinates
// that were accumulated into each tile's lodRegionBBoxes_ during packRawPointsPatch().
void pcc::PCCGSEncoder::fillPackingInfoForLoD( PCCHighLevelSyntax &hls, const PCCGSEncoderParameters &params ) {
  static constexpr size_t atlasIdx = 0;

  auto &vps = hls.getVpsList().front();
  auto &pve = vps.getVpsPackedVideoExtension();

  if ( pve.isPresent() && pve.getPackedVideoPresentFlag( atlasIdx ) ) {
    auto &pin = pve.getPackingInformation( atlasIdx );
    if ( pin.getLodEnabledFlag() ) {
      // Merge bounding boxes across all frames for each LoD mask.
      std::map<uint8_t, std::array<size_t, 4>> mergedBBoxes;
      for ( auto &kv : patchGenerator_.getLodRegionBBoxes() ) {
        const uint8_t                lodMask  = kv.first;
        const std::array<size_t, 4> &bb       = kv.second;
        auto                        &mergedBB = mergedBBoxes[lodMask];
        if ( mergedBB[2] == 0 && mergedBB[3] == 0 ) {
          mergedBB = bb;
        } else {
          mergedBB[0] = std::min( mergedBB[0], bb[0] );
          mergedBB[1] = std::min( mergedBB[1], bb[1] );
          mergedBB[2] = std::max( mergedBB[2], bb[2] );
          mergedBB[3] = std::max( mergedBB[3], bb[3] );
        }
      }

      // Write pixel-space coordinates into each matching region.
      for ( size_t ri = 0; ri <= pin.getRegionsCountMinus1(); ++ri ) {
        auto   &region  = pin.getRegion( ri );
        uint8_t lodMask = region.getRegionLodMask();
        if ( lodMask == 0 ) { continue; }
        auto it = mergedBBoxes.find( lodMask );
        if ( it == mergedBBoxes.end() ) { continue; }
        const auto    &bb       = it->second;
        const size_t   res      = params.occupancyResolution_;
        const uint16_t left     = static_cast<uint16_t>( bb[0] * res );
        const uint16_t top      = static_cast<uint16_t>( bb[1] * res );
        const uint16_t widthM1  = static_cast<uint16_t>( ( bb[2] - bb[0] ) * res - 1 );
        const uint16_t heightM1 = static_cast<uint16_t>( ( bb[3] - bb[1] ) * res - 1 );
        // topLeftX/Y = packed-frame position, assigned by layoutPackedMapsHls() after this function.
        region.setWidthMinus1( widthM1 );
        region.setHeightMinus1( heightM1 );
        // unpackTopLeftX/Y = atlas destination when unpacking (atlas band origin).
        region.setUnpackTopLeftX( left );
        region.setUnpackTopLeftY( top );

        std::cout << "LoD region lodMask=" << static_cast<int>( lodMask ) << " topLeft=(" << left << "," << top << ")"
                  << " size=(" << widthM1 + 1 << "x" << heightM1 + 1 << ")\n";
      }
    }
  }

  // Assign packed-region positions now that LoD region sizes are known.
  hlsBuilder_.layoutPackedMapsHls( hls_, params );
}
