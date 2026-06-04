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
                          PCCLogger                     &logger ) {
  // 1. Initialize high-level syntax
  if ( !hlsBuilder_.isInitialized() ) {
    hlsBuilder_.initialize( hls_, bitstreamStat_, params, adaptationParams );
  } else {
    hls_.clearVideoBitstream();
    hlsBuilder_.replaceVpsHls( hls_, params );
    hlsBuilder_.replaceAtlasAspsHls( hls_, params );
    hlsBuilder_.replaceAtlasAfpsHls( hls_, params );
    hlsBuilder_.replaceAtlasAapsHls( hls_, adaptationParams );
    hlsBuilder_.removeAtlasFrames( hls_ );
  }
  // 2. Generate patches (3D and 2D)
  auto patchResult = patchGenerator_.generate( inputPointCloud, params, adaptationParams );

  // 3. Add patch information to atlas HLS
  hlsBuilder_.addAtlasFrame( hls_, patchResult.patches, params );

  // 4. Update VPS and ASPS based on map dimensions and packing settings
  hlsBuilder_.updateVps( hls_, patchResult.mapWidth, patchResult.mapHeight, params );
  hlsBuilder_.updateAsps( hls_, patchResult.mapWidth, patchResult.mapHeight, params );

  std::cout << "Number of points: " << inputPointCloud.getPointCount() << std::endl;
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
