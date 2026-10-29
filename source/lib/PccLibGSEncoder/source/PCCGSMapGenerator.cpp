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

#include "PCCGSMapGenerator.h"
#include "PCCGSEncoderConstant.h"
#include <iostream>
#include <map>

using namespace pcc;

namespace {
constexpr size_t ATLAS_INDEX = 0;
}  // namespace

PCCGSVideoFrames PCCGSMapGenerator::generate( const PCCPointSet3                     &source,
                                              const PCCGSEncoderParameters           &params,
                                              std::vector<GSEncoderPatchInformation> &patches,
                                              PCCHighLevelSyntax                     &hls,
                                              size_t                                  mapWidth,
                                              size_t                                  mapHeight ) {
  PCCGSVideoFrames frames;
  auto             vps = hls.getVpsList().front();
  attributeBitDepth2D_ = params.attributeBitDepth2D_;

  // Generate geometry video if present
  if ( vps.getGeometryVideoPresentFlag( ATLAS_INDEX ) ) {
    frames.geometry.resize( 1 );
    frames.geometry.getFrame( 0 ).resize( mapWidth, mapHeight, PCCCOLORFORMAT::YUV420 );
    frames.geometry.getFrame( 0 ).set( 0 );
    generateGeometryMap( source, patches, frames.geometry.getFrame( 0 ), params );
  }

  // Generate attribute video if present
  if ( vps.getAttributeVideoPresentFlag( ATLAS_INDEX ) ) {
    auto  &ai             = vps.getAttributeInformation( ATLAS_INDEX );
    size_t attributeCount = ai.getAttributeCount();
    frames.attributes.resize( attributeCount );

    for ( size_t attributeIdx = 0; attributeIdx < attributeCount; attributeIdx++ ) {
      size_t partitionCount = ai.getAttributeDimensionPartitionsMinus1( attributeIdx );
      auto   attributeType  = ai.getAttributeTypeId( attributeIdx );
      frames.attributes[attributeIdx].resize( partitionCount + 1 );

      for ( size_t partitionIdx = 0; partitionIdx <= partitionCount; partitionIdx++ ) {
        frames.attributes[attributeIdx][partitionIdx].resize( 1 );
        frames.attributes[attributeIdx][partitionIdx].getFrame( 0 ).resize( mapWidth, mapHeight,
                                                                            PCCCOLORFORMAT::YUV420 );
        frames.attributes[attributeIdx][partitionIdx].getFrame( 0 ).set( 0 );
        size_t channelCount = ai.getAttributePartitionChannelsMinus1( attributeIdx, partitionIdx ) + 1;

        generateAttributeMapsForRegion( source, patches, frames.attributes[attributeIdx][partitionIdx].getFrame( 0 ),
                                        attributeType, channelCount, partitionIdx, 0, 0 );
      }
    }
  }

  // Generate packed video if present
  if ( vps.getVpsPackedVideoExtension().getPackedVideoPresentFlag( ATLAS_INDEX ) ) {
    generatePackedMaps( hls, params, frames.packed );

    auto &pin       = vps.getVpsPackedVideoExtension().getPackingInformation( ATLAS_INDEX );
    auto  mapsCount = pin.getPackedMapCountMinus1() + 1;

    // Check if any LoD regions are present; if so pre-generate full atlas frames
    // to use as blit sources (one geometry frame + one per attribute/partition pair).
    bool hasLodRegions = false;
    for ( size_t ri = 0; ri <= pin.getRegionsCountMinus1(); ++ri ) {
      if ( pin.getRegion( ri ).getRegionLodMask() != 0 ) {
        hasLodRegions = true;
        break;
      }
    }

    PCCVideoFrame                                      atlasGeoFrame;
    std::map<std::pair<size_t, size_t>, PCCVideoFrame> atlasAttrFrames;  // key: (attributeIdx, partitionIdx)

    if ( hasLodRegions ) {
      atlasGeoFrame.resize( mapWidth, mapHeight, PCCCOLORFORMAT::YUV420 );
      atlasGeoFrame.set( 0 );
      generateGeometryMap( source, patches, atlasGeoFrame, params );

      auto &ai = vps.getAttributeInformation( ATLAS_INDEX );
      for ( size_t attributeIdx = 0; attributeIdx < ai.getAttributeCount(); attributeIdx++ ) {
        size_t partitionCount = ai.getAttributeDimensionPartitionsMinus1( attributeIdx );
        auto   attributeType  = ai.getAttributeTypeId( attributeIdx );
        for ( size_t partitionIdx = 0; partitionIdx <= partitionCount; partitionIdx++ ) {
          auto &f = atlasAttrFrames[{ attributeIdx, partitionIdx }];
          f.resize( mapWidth, mapHeight, PCCCOLORFORMAT::YUV420 );
          f.set( 0 );
          size_t channelCount = ai.getAttributePartitionChannelsMinus1( attributeIdx, partitionIdx ) + 1;
          generateAttributeMapsForRegion( source, patches, f, attributeType, channelCount, partitionIdx, 0, 0 );
        }
      }
    }

    for ( size_t packedMapIndex = 0; packedMapIndex < mapsCount; packedMapIndex++ ) {
      for ( size_t regionIndex = 0; regionIndex <= pin.getRegionsCountMinus1(); ++regionIndex ) {
        auto &region = pin.getRegion( regionIndex );
        if ( region.getPackedMapIndex() != packedMapIndex ) { continue; }

        auto &frame = frames.packed[packedMapIndex].getFrame( region.getConstituentFrameIndex() );

        if ( region.getRegionLodMask() != 0 ) {
          // LoD region: copy a bbox of the atlas frame into the packed frame.
          if ( region.getTypeIdMinus2() == V3C_GVD - 2 ) {
            copyMapWithBBoxForLoD( region, atlasGeoFrame, frame );
          } else if ( region.getTypeIdMinus2() == V3C_AVD - 2 ) {
            size_t attributeIdx = region.getAttrIndex();
            size_t partitionIdx = region.getAttrPartitionIndex();
            copyMapWithBBoxForLoD( region, atlasAttrFrames.at( { attributeIdx, partitionIdx } ), frame );
          }
        } else {
          // Non-LoD region: generate directly into the packed frame using atlas-layout offsets.
          size_t topLeftX = region.getTopLeftX();
          size_t topLeftY = region.getTopLeftY();

          if ( region.getTypeIdMinus2() == V3C_GVD - 2 ) {
            generateGeometryMap( source, patches, frame, params, topLeftX, topLeftY );
          } else if ( region.getTypeIdMinus2() == V3C_AVD - 2 ) {
            size_t attributeIdx  = region.getAttrIndex();
            size_t partitionIdx  = region.getAttrPartitionIndex();
            auto   attributeType = vps.getAttributeInformation( ATLAS_INDEX ).getAttributeTypeId( attributeIdx );
            size_t channelCount  = vps.getAttributeInformation( ATLAS_INDEX )
                                       .getAttributePartitionChannelsMinus1( attributeIdx, partitionIdx ) +
                                   1;
            generateAttributeMapsForRegion( source, patches, frame, attributeType, channelCount, partitionIdx, topLeftX,
                                            topLeftY );
          }
        }
      }
    }
  }

  return frames;
}

void PCCGSMapGenerator::copyMapWithBBoxForLoD( PackedRegion          &region,
                                               PCCImage<uint16_t, 3> &srcFrame,
                                               PCCImage<uint16_t, 3> &dstFrame ) {
  size_t bboxLeft = region.getUnpackTopLeftX();
  size_t bboxTop  = region.getUnpackTopLeftY();
  size_t bboxW    = region.getWidthMinus1() + 1;
  size_t bboxH    = region.getHeightMinus1() + 1;
  size_t topLeftX = region.getTopLeftX();
  size_t topLeftY = region.getTopLeftY();

  for ( size_t y = 0; y < bboxH; ++y )
    for ( size_t x = 0; x < bboxW; ++x )
      dstFrame.setValue( 0, topLeftX + x, topLeftY + y, srcFrame.getValue( 0, bboxLeft + x, bboxTop + y ) );
}

void PCCGSMapGenerator::generateGeometryMap( const PCCPointSet3                     &source,
                                             std::vector<GSEncoderPatchInformation> &patches,
                                             PCCVideoFrame                          &image,
                                             const PCCGSEncoderParameters           &params,
                                             size_t                                  topLeftX,
                                             size_t                                  topLeftY ) {
  auto &positions = source.getPositions();

  for ( const auto &patchInfo : patches ) {
    auto      u1                          = patchInfo.patch.u1_;
    auto      v1                          = patchInfo.patch.v1_;
    auto      d1                          = patchInfo.patch.d1_;
    const int geometryBitDepth2D_MaxValue = pow( 2, params.geometryBitDepth2D_ );
    const int geometryBitDepth2D_MidValue = geometryBitDepth2D_MaxValue / 2;
    for ( size_t p = 0; p < patchInfo.patch.getNumberOfRawPoints(); ++p ) {
      size_t      pointIndex = patchInfo.pointsIndicesInPatch[p];
      const auto &pos        = positions[pointIndex];

      // component 0
      {
        size_t u = patchInfo.pointsPositionsIn2D[0][p].x + topLeftX;
        size_t v = patchInfo.pointsPositionsIn2D[0][p].y + topLeftY;
        assert( u < image.getWidth() && v < image.getHeight() );
        uint16_t val = uint16_t( pos.x() - u1 );
        if ( params.enableGeometryPrediction_ ) {
          if ( p != 0 && p % params.geometryPredictionSegmentVal_ != 0 ) {
            size_t      pointIndex_prev = patchInfo.pointsIndicesInPatch[p - 1];
            const auto &pos_prev        = positions[pointIndex_prev];
            uint16_t    prev_val        = uint16_t( pos_prev.x() - u1 );
            int32_t     diff            = (int32_t)val - (int32_t)prev_val;
            val = ( diff + geometryBitDepth2D_MidValue + geometryBitDepth2D_MaxValue ) % geometryBitDepth2D_MaxValue;
          }
        }
        image.setValue( 0, u, v, uint16_t( val ) );
      }
      // component 1
      {
        size_t u = patchInfo.pointsPositionsIn2D[1][p].x + topLeftX;
        size_t v = patchInfo.pointsPositionsIn2D[1][p].y + topLeftY;
        assert( u < image.getWidth() && v < image.getHeight() );
        uint16_t val = uint16_t( pos.y() - v1 );
        if ( params.enableGeometryPrediction_ ) {
          if ( p != 0 && p % params.geometryPredictionSegmentVal_ != 0 ) {
            size_t      pointIndex_prev = patchInfo.pointsIndicesInPatch[p - 1];
            const auto &pos_prev        = positions[pointIndex_prev];
            uint16_t    prev_val        = uint16_t( pos_prev.y() - v1 );
            int32_t     diff            = (int32_t)val - (int32_t)prev_val;
            val = ( diff + geometryBitDepth2D_MidValue + geometryBitDepth2D_MaxValue ) % geometryBitDepth2D_MaxValue;
          }
        }
        image.setValue( 0, u, v, uint16_t( val ) );
      }
      // component 2
      {
        size_t u = patchInfo.pointsPositionsIn2D[2][p].x + topLeftX;
        size_t v = patchInfo.pointsPositionsIn2D[2][p].y + topLeftY;
        assert( u < image.getWidth() && v < image.getHeight() );
        uint16_t val = uint16_t( pos.z() - d1 );
        if ( params.enableGeometryPrediction_ ) {
          if ( p != 0 && p % params.geometryPredictionSegmentVal_ != 0 ) {
            size_t      pointIndex_prev = patchInfo.pointsIndicesInPatch[p - 1];
            const auto &pos_prev        = positions[pointIndex_prev];
            uint16_t    prev_val        = uint16_t( pos_prev.z() - d1 );
            int32_t     diff            = (int32_t)val - (int32_t)prev_val;
            val = ( diff + geometryBitDepth2D_MidValue + geometryBitDepth2D_MaxValue ) % geometryBitDepth2D_MaxValue;
          }
        }
        image.setValue( 0, u, v, uint16_t( val ) );
      }
    }
  }
}

void PCCGSMapGenerator::generateAttributeMap( std::vector<GSEncoderPatchInformation> &patches,
                                              PCCVideoFrame                          &image,
                                              size_t                                  componentIdx,
                                              std::vector<PCCAttribute16bit>         &attribute,
                                              size_t                                  topLeftX,
                                              size_t                                  topLeftY,
                                              bool                                    isShAttribute ) {
  for ( const auto &patchInfo : patches ) {
    const uint16_t dcOffset =
        isShAttribute ? getShDcOffset( attributeBitDepth2D_, patchInfo.patch.shBitdepthDelta_ ) : 0;
    for ( size_t p = 0; p < patchInfo.patch.getNumberOfRawPoints(); ++p ) {
      size_t     pointIndex     = patchInfo.pointsIndicesInPatch[p];
      const auto attributeValue = attribute[pointIndex][componentIdx];
      size_t     u              = patchInfo.pointsPositionsIn2D[componentIdx][p].x + topLeftX;
      size_t     v              = patchInfo.pointsPositionsIn2D[componentIdx][p].y + topLeftY;

      assert( u < image.getWidth() && v < image.getHeight() );
      image.setValue( 0, u, v, uint16_t( uint16_t( attributeValue ) + dcOffset ) );
    }
  }
}

void PCCGSMapGenerator::generateAttributeMapsForRegion( const PCCPointSet3                     &source,
                                                        std::vector<GSEncoderPatchInformation> &patches,
                                                        PCCVideoFrame                          &frame,
                                                        AttributeTypeId                         attributeType,
                                                        size_t                                  channelCount,
                                                        size_t                                  partitionIdx,
                                                        size_t                                  topLeftX,
                                                        size_t                                  topLeftY ) {
  auto attributeValues = getAttributeValuesByType( source, attributeType, partitionIdx );
  if ( attributeValues.empty() ) {
    assert( 0 );
    return;
  }
  for ( size_t channelIdx = 0; channelIdx < channelCount; channelIdx++ ) {
    std::cout << "Generating attribute map for partition index " << partitionIdx << ", channel index " << channelIdx
              << std::endl;
    generateAttributeMap( patches, frame, channelIdx, attributeValues, topLeftX, topLeftY,
                          attributeType == ATTR_SPHERICAL_HARMONICS );
  }
}

std::vector<PCCAttribute16bit> PCCGSMapGenerator::getAttributeValuesByType( const PCCPointSet3 &source,
                                                                            AttributeTypeId     attributeType,
                                                                            size_t              partitionIdx ) {
  switch ( attributeType ) {
    case ATTR_TRANSPARENCY: return source.getOpacities();
    case ATTR_SPHERICAL_HARMONICS: return source.getSphericalHarmonicPartition( partitionIdx );
    case ATTR_SCALE: return source.getScales();
    case ATTR_ROTATION: return source.getRotationPartition( partitionIdx );
    default: return {};
  }
}

void PCCGSMapGenerator::generatePackedMaps( PCCHighLevelSyntax              &hls,
                                            const PCCGSEncoderParameters    &params,
                                            std::vector<PCCVideoAttributes> &packedVideoFrames ) {
  if ( params.temporalPacking_ )
    std::cout << "Generate temporally-packed maps" << std::endl;
  else
    std::cout << "Generate spatially-packed maps" << std::endl;

  auto &vps = hls.getVps();
  auto &pin = vps.getVpsPackedVideoExtension().getPackingInformation( ATLAS_INDEX );

  if ( !params.temporalPacking_ ) {
    packedVideoFrames.resize( pin.getPackedMapCountMinus1() + 1 );

    for ( uint8_t packedMapIndex = 0; packedMapIndex <= pin.getPackedMapCountMinus1(); ++packedMapIndex ) {
      size_t width  = pin.getLodEnabledFlag()
                          ? static_cast<size_t>( packedMapIndex < params.packedMapColumnsCount_.size()
                                                     ? params.packedMapColumnsCount_[packedMapIndex]
                                                     : 1 ) *
                                vps.getFrameWidth( ATLAS_INDEX )
                          : 0;
      size_t height = 0;

      for ( size_t regionIndex = 0; regionIndex <= pin.getRegionsCountMinus1(); ++regionIndex ) {
        auto &region = pin.getRegion( regionIndex );

        if ( region.getPackedMapIndex() != packedMapIndex ) { continue; }

        // All regions (LoD and non-LoD) are sized by their packed position + extent.
        // topLeftX/Y is set by layoutPackedMapsHls(); widthMinus1/heightMinus1 are the region
        // semantic size (atlas-wide for non-LoD, bbox-wide for LoD).
        width  = std::max( width, static_cast<size_t>( region.getTopLeftX() ) + region.getWidthMinus1() + 1 );
        height = std::max( height, static_cast<size_t>( region.getTopLeftY() ) + region.getHeightMinus1() + 1 );
      }
      if ( width % 2 == 1 ) { ++width; }
      if ( height % 2 == 1 ) { ++height; }

      auto &video = packedVideoFrames[packedMapIndex];
      video.resize( 1 );
      video[0].resize( width, height, YUV420 );
      std::cout << "  packedMapIndex = " << int( packedMapIndex ) << ", frameIndex=" << 0 << ", width=" << width
                << ", height=" << height << std::endl;
    }
  } else {
    size_t packedMapCount = pin.getPackedMapCountMinus1() + 1;
    packedVideoFrames.resize( packedMapCount );  // allocate one frame per packed map

    auto width  = int{};
    auto height = int{};

    for ( size_t packedMapIndex = 0; packedMapIndex < packedMapCount; packedMapIndex++ ) {
      // Count regions belonging to this packed map
      std::vector<size_t> regionIndices;
      for ( size_t regionIndex = 0; regionIndex <= pin.getRegionsCountMinus1(); ++regionIndex ) {
        if ( pin.getRegion( regionIndex ).getPackedMapIndex() == packedMapIndex ) {
          regionIndices.push_back( regionIndex );
        }
      }

      auto &video = packedVideoFrames[packedMapIndex];

      // Allocate one frame per region
      video.resize( regionIndices.size() );

      // Loop over all regions assigned to this packed map
      for ( size_t f = 0; f < regionIndices.size(); f++ ) {
        auto &region = pin.getRegion( regionIndices[f] );

        size_t width  = region.getWidthMinus1() + 1;
        size_t height = region.getHeightMinus1() + 1;

        video[f].resize( width, height, YUV420 );
        video[f].set( 0 );
      }

      std::cout << "  packedMapIndex = " << int( packedMapIndex ) << ", frameIndex=" << 0 << ", width=" << width
                << ", height=" << height << std::endl;
    }
  }
}
