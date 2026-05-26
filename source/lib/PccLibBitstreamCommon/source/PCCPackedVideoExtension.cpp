/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2025, ISO/IEC
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
#include "PCCPackedVideoExtension.h"
#include "PCCV3CParameterSet.h"

namespace pcc {
using namespace std::string_literals;

void VpsPackedVideoExtension::dumpTo( std::ostream &stream, const std::string &indent ) const {
  stream << "VpsPackedVideoExtension {\n";
  stream << indent << "  version: " << unsigned{ version_ } << '\n';
  stream << indent << "  temporalPackingEnabledFlag: " << std::boolalpha << temporalPackingEnabledFlag_ << '\n';
  stream << indent << "  temporalPackingMode: " << static_cast<unsigned>( temporalPackingMode_ ) << '\n';
  stream << indent << "  multiplePackedMapsEnabledFlag: " << std::boolalpha << multiplePackedMapsEnabledFlag_ << '\n';
  stream << indent << "  implicitPackingMode: " << static_cast<unsigned>( implicitPackingMode_ ) << '\n';

  for ( size_t i = 0; i < packingInformation_.size(); ++i ) {
    const auto isPresent = packingInformation_[i].isPresent;

    stream << indent << "  packingInformation[ " << i << " ] {\n";
    stream << indent << "    isPresent: " << std::boolalpha << isPresent << '\n';

    if ( isPresent ) {
      stream << indent << "    pin: ";
      packingInformation_[i].pin.dumpTo( stream, indent + "    "s );
    }
    stream << indent << "  }\n";
  }
  stream << indent << "}\n";
}

void PackingInformation::dumpTo( std::ostream &stream, const std::string &indent ) const {
  stream << "PackingInformation {\n";
  stream << indent << "  codecId: " << unsigned{ codecId_ } << '\n';
  stream << indent << "  occupancyPresentFlag: " << std::boolalpha << occupancyPresentFlag_ << '\n';
  stream << indent << "  geometryPresentFlag: " << std::boolalpha << geometryPresentFlag_ << '\n';
  stream << indent << "  attributePresentFlag: " << std::boolalpha << attributePresentFlag_ << '\n';

  for ( size_t i = 0; i < packedMaps_.size(); ++i ) {
    stream << indent << "  packedMaps[ " << i << " ]: ";
    packedMaps_[i].dumpTo( stream, indent + "  "s );
  }
  for ( size_t i = 0; i < regions_.size(); ++i ) {
    stream << indent << "  regions[ " << i << " ]: ";
    regions_[i].dumpTo( stream, indent + "  "s );
  }
  stream << indent << "}\n";
}

void PackingInformation::PackedMap_::dumpTo( std::ostream &stream, const std::string &indent ) const {
  stream << "{\n";
  stream << indent << "  constituentFrameCount: " << ( constituentFrameCountMinus1 + 1U ) << '\n';
  stream << indent << "  packedMapGridColumns: " << ( packedMapGridColumnsMinus1 + 1U ) << '\n';
  stream << indent << "  packedMapGridRows: " << ( packedMapGridRowsMinus1 + 1U ) << '\n';
  stream << indent << "}\n";
}

void PackedRegion::dumpTo( std::ostream &stream, const std::string &indent ) const {
  stream << "PackedRegion {\n";
  stream << indent << "  packedMapIndex: " << unsigned{ packedMapIndex_ };
  stream << ", constituentFrameIndex: " << unsigned{ constituentFrameIndex_ };
  stream << ", tileId: " << unsigned{ tileId_ };
  stream << ", typeId: " << typeIdMinus2_ + 2U << '\n';
  stream << indent << "  topLeftX: " << topLeftX_;
  stream << ", topLeftY: " << topLeftY_;
  stream << ", width: " << ( widthMinus1_ + 1U );
  stream << ", height: " << ( heightMinus1_ + 1U );
  stream << ", unpackTopLeftX: " << unpackTopLeftX_;
  stream << ", unpackTopLeftY: " << unpackTopLeftY_;
  stream << ", rotationFlag: " << std::boolalpha << rotationFlag_ << '\n';
  stream << indent << "  mapIndex: " << unsigned{ mapIndex_ };
  stream << ", auxiliaryDataFlag: " << std::boolalpha << auxiliaryDataFlag_;
  stream << ", attrIndex: " << unsigned{ attrIndex_ };
  stream << ", attrPartitionIndex: " << unsigned{ attrPartitionIndex_ } << '\n';
  stream << indent << "}\n";
}

void VpsPackedVideoExtension::initializeBoundingBoxes( const V3CParameterSet &vps ) {
  if ( getImplicitPackingMode() == ImplicitPackingMode::explicit_mode ) {
    // Deliberately empty
  } else if ( getImplicitPackingMode() == ImplicitPackingMode::implicit_mode ) {
    for ( size_t atlasIdx = 0; atlasIdx <= vps.getAtlasCountMinus1(); ++atlasIdx ) {
      const auto atlasId = vps.getAtlasId( atlasIdx );
      if ( !getPackedVideoPresentFlag( atlasId ) ) { continue; }
      const auto width  = vps.getFrameWidth( atlasId );
      const auto height = vps.getFrameHeight( atlasId );
      auto      &pin    = getPackingInformation( atlasId );

      for ( size_t i = 0; i <= pin.getRegionsCountMinus1(); ++i ) {
        auto &region = pin.getRegion( i );
        region.setWidthMinus1( width - 1 );
        region.setHeightMinus1( height - 1 );
      }
      for ( uint8_t k = 0; k <= pin.getPackedMapCountMinus1(); ++k ) {
        for ( uint8_t j = 0; j <= pin.getConstituentFrameCountMinus1( k ); ++j ) {
          const auto columnsCount = ( pin.getPackedMapGridColumnsMinus1( k ) + size_t{ 1 } );
          const auto rowsCount    = ( pin.getPackedMapGridRowsMinus1( k ) + size_t{ 1 } );
          auto       next         = size_t{};

          for ( size_t i = 0; i <= pin.getRegionsCountMinus1(); ++i ) {
            auto &region = pin.getRegion( i );

            if ( region.getPackedMapIndex() == k && region.getConstituentFrameIndex() == j ) {
              if ( next >= columnsCount * rowsCount ) {
                std::cerr << "initializeBoundingBoxes: too many packed regions to pack implicitly\n";
                exit( -1 );
              }
              region.setTopLeftX( ( next % columnsCount ) * width );
              region.setTopLeftY( ( next / columnsCount ) * height );
              ++next;
            }
          }
        }
      }
    }
  } else {
    std::cerr << "Unknown implicit packing mode " << static_cast<unsigned>( getImplicitPackingMode() ) << "\n";
    exit( -1 );
  }
}
}  // namespace pcc
