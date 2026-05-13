/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2025, ISO/IEC
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
#ifndef PCC_BITSTREAM_PACKEDVIDEOEXTENSION_H
#define PCC_BITSTREAM_PACKEDVIDEOEXTENSION_H

#include "PCCBitstreamCommon.h"
#include <algorithm>
#include <cstdlib>

namespace pcc {
// 8.3.4.7	Packing information syntax, [ j ][ i ]-indexed syntax elements
struct PackedRegion {
 public:
  uint8_t  getPackedMapIndex() const { return packedMapIndex_; }
  uint8_t  getConstituentFrameIndex() const { return constituentFrameIndex_; }
  uint8_t  getTileId() const { return tileId_; }
  uint8_t  getTypeIdMinus2() const { return typeIdMinus2_; }
  uint16_t getTopLeftX() const { return topLeftX_; }
  uint16_t getTopLeftY() const { return topLeftY_; }
  uint16_t getWidthMinus1() const { return widthMinus1_; }
  uint16_t getHeightMinus1() const { return heightMinus1_; }
  uint16_t getUnpackTopLeftX() const { return unpackTopLeftX_; }
  uint16_t getUnpackTopLeftY() const { return unpackTopLeftY_; }
  bool     getRotationFlag() const { return rotationFlag_; }
  uint8_t  getMapIndex() const { return mapIndex_; }
  bool     getAuxiliaryDataFlag() const { return auxiliaryDataFlag_; }
  uint8_t  getAttrIndex() const { return attrIndex_; }
  uint8_t  getAttrPartitionIndex() const { return attrPartitionIndex_; }

  // Convenience methods for unpacking
  uint8_t  getRegionTypeId() const { return typeIdMinus2_ + 2; }  // Returns V3C_OVD, V3C_GVD, V3C_AVD
  uint16_t getPackedRegionLeft() const { return topLeftX_; }
  uint16_t getPackedRegionTop() const { return topLeftY_; }
  uint16_t getPackedRegionWidth() const { return widthMinus1_ + 1; }
  uint16_t getPackedRegionHeight() const { return heightMinus1_ + 1; }

  void setPackedMapIndex( uint8_t value ) { packedMapIndex_ = value; }
  void setConstituentFrameIndex( uint8_t value ) { constituentFrameIndex_ = value; }
  void setTileId( uint8_t value ) { tileId_ = value; }
  void setTypeIdMinus2( uint8_t value ) { typeIdMinus2_ = value; }
  void setTopLeftX( uint16_t value ) { topLeftX_ = value; }
  void setTopLeftY( uint16_t value ) { topLeftY_ = value; }
  void setWidthMinus1( uint16_t value ) { widthMinus1_ = value; }
  void setHeightMinus1( uint16_t value ) { heightMinus1_ = value; }
  void setUnpackTopLeftX( uint16_t value ) { unpackTopLeftX_ = value; }
  void setUnpackTopLeftY( uint16_t value ) { unpackTopLeftY_ = value; }
  void setRotationFlag( bool value ) { rotationFlag_ = value; }
  void setMapIndex( uint8_t value ) { mapIndex_ = value; }
  void setAuxiliaryDataFlag( bool value ) { auxiliaryDataFlag_ = value; }
  void setAttrIndex( uint8_t value ) { attrIndex_ = value; }
  void setAttrPartitionIndex( uint8_t value ) { attrPartitionIndex_ = value; }

  // Dump the full state to the stream in a human-readable way
  void dumpTo( std::ostream &stream, const std::string &indent ) const;

 private:
  uint8_t  packedMapIndex_{};
  uint8_t  constituentFrameIndex_{};
  uint8_t  tileId_{};
  uint8_t  typeIdMinus2_{};
  uint16_t topLeftX_{};
  uint16_t topLeftY_{};
  uint16_t widthMinus1_{};
  uint16_t heightMinus1_{};
  uint16_t unpackTopLeftX_{};
  uint16_t unpackTopLeftY_{};
  bool     rotationFlag_{};
  uint8_t  mapIndex_{};
  bool     auxiliaryDataFlag_{};
  uint8_t  attrIndex_{};
  uint8_t  attrPartitionIndex_{};
};

enum class TemporalPackingMode : uint8_t { consecutive_frames };
enum class ImplicitPackingMode : uint8_t { explicit_mode, implicit_mode };

// 8.3.4.7	Packing information syntax, [ j ]-indexed syntax elements
class PackingInformation {
 public:
  uint8_t getCodecId() const { return codecId_; }
  bool    getOccupancyPresentFlag() const { return occupancyPresentFlag_; }
  bool    getGeometryPresentFlag() const { return geometryPresentFlag_; }
  bool    getAttributePresentFlag() const { return attributePresentFlag_; }
  uint8_t getPackedMapCountMinus1() const {
    return packedMaps_.empty() ? 0 : static_cast<uint8_t>( packedMaps_.size() - 1 );
  }
  uint8_t getConstituentFrameCountMinus1( size_t index ) const {
    checkPackedMapIndex( index );
    return packedMaps_[index].constituentFrameCountMinus1;
  }
  uint8_t getPackedMapGridColumnsMinus1( size_t index ) const {
    checkPackedMapIndex( index );
    return packedMaps_[index].packedMapGridColumnsMinus1;
  }
  uint8_t getPackedMapGridRowsMinus1( size_t index ) const {
    checkPackedMapIndex( index );
    return packedMaps_[index].packedMapGridRowsMinus1;
  }

  size_t        getRegionsCountMinus1() const { return regions_.size() - 1; }
  PackedRegion &getRegion( size_t index ) { return regions_[index]; }
  PackedRegion &addRegion() { return regions_.push_back( {} ), regions_.back(); }

  // Compute the maximum bit depth from occupancy, geometry, and attribute information
  template <typename VpsType>
  uint8_t getBitDepth( VpsType &vps, size_t atlasIndex ) const {
    uint8_t maxBitDepth = 0;

    if ( occupancyPresentFlag_ ) {
      auto &oi    = vps.getOccupancyInformation( atlasIndex );
      maxBitDepth = std::max( maxBitDepth, static_cast<uint8_t>( oi.getOccupancy2DBitdepthMinus1() + 1 ) );
    }

    if ( geometryPresentFlag_ ) {
      auto &gi    = vps.getGeometryInformation( atlasIndex );
      maxBitDepth = std::max( maxBitDepth, static_cast<uint8_t>( gi.getGeometry2dBitdepthMinus1() + 1 ) );
    }

    if ( attributePresentFlag_ ) {
      auto &ai = vps.getAttributeInformation( atlasIndex );
      for ( size_t i = 0; i < ai.getAttributeCount(); i++ ) {
        maxBitDepth = std::max( maxBitDepth, static_cast<uint8_t>( ai.getAttribute2dBitdepthMinus1( i ) + 1 ) );
      }
    }

    return maxBitDepth;
  }

  void setCodecId( uint8_t value ) { codecId_ = value; }
  void setOccupancyPresentFlag( bool value ) { occupancyPresentFlag_ = value; }
  void setGeometryPresentFlag( bool value ) { geometryPresentFlag_ = value; }
  void setAttributePresentFlag( bool value ) { attributePresentFlag_ = value; }
  void setPackedMapCountMinus1( uint8_t value ) { packedMaps_.resize( value + size_t{ 1 } ); }
  void setConstituentFrameCountMinus1( size_t index, uint8_t value ) {
    checkPackedMapIndex( index );
    packedMaps_[index].constituentFrameCountMinus1 = value;
  }
  void setPackedMapGridColumnsMinus1( size_t index, uint8_t value ) {
    checkPackedMapIndex( index );
    packedMaps_[index].packedMapGridColumnsMinus1 = value;
  }
  void setPackedMapGridRowsMinus1( size_t index, uint8_t value ) {
    checkPackedMapIndex( index );
    packedMaps_[index].packedMapGridRowsMinus1 = value;
  }

  // Dump the full state to the stream in a human-readable way
  void dumpTo( std::ostream &stream, const std::string &indent ) const;

 private:
  void checkPackedMapIndex( size_t index ) const {
    if ( index >= packedMaps_.size() ) {
      std::cerr << "Packed map index is out of bounds\n";
      std::abort();
    }
  }

  uint8_t codecId_{};
  bool    occupancyPresentFlag_{};
  bool    geometryPresentFlag_{};
  bool    attributePresentFlag_{};

  struct PackedMap_ {
    uint8_t constituentFrameCountMinus1{};
    uint8_t packedMapGridColumnsMinus1{};
    uint8_t packedMapGridRowsMinus1{};

    void dumpTo( std::ostream &stream, const std::string &indent ) const;
  };
  std::vector<PackedMap_>   packedMaps_{ {} };
  std::vector<PackedRegion> regions_;
};

class V3CParameterSet;

// 8.3.4.9	Packed video extension syntax
class VpsPackedVideoExtension {
 public:
  bool isPresent() { return !packingInformation_.empty(); }

  uint8_t getVersion() const { return version_; }

  bool                getTemporalPackingEnabledFlag() const { return temporalPackingEnabledFlag_; }
  TemporalPackingMode getTemporalPackingMode() const { return temporalPackingMode_; }
  bool                getMultiplePackedMapsEnabledFlag() const { return multiplePackedMapsEnabledFlag_; }
  ImplicitPackingMode getImplicitPackingMode() const { return implicitPackingMode_; }

  bool getPackedVideoPresentFlag( size_t index ) const {
    return index < packingInformation_.size() && packingInformation_[index].isPresent;
  }

  PackingInformation &getPackingInformation( size_t index ) {
    while ( packingInformation_.size() <= index ) { packingInformation_.push_back( {} ); }
    packingInformation_[index].isPresent = true;
    return packingInformation_[index].pin;
  }

  void setTemporalPackingEnabledFlag( bool value ) { temporalPackingEnabledFlag_ = value; }
  void setTemporalPackingMode( TemporalPackingMode value ) { temporalPackingMode_ = value; }
  void setMultiplePackedMapsEnabledFlag( bool value ) { multiplePackedMapsEnabledFlag_ = value; }
  void setImplicitPackingMode( ImplicitPackingMode value ) { implicitPackingMode_ = value; }

  void setPackedVideoPresentFlag( size_t index, bool value ) {
    while ( packingInformation_.size() <= index ) { packingInformation_.push_back( {} ); }
    packingInformation_[index].isPresent = value;
  }

  void setVersion( uint8_t value ) { version_ = value; }

  // Dump the full state to the stream in a human-readable way
  void dumpTo( std::ostream &stream, const std::string &indent ) const;

  // Annex B.4.5 Packed region bounding box initialization process
  void initializeBoundingBoxes( const V3CParameterSet &vps );

 private:
  struct PackingInformation_ {
    bool               isPresent{ true };
    PackingInformation pin;
  };

  uint8_t                          version_{ 2 };
  bool                             temporalPackingEnabledFlag_{};
  TemporalPackingMode              temporalPackingMode_{};
  bool                             multiplePackedMapsEnabledFlag_{};
  ImplicitPackingMode              implicitPackingMode_{};
  std::vector<PackingInformation_> packingInformation_;
};
};  // namespace pcc

#endif  //~PCC_BITSTREAM_PACKEDVIDEOEXTENSION_H
