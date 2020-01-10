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
#ifndef PCC_BITSTREAM_SYNTAX_H
#define PCC_BITSTREAM_SYNTAX_H

#include "PCCBitstreamCommon.h"
#include "PCCVideoBitstream.h"
#include "PCCBitstream.h"

#include "PCCSei.h"
#include "PCCVpccUnit.h"
#include "PCCSampleStreamNalUnit.h"
#include "PCCSampleStreamVpccUnit.h"
#include "PCCAtlasTileGroupHeader.h"
#include "PCCVpccParameterSet.h"
#include "PCCVpccUnitPayloadHeader.h"
#include "PCCAtlasTileGroupLayerRbsp.h"
#include "PCCAtlasSequenceParameterSetRbsp.h"
#include "PCCAtlasFrameParameterSetRbsp.h"
#include "PCCAtlasTileGroupLayerRbsp.h"

namespace pcc {

class PCCBitstreamStat;

// used for syntax handling
class PCCHighLevelSyntax {
 public:
  PCCHighLevelSyntax();
  ~PCCHighLevelSyntax();
  size_t   getGofSize() { return gofSize_; }
  uint8_t  getOccupancyPrecision() { return occupancyPrecision_; }
  uint8_t  getOccupancyPackingBlockSize() { return occupancyPackingBlockSize_; }
  uint8_t  getLog2PatchQuantizerSizeX() { return log2PatchQuantizerSizeX_; }
  uint8_t  getLog2PatchQuantizerSizeY() { return log2PatchQuantizerSizeY_; }
  bool     getEnablePatchSizeQuantization() { return enablePatchSizeQuantization_; }
  float    getModelScale() { return modelScale_; }
  size_t   getMPGeoWidth() { return MPGeoWidth_; }
  size_t   getMPGeoHeight() { return MPGeoHeight_; }
  size_t   getMPAttWidth() { return MPAttWidth_; }
  size_t   getMPAttHeight() { return MPAttHeight_; }
  bool&    getPrefilterLossyOM() { return prefilterLossyOM_; }
  size_t&  getOffsetLossyOM() { return offsetLossyOM_; }
  size_t   getGeometry3dCoordinatesBitdepth() { return geometry3dCoordinatesBitdepth_; }
  void     setOccupancyPrecision( uint8_t value ) { occupancyPrecision_ = value; }
  void     setOccupancyPackingBlockSize( uint8_t value ) { occupancyPackingBlockSize_ = value; }
  void     setGofSize( size_t gofSize ) { gofSize_ = gofSize; }
  void setLog2PatchQuantizerSizeX( uint8_t value ) { log2PatchQuantizerSizeX_ = value; }
  void setLog2PatchQuantizerSizeY( uint8_t value ) { log2PatchQuantizerSizeY_ = value; }
  void setEnablePatchSizeQuantization( bool value ) { enablePatchSizeQuantization_ = value; }
  void setModelScale( float value ) { modelScale_ = value; }
  void setMPGeoWidth( size_t value ) { MPGeoWidth_ = value; }
  void setMPGeoHeight( size_t value ) { MPGeoHeight_ = value; }
  void setMPAttWidth( size_t value ) { MPAttWidth_ = value; }
  void setMPAttHeight( size_t value ) { MPAttHeight_ = value; }
  void setPrefilterLossyOM( bool value ) { prefilterLossyOM_ = value; }
  void setOffsetLossyOM( size_t value ) { offsetLossyOM_ = value; }
  void setGeometry3dCoordinatesBitdepth( size_t value ) { geometry3dCoordinatesBitdepth_ = value; }
  void setMaxNumRefAtlasFrame( size_t value ) { maxNumRefAtlasFrame_ = value; }
  void setNumOfRefAtlasFrameList( size_t value ) { refAtlasFrameList_.resize( value ); }
  void setSizeOfRefAtlasFrameList( size_t listIndex, size_t listSize ) {
    refAtlasFrameList_[listIndex].resize( listSize );
  }
  void setRefAtlasFrame( size_t listIndex, size_t refIndex, int32_t value ) {
    refAtlasFrameList_[listIndex][refIndex] = value;
  }
  void setRefAtlasFrameList( std::vector<std::vector<int32_t>>& list ) {
    size_t listSize = ( std::min )( refAtlasFrameList_.size(), list.size() );
    for ( size_t i = 0; i < listSize; i++ ) {
      size_t refSize = ( std::min )( refAtlasFrameList_[i].size(), list[i].size() );
      for ( size_t j = 0; j < refSize; j++ ) refAtlasFrameList_[i][j] = list[i][j];
    }
  }
  size_t  getMaxNumRefAtlasFrame() { return maxNumRefAtlasFrame_; }
  size_t  getNumOfRefAtlasFrameList() { return refAtlasFrameList_.size(); }
  size_t  getSizeOfRefAtlasFrameList( size_t listIndex ) { return refAtlasFrameList_[listIndex].size(); }
  int32_t getRefAtlasFrame( size_t listIndex, size_t refIndex ) { return refAtlasFrameList_[listIndex][refIndex]; }
  std::vector<int32_t>& getRefAtlasFrameList( size_t listIndex ) { return refAtlasFrameList_[listIndex]; }
  void                  constructRefList( size_t aspsIdx, size_t afpsIdx );
  size_t                getNumRefIdxActive( AtlasTileGroupHeader& atgh );

  PCCVideoBitstream& createVideoBitstream( PCCVideoType type ) {
    videoBitstream_.push_back( PCCVideoBitstream( type ) );
    return videoBitstream_.back();
  }
  bool& getSingleLayerMode() { return singleLayerMode_; }
  size_t             getVideoBitstreamCount() { return videoBitstream_.size(); }
  PCCVideoBitstream& getVideoBitstream( size_t index ) { return videoBitstream_[index]; }
  PCCVideoBitstream& getVideoBitstream( PCCVideoType type ) {
    for ( auto& value : videoBitstream_ ) {
      if ( value.type() == type ) { return value; }
    }
    printf( "ERROR: can't get video bitstream of type %s \n", toString( type ).c_str() );
    fflush( stdout );
    assert( 0 );
    exit( -1 );
  }

  void                   allocOneLayerData();
  void                   printVideoBitstream();
  void                   printBlockToPatch( const size_t occupancyResolution );
  VpccUnitPayloadHeader& getVpccUnitHeaderAVD() { return vpccUnitHeader_[size_t( VPCC_AVD ) - 1]; }  // 3
  VpccUnitPayloadHeader& getVpccUnitHeaderGVD() { return vpccUnitHeader_[size_t( VPCC_GVD ) - 1]; }  // 2
  VpccUnitPayloadHeader& getVpccUnitHeaderOVD() { return vpccUnitHeader_[size_t( VPCC_OVD ) - 1]; }  // 1
  VpccUnitPayloadHeader& getVpccUnitHeaderAD() { return vpccUnitHeader_[size_t( VPCC_AD ) - 1]; }    // 0
  VpccUnitPayloadHeader& getVpccUnitHeader( int index ) { return vpccUnitHeader_[index]; }

  VpccParameterSet&              getSps() { return vpccParameterSets_[0]; }
  VpccParameterSet&              getSps( size_t index ) { return vpccParameterSets_[index]; }
  std::vector<VpccParameterSet>& getSpsList() { return vpccParameterSets_; }

  VpccParameterSet& addVpccParameterSet( uint8_t index ) {
    VpccParameterSet sps;
    sps.setVpccParameterSetId( index );
    vpccParameterSets_.push_back( sps );
    return vpccParameterSets_.back();
  }

  VpccParameterSet& addVpccParameterSet() {
    uint8_t          index = vpccParameterSets_.size();
    VpccParameterSet sps;
    sps.setVpccParameterSetId( index );
    vpccParameterSets_.push_back( sps );
    return vpccParameterSets_.back();
  }
  AtlasSequenceParameterSetRbsp& getAtlasSequenceParameterSet( size_t setId ) {
    return atlasSequenceParameterSet_[setId];
  }
  std::vector<AtlasSequenceParameterSetRbsp>& getAtlasSequenceParameterSetList() { return atlasSequenceParameterSet_; }

  AtlasFrameParameterSetRbsp& getAtlasFrameParameterSet( size_t setId ) { return atlasFrameParameterSet_[setId]; }
  std::vector<AtlasFrameParameterSetRbsp>& getAtlasFrameParameterSetList() { return atlasFrameParameterSet_; }

  AtlasSequenceParameterSetRbsp& addAtlasSequenceParameterSet() {
    AtlasSequenceParameterSetRbsp asps;
    asps.setAltasSequenceParameterSetId( atlasSequenceParameterSet_.size() );
    atlasSequenceParameterSet_.push_back( asps );
    return atlasSequenceParameterSet_.back();
  }
  AtlasSequenceParameterSetRbsp& addAtlasSequenceParameterSet( uint8_t setId ) {
    AtlasSequenceParameterSetRbsp asps;
    asps.setAltasSequenceParameterSetId( setId );
    if ( atlasSequenceParameterSet_.size() < setId + 1 ) { atlasSequenceParameterSet_.resize( setId + 1 ); }
    atlasSequenceParameterSet_[setId] = asps;
    return atlasSequenceParameterSet_[setId];
  }
  AtlasFrameParameterSetRbsp& addAtlasFrameParameterSet() {
    AtlasFrameParameterSetRbsp afps;
    afps.setAtlasFrameParameterSetId( atlasFrameParameterSet_.size() );
    atlasFrameParameterSet_.push_back( afps );
    return atlasFrameParameterSet_.back();
  }
  size_t addAtlasFrameParameterSet( AtlasFrameParameterSetRbsp& refAfps ) {
    size_t                     setId = atlasFrameParameterSet_.size();
    AtlasFrameParameterSetRbsp afps;
    afps.copyFrom( refAfps );
    afps.setAtlasFrameParameterSetId( setId );
    atlasFrameParameterSet_.resize( setId + 1 );
    atlasFrameParameterSet_[setId] = afps;
    return setId;
  }
  AtlasFrameParameterSetRbsp& addAtlasFrameParameterSet( uint8_t setId ) {
    AtlasFrameParameterSetRbsp afps;
    afps.setAtlasFrameParameterSetId( setId );
    if ( atlasFrameParameterSet_.size() < setId + 1 ) { atlasFrameParameterSet_.resize( setId + 1 ); }
    atlasFrameParameterSet_[setId] = afps;
    return atlasFrameParameterSet_[setId];
  }

  AtlasTileGroupLayerRbsp& addAtlasTileGroupLayer() {
    size_t                  frameIdx = atlasTileGroupLayer_.size();
    AtlasTileGroupLayerRbsp atgl;
    atgl.setFrameIndex( frameIdx );
    atgl.getAtlasTileGroupDataUnit().setFrameIndex( frameIdx );
    atlasTileGroupLayer_.push_back( atgl );
    return atlasTileGroupLayer_.back();
  }

  AtlasTileGroupLayerRbsp& addAtlasTileGroupLayer( size_t frameIdx ) {
    AtlasTileGroupLayerRbsp atgl;
    atlasTileGroupLayer_.resize( frameIdx + 1 );
    atlasTileGroupLayer_[frameIdx] = atgl;
    return atlasTileGroupLayer_[frameIdx];
  }

  std::vector<AtlasTileGroupLayerRbsp>& getAtlasTileGroupLayerList() { return atlasTileGroupLayer_; }
  AtlasTileGroupLayerRbsp&              getAtlasTileGroupLayer( size_t index ) { return atlasTileGroupLayer_[index]; }

  void addPointLocalReconstructionMode( const PointLocalReconstructionMode& mode ) {
    pointLocalReconstructionMode_.push_back( mode );
  }
  PointLocalReconstructionMode& getPointLocalReconstructionMode( size_t index ) {
    return pointLocalReconstructionMode_[index];
  }
  size_t getPointLocalReconstructionModeNumber() { return pointLocalReconstructionMode_.size(); }

  void              setBitstreamStat( PCCBitstreamStat& bitstreamStat ) { bitstreamStat_ = &bitstreamStat; }
  PCCBitstreamStat& getBitstreamStat() { return *bitstreamStat_; }

  SEI& addSei( NalUnitType nalUnitType, SeiPayloadType payloadType ) {
    std::shared_ptr<SEI> sharedPtr;
    switch ( payloadType ) {
      case BUFFERING_PERIOD: sharedPtr = std::make_shared<SEIBufferingPeriod>(); break;
      case ATLAS_FRAME_TIMING: sharedPtr = std::make_shared<SEIAtlasFrameTiming>(); break;
      case FILLER_PAYLOAD: break;
      case USER_DATAREGISTERED_ITUTT35: sharedPtr = std::make_shared<SEIUserDataRegisteredItuTT35>(); break;
      case USER_DATA_UNREGISTERED: sharedPtr = std::make_shared<SEIUserDataUnregistered>(); break;
      case RECOVERY_POINT: sharedPtr = std::make_shared<SEIRecoveryPoint>(); break;
      case NO_DISPLAY: sharedPtr = std::make_shared<SEINoDisplay>(); break;
      case TIME_CODE: break;
      case REGIONAL_NESTING: break;
      case SEI_MANIFEST: sharedPtr = std::make_shared<SEIManifest>(); break;
      case SEI_PREFIX_INDICATION: sharedPtr = std::make_shared<SEIPrefixIndication>(); break;
      case GEOMETRY_TRANSFORMATION_PARAMS: sharedPtr = std::make_shared<SEIGeometryTransformationParams>(); break;
      case ATTRIBUTE_TRANSFORMATION_PARAMS: sharedPtr = std::make_shared<SEIAttributeTransformationParams>(); break;
      case ACTIVE_SUBSTREAMS: sharedPtr = std::make_shared<SEIActiveSubstreams>(); break;
      case COMPONENT_CODEC_MAPPING: sharedPtr = std::make_shared<SEIComponentCodecMapping>(); break;
      case VOLUMETRIC_TILING_INFO: sharedPtr = std::make_shared<SEIVolumetricTilingInfo>(); break;
      case PRESENTATION_INFORMATION: sharedPtr = std::make_shared<SEIPresentationInformation>(); break;
      case SMOOTHING_PARAMETERS: sharedPtr = std::make_shared<SEISmoothingParameters>(); break;
      case RESERVED_SEI_MESSAGE: sharedPtr = std::make_shared<SEIReservedSeiMessage>(); break;
      default:
        fprintf( stderr, "SEI payload type not supported \n" );
        exit( -1 );
        break;
    }
    if ( nalUnitType == NAL_PREFIX_SEI ) {
      seiPrefix_.push_back( sharedPtr );
      return *( seiPrefix_.back().get() );
    } else if ( nalUnitType == NAL_SUFFIX_SEI ) {
      seiSuffix_.push_back( sharedPtr );
      return *( seiSuffix_.back().get() );
    } else {
      fprintf( stderr, "Nal unit type of SEI not correct\n" );
      exit( -1 );
    }
    return *( sharedPtr );
  }
  bool seiIsPresent( NalUnitType nalUnitType, SeiPayloadType payloadType ) {
    if ( nalUnitType != NAL_PREFIX_SEI && nalUnitType != NAL_SUFFIX_SEI ) { return false; }
    for ( auto& sei : nalUnitType == NAL_PREFIX_SEI ? seiPrefix_ : seiSuffix_ ) {
      if ( sei->getPayloadType() == payloadType ) { return true; }
    }
    return false;
  }

  SEI& getSei( NalUnitType nalUnitType, SeiPayloadType payloadType ) {
    for ( auto& sei : nalUnitType == NAL_PREFIX_SEI ? seiPrefix_ : seiSuffix_ ) {
      if ( sei->getPayloadType() == payloadType ) { return *sei; }
    }
  }
  SEI& addSeiPrefix( SeiPayloadType payloadType ) { return addSei( NAL_PREFIX_SEI, payloadType ); }
  SEI& addSeiSuffix( SeiPayloadType payloadType ) { return addSei( NAL_SUFFIX_SEI, payloadType ); }
  std::vector<std::shared_ptr<SEI>>& getSeiPrefix() { return seiPrefix_; }
  std::vector<std::shared_ptr<SEI>>& getSeiSuffix() { return seiSuffix_; }
  SEI&                               getSeiPrefix( size_t index ) { return *( seiPrefix_[index] ); }
  SEI&                               getSeiSuffix( size_t index ) { return *( seiSuffix_[index] ); }

 private:
  size_t                                     gofSize_;
  std::vector<PCCVideoBitstream>             videoBitstream_;
  VpccUnitPayloadHeader                      vpccUnitHeader_[5];
  std::vector<AtlasSequenceParameterSetRbsp> atlasSequenceParameterSet_;
  std::vector<AtlasFrameParameterSetRbsp>    atlasFrameParameterSet_;
  std::vector<AtlasTileGroupLayerRbsp>       atlasTileGroupLayer_;
  std::vector<std::shared_ptr<SEI>>          seiPrefix_;
  std::vector<std::shared_ptr<SEI>>          seiSuffix_;
  std::vector<VpccParameterSet>              vpccParameterSets_;
  uint8_t                                    occupancyPrecision_;
  uint8_t                                    occupancyPackingBlockSize_;
  uint8_t                                    log2PatchQuantizerSizeX_;
  uint8_t                                    log2PatchQuantizerSizeY_;
  bool                                       enablePatchSizeQuantization_;
  size_t                                     MPGeoWidth_;
  size_t                                     MPGeoHeight_;
  size_t                                     MPAttWidth_;
  size_t                                     MPAttHeight_;
  float                                      modelScale_;
  std::vector<PointLocalReconstructionMode>  pointLocalReconstructionMode_;
  bool                                       prefilterLossyOM_;
  size_t                                     offsetLossyOM_;
  size_t                                     geometry3dCoordinatesBitdepth_;
  bool                                       singleLayerMode_;
  size_t                                     maxNumRefAtlasFrame_;
  std::vector<std::vector<int32_t>>          refAtlasFrameList_;
  PCCBitstreamStat*                          bitstreamStat_;
};
};  // namespace pcc

#endif /* PCC_BITSTREAM_SYNTAX_H */
