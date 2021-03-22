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
#ifndef PCC_BITSTREAM_HIGHLEVELSYNTAX_H
#define PCC_BITSTREAM_HIGHLEVELSYNTAX_H

#include "PCCBitstreamCommon.h"
#include "PCCVideoBitstream.h"
#include "PCCBitstream.h"

#include "PCCSei.h"
#include "PCCV3CUnit.h"
#include "PCCSampleStreamNalUnit.h"
#include "PCCSampleStreamV3CUnit.h"
#include "PCCAtlasTileHeader.h"
#include "PCCV3CParameterSet.h"
#include "PCCV3CUnitHeader.h"
#include "PCCAtlasTileLayerRbsp.h"
#include "PCCAtlasSequenceParameterSetRbsp.h"
#include "PCCAtlasFrameParameterSetRbsp.h"
#include "PCCAtlasTileLayerRbsp.h"

namespace pcc {

class PCCBitstreamStat;

// used for syntax handling
class PCCAtlasHighLevelSyntax {
 public:
  PCCAtlasHighLevelSyntax();
  ~PCCAtlasHighLevelSyntax();

  // video data related functions
  PCCVideoBitstream& createVideoBitstream( PCCVideoType type ) {
    videoBitstream_.push_back( PCCVideoBitstream( type ) );
    return videoBitstream_.back();
  }
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
  void printVideoBitstream();

  // ASPS related functions
  AtlasSequenceParameterSetRbsp& getAtlasSequenceParameterSet( size_t setId ) {
    return atlasSequenceParameterSet_[setId];
  }
  AtlasSequenceParameterSetRbsp& addAtlasSequenceParameterSet() {
    AtlasSequenceParameterSetRbsp asps;
    asps.setAtlasSequenceParameterSetId( atlasSequenceParameterSet_.size() );
    atlasSequenceParameterSet_.push_back( asps );
    return atlasSequenceParameterSet_.back();
  }
  AtlasSequenceParameterSetRbsp& addAtlasSequenceParameterSet( uint8_t setId ) {
    AtlasSequenceParameterSetRbsp asps;
    asps.setAtlasSequenceParameterSetId( setId );
    if ( atlasSequenceParameterSet_.size() < setId + 1 ) { atlasSequenceParameterSet_.resize( setId + 1 ); }
    atlasSequenceParameterSet_[setId] = asps;
    return atlasSequenceParameterSet_[setId];
  }
  std::vector<AtlasSequenceParameterSetRbsp>& getAtlasSequenceParameterSetList() { return atlasSequenceParameterSet_; }

  // reference list, defined in ASPS
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
  size_t  getNumOfRefAtlasFrameList() { return refAtlasFrameList_.size(); }
  size_t  getSizeOfRefAtlasFrameList( size_t listIndex ) { return refAtlasFrameList_[listIndex].size(); }
  int32_t getRefAtlasFrame( size_t listIndex, size_t refIndex ) { return refAtlasFrameList_[listIndex][refIndex]; }
  std::vector<int32_t>& getRefAtlasFrameList( size_t listIndex ) { return refAtlasFrameList_[listIndex]; }
  size_t                getNumRefIdxActive( AtlasTileHeader& ath );
  size_t                getMaxNumRefAtlasFrame( size_t listIndex ) { return maxNumRefAtlasFrame_; }
  void                  setMaxNumRefAtlasFrame( size_t value ) { maxNumRefAtlasFrame_ = value; }

  // point local recosntruction, defined in ASPS
  void addPointLocalReconstructionMode( const PointLocalReconstructionMode& mode ) {
    pointLocalReconstructionMode_.push_back( mode );
  }
  PointLocalReconstructionMode& getPointLocalReconstructionMode( size_t index ) {
    return pointLocalReconstructionMode_[index];
  }
  size_t getPointLocalReconstructionModeNumber() { return pointLocalReconstructionMode_.size(); }

  // AFPS related functions
  AtlasFrameParameterSetRbsp& getAtlasFrameParameterSet( size_t setId ) { return atlasFrameParameterSet_[setId]; }
  std::vector<AtlasFrameParameterSetRbsp>& getAtlasFrameParameterSetList() {
#if 0
    printf("\t-------(inPCCAtlasHighLevelSyntax)------------\n");
    printf("\t-- size of videoBitstream_               : %zu\n", videoBitstream_.size());
    printf("\t-- size of atlasSequenceParameterSet_    : %zu\n", atlasSequenceParameterSet_.size());
    printf("\t-- size of refAtlasFrameList_            : %zu\n", refAtlasFrameList_.size());
    printf("\t-- maxNumRefAtlasFrame_                  : %zu\n", maxNumRefAtlasFrame_);
    printf("\t-- size of pointLocalReconstructionMode_ : %zu\n", pointLocalReconstructionMode_.size());
    printf("\t-- size of atlasFrameParameterSet_       : %zu\n", atlasFrameParameterSet_.size());
    printf("\t-- size of atlasTileLayer_               : %zu\n", atlasTileLayer_.size());
    printf("\t-- size of seiPrefix_                    : %zu\n", seiPrefix_.size());
    printf("\t-- size of seiSuffix_                    : %zu\n", seiSuffix_.size());
    printf("\t----------------------------------------------\n");

#endif
    return atlasFrameParameterSet_;
  }
  AtlasFrameParameterSetRbsp& addAtlasFrameParameterSet() {
    AtlasFrameParameterSetRbsp afps;
    afps.setAtlasFrameParameterSetId( atlasFrameParameterSet_.size() );
    atlasFrameParameterSet_.push_back( afps );
    return atlasFrameParameterSet_.back();
  }
  AtlasFrameParameterSetRbsp& addAtlasFrameParameterSet( AtlasFrameParameterSetRbsp& refAfps ) {
    size_t                     setId = atlasFrameParameterSet_.size();
    AtlasFrameParameterSetRbsp afps;
    afps.copyFrom( refAfps );
    afps.setAtlasFrameParameterSetId( setId );
    atlasFrameParameterSet_.push_back( afps );
    return atlasFrameParameterSet_.back();
  }
  AtlasFrameParameterSetRbsp& addAtlasFrameParameterSet( uint8_t setId ) {
    AtlasFrameParameterSetRbsp afps;
    afps.setAtlasFrameParameterSetId( setId );
    if ( atlasFrameParameterSet_.size() < setId + 1 ) { atlasFrameParameterSet_.resize( setId + 1 ); }
    atlasFrameParameterSet_[setId] = afps;
    return atlasFrameParameterSet_[setId];
  }

  // ATGL related functions
  AtlasTileLayerRbsp& addAtlasTileLayer() {
    AtlasTileLayerRbsp atgl;
    atgl.setTileOrder( atlasTileLayer_.size() );
    atgl.getDataUnit().setTileOrder( atlasTileLayer_.size() );
    atlasTileLayer_.push_back( atgl );
    return atlasTileLayer_.back();
  }
  AtlasTileLayerRbsp& addAtlasTileLayer(
      size_t frameIdx,
      size_t tileIdx ) {  // ajt::how is tileIdx is used, should it also do setTileOrder(tileIdx)?
    AtlasTileLayerRbsp atgl;
    atgl.setAtlasFrmOrderCntVal( frameIdx );
    atlasTileLayer_.push_back( atgl );
    return atlasTileLayer_.back();
  }
  std::vector<AtlasTileLayerRbsp>& getAtlasTileLayerList() { return atlasTileLayer_; }
  AtlasTileLayerRbsp&              getAtlasTileLayer( size_t atglOrder ) { return atlasTileLayer_[atglOrder]; }

  // SEI related functions
  SEI& addSei( NalUnitType nalUnitType, SeiPayloadType payloadType ) {
    std::shared_ptr<SEI> sharedPtr;
    switch ( payloadType ) {
      case BUFFERING_PERIOD: sharedPtr = std::make_shared<SEIBufferingPeriod>(); break;
      case ATLAS_FRAME_TIMING: sharedPtr = std::make_shared<SEIAtlasFrameTiming>(); break;
      case FILLER_PAYLOAD: break;
      case USER_DATAREGISTERED_ITUTT35: sharedPtr = std::make_shared<SEIUserDataRegisteredItuTT35>(); break;
      case USER_DATA_UNREGISTERED: sharedPtr = std::make_shared<SEIUserDataUnregistered>(); break;
      case RECOVERY_POINT: sharedPtr = std::make_shared<SEIRecoveryPoint>(); break;
      case NO_RECONSTRUCTION: sharedPtr = std::make_shared<SEINoDisplay>(); break;
      case TIME_CODE: sharedPtr = std::make_shared<SEITimeCode>(); break;
      case SEI_MANIFEST: sharedPtr = std::make_shared<SEIManifest>(); break;
      case SEI_PREFIX_INDICATION: sharedPtr = std::make_shared<SEIPrefixIndication>(); break;
      case ACTIVE_SUB_BITSTREAMS: sharedPtr = std::make_shared<SEIActiveSubBitstreams>(); break;
      case COMPONENT_CODEC_MAPPING: sharedPtr = std::make_shared<SEIComponentCodecMapping>(); break;
      case SCENE_OBJECT_INFORMATION: sharedPtr = std::make_shared<SEISceneObjectInformation>(); break;
      case OBJECT_LABEL_INFORMATION: sharedPtr = std::make_shared<SEIObjectLabelInformation>(); break;
      case PATCH_INFORMATION: sharedPtr = std::make_shared<SEIPatchInformation>(); break;
      case VOLUMETRIC_RECTANGLE_INFORMATION: sharedPtr = std::make_shared<SEIVolumetricRectangleInformation>(); break;
      case ATLAS_OBJECT_INFORMATION: sharedPtr = std::make_shared<SEIAtlasInformation>(); break;
      case VIEWPORT_CAMERA_PARAMETERS: sharedPtr = std::make_shared<SEIViewportCameraParameters>(); break;
      case VIEWPORT_POSITION: sharedPtr = std::make_shared<SEIViewportPosition>(); break;
      case DECODED_ATLAS_INFORMATION_HASH: sharedPtr = std::make_shared<SEIDecodedAtlasInformationHash>(); break;
      case ATTRIBUTE_TRANSFORMATION_PARAMS: sharedPtr = std::make_shared<SEIAttributeTransformationParams>(); break;
      case OCCUPANCY_SYNTHESIS: sharedPtr = std::make_shared<SEIOccupancySynthesis>(); break;
      case GEOMETRY_SMOOTHING: sharedPtr = std::make_shared<SEIGeometrySmoothing>(); break;
      case ATTRIBUTE_SMOOTHING: sharedPtr = std::make_shared<SEIAttributeSmoothing>(); break;
      case RESERVED_SEI_MESSAGE: sharedPtr = std::make_shared<SEIReservedSeiMessage>(); break;
      default:
        fprintf( stderr, "SEI payload type not supported \n" );
        exit( -1 );
        break;
    }
    if ( nalUnitType == NAL_PREFIX_ESEI || nalUnitType == NAL_PREFIX_NSEI ) {
      seiPrefix_.push_back( sharedPtr );
      return *( seiPrefix_.back().get() );
    } else if ( nalUnitType == NAL_SUFFIX_ESEI || nalUnitType == NAL_SUFFIX_NSEI ) {
      seiSuffix_.push_back( sharedPtr );
      return *( seiSuffix_.back().get() );
    } else {
      fprintf( stderr, "Nal unit type of SEI not correct\n" );
      exit( -1 );
    }
    return *( sharedPtr );
  }
  bool seiIsPresent( NalUnitType nalUnitType, SeiPayloadType payloadType ) {
    if ( nalUnitType != NAL_PREFIX_ESEI && nalUnitType != NAL_SUFFIX_ESEI && nalUnitType != NAL_PREFIX_NSEI &&
         nalUnitType != NAL_SUFFIX_NSEI ) {
      return false;
    }
    for ( auto& sei : nalUnitType == NAL_PREFIX_ESEI || nalUnitType == NAL_PREFIX_NSEI ? seiPrefix_ : seiSuffix_ ) {
      if ( sei->getPayloadType() == payloadType ) { return true; }
    }
    return false;
  }
  SEI* getSei( NalUnitType nalUnitType, SeiPayloadType payloadType ) {
    auto& seis = ( nalUnitType == NAL_PREFIX_ESEI || nalUnitType == NAL_PREFIX_NSEI ) ? seiPrefix_ : seiSuffix_;
    for ( auto& sei : seis ) {
      if ( sei->getPayloadType() == payloadType ) { return sei.get(); }
    }
    assert( 0 );
    return (SEI*)nullptr;
  }

  SEI* getLastSei( NalUnitType nalUnitType, SeiPayloadType payloadType ) {
    auto& seis = ( nalUnitType == NAL_PREFIX_ESEI || nalUnitType == NAL_PREFIX_NSEI ) ? seiPrefix_ : seiSuffix_;
    for ( auto sei = seis.rbegin(); sei != seis.rend(); ++sei ) {
      if ( sei->get()->getPayloadType() == payloadType ) { return sei->get(); }
    }
    assert( 0 );
    return (SEI*)nullptr;
  }
  SEI& addSeiPrefix( SeiPayloadType payloadType, bool essensial ) {
    return addSei( essensial ? NAL_PREFIX_ESEI : NAL_PREFIX_NSEI, payloadType );
  }
  SEI& addSeiSuffix( SeiPayloadType payloadType, bool essensial ) {
    return addSei( essensial ? NAL_SUFFIX_ESEI : NAL_SUFFIX_NSEI, payloadType );
  }
  void addSeiToSeiSuffix( SeiPayloadType payloadType, bool essensial, SEIDecodedAtlasInformationHash& seiContext ) {
    seiSuffix_.push_back( std::make_shared<SEIDecodedAtlasInformationHash>( seiContext ) );
  }

  std::vector<std::shared_ptr<SEI>>& getSeiPrefix() { return seiPrefix_; }
  std::vector<std::shared_ptr<SEI>>& getSeiSuffix() { return seiSuffix_; }
  SEI&                               getSeiPrefix( size_t index ) { return *( seiPrefix_[index] ); }
  SEI&                               getSeiSuffix( size_t index ) { return *( seiSuffix_[index] ); }

 private:
  std::vector<PCCVideoBitstream>             videoBitstream_;             // video related variables
  std::vector<AtlasSequenceParameterSetRbsp> atlasSequenceParameterSet_;  // ASPS related variables
  std::vector<std::vector<int32_t>>          refAtlasFrameList_;
  size_t                                     maxNumRefAtlasFrame_;
  std::vector<PointLocalReconstructionMode>  pointLocalReconstructionMode_;
  std::vector<AtlasFrameParameterSetRbsp>    atlasFrameParameterSet_;  // AFPS related variables
  std::vector<AtlasTileLayerRbsp>            atlasTileLayer_;          // ATGL related variables
  std::vector<std::shared_ptr<SEI>>          seiPrefix_;               // SEI related variables
  std::vector<std::shared_ptr<SEI>>          seiSuffix_;
};

class PCCHighLevelSyntax {
 public:
  PCCHighLevelSyntax();
  ~PCCHighLevelSyntax();

  // bitstream statistic related functions
  void              setBitstreamStat( PCCBitstreamStat& bitstreamStat ) { bitstreamStat_ = &bitstreamStat; }
  PCCBitstreamStat& getBitstreamStat() { return *bitstreamStat_; }

  // V3C unit related functions
  V3CUnitHeader& getV3CUnitHeaderAVD() { return v3cUnitHeader_[size_t( V3C_AVD ) - 1]; }  // 3
  V3CUnitHeader& getV3CUnitHeaderGVD() { return v3cUnitHeader_[size_t( V3C_GVD ) - 1]; }  // 2
  V3CUnitHeader& getV3CUnitHeaderOVD() { return v3cUnitHeader_[size_t( V3C_OVD ) - 1]; }  // 1
  V3CUnitHeader& getV3CUnitHeaderAD() { return v3cUnitHeader_[size_t( V3C_AD ) - 1]; }    // 0
  V3CUnitHeader& getV3CUnitHeader( int index ) { return v3cUnitHeader_[index]; }

  // VPS related functions
  V3CParameterSet& getVps() { return getVps( activeVPS_ ); }
  V3CParameterSet& getVps( uint8_t vps_id ) {
    for ( auto& value : vpccParameterSets_ ) {
      if ( value.getV3CParameterSetId() == vps_id ) { return value; }
    }
    printf( "ERROR: can't get vps with id %d \n", vps_id );
    fflush( stdout );
    assert( 0 );
    exit( -1 );
  }
  std::vector<V3CParameterSet>& getVpsList() { return vpccParameterSets_; }
  void                          setActiveVpsId( uint8_t val ) { activeVPS_ = val; }
  V3CParameterSet&              addV3CParameterSet( uint8_t index ) {
    V3CParameterSet sps;
    sps.setV3CParameterSetId( index );
    vpccParameterSets_.push_back( sps );
    return vpccParameterSets_.back();
  }
  V3CParameterSet& addV3CParameterSet() {
    uint8_t         index = vpccParameterSets_.size();
    V3CParameterSet sps;
    sps.setV3CParameterSetId( index );
    vpccParameterSets_.push_back( sps );
    return vpccParameterSets_.back();
  }

  // high-level syntax related to the atlas sub-stream
  PCCAtlasHighLevelSyntax& getAtlasHighLevelSyntax( size_t atlasId ) { return atlasHLS_[atlasId]; }
  void                     allocateAtlasHLS( size_t size ) {
    atlasHLS_.resize( size );
    atlasIndex_ = 0;
  }
  void setAtlasIndex( size_t atlId ) { atlasIndex_ = atlId; }

  // All the functions below are just redirect to the corresponding atlas function
  // video related functions
  PCCVideoBitstream& createVideoBitstream( PCCVideoType type ) {
    return atlasHLS_[atlasIndex_].createVideoBitstream( type );
  }
  size_t             getVideoBitstreamCount() { return atlasHLS_[atlasIndex_].getVideoBitstreamCount(); }
  PCCVideoBitstream& getVideoBitstream( size_t index ) { return atlasHLS_[atlasIndex_].getVideoBitstream( index ); }
  PCCVideoBitstream& getVideoBitstream( PCCVideoType type ) { return atlasHLS_[atlasIndex_].getVideoBitstream( type ); }
  void               printVideoBitstream() { return atlasHLS_[atlasIndex_].printVideoBitstream(); };

  // ASPS related functions
  AtlasSequenceParameterSetRbsp& getAtlasSequenceParameterSet( size_t setId ) {
    return atlasHLS_[atlasIndex_].getAtlasSequenceParameterSet( setId );
  }
  AtlasSequenceParameterSetRbsp& addAtlasSequenceParameterSet() {
    return atlasHLS_[atlasIndex_].addAtlasSequenceParameterSet();
  }
  AtlasSequenceParameterSetRbsp& addAtlasSequenceParameterSet( uint8_t setId ) {
    return atlasHLS_[atlasIndex_].addAtlasSequenceParameterSet( setId );
  }
  std::vector<AtlasSequenceParameterSetRbsp>& getAtlasSequenceParameterSetList() {
    return atlasHLS_[atlasIndex_].getAtlasSequenceParameterSetList();
  }

  void setNumOfRefAtlasFrameList( size_t value ) { atlasHLS_[atlasIndex_].setNumOfRefAtlasFrameList( value ); }
  void setSizeOfRefAtlasFrameList( size_t listIndex, size_t listSize ) {
    atlasHLS_[atlasIndex_].setSizeOfRefAtlasFrameList( listIndex, listSize );
  }
  void setRefAtlasFrame( size_t listIndex, size_t refIndex, int32_t value ) {
    atlasHLS_[atlasIndex_].setRefAtlasFrame( listIndex, refIndex, value );
  }
  void setRefAtlasFrameList( std::vector<std::vector<int32_t>>& list ) {
    atlasHLS_[atlasIndex_].setRefAtlasFrameList( list );
  }
  size_t getNumOfRefAtlasFrameList() { return atlasHLS_[atlasIndex_].getNumOfRefAtlasFrameList(); }
  size_t getSizeOfRefAtlasFrameList( size_t listIndex ) {
    return atlasHLS_[atlasIndex_].getSizeOfRefAtlasFrameList( listIndex );
  }
  int32_t getRefAtlasFrame( size_t listIndex, size_t refIndex ) {
    return atlasHLS_[atlasIndex_].getRefAtlasFrame( listIndex, refIndex );
  }
  std::vector<int32_t>& getRefAtlasFrameList( size_t listIndex ) {
    return atlasHLS_[atlasIndex_].getRefAtlasFrameList( listIndex );
  }
  size_t getNumRefIdxActive( AtlasTileHeader& ath ) { return atlasHLS_[atlasIndex_].getNumRefIdxActive( ath ); };
  size_t getMaxNumRefAtlasFrame( size_t listIndex ) {
    return atlasHLS_[atlasIndex_].getMaxNumRefAtlasFrame( listIndex );
  }
  void setMaxNumRefAtlasFrame( size_t value ) { atlasHLS_[atlasIndex_].setMaxNumRefAtlasFrame( value ); }

  // point local recosntruction, defined in ASPS
  void addPointLocalReconstructionMode( const PointLocalReconstructionMode& mode ) {
    atlasHLS_[atlasIndex_].addPointLocalReconstructionMode( mode );
  }
  PointLocalReconstructionMode& getPointLocalReconstructionMode( size_t index ) {
    return atlasHLS_[atlasIndex_].getPointLocalReconstructionMode( index );
  }
  size_t getPointLocalReconstructionModeNumber() {
    return atlasHLS_[atlasIndex_].getPointLocalReconstructionModeNumber();
  }

  // AFPS related functions
  AtlasFrameParameterSetRbsp& getAtlasFrameParameterSet( size_t setId ) {
    return atlasHLS_[atlasIndex_].getAtlasFrameParameterSet( setId );
  }
  std::vector<AtlasFrameParameterSetRbsp>& getAtlasFrameParameterSetList() {
    return atlasHLS_[atlasIndex_].getAtlasFrameParameterSetList();
  }
  AtlasFrameParameterSetRbsp& addAtlasFrameParameterSet() { return atlasHLS_[atlasIndex_].addAtlasFrameParameterSet(); }
  AtlasFrameParameterSetRbsp& addAtlasFrameParameterSet( AtlasFrameParameterSetRbsp& refAfps ) {
    return atlasHLS_[atlasIndex_].addAtlasFrameParameterSet( refAfps );
  }
  AtlasFrameParameterSetRbsp& addAtlasFrameParameterSet( uint8_t setId ) {
    return atlasHLS_[atlasIndex_].addAtlasFrameParameterSet( setId );
  }

  // ATGL related functions
  AtlasTileLayerRbsp& addAtlasTileLayer() { return atlasHLS_[atlasIndex_].addAtlasTileLayer(); }  // decoder
  AtlasTileLayerRbsp& addAtlasTileLayer( size_t frameIdx, size_t tileIdx ) {
    return atlasHLS_[atlasIndex_].addAtlasTileLayer( frameIdx, tileIdx );
  }
  std::vector<AtlasTileLayerRbsp>& getAtlasTileLayerList() { return atlasHLS_[atlasIndex_].getAtlasTileLayerList(); }
  AtlasTileLayerRbsp&              getAtlasTileLayer( size_t atglOrder ) {
    return atlasHLS_[atlasIndex_].getAtlasTileLayer( atglOrder );
  }

  // SEI related functions
  SEI& addSei( NalUnitType nalUnitType, SeiPayloadType payloadType ) {
    return atlasHLS_[atlasIndex_].addSei( nalUnitType, payloadType );
  }
  bool seiIsPresent( NalUnitType nalUnitType, SeiPayloadType payloadType ) {
    return atlasHLS_[atlasIndex_].seiIsPresent( nalUnitType, payloadType );
  }
  SEI* getSei( NalUnitType nalUnitType, SeiPayloadType payloadType ) {
    return atlasHLS_[atlasIndex_].getSei( nalUnitType, payloadType );
  }
  SEI* getLastSei( NalUnitType nalUnitType, SeiPayloadType payloadType ) {
    return atlasHLS_[atlasIndex_].getLastSei( nalUnitType, payloadType );
  }
  SEI& addSeiPrefix( SeiPayloadType payloadType, bool essensial ) {
    return atlasHLS_[atlasIndex_].addSeiPrefix( payloadType, essensial );
  }
  SEI& addSeiSuffix( SeiPayloadType payloadType, bool essensial ) {
    return atlasHLS_[atlasIndex_].addSeiSuffix( payloadType, essensial );
  }
  std::vector<std::shared_ptr<SEI>>& getSeiPrefix() { return atlasHLS_[atlasIndex_].getSeiPrefix(); }
  std::vector<std::shared_ptr<SEI>>& getSeiSuffix() { return atlasHLS_[atlasIndex_].getSeiSuffix(); }
  SEI& getSeiPrefix( size_t index ) { return atlasHLS_[atlasIndex_].getSeiPrefix( index ); }
  SEI& getSeiSuffix( size_t index ) { return atlasHLS_[atlasIndex_].getSeiSuffix( index ); }

  std::vector<SEIDecodedAtlasInformationHash>& getSeiHash() { return seiHash_; }
  SEIDecodedAtlasInformationHash&              getSeiHash( size_t index ) { return seiHash_[index]; }

  void addSeiHashToSeiSuffix( size_t i ) {
    return atlasHLS_[atlasIndex_].addSeiToSeiSuffix( DECODED_ATLAS_INFORMATION_HASH, true, seiHash_[i] );
  }

  void allocateSeiHash( size_t hashCount ) { seiHash_.resize( hashCount ); }
  void allocateSeiHash() {
    size_t size = seiHash_.size();
    seiHash_.resize( size + 1 );
  }

  uint8_t getOccupancyPrecision() { return occupancyPrecision_; }
  uint8_t getLog2PatchQuantizerSizeX() { return log2PatchQuantizerSizeX_; }
  uint8_t getLog2PatchQuantizerSizeY() { return log2PatchQuantizerSizeY_; }
  bool    getEnablePatchSizeQuantization() { return enablePatchSizeQuantization_; }
  size_t  getAuxVideoWidth() { return auxVideWidth_; }
  size_t  getAuxVideoHeight() {
    size_t videoHeight = 0;
    for ( auto& height : auxTileHeight_ ) { videoHeight += height; }
    return videoHeight;
  }
  size_t               getAuxTileHeight( size_t index ) { return auxTileHeight_[index]; }
  std::vector<size_t>& getAuxTileHeight() { return auxTileHeight_; }
  size_t               getAuxTileLeftTopY( size_t index ) { return auxTileLeftTopY_[index]; }
  std::vector<size_t>& getAuxTileLeftTopY() { return auxTileLeftTopY_; }
  bool&                getPrefilterLossyOM() { return prefilterLossyOM_; }
  size_t&              getOffsetLossyOM() { return offsetLossyOM_; }
  size_t               getGeometry3dCoordinatesBitdepth() { return geometry3dCoordinatesBitdepth_; }
  void                 setOccupancyPrecision( uint8_t value ) { occupancyPrecision_ = value; }
  void                 setLog2PatchQuantizerSizeX( uint8_t value ) { log2PatchQuantizerSizeX_ = value; }
  void                 setLog2PatchQuantizerSizeY( uint8_t value ) { log2PatchQuantizerSizeY_ = value; }
  void                 setEnablePatchSizeQuantization( bool value ) { enablePatchSizeQuantization_ = value; }
  void                 setAuxVideoWidth( size_t value ) { auxVideWidth_ = value; }
  void                 setAuxTileHeight( size_t index, size_t value ) { auxTileHeight_[index] = value; }
  void                 setAuxTileLeftTopY( size_t index, size_t value ) { auxTileLeftTopY_[index] = value; }
  void                 setPrefilterLossyOM( bool value ) { prefilterLossyOM_ = value; }
  void                 setOffsetLossyOM( size_t value ) { offsetLossyOM_ = value; }
  void                 setGeometry3dCoordinatesBitdepth( size_t value ) { geometry3dCoordinatesBitdepth_ = value; }
  bool&                getSingleLayerMode() { return singleLayerMode_; }

 private:
  std::vector<PCCVideoBitstream>              videoBitstream_;
  V3CUnitHeader                               v3cUnitHeader_[5];
  std::vector<V3CParameterSet>                vpccParameterSets_;
  uint8_t                                     activeVPS_;
  uint8_t                                     occupancyPrecision_;
  uint8_t                                     log2PatchQuantizerSizeX_;
  uint8_t                                     log2PatchQuantizerSizeY_;
  bool                                        enablePatchSizeQuantization_;
  bool                                        prefilterLossyOM_;
  size_t                                      offsetLossyOM_;
  size_t                                      geometry3dCoordinatesBitdepth_;
  bool                                        singleLayerMode_;
  PCCBitstreamStat*                           bitstreamStat_;
  std::vector<PCCAtlasHighLevelSyntax>        atlasHLS_;
  size_t                                      atlasIndex_;
  size_t                                      auxVideWidth_;
  std::vector<size_t>                         auxTileHeight_;
  std::vector<size_t>                         auxTileLeftTopY_;
  std::vector<SEIDecodedAtlasInformationHash> seiHash_;
};
};  // namespace pcc

#endif /* PCC_BITSTREAM_HIGHLEVELSYNTAX_H */
