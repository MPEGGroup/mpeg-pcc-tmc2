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
#ifndef PCCContext_h
#define PCCContext_h

#include "PCCCommon.h"
#include "PCCVideo.h"
#include "PCCMetadata.h"
#include "PCCVideoBitstream.h"
#include <map>

namespace pcc {

class PCCGroupOfFrames;
class PCCFrameContext;
typedef pcc::PCCVideo<uint8_t, 3>  PCCVideoTexture;
typedef pcc::PCCVideo<uint16_t, 3> PCCVideoGeometry;
typedef pcc::PCCVideo<uint8_t, 3>  PCCVideoOccupancyMap;

class PCCPatch;
typedef std::map<size_t, PCCPatch>                 unionPatch;     // [TrackIndex, PatchUnion]
typedef std::pair<size_t, size_t>                  GlobalPatch;    // [FrameIndex, PatchIndex]
typedef std::map<size_t, std::vector<GlobalPatch>> GlobalPatches;  // [TrackIndex, <GlobalPatch>]
typedef std::pair<size_t, size_t>                  SubContext;     // [start, end)

// 7.3.5.22 Supplemental enhancement information message syntax
class SupplementalEnhancementInformationMessage {
 public:
  SupplementalEnhancementInformationMessage() {}
  ~SupplementalEnhancementInformationMessage() {}
  SupplementalEnhancementInformationMessage& operator=( const SupplementalEnhancementInformationMessage& ) = default;

  uint8_t getSmPayloadTypeByte() { return smPayloadTypeByte_; }
  uint8_t getSmPayloadSizeByte() { return smPayloadSizeByte_; }
  void    setSmPayloadTypeByte( uint8_t value ) { smPayloadTypeByte_ = value; }
  void    setSmPayloadSizeByte( uint8_t value ) { smPayloadSizeByte_ = value; }

 private:
  size_t smPayloadTypeByte_;
  size_t smPayloadSizeByte_;
};

// 7.3.34' Point local reconstruction syntax
class PointLocalReconstructionInformation {
 public:
  PointLocalReconstructionInformation() : plrlNumberOfModesMinus1_( 0 ), plrBlockThresholdPerPatchMinus1_( 0 ) {
    plrlMinimumDepth_.clear();
    plrlNeighbourMinus1_.clear();
    plrlInterpolateFlag_.clear();
    plrlFillingFlag_.clear();
  };
  ~PointLocalReconstructionInformation() {
    plrlMinimumDepth_.clear();
    plrlNeighbourMinus1_.clear();
    plrlInterpolateFlag_.clear();
    plrlFillingFlag_.clear();
  };

  PointLocalReconstructionInformation& operator=( const PointLocalReconstructionInformation& ) = default;

  void allocate() {
    plrlMinimumDepth_.resize( plrlNumberOfModesMinus1_ + 1, 0 );
    plrlNeighbourMinus1_.resize( plrlNumberOfModesMinus1_ + 1, 0 );
    plrlInterpolateFlag_.resize( plrlNumberOfModesMinus1_ + 1, false );
    plrlFillingFlag_.resize( plrlNumberOfModesMinus1_ + 1, false );
  }

  uint8_t getPlrlNumberOfModesMinus1() { return plrlNumberOfModesMinus1_; }
  uint8_t getPlrBlockThresholdPerPatchMinus1() { return plrBlockThresholdPerPatchMinus1_; }
  uint8_t getPlrlMinimumDepth( size_t index ) { return plrlMinimumDepth_[index]; }
  uint8_t getPlrlNeighbourMinus1( size_t index ) { return plrlNeighbourMinus1_[index]; }
  bool    getPlrlInterpolateFlag( size_t index ) { return plrlInterpolateFlag_[index]; }
  bool    getPlrlFillingFlag( size_t index ) { return plrlFillingFlag_[index]; }
  void    setPlrlNumberOfModesMinus1( uint8_t value ) { plrlNumberOfModesMinus1_ = value; }
  void    setPlrBlockThresholdPerPatchMinus1( uint8_t value ) { plrBlockThresholdPerPatchMinus1_ = value; }
  void    setPlrlMinimumDepth( size_t index, uint8_t value ) { plrlMinimumDepth_[index] = value; }
  void    setPlrlNeighbourMinus1( size_t index, uint8_t value ) { plrlNeighbourMinus1_[index] = value; }
  void    setPlrlInterpolateFlag( size_t index, bool value ) { plrlInterpolateFlag_[index] = value; }
  void    setPlrlFillingFlag( size_t index, bool value ) { plrlFillingFlag_[index] = value; }

 private:
  uint8_t              plrlNumberOfModesMinus1_;
  uint8_t              plrBlockThresholdPerPatchMinus1_;
  std::vector<uint8_t> plrlMinimumDepth_;
  std::vector<uint8_t> plrlNeighbourMinus1_;
  std::vector<bool>    plrlInterpolateFlag_;
  std::vector<bool>    plrlFillingFlag_;
};

// 7.3.34 Point local reconstruction data syntax
class PointLocalReconstructionData {
 public:
  PointLocalReconstructionData() :
      plrBlockToPatchMapHeight_( 0 ),
      plrBlockToPatchMapWidth_( 0 ),
      plrLevelFlag_( 0 ),
      plrPresentFlag_( false ),
      plrModeMinus1_( 0 ) {
    plrBlockPresentFlag_.clear();
    plrBlockModeMinus1_.clear();
  };
  ~PointLocalReconstructionData() {
    plrBlockPresentFlag_.clear();
    plrBlockModeMinus1_.clear();
  };

  PointLocalReconstructionData& operator=( const PointLocalReconstructionData& ) = default;

  void allocate( size_t plrBlockToPatchMapWidth, size_t plrBlockToPatchMapHeight ) {
    plrBlockToPatchMapWidth_  = plrBlockToPatchMapWidth;
    plrBlockToPatchMapHeight_ = plrBlockToPatchMapHeight;
    plrBlockPresentFlag_.resize( plrBlockToPatchMapWidth_ * plrBlockToPatchMapHeight_, false );
    plrBlockModeMinus1_.resize( plrBlockToPatchMapWidth_ * plrBlockToPatchMapHeight_, 0 );
  }
  size_t  getPlrBlockToPatchMapHeight() { return plrBlockToPatchMapHeight_; }
  size_t  getPlrBlockToPatchMapWidth() { return plrBlockToPatchMapWidth_; }
  bool    getPlrLevelFlag() { return plrLevelFlag_; }
  bool    getPlrPresentFlag() { return plrPresentFlag_; }
  uint8_t getPlrModeMinus1() { return plrModeMinus1_; }
  bool    getPlrBlockPresentFlag( size_t index ) { return plrBlockPresentFlag_[index]; }
  uint8_t getPlrBlockModeMinus1( size_t index ) { return plrBlockModeMinus1_[index]; }

  void setPlrLevelFlag( bool value ) { plrLevelFlag_ = value; }
  void setPlrPresentFlag( bool value ) { plrPresentFlag_ = value; }
  void setPlrModeMinus1( uint8_t value ) { plrModeMinus1_ = value; }
  void setPlrBlockPresentFlag( size_t index, bool value ) { plrBlockPresentFlag_[index] = value; }
  void setPlrBlockModeMinus1( size_t index, uint8_t value ) { plrBlockModeMinus1_[index] = value; }

 private:
  size_t               plrBlockToPatchMapHeight_;
  size_t               plrBlockToPatchMapWidth_;
  bool                 plrLevelFlag_;
  bool                 plrPresentFlag_;
  uint8_t              plrModeMinus1_;
  std::vector<bool>    plrBlockPresentFlag_;
  std::vector<uint8_t> plrBlockModeMinus1_;
};

// 7.3.5.20  PCM patch data unit syntax (
class PCMPatchDataUnit {
 public:
  PCMPatchDataUnit() :
      ppduPatchInPcmVideoFlag_( false ),
      ppdu2DShiftU_( 0 ),
      ppdu2DShiftV_( 0 ),
      ppdu2DDeltaSizeU_( 0 ),
      ppdu2DDeltaSizeV_( 0 ),
      ppdu3DShiftTangentAxis_( 0 ),
      ppdu3DShiftBiTangentAxis_( 0 ),
      ppdu3DShiftNormalAxis_( 0 ),
      ppduPcmPoints_( 0 ),
      ppduPatchIndex_( 0 ),
      ppduFrameIndex_( 0 ){};
  ~PCMPatchDataUnit(){};
  PCMPatchDataUnit& operator=( const PCMPatchDataUnit& ) = default;

  bool     getPatchInPcmVideoFlag() { return ppduPatchInPcmVideoFlag_; }
  size_t   get2DShiftU() { return ppdu2DShiftU_; }
  size_t   get2DShiftV() { return ppdu2DShiftV_; }
  int64_t  get2DDeltaSizeU() { return ppdu2DDeltaSizeU_; }
  int64_t  get2DDeltaSizeV() { return ppdu2DDeltaSizeV_; }
  size_t   get3DShiftTangentAxis() { return ppdu3DShiftTangentAxis_; }
  size_t   get3DShiftBiTangentAxis() { return ppdu3DShiftBiTangentAxis_; }
  size_t   get3DShiftNormalAxis() { return ppdu3DShiftNormalAxis_; }
  uint32_t getPcmPoints() { return ppduPcmPoints_; }
  size_t   getPpduPatchIndex() { return ppduPatchIndex_; }
  size_t   getPpduFrameIndex() { return ppduFrameIndex_; }
  void     setPpduPatchIndex( size_t value ) { ppduPatchIndex_ = value; }
  void     setPpduFrameIndex( size_t value ) { ppduFrameIndex_ = value; }
  void     setPatchInPcmVideoFlag( bool value ) { ppduPatchInPcmVideoFlag_ = value; }
  void     set2DShiftU( size_t value ) { ppdu2DShiftU_ = value; }
  void     set2DShiftV( size_t value ) { ppdu2DShiftV_ = value; }
  void     set2DDeltaSizeU( int64_t value ) { ppdu2DDeltaSizeU_ = value; }
  void     set2DDeltaSizeV( int64_t value ) { ppdu2DDeltaSizeV_ = value; }
  void     set3DShiftTangentAxis( size_t value ) { ppdu3DShiftTangentAxis_ = value; }
  void     set3DShiftBiTangentAxis( size_t value ) { ppdu3DShiftBiTangentAxis_ = value; }
  void     set3DShiftNormalAxis( size_t value ) { ppdu3DShiftNormalAxis_ = value; }
  void     setPcmPoints( uint32_t value ) { ppduPcmPoints_ = value; }

 private:
  bool     ppduPatchInPcmVideoFlag_;
  size_t   ppdu2DShiftU_;
  size_t   ppdu2DShiftV_;
  int64_t  ppdu2DDeltaSizeU_;
  int64_t  ppdu2DDeltaSizeV_;
  size_t   ppdu3DShiftTangentAxis_;
  size_t   ppdu3DShiftBiTangentAxis_;
  size_t   ppdu3DShiftNormalAxis_;
  uint32_t ppduPcmPoints_;
  size_t   ppduPatchIndex_;
  size_t   ppduFrameIndex_;
};

// 7.3.5.19  Delta Patch data unit syntax
class DeltaPatchDataUnit {
 public:
  DeltaPatchDataUnit() :
      dpdutDeltaPatchIndex_( 0 ),
      dpdu2DDeltaShiftU_( 0 ),
      dpdu2DDeltaShiftV_( 0 ),
      dpdu2DDeltaSizeU_( 0 ),
      dpdu2DDeltaSizeV_( 0 ),
      dpdu3DDeltaShiftTangentAxis_( 0 ),
      dpdu3DDeltaShiftBiTangentAxis_( 0 ),
      dpdu3DDeltaShiftMinNormalAxis_( 0 ),
      dpdu3DShiftDeltaMaxNormalAxis_( 0 ),
      dpduProjectPlane_( pcc::PCCAxis6( 0 ) ),
      dpduLod_( 0 ),
      dpduPatchIndex_( 0 ),
      dpduFrameIndex_( 0 ){};
  ~DeltaPatchDataUnit(){};
  DeltaPatchDataUnit&           operator=( const DeltaPatchDataUnit& ) = default;
  int64_t                       getDeltaPatchIdx() { return dpdutDeltaPatchIndex_; }
  int64_t                       get2DDeltaShiftU() { return dpdu2DDeltaShiftU_; }
  int64_t                       get2DDeltaShiftV() { return dpdu2DDeltaShiftV_; }
  int64_t                       get2DDeltaSizeU() { return dpdu2DDeltaSizeU_; }
  int64_t                       get2DDeltaSizeV() { return dpdu2DDeltaSizeV_; }
  int64_t                       get3DDeltaShiftTangentAxis() { return dpdu3DDeltaShiftTangentAxis_; }
  int64_t                       get3DDeltaShiftBiTangentAxis() { return dpdu3DDeltaShiftBiTangentAxis_; }
  int64_t                       get3DDeltaShiftMinNormalAxis() { return dpdu3DDeltaShiftMinNormalAxis_; }
  int64_t                       get3DShiftDeltaMaxNormalAxis() { return dpdu3DShiftDeltaMaxNormalAxis_; }
  PCCAxis6                      getProjectPlane() { return dpduProjectPlane_; }
  uint8_t                       getLod() { return dpduLod_; }
  PointLocalReconstructionData& getPointLocalReconstructionData() { return pointLocalReconstructionData_; }
  size_t                        getDpduPatchIndex() { return dpduPatchIndex_; }
  size_t                        getDpduFrameIndex() { return dpduFrameIndex_; }
  void                          setDpduPatchIndex( size_t value ) { dpduPatchIndex_ = value; }
  void                          setDpduFrameIndex( size_t value ) { dpduFrameIndex_ = value; }
  void                          setDeltaPatchIdx( int64_t value ) { dpdutDeltaPatchIndex_ = value; }
  void                          set2DDeltaShiftU( int64_t value ) { dpdu2DDeltaShiftU_ = value; }
  void                          set2DDeltaShiftV( int64_t value ) { dpdu2DDeltaShiftV_ = value; }
  void                          set2DDeltaSizeU( int64_t value ) { dpdu2DDeltaSizeU_ = value; }
  void                          set2DDeltaSizeV( int64_t value ) { dpdu2DDeltaSizeV_ = value; }
  void                          set3DDeltaShiftTangentAxis( int64_t value ) { dpdu3DDeltaShiftTangentAxis_ = value; }
  void set3DDeltaShiftBiTangentAxis( int64_t value ) { dpdu3DDeltaShiftBiTangentAxis_ = value; }
  void set3DDeltaShiftMinNormalAxis( int64_t value ) { dpdu3DDeltaShiftMinNormalAxis_ = value; }
  void set3DShiftDeltaMaxNormalAxis( int64_t value ) { dpdu3DShiftDeltaMaxNormalAxis_ = value; }
  void setProjectPlane( PCCAxis6 value ) { dpduProjectPlane_ = value; }
  void setLod( uint8_t value ) { dpduLod_ = value; }
  void setPointLocalReconstructionData( PointLocalReconstructionData value ) { pointLocalReconstructionData_ = value; }

 private:
  int64_t                      dpdutDeltaPatchIndex_;
  int64_t                      dpdu2DDeltaShiftU_;
  int64_t                      dpdu2DDeltaShiftV_;
  int64_t                      dpdu2DDeltaSizeU_;
  int64_t                      dpdu2DDeltaSizeV_;
  int64_t                      dpdu3DDeltaShiftTangentAxis_;
  int64_t                      dpdu3DDeltaShiftBiTangentAxis_;
  int64_t                      dpdu3DDeltaShiftMinNormalAxis_;
  int64_t                      dpdu3DShiftDeltaMaxNormalAxis_;
  PCCAxis6                     dpduProjectPlane_;
  uint8_t                      dpduLod_;
  size_t                       dpduPatchIndex_;
  size_t                       dpduFrameIndex_;
  PointLocalReconstructionData pointLocalReconstructionData_;
};

// 7.3.5.18  Patch data unit syntax (pdu)
class PatchDataUnit {
 public:
  PatchDataUnit() :
      pdu2DShiftU_( 0 ),
      pdu2DShiftV_( 0 ),
      pdu2DDeltaSizeU_( 0 ),
      pdu2DDeltaSizeV_( 0 ),
      pdu3DShiftTangentAxis_( 0 ),
      pdu3DShiftBiTangentAxis_( 0 ),
      pdu3DShiftMinNormalAxis_( 0 ),
      pdu3DShiftDeltaMaxNormalAxis_( 255 ),
      pduProjectPlane_( pcc::PCCAxis6( 0 ) ),
      pduOrientationIndex_( 0 ),
      pduLod_( 0 ),
      pdu45DegreeProjectionPresentFlag_( false ),
      pdu45DegreeProjectionRotationAxis_( 0 ),
      pduPatchIndex_( 0 ),
      pduFrameIndex_( 0 ){};
  ~PatchDataUnit(){};

  PatchDataUnit& operator=( const PatchDataUnit& ) = default;

  size_t                        get2DShiftU() { return pdu2DShiftU_; }
  size_t                        get2DShiftV() { return pdu2DShiftV_; }
  int64_t                       get2DDeltaSizeU() { return pdu2DDeltaSizeU_; }
  int64_t                       get2DDeltaSizeV() { return pdu2DDeltaSizeV_; }
  size_t                        get3DShiftTangentAxis() { return pdu3DShiftTangentAxis_; }
  size_t                        get3DShiftBiTangentAxis() { return pdu3DShiftBiTangentAxis_; }
  size_t                        get3DShiftMinNormalAxis() { return pdu3DShiftMinNormalAxis_; }
  size_t                        get3DShiftDeltaMaxNormalAxis() { return pdu3DShiftDeltaMaxNormalAxis_; }
  PCCAxis6                      getProjectPlane() { return pduProjectPlane_; }
  uint8_t                       getOrientationIndex() { return pduOrientationIndex_; }
  uint8_t                       getLod() { return pduLod_; }
  PointLocalReconstructionData& getPointLocalReconstructionData() { return pointLocalReconstructionData_; }
  bool                          get45DegreeProjectionPresentFlag() { return pdu45DegreeProjectionPresentFlag_; }
  uint8_t                       get45DegreeProjectionRotationAxis() { return pdu45DegreeProjectionRotationAxis_; }
  size_t                        getPduPatchIndex() { return pduPatchIndex_; }
  size_t                        getPduFrameIndex() { return pduFrameIndex_; }
  void                          setPduPatchIndex( size_t value ) { pduPatchIndex_ = value; }
  void                          setPduFrameIndex( size_t value ) { pduFrameIndex_ = value; }
  void                          set2DShiftU( size_t value ) { pdu2DShiftU_ = value; }
  void                          set2DShiftV( size_t value ) { pdu2DShiftV_ = value; }
  void                          set2DDeltaSizeU( int64_t value ) { pdu2DDeltaSizeU_ = value; }
  void                          set2DDeltaSizeV( int64_t value ) { pdu2DDeltaSizeV_ = value; }
  void                          set3DShiftTangentAxis( size_t value ) { pdu3DShiftTangentAxis_ = value; }
  void                          set3DShiftBiTangentAxis( size_t value ) { pdu3DShiftBiTangentAxis_ = value; }
  void                          set3DShiftMinNormalAxis( size_t value ) { pdu3DShiftMinNormalAxis_ = value; }
  void                          set3DShiftDeltaMaxNormalAxis( size_t value ) { pdu3DShiftDeltaMaxNormalAxis_ = value; }
  void                          setProjectPlane( PCCAxis6 value ) { pduProjectPlane_ = value; }
  void                          setOrientationIndex( uint8_t value ) { pduOrientationIndex_ = value; }
  void                          setLod( uint8_t value ) { pduLod_ = value; }
  void setPointLocalReconstructionData( PointLocalReconstructionData value ) { pointLocalReconstructionData_ = value; }
  void set45DegreeProjectionPresentFlag( bool value ) { pdu45DegreeProjectionPresentFlag_ = value; }
  void set45DegreeProjectionRotationAxis( uint8_t value ) { pdu45DegreeProjectionRotationAxis_ = value; }

 private:
  size_t                       pdu2DShiftU_;  // sizes need to be determined
  size_t                       pdu2DShiftV_;
  int64_t                      pdu2DDeltaSizeU_;
  int64_t                      pdu2DDeltaSizeV_;
  size_t                       pdu3DShiftTangentAxis_;
  size_t                       pdu3DShiftBiTangentAxis_;
  size_t                       pdu3DShiftMinNormalAxis_;
  size_t                       pdu3DShiftDeltaMaxNormalAxis_;
  PCCAxis6                     pduProjectPlane_;
  uint8_t                      pduOrientationIndex_;
  uint8_t                      pduLod_;
  PointLocalReconstructionData pointLocalReconstructionData_;
  bool                         pdu45DegreeProjectionPresentFlag_;
  uint8_t                      pdu45DegreeProjectionRotationAxis_;
  size_t                       pduPatchIndex_;
  size_t                       pduFrameIndex_;
};

// 7.3.5.17  Patch information data syntax (pid)
class PatchInformationData {
 public:
  PatchInformationData( size_t size = 0 ) { allocate( size ); };
  ~PatchInformationData() {
    overrideAttributePatchFlag_.clear();
    attributePatchParameterSetId_.clear();
  };

  void init() {
    overrideGeometryPatchFlag_ = false;
    overrideAttributePatchFlag_.clear();
    geometryPatchParameterSetId_ = 0;
    attributePatchParameterSetId_.clear();
  }

  PatchInformationData& operator=( const PatchInformationData& ) = default;

  void allocate( size_t size ) {
    if ( size ) {
      overrideAttributePatchFlag_.resize( size, false );
      attributePatchParameterSetId_.resize( size, 0 );
    }
  }
  void addOverrideAttributePatchFlag( bool value ) { overrideAttributePatchFlag_.push_back( value ); }
  void addAttributePatchParameterSetId( uint8_t value ) { attributePatchParameterSetId_.push_back( value ); }

  bool                getOverrideGeometryPatchFlag() { return overrideGeometryPatchFlag_; }
  uint8_t             getGeometryPatchParameterSetId() { return geometryPatchParameterSetId_; }
  bool                getOverrideAttributePatchFlag( size_t index ) { return overrideAttributePatchFlag_[index]; }
  uint8_t             getAttributePatchParameterSetId( size_t index ) { return attributePatchParameterSetId_[index]; }
  PatchDataUnit&      getPatchDataUnit() { return patchDataUnit_; }
  DeltaPatchDataUnit& getDeltaPatchDataUnit() { return deltaPatchDataUnit_; }
  PCMPatchDataUnit&   getPCMPatchDataUnit() { return pcmPatchDataUnit_; }
  size_t              getFrameIndex() { return frameIndex_; }
  size_t              getPatchIndex() { return patchIndex_; }

  void setFrameIndex( size_t value ) { frameIndex_ = value; }
  void setPatchIndex( size_t value ) { patchIndex_ = value; }
  void setOverrideGeometryPatchFlag( bool value ) { overrideGeometryPatchFlag_ = value; }
  void setGeometryPatchParameterSetId( uint8_t value ) { geometryPatchParameterSetId_ = value; }
  void setOverrideAttributePatchFlag( size_t index, bool value ) { overrideAttributePatchFlag_[index] = value; }
  void setAttributePatchParameterSetId( size_t index, uint8_t value ) { attributePatchParameterSetId_[index] = value; }
  void setPatchDataUnit( PatchDataUnit& dataUnit ) { patchDataUnit_ = dataUnit; }
  void setDeltaPatchDataUnit( DeltaPatchDataUnit& dataUnit ) { deltaPatchDataUnit_ = dataUnit; }
  void setPCMPatchDataUnit( PCMPatchDataUnit& dataUnit ) { pcmPatchDataUnit_ = dataUnit; }

 private:
  size_t               frameIndex_;
  size_t               patchIndex_;
  bool                 overrideGeometryPatchFlag_;
  uint8_t              geometryPatchParameterSetId_;
  std::vector<bool>    overrideAttributePatchFlag_;
  std::vector<uint8_t> attributePatchParameterSetId_;
  PatchDataUnit        patchDataUnit_;
  DeltaPatchDataUnit   deltaPatchDataUnit_;
  PCMPatchDataUnit     pcmPatchDataUnit_;
};

// 7.3.5.16  Patch frame data unit syntax (ptgdu)
class PatchTileGroupDataUnit {
 public:
  PatchTileGroupDataUnit() {}
  ~PatchTileGroupDataUnit() {
    patchMode_.clear();
    patchInformationData_.clear();
  }
  PatchTileGroupDataUnit& operator=( const PatchTileGroupDataUnit& ) = default;

  void init() {
    patchMode_.clear();
    patchInformationData_.clear();
  }
  void allocate( size_t size ) {
    patchMode_.resize( size );
    patchInformationData_.resize( size );
  }
  void addPatchMode( uint8_t value ) { patchMode_.push_back( value ); }
  void addPatchInformationData( PatchInformationData& value ) { patchInformationData_.push_back( value ); }
  PatchInformationData& addPatchInformationData() {
    PatchInformationData pid;
    patchInformationData_.push_back( pid );
    return patchInformationData_.back();
  }
  size_t                getPatchCount() { return patchMode_.size(); }
  uint8_t               getPatchMode( size_t index ) { return patchMode_[index]; }
  PatchInformationData& getPatchInformationData( size_t index ) { return patchInformationData_[index]; }
  size_t                getMatchedPatchCount() {
    size_t matchedPatchCount = 0;
    for ( auto& v : patchMode_ ) {
      if ( v == PATCH_MODE_P_INTER ) { matchedPatchCount++; }
    }
    return matchedPatchCount;
  }

  size_t getFrameIndex() { return frameIndex_; }
  void   setFrameIndex( size_t value ) { frameIndex_ = value; }
  void   setPatchCount( size_t value ) { patchCount_ = value; }
  void   setPatchMode( size_t index, uint8_t value ) { patchMode_[index] = value; }
  void   setPatchInformationData( size_t index, PatchInformationData& value ) { patchInformationData_[index] = value; }

 private:
  size_t                            frameIndex_;
  std::vector<uint8_t>              patchMode_;
  size_t                            patchCount_;
  std::vector<PatchInformationData> patchInformationData_;
};

// 7.3.5.15  Reference list structure syntax
class RefListStruct {
 public:
  RefListStruct() : numRefEntries_( 0 ) {
    absDeltaPfocSt_.clear();
    pfocLsbLt_.clear();
    stRefPatchFrameFlag_.clear();
    strpfEntrySignFlag_.clear();
  }
  ~RefListStruct() {
    absDeltaPfocSt_.clear();
    pfocLsbLt_.clear();
    stRefPatchFrameFlag_.clear();
    strpfEntrySignFlag_.clear();
  }
  RefListStruct& operator=( const RefListStruct& ) = default;

  void allocate() {
    absDeltaPfocSt_.resize( numRefEntries_, 0 );
    pfocLsbLt_.resize( numRefEntries_, 0 );
    stRefPatchFrameFlag_.resize( numRefEntries_, false );
    strpfEntrySignFlag_.resize( numRefEntries_, false );
  }

  void addAbsDeltaPfocSt( uint8_t value ) { absDeltaPfocSt_.push_back( value ); }
  void addPfocLsbLt( uint8_t value ) { pfocLsbLt_.push_back( value ); }
  void addStRefPatchFrameFlag( bool value ) { stRefPatchFrameFlag_.push_back( value ); }
  void addStrpfEntrySignFlag( bool value ) { strpfEntrySignFlag_.push_back( value ); }

  uint8_t getNumRefEntries() { return numRefEntries_; }
  uint8_t getAbsDeltaPfocSt( uint16_t index ) { return absDeltaPfocSt_[index]; }
  uint8_t getPfocLsbLt( uint16_t index ) { return pfocLsbLt_[index]; }
  bool    getStRefPatchFrameFlag( uint16_t index ) { return stRefPatchFrameFlag_[index]; }
  bool    getStrpfEntrySignFlag( uint16_t index ) { return strpfEntrySignFlag_[index]; }

  void setNumRefEntries( uint8_t value ) { numRefEntries_ = value; }
  void setAbsDeltaPfocSt( uint16_t index, uint8_t value ) { absDeltaPfocSt_[index] = value; }
  void setPfocLsbLt( uint16_t index, uint8_t value ) { pfocLsbLt_[index] = value; }
  void setStRefPatchFrameFlag( uint16_t index, bool value ) { stRefPatchFrameFlag_[index] = value; }
  void setStrpfEntrySignFlag( uint16_t index, bool value ) { strpfEntrySignFlag_[index] = value; }

 private:
  uint8_t              numRefEntries_;
  std::vector<uint8_t> absDeltaPfocSt_;
  std::vector<uint8_t> pfocLsbLt_;
  std::vector<bool>    stRefPatchFrameFlag_;
  std::vector<bool>    strpfEntrySignFlag_;
};

// 7.3.5.14  Patch frame header syntax (ptgh)
class PatchTileGroupHeader {
 public:
  PatchTileGroupHeader() :
      frameIndex_( 0 ),
      patchFrameParameterSetId_( 0 ),
      type_( 0 ),
      address_( 0 ),
      patchFrameOrderCntLsb_( 0 ),
      refPatchFrameListIdx_( 0 ),
      refPatchFrameListSpsFlag_( 0 ),
      numRefIdxActiveOverrideFlag_( false ),
      numRefIdxActiveMinus1_( 0 ),
      interPredictPatch2dShiftUBitCountMinus1_( 0 ),
      interPredictPatch2dShiftVBitCountMinus1_( 0 ),
      interPredictPatch2dDeltaSizeDBitCountMinus1_( 0 ),
      interPredictPatch3dShiftTangentAxisBitCountMinus1_( 0 ),
      interPredictPatch3dShiftBitangentAxisBitCountMinus1_( 0 ),
      interPredictPatch3dShiftNormalAxisBitCountMinus1_( 0 ),
      interPredictPatchLodBitCount_( 0 ),
      interPredictPatchBitCountFlag_( false ),
      interPredictPatch2dShiftUBitCountFlag_( false ),
      interPredictPatch2dShiftVBitCountFlag_( false ),
      interPredictPatch3dShiftTangentAxisBitCountFlag_( false ),
      interPredictPatch3dShiftBitangentAxisBitCountFlag_( false ),
      interPredictPatch3dShiftNormalAxisBitCountFlag_( false ),
      interPredictPatchLodBitCountFlag_( false ),
      pcm3dShiftAxisBitCountMinus1_( 9 ),
      pcm3dShiftBitCountPresentFlag_( true ) {
    additionalPfocLsbPresentFlag_.resize( 1, 0 );
    additionalPfocLsbVal_.resize( 1, 0 );
  }

  ~PatchTileGroupHeader() {
    additionalPfocLsbPresentFlag_.clear();
    additionalPfocLsbVal_.clear();
  }

  PatchTileGroupHeader& operator=( const PatchTileGroupHeader& ) = default;

  uint8_t  getFrameIndex() { return frameIndex_; }
  uint8_t  getPatchFrameParameterSetId() { return patchFrameParameterSetId_; }
  uint32_t getAddress() { return address_; }
  uint8_t  getType() { return type_; }
  uint8_t  getPatchFrameOrderCntLsb() { return patchFrameOrderCntLsb_; }
  bool     getRefPatchFrameListSpsFlag() { return refPatchFrameListSpsFlag_; }
  uint8_t  getRefPatchFrameListIdx() { return refPatchFrameListIdx_; }
  bool     getAdditionalPfocLsbPresentFlag( size_t index ) { return additionalPfocLsbPresentFlag_[index]; }
  uint32_t getAdditionalPfocLsbVal( size_t index ) { return additionalPfocLsbVal_[index]; }
  bool     getNumRefIdxActiveOverrideFlag() { return numRefIdxActiveOverrideFlag_; }
  uint8_t  getNumRefIdxActiveMinus1() { return numRefIdxActiveMinus1_; }
  bool     getInterPredictPatchBitCountFlag() { return interPredictPatchBitCountFlag_; }
  bool     getInterPredictPatch2dShiftUBitCountFlag() { return interPredictPatch2dShiftUBitCountFlag_; }
  bool     getInterPredictPatch2dShiftVBitCountFlag() { return interPredictPatch2dShiftVBitCountFlag_; }
  bool getInterPredictPatch3dShiftTangentAxisBitCountFlag() { return interPredictPatch3dShiftTangentAxisBitCountFlag_; }
  bool getInterPredictPatch3dShiftBitangentAxisBitCountFlag() {
    return interPredictPatch3dShiftBitangentAxisBitCountFlag_;
  }
  bool getInterPredictPatch3dShiftNormalAxisBitCountFlag() { return interPredictPatch3dShiftNormalAxisBitCountFlag_; }
  bool getInterPredictPatchLodBitCountFlag() { return interPredictPatchLodBitCountFlag_; }
  uint8_t getInterPredictPatch2dShiftUBitCountMinus1() { return interPredictPatch2dShiftUBitCountMinus1_; }
  uint8_t getInterPredictPatch2dShiftVBitCountMinus1() { return interPredictPatch2dShiftVBitCountMinus1_; }
  uint8_t getInterPredictPatch2dDeltaSizeDBitCountMinus1() { return interPredictPatch2dDeltaSizeDBitCountMinus1_; }
  uint8_t getInterPredictPatch3dShiftTangentAxisBitCountMinus1() {
    return interPredictPatch3dShiftTangentAxisBitCountMinus1_;
  }
  uint8_t getInterPredictPatch3dShiftBitangentAxisBitCountMinus1() {
    return interPredictPatch3dShiftBitangentAxisBitCountMinus1_;
  }
  uint8_t getInterPredictPatch3dShiftNormalAxisBitCountMinus1() {
    return interPredictPatch3dShiftNormalAxisBitCountMinus1_;
  }
  uint8_t getPcm3dShiftAxisBitCountMinus1() { return pcm3dShiftAxisBitCountMinus1_; }
  bool    getPcm3dShiftBitCountPresentFlag() { return pcm3dShiftBitCountPresentFlag_; }
  uint8_t getInterPredictPatchLodBitCount() { return interPredictPatchLodBitCount_; }
  uint8_t getNormalAxisMinValueQuantizer() { return normalAxisMinValueQuantizer_; }
  uint8_t getNormalAxisMaxDeltaValueQuantizer() { return normalAxisMaxDeltaValueQuantizer_; }

  void setFrameIndex( uint8_t index ) { frameIndex_ = index; }
  void setPatchFrameParameterSetId( uint8_t value ) { patchFrameParameterSetId_ = value; }
  void setAddress( uint8_t value ) { address_ = value; }
  void setType( uint8_t value ) { type_ = value; }
  void setPatchFrameOrderCntLsb( uint8_t value ) { patchFrameOrderCntLsb_ = value; }
  void setRefPatchFrameListSpsFlag( bool value ) { refPatchFrameListSpsFlag_ = value; }
  void setRefPatchFrameListIdx( uint8_t value ) { refPatchFrameListIdx_ = value; }
  void setAdditionalPfocLsbPresentFlag( size_t index, bool value ) { additionalPfocLsbPresentFlag_[index] = value; }
  void setAdditionalPfocLsbVal( size_t index, uint32_t value ) { additionalPfocLsbVal_[index] = value; }
  void setNumRefIdxActiveOverrideFlag( bool value ) { numRefIdxActiveOverrideFlag_ = value; }
  void setNumRefIdxActiveMinus1( uint8_t value ) { numRefIdxActiveMinus1_ = value; }
  void setInterPredictPatch2dDeltaSizeDBitCountMinus1( uint8_t value ) {
    interPredictPatch2dDeltaSizeDBitCountMinus1_ = value;
  }
  void setInterPredictPatch2dShiftUBitCountMinus1( uint8_t value ) { interPredictPatch2dShiftUBitCountMinus1_ = value; }
  void setInterPredictPatch2dShiftVBitCountMinus1( uint8_t value ) { interPredictPatch2dShiftVBitCountMinus1_ = value; }
  void setInterPredictPatch3dShiftTangentAxisBitCountMinus1( uint8_t value ) {
    interPredictPatch3dShiftTangentAxisBitCountMinus1_ = value;
  }
  void setInterPredictPatch3dShiftBitangentAxisBitCountMinus1( uint8_t value ) {
    interPredictPatch3dShiftBitangentAxisBitCountMinus1_ = value;
  }
  void setInterPredictPatch3dShiftNormalAxisBitCountMinus1( uint8_t value ) {
    interPredictPatch3dShiftNormalAxisBitCountMinus1_ = value;
  }
  void setPcm3dShiftAxisBitCountMinus1( uint8_t value ) { pcm3dShiftAxisBitCountMinus1_ = value; }
  void setPcm3dShiftBitCountPresentFlag( uint8_t value ) { pcm3dShiftBitCountPresentFlag_ = value; }
  void setInterPredictPatchLodBitCount( uint8_t value ) { interPredictPatchLodBitCount_ = value; }
  void setInterPredictPatchBitCountFlag( bool value ) { interPredictPatchBitCountFlag_ = value; }
  void setInterPredictPatch2dShiftUBitCountFlag( bool value ) { interPredictPatch2dShiftUBitCountFlag_ = value; }
  void setInterPredictPatch2dShiftVBitCountFlag( bool value ) { interPredictPatch2dShiftVBitCountFlag_ = value; }
  void setInterPredictPatch3dShiftTangentAxisBitCountFlag( bool value ) {
    interPredictPatch3dShiftTangentAxisBitCountFlag_ = value;
  }
  void setInterPredictPatch3dShiftBitangentAxisBitCountFlag( bool value ) {
    interPredictPatch3dShiftBitangentAxisBitCountFlag_ = value;
  }
  void setInterPredictPatch3dShiftNormalAxisBitCountFlag( bool value ) {
    interPredictPatch3dShiftNormalAxisBitCountFlag_ = value;
  }
  void setInterPredictPatchLodBitCountFlag( bool value ) { interPredictPatchLodBitCountFlag_ = value; }
  void setNormalAxisMinValueQuantizer( uint8_t value ) { normalAxisMinValueQuantizer_ = value; }
  void setNormalAxisMaxDeltaValueQuantizer( uint8_t value ) { normalAxisMaxDeltaValueQuantizer_ = value; }

 private:
  uint8_t               frameIndex_;
  uint8_t               patchFrameParameterSetId_;
  uint8_t               type_;
  uint32_t              address_;
  uint8_t               patchFrameOrderCntLsb_;
  uint8_t               refPatchFrameListIdx_;
  bool                  refPatchFrameListSpsFlag_;
  std::vector<bool>     additionalPfocLsbPresentFlag_;
  std::vector<uint32_t> additionalPfocLsbVal_;
  bool                  numRefIdxActiveOverrideFlag_;
  uint8_t               numRefIdxActiveMinus1_;
  uint8_t               normalAxisMinValueQuantizer_;
  uint8_t               normalAxisMaxDeltaValueQuantizer_;
  uint8_t               interPredictPatch2dShiftUBitCountMinus1_;
  uint8_t               interPredictPatch2dShiftVBitCountMinus1_;
  uint8_t               interPredictPatch2dDeltaSizeDBitCountMinus1_;
  uint8_t               interPredictPatch3dShiftTangentAxisBitCountMinus1_;
  uint8_t               interPredictPatch3dShiftBitangentAxisBitCountMinus1_;
  uint8_t               interPredictPatch3dShiftNormalAxisBitCountMinus1_;
  uint8_t               interPredictPatchLodBitCount_;
  bool                  interPredictPatchBitCountFlag_;
  bool                  interPredictPatch2dShiftUBitCountFlag_;
  bool                  interPredictPatch2dShiftVBitCountFlag_;
  bool                  interPredictPatch3dShiftTangentAxisBitCountFlag_;
  bool                  interPredictPatch3dShiftBitangentAxisBitCountFlag_;
  bool                  interPredictPatch3dShiftNormalAxisBitCountFlag_;
  bool                  interPredictPatchLodBitCountFlag_;
  uint8_t               pcm3dShiftAxisBitCountMinus1_;
  bool                  pcm3dShiftBitCountPresentFlag_;
};

// 7.3.5.13  Patch frame layer unit syntax
class PatchTileGroupLayerUnit {
 public:
  PatchTileGroupLayerUnit() : frameIndex_( 0 ) {}
  ~PatchTileGroupLayerUnit() {}

  PatchTileGroupLayerUnit& operator=( const PatchTileGroupLayerUnit& ) = default;

  uint8_t                 getFrameIndex() { return frameIndex_; }
  PatchTileGroupHeader&   getPatchTileGroupHeader() { return patchTileGroupHeader_; }
  PatchTileGroupDataUnit& getPatchTileGroupDataUnit() { return patchTileGroupDataUnit_; }

  void setFrameIndex( uint8_t value ) { frameIndex_ = value; }
  void setPatchTileGroupHeader( PatchTileGroupHeader value ) { patchTileGroupHeader_ = value; }
  void setPatchTileGroupDataUnit( PatchTileGroupDataUnit value ) { patchTileGroupDataUnit_ = value; }

 private:
  uint8_t                frameIndex_;
  PatchTileGroupHeader   patchTileGroupHeader_;
  PatchTileGroupDataUnit patchTileGroupDataUnit_;
};

class PatchFrameTileInformation {
 public:
  PatchFrameTileInformation() :
      singleTileInPatchFrameFlag_( 0 ),
      uniformTileSpacingFlag_( 0 ),
      numTileColumnsMinus1_( 0 ),
      numTileRowsMinus1_( 0 ),
      singleTilePerTileGroupFlag_( 0 ),
      numTileGroupsInPatchFrameMinus1_( 0 ),
      signalledTileGroupIdFlag_( 0 ),
      signalledTileGroupIdLengthMinus1_( 0 ) {
    tileColumnWidthMinus1_.resize( 1, 0 );
    tileRowHeightMinus1_.resize( 1, 0 );
    topLeftTileIdx_.resize( 1, 0 );
    bottomRightTileIdxDelta_.resize( 1, 0 );
    tileGroupId_.resize( 1, 0 );
  }
  ~PatchFrameTileInformation() {
    tileColumnWidthMinus1_.clear();
    tileRowHeightMinus1_.clear();
    topLeftTileIdx_.clear();
    bottomRightTileIdxDelta_.clear();
    tileGroupId_.clear();
  };

  PatchFrameTileInformation& operator=( const PatchFrameTileInformation& ) = default;

  bool     getSingleTileInPatchFrameFlag() { return singleTileInPatchFrameFlag_; }
  bool     getUniformTileSpacingFlag() { return uniformTileSpacingFlag_; }
  uint32_t getNumTileColumnsMinus1() { return numTileColumnsMinus1_; }
  uint32_t getNumTileRowsMinus1() { return numTileRowsMinus1_; }
  uint32_t getSingleTilePerTileGroupFlag() { return singleTilePerTileGroupFlag_; }
  uint32_t getNumTileGroupsInPatchFrameMinus1() { return numTileGroupsInPatchFrameMinus1_; }
  bool     getSignalledTileGroupIdFlag() { return signalledTileGroupIdFlag_; }
  uint32_t getSignalledTileGroupIdLengthMinus1() { return signalledTileGroupIdLengthMinus1_; }
  uint32_t getTileColumnWidthMinus1( size_t index ) { return tileColumnWidthMinus1_[index]; }
  uint32_t getTileRowHeightMinus1( size_t index ) { return tileRowHeightMinus1_[index]; }
  uint32_t getTopLeftTileIdx( size_t index ) { return topLeftTileIdx_[index]; }
  uint32_t getBottomRightTileIdxDelta( size_t index ) { return bottomRightTileIdxDelta_[index]; }
  uint32_t getTileGroupId( size_t index ) { return tileGroupId_[index]; }

  void setSingleTileInPatchFrameFlag( bool value ) { singleTileInPatchFrameFlag_ = value; }
  void setUniformTileSpacingFlag( bool value ) { uniformTileSpacingFlag_ = value; }
  void setNumTileColumnsMinus1( uint32_t value ) { numTileColumnsMinus1_ = value; }
  void setNumTileRowsMinus1( uint32_t value ) { numTileRowsMinus1_ = value; }
  void setSingleTilePerTileGroupFlag( uint32_t value ) { singleTilePerTileGroupFlag_ = value; }
  void setNumTileGroupsInPatchFrameMinus1( uint32_t value ) { numTileGroupsInPatchFrameMinus1_ = value; }
  void setSignalledTileGroupIdFlag( bool value ) { signalledTileGroupIdFlag_ = value; }
  void setSignalledTileGroupIdLengthMinus1( uint32_t value ) { signalledTileGroupIdLengthMinus1_ = value; }

  void setTileColumnWidthMinus1( size_t index, uint32_t value ) {
    if ( index == ( tileColumnWidthMinus1_.size() ) )
      tileColumnWidthMinus1_.resize( tileColumnWidthMinus1_.size() + 1 );
    else if ( index > tileColumnWidthMinus1_.size() )
      assert( 0 );
    tileColumnWidthMinus1_[index] = value;
  }
  void setTileRowHeightMinus1( size_t index, uint32_t value ) {
    if ( index == ( tileRowHeightMinus1_.size() ) )
      tileRowHeightMinus1_.resize( tileRowHeightMinus1_.size() + 1 );
    else if ( index > tileRowHeightMinus1_.size() )
      assert( 0 );
    tileRowHeightMinus1_[index] = value;
  }
  void setTopLeftTileIdx( size_t index, uint32_t value ) {
    if ( index == ( topLeftTileIdx_.size() ) )
      topLeftTileIdx_.resize( topLeftTileIdx_.size() + 1 );
    else if ( index > topLeftTileIdx_.size() )
      assert( 0 );
    topLeftTileIdx_[index] = value;
  }
  void setBottomRightTileIdxDelta( size_t index, uint32_t value ) {
    if ( index == ( bottomRightTileIdxDelta_.size() ) )
      bottomRightTileIdxDelta_.resize( bottomRightTileIdxDelta_.size() + 1 );
    else if ( index > bottomRightTileIdxDelta_.size() )
      assert( 0 );
    bottomRightTileIdxDelta_[index] = value;
  }
  void setTileGroupId( size_t index, uint32_t value ) {
    if ( index == ( tileGroupId_.size() ) )
      tileGroupId_.resize( tileGroupId_.size() + 1 );
    else if ( index > tileGroupId_.size() )
      assert( 0 );
    tileGroupId_[index] = value;
  }

 private:
  bool                  singleTileInPatchFrameFlag_;
  bool                  uniformTileSpacingFlag_;
  uint32_t              numTileColumnsMinus1_;
  uint32_t              numTileRowsMinus1_;
  uint32_t              singleTilePerTileGroupFlag_;
  uint32_t              numTileGroupsInPatchFrameMinus1_;
  bool                  signalledTileGroupIdFlag_;
  uint32_t              signalledTileGroupIdLengthMinus1_;
  std::vector<uint32_t> tileColumnWidthMinus1_;
  std::vector<uint32_t> tileRowHeightMinus1_;
  std::vector<uint32_t> topLeftTileIdx_;
  std::vector<uint32_t> bottomRightTileIdxDelta_;
  std::vector<uint32_t> tileGroupId_;
};

// 7.3.5.12  Patch frame parameter set syntax
class PatchFrameParameterSet {
 public:
  PatchFrameParameterSet() :
      patchFrameParameterSetId_( 0 ),
      patchSequenceParameterSetId_( 0 ),
      geometryPatchFrameParameterSetId_( 0 ),
      additionalLtPfocLsbLen_( 0 ),
      localOverrideGeometryPatchEnableFlag_( false ),
      projection45DegreeEnableFlag_( false ) {}
  ~PatchFrameParameterSet() {
    localOverrideAttributePatchEnableFlag_.clear();
    attributePatchFrameParameterSetId_.clear();
  }
  PatchFrameParameterSet& operator=( const PatchFrameParameterSet& ) = default;

  void allocate( size_t size = 255 ) {
    localOverrideAttributePatchEnableFlag_.resize( size, false );
    attributePatchFrameParameterSetId_.resize( size, 0 );
  }

  uint8_t getPatchFrameParameterSetId() { return patchFrameParameterSetId_; }
  uint8_t getPatchSequenceParameterSetId() { return patchSequenceParameterSetId_; }
  uint8_t getGeometryPatchFrameParameterSetId() { return geometryPatchFrameParameterSetId_; }
  uint8_t getAttributePatchFrameParameterSetId( size_t index ) { return attributePatchFrameParameterSetId_[index]; }
  uint8_t getAdditionalLtPfocLsbLen() { return additionalLtPfocLsbLen_; }
  bool    getLocalOverrideAttributePatchEnableFlag( size_t index ) {
    return localOverrideAttributePatchEnableFlag_[index];
  }
  bool                       getLocalOverrideGeometryPatchEnableFlag() { return localOverrideGeometryPatchEnableFlag_; }
  bool                       getProjection45DegreeEnableFlag() { return projection45DegreeEnableFlag_; }
  PatchFrameTileInformation& getPatchFrameTileInformation() { return patchFrameTileInformation_; }
  void                       setPatchFrameParameterSetId( uint8_t value ) { patchFrameParameterSetId_ = value; }
  void                       setPatchSequenceParameterSetId( uint8_t value ) { patchSequenceParameterSetId_ = value; }
  void setGeometryPatchFrameParameterSetId( uint8_t value ) { geometryPatchFrameParameterSetId_ = value; }
  void setAttributePatchFrameParameterSetId( size_t index, uint8_t value ) {
    attributePatchFrameParameterSetId_[index] = value;
  }
  void setAdditionalLtPfocLsbLen( uint8_t value ) { additionalLtPfocLsbLen_ = value; }
  void setLocalOverrideAttributePatchEnableFlag( size_t index, bool value ) {
    localOverrideAttributePatchEnableFlag_[index] = value;
  }
  void setLocalOverrideGeometryPatchEnableFlag( bool value ) { localOverrideGeometryPatchEnableFlag_ = value; }
  void setProjection45DegreeEnableFlag( bool value ) { projection45DegreeEnableFlag_ = value; }
  void setPatchFrameTileInformation( PatchFrameTileInformation value ) { patchFrameTileInformation_ = value; }

 private:
  uint8_t                   patchFrameParameterSetId_;
  uint8_t                   patchSequenceParameterSetId_;
  uint8_t                   geometryPatchFrameParameterSetId_;
  std::vector<uint8_t>      attributePatchFrameParameterSetId_;
  uint8_t                   additionalLtPfocLsbLen_;
  bool                      localOverrideGeometryPatchEnableFlag_;
  std::vector<bool>         localOverrideAttributePatchEnableFlag_;
  bool                      projection45DegreeEnableFlag_;
  PatchFrameTileInformation patchFrameTileInformation_;
};

// 7.3.5.11 Attribute patch params syntax (apps)
class AttributePatchParams {
 public:
  AttributePatchParams() :
      attributePatchScaleParamsPresentFlag_( false ),
      attributePatchOffsetParamsPresentFlag_( false ) {
    attributePatchScale_.clear();
    attributePatchOffset_.clear();
  }
  ~AttributePatchParams() {
    attributePatchScale_.clear();
    attributePatchOffset_.clear();
  }
  void allocate( size_t size = 255 ) {
    attributePatchScale_.resize( size, 0 );
    attributePatchOffset_.resize( size, 0 );
  }
  AttributePatchParams& operator=( const AttributePatchParams& ) = default;
  bool                  getAttributePatchScaleParamsPresentFlag() { return attributePatchScaleParamsPresentFlag_; }
  bool                  getAttributePatchOffsetParamsPresentFlag() { return attributePatchOffsetParamsPresentFlag_; }
  uint32_t              getAttributePatchScale( size_t index ) { return attributePatchScale_[index]; }
  int32_t               getAttributePatchOffset( size_t index ) { return attributePatchOffset_[index]; }

  void setAttributePatchScaleParamsPresentFlag( bool value ) { attributePatchScaleParamsPresentFlag_ = value; }
  void setAttributePatchOffsetParamsPresentFlag( bool value ) { attributePatchOffsetParamsPresentFlag_ = value; }
  void setAttributePatchScale( size_t index, uint32_t value ) { attributePatchScale_[index] = value; }
  void setAttributePatchOffset( size_t index, int32_t value ) { attributePatchOffset_[index] = value; }

 private:
  bool                  attributePatchScaleParamsPresentFlag_;
  std::vector<uint32_t> attributePatchScale_;
  bool                  attributePatchOffsetParamsPresentFlag_;
  std::vector<int32_t>  attributePatchOffset_;
};

// 7.3.5.10 Attribute Patch Parameter Set syntax (appss)
class AttributePatchParameterSet {
 public:
  AttributePatchParameterSet() :
      attributePatchParameterSetId_( 0 ),
      patchFrameAttributeParameterSetId_( 0 ),
      attributeDimensionMinus1_( 0 ),
      attributePatchParamsPresentFlag_( false ) {}
  ~AttributePatchParameterSet() {}
  AttributePatchParameterSet& operator=( const AttributePatchParameterSet& ) = default;

  uint8_t               getAttributePatchParameterSetId() { return attributePatchParameterSetId_; }
  uint8_t               getPatchFrameAttributeParameterSetId() { return patchFrameAttributeParameterSetId_; }
  bool                  getAttributePatchParamsPresentFlag() { return attributePatchParamsPresentFlag_; }
  AttributePatchParams& getAttributePatchParams() { return attributePatchParams_; }
  uint8_t               getAttributeDimensionMinus1() { return attributeDimensionMinus1_; }
  void                  setAttributePatchParameterSetId( uint8_t value ) { attributePatchParameterSetId_ = value; }
  void setPatchFrameAttributeParameterSetId( uint8_t value ) { patchFrameAttributeParameterSetId_ = value; }
  void setAttributePatchParamsPresentFlag( bool value ) { attributePatchParamsPresentFlag_ = value; }
  void setAttributePatchParams( AttributePatchParams value ) { attributePatchParams_ = value; }
  void setAttributeDimensionMinus1( uint8_t value ) { attributeDimensionMinus1_ = value; }

 private:
  uint8_t              attributePatchParameterSetId_;
  uint8_t              patchFrameAttributeParameterSetId_;
  uint8_t              attributeDimensionMinus1_;
  bool                 attributePatchParamsPresentFlag_;
  AttributePatchParams attributePatchParams_;
};

// 7.3.5.9 Geometry patch params syntax
class GeometryPatchParams {
 public:
  GeometryPatchParams() :
      geometryPatchScaleParamsPresentFlag_( false ),
      geometryPatchOffsetParamsPresentFlag_( false ),
      geometryPatchRotationParamsPresentFlag_( false ),
      geometryPatchPointSizeInfoPresentFlag_( false ),
      geometryPatchPointShapeInfoPresentFlag_( false ),
      geometryPatchPointSizeInfo_( 0 ),
      geometryPatchPointShapeInfo_( 0 ) {
    for ( size_t d = 0; d < 3; d++ ) {
      geometryPatchScaleOnAxis_[d]  = 0;  // u(32)
      geometryPatchOffsetOnAxis_[d] = 0;  // i(32)
    }
    for ( size_t d = 0; d < 4; d++ ) {
      geometryPatchRotationXYZW_[d] = 0;  // i(32)
    }
  }
  ~GeometryPatchParams() {}

  GeometryPatchParams& operator=( const GeometryPatchParams& ) = default;

  bool     getGeometryPatchScaleParamsPresentFlag() { return geometryPatchScaleParamsPresentFlag_; }
  bool     getGeometryPatchOffsetParamsPresentFlag() { return geometryPatchOffsetParamsPresentFlag_; }
  bool     getGeometryPatchRotationParamsPresentFlag() { return geometryPatchRotationParamsPresentFlag_; }
  bool     getGeometryPatchPointSizeInfoPresentFlag() { return geometryPatchPointSizeInfoPresentFlag_; }
  bool     getGeometryPatchPointShapeInfoPresentFlag() { return geometryPatchPointShapeInfoPresentFlag_; }
  uint32_t getGeometryPatchScaleOnAxis( size_t index ) { return geometryPatchScaleOnAxis_[index]; }
  int32_t  getGeometryPatchOffsetOnAxis( size_t index ) { return geometryPatchOffsetOnAxis_[index]; }
  int32_t  getGeometryPatchRotationQuaternion( size_t index ) { return geometryPatchRotationXYZW_[index]; }
  uint16_t getGeometryPatchPointSizeInfo() { return geometryPatchPointSizeInfo_; }
  uint32_t getGeometryPatchPointShapeInfo() { return geometryPatchPointShapeInfo_; }

  void setGeometryPatchScaleParamsPresentFlag( bool value ) { geometryPatchScaleParamsPresentFlag_ = value; }
  void setGeometryPatchOffsetParamsPresentFlag( bool value ) { geometryPatchOffsetParamsPresentFlag_ = value; }
  void setGeometryPatchRotationParamsPresentFlag( bool value ) { geometryPatchRotationParamsPresentFlag_ = value; }
  void setGeometryPatchPointSizeInfoPresentFlag( bool value ) { geometryPatchPointSizeInfoPresentFlag_ = value; }
  void setGeometryPatchPointShapeInfoPresentFlag( bool value ) { geometryPatchPointShapeInfoPresentFlag_ = value; }
  void setGeometryPatchScaleOnAxis( size_t index, uint32_t value ) { geometryPatchScaleOnAxis_[index] = value; }
  void setGeometryPatchOffsetOnAxis( size_t index, int32_t value ) { geometryPatchOffsetOnAxis_[index] = value; }
  void setGeometryPatchRotationQuaternion( size_t index, int32_t value ) { geometryPatchRotationXYZW_[index] = value; }
  void setGeometryPatchPointSizeInfo( uint16_t value ) { geometryPatchPointSizeInfo_ = value; }
  void setGeometryPatchPointShapeInfo( uint32_t value ) { geometryPatchPointShapeInfo_ = value; }

 private:
  bool     geometryPatchScaleParamsPresentFlag_;
  bool     geometryPatchOffsetParamsPresentFlag_;
  bool     geometryPatchRotationParamsPresentFlag_;
  bool     geometryPatchPointSizeInfoPresentFlag_;
  bool     geometryPatchPointShapeInfoPresentFlag_;
  uint32_t geometryPatchScaleOnAxis_[3];
  int32_t  geometryPatchOffsetOnAxis_[3];
  int32_t  geometryPatchRotationXYZW_[4];
  uint16_t geometryPatchPointSizeInfo_;
  uint32_t geometryPatchPointShapeInfo_;
};

// 7.3.5.8 Geometry patch parameter set syntax
class GeometryPatchParameterSet {
 public:
  GeometryPatchParameterSet() :
      geometryPatchParameterSetId_( 0 ),
      patchFrameGeometryParameterSetId_( 0 ),
      geometryPatchParamsPresentFlag_( false ) {}
  ~GeometryPatchParameterSet() {}
  GeometryPatchParameterSet& operator=( const GeometryPatchParameterSet& ) = default;

  uint8_t              getGeometryPatchParameterSetId() { return geometryPatchParameterSetId_; }
  uint8_t              getPatchFrameGeometryParameterSetId() { return patchFrameGeometryParameterSetId_; }
  bool                 getGeometryPatchParamsPresentFlag() { return geometryPatchParamsPresentFlag_; }
  GeometryPatchParams& getGeometryPatchParams() { return geometryPatchParams_; }

  void setGeometryPatchParameterSetId( uint8_t value ) { geometryPatchParameterSetId_ = value; }
  void setPatchFrameGeometryParameterSetId( uint8_t value ) { patchFrameGeometryParameterSetId_ = value; }
  void setGeometryPatchParamsPresentFlag( bool value ) { geometryPatchParamsPresentFlag_ = value; }
  void setGeometryPatchParams( GeometryPatchParams value ) { geometryPatchParams_ = value; }

 private:
  uint8_t             geometryPatchParameterSetId_;
  uint8_t             patchFrameGeometryParameterSetId_;
  bool                geometryPatchParamsPresentFlag_;
  GeometryPatchParams geometryPatchParams_;
};

// 7.3.5.7 Attribute frame paramS syntax (afp)
class AttributeFrameParams {
 public:
  AttributeFrameParams() :
      attributeSmoothingParamsPresentFlag_( false ),
      attributeScaleParamsPresentFlag_( false ),
      attributeOffsetParamsPresentFlag_( false ) {
    attributeScale_.clear();
    attributeOffset_.clear();
    attributeSmoothingParamsPresentFlag_.clear();
    attributeSmoothingGridSizeMinus2_.clear();
    attributeSmoothingThreshold_.clear();
    attributeSmoothingThresholdAttributeDifference_.clear();
    attributeSmoothingThresholdAttributeVariation_.clear();
    attributeSmoothingLocalEntropyThreshold_.clear();
  }
  ~AttributeFrameParams() {
    attributeScale_.clear();
    attributeOffset_.clear();
    attributeSmoothingParamsPresentFlag_.clear();
    attributeSmoothingGridSizeMinus2_.clear();
    attributeSmoothingThreshold_.clear();
    attributeSmoothingThresholdAttributeDifference_.clear();
    attributeSmoothingThresholdAttributeVariation_.clear();
    attributeSmoothingLocalEntropyThreshold_.clear();
  }

  AttributeFrameParams& operator=( const AttributeFrameParams& ) = default;

  void allocate( size_t size ) {
    attributeScale_.resize( size, 0 );
    attributeOffset_.resize( size, 0 );
    attributeSmoothingParamsPresentFlag_.resize( size, 0 );
    attributeSmoothingGridSizeMinus2_.resize( size, 0 );
    attributeSmoothingThreshold_.resize( size, 0 );
    attributeSmoothingThresholdAttributeDifference_.resize( size, 0 );
    attributeSmoothingThresholdAttributeVariation_.resize( size, 0 );
    attributeSmoothingLocalEntropyThreshold_.resize( size, 0 );
  }

  bool    getAttributeSmoothingParamsPresentFlag( size_t index ) { return attributeSmoothingParamsPresentFlag_[index]; }
  uint8_t getAttributeSmoothingGridSizeMinus2( size_t index ) { return attributeSmoothingGridSizeMinus2_[index]; }
  uint8_t getAttributeSmoothingThreshold( size_t index ) { return attributeSmoothingThreshold_[index]; }
  uint32_t getAttributeSmoothingLocalEntropyThreshold( size_t index ) {
    return attributeSmoothingLocalEntropyThreshold_[index];
  }
  uint8_t getAttributeSmoothingThresholdAttributeDifference( size_t index ) {
    return attributeSmoothingThresholdAttributeDifference_[index];
  }
  uint8_t getAttributeSmoothingThresholdAttributeVariation( size_t index ) {
    return attributeSmoothingThresholdAttributeVariation_[index];
  }
  bool     getAttributeScaleParamsPresentFlag() { return attributeScaleParamsPresentFlag_; }
  bool     getAttributeOffsetParamsPresentFlag() { return attributeOffsetParamsPresentFlag_; }
  uint32_t getAttributeScale( size_t index ) { return attributeScale_[index]; }
  int32_t  getAttributeOffset( size_t index ) { return attributeOffset_[index]; }

  void setAttributeSmoothingParamsPresentFlag( std::vector<bool> value ) {
    attributeSmoothingParamsPresentFlag_ = value;
  }
  void setAttributeSmoothingGridSize( std::vector<uint8_t> value ) { attributeSmoothingGridSizeMinus2_ = value; }
  void setAttributeSmoothingThreshold( std::vector<uint8_t> value ) { attributeSmoothingThreshold_ = value; }
  void setAttributeSmoothingLocalEntropyThreshold( std::vector<uint32_t> value ) {
    attributeSmoothingLocalEntropyThreshold_ = value;
  }
  void setAttributeSmoothingThresholdAttributeDifference( std::vector<uint8_t> value ) {
    attributeSmoothingThresholdAttributeDifference_ = value;
  }
  void setAttributeSmoothingThresholdAttributeVariation( std::vector<uint8_t> value ) {
    attributeSmoothingThresholdAttributeVariation_ = value;
  }

  void setAttributeSmoothingParamsPresentFlag( size_t index, bool value ) {
    attributeSmoothingParamsPresentFlag_[index] = value;
  }
  void setAttributeSmoothingGridSizeMinus2( size_t index, uint8_t value ) {
    attributeSmoothingGridSizeMinus2_[index] = value;
  }
  void setAttributeSmoothingThreshold( size_t index, uint8_t value ) { attributeSmoothingThreshold_[index] = value; }
  void setAttributeSmoothingLocalEntropyThreshold( size_t index, uint32_t value ) {
    attributeSmoothingLocalEntropyThreshold_[index] = value;
  }
  void setAttributeSmoothingThresholdAttributeDifference( size_t index, uint8_t value ) {
    attributeSmoothingThresholdAttributeDifference_[index] = value;
  }
  void setAttributeSmoothingThresholdAttributeVariation( size_t index, uint8_t value ) {
    attributeSmoothingThresholdAttributeVariation_[index] = value;
  }
  void setAttributeScaleParamsPresentFlag( bool value ) { attributeScaleParamsPresentFlag_ = value; }
  void setAttributeOffsetParamsPresentFlag( bool value ) { attributeOffsetParamsPresentFlag_ = value; }
  void setAttributeScale( size_t index, uint32_t value ) { attributeScale_[index] = value; }
  void setAttributeOffset( size_t index, int32_t value ) { attributeOffset_[index] = value; }

 private:
  std::vector<bool>     attributeSmoothingParamsPresentFlag_;
  std::vector<uint8_t>  attributeSmoothingGridSizeMinus2_;
  std::vector<uint8_t>  attributeSmoothingThreshold_;
  std::vector<uint32_t> attributeSmoothingLocalEntropyThreshold_;
  std::vector<uint8_t>  attributeSmoothingThresholdAttributeVariation_;
  std::vector<uint8_t>  attributeSmoothingThresholdAttributeDifference_;
  bool                  attributeScaleParamsPresentFlag_;
  bool                  attributeOffsetParamsPresentFlag_;
  std::vector<uint32_t> attributeScale_;
  std::vector<int32_t>  attributeOffset_;
};

// 7.3.5.6 Patch frame attribute parameter set syntax
class PatchFrameAttributeParameterSet {
 public:
  PatchFrameAttributeParameterSet() :
      patchFrameAttributeParameterSetId_( 0 ),
      patchSequencParameterSetId_( 0 ),
      attributeDimensionMinus1_( 3 ),
      attributePatchScaleParamsEnabledFlag_( false ),
      attributePatchOffsetParamsEnabledFlag_( false ) {}

  ~PatchFrameAttributeParameterSet() {}

  PatchFrameAttributeParameterSet& operator=( const PatchFrameAttributeParameterSet& ) = default;

  uint8_t               getPatchFrameAttributeParameterSetId() { return patchFrameAttributeParameterSetId_; }
  uint8_t               getPatchSequencParameterSetId() { return patchSequencParameterSetId_; }
  uint8_t               getAttributeDimensionMinus1() { return attributeDimensionMinus1_; }
  bool                  getAttributePatchScaleParamsEnabledFlag() { return attributePatchScaleParamsEnabledFlag_; }
  bool                  getAttributePatchOffsetParamsEnabledFlag() { return attributePatchOffsetParamsEnabledFlag_; }
  AttributeFrameParams& getAttributeFrameParams() { return attributeFrameParams_; }

  void setPatchFrameAttributeParameterSetId( uint8_t value ) { patchFrameAttributeParameterSetId_ = value; }
  void setPatchSequencParameterSetId( uint8_t value ) { patchSequencParameterSetId_ = value; }
  void setAttributeDimensionMinus1( uint8_t value ) { attributeDimensionMinus1_ = value; }
  void setAttributePatchScaleParamsEnabledFlag( bool value ) { attributePatchScaleParamsEnabledFlag_ = value; }
  void setAttributePatchOffsetParamsEnabledFlag( bool value ) { attributePatchOffsetParamsEnabledFlag_ = value; }
  void setAttributeFrameParams( AttributeFrameParams value ) { attributeFrameParams_ = value; }

 private:
  uint8_t              patchFrameAttributeParameterSetId_;
  uint8_t              patchSequencParameterSetId_;
  uint8_t              attributeDimensionMinus1_;
  bool                 attributePatchScaleParamsEnabledFlag_;
  bool                 attributePatchOffsetParamsEnabledFlag_;
  AttributeFrameParams attributeFrameParams_;
};

// 7.3.5.5 Geometry frame params syntax
class GeometryFrameParams {
 public:
  GeometryFrameParams() :
      geometrySmoothingParamsPresentFlag_( false ),
      geometryScaleParamsPresentFlag_( false ),
      geometryOffsetParamsPresentFlag_( false ),
      geometryRotationParamsPresentFlag_( false ),
      geometryPointSizeInfoPresentFlag_( false ),
      geometryPointShapeInfoPresentFlag_( false ),
      geometrySmoothingEnabledFlag_( false ),
      geometrySmoothingGridSizeMinus2_( 0 ),
      geometrySmoothingThreshold_( 0 ),
      geometryPointSizeInfo_( 0 ),
      geometryPointShapeInfo_( 0 ) {
    for ( uint8_t i = 0; i < 3; i++ ) {
      geometryScaleOnAxis_[i]  = 0;
      geometryOffsetOnAxis_[i] = 0;
    }
    for ( uint8_t i = 0; i < 4; i++ ) { geometryRotationXYZW_[i] = 0; }
  }
  ~GeometryFrameParams() {}
  GeometryFrameParams& operator=( const GeometryFrameParams& ) = default;

  bool     getGeometrySmoothingParamsPresentFlag() { return geometrySmoothingParamsPresentFlag_; }
  bool     getGeometryScaleParamsPresentFlag() { return geometryScaleParamsPresentFlag_; }
  bool     getGeometryOffsetParamsPresentFlag() { return geometryOffsetParamsPresentFlag_; }
  bool     getGeometryRotationParamsPresentFlag() { return geometryRotationParamsPresentFlag_; }
  bool     getGeometryPointSizeInfoPresentFlag() { return geometryPointSizeInfoPresentFlag_; }
  bool     getGeometryPointShapeInfoPresentFlag() { return geometryPointShapeInfoPresentFlag_; }
  bool     getGeometrySmoothingEnabledFlag() { return geometrySmoothingEnabledFlag_; }
  uint8_t  getGeometrySmoothingGridSizeMinus2() { return geometrySmoothingGridSizeMinus2_; }
  uint8_t  getGeometrySmoothingThreshold() { return geometrySmoothingThreshold_; }
  uint32_t getGeometryScaleOnAxis( size_t index ) { return geometryScaleOnAxis_[index]; }
  int32_t  getGeometryOffsetOnAxis( size_t index ) { return geometryOffsetOnAxis_[index]; }
  int32_t  getGeometryRotationQuaternion( size_t index ) { return geometryRotationXYZW_[index]; }
  uint16_t getGeometryPointSizeInfo() { return geometryPointSizeInfo_; }
  uint32_t getGeometryPointShapeInfo() { return geometryPointShapeInfo_; }

  void setGeometrySmoothingParamsPresentFlag( bool value ) { geometrySmoothingParamsPresentFlag_ = value; }
  void setGeometryScaleParamsPresentFlag( bool value ) { geometryScaleParamsPresentFlag_ = value; }
  void setGeometryOffsetParamsPresentFlag( bool value ) { geometryOffsetParamsPresentFlag_ = value; }
  void setGeometryRotationParamsPresentFlag( bool value ) { geometryRotationParamsPresentFlag_ = value; }
  void setGeometryPointSizeInfoPresentFlag( bool value ) { geometryPointSizeInfoPresentFlag_ = value; }
  void setGeometryPointShapeInfoPresentFlag( bool value ) { geometryPointShapeInfoPresentFlag_ = value; }
  void setGeometrySmoothingEnabledFlag( bool value ) { geometrySmoothingEnabledFlag_ = value; }
  void setGeometrySmoothingGridSizeMinus2( uint8_t value ) { geometrySmoothingGridSizeMinus2_ = value; }
  void setGeometrySmoothingThreshold( uint8_t value ) { geometrySmoothingThreshold_ = value; }
  void setGeometryScaleOnAxis( size_t index, uint32_t value ) { geometryScaleOnAxis_[index] = value; }
  void setGeometryOffsetOnAxis( size_t index, int32_t value ) { geometryOffsetOnAxis_[index] = value; }
  void setGeometryRotationQuaternion( size_t index, int32_t value ) { geometryRotationXYZW_[index] = value; }
  void setGeometryPointSizeInfo( uint16_t value ) { geometryPointSizeInfo_ = value; }
  void setGeometryPointShapeInfo( uint32_t value ) { geometryPointShapeInfo_ = value; }

 private:
  bool     geometrySmoothingParamsPresentFlag_;
  bool     geometryScaleParamsPresentFlag_;
  bool     geometryOffsetParamsPresentFlag_;
  bool     geometryRotationParamsPresentFlag_;
  bool     geometryPointSizeInfoPresentFlag_;
  bool     geometryPointShapeInfoPresentFlag_;
  bool     geometrySmoothingEnabledFlag_;
  uint8_t  geometrySmoothingGridSizeMinus2_;
  uint8_t  geometrySmoothingThreshold_;
  uint32_t geometryScaleOnAxis_[3];
  int32_t  geometryOffsetOnAxis_[3];
  int32_t  geometryRotationXYZW_[4];
  uint16_t geometryPointSizeInfo_;
  uint32_t geometryPointShapeInfo_;
};

// 7.3.5.4 Patch frame geometry parameter set syntax
class PatchFrameGeometryParameterSet {
 public:
  PatchFrameGeometryParameterSet() :
      patchFrameGeometryParameterSetId_( 0 ),
      patchSequenceParameterSetId_( 0 ),
      geometryPatchParamsEnabledFlag_( false ),
      geometryPatchScaleParamsEnabledFlag_( false ),
      geometryPatchOffsetParamsEnabledFlag_( false ),
      geometryPatchRotationParamsEnabledFlag_( false ),
      geometryPatchPointSizeInfoEnabledFlag_( false ),
      geometryPatchPointShapeInfoEnabledFlag_( false ) {}

  ~PatchFrameGeometryParameterSet() {}
  PatchFrameGeometryParameterSet& operator=( const PatchFrameGeometryParameterSet& ) = default;

  uint8_t              getPatchFrameGeometryParameterSetId() { return patchFrameGeometryParameterSetId_; }
  uint8_t              getPatchSequenceParameterSetId() { return patchSequenceParameterSetId_; }
  bool                 getGeometryPatchParamsEnabledFlag() { return geometryPatchParamsEnabledFlag_; }
  bool                 getOverrideGeometryPatchParamsFlag() { return overrideGeometryPatchParamsFlag_; }
  bool                 getGeometryPatchScaleParamsEnabledFlag() { return geometryPatchScaleParamsEnabledFlag_; }
  bool                 getGeometryPatchOffsetParamsEnabledFlag() { return geometryPatchOffsetParamsEnabledFlag_; }
  bool                 getGeometryPatchRotationParamsEnabledFlag() { return geometryPatchRotationParamsEnabledFlag_; }
  bool                 getGeometryPatchPointSizeInfoEnabledFlag() { return geometryPatchPointSizeInfoEnabledFlag_; }
  bool                 getGeometryPatchPointShapeInfoEnabledFlag() { return geometryPatchPointShapeInfoEnabledFlag_; }
  GeometryFrameParams& getGeometryFrameParams() { return geometryFrameParams_; }

  void setPatchFrameGeometryParameterSetId( uint8_t value ) { patchFrameGeometryParameterSetId_ = value; }
  void setPatchSequenceParameterSetId( uint8_t value ) { patchSequenceParameterSetId_ = value; }
  void setGeometryPatchParamsEnabledFlag( bool value ) { geometryPatchParamsEnabledFlag_ = value; }
  void setOverrideGeometryPatchParamsFlag( bool value ) { overrideGeometryPatchParamsFlag_ = value; }
  void setGeometryPatchScaleParamsEnabledFlag( bool value ) { geometryPatchScaleParamsEnabledFlag_ = value; }
  void setGeometryPatchOffsetParamsEnabledFlag( bool value ) { geometryPatchOffsetParamsEnabledFlag_ = value; }
  void setGeometryPatchRotationParamsEnabledFlag( bool value ) { geometryPatchRotationParamsEnabledFlag_ = value; }
  void setGeometryPatchPointSizeInfoEnabledFlag( bool value ) { geometryPatchPointSizeInfoEnabledFlag_ = value; }
  void setGeometryPatchPointShapeInfoEnabledFlag( bool value ) { geometryPatchPointShapeInfoEnabledFlag_ = value; }
  void setGeometryFrameParams( GeometryFrameParams value ) { geometryFrameParams_ = value; }

 private:
  uint8_t             patchFrameGeometryParameterSetId_;
  uint8_t             patchSequenceParameterSetId_;
  bool                geometryPatchParamsEnabledFlag_;
  bool                overrideGeometryPatchParamsFlag_;
  bool                geometryPatchScaleParamsEnabledFlag_;
  bool                geometryPatchOffsetParamsEnabledFlag_;
  bool                geometryPatchRotationParamsEnabledFlag_;
  bool                geometryPatchPointSizeInfoEnabledFlag_;
  bool                geometryPatchPointShapeInfoEnabledFlag_;
  GeometryFrameParams geometryFrameParams_;
};

// 7.3.5.3  Patch sequence parameter set syntax (psps)
class PatchSequenceParameterSet {
 public:
  PatchSequenceParameterSet() :
      patchSequenceParameterSetId_( 0 ),
      log2PatchPackingBlockSize_( 0 ),
      log2MaxPatchFrameOrderCntLsb_( 0 ),
      maxDecPatchFrameBufferingMinus1_( 0 ),
      numRefPatchFrameListsInPsps_( 0 ),
      longTermRefPatchFramesFlag_( false ),
      useEightOrientationsFlag_( false ),
      normalAxisLimitsQuantizationEnableFlag_( false ),
      normalAxisMaxDeltaValueEnableFlag_( false ) {
    refListStruct_.clear();
  }
  ~PatchSequenceParameterSet() { refListStruct_.clear(); }

  PatchSequenceParameterSet& operator=( const PatchSequenceParameterSet& ) = default;

  void           allocate( size_t size ) { refListStruct_.resize( size ); }
  uint8_t        getPatchSequenceParameterSetId() { return patchSequenceParameterSetId_; }
  uint8_t        getLog2PatchPackingBlockSize() { return log2PatchPackingBlockSize_; }
  uint8_t        getLog2MaxPatchFrameOrderCntLsbMinus4() { return log2MaxPatchFrameOrderCntLsb_; }
  uint8_t        getMaxDecPatchFrameBufferingMinus1() { return maxDecPatchFrameBufferingMinus1_; }
  uint8_t        getNumRefPatchFrameListsInPsps() { return numRefPatchFrameListsInPsps_; }
  bool           getLongTermRefPatchFramesFlag() { return longTermRefPatchFramesFlag_; }
  bool           getUseEightOrientationsFlag() { return useEightOrientationsFlag_; }
  bool           getNormalAxisLimitsQuantizationEnableFlag() { return normalAxisLimitsQuantizationEnableFlag_; }
  bool           getNormalAxisMaxDeltaValueEnableFlag() { return normalAxisMaxDeltaValueEnableFlag_; }
  RefListStruct& getRefListStruct( uint8_t index ) { return refListStruct_[index]; }
  uint8_t        getRefListStructSize() { return refListStruct_.size(); }

  void setPatchSequenceParameterSetId( uint8_t value ) { patchSequenceParameterSetId_ = value; }
  void setLog2PatchPackingBlockSize( uint8_t value ) { log2PatchPackingBlockSize_ = value; }
  void setLog2MaxPatchFrameOrderCntLsbMinus4( uint8_t value ) { log2MaxPatchFrameOrderCntLsb_ = value; }
  void setMaxDecPatchFrameBufferingMinus1( uint8_t value ) { maxDecPatchFrameBufferingMinus1_ = value; }
  void setNumRefPatchFrameListsInPsps( uint8_t value ) { numRefPatchFrameListsInPsps_ = value; }
  void setLongTermRefPatchFramesFlag( bool value ) { longTermRefPatchFramesFlag_ = value; }
  void setUseEightOrientationsFlag( bool value ) { useEightOrientationsFlag_ = value; }
  void setNormalAxisLimitsQuantizationEnableFlag( bool value ) { normalAxisLimitsQuantizationEnableFlag_ = value; }
  void setNormalAxisMaxDeltaValueEnableFlag( bool value ) { normalAxisMaxDeltaValueEnableFlag_ = value; }
  void setRefListStruct( uint8_t index, RefListStruct value ) { refListStruct_[index] = value; }
  void addRefListStruct( RefListStruct value ) { refListStruct_.push_back( value ); }
  RefListStruct& addRefListStruct() {
    RefListStruct refListStruct;
    refListStruct_.push_back( refListStruct );
    return refListStruct_.back();
  }

 private:
  uint8_t patchSequenceParameterSetId_;
  uint8_t log2PatchPackingBlockSize_;
  uint8_t log2MaxPatchFrameOrderCntLsb_;
  uint8_t maxDecPatchFrameBufferingMinus1_;
  uint8_t numRefPatchFrameListsInPsps_;
  bool    longTermRefPatchFramesFlag_;
  bool    useEightOrientationsFlag_;
  bool    normalAxisLimitsQuantizationEnableFlag_;
  bool    normalAxisMaxDeltaValueEnableFlag_;

  std::vector<RefListStruct> refListStruct_;
};

class SeiPayload {
 public:
  SeiPayload() {}
  ~SeiPayload() {}
  SeiPayload& operator=( const SeiPayload& ) = default;
};

class SeiMessage {
 public:
  SeiMessage() : payloadTypeByte_( 0 ), payloadSizeByte_( 0 ) { seiPayload_.clear(); }
  ~SeiMessage() { seiPayload_.clear(); }
  SeiMessage& operator=( const SeiMessage& ) = default;
  uint8_t     getPayloadTypeByte() { return payloadTypeByte_; }
  uint8_t     getPayloadSizeByte() { return payloadSizeByte_; }
  void        setPayloadTypeByte( uint8_t value ) { payloadTypeByte_ = value; }
  void        setPayloadSizeByte( uint8_t value ) { payloadSizeByte_ = value; }

 private:
  uint8_t                 payloadTypeByte_;
  uint8_t                 payloadSizeByte_;
  std::vector<SeiPayload> seiPayload_;
};

// 7.3.5.2  Patch data group unit payload syntax (pdgup)
class PatchDataGroup {
 public:
  PatchDataGroup() {}
  ~PatchDataGroup() {
    patchTileGroupLayerUnit_.clear();
    seiMessageSuffix_.clear();
    seiMessagePrefix_.clear();
  }
  PatchDataGroup&             operator=( const PatchDataGroup& ) = default;
  PatchSequenceParameterSet&  getPatchSequenceParameterSet( size_t index ) { return patchSequenceParameterSet_[index]; }
  GeometryPatchParameterSet&  getGeometryPatchParameterSet( size_t index ) { return geometryPatchParameterSet_[index]; }
  AttributePatchParameterSet& getAttributePatchParameterSet( size_t index ) {
    return attributePatchParameterSet_[index];
  }
  PatchFrameParameterSet&          getPatchFrameParameterSet( size_t index ) { return patchFrameParameterSet_[index]; }
  PatchFrameAttributeParameterSet& getPatchFrameAttributeParameterSet( size_t index ) {
    return patchFrameAttributeParameterSet_[index];
  }
  PatchFrameGeometryParameterSet& getPatchFrameGeometryParameterSet( size_t index ) {
    return patchFrameGeometryParameterSet_[index];
  }
  PatchTileGroupLayerUnit& getPatchTileGroupLayerUnit( size_t index ) { return patchTileGroupLayerUnit_[index]; }
  std::vector<SeiMessage>& getSeiMessagePrefix() { return seiMessagePrefix_; }
  std::vector<SeiMessage>& getSeiMessageSuffix() { return seiMessageSuffix_; }
  SeiMessage&              getSeiMessagePrefix( size_t index ) { return seiMessagePrefix_[index]; }
  SeiMessage&              getSeiMessageSuffix( size_t index ) { return seiMessageSuffix_[index]; }
  size_t                   getPatchTileGroupLayerUnitSize() { return patchTileGroupLayerUnit_.size(); }
  size_t                   getSeiMessagePrefixSize() { return seiMessagePrefix_.size(); }
  size_t                   getSeiMessageSuffixSize() { return seiMessageSuffix_.size(); }
  size_t                   getPatchSequenceParameterSetSize() { return patchSequenceParameterSetSize_; }
  size_t                   getGeometryPatchParameterSetSize() { return geometryPatchParameterSetSize_; }
  size_t                   getAttributePatchParameterSetSize() { return attributePatchParameterSetSize_; }
  size_t                   getPatchFrameParameterSetSize() { return patchFrameParameterSetSize_; }
  size_t                   getPatchFrameAttributeParameterSetSize() { return patchFrameAttributeParameterSetSize_; }
  size_t                   getPatchFrameGeometryParameterSetSize() { return patchFrameGeometryParameterSetSize_; }
  void                     setPatchSequenceParameterSetSize( size_t value ) { patchSequenceParameterSetSize_ = value; }
  void                     setGeometryPatchParameterSetSize( size_t value ) { geometryPatchParameterSetSize_ = value; }
  void setAttributePatchParameterSetSize( size_t value ) { attributePatchParameterSetSize_ = value; }
  void setPatchFrameParameterSetSize( size_t value ) { patchFrameParameterSetSize_ = value; }
  void setPatchFrameAttributeParameterSetSize( size_t value ) { patchFrameAttributeParameterSetSize_ = value; }
  void setPatchFrameGeometryParameterSetSize( size_t value ) { patchFrameGeometryParameterSetSize_ = value; }
  void setPatchSequenceParameterSet( size_t index, PatchSequenceParameterSet value ) {
    patchSequenceParameterSet_[index] = value;
  }
  void setGeometryPatchParameterSet( size_t index, GeometryPatchParameterSet value ) {
    geometryPatchParameterSet_[index] = value;
  }
  void setAttributePatchParameterSet( size_t index, AttributePatchParameterSet& value ) {
    attributePatchParameterSet_[index] = value;
  }
  void setPatchFrameParameterSet( size_t index, PatchFrameParameterSet value ) {
    patchFrameParameterSet_[index] = value;
  }
  void setPatchFrameAttributeParameterSet( size_t index, PatchFrameAttributeParameterSet& value ) {
    patchFrameAttributeParameterSet_[index] = value;
  }
  void setPatchFrameGeometryParameterSet( size_t index, PatchFrameGeometryParameterSet value ) {
    patchFrameGeometryParameterSet_[index] = value;
  }
  PatchTileGroupLayerUnit& addPatchTileGroupLayerUnit() {
    PatchTileGroupLayerUnit patchTileGroupLayerUnit;
    patchTileGroupLayerUnit_.push_back( patchTileGroupLayerUnit );
    return patchTileGroupLayerUnit_.back();
  }
  SeiMessage& addSeiMessagePrefix() {
    SeiMessage seiMessage;
    seiMessagePrefix_.push_back( seiMessage );
    return seiMessagePrefix_.back();
  }
  SeiMessage& addSeiMessageSuffix() {
    SeiMessage seiMessage;
    seiMessageSuffix_.push_back( seiMessage );
    return seiMessageSuffix_.back();
  }

 private:
  PatchSequenceParameterSet            patchSequenceParameterSet_[16];
  GeometryPatchParameterSet            geometryPatchParameterSet_[64];
  AttributePatchParameterSet           attributePatchParameterSet_[64];
  PatchFrameParameterSet               patchFrameParameterSet_[64];
  PatchFrameAttributeParameterSet      patchFrameAttributeParameterSet_[64];
  PatchFrameGeometryParameterSet       patchFrameGeometryParameterSet_[64];
  std::vector<PatchTileGroupLayerUnit> patchTileGroupLayerUnit_;
  std::vector<SeiMessage>              seiMessagePrefix_;
  std::vector<SeiMessage>              seiMessageSuffix_;
  size_t                               patchSequenceParameterSetSize_;
  size_t                               geometryPatchParameterSetSize_;
  size_t                               attributePatchParameterSetSize_;
  size_t                               patchFrameParameterSetSize_;
  size_t                               patchFrameAttributeParameterSetSize_;
  size_t                               patchFrameGeometryParameterSetSize_;
};

class AttributeInformation {
 public:
  AttributeInformation() :
      attributeCount_( 0 ),
      attributeParamsEnabledFlag_( false ),
      attributePatchParamsEnabledFlag_( false ),
      attributeMSBAlignFlag_( false ) {}
  ~AttributeInformation() {
    attributeTypeId_.clear();
    attributeCodecId_.clear();
    pcmAttributeCodecId_.clear();
    attributeDimensionMinus1_.clear();
    attributeDimensionPartitionsMinus1_.clear();
    attributeNominal2dBitdepthMinus1_.clear();
    for ( auto& value : attributePartitionChannelsMinus1_ ) { value.clear(); }
    attributePartitionChannelsMinus1_.clear();
  }
  AttributeInformation& operator=( const AttributeInformation& ) = default;

  void allocate() {
    attributeTypeId_.resize( attributeCount_, 0 );
    attributeCodecId_.resize( attributeCount_, 0 );
    pcmAttributeCodecId_.resize( attributeCount_, 0 );
    attributeDimensionMinus1_.resize( attributeCount_, 0 );
    attributeDimensionPartitionsMinus1_.resize( attributeCount_, 0 );
    attributeNominal2dBitdepthMinus1_.resize( attributeCount_, 0 );
    attributePartitionChannelsMinus1_.resize( attributeCount_ );
  }
  uint8_t getAttributeCount() { return attributeCount_; }
  uint8_t getAttributeParamsEnabledFlag() { return attributeParamsEnabledFlag_; }
  uint8_t getAttributePatchParamsEnabledFlag() { return attributePatchParamsEnabledFlag_; }
  uint8_t getAttributeMSBAlignFlag() { return attributeMSBAlignFlag_; }
  uint8_t getAttributeTypeId( uint32_t index ) { return attributeTypeId_[index]; }
  uint8_t getAttributeCodecId( uint32_t index ) { return attributeCodecId_[index]; };
  uint8_t getPcmAttributeCodecId( uint32_t index ) { return pcmAttributeCodecId_[index]; }
  uint8_t getAttributeDimensionMinus1( uint32_t index ) { return attributeDimensionMinus1_[index]; }
  uint8_t getAttributeDimensionPartitionsMinus1( uint32_t index ) { return attributeDimensionPartitionsMinus1_[index]; }
  uint8_t getAttributeNominal2dBitdepthMinus1( uint32_t index ) { return attributeNominal2dBitdepthMinus1_[index]; }
  uint8_t getAttributePartitionChannelsMinus1( uint32_t index, uint32_t j ) {
    if ( j >= attributePartitionChannelsMinus1_[index].size() ) {
      attributePartitionChannelsMinus1_[index].resize( j + 1, 0 );
    }
    return attributePartitionChannelsMinus1_[index][j];
  }
  void setAttributeCount( uint8_t value ) { attributeCount_ = value; }
  void setAttributeParamsEnabledFlag( bool value ) { attributeParamsEnabledFlag_ = value; }
  void setAttributePatchParamsEnabledFlag( bool value ) { attributePatchParamsEnabledFlag_ = value; }
  void setAttributeMSBAlignFlag( bool value ) { attributeMSBAlignFlag_ = value; }
  void setAttributeTypeId( uint32_t index, uint8_t value ) { attributeTypeId_[index] = value; }
  void setAttributeCodecId( uint32_t index, uint8_t value ) { attributeCodecId_[index] = value; };
  void setPcmAttributeCodecId( uint32_t index, uint8_t value ) { pcmAttributeCodecId_[index] = value; }
  void setAttributeDimensionMinus1( uint32_t index, uint8_t value ) { attributeDimensionMinus1_[index] = value; }
  void setAttributeDimensionPartitionsMinus1( uint32_t index, uint8_t value ) {
    attributeDimensionPartitionsMinus1_[index] = value;
  }
  void setAttributeNominal2dBitdepthMinus1( uint32_t index, uint8_t value ) {
    attributeNominal2dBitdepthMinus1_[index] = value;
  }
  void setAttributePartitionChannelsMinus1( uint32_t index, uint32_t j, uint8_t value ) {
    if ( j >= attributePartitionChannelsMinus1_[index].size() ) {
      attributePartitionChannelsMinus1_[index].resize( j + 1, 0 );
    }
    attributePartitionChannelsMinus1_[index][j] = value;
  }

 private:
  uint8_t                           attributeCount_;
  bool                              attributeParamsEnabledFlag_;
  bool                              attributePatchParamsEnabledFlag_;
  bool                              attributeMSBAlignFlag_;
  std::vector<uint8_t>              attributeTypeId_;
  std::vector<uint8_t>              attributeCodecId_;
  std::vector<uint8_t>              pcmAttributeCodecId_;
  std::vector<uint8_t>              attributeDimensionMinus1_;
  std::vector<uint8_t>              attributeDimensionPartitionsMinus1_;
  std::vector<uint8_t>              attributeNominal2dBitdepthMinus1_;
  std::vector<std::vector<uint8_t>> attributePartitionChannelsMinus1_;
};

// 7.3.4.4 Geometry information Syntax
class GeometryInformation {
 public:
  GeometryInformation() :
      geometryCodecId_( 0 ),
      geometryNominal2dBitdepthMinus1_( 10 ),
      geometry3dCoordinatesBitdepthMinus1_( 9 ),
      pcmGeometryCodecId_( 0 ),
      geometryParamsEnabledFlag_( false ) {}
  ~GeometryInformation() {}
  GeometryInformation& operator=( const GeometryInformation& ) = default;

  void init( uint8_t codecId,
             uint8_t nominal2dBitdepthMinus1,
             uint8_t coordinatesBitdepthMinus1,
             uint8_t pcmGeometryCodecId,
             bool    paramsEnabledFlag,
             bool    patchParamsEnabledFlag,
             bool    patchScaleParamsEnabledFlag,
             bool    patchOffsetParamsEnabledFlag,
             bool    patchRotationParamsEnabledFlag,
             bool    patchPointSizeInfoEnabledFlag,
             bool    patchPointShapeInfoEnabledFlag ) {
    geometryCodecId_                     = codecId;
    geometryNominal2dBitdepthMinus1_     = nominal2dBitdepthMinus1;
    geometry3dCoordinatesBitdepthMinus1_ = coordinatesBitdepthMinus1;
    pcmGeometryCodecId_                  = pcmGeometryCodecId;
    geometryParamsEnabledFlag_           = paramsEnabledFlag;
  }
  uint8_t getGeometryCodecId() { return geometryCodecId_; }
  uint8_t getGeometryNominal2dBitdepthMinus1() { return geometryNominal2dBitdepthMinus1_; }
  uint8_t getGeometry3dCoordinatesBitdepthMinus1() { return geometry3dCoordinatesBitdepthMinus1_; }
  uint8_t getPcmGeometryCodecId() { return pcmGeometryCodecId_; }
  bool    getGeometryParamsEnabledFlag() { return geometryParamsEnabledFlag_; }
  bool    getGeometryPatchParamsEnabledFlag() { return geometryPatchParamsEnabledFlag_; }
  void    setGeometryCodecId( uint8_t value ) { geometryCodecId_ = value; }
  void    setGeometryNominal2dBitdepthMinus1( uint8_t value ) { geometryNominal2dBitdepthMinus1_ = value; }
  void    setGeometry3dCoordinatesBitdepthMinus1( uint8_t value ) { geometry3dCoordinatesBitdepthMinus1_ = value; }
  void    setPcmGeometryCodecId( uint8_t value ) { pcmGeometryCodecId_ = value; }
  void    setGeometryParamsEnabledFlag( bool value ) { geometryParamsEnabledFlag_ = value; }
  void    setGeometryPatchParamsEnabledFlag( bool value ) { geometryPatchParamsEnabledFlag_ = value; }

 private:
  uint8_t geometryCodecId_;
  uint8_t geometryNominal2dBitdepthMinus1_;
  uint8_t geometry3dCoordinatesBitdepthMinus1_;
  uint8_t pcmGeometryCodecId_;
  bool    geometryParamsEnabledFlag_;
  bool    geometryPatchParamsEnabledFlag_;
};

// 7.3.4.3 Occupancy information Set Syntax
class OccupancyInformation {
 public:
  OccupancyInformation() : occupancyCodecId_( 0 ), lossyOccupancyMapCompressionThreshold_( 0 ) {}
  ~OccupancyInformation() {}
  OccupancyInformation& operator=( const OccupancyInformation& ) = default;
  void                  init( uint8_t codecId, uint8_t threshold, uint8_t packingBlockSize ) {
    occupancyCodecId_                      = codecId;
    lossyOccupancyMapCompressionThreshold_ = threshold;
  }
  uint8_t getOccupancyCodecId() { return occupancyCodecId_; }
  uint8_t getLossyOccupancyMapCompressionThreshold() { return lossyOccupancyMapCompressionThreshold_; }
  void    setOccupancyCodecId( uint8_t value ) { occupancyCodecId_ = value; }
  void    setLossyOccupancyMapCompressionThreshold( uint8_t value ) { lossyOccupancyMapCompressionThreshold_ = value; }

 private:
  uint8_t occupancyCodecId_;
  uint8_t lossyOccupancyMapCompressionThreshold_;
};

// 7.3.4.2 Profile, Tier and Level Syntax
class ProfileTierLevel {
 public:
  ProfileTierLevel() :
      tierFlag_( false ),
      profileCodecGroupIdc_( 0 ),
      profilePccToolsetIdc_( 0 ),
      profileReconctructionIdc_( 0 ),
      levelIdc_( 0 ) {}
  ~ProfileTierLevel() {}
  ProfileTierLevel& operator=( const ProfileTierLevel& ) = default;
  bool              getTierFlag() { return tierFlag_; }
  uint8_t           getProfileCodecGroupIdc() { return profileCodecGroupIdc_; }
  uint8_t           getProfilePccToolsetIdc() { return profilePccToolsetIdc_; }
  uint8_t           getProfileReconctructionIdc() { return profileReconctructionIdc_; }
  uint8_t           getLevelIdc() { return levelIdc_; }
  void              setTierFlag( bool value ) { tierFlag_ = value; }
  void              setProfileCodecGroupIdc( uint8_t value ) { profileCodecGroupIdc_ = value; }
  void              setProfilePccToolsetIdc( uint8_t value ) { profilePccToolsetIdc_ = value; }
  void              setProfileReconctructionIdc( uint8_t value ) { profileReconctructionIdc_ = value; }
  void              setLevelIdc( uint8_t value ) { levelIdc_ = value; }

 private:
  bool    tierFlag_;
  uint8_t profileCodecGroupIdc_;
  uint8_t profilePccToolsetIdc_;
  uint8_t profileReconctructionIdc_;
  uint8_t levelIdc_;
};

// 7.3.4.1  General Sequence parameter set syntax
class SequenceParameterSet {
 public:
  SequenceParameterSet() :
      sequenceParameterSetId_( 0 ),
      frameWidth_( 0 ),
      frameHeight_( 0 ),
      avgFrameRate_( 0 ),
      layerCountMinus1_( 0 ),
      avgFrameRatePresentFlag_( false ),
      enhancedOccupancyMapForDepthFlag_( false ),
      multipleLayerStreamsPresentFlag_( false ),
      pcmPatchEnabledFlag_( false ),
      pcmSeparateVideoPresentFlag_( false ),
      patchInterPredictionEnabledFlag_( false ),
      pixelDeinterleavingFlag_( false ),
      pointLocalReconstructionEnabledFlag_( false ),
      removeDuplicatePointEnabledFlag_( false ),
      projection45degreeEnabledFlag_( false ),
      patchPrecedenceOrderFlag_( false ) {
    layerAbsoluteCodingEnabledFlag_.clear();
    layerPredictorIndexDiff_.clear();
  }
  ~SequenceParameterSet() {
    layerAbsoluteCodingEnabledFlag_.clear();
    layerPredictorIndexDiff_.clear();
  }

  SequenceParameterSet& operator=( const SequenceParameterSet& ) = default;
  void                  init( uint32_t sequenceParameterSetId,
                              uint16_t frameWidth,
                              uint16_t frameHeight,
                              uint16_t avgFrameRate,
                              uint32_t layerCountMinus1,
                              uint16_t attributeCount,
                              bool     avgFrameRatePresentFlag,
                              bool     enhancedOccupancyMapForDepthFlag,
                              bool     multipleLayerStreamsPresentFlag,
                              bool     pcmPatchEnabledFlag,
                              bool     pcmSeparateVideoPresentFlag,
                              bool     patchSequenceOrientationEnabledFlag,
                              bool     patchInterPredictionEnabledFlag,
                              bool     pixelDeinterleavingFlag,
                              bool     pointLocalReconstructionEnabledFlag,
                              bool     removeDuplicatePointEnabledFlag,
                              bool     projection45degreeEnabledFlag,
                              bool     patchPrecedenceOrderFlag ) {
    sequenceParameterSetId_              = sequenceParameterSetId;
    frameWidth_                          = frameWidth;
    frameHeight_                         = frameHeight;
    avgFrameRate_                        = avgFrameRate;
    layerCountMinus1_                    = layerCountMinus1;
    avgFrameRatePresentFlag_             = avgFrameRatePresentFlag;
    enhancedOccupancyMapForDepthFlag_    = enhancedOccupancyMapForDepthFlag;
    multipleLayerStreamsPresentFlag_     = multipleLayerStreamsPresentFlag;
    pcmPatchEnabledFlag_                 = pcmPatchEnabledFlag;
    pcmSeparateVideoPresentFlag_         = pcmSeparateVideoPresentFlag;
    patchInterPredictionEnabledFlag_     = patchInterPredictionEnabledFlag;
    pixelDeinterleavingFlag_             = pixelDeinterleavingFlag;
    pointLocalReconstructionEnabledFlag_ = pointLocalReconstructionEnabledFlag;
    removeDuplicatePointEnabledFlag_     = removeDuplicatePointEnabledFlag;
    projection45degreeEnabledFlag_       = projection45degreeEnabledFlag;
    patchPrecedenceOrderFlag_            = patchPrecedenceOrderFlag;
    allocate();
  }

  void allocate() {
    layerAbsoluteCodingEnabledFlag_.resize( layerCountMinus1_ + 1 );
    layerPredictorIndexDiff_.resize( layerCountMinus1_ + 1 );
  }
  uint32_t          getSequenceParameterSetId() { return sequenceParameterSetId_; }
  uint16_t          getFrameWidth() { return frameWidth_; }
  uint16_t          getFrameHeight() { return frameHeight_; }
  uint16_t          getAvgFrameRate() { return avgFrameRate_; }
  uint32_t          getLayerCountMinus1() { return layerCountMinus1_; }
  bool              getAvgFrameRatePresentFlag() { return avgFrameRatePresentFlag_; }
  bool              getEnhancedOccupancyMapForDepthFlag() { return enhancedOccupancyMapForDepthFlag_; }
  bool              getMultipleLayerStreamsPresentFlag() { return multipleLayerStreamsPresentFlag_; }
  bool              getPcmPatchEnabledFlag() { return pcmPatchEnabledFlag_; }
  bool              getPcmSeparateVideoPresentFlag() { return pcmSeparateVideoPresentFlag_; }
  bool              getPatchInterPredictionEnabledFlag() { return patchInterPredictionEnabledFlag_; }
  bool              getPixelDeinterleavingFlag() { return pixelDeinterleavingFlag_; }
  bool              getPointLocalReconstructionEnabledFlag() { return pointLocalReconstructionEnabledFlag_; }
  bool              getRemoveDuplicatePointEnabledFlag() { return removeDuplicatePointEnabledFlag_; }
  bool              getProjection45DegreeEnableFlag() { return projection45degreeEnabledFlag_; }
  bool              getPatchPrecedenceOrderFlag() { return patchPrecedenceOrderFlag_; }
  size_t            getLayerPredictorIndexDiff( size_t index ) { return layerPredictorIndexDiff_[index]; }
  bool              getLayerAbsoluteCodingEnabledFlag( size_t index ) { return layerAbsoluteCodingEnabledFlag_[index]; }
  ProfileTierLevel& getProfileTierLevel() { return profileTierLevel_; }
  GeometryInformation&                 getGeometryInformation() { return geometryInformation_; }
  OccupancyInformation&                getOccupancyInformation() { return occupancyInformation_; }
  AttributeInformation&                getAttributeInformation() { return attributeInformation_; }
  PointLocalReconstructionInformation& getPointLocalReconstructionInformation() {
    return pointLocalReconstructionInformation_;
  }

  void setSequenceParameterSetId( uint32_t value ) { sequenceParameterSetId_ = value; }
  void setFrameWidth( uint16_t value ) { frameWidth_ = value; }
  void setFrameHeight( uint16_t value ) { frameHeight_ = value; }
  void setAvgFrameRate( uint16_t value ) { avgFrameRate_ = value; }
  void setLayerCountMinus1( uint32_t value ) { layerCountMinus1_ = value; }
  void setAvgFrameRatePresentFlag( bool value ) { avgFrameRatePresentFlag_ = value; }
  void setEnhancedOccupancyMapForDepthFlag( bool value ) { enhancedOccupancyMapForDepthFlag_ = value; }
  void setMultipleLayerStreamsPresentFlag( bool value ) { multipleLayerStreamsPresentFlag_ = value; }
  void setPcmPatchEnabledFlag( bool value ) { pcmPatchEnabledFlag_ = value; }
  void setPcmSeparateVideoPresentFlag( bool value ) { pcmSeparateVideoPresentFlag_ = value; }
  void setPatchInterPredictionEnabledFlag( bool value ) { patchInterPredictionEnabledFlag_ = value; }
  void setPixelDeinterleavingFlag( bool value ) { pixelDeinterleavingFlag_ = value; }
  void setPointLocalReconstructionEnabledFlag( bool value ) { pointLocalReconstructionEnabledFlag_ = value; }
  void setRemoveDuplicatePointEnabledFlag( bool value ) { removeDuplicatePointEnabledFlag_ = value; }
  void setProjection45DegreeEnableFlag( bool value ) { projection45degreeEnabledFlag_ = value; }
  void setPatchPrecedenceOrderFlag( bool value ) { patchPrecedenceOrderFlag_ = value; }
  void setProfileTierLevel( ProfileTierLevel value ) { profileTierLevel_ = value; }
  void setGeometryInformation( GeometryInformation value ) { geometryInformation_ = value; }
  void setOccupancyInformation( OccupancyInformation value ) { occupancyInformation_ = value; }
  void setLayerAbsoluteCodingEnabledFlag( size_t index, bool value ) {
    if ( layerAbsoluteCodingEnabledFlag_.size() < index + 1 ) {
      layerAbsoluteCodingEnabledFlag_.resize( index + 1, 0 );
    }
    layerAbsoluteCodingEnabledFlag_[index] = value;
  }
  void setLayerPredictorIndexDiff( size_t index, bool value ) {
    if ( layerPredictorIndexDiff_.size() < index + 1 ) { layerPredictorIndexDiff_.resize( index + 1 ); }
    layerPredictorIndexDiff_[index] = value;
  }

  void addLayerAbsoluteCodingEnabledFlag( bool value ) { layerAbsoluteCodingEnabledFlag_.push_back( value ); }
  void addLayerPredictorIndexDiff( bool value ) { layerPredictorIndexDiff_.push_back( value ); }

 private:
  uint32_t                            sequenceParameterSetId_;
  uint16_t                            frameWidth_;
  uint16_t                            frameHeight_;
  uint16_t                            avgFrameRate_;
  uint32_t                            layerCountMinus1_;
  bool                                avgFrameRatePresentFlag_;
  bool                                enhancedOccupancyMapForDepthFlag_;
  bool                                multipleLayerStreamsPresentFlag_;
  bool                                pcmPatchEnabledFlag_;
  bool                                pcmSeparateVideoPresentFlag_;
  bool                                patchInterPredictionEnabledFlag_;
  bool                                pixelDeinterleavingFlag_;
  bool                                pointLocalReconstructionEnabledFlag_;
  bool                                removeDuplicatePointEnabledFlag_;
  bool                                projection45degreeEnabledFlag_;
  bool                                patchPrecedenceOrderFlag_;
  std::vector<bool>                   layerAbsoluteCodingEnabledFlag_;
  std::vector<size_t>                 layerPredictorIndexDiff_;
  ProfileTierLevel                    profileTierLevel_;
  GeometryInformation                 geometryInformation_;
  OccupancyInformation                occupancyInformation_;
  AttributeInformation                attributeInformation_;
  PointLocalReconstructionInformation pointLocalReconstructionInformation_;

  // THE NEXT PARAMETERS ARE NOT IN THE VPCC CD SYNTAX DOCUMENTS AND WILL BE REMOVE
 public:
  bool    getLosslessGeo444() { return losslessGeo444_; }
  bool    getLosslessGeo() { return losslessGeo_; }
  bool    getLosslessTexture() { return losslessTexture_; }
  uint8_t getMinLevel() { return minLevel_; }
  size_t  getSurfaceThickness() { return surfaceThickness_; }
  void    setLosslessGeo444( bool losslessGeo444 ) { losslessGeo444_ = losslessGeo444; }
  void    setLosslessGeo( bool losslessGeo ) { losslessGeo_ = losslessGeo; }
  void    setLosslessTexture( bool losslessTexture ) { losslessTexture_ = losslessTexture; }
  void    setMinLevel( uint8_t minLevel ) { minLevel_ = minLevel; }
  void    setSurfaceThickness( size_t surfaceThickness ) { surfaceThickness_ = surfaceThickness; }

 private:
  bool    losslessGeo444_;
  bool    losslessGeo_;
  bool    losslessTexture_;
  size_t  surfaceThickness_;
  uint8_t minLevel_;
  // THE NEXT PARAMETERS ARE NOT IN THE VPCC CD SYNTAX DOCUMENTS AND WILL BE REMOVE
};

// VPCCParameterSet Class
class VPCCParameterSet {
 public:
  VPCCParameterSet() :
      unitType_( 0 ),
      sequenceParamterSetId_( 0 ),
      attributeIndex_( 0 ),
      attributeDimensionIndex_( 0 ),
      layerIndex_( 0 ),
      pcmVideoFlag_( false ) {}

  ~VPCCParameterSet() {}
  VPCCParameterSet& operator=( const VPCCParameterSet& ) = default;
  uint8_t           getUnitType() { return unitType_; }
  uint8_t           getSequenceParameterSetId() { return sequenceParamterSetId_; }
  uint8_t           getAttributeIndex() { return attributeIndex_; }
  uint8_t           getAttributeDimensionIndex() { return attributeDimensionIndex_; }
  uint8_t           getLayerIndex() { return layerIndex_; }
  bool              getPCMVideoFlag() { return pcmVideoFlag_; }
  void              setUnitType( uint8_t value ) { unitType_ = value; }
  void              setSequenceParameterSetId( uint8_t value ) { sequenceParamterSetId_ = value; }
  void              setAttributeIndex( uint8_t value ) { attributeIndex_ = value; }
  void              setAttributeDimensionIndex( uint8_t value ) { attributeDimensionIndex_ = value; }
  void              setLayerIndex( uint8_t value ) { layerIndex_ = value; }
  void              setPCMVideoFlag( bool value ) { pcmVideoFlag_ = value; }

 private:
  uint8_t unitType_;
  uint8_t sequenceParamterSetId_;
  uint8_t attributeIndex_;
  uint8_t attributeDimensionIndex_;
  uint8_t layerIndex_;
  bool    pcmVideoFlag_;
};

typedef struct PointLocalReconstructionMode {
  bool    interpolate_;
  bool    filling_;
  uint8_t minD1_;
  uint8_t neighbor_;
} PointLocalReconstructionMode;

class PCCContext {
 public:
  PCCContext();
  ~PCCContext();

  std::vector<PCCFrameContext>::iterator begin() { return frames_.begin(); }
  std::vector<PCCFrameContext>::iterator end() { return frames_.end(); }

  void resize( size_t size );

  const size_t                  size() { return frames_.size(); }
  std::vector<PCCFrameContext>& getFrames() { return frames_; }
  PCCFrameContext&              getFrame( int16_t index ) { return frames_[index]; }
  PCCFrameContext&              operator[]( int index ) { return frames_[index]; }
  PCCVideoGeometry&             getVideoGeometry() { return videoGeometry_; }
  PCCVideoGeometry&             getVideoGeometryD1() { return videoGeometryD1_; }
  PCCVideoTexture&              getVideoTexture() { return videoTexture_; }
  PCCVideoOccupancyMap&         getVideoOccupancyMap() { return videoOccupancyMap_; }
  PCCVideoGeometry&             getVideoMPsGeometry() { return videoMPsGeometry_; }
  PCCVideoTexture&              getVideoMPsTexture() { return videoMPsTexture_; }
  uint8_t                       getOccupancyPrecision() { return occupancyPrecision_; }
  uint8_t                       getOccupancyPackingBlockSize() { return occupancyPackingBlockSize_; }
  float                         getModelScale() { return modelScale_; }
  PCCVector3<float>&            getModelOrigin() { return modelOrigin_; }
  size_t                        getMPGeoWidth() { return MPGeoWidth_; }
  size_t                        getMPGeoHeight() { return MPGeoHeight_; }
  size_t                        getMPAttWidth() { return MPAttWidth_; }
  size_t                        getMPAttHeight() { return MPAttHeight_; }
  PCCMetadata&                  getGOFLevelMetadata() { return gofLevelMetadata_; }
  std::vector<SubContext>&      getSubContexts() { return subContexts_; }
  std::vector<unionPatch>&      getUnionPatch() { return unionPatch_; }
  bool&                         getPrefilterLossyOM() { return prefilterLossyOM_; }
  size_t&                       getOffsetLossyOM() { return offsetLossyOM_; }
  size_t                        getGeometry3dCoordinatesBitdepth() { return geometry3dCoordinatesBitdepth_; }
  void                          setOccupancyPrecision( uint8_t value ) { occupancyPrecision_ = value; }
  void                          setOccupancyPackingBlockSize( uint8_t value ) { occupancyPackingBlockSize_ = value; }
  void                          setModelScale( float value ) { modelScale_ = value; }
  void                          setModelOrigin( PCCVector3<float>& value ) { modelOrigin_ = value; }
  void                          setMPGeoWidth( size_t value ) { MPGeoWidth_ = value; }
  void                          setMPGeoHeight( size_t value ) { MPGeoHeight_ = value; }
  void                          setMPAttWidth( size_t value ) { MPAttWidth_ = value; }
  void                          setMPAttHeight( size_t value ) { MPAttHeight_ = value; }
  void                          setPrefilterLossyOM( bool value ) { prefilterLossyOM_ = value; }
  void                          setOffsetLossyOM( size_t value ) { offsetLossyOM_ = value; }
  void setGeometry3dCoordinatesBitdepth( size_t value ) { geometry3dCoordinatesBitdepth_ = value; }

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
    exit( -1 );
  }

  void                  allocOneLayerData();
  void                  printVideoBitstream();
  void                  printBlockToPatch( const size_t occupancyResolution );
  VPCCParameterSet&     getVPCC() { return vpccParameterSet_; }
  SequenceParameterSet& getSps() {
    for ( auto& sps : sequenceParameterSets_ ) {
      if ( vpccParameterSet_.getSequenceParameterSetId() == sps.getSequenceParameterSetId() ) { return sps; }
    }
    fprintf( stderr, "Error: can't find sps[ %lc ] \n", vpccParameterSet_.getSequenceParameterSetId() );
    exit( -1 );
  }

  SequenceParameterSet& addSequenceParameterSet( uint8_t index ) {
    SequenceParameterSet sps;
    sps.setSequenceParameterSetId( index );
    sequenceParameterSets_.push_back( sps );
    return sequenceParameterSets_.back();
  }

  PatchDataGroup& getPatchDataGroup() { return patchSequenceDataUnit_; }

  void addPointLocalReconstructionMode( const PointLocalReconstructionMode& mode ) {
    pointLocalReconstructionMode_.push_back( mode );
  }
  PointLocalReconstructionMode& getPointLocalReconstructionMode( size_t index ) {
    return pointLocalReconstructionMode_[index];
  }
  size_t getPointLocalReconstructionModeNumber() { return pointLocalReconstructionMode_.size(); }

 private:
  std::vector<PCCFrameContext>              frames_;
  PCCVideoGeometry                          videoGeometry_;
  PCCVideoGeometry                          videoGeometryD1_;
  PCCVideoTexture                           videoTexture_;
  PCCVideoOccupancyMap                      videoOccupancyMap_;
  PCCVideoGeometry                          videoMPsGeometry_;
  PCCVideoTexture                           videoMPsTexture_;
  std::vector<PCCVideoBitstream>            videoBitstream_;
  PCCMetadata                               gofLevelMetadata_;
  VPCCParameterSet                          vpccParameterSet_;
  PatchDataGroup                            patchSequenceDataUnit_;
  std::vector<SequenceParameterSet>         sequenceParameterSets_;
  uint8_t                                   occupancyPrecision_;
  uint8_t                                   occupancyPackingBlockSize_;
  size_t                                    MPGeoWidth_;
  size_t                                    MPGeoHeight_;
  size_t                                    MPAttWidth_;
  size_t                                    MPAttHeight_;
  float                                     modelScale_;
  PCCVector3<float>                         modelOrigin_;
  std::vector<SubContext>                   subContexts_;
  std::vector<unionPatch>                   unionPatch_;
  std::vector<PointLocalReconstructionMode> pointLocalReconstructionMode_;
  bool                                      prefilterLossyOM_;
  size_t                                    offsetLossyOM_;
  size_t                                    geometry3dCoordinatesBitdepth_;
};
};  // namespace pcc

#endif /* PCCContext_h */
