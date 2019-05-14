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
typedef std::map<size_t, PCCPatch>                  unionPatch;     // [TrackIndex, PatchUnion]
typedef std::pair<size_t, size_t>                   GlobalPatch;    // [FrameIndex, PatchIndex]
typedef std::map<size_t, std::vector<GlobalPatch> > GlobalPatches;  // [TrackIndex, <GlobalPatch>]
typedef std::pair<size_t, size_t>                   SubContext;     // [start, end)

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
class PointLocalReconstruction {
 public:
  PointLocalReconstruction() : plrlNumberOfModesMinus1_( 0 ), plrBlockThresholdPerPatchMinus1_( 0 ) {
    plrlMinimumDepth_.clear();
    plrlNeighbourMinus1_.clear();
    plrlInterpolateFlag_.clear();
    plrlFillingFlag_.clear();
  };
  ~PointLocalReconstruction() {
    plrlMinimumDepth_.clear();
    plrlNeighbourMinus1_.clear();
    plrlInterpolateFlag_.clear();
    plrlFillingFlag_.clear();
  };

  PointLocalReconstruction& operator=( const PointLocalReconstruction& ) = default;

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

  void setPlrlNumberOfModesMinus1( uint8_t value ) { plrlNumberOfModesMinus1_ = value; }
  void setPlrBlockThresholdPerPatchMinus1( uint8_t value ) { plrBlockThresholdPerPatchMinus1_ = value; }
  void setPlrlMinimumDepth( size_t index, uint8_t value ) { plrlMinimumDepth_[index] = value; }
  void setPlrlNeighbourMinus1( size_t index, uint8_t value ) { plrlNeighbourMinus1_[index] = value; }
  void setPlrlInterpolateFlag( size_t index, bool value ) { plrlInterpolateFlag_[index] = value; }
  void setPlrlFillingFlag( size_t index, bool value ) { plrlFillingFlag_[index] = value; }

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
  ppduPatchIndex_(0),
  ppduFrameIndex_(0)
  {};
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
  size_t getPpduPatchIndex() { return ppduPatchIndex_; }
  size_t getPpduFrameIndex() { return ppduFrameIndex_; }
  void setPpduPatchIndex(size_t value) { ppduPatchIndex_=value; }
  void setPpduFrameIndex(size_t value) { ppduFrameIndex_=value; }
  void setPatchInPcmVideoFlag( bool value ) { ppduPatchInPcmVideoFlag_ = value; }
  void set2DShiftU( size_t value ) { ppdu2DShiftU_ = value; }
  void set2DShiftV( size_t value ) { ppdu2DShiftV_ = value; }
  void set2DDeltaSizeU( int64_t value ) { ppdu2DDeltaSizeU_ = value; }
  void set2DDeltaSizeV( int64_t value ) { ppdu2DDeltaSizeV_ = value; }
  void set3DShiftTangentAxis( size_t value ) { ppdu3DShiftTangentAxis_ = value; }
  void set3DShiftBiTangentAxis( size_t value ) { ppdu3DShiftBiTangentAxis_ = value; }
  void set3DShiftNormalAxis( size_t value ) { ppdu3DShiftNormalAxis_ = value; }
  void setPcmPoints( uint32_t value ) { ppduPcmPoints_ = value; }

 private:
  bool     ppduPatchInPcmVideoFlag_;
  size_t   ppdu2DShiftU_;
  size_t   ppdu2DShiftV_;
  int64_t  ppdu2DDeltaSizeU_;
  int64_t  ppdu2DDeltaSizeV_;
  size_t  ppdu3DShiftTangentAxis_;
  size_t  ppdu3DShiftBiTangentAxis_;
  size_t  ppdu3DShiftNormalAxis_;
  uint32_t ppduPcmPoints_;
  size_t ppduPatchIndex_;
  size_t ppduFrameIndex_;
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
      dpdu2DDeltaSizeD_( 0 ),
      dpdu3DDeltaShiftTangentAxis_( 0 ),
      dpdu3DDeltaShiftBiTangentAxis_( 0 ),
      dpdu3DDeltaShiftNormalAxis_( 0 ),
      dpduProjectPlane_(pcc::PCCAxis6(0)),
      dpduLod_( 0 ),
  dpduPatchIndex_(0),
  dpduFrameIndex_(0)
  {};
  ~DeltaPatchDataUnit(){};
  DeltaPatchDataUnit& operator=( const DeltaPatchDataUnit& ) = default;
  int64_t                       getDeltaPatchIdx() { return dpdutDeltaPatchIndex_; }
  int64_t                       get2DDeltaShiftU() { return dpdu2DDeltaShiftU_; }
  int64_t                       get2DDeltaShiftV() { return dpdu2DDeltaShiftV_; }
  int64_t                       get2DDeltaSizeU() { return dpdu2DDeltaSizeU_; }
  int64_t                       get2DDeltaSizeV() { return dpdu2DDeltaSizeV_; }
  int64_t                       get2DDeltaSizeD() { return dpdu2DDeltaSizeD_; }
  int64_t                       get3DDeltaShiftTangentAxis() { return dpdu3DDeltaShiftTangentAxis_; }
  int64_t                       get3DDeltaShiftBiTangentAxis() { return dpdu3DDeltaShiftBiTangentAxis_; }
  int64_t                       get3DDeltaShiftNormalAxis() { return dpdu3DDeltaShiftNormalAxis_; }
  PCCAxis6                      getProjectPlane() { return dpduProjectPlane_; }
  uint8_t                       getLod() { return dpduLod_; }
  PointLocalReconstructionData& getPointLocalReconstructionData() { return pointLocalReconstructionData_; }
  size_t getDpduPatchIndex() { return dpduPatchIndex_; }
  size_t getDpduFrameIndex() { return dpduFrameIndex_; }
  void setDpduPatchIndex(size_t value) { dpduPatchIndex_=value; }
  void setDpduFrameIndex(size_t value) { dpduFrameIndex_=value; }
  void setDeltaPatchIdx( int64_t value ) { dpdutDeltaPatchIndex_ = value; }
  void set2DDeltaShiftU( int64_t value ) { dpdu2DDeltaShiftU_ = value; }
  void set2DDeltaShiftV( int64_t value ) { dpdu2DDeltaShiftV_ = value; }
  void set2DDeltaSizeU( int64_t value ) { dpdu2DDeltaSizeU_ = value; }
  void set2DDeltaSizeV( int64_t value ) { dpdu2DDeltaSizeV_ = value; }
  void set2DDeltaSizeD( int64_t value ) { dpdu2DDeltaSizeD_ = value; }
  void set3DDeltaShiftTangentAxis( int64_t value ) { dpdu3DDeltaShiftTangentAxis_ = value; }
  void set3DDeltaShiftBiTangentAxis( int64_t value ) { dpdu3DDeltaShiftBiTangentAxis_ = value; }
  void set3DDeltaShiftNormalAxis( int64_t value ) { dpdu3DDeltaShiftNormalAxis_ = value; }
  void setProjectPlane(PCCAxis6 value) { dpduProjectPlane_ = value; }
  void setLod( uint8_t value ) { dpduLod_ = value; }
  void setPointLocalReconstructionData( PointLocalReconstructionData value ) { pointLocalReconstructionData_ = value; }

 private:
  int64_t                      dpdutDeltaPatchIndex_;
  int64_t                      dpdu2DDeltaShiftU_;  // sizes need to be determined
  int64_t                      dpdu2DDeltaShiftV_;
  int64_t                      dpdu2DDeltaSizeU_;
  int64_t                      dpdu2DDeltaSizeV_;
  int64_t                      dpdu2DDeltaSizeD_;
  int64_t                      dpdu3DDeltaShiftTangentAxis_;
  int64_t                      dpdu3DDeltaShiftBiTangentAxis_;
  int64_t                      dpdu3DDeltaShiftNormalAxis_;
  PCCAxis6                     dpduProjectPlane_;
  uint8_t                      dpduLod_;
  size_t dpduPatchIndex_;
  size_t dpduFrameIndex_;
  PointLocalReconstructionData pointLocalReconstructionData_;
};

// 7.3.5.18  Patch data unit syntax (pdu)
class PatchDataUnit {
 public:
  PatchDataUnit() :
      pdu2DShiftU_( 0 ),  // sizes need to be determined
      pdu2DShiftV_( 0 ),
      pdu2DDeltaSizeU_( 0 ),
      pdu2DDeltaSizeV_( 0 ),
      pdu2DDeltaSizeD_( 255 ),
      pdu3DShiftTangentAxis_( 0 ),
      pdu3DShiftBiTangentAxis_( 0 ),
      pdu3DShiftNormalAxis_( 0 ),
      pduProjectPlane_( pcc::PCCAxis6(0) ),
      pduOrientationIndex_( 0 ),
      pduLod_( 0 ),
      pdu45DegreeProjectionPresentFlag_( false ),
      pdu45DegreeProjectionRotationAxis_( 0 ),
      pduPatchIndex_(0),
      pduFrameIndex_(0)
  {};

  ~PatchDataUnit(){};

  PatchDataUnit& operator=( const PatchDataUnit& ) = default;

  size_t                        get2DShiftU() { return pdu2DShiftU_; }
  size_t                        get2DShiftV() { return pdu2DShiftV_; }
  int64_t                       get2DDeltaSizeU() { return pdu2DDeltaSizeU_; }
  int64_t                       get2DDeltaSizeV() { return pdu2DDeltaSizeV_; }
  int64_t                       get2DDeltaSizeD() { return pdu2DDeltaSizeD_; }
  size_t                        get3DShiftTangentAxis() { return pdu3DShiftTangentAxis_; }
  size_t                        get3DShiftBiTangentAxis() { return pdu3DShiftBiTangentAxis_; }
  size_t                        get3DShiftNormalAxis() { return pdu3DShiftNormalAxis_; }
  PCCAxis6                      getProjectPlane() { return pduProjectPlane_; }
  uint8_t                       getOrientationIndex() { return pduOrientationIndex_; }
  uint8_t                       getLod() { return pduLod_; }
  PointLocalReconstructionData& getPointLocalReconstructionData() { return pointLocalReconstructionData_; }
  bool                          get45DegreeProjectionPresentFlag() { return pdu45DegreeProjectionPresentFlag_; }
  uint8_t                       get45DegreeProjectionRotationAxis() { return pdu45DegreeProjectionRotationAxis_; }
  size_t getPduPatchIndex() { return pduPatchIndex_; }
  size_t getPduFrameIndex() { return pduFrameIndex_; }
  void setPduPatchIndex(size_t value) { pduPatchIndex_=value; }
  void setPduFrameIndex(size_t value) { pduFrameIndex_=value; }
  void set2DShiftU( size_t value ) { pdu2DShiftU_ = value; }
  void set2DShiftV( size_t value ) { pdu2DShiftV_ = value; }
  void set2DDeltaSizeU( int64_t value ) { pdu2DDeltaSizeU_ = value; }
  void set2DDeltaSizeV( int64_t value ) { pdu2DDeltaSizeV_ = value; }
  void set2DDeltaSizeD( int64_t value ) { pdu2DDeltaSizeD_ = value; }
  void set3DShiftTangentAxis( size_t value ) { pdu3DShiftTangentAxis_ = value; }
  void set3DShiftBiTangentAxis( size_t value ) { pdu3DShiftBiTangentAxis_ = value; }
  void set3DShiftNormalAxis( size_t value ) { pdu3DShiftNormalAxis_ = value; }
  void setProjectPlane(PCCAxis6 value) { pduProjectPlane_ = value; }
  void setOrientationIndex( uint8_t value ) { pduOrientationIndex_ = value; }
  void setLod( uint8_t value ) { pduLod_ = value; }
  void setPointLocalReconstructionData( PointLocalReconstructionData value ) { pointLocalReconstructionData_ = value; }
  void set45DegreeProjectionPresentFlag( bool value ) { pdu45DegreeProjectionPresentFlag_ = value; }
  void set45DegreeProjectionRotationAxis( uint8_t value ) { pdu45DegreeProjectionRotationAxis_ = value; }

 private:
  size_t                       pdu2DShiftU_;  // sizes need to be determined
  size_t                       pdu2DShiftV_;
  int64_t                      pdu2DDeltaSizeU_;
  int64_t                      pdu2DDeltaSizeV_;
  int64_t                      pdu2DDeltaSizeD_;
  size_t                       pdu3DShiftTangentAxis_;
  size_t                       pdu3DShiftBiTangentAxis_;
  size_t                       pdu3DShiftNormalAxis_;
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
  size_t getFrameIndex(){ return frameIndex_;}
  size_t getPatchIndex(){ return patchIndex_;}
  
  void setFrameIndex(size_t value){ frameIndex_=value; }
  void setPatchIndex(size_t value){ patchIndex_=value; }
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

// 7.3.5.16  Patch frame data unit syntax (pfdu)
class PatchFrameDataUnit {
 public:
  PatchFrameDataUnit() {}
  ~PatchFrameDataUnit() {
    patchMode_.clear();
    patchInformationData_.clear();
  }
  void init() {
    patchMode_.clear();
    patchInformationData_.clear();
  }
  void allocatePatch( uint8_t pCount ) {
    if ( pCount ) {
      patchMode_.resize( pCount );
      patchInformationData_.resize( pCount );
    }
  }

  PatchFrameDataUnit& operator=( const PatchFrameDataUnit& ) = default;

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
  size_t getMatchedPatchCount() {
    size_t matchedPatchCount = 0;
    for ( auto& v : patchMode_ ) {
      if ( v == PATCH_MODE_P_INTER ) { matchedPatchCount++; }
    }
    return matchedPatchCount;
  }

  size_t getFrameIndex(){ return frameIndex_;}
  void setFrameIndex(size_t value){ frameIndex_=value; }
  void setPatchCount( size_t value ) { patchCount_ = value; }
  void setPatchMode( size_t index, uint8_t value ) { patchMode_[index] = value; }
  void setPatchInformationData( size_t index, PatchInformationData& value ) { patchInformationData_[index] = value; }

 private:
  size_t                            frameIndex_;
  size_t                            patchCount_;
  std::vector<uint8_t>              patchMode_;
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

// 7.3.5.14  Patch frame header syntax (pfh) TODO: should we create variables similar to CD text?
class PatchFrameHeader {
 public:
  PatchFrameHeader() :
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
      pcm3dShiftBitCountPresentFlag_( true )
  {
    additionalPfocLsbPresentFlag_.resize( 1, 0 );
    additionalPfocLsbVal_.resize( 1, 0 );
  }
  PatchFrameHeader& operator=( const PatchFrameHeader& ) = default;

  uint8_t  getFrameIndex() { return frameIndex_; }
  uint8_t  getPatchFrameParameterSetId() { return patchFrameParameterSetId_; }
  uint32_t getAddress() { return address_; }
  uint32_t getType() { return type_; }
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
  uint8_t getPcm3dShiftAxisBitCountMinus1() {
    return pcm3dShiftAxisBitCountMinus1_;
  }
  bool getPcm3dShiftBitCountPresentFlag() { return pcm3dShiftBitCountPresentFlag_; }
  uint8_t getInterPredictPatchLodBitCount() { return interPredictPatchLodBitCount_; }

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
  void setPcm3dShiftAxisBitCountMinus1(uint8_t value) {
    pcm3dShiftAxisBitCountMinus1_ = value;
  }
  
  void setPcm3dShiftBitCountPresentFlag(uint8_t value) {
      pcm3dShiftBitCountPresentFlag_ = value;
  }

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
class PatchFrameLayerUnit {
 public:
  PatchFrameLayerUnit() : frameIndex_( 0 ) {}
  PatchFrameLayerUnit& operator=( const PatchFrameLayerUnit& ) = default;

  uint8_t             getFrameIndex() { return frameIndex_; }
  PatchFrameHeader&   getPatchFrameHeader() { return patchFrameHeader_; }
  PatchFrameDataUnit& getPatchFrameDataUnit() { return patchFrameDataUnit_; }

  void setFrameIndex( uint8_t value ) { frameIndex_ = value; }
  void setPatchFrameHeader( PatchFrameHeader value ) { patchFrameHeader_ = value; }
  void setPatchFrameDataUnit( PatchFrameDataUnit value ) { patchFrameDataUnit_ = value; }

 private:
  uint8_t            frameIndex_;
  PatchFrameHeader   patchFrameHeader_;
  PatchFrameDataUnit patchFrameDataUnit_;
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
      projection45DegreeEnableFlag_( false ) {
    allocatePatchFrame();
  }
  ~PatchFrameParameterSet() {
    localOverrideAttributePatchEnableFlag_.clear();
    attributePatchFrameParameterSetId_.clear();
  }
  PatchFrameParameterSet& operator=( const PatchFrameParameterSet& ) = default;

  void allocatePatchFrame( size_t size = 255 ) {
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
  bool getLocalOverrideGeometryPatchEnableFlag() { return localOverrideGeometryPatchEnableFlag_; }
  bool getProjection45DegreeEnableFlag() { return projection45DegreeEnableFlag_; }

  void setPatchFrameParameterSetId( uint8_t value ) { patchFrameParameterSetId_ = value; }
  void setPatchSequenceParameterSetId( uint8_t value ) { patchSequenceParameterSetId_ = value; }
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

 private:
  uint8_t              patchFrameParameterSetId_;
  uint8_t              patchSequenceParameterSetId_;
  uint8_t              geometryPatchFrameParameterSetId_;
  std::vector<uint8_t> attributePatchFrameParameterSetId_;
  uint8_t              additionalLtPfocLsbLen_;
  bool                 localOverrideGeometryPatchEnableFlag_;
  std::vector<bool>    localOverrideAttributePatchEnableFlag_;
  bool                 projection45DegreeEnableFlag_;
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

// 7.3.5.10 Attribute Patch Parameter Set syntax (appss) TODO: add attributeDimension_
class AttributePatchParameterSet {
 public:
  AttributePatchParameterSet() :
      attributePatchParameterSetId_( 0 ),
      attributeFrameParameterSetId_( 0 ),
      attributePatchParamsPresentFlag_( false ) {}
  AttributePatchParameterSet& operator=( const AttributePatchParameterSet& ) = default;

  uint8_t               getAttributePatchParameterSetId() { return attributePatchParameterSetId_; }
  uint8_t               getAttributeFrameParameterSetId() { return attributeFrameParameterSetId_; }
  bool                  getAttributePatchParamsPresentFlag() { return attributePatchParamsPresentFlag_; }
  AttributePatchParams& getAttributePatchParams() { return attributePatchParams_; }

  void setAttributePatchParameterSetId( uint8_t value ) { attributePatchParameterSetId_ = value; }
  void setAttributeFrameParameterSetId( uint8_t value ) { attributeFrameParameterSetId_ = value; }
  void setAttributePatchParamsPresentFlag( bool value ) { attributePatchParamsPresentFlag_ = value; }
  void setAttributePatchParams( AttributePatchParams value ) { attributePatchParams_ = value; }

 private:
  uint8_t              attributePatchParameterSetId_;
  uint8_t              attributeFrameParameterSetId_;
  bool                 attributePatchParamsPresentFlag_;
  AttributePatchParams attributePatchParams_;
};

// 7.3.5.9 Geometry patch params syntax TODO: rotation should be defined using quaternions (dimension 4)
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
      geometryPatchScaleOnAxis_[d]    = 0;  // u(32)
      geometryPatchOffsetOnAxis_[d]   = 0;  // i(32)
    }
   for ( size_t d = 0; d < 4; d++ ) {
     geometryPatchRotationXYZW_[d] = 0;  // i(32)
   }
  }
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
      geometryFrameParameterSetId_( 0 ),
      geometryPatchParamsPresentFlag_( false ) {}
  GeometryPatchParameterSet& operator=( const GeometryPatchParameterSet& ) = default;

  uint8_t              getGeometryPatchParameterSetId() { return geometryPatchParameterSetId_; }
  uint8_t              getGeometryFrameParameterSetId() { return geometryFrameParameterSetId_; }
  bool                 getGeometryPatchParamsPresentFlag() { return geometryPatchParamsPresentFlag_; }
  GeometryPatchParams& getGeometryPatchParams() { return geometryPatchParams_; }

  void setGeometryPatchParameterSetId( uint8_t value ) { geometryPatchParameterSetId_ = value; }
  void setGeometryFrameParameterSetId( uint8_t value ) { geometryFrameParameterSetId_ = value; }
  void setGeometryPatchParamsPresentFlag( bool value ) { geometryPatchParamsPresentFlag_ = value; }
  void setGeometryPatchParams( GeometryPatchParams value ) { geometryPatchParams_ = value; }

 private:
  uint8_t             geometryPatchParameterSetId_;
  uint8_t             geometryFrameParameterSetId_;
  bool                geometryPatchParamsPresentFlag_;
  GeometryPatchParams geometryPatchParams_;
};

// 7.3.5.7 Attribute frame paramS syntax (afp) TODO: attributeSmoothingParamsPresentFlag_, attributeSmoothingThreshold_,
// attributeSmoothingThresholdLocalEntropy_ are lists with size attributeDimension; remove attributeSmoothingRadius_,
// attributeSmoothingNeighbourCount_ and attributeSmoothingRadius2BoundaryDetection_; add attributeSmoothingGridSize_,
// attributeSmoothingAttributeVariation_ and attributeSmoothingAttributeDifference_
class AttributeFrameParams {
 public:
  AttributeFrameParams() :
      attributeSmoothingParamsPresentFlag_( false ),
      attributeScaleParamsPresentFlag_( false ),
      attributeOffsetParamsPresentFlag_( false ),
      attributeSmoothingRadius_( 0 ),
      attributeSmoothingNeighbourCount_( 0 ),
      attributeSmoothingRadius2BoundaryDetection_( 0 ),
      attributeSmoothingThreshold_( 0 ),
      attributeSmoothingThresholdLocalEntropy_( 0 ),
      attributeDimension_( 0 ) {
    attributeScale_.clear();
    attributeOffset_.clear();
  }
  AttributeFrameParams& operator=( const AttributeFrameParams& ) = default;

  void allocate( size_t size ) {
    attributeScale_.resize( size, 0 );
    attributeOffset_.resize( size, 0 );
  }

  bool     getAttributeSmoothingParamsPresentFlag() { return attributeSmoothingParamsPresentFlag_; }
  bool     getAttributeScaleParamsPresentFlag() { return attributeScaleParamsPresentFlag_; }
  bool     getAttributeOffsetParamsPresentFlag() { return attributeOffsetParamsPresentFlag_; }
  uint8_t  getAttributeSmoothingRadius() { return attributeSmoothingRadius_; }
  uint8_t  getAttributeSmoothingNeighbourCount() { return attributeSmoothingNeighbourCount_; }
  uint8_t  getAttributeSmoothingRadius2BoundaryDetection() { return attributeSmoothingRadius2BoundaryDetection_; }
  uint8_t  getAttributeSmoothingThreshold() { return attributeSmoothingThreshold_; }
  uint32_t getAttributeSmoothingThresholdLocalEntropy() { return attributeSmoothingThresholdLocalEntropy_; }
  uint32_t getAttributeScale( size_t index ) { return attributeScale_[index]; }
  int32_t  getAttributeOffset( size_t index ) { return attributeOffset_[index]; }

  void setAttributeSmoothingParamsPresentFlag( bool value ) { attributeSmoothingParamsPresentFlag_ = value; }
  void setAttributeScaleParamsPresentFlag( bool value ) { attributeScaleParamsPresentFlag_ = value; }
  void setAttributeOffsetParamsPresentFlag( bool value ) { attributeOffsetParamsPresentFlag_ = value; }
  void setAttributeSmoothingRadius( uint8_t value ) { attributeSmoothingRadius_ = value; }
  void setAttributeSmoothingNeighbourCount( uint8_t value ) { attributeSmoothingNeighbourCount_ = value; }
  void setAttributeSmoothingRadius2BoundaryDetection( uint8_t value ) {
    attributeSmoothingRadius2BoundaryDetection_ = value;
  }
  void setAttributeSmoothingThreshold( uint8_t value ) { attributeSmoothingThreshold_ = value; }
  void setAttributeSmoothingThresholdLocalEntropy( uint32_t value ) {
    attributeSmoothingThresholdLocalEntropy_ = value;
  }
  void setAttributeScale( size_t index, uint32_t value ) { attributeScale_[index] = value; }
  void setAttributeOffset( size_t index, int32_t value ) { attributeOffset_[index] = value; }
  void setDimension( size_t value ) { attributeDimension_ = value; }

 private:
  bool                  attributeSmoothingParamsPresentFlag_;
  bool                  attributeScaleParamsPresentFlag_;
  bool                  attributeOffsetParamsPresentFlag_;
  uint8_t               attributeSmoothingRadius_;
  uint8_t               attributeSmoothingNeighbourCount_;
  uint8_t               attributeSmoothingRadius2BoundaryDetection_;
  uint8_t               attributeSmoothingThreshold_;
  uint32_t              attributeSmoothingThresholdLocalEntropy_;
  std::vector<uint32_t> attributeScale_;
  std::vector<int32_t>  attributeOffset_;
  size_t                attributeDimension_;
};

// 7.3.5.6 Patch frame attribute parameter set syntax TODO: change name of the class from AttributeFrameParameterSet to
// PatchFrameAttributeParameterSet, remove overrideAttributeParamsFlag_, overrideAttributePatchParamsFlag_
class AttributeFrameParameterSet {
 public:
  AttributeFrameParameterSet() :
      attributeFrameParameterSetId_( 0 ),
      patchSequencParameterSetId_( 0 ),
      attributeDimensionMinus1_( 3 ),
      overrideAttributeParamsFlag_( false ),
      overrideAttributePatchParamsFlag_( false ),
      attributePatchScaleParamsEnabledFlag_( false ),
      attributePatchOffsetParamsEnabledFlag_( false ) {
    attributeFrameParams_.setDimension( attributeDimensionMinus1_ + 1 );
  }
  AttributeFrameParameterSet& operator=( const AttributeFrameParameterSet& ) = default;

  uint8_t               getAttributeFrameParameterSetId() { return attributeFrameParameterSetId_; }
  uint8_t               getPatchSequencParameterSetId() { return patchSequencParameterSetId_; }
  uint8_t               getAttributeDimensionMinus1() { return attributeDimensionMinus1_; }
  bool                  getOverrideAttributeParamsFlag() { return overrideAttributeParamsFlag_; }
  bool                  getOverrideAttributePatchParamsFlag() { return overrideAttributePatchParamsFlag_; }
  bool                  getAttributePatchScaleParamsEnabledFlag() { return attributePatchScaleParamsEnabledFlag_; }
  bool                  getAttributePatchOffsetParamsEnabledFlag() { return attributePatchOffsetParamsEnabledFlag_; }
  AttributeFrameParams& getAttributeFrameParams() { return attributeFrameParams_; }

  void setAttributeFrameParameterSetId( uint8_t value ) { attributeFrameParameterSetId_ = value; }
  void setPatchSequencParameterSetId( uint8_t value ) { patchSequencParameterSetId_ = value; }
  void setAttributeDimensionMinus1( uint8_t value ) { attributeDimensionMinus1_ = value; }
  void setOverrideAttributeParamsFlag( bool value ) { overrideAttributeParamsFlag_ = value; }
  void setOverrideAttributePatchParamsFlag( bool value ) { overrideAttributePatchParamsFlag_ = value; }
  void setAttributePatchScaleParamsEnabledFlag( bool value ) { attributePatchScaleParamsEnabledFlag_ = value; }
  void setAttributePatchOffsetParamsEnabledFlag( bool value ) { attributePatchOffsetParamsEnabledFlag_ = value; }
  void setAttributeFrameParams( AttributeFrameParams value ) { attributeFrameParams_ = value; }

 private:
  uint8_t              attributeFrameParameterSetId_;
  uint8_t              patchSequencParameterSetId_;
  uint8_t              attributeDimensionMinus1_;
  bool                 overrideAttributeParamsFlag_;
  bool                 overrideAttributePatchParamsFlag_;
  bool                 attributePatchScaleParamsEnabledFlag_;
  bool                 attributePatchOffsetParamsEnabledFlag_;
  AttributeFrameParams attributeFrameParams_;
};

// 7.3.5.5 Geometry frame params syntax TODO: rotation should be defined using quaternions (dimension 4)
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
      geometrySmoothingGridSize_( 0 ),
      geometrySmoothingThreshold_( 0 ),
      geometryPointSizeInfo_( 0 ),
      geometryPointShapeInfo_( 0 ) {
    for ( uint8_t i = 0; i < 3; i++ ) {
      geometryScaleOnAxis_[i]    = 0;
      geometryOffsetOnAxis_[i]   = 0;
    }
    for ( uint8_t i = 0; i < 4; i++ ) {
     geometryRotationXYZW_[i] = 0;
    }
  }
  GeometryFrameParams& operator=( const GeometryFrameParams& ) = default;

  bool     getGeometrySmoothingParamsPresentFlag() { return geometrySmoothingParamsPresentFlag_; }
  bool     getGeometryScaleParamsPresentFlag() { return geometryScaleParamsPresentFlag_; }
  bool     getGeometryOffsetParamsPresentFlag() { return geometryOffsetParamsPresentFlag_; }
  bool     getGeometryRotationParamsPresentFlag() { return geometryRotationParamsPresentFlag_; }
  bool     getGeometryPointSizeInfoPresentFlag() { return geometryPointSizeInfoPresentFlag_; }
  bool     getGeometryPointShapeInfoPresentFlag() { return geometryPointShapeInfoPresentFlag_; }
  bool     getGeometrySmoothingEnabledFlag() { return geometrySmoothingEnabledFlag_; }
  uint8_t  getGeometrySmoothingGridSize() { return geometrySmoothingGridSize_; }
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
  void setGeometrySmoothingGridSize( uint8_t value ) { geometrySmoothingGridSize_ = value; }
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
  uint8_t  geometrySmoothingGridSize_;
  uint8_t  geometrySmoothingThreshold_;
  uint32_t geometryScaleOnAxis_[3];
  int32_t  geometryOffsetOnAxis_[3];
  int32_t  geometryRotationXYZW_[4];
  uint16_t geometryPointSizeInfo_;
  uint32_t geometryPointShapeInfo_;
};

// 7.3.5.4 Patch frame geometry parameter set syntax TODO: rename class from GeometryFrameParameterSet to
// PatchFrameGeometryParameterSet, remove geometryParamsEnabledFlag_, overrideGeometryParamsFlag_
class GeometryFrameParameterSet {
 public:
  GeometryFrameParameterSet() :
      geometryFrameParameterSetId_( 0 ),
      patchSequenceParameterSetId_( 0 ),
      geometryParamsEnabledFlag_( false ),
      overrideGeometryParamsFlag_( false ),
      geometryPatchParamsEnabledFlag_( false ),
      geometryPatchScaleParamsEnabledFlag_( false ),
      geometryPatchOffsetParamsEnabledFlag_( false ),
      geometryPatchRotationParamsEnabledFlag_( false ),
      geometryPatchPointSizeInfoEnabledFlag_( false ),
      geometryPatchPointShapeInfoEnabledFlag_( false ) {}

  GeometryFrameParameterSet& operator=( const GeometryFrameParameterSet& ) = default;

  uint8_t              getGeometryFrameParameterSetId() { return geometryFrameParameterSetId_; }
  uint8_t              getPatchSequenceParameterSetId() { return patchSequenceParameterSetId_; }
  bool                 getGeometryParamsEnabledFlag() { return geometryParamsEnabledFlag_; }
  bool                 getOverrideGeometryParamsFlag() { return overrideGeometryParamsFlag_; }
  bool                 getGeometryPatchParamsEnabledFlag() { return geometryPatchParamsEnabledFlag_; }
  bool                 getOverrideGeometryPatchParamsFlag() { return overrideGeometryPatchParamsFlag_; }
  bool                 getGeometryPatchScaleParamsEnabledFlag() { return geometryPatchScaleParamsEnabledFlag_; }
  bool                 getGeometryPatchOffsetParamsEnabledFlag() { return geometryPatchOffsetParamsEnabledFlag_; }
  bool                 getGeometryPatchRotationParamsEnabledFlag() { return geometryPatchRotationParamsEnabledFlag_; }
  bool                 getGeometryPatchPointSizeInfoEnabledFlag() { return geometryPatchPointSizeInfoEnabledFlag_; }
  bool                 getGeometryPatchPointShapeInfoEnabledFlag() { return geometryPatchPointShapeInfoEnabledFlag_; }
  GeometryFrameParams& getGeometryFrameParams() { return geometryFrameParams_; }

  void setGeometryFrameParameterSetId( uint8_t value ) { geometryFrameParameterSetId_ = value; }
  void setPatchSequenceParameterSetId( uint8_t value ) { patchSequenceParameterSetId_ = value; }
  void setGeometryParamsEnabledFlag( bool value ) { geometryParamsEnabledFlag_ = value; }
  void setOverrideGeometryParamsFlag( bool value ) { overrideGeometryParamsFlag_ = value; }
  void setGeometryPatchParamsEnabledFlag( bool value ) { geometryPatchParamsEnabledFlag_ = value; }
  void setOverrideGeometryPatchParamsFlag( bool value ) { overrideGeometryPatchParamsFlag_ = value; }
  void setGeometryPatchScaleParamsEnabledFlag( bool value ) { geometryPatchScaleParamsEnabledFlag_ = value; }
  void setGeometryPatchOffsetParamsEnabledFlag( bool value ) { geometryPatchOffsetParamsEnabledFlag_ = value; }
  void setGeometryPatchRotationParamsEnabledFlag( bool value ) { geometryPatchRotationParamsEnabledFlag_ = value; }
  void setGeometryPatchPointSizeInfoEnabledFlag( bool value ) { geometryPatchPointSizeInfoEnabledFlag_ = value; }
  void setGeometryPatchPointShapeInfoEnabledFlag( bool value ) { geometryPatchPointShapeInfoEnabledFlag_ = value; }
  void setGeometryFrameParams( GeometryFrameParams value ) { geometryFrameParams_ = value; }

 private:
  uint8_t             geometryFrameParameterSetId_;
  uint8_t             patchSequenceParameterSetId_;
  bool                geometryParamsEnabledFlag_;
  bool                overrideGeometryParamsFlag_;
  bool                geometryPatchParamsEnabledFlag_;
  bool                overrideGeometryPatchParamsFlag_;
  bool                geometryPatchScaleParamsEnabledFlag_;
  bool                geometryPatchOffsetParamsEnabledFlag_;
  bool                geometryPatchRotationParamsEnabledFlag_;
  bool                geometryPatchPointSizeInfoEnabledFlag_;
  bool                geometryPatchPointShapeInfoEnabledFlag_;
  GeometryFrameParams geometryFrameParams_;
};

// 7.3.5.3  Patch sequence parameter set syntax (psps) TODO: add patchPackingBlockSize
class PatchSequenceParameterSet {
 public:
  PatchSequenceParameterSet() :
      patchSequenceParameterSetId_( 0 ),
      log2MaxPatchFrameOrderCntLsb_( 0 ),
      maxDecPatchFrameBuffering_( 0 ),
      numRefPatchFrameListsInSps_( 0 ),
      longTermRefPatchFramesFlag_( false ),
      useEightOrientationsFlag_( false ) {
    refListStruct_.clear();
  }
  ~PatchSequenceParameterSet() { refListStruct_.clear(); }

  PatchSequenceParameterSet& operator=( const PatchSequenceParameterSet& ) = default;

  void           allocate( size_t size ) { refListStruct_.resize( size ); }
  uint8_t        getPatchSequenceParameterSetId() { return patchSequenceParameterSetId_; }
  uint8_t        getLog2MaxPatchFrameOrderCntLsbMinus4() { return log2MaxPatchFrameOrderCntLsb_; }
  uint8_t        getMaxDecPatchFrameBufferingMinus1() { return maxDecPatchFrameBuffering_; }
  uint8_t        getNumRefPatchFrameListsInSps() { return numRefPatchFrameListsInSps_; }
  bool           getLongTermRefPatchFramesFlag() { return longTermRefPatchFramesFlag_; }
  bool           getUseEightOrientationsFlag() { return useEightOrientationsFlag_; }
  RefListStruct& getRefListStruct( uint8_t index ) { return refListStruct_[index]; }
  uint8_t        getRefListStructSize() { return refListStruct_.size(); }

  void setPatchSequenceParameterSetId( uint8_t value ) { patchSequenceParameterSetId_ = value; }
  void setLog2MaxPatchFrameOrderCntLsbMinus4( uint8_t value ) { log2MaxPatchFrameOrderCntLsb_ = value; }
  void setMaxDecPatchFrameBufferingMinus1( uint8_t value ) { maxDecPatchFrameBuffering_ = value; }
  void setNumRefPatchFrameListsInSps( uint8_t value ) { numRefPatchFrameListsInSps_ = value; }
  void setLongTermRefPatchFramesFlag( bool value ) { longTermRefPatchFramesFlag_ = value; }
  void setUseEightOrientationsFlag( bool value ) { useEightOrientationsFlag_ = value; }
  void setRefListStruct( uint8_t index, RefListStruct value ) { refListStruct_[index] = value; }

  void addRefListStruct( RefListStruct value ) { refListStruct_.push_back( value ); }

 private:
  uint8_t                    patchSequenceParameterSetId_;
  uint8_t                    log2MaxPatchFrameOrderCntLsb_;
  uint8_t                    maxDecPatchFrameBuffering_;
  uint8_t                    numRefPatchFrameListsInSps_;
  bool                       longTermRefPatchFramesFlag_;
  bool                       useEightOrientationsFlag_;
  std::vector<RefListStruct> refListStruct_;
};

// 7.3.5.2  Patch data group unit payload syntax (pdgup) TODO: change name of the class from PatchSequenceUnitPayload to
// PatchDataGroupUnitPayload, update enum PSDUnitType with values defined in the CD, missing SEI message
class PatchSequenceUnitPayload {
 public:
  PatchSequenceUnitPayload( PSDUnitType unitType = PSD_SPS, uint8_t index = 0 ) :
      unitType_( unitType ),
      frameIndex_( index ) {}
  PatchSequenceUnitPayload& operator=( const PatchSequenceUnitPayload& ) = default;

  void init( PSDUnitType unitType, uint8_t index ) {
    unitType_   = unitType;
    frameIndex_ = index;
  }
  std::string strUnitType() {
    switch ( unitType_ ) {
      case PSD_SPS: return std::string( "PSD_SPS" ); break;
      case PSD_FPS: return std::string( "PSD_FPS" ); break;
      case PSD_GFPS: return std::string( "PSD_GFPS" ); break;
      case PSD_AFPS: return std::string( "PSD_AFPS" ); break;
      case PSD_GPPS: return std::string( "PSD_GPPS" ); break;
      case PSD_APPS: return std::string( "PSD_APPS" ); break;
      case PSD_PFLU: return std::string( "PSD_PFLU" ); break;
      default: break;
    }
    return std::string( "ERROR" );
  }

  PSDUnitType                 getUnitType() { return unitType_; }
  uint8_t                     getFrameIndex() { return frameIndex_; }
  PatchSequenceParameterSet&  getPatchSequenceParameterSet() { return patchSequenceParameterSet_; }
  GeometryPatchParameterSet&  getGeometryPatchParameterSet() { return geometryPatchParameterSet_; }
  AttributePatchParameterSet& getAttributePatchParameterSet( size_t index ) {
    return attributePatchParameterSet_[index];
  }
  PatchFrameParameterSet&     getPatchFrameParameterSet() { return patchFrameParameterSet_; }
  AttributeFrameParameterSet& getAttributeFrameParameterSet( size_t index ) {
    return attributeFrameParameterSet_[index];
  }
  GeometryFrameParameterSet& getGeometryFrameParameterSet() { return geometryFrameParameterSet_; }
  PatchFrameLayerUnit&       getPatchFrameLayerUnit() { return patchFrameLayerUnit_; }

  void setUnitType( PSDUnitType value ) { unitType_ = value; }
  void setFrameIndex( uint8_t value ) { frameIndex_ = value; }
  void setPatchSequenceParameterSet( PatchSequenceParameterSet value ) { patchSequenceParameterSet_ = value; }
  void setGeometryPatchParameterSet( GeometryPatchParameterSet value ) { geometryPatchParameterSet_ = value; }
  void setAttributePatchParameterSet( size_t index, AttributePatchParameterSet& value ) {
    attributePatchParameterSet_[index] = value;
  }
  void setPatchFrameParameterSet( PatchFrameParameterSet value ) { patchFrameParameterSet_ = value; }
  void setAttributeFrameParameterSet( size_t index, AttributeFrameParameterSet& value ) {
    attributeFrameParameterSet_[index] = value;
  }
  void setGeometryFrameParameterSet( GeometryFrameParameterSet value ) { geometryFrameParameterSet_ = value; }
  void setPatchFrameLayerUnit( PatchFrameLayerUnit value ) { patchFrameLayerUnit_ = value; }

  void addAttributePatchParameterSet( AttributePatchParameterSet& value ) {
    attributePatchParameterSet_.push_back( value );
  }
  void addAttributeFrameParameterSet( AttributeFrameParameterSet& value ) {
    attributeFrameParameterSet_.push_back( value );
  }

 private:
  PSDUnitType                             unitType_;
  uint8_t                                 frameIndex_;
  PatchSequenceParameterSet               patchSequenceParameterSet_;
  GeometryPatchParameterSet               geometryPatchParameterSet_;
  std::vector<AttributePatchParameterSet> attributePatchParameterSet_;
  PatchFrameParameterSet                  patchFrameParameterSet_;
  std::vector<AttributeFrameParameterSet> attributeFrameParameterSet_;
  GeometryFrameParameterSet               geometryFrameParameterSet_;
  PatchFrameLayerUnit                     patchFrameLayerUnit_;
};

// 7.3.5.1  Patch data group unit syntax (pdgu) TODO: change the name of the class from PatchSequenceDataUnit to
// PatchDataGroupUnit
class PatchSequenceDataUnit {
 public:
  PatchSequenceDataUnit() : frameCount_( 0 ) { patchSequenceUnitPayload_.clear(); }
  ~PatchSequenceDataUnit() { patchSequenceUnitPayload_.clear(); }

  PatchSequenceDataUnit& operator=( const PatchSequenceDataUnit& ) = default;

  void addPatchSequenceUnitPayload( PatchSequenceUnitPayload& value ) { patchSequenceUnitPayload_.push_back( value ); }
  PatchSequenceUnitPayload& addPatchSequenceUnitPayload() {
    PatchSequenceUnitPayload psup;
    patchSequenceUnitPayload_.push_back( psup );
    return patchSequenceUnitPayload_.back();
  }

  void addPatchSequenceUnitPayload( const PSDUnitType psdUnitType, const uint8_t index ) {
    patchSequenceUnitPayload_.push_back( PatchSequenceUnitPayload( psdUnitType, index ) );
  }

  void addAttributeFrameParameterSet( AttributeFrameParameterSet& afps, const uint8_t index ) {
    for ( size_t i = 0; i < patchSequenceUnitPayload_.size(); i++ ) {
      if ( patchSequenceUnitPayload_[i].getUnitType() == PSD_AFPS &&
           patchSequenceUnitPayload_[i].getFrameIndex() == index ) {
        patchSequenceUnitPayload_[i].addAttributeFrameParameterSet( afps );
        ;
        return;
      }
    }
    PatchSequenceUnitPayload psup( PSD_AFPS, index );
    psup.addAttributeFrameParameterSet( afps );
    patchSequenceUnitPayload_.push_back( psup );
  }

  void addAttributePatchParameterSet( AttributePatchParameterSet& apps, const uint8_t index ) {
    for ( size_t i = 0; i < patchSequenceUnitPayload_.size(); i++ ) {
      if ( patchSequenceUnitPayload_[i].getUnitType() == PSD_APPS &&
           patchSequenceUnitPayload_[i].getFrameIndex() == index ) {
        patchSequenceUnitPayload_[i].addAttributePatchParameterSet( apps );
        ;
        return;
      }
    }
    PatchSequenceUnitPayload psup( PSD_APPS, index );
    psup.addAttributePatchParameterSet( apps );
    patchSequenceUnitPayload_.push_back( psup );
  }

  uint8_t                   getPatchSequenceDataUnitSize() { return patchSequenceUnitPayload_.size(); }
  uint8_t                   getFrameCount() { return frameCount_; }
  PatchSequenceUnitPayload& getPatchSequenceUnitPayloadElement( const uint8_t index ) {
    return patchSequenceUnitPayload_[index];
  }

  PatchSequenceUnitPayload& getPatchSequenceUnitPayloadPreviousFrame() {
    return patchSequenceUnitPayload_.size() == 1 ? patchSequenceUnitPayload_[0]
                                                 : patchSequenceUnitPayload_[patchSequenceUnitPayload_.size() - 2];
  }

  void printPatchSequenceUnitPayload() {
    for ( size_t i = 0; i < patchSequenceUnitPayload_.size(); i++ ) {
      printf( "PatchSequenceUnitPayload %2lu / %2lu: Type = %u FrameIndex = %u \n", i, patchSequenceUnitPayload_.size(),
              patchSequenceUnitPayload_[i].getUnitType(), patchSequenceUnitPayload_[i].getFrameIndex() );
      fflush( stdout );
    }
  }

  PatchSequenceParameterSet& getPatchSequenceParameterSet( const uint8_t index ) {
    auto& psup = getPatchSequenceUnitPayload( PSD_SPS, index );
    return psup.getPatchSequenceParameterSet();
  }

  PatchFrameParameterSet& getPatchFrameParameterSet( const uint8_t index ) {
    auto& psup = getPatchSequenceUnitPayload( PSD_FPS, index );
    return psup.getPatchFrameParameterSet();
  }

  AttributeFrameParameterSet& getAttributeFrameParameterSet( const uint8_t index, const uint8_t attribute_idx ) {
    auto& psup = getPatchSequenceUnitPayload( PSD_AFPS, index );
    return psup.getAttributeFrameParameterSet( attribute_idx );
  }

  GeometryFrameParameterSet& getGeometryFrameParameterSet( const uint8_t index ) {
    auto& psup = getPatchSequenceUnitPayload( PSD_GFPS, index );
    return psup.getGeometryFrameParameterSet();
  }
  GeometryPatchParameterSet& getGeometryPatchParameterSet( const uint8_t index ) {
    auto& psup = getPatchSequenceUnitPayload( PSD_GPPS, index );
    return psup.getGeometryPatchParameterSet();
  }

  AttributePatchParameterSet& getAttributePatchParameterSet( const uint8_t index, const uint8_t apps_id ) {
    auto& psup = getPatchSequenceUnitPayload( PSD_APPS, index );
    return psup.getAttributePatchParameterSet( apps_id );
  }

  PatchFrameLayerUnit& getPatchFrameLayerUnit( const uint8_t index ) {
    auto& psup = getPatchSequenceUnitPayload( PSD_PFLU, index );
    return psup.getPatchFrameLayerUnit();
  }

  void setFrameCount( uint8_t frameCount ) { frameCount_ = frameCount; }
  void setPatchSequenceUnitPayload( size_t index, PatchSequenceUnitPayload& value ) {
    patchSequenceUnitPayload_[index] = value;
  }

 private:
  PatchSequenceUnitPayload& getPatchSequenceUnitPayload( const PSDUnitType psdUnitType, const uint8_t index ) {
    for ( size_t i = 0; i < patchSequenceUnitPayload_.size(); i++ ) {
      if ( patchSequenceUnitPayload_[i].getUnitType() == psdUnitType &&
           patchSequenceUnitPayload_[i].getFrameIndex() == index ) {
        return patchSequenceUnitPayload_[i];
      }
    }
    fprintf( stderr, "Error: can't find PatchSequenceUnitPayload of type: %u and index = %u  \n", psdUnitType, index );
    exit( -1 );
  }
  uint8_t                               frameCount_;
  std::vector<PatchSequenceUnitPayload> patchSequenceUnitPayload_;
};

// OLD 7.3.13 Attribute Sequence Params Syntax (asps) TODO: remove class, does not exist in CD anymore
class AttributeSequenceParams {
 public:
  AttributeSequenceParams() :
      attributeSmoothingParamsPresentFlag_( false ),
      attributeScaleParamsPresentFlag_( false ),
      attributeOffsetParamsPresentFlag_( false ),
      attributeSmoothingRadius_( 0 ),
      attributeSmoothingNeighbourCount_( 0 ),
      attributeSmoothingRadius2BoundaryDetection_( 0 ),
      attributeSmoothingThreshold_( 0 ),
      attributeSmoothingThresholdLocalEntropy_( 0 ) {
    attributeScale_.clear();
    attributeOffset_.clear();
  }
  AttributeSequenceParams& operator=( const AttributeSequenceParams& ) = default;

  void allocate( size_t size = 1 ) {
    attributeScale_.resize( size );
    attributeOffset_.resize( size );
  }
  void addAttributeScale( uint32_t value ) { attributeScale_.push_back( value ); }
  void addAttributeOffset( int32_t value ) { attributeOffset_.push_back( value ); }

  void init( bool     smoothingParamsPresentFlag,
             bool     scaleParamsPresentFlag,
             bool     offsetParamsPresentFlag,
             uint8_t  smoothingRadius,
             uint8_t  smoothingNeighbourCount,
             uint8_t  smoothingRadius2BoundaryDetection,
             uint8_t  smoothingThreshold,
             uint32_t smoothingThresholdLocalEntropy ) {
    attributeSmoothingParamsPresentFlag_        = smoothingParamsPresentFlag;
    attributeScaleParamsPresentFlag_            = scaleParamsPresentFlag;
    attributeOffsetParamsPresentFlag_           = offsetParamsPresentFlag;
    attributeSmoothingRadius_                   = smoothingRadius;
    attributeSmoothingNeighbourCount_           = smoothingNeighbourCount;
    attributeSmoothingRadius2BoundaryDetection_ = smoothingRadius2BoundaryDetection;
    attributeSmoothingThreshold_                = smoothingThreshold;
    attributeSmoothingThresholdLocalEntropy_    = smoothingThresholdLocalEntropy;
  }

  bool     getAttributeSmoothingParamsPresentFlag() { return attributeSmoothingParamsPresentFlag_; }
  bool     getAttributeScaleParamsPresentFlag() { return attributeScaleParamsPresentFlag_; }
  bool     getAttributeOffsetParamsPresentFlag() { return attributeOffsetParamsPresentFlag_; }
  uint8_t  getAttributeSmoothingRadius() { return attributeSmoothingRadius_; }
  uint8_t  getAttributeSmoothingNeighbourCount() { return attributeSmoothingNeighbourCount_; }
  uint8_t  getAttributeSmoothingRadius2BoundaryDetection() { return attributeSmoothingRadius2BoundaryDetection_; }
  uint8_t  getAttributeSmoothingThreshold() { return attributeSmoothingThreshold_; }
  uint32_t getAttributeSmoothingThresholdLocalEntropy() { return attributeSmoothingThresholdLocalEntropy_; }
  uint32_t getAttributeScale( size_t index ) { return attributeScale_[index]; }
  int32_t  getAttributeOffset( size_t index ) { return attributeOffset_[index]; }

  void setAttributeSmoothingParamsPresentFlag( bool value ) { attributeSmoothingParamsPresentFlag_ = value; }
  void setAttributeScaleParamsPresentFlag( bool value ) { attributeScaleParamsPresentFlag_ = value; }
  void setAttributeOffsetParamsPresentFlag( bool value ) { attributeOffsetParamsPresentFlag_ = value; }
  void setAttributeSmoothingRadius( uint8_t value ) { attributeSmoothingRadius_ = value; }
  void setAttributeSmoothingNeighbourCount( uint8_t value ) { attributeSmoothingNeighbourCount_ = value; }
  void setAttributeSmoothingRadius2BoundaryDetection( uint8_t value ) {
    attributeSmoothingRadius2BoundaryDetection_ = value;
  }
  void setAttributeSmoothingThreshold( uint8_t value ) { attributeSmoothingThreshold_ = value; }
  void setAttributeSmoothingThresholdLocalEntropy( uint32_t value ) {
    attributeSmoothingThresholdLocalEntropy_ = value;
  }
  void setAttributeScale( size_t index, uint32_t value ) { attributeScale_[index] = value; }
  void setAttributeOffset( size_t index, uint32_t value ) { attributeOffset_[index] = value; }

 private:
  bool                  attributeSmoothingParamsPresentFlag_;
  bool                  attributeScaleParamsPresentFlag_;
  bool                  attributeOffsetParamsPresentFlag_;
  uint8_t               attributeSmoothingRadius_;
  uint8_t               attributeSmoothingNeighbourCount_;
  uint8_t               attributeSmoothingRadius2BoundaryDetection_;
  uint8_t               attributeSmoothingThreshold_;
  uint32_t              attributeSmoothingThresholdLocalEntropy_;
  std::vector<uint32_t> attributeScale_;
  std::vector<int32_t>  attributeOffset_;
};

// 7.3.4.5 Attribute Parameter Set Syntax (apss) TODO: add attributeCount, all member variables will be part of lists
// with size attributeCount, and remove attributePatchScaleParamsEnabledFlag_ and attributePatchOffsetParamsEnabledFlag_
class AttributeParameterSet {
 public:
  AttributeParameterSet() :
      attributeTypeId_( 0 ),
      attributeDimensionMinus1_( 2 ),
      attributeCodecId_( 0 ),
      pcmAttributeCodecId_( 0 ),
      attributeParamsEnabledFlag_( false ),
      attributePatchParamsEnabledFlag_( false ),
      attributePatchScaleParamsEnabledFlag_( false ),
      attributePatchOffsetParamsEnabledFlag_( false ) {}

  AttributeParameterSet& operator=( const AttributeParameterSet& ) = default;

  uint8_t                  getAttributeTypeId() { return attributeTypeId_; }
  uint8_t                  getAttributeDimensionMinus1() { return attributeDimensionMinus1_; }
  uint8_t                  getAttributeCodecId() { return attributeCodecId_; }
  uint8_t                  getPcmAttributeCodecId() { return pcmAttributeCodecId_; }
  bool                     getAttributeParamsEnabledFlag() { return attributeParamsEnabledFlag_; }
  bool                     getAttributePatchParamsEnabledFlag() { return attributePatchParamsEnabledFlag_; }
  bool                     getAttributePatchScaleParamsEnabledFlag() { return attributePatchScaleParamsEnabledFlag_; }
  bool                     getAttributePatchOffsetParamsEnabledFlag() { return attributePatchOffsetParamsEnabledFlag_; }
  AttributeSequenceParams& getAttributeSequenceParams() { return attributeSequenceParams_; }

  void setAttributeTypeId( uint8_t value ) { attributeTypeId_ = value; }
  void setAttributeDimensionMinus1( uint8_t value ) { attributeDimensionMinus1_ = value; }
  void setAttributeCodecId( uint8_t value ) { attributeCodecId_ = value; }
  void setPcmAttributeCodecId( uint8_t value ) { pcmAttributeCodecId_ = value; }
  void setAttributeParamsEnabledFlag( bool value ) { attributeParamsEnabledFlag_ = value; }
  void setAttributePatchParamsEnabledFlag( bool value ) { attributePatchParamsEnabledFlag_ = value; }
  void setAttributePatchScaleParamsEnabledFlag( bool value ) { attributePatchScaleParamsEnabledFlag_ = value; }
  void setAttributePatchOffsetParamsEnabledFlag( bool value ) { attributePatchOffsetParamsEnabledFlag_ = value; }
  void setAttributeSequenceParams( AttributeSequenceParams value ) { attributeSequenceParams_ = value; }

 private:
  uint8_t                 attributeTypeId_;
  uint8_t                 attributeDimensionMinus1_;
  uint8_t                 attributeCodecId_;
  uint8_t                 pcmAttributeCodecId_;
  bool                    attributeParamsEnabledFlag_;
  bool                    attributePatchParamsEnabledFlag_;
  bool                    attributePatchScaleParamsEnabledFlag_;
  bool                    attributePatchOffsetParamsEnabledFlag_;
  AttributeSequenceParams attributeSequenceParams_;
};

// OLD 7.3.11 Geometry Sequence Params Syntax TODO: remove class, does not exist in CD anymore
class GeometrySequenceParams {
 public:
  GeometrySequenceParams() :
      geometrySmoothingParamsPresentFlag_( false ),
      geometryScaleParamsPresentFlag_( false ),
      geometryOffsetParamsPresentFlag_( false ),
      geometryRotationParamsPresentFlag_( false ),
      geometryPointSizeInfoPresentFlag_( false ),
      geometryPointShapeInfoPresentFlag_( false ),
      geometrySmoothingEnableFlag_( false ),
      geometrySmoothingGridSize_( 0 ),
      geometrySmoothingThreshold_( 0 ),
      geometryPointSizeInfo_( 0 ),
      geometryPointShapeInfo_( 0 ) {
    for ( uint8_t i = 0; i < 3; i++ ) {
      geometryScaleOnAxis_[i]    = 0;
      geometryOffsetOnAxis_[i]   = 0;
    }
   for ( uint8_t i = 0; i < 4; i++ ) {
      geometryRotationXYZW_[i] = 0;
   }
        
  }
  GeometrySequenceParams& operator=( const GeometrySequenceParams& ) = default;
  void                    init( bool    smoothingParamsPresentFlag,
                                bool    scaleParamsPresentFlag,
                                bool    offsetParamsPresentFlag,
                                bool    rotationParamsPresentFlag,
                                bool    pointSizeInfoPresentFlag,
                                bool    pointShapeInfoPresentFlag,
                                bool    smoothingEnableFlag,
                                uint8_t smoothingGridSize,
                                uint8_t smoothingThreshold ) {
    geometrySmoothingParamsPresentFlag_ = smoothingParamsPresentFlag;
    geometryScaleParamsPresentFlag_     = scaleParamsPresentFlag;
    geometryOffsetParamsPresentFlag_    = offsetParamsPresentFlag;
    geometryRotationParamsPresentFlag_  = rotationParamsPresentFlag;
    geometryPointSizeInfoPresentFlag_   = pointSizeInfoPresentFlag;
    geometryPointShapeInfoPresentFlag_  = pointShapeInfoPresentFlag;
    geometrySmoothingEnableFlag_        = smoothingEnableFlag;
    geometrySmoothingGridSize_          = smoothingGridSize;
    geometrySmoothingThreshold_         = smoothingThreshold;
  }
  bool     getGeometrySmoothingParamsPresentFlag() { return geometrySmoothingParamsPresentFlag_; }
  bool     getGeometryScaleParamsPresentFlag() { return geometryScaleParamsPresentFlag_; }
  bool     getGeometryOffsetParamsPresentFlag() { return geometryOffsetParamsPresentFlag_; }
  bool     getGeometryRotationParamsPresentFlag() { return geometryRotationParamsPresentFlag_; }
  bool     getGeometryPointSizeInfoPresentFlag() { return geometryPointSizeInfoPresentFlag_; }
  bool     getGeometryPointShapeInfoPresentFlag() { return geometryPointShapeInfoPresentFlag_; }
  bool     getGeometrySmoothingEnabledFlag() { return geometrySmoothingEnableFlag_; }
  uint8_t  getGeometrySmoothingGridSize() { return geometrySmoothingGridSize_; }
  uint8_t  getGeometrySmoothingThreshold() { return geometrySmoothingThreshold_; }
  uint32_t getGeometryScaleOnAxis( int index ) { return geometryScaleOnAxis_[index]; }
  int32_t  getGeometryOffsetOnAxis( int index ) { return geometryOffsetOnAxis_[index]; }
  int32_t  getGeometryRotationQuaternion( int index ) { return geometryRotationXYZW_[index]; }
  uint16_t getGeometryPointSizeInfo() { return geometryPointSizeInfo_; }
  uint32_t getGeometryPointShapeInfo() { return geometryPointShapeInfo_; }

  void setGeometrySmoothingParamsPresentFlag( bool value ) { geometrySmoothingParamsPresentFlag_ = value; }
  void setGeometryScaleParamsPresentFlag( bool value ) { geometryScaleParamsPresentFlag_ = value; }
  void setGeometryOffsetParamsPresentFlag( bool value ) { geometryOffsetParamsPresentFlag_ = value; }
  void setGeometryRotationParamsPresentFlag( bool value ) { geometryRotationParamsPresentFlag_ = value; }
  void setGeometryPointSizeInfoPresentFlag( bool value ) { geometryPointSizeInfoPresentFlag_ = value; }
  void setGeometryPointShapeInfoPresentFlag( bool value ) { geometryPointShapeInfoPresentFlag_ = value; }
  void setGeometrySmoothingEnabledFlag( bool value ) { geometrySmoothingEnableFlag_ = value; }
  void setGeometrySmoothingGridSize( uint8_t value ) { geometrySmoothingGridSize_ = value; }
  void setGeometrySmoothingThreshold( uint8_t value ) { geometrySmoothingThreshold_ = value; }
  void setGeometryScaleOnAxis( int index, uint32_t value ) { geometryScaleOnAxis_[index] = value; }
  void setGeometryOffsetOnAxis( int index, int32_t value ) { geometryOffsetOnAxis_[index] = value; }
  void setGeometryRotationQuaternion( int index, int32_t value ) { geometryRotationXYZW_[index] = value; }
  void setGeometryPointSizeInfo( uint16_t value ) { geometryPointSizeInfo_ = value; }
  void setGeometryPointShapeInfo( uint32_t value ) { geometryPointShapeInfo_ = value; }

 private:
  bool     geometrySmoothingParamsPresentFlag_;
  bool     geometryScaleParamsPresentFlag_;
  bool     geometryOffsetParamsPresentFlag_;
  bool     geometryRotationParamsPresentFlag_;
  bool     geometryPointSizeInfoPresentFlag_;
  bool     geometryPointShapeInfoPresentFlag_;
  bool     geometrySmoothingEnableFlag_;
  uint8_t  geometrySmoothingGridSize_;
  uint8_t  geometrySmoothingThreshold_;
  uint32_t geometryScaleOnAxis_[3];
  int32_t  geometryOffsetOnAxis_[3];
  int32_t  geometryRotationXYZW_[4];
  uint16_t geometryPointSizeInfo_;
  uint32_t geometryPointShapeInfo_;
};

// 7.3.4.4 Geometry Parameter Set Syntax TODO: remove geometryPatchScaleParamsEnabledFlag_,
// geometryPatchOffsetParamsEnabledFlag_, geometryPatchRotationParamsEnabledFlag_,
// geometryPatchPointSizeInfoEnabledFlag_, geometryPatchPointShapeInfoEnabledFlag_
class GeometryParameterSet {
 public:
  GeometryParameterSet() :
      geometryCodecId_( 0 ),
      geometryNominal2dBitdepthMinus1_( 10 ),
      geometry3dCoordinatesBitdepthMinus1_( 9 ),
      pcmGeometryCodecId_( 0 ),
      geometryParamsEnabledFlag_( false ),
      geometryPatchParamsEnabledFlag_( false ),
      geometryPatchScaleParamsEnabledFlag_( false ),
      geometryPatchOffsetParamsEnabledFlag_( false ),
      geometryPatchRotationParamsEnabledFlag_( false ),
      geometryPatchPointSizeInfoEnabledFlag_( false ),
      geometryPatchPointShapeInfoEnabledFlag_( false ) {}
  GeometryParameterSet& operator=( const GeometryParameterSet& ) = default;

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
    geometryCodecId_                        = codecId;
    geometryNominal2dBitdepthMinus1_        = nominal2dBitdepthMinus1;
    geometry3dCoordinatesBitdepthMinus1_    = coordinatesBitdepthMinus1;
    pcmGeometryCodecId_                     = pcmGeometryCodecId;
    geometryParamsEnabledFlag_              = paramsEnabledFlag;
    geometryPatchParamsEnabledFlag_         = patchParamsEnabledFlag;
    geometryPatchScaleParamsEnabledFlag_    = patchParamsEnabledFlag;
    geometryPatchOffsetParamsEnabledFlag_   = patchOffsetParamsEnabledFlag;
    geometryPatchRotationParamsEnabledFlag_ = patchRotationParamsEnabledFlag;
    geometryPatchPointSizeInfoEnabledFlag_  = patchPointSizeInfoEnabledFlag;
    geometryPatchPointShapeInfoEnabledFlag_ = patchPointShapeInfoEnabledFlag;
  }
  uint8_t getGeometryCodecId() { return geometryCodecId_; }
  uint8_t getGeometryNominal2dBitdepthMinus1() { return geometryNominal2dBitdepthMinus1_; }
  uint8_t getGeometry3dCoordinatesBitdepthMinus1() { return geometry3dCoordinatesBitdepthMinus1_; }
  uint8_t getPcmGeometryCodecId() { return pcmGeometryCodecId_; }
  bool    getGeometryParamsEnabledFlag() { return geometryParamsEnabledFlag_; }
  bool    getGeometryPatchParamsEnabledFlag() { return geometryPatchParamsEnabledFlag_; }
  bool    getGeometryPatchScaleParamsEnabledFlag() { return geometryPatchScaleParamsEnabledFlag_; }
  bool    getGeometryPatchOffsetParamsEnabledFlag() { return geometryPatchOffsetParamsEnabledFlag_; }
  bool    getGeometryPatchRotationParamsEnabledFlag() { return geometryPatchRotationParamsEnabledFlag_; }
  bool    getGeometryPatchPointSizeInfoEnabledFlag() { return geometryPatchPointSizeInfoEnabledFlag_; }
  bool    getGeometryPatchPointShapeInfoEnabledFlag() { return geometryPatchPointShapeInfoEnabledFlag_; }
  GeometrySequenceParams& getGeometrySequenceParams() { return geometrySequenceParams_; }

  void setGeometryCodecId( uint8_t value ) { geometryCodecId_ = value; }
  void setGeometryNominal2dBitdepthMinus1( uint8_t value ) { geometryNominal2dBitdepthMinus1_ = value; }
  void setGeometry3dCoordinatesBitdepthMinus1( uint8_t value ) { geometry3dCoordinatesBitdepthMinus1_ = value; }
  void setPcmGeometryCodecId( uint8_t value ) { pcmGeometryCodecId_ = value; }
  void setGeometryParamsEnabledFlag( bool value ) { geometryParamsEnabledFlag_ = value; }
  void setGeometryPatchParamsEnabledFlag( bool value ) { geometryPatchParamsEnabledFlag_ = value; }
  void setGeometryPatchScaleParamsEnabledFlag( bool value ) { geometryPatchScaleParamsEnabledFlag_ = value; }
  void setGeometryPatchOffsetParamsEnabledFlag( bool value ) { geometryPatchOffsetParamsEnabledFlag_ = value; }
  void setGeometryPatchRotationParamsEnabledFlag( bool value ) { geometryPatchRotationParamsEnabledFlag_ = value; }
  void setGeometryPatchPointSizeInfoEnabledFlag( bool value ) { geometryPatchPointSizeInfoEnabledFlag_ = value; }
  void setGeometryPatchPointShapeInfoEnabledFlag( bool value ) { geometryPatchPointShapeInfoEnabledFlag_ = value; }
  void setGeometrySequenceParams( GeometrySequenceParams value ) { geometrySequenceParams_ = value; }

 private:
  uint8_t                geometryCodecId_;
  uint8_t                geometryNominal2dBitdepthMinus1_;
  uint8_t                geometry3dCoordinatesBitdepthMinus1_;
  uint8_t                pcmGeometryCodecId_;
  bool                   geometryParamsEnabledFlag_;
  bool                   geometryPatchParamsEnabledFlag_;
  bool                   geometryPatchScaleParamsEnabledFlag_;
  bool                   geometryPatchOffsetParamsEnabledFlag_;
  bool                   geometryPatchRotationParamsEnabledFlag_;
  bool                   geometryPatchPointSizeInfoEnabledFlag_;
  bool                   geometryPatchPointShapeInfoEnabledFlag_;
  GeometrySequenceParams geometrySequenceParams_;
};

// 7.3.4.3 Occupancy Parameter Set Syntax TODO: remove occupancyPackingBlockSize_
class OccupancyParameterSet {
 public:
  OccupancyParameterSet() : occupancyCodecId_( 0 ), occupancyLossyThreshold_( 0 ), occupancyPackingBlockSize_( 0 ) {}
  OccupancyParameterSet& operator=( const OccupancyParameterSet& ) = default;

  void init( uint8_t codecId, uint8_t threshold, uint8_t packingBlockSize ) {
    occupancyCodecId_          = codecId;
    occupancyLossyThreshold_   = threshold;
    occupancyPackingBlockSize_ = packingBlockSize;
  }

  uint8_t getOccupancyCodecId() { return occupancyCodecId_; }
  uint8_t getOccupancyLossyThreshold() { return occupancyLossyThreshold_; }
  uint8_t getOccupancyPackingBlockSize() { return occupancyPackingBlockSize_; }

  void setOccupancyCodecId( uint8_t value ) { occupancyCodecId_ = value; }
  void setOccupancyLossyThreshold( uint8_t value ) { occupancyLossyThreshold_ = value; }
  void setOccupancyPackingBlockSize( uint8_t value ) { occupancyPackingBlockSize_ = value; }

 private:
  uint8_t occupancyCodecId_;
  uint8_t occupancyLossyThreshold_;
  uint8_t occupancyPackingBlockSize_;
};

// 7.3.4.2 Profile, Tier and Level Syntax TODO: change profileIdc to profileCodecGroupIdc, missing profilePCCToolsetIdc,
// profileReconstructionIdc
class ProfileTierLevel {
 public:
  ProfileTierLevel() : tierFlag_( false ), profileIdc_( 0 ), levelIdc_( 0 ) {}
  ProfileTierLevel& operator=( const ProfileTierLevel& ) = default;

  uint8_t getProfileIdc() { return profileIdc_; }
  bool    getTierFlag() { return tierFlag_; }
  uint8_t getLevelIdc() { return levelIdc_; }

  void setTierFlag( bool value ) { tierFlag_ = value; }
  void setProfileIdc( uint8_t value ) { profileIdc_ = value; }
  void setLevelIdc( uint8_t value ) { levelIdc_ = value; }

 private:
  bool    tierFlag_;
  uint8_t profileIdc_;
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
      attributeCount_( 0 ),
      avgFrameRatePresentFlag_( false ),
      enhancedOccupancyMapForDepthFlag_( false ),
      multipleLayerStreamsPresentFlag_( false ),
      pcmPatchEnabledFlag_( false ),
      pcmSeparateVideoPresentFlag_( false ),
      patchInterPredictionEnabledFlag_( false ),
      pixelDeinterleavingFlag_( false ),
      pointLocalReconstructionEnabledFlag_( false ),
      removeDuplicatePointEnabledFlag_( false ),
      projection45degreeEnabledFlag_( false ) {
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
                              bool     projection45degreeEnabledFlag ) {
    sequenceParameterSetId_              = sequenceParameterSetId;
    frameWidth_                          = frameWidth;
    frameHeight_                         = frameHeight;
    avgFrameRate_                        = avgFrameRate;
    layerCountMinus1_                    = layerCountMinus1;
    attributeCount_                      = attributeCount;
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
    allocate();
  }

  void allocate() {
    attributeParameterSets_.resize( attributeCount_ );
    layerAbsoluteCodingEnabledFlag_.resize( layerCountMinus1_ + 1 );
    layerPredictorIndexDiff_.resize( layerCountMinus1_ + 1 );
  }
  uint32_t          getSequenceParameterSetId() { return sequenceParameterSetId_; }
  uint16_t          getFrameWidth() { return frameWidth_; }
  uint16_t          getFrameHeight() { return frameHeight_; }
  uint16_t          getAvgFrameRate() { return avgFrameRate_; }
  uint32_t          getLayerCountMinus1() { return layerCountMinus1_; }
  uint16_t          getAttributeCount() { return attributeCount_; }
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
  size_t            getLayerPredictorIndexDiff( size_t index ) { return layerPredictorIndexDiff_[index]; }
  bool              getLayerAbsoluteCodingEnabledFlag( size_t index ) { return layerAbsoluteCodingEnabledFlag_[index]; }
  ProfileTierLevel& getProfileTierLevel() { return profileTierLevel_; }
  GeometryParameterSet&     getGeometryParameterSet() { return geometryParameterSet_; }
  OccupancyParameterSet&    getOccupancyParameterSet() { return occupancyParameterSet_; }
  AttributeParameterSet&    getAttributeParameterSet( size_t index ) { return attributeParameterSets_[index]; }
  PointLocalReconstruction& getPointLocalReconstruction() { return pointLocalReconstruction_; }

  void setSequenceParameterSetId( uint32_t value ) { sequenceParameterSetId_ = value; }
  void setFrameWidth( uint16_t value ) { frameWidth_ = value; }
  void setFrameHeight( uint16_t value ) { frameHeight_ = value; }
  void setAvgFrameRate( uint16_t value ) { avgFrameRate_ = value; }
  void setLayerCountMinus1( uint32_t value ) { layerCountMinus1_ = value; }
  void setAttributeCount( size_t value ) { attributeCount_ = value; }
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
  void setProfileTierLevel( ProfileTierLevel value ) { profileTierLevel_ = value; }
  void setGeometryParameterSet( GeometryParameterSet value ) { geometryParameterSet_ = value; }
  void setOccupancyParameterSet( OccupancyParameterSet value ) { occupancyParameterSet_ = value; }
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
  void setAttributeParameterSets( size_t index, AttributeParameterSet& value ) {
    if ( attributeParameterSets_.size() < index + 1 ) { attributeParameterSets_.resize( index + 1 ); }
    attributeParameterSets_[index] = value;
  }

  void addLayerAbsoluteCodingEnabledFlag( bool value ) { layerAbsoluteCodingEnabledFlag_.push_back( value ); }
  void addLayerPredictorIndexDiff( bool value ) { layerPredictorIndexDiff_.push_back( value ); }

 private:
  uint32_t sequenceParameterSetId_;
  uint16_t frameWidth_;
  uint16_t frameHeight_;
  uint16_t avgFrameRate_;
  uint32_t layerCountMinus1_;
  uint16_t attributeCount_;
  bool     avgFrameRatePresentFlag_;
  bool     enhancedOccupancyMapForDepthFlag_;
  bool     multipleLayerStreamsPresentFlag_;
  bool     pcmPatchEnabledFlag_;
  bool     pcmSeparateVideoPresentFlag_;
  bool     patchInterPredictionEnabledFlag_;
  bool     pixelDeinterleavingFlag_;
  bool     pointLocalReconstructionEnabledFlag_;
  bool     removeDuplicatePointEnabledFlag_;
  bool     projection45degreeEnabledFlag_;

  std::vector<bool>                  layerAbsoluteCodingEnabledFlag_;
  std::vector<size_t>                layerPredictorIndexDiff_;
  ProfileTierLevel                   profileTierLevel_;
  GeometryParameterSet               geometryParameterSet_;
  OccupancyParameterSet              occupancyParameterSet_;
  std::vector<AttributeParameterSet> attributeParameterSets_;
  PointLocalReconstruction           pointLocalReconstruction_;

  // THE NEXT PARAMETERS ARE NOT IN THE VPCC CD SYNTAX DOCUMENTS AND WILL BE REMOVE
 public:
  bool    getLosslessGeo444() { return losslessGeo444_; }
  bool    getLosslessGeo() { return losslessGeo_; }
  bool    getLosslessTexture() { return losslessTexture_; }
  uint8_t getMinLevel() { return minLevel_; }
  size_t  getSurfaceThickness() { return surfaceThickness_; }

  void setLosslessGeo444( bool losslessGeo444 ) { losslessGeo444_ = losslessGeo444; }
  void setLosslessGeo( bool losslessGeo ) { losslessGeo_ = losslessGeo; }
  void setLosslessTexture( bool losslessTexture ) { losslessTexture_ = losslessTexture; }
  void setMinLevel( uint8_t minLevel ) { minLevel_ = minLevel; }
  void setSurfaceThickness( size_t surfaceThickness ) { surfaceThickness_ = surfaceThickness; }

 private:
  bool    losslessGeo444_;
  bool    losslessGeo_;
  bool    losslessTexture_;
  size_t  surfaceThickness_;
  uint8_t minLevel_;
  // THE NEXT PARAMETERS ARE NOT IN THE VPCC CD SYNTAX DOCUMENTS AND WILL BE REMOVE
};

// VPCCParameterSet Class TODO: add attributeDimensionIndex_
class VPCCParameterSet {
 public:
  VPCCParameterSet() : pcmVideoFlag_( false ), sequenceParamterSetId_( 0 ), attributeIndex_( 0 ), layerIndex_( 0 ) {}
  VPCCParameterSet& operator=( const VPCCParameterSet& ) = default;

  uint8_t getUnitType() { return unitType_; }
  bool    getPCMVideoFlag() { return pcmVideoFlag_; }
  uint8_t getSequenceParameterSetId() { return sequenceParamterSetId_; }
  uint8_t getAttributeIndex() { return attributeIndex_; }
  uint8_t getLayerIndex() { return layerIndex_; }

  void setUnitType( uint8_t type ) { unitType_ = type; }
  void setPCMVideoFlag( bool flag ) { pcmVideoFlag_ = flag; }
  void setSequenceParameterSetId( uint8_t setId ) { sequenceParamterSetId_ = setId; }
  void setAttributeIndex( uint8_t attributeIdx ) { attributeIndex_ = attributeIdx; }
  void setLayerIndex( uint8_t layerIdx ) { layerIndex_ = layerIdx; }

 private:
  uint8_t unitType_;
  bool    pcmVideoFlag_;
  uint8_t sequenceParamterSetId_;
  uint8_t attributeIndex_;
  uint8_t layerIndex_;
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
  uint8_t&                      getOccupancyPrecision() { return occupancyPrecision_; }
  float&                        getModelScale() { return modelScale_; }
  PCCVector3<float>&            getModelOrigin() { return modelOrigin_; }
  size_t&                       getMPGeoWidth() { return MPGeoWidth_; }
  size_t&                       getMPGeoHeight() { return MPGeoHeight_; }
  size_t&                       getMPAttWidth() { return MPAttWidth_; }
  size_t&                       getMPAttHeight() { return MPAttHeight_; }
  PCCMetadata&                  getGOFLevelMetadata() { return gofLevelMetadata_; }
  std::vector<SubContext>&      getSubContexts() { return subContexts_; }
  std::vector<unionPatch>&      getUnionPatch() { return unionPatch_; }
  // Lossy occupancy map
  bool&   getPrefilterLossyOM() { return prefilterLossyOM_; }
  size_t& getOffsetLossyOM() { return offsetLossyOM_; }

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

  PatchSequenceDataUnit& getPatchSequenceDataUnit() { return patchSequenceDataUnit_; }

  void addPointLocalReconstructionMode( const PointLocalReconstructionMode& mode ) {
    pointLocalReconstructionMode_.push_back( mode );
  }
  PointLocalReconstructionMode& getPointLocalReconstructionMode( size_t index ) {
    return pointLocalReconstructionMode_[index];
  }
  size_t getPointLocalReconstructionModeNumber() { return pointLocalReconstructionMode_.size(); }
  size_t getGeometry3dCoordinatesBitdepth() {return geometry3dCoordinatesBitdepth_;}
  void setGeometry3dCoordinatesBitdepth(size_t value){geometry3dCoordinatesBitdepth_=value;}
 private:
  std::vector<PCCFrameContext>      frames_;
  PCCVideoGeometry                  videoGeometry_;
  PCCVideoGeometry                  videoGeometryD1_;
  PCCVideoTexture                   videoTexture_;
  PCCVideoOccupancyMap              videoOccupancyMap_;
  PCCVideoGeometry                  videoMPsGeometry_;
  PCCVideoTexture                   videoMPsTexture_;
  std::vector<PCCVideoBitstream>    videoBitstream_;
  PCCMetadata                       gofLevelMetadata_;
  VPCCParameterSet                  vpccParameterSet_;
  PatchSequenceDataUnit             patchSequenceDataUnit_;
  std::vector<SequenceParameterSet> sequenceParameterSets_;

  // Internale data
  uint8_t                                   occupancyPrecision_;
  size_t                                    MPGeoWidth_;
  size_t                                    MPGeoHeight_;
  size_t                                    MPAttWidth_;
  size_t                                    MPAttHeight_;
  float                                     modelScale_;
  PCCVector3<float>                         modelOrigin_;
  std::vector<SubContext>                   subContexts_;
  std::vector<unionPatch>                   unionPatch_;
  std::vector<PointLocalReconstructionMode> pointLocalReconstructionMode_;
  // Lossy occupancy map encoder parameter
  bool   prefilterLossyOM_;
  size_t offsetLossyOM_;
  size_t geometry3dCoordinatesBitdepth_;
};
};  // namespace pcc

#endif /* PCCContext_h */
