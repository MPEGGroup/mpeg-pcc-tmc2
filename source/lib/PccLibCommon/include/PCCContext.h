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

// 7.3.34 Point local reconstruction syntax
class PointLocalReconstruction {
 public:
  PointLocalReconstruction():
      plrBlockToPatchMapHeight_(0),
      plrBlockToPatchMapWidth_(0) {
    blockToPatchMap_.clear();
    plrModeInterpolateFlag_.clear();
    plrModeNeighbourMinus1_.clear();
    plrModeMinimumDepthMinus1_.clear();
    plrModeFillingFlag_.clear();
  };
  ~PointLocalReconstruction() {
    blockToPatchMap_.clear();
    plrModeInterpolateFlag_.clear();
    plrModeNeighbourMinus1_.clear();
    plrModeMinimumDepthMinus1_.clear();
    plrModeFillingFlag_.clear();
  };

  PointLocalReconstruction& operator=( const PointLocalReconstruction& ) = default;

  uint64_t getBlockToPatchMapHeight() { return plrBlockToPatchMapHeight_; };
  uint64_t getBlockToPatchMapWidth() { return plrBlockToPatchMapWidth_; };
  std::vector<uint8_t> getBlockToPatchMap() { return blockToPatchMap_; }
  uint8_t getBlockToPatchMap( uint64_t x, uint64_t y ) {
    return blockToPatchMap_[y * plrBlockToPatchMapWidth_ + x];
  }
  std::vector<bool> getModeInterpolateFlag() { return plrModeInterpolateFlag_; }
  bool getModeInterpolateFlag( uint64_t x, uint64_t y) {
    return plrModeInterpolateFlag_[y * plrBlockToPatchMapWidth_ + x]; 
  }
  std::vector<uint8_t> getModeNeighbourMinus1() { return plrModeNeighbourMinus1_; }
  uint8_t getModeNeighbourMinus1( uint64_t x, uint64_t y ) {
    return plrModeNeighbourMinus1_[y * plrBlockToPatchMapWidth_ + x];
  }
  std::vector<uint8_t> getModeMinimumDepthMinus1() { return plrModeMinimumDepthMinus1_; }
  uint8_t getModeMinimumDepthMinus1( uint64_t x, uint64_t y ) {
    return plrModeMinimumDepthMinus1_[y * plrBlockToPatchMapWidth_ + y];
  }
  std::vector<bool> getModeFillingFlag() { return plrModeFillingFlag_; }
  bool getModeFillingFlag( uint64_t x, uint64_t y) {
    return plrModeFillingFlag_[y * plrBlockToPatchMapWidth_ + x]; 
  }

  void setBlockToPatchMapHeight( uint64_t height ) { plrBlockToPatchMapHeight_ = height; };
  void setBlockToPatchMapWidth( uint64_t width ) { plrBlockToPatchMapWidth_ = width; };
  void setBlockToPatchMap( std::vector<uint8_t> map ) { blockToPatchMap_ = map; }
  void setBlockToPatchMap( uint8_t val,  uint64_t x, uint64_t y ) {
    blockToPatchMap_[y * plrBlockToPatchMapWidth_ + x] = val;
  }
  void setModeInterpolateFlag( std::vector<bool> modeFlags ) {
    plrModeInterpolateFlag_ = modeFlags;
  }
  void setModeInterpolateFlag( bool flag, uint64_t x, uint64_t y) {
    plrModeInterpolateFlag_[y * plrBlockToPatchMapWidth_ + x] = flag; 
  }
  void setModeNeighbourMinus1(std::vector<uint8_t> modeNeighbous) { 
    plrModeNeighbourMinus1_ = modeNeighbous;
  }
  void setModeNeighbourMinus1(uint8_t modeNeighbour, uint64_t x, uint64_t y ) {
    plrModeNeighbourMinus1_[y * plrBlockToPatchMapWidth_ + x] = modeNeighbour;
  }
  void setModeMinimumDepthMinus1(std::vector<uint8_t> minDepth ) {
    plrModeMinimumDepthMinus1_ = minDepth; 
  }
  void setModeMinimumDepthMinus1(uint8_t minDepthVal, uint64_t x, uint64_t y ) {
    plrModeMinimumDepthMinus1_[y * plrBlockToPatchMapWidth_ + x] = minDepthVal;
  }
  void setModeFillingFlag( std::vector<bool> modeFillingFlags ) {
    plrModeFillingFlag_ = modeFillingFlags;
  }
  void setModeFillingFlag(bool flag, uint64_t x, uint64_t y) {
    plrModeFillingFlag_[y * plrBlockToPatchMapWidth_ + x] = flag; 
  }


 private:
   /* internal */
   uint64_t             plrBlockToPatchMapHeight_;
   uint64_t             plrBlockToPatchMapWidth_;
   /* syntax */
   std::vector<uint8_t> blockToPatchMap_; // size is plrBlockToPatchMapHeight_ * plrBlockToPatchMapWidth_
   std::vector<bool>    plrModeInterpolateFlag_;
   std::vector<uint8_t> plrModeNeighbourMinus1_;
   std::vector<uint8_t> plrModeMinimumDepthMinus1_;
   std::vector<bool>    plrModeFillingFlag_;
};

// 7.3.33  PCM patch data unit syntax
class PCMPatchDataUnit {
 public:
  PCMPatchDataUnit() : 
    ppduPatchInPcmVideoFlag_( false ),
    ppdu2DShiftU_( 0 ), 
    ppdu2DShiftV_( 0 ), 
    ppdu2DDeltaSizeU_( 0 ),
    ppdu2DDeltaSizeV_( 0 ),
    ppduPcmPoints_( 0 ){};
  ~PCMPatchDataUnit(){};
  PCMPatchDataUnit& operator=( const PCMPatchDataUnit& ) = default;

  bool     getPatchInPcmVideoFlag() { return ppduPatchInPcmVideoFlag_; }
  uint64_t get2DShiftU() { return ppdu2DShiftU_; }
  uint64_t get2DShiftV() { return ppdu2DShiftV_; }
  int64_t  get2DDeltaSizeU() { return ppdu2DDeltaSizeU_; }
  int64_t  get2DDeltaSizeV() { return ppdu2DDeltaSizeV_; }
  uint32_t getPcmPoints() { return ppduPcmPoints_;  }

  void setPatchInPcmVideoFlag( bool flag ) { ppduPatchInPcmVideoFlag_ = flag; }
  void set2DShiftU( uint64_t sizeU ) { ppdu2DShiftU_ = sizeU; }
  void set2DShiftV( uint64_t sizeV ) { ppdu2DShiftV_ = sizeV; }
  void set2DDeltaSizeU( int64_t deltaSizeU ) { ppdu2DDeltaSizeU_ = deltaSizeU; }
  void set2DDeltaSizeV( int64_t deltaSizeV ) { ppdu2DDeltaSizeV_ = deltaSizeV; }
  void setPcmPoints( uint32_t numPcmPoints ) { ppduPcmPoints_ = numPcmPoints; }

 private:
   bool     ppduPatchInPcmVideoFlag_;
   uint64_t ppdu2DShiftU_;
   uint64_t ppdu2DShiftV_;
   int64_t  ppdu2DDeltaSizeU_;
   int64_t  ppdu2DDeltaSizeV_;
   uint32_t ppduPcmPoints_;
};

// 7.3.32  Delta Patch data unit syntax
class DeltaPatchDataUnit {
 public:
   DeltaPatchDataUnit() :
     dpduPatchIndex_( 0 ),
     dpdu2DDeltaShiftU_( 0 ),
     dpdu2DDeltaShiftV_( 0 ),
     dpdu2DDeltaSizeU_( 0 ),
     dpdu2DDeltaSizeV_( 0 ),
     dpdu3DDeltaShiftTangentAxis_( 0 ),
     dpdu3DDeltaShiftBiTangentAxis_( 0 ),
     dpdu3DDeltaShiftNormalAxis_( 0 ),
     dpduNormalAxis_( PCC_AXIS3_Z ),
     dpduOrientationSwapFlag_( false ),
     dpduLod_( 1 ),
     dpduProjectionMode_( false ){};
  ~DeltaPatchDataUnit(){};
  DeltaPatchDataUnit& operator=( const DeltaPatchDataUnit& ) = default;
  
  uint8_t  getDeltaPatchIdx() { return dpduPatchIndex_; }
  int64_t  get2DDeltaShiftU() { return dpdu2DDeltaShiftU_; }
  int64_t  get2DDeltaShiftV() { return dpdu2DDeltaShiftV_; }
  int64_t  get2DDeltaSizeU() { return dpdu2DDeltaSizeU_; }
  int64_t  get2DDeltaSizeV() { return dpdu2DDeltaSizeV_; }
  int64_t  get3DDeltaShiftTangentAxis() { return dpdu3DDeltaShiftTangentAxis_; }
  int64_t  get3DDeltaShiftBiTangentAxis() { return dpdu3DDeltaShiftBiTangentAxis_; }
  int64_t  get3DDeltaShiftNormalAxis() { return dpdu3DDeltaShiftNormalAxis_; }
  PCCAxis3 getNormalAxis() { return dpduNormalAxis_; }
  uint8_t  getOrientationSwapFlag() { return dpduOrientationSwapFlag_; }
  uint8_t  getLod() { return dpduLod_; }
  bool     getProjectionMode() { return dpduProjectionMode_; }

  void setDeltaPatchIdx( uint8_t dIdx ) { dpduPatchIndex_ = dIdx; }
  void set2DDeltaShiftU( int64_t uShift ) { dpdu2DDeltaShiftU_ = uShift; }
  void set2DDeltaShiftV( int64_t vShift ) { dpdu2DDeltaShiftV_ = vShift; }
  void set2DDeltaSizeU( int64_t deltaU ) { dpdu2DDeltaSizeU_ = deltaU; }
  void set2DDeltaSizeV( int64_t deltaV ) { dpdu2DDeltaSizeV_ = deltaV; }
  void set3DDeltaShiftTangentAxis( int64_t tangentAxis ) { dpdu3DDeltaShiftTangentAxis_ = tangentAxis; }
  void set3DDeltaShiftBiTangentAxis( int64_t biTangentAxis ) { dpdu3DDeltaShiftBiTangentAxis_ = biTangentAxis; }
  void set3DDeltaShiftNormalAxis( int64_t normalAxis ) { dpdu3DDeltaShiftNormalAxis_ = normalAxis; }
  void setNormalAxis( PCCAxis3 axis ) { dpduNormalAxis_ = axis; }
  void setOrientationSwapFlag( bool flag ) { dpduOrientationSwapFlag_ = flag; }
  void setLod( uint8_t lod ) { dpduLod_ = lod; }
  void setProjectionMode( bool mode ) { dpduProjectionMode_ = mode; }

 private:
   uint8_t  dpduPatchIndex_;
   int64_t  dpdu2DDeltaShiftU_;  // sizes need to be determined
   int64_t  dpdu2DDeltaShiftV_;
   int64_t  dpdu2DDeltaSizeU_;
   int64_t  dpdu2DDeltaSizeV_;
   int64_t  dpdu3DDeltaShiftTangentAxis_;
   int64_t  dpdu3DDeltaShiftBiTangentAxis_;
   int64_t  dpdu3DDeltaShiftNormalAxis_;
   PCCAxis3 dpduNormalAxis_;
   bool     dpduOrientationSwapFlag_;
   uint8_t  dpduLod_;
   bool     dpduProjectionMode_;
};

// 7.3.31  Patch data unit syntax
class PatchDataUnit {
 public:
  PatchDataUnit() :
      pdu2DShiftU_( 0 ),  // sizes need to be determined
      pdu2DShiftV_( 0 ),
      pdu2DDeltaSizeU_( 0 ),
      pdu2DDeltaSizeV_( 0 ),
      pdu3DShiftTangentAxis_( 0 ),
      pdu3DShiftBiTangentAxis_( 0 ),
      pdu3DShiftNormalAxis_( 0 ),
      pduNormalAxis_( PCC_AXIS3_Z ),
      pduOrientationSwapFlag_( false ),
      pduLod_( 1 ),
      pduProjectionMode_( false ){};

  ~PatchDataUnit(){};

  PatchDataUnit& operator=( const PatchDataUnit& ) = default;

  uint64_t get2DShiftU() { return pdu2DShiftU_; }
  uint64_t get2DShiftV() { return pdu2DShiftV_; }
  int64_t  get2DDeltaSizeU() { return pdu2DDeltaSizeU_; }
  int64_t  get2DDeltaSizeV() { return pdu2DDeltaSizeV_; }
  uint64_t get3DShiftTangentAxis() { return pdu3DShiftTangentAxis_; }
  uint64_t get3DShiftBiTangentAxis() { return pdu3DShiftBiTangentAxis_; }
  uint64_t get3DShiftNormalAxis() { return pdu3DShiftNormalAxis_; }
  PCCAxis3 getNormalAxis() { return pduNormalAxis_; }
  uint8_t  getOrientationSwapFlag() { return pduOrientationSwapFlag_; }
  uint8_t  getLod() { return pduLod_; }
  bool     getProjectionMode() { return pduProjectionMode_; }

  void set2DShiftU( uint64_t uShift ) { pdu2DShiftU_ = uShift; }
  void set2DShiftV( uint64_t vShift ) { pdu2DShiftV_ = vShift; }
  void set2DDeltaSizeU( int64_t deltaU ) { pdu2DDeltaSizeU_ = deltaU; }
  void set2DDeltaSizeV( int64_t deltaV ) { pdu2DDeltaSizeV_ = deltaV; }
  void set3DShiftTangentAxis( uint64_t tangentAxis ) { pdu3DShiftTangentAxis_ = tangentAxis; }
  void set3DShiftBiTangentAxis( uint64_t biTangentAxis ) {
    pdu3DShiftBiTangentAxis_ = biTangentAxis;
  }
  void set3DShiftNormalAxis( uint64_t normalAxis ) { pdu3DShiftNormalAxis_ = normalAxis; }
  void setNormalAxis( PCCAxis3 axis ) { pduNormalAxis_ = axis; }
  void setOrientationSwapFlag( bool flag ) { pduOrientationSwapFlag_ = flag; }
  void setLod( uint8_t lod ) { pduLod_ = lod; }
  void setProjectionMode( bool mode ) { pduProjectionMode_ = mode; }

 private:
  uint64_t pdu2DShiftU_;  // sizes need to be determined
  uint64_t pdu2DShiftV_;
  int64_t  pdu2DDeltaSizeU_;
  int64_t  pdu2DDeltaSizeV_;
  uint64_t pdu3DShiftTangentAxis_;
  uint64_t pdu3DShiftBiTangentAxis_;
  uint64_t pdu3DShiftNormalAxis_;
  PCCAxis3 pduNormalAxis_;
  bool     pduOrientationSwapFlag_;
  uint8_t  pduLod_;
  bool     pduProjectionMode_;
};

// 7.3.30  Patch information data syntax
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

  bool                  getOverrideGeometryPatchFlag() { return overrideGeometryPatchFlag_; }
  uint8_t               getGeometryPatchParameterSetId() { return geometryPatchParameterSetId_; }
  std::vector<bool>&    getOverrideAttributePatchFlag() { return overrideAttributePatchFlag_; }
  std::vector<uint8_t>& getAttributePatchParameterSetId() { return attributePatchParameterSetId_; }
  PatchDataUnit&        getPatchDataUnit() { return patchDataUnit_; }
  DeltaPatchDataUnit&   getDeltaPatchDataUnit() { return deltaPatchDataUnit_; }
  PCMPatchDataUnit&     getPCMPatchDataUnit() { return pcmPatchDataUnit_; }

  void setOverrideGeometryPatchFlag( bool flag ) { overrideGeometryPatchFlag_ = flag; }
  void setGeometryPatchParameterSetId( uint8_t gpParameterSetId ) {
    geometryPatchParameterSetId_ = gpParameterSetId;
  }
  void setOverrideAttributePatchFlag( std::vector<bool>& flagVec ) {
    overrideAttributePatchFlag_ = flagVec;
  }
  void seAttributePatchParameterSetId( std::vector<uint8_t>& apParameterSetIdVec ) {
    attributePatchParameterSetId_ = apParameterSetIdVec;
  }
  void setPatchDataUnit( PatchDataUnit& dataUnit ) { patchDataUnit_ = dataUnit; }
  void setDeltaPatchDataUnit( DeltaPatchDataUnit& dataUnit ) { deltaPatchDataUnit_ = dataUnit; }
  void setPCMPatchDataUnit( PCMPatchDataUnit& dataUnit ) { pcmPatchDataUnit_ = dataUnit; }

 private:
  bool                 overrideGeometryPatchFlag_;
  uint8_t              geometryPatchParameterSetId_;
  std::vector<bool>    overrideAttributePatchFlag_;    // size is number of attributes
  std::vector<uint8_t> attributePatchParameterSetId_;  // size is number of attributes
  PatchDataUnit        patchDataUnit_;
  DeltaPatchDataUnit   deltaPatchDataUnit_;
  PCMPatchDataUnit     pcmPatchDataUnit_;
};

// 7.3.29  Patch frame data unit syntax
class PatchFrameDataUnit {
 public:
  PatchFrameDataUnit( uint8_t pCount = 0 ) { allocate( pCount ); }

  ~PatchFrameDataUnit() {
    patchMode_.clear();
    patchInformationData_.clear();
  }

  void init() {
    patchMode_.clear();
    patchInformationData_.clear();
  }

  void allocate( uint8_t pCount ) {
    if ( pCount ) {
      patchMode_.resize( pCount );
      patchInformationData_.resize( pCount );
    }
  }

  PatchFrameDataUnit& operator=( const PatchFrameDataUnit& ) = default;

  uint8_t                            getPatchCount() { return patchMode_.size(); }
  std::vector<uint8_t>&              getPatchMode() { return patchMode_; }
  std::vector<PatchInformationData>& getPatchInformationData() { return patchInformationData_; }
  PointLocalReconstruction&          getPointLocalReconstruction() { return pointLocalReconstruction_; }

  void setPatchFrameMode( std::vector<uint8_t>& mode ) {
    patchMode_ = mode;
  }  // dependency of enum on pfy_type vs. uint8_t?
  void setPatchFrameMode( uint8_t patch, uint8_t mode ) { patchMode_[patch] = mode; }
  void setPatchInformationData( std::vector<PatchInformationData>& pid ) {
    patchInformationData_ = pid;
  }

 private:
  std::vector<uint8_t> patchMode_;
  std::vector<PatchInformationData> patchInformationData_;  // patchCount_ = size of vector
  PointLocalReconstruction pointLocalReconstruction_;
};

// 7.3.28  Reference list structure syntax
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

  uint8_t&              getNumRefEntries() { return numRefEntries_; }
  std::vector<uint8_t>& getAbsDeltaPfocSt() { return absDeltaPfocSt_; }
  std::vector<uint8_t>& getPfocLsbLt() { return pfocLsbLt_; }
  std::vector<bool>&    getStRefPatchFrameFlag() { return stRefPatchFrameFlag_; }
  std::vector<bool>&    getStrpfEntrySignFlag() { return strpfEntrySignFlag_; }
  bool                  getStRefPatchFrameFlag( size_t index ) { return stRefPatchFrameFlag_[index]; }

  void allocate() {
    absDeltaPfocSt_.resize( numRefEntries_, 0 );
    pfocLsbLt_.resize( numRefEntries_, 0 );
    stRefPatchFrameFlag_.resize( numRefEntries_, false );
    strpfEntrySignFlag_.resize( numRefEntries_, false );
  }

 private:
  uint8_t              numRefEntries_;
  std::vector<uint8_t> absDeltaPfocSt_;
  std::vector<uint8_t> pfocLsbLt_;
  std::vector<bool>    stRefPatchFrameFlag_;
  std::vector<bool>    strpfEntrySignFlag_;
};

// 7.3.27  Patch frame header syntax
class PatchFrameHeader {
 public:
  PatchFrameHeader() : frameIndex_( 0 ),
                       patchFrameParameterSetId_( 0 ) ,
                       type_( 0 ),
                       address_( 0 ),
                       patchFrameOrderCntLsb_( 0 ),
                       refPatchFrameListIdx_( 0 ),
                       refPatchFrameListSpsFlag_( 0 ),
                       numRefIdxActiveOverrideFlag_( false ),
                       numRefIdxActiveMinus1_( 0 ),
                       interPredictPatch2dShiftUBitCountMinus1_( 0 ),
                       interPredictPatch2dShiftVBitCountMinus1_( 0 ),
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
                       interPredictPatchLodBitCountFlag_( false ) {
                     additionalPfocLsbPresentFlag_.clear();
                     additionalPfocLsbVal_.clear();
  }
  PatchFrameHeader& operator=( const PatchFrameHeader& ) = default;

  uint8_t               getPatchFrameParameterSetId() { return patchFrameParameterSetId_; }
  uint32_t              getPatchFrameAddress() { return address_; }
  uint32_t              getPatchFrameType() { return type_; }
  uint8_t               getPatchFrameOderCntLsb() { return patchFrameOrderCntLsb_; }
  bool                  getRefPatchFrameListSpsFlag() { return refPatchFrameListSpsFlag_; }
  uint8_t               getRefPatchFrameListIdx() { return refPatchFrameListIdx_; }
  std::vector<bool>     getAdditionalPfocLsbPresentFlag() { return additionalPfocLsbPresentFlag_; }
  std::vector<uint32_t> getAdditionalPfocLsbVal() { return additionalPfocLsbVal_; }
  bool                  getAdditionalPfocLsbPresentFlag(size_t index) { return  additionalPfocLsbPresentFlag_[index]; }
  uint32_t              getAdditionalPfocLsbVal(size_t index) { return additionalPfocLsbVal_[index]; }
  bool                  getPatchFrameNumRefIdxActiveOverrideFlag() { return numRefIdxActiveOverrideFlag_; }
  uint8_t               getPatchFrameNumRefIdxActiveMinus1() { return numRefIdxActiveMinus1_; }

  bool                  getPatchFrameInterPredictPatchBitCountFlag() { return interPredictPatchBitCountFlag_; }
  bool                  getPatchFrameInterPredictPatch2dShiftUBitCountFlag() { return interPredictPatch2dShiftUBitCountFlag_; }
  bool                  getPatchFrameInterPredictPatch2dShiftVBitCountFlag() { return interPredictPatch2dShiftVBitCountFlag_; }
  bool                  getPatchFrameInterPredictPatch3dShiftTangentAxisBitCountFlag() { return interPredictPatch3dShiftTangentAxisBitCountFlag_; }
  bool                  getPatchFrameInterPredictPatch3dShiftBitangentAxisBitCountFlag() { return interPredictPatch3dShiftBitangentAxisBitCountFlag_; }
  bool                  getPatchFrameInterPredictPatch3dShiftNormalAxisBitCountFlag() { return interPredictPatch3dShiftNormalAxisBitCountFlag_; }
  bool                  getPatchFrameInterPredictPatchLodBitCountFlag() { return interPredictPatchLodBitCountFlag_; }
                        
  uint8_t               getPatchFramehPatch2dShiftUBitCountMinus1() { return interPredictPatch2dShiftUBitCountMinus1_; }
  uint8_t               getPatchFramehPatch2dShiftVBitCountMinus1() { return interPredictPatch2dShiftVBitCountMinus1_; }
  uint8_t               getPatchFramehPatch3dShiftTangentAxisBitCountMinus1() { return interPredictPatch3dShiftTangentAxisBitCountMinus1_; }
  uint8_t               getPatchFramehPatch3dShiftBitangentAxisBitCountMinus1() { return interPredictPatch3dShiftBitangentAxisBitCountMinus1_; }
  uint8_t               getPatchFramehPatch3dShiftNormalAxisBitCountMinus1() { return interPredictPatch3dShiftNormalAxisBitCountMinus1_; }
  uint8_t               getPatchFramehPatchLodBitCount() { return interPredictPatchLodBitCount_; }

  void setPatchFrameParameterSetId( uint8_t setIdx ) { patchFrameParameterSetId_ = setIdx ; }
  void setPatchFrameAddress( uint8_t addr ) { address_ = addr; }
  void setPatchFrameType( uint8_t type ) { type_ = type ;}
  void setPatchFrameOderCntLsb( uint8_t patchOrderCnt ) { patchFrameOrderCntLsb_ = patchOrderCnt; }
  void setRefPatchFrameListSpsFlag( bool flag ) { refPatchFrameListSpsFlag_ = flag; }
  void setRefPatchFrameListIdx( uint8_t listIdx ) { refPatchFrameListIdx_ = listIdx ; }
  void setAdditionalPfocLsbPresentFlag(std::vector<bool> flags) { additionalPfocLsbPresentFlag_ = flags; }
  void setAdditionalPfocLsbPresentFlag(size_t index, bool flag) { additionalPfocLsbPresentFlag_[index] = flag; }
  void setAdditionalPfocLsbVal(std::vector<uint32_t> pFLsbVals) { additionalPfocLsbVal_ = pFLsbVals; }
  void setAdditionalPfocLsbVal(size_t index, uint32_t pFLsbVal) { additionalPfocLsbVal_[index] = pFLsbVal; }
  void setPatchFrameNumRefIdxActiveOverrideFlag( bool flag ) { numRefIdxActiveOverrideFlag_ = flag; }
  void setPatchFrameNumRefIdxActiveMinus1( uint8_t refIdx ) { numRefIdxActiveMinus1_ = refIdx; }

  void setPatchFrameInterPredictPatchBitCountFlag(bool flag) { interPredictPatchBitCountFlag_  = flag; }
  void setPatchFrameInterPredictPatch2dShiftUBitCountFlag(bool flag) { interPredictPatch2dShiftUBitCountFlag_ = flag; }
  void setPatchFrameInterPredictPatch2dShiftVBitCountFlag(bool flag) { interPredictPatch2dShiftVBitCountFlag_  = flag; }
  void setPatchFrameInterPredictPatch3dShiftTangentAxisBitCountFlag(bool flag) { interPredictPatch3dShiftTangentAxisBitCountFlag_  = flag; }
  void setPatchFrameInterPredictPatch3dShiftBitangentAxisBitCountFlag(bool flag) { interPredictPatch3dShiftBitangentAxisBitCountFlag_  = flag; }
  void setPatchFrameInterPredictPatch3dShiftNormalAxisBitCountFlag(bool flag) { interPredictPatch3dShiftNormalAxisBitCountFlag_  = flag; }
  void setPatchFrameInterPredictPatchLodBitCountFlag(bool flag) { interPredictPatchLodBitCountFlag_  = flag; }

  void setPatchFramehPatch2dShiftUBitCountMinus1(uint8_t bitCount) { interPredictPatch2dShiftUBitCountMinus1_ = bitCount; }
  void setPatchFramehPatch2dShiftVBitCountMinus1(uint8_t bitCount) { interPredictPatch2dShiftVBitCountMinus1_ = bitCount; }
  void setPatchFramehPatch3dShiftTangentAxisBitCountMinus1(uint8_t bitCount) { interPredictPatch3dShiftTangentAxisBitCountMinus1_ = bitCount; }
  void setPatchFramehPatch3dShiftBitangentAxisBitCountMinus1(uint8_t bitCount) { interPredictPatch3dShiftBitangentAxisBitCountMinus1_ = bitCount; }
  void setPatchFramehPatch3dShiftNormalAxisBitCountMinus1(uint8_t bitCount) { interPredictPatch3dShiftNormalAxisBitCountMinus1_ = bitCount; }
  void setPatchFramehPatchLodBitCount(uint8_t bitCount) { interPredictPatchLodBitCount_ = bitCount; }

 private:
  uint8_t               frameIndex_;
  uint8_t               patchFrameParameterSetId_;
  uint8_t               type_; // this should be enum
  uint32_t              address_; /*is yet to be defined*/
  uint8_t               patchFrameOrderCntLsb_;
  uint8_t               refPatchFrameListIdx_;
  bool                  refPatchFrameListSpsFlag_;
  std::vector<bool>     additionalPfocLsbPresentFlag_;
  std::vector<uint32_t> additionalPfocLsbVal_;
  bool                  numRefIdxActiveOverrideFlag_;
  
  uint8_t               numRefIdxActiveMinus1_;
  uint8_t               interPredictPatch2dShiftUBitCountMinus1_;
  uint8_t               interPredictPatch2dShiftVBitCountMinus1_;
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
};

// 7.3.26  Patch frame layer unit syntax
class PatchFrameLayerUnit {
public:
  PatchFrameLayerUnit() : frameIndex_( 0 ) { }
  PatchFrameLayerUnit& operator=(const PatchFrameLayerUnit&) = default;

  uint8_t             getFrameIndex() { return frameIndex_; }
  PatchFrameHeader&   getPatchFrameHeader() { return patchFrameHeader_; }
  PatchFrameDataUnit& getPatchFrameDataUnit() { return patchFrameDataUnit_; }

  void                setFrameIndex(uint8_t idx) { frameIndex_ = idx; }
  void                setPatchFrameHeader(PatchFrameHeader params) { patchFrameHeader_ = params; }
  void                setPatchFrameDataUnit(PatchFrameDataUnit params) { patchFrameDataUnit_ = params; }

private:
  uint8_t            frameIndex_;
  PatchFrameHeader   patchFrameHeader_;
  PatchFrameDataUnit patchFrameDataUnit_;

};

// 7.3.25  Patch frame parameter set syntax
class PatchFrameParameterSet {
 public:
  PatchFrameParameterSet() :
      patchFrameParameterSetId_( 0 ),
      patchSequenceParameterSetId_( 0 ),
      geometryPatchFrameParameterSetId_( 0 ),
      additionalLtPfocLsbLen_( 0 ),
      localOverrideGeometryPatchEnableFlag_( false ),
      patchOrientationPresentFlag_( false ) {
    localOverrideAttributePatchEnableFlag_.clear();
    attributePatchFrameParameterSetId_.clear();
  }
  ~PatchFrameParameterSet() {
    localOverrideAttributePatchEnableFlag_.clear();
    attributePatchFrameParameterSetId_.clear();
  }
  PatchFrameParameterSet& operator=( const PatchFrameParameterSet& ) = default;

  uint8_t getPatchFrameParameterSetId() { return patchFrameParameterSetId_; }
  uint8_t getPatchSequenceParameterSetId() { return patchSequenceParameterSetId_; }
  uint8_t getGeometryPatchFrameParameterSetId() { return geometryPatchFrameParameterSetId_; }
  std::vector<uint8_t> getAttributePatchFrameParameterSetId() {
    return attributePatchFrameParameterSetId_;
  }
  uint8_t           getAdditionalLtPfocLsbLen() { return additionalLtPfocLsbLen_; }
  std::vector<bool> getLocalOverrideAttributePatchEnableFlag() {
    return localOverrideAttributePatchEnableFlag_;
  }
  bool getPatchOrientationPresentFlag() { return patchOrientationPresentFlag_; }
  bool getLocalOverrideGeometryPatchEnableFlag() { return localOverrideGeometryPatchEnableFlag_; }

  void setPatchFrameParameterSetId( uint8_t setIdx ) { patchFrameParameterSetId_ = setIdx; }
  void setPatchSequenceParameterSetId( uint8_t setIdx ) { patchSequenceParameterSetId_ = setIdx; }
  void setGeometryPatchFrameParameterSetId( uint8_t setIdx ) {
    geometryPatchFrameParameterSetId_ = setIdx;
  }
  void setAttributePatchFrameParameterSetId( std::vector<uint8_t> setIdxArray ) {
    attributePatchFrameParameterSetId_ = setIdxArray;
  }
  void setAdditionalLtPfocLsbLen( uint8_t len ) { additionalLtPfocLsbLen_ = len; }
  void setLocalOverrideAttributePatchEnableFlag( std::vector<bool> flagArray ) {
    localOverrideAttributePatchEnableFlag_ = flagArray;
  }
  void setLocalOverrideAttributePatchEnableFlag( size_t index, bool flag ) {
    localOverrideAttributePatchEnableFlag_[index] = flag;
  }
  void setPatchOrientationPresentFlag( bool flag ) { patchOrientationPresentFlag_ = flag; }
  void setLocalOverrideGeometryPatchEnableFlag( bool flag ) {
    localOverrideGeometryPatchEnableFlag_ = flag;
  }

 private:
  uint8_t              patchFrameParameterSetId_;
  uint8_t              patchSequenceParameterSetId_;
  uint8_t              geometryPatchFrameParameterSetId_;   
  std::vector<uint8_t> attributePatchFrameParameterSetId_;  // sps_attribute_count size
  uint8_t              additionalLtPfocLsbLen_;
  bool                 localOverrideGeometryPatchEnableFlag_;
  bool                 patchOrientationPresentFlag_;
  std::vector<bool>    localOverrideAttributePatchEnableFlag_;
};

// 7.3.24 Attribute patch params syntax
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
  AttributePatchParams& operator=( const AttributePatchParams& ) = default;
  bool getAttributePatchScaleParamsPresentFlag() { return attributePatchScaleParamsPresentFlag_; }
  bool getAttributePatchOffsetParamsPresentFlag() { return attributePatchOffsetParamsPresentFlag_; }
  std::vector<uint32_t> getAttributePatchScale() { return attributePatchScale_; }
  std::vector<int32_t>  getAttributePatchOffset() { return attributePatchOffset_; }
  uint32_t getAttributePatchScale( size_t index ) { return attributePatchScale_[index]; }
  int32_t  getAttributePatchOffset( size_t index ) { return attributePatchOffset_[index]; }

  void setAttributePatchScaleParamsPresentFlag( bool flag ) {
    attributePatchScaleParamsPresentFlag_ = flag;
  }
  void setAttributePatchOffsetParamsPresentFlag( bool flag ) {
    attributePatchOffsetParamsPresentFlag_ = flag;
  }
  void setAttributePatchScale( std::vector<uint32_t> scaleArray ) {
    attributePatchScale_ = scaleArray;
  }
  void setAttributePatchOffset( std::vector<int32_t> offsetArray ) {
    attributePatchOffset_ = offsetArray;
  }
  void setAttributePatchScale( size_t index, uint32_t scale ) {
    attributePatchScale_[index] = scale;
  }
  void setAttributePatchOffset( size_t index, int32_t offset ) {
    attributePatchOffset_[index] = offset;
  }

 private:
  bool                  attributePatchScaleParamsPresentFlag_;   
  std::vector<uint32_t> attributePatchScale_;                    
  bool                  attributePatchOffsetParamsPresentFlag_;  
  std::vector<int32_t>  attributePatchOffset_;                   
};

// 7.3.23 Attribute Patch Parameter Set syntax
class AttributePatchParameterSet {
 public:
  AttributePatchParameterSet() :

      attributePatchParameterSetId_( 0 ),
      attributeFrameParameterSetId_( 0 ),
      // dimension_( 3 ),
      attributePatchParamsPresentFlag_( false ) {}
  AttributePatchParameterSet& operator=( const AttributePatchParameterSet& ) = default;

  uint8_t getAttributePatchParameterSetId() { return attributePatchParameterSetId_; }
  uint8_t getAttributeFrameParameterSetId() { return attributeFrameParameterSetId_; }
  // uint8_t getDimension() { return dimension_; }
  bool    getAttributePatchParamsPresentFlag() { return attributePatchParamsPresentFlag_; }
  AttributePatchParams& getAttributePatchParams() { return attributePatchParams_; }

  void setAttributePatchParameterSetId( uint8_t setId ) { attributePatchParameterSetId_ = setId; }
  void setAttributeFrameParameterSetId( uint8_t setId ) { attributeFrameParameterSetId_ = setId; }
  // void setDimension( uint8_t dimension ) { dimension_ = dimension; }
  void setAttributePatchParamsPresentFlag( bool flag ) { attributePatchParamsPresentFlag_ = flag; }
  void setAttributePatchParams( AttributePatchParams params ) { attributePatchParams_ = params; }

 private:
  uint8_t              attributePatchParameterSetId_;  
  uint8_t              attributeFrameParameterSetId_;  
  // uint8_t              dimension_;                  //JR: must be check with JK 
  bool                 attributePatchParamsPresentFlag_;
  AttributePatchParams attributePatchParams_;
};

// 7.3.22 Geometry patch params syntax
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
      geometryPatchRotationOnAxis_[d] = 0;  // i(32)
    }
  }
  GeometryPatchParams& operator=( const GeometryPatchParams& ) = default;

  bool getGeometryPatchScaleParamsPresentFlag() { return geometryPatchScaleParamsPresentFlag_; }
  bool getGeometryPatchOffsetParamsPresentFlag() { return geometryPatchOffsetParamsPresentFlag_; }
  bool getGeometryPatchRotationParamsPresentFlag() {
    return geometryPatchRotationParamsPresentFlag_;
  }
  bool getGeometryPatchPointSizeInfoPresentFlag() { return geometryPatchPointSizeInfoPresentFlag_; }
  bool getGeometryPatchPointShapeInfoPresentFlag() {
    return geometryPatchPointShapeInfoPresentFlag_;
  }
  uint32_t getGeometryPatchScaleOnAxis( size_t index ) { return geometryPatchScaleOnAxis_[index]; }
  int32_t getGeometryPatchOffsetOnAxis( size_t index ) { return geometryPatchOffsetOnAxis_[index]; }
  int32_t getGeometryPatchRotationOnAxis( size_t index ) {
    return geometryPatchRotationOnAxis_[index];
  }
  uint16_t getGeometryPatchPointSizeInfo() { return geometryPatchPointSizeInfo_; }
  uint32_t getGeometryPatchPointShapeInfo() { return geometryPatchPointShapeInfo_; }

  void setGeometryPatchScaleParamsPresentFlag( bool flag ) {
    geometryPatchScaleParamsPresentFlag_ = flag;
  }
  void setGeometryPatchOffsetParamsPresentFlag( bool flag ) {
    geometryPatchOffsetParamsPresentFlag_ = flag;
  }
  void setGeometryPatchRotationParamsPresentFlag( bool flag ) {
    geometryPatchRotationParamsPresentFlag_ = flag;
  }
  void setGeometryPatchPointSizeInfoPresentFlag( bool flag ) {
    geometryPatchPointSizeInfoPresentFlag_ = flag;
  }
  void setGeometryPatchPointShapeInfoPresentFlag( bool flag ) {
    geometryPatchPointShapeInfoPresentFlag_ = flag;
  }
  void setGeometryPatchScaleOnAxis( size_t index, uint32_t scale ) {
    geometryPatchScaleOnAxis_[index] = scale;
  }
  void setGeometryPatchOffsetOnAxis( size_t index, int32_t offset ) {
    geometryPatchOffsetOnAxis_[index] = offset;
  }
  void setGeometryPatchRotationOnAxis( size_t index, int32_t rotation ) {
    geometryPatchRotationOnAxis_[index] = rotation;
  }
  void setGeometryPatchPointSizeInfo( uint16_t pointSize ) {
    geometryPatchPointSizeInfo_ = pointSize;
  }
  void setGeometryPatchPointShapeInfo( uint32_t pointShape ) {
    geometryPatchPointShapeInfo_ = pointShape;
  }

 private:
  bool     geometryPatchScaleParamsPresentFlag_;
  bool     geometryPatchOffsetParamsPresentFlag_;
  bool     geometryPatchRotationParamsPresentFlag_;
  bool     geometryPatchPointSizeInfoPresentFlag_;
  bool     geometryPatchPointShapeInfoPresentFlag_;
  uint32_t geometryPatchScaleOnAxis_[3];     
  int32_t  geometryPatchOffsetOnAxis_[3];    
  int32_t  geometryPatchRotationOnAxis_[3];  
  uint16_t geometryPatchPointSizeInfo_;      
  uint32_t geometryPatchPointShapeInfo_;     
};

// 7.3.21 Geometry patch parameter set syntax
class GeometryPatchParameterSet {
 public:
  GeometryPatchParameterSet() :
      geometryPatchParameterSetId_( 0 ),
      geometryFrameParameterSetId_( 0 ),
      geometryPatchParamsPresentFlag_( false ) {}
  GeometryPatchParameterSet& operator=( const GeometryPatchParameterSet& ) = default;
  uint8_t getGeometryPatchParameterSetId() { return geometryPatchParameterSetId_; }
  uint8_t getGeometryFrameParameterSetId() { return geometryFrameParameterSetId_; }
  bool    getGeometryPatchParamsPresentFlag() { return geometryPatchParamsPresentFlag_; }
  GeometryPatchParams& getGeometryPatchParams() { return geometryPatchParams_; }
  void setGeometryPatchParameterSetId( uint8_t setId ) { geometryPatchParameterSetId_ = setId; }
  void setGeometryFrameParameterSetId( uint8_t setId ) { geometryFrameParameterSetId_ = setId; }
  void setGeometryPatchParamsPresentFlag( bool flag ) { geometryPatchParamsPresentFlag_ = flag; }
  void setGeometryPatchParams( GeometryPatchParams params ) { geometryPatchParams_ = params; }

 private:
  uint8_t             geometryPatchParameterSetId_;  
  uint8_t             geometryFrameParameterSetId_;  
  bool                geometryPatchParamsPresentFlag_;
  GeometryPatchParams geometryPatchParams_;
};

// 7.3.20 Attribute frame paramse syntax
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
  bool    getAttributeSmoothingParamsPresentFlag() { return attributeSmoothingParamsPresentFlag_; }
  bool    getAttributeScaleParamsPresentFlag() { return attributeScaleParamsPresentFlag_; }
  bool    getAttributeOffsetParamsPresentFlag() { return attributeOffsetParamsPresentFlag_; }
  uint8_t getAttributeSmoothingRadius() { return attributeSmoothingRadius_; }
  uint8_t getAttributeSmoothingNeighbourCount() { return attributeSmoothingNeighbourCount_; }
  uint8_t getAttributeSmoothingRadius2BoundaryDetection() {
    return attributeSmoothingRadius2BoundaryDetection_;
  }
  uint8_t getAttributeSmoothingThreshold() { return attributeSmoothingThreshold_; }
  uint32_t getAttributeSmoothingThresholdLocalEntropy() {
    return attributeSmoothingThresholdLocalEntropy_;
  }  
  std::vector<uint32_t> getAttributeScale() { return attributeScale_; }
  std::vector<int32_t>  getAttributeOffset() { return attributeOffset_; }
  uint32_t              getAttributeScale( size_t index ) { return attributeScale_[index]; }
  int32_t               getAttributeOffset( size_t index ) { return attributeOffset_[index]; }

  void setAttributeSmoothingParamsPresentFlag( bool flag ) {
    attributeSmoothingParamsPresentFlag_ = flag;
  }
  void setAttributeScaleParamsPresentFlag( bool flag ) { attributeScaleParamsPresentFlag_ = flag; }
  void setAttributeOffsetParamsPresentFlag( bool flag ) {
    attributeOffsetParamsPresentFlag_ = flag;
  }
  void setAttributeSmoothingRadius( uint8_t radius ) { attributeSmoothingRadius_ = radius; }
  void setAttributeSmoothingNeighbourCount( uint8_t count ) {
    attributeSmoothingNeighbourCount_ = count;
  }
  void setAttributeSmoothingRadius2BoundaryDetection( uint8_t boundaryDetection ) {
    attributeSmoothingRadius2BoundaryDetection_ = boundaryDetection;
  }
  void setAttributeSmoothingThreshold( uint8_t threshold ) {
    attributeSmoothingThreshold_ = threshold;
  }
  void setAttributeSmoothingThresholdLocalEntropy( uint32_t thresholdLocalEntropy ) {
    attributeSmoothingThresholdLocalEntropy_ = thresholdLocalEntropy;
  }
  void setAttributeScale( std::vector<uint32_t> scaleArray ) { attributeScale_ = scaleArray; }
  void setAttributeOffset( std::vector<int32_t> offsetArray ) { attributeOffset_ = offsetArray; }
  void setAttributeScale( size_t index, uint32_t scale ) { attributeScale_[index] = scale; }
  void setAttributeOffset( size_t index, int32_t offset ) { attributeOffset_[index] = offset; }
  void setDimension( size_t dimension ) { attributeDimension_ = dimension; }

 private:
  bool                  attributeSmoothingParamsPresentFlag_;
  bool                  attributeScaleParamsPresentFlag_;
  bool                  attributeOffsetParamsPresentFlag_;
  uint8_t               attributeSmoothingRadius_;
  uint8_t               attributeSmoothingNeighbourCount_;
  uint8_t               attributeSmoothingRadius2BoundaryDetection_;
  uint8_t               attributeSmoothingThreshold_;
  uint32_t              attributeSmoothingThresholdLocalEntropy_;  // u3
  std::vector<uint32_t> attributeScale_;
  std::vector<int32_t>  attributeOffset_;
  size_t                attributeDimension_;
};

// 7.3.19 Attribute frame parameter set syntax
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
    attributeFrameParams_.setDimension(
        attributeDimensionMinus1_ + 1 );  // jkei : is it correct?? attributeDimensionMinus1_+1?? => JR add 1
  }
  AttributeFrameParameterSet& operator=( const AttributeFrameParameterSet& ) = default;
  uint8_t getAttributeFrameParameterSetId() { return attributeFrameParameterSetId_; }
  uint8_t getPatchSequencParameterSetId() { return patchSequencParameterSetId_; }
  uint8_t getAttributeDimensionMinus1() { return attributeDimensionMinus1_; }
  bool    getOverrideAttributeParamsFlag() { return overrideAttributeParamsFlag_; }
  bool    getOverrideAttributePatchParamsFlag() { return overrideAttributePatchParamsFlag_; }
  bool getAttributePatchScaleParamsEnabledFlag() { return attributePatchScaleParamsEnabledFlag_; }
  bool getAttributePatchOffsetParamsEnabledFlag() { return attributePatchOffsetParamsEnabledFlag_; }
  AttributeFrameParams& getAttributeFrameParams() { return attributeFrameParams_; }

  void setAttributeFrameParameterSetId( uint8_t setId ) { attributeFrameParameterSetId_ = setId; }
  void setPatchSequencParameterSetId( uint8_t setId ) { patchSequencParameterSetId_ = setId; }
  void setAttributeDimensionMinus1( uint8_t dimensionMinus1 ) {
    attributeDimensionMinus1_ = dimensionMinus1;
  }
  void setOverrideAttributeParamsFlag( bool flag ) { overrideAttributeParamsFlag_ = flag; }
  void setOverrideAttributePatchParamsFlag( bool flag ) {
    overrideAttributePatchParamsFlag_ = flag;
  }
  void setAttributePatchScaleParamsEnabledFlag( bool flag ) {
    attributePatchScaleParamsEnabledFlag_ = flag;
  }
  void setAttributePatchOffsetParamsEnabledFlag( bool flag ) {
    attributePatchOffsetParamsEnabledFlag_ = flag;
  }
  void setAttributeFrameParams( AttributeFrameParams params ) { attributeFrameParams_ = params; }

 private:
  uint8_t attributeFrameParameterSetId_;   
  uint8_t patchSequencParameterSetId_;     
  uint8_t attributeDimensionMinus1_;       
  bool    overrideAttributeParamsFlag_;
  bool    overrideAttributePatchParamsFlag_;
  bool    attributePatchScaleParamsEnabledFlag_;
  bool    attributePatchOffsetParamsEnabledFlag_;
  AttributeFrameParams attributeFrameParams_;
};

// 7.3.18 Geometry frame params syntax
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
    for ( size_t d = 0; d < 3; d++ ) {
      geometryScaleOnAxis_[d]    = 0;
      geometryOffsetOnAxis_[d]   = 0;
      geometryRotationOnAxis_[d] = 0;
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
  int32_t  getGeometryRotationOnAxis( size_t index ) { return geometryRotationOnAxis_[index]; }
  uint16_t getGeometryPointSizeInfo() { return geometryPointSizeInfo_; }
  uint32_t getGeometryPointShapeInfo() { return geometryPointShapeInfo_; }

  void setGeometrySmoothingParamsPresentFlag( bool flag ) {
    geometrySmoothingParamsPresentFlag_ = flag;
  }
  void setGeometryScaleParamsPresentFlag( bool flag ) { geometryScaleParamsPresentFlag_ = flag; }
  void setGeometryOffsetParamsPresentFlag( bool flag ) { geometryOffsetParamsPresentFlag_ = flag; }
  void setGeometryRotationParamsPresentFlag( bool flag ) {
    geometryRotationParamsPresentFlag_ = flag;
  }
  void setGeometryPointSizeInfoPresentFlag( bool flag ) {
    geometryPointSizeInfoPresentFlag_ = flag;
  }
  void setGeometryPointShapeInfoPresentFlag( bool flag ) {
    geometryPointShapeInfoPresentFlag_ = flag;
  }
  void setGeometrySmoothingEnabledFlag( bool flag ) { geometrySmoothingEnabledFlag_ = flag; }
  void setGeometrySmoothingGridSize( uint8_t gridSize ) { geometrySmoothingGridSize_ = gridSize; }
  void setGeometrySmoothingThreshold( uint8_t threshold ) {
    geometrySmoothingThreshold_ = threshold;
  }
  void setGeometryScaleOnAxis( size_t index, uint32_t scale ) {
    geometryScaleOnAxis_[index] = scale;
  }
  void setGeometryOffsetOnAxis( size_t index, int32_t offset ) {
    geometryOffsetOnAxis_[index] = offset;
  }
  void setGeometryRotationOnAxis( size_t index, int32_t rotation ) {
    geometryRotationOnAxis_[index] = rotation;
  }
  void setGeometryPointSizeInfo( uint16_t pointSize ) { geometryPointSizeInfo_ = pointSize; }
  void setGeometryPointShapeInfo( uint32_t pointShape ) { geometryPointShapeInfo_ = pointShape; }

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
  int32_t  geometryRotationOnAxis_[3];  
  uint16_t geometryPointSizeInfo_;      
  uint32_t geometryPointShapeInfo_;     
};

// 7.3.17 Geometry frame parameter set syntax
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
  uint8_t getGeometryFrameParameterSetId() { return geometryFrameParameterSetId_; }
  uint8_t getPatchSequenceParameterSetId() { return patchSequenceParameterSetId_; }
  bool    getGeometryParamsEnabledFlag() { return geometryParamsEnabledFlag_; }
  bool    getOverrideGeometryParamsFlag() { return overrideGeometryParamsFlag_; }
  bool    getGeometryPatchParamsEnabledFlag() { return geometryPatchParamsEnabledFlag_; }
  bool    getOverrideGeometryPatchParamsFlag() { return overrideGeometryPatchParamsFlag_; }
  bool    getGeometryPatchScaleParamsEnabledFlag() { return geometryPatchScaleParamsEnabledFlag_; }
  bool getGeometryPatchOffsetParamsEnabledFlag() { return geometryPatchOffsetParamsEnabledFlag_; }
  bool getGeometryPatchRotationParamsEnabledFlag() {
    return geometryPatchRotationParamsEnabledFlag_;
  }
  bool getGeometryPatchPointSizeInfoEnabledFlag() { return geometryPatchPointSizeInfoEnabledFlag_; }
  bool getGeometryPatchPointShapeInfoEnabledFlag() {
    return geometryPatchPointShapeInfoEnabledFlag_;
  }
  GeometryFrameParams& getGeometryFrameParams() { return geometryFrameParams_; }

  void setGeometryFrameParameterSetId( uint8_t setId ) { geometryFrameParameterSetId_ = setId; }
  void setPatchSequenceParameterSetId( uint8_t setId ) { patchSequenceParameterSetId_ = setId; }
  void setGeometryParamsEnabledFlag( bool flag ) { geometryParamsEnabledFlag_ = flag; }
  void setOverrideGeometryParamsFlag( bool flag ) { overrideGeometryParamsFlag_ = flag; }
  void setGeometryPatchParamsEnabledFlag( bool flag ) { geometryPatchParamsEnabledFlag_ = flag; }
  void setOverrideGeometryPatchParamsFlag( bool flag ) { overrideGeometryPatchParamsFlag_ = flag; }
  void setGeometryPatchScaleParamsEnabledFlag( bool flag ) {
    geometryPatchScaleParamsEnabledFlag_ = flag;
  }
  void setGeometryPatchOffsetParamsEnabledFlag( bool flag ) {
    geometryPatchOffsetParamsEnabledFlag_ = flag;
  }
  void setGeometryPatchRotationParamsEnabledFlag( bool flag ) {
    geometryPatchRotationParamsEnabledFlag_ = flag;
  }
  void setGeometryPatchPointSizeInfoEnabledFlag( bool flag ) {
    geometryPatchPointSizeInfoEnabledFlag_ = flag;
  }
  void setGeometryPatchPointShapeInfoEnabledFlag( bool flag ) {
    geometryPatchPointShapeInfoEnabledFlag_ = flag;
  }
  void setGeometryFrameParams( GeometryFrameParams geometryFrameParams ) {
    geometryFrameParams_ = geometryFrameParams;
  }

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

// 7.3.16  Patch sequence parameter set syntax
class PatchSequenceParameterSet {
 public:
  PatchSequenceParameterSet() :
      patchSequenceParameterSetId_( 0 ),
      log2MaxPatchFrameOrderCntLsb_( 0 ),
      maxDecPatchFrameBuffering_( 0 ),
      numRefPatchFrameListsInSps_( 0 ),
      longTermRefPatchFramesFlag_( false ) {
    refListStruct_.clear();
  }
  ~PatchSequenceParameterSet() { refListStruct_.clear(); }
  PatchSequenceParameterSet& operator=( const PatchSequenceParameterSet& ) = default;

  uint8_t        getPatchSequenceParameterSetId() { return patchSequenceParameterSetId_; }
  uint8_t        getLog2MaxPatchFrameOrderCntLsbMinus4() { return log2MaxPatchFrameOrderCntLsb_; }
  uint8_t        getMaxDecPatchFrameBufferingMinus1() { return maxDecPatchFrameBuffering_; }
  uint8_t        getNumRefPatchFrameListsInSps() { return numRefPatchFrameListsInSps_; }
  bool           getLongTermRefPatchFramesFlag() { return longTermRefPatchFramesFlag_; }
  RefListStruct& getRefListStruct( uint8_t index ) { return refListStruct_[index]; }
  std::vector<RefListStruct>& getRefListStruct() { return refListStruct_; }

  void setPatchSequenceParameterSetId( uint8_t setIdx ) { patchSequenceParameterSetId_ = setIdx; }
  void setLog2MaxPatchFrameOrderCntLsbMinus4( uint8_t frameOrderCnt ) {
    log2MaxPatchFrameOrderCntLsb_ = frameOrderCnt;
  }
  void setMaxDecPatchFrameBufferingMinus1( uint8_t frameBuffering ) {
    maxDecPatchFrameBuffering_ = frameBuffering;
  }
  void setNumRefPatchFrameListsInSps( uint8_t frameListInSPS ) {
    numRefPatchFrameListsInSps_ = frameListInSPS;
  }
  void setLongTermRefPatchFramesFlag( bool flag ) { longTermRefPatchFramesFlag_ = flag; }
  void setRefListStruct( uint8_t index, RefListStruct refStruct ) {
    refListStruct_[index] = refStruct;
  }
  void setRefListStruct( std::vector<RefListStruct> refLit ) { refListStruct_ = refLit; }

 private:
  uint8_t                    patchSequenceParameterSetId_;
  uint8_t                    log2MaxPatchFrameOrderCntLsb_;
  uint8_t                    maxDecPatchFrameBuffering_;
  uint8_t                    numRefPatchFrameListsInSps_;
  bool                       longTermRefPatchFramesFlag_;
  std::vector<RefListStruct> refListStruct_;
};

// 7.3.15  Patch sequence unit payload syntax
class PatchSequenceUnitPayload {
 public:
  PatchSequenceUnitPayload() : unitType_( PSD_SPS ), frameIndex_( 0 ) {}
  PatchSequenceUnitPayload& operator=( const PatchSequenceUnitPayload& ) = default;

  PSDUnitType& getUnitType() { return unitType_; }
  uint8_t&     getFrameIndex() { return frameIndex_; }
  void         setUnitType( PSDUnitType unitType ) { unitType_ = unitType; }
  void         setFrameIndex( uint8_t idx ) { frameIndex_ = idx; }

  PatchSequenceParameterSet&  getPatchSequenceParameterSet() { return patchSequenceParameterSet_; }
  GeometryPatchParameterSet&  getGeometryPatchParameterSet() { return geometryPatchParameterSet_; }
  std::vector<AttributePatchParameterSet>& getAttributePatchParameterSets() { return attributePatchParameterSet_; }
  AttributePatchParameterSet& getAttributePatchParameterSet( size_t attributeIndex ) {
    return attributePatchParameterSet_[attributeIndex];
  }
  AttributePatchParameterSet& getAttributeParameterSet( size_t attributeIndex ) {
    return attributePatchParameterSet_[attributeIndex];
  }
  PatchFrameParameterSet&     getPatchFrameParameterSet() { return patchFrameParameterSet_; }
  std::vector < AttributeFrameParameterSet>& getAttributeFrameParameterSets() {
    return attributeFrameParameterSet_;
  }
  AttributeFrameParameterSet& getAttributeFrameParameterSet( size_t attributeIndex ) {
    return attributeFrameParameterSet_[attributeIndex];
  }
  GeometryFrameParameterSet& getGeometryFrameParameterSet() { return geometryFrameParameterSet_; }
  PatchFrameLayerUnit&       getPatchFrameLayerUnit() { return patchFrameLayerUnit_; }

  void setPatchSequenceParameterSet( PatchSequenceParameterSet param ) {
    patchSequenceParameterSet_ = param;
  }
  void setGeometryPatchParameterSet( GeometryPatchParameterSet param ) {
    geometryPatchParameterSet_ = param;
  }
  void setAttributePatchParameterSet( std::vector<AttributePatchParameterSet> param ) {
    attributePatchParameterSet_ = param;
  }
  void setPatchFrameParameterSet( PatchFrameParameterSet param ) {
    patchFrameParameterSet_ = param;
  }
  void setAttributeFrameParameterSet( std::vector<AttributeFrameParameterSet> param ) {
    attributeFrameParameterSet_ = param;
  }
  void setGeometryFrameParameterSet( GeometryFrameParameterSet param ) {
    geometryFrameParameterSet_ = param;
  }
  void setPatchFrameLayerUnit( PatchFrameLayerUnit param ) { patchFrameLayerUnit_ = param; }

 private:
  PSDUnitType                unitType_;
  uint8_t                    frameIndex_;
  PatchSequenceParameterSet  patchSequenceParameterSet_;
  GeometryPatchParameterSet  geometryPatchParameterSet_;
  std::vector<AttributePatchParameterSet> attributePatchParameterSet_;
  PatchFrameParameterSet     patchFrameParameterSet_;
  std::vector<AttributeFrameParameterSet> attributeFrameParameterSet_;
  GeometryFrameParameterSet  geometryFrameParameterSet_;
  PatchFrameLayerUnit        patchFrameLayerUnit_;
};

// 7.3.14  Patch sequence data unit syntax
class PatchSequenceDataUnit {
 public:
  PatchSequenceDataUnit() : frameCount_( 0 ) { patchSequenceUnitPayload_.clear(); }
  ~PatchSequenceDataUnit() { patchSequenceUnitPayload_.clear(); }

  PatchSequenceDataUnit& operator=( const PatchSequenceDataUnit& ) = default;

  uint8_t getPatchSequenceDataUnitSize() { return patchSequenceUnitPayload_.size(); }
  uint8_t getFrameCount( std::vector<PatchSequenceUnitPayload> patchSequenceUnitPayload ) {
    uint8_t frameCount = 0;
    for ( int payloadIndex = 0; payloadIndex < patchSequenceUnitPayload.size(); payloadIndex++ ) {
      frameCount += ( patchSequenceUnitPayload[payloadIndex].getUnitType() == PSD_PFLU );
    }
    return frameCount;
  }
  uint8_t&                               getFrameCount() { return frameCount_; }  // PSD_PFLU only
  std::vector<PatchSequenceUnitPayload>& getPatchSequenceUnitPayload() {
    return patchSequenceUnitPayload_;
  }

  PatchSequenceParameterSet& getPatchSequenceParameterSet( const uint8_t psps_id ) {
    for ( int payloadIndex = 0; payloadIndex < patchSequenceUnitPayload_.size(); payloadIndex++ ) {
      if ( patchSequenceUnitPayload_[payloadIndex].getUnitType() == PSD_SPS ) {
        // now check if the id is the requested one
        auto& psps = patchSequenceUnitPayload_[payloadIndex].getPatchSequenceParameterSet();
        if ( psps.getPatchSequenceParameterSetId() == psps_id ) return psps;
      }
    }
  }

  GeometryFrameParameterSet& getGeometryFrameParameterSet( const uint8_t gfps_id ) {
    for ( int payloadIndex = 0; payloadIndex < patchSequenceUnitPayload_.size(); payloadIndex++ ) {
      if ( patchSequenceUnitPayload_[payloadIndex].getUnitType() == PSD_GFPS ) {
        // now check if the id is the requested one
        auto& gfps = patchSequenceUnitPayload_[payloadIndex].getGeometryFrameParameterSet();
        if ( gfps.getGeometryFrameParameterSetId() == gfps_id ) return gfps;
      }
    }
  }

  AttributeFrameParameterSet& getAttributeFrameParameterSet( const uint8_t afps_id, const uint8_t attribute_idx ) {
    for ( int payloadIndex = 0; payloadIndex < patchSequenceUnitPayload_.size(); payloadIndex++ ) {
      if ( patchSequenceUnitPayload_[payloadIndex].getUnitType() == PSD_AFPS ) {
        // now check if the id is the requested one
        auto& afps =
            patchSequenceUnitPayload_[payloadIndex].getAttributeFrameParameterSet( attribute_idx );
        if ( afps.getAttributeFrameParameterSetId() == afps_id ) return afps;
      }
    }
  }

  PatchFrameParameterSet& getPatchFrameParameterSet( const uint8_t pfps_id) {
    for ( int payloadIndex = 0; payloadIndex < patchSequenceUnitPayload_.size(); payloadIndex++ ) {
      if ( patchSequenceUnitPayload_[payloadIndex].getUnitType() == PSD_FPS ) {
        // now check if the id is the requested one
        auto& pfps = patchSequenceUnitPayload_[payloadIndex].getPatchFrameParameterSet();
        if ( pfps.getPatchFrameParameterSetId() == pfps_id ) return pfps;
      }
    }
  }

  void setFrameCount( uint8_t frameCount ) { frameCount_ = frameCount; }
  void setPatchSequenceUnitPayload( std::vector<PatchSequenceUnitPayload> unitPayload ) {
    patchSequenceUnitPayload_ = unitPayload;
  }
 private:
  uint8_t frameCount_;

  std::vector<PatchSequenceUnitPayload> patchSequenceUnitPayload_;
};

// 7.3.13 Attribute Sequence Params Syntax
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
  bool    getAttributeSmoothingParamsPresentFlag() { return attributeSmoothingParamsPresentFlag_; }
  bool    getAttributeScaleParamsPresentFlag() { return attributeScaleParamsPresentFlag_; }
  bool    getAttributeOffsetParamsPresentFlag() { return attributeOffsetParamsPresentFlag_; }
  uint8_t getAttributeSmoothingRadius() { return attributeSmoothingRadius_; }
  uint8_t getAttributeSmoothingNeighbourCount() { return attributeSmoothingNeighbourCount_; }
  uint8_t getAttributeSmoothingRadius2BoundaryDetection() {
    return attributeSmoothingRadius2BoundaryDetection_;
  }
  uint8_t getAttributeSmoothingThreshold() { return attributeSmoothingThreshold_; }
  uint32_t getAttributeSmoothingThresholdLocalEntropy() {
    return attributeSmoothingThresholdLocalEntropy_;
  }
  std::vector<uint32_t>& getAttributeScaleParams() { return attributeScale_; }
  std::vector<int32_t>&  getAttributeOffsetParams() { return attributeOffset_; }

  void setAttributeSmoothingParamsPresentFlag( bool flag ) {
    attributeSmoothingParamsPresentFlag_ = flag;
  }
  void setAttributeScaleParamsPresentFlag( bool flag ) { attributeScaleParamsPresentFlag_ = flag; }
  void setAttributeOffsetParamsPresentFlag( bool flag ) {
    attributeOffsetParamsPresentFlag_ = flag;
  }
  void setAttributeSmoothingRadius( uint8_t flag ) { attributeSmoothingRadius_ = flag; }
  void setAttributeSmoothingNeighbourCount( uint8_t flag ) {
    attributeSmoothingNeighbourCount_ = flag;
  }
  void setAttributeSmoothingRadius2BoundaryDetection( uint8_t flag ) {
    attributeSmoothingRadius2BoundaryDetection_ = flag;
  }
  void setAttributeSmoothingThreshold( uint8_t threshold ) {
    attributeSmoothingThreshold_ = threshold;
  }
  void setAttributeSmoothingThresholdLocalEntropy( uint32_t localEntropy ) {
    attributeSmoothingThresholdLocalEntropy_ = localEntropy;
  }
  void setAttributeScaleParams( std::vector<uint32_t> params ) { attributeScale_ = params; }
  void setAttributeScaleParam( size_t index, uint32_t scale ) { attributeScale_[index] = scale; }
  void setAttributeOffsetParams( std::vector<int32_t> params ) { attributeOffset_ = params; }
  void setAttributeOffsetParam( size_t index, uint32_t offset ) {
    attributeOffset_[index] = offset;
  }

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

// 7.3.12 Attribute Parameter Set Syntax
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

  uint8_t getAttributeTypeId() { return attributeTypeId_; }
  uint8_t getAttributeDimensionMinus1() { return attributeDimensionMinus1_; }
  uint8_t getAttributeCodecId() { return attributeCodecId_; }
  uint8_t getPcmAttributeCodecId() { return pcmAttributeCodecId_; }
  bool    getAttributeParamsEnabledFlag() { return attributeParamsEnabledFlag_; }
  bool    getAttributePatchParamsEnabledFlag() { return attributePatchParamsEnabledFlag_; }
  bool getAttributePatchScaleParamsEnabledFlag() { return attributePatchScaleParamsEnabledFlag_; }
  bool getAttributePatchOffsetParamsEnabledFlag() { return attributePatchOffsetParamsEnabledFlag_; }
  AttributeSequenceParams& getAttributeSequenceParams() { return attributeSequenceParams_; }

  void setAttributeTypeId( uint8_t typeId ) { attributeTypeId_ = typeId; }
  void setAttributeDimensionMinus1( uint8_t dimension ) { attributeDimensionMinus1_ = dimension; }
  void setAttributeCodecId( uint8_t codecIdx ) { attributeCodecId_ = codecIdx; }
  void setPcmAttributeCodecId( uint8_t codecIdx ) { pcmAttributeCodecId_ = codecIdx; }
  void setAttributeParamsEnabledFlag( bool flag ) { attributeParamsEnabledFlag_ = flag; }
  void setAttributePatchParamsEnabledFlag( bool flag ) { attributePatchParamsEnabledFlag_ = flag; }
  void setAttributePatchScaleParamsEnabledFlag( bool flag ) {
    attributePatchScaleParamsEnabledFlag_ = flag;
  }
  void setAttributePatchOffsetParamsEnabledFlag( bool flag ) {
    attributePatchOffsetParamsEnabledFlag_ = flag;
  }
  void setAttributeSequenceParams( AttributeSequenceParams params ) {
    attributeSequenceParams_ = params;
  }

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

// 7.3.11 Geometry Sequence Params Syntax
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
    for ( size_t d = 0; d < 3; d++ ) {
      geometryScaleOnAxis_[d]    = 0;
      geometryOffsetOnAxis_[d]   = 0;
      geometryRotationOnAxis_[d] = 0;
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
  int32_t  getGeometryRotationOnAxis( int index ) { return geometryRotationOnAxis_[index]; }
  uint16_t getGeometryPointSizeInfo() { return geometryPointSizeInfo_; }
  uint32_t getGeometryPointShapeInfo() { return geometryPointShapeInfo_; }

  void setGeometrySmoothingParamsPresentFlag( bool flag ) {
    geometrySmoothingParamsPresentFlag_ = flag;
  }
  void setGeometryScaleParamsPresentFlag( bool flag ) { geometryScaleParamsPresentFlag_ = flag; }
  void setGeometryOffsetParamsPresentFlag( bool flag ) { geometryOffsetParamsPresentFlag_ = flag; }
  void setGeometryRotationParamsPresentFlag( bool flag ) {
    geometryRotationParamsPresentFlag_ = flag;
  }
  void setGeometryPointSizeInfoPresentFlag( bool flag ) {
    geometryPointSizeInfoPresentFlag_ = flag;
  }
  void setGeometryPointShapeInfoPresentFlag( bool flag ) {
    geometryPointShapeInfoPresentFlag_ = flag;
  }
  void setGeometrySmoothingEnabledFlag( bool flag ) { geometrySmoothingEnableFlag_ = flag; }
  void setGeometrySmoothingGridSize( uint8_t gridSize ) { geometrySmoothingGridSize_ = gridSize; }
  void setGeometrySmoothingThreshold( uint8_t threshold ) {
    geometrySmoothingThreshold_ = threshold;
  }
  void setGeometryScaleOnAxis( int index, uint32_t scale ) { geometryScaleOnAxis_[index] = scale; }
  void setGeometryOffsetOnAxis( int index, int32_t offset ) {
    geometryOffsetOnAxis_[index] = offset;
  }
  void setGeometryRotationOnAxis( int index, int32_t rotation ) {
    geometryRotationOnAxis_[index] = rotation;
  }
  void setGeometryPointSizeInfo( uint16_t pointSize ) { geometryPointSizeInfo_ = pointSize; }
  void setGeometryPointShapeInfo( uint32_t pointShape ) { geometryPointShapeInfo_ = pointShape; }

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
  int32_t  geometryRotationOnAxis_[3];  
  uint16_t geometryPointSizeInfo_;      
  uint32_t geometryPointShapeInfo_;     
};

// 7.3.10 Geometry Parameter Set Syntax
class GeometryParameterSet {
 public:
  GeometryParameterSet() :
      geometryCodecId_( 0 ),
      geometryNominal2dBitdepthMinus1_( 0 ),
      geometry3dCoordinatesBitdepthMinus1_( 0 ),
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
  bool getGeometryPatchOffsetParamsEnabledFlag() { return geometryPatchOffsetParamsEnabledFlag_; }
  bool getGeometryPatchRotationParamsEnabledFlag() {
    return geometryPatchRotationParamsEnabledFlag_;
  }
  bool getGeometryPatchPointSizeInfoEnabledFlag() { return geometryPatchPointSizeInfoEnabledFlag_; }
  bool getGeometryPatchPointShapeInfoEnabledFlag() {
    return geometryPatchPointShapeInfoEnabledFlag_;
  }
  GeometrySequenceParams& getGeometrySequenceParams() { return geometrySequenceParams_; }

  void setGeometryCodecId( uint8_t codecId ) { geometryCodecId_ = codecId; }
  void setGeometryNominal2dBitdepthMinus1( uint8_t bitdepth ) {
    geometryNominal2dBitdepthMinus1_ = bitdepth;
  }
  void setGeometry3dCoordinatesBitdepthMinus1( uint8_t bitdepth ) {
    geometry3dCoordinatesBitdepthMinus1_ = bitdepth;
  }
  void setPcmGeometryCodecId( uint8_t codecId ) { pcmGeometryCodecId_ = codecId; }
  void setGeometryParamsEnabledFlag( bool flag ) { geometryParamsEnabledFlag_ = flag; }
  void setGeometryPatchParamsEnabledFlag( bool flag ) { geometryPatchParamsEnabledFlag_ = flag; }
  void setGeometryPatchScaleParamsEnabledFlag( bool flag ) {
    geometryPatchScaleParamsEnabledFlag_ = flag;
  }
  void setGeometryPatchOffsetParamsEnabledFlag( bool flag ) {
    geometryPatchOffsetParamsEnabledFlag_ = flag;
  }
  void setGeometryPatchRotationParamsEnabledFlag( bool flag ) {
    geometryPatchRotationParamsEnabledFlag_ = flag;
  }
  void setGeometryPatchPointSizeInfoEnabledFlag( bool flag ) {
    geometryPatchPointSizeInfoEnabledFlag_ = flag;
  }
  void setGeometryPatchPointShapeInfoEnabledFlag( bool flag ) {
    geometryPatchPointShapeInfoEnabledFlag_ = flag;
  }
  void setGeometrySequenceParams( GeometrySequenceParams params ) {
    geometrySequenceParams_ = params;
  }

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

// 7.3.9 Occupancy Parameter Set Syntax
class OccupancyParameterSet {
 public:
  OccupancyParameterSet() : occupancyCodecId_( 0 ), occupancyPackingBlockSize_( 0 ) {}
  OccupancyParameterSet& operator=( const OccupancyParameterSet& ) = default;

  void init( uint8_t codecId, uint8_t packingBlockSize ) {
    occupancyCodecId_          = codecId;
    occupancyPackingBlockSize_ = packingBlockSize;
  }

  uint8_t getOccupancyCodecId() { return occupancyCodecId_; }
  uint8_t getOccupancyPackingBlockSize() { return occupancyPackingBlockSize_; }
  void    setOccupancyCodecId( uint8_t codecIdx ) { occupancyCodecId_ = codecIdx; }
  void setOccupancyPackingBlockSize( uint8_t blocksize ) { occupancyPackingBlockSize_ = blocksize; }

 private:
  uint8_t occupancyCodecId_;
  uint8_t occupancyPackingBlockSize_;
};

// 7.3.8 Profile, Tiere and Level Syntax
class ProfileTierLevel {
 public:
  ProfileTierLevel() : tierFlag_( false ), profileIdc_( 0 ), levelIdc_( 0 ) {}
  ProfileTierLevel& operator=( const ProfileTierLevel& ) = default;

  uint8_t getProfileIdc() { return profileIdc_; }
  bool    getTierFlag() { return tierFlag_; }
  uint8_t getLevelIdc() { return levelIdc_; }

  void setTierFlag( bool tier ) { tierFlag_ = tier; }
  void setProfileIdc( uint8_t profileIdx ) { profileIdc_ = profileIdx; }
  void setLevelIdc( uint8_t levelIdx ) { levelIdc_ = levelIdx; }

 private:
  bool    tierFlag_;
  uint8_t profileIdc_;
  uint8_t levelIdc_;
};

// 7.3.6  Sequence parameter set syntax
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
      patchSequenceOrientationEnabledFlag_( false ),
      patchInterPredictionEnabledFlag_( false ),
      pixelDeinterleavingFlag_( false ),
      pointLocalReconstructionEnabledFlag_( false ),
      removeDuplicatePointEnabledFlag_( false ) {
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
                              bool     removeDuplicatePointEnabledFlag ) {
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
    patchSequenceOrientationEnabledFlag_ = patchSequenceOrientationEnabledFlag;
    patchInterPredictionEnabledFlag_     = patchInterPredictionEnabledFlag;
    pixelDeinterleavingFlag_             = pixelDeinterleavingFlag;
    pointLocalReconstructionEnabledFlag_ = pointLocalReconstructionEnabledFlag;
    removeDuplicatePointEnabledFlag_     = removeDuplicatePointEnabledFlag;
    allocateAttributeParameterSets();
  }

  uint32_t getSequenceParameterSetId() { return sequenceParameterSetId_; }
  uint16_t getFrameWidth() { return frameWidth_; }
  uint16_t getFrameHeight() { return frameHeight_; }
  uint16_t getAvgFrameRate() { return avgFrameRate_; }
  uint32_t getLayerCountMinus1() { return layerCountMinus1_; }
  uint16_t getAttributeCount() { return attributeCount_; }
  bool     getAvgFrameRatePresentFlag() { return avgFrameRatePresentFlag_; }
  bool     getEnhancedOccupancyMapForDepthFlag() { return enhancedOccupancyMapForDepthFlag_; }
  bool     getMultipleLayerStreamsPresentFlag() { return multipleLayerStreamsPresentFlag_; }
  bool     getPcmPatchEnabledFlag() { return pcmPatchEnabledFlag_; }
  bool     getPcmSeparateVideoPresentFlag() { return pcmSeparateVideoPresentFlag_; }
  bool     getPatchSequenceOrientationEnabledFlag() { return patchSequenceOrientationEnabledFlag_; }
  bool     getPatchInterPredictionEnabledFlag() { return patchInterPredictionEnabledFlag_; }
  bool     getPixelDeinterleavingFlag() { return pixelDeinterleavingFlag_; }
  bool     getPointLocalReconstructionEnabledFlag() { return pointLocalReconstructionEnabledFlag_; }
  bool     getRemoveDuplicatePointEnabledFlag() { return removeDuplicatePointEnabledFlag_; }
  std::vector<size_t>& getLayerPredictorIndexDiff() { return layerPredictorIndexDiff_; }
  std::vector<bool>& getLayerAbsoluteCodingEnabledFlag() { return layerAbsoluteCodingEnabledFlag_; }
  ProfileTierLevel&  getProfileTierLevel() { return profileTierLevel_; }
  GeometryParameterSet&               getGeometryParameterSet() { return geometryParameterSet_; }
  OccupancyParameterSet&              getOccupancyParameterSet() { return occupancyParameterSet_; }
  std::vector<AttributeParameterSet>& getAttributeParameterSets() {
    return attributeParameterSets_;
  }
  AttributeParameterSet& getAttributeParameterSet( size_t attributeIndex ) {
    return attributeParameterSets_[attributeIndex];
  }

  void setSequenceParameterSetId( uint32_t idx ) { sequenceParameterSetId_ = idx; }
  void setFrameWidth( uint16_t width ) { frameWidth_ = width; }
  void setFrameHeight( uint16_t height ) { frameHeight_ = height; }
  void setAvgFrameRate( uint16_t frameRate ) { avgFrameRate_ = frameRate; }
  void setLayerCountMinus1( uint32_t countMinus1 ) { layerCountMinus1_ = countMinus1; }
  void setAttributeCount( size_t count ) { attributeCount_ = count; }
  void setAvgFrameRatePresentFlag( bool flag ) { avgFrameRatePresentFlag_ = flag; }
  void setEnhancedOccupancyMapForDepthFlag( bool flag ) {
    enhancedOccupancyMapForDepthFlag_ = flag;
  }
  void setMultipleLayerStreamsPresentFlag( bool flag ) { multipleLayerStreamsPresentFlag_ = flag; }
  void setPcmPatchEnabledFlag( bool flag ) { pcmPatchEnabledFlag_ = flag; }
  void setPcmSeparateVideoPresentFlag( bool flag ) { pcmSeparateVideoPresentFlag_ = flag; }
  void setPatchSequenceOrientationEnabledFlag( bool flag ) {
    patchSequenceOrientationEnabledFlag_ = flag;
  }
  void setPatchInterPredictionEnabledFlag( bool flag ) { patchInterPredictionEnabledFlag_ = flag; }
  void setPixelDeinterleavingFlag( bool flag ) { pixelDeinterleavingFlag_ = flag; }
  void setPointLocalReconstructionEnabledFlag( bool flag ) {
    pointLocalReconstructionEnabledFlag_ = flag;
  }
  void setRemoveDuplicatePointEnabledFlag( bool flag ) { removeDuplicatePointEnabledFlag_ = flag; }
  void setLayerPredictorIndexDiff( std::vector<size_t> predictorIdxDiffList ) {
    layerPredictorIndexDiff_ = predictorIdxDiffList;
  }
  void setLayerAbsoluteCodingEnabledFlag( std::vector<bool> flagList ) {
    layerAbsoluteCodingEnabledFlag_ = flagList;
  }
  void setProfileTierLevel( ProfileTierLevel level ) { profileTierLevel_ = level; }
  void setGeometryParameterSet( GeometryParameterSet params ) { geometryParameterSet_ = params; }
  void setOccupancyParameterSet( OccupancyParameterSet params ) { occupancyParameterSet_ = params; }
  void setAttributeParameterSets( std::vector<AttributeParameterSet> paramsList ) {
    attributeParameterSets_ = paramsList;
  }

  void allocateAttributeParameterSets() { attributeParameterSets_.resize( attributeCount_ ); }

 private:
  uint32_t              sequenceParameterSetId_;
  uint16_t              frameWidth_;
  uint16_t              frameHeight_;
  uint16_t              avgFrameRate_;
  uint32_t              layerCountMinus1_; 
  uint16_t              attributeCount_;
  bool                  avgFrameRatePresentFlag_;
  bool                  enhancedOccupancyMapForDepthFlag_;
  bool                  multipleLayerStreamsPresentFlag_;
  bool                  pcmPatchEnabledFlag_;
  bool                  pcmSeparateVideoPresentFlag_;
  bool                  patchSequenceOrientationEnabledFlag_;
  bool                  patchInterPredictionEnabledFlag_;
  bool                  pixelDeinterleavingFlag_; 
  bool                  pointLocalReconstructionEnabledFlag_;
  bool                  removeDuplicatePointEnabledFlag_;
  std::vector<bool>     layerAbsoluteCodingEnabledFlag_;  // layerCount_ size
  std::vector<size_t>   layerPredictorIndexDiff_;         // layerCount_ size
  ProfileTierLevel      profileTierLevel_;
  GeometryParameterSet  geometryParameterSet_;
  OccupancyParameterSet occupancyParameterSet_;
  std::vector<AttributeParameterSet> attributeParameterSets_;  // attributeCount_ size
};

// 7.3.4  PCM separate video data syntax
class VPCCParameterSet {
 public:
  VPCCParameterSet() :
      pcmVideoFlag_( false ),
      sequenceParamterSetId_( 0 ),
      attributeIndex_( 0 ),
      layerIndex_( 0 ) {}
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

class PCCContext {
 public:
  PCCContext();
  ~PCCContext();

  std::vector<PCCFrameContext>::iterator begin() { return frames_.begin(); }
  std::vector<PCCFrameContext>::iterator end() { return frames_.end(); }

  void resize( size_t size );

  const size_t                  size() { return frames_.size(); }
  std::vector<PCCFrameContext>& getFrames() { return frames_; }
  PCCFrameContext&              operator[]( int i ) { return frames_[i]; }

  PCCVideoGeometry&     getVideoGeometry() { return videoGeometry_; }
  PCCVideoGeometry&     getVideoGeometryD1() { return videoGeometryD1_; }
  PCCVideoTexture&      getVideoTexture() { return videoTexture_; }
  PCCVideoOccupancyMap& getVideoOccupancyMap() { return videoOccupancyMap_; }
  PCCVideoGeometry&     getVideoMPsGeometry() { return videoMPsGeometry_; }
  PCCVideoTexture&      getVideoMPsTexture() { return videoMPsTexture_; }

  // deprecated, must be removed:
  bool&                    getLosslessGeo444() { return losslessGeo444_; }
  bool&                    getLosslessGeo() { return losslessGeo_; }
  bool&                    getLosslessTexture() { return losslessTexture_; }
  uint8_t&                 getOccupancyPrecision() { return occupancyPrecision_; }
  bool&                    getGridSmoothing() { return gridSmoothing_; }
  bool&                    getNoAttributes() { return noAttributes_; }
  bool&                    getAbsoluteD1() { return absoluteD1_; }
  bool&                    getBinArithCoding() { return binArithCoding_; }
  float&                   getModelScale() { return modelScale_; }
  PCCVector3<float>&       getModelOrigin() { return modelOrigin_; }
  bool&                    getImproveEDD() { return improveEDD_; }
  bool&                    getDeltaCoding() { return deltaCoding_; }
  bool&                    getSixDirectionMode() { return sixDirectionMode_; }
  bool&                    getUseAdditionalPointsPatch() { return useAdditionalPointsPatch_; }
  uint8_t&                 getMinLevel() { return minLevel_; }
  bool&                    getGlobalPatchAllocation() { return globalPatchAllocation_; }
  bool&                    getUse3dmc() { return use3dmc_; }
  size_t&                  getMPGeoWidth() { return MPGeoWidth_; }
  size_t&                  getMPGeoHeight() { return MPGeoHeight_; }
  size_t&                  getMPAttWidth() { return MPAttWidth_; }
  size_t&                  getMPAttHeight() { return MPAttHeight_; }
  PCCMetadata&             getGOFLevelMetadata() { return gofLevelMetadata_; }
  std::vector<SubContext>& getSubContexts() { return subContexts_; }
  std::vector<unionPatch>& getUnionPatch() { return unionPatch_; }
  //~ deprecated, must be removed

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

  void                  allocOneLayerData( const size_t occupancyResolution );
  void                  printVideoBitstream();
  void                  printBlockToPatch( const size_t occupancyResolution );
  VPCCParameterSet&     getVPCC() { return vpccParameterSet_; }
  SequenceParameterSet& getSps() {
    for ( auto& sps : sequenceParameterSets_ ) {
      if ( vpccParameterSet_.getSequenceParameterSetId() == sps.getSequenceParameterSetId() ) {
        return sps;
      }
    }
    fprintf( stderr, "Error: can't find sps[ %lc ] \n",
             vpccParameterSet_.getSequenceParameterSetId() );
    exit( -1 );
  }

  PatchSequenceDataUnit& getPatchSequenceDataUnit() { return patchSequenceDataUnit_; }

 private:
  std::vector<PCCFrameContext> frames_;
  PCCVideoGeometry             videoGeometry_;
  PCCVideoGeometry             videoGeometryD1_;
  PCCVideoTexture              videoTexture_;
  PCCVideoOccupancyMap         videoOccupancyMap_;
  PCCVideoGeometry             videoMPsGeometry_;
  PCCVideoTexture              videoMPsTexture_;

  // deprecated, must be removed:
  bool losslessGeo444_;
  bool losslessGeo_;
  bool losslessTexture_;
  bool gridSmoothing_;
  bool absoluteD1_;
  bool binArithCoding_;
  bool improveEDD_;
  bool deltaCoding_;
  bool sixDirectionMode_;
  bool useAdditionalPointsPatch_;
  bool globalPatchAllocation_;
  bool use3dmc_;
  bool noAttributes_;

  uint8_t                        minLevel_;
  uint8_t                        occupancyPrecision_;
  size_t                         MPGeoWidth_;
  size_t                         MPGeoHeight_;
  size_t                         MPAttWidth_;
  size_t                         MPAttHeight_;
  float                          modelScale_;
  PCCVector3<float>              modelOrigin_;
  PCCMetadata                    gofLevelMetadata_;
  std::vector<PCCVideoBitstream> videoBitstream_;
  std::vector<SubContext>        subContexts_;
  std::vector<unionPatch>        unionPatch_;
  //~ deprecated, must be removed

  VPCCParameterSet                  vpccParameterSet_;
  PatchSequenceDataUnit             patchSequenceDataUnit_;
  std::vector<SequenceParameterSet> sequenceParameterSets_;
};
};  // namespace pcc

#endif /* PCCContext_h */
