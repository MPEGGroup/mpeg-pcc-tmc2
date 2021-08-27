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
#include "PCCMath.h"
#include "PCCVideo.h"
#include "PCCBitstreamCommon.h"
#include "PCCHighLevelSyntax.h"
#include <map>

namespace pcc {

class PCCGroupOfFrames;
class PCCFrameContext;
class PCCAtlasFrameContext;
typedef PCCVideo<uint8_t, 3>  PCCVideoOccupancyMap;
typedef PCCVideo<uint16_t, 3> PCCVideoGeometry;
typedef PCCVideo<uint16_t, 3> PCCVideoAttribute;

class PCCPatch;
typedef std::map<size_t, PCCPatch>                 unionPatch;     // [TrackIndex, PatchUnion]
typedef std::pair<size_t, size_t>                  GlobalPatch;    // [FrameIndex, PatchIndex]
typedef std::map<size_t, std::vector<GlobalPatch>> GlobalPatches;  // [TrackIndex, <GlobalPatch>]
typedef std::pair<size_t, size_t>                  SubContext;     // [start, end)

class PCCAtlasContext {
 public:
  PCCAtlasContext();
  ~PCCAtlasContext();

  // atlas related functions
  void   setAtlasIndex( size_t atlIdx ) { atlasIndex_ = atlIdx; }
  size_t getAtlasIndex() { return atlasIndex_; }

  // frame context related functions
  std::vector<PCCAtlasFrameContext>::iterator begin() { return frameContexts_.begin(); }
  std::vector<PCCAtlasFrameContext>::iterator end() { return frameContexts_.end(); }
  void                                        resize( size_t size, size_t frameStart = 0 );
  const size_t                                size() { return frameContexts_.size(); }
  PCCAtlasFrameContext&                       operator[]( int index ) { return frameContexts_[index]; }
  std::vector<PCCAtlasFrameContext>&          getFrameContexts() { return frameContexts_; }
  PCCAtlasFrameContext&                       getFrameContext( int16_t index ) { return frameContexts_[index]; }
  std::vector<std::pair<size_t, size_t>>&     getFramesInAFPS() { return framesInAFPS_; }
  size_t                                      calculateAFOCLsb( size_t frameOrder );
  void                                        allocOneLayerData();
  void                                        printBlockToPatch( const size_t occupancyResolution );
  void   setLog2MaxAtlasFrameOrderCntLsb( size_t value ) { log2MaxAtlasFrameOrderCntLsb_ = value; }
  size_t getLog2MaxAtlasFrameOrderCntLsb() { return log2MaxAtlasFrameOrderCntLsb_; }

  // video related functions
  void                            allocateVideoFrames( PCCHighLevelSyntax& syntax, size_t numFrames );
  void                            clearVideoFrames();
  PCCVideoOccupancyMap&           getVideoOccupancyMap() { return occFrames_; }
  std::vector<PCCVideoGeometry>&  getVideoGeometryMultiple() { return geoFrames_; }
  PCCVideoGeometry&               getVideoGeometryMultiple( size_t index ) { return geoFrames_[index]; }
  PCCVideoGeometry&               getVideoAuxGeometry() { return geoAuxFrames_; }
  std::vector<PCCVideoAttribute>& getVideoAttributeMultiple( size_t attrIdx, size_t partIdx ) {
    return attrFrames_[attrIdx][partIdx];
  }
  PCCVideoAttribute& getVideoAttributeMultiple( size_t attrIdx, size_t partIdx, size_t index ) {
    return attrFrames_[attrIdx][partIdx][index];
  }
  PCCVideoAttribute& getVideoAuxAttribute( size_t attrIdx, size_t partIdx ) { return attrAuxFrames_[attrIdx][partIdx]; }

  // GPA related functions
  std::vector<SubContext>& getSubContexts() { return subContexts_; }
  std::vector<unionPatch>& getUnionPatch() { return unionPatch_; }

 private:
  size_t                                                     atlasIndex_;
  size_t                                                     log2MaxAtlasFrameOrderCntLsb_;
  std::vector<PCCAtlasFrameContext>                          frameContexts_;
  std::vector<std::pair<size_t, size_t>>                     framesInAFPS_;
  PCCVideoOccupancyMap                                       occFrames_;
  std::vector<size_t>                                        occBitdepth_;
  std::vector<size_t>                                        occWidth_;
  std::vector<size_t>                                        occHeight_;
  std::vector<PCCVideoGeometry>                              geoFrames_;
  std::vector<std::vector<size_t>>                           geoBitdepth_;
  std::vector<std::vector<size_t>>                           geoWidth_;
  std::vector<std::vector<size_t>>                           geoHeight_;
  PCCVideoGeometry                                           geoAuxFrames_;
  std::vector<std::vector<std::vector<PCCVideoAttribute>>>   attrFrames_;
  std::vector<std::vector<std::vector<std::vector<size_t>>>> attrBitdepth_;
  std::vector<std::vector<std::vector<std::vector<size_t>>>> attrWidth_;
  std::vector<std::vector<std::vector<std::vector<size_t>>>> attrHeight_;
  std::vector<std::vector<PCCVideoAttribute>>                attrAuxFrames_;
  std::vector<SubContext>                                    subContexts_;
  std::vector<unionPatch>                                    unionPatch_;
};

// used for context handling
class PCCContext : public PCCHighLevelSyntax {
 public:
  PCCContext();
  ~PCCContext();

  // Atlas related functions
  const size_t                  sizeAtlas() { return atlasContexts_.size(); }
  std::vector<PCCAtlasContext>& getAtlases() { return atlasContexts_; }
  PCCAtlasContext&              getAtlas( int16_t index ) { return atlasContexts_[index]; }
  void                          resizeAtlas( size_t size );
  void                          setAtlasIndex( size_t atlId ) { atlasIndex_ = atlId; }
  size_t                        getAtlasIndex() { return atlasIndex_; }

  // video related functions (if not argument is passed, assumptions are made
  // occupancy map
  PCCVideoOccupancyMap& getVideoOccupancyMap( size_t atlId ) { return atlasContexts_[atlId].getVideoOccupancyMap(); }
  PCCVideoOccupancyMap& getVideoOccupancyMap() { return atlasContexts_[atlasIndex_].getVideoOccupancyMap(); }
  // geometry
  std::vector<PCCVideoGeometry>& getVideoGeometryMultiple() {
    return atlasContexts_[atlasIndex_].getVideoGeometryMultiple();
  }
  PCCVideoGeometry& getVideoGeometryMultiple( size_t index ) {
    return atlasContexts_[atlasIndex_].getVideoGeometryMultiple( index );
  }
  PCCVideoGeometry& getVideoRawPointsGeometry( size_t atlId ) { return atlasContexts_[atlId].getVideoAuxGeometry(); }
  PCCVideoGeometry& getVideoRawPointsGeometry() { return atlasContexts_[atlasIndex_].getVideoAuxGeometry(); }
  // attributes
  std::vector<PCCVideoAttribute>& getVideoAttributesMultiple( size_t atlId, size_t attrIdx, size_t partIdx ) {
    return atlasContexts_[atlId].getVideoAttributeMultiple( attrIdx, partIdx );
  }
  std::vector<PCCVideoAttribute>& getVideoAttributesMultiple() {
    return atlasContexts_[atlasIndex_].getVideoAttributeMultiple( 0, 0 );
  }
  PCCVideoAttribute& getVideoAttributesMultiple( size_t index ) {
    return atlasContexts_[atlasIndex_].getVideoAttributeMultiple( 0, 0, index );
  }
  PCCVideoAttribute& getVideoRawPointsAttribute( size_t atlId, size_t attrIdx, size_t partIdx ) {
    return atlasContexts_[atlId].getVideoAuxAttribute( attrIdx, partIdx );
  }
  PCCVideoAttribute& getVideoRawPointsAttribute() { return atlasContexts_[atlasIndex_].getVideoAuxAttribute( 0, 0 ); }

  // fame context related functions
  std::vector<PCCAtlasFrameContext>::iterator begin() { return atlasContexts_[atlasIndex_].getFrameContexts().begin(); }
  std::vector<PCCAtlasFrameContext>::iterator end() { return atlasContexts_[atlasIndex_].getFrameContexts().end(); }
  const size_t                                size() { return atlasContexts_[atlasIndex_].getFrameContexts().size(); }
  std::vector<PCCAtlasFrameContext>&          getFrames() { return atlasContexts_[atlasIndex_].getFrameContexts(); }
  PCCAtlasFrameContext& getFrame( int16_t index ) { return atlasContexts_[atlasIndex_].getFrameContext( index ); }
  std::vector<std::pair<size_t, size_t>>& getFramesInAFPS() { return atlasContexts_[atlasIndex_].getFramesInAFPS(); }
  size_t calculateAFOCLsb( size_t frameOrder ) { return atlasContexts_[atlasIndex_].calculateAFOCLsb( frameOrder ); }
  size_t calculateAFOCval( std::vector<AtlasTileLayerRbsp>& atglList, size_t atglIndex );
  PCCAtlasFrameContext& operator[]( int index ) { return atlasContexts_[atlasIndex_].getFrameContexts()[index]; }
  void                  resize( size_t size ) { atlasContexts_[atlasIndex_].resize( size ); };
  void                  allocOneLayerData();
  void                  printBlockToPatch( const size_t occupancyResolution ) {
    atlasContexts_[atlasIndex_].printBlockToPatch( occupancyResolution );
  };
  void setLog2MaxAtlasFrameOrderCntLsb( size_t value ) {
    atlasContexts_[atlasIndex_].setLog2MaxAtlasFrameOrderCntLsb( value );
  }
  size_t getLog2MaxAtlasFrameOrderCntLsb() { return atlasContexts_[atlasIndex_].getLog2MaxAtlasFrameOrderCntLsb(); }

  // GPA related functions
  std::vector<SubContext>& getSubContexts() { return atlasContexts_[atlasIndex_].getSubContexts(); }
  std::vector<unionPatch>& getUnionPatch() { return atlasContexts_[atlasIndex_].getUnionPatch(); }

  PCCVector3<float>& getModelOrigin() { return modelOrigin_; }
  void               setModelOrigin( PCCVector3<float>& value ) { modelOrigin_ = value; }
  float              getModelScale() { return modelScale_; }
  void               setModelScale( float value ) { modelScale_ = value; }

  std::vector<uint8_t> computeMD5( uint8_t* byteString, size_t size );
  uint16_t             computeCRC( uint8_t* byteString, size_t size );
  uint32_t             computeCheckSum( uint8_t* byteString, size_t size );

 private:
  PCCVector3<float>            modelOrigin_;
  float                        modelScale_;
  std::vector<PCCAtlasContext> atlasContexts_;
  size_t                       atlasIndex_;
};
};  // namespace pcc

#endif /* PCCContext_h */
