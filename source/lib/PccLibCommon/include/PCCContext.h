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
// #include "PCCBitstream.h"
// #include "PCCVideoBitstream.h"
#include "PCCHighLevelSyntax.h"
#include <map>

namespace pcc {

class PCCGroupOfFrames;
class PCCFrameContext;
typedef pcc::PCCVideo<uint16_t, 3> PCCVideoAttributes; 
typedef pcc::PCCVideo<uint16_t, 3> PCCVideoTexture; 
typedef pcc::PCCVideo<uint16_t, 3> PCCVideoGeometry;
typedef pcc::PCCVideo<uint8_t, 3>  PCCVideoOccupancyMap;

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
	void setAtlasIndex(size_t atlIdx) { atlasIndex_ = atlIdx; }
	size_t getAtlasIndex() { return atlasIndex_; }

	// frame context related functions
  std::vector<PCCFrameContext>::iterator begin() { return frameContexts_.begin(); }
  std::vector<PCCFrameContext>::iterator end() { return frameContexts_.end(); }
	void                                   resize( size_t size, size_t frameStart = 0 );
  const size_t                           size() { return frameContexts_.size(); }
  PCCFrameContext&                       operator[]( int index ) { return frameContexts_[index]; }
  std::vector<PCCFrameContext>&          getFrameContexts() { return frameContexts_; }
  PCCFrameContext&                       getFrameContext( int16_t index ) { return frameContexts_[index]; }
  void                                   allocOneLayerData();
  void                                   printBlockToPatch( const size_t occupancyResolution );

  // video related functions
	void                                   allocateVideoFrames( PCCHighLevelSyntax& syntax, size_t numFrames );
	void                                   clearVideoFrames();
  PCCVideoOccupancyMap&                  getVideoOccupancyMap() { return occFrames_; }
  PCCVideoGeometry&                      getVideoGeometry( size_t mapIdx ) { return geoFrames_[mapIdx]; }
  std::vector<PCCVideoGeometry>&         getVideoGeometryMultiple() { return geoFrames_; }
  PCCVideoGeometry&                      getVideoAuxGeometry() { return geoAuxFrames_; }
  PCCVideoAttributes&                    getVideoAttribute( size_t attrIdx, size_t partIdx, size_t mapIdx ) { return attrFrames_[attrIdx][partIdx][mapIdx]; }
  std::vector<PCCVideoTexture>&          getVideoAttributeMultiple( size_t attrIdx, size_t partIdx ) { return attrFrames_[attrIdx][partIdx]; }
  PCCVideoAttributes&                    getVideoAuxAttribute( size_t attrIdx, size_t partIdx ) { return attrAuxFrames_[attrIdx][partIdx]; }

	// GPA related functions
	std::vector<SubContext>&               getSubContexts() { return subContexts_; }
  std::vector<unionPatch>&               getUnionPatch() { return unionPatch_; }

 private:
  // atlas index
	 size_t atlasIndex_;
  // frame context
  std::vector<PCCFrameContext> frameContexts_;
	// video variables
  // OccFrame[ orderIdx ][ compIdx ][ y ][ x ]
  // where:
  // orderIdx  is the display order index of the decoded occupancy frames
  // compIdx = 0,
  // y = [0, OccHeight [ orderIdx ] - 1 ]
  // x = [0, OccWidth [ orderIdx ] - 1 ]
  // PCCVideoOccupancyMap -> pcc::PCCVideo<uint8_t, 3>
  // PCCVideo -> std::vector<PCCImage<T, N> > frames_;
  // PCCImage -> std::vector<T> channels_[N]
  PCCVideoOccupancyMap          occFrames_;
  // OccBitdepth [ orderIdx ]  
  std::vector<size_t>           occBitdepth_;
  // OccWidth  [ orderIdx ]  
  std::vector<size_t>           occWidth_;
  // OccHeight [ orderIdx ]  
  std::vector<size_t>           occHeight_;
  // GeoFrame [ mapIdx ] [ orderIdx ][ compIdx ][ y ][ x ] 
  // where:
  // mapIdx = [0, vps_map_count_minus1[ atlasIdx ]]
  // orderIdx  is the display order index of the decoded geometry frames
  // compIdx = 0,
  // y = [0, GeoHeight [ mapIdx ] [ orderIdx ] - 1 ]
  // x = [0, GeoWidth [ mapIdx ] [ orderIdx ] - 1 ]
  // std::vector<PCCVideoGeometry>
  // PCCVideoGeometry -> pcc::PCCVideo<uint16_t, 3> 
  // PCCVideo -> std::vector<PCCImage<T, N> > frames_;
  // PCCImage -> std::vector<T> channels_[N]
  std::vector<PCCVideoGeometry>              geoFrames_;
  // GeoBitdepth [ mapIdx ] [ orderIdx ]  
  std::vector<std::vector<size_t>>           geoBitdepth_;
  // GeoWidth [ mapIdx ] [ orderIdx ]  
  std::vector<std::vector<size_t>>           geoWidth_;
  // GeoHeight [ mapIdx ] [ orderIdx ]  
  std::vector<std::vector<size_t>>           geoHeight_;
  // GeoAuxFrame [ orderIdx ][ compIdx ][ y ][ x ] 
  // PCCVideoGeometry -> pcc::PCCVideo<uint16_t, 3> 
  // PCCVideo -> std::vector<PCCImage<T, N> > frames_;
  // PCCImage -> std::vector<T> channels_[N]
  PCCVideoGeometry              geoAuxFrames_;
  // AttrFrame [ attrIdx ] [ partIdx ] [ mapIdx ] [ orderIdx ][ compIdx ][ y ][ x ]
  // attrIdx = [0, ai_attribute_count[ atlasIdx ] - 1]
  // partIdx = [0, ai_attribute_dimension_partitions_minus1[ atlasIdx ][ attrIdx ]]
  // mapIdx = [0, vps_map_count_minus1[ atlasIdx ]]
  // orderIdx  is the display order index of the decoded attribute frames
  // compIdx = = [0, ai_attribute_partition_channels_minus1[ atlasIdx ][ attrIdx ][ partIdx ]]
  // y = [0, AttrHeight [ mapIdx ] [ orderIdx ] - 1 ]
  // x = [0, AttrWidth [ mapIdx ] [ orderIdx ] - 1 ]
  // std::vector<std::vector<std::vector<PCCVideoAttributes>>>
  // PCCVideoAttributes -> pcc::PCCVideo<uint16_t, 3> 
  // PCCVideo -> std::vector<PCCImage<T, N> > frames_;
  // PCCImage -> std::vector<T> channels_[N]
  std::vector<std::vector<std::vector<PCCVideoAttributes>>>              attrFrames_;
  // AttrBitdepth [ attrIdx ] [ partIdx ] [ mapIdx ] [ orderIdx ]  
  std::vector<std::vector<std::vector<std::vector<size_t>>>>             attrBitdepth_;
  // AttrWidth [ attrIdx ] [ partIdx ] [ mapIdx ] [ orderIdx ]  
  std::vector<std::vector<std::vector<std::vector<size_t>>>>             attrWidth_;
  // AttrHeight [ attrIdx ] [ partIdx ] [ mapIdx ] [ orderIdx ]  
  std::vector<std::vector<std::vector<std::vector<size_t>>>>             attrHeight_;
  // AttrAuxFrame [ attrIdx ] [ partIdx ] [ orderIdx ][ compIdx ][ y ][ x ]
  // PCCVideoAttributes -> std::vector<std::vector<pcc::PCCVideo<uint16_t, 3>>> 
  // PCCVideo -> std::vector<PCCImage<T, N> > frames_;
  // PCCImage -> std::vector<T> channels_[N]
  std::vector<std::vector<PCCVideoAttributes>>                           attrAuxFrames_;
	// GPA variables
	std::vector<SubContext>      subContexts_;
  std::vector<unionPatch>      unionPatch_;
};

// used for context handling
class PCCContext : public PCCHighLevelSyntax {
 public:
  PCCContext();
  ~PCCContext();

	//Atlas related functions
	const size_t                  sizeAtlas() { return atlasContexts_.size(); }
  std::vector<PCCAtlasContext>& getAtlases() { return atlasContexts_; }
  PCCAtlasContext&              getAtlas( int16_t index ) { return atlasContexts_[index]; }
	void                          resizeAtlas(size_t size);
	void                          setAtlasIndex(size_t atlId) { atlasIndex_ = atlId; }
	size_t                        getAtlasIndex() { return atlasIndex_; }

	// video related functions (if not argument is passed, assumptions are made
	//occupancy map
  PCCVideoOccupancyMap&         getVideoOccupancyMap(size_t atlId) { return atlasContexts_[atlId].getVideoOccupancyMap(); }
  PCCVideoOccupancyMap&         getVideoOccupancyMap() { return atlasContexts_[atlasIndex_].getVideoOccupancyMap(); }
	//geometry 
  PCCVideoGeometry&             getVideoGeometry(size_t atlId, size_t mapIdx) { return  atlasContexts_[atlId].getVideoGeometry(mapIdx); }
  PCCVideoGeometry&             getVideoGeometry() { return  atlasContexts_[atlasIndex_].getVideoGeometry(0); }
  std::vector<PCCVideoGeometry>& getVideoGeometryMultiple(size_t atlId) { return  atlasContexts_[atlId].getVideoGeometryMultiple(); }
  std::vector<PCCVideoGeometry>& getVideoGeometryMultiple() { return  atlasContexts_[atlasIndex_].getVideoGeometryMultiple(); }
	PCCVideoGeometry&             getVideoMPsGeometry(size_t atlId) { return atlasContexts_[atlId].getVideoAuxGeometry(); }
  PCCVideoGeometry&             getVideoMPsGeometry() { return atlasContexts_[atlasIndex_].getVideoAuxGeometry(); }
	//attributes
  PCCVideoTexture&              getVideoTexture(size_t atlId, size_t attrIdx, size_t mapIdx, size_t partIdx) { return atlasContexts_[atlId].getVideoAttribute(attrIdx,mapIdx,partIdx); }
  PCCVideoTexture&              getVideoTexture() { return atlasContexts_[atlasIndex_].getVideoAttribute(0,0,0); }
  std::vector<PCCVideoTexture>& getVideoTextureMultiple(size_t atlId, size_t attrIdx, size_t partIdx) { return atlasContexts_[atlId].getVideoAttributeMultiple(attrIdx, partIdx); }
  std::vector<PCCVideoTexture>& getVideoTextureMultiple() { return atlasContexts_[atlasIndex_].getVideoAttributeMultiple(0, 0); }
  PCCVideoTexture&              getVideoMPsTexture(size_t atlId, size_t attrIdx, size_t partIdx) { return atlasContexts_[atlId].getVideoAuxAttribute(attrIdx, partIdx); }
  PCCVideoTexture&              getVideoMPsTexture() { return atlasContexts_[atlasIndex_].getVideoAuxAttribute(0, 0); }

	//fame context related functions
  std::vector<PCCFrameContext>::iterator begin() { return atlasContexts_[atlasIndex_].getFrameContexts().begin(); }
  std::vector<PCCFrameContext>::iterator end() { return atlasContexts_[atlasIndex_].getFrameContexts().end(); }
  const size_t                           size() { return atlasContexts_[atlasIndex_].getFrameContexts().size(); }
  std::vector<PCCFrameContext>&          getFrames() { return atlasContexts_[atlasIndex_].getFrameContexts(); }
  PCCFrameContext&                       getFrame( int16_t index ) { return atlasContexts_[atlasIndex_].getFrameContext(index); }
  PCCFrameContext&                       operator[]( int index ) { return atlasContexts_[atlasIndex_].getFrameContexts()[index]; }
	void                                   resize(size_t size) { atlasContexts_[atlasIndex_].resize(size); };
	void                                   allocOneLayerData() { atlasContexts_[atlasIndex_].allocOneLayerData(); };
	void                                   printBlockToPatch(const size_t occupancyResolution) { atlasContexts_[atlasIndex_].printBlockToPatch(occupancyResolution); };

	// GPA related functions
	std::vector<SubContext>&      getSubContexts() { return atlasContexts_[atlasIndex_].getSubContexts(); }
  std::vector<unionPatch>&      getUnionPatch() { return atlasContexts_[atlasIndex_].getUnionPatch(); }

  PCCVector3<float>&            getModelOrigin() { return modelOrigin_; }
  void                          setModelOrigin( PCCVector3<float>& value ) { modelOrigin_ = value; }
  float                         getModelScale() { return modelScale_; }
  void                          setModelScale( float value ) { modelScale_ = value; }

 private:
  PCCVector3<float>            modelOrigin_;
  float                        modelScale_;
	std::vector<PCCAtlasContext> atlasContexts_;
	size_t                       atlasIndex_;
};
};  // namespace pcc

#endif /* PCCContext_h */
