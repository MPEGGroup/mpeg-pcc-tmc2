
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

namespace pcc {

class PCCGroupOfFrames;
class PCCFrameContext;
typedef pcc::PCCVideo<uint8_t, 3>  PCCVideoTexture;
typedef pcc::PCCVideo<uint16_t, 3> PCCVideoGeometry;
typedef pcc::PCCVideo<uint8_t, 3>  PCCVideoOccupancyMap;

class PCCContext {
 public:
  PCCContext();
  ~PCCContext();

  std::vector<PCCFrameContext>::iterator begin() { return frames_.begin(); }
  std::vector<PCCFrameContext>::iterator end  () { return frames_.end  (); }

  const size_t size() { return frames_.size(); }

  PCCFrameContext& operator[]( int i ) { return frames_[i]; }
  std::vector<PCCFrameContext>& getFrames() { return frames_; }
  PCCVideoGeometry& getVideoGeometry() { return videoGeometry_; }
  PCCVideoGeometry& getVideoGeometryD1() { return videoGeometryD1_; }
  PCCVideoTexture&  getVideoTexture() { return videoTexture_; }
  PCCVideoOccupancyMap& getVideoOccupancyMap() { return videoOccupancyMap_; }
    
  PCCVideoGeometry& getVideoMPsGeometry() { return videoMPsGeometry_; }
  PCCVideoTexture&  getVideoMPsTexture() { return videoMPsTexture_; }
  PCCFrameContext& getFrameContext(int i) { return frames_[i]; }
  const size_t getGofSize() { return frames_.size(); }
  
  const bool getLosslessGeo444() { return losslessGeo444_; }
  const bool getLosslessGeo() { return losslessGeo_; }
  const bool getLosslessAtt() { return losslessTexture_; }
  
  void setLosslessGeo444(bool losslessGeo444){ losslessGeo444_ = losslessGeo444;}
  void setLossless(bool losslessGeo){ losslessGeo_ = losslessGeo;}
  void setLosslessAtt(bool losslessTexture){ losslessTexture_ = losslessTexture;}
  
  const size_t getMPGeoWidth() {return MPGeoWidth_;}
  const size_t getMPGeoHeight() {return MPGeoHeight_;}
  const size_t getMPAttWidth() {return MPAttWidth_;}
  const size_t getMPAttHeight() {return MPAttHeight_;}
  
  void setMPGeoWidth(size_t width) { MPGeoWidth_=width;}
  void setMPGeoHeight(size_t height) { MPGeoHeight_=height;}
  void setMPAttWidth(size_t width) { MPAttWidth_=width;}
  void setMPAttHeight(size_t height) { MPAttHeight_=height;}

  void setUseMissedPointsVideo(bool code) {useMissedPointsVideo_=code;}
  bool getUseMissedPointsVideo() {return useMissedPointsVideo_;}
  
  void setUseOccupancyMapVideo(bool code) { useOccupancyMapVideo = code; }
  bool getUseOccupancyMapVideo() { return useOccupancyMapVideo; }
    
  void setEnhancedDeltaDepth(bool code) {enhancedDeltaDepth_=code;}
  bool getEnhancedDeltaDepth( ) {return enhancedDeltaDepth_;}
 

  PCCMetadata&      getGOFLevelMetadata() { return gofLevelMetadata_; }

  
  void setIndex( size_t i ) { index_ = i; }
  size_t getIndex(){ return index_; }
  size_t& getWidth(){ return width_; }
  size_t& getHeight(){ return height_; }
  void   resize( size_t size );

 private:
  size_t index_;
  size_t width_;
  size_t height_;
  std::vector<PCCFrameContext> frames_;
  PCCVideoGeometry videoGeometry_;
  PCCVideoGeometry videoGeometryD1_;
  PCCVideoTexture  videoTexture_;
  PCCVideoOccupancyMap videoOccupancyMap_;
    
  PCCVideoGeometry videoMPsGeometry_;
  PCCVideoTexture videoMPsTexture_;
  bool losslessGeo444_;
  bool losslessGeo_;
  bool losslessTexture_;
  size_t MPGeoWidth_;
  size_t MPGeoHeight_;
  size_t MPAttWidth_;
  size_t MPAttHeight_;
  bool useMissedPointsVideo_;
  bool useOccupancyMapVideo;
  bool enhancedDeltaDepth_;

  PCCMetadata gofLevelMetadata_;
};
}; //~namespace

#endif /* PCCContext_h */
