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
typedef pcc::PCCVideo<uint8_t, 3>  PCCVideoTexture;
typedef pcc::PCCVideo<uint16_t, 3> PCCVideoGeometry;
typedef pcc::PCCVideo<uint8_t, 3>  PCCVideoOccupancyMap;

class PCCPatch;
typedef std::map<size_t, PCCPatch>                 unionPatch;     // [TrackIndex, PatchUnion]
typedef std::pair<size_t, size_t>                  GlobalPatch;    // [FrameIndex, PatchIndex]
typedef std::map<size_t, std::vector<GlobalPatch>> GlobalPatches;  // [TrackIndex, <GlobalPatch>]
typedef std::pair<size_t, size_t>                  SubContext;     // [start, end)

// used for context handling
class PCCContext : public PCCHighLevelSyntax {
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
  PCCVideoTexture&              getVideoTextureT1() { return videoTextureT1_; }
  PCCVideoOccupancyMap&         getVideoOccupancyMap() { return videoOccupancyMap_; }
  PCCVideoGeometry&             getVideoMPsGeometry() { return videoMPsGeometry_; }
  PCCVideoTexture&              getVideoMPsTexture() { return videoMPsTexture_; }
  PCCVector3<float>&            getModelOrigin() { return modelOrigin_; }
  std::vector<SubContext>&      getSubContexts() { return subContexts_; }
  std::vector<unionPatch>&      getUnionPatch() { return unionPatch_; }
  void                          setModelOrigin( PCCVector3<float>& value ) { modelOrigin_ = value; }

  void allocOneLayerData();
  void printBlockToPatch( const size_t occupancyResolution );

 private:
  std::vector<PCCFrameContext> frames_;
  PCCVideoGeometry             videoGeometry_;
  PCCVideoGeometry             videoGeometryD1_;
  PCCVideoTexture              videoTexture_;
  PCCVideoTexture              videoTextureT1_;
  PCCVideoOccupancyMap         videoOccupancyMap_;
  PCCVideoGeometry             videoMPsGeometry_;
  PCCVideoTexture              videoMPsTexture_;
  PCCVector3<float>            modelOrigin_;
  std::vector<SubContext>      subContexts_;
  std::vector<unionPatch>      unionPatch_;
};
};  // namespace pcc

#endif /* PCCContext_h */
