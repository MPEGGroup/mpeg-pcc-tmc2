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
#ifndef PCCFrameContext_h
#define PCCFrameContext_h

#include "PCCCommon.h"

#include "PCCPointSet.h"
#include "PCCPatch.h"
#include "PCCMetadata.h"

namespace pcc {
struct PCCGPAFrameSize {
  size_t widthGPA_;
  size_t heightGPA_;
};

struct PCCFrameOCPInfo {
  size_t            occupancySizeU_;
  size_t            occupancySizeV_;
  size_t            maxOccupancyRow_;
  std::vector<bool> occupancyMap_;
};

class PCCFrameContext {
 public:
  PCCFrameContext();
  ~PCCFrameContext();
  std::vector<PCCVector3<size_t>>&   getPointToPixel() { return pointToPixel_; }
  std::vector<size_t>&               getBlockToPatch() { return blockToPatch_; }
  std::vector<uint32_t>&             getOccupancyMap() { return occupancyMap_; }
  std::vector<uint32_t>&             getFullOccupancyMap() { return fullOccupancyMap_; }
  std::vector<PCCPatch>&             getPatches() { return patches_; }
  PCCMetadata&                       getFrameLevelMetadata() { return frameLevelMetadata_; }
  const PCCPatch&                    getPatch( size_t index ) const { return patches_[index]; }
  std::vector<PCCMissedPointsPatch>& getMissedPointsPatches() { return missedPointsPatches_; }
  PCCMissedPointsPatch&              getMissedPointsPatch( size_t index ) { return missedPointsPatches_[index]; }
  std::vector<size_t>&               getNumberOfMissedPoints() { return numberOfMissedPoints_; };

        size_t&                       getWidth() { return width_; }
        size_t&                       getHeight() { return height_; }
//  const size_t                       getWidth() { return width_; }
//  const size_t                       getHeight() { return height_; }
  const size_t                       getIndex() { return index_; }
  const size_t                       getNumberOfEddPoints() { return NumberOfEddPoints_; }
  const size_t                       getNumberOfMissedPoints( int index ) { return numberOfMissedPoints_[index]; }
  const size_t                       getNumMatchedPatches() { return numMatchedPatches_; }
  const size_t                       getNumberOfMissedPointsPatches() { return missedPointsPatches_.size(); }
  const size_t                       getTotalNumberOfMissedPoints() { return totalNumberOfMissedPoints_; }
  const size_t                       getMPGeoWidth() { return MPGeoWidth_; }
  const size_t                       getMPGeoHeight() { return MPGeoHeight_; }
  const size_t                       getMPAttWidth() { return MPAttWidth_; }
  const size_t                       getMPAttHeight() { return MPAttHeight_; }
  const bool                         getLosslessGeo() { return losslessGeo_; }
  const bool                         getLosslessGeo444() { return losslessGeo444_; }
  bool                               getUseMissedPointsSeparateVideo() { return useMissedPointsSeparateVideo_; }
  const bool                         getPcmPatchEnabledFlag() { return pcmPatchEnabledFlag_; }
  const size_t getMaxDepth() { return maxDepth_;}
  void setMaxDepth(size_t value) {maxDepth_=value;}
  size_t getGeometry2dNorminalBitdepth( ) { return geometry2dNorminalBitdepth_; }
  const size_t                       getSurfaceThickness() { return surfaceThickness_; }
  std::vector<PCCPointSet3>&         getSrcPointCloudByPatch() { return srcPointCloudByPatch_; }
  PCCPointSet3&              getSrcPointCloudByPatch( size_t patchIndex ) { return srcPointCloudByPatch_[patchIndex]; }
  std::vector<PCCPointSet3>& getSrcPointCloudByBlock() { return srcPointCloudByBlock_; }
  PCCVector3D&               getWeightNormal() { return weightNormal_; }
  uint8_t&                   getPointLocalReconstructionNumber() { return pointLocalReconstructionNumber_; }
  PCCGPAFrameSize&           getPrePCCGPAFrameSize() { return prePCCGPAFrameSize_; }
  PCCGPAFrameSize&           getCurPCCGPAFrameSize() { return curPCCGPAFrameSize_; }
  PCCFrameOCPInfo&           getPCCOCPGPAInfo() { return ocpGPAInfo_; }
  size_t&                    getGlobalPatchCount() { return globalPatchCount_; }  
  size_t                     getGeometry3dCoordinatesBitdepth() { return geometry3dCoordinatesBitdepth_; }
  PCCEDDPointsPatch&         getEDDdPointsPatch() { return eddPointsPatch_; }

  void setIndex(size_t value) { index_ = value; }
  void setWidth(size_t value) { width_ = value; }
  void setHeight(size_t value) { height_ = value; }
  void setMPGeoWidth( size_t width ) { MPGeoWidth_ = width; }
  void setMPGeoHeight( size_t height ) { MPGeoHeight_ = height; }
  void setMPAttWidth( size_t width ) { MPAttWidth_ = width; }
  void setMPAttHeight( size_t height ) { MPAttHeight_ = height; }
  void setLosslessGeo( bool lossless ) { losslessGeo_ = lossless; }
  void setLosslessGeo444( bool lossless ) { losslessGeo444_ = lossless; }
  void setUseMissedPointsSeparateVideo( bool value ) { useMissedPointsSeparateVideo_ = value; }
  void setSurfaceThickness( size_t surfaceThickness ) { surfaceThickness_ = surfaceThickness; }
  void setNumberOfEddPoints( size_t numPoints ) { NumberOfEddPoints_ = numPoints; }
  void setNumberOfMissedPoints( int index, size_t value ) { numberOfMissedPoints_[index] = value; }
  void setNumberOfMissedPointsPatches( size_t numPoints ) { numberOfMissedPointsPatches_ = numPoints; }
  void setTotalNumberOfMissedPoints( size_t numPoints ) { totalNumberOfMissedPoints_ = numPoints; }
  void setGeometry3dCoordinatesBitdepth( size_t value ) { geometry3dCoordinatesBitdepth_ = value; }
  void setPcmPatchEnabledFlag( bool value ) { pcmPatchEnabledFlag_ = value; }
  void setGeometry2dNorminalBitdepth( size_t value ) { geometry2dNorminalBitdepth_ = value; }
  void setNumMatchedPatches(size_t value) { numMatchedPatches_ = value; }
  
  void allocOneLayerData();
  void printBlockToPatch( const size_t occupancyResolution );
  void printPatch();
  void printPatchDecoder();

 private:
  size_t                                       index_;
  size_t                                       numMatchedPatches_;
  size_t                                       surfaceThickness_;
  size_t                                       width_;
  size_t                                       height_;
  size_t                                       MPGeoWidth_;
  size_t                                       MPGeoHeight_;
  size_t                                       MPAttWidth_;
  size_t                                       MPAttHeight_;
  size_t                                       numberOfMissedPointsPatches_;
  size_t                                       totalNumberOfMissedPoints_;
  size_t                                       NumberOfEddPoints_;
  size_t                                       globalPatchCount_;   
  size_t                                       geometry3dCoordinatesBitdepth_;
  uint8_t                                      pointLocalReconstructionNumber_;
  bool                                         losslessGeo_;
  bool                                         losslessGeo444_;
  bool                                         useMissedPointsSeparateVideo_;
  bool                                         pcmPatchEnabledFlag_;
  size_t geometry2dNorminalBitdepth_;
  size_t maxDepth_;
  std::vector<PCCVector3<size_t>>              pointToPixel_;
  std::vector<size_t>                          blockToPatch_;
  std::vector<uint32_t>                        occupancyMap_;
  std::vector<uint32_t>                        fullOccupancyMap_;
  std::vector<PCCPatch>                        patches_;
  std::vector<PCCMissedPointsPatch>            missedPointsPatches_;
  std::vector<size_t>                          numberOfMissedPoints_;
  PCCMetadata                                  frameLevelMetadata_;
  std::vector<PCCPointSet3>                    srcPointCloudByPatch_;
  std::vector<PCCPointSet3>                    srcPointCloudByBlock_;
  std::vector<PCCPointSet3>                    recPointCloudByBlock_;
  std::vector<std::vector<PCCVector3<size_t>>> pointToPixelByBlock_;
  PCCGPAFrameSize                              prePCCGPAFrameSize_;
  PCCGPAFrameSize                              curPCCGPAFrameSize_;
  PCCFrameOCPInfo                              ocpGPAInfo_;
  PCCVector3D                                  weightNormal_;
  PCCEDDPointsPatch                            eddPointsPatch_;
};

};  // namespace pcc

#endif /* PCCFrameContext_h */
