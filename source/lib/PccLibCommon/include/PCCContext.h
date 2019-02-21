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

class VPCCParameterSet {
 public:
  VPCCParameterSet()
      : pcmVideoFlag_( false ), sequenceParamterSetId_( 0 ), attributeIndex_( 0 ),
        layerIndex_( 0 ) {}
  bool&    getPCMVideoFlag() { return pcmVideoFlag_; }
  uint8_t& getSequenceParameterSetId() { return sequenceParamterSetId_; }
  uint8_t& getAttributeIndex() { return attributeIndex_; }
  uint8_t& getLayerIndex() { return layerIndex_; }

 private:
  bool    pcmVideoFlag_;
  uint8_t sequenceParamterSetId_;
  uint8_t attributeIndex_;
  uint8_t layerIndex_;
};

class ProfileTierLevel {
 public:
  ProfileTierLevel() : tierFlag_( false ), profileIdc_( 0 ), levelIdc_( 0 ) {}
  bool&    getTierFlag() { return tierFlag_; }
  uint8_t& getProfileIdc() { return profileIdc_; }
  uint8_t& getLevelIdc() { return levelIdc_; }

 private:
  bool    tierFlag_;
  uint8_t profileIdc_;
  uint8_t levelIdc_;
};

class OccupancyParameterSet {
 public:
  OccupancyParameterSet() : codecId_( 0 ), packingBlockSize_( 0 ) {}
  uint8_t& getCodecId() { return codecId_; }
  uint8_t& getPackingBlockSize() { return packingBlockSize_; }

 private:
  uint8_t codecId_;
  uint8_t packingBlockSize_;
};

class GeometrySequenceParams {
 public:
  GeometrySequenceParams()
      : smoothingPresentFlag_( false ), smoothingEnabledFlag_( false ), scalePresentFlag_( false ),
        offsetPresentFlag_( false ), rotationPresentFlag_( false ), pointSizePresentFlag_( false ),
        pointShapePresentFlag_( false ), smoothingGridSize_( 0 ), smoothingThreshold_( 0 ), pointSize_( 0 ),
        pointShape_( 0 ) {}

  bool&     getSmoothingPresentFlag() { return smoothingPresentFlag_; }
  bool&     getSmoothingEnabledFlag() { return smoothingEnabledFlag_; }
  bool&     getScalePresentFlag() { return scalePresentFlag_; }
  bool&     getOffsetPresentFlag() { return offsetPresentFlag_; }
  bool&     getRotationPresentFlag() { return rotationPresentFlag_; }
  bool&     getPointSizePresentFlag() { return pointSizePresentFlag_; }
  bool&     getPointShapePresentFlag() { return pointShapePresentFlag_; }
  uint8_t&  getSmoothingGridSize() { return smoothingGridSize_; }
  uint8_t&  getSmoothingThreshold() { return smoothingThreshold_; }
  uint32_t& getScaleOnAxis( int index ) { return scaleOnAxis_[index]; }
  int32_t&  getOffsetOnAxis( int index ) { return offsetOnAxis_[index]; }
  int32_t&  getRotationOnAxis( int index ) { return rotationOnAxis_[index]; }
  uint8_t&  getPointSize() { return pointSize_; }
  uint8_t&  getPointShape() { return pointShape_; }

 private:
  bool     smoothingPresentFlag_;
  bool     smoothingEnabledFlag_;
  bool     scalePresentFlag_;
  bool     offsetPresentFlag_;
  bool     rotationPresentFlag_;
  bool     pointSizePresentFlag_;
  bool     pointShapePresentFlag_;
  uint8_t  smoothingGridSize_;
  uint8_t  smoothingThreshold_;
  uint32_t scaleOnAxis_[3];
  int32_t  offsetOnAxis_[3];
  int32_t  rotationOnAxis_[3];
  uint8_t  pointSize_;
  uint8_t  pointShape_;
};

class GeometryParameterSet {
 public:
  GeometryParameterSet()
      : codecId_( 0 ), coordinatesBitdepth_( 0 ), pcmCodecId_( 0 ), metadataEnabledFlag_( false ),
        patchEnabledFlag_( false ), patchScaleEnabledFlag_( false ),
        patchOffsetEnabledFlag_( false ), patchRotationEnabledFlag_( false ),
        patchPointSizeEnabledFlag_( false ), patchPointShapeEnabledFlag_( false ) {}

  uint8_t&                getCodecId() { return codecId_; }
  uint8_t&                get3dCoordinatesBitdepth() { return coordinatesBitdepth_; }
  uint8_t&                getPcmCodecId() { return pcmCodecId_; }
  bool&                   getMetadataEnabledFlag() { return metadataEnabledFlag_; }
  bool&                   getPatchEnabledFlag() { return patchEnabledFlag_; }
  bool&                   getPatchScaleEnabledFlag() { return patchScaleEnabledFlag_; }
  bool&                   getPatchOffsetEnabledFlag() { return patchOffsetEnabledFlag_; }
  bool&                   getPatchRotationEnabledFlag() { return patchRotationEnabledFlag_; }
  bool&                   getPatchPointSizeEnabledFlag() { return patchPointSizeEnabledFlag_; }
  bool&                   getPatchPointShapeEnabledFlag() { return patchPointShapeEnabledFlag_; }
  GeometrySequenceParams& getGeometrySequenceParams() { return geometrySequenceParams_; }

 private:
  uint8_t                codecId_;
  uint8_t                coordinatesBitdepth_;
  uint8_t                pcmCodecId_;
  bool                   metadataEnabledFlag_;
  bool                   patchEnabledFlag_;
  bool                   patchScaleEnabledFlag_;
  bool                   patchOffsetEnabledFlag_;
  bool                   patchRotationEnabledFlag_;
  bool                   patchPointSizeEnabledFlag_;
  bool                   patchPointShapeEnabledFlag_;
  GeometrySequenceParams geometrySequenceParams_;
};

class AttributeParameterSet {
 public:
  AttributeParameterSet() {}

 private:
  // TODO
};

class SequenceParameterSet {
 public:
  SequenceParameterSet()
      : index_( 0 ), width_( 0 ), height_( 0 ), layerCount_( 0 ), attributeCount_( 0 ),
        enhancedOccupancyMapForDepthFlag_( false ), multipleLayerStreamsPresentFlag_( false ),
        pcmPatchEnabledFlag_( false ), pcmSeparateVideoPresentFlag_( false ),
        patchSequenceOrientationEnabledFlag_( false ), patchInterPredictionEnabledFlag_( false ),
        pixelInterleavingFlag_( false ), pointLocalReconstructionEnabledFlag_( false ) {
    layerAbsoluteCodingEnabledFlag_.clear();
    layerPredictorIndexDiff_.clear();
    layerPredictorIndex_.clear();
  }
  size_t& getIndex() { return index_; }
  size_t& getWidth() { return width_; }
  size_t& getHeight() { return height_; }
  size_t& getLayerCount() { return layerCount_; }
  size_t& getAttributeCount() { return attributeCount_; }
  bool&   getEnhancedOccupancyMapForDepthFlag() { return enhancedOccupancyMapForDepthFlag_; }
  bool&   getMultipleLayerStreamsPresentFlag() { return multipleLayerStreamsPresentFlag_; }
  bool&   getPcmPatchEnabledFlag() { return pcmPatchEnabledFlag_; }
  bool&   getPcmSeparateVideoPresentFlag() { return pcmSeparateVideoPresentFlag_; }
  bool&   getPatchSequenceOrientationEnabledFlag() { return patchSequenceOrientationEnabledFlag_; }
  bool&   getPatchInterPredictionEnabledFlag() { return patchInterPredictionEnabledFlag_; }
  bool&   getPixelInterleavingFlag() { return pixelInterleavingFlag_; }
  bool&   getPointLocalReconstructionEnabledFlag() { return pointLocalReconstructionEnabledFlag_; }
  std::vector<size_t>& getLayerPredictorIndexDiff() { return layerPredictorIndexDiff_; }
  std::vector<size_t>& getLayerPredictorIndex() { return layerPredictorIndex_; }
  std::vector<bool>& getLayerAbsoluteCodingEnabledFlag() { return layerAbsoluteCodingEnabledFlag_; }

  ProfileTierLevel&      getProfileTierLevel() { return profileTierLevel_; }
  GeometryParameterSet&  getGeometryParameterSet() { return geometryParameterSet_; }
  OccupancyParameterSet& getOccupancyParameterSet() { return occupancyParameterSet_; }

  std::vector<AttributeParameterSet>& getAttributeParameterSets() {
    return attributeParameterSets_;
  }
  AttributeParameterSet& getAttributeParameterSets( size_t index ) {
    return attributeParameterSets_[index];
  }

 private:
  size_t                             index_;
  size_t                             width_;
  size_t                             height_;
  size_t                             layerCount_;
  size_t                             attributeCount_;
  bool                               enhancedOccupancyMapForDepthFlag_;
  bool                               multipleLayerStreamsPresentFlag_;
  bool                               pcmPatchEnabledFlag_;
  bool                               pcmSeparateVideoPresentFlag_;
  bool                               patchSequenceOrientationEnabledFlag_;
  bool                               patchInterPredictionEnabledFlag_;
  bool                               pixelInterleavingFlag_;
  bool                               pointLocalReconstructionEnabledFlag_;
  std::vector<bool>                  layerAbsoluteCodingEnabledFlag_;  // spsLayerCount size
  std::vector<size_t>                layerPredictorIndexDiff_;         // spsLayerCount size
  std::vector<size_t>                layerPredictorIndex_;             // spsLayerCount size
  ProfileTierLevel                   profileTierLevel_;
  GeometryParameterSet               geometryParameterSet_;
  OccupancyParameterSet              occupancyParameterSet_;
  std::vector<AttributeParameterSet> attributeParameterSets_;
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
  bool&              getLosslessGeo444() { return losslessGeo444_; }
  bool&              getLosslessGeo() { return losslessGeo_; }
  bool&              getLosslessTexture() { return losslessTexture_; }
  uint8_t&           getOccupancyPrecision() { return occupancyPrecision_; }
  bool&              getGridSmoothing() { return gridSmoothing_; }
  uint8_t&           getNoAttributes() { return noAttributes_; }
  bool&              getAbsoluteD1() { return absoluteD1_; }
  bool&              getBinArithCoding() { return binArithCoding_; }
  float&             getModelScale() { return modelScale_; }
  PCCVector3<float>& getModelOrigin() { return modelOrigin_; }
  uint8_t&           getThresholdColorSmoothing() { return thresholdColorSmoothing_; }
  double&            getThresholdLocalEntropy() { return thresholdLocalEntropy_; }
  uint8_t&           getRadius2ColorSmoothing() { return radius2ColorSmoothing_; }
  uint8_t&           getNeighborCountColorSmoothing() { return neighborCountColorSmoothing_; }
  uint8_t&           getFlagColorSmoothing() { return flagColorSmoothing_; }
  bool&              getImproveEDD() { return improveEDD_; }
  bool&              getDeltaCoding() { return deltaCoding_; }
  bool&              getSixDirectionMode() { return sixDirectionMode_; }
  bool&              getRemoveDuplicatePoints() { return removeDuplicatePoints_; }
  bool&              getUseAdditionalPointsPatch() { return useAdditionalPointsPatch_; }
  uint8_t&           getMinLevel() { return minLevel_; }
  bool&              getGlobalPatchAllocation() { return globalPatchAllocation_; }
  bool&              getUse3dmc() { return use3dmc_; }
  const size_t       getMPGeoWidth() { return MPGeoWidth_; }
  const size_t       getMPGeoHeight() { return MPGeoHeight_; }
  const size_t       getMPAttWidth() { return MPAttWidth_; }
  const size_t       getMPAttHeight() { return MPAttHeight_; }

  void setLosslessGeo444( bool losslessGeo444 ) { losslessGeo444_ = losslessGeo444; }
  void setLossless( bool losslessGeo ) { losslessGeo_ = losslessGeo; }
  void setLosslessTexture( bool losslessTexture ) { losslessTexture_ = losslessTexture; }
  void setMPGeoWidth( size_t width ) { MPGeoWidth_ = width; }
  void setMPGeoHeight( size_t height ) { MPGeoHeight_ = height; }
  void setMPAttWidth( size_t width ) { MPAttWidth_ = width; }
  void setMPAttHeight( size_t height ) { MPAttHeight_ = height; }

  PCCMetadata& getGOFLevelMetadata() { return gofLevelMetadata_; }

  uint8_t& getSmoothingRadius() { return smoothingRadius_; }
  uint8_t& getSmoothingNeighbourCount() { return smoothingNeighbourCount_; }
  uint8_t& getSmoothingRadius2BoundaryDetection() { return smoothingRadius2BoundaryDetection_; }
  
  std::vector<SubContext>& getSubContexts() { return subContexts_; }
  std::vector<unionPatch>& getunionPatch() { return unionPatch_; }
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

  void allocOneLayerData( const size_t occupancyResolution );
  void printVideoBitstream();
  void printBlockToPatch( const size_t occupancyResolution );

  VPCCParameterSet&     getVPCC() { return vpccParameterSet_; }
  SequenceParameterSet& getSps() {
    for ( auto& sps : sequenceParameterSets_ ) {
      if ( vpccParameterSet_.getSequenceParameterSetId() == sps.getIndex() ) { return sps; }
    }
    fprintf( stderr, "Error: can't find sps[ %lc ] \n",
             vpccParameterSet_.getSequenceParameterSetId() );
    exit( -1 );
  }

 private:
  std::vector<PCCFrameContext> frames_;
  PCCVideoGeometry             videoGeometry_;
  PCCVideoGeometry             videoGeometryD1_;
  PCCVideoTexture              videoTexture_;
  PCCVideoOccupancyMap         videoOccupancyMap_;
  PCCVideoGeometry             videoMPsGeometry_;
  PCCVideoTexture              videoMPsTexture_;

  // deprecated, must be removed:
  bool              losslessGeo444_;
  bool              losslessGeo_;
  bool              losslessTexture_;
  size_t            MPGeoWidth_;
  size_t            MPGeoHeight_;
  size_t            MPAttWidth_;
  size_t            MPAttHeight_;
  uint8_t           occupancyPrecision_;
  bool              gridSmoothing_;
  uint8_t           noAttributes_;
  bool              absoluteD1_;
  bool              binArithCoding_;
  float             modelScale_;
  PCCVector3<float> modelOrigin_;
  uint8_t           thresholdColorSmoothing_;
  double            thresholdLocalEntropy_;
  uint8_t           radius2ColorSmoothing_;
  uint8_t           neighborCountColorSmoothing_;
  uint8_t           flagColorSmoothing_;
  bool              improveEDD_;
  bool              deltaCoding_;
  bool              sixDirectionMode_;
  bool              removeDuplicatePoints_;
  bool              useAdditionalPointsPatch_;
  uint8_t           minLevel_;
  bool              globalPatchAllocation_;
  bool              use3dmc_;

  uint8_t                        smoothingRadius_;
  uint8_t                        smoothingNeighbourCount_;
  uint8_t                        smoothingRadius2BoundaryDetection_;
  
  PCCMetadata                    gofLevelMetadata_;
  std::vector<PCCVideoBitstream> videoBitstream_;
  std::vector<SubContext>        subContexts_;
  std::vector<unionPatch>        unionPatch_;
  //~ deprecated, must be removed

  VPCCParameterSet                  vpccParameterSet_;
  std::vector<SequenceParameterSet> sequenceParameterSets_;
};
};  // namespace pcc

#endif /* PCCContext_h */
