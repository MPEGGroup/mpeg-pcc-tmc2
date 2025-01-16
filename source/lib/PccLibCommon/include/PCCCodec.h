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
#ifndef PCCCodec_h
#define PCCCodec_h

#include "PCCCommon.h"
#include "PCCImage.h"
#include "PCCMath.h"
#include "PCCVideo.h"
#include "PCCContext.h"

namespace pcc {
class PCCPatch;
} /* namespace pcc */

namespace pcc {

class PCCContext;
class PCCFrameContext;
class PCCGroupOfFrames;
class PCCPointSet3;
template <typename T, size_t N>
class PCCVideo;
typedef pcc::PCCVideo<uint16_t, 3> PCCVideoGeometry;
typedef pcc::PCCVideo<uint16_t, 3> PCCVideoAttribute;
template <typename T, size_t N>
class PCCImage;
typedef pcc::PCCImage<uint16_t, 3> PCCImageGeometry;
typedef pcc::PCCImage<uint8_t, 3>  PCCImageOccupancyMap;

struct GeneratePointCloudParameters {
  size_t      occupancyResolution_;
  size_t      occupancyPrecision_;
  bool        enableSizeQuantization_;
  bool        gridSmoothing_;
  size_t      gridSize_;
  size_t      neighborCountSmoothing_;
  double      radius2Smoothing_;
  double      radius2BoundaryDetection_;
  double      thresholdSmoothing_;
  size_t      rawPointColorFormat_;
  size_t      nbThread_;
  bool        multipleStreams_;
  bool        absoluteD1_;
  size_t      surfaceThickness_;
  double      thresholdColorSmoothing_;
  size_t      cgridSize_;
  double      thresholdColorDifference_;
  double      thresholdColorVariation_;
  bool        flagGeometrySmoothing_;
  bool        flagColorSmoothing_;
  bool        enhancedOccupancyMapCode_;
  size_t      EOMFixBitCount_;
  size_t      thresholdLossyOM_;
  bool        removeDuplicatePoints_;
  size_t      mapCountMinus1_;
  bool        pointLocalReconstruction_;
  bool        singleMapPixelInterleaving_;
  std::string path_;
  bool        useAdditionalPointsPatch_;
  bool        useAuxSeperateVideo_;
  size_t      plrlNumberOfModes_;
  size_t      geometryBitDepth3D_;
  size_t      geometry3dCoordinatesBitdepth_;
  bool        pbfEnableFlag_;
  int16_t     pbfPassesCount_;
  int16_t     pbfFilterSize_;
  int16_t     pbfLog2Threshold_;
};

struct PatchParams {
  PatchParams( uint8_t mapCntMinus1 = 0 ) :
      patchType_( PROJECTED ),
      patchInAuxVideo_( 0 ),
      patch2dPosX_( 0 ),
      patch2dPosY_( 0 ),
      patch2dSizeX_( 1 ),
      patch2dSizeY_( 1 ),
      patch3dOffsetU_( 0 ),
      patch3dOffsetV_( 0 ),
      patch3dOffsetD_( 0 ),
      patch3dRangeD_( 1 ),
      patchProjectionID_( 0 ),
      patchOrientationIndex_( 0 ),
      patchLoDScaleX_( 1 ),
      patchLoDScaleY_( 1 ),
      patchRawPoints_( 0 ),
      epduAssociatedPatchCount_( 0 ) {
    aspsMapCountMinus1_    = mapCntMinus1;
    patchPackingBlockSize_ = 64;
    plriMapPresentFlag_.resize( aspsMapCountMinus1_ + 1 );
    plriBlockSize_.resize( aspsMapCountMinus1_ + 1 );
  }
  ~PatchParams() {
    epduAssociatedPoints_.clear();
    plriMapPresentFlag_.clear();
    plriBlockSize_.clear();
  };

  int64_t             patchType_;
  int8_t              patchInAuxVideo_;
  int64_t             patch2dPosX_;
  int64_t             patch2dPosY_;
  int64_t             patch2dSizeX_;
  int64_t             patch2dSizeY_;
  int64_t             patch3dOffsetU_;
  int64_t             patch3dOffsetV_;
  int64_t             patch3dOffsetD_;
  int64_t             patch3dRangeD_;
  int64_t             patchProjectionID_;
  int64_t             patchOrientationIndex_;
  int64_t             patchLoDScaleX_;
  int64_t             patchLoDScaleY_;
  int64_t             patchRawPoints_;
  int64_t             epduAssociatedPatchCount_;
  std::vector<size_t> epduAssociatedPoints_;
  size_t              aspsMapCountMinus1_;
  size_t              patchPackingBlockSize_;
  PLRData             patchPLRData_;
  std::vector<bool>   plriMapPresentFlag_;
  std::vector<size_t> plriBlockSize_;
};

class PCCCodec {
 public:
  PCCCodec();
  ~PCCCodec();

  void generatePointCloud( PCCPointSet3&                       reconstruct,
                           PCCContext&                         context,
                           size_t                              frameIndex,
                           size_t                              tileIndex,
                           const GeneratePointCloudParameters& params,
                           std::vector<uint32_t>&              partition,
                           bool                                bDecoder );

  size_t colorPointCloud( PCCPointSet3&                       reconstruct,
                          PCCContext&                         context,
                          PCCFrameContext&                    tile,
                          const std::vector<bool>&            absoluteT1List,
                          const size_t                        multipleStreams,
                          const uint8_t                       attributeCount,
                          size_t                              accTilePointCount,
                          const GeneratePointCloudParameters& params );

  void smoothPointCloudPostprocess( PCCPointSet3&                       reconstruct,
                                    const PCCColorTransform             colorTransform,
                                    const GeneratePointCloudParameters& params,
                                    std::vector<uint32_t>&              partition );
  void colorSmoothing( PCCPointSet3&                       reconstruct,
                       const PCCColorTransform             colorTransform,
                       const GeneratePointCloudParameters& params );

  void generateRawPointsGeometryfromVideo( PCCContext& context, PCCFrameContext& tile, size_t frameIndex );

  void generateRawPointsAttributefromVideo( PCCContext& context, PCCFrameContext& tile, size_t frameIndex );

  void generateRawPointsGeometryfromVideo( PCCContext& context, size_t frameIndex );
  void generateRawPointsAttributefromVideo( PCCContext& context, size_t frameIndex );

  void setLogger( PCCLogger& logger ) { logger_ = &logger; }

 protected:
  void generateOccupancyMap( PCCFrameContext&      tile,
                             PCCImageOccupancyMap& videoFrame,
                             const size_t          occupancyPrecision,
                             const size_t          thresholdLossyOM,
                             const bool            enhancedOccupancyMapForDepthFlag );

  void generateTileBlockToPatchFromOccupancyMapVideo( PCCContext&  context,
                                                      const size_t occupancyResolution,
                                                      const size_t occupancyPrecision );

  void generateTileBlockToPatchFromOccupancyMapVideo( PCCContext&           context,
                                                      PCCFrameContext&      tile,
                                                      size_t                frameIdx,
                                                      PCCImageOccupancyMap& occupancyMapImage,
                                                      const size_t          occupancyResolution,
                                                      const size_t          occupancyPrecision );

  void generateAtlasBlockToPatchFromOccupancyMapVideo( PCCContext&  context,
                                                       const size_t occupancyResolution,
                                                       const size_t occupancyPrecision );

  void generateAtlasBlockToPatchFromOccupancyMapVideo( PCCContext&           context,
                                                       PCCFrameContext&      tile,
                                                       size_t                frameIdx,
                                                       PCCImageOccupancyMap& occupancyMapImage,
                                                       const size_t          occupancyResolution,
                                                       const size_t          occupancyPrecision );

  void generateBlockToPatchFromOccupancyMapVideo( PCCContext&  context,
                                                  const size_t occupancyResolution,
                                                  const size_t occupancyPrecision );

  void generateBlockToPatchFromOccupancyMapVideo( PCCContext&           context,
                                                  PCCFrameContext&      tile,
                                                  size_t                frameIdx,
                                                  PCCImageOccupancyMap& occupancyMapImage,
                                                  const size_t          occupancyResolution,
                                                  const size_t          occupancyPrecision );

  int getDeltaNeighbors( const PCCImageGeometry& frame,
                         const PCCPatch&         patch,
                         const int               xOrg,
                         const int               yOrg,
                         const int               neighboring,
                         const int               threshold,
                         const bool              projectionMode );

  std::vector<PCCPoint3D> generatePoints( const GeneratePointCloudParameters&  params,
                                          PCCFrameContext&                     tile,
                                          const std::vector<PCCVideoGeometry>& videoMultiple,
                                          const size_t                         videoFrameIndex,
                                          const size_t                         patchIndex,
                                          const size_t                         u,
                                          const size_t                         v,
                                          const size_t                         x,
                                          const size_t                         y,
                                          const bool                           interpolate = 0,
                                          const bool                           filling     = 0,
                                          const size_t                         minD1       = 0,
                                          const size_t                         neighbor    = 0 );
  void                    generateAfti( PCCContext& context, size_t frameIndex, AtlasFrameTileInformationRbsp& afti );

  inline double entropy( std::vector<uint8_t>& Data, int N ) {
    std::vector<size_t> count;
    count.resize( 256, 0 );
    for ( size_t i = 0; i < N; ++i ) { ++count[size_t( Data[i] )]; }
    double s = 0;
    for ( size_t i = 0; i < 256; ++i ) {
      if ( count[i] ) {
        double p = double( count[i] ) / double( N );
        s += -p * std::log2( p );
      }
    }
    return s;
  }

  inline double median( std::vector<uint16_t>& Data, int N ) {
    std::sort( Data.begin(), Data.end() );

    if ( N % 2 == 0 )
      return ( double( Data[N / 2] ) + double( Data[N / 2 - 1] ) ) / 2.0;
    else
      return double( Data[N / 2] );
  }

  inline double mean( std::vector<uint16_t>& Data, int N ) {
    double s = 0.0;
    for ( size_t i = 0; i < N; ++i ) { s += double( Data[i] ); }
    return s / double( N );
  }

  // seiMessage
  void aspsCommonByteString( std::vector<uint8_t>& stringByte, AtlasSequenceParameterSetRbsp& asps );
  void aspsApplicationByteString( std::vector<uint8_t>&          strinByte,
                                  AtlasSequenceParameterSetRbsp& asps,
                                  AtlasFrameParameterSetRbsp&    afps );
  void afpsCommonByteString( std::vector<uint8_t>& stringByte, PCCContext& context, size_t afpsIndex, size_t frmIndex );
  void afpsApplicationByteString( std::vector<uint8_t>&          stringByte,
                                  AtlasSequenceParameterSetRbsp& asps,
                                  AtlasFrameParameterSetRbsp&    afps );

  void atlasPatchCommonByteString( std::vector<uint8_t>&     stringByte,
                                   size_t                    patchIndex,
                                   std::vector<PatchParams>& atlasPatchParams );
  void atlasPatchApplicationByteString( std::vector<uint8_t>&     stringByte,
                                        size_t                    patchIndex,
                                        std::vector<PatchParams>& atlasPatchParams );
  void tilePatchCommonByteString( std::vector<uint8_t>&                  stringByte,
                                  size_t                                 tileId,
                                  size_t                                 patchIndex,
                                  std::vector<std::vector<PatchParams>>& tilePatchParams );
  void tilePatchApplicationByteString( std::vector<uint8_t>&                  stringByte,
                                       size_t                                 tileId,
                                       size_t                                 patchIndex,
                                       std::vector<std::vector<PatchParams>>& tilePatchParams );
  void atlasBlockToPatchByteString( std::vector<uint8_t>& stringByte, std::vector<std::vector<int64_t>> atlasB2p );
  void tileBlockToPatchByteString( std::vector<uint8_t>&                          stringByte,
                                   size_t                                         tileID,
                                   std::vector<std::vector<std::vector<int64_t>>> tileB2p );
  void getHashPatchParams( PCCContext&                            context,
                           size_t                                 frameIndex,
                           size_t                                 tileIndex,
                           std::vector<std::vector<PatchParams>>& tilePatchParams,
                           std::vector<PatchParams>&              atlasPatchParams );
  void getB2PHashPatchParams( PCCContext&                                     context,
                              size_t                                          frameIndex,
                              std::vector<std::vector<std::vector<int64_t>>>& tileB2PPatchParams,
                              std::vector<std::vector<int64_t>>&              atlasB2PPatchParams );

  void getB2PHashAtlasPatchParams( PCCContext&                                     context,
                                   size_t                                          frameIndex,
                                   size_t                                          tileIdx,
                                   size_t                                          atlIndex,
                                   std::vector<std::vector<std::vector<int64_t>>>& tileB2PPatchParams,
                                   std::vector<std::vector<int64_t>>&              atlasB2PPatchParams );

  void inverseRotatePosition45DegreeOnAxis( size_t Axis, size_t lod, PCCPoint3D input, PCCVector3D& output );

  inline void SEIMd5Checksum( PCCContext& context, std::vector<uint8_t>& vec ) {
    std::vector<uint8_t> md5Digest( 16 );
    md5Digest = context.computeMD5( vec.data(), vec.size() );
    for ( auto& e : md5Digest ) { TRACE_HLS( "%02x", e ); }
    TRACE_HLS( "%s", "\n" );
    vec.clear();
  }

  PCCLogger* logger_ = nullptr;

 private:
  void smoothPointCloud( PCCPointSet3&                      reconstruct,
                         const std::vector<uint32_t>&       partition,
                         const GeneratePointCloudParameters params );

  void smoothPointCloudGrid( PCCPointSet3&                       reconstruct,
                             const std::vector<uint32_t>&        partition,
                             const GeneratePointCloudParameters& params,
                             uint16_t                            gridWidth,
#if CELLID_UNORDERED_MAP
                             std::unordered_map<size_t, int64_t>& cellIndex);
#else
                             std::vector<int>&                   cellIndex );
#endif

  void addGridCentroid( PCCPoint3D&                     point,
                        uint32_t                        patchIdx,
                        std::vector<uint16_t>&          count,
                        std::vector<PCCVector3<float>>& center,
                        std::vector<uint32_t>&          partition,
                        std::vector<bool>&              doSmooth,
                        uint8_t                         gridSize,
                        uint16_t                        gridWidth,
#if CELLID_UNORDERED_MAP
                        int64_t                         cellId );
#else
                        int                             cellId );
#endif

  void addGridColorCentroid( PCCPoint3D&                             point,
                             PCCVector3D&                            color,
                             std::pair<size_t, size_t>               tilePatchIdx,
                             std::vector<uint16_t>&                  colorGridCount,
                             std::vector<PCCVector3<float>>&         colorCenter,
                             std::vector<std::pair<size_t, size_t>>& colorPartition,
                             std::vector<bool>&                      colorDoSmooth,
                             uint8_t                                 colorGrid,
                             std::vector<std::vector<uint16_t>>&     colorLum,
                             const GeneratePointCloudParameters&     params,
#if CELLID_UNORDERED_MAP
                             int64_t                                 cellId );
#else
                             int                                     cellId );
#endif

  bool gridFilteringColor( PCCPoint3D&                         curPos,
                           PCCVector3D&                        colorCentroid,
                           int&                                colorCount,
                           std::vector<uint16_t>&              colorGridCount,
                           std::vector<PCCVector3<float>>&     colorCenterGrid,
                           std::vector<bool>&                  colorDoSmooth,
#if COLOR_SMOOTHING_ALIGNMENT
                           size_t                              w,
#endif
                           uint8_t                             gridSize,
                           PCCVector3D&                        curPosColor,
                           const GeneratePointCloudParameters& params,
#if CELLID_UNORDERED_MAP
                           std::unordered_map<size_t, int64_t>& cellIndex );
#else
                           std::vector<int>&                   cellIndex );
#endif

  void smoothPointCloudColorLC( PCCPointSet3&                       reconstruct,
                                const GeneratePointCloudParameters& params,
#if CELLID_UNORDERED_MAP
                                std::unordered_map<size_t, int64_t>& cellIndex);
#else
                                std::vector<int>&                   cellIndex );
#endif

  bool gridFiltering( const std::vector<uint32_t>&    partition,
                      PCCPointSet3&                   pointCloud,
                      PCCPoint3D&                     curPoint,
                      PCCVector3D&                    centroid,
                      int&                            count,
                      std::vector<uint16_t>&          gridCount,
                      std::vector<PCCVector3<float>>& center,
                      std::vector<bool>&              doSmooth,
                      uint8_t                         gridSize,
                      uint16_t                        gridWidth,
#if CELLID_UNORDERED_MAP
                      std::unordered_map<size_t, int64_t>& cellIndex );
#else
                      std::vector<int>&                cellIndex );
#endif

  void identifyBoundaryPoints( const std::vector<uint32_t>& occupancyMap,
                               const size_t                 x,
                               const size_t                 y,
                               const size_t                 imageWidth,
                               const size_t                 imageHeight,
                               const size_t                 pointIndex,
                               std::vector<uint32_t>&       BPflag,
                               PCCPointSet3&                reconstruct );

#ifdef CODEC_TRACE
  void printChecksum( PCCPointSet3& ePointcloud, std::string eString );
#endif
  std::vector<uint16_t>                  geoSmoothingCount_;
  std::vector<PCCVector3<float>>         geoSmoothingCenter_;
  std::vector<bool>                      geoSmoothingDoSmooth_;
  std::vector<uint32_t>                  geoSmoothingPartition_;
  std::vector<uint16_t>                  colorSmoothingCount_;
  std::vector<PCCVector3<float>>         colorSmoothingCenter_;
  std::vector<bool>                      colorSmoothingDoSmooth_;
  std::vector<std::pair<size_t, size_t>> colorSmoothingPartition_;
  std::vector<std::vector<uint16_t>>     colorSmoothingLum_;
};

};  // namespace pcc

#endif /* PCCCodec_h */
