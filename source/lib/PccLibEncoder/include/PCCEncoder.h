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
#ifndef PCCEncoder_h
#define PCCEncoder_h

#include "PCCCommon.h"
#include "PCCEncoderParameters.h"
#include "PCCCodec.h"
#include "PCCKdTree.h"
#include <map>

namespace pcc {

class PCCPointSet3;
class PCCGroupOfFrames;
class PCCContext;
class PCCFrameContext;
class PCCAtalsFrameContext;
class PatchFrameGeometryParameterSet;
class GeometryPatchParameterSet;
class V3CParameterSet;
class PLRData;
struct PatchParams;

template <typename T, size_t N>
class PCCVideo;
typedef pcc::PCCVideo<uint8_t, 3>  PCCVideoOccupancyMap;
typedef pcc::PCCVideo<uint16_t, 3> PCCVideoGeometry;
typedef pcc::PCCVideo<uint16_t, 3> PCCVideoAttribute;
template <typename T, size_t N>
class PCCImage;
typedef pcc::PCCImage<uint8_t, 3>  PCCImageOccupancyMap;
typedef pcc::PCCImage<uint16_t, 3> PCCImageGeometry;
typedef pcc::PCCImage<uint16_t, 3> PCCImageAttribute;
struct PCCPatchSegmenter3Parameters;
class PCCPatch;
struct PCCBistreamPosition;

struct SparseMatrixCoefficient {
  int32_t _index;
  double  _value;
};

struct SparseMatrixRow {
  SparseMatrixRow() { _coefficientCount = 0; }
  void addCofficient( const int32_t j, const double value ) {
    assert( _coefficientCount < 5 );
    auto& coeff  = _coefficients[_coefficientCount++];
    coeff._index = j;
    coeff._value = value;
  }
  int32_t                 _coefficientCount;
  SparseMatrixCoefficient _coefficients[5];
};

struct SparseMatrix {
  const SparseMatrixRow& getRow( const int32_t i ) const {
    assert( i < _rows.size() );
    return _rows[i];
  }
  SparseMatrixRow& getRow( const int32_t i ) {
    assert( i < _rows.size() );
    return _rows[i];
  }
  void                         resize( const int32_t rowCount ) { _rows.resize( rowCount ); }
  std::vector<SparseMatrixRow> _rows;
};
struct PaddingContext {
  std::vector<int32_t> _mapping;
  std::vector<int32_t> _invMapping;
  SparseMatrix         _A;
  std::vector<double>  _b[3];
  std::vector<double>  _x, _p, _r, _q;
};

typedef std::map<size_t, PCCPatch> unionPatch;  // unionPatch ------
                                                // [TrackIndex, UnionPatch];
typedef std::pair<size_t, size_t> SubContext;   // SubContext ------ [start,
                                                // end);

#define BAD_HEIGHT_THRESHOLD 1.10
#define BAD_CONDITION_THRESHOLD 2

class PCCEncoder : public PCCCodec {
 public:
  PCCEncoder();
  ~PCCEncoder();
  void setParameters( const PCCEncoderParameters& params );

  int encode( const PCCGroupOfFrames& sources, PCCContext& context, PCCGroupOfFrames& reconstructs );

  void setPostProcessingSeiParameters( GeneratePointCloudParameters& params, PCCContext& context );
  void setGeneratePointCloudParameters( GeneratePointCloudParameters& gpcParams, PCCContext& context );
  void createPatchFrameDataStructure( PCCContext& context );
  void createPatchFrameDataStructure( PCCContext&         context,
                                      PCCFrameContext&    tile,
                                      AtlasTileLayerRbsp& atglu,
                                      size_t              frameIndex,
                                      size_t              tileIndex = 0 );

  void createGeometrySmoothingSei( PCCContext& context, AtlasTileLayerRbsp& );
  void createAttributeSmoothingSei( PCCContext& context, AtlasTileLayerRbsp& );
  void createCodecComponentMappingSei( PCCContext& context, AtlasTileLayerRbsp& );
  void createHashSEI( PCCContext& context, size_t frameIndex, AtlasTileLayerRbsp& );
  void createHlsAtlasTileLogFiles( PCCContext& context, int frameIndex, int afpsId );

 private:
  template <typename T>
  T limit( T x, T minVal, T maxVal );

  //**occupancy map**//
  bool generateOccupancyMapVideo( const PCCGroupOfFrames& sources, PCCContext& context );
  bool generateOccupancyMapVideo( const size_t           imageWidth,
                                  const size_t           imageHeight,
                                  std::vector<uint32_t>& occupancyMap,
                                  PCCImageOccupancyMap&  videoFrameOccupancyMap );
  //  bool generateOccupancyMap( PCCContext& context );
  bool generateOccupancyMap( PCCContext& context, bool copyToFrame = true );
  void modifyOccupancyMapEOM( PCCFrameContext& tile );
  void generateOccupancyMap( PCCFrameContext& tile );
  void refineOccupancyMap( PCCFrameContext& tile );

  void preFilterOccupancyMap( PCCImageOccupancyMap& image, size_t kwidth, size_t kheight );
  bool modifyOccupancyMap( const PCCGroupOfFrames& sources, PCCContext& context );
  bool modifyOccupancyMap( const size_t           imageWidth,
                           const size_t           imageHeight,
                           std::vector<uint32_t>& occupancyMap,
                           PCCImageOccupancyMap&  videoFrameOccupancyMap,
                           std::ofstream&         ofile,
                           uint64_t&              changedPixCnt,
                           uint64_t&              changedPixCnt0To1,
                           uint64_t&              changedPixCnt1To0,
                           uint64_t&              pixCnt );

  //**auxPatches**//
  void markRawPatchLocationOccupancyMapVideo( PCCContext& context );
  void markRawPatchLocation( PCCFrameContext& titleFrame, PCCImageOccupancyMap& occupancyMap );
  void sortRawPointsPatch( PCCFrameContext& titleFrame, size_t index );
  void sortRawPointsPatchMorton( PCCFrameContext& titleFrame, size_t index );

  //**tile and partitions**//
  void   generateTilesFromSegments( PCCContext& context );
  size_t generateTilesFromImage( PCCContext& context );
  void   placeTiles( PCCContext& context, size_t minFrameWidth, size_t minFrameHeight );
  void   replaceFrameContext( PCCContext& context );

  //**patch segmentation**//
  bool generateSegments( const PCCGroupOfFrames& sources, PCCContext& context );
  bool generateSegments( const PCCPointSet3&                 source,
                         PCCAtlasFrameContext&               frameContext,
                         const PCCPatchSegmenter3Parameters& segmenterParams,
                         size_t                              frameIndex,
                         float&                              distanceSrcRec );
  bool placeSegments( const PCCGroupOfFrames& sources, PCCContext& context );

  //**video/image reneration and resizing**//
  bool resizeGeometryVideo( PCCContext& context, PCCCodecId codecId );
  bool resizeTileGeometryVideo( PCCContext& context,
                                size_t      tileIndex,
                                size_t      frameWidth,
                                size_t      frameHeight,
                                int         firstFrame     = -1,
                                int         lastFramePlus1 = -1 );
  bool relocateTileGeometryVideo( PCCContext& context );
  bool generateGeometryVideo( const PCCGroupOfFrames& sources, PCCContext& context );
  bool generateAttributeVideo( const PCCGroupOfFrames&     sources,
                               PCCGroupOfFrames&           reconstruct,
                               PCCContext&                 context,
                               const PCCEncoderParameters& params );
  void placeAuxiliaryPointsTiles( PCCContext& context );
  void generateRawPointsGeometryVideo( PCCContext& context );
  void generateRawPointsAttributeVideo( PCCContext& context );
  void generateRawPointsGeometryImage( PCCContext& context, PCCFrameContext& tile, PCCImageGeometry& image );
  void generateRawPointsAttributeImage( PCCContext& context, PCCFrameContext& tile, PCCImageAttribute& image );
  void generateIntraImage( PCCAtlasFrameContext& frameContext, const size_t mapIndex, PCCImageGeometry& image );
  bool predictGeometryFrame( PCCFrameContext& titleFrame, const PCCImageGeometry& reference, PCCImageGeometry& image );
  static bool predictAttributeFrame( PCCFrameContext&         titleFrame,
                                     const PCCImageAttribute& reference,
                                     PCCImageAttribute&       image );
  void        generateEomPatch( const PCCPointSet3& source, PCCFrameContext& titleFrame );
  void        generateRawPointsPatch( const PCCPointSet3& source,
                                      PCCFrameContext&    titleFrame,
                                      bool                useEnhancedOccupancyMapCode );
  bool        generateScaledGeometry( PCCFrameContext& tile );
  size_t      generateAttributeVideo( const PCCPointSet3& reconstruct,
                                      PCCContext&         context,
                                      size_t              frameIndex,
                                      size_t              tileIndex,
                                      PCCVideoAttribute&  video,
                                      PCCVideoAttribute&  videoT1,
                                      const size_t        mapCount,
                                      size_t              accTilePointCount );

  void dilateGroupGeometryVideo( PCCContext& context, PCCFrameContext& titleFrame, size_t frameIdx );

  //**video/image padding**//
  template <typename T>
  void dilate( PCCFrameContext& frame, PCCImage<T, 3>& image, const PCCImage<T, 3>* reference = nullptr );

  // 3D geometry padding
  void   dilate3DPadding( const PCCPointSet3&     source,
                          PCCAtlasFrameContext&   frameInfo,
                          PCCFrameContext&        frame,
                          PCCImageGeometry&       image,
                          PCCImageOccupancyMap&   occupancyMap,
                          const PCCImageGeometry* reference = nullptr );
  size_t adjustDepth3DPadding( size_t            x,
                               size_t            y,
                               uint16_t          mean_val,
                               PCCImageGeometry& image,
                               PCCKdTree&        kdtree,
                               PCCFrameContext&  frame );

  // Push-pull background filling
  template <typename T>
  int mean4w( T p1, unsigned char w1, T p2, unsigned char w2, T p3, unsigned char w3, T p4, unsigned char w4 );
  template <typename T>
  void pushPullMip( const PCCImage<T, 3>&        image,
                    PCCImage<T, 3>&              mip,
                    const std::vector<uint32_t>& occupancyMap,
                    std::vector<uint32_t>&       mipOccupancyMap );
  template <typename T>
  void pushPullFill( PCCImage<T, 3>&              image,
                     const PCCImage<T, 3>&        mip,
                     const std::vector<uint32_t>& occupancyMap,
                     int                          numIters );
  template <typename T>
  void dilateSmoothedPushPull( PCCFrameContext& frame, PCCImage<T, 3>& image, int mapIdx = -1 );
  template <typename T>
  void dilateHarmonicBackgroundFill( PCCFrameContext& frame, PCCImage<T, 3>& image );
  template <typename T>
  void createCoarseLayer( PCCImage<T, 3>&        image,
                          PCCImage<T, 3>&        mip,
                          std::vector<uint32_t>& occupancyMap,
                          std::vector<uint32_t>& mipOccupancyMap );
  template <typename T>
  void regionFill( PCCImage<T, 3>& image, std::vector<uint32_t>& occupancyMap, PCCImage<T, 3>& imageLowRes );

  //**placing patches**//
  void packFlexible( PCCFrameContext& tile,
                     int              packingStrategy,
                     size_t           frameWidth,
                     size_t           frameHeight,
                     int              safeguard                    = 0,
                     bool             enablePointCloudPartitioning = false );
  void packTetris( PCCFrameContext& tile, size_t frameWidth, size_t frameHeight, int safeguard = 0 );
  void packMultipleTiles( PCCAtlasFrameContext& frame, int safeguard );
  void packFlexibleMultipleTiles( PCCAtlasFrameContext& frame, int safeguard );
  void spatialConsistencyPackMultipleTiles( PCCAtlasFrameContext& frame,
                                            PCCAtlasFrameContext& prevFrame,
                                            int                   safeguard );
  void spatialConsistencyPackFlexibleMultipleTiles( PCCAtlasFrameContext& frame,
                                                    PCCAtlasFrameContext& prevFrame,
                                                    int                   safeguard );

  size_t packRawPointsPatchSimple( PCCFrameContext& tile, size_t patchStartOffsetX = 0, size_t patchStartOffsetY = 0 );

  size_t packRawPointsPatch( PCCFrameContext&   frame,
                             std::vector<bool>& occupancyMap,
                             size_t             width,
                             size_t&            height,
                             size_t             occupancySizeU,
                             size_t             occupancySizeV,
                             size_t             maxOccupancyRow );
  void   packEOMAttributePointsPatch( PCCFrameContext&   frame,
                                      std::vector<bool>& occupancyMap,
                                      size_t             width,
                                      size_t&            height,
                                      size_t             occupancySizeU,
                                      size_t             occupancySizeV,
                                      size_t             maxOccupancyRow );
  void   adjustReferenceAtlasFrames( PCCContext& context, size_t tileIndex );
  double adjustReferenceAtlasFrame( PCCContext&            context,
                                    PCCFrameContext&       tile,
                                    size_t                 tileIndex,
                                    size_t                 listIndex,
                                    std::vector<PCCPatch>& tempPatchList );
  void   spatialConsistencyPackFlexible( PCCFrameContext& tile,
                                         PCCFrameContext& prevFrame,
                                         int              packingStrategy,
                                         size_t           frameWidth,
                                         size_t           frameHeight,
                                         int              safeguard                    = 0,
                                         bool             enablePointCloudPartitioning = false );
  void   spatialConsistencyPackTetris( PCCFrameContext& tile,
                                       PCCFrameContext& prevFrame,
                                       size_t           frameWidth,
                                       size_t           frameHeight,
                                       int              safeguard = 0 );

  //**GTP**//
  void findMatchesForGlobalTetrisPacking( PCCFrameContext& tile, PCCFrameContext& prevFrame );
  void doGlobalTetrisPacking( PCCContext& context,
                              size_t      tileIndex,
                              size_t      frameWidth,
                              size_t      frameHeight,
                              int         firstFrame     = -1,
                              int         lastFramePlus1 = -1 );

  // perform data-adaptive GPA method;
  void performDataAdaptiveGPAMethod( PCCContext& context,
                                     size_t      tileIndex,
                                     size_t      frameWidth,
                                     size_t      frameHeight,
                                     int         firstFrame     = -1,
                                     int         lastFramePlus1 = -1 );

  // start a subContext;
  static void initializeSubContext( PCCFrameContext& tile,
                                    SubContext&      subContext,
                                    GlobalPatches&   globalPatchTracks,
                                    unionPatch&      unionPatch,
                                    size_t           frameIndex );
  // generate globalPatches;
  static void generateGlobalPatches( PCCContext&    context,
                                     size_t         frameIndex,
                                     size_t         tileIndex,
                                     GlobalPatches& globalPatchTracks,
                                     size_t         preIndex );

  // patch unions generation and packing; return the height of the
  // unionsPackingImage;
  size_t unionPatchGenerationAndPacking( const GlobalPatches& globalPatchTracks,
                                         PCCContext&          context,
                                         size_t               tileIndex,
                                         size_t               frameWidth,
                                         size_t               frameHeight,
                                         unionPatch&          unionPatch,
                                         size_t               refFrameIdx,
                                         int                  safeguard   = 0,
                                         bool                 useRefFrame = false );
  // update patch information;
  static void updateGPAPatchInformation( PCCContext& context,
                                         size_t      tileIndex,
                                         SubContext& subContext,
                                         unionPatch& unionPatch );

  // perform data-adaptive gpa packing;
  void performGPAPacking( const SubContext& subContext,
                          unionPatch&       unionPatch,
                          PCCContext&       context,
                          size_t            tileIndex,
                          size_t            frameWidth,
                          size_t            frameHeight,
                          bool&             badGPAPacking,
                          size_t            unionsHeight,
                          int               safeguard   = 0,
                          bool              useRefFrame = false );

  static void clearCurrentGPAPatchDataInfor( PCCContext& context, size_t tileIndex, SubContext& subContext );

  void        packingFirstFrame( PCCContext& context,
                                 size_t      frameIndex,
                                 size_t      tileIndex,
                                 size_t      frameWidth,
                                 size_t      frameHeight,
                                 bool        packingStrategy,
                                 int         safeguard,
                                 bool        hasRefFrame );
  static void updatePatchInformation( PCCContext& context, size_t tileIndex, SubContext& subContext );
  void        packingWithoutRefForFirstFrameNoglobalPatch( PCCPatch&          patch,
                                                           size_t             i,
                                                           size_t             icount,
                                                           size_t&            occupancySizeU,
                                                           size_t&            occupancySizeV,
                                                           const size_t       safeguard,
                                                           std::vector<bool>& occupancyMap,
                                                           size_t&            heightGPA,
                                                           size_t&            widthGPA,
                                                           size_t&            maxOccupancyRow );

  void packingWithRefForFirstFrameNoglobalPatch( PCCPatch&                    patch,
                                                 const std::vector<PCCPatch>& prePatches,
                                                 size_t                       startFrameIndex,
                                                 size_t                       i,
                                                 size_t                       icount,
                                                 size_t&                      occupancySizeU,
                                                 size_t&                      occupancySizeV,
                                                 const size_t                 safeguard,
                                                 std::vector<bool>&           occupancyMap,
                                                 size_t&                      heightGPA,
                                                 size_t&                      widthGPA,
                                                 size_t&                      maxOccupancyRow );

  //**Point Local Reconstruction**//
  void pointLocalReconstructionSearch( PCCContext&                         context,
                                       size_t                              tileIndex,
                                       const GeneratePointCloudParameters& params );
  void pointLocalReconstructionSearch( PCCContext&                          context,
                                       PCCFrameContext&                     tile,
                                       const std::vector<PCCVideoGeometry>& videoMultiple,
                                       const GeneratePointCloudParameters&  params );

  void setPointLocalReconstruction( PCCContext& context );

  void setPLRData( PCCFrameContext& tile,
                   const PCCPatch&  patch,
                   PLRData&         plrd,
                   size_t           occupancyPackingBlockSize,
                   size_t           patchIndex );
  //**Additional Projection Plane**//
  void segmentationPartiallyAddtinalProjectionPlane( const PCCPointSet3&                 source,
                                                     PCCFrameContext&                    frameContext,
                                                     const PCCPatchSegmenter3Parameters& segmenterParams,
                                                     size_t                              frameIndex,
                                                     float&                              distanceSrcRec );
  //**tools**//
  static inline uint64_t mortonAddr( const int32_t x, const int32_t y, const int32_t z );
  uint64_t               mortonAddr( const PCCPoint3D& vec, int depth );
  void                   create3DMotionEstimationFiles( PCCContext& context, const std::string& path );
  static void            remove3DMotionEstimationFiles( const std::string& path );
  void                   presmoothPointCloudColor( PCCPointSet3& reconstruct, const PCCEncoderParameters params );
  PCCVector3D            calculateWeightNormal( size_t geometryBitDepth3D, const PCCPointSet3& source );

  //**print out**//
  static void printMap( std::vector<bool> img, const size_t sizeU, const size_t sizeV );
  static void printMapTetris( std::vector<bool> img, const size_t sizeU, const size_t sizeV, std::vector<int> horizon );

  PCCEncoderParameters params_;
};

};  // namespace pcc

#endif /* PCCEncoder_h */
