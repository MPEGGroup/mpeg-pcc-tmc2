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

#ifndef PCCPatchSegmenter_h
#define PCCPatchSegmenter_h

#include "PCCCommon.h"
#include <set>

namespace pcc {

typedef std::unordered_map<uint64_t, std::vector<size_t>> Voxels;

class PCCNormalsGenerator3;
class PCCKdTree;
class PCCPatch;

struct PCCPatchSegmenter3Parameters {
  bool             gridBasedSegmentation_;
  size_t           voxelDimensionGridBasedSegmentation_;
  size_t           nnNormalEstimation_;
  size_t           normalOrientation_;
  bool             gridBasedRefineSegmentation_;
  size_t           maxNNCountRefineSegmentation_;
  size_t           iterationCountRefineSegmentation_;
  size_t           voxelDimensionRefineSegmentation_;
  size_t           searchRadiusRefineSegmentation_;
  size_t           occupancyResolution_;
  bool             enablePatchSplitting_;
  size_t           maxPatchSize_;
  size_t           quantizerSizeX_;
  size_t           quantizerSizeY_;
  size_t           minPointCountPerCCPatchSegmentation_;
  size_t           maxNNCountPatchSegmentation_;
  size_t           surfaceThickness_;
  size_t           EOMFixBitCount_;
  bool             EOMSingleLayerMode_;
  size_t           mapCountMinus1_;
  size_t           minLevel_;
  size_t           maxAllowedDepth_;
  double           maxAllowedDist2RawPointsDetection_;
  double           maxAllowedDist2RawPointsSelection_;
  double           lambdaRefineSegmentation_;
  bool             useEnhancedOccupancyMapCode_;
  bool             absoluteD1_;
  bool             createSubPointCloud_;
  bool             surfaceSeparation_;
  PCCVector3D      weightNormal_;
  size_t           additionalProjectionPlaneMode_;
  double           partialAdditionalProjectionPlane_;
  size_t           geometryBitDepth2D_;
  size_t           geometryBitDepth3D_;
  bool             patchExpansion_;
  bool             highGradientSeparation_;
  double           minGradient_;
  size_t           minNumHighGradientPoints_;
  bool             enablePointCloudPartitioning_;
  std::vector<int> roiBoundingBoxMinX_;
  std::vector<int> roiBoundingBoxMaxX_;
  std::vector<int> roiBoundingBoxMinY_;
  std::vector<int> roiBoundingBoxMaxY_;
  std::vector<int> roiBoundingBoxMinZ_;
  std::vector<int> roiBoundingBoxMaxZ_;
  int              numTilesHor_;
  double           tileHeightToWidthRatio_;
  int              numCutsAlong1stLongestAxis_;
  int              numCutsAlong2ndLongestAxis_;
  int              numCutsAlong3rdLongestAxis_;
};

class PCCPatchSegmenter3 {
 public:
  PCCPatchSegmenter3( void ) : nbThread_( 0 ) {}
  PCCPatchSegmenter3( const PCCPatchSegmenter3& ) = delete;
  PCCPatchSegmenter3& operator=( const PCCPatchSegmenter3& ) = delete;
  ~PCCPatchSegmenter3()                                      = default;
  void setNbThread( size_t nbThread );

  void compute( const PCCPointSet3&                 geometry,
                const size_t                        frameIndex,
                const PCCPatchSegmenter3Parameters& params,
                std::vector<PCCPatch>&              patches,
                std::vector<PCCPointSet3>&          subPointCloud,
                float&                              distanceSrcRec );

  void convertPointsToVoxels( const PCCPointSet3& source,
                              size_t              geoBits,
                              size_t              voxDim,
                              PCCPointSet3&       sourceVox,
                              Voxels&             voxels );

  void applyVoxelsDataToPoints( size_t                pointCount,
                                size_t                geoBits,
                                size_t                voxDim,
                                Voxels&               voxels,  // const
                                const PCCPointSet3&   source,
                                PCCNormalsGenerator3& normalsGen,
                                std::vector<size_t>&  partitions );

  void initialSegmentation( const PCCPointSet3&         geometry,
                            const PCCNormalsGenerator3& normalsGen,
                            const PCCVector3D*          orientations,
                            const size_t                orientationCount,
                            std::vector<size_t>&        partition,
                            const PCCVector3D&          axisWeight );
  void initialSegmentation( const PCCPointSet3&         geometry,
                            const PCCNormalsGenerator3& normalsGen,
                            const PCCVector3D*          orientations,
                            const size_t                orientationCount,
                            std::vector<size_t>&        partition );
  void computeAdjacencyInfo( const PCCPointSet3&               pointCloud,
                             const PCCKdTree&                  kdtree,
                             std::vector<std::vector<size_t>>& adj,
                             const size_t                      maxNNCount );

  void computeAdjacencyInfoDist( const PCCPointSet3&               pointCloud,
                                 const PCCKdTree&                  kdtree,
                                 std::vector<std::vector<size_t>>& adj,
                                 std::vector<std::vector<double>>& adjDist,
                                 const size_t                      maxNNCount );

  void computeAdjacencyInfoInRadius( const PCCPointSet3&                 pointCloud,
                                     const PCCKdTree&                    kdtree,
                                     std::vector<std::vector<uint32_t>>& adj,
                                     const size_t                        maxNNCount,
                                     const size_t                        radius );

  bool colorSimilarity( PCCColor3B& colorD1candidate, PCCColor3B& colorD0, uint8_t threshold ) {
    bool bSimilarity = ( std::abs( colorD0[0] - colorD1candidate[0] ) < threshold ) &&
                       ( std::abs( colorD0[1] - colorD1candidate[1] ) < threshold ) &&
                       ( std::abs( colorD0[2] - colorD1candidate[2] ) < threshold );
    return bSimilarity;
  }

  int64_t colorDifference( PCCColor3B& D0_Color, PCCColor3B& D1_Color ) {
    int32_t delta_R = int32_t( D0_Color[0] ) - int32_t( D1_Color[0] );
    int32_t delta_G = int32_t( D0_Color[1] ) - int32_t( D1_Color[1] );
    int32_t delta_B = int32_t( D0_Color[2] ) - int32_t( D1_Color[2] );
    int64_t delta_e = delta_R * delta_R + delta_G * delta_G + delta_B * delta_B;
    return delta_e;
  }
  void resampledPointcloud( std::vector<size_t>& pointCount,
                            PCCPointSet3&        resampled,
                            std::vector<size_t>& resampledPatchPartition,
                            PCCPatch&            patch,
                            size_t               patchIndex,
                            bool                 multipleMaps,
                            size_t               surfaceThickness,
                            size_t               EOMFixBitCount,
                            bool                 bIsAdditionalProjectionPlane,
                            bool                 useEnhancedOccupancyMapCode,
                            size_t               geometryBitDepth3D,
                            bool                 createSubPointCloud,
                            PCCPointSet3&        rec );

  int16_t getPatchSurfaceThickness( const PCCPointSet3&      points,
                                    PCCPatch&                patch,
                                    size_t                   patchIndex,
                                    std::vector<PCCColor3B>& frame_pcc_color,
                                    std::vector<size_t>&     connectedComponent,
                                    size_t                   surfaceThickness,
                                    size_t                   projectionMode,
                                    bool                     bIsAdditionalProjectionPlane,
                                    size_t                   geometryBitDepth3D );

  void segmentPatches( const PCCPointSet3&                 points,
                       const size_t                        frameIndex,
                       const PCCKdTree&                    kdtree,
                       const PCCPatchSegmenter3Parameters& params,
                       std::vector<size_t>&                partition,
                       std::vector<PCCPatch>&              patches,
                       std::vector<size_t>&                patchPartition,
                       std::vector<size_t>&                resampledPatchPartition,
                       std::vector<size_t>                 rawPoints,
                       PCCPointSet3&                       resampled,
                       std::vector<PCCPointSet3>&          subPointCloud,
                       float&                              distanceSrcRec,
                       const PCCNormalsGenerator3&         normalsGen,
                       const PCCVector3D*                  orientations,
                       const size_t                        orientationCount );

  void refineSegmentation( const PCCPointSet3&         pointCloud,
                           const PCCKdTree&            kdtree,
                           const PCCNormalsGenerator3& normalsGen,
                           const PCCVector3D*          orientations,
                           const size_t                orientationCount,
                           const size_t                maxNNCount,
                           const double                lambda,
                           const size_t                iterationCount,
                           std::vector<size_t>&        partition );

  void refineSegmentationGridBased( const PCCPointSet3&         pointCloud,
                                    const PCCNormalsGenerator3& normalsGen,
                                    const PCCVector3D*          orientations,
                                    const size_t                orientationCount,
                                    const size_t                maxNNCount,
                                    const double                lambda,
                                    const size_t                iterationCount,
                                    const size_t                voxDim,
                                    const size_t                searchRadiusRefineSegmentation,
                                    std::vector<size_t>&        partition );

 private:
  size_t                nbThread_;
  std::vector<PCCPatch> boxMinDepths_;  // box depth list
  std::vector<PCCPatch> boxMaxDepths_;  // box depth list

  void convert( size_t axis, size_t lod, PCCPoint3D input, PCCPoint3D& output ) {
    size_t shif = ( 1u << ( lod - 1 ) ) - 1;
    if ( axis == 1 ) {
      output.x() = input.x() + input.z();
      output.y() = input.y();
      output.z() = -input.x() + input.z() + shif;
    }
    if ( axis == 2 ) {
      output.x() = input.x();
      output.y() = -input.z() + input.y() + shif;
      output.z() = input.z() + input.y();
    }
    if ( axis == 3 ) {
      output.x() = -input.y() + input.x() + shif;
      output.y() = input.y() + input.x();
      output.z() = input.z();
    }
  }

  void iconvert( size_t axis, size_t lod, PCCVector3D input, PCCVector3D& output ) {
    size_t shif = ( 1u << ( lod - 1 ) ) - 1;
    output      = input;
    if ( axis == 1 ) {
      output.x() = ( input.x() - input.z() + shif ) / 2.0;
      output.y() = input.y();
      output.z() = ( input.x() + input.z() - shif ) / 2.0;
    }
    if ( axis == 2 ) {
      output.x() = input.x();
      output.y() = ( input.z() + input.y() - shif ) / 2.0;
      output.z() = ( input.z() - input.y() + shif ) / 2.0;
    }
    if ( axis == 3 ) {
      output.x() = ( input.y() + input.x() - shif ) / 2.0;
      output.y() = ( input.y() - input.x() + shif ) / 2.0;
      output.z() = input.z();
    }
  }

  void        separateHighGradientPoints( const PCCPointSet3&               points,
                                          const size_t                      additionalProjectionAxis,
                                          const bool                        absoluteD1,
                                          const PCCNormalsGenerator3&       normalsGen,
                                          const PCCVector3D*                orientations,
                                          const size_t                      orientationCount,
                                          const size_t                      surfaceThickness,
                                          const size_t                      geometryBitDepth3D,
                                          const double                      minGradient,
                                          const size_t                      minNumHighGradientPoints,
                                          std::vector<size_t>&              partition,
                                          std::vector<std::vector<size_t>>& adj,
                                          std::vector<std::vector<size_t>>& connectedComponents );
  static void determinePatchOrientation( const size_t         additionalProjectionAxis,
                                         const bool           absoluteD1,
                                         bool&                bIsAdditionalProjectionPlane,
                                         PCCPatch&            patch,
                                         std::vector<size_t>& partition,
                                         std::vector<size_t>& connectedComponent );
  void        generatePatchD0( const PCCPointSet3&  points,
                               const size_t         geometryBitDepth3D,
                               const bool           bIsAdditionalProjectionPlane,
                               PCCPatch&            patch,
                               std::vector<size_t>& connectedComponent );
  void        calculateGradient( const PCCPointSet3&               points,
                                 const std::vector<size_t>&        connectedComponent,
                                 const PCCNormalsGenerator3&       normalsGen,
                                 const PCCVector3D*                orientations,
                                 const size_t                      orientationCount,
                                 const size_t                      orgPartitionIdx,
                                 const size_t                      surfaceThickness,
                                 const size_t                      geometryBitDepth3D,
                                 const bool                        bIsAdditionalProjectionPlane,
                                 const double                      minGradient,
                                 const size_t                      minNumHighGradientPoints,
                                 PCCPatch&                         patch,
                                 std::vector<std::vector<size_t>>& adj,
                                 std::vector<std::vector<size_t>>& highGradientConnectedComponents,
                                 std::vector<bool>&                isRemoved );

  PCCVector3D orientations6[6] = {
      PCCVector3D( 1.0, 0.0, 0.0 ),  PCCVector3D( 0.0, 1.0, 0.0 ),  PCCVector3D( 0.0, 0.0, 1.0 ),
      PCCVector3D( -1.0, 0.0, 0.0 ), PCCVector3D( 0.0, -1.0, 0.0 ), PCCVector3D( 0.0, 0.0, -1.0 ),
  };
  const size_t orientationCount6 = 6;
  //  const size_t orientation10Count = 10;
  PCCVector3D orientations10_XAxis[10] = {
      PCCVector3D( 1.0, 0.0, 0.0 ),                      // 0
      PCCVector3D( 0.0, 1.0, 0.0 ),                      // 1
      PCCVector3D( 0.0, 0.0, 1.0 ),                      // 2
      PCCVector3D( -1.0, 0.0, 0.0 ),                     // 3
      PCCVector3D( 0.0, -1.0, 0.0 ),                     // 4
      PCCVector3D( 0.0, 0.0, -1.0 ),                     // 5
      PCCVector3D( 0.0, sqrt( 2 ) / 2, sqrt( 2 ) / 2 ),  // 6
      // PCCVector3D( 0.0, -sqrt( 2 ) / 2, sqrt( 2 ) / 2 ),   // 7
      PCCVector3D( 0.0, sqrt( 2 ) / 2, -sqrt( 2 ) / 2 ),   // 7
      PCCVector3D( 0.0, -sqrt( 2 ) / 2, -sqrt( 2 ) / 2 ),  // 8
      // PCCVector3D( 0.0, sqrt( 2 ) / 2, -sqrt( 2 ) / 2 ),   // 9
      PCCVector3D( 0.0, -sqrt( 2 ) / 2, sqrt( 2 ) / 2 ),  // 9
  };

  PCCVector3D orientations10_YAxis[10] = {
      PCCVector3D( 1.0, 0.0, 0.0 ),                        // 0
      PCCVector3D( 0.0, 1.0, 0.0 ),                        // 1
      PCCVector3D( 0.0, 0.0, 1.0 ),                        // 2
      PCCVector3D( -1.0, 0.0, 0.0 ),                       // 3
      PCCVector3D( 0.0, -1.0, 0.0 ),                       // 4
      PCCVector3D( 0.0, 0.0, -1.0 ),                       // 5
      PCCVector3D( sqrt( 2 ) / 2, 0.0, sqrt( 2 ) / 2 ),    // 6
      PCCVector3D( -sqrt( 2 ) / 2, 0.0, sqrt( 2 ) / 2 ),   // 7
      PCCVector3D( -sqrt( 2 ) / 2, 0.0, -sqrt( 2 ) / 2 ),  // 8
      PCCVector3D( sqrt( 2 ) / 2, 0.0, -sqrt( 2 ) / 2 ),   // 9
  };

  PCCVector3D orientations10_ZAxis[10] = {
      PCCVector3D( 1.0, 0.0, 0.0 ),                      // 0
      PCCVector3D( 0.0, 1.0, 0.0 ),                      // 1
      PCCVector3D( 0.0, 0.0, 1.0 ),                      // 2
      PCCVector3D( -1.0, 0.0, 0.0 ),                     // 3
      PCCVector3D( 0.0, -1.0, 0.0 ),                     // 4
      PCCVector3D( 0.0, 0.0, -1.0 ),                     // 5
      PCCVector3D( sqrt( 2 ) / 2, sqrt( 2 ) / 2, 0.0 ),  // 6
      // PCCVector3D( -sqrt( 2 ) / 2, sqrt( 2 ) / 2, 0.0 ),   // 7
      PCCVector3D( sqrt( 2 ) / 2, -sqrt( 2 ) / 2, 0.0 ),   // 7
      PCCVector3D( -sqrt( 2 ) / 2, -sqrt( 2 ) / 2, 0.0 ),  // 8
      // PCCVector3D( sqrt( 2 ) / 2, -sqrt( 2 ) / 2, 0.0 ),   // 9
      PCCVector3D( -sqrt( 2 ) / 2, sqrt( 2 ) / 2, 0.0 ),  // 9
  };

  //  const size_t orientation18Count = 18;
  PCCVector3D orientations18[18] = {
      PCCVector3D( 1.0, 0.0, 0.0 ),                        // 0
      PCCVector3D( 0.0, 1.0, 0.0 ),                        // 1
      PCCVector3D( 0.0, 0.0, 1.0 ),                        // 2
      PCCVector3D( -1.0, 0.0, 0.0 ),                       // 3
      PCCVector3D( 0.0, -1.0, 0.0 ),                       // 4
      PCCVector3D( 0.0, 0.0, -1.0 ),                       // 5
      PCCVector3D( sqrt( 2 ) / 2, 0.0, sqrt( 2 ) / 2 ),    // 6   1
      PCCVector3D( -sqrt( 2 ) / 2, 0.0, sqrt( 2 ) / 2 ),   // 7
      PCCVector3D( -sqrt( 2 ) / 2, 0.0, -sqrt( 2 ) / 2 ),  // 8
      PCCVector3D( sqrt( 2 ) / 2, 0.0, -sqrt( 2 ) / 2 ),   // 9
      PCCVector3D( 0.0, sqrt( 2 ) / 2, sqrt( 2 ) / 2 ),    // 10   2
      // PCCVector3D( 0.0, -sqrt( 2 ) / 2, sqrt( 2 ) / 2 ),   // 11
      PCCVector3D( 0.0, sqrt( 2 ) / 2, -sqrt( 2 ) / 2 ),   // 11
      PCCVector3D( 0.0, -sqrt( 2 ) / 2, -sqrt( 2 ) / 2 ),  // 12
      // PCCVector3D( 0.0, sqrt( 2 ) / 2, -sqrt( 2 ) / 2 ),   // 13
      PCCVector3D( 0.0, -sqrt( 2 ) / 2, sqrt( 2 ) / 2 ),  // 13
      PCCVector3D( sqrt( 2 ) / 2, sqrt( 2 ) / 2, 0.0 ),   // 14    3
      // PCCVector3D( -sqrt( 2 ) / 2, sqrt( 2 ) / 2, 0.0 ),   // 15
      PCCVector3D( sqrt( 2 ) / 2, -sqrt( 2 ) / 2, 0.0 ),   // 15
      PCCVector3D( -sqrt( 2 ) / 2, -sqrt( 2 ) / 2, 0.0 ),  // 16
      // PCCVector3D( sqrt( 2 ) / 2, -sqrt( 2 ) / 2, 0.0 ),   // 17
      PCCVector3D( -sqrt( 2 ) / 2, sqrt( 2 ) / 2, 0.0 ),  // 17
  };
};

class Rect {
 public:
  Rect& operator=( const Rect& ) = delete;
  ~Rect()                        = default;

  Rect() { x_ = y_ = width_ = height_ = 0; }
  Rect( int x, int y, int w, int h ) {
    x_      = x;
    y_      = y;
    width_  = w;
    height_ = h;
  }
  int area() { return ( width_ * height_ ); }

  Rect operator&( const Rect& rhs ) {
    Rect c;
    int  x1   = ( x_ > rhs.x_ ) ? x_ : rhs.x_;
    int  y1   = ( y_ > rhs.y_ ) ? y_ : rhs.y_;
    c.width_  = ( ( ( x_ + width_ ) < ( rhs.x_ + rhs.width_ ) ) ? x_ + width_ : rhs.x_ + rhs.width_ ) - x1;
    c.height_ = ( ( ( y_ + height_ ) < ( rhs.y_ + rhs.height_ ) ) ? y_ + height_ : rhs.y_ + rhs.height_ ) - y1;
    c.x_      = x1;
    c.y_      = y1;
    if ( c.width_ <= 0 || c.height_ <= 0 ) { c.x_ = c.y_ = c.width_ = c.height_ = 0; }
    return Rect( c );
  }

 private:
  int width_;
  int height_;
  int x_;
  int y_;
};

typedef std::vector<uint32_t> PointIndicesVector_t;
typedef std::vector<uint16_t> ScoresVector_t;

typedef enum {
  NO_EDGE       = 0x00,  // one ppi-vaue in a voxel
  INDIRECT_EDGE = 0x01,  // adjcent voxels of M_DIRECT_EDGE, S_DIRECT_EDGE
  M_DIRECT_EDGE = 0x10,  // multiple points && more than two ppi-values in a voxel
  S_DIRECT_EDGE = 0x11   // single-point in a voxel, considered as a direct edge-voxel
} VoxEdge;

// holds the point indices of every point in a given grid cell
class PointIndicesOfGridCell {
 public:
  PointIndicesOfGridCell() = default;
  PointIndicesOfGridCell( const uint32_t uiMaxNumPoints ) :
      m_pPointIndices( std::make_unique<PointIndicesVector_t>() ) {
    m_pPointIndices->reserve( uiMaxNumPoints );
  }

  inline PointIndicesVector_t* getPointIndices() { return m_pPointIndices.get(); }
  inline void                  addPointIndex( uint32_t uiPointIdx ) { m_pPointIndices->push_back( uiPointIdx ); }
  inline uint8_t               getPointCount() { return (uint8_t)m_pPointIndices->size(); }

 protected:
  std::unique_ptr<PointIndicesVector_t> m_pPointIndices;
};

// holds the scoreSmooths of the points in a cell
class AttributeOfGridCell {
 public:
  AttributeOfGridCell() = default;

  AttributeOfGridCell( const size_t orientationCount ) {
    m_pScoreSmooth  = std::make_unique<ScoresVector_t>( orientationCount, 0 );
    m_uiUpdateFlag  = true;
    m_uiTotalPoints = 0;
  }

  inline ScoresVector_t* getScoreSmooth() { return m_pScoreSmooth.get(); }

  inline void updateScores( const PointIndicesVector_t& point_indices,
                            const std::vector<size_t>&  partitions ) noexcept {
    // clears the scores
    std::fill( m_pScoreSmooth->begin(), m_pScoreSmooth->end(), 0 );
    auto& scores = *m_pScoreSmooth;
    for ( const auto& j : point_indices ) { ++scores[partitions[j]]; }

    // update voxel type (1st voxel classification)
    updateVoxelTypeWithScores();
  }

  inline uint8_t getEdge() { return m_uiVoxEdge; }
  inline uint8_t getPPI() { return m_uiVoxPpi; }

  inline void updateEdge( uint8_t uiVoxEdge ) { m_uiVoxEdge = uiVoxEdge; }
  inline void setUpdatedFlag() { m_uiUpdateFlag = true; }

  inline void updateVoxelTypeWithScores() {
    if ( !m_uiUpdateFlag ) { return; }

    // update edge-type
    if ( m_uiVoxEdge != S_DIRECT_EDGE ) {
      size_t uniformityIdx = m_pScoreSmooth->size() - std::count( m_pScoreSmooth->begin(), m_pScoreSmooth->end(), 0 );
      m_uiVoxEdge          = ( uniformityIdx == 1 ) ? NO_EDGE : M_DIRECT_EDGE;
    }

    // update _ppi
    const auto& maxScore = std::max_element( m_pScoreSmooth->begin(), m_pScoreSmooth->end() );
    m_uiVoxPpi           = (uint8_t)std::distance( m_pScoreSmooth->begin(), maxScore );

    m_uiUpdateFlag = false;
  }

  inline void setNumOfTotalPoints( uint8_t uiPointCount ) {
    m_uiTotalPoints = uiPointCount;
    m_uiVoxEdge     = ( m_uiTotalPoints == 1 ) ? S_DIRECT_EDGE : M_DIRECT_EDGE;
  }

 protected:
  std::unique_ptr<ScoresVector_t> m_pScoreSmooth;
  uint8_t                         m_uiVoxEdge;      //
  uint8_t                         m_uiUpdateFlag;   // TRUE, FALSE
  uint8_t                         m_uiTotalPoints;  // voxDim * voxDim * voxDim
  uint8_t                         m_uiVoxPpi;       // 0 - (orientationCount-1) [6, 10 ..etc]
};

float computeIOU( Rect a, Rect b );

}  // namespace pcc

#endif /* PCCPatchSegmenter_h */
