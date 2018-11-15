
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

#include "../../../../dependencies/tbb/include/tbb/compat/condition_variable"
#include "PCCCommon.h"
#include "PCCImage.h"
#include "PCCMath.h"
#include "PCCVideo.h"

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
typedef pcc::PCCVideo<uint8_t,  3> PCCVideoTexture;
typedef pcc::PCCVideo<uint16_t, 3> PCCVideoGeometry;

template <typename T, size_t N>
class PCCImage;
typedef pcc::PCCImage<uint16_t, 3> PCCImageGeometry;
struct GeneratePointCloudParameters {
  size_t             occupancyResolution_;
  size_t             occupancyPrecision_;
  bool               gridSmoothing_;
  size_t             neighborCountSmoothing_;
  double             radius2Smoothing_;
  double             radius2BoundaryDetection_;
  double             thresholdSmoothing_;
  bool               losslessGeo_;
  bool               losslessGeo444_;
  size_t             nbThread_;
  bool               absoluteD1_;
  size_t             surfaceThickness;
  bool               ignoreLod_;
  double             thresholdColorSmoothing_;
  double             thresholdLocalEntropy_;
  double             radius2ColorSmoothing_;
  size_t             neighborCountColorSmoothing_;
  bool               flagColorSmoothing_;
  bool               enhancedDeltaDepthCode_;
  bool               deltaCoding_;
  bool               oneLayerMode_;
  bool               singleLayerPixelInterleaving_;
  bool               sixDirectionMode_;
  std::string        path_;
};

class PCCCodec {
 public:
  PCCCodec();
  ~PCCCodec();

  void generatePointCloud( PCCGroupOfFrames& reconstructs, PCCContext& context,
                           const GeneratePointCloudParameters params );


  bool colorPointCloud( PCCGroupOfFrames& reconstructs, PCCContext& context,
                        const bool noAttributes, const ColorTransform colorTransform,
                        const GeneratePointCloudParameters params);

 protected:
  int getDeltaNeighbors( const PCCImageGeometry& frame,
                         const PCCPatch& patch,
                         const int xOrg,
                         const int yOrg,
                         const int neighboring,
                         const int threshold,
                         const bool   projectionMode,
                         const double lodScale);

  std::vector<PCCVector3D> generatePoints( const GeneratePointCloudParameters& params,
                                           PCCFrameContext &frame,
                                           const PCCVideoGeometry &video,
                                           const PCCVideoGeometry &videoD1,
                                           const size_t shift,
                                           const size_t patchIndex,
                                           const size_t u,
                                           const size_t v,
                                           const size_t x,
                                           const size_t y,
                                           const bool   interpolate,
                                           const bool   filling,
                                           const size_t minD1,
                                           const size_t neighbor,
                                           const double lodScale );

  inline double entropy(std::vector<uint8_t> &Data, int N) {
    std::vector<size_t> count;
    count.resize(256, 0);
    for (size_t i = 0; i < N; ++i) {
      ++count[size_t(Data[i])];
    }
    double s = 0;
    for (size_t i = 0; i < 256; ++i) {
      if (count[i]) {
        double p = double(count[i]) / double(N);
        s += -p * std::log2(p);
      }
    }
    return s;
  }


 private:
  void generatePointCloud( PCCPointSet3& reconstruct, PCCFrameContext& frame,
                           const PCCVideoGeometry &video, const PCCVideoGeometry &videoD1,
                           const GeneratePointCloudParameters params,
                           std::vector<uint32_t> &partition );

  void smoothPointCloud( PCCPointSet3& reconstruct,
                         const std::vector<uint32_t> &partition,
                         const GeneratePointCloudParameters params );

  void createSubReconstruct( const PCCPointSet3&                 reconstruct,
                             const std::vector<uint32_t>&        partition,
                             PCCFrameContext&                    frame,
                             const GeneratePointCloudParameters& params,
                             const size_t                        frameCount,
                             PCCPointSet3&                       subReconstruct,
                             std::vector<uint32_t>&              subPartition,
                             std::vector<size_t>&                subReconstructIndex );

  void createSpecificLayerReconstruct( const PCCPointSet3&                 reconstruct,
                                       const std::vector<uint32_t>&        partition,
                                       PCCFrameContext&                    frame,
                                       const GeneratePointCloudParameters& params,
                                       const size_t                        frameCount,
                                       PCCPointSet3&                       subReconstruct,
                                       std::vector<uint32_t>&              subPartition,
                                       std::vector<size_t>&                subReconstructIndex );

  void updateReconstruct( PCCPointSet3& reconstruct,
                          const PCCPointSet3& subReconstruct,
                          const std::vector<size_t>& subReconstructIndex );

  bool colorPointCloud( PCCPointSet3& reconstruct, PCCFrameContext& frame,
                        const PCCVideoTexture &video, const bool noAttributes,
                        const GeneratePointCloudParameters& params,
                        const size_t frameCount );

  void smoothPointCloudColor(PCCPointSet3& reconstruct, const GeneratePointCloudParameters params);

  void smoothPointCloudGrid( PCCPointSet3& reconstruct,
                             const std::vector<uint32_t> &partition,
                             const GeneratePointCloudParameters params );

  void addGridCentroid( PCCVector3D& point, int patchIdx,
                        std::vector<int>& cnt, std::vector<PCCVector3D>& center_grid,
                        std::vector<int> &gpartition, std::vector<bool>& doSmooth, int grid);


  bool gridFiltering( const std::vector<uint32_t> &partition, PCCPointSet3 &pointCloud,
                      PCCVector3D& curPos, PCCVector3D& centroid, int &cnt,
                      std::vector<int>& gcnt, std::vector<PCCVector3D>& center_grid, std::vector<bool>& doSmooth, int grid );


  void identifyBoundaryPoints( const std::vector<uint32_t>& occupancyMap,
                               const size_t x,
                               const size_t y,
                               const size_t imageWidth,
                               const size_t imageHeight,
                               const size_t pointindex_1,
                               std::vector<uint32_t>& PBflag,
                               PCCPointSet3& reconstruct );

  std::vector<int> gcnt_;
  std::vector<PCCVector3D> center_grid_;
  std::vector<bool> doSmooth_;
  std::vector<int> gpartition_;

};

}; //~namespace

#endif /* PCCCodec_h */
