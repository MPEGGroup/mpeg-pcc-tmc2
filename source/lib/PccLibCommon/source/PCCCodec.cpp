
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
#include "PCCCommon.h"

#include "PCCVideo.h"
#include "PCCPointSet.h"
#include "tbb/tbb.h"
#include "PCCKdTree.h"
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCGroupOfFrames.h"
#include "PCCPatch.h"

#include "PCCCodec.h"

using namespace pcc;

PCCCodec::PCCCodec() {}
PCCCodec::~PCCCodec() {}

void PCCCodec::generatePointCloud( PCCGroupOfFrames& reconstructs, PCCContext& context,
                                   const GeneratePointCloudParameters params ) {
  auto& frames = context.getFrames();
  auto &videoGeometry = context.getVideoGeometry();
  auto &videoGeometryD1 = context.getVideoGeometryD1();
  for( size_t i = 0; i < frames.size(); i++ ) {
    std::vector<uint32_t> partition;
    generatePointCloud( reconstructs[i], frames[i], videoGeometry, videoGeometryD1, params, partition );
    if (!params.losslessGeo_ ) {
      smoothPointCloud( reconstructs[i], partition, params );
    }
  }
}

bool PCCCodec::colorPointCloud( PCCGroupOfFrames& reconstructs, PCCContext& context,
                                const bool noAttributes, const ColorTransform colorTransform ) {
  auto &video = context.getVideoTexture();
  auto& frames = context.getFrames();
  for( size_t i = 0; i < frames.size(); i++ ) {
    colorPointCloud( reconstructs[i], frames[i],  video, noAttributes );
    if ( colorTransform == COLOR_TRANSFORM_RGB_TO_YCBCR ) {
      reconstructs[i].convertYUVToRGB();
    }
  }
  return true;
}

void PCCCodec::generatePointCloud( PCCPointSet3& reconstruct, PCCFrameContext &frame,
                                   const PCCVideoGeometry &video, const PCCVideoGeometry &videoD1,
                                   const GeneratePointCloudParameters params,
                                   std::vector<uint32_t> &partition ) {
  auto& patches      = frame.getPatches();
  auto& pointToPixel = frame.getPointToPixel();
  auto& blockToPatch = frame.getBlockToPatch();
  auto& occupancyMap = frame.getOccupancyMap();
  partition.resize(0);
  pointToPixel.resize(0);
  reconstruct.clear();

  size_t shift;
  const size_t layerCount = 2;
  if (!params.absoluteD1_) {
    shift = frame.getIndex();
    if (video.getFrameCount() < (shift + 1)) {
      return;
    }
  } else {
    shift = frame.getIndex() * 2;
    if (video.getFrameCount() < (shift + layerCount)) {
      return;
    }
  }
  const auto &frame0 = video.getFrame(shift);
  const size_t imageWidth = video.getWidth();
  const size_t blockToPatchWidth = frame.getWidth() / params.occupancyResolution_;
  reconstruct.addColors();
  const size_t patchCount = patches.size();
  for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
    const size_t patchIndexPlusOne = patchIndex + 1;
    const auto &patch = patches[patchIndex];
    PCCColor3B color(uint8_t(0));
    while (color[0] == color[1] || color[2] == color[1] || color[2] == color[0]) {
      color[0] = static_cast<uint8_t>(rand() % 32) * 8;
      color[1] = static_cast<uint8_t>(rand() % 32) * 8;
      color[2] = static_cast<uint8_t>(rand() % 32) * 8;
    }
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        if (blockToPatch[(v0 + patch.getV0()) * blockToPatchWidth + u0 + patch.getU0()] ==
            patchIndexPlusOne) {
          for (size_t v1 = 0; v1 < patch.getOccupancyResolution(); ++v1) {
            const size_t v = v0 * patch.getOccupancyResolution() + v1;
            for (size_t u1 = 0; u1 < patch.getOccupancyResolution(); ++u1) {
              const size_t u = u0 * patch.getOccupancyResolution() + u1;
              const size_t x = patch.getU0() * patch.getOccupancyResolution() + u;
              const size_t y = patch.getV0() * patch.getOccupancyResolution() + v;
              const bool occupancy = occupancyMap[y * imageWidth + x] != 0;
              if (!occupancy) {
                continue;
              }

              PCCVector3D point0;
              point0[patch.getNormalAxis()] = double(frame0.getValue(0, x, y) + patch.getD1());
              point0[patch.getTangentAxis()] = double(u) + patch.getU1();
              point0[patch.getBitangentAxis()] = double(v) + patch.getV1();
              for (size_t f = 0; f < layerCount; ++f) {
                PCCVector3D point1(point0);
                if (f > 0) {
                  if( !params.absoluteD1_ ) {
                    const auto &frame1 = videoD1.getFrame(shift);
                    point1[patch.getNormalAxis()] += frame1.getValue(0, x, y);
                  } else {
                    const auto &frame1 = video.getFrame(f + shift);
                    point1[patch.getNormalAxis()] = double( frame1.getValue(0, x, y) + patch.getD1());
                  }
                }
                const size_t pointindex_1 = reconstruct.addPoint(point1);
                reconstruct.setColor(pointindex_1, color);
                if( PCC_SAVE_POINT_TYPE == 1 ) {
                  reconstruct.setType( pointindex_1, f == 0 ? PointType::D0 : PointType::D1 );
                }
                partition.push_back(uint32_t(patchIndex));
                pointToPixel.push_back(PCCVector3<size_t>(x, y, f));
              }
            }
          }
        }
      }
    }
  }
  // Add points from missedPointsPatch
  auto& missedPointsPatch = frame.getMissedPointsPatch();
  PCCColor3B missedPointsColor(uint8_t(0));
  missedPointsColor[0] = 0;
  missedPointsColor[1] = 255;
  missedPointsColor[2] = 255;
  if (params.losslessGeo_ && params.losslessGeo444_) {
    for (size_t v0 = 0; v0 < missedPointsPatch.sizeV0; ++v0) {
      for (size_t u0 = 0; u0 < missedPointsPatch.sizeU0; ++u0) {
        for (size_t v1 = 0; v1 < missedPointsPatch.occupancyResolution; ++v1) {
          const size_t v = v0 * missedPointsPatch.occupancyResolution + v1;
          for (size_t u1 = 0; u1 < missedPointsPatch.occupancyResolution; ++u1) {
            const size_t u = u0 * missedPointsPatch.occupancyResolution + u1;
            const size_t x = missedPointsPatch.u0 * missedPointsPatch.occupancyResolution + u;
            const size_t y = missedPointsPatch.v0 * missedPointsPatch.occupancyResolution + v;
            const bool occupancy = occupancyMap[y * imageWidth + x] != 0;
            if (!occupancy) {
              continue;
            }
            PCCVector3D point0;
            point0[0] = double(frame0.getValue(0, x, y));
            point0[1] = double(frame0.getValue(1, x, y));
            point0[2] = double(frame0.getValue(2, x, y));
            const size_t pointIndex = reconstruct.addPoint(point0);
            reconstruct.setColor(pointIndex, missedPointsColor);
            for (size_t f = 0; f < layerCount; ++f)
              pointToPixel.push_back(PCCVector3<size_t>(x, y, f));
          }
        }
      }
    }
  } else if (params.losslessGeo_) {
    size_t numMissedPts{ 0 };
    for (size_t v0 = 0; v0 < missedPointsPatch.sizeV0; ++v0) {
      for (size_t u0 = 0; u0 < missedPointsPatch.sizeU0; ++u0) {
        for (size_t v1 = 0; v1 < missedPointsPatch.occupancyResolution; ++v1) {
          const size_t v = v0 * missedPointsPatch.occupancyResolution + v1;
          for (size_t u1 = 0; u1 < missedPointsPatch.occupancyResolution; ++u1) {
            const size_t u = u0 * missedPointsPatch.occupancyResolution + u1;
            const size_t x = missedPointsPatch.u0 * missedPointsPatch.occupancyResolution + u;
            const size_t y = missedPointsPatch.v0 * missedPointsPatch.occupancyResolution + v;
            if (occupancyMap[y * imageWidth + x] != 0)
              numMissedPts++;
          }
        }
      }
    }
    numMissedPts /= 3;
    std::vector<PCCVector3D> missedPoints;
    missedPoints.resize(numMissedPts);
    size_t numMissedPointsAdded{ 0 };
    size_t missedPointsPatchWidth = missedPointsPatch.sizeU0 * missedPointsPatch.occupancyResolution;
    size_t missedPointsPatchHeight = missedPointsPatch.sizeV0 * missedPointsPatch.occupancyResolution;
    const size_t v0 = missedPointsPatch.v0 * missedPointsPatch.occupancyResolution;
    const size_t u0 = missedPointsPatch.u0 * missedPointsPatch.occupancyResolution;
    for (size_t v = 0; v < missedPointsPatchHeight; ++v) {
      for (size_t u = 0; u < missedPointsPatchWidth; ++u) {
        const size_t x = (u0 + u);
        const size_t y = (v0 + v);
        const bool occupancy = occupancyMap[y * imageWidth + x] != 0;
        if (!occupancy) {
          continue;
        }
        if (numMissedPointsAdded < numMissedPts) {
          missedPoints[numMissedPointsAdded][0] = double(frame0.getValue(0, x, y));
        } else if (numMissedPointsAdded <= numMissedPointsAdded && numMissedPointsAdded < 2 * numMissedPts) {
          missedPoints[numMissedPointsAdded - numMissedPts][1] = double(frame0.getValue(0, x, y));
        } else if (2 * numMissedPts <= numMissedPointsAdded && numMissedPointsAdded < 3 * numMissedPts) {
          missedPoints[numMissedPointsAdded - 2 * numMissedPts][2] = double(frame0.getValue(0, x, y));
          const size_t pointIndex = reconstruct.addPoint(missedPoints[numMissedPointsAdded - 2 * numMissedPts]);
          reconstruct.setColor(pointIndex, missedPointsColor);
          for (size_t f = 0; f < layerCount; ++f) {
            pointToPixel.push_back(PCCVector3<size_t>(x, y, f));
          }
        }
        numMissedPointsAdded++;
      }
    }
  }
}

void PCCCodec::smoothPointCloud( PCCPointSet3& reconstruct,
                                 const std::vector<uint32_t> &partition,
                                 const GeneratePointCloudParameters params ) {
  const size_t pointCount = reconstruct.getPointCount();
  PCCStaticKdTree3 kdtree;
  kdtree.build(reconstruct);
  PCCPointSet3 temp;
  temp.resize(pointCount);
  tbb::task_arena limited( (int)params.nbThread_ );
  limited.execute([&]{ 
    tbb::parallel_for(size_t( 0 ), pointCount, [&](const size_t i) {
      //  for (size_t i = 0; i < pointCount; ++i) {
      const size_t maxNeighborCount = 512;
      PCCPointDistInfo nNeighbor[maxNeighborCount];
      PCCNNResult result = {nNeighbor, 0};
      const double maxDist = ceil( sqrt( params.radius2Smoothing_ ) );
      PCCNNQuery3 query = {PCCVector3D(0.0), maxDist, (std::min)(maxNeighborCount, params.neighborCountSmoothing_)};

      const size_t clusterindex_ = partition[i];
      query.point = reconstruct[i];
      kdtree.findNearestNeighbors(query, result);
      assert(result.resultCount);
      PCCVector3D centroid(0.0);
      bool otherClusterPointCount = false;
      size_t neighborCount = 0;
      for (size_t r = 0; r < result.resultCount; ++r) {
        const double dist2 = result.neighbors[r].dist2;
        if (dist2 > params.radius2Smoothing_) {
          break;
        }
        ++neighborCount;
        const size_t pointindex_ = result.neighbors[r].index;
        centroid += reconstruct[pointindex_];
        otherClusterPointCount |=
            (dist2 <= params.radius2BoundaryDetection_) && (partition[pointindex_] != clusterindex_);
      }

      if (otherClusterPointCount) {
        const auto sclaedPoint = double(neighborCount) * query.point;
        const double distToCentroid2 =
            int64_t((centroid - sclaedPoint).getNorm2() + (neighborCount / 2.0)) /
            double(neighborCount);
        for (size_t k = 0; k < 3; ++k) {
          centroid[k] = double(int64_t(centroid[k] + (neighborCount / 2)) / neighborCount);
        }
        if (distToCentroid2 >= params.thresholdSmoothing_) {
          temp[i] = centroid;
          reconstruct.setColor(i, PCCColor3B(255, 0, 0));
          if( PCC_SAVE_POINT_TYPE == 1 ) {
            reconstruct.setType( i, PointType::Smooth );
          }
        } else {
          temp[i] = query.point;
        }
      } else {
        temp[i] = query.point;
      }
    });
  }); 
  limited.execute([&]{ 
    tbb::parallel_for(size_t( 0 ), pointCount, [&](const size_t i) {
      // for (size_t i = 0; i < pointCount; ++i) {
      reconstruct[i] = temp[i];
    });
  });
}
bool PCCCodec::colorPointCloud( PCCPointSet3& reconstruct, PCCFrameContext& frame,
                                const PCCVideoTexture &video, const bool noAttributes ) {
  if( noAttributes ) {
    for (auto& color : reconstruct.getColors() ) {
      for (size_t c = 0; c < 3; ++c) {
        color[c] = static_cast<uint8_t>(127);
      }
    }
  } else {
    auto& pointToPixel = frame.getPointToPixel();
    auto& color        = reconstruct.getColors();
    const size_t pointCount = reconstruct.getPointCount();
    if (!pointCount || !reconstruct.hasColors()) {
      return false;
    }
    const size_t shift = frame.getIndex() * 2;
    for (size_t i = 0; i < pointCount; ++i) {
      const PCCVector3<size_t> location = pointToPixel[i];
      const size_t x = location[0];
      const size_t y = location[1];
      const size_t f = location[2];
      const auto &frame = video.getFrame( shift + f );
      for (size_t c = 0; c < 3; ++c) {
        color[i][c] = frame.getValue(c, x, y);
      }
    }
  }
  return true;
}


