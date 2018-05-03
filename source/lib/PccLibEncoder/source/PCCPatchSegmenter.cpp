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

#include "PCCKdTree.h"
#include "PCCNormalsGenerator.h"
#include "tbb/tbb.h"
#include "PCCPatchSegmenter.h"
#include "PCCPatch.h"

using namespace pcc;


void PCCPatchSegmenter3::setNbThread( size_t nbThread ) {
  nbThread_ = nbThread;
  if( nbThread_ > 0 ) {
    tbb::task_scheduler_init init( (int)nbThread );
  }
}

void PCCPatchSegmenter3::compute( const PCCPointSet3 &geometry, const PCCPatchSegmenter3Parameters &params,
                                  std::vector<PCCPatch> &patches ) {
  const PCCVector3D orientations[6] = {
      PCCVector3D(1.0, 0.0, 0.0),  PCCVector3D(0.0, 1.0, 0.0),  PCCVector3D(0.0, 0.0, 1.0),
      PCCVector3D(-1.0, 0.0, 0.0), PCCVector3D(0.0, -1.0, 0.0), PCCVector3D(0.0, 0.0, -1.0),
  };
  const size_t orientationCount = 6;
  std::cout << "  Computing normals for original point cloud... ";
  PCCStaticKdTree3 kdtree;
  kdtree.build(geometry);
  PCCNormalsGenerator3 normalsGen;
  const PCCNormalsGenerator3Parameters normalsGenParams = {
      PCCPoint3D(0.0),
      (std::numeric_limits<double>::max)(),
      (std::numeric_limits<double>::max)(),
      (std::numeric_limits<double>::max)(),
      (std::numeric_limits<double>::max)(),
      params.nnNormalEstimation,
      params.nnNormalEstimation,
      params.nnNormalEstimation,
      0,
      PCC_NORMALS_GENERATOR_ORIENTATION_SPANNING_TREE,
      false,
      false,
      false };
  normalsGen.compute(geometry, kdtree, normalsGenParams, nbThread_ );
  std::cout << "[done]" << std::endl;

  std::cout << "  Computing initial segmentation... ";
  std::vector<size_t> partition;
  initialSegmentation(geometry, normalsGen, orientations, orientationCount, partition);
  std::cout << "[done]" << std::endl;

  std::cout << "  Refining segmentation... ";
  refineSegmentation(geometry, kdtree, normalsGen, orientations, orientationCount,
                      params.maxNNCountRefineSegmentation, params.lambdaRefineSegmentation,
                      params.iterationCountRefineSegmentation, partition);
  std::cout << "[done]" << std::endl;

  std::cout << "  Patch segmentation... ";
  PCCPointSet3 resampled;
  std::vector<size_t> patchPartition;
  std::vector<size_t> resampledPatchPartition;
  std::vector<size_t> missedPoints;
  segmentPatches(geometry, kdtree, params.maxNNCountPatchSegmentation,
                  params.minPointCountPerCCPatchSegmentation, params.occupancyResolution,
                  params.maxAllowedDist2MissedPointsDetection,
                  params.maxAllowedDist2MissedPointsSelection, params.surfaceThickness,
                  params.maxAllowedDepth, partition, patches, patchPartition,
                  resampledPatchPartition, missedPoints, resampled);
  std::cout << "[done]" << std::endl;
}

void PCCPatchSegmenter3::initialSegmentation(const PCCPointSet3 &geometry,
                                const PCCNormalsGenerator3 &normalsGen,
                                const PCCVector3D *orientations, const size_t orientationCount,
                                std::vector<size_t> &partition) {
  assert(orientations);
  const size_t pointCount = geometry.getPointCount();
  partition.resize(pointCount);
  tbb::task_arena limited( (int)nbThread_ );       
  limited.execute([&]{ 
    tbb::parallel_for(size_t( 0 ), pointCount, [&](const size_t i) {
      const PCCVector3D normal = normalsGen.getNormal(i);
      size_t clusterIndex = 0;
      double bestScore = normal * orientations[0];
      for (size_t j = 1; j < orientationCount; ++j) {
        const double score = normal * orientations[j];
        if (score > bestScore) {
          bestScore = score;
          clusterIndex = j;
        }
      }
      partition[i] = clusterIndex;
    });
  });
}

void PCCPatchSegmenter3::computeAdjacencyInfo(const PCCPointSet3 &pointCloud, const PCCStaticKdTree3 &kdtree,
                                  std::vector<std::vector<size_t>> &adj, const size_t maxNNCount) {
  const size_t pointCount = pointCloud.getPointCount();
  adj.resize(pointCount);
  tbb::task_arena limited( (int)nbThread_ );       
  limited.execute([&]{ 
    tbb::parallel_for(size_t( 0 ), pointCount, [&](const size_t i) {
      std::vector<PCCPointDistInfo> nNeighbor;
      nNeighbor.resize(maxNNCount);
      PCCNNResult result = {nNeighbor.data(), 0};
      PCCNNQuery3 query = {PCCVector3D(0.0), (std::numeric_limits<double>::max)(), maxNNCount};
      query.point = pointCloud[i];
      kdtree.findNearestNeighbors(query, result);
      std::vector<size_t> &neighbors = adj[i];
      neighbors.resize(result.resultCount);
      for (size_t j = 0; j < result.resultCount; ++j) {
        neighbors[j] = result.neighbors[j].index;
      }
    });
  });
}

void PCCPatchSegmenter3::segmentPatches(const PCCPointSet3 &points, const PCCStaticKdTree3 &kdtree,
                            const size_t maxNNCount, const size_t minPointCountPerCC,
                            const size_t occupancyResolution,
                            const double maxAllowedDist2MissedPointsDetection,
                            const double maxAllowedDist2MissedPointsSelection,
                            const size_t surfaceThickness, const size_t maxAllowedDepth,
                            const std::vector<size_t> &partition,
                            std::vector<PCCPatch> &patches,
                            std::vector<size_t> &patchPartition,
                            std::vector<size_t> &resampledPatchPartition,
                            std::vector<size_t> missedPoints, PCCPointSet3 &resampled) {
  const size_t pointCount = points.getPointCount();
  patchPartition.resize(pointCount, 0);

  resampledPatchPartition.reserve(pointCount);

  std::cout << "\n\t Computing adjacency info... ";
  std::vector<std::vector<size_t>> adj;
  computeAdjacencyInfo(points, kdtree, adj, maxNNCount);
  std::cout << "[done]" << std::endl;

  std::cout << "\n\t Extracting patches... " << std::endl;
  std::vector<double> missedPointsDistance;
  missedPoints.resize(pointCount);
  missedPointsDistance.resize(pointCount);
  for (size_t i = 0; i < pointCount; ++i) {
    missedPoints[i] = i;
    missedPointsDistance[i] = (std::numeric_limits<double>::max)();
  }

  while (!missedPoints.empty()) {
    std::vector<size_t> fifo;
    fifo.reserve(pointCount);
    std::vector<bool> flags;
    std::vector<std::vector<size_t>> connectedComponents;
    flags.resize(pointCount, false);
    for (const auto i : missedPoints) {
      flags[i] = true;
    }
    connectedComponents.reserve(256);
    for (const auto i : missedPoints) {
      if (flags[i] && missedPointsDistance[i] > maxAllowedDist2MissedPointsDetection) {
        flags[i] = false;
        const size_t indexCC = connectedComponents.size();
        const size_t clusterIndex = partition[i];
        connectedComponents.resize(indexCC + 1);
        std::vector<size_t> &connectedComponent = connectedComponents[indexCC];
        fifo.push_back(i);
        connectedComponent.push_back(i);
        while (!fifo.empty()) {
          const size_t current = fifo.back();
          fifo.pop_back();
          for (const auto n : adj[current]) {
            if (clusterIndex == partition[n] && flags[n]) {
              flags[n] = false;
              fifo.push_back(n);
              connectedComponent.push_back(n);
            }
          }
        }
        if (connectedComponent.size() < minPointCountPerCC) {
          connectedComponents.resize(indexCC);
        } else {
          std::cout << "\t\t CC " << indexCC << " -> " << connectedComponent.size() << std::endl;
        }
      }
    }

    std::cout << " # CC " << connectedComponents.size() << std::endl;
    if (connectedComponents.empty()) {
      break;
    }
    for (auto &connectedComponent : connectedComponents) {
      const size_t patchIndex = patches.size();
      patches.resize(patchIndex + 1);
      PCCPatch &patch = patches[patchIndex];
      patch.getIndex() = patchIndex;
      const size_t clusterIndex = partition[connectedComponent[0]];
      if (clusterIndex == 0 || clusterIndex == 3) {
        patch.getNormalAxis() = 0;
        patch.getTangentAxis() = 2;
        patch.getBitangentAxis() = 1;
      } else if (clusterIndex == 1 || clusterIndex == 4) {
        patch.getNormalAxis() = 1;
        patch.getTangentAxis() = 2;
        patch.getBitangentAxis() = 0;
      } else {
        patch.getNormalAxis() = 2;
        patch.getTangentAxis() = 0;
        patch.getBitangentAxis() = 1;
      }

      patch.setViewId() = clusterIndex;
      patch.setBestMatchIdx() = -1;

      PCCBox3D boundingBox;
      boundingBox.min_ = boundingBox.max_ = points[connectedComponent[0]];
      for (const auto i : connectedComponent) {
        patchPartition[i] = patchIndex + 1;
        const PCCVector3D point = points[i];
        for (size_t k = 0; k < 3; ++k) {
          if (point[k] < boundingBox.min_[k]) {
            boundingBox.min_[k] = floor(point[k]);
          }
          if (point[k] > boundingBox.max_[k]) {
            boundingBox.max_[k] = ceil(point[k]);
          }
        }
      }

      const int16_t infiniteDepth = (std::numeric_limits<int16_t>::max)();
      patch.getSizeU() = 1 + size_t(boundingBox.max_[patch.getTangentAxis()] - boundingBox.min_[patch.getTangentAxis()]);
      patch.getSizeV() = 1 + size_t(boundingBox.max_[patch.getBitangentAxis()] - boundingBox.min_[patch.getBitangentAxis()]);
      patch.getU1()    = size_t(boundingBox.min_[patch.getTangentAxis()]);
      patch.getV1()    = size_t(boundingBox.min_[patch.getBitangentAxis()]);
      patch.getD1()    = infiniteDepth;
      patch.getDepth(0).resize(patch.getSizeU() * patch.getSizeV(), infiniteDepth);

      patch.getOccupancyResolution() = occupancyResolution;
      patch.getSizeU0() = 0;
      patch.getSizeV0() = 0;
      for (const auto i : connectedComponent) {
        const PCCVector3D point = points[i];
        const int16_t d = int16_t(round(point[patch.getNormalAxis()]));
        const size_t u = size_t(round(point[patch.getTangentAxis()] - patch.getU1()));
        const size_t v = size_t(round(point[patch.getBitangentAxis()] - patch.getV1()));
        assert(u >= 0 && u < patch.getSizeU());
        assert(v >= 0 && v < patch.getSizeV());
        const size_t p = v * patch.getSizeU() + u;
        if (patch.getDepth(0)[p] > d) {
          patch.getDepth(0)[p] = d;
          patch.getSizeU0() = (std::max)(patch.getSizeU0(), u / patch.getOccupancyResolution());
          patch.getSizeV0() = (std::max)(patch.getSizeV0(), v / patch.getOccupancyResolution());
          patch.getD1() = (std::min)(patch.getD1(), size_t(d));
        }
      }

      ++patch.getSizeU0();
      ++patch.getSizeV0();
      patch.getOccupancy().resize(patch.getSizeU0() * patch.getSizeV0(), false);
      // filter depth
      std::vector<int16_t> minPerBlock;
      minPerBlock.resize(patch.getSizeU0() * patch.getSizeV0(), infiniteDepth);
      for (int64_t v = 0; v < int64_t(patch.getSizeV()); ++v) {
        for (int64_t u = 0; u < int64_t(patch.getSizeU()); ++u) {
          const size_t p = v * patch.getSizeU() + u;
          const int16_t depth0 = patch.getDepth(0)[p];
          const size_t u0 = u / patch.getOccupancyResolution();
          const size_t v0 = v / patch.getOccupancyResolution();
          const size_t p0 = v0 * patch.getSizeU0() + u0;
          const int16_t minDepth = minPerBlock[p0];
          minPerBlock[p0] = (std::min)(minDepth, depth0);
        }
      }
      for (int64_t v = 0; v < int64_t(patch.getSizeV()); ++v) {
        for (int64_t u = 0; u < int64_t(patch.getSizeU()); ++u) {
          const size_t p = v * patch.getSizeU() + u;
          const int16_t depth0 = patch.getDepth(0)[p];
          const size_t u0 = u / patch.getOccupancyResolution();
          const size_t v0 = v / patch.getOccupancyResolution();
          const size_t p0 = v0 * patch.getSizeU0() + u0;
          const int16_t minDepth = minPerBlock[p0];
          if ((depth0 - minDepth > 32) ||
              (surfaceThickness + depth0 > patch.getD1() + maxAllowedDepth)) {
            patch.getDepth(0)[p] = infiniteDepth;
          }
        }
      }

      patch.getDepth(1) = patch.getDepth(0);
      for (const auto i : connectedComponent) {
        const PCCVector3D point = points[i];
        const int16_t d = int16_t(round(point[patch.getNormalAxis()]));
        const size_t u = size_t(round(point[patch.getTangentAxis()] - patch.getU1()));
        const size_t v = size_t(round(point[patch.getBitangentAxis()] - patch.getV1()));
        assert(u >= 0 && u < patch.getSizeU());
        assert(v >= 0 && v < patch.getSizeV());
        const size_t p = v * patch.getSizeU() + u;
        const int16_t depth0 = patch.getDepth(0)[p];
        if (depth0 < infiniteDepth && (d - depth0) <= int16_t(surfaceThickness) &&
            d > patch.getDepth(1)[p]) {
          patch.getDepth(1)[p] = d;
        }
      }

      patch.getSizeD() = 0;

      for (size_t v = 0; v < patch.getSizeV(); ++v) {
        for (size_t u = 0; u < patch.getSizeU(); ++u) {
          const size_t p = v * patch.getSizeU() + u;
          int16_t depth0 = patch.getDepth(0)[p];
          if (depth0 < infiniteDepth) {
            depth0 -= int16_t(patch.getD1());
            assert(depth0 >= 0);
            patch.getDepth(0)[p] = depth0;
            patch.getSizeD() =
                (std::max)(patch.getSizeD(), static_cast<size_t>(depth0));  // compute max depth

            const size_t u0 = u / patch.getOccupancyResolution();
            const size_t v0 = v / patch.getOccupancyResolution();
            const size_t p0 = v0 * patch.getSizeU0() + u0;
            assert(u0 >= 0 && u0 < patch.getSizeU0());
            assert(v0 >= 0 && v0 < patch.getSizeV0());
            patch.getOccupancy()[p0] = true;

            PCCVector3D point;
            point[patch.getNormalAxis()] = double(depth0) + patch.getD1();
            point[patch.getTangentAxis()] = double(u) + patch.getU1();
            point[patch.getBitangentAxis()] = double(v) + patch.getV1();
            resampled.addPoint(point);
            resampledPatchPartition.push_back(patchIndex);

            patch.getDepth(1)[p] -= int16_t(patch.getD1());
            point[patch.getNormalAxis()] = double(patch.getDepth(1)[p] + patch.getD1());
            resampled.addPoint(point);
            resampledPatchPartition.push_back(patchIndex);
            assert( abs(patch.getDepth(1)[p] - patch.getDepth(0)[p]) <= int(surfaceThickness));
          }
        }
      }
      std::cout << "\t\t Patch " << patchIndex << " -> (d1, u1,v1) = (" << patch.getD1() << ", "
                << patch.getU1() << ", " << patch.getV1() << ") (dd, du,dv) = (" << patch.getSizeD() << ", "
                << patch.getSizeU() << "," << patch.getSizeV() << ")" << std::endl;
    }
    PCCStaticKdTree3 kdtreeResampled;
    kdtreeResampled.build(resampled);
    PCCPointDistInfo nNeighbor;
    PCCNNResult result = {&nNeighbor, 0};
    PCCNNQuery3 query = {PCCVector3D(0.0), (std::numeric_limits<double>::max)(), 1};

    missedPoints.resize(0);
    for (size_t i = 0; i < pointCount; ++i) {
      query.point = points[i];
      kdtreeResampled.findNearestNeighbors(query, result);
      assert(result.resultCount);
      const double dist2 = result.neighbors[0].dist2;
      missedPointsDistance[i] = dist2;
      if (dist2 > maxAllowedDist2MissedPointsSelection) {
        missedPoints.push_back(i);
      }
    }
    std::cout << " # patches " << patches.size() << std::endl;
    std::cout << " # missed points " << missedPoints.size() << std::endl;
  }
}
void PCCPatchSegmenter3::refineSegmentation(const PCCPointSet3 &pointCloud, const PCCStaticKdTree3 &kdtree,
                                            const PCCNormalsGenerator3 &normalsGen,
                                            const PCCVector3D *orientations, const size_t orientationCount,
                                            const size_t maxNNCount, const double lambda,
                                            const size_t iterationCount, std::vector<size_t> &partition) {
  assert(orientations);
  std::vector<std::vector<size_t>> adj;
  computeAdjacencyInfo(pointCloud, kdtree, adj, maxNNCount);
  const size_t pointCount = pointCloud.getPointCount();
  const double weight = lambda / maxNNCount;
  std::vector<size_t> tempPartition = partition;
  tempPartition.resize(pointCount);
  for (size_t k = 0; k < iterationCount; ++k) {
    tbb::task_arena limited( (int)nbThread_ );       
    limited.execute([&]{ 
      tbb::parallel_for( size_t( 0 ), pointCount, [&](const size_t i) {
        const PCCVector3D normal = normalsGen.getNormal(i);
        const std::vector<size_t> &neighbors = adj[i];
        size_t clusterIndex = partition[i];
        double bestScore = 0.0;
        for (size_t j = 0; j < orientationCount; ++j) {
          const double scoreNormal = normal * orientations[j];
          double scoreSmooth = 0.0;
          for (const auto n : neighbors) {
            scoreSmooth += (j == partition[n]);
          }
          const double score = scoreNormal + weight * scoreSmooth;
          if (score > bestScore) {
            bestScore = score;
            clusterIndex = j;
          }
        }
        tempPartition[i] = clusterIndex;
      });
    });
    swap(tempPartition, partition);
  }
}

float pcc::computeIOU(Rect a, Rect b) {
  float iou = 0.0f;
  Rect intersec = a & b;
  
  //printf("intersec: [%d,%d],%d,%d\n", intersec.x, intersec.y, intersec.width, intersec.height);
  int intersectionArea = intersec.area();
  int unionArea = a.area() + b.area() - intersectionArea;
  iou = (float)intersectionArea / unionArea;
  return iou;
}

