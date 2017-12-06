/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * <OWNER> = Apple Inc.
 * <ORGANIZATION> = Apple Inc.
 * <YEAR> = 2017
 *
 * Copyright (c) 2017, Apple Inc.
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
 *  * Neither the name of the <ORGANIZATION> nor the names of its contributors may
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

#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "PCCKdTree.h"
#include "PCCNormalsGenerator.h"
#include "tbb/tbb.h"

namespace pcc {
struct PCCPointCloudPatch {
  size_t u1;                   // tangential shift
  size_t v1;                   // bitangential shift
  size_t d1;                   // depth shift
  size_t sizeD;                // size for depth
  size_t sizeU;                // size for depth
  size_t sizeV;                // size for depth
  size_t u0;                   // location in packed image
  size_t v0;                   // location in packed image
  size_t sizeV0;               // size of occupancy map
  size_t sizeU0;               // size of occupancy map
  size_t occupancyResolution;  // ocupacy map resolution

  size_t normalAxis;              // x
  size_t tangentAxis;             // y
  size_t bitangentAxis;           // z
  std::vector<int16_t> depth[2];  // depth
  std::vector<bool> occupancy;    // occupancy map

  friend bool operator<(const PCCPointCloudPatch &lhs, const PCCPointCloudPatch &rhs) {
    return lhs.sizeV != rhs.sizeV ? lhs.sizeV > rhs.sizeV : lhs.sizeU > rhs.sizeU;
  }
};

struct PCCPatchSegmenter3Parameters {
  size_t nnNormalEstimation;
  size_t maxNNCountRefineSegmentation;
  size_t iterationCountRefineSegmentation;
  size_t occupancyResolution;
  size_t minPointCountPerCCPatchSegmentation;
  size_t maxNNCountPatchSegmentation;
  size_t surfaceThickness;
  size_t maxAllowedDepth;
  double maxAllowedDist2MissedPointsDetection;
  double maxAllowedDist2MissedPointsSelection;
  double lambdaRefineSegmentation;
};

class PCCPatchSegmenter3 {
 public:
  PCCPatchSegmenter3(void) = default;
  PCCPatchSegmenter3(const PCCPatchSegmenter3 &) = delete;
  PCCPatchSegmenter3 &operator=(const PCCPatchSegmenter3 &) = delete;
  ~PCCPatchSegmenter3() = default;
  void compute(const PCCPointSet3 &geometry, const PCCPatchSegmenter3Parameters &params,
               const std::string path, std::vector<PCCPointCloudPatch> &patches) {
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
        false};
    normalsGen.compute(geometry, kdtree, normalsGenParams);
    std::cout << "[done]" << std::endl;

    std::cout << "  Computing initial segmentation... ";
    std::vector<size_t> partition;
    InitialSegmentation(geometry, normalsGen, orientations, orientationCount, partition);
    std::cout << "[done]" << std::endl;

    std::cout << "  Refining segmentation... ";
    RefineSegmentation(geometry, kdtree, normalsGen, orientations, orientationCount,
                       params.maxNNCountRefineSegmentation, params.lambdaRefineSegmentation,
                       params.iterationCountRefineSegmentation, partition);
    std::cout << "[done]" << std::endl;

    std::cout << "  Patch segmentation... ";
    PCCPointSet3 resampled;
    std::vector<size_t> patchPartition;
    std::vector<size_t> resampledPatchPartition;
    std::vector<size_t> missedPoints;
    SegmentPatches(geometry, kdtree, params.maxNNCountPatchSegmentation,
                   params.minPointCountPerCCPatchSegmentation, params.occupancyResolution,
                   params.maxAllowedDist2MissedPointsDetection,
                   params.maxAllowedDist2MissedPointsSelection, params.surfaceThickness,
                   params.maxAllowedDepth, partition, patches, patchPartition,
                   resampledPatchPartition, missedPoints, resampled);
    std::cout << "[done]" << std::endl;
  }

 private:
  static void InitialSegmentation(const PCCPointSet3 &geometry,
                                  const PCCNormalsGenerator3 &normalsGen,
                                  const PCCVector3D *orientations, const size_t orientationCount,
                                  std::vector<size_t> &partition) {
    assert(orientations);
    const size_t pointCount = geometry.getPointCount();
    partition.resize(pointCount);
    tbb::parallel_for(size_t(0), pointCount, [&](const size_t i) {
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
  }

  static void ComputeAdjacencyInfo(const PCCPointSet3 &pointCloud, const PCCStaticKdTree3 &kdtree,
                                   std::vector<std::vector<size_t>> &adj, const size_t maxNNCount) {
    const size_t pointCount = pointCloud.getPointCount();
    adj.resize(pointCount);
    tbb::parallel_for(size_t(0), pointCount, [&](const size_t i) {
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
  }

  static void SegmentPatches(const PCCPointSet3 &points, const PCCStaticKdTree3 &kdtree,
                             const size_t maxNNCount, const size_t minPointCountPerCC,
                             const size_t occupancyResolution,
                             const double maxAllowedDist2MissedPointsDetection,
                             const double maxAllowedDist2MissedPointsSelection,
                             const size_t surfaceThickness, const size_t maxAllowedDepth,
                             const std::vector<size_t> &partition,
                             std::vector<PCCPointCloudPatch> &patches,
                             std::vector<size_t> &patchPartition,
                             std::vector<size_t> &resampledPatchPartition,
                             std::vector<size_t> missedPoints, PCCPointSet3 &resampled) {
    const size_t pointCount = points.getPointCount();
    patchPartition.resize(pointCount, 0);

    resampledPatchPartition.reserve(pointCount);

    std::cout << "\n\t Computing adjacency info... ";
    std::vector<std::vector<size_t>> adj;
    ComputeAdjacencyInfo(points, kdtree, adj, maxNNCount);
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
        PCCPointCloudPatch &patch = patches[patchIndex];
        const size_t clusterIndex = partition[connectedComponent[0]];
        if (clusterIndex == 0 || clusterIndex == 3) {
          patch.normalAxis = 0;
          patch.tangentAxis = 2;
          patch.bitangentAxis = 1;
        } else if (clusterIndex == 1 || clusterIndex == 4) {
          patch.normalAxis = 1;
          patch.tangentAxis = 2;
          patch.bitangentAxis = 0;
        } else {
          patch.normalAxis = 2;
          patch.tangentAxis = 0;
          patch.bitangentAxis = 1;
        }
        PCCBox3D boundingBox;
        boundingBox.min = boundingBox.max = points[connectedComponent[0]];
        for (const auto i : connectedComponent) {
          patchPartition[i] = patchIndex + 1;
          const PCCVector3D point = points[i];
          for (size_t k = 0; k < 3; ++k) {
            if (point[k] < boundingBox.min[k]) {
              boundingBox.min[k] = floor(point[k]);
            }
            if (point[k] > boundingBox.max[k]) {
              boundingBox.max[k] = ceil(point[k]);
            }
          }
        }

        const int16_t infiniteDepth = (std::numeric_limits<int16_t>::max)();
        patch.sizeU =
            1 + size_t(boundingBox.max[patch.tangentAxis] - boundingBox.min[patch.tangentAxis]);
        patch.sizeV =
            1 + size_t(boundingBox.max[patch.bitangentAxis] - boundingBox.min[patch.bitangentAxis]);
        patch.u1 = size_t(boundingBox.min[patch.tangentAxis]);
        patch.v1 = size_t(boundingBox.min[patch.bitangentAxis]);
        patch.d1 = infiniteDepth;
        patch.depth[0].resize(patch.sizeU * patch.sizeV, infiniteDepth);

        patch.occupancyResolution = occupancyResolution;
        patch.sizeU0 = 0;
        patch.sizeV0 = 0;
        for (const auto i : connectedComponent) {
          const PCCVector3D point = points[i];
          const int16_t d = int16_t(round(point[patch.normalAxis]));
          const size_t u = size_t(round(point[patch.tangentAxis] - patch.u1));
          const size_t v = size_t(round(point[patch.bitangentAxis] - patch.v1));
          assert(u >= 0 && u < patch.sizeU);
          assert(v >= 0 && v < patch.sizeV);
          const size_t p = v * patch.sizeU + u;
          if (patch.depth[0][p] > d) {
            patch.depth[0][p] = d;
            patch.sizeU0 = (std::max)(patch.sizeU0, u / patch.occupancyResolution);
            patch.sizeV0 = (std::max)(patch.sizeV0, v / patch.occupancyResolution);
            patch.d1 = (std::min)(patch.d1, size_t(d));
          }
        }

        ++patch.sizeU0;
        ++patch.sizeV0;
        patch.occupancy.resize(patch.sizeU0 * patch.sizeV0, false);
        // filter depth
        std::vector<int16_t> minPerBlock;
        minPerBlock.resize(patch.sizeU0 * patch.sizeV0, infiniteDepth);
        for (int64_t v = 0; v < int64_t(patch.sizeV); ++v) {
          for (int64_t u = 0; u < int64_t(patch.sizeU); ++u) {
            const size_t p = v * patch.sizeU + u;
            const int16_t depth0 = patch.depth[0][p];
            const size_t u0 = u / patch.occupancyResolution;
            const size_t v0 = v / patch.occupancyResolution;
            const size_t p0 = v0 * patch.sizeU0 + u0;
            const int16_t minDepth = minPerBlock[p0];
            minPerBlock[p0] = (std::min)(minDepth, depth0);
          }
        }
        for (int64_t v = 0; v < int64_t(patch.sizeV); ++v) {
          for (int64_t u = 0; u < int64_t(patch.sizeU); ++u) {
            const size_t p = v * patch.sizeU + u;
            const int16_t depth0 = patch.depth[0][p];
            const size_t u0 = u / patch.occupancyResolution;
            const size_t v0 = v / patch.occupancyResolution;
            const size_t p0 = v0 * patch.sizeU0 + u0;
            const int16_t minDepth = minPerBlock[p0];
            if ((depth0 - minDepth > 32) ||
                (surfaceThickness + depth0 > patch.d1 + maxAllowedDepth)) {
              patch.depth[0][p] = infiniteDepth;
            }
          }
        }

        patch.depth[1] = patch.depth[0];
        for (const auto i : connectedComponent) {
          const PCCVector3D point = points[i];
          const int16_t d = int16_t(round(point[patch.normalAxis]));
          const size_t u = size_t(round(point[patch.tangentAxis] - patch.u1));
          const size_t v = size_t(round(point[patch.bitangentAxis] - patch.v1));
          assert(u >= 0 && u < patch.sizeU);
          assert(v >= 0 && v < patch.sizeV);
          const size_t p = v * patch.sizeU + u;
          const int16_t depth0 = patch.depth[0][p];
          if (depth0 < infiniteDepth && (d - depth0) <= int16_t(surfaceThickness) &&
              d > patch.depth[1][p]) {
            patch.depth[1][p] = d;
          }
        }

        patch.sizeD = 0;
        for (size_t v = 0; v < patch.sizeV; ++v) {
          for (size_t u = 0; u < patch.sizeU; ++u) {
            const size_t p = v * patch.sizeU + u;
            int16_t depth0 = patch.depth[0][p];
            if (depth0 < infiniteDepth) {
              depth0 -= int16_t(patch.d1);
              assert(depth0 >= 0);
              patch.depth[0][p] = depth0;
              patch.sizeD =
                  (std::max)(patch.sizeD, static_cast<size_t>(depth0));  // compute max depth

              const size_t u0 = u / patch.occupancyResolution;
              const size_t v0 = v / patch.occupancyResolution;
              const size_t p0 = v0 * patch.sizeU0 + u0;
              assert(u0 >= 0 && u0 < patch.sizeU0);
              assert(v0 >= 0 && v0 < patch.sizeV0);
              patch.occupancy[p0] = true;

              PCCVector3D point;
              point[patch.normalAxis] = double(depth0) + patch.d1;
              point[patch.tangentAxis] = double(u) + patch.u1;
              point[patch.bitangentAxis] = double(v) + patch.v1;
              resampled.addPoint(point);
              resampledPatchPartition.push_back(patchIndex);

              patch.depth[1][p] -= int16_t(patch.d1);
              point[patch.normalAxis] = double(patch.depth[1][p] + patch.d1);
              resampled.addPoint(point);
              resampledPatchPartition.push_back(patchIndex);
              assert(abs(patch.depth[1][p] - patch.depth[0][p]) <= int(surfaceThickness));
            }
          }
        }
        std::cout << "\t\t Patch " << patchIndex << " -> (d1, u1,v1) = (" << patch.d1 << ", "
                  << patch.u1 << ", " << patch.v1 << ") (dd, du,dv) = (" << patch.sizeD << ", "
                  << patch.sizeU << "," << patch.sizeV << ")" << std::endl;
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
  static void RefineSegmentation(const PCCPointSet3 &pointCloud, const PCCStaticKdTree3 &kdtree,
                                 const PCCNormalsGenerator3 &normalsGen,
                                 const PCCVector3D *orientations, const size_t orientationCount,
                                 const size_t maxNNCount, const double lambda,
                                 const size_t iterationCount, std::vector<size_t> &partition) {
    assert(orientations);
    std::vector<std::vector<size_t>> adj;
    ComputeAdjacencyInfo(pointCloud, kdtree, adj, maxNNCount);
    const size_t pointCount = pointCloud.getPointCount();
    const double weight = lambda / maxNNCount;
    std::vector<size_t> tempPartition = partition;
    tempPartition.resize(pointCount);
    for (size_t k = 0; k < iterationCount; ++k) {
      tbb::parallel_for(size_t(0), pointCount, [&](const size_t i) {
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
      swap(tempPartition, partition);
    }
  }

 private:
};
}

#endif /* PCCPatchSegmenter_h */
