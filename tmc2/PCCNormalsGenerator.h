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

#ifndef PCCNormalsGenerator_h
#define PCCNormalsGenerator_h

#include <queue>
#include <vector>

#include "PCCKdTree.h"
#include "tbb/tbb.h"

namespace pcc {
enum PCCNormalsGeneratorOrientation {
  PCC_NORMALS_GENERATOR_ORIENTATION_NONE = 0,
  PCC_NORMALS_GENERATOR_ORIENTATION_SPANNING_TREE = 1,
  PCC_NORMALS_GENERATOR_ORIENTATION_VIEW_POINT = 2
};

struct PCCNormalsGenerator3Parameters {
  PCCPoint3D viewPoint;
  double radiusNormalSmoothing;
  double radiusNormalEstimation;
  double radiusNormalOrientation;
  double weightNormalSmoothing;
  size_t numberOfNearestNeighborsInNormalSmoothing;
  size_t numberOfNearestNeighborsInNormalEstimation;
  size_t numberOfNearestNeighborsInNormalOrientation;
  size_t numberOfIterationsInNormalSmoothing;
  PCCNormalsGeneratorOrientation orientationStrategy;
  bool storeEigenvalues;
  bool storeNumberOfNearestNeighborsInNormalEstimation;
  bool storeCentroids;
};

class PCCNormalsGenerator3 {
  struct PCCWeightedEdge {
    double weight;
    uint32_t start;
    uint32_t end;
    bool operator<(const PCCWeightedEdge &rhs) const { return weight < rhs.weight; }
  };

 public:
  PCCNormalsGenerator3(void) = default;
  PCCNormalsGenerator3(const PCCNormalsGenerator3 &) = delete;
  PCCNormalsGenerator3 &operator=(const PCCNormalsGenerator3 &) = delete;
  ~PCCNormalsGenerator3() = default;
  void clear() { normals.resize(0); }
  void init(const size_t pointCount, const PCCNormalsGenerator3Parameters &params) {
    normals.resize(pointCount);
    if (params.storeNumberOfNearestNeighborsInNormalEstimation) {
      numberOfNearestNeighborsInNormalEstimation.resize(pointCount);
    } else {
      numberOfNearestNeighborsInNormalEstimation.resize(0);
    }
    if (params.storeEigenvalues) {
      eigenvalues.resize(pointCount);
    } else {
      eigenvalues.resize(0);
    }
    if (params.storeCentroids) {
      barycenters.resize(pointCount);
    } else {
      barycenters.resize(0);
    }
  }
  void compute(const PCCPointSet3 &pointCloud, const PCCStaticKdTree3 &kdtree,
               const PCCNormalsGenerator3Parameters &params) {
    init(pointCloud.getPointCount(), params);
    computeNormals(pointCloud, kdtree, params);
    if (params.numberOfIterationsInNormalSmoothing) {
      smoothNormals(pointCloud, kdtree, params);
    }
    orientNormals(pointCloud, kdtree, params);
  }
  PCCVector3D getNormal(const size_t pos) const {
    assert(pos < normals.size());
    return normals[pos];
  }
  PCCVector3D getEigenvalues(const size_t pos) const {
    assert(pos < eigenvalues.size());
    return eigenvalues[pos];
  }
  PCCVector3D getCentroid(const size_t pos) const {
    assert(pos < barycenters.size());
    return barycenters[pos];
  }
  uint32_t getNumberOfNearestNeighborsInNormalEstimation(const size_t index) const {
    assert(index < numberOfNearestNeighborsInNormalEstimation.size());
    return numberOfNearestNeighborsInNormalEstimation[index];
  }
  size_t getNormalCount() const { return normals.size(); }
  void computeNormal(const size_t index, const PCCPointSet3 &pointCloud,
                     const PCCStaticKdTree3 &kdtree, const PCCNormalsGenerator3Parameters &params,
                     PCCNNResult &nNResult) {
    const PCCNNQuery3 nNQuery = {pointCloud[index], params.radiusNormalEstimation,
                                 params.numberOfNearestNeighborsInNormalEstimation};
    PCCVector3D bary(pointCloud[index]), normal(0.0), eigenval(0.0);
    PCCMatrix3D covMat, Q, D;
    kdtree.findNearestNeighbors(nNQuery, nNResult);
    if (nNResult.resultCount > 1) {
      bary = 0.0;
      for (size_t i = 0; i < nNResult.resultCount; ++i) {
        bary += pointCloud[nNResult.neighbors[i].index];
      }
      bary /= double(nNResult.resultCount);

      covMat = 0.0;
      PCCVector3D pt;
      for (size_t i = 0; i < nNResult.resultCount; ++i) {
        pt = pointCloud[nNResult.neighbors[i].index] - bary;
        covMat[0][0] += pt[0] * pt[0];
        covMat[1][1] += pt[1] * pt[1];
        covMat[2][2] += pt[2] * pt[2];
        covMat[0][1] += pt[0] * pt[1];
        covMat[0][2] += pt[0] * pt[2];
        covMat[1][2] += pt[1] * pt[2];
      }
      covMat[1][0] = covMat[0][1];
      covMat[2][0] = covMat[0][2];
      covMat[2][1] = covMat[1][2];
      covMat /= (nNResult.resultCount - 1.0);

      Diagonalize(covMat, Q, D);

      D[0][0] = fabs(D[0][0]);
      D[1][1] = fabs(D[1][1]);
      D[2][2] = fabs(D[2][2]);

      if (D[0][0] < D[1][1] && D[0][0] < D[2][2]) {
        normal[0] = Q[0][0];
        normal[1] = Q[1][0];
        normal[2] = Q[2][0];
        eigenval[0] = D[0][0];
        if (D[1][1] < D[2][2]) {
          eigenval[1] = D[1][1];
          eigenval[2] = D[2][2];
        } else {
          eigenval[2] = D[1][1];
          eigenval[1] = D[2][2];
        }
      } else if (D[1][1] < D[2][2]) {
        normal[0] = Q[0][1];
        normal[1] = Q[1][1];
        normal[2] = Q[2][1];
        eigenval[0] = D[1][1];
        if (D[0][0] < D[2][2]) {
          eigenval[1] = D[0][0];
          eigenval[2] = D[2][2];
        } else {
          eigenval[2] = D[0][0];
          eigenval[1] = D[2][2];
        }
      } else {
        normal[0] = Q[0][2];
        normal[1] = Q[1][2];
        normal[2] = Q[2][2];
        eigenval[0] = D[2][2];
        if (D[0][0] < D[1][1]) {
          eigenval[1] = D[0][0];
          eigenval[2] = D[1][1];
        } else {
          eigenval[2] = D[0][0];
          eigenval[1] = D[1][1];
        }
      }
    }
    if (normal * (params.viewPoint - pointCloud[index]) < 0.0) {
      normals[index] = -normal;
    } else {
      normals[index] = normal;
    }
    if (params.storeEigenvalues) {
      eigenvalues[index] = eigenval;
    }
    if (params.storeCentroids) {
      barycenters[index] = bary;
    }
    if (params.storeNumberOfNearestNeighborsInNormalEstimation) {
      numberOfNearestNeighborsInNormalEstimation[index] = uint32_t(nNResult.resultCount);
    }
  }
  void computeNormals(const PCCPointSet3 &pointCloud, const PCCStaticKdTree3 &kdtree,
                      const PCCNormalsGenerator3Parameters &params) {
    const size_t pointCount = pointCloud.getPointCount();
    normals.resize(pointCount);
    std::vector<size_t> subRanges;
    const size_t chunckCount = 64;
    PCCDivideRange(0, pointCount, chunckCount, subRanges);
    tbb::parallel_for(size_t(0), subRanges.size() - 1, [&](const size_t i) {
      const size_t start = subRanges[i];
      const size_t end = subRanges[i + 1];
      PCCNNResult nNResult;
      std::vector<PCCPointDistInfo> neighbors;
      neighbors.resize((std::min)(pointCount, params.numberOfNearestNeighborsInNormalEstimation));
      nNResult.neighbors = neighbors.data();
      nNResult.resultCount = 0;
      for (size_t ptIndex = start; ptIndex < end; ++ptIndex) {
        computeNormal(ptIndex, pointCloud, kdtree, params, nNResult);
      }
    });
  }
  void orientNormals(const PCCPointSet3 &pointCloud, const PCCStaticKdTree3 &kdtree,
                     const PCCNormalsGenerator3Parameters &params) {
    if (params.orientationStrategy == PCC_NORMALS_GENERATOR_ORIENTATION_SPANNING_TREE) {
      const size_t pointCount = pointCloud.getPointCount();
      PCCNNResult nNResult;
      std::vector<PCCPointDistInfo> neighbors;
      neighbors.resize((std::min)(pointCount, params.numberOfNearestNeighborsInNormalOrientation));
      nNResult.neighbors = neighbors.data();
      nNResult.resultCount = 0;
      visited.resize(pointCount);
      std::fill(visited.begin(), visited.end(), 0);
      PCCNNQuery3 nNQuery = {PCCVector3<double>(0.0), params.radiusNormalOrientation,
                             params.numberOfNearestNeighborsInNormalOrientation};
      PCCNNQuery3 nNQuery2 = {PCCVector3<double>(0.0), (std::numeric_limits<double>::max)(),
                              params.numberOfNearestNeighborsInNormalOrientation};
      size_t processedPointCount = 0;
      for (size_t ptIndex = 0; ptIndex < pointCount; ++ptIndex) {
        if (!visited[ptIndex]) {
          visited[ptIndex] = 1;
          ++processedPointCount;
          size_t numberOfNormals;
          PCCVector3<double> accumulatedNormals;
          addNeighbors(uint32_t(ptIndex), pointCloud, kdtree, nNQuery2, nNResult,
                       accumulatedNormals, numberOfNormals);
          if (!numberOfNormals) {
            if (ptIndex) {
              accumulatedNormals = normals[ptIndex - 1];
            } else {
              accumulatedNormals = (params.viewPoint - pointCloud[ptIndex]);
            }
          }
          if (normals[ptIndex] * accumulatedNormals < 0.0) {
            normals[ptIndex] = -normals[ptIndex];
          }
          while (!edges.empty()) {
            PCCWeightedEdge edge = edges.top();
            edges.pop();
            uint32_t current = edge.end;
            if (!visited[current]) {
              visited[current] = 1;
              ++processedPointCount;
              if (normals[edge.start] * normals[current] < 0.0) {
                normals[current] = -normals[current];
              }
              addNeighbors(current, pointCloud, kdtree, nNQuery, nNResult, accumulatedNormals,
                           numberOfNormals);
            }
          }
        }
      }
      size_t negNormalCount = 0;
      for (size_t ptIndex = 0; ptIndex < pointCount; ++ptIndex) {
        negNormalCount += (normals[ptIndex] * (params.viewPoint - pointCloud[ptIndex]) < 0.0);
      }
      if (negNormalCount > (pointCount + 1) / 2) {
        for (size_t ptIndex = 0; ptIndex < pointCount; ++ptIndex) {
          normals[ptIndex] = -normals[ptIndex];
        }
      }
    } else if (params.orientationStrategy == PCC_NORMALS_GENERATOR_ORIENTATION_VIEW_POINT) {
      const size_t pointCount = pointCloud.getPointCount();
      tbb::parallel_for(size_t(0), pointCount, [&](const size_t ptIndex) {
        if (normals[ptIndex] * (params.viewPoint - pointCloud[ptIndex]) < 0.0) {
          normals[ptIndex] = -normals[ptIndex];
        }
      });
    }
  }
  void addNeighbors(const uint32_t current, const PCCPointSet3 &pointCloud,
                    const PCCStaticKdTree3 &kdtree, PCCNNQuery3 &nNQuery, PCCNNResult &nNResult,
                    PCCVector3<double> &accumulatedNormals, size_t &numberOfNormals) {
    accumulatedNormals = 0.0;
    numberOfNormals = 0;
    nNQuery.point = pointCloud[current];
    kdtree.findNearestNeighbors(nNQuery, nNResult);
    PCCWeightedEdge newEdge;
    uint32_t index;
    for (size_t i = 0; i < nNResult.resultCount; ++i) {
      index = nNResult.neighbors[i].index;
      if (!visited[index]) {
        newEdge.weight = fabs(normals[current] * normals[index]);
        newEdge.end = index;
        newEdge.start = current;
        edges.push(newEdge);
      } else if (index != current) {
        accumulatedNormals += normals[index];
        ++numberOfNormals;
      }
    }
  }
  void smoothNormals(const PCCPointSet3 &pointCloud, const PCCStaticKdTree3 &kdtree,
                     const PCCNormalsGenerator3Parameters &params) {
    const double w2 = params.weightNormalSmoothing;
    const double w0 = (1 - w2);
    const size_t pointCount = pointCloud.getPointCount();
    PCCNNQuery3 nNQuery = {PCCVector3<double>(0.0), params.radiusNormalSmoothing,
                           params.numberOfNearestNeighborsInNormalSmoothing};
    PCCVector3<double> n0, n1, n2;
    std::vector<size_t> subRanges;
    const size_t chunckCount = 64;
    PCCDivideRange(0, pointCount, chunckCount, subRanges);
    for (size_t it = 0; it < params.numberOfIterationsInNormalSmoothing; ++it) {
      tbb::parallel_for(size_t(0), subRanges.size() - 1, [&](const size_t i) {
        const size_t start = subRanges[i];
        const size_t end = subRanges[i + 1];
        PCCNNResult nNResult;
        std::vector<PCCPointDistInfo> neighbors;
        neighbors.resize((std::min)(pointCount, params.numberOfNearestNeighborsInNormalSmoothing));
        nNResult.neighbors = neighbors.data();
        nNResult.resultCount = 0;
        for (size_t ptIndex = start; ptIndex < end; ++ptIndex) {
          nNQuery.point = pointCloud[ptIndex];
          kdtree.findNearestNeighbors(nNQuery, nNResult);
          n0 = normals[ptIndex];
          n1 = 0.0;
          for (size_t i = 1; i < nNResult.resultCount; ++i) {
            n2 = normals[nNResult.neighbors[i].index];
            if (n0 * n2 < 0.0) {
              n1 -= n2;
            } else {
              n1 += n2;
            }
          }
          n1.normalize();
          n1 = w0 * n0 + w2 * n1;
          n1.normalize();
          normals[ptIndex] = n1;
        }
      });
    }
  }

 private:
  std::vector<PCCVector3D> normals;
  std::vector<PCCVector3D> eigenvalues;
  std::vector<PCCPoint3D> barycenters;
  std::vector<uint32_t> numberOfNearestNeighborsInNormalEstimation;
  std::vector<uint32_t> visited;
  std::priority_queue<PCCWeightedEdge> edges;
};
}

#endif /* PCCNormalsGenerator_h */
