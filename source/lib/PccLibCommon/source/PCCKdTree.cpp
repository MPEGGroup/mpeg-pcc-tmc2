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

#include "PCCPointSet.h"
#include "PCCKdTree.h"

using namespace pcc;

void PCCStaticKdTree3::build(const PCCPointSet3 &pointCloud) {
  const size_t pointCount = pointCloud.getPointCount();
  init(pointCloud);
  if (pointCount) {
    build(0, uint32_t(pointCount));
  }
}
void PCCStaticKdTree3::findNearestNeighbors(const PCCNNQuery3 &query, PCCNNResult &result) const {
  assert(result.neighbors && query.nearestNeighborCount > 0);
  result.resultCount = 0;
  const PCCNNQuery3 query2 = {query.point, query.radius * query.radius,
                              query.nearestNeighborCount};
  PCCHeap<PCCPointDistInfo> neighbors(result.neighbors, query.nearestNeighborCount);
  findNearestNeighbors(0, uint32_t(nodes_.size()), query2, neighbors);
  std::sort(result.neighbors, result.neighbors + neighbors.size());
  result.resultCount = neighbors.size();
}
void PCCStaticKdTree3::findNeighbors(const PCCRangeQuery3 &query, PCCRangeResult &result) const {
  assert(result.neighbors && query.maxResultCount > 0);
  result.resultCount = 0;
  const PCCRangeQuery3 query2 = {query.point, query.radius * query.radius, query.maxResultCount};
  findNeighbors(0, uint32_t(nodes_.size()), query2, result);
}

void PCCStaticKdTree3::init(const PCCPointSet3 &pointCloud) {
  const size_t size = pointCloud.getPointCount();
  nodes_.resize(size);
  for (size_t i = 0; i < size; ++i) {
    nodes_[i].id = uint32_t(i);
    nodes_[i].axis = PCC_AXIS3_UNDEFINED;
    nodes_[i].pos = pointCloud[i];
  }
}
void PCCStaticKdTree3::build(const uint32_t start, const uint32_t end) {
  assert(start < end);
  uint32_t index = start;
  if (end == start + 1) {
    nodes_[start].axis = PCC_AXIS3_X;
  } else {
    PCCAxis3 splitAxis = computeSplitAxis(start, end);
    index = findMedian(start, end, splitAxis);
    PCCKdTree3Node &current = nodes_[index];
    current.axis = splitAxis;
    if (start < index) {
      build(start, index);
    }
    if (index + 1 < end) {
      build(index + 1, end);
    }
  }
}
PCCAxis3 PCCStaticKdTree3::computeSplitAxis(const uint32_t start, const uint32_t end) {
  PCCPoint3D minBB = nodes_[start].pos;
  PCCPoint3D maxBB = nodes_[start].pos;
  for (size_t i = start + 1; i < end; ++i) {
    const PCCPoint3D &pt = nodes_[i].pos;
    for (int32_t k = 0; k < 3; ++k) {
      if (minBB[k] > pt[k]) {
        minBB[k] = pt[k];
      } else if (maxBB[k] < pt[k]) {
        maxBB[k] = pt[k];
      }
    }
  }
  PCCPoint3D d = maxBB - minBB;
  if (d.x() > d.y() && d.x() > d.z()) {
    return PCC_AXIS3_X;
  } else if (d.y() > d.z()) {
    return PCC_AXIS3_Y;
  } else {
    return PCC_AXIS3_Z;
  }
}
uint32_t PCCStaticKdTree3::findMedian(uint32_t start, uint32_t end, const PCCAxis3 splitAxis) {
  assert(start < end);
  if (end == start + 1) {
    return start;
  }
  const uint32_t medianIndex = start + (end - start) / 2;
  while (1) {
    double pivot = nodes_[medianIndex].pos[splitAxis];
    std::swap(nodes_[medianIndex], nodes_[end - 1]);
    uint32_t store, p;
    for (store = p = start; p < end; p++) {
      if (nodes_[p].pos[splitAxis] < pivot) {
        if (p != store) {
          std::swap(nodes_[p], nodes_[store]);
        }
        ++store;
      }
    }
    std::swap(nodes_[store], nodes_[end - 1]);

    while (store < medianIndex &&
            PCCApproximatelyEqual(
                nodes_[store].pos[splitAxis],
                nodes_[store + 1].pos[splitAxis])) {  // optimization in case of duplicated values
      ++store;
    }

    if (store == medianIndex) {
      return medianIndex;
    } else if (store > medianIndex) {
      end = store;
    } else {
      start = store + 1;
    }
  }
}
void PCCStaticKdTree3::findNearestNeighbors(const uint32_t start, const uint32_t end, const PCCNNQuery3 &query2,
                          PCCHeap<PCCPointDistInfo> &neighbors) const {
  if (start >= end) {
    return;
  } else if (start + 1 == end) {
    const PCCKdTree3Node &medianNode = nodes_[start];
    const double dist2 = (query2.point - medianNode.pos).getNorm2();

    if (dist2 < query2.radius) {
      PCCPointDistInfo distInfo = {dist2, medianNode.id};
      neighbors.push(distInfo);
    }
  } else {
    const uint32_t medianIndex = start + (end - start) / 2;
    const PCCKdTree3Node &medianNode = nodes_[medianIndex];
    const double dist2 = (query2.point - medianNode.pos).getNorm2();
    const PCCAxis3 splitAxis = medianNode.axis;
    const double coord = medianNode.pos[splitAxis];
    const double dx = query2.point[splitAxis] - coord;
    const double dx2 = dx * dx;

    if (dx < 0.0) {
      findNearestNeighbors(start, medianIndex, query2, neighbors);
      if (dist2 < query2.radius) {
        PCCPointDistInfo distInfo = {dist2, medianNode.id};
        neighbors.push(distInfo);
      }
      if (dx2 > query2.radius ||
          (neighbors.size() == query2.nearestNeighborCount && dx2 > neighbors.top().dist2)) {
        return;
      }
      findNearestNeighbors(medianIndex + 1, end, query2, neighbors);
    } else {
      findNearestNeighbors(medianIndex + 1, end, query2, neighbors);
      if (dist2 < query2.radius) {
        PCCPointDistInfo distInfo = {dist2, medianNode.id};
        neighbors.push(distInfo);
      }
      if (dx2 > query2.radius ||
          (neighbors.size() == query2.nearestNeighborCount && dx2 > neighbors.top().dist2)) {
        return;
      }
      findNearestNeighbors(start, medianIndex, query2, neighbors);
    }
  }
}
void PCCStaticKdTree3::findNeighbors(const uint32_t start, const uint32_t end, const PCCRangeQuery3 &query2,
                    PCCRangeResult &result) const {
  if (start >= end || result.resultCount == query2.maxResultCount) {
    return;
  } else if (start + 1 == end) {
    const PCCKdTree3Node &medianNode = nodes_[start];
    const double dist2 = (query2.point - medianNode.pos).getNorm2();

    if (dist2 < query2.radius) {
      result.neighbors[result.resultCount].dist2 = dist2;
      result.neighbors[result.resultCount].index = medianNode.id;
      result.resultCount++;
    }
  } else {
    const uint32_t medianIndex = start + (end - start) / 2;
    const PCCKdTree3Node &medianNode = nodes_[medianIndex];
    const double dist2 = (query2.point - medianNode.pos).getNorm2();
    const PCCAxis3 splitAxis = medianNode.axis;
    const double coord = medianNode.pos[splitAxis];
    const double dx = query2.point[splitAxis] - coord;
    const double dx2 = dx * dx;

    if (dx < 0.0) {
      findNeighbors(start, medianIndex, query2, result);
      if (result.resultCount == query2.maxResultCount) {
        return;
      }
      if (dist2 < query2.radius) {
        result.neighbors[result.resultCount].dist2 = dist2;
        result.neighbors[result.resultCount].index = medianNode.id;
        result.resultCount++;
      }
      if (dx2 > query2.radius || result.resultCount == query2.maxResultCount) {
        return;
      }
      findNeighbors(medianIndex + 1, end, query2, result);
    } else {
      findNeighbors(medianIndex + 1, end, query2, result);
      if (result.resultCount == query2.maxResultCount) {
        return;
      }
      if (dist2 < query2.radius) {
        result.neighbors[result.resultCount].dist2 = dist2;
        result.neighbors[result.resultCount].index = medianNode.id;
        result.resultCount++;
      }
      if (dx2 > query2.radius || result.resultCount == query2.maxResultCount) {
        return;
      }
      findNeighbors(start, medianIndex, query2, result);
    }
  }
}


void PCCIncrementalKdTree3::build(const PCCPointSet3 &pointCloud) {
  clear();
  insert(pointCloud);
}

uint32_t PCCIncrementalKdTree3::insert(const PCCPoint3D point) {
  const uint32_t id = static_cast<uint32_t>(nodes_.size());
  nodes_.resize(id + 1);
  PCCIncrementalKdTree3Node &node = nodes_[id];
  node.pos = point;
  node.id = id;
  node.left = PCC_UNDEFINED_INDEX;
  node.right = PCC_UNDEFINED_INDEX;
  if (root_ == PCC_UNDEFINED_INDEX) {
    root_ = id;
    node.axis = PCC_AXIS3_X;
  } else {
    insert(node, root_);
  }
  return id;
}
void PCCIncrementalKdTree3::insert(const PCCPointSet3 &pointCloud) {
  if (pointCloud.getPointCount()) {
    const uint32_t start = static_cast<uint32_t>(nodes_.size());
    const uint32_t end = start + static_cast<uint32_t>(pointCloud.getPointCount());
    nodes_.resize(end);
    for (uint32_t index = start; index < end; ++index) {
      PCCIncrementalKdTree3Node &node = nodes_[index];
      node.pos = pointCloud[index - start];
      node.id = index;
    }
    root_ = balance(0, uint32_t(nodes_.size()));
  }
}

void PCCIncrementalKdTree3::findNearestNeighbors(const PCCNNQuery3 &query, PCCNNResult &result) const {
  assert(result.neighbors && query.nearestNeighborCount > 0);
  result.resultCount = 0;
  const PCCNNQuery3 query2 = {query.point, query.radius * query.radius,
                              query.nearestNeighborCount};
  PCCHeap<PCCPointDistInfo> neighbors(result.neighbors, query.nearestNeighborCount);
  findNearestNeighbors(root_, query2, neighbors);
  std::sort(result.neighbors, result.neighbors + neighbors.size());
  result.resultCount = neighbors.size();
}
void PCCIncrementalKdTree3::findNearestNeighbors2(const PCCNNQuery3 &query2, PCCNNResult &result) const {
  assert(result.neighbors && query2.nearestNeighborCount > 0);
  result.resultCount = 0;
  PCCHeap<PCCPointDistInfo> neighbors(result.neighbors, query2.nearestNeighborCount);
  findNearestNeighbors(root_, query2, neighbors);
  std::sort(result.neighbors, result.neighbors + neighbors.size());
  result.resultCount = neighbors.size();
}
void PCCIncrementalKdTree3::findNeighbors(const PCCRangeQuery3 &query, PCCRangeResult &result) const {
  assert(result.neighbors && query.maxResultCount > 0);
  result.resultCount = 0;
  const PCCRangeQuery3 query2 = {query.point, query.radius * query.radius, query.maxResultCount};
  findNeighbors(root_, query2, result);
}
void PCCIncrementalKdTree3::append(const PCCPoint3D point) {
  const uint32_t id = static_cast<uint32_t>(nodes_.size());
  nodes_.resize(id + 1);
  PCCIncrementalKdTree3Node &node = nodes_[id];
  node.pos = point;
  node.id = id;
  node.left = PCC_UNDEFINED_INDEX;
  node.right = PCC_UNDEFINED_INDEX;
}
void PCCIncrementalKdTree3::append(const PCCPointSet3 &pointCloud) {
  if (pointCloud.getPointCount()) {
    const uint32_t start = static_cast<uint32_t>(nodes_.size());
    const uint32_t end = start + static_cast<uint32_t>(pointCloud.getPointCount());
    nodes_.resize(end);
    for (uint32_t index = start; index < end; ++index) {
      PCCIncrementalKdTree3Node &node = nodes_[index];
      node.pos = pointCloud[index - start];
      node.id = index;
      node.left = PCC_UNDEFINED_INDEX;
      node.right = PCC_UNDEFINED_INDEX;
    }
  }
}

void PCCIncrementalKdTree3::insert(PCCIncrementalKdTree3Node &node, const uint32_t parent) {
  const PCCAxis3 axis = nodes_[parent].axis;
  uint32_t &index =
      (node.pos[axis] < nodes_[parent].pos[axis]) ? nodes_[parent].left : nodes_[parent].right;
  if (index == PCC_UNDEFINED_INDEX) {
    index = node.id;
    node.axis = nextAxis(axis);
  } else {
    insert(node, index);
  }
}
PCCAxis3 PCCIncrementalKdTree3::computeSplitAxis(const uint32_t start, const uint32_t end) const {
  PCCPoint3D minBB = nodes_[start].pos;
  PCCPoint3D maxBB = nodes_[start].pos;
  for (size_t i = start + 1; i < end; ++i) {
    const PCCPoint3D &pt = nodes_[i].pos;
    for (int32_t k = 0; k < 3; ++k) {
      if (minBB[k] > pt[k]) {
        minBB[k] = pt[k];
      } else if (maxBB[k] < pt[k]) {
        maxBB[k] = pt[k];
      }
    }
  }
  PCCPoint3D d = maxBB - minBB;
  if (d.x() > d.y() && d.x() > d.z()) {
    return PCC_AXIS3_X;
  } else if (d.y() > d.z()) {
    return PCC_AXIS3_Y;
  } else {
    return PCC_AXIS3_Z;
  }
}
uint32_t PCCIncrementalKdTree3::findMedian(uint32_t start, uint32_t end, const PCCAxis3 splitAxis) {
  assert(start < end);
  if (end == start + 1) {
    return start;
  }
  const uint32_t medianIndex = start + (end - start) / 2;
  while (1) {
    double pivot = nodes_[medianIndex].pos[splitAxis];
    std::swap(nodes_[medianIndex], nodes_[end - 1]);
    uint32_t store, p;
    for (store = p = start; p < end; p++) {
      if (nodes_[p].pos[splitAxis] < pivot) {
        if (p != store) {
          std::swap(nodes_[p], nodes_[store]);
        }
        ++store;
      }
    }
    std::swap(nodes_[store], nodes_[end - 1]);

    while (store < medianIndex &&
            PCCApproximatelyEqual(
                nodes_[store].pos[splitAxis],
                nodes_[store + 1].pos[splitAxis])) {  // optimization in case of duplicated values
      ++store;
    }

    if (store == medianIndex) {
      return medianIndex;
    } else if (store > medianIndex) {
      end = store;
    } else {
      start = store + 1;
    }
  }
}
uint32_t PCCIncrementalKdTree3::balance(const uint32_t start, const uint32_t end) {
  assert(start < end);
  uint32_t index = start;
  PCCAxis3 splitAxis = computeSplitAxis(start, end);
  index = findMedian(start, end, splitAxis);
  PCCIncrementalKdTree3Node &node = nodes_[index];
  node.axis = splitAxis;
  if (start < index) {
    node.left = balance(start, index);
  } else {
    node.left = PCC_UNDEFINED_INDEX;
  }
  if (index + 1 < end) {
    node.right = balance(index + 1, end);
  } else {
    node.right = PCC_UNDEFINED_INDEX;
  }
  return index;
}
void PCCIncrementalKdTree3::findNearestNeighbors(const uint32_t current, const PCCNNQuery3 &query2,
                          PCCHeap<PCCPointDistInfo> &neighbors) const {
  if (current == PCC_UNDEFINED_INDEX) {
    return;
  }
  const PCCIncrementalKdTree3Node &node = nodes_[current];
  const double dist2 = (query2.point - node.pos).getNorm2();
  const PCCAxis3 splitAxis = node.axis;
  const double coord = node.pos[splitAxis];
  const double dx = query2.point[splitAxis] - coord;
  uint32_t first, second;
  if (dx < 0.0) {
    first = node.left;
    second = node.right;
  } else {
    first = node.right;
    second = node.left;
  }
  findNearestNeighbors(first, query2, neighbors);
  if (dist2 < query2.radius) {
    PCCPointDistInfo distInfo = {dist2, node.id};
    neighbors.push(distInfo);
  }
  const double dx2 = dx * dx;
  if (dx2 > query2.radius ||
      (neighbors.size() == query2.nearestNeighborCount && dx2 > neighbors.top().dist2)) {
    return;
  }
  findNearestNeighbors(second, query2, neighbors);
}
void PCCIncrementalKdTree3::findNeighbors(const uint32_t current, const PCCRangeQuery3 &query2,
                    PCCRangeResult &result) const {
  if (current == PCC_UNDEFINED_INDEX) {
    return;
  }
  const PCCIncrementalKdTree3Node &node = nodes_[current];
  const double dist2 = (query2.point - node.pos).getNorm2();
  const PCCAxis3 splitAxis = node.axis;
  const double coord = node.pos[splitAxis];
  const double dx = query2.point[splitAxis] - coord;
  uint32_t first, second;
  if (dx < 0.0) {
    first = node.left;
    second = node.right;
  } else {
    first = node.right;
    second = node.left;
  }
  findNeighbors(first, query2, result);
  if (result.resultCount == query2.maxResultCount) {
    return;
  }
  if (dist2 < query2.radius) {
    result.neighbors[result.resultCount].dist2 = dist2;
    result.neighbors[result.resultCount].index = node.id;
    result.resultCount++;
  }
  if ((dx * dx) > query2.radius || result.resultCount == query2.maxResultCount) {
    return;
  }
  findNeighbors(second, query2, result);
}

