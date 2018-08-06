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
#ifndef PCCKdTree_h
#define PCCKdTree_h

#include "PCCCommon.h"

#include "PCCPointSet.h"

namespace pcc {

struct PCCPointDistInfo {
  double dist2;
  uint32_t index;
  bool operator<(const PCCPointDistInfo &rhs) const { return dist2 < rhs.dist2; }
};

struct PCCNNResult {
  PCCPointDistInfo *neighbors;
  size_t resultCount;
};

struct PCCRangeResult {
  PCCPointDistInfo *neighbors;
  size_t resultCount;
};

struct PCCNNQuery3 {
  PCCPoint3D point;
  double radius;
  size_t nearestNeighborCount;
};

struct PCCRangeQuery3 {
  PCCPoint3D point;
  double radius;
  size_t maxResultCount;
};

template <typename T>
class PCCHeap {
 public:
  PCCHeap(void) = default;
  PCCHeap(const PCCHeap &) = delete;
  PCCHeap &operator=(const PCCHeap &) = delete;
  PCCHeap(T *array, size_t capacity) : array(array), cap(capacity), sz(0) {}
  T &top() {
    assert(sz > 0);
    return array[0];
  }
  const T &top() const {
    assert(sz > 0);
    return array[0];
  }
  void push(const T &elem) {
    if (sz < cap) {
      size_t i = sz;
      array[i] = elem;
      size_t p = parent(i);
      while (i > 0 && array[p] < elem) {
        std::swap(array[i], array[p]);
        i = p;
        p = parent(i);
      }
      ++sz;
    } else if (elem < array[0]) {
      array[0] = elem;
      maxHeapify(0);
    }
  }
  void pop() {
    assert(sz > 0);
    array[0] = array[--sz];
    maxHeapify(0);
  }
  void clear() { sz = 0; }
  size_t size() const { return sz; }
  size_t capacity() const { return cap; }

 private:
  void maxHeapify(size_t i) {
    size_t l = left(i);
    size_t r = right(i);
    size_t g = (l < sz && array[i] < array[l]) ? l : i;
    if (r < sz && array[g] < array[r]) {
      g = r;
    }
    if (g != i) {
      std::swap(array[i], array[g]);
      maxHeapify(g);
    }
  }
  size_t parent(size_t index) const { return (index + (index & 1) - 1) / 2; }
  size_t left(size_t index) const { return 2 * index + 1; }
  size_t right(size_t index) const { return 2 * index + 2; }

 private:
  T *array;
  size_t cap;
  size_t sz;
};

class PCCStaticKdTree3 {
  struct PCCKdTree3Node {
    PCCPoint3D pos;
    uint32_t id;
    PCCAxis3 axis;
  };

 public:
  PCCStaticKdTree3(void) = default;
  PCCStaticKdTree3(const PCCStaticKdTree3 &) = default;
  PCCStaticKdTree3 &operator=(const PCCStaticKdTree3 &) = default;
  ~PCCStaticKdTree3(void) = default;
  size_t size() const { return nodes_.size(); }
  size_t capacity() const { return nodes_.capacity(); }
  void clear() { nodes_.resize(0); }
  void reserve(const size_t pointCount) { nodes_.reserve(pointCount); }
  void build( const PCCPointSet3 &pointCloud );
  void findNearestNeighbors( const PCCNNQuery3 &query, PCCNNResult &result ) const;
  void findNeighbors( const PCCRangeQuery3 &query, PCCRangeResult &result ) const;

  void init( const PCCPointSet3 &pointCloud );
  void build( const uint32_t start, const uint32_t end );
  PCCAxis3 computeSplitAxis( const uint32_t start, const uint32_t end );
  uint32_t findMedian( uint32_t start, uint32_t end, const PCCAxis3 splitAxis );
  void findNearestNeighbors( const uint32_t start, const uint32_t end, const PCCNNQuery3 &query2,
                             PCCHeap<PCCPointDistInfo> &neighbors ) const;
  void findNeighbors( const uint32_t start, const uint32_t end, const PCCRangeQuery3 &query2,
                      PCCRangeResult &result ) const;

 private:
  std::vector<PCCKdTree3Node> nodes_;
};

class PCCIncrementalKdTree3 {
  struct PCCIncrementalKdTree3Node {
    PCCPoint3D pos;
    uint32_t id;
    uint32_t left;
    uint32_t right;
    PCCAxis3 axis;
  };

 public:
  PCCIncrementalKdTree3(void) { root_ = PCC_UNDEFINED_INDEX; }
  PCCIncrementalKdTree3(const PCCIncrementalKdTree3 &) = default;
  PCCIncrementalKdTree3 &operator=(const PCCIncrementalKdTree3 &) = default;
  ~PCCIncrementalKdTree3(void) = default;

  void build( const PCCPointSet3 &pointCloud );

  size_t size() const { return nodes_.size(); }
  size_t capacity() const { return nodes_.capacity(); }
  void clear() {
    nodes_.resize(0);
    root_ = PCC_UNDEFINED_INDEX;
  }
  void reserve(const size_t pointCount) { nodes_.reserve(pointCount); }
  uint32_t insert( const PCCPoint3D point );
  void insert( const PCCPointSet3 &pointCloud );
  void balance() {
    if (!nodes_.empty()) {
      root_ = balance(0, uint32_t(nodes_.size()));
    }
  }
  void findNearestNeighbors( const PCCNNQuery3 &query, PCCNNResult &result ) const;
  void findNearestNeighbors2( const PCCNNQuery3 &query2, PCCNNResult &result ) const;
  void findNeighbors( const PCCRangeQuery3 &query, PCCRangeResult &result ) const;
  void append( const PCCPoint3D point );
  void append( const PCCPointSet3 &pointCloud );
 private:
   static PCCAxis3 nextAxis( const PCCAxis3 axis ){
     switch (axis) {
       case PCC_AXIS3_X:
         return PCC_AXIS3_Y;
       case PCC_AXIS3_Y:
         return PCC_AXIS3_Z;
       case PCC_AXIS3_Z:
         return PCC_AXIS3_X;
       case PCC_AXIS3_UNDEFINED:
         return PCC_AXIS3_X;
       default:
         return PCC_AXIS3_X;
     }
   }
   void insert( PCCIncrementalKdTree3Node &node, const uint32_t parent );
   PCCAxis3 computeSplitAxis( const uint32_t start, const uint32_t end ) const;
   uint32_t findMedian( uint32_t start, uint32_t end, const PCCAxis3 splitAxis );
   uint32_t balance( const uint32_t start, const uint32_t end );
   void findNearestNeighbors( const uint32_t current, const PCCNNQuery3 &query2,
                              PCCHeap<PCCPointDistInfo> &neighbors ) const;
   void findNeighbors( const uint32_t current, const PCCRangeQuery3 &query2,
                       PCCRangeResult &result ) const;

 private:
  std::vector<PCCIncrementalKdTree3Node> nodes_;
  uint32_t root_;
};
}
#endif /* PCCKdTree_h */
