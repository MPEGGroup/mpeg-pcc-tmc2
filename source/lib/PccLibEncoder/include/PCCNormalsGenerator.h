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

#ifndef PCCNormalsGenerator_h
#define PCCNormalsGenerator_h

#include "PCCCommon.h"

namespace pcc {
enum PCCNormalsGeneratorOrientation {
  PCC_NORMALS_GENERATOR_ORIENTATION_NONE               = 0,
  PCC_NORMALS_GENERATOR_ORIENTATION_SPANNING_TREE      = 1,
  PCC_NORMALS_GENERATOR_ORIENTATION_VIEW_POINT         = 2,
  PCC_NORMALS_GENERATOR_ORIENTATION_CUBEMAP_PROJECTION = 3
};

struct PCCNormalsGenerator3Parameters {
  PCCVector3D                    viewPoint_;
  double                         radiusNormalSmoothing_;
  double                         radiusNormalEstimation_;
  double                         radiusNormalOrientation_;
  double                         weightNormalSmoothing_;
  size_t                         numberOfNearestNeighborsInNormalSmoothing_;
  size_t                         numberOfNearestNeighborsInNormalEstimation_;
  size_t                         numberOfNearestNeighborsInNormalOrientation_;
  size_t                         numberOfIterationsInNormalSmoothing_;
  PCCNormalsGeneratorOrientation orientationStrategy_;
  bool                           storeEigenvalues_;
  bool                           storeNumberOfNearestNeighborsInNormalEstimation_;
  bool                           storeCentroids_;
};

class PCCNormalsGenerator3 {
  struct PCCWeightedEdge {
    double   weight_;
    uint32_t start_;
    uint32_t end_;
    bool     operator<( const PCCWeightedEdge& rhs ) const {
      if ( weight_ == rhs.weight_ ) { return start_ == rhs.start_ ? end_ < rhs.end_ : start_ < rhs.start_; }
      return weight_ < rhs.weight_;
    }
  };

 public:
  PCCNormalsGenerator3( void )                        = default;
  PCCNormalsGenerator3( const PCCNormalsGenerator3& ) = delete;
  PCCNormalsGenerator3& operator=( const PCCNormalsGenerator3& ) = delete;
  ~PCCNormalsGenerator3()                                        = default;
  void                      clear() { normals_.resize( 0 ); }
  void                      init( const size_t pointCount, const PCCNormalsGenerator3Parameters& params );
  void                      compute( const PCCPointSet3&                   pointCloud,
                                     const PCCKdTree&                      kdtree,
                                     const PCCNormalsGenerator3Parameters& params,
                                     const size_t                          nbThread );
  std::vector<PCCVector3D>& getNormals() { return normals_; }
  PCCVector3D               getNormal( const size_t pos ) const {
    assert( pos < normals_.size() );
    return normals_[pos];
  }
  PCCVector3D getEigenvalues( const size_t pos ) const {
    assert( pos < eigenvalues_.size() );
    return eigenvalues_[pos];
  }
  PCCVector3D getCentroid( const size_t pos ) const {
    assert( pos < barycenters_.size() );
    return barycenters_[pos];
  }
  uint32_t getNumberOfNearestNeighborsInNormalEstimation( const size_t index ) const {
    assert( index < numberOfNearestNeighborsInNormalEstimation_.size() );
    return numberOfNearestNeighborsInNormalEstimation_[index];
  }
  size_t getNormalCount() const { return normals_.size(); }

  void computeNormal( const size_t                          index,
                      const PCCPointSet3&                   pointCloud,
                      const PCCKdTree&                      kdtree,
                      const PCCNormalsGenerator3Parameters& params,
                      PCCNNResult&                          nNResult );
  void computeNormals( const PCCPointSet3&                   pointCloud,
                       const PCCKdTree&                      kdtree,
                       const PCCNormalsGenerator3Parameters& params );
  void orientNormals( const PCCPointSet3&                   pointCloud,
                      const PCCKdTree&                      kdtree,
                      const PCCNormalsGenerator3Parameters& params );
  void addNeighbors( const uint32_t      current,
                     const PCCPointSet3& pointCloud,
                     const PCCKdTree&    kdtree,
                     PCCNNQuery3&        nNQuery,
                     PCCNNResult&        nNResult,
                     PCCVector3D&        accumulatedNormals,
                     size_t&             numberOfNormals );
  void smoothNormals( const PCCPointSet3&                   pointCloud,
                      const PCCKdTree&                      kdtree,
                      const PCCNormalsGenerator3Parameters& params );

 private:
  std::vector<PCCVector3D>             normals_;
  std::vector<PCCVector3D>             eigenvalues_;
  std::vector<PCCVector3D>             barycenters_;
  std::vector<uint32_t>                numberOfNearestNeighborsInNormalEstimation_;
  std::vector<uint32_t>                visited_;
  std::priority_queue<PCCWeightedEdge> edges_;
  size_t                               nbThread_;
};
}  // namespace pcc

#endif /* PCCNormalsGenerator_h */
