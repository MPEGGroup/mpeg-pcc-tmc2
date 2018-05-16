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

namespace pcc {

class PCCNormalsGenerator3;
class PCCStaticKdTree3;
class PCCPatch;

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
  PCCPatchSegmenter3( void ) : nbThread_( 0 ) {}
  PCCPatchSegmenter3(const PCCPatchSegmenter3 &) = delete;
  PCCPatchSegmenter3 &operator=(const PCCPatchSegmenter3 &) = delete;
  ~PCCPatchSegmenter3() = default;
  void setNbThread( size_t nbThread );

  void compute( const PCCPointSet3 &geometry, const PCCPatchSegmenter3Parameters &params,
                std::vector<PCCPatch> &patches );
  void initialSegmentation( const PCCPointSet3 &geometry,
                            const PCCNormalsGenerator3 &normalsGen,
                            const PCCVector3D *orientations, const size_t orientationCount,
                            std::vector<size_t> &partition );
  void computeAdjacencyInfo( const PCCPointSet3 &pointCloud, const PCCStaticKdTree3 &kdtree,
                             std::vector<std::vector<size_t>> &adj, const size_t maxNNCount );
  void segmentPatches( const PCCPointSet3 &points, const PCCStaticKdTree3 &kdtree,
                       const size_t maxNNCount, const size_t minPointCountPerCC,
                       const size_t occupancyResolution,
                       const double maxAllowedDist2MissedPointsDetection,
                       const double maxAllowedDist2MissedPointsSelection,
                       const size_t surfaceThickness, const size_t maxAllowedDepth,
                       const std::vector<size_t> &partition,
                       std::vector<PCCPatch> &patches,
                       std::vector<size_t> &patchPartition,
                       std::vector<size_t> &resampledPatchPartition,
                       std::vector<size_t> missedPoints, PCCPointSet3 &resampled );
  void refineSegmentation( const PCCPointSet3 &pointCloud, const PCCStaticKdTree3 &kdtree,
                           const PCCNormalsGenerator3 &normalsGen,
                           const PCCVector3D *orientations, const size_t orientationCount,
                           const size_t maxNNCount, const double lambda,
                           const size_t iterationCount, std::vector<size_t> &partition );

 private:
  size_t nbThread_;
};
}

#endif /* PCCPatchSegmenter_h */
