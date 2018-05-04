
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

#include "PCCCommon.h"

namespace pcc {

class PCCContext;
class PCCFrameContext;
class PCCGroupOfFrames;
class PCCPointSet3;
template <typename T, size_t N>
class PCCVideo;
typedef pcc::PCCVideo<uint8_t, 3> PCCVideo3B;
template <typename T, size_t N>
class PCCImage;
typedef pcc::PCCImage<uint8_t, 3> PCCImage3B;

struct GeneratePointCloudParameters {
  size_t occupancyResolution_;
  size_t neighborCountSmoothing_;
  double radius2Smoothing_;
  double radius2BoundaryDetection_;
  double thresholdSmoothing_;
  bool   losslessGeo_;
  size_t nbThread_;
  bool   absoluteD1_;
  bool   constrainedPack_;
  bool   binArithCoding_;
};

class PCCCodec {
 public:
  PCCCodec();
  ~PCCCodec();

  void generatePointCloud( PCCGroupOfFrames& reconstructs, PCCContext& context,
                           const GeneratePointCloudParameters params );

  bool colorPointCloud( PCCGroupOfFrames& reconstructs, PCCContext& context,
                        const bool noAttributes, const ColorTransform colorTransform );

 private:
  void generatePointCloud( PCCPointSet3& reconstruct, PCCFrameContext& frame,
                           const PCCVideo3B &video,
                           const GeneratePointCloudParameters params,
                           std::vector<uint32_t> &partition );

  void smoothPointCloud( PCCPointSet3& reconstruct, PCCFrameContext& frame,
                         const std::vector<uint32_t> &partition,
                         const GeneratePointCloudParameters params );

  bool colorPointCloud( PCCPointSet3& reconstruct, PCCFrameContext& frame,
                        const PCCVideo3B &video, const bool noAttributes);

};

}; //~namespace

#endif /* PCCCodec_h */
