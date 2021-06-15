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
#ifndef PCCMetrics_h
#define PCCMetrics_h

#include "PCCCommon.h"

#include "PCCPointSet.h"
#include "PCCMetricsParameters.h"

namespace pcc {

class PCCGroupOfFrames;

/**
 * Note: This object is a integration of the mpeg-pcc-dmetric tool (
 *http://mpegx.int-evry.fr/software/MPEG/PCC/mpeg-pcc-dmetric ) in the
 *mpeg-pcc-tmc2 based on the works of : Dong Tian
 *<tian@merl.com> Maja Krivokuca <majakri01@gmail.com> Phil Chou
 *<philchou@msn.com>
 **/

class QualityMetrics {
 public:
  QualityMetrics();

  void setParameters( const PCCMetricsParameters& params );

  void compute( const PCCPointSet3& cloudA, const PCCPointSet3& cloudB );

  QualityMetrics operator+( const QualityMetrics& metric ) const;

  void print( char code );

 private:
  // point-2-point ( cloud 2 cloud ), benchmark metric
  float c2cMse_;
  float c2cHausdorff_;
  float c2cPsnr_;
  float c2cHausdorffPsnr_;

  // point-2-plane ( cloud 2 plane ), proposed metric
  float c2pMse_;
  float c2pHausdorff_;
  float c2pPsnr_;
  float c2pHausdorffPsnr_;

  // Color
  float colorMse_[3];
  float colorPsnr_[3];

  // point 2 plane ( cloud 2 plane ), proposed metric
  float psnr_;

  // reflectance
  float reflectanceMse_;
  float reflectancePsnr_;

  PCCMetricsParameters params_;
};

class PCCMetrics {
 public:
  PCCMetrics();
  ~PCCMetrics();
  void setParameters( const PCCMetricsParameters& params );
  void compute( const PCCGroupOfFrames& sources,
                const PCCGroupOfFrames& reconstructs,
                const PCCGroupOfFrames& normals );
  void compute( PCCPointSet3& source, PCCPointSet3& reconstruct, const PCCPointSet3& normalSource );
  void display();

 private:
  std::vector<size_t>         sourcePoints_;
  std::vector<size_t>         sourceDuplicates_;
  std::vector<size_t>         reconstructPoints_;
  std::vector<size_t>         reconstructDuplicates_;
  std::vector<QualityMetrics> quality1_;
  std::vector<QualityMetrics> quality2_;
  std::vector<QualityMetrics> qualityF_;
  PCCMetricsParameters        params_;
};

};  // namespace pcc

#endif /* PCCMetrics.h */
