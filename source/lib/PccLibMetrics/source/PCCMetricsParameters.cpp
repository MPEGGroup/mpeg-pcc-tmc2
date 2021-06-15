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
#include "PCCMetricsParameters.h"

using namespace pcc;

PCCMetricsParameters::PCCMetricsParameters() {
  computeMetrics_         = true;
  computeChecksum_        = true;
  startFrameNumber_       = 0;
  frameCount_             = 0;
  groupOfFramesSize_      = 32;
  uncompressedDataFolder_ = {};
  uncompressedDataPath_   = {};
  reconstructedDataPath_  = {};
  normalDataPath_         = {};
  nbThread_               = 0;
  resolution_             = 1023;
  dropDuplicates_         = 2;
  neighborsProc_          = 1;
  computeC2c_             = true;
  computeC2p_             = true;
  computeColor_           = true;
  computeLidar_           = false;
  computeReflectance_     = false;
  computeHausdorff_       = false;
}

PCCMetricsParameters::~PCCMetricsParameters() = default;

void PCCMetricsParameters::completePath() {
  if ( !uncompressedDataFolder_.empty() ) {
    if ( !uncompressedDataPath_.empty() ) { uncompressedDataPath_ = uncompressedDataFolder_ + uncompressedDataPath_; }
  }
}

void PCCMetricsParameters::print() {
  std::cout << "+ Parameters" << std::endl;
  std::cout << "\t   computeChecksum                      " << computeChecksum_ << std::endl;
  std::cout << "\t   computeMetrics                       " << computeMetrics_ << std::endl;
  std::cout << "\t   startFrameNumber                     " << startFrameNumber_ << std::endl;
  std::cout << "\t   frameCount                           " << frameCount_ << std::endl;
  std::cout << "\t   uncompressedDataPath                 " << uncompressedDataPath_ << std::endl;
  std::cout << "\t   reconstructedDataPath                " << reconstructedDataPath_ << std::endl;
  std::cout << "\t   normalDataPath                       " << normalDataPath_ << std::endl;
  std::cout << "\t   nbThread                             " << nbThread_ << std::endl;
  std::cout << "\t   resolution                           " << resolution_ << std::endl;
  std::cout << "\t   dropDuplicates                       " << dropDuplicates_ << std::endl;
  std::cout << "\t   neighborsProc                        " << neighborsProc_ << std::endl;
  std::cout << "\t   computeC2c                           " << computeC2c_ << std::endl;
  std::cout << "\t   computeC2p                           " << computeC2p_ << std::endl;
  std::cout << "\t   computeColor                         " << computeColor_ << std::endl;
  std::cout << "\t   computeLidar                         " << computeLidar_ << std::endl;
  std::cout << "\t   computeReflectance                   " << computeReflectance_ << std::endl;
  std::cout << "\t   computeHausdorff                     " << computeHausdorff_ << std::endl;
  std::cout << std::endl;
}

bool PCCMetricsParameters::check( bool checkFiles ) {
  bool ret = true;
  if ( computeMetrics_ ) {
    if ( checkFiles ) {
      if ( uncompressedDataPath_.empty() ) {
        std::cout << "uncompressedDataPath not set\n";
        std::cout << "WARNING: Source ply not correctly set: disable compute "
                     "metric. \n";
        std::cout << "WARNING: Source ply not correctly set: disable compute "
                     "metric. \n";
        computeMetrics_ = false;
      }
      if ( reconstructedDataPath_.empty() ) {
        std::cout << "reconstructedDataPath_ not set\n";
        std::cout << "WARNING: Reconstructed ply not correctly set: disable "
                     "compute metric. \n";
        std::cout << "WARNING: Reconstructed ply not correctly set: disable "
                     "compute metric. \n";
      }
    }
    if ( normalDataPath_.empty() ) { computeC2p_ = false; }
    if ( computeC2p_ && normalDataPath_.empty() ) {
      std::cout << "normalDataPath_ not set if computeC2p_ == true \n";
      std::cout << "WARNING: Normal ply not correctly set: disable compute "
                   "metric. \n";
      std::cout << "WARNING: Normal ply not correctly set: disable compute "
                   "metric. \n";
      computeMetrics_ = false;
    }
  }
  return ret;
}
