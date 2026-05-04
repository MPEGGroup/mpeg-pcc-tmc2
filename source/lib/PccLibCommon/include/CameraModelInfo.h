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

#pragma once

#include <vector>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <cstdint>

class CameraModelInfo {
 public:
  struct CameraModelParams {
    uint32_t image_id;  // AJT: This is important and needs to be discussed and consdidered: pose numbers are determined
                        // by number of images?
    uint32_t    camera_id;
    std::string image_name = "";  // AJT: I have included this just in case for future extension?

    uint32_t width  = 0;
    uint32_t height = 0;

    double fx = 0.f;
    double fy = 0.f;
    double cx = 0.f;
    double cy = 0.f;

    double qw = 1.0f;
    double qx = 0.f;
    double qy = 0.f;
    double qz = 0.f;

    double tx = 0.f;
    double ty = 0.f;
    double tz = 0.f;
  };

 public:
  CameraModelInfo() = default;

  void load( const std::string &cameras_file, const std::string &images_file );

  const std::vector<CameraModelParams> &getCamPoseParams() const { return frames_; }

  size_t numImages() const { return frames_.size(); }
  size_t numCameras() const;
  bool   hasSameIntrinsics() const;

 private:
  std::vector<CameraModelParams>                  frames_;
  std::unordered_map<uint32_t, CameraModelParams> cameras_;

  void loadCamerasTxt( const std::string &file, std::unordered_map<uint32_t, CameraModelParams> &camMap );

  void loadCamerasBin( const std::string &file, std::unordered_map<uint32_t, CameraModelParams> &camMap );

  void loadImagesTxt( const std::string &file, const std::unordered_map<uint32_t, CameraModelParams> &camMap );

  void loadImagesBin( const std::string &file, const std::unordered_map<uint32_t, CameraModelParams> &camMap );

  static bool isBinary( const std::string &file );
};
