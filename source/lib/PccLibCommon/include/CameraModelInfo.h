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

enum class CameraModel : uint32_t { SIMPLE_PINHOLE = 0, PINHOLE = 1 };

CameraModel stringToModel( const std::string & );
const char *modelToString( CameraModel );
size_t      numParams( CameraModel );

class CameraModelInfo {
 public:
  struct CamIntrinsics {
    uint32_t            camera_id;
    CameraModel         model;
    uint64_t            width;
    uint64_t            height;
    std::vector<double> params;
  };

  struct CamExtrinsics {
    uint32_t    image_id;
    uint32_t    camera_id;
    std::string image_name;

    double qw, qx, qy, qz;
    double tx, ty, tz;
  };

  struct PinholeParams {
    uint64_t width  = 0;
    uint64_t height = 0;

    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
  };

  CameraModelInfo() = default;

  void readColmapData( const std::string &cameras_file, const std::string &images_file );

  const std::unordered_map<uint32_t, CamIntrinsics> &getIntrParams() const { return cameras_; }
  PinholeParams                                      getPinholeParams( uint32_t camera_id ) const;
  const std::vector<CamExtrinsics>                  &getExtrParams() const { return images_; }

  size_t numImages() const { return images_.size(); }
  size_t numCameras() const;
  bool   hasSameIntrinsics() const;

 private:
  std::unordered_map<uint32_t, CamIntrinsics> cameras_;
  std::vector<CamExtrinsics>                  images_;

  void readCamerasTxt( const std::string &file, std::unordered_map<uint32_t, CamIntrinsics> &camMap );

  void readCamerasBin( const std::string &file, std::unordered_map<uint32_t, CamIntrinsics> &camMap );

  void readImagesTxt( const std::string &file, const std::unordered_map<uint32_t, CamIntrinsics> &camMap );

  void readImagesBin( const std::string &file, const std::unordered_map<uint32_t, CamIntrinsics> &camMap );

  static bool          isBinary( const std::string &file );
  static PinholeParams convertToPinhole( const CamIntrinsics & );
};
