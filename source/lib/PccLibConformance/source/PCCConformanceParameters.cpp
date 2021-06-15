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
#include "PCCConformanceParameters.h"

using namespace pcc;

PCCConformanceParameters::PCCConformanceParameters() {
  checkConformance_ = false;
  path_             = {};
  levelIdc_         = 30;
  fps_              = 30;
}

PCCConformanceParameters::~PCCConformanceParameters() = default;

void PCCConformanceParameters::print() {
  std::cout << "+ Parameters" << std::endl;
  std::cout << "\t   checkConformance                     " << checkConformance_ << std::endl;
  std::cout << "\t   path                                 " << path_ << std::endl;
  std::cout << "\t   levelIdc                             " << levelIdc_ << std::endl;
  std::cout << "\t   fps                                  " << fps_ << std::endl;
  std::cout << std::endl;
}

bool PCCConformanceParameters::check() {
  bool ret = true;
  if ( checkConformance_ ) {
    if ( path_.empty() ) {
      std::cout << "path not set\n";
      ret = false;
    }
    if ( levelIdc_ == 0 ) {
      std::cout << "levelIdc not set\n";
      ret = false;
    }
    if ( fps_ == 0 ) {
      std::cout << "levelIdc not set\n";
      ret = false;
    }
  }
  return ret;
}
