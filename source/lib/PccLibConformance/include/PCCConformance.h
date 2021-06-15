/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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

#ifndef PCCConformanceParser_h
#define PCCConformanceParser_h

#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <list>
#include <map>
#include <vector>
#include <utility>

namespace pcc {

class PCCConformanceParameters;
typedef std::vector<std::map<std::string, std::string>> KeyValMaps;

class PCCConformance {
 public:
  PCCConformance();
  ~PCCConformance();
  void check( const PCCConformanceParameters& params );

 private:
  bool compareLogFiles( std::string&, std::string&, const std::vector<std::string>&, KeyValMaps&, KeyValMaps& );
  void checkLevelLimits( uint8_t, double, KeyValMaps&, bool );
  template <typename T>
  inline bool checkLimit( const std::string& keyVal, T&, T& );
  template <typename T>
  inline void convertString( const std::string& keyValue, T& val );

  size_t levelLimitsCount_;
  size_t levelLimitsExceedCount_;
  size_t logFilesCount_;
  size_t logFilesMatchCount_;
  bool   logFileTestsMatch_;
  bool   levelLimitTestsMatch_;
};

}  // namespace pcc

#endif  //~PCCConformanceParser_h
