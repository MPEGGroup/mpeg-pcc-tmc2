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
#ifndef PCC_BITSTREAM_PROFILETIERLEVEL_H
#define PCC_BITSTREAM_PROFILETIERLEVEL_H

#include "PCCBitstreamCommon.h"

namespace pcc {

// 7.3.4.2 Profile, Tier and Level Syntax
class ProfileTierLevel {
 public:
  ProfileTierLevel() :
      tierFlag_( false ),
      profileCodecGroupIdc_( 0 ),
      profilePccToolsetIdc_( 0 ),
      profileReconctructionIdc_( 0 ),
      levelIdc_( 0 ) {}
  ~ProfileTierLevel() {}
  ProfileTierLevel& operator=( const ProfileTierLevel& ) = default;
  bool              getTierFlag() { return tierFlag_; }
  uint8_t           getProfileCodecGroupIdc() { return profileCodecGroupIdc_; }
  uint8_t           getProfilePccToolsetIdc() { return profilePccToolsetIdc_; }
  uint8_t           getProfileReconctructionIdc() { return profileReconctructionIdc_; }
  uint8_t           getLevelIdc() { return levelIdc_; }
  void              setTierFlag( bool value ) { tierFlag_ = value; }
  void              setProfileCodecGroupIdc( uint8_t value ) { profileCodecGroupIdc_ = value; }
  void              setProfilePccToolsetIdc( uint8_t value ) { profilePccToolsetIdc_ = value; }
  void              setProfileReconctructionIdc( uint8_t value ) { profileReconctructionIdc_ = value; }
  void              setLevelIdc( uint8_t value ) { levelIdc_ = value; }

 private:
  bool    tierFlag_;
  uint8_t profileCodecGroupIdc_;
  uint8_t profilePccToolsetIdc_;
  uint8_t profileReconctructionIdc_;
  uint8_t levelIdc_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_PROFILETIERLEVEL_H