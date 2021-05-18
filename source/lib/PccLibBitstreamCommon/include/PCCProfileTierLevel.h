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
#include "PCCProfileToolsetConstraintsInformation.h"

namespace pcc {

// 8.3.4.2 Profile, Tier and Level Syntax
class ProfileTierLevel {
 public:
  ProfileTierLevel() :
      tierFlag_( false ),
      profileCodecGroupIdc_( 0 ),
      profileToolsetIdc_( 0 ),
      profileReconstructionIdc_( 0 ),
      levelIdc_( 0 ),
      numSubProfiles_( 0 ),
      extendedSubProfileFlag_( false ),
      toolConstraintsPresentFlag_( false ) {}
  ~ProfileTierLevel() { subProfileIdc_.clear(); }
  ProfileTierLevel& operator=( const ProfileTierLevel& ) = default;

  void allocate() { subProfileIdc_.resize( numSubProfiles_, 0 ); }

  bool                                  getTierFlag() { return tierFlag_; }
  uint8_t                               getProfileCodecGroupIdc() { return profileCodecGroupIdc_; }
  uint8_t                               getProfileToolsetIdc() { return profileToolsetIdc_; }
  uint8_t                               getProfileReconstructionIdc() { return profileReconstructionIdc_; }
  uint8_t                               getLevelIdc() { return levelIdc_; }
  uint8_t                               getNumSubProfiles() { return numSubProfiles_; }
  bool                                  getExtendedSubProfileFlag() { return extendedSubProfileFlag_; }
  bool                                  getToolConstraintsPresentFlag() { return toolConstraintsPresentFlag_; }
  uint8_t                               getSubProfileIdc( size_t index ) { return subProfileIdc_[index]; }
  ProfileToolsetConstraintsInformation& getProfileToolsetConstraintsInformation() {
    return profileToolsetConstraintsInformation_;
  }

  void setTierFlag( bool value ) { tierFlag_ = value; }
  void setProfileCodecGroupIdc( uint8_t value ) { profileCodecGroupIdc_ = value; }
  void setProfileToolsetIdc( uint8_t value ) { profileToolsetIdc_ = value; }
  void setProfileReconstructionIdc( uint8_t value ) { profileReconstructionIdc_ = value; }
  void setLevelIdc( uint8_t value ) { levelIdc_ = value; }
  void setNumSubProfiles( uint8_t value ) { numSubProfiles_ = value; }
  void setExtendedSubProfileFlag( bool value ) { extendedSubProfileFlag_ = value; }
  void setToolConstraintsPresentFlag( bool value ) { toolConstraintsPresentFlag_ = value; }
  void setSubProfileIdc( size_t index, uint8_t value ) { subProfileIdc_[index] = value; }

 private:
  bool                                 tierFlag_;
  uint8_t                              profileCodecGroupIdc_;
  uint8_t                              profileToolsetIdc_;
  uint8_t                              profileReconstructionIdc_;
  uint8_t                              levelIdc_;
  uint8_t                              numSubProfiles_;
  bool                                 extendedSubProfileFlag_;
  bool                                 toolConstraintsPresentFlag_;
  std::vector<uint8_t>                 subProfileIdc_;
  ProfileToolsetConstraintsInformation profileToolsetConstraintsInformation_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAM_PROFILETIERLEVEL_H