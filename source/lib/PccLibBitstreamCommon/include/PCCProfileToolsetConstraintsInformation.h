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
#ifndef PCC_BITSTREAMPROFILETOOLSETCONSTRAINTSINFORMATION_H
#define PCC_BITSTREAMPROFILETOOLSETCONSTRAINTSINFORMATION_H

#include "PCCBitstreamCommon.h"

namespace pcc {

// 8.3.4.6	Profile toolset constraints information syntax
class ProfileToolsetConstraintsInformation {
 public:
  ProfileToolsetConstraintsInformation() :
      oneFrameOnlyFlag_( false ),
      EOMContraintFlag_( false ),
      maxMapCountMinus1_( 1 ),
      maxAtlasCountMinus1_( 0 ),
      multipleMapStreamsConstraintFlag_( false ),
      PLRConstraintFlag_( false ),
      attributeMaxDimensionMinus1_( 2 ),
      attributeMaxDimensionPartitionsMinus1_( 0 ),
      noEightOrientationsConstraintFlag_( true ),
      No45DegreeProjectionPatchConstraintFlag_( true ),
      NumReservedConstraintBytes_( 0 ) {}
  ~ProfileToolsetConstraintsInformation() { reservedConstraintByte_.clear(); }
  ProfileToolsetConstraintsInformation& operator=( const ProfileToolsetConstraintsInformation& ) = default;

  void allocate() { reservedConstraintByte_.resize( NumReservedConstraintBytes_, 0 ); }

  bool    getOneFrameOnlyFlag() { return oneFrameOnlyFlag_; }
  bool    getEOMContraintFlag() { return EOMContraintFlag_; }
  uint8_t getMaxMapCountMinus1() { return maxMapCountMinus1_; }
  uint8_t getMaxAtlasCountMinus1() { return maxAtlasCountMinus1_; }
  bool    getMultipleMapStreamsConstraintFlag() { return multipleMapStreamsConstraintFlag_; }
  bool    getPLRConstraintFlag() { return PLRConstraintFlag_; }
  uint8_t getAttributeMaxDimensionMinus1() { return attributeMaxDimensionMinus1_; }
  uint8_t getAttributeMaxDimensionPartitionsMinus1() { return attributeMaxDimensionPartitionsMinus1_; }
  bool    getNoEightOrientationsConstraintFlag() { return noEightOrientationsConstraintFlag_; }
  bool    getNo45DegreeProjectionPatchConstraintFlag() { return No45DegreeProjectionPatchConstraintFlag_; }
  uint8_t getNumReservedConstraintBytes() { return NumReservedConstraintBytes_; }
  uint8_t getReservedConstraintByte( size_t index ) { return reservedConstraintByte_[index]; }
  void    setOneFrameOnlyFlag( bool value ) { oneFrameOnlyFlag_ = value; }
  void    setEOMContraintFlag( bool value ) { EOMContraintFlag_ = value; }
  void    setMaxMapCountMinus1( bool value ) { maxMapCountMinus1_ = value; }
  void    setMaxAtlasCountMinus1( bool value ) { maxAtlasCountMinus1_ = value; }
  void    setMultipleMapStreamsConstraintFlag( bool value ) { multipleMapStreamsConstraintFlag_ = value; }
  void    setPLRConstraintFlag( bool value ) { PLRConstraintFlag_ = value; }
  void    setAttributeMaxDimensionMinus1( bool value ) { attributeMaxDimensionMinus1_ = value; }
  void    setAttributeMaxDimensionPartitionsMinus1( bool value ) { attributeMaxDimensionPartitionsMinus1_ = value; }
  void    setNoEightOrientationsConstraintFlag( bool value ) { noEightOrientationsConstraintFlag_ = value; }
  void    setNo45DegreeProjectionPatchConstraintFlag( bool value ) { No45DegreeProjectionPatchConstraintFlag_ = value; }
  void    setNumReservedConstraintBytes( bool value ) { NumReservedConstraintBytes_ = value; }
  void    setReservedConstraintByte( size_t index, bool value ) { reservedConstraintByte_[index] = value; }

 private:
  bool                 oneFrameOnlyFlag_;
  bool                 EOMContraintFlag_;
  uint8_t              maxMapCountMinus1_;
  uint8_t              maxAtlasCountMinus1_;
  bool                 multipleMapStreamsConstraintFlag_;
  bool                 PLRConstraintFlag_;
  uint8_t              attributeMaxDimensionMinus1_;
  uint8_t              attributeMaxDimensionPartitionsMinus1_;
  bool                 noEightOrientationsConstraintFlag_;
  bool                 No45DegreeProjectionPatchConstraintFlag_;
  uint8_t              NumReservedConstraintBytes_;
  std::vector<uint8_t> reservedConstraintByte_;
};

};  // namespace pcc

#endif  //~PCC_BITSTREAMPROFILETOOLSETCONSTRAINTSINFORMATION_H
