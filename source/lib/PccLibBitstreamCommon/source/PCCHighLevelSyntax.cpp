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
#include "PCCBitstreamCommon.h"
#include "PCCHighLevelSyntax.h"

using namespace pcc;

PCCHighLevelSyntax::PCCHighLevelSyntax() {}

PCCHighLevelSyntax::~PCCHighLevelSyntax() {
  vpccParameterSets_.clear();
  atlasHLS_.clear();
}

size_t PCCAtlasHighLevelSyntax::getNumRefIdxActive( AtlasTileHeader& ath ) {
  size_t afpsId          = ath.getAtlasFrameParameterSetId();
  auto&  afps            = getAtlasFrameParameterSet( afpsId );
  size_t numRefIdxActive = 0;
  if ( ath.getType() == P_TILE || ath.getType() == SKIP_TILE ) {
    if ( ath.getNumRefIdxActiveOverrideFlag() ) {
      numRefIdxActive = ath.getNumRefIdxActiveMinus1() + 1;
    } else {
      auto& asps    = getAtlasSequenceParameterSet( afps.getAtlasSequenceParameterSetId() );
      auto& refList = ath.getRefAtlasFrameListSpsFlag() ? asps.getRefListStruct( ath.getRefAtlasFrameListIdx() )
                                                        : ath.getRefListStruct();
      numRefIdxActive =
          static_cast<size_t>( ( std::min )( static_cast<int>( refList.getNumRefEntries() ),
                                             static_cast<int>( afps.getNumRefIdxDefaultActiveMinus1() ) + 1 ) );
    }
  }
  return numRefIdxActive;
}

void PCCAtlasHighLevelSyntax::printVideoBitstream() {
  size_t index = 0;
  printf( "VideoBitstream list: \n" );
  for ( auto& value : videoBitstream_ ) {
    printf( "  * %zu / %zu: ", index, videoBitstream_.size() );
    value.trace();
    index++;
  }
  fflush( stdout );
}

PCCAtlasHighLevelSyntax::PCCAtlasHighLevelSyntax() {}

PCCAtlasHighLevelSyntax::~PCCAtlasHighLevelSyntax() { videoBitstream_.clear(); }

V3CParameterSet& PCCHighLevelSyntax::getActiveVpccParameterSets() {
  for ( auto& vps : vpccParameterSets_ ) {
    if ( vps.getV3CParameterSetId() == activeVPS_ ) { return vps; }
  }
  fprintf( stderr, "Error: the VPS of index: %u can't find in the VPS list\n", activeVPS_ );
  fflush( stdout );
  exit( -1 );
  return vpccParameterSets_[0];
}

size_t PCCHighLevelSyntax::checkProfile() {
  size_t ret = 0;
  if ( atlasHLS_.size() != 1 ) {
    std::cout << "ProfileToolsetConstraint Violation(1) : number of atlas should be 1\n";
    return 1;
  }
  auto& vps = getActiveVpccParameterSets();
  auto& ptl = vps.getProfileTierLevel();
  printf( " ptl.getToolConstraintsPresentFlag() = %zu \n", (size_t)ptl.getToolConstraintsPresentFlag() );
  if ( ptl.getToolConstraintsPresentFlag() == false ) {
    printf( "checkProfile return 0 \n" );
    fflush( stdout );
    return 0;
  }
  auto& ptci                                    = ptl.getProfileToolsetConstraintsInformation();
  auto  EOMContraintFlag                        = ptci.getEOMContraintFlag();
  auto  maxMapCountMinus1                       = ptci.getMaxMapCountMinus1();
  auto  maxAtlasCountMinus1                     = ptci.getMaxAtlasCountMinus1();
  auto  multipleMapStreamsConstraintFlag        = ptci.getMultipleMapStreamsConstraintFlag();
  auto  PLRConstraintFlag                       = ptci.getPLRConstraintFlag();
  auto  attributeMaxDimensionMinus1             = ptci.getAttributeMaxDimensionMinus1();
  auto  attributeMaxDimensionPartitionsMinus1   = ptci.getAttributeMaxDimensionPartitionsMinus1();
  auto  noEightOrientationsConstraintFlag       = ptci.getNoEightOrientationsConstraintFlag();
  auto  no45DegreeProjectionPatchConstraintFlag = ptci.getNo45DegreeProjectionPatchConstraintFlag();

  printf( "activeVPS_ : %zu\n", (size_t)activeVPS_ );
  printf( "---ProfileToolsetConstraintsInformation-------\n" );
  printf( " EOMContraintFlag                        : %zu\n", (size_t)EOMContraintFlag );
  printf( " maxMapCountMinus1                       : %zu\n", (size_t)maxMapCountMinus1 );
  printf( " maxAtlasCountMinus1                     : %zu\n", (size_t)maxAtlasCountMinus1 );
  printf( " multipleMapStreamsConstraintFlag        : %zu\n", (size_t)multipleMapStreamsConstraintFlag );
  printf( " PLRConstraintFlag                       : %zu\n", (size_t)PLRConstraintFlag );
  printf( " attributeMaxDimensionMinus1             : %zu\n", (size_t)attributeMaxDimensionMinus1 );
  printf( " attributeMaxDimensionPartitionsMinus1   : %zu\n", (size_t)attributeMaxDimensionPartitionsMinus1 );
  printf( " noEightOrientationsConstraintFlag       : %zu\n", (size_t)noEightOrientationsConstraintFlag );
  printf( " no45DegreeProjectionPatchConstraintFlag : %zu\n", (size_t)no45DegreeProjectionPatchConstraintFlag );

  // constraints by profile_toolset_constraints_information( )
  if ( multipleMapStreamsConstraintFlag ) {
    if ( vps.getMultipleMapStreamsPresentFlag( 0 ) == 1 ) {
      std::cout << "ProfileToolsetConstraint Violation(3) : MultipleMapStreamsPresentFlag is 1 wherea "
                   "ptci.multipleMapStreamsConstraintFlag is 1.\n";
      ret = 3;
    }
  }
  if ( vps.getMapCountMinus1( 0 ) > maxMapCountMinus1 ) {
    std::cout << "ProfileToolsetConstraint Violation(7) : mapCountMinus1 is set to " << vps.getMapCountMinus1( 0 )
              << " wherea ptci.maxMapCountMinus1_ is " << maxMapCountMinus1 << ". \n";
    ret = 7;
  }
  for ( auto& asps : atlasHLS_[0].getAtlasSequenceParameterSetList() ) {
    if ( EOMContraintFlag ) {
      if ( asps.getEomPatchEnabledFlag() != false ) {
        std::cout
            << "ProfileToolsetConstraint Violation(2) : EOMContraintFlag is 1 wherea ptci.EOMContraintFlag is 1\n";
        ret = 2;
      }
    }
    if ( PLRConstraintFlag ) {
      if ( asps.getPLREnabledFlag() ) {
        std::cout << "ProfileToolsetConstraint Violation(4) : pointLocalReconstruction is 1 wherea "
                     "ptci.PLRConstraintFlag is 1. \n";
        ret = 4;
      }
    }
    if ( noEightOrientationsConstraintFlag ) {
      if ( asps.getUseEightOrientationsFlag() ) {
        std::cout << "ProfileToolsetConstraint Violation(5) : useEightOrientations is 1 wherea "
                     "ptci.noEightOrientationsConstraintFlag is 1. \n";
        ret = 5;
      }
    }
    if ( no45DegreeProjectionPatchConstraintFlag ) {
      if ( asps.getExtendedProjectionEnabledFlag() ) {
        std::cout << "ProfileToolsetConstraint Violation(6) : getExtendedProjectionEnabledFlag is 1 wherea "
                     "ptci.no45DegreeProjectionPatchConstraintFlag is 1. \n";
        ret = 6;
      }
    }
  }

  // profile.reconstruction setting
  auto profileToolsetIdc        = ptl.getProfileToolsetIdc();
  auto profileReconstructionIdc = ptl.getProfileReconstructionIdc();
  for ( auto& asps : atlasHLS_[0].getAtlasSequenceParameterSetList() ) {
    // Profile Tools set idc
    // Basic
    if ( profileToolsetIdc == 0 ) {
      if ( asps.getEomPatchEnabledFlag() ) {
        std::cout << "ProfileToolsetConstraint Violation(11) : enhancedOccupancyMapCode is not 0 wherea "
                     "profileToolsetIdc is 0. \n";
        ret = 11;
      }
      if ( vps.getMultipleMapStreamsPresentFlag( 0 ) != 1 && vps.getMapCountMinus1( 0 ) > 0 ) {
        std::cout
            << "ProfileToolsetConstraint Violation(12) : multipleStreams is not 1 wherea profileToolsetIdc is 0. \n";
        ret = 12;
      }
      if ( asps.getPLREnabledFlag() ) {
        std::cout << "ProfileToolsetConstraint Violation(13) : pointLocalReconstruction is 1 wherea profileToolsetIdc "
                     "is 0. \n";
        ret = 13;
      }
      if ( asps.getUseEightOrientationsFlag() ) {
        std::cout
            << "ProfileToolsetConstraint Violation(14) : useEightOrientations is 1 wherea profileToolsetIdc is 0. \n";
        ret = 14;
      }
      if ( asps.getExtendedProjectionEnabledFlag() ) {
        std::cout << "ProfileToolsetConstraint Violation(15) : additionalProjection is 1  and wherea profileToolsetIdc "
                     "is 0. \n";
        ret = 15;
      }
    }  // profile
  }    // asps
  return ret;
}
