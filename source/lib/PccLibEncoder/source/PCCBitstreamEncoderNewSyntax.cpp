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
#include "ArithmeticCodec.h"
#include "PCCBitstream.h"
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCPatch.h"
#include "PCCGroupOfFrames.h"
#include "PCCEncoderParameters.h"
#include <tbb/tbb.h>
#include "PCCChrono.h"

#include "PCCBitstreamEncoderNewSyntax.h"

using namespace std;
using namespace pcc;

PCCBitstreamEncoderNewSyntax::PCCBitstreamEncoderNewSyntax() {}
PCCBitstreamEncoderNewSyntax::~PCCBitstreamEncoderNewSyntax() {}

void PCCBitstreamEncoderNewSyntax::setParameters( PCCEncoderParameters params ) { params = params; }

// jkei : are we(??) going to change it to while (...) style later?
int PCCBitstreamEncoderNewSyntax::encode( PCCContext& context, PCCBitstream& bitstream ) {
  vpccUnit( context, bitstream, VPCC_SPS );
  vpccUnit( context, bitstream, VPCC_PSD );
  vpccUnit( context, bitstream, VPCC_OVD );
  vpccUnit( context, bitstream, VPCC_GVD );
  vpccUnit( context, bitstream, VPCC_AVD );
  return 0;
}

void PCCBitstreamEncoderNewSyntax::vpccVideoDataUnit( PCCContext&   context,
                                                      PCCBitstream& bitstream,
                                                      VPCCUnitType  vpccUnitType ) {
  auto& sps = context.getSps();
  if ( vpccUnitType == VPCC_OVD ) {
    bitstream.write( context.getVideoBitstream( PCCVideoType::OccupancyMap ) );
  } else if ( vpccUnitType == VPCC_GVD ) {
    if ( !context.getAbsoluteD1() ) {
      bitstream.write( context.getVideoBitstream( PCCVideoType::GeometryD0 ) );
      bitstream.write( context.getVideoBitstream( PCCVideoType::GeometryD1 ) );
    } else {
      bitstream.write( context.getVideoBitstream( PCCVideoType::Geometry ) );
    }
    if ( context.getUseAdditionalPointsPatch() && sps.getPcmSeparateVideoPresentFlag() ) {
      bitstream.write( context.getVideoBitstream( PCCVideoType::GeometryMP ) );
    }
  } else if ( vpccUnitType == VPCC_AVD ) {
    if ( !context.getNoAttributes() ) {
      bitstream.write( context.getVideoBitstream( PCCVideoType::Texture ) );
      if ( context.getUseAdditionalPointsPatch() && sps.getPcmSeparateVideoPresentFlag() ) {
        bitstream.write( context.getVideoBitstream( PCCVideoType::TextureMP ) );
      }
    }
  }
}

void PCCBitstreamEncoderNewSyntax::EncodeUInt32( const uint32_t           value,
                                                 const uint32_t           bitCount,
                                                 o3dgc::Arithmetic_Codec& arithmeticEncoder,
                                                 o3dgc::Static_Bit_Model& bModel0 ) {
  uint32_t valueToEncode = PCCToLittleEndian<uint32_t>( value );
  for ( uint32_t i = 0; i < bitCount; ++i ) {
    arithmeticEncoder.encode( valueToEncode & 1, bModel0 );
    valueToEncode >>= 1;
  }
}

// 7.3.2 V-PCC unit syntax
void PCCBitstreamEncoderNewSyntax::vpccUnit( PCCContext&   context,
                                             PCCBitstream& bitstream,
                                             VPCCUnitType  vpccUnitType ) {
  vpccUnitHeader( context, bitstream, vpccUnitType );
  vpccUnitPayload( context, bitstream, vpccUnitType );
}

// 7.3.3 V-PCC unit header syntax
void PCCBitstreamEncoderNewSyntax::vpccUnitHeader( PCCContext&   context,
                                                   PCCBitstream& bitstream,
                                                   VPCCUnitType  vpccUnitType ) {
  auto& vpcc = context.getVPCC();
  auto& sps  = context.getSps();
  bitstream.write( vpccUnitType, 5 );  // u(5)
  if ( vpccUnitType == VPCC_AVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_OVD ||
       vpccUnitType == VPCC_PSD ) {
    bitstream.write( (uint32_t)vpcc.getSequenceParameterSetId(), 4 );  // u(4)
  }
  if ( vpccUnitType == VPCC_AVD ) {
    bitstream.write( (uint32_t)vpcc.getAttributeIndex(), 7 );  // u(7)
    if ( sps.getMultipleLayerStreamsPresentFlag() ) {
      bitstream.write( (uint32_t)vpcc.getLayerIndex(), 4 );  // u(4)
      pcmSeparateVideoData( context, bitstream, 11 );
    } else {
      pcmSeparateVideoData( context, bitstream, 15 );
    }
  } else if ( vpccUnitType == VPCC_GVD ) {
    if ( sps.getMultipleLayerStreamsPresentFlag() ) {
      bitstream.write( vpcc.getLayerIndex(), 4 );  // u(4)
      pcmSeparateVideoData( context, bitstream, 18 );
    } else {
      pcmSeparateVideoData( context, bitstream, 22 );
    }
  } else if ( vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_PSD ) {
    bitstream.write( (uint32_t)0, 23 );  // u(23)
  } else {
    bitstream.write( (uint32_t)0, 27 );  // u(27)
  }
}

// 7.3.4 PCM separate video data syntax
void PCCBitstreamEncoderNewSyntax::pcmSeparateVideoData( PCCContext&   context,
                                                         PCCBitstream& bitstream,
                                                         uint8_t       bitCount ) {
  auto& vpcc = context.getVPCC();
  auto& sps  = context.getSps();
  if ( sps.getPcmSeparateVideoPresentFlag() && !vpcc.getLayerIndex() ) {
    bitstream.write( (uint32_t)vpcc.getPCMVideoFlag(), 1 );  // u(1)
    bitstream.write( (uint32_t)0, bitCount );                // u(bitCount)
  } else {
    bitstream.write( (uint32_t)0, bitCount + 1 );  // u(bitCount + 1)
  }
}

// 7.3.5 V-PCC unit payload syntax
void PCCBitstreamEncoderNewSyntax::vpccUnitPayload( PCCContext&   context,
                                                    PCCBitstream& bitstream,
                                                    VPCCUnitType  vpccUnitType ) {
  auto& sps  = context.getSps();  // jkei: for future update
  auto& psdu = context.getPatchSequenceDataUnit();
  if ( vpccUnitType == VPCC_SPS ) {
    sequenceParameterSet( sps, bitstream );
  } else if ( vpccUnitType == VPCC_PSD ) {
    patchSequenceDataUnit( psdu, sps, bitstream );
  } else if ( vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_AVD ) {
    vpccVideoDataUnit( context, bitstream, vpccUnitType );
  }
}

// 7.3.6 Sequence parameter set syntax
void PCCBitstreamEncoderNewSyntax::sequenceParameterSet( SequenceParameterSet& sps,
                                                         PCCBitstream&         bitstream ) {
  int layerCountMinus1 = sps.getLayerCountMinus1();
  profileTierLevel( sps.getProfileTierLevel(), bitstream );
  bitstream.write( (uint32_t)sps.getSequenceParameterSetId(), 4 );   // u(4)
  bitstream.write( (uint32_t)sps.getFrameWidth(), 16 );              // u(16)
  bitstream.write( (uint32_t)sps.getFrameHeight(), 16 );             // u(16)
  bitstream.write( (uint32_t)sps.getAvgFrameRatePresentFlag(), 1 );  // u(1)
  if ( sps.getAvgFrameRatePresentFlag() ) {
    bitstream.write( (uint32_t)sps.getAvgFrameRate(), 16 );  // u(16)
  }
  bitstream.write( (uint32_t)sps.getEnhancedOccupancyMapForDepthFlag(), 1 );  // u(1)
  bitstream.write( (uint32_t)layerCountMinus1, 4 );                           // u(4)
  if ( layerCountMinus1 > 0 ) {
    bitstream.write( (uint32_t)sps.getMultipleLayerStreamsPresentFlag(), 1 );  // u(1)
  }
  auto& layerAbsoluteCodingEnabledFlag = sps.getLayerAbsoluteCodingEnabledFlag();
  auto& layerPredictorIndexDiff        = sps.getLayerPredictorIndexDiff();
  for ( size_t i = 0; i < layerCountMinus1; i++ ) {
    bitstream.write( (uint32_t)layerAbsoluteCodingEnabledFlag[i + 1], 1 );  // u(1)
    if ( ( layerAbsoluteCodingEnabledFlag[i + 1] == 0 ) ) {
      if ( i > 0 ) {
        bitstream.writeUvlc( (uint32_t)layerPredictorIndexDiff[i + 1] );  // ue(v)
      }
    }
  }
  bitstream.write( (uint32_t)sps.getPcmPatchEnabledFlag(), 1 );  // u(1)
  if ( sps.getPcmPatchEnabledFlag() ) {
    bitstream.write( (uint32_t)sps.getPcmSeparateVideoPresentFlag(), 1 );  // u(1)
  }

  occupancyParameterSet( sps.getOccupancyParameterSet(), bitstream );
  geometryParameterSet( sps.getGeometryParameterSet(), sps, bitstream );
  bitstream.write( (uint32_t)sps.getAttributeCount(), 16 );  // u(16)
  for ( size_t i = 0; i < sps.getAttributeCount(); i++ ) {
    attributeParameterSet( sps.getAttributeParameterSet( i ), sps, bitstream );
  }
  bitstream.write( (uint32_t)sps.getPatchSequenceOrientationEnabledFlag(), 1 );  // u(1)
  bitstream.write( (uint32_t)sps.getPatchInterPredictionEnabledFlag(), 1 );      // u(1)
  bitstream.write( (uint32_t)sps.getPixelDeinterleavingFlag(), 1 );              // u(1)
  bitstream.write( (uint32_t)sps.getPointLocalReconstructionEnabledFlag(), 1 );  // u(1)
  bitstream.write( (uint32_t)sps.getRemoveDuplicatePointEnabledFlag(), 1 );      // u(1)
  byteAlignment( bitstream );
}

// 7.3.7 Byte alignment syntax
void PCCBitstreamEncoderNewSyntax::byteAlignment( PCCBitstream& bitstream ) {
  bitstream.write( 1, 1 );  // f(1): equal to 1
  while ( !bitstream.byteAligned() ) {
    bitstream.write( 0, 1 );  // f(1): equal to 0
  }
}

// 7.3.8 Profile, tier, and level syntax
void PCCBitstreamEncoderNewSyntax::profileTierLevel( ProfileTierLevel& ptl,
                                                     PCCBitstream&     bitstream ) {
  bitstream.write( (uint32_t)ptl.getTierFlag(), 1 );    // u(1)
  bitstream.write( (uint32_t)ptl.getProfileIdc(), 1 );  // u(7)
  bitstream.write( (uint32_t)0, 48 );                   // u(48)
  bitstream.write( (uint32_t)ptl.getLevelIdc(), 8 );    // u(8)
}

// 7.3.9 Occupancy parameter set syntax
void PCCBitstreamEncoderNewSyntax::occupancyParameterSet( OccupancyParameterSet& ops,
                                                          PCCBitstream&          bitstream ) {
  bitstream.write( (uint32_t)ops.getOccupancyCodecId(), 8 );           // u(8)
  bitstream.write( (uint32_t)ops.getOccupancyPackingBlockSize(), 8 );  // u(8)
}

// 7.3.10 Geometry parameter set syntax
void PCCBitstreamEncoderNewSyntax::geometryParameterSet( GeometryParameterSet& gps,
                                                         SequenceParameterSet& sps,
                                                         PCCBitstream&         bitstream ) {
  bitstream.write( (uint32_t)gps.getGeometryCodecId(), 8 );
  bitstream.write( (uint32_t)gps.getGeometryNominal2dBitdepthMinus1(), 5 );            // u(5)
  bitstream.write( ( uint32_t )( gps.getGeometry3dCoordinatesBitdepthMinus1() ), 5 );  // u(5)
  if ( sps.getPcmSeparateVideoPresentFlag() ) {
    bitstream.write( (uint32_t)gps.getPcmGeometryCodecId(), 1 );  // u(8)
  }
  gps.getGeometryParamsEnabledFlag();  // u(1)
  if ( gps.getGeometryParamsEnabledFlag() ) {
    geometrySequenceParams( gps.getGeometrySequenceParams(), bitstream );
  }
  // jkei[??] is it changed as intended?
  bitstream.write( (uint32_t)gps.getGeometryPatchParamsEnabledFlag(), 1 );  // u(1)
  if ( gps.getGeometryPatchParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)gps.getGeometryPatchScaleParamsEnabledFlag(), 1 );     // u(1)
    bitstream.write( (uint32_t)gps.getGeometryPatchOffsetParamsEnabledFlag(), 1 );    // u(1)
    bitstream.write( (uint32_t)gps.getGeometryPatchRotationParamsEnabledFlag(), 1 );  // u(1)
    bitstream.write( (uint32_t)gps.getGeometryPatchPointSizeInfoEnabledFlag(), 1 );   // u(1)
    bitstream.write( (uint32_t)gps.getGeometryPatchPointShapeInfoEnabledFlag(), 1 );  // u(1)
  }
}

// 7.3.11 Geometry sequence metadata syntax
void PCCBitstreamEncoderNewSyntax::geometrySequenceParams( GeometrySequenceParams& gsp,
                                                           PCCBitstream&           bitstream ) {
  bitstream.write( (uint32_t)gsp.getGeometrySmoothingParamsPresentFlag(), 1 );  // u(1)
  bitstream.write( (uint32_t)gsp.getGeometryScaleParamsPresentFlag(), 1 );      // u(1)
  bitstream.write( (uint32_t)gsp.getGeometryOffsetParamsPresentFlag(), 1 );     // u(1)
  bitstream.write( (uint32_t)gsp.getGeometryRotationParamsPresentFlag(), 1 );   // u(1)
  bitstream.write( (uint32_t)gsp.getGeometryPointSizeInfoPresentFlag(), 1 );    // u(1)
  bitstream.write( (uint32_t)gsp.getGeometryPointShapeInfoPresentFlag(), 1 );   // u(1)
  if ( gsp.getGeometrySmoothingParamsPresentFlag() ) {
    bitstream.write( (uint32_t)gsp.getGeometrySmoothingEnabledFlag(), 1 );  // u(8)
    if ( gsp.getGeometrySmoothingEnabledFlag() ) {
      bitstream.write( (uint32_t)gsp.getGeometrySmoothingGridSize(), 8 );   // u(8)
      bitstream.write( (uint32_t)gsp.getGeometrySmoothingThreshold(), 8 );  // u(8)
    }
  }
  if ( gsp.getGeometryScaleParamsPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      bitstream.write( (int32_t)gsp.getGeometryScaleOnAxis( d ), 32 );  // u(32)
    }
    if ( gsp.getGeometryOffsetParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        bitstream.write( convertToUInt( gsp.getGeometryOffsetOnAxis( d ) ), 32 );  // i(32)
      }
    }
    if ( gsp.getGeometryRotationParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        bitstream.write( convertToUInt( gsp.getGeometryRotationOnAxis( d ) ), 32 );  // i(32)
      }
    }
    if ( gsp.getGeometryPointSizeInfoPresentFlag() ) {
      bitstream.write( (uint32_t)gsp.getGeometryPointSizeInfo(), 8 );  // u(8)
    }
    if ( gsp.getGeometryPointShapeInfoPresentFlag() ) {
      bitstream.write( (uint32_t)gsp.getGeometryPointShapeInfo(), 8 );  // u(8)
    }
  }
}

// 7.3.12 Attribute parameter set syntax
void PCCBitstreamEncoderNewSyntax::attributeParameterSet( AttributeParameterSet& aps,
                                                          SequenceParameterSet&  sps,
                                                          PCCBitstream&          bitstream ) {
  bitstream.write( (uint32_t)aps.getAttributeTypeId(), 4 );           // u(4)
  bitstream.write( (uint32_t)aps.getAttributeDimensionMinus1(), 8 );  // u(8)
  bitstream.write( (uint32_t)aps.getAttributeCodecId(), 8 );          // u(8)
  if ( sps.getPcmSeparateVideoPresentFlag() ) {
    bitstream.write( (uint32_t)aps.getPcmAttributeCodecId(), 8 );  // u(8)
  }
  bitstream.write( (uint32_t)aps.getAttributeParamsEnabledFlag(), 1 );  // u(1)
  if ( aps.getAttributeParamsEnabledFlag() ) {
    attributeSequenceParams( aps.getAttributeSequenceParams(), aps.getAttributeDimensionMinus1(),
                             bitstream );
  }
  bitstream.write( (uint32_t)aps.getAttributeParamsEnabledFlag(), 1 );  // u(1)
  if ( aps.getAttributeParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)aps.getAttributePatchScaleParamsEnabledFlag(), 1 );   // u(1)
    bitstream.write( (uint32_t)aps.getAttributePatchOffsetParamsEnabledFlag(), 1 );  // u(1)
  }
}

// 7.3.13 Attribute sequence Params syntax
void PCCBitstreamEncoderNewSyntax::attributeSequenceParams( AttributeSequenceParams& asp,
                                                            uint8_t                  dimension,
                                                            PCCBitstream&            bitstream ) {
  bitstream.write( (uint32_t)asp.getAttributeSmoothingParamsPresentFlag(), 1 );  // u(1)
  bitstream.write( (uint32_t)asp.getAttributeScaleParamsPresentFlag(), 1 );      // u(1)
  bitstream.write( (uint32_t)asp.getAttributeOffsetParamsPresentFlag(), 1 );     // u(1)
  if ( asp.getAttributeSmoothingParamsPresentFlag() ) {
    bitstream.write( (uint32_t)asp.getAttributeSmoothingRadius(), 8 );                    // u(8)
    bitstream.write( (uint32_t)asp.getAttributeSmoothingNeighbourCount(), 8 );            // u(8)
    bitstream.write( (uint32_t)asp.getAttributeSmoothingRadius2BoundaryDetection(), 8 );  // u(8)
    bitstream.write( (uint32_t)asp.getAttributeSmoothingThreshold(), 8 );                 // u(8)
    bitstream.write( (uint32_t)asp.getAttributeSmoothingThresholdLocalEntropy(), 3 );     // u(3)
  }
  if ( asp.getAttributeScaleParamsPresentFlag() ) {
    for ( size_t i = 0; i < dimension; i++ ) {
      bitstream.write( (uint32_t)asp.getAttributeScaleParams()[i], 32 );  // u(32)
    }
  }
  if ( asp.getAttributeOffsetParamsPresentFlag() ) {
    for ( size_t i = 0; i < dimension; i++ ) {
      bitstream.write( convertToUInt( asp.getAttributeOffsetParams()[i] ), 32 );  // i(32)
    }
  }
}

// 7.3.14 Patch sequence data unit syntax
void PCCBitstreamEncoderNewSyntax::patchSequenceDataUnit( PatchSequenceDataUnit& psdu,
                                                          SequenceParameterSet&  sps,
                                                          PCCBitstream&          bitstream ) {
  auto&  psup                                          = psdu.getPatchSequenceUnitPayload();
  bool   psd_terminate_patch_sequence_information_flag = false;
  size_t psdFrameCount                                 = 0;
  for ( size_t duCount = 0; duCount < psup.size(); duCount++ ) {
    bitstream.writeUvlc( (uint32_t)psup[duCount].getUnitType() );  // ue(v)
    patchSequenceUnitPayload( psup[duCount], sps, bitstream );
    psd_terminate_patch_sequence_information_flag = ( duCount + 1 ) == psup.size();
    bitstream.write( (uint32_t)psd_terminate_patch_sequence_information_flag,
                     1 );  // u(1) : psd_terminate_patch_sequence_information_flag

    if ( psup[psdFrameCount].getUnitType() == PSD_PFLU ) psdFrameCount++;
  }
  assert( psdFrameCount == psdu.getFrameCount() );
  byteAlignment( bitstream );
}

// 7.3.15 Patch sequence unit payload syntax
void PCCBitstreamEncoderNewSyntax::patchSequenceUnitPayload( PatchSequenceUnitPayload& psup,
                                                             SequenceParameterSet&     sps,
                                                             PCCBitstream&             bitstream ) {
  // jkei : do we need functions to call a parent object?
  if ( psup.getUnitType() == PSD_SPS ) {
    patchSequenceParameterSet( psup.getPatchSequenceParameterSet(), bitstream );
  } else if ( psup.getUnitType() == PSD_GPPS ) {
    geometryPatchParameterSet( psup.getGeometryPatchParameterSet(),
                               psup.getGeometryFrameParameterSet(), bitstream );
  } else if ( psup.getUnitType() == PSD_APPS ) {
    for ( int attributeIndex = 0; attributeIndex < sps.getAttributeCount(); attributeIndex++ ) {
      attributePatchParameterSet( psup.getAttributePatchParameterSet(),
                                  sps.getAttributeParameterSet( attributeIndex ),
                                  psup.getAttributeFrameParameterSet(), bitstream );
    }
  } else if ( psup.getUnitType() == PSD_FPS ) {
    patchFrameParameterSet( psup.getPatchFrameParameterSet(), sps, bitstream );
  } else if ( psup.getUnitType() == PSD_AFPS ) {
    for ( int attributeIndex = 0; attributeIndex < sps.getAttributeCount(); attributeIndex++ ) {
      attributeFrameParameterSet( psup.getAttributeFrameParameterSet(),
                                  sps.getAttributeParameterSet( attributeIndex ), bitstream );
    }
  } else if ( psup.getUnitType() == PSD_GFPS ) {
    geometryFrameParameterSet( psup.getGeometryFrameParameterSet(), sps.getGeometryParameterSet(),
                               bitstream );
  } else if ( psup.getUnitType() == PSD_PFLU ) {
    patchFrameLayerUnit( psup.getPatchFrameLayerUnit(), psup.getFrameIndex(), bitstream );
  }
}

// 7.3.16 Patch sequence parameter set syntax
void PCCBitstreamEncoderNewSyntax::patchSequenceParameterSet( PatchSequenceParameterSet& psps,
                                                              PCCBitstream& bitstream ) {
  bitstream.writeUvlc( psps.getPatchSequenceParameterSetId() );         // ue(v)
  bitstream.writeUvlc( psps.getLog2MaxPatchFrameOrderCntLsbMinus4() );  // ue(v)
  bitstream.writeUvlc( psps.getMaxDecPatchFrameBufferingMinus1() );     // ue(v)
  bitstream.write( psps.getLongTermRefPatchFramesFlag(), 1 );           // u(1)
  bitstream.writeUvlc( psps.getNumRefPatchFrameListsInSps() );          // ue(v)
  for ( size_t i = 0; i < psps.getNumRefPatchFrameListsInSps(); i++ ) {
    refListStruct( psps.getRefListStruct( i ), psps, bitstream );
  }
}

// 7.3.17 Geometry frame parameter set syntax
void PCCBitstreamEncoderNewSyntax::geometryFrameParameterSet( GeometryFrameParameterSet& gfps,
                                                              GeometryParameterSet&      gps,
                                                              PCCBitstream& bitstream ) {
  bitstream.writeUvlc(
      gfps.getGeometryFrameParameterSetId() );  // gfps_geometry_frame_parameter_set_id  ue(v)
  bitstream.writeUvlc(
      gfps.getPatchSequenceParameterSetId() );  // gfps_patch_sequence_parameter_set_id  ue(v)
  if ( gps.getGeometryParamsEnabledFlag() )     // if( gps_geometry_params_enabled_flag ) {
  {
    bitstream.write( (uint32_t)gfps.getOverrideGeometryParamsFlag(), 1 );  // u(1)
    if ( gfps.getOverrideGeometryParamsFlag() ) {
      geometryFrameParams( gfps.getGeometryFrameParams(), bitstream );
    }
  }

  if ( gps.getGeometryPatchParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)gfps.getOverrideGeometryPatchParamsFlag(), 1 );  // u(1)
    if ( gfps.getOverrideGeometryPatchParamsFlag() ) {
      bitstream.write( (uint32_t)gfps.getGeometryPatchScaleParamsEnabledFlag(), 1 );
      bitstream.write( (uint32_t)gfps.getGeometryPatchOffsetParamsEnabledFlag(), 1 );
      bitstream.write( (uint32_t)gfps.getGeometryPatchRotationParamsEnabledFlag(), 1 );
      bitstream.write( (uint32_t)gfps.getGeometryPatchPointSizeInfoEnabledFlag(), 1 );
      bitstream.write( (uint32_t)gfps.getGeometryPatchPointShapeInfoEnabledFlag(), 1 );
    }
  }
  byteAlignment( bitstream );
}

// 7.3.18 Geometry frame Params syntax
void PCCBitstreamEncoderNewSyntax::geometryFrameParams( GeometryFrameParams& gfp,
                                                        PCCBitstream&        bitstream ) {
  bitstream.write( (uint32_t)gfp.getGeometrySmoothingParamsPresentFlag(), 1 );  // u(1)
  bitstream.write( (uint32_t)gfp.getGeometryScaleParamsPresentFlag(), 1 );      // u(1)
  bitstream.write( (uint32_t)gfp.getGeometryOffsetParamsPresentFlag(), 1 );     // u(1)
  bitstream.write( (uint32_t)gfp.getGeometryRotationParamsPresentFlag(), 1 );   // u(1)
  bitstream.write( (uint32_t)gfp.getGeometryPointSizeInfoPresentFlag(), 1 );    // u(1)
  bitstream.write( (uint32_t)gfp.getGeometryPointShapeInfoPresentFlag(), 1 );   // u(1)

  if ( gfp.getGeometrySmoothingParamsPresentFlag() ) {
    bitstream.write( (uint32_t)gfp.getGeometrySmoothingEnabledFlag(), 1 );
    if ( gfp.getGeometrySmoothingEnabledFlag() ) {
      bitstream.write( (uint32_t)gfp.getGeometrySmoothingGridSize(), 8 );
      bitstream.write( (uint32_t)gfp.getGeometrySmoothingThreshold(), 8 );
    }
  }
  if ( gfp.getGeometryScaleParamsPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      bitstream.write( (uint32_t)gfp.getGeometryScaleOnAxis( d ),
                       32 );  //      gfp_geometry_scale_on_axis[ d ]  u(32)
    }
  }
  if ( gfp.getGeometryOffsetParamsPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      bitstream.write( convertToUInt( gfp.getGeometryOffsetOnAxis( d ) ), 32 );  // i32
    }
  }
  if ( gfp.getGeometryRotationParamsPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      bitstream.write( convertToUInt( gfp.getGeometryRotationOnAxis( d ) ), 32 );  // i32
    }
  }
  if ( gfp.getGeometryPointSizeInfoPresentFlag() ) {
    bitstream.write( (uint32_t)gfp.getGeometryPointSizeInfo(), 16 );
  }
  if ( gfp.getGeometryPointShapeInfoPresentFlag() ) {
    bitstream.write( (uint32_t)gfp.getGeometryPointShapeInfo(), 4 );
  }
}

// 7.3.19 Attribute frame parameter set syntax
void PCCBitstreamEncoderNewSyntax::attributeFrameParameterSet( AttributeFrameParameterSet& afps,
                                                               AttributeParameterSet&      aps,
                                                               PCCBitstream& bitstream ) {
  // jkei : this is attributeIndex-th afps
  bitstream.writeUvlc( afps.getAttributeFrameParameterSetId() );  // ue(v)
  bitstream.writeUvlc( afps.getPatchSequencParameterSetId() );    //  ue(v)

  size_t attributeDimension = aps.getAttributeDimensionMinus1() + 1;
  if ( aps.getAttributeParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)afps.getOverrideAttributeParamsFlag(), 1 );  // u(1)
    if ( afps.getOverrideAttributeParamsFlag() ) {
      attributeFrameParams( afps.getAttributeFrameParams(), attributeDimension, bitstream );
    }
  }
  if ( aps.getAttributePatchParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)afps.getOverrideAttributePatchParamsFlag(),
                     1 );  //    u(1)
    if ( afps.getOverrideAttributePatchParamsFlag() ) {
      bitstream.write( (uint32_t)afps.getAttributePatchScaleParamsEnabledFlag(), 1 );   //  u(1)
      bitstream.write( (uint32_t)afps.getAttributePatchOffsetParamsEnabledFlag(), 1 );  // u(1)
    }
  }

  byteAlignment( bitstream );
}

// 7.3.20 Attribute frame Params syntax
void PCCBitstreamEncoderNewSyntax::attributeFrameParams( AttributeFrameParams& afp,
                                                         size_t                attributeDimension,
                                                         PCCBitstream&         bitstream ) {
  bitstream.write( (uint32_t)afp.getAttributeSmoothingParamsPresentFlag(), 1 );  // u(1)
  bitstream.write( (uint32_t)afp.getAttributeScaleParamsPresentFlag(), 1 );      // u(1)
  bitstream.write( (uint32_t)afp.getAttributeOffsetParamsPresentFlag(), 1 );     // u(1)
  if ( afp.getAttributeSmoothingParamsPresentFlag() ) {
    bitstream.write( (uint32_t)afp.getAttributeSmoothingRadius(), 8 );                    //  u(8)
    bitstream.write( (uint32_t)afp.getAttributeSmoothingNeighbourCount(), 8 );            //   u(8)
    bitstream.write( (uint32_t)afp.getAttributeSmoothingRadius2BoundaryDetection(), 8 );  //  u(8)
    bitstream.write( (uint32_t)afp.getAttributeSmoothingThreshold(), 8 );                 // u(8)
    bitstream.write( (uint32_t)afp.getAttributeSmoothingThresholdLocalEntropy(), 3 );     // u(3)
  }

  if ( afp.getAttributeScaleParamsPresentFlag() ) {
    for ( size_t i = 0; i < attributeDimension; i++ )
      bitstream.write( (uint32_t)afp.getAttributeScale( i ), 32 ); // u(32)
  }
  if ( afp.getAttributeOffsetParamsPresentFlag() ) {
    for ( size_t i = 0; i < attributeDimension; i++ )
      bitstream.write( convertToUInt( afp.getAttributeOffset( i ) ), 32 );  // i32
  }
}

// 7.3.21 Geometry patch parameter set syntax
void PCCBitstreamEncoderNewSyntax::geometryPatchParameterSet( GeometryPatchParameterSet& gpps,
                                                              GeometryFrameParameterSet& gfps,
                                                              PCCBitstream& bitstream ) {
  bitstream.writeUvlc( gpps.getGeometryPatchParameterSetId() );  // ue(v)
  bitstream.writeUvlc( gpps.getGeometryFrameParameterSetId() );  //  ue(v)
  if ( gfps.getGeometryPatchScaleParamsEnabledFlag() ||
       gfps.getGeometryPatchOffsetParamsEnabledFlag() ||
       gfps.getGeometryPatchRotationParamsEnabledFlag() ||
       gfps.getGeometryPatchPointSizeInfoEnabledFlag() ||
       gfps.getGeometryPatchPointShapeInfoEnabledFlag() ) {
    bitstream.write( (uint32_t)gpps.getGeometryPatchParamsPresentFlag(), 1 );  // u(1)
    if ( gpps.getGeometryPatchParamsPresentFlag() )
      geometryPatchParams( gpps.getGeometryPatchParams(), gfps, bitstream );
  }

  byteAlignment( bitstream );
}

// 7.3.22 Geometry patch Params syntax
void PCCBitstreamEncoderNewSyntax::geometryPatchParams( GeometryPatchParams&       gpp,
                                                        GeometryFrameParameterSet& gfps,
                                                        PCCBitstream&              bitstream ) {
  if ( gfps.getGeometryPatchScaleParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)gpp.getGeometryPatchScaleParamsPresentFlag(), 1 );  // u(1)
    if ( gpp.getGeometryPatchScaleParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        bitstream.write( (uint32_t)gpp.getGeometryPatchScaleOnAxis( d ), 32 ); // u(32)
      }
    }
  }
  if ( gfps.getGeometryPatchOffsetParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)gpp.getGeometryPatchOffsetParamsPresentFlag(), 1 ); // u(1)
    if ( gpp.getGeometryPatchOffsetParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        bitstream.write( convertToUInt( gpp.getGeometryPatchOffsetOnAxis( d ) ), 32 );  // i32
      }
    }
  }
  if ( gfps.getGeometryPatchRotationParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)gpp.getGeometryPatchRotationParamsPresentFlag(), 1 ); // u(1)
    if ( gpp.getGeometryPatchRotationParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        bitstream.write( convertToUInt( gpp.getGeometryPatchRotationOnAxis( d ) ), 32 );  // i(32)
      }
    }
  }
  if ( gfps.getGeometryPatchPointSizeInfoEnabledFlag() ) {
    bitstream.write( (uint32_t)gpp.getGeometryPatchPointSizeInfoPresentFlag(), 1 ); // u(1)
    if ( gpp.getGeometryPatchPointSizeInfoPresentFlag() ) {
      bitstream.write( (uint32_t)gpp.getGeometryPatchPointSizeInfo(), 16 ); // u(16)
    }
  }
  if ( gfps.getGeometryPatchPointShapeInfoEnabledFlag() ) {
    bitstream.write( (uint32_t)gpp.getGeometryPatchPointShapeInfoPresentFlag(), 1 ); // u(1)
    if ( gpp.getGeometryPatchPointShapeInfoPresentFlag() ) {
      bitstream.write( (uint32_t)gpp.getGeometryPatchPointShapeInfo(), 4 ); // u(14
    }
  }
}

// 7.3.23 Attribute patch parameter set syntax
void PCCBitstreamEncoderNewSyntax::attributePatchParameterSet( AttributePatchParameterSet& apps,
                                                               AttributeParameterSet&      aps,
                                                               AttributeFrameParameterSet& afps,
                                                               PCCBitstream& bitstream ) {
  bitstream.writeUvlc( apps.getAttributePatchParameterSetId() );
  bitstream.writeUvlc( apps.getAttributeFrameParameterSetId() );
  size_t attributeDimension = aps.getAttributeDimensionMinus1() + 1;
  if ( afps.getAttributePatchScaleParamsEnabledFlag() ||
       afps.getAttributePatchOffsetParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)apps.getAttributePatchParamsPresentFlag(), 1 );   // u(1)
    if ( apps.getAttributePatchParamsPresentFlag() ) {
      attributePatchParams( apps.getAttributePatchParams(), afps, attributeDimension, bitstream );
    }
  }
  byteAlignment( bitstream );
}

// 7.3.24 Attribute patch Params syntax
void PCCBitstreamEncoderNewSyntax::attributePatchParams( AttributePatchParams&       app,
                                                         AttributeFrameParameterSet& afps,
                                                         size_t                      dimension,
                                                         PCCBitstream&               bitstream ) {
  if ( afps.getAttributePatchScaleParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)app.getAttributePatchScaleParamsPresentFlag(), 1 );  //  u(1)
    if ( app.getAttributePatchScaleParamsPresentFlag() ) {
      for ( size_t i = 0; i < dimension; i++ ) {
        bitstream.write( (uint32_t)app.getAttributePatchScale( i ), 32 );  //  u(32)
      }
    }
  }
  if ( afps.getAttributePatchOffsetParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)app.getAttributePatchOffsetParamsPresentFlag(), 1 );  //   u(1)
    if ( app.getAttributePatchOffsetParamsPresentFlag() ) {
      for ( size_t i = 0; i < dimension; i++ ) {
        bitstream.write( convertToUInt( app.getAttributePatchOffset( i ) ), 32 );  // i(32)
      }
    }
  }
}

// 7.3.25 Patch frame parameter set syntax
void PCCBitstreamEncoderNewSyntax::patchFrameParameterSet( PatchFrameParameterSet& pfps,
                                                           SequenceParameterSet&   sps,
                                                           PCCBitstream&           bitstream ) {
  bitstream.writeUvlc( pfps.getPatchFrameParameterSetId() );     // ue(v)
  bitstream.writeUvlc( pfps.getPatchSequenceParameterSetId() );  // ue(v)
  // Commented in CD: pfps.getGeometryPatchFrameParameterSetId;  // ue(v)
  // Commented in CD: for ( i = 0; i < spsAttributeCount; i++ ) {
  // Commented in CD:   pfps.getAttributePatchFrameParameterSetId[i]  // ue(v)
  // Commented in CD: }
  bitstream.write( pfps.getLocalOverrideGeometryPatchEnableFlag(), 1 );  // u(1)
  for ( size_t i = 0; i < sps.getAttributeCount(); i++ ) {
    bitstream.write( pfps.getLocalOverrideAttributePatchEnableFlag()[i], 1 );  // u(1)
  }
  // Commented in CD: pfps.getNumRefIdxDefaultActiveMinus1 = read() // ue(v)
  bitstream.writeUvlc( pfps.getAdditionalLtPfocLsbLen() );  // ue(v)
  if ( sps.getPatchSequenceOrientationEnabledFlag() ) {
    bitstream.write( pfps.getPatchOrientationPresentFlag(), 1 );  // u(1)
  }
  // Commented in CD: Other?
  byteAlignment( bitstream );
}

// 7.3.26 Patch frame layer unit syntax
void PCCBitstreamEncoderNewSyntax::patchFrameLayerUnit( PatchFrameLayerUnit& pflu,
                                                        size_t               frameIndex,
                                                        PCCBitstream&        bitstream ) {
  // patchFrameHeader( frameIndex );
  // patchFrameDataUnit( frameIndex );
}

// 7.3.27 Patch frame header syntax
void PCCBitstreamEncoderNewSyntax::patchFrameHeader( PatchFrameHeader& pfh,
                                                     size_t            frameIndex,
                                                     PCCBitstream&     bitstream ) {
  // pfhPatchFrameParameterSetId[frameIndex];  // ue(v)
  // pfhAddress[frameIndex];                   // u(v)
  // pfhType[frameIndex];                      // ue(v)
  // pfhPatchFrameOrderCntLsb[frameIndex];     // u(v)
  // if ( pspsNumRefPatchFrameListsInSps > 0 ) {
  //   pfhRefPatchFrameListSpsFlag[frameIndex];  // u(1)
  // }
  // if ( pfhRefPatchFrameListSpsFlag[frameIndex] ) {
  //   if ( pspsNumRefPatchFrameListsInSps > 1 ) {
  //     pfhRefPatchFrameListIdx[frameIndex];  // u(v)
  //   }
  // } else {
  //   refListStruct( pspsNumRefPatchFrameListsInSps );
  // }
  // for ( size_t j = 0; j < NumLtrpEntries; j++ ) {
  //   pfhAdditionalPfocLsbPresentFlag[frameIndex][j];  // u(1)
  //   if ( pfhAdditionalPfocLsbPresentFlag[frameIndex][j] ) {
  //     pfhAdditionalPfocLsbVal[frameIndex][j];  // u(v)
  //   }
  // }
  // if ( pfhType[frameIndex] == P && numRefEntries[RlsIdx] > 1 ) {
  //   pfhNumRefIdxActiveOverrideFlag[frameIndex];  // u(1)
  //   if ( pfhNumRefIdxActiveOverrideFlag[frameIndex] ) {
  //     pfhNumRefIdxActiveMinus1[frameIndex];  // ue(v)
  //   }
  // }
  // if ( pfhType[frameIndex] == I ) {
  //   pfhPatch2dShiftUBitCountMinus1[frameIndex];               // u( 8 )
  //   pfhPatch2dShiftVBitCountMinus1[frameIndex];               // u( 8 )
  //   pfhPatch3dShiftTangentAxisBitCountMinus1[frameIndex];    // u( 8 )
  //   pfhPatch3dShiftBitangentAxisBitCountMinus1[frameIndex];  // u( 8 )
  //   pfhPatch3dShiftNormalAxisBitCountMinus1[frameIndex];     // u( 8 )
  //   pfhPatchLodBitCount[frameIndex];                             //  u( 8 )
  // } else {
  //   pfhInterPredictPatchBitCountFlag;  // u(1)
  //   if ( pfhInterPredictPatchBitCountFlag ) {
  //     pfhInterPredictPatch2dShiftUBitCountFlag;  // u(1)
  //     if ( !pfhInterPredictPatch2dShiftUBitCountFlag ) {
  //       pfhPatch2dShiftUBitCountMinus1[frameIndex];  // u( 8 )
  //     }
  //     pfhInterPredictPatch2dShiftVBitCountFlag;  // u(1)
  //     if ( !pfhInterPredictPatch2dShiftVBitCountFlag ) {
  //       pfhPatch2dShiftVBitCountMinus1[frameIndex];  // u( 8 )
  //     }
  //     pfhInterPredictPatch3dShiftTangentAxisBitCountFlag;  // u(1)
  //     if ( !pfhInterPredictPatch3dShiftTangentAxisBitCountFlag ) {
  //       pfhPatch3dShiftTangentAxisBitCountMinus1[frameIndex];  // u( 8 )
  //     }
  //     pfhInterPredictPatch3dShiftBitangentAxisBitCountFlag;  // u(1)
  //     if ( !pfhInterPredictPatch3dShiftBitangentAxisBitCountFlag ) {
  //       pfhPatch3dShiftBitangentAxisBitCountMinus1[frameIndex];  // u( 8 )
  //     }
  //     pfhInterPredictPatch3dShiftNormalAxisBitCountFlag;  // u(1)
  //     if ( !pfhInterPredictPatch3dShiftNormalAxisBitCountFlag ) {
  //       pfhPatch3dShiftNormalAxisBitCountMinus1[frameIndex];  // u( 8 )
  //     }
  //     pfhInterPredictPatchLodBitCountFlag;  // u(1)
  //     if ( !pfhInterPredictPatchLodBitCountFlag ) {
  //       pfhPatchLodBitCount[frameIndex];  //  u( 8 )
  //     }
  //   }
  // }
  // byteAlignment( bitstream );
}

// 7.3.28 Reference list structure syntax
void PCCBitstreamEncoderNewSyntax::refListStruct( RefListStruct&             rls,
                                                  PatchSequenceParameterSet& psps,
                                                  PCCBitstream&              bitstream ) {
  bitstream.writeUvlc( rls.getNumRefEntries() );  // ue(v)
  rls.allocate();
  for ( size_t i = 0; i < rls.getNumRefEntries(); i++ ) {
    if ( psps.getLongTermRefPatchFramesFlag() ) {
      bitstream.write( rls.getStRefPatchFrameFlag()[i], 1 );  // u(1)
      if ( rls.getStRefPatchFrameFlag()[i] ) {
        bitstream.writeUvlc( rls.getAbsDeltaPfocSt()[i] );  // ue(v)
        if ( rls.getAbsDeltaPfocSt()[i] > 0 ) {
          bitstream.write( rls.getStrpfEntrySignFlag()[i], 1 );  // u(1)
        } else {
          bitstream.writeUvlc( rls.getPfocLsbLt()[i] );  // u(v)
        }
      }
    }
  }
}

// 7.3.29 Patch frame data unit syntax
void PCCBitstreamEncoderNewSyntax::patchFrameDataUnit( PCCContext&   context,
                                                       PCCBitstream& bitstream,
                                                       size_t        frameIndex ) {
  // patchFrameDataUnit( frmIdx ) {
  //   pfduPatchCountMinus1 ; // u(32)
  //   if( pfhType[ frmIdx ] != I )
  //     pfduInterPatchCount ; // ae(v)
  //   else
  //     pfduInterPatchCount = 0
  //   for( p = 0; p < pfduInterPatchCount; p++ ) {
  //     pfduPatchMode[ frmIdx ][ p ] = PINTER
  //     patchInformationData( frmIdx, p, pfduPatchMode[ frmIdx ][ p ] )
  //   }
  //   for( p = pfduInterPatchCount; p <= pfduPatchCountMinus1; p++ ) {
  //     pfduPatchMode[ frmIdx ][ p ] = pfhType[ frmIdx ]  ==  I ? IINTRA : PINTRA
  //     patchInformationData( frmIdx, p, pfduPatchMode[ frmIdx ][ p ] )
  //   }
  //   PfduTotalNumberOfPatches[ frmIdx ] = p
  //   if( spsPcmPatchEnabledFlag ) {
  //     pfduPatchMode[ frmIdx ][ p ] = pfhType[ frmIdx ]  ==  I ? IPCM : PPCM
  //     patchInformationData( frmIdx, p , pfduPatchMode[ frmIdx ][ p ] )
  //     PfduTotalNumberOfPatches[ frmIdx ]++
  //   }
  //   if( spsPointLocalReconstructionEnabledFlag )
  //     pointLocalReconstruction();
  //   byteAlignment();
  // }

  // patchFrameDataUnit( frmIdx) {
  //   p = -1
  //   pfduMorePatchesAvailableFlag ; // ae(v)
  //   while ( morePatchesAvailableFlag ) {
  //     p ++
  //     pfduPatchMode[ frmIdx ][ p ] ; // ae(v)
  //     patchInformationData(frmIdx, p, pfduPatchMode[ frmIdx ][ p ])
  //     pfduMorePatchesAvailableFlag ; // ae(v)
  //   }
  //   PfduTotalNumberOfPatches[ frmIdx ] = p + 1
  //   if( spsPointLocalReconstructionEnabledFlag )
  //     pointLocalReconstruction();
  //   byteAlignment();
  // }

  // // Vlad also proposed the following alternative mode
  // patchFrameDataUnit( frmIdx) {
  //   p = -1
  //   do {
  //     p ++
  //     pfduPatchMode[ frmIdx ][ p ] ; // ae(v)
  //     if( pfduPatchMode[ frmIdx ][ p ] !=IEND &&
  //       pfduPatchMode[ frmIdx ][ p ] != PEND)
  //       patchInformationData(frmIdx, p, pfduPatchMode[ frmIdx ][ p ])
  //   } while( pfduPatchMode[ frmIdx ][ p ] !=IEND &&
  //       pfduPatchMode[ frmIdx ][ p ] != PEND)
  //   PfduTotalNumberOfPatches[ frmIdx ] = p
  //   if( spsPointLocalReconstructionEnabledFlag )
  //     pointLocalReconstruction();
  //   byteAlignment();
  // }
}

// 7.3.30 Patch information data syntax
void PCCBitstreamEncoderNewSyntax::patchInformationData( PCCContext&   context,
                                                         PCCBitstream& bitstream,
                                                         size_t        frameIndex,
                                                         size_t        patchIndex,
                                                         size_t        patchMode ) {
  // if ( patchMode == PSKIP ) {
  //   // skip mode.
  //   // currently not supported but added it for convenience. Could easily be removed
  // } else if ( patchMode == IINTRA || patchMode == PINTRA ) {
  //   if ( pfpsLocalOverrideGeometryPatchEnableFlag ) {
  //     pidOverrideGeometryPatchFlag[frameIndex][patchIndex];  // u(1)
  //     if ( pidOverrideGeometryPatchFlag[frameIndex][patchIndex] ) {
  //       pidGeometryPatchParameterSetId[frameIndex][patchIndex];  // ue(v)
  //     }
  //   }
  //   for ( i = 0; i < spsAttributeCount; i++ ) {
  //     if ( pfpsLocalOverrideAttributePatchEnableFlag[i] ) {
  //       pidOverrideAttributePatchFlag[patchIndex][i];  // u(1)
  //     }
  //     if ( pidOverrideAttributePatchFlag[patchIndex][i] )
  //       pidAttributePatchParameterSetId[patchIndex][i];  // ue(v)
  //   }
  //   patchDataUnit( frameIndex, patchIndex );
  // } else if ( patchMode == PINTER ) {
  //   deltaPatchDataUnit( frameIndex, patchIndex );
  // } else if ( patchMode == IPCM || patchMode == PPCM ) {
  //   pcmPatchDataUnit( frameIndex, patchIndex );
  // }
}

// 7.3.31 Patch data unit syntax
void PCCBitstreamEncoderNewSyntax::patchDataUnit( PCCContext&   context,
                                                  PCCBitstream& bitstream,
                                                  size_t        frameIndex,
                                                  size_t        patchIndex ) {
  // pdu2dShiftU[frameIndex][patchIndex];              // ae(v)
  // pdu2dShiftV[frameIndex][patchIndex];              // ae(v)
  // pdu2dDeltaSizeU[frameIndex][patchIndex];          // ae(v)
  // pdu2dDeltaSizeV[frameIndex][patchIndex];          // ae(v)
  // pdu3dShiftTangentAxis[frameIndex][patchIndex];    // u(v)
  // pdu3dShiftBitangentAxis[frameIndex][patchIndex];  // u(v)
  // pdu3dShiftNormalAxis[frameIndex][patchIndex];     // u(v)
  // pduNormalAxis[frameIndex][patchIndex];              // ae(v)
  // if ( pfhPatchOrientationPresentFlag[frameIndex] ) {
  //   pduOrientationSwapFlag[frameIndex][patchIndex];  // ae(v)
  // }
  // if ( pfhPatchLodBitCount[frameIndex] > 0 ) {
  //    pduLod[frameIndex][patchIndex];  // u(v)
  // }
  // projectionFlag = 0;
  // i              = 0;
  // while ( i < spsLayerCountMinus1 + 1 && projectionFlag == 0 ) {
  //   projectionFlag = projectionFlag | spsLayerAbsoluteCodingEnabledFlag[i];
  //   i++;
  // }
  // if ( projectionFlag ) {
  //   pduProjectionMode[frameIndex][patchIndex]  // ae(v)
  // }
}

// 7.3.32  Delta Patch data unit syntax
void PCCBitstreamEncoderNewSyntax::deltaPatchDataUnit( PCCContext&   context,
                                                       PCCBitstream& bitstream,
                                                       size_t        frameIndex,
                                                       size_t        patchIndex ) {
  // dpduPatchIndex[frameIndex][patchIndex];            // ae(v)
  // dpdu2dShiftU[frameIndex][patchIndex];              // ae(v)
  // dpdu2dShiftV[frameIndex][patchIndex];              // ae(v)
  // dpdu2dDeltaSizeU[frameIndex][patchIndex];          // ae(v)
  // dpdu2dDeltaSizeV[frameIndex][patchIndex];          // ae(v)
  // dpdu3dShiftTangentAxis[frameIndex][patchIndex];    // ae(v)
  // dpdu3dShiftBitangentAxis[frameIndex][patchIndex];  // ae(v)
  // dpdu3dShiftNormalAxis[frameIndex][patchIndex];     // ae(v)
  // projectionFlag = 0
  // i = 0
  // while (i < spsLayerCountMinus1 + 1 && projectionFlag == 0 )  {
  //   projectionFlag = projectionFlag | spsLayerAbsoluteCodingEnabledFlag[ i ]
  //   i++;
  // }
  // if ( projectionFlag )  {
  //   dpduProjectionMode[ frmIdx ][ patchIndex ] ; // ae(v)
  // }
}

// 7.3.33 PCM patch data unit syntax
void PCCBitstreamEncoderNewSyntax::pcmPatchDataUnit( PCCContext&   context,
                                                     PCCBitstream& bitstream,
                                                     size_t        frameIndex,
                                                     size_t        patchIndex ) {
  // if ( spsPcmSeparateVideoFlag ) {
  //   ppduPatchInPcmVideoFlag[frameIndex][patchIndex];  // u(1)
  //   ppdu2dShiftU[frameIndex][patchIndex];             // u(v)
  //   ppdu2dShiftV[frameIndex][patchIndex];             // u(v)
  //   ppdu2dDeltaSizeU[frameIndex][patchIndex];         // u(v)
  //   ppdu2dDeltaSizeV[frameIndex][patchIndex];         // u(v)
  //   ppduPcmPoints[frameIndex][patchIndex];            // ue(v)
  // }
}

// 7.3.34 Point local reconstruction syntax
void PCCBitstreamEncoderNewSyntax::pointLocalReconstruction( PCCContext&   context,
                                                             PCCBitstream& bitstream ) {
  // for( j = 0; j < BlockToPatchMapHeight; j++ ) {
  //   for( i = 0; i < BlockToPatchMapWidth; i++ ) {
  //     if( BlockToPatchMap[ j ][ i ] >= 0 ) {
  //       plrModeInterpolateFlag[ j ][ i ]   ; // ae(v)
  //       if( plrModeInterpolateFlag[ j ][ i ] )
  //         plrModeNeighbour[ j ][ i ]  ; // ae(v)
  //       plrModeMinimumDepthMinus1[ j ][ i ]  ; // ae(v)
  //       if( plrModeMinimumDepthMinus1[ j ][ i ]  > 0
  //         || plrModeInterpolateFlag[ j ][ i ]  )
  //         plrModeFillingFlag[ j ][ i ]  ; // ae(v)
  //     }
  //   }
  // }
}
