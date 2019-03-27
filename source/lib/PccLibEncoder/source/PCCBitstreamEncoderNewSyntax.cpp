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
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  auto& sps = context.getSps();
  if ( vpccUnitType == VPCC_OVD ) {
    bitstream.write( context.getVideoBitstream( PCCVideoType::OccupancyMap ) );
  } else if ( vpccUnitType == VPCC_GVD ) {
    if ( !sps.getLayerAbsoluteCodingEnabledFlag( 0 ) ) {
      bitstream.write( context.getVideoBitstream( PCCVideoType::GeometryD0 ) );
      bitstream.write( context.getVideoBitstream( PCCVideoType::GeometryD1 ) );
    } else {
      bitstream.write( context.getVideoBitstream( PCCVideoType::Geometry ) );
    }
    if ( sps.getPcmPatchEnabledFlag() && sps.getPcmSeparateVideoPresentFlag() ) {
      bitstream.write( context.getVideoBitstream( PCCVideoType::GeometryMP ) );
    }
  } else if ( vpccUnitType == VPCC_AVD ) {
    if ( sps.getAttributeCount() > 0 ) {
      bitstream.write( context.getVideoBitstream( PCCVideoType::Texture ) );
      if ( sps.getPcmPatchEnabledFlag() && sps.getPcmSeparateVideoPresentFlag() ) {
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
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  vpccUnitHeader( context, bitstream, vpccUnitType );
  vpccUnitPayload( context, bitstream, vpccUnitType );
}

// 7.3.3 V-PCC unit header syntax
void PCCBitstreamEncoderNewSyntax::vpccUnitHeader( PCCContext&   context,
                                                   PCCBitstream& bitstream,
                                                   VPCCUnitType  vpccUnitType ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
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
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
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
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  auto& sps  = context.getSps();  // jkei: for future update
  auto& psdu = context.getPatchSequenceDataUnit();
  if ( vpccUnitType == VPCC_SPS ) {
    sequenceParameterSet( sps, bitstream );
  } else if ( vpccUnitType == VPCC_PSD ) {
    patchSequenceDataUnit( context, bitstream );
  } else if ( vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_AVD ) {
    vpccVideoDataUnit( context, bitstream, vpccUnitType );
  }
}

// 7.3.6 Sequence parameter set syntax
void PCCBitstreamEncoderNewSyntax::sequenceParameterSet( SequenceParameterSet& sps,
                                                         PCCBitstream&         bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  profileTierLevel( sps.getProfileTierLevel(), bitstream );
  bitstream.write( (uint32_t)sps.getSequenceParameterSetId(), 4 );   // u(4)
  bitstream.write( (uint32_t)sps.getFrameWidth(), 16 );              // u(16)
  bitstream.write( (uint32_t)sps.getFrameHeight(), 16 );             // u(16)
  bitstream.write( (uint32_t)sps.getAvgFrameRatePresentFlag(), 1 );  // u(1)
  if ( sps.getAvgFrameRatePresentFlag() ) {
    bitstream.write( (uint32_t)sps.getAvgFrameRate(), 16 );  // u(16)
  }
  bitstream.write( (uint32_t)sps.getEnhancedOccupancyMapForDepthFlag(), 1 );  // u(1)
  bitstream.write( (uint32_t)sps.getLayerCountMinus1(), 4 );                           // u(4)
  if ( sps.getLayerCountMinus1() > 0 ) {
    bitstream.write( (uint32_t)sps.getMultipleLayerStreamsPresentFlag(), 1 );  // u(1)
  }
  auto layerAbsoluteCodingEnabledFlag = sps.getLayerAbsoluteCodingEnabledFlag( 0 );
  auto layerPredictorIndexDiff        = sps.getLayerPredictorIndexDiff( 0 );
  for ( size_t i = 0; i < sps.getLayerCountMinus1(); i++ ) {
    bitstream.write( (uint32_t)sps.getLayerAbsoluteCodingEnabledFlag( i + 1 ), 1 );  // u(1)
    if ( ( sps.getLayerAbsoluteCodingEnabledFlag( i + 1 ) == 0 ) ) {
      if ( i > 0 ) {
        bitstream.writeUvlc( (uint32_t)sps.getLayerPredictorIndexDiff( i + 1 ) );  // ue(v)
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
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  bitstream.write( 1, 1 );  // f(1): equal to 1
  while ( !bitstream.byteAligned() ) {
    bitstream.write( 0, 1 );  // f(1): equal to 0
  }
}

// 7.3.8 Profile, tier, and level syntax
void PCCBitstreamEncoderNewSyntax::profileTierLevel( ProfileTierLevel& ptl,
                                                     PCCBitstream&     bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  bitstream.write( (uint32_t)ptl.getTierFlag(), 1 );    // u(1)
  bitstream.write( (uint32_t)ptl.getProfileIdc(), 1 );  // u(7)
  bitstream.write( (uint32_t)0, 48 );                   // u(48)
  bitstream.write( (uint32_t)ptl.getLevelIdc(), 8 );    // u(8)
}

// 7.3.9 Occupancy parameter set syntax
void PCCBitstreamEncoderNewSyntax::occupancyParameterSet( OccupancyParameterSet& ops,
                                                          PCCBitstream&          bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  bitstream.write( (uint32_t)ops.getOccupancyCodecId(), 8 );           // u(8)
  bitstream.write( (uint32_t)ops.getOccupancyPackingBlockSize(), 8 );  // u(8)
}

// 7.3.10 Geometry parameter set syntax
void PCCBitstreamEncoderNewSyntax::geometryParameterSet( GeometryParameterSet& gps,
                                                         SequenceParameterSet& sps,
                                                         PCCBitstream&         bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  bitstream.write( (uint32_t)gps.getGeometryCodecId(), 8 );
  bitstream.write( (uint32_t)gps.getGeometryNominal2dBitdepthMinus1(), 5 );            // u(5)
  bitstream.write( ( uint32_t )( gps.getGeometry3dCoordinatesBitdepthMinus1() ), 5 );  // u(5)
  if ( sps.getPcmSeparateVideoPresentFlag() ) {
    bitstream.write( (uint32_t)gps.getPcmGeometryCodecId(), 1 );  // u(8)
  }
  bitstream.write( (uint32_t)gps.getGeometryParamsEnabledFlag(), 1 );  // u(1)
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
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
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
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
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
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
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
      bitstream.write( (uint32_t)asp.getAttributeScale( i ), 32 );  // u(32)
    }
  }
  if ( asp.getAttributeOffsetParamsPresentFlag() ) {
    for ( size_t i = 0; i < dimension; i++ ) {
      bitstream.write( convertToUInt( asp.getAttributeOffset( i ) ), 32 );  // i(32)
    }
  }
}

// 7.3.14 Patch sequence data unit syntax
void PCCBitstreamEncoderNewSyntax::patchSequenceDataUnit( PCCContext&   context,
                                                          PCCBitstream&          bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  auto&  sps                                           = context.getSps();
  auto&  psdu                                          = context.getPatchSequenceDataUnit();
  // auto&  psup                                          = psdu.getPatchSequenceUnitPayload();
  bool   psd_terminate_patch_sequence_information_flag = false;
  size_t psdFrameCount                                 = 0;
  printf(" psup size = %lu \n",psdu.getPatchSequenceDataUnitSize());
  for ( size_t duCount = 0; duCount < psdu.getPatchSequenceDataUnitSize(); duCount++ ) {
    bitstream.writeUvlc( (uint32_t) psdu.getPatchSequenceUnitPayload(duCount ).getUnitType() );  // ue(v)
    patchSequenceUnitPayload(  psdu.getPatchSequenceUnitPayload(duCount ), context, bitstream );
    psd_terminate_patch_sequence_information_flag = ( duCount + 1 ) == psdu.getPatchSequenceDataUnitSize();
    bitstream.write( (uint32_t)psd_terminate_patch_sequence_information_flag, 1 );  // u(1)
    if ( psdu.getPatchSequenceUnitPayload(psdFrameCount ).getUnitType() == PSD_PFLU ) 
		psdu.getPatchSequenceUnitPayload(psdFrameCount ).setFrameIndex( psdFrameCount++ );
  }
  assert( psdFrameCount == psdu.getFrameCount() );
  byteAlignment( bitstream );
}

// 7.3.15 Patch sequence unit payload syntax
void PCCBitstreamEncoderNewSyntax::patchSequenceUnitPayload( PatchSequenceUnitPayload& psup,
                                                             PCCContext&               context,
                                                             PCCBitstream&             bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  // jkei : do we need functions to call a parent object? --> we need to have access to upper level
  // objects in patch frame layer unit, so let's always carry the context variable and use it to
  // access those upper level variables
  auto& sps = context.getSps();
  if ( psup.getUnitType() == PSD_SPS ) {
    patchSequenceParameterSet( psup.getPatchSequenceParameterSet(), bitstream );
  } else if ( psup.getUnitType() == PSD_GPPS ) {
    geometryPatchParameterSet( psup.getGeometryPatchParameterSet(),
                               psup.getGeometryFrameParameterSet(), bitstream );
  } else if ( psup.getUnitType() == PSD_APPS ) {
    for ( int attributeIndex = 0; attributeIndex < sps.getAttributeCount(); attributeIndex++ ) {
      attributePatchParameterSet( psup.getAttributePatchParameterSet(attributeIndex),
                                  sps.getAttributeParameterSet( attributeIndex ),
                                  psup.getAttributeFrameParameterSet(attributeIndex), bitstream );
    }
  } else if ( psup.getUnitType() == PSD_FPS ) {
    patchFrameParameterSet( psup.getPatchFrameParameterSet(), sps, bitstream );
  } else if ( psup.getUnitType() == PSD_AFPS ) {
    for ( int attributeIndex = 0; attributeIndex < sps.getAttributeCount(); attributeIndex++ ) {
      attributeFrameParameterSet( psup.getAttributeFrameParameterSet(attributeIndex),
                                  sps.getAttributeParameterSet( attributeIndex ), bitstream );
    }
  } else if ( psup.getUnitType() == PSD_GFPS ) {
    geometryFrameParameterSet( psup.getGeometryFrameParameterSet(), sps.getGeometryParameterSet(),
                               bitstream );
  } else if ( psup.getUnitType() == PSD_PFLU ) {
    patchFrameLayerUnit( psup.getPatchFrameLayerUnit(), context, bitstream );
  }
}

// 7.3.16 Patch sequence parameter set syntax
void PCCBitstreamEncoderNewSyntax::patchSequenceParameterSet( PatchSequenceParameterSet& psps,
                                                              PCCBitstream& bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
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
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
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
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
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
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
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
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
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
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
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
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
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
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
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

// 7.3.24 Attribute patch Params syntax (apps)
void PCCBitstreamEncoderNewSyntax::attributePatchParams( AttributePatchParams&       app,
                                                         AttributeFrameParameterSet& afps,
                                                         size_t                      dimension,
                                                         PCCBitstream&               bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
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
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  bitstream.writeUvlc( pfps.getPatchFrameParameterSetId() );     // ue(v)
  bitstream.writeUvlc( pfps.getPatchSequenceParameterSetId() );  // ue(v)
  // Commented in CD: pfps.getGeometryPatchFrameParameterSetId;  // ue(v)
  // Commented in CD: for ( i = 0; i < spsAttributeCount; i++ ) {
  // Commented in CD:   pfps.getAttributePatchFrameParameterSetId[i]  // ue(v)
  // Commented in CD: }
  bitstream.write( pfps.getLocalOverrideGeometryPatchEnableFlag(), 1 );  // u(1)
  for ( size_t i = 0; i < sps.getAttributeCount(); i++ ) {
    bitstream.write( pfps.getLocalOverrideAttributePatchEnableFlag(i), 1 );  // u(1)
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
                                                        PCCContext&          context,
                                                        PCCBitstream&        bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  patchFrameHeader( pflu.getPatchFrameHeader(), context, bitstream );
  patchFrameDataUnit( pflu.getPatchFrameDataUnit(), pflu.getPatchFrameHeader(), context, bitstream);
}

// 7.3.27 Patch frame header syntax
void PCCBitstreamEncoderNewSyntax::patchFrameHeader( PatchFrameHeader& pfh,
                                                     PCCContext&       context,
                                                     PCCBitstream&     bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  

  auto& psdu = context.getPatchSequenceDataUnit();
  auto& psps = psdu.getPatchSequenceParameterSet( pfh.getPatchFrameParameterSetId() );
  bitstream.writeUvlc( pfh.getPatchFrameParameterSetId() );                                                 // ue(v )
  bitstream.writeUvlc( pfh.getAddress() );                                                        // u( v )
  bitstream.writeUvlc( pfh.getType() );                                                           // u( v )
  bitstream.writeUvlc( pfh.getPatchFrameOrderCntLsb() );                                                     // u( v )
  if (psps.getNumRefPatchFrameListsInSps() > 0 )
    bitstream.write( (uint32_t)pfh.getRefPatchFrameListSpsFlag(), 1 );                                      // u( 1 )
  if ( pfh.getRefPatchFrameListSpsFlag() ) {
    if ( psps.getNumRefPatchFrameListsInSps() > 1 ) {
      bitstream.writeUvlc( pfh.getRefPatchFrameListIdx() );                                                 // u( v )
    } else { 
      psps.getRefListStruct( psps.getNumRefPatchFrameListsInSps() );
    }
  }
  // RlsIdx[i] = psps_num_ref_patch_frame_lists_in_sps ?
  //   pfh_ref_patch_frame_list_idx[i] : psps_num_ref_patch_frame_lists_in_sps  (7 - 4)
  uint8_t rlsIdx = psps.getNumRefPatchFrameListsInSps() ? pfh.getRefPatchFrameListIdx()
                                                        : psps.getNumRefPatchFrameListsInSps();
  //NumLtrpfEntries[rlsIdx] = 0
  //  for (i = 0; i < num_ref_entries[rlsIdx]; i++)
  //    if (!st_ref_patch_frame_flag[rlsIdx][i])										           	(7 - 8)
  //      NumLtrpfEntries[rlsIdx]++
  size_t numLtrpEntries = 0;
  for ( size_t i = 0; i < psps.getRefListStruct( rlsIdx ).getNumRefEntries(); i++ ) {
    if ( !psps.getRefListStruct( rlsIdx ).getStRefPatchFrameFlag( i ) )
      numLtrpEntries++;
  }
  
  for ( size_t j; j < numLtrpEntries; j++ ){
    bitstream.write( (uint32_t)pfh.getAdditionalPfocLsbPresentFlag( j ) );                                  // u( 1 )
    if ( pfh.getAdditionalPfocLsbPresentFlag( j ) )                                                    
      bitstream.writeUvlc( pfh.getAdditionalPfocLsbVal( j ) );                                              // u( v )
  }
  
  if ( pfh.getType() == P_PATCH_FRAME && 
    psps.getRefListStruct(rlsIdx).getNumRefEntries() > 1 ) {
    bitstream.write( (uint32_t)pfh.getNumRefIdxActiveOverrideFlag(), 1 );                         // u( 1 )
    if ( pfh.getNumRefIdxActiveOverrideFlag() )                                            
      bitstream.writeUvlc( pfh.getNumRefIdxActiveMinus1() );                                      // u( v )
  }

  if ( pfh.getType() == I_PATCH_FRAME ) {
    bitstream.write( (uint32_t)pfh.getInterPredictPatch2dShiftUBitCountMinus1(), 8 );                        // u( 8 )
    bitstream.write( (uint32_t)pfh.getInterPredictPatch2dShiftVBitCountMinus1(), 8 );                        // u( 8 )
    bitstream.write( (uint32_t)pfh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1(), 8 );              // u( 8 )
    bitstream.write( (uint32_t)pfh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1(), 8 );            // u( 8 )
    bitstream.write( (uint32_t)pfh.getInterPredictPatch3dShiftNormalAxisBitCountMinus1(), 8 );               // u( 8 )
    bitstream.write( (uint32_t)pfh.getInterPredictPatchLodBitCount(), 8 );                                   // u( 8 )
  } else { 
    bitstream.write( (uint32_t)pfh.getInterPredictPatchBitCountFlag(), 1 );                       // u( 1 )
    if ( pfh.getInterPredictPatchBitCountFlag() ) {                                          
      bitstream.write( (uint32_t)pfh.getInterPredictPatch2dShiftUBitCountFlag(), 1 );             // u( 1 )
      if ( pfh.getInterPredictPatch2dShiftUBitCountFlag() )
        bitstream.write( (uint32_t)pfh.getInterPredictPatch2dShiftUBitCountMinus1(), 8 );                    // u( 8 )
      bitstream.write( (uint32_t)pfh.getInterPredictPatch2dShiftVBitCountFlag(), 1 );             // u( 1 )
      if ( pfh.getInterPredictPatch2dShiftVBitCountFlag() )
        bitstream.write( (uint32_t)pfh.getInterPredictPatch2dShiftVBitCountMinus1(), 8 );                    // u( 8 )
      bitstream.write( (uint32_t)pfh.getInterPredictPatch3dShiftTangentAxisBitCountFlag(), 1 );   // u( 1 )
      if ( pfh.getInterPredictPatch3dShiftTangentAxisBitCountFlag() )
        bitstream.write( (uint32_t)pfh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1(), 8 );          // u( 8 )
      bitstream.write( (uint32_t)pfh.getInterPredictPatch3dShiftBitangentAxisBitCountFlag(), 1 ); // u( 1 )
      if ( pfh.getInterPredictPatch3dShiftBitangentAxisBitCountFlag() )
        bitstream.write( (uint32_t)pfh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1(), 8 );        // u( 8 )
      bitstream.write( (uint32_t)pfh.getInterPredictPatch3dShiftNormalAxisBitCountFlag(), 1 );    // u( 1 )
      if ( pfh.getInterPredictPatch3dShiftNormalAxisBitCountFlag() )
        bitstream.write( (uint32_t)pfh.getInterPredictPatch3dShiftNormalAxisBitCountMinus1(), 8 );           // u( 8 )
      bitstream.write( (uint32_t)pfh.getInterPredictPatchLodBitCountFlag(), 1 );                  // u( 1 )
      if ( pfh.getInterPredictPatchLodBitCountFlag() )
        bitstream.write( (uint32_t)pfh.getInterPredictPatchLodBitCount(), 8 );                               // u( 8 )
    }
  }
  byteAlignment( bitstream );
}

// 7.3.28 Reference list structure syntax
void PCCBitstreamEncoderNewSyntax::refListStruct( RefListStruct&             rls,
                                                  PatchSequenceParameterSet& psps,
                                                  PCCBitstream&              bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  bitstream.writeUvlc( rls.getNumRefEntries() );  // ue(v)
  rls.allocate();
  for ( size_t i = 0; i < rls.getNumRefEntries(); i++ ) {
    if ( psps.getLongTermRefPatchFramesFlag() ) {
      bitstream.write( rls.getStRefPatchFrameFlag( i ), 1 );  // u(1)
      if ( rls.getStRefPatchFrameFlag( i ) ) {
        bitstream.writeUvlc( rls.getAbsDeltaPfocSt( i ) );  // ue(v)
        if ( rls.getAbsDeltaPfocSt( i ) > 0 ) {
          bitstream.write( rls.getStrpfEntrySignFlag( i ), 1 );  // u(1)
        } else {
          bitstream.writeUvlc( rls.getPfocLsbLt( i ) );  // u(v)
        }
      }
    }
  }
}

// 7.3.29 Patch frame data unit syntax
void PCCBitstreamEncoderNewSyntax::patchFrameDataUnit( PatchFrameDataUnit& pfdu,
                                                       PatchFrameHeader&   pfh,
                                                       PCCContext&         context,
                                                       PCCBitstream&       bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  

  auto&         sps                            = context.getSps();
  uint8_t       pfdu_more_available_patch_flag = pfdu.getPatchCount() > 0;
  const uint8_t bitCountPatchMode = ( PATCH_FRAME_TYPE( pfh.getType() ) ) == I_PATCH_FRAME ? 1 : 2;

  o3dgc::Arithmetic_Codec arithmeticEncoder;
  arithmeticEncoder.set_buffer( 0x00ffffff );
  arithmeticEncoder.start_encoder();

  o3dgc::Static_Bit_Model   bModel;
  o3dgc::Adaptive_Bit_Model bModelMoreAvailablePatchFlag;

  arithmeticEncoder.encode( pfdu_more_available_patch_flag,  bModelMoreAvailablePatchFlag );  // ae(v)
  for ( size_t puCount = 0; puCount < pfdu.getPatchCount(); puCount++ ) {
    patchInformationData( pfdu.getPatchInformationData( puCount ), pfdu.getPatchMode(puCount), pfh, context, bitstream, arithmeticEncoder );
    pfdu_more_available_patch_flag = !( ( puCount + 1 ) == pfdu.getPatchCount() );
    arithmeticEncoder.encode( pfdu_more_available_patch_flag, bModelMoreAvailablePatchFlag );  // ae(v)
  }
  if ( sps.getPointLocalReconstructionEnabledFlag() ) {
    pointLocalReconstruction( pfdu.getPointLocalReconstruction(), context, bitstream, arithmeticEncoder );
  }
  uint32_t compressedBitstreamSize = arithmeticEncoder.stop_encoder();
  bitstream.writeBuffer( arithmeticEncoder.buffer(), compressedBitstreamSize );

  byteAlignment( bitstream );
}

// 7.3.30 Patch information data syntax
void PCCBitstreamEncoderNewSyntax::patchInformationData(
    PatchInformationData&    pid,
    size_t                   patchMode,
    PatchFrameHeader&        pfh,
    PCCContext&              context,
    PCCBitstream&            bitstream,
    o3dgc::Arithmetic_Codec& arithmeticEncoder ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  

  auto& sps  = context.getSps();
  auto& psdu = context.getPatchSequenceDataUnit();
  auto& pfps = psdu.getPatchFrameParameterSet( pfh.getPatchFrameParameterSetId() );

  o3dgc::Static_Bit_Model bModel;
  const uint8_t           bitCountGAppsId = 6;
  if ( ( PATCH_FRAME_TYPE( pfh.getType() ) ) == P_PATCH_FRAME && patchMode == P_SKIP ) {
    // skip mode.
    // currently not supported but added it for convenience. Could easily be removed
  } else if ( ( ( PATCH_FRAME_TYPE( pfh.getType() ) ) == I_PATCH_FRAME && patchMode == I_INTRA )||
              ( ( PATCH_FRAME_TYPE( pfh.getType() ) ) == P_PATCH_FRAME && patchMode == P_INTRA ) ) {
    if ( pfps.getLocalOverrideGeometryPatchEnableFlag() ) {
      arithmeticEncoder.encode( pid.getOverrideGeometryPatchFlag(), bModel );  // ae(v) needs to clarify bypass vs. regular?
      if ( pid.getOverrideGeometryPatchFlag() ) {
        EncodeUInt32( uint32_t( pid.getGeometryPatchParameterSetId() ), bitCountGAppsId, arithmeticEncoder, bModel );  // ae(v)
      }
    }
    for ( int i = 0; i < sps.getAttributeCount(); i++ ) {
      if ( pfps.getLocalOverrideAttributePatchEnableFlag( i ) ) {
        arithmeticEncoder.encode( pid.getOverrideAttributePatchFlag( i ), bModel );  // ae(v) needs to clarify bypass vs. regular?
      }
      if ( pid.getOverrideAttributePatchFlag( i ) ) {
        EncodeUInt32( uint32_t( pid.getAttributePatchParameterSetId( i ) ), bitCountGAppsId, arithmeticEncoder, bModel );  // ae(v)
      }
    }

    auto& pdu = pid.getPatchDataUnit();
    patchDataUnit( pdu, pfh, context, bitstream, arithmeticEncoder );
  } else if ( ( PATCH_FRAME_TYPE( pfh.getType() ) ) == P_PATCH_FRAME && patchMode == P_INTER ) {
    auto& dpdu = pid.getDeltaPatchDataUnit();
    deltaPatchDataUnit( dpdu, pfh, context, bitstream, arithmeticEncoder );
  } else if ( ( ( PATCH_FRAME_TYPE( pfh.getType() ) ) == I_PATCH_FRAME && patchMode == I_PCM ) ||
              ( ( PATCH_FRAME_TYPE( pfh.getType() ) ) == P_PATCH_FRAME && patchMode == P_PCM )  ) {
    auto& ppdu = pid.getPCMPatchDataUnit();
    pcmPatchDataUnit( ppdu, pfh, context, bitstream, arithmeticEncoder );
  }
}

// 7.3.31 Patch data unit syntax
void PCCBitstreamEncoderNewSyntax::patchDataUnit( PatchDataUnit&           pdu,
                                                  PatchFrameHeader&        pfh,
                                                  PCCContext&              context,
                                                  PCCBitstream& bitstream,
                                                  o3dgc::Arithmetic_Codec& arithmeticEncoder ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  

  o3dgc::Static_Bit_Model   bModel;
  o3dgc::Adaptive_Bit_Model bModelIntSizeU0, bModelIntSizeV0, bModelOrientationSwapFlag, bModelProjectionFlag;
  o3dgc::Adaptive_Data_Model orientationModel( 4 );

  EncodeUInt32( uint32_t( pdu.get2DShiftU() ), pfh.getInterPredictPatch2dShiftUBitCountMinus1(),  arithmeticEncoder, bModel );  // ae(v)
  EncodeUInt32( uint32_t( pdu.get2DShiftV() ), pfh.getInterPredictPatch2dShiftVBitCountMinus1(),
                arithmeticEncoder, bModel );  // ae(v)
  arithmeticEncoder.ExpGolombEncode( o3dgc::IntToUInt( int32_t( pdu.get2DDeltaSizeU() ) ), 0,
                                     bModel, bModelIntSizeU0 );  // The way it is implemented in TM
  arithmeticEncoder.ExpGolombEncode( o3dgc::IntToUInt( int32_t( pdu.get2DDeltaSizeU() ) ), 0,
                                     bModel, bModelIntSizeV0 );  // The way it is implemented in TM

  EncodeUInt32( uint32_t( pdu.get3DShiftTangentAxis() ),
                pfh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1(), arithmeticEncoder,
                bModel );  // ae(v)
  EncodeUInt32( uint32_t( pdu.get3DShiftBiTangentAxis() ),
                pfh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1(), arithmeticEncoder,
                bModel );  // ae(v)
  EncodeUInt32( uint32_t( pdu.get3DShiftNormalAxis() ),
                pfh.getInterPredictPatch3dShiftNormalAxisBitCountMinus1(), arithmeticEncoder,
                bModel );  // ae(v)

  arithmeticEncoder.encode( uint32_t( pdu.getNormalAxis() ), orientationModel ); // in TM, this is being encoded with binarization (bBinArithCoding), what to do in that case???? 
  //    ******  Does not appear in CD and not implemented by Vlad  *******
  //if ( pfh.getPathOrientationPresentFlag() )
  //  arithmeticEncode.encode( pdu.getOrientationSwapFlag(), bModelOrientationSwapFlag );  // ae(v)

  if ( pfh.getInterPredictPatchLodBitCount() > 0 ) {
    EncodeUInt32( uint32_t( pdu.getLod() ), pfh.getInterPredictPatchLodBitCount(), arithmeticEncoder,
                  bModel );  // ae(v)
  }
  bool  projectionFlag = 0;
  int   i              = 0;
  auto& sps            = context.getSps();
  while ( i < sps.getLayerCountMinus1() + 1 && projectionFlag == 0 ) {
    projectionFlag = projectionFlag | sps.getLayerAbsoluteCodingEnabledFlag( i );
    i++;
  }
  if ( projectionFlag ) {
    arithmeticEncoder.encode( pdu.getProjectionMode(), bModelProjectionFlag );  // ae(v)
  }
}

// 7.3.32  Delta Patch data unit syntax
void PCCBitstreamEncoderNewSyntax::deltaPatchDataUnit( DeltaPatchDataUnit&      dpdu,
                                                       PatchFrameHeader&        pfh,
                                                       PCCContext&              context,
                                                       PCCBitstream&            bitstream,
                                                       o3dgc::Arithmetic_Codec& arithmeticEncoder ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif
  o3dgc::Static_Bit_Model    bModel;
  o3dgc::Adaptive_Bit_Model  bModelPatchIdx, bModelPatch2DShiftU, bModelPatch2DShiftV, bModelPatchSizeU, bModelPatchSizeV;
  o3dgc::Adaptive_Bit_Model  bModelPatch3DShiftNorm, bModelPatch3DShiftTan, bModelPatch3DShiftBitan; 
  o3dgc::Adaptive_Bit_Model  bModelProjectionFlag;
  
  arithmeticEncoder.ExpGolombEncode( o3dgc::IntToUInt( int32_t( dpdu.getDeltaPatchIdx() ) ), 0,
                                     bModel, bModelPatchIdx );
  arithmeticEncoder.ExpGolombEncode( o3dgc::IntToUInt( int32_t( dpdu.get2DDeltaShiftU() ) ), 0,
                                     bModel, bModelPatch2DShiftU );
  arithmeticEncoder.ExpGolombEncode( o3dgc::IntToUInt( int32_t( dpdu.get2DDeltaShiftV() ) ), 0,
                                     bModel, bModelPatch2DShiftV );
  arithmeticEncoder.ExpGolombEncode( o3dgc::IntToUInt( int32_t( dpdu.get2DDeltaSizeU() ) ), 0,
                                     bModel, bModelPatchSizeU );
  arithmeticEncoder.ExpGolombEncode( o3dgc::IntToUInt( int32_t( dpdu.get2DDeltaSizeV() ) ), 0,
                                     bModel, bModelPatchSizeV );
  arithmeticEncoder.ExpGolombEncode( o3dgc::IntToUInt( int32_t( dpdu.get3DDeltaShiftTangentAxis() ) ), 0,
                                     bModel, bModelPatch3DShiftTan );
  arithmeticEncoder.ExpGolombEncode( o3dgc::IntToUInt( int32_t( dpdu.get3DDeltaShiftBiTangentAxis() ) ), 0,
                                     bModel, bModelPatch3DShiftBitan );
  arithmeticEncoder.ExpGolombEncode( o3dgc::IntToUInt( int32_t( dpdu.get3DDeltaShiftNormalAxis() ) ), 0,
                                     bModel, bModelPatch3DShiftNorm );
  bool  projectionFlag = 0;
  int   i              = 0;
  auto& sps            = context.getSps();
  while ( i < sps.getLayerCountMinus1() + 1 && projectionFlag == 0 ) {
    projectionFlag = projectionFlag | sps.getLayerAbsoluteCodingEnabledFlag( i );
    i++;
  }
  if ( projectionFlag ){ 
    arithmeticEncoder.encode( dpdu.getProjectionMode(), bModelProjectionFlag );
  }
}

// 7.3.33 PCM patch data unit syntax
void PCCBitstreamEncoderNewSyntax::pcmPatchDataUnit( PCMPatchDataUnit&        ppdu,
                                                     PatchFrameHeader&        pfh,
                                                     PCCContext&              context,
                                                     PCCBitstream&            bitstream,
                                                     o3dgc::Arithmetic_Codec& arithmeticEncoder) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif
  o3dgc::Static_Bit_Model    bModel;
  o3dgc::Adaptive_Bit_Model  bModelVideoPCMFlag;
  o3dgc::Adaptive_Bit_Model  bModelIntSizeU, bModelIntSizeV, bModelPcmPoints;

  auto& sps = context.getSps();
  if ( sps.getPcmSeparateVideoPresentFlag() ){
    arithmeticEncoder.encode( ppdu.getPatchInPcmVideoFlag(), bModelVideoPCMFlag );
  }
  EncodeUInt32( uint32_t( ppdu.get2DShiftU() ), pfh.getInterPredictPatch2dShiftUBitCountMinus1(), arithmeticEncoder, bModel );  // ae(v)
  EncodeUInt32( uint32_t( ppdu.get2DShiftV() ), pfh.getInterPredictPatch2dShiftVBitCountMinus1(),   arithmeticEncoder, bModel );  // ae(v)
  arithmeticEncoder.ExpGolombEncode( o3dgc::IntToUInt( int32_t( ppdu.get2DDeltaSizeU() ) ), 0,  bModel, bModelIntSizeU );  // ae(v)
  arithmeticEncoder.ExpGolombEncode( o3dgc::IntToUInt( int32_t( ppdu.get2DDeltaSizeU() ) ), 0, bModel, bModelIntSizeV );  // ae(v)
  arithmeticEncoder.ExpGolombEncode( o3dgc::IntToUInt( int32_t( ppdu.getPcmPoints() ) ), 0, bModel, bModelPcmPoints );  // ae(v)
}

// 7.3.34 Point local reconstruction syntax
void PCCBitstreamEncoderNewSyntax::pointLocalReconstruction( PointLocalReconstruction& plr,
                                                             PCCContext& context, 
                                                             PCCBitstream& bitstream, 
                                                             o3dgc::Arithmetic_Codec& arithmeticEncoder ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif
  //o3dgc::Static_Bit_Model    bModel;
  o3dgc::Adaptive_Bit_Model  bModelInterpolateFlag, bModelNeighbour, bModelMinDepth, bModelFillingFlag;

  /* 8.4.5 */
  /*blockToPatchMapWidth = Ceil( sps_frm_width / ocp_occupancy_packing_block_size )  */
  /*blockToPatchMapHeight = Ceil( sps_frm_height / ocp_occupancy_packing_block_size )*/
  /* this can be derived, but what happens if sps is updated? */
  /*  uint64_t blockToPatchMapHeight = */
  /*    static_cast<uint64_t>( ceil( double( context.getSps().getFrameHeight  */
  /*                           context.getSps().getOccupancyParameterSet().getOccupancyPackingBlockSize) ) ); */
  /*  uint64_t blockToPatchMapWidth = */
  /*    static_cast<size_t>( ceil( double( context.getSps().getFrameWidth */
  /*                         context.getSps().getOccupancyParameterSet().getOccupancyPackingBlockSize) ) ); */

  /* [VZ: here for flag coding I've used adaptive model, should the static be used instead for ae(v)?] */
  /* note array is row major, the indexing though is (i,j) (x,y) for convenience class methods take care of this*/
  for ( uint64_t j = 0; j < plr.getBlockToPatchMapHeight(); j++ ) {
    for ( uint64_t i = 0; i < plr.getBlockToPatchMapWidth(); i++ ) {
      if ( plr.getBlockToPatchMap( i, j ) >= 0 ) {
        arithmeticEncoder.encode( plr.getModeInterpolateFlag( i, j ), bModelInterpolateFlag );   // ae(v)
        if ( plr.getModeInterpolateFlag( i, j ) )
          arithmeticEncoder.encode( plr.getModeNeighbourMinus1( i, j ), bModelNeighbour );       // ae(v)
        arithmeticEncoder.encode( plr.getModeMinimumDepthMinus1( i, j ), bModelMinDepth );       // ae(v)
        if ( ( plr.getModeMinimumDepthMinus1( i, j ) > 0 ) || 
             ( plr.getModeInterpolateFlag( i, j ) ) )
          arithmeticEncoder.encode( plr.getModeFillingFlag( i, j ), bModelFillingFlag );         // ae(v)
      }
    }
  }
}


/* THIS SECTION ADDS PATCH RELATED CALCULATIONS AND GENERATED PFDS */

void PCCBitstreamEncoderNewSyntax::createPatchFrameDataStructure( PCCContext&   context ) {
  // emulate the compress occupancy map functionality
  // this method operates on patch_sequence_unit_payload level
  size_t numPFDUs = context.getFrames().size();
  auto& sps = context.getSps();
  auto& psdu = context.getPatchSequenceDataUnit(); // perhaps I need to allocate this

  /*I want to have a reflist constructed here, but there is no access for the frame coding order*/
  /* add frame to the refernce list */ 
  RefListStruct refList;
  refList.allocate();
  refList.setNumRefEntries( 1 ); // hardcoded allow only 1 reference frame
  refList.setAbsDeltaPfocSt( 1 , 0 ); //hardcoded: allowed previous farme as reference
  
  PCCFrameContext refFrame  = context.getFrames()[0]; // this process reflects current sowtware reference selection and should be used only as an attempt of the bitexact implenentation.

  // applocate the vector of psup


  /* 1. Create sequence of the patch frames */
  for ( size_t i = 0; i < numPFDUs; i++ ) {
    PCCFrameContext &frame = context.getFrames()[i];
    PatchFrameHeader pfh;
    PatchFrameDataUnit pfdu;
    PatchFrameLayerUnit pflu;
    PatchSequenceUnitPayload psup;

    pflu.setFrameIndex( i );
    

    /* prepare :  allocate the total number of patches of each type */
    /* TODO: create the parent class for the patch, inherit the child: pdu, ppdu, dpdu  */
    /* N.B. it makes more sence to do as follows: pdu is parent : ipdu; ppdu; dpdu; spdu; mpdu are inherited */

    /* iterate inter patches */
    for ( size_t k = 0; k < frame.getNumMatchedPatches(); k++ ) {
      PatchInformationData pid;
      DeltaPatchDataUnit dpdu;
      pfdu.addPatchMode( P_INTER );
      pid.setDeltaPatchDataUnit( dpdu );
      pfdu.addPatchInformationData( pid );
    }
    /* iterate intra patches */
    for ( size_t k = frame.getNumMatchedPatches(); k < frame.getPatches().size(); k++ ) {
      PatchInformationData pid;
      PatchDataUnit pdu;
      pid.setPatchDataUnit( pdu );
      pfdu.setPatchInformationData( k, pid );
      if ( frame.getIndex() == 0 ) {
        pfdu.setPatchFrameMode( k, I_INTRA );
      } else {
        pfdu.setPatchFrameMode( k, P_INTRA );
      }
    }

    /*
      N.B.
      sps.getPcmSeparateVideoPresentFlag is not defined in the CD --> fix    
    */
    if ((sps.getLosslessGeo() || params_.lossyMissedPointsPatch_ ) && !sps.getPcmSeparateVideoPresentFlag()) {
      /* get pfdu structure */
      /* crate ppdu, currently ift is a last patch */
      PatchInformationData pid;
      PCMPatchDataUnit ppdu;

      /* iterate patch list with current fraem order */
      pfh.setPatchFrameOrderCntLsb( i ); // this is not correct, need to add functionaliuty to control index and order

      auto& patches = frame.getPatches();
      auto& missedPointsPatch = frame.getMissedPointsPatch();
      const size_t patchIndex = patches.size();
      patches.resize(patchIndex + 1);
      PCCPatch &dummyPatch = patches[patchIndex];
      dummyPatch.getIndex() = patchIndex;
      dummyPatch.getU0() = missedPointsPatch.u0_;
      dummyPatch.getV0() = missedPointsPatch.v0_;
      dummyPatch.getSizeU0() = missedPointsPatch.sizeU0_;
      dummyPatch.getSizeV0() = missedPointsPatch.sizeV0_;
      dummyPatch.getU1() = 0;
      dummyPatch.getV1() = 0;
      dummyPatch.getD1() = 0;
      dummyPatch.getNormalAxis() = 0;
      dummyPatch.getTangentAxis() = 1;
      dummyPatch.getBitangentAxis() = 2;
      dummyPatch.getOccupancyResolution() = missedPointsPatch.occupancyResolution_;
      dummyPatch.getOccupancy() = missedPointsPatch.occupancy_;
      dummyPatch.getLod() = params_.testLevelOfDetail_;
      dummyPatch.setBestMatchIdx() = -1;
      dummyPatch.getPatchOrientation() = PatchOrientation::DEFAULT;
      createPatchFrameDataStructure( context, frame, refFrame, i);
      patches.pop_back();
      
      ppdu.set2DShiftU( dummyPatch.getU0() );
      ppdu.set2DShiftU( dummyPatch.getV0() );
      ppdu.set2DDeltaSizeU( dummyPatch.getSizeU0() );
      ppdu.set2DDeltaSizeV( dummyPatch.getSizeV0() );

      if ( frame.getIndex() == 0 ) { // get aclual frame type here, now is stub
        pfdu.setPatchFrameMode( patchIndex, I_PCM );
      } else {
        pfdu.setPatchFrameMode( patchIndex, P_PCM );
      }
      pid.setPCMPatchDataUnit( ppdu );
      pfdu.setPatchInformationData(patchIndex, pid);
    } else {
      createPatchFrameDataStructure( context, frame, refFrame, i);
    }
    psup.setPatchFrameLayerUnit(pflu);

    refFrame = frame;
    /* put this frame to the refList structure */

    psdu.setPatchSequenceUnitPayload( i, psup );
  } 
}

void PCCBitstreamEncoderNewSyntax::createPatchFrameDataStructure( PCCContext&      context,
                                                                  PCCFrameContext& frame,
                                                                  PCCFrameContext& refFrame,
                                                                  size_t           frameIndex ) {
  auto&        sps        = context.getSps();
  auto&        ops        = sps.getOccupancyParameterSet();
  auto&        patches    = frame.getPatches();
  const size_t patchCount = patches.size();

  auto&        pfh        = context.getPatchSequenceDataUnit().getPatchSequenceUnitPayload( frameIndex ).getPatchFrameLayerUnit().getPatchFrameHeader();
  auto&        pfdu       = context.getPatchSequenceDataUnit().getPatchSequenceUnitPayload( frameIndex ).getPatchFrameLayerUnit().getPatchFrameDataUnit();
  
  pfdu.setPatchCount(patchCount-1);
  /*
    N.B.
    cuurent sw has no support for CD patch information coding 7.3.29 has only NEW implementation...
  */
  /* 
    N.B.
    maxCandidateCount --> this is not used any longer 
  */

  if ( !sps.getLayerAbsoluteCodingEnabledFlag( 0 ) ) {
    //params_.surfaceThickness_; this is missing in the current CD should be a part of gfps
    pfdu.getPatchInformationData( 0 ).getPatchDataUnit().setProjectionMode(patches[0].getFrameProjectionMode());
  }

  const size_t minLevel = sps.getMinLevel();
  const uint8_t maxBitCountForMinDepth=uint8_t(10-gbitCountSize[minLevel]);
  const uint8_t maxBitCountForMaxDepth=uint8_t(9-gbitCountSize[minLevel]); //20190129
  uint8_t enable_flexible_patch_flag = ( params_.packingStrategy_ > 0);
  uint8_t id = context.getPatchSequenceDataUnit().getPatchSequenceUnitPayload(frameIndex).getPatchFrameLayerUnit().getPatchFrameHeader().getPatchFrameParameterSetId();
  context.getPatchSequenceDataUnit().getPatchFrameParameterSet(id).setPatchOrientationPresentFlag(enable_flexible_patch_flag);

  if ( (frameIndex == 0) || (!sps.getPatchInterPredictionEnabledFlag()) ) {
    size_t maxU0 = 0;
    size_t maxV0 = 0;
    size_t maxU1 = 0;
    size_t maxV1 = 0;
    size_t maxLod = 0;
    for ( size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex ) {
      const auto &patch = patches[patchIndex];
      maxU0 = (std::max)(maxU0, patch.getU0());
      maxV0 = (std::max)(maxV0, patch.getV0());
      maxU1 = (std::max)(maxU1, patch.getU1());
      maxV1 = (std::max)(maxV1, patch.getV1());
      maxLod = (std::max)(maxLod, patch.getLod());
    }

    const uint8_t bitCountU0 = uint8_t( PCCGetNumberOfBitsInFixedLengthRepresentation( uint32_t( maxU0 + 1 ) ) );
    const uint8_t bitCountV0 = uint8_t( PCCGetNumberOfBitsInFixedLengthRepresentation( uint32_t( maxV0 + 1 ) ) );
    const uint8_t bitCountU1 = uint8_t( PCCGetNumberOfBitsInFixedLengthRepresentation( uint32_t( maxU1 + 1 ) ) );
    const uint8_t bitCountV1 = uint8_t( PCCGetNumberOfBitsInFixedLengthRepresentation( uint32_t( maxV1 + 1 ) ) );
    const uint8_t bitCountD1 = maxBitCountForMinDepth;
    const uint8_t bitCountDD = maxBitCountForMaxDepth;
    const uint8_t bitCountLod = uint8_t( PCCGetNumberOfBitsInFixedLengthRepresentation( uint32_t( maxLod + 1 ) ) );

    pfh.setInterPredictPatch2dShiftUBitCountMinus1( bitCountU0 - 1 );
    pfh.setInterPredictPatch2dShiftVBitCountMinus1( bitCountV0 - 1 );
    pfh.setInterPredictPatch3dShiftTangentAxisBitCountMinus1( bitCountU1 - 1 );
    pfh.setInterPredictPatch3dShiftBitangentAxisBitCountMinus1( bitCountV1 - 1 );
    pfh.setInterPredictPatch3dShiftNormalAxisBitCountMinus1( maxBitCountForMinDepth - 1 ); // minDepth
    //pfh.setInterPredictPatch23dShiftNormalAxisBitCountMinus1( maxBitCountForMaxDepth - 1 ); //maxDepth TODO: add this to pfh
    pfh.setInterPredictPatchLodBitCount( bitCountLod ); //AS in CD, not minus1, if equls to 0 no LoD present

    //compress compressMetadata
    int64_t prevSizeU0 = 0;
    int64_t prevSizeV0 = 0;

    for ( size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex ) {
      const auto &patch = patches[patchIndex];
      //patchInformationData
      if ( pfh.getType() == I_PATCH_FRAME ) {
        pfdu.setPatchFrameMode( patchIndex, I_INTRA );
      } else {
        pfdu.setPatchFrameMode( patchIndex, P_INTRA );
      }

      pfdu.getPatchInformationData( patchIndex ).getPatchDataUnit().set2DShiftU(patch.getU0());
      pfdu.getPatchInformationData( patchIndex ).getPatchDataUnit().set2DShiftV(patch.getV0());
      if ( enable_flexible_patch_flag ) {
        bool flexible_patch_present_flag = (patch.getPatchOrientation() != PatchOrientation::DEFAULT);
        if (flexible_patch_present_flag) {
          //pfh_patch_orientation_present_flag[ frmIdx ]  = 1 --> this is missing in CD 7.3.27, but is used in 7.3.31
          pfdu.getPatchInformationData( patchIndex ).getPatchDataUnit().setOrientationSwapFlag( patch.getPatchOrientation() - 1 );
        } else {
          //pfh_patch_orientation_present_flag[ frmIdx ]  = 0 --> this is missing int CD 7.3.27, but is used in 7.3.31
        }
      }
      pfdu.getPatchInformationData( patchIndex ).getPatchDataUnit().set3DShiftTangentAxis(patch.getU1());
      pfdu.getPatchInformationData( patchIndex ).getPatchDataUnit().set3DShiftBiTangentAxis(patch.getV1());

      size_t D1;
      if ( patch.getProjectionMode() == 0 ) {
        D1 = patch.getD1() / minLevel;
      } else {
        D1 = 1024 - patch.getD1();
        D1 = D1 / minLevel;
      }
      pfdu.getPatchInformationData( patchIndex ).getPatchDataUnit().set3DShiftNormalAxis( D1 );
      size_t quantDD = patch.getSizeD() == 0 ? 0 : ((patch.getSizeD() - 1) / minLevel + 1);

      auto &patchTemp = patches[patchIndex];
      PCCMetadata& metadata=patchTemp.getPatchLevelMetadata();
      metadata.setbitCountQDepth(bitCountDD);
#ifdef CE210_MAXDEPTH_EVALUATION
      metadata.setQMaxDepthInPatch(int64_t(quantDD));
#endif
      metadata.setIndex(patchIndex);
      metadata.setMetadataType(METADATA_PATCH);
      metadata.setMetadataPresent(true);

      pfdu.getPatchInformationData( patchIndex ).getPatchDataUnit().setLod(patch.getLod());
      
      if ( !sps.getLayerAbsoluteCodingEnabledFlag( 0 ) && (pfdu.getPatchInformationData( patchIndex ).getPatchDataUnit().getProjectionMode() == 2) ) {
        const uint8_t bitCountProjDir = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(2 + 1)));
      } // this is later usd with entropy coder, should we add this to PFH?
      if ( sps.getLayerAbsoluteCodingEnabledFlag( 0 ) ) {
        if ( patch.getProjectionMode() == 0 ) {
          pfdu.getPatchInformationData( patchIndex ).getPatchDataUnit().setProjectionMode( 0 );
        } else {
          pfdu.getPatchInformationData( patchIndex ).getPatchDataUnit().setProjectionMode( 1 );
        }
      }

      const int64_t deltaSizeU0 = static_cast<int64_t>(patch.getSizeU0()) - prevSizeU0;
      const int64_t deltaSizeV0 = static_cast<int64_t>(patch.getSizeV0()) - prevSizeV0;
      pfdu.getPatchInformationData( patchIndex ).getPatchDataUnit().set2DDeltaSizeU(deltaSizeU0);
      pfdu.getPatchInformationData( patchIndex ).getPatchDataUnit().set2DDeltaSizeV(deltaSizeV0);
      prevSizeU0 = patch.getSizeU0();
      prevSizeV0 = patch.getSizeV0();

      /*
      compressMetadata( patch.getPatchLevelMetadata(), arithmeticEncoder, bModel0, bModelDD );
      compressOneLayerData( context, frame, patch, arithmeticEncoder,
                            occupiedModel, interpolateModel, neighborModel, minD1Model, fillingModel );
      */
      
      // compressMetadata     --> mapping the data structures 7.3.17, 7.3.18
      GeometryFrameParams gfp;
      auto &metadataEnabingFlags = metadata.getMetadataEnabledFlags();

      if ( metadata.getMetadataPresent() ) {
        if ( metadataEnabingFlags.getScaleEnabled() ) {
          gfp.setGeometryScaleOnAxis( 0, metadata.getScale()[0] );
          gfp.setGeometryScaleOnAxis( 1, metadata.getScale()[1] );
          gfp.setGeometryScaleOnAxis( 2, metadata.getScale()[2] );
        }
        if ( metadataEnabingFlags.getOffsetEnabled() ) {
          gfp.setGeometryOffsetOnAxis( 0, metadata.getOffset()[0] );
          gfp.setGeometryOffsetOnAxis( 1, metadata.getOffset()[1] );
          gfp.setGeometryOffsetOnAxis( 2, metadata.getOffset()[2] );
        }
        if ( metadataEnabingFlags.getRotationEnabled() ) {
          gfp.setGeometryRotationOnAxis( 0, metadata.getRotation()[0] );
          gfp.setGeometryRotationOnAxis( 1, metadata.getRotation()[1] );
          gfp.setGeometryRotationOnAxis( 2, metadata.getRotation()[2] );
        }
        if ( metadataEnabingFlags.getPointSizeEnabled() ) {
          gfp.setGeometryPointShapeInfoPresentFlag(metadata.getPointSizePresent());
          if ( gfp.getGeometryPointShapeInfoPresentFlag() ) {
            gfp.setGeometryPointSizeInfo(metadata.getPointSize());
          }
        }
        if ( metadataEnabingFlags.getPointShapeEnabled() ) {
          gfp.setGeometryPointShapeInfoPresentFlag(metadata.getPointShapePresent());
          if ( gfp.getGeometryPointShapeInfoPresentFlag() ) {
            gfp.setGeometryPointSizeInfo( metadata.getPointShape() );
          }
        }
#ifdef CE210_MAXDEPTH_EVALUATION
        if ( metadataEnabingFlags.getMaxDepthEnabled() ) {
          currentDD = metadata.getQMaxDepthInPatch();
          //TODO: gfp.setGeometryMaxDeltaDepth(currentDD);
        }
#endif
      }
      auto &lowerLevelMetadataEnabledFlags = metadata.getLowerLevelMetadataEnabledFlags();
      // gfp.set...(lowerLevelMetadataEnabledFlags.getMetadataEnabled())
      if ( lowerLevelMetadataEnabledFlags.getMetadataEnabled() ) {
        /* TODO: I could not locate the corespondence for these parameters*/
        // lowerLevelMetadataEnabledFlags.getScaleEnabled()
        // lowerLevelMetadataEnabledFlags.getOffsetEnabled()
        // lowerLevelMetadataEnabledFlags.getRotationEnabled()
        // lowerLevelMetadataEnabledFlags.getPointSizeEnabled()
        // lowerLevelMetadataEnabledFlags.getPointShapeEnabled()

      }

      // compressOneLayerData --> PointLocalReconstruction 7.3.34
      if ( sps.getLayerAbsoluteCodingEnabledFlag( 0 ) && !sps.getMultipleLayerStreamsPresentFlag() ) {
        PointLocalReconstruction plr;
        auto&        blockToPatch = frame.getBlockToPatch();
        const size_t blockToPatchWidth = frame.getWidth() / ops.getOccupancyPackingBlockSize();
        const size_t blockToPatchHeight = frame.getHeight() / ops.getOccupancyPackingBlockSize();
        auto&        interpolateMap = frame.getInterpolate();
        auto&        fillingMap = frame.getFilling();
        auto&        minD1Map = frame.getMinD1();
        auto&        neighborMap = frame.getNeighbor();

        plr.setBlockToPatchMapWidth(blockToPatchWidth);
        plr.setBlockToPatchMapHeight(blockToPatchHeight);


        for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
          for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
            int pos = patch.patchBlock2CanvasBlock((u0), (v0), blockToPatchWidth, blockToPatchHeight);
            plr.setBlockToPatchMap( static_cast<uint32_t>( blockToPatch[pos] > 0 ), u0, v0 );
            if( blockToPatch[ pos ] > 0 ) {
              plr.setModeInterpolateFlag( bool( interpolateMap[pos] ), u0, v0 );
              if( interpolateMap[ pos ] > 0 ) {
                uint32_t code = static_cast<uint32_t>( int(neighborMap[ pos ]) - 1 );
                plr.setModeNeighbourMinus1( code, u0, v0 );
              }
              uint8_t code =static_cast<uint8_t>( minD1Map[pos] ) - 1;
              plr.setModeMinimumDepthMinus1( code, u0, v0 );
              if( minD1Map[ pos ] > 1 || interpolateMap[ pos ] > 0 ) {
                plr.setModeFillingFlag(static_cast<uint32_t>(fillingMap[ pos ] ), u0, v0);
              }
            }
          }
        }
      }
    }
  } else {

    auto&  refPatches = refFrame.getPatches();
    // ref frame should go to ref structure 7.3.28
    // currently only 1 frame shall be supported

    size_t numMatchedPatches = frame.getNumMatchedPatches();
    size_t TopNmaxU0 = 0, maxU0 = 0;
    size_t TopNmaxV0 = 0, maxV0 = 0;
    size_t TopNmaxU1 = 0, maxU1 = 0;
    size_t TopNmaxV1 = 0, maxV1 = 0;
    size_t TopNmaxD1 = 0, maxD1 = 0;
    size_t TopNmaxDD = 0, maxDD = 0;
    const size_t minLevel= sps.getMinLevel();
    const uint8_t maxBitCountForMinDepth=uint8_t(10-gbitCountSize[minLevel]);
    uint8_t maxAllowedDepthP1 = params_.maxAllowedDepth_+1;
    uint8_t bitCountDDMax=0; //255
    while((maxAllowedDepthP1)>>bitCountDDMax) bitCountDDMax++;
    const uint8_t maxBitCountForMaxDepth=uint8_t(9-gbitCountSize[minLevel]); //20190129

                                                                             //get the maximum u0,v0,u1,v1 and d1.
    for (size_t patchIndex = 0; patchIndex < numMatchedPatches; ++patchIndex) {
      const auto &patch = patches[patchIndex];
      TopNmaxU0 = (std::max)(TopNmaxU0, patch.getU0());
      TopNmaxV0 = (std::max)(TopNmaxV0, patch.getV0());
      TopNmaxU1 = (std::max)(TopNmaxU1, patch.getU1());
      TopNmaxV1 = (std::max)(TopNmaxV1, patch.getV1());
      size_t D1 = patch.getD1()/minLevel;
      TopNmaxD1 = (std::max)(TopNmaxD1, D1);
      size_t DD = patch.getSizeD()/minLevel;
      TopNmaxDD = (std::max)(TopNmaxDD, DD);
    }
    for (size_t patchIndex = numMatchedPatches; patchIndex < patchCount; ++patchIndex) {
      const auto &patch = patches[patchIndex];
      maxU0 = (std::max)(maxU0, patch.getU0());
      maxV0 = (std::max)(maxV0, patch.getV0());
      maxU1 = (std::max)(maxU1, patch.getU1());
      maxV1 = (std::max)(maxV1, patch.getV1());
      size_t D1 = patch.getD1()/minLevel;
      maxD1 = (std::max)(maxD1, D1);
      size_t DD = patch.getSizeD()/minLevel;
      maxDD = (std::max)(maxDD, DD);
    }
    //  compressMetadata(frame.getFrameLevelMetadata(), arithmeticEncoder);
    const uint8_t bitCountNumPatches =
      uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(patchCount)));//numMatchedPatches <= patchCount
    
    uint8_t flag = 0;
    uint8_t F = 1;  //true if the maximum value comes from the latter part.
    uint8_t A[4] = { 1, 1, 1, 1 };

    uint8_t bitCount[5], topBitCount[5];
    bitCount   [0] = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxU0     + 1)));
    topBitCount[0] = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(TopNmaxU0 + 1)));
    bitCount   [1] = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxV0     + 1)));
    topBitCount[1] = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(TopNmaxV0 + 1)));
    bitCount   [2] = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxU1     + 1)));
    topBitCount[2] = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(TopNmaxU1 + 1)));
    bitCount   [3] = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxV1     + 1)));
    topBitCount[3] = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(TopNmaxV1 + 1)));
    bitCount   [4] = maxBitCountForMinDepth;

    for (int i = 0; i < 4; i++) {
      if (bitCount[i] <= topBitCount[i]) {
        bitCount[i] = topBitCount[i];
        A[i] = 0;
      }
      flag = flag << 1;
      flag += A[i];
    }
    //Generate F and A.
    if (flag == 0) { F = 0; }
    pfh.setInterPredictPatchBitCountFlag( F );
    if ( F ) {
      pfh.setInterPredictPatch2dShiftUBitCountFlag( A[0] );
      pfh.setInterPredictPatch2dShiftVBitCountFlag( A[1] );
      pfh.setInterPredictPatch3dShiftTangentAxisBitCountFlag( A[2] );
      pfh.setInterPredictPatch3dShiftBitangentAxisBitCountFlag( A[3] );
      pfh.setInterPredictPatchLodBitCountFlag( A[4] );
      if ( A[0] ) { pfh.setInterPredictPatch2dShiftUBitCountMinus1( bitCount[0] ); }
      if ( A[1] ) { pfh.setInterPredictPatch2dShiftVBitCountMinus1( bitCount[1] ); }
      if ( A[2] ) { pfh.setInterPredictPatch3dShiftTangentAxisBitCountMinus1( bitCount[2] ); }
      if ( A[3] ) { pfh.setInterPredictPatch3dShiftBitangentAxisBitCountMinus1( bitCount[3] ); }
      if ( A[4] ) { pfh.setInterPredictPatchLodBitCount( bitCount[4] ); }
    }

    int64_t predIndex = 0;
    for ( size_t patchIndex = 0; patchIndex < numMatchedPatches; ++patchIndex ) {
      const auto &patch    = patches[patchIndex];
      const auto &refPatch = refPatches[patch.getBestMatchIdx()];
      auto &curDpdu = pfdu.getPatchInformationData( patchIndex ).getDeltaPatchDataUnit();
      pfdu.setPatchFrameMode( patchIndex, P_INTER );
       
      const int64_t delta_index = patch.getBestMatchIdx() - predIndex;
      predIndex += (delta_index + 1);
      const int64_t delta_u0 = static_cast<int64_t>(patch.getU0() - refPatch.getU0());
      const int64_t delta_v0 = static_cast<int64_t>(patch.getV0() - refPatch.getV0());
      const int64_t delta_u1 = static_cast<int64_t>(patch.getU1() - refPatch.getU1());
      const int64_t delta_v1 = static_cast<int64_t>(patch.getV1() - refPatch.getV1());
      size_t currentD1=patch.getD1();
      size_t prevD1=refPatch.getD1();
      if(patch.getProjectionMode()==0) {
        currentD1=currentD1/minLevel;
        prevD1=prevD1/minLevel;
      } else {
        currentD1=(1024-currentD1)/minLevel;
        prevD1=(1024-prevD1)/minLevel;
      }
      const int64_t delta_d1 = static_cast<int64_t>(currentD1 - prevD1);
      const int64_t deltaSizeU0 = static_cast<int64_t>(patch.getSizeU0() - refPatch.getSizeU0());
      const int64_t deltaSizeV0 = static_cast<int64_t>(patch.getSizeV0() - refPatch.getSizeV0());

      curDpdu.set2DDeltaShiftU( delta_u0 );
      curDpdu.set2DDeltaShiftV( delta_v0 );
      curDpdu.set3DDeltaShiftTangentAxis( delta_u1 );
      curDpdu.set3DDeltaShiftTangentAxis( delta_v1 );
      curDpdu.set3DDeltaShiftNormalAxis( delta_d1 );
      curDpdu.set2DDeltaSizeU(deltaSizeU0);
      curDpdu.set2DDeltaSizeV(deltaSizeV0);
      
      size_t quantDD=patch.getSizeD()==0?0:((patch.getSizeD()-1)/minLevel +1);
      size_t prevQDD=patch.getSizeD()==0?0:((refPatch.getSizeD()-1)/minLevel +1);

      const int64_t delta_dd = static_cast<int64_t>(quantDD - prevQDD);
      auto &patchTemp = patches[patchIndex];
      PCCMetadata& metadata=patchTemp.getPatchLevelMetadata();
      metadata.setMetadataPresent(true);
      metadata.setbitCountQDepth(0);
      //metadata.setIndex(patchIndex);
      //compressMetadata(metadata, arithmeticEncoder, bModel0, bModelDD);
    }

    int64_t prevSizeU0 = patches[numMatchedPatches-1].getSizeU0();
    int64_t prevSizeV0 = patches[numMatchedPatches-1].getSizeV0();

    for (size_t patchIndex = numMatchedPatches; patchIndex < patchCount; ++patchIndex) {
      const auto &patch = patches[patchIndex];
      auto &curPdu = pfdu.getPatchInformationData( patchIndex ).getPatchDataUnit();

      curPdu.set2DShiftU(patch.getU0());
      curPdu.set2DShiftV(patch.getV0());

      if ( enable_flexible_patch_flag ) {
        bool flexible_patch_present_flag = (patch.getPatchOrientation() != PatchOrientation::DEFAULT);
        if (flexible_patch_present_flag) {
          //pfh_patch_orientation_present_flag[ frmIdx ]  = 1 --> this is missing in CD 7.3.27, but is used in 7.3.31
          pfdu.getPatchInformationData( patchIndex ).getPatchDataUnit().setOrientationSwapFlag( patch.getPatchOrientation() - 1 );
        } else {
          //pfh_patch_orientation_present_flag[ frmIdx ]  = 0 --> this is missing int CD 7.3.27, but is used in 7.3.31
        }
      }
      curPdu.set3DShiftTangentAxis(patch.getU1());
      curPdu.set3DShiftBiTangentAxis(patch.getV1());

      size_t currentD1=patch.getD1();
      if(patch.getProjectionMode()==0) {
        currentD1=currentD1/minLevel;
      } else {
        currentD1=(1024-currentD1)/minLevel;
      }
      curPdu.set3DShiftNormalAxis( currentD1 );

      size_t quantDD = patch.getSizeD() == 0 ? 0 : ((patch.getSizeD() - 1) / minLevel + 1);
      auto &patchTemp = patches[patchIndex];
      PCCMetadata& metadata=patchTemp.getPatchLevelMetadata();
      metadata.setbitCountQDepth(maxBitCountForMaxDepth);
#ifdef CE210_MAXDEPTH_EVALUATION
      metadata.setQMaxDepthInPatch(int64_t(quantDD));
#endif
      metadata.setIndex(patchIndex);
      metadata.setMetadataPresent(true);

      const int64_t deltaSizeU0 = static_cast<int64_t>(patch.getSizeU0()) - prevSizeU0;
      const int64_t deltaSizeV0 = static_cast<int64_t>(patch.getSizeV0()) - prevSizeV0;

      curPdu.set2DDeltaSizeU(deltaSizeU0);
      curPdu.set2DDeltaSizeU(deltaSizeV0);
      curPdu.setNormalAxis(static_cast<pcc::PCCAxis3>(patch.getNormalAxis()));

      //compressMetadata(patch.getPatchLevelMetadata(), arithmeticEncoder,bModel0, bModelDD);
      GeometryFrameParams gfp;
      auto &metadataEnabingFlags = metadata.getMetadataEnabledFlags();

      if ( metadata.getMetadataPresent() ) {
        if ( metadataEnabingFlags.getScaleEnabled() ) {
          gfp.setGeometryScaleOnAxis( 0, metadata.getScale()[0] );
          gfp.setGeometryScaleOnAxis( 1, metadata.getScale()[1] );
          gfp.setGeometryScaleOnAxis( 2, metadata.getScale()[2] );
        }
        if ( metadataEnabingFlags.getOffsetEnabled() ) {
          gfp.setGeometryOffsetOnAxis( 0, metadata.getOffset()[0] );
          gfp.setGeometryOffsetOnAxis( 1, metadata.getOffset()[1] );
          gfp.setGeometryOffsetOnAxis( 2, metadata.getOffset()[2] );
        }
        if ( metadataEnabingFlags.getRotationEnabled() ) {
          gfp.setGeometryRotationOnAxis( 0, metadata.getRotation()[0] );
          gfp.setGeometryRotationOnAxis( 1, metadata.getRotation()[1] );
          gfp.setGeometryRotationOnAxis( 2, metadata.getRotation()[2] );
        }
        if ( metadataEnabingFlags.getPointSizeEnabled() ) {
          gfp.setGeometryPointShapeInfoPresentFlag(metadata.getPointSizePresent());
          if ( gfp.getGeometryPointShapeInfoPresentFlag() ) {
            gfp.setGeometryPointSizeInfo(metadata.getPointSize());
          }
        }
        if ( metadataEnabingFlags.getPointShapeEnabled() ) {
          gfp.setGeometryPointShapeInfoPresentFlag(metadata.getPointShapePresent());
          if ( gfp.getGeometryPointShapeInfoPresentFlag() ) {
            gfp.setGeometryPointSizeInfo( metadata.getPointShape() );
          }
        }
#ifdef CE210_MAXDEPTH_EVALUATION
        if ( metadataEnabingFlags.getMaxDepthEnabled() ) {
          currentDD = metadata.getQMaxDepthInPatch();
          //TODO: gfp.setGeometryMaxDeltaDepth(currentDD);
        }
#endif
      }
      auto &lowerLevelMetadataEnabledFlags = metadata.getLowerLevelMetadataEnabledFlags();
      // gfp.set...(lowerLevelMetadataEnabledFlags.getMetadataEnabled())
      if ( lowerLevelMetadataEnabledFlags.getMetadataEnabled() ) {
        /* TODO: I could not locate the corespondence for these parameters*/
        // lowerLevelMetadataEnabledFlags.getScaleEnabled()
        // lowerLevelMetadataEnabledFlags.getOffsetEnabled()
        // lowerLevelMetadataEnabledFlags.getRotationEnabled()
        // lowerLevelMetadataEnabledFlags.getPointSizeEnabled()
        // lowerLevelMetadataEnabledFlags.getPointShapeEnabled()
      }
    }

    for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
      const auto &patch    = patches[patchIndex];

      if ( sps.getLayerAbsoluteCodingEnabledFlag( 0 ) && !sps.getMultipleLayerStreamsPresentFlag() ) {
        PointLocalReconstruction plr;
        auto&        blockToPatch = frame.getBlockToPatch();
        const size_t blockToPatchWidth = frame.getWidth() / ops.getOccupancyPackingBlockSize();
        const size_t blockToPatchHeight = frame.getHeight() / ops.getOccupancyPackingBlockSize();
        auto&        interpolateMap = frame.getInterpolate();
        auto&        fillingMap = frame.getFilling();
        auto&        minD1Map = frame.getMinD1();
        auto&        neighborMap = frame.getNeighbor();

        plr.setBlockToPatchMapWidth(blockToPatchWidth);
        plr.setBlockToPatchMapHeight(blockToPatchHeight);


        for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
          for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
            int pos = patch.patchBlock2CanvasBlock((u0), (v0), blockToPatchWidth, blockToPatchHeight);
            plr.setBlockToPatchMap( static_cast<uint32_t>( blockToPatch[pos] > 0 ), u0, v0 );
            if( blockToPatch[ pos ] > 0 ) {
              plr.setModeInterpolateFlag( bool( interpolateMap[pos] ), u0, v0 );
              if( interpolateMap[ pos ] > 0 ) {
                uint32_t code = static_cast<uint32_t>( int(neighborMap[ pos ]) - 1 );
                plr.setModeNeighbourMinus1( code, u0, v0 );
              }
              uint8_t code =static_cast<uint8_t>( minD1Map[pos] ) - 1;
              plr.setModeMinimumDepthMinus1( code, u0, v0 );
              if( minD1Map[ pos ] > 1 || interpolateMap[ pos ] > 0 ) {
                plr.setModeFillingFlag(static_cast<uint32_t>(fillingMap[ pos ] ), u0, v0);
              }
            }
          }
        }
      }

      //compressOneLayerData( context, frame, patch, arithmeticEncoder,
      //                    occupiedModel, interpolateModel, neighborModel, minD1Model, fillingModel );
    } //end of matched patches
  }
}
