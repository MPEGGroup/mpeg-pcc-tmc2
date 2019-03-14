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
#include "PCCVideoBitstream.h"
#include "PCCBitstream.h"
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCPatch.h"

#include "PCCBitstreamDecoderNewSyntax.h"

using namespace pcc;

PCCBitstreamDecoderNewSyntax::PCCBitstreamDecoderNewSyntax() {}
PCCBitstreamDecoderNewSyntax::~PCCBitstreamDecoderNewSyntax() {}

// jkei[?] are we going to change it while(...) style later?
// jkei[?] why do we need a variable vpccUntType here''?
int PCCBitstreamDecoderNewSyntax::decode( PCCContext& context, PCCBitstream& bitstream ) {
  VPCCUnitType vpccUnitType;
  vpccUnit( context, bitstream, vpccUnitType );  // VPCC_SPS
  vpccUnit( context, bitstream, vpccUnitType );  // VPCC_PSD
  vpccUnit( context, bitstream, vpccUnitType );  // VPCC_OVD
  vpccUnit( context, bitstream, vpccUnitType );  // VPCC_GVD
  vpccUnit( context, bitstream, vpccUnitType );  // VPCC_AVD
  return 0;
}

void PCCBitstreamDecoderNewSyntax::vpccVideoDataUnit( PCCContext&   context,
                                                      PCCBitstream& bitstream,
                                                      VPCCUnitType& vpccUnitType ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  if ( vpccUnitType == VPCC_OVD ) {
    bitstream.read( context.getVideoBitstream( PCCVideoType::OccupancyMap ) );
  } else if ( vpccUnitType == VPCC_GVD ) {
    if ( !context.getAbsoluteD1() ) {
      bitstream.read( context.getVideoBitstream( PCCVideoType::GeometryD0 ) );
      bitstream.read( context.getVideoBitstream( PCCVideoType::GeometryD1 ) );
    } else {
      bitstream.read( context.getVideoBitstream( PCCVideoType::Geometry ) );
    }
    if ( context.getUseAdditionalPointsPatch() &&
         context.getSps().getPcmSeparateVideoPresentFlag() ) {
      bitstream.read( context.getVideoBitstream( PCCVideoType::GeometryMP ) );
    }
  } else if ( vpccUnitType == VPCC_AVD ) {
    if ( !context.getNoAttributes() ) {
      bitstream.read( context.getVideoBitstream( PCCVideoType::Texture ) );
      if ( context.getUseAdditionalPointsPatch() &&
           context.getSps().getPcmSeparateVideoPresentFlag() ) {
        bitstream.read( context.getVideoBitstream( PCCVideoType::TextureMP ) );
      }
    }
  }
}

uint32_t PCCBitstreamDecoderNewSyntax::DecodeUInt32( const uint32_t           bitCount,
                                                     o3dgc::Arithmetic_Codec& arithmeticDecoder,
                                                     o3dgc::Static_Bit_Model& bModel0 ) {
  uint32_t decodedValue = 0;
  for ( uint32_t i = 0; i < bitCount; ++i ) {
    decodedValue += ( arithmeticDecoder.decode( bModel0 ) << i );
  }
  return PCCFromLittleEndian<uint32_t>( decodedValue );
}

// 7.3.2 V-PCC unit syntax
void PCCBitstreamDecoderNewSyntax::vpccUnit( PCCContext&   context,
                                             PCCBitstream& bitstream,
                                             VPCCUnitType& vpccUnitType ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  // jkei[?]would vpccUnitType=vpccUnitHeader( context, bitstream) be more straightfoward?? but
  // current one looks prettier...
  vpccUnitHeader( context, bitstream, vpccUnitType );   // jkei[!] vpccUnitType : output
  vpccUnitPayload( context, bitstream, vpccUnitType );  // jkei[!] vpccUnitType : input
}

// 7.3.3 V-PCC unit header syntax
void PCCBitstreamDecoderNewSyntax::vpccUnitHeader( PCCContext&   context,
                                                   PCCBitstream& bitstream,
                                                   VPCCUnitType& vpccUnitType ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  auto& vpcc   = context.getVPCC();
  auto& sps    = context.getSps();
  vpccUnitType = (VPCCUnitType)bitstream.read( 5 );  // u(5)
  if ( vpccUnitType == VPCC_AVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_OVD ||
       vpccUnitType == VPCC_PSD ) {
    vpcc.setSequenceParameterSetId( bitstream.read( 4 ) );  // u(4)
  }
  if ( vpccUnitType == VPCC_AVD ) {
    vpcc.setAttributeIndex( bitstream.read( 7 ) );  // u(7)
    if ( sps.getMultipleLayerStreamsPresentFlag() ) {
      vpcc.setLayerIndex( bitstream.read( 4 ) );  // u(4)
      pcmSeparateVideoData( context, bitstream, 11 );
    } else {
      pcmSeparateVideoData( context, bitstream, 15 );
    }
  } else if ( vpccUnitType == VPCC_GVD ) {
    if ( sps.getMultipleLayerStreamsPresentFlag() ) {
      vpcc.setLayerIndex( bitstream.read( 4 ) );  // u(4)
      pcmSeparateVideoData( context, bitstream, 18 );
    } else {
      pcmSeparateVideoData( context, bitstream, 22 );
    }
  } else if ( vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_PSD ) {
    bitstream.read( 23 );  // u(23)
  } else {
    bitstream.read( 27 );  // u(27)
  }
}

// 7.3.4 PCM separate video data syntax
void PCCBitstreamDecoderNewSyntax::pcmSeparateVideoData( PCCContext&   context,
                                                         PCCBitstream& bitstream,
                                                         uint8_t       bitCount ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  auto& vpcc = context.getVPCC();
  auto& sps  = context.getSps();
  if ( sps.getPcmSeparateVideoPresentFlag() && !vpcc.getLayerIndex() ) {
    vpcc.setPCMVideoFlag( bitstream.read( 1 ) );  // u(1)
    bitstream.read( bitCount );                   // u(bitCount)
  } else {
    bitstream.read( bitCount + 1 );  // u(bitCount + 1)
  }
}

// 7.3.5 V-PCC unit payload syntax
void PCCBitstreamDecoderNewSyntax::vpccUnitPayload( PCCContext&   context,
                                                    PCCBitstream& bitstream,
                                                    VPCCUnitType& vpccUnitType ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  auto& sps  = context.getSps();  // jkei[!] just for consistancy
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
void PCCBitstreamDecoderNewSyntax::sequenceParameterSet( SequenceParameterSet& sps,
                                                         PCCBitstream&         bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  profileTierLevel( sps.getProfileTierLevel(), bitstream );

  sps.setSequenceParameterSetId( bitstream.read( 4 ) );   // u(4)
  sps.setFrameWidth( bitstream.read( 16 ) );              // u(16)
  sps.setFrameHeight( bitstream.read( 16 ) );             // u(16)
  sps.setAvgFrameRatePresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( sps.getAvgFrameRatePresentFlag() ) {
    sps.setAvgFrameRate( bitstream.read( 16 ) );  // u(16)
  }
  sps.setEnhancedOccupancyMapForDepthFlag( bitstream.read( 1 ) );  // u(1)
  sps.setLayerCountMinus1( bitstream.read( 4 ) );                  // u(4)
  int32_t layerCountMinus1 = (int32_t)sps.getLayerCountMinus1();
  if ( layerCountMinus1 > 0 ) {
    sps.setMultipleLayerStreamsPresentFlag( bitstream.read( 1 ) );  // u(1)
  }
  auto& layerAbsoluteCodingEnabledFlag = sps.getLayerAbsoluteCodingEnabledFlag();
  auto& layerPredictorIndexDiff        = sps.getLayerPredictorIndexDiff();
  for ( size_t i = 0; i < layerCountMinus1; i++ ) {
    layerAbsoluteCodingEnabledFlag[i + 1] = bitstream.read( 1 );  // u(1)
    if ( ( layerAbsoluteCodingEnabledFlag[i + 1] == 0 ) ) {
      if ( i > 0 ) {
        layerPredictorIndexDiff[i + 1] = bitstream.readUvlc();  // ue(v)
      } else {
        layerPredictorIndexDiff[i + 1] = 0;
      }
    }
  }
  sps.setPcmPatchEnabledFlag( bitstream.read( 1 ) );  // u(1)
  if ( sps.getPcmPatchEnabledFlag() ) {
    sps.setPcmSeparateVideoPresentFlag( bitstream.read( 1 ) );  // u(1)
  }
  occupancyParameterSet( sps.getOccupancyParameterSet(), bitstream );
  geometryParameterSet( sps.getGeometryParameterSet(), sps, bitstream );
  sps.setAttributeCount( bitstream.read( 16 ) );  // u(16)

  sps.allocateAttributeParameterSets();
  for ( size_t i = 0; i < sps.getAttributeCount(); i++ ) {
    attributeParameterSet( sps.getAttributeParameterSet( i ), sps, bitstream );
  }
  sps.setPatchSequenceOrientationEnabledFlag( bitstream.read( 1 ) );  // u(1)
  sps.setPatchInterPredictionEnabledFlag( bitstream.read( 1 ) );      // u(1)
  sps.setPixelDeinterleavingFlag( bitstream.read( 1 ) );              // u(1)
  sps.setPointLocalReconstructionEnabledFlag( bitstream.read( 1 ) );  // u(1)
  sps.setRemoveDuplicatePointEnabledFlag( bitstream.read( 1 ) );      // u(1)
  byteAlignment( bitstream );
}

// 7.3.7 Byte alignment syntax
void PCCBitstreamDecoderNewSyntax::byteAlignment( PCCBitstream& bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  bitstream.read( 1 );  // f(1): equal to 1
  while ( !bitstream.byteAligned() ) {
    bitstream.read( 1 );  // f(1): equal to 0
  }
}

// 7.3.8 Profile, tier, and level syntax
void PCCBitstreamDecoderNewSyntax::profileTierLevel( ProfileTierLevel& ptl,
                                                     PCCBitstream&     bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  ptl.setTierFlag( bitstream.read( 1 ) );    // u(1)
  ptl.setProfileIdc( bitstream.read( 1 ) );  // u(7)
  bitstream.read( 48 );                      // u(48)
  ptl.setLevelIdc( bitstream.read( 8 ) );    // u(8)
}

// 7.3.9 Occupancy parameter set syntax
void PCCBitstreamDecoderNewSyntax::occupancyParameterSet( OccupancyParameterSet& ops,
                                                          PCCBitstream&          bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  ops.setOccupancyCodecId( bitstream.read( 8 ) );           // u(8)
  ops.setOccupancyPackingBlockSize( bitstream.read( 8 ) );  // u(8)
}

// 7.3.10 Geometry parameter set syntax
void PCCBitstreamDecoderNewSyntax::geometryParameterSet( GeometryParameterSet& gps,
                                                         SequenceParameterSet& sps,
                                                         PCCBitstream&         bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  gps.setGeometryCodecId( bitstream.read( 8 ) );  // u(8)
  gps.setGeometryNominal2dBitdepthMinus1( bitstream.read( 5 ) );      // u(5)
  gps.setGeometry3dCoordinatesBitdepthMinus1( bitstream.read( 5 ) );  // u(5)
  if ( sps.getPcmSeparateVideoPresentFlag() ) {
    gps.setPcmGeometryCodecId( bitstream.read( 1 ) );  // u(8)
  }
  gps.setGeometryParamsEnabledFlag( bitstream.read( 1 ) );  // u(1)
  if ( gps.getGeometryParamsEnabledFlag() ) {
    geometrySequenceParams( gps.getGeometrySequenceParams(), bitstream );
  }
  
  gps.setGeometryPatchParamsEnabledFlag( bitstream.read( 1 ) );  // u(1)
  if ( gps.getGeometryPatchParamsEnabledFlag() ) {
    gps.setGeometryPatchScaleParamsEnabledFlag( bitstream.read( 1 ) );     // u(1)
    gps.setGeometryPatchOffsetParamsEnabledFlag( bitstream.read( 1 ) );    // u(1)
    gps.setGeometryPatchRotationParamsEnabledFlag( bitstream.read( 1 ) );  // u(1)
    gps.setGeometryPatchPointSizeInfoEnabledFlag( bitstream.read( 1 ) );   // u(1)
    gps.setGeometryPatchPointShapeInfoEnabledFlag( bitstream.read( 1 ) );  // u(1)
  }
}

// 7.3.11 Geometry sequence Params syntax
void PCCBitstreamDecoderNewSyntax::geometrySequenceParams( GeometrySequenceParams& gsp,
                                                           PCCBitstream&           bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  gsp.setGeometrySmoothingParamsPresentFlag( bitstream.read( 1 ) );  // u(1)
  gsp.setGeometryScaleParamsPresentFlag( bitstream.read( 1 ) );      // u(1)
  gsp.setGeometryOffsetParamsPresentFlag( bitstream.read( 1 ) );     // u(1)
  gsp.setGeometryRotationParamsPresentFlag( bitstream.read( 1 ) );   // u(1)
  gsp.setGeometryPointSizeInfoPresentFlag( bitstream.read( 1 ) );    // u(1)
  gsp.setGeometryPointShapeInfoPresentFlag( bitstream.read( 1 ) );   // u(1)
  if ( gsp.getGeometrySmoothingParamsPresentFlag() ) {
    gsp.setGeometrySmoothingEnabledFlag( bitstream.read( 1 ) );  // u(8)
    if ( gsp.getGeometrySmoothingEnabledFlag() ) {
      gsp.setGeometrySmoothingGridSize( bitstream.read( 8 ) );   // u(8)
      gsp.setGeometrySmoothingThreshold( bitstream.read( 8 ) );  // u(8)
    }
  }
  if ( gsp.getGeometryScaleParamsPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      gsp.setGeometryScaleOnAxis( d, bitstream.read( 32 ) );  // u(32)
    }
    if ( gsp.getGeometryOffsetParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        gsp.setGeometryOffsetOnAxis( d, convertToInt( bitstream.read( 32 ) ) );  // i(32)
      }
    }
    if ( gsp.getGeometryRotationParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        gsp.setGeometryRotationOnAxis( d, convertToInt( bitstream.read( 32 ) ) );  // i(32)
      }
    }
    if ( gsp.getGeometryPointSizeInfoPresentFlag() ) {
      gsp.setGeometryPointSizeInfo( bitstream.read( 8 ) );  // u(8)
    }
    if ( gsp.getGeometryPointShapeInfoPresentFlag() ) {
      gsp.setGeometryPointShapeInfo( bitstream.read( 8 ) );  // u(8)
    }
  }
}

// 7.3.12 Attribute parameter set syntax
void PCCBitstreamDecoderNewSyntax::attributeParameterSet( AttributeParameterSet& aps,
                                                          SequenceParameterSet&  sps,
                                                          PCCBitstream&          bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  aps.setAttributeTypeId( bitstream.read( 4 ) );           // u(4)
  aps.setAttributeDimensionMinus1( bitstream.read( 8 ) );  // u(8)
  aps.setAttributeCodecId( bitstream.read( 8 ) );          // u(8)
  if ( sps.getPcmSeparateVideoPresentFlag() ) {
    aps.setPcmAttributeCodecId( bitstream.read( 8 ) );  // u(8)
  }
  aps.setAttributeParamsEnabledFlag( bitstream.read( 1 ) );  // u(1)
  if ( aps.getAttributeParamsEnabledFlag() ) {
    attributeSequenceParams( aps.getAttributeSequenceParams(), aps.getAttributeDimensionMinus1(),
                             bitstream );
  }
  aps.setAttributePatchParamsEnabledFlag( bitstream.read( 1 ) );  // u(1)
  if ( aps.getAttributePatchParamsEnabledFlag() ) {
    aps.setAttributePatchScaleParamsEnabledFlag( bitstream.read( 1 ) );   // u(1)
    aps.setAttributePatchOffsetParamsEnabledFlag( bitstream.read( 1 ) );  // u(1)
  }
}

// 7.3.13 Attribute sequence Params syntax
void PCCBitstreamDecoderNewSyntax::attributeSequenceParams( AttributeSequenceParams& asp,
                                                            uint8_t                  dimension,
                                                            PCCBitstream&            bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  asp.setAttributeSmoothingParamsPresentFlag( bitstream.read( 1 ) );  // u(1)
  asp.setAttributeScaleParamsPresentFlag( bitstream.read( 1 ) );      // u(1)
  asp.setAttributeOffsetParamsPresentFlag( bitstream.read( 1 ) );     // u(1)
  if ( asp.getAttributeSmoothingParamsPresentFlag() ) {
    asp.setAttributeSmoothingRadius( bitstream.read( 8 ) );                    // u(8)
    asp.setAttributeSmoothingNeighbourCount( bitstream.read( 8 ) );            // u(8)
    asp.setAttributeSmoothingRadius2BoundaryDetection( bitstream.read( 8 ) );  // u(8)
    asp.setAttributeSmoothingThreshold( bitstream.read( 8 ) );                 // u(8)
    asp.setAttributeSmoothingThresholdLocalEntropy( bitstream.read( 8 ) );     // u(3)
  }
  if ( asp.getAttributeScaleParamsPresentFlag() ) {
    asp.getAttributeScaleParams().resize( dimension );
    for ( size_t i = 0; i < dimension; i++ ) {
      asp.setAttributeScaleParam( i, bitstream.read( 32 ) );  // u(32)
    }
  }
  if ( asp.getAttributeOffsetParamsPresentFlag() ) {
    asp.getAttributeOffsetParams().resize( dimension );
    for ( size_t i = 0; i < dimension; i++ ) {
      asp.setAttributeOffsetParam( i, bitstream.read( 32 ) );  // i(32)
    }
  }
}

// 7.3.14 Patch sequence data unit syntax
void PCCBitstreamDecoderNewSyntax::patchSequenceDataUnit( PCCContext& context,
                                                          PCCBitstream&          bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  auto& sps  = context.getSps();
  auto& psdu = context.getPatchSequenceDataUnit();
  auto& psup = psdu.getPatchSequenceUnitPayload();
  psdu.setFrameCount( 0 );
  bool terminatePatchSequenceInformationFlag = false;
  while ( !terminatePatchSequenceInformationFlag ) {
    PatchSequenceUnitPayload psup;
    psup.setUnitType( (PSDUnitType)bitstream.readUvlc() );  // ue(v)
    psup.setFrameIndex( psdu.getFrameCount() );
    patchSequenceUnitPayload( psup, context, bitstream );
    psdu.getPatchSequenceUnitPayload().push_back( psup );
    if ( psup.getUnitType() == PSD_PFLU ) { psdu.getFrameCount()++; }
    terminatePatchSequenceInformationFlag = bitstream.read( 1 );  // u(1)
  }
  byteAlignment( bitstream );
}

// 7.3.15 Patch sequence unit payload syntax
void PCCBitstreamDecoderNewSyntax::patchSequenceUnitPayload( PatchSequenceUnitPayload& psup,
                                                             PCCContext&               context,
                                                             PCCBitstream&             bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  if ( psup.getUnitType() == PSD_SPS ) {
    patchSequenceParameterSet( psup.getPatchSequenceParameterSet(), bitstream );
  } else if ( psup.getUnitType() == PSD_GPPS ) {
    geometryPatchParameterSet( psup.getGeometryPatchParameterSet(),
                               psup.getGeometryFrameParameterSet(), bitstream );
  } else if ( psup.getUnitType() == PSD_APPS ) {
    auto& sps = context.getSps();
    for ( int attributeIndex = 0; attributeIndex < sps.getAttributeCount(); attributeIndex++ ) {
      attributePatchParameterSet( psup.getAttributePatchParameterSet( attributeIndex ),
                                  sps.getAttributeParameterSet( attributeIndex ),
                                  psup.getAttributeFrameParameterSet( attributeIndex ), bitstream );
    }
  } else if ( psup.getUnitType() == PSD_FPS ) {
    auto& sps = context.getSps();
    patchFrameParameterSet( psup.getPatchFrameParameterSet(), sps, bitstream );
  } else if ( psup.getUnitType() == PSD_AFPS ) {
    auto& sps = context.getSps();
    for ( int attributeIndex = 0; attributeIndex < sps.getAttributeCount(); attributeIndex++ ) {
      attributeFrameParameterSet( psup.getAttributeFrameParameterSet( attributeIndex ),
                                  sps.getAttributeParameterSet( attributeIndex ), bitstream );
    }
  } else if ( psup.getUnitType() == PSD_GFPS ) {
    auto& sps = context.getSps();
    geometryFrameParameterSet( psup.getGeometryFrameParameterSet(), sps.getGeometryParameterSet(),
                               bitstream );
  } else if ( psup.getUnitType() == PSD_PFLU ) {
    patchFrameLayerUnit( psup.getPatchFrameLayerUnit(), context, bitstream );
  }
}

// 7.3.16 Patch sequence parameter set syntax
void PCCBitstreamDecoderNewSyntax::patchSequenceParameterSet( PatchSequenceParameterSet& psps,
                                                              PCCBitstream& bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  psps.setPatchSequenceParameterSetId( bitstream.readUvlc() );         // ue(v)
  psps.setLog2MaxPatchFrameOrderCntLsbMinus4( bitstream.readUvlc() );  // ue(v)
  psps.setMaxDecPatchFrameBufferingMinus1( bitstream.readUvlc() );     // ue(v)
  psps.setLongTermRefPatchFramesFlag( bitstream.read( 1 ) );           // u(1)
  psps.setNumRefPatchFrameListsInSps( bitstream.readUvlc() );          // ue(v)
  psps.getRefListStruct().resize( psps.getNumRefPatchFrameListsInSps() );
  for ( size_t i = 0; i < psps.getNumRefPatchFrameListsInSps(); i++ ) {
    refListStruct( psps.getRefListStruct( i ), psps, bitstream );
  }
}

// 7.3.17 Geometry frame parameter set syntax
void PCCBitstreamDecoderNewSyntax::geometryFrameParameterSet( GeometryFrameParameterSet& gfps,
                                                              GeometryParameterSet&      gps,
                                                              PCCBitstream& bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  gfps.setGeometryFrameParameterSetId( bitstream.readUvlc() );  // ue(v)
  gfps.setPatchSequenceParameterSetId( bitstream.readUvlc() );  // ue(v)
  if ( gps.getGeometryParamsEnabledFlag() ) {
    gfps.setOverrideGeometryParamsFlag( bitstream.read( 1 ) );  // u(1)
    if ( gfps.getOverrideGeometryParamsFlag() ) {
      geometryFrameParams( gfps.getGeometryFrameParams(), bitstream );
    }
  }

  if ( gps.getGeometryPatchParamsEnabledFlag() ) {
    gfps.setOverrideGeometryPatchParamsFlag( bitstream.read( 1 ) );  // u(1)
    if ( gfps.getOverrideGeometryPatchParamsFlag() ) {
      gfps.setGeometryPatchScaleParamsEnabledFlag( bitstream.read( 1 ) );     // u(1)
      gfps.setGeometryPatchOffsetParamsEnabledFlag( bitstream.read( 1 ) );    // u(1)
      gfps.setGeometryPatchRotationParamsEnabledFlag( bitstream.read( 1 ) );  // u(1)
      gfps.setGeometryPatchPointSizeInfoEnabledFlag( bitstream.read( 1 ) );   // u(1)
      gfps.setGeometryPatchPointShapeInfoEnabledFlag( bitstream.read( 1 ) );  // u(1)
    }
  }
  byteAlignment( bitstream );
}

// 7.3.18 Geometry frame Params syntax
void PCCBitstreamDecoderNewSyntax::geometryFrameParams( GeometryFrameParams& gfp,
                                                        PCCBitstream&        bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  gfp.setGeometrySmoothingParamsPresentFlag( bitstream.read( 1 ) );  // u(1)
  gfp.setGeometryScaleParamsPresentFlag( bitstream.read( 1 ) );      // u(1)
  gfp.setGeometryOffsetParamsPresentFlag( bitstream.read( 1 ) );     // u(1)
  gfp.setGeometryRotationParamsPresentFlag( bitstream.read( 1 ) );   // u(1)
  gfp.setGeometryPointSizeInfoPresentFlag( bitstream.read( 1 ) );    // u(1)
  gfp.setGeometryPointShapeInfoPresentFlag( bitstream.read( 1 ) );   // u(1)

  if ( gfp.getGeometrySmoothingParamsPresentFlag() ) {
    gfp.setGeometrySmoothingEnabledFlag( bitstream.read( 1 ) );  // u(1)
    if ( gfp.getGeometrySmoothingEnabledFlag() ) {
      gfp.setGeometrySmoothingGridSize( bitstream.read( 8 ) );   // u(8)
      gfp.setGeometrySmoothingThreshold( bitstream.read( 8 ) );  // u(8)
    }
  }
  if ( gfp.getGeometryScaleParamsPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      gfp.setGeometryScaleOnAxis( d, bitstream.read( 32 ) );  // u(32)
    }
  }
  if ( gfp.getGeometryOffsetParamsPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      gfp.setGeometryOffsetOnAxis( d, convertToInt( bitstream.read( 32 ) ) );  // i(32)
    }
  }
  if ( gfp.getGeometryRotationParamsPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      gfp.setGeometryRotationOnAxis( d, convertToInt( bitstream.read( 32 ) ) );  // i(32)
    }
  }
  if ( gfp.getGeometryPointSizeInfoPresentFlag() ) {
    gfp.setGeometryPointSizeInfo( bitstream.read( 16 ) );  // u(16)
  }
  if ( gfp.getGeometryPointShapeInfoPresentFlag() ) {
    gfp.setGeometryPointShapeInfo( bitstream.read( 4 ) );  // u(4)
  }
}

// 7.3.19 Attribute frame parameter set syntax
void PCCBitstreamDecoderNewSyntax::attributeFrameParameterSet( AttributeFrameParameterSet& afps,
                                                               AttributeParameterSet&      aps,
                                                               PCCBitstream& bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  // jkei : this is attributeIndex-th afps
  afps.setAttributeFrameParameterSetId( bitstream.readUvlc() );  // ue(v)
  afps.setPatchSequencParameterSetId( bitstream.readUvlc() );
  size_t attributeDimension = aps.getAttributeDimensionMinus1() + 1;
  if ( aps.getAttributeParamsEnabledFlag() ) {
    afps.setOverrideAttributeParamsFlag( bitstream.read( 1 ) );  // u(1)
    if ( afps.getOverrideAttributeParamsFlag() ) {
      attributeFrameParams( afps.getAttributeFrameParams(), attributeDimension, bitstream );
    }
  }
  if ( aps.getAttributePatchParamsEnabledFlag() ) {
    afps.setOverrideAttributePatchParamsFlag( bitstream.read( 1 ) );  // u(1)
    if ( afps.getOverrideAttributePatchParamsFlag() ) {
      afps.setAttributePatchScaleParamsEnabledFlag( bitstream.read( 1 ) );   // u(1)
      afps.setAttributePatchOffsetParamsEnabledFlag( bitstream.read( 1 ) );  // u(1)
    }
  }

  byteAlignment( bitstream );
}

// 7.3.20 Attribute frame Params syntax
void PCCBitstreamDecoderNewSyntax::attributeFrameParams( AttributeFrameParams& afp,
                                                         size_t                attributeDimension,
                                                         PCCBitstream&         bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  afp.setAttributeSmoothingParamsPresentFlag( bitstream.read( 1 ) );  // u(1)
  afp.setAttributeScaleParamsPresentFlag( bitstream.read( 1 ) );      // u(1)
  afp.setAttributeOffsetParamsPresentFlag( bitstream.read( 1 ) );     // u(1)
  if ( afp.getAttributeSmoothingParamsPresentFlag() ) {
    afp.setAttributeSmoothingRadius( bitstream.read( 8 ) );                    // u(8)
    afp.setAttributeSmoothingNeighbourCount( bitstream.read( 8 ) );            // u(8)
    afp.setAttributeSmoothingRadius2BoundaryDetection( bitstream.read( 8 ) );  // u(8)
    afp.setAttributeSmoothingThreshold( bitstream.read( 8 ) );                 // u(8)
    afp.setAttributeSmoothingThresholdLocalEntropy( bitstream.read( 3 ) );     // u(3)
  }
  if ( afp.getAttributeScaleParamsPresentFlag() ) {
    for ( size_t i = 0; i < attributeDimension; i++ )
      afp.setAttributeScale( i, bitstream.read( 32 ) );  // u32
  }
  if ( afp.getAttributeOffsetParamsPresentFlag() ) {
    for ( size_t i = 0; i < attributeDimension; i++ )
      afp.setAttributeOffset( i, convertToInt( bitstream.read( 32 ) ) );  // i32
  }
}

// 7.3.21 Geometry patch parameter set syntax
void PCCBitstreamDecoderNewSyntax::geometryPatchParameterSet( GeometryPatchParameterSet& gpps,
                                                              GeometryFrameParameterSet& gfps,
                                                              PCCBitstream& bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  gpps.setGeometryPatchParameterSetId( bitstream.readUvlc() );  // ue(v)
  gpps.setGeometryFrameParameterSetId( bitstream.readUvlc() );  // ue(v)
  if ( gfps.getGeometryPatchScaleParamsEnabledFlag() ||
       gfps.getGeometryPatchOffsetParamsEnabledFlag() ||
       gfps.getGeometryPatchRotationParamsEnabledFlag() ||
       gfps.getGeometryPatchPointSizeInfoEnabledFlag() ||
       gfps.getGeometryPatchPointShapeInfoEnabledFlag() ) {
    gpps.setGeometryPatchParamsPresentFlag( bitstream.read( 32 ) );   // u(32)
    if ( gpps.getGeometryPatchParamsPresentFlag() )
      geometryPatchParams( gpps.getGeometryPatchParams(), gfps, bitstream );
  }
  byteAlignment( bitstream );
}

// 7.3.22 Geometry patch Params syntax
void PCCBitstreamDecoderNewSyntax::geometryPatchParams( GeometryPatchParams&       gpp,
                                                        GeometryFrameParameterSet& gfps,
                                                        PCCBitstream&              bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  if ( gfps.getGeometryPatchScaleParamsEnabledFlag() ) {
    gpp.setGeometryPatchScaleParamsPresentFlag( bitstream.read( 1 ) ); // u(1)
    if ( gpp.getGeometryPatchScaleParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        gpp.setGeometryPatchScaleOnAxis( d, bitstream.read( 32 ) );   // u(32)
      }
    }
  }
  if ( gfps.getGeometryPatchOffsetParamsEnabledFlag() ) {
    gpp.setGeometryPatchOffsetParamsPresentFlag( bitstream.read( 1 ) ); // u(1)
    if ( gpp.getGeometryPatchOffsetParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        gpp.setGeometryPatchOffsetOnAxis( d, convertToInt( bitstream.read( 32 ) ) );  // i32
      }
    }
  }
  if ( gfps.getGeometryPatchRotationParamsEnabledFlag() ) {
    gpp.setGeometryPatchRotationParamsPresentFlag( bitstream.read( 1 ) ); // u(1)
    if ( gpp.getGeometryPatchRotationParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        gpp.setGeometryPatchRotationOnAxis( d, convertToInt( bitstream.read( 32 ) ) );  // i(32)
      }
    }
  }
  if ( gfps.getGeometryPatchPointSizeInfoEnabledFlag() ) {
    gpp.setGeometryPatchPointSizeInfoPresentFlag( bitstream.read( 1 ) ); // u(1)
    if ( gpp.getGeometryPatchPointSizeInfoPresentFlag() ) {
      gpp.setGeometryPatchPointSizeInfo( bitstream.read( 16 ) ); // u(16)
    }
  }
  if ( gfps.getGeometryPatchPointShapeInfoEnabledFlag() ) {
    gpp.setGeometryPatchPointShapeInfoPresentFlag( bitstream.read( 1 ) ); // u(1)
    if ( gpp.getGeometryPatchPointShapeInfoPresentFlag() ) {
      gpp.setGeometryPatchPointShapeInfo( bitstream.read( 4 ) ); // u(4)
    }
  }
}

// 7.3.23 Attribute patch parameter set syntax
void PCCBitstreamDecoderNewSyntax::attributePatchParameterSet( AttributePatchParameterSet& apps,
                                                               AttributeParameterSet&      aps,
                                                               AttributeFrameParameterSet& afps,
                                                               PCCBitstream& bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  apps.setAttributePatchParameterSetId( bitstream.readUvlc() ); // ue(v)
  apps.setAttributeFrameParameterSetId( bitstream.readUvlc() ); // ue(v)
  size_t attributeDimension = aps.getAttributeDimensionMinus1() + 1;
  if ( afps.getAttributePatchScaleParamsEnabledFlag() ||
       afps.getAttributePatchOffsetParamsEnabledFlag() ) {
    apps.setAttributePatchParamsPresentFlag(
        bitstream.read( 1 ) );  // u(1)
    if ( apps.getAttributePatchParamsPresentFlag() ) {
      attributePatchParams( apps.getAttributePatchParams(), afps, attributeDimension, bitstream );
    }
  }
  byteAlignment( bitstream );
}

// 7.3.24 Attribute patch Params syntax
void PCCBitstreamDecoderNewSyntax::attributePatchParams( AttributePatchParams&       app,
                                                         AttributeFrameParameterSet& afps,
                                                         size_t                      dimension,
                                                         PCCBitstream&               bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  if ( afps.getAttributePatchScaleParamsEnabledFlag() ) {
    app.setAttributePatchScaleParamsPresentFlag( bitstream.read( 1 ) );  // u(1)
    if ( app.getAttributePatchScaleParamsPresentFlag() ) {
      for ( size_t i = 0; i < dimension; i++ ) {
        app.setAttributePatchScale( i, bitstream.read( 32 ) );  // u(32)
      }
    }
  }
  if ( afps.getAttributePatchOffsetParamsEnabledFlag() ) {
    app.setAttributePatchOffsetParamsPresentFlag( bitstream.read( 1 ) );  // u(1)
    if ( app.getAttributePatchOffsetParamsPresentFlag() ) {
      for ( size_t i = 0; i < dimension; i++ ) {
        app.setAttributePatchOffset( i, convertToInt( bitstream.read( 32 ) ) );  // i(32)
      }
    }
  }
}

// 7.3.25 Patch frame parameter set syntax
void PCCBitstreamDecoderNewSyntax::patchFrameParameterSet( PatchFrameParameterSet& pfps,
                                                           SequenceParameterSet&   sps,
                                                           PCCBitstream&           bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  pfps.setPatchFrameParameterSetId( bitstream.readUvlc() );     // ue(v)
  pfps.setPatchSequenceParameterSetId( bitstream.readUvlc() );  // ue(v)
  // Commented in CD: pfps.getGeometryPatchFrameParameterSetId;  // ue(v)
  // Commented in CD: for ( i = 0; i < spsAttributeCount; i++ ) {
  // Commented in CD:   pfps.getAttributePatchFrameParameterSetId[i]  // ue(v)
  // Commented in CD: }
  pfps.setLocalOverrideGeometryPatchEnableFlag( bitstream.read( 1 ) );  // u(1)
  for ( size_t i = 0; i < sps.getAttributeCount(); i++ ) {
    pfps.setLocalOverrideAttributePatchEnableFlag( i, bitstream.read( 1 ) );  // u(1)
  }
  // Commented in CD: pfps.getNumRefIdxDefaultActiveMinus1 = read() // ue(v)
  pfps.setAdditionalLtPfocLsbLen( bitstream.readUvlc() );  // ue(v)
  if ( sps.getPatchSequenceOrientationEnabledFlag() ) {
    pfps.setPatchOrientationPresentFlag( bitstream.read( 1 ) );  // u(1)
  }
  // Commented in CD: Other?
  byteAlignment( bitstream );
}

// 7.3.26 Patch frame layer unit syntax
void PCCBitstreamDecoderNewSyntax::patchFrameLayerUnit( PatchFrameLayerUnit& pflu,
                                                        PCCContext&          context,
                                                        PCCBitstream&        bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  

  PatchFrameHeader pfh;
  patchFrameHeader( pfh, context, bitstream );
  pflu.setPatchFrameHeader( pfh );
  PatchFrameDataUnit pfdu;
  patchFrameDataUnit( pfdu, pflu.getPatchFrameHeader(), context,
                      bitstream );
  pflu.setPatchFrameDataUnit( pfdu );
  //patchFrameHeader( psps, pflu.getPatchFrameHeader(), frameIndex, bitstream );
  // patchFrameDataUnit( frameIndex );
}

// 7.3.27 Patch frame header syntax
void PCCBitstreamDecoderNewSyntax::patchFrameHeader( PatchFrameHeader& pfh,
                                                     PCCContext&       context,
                                                     PCCBitstream&     bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  pfh.setPatchFrameParameterSetId( bitstream.readUvlc() );                                         // ue( v )
  auto& psdu = context.getPatchSequenceDataUnit();
  auto& psps = psdu.getPatchSequenceParameterSet( pfh.getPatchFrameParameterSetId() );

  pfh.setPatchFrameAddress( bitstream.readUvlc() );                                                // ue( v )
  pfh.setPatchFrameType( bitstream.readUvlc() );                                                   // ue( v )
  pfh.setPatchFrameOderCntLsb( bitstream.readUvlc() );                                             // ue( v )
  if ( psps.getNumRefPatchFrameListsInSps() > 0 ) {
    pfh.setRefPatchFrameListSpsFlag( bitstream.read( 1 ) );                                        // u( 1 )
    if ( pfh.getRefPatchFrameListSpsFlag() ) {                                        
      if ( psps.getNumRefPatchFrameListsInSps() > 1 ) {
        pfh.setRefPatchFrameListIdx( bitstream.readUvlc() );                                       // u( v )
      } else {
        psps.getRefListStruct( psps.getNumRefPatchFrameListsInSps() );
      }
    }

    // RlsIdx[i] = psps_num_ref_patch_frame_lists_in_sps ?
    //   pfh_ref_patch_frame_list_idx[i] : psps_num_ref_patch_frame_lists_in_sps  (7 - 4)
    uint8_t rlsIdx = psps.getNumRefPatchFrameListsInSps() ?
      pfh.getRefPatchFrameListIdx() : psps.getNumRefPatchFrameListsInSps();
    //NumLtrpfEntries[rlsIdx] = 0
    //  for (i = 0; i < num_ref_entries[rlsIdx]; i++)
    //    if (!st_ref_patch_frame_flag[rlsIdx][i])											          (7 - 8)
    //      NumLtrpfEntries[rlsIdx]++
    size_t numLtrpEntries = 0;
    for ( size_t i = 0; i < psps.getRefListStruct( rlsIdx ).getNumRefEntries(); i++ ) {
      if ( !psps.getRefListStruct( rlsIdx ).getStRefPatchFrameFlag( i ) )
        numLtrpEntries++;                                                             
    }

    for ( size_t j = 0; j < numLtrpEntries; j++ ) {
      pfh.setAdditionalPfocLsbPresentFlag( j, bitstream.read( 1 ) );                               // u( 1 )
      if ( pfh.getAdditionalPfocLsbPresentFlag( j ) )
        pfh.setAdditionalPfocLsbVal( j, bitstream.readUvlc() );                                    // ue( v )
    }

    if ( pfh.getPatchFrameType() == P_PATCH_FRAME 
      && psps.getRefListStruct( rlsIdx ).getNumRefEntries() > 1 ) {
      pfh.setPatchFrameNumRefIdxActiveOverrideFlag( bitstream.read( 1 ) );                         // u( 1 )
      if ( pfh.getPatchFrameNumRefIdxActiveOverrideFlag() )
        pfh.setPatchFrameNumRefIdxActiveMinus1( bitstream.readUvlc() );                            // ue( v )
    }
    if ( pfh.getPatchFrameType() == I_PATCH_FRAME ) {
      pfh.setPatchFramehPatch2dShiftUBitCountMinus1( bitstream.read( 8 ) + 1 );                    // u( 8 )
      pfh.setPatchFramehPatch2dShiftVBitCountMinus1( bitstream.read( 8 ) + 1 );                    // u( 8 )
      pfh.setPatchFramehPatch3dShiftTangentAxisBitCountMinus1( bitstream.read( 8 ) + 1 );          // u( 8 )
      pfh.setPatchFramehPatch3dShiftBitangentAxisBitCountMinus1( bitstream.read( 8 ) + 1 );        // u( 8 )
      pfh.setPatchFramehPatch3dShiftNormalAxisBitCountMinus1( bitstream.read( 8 ) + 1 );           // u( 8 )
      pfh.setPatchFramehPatchLodBitCount( bitstream.read( 8 ) + 1 );                               // u( 8 )
    } else {                                                                                   
      pfh.setPatchFrameInterPredictPatchBitCountFlag( bitstream.read( 1 ) );                       // u( 1 )
      if ( pfh.getPatchFrameInterPredictPatchBitCountFlag() ) {
        pfh.setPatchFrameInterPredictPatch2dShiftUBitCountFlag( bitstream.read( 1 ) );             // u( 1 )
        if ( pfh.getPatchFrameInterPredictPatch2dShiftUBitCountFlag() )
          pfh.setPatchFramehPatch2dShiftUBitCountMinus1( bitstream.read( 8 ) + 1 );                // u( 8 )
        pfh.setPatchFrameInterPredictPatch2dShiftVBitCountFlag( bitstream.read( 1 ) );             // u( 1 )
        if ( pfh.getPatchFrameInterPredictPatch2dShiftVBitCountFlag() )
          pfh.setPatchFramehPatch2dShiftVBitCountMinus1( bitstream.read( 8 ) + 1 );                // u( 8 )
        pfh.setPatchFrameInterPredictPatch3dShiftTangentAxisBitCountFlag( bitstream.read( 1 ) );   // u( 1 )
        if ( pfh.getPatchFrameInterPredictPatch3dShiftTangentAxisBitCountFlag() )
          pfh.setPatchFramehPatch3dShiftTangentAxisBitCountMinus1( bitstream.read( 8 ) + 1 );      // u( 8 )
        pfh.setPatchFrameInterPredictPatch3dShiftBitangentAxisBitCountFlag( bitstream.read( 1 ) ); // u( 1 )
        if ( pfh.getPatchFrameInterPredictPatch3dShiftBitangentAxisBitCountFlag() )
          pfh.setPatchFramehPatch3dShiftBitangentAxisBitCountMinus1( bitstream.read( 8 ) + 1 );    // u( 8 )
        pfh.setPatchFrameInterPredictPatch3dShiftNormalAxisBitCountFlag( bitstream.read( 1 ) );    // u( 1 )
        if ( pfh.getPatchFrameInterPredictPatch3dShiftNormalAxisBitCountFlag() )
          pfh.setPatchFramehPatch3dShiftNormalAxisBitCountMinus1( bitstream.read( 8 ) + 1 );       // u( 8 )
        pfh.setPatchFrameInterPredictPatchLodBitCountFlag( bitstream.read( 1 ) );                  // u( 1 )
        if ( pfh.getPatchFrameInterPredictPatchLodBitCountFlag() )
          pfh.setPatchFramehPatchLodBitCount( bitstream.read( 8 ) + 1 );                           // u( 8 )
      }
    }
    byteAlignment(bitstream);
  }
}

// 7.3.28 Reference list structure syntax
void PCCBitstreamDecoderNewSyntax::refListStruct( RefListStruct&             rls,
                                                  PatchSequenceParameterSet& psps,
                                                  PCCBitstream&              bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  
  rls.getNumRefEntries() = bitstream.readUvlc();  // ue(v)
  rls.allocate();
  for ( size_t i = 0; i < rls.getNumRefEntries(); i++ ) {
    if ( psps.getLongTermRefPatchFramesFlag() ) {
      rls.getStRefPatchFrameFlag()[i] = bitstream.read( 1 );  // u(1)
      if ( rls.getStRefPatchFrameFlag()[i] ) {
        rls.getAbsDeltaPfocSt()[i] = bitstream.readUvlc();  // ue(v)
        if ( rls.getAbsDeltaPfocSt()[i] > 0 ) {
          rls.getStrpfEntrySignFlag()[i] = bitstream.read( 1 );  // u(1)
        } else {
          rls.getPfocLsbLt()[i] = bitstream.readUvlc();  // u(v)
        }
      }
    }
  }
}

// 7.3.29 Patch frame data unit syntax
void PCCBitstreamDecoderNewSyntax::patchFrameDataUnit( PatchFrameDataUnit& pfdu,
                                                       PatchFrameHeader&   pfh,
                                                       PCCContext&         context,
                                                       PCCBitstream& bitstream) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  

  uint8_t puCount = -1;
  o3dgc::Arithmetic_Codec arithmeticDecoder;
  uint32_t                compressedBitstreamSize;

  compressedBitstreamSize = bitstream.read<uint32_t>();
  assert( compressedBitstreamSize + bitstream.size() <= bitstream.capacity() );
  arithmeticDecoder.set_buffer( uint32_t( bitstream.capacity() - bitstream.size() ),
                                bitstream.buffer() + bitstream.size() );
  arithmeticDecoder.start_decoder();

  o3dgc::Static_Bit_Model   bModel;
  o3dgc::Adaptive_Bit_Model bModelMoreAvailablePatchFlag;

  bool pfdu_more_available_patch_flag =
      arithmeticDecoder.decode( bModelMoreAvailablePatchFlag );  // ae(v)
  pfdu.init();
  const uint8_t bitCountPatchMode =
      ( PATCH_FRAME_TYPE( pfh.getPatchFrameType() ) ) == I_PATCH_FRAME ? 1 : 2;

  while ( pfdu_more_available_patch_flag ) {
    uint8_t patchMode = DecodeUInt32( bitCountPatchMode, arithmeticDecoder, bModel );  // ae(v) need to write the model
    pfdu.getPatchMode().push_back( patchMode );
    PatchInformationData pid;
    patchInformationData(pid, patchMode, pfh, context, bitstream, arithmeticDecoder);
    pfdu.getPatchInformationData().push_back( pid );
    pfdu_more_available_patch_flag = arithmeticDecoder.decode( bModelMoreAvailablePatchFlag );  // ae(v)
  }

  auto& sps = context.getSps();
  if ( sps.getPointLocalReconstructionEnabledFlag() ) {
    PointLocalReconstruction plr;
    pointLocalReconstruction( plr, context, bitstream, arithmeticDecoder );
  }
  arithmeticDecoder.stop_decoder();
  bitstream += (uint64_t)compressedBitstreamSize;

  byteAlignment( bitstream );

}

// 7.3.30 Patch information data syntax
void PCCBitstreamDecoderNewSyntax::patchInformationData(
    PatchInformationData&    pid,
    size_t                   patchMode,
    PatchFrameHeader&        pfh,
    PCCContext&              context,
                                                         PCCBitstream& bitstream,
    o3dgc::Arithmetic_Codec& arithmeticDecoder ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  

  o3dgc::Static_Bit_Model   bModel;
  o3dgc::Adaptive_Bit_Model bModelOverrideGeometryPatchFlag;
  o3dgc::Adaptive_Bit_Model bModelOverrideAttributePatchFlag;

  auto& sps  = context.getSps();
  auto& psdu = context.getPatchSequenceDataUnit();
  auto& pfps = psdu.getPatchFrameParameterSet( pfh.getPatchFrameParameterSetId() );

  const uint8_t bitCountGAppsId = 6;

  if ( ( PATCH_FRAME_TYPE( pfh.getPatchFrameType() ) ) == P_PATCH_FRAME && patchMode == P_SKIP ) {
    // skip mode.
    // currently not supported but added it for convenience. Could easily be removed
  } else if ( ( PATCH_FRAME_TYPE( pfh.getPatchFrameType() ) ) == I_PATCH_FRAME &&
                  patchMode == I_INTRA ||
              ( ( PATCH_FRAME_TYPE( pfh.getPatchFrameType() ) ) == P_PATCH_FRAME &&
                patchMode == P_INTRA ) ) {
    if ( pfps.getLocalOverrideGeometryPatchEnableFlag() ) {
      const bool overrideGeometryPatchFlag =
          arithmeticDecoder.decode( bModelOverrideGeometryPatchFlag );  // ae(v) regular
      pid.setOverrideGeometryPatchFlag( overrideGeometryPatchFlag );
      if ( pid.getOverrideGeometryPatchFlag() ) {
        uint8_t gppsId = DecodeUInt32( bitCountGAppsId, arithmeticDecoder, bModel );
        pid.setGeometryPatchParameterSetId( gppsId );
      }
    }
    for ( int i = 0; i < sps.getAttributeCount(); i++ ) {
      if ( pfps.getLocalOverrideAttributePatchEnableFlag()[i] ) {
        const bool overrideAttributePatchFlag =
            arithmeticDecoder.decode( bModelOverrideAttributePatchFlag );
        pid.getOverrideAttributePatchFlag().push_back( overrideAttributePatchFlag );
      }
      if ( pid.getOverrideAttributePatchFlag()[i] ) {
        uint8_t appsId = DecodeUInt32( bitCountGAppsId, arithmeticDecoder, bModel );  // ae(v)
        pid.getAttributePatchParameterSetId().push_back( appsId );
      }
    }
    PatchDataUnit pdu;
    patchDataUnit( pdu, pfh, context, bitstream, arithmeticDecoder );
    pid.setPatchDataUnit( pdu );
  } else if ( ( PATCH_FRAME_TYPE( pfh.getPatchFrameType() ) ) == P_PATCH_FRAME &&
              patchMode == P_INTER ) {
    DeltaPatchDataUnit dpdu;
    deltaPatchDataUnit( dpdu, pfh, context, bitstream, arithmeticDecoder);
    pid.setDeltaPatchDataUnit( dpdu );
  } else if ( ( PATCH_FRAME_TYPE( pfh.getPatchFrameType() ) ) == I_PATCH_FRAME &&
                  patchMode == I_PCM ||
              ( PATCH_FRAME_TYPE( pfh.getPatchFrameType() ) ) == P_PATCH_FRAME &&
                  patchMode == P_PCM ) {
    PCMPatchDataUnit ppdu;
    pcmPatchDataUnit( ppdu, pfh, context, bitstream, arithmeticDecoder);
  }
}

// 7.3.31 Patch data unit syntax
void PCCBitstreamDecoderNewSyntax::patchDataUnit( PatchDataUnit&           pdu,
                                                  PatchFrameHeader&        pfh,
                                                  PCCContext&              context,
                                                  PCCBitstream& bitstream,
                                                  o3dgc::Arithmetic_Codec& arithmeticDecoder ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif  

  o3dgc::Static_Bit_Model   bModel;
  o3dgc::Adaptive_Bit_Model bModelSizeU0, bModelSizeV0, bModelOrientationSwapFlag, bModelProjectionFlag;

  size_t val;

  pdu.set2DShiftU(DecodeUInt32( pfh.getPatchFramehPatch2dShiftUBitCountMinus1(), arithmeticDecoder, bModel ) );
  pdu.set2DShiftV(DecodeUInt32( pfh.getPatchFramehPatch2dShiftVBitCountMinus1(), arithmeticDecoder, bModel ) );

  const int64_t deltaSizeU0 = o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelSizeU0 ) );
  pdu.set2DDeltaSizeU( deltaSizeU0 );
  const int64_t deltaSizeV0 = o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelSizeV0 ) );
  pdu.set2DDeltaSizeV( deltaSizeV0 );

  uint32_t axisT = DecodeUInt32(
      pfh.getPatchFramehPatch3dShiftTangentAxisBitCountMinus1(), arithmeticDecoder, bModel );
  pdu.set3DShiftTangentAxis( axisT );
  uint32_t axisB = DecodeUInt32(
      pfh.getPatchFramehPatch3dShiftBitangentAxisBitCountMinus1(), arithmeticDecoder, bModel );
  pdu.set3DShiftTangentAxis( axisB );
  uint32_t axisN = DecodeUInt32( pfh.getPatchFramehPatch3dShiftNormalAxisBitCountMinus1(),
                                           arithmeticDecoder, bModel );
  pdu.set3DShiftTangentAxis( axisN );

  o3dgc::Adaptive_Data_Model orientationModel( 4 );
  pcc::PCCAxis3 normalAxis;
  normalAxis = pcc::PCCAxis3(arithmeticDecoder.decode( orientationModel ));
  pdu.setNormalAxis( normalAxis );

  //******  Does not appear in CD and not implemented by Vlad  *******
  // if (pfh.getPathOrientationPresentFlag())
  //	arithmeticDecoder.decode(pdu.getOrientationSwapFlag(), bModel); // ae(v)

  if ( pfh.getPatchFramehPatchLodBitCount() > 0 ) {
    uint8_t lod =
        DecodeUInt32( pfh.getPatchFramehPatchLodBitCount(), arithmeticDecoder, bModel );  // ae(v)
    pdu.setLod( lod );
  }
  bool  projectionFlag = 0;
  int   i              = 0;
  auto& sps            = context.getSps();
  while ( i < sps.getLayerCountMinus1() + 1 && projectionFlag == 0 ) {
    projectionFlag = projectionFlag | sps.getLayerAbsoluteCodingEnabledFlag()[i];
    i++;
  }
  if ( projectionFlag ) {
    bool projectionMode = arithmeticDecoder.decode( bModelProjectionFlag );  // ae(v)
    pdu.setProjectionMode( projectionMode );
  }
}

// 7.3.32  Delta Patch data unit syntax
void PCCBitstreamDecoderNewSyntax::deltaPatchDataUnit( DeltaPatchDataUnit&      dpdu,
                                                       PatchFrameHeader&        pfh,
                                                       PCCContext&              context, 
                                                       PCCBitstream&            bitstream, 
                                                       o3dgc::Arithmetic_Codec& arithmeticDecoder ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif
  o3dgc::Static_Bit_Model    bModel;
  o3dgc::Adaptive_Bit_Model  bModelPatchIdx, bModelPatch2DShiftU, bModelPatch2DShiftV, bModelPatchSizeU, bModelPatchSizeV;
  o3dgc::Adaptive_Bit_Model  bModelPatch3DShiftNorm, bModelPatch3DShiftTan, bModelPatch3DShiftBitan; 
  o3dgc::Adaptive_Bit_Model  bModelProjectionFlag;

  dpdu.setDeltaPatchIdx( o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPatchIdx ) ) ); // ae(v)
  dpdu.set2DDeltaShiftU( o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPatch2DShiftU ) ) ); // ae(v)
  dpdu.set2DDeltaShiftV( o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPatch2DShiftV ) ) ); // ae(v)
  dpdu.set2DDeltaSizeU( o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPatchSizeU ) ) ); // ae(v)
  dpdu.set2DDeltaSizeV( o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPatchSizeV ) ) ); // ae(v)
  dpdu.set3DDeltaShiftTangentAxis( o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPatch3DShiftTan ) ) ); // ae(v)
  dpdu.set3DDeltaShiftBiTangentAxis( o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPatch3DShiftBitan ) ) ); // ae(v)
  dpdu.set3DDeltaShiftNormalAxis( o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPatch3DShiftNorm ) ) ); // ae(v)
  
  bool  projectionFlag = 0;
  int   i              = 0;
  auto& sps            = context.getSps();

  while ( i < sps.getLayerCountMinus1() + 1 && projectionFlag == 0 ) {
    projectionFlag = projectionFlag | sps.getLayerAbsoluteCodingEnabledFlag()[i];
    i++;
  }
  if ( projectionFlag )
   dpdu.setProjectionMode( arithmeticDecoder.decode( bModelProjectionFlag ) );  // ae(v)
}

// 7.3.33 PCM patch data unit syntax
void PCCBitstreamDecoderNewSyntax::pcmPatchDataUnit( PCMPatchDataUnit&        ppdu,
                                                     PatchFrameHeader&        pfh,
                                                     PCCContext&              context,
                                                     PCCBitstream&            bitstream,
                                                     o3dgc::Arithmetic_Codec& arithmeticDecoder ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif


  o3dgc::Static_Bit_Model    bModel;
  o3dgc::Adaptive_Bit_Model  bModelVideoPCMFlag;
  o3dgc::Adaptive_Bit_Model  bModelPatchSizeU, bModelPatchSizeV, bModelPcmPoints;

  auto& sps = context.getSps();

  if ( sps.getPcmSeparateVideoPresentFlag() )
    ppdu.setPatchInPcmVideoFlag( arithmeticDecoder.decode( bModelVideoPCMFlag ) );                               // ae(v)
  ppdu.set2DShiftU(DecodeUInt32( pfh.getPatchFramehPatch2dShiftUBitCountMinus1(), arithmeticDecoder, bModel ) ); // ae(v)
  ppdu.set2DShiftV(DecodeUInt32( pfh.getPatchFramehPatch2dShiftVBitCountMinus1(), arithmeticDecoder, bModel ) ); // ae(v)
  ppdu.set2DDeltaSizeU( o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPatchSizeU ) ) );  // ae(v)
  ppdu.set2DDeltaSizeV( o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPatchSizeV ) ) );  // ae(v)
  ppdu.setPcmPoints( o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel, bModelPcmPoints ) ) );      // ae(v)
}

// 7.3.34 Point local reconstruction syntax
void PCCBitstreamDecoderNewSyntax::pointLocalReconstruction( PointLocalReconstruction& plr,
                                                             PCCContext& context, 
                                                             PCCBitstream& bitstream, 
                                                             o3dgc::Arithmetic_Codec& arithmeticDecoder ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("%s \n", __func__ );
#endif
  
  o3dgc::Adaptive_Bit_Model  bModelInterpolateFlag, bModelNeighbour, bModelMinDepth, bModelFillingFlag;
  
  /* note: order i, j is handled internally in class set method */
  for ( uint64_t j = 0; j < plr.getBlockToPatchMapHeight(); j++ ) {
    for ( uint64_t i = 0; i < plr.getBlockToPatchMapWidth(); i++ ) {
      if ( plr.getBlockToPatchMap( i, j ) >= 0 ) {
        plr.setModeInterpolateFlag( arithmeticDecoder.decode( bModelInterpolateFlag ), i, j );   // ae(v)
        if ( plr.getModeInterpolateFlag( i, j ) )
          plr.setModeNeighbourMinus1( arithmeticDecoder.decode( bModelNeighbour ), i, j );       // ae(v)
        plr.setModeMinimumDepthMinus1( arithmeticDecoder.decode( bModelMinDepth ), i, j );       // ae(v)
        if ( ( plr.getModeMinimumDepthMinus1( i, j ) > 0 ) ||
             ( plr.getModeInterpolateFlag( i, j ) ) ) 
          plr.setModeFillingFlag( arithmeticDecoder.decode( bModelFillingFlag ), i, j );         // ae(v)
      }
    }
  }
}
