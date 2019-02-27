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

//jkei[?] are we going to change it while(...) style later?
//jkei[?] why do we need a variable vpccUntType here''?
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
  //jkei[?]would vpccUnitType=vpccUnitHeader( context, bitstream) be more straightfoward?? but current one looks prettier...
  vpccUnitHeader( context, bitstream, vpccUnitType ); //jkei[!] vpccUnitType : output
  vpccUnitPayload( context, bitstream, vpccUnitType ); //jkei[!] vpccUnitType : input
}

// 7.3.3 V-PCC unit header syntax
void PCCBitstreamDecoderNewSyntax::vpccUnitHeader( PCCContext&   context,
                                                   PCCBitstream& bitstream,
                                                   VPCCUnitType& vpccUnitType ) {
  auto& vpcc   = context.getVPCC();
  auto& sps    = context.getSps();
  vpccUnitType = (VPCCUnitType)bitstream.read( 5 );  // u(5)
  if ( vpccUnitType == VPCC_AVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_OVD ||
       vpccUnitType == VPCC_PSD )
  {
    vpcc.setSequenceParameterSetId( bitstream.read( 4 ) ) ;  // u(4)
  }
  if ( vpccUnitType == VPCC_AVD )
  {
    vpcc.setAttributeIndex( bitstream.read( 7 ) ) ;  // u(7)
    if ( sps.getMultipleLayerStreamsPresentFlag() )
    {
      vpcc.setLayerIndex( bitstream.read( 4 ) )  ;  // u(4)
      pcmSeparateVideoData( context, bitstream, 11 );
    }
    else
    {
      pcmSeparateVideoData( context, bitstream, 15 );
    }
  }
  else if ( vpccUnitType == VPCC_GVD )
  {
    if ( sps.getMultipleLayerStreamsPresentFlag() )
    {
      vpcc.setLayerIndex(bitstream.read( 4 )) ;  // u(4)
      pcmSeparateVideoData( context, bitstream, 18 );
    }
    else
    {
      pcmSeparateVideoData( context, bitstream, 22 );
    }
  }
  else if ( vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_PSD )
  {
    bitstream.read( 23 );  // u(23)
  }
  else
  {
    bitstream.read( 27 );  // u(27)
  }
}

// 7.3.4 PCM separate video data syntax
void PCCBitstreamDecoderNewSyntax::pcmSeparateVideoData( PCCContext&   context,
                                                         PCCBitstream& bitstream,
                                                         uint8_t       bitCount ) {
  auto& vpcc = context.getVPCC();
  auto& sps  = context.getSps();
  if ( sps.getPcmSeparateVideoPresentFlag() && !vpcc.getLayerIndex() )
  {
    vpcc.setPCMVideoFlag( bitstream.read( 1 ) ) ;  // u(1)
    bitstream.read( bitCount );                    // u(bitCount)
  }
  else
  {
    bitstream.read( bitCount + 1 );  // u(bitCount + 1)
  }
}

// 7.3.5 V-PCC unit payload syntax
void PCCBitstreamDecoderNewSyntax::vpccUnitPayload( PCCContext&   context,
                                                    PCCBitstream& bitstream,
                                                    VPCCUnitType& vpccUnitType )
{
  auto& sps  = context.getSps(); //jkei[!] just for consistancy
  auto& psdu = context.getPatchSequenceDataUnit();
  if ( vpccUnitType == VPCC_SPS )
  {
    sequenceParameterSet( sps, bitstream );
  } else if ( vpccUnitType == VPCC_PSD ) {
    patchSequenceDataUnit( psdu, sps, bitstream );
  } else if ( vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_AVD ) {
    vpccVideoDataUnit( context, bitstream, vpccUnitType );
  }
}

// 7.3.6 Sequence parameter set syntax
void PCCBitstreamDecoderNewSyntax::sequenceParameterSet( SequenceParameterSet& sps,
                                                         PCCBitstream&         bitstream ) {
  profileTierLevel( sps.getProfileTierLevel(), bitstream );

  sps.setSequenceParameterSetId( bitstream.read( 4 ) );   // u(4)
  sps.setFrameWidth( bitstream.read( 16 ) );              // u(16)
  sps.setFrameHeight( bitstream.read( 16 ) );             // u(16)
  sps.setAvgFrameRatePresentFlag( bitstream.read( 1 ) );  // u(1)
  if ( sps.getAvgFrameRatePresentFlag() ) {
    sps.setAvgFrameRate( bitstream.read( 16 ) );  // u(16)
  }
  sps.setEnhancedOccupancyMapForDepthFlag(bitstream.read( 1 )) ;      // u(1)
  sps.setLayerCountMinus1(bitstream.read( 4 )) ;  // u(4)
  int32_t layerCountMinus1                  = (int32_t)sps.getLayerCountMinus1();
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
    sps.setPcmSeparateVideoPresentFlag( bitstream.read( 1 ) ) ;  // u(1)
  }
  occupancyParameterSet( sps.getOccupancyParameterSet(), bitstream );
  geometryParameterSet ( sps.getGeometryParameterSet(), sps, bitstream );
  sps.setAttributeCount( bitstream.read( 16 ) );  // u(16)

  sps.allocateAttributeParameterSets();
  for ( size_t i = 0; i < sps.getAttributeCount(); i++ ) {
    attributeParameterSet( sps.getAttributeParameterSet( i ), sps, bitstream );
  }
  sps.setPatchSequenceOrientationEnabledFlag(bitstream.read( 1 )) ;  // u(1)
  sps.setPatchInterPredictionEnabledFlag(bitstream.read( 1 ))  ;  // u(1)
  sps.setPixelDeinterleavingFlag(bitstream.read( 1 )) ;  // u(1)
  sps.setPointLocalReconstructionEnabledFlag(bitstream.read( 1 )) ;  // u(1)
  sps.setRemoveDuplicatePointEnabledFlag(bitstream.read( 1 )) ;  // u(1)
  byteAlignment( bitstream );
}

// 7.3.7 Byte alignment syntax
void PCCBitstreamDecoderNewSyntax::byteAlignment( PCCBitstream& bitstream ) {
  bitstream.read( 1 );  // f(1): equal to 1
  while ( !bitstream.byteAligned() ) {
    bitstream.read( 1 );  // f(1): equal to 0
  }
}

// 7.3.8 Profile, tier, and level syntax
void PCCBitstreamDecoderNewSyntax::profileTierLevel( ProfileTierLevel& ptl,
                                                     PCCBitstream&     bitstream ) {
  ptl.setTierFlag  ( bitstream.read( 1 ) ); // u(1)
  ptl.setProfileIdc( bitstream.read( 1 ) ); // u(7)
  bitstream.read( 48 );                     // u(48)
  ptl.setLevelIdc( bitstream.read( 8 ) );   // u(8)
}

// 7.3.9 Occupancy parameter set syntax
void PCCBitstreamDecoderNewSyntax::occupancyParameterSet( OccupancyParameterSet& ops,
                                                          PCCBitstream&          bitstream ) {
  ops.setOccupancyCodecId( bitstream.read( 8 ) );           // u(8)
  ops.setOccupancyPackingBlockSize( bitstream.read( 8 ) ) ; // u(8)
}

// 7.3.10 Geometry parameter set syntax
void PCCBitstreamDecoderNewSyntax::geometryParameterSet( GeometryParameterSet& gps,
                                                         SequenceParameterSet& sps,
                                                         PCCBitstream&         bitstream ) {

  gps.setGeometryCodecId(bitstream.read( 8 )) ;      // u(8)
  
  gps.setGeometryNominal2dBitdepthMinus1(bitstream.read( 5 ) );  // u(5)
  gps.setGeometry3dCoordinatesBitdepthMinus1(bitstream.read( 5 ) );  // u(5)
  if ( sps.getPcmSeparateVideoPresentFlag() ) {
    gps.setPcmGeometryCodecId(bitstream.read( 1 ));  // u(8)
  }  
  gps.setGeometryParamsEnabledFlag(bitstream.read( 1 ));  // u(1)
  if ( gps.getGeometryParamsEnabledFlag() ) {
    geometrySequenceParams( gps.getGeometrySequenceParams(), bitstream );
  }
  gps.setGeometryPatchParamsEnabledFlag(bitstream.read( 1 ));  // u(1)
  if ( gps.getGeometryPatchParamsEnabledFlag() ) {
    gps.setGeometryPatchScaleParamsEnabledFlag(bitstream.read( 1 ));  // u(1)
    gps.setGeometryPatchOffsetParamsEnabledFlag(bitstream.read( 1 ));  // u(1)
    gps.setGeometryPatchRotationParamsEnabledFlag(bitstream.read( 1 ));  // u(1)
    gps.setGeometryPatchPointSizeInfoEnabledFlag(bitstream.read( 1 ));  // u(1)
    gps.setGeometryPatchPointShapeInfoEnabledFlag(bitstream.read( 1 ));  // u(1)
  }
}

// 7.3.11 Geometry sequence Params syntax
void PCCBitstreamDecoderNewSyntax::geometrySequenceParams( GeometrySequenceParams& gsp,
                                                           PCCBitstream&           bitstream ) {
  gsp.setGeometrySmoothingParamsPresentFlag(bitstream.read( 1 ));  // u(1)
  gsp.setGeometryScaleParamsPresentFlag(bitstream.read( 1 ));  // u(1)
  gsp.setGeometryOffsetParamsPresentFlag(bitstream.read( 1 ));  // u(1)
  gsp.setGeometryRotationParamsPresentFlag(bitstream.read( 1 ));  // u(1)
  gsp.setGeometryPointSizeInfoPresentFlag(bitstream.read( 1 ));  // u(1)
  gsp.setGeometryPointShapeInfoPresentFlag(bitstream.read( 1 )) ;  // u(1)
  if ( gsp.getGeometrySmoothingParamsPresentFlag() ) {
    gsp.setGeometrySmoothingEnabledFlag(bitstream.read( 1 ));  // u(8)
    if ( gsp.getGeometrySmoothingEnabledFlag() ) {
      gsp.setGeometrySmoothingGridSize(bitstream.read( 8 ));  // u(8)
      gsp.setGeometrySmoothingThreshold(bitstream.read( 8 ));  // u(8)
    }
  }
  if ( gsp.getGeometryScaleParamsPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      gsp.setGeometryScaleOnAxis( d,  bitstream.read( 32 )) ;  // u(32)
    }
    if ( gsp.getGeometryOffsetParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        gsp.setGeometryOffsetOnAxis( d, convertToInt( bitstream.read( 32 ) ) ) ;  // i(32)
      }
    }
    if ( gsp.getGeometryRotationParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        gsp.setGeometryRotationOnAxis( d, convertToInt( bitstream.read( 32 ) ) ) ;  // i(32)
      }
    }
    if ( gsp.getGeometryPointSizeInfoPresentFlag() ) {
      gsp.setGeometryPointSizeInfo(bitstream.read( 8 )) ;  // u(8)
    }
    if ( gsp.getGeometryPointShapeInfoPresentFlag() ) {
      gsp.setGeometryPointShapeInfo(bitstream.read( 8 )) ;  // u(8)
    }
  }
}

// 7.3.12 Attribute parameter set syntax
void PCCBitstreamDecoderNewSyntax::attributeParameterSet( AttributeParameterSet& aps,
                                                          SequenceParameterSet&  sps,
                                                          PCCBitstream&          bitstream ) {
  aps.setAttributeTypeId(bitstream.read( 4 )) ;      // u(4)
  aps.setAttributeDimensionMinus1(bitstream.read( 8 ) ) ;  // u(8)
  aps.setAttributeCodecId(bitstream.read( 8 ));      // u(8)
  if ( sps.getPcmSeparateVideoPresentFlag() ) {
    aps.setPcmAttributeCodecId(bitstream.read( 8 ))  ;  // u(8)
  }
  aps.setAttributeParamsEnabledFlag(bitstream.read( 1 ));  // u(1)
  if ( aps.getAttributeParamsEnabledFlag() ) {
    attributeSequenceParams( aps.getAttributeSequenceParams(), aps.getAttributeDimensionMinus1(), bitstream );
  }
  aps.setAttributePatchParamsEnabledFlag(bitstream.read( 1 ));  // u(1)
  if ( aps.getAttributePatchParamsEnabledFlag() ) {
    aps.setAttributePatchScaleParamsEnabledFlag(bitstream.read( 1 )) ;  // u(1)
    aps.setAttributePatchOffsetParamsEnabledFlag(bitstream.read( 1 ));  // u(1)
  }
}

// 7.3.13 Attribute sequence Params syntax
void PCCBitstreamDecoderNewSyntax::attributeSequenceParams( AttributeSequenceParams& asp,
                                                            uint8_t                  dimension,
                                                            PCCBitstream&            bitstream ) {
  asp.setAttributeSmoothingParamsPresentFlag(bitstream.read( 1 )) ;  // u(1)
  asp.setAttributeScaleParamsPresentFlag(bitstream.read( 1 ));  // u(1)
  asp.setAttributeOffsetParamsPresentFlag(bitstream.read( 1 )) ;  // u(1)
  if ( asp.getAttributeSmoothingParamsPresentFlag() ) {
    asp.setAttributeSmoothingRadius(bitstream.read( 8 ));  // u(8)
    asp.setAttributeSmoothingNeighbourCount(bitstream.read( 8 ))           ;  // u(8)
    asp.setAttributeSmoothingRadius2BoundaryDetection(bitstream.read( 8 )) ;  // u(8)
    asp.setAttributeSmoothingThreshold(bitstream.read( 8 ))                ;  // u(8)
    asp.setAttributeSmoothingThresholdLocalEntropy(bitstream.read( 8 ))    ;  // u(3)
  }
  if ( asp.getAttributeScaleParamsPresentFlag() ) {
    asp.getAttributeScaleParams().resize( dimension );
    for ( size_t i = 0; i < dimension; i++ ) {
      asp.setAttributeScaleParam(i, bitstream.read( 32 ));  // u(32)
    }
  }
  if ( asp.getAttributeOffsetParamsPresentFlag() ) {
    asp.getAttributeOffsetParams().resize( dimension );
    for ( size_t i = 0; i < dimension; i++ ) {
      asp.setAttributeOffsetParam(i, bitstream.read( 32 ));  // i(32)
    }
  }
}

// 7.3.14 Patch sequence data unit syntax
void PCCBitstreamDecoderNewSyntax::patchSequenceDataUnit( PatchSequenceDataUnit& psdu,
                                                          SequenceParameterSet&  sps,
                                                          PCCBitstream&          bitstream ) {
  psdu.setFrameCount(0);
  bool terminatePatchSequenceInformationFlag = false;
  while ( !terminatePatchSequenceInformationFlag ) {
    PatchSequenceUnitPayload psup;
    psup.setUnitType((PSDUnitType)bitstream.readUvlc());  // ue(v)
    psup.setFrameIndex(psdu.getFrameCount());
    patchSequenceUnitPayload( psup, sps, bitstream );
    psdu.getPatchSequenceUnitPayload().push_back( psup );
    if ( psup.getUnitType() == PSD_PFLU ) {
      psdu.getFrameCount()++;
    }
    terminatePatchSequenceInformationFlag = bitstream.read( 1 );  // u(1)
  }
  byteAlignment( bitstream );
}

// 7.3.15 Patch sequence unit payload syntax
void PCCBitstreamDecoderNewSyntax::patchSequenceUnitPayload( PatchSequenceUnitPayload& psup,
                                                             SequenceParameterSet&     sps,
                                                             PCCBitstream&             bitstream ) {
  if ( psup.getUnitType() == PSD_SPS ) {
    patchSequenceParameterSet( psup.getPatchSequenceParameterSet(), bitstream );
  } else if ( psup.getUnitType() == PSD_GPPS ) {
    geometryPatchParameterSet( psup.getGeometryPatchParameterSet(), psup.getGeometryFrameParameterSet(), bitstream );
  } else if ( psup.getUnitType() == PSD_APPS ) {
    for( int attributeIndex = 0; attributeIndex < sps.getAttributeCount(); attributeIndex++)
    {
          attributePatchParameterSet( psup.getAttributePatchParameterSet(), sps.getAttributeParameterSet(attributeIndex), psup.getAttributeFrameParameterSet(), bitstream );
    }
  } else if ( psup.getUnitType() == PSD_FPS ) {
    patchFrameParameterSet( psup.getPatchFrameParameterSet(), sps, bitstream );
  } else if ( psup.getUnitType() == PSD_AFPS ) {
    for( int attributeIndex = 0; attributeIndex < sps.getAttributeCount(); attributeIndex++)
    {
      attributeFrameParameterSet( psup.getAttributeFrameParameterSet(), sps.getAttributeParameterSet(attributeIndex), bitstream );
    }
  } else if ( psup.getUnitType() == PSD_GFPS ) {
    geometryFrameParameterSet( psup.getGeometryFrameParameterSet(), sps.getGeometryParameterSet(), bitstream );
  } else if ( psup.getUnitType() == PSD_PFLU ) {
    patchFrameLayerUnit( psup.getPatchFrameLayerUnit(), psup.getFrameIndex(), bitstream );
  }
}

// 7.3.16 Patch sequence parameter set syntax
void PCCBitstreamDecoderNewSyntax::patchSequenceParameterSet( PatchSequenceParameterSet& psps,
                                                              PCCBitstream& bitstream ) {
  psps.setPatchSequenceParameterSetId       ( bitstream.readUvlc() );  // ue(v)
  psps.setLog2MaxPatchFrameOrderCntLsbMinus4( bitstream.readUvlc() );  // ue(v)
  psps.setMaxDecPatchFrameBufferingMinus1   ( bitstream.readUvlc() );  // ue(v)
  psps.setLongTermRefPatchFramesFlag        ( bitstream.read( 1 ) );   // u(1)
  psps.setNumRefPatchFrameListsInSps        ( bitstream.readUvlc() );  // ue(v)
  psps.getRefListStruct().resize( psps.getNumRefPatchFrameListsInSps() );
  for ( size_t i = 0; i < psps.getNumRefPatchFrameListsInSps(); i++ ) {
    refListStruct( psps.getRefListStruct( i ), psps, bitstream );
  }
}
/**********************************************************************/
/* 2019.02.RC4                                                        */
/* ISO/IEC 23090-5:2019(E) d19                                        */
// 7.3.17 Geometry frame parameter set syntax
/**********************************************************************/
void PCCBitstreamDecoderNewSyntax::geometryFrameParameterSet( GeometryFrameParameterSet& gfps,
                                                              GeometryParameterSet&      gps,
                                                              PCCBitstream& bitstream ) {
  
  gfps.setGeometryFrameParameterSetId( bitstream.readUvlc() ); 
  gfps.setPatchSequenceParameterSetId( bitstream.readUvlc() );
  if(gps.getGeometryParamsEnabledFlag() )
  {
    gfps.setOverrideGeometryParamsFlag( bitstream.read( 1 ) );
    if(gfps.getOverrideGeometryParamsFlag())
    {
      geometryFrameParams(gfps.getGeometryFrameParams(), bitstream );
    }
  }
  
  if(gps.getGeometryPatchParamsEnabledFlag())
  {
    gfps.setOverrideGeometryPatchParamsFlag( bitstream.read( 1 ) );
    if(gfps.getOverrideGeometryPatchParamsFlag())
    {
      gfps.setGeometryPatchScaleParamsEnabledFlag   ( bitstream.read( 1 ) );
      gfps.setGeometryPatchOffsetParamsEnabledFlag  ( bitstream.read( 1 ) );
      gfps.setGeometryPatchRotationParamsEnabledFlag( bitstream.read( 1 ) );
      gfps.setGeometryPatchPointSizeInfoEnabledFlag ( bitstream.read( 1 ) );
      gfps.setGeometryPatchPointShapeInfoEnabledFlag( bitstream.read( 1 ) );
    }
  }
  byteAlignment( bitstream );
}

/**********************************************************************/
/* 2019.02.RC4                                                        */
/* ISO/IEC 23090-5:2019(E) d19                                        */
// 7.3.18 Geometry frame Params syntax
/**********************************************************************/
void PCCBitstreamDecoderNewSyntax::geometryFrameParams( GeometryFrameParams& gfp,
                                                        PCCBitstream&        bitstream )
{ 
  gfp.setGeometrySmoothingParamsPresentFlag( bitstream.read( 1 ) ); //  gfp_geometry_smoothing_params_present_flag  u(1)
  gfp.setGeometryScaleParamsPresentFlag    ( bitstream.read( 1 ) ); //  gfp_geometry_scale_params_present_flag  u(1)
  gfp.setGeometryOffsetParamsPresentFlag   ( bitstream.read( 1 ) ); //  gfp_geometry_offset_params_present_flag  u(1)
  gfp.setGeometryRotationParamsPresentFlag ( bitstream.read( 1 ) ); //  gfp_geometry_rotation_params_present_flag  u(1)
  gfp.setGeometryPointSizeInfoPresentFlag  ( bitstream.read( 1 ) ); //  gfp_geometry_point_size_info_present_flag  u(1)
  gfp.setGeometryPointShapeInfoPresentFlag ( bitstream.read( 1 ) ); //  gfp_geometry_point_shape_info_present_flag  u(1)
  
  if(gfp.getGeometrySmoothingParamsPresentFlag())
  {
    gfp.setGeometrySmoothingEnabledFlag( bitstream.read( 1 ) );
    if(gfp.getGeometrySmoothingEnabledFlag())
    {
      gfp.setGeometrySmoothingGridSize( bitstream.read( 8 ) );
      gfp.setGeometrySmoothingThreshold( bitstream.read( 8 ) );
    }
  }
  if(gfp.getGeometryScaleParamsPresentFlag())
  {
    for( size_t d = 0; d < 3; d++)
    {
      gfp.setGeometryScaleOnAxis(d, bitstream.read( 32 )); //      gfp_geometry_scale_on_axis[ d ]  u(32)
    }
  }
  if(gfp.getGeometryOffsetParamsPresentFlag())
  {
    for( size_t d = 0; d < 3; d++)
    {
      gfp.setGeometryOffsetOnAxis(d, convertToInt(bitstream.read(32)) ) ; //i32
    }
  }
  if(gfp.getGeometryRotationParamsPresentFlag())
  {
    for( size_t d = 0; d < 3; d++)
    {
      gfp.setGeometryRotationOnAxis(d, convertToInt(bitstream.read(32))); //i32
    }
  }
  if(gfp.getGeometryPointSizeInfoPresentFlag())
  {
    gfp.setGeometryPointSizeInfo( bitstream.read(16) );
  }
  if(gfp.getGeometryPointShapeInfoPresentFlag())
  {
    gfp.setGeometryPointShapeInfo( bitstream.read(4) );
  }
}
/**********************************************************************/
/* 2019.02.RC5                                                        */
/* ISO/IEC 23090-5:2019(E) d19                                        */
// 7.3.19 Attribute frame parameter set syntax
void PCCBitstreamDecoderNewSyntax::attributeFrameParameterSet( AttributeFrameParameterSet& afps,
                                                               AttributeParameterSet& aps,
                                                               PCCBitstream& bitstream )
{
  //jkei : this is attributeIndex-th afps
  afps.setAttributeFrameParameterSetId( bitstream.readUvlc() );
  afps.setPatchSequencParameterSetId  ( bitstream.readUvlc() );
  size_t attributeDimension = aps.getAttributeDimensionMinus1()+1;
  if( aps.getAttributeParamsEnabledFlag() )
  {
    afps.setOverrideAttributeParamsFlag( bitstream.read(1) );     //    afps_override_attribute_params_flag[ attributeIndex ]  u(1)
    if(afps.getOverrideAttributeParamsFlag())
    {
      attributeFrameParams( afps.getAttributeFrameParams(), attributeDimension, bitstream );
    }
  }
  if(aps.getAttributePatchParamsEnabledFlag())
  {
    
    afps.setOverrideAttributePatchParamsFlag( bitstream.read(1) );
    if( afps.getOverrideAttributePatchParamsFlag() )
    {
      afps.setAttributePatchScaleParamsEnabledFlag ( bitstream.read(1) );
      afps.setAttributePatchOffsetParamsEnabledFlag( bitstream.read(1) );
    }
  }
  
  byteAlignment( bitstream );
}

/**********************************************************************/
/* 2019.02.RC5                                                        */
/* ISO/IEC 23090-5:2019(E) d19                                        */
// 7.3.20 Attribute frame Params syntax
void PCCBitstreamDecoderNewSyntax::attributeFrameParams( AttributeFrameParams& afp,
                                                         size_t                attributeDimension,
                                                         PCCBitstream&         bitstream )
{
  afp.setAttributeSmoothingParamsPresentFlag( bitstream.read(1) );
  afp.setAttributeScaleParamsPresentFlag    ( bitstream.read(1) );
  afp.setAttributeOffsetParamsPresentFlag   ( bitstream.read(1) );
  if( afp.getAttributeSmoothingParamsPresentFlag() )
  {
    afp.setAttributeSmoothingRadius                  ( bitstream.read(8) );
    afp.setAttributeSmoothingNeighbourCount          ( bitstream.read(8) );
    afp.setAttributeSmoothingRadius2BoundaryDetection( bitstream.read(8) );
    afp.setAttributeSmoothingThreshold               ( bitstream.read(8) );
    afp.setAttributeSmoothingThresholdLocalEntropy   ( bitstream.read(3) );
  }
  
  if( afp.getAttributeScaleParamsPresentFlag() )
  {
    for( size_t i = 0; i < attributeDimension; i++ )
      afp.setAttributeScale( i,  bitstream.read(32) ); //u32
  }
  if( afp.getAttributeOffsetParamsPresentFlag() )
  {
    for( size_t i = 0; i < attributeDimension; i++ )
      afp.setAttributeOffset( i,  convertToInt( bitstream.read(32) ) ); //i32
  }
}

/**********************************************************************/
/* 2019.02.RC6                                                        */
/* ISO/IEC 23090-5:2019(E) d19                                        */
// 7.3.21 Geometry patch parameter set syntax
void PCCBitstreamDecoderNewSyntax::geometryPatchParameterSet( GeometryPatchParameterSet& gpps,
                                                              GeometryFrameParameterSet& gfps,
                                                              PCCBitstream&              bitstream  )
{
  gpps.setGeometryPatchParameterSetId( bitstream.readUvlc() );
  gpps.setGeometryFrameParameterSetId( bitstream.readUvlc() );
  if( gfps.getGeometryPatchScaleParamsEnabledFlag() ||
     gfps.getGeometryPatchOffsetParamsEnabledFlag() ||
     gfps.getGeometryPatchRotationParamsEnabledFlag() ||
     gfps.getGeometryPatchPointSizeInfoEnabledFlag() ||
     gfps.getGeometryPatchPointShapeInfoEnabledFlag() )
  {
    gpps.setGeometryPatchParamsPresentFlag( bitstream.read(32) );
    if( gpps.getGeometryPatchParamsPresentFlag() )
      geometryPatchParams(gpps.getGeometryPatchParams(), gfps, bitstream );
  }
    
  byteAlignment( bitstream );
}

/**********************************************************************/
/* 2019.02.RC6                                                        */
/* ISO/IEC 23090-5:2019(E) d19                                        */
// 7.3.22 Geometry patch Params syntax
void PCCBitstreamDecoderNewSyntax::geometryPatchParams( GeometryPatchParams&       gpp,
                                                        GeometryFrameParameterSet& gfps,
                                                        PCCBitstream&              bitstream )
{
 
  if( gfps.getGeometryPatchScaleParamsEnabledFlag() )
  {
    gpp.setGeometryPatchScaleParamsPresentFlag( bitstream.read(1) );
    if( gpp.getGeometryPatchScaleParamsPresentFlag())
    {
      for( size_t d = 0; d < 3; d++ )
      {
        gpp.setGeometryPatchScaleOnAxis( d,  bitstream.read(32) );
      }
    }
  }
  if(gfps.getGeometryPatchOffsetParamsEnabledFlag() )
  {
    gpp.setGeometryPatchOffsetParamsPresentFlag( bitstream.read(1) );
    if( gpp.getGeometryPatchOffsetParamsPresentFlag() )
    {
      for( size_t d = 0; d < 3; d++ )
      {
        gpp.setGeometryPatchOffsetOnAxis( d,  convertToInt(bitstream.read(32))  ); //i32
      }
    }
  }
  if( gfps.getGeometryPatchRotationParamsEnabledFlag() )
  {
    gpp.setGeometryPatchRotationParamsPresentFlag( bitstream.read(1) );
    if( gpp.getGeometryPatchRotationParamsPresentFlag() )
    {
      for( size_t d = 0; d < 3; d++ )
      {
        gpp.setGeometryPatchRotationOnAxis( d,  convertToInt(bitstream.read(32))  );//i(32)
      }
    }
  }
  if( gfps.getGeometryPatchPointSizeInfoEnabledFlag() )
  {
    gpp.setGeometryPatchPointSizeInfoPresentFlag( bitstream.read(1) );
    if( gpp.getGeometryPatchPointSizeInfoPresentFlag() )
    {
      gpp.setGeometryPatchPointSizeInfo( bitstream.read(16) );
    }
  }
  if( gfps.getGeometryPatchPointShapeInfoEnabledFlag() )
  {
    gpp.setGeometryPatchPointShapeInfoPresentFlag( bitstream.read(1) );
    if( gpp.getGeometryPatchPointShapeInfoPresentFlag() )
    {
      gpp.setGeometryPatchPointShapeInfo( bitstream.read(4) );
    }
  }
  
}

/**********************************************************************/
/* 2019.02.RC7                                                        */
/* ISO/IEC 23090-5:2019(E) d19                                        */
// 7.3.23 Attribute patch parameter set syntax
/**********************************************************************/
void PCCBitstreamDecoderNewSyntax::attributePatchParameterSet( AttributePatchParameterSet& apps,
                                                               AttributeParameterSet&      aps,
                                                               AttributeFrameParameterSet& afps,
                                                               PCCBitstream&               bitstream )
{
  apps.setAttributePatchParameterSetId( bitstream.readUvlc() );
  apps.setAttributeFrameParameterSetId( bitstream.readUvlc() );
  
  size_t attributeDimension = aps.getAttributeDimensionMinus1() + 1;
  if( afps.getAttributePatchScaleParamsEnabledFlag() || //_attribute_patch_scale_params_enabled_flag[ attributeIndex ]
     afps.getAttributePatchOffsetParamsEnabledFlag() ) //afps_attribute_patch_offset_params_enabled_flag[ attributeIndex ]
  {
    apps.setAttributePatchParamsPresentFlag( bitstream.read(1) ); //apps_attribute_patch_params_present_flag[ attributeIndex ]
    if( apps.getAttributePatchParamsPresentFlag() )
    {
      attributePatchParams(apps.getAttributePatchParams(), afps, attributeDimension, bitstream);
    }
  }
  byteAlignment( bitstream );
}

/**********************************************************************/
/* 2019.02.RC7                                                        */
/* ISO/IEC 23090-5:2019(E) d19                                        */
// 7.3.24 Attribute patch Params syntax
/**********************************************************************/
void PCCBitstreamDecoderNewSyntax::attributePatchParams( AttributePatchParams&       app,
                                                         AttributeFrameParameterSet& afps,
                                                         size_t                      attributeDimension,
                                                         PCCBitstream&               bitstream )
{
  if( afps.getAttributePatchScaleParamsEnabledFlag() )
  {
    app.setAttributePatchScaleParamsPresentFlag(bitstream.read(1));
    if( app.getAttributePatchScaleParamsPresentFlag() )
    {
      for( size_t i = 0; i < attributeDimension; i++ )
      {
        app.setAttributePatchScale( i, bitstream.read(32) ); //u(32)
      }
    }
  }
  if( afps.getAttributePatchOffsetParamsEnabledFlag() )
  {
    app.setAttributePatchOffsetParamsPresentFlag( bitstream.read(1) ); 
    if( app.getAttributePatchOffsetParamsPresentFlag() )
    {
      for( size_t i = 0; i < attributeDimension; i++ )
      {
        app.setAttributePatchOffset( i , convertToInt( bitstream.read(32)) ); // i(32)
      }
    }
  }
  
}


// 7.3.25 Patch frame parameter set syntax
void PCCBitstreamDecoderNewSyntax::patchFrameParameterSet( PatchFrameParameterSet& pfps,
                                                           SequenceParameterSet&   sps,
                                                           PCCBitstream&           bitstream ) {
  pfps.setPatchFrameParameterSetId(bitstream.readUvlc()) ;  // ue(v)
  pfps.setPatchSequenceParameterSetId(bitstream.readUvlc()) ;  // ue(v)
  // Commented in CD: pfps.getGeometryPatchFrameParameterSetId;  // ue(v)
  // Commented in CD: for ( i = 0; i < spsAttributeCount; i++ ) {
  // Commented in CD:   pfps.getAttributePatchFrameParameterSetId[i]  // ue(v)
  // Commented in CD: }
  pfps.setLocalOverrideGeometryPatchEnableFlag(bitstream.read( 1 )) ;  // u(1)
  for ( size_t i = 0; i < sps.getAttributeCount(); i++ ) {
    pfps.setLocalOverrideAttributePatchEnableFlag(i, bitstream.read( 1 ));  // u(1)
  }
  // Commented in CD: pfps.getNumRefIdxDefaultActiveMinus1 = read() // ue(v)
  pfps.setAdditionalLtPfocLsbLen(bitstream.readUvlc());  // ue(v)
  if ( sps.getPatchSequenceOrientationEnabledFlag() ) {
    pfps.setPatchOrientationPresentFlag(bitstream.read( 1 )) ;  // u(1)
  }
  // Commented in CD: Other?
  byteAlignment( bitstream );
}

// 7.3.26 Patch frame layer unit syntax
void PCCBitstreamDecoderNewSyntax::patchFrameLayerUnit( PatchFrameLayerUnit& pflu,
                                                        size_t               frameIndex,
                                                        PCCBitstream&        bitstream ) {
  // patchFrameHeader( frameIndex );
  // patchFrameDataUnit( frameIndex );
}

// 7.3.27 Patch frame header syntax
void PCCBitstreamDecoderNewSyntax::patchFrameHeader( PatchFrameHeader& pfh,
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
void PCCBitstreamDecoderNewSyntax::refListStruct( RefListStruct&             rls,
                                                  PatchSequenceParameterSet& psps,
                                                  PCCBitstream&              bitstream ) {
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
void PCCBitstreamDecoderNewSyntax::patchFrameDataUnit( PCCContext&   context,
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
void PCCBitstreamDecoderNewSyntax::patchInformationData( PCCContext&   context,
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
void PCCBitstreamDecoderNewSyntax::patchDataUnit( PCCContext&   context,
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
void PCCBitstreamDecoderNewSyntax::deltaPatchDataUnit( PCCContext&   context,
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
void PCCBitstreamDecoderNewSyntax::pcmPatchDataUnit( PCCContext&   context,
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
void PCCBitstreamDecoderNewSyntax::pointLocalReconstruction( PCCContext&   context,
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
