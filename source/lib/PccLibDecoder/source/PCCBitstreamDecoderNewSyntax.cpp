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
  vpccUnitHeader( context, bitstream, vpccUnitType );
  vpccUnitPayload( context, bitstream, vpccUnitType );
}

// 7.3.3 V-PCC unit header syntax
void PCCBitstreamDecoderNewSyntax::vpccUnitHeader( PCCContext&   context,
                                                   PCCBitstream& bitstream,
                                                   VPCCUnitType& vpccUnitType ) {
  auto& vpcc   = context.getVPCC();
  auto& sps    = context.getSps();
  vpccUnitType = (VPCCUnitType)bitstream.read( 5 );  // u(5)
  if ( vpccUnitType == VPCC_AVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_OVD ||
       vpccUnitType == VPCC_PSD ) {
    vpcc.setSequenceParameterSetId(bitstream.read( 4 )) ;  // u(4)
  }
  if ( vpccUnitType == VPCC_AVD ) {
    vpcc.setAttributeIndex(bitstream.read( 7 )) ;  // u(7)
    if ( sps.getMultipleLayerStreamsPresentFlag() ) {
      vpcc.setLayerIndex(bitstream.read( 4 ))  ;  // u(4)
      pcmSeparateVideoData( context, bitstream, 11 );
    } else {
      pcmSeparateVideoData( context, bitstream, 15 );
    }
  } else if ( vpccUnitType == VPCC_GVD ) {
    if ( sps.getMultipleLayerStreamsPresentFlag() ) {
      vpcc.setLayerIndex(bitstream.read( 4 )) ;  // u(4)
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
  auto& vpcc = context.getVPCC();
  auto& sps  = context.getSps();
  if ( sps.getPcmSeparateVideoPresentFlag() && !vpcc.getLayerIndex() ) {
    vpcc.setPCMVideoFlag(bitstream.read( 1 )) ;  // u(1)
    bitstream.read( bitCount );                    // u(bitCount)
  } else {
    bitstream.read( bitCount + 1 );  // u(bitCount + 1)
  }
}

// 7.3.5 V-PCC unit payload syntax
void PCCBitstreamDecoderNewSyntax::vpccUnitPayload( PCCContext&   context,
                                                    PCCBitstream& bitstream,
                                                    VPCCUnitType& vpccUnitType ) {
  if ( vpccUnitType == VPCC_SPS ) {
    vpccSequenceParameterSet( context.getSps(), bitstream );
  } else if ( vpccUnitType == VPCC_PSD ) {
    patchSequenceDataUnit( context.getPatchSequenceDataUnit(), context.getSps(), bitstream );
  } else if ( vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_AVD ) {
    vpccVideoDataUnit( context, bitstream, vpccUnitType );
  }
}

// 7.3.6 Sequence parameter set syntax
void PCCBitstreamDecoderNewSyntax::vpccSequenceParameterSet( SequenceParameterSet& sps,
                                                             PCCBitstream&         bitstream ) {
  profileTierLevel( sps.getProfileTierLevel(), bitstream );
  sps.setSequenceParameterSetId( bitstream.read( 4 ));      // u(4)
  sps.setFrameWidth( bitstream.read( 16 ));     // u(16)
  sps.setFrameHeight( bitstream.read( 16 ));     // u(16)
  sps.setEnhancedOccupancyMapForDepthFlag(bitstream.read( 1 )) ;      // u(1)
  sps.setLayerCount(bitstream.read( 4 ) + 1) ;  // u(4)
  int32_t layerCountMinus1                  = (int32_t)sps.getLayerCount() - 1;
  if ( layerCountMinus1 > 0 ) {
    sps.setMultipleLayerStreamsPresentFlag(bitstream.read( 1 ));  // u(1)
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
  sps.setPcmPatchEnabledFlag(bitstream.read( 1 ));  // u(1)
  if ( sps.getPcmPatchEnabledFlag() ) {
    sps.setPcmSeparateVideoPresentFlag(bitstream.read( 1 )) ;  // u(1)
  }
  occupancyParameterSet( sps.getOccupancyParameterSet(), bitstream );
  geometryParameterSet( sps.getGeometryParameterSet(), sps, bitstream );
  sps.setAttributeCount(bitstream.read( 16 ))  ;  // u(16)

  sps.allocateAttributeParameterSets();
  for ( size_t i = 0; i < sps.getAttributeCount(); i++ ) {
    attributeParameterSet( sps.getAttributeParameterSets( i ), sps, bitstream );
  }
  sps.setPatchSequenceOrientationEnabledFlag(bitstream.read( 1 )) ;  // u(1)
  sps.setPatchInterPredictionEnabledFlag(bitstream.read( 1 ))  ;  // u(1)
  sps.setPixelInterleavingFlag(bitstream.read( 1 )) ;  // u(1)
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
  ptl.setTierFlag(bitstream.read( 1 ));  // u(1)
  ptl.setProfileIdc(bitstream.read( 1 ));  // u(7)
  bitstream.read( 48 );                       // u(48)
  ptl.setLevelIdc(bitstream.read( 8 ));    // u(8)
}

// 7.3.9 Occupancy parameter set syntax
void PCCBitstreamDecoderNewSyntax::occupancyParameterSet( OccupancyParameterSet& ops,
                                                          PCCBitstream&          bitstream ) {
  ops.setOccupancyCodecId(bitstream.read( 8 ));  // u(8)
  ops.setOccupancyPackingBlockSize(bitstream.read( 8 )) ;// u(8)
}

// 7.3.10 Geometry parameter set syntax
void PCCBitstreamDecoderNewSyntax::geometryParameterSet( GeometryParameterSet& gps,
                                                         SequenceParameterSet& sps,
                                                         PCCBitstream&         bitstream ) {
  gps.setGeometryCodecId(bitstream.read( 8 )) ;      // u(8)
  gps.setGeometry3dCoordinatesBitdepth(bitstream.read( 5 ) + 1);  // u(5)
  if ( sps.getPcmSeparateVideoPresentFlag() ) {
    gps.setPcmGeometryCodecId(bitstream.read( 1 ));  // u(8)
  }
  
  gps.setGeometryParamsEnabledFlag(bitstream.read( 1 ));  // u(1)
  //gps.getMetadataEnabledFlag() = bitstream.read( 1 );  // u(1)
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
  aps.setAttributeDimension(bitstream.read( 8 ) + 1) ;  // u(8)
  aps.setAttributeCodecId(bitstream.read( 8 ));      // u(8)
  if ( sps.getPcmSeparateVideoPresentFlag() ) {
    aps.setPcmAttributeCodecId(bitstream.read( 8 ))  ;  // u(8)
  }
  aps.setAttributeParamsEnabledFlag(bitstream.read( 1 ));  // u(1)
  if ( aps.getAttributeParamsEnabledFlag() ) {
    attributeSequenceParams( aps.getAttributeSequenceParams(), aps.getAttributeDimension(), bitstream );
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
      terminatePatchSequenceInformationFlag = bitstream.read( 1 );  // u(1)
    }
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
    geometryPatchParameterSet( psup.getGeometryPatchParameterSet(), bitstream );
  } else if ( psup.getUnitType() == PSD_APPS ) {
    attributePatchParameterSet( psup.getAttributePatchParameterSet(), bitstream );
  } else if ( psup.getUnitType() == PSD_FPS ) {
    patchFrameParameterSet( psup.getPatchFrameParameterSet(), sps, bitstream );
  } else if ( psup.getUnitType() == PSD_AFPS ) {
    attributeFrameParameterSet( psup.getAttributeFrameParameterSet(), bitstream );
  } else if ( psup.getUnitType() == PSD_GFPS ) {
    geometryFrameParameterSet( psup.getGeometryFrameParameterSet(), bitstream );
  } else if ( psup.getUnitType() == PSD_PFLU ) {
    patchFrameLayerUnit( psup.getPatchFrameLayerUnit(), psup.getFrameIndex(), bitstream );
  }
}

// 7.3.16 Patch sequence parameter set syntax
void PCCBitstreamDecoderNewSyntax::patchSequenceParameterSet( PatchSequenceParameterSet& psps,
                                                              PCCBitstream& bitstream ) {
  psps.setPatchSequenceParameterSetId(bitstream.readUvlc())  ;  // ue(v)
  psps.setLog2MaxPatchFrameOrderCntLsbMinus4(bitstream.readUvlc())  ;  // ue(v)
  psps.setMaxDecPatchFrameBufferingMinus1(bitstream.readUvlc())  ;  // ue(v)
  psps.setLongTermRefPatchFramesFlag(bitstream.read( 1 ))         ;   // u(1)
  psps.setNumRefPatchFrameListsInSps(bitstream.readUvlc())         ;  // ue(v)
  psps.getRefListStruct().resize( psps.getNumRefPatchFrameListsInSps() );
  for ( size_t i = 0; i < psps.getNumRefPatchFrameListsInSps(); i++ ) {
    refListStruct( psps.getRefListStruct( i ), psps, bitstream );
  }
}

// 7.3.17 Geometry frame parameter set syntax
void PCCBitstreamDecoderNewSyntax::geometryFrameParameterSet( GeometryFrameParameterSet& gfps,
                                                              PCCBitstream& bitstream ) {
  // gfpsGeometryFrameParameterSetId;  // ue(v)
  // gfpsPatchSequenceParameterSetId;  // ue(v)
  // if ( gpsGeometryParamsEnabledFlag ) {
  //   gfpsOverrideGeometryParamsFlag;  // u(1)
  //   if ( gfpsOverrideGeometryParamsFlag ) { geometryFrameParams(); }
  // }
  // if ( gpsGeometryPatchParamsEnabledFlag ) {
  //   gfpsOverrideGeometryPatchParamsFlag;  // u(1)
  //   if ( gfpsOverrideGeometryPatchParamsFlag ) {
  //     gfpsGeometryPatchScaleParamsEnabledFlag;       // u(1)
  //     gfpsGeometryPatchOffsetParamsEnabledFlag;      // u(1)
  //     gfpsGeometryPatchRotationParamsEnabledFlag;    // u(1)
  //     gfpsGeometryPatchPointSizeParamsEnabledFlag;   // u(1)
  //     gfpsGeometryPatchPointShapeParamsEnabledFlag;  // u(1)
  //   }
  // }
  // byteAlignment( bitstream );
}

// 7.3.18 Geometry frame Params syntax
void PCCBitstreamDecoderNewSyntax::geometryFrameParams( GeometryFrameParams& gfp,
                                                        PCCBitstream&        bitstream ) {
  // gfmGeometrySmoothingParamsPresentFlag;    // u(1)
  // gfmGeometryScaleParamsPresentFlag;        // u(1)
  // gfmGeometryOffsetParamsPresentFlag;       // u(1)
  // gfmGeometryRotationParamsPresentFlag;     // u(1)
  // gfmGeometryPointSizeParamsPresentFlag;    // u(1)
  // gfmGeometryPointShapeParamsPresentFlag;   // u(1)
  // if ( gfmGeometrySmoothingParamsPresentFlag ) {
  //   gfpGeometrySmoothingEnabledFlag ; // u(1)
  //   if ( gfpGeometrySmoothingEnabledFlag ) {
  //     gfpGeometrySmoothingGridSize ; // u(8)
  //     gfpGeometrySmoothingThreshold ; // u(8)
  //   }
  // }
  // if ( gfmGeometryScaleParamsPresentFlag ) {
  //   for ( size_t d = 0; d < 3; d++ ) {
  //     gfmGeometryScaleParamsOnAxis[d];  // u(32)
  //   }
  // }
  // if ( gfmGeometryOffsetParamsPresentFlag ) {
  //   for ( size_t d = 0; d < 3; d++ ) {
  //     gfmGeometryOffsetParamsOnAxis[d];  // i(32)
  //   }
  // }
  // if ( gfmGeometryRotationParamsPresentFlag ) {
  //   for ( size_t d = 0; d < 3; d++ ) {
  //     gfmGeometryRotationParamsOnAxis[d];  // i(32)
  //   }
  // }
  // if ( gfmGeometryPointSizeParamsPresentFlag ) {
  //   gfmGeometryPointSizeParams;  // u(8)
  // }
  // if ( gfmGeometryPointShapeParamsPresentFlag ) {
  //   gfmGeometryPointShapeParams;  // u(8)
  // }
}

// 7.3.19 Attribute frame parameter set syntax
void PCCBitstreamDecoderNewSyntax::attributeFrameParameterSet( AttributeFrameParameterSet& afps,
                                                               PCCBitstream& bitstream ) {
  // afpsAttributeFrameParameterSetId[attributeIndex];  // ue(v)
  // afpsPatchSequenceParameterSetId[attributeIndex];   // ue(v)
  // attributeDimension = apsAttributeDimensionMinus1[attributeIndex] + 1;
  // if ( apsAttributeParamsEnabledFlag[attributeIndex] ) {
  //   afpsOverrideAttributeParams flag[attributeIndex];  // u(1)
  //   if ( afpsOverrideAttributeParamsFlag[attributeIndex] ) {
  //     attributeFrameParams( attributeIndex, attributeDimension );
  //   }
  // }
  // if ( apsAttributePatchParamsEnabledFlag[attributeIndex] ) {
  //   afpsOverrideAttributePatchParamsFlag[attributeIndex];  // u(1)
  //   if ( afpsOverrideAttributePatchParamsFlag[attributeIndex] ) {
  //     afpsAttributePatchScaleParamsEnabledFlag[attributeIndex];   // u(1)
  //     afpsAttributePatchOffsetParamsEnabledFlag[attributeIndex];  // u(1)
  //   }
  // }
  // byteAlignment( bitstream );
}

// 7.3.20 Attribute frame Params syntax
void PCCBitstreamDecoderNewSyntax::attributeFrameParams( AttributeFrameParams& afp,
                                                         size_t                dimension,
                                                         PCCBitstream&         bitstream ) {
  // afmAttributeSmoothingParamsPresentFlag[attributeIndex];  // u(1)
  // afmAttributeScaleParamsPresentFlag[attributeIndex];      // u(1)
  // afmAttributeOffsetParamsPresentFlag[attributeIndex];     // u(1)
  // if ( afmAttributeSmoothingParamsPresentFlag[attributeIndex] ) {
  //   afmAttributeSmoothingRadius[attributeIndex];                      // u(8)
  //   afmAttributeSmoothingNeighbourCount[attributeIndex];              // u(8)
  //   afmAttributeSmoothingRadius2BoundaryDetection[attributeIndex];    // u(8)
  //   afmAttributeSmoothingThreshold[attributeIndex];                   // u(8)
  //   afmAttributeSmoothingThresholdLocalEntropy[attributeIndex];       // u(3)
  // }
  // if ( afmAttributeScaleParamsPresentFlag[attributeIndex] ) {
  //   for ( size_t i = 0; i < attributeDimension; i++ ) {
  //     afmAttributeScaleParams[attributeIndex][i];  // u(32)
  //   }
  // }
  // if ( afmAttributeOffsetParamsPresentFlag[attributeIndex] ) {
  //   for ( size_t i = 0; i < attributeDimension; i++ ) {
  //     afmAttributeOffsetParams[attributeIndex][i];  // i(32)
  //   }
  // }
}

// 7.3.21 Geometry patch parameter set syntax
void PCCBitstreamDecoderNewSyntax::geometryPatchParameterSet( GeometryPatchParameterSet& gpps,
                                                              PCCBitstream& bitstream ) {
  // gppsGeometryPatchParameterSetId;  // ue(v)
  // gppsGeometryFrameParameterSetId;  // ue(v)
  // if ( gfpsGeometryPatchScaleParamsEnabledFlag ||
  //      gfpsGeometryPatchOffsetParamsEnabledFlag ||
  //      gfpsGeometryPatchRotationParamsEnabledFlag ||
  //      gfpsGeometryPatchPointSizeParamsEnabledFlag ||
  //      gfpsGeometryPatchPointShapeParamsEnabledFlag ) {
  //   gppsGeometryPatchParamsPresentFlag;  // u(1)
  //   if ( gppsGeometryPatchParamsPresentFlag ) { geometryPatchParams(); }
  // }
  // byteAlignment( bitstream );
}

// 7.3.22 Geometry patch Params syntax
void PCCBitstreamDecoderNewSyntax::geometryPatchParams( GeometryPatchParams& gpp,
                                                        PCCBitstream&        bitstream ) {
  // if ( gfpsGeometryPatchScaleParamsEnabledFlag ) {
  //   gpmGeometryPatchScaleParamsPresentFlag;  // u(1)
  //   if ( gpmGeometryPatchScaleParamsPresentFlag ) {
  //     for ( size_t d = 0; d < 3; d++ ) {
  //       gpmGeometryPatchScaleParamsOnAxis[d];  // u( 32 )
  //     }
  //   }
  // }
  // if ( gfpsGeometryPatchOffsetParamsEnabledFlag ) {
  //   gpmGeometryPatchOffsetParamsPresentFlag;  // u(1)
  //   if ( gpmGeometryPatchOffsetParamsPresentFlag ) {
  //     for ( size_t d = 0; d < 3; d++ ) {
  //       gpmGeometryPatchOffsetParamsOnAxis[d];  // i( 32 )
  //     }
  //   }
  // }
  // if ( gfpsGeometryPatchRotationParamsEnabledFlag ) {
  //   gpmGeometryPatchRotationParamsPresentFlag;  // u(1)
  //   if ( gpmGeometryPachRotationParamsPresentFlag ) {
  //     for ( size_t d = 0; d < 3; d++ ) {
  //       gpmGeometryPatchRotationParamsOnAxis[d];  // i( 32 )
  //     }
  //   }
  // }
  // if ( gfpsGeometryPatchPointSizeParamsEnabledFlag ) {
  //   gpmGeometryPatchPointSizeParamsPresentFlag;  // u(1)
  //   if ( gpmGeometryPatchPointSizeParamsPresentFlag ) {
  //     gpmGeometryPatchPointSizeParams;  // u(16)
  //   }
  // }
  // if ( gfpsGeometryPatchPointShapeParamsEnabledFlag ) {
  //   gpmGeometryPatchPointShapeParamsPresentFlag;  // u(1)
  //   if ( gpmGeometryPatchPointShapeParamsPresentFlag ) {
  //     gpmGeometryPatchPointShapeParams;  // u(8)
  //   }
  // }
}

// 7.3.23 Attribute patch parameter set syntax
void PCCBitstreamDecoderNewSyntax::attributePatchParameterSet( AttributePatchParameterSet& apps,
                                                               PCCBitstream& bitstream ) {
  // appsAttributePatchParameterSetId[attributeIndex];  // ue(v)
  // appsAttributeFrameParameterSetId[attributeIndex];  // ue(v)
  // attributeDimension = apsAttributeDimensionMinus1[attributeIndex] + 1;
  // if ( afpsAttributePatchScaleParamsEnabledFlag[attributeIndex] ||
  //      afpsAttributePatchOffsetParamsEnabledFlag[attributeIndex] ) {
  //   appsAttributePatchParamsPresentFlag[attributeIndex];  // u(1)
  //   if ( appsAttributePatchParamsPresentFlag[attributeIndex] ) {
  //     attributePatchParams( attributeIndex, attributeDimension );
  //   }
  // }
  // byteAlignment( bitstream );
}

// 7.3.24 Attribute patch Params syntax
void PCCBitstreamDecoderNewSyntax::attributePatchParams( AttributePatchParams& app,
                                                         size_t                dimension,
                                                         PCCBitstream&         bitstream ) {
  // if ( afpsAttributePatchScaleParamsEnabledFlag[attributeIndex] ) {
  //   apmAttributePatchScaleParamsPresentFlag[attributeIndex];  // u(1)
  //   if ( apmAttributePatchScaleParamsPresentFlag[attributeIndex] ) {
  //     for ( size_t i = 0; i < attributeDimension; i++ ) {
  //       apmAttributePatchScaleParams[attributeIndex][i];  // u( 32 )
  //     }
  //   }
  // }
  // if ( afpsAttributePatchOffsetParamsEnabledFlag[attributeIndex] ) {
  //   apmAttributePatchOffsetParamsPresentFlag[attributeIndex];  // u(1)
  //   if ( apmAttributePatchOffsetParamsPresentFlag[attributeIndex] ) {
  //     for ( size_t i = 0; i < attributeDimension; i++ ) {
  //       apmAttributePatchOffsetParams[attributeIndex][i];  // i(32)
  //     }
  //   }
  // }
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
  // if ( pfhType[frameIndex] == P && numRefEntries[RplsIdx] > 1 ) {
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
