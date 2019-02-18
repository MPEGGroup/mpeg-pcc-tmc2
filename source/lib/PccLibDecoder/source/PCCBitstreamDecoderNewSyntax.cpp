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
    vpcc.getSequenceParameterSetId() = bitstream.read( 4 );  // u(4)
  }
  if ( vpccUnitType == VPCC_AVD ) {
    vpcc.getAttributeIndex() = bitstream.read( 7 );  // u(7)
    if ( sps.getMultipleLayerStreamsPresentFlag() ) {
      vpcc.getLayerIndex() = bitstream.read( 4 );  // u(4)
      pcmSeparateVideoData( context, bitstream, 11 );
    } else {
      pcmSeparateVideoData( context, bitstream, 15 );
    }
  } else if ( vpccUnitType == VPCC_GVD ) {
    if ( sps.getMultipleLayerStreamsPresentFlag() ) {
      vpcc.getLayerIndex() = bitstream.read( 4 );  // u(4)
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
    vpcc.getPCMVideoFlag() = bitstream.read( 1 );  // u(1)
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
    patchSequenceDataUnit( context, bitstream );
  } else if ( vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_AVD ) {
    vpccVideoDataUnit( context, bitstream, vpccUnitType );
  }
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
    if ( context.getUseAdditionalPointsPatch() && context.getSps().getPcmSeparateVideoPresentFlag() ) {
      bitstream.read( context.getVideoBitstream( PCCVideoType::GeometryMP ) );
    }
  } else if ( vpccUnitType == VPCC_AVD ) {
    if ( !context.getNoAttributes() ) {
      bitstream.read( context.getVideoBitstream( PCCVideoType::Texture ) );
      if ( context.getUseAdditionalPointsPatch() && context.getSps().getPcmSeparateVideoPresentFlag() ) {
        bitstream.read( context.getVideoBitstream( PCCVideoType::TextureMP ) );
      }
    }
  }
}

// 7.3.6 Sequence parameter set syntax
void PCCBitstreamDecoderNewSyntax::vpccSequenceParameterSet( SequenceParameterSet& sps,
                                                             PCCBitstream&         bitstream ) {
  profileTierLevel( sps.getProfileTierLevel(), bitstream );
  sps.getIndex()                        = bitstream.read( 4 );      // u(4)
  sps.getWidth()                        = bitstream.read( 16 );     // u(16)
  sps.getHeight()                       = bitstream.read( 16 );     // u(16)
  sps.getEnhancedDepthCodeEnabledFlag() = bitstream.read( 1 );      // u(1)
  sps.getLayerCount()                   = bitstream.read( 4 ) + 1;  // u(4)
  int32_t layerCountMinus1              = (int32_t)sps.getLayerCount() - 1;
  if ( layerCountMinus1 > 0 ) {
    sps.getMultipleLayerStreamsPresentFlag() = bitstream.read( 1 );  // u(1)
  }
  auto& layerAbsoluteCodingEnabledFlag = sps.getLayerAbsoluteCodingEnabledFlag();
  auto& layerPredictorIndexDiff        = sps.getLayerPredictorIndexDiff();
  auto& layerPredictorIndex            = sps.getLayerPredictorIndex();
  for ( size_t i = 0; i < layerCountMinus1; i++ ) {
    layerAbsoluteCodingEnabledFlag[i + 1] = bitstream.read( 1 );  // u(1)
    if ( ( layerAbsoluteCodingEnabledFlag[i + 1] == 0 ) && ( i > 0 ) ) {
      if ( i > 0 ) {
        layerPredictorIndexDiff[i + 1] = bitstream.readUvlc();  // ue(v)
      } else {
        layerPredictorIndexDiff[i + 1] = 0;
      }
      layerPredictorIndex[i + 1] = i - layerPredictorIndexDiff[i + 1];
    }
  }
  sps.getPcmPatchEnabledFlag() = bitstream.read( 1 );  // u(1)
  if ( sps.getPcmPatchEnabledFlag() ) {
    sps.getPcmSeparateVideoPresentFlag() = bitstream.read( 1 );  // u(1)
  }
  occupancyParameterSet( sps.getOccupancyParameterSet(), bitstream );
  geometryParameterSet( sps.getGeometryParameterSet(), sps, bitstream );
  sps.getAttributeCount() = bitstream.read( 16 );  // u(16)
  for ( size_t i = 0; i < sps.getAttributeCount(); i++ ) {
    attributeParameterSet( sps.getAttributeParameterSets( i ), bitstream, i );
  }
  sps.getPatchSequenceOrientationEnabledFlag() = bitstream.read( 1 );  // u(1)
  sps.getPatchInterPredictionEnabledFlag()     = bitstream.read( 1 );  // u(1)
  sps.getPixelInterleavingFlag()               = bitstream.read( 1 );  // u(1)
  sps.getPointLocalReconstructionEnabledFlag() = bitstream.read( 1 );  // u(1)
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
  ptl.getTierFlag()   = bitstream.read( 1 );  // u(1)
  ptl.getProfileIdc() = bitstream.read( 1 );  // u(7)
  bitstream.read( 48 );                       // u(48)
  ptl.getLevelIdc() = bitstream.read( 8 );    // u(8)
}

// 7.3.9 Occupancy parameter set syntax
void PCCBitstreamDecoderNewSyntax::occupancyParameterSet( OccupancyParameterSet& ops,
                                                          PCCBitstream&          bitstream ) {
  ops.getCodecId()          = bitstream.read( 8 );  // u(8)
  ops.getPackingBlockSize() = bitstream.read( 8 );  // u(8)
}

// 7.3.10 Geometry parameter set syntax
void PCCBitstreamDecoderNewSyntax::geometryParameterSet( GeometryParameterSet& gps,
                                                         SequenceParameterSet& sps,
                                                         PCCBitstream&         bitstream ) {
  gps.getCodecId()               = bitstream.read( 8 );      // u(8)
  gps.get3dCoordinatesBitdepth() = bitstream.read( 5 ) + 1;  // u(5)
  if ( sps.getPcmSeparateVideoPresentFlag() ) {
    gps.getPcmCodecId() = bitstream.read( 1 );  // u(8)
  }
  gps.getMetadataEnabledFlag();  // u(1)
  if ( gps.getMetadataEnabledFlag() ) {
    geometrySequenceMetadata( gps.getGeometrySequenceMetadata(), bitstream );
  }
  gps.getPatchMetadataEnabledFlag() = bitstream.read( 1 );  // u(1)
  if ( gps.getPatchMetadataEnabledFlag() ) {
    gps.getPatchScaleMetadataEnabledFlag()      = bitstream.read( 1 );  // u(1)
    gps.getPatchOffsetMetadataEnabledFlag()     = bitstream.read( 1 );  // u(1)
    gps.getPatchRotationMetadataEnabledFlag()   = bitstream.read( 1 );  // u(1)
    gps.getPatchPointSizeMetadataEnabledFlag()  = bitstream.read( 1 );  // u(1)
    gps.getPatchPointShapeMetadataEnabledFlag() = bitstream.read( 1 );  // u(1)
  }
}

// 7.3.11 Geometry sequence metadata syntax
void PCCBitstreamDecoderNewSyntax::geometrySequenceMetadata( GeometrySequenceMetadata& gsm,
                                                             PCCBitstream&             bitstream ) {
  gsm.getSmoothingMetadataPresentFlag()  = bitstream.read( 1 );  // u(1)
  gsm.getScaleMetadataPresentFlag()      = bitstream.read( 1 );  // u(1)
  gsm.getOffsetMetadataPresentFlag()     = bitstream.read( 1 );  // u(1)
  gsm.getRotationMetadataPresentFlag()   = bitstream.read( 1 );  // u(1)
  gsm.getPointSizeMetadataPresentFlag()  = bitstream.read( 1 );  // u(1)
  gsm.getPointShapeMetadataPresentFlag() = bitstream.read( 1 );  // u(1)
  if ( gsm.getSmoothingMetadataPresentFlag() ) {
    gsm.getSmoothingRadius()                   = bitstream.read( 8 );  // u(8)
    gsm.getSmoothingNeighbourCount()           = bitstream.read( 8 );  // u(8)
    gsm.getSmoothingRadius2BoundaryDetection() = bitstream.read( 8 );  // u(8)
    gsm.getSmoothingThreshold()                = bitstream.read( 8 );  // u(8)
  }
  if ( gsm.getScaleMetadataPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      gsm.getScaleMetadataOnAxis( d ) = bitstream.read( 32 );  // u(32)
    }
    if ( gsm.getOffsetMetadataPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        gsm.getOffsetMetadataOnAxis( d ) = bitstream.read( 32 );  // i(32)
      }
    }
    if ( gsm.getRotationMetadataPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        gsm.getRotationMetadataOnAxis( d ) = bitstream.read( 32 );  // i(32)
      }
    }
    if ( gsm.getPointSizeMetadataPresentFlag() ) {
      gsm.getPointSizeMetadata() = bitstream.read( 8 );  // u(8)
    }
    if ( gsm.getPointShapeMetadataPresentFlag() ) {
      gsm.getPointShapeMetadata() = bitstream.read( 8 );  // u(8)
    }
  }
}

// 7.3.12 Attribute parameter set syntax
void PCCBitstreamDecoderNewSyntax::attributeParameterSet(
    AttributeParameterSet& attributeParameterSet, PCCBitstream& bitstream, size_t attributeIndex ) {
  // apsAttributeId = attributeIndex;
  // apsAttributeTypeId[attributeIndex];           // u(4)
  // apsAttributeDimensionMinus1[attributeIndex];  // u(8)
  // apsAttributeCodecId[attributeIndex];          // u(8)
  // attributeDimension = apsAttributeDimensionMinus1[attributeIndex] + 1;
  // if ( spsPcmSeparateVideoPresentFlag ) {
  //   apsPcmAttributeCodecId[attributeIndex];  // u(8)
  // }
  // apsAttributeMetadataEnabledFlag[attributeIndex];  // u(1)
  // if ( apsAttributeMetadataEnabledFlag[attributeIndex] ) {
  //   attributeSequenceMetadata( attributeIndex, attributeDimension );
  // }
  // apsAttributePatchMetadataEnabledFlag[attributeIndex];  // u(1)
  // if ( apsAttributePatchMetadataEnabledFlag[attributeIndex] ) {
  //   apsAttributePatchScaleMetadataEnabledFlag[attributeIndex];   // u(1)
  //   apsAttributePatchOffsetMetadataEnabledFlag[attributeIndex];  // u(1)
  // }
}

// 7.3.13 Attribute sequence metadata syntax
void PCCBitstreamDecoderNewSyntax::attributeSequenceMetadata( PCCContext&   context,
                                                              PCCBitstream& bitstream,
                                                              size_t        attributeIndex,
                                                              size_t        attributeDimension ) {
  // asmAttributeSmoothingMetadataPresentFlag[attributeIndex];  // u(1)
  // asmAttributeScaleMetadataPresentFlag[attributeIndex];      // u(1)
  // asmAttributeOffsetMetadataPresentFlag[attributeIndex];     // u(1)
  // if ( asmAttributeSmoothingMetadataPresentFlag[attributeIndex] ) {
  //   asmAttributeSmoothingRadius[attributeIndex];                      // u(8)
  //   asmAttributeSmoothingNeighbourCount[attributeIndex];             // u(8)
  //   asmAttributeSmoothingRadius2BoundaryDetection[attributeIndex];  // u(8)
  //   asmAttributeSmoothingThreshold[attributeIndex];                   // u(8)
  //   asmAttributeSmoothingThresholdLocalEntropy[attributeIndex];     // u(3)
  // }
  // if ( asmAttributeScaleMetadataPresentFlag[attributeIndex] ) {
  //   for ( size_t i = 0; i < attributeDimension; i++ ) {
  //     asmAttributeScaleMetadata[attributeIndex][i];  // u(32)
  //     if ( asmAttributeOffsetMetadataPresentFlag[attributeIndex] ) {
  //       for ( size_t i = 0; i < attributeDimension; i++ ) {
  //         asmAttributeOffsetMetadata[attributeIndex][i];  // u(32)
  //       }
  //     }
  //   }
  // }
}

// 7.3.14 Patch sequence data unit syntax
void PCCBitstreamDecoderNewSyntax::patchSequenceDataUnit( PCCContext&   context,
                                                          PCCBitstream& bitstream ) {
  // psdFrameCount                                 = 0;
  // psdTerminatePatchSequenceInformationFlag = 0;
  // while ( psdTerminatePatchSequenceInformationSignal == 0 ) {
  //   psdUnitType;  // ue(v)
  //   patchSequenceUnitPayload( psdFrameCount );
  //   if ( psdUnitType == PSDPFLU ) {
  //     psdFrameCount++ psdTerminatePatchSequenceInformationFlag;  // u(1)
  //   }
  // }
  // byteAlignment( bitstream );
}

// 7.3.15 Patch sequence unit payload syntax
void PCCBitstreamDecoderNewSyntax::patchSequenceUnitPayload( PCCContext&   context,
                                                             PCCBitstream& bitstream,
                                                             size_t        frameIndex ) {
  // if ( psdUnitType == PSDSPS ) {
  //   patchSequenceParameterSet();
  // } else if ( psdUnitType == PSDGPPS ) {
  //   geometryPatchParameterSet();
  // } else if ( psdUnitType == PSDAPPS ) {
  //   attributePatchParameterSet();
  // } else if ( psdUnitType == PSDFPS ) {
  //   patchFrameParameterSet();
  // } else if ( psdUnitType == PSDAFPS ) {
  //   attributeFrameParameterSet();
  // } else if ( psdUnitType == PSDGFPS ) {
  //   geometryFrameParameterSet();
  // } else if ( psdUnitType == PSDPFLU ) {
  //   patchFrameLayerUnit( frameIndex );
  // }
}

// 7.3.16 Patch sequence parameter set syntax
void PCCBitstreamDecoderNewSyntax::patchSequenceParameterSet( PCCContext&   context,
                                                              PCCBitstream& bitstream ) {
  // pspsPatchSequenceParameterSetId;            // ue(v)
  // pspsLog2MaxPatchFrameOrderCntLsbMinus4;     // ue(v)
  // pspsMaxDecPatchFrameBufferingMinus1;        // ue(v)
  // pspsLongTermRefPatchFramesFlag;             // u(1)
  // pspsNumRefPatchFrameListsInSps;             // ue(v)
  // for ( size_t j = 0; j < pspsNumRefPatchFrameListsInSps; j++ ) { refListStruct( j ); }
}

// 7.3.17 Geometry frame parameter set syntax
void PCCBitstreamDecoderNewSyntax::geometryFrameParameterSet( PCCContext&   context,
                                                              PCCBitstream& bitstream ) {
  // gfpsGeometryFrameParameterSetId;  // ue(v)
  // gfpsPatchSequenceParameterSetId;  // ue(v)
  // if ( gpsGeometryMetadataEnabledFlag ) {
  //   gfpsOverrideGeometryMetadataFlag;  // u(1)
  //   if ( gfpsOverrideGeometryMetadataFlag ) { geometryFrameMetadata(); }
  // }
  // if ( gpsGeometryPatchMetadataEnabledFlag ) {
  //   gfpsOverrideGeometryPatchMetadataFlag;  // u(1)
  //   if ( gfpsOverrideGeometryPatchMetadataFlag ) {
  //     gfpsGeometryPatchScaleMetadataEnabledFlag;       // u(1)
  //     gfpsGeometryPatchOffsetMetadataEnabledFlag;      // u(1)
  //     gfpsGeometryPatchRotationMetadataEnabledFlag;    // u(1)
  //     gfpsGeometryPatchPointSizeMetadataEnabledFlag;   // u(1)
  //     gfpsGeometryPatchPointShapeMetadataEnabledFlag;  // u(1)
  //   }
  // }
  // byteAlignment( bitstream );
}

// 7.3.18 Geometry frame metadata syntax
void PCCBitstreamDecoderNewSyntax::geometryFrameMetadata( PCCContext&   context,
                                                          PCCBitstream& bitstream ) {
  // gfmGeometrySmoothingMetadataPresentFlag;    // u(1)
  // gfmGeometryScaleMetadataPresentFlag;        // u(1)
  // gfmGeometryOffsetMetadataPresentFlag;       // u(1)
  // gfmGeometryRotationMetadataPresentFlag;     // u(1)
  // gfmGeometryPointSizeMetadataPresentFlag;    // u(1)
  // gfmGeometryPointShapeMetadataPresentFlag;   // u(1)
  // if ( gfmGeometrySmoothingMetadataPresentFlag ) {
  //   gfmGeometrySmoothingRadius;                      // u(8)
  //   gfmGeometrySmoothingNeighbourCount;              // u(8)
  //   gfmGeometrySmoothingRadius2BoundaryDetection;    // u(8)
  //   gfmGeometrySmoothingThreshold;                   // u(8)
  // }
  // if ( gfmGeometryScaleMetadataPresentFlag ) {
  //   for ( size_t d = 0; d < 3; d++ ) {
  //     gfmGeometryScaleMetadataOnAxis[d];  // u(32)
  //   }
  // }
  // if ( gfmGeometryOffsetMetadataPresentFlag ) {
  //   for ( size_t d = 0; d < 3; d++ ) {
  //     gfmGeometryOffsetMetadataOnAxis[d];  // i(32)
  //   }
  // }
  // if ( gfmGeometryRotationMetadataPresentFlag ) {
  //   for ( size_t d = 0; d < 3; d++ ) {
  //     gfmGeometryRotationMetadataOnAxis[d];  // i(32)
  //   }
  // }
  // if ( gfmGeometryPointSizeMetadataPresentFlag ) {
  //   gfmGeometryPointSizeMetadata;  // u(8)
  // }
  // if ( gfmGeometryPointShapeMetadataPresentFlag ) {
  //   gfmGeometryPointShapeMetadata;  // u(8)
  // }
}

// 7.3.19 Attribute frame parameter set syntax [ED.Note: should this be renamed to patch frame
// attribute parameter set?]
void PCCBitstreamDecoderNewSyntax::attributeFrameParameterSet( PCCContext&   context,
                                                               PCCBitstream& bitstream,
                                                               size_t        attributeIndex ) {
  // afpsAttributeFrameParameterSetId[attributeIndex];  // ue(v)
  // afpsPatchSequenceParameterSetId[attributeIndex];   // ue(v)
  // attributeDimension = apsAttributeDimensionMinus1[attributeIndex] + 1;
  // if ( apsAttributeMetadataEnabledFlag[attributeIndex] ) {
  //   afpsOverrideAttributeMetadata flag[attributeIndex];  // u(1)
  //   if ( afpsOverrideAttributeMetadataFlag[attributeIndex] ) {
  //     attributeFrameMetadata( attributeIndex, attributeDimension );
  //   }
  // }
  // if ( apsAttributePatchMetadataEnabledFlag[attributeIndex] ) {
  //   afpsOverrideAttributePatchMetadataFlag[attributeIndex];  // u(1)
  //   if ( afpsOverrideAttributePatchMetadataFlag[attributeIndex] ) {
  //     afpsAttributePatchScaleMetadataEnabledFlag[attributeIndex];   // u(1)
  //     afpsAttributePatchOffsetMetadataEnabledFlag[attributeIndex];  // u(1)
  //   }
  // }
  // byteAlignment( bitstream );
}

// 7.3.20 Attribute frame metadata syntax
void PCCBitstreamDecoderNewSyntax::attributeFrameMetadata( PCCContext&   context,
                                                           PCCBitstream& bitstream,
                                                           size_t        attributeIndex,
                                                           size_t        attributeDimension ) {
  // afmAttributeSmoothingMetadataPresentFlag[attributeIndex];  // u(1)
  // afmAttributeScaleMetadataPresentFlag[attributeIndex];      // u(1)
  // afmAttributeOffsetMetadataPresentFlag[attributeIndex];     // u(1)
  // if ( afmAttributeSmoothingMetadataPresentFlag[attributeIndex] ) {
  //   afmAttributeSmoothingRadius[attributeIndex];                      // u(8)
  //   afmAttributeSmoothingNeighbourCount[attributeIndex];              // u(8)
  //   afmAttributeSmoothingRadius2BoundaryDetection[attributeIndex];    // u(8)
  //   afmAttributeSmoothingThreshold[attributeIndex];                   // u(8)
  //   afmAttributeSmoothingThresholdLocalEntropy[attributeIndex];       // u(3)
  // }
  // if ( afmAttributeScaleMetadataPresentFlag[attributeIndex] ) {
  //   for ( size_t i = 0; i < attributeDimension; i++ ) {
  //     afmAttributeScaleMetadata[attributeIndex][i];  // u(32)
  //   }
  // }
  // if ( afmAttributeOffsetMetadataPresentFlag[attributeIndex] ) {
  //   for ( size_t i = 0; i < attributeDimension; i++ ) {
  //     afmAttributeOffsetMetadata[attributeIndex][i];  // u(32)
  //   }
  // }
}

// 7.3.21 Geometry patch parameter set syntax
void PCCBitstreamDecoderNewSyntax::geometryPatchParameterSet( PCCContext&   context,
                                                              PCCBitstream& bitstream ) {
  // gppsGeometryPatchParameterSetId;  // ue(v)
  // gppsGeometryFrameParameterSetId;  // ue(v)
  // if ( gfpsGeometryPatchScaleMetadataEnabledFlag ||
  //      gfpsGeometryPatchOffsetMetadataEnabledFlag ||
  //      gfpsGeometryPatchRotationMetadataEnabledFlag ||
  //      gfpsGeometryPatchPointSizeMetadataEnabledFlag ||
  //      gfpsGeometryPatchPointShapeMetadataEnabledFlag ) {
  //   gppsGeometryPatchMetadataPresentFlag;  // u(1)
  //   if ( gppsGeometryPatchMetadataPresentFlag ) { geometryPatchMetadata(); }
  // }
  // byteAlignment( bitstream );
}

// 7.3.22 Geometry patch metadata syntax
void PCCBitstreamDecoderNewSyntax::geometryPatchMetadata( PCCContext&   context,
                                                          PCCBitstream& bitstream ) {
  // if ( gfpsGeometryPatchScaleMetadataEnabledFlag ) {
  //   gpmGeometryPatchScaleMetadataPresentFlag;  // u(1)
  //   if ( gpmGeometryPatchScaleMetadataPresentFlag ) {
  //     for ( size_t d = 0; d < 3; d++ ) {
  //       gpmGeometryPatchScaleMetadataOnAxis[d];  // u( 32 )
  //     }
  //   }
  // }
  // if ( gfpsGeometryPatchOffsetMetadataEnabledFlag ) {
  //   gpmGeometryPatchOffsetMetadataPresentFlag;  // u(1)
  //   if ( gpmGeometryPatchOffsetMetadataPresentFlag ) {
  //     for ( size_t d = 0; d < 3; d++ ) {
  //       gpmGeometryPatchOffsetMetadataOnAxis[d];  // i( 32 )
  //     }
  //   }
  // }
  // if ( gfpsGeometryPatchRotationMetadataEnabledFlag ) {
  //   gpmGeometryPatchRotationMetadataPresentFlag;  // u(1)
  //   if ( gpmGeometryPachRotationMetadataPresentFlag ) {
  //     for ( size_t d = 0; d < 3; d++ ) {
  //       gpmGeometryPatchRotationMetadataOnAxis[d];  // i( 32 )
  //     }
  //   }
  // }
  // if ( gfpsGeometryPatchPointSizeMetadataEnabledFlag ) {
  //   gpmGeometryPatchPointSizeMetadataPresentFlag;  // u(1)
  //   if ( gpmGeometryPatchPointSizeMetadataPresentFlag ) {
  //     gpmGeometryPatchPointSizeMetadata;  // u(16)
  //   }
  // }
  // if ( gfpsGeometryPatchPointShapeMetadataEnabledFlag ) {
  //   gpmGeometryPatchPointShapeMetadataPresentFlag;  // u(1)
  //   if ( gpmGeometryPatchPointShapeMetadataPresentFlag ) {
  //     gpmGeometryPatchPointShapeMetadata;  // u(8)
  //   }
  // }
}

// 7.3.23 Attribute patch parameter set syntax
void PCCBitstreamDecoderNewSyntax::attributePatchParameterSet( PCCContext&   context,
                                                               PCCBitstream& bitstream,
                                                               size_t        attributeIndex ) {
  // appsAttributePatchParameterSetId[attributeIndex];  // ue(v)
  // appsAttributeFrameParameterSetId[attributeIndex];  // ue(v)
  // attributeDimension = apsAttributeDimensionMinus1[attributeIndex] + 1;
  // if ( afpsAttributePatchScaleMetadataEnabledFlag[attributeIndex] ||
  //      afpsAttributePatchOffsetMetadataEnabledFlag[attributeIndex] ) {
  //   appsAttributePatchMetadataPresentFlag[attributeIndex];  // u(1)
  //   if ( appsAttributePatchMetadataPresentFlag[attributeIndex] ) {
  //     attributePatchMetadata( attributeIndex, attributeDimension );
  //   }
  // }
  // byteAlignment( bitstream );
}

// 7.3.24 Attribute patch metadata syntax
void PCCBitstreamDecoderNewSyntax::attributePatchMetadata( PCCContext&   context,
                                                           PCCBitstream& bitstream,
                                                           size_t        attributeIndex,
                                                           size_t        attributeDimension ) {
  // if ( afpsAttributePatchScaleMetadataEnabledFlag[attributeIndex] ) {
  //   apmAttributePatchScaleMetadataPresentFlag[attributeIndex];  // u(1)
  //   if ( apmAttributePatchScaleMetadataPresentFlag[attributeIndex] ) {
  //     for ( size_t i = 0; i < attributeDimension; i++ ) {
  //       apmAttributePatchScaleMetadata[attributeIndex][i];  // u( 32 )
  //     }
  //   }
  // }
  // if ( afpsAttributePatchOffsetMetadataEnabledFlag[attributeIndex] ) {
  //   apmAttributePatchOffsetMetadataPresentFlag[attributeIndex];  // u(1)
  //   if ( apmAttributePatchOffsetMetadataPresentFlag[attributeIndex] ) {
  //     for ( size_t i = 0; i < attributeDimension; i++ ) {
  //       apmAttributePatchOffsetMetadata[attributeIndex][i];  // i(32)
  //     }
  //   }
  // }
}

// 7.3.25 Patch frame parameter set syntax
void PCCBitstreamDecoderNewSyntax::patchFrameParameterSet( PCCContext&   context,
                                                           PCCBitstream& bitstream ) {
  // pfpsPatchFrameParameterSetId;     // ue(v)
  // pfpsPatchSequenceParameterSetId;  // ue(v)
  // // pfpsGeometryPatchFrameParameterSetId;  // ue(v)
  // // for ( i = 0; i < spsAttributeCount; i++ ) {
  // //   pfpsAttributePatchFrameParameterSetId[i]  // ue(v)
  // // }
  // pfpsLocalOverrideGeometryPatchEnableFlag;  // u(1)
  // for ( size_t i = 0; i < spsAttributeCount; i++ ) {
  //   pfpsLocalOverrideAttributePatchEnableFlag[i];  // u(1)
  // }
  // // pfpsNumRefIdxDefaultActiveMinus1 // ue(v)
  // pfpsAdditionalLtPfocLsbLen;  // ue(v)
  // if ( spsPatchSequenceOrientationEnabledFlag ) {
  //   pfpsPatchOrientationPresentFlag;  // u(1)
  // }
  // // Other?
  // byteAlignment( bitstream );
}

// 7.3.26 Patch frame layer unit syntax
void PCCBitstreamDecoderNewSyntax::patchFrameLayerUnit( PCCContext&   context,
                                                        PCCBitstream& bitstream,
                                                        size_t        frameIndex ) {
  // patchFrameHeader( frameIndex );
  // patchFrameDataUnit( frameIndex );
}

// 7.3.27 Patch frame header syntax
void PCCBitstreamDecoderNewSyntax::patchFrameHeader( PCCContext&   context,
                                                     PCCBitstream& bitstream,
                                                     size_t        frameIndex ) {
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
void PCCBitstreamDecoderNewSyntax::refListStruct( PCCContext&   context,
                                                  PCCBitstream& bitstream,
                                                  size_t        referenceListIndex ) {
  // numRefEntries[referenceListIndex];  // ue(v)
  // for ( size_t i = 0; i < numRefEntries[referenceListIndex]; i++ ) {
  //   if ( pspsLongTermRefPatchFramesFlag ) {
  //     stRefPatchFrameFlag[referenceListIndex][i];  // u(1)
  //     if ( stRefPatchFrameFlag[referenceListIndex][i] ) {
  //       absDeltaPfocSt[referenceListIndex][i];  // ue(v)
  //       if ( absDeltaPfocSt[referenceListIndex][i] > 0 ) {
  //         strpfEntrySignFlag[referenceListIndex][i];  // u(1)
  //       }
  //     } else {
  //       pfocLsbLt[referenceListIndex][i];  // u(v)
  //     }
  //   }
}

// 7.3.29 Patch frame data unit syntax
// void PCCBitstreamDecoderNewSyntax::patchFrameDataUnit( PCCContext&   context,
//                                                           PCCBitstream& bitstream,
//                                                           size_t        frameIndex ) {
//   // pfduPatchCountMinus1;  // u( 32 )
//   // if ( pfhType[frameIndex] != I ) {
//   //   pfduInterPatchCount;  // ae(v)
//   // } else {
//   //   pfduInterPatchCount = 0;
//   // }
//   // for ( size_t p = 0; p < pfduInterPatchCount; p++ ) {
//   //   pfduPatchMode[frameIndex][p] = PINTER;
//   //   patchInformationData( frameIndex, p, pfduPatchMode[frameIndex][p] );
//   // }
//   // for ( size_t p = pfduInterPatchCount; p <= pfduPatchCountMinus1; p++ ) {
//   //   pfduPatchMode[frameIndex][p] = ( pfhType[frameIndex] == I ) ? IINTRA : PINTRA;
//   //   patchInformationData( frameIndex, p, pfduPatchMode[frameIndex][p] );
//   // }
//   // if ( spsPcmPatchEnabledFlag ) {
//   //   pfduPatchMode[frameIndex][p + 1] = pfhType[frameIndex] == I ? IPCM : PPCM;
//   //   patchInformationData( frameIndex, p + 1, pfduPatchMode[frameIndex][p + 1] );
//   // }
//   // if ( spsPointLocalReconstructionEnabledFlag ) { pointLocalReconstruction(); }
//   // byteAlignment( bitstream );
// }

void PCCBitstreamDecoderNewSyntax::patchFrameDataUnit( PCCContext&   context,
                                                       PCCBitstream& bitstream,
                                                       size_t        frameIndex ) {
  // int patchIndex = -1 ;
  // pfduMorePatchesAvailableFlag;  // u(1)
  // while ( morePatchesAvailableFlag ) {
  //   p++;
  //   pfduPatchMode[frameIndex][patchIndex];  // ue(v)
  //   patchInformationData( frameIndex, p, pfduPatchMode[frameIndex][patchIndex] );
  //   pfduMorePatchesAvailableFlag;  // u(1)
  // }
  // PdfuTotalNumberOfPatches[frameIndex] = patchIndex + 1;
  // if ( spsPointLocalReconstructionEnabledFlag ) { pointLocalReconstruction(); }
  // byteAlignment( bitstream );
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
  //   patchDeltaDataUnit( frameIndex, patchIndex );
  // } else if ( patchMode == IPCM || patchMode == PPCM ) {
  //   pcmPatchDataUnit( frameIndex, patchIndex );
  // }
}

// 7.3.31 Patch data unit syntax
void PCCBitstreamDecoderNewSyntax::patchDataUnit( PCCContext&   context,
                                                  PCCBitstream& bitstream,
                                                  size_t        frameIndex,
                                                  size_t        patchIndex ) {
  // pdu2dShiftU[frameIndex][patchIndex];               // u(v)
  // pdu2dShiftV[frameIndex][patchIndex];               // u(v)
  // pdu2dDeltaSizeU[frameIndex][patchIndex];          // se( v )
  // pdu2dDeltaSizeV[frameIndex][patchIndex];          // se( v )
  // pdu3dShiftTangentAxis[frameIndex][patchIndex];    // u(v)
  // pdu3dShiftBitangentAxis[frameIndex][patchIndex];  // u(v)
  // pdu3dShiftNormalAxis[frameIndex][patchIndex];     // u(v)
  // pduNormalAxis[frameIndex][patchIndex];              // ae(v)
  // if ( pfhPatchOrientationPresentFlag[frameIndex] ) {
  //   pduOrientationIndex[frameIndex][patchIndex];  // ae(v)
  // }
  // if ( pfhPatchLodBitCount[frameIndex] > 0 ) {
  //    pduLod[frameIndex][patchIndex];  // u(v)
  // }
  // projectionFlag = 0;
  // i              = 0;
  // while ( i < spsLayerCountMinus1 + 1 && projectionFlag == 0 ) {
  //   projectionFlag = projectionFlag | spsLayerAbsoluteCodingEnabledFlag[i];
  //   if ( projectionFlag ) {
  //     pduProjectionMode[frameIndex][patchIndex]  // ae(v)
  //   }
  // }
}

// 7.3.32  Delta Patch data unit syntax
void PCCBitstreamDecoderNewSyntax::deltaPatchDataUnit( PCCContext&   context,
                                                       PCCBitstream& bitstream,
                                                       size_t        frameIndex,
                                                       size_t        patchIndex ) {
  // dpduPatchIndex[frameIndex][patchIndex];              // ae(v)
  // dpdu2dShiftU[frameIndex][patchIndex];               // ae(v)
  // dpdu2dShiftV[frameIndex][patchIndex];               // ae(v)
  // dpdu2dDeltaSizeU[frameIndex][patchIndex];          // ae(v)
  // dpdu2dDeltaSizeV[frameIndex][patchIndex];          // ae(v)
  // dpdu3dShiftTangentAxis[frameIndex][patchIndex];    // ae(v)
  // dpdu3dShiftBitangentAxis[frameIndex][patchIndex];  // ae(v)
  // dpdu3dShiftNormalAxis[frameIndex][patchIndex];     // ae(v)
}

// 7.3.33 PCM patch data unit syntax
void PCCBitstreamDecoderNewSyntax::pcmPatchDataUnit( PCCContext&   context,
                                                     PCCBitstream& bitstream,
                                                     size_t        frameIndex,
                                                     size_t        patchIndex ) {
  // if ( spsPcmSeparateVideoFlag ) {
  //   ppduPatchInPcmVideoFlag[frameIndex][patchIndex];  // u(1)
  //   ppdu2dShiftU[frameIndex][patchIndex];               // u(v)
  //   ppdu2dShiftV[frameIndex][patchIndex];               // u(v)
  //   ppdu2dSizeU[frameIndex][patchIndex];                // u(v)
  //   ppdu2dSizeV[frameIndex][patchIndex];                // u(v)
  //   ppduPcmPoints[frameIndex][patchIndex];               // ue(v)
  // }
}

// 7.3.34 Point local reconstruction syntax
void PCCBitstreamDecoderNewSyntax::pointLocalReconstruction( PCCContext&   context,
                                                             PCCBitstream& bitstream ) {
  // for ( i = 0; i < BlockCount; i++ ) {
  //   if ( blockToPatch[i] >= 0 ) {
  //     plrModeInterpolateFlag[i];                                   // ae(v)
  //     if ( plrModeInterpolateFlag[i] ) { plrModeNeighbour[i]; }  // ae(v)
  //     plrModeMinimumDepthMinus1[i];                               // ae(v)
  //     if ( plrMode minimumDepthMinus1[i] > 0 || plrModeInterpolateFlag[i] ) {
  //       plrModeFillingFlag[i]  // ae(v)
  //     }
  //   }
  // }
}
