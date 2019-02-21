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

#include "PCCBitstreamDecoder.h"

using namespace std;
using namespace pcc;

PCCBitstreamDecoder::PCCBitstreamDecoder() {}
PCCBitstreamDecoder::~PCCBitstreamDecoder() {}

int PCCBitstreamDecoder::decode( PCCBitstream& bitstream, PCCContext& context ) {
  if ( !decompressHeader( context, bitstream ) ) { return 0; }
  bitstream.read( context.createVideoBitstream( PCCVideoType::OccupancyMap ) );
  if ( !context.getAbsoluteD1() ) {
    bitstream.read( context.createVideoBitstream( PCCVideoType::GeometryD0 ) );
    bitstream.read( context.createVideoBitstream( PCCVideoType::GeometryD1 ) );
  } else {
    bitstream.read( context.createVideoBitstream( PCCVideoType::Geometry ) );
  }

  if ( context.getUseAdditionalPointsPatch() &&
       context.getSps().getPcmSeparateVideoPresentFlag() ) {
    bitstream.read( context.createVideoBitstream( PCCVideoType::GeometryMP ) );
  }
  if ( !context.getNoAttributes() ) {
    bitstream.read( context.createVideoBitstream( PCCVideoType::Texture ) );
    if ( context.getUseAdditionalPointsPatch() &&
         context.getSps().getPcmSeparateVideoPresentFlag() ) {
      bitstream.read( context.createVideoBitstream( PCCVideoType::TextureMP ) );
      auto sizeMissedPointsTexture = bitstream.size();
    }
  }

  decompressOccupancyMap( context, bitstream );

  if ( context.getUseAdditionalPointsPatch() &&
       context.getSps().getPcmSeparateVideoPresentFlag() ) {
    readMissedPointsGeometryNumber( context, bitstream );
  } else if ( context.getUseAdditionalPointsPatch() &&
              !context.getSps().getPcmSeparateVideoPresentFlag() ) {
    size_t numMissedPts{0};
    for ( auto& frame : context.getFrames() ) {
      frame.getMissedPointsPatch().numMissedPts_ = bitstream.read<size_t>();
    }
  }
  if ( !context.getNoAttributes() ) {
    if ( context.getUseAdditionalPointsPatch() &&
         context.getSps().getPcmSeparateVideoPresentFlag() ) {
      auto sizeMissedPointsTexture = bitstream.size();
      readMissedPointsTextureNumber( context, bitstream );
    }
  }
  return 1;
}

uint32_t PCCBitstreamDecoder::DecodeUInt32( const uint32_t           bitCount,
                                            o3dgc::Arithmetic_Codec& arithmeticDecoder,
                                            o3dgc::Static_Bit_Model& bModel0 ) {
  uint32_t decodedValue = 0;
  for ( uint32_t i = 0; i < bitCount; ++i ) {
    decodedValue += ( arithmeticDecoder.decode( bModel0 ) << i );
  }
  return PCCFromLittleEndian<uint32_t>( decodedValue );
}

int PCCBitstreamDecoder::readMetadata( PCCMetadata& metadata, PCCBitstream& bitstream ) {
  auto& metadataEnabledFlags = metadata.getMetadataEnabledFlags();
  if ( !metadataEnabledFlags.getMetadataEnabled() ) { return 0; }
  metadata.getMetadataPresent() = bitstream.read<uint8_t>();
  std::cout << "read/write METADATA TYPE: " << metadata.getMetadataType()
            << " METADATA present: " << metadata.getMetadataPresent() << std::endl;
  if ( metadata.getMetadataPresent() ) {
    if ( metadataEnabledFlags.getScaleEnabled() ) {
      metadata.getScalePresent() = bitstream.read<uint8_t>();
      if ( metadata.getScalePresent() ) { metadata.getScale() = bitstream.read<PCCVector3U>(); }
    }
    if ( metadataEnabledFlags.getOffsetEnabled() ) {
      metadata.getOffsetPresent() = bitstream.read<uint8_t>();
      if ( metadata.getOffsetPresent() ) { metadata.getOffset() = bitstream.read<PCCVector3I>(); }
    }
    if ( metadataEnabledFlags.getRotationEnabled() ) {
      metadata.getRotationPresent() = bitstream.read<uint8_t>();
      if ( metadata.getRotationPresent() ) {
        metadata.getRotation() = bitstream.read<PCCVector3I>();
      }
    }
    if ( metadataEnabledFlags.getPointSizeEnabled() ) {
      metadata.getPointSizePresent() = bitstream.read<uint8_t>();
      if ( metadata.getPointSizePresent() ) {
        metadata.getPointSize() = bitstream.read<uint16_t>();
      }
    }
    if ( metadataEnabledFlags.getPointShapeEnabled() ) {
      metadata.getPointShapePresent() = bitstream.read<uint8_t>();
      if ( metadata.getPointShapePresent() ) {
        metadata.getPointShape() = static_cast<pcc::PointShape>( bitstream.read<uint8_t>() );
      }
    }
  }
  auto& lowerLevelMetadataEnabledFlags = metadata.getLowerLevelMetadataEnabledFlags();
  lowerLevelMetadataEnabledFlags.getMetadataEnabled() = bitstream.read<uint8_t>();
  if ( lowerLevelMetadataEnabledFlags.getMetadataEnabled() ) {
    lowerLevelMetadataEnabledFlags.getScaleEnabled()      = bitstream.read<uint8_t>();
    lowerLevelMetadataEnabledFlags.getOffsetEnabled()     = bitstream.read<uint8_t>();
    lowerLevelMetadataEnabledFlags.getRotationEnabled()   = bitstream.read<uint8_t>();
    lowerLevelMetadataEnabledFlags.getPointSizeEnabled()  = bitstream.read<uint8_t>();
    lowerLevelMetadataEnabledFlags.getPointShapeEnabled() = bitstream.read<uint8_t>();
#ifdef CE210_MAXDEPTH_EVALUATION
    lowerLevelMetadataEnabledFlags.setMaxDepthEnabled( bitstream.read<uint8_t>(); );
#endif
  }
  return 1;
}

int PCCBitstreamDecoder::decompressMetadata( PCCMetadata&             metadata,
                                             o3dgc::Arithmetic_Codec& arithmeticDecoder ) {
  auto& metadataEnabingFlags = metadata.getMetadataEnabledFlags();
  if ( !metadataEnabingFlags.getMetadataEnabled() ) { return 0; }
  static o3dgc::Static_Bit_Model   bModel0;
  static o3dgc::Adaptive_Bit_Model bModelMetadataPresent;
  metadata.getMetadataPresent() = arithmeticDecoder.decode( bModelMetadataPresent );
  std::cout << "METADATA TYPE: " << metadata.getMetadataType() << " Index: " << metadata.getIndex()
            << " present: " << ( metadata.getMetadataPresent() == true ? 1 : 0 )
            << " lowerLevelFlags: "
            << metadata.getLowerLevelMetadataEnabledFlags().getMetadataEnabled() << std::endl;

  if ( metadata.getMetadataPresent() ) {
    if ( metadataEnabingFlags.getScaleEnabled() ) {
      static o3dgc::Adaptive_Bit_Model bModelScalePresent;
      metadata.getScalePresent() = arithmeticDecoder.decode( bModelScalePresent );
      if ( metadata.getScalePresent() ) {
        metadata.getScale()[0] = DecodeUInt32( 32, arithmeticDecoder, bModel0 );
        metadata.getScale()[1] = DecodeUInt32( 32, arithmeticDecoder, bModel0 );
        metadata.getScale()[2] = DecodeUInt32( 32, arithmeticDecoder, bModel0 );
      }
    }
    if ( metadataEnabingFlags.getOffsetEnabled() ) {
      static o3dgc::Adaptive_Bit_Model bModelOffsetPresent;
      metadata.getOffsetPresent() = arithmeticDecoder.decode( bModelOffsetPresent );
      if ( metadata.getOffsetPresent() ) {
        metadata.getOffset()[0] =
            (int32_t)o3dgc::UIntToInt( DecodeUInt32( 32, arithmeticDecoder, bModel0 ) );
        metadata.getOffset()[1] =
            (int32_t)o3dgc::UIntToInt( DecodeUInt32( 32, arithmeticDecoder, bModel0 ) );
        metadata.getOffset()[2] =
            (int32_t)o3dgc::UIntToInt( DecodeUInt32( 32, arithmeticDecoder, bModel0 ) );
      }
    }
    if ( metadataEnabingFlags.getRotationEnabled() ) {
      static o3dgc::Adaptive_Bit_Model bModelRotationPresent;
      metadata.getRotationPresent() = arithmeticDecoder.decode( bModelRotationPresent );
      if ( metadata.getRotationPresent() ) {
        metadata.getRotation()[0] =
            (int32_t)o3dgc::UIntToInt( DecodeUInt32( 32, arithmeticDecoder, bModel0 ) );
        metadata.getRotation()[1] =
            (int32_t)o3dgc::UIntToInt( DecodeUInt32( 32, arithmeticDecoder, bModel0 ) );
        metadata.getRotation()[2] =
            (int32_t)o3dgc::UIntToInt( DecodeUInt32( 32, arithmeticDecoder, bModel0 ) );
      }
    }
    if ( metadataEnabingFlags.getPointSizeEnabled() ) {
      static o3dgc::Adaptive_Bit_Model bModelPointSizePresent;
      metadata.getPointSizePresent() = arithmeticDecoder.decode( bModelPointSizePresent );
      if ( metadata.getPointSizePresent() ) {
        metadata.getPointSize() = DecodeUInt32( 16, arithmeticDecoder, bModel0 );
      }
    }
    if ( metadataEnabingFlags.getPointShapeEnabled() ) {
      static o3dgc::Adaptive_Bit_Model bModelPointShapePresent;
      metadata.getPointShapePresent() = arithmeticDecoder.decode( bModelPointShapePresent );
      if ( metadata.getPointShapePresent() ) {
        metadata.getPointShape() =
            static_cast<PointShape>( DecodeUInt32( 8, arithmeticDecoder, bModel0 ) );
      }
    }
  }

  auto& lowerLevelMetadataEnabledFlags = metadata.getLowerLevelMetadataEnabledFlags();
  static o3dgc::Adaptive_Bit_Model bModelLowerLevelMetadataEnabled;
  lowerLevelMetadataEnabledFlags.getMetadataEnabled() =
      arithmeticDecoder.decode( bModelLowerLevelMetadataEnabled );

  if ( lowerLevelMetadataEnabledFlags.getMetadataEnabled() ) {
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelScaleEnabled;
    lowerLevelMetadataEnabledFlags.getScaleEnabled() =
        arithmeticDecoder.decode( bModelLowerLevelScaleEnabled );
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelOffsetEnabled;
    lowerLevelMetadataEnabledFlags.getOffsetEnabled() =
        arithmeticDecoder.decode( bModelLowerLevelOffsetEnabled );
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelRotationEnabled;
    lowerLevelMetadataEnabledFlags.getRotationEnabled() =
        arithmeticDecoder.decode( bModelLowerLevelRotationEnabled );
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelPointSizeEnabled;
    lowerLevelMetadataEnabledFlags.getPointSizeEnabled() =
        arithmeticDecoder.decode( bModelLowerLevelPointSizeEnabled );
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelPointShapeEnabled;
    lowerLevelMetadataEnabledFlags.getPointShapeEnabled() =
        arithmeticDecoder.decode( bModelLowerLevelPointShapeEnabled );
#ifdef CE210_MAXDEPTH_EVALUATION
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelMaxDepthEnabled;
    auto tmp = arithmeticDecoder.decode( bModelLowerLevelMaxDepthEnabled );
    lowerLevelMetadataEnabledFlags.setMaxDepthEnabled( bool( tmp ) );
#endif
  }
  return 1;
}

int PCCBitstreamDecoder::decompressMetadata( PCCMetadata&               metadata,
                                             o3dgc::Arithmetic_Codec&   arithmeticDecoder,
                                             o3dgc::Static_Bit_Model&   bModelMaxDepth0,
                                             o3dgc::Adaptive_Bit_Model& bModelMaxDepthDD ) {
  auto& metadataEnabingFlags = metadata.getMetadataEnabledFlags();
  if ( !metadataEnabingFlags.getMetadataEnabled() ) { return 0; }

  static o3dgc::Static_Bit_Model   bModel0;
  static o3dgc::Adaptive_Bit_Model bModelMetadataPresent;
  metadata.getMetadataPresent() = arithmeticDecoder.decode( bModelMetadataPresent );

  if ( metadata.getMetadataPresent() ) {
    if ( metadataEnabingFlags.getScaleEnabled() ) {
      static o3dgc::Adaptive_Bit_Model bModelScalePresent;
      metadata.getScalePresent() = arithmeticDecoder.decode( bModelScalePresent );
      if ( metadata.getScalePresent() ) {
        metadata.getScale()[0] = DecodeUInt32( 32, arithmeticDecoder, bModel0 );
        metadata.getScale()[1] = DecodeUInt32( 32, arithmeticDecoder, bModel0 );
        metadata.getScale()[2] = DecodeUInt32( 32, arithmeticDecoder, bModel0 );
      }
    }
    if ( metadataEnabingFlags.getOffsetEnabled() ) {
      static o3dgc::Adaptive_Bit_Model bModelOffsetPresent;
      metadata.getOffsetPresent() = arithmeticDecoder.decode( bModelOffsetPresent );
      if ( metadata.getOffsetPresent() ) {
        metadata.getOffset()[0] =
            (int32_t)o3dgc::UIntToInt( DecodeUInt32( 32, arithmeticDecoder, bModel0 ) );
        metadata.getOffset()[1] =
            (int32_t)o3dgc::UIntToInt( DecodeUInt32( 32, arithmeticDecoder, bModel0 ) );
        metadata.getOffset()[2] =
            (int32_t)o3dgc::UIntToInt( DecodeUInt32( 32, arithmeticDecoder, bModel0 ) );
      }
    }
    if ( metadataEnabingFlags.getRotationEnabled() ) {
      static o3dgc::Adaptive_Bit_Model bModelRotationPresent;
      metadata.getRotationPresent() = arithmeticDecoder.decode( bModelRotationPresent );
      if ( metadata.getRotationPresent() ) {
        metadata.getRotation()[0] =
            (int32_t)o3dgc::UIntToInt( DecodeUInt32( 32, arithmeticDecoder, bModel0 ) );
        metadata.getRotation()[1] =
            (int32_t)o3dgc::UIntToInt( DecodeUInt32( 32, arithmeticDecoder, bModel0 ) );
        metadata.getRotation()[2] =
            (int32_t)o3dgc::UIntToInt( DecodeUInt32( 32, arithmeticDecoder, bModel0 ) );
      }
    }
    if ( metadataEnabingFlags.getPointSizeEnabled() ) {
      static o3dgc::Adaptive_Bit_Model bModelPointSizePresent;
      metadata.getPointSizePresent() = arithmeticDecoder.decode( bModelPointSizePresent );
      if ( metadata.getPointSizePresent() ) {
        metadata.getPointSize() = DecodeUInt32( 16, arithmeticDecoder, bModel0 );
      }
    }
    if ( metadataEnabingFlags.getPointShapeEnabled() ) {
      static o3dgc::Adaptive_Bit_Model bModelPointShapePresent;
      metadata.getPointShapePresent() = arithmeticDecoder.decode( bModelPointShapePresent );
      if ( metadata.getPointShapePresent() ) {
        metadata.getPointShape() =
            static_cast<PointShape>( DecodeUInt32( 8, arithmeticDecoder, bModel0 ) );
      }
    }
    int64_t       tempcurrentDD = 0;
    const uint8_t maxBitCountForMaxDepth =
        metadata.getbitCountQDepth();  // uint8_t(9-gbitCountSize[minLevel]);
#ifdef CE210_MAXDEPTH_EVALUATION
    if ( metadataEnabingFlags.getMaxDepthEnabled() ) {
      // size_t currentDD=metadata.getQMaxDepthInPatch();
      if ( maxBitCountForMaxDepth == 0 ) {  // delta_DD
        const int64_t delta_DD = o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode(
            0, bModelMaxDepth0, bModelMaxDepthDD ) );  // currentDD is delta_DD
        tempcurrentDD          = delta_DD;
        metadata.setQMaxDepthInPatch( int64_t( delta_DD ) );  // add 20190129
      } else {
        size_t currentDD =
            DecodeUInt32( maxBitCountForMaxDepth, arithmeticDecoder, bModelMaxDepth0 );
        metadata.setQMaxDepthInPatch( int64_t( currentDD ) );
        tempcurrentDD = currentDD;
      }
    }
#endif
  }

  if ( metadata.getMetadataType() == METADATA_PATCH ) {
    metadata.getLowerLevelMetadataEnabledFlags().setMetadataEnabled( false );
    // std::cout<<"lowerLevelFlags:
    // "<<metadata.getLowerLevelMetadataEnabledFlags().getMetadataEnabled()<<std::endl;
    return 1;
  }
  auto& lowerLevelMetadataEnabledFlags = metadata.getLowerLevelMetadataEnabledFlags();
  static o3dgc::Adaptive_Bit_Model bModelLowerLevelMetadataEnabled;
  lowerLevelMetadataEnabledFlags.getMetadataEnabled() =
      arithmeticDecoder.decode( bModelLowerLevelMetadataEnabled );

  if ( lowerLevelMetadataEnabledFlags.getMetadataEnabled() ) {
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelScaleEnabled;
    lowerLevelMetadataEnabledFlags.getScaleEnabled() =
        arithmeticDecoder.decode( bModelLowerLevelScaleEnabled );
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelOffsetEnabled;
    lowerLevelMetadataEnabledFlags.getOffsetEnabled() =
        arithmeticDecoder.decode( bModelLowerLevelOffsetEnabled );
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelRotationEnabled;
    lowerLevelMetadataEnabledFlags.getRotationEnabled() =
        arithmeticDecoder.decode( bModelLowerLevelRotationEnabled );
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelPointSizeEnabled;
    lowerLevelMetadataEnabledFlags.getPointSizeEnabled() =
        arithmeticDecoder.decode( bModelLowerLevelPointSizeEnabled );
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelPointShapeEnabled;
    lowerLevelMetadataEnabledFlags.getPointShapeEnabled() =
        arithmeticDecoder.decode( bModelLowerLevelPointShapeEnabled );
#ifdef CE210_MAXDEPTH_EVALUATION
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelMaxDepthEnabled;
    auto tmp = arithmeticDecoder.decode( bModelLowerLevelMaxDepthEnabled );
    lowerLevelMetadataEnabledFlags.setMaxDepthEnabled( bool( tmp ) );
#endif
  }
  return 1;
}
int PCCBitstreamDecoder::decompressHeader( PCCContext& context, PCCBitstream& bitstream ) {
  auto& sps = context.getSps();
  auto& gps = sps.getGeometryParameterSet();
  auto& gsp = gps.getGeometrySequenceParams();
  auto& ops = sps.getOccupancyParameterSet();
  sps.getAttributeCount() = 1;
  sps.allocateAttributeParameterSets();
  auto& aps = sps.getAttributeParameterSets( 0 );
  auto& asp = aps.getAttributeSequenceParams();

  uint8_t groupOfFramesSize = bitstream.read<uint8_t>();
  if ( !groupOfFramesSize ) { return 0; }
  context.resize( groupOfFramesSize );
  sps.getWidth()                  = bitstream.read<uint16_t>();
  sps.getHeight()                 = bitstream.read<uint16_t>();
  ops.getPackingBlockSize()       = bitstream.read<uint8_t>();
  context.getOccupancyPrecision() = bitstream.read<uint8_t>();
  gsp.getSmoothingPresentFlag()   = bitstream.read<uint8_t>();
  if ( gsp.getSmoothingPresentFlag() ) {
    context.getGridSmoothing() = bitstream.read<uint8_t>() > 0;
    if ( context.getGridSmoothing() ) {
      gsp.getSmoothingGridSize()  = bitstream.read<uint8_t>();
      gsp.getSmoothingThreshold() = bitstream.read<uint8_t>();
    } else {
      asp.getSmoothingParamsPresentFlag()        = bitstream.read<uint8_t>();
      asp.getSmoothingNeighbourCount()           = bitstream.read<uint8_t>();
      asp.getSmoothingRadius2BoundaryDetection() = bitstream.read<uint8_t>();
      gsp.getSmoothingThreshold()                = bitstream.read<uint8_t>();
    }
  }
  context.getLosslessGeo()             = bitstream.read<uint8_t>();
  context.getLosslessTexture()         = bitstream.read<uint8_t>();
  context.getNoAttributes()            = bitstream.read<uint8_t>();
  context.getLosslessGeo444()          = bitstream.read<uint8_t>();
  context.getMinLevel()                = bitstream.read<uint8_t>();
  sps.getPcmSeparateVideoPresentFlag() = bitstream.read<uint8_t>();
  context.getAbsoluteD1()              = bitstream.read<uint8_t>() > 0;
  if ( context.getAbsoluteD1() ) { context.getSixDirectionMode() = bitstream.read<uint8_t>() > 0; }

  context.getBinArithCoding() = bitstream.read<uint8_t>() > 0;
  context.getModelScale()     = bitstream.read<float>();
  context.getModelOrigin()    = bitstream.read<PCCVector3<float> >();
  readMetadata( context.getGOFLevelMetadata(), bitstream );
  asp.getSmoothingParamsPresentFlag() = bitstream.read<uint8_t>();
  if ( asp.getSmoothingParamsPresentFlag() ) {
    asp.getSmoothingThreshold()             = bitstream.read<uint8_t>();
    asp.getSmoothingThresholdLocalEntropy() = bitstream.read<double>();
    asp.getSmoothingRadius()                = bitstream.read<uint8_t>();
    asp.getSmoothingNeighbourCount()        = bitstream.read<uint8_t>();
  }
  sps.getEnhancedOccupancyMapForDepthFlag() = false;
  if ( context.getLosslessGeo() ) {
    sps.getEnhancedOccupancyMapForDepthFlag() = bitstream.read<uint8_t>() > 0;
    context.getImproveEDD()                   = bitstream.read<uint8_t>() > 0;
  }
  context.getDeltaCoding()                 = bitstream.read<uint8_t>() > 0;
  context.getRemoveDuplicatePoints()       = bitstream.read<uint8_t>() > 0;
  sps.getMultipleLayerStreamsPresentFlag() = bitstream.read<uint8_t>() > 0;
  sps.getPixelInterleavingFlag()           = bitstream.read<uint8_t>() > 0;
  context.getUseAdditionalPointsPatch()    = bitstream.read<uint8_t>() > 0;
  context.getGlobalPatchAllocation()       = bitstream.read<uint8_t>() > 0;

  context.getMPGeoWidth()  = 64;
  context.getMPAttWidth()  = 64;
  context.getMPGeoHeight() = 0;
  context.getMPAttHeight() = 0;
  auto& frames             = context.getFrames();
  for ( size_t i = 0; i < frames.size(); i++ ) {
    frames[i].setLosslessGeo( context.getLosslessGeo() );
    frames[i].setLosslessGeo444( context.getLosslessGeo444() );
    frames[i].setLosslessTexture( context.getLosslessTexture() );
    frames[i].setEnhancedDeltaDepth( sps.getEnhancedOccupancyMapForDepthFlag() );
    frames[i].setUseMissedPointsSeparateVideo( sps.getPcmSeparateVideoPresentFlag() );
    frames[i].setUseAdditionalPointsPatch( context.getUseAdditionalPointsPatch() );
  }
  return 1;
}

void PCCBitstreamDecoder::readMissedPointsGeometryNumber( PCCContext&   context,
                                                          PCCBitstream& bitstream ) {
  size_t maxHeight = 0;
  size_t MPwidth   = bitstream.read<size_t>();
  for ( auto& framecontext : context.getFrames() ) {
    size_t numofMPs = bitstream.read<size_t>();
    framecontext.getMissedPointsPatch().setMPnumber( size_t( numofMPs ) );
    if ( context.getLosslessGeo444() ) {
      framecontext.getMissedPointsPatch().resize( numofMPs );
    } else {
      framecontext.getMissedPointsPatch().resize( numofMPs * 3 );
    }
    size_t height    = ( 3 * numofMPs ) / MPwidth + 1;
    size_t heightby8 = height / 8;
    if ( heightby8 * 8 != height ) { height = ( heightby8 + 1 ) * 8; }
    maxHeight = ( std::max )( maxHeight, height );
  }
  context.getMPGeoWidth()  = size_t( MPwidth );
  context.getMPGeoHeight() = size_t( maxHeight );
}

void PCCBitstreamDecoder::readMissedPointsTextureNumber( PCCContext&   context,
                                                         PCCBitstream& bitstream ) {
  size_t maxHeight = 0;
  size_t MPwidth   = bitstream.read<size_t>();
  for ( auto& framecontext : context.getFrames() ) {
    size_t numofMPs = bitstream.read<size_t>();
    framecontext.getMissedPointsPatch().setMPnumbercolor( size_t( numofMPs ) );
    framecontext.getMissedPointsPatch().resizecolor( numofMPs );
    size_t height    = numofMPs / MPwidth + 1;
    size_t heightby8 = height / 8;
    if ( heightby8 * 8 != height ) { height = ( heightby8 + 1 ) * 8; }
    maxHeight = ( std::max )( maxHeight, height );
  }
  context.getMPAttWidth()  = size_t( MPwidth );
  context.getMPAttHeight() = size_t( maxHeight );
}

void PCCBitstreamDecoder::decompressOccupancyMap( PCCContext& context, PCCBitstream& bitstream ) {
  size_t sizeFrames = context.getFrames().size();
  auto   preFrame   = context.getFrames()[0];
  auto&  sps        = context.getSps();
  for ( int i = 0; i < sizeFrames; i++ ) {
    auto& frame       = context.getFrames()[i];
    frame.getWidth()  = sps.getWidth();
    frame.getHeight() = sps.getHeight();
    auto& frameLevelMetadataEnabledFlags =
        context.getGOFLevelMetadata().getLowerLevelMetadataEnabledFlags();
    frame.getFrameLevelMetadata().getMetadataEnabledFlags() = frameLevelMetadataEnabledFlags;
    decompressOccupancyMap( context, frame, bitstream, preFrame, i );

    if ( context.getUseAdditionalPointsPatch() && !sps.getPcmSeparateVideoPresentFlag() ) {
      if ( !sps.getPcmSeparateVideoPresentFlag() ) {
        auto& patches           = frame.getPatches();
        auto& missedPointsPatch = frame.getMissedPointsPatch();
        if ( context.getUseAdditionalPointsPatch() ) {
          const size_t patchIndex                = patches.size();
          PCCPatch&    dummyPatch                = patches[patchIndex - 1];
          missedPointsPatch.u0_                  = dummyPatch.getU0();
          missedPointsPatch.v0_                  = dummyPatch.getV0();
          missedPointsPatch.sizeU0_              = dummyPatch.getSizeU0();
          missedPointsPatch.sizeV0_              = dummyPatch.getSizeV0();
          missedPointsPatch.occupancyResolution_ = dummyPatch.getOccupancyResolution();
          patches.pop_back();
        }
      }
    }
    preFrame = frame;
  }
}

void PCCBitstreamDecoder::decompressPatchMetaDataM42195( PCCContext&              context,
                                                         PCCFrameContext&         frame,
                                                         PCCFrameContext&         preFrame,
                                                         PCCBitstream&            bitstream,
                                                         o3dgc::Arithmetic_Codec& arithmeticDecoder,
                                                         o3dgc::Static_Bit_Model& bModel0,
                                                         uint32_t& compressedBitstreamSize,
                                                         size_t    occupancyPrecision,
                                                         uint8_t   enable_flexible_patch_flag ) {
  auto&   sps        = context.getSps();
  auto&   ops        = sps.getOccupancyParameterSet();
  auto&   patches    = frame.getPatches();
  auto&   prePatches = preFrame.getPatches();
  size_t  patchCount = patches.size();
  uint8_t bitCount[6];
  uint8_t F = 0, A[5] = {0, 0, 0, 0, 0};
  size_t  topNmax[6] = {0, 0, 0, 0, 0, 0};

  const size_t  minLevel               = context.getMinLevel();
  const uint8_t maxBitCountForMinDepth = uint8_t( 10 - gbitCountSize[minLevel] );
  bitCount[4]                          = maxBitCountForMinDepth;
  const uint8_t maxBitCountForMaxDepth = uint8_t( 9 - gbitCountSize[minLevel] );
  bitCount[5]                          = maxBitCountForMaxDepth;
  compressedBitstreamSize              = bitstream.read<uint32_t>();
  assert( compressedBitstreamSize + bitstream.size() <= bitstream.capacity() );
  arithmeticDecoder.set_buffer( uint32_t( bitstream.capacity() - bitstream.size() ),
                                bitstream.buffer() + bitstream.size() );

  bool bBinArithCoding = context.getBinArithCoding() && ( !context.getLosslessGeo() ) &&
                         ( ops.getPackingBlockSize() == 16 ) && ( occupancyPrecision == 4 );

  arithmeticDecoder.start_decoder();
  o3dgc::Adaptive_Bit_Model bModelPatchIndex, bModelU0, bModelV0, bModelU1, bModelV1, bModelD1,
      bModelIntSizeU0, bModelIntSizeV0;
  o3dgc::Adaptive_Bit_Model  bModelSizeU0, bModelSizeV0, bModelAbsoluteD1;
  o3dgc::Adaptive_Bit_Model  orientationModel2;
  o3dgc::Adaptive_Data_Model orientationModel( 4 );
  o3dgc::Adaptive_Data_Model orientationPatchModel( NumPatchOrientations - 1 + 2 );
  o3dgc::Adaptive_Bit_Model  orientationPatchFlagModel2;
  o3dgc::Adaptive_Bit_Model  interpolateModel, fillingModel, occupiedModel;
  o3dgc::Adaptive_Data_Model minD1Model( 3 ), neighborModel( 3 );

  frame.getFrameLevelMetadata().setMetadataType( METADATA_FRAME );
  frame.getFrameLevelMetadata().setIndex( frame.getIndex() );
  decompressMetadata( frame.getFrameLevelMetadata(), arithmeticDecoder );

  int64_t       prevSizeU0 = 0;
  int64_t       prevSizeV0 = 0;
  uint32_t      numMatchedPatches;
  const uint8_t bitMatchedPatchCount =
      uint8_t( PCCGetNumberOfBitsInFixedLengthRepresentation( uint32_t( patchCount ) ) );
  numMatchedPatches = DecodeUInt32( bitMatchedPatchCount, arithmeticDecoder, bModel0 );
  F                 = uint8_t( DecodeUInt32( 1, arithmeticDecoder, bModel0 ) );
  if ( F ) {
    uint8_t flag = uint8_t( DecodeUInt32( 4, arithmeticDecoder, bModel0 ) );

    for ( int i = 0; i < 4; i++ ) {
      A[3 - i] = flag & 1;
      flag     = flag >> 1;
    }
    for ( int i = 0; i < 4; i++ ) {
      if ( A[i] ) bitCount[i] = uint8_t( DecodeUInt32( 8, arithmeticDecoder, bModel0 ) );
    }
  }
  if ( printDetailedInfo ) {
    printf( "numPatch:%d(%d), numMatchedPatches:%d, F:%d,A[4]:%d,%d,%d,%d\n", (int)patchCount,
            (int)bitMatchedPatchCount, (int)numMatchedPatches, F, A[0], A[1], A[2], A[3] );
  }

  int64_t predIndex = 0;
  for ( size_t patchIndex = 0; patchIndex < numMatchedPatches; ++patchIndex ) {
    auto& patch                    = patches[patchIndex];
    patch.getOccupancyResolution() = ops.getPackingBlockSize();
    int64_t delta_index =
        o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel0, bModelPatchIndex ) );
    patch.setBestMatchIdx() = ( size_t )( delta_index + predIndex );
    predIndex += ( delta_index + 1 );

    const auto&   prePatch = prePatches[patch.getBestMatchIdx()];
    const int64_t delta_U0 =
        o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel0, bModelU0 ) );
    const int64_t delta_V0 =
        o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel0, bModelV0 ) );
    const int64_t delta_U1 =
        o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel0, bModelU1 ) );
    const int64_t delta_V1 =
        o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel0, bModelV1 ) );
    const int64_t delta_D1 =
        o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel0, bModelD1 ) );
    const int64_t deltaSizeU0 =
        o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel0, bModelIntSizeU0 ) );
    const int64_t deltaSizeV0 =
        o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel0, bModelIntSizeV0 ) );

    if ( context.getSixDirectionMode() && context.getAbsoluteD1() ) {
      patch.getProjectionMode() = arithmeticDecoder.decode( bModel0 );
    } else {
      patch.getProjectionMode() = 0;
    }

    patch.getU0()               = delta_U0 + prePatch.getU0();
    patch.getV0()               = delta_V0 + prePatch.getV0();
    patch.getPatchOrientation() = prePatch.getPatchOrientation();
    patch.getU1()               = delta_U1 + prePatch.getU1();
    patch.getV1()               = delta_V1 + prePatch.getV1();
    size_t currentD1            = 0;
    size_t prevD1               = prePatch.getD1();
    if ( patch.getProjectionMode() == 0 ) {
      prevD1        = prevD1 / minLevel;
      currentD1     = ( prevD1 - delta_D1 );
      patch.getD1() = ( delta_D1 + prevD1 ) * minLevel;
    } else {
      prevD1    = ( 1024 - prevD1 ) / minLevel;
      currentD1 = ( delta_D1 + prevD1 );
      patch.getD1() =
          1024 - currentD1 * minLevel;  //(delta_D1 + prePatch.getD1()/minLevel)*minLevel;
    }

    patch.getSizeU0() = deltaSizeU0 + prePatch.getSizeU0();
    patch.getSizeV0() = deltaSizeV0 + prePatch.getSizeV0();

    // get maximum
    topNmax[0] = topNmax[0] < patch.getU0() ? patch.getU0() : topNmax[0];
    topNmax[1] = topNmax[1] < patch.getV0() ? patch.getV0() : topNmax[1];
    topNmax[2] = topNmax[2] < patch.getU1() ? patch.getU1() : topNmax[2];
    topNmax[3] = topNmax[3] < patch.getV1() ? patch.getV1() : topNmax[3];
    size_t D1  = patch.getD1() / minLevel;
    topNmax[4] = topNmax[4] < D1 ? D1 : topNmax[4];

    prevSizeU0 = patch.getSizeU0();
    prevSizeV0 = patch.getSizeV0();

    patch.getNormalAxis()    = prePatch.getNormalAxis();
    patch.getTangentAxis()   = prePatch.getTangentAxis();
    patch.getBitangentAxis() = prePatch.getBitangentAxis();

    if ( printDetailedInfo ) { patch.printDecoder(); }
  }

  // read info from metadata and resconstruc maxDepth
  o3dgc::Adaptive_Bit_Model bModelDD;
  for ( size_t patchIndex = 0; patchIndex < numMatchedPatches; ++patchIndex ) {
    auto& patch = patches[patchIndex];
    auto& patchLevelMetadataEnabledFlags =
        frame.getFrameLevelMetadata().getLowerLevelMetadataEnabledFlags();
    auto& patchLevelMetadata                     = patch.getPatchLevelMetadata();
    patchLevelMetadata.getMetadataEnabledFlags() = patchLevelMetadataEnabledFlags;
    patchLevelMetadata.setIndex( patchIndex );
    patchLevelMetadata.setMetadataType( METADATA_PATCH );
    patchLevelMetadata.setbitCountQDepth( 0 );  // added 20190129
    decompressMetadata( patchLevelMetadata, arithmeticDecoder, bModel0, bModelDD );

#ifdef CE210_MAXDEPTH_EVALUATION
    const int64_t delta_DD = patchLevelMetadata.getQMaxDepthInPatch();
#else
    const int64_t delta_DD = 0;
#endif
    const auto& prePatch = prePatches[patch.getBestMatchIdx()];
    size_t      currentDD;
    size_t      prevDD = prePatch.getSizeD() / minLevel;
    if ( prevDD * minLevel != prePatch.getSizeD() ) prevDD += 1;
    currentDD        = ( delta_DD + prevDD ) * minLevel;
    patch.getSizeD() = currentDD;
  }

  // Get Bitcount.
  for ( int i = 0; i < 4; i++ ) {
    if ( A[i] == 0 )
      bitCount[i] =
          uint8_t( PCCGetNumberOfBitsInFixedLengthRepresentation( uint32_t( topNmax[i] + 1 ) ) );
  }

  for ( size_t patchIndex = numMatchedPatches; patchIndex < patchCount; ++patchIndex ) {
    auto& patch                    = patches[patchIndex];
    patch.getOccupancyResolution() = ops.getPackingBlockSize();
    patch.getU0()                  = DecodeUInt32( bitCount[0], arithmeticDecoder, bModel0 );
    patch.getV0()                  = DecodeUInt32( bitCount[1], arithmeticDecoder, bModel0 );
    if ( enable_flexible_patch_flag ) {
      bool flexible_patch_present_flag = arithmeticDecoder.decode( orientationPatchFlagModel2 );
      if ( flexible_patch_present_flag ) {
        patch.getPatchOrientation() = arithmeticDecoder.decode( orientationPatchModel ) + 1;
      } else {
        patch.getPatchOrientation() = PatchOrientation::DEFAULT;
      }
    } else {
      patch.getPatchOrientation() = PatchOrientation::DEFAULT;
    }
    patch.getU1() = DecodeUInt32( bitCount[2], arithmeticDecoder, bModel0 );
    patch.getV1() = DecodeUInt32( bitCount[3], arithmeticDecoder, bModel0 );
    size_t D1     = DecodeUInt32( bitCount[4], arithmeticDecoder, bModel0 );

    if ( context.getSixDirectionMode() && context.getAbsoluteD1() ) {
      patch.getProjectionMode() = arithmeticDecoder.decode( bModel0 );
    } else {
      patch.getProjectionMode() = 0;
    }
    if ( patch.getProjectionMode() == 0 ) {
      patch.getD1() = D1 * minLevel;
    } else {
      patch.getD1() = ( 1024 - D1 * minLevel );
    }

    const int64_t deltaSizeU0 =
        o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel0, bModelSizeU0 ) );
    const int64_t deltaSizeV0 =
        o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel0, bModelSizeV0 ) );

    patch.getSizeU0() = prevSizeU0 + deltaSizeU0;
    patch.getSizeV0() = prevSizeV0 + deltaSizeV0;

    prevSizeU0 = patch.getSizeU0();
    prevSizeV0 = patch.getSizeV0();

    if ( bBinArithCoding ) {
      size_t bit0 = arithmeticDecoder.decode( orientationModel2 );
      if ( bit0 == 0 ) {  // 0
        patch.getNormalAxis() = 0;
      } else {
        size_t bit1 = arithmeticDecoder.decode( bModel0 );
        if ( bit1 == 0 ) {  // 10
          patch.getNormalAxis() = 1;
        } else {  // 11
          patch.getNormalAxis() = 2;
        }
      }
    } else {
      patch.getNormalAxis() = arithmeticDecoder.decode( orientationModel );
    }
    if ( patch.getNormalAxis() == 0 ) {
      patch.getTangentAxis()   = 2;
      patch.getBitangentAxis() = 1;
    } else if ( patch.getNormalAxis() == 1 ) {
      patch.getTangentAxis()   = 2;
      patch.getBitangentAxis() = 0;
    } else {
      patch.getTangentAxis()   = 0;
      patch.getBitangentAxis() = 1;
    }
    auto& patchLevelMetadataEnabledFlags =
        frame.getFrameLevelMetadata().getLowerLevelMetadataEnabledFlags();
    auto& patchLevelMetadata                     = patch.getPatchLevelMetadata();
    patchLevelMetadata.getMetadataEnabledFlags() = patchLevelMetadataEnabledFlags;
    patchLevelMetadata.setIndex( patchIndex );
    patchLevelMetadata.setMetadataType( METADATA_PATCH );
    patchLevelMetadata.getMetadataEnabledFlags() = patchLevelMetadataEnabledFlags;
    patchLevelMetadata.setbitCountQDepth( maxBitCountForMaxDepth );

    decompressMetadata( patchLevelMetadata, arithmeticDecoder, bModel0, bModelDD );

    // maxdepth reconstruction
#ifdef CE210_MAXDEPTH_EVALUATION
    size_t DD        = size_t( patchLevelMetadata.getQMaxDepthInPatch() );
    patch.getSizeD() = (DD)*minLevel;
#else
    patch.getSizeD()       = minLevel;
#endif

    if ( printDetailedInfo ) { patch.printDecoder(); }
  }

  for ( size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex ) {
    auto& patch = patches[patchIndex];
    decompressOneLayerData( context, frame, patch, arithmeticDecoder, occupiedModel,
                            interpolateModel, neighborModel, minD1Model, fillingModel );
  }
}

void PCCBitstreamDecoder::decompressOneLayerData( PCCContext&                 context,
                                                  PCCFrameContext&            frame,
                                                  PCCPatch&                   patch,
                                                  o3dgc::Arithmetic_Codec&    arithmeticDecoder,
                                                  o3dgc::Adaptive_Bit_Model&  occupiedModel,
                                                  o3dgc::Adaptive_Bit_Model&  interpolateModel,
                                                  o3dgc::Adaptive_Data_Model& neighborModel,
                                                  o3dgc::Adaptive_Data_Model& minD1Model,
                                                  o3dgc::Adaptive_Bit_Model&  fillingModel ) {
  auto& sps = context.getSps();
  auto& ops = sps.getOccupancyParameterSet();
  if ( context.getAbsoluteD1() && !sps.getMultipleLayerStreamsPresentFlag() ) {
    auto&        interpolateMap     = frame.getInterpolate();
    auto&        fillingMap         = frame.getFilling();
    auto&        minD1Map           = frame.getMinD1();
    auto&        neighborMap        = frame.getNeighbor();
    const size_t blockToPatchWidth  = frame.getWidth() / ops.getPackingBlockSize();
    const size_t blockToPatchHeight = frame.getHeight() / ops.getPackingBlockSize();
    for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
      for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
        int pos =
            patch.patchBlock2CanvasBlock( ( u0 ), ( v0 ), blockToPatchWidth, blockToPatchHeight );
        uint32_t occupied = arithmeticDecoder.decode( occupiedModel );
        if ( occupied ) {
          interpolateMap[pos] = (bool)arithmeticDecoder.decode( interpolateModel );
          if ( interpolateMap[pos] > 0 ) {
            uint32_t code    = arithmeticDecoder.decode( neighborModel );
            neighborMap[pos] = size_t( code + 1 );
          }
          minD1Map[pos] = (size_t)arithmeticDecoder.decode( minD1Model );
          if ( minD1Map[pos] > 1 || interpolateMap[pos] > 0 ) {
            fillingMap[pos] = (bool)arithmeticDecoder.decode( fillingModel );
          }
        }
      }
    }
  }
}

void PCCBitstreamDecoder::decompressOccupancyMap( PCCContext&      context,
                                                  PCCFrameContext& frame,
                                                  PCCBitstream&    bitstream,
                                                  PCCFrameContext& preFrame,
                                                  size_t           frameIndex ) {
  auto&    sps        = context.getSps();
  auto&    gps        = sps.getGeometryParameterSet();
  auto&    gsp        = gps.getGeometrySequenceParams();
  auto&    ops        = sps.getOccupancyParameterSet();
  auto&    patches    = frame.getPatches();
  uint32_t patchCount = bitstream.read<uint32_t>();
  patches.resize( patchCount );
  const size_t  minLevel               = context.getMinLevel();
  const uint8_t maxBitCountForMinDepth = uint8_t( 10 - gbitCountSize[minLevel] );
  const uint8_t maxBitCountForMaxDepth = uint8_t( 9 - gbitCountSize[minLevel] );

  frame.allocOneLayerData( ops.getPackingBlockSize() );
  size_t  maxCandidateCount   = bitstream.read<uint8_t>();
  uint8_t frameProjectionMode = 0, surfaceThickness = 4;
  if ( !context.getAbsoluteD1() ) {
    surfaceThickness    = bitstream.read<uint8_t>();
    frameProjectionMode = bitstream.read<uint8_t>();
  }
  frame.setSurfaceThickness( surfaceThickness );

  o3dgc::Arithmetic_Codec arithmeticDecoder;
  o3dgc::Static_Bit_Model bModel0;
  uint32_t                compressedBitstreamSize;

  bool bBinArithCoding = context.getBinArithCoding() && ( !context.getLosslessGeo() ) &&
                         ( ops.getPackingBlockSize() == 16 ) &&
                         ( context.getOccupancyPrecision() == 4 );

  uint8_t enable_flexible_patch_flag = bitstream.read<uint8_t>();
  if ( ( frameIndex == 0 ) || ( !context.getDeltaCoding() ) ) {
    uint8_t bitCountU0      = bitstream.read<uint8_t>();
    uint8_t bitCountV0      = bitstream.read<uint8_t>();
    uint8_t bitCountU1      = bitstream.read<uint8_t>();
    uint8_t bitCountV1      = bitstream.read<uint8_t>();
    uint8_t bitCountD1      = maxBitCountForMinDepth;
    uint8_t bitCountDD      = maxBitCountForMaxDepth;
    uint8_t bitCountLod     = bitstream.read<uint8_t>();
    compressedBitstreamSize = bitstream.read<uint32_t>();

    assert( compressedBitstreamSize + bitstream.size() <= bitstream.capacity() );
    arithmeticDecoder.set_buffer( uint32_t( bitstream.capacity() - bitstream.size() ),
                                  bitstream.buffer() + bitstream.size() );
    arithmeticDecoder.start_decoder();

    frame.getFrameLevelMetadata().setMetadataType( METADATA_FRAME );
    frame.getFrameLevelMetadata().setIndex( frame.getIndex() );
    decompressMetadata( frame.getFrameLevelMetadata(), arithmeticDecoder );

    o3dgc::Adaptive_Bit_Model  bModelDD;
    o3dgc::Adaptive_Bit_Model  bModelSizeU0, bModelSizeV0, bModelAbsoluteD1;
    o3dgc::Adaptive_Bit_Model  orientationModel2;
    o3dgc::Adaptive_Data_Model orientationModel( 4 );
    o3dgc::Adaptive_Data_Model orientationPatchModel( NumPatchOrientations - 1 + 2 );
    o3dgc::Adaptive_Bit_Model  orientationPatchFlagModel2;
    o3dgc::Adaptive_Bit_Model  interpolateModel, fillingModel, occupiedModel;
    o3dgc::Adaptive_Data_Model minD1Model( 3 ), neighborModel( 3 );
    int64_t                    prevSizeU0 = 0;
    int64_t                    prevSizeV0 = 0;

    for ( size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex ) {
      auto& patch                    = patches[patchIndex];
      patch.getOccupancyResolution() = ops.getPackingBlockSize();

      patch.getU0() = DecodeUInt32( bitCountU0, arithmeticDecoder, bModel0 );
      patch.getV0() = DecodeUInt32( bitCountV0, arithmeticDecoder, bModel0 );
      if ( enable_flexible_patch_flag ) {
        bool flexible_patch_present_flag = arithmeticDecoder.decode( orientationPatchFlagModel2 );
        if ( flexible_patch_present_flag ) {
          patch.getPatchOrientation() = arithmeticDecoder.decode( orientationPatchModel ) + 1;
        } else {
          patch.getPatchOrientation() = PatchOrientation::DEFAULT;
        }
      } else {
        patch.getPatchOrientation() = PatchOrientation::DEFAULT;
      }
      patch.getU1() = DecodeUInt32( bitCountU1, arithmeticDecoder, bModel0 );
      patch.getV1() = DecodeUInt32( bitCountV1, arithmeticDecoder, bModel0 );
      size_t D1     = DecodeUInt32( bitCountD1, arithmeticDecoder, bModel0 );
      patch.getD1() = D1 * minLevel;
      // size_t DD = DecodeUInt32(bitCount[5], arithmeticDecoder, bModel0);
      patch.getLod() = DecodeUInt32( bitCountLod, arithmeticDecoder, bModel0 );

      if ( !context.getAbsoluteD1() ) {
        patch.getFrameProjectionMode() = frameProjectionMode;
        if ( patch.getFrameProjectionMode() == 0 ) {
          patch.getProjectionMode() = 0;
        } else if ( patch.getFrameProjectionMode() == 1 ) {
          patch.getProjectionMode() = 1;
        } else if ( patch.getFrameProjectionMode() == 2 ) {
          patch.getProjectionMode() = 0;
          const uint8_t bitCountProjDir =
              uint8_t( PCCGetNumberOfBitsInFixedLengthRepresentation( uint32_t( 2 + 1 ) ) );
          patch.getProjectionMode() = DecodeUInt32( bitCountProjDir, arithmeticDecoder, bModel0 );
          std::cout << "patch.getProjectionMode()= " << patch.getProjectionMode() << std::endl;
        } else {
          std::cout << "This frameProjectionMode doesn't exist!" << std::endl;
        }
        std::cout << "(frameProjMode, projMode)= (" << patch.getFrameProjectionMode() << ", "
                  << patch.getProjectionMode() << ")" << std::endl;
      } else {
        patch.getFrameProjectionMode() = 0;
        if ( context.getSixDirectionMode() ) {
          patch.getProjectionMode() = arithmeticDecoder.decode( bModel0 );
        } else {
          patch.getProjectionMode() = 0;
        }
      }
      if ( patch.getProjectionMode() == 1 ) patch.getD1() = 1024 - D1 * minLevel;

      const int64_t deltaSizeU0 =
          o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel0, bModelSizeU0 ) );
      const int64_t deltaSizeV0 =
          o3dgc::UIntToInt( arithmeticDecoder.ExpGolombDecode( 0, bModel0, bModelSizeV0 ) );

      patch.getSizeU0() = prevSizeU0 + deltaSizeU0;
      patch.getSizeV0() = prevSizeV0 + deltaSizeV0;

      prevSizeU0 = patch.getSizeU0();
      prevSizeV0 = patch.getSizeV0();

      if ( bBinArithCoding ) {
        size_t bit0 = arithmeticDecoder.decode( orientationModel2 );
        if ( bit0 == 0 ) {  // 0
          patch.getNormalAxis() = 0;
        } else {
          size_t bit1 = arithmeticDecoder.decode( bModel0 );
          if ( bit1 == 0 ) {  // 10
            patch.getNormalAxis() = 1;
          } else {  // 11
            patch.getNormalAxis() = 2;
          }
        }
      } else {
        patch.getNormalAxis() = arithmeticDecoder.decode( orientationModel );
      }

      if ( patch.getNormalAxis() == 0 ) {
        patch.getTangentAxis()   = 2;
        patch.getBitangentAxis() = 1;
      } else if ( patch.getNormalAxis() == 1 ) {
        patch.getTangentAxis()   = 2;
        patch.getBitangentAxis() = 0;
      } else {
        patch.getTangentAxis()   = 0;
        patch.getBitangentAxis() = 1;
      }
      auto& patchLevelMetadataEnabledFlags =
          frame.getFrameLevelMetadata().getLowerLevelMetadataEnabledFlags();
      auto& patchLevelMetadata = patch.getPatchLevelMetadata();
      patchLevelMetadata.setIndex( patchIndex );
      patchLevelMetadata.setMetadataType( METADATA_PATCH );
      patchLevelMetadata.getMetadataEnabledFlags() = patchLevelMetadataEnabledFlags;
      patchLevelMetadata.setbitCountQDepth( bitCountDD );
      decompressMetadata( patchLevelMetadata, arithmeticDecoder, bModel0, bModelDD );

      // maxDepth reconstruction
#ifdef CE210_MAXDEPTH_EVALUATION
      patch.getSizeD() = size_t( patchLevelMetadata.getQMaxDepthInPatch() ) * minLevel;
#else
      patch.getSizeD() = minLevel;
#endif
      decompressOneLayerData( context, frame, patch, arithmeticDecoder, occupiedModel,
                              interpolateModel, neighborModel, minD1Model, fillingModel );
      if ( printDetailedInfo ) { patch.printDecoder(); }
    }
  } else {
    decompressPatchMetaDataM42195( context, frame, preFrame, bitstream, arithmeticDecoder, bModel0,
                                   compressedBitstreamSize, context.getOccupancyPrecision(),
                                   enable_flexible_patch_flag );
  }

  arithmeticDecoder.stop_decoder();
  bitstream += (uint64_t)compressedBitstreamSize;
}
