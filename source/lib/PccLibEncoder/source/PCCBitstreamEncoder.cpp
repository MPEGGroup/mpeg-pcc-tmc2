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

#include "PCCBitstreamEncoder.h"

using namespace std;
using namespace pcc;

PCCBitstreamEncoder::PCCBitstreamEncoder(){
}

PCCBitstreamEncoder::~PCCBitstreamEncoder(){
}

void PCCBitstreamEncoder::setParameters( PCCEncoderParameters params ) {
  params_ = params;
}

int PCCBitstreamEncoder::encode( PCCContext &context, PCCBitstream &bitstream ){
  
  auto sizeFrameHeader = bitstream.size();
  std::cout << "PCCEncoder: compress (start: bitstream = " << bitstream.size() << " / " << bitstream.capacity() << " )" << std::endl;
  compressHeader( context, bitstream );
  std::cout << "PCCEncoder: compress (header: bitstream = " << bitstream.size() << " / " << bitstream.capacity() << " )" << std::endl;
  sizeFrameHeader = bitstream.size() - sizeFrameHeader;
  std::cout << "frame header info  ->" << sizeFrameHeader << " B " << std::endl;
  context.printVideoBitstream();

  auto& sps = context.getSps();
  std::cout << "PCCEncoder: compress (video bitstream start: bitstream = "<< bitstream.size()<<" / "<< bitstream.capacity()<<" )" << std::endl;
  bitstream.write( context.getVideoBitstream( PCCVideoType::OccupancyMap ) );
  if (!sps.getLayerAbsoluteCodingEnabledFlag( 1 )) {
    bitstream.write( context.getVideoBitstream( PCCVideoType::GeometryD0 ) );
    bitstream.write( context.getVideoBitstream( PCCVideoType::GeometryD1 ) );
  } else {
    bitstream.write( context.getVideoBitstream(  PCCVideoType::Geometry ) );
  }
  if( sps.getPcmPatchEnabledFlag() && sps.getPcmSeparateVideoPresentFlag()) {
    bitstream.write( context.getVideoBitstream(  PCCVideoType::GeometryMP ) );
  }
  if ( sps.getAttributeCount() > 0 ) {
    bitstream.write( context.getVideoBitstream( PCCVideoType::Texture ) );
    if( sps.getPcmPatchEnabledFlag() && sps.getPcmSeparateVideoPresentFlag()) {
      bitstream.write( context.getVideoBitstream( PCCVideoType::TextureMP ) );
    }
  }
  std::cout << "PCCEncoder: compress (video bitstream done: bitstream = :"<<bitstream.size()<<" / "<< bitstream.capacity()<<" )" << std::endl;

  size_t sizeOccupancyMap = bitstream.size();
  compressOccupancyMap(context, bitstream);

  std::cout << "PCCEncoder: compress (metadata done: bitstream = "<< bitstream.size()<<" / "<< bitstream.capacity()<<" )" << std::endl;
  sizeOccupancyMap = ( bitstream.size() - sizeOccupancyMap )
          + context.getVideoBitstream( PCCVideoType::OccupancyMap ).naluSize();
  std::cout << " occupancy map  ->" << sizeOccupancyMap << " B " << std::endl;

  if ( sps.getPcmPatchEnabledFlag() && sps.getPcmSeparateVideoPresentFlag()) {
    writeMissedPointsGeometryNumber( context, bitstream );
  }
  else if ( sps.getPcmPatchEnabledFlag() && !sps.getPcmSeparateVideoPresentFlag()) {
    for (auto &frame : context.getFrames()){
      size_t numMissedPts = frame.getMissedPointsPatch().numMissedPts_;
      bitstream.write<size_t>(size_t(numMissedPts));
    }
  }

  if( sps.getAttributeCount() > 0 ) {
    if( sps.getPcmPatchEnabledFlag() && sps.getPcmSeparateVideoPresentFlag())  {
      writeMissedPointsTextureNumber( context, bitstream );
    }
  }
  std::cout << "PCCEncoder: compress (done: bitstream = " << bitstream.size() << " / " << bitstream.capacity() << " )" << std::endl;
  return 0;
}

void PCCBitstreamEncoder::EncodeUInt32( const uint32_t value,
                   const uint32_t bitCount,
                   o3dgc::Arithmetic_Codec &arithmeticEncoder,
                   o3dgc::Static_Bit_Model &bModel0 ) {
  uint32_t valueToEncode = PCCToLittleEndian<uint32_t>(value);
  for (uint32_t i = 0; i < bitCount; ++i) {
    arithmeticEncoder.encode(valueToEncode & 1, bModel0);
    valueToEncode >>= 1;
  }
}

void PCCBitstreamEncoder::writeMissedPointsGeometryNumber(PCCContext& context, PCCBitstream &bitstream) {
  size_t maxHeight = 0;
  size_t MPwidth=context.getMPGeoWidth ();
  auto& sps = context.getSps();
  bitstream.write<size_t>(size_t( context.getMPGeoWidth () ) );
  for (auto &frame : context.getFrames() ) {
    size_t numofMPs=frame.getMissedPointsPatch().size();
    if(!sps.getLosslessGeo444()) {
      numofMPs/=3;
    }
    bitstream.write<size_t>(size_t( numofMPs ) );
    size_t height = (3*numofMPs)/MPwidth+1;
    size_t heightby8= height/8;
    if(heightby8*8!=height) {
      height = (heightby8+1)*8;
    }
    maxHeight = (std::max)( maxHeight, height );
  }
  assert(maxHeight==context.getMPGeoHeight ());
}

void PCCBitstreamEncoder::writeMissedPointsTextureNumber(PCCContext& context, PCCBitstream &bitstream) {
  size_t maxHeight = 0;
  size_t MPwidth=context.getMPAttWidth ();
  bitstream.write<size_t>(size_t( context.getMPAttWidth () ) );
  for (auto &frame : context.getFrames() ) {
    size_t numofMPs=frame.getMissedPointsPatch().sizeofcolor();
    bitstream.write<size_t>(size_t( numofMPs ) );
    size_t height = (3*numofMPs)/MPwidth+1;
    size_t heightby8= height/8;
    if(heightby8*8!=height) {
      height = (heightby8+1)*8;
    }
    maxHeight = (std::max)( maxHeight, height );
  }
  assert(maxHeight==context.getMPAttHeight ());
}


int PCCBitstreamEncoder::writeMetadata( const PCCMetadata &metadata, pcc::PCCBitstream &bitstream ) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("Metadata start \n" ) ; 
#endif
  auto &metadataEnabledFlags = metadata.getMetadataEnabledFlags();
  if (!metadataEnabledFlags.getMetadataEnabled()) {
#ifdef BITSTREAM_TRACE
  bitstream.trace("Metadata done not metadata \n" ) ; 
#endif
    return 0;
  }
  bitstream.write<uint8_t>(metadata.getMetadataPresent());
  std::cout<<"read/write METADATA TYPE: "<<metadata.getMetadataType()
      <<" METADATA present: "<<(metadata.getMetadataPresent()==true?1:0)<<std::endl;
  if (metadata.getMetadataPresent()) {
    if (metadataEnabledFlags.getScaleEnabled()) {
      bitstream.write<uint8_t>(metadata.getScalePresent());
      if (metadata.getScalePresent()) {
        bitstream.write<PCCVector3U>(metadata.getScale());
      }
    }
    if (metadataEnabledFlags.getOffsetEnabled()) {
      bitstream.write<uint8_t>(metadata.getOffsetPresent());
      if (metadata.getOffsetPresent()) {
        bitstream.write<PCCVector3I>(metadata.getOffset());
      }
    }
    if (metadataEnabledFlags.getRotationEnabled()) {
      bitstream.write<uint8_t>(metadata.getRotationPresent());
      if (metadata.getRotationPresent()) {
        bitstream.write<PCCVector3I>(metadata.getRotation());
      }
    }
    if (metadataEnabledFlags.getPointSizeEnabled()) {
      bitstream.write<uint8_t>(metadata.getPointSizePresent());
      if (metadata.getPointSizePresent()) {
        bitstream.write<uint16_t>(metadata.getPointSize());
      }
    }
    if (metadataEnabledFlags.getPointShapeEnabled()) {
      bitstream.write<uint8_t>(metadata.getPointShapePresent());
      if (metadata.getPointShapePresent()) {
        bitstream.write<uint8_t>(uint8_t(metadata.getPointShape()));
      }
    }
  }

  auto &lowerLevelMetadataEnabledFlags = metadata.getLowerLevelMetadataEnabledFlags();
  bitstream.write<uint8_t>(lowerLevelMetadataEnabledFlags.getMetadataEnabled());

  if (lowerLevelMetadataEnabledFlags.getMetadataEnabled()) {
    bitstream.write<uint8_t>(lowerLevelMetadataEnabledFlags.getScaleEnabled());
    bitstream.write<uint8_t>(lowerLevelMetadataEnabledFlags.getOffsetEnabled());
    bitstream.write<uint8_t>(lowerLevelMetadataEnabledFlags.getRotationEnabled());
    bitstream.write<uint8_t>(lowerLevelMetadataEnabledFlags.getPointSizeEnabled());
    bitstream.write<uint8_t>(lowerLevelMetadataEnabledFlags.getPointShapeEnabled());
#ifdef CE210_MAXDEPTH_EVALUATION
    bitstream.write<uint8_t>(lowerLevelMetadataEnabledFlags.getMaxDepthEnabled());
#endif
  }
#ifdef BITSTREAM_TRACE
  bitstream.trace("Metadata done \n" ) ; 
#endif
  return 1;
}

int PCCBitstreamEncoder::compressMetadata(const PCCMetadata &metadata, o3dgc::Arithmetic_Codec &arithmeticEncoder) {
  auto &metadataEnabingFlags = metadata.getMetadataEnabledFlags();
  if (!metadataEnabingFlags.getMetadataEnabled()) {
    return 0;
  }
  static o3dgc::Static_Bit_Model   bModel0;
  static o3dgc::Adaptive_Bit_Model bModelMetadataPresent;
  arithmeticEncoder.encode(metadata.getMetadataPresent(), bModelMetadataPresent);

  if (metadata.getMetadataPresent()) {
    if (metadataEnabingFlags.getScaleEnabled()) {
      static o3dgc::Adaptive_Bit_Model bModelScalePresent;
      arithmeticEncoder.encode(metadata.getScalePresent(), bModelScalePresent);
      if (metadata.getScalePresent()) {
        EncodeUInt32(uint32_t(metadata.getScale()[0]), 32, arithmeticEncoder, bModel0);
        EncodeUInt32(uint32_t(metadata.getScale()[1]), 32, arithmeticEncoder, bModel0);
        EncodeUInt32(uint32_t(metadata.getScale()[2]), 32, arithmeticEncoder, bModel0);
      }
    }

    if (metadataEnabingFlags.getOffsetEnabled()) {
      static o3dgc::Adaptive_Bit_Model bModelOffsetPresent;
      arithmeticEncoder.encode(metadata.getOffsetPresent(), bModelOffsetPresent);
      if (metadata.getOffsetPresent()) {
        EncodeUInt32(o3dgc::IntToUInt(metadata.getOffset()[0]), 32, arithmeticEncoder, bModel0);
        EncodeUInt32(o3dgc::IntToUInt(metadata.getOffset()[1]), 32, arithmeticEncoder, bModel0);
        EncodeUInt32(o3dgc::IntToUInt(metadata.getOffset()[2]), 32, arithmeticEncoder, bModel0);
      }
    }

    if (metadataEnabingFlags.getRotationEnabled()) {
      static o3dgc::Adaptive_Bit_Model bModelRotationPresent;
      arithmeticEncoder.encode(metadata.getRotationPresent(), bModelRotationPresent);
      if (metadata.getRotationPresent()) {
        EncodeUInt32(o3dgc::IntToUInt(metadata.getRotation()[0]), 32, arithmeticEncoder, bModel0);
        EncodeUInt32(o3dgc::IntToUInt(metadata.getRotation()[1]), 32, arithmeticEncoder, bModel0);
        EncodeUInt32(o3dgc::IntToUInt(metadata.getRotation()[2]), 32, arithmeticEncoder, bModel0);
      }
    }

    if (metadataEnabingFlags.getPointSizeEnabled()) {
      static o3dgc::Adaptive_Bit_Model bModelPointSizePresent;
      arithmeticEncoder.encode(metadata.getPointSizePresent(), bModelPointSizePresent);
      if (metadata.getPointSizePresent()) {
        EncodeUInt32(uint32_t(metadata.getPointSize()), 16, arithmeticEncoder, bModel0);
      }
    }

    if (metadataEnabingFlags.getPointShapeEnabled()) {
      static o3dgc::Adaptive_Bit_Model bModelPointShapePresent;
      arithmeticEncoder.encode(metadata.getPointShapePresent(), bModelPointShapePresent);
      if (metadata.getPointShapePresent()) {
        EncodeUInt32(uint32_t(metadata.getPointShape()), 8, arithmeticEncoder, bModel0);
      }
    }
  }

  auto &lowerLevelMetadataEnabledFlags = metadata.getLowerLevelMetadataEnabledFlags();
  static o3dgc::Adaptive_Bit_Model bModelLowerLevelMetadataEnabled;
  arithmeticEncoder.encode(lowerLevelMetadataEnabledFlags.getMetadataEnabled(), bModelLowerLevelMetadataEnabled);
  //std::cout<<"METADATA lowerLevelMetaData: "<<lowerLevelMetadataEnabledFlags.getMetadataEnabled()<<std::endl;
  if (lowerLevelMetadataEnabledFlags.getMetadataEnabled()) {
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelScaleEnabled;
    arithmeticEncoder.encode(lowerLevelMetadataEnabledFlags.getScaleEnabled(), bModelLowerLevelScaleEnabled);

    static o3dgc::Adaptive_Bit_Model bModelLowerLevelOffsetEnabled;
    arithmeticEncoder.encode(lowerLevelMetadataEnabledFlags.getOffsetEnabled(), bModelLowerLevelOffsetEnabled);

    static o3dgc::Adaptive_Bit_Model bModelLowerLevelRotationEnabled;
    arithmeticEncoder.encode(lowerLevelMetadataEnabledFlags.getRotationEnabled(), bModelLowerLevelRotationEnabled);

    static o3dgc::Adaptive_Bit_Model bModelLowerLevelPointSizeEnabled;
    arithmeticEncoder.encode(lowerLevelMetadataEnabledFlags.getPointSizeEnabled(), bModelLowerLevelPointSizeEnabled);

    static o3dgc::Adaptive_Bit_Model bModelLowerLevelPointShapeEnabled;
    arithmeticEncoder.encode(lowerLevelMetadataEnabledFlags.getPointShapeEnabled(), bModelLowerLevelPointShapeEnabled);

#ifdef CE210_MAXDEPTH_EVALUATION
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelMaxDepthEnabled;
    arithmeticEncoder.encode(lowerLevelMetadataEnabledFlags.getMaxDepthEnabled(), bModelLowerLevelMaxDepthEnabled);
#endif

  }
  return 1;
}

int PCCBitstreamEncoder::compressMetadata(const PCCMetadata &metadata, o3dgc::Arithmetic_Codec &arithmeticEncoder, o3dgc::Static_Bit_Model &bModelMaxDepth0, o3dgc::Adaptive_Bit_Model &bModelMaxDepthDD
) {
  auto &metadataEnabingFlags = metadata.getMetadataEnabledFlags();
  if (!metadataEnabingFlags.getMetadataEnabled()) {
    return 0;
  }
  static o3dgc::Static_Bit_Model   bModel0;
  static o3dgc::Adaptive_Bit_Model bModelMetadataPresent;
  arithmeticEncoder.encode(metadata.getMetadataPresent(), bModelMetadataPresent);

  if (metadata.getMetadataPresent()) {

    if (metadataEnabingFlags.getScaleEnabled()) {
      static o3dgc::Adaptive_Bit_Model bModelScalePresent;
      arithmeticEncoder.encode(metadata.getScalePresent(), bModelScalePresent);
      if (metadata.getScalePresent()) {
        EncodeUInt32(uint32_t(metadata.getScale()[0]), 32, arithmeticEncoder, bModel0);
        EncodeUInt32(uint32_t(metadata.getScale()[1]), 32, arithmeticEncoder, bModel0);
        EncodeUInt32(uint32_t(metadata.getScale()[2]), 32, arithmeticEncoder, bModel0);
      }
    }

    if (metadataEnabingFlags.getOffsetEnabled()) {
      static o3dgc::Adaptive_Bit_Model bModelOffsetPresent;
      arithmeticEncoder.encode(metadata.getOffsetPresent(), bModelOffsetPresent);
      if (metadata.getOffsetPresent()) {
        EncodeUInt32(o3dgc::IntToUInt(metadata.getOffset()[0]), 32, arithmeticEncoder, bModel0);
        EncodeUInt32(o3dgc::IntToUInt(metadata.getOffset()[1]), 32, arithmeticEncoder, bModel0);
        EncodeUInt32(o3dgc::IntToUInt(metadata.getOffset()[2]), 32, arithmeticEncoder, bModel0);
      }
    }

    if (metadataEnabingFlags.getRotationEnabled()) {
      static o3dgc::Adaptive_Bit_Model bModelRotationPresent;
      arithmeticEncoder.encode(metadata.getRotationPresent(), bModelRotationPresent);
      if (metadata.getRotationPresent()) {
        EncodeUInt32(o3dgc::IntToUInt(metadata.getRotation()[0]), 32, arithmeticEncoder, bModel0);
        EncodeUInt32(o3dgc::IntToUInt(metadata.getRotation()[1]), 32, arithmeticEncoder, bModel0);
        EncodeUInt32(o3dgc::IntToUInt(metadata.getRotation()[2]), 32, arithmeticEncoder, bModel0);
      }
    }

    if (metadataEnabingFlags.getPointSizeEnabled()) {
      static o3dgc::Adaptive_Bit_Model bModelPointSizePresent;
      arithmeticEncoder.encode(metadata.getPointSizePresent(), bModelPointSizePresent);
      if (metadata.getPointSizePresent()) {
        EncodeUInt32(uint32_t(metadata.getPointSize()), 16, arithmeticEncoder, bModel0);
      }
    }

    if (metadataEnabingFlags.getPointShapeEnabled()) {
      static o3dgc::Adaptive_Bit_Model bModelPointShapePresent;
      arithmeticEncoder.encode(metadata.getPointShapePresent(), bModelPointShapePresent);
      if (metadata.getPointShapePresent()) {
        EncodeUInt32(uint32_t(metadata.getPointShape()), 8, arithmeticEncoder, bModel0);
      }
    }

#ifdef CE210_MAXDEPTH_EVALUATION
    if(metadataEnabingFlags.getMaxDepthEnabled()) {
      const uint8_t maxBitCountForMaxDepth= metadata.getbitCountQDepth();
      int64_t currentDD=0;
      if(maxBitCountForMaxDepth==0) { //delta_DD
        currentDD=metadata.getQMaxDepthInPatch();
        arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(currentDD)),    0, bModelMaxDepth0, bModelMaxDepthDD);
      } else {
        currentDD=metadata.getQMaxDepthInPatch();
        EncodeUInt32(uint32_t(currentDD), maxBitCountForMaxDepth, arithmeticEncoder, bModelMaxDepth0);
      }
    }
#endif
  }

  if(metadata.getMetadataType()==METADATA_PATCH) {
    //std::cout<<"lowerLevelFlags: "<<metadata.getLowerLevelMetadataEnabledFlags().getMetadataEnabled()<<std::endl;
    return 1;
  }

  auto &lowerLevelMetadataEnabledFlags = metadata.getLowerLevelMetadataEnabledFlags();
  static o3dgc::Adaptive_Bit_Model bModelLowerLevelMetadataEnabled;
  arithmeticEncoder.encode(lowerLevelMetadataEnabledFlags.getMetadataEnabled(), bModelLowerLevelMetadataEnabled);
  std::cout<<"lowerLevelFlags: "<<metadata.getLowerLevelMetadataEnabledFlags().getMetadataEnabled()<<std::endl;
  if (lowerLevelMetadataEnabledFlags.getMetadataEnabled()) {
    static o3dgc::Adaptive_Bit_Model bModelLowerLevelScaleEnabled;
    arithmeticEncoder.encode(lowerLevelMetadataEnabledFlags.getScaleEnabled(), bModelLowerLevelScaleEnabled);

    static o3dgc::Adaptive_Bit_Model bModelLowerLevelOffsetEnabled;
    arithmeticEncoder.encode(lowerLevelMetadataEnabledFlags.getOffsetEnabled(), bModelLowerLevelOffsetEnabled);

    static o3dgc::Adaptive_Bit_Model bModelLowerLevelRotationEnabled;
    arithmeticEncoder.encode(lowerLevelMetadataEnabledFlags.getRotationEnabled(), bModelLowerLevelRotationEnabled);

    static o3dgc::Adaptive_Bit_Model bModelLowerLevelPointSizeEnabled;
    arithmeticEncoder.encode(lowerLevelMetadataEnabledFlags.getPointSizeEnabled(), bModelLowerLevelPointSizeEnabled);

    static o3dgc::Adaptive_Bit_Model bModelLowerLevelPointShapeEnabled;
    arithmeticEncoder.encode(lowerLevelMetadataEnabledFlags.getPointShapeEnabled(), bModelLowerLevelPointShapeEnabled);
  }
  return 1;
}

int PCCBitstreamEncoder::compressHeader( PCCContext& context, pcc::PCCBitstream& bitstream ) {
  auto& sps = context.getSps();
  auto& gps = sps.getGeometryParameterSet();
  auto& gsp = gps.getGeometrySequenceParams();
  auto& ops = sps.getOccupancyParameterSet();
  auto& aps = sps.getAttributeParameterSet( 0 );
  auto& asp = aps.getAttributeSequenceParams();
  
#ifdef BITSTREAM_TRACE
  bitstream.trace("Header start \n" ) ; 
#endif
  bitstream.write<uint8_t>( ( uint8_t )( context.size() ) );
  if ( !context.size() ) { return 0; }
  bitstream.write<uint16_t>( uint16_t( sps.getFrameWidth() ) );
  bitstream.write<uint16_t>( uint16_t( sps.getFrameHeight() ) );
  bitstream.write<uint8_t>( uint8_t( ops.getOccupancyPackingBlockSize() ) );
  // bitstream.write<uint8_t>( uint8_t( context.getOccupancyPrecision() ) );
  //jkei[??] is it changed as intended?
  
#ifdef BITSTREAM_TRACE
  bitstream.trace("Header smooth geo \n" ) ; 
#endif
  bitstream.write<uint8_t>( uint8_t( gsp.getGeometrySmoothingParamsPresentFlag() ) );
  if ( gsp.getGeometrySmoothingParamsPresentFlag() ) {
    bitstream.write<uint8_t>( uint8_t( gsp.getGeometrySmoothingEnabledFlag() ) );
    // if ( context.getGridSmoothing() ) {
      bitstream.write<uint8_t>( uint8_t( gsp.getGeometrySmoothingGridSize() ) );
      bitstream.write<uint8_t>( uint8_t( gsp.getGeometrySmoothingThreshold() ) );
    // } else {
    //   bitstream.write<uint8_t>( uint8_t( asp.getAttributeSmoothingParamsPresentFlag() ) );
    //   bitstream.write<uint8_t>( uint8_t( asp.getAttributeSmoothingNeighbourCount() ) );
    //   bitstream.write<uint8_t>( uint8_t( asp.getAttributeSmoothingRadius2BoundaryDetection() ) );
    //   bitstream.write<uint8_t>( uint8_t( gsp.getGeometrySmoothingThreshold() ) );
    // }
  }
#ifdef BITSTREAM_TRACE
  bitstream.trace("Header general \n" ) ; 
#endif
  bitstream.write<uint8_t>( uint8_t( sps.getLosslessGeo() ) );
  bitstream.write<uint8_t>( uint8_t( sps.getLosslessTexture() ) );
  bitstream.write<uint8_t>( uint8_t( sps.getAttributeCount() ) );
  bitstream.write<uint8_t>( uint8_t( sps.getLosslessGeo444() ) );
  bitstream.write<uint8_t>( uint8_t( sps.getMinLevel() ) );
  bitstream.write<uint8_t>( uint8_t( sps.getPcmSeparateVideoPresentFlag() ) );
  bitstream.write<uint8_t>( uint8_t( sps.getLayerAbsoluteCodingEnabledFlag( 1 ) ) );
  // if ( sps.getLayerAbsoluteCodingEnabledFlag( 1 ) ) {
  //    bitstream.write<uint8_t>( uint8_t( context.getSixDirectionMode() ) );
  // }
#ifdef BITSTREAM_TRACE
  bitstream.trace("Header Model \n" ) ; 
#endif

  //bitstream.write<uint8_t>( uint8_t( context.getBinArithCoding() ) );
// bitstream.write<float>( context.getModelScale() );
// bitstream.write<float>( context.getModelOrigin()[0] );
// bitstream.write<float>( context.getModelOrigin()[1] );
// bitstream.write<float>( context.getModelOrigin()[2] );

#ifdef BITSTREAM_TRACE
  bitstream.trace("Header metadata \n" ) ; 
#endif
  writeMetadata( context.getGOFLevelMetadata(), bitstream );

#ifdef BITSTREAM_TRACE
  bitstream.trace("Header smooth attribut \n" ) ; 
#endif
  bitstream.write<uint8_t>( uint8_t( asp.getAttributeSmoothingParamsPresentFlag() ) );
  if ( asp.getAttributeSmoothingParamsPresentFlag() ) {
    bitstream.write<uint8_t>( uint8_t( asp.getAttributeSmoothingThreshold() ) );
    bitstream.write<double>( double( asp.getAttributeSmoothingThresholdLocalEntropy() ) );
    bitstream.write<uint8_t>( uint8_t( asp.getAttributeSmoothingRadius() ) );
    bitstream.write<uint8_t>( uint8_t( asp.getAttributeSmoothingNeighbourCount() ) );
  }
#ifdef BITSTREAM_TRACE
  bitstream.trace("Header lossless \n" ) ; 
#endif
  if ( sps.getLosslessGeo() ) {
    bitstream.write<uint8_t>( uint8_t( sps.getEnhancedOccupancyMapForDepthFlag() ) );
    // bitstream.write<uint8_t>( uint8_t( context.getImproveEDD() ) );
  }
  bitstream.write<uint8_t>( uint8_t( sps.getPatchInterPredictionEnabledFlag() ) ); // context.getDeltaCoding() ) );
  bitstream.write<uint8_t>( uint8_t( sps.getRemoveDuplicatePointEnabledFlag() ) );
  bitstream.write<uint8_t>( uint8_t( sps.getMultipleLayerStreamsPresentFlag() ) );
  bitstream.write<uint8_t>( uint8_t( sps.getPixelDeinterleavingFlag() ) );
  bitstream.write<uint8_t>( uint8_t( sps.getPcmPatchEnabledFlag() ) );
  // bitstream.write<uint8_t>( uint8_t( context.getGlobalPatchAllocation() ) );
#ifdef BITSTREAM_TRACE
  bitstream.trace("Header done \n" ) ; 
#endif
  return 1;
}

void PCCBitstreamEncoder::compressPatchMetaDataM42195( PCCContext      &context,
                                                       PCCFrameContext &frame,
                                                       PCCFrameContext &preFrame,
                                                       size_t numMatchedPatches,
                                                       PCCBitstream &bitstream ,
                                                       o3dgc::Arithmetic_Codec &arithmeticEncoder,
                                                       o3dgc::Static_Bit_Model &bModel0,
                                                       uint8_t enable_flexible_patch_flag) {                                                 
#ifdef BITSTREAM_TRACE
  bitstream.trace("OccupancyMapM42195 start enable_flexible_patch_flag = %lu \n",enable_flexible_patch_flag ) ; 
#endif
  auto&         sps        = context.getSps();
  auto&         ops        = sps.getOccupancyParameterSet();
  auto&         patches    = frame.getPatches();
  auto&         prePatches = preFrame.getPatches();
  size_t        patchCount = patches.size();
  size_t        TopNmaxU0 = 0, maxU0 = 0;
  size_t TopNmaxV0 = 0, maxV0 = 0;
  size_t TopNmaxU1 = 0, maxU1 = 0;
  size_t TopNmaxV1 = 0, maxV1 = 0;
  size_t TopNmaxD1 = 0, maxD1 = 0;
  size_t TopNmaxDD = 0, maxDD = 0;
  const size_t minLevel = sps.getMinLevel();
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

  // bool bBinArithCoding = context.getBinArithCoding() && (!context.getLosslessGeo()) &&
  //     ( ops.getOccupancyPackingBlockSize() == 16) && (context.getOccupancyPrecision() == 4);

  o3dgc::Adaptive_Bit_Model bModelPatchIndex, bModelU0, bModelV0, bModelU1, bModelV1, bModelD1,bModelIntSizeU0,bModelIntSizeV0;
  o3dgc::Adaptive_Bit_Model bModelSizeU0, bModelSizeV0, bModelAbsoluteD1;
  o3dgc::Adaptive_Data_Model orientationModel(4);
  o3dgc::Adaptive_Bit_Model orientationModel2;
  o3dgc::Adaptive_Data_Model orientationPatchModel(NumPatchOrientations - 1 + 2);
  o3dgc::Adaptive_Bit_Model orientationPatchFlagModel2;
  o3dgc::Adaptive_Bit_Model occupiedModel, interpolateModel, fillingModel;
  o3dgc::Adaptive_Data_Model minD1Model(3), neighborModel(3);

  //jkei[??] do we need to signal frame metadata here too?
  // compressMetadata(frame.getFrameLevelMetadata(), arithmeticEncoder);

  const uint8_t bitCountNumPatches =
      uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(patchCount)));//numMatchedPatches <= patchCount
  EncodeUInt32(uint32_t(numMatchedPatches), bitCountNumPatches, arithmeticEncoder, bModel0);
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
  EncodeUInt32(uint32_t(F), 1, arithmeticEncoder, bModel0);
  if(F){
    EncodeUInt32(uint32_t(flag), 4, arithmeticEncoder, bModel0);
    for (int i = 0; i < 4; i++) {
      if (A[i])  EncodeUInt32(uint32_t(bitCount[i]), 8, arithmeticEncoder, bModel0);
    }
  }
  if (printDetailedInfo) {
    printf("numPatch:%d(%d), numMatchedPatches:%d, F:%d,A:%d,%d,%d,%d\n",
           (int)patchCount,(int)bitCountNumPatches, (int)numMatchedPatches, F, A[0], A[1], A[2], A[3]);
    printf("bitCount:%d,%d,%d,%d,%d\n",
           (int)bitCount[0], (int)bitCount[1], (int)bitCount[2], (int)bitCount[3], (int)bitCount[4]);
  }
  int64_t predIndex = 0;
  for (size_t patchIndex = 0; patchIndex < numMatchedPatches; ++patchIndex) {
    const auto &patch    = patches[patchIndex];
    const auto &prepatch = prePatches[patch.getBestMatchIdx()];

    const int64_t delta_index = patch.getBestMatchIdx() - predIndex;
    predIndex += (delta_index + 1);
    const int64_t delta_u0 = static_cast<int64_t>(patch.getU0() - prepatch.getU0());
    const int64_t delta_v0 = static_cast<int64_t>(patch.getV0() - prepatch.getV0());
    const int64_t delta_u1 = static_cast<int64_t>(patch.getU1() - prepatch.getU1());
    const int64_t delta_v1 = static_cast<int64_t>(patch.getV1() - prepatch.getV1());
    size_t currentD1=patch.getD1();
    size_t prevD1=prepatch.getD1();
    if(patch.getProjectionMode()==0) {
      currentD1=currentD1/minLevel;
      prevD1=prevD1/minLevel;
    } else{
      currentD1=(1024-currentD1)/minLevel;
      prevD1=(1024-prevD1)/minLevel;
    }
    const int64_t delta_d1 = static_cast<int64_t>(currentD1 - prevD1);
    const int64_t deltaSizeU0 = static_cast<int64_t>(patch.getSizeU0() - prepatch.getSizeU0());
    const int64_t deltaSizeV0 = static_cast<int64_t>(patch.getSizeV0() - prepatch.getSizeV0());

    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(delta_index)), 0, bModel0, bModelPatchIndex);
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(delta_u0)),    0, bModel0, bModelU0);
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(delta_v0)),    0, bModel0, bModelV0);
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(delta_u1)),    0, bModel0, bModelU1);
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(delta_v1)),    0, bModel0, bModelV1);
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(delta_d1)),    0, bModel0, bModelD1);
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(deltaSizeU0)), 0, bModel0, bModelIntSizeU0);
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(deltaSizeV0)), 0, bModel0, bModelIntSizeV0);

    if ( sps.getLayerAbsoluteCodingEnabledFlag( 1 )  ) { // && context.getSixDirectionMode()
      arithmeticEncoder.encode(patch.getProjectionMode(), bModel0);
    }
    
  }//end of matched patches

  o3dgc::Adaptive_Bit_Model bModelDD;
  for (size_t patchIndex = 0; patchIndex < numMatchedPatches; ++patchIndex) {
    const auto &patch    = patches[patchIndex];
    const auto &prepatch = prePatches[patch.getBestMatchIdx()];

    size_t quantDD=patch.getSizeD()==0?0:((patch.getSizeD()-1)/minLevel +1);
    size_t prevQDD=patch.getSizeD()==0?0:((prepatch.getSizeD()-1)/minLevel +1);

    const int64_t delta_dd = static_cast<int64_t>(quantDD - prevQDD);
    auto &patchTemp = patches[patchIndex];
    PCCMetadata& metadata=patchTemp.getPatchLevelMetadata();
    metadata.setMetadataPresent(true);
    metadata.setbitCountQDepth(0);
#ifdef CE210_MAXDEPTH_EVALUATION
    metadata.setQMaxDepthInPatch(delta_dd);
#endif
    metadata.setIndex(patchIndex);
    // compressMetadata(metadata, arithmeticEncoder, bModel0, bModelDD);
  }

  int64_t prevSizeU0 = patches[numMatchedPatches-1].getSizeU0();
  int64_t prevSizeV0 = patches[numMatchedPatches-1].getSizeV0();
  for (size_t patchIndex = numMatchedPatches; patchIndex < patchCount; ++patchIndex) {
    const auto &patch = patches[patchIndex];
    EncodeUInt32(uint32_t(patch.getU0()), bitCount[0], arithmeticEncoder, bModel0);
    EncodeUInt32(uint32_t(patch.getV0()), bitCount[1], arithmeticEncoder, bModel0);
    if (enable_flexible_patch_flag) {
      bool flexible_patch_present_flag = (patch.getPatchOrientation() != PatchOrientation::DEFAULT);
      if (flexible_patch_present_flag) {
        arithmeticEncoder.encode(1, orientationPatchFlagModel2);
        arithmeticEncoder.encode(patch.getPatchOrientation() - 1, orientationPatchModel);
      } else {
        arithmeticEncoder.encode(0, orientationPatchFlagModel2);
      }
    }
    EncodeUInt32(uint32_t(patch.getU1()), bitCount[2], arithmeticEncoder, bModel0);
    EncodeUInt32(uint32_t(patch.getV1()), bitCount[3], arithmeticEncoder, bModel0);
    size_t currentD1=patch.getD1();
    if(patch.getProjectionMode()==0) {
      currentD1=currentD1/minLevel;
    } else{
      currentD1=(1024-currentD1)/minLevel;
    }
    EncodeUInt32(uint32_t(currentD1), bitCount[4], arithmeticEncoder, bModel0);

    //maxDepth
    size_t quantDD=patch.getSizeD()==0?0:((patch.getSizeD()-1)/minLevel +1);
    auto &patchTemp = patches[patchIndex];
    PCCMetadata& metadata=patchTemp.getPatchLevelMetadata();
    metadata.setbitCountQDepth(maxBitCountForMaxDepth);
#ifdef CE210_MAXDEPTH_EVALUATION
    metadata.setQMaxDepthInPatch(int64_t(quantDD));
#endif
    metadata.setIndex(patchIndex);
    metadata.setMetadataPresent(true);

    if (!sps.getLayerAbsoluteCodingEnabledFlag( 1 )  && (patch.getFrameProjectionMode() == 2)) {
      const uint8_t bitCountProjDir = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(2 + 1)));
      EncodeUInt32(uint32_t(patch.getProjectionMode()), bitCountProjDir, arithmeticEncoder, bModel0);
    }
    if ( sps.getLayerAbsoluteCodingEnabledFlag( 1 )  ) { // && context.getSixDirectionMode()
      arithmeticEncoder.encode(patch.getProjectionMode(), bModel0);
    }
    const int64_t deltaSizeU0 = static_cast<int64_t>(patch.getSizeU0()) - prevSizeU0;
    const int64_t deltaSizeV0 = static_cast<int64_t>(patch.getSizeV0()) - prevSizeV0;
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(deltaSizeU0)), 0, bModel0, bModelSizeU0);
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(deltaSizeV0)), 0, bModel0, bModelSizeV0);
    prevSizeU0 = patch.getSizeU0();
    prevSizeV0 = patch.getSizeV0();
    // if (bBinArithCoding) {
      if (patch.getNormalAxis() == 0) {
        arithmeticEncoder.encode(0, orientationModel2);
      } else if (patch.getNormalAxis() == 1) {
        arithmeticEncoder.encode(1, orientationModel2);
        arithmeticEncoder.encode(0, bModel0);
      } else {
        arithmeticEncoder.encode(1, orientationModel2);
        arithmeticEncoder.encode(1, bModel0);
      }
    // } else {
    //   arithmeticEncoder.encode(uint32_t(patch.getNormalAxis()), orientationModel);
    // }
    // compressMetadata(patch.getPatchLevelMetadata(), arithmeticEncoder,bModel0, bModelDD);
  }
  for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
    const auto &patch    = patches[patchIndex];
    compressOneLayerData( context, frame, patch, arithmeticEncoder,
                          occupiedModel, interpolateModel, neighborModel, minD1Model, fillingModel );
  } //end of matched patches
#ifdef BITSTREAM_TRACE
  bitstream.trace("OccupancyMapM42195 done \n" ) ; 
#endif
}

void PCCBitstreamEncoder::compressOneLayerData( PCCContext&                 context,
                                                PCCFrameContext&            frame,
                                                const PCCPatch&             patch,
                                                o3dgc::Arithmetic_Codec&    arithmeticEncoder,
                                                o3dgc::Adaptive_Bit_Model&  occupiedModel,
                                                o3dgc::Adaptive_Bit_Model&  interpolateModel,
                                                o3dgc::Adaptive_Data_Model& neighborModel,
                                                o3dgc::Adaptive_Data_Model& minD1Model,
                                                o3dgc::Adaptive_Bit_Model&  fillingModel ) {
  auto& sps = context.getSps();
  if ( sps.getLayerAbsoluteCodingEnabledFlag( 1 ) && !sps.getMultipleLayerStreamsPresentFlag() ) {
    auto&        sps                = context.getSps();
    auto&        ops                = sps.getOccupancyParameterSet();
    auto&        blockToPatch       = frame.getBlockToPatch();
    const size_t blockToPatchWidth  = frame.getWidth() / ops.getOccupancyPackingBlockSize();
    const size_t blockToPatchHeight = frame.getHeight() / ops.getOccupancyPackingBlockSize();
    auto&        interpolateMap     = frame.getInterpolate();
    auto&        fillingMap         = frame.getFilling();
    auto&        minD1Map           = frame.getMinD1();
    auto&        neighborMap        = frame.getNeighbor();
    for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
      for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
        int pos = patch.patchBlock2CanvasBlock((u0), (v0), blockToPatchWidth, blockToPatchHeight);
        arithmeticEncoder.encode(uint32_t( blockToPatch[ pos ] > 0 ), occupiedModel);
        if( blockToPatch[ pos ] > 0 ) {
          arithmeticEncoder.encode(uint32_t(interpolateMap[ pos ]), interpolateModel);
          if( interpolateMap[ pos ] > 0 ) {
            uint32_t code = uint32_t( int(neighborMap[ pos ]) - 1 );
            arithmeticEncoder.encode( code, neighborModel);
          }
          arithmeticEncoder.encode(uint32_t(minD1Map[ pos ] ), minD1Model);
          if( minD1Map[ pos ] > 1 || interpolateMap[ pos ] > 0 ) {
            arithmeticEncoder.encode( uint32_t(fillingMap[ pos ] ), fillingModel);
          }
        }
      }
    }
  }
}

void PCCBitstreamEncoder::compressOccupancyMap( PCCContext &context, PCCBitstream& bitstream ){

  size_t sizeFrames = context.getFrames().size();
  PCCFrameContext preFrame = context.getFrames()[0];
  auto& sps = context.getSps();
  for( int i = 0; i < sizeFrames; i++ ){
    PCCFrameContext &frame = context.getFrames()[i];
    if ((sps.getLosslessGeo() || params_.lossyMissedPointsPatch_ ) && !sps.getPcmSeparateVideoPresentFlag()) {
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
      compressOccupancyMap( context, frame, bitstream, preFrame, i);
      patches.pop_back();
    } else {
      compressOccupancyMap( context, frame, bitstream, preFrame, i);
    }
    preFrame = frame;
  } 
}

void PCCBitstreamEncoder::compressOccupancyMap( PCCContext&      context,
                                                PCCFrameContext& frame,
                                                PCCBitstream&    bitstream,
                                                PCCFrameContext& preFrame,
                                                size_t           frameIndex ) {
                                                    
#ifdef BITSTREAM_TRACE
  bitstream.trace("OccupancyMap start \n" ) ; 
#endif
 
  auto&        sps        = context.getSps();
  auto&        ops        = sps.getOccupancyParameterSet();
  auto&        patches    = frame.getPatches();
  const size_t patchCount = patches.size();
  bitstream.write<uint32_t>(uint32_t(patchCount));
  bitstream.write<uint8_t>( uint8_t( params_.maxCandidateCount_));

  if (!sps.getLayerAbsoluteCodingEnabledFlag( 1 )) {
    bitstream.write<uint8_t>(uint8_t( params_.surfaceThickness_ ) );
    bitstream.write<uint8_t>(uint8_t( patches[0].getFrameProjectionMode() ) );
  }
  const size_t minLevel = sps.getMinLevel();
  const uint8_t maxBitCountForMinDepth=uint8_t(10-gbitCountSize[minLevel]);
  const uint8_t maxBitCountForMaxDepth=uint8_t(9-gbitCountSize[minLevel]); //20190129

  o3dgc::Arithmetic_Codec arithmeticEncoder;
  o3dgc::Static_Bit_Model bModel0;
  // bool bBinArithCoding = context.getBinArithCoding() && (!context.getLosslessGeo()) &&
  //     ( ops.getOccupancyPackingBlockSize() == 16) && (context.getOccupancyPrecision() == 4);
  uint8_t enable_flexible_patch_flag = ( params_.packingStrategy_ > 0);
  bitstream.write<uint8_t>(enable_flexible_patch_flag);

  arithmeticEncoder.set_buffer( 0x00ffffff );
  arithmeticEncoder.start_encoder();

  if((frameIndex == 0)||(! sps.getPatchInterPredictionEnabledFlag() ) ) {  // context.getDeltaCoding()
    size_t maxU0 = 0;
    size_t maxV0 = 0;
    size_t maxU1 = 0;
    size_t maxV1 = 0;
    size_t maxLod = 0;
    for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
      const auto &patch = patches[patchIndex];
      maxU0 = (std::max)(maxU0, patch.getU0());
      maxV0 = (std::max)(maxV0, patch.getV0());
      maxU1 = (std::max)(maxU1, patch.getU1());
      maxV1 = (std::max)(maxV1, patch.getV1());

      maxLod = (std::max)(maxLod, patch.getLod());
    }
    const uint8_t bitCountU0  = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxU0 + 1)));
    const uint8_t bitCountV0  = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxV0 + 1)));
    const uint8_t bitCountU1  = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxU1 + 1)));
    const uint8_t bitCountV1  = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxV1 + 1)));
    const uint8_t bitCountD1  = maxBitCountForMinDepth;
    const uint8_t bitCountDD  = maxBitCountForMaxDepth;
    const uint8_t bitCountLod = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxLod + 1)));

    bitstream.write<uint8_t>(bitCountU0);
    bitstream.write<uint8_t>(bitCountV0);
    bitstream.write<uint8_t>(bitCountU1);
    bitstream.write<uint8_t>(bitCountV1);

    bitstream.write<uint8_t>(bitCountLod);

    // compressMetadata(frame.getFrameLevelMetadata(), arithmeticEncoder);

    o3dgc::Adaptive_Bit_Model bModelDD;
    o3dgc::Static_Bit_Model bModel0;
    o3dgc::Adaptive_Bit_Model bModelSizeU0, bModelSizeV0, bModelAbsoluteD1;
    o3dgc::Adaptive_Data_Model orientationModel(4);
    o3dgc::Adaptive_Bit_Model orientationModel2;
    o3dgc::Adaptive_Data_Model orientationPatchModel(NumPatchOrientations - 1 + 2);
    o3dgc::Adaptive_Bit_Model orientationPatchFlagModel2;
    o3dgc::Adaptive_Bit_Model occupiedModel, interpolateModel, fillingModel;
    o3dgc::Adaptive_Data_Model minD1Model(3), neighborModel(3);
    int64_t prevSizeU0 = 0;
    int64_t prevSizeV0 = 0;
    for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
      const auto &patch = patches[patchIndex];
      EncodeUInt32(uint32_t(patch.getU0()), bitCountU0, arithmeticEncoder, bModel0);
      EncodeUInt32(uint32_t(patch.getV0()), bitCountV0, arithmeticEncoder, bModel0);
      if (enable_flexible_patch_flag) {
        bool flexible_patch_present_flag = (patch.getPatchOrientation() != PatchOrientation::DEFAULT);
        if (flexible_patch_present_flag) {
          arithmeticEncoder.encode(1, orientationPatchFlagModel2);
          arithmeticEncoder.encode(patch.getPatchOrientation()-1, orientationPatchModel);
        } else {
          arithmeticEncoder.encode(0, orientationPatchFlagModel2);
        }
      }
      EncodeUInt32(uint32_t(patch.getU1()), bitCountU1, arithmeticEncoder, bModel0);
      EncodeUInt32(uint32_t(patch.getV1()), bitCountV1, arithmeticEncoder, bModel0);
      size_t D1;
      if(patch.getProjectionMode()==0) {
        D1 = patch.getD1()/minLevel;
      } else {
        D1=1024-patch.getD1();
        D1=D1/minLevel;
      }

      EncodeUInt32(uint32_t(D1), bitCountD1, arithmeticEncoder, bModel0);

      size_t quantDD=patch.getSizeD()==0?0:((patch.getSizeD()-1)/minLevel +1);

      auto &patchTemp = patches[patchIndex];
      PCCMetadata& metadata=patchTemp.getPatchLevelMetadata();
      metadata.setbitCountQDepth(bitCountDD);
#ifdef CE210_MAXDEPTH_EVALUATION
      metadata.setQMaxDepthInPatch(int64_t(quantDD));
#endif
      metadata.setIndex(patchIndex);
      metadata.setMetadataType(METADATA_PATCH);
      metadata.setMetadataPresent(true);

      EncodeUInt32(uint32_t(patch.getLod()), bitCountLod, arithmeticEncoder, bModel0);
      if (!sps.getLayerAbsoluteCodingEnabledFlag( 1 ) && (patch.getFrameProjectionMode() == 2)) {
        const uint8_t bitCountProjDir = uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(2 + 1)));
        EncodeUInt32(uint32_t(patch.getProjectionMode()), bitCountProjDir, arithmeticEncoder, bModel0);
      }
      if ( sps.getLayerAbsoluteCodingEnabledFlag( 1 )  ) { // && context.getSixDirectionMode()
        if (patch.getProjectionMode() == 0) {
          arithmeticEncoder.encode(0, bModel0);
        } else {
          arithmeticEncoder.encode(1, bModel0);
        }
      }

      const int64_t deltaSizeU0 = static_cast<int64_t>(patch.getSizeU0()) - prevSizeU0;
      const int64_t deltaSizeV0 = static_cast<int64_t>(patch.getSizeV0()) - prevSizeV0;
      arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(deltaSizeU0)), 0, bModel0, bModelSizeU0);
      arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(deltaSizeV0)), 0, bModel0, bModelSizeV0);
      prevSizeU0 = patch.getSizeU0();
      prevSizeV0 = patch.getSizeV0();
      // if (bBinArithCoding) {
        if (patch.getNormalAxis() == 0) {
          arithmeticEncoder.encode(0, orientationModel2);
        } else if (patch.getNormalAxis() == 1) {
          arithmeticEncoder.encode(1, orientationModel2);
          arithmeticEncoder.encode(0, bModel0);
        } else {
          arithmeticEncoder.encode(1, orientationModel2);
          arithmeticEncoder.encode(1, bModel0);
        }
      // } else {
      //   arithmeticEncoder.encode(uint32_t(patch.getNormalAxis()), orientationModel);
      // }
       // compressMetadata( patch.getPatchLevelMetadata(), arithmeticEncoder, bModel0, bModelDD );
      compressOneLayerData( context, frame, patch, arithmeticEncoder,
                            occupiedModel, interpolateModel, neighborModel, minD1Model, fillingModel );
    }
  } else {
    size_t numMatchedPatches = frame.getNumMatchedPatches();
    compressPatchMetaDataM42195( context, frame, preFrame, numMatchedPatches, bitstream,
                                 arithmeticEncoder, bModel0, enable_flexible_patch_flag);
  }
  uint32_t compressedBitstreamSize = arithmeticEncoder.stop_encoder();
  bitstream.writeBuffer( arithmeticEncoder.buffer(), compressedBitstreamSize );
                                                   
#ifdef BITSTREAM_TRACE
  bitstream.trace("OccupancyMap done \n" ) ; 
#endif

}

