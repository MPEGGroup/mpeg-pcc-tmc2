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

PCCBitstreamEncoder::PCCBitstreamEncoder() {}
PCCBitstreamEncoder::~PCCBitstreamEncoder() {}

void PCCBitstreamEncoder::setParameters( PCCEncoderParameters params ) { params_ = params; }

// JR: OLD-ER
int PCCBitstreamEncoder::encode_older( PCCContext& context, PCCBitstream& bitstream ) {
  context.getBitstreamStat().newGOF();
  vpccUnit( context, bitstream, VPCC_VPS );
  vpccUnit( context, bitstream, VPCC_AD );
  vpccUnit( context, bitstream, VPCC_OVD );
  vpccUnit( context, bitstream, VPCC_GVD );
  vpccUnit( context, bitstream, VPCC_AVD );
  std::cout << " occupancy map  ->" << context.getBitstreamStat().getTotalMetadata() << " B " << std::endl;
  return 0;
}

// JR: NEW - not used now
int PCCBitstreamEncoder::write( SampleStreamNalUnit& ssnu, PCCBitstream& bitstream ) {
  printf("PCCBitstreamEncoder::write start \n"); fflush(stdout);
  ssnu.setUnitSizePrecisionBytesMinus1( 3 );  // JR: TODO fixe this value according to the size of the stored NalUnits.
  sampleStreamNalHeader( bitstream, ssnu.getUnitSizePrecisionBytesMinus1() );
  //sampleStreamNalUnit( bitstream, ssnu, ssnu.getNalUnit() );
//  for(auto& nalUnit : ssnu.getNalUnit() ){
//  sampleStreamNalUnit( bitstream, ssnu, ssnu.getNalUnit() );
//  }
  printf("PCCBitstreamEncoder::write done \n"); fflush(stdout);
  return 0;
}
int PCCBitstreamEncoder::encode_old( PCCContext& context, SampleStreamNalUnit& ssnu ) {
  printf("PCCBitstreamEncoder::encode start \n"); fflush(stdout);
  for( size_t i=0;i<6;i++){
    //auto& nalUnit = ssnu.addNalUnit();
    auto& nalUnit = ssnu.getNalUnit();
    PCCBitstream bitstream;
    context.getBitstreamStat().newGOF();
#ifdef BITSTREAM_TRACE
    bitstream.setTrace( true );
    bitstream.setTraceFile( traceFile_ );
#endif
    printf("Nalu %lu  \n",i); fflush(stdout);
    switch ( i ) {
      case 0:
        vpccUnit( context, bitstream, VPCC_VPS );
        nalUnit.initialize( NAL_VPCC_VPS, 0, 0, bitstream.size(), bitstream.buffer() );
        break;
      case 1:
        vpccUnit( context, bitstream, VPCC_AD );
        nalUnit.initialize( NAL_VPCC_AD, 0, 0, bitstream.size(), bitstream.buffer() );
        break;
      case 2:
        vpccUnit( context, bitstream, VPCC_OVD );
        nalUnit.initialize( NAL_VPCC_OVD, 0, 0, bitstream.size(), bitstream.buffer() );
        break;
      case 3:
        vpccUnit( context, bitstream, VPCC_GVD );
        nalUnit.initialize( NAL_VPCC_GVD, 0, 0, bitstream.size(), bitstream.buffer() );
        break;
      case 4:
        vpccUnit( context, bitstream, VPCC_AVD );
        nalUnit.initialize( NAL_VPCC_AVD, 0, 0, bitstream.size(), bitstream.buffer() );
        break;
      case 5: 
        // endOfAtlasSubstreamRbsp()
        nalUnit.initialize( NAL_EOB, 0, 0, 0, NULL );
        break;
    }
  }
  printf("ssnu num NalUnit %lu  \n",ssnu.getNalUnitCount()); fflush(stdout);
  printf("PCCBitstreamEncoder::encode(old) done \n"); fflush(stdout);
  return 0;
}

// DBG: NEW
//jkei : writeSampleStream updated PSTNov6. 
int PCCBitstreamEncoder::writeSampleStream( VpccUnitStream& vpccUS, PCCBitstream& bitstream){
  printf("\n***PCCBitstreamEncoder::write V-PCC Sample Stream start \n"); fflush(stdout);
  //calculating the precision of the unit size
  uint32_t maxUnitSize = 0;
 for(auto& vpccUnit : vpccUS.getVpccUnit() ){
    if (maxUnitSize < vpccUnit.getVpccUnitSize())
      maxUnitSize = (uint32_t) vpccUnit.getVpccUnitSize();
  }
  uint32_t precision = (uint32_t) (min(max((int)ceil((double)getFixedLengthCodeBitsCount(maxUnitSize) / 8.0), 1), 8)-1);
  printf("PCCBitstreamEncoder:: UnitSizePrecisionBytesMinus1 %d (= %d/8 - 1)\n",precision,getFixedLengthCodeBitsCount(maxUnitSize)); fflush(stdout);
  vpccUS.setSsvhUnitSizePrecisionBytesMinus1(precision);
  sampleStreamVpccHeader( bitstream, precision );
  size_t unitCount=0;
  for(auto& vpccUnit : vpccUS.getVpccUnit() ){
    sampleStreamVpccUnit( bitstream, vpccUS, vpccUnit);
    printf("V-PCC Unit Size(unit type:%zu, %zuth/%zu)  = %lu \n",unitCount, (size_t) vpccUnit.getVpccUnitType(), vpccUS.getVpccUnitCount(), vpccUnit.getVpccUnitSize()); fflush(stdout);
    unitCount++;
  }
  printf("PCCBitstreamEncoder::write V-PCC Sample Stream done \n"); fflush(stdout);
  return 0;
}
int PCCBitstreamEncoder::encode( PCCContext& context, VpccUnitStream& vpccus ) {
	auto& vpcc = context.getVPCC();
	auto&  sps        = context.getSps();
  printf("\n***PCCBitstreamEncoder::encode start \n"); fflush(stdout);
	context.getBitstreamStat().newGOF();
	//encode the SPS
  PCCBitstream bitstreamVPS;
#ifdef BITSTREAM_TRACE
    bitstreamVPS.setTrace( true );
    bitstreamVPS.setTraceFile( traceFile_ );
    bitstreamVPS.trace("PCCBitstreamXXcoder::XXcode(VPS)\n");
#endif
	vpccUnit( context, bitstreamVPS, VPCC_VPS );
  vpccus.addVpccUnit().initialize( bitstreamVPS.size(), bitstreamVPS.buffer(), VPCC_VPS );
  
	for (int atlasIdx = 0; atlasIdx < sps.getAtlasCountMinus1() + 1; atlasIdx++) {
		vpcc.setAtlasId(atlasIdx);
		//encode the AD
	  PCCBitstream bitstreamAD;
#ifdef BITSTREAM_TRACE
			bitstreamAD.setTrace( true );
			bitstreamAD.setTraceFile( traceFile_ );
      bitstreamAD.trace("PCCBitstreamXXcoder::XXcode(AD)\n");
#endif
		vpccUnit(context, bitstreamAD, VPCC_AD);
		vpccus.addVpccUnit().initialize( bitstreamAD.size(), bitstreamAD.buffer(), VPCC_AD);
		//encode OVD
	  PCCBitstream bitstreamOVD;
#ifdef BITSTREAM_TRACE
			bitstreamOVD.setTrace( true );
			bitstreamOVD.setTraceFile( traceFile_ );
      bitstreamOVD.trace("PCCBitstreamXXcoder::XXcode(OVD)\n");
#endif
		vpccUnit(context, bitstreamOVD, VPCC_OVD);
		vpccus.addVpccUnit().initialize( bitstreamOVD.size(), bitstreamOVD.buffer(), VPCC_OVD);
		//encode GVD
		if (sps.getMapCountMinus1(atlasIdx) > 0 && sps.getMultipleMapStreamsPresentFlag(atlasIdx)) {
			for (int mapIdx = 0; mapIdx < sps.getMapCountMinus1(atlasIdx) + 1; mapIdx++) {
				PCCBitstream bitstreamGVD;
#ifdef BITSTREAM_TRACE
				bitstreamGVD.setTrace( true );
				bitstreamGVD.setTraceFile( traceFile_ );
        bitstreamGVD.trace("PCCBitstreamXXcoder::XXcode(GVD)\n");
#endif
				//encode D(mapIdx)
				vpcc.setMapIndex(mapIdx);
				vpcc.setRawVideoFlag(0);
				vpccUnit(context, bitstreamGVD, VPCC_GVD);
				vpccus.addVpccUnit().initialize( bitstreamGVD.size(), bitstreamGVD.buffer(), VPCC_GVD);
			}
		}
		else {
			PCCBitstream bitstreamGVD;
#ifdef BITSTREAM_TRACE
			bitstreamGVD.setTrace( true );
			bitstreamGVD.setTraceFile( traceFile_ );
      bitstreamGVD.trace("PCCBitstreamXXcoder::XXcode(GVD)\n");
#endif
			//encode D=D(0)|D(1)|...|D(N)
			vpcc.setMapIndex(0);
			vpcc.setRawVideoFlag(0);
			vpccUnit(context, bitstreamGVD, VPCC_GVD);
			vpccus.addVpccUnit().initialize( bitstreamGVD.size(), bitstreamGVD.buffer(), VPCC_GVD);
		}
		if (sps.getRawPatchEnabledFlag(atlasIdx) && sps.getRawSeparateVideoPresentFlag(atlasIdx)) {
			PCCBitstream bitstreamGVD;
#ifdef BITSTREAM_TRACE
			bitstreamGVD.setTrace( true );
			bitstreamGVD.setTraceFile( traceFile_ );
      bitstreamGVD.trace("PCCBitstreamXXcoder::XXcode(GVD)\n");
#endif
			//encode RAW
			vpcc.setMapIndex(0);
			vpcc.setRawVideoFlag(1);
			vpccUnit(context, bitstreamGVD, VPCC_GVD);
			vpccus.addVpccUnit().initialize( bitstreamGVD.size(), bitstreamGVD.buffer(), VPCC_GVD);
		}
		for (int attIdx = 0; attIdx < sps.getAttributeInformation(atlasIdx).getAttributeCount(); attIdx++) {
			vpcc.setAttributeIndex(attIdx);
			for (int attDim = 0; attDim < sps.getAttributeInformation(atlasIdx).getAttributeDimensionPartitionsMinus1(attIdx) + 1; attDim++) {
				vpcc.setAttributeDimensionIndex(attDim);
				//encode AVD
				if (0){
					//How to signal that the attribute is going to be sent separately??? 
					//Should it follow the SPS, or have a syntax element of its own?
					//code below folows geometry, but software only has single stream implemented
					//if(sps.getMapCountMinus1(atlasIdx) > 0 && sps.getMultipleMapStreamsPresentFlag(atlasIdx)) {
					for (int mapIdx = 0; mapIdx < sps.getMapCountMinus1(atlasIdx) + 1; mapIdx++) {
						PCCBitstream bitstreamAVD;
#ifdef BITSTREAM_TRACE
						bitstreamAVD.setTrace( true );
						bitstreamAVD.setTraceFile( traceFile_ );
            bitstreamAVD.trace("PCCBitstreamXXcoder::XXcode(AVD)\n");
#endif
						//encode D(mapIdx)
						vpcc.setMapIndex(mapIdx);
						vpcc.setRawVideoFlag(0);
						vpccUnit(context, bitstreamAVD, VPCC_AVD);
						vpccus.addVpccUnit().initialize(bitstreamAVD.size(), bitstreamAVD.buffer(), VPCC_AVD);
					}
				}
				else {
					PCCBitstream bitstreamAVD;
#ifdef BITSTREAM_TRACE
					bitstreamAVD.setTrace( true );
					bitstreamAVD.setTraceFile( traceFile_ );
          bitstreamAVD.trace("PCCBitstreamXXcoder::XXcode(AVD)\n");
#endif
					//encode D=D(0)|D(1)|...|D(N)
					vpcc.setMapIndex(0);
					vpcc.setRawVideoFlag(0);
					vpccUnit(context, bitstreamAVD, VPCC_AVD);
					vpccus.addVpccUnit().initialize(bitstreamAVD.size(), bitstreamAVD.buffer(), VPCC_AVD);
				}
				if (sps.getRawPatchEnabledFlag(atlasIdx) && sps.getRawSeparateVideoPresentFlag(atlasIdx)) {
					PCCBitstream bitstreamAVD;
#ifdef BITSTREAM_TRACE
					bitstreamAVD.setTrace( true );
					bitstreamAVD.setTraceFile( traceFile_ );
          bitstreamAVD.trace("PCCBitstreamXXcoder::XXcode(AVD)\n");
#endif
					//encode RAW
					vpcc.setMapIndex(0);
					vpcc.setRawVideoFlag(1);
					vpccUnit(context, bitstreamAVD, VPCC_AVD);
					vpccus.addVpccUnit().initialize(bitstreamAVD.size(), bitstreamAVD.buffer(), VPCC_AVD);
				}
			}
		}
	}
	printf("V-PCC Unit stream, num V-PCC Unit %lu  \n", vpccus.getVpccUnitCount()); fflush(stdout);
	printf("PCCBitstreamEncoder::encode done \n"); fflush(stdout);
  return 0;
}

/*
OLD FUNCTION, should be removed if function below is accepted
void PCCBitstreamEncoder::videoSubStream( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&  sps        = context.getSps();
  size_t atlasIndex = 0;
  if ( vpccUnitType == VPCC_OVD ) {
    TRACE_BITSTREAM( "OccupancyMap \n" );
    bitstream.write( context.getVideoBitstream( VIDEO_OCCUPANCY ) );
    context.getBitstreamStat().setVideoBinSize( VIDEO_OCCUPANCY, context.getVideoBitstream( VIDEO_OCCUPANCY ).size() );
  } else if ( vpccUnitType == VPCC_GVD ) {
    TRACE_BITSTREAM( "Geometry \n" );
    if ( sps.getMapCountMinus1( atlasIndex ) > 0 && !sps.getMapAbsoluteCodingEnableFlag( atlasIndex, 1 ) ) {
      bitstream.write( context.getVideoBitstream( VIDEO_GEOMETRY_D0 ) ); 
      bitstream.write( context.getVideoBitstream( VIDEO_GEOMETRY_D1 ) ); 
      context.getBitstreamStat().setVideoBinSize( VIDEO_GEOMETRY_D0, context.getVideoBitstream( VIDEO_GEOMETRY_D0 ).size() );
      context.getBitstreamStat().setVideoBinSize( VIDEO_GEOMETRY_D1, context.getVideoBitstream( VIDEO_GEOMETRY_D1 ).size() );
    } else {
      bitstream.write( context.getVideoBitstream( VIDEO_GEOMETRY ) ); 
      context.getBitstreamStat().setVideoBinSize( VIDEO_GEOMETRY, context.getVideoBitstream( VIDEO_GEOMETRY ).size() );
    }
    if ( sps.getRawPatchEnabledFlag( atlasIndex ) && sps.getRawSeparateVideoPresentFlag( atlasIndex ) ) {
      bitstream.write( context.getVideoBitstream( VIDEO_GEOMETRY_MP ) ); //this is wrong (this should be on a separate v-pcc unit)
      context.getBitstreamStat().setVideoBinSize( VIDEO_GEOMETRY_MP, context.getVideoBitstream( VIDEO_GEOMETRY_MP ).size() );
    }
  } else if ( vpccUnitType == VPCC_AVD ) {
    if ( sps.getAttributeInformation( atlasIndex ).getAttributeCount() > 0 ) {
      TRACE_BITSTREAM( "Texture \n" );
      bitstream.write( context.getVideoBitstream( VIDEO_TEXTURE ) );
      context.getBitstreamStat().setVideoBinSize( VIDEO_TEXTURE, context.getVideoBitstream( VIDEO_TEXTURE ).size() );
      if ( sps.getRawPatchEnabledFlag( atlasIndex ) && sps.getRawSeparateVideoPresentFlag( atlasIndex ) ) {
        bitstream.write( context.getVideoBitstream( VIDEO_TEXTURE_MP ) );
      context.getBitstreamStat().setVideoBinSize( VIDEO_TEXTURE_MP, context.getVideoBitstream( VIDEO_TEXTURE_MP ).size() );
      }
    }
  }
}
*/
void PCCBitstreamEncoder::videoSubStream( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&  sps        = context.getSps();
  auto&  vpcc       = context.getVPCC();
  size_t atlasIndex = context.getVPCC().getAtlasId();
  if ( vpccUnitType == VPCC_OVD ) {
    TRACE_BITSTREAM( "OccupancyMap \n" );
    bitstream.write( context.getVideoBitstream( VIDEO_OCCUPANCY ) );
    context.getBitstreamStat().setVideoBinSize( VIDEO_OCCUPANCY, context.getVideoBitstream( VIDEO_OCCUPANCY ).size() );
  } else if ( vpccUnitType == VPCC_GVD ) {
    TRACE_BITSTREAM( "Geometry \n" );
		// this function should return the desired video bitstream ????
		// bitstream.write(context.getVideoBitstream(atlasIndex,vpcc.getMapIndex(),sps.getMultipleMapStreamsPresentFlag(atlasIndex),vpcc.getRawVideoFlag(),VIDEO_GEOMETRY)); 
		// need to change this to allow for more maps, but for now only 2 are used in the software anyways
    if ( sps.getMapCountMinus1( atlasIndex ) > 0 && sps.getMultipleMapStreamsPresentFlag(atlasIndex) ) {
			if (vpcc.getMapIndex() == 0)
			{
				bitstream.write(context.getVideoBitstream(VIDEO_GEOMETRY_D0));
				context.getBitstreamStat().setVideoBinSize(VIDEO_GEOMETRY_D0, context.getVideoBitstream(VIDEO_GEOMETRY_D0).size());
			}
			else {
				if (vpcc.getMapIndex() == 1)
				{
					bitstream.write(context.getVideoBitstream(VIDEO_GEOMETRY_D1));
					context.getBitstreamStat().setVideoBinSize(VIDEO_GEOMETRY_D1, context.getVideoBitstream(VIDEO_GEOMETRY_D1).size());
				}
			}
    } else {
      bitstream.write( context.getVideoBitstream( VIDEO_GEOMETRY ) ); 
      context.getBitstreamStat().setVideoBinSize( VIDEO_GEOMETRY, context.getVideoBitstream( VIDEO_GEOMETRY ).size() );
    }
    if ( vpcc.getRawVideoFlag() ) {
      bitstream.write( context.getVideoBitstream( VIDEO_GEOMETRY_MP ) ); 
      context.getBitstreamStat().setVideoBinSize( VIDEO_GEOMETRY_MP, context.getVideoBitstream( VIDEO_GEOMETRY_MP ).size() );
    }
  } else if ( vpccUnitType == VPCC_AVD ) {
    if ( sps.getAttributeInformation( atlasIndex ).getAttributeCount() > 0 ) {
      TRACE_BITSTREAM( "Texture \n" );
			// this function should return the desired video bitstream ????
			// bitstream.write(context.getVideoBitstream(atlasIndex,vpcc.getAttributeIndex(),vpcc.getMapIndex(),sps.getMultipleMapStreamsPresentFlag(atlasIndex),vpcc.getRawVideoFlag(),VIDEO_TEXTURE)); 
			// need to change this to allow for more maps, but for now only 2 are used in the software anyways
		  if (0){// sps.getMapCountMinus1( atlasIndex ) > 0 && sps.getMultipleMapStreamsPresentFlag(atlasIndex) ) {
				/*
				NOT IMPLEMENTED - check definition of separate texture map coding
				*/
			} else {
				bitstream.write( context.getVideoBitstream( VIDEO_TEXTURE ) ); 
				context.getBitstreamStat().setVideoBinSize( VIDEO_TEXTURE, context.getVideoBitstream( VIDEO_TEXTURE ).size() );
			}
      if ( vpcc.getRawVideoFlag() ) {
        bitstream.write( context.getVideoBitstream( VIDEO_TEXTURE_MP ) );
        context.getBitstreamStat().setVideoBinSize( VIDEO_TEXTURE_MP, context.getVideoBitstream( VIDEO_TEXTURE_MP ).size() );
      }
    }
  }
}

// 7.3.2.1 General V-PCC unit syntax
void PCCBitstreamEncoder::vpccUnitInSampleStream( PCCBitstream& bitstream, VpccUnit& vpccUnit) {
  //int32_t numBytesInVPCCUnit= (int32_t) vpccUnit.getVpccUnitSize();
  //jkei: I don't understand this part clearly... :'(
  //jkei: just write #numBytesInVPCCUnit to the bitstream
  //B.2.2  Sample stream V-PCC unit syntax vpcc_unit(ssvu_vpcc_unit_size)
#if 1
  for ( size_t i = 0; i < vpccUnit.getVpccUnitSize(); i++ ) {
    bitstream.write( uint32_t( vpccUnit.getVpccUnitData( i ) ), 8, true );  // b(8)
#if 1
#endif
  }
#else
	//vpcc_unit_header() -> vpccUnit.writeHeader()
  for ( size_t i = 0; i < 4; i++ ) {
    bitstream.write( uint32_t( vpccUnit.getVpccUnitData( i ) ), 8 );  // b(8)
  }
	//vpp_unit_payload() -> vpccUnit.writePayload()
  for ( size_t i = 4; i < vpccUnit.getVpccUnitSize(); i++ ) {
    bitstream.write( uint32_t( vpccUnit.getVpccUnitData( i ) ), 8 );  // b(8)
  }
	numBytesInVPCCUnit -= vpccUnit.getVpccUnitSize();
	while (numBytesInVPCCUnit > 0) {
		bitstream.write( 0, 8 ); //f(8): trailing zero bits
		--numBytesInVPCCUnit;
	}
#endif
}
void PCCBitstreamEncoder::vpccUnit( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  int32_t position = (int32_t)bitstream.size();
  vpccUnitHeader( context, bitstream, vpccUnitType );
  vpccUnitPayload( context, bitstream, vpccUnitType );
  // while( moreDataInVpccUnit() ) { bitstream.write( 0, 8 ); }
  context.getBitstreamStat().setVpccUnitSize( vpccUnitType, (int32_t)bitstream.size() - position );
  TRACE_BITSTREAM( "%s done\n", __func__ );
}

// 7.3.2.2 V-PCC unit header syntax
void PCCBitstreamEncoder::vpccUnitHeader( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& vpcc = context.getVPCC();
  bitstream.write( vpccUnitType, 5 );  // u(5)
  if ( vpccUnitType == VPCC_AVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_AD ) {
    bitstream.write( (uint32_t)vpcc.getVpccParameterSetId(), 4 );  // u(4)
    bitstream.write( (uint32_t)vpcc.getAtlasId(), 6 );             // u(6)
  }
  if ( vpccUnitType == VPCC_AVD ) {
    bitstream.write( (uint32_t)vpcc.getAttributeIndex(), 7 );           // u(7)
    bitstream.write( (uint32_t)vpcc.getAttributeDimensionIndex(), 5 );  // u(5)
    bitstream.write( (uint32_t)vpcc.getMapIndex(), 4 );                 // u(4)
    bitstream.write( (uint32_t)vpcc.getRawVideoFlag(), 1 );             // u(1)

  } else if ( vpccUnitType == VPCC_GVD ) {
    bitstream.write( (uint32_t)vpcc.getMapIndex(), 4 );      // u(4)
    bitstream.write( (uint32_t)vpcc.getRawVideoFlag(), 1 );  // u(1)
    bitstream.write( (uint32_t)0, 12 );                      // u(12)

  } else if ( vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_AD ) {
    bitstream.write( (uint32_t)0, 17 );  // u(17)
  } else {
    bitstream.write( (uint32_t)0, 27 );  // u(27)
  }
}

// 7.3.2.3 V-PCC unit payload syntax
void PCCBitstreamEncoder::vpccUnitPayload( PCCContext& context, PCCBitstream& bitstream, VPCCUnitType vpccUnitType ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& sps = context.getSps();
  TRACE_BITSTREAM( "vpccUnitType = %d \n", (int32_t)vpccUnitType );
  if ( vpccUnitType == VPCC_VPS ) {
    vpccParameterSet( sps, context, bitstream );
  } else if ( vpccUnitType == VPCC_AD ) {
    atlasSubStream( context, bitstream );
  } else if ( vpccUnitType == VPCC_OVD || vpccUnitType == VPCC_GVD || vpccUnitType == VPCC_AVD ) {
    videoSubStream( context, bitstream, vpccUnitType );
  }
}

// 7.3.3 Byte alignment syntax
void PCCBitstreamEncoder::byteAlignment( PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( 1, 1 );  // f(1): equal to 1
  while ( !bitstream.byteAligned() ) {
    bitstream.write( 0, 1 );  // f(1): equal to 0
  }
}

// 7.3.4.1 General V-PCC parameter set syntax
void PCCBitstreamEncoder::vpccParameterSet( VpccParameterSet& vps, PCCContext& context, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );

  profileTierLevel( vps.getProfileTierLevel(), bitstream );
  bitstream.write( (uint32_t)vps.getVpccParameterSetId(), 4 );  // u(4)
  bitstream.write( (uint32_t)vps.getAtlasCountMinus1(), 6 );    // u(6)
  for ( int j = 0; j < vps.getAtlasCountMinus1() + 1; j++ ) {
    TRACE_BITSTREAM( "Atlas = %lu \n", j );
    bitstream.write( (uint32_t)vps.getFrameWidth( j ), 16 );     // u(16)
    bitstream.write( (uint32_t)vps.getFrameHeight( j ), 16 );    // u(16)
    bitstream.write( (uint32_t)vps.getMapCountMinus1( j ), 4 );  // u(4)
    TRACE_BITSTREAM( " MapCountMinus1 = %lu \n", vps.getMapCountMinus1( j ) );
    if ( vps.getMapCountMinus1( j ) > 0 ) {
      bitstream.write( (uint32_t)vps.getMultipleMapStreamsPresentFlag( j ), 1 );  // u(1)
      TRACE_BITSTREAM( "MultipleMapStreamsPresentFlag = %lu \n", vps.getMultipleMapStreamsPresentFlag( j ) );
    }
    for ( size_t i = 1; i <= vps.getMapCountMinus1( j ); i++ ) {
      if ( vps.getMultipleMapStreamsPresentFlag( j ) ) {
        bitstream.write( (uint32_t)vps.getMapAbsoluteCodingEnableFlag( j, i ), 1 );  // u(1)
      }
      if ( vps.getMapAbsoluteCodingEnableFlag( j, i ) == 0 ) {
        //if ( i > 0 ) {
          bitstream.writeUvlc( (uint32_t)vps.getMapPredictorIndexDiff( j, i ) );  // ue(v)
        //}
      }
    }
    bitstream.write( (uint32_t)vps.getRawPatchEnabledFlag( j ), 1 );  // u(1)
    TRACE_BITSTREAM( " RawPatchEnabledFlag = %lu \n", vps.getRawPatchEnabledFlag( j ) );
    if ( vps.getRawPatchEnabledFlag( j ) ) {
      bitstream.write( (uint32_t)vps.getRawSeparateVideoPresentFlag( j ), 1 );  // u(1)
      TRACE_BITSTREAM( " RawSeparateVideoPresentFlag = %lu \n", vps.getRawSeparateVideoPresentFlag( j ) );
    }
#ifdef BITSTREAM_TRACE
    for ( size_t i = 0; i < vps.getMapCountMinus1( j ) + 1; i++ ) {
      TRACE_BITSTREAM( " AbsoluteCoding L%lu = %lu \n", i, vps.getMapAbsoluteCodingEnableFlag( j, i ) );
    }
#endif
    occupancyInformation( vps.getOccupancyInformation( j ), bitstream );
    geometryInformation( vps.getGeometryInformation( j ), vps, bitstream );
    attributeInformation( vps.getAttributeInformation( j ), vps, bitstream );
  }
  bitstream.write( (uint32_t)vps.getExtensionPresentFlag(), 1 );  // u(1)
  if ( vps.getExtensionPresentFlag() ) {
    bitstream.writeUvlc( (uint32_t)vps.getExtensionLength() );  // ue(v)
    for ( size_t i = 0; i < vps.getExtensionLength(); i++ ) {
      bitstream.write( (uint32_t)vps.getExtensionDataByte( i ), 8 );  // u(8)
    }
  }

  // THE NEXT PARAMETERS ARE NOT IN THE VPCC CD SYNTAX DOCUMENTS AND WILL BE REMOVE
#ifdef BITSTREAM_TRACE
  bitstream.trace( "  Deprecated1\n" );
#endif
  bitstream.write( (uint32_t)vps.getLosslessGeo444(), 1 );  // u(1) TODO: remove?
  bitstream.write( (uint32_t)vps.getLosslessGeo(), 1 );     // u(1) TODO: remove?

  bitstream.write( (uint32_t)vps.getMinLevel(), 8 );          // u(8) TODO: remove?
  bitstream.write( (uint32_t)vps.getSurfaceThickness(), 8 );  // u(8) TODO: remove?

  // THE NEXT PARAMETERS ARE NOT IN THE VPCC CD SYNTAX DOCUMENTS AND WILL BE REMOVE
  byteAlignment( bitstream );
}

// 7.3.4.2 Profile, tier, and level syntax
void PCCBitstreamEncoder::profileTierLevel( ProfileTierLevel& ptl, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( (uint32_t)ptl.getTierFlag(), 1 );                  // u(1)
  bitstream.write( (uint32_t)ptl.getProfileCodecGroupIdc(), 7 );      // u(7)
  bitstream.write( (uint32_t)ptl.getProfilePccToolsetIdc(), 8 );      // u(8)
  bitstream.write( (uint32_t)ptl.getProfileReconctructionIdc(), 8 );  // u(8)
  bitstream.write( (uint32_t)0, 32 );                                 // u(32)
  bitstream.write( (uint32_t)ptl.getLevelIdc(), 8 );                  // u(8)
}

// 7.3.4.3 Occupancy parameter set syntax
void PCCBitstreamEncoder::occupancyInformation( OccupancyInformation& oi, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( (uint32_t)oi.getOccupancyCodecId(), 8 );                       // u(8)
  bitstream.write( (uint32_t)oi.getLossyOccupancyMapCompressionThreshold(), 8 );  // u(8)
  bitstream.write( (uint32_t)oi.getOccupancyNominal2DBitdepthMinus1(), 5 );       // u(5)
  bitstream.write( (uint32_t)oi.getOccupancyMSBAlignFlag(), 1 );                  // u(1)
  TRACE_BITSTREAM( "  OccupancyLossyThreshold = %d  \n", oi.getLossyOccupancyMapCompressionThreshold() );
}

// 7.3.4.4 Geometry parameter set syntax
void PCCBitstreamEncoder::geometryInformation( GeometryInformation& gi,
                                               VpccParameterSet&    vps,
                                               PCCBitstream&        bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  bitstream.write( (uint32_t)gi.getGeometryCodecId(), 8 );                            // u(8)
  bitstream.write( (uint32_t)gi.getGeometryNominal2dBitdepthMinus1(), 5 );            // u(5)
  bitstream.write( (uint32_t)gi.getGeometryMSBAlignFlag(), 1 );                       // u(1)
  bitstream.write( ( uint32_t )( gi.getGeometry3dCoordinatesBitdepthMinus1() ), 5 );  // u(5)
  if ( vps.getRawSeparateVideoPresentFlag( atlasIndex ) ) {
    bitstream.write( (uint32_t)gi.getRawGeometryCodecId(), 8 );  // u(8)
  }
  // JR TODO:  Remove
  bitstream.write( (uint32_t)gi.getGeometryParamsEnabledFlag(), 1 );       // u(1)
  bitstream.write( (uint32_t)gi.getGeometryPatchParamsEnabledFlag(), 1 );  // u(1)
  TRACE_BITSTREAM( "GeometryParamsEnabledFlag = %d \n", gi.getGeometryParamsEnabledFlag() );
  TRACE_BITSTREAM( "GeometryPatchParamsEnabledFlag = %d \n", gi.getGeometryPatchParamsEnabledFlag() );
}

// 7.3.4.5 Attribute information
void PCCBitstreamEncoder::attributeInformation( AttributeInformation& ai,
                                                VpccParameterSet&     sps,
                                                PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  bitstream.write( (uint32_t)ai.getAttributeCount(), 7 );  // u(7)
  TRACE_BITSTREAM( "AttributeCount = %u  \n", ai.getAttributeCount() );
  for ( uint32_t i = 0; i < ai.getAttributeCount(); i++ ) {
    bitstream.write( (uint32_t)ai.getAttributeTypeId( i ), 4 );   // u(4)
    bitstream.write( (uint32_t)ai.getAttributeCodecId( i ), 8 );  // u(8)
    if ( sps.getRawSeparateVideoPresentFlag( atlasIndex ) ) {
      bitstream.write( (uint32_t)ai.getRawAttributeCodecId( i ), 8 );  // u(8)
    }
    for ( int32_t j = 0; j < sps.getMapCountMinus1( atlasIndex ); j++ ) {
      if ( sps.getMapAbsoluteCodingEnableFlag( atlasIndex, j ) == 0 ) {
        bitstream.write( (uint32_t)ai.getAttributeMapAbsoluteCodingEnabledFlag( i ), 1 );  // u(1)
      }
    }
    bitstream.write( (uint32_t)ai.getAttributeDimensionMinus1( i ), 6 );  // u(6)

    if ( ai.getAttributeDimensionMinus1( i ) > 0 ) {
      bitstream.write( (uint32_t)ai.getAttributeDimensionPartitionsMinus1( i ), 6 );  // u(6)
      int32_t remainingDimensions = ai.getAttributeDimensionMinus1( i );
      int32_t k                   = ai.getAttributeDimensionPartitionsMinus1( i );
      for ( int32_t j = 0; j < k; j++ ) {
        if ( k - j != remainingDimensions ) {
          bitstream.writeUvlc( (uint32_t)ai.getAttributePartitionChannelsMinus1( i, j ) );  // ue(v)
        }
        remainingDimensions -= ai.getAttributePartitionChannelsMinus1( i, j ) + 1;
      }
    }
    bitstream.write( (uint32_t)ai.getAttributeNominal2dBitdepthMinus1( i ), 5 );  // u(5)
  }
  if ( ai.getAttributeCount() > 0 ) {
    bitstream.write( (uint32_t)ai.getAttributeMSBAlignFlag(), 1 );  // u(1)
  }
}

// 7.3.5 NAL unit syntax
// 7.3.5.1 General NAL unit syntax
void PCCBitstreamEncoder::nalUnit( PCCBitstream& bitstream, NalUnit& nalUnit) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  nalUnitHeader( bitstream, nalUnit );
  for ( size_t i = 0; i < nalUnit.getNalUnitSize() - 2; i++ ) {
    bitstream.write( uint32_t( nalUnit.getNalUnitData( i ) ), 8 );  // b(8)
  }
}

// 7.3.5.2 NAL unit header syntax
void PCCBitstreamEncoder::nalUnitHeader( PCCBitstream& bitstream, NalUnit& nalUnit ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( 0, 1 );                                          // f(1)
  bitstream.write( uint32_t( nalUnit.getNalUnitType() ), 6 );       // u(6)
  bitstream.write( uint32_t( nalUnit.getLayerId() ), 6 );           // u(6)
  bitstream.write( uint32_t( nalUnit.getTemporalyIdPlus1() ), 3 );  // u(3)
  TRACE_BITSTREAM( " NalUnitSize      = %lu \n", nalUnit.getNalUnitSize());
  TRACE_BITSTREAM( " NalUnitType      = %hhu \n",nalUnit.getNalUnitType());
  TRACE_BITSTREAM( " LayerId          = %hhu \n",nalUnit.getLayerId());
  TRACE_BITSTREAM( " TemporalyIdPlus1 = %hhu \n",nalUnit.getTemporalyIdPlus1());

}

// 7.3.6.1 Atlas sequence parameter set RBSP
void PCCBitstreamEncoder::atlasSequenceParameterSetRBSP( AtlasSequenceParameterSetRBSP& asps,
                                                         PCCContext&                    context,
                                                         PCCBitstream&                  bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.writeUvlc( (uint32_t)asps.getAltasSequenceParameterSetId() );         // ue(v)
  bitstream.write( (uint32_t)asps.getFrameWidth(), 16 );                          // u(16)
  bitstream.write( (uint32_t)asps.getFrameHeight(), 16 );                         // u(16)
  bitstream.write( (uint32_t)asps.getLog2PatchPackingBlockSize(), 3 );            // u(3)
  bitstream.writeUvlc( (uint32_t)asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() );  // ue(v)
  bitstream.writeUvlc( (uint32_t)asps.getMaxDecAtlasFrameBufferingMinus1() );     // ue(v)
  bitstream.write( (uint32_t)asps.getLongTermRefAtlasFramesFlag(), 1 );           // u(1)
  bitstream.writeUvlc( (uint32_t)asps.getNumRefAtlasFrameListsInAsps() );         // ue(v)
  for ( size_t i = 0; i < asps.getNumRefAtlasFrameListsInAsps(); i++ ) {
    refListStruct( asps.getRefListStruct( i ), asps, bitstream );
  }
  bitstream.write( (uint32_t)asps.getUseEightOrientationsFlag(), 1 );                 // u(1)
  bitstream.write( (uint32_t)asps.get45DegreeProjectionPatchPresentFlag(), 1 );       // u(1)
  bitstream.write( (uint32_t)asps.getNormalAxisLimitsQuantizationEnabledFlag(), 1 );  // u(1)
  bitstream.write( (uint32_t)asps.getNormalAxisMaxDeltaValueEnabledFlag(), 1 );       // u(1)
  bitstream.write( (uint32_t)asps.getRemoveDuplicatePointEnabledFlag(), 1 );          // u(1)
  bitstream.write( (uint32_t)asps.getPixelDeinterleavingFlag(), 1 );                  // u(1)
  bitstream.write( (uint32_t)asps.getPatchPrecedenceOrderFlag(), 1 );                 // u(1)
  bitstream.write( (uint32_t)asps.getPatchSizeQuantizerPresentFlag(), 1 );            // u(1)
  bitstream.write( (uint32_t)asps.getEnhancedOccupancyMapForDepthFlag(), 1 );         // u(1)
  bitstream.write( (uint32_t)asps.getPointLocalReconstructionEnabledFlag(), 1 );      // u(1)

  bitstream.write( (uint32_t)asps.getMapCountMinus1(), 4 );  // u(4)
  if ( asps.getEnhancedOccupancyMapForDepthFlag() && asps.getMapCountMinus1() == 0 ) {
    bitstream.write( (uint32_t)asps.getEnhancedOccupancyMapFixBitCountMinus1(), 4 );  // u(4)
  }
  if ( asps.getPointLocalReconstructionEnabledFlag() ) {
    asps.allocatePointLocalReconstructionInformation();
    pointLocalReconstructionInformation( asps, context, bitstream );
  }
  if ( asps.getPixelDeinterleavingFlag() || asps.getPointLocalReconstructionEnabledFlag() ) {
    bitstream.write( (uint32_t)asps.getSurfaceThicknessMinus1(), 8 );  // u(8)
  }
  bitstream.write( (uint32_t)asps.getVuiParametersPresentFlag(), 1 );  // u(1)
  if ( asps.getVuiParametersPresentFlag() ) {
    vuiParameters();  // TODO
  }
    TRACE_BITSTREAM( " VuiParametersPresentFlag: %zu\n", (size_t)asps.getVuiParametersPresentFlag() );
  
  bitstream.write( (uint32_t)asps.getExtensionPresentFlag(), 1 );  // u(1)
  if ( asps.getExtensionPresentFlag() ) {
    while ( moreRbspData( bitstream ) ) {
      bitstream.write( (uint32_t)asps.getExtensionPresentFlag(), 1 );  // u(1)
    }
  }
  rbspTrailingBits( bitstream );
}

// 7.3.4.6 Point local reconstruction information syntax (OLD)
void PCCBitstreamEncoder::pointLocalReconstructionInformation( PointLocalReconstructionInformation& plri,
                                                               PCCContext&                          context,
                                                               PCCBitstream&                        bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( uint32_t( plri.getNumberOfModesMinus1() ), 4 );  // u(4)
  TRACE_BITSTREAM( "  NumberOfModesMinus1 = %u \n", plri.getNumberOfModesMinus1() );
  for ( size_t i = 0; i < plri.getNumberOfModesMinus1(); i++ ) {
    bitstream.write( uint32_t( plri.getInterpolateFlag( i ) ), 1 );  // u(1)
    bitstream.write( uint32_t( plri.getFillingFlag( i ) ), 1 );      // u(1)
    bitstream.write( uint32_t( plri.getMinimumDepth( i ) ), 2 );     // u(2)
    bitstream.write( uint32_t( plri.getNeighbourMinus1( i ) ), 2 );  // u(2)
    TRACE_BITSTREAM( "  Mode[%lu] = I = %d F = %d D = %u N = %u \n", i, plri.getInterpolateFlag( i ),
                     plri.getFillingFlag( i ), plri.getMinimumDepth( i ), plri.getNeighbourMinus1( i ) );
  }
  bitstream.writeUvlc( uint32_t( plri.getBlockThresholdPerPatchMinus1() ) );  // ue(v)
  TRACE_BITSTREAM( "  BlockThresholdPerPatchMinus1 = %u \n", plri.getBlockThresholdPerPatchMinus1() );
}

// 7.3.4.6 Point local reconstruction information syntax (NEW)
void PCCBitstreamEncoder::pointLocalReconstructionInformation( AtlasSequenceParameterSetRBSP& asps,
                                                               PCCContext&                    context,
                                                               PCCBitstream&                  bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  for ( size_t j = 0; j < asps.getMapCountMinus1() + 1; j++ ) {
    auto& plri = asps.getPointLocalReconstructionInformation( j );
    bitstream.write( (uint32_t)plri.getMapEnabledFlag(), 1 );  // u(1)
    if ( plri.getMapEnabledFlag() ) {
      bitstream.write( uint32_t( plri.getNumberOfModesMinus1() ), 4 );  // u(4)
      TRACE_BITSTREAM( "  NumberOfModesMinus1 = %u \n", plri.getNumberOfModesMinus1() );
      for ( size_t i = 0; i < plri.getNumberOfModesMinus1(); i++ ) {
        bitstream.write( (uint32_t)plri.getInterpolateFlag( i ), 1 );  // u(1)
        bitstream.write( (uint32_t)plri.getFillingFlag( i ), 1 );      // u(1)
        bitstream.write( (uint32_t)plri.getMinimumDepth( i ), 2 );     // u(2)
        bitstream.write( (uint32_t)plri.getNeighbourMinus1( i ), 2 );  // u(2)
        TRACE_BITSTREAM( "  Mode[%lu] = I = %d F = %d D = %u N = %u \n", i, plri.getInterpolateFlag( i ),
                         plri.getFillingFlag( i ), plri.getMinimumDepth( i ), plri.getNeighbourMinus1( i ) );
      }
      bitstream.write( (uint32_t)plri.getBlockThresholdPerPatchMinus1(), 6 );  // u(6)
      TRACE_BITSTREAM( "  BlockThresholdPerPatchMinus1 = %u \n", plri.getBlockThresholdPerPatchMinus1() );
    }
  }
}

// jkei: newly added PST:Oct30th
bool PCCBitstreamEncoder::moreRbspData( PCCBitstream& bitstream ) {
  // jkei: TODO: need to fill this part
  return false;
}
void PCCBitstreamEncoder::rbspTrailingBits( PCCBitstream& bitstream ) {
  // jkei: TODO: need to fill this part
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( 1 , 1);  // f(1): equal to 1
  while ( !bitstream.byteAligned() ) {
    bitstream.write( 1 , 1);  // f(1): equal to 0
  }
}

// 7.3.6.3  Atlas frame parameter set RBSP syntax
void PCCBitstreamEncoder::atlasFrameParameterSetRbsp( AtlasFrameParameterSetRbsp& afps,
                                                      PCCContext&                 context,
                                                      PCCBitstream&               bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.writeUvlc( afps.getAtlasFrameParameterSetId() );    // ue(v)
  bitstream.writeUvlc( afps.getAtlasSequenceParameterSetId() ); // ue(v)
  atlasFrameTileInformation( afps.getAtlasFrameTileInformation(), context.getSps(), bitstream );

  TRACE_BITSTREAM( "-> back to %s \n", __func__ );
  bitstream.writeUvlc( uint32_t( afps.getAfpsNumRefIdxDefaultActiveMinus1() ) ); // ue(v)
  bitstream.writeUvlc( uint32_t( afps.getAfpsAdditionalLtAfocLsbLen() ) );       // ue(v)
  bitstream.write( uint32_t( afps.getAfps2dPosXBitCountMinus1() ), 4 );          // u(4)
  bitstream.write( uint32_t( afps.getAfps2dPosYBitCountMinus1() ), 4 );          // u(4)
  bitstream.write( uint32_t( afps.getAfps3dPosXBitCountMinus1() ), 5 );          // u(5)
  bitstream.write( uint32_t( afps.getAfps3dPosYBitCountMinus1() ), 5 );          // u(5)
  //bitstream.write( uint32_t( afps.getLodBitCount() ), 5 );                     // u(5)
  bitstream.write( uint32_t( afps.getAfpsOverrideEomForDepthFlag() ), 1 );       // u(1)
  TRACE_BITSTREAM(" NumRefIdxDefaultActiveMinus1: %zu\n", (size_t)afps.getAfpsNumRefIdxDefaultActiveMinus1());
  TRACE_BITSTREAM(" AdditionalLtAfocLsbLen:       %zu\n", (size_t)afps.getAfpsAdditionalLtAfocLsbLen());
  TRACE_BITSTREAM(" 2dPosXBitCountMinus1: %zu\n", (size_t)afps.getAfps2dPosXBitCountMinus1());
  TRACE_BITSTREAM(" 2dPosYBitCountMinus1: %zu\n", (size_t)afps.getAfps2dPosYBitCountMinus1());
  TRACE_BITSTREAM(" 3dPosXBitCountMinus1: %zu\n", (size_t)afps.getAfps3dPosXBitCountMinus1());
  TRACE_BITSTREAM(" 3dPosYBitCountMinus1: %zu\n", (size_t)afps.getAfps3dPosYBitCountMinus1());
  
  
  if ( afps.getAfpsOverrideEomForDepthFlag() ) {
    bitstream.write( uint32_t( afps.getAfpsEomNumberOfPatchBitCountMinus1() ), 4 );
    bitstream.write( uint32_t( afps.getAfpsEomMaxBitCountMinus1() ), 4 );
  }
  bitstream.write( uint32_t( afps.getAfpsRaw3dPosBitCountExplicitModeFlag() ), 1 );
  bitstream.write( uint32_t( afps.getAfpsExtensionPresentFlag() ), 1 );
  if ( afps.getAfpsExtensionPresentFlag() ) {
    while ( moreRbspData( bitstream ) ) { bitstream.write( uint32_t( afps.getAfpsExtensionDataFlag() ), 1 ); }
  }
  rbspTrailingBits( bitstream );
}

// 7.3.6.4  Atlas frame tile information syntax
void PCCBitstreamEncoder::atlasFrameTileInformation( AtlasFrameTileInformation& afti,
                                                     VpccParameterSet&          sps,
                                                     PCCBitstream&              bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( afti.getSingleTileInAtlasFrameFlag(), 1 );  // u(1)
  if ( !afti.getSingleTileInAtlasFrameFlag() ) {
    bitstream.write( afti.getUniformTileSpacingFlag(), 1 );    // u(1)
    if ( afti.getUniformTileSpacingFlag() ) {
      bitstream.writeUvlc( afti.getTileColumnWidthMinus1( 0 ) );  //  ue(v)
      bitstream.writeUvlc( afti.getTileRowHeightMinus1( 0 ) );    //  ue(v)
    } else {
      bitstream.writeUvlc( afti.getNumTileColumnsMinus1() );        //  ue(v)
      bitstream.writeUvlc( afti.getNumTileRowsMinus1() );           //  ue(v)
      for ( size_t i = 0; i < afti.getNumTileColumnsMinus1(); i++ ) {
        bitstream.writeUvlc( afti.getTileColumnWidthMinus1( i ) );  //  ue(v)
      }
      for ( size_t i = 0; i < afti.getNumTileRowsMinus1(); i++ ) {
        bitstream.writeUvlc( afti.getTileRowHeightMinus1( i ) );    //  ue(v)
      }
    }
  }
  bitstream.write( afti.getSingleTilePerTileGroupFlag(), 1 );  //  u(1)
  if ( !afti.getSingleTilePerTileGroupFlag() ) {
    uint32_t NumTilesInPatchFrame = ( afti.getNumTileColumnsMinus1() + 1 ) * ( afti.getNumTileRowsMinus1() + 1 );
    bitstream.writeUvlc( afti.getNumTileGroupsInAtlasFrameMinus1() );  // ue(v)
    for ( size_t i = 0; i <= afti.getNumTileGroupsInAtlasFrameMinus1(); i++ ) {
      uint8_t bitCount = getFixedLengthCodeBitsCount( NumTilesInPatchFrame + 1 );
      if ( i > 0 ) {
        bitstream.write( afti.getTopLeftTileIdx( i ), bitCount ); 
      }  // u(v) : Ceil( Log2( NumTilesInPatchFrame )
      bitCount = getFixedLengthCodeBitsCount( NumTilesInPatchFrame - afti.getTopLeftTileIdx( i ) + 1 );
      bitstream.write( afti.getBottomRightTileIdxDelta( i ),
                       bitCount );  // u(v) : Ceil( Log2( NumTilesInPatchFrame − pfti_top_left_tile_idx[ i ] ) )
    }
  }
  bitstream.write( afti.getSignalledTileGroupIdFlag(), 1 );  // u(1)
  if ( afti.getSignalledTileGroupIdFlag() ) {
    bitstream.writeUvlc( afti.getSignalledTileGroupIdLengthMinus1() );  // ue(v)
    for ( size_t i = 0; i <= afti.getSignalledTileGroupIdLengthMinus1(); i++ ) {
      uint8_t bitCount = afti.getSignalledTileGroupIdLengthMinus1() + 1;
      bitstream.write( afti.getTileGroupId( i ), bitCount );  // u(v) : pfti_signalled_tile_group_id_length_minus1 + 1  bits
    }
  }
}

// 7.3.6.5  Supplemental enhancement information RBSP syntax
// 7.3.6.6  Access unit delimiter RBSP syntax
// 7.3.6.7  End of sequence RBSP syntax
// 7.3.6.8  End of bitstream RBSP syntax
// 7.3.6.9  Filler data RBSP syntax

// 7.3.6.10  Atlas tile group layer RBSP syntax = patchTileGroupLayerUnit
void PCCBitstreamEncoder::atlasTileGroupLayerRbsp( AtlasTileGroupLayerRbsp& atgl,
                                                   PCCContext&              context,
                                                   PCCBitstream&            bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  // setFrameIndex
  atlasTileGroupHeader( atgl.getAtlasTileGroupHeader(), context, bitstream );
  if ( atgl.getAtlasTileGroupHeader().getAtghType() != SKIP_TILE_GRP )
    atlasTileGroupDataUnit( atgl.getAtlasTileGroupDataUnit(), atgl.getAtlasTileGroupHeader(), context, bitstream );
  rbspTrailingBits( bitstream );
}

// 7.3.6.11  Atlas tile group header syntax
void PCCBitstreamEncoder::atlasTileGroupHeader( AtlasTileGroupHeader& atgh,
                                                PCCContext&           context,
                                                PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t afpsId = atgh.getAtghAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp&    afps = context.getAtlasFrameParameterSet(afpsId);
  size_t aspsId = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRBSP& asps = context.getAtlasSequenceParameterSet(aspsId);
  AtlasFrameTileInformation&    afti = afps.getAtlasFrameTileInformation();

  bitstream.writeUvlc( atgh.getAtghAtlasFrameParameterSetId() );
  bitstream.write( uint32_t( atgh.getAtghAddress() ), afti.getSignalledTileGroupIdLengthMinus1() + 1 );
  bitstream.writeUvlc( uint32_t( atgh.getAtghType() ) );
  bitstream.write( uint32_t( atgh.getAtghAtlasFrmOrderCntLsb() ), asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() + 4 );
  TRACE_BITSTREAM( " AtlasFrameParameterSetId: %zu\n", atgh.getAtghAtlasFrameParameterSetId() );
  TRACE_BITSTREAM( " AtlasSequenceParameterSetId: %zu\n", afps.getAtlasSequenceParameterSetId() );
  TRACE_BITSTREAM( " Address       %zu\n", (size_t) atgh.getAtghAddress() );
  TRACE_BITSTREAM( " Type          %zu\n", (size_t) atgh.getAtghType() );
  TRACE_BITSTREAM( " FrameOrderCnt %zu\n", (size_t) atgh.getAtghAtlasFrmOrderCntLsb() );
  if ( asps.getNumRefAtlasFrameListsInAsps() > 0 )
    bitstream.write( uint32_t( atgh.getAtghRefAtlasFrameListSpsFlag() ), 1 );
  
  if ( atgh.getAtghRefAtlasFrameListSpsFlag() == 0 ) {
    //atgh.allocate( asps.getNumRefAtlasFrameListsInAsps() );
    refListStruct( atgh.getRefListStruct(), asps, bitstream );
  } else if (asps.getNumRefAtlasFrameListsInAsps() > 1){
    size_t bitCount = getFixedLengthCodeBitsCount( asps.getNumRefAtlasFrameListsInAsps() +
                                                   1 );  // by Ceil( Log2( asps_num_ref_atlas_frame_lists_in_asps ) )
    bitstream.write( uint32_t( atgh.getAtghRefAtlasFrameListIdx() ), bitCount );
  }

  //jkei:(refList) rlsIdx is correct??
  uint8_t rlsIdx = atgh.getAtghRefAtlasFrameListIdx();
  auto& refList = atgh.getAtghRefAtlasFrameListSpsFlag() ? asps.getRefListStruct(rlsIdx) : atgh.getRefListStruct();
  size_t numLtrAtlasFrmEntries = 0;
  for ( size_t i = 0; i < refList.getNumRefEntries(); i++ ) {
    if ( !refList.getStRefAtalsFrameFlag( i ) ) { numLtrAtlasFrmEntries++; }
  }
  TRACE_BITSTREAM( " rlsIdx %u numLtrAtlasFrmEntries %zu \n", rlsIdx, (size_t) numLtrAtlasFrmEntries );

  for ( size_t j = 0; j < numLtrAtlasFrmEntries; j++ ) {
    bitstream.write( uint32_t( atgh.getAtghAdditionalAfocLsbPresentFlag( j ) ), 1 );
    // afps_additional_lt_afoc_lsb_len
    if ( atgh.getAtghAdditionalAfocLsbPresentFlag( j ) ) {
      uint8_t bitCount = afps.getAfpsAdditionalLtAfocLsbLen();
      bitstream.write( uint32_t( atgh.getAtghAdditionalAfocLsbVal( j ) ), bitCount );
    }
  }
  if ( atgh.getAtghType() != SKIP_TILE_GRP ) {
    if ( asps.getNormalAxisLimitsQuantizationEnabledFlag() ) {
      bitstream.write( uint32_t( atgh.getAtghPosMinZQuantizer() ), 5 );
      if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() )
        bitstream.write( uint32_t( atgh.getAtghPosDeltaMaxZQuantizer() ), 5 );
    }
    if ( asps.getPatchSizeQuantizerPresentFlag() ) {
      bitstream.write( uint32_t( atgh.getAtghPatchSizeXinfoQuantizer() ), 3 );
      bitstream.write( uint32_t( atgh.getAtghPatchSizeYinfoQuantizer() ), 3 );
    }
    if ( afps.getAfpsRaw3dPosBitCountExplicitModeFlag() ) {
      size_t bitCount = getFixedLengthCodeBitsCount(
          context.getSps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1() + 1 );  // 0??
      bitstream.write( uint32_t( atgh.getAtghRaw3dPosAxisBitCountMinus1() ), bitCount );
    }
    
    if ( atgh.getAtghType() == P_TILE_GRP && refList.getNumRefEntries() > 1 ) {
      bitstream.write( uint32_t( atgh.getAtghNumRefIdxActiveOverrideFlag() ), 1 );
      if ( atgh.getAtghNumRefIdxActiveOverrideFlag() )
        bitstream.writeUvlc( uint32_t( atgh.getAtghNumRefdxActiveMinus1() ) );
    }
  }
  byteAlignment( bitstream );
}

// 7.3.6.12  Reference list structure syntax (NEW)
void PCCBitstreamEncoder::refListStruct( RefListStruct&                 rls,
                                         AtlasSequenceParameterSetRBSP& asps,
                                         PCCBitstream&                  bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.writeUvlc( rls.getNumRefEntries() );  // ue(v)
  TRACE_BITSTREAM( "NumRefEntries = %lu  \n", rls.getNumRefEntries() );
  //rls.allocate(); //jkei: why is it here?
  for ( size_t i = 0; i < rls.getNumRefEntries(); i++ ) {
    if ( asps.getLongTermRefAtlasFramesFlag() )
      bitstream.write( rls.getStRefAtalsFrameFlag( i ), 1 );  // u(1)
    
      if ( rls.getStRefAtalsFrameFlag( i ) ) {
        bitstream.writeUvlc( rls.getAbsDeltaAfocSt( i ) );  // ue(v)
        if ( rls.getAbsDeltaAfocSt( i ) > 0 ) {
          bitstream.write( rls.getStrpfEntrySignFlag( i ), 1 );  // u(1)
        }
      }else {
        uint8_t bitCount = asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() + 4;
        bitstream.write( rls.getAfocLsbLt( i ),
                         bitCount );  // u(v) psps_log2_max_patch_frame_order_cnt_lsb_minus4  + 4 bits
      }
    TRACE_BITSTREAM( "stRefAtalsFrameFlag = %lu  \n", rls.getStRefAtalsFrameFlag(i) );
    TRACE_BITSTREAM( "absDeltaAfocSt      = %lu  \n", rls.getAbsDeltaAfocSt(i) );
    TRACE_BITSTREAM( "strpfEntrySignFlag  = %lu  \n", rls.getStrpfEntrySignFlag(i) );
  }
  TRACE_BITSTREAM( "%s done \n", __func__ );
}

// 7.3.7.1  General atlas tile group data unit syntax =patchTileGroupDataUnit
void PCCBitstreamEncoder::atlasTileGroupDataUnit( AtlasTileGroupDataUnit& atgdu,
                                                  AtlasTileGroupHeader&   atgh,
                                                  PCCContext&             context,
                                                  PCCBitstream&           bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "atgh.getAtghType()        = %lu \n", atgh.getAtghType() );
  for ( size_t puCount = 0; puCount < atgdu.getPatchCount(); puCount++ ) {
    bitstream.writeUvlc( uint32_t( atgdu.getPatchMode( puCount ) ) );
    TRACE_BITSTREAM( "patchMode = %lu \n", atgdu.getPatchMode( puCount ) );
    atgdu.getPatchInformationData( puCount ).setFrameIndex( atgdu.getFrameIndex() );
    atgdu.getPatchInformationData( puCount ).setPatchIndex( puCount );
    patchInformationData( atgdu.getPatchInformationData( puCount ), atgdu.getPatchMode( puCount ), atgh, context,
                          bitstream );
  }
  TRACE_BITSTREAM( "atgdu.getPatchCount() including END = %lu \n", atgdu.getPatchCount() );
  byteAlignment( bitstream );
}

// 7.3.7.2  Patch information data syntax
void PCCBitstreamEncoder::patchInformationData( PatchInformationData& pid,
                                                size_t                patchMode,
                                                AtlasTileGroupHeader& atgh,
                                                PCCContext&           context,
                                                PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  if ( atgh.getAtghType() == (uint8_t) P_TILE_GRP && patchMode == PATCH_MODE_P_SKIP ) {
    // skip mode: currently not supported but added it for convenience. Could easily be removed
  } else if ( atgh.getAtghType() == (uint8_t) P_TILE_GRP && patchMode == PATCH_MODE_P_MERGE ) {
    // skip mode: currently not supported but added it for convenience. Could easily be removed
    auto& mpdu = pid.getMergePatchDataUnit();
    mpdu.setFrameIndex( pid.getFrameIndex() );
    mpdu.setPatchIndex( pid.getPatchIndex() );
    mergePatchDataUnit( mpdu, atgh, context, bitstream );
  } else if ( atgh.getAtghType() == (uint8_t) P_TILE_GRP && patchMode == PATCH_MODE_P_INTER ) {
    auto& ipdu = pid.getInterPatchDataUnit();
    ipdu.setFrameIndex( pid.getFrameIndex() );
    ipdu.setPatchIndex( pid.getPatchIndex() );
    interPatchDataUnit( ipdu, atgh, context, bitstream );
  } else if ( ( ( PCCTILEGROUP( atgh.getAtghType() ) ) == I_TILE_GRP && patchMode == PATCH_MODE_I_INTRA ) ||
              ( ( PCCTILEGROUP( atgh.getAtghType() ) ) == P_TILE_GRP && patchMode == PATCH_MODE_P_INTRA ) ) {
    auto& pdu = pid.getPatchDataUnit();
    pdu.setFrameIndex( pid.getFrameIndex() );
    pdu.setPatchIndex( pid.getPatchIndex() );
    patchDataUnit( pdu, atgh, context, bitstream );
  } else if ( ( ( PCCTILEGROUP( atgh.getAtghType() ) ) == I_TILE_GRP && patchMode == PATCH_MODE_I_Raw ) ||
              ( ( PCCTILEGROUP( atgh.getAtghType() ) ) == P_TILE_GRP && patchMode == PATCH_MODE_P_Raw ) ) {
    auto& rpdu = pid.getRawPatchDataUnit();
    rpdu.setFrameIndex( pid.getFrameIndex() );
    rpdu.setPatchIndex( pid.getPatchIndex() );
    rawPatchDataUnit( rpdu, atgh, context, bitstream );
  } else if ( ( ( PCCTILEGROUP( atgh.getAtghType() ) ) == I_TILE_GRP && patchMode == PATCH_MODE_I_EOM ) ||
              ( ( PCCTILEGROUP( atgh.getAtghType() ) ) == P_TILE_GRP && patchMode == PATCH_MODE_P_EOM ) ) {
    auto& epdu = pid.getEomPatchDataUnit();
    epdu.setFrameIndex( pid.getFrameIndex() );
    epdu.setPatchIndex( pid.getPatchIndex() );
    eomPatchDataUnit( epdu, atgh, context, bitstream );
  }
}

// 7.3.7.3  Patch data unit syntax : AtlasTileGroupHeader instead of PatchTileGroupHeader
void PCCBitstreamEncoder::patchDataUnit( PatchDataUnit&        pdu,
                                         AtlasTileGroupHeader& atgh,
                                         PCCContext&           context,
                                         PCCBitstream&         bitstream ) {
  
  size_t afpsId = atgh.getAtghAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp&    afps = context.getAtlasFrameParameterSet(afpsId);
  size_t aspsId = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRBSP& asps = context.getAtlasSequenceParameterSet(aspsId);

  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( uint32_t( pdu.getPdu2dPosX() ), afps.getAfps2dPosXBitCountMinus1() + 1 );  // u(v)
  bitstream.write( uint32_t( pdu.getPdu2dPosY() ), afps.getAfps2dPosYBitCountMinus1() + 1 );  // u(v)
  TRACE_BITSTREAM( " 2dPosXY: %zu,%zu\n", pdu.getPdu2dPosX(), pdu.getPdu2dPosX() );
  bitstream.writeSvlc( int32_t( pdu.getPdu2dDeltaSizeX() ) );
  bitstream.writeSvlc( int32_t( pdu.getPdu2dDeltaSizeY() ) );
  TRACE_BITSTREAM( " 2dDeltaSizeXY: %d,%d\n", int32_t( pdu.getPdu2dDeltaSizeX() ) , int32_t( pdu.getPdu2dDeltaSizeY() )  );
  bitstream.write( uint32_t( pdu.getPdu3dPosX() ), afps.getAfps3dPosXBitCountMinus1() + 1 );  // u(v)
  bitstream.write( uint32_t( pdu.getPdu3dPosY() ), afps.getAfps3dPosYBitCountMinus1() + 1 );  // u(v)
  TRACE_BITSTREAM( " 3dPosXY: %zu,%zu\n", pdu.getPdu3dPosX(), pdu.getPdu3dPosY()  );
  const uint8_t bitCountForMinDepth =
      context.getSps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1() + 1 - atgh.getAtghPosMinZQuantizer() + (pdu.getProjectPlane()>5?2:1);

  //minZ:(gi_geometry_3d_coordinates_bitdepth_minus1 – atgh_pos_min_z_quantizer + ( pdu_projection_id[ p ] > 5 ) ? 2 : 1 ).
  bitstream.write( uint32_t( pdu.getPdu3dPosMinZ() ), bitCountForMinDepth );  // u(v)
  if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
    uint8_t bitCountForMaxDepth =
    context.getSps().getGeometryInformation( 0 ).getGeometry3dCoordinatesBitdepthMinus1() + 1 - atgh.getAtghPosDeltaMaxZQuantizer() + (pdu.getProjectPlane()>5?2:1);
    //maxZ:( gi_geometry_3d_coordinates_bitdepth_minus1 – atgh_pos_delta_max_z_quantizer + ( pdu_projection_id[ p ] > 5 ) ? 2 : 1 ).
    if ( asps.get45DegreeProjectionPatchPresentFlag() ) bitCountForMaxDepth++;
    bitstream.write( uint32_t( pdu.getPdu3dPosDeltaMaxZ() ), bitCountForMaxDepth );
  }

  bitstream.write( uint32_t( pdu.getProjectPlane() ), ( asps.get45DegreeProjectionPatchPresentFlag()? 5 : 3));  // u(5 or 3)
  bitstream.write( uint32_t( pdu.getPduOrientationIndex() ), ( asps.getUseEightOrientationsFlag() ? 3 : 1 ) );// u(3 or 1)
  if ( afps.getLodModeEnableFlag() ) {
    bitstream.write( uint32_t( pdu.getLodEnableFlag() ),
                     1 );  // u(1) //    pdu.setLodEnableFlag(bitstream.read( 1 ) ); //u1
    if ( pdu.getLodEnableFlag() ) {
      bitstream.writeUvlc( uint32_t( pdu.getLodScaleXminus1() ) );
      bitstream.writeUvlc( uint32_t( pdu.getLodScaleY() ) );
    }
  }

  if ( asps.getPointLocalReconstructionEnabledFlag() ) {
    pointLocalReconstructionData( pdu.getPointLocalReconstructionData(), context, bitstream );
  }
  TRACE_BITSTREAM(
        "Frame %zu, Patch(%zu) => 2dPos=%4zu %4zu deltaSize=%4ld %4ld 3dPos=%4zu %4zu %4zu %4zu P=%zu O=%d lod=(%u/%u) %u %u\n ",
        pdu.getFrameIndex(), pdu.getPatchIndex(),
        pdu.getPdu2dPosX(), pdu.getPdu2dPosY(), pdu.getPdu2dDeltaSizeX(), pdu.getPdu2dDeltaSizeY(), pdu.getPdu3dPosX(),
        pdu.getPdu3dPosY(), pdu.getPdu3dPosMinZ(), pdu.getPdu3dPosDeltaMaxZ(), pdu.getPduProjectionId(),
        pdu.getPduOrientationIndex(), afps.getLodModeEnableFlag(), pdu.getLodEnableFlag(), pdu.getLodScaleXminus1(), pdu.getLodScaleY() );
 }

// 7.3.7.4  Skip patch data unit syntax
void PCCBitstreamEncoder::skipPatchDataUnit( SkipPatchDataUnit&    spdu,
                                             AtlasTileGroupHeader& atgh,
                                             PCCContext&           context,
                                             PCCBitstream&         bitstream ) {}

// 7.3.7.5  Merge patch data unit syntax
void PCCBitstreamEncoder::mergePatchDataUnit( MergePatchDataUnit&   mpdu,
                                              AtlasTileGroupHeader& atgh,
                                              PCCContext&           context,
                                              PCCBitstream&         bitstream ) {
  size_t afpsId = atgh.getAtghAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp&    afps = context.getAtlasFrameParameterSet(afpsId);
  size_t aspsId = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRBSP& asps = context.getAtlasSequenceParameterSet(aspsId);
  bool                          overridePlrFlag = false;

  TRACE_BITSTREAM( "%s \n", __func__ );
  if ( atgh.getAtghNumRefdxActiveMinus1() > 0 ) bitstream.writeUvlc( uint32_t( mpdu.getMpduRefIndex() ));
  bitstream.write( mpdu.getMpduOverride2dParamsFlag(), 1 );
  if ( mpdu.getMpduOverride2dParamsFlag() ) {
    bitstream.writeSvlc( int32_t( mpdu.getMpdu2dPosX()) );       // se(v)
    bitstream.writeSvlc( int32_t( mpdu.getMpdu2dPosY()) );        // se(v)
    bitstream.writeSvlc( int32_t( mpdu.getMpdu2dDeltaSizeX()) );  // se(v)
    bitstream.writeSvlc( int32_t( mpdu.getMpdu2dDeltaSizeY()) );  // se(v)
    if ( asps.getPointLocalReconstructionEnabledFlag() ) overridePlrFlag = true;
  } else {
    bitstream.write( mpdu.getMpduOverride3dParamsFlag(), 1 );
    if ( mpdu.getMpduOverride3dParamsFlag() ) {
      bitstream.writeSvlc( int32_t(mpdu.getMpdu3dPosX()) );
      bitstream.writeSvlc( int32_t(mpdu.getMpdu3dPosY()) );
      bitstream.writeSvlc( int32_t(mpdu.getMpdu3dPosMinZ()) );
      if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) { bitstream.writeSvlc( int32_t(mpdu.getMpdu3dPosDeltaMaxZ()) ); }
      if ( asps.getPointLocalReconstructionEnabledFlag() ) { bitstream.write( uint32_t(mpdu.getMpduOverridePlrFlag()), 1 ); }
    }
  }  // else

  if ( overridePlrFlag && asps.getPointLocalReconstructionEnabledFlag() ) {
    pointLocalReconstructionData( mpdu.getPointLocalReconstructionData(), context, bitstream );
  }

  TRACE_BITSTREAM(
      "%zu frame %zu MergePatch => Ref Frame Idx = %d ShiftUV = %ld %ld DeltaSize = %ld %ld Axis = %ld %ld %ld\n",
      mpdu.getMpduRefIndex(), mpdu.getMpdu2dPosX(), mpdu.getMpdu2dPosY(), mpdu.getMpdu2dDeltaSizeX(),
      mpdu.getMpdu2dDeltaSizeY(), mpdu.getMpdu3dPosX(), mpdu.getMpdu3dPosY(), mpdu.getMpdu3dPosMinZ(),
      mpdu.getMpdu3dPosDeltaMaxZ() );
}

// 7.3.7.6  Inter patch data unit syntax
void PCCBitstreamEncoder::interPatchDataUnit( InterPatchDataUnit&   ipdu,
                                              AtlasTileGroupHeader& atgh,
                                              PCCContext&           context,
                                              PCCBitstream&         bitstream ) {
  size_t afpsId = atgh.getAtghAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp&    afps = context.getAtlasFrameParameterSet(afpsId);
  size_t aspsId = afps.getAtlasSequenceParameterSetId();
  AtlasSequenceParameterSetRBSP& asps = context.getAtlasSequenceParameterSet(aspsId);

  TRACE_BITSTREAM( "%s \n", __func__ );
  if ( atgh.getAtghNumRefdxActiveMinus1() > 0 ) bitstream.writeUvlc( int32_t( ipdu.getIpduRefIndex() ) );

  bitstream.writeSvlc( int32_t( ipdu.getIpduRefPatchIndex() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu2dPosX() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu2dPosY() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu2dDeltaSizeX() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu2dDeltaSizeY() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu3dPosX() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu3dPosY() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu3dPosMinZ() ) );
  if ( asps.getNormalAxisMaxDeltaValueEnabledFlag() ) {
    bitstream.writeSvlc( int32_t( ipdu.getIpdu3dPosDeltaMaxZ() ) );
  }
  if ( asps.getPointLocalReconstructionEnabledFlag() ) {
    pointLocalReconstructionData( ipdu.getPointLocalReconstructionData(), context, bitstream );
  }

  TRACE_BITSTREAM(
      "%zu frame %zu DeltaPatch => DeltaIdx = %d ShiftUV = %ld %ld DeltaSize = %ld %ld Axis = %ld %ld %ld\n",
      ipdu.getIpduRefIndex(), ipdu.getIpduRefPatchIndex(), ipdu.getIpdu2dPosX(), ipdu.getIpdu2dPosY(),
      ipdu.getIpdu2dDeltaSizeX(), ipdu.getIpdu2dDeltaSizeY(), ipdu.getIpdu3dPosX(), ipdu.getIpdu3dPosY(),
      ipdu.getIpdu3dPosMinZ(), ipdu.getIpdu3dPosDeltaMaxZ() );
}

// 7.3.7.7 raw patch data unit syntax
void PCCBitstreamEncoder::rawPatchDataUnit( RawPatchDataUnit&     ppdu,
                                            AtlasTileGroupHeader& atgh,
                                            PCCContext&           context,
                                            PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t afpsId = atgh.getAtghAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp&    afps = context.getAtlasFrameParameterSet(afpsId);

  auto&  sps        = context.getSps();
  size_t atlasIndex = 0;
  if ( sps.getRawSeparateVideoPresentFlag( atlasIndex ) ) {
    bitstream.write( ppdu.getRpduPatchInRawVideoFlag(), 1 );
  }                                                                                             // u(1)
  bitstream.write( uint32_t( ppdu.getRpdu2dPosX() ), afps.getAfps2dPosXBitCountMinus1() + 1 );  // u(v)
  bitstream.write( uint32_t( ppdu.getRpdu2dPosY() ), afps.getAfps2dPosYBitCountMinus1() + 1 );  // u(v)
  bitstream.writeSvlc( int32_t( ppdu.getRpdu2dDeltaSizeX() ) );                                 // se(v)
  bitstream.writeSvlc( int32_t( ppdu.getRpdu2dDeltaSizeY() ) );                                 // se(v)
  bitstream.write( uint32_t( ppdu.getRpdu3dPosX() ), atgh.getAtghRaw3dPosAxisBitCountMinus1() + 1 );
  bitstream.write( uint32_t( ppdu.getRpdu3dPosY() ), atgh.getAtghRaw3dPosAxisBitCountMinus1() + 1 );  // u(v)
  bitstream.write( uint32_t( ppdu.getRpdu3dPosZ() ), atgh.getAtghRaw3dPosAxisBitCountMinus1() + 1 );  // u(v)
  bitstream.writeUvlc( int32_t( ppdu.getRpduRawPoints() ) );
  TRACE_BITSTREAM(
      "Raw Patch => 2dPos=%4zu %4zu  deltaSize=%4ld %4ld 3dPos=%4zu %4zu %4zu NumPcmPoints=%lu PatchInRawVideoFlag=%d \n",
      ppdu.getRpdu2dPosX(), ppdu.getRpdu2dPosY(), ppdu.getRpdu2dDeltaSizeX(), ppdu.getRpdu2dDeltaSizeY(),
      ppdu.getRpdu3dPosX(), ppdu.getRpdu3dPosY(), ppdu.getRpdu3dPosZ(), ppdu.getRpduRawPoints(),
      ppdu.getRpduPatchInRawVideoFlag() );
}

// 7.3.7.8 EOM patch data unit syntax
void PCCBitstreamEncoder::eomPatchDataUnit( EOMPatchDataUnit&     epdu,
                                            AtlasTileGroupHeader& atgh,
                                            PCCContext&           context,
                                            PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t afpsId = atgh.getAtghAtlasFrameParameterSetId();
  AtlasFrameParameterSetRbsp&    afps = context.getAtlasFrameParameterSet(afpsId);

  bitstream.write( uint32_t( epdu.getEpdu2dPosX() ), afps.getAfps2dPosXBitCountMinus1() + 1 );  // u(v)
  bitstream.write( uint32_t( epdu.getEpdu2dPosY() ), afps.getAfps2dPosYBitCountMinus1() + 1 );  // u(v)
  bitstream.writeSvlc( int32_t( epdu.getEpdu2dDeltaSizeX() ) );                                 // se(v)
  bitstream.writeSvlc( int32_t( epdu.getEpdu2dDeltaSizeY() ) );                                 // se(v)
  bitstream.write( uint32_t( epdu.getEpduAssociatedPatchesCountMinus1() ), 8 );                 // max255
  for ( size_t cnt = 0; cnt < epdu.getEpduAssociatedPatchesCountMinus1() + 1; cnt++ ) {
    bitstream.write( uint32_t( epdu.getEpduAssociatedPatches()[cnt] ), 8 );
    bitstream.writeUvlc( uint32_t( epdu.getEpduEomPointsPerPatch()[cnt] ) );
  }
  TRACE_BITSTREAM( "EOM Patch => UV %4lu %4lu  N=%4ld\n", epdu.getEpdu2dPosX(), epdu.getEpdu2dPosY(),
                   epdu.getEpduAssociatedPatchesCountMinus1() + 1 );
  for ( size_t cnt = 0; cnt < epdu.getEpduAssociatedPatchesCountMinus1() + 1; cnt++ )
    TRACE_BITSTREAM( "%4ld, %4ld\n", epdu.getEpduAssociatedPatches()[cnt], epdu.getEpduEomPointsPerPatch()[cnt] );
}

//jkei: atlasSubStream updated PST:Nov6
void PCCBitstreamEncoder::atlasSubStream( PCCContext& context, PCCBitstream& bitstream )
{
  TRACE_BITSTREAM("atlasSub sample Nal Stream:\n");
  SampleStreamNalUnit ssNalu;
  uint32_t maxUnitSize = 0;
  uint32_t aspsSize =sizeof(context.getAtlasSequenceParameterSet(0));
  uint32_t afpsSize =sizeof(context.getAtlasFrameParameterSet(0)); //jkei: it doesnot work as i want...
  
  for(size_t frameIdx=0; frameIdx<context.size(); frameIdx++){
    if (maxUnitSize < sizeof(context.getAtlasTileGroupLayer(frameIdx)) )
      maxUnitSize = (uint32_t) sizeof(context.getAtlasTileGroupLayer(frameIdx)) ;
  }
  maxUnitSize= max(max(maxUnitSize, aspsSize),afpsSize);
  uint32_t precision = (uint32_t) (min(max((int)ceil((double)getFixedLengthCodeBitsCount(maxUnitSize) / 8.0), 1), 8)-1);
  ssNalu.setUnitSizePrecisionBytesMinus1(precision);
  sampleStreamNalHeader(bitstream, precision);
#if 1
  printf("sampleStreamNalHeader size: %lld\n", bitstream.size());
#endif

  for(size_t aspsCount=0; aspsCount<context.getAtlasSequenceParameterSetList().size(); aspsCount++){
    NalUnit naluASPS;
    naluASPS.setNalUnitType( (uint8_t) NAL_ASPS);
    naluASPS.setLayerId( 0 );           // u(6)
    naluASPS.setTemporalyIdPlus1( 1 );  // u(3)
    naluASPS.setNalUnitSize(aspsSize+2); //+headersize :
    sampleStreamNalUnit(context, bitstream, ssNalu, naluASPS, aspsCount);
#if 1
    printf("nalu%hhu, naluSize:%zu, sizeBitstream written: %lld\n",naluASPS.getNalUnitType(), naluASPS.getNalUnitSize(), bitstream.size());
#endif
  }
  
  for(size_t afpsCount=0; afpsCount<context.getAtlasFrameParameterSetList().size(); afpsCount++){
    NalUnit naluAFPS;
    naluAFPS.setNalUnitType( (uint8_t) NAL_AFPS);
    naluAFPS.setLayerId( 0 );           // u(6)
    naluAFPS.setTemporalyIdPlus1( 1 );  // u(3)
    naluAFPS.setNalUnitSize(sizeof(context.getAtlasFrameParameterSet(0))+2);//+headersize
    sampleStreamNalUnit(context, bitstream, ssNalu, naluAFPS, afpsCount);
#if 1
    printf("nalu%hhu, naluSize:%zu, sizeBitstream written: %lld\n",naluAFPS.getNalUnitType(), naluAFPS.getNalUnitSize(), bitstream.size());
#endif
  }
 //NAL_TRAIL, NAL_TSA, NAL_STSA, NAL_RADL, NAL_RASL,NAL_SKIP
  for(size_t frameIdx=0; frameIdx<context.size(); frameIdx++){
    NalUnit nalATGL;
    nalATGL.setNalUnitType( (uint8_t) NAL_TSA); //is it correct?
    nalATGL.setLayerId( 0 );           // u(6)
    nalATGL.setTemporalyIdPlus1( 1 );  // u(3)
    nalATGL.setNalUnitSize(sizeof(context.getAtlasTileGroupLayer(frameIdx))+2); //+headsize
    context.getAtlasTileGroupLayer(frameIdx).getAtlasTileGroupDataUnit().setFrameIndex(frameIdx); //jkei: actually it is not frameidx since increased per tile!
    TRACE_BITSTREAM(" ATGL: frame %zu\n", frameIdx);
    sampleStreamNalUnit(context, bitstream, ssNalu, nalATGL, frameIdx);
#if 1
    printf("nalu%hhu, naluSize:%zu, sizeBitstream written: %lld\n",nalATGL.getNalUnitType(), nalATGL.getNalUnitSize(), bitstream.size());
#endif
  }
  
}

//jkei: updated up to this point PST:Nov6
// jkei -----> OLD
// 7.3.5.1 General patch data group unit syntax
void PCCBitstreamEncoder::atlasSubStream_old( PCCContext& context, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& pdg = context.getPatchDataGroup();
  for ( uint8_t unitType = (uint8_t)PDG_PSPS; unitType < (uint8_t)PDG_PTGLU; unitType++ ) {
    bitstream.writeUvlc( ( (uint32_t)unitType ) );  // ue(v)
    TRACE_BITSTREAM( "%s (%u): frame = %u \n", strUnitType( (PDGUnitType)unitType ).c_str(), (uint32_t)unitType, 0 );
    atlasSubStreamUnitPayload( pdg, (PDGUnitType)unitType, 0, 0, context, bitstream );
    bitstream.write( (uint32_t)0, 1 );  // u(1)
  }
  for ( uint8_t i = 0; i < pdg.getPatchTileGroupLayerUnitSize(); i++ ) {
    bitstream.writeUvlc( ( (uint32_t)PDG_PTGLU ) );  // ue(v)
    TRACE_BITSTREAM( "%s (%u): frame = %u \n", strUnitType( PDG_PTGLU ).c_str(), (uint32_t)PDG_PTGLU, i );
    atlasSubStreamUnitPayload( pdg, PDG_PTGLU, i, i, context, bitstream );
    bitstream.write( ( uint32_t )( i + 1 == pdg.getPatchTileGroupLayerUnitSize() ), 1 );  // u(1)
  }
  byteAlignment( bitstream );
}

// 7.3.5.2 Patch data group unit payload syntax
void PCCBitstreamEncoder::atlasSubStreamUnitPayload( PatchDataGroup& pdg,
                                                     PDGUnitType     unitType,
                                                     size_t          index,
                                                     size_t          frameIndex,
                                                     PCCContext&     context,
                                                     PCCBitstream&   bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "  type = %u frameIndex = %u \n", (uint8_t)unitType, frameIndex );
  auto& sps = context.getSps();
  switch ( unitType ) {
    case PDG_PSPS: patchVpccParameterSet( pdg, index, bitstream ); break;
    case PDG_GPPS: geometryPatchParameterSet( pdg, index, bitstream ); break;
    case PDG_APPS: attributePatchParameterSet( pdg, index, sps, bitstream ); break;
    case PDG_PFPS: patchFrameParameterSet( pdg, index, sps, bitstream ); break;
    case PDG_PFAPS: patchFrameAttributeParameterSet( pdg, index, sps, bitstream ); break;
    case PDG_PFGPS: patchFrameGeometryParameterSet( pdg, index, sps, bitstream ); break;
    case PDG_PTGLU: patchTileGroupLayerUnit( pdg, index, context, bitstream ); break;
    case PDG_PREFIX_SEI: seiMessage( pdg, index, context, bitstream ); break;
    case PDG_SUFFIX_SEI: seiMessage( pdg, index, context, bitstream ); break;
    default: assert( 0 ); break;
  }
}

// 7.3.5.3 Patch sequence parameter set syntax
void PCCBitstreamEncoder::patchVpccParameterSet( PatchDataGroup& pdg, size_t index, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& psps = pdg.getPatchVpccParameterSet( index );
  assert( psps.getPatchVpccParameterSetId() == index );
  bitstream.writeUvlc( psps.getPatchVpccParameterSetId() );             // ue(v)
  bitstream.write( psps.getLog2PatchPackingBlockSize(), 3 );            // u(3)
  bitstream.writeUvlc( psps.getLog2MaxPatchFrameOrderCntLsbMinus4() );  // ue(v)
  bitstream.writeUvlc( psps.getMaxDecPatchFrameBufferingMinus1() );     // ue(v)
  bitstream.write( psps.getLongTermRefPatchFramesFlag(), 1 );           // u(1)
  bitstream.writeUvlc( psps.getNumRefPatchFrameListsInPsps() );         // ue(v)
  for ( size_t i = 0; i < psps.getNumRefPatchFrameListsInPsps(); i++ ) {
    refListStruct( psps.getRefListStruct( i ), psps, bitstream );
  }
  bitstream.write( psps.getUseEightOrientationsFlag(), 1 );                // u(1)
  bitstream.write( psps.getNormalAxisLimitsQuantizationEnableFlag(), 1 );  // u(1)
  bitstream.write( psps.getNormalAxisMaxDeltaValueEnableFlag(), 1 );       // u(1)
}

// 7.3.5.4 Patch frame geometry parameter set syntax
void PCCBitstreamEncoder::patchFrameGeometryParameterSet( PatchDataGroup&   pdg,
                                                          size_t            index,
                                                          VpccParameterSet& sps,
                                                          PCCBitstream&     bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  auto&  gi         = sps.getGeometryInformation( atlasIndex );
  auto&  pfgps      = pdg.getPatchFrameGeometryParameterSet( index );
  assert( pfgps.getPatchFrameGeometryParameterSetId() == index );
  bitstream.writeUvlc( pfgps.getPatchFrameGeometryParameterSetId() );  // ue(v)
  bitstream.writeUvlc( pfgps.getPatchVpccParameterSetId() );           // ue(v)
  TRACE_BITSTREAM( "GeometryParamsEnabledFlag = %d \n", gi.getGeometryParamsEnabledFlag() );
  if ( gi.getGeometryParamsEnabledFlag() ) { geometryFrameParams( pfgps.getGeometryFrameParams(), bitstream ); }
  TRACE_BITSTREAM( "GeometryPatchParamsEnabledFlag = %d \n", gi.getGeometryPatchParamsEnabledFlag() );
  if ( gi.getGeometryPatchParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)pfgps.getGeometryPatchScaleParamsEnabledFlag(), 1 );     // u(1)
    bitstream.write( (uint32_t)pfgps.getGeometryPatchOffsetParamsEnabledFlag(), 1 );    // u(1)
    bitstream.write( (uint32_t)pfgps.getGeometryPatchRotationParamsEnabledFlag(), 1 );  // u(1)
    bitstream.write( (uint32_t)pfgps.getGeometryPatchPointSizeInfoEnabledFlag(), 1 );   // u(1)
    bitstream.write( (uint32_t)pfgps.getGeometryPatchPointShapeInfoEnabledFlag(), 1 );  // u(1)
  }
  byteAlignment( bitstream );
}

// 7.3.5.5 Geometry frame Params syntax
void PCCBitstreamEncoder::geometryFrameParams( GeometryFrameParams& gfp, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( (uint32_t)gfp.getGeometrySmoothingParamsPresentFlag(), 1 );  // u(1)
  bitstream.write( (uint32_t)gfp.getGeometryScaleParamsPresentFlag(), 1 );      // u(1)
  bitstream.write( (uint32_t)gfp.getGeometryOffsetParamsPresentFlag(), 1 );     // u(1)
  bitstream.write( (uint32_t)gfp.getGeometryRotationParamsPresentFlag(), 1 );   // u(1)
  bitstream.write( (uint32_t)gfp.getGeometryPointSizeInfoPresentFlag(), 1 );    // u(1)
  bitstream.write( (uint32_t)gfp.getGeometryPointShapeInfoPresentFlag(), 1 );   // u(1)
  if ( gfp.getGeometrySmoothingParamsPresentFlag() ) {
    bitstream.write( (uint32_t)gfp.getGeometrySmoothingEnabledFlag(), 1 );  // u(1)
    if ( gfp.getGeometrySmoothingEnabledFlag() ) {
      bitstream.write( (uint32_t)gfp.getGeometrySmoothingGridSizeMinus2(), 7 );  // u(7)
      bitstream.write( (uint32_t)gfp.getGeometrySmoothingThreshold(), 8 );       // u(8)
    }
    bitstream.write( (uint32_t)gfp.getGeometryPatchBlockFilteringEnableFlag(), 1 );  // u(1)
    if ( gfp.getGeometryPatchBlockFilteringEnableFlag() ) {
      bitstream.write( (uint32_t)gfp.getGeometryPatchBlockFilteringLog2ThresholdMinus1(), 2 );  // u(2)
      bitstream.write( (uint32_t)gfp.getGeometryPatchBlockFilteringPassesCountMinus1(), 2 );    // u(3)
      bitstream.write( (uint32_t)gfp.getGeometryPatchBlockFilteringFilterSizeMinus1(), 3 );     // u(3)
    }
  }
  if ( gfp.getGeometryScaleParamsPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      bitstream.write( (uint32_t)gfp.getGeometryScaleOnAxis( d ), 32 );  // u(32)
    }
  }
  if ( gfp.getGeometryOffsetParamsPresentFlag() ) {
    for ( size_t d = 0; d < 3; d++ ) {
      bitstream.writeS( gfp.getGeometryOffsetOnAxis( d ), 32 );  // i32
    }
  }
  if ( gfp.getGeometryRotationParamsPresentFlag() ) {
    for ( size_t d = 0; d < 4; d++ ) {
      bitstream.writeS( gfp.getGeometryRotationQuaternion( d ),
                        32 );  // i32 -> TODO: This should be i(16) according to the CD
    }
  }
  if ( gfp.getGeometryPointSizeInfoPresentFlag() ) {
    bitstream.write( (uint32_t)gfp.getGeometryPointSizeInfo(), 16 );  // u(16)
  }
  if ( gfp.getGeometryPointShapeInfoPresentFlag() ) {
    bitstream.write( (uint32_t)gfp.getGeometryPointShapeInfo(), 4 );  // u(4)
  }
}

// 7.3.5.6 Patch frame attribute parameter set syntax
void PCCBitstreamEncoder::patchFrameAttributeParameterSet( PatchDataGroup&   pdg,
                                                           size_t            index,
                                                           VpccParameterSet& sps,
                                                           PCCBitstream&     bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  auto&  ai         = sps.getAttributeInformation( atlasIndex );
  auto&  pfaps      = pdg.getPatchFrameAttributeParameterSet( index );
  assert( pfaps.getPatchFrameAttributeParameterSetId() == index );
  bitstream.writeUvlc( pfaps.getPatchFrameAttributeParameterSetId() );  // ue(v)
  bitstream.writeUvlc( pfaps.getPatchSequencParameterSetId() );         //  ue(v)

  TRACE_BITSTREAM( "PatchFrameAttributeParameterSetId = %u  \n", pfaps.getPatchFrameAttributeParameterSetId() );
  TRACE_BITSTREAM( "PatchSequencParameterSetId       = %u  \n", pfaps.getPatchSequencParameterSetId() );

  size_t attributeDimension = ai.getAttributeDimensionMinus1( pfaps.getPatchFrameAttributeParameterSetId() ) + 1;
  TRACE_BITSTREAM( "attributeDimension = %lu \n", attributeDimension );
  if ( ai.getAttributeParamsEnabledFlag() ) {
    attributeFrameParams( pfaps.getAttributeFrameParams(), attributeDimension, bitstream );
  }
  if ( ai.getAttributePatchParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)pfaps.getAttributePatchScaleParamsEnabledFlag(), 1 );   //  u(1)
    bitstream.write( (uint32_t)pfaps.getAttributePatchOffsetParamsEnabledFlag(), 1 );  // u(1)
  }
  byteAlignment( bitstream );
}

// 7.3.5.7 Attribute frame Params syntax
void PCCBitstreamEncoder::attributeFrameParams( AttributeFrameParams& afp,
                                                size_t                attributeDimension,
                                                PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  for ( size_t i = 0; i < attributeDimension; i++ ) {
    bitstream.write( (uint32_t)afp.getAttributeSmoothingParamsPresentFlag( i ), 1 );  // u(1)
  }
  bitstream.write( (uint32_t)afp.getAttributeScaleParamsPresentFlag(), 1 );   // u(1)
  bitstream.write( (uint32_t)afp.getAttributeOffsetParamsPresentFlag(), 1 );  // u(1)
  for ( size_t i = 0; i < attributeDimension; i++ ) {
    if ( afp.getAttributeSmoothingParamsPresentFlag( i ) ) {
      bitstream.write( (uint32_t)afp.getAttributeSmoothingGridSizeMinus2( i ), 8 );                // u(8)
      bitstream.write( (uint32_t)afp.getAttributeSmoothingThreshold( i ), 8 );                     // u(8)
      bitstream.write( (uint32_t)afp.getAttributeSmoothingLocalEntropyThreshold( i ), 3 );         // u(3)
      bitstream.write( (uint32_t)afp.getAttributeSmoothingThresholdAttributeVariation( i ), 8 );   // u(8)
      bitstream.write( (uint32_t)afp.getAttributeSmoothingThresholdAttributeDifference( i ), 8 );  // u(8)
    }
  }
  if ( afp.getAttributeScaleParamsPresentFlag() ) {
    for ( size_t i = 0; i < attributeDimension; i++ ) {
      bitstream.write( (uint32_t)afp.getAttributeScale( i ), 32 );  // u(32)
    }
  }
  if ( afp.getAttributeOffsetParamsPresentFlag() ) {
    for ( size_t i = 0; i < attributeDimension; i++ ) bitstream.writeS( afp.getAttributeOffset( i ), 32 );  // i32
  }
}

// 7.3.5.8 Geometry patch parameter set syntax
void PCCBitstreamEncoder::geometryPatchParameterSet( PatchDataGroup& pdg, size_t index, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& gpps  = pdg.getGeometryPatchParameterSet( index );
  auto& pfgps = pdg.getPatchFrameGeometryParameterSet( gpps.getPatchFrameGeometryParameterSetId() );
  assert( gpps.getGeometryPatchParameterSetId() == index );
  bitstream.writeUvlc( gpps.getGeometryPatchParameterSetId() );       // ue(v)
  bitstream.writeUvlc( gpps.getPatchFrameGeometryParameterSetId() );  //  ue(v)
  if ( pfgps.getGeometryPatchScaleParamsEnabledFlag() || pfgps.getGeometryPatchOffsetParamsEnabledFlag() ||
       pfgps.getGeometryPatchRotationParamsEnabledFlag() || pfgps.getGeometryPatchPointSizeInfoEnabledFlag() ||
       pfgps.getGeometryPatchPointShapeInfoEnabledFlag() ) {
    bitstream.write( (uint32_t)gpps.getGeometryPatchParamsPresentFlag(), 1 );  // u(1)
    if ( gpps.getGeometryPatchParamsPresentFlag() )
      geometryPatchParams( gpps.getGeometryPatchParams(), pfgps, bitstream );
  }
  byteAlignment( bitstream );
}

// 7.3.5.9 Geometry patch Params syntax
void PCCBitstreamEncoder::geometryPatchParams( GeometryPatchParams&            gpp,
                                               PatchFrameGeometryParameterSet& gfps,
                                               PCCBitstream&                   bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  if ( gfps.getGeometryPatchScaleParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)gpp.getGeometryPatchScaleParamsPresentFlag(), 1 );  // u(1)
    if ( gpp.getGeometryPatchScaleParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        bitstream.write( (uint32_t)gpp.getGeometryPatchScaleOnAxis( d ), 32 );  // u(32)
      }
    }
  }
  if ( gfps.getGeometryPatchOffsetParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)gpp.getGeometryPatchOffsetParamsPresentFlag(), 1 );  // u(1)
    if ( gpp.getGeometryPatchOffsetParamsPresentFlag() ) {
      for ( size_t d = 0; d < 3; d++ ) {
        bitstream.writeS( gpp.getGeometryPatchOffsetOnAxis( d ), 32 );  // i(32)
      }
    }
  }
  if ( gfps.getGeometryPatchRotationParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)gpp.getGeometryPatchRotationParamsPresentFlag(), 1 );  // u(1)
    if ( gpp.getGeometryPatchRotationParamsPresentFlag() ) {
      for ( size_t d = 0; d < 4; d++ ) {
        bitstream.writeS( gpp.getGeometryPatchRotationQuaternion( d ),
                          32 );  // i(32) -> TODO: i(16) according to the CD
      }
    }
  }
  if ( gfps.getGeometryPatchPointSizeInfoEnabledFlag() ) {
    bitstream.write( (uint32_t)gpp.getGeometryPatchPointSizeInfoPresentFlag(), 1 );  // u(1)
    if ( gpp.getGeometryPatchPointSizeInfoPresentFlag() ) {
      bitstream.write( (uint32_t)gpp.getGeometryPatchPointSizeInfo(), 16 );  // u(16)
    }
  }
  if ( gfps.getGeometryPatchPointShapeInfoEnabledFlag() ) {
    bitstream.write( (uint32_t)gpp.getGeometryPatchPointShapeInfoPresentFlag(), 1 );  // u(1)
    if ( gpp.getGeometryPatchPointShapeInfoPresentFlag() ) {
      bitstream.write( (uint32_t)gpp.getGeometryPatchPointShapeInfo(), 4 );  // u(14
    }
  }
}

// 7.3.5.10 Attribute patch parameter set syntax
void PCCBitstreamEncoder::attributePatchParameterSet( PatchDataGroup&   pdg,
                                                      size_t            index,
                                                      VpccParameterSet& sps,
                                                      PCCBitstream&     bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );

  auto& apps  = pdg.getAttributePatchParameterSet( index );
  auto& pfaps = pdg.getPatchFrameAttributeParameterSet( apps.getPatchFrameAttributeParameterSetId() );
  assert( apps.getAttributePatchParameterSetId() == index );
  bitstream.writeUvlc( apps.getAttributePatchParameterSetId() );
  bitstream.writeUvlc( apps.getPatchFrameAttributeParameterSetId() );
  bitstream.write( apps.getAttributeDimensionMinus1(), 8 );
  size_t attributeDimension = apps.getAttributeDimensionMinus1() + 1;
  if ( pfaps.getAttributePatchScaleParamsEnabledFlag() || pfaps.getAttributePatchOffsetParamsEnabledFlag() ) {
    bitstream.write( (uint32_t)apps.getAttributePatchParamsPresentFlag(), 1 );  // u(1)
    if ( apps.getAttributePatchParamsPresentFlag() ) {
      attributePatchParams( apps.getAttributePatchParams(), pfaps, attributeDimension, bitstream );
    }
  }
  byteAlignment( bitstream );
}

// 7.3.5.11 Attribute patch Params syntax (apps)
void PCCBitstreamEncoder::attributePatchParams( AttributePatchParams&            app,
                                                PatchFrameAttributeParameterSet& afps,
                                                size_t                           dimension,
                                                PCCBitstream&                    bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
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
        bitstream.writeS( app.getAttributePatchOffset( i ), 32 );  // i(32)
      }
    }
  }
}

// 7.3.5.12 Patch frame parameter set syntax
void PCCBitstreamEncoder::patchFrameParameterSet( PatchDataGroup&   pdg,
                                                  size_t            index,
                                                  VpccParameterSet& sps,
                                                  PCCBitstream&     bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  size_t atlasIndex = 0;
  auto&  ai         = sps.getAttributeInformation( atlasIndex );
  auto&  pfps       = pdg.getPatchFrameParameterSet( index );
  assert( pfps.getPatchFrameParameterSetId() == index );
  bitstream.writeUvlc( pfps.getPatchFrameParameterSetId() );          // ue(v)
  bitstream.writeUvlc( pfps.getPatchVpccParameterSetId() );           // ue(v)
  bitstream.writeUvlc( pfps.getGeometryPatchFrameParameterSetId() );  // ue(v)

  TRACE_BITSTREAM( " ai.getAttributeCount() = %u \n", ai.getAttributeCount() );

  for ( size_t i = 0; i < ai.getAttributeCount(); i++ ) {
    bitstream.writeUvlc( pfps.getAttributePatchFrameParameterSetId( i ) );  // ue(v)
  }
  atlasFrameTileInformation( pfps.getAtlasFrameTileInformation(), sps, bitstream );

  bitstream.write( pfps.getLocalOverrideGeometryPatchEnableFlag(), 1 );  // u(1)
  for ( size_t i = 0; i < ai.getAttributeCount(); i++ ) {
    bitstream.write( pfps.getLocalOverrideAttributePatchEnableFlag( i ), 1 );  // u(1)
  }
  bitstream.writeUvlc( pfps.getAdditionalLtPfocLsbLen() );  // ue(v)
//  if ( sps.getProjection45DegreeEnableFlag() ) {
//    bitstream.write( pfps.getProjection45DegreeEnableFlag(), 1 );  // u(1)
//  }
  bitstream.write( pfps.getLodModeEnableFlag(), 1 );  // u(1)
  byteAlignment( bitstream );
}

// jkei : moved up
// void PCCBitstreamEncoder::atlasFrameTileInformation(
//                                                    AtlasFrameTileInformation& pfti,
//                                                     VpccParameterSet&      sps,
//                                                     PCCBitstream&              bitstream ) {
//  TRACE_BITSTREAM( "%s \n", __func__ );
//  bitstream.write( pfti.getSingleTileInAtlasFrameFlag(), 1 );  // u(1)
//  if ( !pfti.getSingleTileInAtlasFrameFlag() ) {
//    bitstream.write( pfti.getUniformTileSpacingFlag(), 1 );  // u(1)
//    if ( pfti.getUniformTileSpacingFlag() ) {
//      bitstream.writeUvlc( pfti.getTileColumnWidthMinus1( 0 ) );  //  ue(v)
//      bitstream.writeUvlc( pfti.getTileRowHeightMinus1( 0 ) );    //  ue(v)
//    } else {
//      bitstream.writeUvlc( pfti.getNumTileColumnsMinus1() );  //  ue(v)
//      bitstream.writeUvlc( pfti.getNumTileRowsMinus1() );     //  ue(v)
//
//      for ( size_t i = 0; i < pfti.getNumTileColumnsMinus1(); i++ ) {
//        bitstream.writeUvlc( pfti.getTileColumnWidthMinus1( i ) );  //  ue(v)
//      }
//      for ( size_t i = 0; i < pfti.getNumTileRowsMinus1(); i++ ) {
//        bitstream.writeUvlc( pfti.getTileRowHeightMinus1( i ) );  //  ue(v)
//      }
//    }
//  }
//  bitstream.write( pfti.getSingleTilePerTileGroupFlag(), 1 );  //  u(1)
//  if ( !pfti.getSingleTilePerTileGroupFlag() ) {
//    uint32_t NumTilesInPatchFrame     = ( pfti.getNumTileColumnsMinus1() + 1 ) * ( pfti.getNumTileRowsMinus1() + 1 );
//    bitstream.writeUvlc( pfti.getNumTileGroupsInAtlasFrameMinus1() );  // ue(v)
//    for ( size_t i = 0; i <= pfti.getNumTileGroupsInAtlasFrameMinus1(); i++ ) {
//      uint8_t bitCount = getFixedLengthCodeBitsCount( NumTilesInPatchFrame + 1 );
//      if ( i > 0 ) {
//        bitstream.write( pfti.getTopLeftTileIdx( i ), bitCount );
//      }  // u(v) : Ceil( Log2( NumTilesInPatchFrame )
//      bitCount = getFixedLengthCodeBitsCount( NumTilesInPatchFrame - pfti.getTopLeftTileIdx( i ) + 1 );
//      bitstream.write( pfti.getBottomRightTileIdxDelta( i ),
//                       bitCount );  // u(v) : Ceil( Log2( NumTilesInPatchFrame − pfti_top_left_tile_idx[ i ] ) )
//    }
//  }
//  bitstream.write( pfti.getSignalledTileGroupIdFlag(), 1 );  // u(1)
//  if ( pfti.getSignalledTileGroupIdFlag() ) {
//    bitstream.writeUvlc( pfti.getSignalledTileGroupIdLengthMinus1() );  // ue(v)
//    for ( size_t i = 0; i <= pfti.getSignalledTileGroupIdLengthMinus1(); i++ ) {
//      uint8_t bitCount = pfti.getSignalledTileGroupIdLengthMinus1() + 1;
//      bitstream.write( pfti.getTileGroupId( i ), bitCount );  // pfti_signalled_tile_group_id_length_minus1 + 1  bits
//    }
//  }
//}

// 7.3.5.14 Patch tile group layer unit syntax
void PCCBitstreamEncoder::patchTileGroupLayerUnit( PatchDataGroup& pdg,
                                                   size_t          index,
                                                   PCCContext&     context,
                                                   PCCBitstream&   bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto& ptglu    = pdg.getPatchTileGroupLayerUnit( index );
  auto& pfluPrev = pdg.getPatchTileGroupLayerUnit( ( std::max )( 0, (int32_t)index - 1 ) );
  ptglu.getPatchTileGroupHeader().setFrameIndex( ptglu.getFrameIndex() );
  ptglu.getPatchTileGroupDataUnit().setFrameIndex( ptglu.getFrameIndex() );
  patchTileGroupHeader( ptglu.getPatchTileGroupHeader(), pfluPrev.getPatchTileGroupHeader(), context, bitstream );
  patchTileGroupDataUnit( ptglu.getPatchTileGroupDataUnit(), ptglu.getPatchTileGroupHeader(), context, bitstream );
}

// 7.3.5.15 Patch tile group header syntax
void PCCBitstreamEncoder::patchTileGroupHeader( PatchTileGroupHeader& ptgh,
                                                PatchTileGroupHeader& pfhPrev,
                                                PCCContext&           context,
                                                PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&  pdg        = context.getPatchDataGroup();
  size_t atlasIndex = 0;
  auto&  pfps       = pdg.getPatchFrameParameterSet( ptgh.getPatchFrameParameterSetId() );
  auto&  psps       = pdg.getPatchVpccParameterSet( pfps.getPatchVpccParameterSetId() );
  auto&  gi         = context.getSps().getGeometryInformation( atlasIndex );
  auto&  sps        = context.getSps();

  bitstream.writeUvlc( ptgh.getPatchFrameParameterSetId() );  // ue(v )
  auto&   pfti     = pfps.getAtlasFrameTileInformation();
  uint8_t bitCount = pfti.getSignalledTileGroupIdLengthMinus1() + 1;
  bitstream.write( ptgh.getAddress(), bitCount );  // u(v)
  // The length of ptgh_address is pfti_signalled_tile_group_id_length_minus1 + 1 bits.
  // If pfti_signalled_tile_group_id_flag is equal to 0, the value of ptgh_address shall be in the range of 0 to
  // pfti_num_tile_groups_in_patch_frame_minus1, inclusive. Otherwise, the value of ptgh_address shall be in the range
  // of 0 to 2( pfti_signalled_tile_group_id_length_minus1 + 1 ) − 1, inclusive.
  bitstream.writeUvlc( ptgh.getType() );  // ue( v )
  bitCount = psps.getLog2MaxPatchFrameOrderCntLsbMinus4() + 4;
  bitstream.write( ptgh.getPatchFrameOrderCntLsb(), bitCount );  // u( v )
  // The length of the ptgh_patch_frm_order_cnt_lsb syntax element is equal to
  // psps_log2_max_patch_frame_order_cnt_lsb_minus4 + 4 bits. The value of the ptgh_patch_frm_order_cnt_lsb shall be in
  // the range of 0 to MaxPatchFrmOrderCntLsb − 1, inclusive.

  TRACE_BITSTREAM( "Id     = %u \n", ptgh.getPatchFrameParameterSetId() );
  TRACE_BITSTREAM( "Adress = %u \n", ptgh.getAddress() );
  TRACE_BITSTREAM( "Type   = %u \n", ptgh.getType() );
  TRACE_BITSTREAM( "POC    = %u \n", ptgh.getPatchFrameOrderCntLsb() );
  TRACE_BITSTREAM( "psps.getNumRefPatchFrameListsInPsps() = %lu\n", psps.getNumRefPatchFrameListsInPsps() );
  TRACE_BITSTREAM( "psps.getNumRefPatchFrameListsInPsps() = %lu \n", psps.getNumRefPatchFrameListsInPsps() );
  TRACE_BITSTREAM( "gi.getGeometry3dCoordinatesBitdepthMinus1() = %lu \n",
                   gi.getGeometry3dCoordinatesBitdepthMinus1() );

  if ( psps.getNumRefPatchFrameListsInPsps() > 0 ) {
    bitstream.write( (uint32_t)ptgh.getRefPatchFrameListSpsFlag(), 1 );  // u( 1 )
  }
  if ( ptgh.getRefPatchFrameListSpsFlag() ) {
    if ( psps.getNumRefPatchFrameListsInPsps() > 1 ) {
      bitCount = getFixedLengthCodeBitsCount( psps.getNumRefPatchFrameListsInPsps() + 1 );
      bitstream.write( ptgh.getRefPatchFrameListIdx(),
                       bitCount );  // u( v ) :Ceil( Log2( psps_num_ref_patch_frame_lists_in_psps ) )
    }
  } else {
    refListStruct( psps.getRefListStruct( psps.getNumRefPatchFrameListsInPsps() ), psps, bitstream );
  }
  uint8_t rlsIdx =
      psps.getNumRefPatchFrameListsInPsps() ? ptgh.getRefPatchFrameListIdx() : psps.getNumRefPatchFrameListsInPsps();
  size_t numLtrpEntries = 0;
  for ( size_t i = 0; i < psps.getRefListStruct( rlsIdx ).getNumRefEntries(); i++ ) {
    if ( !psps.getRefListStruct( rlsIdx ).getStRefAtalsFrameFlag( i ) ) { numLtrpEntries++; }
  }

  for ( size_t j = 0; j < numLtrpEntries; j++ ) {                               // numLtrpEntries=numLtrPatchFrmEntries
    bitstream.write( (uint32_t)ptgh.getAdditionalPfocLsbPresentFlag( j ), 1 );  // u( 1 )
    if ( ptgh.getAdditionalPfocLsbPresentFlag( j ) ) {
      uint8_t bitCount = pfps.getAdditionalLtPfocLsbLen();
      bitstream.write( ptgh.getAdditionalPfocLsbVal( j ), bitCount );  // u( v ) pfps_additional_lt_pfoc_lsb_len
    }
  }

  ptgh.setNormalAxisMinValueQuantizer( 0 );
  ptgh.setNormalAxisMaxDeltaValueQuantizer( 0 );
  if ( psps.getNormalAxisLimitsQuantizationEnableFlag() ) {
    ptgh.setNormalAxisMinValueQuantizer( gbitCountSize[context.getSps().getMinLevel()] );
    ptgh.setNormalAxisMaxDeltaValueQuantizer( gbitCountSize[context.getSps().getMinLevel()] );

    bitstream.write( ptgh.getNormalAxisMinValueQuantizer(), 5 );  // u(5)
    if ( psps.getNormalAxisMaxDeltaValueEnableFlag() ) {
      bitstream.write( ptgh.getNormalAxisMaxDeltaValueQuantizer(), 5 );  // u(5)
    }
  }
  const uint8_t maxBitCountForMinDepth = uint8_t( gi.getGeometry3dCoordinatesBitdepthMinus1() );
  const uint8_t maxBitCountForMaxDepth = uint8_t( gi.getGeometry3dCoordinatesBitdepthMinus1() );
  // min
  ptgh.setInterPredictPatch3dShiftNormalAxisBitCountMinus1( maxBitCountForMinDepth );

  // max
  if ( pfps.getProjection45DegreeEnableFlag() == 0 ) {
    ptgh.setInterPredictPatch2dDeltaSizeDBitCountMinus1( maxBitCountForMaxDepth );
  } else {
    ptgh.setInterPredictPatch2dDeltaSizeDBitCountMinus1( maxBitCountForMaxDepth + 1 );
  }

  if ( ptgh.getType() == PATCH_FRAME_P && psps.getRefListStruct( rlsIdx ).getNumRefEntries() > 1 ) {
    bitstream.write( (uint32_t)ptgh.getNumRefIdxActiveOverrideFlag(), 1 );  // u( 1 )
    if ( ptgh.getNumRefIdxActiveOverrideFlag() ) {
      bitstream.writeUvlc( ptgh.getNumRefIdxActiveMinus1() );  // ue(v)
    }
  }
  if ( ptgh.getType() == PATCH_FRAME_I ) {
    bitstream.write( (uint32_t)ptgh.getInterPredictPatch2dShiftUBitCountMinus1(), 8 );              // u( 8 )
    bitstream.write( (uint32_t)ptgh.getInterPredictPatch2dShiftVBitCountMinus1(), 8 );              // u( 8 )
    bitstream.write( (uint32_t)ptgh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1(), 8 );    // u( 8 )
    bitstream.write( (uint32_t)ptgh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1(), 8 );  // u( 8 )
    bitstream.write( (uint32_t)ptgh.getInterPredictPatchLodBitCount(), 8 );                         // u( 8 )
  } else {
    bool countFlag[6] = {0, 0, 0, 0, 0, 0};
    bool countSumFlag = 0;
    countFlag[0] =
        ptgh.getInterPredictPatch2dShiftUBitCountMinus1() > pfhPrev.getInterPredictPatch2dShiftUBitCountMinus1();
    countFlag[1] =
        ptgh.getInterPredictPatch2dShiftVBitCountMinus1() > pfhPrev.getInterPredictPatch2dShiftVBitCountMinus1();
    countFlag[2] = ptgh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1() >
                   pfhPrev.getInterPredictPatch3dShiftTangentAxisBitCountMinus1();
    countFlag[3] = ptgh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1() >
                   pfhPrev.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1();
    countFlag[4] = ptgh.getInterPredictPatchLodBitCount() > pfhPrev.getInterPredictPatchLodBitCount();
    for ( size_t i = 0; i < 5; i++ ) countSumFlag |= countFlag[i];
    ptgh.setInterPredictPatchBitCountFlag( countSumFlag );
    ptgh.setInterPredictPatch2dShiftUBitCountFlag( countFlag[0] );
    ptgh.setInterPredictPatch2dShiftVBitCountFlag( countFlag[1] );
    ptgh.setInterPredictPatch3dShiftTangentAxisBitCountFlag( countFlag[2] );
    ptgh.setInterPredictPatch3dShiftBitangentAxisBitCountFlag( countFlag[3] );
    ptgh.setInterPredictPatchLodBitCountFlag( countFlag[4] );
    if ( !ptgh.getInterPredictPatchBitCountFlag() || !ptgh.getInterPredictPatch2dShiftUBitCountFlag() ) {
      ptgh.setInterPredictPatch2dShiftUBitCountMinus1( pfhPrev.getInterPredictPatch2dShiftUBitCountMinus1() );
    }
    if ( !ptgh.getInterPredictPatchBitCountFlag() || !ptgh.getInterPredictPatch2dShiftVBitCountFlag() ) {
      ptgh.setInterPredictPatch2dShiftVBitCountMinus1( pfhPrev.getInterPredictPatch2dShiftVBitCountMinus1() );
    }
    if ( !ptgh.getInterPredictPatchBitCountFlag() || !ptgh.getInterPredictPatch3dShiftTangentAxisBitCountFlag() ) {
      ptgh.setInterPredictPatch3dShiftTangentAxisBitCountMinus1(
          pfhPrev.getInterPredictPatch3dShiftTangentAxisBitCountMinus1() );
    }
    if ( !ptgh.getInterPredictPatchBitCountFlag() || !ptgh.getInterPredictPatch3dShiftBitangentAxisBitCountFlag() ) {
      ptgh.setInterPredictPatch3dShiftBitangentAxisBitCountMinus1(
          pfhPrev.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1() );
    }
    if ( !ptgh.getInterPredictPatchBitCountFlag() || !ptgh.getInterPredictPatchLodBitCountFlag() ) {
      ptgh.setInterPredictPatchLodBitCount( pfhPrev.getInterPredictPatchLodBitCount() );
    }
    bitstream.write( (uint32_t)ptgh.getInterPredictPatchBitCountFlag(), 1 );  // u( 1 )
    if ( ptgh.getInterPredictPatchBitCountFlag() ) {
      bitstream.write( (uint32_t)ptgh.getInterPredictPatch2dShiftUBitCountFlag(), 1 );  // u( 1 )
      if ( ptgh.getInterPredictPatch2dShiftUBitCountFlag() ) {
        bitstream.write( (uint32_t)ptgh.getInterPredictPatch2dShiftUBitCountMinus1(), 8 );  // u( 8 )
      }
      bitstream.write( (uint32_t)ptgh.getInterPredictPatch2dShiftVBitCountFlag(), 1 );  // u( 1 )
      if ( ptgh.getInterPredictPatch2dShiftVBitCountFlag() ) {
        bitstream.write( (uint32_t)ptgh.getInterPredictPatch2dShiftVBitCountMinus1(), 8 );  // u( 8 )
      }
      bitstream.write( (uint32_t)ptgh.getInterPredictPatch3dShiftTangentAxisBitCountFlag(), 1 );  // u( 1 )
      if ( ptgh.getInterPredictPatch3dShiftTangentAxisBitCountFlag() ) {
        bitstream.write( (uint32_t)ptgh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1(), 8 );  // u( 8 )
      }
      bitstream.write( (uint32_t)ptgh.getInterPredictPatch3dShiftBitangentAxisBitCountFlag(), 1 );  // u( 1 )
      if ( ptgh.getInterPredictPatch3dShiftBitangentAxisBitCountFlag() ) {
        bitstream.write( (uint32_t)ptgh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1(), 8 );  // u( 8 )
      }

      bitstream.write( (uint32_t)ptgh.getInterPredictPatchLodBitCountFlag(), 1 );  // u( 1 )
      if ( ptgh.getInterPredictPatchLodBitCountFlag() ) {
        bitstream.write( (uint32_t)ptgh.getInterPredictPatchLodBitCount(), 8 );  // u( 8 )
      }
    }
  }
//  if ( sps.getEnhancedOccupancyMapForDepthFlag() && sps.getEOMTexturePatch() ) {
//    bitstream.write( (uint32_t)ptgh.getEOMPatchNbPatchBitCountMinus1(), 8 );  // u( 8 )
//    bitstream.write( (uint32_t)ptgh.getEOMPatchMaxEPBitCountMinus1(), 8 );    // u( 8 )
//  }
  TRACE_BITSTREAM( "RawPatchEnabledFlag = %d \n", sps.getRawPatchEnabledFlag( atlasIndex ) );
  if ( sps.getRawPatchEnabledFlag( atlasIndex ) ) {
    bitstream.write( (uint32_t)ptgh.getRaw3dShiftBitCountPresentFlag(), 1 );  // u( 1 )
    if ( ptgh.getRaw3dShiftBitCountPresentFlag() ) {
      bitstream.write( (uint32_t)ptgh.getRaw3dShiftAxisBitCountMinus1(),
                       gi.getGeometry3dCoordinatesBitdepthMinus1() + 1 );
    }
  } else {
    size_t bitCountPcmU1V1D1 = gi.getGeometry3dCoordinatesBitdepthMinus1() - gi.getGeometryNominal2dBitdepthMinus1();
    ptgh.setRaw3dShiftAxisBitCountMinus1( bitCountPcmU1V1D1 - 1 );
  }
  TRACE_BITSTREAM(
      "InterPredictPatchBitCount Flag %d %d %d %d %d %d %d Count = %u %u %u %u %u %u \n",
      ptgh.getInterPredictPatchBitCountFlag(), ptgh.getInterPredictPatch2dShiftUBitCountFlag(),
      ptgh.getInterPredictPatch2dShiftVBitCountFlag(), ptgh.getInterPredictPatch3dShiftTangentAxisBitCountFlag(),
      ptgh.getInterPredictPatch3dShiftBitangentAxisBitCountFlag(), ptgh.getInterPredictPatchLodBitCountFlag(),
      ptgh.getRaw3dShiftBitCountPresentFlag(), ptgh.getInterPredictPatch2dShiftUBitCountMinus1(),
      ptgh.getInterPredictPatch2dShiftVBitCountMinus1(), ptgh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1(),
      ptgh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1(), ptgh.getInterPredictPatchLodBitCount(),
      ptgh.getRaw3dShiftAxisBitCountMinus1() );
  byteAlignment( bitstream );
}

// 7.3.5.16 Reference list structure syntax (OLD)
void PCCBitstreamEncoder::refListStruct( RefListStruct& rls, PatchVpccParameterSet& psps, PCCBitstream& bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.writeUvlc( rls.getNumRefEntries() );  // ue(v)
  TRACE_BITSTREAM( "NumRefEntries = %lu  \n", rls.getNumRefEntries() );
  //rls.allocate(); //why?!
  for ( size_t i = 0; i < rls.getNumRefEntries(); i++ ) {
    if ( psps.getLongTermRefPatchFramesFlag() ) {
      bitstream.write( rls.getStRefAtalsFrameFlag( i ), 1 );  // u(1)
      if ( rls.getStRefAtalsFrameFlag( i ) ) {
        bitstream.writeUvlc( rls.getAbsDeltaAfocSt( i ) );  // ue(v)
        if ( rls.getAbsDeltaAfocSt( i ) > 0 ) {
          bitstream.write( rls.getStrpfEntrySignFlag( i ), 1 );  // u(1)
        } else {
          uint8_t bitCount = psps.getLog2MaxPatchFrameOrderCntLsbMinus4() + 4;
          bitstream.write( rls.getAfocLsbLt( i ),
                           bitCount );  // u(v) psps_log2_max_patch_frame_order_cnt_lsb_minus4  + 4 bits
        }
      }
    }
  }
  TRACE_BITSTREAM( "%s done \n", __func__ );
}
// 7.3.5.16 Reference list structure syntax (NEW jkei: moved up)
// void PCCBitstreamEncoder::refListStruct( RefListStruct&                 rls,
//                                         AtlasSequenceParameterSetRBSP& asps,
//                                         PCCBitstream&                  bitstream ) {
//  TRACE_BITSTREAM( "%s \n", __func__ );
//  bitstream.writeUvlc( rls.getNumRefEntries() );  // ue(v)
//  TRACE_BITSTREAM( "NumRefEntries = %lu  \n", rls.getNumRefEntries() );
//  rls.allocate();
//  for ( size_t i = 0; i < rls.getNumRefEntries(); i++ ) {
//    if ( asps.getLongTermRefAtlasFramesFlag() ) {
//      bitstream.write( rls.getStRefAtalsFrameFlag( i ), 1 );  // u(1)
//      if ( rls.getStRefAtalsFrameFlag( i ) ) {
//        bitstream.writeUvlc( rls.getAbsDeltaPfocSt( i ) );  // ue(v)
//        if ( rls.getAbsDeltaPfocSt( i ) > 0 ) {
//          bitstream.write( rls.getStrpfEntrySignFlag( i ), 1 );  // u(1)
//        } else {
//          uint8_t bitCount = asps.getLog2MaxAtlasFrameOrderCntLsbMinus4() + 4;
//          bitstream.write( rls.getAfocLsbLt( i ),
//                           bitCount );  // u(v) psps_log2_max_patch_frame_order_cnt_lsb_minus4  + 4 bits
//        }
//      }
//    }
//  }
//  TRACE_BITSTREAM( "%s done \n", __func__ );
//}

// 7.3.6.1 General patch tile group data unit syntax
void PCCBitstreamEncoder::patchTileGroupDataUnit( PatchTileGroupDataUnit& ptgdu,
                                                  PatchTileGroupHeader&   ptgh,
                                                  PCCContext&             context,
                                                  PCCBitstream&           bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "ptgh.getType()        = %lu \n", ptgh.getType() );
  for ( size_t puCount = 0; puCount < ptgdu.getPatchCount(); puCount++ ) {
    bitstream.writeUvlc( uint32_t( ptgdu.getPatchMode( puCount ) ) );
    TRACE_BITSTREAM( "patchMode = %lu \n", ptgdu.getPatchMode( puCount ) );
    ptgdu.getPatchInformationData( puCount ).setFrameIndex( ptgdu.getFrameIndex() );
    ptgdu.getPatchInformationData( puCount ).setPatchIndex( puCount );
    patchInformationData( ptgdu.getPatchInformationData( puCount ),
                          ptgdu.getPatchInformationData( puCount ).getPatchIndex(), ptgdu.getPatchMode( puCount ), ptgh,
                          context, bitstream );
  }
  TRACE_BITSTREAM( "ptgdu.getPatchCount() = %lu \n", ptgdu.getPatchCount() );
  byteAlignment( bitstream );
}

// 7.3.6.2 Patch information data syntax
void PCCBitstreamEncoder::patchInformationData( PatchInformationData& pid,
                                                size_t                patchIndex,
                                                size_t                patchMode,
                                                PatchTileGroupHeader& ptgh,
                                                PCCContext&           context,
                                                PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  if ( ( PCCPatchFrameType( ptgh.getType() ) ) == PATCH_FRAME_P && patchMode == PATCH_MODE_P_SKIP ) {
    // skip mode.
    // currently not supported but added it for convenience. Could easily be removed
  } else if ( ( ( PCCPatchFrameType( ptgh.getType() ) ) == PATCH_FRAME_I && patchMode == PATCH_MODE_I_INTRA ) ||
              ( ( PCCPatchFrameType( ptgh.getType() ) ) == PATCH_FRAME_P && patchMode == PATCH_MODE_P_INTRA ) ) {
    auto& pdu = pid.getPatchDataUnit();
    pdu.setFrameIndex( pid.getFrameIndex() );
    pdu.setPatchIndex( pid.getPatchIndex() );
    patchDataUnit( pdu, ptgh, context, bitstream );
  } else if ( ( PCCPatchFrameType( ptgh.getType() ) ) == PATCH_FRAME_P && patchMode == PATCH_MODE_P_INTER ) {
    auto& ipdu = pid.getInterPatchDataUnit();
    ipdu.setFrameIndex( pid.getFrameIndex() );
    ipdu.setPatchIndex( pid.getPatchIndex() );
    deltaPatchDataUnit( ipdu, ptgh, context, bitstream );
  } else if ( ( ( PCCPatchFrameType( ptgh.getType() ) ) == PATCH_FRAME_I && patchMode == PATCH_MODE_I_Raw ) ||
              ( ( PCCPatchFrameType( ptgh.getType() ) ) == PATCH_FRAME_P && patchMode == PATCH_MODE_P_Raw ) ) {
    auto& ppdu = pid.getRawPatchDataUnit();
    ppdu.setFrameIndex( pid.getFrameIndex() );
    ppdu.setPatchIndex( pid.getPatchIndex() );
    pcmPatchDataUnit( ppdu, ptgh, context, bitstream );
  } else if ( ( ( PCCPatchFrameType( ptgh.getType() ) ) == PATCH_FRAME_I && patchMode == PATCH_MODE_I_EOM ) ||
              ( ( PCCPatchFrameType( ptgh.getType() ) ) == PATCH_FRAME_P && patchMode == PATCH_MODE_P_EOM ) ) {
    auto& epdu = pid.getEomPatchDataUnit();
    eomPatchDataUnit( epdu, ptgh, context, bitstream );
  }
}

// 7.3.6.3 Patch data unit syntax
void PCCBitstreamEncoder::patchDataUnit( PatchDataUnit&        pdu,
                                         PatchTileGroupHeader& ptgh,
                                         PCCContext&           context,
                                         PCCBitstream&         bitstream ) {
  auto& sps                          = context.getSps();
  auto  ptghPatchFrameParameterSetId = ptgh.getPatchFrameParameterSetId();
  auto& pfps = context.getPatchDataGroup().getPatchFrameParameterSet( ptghPatchFrameParameterSetId );
  auto  pfpsPatchVpccParameterSetId = pfps.getPatchVpccParameterSetId();
  auto& psps = context.getPatchDataGroup().getPatchVpccParameterSet( pfpsPatchVpccParameterSetId );
  TRACE_BITSTREAM( "%s \n", __func__ );
  bitstream.write( uint32_t( pdu.getPdu2dPosX() ), ptgh.getInterPredictPatch2dShiftUBitCountMinus1() + 1 );  // u(v)
  bitstream.write( uint32_t( pdu.getPdu2dPosY() ), ptgh.getInterPredictPatch2dShiftVBitCountMinus1() + 1 );  // u(v)
  bitstream.writeSvlc( int32_t( pdu.getPdu2dDeltaSizeX() ) );  // se(v) The way it is implemented in TM
  bitstream.writeSvlc( int32_t( pdu.getPdu2dDeltaSizeY() ) );  // se(v) The way it is implemented in TM
  bitstream.write( uint32_t( pdu.getPdu3dPosX() ),
                   ptgh.getInterPredictPatch3dShiftTangentAxisBitCountMinus1() + 1 );  // u(v)
  bitstream.write( uint32_t( pdu.getPdu3dPosY() ),
                   ptgh.getInterPredictPatch3dShiftBitangentAxisBitCountMinus1() + 1 );  // u(v)
  bitstream.write( uint32_t( pdu.getPdu3dPosMinZ() ),
                   ptgh.getInterPredictPatch3dShiftNormalAxisBitCountMinus1() + 1 );  // u(v)
  if ( psps.getNormalAxisMaxDeltaValueEnableFlag() ) {
    bitstream.write( uint32_t( pdu.getPdu3dPosDeltaMaxZ() ),
                     ptgh.getInterPredictPatch2dDeltaSizeDBitCountMinus1() + 1 );  // u(v)
  }
  bitstream.write( uint32_t( pdu.getProjectPlane() ), 3 );  // u(3) 0,1,2(near 0,1,2)
  if ( psps.getUseEightOrientationsFlag() ) {
    bitstream.write( pdu.getPduOrientationIndex(), 3 );  // u(3)
  } else {
    bitstream.write( pdu.getPduOrientationIndex(), 1 );  // u(1)
  }
  if ( pfps.getLodModeEnableFlag() ) {
    bitstream.write( uint32_t( pdu.getLodEnableFlag() ),
                     1 );  // u(1) //    pdu.setLodEnableFlag(bitstream.read( 1 ) ); //u1
    if ( pdu.getLodEnableFlag() ) {
      bitstream.writeUvlc( uint32_t( pdu.getLodScaleXminus1() ) );
      bitstream.writeUvlc( uint32_t( pdu.getLodScaleY() ) );
    }
  }
  if ( pfps.getProjection45DegreeEnableFlag() ) {
    bitstream.write( uint32_t( pdu.get45DegreeProjectionPresentFlag() ), 1 );  // u(1)
  }

  if ( pdu.get45DegreeProjectionPresentFlag() ) {
    bitstream.write( uint32_t( pdu.get45DegreeProjectionRotationAxis() ), 2 );  // u(2)
  }
//  if ( sps.getPointLocalReconstructionEnabledFlag() ) {
//    pointLocalReconstructionData( pdu.getPointLocalReconstructionData(), context, bitstream );
//  }
  TRACE_BITSTREAM(
      "Patch(%zu/%zu) => UV %4lu %4lu S=%4ld %4ld P=%zu O=%d A=%lu %lu %lu P45= %d %d lod=(%lu) %lu %lu\n ",
      pdu.getPatchIndex(), pdu.getFrameIndex(), pdu.getPdu2dPosX(), pdu.getPdu2dPosY(), pdu.getPdu2dDeltaSizeX(),
      pdu.getPdu2dDeltaSizeY(), (size_t)pdu.getProjectPlane(), pdu.getPduOrientationIndex(), pdu.getPdu3dPosX(),
      pdu.getPdu3dPosY(), pdu.getPdu3dPosMinZ(), pdu.get45DegreeProjectionPresentFlag(),
      pdu.get45DegreeProjectionRotationAxis(), pdu.getLodEnableFlag(), pdu.getLodScaleXminus1(), pdu.getLodScaleY() );
}

// 7.3.6.4  Delta Patch data unit syntax TODO: Missing 10-projection syntax element?
void PCCBitstreamEncoder::deltaPatchDataUnit( InterPatchDataUnit&   ipdu,
                                              PatchTileGroupHeader& ptgh,
                                              PCCContext&           context,
                                              PCCBitstream&         bitstream ) {
  auto& sps                          = context.getSps();
  auto  ptghPatchFrameParameterSetId = ptgh.getPatchFrameParameterSetId();
  auto& pfps = context.getPatchDataGroup().getPatchFrameParameterSet( ptghPatchFrameParameterSetId );
  auto  pfpsPatchVpccParameterSetId = pfps.getPatchVpccParameterSetId();
  auto& psps = context.getPatchDataGroup().getPatchVpccParameterSet( pfpsPatchVpccParameterSetId );

  TRACE_BITSTREAM( "%s \n", __func__ );
//  if ( atgh.getAtghNumRefdxActiveMinus1() > 0 ) bitstream.writeUvlc( int32_t( ipdu.getIpduRefIndex() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpduRefPatchIndex() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu2dPosX() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu2dPosY() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu2dDeltaSizeX() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu2dDeltaSizeY() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu3dPosX() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu3dPosY() ) );
  bitstream.writeSvlc( int32_t( ipdu.getIpdu3dPosMinZ() ) );
  
  if ( psps.getNormalAxisMaxDeltaValueEnableFlag() )
    bitstream.writeSvlc( int32_t( ipdu.getIpdu3dPosDeltaMaxZ() ) );
  
//  if ( sps.getPointLocalReconstructionEnabledFlag() ) {
//    pointLocalReconstructionData( ipdu.getPointLocalReconstructionData(), context, bitstream );
//  }

  TRACE_BITSTREAM(
      "%zu frame %zu DeltaPatch => DeltaIdx = %d ShiftUV = %ld %ld DeltaSize = %ld %ld Axis = %ld %ld %ld\n",
      ipdu.getIpduRefIndex(), ipdu.getIpduRefPatchIndex(), ipdu.getIpdu2dPosX(), ipdu.getIpdu2dPosY(),
      ipdu.getIpdu2dDeltaSizeX(), ipdu.getIpdu2dDeltaSizeY(), ipdu.getIpdu3dPosX(), ipdu.getIpdu3dPosY(),
      ipdu.getIpdu3dPosMinZ(), ipdu.getIpdu3dPosDeltaMaxZ() );
}

// 7.3.6.5 raw patch data unit syntax
void PCCBitstreamEncoder::pcmPatchDataUnit( RawPatchDataUnit&     ppdu,
                                            PatchTileGroupHeader& ptgh,
                                            PCCContext&           context,
                                            PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&  sps        = context.getSps();
  size_t atlasIndex = 0;
  if ( sps.getRawSeparateVideoPresentFlag( atlasIndex ) ) {
    bitstream.write( ppdu.getRpduPatchInRawVideoFlag(), 1 );
  }                                                                                                            // u(1)
  bitstream.write( uint32_t( ppdu.getRpdu2dPosX() ), ptgh.getInterPredictPatch2dShiftUBitCountMinus1() + 1 );  // u(v)
  bitstream.write( uint32_t( ppdu.getRpdu2dPosY() ), ptgh.getInterPredictPatch2dShiftVBitCountMinus1() + 1 );  // u(v)
  bitstream.writeSvlc( int32_t( ppdu.getRpdu2dDeltaSizeX() ) );                                                // se(v)
  bitstream.writeSvlc( int32_t( ppdu.getRpdu2dDeltaSizeY() ) );                                                // se(v)
  bitstream.write( uint32_t( ppdu.getRpdu3dPosX() ), ptgh.getRaw3dShiftAxisBitCountMinus1() + 1 );             // u(v)
  bitstream.write( uint32_t( ppdu.getRpdu3dPosY() ), ptgh.getRaw3dShiftAxisBitCountMinus1() + 1 );             // u(v)
  bitstream.write( uint32_t( ppdu.getRpdu3dPosZ() ), ptgh.getRaw3dShiftAxisBitCountMinus1() + 1 );             // u(v)
  bitstream.writeUvlc( int32_t( ppdu.getRpduRawPoints() ) );
  TRACE_BITSTREAM(
      "Raw Patch => UV %4lu %4lu  S=%4ld %4ld  UVD1=%4ld %4ld %4ld NumPcmPoints=%lu PatchInRawVideoFlag=%d \n",
      ppdu.getRpdu2dPosX(), ppdu.getRpdu2dPosY(), ppdu.getRpdu2dDeltaSizeX(), ppdu.getRpdu2dDeltaSizeY(),
      ppdu.getRpdu3dPosX(), ppdu.getRpdu3dPosY(), ppdu.getRpdu3dPosZ(), ppdu.getRpduRawPoints(),
      ppdu.getRpduPatchInRawVideoFlag() );
}

// 7.3.6.x EOM patch data unit syntax
void PCCBitstreamEncoder::eomPatchDataUnit( EOMPatchDataUnit&     epdu,
                                            PatchTileGroupHeader& ptgh,
                                            PCCContext&           context,
                                            PCCBitstream&         bitstream ) {
  TRACE_BITSTREAM( "%s \n", __func__ );

  bitstream.write( uint32_t( epdu.getEpdu2dPosX() ), ptgh.getInterPredictPatch2dShiftUBitCountMinus1() + 1 );  // u(v)
  bitstream.write( uint32_t( epdu.getEpdu2dPosY() ), ptgh.getInterPredictPatch2dShiftVBitCountMinus1() + 1 );  // u(v)
  bitstream.writeSvlc( int32_t( epdu.getEpdu2dDeltaSizeX() ) );                                                // se(v)
  bitstream.writeSvlc( int32_t( epdu.getEpdu2dDeltaSizeY() ) );                                                // se(v)
  bitstream.write( uint32_t( epdu.getEpduAssociatedPatchesCountMinus1() ), 8 );                                // max255
  for ( size_t cnt = 0; cnt < epdu.getEpduAssociatedPatchesCountMinus1() + 1; cnt++ ) {
    bitstream.write( uint32_t( epdu.getEpduAssociatedPatches()[cnt] ), 8 );
    bitstream.writeUvlc( uint32_t( epdu.getEpduEomPointsPerPatch()[cnt] ) );
  }
  TRACE_BITSTREAM( "EOM Patch => UV %4lu %4lu  N=%4ld\n", epdu.getEpdu2dPosX(), epdu.getEpdu2dPosY(),
                   epdu.getEpduAssociatedPatchesCountMinus1() + 1 );
  for ( size_t cnt = 0; cnt < epdu.getEpduAssociatedPatchesCountMinus1() + 1; cnt++ )
    TRACE_BITSTREAM( "%4ld, %4ld\n", epdu.getEpduAssociatedPatches()[cnt], epdu.getEpduEomPointsPerPatch()[cnt] );
}

// 7.3.6.6 Point local reconstruction data syntax
void PCCBitstreamEncoder::pointLocalReconstructionData( PointLocalReconstructionData& plrd,
                                                        PCCContext&                   context,
                                                        PCCBitstream&                 bitstream ) {
  auto& plri = context.getSps().getPointLocalReconstructionInformation();
  TRACE_BITSTREAM( "%s \n", __func__ );
  TRACE_BITSTREAM( "WxH= %lu x %lu \n", plrd.getBlockToPatchMapWidth(), plrd.getBlockToPatchMapHeight() );

  const size_t  blockCount   = plrd.getBlockToPatchMapWidth() * plrd.getBlockToPatchMapHeight();
  const uint8_t bitCountMode = uint8_t( getFixedLengthCodeBitsCount( uint32_t( plri.getNumberOfModesMinus1() ) ) );
  TRACE_BITSTREAM( "  bitCountMode = %u \n", bitCountMode );

  if ( blockCount > plri.getBlockThresholdPerPatchMinus1() + 1 ) {
    bitstream.write( uint32_t( plrd.getLevelFlag() ), 1 );  // u(1)
  }
  TRACE_BITSTREAM( "  LevelFlag = %u \n", plrd.getLevelFlag() );
  if ( plrd.getLevelFlag() ) {
    bitstream.write( uint32_t( plrd.getPresentFlag() ), 1 );                                             // u(1)
    if ( plrd.getPresentFlag() ) { bitstream.write( uint32_t( plrd.getModeMinus1() ), bitCountMode ); }  // u(v)
    TRACE_BITSTREAM( "  ModePatch: Present = %d ModeMinus1 = %d \n", plrd.getPresentFlag(),
                     plrd.getPresentFlag() ? (int32_t)plrd.getModeMinus1() : -1 );
  } else {
    for ( size_t i = 0; i < blockCount; i++ ) {
      bitstream.write( uint32_t( plrd.getBlockPresentFlag( i ) ), 1 );  // u(1)
      if ( plrd.getBlockPresentFlag( i ) ) {
        bitstream.write( uint32_t( plrd.getBlockModeMinus1( i ) ), bitCountMode );  // u(v)
      }
      TRACE_BITSTREAM( "  Mode[ %4lu / %4lu ]: Present = %d ModeMinus1 = %d \n", i, blockCount,
                       plrd.getBlockPresentFlag( i ),
                       plrd.getBlockPresentFlag( i ) ? plrd.getBlockModeMinus1( i ) : -1 );
    }
  }
#ifdef BITSTREAM_TRACE
  for ( size_t v0 = 0; v0 < plrd.getBlockToPatchMapHeight(); ++v0 ) {
    for ( size_t u0 = 0; u0 < plrd.getBlockToPatchMapWidth(); ++u0 ) {
      size_t i = v0 * plrd.getBlockToPatchMapWidth() + u0;
      TRACE_BITSTREAM( "Patch Block[ %2lu %2lu <=> %4lu ] / [ %2lu %2lu ] Level = %d Present = %d Mode = %d \n", u0, v0,
                       i, plrd.getBlockToPatchMapWidth(), plrd.getBlockToPatchMapHeight(), plrd.getLevelFlag(),
                       plrd.getLevelFlag() ? plrd.getPresentFlag() : plrd.getBlockPresentFlag( i ),
                       plrd.getLevelFlag() ? plrd.getPresentFlag() ? (int32_t)plrd.getModeMinus1() : -1
                                           : plrd.getBlockPresentFlag( i ) ? plrd.getBlockModeMinus1( i ) : -1 );
    }
  }
#endif
}

// 7.3.5.22 Supplemental enhancement information message syntax TODO: Implement
void PCCBitstreamEncoder::seiMessage( PatchDataGroup& pdg,
                                      size_t          index,
                                      PCCContext&     context,
                                      PCCBitstream&   bitstream ) {}

// JR TODO: continue
// B.2 Sample stream V-PCC unit syntax and semantics
// B.2.1 Sample stream V-PCC header syntax
void PCCBitstreamEncoder::sampleStreamVpccHeader( PCCBitstream& bitstream, uint32_t precisionBytesMinus1 ) {
  TRACE_BITSTREAM("sampleStreamVpccHeader\n");
  bitstream.write( uint32_t( precisionBytesMinus1 ), 3 );  // u(3)
  bitstream.write( 0, 5 );                                                      // u(5)
}

// B.2.2 Sample stream V-PCC unit syntax
void PCCBitstreamEncoder::sampleStreamVpccUnit( PCCBitstream& bitstream, VpccUnitStream& vpccUS, VpccUnit& vpccu) {
  TRACE_BITSTREAM("sampleStreamVpccUnit\n");
  bitstream.write( uint32_t( vpccu.getVpccUnitSize() ), 8 * ( vpccUS.getSsvhUnitSizePrecisionBytesMinus1() + 1 ) );  // u(v)
  TRACE_BITSTREAM( "vpccUnitType: %hhu VpccUnitSize: %zu\n", (uint8_t) vpccu.getVpccUnitType(), vpccu.getVpccUnitSize() );
  for ( size_t i = 0; i < vpccu.getVpccUnitSize(); i++ ) {
     bitstream.write( uint32_t( vpccu.getVpccUnitData( i ) ), 8, true );  // b(8)
  }
  //vpccUnitInSampleStream( bitstream, vpccu);
  //vpccUnit(context, bitstream, vpccu.getVpccUnitType() );
}

// C.2 Sample stream NAL unit syntax and semantics
// C.2.1 Sample stream NAL header syntax
void PCCBitstreamEncoder::sampleStreamNalHeader( PCCBitstream& bitstream, uint32_t precisionBytesMinus1  ) {
  bitstream.write( precisionBytesMinus1, 3 );  // u(3)
  bitstream.write( 0, 5 );                                                   // u(5)
}

// C.2.2 Sample stream NAL unit syntax
void PCCBitstreamEncoder::sampleStreamNalUnit( PCCContext& context, PCCBitstream& bitstream, SampleStreamNalUnit& ssnu, NalUnit& nalu, size_t index ) {
  bitstream.write( uint32_t( nalu.getNalUnitSize() ), 8 * ( ssnu.getUnitSizePrecisionBytesMinus1() + 1 ) );  // u(v)
  nalUnitHeader(bitstream, nalu);
  if( NaUnitType(nalu.getNalUnitType())==NAL_ASPS )
  {
    atlasSequenceParameterSetRBSP(context.getAtlasSequenceParameterSet(index), context, bitstream);
  }else if( NaUnitType(nalu.getNalUnitType())==NAL_AFPS )
  {
    atlasFrameParameterSetRbsp(context.getAtlasFrameParameterSet(index), context, bitstream);
  }else if( NaUnitType(nalu.getNalUnitType())==NAL_TRAIL||
            NaUnitType(nalu.getNalUnitType())==NAL_TSA ||
            NaUnitType(nalu.getNalUnitType())==NAL_STSA||
            NaUnitType(nalu.getNalUnitType())==NAL_RADL||
            NaUnitType(nalu.getNalUnitType())==NAL_RASL||
            NaUnitType(nalu.getNalUnitType())==NAL_SKIP){
    atlasTileGroupLayerRbsp(context.getAtlasTileGroupLayer(index), context, bitstream);
  }
    
//  nalUnit( bitstream, nalu );
//  printf("NalUnitSize      = %lu \n",nalu.getNalUnitSize()); fflush(stdout);
//  printf("NalUnitType      = %lu \n",nalu.getNalUnitType()); fflush(stdout);
//  printf("LayerId          = %lu \n",nalu.getLayerId());          fflush(stdout);
//  printf("TemporalyIdPlus1 = %lu \n",nalu.getTemporalyIdPlus1()); fflush(stdout);
}

// F.2.1 VUI parameters syntax
void PCCBitstreamEncoder::vuiParameters() {
  // vui_timing_info_present_flag //   u(1)
  // if( vui_timing_info_present_flag ) {
  //   vui_num_units_in_tick // u(32)
  //   vui_time_scale // u(32)
  //   vui_poc_proportional_to_timing_flag // u(1)
  //   if( vui_poc_proportional_to_timing_flag ) {
  //     vui_num_ticks_poc_diff_one_minus1 // ue(v)
  //   }
  //   vui_hrd_parameters_present_flag   // u(1)
  //   if( vui_hrd_parameters_present_flag ){
  //     hrdParameters()
  //   }
  // }
}
// F.2.2 HRD parameters syntax
void PCCBitstreamEncoder::hrdParameters() {
  // hrd_nal_parameters_present_flag // u(1)
  // hrd_acl_parameters_present_flag // u(1)
  // if( hrd_nal_parameters_present_flag | | hrd_acl_parameters_present_flag ){
  //   hrd_bit_rate_scale // u(4)
  //   hrd_cab_size_scale // u(4)
  //   hrd_initial_cab_removal_delay_length_minus1 // u(5)
  //   hrd_au_cab_removal_delay_length_minus1 // u(5)
  //   hrd_dab_output_delay_length_minus1  // u(5)
  // }
  // maxNumSubLayersMinus1 = 0
  // for( i = 0; i <= maxNumSubLayersMinus1; i++ ) {
  //   hrd_fixed_atlas_rate_general_flag[ i ] // u(1)
  // if( !hrd_fixed_atlas_rate_general_flag[ i ] )
  //   hrd_fixed_atlas_rate_within_cas_flag[ i ] // u(1)
  // if( hrd_fixed_atlas_rate_within_cas_flag[ i ] )
  //   hrd_elemental_duration_in_tc_minus1[ i ] // ue(v)
  // else
  //   hrd_low_delay_flag[ i ] // u(1)
  // if( !hrd_low_delay_flag[ i ] )
  //   hrd_cab_cnt_minus1[ i ] // ue(v)
  // if( hrd_nal_parameters_present_flag )
  //   hrd_sub_layer_parameters( 0, i )
  // if( hrd_acl_parameters_present_flag )
  //   hrd_sub_layer_parameters( 1, i )
  // }
}

// F.2.3 Sub-layer HRD parameters syntax
void PCCBitstreamEncoder::hrdSubLayerParameters() {
  // for( i = 0; i <= CabCnt; i++ ) {
  //  hrd_bit_rate_value_minus1[ type ][ i ] // ue(v)
  //  hrd_cab_size_value_minus1[ type ][ i ]  // ue(v)
  //  hrd_cbr_flag[ type ][ i ] // u(1)
  // }
}

// JR TODO: Remove

// 7.3.2.3 raw separate video data syntax TODO: remove this
void PCCBitstreamEncoder::pcmSeparateVideoData( PCCContext& context, PCCBitstream& bitstream, uint8_t bitCount ) {
  TRACE_BITSTREAM( "%s \n", __func__ );
  auto&  vpcc       = context.getVPCC();
  auto&  sps        = context.getSps();
  size_t atlasIndex = 0;
  if ( sps.getRawSeparateVideoPresentFlag( atlasIndex ) && !vpcc.getMapIndex() ) {
    bitstream.write( (uint32_t)vpcc.getRawVideoFlag(), 1 );  // u(1)
    bitstream.write( (uint32_t)0, bitCount );                // u(bitCount)
  } else {
    bitstream.write( (uint32_t)0, bitCount + 1 );  // u(bitCount + 1)
  }
}
