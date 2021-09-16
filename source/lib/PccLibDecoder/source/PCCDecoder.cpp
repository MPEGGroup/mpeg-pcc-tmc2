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
#include "PCCHighLevelSyntax.h"
#include "PCCBitstream.h"
#include "PCCVideoBitstream.h"
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCPatch.h"
#include "PCCVideoDecoder.h"
#include "PCCGroupOfFrames.h"
#include <tbb/tbb.h>
#include "PCCDecoder.h"

using namespace pcc;
using namespace std;

PCCDecoder::PCCDecoder() {
#ifdef ENABLE_PAPI_PROFILING
  initPapiProfiler();
#endif
}
PCCDecoder::~PCCDecoder() = default;
void PCCDecoder::setParameters( const PCCDecoderParameters& params ) { params_ = params; }
void PCCDecoder::setReconstructionParameters( const PCCDecoderParameters& params ) {
  params_.pixelDeinterleavingType_      = params.pixelDeinterleavingType_;
  params_.pointLocalReconstructionType_ = params.pointLocalReconstructionType_;
  params_.reconstructEomType_           = params.reconstructEomType_;
  params_.duplicatedPointRemovalType_   = params.duplicatedPointRemovalType_;
  params_.reconstructRawType_           = params.reconstructRawType_;
  params_.applyGeoSmoothingType_        = params.applyGeoSmoothingType_;
  params_.applyAttrSmoothingType_       = params.applyAttrSmoothingType_;
  params_.attrTransferFilterType_       = params.attrTransferFilterType_;
  params_.applyOccupanySynthesisType_   = params.applyOccupanySynthesisType_;
}

int PCCDecoder::decode( PCCContext& context, PCCGroupOfFrames& reconstructs, int32_t atlasIndex = 0 ) {
  if ( params_.nbThread_ > 0 ) { tbb::task_scheduler_init init( static_cast<int>( params_.nbThread_ ) ); }
  createPatchFrameDataStructure( context );

  PCCVideoDecoder videoDecoder;
  videoDecoder.setLogger( *logger_ );
  std::stringstream path;
  auto&             sps              = context.getVps();
  auto&             ai               = sps.getAttributeInformation( atlasIndex );
  auto&             oi               = sps.getOccupancyInformation( atlasIndex );
  auto&             gi               = sps.getGeometryInformation( atlasIndex );
  auto&             asps             = context.getAtlasSequenceParameterSet( 0 );
  size_t            frameCount       = context.size();
  auto&             plt              = sps.getProfileTierLevel();
  const size_t      mapCount         = sps.getMapCountMinus1( atlasIndex ) + 1;
  int               geometryBitDepth = gi.getGeometry2dBitdepthMinus1() + 1;
  setConsitantFourCCCode( context, 0 );  //
  auto occupancyCodecId = getCodedCodecId( context, oi.getOccupancyCodecId(), params_.videoDecoderOccupancyPath_ );
  auto geometryCodecId  = getCodedCodecId( context, gi.getGeometryCodecId(), params_.videoDecoderGeometryPath_ );
  path << removeFileExtension( params_.compressedStreamPath_ ) << "_dec_GOF" << sps.getV3CParameterSetId() << "_";

  printf( "CodecCodecId: ProfileCodecGroupIdc = %u occupancyCodecId = %u geometry = %u auxGeo = %u \n",
          plt.getProfileCodecGroupIdc(), oi.getOccupancyCodecId(), gi.getGeometryCodecId(),
          gi.getAuxiliaryGeometryCodecId() );
  printf( "=> Video decoder : occupancy = %d geometry = %d \n", (int)occupancyCodecId, (int)geometryCodecId );
  printf( " Decode 0 size = %zu \n", context.getVideoBitstream( VIDEO_OCCUPANCY ).size() );
  fflush( stdout );
  TRACE_PICTURE( "Occupancy\n" );
  TRACE_PICTURE( "MapIdx = 0, AuxiliaryVideoFlag = 0\n" );
  videoDecoder.decompress( context.getVideoOccupancyMap(),                // video
                           context,                                       // contexts
                           path.str(),                                    // path
                           context.getVideoBitstream( VIDEO_OCCUPANCY ),  // bitstream
                           params_.byteStreamVideoCoderOccupancy_,        // byte stream video coder
                           occupancyCodecId,                              // codecId
                           params_.videoDecoderOccupancyPath_,            // decoder path
                           8,                                             // output bit depth
                           params_.keepIntermediateFiles_ );              // keep intermediate files

  // converting the decoded bitdepth to the nominal bitdepth
  context.getVideoOccupancyMap().convertBitdepth( 8, oi.getOccupancy2DBitdepthMinus1() + 1,
                                                  oi.getOccupancyMSBAlignFlag() );

  if ( sps.getMultipleMapStreamsPresentFlag( atlasIndex ) ) {
    context.getVideoGeometryMultiple().resize( sps.getMapCountMinus1( atlasIndex ) + 1 );
    size_t totalGeoSize = 0;
    for ( uint32_t mapIndex = 0; mapIndex < sps.getMapCountMinus1( atlasIndex ) + 1; mapIndex++ ) {
      TRACE_PICTURE( "Geometry\n" );
      TRACE_PICTURE( "MapIdx = %d, AuxiliaryVideoFlag = 0\n", mapIndex );
      std::cout << "*******Video Decoding: Geometry[" << mapIndex << "] ********" << std::endl;
      auto  geometryIndex  = static_cast<PCCVideoType>( VIDEO_GEOMETRY_D0 + mapIndex );
      auto& videoBitstream = context.getVideoBitstream( geometryIndex );
      videoDecoder.decompress( context.getVideoGeometryMultiple( mapIndex ),  // video
                               context,                                       // contexts
                               path.str(),                                    // path
                               videoBitstream,                                // bitstream
                               params_.byteStreamVideoCoderGeometry_,         // byte stream video coder
                               geometryCodecId,                               // codecId
                               params_.videoDecoderGeometryPath_,             // decoder path
                               geometryBitDepth,                              // output bit depth
                               params_.keepIntermediateFiles_,                // keep intermediate files
                               0 );                                           // SHVC layer index

      context.getVideoGeometryMultiple()[mapIndex].convertBitdepth(
          geometryBitDepth, gi.getGeometry2dBitdepthMinus1() + 1, gi.getGeometryMSBAlignFlag() );
      std::cout << "geometry D" << mapIndex << " video ->" << videoBitstream.size() << " B" << std::endl;
      totalGeoSize += videoBitstream.size();
    }
    std::cout << "total geometry video ->" << totalGeoSize << " B" << std::endl;
  } else {
    TRACE_PICTURE( "Geometry\n" );
    TRACE_PICTURE( "MapIdx = 0, AuxiliaryVideoFlag = 0\n" );
    std::cout << "*******Video Decoding: Geometry ********" << std::endl;
    auto& videoBitstream = context.getVideoBitstream( VIDEO_GEOMETRY );

    printf( " Decode G size = %zu \n", videoBitstream.size() );
    fflush( stdout );
    videoDecoder.decompress( context.getVideoGeometryMultiple( 0 ),  // video
                             context,                                // contexts
                             path.str(),                             // path
                             videoBitstream,                         // bitstream
                             params_.byteStreamVideoCoderGeometry_,  // byte stream video coder
                             geometryCodecId,                        // codecId
                             params_.videoDecoderGeometryPath_,      // decoder path
                             geometryBitDepth,                       // output bit depth
                             params_.keepIntermediateFiles_,         // keep intermediate files
                             params_.shvcLayerIndex_ );              // SHVC layer index

    context.getVideoGeometryMultiple()[0].convertBitdepth( geometryBitDepth, gi.getGeometry2dBitdepthMinus1() + 1,
                                                           gi.getGeometryMSBAlignFlag() );
    std::cout << "geometry video ->" << videoBitstream.size() << " B" << std::endl;
  }

  if ( asps.getRawPatchEnabledFlag() && asps.getAuxiliaryVideoEnabledFlag() &&
       sps.getAuxiliaryVideoPresentFlag( atlasIndex ) ) {
    TRACE_PICTURE( "MapIdx = 0, AuxiliaryVideoFlag = 1\n" );
    std::cout << "*******Video Decoding: Aux Geometry ********" << std::endl;
    auto& videoBitstreamMP = context.getVideoBitstream( VIDEO_GEOMETRY_RAW );
    auto  auxGeometryCodecId =
        getCodedCodecId( context, gi.getAuxiliaryGeometryCodecId(), params_.videoDecoderGeometryPath_ );
    videoDecoder.decompress( context.getVideoRawPointsGeometry(),    // video
                             context,                                // contexts
                             path.str(),                             // path
                             videoBitstreamMP,                       // bitstream
                             params_.byteStreamVideoCoderGeometry_,  // byte stream video coder
                             auxGeometryCodecId,                     // codecId
                             params_.videoDecoderGeometryPath_,      // decoder path
                             geometryBitDepth,                       // output bit depth
                             params_.keepIntermediateFiles_,         // keep intermediate files
                             params_.shvcLayerIndex_ );              // SHVC layer index

    context.getVideoRawPointsGeometry().convertBitdepth( geometryBitDepth, gi.getGeometry2dBitdepthMinus1() + 1,
                                                         gi.getGeometryMSBAlignFlag() );
    std::cout << " raw points geometry -> " << videoBitstreamMP.size() << " B " << endl;
  }

  if ( ai.getAttributeCount() > 0 ) {
    for ( int attrIndex = 0; attrIndex < ai.getAttributeCount(); attrIndex++ ) {
      int  attributeBitDepth  = ai.getAttribute2dBitdepthMinus1( attrIndex ) + 1;
      int  attributeTypeId    = ai.getAttributeTypeId( attrIndex );
      int  attributeDimension = ai.getAttributeDimensionPartitionsMinus1( attrIndex ) + 1;
      auto attributeCodecId =
          getCodedCodecId( context, ai.getAttributeCodecId( attrIndex ), params_.videoDecoderAttributePath_ );
      printf( "CodecId attributeCodecId = %d \n", (int)attributeCodecId );
      for ( int attrPartitionIndex = 0; attrPartitionIndex < attributeDimension; attrPartitionIndex++ ) {
        if ( sps.getMultipleMapStreamsPresentFlag( atlasIndex ) ) {
          int sizeAttributeVideo = 0;
          context.getVideoAttributesMultiple().resize( sps.getMapCountMinus1( atlasIndex ) + 1 );
          // this allocation is considering only one attribute, with a single partition, but multiple streams
          for ( uint32_t mapIndex = 0; mapIndex < sps.getMapCountMinus1( atlasIndex ) + 1; mapIndex++ ) {
            // decompress T[mapIndex]
            TRACE_PICTURE( "Attribute\n" );
            TRACE_PICTURE( "AttrIdx = %d, AttrPartIdx = %d, AttrTypeID = %d, MapIdx = %d, AuxiliaryVideoFlag = 0\n",
                           attrIndex, attrPartitionIndex, attributeTypeId, mapIndex );
            std::cout << "*******Video Decoding: Attribute [" << mapIndex << "] ********" << std::endl;
            auto  attributeIndex = static_cast<PCCVideoType>( VIDEO_ATTRIBUTE_T0 + attrPartitionIndex +
                                                             MAX_NUM_ATTR_PARTITIONS * mapIndex );
            auto& videoBitstream = context.getVideoBitstream( attributeIndex );
            videoDecoder.decompress( context.getVideoAttributesMultiple( mapIndex ),  // video
                                     context,                                         // contexts
                                     path.str(),                                      // path
                                     videoBitstream,                                  // bitstream
                                     params_.byteStreamVideoCoderAttribute_,          // byte stream video coder
                                     attributeCodecId,                                // codecId
                                     params_.videoDecoderAttributePath_,              // decoder path
                                     attributeBitDepth,                               // output bit depth
                                     params_.keepIntermediateFiles_,                  // keep intermediate files
                                     params_.shvcLayerIndex_,                         // SHVC layer index
                                     params_.patchColorSubsampling_,                  // patch color subsampling
                                     params_.inverseColorSpaceConversionConfig_,      // inverse color space conversion
                                     params_.colorSpaceConversionPath_ );             // color space conversion path
            std::cout << "attribute T" << mapIndex << " video ->" << videoBitstream.size() << " B" << std::endl;
            sizeAttributeVideo += videoBitstream.size();
          }
          std::cout << "attribute    video ->" << sizeAttributeVideo << " B" << std::endl;
        } else {
          TRACE_PICTURE( "Attribute\n" );
          TRACE_PICTURE( "AttrIdx = 0, AttrPartIdx = %d, AttrTypeID = %d, MapIdx = 0, AuxiliaryVideoFlag = 0\n",
                         attrPartitionIndex, attributeTypeId );
          std::cout << "*******Video Decoding: Attribute ********" << std::endl;
          auto  attributeIndex = static_cast<PCCVideoType>( VIDEO_ATTRIBUTE + attrPartitionIndex );
          auto& videoBitstream = context.getVideoBitstream( attributeIndex );
          printf( " Decode T size = %zu \n", videoBitstream.size() );
          fflush( stdout );
          videoDecoder.decompress( context.getVideoAttributesMultiple( 0 ),     // video
                                   context,                                     // contexts
                                   path.str(),                                  // path
                                   videoBitstream,                              // bitstream
                                   params_.byteStreamVideoCoderAttribute_,      // byte stream video coder
                                   attributeCodecId,                            // codecId
                                   params_.videoDecoderAttributePath_,          // decoder path
                                   attributeBitDepth,                           // output bit depth
                                   params_.keepIntermediateFiles_,              // keep intermediate files
                                   params_.shvcLayerIndex_,                     // SHVC layer index
                                   params_.patchColorSubsampling_,              // patch color subsampling
                                   params_.inverseColorSpaceConversionConfig_,  // inverse color space conversionConfig
                                   params_.colorSpaceConversionPath_ );         // color space conversion path
          std::cout << "attribute video  ->" << videoBitstream.size() << " B" << std::endl;
        }

        if ( asps.getRawPatchEnabledFlag() && asps.getAuxiliaryVideoEnabledFlag() &&
             sps.getAuxiliaryVideoPresentFlag( atlasIndex ) ) {
          std::cout << "*******Video Decoding: Aux Attribute ********" << std::endl;
          auto attributeIndex = static_cast<PCCVideoType>( VIDEO_ATTRIBUTE_RAW + attrPartitionIndex );
          TRACE_PICTURE( "Attribute\n" );
          TRACE_PICTURE( "AttrIdx = 0, AttrPartIdx = %d, AttrTypeID = %d, MapIdx = 0, AuxiliaryVideoFlag = 1\n",
                         attrPartitionIndex, attributeTypeId );
          auto& videoBitstreamMP    = context.getVideoBitstream( attributeIndex );
          auto  auxAttributeCodecId = getCodedCodecId( context, ai.getAuxiliaryAttributeCodecId( attrIndex ),
                                                      params_.videoDecoderAttributePath_ );
          printf( "CodecId auxAttributeCodecId = %d \n", (int)auxAttributeCodecId );
          videoDecoder.decompress( context.getVideoRawPointsAttribute(),        // video
                                   context,                                     // contexts
                                   path.str(),                                  // path
                                   videoBitstreamMP,                            // bitstream
                                   params_.byteStreamVideoCoderAttribute_,      // byte stream video coder
                                   auxAttributeCodecId,                         // codecId
                                   params_.videoDecoderAttributePath_,          // decoder path
                                   attributeBitDepth,                           // output bit depth
                                   params_.keepIntermediateFiles_,              // keep intermediate files
                                   params_.shvcLayerIndex_,                     // SHVC layer index
                                   false,                                       // patch color subsampling
                                   params_.inverseColorSpaceConversionConfig_,  // inverse color space conversionConfig
                                   params_.colorSpaceConversionPath_ );         // color space conversion path
          // generateRawPointsAttributefromVideo( context, reconstructs );
          std::cout << " raw points attribute -> " << videoBitstreamMP.size() << " B" << endl;
        }
      }
    }
  }

  reconstructs.setFrameCount( frameCount );
  // recreating the prediction list per attribute (either the attribute is coded absolute, or follows the geometry)
  // see contribution m52529
  std::vector<std::vector<bool>> absoluteT1List;
  absoluteT1List.resize( ai.getAttributeCount() );
  for ( int attrIdx = 0; attrIdx < ai.getAttributeCount(); ++attrIdx ) {
    absoluteT1List[attrIdx].resize( sps.getMapCountMinus1( atlasIndex ) + 1 );
    if ( ai.getAttributeMapAbsoluteCodingPersistenceFlag( attrIdx ) != 0u ) {
      for ( uint32_t mapIdx = 0; mapIdx < sps.getMapCountMinus1( atlasIndex ) + 1; ++mapIdx ) {
        absoluteT1List[attrIdx][mapIdx] = true;
      }
    } else {
      // follow geometry
      for ( uint32_t mapIdx = 0; mapIdx < sps.getMapCountMinus1( atlasIndex ) + 1; ++mapIdx ) {
        absoluteT1List[attrIdx][mapIdx] = sps.getMapAbsoluteCodingEnableFlag( atlasIndex, mapIdx );
      }
    }
  }
  printf( "generate point cloud of %zu frames \n", frameCount );
  fflush( stdout );
  for ( size_t frameIdx = 0; frameIdx < frameCount; frameIdx++ ) {
    // All video have been decoded, start reconsctruction processes
    if ( asps.getRawPatchEnabledFlag() && asps.getAuxiliaryVideoEnabledFlag() &&
         sps.getAuxiliaryVideoPresentFlag( atlasIndex ) ) {
      for ( int attrIndex = 0; attrIndex < ai.getAttributeCount(); attrIndex++ ) {
        int attributeDimensionPartitions = ai.getAttributeDimensionPartitionsMinus1( attrIndex ) + 1;
        for ( int attrPartitionIndex = 0; attrPartitionIndex < attributeDimensionPartitions; attrPartitionIndex++ ) {
          printf( "generateRawPointsAttributefromVideo attrIndex = %d attrPartitionIndex = %d \n", attrIndex,
                  attrPartitionIndex );
          fflush( stdout );
          generateRawPointsAttributefromVideo( context, frameIdx );
        }
      }
    }  // getAuxiliaryVideoEnabledFlag()

    context.setOccupancyPrecision( sps.getFrameWidth( atlasIndex ) / context.getVideoOccupancyMap().getWidth() );
    GeneratePointCloudParameters gpcParams;
    GeneratePointCloudParameters ppSEIParams;

    auto&                 reconstruct = reconstructs[frameIdx];
    std::vector<uint32_t> partition;
    // Decode point cloud
    printf( "call generatePointCloud() \n" );
    std::vector<size_t> accTilePointCount;
    accTilePointCount.resize( ai.getAttributeCount(), 0 );
    for ( size_t tileIdx = 0; tileIdx < context[frameIdx].getNumTilesInAtlasFrame(); tileIdx++ ) {
      auto atglIndex = context.getAtlasHighLevelSyntax().getAtlasTileLayerIndex( frameIdx, tileIdx );
      setGeneratePointCloudParameters( gpcParams, context, atglIndex );
      setPostProcessingSeiParameters( ppSEIParams, context, atglIndex );
      // std::cout << "Processing frame " << frameIdx << " tile " << tileIdx << std::endl;
      auto& tile = context[frameIdx].getTile( tileIdx );
      if ( !ppSEIParams.pbfEnableFlag_ ) {
        generateOccupancyMap( tile, context.getVideoOccupancyMap().getFrame( tile.getFrameIndex() ),
                              context.getOccupancyPrecision(), oi.getLossyOccupancyCompressionThreshold(),
                              asps.getEomPatchEnabledFlag() );
      }
      if ( context[frameIdx].getNumTilesInAtlasFrame() > 1 ) {
        generateTileBlockToPatchFromOccupancyMapVideo(
            context, tile, frameIdx, context.getVideoOccupancyMap().getFrame( frameIdx ),
            size_t( 1 ) << asps.getLog2PatchPackingBlockSize(), context.getOccupancyPrecision() );

      } else {
        generateBlockToPatchFromOccupancyMapVideo(
            context, tile, frameIdx, context.getVideoOccupancyMap().getFrame( frameIdx ),
            size_t( 1 ) << asps.getLog2PatchPackingBlockSize(), context.getOccupancyPrecision() );
      }

      printf( "call generatePointCloud() \n" );
      PCCPointSet3 tileReconstrct;
      generatePointCloud( tileReconstrct, context, frameIdx, tileIdx, gpcParams, partition, true );
      reconstruct.appendPointSet( tileReconstrct );
      if ( context[frameIdx].getNumTilesInAtlasFrame() > 1 )
        context[frameIdx].getTitleFrameContext().appendPointToPixel(
            context[frameIdx].getTile( tileIdx ).getPointToPixel() );
      if ( ai.getAttributeCount() > 0 ) {
        reconstruct.addColors();
        reconstruct.addColors16bit();
        for ( size_t attIdx = 0; attIdx < ai.getAttributeCount(); attIdx++ ) {
          printf( "start colorPointCloud attIdx = %zu / %u ] \n", attIdx, ai.getAttributeCount() );
          fflush( stdout );
          size_t updatedPointCount  = colorPointCloud( reconstruct, context, tile, absoluteT1List[attIdx],
                                                      sps.getMultipleMapStreamsPresentFlag( atlasIndex ),
                                                      ai.getAttributeCount(), accTilePointCount[attIdx], gpcParams );
          accTilePointCount[attIdx] = updatedPointCount;
        }
      }
    }  // tile

#ifdef CONFORMANCE_TRACE
    size_t numProjPoints = 0, numRawPoints = 0, numEomPoints = 0;
    for ( size_t tileIdx = 0; tileIdx < context[frameIdx].getNumTilesInAtlasFrame(); tileIdx++ ) {
      auto& tile = context[frameIdx].getTile( tileIdx );
      numProjPoints += tile.getTotalNumberOfRegularPoints();
      numEomPoints += tile.getTotalNumberOfEOMPoints();
      numRawPoints += tile.getTotalNumberOfRawPoints();
    }  // tile
    if ( ai.getAttributeCount() == 0 ) {
      reconstructs[frameIdx].removeColors();
      reconstructs[frameIdx].removeColors16bit();
    } else {
      bool isAttributes444 = context.getVideoAttributesMultiple( 0 ).getColorFormat() == PCCCOLORFORMAT::RGB444;
      if ( !isAttributes444 ) {  // lossy: convert 16-bit yuv444 to 8-bit RGB444
        reconstructs[frameIdx].convertYUV16ToRGB8();
      } else {
        reconstructs[frameIdx].copyRGB16ToRGB8();
      }
    }
    TRACE_PCFRAME( "AtlasFrameIndex = %d\n", frameIdx );
    TRACE_PCFRAME( "PointCloudFrameOrderCntVal = %d, NumProjPoints = %zu, NumRawPoints = %zu, NumEomPoints = %zu,",
                   frameIdx, numProjPoints, numRawPoints, numEomPoints );
    auto checksumFrame = reconstructs[frameIdx].computeChecksum( true );
    TRACE_PCFRAME( " MD5 checksum = " );
    for ( auto& c : checksumFrame ) { TRACE_PCFRAME( "%02x", c ); }
    TRACE_PCFRAME( "\n" );
#endif

    // Post-Processing
    TRACE_PATCH( "Post-Processing: postprocessSmoothing = %zu pbfEnableFlag = %d \n", params_.attrTransferFilterType_,
                 ppSEIParams.pbfEnableFlag_ );
    if ( params_.applyGeoSmoothingType_ != 0 && ppSEIParams.flagGeometrySmoothing_ ) {
      PCCPointSet3 tempFrameBuffer = reconstruct;
      if ( ppSEIParams.gridSmoothing_ ) {
        smoothPointCloudPostprocess( reconstruct, params_.colorTransform_, ppSEIParams, partition );
      }
      if ( ai.getAttributeCount() > 0 ) {
        bool isAttributes444 = context.getVideoAttributesMultiple( 0 ).getColorFormat() == PCCCOLORFORMAT::RGB444;
        printf( "isAttributes444 = %d Format = %d \n", isAttributes444,
                context.getVideoAttributesMultiple( 0 ).getColorFormat() );
        fflush( stdout );

        if ( !ppSEIParams.pbfEnableFlag_ ) {
          // These are different attribute transfer functions
          if ( params_.attrTransferFilterType_ == 1 || params_.attrTransferFilterType_ == 5 ) {
            TRACE_PATCH( " transferColors16bitBP \n" );
            tempFrameBuffer.transferColors16bitBP( reconstruct,                      // target
                                                   params_.attrTransferFilterType_,  // filterType
                                                   int32_t( 0 ),                     // searchRange
                                                   isAttributes444,                  // losslessAttribute
                                                   8,                                // numNeighborsColorTransferFwd
                                                   1,                                // numNeighborsColorTransferBwd
                                                   true,                             // useDistWeightedAverageFwd
                                                   true,                             // useDistWeightedAverageBwd
                                                   true,        // skipAvgIfIdenticalSourcePointPresentFwd
                                                   false,       // skipAvgIfIdenticalSourcePointPresentBwd
                                                   4,           // distOffsetFwd
                                                   4,           // distOffsetBwd
                                                   1000,        // maxGeometryDist2Fwd
                                                   1000,        // maxGeometryDist2Bwd
                                                   1000 * 256,  // maxColorDist2Fwd
                                                   1000 * 256   // maxColorDist2Bwd
            );
          } else if ( params_.attrTransferFilterType_ == 2 ) {
            TRACE_PATCH( " transferColorWeight \n" );
            tempFrameBuffer.transferColorWeight( reconstruct, 0.1 );
          } else if ( params_.attrTransferFilterType_ == 3 ) {
            TRACE_PATCH( " transferColorsFilter3 \n" );
            tempFrameBuffer.transferColorsFilter3( reconstruct, int32_t( 0 ), isAttributes444 );
          } else if ( params_.attrTransferFilterType_ == 7 || params_.attrTransferFilterType_ == 9 ) {
            TRACE_PATCH( " transferColorsFilter3 \n" );
            tempFrameBuffer.transferColorsBackward16bitBP( reconstruct,                      //  target
                                                           params_.attrTransferFilterType_,  //  filterType
                                                           int32_t( 0 ),                     //  searchRange
                                                           isAttributes444,                  //  losslessAttribute
                                                           8,           //  numNeighborsColorTransferFwd
                                                           1,           //  numNeighborsColorTransferBwd
                                                           true,        //  useDistWeightedAverageFwd
                                                           true,        //  useDistWeightedAverageBwd
                                                           true,        //  skipAvgIfIdenticalSourcePointPresentFwd
                                                           false,       //  skipAvgIfIdenticalSourcePointPresentBwd
                                                           4,           //  distOffsetFwd
                                                           4,           //  distOffsetBwd
                                                           1000,        //  maxGeometryDist2Fwd
                                                           1000,        //  maxGeometryDist2Bwd
                                                           1000 * 256,  //  maxColorDist2Fwd
                                                           1000 * 256   //  maxColorDist2Bwd
            );
          }
        }
      }  // if ( ai.getAttributeCount() > 0 )
    }
    if ( ai.getAttributeCount() > 0 ) {
      if ( params_.applyAttrSmoothingType_ != 0 && ppSEIParams.flagColorSmoothing_ ) {
        TRACE_PATCH( " colorSmoothing \n" );
        colorSmoothing( reconstruct, params_.colorTransform_, ppSEIParams );
      }
      if ( context.getVideoAttributesMultiple( 0 ).getColorFormat() !=
           PCCCOLORFORMAT::RGB444 ) {  // lossy: convert 16-bit yuv444 to 8-bit RGB444
        TRACE_PATCH( "lossy: convert 16-bit yuv444 to 8-bit RGB444 (convertYUV16ToRGB8) \n" );
        reconstruct.convertYUV16ToRGB8();
      } else {  // lossless: copy 16-bit RGB to 8-bit RGB
        TRACE_PATCH( "lossy: lossless: copy 16-bit RGB to 8-bit RGB (copyRGB16ToRGB8) \n" );
        reconstruct.copyRGB16ToRGB8();
      }
    }
    /*auto tmp = reconstruct.computeChecksum();
    TRACE_PCFRAME( " MD5 checksum = " );
    for ( auto& c : tmp ) { TRACE_PCFRAME( "%02x", c ); }
    TRACE_PCFRAME( "\n" );*/
    TRACE_RECFRAME( "AtlasFrameIndex = %d\n", frameIdx );
    auto checksum = reconstructs[frameIdx].computeChecksum( true );
    TRACE_RECFRAME( " MD5 checksum = " );
    for ( auto& c : checksum ) { TRACE_RECFRAME( "%02x", c ); }
    TRACE_RECFRAME( "\n" );
  }
  return 0;
}

void PCCDecoder::setPointLocalReconstruction( PCCContext& context ) {
  auto& asps = context.getAtlasSequenceParameterSet( 0 );
  TRACE_PATCH( "PLR = %d \n", asps.getPLREnabledFlag() );
  PointLocalReconstructionMode mode = {false, false, 0, 1};
  context.addPointLocalReconstructionMode( mode );
  if ( asps.getPLREnabledFlag() ) {
    auto& plri = asps.getPLRInformation( 0 );
    for ( size_t i = 0; i < plri.getNumberOfModesMinus1(); i++ ) {
      mode.interpolate_ = plri.getInterpolateFlag( i );
      mode.filling_     = plri.getFillingFlag( i );
      mode.minD1_       = plri.getMinimumDepth( i );
      mode.neighbor_    = plri.getNeighbourMinus1( i ) + 1;
      context.addPointLocalReconstructionMode( mode );
    }
#ifdef CODEC_TRACE
    for ( size_t i = 0; i < context.getPointLocalReconstructionModeNumber(); i++ ) {
      auto& mode = context.getPointLocalReconstructionMode( i );
      TRACE_PATCH( "Plrm[%zu]: Inter = %d Fill = %d minD1 = %u neighbor = %u \n", i, mode.interpolate_, mode.filling_,
                   mode.minD1_, mode.neighbor_ );
    }
#endif
  }
}

void PCCDecoder::setPLRData( PCCFrameContext& tile, PCCPatch& patch, PLRData& plrd, size_t occupancyPackingBlockSize ) {
  patch.allocOneLayerData();
  TRACE_PATCH( "WxH = %zu x %zu \n", plrd.getBlockToPatchMapWidth(), plrd.getBlockToPatchMapHeight() );
  patch.getPointLocalReconstructionLevel() = static_cast<uint8_t>( plrd.getLevelFlag() );
  TRACE_PATCH( "  LevelFlag = %d \n", plrd.getLevelFlag() );
  if ( plrd.getLevelFlag() ) {
    if ( plrd.getPresentFlag() ) {
      patch.setPointLocalReconstructionMode( plrd.getModeMinus1() + 1 );
    } else {
      patch.setPointLocalReconstructionMode( 0 );
    }
    TRACE_PATCH( "  ModePatch: Present = %d ModeMinus1 = %2d \n", plrd.getPresentFlag(),
                 plrd.getPresentFlag() ? (int32_t)plrd.getModeMinus1() : -1 );
  } else {
    for ( size_t v0 = 0; v0 < plrd.getBlockToPatchMapHeight(); ++v0 ) {
      for ( size_t u0 = 0; u0 < plrd.getBlockToPatchMapWidth(); ++u0 ) {
        size_t index = v0 * plrd.getBlockToPatchMapWidth() + u0;
        if ( plrd.getBlockPresentFlag( index ) ) {
          patch.setPointLocalReconstructionMode( u0, v0, plrd.getBlockModeMinus1( index ) + 1 );
        } else {
          patch.setPointLocalReconstructionMode( u0, v0, 0 );
        }
        TRACE_PATCH( "  Mode[%3u]: Present = %d ModeMinus1 = %2d \n", index, plrd.getBlockPresentFlag( index ),
                     plrd.getBlockPresentFlag( index ) ? (int32_t)plrd.getBlockModeMinus1( index ) : -1 );
      }
    }
  }
#ifdef CODEC_TRACE
  for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
    for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
      TRACE_PATCH(
          "Block[ %2lu %2lu <=> %4zu ] / [ %2lu %2lu ]: Level = %d Present = "
          "%d mode = %zu \n",
          u0, v0, v0 * patch.getSizeU0() + u0, patch.getSizeU0(), patch.getSizeV0(),
          patch.getPointLocalReconstructionLevel(), plrd.getBlockPresentFlag( v0 * patch.getSizeU0() + u0 ),
          patch.getPointLocalReconstructionMode( u0, v0 ) );
    }
  }
#endif
}

void PCCDecoder::setPostProcessingSeiParameters( GeneratePointCloudParameters& params,
                                                 PCCContext&                   context,
                                                 size_t                        atglIndex ) {
  auto&   sps                   = context.getVps();
  int32_t atlasIndex            = 0;
  auto&   oi                    = sps.getOccupancyInformation( atlasIndex );
  auto&   gi                    = sps.getGeometryInformation( atlasIndex );
  auto&   asps                  = context.getAtlasSequenceParameterSet( 0 );
  auto&   plt                   = sps.getProfileTierLevel();
  params.flagGeometrySmoothing_ = false;
  params.gridSmoothing_         = false;
  params.gridSize_              = 0;
  params.thresholdSmoothing_    = 0;
  params.pbfEnableFlag_         = false;
  params.pbfPassesCount_        = 0;
  params.pbfFilterSize_         = 0;
  params.pbfLog2Threshold_      = 0;
  if ( params_.applyGeoSmoothingType_ != 0 &&
       context.seiIsPresentInReceivedData( NAL_PREFIX_ESEI, GEOMETRY_SMOOTHING, atglIndex ) ) {
    auto* sei = static_cast<SEIGeometrySmoothing*>(
        context.getSeiInReceivedData( NAL_PREFIX_ESEI, GEOMETRY_SMOOTHING, atglIndex ) );
    for ( size_t i = 0; i < sei->getInstancesUpdated(); i++ ) {
      size_t k = sei->getInstanceIndex( i );
      if ( !sei->getInstanceCancelFlag( k ) ) {
        params.flagGeometrySmoothing_ = true;
        if ( sei->getMethodType( k ) == 1 ) {
          params.gridSmoothing_      = true;
          params.gridSize_           = sei->getGridSizeMinus2( k ) + 2;
          params.thresholdSmoothing_ = static_cast<double>( sei->getThreshold( k ) );
        }
      }
    }
  }
  if ( params_.applyOccupanySynthesisType_ != 0 &&
       context.seiIsPresentInReceivedData( NAL_PREFIX_ESEI, OCCUPANCY_SYNTHESIS, atglIndex ) ) {
    auto* sei = static_cast<SEIOccupancySynthesis*>(
        context.getSeiInReceivedData( NAL_PREFIX_ESEI, OCCUPANCY_SYNTHESIS, atglIndex ) );
    for ( size_t i = 0; i < sei->getInstancesUpdated(); i++ ) {
      size_t k = sei->getInstanceIndex( i );
      if ( !sei->getInstanceCancelFlag( k ) ) {
        params.flagGeometrySmoothing_ = true;
        if ( sei->getMethodType( k ) == 1 ) {
          params.pbfEnableFlag_    = true;
          params.pbfPassesCount_   = sei->getPbfPassesCountMinus1( k ) + 1;
          params.pbfFilterSize_    = sei->getPbfFilterSizeMinus1( k ) + 1;
          params.pbfLog2Threshold_ = sei->getPbfLog2ThresholdMinus1( k ) + 1;
        }
      }
    }
  }
  params.occupancyResolution_    = size_t( 1 ) << asps.getLog2PatchPackingBlockSize();
  params.occupancyPrecision_     = context.getOccupancyPrecision();
  params.enableSizeQuantization_ = context.getAtlasSequenceParameterSet( 0 ).getPatchSizeQuantizerPresentFlag();
  params.rawPointColorFormat_ =
      size_t( plt.getProfileCodecGroupIdc() == CODEC_GROUP_HEVC444 ? COLOURFORMAT444 : COLOURFORMAT420 );
  params.nbThread_   = params_.nbThread_;
  params.absoluteD1_ = sps.getMapCountMinus1( atlasIndex ) == 0 || sps.getMapAbsoluteCodingEnableFlag( atlasIndex, 1 );
  params.multipleStreams_          = sps.getMultipleMapStreamsPresentFlag( atlasIndex );
  params.surfaceThickness_         = asps.getAspsVpccExtension().getSurfaceThicknessMinus1() + 1;
  params.thresholdColorSmoothing_  = 0.;
  params.flagColorSmoothing_       = false;
  params.cgridSize_                = 0;
  params.thresholdColorDifference_ = 0;
  params.thresholdColorVariation_  = 0;
  if ( params_.applyAttrSmoothingType_ != 0 &&
       context.seiIsPresentInReceivedData( NAL_PREFIX_ESEI, ATTRIBUTE_SMOOTHING, atglIndex ) ) {
    auto* sei = static_cast<SEIAttributeSmoothing*>(
        context.getSeiInReceivedData( NAL_PREFIX_ESEI, ATTRIBUTE_SMOOTHING, atglIndex ) );
    for ( size_t j = 0; j < sei->getNumAttributesUpdated(); j++ ) {
      size_t k = sei->getAttributeIdx( j );
      if ( !sei->getAttributeSmoothingCancelFlag( k ) ) {
        for ( size_t i = 0; i < sei->getInstancesUpdated( k ); i++ ) {
          size_t m = sei->getInstanceIndex( k, i );
          if ( !sei->getInstanceCancelFlag( k, m ) ) {
            params.flagColorSmoothing_       = true;
            params.cgridSize_                = sei->getGridSizeMinus2( k, m ) + 2;
            params.thresholdColorSmoothing_  = static_cast<double>( sei->getThreshold( k, m ) );
            params.thresholdColorDifference_ = sei->getThresholdDifference( k, m );
            params.thresholdColorVariation_  = sei->getThresholdVariation( k, m );
          }
        }
      }
    }
  }
  params.thresholdLossyOM_ = static_cast<size_t>( oi.getLossyOccupancyCompressionThreshold() );
  params.removeDuplicatePoints_ =
      params_.duplicatedPointRemovalType_ != 0 && asps.getAspsVpccExtension().getRemoveDuplicatePointEnableFlag();
  params.pointLocalReconstruction_      = params_.pointLocalReconstructionType_ != 0 && asps.getPLREnabledFlag();
  params.mapCountMinus1_                = sps.getMapCountMinus1( atlasIndex );
  params.singleMapPixelInterleaving_    = params_.pixelDeinterleavingType_ != 0 && asps.getPixelDeinterleavingFlag();
  params.useAdditionalPointsPatch_      = params_.reconstructRawType_ != 0 && asps.getRawPatchEnabledFlag();
  params.enhancedOccupancyMapCode_      = params_.reconstructEomType_ != 0 && asps.getEomPatchEnabledFlag();
  params.EOMFixBitCount_                = asps.getEomFixBitCountMinus1() + 1;
  params.geometry3dCoordinatesBitdepth_ = gi.getGeometry3dCoordinatesBitdepthMinus1() + 1;
  params.geometryBitDepth3D_            = gi.getGeometry3dCoordinatesBitdepthMinus1() + 1;
}

void PCCDecoder::setGeneratePointCloudParameters( GeneratePointCloudParameters& params,
                                                  PCCContext&                   context,
                                                  size_t                        atglIndex ) {
  auto&   sps                   = context.getVps();
  int32_t atlasIndex            = 0;
  auto&   oi                    = sps.getOccupancyInformation( atlasIndex );
  auto&   gi                    = sps.getGeometryInformation( atlasIndex );
  auto&   asps                  = context.getAtlasSequenceParameterSet( 0 );
  auto&   plt                   = sps.getProfileTierLevel();
  params.flagGeometrySmoothing_ = false;
  params.gridSmoothing_         = false;
  params.gridSize_              = 0;
  params.thresholdSmoothing_    = 0;
  params.pbfEnableFlag_         = false;
  params.pbfPassesCount_        = 0;
  params.pbfFilterSize_         = 0;
  params.pbfLog2Threshold_      = 0;
  if ( params_.applyGeoSmoothingType_ != 0 &&
       context.seiIsPresentInReceivedData( NAL_PREFIX_ESEI, GEOMETRY_SMOOTHING, atglIndex ) ) {
    auto* sei = static_cast<SEIGeometrySmoothing*>(
        context.getSeiInReceivedData( NAL_PREFIX_ESEI, GEOMETRY_SMOOTHING, atglIndex ) );
    for ( size_t i = 0; i < sei->getInstancesUpdated(); i++ ) {
      size_t k = sei->getInstanceIndex( i );
      if ( !sei->getInstanceCancelFlag( k ) ) {
        params.flagGeometrySmoothing_ = true;
        if ( sei->getMethodType( k ) == 1 ) {
          params.gridSmoothing_      = true;
          params.gridSize_           = sei->getGridSizeMinus2( k ) + 2;
          params.thresholdSmoothing_ = static_cast<double>( sei->getThreshold( k ) );
        }
      }
    }
  }
  if ( params_.applyOccupanySynthesisType_ != 0 &&
       context.seiIsPresentInReceivedData( NAL_PREFIX_ESEI, OCCUPANCY_SYNTHESIS, atglIndex ) ) {
    auto* sei = static_cast<SEIOccupancySynthesis*>(
        context.getSeiInReceivedData( NAL_PREFIX_ESEI, OCCUPANCY_SYNTHESIS, atglIndex ) );
    for ( size_t i = 0; i < sei->getInstancesUpdated(); i++ ) {
      size_t k = sei->getInstanceIndex( i );
      if ( !sei->getInstanceCancelFlag( k ) ) {
        params.flagGeometrySmoothing_ = true;
        if ( sei->getMethodType( k ) == 1 ) {
          params.pbfEnableFlag_    = true;
          params.pbfPassesCount_   = sei->getPbfPassesCountMinus1( k ) + 1;
          params.pbfFilterSize_    = sei->getPbfFilterSizeMinus1( k ) + 1;
          params.pbfLog2Threshold_ = sei->getPbfLog2ThresholdMinus1( k ) + 1;
        }
      }
    }
  }
  params.occupancyResolution_    = size_t( 1 ) << asps.getLog2PatchPackingBlockSize();
  params.occupancyPrecision_     = context.getOccupancyPrecision();
  params.enableSizeQuantization_ = context.getAtlasSequenceParameterSet( 0 ).getPatchSizeQuantizerPresentFlag();
  params.rawPointColorFormat_ =
      size_t( plt.getProfileCodecGroupIdc() == CODEC_GROUP_HEVC444 ? COLOURFORMAT444 : COLOURFORMAT420 );
  params.nbThread_   = params_.nbThread_;
  params.absoluteD1_ = sps.getMapCountMinus1( atlasIndex ) == 0 || sps.getMapAbsoluteCodingEnableFlag( atlasIndex, 1 );
  params.multipleStreams_          = sps.getMultipleMapStreamsPresentFlag( atlasIndex );
  params.surfaceThickness_         = asps.getAspsVpccExtension().getSurfaceThicknessMinus1() + 1;
  params.flagColorSmoothing_       = false;
  params.cgridSize_                = 0;
  params.thresholdColorSmoothing_  = 0.;
  params.thresholdColorDifference_ = 0;
  params.thresholdColorVariation_  = 0;
  if ( params_.applyAttrSmoothingType_ != 0 &&
       context.seiIsPresentInReceivedData( NAL_PREFIX_ESEI, ATTRIBUTE_SMOOTHING, atglIndex ) ) {
    auto* sei = static_cast<SEIAttributeSmoothing*>(
        context.getSeiInReceivedData( NAL_PREFIX_ESEI, ATTRIBUTE_SMOOTHING, atglIndex ) );
    for ( size_t j = 0; j < sei->getNumAttributesUpdated(); j++ ) {
      size_t k = sei->getAttributeIdx( j );
      if ( !sei->getAttributeSmoothingCancelFlag( k ) ) {
        for ( size_t i = 0; i < sei->getInstancesUpdated( k ) + 1; i++ ) {
          size_t m = sei->getInstanceIndex( k, i );
          if ( !sei->getInstanceCancelFlag( k, m ) ) {
            if ( sei->getMethodType( k, m ) == 1 ) {
              params.flagColorSmoothing_       = true;
              params.cgridSize_                = sei->getGridSizeMinus2( k, m ) + 2;
              params.thresholdColorSmoothing_  = static_cast<double>( sei->getThreshold( k, m ) );
              params.thresholdColorDifference_ = sei->getThresholdDifference( k, m );
              params.thresholdColorVariation_  = sei->getThresholdVariation( k, m );
            }
          }
        }
      }
    }
  }
  params.thresholdLossyOM_ = static_cast<size_t>( oi.getLossyOccupancyCompressionThreshold() );
  params.removeDuplicatePoints_ =
      params_.duplicatedPointRemovalType_ != 0 && asps.getAspsVpccExtension().getRemoveDuplicatePointEnableFlag();
  params.pointLocalReconstruction_      = params_.pointLocalReconstructionType_ != 0 && asps.getPLREnabledFlag();
  params.mapCountMinus1_                = sps.getMapCountMinus1( atlasIndex );
  params.singleMapPixelInterleaving_    = params_.pixelDeinterleavingType_ != 0 && asps.getPixelDeinterleavingFlag();
  params.useAdditionalPointsPatch_      = params_.reconstructRawType_ != 0 && asps.getRawPatchEnabledFlag();
  params.useAuxSeperateVideo_           = asps.getAuxiliaryVideoEnabledFlag();
  params.enhancedOccupancyMapCode_      = params_.reconstructEomType_ != 0 && asps.getEomPatchEnabledFlag();
  params.EOMFixBitCount_                = asps.getEomFixBitCountMinus1() + 1;
  params.geometry3dCoordinatesBitdepth_ = gi.getGeometry3dCoordinatesBitdepthMinus1() + 1;
  params.geometryBitDepth3D_            = gi.getGeometry3dCoordinatesBitdepthMinus1() + 1;
}

void PCCDecoder::createPatchFrameDataStructure( PCCContext& context ) {
  TRACE_PATCH( "createPatchFrameDataStructure GOP start \n" );
  size_t frameCount = 0;
  auto&  atlList    = context.getAtlasTileLayerList();

  // partition information derivation
  setTilePartitionSizeAfti( context );
  for ( size_t i = 0; i < atlList.size(); i++ ) {
    size_t afocVal = context.calculateAFOCval( atlList, i );
    frameCount     = std::max( frameCount, ( afocVal + 1 ) );
    atlList[i].getHeader().setFrameIndex( afocVal );
  }
  context.resize( frameCount );
  setPointLocalReconstruction( context );
  for ( size_t atglIndex = 0; atglIndex < atlList.size(); atglIndex++ ) {
    auto& atgl = atlList[atglIndex];
    if ( atglIndex == 0 || atgl.getAtlasFrmOrderCntVal() != atlList[atglIndex - 1].getAtlasFrmOrderCntVal() ) {
      setTileSizeAndLocation( context, atgl.getHeader().getFrameIndex(), atgl.getHeader() );
    }
    auto&  atlu       = context.getAtlasTileLayer( atglIndex );
    auto&  ath        = atlu.getHeader();
    size_t frameIndex = ath.getFrameIndex();
    createPatchFrameDataStructure( context, atglIndex );

#ifdef CONFORMANCE_TRACE
    if ( atgl.getSEI().seiIsPresent( NAL_PREFIX_ESEI, GEOMETRY_SMOOTHING ) ) {
      auto* sei = static_cast<SEIGeometrySmoothing*>( atgl.getSEI().getSei( NAL_PREFIX_ESEI, GEOMETRY_SMOOTHING ) );
      auto& vec = sei->getMD5ByteStrData();
      if ( vec.size() > 0 ) {
        TRACE_HLS( "**********GEOMETRY_SMOOTHING_ESEI***********\n" );
        TRACE_HLS( "SEI%02dMD5 = ", sei->getPayloadType() );
        SEIMd5Checksum( context, vec );
      }
    }
    if ( atgl.getSEI().seiIsPresent( NAL_PREFIX_ESEI, OCCUPANCY_SYNTHESIS ) ) {
      auto* sei = static_cast<SEIOccupancySynthesis*>( atgl.getSEI().getSei( NAL_PREFIX_ESEI, OCCUPANCY_SYNTHESIS ) );
      auto& vec = sei->getMD5ByteStrData();
      if ( vec.size() > 0 ) {
        TRACE_HLS( "**********OCCUPANCY_SYNTHESIS_ESEI***********\n" );
        TRACE_HLS( "SEI%02dMD5 = ", sei->getPayloadType() );
        SEIMd5Checksum( context, vec );
      }
    }
    if ( atgl.getSEI().seiIsPresent( NAL_PREFIX_ESEI, ATTRIBUTE_SMOOTHING ) ) {
      auto* sei = static_cast<SEIAttributeSmoothing*>( atgl.getSEI().getSei( NAL_PREFIX_ESEI, ATTRIBUTE_SMOOTHING ) );
      auto& vec = sei->getMD5ByteStrData();
      if ( vec.size() > 0 ) {
        TRACE_HLS( "**********ATTRIBUTE_SMOOTHING_ESEI***********\n" );
        TRACE_HLS( "SEI%02dMD5 = ", sei->getPayloadType() );
        SEIMd5Checksum( context, vec );
      }
    }
    if ( atgl.getSEI().seiIsPresent( NAL_PREFIX_ESEI, COMPONENT_CODEC_MAPPING ) ) {
      auto* sei =
          static_cast<SEIOccupancySynthesis*>( atgl.getSEI().getSei( NAL_PREFIX_ESEI, COMPONENT_CODEC_MAPPING ) );
      auto& temp = sei->getMD5ByteStrData();
      if ( temp.size() > 0 ) {
        TRACE_HLS( "**********CODEC_COMPONENT_MAPPING_ESEI***********\n" );
        TRACE_HLS( "SEI%02dMD5 = ", sei->getPayloadType() );
        SEIMd5Checksum( context, temp );
      }
    }
#endif
    bool isLastTileOfTheFrames = atglIndex + 1 == atlList.size() ||
                                 atgl.getAtlasFrmOrderCntVal() != atlList[atglIndex + 1].getAtlasFrmOrderCntVal();
    if ( isLastTileOfTheFrames && atgl.getSEI().seiIsPresent( NAL_SUFFIX_NSEI, DECODED_ATLAS_INFORMATION_HASH ) ) {
      auto* sei = static_cast<SEIDecodedAtlasInformationHash*>(
          atgl.getSEI().getSei( NAL_SUFFIX_NSEI, DECODED_ATLAS_INFORMATION_HASH ) );
      TRACE_PATCH( "create Hash SEI \n" );
      auto& atlu = context.getAtlasTileLayer( atglIndex );
      auto& ath  = atlu.getHeader();
      createHashSEI( context, ath.getFrameIndex(), *sei );
    }
#ifdef CONFORMANCE_TRACE
    if ( isLastTileOfTheFrames ) { createHlsAtlasTileLogFiles( context, frameIndex ); }
#endif
  }
}

void PCCDecoder::createPatchFrameDataStructure( PCCContext& context, size_t atglIndex ) {
  TRACE_PATCH( "createPatchFrameDataStructure Tile %zu \n", atglIndex );
  auto&  sps                = context.getVps();
  size_t atlasIndex         = context.getAtlasIndex();
  auto&  atlu               = context.getAtlasTileLayer( atglIndex );
  auto&  ath                = atlu.getHeader();
  auto&  afps               = context.getAtlasFrameParameterSet( ath.getAtlasFrameParameterSetId() );
  auto&  asps               = context.getAtlasSequenceParameterSet( afps.getAtlasSequenceParameterSetId() );
  auto&  afti               = afps.getAtlasFrameTileInformation();
  auto&  atgdu              = atlu.getDataUnit();
  auto   geometryBitDepth2D = asps.getGeometry2dBitdepthMinus1() + 1;
  auto   geometryBitDepth3D = asps.getGeometry3dBitdepthMinus1() + 1;
  size_t frameIndex         = ath.getFrameIndex();
  size_t tileIndex          = afti.getSignalledTileIdFlag() ? afti.getTileId( ath.getId() ) : ath.getId();

  printf( "createPatchFrameDataStructure Frame = %zu Tiles = %zu atlasIndex = %zu atglIndex %zu \n", frameIndex,
          tileIndex, context.getAtlasIndex(), atglIndex );
  fflush( stdout );
  PCCFrameContext& tile = context[frameIndex].getTile( tileIndex );
  tile.setFrameIndex( frameIndex );
  tile.setAtlasFrmOrderCntVal( atlu.getAtlasFrmOrderCntVal() );
  tile.setAtlasFrmOrderCntMsb( atlu.getAtlasFrmOrderCntMsb() );
  tile.setTileIndex( tileIndex );
  tile.setAtlIndex( atglIndex );
  tile.setUseRawPointsSeparateVideo( sps.getAuxiliaryVideoPresentFlag( atlasIndex ) &&
                                     asps.getAuxiliaryVideoEnabledFlag() );
  tile.setRawPatchEnabledFlag( asps.getRawPatchEnabledFlag() );
  if ( tile.getFrameIndex() > 0 && ath.getType() != I_TILE ) {
    tile.setRefAfocList( context, ath, ath.getAtlasFrameParameterSetId() );
    TRACE_PATCH( "\tframe[%zu]\tRefAfocList:", frameIndex );
    for ( size_t i = 0; i < tile.getRefAfocListSize(); i++ ) { TRACE_PATCH( "\t%zu", tile.getRefAfoc( i ) ); }
    TRACE_PATCH( "\n" );
  }

  // local variable initialization
  auto&        patches        = tile.getPatches();
  auto&        rawPatches     = tile.getRawPointsPatches();
  auto&        eomPatches     = tile.getEomPatches();
  int64_t      predIndex      = 0;
  const size_t minLevel       = pow( 2., ath.getPosMinDQuantizer() );
  size_t       numRawPatches  = 0;
  size_t       numNonRawPatch = 0;
  size_t       numEomPatch    = 0;
  PCCTileType  tileType       = ath.getType();
  size_t       patchCount     = atgdu.getPatchCount();
  for ( size_t i = 0; i < patchCount; i++ ) {
    PCCPatchType currPatchType = getPatchType( tileType, atgdu.getPatchMode( i ) );
    if ( currPatchType == RAW_PATCH ) {
      numRawPatches++;
    } else if ( currPatchType == EOM_PATCH ) {
      numEomPatch++;
    }
  }
  TRACE_PATCH( "Patches size                      = %zu \n", patches.size() );
  TRACE_PATCH( "non-regular Patches(raw, eom)     = %zu, %zu \n", numRawPatches, numEomPatch );
  TRACE_PATCH( "Tile Type                         = %zu (0.P_TILE 1.I_TILE 2.SKIP_TILE)\n", (size_t)ath.getType() );
  size_t  totalNumberOfRawPoints = 0;
  size_t  totalNumberOfEomPoints = 0;
  size_t  patchIndex             = 0;
  int32_t packingBlockSize       = 1 << asps.getLog2PatchPackingBlockSize();
  double  packingBlockSizeD      = static_cast<double>( packingBlockSize );
  int32_t quantizerSizeX         = 1 << ath.getPatchSizeXinfoQuantizer();
  int32_t quantizerSizeY         = 1 << ath.getPatchSizeYinfoQuantizer();
  tile.setLog2PatchQuantizerSizeX( ath.getPatchSizeXinfoQuantizer() );
  tile.setLog2PatchQuantizerSizeY( ath.getPatchSizeYinfoQuantizer() );
  for ( patchIndex = 0; patchIndex < patchCount; patchIndex++ ) {
    auto&        pid           = atgdu.getPatchInformationData( patchIndex );
    PCCPatchType currPatchType = getPatchType( tileType, atgdu.getPatchMode( patchIndex ) );
    if ( currPatchType == INTRA_PATCH ) {
      PCCPatch patch;
      auto&    pdu = pid.getPatchDataUnit();
      patch.setOccupancyResolution( size_t( 1 ) << asps.getLog2PatchPackingBlockSize() );
      patch.setU0( pdu.get2dPosX() );
      patch.setV0( pdu.get2dPosY() );
      patch.setU1( pdu.get3dOffsetU() );
      patch.setV1( pdu.get3dOffsetV() );
      bool lodEnableFlag = pdu.getLodEnableFlag();
      if ( lodEnableFlag ) {
        patch.setLodScaleX( pdu.getLodScaleXMinus1() + 1 );
        patch.setLodScaleYIdc( pdu.getLodScaleYIdc() + ( patch.getLodScaleX() > 1 ? 1 : 2 ) );
      } else {
        patch.setLodScaleX( 1 );
        patch.setLodScaleYIdc( 1 );
      }
      patch.setSizeD( pdu.get3dRangeD() == 0 ? 0 : ( pdu.get3dRangeD() * minLevel - 1 ) );
      if ( asps.getPatchSizeQuantizerPresentFlag() ) {
        patch.setPatchSize2DXInPixel( ( pdu.get2dSizeXMinus1() + 1 ) * quantizerSizeX );
        patch.setPatchSize2DYInPixel( ( pdu.get2dSizeYMinus1() + 1 ) * quantizerSizeY );
        patch.setSizeU0( ceil( static_cast<double>( patch.getPatchSize2DXInPixel() ) / packingBlockSizeD ) );
        patch.setSizeV0( ceil( static_cast<double>( patch.getPatchSize2DYInPixel() ) / packingBlockSizeD ) );
      } else {
        patch.setSizeU0( pdu.get2dSizeXMinus1() + 1 );
        patch.setSizeV0( pdu.get2dSizeYMinus1() + 1 );
      }
      patch.setPatchOrientation( pdu.getOrientationIndex() );
      patch.setViewId( pdu.getProjectionId() );
      TRACE_PATCH( "patch %zu / %zu: Intra \n", patchIndex, patchCount );
      const size_t max3DCoordinate = size_t( 1 ) << geometryBitDepth3D;
      if ( patch.getProjectionMode() == 0 ) {
        patch.setD1( static_cast<int32_t>( pdu.get3dOffsetD() ) * minLevel );
      } else {
        patch.setD1( max3DCoordinate - static_cast<int32_t>( pdu.get3dOffsetD() ) * minLevel );
      }
      if ( patch.getNormalAxis() == 0 ) {
        patch.setTangentAxis( 2 );
        patch.setBitangentAxis( 1 );
      } else if ( patch.getNormalAxis() == 1 ) {
        patch.setTangentAxis( 2 );
        patch.setBitangentAxis( 0 );
      } else {
        patch.setTangentAxis( 0 );
        patch.setBitangentAxis( 1 );
      }
      TRACE_PATCH(
          "patch(Intra) %zu: UV0 %4zu %4zu UV1 %4zu %4zu D1=%4zu S=%4zu %4zu %4zu(%4zu) P=%zu O=%zu A=%u%u%u Lod "
          "=(%zu) %zu,%zu 45=%d ProjId=%4zu Axis=%zu \n",
          patchIndex, patch.getU0(), patch.getV0(), patch.getU1(), patch.getV1(), patch.getD1(), patch.getSizeU0(),
          patch.getSizeV0(), patch.getSizeD(), pdu.get3dRangeD(), patch.getProjectionMode(),
          patch.getPatchOrientation(), patch.getNormalAxis(), patch.getTangentAxis(), patch.getBitangentAxis(),
          (size_t)lodEnableFlag, patch.getLodScaleX(), patch.getLodScaleY(), asps.getExtendedProjectionEnabledFlag(),
          pdu.getProjectionId(), patch.getAxisOfAdditionalPlane() );
      patch.allocOneLayerData();
      if ( asps.getPLREnabledFlag() ) {
        setPLRData( tile, patch, pdu.getPLRData(), size_t( 1 ) << asps.getLog2PatchPackingBlockSize() );
      }
      patches.push_back( patch );
    } else if ( currPatchType == INTER_PATCH ) {
      PCCPatch patch;
      patch.setOccupancyResolution( size_t( 1 ) << asps.getLog2PatchPackingBlockSize() );
      auto& ipdu = pid.getInterPatchDataUnit();
      TRACE_PATCH( "patch %zu / %zu: Inter \n", patchIndex, patchCount );
      TRACE_PATCH(
          "\tIPDU: refAtlasFrame= %d refPatchIdx = %d pos2DXY = %ld %ld pos3DXYZW = %ld %ld %ld %ld size2D = %ld %ld "
          "\n",
          ipdu.getRefIndex(), ipdu.getRefPatchIndex(), ipdu.get2dPosX(), ipdu.get2dPosY(), ipdu.get3dOffsetU(),
          ipdu.get3dOffsetV(), ipdu.get3dOffsetD(), ipdu.get3dRangeD(), ipdu.get2dDeltaSizeX(),
          ipdu.get2dDeltaSizeY() );
      patch.setBestMatchIdx( static_cast<int32_t>( ipdu.getRefPatchIndex() + predIndex ) );
      predIndex += ipdu.getRefPatchIndex() + 1;
      patch.setRefAtlasFrameIndex( ipdu.getRefIndex() );
      size_t      refPOC   = (size_t)tile.getRefAfoc( patch.getRefAtlasFrameIndex() );
      const auto& refPatch = context.getFrame( refPOC ).getTile( tileIndex ).getPatches()[patch.getBestMatchIdx()];
      TRACE_PATCH(
          "\trefPatch: refIndex = %zu, refFrame = %zu, Idx = %zu/%zu UV0 = %zu %zu  UV1 = %zu %zu Size = %zu %zu %zu "
          "Lod = %u,%u\n",
          patch.getRefAtlasFrameIndex(), refPOC, patch.getBestMatchIdx(),
          context.getFrame( refPOC ).getTile( tileIndex ).getPatches().size(), refPatch.getU0(), refPatch.getV0(),
          refPatch.getU1(), refPatch.getV1(), refPatch.getSizeU0(), refPatch.getSizeV0(), refPatch.getSizeD(),
          refPatch.getLodScaleX(), refPatch.getLodScaleY() );
      patch.setProjectionMode( refPatch.getProjectionMode() );
      patch.setViewId( refPatch.getViewId() );
      patch.setU0( ipdu.get2dPosX() + refPatch.getU0() );
      patch.setV0( ipdu.get2dPosY() + refPatch.getV0() );
      patch.setPatchOrientation( refPatch.getPatchOrientation() );
      patch.setU1( ipdu.get3dOffsetU() + refPatch.getU1() );
      patch.setV1( ipdu.get3dOffsetV() + refPatch.getV1() );
      if ( asps.getPatchSizeQuantizerPresentFlag() ) {
        patch.setPatchSize2DXInPixel( refPatch.getPatchSize2DXInPixel() + ( ipdu.get2dDeltaSizeX() ) * quantizerSizeX );
        patch.setPatchSize2DYInPixel( refPatch.getPatchSize2DYInPixel() + ( ipdu.get2dDeltaSizeY() ) * quantizerSizeY );
        patch.setSizeU0( ceil( static_cast<double>( patch.getPatchSize2DXInPixel() ) / packingBlockSizeD ) );
        patch.setSizeV0( ceil( static_cast<double>( patch.getPatchSize2DYInPixel() ) / packingBlockSizeD ) );
      } else {
        patch.setSizeU0( ipdu.get2dDeltaSizeX() + refPatch.getSizeU0() );
        patch.setSizeV0( ipdu.get2dDeltaSizeY() + refPatch.getSizeV0() );
      }
      patch.setNormalAxis( refPatch.getNormalAxis() );
      patch.setTangentAxis( refPatch.getTangentAxis() );
      patch.setBitangentAxis( refPatch.getBitangentAxis() );
      patch.setAxisOfAdditionalPlane( refPatch.getAxisOfAdditionalPlane() );
      const size_t max3DCoordinate = size_t( 1 ) << geometryBitDepth3D;
      if ( patch.getProjectionMode() == 0 ) {
        patch.setD1( ( ipdu.get3dOffsetD() + ( refPatch.getD1() / minLevel ) ) * minLevel );
      } else {
        patch.setD1( max3DCoordinate -
                     ( ipdu.get3dOffsetD() + ( ( max3DCoordinate - refPatch.getD1() ) / minLevel ) ) * minLevel );
      }
      const int64_t delta_DD = ipdu.get3dRangeD() == 0 ? 0 : ( ipdu.get3dRangeD() * minLevel - 1 );
      patch.setSizeD( refPatch.getSizeD() + delta_DD );
      patch.setLodScaleX( refPatch.getLodScaleX() );
      patch.setLodScaleYIdc( refPatch.getLodScaleY() );
      TRACE_PATCH(
          "\tpatch(Inter) %zu: UV0 %4zu %4zu UV1 %4zu %4zu D1=%4zu S=%4zu %4zu %4zu from DeltaSize = %4ld %4ld P=%zu "
          "O=%zu A=%u%u%u Lod = %zu,%zu \n",
          patchIndex, patch.getU0(), patch.getV0(), patch.getU1(), patch.getV1(), patch.getD1(), patch.getSizeU0(),
          patch.getSizeV0(), patch.getSizeD(), ipdu.get2dDeltaSizeX(), ipdu.get2dDeltaSizeY(),
          patch.getProjectionMode(), patch.getPatchOrientation(), patch.getNormalAxis(), patch.getTangentAxis(),
          patch.getBitangentAxis(), patch.getLodScaleX(), patch.getLodScaleY() );

      patch.allocOneLayerData();
      if ( asps.getPLREnabledFlag() ) {
        setPLRData( tile, patch, ipdu.getPLRData(), size_t( 1 ) << asps.getLog2PatchPackingBlockSize() );
      }
      patches.push_back( patch );
    } else if ( currPatchType == MERGE_PATCH ) {
      assert( -2 );
      PCCPatch patch;
      patch.setOccupancyResolution( size_t( 1 ) << asps.getLog2PatchPackingBlockSize() );
      auto&        mpdu            = pid.getMergePatchDataUnit();
      bool         overridePlrFlag = false;
      const size_t max3DCoordinate = size_t( 1 ) << geometryBitDepth3D;
      TRACE_PATCH( "patch %zu / %zu: Inter \n", patchIndex, patchCount );
      TRACE_PATCH(
          "MPDU: refAtlasFrame= %d refPatchIdx = ?? pos2DXY = %ld %ld pos3DXYZW = %ld %ld %ld %ld size2D = %ld %ld \n",
          mpdu.getRefIndex(), mpdu.get2dPosX(), mpdu.get2dPosY(), mpdu.get3dOffsetU(), mpdu.get3dOffsetV(),
          mpdu.get3dOffsetD(), mpdu.get3dRangeD(), mpdu.get2dDeltaSizeX(), mpdu.get2dDeltaSizeY() );

      patch.setBestMatchIdx( patchIndex );
      predIndex = patchIndex;
      patch.setRefAtlasFrameIndex( mpdu.getRefIndex() );
      size_t      refPOC   = (size_t)tile.getRefAfoc( patch.getRefAtlasFrameIndex() );
      const auto& refPatch = context.getFrame( refPOC ).getTile( tileIndex ).getPatches()[patch.getBestMatchIdx()];
      if ( mpdu.getOverride2dParamsFlag() ) {
        patch.setU0( mpdu.get2dPosX() + refPatch.getU0() );
        patch.setV0( mpdu.get2dPosY() + refPatch.getV0() );
        if ( asps.getPatchSizeQuantizerPresentFlag() ) {
          patch.setPatchSize2DXInPixel( refPatch.getPatchSize2DXInPixel() + mpdu.get2dDeltaSizeX() * quantizerSizeX );
          patch.setPatchSize2DYInPixel( refPatch.getPatchSize2DYInPixel() + mpdu.get2dDeltaSizeY() * quantizerSizeY );
          patch.setSizeU0( ceil( static_cast<double>( patch.getPatchSize2DXInPixel() ) / packingBlockSizeD ) );
          patch.setSizeV0( ceil( static_cast<double>( patch.getPatchSize2DYInPixel() ) / packingBlockSizeD ) );
        } else {
          patch.setSizeU0( mpdu.get2dDeltaSizeX() + refPatch.getSizeU0() );
          patch.setSizeV0( mpdu.get2dDeltaSizeY() + refPatch.getSizeV0() );
        }

        if ( asps.getPLREnabledFlag() ) { overridePlrFlag = true; }
      } else {
        if ( mpdu.getOverride3dParamsFlag() ) {
          patch.setU1( mpdu.get3dOffsetU() + refPatch.getU1() );
          patch.setV1( mpdu.get3dOffsetV() + refPatch.getV1() );
          if ( patch.getProjectionMode() == 0 ) {
            patch.setD1( ( mpdu.get3dOffsetD() + ( refPatch.getD1() / minLevel ) ) * minLevel );
          } else {
            patch.setD1( max3DCoordinate -
                         ( mpdu.get3dOffsetD() + ( ( max3DCoordinate - refPatch.getD1() ) / minLevel ) ) * minLevel );
          }
          const int64_t delta_DD = mpdu.get3dRangeD() == 0 ? 0 : ( mpdu.get3dRangeD() * minLevel - 1 );
          patch.setSizeD( refPatch.getSizeD() + delta_DD );
          if ( asps.getPLREnabledFlag() ) { overridePlrFlag = ( mpdu.getOverridePlrFlag() != 0 ); }
        }
      }
      patch.setProjectionMode( refPatch.getProjectionMode() );
      patch.setViewId( refPatch.getViewId() );
      patch.setPatchOrientation( refPatch.getPatchOrientation() );
      patch.setNormalAxis( refPatch.getNormalAxis() );
      patch.setTangentAxis( refPatch.getTangentAxis() );
      patch.setBitangentAxis( refPatch.getBitangentAxis() );
      patch.setAxisOfAdditionalPlane( refPatch.getAxisOfAdditionalPlane() );
      patch.setLodScaleX( refPatch.getLodScaleX() );
      patch.setLodScaleYIdc( refPatch.getLodScaleY() );
      TRACE_PATCH(
          "patch(Inter) %zu: UV0 %4zu %4zu UV1 %4zu %4zu D1=%4zu S=%4zu %4zu %4zu from DeltaSize = %4ld %4ld P=%zu "
          "O=%zu A=%u%u%u Lod = %zu,%zu \n",
          patchIndex, patch.getU0(), patch.getV0(), patch.getU1(), patch.getV1(), patch.getD1(), patch.getSizeU0(),
          patch.getSizeV0(), patch.getSizeD(), mpdu.get2dDeltaSizeX(), mpdu.get2dDeltaSizeY(),
          patch.getProjectionMode(), patch.getPatchOrientation(), patch.getNormalAxis(), patch.getTangentAxis(),
          patch.getBitangentAxis(), patch.getLodScaleX(), patch.getLodScaleY() );

      patch.allocOneLayerData();
      if ( asps.getPLREnabledFlag() ) {
        setPLRData( tile, patch, mpdu.getPLRData(), size_t( 1 ) << asps.getLog2PatchPackingBlockSize() );
      }
      patches.push_back( patch );
    } else if ( currPatchType == SKIP_PATCH ) {
      assert( -1 );
      PCCPatch patch;
      TRACE_PATCH( "patch %zu / %zu: Inter \n", patchIndex, patchCount );
      TRACE_PATCH( "SDU: refAtlasFrame= 0 refPatchIdx = %d \n", patchIndex );
      patch.setBestMatchIdx( static_cast<int32_t>( patchIndex ) );
      predIndex += patchIndex;
      patch.setRefAtlasFrameIndex( 0 );
      size_t      refPOC   = (size_t)tile.getRefAfoc( patch.getRefAtlasFrameIndex() );
      const auto& refPatch = context.getFrame( refPOC ).getTile( tileIndex ).getPatches()[patch.getBestMatchIdx()];
      TRACE_PATCH( "\trefPatch: Idx = %zu UV0 = %zu %zu  UV1 = %zu %zu Size = %zu %zu %zu  Lod = %u,%u \n",
                   patch.getBestMatchIdx(), refPatch.getU0(), refPatch.getV0(), refPatch.getU1(), refPatch.getV1(),
                   refPatch.getSizeU0(), refPatch.getSizeV0(), refPatch.getSizeD(), refPatch.getLodScaleX(),
                   refPatch.getLodScaleY() );
      patch.setProjectionMode( refPatch.getProjectionMode() );
      patch.setViewId( refPatch.getViewId() );
      patch.setU0( refPatch.getU0() );
      patch.setV0( refPatch.getV0() );
      patch.setPatchOrientation( refPatch.getPatchOrientation() );
      patch.setU1( refPatch.getU1() );
      patch.setV1( refPatch.getV1() );
      if ( asps.getPatchSizeQuantizerPresentFlag() ) {
        patch.setPatchSize2DXInPixel( refPatch.getPatchSize2DXInPixel() );
        patch.setPatchSize2DYInPixel( refPatch.getPatchSize2DYInPixel() );
      }
      patch.setSizeU0( refPatch.getSizeU0() );
      patch.setSizeV0( refPatch.getSizeV0() );
      patch.setNormalAxis( refPatch.getNormalAxis() );
      patch.setTangentAxis( refPatch.getTangentAxis() );
      patch.setBitangentAxis( refPatch.getBitangentAxis() );
      patch.setAxisOfAdditionalPlane( refPatch.getAxisOfAdditionalPlane() );
      patch.setD1( refPatch.getD1() );
      patch.setSizeD( refPatch.getSizeD() );
      patch.setLodScaleX( refPatch.getLodScaleX() );
      patch.setLodScaleYIdc( refPatch.getLodScaleY() );
      TRACE_PATCH(
          "patch(skip) %zu: UV0 %4zu %4zu UV1 %4zu %4zu D1=%4zu S=%4zu %4zu %4zu P=%zu O=%zu A=%u%u%u Lod = %zu,%zu \n",
          patchIndex, patch.getU0(), patch.getV0(), patch.getU1(), patch.getV1(), patch.getD1(), patch.getSizeU0(),
          patch.getSizeV0(), patch.getSizeD(), patch.getProjectionMode(), patch.getPatchOrientation(),
          patch.getNormalAxis(), patch.getTangentAxis(), patch.getBitangentAxis(), patch.getLodScaleX(),
          patch.getLodScaleY() );
      patch.allocOneLayerData();
      patches.push_back( patch );
    } else if ( currPatchType == RAW_PATCH ) {
      TRACE_PATCH( "patch %zu / %zu: raw \n", patchIndex, patchCount );
      auto&             rpdu = pid.getRawPatchDataUnit();
      PCCRawPointsPatch rawPointsPatch;
      rawPointsPatch.isPatchInAuxVideo_ = rpdu.getPatchInAuxiliaryVideoFlag();
      rawPointsPatch.u0_                = rpdu.get2dPosX();
      rawPointsPatch.v0_                = rpdu.get2dPosY();
      rawPointsPatch.sizeU0_            = rpdu.get2dSizeXMinus1() + 1;
      rawPointsPatch.sizeV0_            = rpdu.get2dSizeYMinus1() + 1;
      if ( afps.getRaw3dOffsetBitCountExplicitModeFlag() ) {
        rawPointsPatch.u1_ = rpdu.get3dOffsetU();
        rawPointsPatch.v1_ = rpdu.get3dOffsetV();
        rawPointsPatch.d1_ = rpdu.get3dOffsetD();
      } else {
        const size_t pcmU1V1D1Level = size_t( 1 ) << geometryBitDepth2D;
        rawPointsPatch.u1_          = rpdu.get3dOffsetU() * pcmU1V1D1Level;
        rawPointsPatch.v1_          = rpdu.get3dOffsetV() * pcmU1V1D1Level;
        rawPointsPatch.d1_          = rpdu.get3dOffsetD() * pcmU1V1D1Level;
      }
      rawPointsPatch.setNumberOfRawPoints( rpdu.getRawPointsMinus1() + 1 );
      rawPointsPatch.occupancyResolution_ = size_t( 1 ) << asps.getLog2PatchPackingBlockSize();
      totalNumberOfRawPoints += rawPointsPatch.getNumberOfRawPoints();
      rawPatches.push_back( rawPointsPatch );
      TRACE_PATCH( "Raw :UV = %zu %zu  size = %zu %zu  uvd1 = %zu %zu %zu numPoints = %zu ocmRes = %zu \n",
                   rawPointsPatch.u0_, rawPointsPatch.v0_, rawPointsPatch.sizeU0_, rawPointsPatch.sizeV0_,
                   rawPointsPatch.u1_, rawPointsPatch.v1_, rawPointsPatch.d1_, rawPointsPatch.numberOfRawPoints_,
                   rawPointsPatch.occupancyResolution_ );
    } else if ( currPatchType == EOM_PATCH ) {
      TRACE_PATCH( "patch %zu / %zu: EOM \n", patchIndex, patchCount );
      auto&       epdu       = pid.getEomPatchDataUnit();
      auto&       eomPatches = tile.getEomPatches();
      PCCEomPatch eomPatch;
      eomPatch.isPatchInAuxVideo_ = epdu.getPatchInAuxiliaryVideoFlag();
      eomPatch.u0_                = epdu.get2dPosX();
      eomPatch.v0_                = epdu.get2dPosY();
      eomPatch.sizeU_             = epdu.get2dSizeXMinus1() + 1;
      eomPatch.sizeV_             = epdu.get2dSizeYMinus1() + 1;
      eomPatch.memberPatches_.resize( epdu.getPatchCountMinus1() + 1 );
      eomPatch.eomCountPerPatch_.resize( epdu.getPatchCountMinus1() + 1 );
      eomPatch.eomCount_ = 0;
      for ( size_t i = 0; i < eomPatch.memberPatches_.size(); i++ ) {
        eomPatch.memberPatches_[i]    = epdu.getAssociatedPatchesIdx( i );
        eomPatch.eomCountPerPatch_[i] = epdu.getPoints( i );
        eomPatch.eomCount_ += eomPatch.eomCountPerPatch_[i];
      }
      eomPatch.occupancyResolution_ = size_t( 1 ) << asps.getLog2PatchPackingBlockSize();
      eomPatches.push_back( eomPatch );
      totalNumberOfEomPoints += eomPatch.eomCount_;
      TRACE_PATCH( "EOM: U0V0 %zu,%zu\tSizeU0V0 %zu,%zu\tN= %zu,%zu\n", eomPatch.u0_, eomPatch.v0_, eomPatch.sizeU_,
                   eomPatch.sizeV_, eomPatch.memberPatches_.size(), eomPatch.eomCount_ );
      for ( size_t i = 0; i < eomPatch.memberPatches_.size(); i++ ) {
        TRACE_PATCH( "%zu, %zu\n", eomPatch.memberPatches_[i], eomPatch.eomCountPerPatch_[i] );
      }
      TRACE_PATCH( "\n" );
    } else if ( currPatchType == END_PATCH ) {
      break;
    } else {
      std::printf( "Error: unknow frame/patch type \n" );
      TRACE_PATCH( "Error: unknow frame/patch type \n" );
    }
  }
  TRACE_PATCH( "patch %zu / %zu: end \n", patches.size(), patches.size() );
  tile.setTotalNumberOfRawPoints( totalNumberOfRawPoints );
  tile.setTotalNumberOfEOMPoints( totalNumberOfEomPoints );
}

bool PCCDecoder::compareHashSEIMD5( std::vector<uint8_t>& encMD5, std::vector<uint8_t>& decMD5 ) {
  bool equal = true;
  for ( size_t i = 0; i < 16; i++ ) {
    if ( encMD5[i] != decMD5[i] ) {
      equal = false;
      break;
    }
  }
  for ( auto& e : encMD5 ) TRACE_SEI( "%02x", e );
  TRACE_SEI( ", " );
  for ( auto& d : decMD5 ) TRACE_SEI( "%02x", d );
  return equal;
}
bool PCCDecoder::compareHashSEICrc( uint16_t encCrc, uint16_t decCrc ) {
  bool equal = true;
  if ( encCrc != decCrc ) equal = false;
  TRACE_SEI( "%04x", encCrc );
  TRACE_SEI( ", " );
  TRACE_SEI( "%04x", decCrc );
  return equal;
}

bool PCCDecoder::compareHashSEICheckSum( uint32_t encCheckSum, uint32_t decCheckSum ) {
  bool equal = true;
  if ( encCheckSum != decCheckSum ) equal = false;
  TRACE_SEI( "%08x", encCheckSum );
  TRACE_SEI( ", " );
  TRACE_SEI( "%08x", decCheckSum );
  return equal;
}

void PCCDecoder::createHashSEI( PCCContext& context, int frameIndex, SEIDecodedAtlasInformationHash& sei ) {
  bool                                           seiHashCancelFlag = sei.getCancelFlag();
  std::vector<PatchParams>                       atlasPatchParams;
  std::vector<std::vector<PatchParams>>          tilePatchParams;
  std::vector<std::vector<std::vector<int64_t>>> tileB2PPatchParams;
  std::vector<std::vector<int64_t>>              atlasB2PPatchParams;
  TRACE_SEI( "*** Hash SEI Frame (%d) ***\n", frameIndex );

  if ( !seiHashCancelFlag && sei.getDecodedHighLevelHashPresentFlag() ) {
    size_t               atlIdx     = context[frameIndex].getTile( 0 ).getAtlIndex();
    auto&                tileHeader = context.getAtlasTileLayerList()[atlIdx].getHeader();
    size_t               afpsIndex  = tileHeader.getAtlasFrameParameterSetId();
    size_t               aspsIndex  = context.getAtlasFrameParameterSet( afpsIndex ).getAtlasSequenceParameterSetId();
    auto&                asps       = context.getAtlasSequenceParameterSet( aspsIndex );
    auto&                afps       = context.getAtlasFrameParameterSet( afpsIndex );
    std::vector<uint8_t> highLevelAtlasData;
    aspsCommonByteString( highLevelAtlasData, asps );
    aspsApplicationByteString( highLevelAtlasData, asps, afps );
    afpsCommonByteString( highLevelAtlasData, context, afpsIndex, frameIndex );
    afpsApplicationByteString( highLevelAtlasData, asps, afps );

    if ( sei.getHashType() == 0 ) {
      std::vector<uint8_t> encMD5( 16 ), decMD5( 16 );
      encMD5 = context.computeMD5( highLevelAtlasData.data(), highLevelAtlasData.size() );
      TRACE_SEI( " Derived (MD5) = " );
      for ( int j = 0; j < 16; j++ ) {
        decMD5[j] = sei.getHighLevelMd5( j );
        TRACE_SEI( "%02x", encMD5[j] );
      }
      TRACE_SEI( "\t Derived vs. SEI (MD5) : " );
      TRACE_SEI( "HLS MD5: " );
      bool equal = compareHashSEIMD5( encMD5, decMD5 );
      TRACE_SEI( " (%s) \n", equal ? "OK" : "DIFF" );
    } else if ( sei.getHashType() == 1 ) {
      uint16_t crc = context.computeCRC( highLevelAtlasData.data(), highLevelAtlasData.size() );
      TRACE_SEI( " Derived (CRC): %d ", crc );
      TRACE_SEI( "HLS CRC: " );
      bool equal = compareHashSEICrc( crc, sei.getHighLevelCrc() );
      TRACE_SEI( " (%s) \n", equal ? "OK" : "DIFF" );
    } else if ( sei.getHashType() == 2 ) {
      uint32_t checkSum = context.computeCheckSum( highLevelAtlasData.data(), highLevelAtlasData.size() );
      TRACE_SEI( " Derived (CheckSum): %d ", checkSum );
      TRACE_SEI( "HLS CheckSum: " );
      bool equal = compareHashSEICheckSum( checkSum, sei.getHighLevelCheckSum() );
      TRACE_SEI( " (%s) \n", equal ? "OK" : "DIFF" );
    }
    highLevelAtlasData.clear();
  }

  if ( !seiHashCancelFlag && ( sei.getDecodedAtlasTilesHashPresentFlag() || sei.getDecodedAtlasHashPresentFlag() ) ) {
    size_t numTilesInPatchFrame = context[frameIndex].getNumTilesInAtlasFrame();
    if ( sei.getDecodedAtlasTilesHashPresentFlag() ) { tilePatchParams.resize( numTilesInPatchFrame ); }
    for ( size_t tileIdx = 0; tileIdx < numTilesInPatchFrame; tileIdx++ ) {
      getHashPatchParams( context, frameIndex, tileIdx, tilePatchParams, atlasPatchParams );
    }
  }
  if ( !seiHashCancelFlag &&
       ( sei.getDecodedAtlasB2pHashPresentFlag() || sei.getDecodedAtlasTilesB2pHashPresentFlag() ) ) {
    getB2PHashPatchParams( context, frameIndex, tileB2PPatchParams, atlasB2PPatchParams );
  }
  // frame
  TRACE_SEI( "\n" );
  if ( !seiHashCancelFlag && sei.getDecodedAtlasHashPresentFlag() ) {
    std::vector<uint8_t> atlasData;
    size_t               patchCount = atlasPatchParams.size();
    for ( size_t patchIdx = 0; patchIdx < patchCount; patchIdx++ ) {
      atlasPatchCommonByteString( atlasData, patchIdx, atlasPatchParams );
      atlasPatchApplicationByteString( atlasData, patchIdx, atlasPatchParams );
    }
    printf( "AtlasPatchHash: frame(%d) (#patches %zu)\n", frameIndex, patchCount );

    if ( sei.getHashType() == 0 ) {
      std::vector<uint8_t> encMD5( 16 ), decMD5( 16 );
      encMD5 = context.computeMD5( atlasData.data(), atlasData.size() );
      TRACE_SEI( " Derived Atlas MD5 = " );
      TRACE_SEI( "Atlas MD5: " );
      for ( int j = 0; j < 16; j++ ) {
        decMD5[j] = sei.getAtlasMd5( j );
        TRACE_SEI( "%02x", encMD5[j] );
      }
      TRACE_SEI( "\n\t**sei** (MD5): " );
      bool equal = compareHashSEIMD5( encMD5, decMD5 );
      TRACE_SEI( " (%s) \n", equal ? "OK" : "DIFF" );
    } else if ( sei.getHashType() == 1 ) {
      uint16_t crc = context.computeCRC( atlasData.data(), atlasData.size() );
      TRACE_SEI( "\n Derived (CRC): %d", crc );
      TRACE_SEI( "Atlas CRC: " );
      bool equal = compareHashSEICrc( crc, sei.getAtlasCrc() );
      TRACE_SEI( " (%s) \n", equal ? "OK" : "DIFF" );
    } else if ( sei.getHashType() == 2 ) {
      uint32_t checkSum = context.computeCheckSum( atlasData.data(), atlasData.size() );
      TRACE_SEI( "\n Derived (CheckSum): %d", checkSum );
      TRACE_SEI( "Atlas CheckSum: " );
      bool equal = compareHashSEICheckSum( checkSum, sei.getAtlasCheckSum() );
      TRACE_SEI( " (%s) \n", equal ? "OK" : "DIFF" );
    }
    atlasData.clear();
  }
  if ( sei.getDecodedAtlasB2pHashPresentFlag() && !seiHashCancelFlag ) {
    std::vector<uint8_t> atlasB2PData;
    atlasBlockToPatchByteString( atlasB2PData, atlasB2PPatchParams );

    TRACE_SEI( "**sei** AtlasBlockToPatchHash: frame(%d)", frameIndex );
    if ( sei.getHashType() == 0 ) {
      bool                 equal = true;
      std::vector<uint8_t> encMD5( 16 ), decMD5( 16 );
      encMD5 = context.computeMD5( atlasB2PData.data(), atlasB2PData.size() );
      TRACE_SEI( " Derived Atlas B2P MD5 = " );
      TRACE_SEI( "Atlas B2P MD5: " );
      for ( int j = 0; j < 16; j++ ) {
        decMD5[j] = sei.getAtlasB2pMd5( j );
        TRACE_SEI( "%02x", encMD5[j] );
      }
      equal = compareHashSEIMD5( encMD5, decMD5 );
      TRACE_SEI( " (%s) \n", equal ? "OK" : "DIFF" );
    } else if ( sei.getHashType() == 1 ) {
      uint16_t crc = context.computeCRC( atlasB2PData.data(), atlasB2PData.size() );
      TRACE_SEI( "\n Derived (CRC): %d ", crc );
      TRACE_SEI( "Atlas B2P CRC: " );
      bool equal = compareHashSEICrc( crc, sei.getAtlasB2pCrc() );
      TRACE_SEI( " (%s) \n", equal ? "OK" : "DIFF" );
    } else if ( sei.getHashType() == 2 ) {
      uint32_t checkSum = context.computeCheckSum( atlasB2PData.data(), atlasB2PData.size() );
      TRACE_SEI( "\n Derived (CheckSum): %d ", checkSum );
      TRACE_SEI( "Atlas B2P CheckSum: " );
      bool equal = compareHashSEICheckSum( checkSum, sei.getAtlasB2pCheckSum() );
      TRACE_SEI( " (%s) \n", equal ? "OK" : "DIFF" );
    }
    atlasB2PData.clear();
    TRACE_SEI( "\n" );
  }

  // for tiles
  if ( !seiHashCancelFlag && sei.getDecodedAtlasTilesHashPresentFlag() ||
       sei.getDecodedAtlasTilesB2pHashPresentFlag() ) {
    size_t numTilesInPatchFrame = context[frameIndex].getNumTilesInAtlasFrame();
    printf( "**sei** AtlasTilesHash: frame(%d) (#Tiles %zu)", frameIndex, numTilesInPatchFrame );
    for ( size_t tileIdx = 0; tileIdx < numTilesInPatchFrame; tileIdx++ ) {
      auto&       tile     = context[frameIndex].getTile( tileIdx );
      auto&       atlu     = context.getAtlasTileLayer( tile.getAtlIndex() );
      auto&       ath      = atlu.getHeader();
      size_t      tileId   = ath.getId();
      PCCTileType tileType = ath.getType();
      auto&       afps     = context.getAtlasFrameParameterSet( ath.getAtlasFrameParameterSetId() );
      auto&       afti     = afps.getAtlasFrameTileInformation();
      if ( sei.getDecodedAtlasTilesHashPresentFlag() ) {
        std::vector<uint8_t> atlasTileData;
        for ( size_t patchIdx = 0; patchIdx < atlu.getDataUnit().getPatchCount(); patchIdx++ ) {
          tilePatchCommonByteString( atlasTileData, tileId, patchIdx, tilePatchParams );
          tilePatchApplicationByteString( atlasTileData, tileId, patchIdx, tilePatchParams );
        }
        printf( "**sei** TilesPatchHash: frame(%d), tile(tileIdx %zu, tileId %zu)\n", frameIndex, tileIdx, tileId );
        if ( sei.getHashType() == 0 ) {
          std::vector<uint8_t> encMD5( 16 ), decMD5( 16 );
          encMD5 = context.computeMD5( atlasTileData.data(), atlasTileData.size() );
          TRACE_SEI( " Derived Tile MD5 = " );
          TRACE_SEI( "Tile( id = %d, idx = %d ) MD5: ", tileId, tileIdx );
          for ( int j = 0; j < 16; j++ ) {
            decMD5[j] = sei.getAtlasTilesMd5( tileId, j );
            TRACE_SEI( "%02x", encMD5[j] );
          }
          bool equal = compareHashSEIMD5( encMD5, decMD5 );
          TRACE_SEI( " (%s) \n", equal ? "OK" : "DIFF" );
        } else if ( sei.getHashType() == 1 ) {
          uint16_t crc = context.computeCRC( atlasTileData.data(), atlasTileData.size() );
          // TRACE_SEI( "\n Derived  (CRC): %d ", crc );
          TRACE_SEI( "Tile( id = %d, idx = %d ) CRC: ", tileId, tileIdx );
          bool equal = compareHashSEICrc( crc, sei.getAtlasTilesCrc( tileId ) );
          TRACE_SEI( " (%s) \n", equal ? "OK" : "DIFF" );
        } else if ( sei.getHashType() == 2 ) {
          uint32_t checkSum = context.computeCheckSum( atlasTileData.data(), atlasTileData.size() );
          TRACE_SEI( "\n Derived CheckSum: %d ", checkSum );
          TRACE_SEI( "Tile( id = %d, idx = %d ) CheckSum: ", tileId, tileIdx );
          bool equal = compareHashSEICheckSum( checkSum, sei.getAtlasTilesCheckSum( tileId ) );
          TRACE_SEI( " (%s) \n", equal ? "OK" : "DIFF" );
        }
        atlasTileData.clear();
      }
      if ( sei.getDecodedAtlasTilesB2pHashPresentFlag() ) {
        std::vector<uint8_t> tileB2PData;
        tileBlockToPatchByteString( tileB2PData, tileId, tileB2PPatchParams );
        printf( "\n**sei** TilesBlockToPatchHash: frame(%d), tile(tileIdx %zu, tileId %zu)", frameIndex, tileIdx,
                tileId );
        if ( sei.getHashType() == 0 ) {
          std::vector<uint8_t> encMD5( 16 ), decMD5( 16 );
          encMD5 = context.computeMD5( tileB2PData.data(), tileB2PData.size() );
          TRACE_SEI( " Derived Tile B2P MD5 = " );
          TRACE_SEI( "Tile B2P( id = %d, idx = %d ) MD5: ", tileId, tileIdx );
          for ( int j = 0; j < 16; j++ ) {
            decMD5[j] = sei.getAtlasTilesB2pMd5( tileId, j );
            TRACE_SEI( "%02x", encMD5[j] );
          }
          bool equal = compareHashSEIMD5( encMD5, decMD5 );
          TRACE_SEI( " (%s) \n", equal ? "OK" : "DIFF" );
        } else if ( sei.getHashType() == 1 ) {
          uint16_t crc = context.computeCRC( tileB2PData.data(), tileB2PData.size() );
          TRACE_SEI( "\n Derived Tile B2P CRC: %d ", crc );
          TRACE_SEI( "Tile B2P( id = %d, idx = %d ) CRC: ", tileId, tileIdx );
          bool equal = compareHashSEICrc( crc, sei.getAtlasTilesB2pCrc( tileId ) );
          TRACE_SEI( " (%s) \n", equal ? "OK" : "DIFF" );
        } else if ( sei.getHashType() == 2 ) {
          uint32_t checkSum = context.computeCheckSum( tileB2PData.data(), tileB2PData.size() );
          TRACE_SEI( "\n Derived Tile B2P CheckSum: %d ", checkSum );
          TRACE_SEI( "Tile( id = %d, idx = %d ) CheckSum: ", tileId, tileIdx );
          bool equal = compareHashSEICheckSum( checkSum, sei.getAtlasTilesB2pCheckSum( tileId ) );
          TRACE_SEI( " (%s) \n", equal ? "OK" : "DIFF" );
        }
        tileB2PData.clear();
      }
      TRACE_SEI( "\n" );
    }  // tileIdx
  }

  if ( atlasPatchParams.size() != 0 ) atlasPatchParams.clear();
  if ( tilePatchParams.size() != 0 ) {
    for ( size_t ti = 0; ti < tilePatchParams.size(); ti++ )
      if ( tilePatchParams[ti].size() != 0 ) tilePatchParams[ti].clear();
  }
  tilePatchParams.clear();
  for ( auto& e : atlasB2PPatchParams ) e.clear();
  atlasB2PPatchParams.clear();
  for ( auto& e : tileB2PPatchParams ) {
    for ( auto d : e ) d.clear();
    e.clear();
  }
  tileB2PPatchParams.clear();

#ifdef CONFORMANCE_TRACE
  auto& temp = sei.getMD5ByteStrData();
  if ( temp.size() > 0 ) {  // ajt:: An example of how to generate md5 checksum for hash SEI message - could be computed
                            // different ways!
    TRACE_HLS( "**********DECODED_ATLAS_INFORMATION_HASH_NSEI***********\n" );
    TRACE_HLS( "SEI%02dMD5 = ", sei.getPayloadType() );
    SEIMd5Checksum( context, temp );
  }
#endif
}

void PCCDecoder::createHlsAtlasTileLogFiles( PCCContext& context, int frameIndex ) {
  size_t atlIdx     = context[frameIndex].getTile( 0 ).getAtlIndex();
  auto&  tileHeader = context.getAtlasTileLayerList()[atlIdx].getHeader();
  auto&  atlu       = context.getAtlasTileLayer( atlIdx );
  size_t afpsIndex  = tileHeader.getAtlasFrameParameterSetId();
  size_t aspsIndex  = context.getAtlasFrameParameterSet( afpsIndex ).getAtlasSequenceParameterSetId();
  auto&  asps       = context.getAtlasSequenceParameterSet( aspsIndex );
  auto&  afps       = context.getAtlasFrameParameterSet( afpsIndex );
  auto&  vps        = context.getVps();

  TRACE_HLS( "AtlasFrameIndex = %d\n", frameIndex );
  /*TRACE_HLS( "Atlas Frame Parameter Set Index = %d\n", afpsIndex );*/
  std::vector<uint8_t> decMD5( 16 );
  std::vector<uint8_t> highLevelAtlasData;
  aspsCommonByteString( highLevelAtlasData, asps );
  /*decMD5 = context.computeMD5( highLevelAtlasData.data(), highLevelAtlasData.size() );
  TRACE_HLS( " HLSMD5 = " );
  for ( int j = 0; j < 16; j++ ) TRACE_HLS( "%02x", decMD5[j] );
  TRACE_HLS( "\n" );*/
  aspsApplicationByteString( highLevelAtlasData, asps, afps );
  /*decMD5 = context.computeMD5( highLevelAtlasData.data(), highLevelAtlasData.size() );
  TRACE_HLS( " HLSMD5 = " );
  for ( int j = 0; j < 16; j++ ) TRACE_HLS( "%02x", decMD5[j] );
  TRACE_HLS( "\n" );*/
  afpsCommonByteString( highLevelAtlasData, context, afpsIndex, frameIndex );
  /*decMD5 = context.computeMD5( highLevelAtlasData.data(), highLevelAtlasData.size() );
  TRACE_HLS( " HLSMD5 = " );
  for ( int j = 0; j < 16; j++ ) TRACE_HLS( "%02x", decMD5[j] );
  TRACE_HLS( "\n" );*/
  afpsApplicationByteString( highLevelAtlasData, asps, afps );
  decMD5 = context.computeMD5( highLevelAtlasData.data(), highLevelAtlasData.size() );
  TRACE_HLS( "HLSMD5 = " );
  for ( int j = 0; j < 16; j++ ) TRACE_HLS( "%02x", decMD5[j] );
  TRACE_HLS( "\n" );
  highLevelAtlasData.clear();

  std::vector<PatchParams>                       atlasPatchParams;
  std::vector<std::vector<PatchParams>>          tilePatchParams;
  std::vector<std::vector<std::vector<int64_t>>> tileB2PPatchParams;
  std::vector<std::vector<int64_t>>              atlasB2PPatchParams;
  size_t                                         numTilesInPatchFrame = context[frameIndex].getNumTilesInAtlasFrame();
  tilePatchParams.resize( numTilesInPatchFrame );
  for ( size_t tileIdx = 0; tileIdx < numTilesInPatchFrame; tileIdx++ ) {
    getHashPatchParams( context, frameIndex, tileIdx, tilePatchParams, atlasPatchParams );
  }
  getB2PHashPatchParams( context, frameIndex, tileB2PPatchParams, atlasB2PPatchParams );
  // frame
  size_t numProjPatches = 0, numRawPatches = 0, numEomPatches = 0;
  size_t numProjPoints = 0, numRawPoints = 0, numEomPoints = 0;
  int    atlasFrameOrderCnt = atlu.getAtlasFrmOrderCntVal();
  for ( size_t tileIdx = 0; tileIdx < numTilesInPatchFrame; tileIdx++ ) {
    auto& tile = context[frameIndex].getTile( tileIdx );
    numProjPatches += tile.getPatches().size();
    numEomPatches += tile.getEomPatches().size();
    numRawPatches += tile.getRawPointsPatches().size();
    numProjPoints += tile.getTotalNumberOfRegularPoints();
    numEomPoints += tile.getTotalNumberOfEOMPoints();
    numRawPoints += tile.getTotalNumberOfRawPoints();
  }
  TRACE_ATLAS( "AtlasFrameIndex = %d\n", frameIndex );
  TRACE_ATLAS(
      "AtlasFrameOrderCntVal = %d,  AtlasFrameWidthMax =  %d, AtlasFrameHeightMax = %d, AtlasID = %d, "
      "ASPSFrameSize = %d, VPSMapCount = %d, AttributeCount = %d, AttributeDimension = %d, NumTilesAtlasFrame = %d, "
      "AtlasTotalNumProjPatches = %d, AtlasTotalNumRawPatches = %d, AtlasTotalNumEomPatches = %d, ",
      atlasFrameOrderCnt, asps.getFrameWidth(), asps.getFrameHeight(), vps.getAtlasId( 0 ),
      asps.getFrameWidth() * asps.getFrameHeight(), vps.getMapCountMinus1( 0 ) + 1,
      vps.getAttributeInformation( 0 ).getAttributeCount(),
      vps.getAttributeInformation( 0 ).getAttributeCount() > 0
          ? vps.getAttributeInformation( 0 ).getAttributeDimensionMinus1( 0 ) + 1
          : 0,
      afps.getAtlasFrameTileInformation().getNumTilesInAtlasFrameMinus1() + 1, numProjPatches, numRawPatches,
      numEomPatches );
  std::vector<uint8_t> atlasData;
  size_t               patchCount = atlasPatchParams.size();
  for ( size_t patchIdx = 0; patchIdx < patchCount; patchIdx++ ) {
    atlasPatchCommonByteString( atlasData, patchIdx, atlasPatchParams );
    atlasPatchApplicationByteString( atlasData, patchIdx, atlasPatchParams );
  }
  decMD5 = context.computeMD5( atlasData.data(), atlasData.size() );
  TRACE_ATLAS( " Atlas MD5 = " );
  for ( int j = 0; j < 16; j++ ) { TRACE_ATLAS( "%02x", decMD5[j] ); }
  TRACE_ATLAS( "," );
  atlasData.clear();
  std::vector<uint8_t> atlasB2PData;
  atlasBlockToPatchByteString( atlasB2PData, atlasB2PPatchParams );
  decMD5 = context.computeMD5( atlasB2PData.data(), atlasB2PData.size() );
  TRACE_ATLAS( " Atlas B2P MD5 = " );
  for ( int j = 0; j < 16; j++ ) TRACE_ATLAS( "%02x", decMD5[j] );
  atlasB2PData.clear();
  TRACE_ATLAS( "\n" );

  // for tiles
  TRACE_TILE( "AtlasFrameIndex = %d\n", frameIndex );
  for ( size_t tileIdx = 0; tileIdx < numTilesInPatchFrame; tileIdx++ ) {
    auto&       tile          = context[frameIndex].getTile( tileIdx );
    auto&       atlu          = context.getAtlasTileLayer( tile.getAtlIndex() );  // ajt::why atlIdx?
    auto&       ath           = atlu.getHeader();
    size_t      tileId        = ath.getId();
    PCCTileType tileType      = ath.getType();
    auto&       afps          = context.getAtlasFrameParameterSet( ath.getAtlasFrameParameterSetId() );
    auto&       afti          = afps.getAtlasFrameTileInformation();
    size_t      topLeftColumn = afti.getTopLeftPartitionIdx( tileIdx ) % ( afti.getNumPartitionColumnsMinus1() + 1 );
    size_t      topLeftRow    = afti.getTopLeftPartitionIdx( tileIdx ) / ( afti.getNumPartitionColumnsMinus1() + 1 );
    size_t      tileOffsetX   = context[frameIndex].getPartitionPosX( topLeftColumn );
    size_t      tileOffsetY   = context[frameIndex].getPartitionPosY( topLeftRow );
    TRACE_TILE(
        "TileID = %d, AtlasFrameOrderCntVal = %d, TileType = %d, TileOffsetX = %d, TileOffsetY = %d, TileWidth = %d, "
        "TileHeight = %d, ",
        tileId, ath.getAtlasFrmOrderCntLsb(), tileType, tileOffsetX, tileOffsetY, tile.getWidth(), tile.getHeight() );
    std::vector<uint8_t> atlasTileData;
    for ( size_t patchIdx = 0; patchIdx < atlu.getDataUnit().getPatchCount(); patchIdx++ ) {
      tilePatchCommonByteString( atlasTileData, tileId, patchIdx, tilePatchParams );
      tilePatchApplicationByteString( atlasTileData, tileId, patchIdx, tilePatchParams );
    }
    decMD5 = context.computeMD5( atlasTileData.data(), atlasTileData.size() );
    TRACE_TILE( " Tile MD5 = " );
    for ( int j = 0; j < 16; j++ ) TRACE_TILE( "%02x", decMD5[j] );
    TRACE_TILE( "," );
    atlasTileData.clear();
    std::vector<uint8_t> tileB2PData;
    tileBlockToPatchByteString( tileB2PData, tileId, tileB2PPatchParams );
    decMD5 = context.computeMD5( tileB2PData.data(), tileB2PData.size() );
    TRACE_TILE( " Tile B2P MD5 = " );
    for ( int j = 0; j < 16; j++ ) TRACE_TILE( "%02x", decMD5[j] );
    tileB2PData.clear();
    TRACE_TILE( "\n" );
  }  // tileIdx

  if ( atlasPatchParams.size() != 0 ) { atlasPatchParams.clear(); }
  if ( tilePatchParams.size() != 0 ) {
    for ( size_t ti = 0; ti < tilePatchParams.size(); ti++ ) {
      if ( tilePatchParams[ti].size() != 0 ) { tilePatchParams[ti].clear(); }
    }
  }
  tilePatchParams.clear();
  for ( auto& e : atlasB2PPatchParams ) { e.clear(); }
  atlasB2PPatchParams.clear();
  for ( auto& e : tileB2PPatchParams ) {
    for ( auto d : e ) { d.clear(); }
    e.clear();
  }
  tileB2PPatchParams.clear();
}

void PCCDecoder::setTilePartitionSizeAfti( PCCContext& context ) {  // decoder

  for ( size_t afpsIdx = 0; afpsIdx < context.getAtlasFrameParameterSetList().size(); afpsIdx++ ) {
    auto&  afps             = context.getAtlasFrameParameterSet( afpsIdx );
    auto&  asps             = context.getAtlasSequenceParameterSet( afps.getAtlasSequenceParameterSetId() );
    auto&  afti             = afps.getAtlasFrameTileInformation();
    size_t frameWidth       = asps.getFrameWidth();
    size_t frameHeight      = asps.getFrameHeight();
    size_t numPartitionCols = afti.getNumPartitionColumnsMinus1() + 1;
    size_t numPartitionRows = afti.getNumPartitionRowsMinus1() + 1;
    auto&  partitionWidth   = afti.getPartitionWidth();  // ajt::should be
    auto&  partitionHeight  = afti.getPartitionHeight();
    auto&  partitionPosX    = afti.getPartitionPosX();
    auto&  partitionPosY    = afti.getPartitionPosY();
    partitionWidth.resize( numPartitionCols );
    partitionHeight.resize( numPartitionRows );
    partitionPosX.resize( numPartitionCols );
    partitionPosY.resize( numPartitionRows );

    if ( afti.getSingleTileInAtlasFrameFlag() ) {
      partitionWidth[0]  = asps.getFrameWidth();
      partitionHeight[0] = asps.getFrameHeight();
      partitionPosX[0]   = 0;
      partitionPosY[0]   = 0;
    } else {
      if ( afti.getUniformPartitionSpacingFlag() ) {
        size_t uniformPatitionWidth  = 64 * ( afti.getPartitionColumnWidthMinus1( 0 ) + 1 );
        size_t uniformPatitionHeight = 64 * ( afti.getPartitionRowHeightMinus1( 0 ) + 1 );
        partitionPosX[0]             = 0;
        partitionWidth[0]            = uniformPatitionWidth;
        for ( size_t col = 1; col < numPartitionCols - 1; col++ ) {
          partitionPosX[col]  = partitionPosX[col - 1] + partitionWidth[col - 1];
          partitionWidth[col] = uniformPatitionWidth;
        }
        if ( numPartitionCols > 1 ) {
          partitionPosX[numPartitionCols - 1] =
              partitionPosX[numPartitionCols - 2] + partitionWidth[numPartitionCols - 2];
          partitionWidth[numPartitionCols - 1] = frameWidth - partitionPosX[numPartitionCols - 1];
        }

        partitionPosY[0]   = 0;
        partitionHeight[0] = uniformPatitionHeight;
        for ( size_t row = 1; row < numPartitionRows - 1; row++ ) {
          partitionPosY[row]   = partitionPosY[row - 1] + partitionHeight[row - 1];
          partitionHeight[row] = uniformPatitionHeight;
        }
        if ( numPartitionRows > 1 ) {
          partitionPosY[numPartitionRows - 1] =
              partitionPosY[numPartitionRows - 2] + partitionHeight[numPartitionRows - 2];
          partitionHeight[numPartitionRows - 1] = frameHeight - partitionPosY[numPartitionRows - 1];
        }
      } else {
        partitionPosX[0]  = 0;
        partitionWidth[0] = 64 * ( afti.getPartitionColumnWidthMinus1( 0 ) + 1 );
        for ( size_t col = 1; col < numPartitionCols - 1; col++ ) {
          partitionPosX[col]  = partitionPosX[col - 1] + partitionWidth[col - 1];
          partitionWidth[col] = 64 * ( afti.getPartitionColumnWidthMinus1( col ) + 1 );
        }
        if ( numPartitionCols > 1 ) {
          partitionPosX[numPartitionCols - 1] =
              partitionPosX[numPartitionCols - 2] + partitionWidth[numPartitionCols - 2];
          partitionWidth[numPartitionCols - 1] = frameWidth - partitionPosX[numPartitionCols - 1];
        }

        partitionPosY[0]   = 0;
        partitionHeight[0] = 64 * ( afti.getPartitionRowHeightMinus1( 0 ) + 1 );
        for ( size_t row = 1; row < numPartitionRows - 1; row++ ) {
          partitionPosY[row]   = partitionPosY[row - 1] + partitionHeight[row - 1];
          partitionHeight[row] = 64 * ( afti.getPartitionRowHeightMinus1( row ) + 1 );
        }
        if ( numPartitionRows > 1 ) {
          partitionPosY[numPartitionRows - 1] =
              partitionPosY[numPartitionRows - 2] + partitionHeight[numPartitionRows - 2];
          partitionHeight[numPartitionRows - 1] = frameHeight - partitionPosY[numPartitionRows - 1];
        }
      }
    }
  }  // afpsIdx
}

size_t PCCDecoder::setTileSizeAndLocation( PCCContext& context, size_t frameIndex, AtlasTileHeader& ath ) {  // decoder
  size_t afpsIdx   = ath.getAtlasFrameParameterSetId();
  auto&  afps      = context.getAtlasFrameParameterSet( afpsIdx );
  auto&  asps      = context.getAtlasSequenceParameterSet( afps.getAtlasSequenceParameterSetId() );
  auto&  afti      = afps.getAtlasFrameTileInformation();
  size_t tileIndex = 0;
  printf( "setTileSizeAndLocation frameIndex = %zu \n", frameIndex );
  fflush( stdout );
  // this is for the hash functions
  context[frameIndex].getPartitionPosX().clear();
  context[frameIndex].getPartitionPosY().clear();
  context[frameIndex].getPartitionWidth().clear();
  context[frameIndex].getPartitionHeight().clear();
  for ( auto v : afti.getPartitionWidth() ) { context[frameIndex].getPartitionWidth().push_back( v ); }
  for ( auto v : afti.getPartitionHeight() ) { context[frameIndex].getPartitionHeight().push_back( v ); }
  for ( auto v : afti.getPartitionPosX() ) { context[frameIndex].getPartitionPosX().push_back( v ); }
  for ( auto v : afti.getPartitionPosY() ) { context[frameIndex].getPartitionPosY().push_back( v ); }

  if ( afti.getSingleTileInAtlasFrameFlag() ) {
    if ( afti.getNumTilesInAtlasFrameMinus1() == 0 ) {
      context[frameIndex].setAtlasFrameWidth( asps.getFrameWidth() );
      context[frameIndex].setAtlasFrameHeight( asps.getFrameHeight() );
      context[frameIndex].setNumTilesInAtlasFrame( 1 );
    } else {
      assert( context[frameIndex].getAtlasFrameWidth() == ( asps.getFrameWidth() ) );
      assert( context[frameIndex].getAtlasFrameHeight() == ( asps.getFrameHeight() ) );
      assert( context[frameIndex].getNumTilesInAtlasFrame() == 1 );
    }
    auto& tile = context[frameIndex].getTile( 0 );
    tile.setTileIndex( tileIndex );
    tile.setLeftTopXInFrame( 0 );
    tile.setLeftTopYInFrame( 0 );
    tile.setWidth( asps.getFrameWidth() );
    tile.setHeight( asps.getFrameHeight() );
  } else {
    context[frameIndex].setAtlasFrameWidth( asps.getFrameWidth() );
    context[frameIndex].setAtlasFrameHeight( asps.getFrameHeight() );
    context[frameIndex].setNumTilesInAtlasFrame( afti.getNumTilesInAtlasFrameMinus1() + 1 );

    context[frameIndex].initNumTiles( context[frameIndex].getNumTilesInAtlasFrame() );
    for ( size_t tileId = 0; tileId <= afti.getNumTilesInAtlasFrameMinus1(); tileId++ ) {  // tileId = ath.getId()
      tileIndex   = afti.getSignalledTileIdFlag() ? afti.getTileId( tileId ) : tileId;  // ajt:: tileId vs. tileIndex?
      auto&  tile = context[frameIndex].getTile( tileIndex );
      size_t TopLeftPartitionColumn =
          afti.getTopLeftPartitionIdx( tileIndex ) % ( afti.getNumPartitionColumnsMinus1() + 1 );
      size_t TopLeftPartitionRow =
          afti.getTopLeftPartitionIdx( tileIndex ) / ( afti.getNumPartitionColumnsMinus1() + 1 );
      size_t BottomRightPartitionColumn =
          TopLeftPartitionColumn + afti.getBottomRightPartitionColumnOffset( tileIndex );
      size_t BottomRightPartitionRow = TopLeftPartitionRow + afti.getBottomRightPartitionRowOffset( tileIndex );

      size_t tileStartX = afti.getPartitionPosX( TopLeftPartitionColumn );
      size_t tileStartY = afti.getPartitionPosY( TopLeftPartitionRow );
      size_t tileWidth  = 0;
      size_t tileHeight = 0;
      for ( size_t j = TopLeftPartitionColumn; j <= BottomRightPartitionColumn; j++ ) {
        tileWidth += ( afti.getPartitionWidth( j ) );
      }
      for ( size_t j = TopLeftPartitionRow; j <= BottomRightPartitionRow; j++ ) {
        tileHeight += ( afti.getPartitionHeight( j ) );
      }
      tile.setLeftTopXInFrame( tileStartX );
      tile.setLeftTopYInFrame( tileStartY );

      if ( ( tile.getLeftTopXInFrame() + tileWidth ) > context[frameIndex].getAtlasFrameWidth() )
        tileWidth = context[0].getAtlasFrameWidth() - tile.getLeftTopXInFrame();
      if ( ( tile.getLeftTopYInFrame() + tileHeight ) > context[frameIndex].getAtlasFrameHeight() )
        tileHeight = context[0].getAtlasFrameHeight() - tile.getLeftTopYInFrame();

      tile.setWidth( tileWidth );
      tile.setHeight( tileHeight );

      assert( tile.getLeftTopXInFrame() < asps.getFrameWidth() );
      assert( tile.getLeftTopYInFrame() < asps.getFrameHeight() );

      auto& atlasFrame = context[frameIndex];
      printf( "dec:%zu frame %zu tile:(%zu,%zu), %zux%zu -> leftIdx(%zu,%zu), bottom(%zu,%zu) -> %u,%u,%u\n",
              frameIndex, tileIndex, atlasFrame.getTile( tileIndex ).getLeftTopXInFrame(),
              atlasFrame.getTile( tileIndex ).getLeftTopYInFrame(), atlasFrame.getTile( tileIndex ).getWidth(),
              atlasFrame.getTile( tileIndex ).getHeight(), TopLeftPartitionColumn, TopLeftPartitionRow,
              BottomRightPartitionColumn, BottomRightPartitionRow, afti.getTopLeftPartitionIdx( tileIndex ),
              afti.getBottomRightPartitionColumnOffset( tileIndex ),
              afti.getBottomRightPartitionRowOffset( tileIndex ) );
    }
  }

  if ( asps.getAuxiliaryVideoEnabledFlag() ) {
    context[frameIndex].setAuxVideoWidth( ( afti.getAuxiliaryVideoTileRowWidthMinus1() + 1 ) * 64 );
    context[frameIndex].resizeAuxTileLeftTopY( afti.getNumTilesInAtlasFrameMinus1() + 1, 0 );
    context[frameIndex].resizeAuxTileHeight( afti.getNumTilesInAtlasFrameMinus1() + 1, 0 );
    for ( size_t ti = 0; ti <= afti.getNumTilesInAtlasFrameMinus1(); ti++ ) {
      context[frameIndex].setAuxTileHeight( ti, afti.getAuxiliaryVideoTileRowHeight( ti ) * 64 );
      if ( ti < afti.getNumTilesInAtlasFrameMinus1() )
        context[frameIndex].setAuxTileLeftTopY(
            ti + 1, context[frameIndex].getAuxTileLeftTopY( ti ) + context[frameIndex].getAuxTileHeight( ti ) );
    }
    for ( size_t ti = 0; ti <= afti.getNumTilesInAtlasFrameMinus1(); ti++ ) {
      auto& atlasFrame = context[frameIndex];
      printf( "decAux:%zu frame %zu tile:(%zu,%zu), %zux%zu\n", frameIndex, ti, size_t( 0 ),
              atlasFrame.getAuxTileLeftTopY( ti ), atlasFrame.getAuxVideoWidth(), atlasFrame.getAuxTileHeight( ti ) );
    }
  }
  return tileIndex;
}

void PCCDecoder::setConsitantFourCCCode( PCCContext& context, size_t atglIndex ) {
  if ( context.seiIsPresentInReceivedData( NAL_PREFIX_ESEI, COMPONENT_CODEC_MAPPING, atglIndex ) ) {
    auto* sei = static_cast<SEIComponentCodecMapping*>(
        context.getSeiInReceivedData( NAL_PREFIX_ESEI, COMPONENT_CODEC_MAPPING, atglIndex ) );
    consitantFourCCCode_.resize( 256, std::string( "" ) );
    for ( size_t i = 0; i <= sei->getCodecMappingsCountMinus1(); i++ ) {
      auto codecId                  = sei->getCodecId( i );
      consitantFourCCCode_[codecId] = sei->getCodec4cc( codecId );
      printf( "setConsitantFourCCCode: codecId = %3u  fourCCCode = %s \n", codecId,
              consitantFourCCCode_[codecId].c_str() );
    }
  }
}

PCCCodecId PCCDecoder::getCodedCodecId( PCCContext&        context,
                                        const uint8_t      codecCodecId,
                                        const std::string& videoDecoderPath ) {
  auto& sps = context.getVps();
  auto& plt = sps.getProfileTierLevel();
  printf( "getCodedCodecId profileCodecGroupIdc = %d codecCodecId = %u \n", plt.getProfileCodecGroupIdc(),
          codecCodecId );
  fflush( stdout );
  switch ( plt.getProfileCodecGroupIdc() ) {
    case CODEC_GROUP_AVC_PROGRESSIVE_HIGH:
#if defined( USE_JMAPP_VIDEO_CODEC ) && defined( USE_JMLIB_VIDEO_CODEC )
      return videoDecoderPath.empty() ? JMLIB : JMAPP;
#elif defined( USE_JMLIB_VIDEO_CODEC )
      return JMLIB;
#elif defined( USE_JMAPP_VIDEO_CODEC )
      if ( videoDecoderPath.empty() ) {
        fprintf( stderr, "video decoder path not set and JMAPP video codec is used \n" );
        exit( -1 );
      }
      return JMAPP;
#else
      fprintf( stderr, "JM Codec not supported \n" );
      exit( -1 );
#endif
      break;
    case CODEC_GROUP_HEVC_MAIN10:
    case CODEC_GROUP_HEVC444:
#if defined( USE_HMAPP_VIDEO_CODEC ) && defined( USE_HMLIB_VIDEO_CODEC )
      return videoDecoderPath.empty() ? HMLIB : HMAPP;
#elif defined( USE_HMLIB_VIDEO_CODEC )
      return HMLIB;
#elif defined( USE_HMAPP_VIDEO_CODEC )
      if ( videoDecoderPath.empty() ) {
        fprintf( stderr, "video decoder path not set and HMAPP video codec is used \n" );
        exit( -1 );
      }
      return HMAPP;
#else
      fprintf( stderr, "HM Codec not supported \n" );
      exit( -1 );
#endif
      break;
    case CODEC_GROUP_VVC_MAIN10:
#if defined( USE_VTMLIB_VIDEO_CODEC )
      return VTMLIB;
#else
      fprintf( stderr, "VTM Codec not supported \n" );
      exit( -1 );
#endif
      break;
    case CODEC_GROUP_MP4RA:
      if ( consitantFourCCCode_.size() > codecCodecId && !consitantFourCCCode_[codecCodecId].empty() ) {
        std::string codec4cc = consitantFourCCCode_[codecCodecId];
        printf( "=> codecId = %u => codec4cc = %s \n", codecCodecId, codec4cc.c_str() );
        if ( codec4cc.compare( "avc3" ) == 0 ) {
#if defined( USE_JMAPP_VIDEO_CODEC ) && defined( USE_JMLIB_VIDEO_CODEC )
          return videoDecoderPath.empty() ? JMLIB : JMAPP;
#elif defined( USE_JMLIB_VIDEO_CODEC )
          return JMLIB;
#elif defined( USE_JMAPP_VIDEO_CODEC )
          if ( videoDecoderPath.empty() ) {
            fprintf( stderr, "video decoder path not set and JMAPP video codec is used \n" );
            exit( -1 );
          }
          return JMAPP;
#else
          fprintf( stderr, "JM Codec not supported \n" );
          exit( -1 );
#endif
        } else if ( codec4cc.compare( "hev1" ) == 0 ) {
#if defined( USE_HMAPP_VIDEO_CODEC ) && defined( USE_HMLIB_VIDEO_CODEC )
          return videoDecoderPath.empty() ? HMLIB : HMAPP;
#elif defined( USE_HMLIB_VIDEO_CODEC )
          return HMLIB;
#elif defined( USE_HMAPP_VIDEO_CODEC )
          if ( videoDecoderPath.empty() ) {
            fprintf( stderr, "video decoder path not set and HMAPP video codec is used \n" );
            exit( -1 );
          }
          return HMAPP;
#else
          fprintf( stderr, "HM Codec not supported \n" );
          exit( -1 );
#endif
        } else if ( codec4cc.compare( "svc1" ) == 0 ) {
#if defined( USE_SHMAPP_VIDEO_CODEC )
          if ( videoDecoderPath.empty() ) {
            fprintf( stderr, "video decoder path not set and SHMAPP video codec is used \n" );
            exit( -1 );
          }
          return SHMAPP;
#else
          fprintf( stderr, "SHM Codec not supported \n" );
          exit( -1 );
#endif
        } else if ( codec4cc.compare( "vvi1" ) == 0 ) {
#if defined( USE_VTMLIB_VIDEO_CODEC )
          return VTMLIB;
#else
          fprintf( stderr, "VTM Codec not supported \n" );
          exit( -1 );
#endif
        } else {
          fprintf( stderr, "CODEC_GROUP_MP4RA but codec4cc \"%s\" not supported \n", codec4cc.c_str() );
          exit( -1 );
        }
      } else {
        fprintf( stderr, "CODEC_GROUP_MP4RA but component codec mapping SEI not present or codec index = %u not set \n",
                 codecCodecId );
        exit( -1 );
      }
      break;
    default:
      fprintf( stderr, "ProfileCodecGroupIdc = %d not supported \n", plt.getProfileCodecGroupIdc() );
      exit( -1 );
      break;
  }
  return PCCCodecId::UNKNOWN_CODEC;
}
