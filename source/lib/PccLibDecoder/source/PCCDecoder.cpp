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

int PCCDecoder::decode( PCCContext& context, PCCGroupOfFrames& reconstructs, int32_t atlasIndex = 0 ) {
  if ( params_.nbThread_ > 0 ) { tbb::task_scheduler_init init( static_cast<int>( params_.nbThread_ ) ); }
  createPatchFrameDataStructure( context );

  // Note JR: logger examples
#if 1
  logger_->trace( LOG_DESCR, "test descr = %d \n", 10 );
  logger_->trace( LOG_DESCR, "test descr = %d \n", 11 );
  logger_->trace( LOG_ATLAS, "test atlas = %d \n", 12 );
  logger_->trace( LOG_ATLAS, "test atlas = %d \n", 13 );
  logger_->trace( LOG_FRAME, "test frame = %d \n", 14 );
  logger_->trace( LOG_FRAME, "test frame = %d \n", 15 );
  logger_->trace( LOG_TILES, "test tiles = %d \n", 16 );
  logger_->trace( LOG_TILES, "test tiles = %d \n", 17 );
  logger_->trace( LOG_TRACE, "test trace = %d \n", 18 );
  logger_->trace( LOG_TRACE, "test trace = %d \n", 19 );
#endif

  PCCVideoDecoder   videoDecoder;
  std::stringstream path;
  auto&             sps          = context.getVps();
  auto&             ai           = sps.getAttributeInformation( atlasIndex );
  auto&             oi           = sps.getOccupancyInformation( atlasIndex );
  auto&             gi           = sps.getGeometryInformation( atlasIndex );
  auto&             asps         = context.getAtlasSequenceParameterSet( 0 );
  size_t            pcFrameCount = context.size();
  path << removeFileExtension( params_.compressedStreamPath_ ) << "_dec_GOF" << sps.getV3CParameterSetId() << "_";
  auto&        plt                      = sps.getProfileTierLevel();
  const size_t mapCount                 = sps.getMapCountMinus1( atlasIndex ) + 1;
  auto&        videoBitstreamOM         = context.getVideoBitstream( VIDEO_OCCUPANCY );
  int          decodedBitDepthOM        = 8;
  bool         isOCM444                 = false;
  bool         isGeometry444            = false;
  bool         isAuxiliarygeometry444   = false;
  bool         isAttributes444          = plt.getProfileCodecGroupIdc() == CODEC_GROUP_HEVC444;
  bool         isAuxiliaryAttributes444 = plt.getProfileCodecGroupIdc() == CODEC_GROUP_HEVC444;

  PCCCodecId occupancyCodecId = (PCCCodecId)oi.getOccupancyCodecId();
  PCCCodecId geometryCodecId  = (PCCCodecId)gi.getGeometryCodecId();
  PCCCodecId attributeCodecId;
  for ( uint32_t i = 0; i < ai.getAttributeCount(); i++ ) {
    attributeCodecId = (PCCCodecId)ai.getAttributeCodecId( i );
  }
  printf( "CodecId occupancyCodecId = %d geometry = %d attribute = %d \n", (int)occupancyCodecId, (int)geometryCodecId,
          (int)attributeCodecId );

  printf( " Decode O size = %zu \n", videoBitstreamOM.size() );
  fflush( stdout );
  videoDecoder.decompress( context.getVideoOccupancyMap(),      //  video
                           path.str(),                          // path
                           context.size(),                      // frameCount
                           videoBitstreamOM,                    // bitstream
                           params_.videoDecoderOccupancyPath_,  // decoderPath
                           occupancyCodecId,                    // codecId
                           context,                             // contexts
                           decodedBitDepthOM,                   // bitDepth
                           params_.keepIntermediateFiles_,      // keepIntermediateFiles
                           isOCM444,                            // use444CodecIo
                           false,                               // patchColorSubsampling
                           "",                                  // inverseColorSpaceConversionConfig
                           "" );                                // colorSpaceConversionPath
  // converting the decoded bitdepth to the nominal bitdepth
  context.getVideoOccupancyMap().convertBitdepth( decodedBitDepthOM, oi.getOccupancy2DBitdepthMinus1() + 1,
                                                  oi.getOccupancyMSBAlignFlag() );

  if ( sps.getMultipleMapStreamsPresentFlag( atlasIndex ) ) {
    context.getVideoGeometryMultiple().resize( sps.getMapCountMinus1( atlasIndex ) + 1 );
    size_t totalGeoSize = 0;
    for ( uint32_t mapIndex = 0; mapIndex < sps.getMapCountMinus1( atlasIndex ) + 1; mapIndex++ ) {
      std::cout << "*******Video Decoding: Geometry[" << mapIndex << "] ********" << std::endl;
      int   decodedBitDepth = gi.getGeometry2dBitdepthMinus1() + 1;  // this should be extracted from the bitstream
      auto  geometryIndex   = static_cast<PCCVideoType>( VIDEO_GEOMETRY_D0 + mapIndex );
      auto& videoBitstream  = context.getVideoBitstream( geometryIndex );
      videoDecoder.decompress( context.getVideoGeometryMultiple()[mapIndex], path.str(), pcFrameCount, videoBitstream,
                               params_.videoDecoderGeometryPath_, geometryCodecId, context, decodedBitDepth,
                               params_.keepIntermediateFiles_, isGeometry444 );
      context.getVideoGeometryMultiple()[mapIndex].convertBitdepth(
          decodedBitDepth, gi.getGeometry2dBitdepthMinus1() + 1, gi.getGeometryMSBAlignFlag() );
      std::cout << "geometry D" << mapIndex << " video ->" << videoBitstream.size() << " B" << std::endl;
      totalGeoSize += videoBitstream.size();
    }
    std::cout << "total geometry video ->" << totalGeoSize << " B" << std::endl;
  } else {
    std::cout << "*******Video Decoding: Geometry ********" << std::endl;
    int   decodedBitDepthGeo = gi.getGeometry2dBitdepthMinus1() + 1;
    auto& videoBitstream     = context.getVideoBitstream( VIDEO_GEOMETRY );

    printf( " Decode G size = %zu \n", videoBitstream.size() );
    fflush( stdout );
    videoDecoder.decompress( context.getVideoGeometryMultiple()[0],  //
                             path.str(),                             //
                             context.size() * mapCount,              //
                             videoBitstream,                         //
                             params_.videoDecoderGeometryPath_,      //
                             geometryCodecId,                        //
                             context,                                //
                             decodedBitDepthGeo,                     //
                             params_.keepIntermediateFiles_,         //
                             isGeometry444 );
    context.getVideoGeometryMultiple()[0].convertBitdepth( decodedBitDepthGeo, gi.getGeometry2dBitdepthMinus1() + 1,
                                                           gi.getGeometryMSBAlignFlag() );
    std::cout << "geometry video ->" << videoBitstream.size() << " B" << std::endl;
  }

  if ( asps.getRawPatchEnabledFlag() && asps.getAuxiliaryVideoEnabledFlag() &&
       sps.getAuxiliaryVideoPresentFlag( atlasIndex ) ) {
    std::cout << "*******Video Decoding: Aux Geometry ********" << std::endl;
    int   decodedBitDepthMP = gi.getGeometry2dBitdepthMinus1() + 1;
    auto& videoBitstreamMP  = context.getVideoBitstream( VIDEO_GEOMETRY_RAW );
    videoDecoder.decompress( context.getVideoRawPointsGeometry(), path.str(), pcFrameCount, videoBitstreamMP,
                             params_.videoDecoderGeometryPath_, geometryCodecId, context, decodedBitDepthMP,
                             params_.keepIntermediateFiles_, isAuxiliarygeometry444 );
    context.getVideoRawPointsGeometry().convertBitdepth( decodedBitDepthMP, gi.getGeometry2dBitdepthMinus1() + 1,
                                                         gi.getGeometryMSBAlignFlag() );
    std::cout << " raw points geometry -> " << videoBitstreamMP.size() << " B " << endl;
  }

  if ( ai.getAttributeCount() > 0 ) {
    for ( int attrIndex = 0; attrIndex < sps.getAttributeInformation( atlasIndex ).getAttributeCount();
          attrIndex++ ) {  // right now we only have one attribute, this should be generalized
      int decodedBitdepthAttribute   = ai.getAttribute2dBitdepthMinus1( attrIndex ) + 1;
      int decodedBitdepthAttributeMP = ai.getAttribute2dBitdepthMinus1( attrIndex ) + 1;
      for ( int attrPartitionIndex = 0;
            attrPartitionIndex <
            sps.getAttributeInformation( atlasIndex ).getAttributeDimensionPartitionsMinus1( attrIndex ) + 1;
            attrPartitionIndex++ ) {  // right now we have only one partition, this should be generalized
        if ( sps.getMultipleMapStreamsPresentFlag( atlasIndex ) ) {
          int sizeTextureVideo = 0;
          context.getVideoTextureMultiple().resize( sps.getMapCountMinus1( atlasIndex ) + 1 );
          // this allocation is considering only one attribute, with a single partition, but multiple streams
          for ( uint32_t mapIndex = 0; mapIndex < sps.getMapCountMinus1( atlasIndex ) + 1; mapIndex++ ) {
            // decompress T[mapIndex]
            std::cout << "*******Video Decoding: Attribute [" << mapIndex << "] ********" << std::endl;
            auto textureIndex =
                static_cast<PCCVideoType>( VIDEO_TEXTURE_T0 + attrPartitionIndex + MAX_NUM_ATTR_PARTITIONS * mapIndex );
            auto& videoBitstream = context.getVideoBitstream( textureIndex );
            videoDecoder.decompress( context.getVideoTextureMultiple()[mapIndex], path.str(), context.size(),
                                     videoBitstream, params_.videoDecoderAttributePath_, attributeCodecId, context,
                                     ai.getAttribute2dBitdepthMinus1( 0 ) + 1, params_.keepIntermediateFiles_,
                                     isAttributes444, params_.patchColorSubsampling_,
                                     params_.inverseColorSpaceConversionConfig_, params_.colorSpaceConversionPath_ );
            std::cout << "texture T" << mapIndex << " video ->" << videoBitstream.size() << " B" << std::endl;
            sizeTextureVideo += videoBitstream.size();
          }
          std::cout << "texture    video ->" << sizeTextureVideo << " B" << std::endl;
        } else {
          std::cout << "*******Video Decoding: Attribute ********" << std::endl;
          auto  textureIndex   = static_cast<PCCVideoType>( VIDEO_TEXTURE + attrPartitionIndex );
          auto& videoBitstream = context.getVideoBitstream( textureIndex );
          printf( "call videoDecoder.decompress()::context.getVideoTexture() \n" );
          printf( " Decode T size = %zu \n", videoBitstream.size() );
          fflush( stdout );
          videoDecoder.decompress( context.getVideoTextureMultiple()[0],        // video,
                                   path.str(),                                  // path,
                                   context.size() * mapCount,                   // frameCount,
                                   videoBitstream,                              // bitstream,
                                   params_.videoDecoderAttributePath_,          // decoderPath,
                                   attributeCodecId,                            // attributeCodecId
                                   context,                                     // contexts,
                                   decodedBitdepthAttribute,                    // bitDepth,
                                   params_.keepIntermediateFiles_,              // keepIntermediateFiles
                                   isAttributes444,                             // isAttributes444
                                   params_.patchColorSubsampling_,              // patchColorSubsampling
                                   params_.inverseColorSpaceConversionConfig_,  // inverseColorSpaceConversionConfig_
                                   params_.colorSpaceConversionPath_ );
          std::cout << "texture video  ->" << videoBitstream.size() << " B" << std::endl;
        }

        if ( asps.getRawPatchEnabledFlag() && asps.getAuxiliaryVideoEnabledFlag() &&
             sps.getAuxiliaryVideoPresentFlag( atlasIndex ) ) {
          std::cout << "*******Video Decoding: Aux Attribute ********" << std::endl;
          auto  textureIndex     = static_cast<PCCVideoType>( VIDEO_TEXTURE_RAW + attrPartitionIndex );
          auto& videoBitstreamMP = context.getVideoBitstream( textureIndex );
          videoDecoder.decompress( context.getVideoRawPointsTexture(), path.str(), pcFrameCount, videoBitstreamMP,
                                   params_.videoDecoderAttributePath_, attributeCodecId, context,
                                   decodedBitdepthAttributeMP, params_.keepIntermediateFiles_, isAuxiliaryAttributes444,
                                   false, params_.inverseColorSpaceConversionConfig_,
                                   params_.colorSpaceConversionPath_ );
          // generateRawPointsTexturefromVideo( context, reconstructs );
          std::cout << " raw points texture -> " << videoBitstreamMP.size() << " B" << endl;
        }
      }
    }
  }

  reconstructs.setFrameCount( pcFrameCount );
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

  printf( "generate point cloud of %zu frames \n", pcFrameCount );
  fflush( stdout );
  for ( size_t frameIdx = 0; frameIdx < pcFrameCount; frameIdx++ ) {
    // All video have been decoded, start reconsctruction processes
    if ( asps.getRawPatchEnabledFlag() && asps.getAuxiliaryVideoEnabledFlag() &&
         sps.getAuxiliaryVideoPresentFlag( atlasIndex ) ) {
      printf( "generateRawPointsGeometryfromVideo \n" );
      fflush( stdout );
      generateRawPointsGeometryfromVideo( context, frameIdx );

      if ( ai.getAttributeCount() > 0 ) {
        for ( int attrIndex = 0; attrIndex < sps.getAttributeInformation( atlasIndex ).getAttributeCount();
              attrIndex++ ) {  // right now we only have one attribute, this should be generalized
          for ( int attrPartitionIndex = 0;
                attrPartitionIndex <
                sps.getAttributeInformation( atlasIndex ).getAttributeDimensionPartitionsMinus1( attrIndex ) + 1;
                attrPartitionIndex++ ) {  // right now we have only one partition,
                                          // this should be generalized
            printf( "generateRawPointsTexturefromVideo attrIndex = %d attrPartitionIndex = %d \n", attrIndex,
                    attrPartitionIndex );
            fflush( stdout );
            generateRawPointsTexturefromVideo( context, frameIdx );
          }
        }
      }
    }  // getAuxiliaryVideoEnabledFlag()

    context.setOccupancyPrecision( sps.getFrameWidth( atlasIndex ) / context.getVideoOccupancyMap().getWidth() );
    GeneratePointCloudParameters gpcParams;
    GeneratePointCloudParameters ppSEIParams;
    setGeneratePointCloudParameters( gpcParams, context );
    setPostProcessingSeiParameters( ppSEIParams, context );

    auto&                 reconstruct = reconstructs[frameIdx];
    std::vector<uint32_t> partition;
    // Decode point cloud
    printf( "call generatePointCloud() \n" );
    std::vector<size_t> accTilePointCount;
    accTilePointCount.resize( ai.getAttributeCount(), 0 );
    for ( size_t tileIdx = 0; tileIdx < context[frameIdx].getNumTilesInAtlasFrame(); tileIdx++ ) {
      std::cout << "Processing frame " << frameIdx << " tile " << tileIdx << std::endl;
      auto& tile = context[frameIdx].getTile( tileIdx );
      if ( !ppSEIParams.pbfEnableFlag_ ) {
        generateOccupancyMap( tile, context.getVideoOccupancyMap().getFrame( tile.getFrameIndex() ),
                              context.getOccupancyPrecision(), oi.getLossyOccupancyCompressionThreshold(),
                              asps.getEomPatchEnabledFlag() );
      }
      generateBlockToPatchFromOccupancyMapVideo(
          context, tile, frameIdx, context.getVideoOccupancyMap().getFrame( frameIdx ),
          size_t( 1 ) << asps.getLog2PatchPackingBlockSize(), context.getOccupancyPrecision() );

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
      }
      for ( size_t attIdx = 0; attIdx < ai.getAttributeCount(); attIdx++ ) {
        printf( "start colorPointCloud attIdx = %zu / %u ] \n", attIdx, ai.getAttributeCount() );
        fflush( stdout );
        size_t updatedPointCount  = colorPointCloud( reconstruct, context, tile, absoluteT1List[attIdx],
                                                    sps.getMultipleMapStreamsPresentFlag( atlasIndex ),
                                                    ai.getAttributeCount(), accTilePointCount[attIdx], gpcParams );
        accTilePointCount[attIdx] = updatedPointCount;
      }
    }  // tile

    // Post-Processing
    TRACE_PATCH( "Post-Processing: postprocessSmoothing = %zu pbfEnableFlag = %d \n",
                 params_.postprocessSmoothingFilter_, ppSEIParams.pbfEnableFlag_ );
    if ( ppSEIParams.flagGeometrySmoothing_ ) {
      PCCPointSet3 tempFrameBuffer = reconstruct;
      if ( ppSEIParams.gridSmoothing_ ) {
        smoothPointCloudPostprocess( reconstruct, params_.colorTransform_, ppSEIParams, partition );
      }
      if ( !ppSEIParams.pbfEnableFlag_ ) {
        // These are different attribute transfer functions
        if ( params_.postprocessSmoothingFilter_ == 1 || params_.postprocessSmoothingFilter_ == 5 ) {
          TRACE_PATCH( " transferColors16bitBP \n" );
          tempFrameBuffer.transferColors16bitBP( reconstruct, params_.postprocessSmoothingFilter_, int32_t( 0 ),
                                                 isAttributes444, 8, 1, true, true, true, false, 4, 4, 1000, 1000,
                                                 1000 * 256, 1000 * 256 );  // jkie: let's make it general
        } else if ( params_.postprocessSmoothingFilter_ == 2 ) {
          TRACE_PATCH( " transferColorWeight \n" );
          tempFrameBuffer.transferColorWeight( reconstruct, 0.1 );
        } else if ( params_.postprocessSmoothingFilter_ == 3 ) {
          TRACE_PATCH( " transferColorsFilter3 \n" );
          tempFrameBuffer.transferColorsFilter3( reconstruct, int32_t( 0 ), isAttributes444 );
        } else if ( params_.postprocessSmoothingFilter_ == 7 || params_.postprocessSmoothingFilter_ == 9 ) {
          TRACE_PATCH( " transferColorsFilter3 \n" );
          tempFrameBuffer.transferColorsBackward16bitBP( reconstruct, params_.postprocessSmoothingFilter_, int32_t( 0 ),
                                                         isAttributes444, 8, 1, true, true, true, false, 4, 4, 1000,
                                                         1000, 1000 * 256, 1000 * 256 );
        }
      }
    }
    if ( ppSEIParams.flagColorSmoothing_ ) {
      TRACE_PATCH( " colorSmoothing \n" );
      colorSmoothing( reconstruct, params_.colorTransform_, ppSEIParams );
    }
    if ( !isAttributes444 ) {  // lossy: convert 16-bit yuv444 to 8-bit RGB444
      TRACE_PATCH( "lossy: convert 16-bit yuv444 to 8-bit RGB444 (convertYUV16ToRGB8) \n" );
      reconstruct.convertYUV16ToRGB8();
    } else {  // lossless: copy 16-bit RGB to 8-bit RGB
      TRACE_PATCH( "lossy: lossless: copy 16-bit RGB to 8-bit RGB (copyRGB16ToRGB8) \n" );
      reconstruct.copyRGB16ToRGB8();
    }
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
      patch.getPointLocalReconstructionMode() = plrd.getModeMinus1() + 1;
    } else {
      patch.getPointLocalReconstructionMode() = 0;
    }
    TRACE_PATCH( "  ModePatch: Present = %d ModeMinus1 = %2d \n", plrd.getPresentFlag(),
                 plrd.getPresentFlag() ? (int32_t)plrd.getModeMinus1() : -1 );
  } else {
    for ( size_t v0 = 0; v0 < plrd.getBlockToPatchMapHeight(); ++v0 ) {
      for ( size_t u0 = 0; u0 < plrd.getBlockToPatchMapWidth(); ++u0 ) {
        size_t index = v0 * plrd.getBlockToPatchMapWidth() + u0;
        if ( plrd.getBlockPresentFlag( index ) ) {
          patch.getPointLocalReconstructionMode( u0, v0 ) = plrd.getBlockModeMinus1( index ) + 1;
        } else {
          patch.getPointLocalReconstructionMode( u0, v0 ) = 0;
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

void PCCDecoder::setPostProcessingSeiParameters( GeneratePointCloudParameters& params, PCCContext& context ) {
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
  if ( context.seiIsPresent( NAL_PREFIX_ESEI, GEOMETRY_SMOOTHING ) ) {
    auto* sei = static_cast<SEIGeometrySmoothing*>( context.getSei( NAL_PREFIX_ESEI, GEOMETRY_SMOOTHING ) );
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
  if ( context.seiIsPresent( NAL_PREFIX_ESEI, OCCUPANCY_SYNTHESIS ) ) {
    auto* sei = static_cast<SEIOccupancySynthesis*>( context.getSei( NAL_PREFIX_ESEI, OCCUPANCY_SYNTHESIS ) );
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
  if ( context.seiIsPresent( NAL_PREFIX_ESEI, ATTRIBUTE_SMOOTHING ) ) {
    auto* sei = static_cast<SEIAttributeSmoothing*>( context.getSei( NAL_PREFIX_ESEI, ATTRIBUTE_SMOOTHING ) );
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
  params.thresholdLossyOM_              = static_cast<size_t>( oi.getLossyOccupancyCompressionThreshold() );
  params.removeDuplicatePoints_         = asps.getAspsVpccExtension().getRemoveDuplicatePointEnableFlag();
  params.pointLocalReconstruction_      = asps.getPLREnabledFlag();
  params.mapCountMinus1_                = sps.getMapCountMinus1( atlasIndex );
  params.singleMapPixelInterleaving_    = asps.getPixelDeinterleavingFlag();
  params.useAdditionalPointsPatch_      = asps.getRawPatchEnabledFlag();
  params.enhancedOccupancyMapCode_      = asps.getEomPatchEnabledFlag();
  params.EOMFixBitCount_                = asps.getEomFixBitCountMinus1() + 1;
  params.geometry3dCoordinatesBitdepth_ = gi.getGeometry3dCoordinatesBitdepthMinus1() + 1;
  params.geometryBitDepth3D_            = gi.getGeometry3dCoordinatesBitdepthMinus1() + 1;
}

void PCCDecoder::setGeneratePointCloudParameters( GeneratePointCloudParameters& params, PCCContext& context ) {
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
  if ( context.seiIsPresent( NAL_PREFIX_ESEI, GEOMETRY_SMOOTHING ) ) {
    auto* sei = static_cast<SEIGeometrySmoothing*>( context.getSei( NAL_PREFIX_ESEI, GEOMETRY_SMOOTHING ) );
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
  if ( context.seiIsPresent( NAL_PREFIX_ESEI, OCCUPANCY_SYNTHESIS ) ) {
    auto* sei = static_cast<SEIOccupancySynthesis*>( context.getSei( NAL_PREFIX_ESEI, OCCUPANCY_SYNTHESIS ) );
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
  if ( context.seiIsPresent( NAL_PREFIX_ESEI, ATTRIBUTE_SMOOTHING ) ) {
    auto* sei = static_cast<SEIAttributeSmoothing*>( context.getSei( NAL_PREFIX_ESEI, ATTRIBUTE_SMOOTHING ) );
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
  params.thresholdLossyOM_              = static_cast<size_t>( oi.getLossyOccupancyCompressionThreshold() );
  params.removeDuplicatePoints_         = asps.getAspsVpccExtension().getRemoveDuplicatePointEnableFlag();
  params.pointLocalReconstruction_      = asps.getPLREnabledFlag();
  params.mapCountMinus1_                = sps.getMapCountMinus1( atlasIndex );
  params.singleMapPixelInterleaving_    = asps.getPixelDeinterleavingFlag();
  params.useAdditionalPointsPatch_      = asps.getRawPatchEnabledFlag();
  params.useAuxSeperateVideo_           = asps.getAuxiliaryVideoEnabledFlag();
  params.enhancedOccupancyMapCode_      = asps.getEomPatchEnabledFlag();
  params.EOMFixBitCount_                = asps.getEomFixBitCountMinus1() + 1;
  params.geometry3dCoordinatesBitdepth_ = gi.getGeometry3dCoordinatesBitdepthMinus1() + 1;
  params.geometryBitDepth3D_            = gi.getGeometry3dCoordinatesBitdepthMinus1() + 1;
}

void PCCDecoder::createPatchFrameDataStructure( PCCContext& context ) {
  TRACE_PATCH( "createPatchFrameDataStructure GOP start \n" );
  auto& atglulist = context.getAtlasTileLayerList();
  // partition information derivation
  setTilePartitionSizeAfti( context );
  size_t frameCount = 0;
  for ( size_t i = 0; i < atglulist.size(); i++ ) {
    frameCount = std::max( frameCount, ( context.calculateAFOCval( atglulist, i ) + 1 ) );
  }
  context.resize( frameCount );
  setPointLocalReconstruction( context );
  for ( size_t atglOrder = 0; atglOrder < atglulist.size(); atglOrder++ ) {
    createPatchFrameDataStructure( context, atglOrder );
  }

  // sei hash
  bool bHashSeiIsPresent = context.seiIsPresent( NAL_SUFFIX_NSEI, DECODED_ATLAS_INFORMATION_HASH );
  if ( bHashSeiIsPresent ) {
    assert( context.getSeiHash().size() == frameCount );
    for ( size_t fi = 0; fi < frameCount; fi++ ) { createHashInformation( context, fi ); }
  }
}

void PCCDecoder::createPatchFrameDataStructure( PCCContext& context, size_t atglOrder ) {
  TRACE_PATCH( "createPatchFrameDataStructure Tile %zu \n", atglOrder );
  auto&  sps        = context.getVps();
  size_t atlasIndex = context.getAtlasIndex();
  auto&  gi         = sps.getGeometryInformation( atlasIndex );
  auto&  atlu       = context.getAtlasTileLayer( atglOrder );
  auto&  ath        = atlu.getHeader();
  // the header indicates the structures used
  auto& afps  = context.getAtlasFrameParameterSet( ath.getAtlasFrameParameterSetId() );
  auto& asps  = context.getAtlasSequenceParameterSet( afps.getAtlasSequenceParameterSetId() );
  auto& atgdu = atlu.getDataUnit();

  // current tile position derivation
  size_t frameIndex = atlu.getAtlasFrmOrderCntVal();
  size_t tileIndex  = setTileSizeAndLocation( context, frameIndex, ath );  // width,height,leftTopPosition

  printf( "createPatchFrameDataStructure Frame = %zu Tiles = %zu atlasIndex = %zu atglOrder %zu \n", frameIndex,
          tileIndex, context.getAtlasIndex(), atglOrder );
  fflush( stdout );

  PCCFrameContext& tile = context[frameIndex].getTile( tileIndex );
  tile.setFrameIndex( atlu.getAtlasFrmOrderCntVal() );
  tile.setTileIndex( tileIndex );
  tile.setAtlIndex( atglOrder );
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
  auto&        patches                 = tile.getPatches();
  auto&        pcmPatches              = tile.getRawPointsPatches();
  auto&        eomPatches              = tile.getEomPatches();
  int64_t      prevSizeU0              = 0;
  int64_t      prevSizeV0              = 0;
  int64_t      prevPatchSize2DXInPixel = 0;
  int64_t      prevPatchSize2DYInPixel = 0;
  int64_t      predIndex               = 0;
  const size_t minLevel                = pow( 2., ath.getPosMinDQuantizer() );
  size_t       numRawPatches           = 0;
  size_t       numNonRawPatch          = 0;
  size_t       numEomPatch             = 0;
  PCCTileType  tileType                = ath.getType();
  size_t       patchCount              = atgdu.getPatchCount();
  for ( size_t i = 0; i < patchCount; i++ ) {
    PCCPatchType currPatchType = getPatchType( tileType, atgdu.getPatchMode( i ) );
    if ( currPatchType == RAW_PATCH ) {
      numRawPatches++;
    } else if ( currPatchType == EOM_PATCH ) {
      numEomPatch++;
    }
  }
  numNonRawPatch = patchCount - numRawPatches - numEomPatch;
  eomPatches.reserve( numEomPatch );
  patches.resize( numNonRawPatch );
  pcmPatches.resize( numRawPatches );
  TRACE_PATCH( "Patches size                      = %zu \n", patches.size() );
  TRACE_PATCH( "non-regular Patches(raw, eom)     = %zu, %zu \n", numRawPatches, numEomPatch );
  TRACE_PATCH( "Tile Type                         = %zu (0.P_TILE 1.I_TILE 2.SKIP_TILE)\n", (size_t)ath.getType() );

  // TRACE_PATCH( "OccupancyPackingBlockSize           = %d \n", context.getOccupancyPackingBlockSize() );
  size_t  totalNumberOfRawPoints = 0;
  size_t  totalNumberOfEomPoints = 0;
  size_t  patchIndex             = 0;
  int32_t packingBlockSize       = 1 << asps.getLog2PatchPackingBlockSize();
  int32_t quantizerSizeX         = 1 << ath.getPatchSizeXinfoQuantizer();
  int32_t quantizerSizeY         = 1 << ath.getPatchSizeYinfoQuantizer();
  tile.setLog2PatchQuantizerSizeX( ath.getPatchSizeXinfoQuantizer() );
  tile.setLog2PatchQuantizerSizeY( ath.getPatchSizeYinfoQuantizer() );
  for ( patchIndex = 0; patchIndex < patchCount; patchIndex++ ) {
    auto&        pid           = atgdu.getPatchInformationData( patchIndex );
    PCCPatchType currPatchType = getPatchType( tileType, atgdu.getPatchMode( patchIndex ) );
    if ( currPatchType == INTRA_PATCH ) {
      auto& patch                    = patches[patchIndex];
      patch.getOccupancyResolution() = size_t( 1 ) << asps.getLog2PatchPackingBlockSize();
      auto& pdu                      = pid.getPatchDataUnit();
      patch.getU0()                  = pdu.get2dPosX();
      patch.getV0()                  = pdu.get2dPosY();
      patch.getU1()                  = pdu.get3dOffsetU();
      patch.getV1()                  = pdu.get3dOffsetV();

      bool lodEnableFlag = pdu.getLodEnableFlag();
      if ( lodEnableFlag ) {
        patch.setLodScaleX( pdu.getLodScaleXMinus1() + 1 );
        patch.setLodScaleYIdc( pdu.getLodScaleYIdc() + ( patch.getLodScaleX() > 1 ? 1 : 2 ) );
      } else {
        patch.setLodScaleX( 1 );
        patch.setLodScaleYIdc( 1 );
      }
      patch.getSizeD() = pdu.get3dRangeD() * minLevel;
      if ( asps.getPatchSizeQuantizerPresentFlag() ) {
        patch.setPatchSize2DXInPixel( pdu.get2dSizeXMinus1() * quantizerSizeX + 1 );
        patch.setPatchSize2DYInPixel( pdu.get2dSizeYMinus1() * quantizerSizeY + 1 );
        patch.getSizeU0() =
            ceil( static_cast<double>( patch.getPatchSize2DXInPixel() ) / static_cast<double>( packingBlockSize ) );
        patch.getSizeV0() =
            ceil( static_cast<double>( patch.getPatchSize2DYInPixel() ) / static_cast<double>( packingBlockSize ) );
      } else {
        patch.getSizeU0() = pdu.get2dSizeXMinus1() + 1;
        patch.getSizeV0() = pdu.get2dSizeYMinus1() + 1;
      }
      patch.getPatchOrientation() = pdu.getOrientationIndex();
      patch.setViewId( pdu.getProjectionId() );
      TRACE_PATCH( "patch %zu / %zu: Intra \n", patchIndex, patchCount );
      const size_t max3DCoordinate = size_t( 1 ) << ( gi.getGeometry3dCoordinatesBitdepthMinus1() + 1 );
      if ( patch.getProjectionMode() == 0 ) {
        patch.getD1() = static_cast<int32_t>( pdu.get3dOffsetD() ) * minLevel;
      } else {
        if ( static_cast<int>( asps.getExtendedProjectionEnabledFlag() ) == 0 ) {
          patch.getD1() = max3DCoordinate - static_cast<int32_t>( pdu.get3dOffsetD() ) * minLevel;
        } else {
          patch.getD1() = ( max3DCoordinate ) - static_cast<int32_t>( pdu.get3dOffsetD() ) * minLevel;
        }
      }
      prevSizeU0              = patch.getSizeU0();
      prevSizeV0              = patch.getSizeV0();
      prevPatchSize2DXInPixel = patch.getPatchSize2DXInPixel();
      prevPatchSize2DYInPixel = patch.getPatchSize2DYInPixel();
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
    } else if ( currPatchType == INTER_PATCH ) {
      auto& patch                    = patches[patchIndex];
      patch.getOccupancyResolution() = size_t( 1 ) << asps.getLog2PatchPackingBlockSize();
      auto& ipdu                     = pid.getInterPatchDataUnit();

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
          " Lod = %u,%u\n",
          patch.getRefAtlasFrameIndex(), refPOC, patch.getBestMatchIdx(),
          context.getFrame( refPOC ).getTile( tileIndex ).getPatches().size(), refPatch.getU0(), refPatch.getV0(),
          refPatch.getU1(), refPatch.getV1(), refPatch.getSizeU0(), refPatch.getSizeV0(), refPatch.getSizeD(),
          refPatch.getLodScaleX(), refPatch.getLodScaleY() );
      patch.getProjectionMode()   = refPatch.getProjectionMode();
      patch.getU0()               = ipdu.get2dPosX() + refPatch.getU0();
      patch.getV0()               = ipdu.get2dPosY() + refPatch.getV0();
      patch.getPatchOrientation() = refPatch.getPatchOrientation();
      patch.getU1()               = ipdu.get3dOffsetU() + refPatch.getU1();
      patch.getV1()               = ipdu.get3dOffsetV() + refPatch.getV1();
      if ( asps.getPatchSizeQuantizerPresentFlag() ) {
        patch.setPatchSize2DXInPixel( refPatch.getPatchSize2DXInPixel() + ( ipdu.get2dDeltaSizeX() ) * quantizerSizeX );
        patch.setPatchSize2DYInPixel( refPatch.getPatchSize2DYInPixel() + ( ipdu.get2dDeltaSizeY() ) * quantizerSizeY );
        patch.getSizeU0() =
            ceil( static_cast<double>( patch.getPatchSize2DXInPixel() ) / static_cast<double>( packingBlockSize ) );
        patch.getSizeV0() =
            ceil( static_cast<double>( patch.getPatchSize2DYInPixel() ) / static_cast<double>( packingBlockSize ) );
      } else {
        patch.getSizeU0() = ipdu.get2dDeltaSizeX() + refPatch.getSizeU0();
        patch.getSizeV0() = ipdu.get2dDeltaSizeY() + refPatch.getSizeV0();
      }
      patch.getNormalAxis()            = refPatch.getNormalAxis();
      patch.getTangentAxis()           = refPatch.getTangentAxis();
      patch.getBitangentAxis()         = refPatch.getBitangentAxis();
      patch.getAxisOfAdditionalPlane() = refPatch.getAxisOfAdditionalPlane();
      const size_t max3DCoordinate     = size_t( 1 ) << ( gi.getGeometry3dCoordinatesBitdepthMinus1() + 1 );
      if ( patch.getProjectionMode() == 0 ) {
        patch.getD1() = ( ipdu.get3dOffsetD() + ( refPatch.getD1() / minLevel ) ) * minLevel;
      } else {
        if ( static_cast<int>( asps.getExtendedProjectionEnabledFlag() ) == 0 ) {
          patch.getD1() = max3DCoordinate -
                          ( ipdu.get3dOffsetD() + ( ( max3DCoordinate - refPatch.getD1() ) / minLevel ) ) * minLevel;
        } else {
          patch.getD1() = ( max3DCoordinate ) -
                          ( ipdu.get3dOffsetD() + ( ( (max3DCoordinate)-refPatch.getD1() ) / minLevel ) ) * minLevel;
        }
      }
      const int64_t delta_DD = ipdu.get3dRangeD();
      size_t        prevDD   = refPatch.getSizeD() / minLevel;
      if ( prevDD * minLevel != refPatch.getSizeD() ) { prevDD += 1; }
      patch.getSizeD() = size_t( ( delta_DD + prevDD ) *
                                 minLevel );  //( std::min )( size_t( ( delta_DD + prevDD ) * minLevel ), (size_t)256 );
      patch.setLodScaleX( refPatch.getLodScaleX() );
      patch.setLodScaleYIdc( refPatch.getLodScaleY() );
      prevSizeU0              = patch.getSizeU0();
      prevSizeV0              = patch.getSizeV0();
      prevPatchSize2DXInPixel = patch.getPatchSize2DXInPixel();
      prevPatchSize2DYInPixel = patch.getPatchSize2DYInPixel();

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
    } else if ( currPatchType == MERGE_PATCH ) {
      assert( -2 );
      auto& patch                    = patches[patchIndex];
      patch.getOccupancyResolution() = size_t( 1 ) << asps.getLog2PatchPackingBlockSize();
      auto&        mpdu              = pid.getMergePatchDataUnit();
      bool         overridePlrFlag   = false;
      const size_t max3DCoordinate   = size_t( 1 ) << ( gi.getGeometry3dCoordinatesBitdepthMinus1() + 1 );

      TRACE_PATCH( "patch %zu / %zu: Inter \n", patchIndex, patchCount );
      TRACE_PATCH(
          "MPDU: refAtlasFrame= %d refPatchIdx = ?? pos2DXY = %ld %ld pos3DXYZW = %ld %ld %ld %ld size2D = %ld %ld "
          "\n",
          mpdu.getRefIndex(), mpdu.get2dPosX(), mpdu.get2dPosY(), mpdu.get3dOffsetU(), mpdu.get3dOffsetV(),
          mpdu.get3dOffsetD(), mpdu.get3dRangeD(), mpdu.get2dDeltaSizeX(), mpdu.get2dDeltaSizeY() );

      patch.setBestMatchIdx( patchIndex );
      predIndex = patchIndex;
      patch.setRefAtlasFrameIndex( mpdu.getRefIndex() );
      size_t      refPOC   = (size_t)tile.getRefAfoc( patch.getRefAtlasFrameIndex() );
      const auto& refPatch = context.getFrame( refPOC ).getTile( tileIndex ).getPatches()[patch.getBestMatchIdx()];

      if ( mpdu.getOverride2dParamsFlag() ) {
        patch.getU0() = mpdu.get2dPosX() + refPatch.getU0();
        patch.getV0() = mpdu.get2dPosY() + refPatch.getV0();
        if ( asps.getPatchSizeQuantizerPresentFlag() ) {
          patch.setPatchSize2DXInPixel( refPatch.getPatchSize2DXInPixel() +
                                        ( mpdu.get2dDeltaSizeX() ) * quantizerSizeX );
          patch.setPatchSize2DYInPixel( refPatch.getPatchSize2DYInPixel() +
                                        ( mpdu.get2dDeltaSizeY() ) * quantizerSizeY );

          patch.getSizeU0() =
              ceil( static_cast<double>( patch.getPatchSize2DXInPixel() ) / static_cast<double>( packingBlockSize ) );
          patch.getSizeV0() =
              ceil( static_cast<double>( patch.getPatchSize2DYInPixel() ) / static_cast<double>( packingBlockSize ) );
        } else {
          patch.getSizeU0() = mpdu.get2dDeltaSizeX() + refPatch.getSizeU0();
          patch.getSizeV0() = mpdu.get2dDeltaSizeY() + refPatch.getSizeV0();
        }

        if ( asps.getPLREnabledFlag() ) { overridePlrFlag = true; }
      } else {
        if ( mpdu.getOverride3dParamsFlag() ) {
          patch.getU1() = mpdu.get3dOffsetU() + refPatch.getU1();
          patch.getV1() = mpdu.get3dOffsetV() + refPatch.getV1();
          if ( patch.getProjectionMode() == 0 ) {
            patch.getD1() = ( mpdu.get3dOffsetD() + ( refPatch.getD1() / minLevel ) ) * minLevel;
          } else {
            if ( static_cast<int>( asps.getExtendedProjectionEnabledFlag() ) == 0 ) {
              patch.getD1() =
                  max3DCoordinate -
                  ( mpdu.get3dOffsetD() + ( ( max3DCoordinate - refPatch.getD1() ) / minLevel ) ) * minLevel;
            } else {
              patch.getD1() =
                  max3DCoordinate -
                  ( mpdu.get3dOffsetD() + ( ( max3DCoordinate - refPatch.getD1() ) / minLevel ) ) * minLevel;
            }
          }

          const int64_t delta_DD = mpdu.get3dRangeD();
          size_t        prevDD   = refPatch.getSizeD() / minLevel;
          if ( prevDD * minLevel != refPatch.getSizeD() ) { prevDD += 1; }
          patch.getSizeD() =
              size_t( ( delta_DD + prevDD ) *
                      minLevel );  //( std::min )( size_t( ( delta_DD + prevDD ) * minLevel ), (size_t)256 );

          if ( asps.getPLREnabledFlag() ) { overridePlrFlag = ( mpdu.getOverridePlrFlag() != 0 ); }
        }
      }
      patch.getProjectionMode()        = refPatch.getProjectionMode();
      patch.getPatchOrientation()      = refPatch.getPatchOrientation();
      patch.getNormalAxis()            = refPatch.getNormalAxis();
      patch.getTangentAxis()           = refPatch.getTangentAxis();
      patch.getBitangentAxis()         = refPatch.getBitangentAxis();
      patch.getAxisOfAdditionalPlane() = refPatch.getAxisOfAdditionalPlane();
      patch.setLodScaleX( refPatch.getLodScaleX() );
      patch.setLodScaleYIdc( refPatch.getLodScaleY() );
      prevSizeU0              = patch.getSizeU0();
      prevSizeV0              = patch.getSizeV0();
      prevPatchSize2DXInPixel = patch.getPatchSize2DXInPixel();
      prevPatchSize2DYInPixel = patch.getPatchSize2DYInPixel();

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
    } else if ( currPatchType == SKIP_PATCH ) {
      assert( -1 );
      auto& patch = patches[patchIndex];
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

      patch.getProjectionMode()   = refPatch.getProjectionMode();
      patch.getU0()               = refPatch.getU0();
      patch.getV0()               = refPatch.getV0();
      patch.getPatchOrientation() = refPatch.getPatchOrientation();
      patch.getU1()               = refPatch.getU1();
      patch.getV1()               = refPatch.getV1();
      if ( asps.getPatchSizeQuantizerPresentFlag() ) {
        patch.setPatchSize2DXInPixel( refPatch.getPatchSize2DXInPixel() );
        patch.setPatchSize2DYInPixel( refPatch.getPatchSize2DYInPixel() );

        patch.getSizeU0() =
            ceil( static_cast<double>( patch.getPatchSize2DXInPixel() ) / static_cast<double>( packingBlockSize ) );
        patch.getSizeV0() =
            ceil( static_cast<double>( patch.getPatchSize2DYInPixel() ) / static_cast<double>( packingBlockSize ) );
      } else {
        patch.getSizeU0() = refPatch.getSizeU0();
        patch.getSizeV0() = refPatch.getSizeV0();
      }
      patch.getNormalAxis()            = refPatch.getNormalAxis();
      patch.getTangentAxis()           = refPatch.getTangentAxis();
      patch.getBitangentAxis()         = refPatch.getBitangentAxis();
      patch.getAxisOfAdditionalPlane() = refPatch.getAxisOfAdditionalPlane();
      const size_t max3DCoordinate     = size_t( 1 ) << ( gi.getGeometry3dCoordinatesBitdepthMinus1() + 1 );
      if ( patch.getProjectionMode() == 0 ) {
        patch.getD1() = ( ( refPatch.getD1() / minLevel ) ) * minLevel;
      } else {
        if ( static_cast<int>( asps.getExtendedProjectionEnabledFlag() ) == 0 ) {  // TODO JR: remove the if;
          patch.getD1() = max3DCoordinate - ( ( ( max3DCoordinate - refPatch.getD1() ) / minLevel ) ) * minLevel;
        } else {
          patch.getD1() = max3DCoordinate - ( ( ( max3DCoordinate - refPatch.getD1() ) / minLevel ) ) * minLevel;
        }
      }
      size_t prevDD = refPatch.getSizeD() / minLevel;
      if ( prevDD * minLevel != refPatch.getSizeD() ) { prevDD += 1; }
      patch.getSizeD() = size_t( (prevDD)*minLevel );  //( std::min )( size_t( (prevDD)*minLevel ), (size_t)256 );
      patch.setLodScaleX( refPatch.getLodScaleX() );
      patch.setLodScaleYIdc( refPatch.getLodScaleY() );
      prevSizeU0              = patch.getSizeU0();
      prevSizeV0              = patch.getSizeV0();
      prevPatchSize2DXInPixel = patch.getPatchSize2DXInPixel();
      prevPatchSize2DYInPixel = patch.getPatchSize2DYInPixel();
      TRACE_PATCH(
          "patch(skip) %zu: UV0 %4zu %4zu UV1 %4zu %4zu D1=%4zu S=%4zu %4zu %4zu P=%zu O=%zu A=%u%u%u Lod = %zu,%zu "
          "\n",
          patchIndex, patch.getU0(), patch.getV0(), patch.getU1(), patch.getV1(), patch.getD1(), patch.getSizeU0(),
          patch.getSizeV0(), patch.getSizeD(), patch.getProjectionMode(), patch.getPatchOrientation(),
          patch.getNormalAxis(), patch.getTangentAxis(), patch.getBitangentAxis(), patch.getLodScaleX(),
          patch.getLodScaleY() );
      patch.allocOneLayerData();
    } else if ( currPatchType == RAW_PATCH ) {
      TRACE_PATCH( "patch %zu / %zu: raw \n", patchIndex, patchCount );
      auto& rpdu                        = pid.getRawPatchDataUnit();
      auto& rawPointsPatch              = pcmPatches[patchIndex - numNonRawPatch];
      rawPointsPatch.isPatchInAuxVideo_ = rpdu.getPatchInAuxiliaryVideoFlag();
      rawPointsPatch.u0_                = rpdu.get2dPosX();
      rawPointsPatch.v0_                = rpdu.get2dPosY();
      rawPointsPatch.sizeU0_            = rpdu.get2dSizeXMinus1() + 1;
      rawPointsPatch.sizeV0_            = rpdu.get2dSizeYMinus1() + 1;
      if ( afps.getRaw3dPosBitCountExplicitModeFlag() ) {
        rawPointsPatch.u1_ = rpdu.get3dOffsetU();
        rawPointsPatch.v1_ = rpdu.get3dOffsetV();
        rawPointsPatch.d1_ = rpdu.get3dOffsetD();
      } else {
        const size_t pcmU1V1D1Level = size_t( 1 ) << ( gi.getGeometry2dBitdepthMinus1() + 1 );
        rawPointsPatch.u1_          = rpdu.get3dOffsetU() * pcmU1V1D1Level;
        rawPointsPatch.v1_          = rpdu.get3dOffsetV() * pcmU1V1D1Level;
        rawPointsPatch.d1_          = rpdu.get3dOffsetD() * pcmU1V1D1Level;
      }
      rawPointsPatch.setNumberOfRawPoints( rpdu.getRawPointsMinus1() + 1 );
      rawPointsPatch.occupancyResolution_ = size_t( 1 ) << asps.getLog2PatchPackingBlockSize();
      totalNumberOfRawPoints += rawPointsPatch.getNumberOfRawPoints();
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
      eomPatch.memberPatches.resize( epdu.getPatchCountMinus1() + 1 );
      eomPatch.eomCountPerPatch.resize( epdu.getPatchCountMinus1() + 1 );
      eomPatch.eomCount_ = 0;
      for ( size_t i = 0; i < eomPatch.memberPatches.size(); i++ ) {
        eomPatch.memberPatches[i]    = epdu.getAssociatedPatchesIdx( i );
        eomPatch.eomCountPerPatch[i] = epdu.getPoints( i );
        eomPatch.eomCount_ += eomPatch.eomCountPerPatch[i];
      }
      eomPatch.occupancyResolution_ = size_t( 1 ) << asps.getLog2PatchPackingBlockSize();
      eomPatches.push_back( eomPatch );
      totalNumberOfEomPoints += eomPatch.eomCount_;
      TRACE_PATCH( "EOM: U0V0 %zu,%zu\tSizeU0V0 %zu,%zu\tN= %zu,%zu\n", eomPatch.u0_, eomPatch.v0_, eomPatch.sizeU_,
                   eomPatch.sizeV_, eomPatch.memberPatches.size(), eomPatch.eomCount_ );
      for ( size_t i = 0; i < eomPatch.memberPatches.size(); i++ ) {
        TRACE_PATCH( "%zu, %zu\n", eomPatch.memberPatches[i], eomPatch.eomCountPerPatch[i] );
      }
      TRACE_PATCH( "\n" );
    } else if ( currPatchType == END_PATCH ) {
      break;
    } else {
      printf( "Error: unknow frame/patch type \n" );
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
  for ( auto& e : encMD5 ) printf( "%02x", e );
  std::cout << ", ";
  for ( auto& d : decMD5 ) printf( "%02x", d );
  return equal;
}
bool PCCDecoder::compareHashSEICrc( uint16_t encCrc, uint16_t decCrc ) {
  bool equal = true;
  if ( encCrc != decCrc ) equal = false;
  printf( "%04x", encCrc );
  std::cout << ", ";
  printf( "%04x", decCrc );
  return equal;
}

bool PCCDecoder::compareHashSEICheckSum( uint32_t encCheckSum, uint32_t decCheckSum ) {
  bool equal = true;
  if ( encCheckSum != decCheckSum ) equal = false;
  printf( "%08x", encCheckSum );
  std::cout << ", ";
  printf( "%08x", decCheckSum );
  return equal;
}

void PCCDecoder::createHashInformation( PCCContext& context, int frameIndex ) {
  TRACE_PATCH( "createHashInformation Frame %d \n", frameIndex );
  printf( "createHashInformation frame %d\n", frameIndex );
  size_t                                         hashIndex         = frameIndex;
  auto&                                          sei               = context.getSeiHash( hashIndex );
  bool                                           seiHashCancelFlag = sei.getCancelFlag();
  std::vector<PatchParams>                       atlasPatchParams;
  std::vector<std::vector<PatchParams>>          tilePatchParams;
  std::vector<std::vector<std::vector<int64_t>>> tileB2PPatchParams;
  std::vector<std::vector<int64_t>>              atlasB2PPatchParams;

  if ( !seiHashCancelFlag && sei.getDecodedHighLevelHashPresentFlag() ) {
    size_t               atlIdx     = context[frameIndex].getTile( 0 ).getAtlIndex();
    auto&                tileHeader = context.getAtlasTileLayerList()[atlIdx].getHeader();
    size_t               afpsIndex  = tileHeader.getAtlasFrameParameterSetId();
    size_t               aspsIndex  = context.getAtlasFrameParameterSet( afpsIndex ).getAtlasFrameParameterSetId();
    auto&                asps       = context.getAtlasSequenceParameterSet( aspsIndex );
    auto&                afps       = context.getAtlasFrameParameterSet( afpsIndex );
    std::vector<uint8_t> highLevelAtlasData;
    aspsCommonByteString( highLevelAtlasData, asps );
    aspsApplicationByteString( highLevelAtlasData, asps, afps );
    afpsCommonByteString( highLevelAtlasData, context, afpsIndex, frameIndex );
    afpsApplicationByteString( highLevelAtlasData, asps, afps );

    printf( "**sei** HighLevel Hash\n" );
    if ( sei.getHashType() == 0 ) {
      bool                 equal = false;
      std::vector<uint8_t> encMD5( 16 ), decMD5( 16 );
      encMD5 = context.computeMD5( highLevelAtlasData.data(), highLevelAtlasData.size() );
      for ( int j = 0; j < 16; j++ ) decMD5[j] = sei.getHighLevelMd5( j );
      printf( "\t**sei** (MD5): " );
      equal = compareHashSEIMD5( encMD5, decMD5 );
      printf( " (%s) \n", equal ? "OK" : "DIFF" );
    } else if ( sei.getHashType() == 1 ) {
      bool     equal = true;
      uint16_t crc   = context.computeCRC( highLevelAtlasData.data(), highLevelAtlasData.size() );
      printf( "\t**sei** (CRC): " );
      equal = compareHashSEICrc( crc, sei.getHighLevelCrc() );
      printf( " (%s) \n", equal ? "OK" : "DIFF" );
    } else if ( sei.getHashType() == 2 ) {
      bool     equal    = true;
      uint32_t checkSum = context.computeCheckSum( highLevelAtlasData.data(), highLevelAtlasData.size() );
      printf( "\t**sei** (CheckSum): " );
      equal = compareHashSEICheckSum( checkSum, sei.getHighLevelCheckSum() );
      printf( " (%s) \n", equal ? "OK" : "DIFF" );
    }
    highLevelAtlasData.clear();
  }

  if ( !seiHashCancelFlag && ( sei.getDecodedAtlasTilesHashPresentFlag() || sei.getDecodedAtlasHashPresentFlag() ) ) {
    size_t numTilesInPatchFrame = context[frameIndex].getNumTilesInAtlasFrame();
    if ( sei.getDecodedAtlasTilesHashPresentFlag() ) tilePatchParams.resize( numTilesInPatchFrame );
    for ( size_t tileIdx = 0; tileIdx < numTilesInPatchFrame; tileIdx++ ) {
      auto& tile = context[frameIndex].getTile( tileIdx );
      getHashPatchParams( context, frameIndex, tileIdx, tile.getAtlIndex(), tilePatchParams, atlasPatchParams );
    }
  }
  if ( !seiHashCancelFlag &&
       ( sei.getDecodedAtlasB2pHashPresentFlag() || sei.getDecodedAtlasTilesB2pHashPresentFlag() ) ) {
    getB2PHashPatchParams( context, frameIndex, tileB2PPatchParams, atlasB2PPatchParams );
  }
  // frame
  if ( !seiHashCancelFlag && sei.getDecodedAtlasHashPresentFlag() ) {
    std::vector<uint8_t> atlasData;
    size_t               patchCount = atlasPatchParams.size();
    for ( size_t patchIdx = 0; patchIdx < patchCount; patchIdx++ ) {
      atlasPatchCommonByteString( atlasData, patchIdx, atlasPatchParams );
      atlasPatchApplicationByteString( atlasData, patchIdx, atlasPatchParams );
    }
    printf( "**sei** AtlasPatchHash: frame(%d) (#patches %zu)\n", frameIndex, patchCount );
    if ( sei.getHashType() == 0 ) {
      bool                 equal = false;
      std::vector<uint8_t> encMD5( 16 ), decMD5( 16 );
      encMD5 = context.computeMD5( atlasData.data(), atlasData.size() );
      for ( int j = 0; j < 16; j++ ) decMD5[j] = sei.getAtlasMd5( j );
      printf( "\t**sei** (MD5): " );
      equal = compareHashSEIMD5( encMD5, decMD5 );
      printf( " (%s) \n", equal ? "OK" : "DIFF" );
    } else if ( sei.getHashType() == 1 ) {
      bool     equal = true;
      uint16_t crc   = context.computeCRC( atlasData.data(), atlasData.size() );
      printf( "\t**sei** (CRC): " );
      equal = compareHashSEICrc( crc, sei.getAtlasCrc() );
      printf( " (%s) \n", equal ? "OK" : "DIFF" );
    } else if ( sei.getHashType() == 2 ) {
      bool     equal    = true;
      uint32_t checkSum = context.computeCheckSum( atlasData.data(), atlasData.size() );
      printf( "\t**sei** (CheckSum): " );
      equal = compareHashSEICheckSum( checkSum, sei.getAtlasCheckSum() );
      printf( " (%s) \n", equal ? "OK" : "DIFF" );
    }
    atlasData.clear();
  }
  if ( sei.getDecodedAtlasB2pHashPresentFlag() && !seiHashCancelFlag ) {
    std::vector<uint8_t> atlasB2PData;
    atlasBlockToPatchByteString( atlasB2PData, atlasB2PPatchParams );
    printf( "**sei** AtlasBlockToPatchHash: frame(%d) \n", frameIndex );
    if ( sei.getHashType() == 0 ) {
      bool                 equal = true;
      std::vector<uint8_t> encMD5( 16 ), decMD5( 16 );
      encMD5 = context.computeMD5( atlasB2PData.data(), atlasB2PData.size() );
      for ( int j = 0; j < 16; j++ ) decMD5[j] = sei.getAtlasB2pMd5( j );
      printf( "\t**sei** (MD5): " );
      equal = compareHashSEIMD5( encMD5, decMD5 );
      printf( " (%s) \n", equal ? "OK" : "DIFF" );
    } else if ( sei.getHashType() == 1 ) {
      bool     equal = false;
      uint16_t crc   = context.computeCRC( atlasB2PData.data(), atlasB2PData.size() );
      printf( "\t**sei** (CRC): " );
      equal = compareHashSEICrc( crc, sei.getAtlasB2pCrc() );
      printf( " (%s) \n", equal ? "OK" : "DIFF" );
    } else if ( sei.getHashType() == 2 ) {
      bool     equal    = false;
      uint32_t checkSum = context.computeCheckSum( atlasB2PData.data(), atlasB2PData.size() );
      printf( "\t**sei** (CheckSum): " );
      equal = compareHashSEICheckSum( checkSum, sei.getAtlasB2pCheckSum() );
      printf( " (%s) \n", equal ? "OK" : "DIFF" );
    }
    atlasB2PData.clear();
  }

  // for tiles
  if ( !seiHashCancelFlag && sei.getDecodedAtlasTilesHashPresentFlag() ||
       sei.getDecodedAtlasTilesB2pHashPresentFlag() ) {
    size_t numTilesInPatchFrame = context[frameIndex].getNumTilesInAtlasFrame();
    for ( size_t tileIdx = 0; tileIdx < numTilesInPatchFrame; tileIdx++ ) {
      auto&  tile   = context[frameIndex].getTile( tileIdx );
      auto&  atlu   = context.getAtlasTileLayer( tile.getAtlIndex() );
      auto&  ath    = atlu.getHeader();
      size_t tileId = ath.getId();
      if ( sei.getDecodedAtlasTilesHashPresentFlag() ) {
        std::vector<uint8_t> atlasTileData;
        for ( size_t patchIdx = 0; patchIdx < atlu.getDataUnit().getPatchCount(); patchIdx++ ) {
          tilePatchCommonByteString( atlasTileData, tileId, patchIdx, tilePatchParams );
          tilePatchApplicationByteString( atlasTileData, tileId, patchIdx, tilePatchParams );
        }
        printf( "**sei** TilesPatchHash: frame(%d), tile(%zu, tileId %zu)\n", frameIndex, tileIdx, tileId );
        if ( sei.getHashType() == 0 ) {
          bool                 equal = true;
          std::vector<uint8_t> encMD5( 16 ), decMD5( 16 );
          encMD5 = context.computeMD5( atlasTileData.data(), atlasTileData.size() );
          for ( int j = 0; j < 16; j++ ) decMD5[j] = sei.getAtlasTilesMd5( tileId, j );
          printf( "\t**sei** (MD5): " );
          equal = compareHashSEIMD5( encMD5, decMD5 );
          printf( " (%s) \n", equal ? "OK" : "DIFF" );
        } else if ( sei.getHashType() == 1 ) {
          bool     equal = true;
          uint16_t crc   = context.computeCRC( atlasTileData.data(), atlasTileData.size() );
          printf( "\t**sei** (CRC): " );
          equal = compareHashSEICrc( crc, sei.getAtlasTilesCrc( tileId ) );
          printf( " (%s) \n", equal ? "OK" : "DIFF" );
        } else if ( sei.getHashType() == 2 ) {
          bool     equal    = false;
          uint32_t checkSum = context.computeCheckSum( atlasTileData.data(), atlasTileData.size() );
          printf( "\t**sei** (CheckSum): " );
          equal = compareHashSEICheckSum( checkSum, sei.getAtlasTilesCheckSum( tileId ) );
          printf( " (%s) \n", equal ? "OK" : "DIFF" );
        }
        atlasTileData.clear();
      }
      if ( sei.getDecodedAtlasTilesB2pHashPresentFlag() ) {
        std::vector<uint8_t> tileB2PData;
        tileBlockToPatchByteString( tileB2PData, tileId, tileB2PPatchParams );
        printf( "**sei** TilesBlockToPatchHash: frame(%d), tile(%zu, tileId %zu)\n", frameIndex, tileIdx, tileId );
        if ( sei.getHashType() == 0 ) {
          bool                 equal = false;
          std::vector<uint8_t> encMD5( 16 ), decMD5( 16 );
          encMD5 = context.computeMD5( tileB2PData.data(), tileB2PData.size() );
          for ( int j = 0; j < 16; j++ ) decMD5[j] = sei.getAtlasTilesB2pMd5( tileId, j );
          printf( "\t**sei** (MD5): " );
          equal = compareHashSEIMD5( encMD5, decMD5 );
          printf( " (%s) \n", equal ? "OK" : "DIFF" );
        } else if ( sei.getHashType() == 1 ) {
          bool     equal = false;
          uint16_t crc   = context.computeCRC( tileB2PData.data(), tileB2PData.size() );
          printf( "\t**sei** (CRC): " );
          equal = compareHashSEICrc( crc, sei.getAtlasTilesB2pCrc( tileId ) );
          printf( " (%s) \n", equal ? "OK" : "DIFF" );
        } else if ( sei.getHashType() == 2 ) {
          bool     equal    = false;
          uint32_t checkSum = context.computeCheckSum( tileB2PData.data(), tileB2PData.size() );
          printf( "\t**sei** (CheckSum): " );
          equal = compareHashSEICheckSum( checkSum, sei.getAtlasTilesB2pCheckSum( tileId ) );
          printf( " (%s) \n", equal ? "OK" : "DIFF" );
        }
        tileB2PData.clear();
      }
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
}
