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
#include "PCCBitstreamEncoder.h"
#include "PCCBitstreamEncoder.h"
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCPatch.h"
#include "PCCPatchSegmenter.h"
#include "PCCVideoEncoder.h"
#include "PCCSystem.h"
#include "PCCGroupOfFrames.h"
#include "PCCPointSet.h"
#include "PCCEncoderParameters.h"
#include "PCCKdTree.h"
#include <tbb/tbb.h>
#include "PCCChrono.h"
#include "PCCEncoder.h"

using namespace std;
using namespace pcc;

std::string getEncoderConfig1L( std::string string ) {
  std::string sub    = string.substr( 0, string.find_last_of( "." ) );
  std::string result = sub + "-1L.cfg";
  return result;
}

PCCEncoder::PCCEncoder() {}

PCCEncoder::~PCCEncoder() {}

void PCCEncoder::setParameters( PCCEncoderParameters params ) { params_ = params; }

int PCCEncoder::encode( const PCCGroupOfFrames& sources,
                        PCCContext&             context,
                        PCCBitstream&           bitstream,
                        PCCGroupOfFrames&       reconstructs ) {
  int    ret                                  = 0;
  size_t oneLayerModeOriginal                 = params_.oneLayerMode_;
  size_t singleLayerPixelInterleavingOriginal = params_.singleLayerPixelInterleaving_;
  if ( params_.nbThread_ > 0 ) { tbb::task_scheduler_init init( (int)params_.nbThread_ ); }

  ret |= encode( sources, context, reconstructs );

  params_.initializeContext( context );
#ifdef CODEC_TRACE
  setTrace( true );
  openTrace( removeFileExtension( params_.compressedStreamPath_ ) + "_patch_encode.txt" );
#endif
#if 0
  PCCBitstreamEncoder bitstreamEncoder;
#else
  PCCBitstreamEncoder bitstreamEncoder;
  createPatchFrameDataStructure( context );
#endif
#ifdef CODEC_TRACE
  closeTrace();
#endif

#ifdef BITSTREAM_TRACE
  bitstream.setTrace( true );
  bitstream.openTrace( removeFileExtension( params_.compressedStreamPath_ ) + "_hls_encode.txt" );
#endif
  bitstreamEncoder.setParameters( params_ );
  ret |= bitstreamEncoder.encode( context, bitstream );
#ifdef BITSTREAM_TRACE
  bitstream.closeTrace();
#endif
  params_.oneLayerMode_                 = oneLayerModeOriginal;
  params_.singleLayerPixelInterleaving_ = singleLayerPixelInterleavingOriginal;

  return ret;
}

int PCCEncoder::encode( const PCCGroupOfFrames& sources, PCCContext& context, PCCGroupOfFrames& reconstructs ) {
  assert( sources.size() < 256 );
  if ( sources.size() == 0 ) { return 0; }

#ifdef CODEC_TRACE
  setTrace( true );
  openTrace( removeFileExtension( params_.compressedStreamPath_ ) + "_codec_encode.txt" );
#endif

  params_.initializeContext( context );
  reconstructs.resize( sources.size() );
  context.resize( sources.size() );
  auto& frames                         = context.getFrames();
  auto& frameLevelMetadataEnabledFlags = context.getGOFLevelMetadata().getLowerLevelMetadataEnabledFlags();
  if ( frameLevelMetadataEnabledFlags.getMetadataEnabled() ) {
    for ( size_t i = 0; i < frames.size(); i++ ) {
      // Place to get/set frame-level metadata.
      PCCMetadata frameLevelMetadata;
      frameLevelMetadata.setMetadataType( METADATA_FRAME );
      frames[i].getFrameLevelMetadata() = frameLevelMetadata;
      frames[i].getFrameLevelMetadata().setIndex( i );
      frames[i].getFrameLevelMetadata().getMetadataEnabledFlags() = frameLevelMetadataEnabledFlags;
      // Place to get/set patch metadata enabled flags (in frame level).
      PCCMetadataEnabledFlags patchLevelMetadataEnabledFlags;
      frames[i].getFrameLevelMetadata().getLowerLevelMetadataEnabledFlags() = patchLevelMetadataEnabledFlags;
    }
  }

  for ( size_t i = 0; i < frames.size(); i++ ) {
    frames[i].setLosslessGeo( params_.losslessGeo_ );
    frames[i].setLosslessGeo444( params_.losslessGeo444_ );
    frames[i].setLosslessTexture( params_.losslessTexture_ );
    // frames[i].setEnhancedDeltaDepth(params_.enhancedDeltaDepthCode_);
    frames[i].setUseAdditionalPointsPatch( params_.losslessGeo_ || params_.lossyMissedPointsPatch_ );
    frames[i].setUseAdditionalPointsPatch( params_.useAdditionalPointsPatch_ );
    frames[i].setUseMissedPointsSeparateVideo( params_.useMissedPointsSeparateVideo_ );
  }

  PCCVideoEncoder       videoEncoder;
  const size_t          pointCount = sources[0].getPointCount();
  std::stringstream     path;
  SequenceParameterSet& sps = context.getSps();
  path << removeFileExtension( params_.compressedStreamPath_ ) << "_GOF" << sps.getSequenceParameterSetId() << "_";

  generateGeometryVideo( sources, context );
  // RA mode.
  if ( params_.globalPatchAllocation_ ) { performDataAdaptiveGPAMethod( context ); }
  // const size_t nbFramesGeometry = params_.oneLayerMode_  ? 1 : 2;
  const size_t nbFramesTexture = params_.oneLayerMode_ ? 1 : 2;
  resizeGeometryVideo( context );
  dilateGeometryVideo( context );
  sps.setFrameWidth( (uint16_t)frames[0].getWidth() );
  sps.setFrameHeight( (uint16_t)frames[0].getHeight() );
  // auto width  = sps.getFrameWidth ();
  // auto height = sps.getFrameHeight();

  auto& videoBitstream = context.createVideoBitstream( PCCVideoType::OccupancyMap );
  generateOccupancyMapVideo( sources, context );
  auto& videoOccupancyMap = context.getVideoOccupancyMap();
  videoEncoder.compress( videoOccupancyMap, path.str(), params_.occupancyMapQP_, videoBitstream,
                         params_.occupancyMapVideoEncoderConfig_, params_.videoEncoderOccupancyMapPath_, context, 1,
                         params_.keepIntermediateFiles_ );

  generateBlockToPatchFromOccupancyMap( context, params_.losslessGeo_, params_.lossyMissedPointsPatch_,
                                        params_.testLevelOfDetail_, params_.occupancyResolution_ );

  // Group dilation in Geometry
  if ( params_.groupDilation_ && params_.absoluteD1_ && !params_.oneLayerMode_ ) { geometryGroupDilation( context ); }

  if ( params_.use3dmc_ ) { create3DMotionEstimationFiles( context, path.str() ); }

  const size_t nbyteGeo =
      ( params_.losslessGeo_ || ( params_.lossyMissedPointsPatch_ && !sps.getPcmSeparateVideoPresentFlag() ) ) ? 2 : 1;
  if ( !params_.absoluteD1_ ) {
    if ( params_.lossyMissedPointsPatch_ ) {
      std::cout << "Error: lossyMissedPointsPatch has not been implemented for absoluteD1_ = 0 as "
                   "yet. Exiting... "
                << std::endl;
      std::exit( -1 );
    }
    // Compress geometryD0
    auto& videoBitstreamD0 = context.createVideoBitstream( PCCVideoType::GeometryD0 );
    auto& videoGeometry    = context.getVideoGeometry();
    videoEncoder.compress(
        videoGeometry, path.str(), ( params_.geometryQP_ - 1 ), videoBitstreamD0, params_.geometryD0Config_,
        ( params_.use3dmc_ != 0 ) ? params_.videoEncoderAuxPath_ : params_.videoEncoderPath_, context, nbyteGeo,
        params_.keepIntermediateFiles_, params_.losslessGeo_ && params_.losslessGeo444_, false, params_.use3dmc_ );

    // Form differential video geometryD1
    auto& videoGeometryD1 = context.getVideoGeometryD1();
    for ( size_t f = 0; f < frames.size(); ++f ) {
      auto& frame1 = videoGeometryD1.getFrame( f );
      predictGeometryFrame( frames[f], videoGeometry.getFrame( f ), frame1 );
    }

    // Compress geometryD1
    auto& videoBitstreamD1 = context.createVideoBitstream( PCCVideoType::GeometryD1 );
    videoEncoder.compress(
        videoGeometryD1, path.str(), params_.geometryQP_, videoBitstreamD1, params_.geometryD1Config_,
        ( params_.use3dmc_ != 0 ) ? params_.videoEncoderAuxPath_ : params_.videoEncoderPath_, context, nbyteGeo,
        params_.keepIntermediateFiles_, params_.losslessGeo_ && params_.losslessGeo444_, false, params_.use3dmc_ );

    auto sizeGeometryVideo = videoBitstreamD0.naluSize() + videoBitstreamD1.naluSize();
    std::cout << "geometry video ->" << sizeGeometryVideo << " B ("
              << ( sizeGeometryVideo * 8.0 ) / ( 2 * frames.size() * pointCount ) << " bpp)" << std::endl;
  } else {
    auto& videoBitstream = context.createVideoBitstream( PCCVideoType::Geometry );
    auto& videoGeometry  = context.getVideoGeometry();
    videoEncoder.compress(
        videoGeometry, path.str(), params_.geometryQP_, videoBitstream,
        params_.oneLayerMode_ ? getEncoderConfig1L( params_.geometryConfig_ ) : params_.geometryConfig_,
        ( params_.use3dmc_ != 0 ) ? params_.videoEncoderAuxPath_ : params_.videoEncoderPath_, context, nbyteGeo,
        params_.keepIntermediateFiles_, params_.losslessGeo_ && params_.losslessGeo444_, false, params_.use3dmc_ );
  }

  if ( sps.getPcmPatchEnabledFlag() && sps.getPcmSeparateVideoPresentFlag() ) {
    auto& videoBitstreamMP = context.createVideoBitstream( PCCVideoType::GeometryMP );
    generateMissedPointsGeometryVideo( context, reconstructs );
    auto& videoMPsGeometry = context.getVideoMPsGeometry();
    videoEncoder.compress( videoMPsGeometry, path.str(),
                           params_.lossyMissedPointsPatch_ ? params_.lossyMppGeoQP_ : params_.geometryQP_,
                           videoBitstreamMP, params_.geometryMPConfig_, params_.videoEncoderPath_, context, 2,
                           params_.keepIntermediateFiles_ );
    if ( params_.lossyMissedPointsPatch_ ) { generateMissedPointsGeometryfromVideo( context, reconstructs ); }
  }

  GeneratePointCloudParameters generatePointCloudParameters;
  generatePointCloudParameters.occupancyResolution_         = params_.occupancyResolution_;
  generatePointCloudParameters.occupancyPrecision_          = params_.occupancyPrecision_;
  generatePointCloudParameters.flagGeometrySmoothing_       = params_.flagGeometrySmoothing_;
  generatePointCloudParameters.gridSmoothing_               = params_.gridSmoothing_;
  generatePointCloudParameters.gridSize_                    = params_.gridSize_;
  generatePointCloudParameters.neighborCountSmoothing_      = params_.neighborCountSmoothing_;
  generatePointCloudParameters.radius2Smoothing_            = params_.radius2Smoothing_;
  generatePointCloudParameters.radius2BoundaryDetection_    = params_.radius2BoundaryDetection_;
  generatePointCloudParameters.thresholdSmoothing_          = params_.thresholdSmoothing_;
  generatePointCloudParameters.losslessGeo_                 = params_.losslessGeo_;
  generatePointCloudParameters.losslessGeo444_              = params_.losslessGeo444_;
  generatePointCloudParameters.nbThread_                    = params_.nbThread_;
  generatePointCloudParameters.absoluteD1_                  = params_.absoluteD1_;
  generatePointCloudParameters.surfaceThickness             = params_.surfaceThickness_;
  generatePointCloudParameters.ignoreLod_                   = true;
  generatePointCloudParameters.thresholdColorSmoothing_     = params_.thresholdColorSmoothing_;
  generatePointCloudParameters.thresholdLocalEntropy_       = params_.thresholdLocalEntropy_;
  generatePointCloudParameters.radius2ColorSmoothing_       = params_.radius2ColorSmoothing_;
  generatePointCloudParameters.neighborCountColorSmoothing_ = params_.neighborCountColorSmoothing_;
  generatePointCloudParameters.flagColorSmoothing_          = params_.flagColorSmoothing_;
  generatePointCloudParameters.enhancedDeltaDepthCode_ =
      ( params_.losslessGeo_ ? params_.enhancedDeltaDepthCode_ : false );
  // generatePointCloudParameters.deltaCoding_                  =
  // (params_.testLevelOfDetailSignaling_ > 0);
  generatePointCloudParameters.removeDuplicatePoints_        = params_.removeDuplicatePoints_;
  generatePointCloudParameters.oneLayerMode_                 = params_.oneLayerMode_;
  generatePointCloudParameters.singleLayerPixelInterleaving_ = params_.singleLayerPixelInterleaving_;
  // generatePointCloudParameters.sixDirectionMode_             = params_.sixDirectionMode_;
  // generatePointCloudParameters.improveEDD_                   = params_.improveEDD_;
  generatePointCloudParameters.path_                     = path.str();
  generatePointCloudParameters.useAdditionalPointsPatch_ = params_.useAdditionalPointsPatch_;

  context.allocOneLayerData( params_.occupancyResolution_ );
  if ( params_.absoluteD1_ && params_.oneLayerMode_ && !params_.singleLayerPixelInterleaving_ ) {
    reconstuctionOptimization( context, generatePointCloudParameters );
  }
  generatePointCloud( reconstructs, context, generatePointCloudParameters );

  if ( sps.getAttributeCount() > 0 ) {
    generateTextureVideo( sources, reconstructs, context, params_ );
    auto&        videoTexture      = context.getVideoTexture();
    const size_t textureFrameCount = nbFramesTexture * frames.size();
    assert( textureFrameCount == videoTexture.getFrameCount() );
    if ( !( params_.losslessGeo_ && params_.textureDilationOffLossless_ ) ) {
      for ( size_t f = 0; f < textureFrameCount; ++f ) {
        using namespace std::chrono;
        pcc::chrono::Stopwatch<std::chrono::steady_clock> clockPadding;
        clockPadding.start();

        if ( params_.textureBGFill_ == 1 ) {
          dilatePullPush( frames[f / nbFramesTexture], videoTexture.getFrame( f ) );
        } else if ( params_.textureBGFill_ == 2 )
          dilateSparseLinearModel( frames[f / nbFramesTexture], videoTexture.getFrame( f ), f, Texture );
        else {
          dilate( frames[f / nbFramesTexture], videoTexture.getFrame( f ) );
        }

        clockPadding.stop();
        using ms              = milliseconds;
        auto totalPaddingTime = duration_cast<ms>( clockPadding.count() ).count();
        std::cout << "Processing time (Padding " << f << "): " << totalPaddingTime / 1000.0 << " s\n";

        if ( ( params_.groupDilation_ == true ) && ( ( f & 0x1 ) == 1 ) ) {
          if ( !params_.oneLayerMode_ ) {
            // Group dilation in texture
            auto&    frame        = frames[f / 2];
            auto&    occupancyMap = frame.getOccupancyMap();
            auto&    width        = frame.getWidth();
            auto&    height       = frame.getHeight();
            auto&    frame1       = videoTexture.getFrame( f - 1 );
            auto&    frame2       = videoTexture.getFrame( f );
            uint8_t  tmp_d0, tmp_d1;
            uint32_t tmp_avg;
            for ( size_t y = 0; y < height; y++ ) {
              for ( size_t x = 0; x < width; x++ ) {
                const size_t pos = y * width + x;
                if ( occupancyMap[pos] == 0 ) {
                  for ( size_t c = 0; c < 3; c++ ) {
                    tmp_d0  = frame1.getValue( c, x, y );
                    tmp_d1  = frame2.getValue( c, x, y );
                    tmp_avg = ( (uint32_t)tmp_d0 + (uint32_t)tmp_d1 + 1 ) >> 1;
                    frame1.setValue( c, x, y, (uint8_t)tmp_avg );
                    frame2.setValue( c, x, y, (uint8_t)tmp_avg );
                  }
                }
              }
            }
          }
        }
      }
    }

    auto& videoBitstream = context.createVideoBitstream( PCCVideoType::Texture );
    videoEncoder.compress(
        videoTexture, path.str(), params_.textureQP_, videoBitstream,
        params_.oneLayerMode_ ? getEncoderConfig1L( params_.textureConfig_ ) : params_.textureConfig_,
        ( params_.use3dmc_ != 0 ) ? params_.videoEncoderAuxPath_ : params_.videoEncoderPath_, context, 1,
        params_.keepIntermediateFiles_, params_.losslessTexture_, params_.patchColorSubsampling_, params_.use3dmc_,
        params_.colorSpaceConversionConfig_, params_.inverseColorSpaceConversionConfig_,
        params_.colorSpaceConversionPath_ );

    if ( params_.useAdditionalPointsPatch_ && sps.getPcmSeparateVideoPresentFlag() ) {
      auto& videoBitstreamMP = context.createVideoBitstream( PCCVideoType::TextureMP );
      generateMissedPointsTextureVideo( context, reconstructs );  // 1. texture
      auto& videoMPsTexture = context.getVideoMPsTexture();
      videoEncoder.compress( videoMPsTexture, path.str(), params_.textureQP_, videoBitstreamMP,
                             params_.textureMPConfig_, params_.videoEncoderPath_, context, 1,
                             params_.keepIntermediateFiles_, params_.losslessTexture_, false, false,
                             params_.colorSpaceConversionConfig_, params_.inverseColorSpaceConversionConfig_,
                             params_.colorSpaceConversionPath_ );

      if ( params_.lossyMissedPointsPatch_ ) { generateMissedPointsTexturefromVideo( context, reconstructs ); }
    }
  }

  colorPointCloud( reconstructs, context, sps.getAttributeCount(), params_.colorTransform_,
                   generatePointCloudParameters );

  if ( !params_.keepIntermediateFiles_ && params_.use3dmc_ ) { remove3DMotionEstimationFiles( path.str() ); }
#ifdef CODEC_TRACE
  setTrace( false );
  closeTrace();
#endif
  return 0;
}

void PCCEncoder::printMap( std::vector<bool> img, const size_t sizeU, const size_t sizeV ) {
  std::cout << std::endl;
  std::cout << "PrintMap size = " << sizeU << " x " << sizeV << std::endl;
  for ( size_t v = 0; v < sizeV; ++v ) {
    for ( size_t u = 0; u < sizeU; ++u ) { std::cout << ( img[v * sizeU + u] ? 'X' : '.' ); }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

void PCCEncoder::printMapTetris( std::vector<bool> img,
                                 const size_t      sizeU,
                                 const size_t      sizeV,
                                 std::vector<int>  horizon ) {
  std::cout << std::endl;
  std::cout << "PrintMap size = " << sizeU << " x " << sizeV << std::endl;
  for ( int v = 0; v < sizeV; ++v ) {
    for ( int u = 0; u < sizeU; ++u ) {
      if ( v == horizon[u] )
        std::cout << ( img[v * sizeU + u] ? 'U' : 'O' );
      else
        std::cout << ( img[v * sizeU + u] ? 'X' : '.' );
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

bool PCCEncoder::generateOccupancyMapVideo( const PCCGroupOfFrames& sources, PCCContext& context ) {
  auto& videoOccupancyMap = context.getVideoOccupancyMap();
  bool  ret               = true;
  videoOccupancyMap.resize( sources.size() );
  for ( size_t f = 0; f < sources.size(); ++f ) {
    auto&                 contextFrame = context.getFrames()[f];
    PCCImageOccupancyMap& videoFrame   = videoOccupancyMap.getFrame( f );
    ret &= generateOccupancyMapVideo( contextFrame.getWidth(), contextFrame.getHeight(), contextFrame.getOccupancyMap(),
                                      videoFrame );
  }
  return ret;
}

bool PCCEncoder::generateOccupancyMapVideo( const size_t           imageWidth,
                                            const size_t           imageHeight,
                                            std::vector<uint32_t>& occupancyMap,
                                            PCCImageOccupancyMap&  videoFrameOccupancyMap ) {
  const size_t   blockSize0  = params_.occupancyResolution_ / params_.occupancyPrecision_;
  const size_t   pointCount0 = blockSize0 * blockSize0;
  vector<bool>   block0;
  vector<size_t> bestRuns;
  vector<size_t> runs;
  block0.resize( pointCount0 );
  size_t       videoFrameOccupancyMapSizeU = imageWidth / params_.occupancyPrecision_;
  size_t       videoFrameOccupancyMapSizeV = imageHeight / params_.occupancyPrecision_;
  const size_t blockToPatchWidth           = imageWidth / params_.occupancyResolution_;
  const size_t blockToPatchHeight          = imageHeight / params_.occupancyResolution_;

  if ( !params_.enhancedDeltaDepthCode_ ) {
    videoFrameOccupancyMap.resize( videoFrameOccupancyMapSizeU, videoFrameOccupancyMapSizeV );
    for ( size_t v0 = 0; v0 < blockToPatchHeight; ++v0 ) {
      for ( size_t u0 = 0; u0 < blockToPatchWidth; ++u0 ) {
        size_t fullCount = 0;
        for ( size_t v1 = 0; v1 < blockSize0; ++v1 ) {
          const size_t v2 = v0 * params_.occupancyResolution_ + v1 * params_.occupancyPrecision_;
          for ( size_t u1 = 0; u1 < blockSize0; ++u1 ) {
            const size_t u2     = u0 * params_.occupancyResolution_ + u1 * params_.occupancyPrecision_;
            bool         isFull = false;
            for ( size_t v3 = 0; v3 < params_.occupancyPrecision_ && !isFull; ++v3 ) {
              for ( size_t u3 = 0; u3 < params_.occupancyPrecision_ && !isFull; ++u3 ) {
                isFull |= occupancyMap[( v2 + v3 ) * imageWidth + u2 + u3] == 1;
              }
            }
            block0[v1 * blockSize0 + u1] = isFull;
            fullCount += isFull;
            for ( size_t v3 = 0; v3 < params_.occupancyPrecision_; ++v3 ) {
              for ( size_t u3 = 0; u3 < params_.occupancyPrecision_; ++u3 ) {
                occupancyMap[( v2 + v3 ) * imageWidth + u2 + u3] = isFull;
              }
            }
          }
        }
        for ( size_t iterBlockV = 0; iterBlockV < blockSize0; iterBlockV++ ) {
          for ( size_t iterBlockU = 0; iterBlockU < blockSize0; iterBlockU++ ) {
            uint8_t pixel       = block0[iterBlockV * blockSize0 + iterBlockU];
            size_t  videoFrameU = u0 * blockSize0 + iterBlockU;
            size_t  videoFrameV = v0 * blockSize0 + iterBlockV;
            videoFrameOccupancyMap.setValue( 0, videoFrameU, videoFrameV, pixel );
          }
        }
      }
    }
  } else {
    videoFrameOccupancyMap.resize( imageWidth, imageHeight );
    for ( size_t v = 0; v < imageHeight; v++ ) {
      for ( size_t u = 0; u < imageWidth; u++ ) {
        size_t i      = v * imageWidth + u;
        size_t symbol = occupancyMap[i];
        if ( symbol < 0 ) { symbol = 0; }
        if ( symbol > 1023 ) { symbol = 1023; }
        videoFrameOccupancyMap.setValue( 0, u, v, symbol );
      }
    }
  }
  return true;
}

void PCCEncoder::spatialConsistencyPack( PCCFrameContext& frame, PCCFrameContext& prevFrame, int safeguard ) {
  auto& width       = frame.getWidth();
  auto& height      = frame.getHeight();
  auto& patches     = frame.getPatches();
  auto& prevPatches = prevFrame.getPatches();
  if ( patches.empty() ) { return; }
  std::sort( patches.begin(), patches.end() );
  int    id             = 0;
  size_t occupancySizeU = params_.minimumImageWidth_ / params_.occupancyResolution_;
  size_t occupancySizeV =
      ( std::max )( params_.minimumImageHeight_ / params_.occupancyResolution_, patches[0].getSizeV0() );
  vector<PCCPatch> matchedPatches, tmpPatches;
  matchedPatches.clear();
  float thresholdIOU = 0.2f;

  // main loop.
  for ( auto& patch : prevPatches ) {
    assert( patch.getSizeU0() <= occupancySizeU );
    assert( patch.getSizeV0() <= occupancySizeV );
    id++;
    float maxIou  = 0.0f;
    int   bestIdx = -1, cId = 0;
    for ( auto& cpatch : patches ) {
      if ( ( patch.getViewId() == cpatch.getViewId() ) && ( cpatch.getBestMatchIdx() == InvalidPatchIndex ) ) {
        Rect  rect  = Rect( patch.getU1(), patch.getV1(), patch.getSizeU(), patch.getSizeV() );
        Rect  crect = Rect( cpatch.getU1(), cpatch.getV1(), cpatch.getSizeU(), cpatch.getSizeV() );
        float iou   = computeIOU( rect, crect );
        if ( iou > maxIou ) {
          maxIou  = iou;
          bestIdx = cId;
        }
      }  // end of if (patch.viewId == cpatch.viewId).
      cId++;
    }
    if ( maxIou > thresholdIOU ) {
      // store the best match index
      patches[bestIdx].setBestMatchIdx() = id - 1;  // the matched patch id in previous frame.
      matchedPatches.push_back( patches[bestIdx] );
    }
  }

  // generate new patch order.
  vector<PCCPatch> newOrderPatches = matchedPatches;
  for ( auto patch : patches ) {
    assert( patch.getSizeU0() <= occupancySizeU );
    assert( patch.getSizeV0() <= occupancySizeV );
    if ( patch.getBestMatchIdx() == InvalidPatchIndex ) { newOrderPatches.push_back( patch ); }
  }

  frame.getNumMatchedPatches() = matchedPatches.size();
  // remove the below logs when useless.
  patches = newOrderPatches;
  for ( auto& patch : patches ) { occupancySizeU = ( std::max )( occupancySizeU, patch.getSizeU0() + 1 ); }

  width  = occupancySizeU * params_.occupancyResolution_;
  height = occupancySizeV * params_.occupancyResolution_;
  size_t maxOccupancyRow{0};

  std::vector<bool> occupancyMap;
  occupancyMap.resize( occupancySizeU * occupancySizeV, false );
  for ( auto& patch : patches ) {
    assert( patch.getSizeU0() <= occupancySizeU );
    assert( patch.getSizeV0() <= occupancySizeV );
    bool  locationFound = false;
    auto& occupancy     = patch.getOccupancy();
    while ( !locationFound ) {
      patch.getPatchOrientation() = PatchOrientation::DEFAULT;  // only one orientation is allowed
      for ( size_t v = 0; v <= occupancySizeV && !locationFound; ++v ) {
        for ( size_t u = 0; u <= occupancySizeU && !locationFound; ++u ) {
          patch.getU0() = u;
          patch.getV0() = v;
          if ( patch.checkFitPatchCanvas( occupancyMap, occupancySizeU, occupancySizeV, safeguard ) ) {
            locationFound = true;
          }
        }
      }
      if ( !locationFound ) {
        occupancySizeV *= 2;
        occupancyMap.resize( occupancySizeU * occupancySizeV );
      }
    }
    for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
      const size_t v = patch.getV0() + v0;
      for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
        const size_t u = patch.getU0() + u0;
        occupancyMap[v * occupancySizeU + u] =
            occupancyMap[v * occupancySizeU + u] || occupancy[v0 * patch.getSizeU0() + u0];
      }
    }

    height          = ( std::max )( height, ( patch.getV0() + patch.getSizeV0() ) * patch.getOccupancyResolution() );
    width           = ( std::max )( width, ( patch.getU0() + patch.getSizeU0() ) * patch.getOccupancyResolution() );
    maxOccupancyRow = ( std::max )( maxOccupancyRow, ( patch.getV0() + patch.getSizeV0() ) );
    // print(occupancyMap, occupancySizeU, occupancySizeV);
  }

  if ( frame.getMissedPointsPatch().size() > 0 && !frame.getUseMissedPointsSeparateVideo() ) {
    packMissedPointsPatch( frame, occupancyMap, width, height, occupancySizeU, occupancySizeV, maxOccupancyRow );
  } else {
    if ( printDetailedInfo ) { printMap( occupancyMap, occupancySizeU, occupancySizeV ); }
  }
  std::cout << "actualImageSizeU " << width << std::endl;
  std::cout << "actualImageSizeV " << height << std::endl;
}

void PCCEncoder::spatialConsistencyPackFlexible( PCCFrameContext& frame, PCCFrameContext& prevFrame, int safeguard ) {
  auto& width   = frame.getWidth();
  auto& height  = frame.getHeight();
  auto& patches = frame.getPatches();

  auto& prevPatches = prevFrame.getPatches();
  if ( patches.empty() ) { return; }
  std::sort( patches.begin(), patches.end(), []( PCCPatch& a, PCCPatch& b ) { return a.gt( b ); } );
  int              id             = 0;
  size_t           occupancySizeU = params_.minimumImageWidth_ / params_.occupancyResolution_;
  size_t           occupancySizeV = ( std::max )( patches[0].getSizeU0(), patches[0].getSizeV0() );
  vector<PCCPatch> matchedPatches, tmpPatches;
  matchedPatches.clear();
  float thresholdIOU = 0.2f;

  // main loop.
  for ( auto& patch : prevPatches ) {
    assert( patch.getSizeU0() <= occupancySizeU );
    assert( patch.getSizeV0() <= occupancySizeV );
    id++;
    float maxIou  = 0.0f;
    int   bestIdx = -1, cId = 0;
    for ( auto& cpatch : patches ) {
      if ( ( patch.getViewId() == cpatch.getViewId() ) && ( cpatch.getBestMatchIdx() == InvalidPatchIndex ) ) {
        Rect  rect  = Rect( patch.getU1(), patch.getV1(), patch.getSizeU(), patch.getSizeV() );
        Rect  crect = Rect( cpatch.getU1(), cpatch.getV1(), cpatch.getSizeU(), cpatch.getSizeV() );
        float iou   = computeIOU( rect, crect );
        if ( iou > maxIou ) {
          maxIou  = iou;
          bestIdx = cId;
        }
      }  // end of if (patch.viewId == cpatch.viewId).
      cId++;
    }

    if ( maxIou > thresholdIOU ) {
      // store the best match index
      patches[bestIdx].setBestMatchIdx() = id - 1;  // the matched patch id in preivious frame.
      matchedPatches.push_back( patches[bestIdx] );
    }
  }

  // generate new patch order.
  vector<PCCPatch> newOrderPatches = matchedPatches;

  for ( auto patch : patches ) {
    assert( patch.getSizeU0() <= occupancySizeU );
    assert( patch.getSizeV0() <= occupancySizeV );
    if ( patch.getBestMatchIdx() == InvalidPatchIndex ) { newOrderPatches.push_back( patch ); }
  }
  frame.getNumMatchedPatches() = matchedPatches.size();

  // remove the below logs when useless.
  if ( printDetailedInfo ) {
    std::cout << "patches.size:" << patches.size() << ",reOrderedPatches.size:" << newOrderPatches.size()
              << ",matchedpatches.size:" << frame.getNumMatchedPatches() << std::endl;
  }
  patches = newOrderPatches;
  if ( printDetailedInfo ) {
    std::cout << "Patch order:" << std::endl;
    for ( auto& patch : patches ) {
      std::cout << "Patch[" << patch.getIndex() << "]=(" << patch.getSizeU0() << "," << patch.getSizeV0() << ")"
                << std::endl;
    }
  }

  for ( auto& patch : patches ) { occupancySizeU = ( std::max )( occupancySizeU, patch.getSizeU0() + 1 ); }

  width  = occupancySizeU * params_.occupancyResolution_;
  height = occupancySizeV * params_.occupancyResolution_;
  size_t maxOccupancyRow{0};

  // vector<int> orientation_vertical = {
  // PatchOrientation::DEFAULT,PatchOrientation::ROT90,PatchOrientation::ROT180,
  // PatchOrientation::ROT270,PatchOrientation::MIRROR,PatchOrientation::MROT180,
  // PatchOrientation::MROT90,PatchOrientation::MROT270
  // };    // favoring vertical orientation vector<int> orientation_horizontal = {
  // PatchOrientation::ROT90,PatchOrientation::DEFAULT,PatchOrientation::ROT270,
  // PatchOrientation::ROT180,PatchOrientation::MROT90,PatchOrientation::MROT270,
  // PatchOrientation::MIRROR,PatchOrientation::MROT180
  // };    // favoring horizontal orientations (that should be rotated)
  vector<int> orientation_vertical   = {PatchOrientation::DEFAULT,
                                      PatchOrientation::SWAP};  // favoring vertical orientation
  vector<int> orientation_horizontal = {
      PatchOrientation::SWAP, PatchOrientation::DEFAULT};  // favoring horizontal orientations (that should be rotated)
  std::vector<bool> occupancyMap;
  occupancyMap.resize( occupancySizeU * occupancySizeV, false );
  for ( auto& patch : patches ) {
    assert( patch.getSizeU0() <= occupancySizeU );
    assert( patch.getSizeV0() <= occupancySizeV );
    bool  locationFound = false;
    auto& occupancy     = patch.getOccupancy();
    while ( !locationFound ) {
      if ( patch.getBestMatchIdx() != InvalidPatchIndex ) {
        patch.getPatchOrientation() = prevPatches[patch.getBestMatchIdx()].getPatchOrientation();
        // try to place on the same position as the matched patch
        patch.getU0() = prevPatches[patch.getBestMatchIdx()].getU0();
        patch.getV0() = prevPatches[patch.getBestMatchIdx()].getV0();
        if ( patch.checkFitPatchCanvas( occupancyMap, occupancySizeU, occupancySizeV ) ) {
          locationFound = true;
          if ( printDetailedInfo ) {
            std::cout << "Maintained orientation " << patch.getPatchOrientation() << " for matched patch "
                      << patch.getIndex() << " in the same position (" << patch.getU0() << "," << patch.getV0() << ")"
                      << std::endl;
          }
        }
        // if the patch couldn't fit, try to fit the patch in the top left position
        for ( int v = 0; v <= occupancySizeV && !locationFound; ++v ) {
          for ( int u = 0; u <= occupancySizeU && !locationFound; ++u ) {
            patch.getU0() = u;
            patch.getV0() = v;
            if ( patch.checkFitPatchCanvas( occupancyMap, occupancySizeU, occupancySizeV, safeguard ) ) {
              locationFound = true;
              if ( printDetailedInfo ) {
                std::cout << "Maintained orientation " << patch.getPatchOrientation() << " for matched patch "
                          << patch.getIndex() << " (" << u << "," << v << ")" << std::endl;
              }
            }
          }
        }
      } else {
        // best effort
        for ( size_t v = 0; v < occupancySizeV && !locationFound; ++v ) {
          for ( size_t u = 0; u < occupancySizeU && !locationFound; ++u ) {
            patch.getU0() = u;
            patch.getV0() = v;
            for ( size_t orientationIdx = 0; orientationIdx < pow( 2, params_.packingStrategy_ ) && !locationFound;
                  orientationIdx++ ) {
              if ( patch.getSizeU0() > patch.getSizeV0() ) {
                patch.getPatchOrientation() = orientation_horizontal[orientationIdx];
              } else {
                patch.getPatchOrientation() = orientation_vertical[orientationIdx];
              }
              if ( patch.checkFitPatchCanvas( occupancyMap, occupancySizeU, occupancySizeV, safeguard ) ) {
                locationFound = true;
                if ( printDetailedInfo ) {
                  std::cout << "Orientation " << patch.getPatchOrientation() << " selected for unmatched patch "
                            << patch.getIndex() << " (" << u << "," << v << ")" << std::endl;
                }
              }
            }
          }
        }
      }
      if ( !locationFound ) {
        occupancySizeV *= 2;
        occupancyMap.resize( occupancySizeU * occupancySizeV );
      }
    }
    for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
      for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
        int coord           = patch.patchBlock2CanvasBlock( u0, v0, occupancySizeU, occupancySizeV );
        occupancyMap[coord] = occupancyMap[coord] || occupancy[v0 * patch.getSizeU0() + u0];
      }
    }
    if ( !( patch.isPatchDimensionSwitched() ) ) {
      height          = ( std::max )( height, ( patch.getV0() + patch.getSizeV0() ) * patch.getOccupancyResolution() );
      width           = ( std::max )( width, ( patch.getU0() + patch.getSizeU0() ) * patch.getOccupancyResolution() );
      maxOccupancyRow = ( std::max )( maxOccupancyRow, ( patch.getV0() + patch.getSizeV0() ) );
    } else {
      height          = ( std::max )( height, ( patch.getV0() + patch.getSizeU0() ) * patch.getOccupancyResolution() );
      width           = ( std::max )( width, ( patch.getU0() + patch.getSizeV0() ) * patch.getOccupancyResolution() );
      maxOccupancyRow = ( std::max )( maxOccupancyRow, ( patch.getV0() + patch.getSizeU0() ) );
    }
  }

  if ( frame.getMissedPointsPatch().size() > 0 && !frame.getUseMissedPointsSeparateVideo() ) {
    packMissedPointsPatch( frame, occupancyMap, width, height, occupancySizeU, occupancySizeV, maxOccupancyRow );
  } else {
    if ( printDetailedInfo ) { printMap( occupancyMap, occupancySizeU, occupancySizeV ); }
  }
  std::cout << "actualImageSizeU " << width << std::endl;
  std::cout << "actualImageSizeV " << height << std::endl;
}

void PCCEncoder::spatialConsistencyPackTetris( PCCFrameContext& frame, PCCFrameContext& prevFrame, int safeguard ) {
  auto& width   = frame.getWidth();
  auto& height  = frame.getHeight();
  auto& patches = frame.getPatches();

  auto& prevPatches = prevFrame.getPatches();
  if ( patches.empty() ) { return; }
  std::sort( patches.begin(), patches.end(), []( PCCPatch& a, PCCPatch& b ) { return a.gt( b ); } );
  int              id             = 0;
  size_t           occupancySizeU = params_.minimumImageWidth_ / params_.occupancyResolution_;
  size_t           occupancySizeV = ( std::max )( patches[0].getSizeU0(), patches[0].getSizeV0() );
  vector<PCCPatch> matchedPatches, tmpPatches;
  matchedPatches.clear();
  float thresholdIOU = 0.2;

  // main loop.
  for ( auto& patch : prevPatches ) {
    assert( patch.getSizeU0() <= occupancySizeU );
    assert( patch.getSizeV0() <= occupancySizeV );
    id++;
    float maxIou  = 0.0;
    int   bestIdx = -1, cId = 0;
    for ( auto& cpatch : patches ) {
      if ( ( patch.getViewId() == cpatch.getViewId() ) && ( cpatch.getBestMatchIdx() == -1 ) ) {
        Rect  rect  = Rect( patch.getU1(), patch.getV1(), patch.getSizeU(), patch.getSizeV() );
        Rect  crect = Rect( cpatch.getU1(), cpatch.getV1(), cpatch.getSizeU(), cpatch.getSizeV() );
        float iou   = computeIOU( rect, crect );
        if ( iou > maxIou ) {
          maxIou  = iou;
          bestIdx = cId;
        }
      }  // end of if (patch.viewId == cpatch.viewId).
      cId++;
    }

    if ( maxIou > thresholdIOU ) {
      // store the best match index
      patches[bestIdx].setBestMatchIdx() = id - 1;  // the matched patch id in preivious frame.
      matchedPatches.push_back( patches[bestIdx] );
    }
  }

  // generate new patch order.
  vector<PCCPatch> newOrderPatches = matchedPatches;

  for ( auto patch : patches ) {
    assert( patch.getSizeU0() <= occupancySizeU );
    assert( patch.getSizeV0() <= occupancySizeV );
    if ( patch.getBestMatchIdx() == -1 ) { newOrderPatches.push_back( patch ); }
  }
  frame.getNumMatchedPatches() = matchedPatches.size();

  // remove the below logs when useless.
  if ( printDetailedInfo ) {
    std::cout << "patches.size:" << patches.size() << ",reOrderedPatches.size:" << newOrderPatches.size()
              << ",matchedpatches.size:" << frame.getNumMatchedPatches() << std::endl;
  }
  patches = newOrderPatches;
  if ( printDetailedInfo ) {
    std::cout << "Patch order:" << std::endl;
    for ( auto& patch : patches ) {
      std::cout << "Patch[" << patch.getIndex() << "]=(" << patch.getSizeU0() << "," << patch.getSizeV0() << ")"
                << std::endl;
    }
  }

  for ( auto& patch : patches ) { occupancySizeU = ( std::max )( occupancySizeU, patch.getSizeU0() + 1 ); }

  width  = occupancySizeU * params_.occupancyResolution_;
  height = occupancySizeV * params_.occupancyResolution_;
  size_t maxOccupancyRow{0};

  std::vector<bool> occupancyMap;
  occupancyMap.resize( occupancySizeU * occupancySizeV, false );
  std::vector<int> horizon;
  horizon.resize( occupancySizeU, 0 );

  for ( auto& patch : patches ) {
    assert( patch.getSizeU0() <= occupancySizeU );
    assert( patch.getSizeV0() <= occupancySizeV );
    auto& occupancy = patch.getOccupancy();
    // getting the horizons using the rotation 0 position
    std::vector<int> top_horizon;
    std::vector<int> bottom_horizon;
    std::vector<int> right_horizon;
    std::vector<int> left_horizon;
    patch.get_patch_horizons( top_horizon, bottom_horizon, right_horizon, left_horizon );

    bool locationFound = false;
    while ( !locationFound ) {
      int    best_wasted_space = ( std::numeric_limits<int>::max )();
      size_t best_u, best_v;
      int    best_orientation;
      if ( patch.getBestMatchIdx() != -1 ) {
        patch.getPatchOrientation() = prevPatches[patch.getBestMatchIdx()].getPatchOrientation();
        best_orientation            = patch.getPatchOrientation();
        // spiral search to find the closest available position
        int x   = 0;
        int y   = 0;
        int end = ( std::max )( occupancySizeU, occupancySizeV ) * ( std::max )( occupancySizeU, occupancySizeV ) * 4;
        for ( int i = 0; i < end && !locationFound; ++i ) {
          // Translate coordinates and mask them out.
          int xp = x + prevPatches[patch.getBestMatchIdx()].getU0();
          int yp = y + prevPatches[patch.getBestMatchIdx()].getV0();
          if ( printDetailedInfo ) std::cout << "Testing position (" << xp << ',' << yp << ')' << std::endl;
          if ( xp >= 0 && xp < occupancySizeU && yp >= 0 && yp < occupancySizeV ) {
            patch.getU0() = xp;
            patch.getV0() = yp;
            if ( patch.checkFitPatchCanvas( occupancyMap, occupancySizeU, occupancySizeV, safeguard ) ) {
              locationFound = true;
              best_u        = xp;
              best_v        = yp;
              if ( printDetailedInfo )
                std::cout << "Maintained orientation " << patch.getPatchOrientation() << " for matched patch "
                          << patch.getIndex() << " in new position (" << xp << "," << yp << ")" << std::endl;
            }
          }
          if ( abs( x ) <= abs( y ) && ( x != y || x >= 0 ) )
            x += ( ( y >= 0 ) ? 1 : -1 );
          else
            y += ( ( x >= 0 ) ? -1 : 1 );
        }
      } else {
        // vector<int> orientation_values = {
        // PatchOrientation::DEFAULT,PatchOrientation::ROT90,PatchOrientation::ROT180,PatchOrientation::ROT270,PatchOrientation::MIRROR,PatchOrientation::MROT180,PatchOrientation::MROT90,PatchOrientation::MROT270
        // };    // favoring vertical orientation
        vector<int> orientation_values = {PatchOrientation::DEFAULT,
                                          PatchOrientation::SWAP};  // favoring vertical orientation
        // tetris packing
        for ( size_t u = 0; u < occupancySizeU; ++u ) {
          for ( size_t v = 0; v < occupancySizeV; ++v ) {
            patch.getU0() = u;
            patch.getV0() = v;
            for ( size_t orientationIdx = 0; orientationIdx < pow( 2, params_.packingStrategy_ - 1 );
                  orientationIdx++ ) {
              patch.getPatchOrientation() = orientation_values[orientationIdx];
              if ( !patch.isPatchLocationAboveHorizon( horizon, top_horizon, bottom_horizon, right_horizon,
                                                       left_horizon ) ) {
                if ( printDetailedInfo )
                  std::cout << "(" << u << "," << v << "|" << patch.getPatchOrientation() << ") above horizon"
                            << std::endl;
                continue;
              }
              if ( patch.checkFitPatchCanvas( occupancyMap, occupancySizeU, occupancySizeV, safeguard ) ) {
                // now calculate the wasted space
                int wasted_space =
                    patch.calculate_wasted_space( horizon, top_horizon, bottom_horizon, right_horizon, left_horizon );
                if ( wasted_space < best_wasted_space ) {
                  best_wasted_space = wasted_space;
                  best_u            = u;
                  best_v            = v;
                  best_orientation  = patch.getPatchOrientation();
                  locationFound     = true;
                }
              }
            }
          }
        }
      }
      if ( !locationFound ) {
        occupancySizeV *= 2;
        occupancyMap.resize( occupancySizeU * occupancySizeV );
      } else {
        // select the best position and orientation
        patch.getU0()               = best_u;
        patch.getV0()               = best_v;
        patch.getPatchOrientation() = best_orientation;
        if ( printDetailedInfo )
          std::cout << "Selected position (" << best_u << "," << best_v << ") and orientation " << best_orientation
                    << std::endl;
        // update the horizon
        patch.update_horizon( horizon, top_horizon, bottom_horizon, right_horizon, left_horizon );
        // debugging
        if ( printDetailedInfo ) {
          std::cout << "New Horizon :[";
          for ( int i = 0; i < occupancySizeU; i++ ) { std::cout << horizon[i] << ","; }
          std::cout << "]" << std::endl;
        }
      }
    }
    for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
      for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
        int coord           = patch.patchBlock2CanvasBlock( u0, v0, occupancySizeU, occupancySizeV );
        occupancyMap[coord] = occupancyMap[coord] || occupancy[v0 * patch.getSizeU0() + u0];
      }
    }
    if ( !( patch.isPatchDimensionSwitched() ) ) {
      height          = ( std::max )( height, ( patch.getV0() + patch.getSizeV0() ) * patch.getOccupancyResolution() );
      width           = ( std::max )( width, ( patch.getU0() + patch.getSizeU0() ) * patch.getOccupancyResolution() );
      maxOccupancyRow = ( std::max )( maxOccupancyRow, ( patch.getV0() + patch.getSizeV0() ) );
    } else {
      height          = ( std::max )( height, ( patch.getV0() + patch.getSizeU0() ) * patch.getOccupancyResolution() );
      width           = ( std::max )( width, ( patch.getU0() + patch.getSizeV0() ) * patch.getOccupancyResolution() );
      maxOccupancyRow = ( std::max )( maxOccupancyRow, ( patch.getV0() + patch.getSizeU0() ) );
    }
    if ( printDetailedInfo ) { printMapTetris( occupancyMap, occupancySizeU, occupancySizeV, horizon ); }
  }

  if ( frame.getMissedPointsPatch().size() > 0 && !frame.getUseMissedPointsSeparateVideo() ) {
    packMissedPointsPatch( frame, occupancyMap, width, height, occupancySizeU, occupancySizeV, maxOccupancyRow );
  } else {
    if ( printDetailedInfo ) { printMap( occupancyMap, occupancySizeU, occupancySizeV ); }
  }
  std::cout << "actualImageSizeU " << width << std::endl;
  std::cout << "actualImageSizeV " << height << std::endl;
}

void PCCEncoder::pack( PCCFrameContext& frame, int safeguard ) {
  auto& width   = frame.getWidth();
  auto& height  = frame.getHeight();
  auto& patches = frame.getPatches();
  if ( patches.empty() ) { return; }
  std::sort( patches.begin(), patches.end() );
  size_t occupancySizeU = params_.minimumImageWidth_ / params_.occupancyResolution_;
  size_t occupancySizeV =
      ( std::max )( params_.minimumImageHeight_ / params_.occupancyResolution_, patches[0].getSizeV0() );
  for ( auto& patch : patches ) { occupancySizeU = ( std::max )( occupancySizeU, patch.getSizeU0() + 1 ); }

  width  = occupancySizeU * params_.occupancyResolution_;
  height = occupancySizeV * params_.occupancyResolution_;
  size_t maxOccupancyRow{0};

  std::vector<bool> occupancyMap;
  occupancyMap.resize( occupancySizeU * occupancySizeV, false );
  for ( auto& patch : patches ) {
    assert( patch.getSizeU0() <= occupancySizeU );
    assert( patch.getSizeV0() <= occupancySizeV );
    bool  locationFound = false;
    auto& occupancy     = patch.getOccupancy();
    while ( !locationFound ) {
      patch.getPatchOrientation() = PatchOrientation::DEFAULT;  // only allowed orientation in anchor
      for ( int v = 0; v <= occupancySizeV && !locationFound; ++v ) {
        for ( int u = 0; u <= occupancySizeU && !locationFound; ++u ) {
          patch.getU0() = u;
          patch.getV0() = v;
          if ( patch.checkFitPatchCanvas( occupancyMap, occupancySizeU, occupancySizeV, safeguard ) ) {
            locationFound = true;
          }
        }
      }
      if ( !locationFound ) {
        occupancySizeV *= 2;
        occupancyMap.resize( occupancySizeU * occupancySizeV );
      }
    }
    for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
      const size_t v = patch.getV0() + v0;
      for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
        const size_t u = patch.getU0() + u0;
        occupancyMap[v * occupancySizeU + u] =
            occupancyMap[v * occupancySizeU + u] || occupancy[v0 * patch.getSizeU0() + u0];
      }
    }

    height          = ( std::max )( height, ( patch.getV0() + patch.getSizeV0() ) * patch.getOccupancyResolution() );
    width           = ( std::max )( width, ( patch.getU0() + patch.getSizeU0() ) * patch.getOccupancyResolution() );
    maxOccupancyRow = ( std::max )( maxOccupancyRow, ( patch.getV0() + patch.getSizeV0() ) );
    // print(occupancyMap, occupancySizeU, occupancySizeV);
  }

  if ( frame.getMissedPointsPatch().size() > 0 && !frame.getUseMissedPointsSeparateVideo() ) {
    packMissedPointsPatch( frame, occupancyMap, width, height, occupancySizeU, occupancySizeV, maxOccupancyRow );
  } else {
    if ( printDetailedInfo ) { printMap( occupancyMap, occupancySizeU, occupancySizeV ); }
  }
  std::cout << "actualImageSizeU " << width << std::endl;
  std::cout << "actualImageSizeV " << height << std::endl;
}

void PCCEncoder::packFlexible( PCCFrameContext& frame, int safeguard ) {
  auto& width   = frame.getWidth();
  auto& height  = frame.getHeight();
  auto& patches = frame.getPatches();
  // set no matched patches, since this function does not take into account the previous frame
  frame.getNumMatchedPatches() = 0;
  if ( patches.empty() ) { return; }
  // sorting by patch largest dimension
  std::sort( patches.begin(), patches.end(), []( PCCPatch& a, PCCPatch& b ) { return a.gt( b ); } );
  if ( printDetailedInfo ) {
    std::cout << "Patch order:" << std::endl;
    for ( auto& patch : patches ) {
      std::cout << "Patch[" << patch.getIndex() << "]=(" << patch.getSizeU0() << "," << patch.getSizeV0() << ")"
                << std::endl;
    }
  }

  size_t occupancySizeU = params_.minimumImageWidth_ / params_.occupancyResolution_;
  size_t occupancySizeV = ( std::max )( patches[0].getSizeV0(), patches[0].getSizeU0() );
  for ( auto& patch : patches ) { occupancySizeU = ( std::max )( occupancySizeU, patch.getSizeU0() + 1 ); }

  width  = occupancySizeU * params_.occupancyResolution_;
  height = occupancySizeV * params_.occupancyResolution_;
  size_t maxOccupancyRow{0};

  std::vector<bool> occupancyMap;
  // vector<int> orientation_vertical = {
  // PatchOrientation::DEFAULT,PatchOrientation::ROT90,PatchOrientation::ROT180,PatchOrientation::ROT270,PatchOrientation::MIRROR,PatchOrientation::MROT180,PatchOrientation::MROT90,PatchOrientation::MROT270
  // };    // favoring vertical orientation vector<int> orientation_horizontal = {
  // PatchOrientation::ROT90,PatchOrientation::DEFAULT,PatchOrientation::ROT270,PatchOrientation::ROT180,PatchOrientation::MROT90,PatchOrientation::MROT270,PatchOrientation::MIRROR,PatchOrientation::MROT180
  // };    // favoring horizontal orientations (that should be rotated)
  vector<int> orientation_vertical   = {PatchOrientation::DEFAULT,
                                      PatchOrientation::SWAP};  // favoring vertical orientation
  vector<int> orientation_horizontal = {
      PatchOrientation::SWAP, PatchOrientation::DEFAULT};  // favoring horizontal orientations (that should be rotated)
  occupancyMap.resize( occupancySizeU * occupancySizeV, false );
  for ( auto& patch : patches ) {
    assert( patch.getSizeU0() <= occupancySizeU );
    assert( patch.getSizeV0() <= occupancySizeV );
    bool  locationFound = false;
    auto& occupancy     = patch.getOccupancy();
    while ( !locationFound ) {
      for ( size_t v = 0; v < occupancySizeV && !locationFound; ++v ) {
        for ( size_t u = 0; u < occupancySizeU && !locationFound; ++u ) {
          patch.getU0() = u;
          patch.getV0() = v;
          for ( size_t orientationIdx = 0; orientationIdx < pow( 2, params_.packingStrategy_ ) && !locationFound;
                orientationIdx++ ) {
            if ( patch.getSizeU0() > patch.getSizeV0() ) {
              patch.getPatchOrientation() = orientation_horizontal[orientationIdx];
            } else {
              patch.getPatchOrientation() = orientation_vertical[orientationIdx];
            }
            if ( patch.checkFitPatchCanvas( occupancyMap, occupancySizeU, occupancySizeV, safeguard ) ) {
              locationFound = true;
              if ( printDetailedInfo ) {
                std::cout << "Orientation " << patch.getPatchOrientation() << " selected for patch " << patch.getIndex()
                          << " (" << u << "," << v << ")" << std::endl;
              }
            }
          }
        }
      }
      if ( !locationFound ) {
        occupancySizeV *= 2;
        occupancyMap.resize( occupancySizeU * occupancySizeV );
      }
    }
    for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
      for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
        int coord           = patch.patchBlock2CanvasBlock( u0, v0, occupancySizeU, occupancySizeV );
        occupancyMap[coord] = occupancyMap[coord] || occupancy[v0 * patch.getSizeU0() + u0];
      }
    }

    if ( !( patch.isPatchDimensionSwitched() ) ) {
      height          = ( std::max )( height, ( patch.getV0() + patch.getSizeV0() ) * patch.getOccupancyResolution() );
      width           = ( std::max )( width, ( patch.getU0() + patch.getSizeU0() ) * patch.getOccupancyResolution() );
      maxOccupancyRow = ( std::max )( maxOccupancyRow, ( patch.getV0() + patch.getSizeV0() ) );
    } else {
      height          = ( std::max )( height, ( patch.getV0() + patch.getSizeU0() ) * patch.getOccupancyResolution() );
      width           = ( std::max )( width, ( patch.getU0() + patch.getSizeV0() ) * patch.getOccupancyResolution() );
      maxOccupancyRow = ( std::max )( maxOccupancyRow, ( patch.getV0() + patch.getSizeU0() ) );
    }
  }

  if ( frame.getMissedPointsPatch().size() > 0 && !frame.getUseMissedPointsSeparateVideo() ) {
    packMissedPointsPatch( frame, occupancyMap, width, height, occupancySizeU, occupancySizeV, maxOccupancyRow );
  } else {
    if ( printDetailedInfo ) { printMap( occupancyMap, occupancySizeU, occupancySizeV ); }
  }
  std::cout << "actualImageSizeU " << width << std::endl;
  std::cout << "actualImageSizeV " << height << std::endl;
}

void PCCEncoder::packTetris( PCCFrameContext& frame, int safeguard ) {
  auto& width   = frame.getWidth();
  auto& height  = frame.getHeight();
  auto& patches = frame.getPatches();
  // set no matched patches, since this function does not take into account the previous frame
  frame.getNumMatchedPatches() = 0;
  if ( patches.empty() ) { return; }
  // sorting by patch largest dimension
  std::sort( patches.begin(), patches.end(), []( PCCPatch& a, PCCPatch& b ) { return a.gt( b ); } );
  if ( printDetailedInfo ) {
    std::cout << "Patch order:" << std::endl;
    for ( auto& patch : patches ) {
      std::cout << "Patch[" << patch.getIndex() << "]=(" << patch.getSizeU0() << "," << patch.getSizeV0() << ")"
                << std::endl;
    }
  }
  size_t occupancySizeU = params_.minimumImageWidth_ / params_.occupancyResolution_;
  size_t occupancySizeV = ( std::max )( patches[0].getSizeV0(), patches[0].getSizeU0() );
  for ( auto& patch : patches ) { occupancySizeU = ( std::max )( occupancySizeU, patch.getSizeU0() + 1 ); }

  width  = occupancySizeU * params_.occupancyResolution_;
  height = occupancySizeV * params_.occupancyResolution_;
  size_t maxOccupancyRow{0};

  std::vector<bool> occupancyMap;
  occupancyMap.resize( occupancySizeU * occupancySizeV, false );
  std::vector<int> horizon;
  horizon.resize( occupancySizeU, 0 );
  if ( printDetailedInfo ) {
    std::cout << "Horizon :[";
    for ( int i = 0; i < occupancySizeU; i++ ) { std::cout << horizon[i] << ","; }
    std::cout << "]" << std::endl;
  }

  for ( auto& patch : patches ) {
    assert( patch.getSizeU0() <= occupancySizeU );
    assert( patch.getSizeV0() <= occupancySizeV );
    auto& occupancy = patch.getOccupancy();

    // getting the horizons using the rotation 0 position
    if ( printDetailedInfo ) patch.print();
    std::vector<int> top_horizon;
    std::vector<int> bottom_horizon;
    std::vector<int> right_horizon;
    std::vector<int> left_horizon;
    patch.get_patch_horizons( top_horizon, bottom_horizon, right_horizon, left_horizon );

    bool locationFound = false;
    // try to place the patch tetris-style
    // vector<int> orientation_values = {
    // PatchOrientation::DEFAULT,PatchOrientation::ROT90,PatchOrientation::ROT180,PatchOrientation::ROT270,PatchOrientation::MIRROR,PatchOrientation::MROT180,PatchOrientation::MROT90,PatchOrientation::MROT270
    // };    // favoring vertical orientation
    vector<int> orientation_values = {
        PatchOrientation::DEFAULT,
        PatchOrientation::SWAP};  // favoring horizontal orientations (that should be rotated)
    while ( !locationFound ) {
      int    best_wasted_space = ( std::numeric_limits<int>::max )();
      size_t best_u, best_v;
      int    best_orientation;
      for ( size_t u = 0; u < occupancySizeU; ++u ) {
        for ( size_t v = 0; v < occupancySizeV; ++v ) {
          patch.getU0() = u;
          patch.getV0() = v;
          for ( size_t orientationIdx = 0; orientationIdx < pow( 2, params_.packingStrategy_ - 1 ); orientationIdx++ ) {
            patch.getPatchOrientation() = orientation_values[orientationIdx];
            if ( !patch.isPatchLocationAboveHorizon( horizon, top_horizon, bottom_horizon, right_horizon,
                                                     left_horizon ) ) {
              if ( printDetailedInfo )
                std::cout << "(" << u << "," << v << "|" << patch.getPatchOrientation() << ") above horizon"
                          << std::endl;
              continue;
            }
            if ( printDetailedInfo )
              std::cout << "(" << u << "," << v << "|" << patch.getPatchOrientation() << ")" << std::endl;
            if ( patch.checkFitPatchCanvas( occupancyMap, occupancySizeU, occupancySizeV, safeguard ) ) {
              // now calculate the wasted space
              int wasted_space =
                  patch.calculate_wasted_space( horizon, top_horizon, bottom_horizon, right_horizon, left_horizon );
              if ( printDetailedInfo ) std::cout << "(wasted space) = " << wasted_space << std::endl;
              if ( wasted_space < best_wasted_space ) {
                best_wasted_space = wasted_space;
                best_u            = u;
                best_v            = v;
                best_orientation  = patch.getPatchOrientation();
                locationFound     = true;
              }
            }
          }
        }
      }
      if ( !locationFound ) {
        occupancySizeV *= 2;
        occupancyMap.resize( occupancySizeU * occupancySizeV );
        if ( printDetailedInfo )
          std::cout << "Increasing frame size (" << occupancySizeU << "," << occupancySizeV << ")" << std::endl;
      } else {
        // select the best position and orientation
        patch.getU0()               = best_u;
        patch.getV0()               = best_v;
        patch.getPatchOrientation() = best_orientation;
        if ( printDetailedInfo )
          std::cout << "Selected position (" << best_u << "," << best_v << ") and orientation " << best_orientation
                    << "(wasted space=" << best_wasted_space << ")" << std::endl;
        // update the horizon
        patch.update_horizon( horizon, top_horizon, bottom_horizon, right_horizon, left_horizon );
        // debugging
        if ( printDetailedInfo ) {
          std::cout << "Horizon :[";
          for ( int i = 0; i < occupancySizeU; i++ ) { std::cout << horizon[i] << ","; }
          std::cout << "]" << std::endl;
        }
      }
    }
    for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
      for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
        int coord           = patch.patchBlock2CanvasBlock( u0, v0, occupancySizeU, occupancySizeV );
        occupancyMap[coord] = occupancyMap[coord] || occupancy[v0 * patch.getSizeU0() + u0];
      }
    }
    if ( !( patch.isPatchDimensionSwitched() ) ) {
      height          = ( std::max )( height, ( patch.getV0() + patch.getSizeV0() ) * patch.getOccupancyResolution() );
      width           = ( std::max )( width, ( patch.getU0() + patch.getSizeU0() ) * patch.getOccupancyResolution() );
      maxOccupancyRow = ( std::max )( maxOccupancyRow, ( patch.getV0() + patch.getSizeV0() ) );
    } else {
      height          = ( std::max )( height, ( patch.getV0() + patch.getSizeU0() ) * patch.getOccupancyResolution() );
      width           = ( std::max )( width, ( patch.getU0() + patch.getSizeV0() ) * patch.getOccupancyResolution() );
      maxOccupancyRow = ( std::max )( maxOccupancyRow, ( patch.getV0() + patch.getSizeU0() ) );
    }

    if ( printDetailedInfo ) { printMapTetris( occupancyMap, occupancySizeU, occupancySizeV, horizon ); }
  }

  if ( frame.getMissedPointsPatch().size() > 0 ) {
    packMissedPointsPatch( frame, occupancyMap, width, height, occupancySizeU, occupancySizeV, maxOccupancyRow );
  } else {
    if ( printDetailedInfo ) printMap( occupancyMap, occupancySizeU, occupancySizeV );
  }
  std::cout << "actualImageSizeU " << width << std::endl;
  std::cout << "actualImageSizeV " << height << std::endl;
}

void PCCEncoder::packMissedPointsPatch( PCCFrameContext&         frame,
                                        const std::vector<bool>& occupancyMap,
                                        size_t&                  width,
                                        size_t&                  height,
                                        size_t                   occupancySizeU,
                                        size_t                   occupancySizeV,
                                        size_t                   maxOccupancyRow ) {
  auto& missedPointsPatch        = frame.getMissedPointsPatch();
  missedPointsPatch.v0_          = maxOccupancyRow;
  missedPointsPatch.u0_          = 0;
  size_t missedPointsPatchBlocks = static_cast<size_t>(
      ceil( double( missedPointsPatch.size() ) / ( params_.occupancyResolution_ * params_.occupancyResolution_ ) ) );
  size_t missedPointsPatchBlocksV = static_cast<size_t>( ceil( double( missedPointsPatchBlocks ) / occupancySizeU ) );
  int    occupancyRows2Add =
      static_cast<int>( maxOccupancyRow + missedPointsPatchBlocksV - height / params_.occupancyResolution_ );
  occupancyRows2Add = occupancyRows2Add > 0 ? occupancyRows2Add : 0;
  occupancySizeV += occupancyRows2Add;
  height += occupancyRows2Add * params_.occupancyResolution_;

  vector<bool> newOccupancyMap;
  newOccupancyMap.resize( occupancySizeU * occupancySizeV );

  size_t missedPointsPatchBlocksU =
      static_cast<size_t>( ceil( double( missedPointsPatchBlocks ) / missedPointsPatchBlocksV ) );
  missedPointsPatch.sizeV_ = missedPointsPatchBlocksV * params_.occupancyResolution_;
  missedPointsPatch.sizeU_ =
      static_cast<size_t>( ceil( double( missedPointsPatch.size() ) / missedPointsPatch.sizeV_ ) );
  missedPointsPatch.sizeV0_ = missedPointsPatchBlocksV;
  missedPointsPatch.sizeU0_ = missedPointsPatchBlocksU;

  const int16_t infiniteValue = ( std::numeric_limits<int16_t>::max )();
  missedPointsPatch.resize( missedPointsPatch.sizeU_ * missedPointsPatch.sizeV_, infiniteValue );
  std::vector<bool>& missedPointPatchOccupancy = missedPointsPatch.occupancy_;
  missedPointPatchOccupancy.resize( missedPointsPatch.sizeU0_ * missedPointsPatch.sizeV0_, false );

  for ( size_t v = 0; v < missedPointsPatch.sizeV_; ++v ) {
    for ( size_t u = 0; u < missedPointsPatch.sizeU_; ++u ) {
      const size_t p = v * missedPointsPatch.sizeU_ + u;
      if ( missedPointsPatch.x_[p] < infiniteValue ) {
        const size_t u0 = u / missedPointsPatch.occupancyResolution_;
        const size_t v0 = v / missedPointsPatch.occupancyResolution_;
        const size_t p0 = v0 * missedPointsPatch.sizeU0_ + u0;
        assert( u0 >= 0 && u0 < missedPointsPatch.sizeU0_ );
        assert( v0 >= 0 && v0 < missedPointsPatch.sizeV0_ );
        missedPointPatchOccupancy[p0] = true;
      }
    }
  }

  for ( size_t v0 = 0; v0 < missedPointsPatch.sizeV0_; ++v0 ) {
    const size_t v = missedPointsPatch.v0_ + v0;
    for ( size_t u0 = 0; u0 < missedPointsPatch.sizeU0_; ++u0 ) {
      const size_t u = missedPointsPatch.u0_ + u0;
      newOccupancyMap[v * occupancySizeU + u] =
          newOccupancyMap[v * occupancySizeU + u] || missedPointPatchOccupancy[v0 * missedPointsPatch.sizeU0_ + u0];
    }
  }

  // printMap(newOccupancyMap, occupancySizeU, occupancySizeV);
  for ( size_t v = 0; v < maxOccupancyRow; ++v ) {
    for ( size_t u = 0; u < occupancySizeU; ++u ) {
      const size_t p     = v * occupancySizeU + u;
      newOccupancyMap[p] = occupancyMap[p];
    }
  }

  printMap( newOccupancyMap, occupancySizeU, occupancySizeV );
}

bool PCCEncoder::generateGeometryVideo( const PCCPointSet3&                source,
                                        PCCFrameContext&                   frame,
                                        const PCCPatchSegmenter3Parameters segmenterParams,
                                        PCCVideoGeometry&                  videoGeometry,
                                        PCCFrameContext&                   prevFrame,
                                        size_t                             frameIndex,
                                        float&                             distanceSrcRec ) {
  if ( !source.getPointCount() ) { return false; }
  auto& patches = frame.getPatches();
  patches.reserve( 256 );
  PCCPatchSegmenter3 segmenter;
  segmenter.setNbThread( params_.nbThread_ );
  segmenter.compute( source, segmenterParams, patches, frame.getSrcPointCloudByPatch(), distanceSrcRec );
  auto& patchLevelMetadataEnabledFlags = frame.getFrameLevelMetadata().getLowerLevelMetadataEnabledFlags();

  if ( params_.useAdditionalPointsPatch_ ) {
    generateMissedPointsPatch( source, frame,
                               segmenterParams.useEnhancedDeltaDepthCode );  // useEnhancedDeltaDepthCode for EDD code
    sortMissedPointsPatch( frame );
  }

  if ( params_.testLevelOfDetail_ > 0 ) {
    for ( size_t i = 0; i < patches.size(); i++ ) { patches[i].getLod() = params_.testLevelOfDetail_; }
  }

  else if ( params_.testLevelOfDetailSignaling_ > 0 ) {  // generate semi-random levels of detail for testing
    srand( frame.getPatches().size() );                  // use a deterministic seed based on frame size
    for ( size_t i = 0; i < patches.size(); i++ ) {
      patches[i].getLod() = rand() % params_.testLevelOfDetailSignaling_;
    }
  }

  if ( params_.packingStrategy_ == 0 ) {
    if ( ( frameIndex == 0 ) || ( !params_.constrainedPack_ ) ) {
      pack( frame, params_.safeGuardDistance_ );
    } else {
      spatialConsistencyPack( frame, prevFrame, params_.safeGuardDistance_ );
    }
  } else {
    if ( params_.packingStrategy_ == 1 ) {
      if ( ( frameIndex == 0 ) || ( !params_.constrainedPack_ ) ) {
        packFlexible( frame, params_.safeGuardDistance_ );
      } else {
        spatialConsistencyPackFlexible( frame, prevFrame, params_.safeGuardDistance_ );
      }
    } else {
      if ( params_.packingStrategy_ == 2 ) {
        if ( ( frameIndex == 0 ) || ( !params_.constrainedPack_ ) ) {
          packTetris( frame, params_.safeGuardDistance_ );
        } else {
          spatialConsistencyPackTetris( frame, prevFrame, params_.safeGuardDistance_ );
        }
      }
    }
  }

  if ( patchLevelMetadataEnabledFlags.getMetadataEnabled() ) {
    for ( size_t i = 0; i < patches.size(); i++ ) {
      // Place to get/set patch-level metadata.
      PCCMetadata patchLevelMetadata;
      patchLevelMetadata.setMetadataPresent( true );
      patchLevelMetadataEnabledFlags.setMetadataEnabled( true );
      patchLevelMetadata.setMetadataType( METADATA_PATCH );
      patchLevelMetadata.getLowerLevelMetadataEnabledFlags().setMetadataEnabled( false );
      patches[i].getPatchLevelMetadata()                           = patchLevelMetadata;
      patches[i].getPatchLevelMetadata().getMetadataEnabledFlags() = patchLevelMetadataEnabledFlags;
    }
  }
  return true;
}

void PCCEncoder::geometryGroupDilation( PCCContext& context ) {
  auto& videoGeometry = context.getVideoGeometry();
  auto& frames        = context.getFrames();
  for ( size_t f = 0; f < frames.size(); ++f ) {
    auto& frame        = frames[f];
    auto& width        = frame.getWidth();
    auto& height       = frame.getHeight();
    auto& occupancyMap = frame.getOccupancyMap();
    auto& frame1       = videoGeometry.getFrame( 2 * f );
    auto& frame2       = videoGeometry.getFrame( 2 * f + 1 );
    for ( size_t y = 0; y < height; y++ ) {
      for ( size_t x = 0; x < width; x++ ) {
        const size_t pos = y * width + x;
        if ( occupancyMap[pos] == 0 ) {
          uint32_t avg =
              ( ( (uint32_t)frame1.getValue( 0, x, y ) ) + ( (uint32_t)frame2.getValue( 0, x, y ) ) + 1 ) >> 1;
          frame1.setValue( 0, x, y, (uint16_t)avg );
          frame2.setValue( 0, x, y, (uint16_t)avg );
        }
      }
    }
  }
}

void PCCEncoder::generateOccupancyMap( PCCFrameContext& frame ) {
  auto& occupancyMap     = frame.getOccupancyMap();
  auto& fullOccupancyMap = frame.getFullOccupancyMap();
  auto& width            = frame.getWidth();
  auto& height           = frame.getHeight();
  occupancyMap.resize( width * height, 0 );
  if ( !params_.absoluteD1_ ) { fullOccupancyMap.resize( width * height, 0 ); }
  const int16_t infiniteDepth = ( std::numeric_limits<int16_t>::max )();
  for ( auto& patch : frame.getPatches() ) {
    const size_t v0 = patch.getV0() * patch.getOccupancyResolution();
    const size_t u0 = patch.getU0() * patch.getOccupancyResolution();
    for ( size_t v = 0; v < patch.getSizeV(); ++v ) {
      for ( size_t u = 0; u < patch.getSizeU(); ++u ) {
        const size_t  p = v * patch.getSizeU() + u;
        const int16_t d = patch.getDepth( 0 )[p];
        if ( d < infiniteDepth ) {
          size_t x;
          size_t y;
          occupancyMap[patch.patch2Canvas( u, v, width, height, x, y )] = 1;
        }
      }
    }
  }

  if ( !frame.getUseMissedPointsSeparateVideo() ) {
    auto&        missedPointsPatch = frame.getMissedPointsPatch();
    const size_t v0                = missedPointsPatch.v0_ * missedPointsPatch.occupancyResolution_;
    const size_t u0                = missedPointsPatch.u0_ * missedPointsPatch.occupancyResolution_;
    if ( missedPointsPatch.size() ) {
      for ( size_t v = 0; v < missedPointsPatch.sizeV_; ++v ) {
        for ( size_t u = 0; u < missedPointsPatch.sizeU_; ++u ) {
          const size_t p = v * missedPointsPatch.sizeU_ + u;
          if ( missedPointsPatch.x_[p] < infiniteDepth ) {
            const size_t x = ( u0 + u );
            const size_t y = ( v0 + v );
            assert( x < width && y < height );
            occupancyMap[x + y * width] = 1;
          }
        }
      }
    }
  }
  if ( !params_.absoluteD1_ ) { fullOccupancyMap = occupancyMap; }
}

void PCCEncoder::modifyOccupancyMap( PCCFrameContext&        frame,
                                     const PCCImageGeometry& imageRef,
                                     const PCCImageGeometry& image ) {
  auto& occupancyMap     = frame.getOccupancyMap();
  auto& fullOccupancyMap = frame.getFullOccupancyMap();
  auto& width            = frame.getWidth();
  auto& height           = frame.getHeight();
  occupancyMap.resize( width * height, 0 );
  if ( !params_.absoluteD1_ ) fullOccupancyMap.resize( width * height, 0 );
  const int16_t infiniteDepth = ( std::numeric_limits<int16_t>::max )();
  for ( auto& patch : frame.getPatches() ) {
    const size_t v0 = patch.getV0() * patch.getOccupancyResolution();
    const size_t u0 = patch.getU0() * patch.getOccupancyResolution();
    for ( size_t v = 0; v < patch.getSizeV(); ++v ) {
      for ( size_t u = 0; u < patch.getSizeU(); ++u ) {
        const size_t  p       = v * patch.getSizeU() + u;
        const int16_t d       = patch.getDepth( 0 )[p];
        const int16_t eddCode = patch.getDepthEnhancedDeltaD()[p];
        size_t        x, y;
        auto          indx = patch.patch2Canvas( u, v, width, height, x, y );
        assert( x < width && y < height );
        const size_t d0 = imageRef.getValue( 0, x, y );
        const size_t d1 = image.getValue( 0, x, y );
        if ( ( d < infiniteDepth ) && ( occupancyMap[indx] == 1 ) && ( ( d1 - d0 ) > 1 ) ) {
          uint16_t bits       = d1 - d0 - 1;
          uint16_t eddExtract = eddCode & ( ~( ( ~0 ) << bits ) );
          uint16_t symbol     = ( ( ( 1 << bits ) - 1 ) - eddExtract );
          occupancyMap[indx] += symbol;
        }
      }
    }
  }
  if ( !params_.absoluteD1_ ) { fullOccupancyMap = occupancyMap; }
}

void PCCEncoder::remove3DMotionEstimationFiles( std::string path ) {
  removeFile( path + "occupancy.txt" );
  removeFile( path + "patchInfo.txt" );
  removeFile( path + "blockToPatch.txt" );
}

void PCCEncoder::create3DMotionEstimationFiles( PCCContext& context, std::string path ) {
  FILE* occupancyFile    = fopen( ( path + "occupancy.txt" ).c_str(), "wb" );
  FILE* patchInfoFile    = fopen( ( path + "patchInfo.txt" ).c_str(), "wb" );
  FILE* blockToPatchFile = fopen( ( path + "blockToPatch.txt" ).c_str(), "wb" );

  for ( auto& frame : context.getFrames() ) {
    auto&        patches            = frame.getPatches();
    auto&        occupancyMap       = frame.getOccupancyMap();
    auto&        blockToPatch       = frame.getBlockToPatch();
    const size_t blockToPatchWidth  = frame.getWidth() / params_.occupancyResolution_;
    const size_t blockToPatchHeight = frame.getHeight() / params_.occupancyResolution_;
    fwrite( &blockToPatch[0], sizeof( size_t ), blockToPatchHeight * blockToPatchWidth, blockToPatchFile );
    fwrite( &occupancyMap[0], sizeof( uint32_t ), frame.getHeight() * frame.getWidth(), occupancyFile );
    const size_t numPatches = patches.size();
    fwrite( &numPatches, sizeof( size_t ), 1, patchInfoFile );
    for ( const auto& patch : patches ) {
      size_t projectionIndex = patch.getNormalAxis();
      size_t u0              = patch.getU0();
      size_t v0              = patch.getV0();
      size_t sizeU0          = patch.getSizeU0();
      size_t sizeV0          = patch.getSizeV0();
      size_t d1              = patch.getD1();
      size_t u1              = patch.getU1();
      size_t v1              = patch.getV1();
      fwrite( &projectionIndex, sizeof( size_t ), 1, patchInfoFile );
      fwrite( &u0, sizeof( size_t ), 1, patchInfoFile );
      fwrite( &v0, sizeof( size_t ), 1, patchInfoFile );
      fwrite( &sizeU0, sizeof( size_t ), 1, patchInfoFile );
      fwrite( &sizeV0, sizeof( size_t ), 1, patchInfoFile );
      fwrite( &d1, sizeof( size_t ), 1, patchInfoFile );
      fwrite( &u1, sizeof( size_t ), 1, patchInfoFile );
      fwrite( &v1, sizeof( size_t ), 1, patchInfoFile );
    }
  }
  fclose( blockToPatchFile );
  fclose( occupancyFile );
  fclose( patchInfoFile );
}

void PCCEncoder::generateIntraImage( PCCFrameContext& frame, const size_t depthIndex, PCCImageGeometry& image ) {
  auto& width  = frame.getWidth();
  auto& height = frame.getHeight();
  image.resize( width, height );
  image.set( 0 );
  const int16_t infiniteDepth = ( std::numeric_limits<int16_t>::max )();
  size_t        maxDepth      = 0;
  for ( auto& patch : frame.getPatches() ) {
    const size_t v0 = patch.getV0() * patch.getOccupancyResolution();
    const size_t u0 = patch.getU0() * patch.getOccupancyResolution();
    for ( size_t v = 0; v < patch.getSizeV(); ++v ) {
      for ( size_t u = 0; u < patch.getSizeU(); ++u ) {
        const size_t  p = v * patch.getSizeU() + u;
        const int16_t d = patch.getDepth( depthIndex )[p];
        if ( d < infiniteDepth ) {
          size_t x, y;
          patch.patch2Canvas( u, v, width, height, x, y );
          if ( params_.lossyMissedPointsPatch_ && !params_.useMissedPointsSeparateVideo_ ) {
            // image.setValue(0, x, y, uint16_t(d));
            image.setValue( 0, x, y,
                            uint16_t( d << 2 ) );  // left shift the 8-bit depth values to store in 10-bit video frame
          } else {
            image.setValue( 0, x, y, uint16_t( d ) );
          }
          maxDepth = ( std::max )( maxDepth, patch.getSizeD() );
        }
      }
    }
  }

  if ( maxDepth > 255 ) {
    std::cout << "Error: maxDepth(" << maxDepth << ") > 255" << std::endl;
    exit( -1 );
  }

  if ( !frame.getUseMissedPointsSeparateVideo() ) {
    auto&        missedPointsPatch = frame.getMissedPointsPatch();
    const size_t v0                = missedPointsPatch.v0_ * missedPointsPatch.occupancyResolution_;
    const size_t u0                = missedPointsPatch.u0_ * missedPointsPatch.occupancyResolution_;
    if ( missedPointsPatch.size() ) {
      for ( size_t v = 0; v < missedPointsPatch.sizeV_; ++v ) {
        for ( size_t u = 0; u < missedPointsPatch.sizeU_; ++u ) {
          const size_t p = v * missedPointsPatch.sizeU_ + u;
          if ( missedPointsPatch.x_[p] < infiniteDepth ) {
            const size_t x = ( u0 + u );
            const size_t y = ( v0 + v );
            assert( x < width && y < height );
            image.setValue( 0, x, y, uint16_t( missedPointsPatch.x_[p] ) );
            if ( params_.losslessGeo444_ ) {
              image.setValue( 1, x, y, uint16_t( missedPointsPatch.y_[p] ) );
              image.setValue( 2, x, y, uint16_t( missedPointsPatch.z_[p] ) );
            }
          }
        }
      }
    }
  }
}

bool PCCEncoder::predictGeometryFrame( PCCFrameContext&        frame,
                                       const PCCImageGeometry& reference,
                                       PCCImageGeometry&       image ) {
  assert( reference.getWidth() == image.getWidth() );
  assert( reference.getHeight() == image.getHeight() );
  const size_t refWidth            = reference.getWidth();
  const size_t refHeight           = reference.getHeight();
  auto&        occupancyMap        = frame.getFullOccupancyMap();
  size_t       occupancyResolution = params_.occupancyResolution_;

  for ( size_t y = 0; y < refHeight; ++y ) {
    for ( size_t x = 0; x < refWidth; ++x ) {
      const size_t pos1 = y * refWidth + x;
      if ( occupancyMap[pos1] == 1 ) {
        for ( size_t c = 0; c < 1; ++c ) {
          // for (size_t c = 0; c < 3; ++c) {
          const uint16_t value1 = static_cast<uint16_t>( image.getValue( c, x, y ) );
          const uint16_t value0 = static_cast<uint16_t>( reference.getValue( c, x, y ) );
          size_t p = ( ( y / occupancyResolution ) * ( refWidth / occupancyResolution ) ) + ( x / occupancyResolution );
          size_t patchIndex = frame.getBlockToPatch()[p] - 1;
          auto   patch      = frame.getPatches()[patchIndex];

          int_least32_t delta = 0;
          if ( patch.getProjectionMode() == 0 ) {
            delta = (int_least32_t)value1 - (int_least32_t)value0;
            if ( delta < 0 ) { delta = 0; }
            if ( !params_.losslessGeo_ && delta > 9 ) { delta = 9; }
            image.setValue( c, x, y, (uint8_t)delta );
          } else {
            delta = (int_least32_t)value0 - (int_least32_t)value1;
            if ( delta < 0 ) { delta = 0; }
            if ( !params_.losslessGeo_ && delta > 9 ) {  // no clipping for lossless coding
              delta = 9;
            }
            image.setValue( c, x, y, (uint8_t)delta );
          }
        }
      }
    }
  }
  return true;
}

void PCCEncoder::generateMissedPointsPatch( const PCCPointSet3& source,
                                            PCCFrameContext&    frame,
                                            bool                useEnhancedDeltaDepthCode ) {
  const int16_t infiniteDepth     = ( std::numeric_limits<int16_t>::max )();
  auto&         patches           = frame.getPatches();
  auto&         missedPointsPatch = frame.getMissedPointsPatch();
  missedPointsPatch.reset();
  bool         useOneLayerMode  = params_.oneLayerMode_ || params_.singleLayerPixelInterleaving_;
  bool         sixDirectionFlag = params_.absoluteD1_;
  PCCPointSet3 pointsToBeProjected;
  for ( const auto& patch : patches ) {
    for ( size_t v = 0; v < patch.getSizeV(); ++v ) {
      for ( size_t u = 0; u < patch.getSizeU(); ++u ) {
        const size_t p      = v * patch.getSizeU() + u;
        const size_t depth0 = patch.getDepth( 0 )[p];
        if ( depth0 < infiniteDepth ) {
          PCCPoint3D point0;
          if ( !sixDirectionFlag ) {
            point0[patch.getNormalAxis()] = double( depth0 ) + patch.getD1();
          } else {
            if ( patch.getProjectionMode() == 0 ) {
              point0[patch.getNormalAxis()] = double( depth0 + patch.getD1() );
            } else {
              point0[patch.getNormalAxis()] = double( patch.getD1() - depth0 );
            }
          }
          point0[patch.getTangentAxis()]   = double( u ) + patch.getU1();
          point0[patch.getBitangentAxis()] = double( v ) + patch.getV1();
          pointsToBeProjected.addPoint( point0 );
          if ( useEnhancedDeltaDepthCode ) {
            if ( patch.getDepthEnhancedDeltaD()[p] != 0 ) {
              PCCPoint3D point1;
              point1[patch.getTangentAxis()]   = double( u ) + patch.getU1();
              point1[patch.getBitangentAxis()] = double( v ) + patch.getV1();
              for ( uint16_t i = 0; i < 16; i++ ) {  // surfaceThickness is not necessary here?
                if ( patch.getDepthEnhancedDeltaD()[p] & ( 1 << i ) ) {
                  uint16_t nDeltaDCur = ( i + 1 );
                  if ( !sixDirectionFlag ) {
                    if ( patch.getProjectionMode() == 0 ) {
                      point1[patch.getNormalAxis()] = double( depth0 + patch.getD1() + nDeltaDCur );
                    } else {
                      point1[patch.getNormalAxis()] = double( depth0 + patch.getD1() - nDeltaDCur );
                    }
                  } else {
                    if ( patch.getProjectionMode() == 0 ) {
                      point1[patch.getNormalAxis()] = double( depth0 + patch.getD1() + nDeltaDCur );
                    } else {
                      point1[patch.getNormalAxis()] = double( patch.getD1() - depth0 - nDeltaDCur );
                    }
                  }
                  pointsToBeProjected.addPoint( point1 );
                }
              }  // for each i
            }    // if( patch.getDepthEnhancedDeltaD()[p] != 0) )
          } else {
            const size_t depth1 = patch.getDepth( 1 )[p];
            if ( !sixDirectionFlag ) {
              if ( ( ( depth1 > depth0 ) && ( patch.getProjectionMode() == 0 ) ) ||
                   ( ( depth1 < depth0 ) && ( patch.getProjectionMode() == 1 ) ) ) {
                PCCPoint3D point1;
                point1[patch.getNormalAxis()]    = double( depth1 ) + patch.getD1();
                point1[patch.getTangentAxis()]   = double( u ) + patch.getU1();
                point1[patch.getBitangentAxis()] = double( v ) + patch.getV1();
                pointsToBeProjected.addPoint( point1 );
              }
            } else {
              PCCPoint3D point1;
              point1[patch.getTangentAxis()]   = double( u ) + patch.getU1();
              point1[patch.getBitangentAxis()] = double( v ) + patch.getV1();
              if ( patch.getProjectionMode() == 0 ) {
                point1[patch.getNormalAxis()] = double( depth1 ) + patch.getD1();
              } else {
                point1[patch.getNormalAxis()] = double( patch.getD1() ) - double( depth1 );
              }
              pointsToBeProjected.addPoint( point1 );
            }
          } 
        }
      }
    }
  }
  PCCKdTree kdtreeMissedPoints( pointsToBeProjected );
  PCCNNResult result;
  std::vector<size_t> missedPoints;
  missedPoints.resize( 0 );
  for ( size_t i = 0; i < source.getPointCount(); ++i ) {
    kdtreeMissedPoints.search( source[i], 1, result );
    const double dist2 = result.dist( 0 );
    if ( dist2 > 0.0 ) { missedPoints.push_back( i ); }
  }
  size_t numMissedPts = missedPoints.size();
  if ( params_.lossyMissedPointsPatch_ ) {
    // Settings for selecting/pruning points.
    const size_t maxNeighborCount                      = 16;
    const size_t maxDist                               = 10;  // lower the value of maxDist, fewer points will selected
    const double minSumOfInvDist4MissedPointsSelection = params_.minNormSumOfInvDist4MPSelection_ * maxNeighborCount;
    std::vector<size_t> tmpMissedPoints;
    tmpMissedPoints.resize( 0 );
    PCCPointSet3 missedPointsSet;
    missedPointsSet.resize( numMissedPts );
    // create missed points cloud
    for ( size_t i = 0; i < numMissedPts; ++i ) { missedPointsSet[i] = source[missedPoints[i]]; }
    PCCKdTree kdtreeMissedPointsSet( missedPointsSet );
    double    sumOfInverseDist = 0.0;
    for ( size_t i = 0; i < numMissedPts; ++i ) {
      PCCNNResult result;
      kdtreeMissedPointsSet.searchRadius( missedPointsSet[i], maxNeighborCount, maxDist, result );
      sumOfInverseDist = 0.0;
      for ( size_t j = 1; j < result.count(); ++j ) { sumOfInverseDist += 1 / result.dist( j ); }
      if ( sumOfInverseDist >= minSumOfInvDist4MissedPointsSelection ) { tmpMissedPoints.push_back( missedPoints[i] ); }
    }
    numMissedPts = tmpMissedPoints.size();
    missedPoints.resize( numMissedPts );
    missedPoints = tmpMissedPoints;
  }
  missedPointsPatch.numMissedPts_ = numMissedPts;

  missedPointsPatch.occupancyResolution_ = params_.occupancyResolution_;
  if ( params_.losslessGeo444_ ) {
    missedPointsPatch.resize( numMissedPts );
    for ( auto i = 0; i < missedPoints.size(); ++i ) {
      const PCCPoint3D missedPoint = source[missedPoints[i]];
      missedPointsPatch.x_[i]      = static_cast<uint16_t>( missedPoint.x() );
      missedPointsPatch.y_[i]      = static_cast<uint16_t>( missedPoint.y() );
      missedPointsPatch.z_[i]      = static_cast<uint16_t>( missedPoint.z() );
    }
    if ( frame.getUseMissedPointsSeparateVideo() ) { missedPointsPatch.setMPnumber( missedPoints.size() ); }
  } else {
    missedPointsPatch.resize( 3 * numMissedPts );
    if ( frame.getUseMissedPointsSeparateVideo() ) { missedPointsPatch.setMPnumber( missedPoints.size() ); }
    const int16_t infiniteValue = ( std::numeric_limits<int16_t>::max )();
    for ( auto i = 0; i < numMissedPts; ++i ) {
      const PCCPoint3D missedPoint               = source[missedPoints[i]];
      missedPointsPatch.x_[i]                    = static_cast<uint16_t>( missedPoint.x() );
      missedPointsPatch.x_[numMissedPts + i]     = static_cast<uint16_t>( missedPoint.y() );
      missedPointsPatch.x_[2 * numMissedPts + i] = static_cast<uint16_t>( missedPoint.z() );
      missedPointsPatch.y_[i]                    = infiniteValue;
      missedPointsPatch.y_[numMissedPts + i]     = infiniteValue;
      missedPointsPatch.y_[2 * numMissedPts + i] = infiniteValue;
      missedPointsPatch.z_[i]                    = infiniteValue;
      missedPointsPatch.z_[numMissedPts + i]     = infiniteValue;
      missedPointsPatch.z_[2 * numMissedPts + i] = infiniteValue;
    }
  }
}

void PCCEncoder::sortMissedPointsPatch( PCCFrameContext& frame ) {
  auto&        missedPointsPatch    = frame.getMissedPointsPatch();
  const size_t maxNeighborCount     = 5;
  const size_t neighborSearchRadius = 5 * 5;
  size_t       numMissedPts         = missedPointsPatch.numMissedPts_;
  if ( numMissedPts ) {
    vector<size_t> sortIdx;
    sortIdx.reserve( numMissedPts );
    PCCPointSet3 missedPointSet;
    missedPointSet.resize( numMissedPts );
    for ( size_t i = 0; i < numMissedPts; i++ ) {
      missedPointSet[i] = params_.losslessGeo444_
                              ? PCCPoint3D( missedPointsPatch.x_[i], missedPointsPatch.y_[i], missedPointsPatch.z_[i] )
                              : PCCPoint3D( missedPointsPatch.x_[i], missedPointsPatch.x_[i + numMissedPts],
                                            missedPointsPatch.x_[i + numMissedPts * 2] );
    }
    PCCKdTree           kdtreeMissedPointSet( missedPointSet );
    PCCNNResult         result;
    std::vector<size_t> fifo;
    fifo.reserve( numMissedPts );
    std::vector<bool> flags( numMissedPts, true );

    for ( size_t i = 0; i < numMissedPts; i++ ) {
      if ( flags[i] ) {
        flags[i] = false;
        sortIdx.push_back( i );
        fifo.push_back( i );
        while ( !fifo.empty() ) {
          const size_t currentIdx = fifo.back();
          fifo.pop_back();
          kdtreeMissedPointSet.searchRadius( missedPointSet[currentIdx], maxNeighborCount, neighborSearchRadius,
                                             result );
          for ( size_t j = 0; j < result.count(); j++ ) {
            size_t n = result.indices( j );
            if ( flags[n] ) {
              flags[n] = false;
              sortIdx.push_back( n );
              fifo.push_back( n );
            }
          }
        }
      }
    }

    for ( size_t i = 0; i < numMissedPts; ++i ) {
      const PCCPoint3D missedPoint = missedPointSet[sortIdx[i]];
      if ( params_.losslessGeo444_ ) {
        missedPointsPatch.x_[i] = static_cast<uint16_t>( missedPoint.x() );
        missedPointsPatch.y_[i] = static_cast<uint16_t>( missedPoint.y() );
        missedPointsPatch.z_[i] = static_cast<uint16_t>( missedPoint.z() );
      } else {
        missedPointsPatch.x_[i]                    = static_cast<uint16_t>( missedPoint.x() );
        missedPointsPatch.x_[i + numMissedPts]     = static_cast<uint16_t>( missedPoint.y() );
        missedPointsPatch.x_[i + numMissedPts * 2] = static_cast<uint16_t>( missedPoint.z() );
      }
    }
  }
}

void PCCEncoder::generateMissedPointsGeometryVideo( PCCContext& context, PCCGroupOfFrames& reconstructs ) {
  auto&  sps              = context.getSps();
  auto&  videoMPsGeometry = context.getVideoMPsGeometry();
  auto   gofSize          = context.size();
  size_t maxWidth         = 0;
  size_t maxHeight        = 0;
  videoMPsGeometry.resize( gofSize );
  for ( auto& frame : context.getFrames() ) {
    const size_t shift = frame.getIndex();
    frame.setLosslessGeo( sps.getLosslessGeo() );
    frame.setLosslessGeo444( sps.getLosslessGeo444() );
    frame.setLosslessTexture( sps.getLosslessTexture() );
    frame.setMPGeoWidth( context.getMPGeoWidth() );
    frame.setMPGeoHeight( 0 );
    // frame.setEnhancedDeltaDepth( sps.getEnhancedOccupancyMapForDepthFlag());
    generateMPsGeometryImage( context, frame, videoMPsGeometry.getFrame( shift ) );
    cout << "generate Missed Points (Geometry) : frame " << shift
         << ", # of Missed Points Geometry : " << frame.getMissedPointsPatch().size() << endl;
    // for resizing for mpgeometry
    auto& MPGeoFrame = videoMPsGeometry.getFrame( shift );
    maxWidth         = ( std::max )( maxWidth, MPGeoFrame.getWidth() );
    maxHeight        = ( std::max )( maxHeight, MPGeoFrame.getHeight() );
  }

  // resizing for mpgeometry
  assert( maxWidth == 64 );
  assert( maxHeight % 8 == 0 );
  context.getMPGeoWidth()  = maxWidth;
  context.getMPGeoHeight() = maxHeight;
  for ( auto& frame : context.getFrames() ) {
    const size_t shift      = frame.getIndex();
    auto&        MPGeoFrame = videoMPsGeometry.getFrame( shift );
    MPGeoFrame.resize( maxWidth, maxHeight );
  }
  cout << "MissedPoints Geometry [done]" << endl;
}

void PCCEncoder::generateMissedPointsTextureVideo( PCCContext& context, PCCGroupOfFrames& reconstructs ) {
  auto& videoMPsTexture = context.getVideoMPsTexture();
  auto  gofSize         = context.size();
  auto& sps             = context.getSps();
  videoMPsTexture.resize( gofSize );
  size_t maxWidth  = 0;
  size_t maxHeight = 0;
  for ( auto& frame : context.getFrames() ) {
    const size_t shift = frame.getIndex();
    frame.setLosslessTexture( sps.getLosslessTexture() );
    frame.setMPAttWidth( context.getMPAttWidth() );
    frame.setMPAttHeight( 0 );
    generateMPsTextureImage( context, frame, videoMPsTexture.getFrame( shift ), shift, reconstructs[shift] );
    cout << "generate Missed Points (Texture) : frame " << shift
         << ", # of Missed Points Texture : " << frame.getMissedPointsPatch().size() << endl;
    // for resizing for mpgeometry
    auto& MPTexFrame = videoMPsTexture.getFrame( shift );
    maxWidth         = ( std::max )( maxWidth, MPTexFrame.getWidth() );
    maxHeight        = ( std::max )( maxHeight, MPTexFrame.getHeight() );
  }
  // resizing for mpgeometry
  assert( maxWidth == 64 );
  assert( maxHeight % 8 == 0 );
  context.getMPAttWidth()  = maxWidth;
  context.getMPAttHeight() = maxHeight;
  for ( auto& frame : context.getFrames() ) {
    const size_t shift      = frame.getIndex();
    auto&        MPTexFrame = videoMPsTexture.getFrame( shift );
    MPTexFrame.resize( maxWidth, maxHeight );
  }

  cout << "MissedPoints Texture [done]" << endl;
}

void PCCEncoder::generateMPsGeometryImage( PCCContext& context, PCCFrameContext& frame, PCCImageGeometry& image ) {
  bool  losslessGeo444    = frame.getLosslessGeo444();
  bool  losslessGeo       = frame.getLosslessGeo();
  auto& missedPointsPatch = frame.getMissedPointsPatch();
  if ( !losslessGeo444 ) { assert( missedPointsPatch.size() % 3 == 0 ); }

  const size_t numofMPs = losslessGeo444 ? missedPointsPatch.size() : ( missedPointsPatch.size() / 3 );
  missedPointsPatch.setMPnumber( numofMPs );
  size_t width  = frame.getMPGeoWidth();
  size_t height = ( 3 * numofMPs ) / width + 1;
  // height adjust for HM
  size_t heightby8 = height / 8;
  if ( heightby8 * 8 != height ) { height = ( heightby8 + 1 ) * 8; }
  frame.setMPGeoHeight( height );
  image.resize( width, height );
  image.set( 0 );
  uint16_t lastZ{0};

  for ( size_t i = 0; i < numofMPs; i++ ) {
    if ( frame.getLosslessGeo444() ) {
      image.setValue( 0, i % width, i / width, missedPointsPatch.x_[i] );
      image.setValue( 0, i % width, i / width, missedPointsPatch.y_[i] );
      image.setValue( 0, i % width, i / width, missedPointsPatch.z_[i] );
    } else {
      image.setValue( 0, i % width, i / width, missedPointsPatch.x_[i] );
      image.setValue( 0, ( numofMPs + i ) % width, ( numofMPs + i ) / width, missedPointsPatch.x_[numofMPs + i] );
      image.setValue( 0, ( 2 * numofMPs + i ) % width, ( 2 * numofMPs + i ) / width,
                      missedPointsPatch.x_[2 * numofMPs + i] );
    }
    lastZ = missedPointsPatch.x_[2 * numofMPs + i];
  }

  // dilate with the last z value
  if ( !losslessGeo ) {  // lossy missed points patch and 4:2:0 frame format
    for ( size_t i = 3 * numofMPs; i < width * height; ++i ) {
      image.setValue( 0, i % width, i / width, static_cast<uint16_t>( lastZ ) );
    }
  }
}

void PCCEncoder::generateMPsTextureImage( PCCContext&         context,
                                          PCCFrameContext&    frame,
                                          PCCImageTexture&    image,
                                          size_t              shift,
                                          const PCCPointSet3& reconstruct ) {
  bool         losslessAtt       = frame.getLosslessTexture();
  auto&        missedPointsPatch = frame.getMissedPointsPatch();
  const size_t sizeofMPcolor     = missedPointsPatch.sizeofcolor();
  size_t       width             = frame.getMPAttWidth();
  size_t       height            = sizeofMPcolor / width + 1;
  size_t       heightby8         = height / 8;
  if ( heightby8 * 8 != height ) { height = ( heightby8 + 1 ) * 8; }
  image.resize( width, height );
  image.set( 0 );
  double avgR{0.0};
  double avgG{0.0};
  double avgB{0.0};

  for ( size_t k = 0; k < sizeofMPcolor; k++ ) {
    size_t xx = k % width;
    size_t yy = k / width;
    assert( yy < height );
    image.setValue( 0, xx, yy, missedPointsPatch.r_[k] );
    image.setValue( 1, xx, yy, missedPointsPatch.g_[k] );
    image.setValue( 2, xx, yy, missedPointsPatch.b_[k] );
    avgR = avgR + double( missedPointsPatch.r_[k] ) / sizeofMPcolor;
    avgG = avgG + double( missedPointsPatch.g_[k] ) / sizeofMPcolor;
    avgB = avgB + double( missedPointsPatch.b_[k] ) / sizeofMPcolor;
  }
  if ( !losslessAtt ) {
    for ( size_t k = sizeofMPcolor; k < width * height; ++k ) {
      size_t xx = k % width;
      size_t yy = k / width;
      assert( yy < height );
      image.setValue( 0, xx, yy, static_cast<uint8_t>( avgR ) );
      image.setValue( 1, xx, yy, static_cast<uint8_t>( avgG ) );
      image.setValue( 2, xx, yy, static_cast<uint8_t>( avgB ) );
    }
  }
}

bool PCCEncoder::generateGeometryVideo( const PCCGroupOfFrames& sources, PCCContext& context ) {
  bool                         res = true;
  PCCPatchSegmenter3Parameters segmenterParams;
  segmenterParams.nnNormalEstimation                   = params_.nnNormalEstimation_;
  segmenterParams.maxNNCountRefineSegmentation         = params_.maxNNCountRefineSegmentation_;
  segmenterParams.iterationCountRefineSegmentation     = params_.iterationCountRefineSegmentation_;
  segmenterParams.occupancyResolution                  = params_.occupancyResolution_;
  segmenterParams.minPointCountPerCCPatchSegmentation  = params_.minPointCountPerCCPatchSegmentation_;
  segmenterParams.maxNNCountPatchSegmentation          = params_.maxNNCountPatchSegmentation_;
  segmenterParams.surfaceThickness                     = params_.surfaceThickness_;
  segmenterParams.minLevel                             = params_.minLevel_;
  segmenterParams.maxAllowedDepth                      = params_.maxAllowedDepth_;
  segmenterParams.maxAllowedDist2MissedPointsDetection = params_.maxAllowedDist2MissedPointsDetection_;
  segmenterParams.maxAllowedDist2MissedPointsSelection = params_.maxAllowedDist2MissedPointsSelection_;
  segmenterParams.lambdaRefineSegmentation             = params_.lambdaRefineSegmentation_;
  segmenterParams.projectionMode                       = params_.projectionMode_;
  segmenterParams.useEnhancedDeltaDepthCode            = params_.losslessGeo_ ? params_.enhancedDeltaDepthCode_ : false;
  // segmenterParams.sixDirectionMode                     = params_.sixDirectionMode_;
  segmenterParams.absoluteD1        = params_.absoluteD1_;
  segmenterParams.useOneLayermode   = params_.oneLayerMode_ || params_.singleLayerPixelInterleaving_;
  segmenterParams.surfaceSeparation = params_.surfaceSeparation_;

  auto& videoGeometry = context.getVideoGeometry();
  auto& frames        = context.getFrames();

  if ( params_.losslessGeo_ || params_.lossyMissedPointsPatch_ ) {
    params_.useAdditionalPointsPatch_ = true;
    context.getSps().setPcmPatchEnabledFlag( true );
  }

  float sumDistanceSrcRec = 0;
  for ( size_t i = 0; i < frames.size(); i++ ) {
    size_t preIndex       = i > 0 ? ( i - 1 ) : 0;
    float  distanceSrcRec = 0;
    if ( !generateGeometryVideo( sources[i], frames[i], segmenterParams, videoGeometry, frames[preIndex], i,
                                 distanceSrcRec ) ) {
      res = false;
      break;
    }
    sumDistanceSrcRec += distanceSrcRec;
  }
  if ( params_.oneLayerMode_ ) {
    const float distanceSrcRec = sumDistanceSrcRec / (float)frames.size();
    if ( distanceSrcRec >= 250.f ) {
      params_.oneLayerMode_ = false;
      if ( params_.singleLayerPixelInterleaving_ ) { params_.singleLayerPixelInterleaving_ = false; }
    }
  }
  return res;
}

void PCCEncoder::reconstuctionOptimization( PCCContext& context, const GeneratePointCloudParameters params ) {
  auto& frames          = context.getFrames();
  auto& videoGeometry   = context.getVideoGeometry();
  auto& videoGeometryD1 = context.getVideoGeometryD1();
  for ( size_t i = 0; i < frames.size(); i++ ) {
    reconstuctionOptimization( frames[i], videoGeometry, videoGeometryD1, params );
  }
}

void PCCEncoder::reconstuctionOptimization( PCCFrameContext&                   frame,
                                            const PCCVideoGeometry&            video,
                                            const PCCVideoGeometry&            videoD1,
                                            const GeneratePointCloudParameters params ) {
  auto&                 patches         = frame.getPatches();
  auto&                 blockToPatch    = frame.getBlockToPatch();
  auto&                 occupancyMapOrg = frame.getOccupancyMap();
  std::vector<uint32_t> occupancyMap;
  occupancyMap.resize( occupancyMapOrg.size(), 0 );
  for ( size_t i = 0; i < occupancyMapOrg.size(); i++ ) { occupancyMap[i] = occupancyMapOrg[i]; }
  const size_t width              = frame.getWidth();
  const size_t height             = frame.getHeight();
  const size_t blockToPatchWidth  = width / params_.occupancyResolution_;
  const size_t blockToPatchHeight = height / params_.occupancyResolution_;
  const size_t blockSize0         = params_.occupancyResolution_ / params_.occupancyPrecision_;
  for ( size_t v0 = 0; v0 < blockToPatchHeight; ++v0 ) {
    for ( size_t u0 = 0; u0 < blockToPatchWidth; ++u0 ) {
      for ( size_t v1 = 0; v1 < blockSize0; ++v1 ) {
        const size_t v2 = v0 * params_.occupancyResolution_ + v1 * params_.occupancyPrecision_;
        for ( size_t u1 = 0; u1 < blockSize0; ++u1 ) {
          const size_t u2     = u0 * params_.occupancyResolution_ + u1 * params_.occupancyPrecision_;
          bool         isFull = false;
          for ( size_t v3 = 0; v3 < params_.occupancyPrecision_ && !isFull; ++v3 ) {
            for ( size_t u3 = 0; u3 < params_.occupancyPrecision_ && !isFull; ++u3 ) {
              isFull |= occupancyMap[( v2 + v3 ) * width + u2 + u3] == 1;
            }
          }
          for ( size_t v3 = 0; v3 < params_.occupancyPrecision_; ++v3 ) {
            for ( size_t u3 = 0; u3 < params_.occupancyPrecision_; ++u3 ) {
              occupancyMap[( v2 + v3 ) * width + u2 + u3] = isFull;
            }
          }
        }
      }
    }
  }
  size_t       shift;
  const size_t layerCount = 2;
  if ( !params.absoluteD1_ ) {
    shift = frame.getIndex();
    if ( video.getFrameCount() < ( shift + 1 ) ) { return; }
  } else {
    shift = frame.getIndex() * ( params.oneLayerMode_ ? 1 : 2 );
    if ( video.getFrameCount() < ( shift + ( params.oneLayerMode_ ? 1 : 2 ) ) ) { return; }
  }
  const size_t               patchCount           = patches.size();
  auto&                      interpolateMap       = frame.getInterpolate();
  auto&                      fillingMap           = frame.getFilling();
  auto&                      minD1Map             = frame.getMinD1();
  auto&                      neighborMap          = frame.getNeighbor();
  const auto&                frame0               = video.getFrame( shift );
  const size_t               imageWidth           = video.getWidth();
  const size_t               imageHeight          = video.getHeight();
  std::vector<PCCPointSet3>& srcPointCloudByBlock = frame.getSrcPointCloudByBlock();
  for ( size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex ) {
    const size_t patchIndexPlusOne = patchIndex + 1;
    auto&        patch             = patches[patchIndex];
    const double lodScale          = params.ignoreLod_ ? 1.0 : double( 1u << patch.getLod() );
    for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
      for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
        const size_t blockIndex = patch.patchBlock2CanvasBlock( u0, v0, blockToPatchWidth, blockToPatchHeight );
        if ( blockToPatch[blockIndex] == patchIndexPlusOne ) {
          auto&        srcPointCloudPatch = frame.getSrcPointCloudByPatch( patch.getIndex() );
          PCCPointSet3 blockSrcPointCloud;
          const size_t xMin = u0 * patch.getOccupancyResolution() + patch.getU1();
          const size_t yMin = v0 * patch.getOccupancyResolution() + patch.getV1();
          for ( size_t i = 0; i < srcPointCloudPatch.getPointCount(); i++ ) {
            if ( xMin <= srcPointCloudPatch[i][patch.getTangentAxis()] &&
                 srcPointCloudPatch[i][patch.getTangentAxis()] < xMin + patch.getOccupancyResolution() &&
                 yMin <= srcPointCloudPatch[i][patch.getBitangentAxis()] &&
                 srcPointCloudPatch[i][patch.getBitangentAxis()] < yMin + patch.getOccupancyResolution() ) {
              blockSrcPointCloud.addPoint( srcPointCloudPatch[i] );
            }
          }
          std::vector<PCCPointSet3> reconstruct;
          std::vector<float>        distance;
          reconstruct.resize( 10 );
          distance.resize( 10 );
          size_t optimizationIndex = 0, optimizationIndexMin = 0;
          for ( size_t minD1 = 0; minD1 < 2; minD1++ ) {
            for ( size_t interpolate = 0; interpolate < 2; interpolate++ ) {
              const size_t maxNeighborLocal = interpolate == 0 ? 2 : 3;
              for ( size_t neighbor = 1; neighbor < maxNeighborLocal; neighbor++ ) {
                const size_t maxFillingLocal =
                    params.oneLayerMode_ && ( ( interpolate == 0 && minD1 == 0 ) || ( minD1 == 1 && interpolate == 0 ) )
                        ? 1
                        : 2;
                for ( size_t filling = 0; filling < maxFillingLocal; filling++ ) {
                  for ( size_t v1 = 0; v1 < patch.getOccupancyResolution(); ++v1 ) {
                    const size_t v = v0 * patch.getOccupancyResolution() + v1;
                    for ( size_t u1 = 0; u1 < patch.getOccupancyResolution(); ++u1 ) {
                      const size_t u = u0 * patch.getOccupancyResolution() + u1;
                      size_t       x, y;
                      const bool   occupancy =
                          occupancyMap[patch.patch2Canvas( u, v, imageWidth, imageHeight, x, y )] != 0;
                      if ( !occupancy ) { continue; }
                      auto createdPoints = generatePoints( params, frame, video, videoD1, shift, patchIndex, u, v, x, y,
                                                           interpolate, filling, minD1, neighbor, lodScale );
                      if ( createdPoints.size() > 0 ) {
                        for ( size_t i = 0; i < createdPoints.size(); i++ ) {
                          reconstruct[optimizationIndex].addPoint( createdPoints[i] );
                        }
                      }
                    }
                  }
                  float distancePSrcRec, distancePRecSrc;
                  blockSrcPointCloud.distanceGeo( reconstruct[optimizationIndex], distancePSrcRec, distancePRecSrc );
                  distance[optimizationIndex] = ( std::max )( distancePSrcRec, distancePRecSrc );
                  if ( optimizationIndex == 0 || distance[optimizationIndexMin] > distance[optimizationIndex] ) {
                    optimizationIndexMin       = optimizationIndex;
                    interpolateMap[blockIndex] = (bool)interpolate;
                    fillingMap[blockIndex]     = (bool)filling;
                    minD1Map[blockIndex]       = minD1;
                    neighborMap[blockIndex]    = neighbor;
                  }
                  optimizationIndex++;
                }
              }
            }
          }
        }  // if block is used
      }
    }
  }  // patch
}

bool PCCEncoder::resizeGeometryVideo( PCCContext& context ) {
  size_t maxWidth = 0, maxHeight = 0;
  for ( auto& frame : context.getFrames() ) {
    maxWidth  = ( std::max )( maxWidth, frame.getWidth() );
    maxHeight = ( std::max )( maxHeight, frame.getHeight() );
  }
  for ( auto& frame : context.getFrames() ) {
    frame.getWidth()  = maxWidth;
    frame.getHeight() = maxHeight;
    frame.getOccupancyMap().resize( ( maxWidth / params_.occupancyResolution_ ) *
                                    ( maxHeight / params_.occupancyResolution_ ) );
  }
  return true;
}

bool PCCEncoder::dilateGeometryVideo( PCCContext& context ) {
  auto& videoGeometry   = context.getVideoGeometry();
  auto& videoGeometryD1 = context.getVideoGeometryD1();
  for ( auto& frame : context.getFrames() ) {
    generateOccupancyMap( frame );
    const size_t shift = videoGeometry.getFrameCount();
    if ( !params_.absoluteD1_ ) {
      videoGeometry.resize( shift + 1 );
      videoGeometryD1.resize( shift + 1 );
      auto& frame1 = videoGeometry.getFrame( shift );
      generateIntraImage( frame, 0, frame1 );
      auto& frame2 = videoGeometryD1.getFrame( shift );
      if ( params_.enhancedDeltaDepthCode_ ) {
        generateIntraEnhancedDeltaDepthImage( frame, frame1, frame2 );
      } else {
        generateIntraImage( frame, 1, frame2 );
      }
      dilate( frame, videoGeometry.getFrame( shift ) );
    } else {
      const size_t nbFrames = params_.oneLayerMode_ ? 1 : 2;
      videoGeometry.resize( shift + nbFrames );
      if ( params_.enhancedDeltaDepthCode_ ) {
        auto& frame1 = videoGeometry.getFrame( shift );
        generateIntraImage( frame, 0, frame1 );
        auto& frame2 = videoGeometry.getFrame( shift + 1 );
        // if (params_.improveEDD_) {
        generateIntraImage( frame, 1, frame2 );
        // }else{
        //   generateIntraEnhancedDeltaDepthImage(frame, frame1, frame2);
        // }
        dilate( frame, videoGeometry.getFrame( shift ) );
        dilate( frame, videoGeometry.getFrame( shift + 1 ) );
        // if (params_.improveEDD_)
        modifyOccupancyMap( frame, frame1, frame2 );
      } else {
        if ( params_.oneLayerMode_ && params_.singleLayerPixelInterleaving_ ) {
          auto& frame1 = videoGeometry.getFrame( shift );
          generateIntraImage( frame, 0, frame1 );
          dilate( frame, frame1 );
          PCCImageGeometry frame2;
          generateIntraImage( frame, 1, frame2 );
          dilate( frame, frame2 );
          for ( size_t x = 0; x < frame1.getWidth(); x++ ) {
            for ( size_t y = 0; y < frame1.getHeight(); y++ ) {
              if ( ( x + y ) % 2 == 1 ) { frame1.setValue( 0, x, y, frame2.getValue( 0, x, y ) ); }
            }
          }
        } else {
          for ( size_t f = 0; f < nbFrames; ++f ) {
            auto& frame1 = videoGeometry.getFrame( shift + f );
            generateIntraImage( frame, f, frame1 );
            dilate( frame, videoGeometry.getFrame( shift + f ) );
          }
        }
      }
    }
  }
  return true;
}

template <typename T>
void PCCEncoder::dilate( PCCFrameContext& frame, PCCImage<T, 3>& image, const PCCImage<T, 3>* reference ) {
  auto          occupancyMapTemp         = frame.getOccupancyMap();
  const size_t  pixelBlockCount          = params_.occupancyResolution_ * params_.occupancyResolution_;
  const size_t  occupancyMapSizeU        = image.getWidth() / params_.occupancyResolution_;
  const size_t  occupancyMapSizeV        = image.getHeight() / params_.occupancyResolution_;
  const int64_t neighbors[4][2]          = {{0, -1}, {-1, 0}, {1, 0}, {0, 1}};
  const size_t  MAX_OCCUPANCY_RESOLUTION = 64;
  assert( params_.occupancyResolution_ <= MAX_OCCUPANCY_RESOLUTION );
  size_t              count[MAX_OCCUPANCY_RESOLUTION][MAX_OCCUPANCY_RESOLUTION];
  PCCVector3<int32_t> values[MAX_OCCUPANCY_RESOLUTION][MAX_OCCUPANCY_RESOLUTION];

  for ( size_t v1 = 0; v1 < occupancyMapSizeV; ++v1 ) {
    const int64_t v0 = v1 * params_.occupancyResolution_;
    for ( size_t u1 = 0; u1 < occupancyMapSizeU; ++u1 ) {
      const int64_t u0                = u1 * params_.occupancyResolution_;
      size_t        nonZeroPixelCount = 0;
      for ( size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2 ) {
        for ( size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2 ) {
          const int64_t x0 = u0 + u2;
          const int64_t y0 = v0 + v2;
          assert( x0 < int64_t( image.getWidth() ) && y0 < int64_t( image.getHeight() ) );
          const size_t location0 = y0 * image.getWidth() + x0;
          if ( params_.enhancedDeltaDepthCode_ ) {
            nonZeroPixelCount += ( occupancyMapTemp[location0] > 0 );
          } else {
            nonZeroPixelCount += ( occupancyMapTemp[location0] == 1 );
          }
        }
      }
      if ( !nonZeroPixelCount ) {
        if ( reference ) {
          for ( size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2 ) {
            for ( size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2 ) {
              const size_t x0 = u0 + u2;
              const size_t y0 = v0 + v2;
              image.setValue( 0, x0, y0, reference->getValue( 0, x0, y0 ) );
              image.setValue( 1, x0, y0, reference->getValue( 1, x0, y0 ) );
              image.setValue( 2, x0, y0, reference->getValue( 2, x0, y0 ) );
            }
          }
        } else if ( u1 > 0 ) {
          for ( size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2 ) {
            for ( size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2 ) {
              const size_t x0 = u0 + u2;
              const size_t y0 = v0 + v2;
              assert( x0 > 0 );
              const size_t x1 = x0 - 1;
              image.setValue( 0, x0, y0, image.getValue( 0, x1, y0 ) );
              image.setValue( 1, x0, y0, image.getValue( 1, x1, y0 ) );
              image.setValue( 2, x0, y0, image.getValue( 2, x1, y0 ) );
            }
          }
        } else if ( v1 > 0 ) {
          for ( size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2 ) {
            for ( size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2 ) {
              const size_t x0 = u0 + u2;
              const size_t y0 = v0 + v2;
              assert( y0 > 0 );
              const size_t y1 = y0 - 1;
              image.setValue( 0, x0, y0, image.getValue( 0, x0, y1 ) );
              image.setValue( 1, x0, y0, image.getValue( 1, x0, y1 ) );
              image.setValue( 2, x0, y0, image.getValue( 2, x0, y1 ) );
            }
          }
        }
        continue;
      }
      for ( size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2 ) {
        for ( size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2 ) {
          values[v2][u2] = 0;
          count[v2][u2]  = 0UL;
        }
      }
      uint32_t iteration = 1;
      while ( nonZeroPixelCount < pixelBlockCount ) {
        for ( size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2 ) {
          for ( size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2 ) {
            const int64_t x0 = u0 + u2;
            const int64_t y0 = v0 + v2;
            assert( x0 < int64_t( image.getWidth() ) && y0 < int64_t( image.getHeight() ) );
            const size_t location0 = y0 * image.getWidth() + x0;
            if ( occupancyMapTemp[location0] == iteration ) {
              for ( size_t n = 0; n < 4; ++n ) {
                const int64_t x1        = x0 + neighbors[n][0];
                const int64_t y1        = y0 + neighbors[n][1];
                const size_t  location1 = y1 * image.getWidth() + x1;
                if ( x1 >= u0 && x1 < int64_t( u0 + params_.occupancyResolution_ ) && y1 >= v0 &&
                     y1 < int64_t( v0 + params_.occupancyResolution_ ) && occupancyMapTemp[location1] == 0 ) {
                  const int64_t u3 = u2 + neighbors[n][0];
                  const int64_t v3 = v2 + neighbors[n][1];
                  assert( u3 >= 0 && u3 < int64_t( params_.occupancyResolution_ ) );
                  assert( v3 >= 0 && v3 < int64_t( params_.occupancyResolution_ ) );
                  for ( size_t k = 0; k < 3; ++k ) { values[v3][u3][k] += image.getValue( k, x0, y0 ); }
                  ++count[v3][u3];
                }
              }
            }
          }
        }
        for ( size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2 ) {
          for ( size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2 ) {
            if ( count[v2][u2] ) {
              ++nonZeroPixelCount;
              const size_t x0             = u0 + u2;
              const size_t y0             = v0 + v2;
              const size_t location0      = y0 * image.getWidth() + x0;
              const size_t c              = count[v2][u2];
              const size_t c2             = c / 2;
              occupancyMapTemp[location0] = iteration + 1;
              for ( size_t k = 0; k < 3; ++k ) { image.setValue( k, x0, y0, T( ( values[v2][u2][k] + c2 ) / c ) ); }
              values[v2][u2] = 0;
              count[v2][u2]  = 0UL;
            }
          }
        }
        ++iteration;
      }
    }
  }
}

template <typename T>
void PCCEncoder::dilateSparseLinearModel( PCCFrameContext& framecontext,
                                          PCCImage<T, 3>&  image,
                                          int              layerIdx,
                                          PCCVideoType     videoType ) {
  const size_t   maxChannel        = ( videoType == Texture ) ? 3 : 1;
  const int32_t  width             = image.getWidth();
  const int32_t  height            = image.getHeight();
  const int32_t  maxIterationCount = 1024;
  const double   maxError          = 0.00001;
  const auto     occupancyMap      = framecontext.getOccupancyMap();
  const int32_t  mask              = ( 1 << 16 ) - 1;
  PaddingContext context;
  cout << "start padding in dilateSparseLinearModel - layerIdx:" << layerIdx << ", videoType:" << videoType << endl;

  const int32_t pixelCount = width * height;
  context._invMapping.resize( pixelCount );
  context._mapping.resize( pixelCount );
  int32_t emptyPixelCount = 0;
  for ( int32_t yPix = 0; yPix < height; ++yPix ) {
    for ( int32_t xPix = 0; xPix < width; ++xPix ) {
      if ( occupancyMap[( yPix * width + xPix )] == 0 ) {
        const int32_t index                   = emptyPixelCount++;
        context._mapping[yPix * width + xPix] = index;
        context._invMapping[index]            = ( yPix << 16 ) + xPix;
      }
    }
  }
  if ( emptyPixelCount == 0 ) {
    cout << "emptyPixelCount == 0" << endl;
    return;
  } else if ( emptyPixelCount == pixelCount ) {
    cout << "emptyPixelCount == pixelCount" << endl;
    for ( int32_t yPix = 0; yPix < height; ++yPix ) {
      for ( int32_t xPix = 0; xPix < width; ++xPix ) {
        for ( int ch = 0; ch < maxChannel; ch++ ) image.setValue( ch, xPix, yPix, 128 );
      }
    }
    return;
  }

  context._invMapping.resize( emptyPixelCount );
  context._A.resize( emptyPixelCount );
  for ( int32_t ch = 0; ch < maxChannel; ++ch ) { context._b[ch].resize( emptyPixelCount ); }
  for ( int32_t index0 = 0; index0 < emptyPixelCount; ++index0 ) {
    const int32_t pixelLocation = context._invMapping[index0];
    const int32_t yPix          = pixelLocation >> 16;
    const int32_t xPix          = pixelLocation & mask;
    auto&         row           = context._A.getRow( index0 );
    row._coefficientCount       = 0;

    const int32_t siftY[4] = {-1, 1, 0, 0};
    const int32_t siftX[4] = {0, 0, -1, 1};
    for ( int32_t ch = 0; ch < maxChannel; ++ch ) { context._b[ch][index0] = 0.0; }
    int32_t neighbourCount = 0;
    for ( int32_t k = 0; k < 4; ++k ) {
      const int32_t y1 = yPix + siftY[k];
      const int32_t x1 = xPix + siftX[k];
      if ( y1 >= 0 && y1 < height && x1 >= 0 && x1 < width ) {
        ++neighbourCount;
        if ( occupancyMap[(y1)*width + ( x1 )] == 0 ) {
          const int32_t localPixelLocation = y1 * width + x1;
          assert( localPixelLocation < width * height );
          const int32_t index1 = context._mapping[localPixelLocation];
          assert( index1 < emptyPixelCount );
          row.addCofficient( index1, -1.0 );
        } else {
          for ( int32_t ch = 0; ch < maxChannel; ++ch ) { context._b[ch][index0] += image.getValue( ch, x1, y1 ); }
        }
      }
    }
    row.addCofficient( index0, neighbourCount );
  }
  const auto& A = context._A;
  auto&       x = context._x;
  auto&       p = context._p;
  auto&       r = context._r;
  auto&       q = context._q;
  x.resize( emptyPixelCount );
  p.resize( emptyPixelCount );
  r.resize( emptyPixelCount );
  q.resize( emptyPixelCount );
  for ( int32_t ch = 0; ch < maxChannel; ++ch ) {
    const auto& b = context._b[ch];

    for ( int32_t i = 0; i < emptyPixelCount; ++i ) { x[i] = 128.0; }
    for ( int32_t i = 0; i < emptyPixelCount; ++i ) {
      const auto& row = A.getRow( i );
      double      ri  = b[i];
      for ( int32_t j = 0; j < row._coefficientCount; ++j ) {
        const auto& coeff = row._coefficients[j];
        ri -= coeff._value * x[coeff._index];
      }
      p[i] = r[i] = ri;
    }

    double  error = 0.0;
    int32_t it    = 0;
    for ( ; it < maxIterationCount; ++it ) {
      double rtr = 0.0;
      for ( int32_t i = 0; i < emptyPixelCount; ++i ) { rtr += r[i] * r[i]; }
      error = sqrt( rtr ) / emptyPixelCount;
      if ( error < maxError ) { break; }
      double ptAp = 0.0;
      for ( int32_t i = 0; i < emptyPixelCount; ++i ) {
        const auto& row = A.getRow( i );
        double      sq  = 0.0;
        for ( int32_t j = 0; j < row._coefficientCount; ++j ) {
          const auto& coeff = row._coefficients[j];
          sq += coeff._value * p[coeff._index];
        }
        q[i] = sq;
        ptAp += p[i] * sq;
      }
      const double alpha = rtr / ptAp;
      for ( int32_t i = 0; i < emptyPixelCount; ++i ) {
        x[i] += alpha * p[i];
        r[i] -= alpha * q[i];
      }
      double r1tr1 = 0.0;
      for ( int32_t i = 0; i < emptyPixelCount; ++i ) { r1tr1 += r[i] * r[i]; }
      const double betha = r1tr1 / rtr;
      for ( int32_t i = 0; i < emptyPixelCount; ++i ) { p[i] = r[i] + betha * p[i]; }
    }
    std::cout << it << " -> error = " << error << std::endl;

    for ( int32_t index0 = 0; index0 < emptyPixelCount; ++index0 ) {
      const int32_t pixelLocation = context._invMapping[index0];
      const int32_t yPixOut       = pixelLocation >> 16;
      const int32_t xPixOut       = pixelLocation & mask;
      const double  v             = std::round( x[index0] );
      if ( v >= 255.0 ) {
        image.setValue( ch, xPixOut, yPixOut, 255 );
      } else if ( v <= 0.0 ) {
        image.setValue( ch, xPixOut, yPixOut, 0 );
      } else {
        image.setValue( ch, xPixOut, yPixOut, uint8_t( v ) );
      }
    }  // index0
  }

  cout << "dilateSparseLinear: finished" << endl;
  return;
}
/* pull push filling algorithm */
template <typename T>
int PCCEncoder::mean4w( T             p1,
                        unsigned char w1,
                        T             p2,
                        unsigned char w2,
                        T             p3,
                        unsigned char w3,
                        T             p4,
                        unsigned char w4 ) {
  int result = ( p1 * int( w1 ) + p2 * int( w2 ) + p3 * int( w3 ) + p4 * int( w4 ) ) /
               ( int( w1 ) + int( w2 ) + int( w3 ) + int( w4 ) );
  return result;
}

// Generates a weighted mipmap
template <typename T>
void PCCEncoder::pullPushMip( PCCImage<T, 3>&        image,
                              PCCImage<T, 3>&        mip,
                              std::vector<uint32_t>& occupancyMap,
                              std::vector<uint32_t>& mipOccupancyMap ) {
  // allocate the mipmap with half the resolution
  mip.resize( ( ( image.getWidth() + 1 ) / 2 ), ( ( image.getHeight() + 1 ) / 2 ) );
  mipOccupancyMap.resize( ( ( image.getWidth() + 1 ) / 2 ) * ( ( image.getHeight() + 1 ) / 2 ), 0 );
  unsigned char w1, w2, w3, w4;
  unsigned char val1, val2, val3, val4;
  int           stride    = image.getWidth();
  int           height    = image.getHeight();
  int           newStride = ( ( image.getWidth() + 1 ) / 2 );
  int           x, y;
  for ( y = 0; y < mip.getHeight(); ++y ) {
    for ( x = 0; x < mip.getWidth(); ++x ) {
      if ( occupancyMap[x * 2 + stride * ( y * 2 )] == 0 )
        w1 = 0;
      else
        w1 = 255;
      if ( ( x * 2 + 1 >= stride ) || ( occupancyMap[x * 2 + 1 + stride * ( y * 2 )] == 0 ) )
        w2 = 0;
      else
        w2 = 255;
      if ( ( y * 2 + 1 >= height ) || ( occupancyMap[x * 2 + stride * ( y * 2 + 1 )] == 0 ) )
        w3 = 0;
      else
        w3 = 255;
      if ( ( x * 2 + 1 >= stride ) || ( y * 2 + 1 >= height ) ||
           ( occupancyMap[x * 2 + 1 + stride * ( y * 2 + 1 )] == 0 ) )
        w4 = 0;
      else
        w4 = 255;
      if ( w1 + w2 + w3 + w4 > 0 ) {
        for ( int cc = 0; cc < 3; cc++ ) {
          val1 = image.getValue( cc, x * 2, y * 2 );
          if ( x * 2 + 1 >= stride )
            val2 = 0;
          else
            val2 = image.getValue( cc, x * 2 + 1, y * 2 );
          if ( y * 2 + 1 >= height )
            val3 = 0;
          else
            val3 = image.getValue( cc, x * 2, y * 2 + 1 );
          if ( ( x * 2 + 1 >= stride ) || ( y * 2 + 1 >= height ) )
            val4 = 0;
          else
            val4 = image.getValue( cc, x * 2 + 1, y * 2 + 1 );
          T newVal = mean4w( val1, w1, val2, w2, val3, w3, val4, w4 );
          mip.setValue( cc, x, y, newVal );
        }
        mipOccupancyMap[x + newStride * y] = 1;
      }
    }
  }
}

// interpolate using mipmap
template <typename T>
void PCCEncoder::pullPushFill( PCCImage<T, 3>& image, PCCImage<T, 3>& mip, std::vector<uint32_t>& occupancyMap ) {
  assert( ( ( image.getWidth() + 1 ) / 2 ) == mip.getWidth() );
  assert( ( ( image.getHeight() + 1 ) / 2 ) == mip.getHeight() );
  int           stride = image.getWidth();
  int           x, y, xUp, yUp;
  unsigned char w1, w2, w3, w4;
  for ( yUp = 0; yUp < image.getHeight(); ++yUp ) {
    y = yUp / 2;
    for ( xUp = 0; xUp < image.getWidth(); ++xUp ) {
      x = xUp / 2;
      if ( occupancyMap[xUp + stride * yUp] == 0 ) {
        if ( ( xUp % 2 == 0 ) && ( yUp % 2 == 0 ) ) {
          w1 = 144;
          w2 = ( x > 0 ? (unsigned char)48 : 0 );
          w3 = ( y > 0 ? (unsigned char)48 : 0 );
          w4 = ( ( ( x > 0 ) && ( y > 0 ) ) ? (unsigned char)16 : 0 );
          for ( int cc = 0; cc < 3; cc++ ) {
            T val       = mip.getValue( cc, x, y );
            T valLeft   = ( x > 0 ? mip.getValue( cc, x - 1, y ) : 0 );
            T valUp     = ( y > 0 ? mip.getValue( cc, x, y - 1 ) : 0 );
            T valUpLeft = ( ( x > 0 && y > 0 ) ? mip.getValue( cc, x - 1, y - 1 ) : 0 );
            T newVal    = mean4w( val, w1, valLeft, w2, valUp, w3, valUpLeft, w4 );
            image.setValue( cc, xUp, yUp, newVal );
          }
        } else if ( ( xUp % 2 == 1 ) && ( yUp % 2 == 0 ) ) {
          w1 = 144;
          w2 = ( x < mip.getWidth() - 1 ? (unsigned char)48 : 0 );
          w3 = ( y > 0 ? (unsigned char)48 : 0 );
          w4 = ( ( ( x < mip.getWidth() - 1 ) && ( y > 0 ) ) ? (unsigned char)16 : 0 );
          for ( int cc = 0; cc < 3; cc++ ) {
            T val        = mip.getValue( cc, x, y );
            T valRight   = ( x < mip.getWidth() - 1 ? mip.getValue( cc, x + 1, y ) : 0 );
            T valUp      = ( y > 0 ? mip.getValue( cc, x, y - 1 ) : 0 );
            T valUpRight = ( ( ( x < mip.getWidth() - 1 ) && ( y > 0 ) ) ? mip.getValue( cc, x + 1, y - 1 ) : 0 );
            T newVal     = mean4w( val, w1, valRight, w2, valUp, w3, valUpRight, w4 );
            image.setValue( cc, xUp, yUp, newVal );
          }
        } else if ( ( xUp % 2 == 0 ) && ( yUp % 2 == 1 ) ) {
          w1 = 144;
          w2 = ( x > 0 ? (unsigned char)48 : 0 );
          w3 = ( y < mip.getHeight() - 1 ? (unsigned char)48 : 0 );
          w4 = ( ( ( x > 0 ) && ( y < mip.getHeight() - 1 ) ) ? (unsigned char)16 : 0 );
          for ( int cc = 0; cc < 3; cc++ ) {
            T val         = mip.getValue( cc, x, y );
            T valLeft     = ( x > 0 ? mip.getValue( cc, x - 1, y ) : 0 );
            T valDown     = ( ( y < mip.getHeight() - 1 ) ? mip.getValue( cc, x, y + 1 ) : 0 );
            T valDownLeft = ( ( x > 0 && ( y < mip.getHeight() - 1 ) ) ? mip.getValue( cc, x - 1, y + 1 ) : 0 );
            T newVal      = mean4w( val, w1, valLeft, w2, valDown, w3, valDownLeft, w4 );
            image.setValue( cc, xUp, yUp, newVal );
          }
        } else {
          w1 = 144;
          w2 = ( x < mip.getWidth() - 1 ? (unsigned char)48 : 0 );
          w3 = ( y < mip.getHeight() - 1 ? (unsigned char)48 : 0 );
          w4 = ( ( ( x < mip.getWidth() - 1 ) && ( y < mip.getHeight() - 1 ) ) ? (unsigned char)16 : 0 );
          for ( int cc = 0; cc < 3; cc++ ) {
            T val      = mip.getValue( cc, x, y );
            T valRight = ( x < mip.getWidth() - 1 ? mip.getValue( cc, x + 1, y ) : 0 );
            T valDown  = ( ( y < mip.getHeight() - 1 ) ? mip.getValue( cc, x, y + 1 ) : 0 );
            T valDownRight =
                ( ( ( x < mip.getWidth() - 1 ) && ( y < mip.getHeight() - 1 ) ) ? mip.getValue( cc, x + 1, y + 1 )
                                                                                : 0 );
            T newVal = mean4w( val, w1, valRight, w2, valDown, w3, valDownRight, w4 );
            image.setValue( cc, xUp, yUp, newVal );
          }
        }
      }
    }
  }
}

template <typename T>
void PCCEncoder::dilatePullPush( PCCFrameContext& frame, PCCImage<T, 3>& image ) {
  auto                               occupancyMapTemp = frame.getOccupancyMap();
  int                                i                = 0;
  std::vector<PCCImage<T, 3>>        mipVec;
  std::vector<std::vector<uint32_t>> mipOccupancyMapVec;
  int                                div    = 2;
  int                                miplev = 0;

  // pull phase create the mipmap
  while ( 1 ) {
    mipVec.resize( mipVec.size() + 1 );
    mipOccupancyMapVec.resize( mipOccupancyMapVec.size() + 1 );
    div *= 2;
    if ( miplev > 0 ) {
      pullPushMip( mipVec[miplev - 1], mipVec[miplev], mipOccupancyMapVec[miplev - 1], mipOccupancyMapVec[miplev] );
    } else {
      pullPushMip( image, mipVec[miplev], occupancyMapTemp, mipOccupancyMapVec[miplev] );
    }
    if ( mipVec[miplev].getWidth() <= 4 || mipVec[miplev].getHeight() <= 4 ) { break; }
    ++miplev;
  }
  miplev++;
#if DEBUG_PATCH
  for ( int k = 0; k < miplev; k++ ) {
    char buf[100];
    sprintf( buf, "mip%02i.rgb", k );
    std::string filename = addVideoFormat( buf, mipVec[k].getWidth(), mipVec[k].getHeight(), false );
    mipVec[k].write( filename, 1 );
  }
#endif
  // push phase: refill
  for ( i = miplev - 1; i >= 0; --i ) {
    if ( i > 0 ) {
      pullPushFill( mipVec[i - 1], mipVec[i], mipOccupancyMapVec[i - 1] );
    } else {
      pullPushFill( image, mipVec[i], occupancyMapTemp );
    }
  }
#if DEBUG_PATCH
  for ( int k = 0; k < miplev; k++ ) {
    char buf[100];
    sprintf( buf, "mipfill%02i.rgb", k );
    std::string filename = addVideoFormat( buf, mipVec[k].getWidth(), mipVec[k].getHeight(), false );
    mipVec[k].write( filename, 1 );
  }
#endif
}

void PCCEncoder::presmoothPointCloudColor( PCCPointSet3& reconstruct, const PCCEncoderParameters params ) {
  const size_t            pointCount = reconstruct.getPointCount();
  PCCKdTree               kdtree( reconstruct );
  PCCNNResult             result;
  std::vector<PCCColor3B> temp;
  temp.resize( pointCount );
  for ( size_t m = 0; m < pointCount; ++m ) { temp[m] = reconstruct.getColor( m ); }
  tbb::task_arena limited( (int)params.nbThread_ );
  limited.execute( [&] {
    tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) {
      //  for (size_t i = 0; i < pointCount; ++i) {

      PCCNNResult result;
      if ( reconstruct.getBoundaryPointType( i ) == 2 ) {
        kdtree.searchRadius( reconstruct[i], params.neighborCountColorSmoothing_, params.radius2ColorSmoothing_,
                             result );
        PCCVector3D          centroid( 0.0 );
        size_t               neighborCount = 0;
        std::vector<uint8_t> Lum;
        for ( size_t r = 0; r < result.count(); ++r ) {
          const double dist2 = result.dist( r );
          if ( dist2 > params.radius2ColorSmoothing_ ) { break; }
          ++neighborCount;
          const size_t index = result.indices( r );
          PCCColor3B   color = reconstruct.getColor( index );
          centroid[0] += double( color[0] );
          centroid[1] += double( color[1] );
          centroid[2] += double( color[2] );

          double Y = 0.2126 * double( color[0] ) + 0.7152 * double( color[1] ) + 0.0722 * double( color[2] );
          Lum.push_back( uint8_t( Y ) );
        }

        PCCColor3B color;
        if ( neighborCount ) {
          for ( size_t k = 0; k < 3; ++k ) {
            centroid[k] = double( int64_t( centroid[k] + ( neighborCount / 2 ) ) / neighborCount );
          }

          // Texture characterization
          double     H               = entropy( Lum, int( neighborCount ) );
          PCCColor3B colorQP         = reconstruct.getColor( i );
          double     distToCentroid2 = 0;
          for ( size_t k = 0; k < 3; ++k ) { distToCentroid2 += abs( centroid[k] - double( colorQP[k] ) ); }
          if ( distToCentroid2 >= double( params.thresholdColorSmoothing_ ) &&
               H < double( params.thresholdLocalEntropy_ ) ) {
            color[0] = uint8_t( centroid[0] );
            color[1] = uint8_t( centroid[1] );
            color[2] = uint8_t( centroid[2] );
            temp[i]  = color;
          }
        }
      }
    } );
  } );

  limited.execute( [&] {
    tbb::parallel_for( size_t( 0 ), pointCount, [&]( const size_t i ) {
      // for (size_t i = 0; i < pointCount; ++i) {
      reconstruct.setColor( i, temp[i] );
    } );
  } );
}

bool PCCEncoder::generateTextureVideo( const PCCGroupOfFrames&    sources,
                                       PCCGroupOfFrames&          reconstructs,
                                       PCCContext&                context,
                                       const PCCEncoderParameters params ) {
  auto& frames       = context.getFrames();
  auto& videoTexture = context.getVideoTexture();
  bool  ret          = true;
  auto& sps          = context.getSps();
  for ( size_t i = 0; i < frames.size(); i++ ) {
    auto& frame = frames[i];
    // assert( frame.getWidth() == context.getWidth() && frame.getHeight() == context.getHeight() );
    frame.setLosslessTexture( sps.getLosslessTexture() );
    size_t nbFrame = params_.oneLayerMode_ ? 1 : 2;
    if ( params_.oneLayerMode_ && !params_.singleLayerPixelInterleaving_ ) {
      // create sub reconstruct point cloud
      PCCPointSet3   subReconstruct;
      vector<size_t> subReconstructIndex;
      subReconstructIndex.clear();
      if ( reconstructs[i].hasColors() ) { subReconstruct.addColors(); }
      size_t numPointSub = 0, numPoint = reconstructs[i].getPointCount();
      auto&  pointToPixel = frame.getPointToPixel();
      for ( size_t j = 0; j < numPoint; j++ ) {
        if ( pointToPixel[j][2] < nbFrame ||
             ( !params_.oneLayerMode_ && pointToPixel[j][2] == IntermediateLayerIndex ) ) {
          numPointSub++;
          subReconstruct.addPoint( reconstructs[i][j] );
          subReconstructIndex.push_back( j );
        }
      }
      subReconstruct.resize( numPointSub );
      sources[i].transfertColors( subReconstruct, int32_t( params_.bestColorSearchRange_ ),
                                  params_.losslessTexture_ == 1 );

      for ( size_t j = 0; j < numPointSub; j++ ) {
        reconstructs[i].setColor( subReconstructIndex[j], subReconstruct.getColor( j ) );
        subReconstruct.setBoundaryPointType( j, reconstructs[i].getBoundaryPointType( subReconstructIndex[j] ) );
      }
      // color pre-smoothing
      if ( !params_.losslessGeo_ && params_.flagColorPreSmoothing_ ) {
        presmoothPointCloudColor( subReconstruct, params );
        for ( size_t j = 0; j < numPointSub; j++ ) {
          reconstructs[i].setColor( subReconstructIndex[j], subReconstruct.getColor( j ) );
        }
      }
    } else {
      sources[i].transfertColors( reconstructs[i], int32_t( params_.bestColorSearchRange_ ),
                                  params_.losslessTexture_ == 1 );
      // color pre-smoothing
      if ( !params_.losslessGeo_ && params_.flagColorPreSmoothing_ ) {
        presmoothPointCloudColor( reconstructs[i], params );
      }
    }
    ret &= generateTextureVideo( reconstructs[i], frame, videoTexture, nbFrame );
  }
  return ret;
}

bool PCCEncoder::generateTextureVideo( const PCCPointSet3& reconstruct,
                                       PCCFrameContext&    frame,
                                       PCCVideoTexture&    video,
                                       const size_t        frameCount ) {
  auto&  pointToPixel                 = frame.getPointToPixel();
  bool   useMissedPointsSeparateVideo = frame.getUseMissedPointsSeparateVideo();
  bool   losslessAtt                  = frame.getLosslessTexture();
  bool   losslessGeo                  = frame.getLosslessGeo();
  bool   lossyMissedPointsPatch       = frame.getUseAdditionalPointsPatch() && ( !losslessGeo );
  auto&  missedPointsPatch            = frame.getMissedPointsPatch();
  size_t numOfMPGeos                  = missedPointsPatch.getMPnumber();
  size_t numEddSavedPoints            = missedPointsPatch.numEddSavedPoints_;
  size_t pointCount                   = reconstruct.getPointCount();

  if ( ( losslessAtt || lossyMissedPointsPatch ) && useMissedPointsSeparateVideo ) {
    pointCount = reconstruct.getPointCount() - numOfMPGeos - numEddSavedPoints;
    if ( missedPointsPatch.sizeofcolor() < ( numOfMPGeos + numEddSavedPoints ) ) {
      missedPointsPatch.resizecolor( numOfMPGeos + numEddSavedPoints );
    }
  }
  //  const size_t pointCount = reconstruct.getPointCount();
  if ( !pointCount || !reconstruct.hasColors() ) { return false; }

  const size_t shift = video.getFrameCount();
  video.resize( shift + frameCount );
  for ( size_t f = 0; f < frameCount; ++f ) {
    auto& image = video.getFrame( f + shift );
    image.resize( frame.getWidth(), frame.getHeight() );
    image.set( 0 );
  }

  std::vector<bool> mapD1;
  if ( !params_.oneLayerMode_ ) { mapD1.resize( frame.getWidth() * frame.getHeight(), false ); }

  std::vector<bool> markT1;
  if ( !params_.oneLayerMode_ && params_.removeDuplicatePoints_ ) {
    const size_t size = frame.getWidth() * frame.getHeight();
    markT1.resize( size );
    for ( size_t i = 0; i < size; i++ ) { markT1[i] = false; }
  }

  for ( size_t i = 0; i < pointCount; ++i ) {
    const PCCVector3<size_t> location = pointToPixel[i];
    const PCCColor3B         color    = reconstruct.getColor( i );
    const size_t             u        = location[0];
    const size_t             v        = location[1];
    const size_t             f        = location[2];
    if ( params_.oneLayerMode_ && params_.singleLayerPixelInterleaving_ ) {
      if ( ( f == 0 && ( ( u + v ) % 2 == 0 ) ) || ( f == 1 && ( ( u + v ) % 2 == 1 ) ) ) {
        auto& image = video.getFrame( shift );
        image.setValue( 0, u, v, color[0] );
        image.setValue( 1, u, v, color[1] );
        image.setValue( 2, u, v, color[2] );
      }
    } else {
      if ( f < frameCount ) {
        auto& image = video.getFrame( f + shift );
        image.setValue( 0, u, v, color[0] );
        image.setValue( 1, u, v, color[1] );
        image.setValue( 2, u, v, color[2] );
      }
      if ( !params_.oneLayerMode_ && params_.removeDuplicatePoints_ ) {
        if ( f == 0 ) {
          if ( !markT1[v * frame.getWidth() + u] ) {
            auto& image1 = video.getFrame( 1 + shift );
            image1.setValue( 0, u, v, color[0] );
            image1.setValue( 1, u, v, color[1] );
            image1.setValue( 2, u, v, color[2] );
          }
        } else {
          markT1[v * frame.getWidth() + u] = true;
        }
      }
    }
  }
  if ( ( losslessAtt || lossyMissedPointsPatch ) && useMissedPointsSeparateVideo ) {
    if ( params_.enhancedDeltaDepthCode_ ) {
      for ( size_t i = 0; i < numEddSavedPoints; ++i ) {
        const PCCColor3B color  = reconstruct.getColor( pointCount + i );
        missedPointsPatch.r_[i] = color[0];
        missedPointsPatch.g_[i] = color[1];
        missedPointsPatch.b_[i] = color[2];
      }
    }
    // missed points
    for ( size_t i = 0; i < numOfMPGeos; ++i ) {
      const PCCColor3B color                      = reconstruct.getColor( pointCount + numEddSavedPoints + i );
      missedPointsPatch.r_[numEddSavedPoints + i] = color[0];
      missedPointsPatch.g_[numEddSavedPoints + i] = color[1];
      missedPointsPatch.b_[numEddSavedPoints + i] = color[2];
    }
  }
  return true;
}

void PCCEncoder::generateIntraEnhancedDeltaDepthImage( PCCFrameContext&        frame,
                                                       const PCCImageGeometry& imageRef,
                                                       PCCImageGeometry&       image ) {
  size_t width  = frame.getWidth();
  size_t height = frame.getHeight();
  image.resize( width, height );
  image.set( 0 );
  const int16_t infiniteDepth = ( std::numeric_limits<int16_t>::max )();
  for ( auto& patch : frame.getPatches() ) {
    const size_t v0 = patch.getV0() * patch.getOccupancyResolution();
    const size_t u0 = patch.getU0() * patch.getOccupancyResolution();
    for ( size_t v = 0; v < patch.getSizeV(); ++v ) {
      for ( size_t u = 0; u < patch.getSizeU(); ++u ) {
        const size_t  p = v * patch.getSizeU() + u;
        const int16_t d = patch.getDepth( 0 )[p];
        if ( d < infiniteDepth ) {
          const int16_t enhancedDeltaD = patch.getDepthEnhancedDeltaD()[p];
          size_t        x, y;
          patch.patch2Canvas( u, v, width, height, x, y );
          image.setValue( 0, x, y, uint16_t( enhancedDeltaD + imageRef.getValue( 0, x, y ) ) );
        }
      }
    }
  }

  if ( !frame.getUseMissedPointsSeparateVideo() ) {
    // missed point patch
    auto&        missedPointsPatch = frame.getMissedPointsPatch();
    const size_t v0                = missedPointsPatch.v0_ * missedPointsPatch.occupancyResolution_;
    const size_t u0                = missedPointsPatch.u0_ * missedPointsPatch.occupancyResolution_;
    if ( missedPointsPatch.size() ) {
      for ( size_t v = 0; v < missedPointsPatch.sizeV_; ++v ) {
        for ( size_t u = 0; u < missedPointsPatch.sizeU_; ++u ) {
          const size_t p = v * missedPointsPatch.sizeU_ + u;
          if ( missedPointsPatch.x_[p] < infiniteDepth ) {
            const size_t x = ( u0 + u );
            const size_t y = ( v0 + v );
            assert( x < width && y < height );
            image.setValue( 0, x, y, uint16_t( missedPointsPatch.x_[p] ) );
            if ( params_.losslessGeo444_ ) {
              image.setValue( 1, x, y, uint16_t( missedPointsPatch.y_[p] ) );
              image.setValue( 2, x, y, uint16_t( missedPointsPatch.z_[p] ) );
            }
          }
        }
      }
    }
  }
  return;
}

void PCCEncoder::performDataAdaptiveGPAMethod( PCCContext& context ) {
  // some valid parameters;
  SubContext    subContextPre, subContextCur;  // [start, end);
  unionPatch    unionPatchPre, unionPatchCur;  // [trackIndex, patchUnion];
  GlobalPatches globalPatchTracks;             // [trackIndex, <[frameIndex, patchIndex]>];
  bool          startSubContext      = true;   // startSubContext is initialized as true;  start a subContext;
  bool          endSubContext        = false;  // endSubContext   is initialized as false; end   a subContext;
  int           preSubcontextFrameId = -1;
  size_t        subcontextIdx        = 0;

  // iterate over all frameContexts;
  for ( size_t frameIndex = 0; frameIndex < context.size(); ++frameIndex ) {
    bool useRefFrame = true /*params_.keepGPARotation_*/;
    // determine whether start a subContext or not;
    if ( startSubContext ) {
      initializeSubContext( context[frameIndex], subContextPre, globalPatchTracks, unionPatchPre, frameIndex );

      if ( subContextPre.first == 0 ) { useRefFrame = false; }
      packingFirstFrame( context, frameIndex, params_.packingStrategy_, params_.safeGuardDistance_, useRefFrame );

      context[subContextPre.first].getPrePCCGPAFrameSize() = context[subContextPre.first].getCurPCCGPAFrameSize();
      context[subContextPre.first].getCurPCCGPAFrameSize().widthGPA_  = 0;
      context[subContextPre.first].getCurPCCGPAFrameSize().heightGPA_ = 0;
      for ( auto& patch : context[subContextPre.first].getPatches() ) {
        patch.getPreGPAPatchData() = patch.getCurGPAPatchData();
        patch.getCurGPAPatchData().initialize();
      }
      if ( frameIndex == context.size() - 1 ) {
        context.getSubContexts().emplace_back( subContextPre );
        updatePatchInformation( context, subContextPre );
        break;
      }
      subContextCur   = subContextPre;
      startSubContext = false;
      continue;
    }

    subContextCur.first  = subContextPre.first;
    subContextCur.second = frameIndex + 1;
    preSubcontextFrameId = subContextCur.first - 1;
    if ( subContextCur.first == 0 ) {
      useRefFrame          = false;
      preSubcontextFrameId = -1;
    }

    // clear current information;
    clearCurrentGPAPatchDataInfor( context, subContextCur );

    // genrate globalPatchTracks;
    size_t preIndex =
        frameIndex - subContextCur.first - 1;  // preIndex is the previous index in the current subcontext.
    generateGlobalPatches( context, frameIndex, globalPatchTracks, preIndex );

    // patch unions generation and packing;
    size_t unionsHeight = unionPatchGenerationAndPacking(
        globalPatchTracks, context, unionPatchCur, preSubcontextFrameId, params_.safeGuardDistance_, useRefFrame );

    // perform GPA packing;
    bool badPatchCount   = false;
    bool badUnionsHeight = false;
    bool badGPAPacking   = false;
    if ( double( unionPatchCur.size() ) / globalPatchTracks.size() < 0.15 ) { badPatchCount = true; }
    if ( unionsHeight > params_.minimumImageHeight_ ) { badUnionsHeight = true; }
    if ( printDetailedInfo ) {
      std::cout << "badPatchCount: " << badPatchCount << "badUnionsHeight: " << badUnionsHeight << std::endl;
    }
    if ( !badPatchCount && !badUnionsHeight ) {
      // patch information updating;
      updateGPAPatchInformation( context, subContextCur, unionPatchCur );

      // save the data into preGPAPatchData.
      performGPAPacking( subContextCur, unionPatchCur, context, badGPAPacking, unionsHeight, params_.safeGuardDistance_,
                         useRefFrame );
    }

    endSubContext = ( badPatchCount || badUnionsHeight || badGPAPacking );
    std::cout << "The endSubContext is: " << endSubContext << std::endl;

    if ( endSubContext ) {
      std::cout << "The frame is a end point --- " << frameIndex << std::endl;
      // clear current information;
      clearCurrentGPAPatchDataInfor( context, subContextCur );
      assert( subContextCur.second - subContextCur.first > 1 );

      subContextCur.first  = 0;
      subContextCur.second = 0;
      unionPatchCur.clear();
      globalPatchTracks.clear();  // GlobalPatches.......;
      // retain previous information;
      context.getSubContexts().emplace_back( subContextPre );  // SubContext..........;
      startSubContext = true;
      endSubContext   = false;
      frameIndex -= 1;  // should stay at the start point for next subcontext.

      // update Patch information;
      updatePatchInformation( context, subContextPre );
    } else {
      std::cout << "The frame " << frameIndex << " is not a end point ---" << std::endl;
      // previous information updating;
      for ( size_t j = subContextCur.first; j < subContextCur.second; ++j ) {
        auto& curPatches                   = context[j].getPatches();
        context[j].getPrePCCGPAFrameSize() = context[j].getCurPCCGPAFrameSize();
        assert( !curPatches.empty() );
        for ( auto& curPatch : curPatches ) { curPatch.getPreGPAPatchData() = curPatch.getCurGPAPatchData(); }
        if ( context[j].getMissedPointsPatch().size() > 0 && !context[j].getUseMissedPointsSeparateVideo() ) {
          context[j].getMissedPointsPatch().preV0_ = context[j].getMissedPointsPatch().tempV0_;
        }
      }
      subContextPre = subContextCur;
      unionPatchPre.clear();
      unionPatchPre = unionPatchCur;
      std::cout << "cleared current tried infor:" << std::endl;
      // clear current information;
      for ( size_t j = subContextCur.first; j < subContextCur.second; ++j ) {
        auto& curPatches = context[j].getPatches();
        assert( !curPatches.empty() );
        for ( auto& curPatch : curPatches ) { curPatch.getCurGPAPatchData().initialize(); }
      }
      subContextCur.first  = 0;
      subContextCur.second = 0;
      unionPatchCur.clear();
      // the ending......;
      if ( frameIndex == ( context.size() - 1 ) ) {
        context.getSubContexts().emplace_back( subContextPre );  // SubContext..........;
        std::cout << "This is the last frame......." << std::endl;

        // update information;
        updatePatchInformation( context, subContextPre );
        break;
      }
    }
  }
}

void PCCEncoder::initializeSubContext( PCCFrameContext& frameContext,
                                       SubContext&      subContext,
                                       GlobalPatches&   globalPatchTracks,
                                       unionPatch&      unionPatch,
                                       size_t           frameIndex ) {
  // 1. initialize subContext;
  subContext.first  = frameIndex;
  subContext.second = frameIndex + 1;
  std::cout << "New subContext:[" << subContext.first << "," << subContext.second << ")" << std::endl;

  // 2. initialize globalPatchTracks && unionPatch;
  unionPatch.clear();
  globalPatchTracks.clear();
  for ( size_t patchIndex = 0; patchIndex < frameContext.getPatches().size(); ++patchIndex ) {
    globalPatchTracks[patchIndex].emplace_back( std::make_pair( frameIndex, patchIndex ) );
    frameContext.getPatches()[patchIndex].getCurGPAPatchData().isGlobalPatch    = true;
    frameContext.getPatches()[patchIndex].getCurGPAPatchData().globalPatchIndex = patchIndex;
  }
}
void PCCEncoder::clearCurrentGPAPatchDataInfor( PCCContext& context, SubContext& subContext ) {
  // clear current information;
  for ( size_t j = subContext.first; j < subContext.second; ++j ) {
    auto& curPatches                          = context[j].getPatches();
    context[j].getMissedPointsPatch().tempV0_ = 0;
    assert( !curPatches.empty() );
    for ( auto& curPatch : curPatches ) { curPatch.getCurGPAPatchData().initialize(); }
  }
}
void PCCEncoder::generateGlobalPatches( PCCContext&    context,
                                        size_t         frameIndex,
                                        GlobalPatches& globalPatchTracks,
                                        size_t         preIndex ) {
  auto& curPatches = context[frameIndex].getPatches();
  assert( curPatches.size() > 0 );
  for ( GlobalPatches::iterator iter = globalPatchTracks.begin(); iter != globalPatchTracks.end(); iter++ ) {
    size_t trackIndex   = iter->first;
    auto&  trackPatches = iter->second;  // !!!< <frameIndex, patchIndex> >;
    if ( trackPatches.empty() ) { continue; }
    const auto& preGlobalPatch = trackPatches[preIndex];
    const auto& prePatch       = context[preGlobalPatch.first].getPatches()[preGlobalPatch.second];
    float       thresholdIOU   = 0.2;
    float       maxIou         = 0.0;
    int         bestIdx        = -1;       // best matched patch index in curPatches;
    int         cId            = 0;        // patch index in curPatches;
    for ( auto& curPatch : curPatches ) {  // curPatches; may be modified;
      if ( prePatch.getViewId() == curPatch.getViewId() && !( curPatch.getCurGPAPatchData().isMatched ) ) {
        Rect  preRect = Rect( prePatch.getU1(), prePatch.getV1(), prePatch.getSizeU(), prePatch.getSizeV() );
        Rect  curRect = Rect( curPatch.getU1(), curPatch.getV1(), curPatch.getSizeU(), curPatch.getSizeV() );
        float iou     = computeIOU( preRect, curRect );
        if ( iou > maxIou ) {
          maxIou  = iou;
          bestIdx = cId;
        }
      }
      cId++;
    }
    if ( maxIou > thresholdIOU ) {                                // !!!best match found;
      curPatches[bestIdx].getCurGPAPatchData().isMatched = true;  // indicating the patch is already matched;
      trackPatches.emplace_back( std::make_pair( frameIndex, bestIdx ) );
    } else {
      trackPatches.clear();
    }
  }

  // update global patch information according to curGlobalPatches;
  for ( GlobalPatches::iterator iter = globalPatchTracks.begin(); iter != globalPatchTracks.end(); iter++ ) {
    const size_t trackIndex   = iter->first;
    const auto&  trackPatches = iter->second;  // !!!< <frameIndex, patchIndex> >;
    if ( trackPatches.empty() ) { continue; }
    for ( const auto& trackPatch : trackPatches ) {
      GPAPatchData& curGPAPatchData    = context[trackPatch.first].getPatches()[trackPatch.second].getCurGPAPatchData();
      curGPAPatchData.isGlobalPatch    = true;
      curGPAPatchData.globalPatchIndex = trackIndex;
    }
  }
}

size_t PCCEncoder::unionPatchGenerationAndPacking( const GlobalPatches& globalPatchTracks,
                                                   PCCContext&          context,
                                                   unionPatch&          unionPatchTemp,
                                                   size_t               refFrameIdx,
                                                   int                  safeguard,
                                                   bool                 useRefFrame ) {
  // 1. unionPatch generation;
  unionPatchTemp.clear();
  // 1.1 patchTracks generation;
  std::map<size_t, std::vector<PCCPatch>> patchTracks;
  for ( GlobalPatches::const_iterator iter = globalPatchTracks.begin(); iter != globalPatchTracks.end(); iter++ ) {
    const auto& trackIndex   = iter->first;
    const auto& trackPatches = iter->second;
    if ( trackPatches.empty() ) { continue; }
    for ( const auto& trackPatch : trackPatches ) {
      patchTracks[trackIndex].emplace_back( context[trackPatch.first].getPatches()[trackPatch.second] );
    }
  }
  // 1.2 union processing --- patchTracks -> unionPatch;
  for ( std::map<size_t, std::vector<PCCPatch>>::const_iterator iter = patchTracks.begin(); iter != patchTracks.end();
        iter++ ) {
    const auto& trackIndex   = iter->first;
    const auto& trackPatches = iter->second;
    assert( trackPatches.size() != 0 );
    // get the sizeU0 && sizeV0;
    size_t maxSizeU0 = 0;
    size_t maxSizeV0 = 0;
    for ( const auto& trackPatch : trackPatches ) {
      maxSizeU0 = std::max<size_t>( maxSizeU0, trackPatch.getSizeU0() );
      maxSizeV0 = std::max<size_t>( maxSizeV0, trackPatch.getSizeV0() );
    }

    // get the patch union;
    PCCPatch curPatchUnion;
    curPatchUnion.getIndex()  = trackIndex;
    curPatchUnion.getSizeU0() = maxSizeU0;
    curPatchUnion.getSizeV0() = maxSizeV0;
    curPatchUnion.getOccupancy().resize( maxSizeU0 * maxSizeV0, false );
    if ( useRefFrame && ( trackPatches.size() > 0 ) ) {
      assert( refFrameIdx != -1 );
      size_t matchedPatchIdx = trackPatches[0].getBestMatchIdx();  // the first frame in the subcontext.
      if ( matchedPatchIdx == -1 ) {
        curPatchUnion.getPatchOrientation() = -1;
      } else {  // suppose the refFrame is the same frame for all patches.
        curPatchUnion.getPatchOrientation() = context[refFrameIdx].getPatches()[matchedPatchIdx].getPatchOrientation();
        if ( printDetailedInfo ) {
          std::cout << "Maintained orientation for curPatchUnion.getPatchOrientation() = "
                    << curPatchUnion.getPatchOrientation() << std::endl;
        }
      }
    }
    for ( const auto& trackPatch : trackPatches ) {
      const auto& occupancy = trackPatch.getOccupancy();
      for ( size_t v = 0; v < trackPatch.getSizeV0(); ++v ) {
        for ( size_t u = 0; u < trackPatch.getSizeU0(); ++u ) {
          assert( v < maxSizeV0 );
          assert( u < maxSizeU0 );
          size_t p  = v * trackPatch.getSizeU0() + u;
          size_t up = v * curPatchUnion.getSizeU0() + u;
          if ( occupancy[p] && !( curPatchUnion.getOccupancy()[up] ) ) curPatchUnion.getOccupancy()[up] = true;
        }
      }
    }
    unionPatchTemp[trackIndex] = curPatchUnion;
  }

  // 2. unionPatch packing;
  size_t occupancySizeU = params_.minimumImageWidth_ / params_.occupancyResolution_;
  size_t occupancySizeV = ( std::max )( unionPatchTemp[0].getSizeV0(), unionPatchTemp[0].getSizeU0() );
  for ( unionPatch::const_iterator iter = unionPatchTemp.begin(); iter != unionPatchTemp.end(); iter++ ) {
    const auto& curPatchUnion = iter->second;
    occupancySizeU            = std::max<size_t>( occupancySizeU, curPatchUnion.getSizeU0() + 1 );
    occupancySizeV            = std::max<size_t>( occupancySizeV, curPatchUnion.getSizeV0() + 1 );
  }
  size_t width  = occupancySizeU * params_.occupancyResolution_;
  size_t height = occupancySizeV * params_.occupancyResolution_;
  size_t maxOccupancyRow{0};

  std::vector<bool> occupancyMap;
  vector<int>       orientation_vertical   = {PatchOrientation::DEFAULT,
                                      PatchOrientation::SWAP};  // favoring vertical orientation
  vector<int>       orientation_horizontal = {
      PatchOrientation::SWAP, PatchOrientation::DEFAULT};  // favoring horizontal orientations (that should be rotated)
  occupancyMap.resize( occupancySizeU * occupancySizeV, false );
  for ( unionPatch::iterator iter = unionPatchTemp.begin(); iter != unionPatchTemp.end(); iter++ ) {
    auto& curUnionIndex = iter->first;
    auto& curPatchUnion = iter->second;  // [u0, v0] may be modified;
    assert( curPatchUnion.getSizeU0() < occupancySizeU );
    assert( curPatchUnion.getSizeV0() < occupancySizeV );
    bool  locationFound = false;
    auto& occupancy     = curPatchUnion.getOccupancy();
    while ( !locationFound ) {
      for ( size_t v = 0; v < occupancySizeV && !locationFound; ++v ) {
        for ( size_t u = 0; u < occupancySizeU && !locationFound; ++u ) {
          curPatchUnion.getU0() = u;
          curPatchUnion.getV0() = v;
          if ( params_.packingStrategy_ == 0 ) {
            curPatchUnion.getPatchOrientation() = PatchOrientation::DEFAULT;
            if ( curPatchUnion.checkFitPatchCanvas( occupancyMap, occupancySizeU, occupancySizeV, safeguard ) ) {
              locationFound = true;
              if ( printDetailedInfo ) {
                std::cout << "Orientation " << curPatchUnion.getPatchOrientation() << " selected for unionPatch "
                          << curPatchUnion.getIndex() << " (" << u << "," << v << ")" << std::endl;
              }
            }
          } else {
            if ( useRefFrame && ( curPatchUnion.getPatchOrientation() != -1 ) ) {
              // already knonw Patch Orientation. just try.
              if ( curPatchUnion.checkFitPatchCanvas( occupancyMap, occupancySizeU, occupancySizeV, safeguard ) ) {
                locationFound = true;
                if ( printDetailedInfo ) {
                  std::cout << "location u0,v0 selected for unionPatch " << curPatchUnion.getIndex() << " (" << u << ","
                            << v << ")" << std::endl;
                }
              }
            } else {
              for ( size_t orientationIdx = 0; orientationIdx < pow( 2, params_.packingStrategy_ ) && !locationFound;
                    orientationIdx++ ) {
                if ( curPatchUnion.getSizeU0() > curPatchUnion.getSizeV0() ) {
                  curPatchUnion.getPatchOrientation() = orientation_horizontal[orientationIdx];
                } else {
                  curPatchUnion.getPatchOrientation() = orientation_vertical[orientationIdx];
                }
                if ( curPatchUnion.checkFitPatchCanvas( occupancyMap, occupancySizeU, occupancySizeV, safeguard ) ) {
                  locationFound = true;
                  if ( printDetailedInfo ) {
                    std::cout << "Orientation " << curPatchUnion.getPatchOrientation() << " selected for unionPatch "
                              << curPatchUnion.getIndex() << " (" << u << "," << v << ")" << std::endl;
                  }
                }
              }
            }
          }
        }
      }
      if ( !locationFound ) {
        occupancySizeV *= 2;
        occupancyMap.resize( occupancySizeU * occupancySizeV );
      }
    }
    for ( size_t v0 = 0; v0 < curPatchUnion.getSizeV0(); ++v0 ) {
      for ( size_t u0 = 0; u0 < curPatchUnion.getSizeU0(); ++u0 ) {
        int coord           = curPatchUnion.patchBlock2CanvasBlock( u0, v0, occupancySizeU, occupancySizeV );
        occupancyMap[coord] = occupancyMap[coord] || occupancy[v0 * curPatchUnion.getSizeU0() + u0];
      }
    }

    if ( !( curPatchUnion.isPatchDimensionSwitched() ) ) {
      height =
          ( std::max )( height, ( curPatchUnion.getV0() + curPatchUnion.getSizeV0() ) * params_.occupancyResolution_ );
      width =
          ( std::max )( width, ( curPatchUnion.getU0() + curPatchUnion.getSizeU0() ) * params_.occupancyResolution_ );
      maxOccupancyRow = ( std::max )( maxOccupancyRow, ( curPatchUnion.getV0() + curPatchUnion.getSizeV0() ) );
    } else {
      height =
          ( std::max )( height, ( curPatchUnion.getV0() + curPatchUnion.getSizeU0() ) * params_.occupancyResolution_ );
      width =
          ( std::max )( width, ( curPatchUnion.getU0() + curPatchUnion.getSizeV0() ) * params_.occupancyResolution_ );
      maxOccupancyRow = ( std::max )( maxOccupancyRow, ( curPatchUnion.getV0() + curPatchUnion.getSizeU0() ) );
    }
  }
  std::cout << "actualImageSizeU " << width << std::endl;
  std::cout << "actualImageSizeV " << height << std::endl;
  return height;
}

void PCCEncoder::packingFirstFrame( PCCContext& context,
                                    size_t      frameIndex,
                                    bool        packingStrategy,
                                    int         safeguard,
                                    bool        hasRefFrame ) {
  PCCFrameContext& frame          = context[frameIndex];
  auto&            patches        = frame.getPatches();
  size_t           occupancySizeU = params_.minimumImageWidth_ / params_.occupancyResolution_;
  size_t           occupancySizeV = ( std::max )( patches[0].getSizeU0(), patches[0].getSizeV0() );

  for ( auto& patch : patches ) { occupancySizeU = ( std::max )( occupancySizeU, patch.getSizeU0() + 1 ); }
  auto& widthGPA = frame.getCurPCCGPAFrameSize().widthGPA_;
  auto& heithGPA = frame.getCurPCCGPAFrameSize().heightGPA_;
  widthGPA       = occupancySizeU * params_.occupancyResolution_;
  heithGPA       = occupancySizeV * params_.occupancyResolution_;
  size_t maxOccupancyRow{0};

  if ( packingStrategy == 0 ) {
    std::vector<bool> occupancyMap;
    occupancyMap.resize( occupancySizeU * occupancySizeV, false );
    for ( auto& patch : patches ) {
      assert( patch.getSizeU0() <= occupancySizeU );
      assert( patch.getSizeV0() <= occupancySizeV );
      bool  locationFound = false;
      auto& occupancy     = patch.getOccupancy();
      while ( !locationFound ) {
        patch.getPatchOrientation() = PatchOrientation::DEFAULT;  // only one orientation is allowed
        for ( int v = 0; v <= occupancySizeV && !locationFound; ++v ) {
          for ( int u = 0; u <= occupancySizeU && !locationFound; ++u ) {
            patch.getU0() = u;
            patch.getV0() = v;
            if ( patch.checkFitPatchCanvas( occupancyMap, occupancySizeU, occupancySizeV, safeguard ) ) {
              locationFound = true;
            }
          }
        }
        if ( !locationFound ) {
          occupancySizeV *= 2;
          occupancyMap.resize( occupancySizeU * occupancySizeV );
        }
      }
      for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
        const size_t v = patch.getV0() + v0;
        for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
          const size_t u = patch.getU0() + u0;
          occupancyMap[v * occupancySizeU + u] =
              occupancyMap[v * occupancySizeU + u] || occupancy[v0 * patch.getSizeU0() + u0];
        }
      }

      heithGPA = ( std::max )( heithGPA, ( patch.getV0() + patch.getSizeV0() ) * patch.getOccupancyResolution() );
      widthGPA = ( std::max )( widthGPA, ( patch.getU0() + patch.getSizeU0() ) * patch.getOccupancyResolution() );
      maxOccupancyRow = ( std::max )( maxOccupancyRow, ( patch.getV0() + patch.getSizeV0() ) );
    }

    if ( frame.getMissedPointsPatch().size() > 0 && !frame.getUseMissedPointsSeparateVideo() ) {
      packMissedPointsPatch( frame, occupancyMap, widthGPA, heithGPA, occupancySizeU, occupancySizeV, maxOccupancyRow );
      frame.getMissedPointsPatch().preV0_ = frame.getMissedPointsPatch().v0_;
    } else {
      if ( printDetailedInfo ) { printMap( occupancyMap, occupancySizeU, occupancySizeV ); }
    }
    std::cout << "actualImageSizeU " << widthGPA << std::endl;
    std::cout << "actualImageSizeV " << heithGPA << std::endl;
  } else {
    // vector<int> orientation_vertical = { PatchOrientation::DEFAULT,PatchOrientation::ROT90,
    // PatchOrientation::ROT180,PatchOrientation::ROT270,PatchOrientation::MIRROR,PatchOrientation::MROT180,
    // PatchOrientation::MROT90,PatchOrientation::MROT270 };    // favoring vertical orientation
    // vector<int> orientation_horizontal = { PatchOrientation::ROT90,PatchOrientation::DEFAULT,
    // PatchOrientation::ROT270,PatchOrientation::ROT180,PatchOrientation::MROT90,PatchOrientation::MROT270,
    // PatchOrientation::MIRROR,PatchOrientation::MROT180 };    // favoring horizontal orientations
    // (that should be rotated)
    vector<int> orientation_vertical   = {PatchOrientation::DEFAULT,
                                        PatchOrientation::SWAP};  // favoring vertical orientation
    vector<int> orientation_horizontal = {
        PatchOrientation::SWAP,
        PatchOrientation::DEFAULT};  // favoring horizontal orientations (that should be rotated)
    std::vector<bool> occupancyMap;
    occupancyMap.resize( occupancySizeU * occupancySizeV, false );

    for ( auto& patch : patches ) {
      assert( patch.getSizeU0() <= occupancySizeU );
      assert( patch.getSizeV0() <= occupancySizeV );
      bool          locationFound   = false;
      auto&         occupancy       = patch.getOccupancy();
      GPAPatchData& curGPAPatchData = patch.getCurGPAPatchData();
      curGPAPatchData.occupancy     = occupancy;
      curGPAPatchData.sizeU0        = patch.getSizeU0();
      curGPAPatchData.sizeV0        = patch.getSizeV0();

      while ( !locationFound ) {
        // no ref patch for first frame.
        if ( ( patch.getBestMatchIdx() != InvalidPatchIndex ) && ( hasRefFrame ) ) {
          std::vector<PCCPatch>& prevPatches = context[frameIndex - 1].getPatches();
          curGPAPatchData.patchOrientation   = prevPatches[patch.getBestMatchIdx()].getPatchOrientation();
          // try to place on the same position as the matched patch
          curGPAPatchData.u0 = prevPatches[patch.getBestMatchIdx()].getU0();
          curGPAPatchData.v0 = prevPatches[patch.getBestMatchIdx()].getV0();
          if ( patch.checkFitPatchCanvasForGPA( occupancyMap, occupancySizeU, occupancySizeV ) ) {
            locationFound = true;
            if ( printDetailedInfo ) {
              std::cout << "Maintained orientation " << curGPAPatchData.patchOrientation
                        << " for matched patch in the same position (" << curGPAPatchData.u0 << ","
                        << curGPAPatchData.v0 << ")" << std::endl;
            }
          }
          // if the patch couldn't fit, try to fit the patch in the top left position
          for ( int v = 0; v <= occupancySizeV && !locationFound; ++v ) {
            for ( int u = 0; u <= occupancySizeU && !locationFound; ++u ) {
              curGPAPatchData.u0 = u;
              curGPAPatchData.v0 = v;
              if ( patch.checkFitPatchCanvasForGPA( occupancyMap, occupancySizeU, occupancySizeV, safeguard ) ) {
                locationFound = true;
                if ( printDetailedInfo ) {
                  std::cout << "Maintained orientation " << curGPAPatchData.patchOrientation << " for matched patch:("
                            << curGPAPatchData.u0 << "," << curGPAPatchData.v0 << ")" << std::endl;
                }
              }
            }
          }
        } else {
          // best effort
          for ( size_t v = 0; v < occupancySizeV && !locationFound; ++v ) {
            for ( size_t u = 0; u < occupancySizeU && !locationFound; ++u ) {
              curGPAPatchData.u0 = u;
              curGPAPatchData.v0 = v;
              for ( size_t orientationIdx = 0; orientationIdx < pow( 2, params_.packingStrategy_ ) && !locationFound;
                    orientationIdx++ ) {
                if ( curGPAPatchData.sizeU0 > curGPAPatchData.sizeV0 ) {
                  curGPAPatchData.patchOrientation = orientation_horizontal[orientationIdx];
                } else {
                  curGPAPatchData.patchOrientation = orientation_vertical[orientationIdx];
                }
                if ( patch.checkFitPatchCanvasForGPA( occupancyMap, occupancySizeU, occupancySizeV, safeguard ) ) {
                  locationFound = true;
                  if ( printDetailedInfo ) {
                    std::cout << "Orientation " << curGPAPatchData.patchOrientation << " selected for unmatched patch:("
                              << curGPAPatchData.u0 << "," << curGPAPatchData.v0 << ")" << std::endl;
                  }
                }
              }
            }
          }
        }
        if ( !locationFound ) {
          occupancySizeV *= 2;
          occupancyMap.resize( occupancySizeU * occupancySizeV );
        }
      }
      for ( size_t v0 = 0; v0 < curGPAPatchData.sizeV0; ++v0 ) {
        for ( size_t u0 = 0; u0 < curGPAPatchData.sizeU0; ++u0 ) {
          int coord           = patch.patchBlock2CanvasBlockForGPA( u0, v0, occupancySizeU, occupancySizeV );
          occupancyMap[coord] = occupancyMap[coord] || occupancy[v0 * patch.getSizeU0() + u0];
        }
      }
      if ( !( curGPAPatchData.isPatchDimensionSwitched() ) ) {
        heithGPA =
            ( std::max )( heithGPA, ( curGPAPatchData.v0 + curGPAPatchData.sizeV0 ) * patch.getOccupancyResolution() );
        widthGPA =
            ( std::max )( widthGPA, ( curGPAPatchData.u0 + curGPAPatchData.sizeU0 ) * patch.getOccupancyResolution() );
        maxOccupancyRow = ( std::max )( maxOccupancyRow, ( curGPAPatchData.v0 + curGPAPatchData.sizeV0 ) );
      } else {
        heithGPA =
            ( std::max )( heithGPA, ( curGPAPatchData.v0 + curGPAPatchData.sizeU0 ) * patch.getOccupancyResolution() );
        widthGPA =
            ( std::max )( widthGPA, ( curGPAPatchData.u0 + curGPAPatchData.sizeV0 ) * patch.getOccupancyResolution() );
        maxOccupancyRow = ( std::max )( maxOccupancyRow, ( curGPAPatchData.v0 + curGPAPatchData.sizeU0 ) );
      }
    }

    if ( frame.getMissedPointsPatch().size() > 0 && !frame.getUseMissedPointsSeparateVideo() ) {
      packMissedPointsPatch( frame, occupancyMap, widthGPA, heithGPA, occupancySizeU, occupancySizeV, maxOccupancyRow );
      frame.getMissedPointsPatch().preV0_ = frame.getMissedPointsPatch().v0_;
    } else {
      if ( printDetailedInfo ) { printMap( occupancyMap, occupancySizeU, occupancySizeV ); }
    }
    std::cout << "actualImageSizeU " << widthGPA << std::endl;
    std::cout << "actualImageSizeV " << heithGPA << std::endl;
  }
}

void PCCEncoder::updatePatchInformation( PCCContext& context, SubContext& subContext ) {
  std::cout << "The subContext is: [" << subContext.first << ", " << subContext.second << ")" << std::endl;

  for ( size_t frameIndex = subContext.first; frameIndex < subContext.second; ++frameIndex ) {
    PCCFrameContext& frame = context[frameIndex];

    frame.getWidth()  = frame.getPrePCCGPAFrameSize().widthGPA_;
    frame.getHeight() = frame.getPrePCCGPAFrameSize().heightGPA_;
    auto& curPatches  = frame.getPatches();
    for ( auto& curPatch : curPatches ) {
      GPAPatchData& preGPAPatchData  = curPatch.getPreGPAPatchData();
      curPatch.getSizeU0()           = preGPAPatchData.sizeU0;
      curPatch.getSizeV0()           = preGPAPatchData.sizeV0;
      curPatch.getOccupancy()        = preGPAPatchData.occupancy;
      curPatch.getU0()               = preGPAPatchData.u0;
      curPatch.getV0()               = preGPAPatchData.v0;
      curPatch.getPatchOrientation() = preGPAPatchData.patchOrientation;
      curPatch.getIsGlobalPatch()    = preGPAPatchData.isGlobalPatch;
    }

    // update missedPoint patch infor.
    if ( frame.getMissedPointsPatch().size() > 0 && !frame.getUseMissedPointsSeparateVideo() ) {
      frame.getMissedPointsPatch().v0_ = frame.getMissedPointsPatch().preV0_;
    }
  }
  return;
}

void PCCEncoder::updateGPAPatchInformation( PCCContext& context, SubContext& subContext, unionPatch& unionPatch ) {
  for ( size_t i = subContext.first; i < subContext.second; ++i ) {
    auto& patches = context[i].getPatches();
    for ( auto& patch : patches ) {
      GPAPatchData& curGPAPatchData = patch.getCurGPAPatchData();
      if ( curGPAPatchData.isGlobalPatch ) {
        size_t            globalIndex      = curGPAPatchData.globalPatchIndex;
        auto&             cPatchUnion      = unionPatch[globalIndex];
        size_t            initialSizeU0    = patch.getSizeU0();
        size_t            initialSizeV0    = patch.getSizeV0();
        size_t            updatedSizeU0    = cPatchUnion.getSizeU0();
        size_t            updatedSizeV0    = cPatchUnion.getSizeV0();
        auto&             initialOccupancy = patch.getOccupancy();
        std::vector<bool> updatedOccupancy( updatedSizeU0 * updatedSizeV0, false );
        for ( size_t v0 = 0; v0 < initialSizeV0; ++v0 ) {
          for ( size_t u0 = 0; u0 < initialSizeU0; ++u0 ) {
            size_t initialIndex = v0 * initialSizeU0 + u0;
            size_t updatedIndex = v0 * updatedSizeU0 + u0;
            if ( initialOccupancy[initialIndex] && !updatedOccupancy[updatedIndex] )
              updatedOccupancy[updatedIndex] = true;
          }
        }
        curGPAPatchData.sizeU0    = updatedSizeU0;
        curGPAPatchData.sizeV0    = updatedSizeV0;
        curGPAPatchData.occupancy = updatedOccupancy;
      } else {
        curGPAPatchData.sizeU0    = patch.getSizeU0();
        curGPAPatchData.sizeV0    = patch.getSizeV0();
        curGPAPatchData.occupancy = patch.getOccupancy();
      }
    }
  }
}

void PCCEncoder::performGPAPacking( const SubContext& subContext,
                                    unionPatch&       unionPatch,
                                    PCCContext&       context,
                                    bool&             badGPAPacking,
                                    size_t            unionsHeight,
                                    int               safeguard,
                                    bool              useRefFrame ) {
  bool   exceedMinimumImageHeight = false;  // whether exceed minimunImageHeight or not;
  size_t badCondition             = 0;      // GPA bad condition count;
  for ( size_t i = subContext.first; i < subContext.second; ++i ) {
    auto& curFrameContext = context[i];
    auto& widthGPA        = curFrameContext.getCurPCCGPAFrameSize().widthGPA_;
    auto& heightGPA       = curFrameContext.getCurPCCGPAFrameSize().heightGPA_;
    auto& patches         = curFrameContext.getPatches();
    if ( patches.empty() ) { return; }
    int   preIndex   = i > 0 ? i - 1 : 0;
    auto& prePatches = context[preIndex].getPatches();

    size_t occupancySizeU = params_.minimumImageWidth_ / params_.occupancyResolution_;
    size_t occupancySizeV = unionsHeight / params_.occupancyResolution_;
    for ( auto& patch : patches ) {
      occupancySizeU = ( std::max )( occupancySizeU, patch.getCurGPAPatchData().sizeU0 + 1 );
    }
    widthGPA  = occupancySizeU * params_.occupancyResolution_;
    heightGPA = occupancySizeV * params_.occupancyResolution_;
    size_t maxOccupancyRow{0};

    vector<int> orientation_vertical   = {PatchOrientation::DEFAULT,
                                        PatchOrientation::SWAP};  // favoring vertical orientation
    vector<int> orientation_horizontal = {
        PatchOrientation::SWAP,
        PatchOrientation::DEFAULT};  // favoring horizontal orientations (that should be rotated)
    std::vector<bool> occupancyMap;
    occupancyMap.resize( occupancySizeU * occupancySizeV, false );
    // !!!packing global matched patch;
    for ( auto& patch : patches ) {
      GPAPatchData& curGPAPatchData = patch.getCurGPAPatchData();
      if ( curGPAPatchData.isGlobalPatch ) {
        assert( curGPAPatchData.sizeU0 <= occupancySizeU );
        assert( curGPAPatchData.sizeV0 <= occupancySizeV );
        const size_t trackIndex = curGPAPatchData.globalPatchIndex;
        assert( unionPatch.count( trackIndex ) != 0 );
        curGPAPatchData.u0               = unionPatch[trackIndex].getU0();
        curGPAPatchData.v0               = unionPatch[trackIndex].getV0();
        curGPAPatchData.patchOrientation = unionPatch[trackIndex].getPatchOrientation();
        if ( printDetailedInfo ) {
          std::cout << "Orientation:" << curGPAPatchData.patchOrientation << " for GPA patch in the same position ("
                    << curGPAPatchData.u0 << "," << curGPAPatchData.v0 << ")" << std::endl;
        }
        for ( size_t v0 = 0; v0 < curGPAPatchData.sizeV0; ++v0 ) {
          for ( size_t u0 = 0; u0 < curGPAPatchData.sizeU0; ++u0 ) {
            int coord           = patch.patchBlock2CanvasBlockForGPA( u0, v0, occupancySizeU, occupancySizeV );
            occupancyMap[coord] = occupancyMap[coord] || curGPAPatchData.occupancy[v0 * curGPAPatchData.sizeU0 + u0];
          }
        }
        if ( !( curGPAPatchData.isPatchDimensionSwitched() ) ) {
          heightGPA       = ( std::max )( heightGPA,
                                    ( curGPAPatchData.v0 + curGPAPatchData.sizeV0 ) * patch.getOccupancyResolution() );
          widthGPA        = ( std::max )( widthGPA,
                                   ( curGPAPatchData.u0 + curGPAPatchData.sizeU0 ) * patch.getOccupancyResolution() );
          maxOccupancyRow = ( std::max )( maxOccupancyRow, ( curGPAPatchData.v0 + curGPAPatchData.sizeV0 ) );
        } else {
          heightGPA       = ( std::max )( heightGPA,
                                    ( curGPAPatchData.v0 + curGPAPatchData.sizeU0 ) * patch.getOccupancyResolution() );
          widthGPA        = ( std::max )( widthGPA,
                                   ( curGPAPatchData.u0 + curGPAPatchData.sizeV0 ) * patch.getOccupancyResolution() );
          maxOccupancyRow = ( std::max )( maxOccupancyRow, ( curGPAPatchData.v0 + curGPAPatchData.sizeU0 ) );
        }
      }
    }
    // !!!packing non-global matched patch;
    int icount = -1;
    for ( auto& patch : patches ) {
      icount++;
      GPAPatchData& curGPAPatchData = patch.getCurGPAPatchData();
      if ( curGPAPatchData.isGlobalPatch ) { continue; }

      // not use reference frame only if the first frame or useRefFrame is disabled.
      if ( ( i == 0 ) || ( ( i == subContext.first ) && ( !useRefFrame ) ) ) {  // not use ref.
        packingWithoutRefForFirstFrameNoglobalPatch( patch, i, icount, occupancySizeU, occupancySizeV, safeguard,
                                                     occupancyMap, heightGPA, widthGPA, maxOccupancyRow );
      } else {
        // PCCPatch prePatch = prePatches[patch.getBestMatchIdx()];
        packingWithRefForFirstFrameNoglobalPatch( patch, prePatches, subContext.first, i, icount, occupancySizeU,
                                                  occupancySizeV, safeguard, occupancyMap, heightGPA, widthGPA,
                                                  maxOccupancyRow );
      }
    }

    if ( curFrameContext.getMissedPointsPatch().size() > 0 && !curFrameContext.getUseMissedPointsSeparateVideo() ) {
      packMissedPointsPatch( curFrameContext, occupancyMap, widthGPA, heightGPA, occupancySizeU, occupancySizeV,
                             maxOccupancyRow );
      curFrameContext.getMissedPointsPatch().tempV0_ = curFrameContext.getMissedPointsPatch().v0_;
    } else {
      if ( printDetailedInfo ) { printMap( occupancyMap, occupancySizeU, occupancySizeV ); }
    }

    // determination......;
    if ( heightGPA > params_.minimumImageHeight_ ) {
      exceedMinimumImageHeight = true;
      break;
    }
    double validHeightRatio = ( double( heightGPA ) ) / ( double( curFrameContext.getHeight() ) );
    if ( validHeightRatio >= BAD_HEIGHT_THRESHOLD ) { badCondition++; }
  }

  if ( exceedMinimumImageHeight || badCondition > BAD_CONDITION_THRESHOLD ) { badGPAPacking = true; }
}

void PCCEncoder::packingWithoutRefForFirstFrameNoglobalPatch( PCCPatch&          patch,
                                                              size_t             i,
                                                              size_t             icount,
                                                              size_t&            occupancySizeU,
                                                              size_t&            occupancySizeV,
                                                              const size_t       safeguard,
                                                              std::vector<bool>& occupancyMap,
                                                              size_t&            heightGPA,
                                                              size_t&            widthGPA,
                                                              size_t             maxOccupancyRow ) {
  vector<int> orientation_vertical   = {PatchOrientation::DEFAULT,
                                      PatchOrientation::SWAP};  // favoring vertical orientation
  vector<int> orientation_horizontal = {
      PatchOrientation::SWAP, PatchOrientation::DEFAULT};  // favoring horizontal orientations (that should be rotated)

  GPAPatchData& preGPAPatchData = patch.getCurGPAPatchData();

  assert( preGPAPatchData.sizeU0 <= occupancySizeU );
  assert( preGPAPatchData.sizeV0 <= occupancySizeV );
  bool  locationFound = false;
  auto& occupancy     = patch.getOccupancy();
  while ( !locationFound ) {
    for ( size_t v = 0; v < occupancySizeV && !locationFound; ++v ) {
      for ( size_t u = 0; u < occupancySizeU && !locationFound; ++u ) {
        preGPAPatchData.u0 = u;
        preGPAPatchData.v0 = v;
        if ( params_.packingStrategy_ == 0 ) {
          if ( patch.checkFitPatchCanvasForGPA( occupancyMap, occupancySizeU, occupancySizeV, safeguard ) ) {
            locationFound = true;
            if ( printDetailedInfo ) {
              std::cout << "Orientation " << preGPAPatchData.patchOrientation << " selected for Patch: [" << icount
                        << "] in the position (" << u << "," << v << ")" << std::endl;
            }
          }
        } else {  // try several orientation.
          for ( size_t orientationIdx = 0; orientationIdx < pow( 2, params_.packingStrategy_ ) && !locationFound;
                orientationIdx++ ) {
            if ( patch.getSizeU0() > patch.getSizeV0() ) {
              preGPAPatchData.patchOrientation = orientation_horizontal[orientationIdx];
            } else {
              preGPAPatchData.patchOrientation = orientation_vertical[orientationIdx];
            }
            if ( patch.checkFitPatchCanvasForGPA( occupancyMap, occupancySizeU, occupancySizeV, safeguard ) ) {
              locationFound = true;
              if ( printDetailedInfo ) {
                std::cout << "Orientation " << preGPAPatchData.patchOrientation << "selected for Patch: [" << icount
                          << "] in the position (" << preGPAPatchData.u0 << "," << preGPAPatchData.v0 << ")"
                          << std::endl;
              }
            }
          }
        }
      }
    }
    if ( !locationFound ) {
      occupancySizeV *= 2;
      occupancyMap.resize( occupancySizeU * occupancySizeV );
      if ( printDetailedInfo ) { std::cout << "Increase occupancySizeV " << occupancySizeV << std::endl; }
    }
  }
  // update occupancy.
  for ( size_t v0 = 0; v0 < preGPAPatchData.sizeV0; ++v0 ) {
    for ( size_t u0 = 0; u0 < preGPAPatchData.sizeU0; ++u0 ) {
      int coord           = patch.patchBlock2CanvasBlockForGPA( u0, v0, occupancySizeU, occupancySizeV );
      occupancyMap[coord] = occupancyMap[coord] || occupancy[v0 * patch.getSizeU0() + u0];
    }
  }
  if ( !( preGPAPatchData.isPatchDimensionSwitched() ) ) {
    heightGPA =
        ( std::max )( heightGPA, ( preGPAPatchData.v0 + preGPAPatchData.sizeV0 ) * patch.getOccupancyResolution() );
    widthGPA =
        ( std::max )( widthGPA, ( preGPAPatchData.u0 + preGPAPatchData.sizeU0 ) * patch.getOccupancyResolution() );
    maxOccupancyRow = ( std::max )( maxOccupancyRow, ( preGPAPatchData.v0 + preGPAPatchData.sizeV0 ) );
  } else {
    heightGPA =
        ( std::max )( heightGPA, ( preGPAPatchData.v0 + preGPAPatchData.sizeU0 ) * patch.getOccupancyResolution() );
    widthGPA =
        ( std::max )( widthGPA, ( preGPAPatchData.u0 + preGPAPatchData.sizeV0 ) * patch.getOccupancyResolution() );
    maxOccupancyRow = ( std::max )( maxOccupancyRow, ( preGPAPatchData.v0 + preGPAPatchData.sizeU0 ) );
  }
}
void PCCEncoder::packingWithRefForFirstFrameNoglobalPatch( PCCPatch&                   patch,
                                                           const std::vector<PCCPatch> prePatches,
                                                           size_t                      startFrameIndex,
                                                           size_t                      i,
                                                           size_t                      icount,
                                                           size_t&                     occupancySizeU,
                                                           size_t&                     occupancySizeV,
                                                           const size_t                safeguard,
                                                           std::vector<bool>&          occupancyMap,
                                                           size_t&                     heightGPA,
                                                           size_t&                     widthGPA,
                                                           size_t                      maxOccupancyRow ) {
  vector<int> orientation_vertical   = {PatchOrientation::DEFAULT,
                                      PatchOrientation::SWAP};  // favoring vertical orientation
  vector<int> orientation_horizontal = {
      PatchOrientation::SWAP, PatchOrientation::DEFAULT};  // favoring horizontal orientations (that should be rotated)

  GPAPatchData& preGPAPatchData = patch.getCurGPAPatchData();

  assert( preGPAPatchData.sizeU0 <= occupancySizeU );
  assert( preGPAPatchData.sizeV0 <= occupancySizeV );
  bool  locationFound = false;
  auto& occupancy     = patch.getOccupancy();
  while ( !locationFound ) {
    if ( patch.getBestMatchIdx() != InvalidPatchIndex ) {
      PCCPatch prePatch = prePatches[patch.getBestMatchIdx()];
      if ( i == startFrameIndex ) {
        preGPAPatchData.patchOrientation = prePatch.getPatchOrientation();
        // try to place on the same position as the matched patch
        preGPAPatchData.u0 = prePatch.getU0();
        preGPAPatchData.v0 = prePatch.getV0();
      } else {
        preGPAPatchData.patchOrientation = prePatch.getCurGPAPatchData().patchOrientation;
        // try to place on the same position as the matched patch
        preGPAPatchData.u0 = prePatch.getCurGPAPatchData().u0;
        preGPAPatchData.v0 = prePatch.getCurGPAPatchData().v0;
      }
      if ( preGPAPatchData.patchOrientation == -1 ) { assert( preGPAPatchData.patchOrientation != -1 ); }

      if ( patch.checkFitPatchCanvasForGPA( occupancyMap, occupancySizeU, occupancySizeV, safeguard ) ) {
        locationFound = true;
        if ( printDetailedInfo ) {
          std::cout << "Maintained TempGPA.orientation " << preGPAPatchData.patchOrientation << " for patch[" << icount
                    << "] in the same position (" << preGPAPatchData.u0 << "," << preGPAPatchData.v0 << ")"
                    << std::endl;
        }
      }

      // if the patch couldn't fit, try to fit the patch in the top left position
      for ( int v = 0; v <= occupancySizeV && !locationFound; ++v ) {
        for ( int u = 0; u <= occupancySizeU && !locationFound; ++u ) {
          preGPAPatchData.u0 = u;
          preGPAPatchData.v0 = v;
          if ( patch.checkFitPatchCanvasForGPA( occupancyMap, occupancySizeU, occupancySizeV,
                                                safeguard ) ) {  // !!! function overload for GPA;
            locationFound = true;
            if ( printDetailedInfo ) {
              std::cout << "Maintained TempGPA.orientation " << preGPAPatchData.patchOrientation
                        << " for unmatched patch[" << icount << "] in the position (" << preGPAPatchData.u0 << ","
                        << preGPAPatchData.v0 << ")" << std::endl;
            }
          }
        }
      }
    } else {
      // best effort
      for ( size_t v = 0; v < occupancySizeV && !locationFound; ++v ) {
        for ( size_t u = 0; u < occupancySizeU && !locationFound; ++u ) {
          preGPAPatchData.u0 = u;
          preGPAPatchData.v0 = v;
          for ( size_t orientationIdx = 0; orientationIdx < pow( 2, params_.packingStrategy_ ) && !locationFound;
                orientationIdx++ ) {
            if ( patch.getSizeU0() > patch.getSizeV0() ) {
              preGPAPatchData.patchOrientation = orientation_horizontal[orientationIdx];
            } else {
              preGPAPatchData.patchOrientation = orientation_vertical[orientationIdx];
            }
            if ( patch.checkFitPatchCanvasForGPA( occupancyMap, occupancySizeU, occupancySizeV, safeguard ) ) {
              locationFound = true;
              if ( printDetailedInfo ) {
                std::cout << "Maintained TempGPA.orientation " << preGPAPatchData.patchOrientation
                          << " for unmatched patch[" << icount << "] in the position (" << preGPAPatchData.u0 << ","
                          << preGPAPatchData.v0 << ")" << std::endl;
              }
            }
          }
        }
      }
    }
    if ( !locationFound ) {
      occupancySizeV *= 2;
      occupancyMap.resize( occupancySizeU * occupancySizeV );
      if ( printDetailedInfo ) { std::cout << "Increase occupancySizeV " << occupancySizeV << std::endl; }
    }
  }
  for ( size_t v0 = 0; v0 < preGPAPatchData.sizeV0; ++v0 ) {
    for ( size_t u0 = 0; u0 < preGPAPatchData.sizeU0; ++u0 ) {
      int coord           = patch.patchBlock2CanvasBlockForGPA( u0, v0, occupancySizeU, occupancySizeV );
      occupancyMap[coord] = occupancyMap[coord] || occupancy[v0 * preGPAPatchData.sizeU0 + u0];
    }
  }
  if ( !( preGPAPatchData.isPatchDimensionSwitched() ) ) {
    heightGPA =
        ( std::max )( heightGPA, ( preGPAPatchData.v0 + preGPAPatchData.sizeV0 ) * patch.getOccupancyResolution() );
    widthGPA =
        ( std::max )( widthGPA, ( preGPAPatchData.u0 + preGPAPatchData.sizeU0 ) * patch.getOccupancyResolution() );
    maxOccupancyRow = ( std::max )( maxOccupancyRow, ( preGPAPatchData.v0 + preGPAPatchData.sizeV0 ) );
  } else {
    heightGPA =
        ( std::max )( heightGPA, ( preGPAPatchData.v0 + preGPAPatchData.sizeU0 ) * patch.getOccupancyResolution() );
    widthGPA =
        ( std::max )( widthGPA, ( preGPAPatchData.u0 + preGPAPatchData.sizeV0 ) * patch.getOccupancyResolution() );
    maxOccupancyRow = ( std::max )( maxOccupancyRow, ( preGPAPatchData.v0 + preGPAPatchData.sizeU0 ) );
  }
}

/* THIS SECTION ADDS PATCH RELATED CALCULATIONS AND GENERATED PFDS */

void PCCEncoder::setGeometryFrameParameterSet( PCCMetadata& metadata, GeometryFrameParameterSet& gfps ) {
  auto& gfp                  = gfps.getGeometryFrameParams();
  auto& metadataEnabingFlags = metadata.getMetadataEnabledFlags();
  gfps.setGeometryParamsEnabledFlag( metadataEnabingFlags.getMetadataEnabled() );

  gfps.setGeometryFrameParameterSetId( 0 );
  gfps.setPatchSequenceParameterSetId( 0 );
  gfps.setGeometryParamsEnabledFlag( metadataEnabingFlags.getMetadataEnabled() );
  gfps.setGeometryPatchParamsEnabledFlag( true );
  gfps.setOverrideGeometryParamsFlag( false );
  gfps.setOverrideGeometryPatchParamsFlag( false );

  if ( gfps.getGeometryParamsEnabledFlag() ) {
    // Scale
    gfps.setGeometryPatchScaleParamsEnabledFlag( metadataEnabingFlags.getScaleEnabled() );
    if ( gfps.getGeometryPatchScaleParamsEnabledFlag() ) {
      gfp.setGeometryScaleParamsPresentFlag( metadataEnabingFlags.getScaleEnabled() );
      if ( gfp.getGeometryScaleParamsPresentFlag() ) {
        gfp.setGeometryScaleOnAxis( 0, metadata.getScale()[0] );
        gfp.setGeometryScaleOnAxis( 1, metadata.getScale()[1] );
        gfp.setGeometryScaleOnAxis( 2, metadata.getScale()[2] );
      }
    }

    // Offset
    gfps.setGeometryPatchOffsetParamsEnabledFlag( metadataEnabingFlags.getOffsetEnabled() );
    if ( gfps.getGeometryPatchOffsetParamsEnabledFlag() ) {
      gfp.setGeometryOffsetParamsPresentFlag( metadataEnabingFlags.getOffsetEnabled() );
      if ( gfp.getGeometryOffsetParamsPresentFlag() ) {
        gfp.setGeometryOffsetOnAxis( 0, metadata.getOffset()[0] );
        gfp.setGeometryOffsetOnAxis( 1, metadata.getOffset()[1] );
        gfp.setGeometryOffsetOnAxis( 2, metadata.getOffset()[2] );
      }
    }

    // Rotation
    gfps.setGeometryPatchRotationParamsEnabledFlag( metadataEnabingFlags.getRotationEnabled() );
    if ( gfps.getGeometryPatchRotationParamsEnabledFlag() ) {
      gfp.setGeometryRotationParamsPresentFlag( metadataEnabingFlags.getRotationEnabled() );
      if ( gfp.getGeometryRotationParamsPresentFlag() ) {
        gfp.setGeometryRotationOnAxis( 0, metadata.getRotation()[0] );
        gfp.setGeometryRotationOnAxis( 1, metadata.getRotation()[1] );
        gfp.setGeometryRotationOnAxis( 2, metadata.getRotation()[2] );
      }
    }

    // Point size
    gfps.setGeometryPatchPointSizeInfoEnabledFlag( metadataEnabingFlags.getPointSizeEnabled() );
    if ( gfps.getGeometryPatchPointSizeInfoEnabledFlag() ) {
      gfp.setGeometryPointShapeInfoPresentFlag( metadata.getPointSizePresent() );
      if ( gfp.getGeometryPointShapeInfoPresentFlag() ) { gfp.setGeometryPointSizeInfo( metadata.getPointSize() ); }
    }

    // Point shape
    gfps.setGeometryPatchPointShapeInfoEnabledFlag( metadataEnabingFlags.getPointShapeEnabled() );
    if ( gfps.getGeometryPatchPointShapeInfoEnabledFlag() ) {
      gfp.setGeometryPointShapeInfoPresentFlag( metadata.getPointShapePresent() );
      if ( gfp.getGeometryPointShapeInfoPresentFlag() ) { gfp.setGeometryPointSizeInfo( metadata.getPointShape() ); }
    }
  }
}

void PCCEncoder::setGeometryPatchParameterSet( PCCMetadata& metadata, GeometryPatchParameterSet& gpps ) {
  auto& gpp                  = gpps.getGeometryPatchParams();
  auto& metadataEnabingFlags = metadata.getMetadataEnabledFlags();

  gpps.setGeometryPatchParamsPresentFlag( metadataEnabingFlags.getMetadataEnabled() );
  if ( gpps.getGeometryPatchParamsPresentFlag() ) {
    // Scale
    gpp.setGeometryPatchScaleParamsPresentFlag( metadataEnabingFlags.getScaleEnabled() );
    if ( gpp.getGeometryPatchScaleParamsPresentFlag() ) {
      gpp.setGeometryPatchScaleOnAxis( 0, metadata.getScale()[0] );
      gpp.setGeometryPatchScaleOnAxis( 1, metadata.getScale()[1] );
      gpp.setGeometryPatchScaleOnAxis( 2, metadata.getScale()[2] );
    }

    // Offset
    gpp.setGeometryPatchOffsetParamsPresentFlag( metadataEnabingFlags.getOffsetEnabled() );
    if ( gpp.getGeometryPatchOffsetParamsPresentFlag() ) {
      gpp.setGeometryPatchOffsetOnAxis( 0, metadata.getOffset()[0] );
      gpp.setGeometryPatchOffsetOnAxis( 1, metadata.getOffset()[1] );
      gpp.setGeometryPatchOffsetOnAxis( 2, metadata.getOffset()[2] );
    }

    // Rotation
    gpp.setGeometryPatchRotationParamsPresentFlag( metadataEnabingFlags.getRotationEnabled() );
    if ( gpp.getGeometryPatchRotationParamsPresentFlag() ) {
      gpp.setGeometryPatchRotationOnAxis( 0, metadata.getRotation()[0] );
      gpp.setGeometryPatchRotationOnAxis( 1, metadata.getRotation()[1] );
      gpp.setGeometryPatchRotationOnAxis( 2, metadata.getRotation()[2] );
    }

    // Point size
    gpp.setGeometryPatchPointSizeInfoPresentFlag( metadataEnabingFlags.getPointSizeEnabled() );
    if ( gpp.getGeometryPatchPointSizeInfoPresentFlag() ) {
      gpp.setGeometryPatchPointSizeInfo( metadata.getPointSize() );
    }

    // Point shape
    gpp.setGeometryPatchPointShapeInfoPresentFlag( metadataEnabingFlags.getPointShapeEnabled() );
    if ( gpp.getGeometryPatchPointShapeInfoPresentFlag() ) {
      gpp.setGeometryPatchPointShapeInfo( metadata.getPointShape() );
    }
#ifdef CE210_MAXDEPTH_EVALUATION
    if ( metadataEnabingFlags.getMaxDepthEnabled() ) {
      currentDD = metadata.getQMaxDepthInPatch();
      // TODO: gfp.setGeometryMaxDeltaDepth(currentDD);
    }
#endif
  }
  // auto& lowerLevelMetadataEnabledFlags = metadata.getLowerLevelMetadataEnabledFlags();
  // // gfp.set...(lowerLevelMetadataEnabledFlags.getMetadataEnabled())
  // if ( lowerLevelMetadataEnabledFlags.getMetadataEnabled() ) {
  //   /* TODO: I could not locate the corespondence for these parameters*/
  //   // lowerLevelMetadataEnabledFlags.getScaleEnabled()
  //   // lowerLevelMetadataEnabledFlags.getOffsetEnabled()
  //   // lowerLevelMetadataEnabledFlags.getRotationEnabled()
  //   // lowerLevelMetadataEnabledFlags.getPointSizeEnabled()
  //   // lowerLevelMetadataEnabledFlags.getPointShapeEnabled()
  // }
}

void PCCEncoder::setPointLocalReconstruction( PCCFrameContext&          frame,
                                              const PCCPatch&           patch,
                                              PointLocalReconstruction& plr,
                                              size_t                    occupancyPackingBlockSize ) {
  auto&  blockToPatch       = frame.getBlockToPatch();
  size_t blockToPatchWidth  = frame.getWidth() / occupancyPackingBlockSize;
  size_t blockToPatchHeight = frame.getHeight() / occupancyPackingBlockSize;
  auto&  interpolateMap     = frame.getInterpolate();
  auto&  fillingMap         = frame.getFilling();
  auto&  minD1Map           = frame.getMinD1();
  auto&  neighborMap        = frame.getNeighbor();

  plr.setBlockToPatchMapWidth( blockToPatchWidth );
  plr.setBlockToPatchMapHeight( blockToPatchHeight );

  for ( size_t v0 = 0; v0 < patch.getSizeV0(); ++v0 ) {
    for ( size_t u0 = 0; u0 < patch.getSizeU0(); ++u0 ) {
      int pos = patch.patchBlock2CanvasBlock( ( u0 ), ( v0 ), blockToPatchWidth, blockToPatchHeight );
      plr.setBlockToPatchMap( static_cast<uint32_t>( blockToPatch[pos] > 0 ), u0, v0 );
      if ( blockToPatch[pos] > 0 ) {
        plr.setModeInterpolateFlag( bool( interpolateMap[pos] ), u0, v0 );
        if ( interpolateMap[pos] > 0 ) {
          uint32_t code = static_cast<uint32_t>( int( neighborMap[pos] ) - 1 );
          plr.setModeNeighbourMinus1( code, u0, v0 );
        }
        uint8_t code = static_cast<uint8_t>( minD1Map[pos] ) - 1;
        plr.setModeMinimumDepthMinus1( code, u0, v0 );
        if ( minD1Map[pos] > 1 || interpolateMap[pos] > 0 ) {
          plr.setModeFillingFlag( static_cast<uint32_t>( fillingMap[pos] ), u0, v0 );
        }
      }
    }
  }
}

void PCCEncoder::createPatchFrameDataStructure( PCCContext& context ) {
  TRACE_CODEC( "createPatchFrameDataStructure GOP start \n" );
  auto& sps  = context.getSps();
  auto& psdu = context.getPatchSequenceDataUnit();
  psdu.setFrameCount( context.getFrames().size() );
  TRACE_CODEC( "getFrameCount %u \n", psdu.getFrameCount() );
  psdu.addPatchSequenceUnitPayload( PSD_SPS, 0 );
  auto& psps = psdu.getPatchSequenceParameterSet( 0 );
  RefListStruct refList;
  refList.allocate();
  refList.setNumRefEntries( 1 );
  refList.addAbsDeltaPfocSt( 1 );
  psps.addRefListStruct( refList );
  psdu.addPatchSequenceUnitPayload( PSD_FPS, 0 );
  psdu.addPatchSequenceUnitPayload( PSD_GFPS, 0 );
  setGeometryFrameParameterSet( context.getGOFLevelMetadata(), psdu.getGeometryFrameParameterSet( 0 ) );
  for ( size_t i = 0; i < sps.getAttributeCount(); i++ ) {
    AttributeFrameParameterSet afps;
    psdu.addAttributeFrameParameterSet( afps, 0 );
  }
  psdu.addPatchSequenceUnitPayload( PSD_GPPS, 0 );
  psdu.addPatchSequenceUnitPayload( PSD_APPS, 0 );
  for ( size_t i = 0; i < sps.getAttributeCount(); i++ ) {
    AttributePatchParameterSet apps;
    psdu.addAttributePatchParameterSet( apps, 0 );
  }
  for ( size_t i = 0; i < psdu.getFrameCount(); i++ ) { psdu.addPatchSequenceUnitPayload( PSD_PFLU, i ); }
  psdu.printPatchSequenceUnitPayload();

  PCCFrameContext& refFrame = context.getFrame( 0 );
  for ( size_t i = 0; i < psdu.getFrameCount(); i++ ) {
    PCCFrameContext& frame = context.getFrame( i );
    createPatchFrameDataStructure( context, frame, refFrame, i );
    refFrame = frame;
  }
}

void PCCEncoder::createPatchFrameDataStructure( PCCContext&      context,
                                                PCCFrameContext& frame,
                                                PCCFrameContext& refFrame,
                                                size_t           frameIndex ) {
  TRACE_CODEC( "createPatchFrameDataStructure Frame %lu \n", frame.getIndex() );
  auto&         patches                = frame.getPatches();
  auto&         sps                    = context.getSps();
  auto&         psdu                   = context.getPatchSequenceDataUnit();
  auto&         ops                    = sps.getOccupancyParameterSet();
  auto&         pflu                   = psdu.getPatchFrameLayerUnit( frameIndex );
  auto&         pfh                    = pflu.getPatchFrameHeader();
  auto&         pfdu                   = pflu.getPatchFrameDataUnit();
  auto&         pfps                   = psdu.getPatchFrameParameterSet( 0 );
  const size_t  minLevel               = sps.getMinLevel();
  const uint8_t maxBitCountForMinDepth = uint8_t( 10 - gbitCountSize[minLevel] );
  const uint8_t maxBitCountForMaxDepth = uint8_t( 9 - gbitCountSize[minLevel] );  // 20190129
  uint8_t       id                     = pfh.getPatchFrameParameterSetId();
  int64_t       prevSizeU0 = 0, prevSizeV0 = 0, predIndex = 0;
  auto&         refPatches = refFrame.getPatches();
  pfdu.setPatchCount( patches.size() );
  pflu.setFrameIndex( frameIndex );
  pfh.setPatchFrameOrderCntLsb( frameIndex );
  pfps.setPatchOrientationPresentFlag( params_.packingStrategy_ > 0 );
  if ( ( frameIndex == 0 ) || ( !sps.getPatchInterPredictionEnabledFlag() ) ) {
    pfh.setType( I_PATCH_FRAME );
    frame.getNumMatchedPatches() = 0;
  } else {
    pfh.setType( P_PATCH_FRAME );
  }

  TRACE_CODEC( "patches size      = %lu \n", patches.size() );
  TRACE_CODEC( "NumMatchedPatches = %lu \n", frame.getNumMatchedPatches() );
  TRACE_CODEC( "OccupancyPackingBlockSize           = %d \n", ops.getOccupancyPackingBlockSize() );
  TRACE_CODEC( "PatchSequenceOrientationEnabledFlag = %d \n", sps.getPatchSequenceOrientationEnabledFlag() );
  TRACE_CODEC( "PatchOrientationPresentFlag         = %d \n", pfps.getPatchOrientationPresentFlag() );
  TRACE_CODEC( "PatchInterPredictionEnabledFlag     = %d \n", sps.getPatchInterPredictionEnabledFlag() );

  // Inter patches
  for ( size_t patchIndex = 0; patchIndex < frame.getNumMatchedPatches(); patchIndex++ ) {
    const auto& patch    = patches[patchIndex];
    const auto& refPatch = refPatches[patch.getBestMatchIdx()];
    auto&       pid      = pfdu.addPatchInformationData();
    auto&       dpdu     = pid.getDeltaPatchDataUnit();
    pfdu.addPatchMode( P_INTER );
    TRACE_CODEC( "patch %lu / %lu \n", patchIndex, patches.size() );
    pfdu.setPatchMode( patchIndex, P_INTER );
    dpdu.setDeltaPatchIdx( patch.getBestMatchIdx() - predIndex );
    dpdu.set2DDeltaShiftU( patch.getU0() - refPatch.getU0() );
    dpdu.set2DDeltaShiftV( patch.getV0() - refPatch.getV0() );
    dpdu.set3DDeltaShiftTangentAxis( patch.getU1() - refPatch.getU1() );
    dpdu.set3DDeltaShiftBiTangentAxis( patch.getV1() - refPatch.getV1() );
    dpdu.set2DDeltaSizeU( patch.getSizeU0() - refPatch.getSizeU0() );
    dpdu.set2DDeltaSizeV( patch.getSizeV0() - refPatch.getSizeV0() );
    dpdu.setProjectionMode( patch.getProjectionMode() );
    dpdu.setLod( patch.getLod() );
    if ( patch.getProjectionMode() == 0 ) {
      dpdu.set3DDeltaShiftNormalAxis( ( patch.getD1() / minLevel ) - ( refPatch.getD1() / minLevel ) );
    } else {
      dpdu.set3DDeltaShiftNormalAxis( ( 1024 - patch.getD1() ) / minLevel - ( 1024 - refPatch.getD1() ) / minLevel );
    }
    TRACE_CODEC( "dpdu.getDeltaPatchIdx() = %d \n", dpdu.getDeltaPatchIdx() );
    TRACE_CODEC( "patch.getBestMatchIdx() = %d \n", patch.getBestMatchIdx() );
    TRACE_CODEC( "predIndex               = %d \n", predIndex );
    TRACE_CODEC( "deltaIndex              = %d \n", dpdu.getDeltaPatchIdx() );
    TRACE_CODEC( "DeltaIdx = %d ShiftUV = %ld %ld ShiftAxis = %ld %ld %ld Size = %ld %ld \n", dpdu.getDeltaPatchIdx(),
                 dpdu.get2DDeltaShiftU(), dpdu.get2DDeltaShiftV(), dpdu.get3DDeltaShiftTangentAxis(),
                 dpdu.get3DDeltaShiftBiTangentAxis(), dpdu.get3DDeltaShiftNormalAxis(), dpdu.get2DDeltaSizeU(),
                 dpdu.get2DDeltaSizeV() );

    size_t        quantDD   = patch.getSizeD() == 0 ? 0 : ( ( patch.getSizeD() - 1 ) / minLevel + 1 );
    size_t        prevQDD   = patch.getSizeD() == 0 ? 0 : ( ( refPatch.getSizeD() - 1 ) / minLevel + 1 );
    const int64_t delta_dd  = static_cast<int64_t>( quantDD - prevQDD );
    auto&         patchTemp = patches[patchIndex];
    PCCMetadata&  metadata  = patchTemp.getPatchLevelMetadata();
    metadata.setMetadataPresent( true );
    metadata.setbitCountQDepth( 0 );
    setGeometryFrameParameterSet( metadata, psdu.getGeometryFrameParameterSet( 0 ) );
    prevSizeU0 = patch.getSizeU0();
    prevSizeV0 = patch.getSizeV0();
    predIndex += dpdu.getDeltaPatchIdx() + 1;
    TRACE_CODEC( "patch Inter UV0 %4lu %4lu UV1 %4lu %4lu D1=%4lu S=%4lu %4lu P=%lu O=%lu A=%u%u%u \n", patch.getU0(),
                 patch.getV0(), patch.getU1(), patch.getV1(), patch.getD1(), patch.getSizeU0(), patch.getSizeV0(),
                 patch.getProjectionMode(), patch.getPatchOrientation(), patch.getNormalAxis(), patch.getTangentAxis(),
                 patch.getBitangentAxis() );
  }

  // Intra patches
  for ( size_t patchIndex = frame.getNumMatchedPatches(); patchIndex < patches.size(); patchIndex++ ) {
    const auto& patch = patches[patchIndex];
    auto&       pid   = pfdu.addPatchInformationData();
    TRACE_CODEC( "patch %lu / %lu: Intra \n", patchIndex, patches.size() );
    pfdu.addPatchMode( ( ( frameIndex == 0 ) || ( !sps.getPatchInterPredictionEnabledFlag() ) ) ? (uint8_t)I_INTRA
                                                                                                : (uint8_t)P_INTRA );
    auto& pdu = pid.getPatchDataUnit();
    pdu.set2DShiftU( patch.getU0() );
    pdu.set2DShiftV( patch.getV0() );
    pdu.setProjectionMode( patch.getProjectionMode() );
    pdu.setLod( patch.getLod() );
    pdu.set3DShiftTangentAxis( patch.getU1() );
    pdu.set3DShiftBiTangentAxis( patch.getV1() );
    pdu.setNormalAxis( static_cast<pcc::PCCAxis3>( patch.getNormalAxis() ) );
    pdu.set2DDeltaSizeU( patch.getSizeU0() - prevSizeU0 );
    pdu.set2DDeltaSizeV( patch.getSizeV0() - prevSizeV0 );
    pdu.setOrientationSwapFlag( pfps.getPatchOrientationPresentFlag() &&
                                patch.getPatchOrientation() != PatchOrientation::DEFAULT );
    if ( pdu.getProjectionMode() == 0 ) {
      pdu.set3DShiftNormalAxis( patch.getD1() / minLevel );
    } else {
      pdu.set3DShiftNormalAxis( ( 1024 - patch.getD1() ) / minLevel );
    }
    prevSizeU0 = patch.getSizeU0();
    prevSizeV0 = patch.getSizeV0();
    TRACE_CODEC( "PatchOrientationPresentFlag = %d \n", pfps.getPatchOrientationPresentFlag() );
    TRACE_CODEC( "minLevel                      = %lu \n", minLevel );
    TRACE_CODEC( "patch.getProjectionMode()     = %lu \n", patch.getProjectionMode() );
    TRACE_CODEC( "pdu.get3DShiftTangentAxis()   = %lu \n", pdu.get3DShiftTangentAxis() );
    TRACE_CODEC( "pdu.get3DShiftBiTangentAxis() = %lu \n", pdu.get3DShiftBiTangentAxis() );
    TRACE_CODEC( "pdu.get3DShiftNormalAxis()    = %lu \n", pdu.get3DShiftNormalAxis() );
    TRACE_CODEC( "pdu.get2DDeltaSizeU()         = %ld \n", pdu.get2DDeltaSizeU() );
    TRACE_CODEC( "pdu.get2DDeltaSizeV()         = %ld \n", pdu.get2DDeltaSizeV() );
    TRACE_CODEC( "pdu.getNormalAxis()           = %lu \n", pdu.getNormalAxis() );
    TRACE_CODEC( "patch.getNormalAxis()         = %lu \n", patch.getNormalAxis() );
    TRACE_CODEC( "patch UV0 %4lu %4lu UV1 %4lu %4lu D1=%4lu S=%4lu %4lu P=%lu O=%lu A=%u%u%u \n", patch.getU0(),
                 patch.getV0(), patch.getU1(), patch.getV1(), patch.getD1(), patch.getSizeU0(), patch.getSizeV0(),
                 patch.getProjectionMode(), patch.getPatchOrientation(), patch.getNormalAxis(), patch.getTangentAxis(),
                 patch.getBitangentAxis() );
    size_t       quantDD   = patch.getSizeD() == 0 ? 0 : ( ( patch.getSizeD() - 1 ) / minLevel + 1 );
    auto&        patchTemp = patches[patchIndex];
    PCCMetadata& metadata  = patchTemp.getPatchLevelMetadata();
    metadata.setbitCountQDepth( 0 );  // bitCountDD );
#ifdef CE210_MAXDEPTH_EVALUATION
    metadata.setQMaxDepthInPatch( int64_t( quantDD ) );
#endif
    metadata.setIndex( patchIndex );
    metadata.setMetadataType( METADATA_PATCH );
    metadata.setMetadataPresent( true );
    setGeometryPatchParameterSet( metadata, psdu.getGeometryPatchParameterSet( 0 ) );
    if ( sps.getLayerAbsoluteCodingEnabledFlag( 1 ) && !sps.getMultipleLayerStreamsPresentFlag() ) {
      setPointLocalReconstruction( frame, patch, pfdu.getPointLocalReconstruction(),
                                   ops.getOccupancyPackingBlockSize() );
    }
  }

  // PCM patch
  if ( ( sps.getLosslessGeo() || params_.lossyMissedPointsPatch_ ) && !sps.getPcmSeparateVideoPresentFlag() ) {
    auto& pid               = pfdu.addPatchInformationData();
    auto& ppdu              = pid.getPCMPatchDataUnit();
    auto& missedPointsPatch = frame.getMissedPointsPatch();
    TRACE_CODEC( "patch %lu / %lu: PCM \n", patches.size(), patches.size() );
    ppdu.set2DShiftU( missedPointsPatch.u0_ );
    ppdu.set2DShiftV( missedPointsPatch.v0_ );
    ppdu.set2DDeltaSizeU( missedPointsPatch.sizeU0_ );
    ppdu.set2DDeltaSizeV( missedPointsPatch.sizeV0_ );
    ppdu.setPatchInPcmVideoFlag( sps.getPcmSeparateVideoPresentFlag() );
    ppdu.setPcmPoints( missedPointsPatch.numMissedPts_ );
    pfdu.addPatchMode( ( ( frameIndex == 0 ) || ( !sps.getPatchInterPredictionEnabledFlag() ) ) ? (uint8_t)I_PCM
                                                                                                : (uint8_t)P_PCM );
    TRACE_CODEC( "PCM :UV = %lu %lu  size = %lu %lu  numPoints = %lu ocmRes = %lu \n", missedPointsPatch.u0_,
                 missedPointsPatch.v0_, missedPointsPatch.sizeU0_, missedPointsPatch.sizeV0_,
                 missedPointsPatch.numMissedPts_, missedPointsPatch.occupancyResolution_ );
  }

  // pfh bitcount
  if ( ( frameIndex == 0 ) || ( !sps.getPatchInterPredictionEnabledFlag() ) ) {
    size_t maxU0 = 0, maxV0 = 0, maxU1 = 0, maxV1 = 0, maxLod = 0;
    for ( size_t patchIndex = 0; patchIndex < pfdu.getPatchCount(); ++patchIndex ) {
      const auto& patch = patches[patchIndex];
      maxU0             = ( std::max )( maxU0, patch.getU0() );
      maxV0             = ( std::max )( maxV0, patch.getV0() );
      maxU1             = ( std::max )( maxU1, patch.getU1() );
      maxV1             = ( std::max )( maxV1, patch.getV1() );
      maxLod            = ( std::max )( maxLod, patch.getLod() );
    }
    const uint8_t bitCountU0  = uint8_t( getFixedLengthCodeBitsCount( uint32_t( maxU0 + 1 ) ) );
    const uint8_t bitCountV0  = uint8_t( getFixedLengthCodeBitsCount( uint32_t( maxV0 + 1 ) ) );
    const uint8_t bitCountU1  = uint8_t( getFixedLengthCodeBitsCount( uint32_t( maxU1 + 1 ) ) );
    const uint8_t bitCountV1  = uint8_t( getFixedLengthCodeBitsCount( uint32_t( maxV1 + 1 ) ) );
    const uint8_t bitCountD1  = maxBitCountForMinDepth;
    const uint8_t bitCountDD  = maxBitCountForMaxDepth;
    const uint8_t bitCountLod = uint8_t( getFixedLengthCodeBitsCount( uint32_t( maxLod + 1 ) ) );

    pfh.setInterPredictPatch2dShiftUBitCountMinus1( bitCountU0 - 1 );
    pfh.setInterPredictPatch2dShiftVBitCountMinus1( bitCountV0 - 1 );
    pfh.setInterPredictPatch3dShiftTangentAxisBitCountMinus1( bitCountU1 - 1 );
    pfh.setInterPredictPatch3dShiftBitangentAxisBitCountMinus1( bitCountV1 - 1 );
    pfh.setInterPredictPatch3dShiftNormalAxisBitCountMinus1( maxBitCountForMinDepth - 1 );
    pfh.setInterPredictPatchLodBitCount( bitCountLod );

  } else {
    size_t        TopNmaxU0 = 0, maxU0 = 0;
    size_t        TopNmaxV0 = 0, maxV0 = 0;
    size_t        TopNmaxU1 = 0, maxU1 = 0;
    size_t        TopNmaxV1 = 0, maxV1 = 0;
    size_t        TopNmaxD1 = 0, maxD1 = 0;
    size_t        TopNmaxDD = 0, maxDD = 0, maxLod = 0;
    const size_t  minLevel               = sps.getMinLevel();
    const uint8_t maxBitCountForMinDepth = uint8_t( 10 - gbitCountSize[minLevel] );
    uint8_t       maxAllowedDepthP1      = params_.maxAllowedDepth_ + 1;
    uint8_t       bitCountDDMax          = 0;  // 255
    while ( ( maxAllowedDepthP1 ) >> bitCountDDMax ) { bitCountDDMax++; }
    const uint8_t maxBitCountForMaxDepth = uint8_t( 9 - gbitCountSize[minLevel] );

    // get the maximum u0,v0,u1,v1 and d1.
    for ( size_t patchIndex = 0; patchIndex < frame.getNumMatchedPatches(); ++patchIndex ) {
      const auto& patch = patches[patchIndex];
      TopNmaxU0         = ( std::max )( TopNmaxU0, patch.getU0() );
      TopNmaxV0         = ( std::max )( TopNmaxV0, patch.getV0() );
      TopNmaxU1         = ( std::max )( TopNmaxU1, patch.getU1() );
      TopNmaxV1         = ( std::max )( TopNmaxV1, patch.getV1() );
      size_t D1         = patch.getD1() / minLevel;
      TopNmaxD1         = ( std::max )( TopNmaxD1, D1 );
      size_t DD         = patch.getSizeD() / minLevel;
      TopNmaxDD         = ( std::max )( TopNmaxDD, DD );
    }
    for ( size_t patchIndex = frame.getNumMatchedPatches(); patchIndex < pfdu.getPatchCount(); ++patchIndex ) {
      const auto& patch = patches[patchIndex];
      maxU0             = ( std::max )( maxU0, patch.getU0() );
      maxV0             = ( std::max )( maxV0, patch.getV0() );
      maxU1             = ( std::max )( maxU1, patch.getU1() );
      maxV1             = ( std::max )( maxV1, patch.getV1() );
      size_t D1         = patch.getD1() / minLevel;
      maxD1             = ( std::max )( maxD1, D1 );
      size_t DD         = patch.getSizeD() / minLevel;
      maxDD             = ( std::max )( maxDD, DD );
      maxLod            = ( std::max )( maxLod, patch.getLod() );
    }
    uint8_t flag = 0;
    uint8_t F    = 1;  // true if the maximum value comes from the latter part.
    uint8_t A[4] = {1, 1, 1, 1};

    uint8_t bitCount[5], topBitCount[5];
    bitCount[0]               = uint8_t( getFixedLengthCodeBitsCount( uint32_t( maxU0 + 1 ) ) );
    bitCount[1]               = uint8_t( getFixedLengthCodeBitsCount( uint32_t( maxV0 + 1 ) ) );
    bitCount[2]               = uint8_t( getFixedLengthCodeBitsCount( uint32_t( maxU1 + 1 ) ) );
    bitCount[3]               = uint8_t( getFixedLengthCodeBitsCount( uint32_t( maxV1 + 1 ) ) );
    bitCount[4]               = maxBitCountForMinDepth;
    topBitCount[0]            = uint8_t( getFixedLengthCodeBitsCount( uint32_t( TopNmaxU0 + 1 ) ) );
    topBitCount[1]            = uint8_t( getFixedLengthCodeBitsCount( uint32_t( TopNmaxV0 + 1 ) ) );
    topBitCount[2]            = uint8_t( getFixedLengthCodeBitsCount( uint32_t( TopNmaxU1 + 1 ) ) );
    topBitCount[3]            = uint8_t( getFixedLengthCodeBitsCount( uint32_t( TopNmaxV1 + 1 ) ) );
    const uint8_t bitCountLod = uint8_t( getFixedLengthCodeBitsCount( uint32_t( maxLod + 1 ) ) );

    // for ( int i = 0; i < 4; i++ ) {
    //   if ( bitCount[i] <= topBitCount[i] ) {
    //     bitCount[i] = topBitCount[i];
    //     A[i]        = 0;
    //   }
    //   flag = flag << 1;
    //   flag += A[i];
    // }
    // // Generate F and A.
    // if ( flag == 0 ) { F = 0; }
    pfh.setInterPredictPatchBitCountFlag( true );
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

    pfh.setInterPredictPatch2dShiftUBitCountFlag( true );
    pfh.setInterPredictPatch2dShiftVBitCountFlag( true );
    pfh.setInterPredictPatch3dShiftTangentAxisBitCountFlag( true );
    pfh.setInterPredictPatch3dShiftBitangentAxisBitCountFlag( true );
    pfh.setInterPredictPatch3dShiftNormalAxisBitCountFlag( true );
    pfh.setInterPredictPatchLodBitCountFlag( false );
    pfh.setInterPredictPatch2dShiftUBitCountMinus1( bitCount[0] );
    pfh.setInterPredictPatch2dShiftVBitCountMinus1( bitCount[1] );
    pfh.setInterPredictPatch3dShiftTangentAxisBitCountMinus1( bitCount[2] );
    pfh.setInterPredictPatch3dShiftBitangentAxisBitCountMinus1( bitCount[3] );
    pfh.setInterPredictPatch3dShiftNormalAxisBitCountMinus1( bitCount[4] );
    pfh.setInterPredictPatchLodBitCount( bitCountLod );
  }
}
