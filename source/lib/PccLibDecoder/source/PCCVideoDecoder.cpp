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
#include "PCCVideo.h"

#include "PCCVideoDecoder.h"

#include "PCCVideoBitstream.h"
#include "PCCVideo.h"
#include "PCCVideoDecoder.h"
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCPatch.h"
#include "PCCVirtualVideoDecoder.h"
#include "PCCInternalColorConverter.h"
#ifdef USE_HDRTOOLS
#include "PCCHDRToolsLibColorConverter.h"
#else
#include "PCCHDRToolsAppColorConverter.h"
#endif

#include "PCCSHMAppVideoDecoder.h"

using namespace pcc;

PCCVideoDecoder::PCCVideoDecoder()  = default;
PCCVideoDecoder::~PCCVideoDecoder() = default;

template <typename T>
void PCCVideoDecoder::patchBasedUpsample( PCCVideo<T, 3> &video,
                                          PCCVideo<T, 3> &upsampledVideo,
                                          const size_t    newWidth,
                                          const size_t    newHeight,
                                          PCCContext     &contexts ) {
  printf( "Decoder convert : patchBasedUpsampling\n" );
  // perform color-subsampling based on patch information
  upsampledVideo.resize( video.getFrameCount() );
  for ( size_t frNum = 0; frNum < video.getFrameCount(); frNum++ ) {
    // context variable, contains the patch information
    auto &context = contexts[(int)( frNum / 2 )];  // this is only valid for coding with 2 maps, OK for now
    // low resolution image
    auto &refImage = video.getFrame( frNum );
    // image that will contain the per-patch upsampled image
    auto &destImage = upsampledVideo.getFrame( frNum );
    destImage.resize( newWidth, newHeight, video.getColorFormat() );
    // create the full resolution block to patch image
    int ratioH = newHeight / refImage.getHeight();
    int ratioW = newWidth / refImage.getWidth();

    std::vector<PCCPatch> &patches      = context.getTitleFrameContext().getPatches();
    std::vector<size_t>   &blockToPatch = context.getTitleFrameContext().getBlockToPatch();
    std::vector<size_t>    pixelToPatchHighRes;
    pixelToPatchHighRes.resize( newWidth * newHeight, 0 );
    int occRes = patches[0].getOccupancyResolution();  // is this the correct value? CHECK!!!
    for ( int patchIdx = 0; patchIdx <= patches.size(); patchIdx++ ) {
      size_t occupancyResolution;
      size_t patch_left;
      size_t patch_top;
      size_t patch_width;
      size_t patch_height;
      if ( patchIdx == 0 ) {
        // background, does not have a corresponding patch
        auto &patch         = patches[0];
        occupancyResolution = patch.getOccupancyResolution();
        patch_left          = 0;
        patch_top           = 0;
        patch_width         = newWidth;
        patch_height        = newHeight;
      } else {
        auto &patch         = patches[patchIdx - 1];
        occupancyResolution = patch.getOccupancyResolution();
        patch_left          = patch.getU0() * occupancyResolution;
        patch_top           = patch.getV0() * occupancyResolution;
        if ( !( patch.isPatchDimensionSwitched() ) ) {
          patch_width  = patch.getSizeU0() * occupancyResolution;
          patch_height = patch.getSizeV0() * occupancyResolution;
        } else {
          patch_width  = patch.getSizeV0() * occupancyResolution;
          patch_height = patch.getSizeU0() * occupancyResolution;
        }
      }
      for ( size_t i = patch_top; i < patch_top + patch_height; i++ ) {
        for ( size_t j = patch_left; j < patch_left + patch_width; j++ ) {
          if ( blockToPatch[( i / occupancyResolution ) * ( newWidth / occupancyResolution ) +
                            ( j / occupancyResolution )] == patchIdx ) {
            pixelToPatchHighRes[i * newWidth + j] = patchIdx;
          }
        }
      }
    }
    // now downsample the pixelToPatchHighRes
    std::vector<size_t> pixelToPatchLowRes;
    pixelToPatchLowRes.resize( ( newWidth / ratioW ) * ( newHeight / ratioH ), 0 );
    for ( int h = 0; h < newHeight; h++ ) {
      for ( int w = 0; w < newWidth; w++ ) {
        pixelToPatchLowRes[( h / ratioW ) * ( newWidth / ratioW ) + w / ratioW] = pixelToPatchHighRes[h * newWidth + w];
      }
    }
    // now performing up-sampling according to the patch index
    for ( size_t i = 0; i < newHeight; i++ ) {
      for ( size_t j = 0; j < newWidth; j++ ) {
        int lowResI = ( 2 * i + ratioH ) / ( 2 * ratioH );
        if ( lowResI > ( newHeight / ratioH ) - 1 ) lowResI = ( newHeight / ratioH ) - 1;
        int lowResJ = ( 2 * j + ratioW ) / ( 2 * ratioW );
        if ( lowResJ > ( newWidth / ratioW ) - 1 ) lowResJ = ( newWidth / ratioW ) - 1;
        int refIndex = pixelToPatchLowRes[lowResI * ( newWidth / ratioW ) + lowResJ];
        int curIndex = pixelToPatchHighRes[i * newWidth + j];
        if ( refIndex == curIndex ) {
          // just copy the value
          for ( int c = 0; c < 3; c++ ) {
            int posY = lowResI;
            int posX = lowResJ;
            destImage.setValue( c, j, i, refImage.getValue( c, posX, posY ) );
          }
        } else {
          // search the index that contains the same patchIndex
          int searchIndex = -1;
          for ( int ii = -1; ii < 2; ii++ ) {
            for ( int jj = -1; jj < 2; jj++ ) {
              int posY = lowResI + ii;
              if ( posY < 0 ) posY = 0;
              if ( posY > ( newHeight / ratioH ) - 1 ) posY = ( newHeight / ratioH ) - 1;
              int posX = lowResJ + jj;
              if ( posX < 0 ) posX = 0;
              if ( posX > ( newWidth / ratioW ) - 1 ) posX = ( newWidth / ratioW ) - 1;
              searchIndex = pixelToPatchLowRes[posY * ( newWidth / ratioW ) + posX];
              if ( searchIndex == curIndex ) {
                // copy the value from position (lowResI+ii),(lowResJ+jj)
                for ( int c = 0; c < 3; c++ ) { destImage.setValue( c, j, i, refImage.getValue( c, posX, posY ) ); }
                break;
              }
            }
          }
        }
      }
    }
  }
}

template <typename T>
bool PCCVideoDecoder::decompress( PCCVideo<T, 3>    &video,
                                  PCCContext        &contexts,
                                  const std::string &path,
                                  PCCVideoBitstream &bitstream,
                                  bool               byteStreamVideoCoder,
                                  PCCCodecId         codecId,
                                  const std::string &decoderPath,
                                  size_t             outputBitDepth,
                                  const std::string &scalerConfigPath,
                                  const bool         keepIntermediateFiles,
                                  const size_t       shvcLayerIndex,
                                  const bool         patchColorSubsampling,
                                  const std::string &inverseColorSpaceConversionConfig,
                                  const std::string &colorSpaceConversionPath,
                                  const size_t       upsamplingFilter ) {
  const std::string type        = bitstream.getExtension();
  const std::string fileName    = path + type;
  const std::string binFileName = fileName + ".bin";

  printf( "byteStreamVideoCoder = %d codecId = %d \n", byteStreamVideoCoder, codecId );
  fflush( stdout );
  if ( byteStreamVideoCoder ) {
    bitstream.sampleStreamToByteStream(
#if defined( USE_JMAPP_VIDEO_CODEC ) && defined( USE_JMLIB_VIDEO_CODEC )
        codecId == JMAPP || codecId == JMLIB,
#elif defined( USE_JMAPP_VIDEO_CODEC )
        codecId == JMAPP,
#elif defined( USE_JMLIB_VIDEO_CODEC )
        codecId == JMLIB,
#else
        false,
#endif
#if defined( USE_VTMLIB_VIDEO_CODEC )
        codecId == VTMLIB
#else
        false
#endif
    );
  }

  // Decode video
  auto decoder = PCCVirtualVideoDecoder<T>::create( codecId );
  printf( " decompress codecId = %d size(T) = %zu \n", (int)codecId, sizeof( T ) );
  fflush( stdout );
#ifdef USE_SHMAPP_VIDEO_CODEC
  if ( codecId == SHMAPP ) {
    std::shared_ptr<PCCSHMAppVideoDecoder<T>> shmDecoder =
        std::dynamic_pointer_cast<PCCSHMAppVideoDecoder<T>>( decoder );
    shmDecoder->setLayerIndex( shvcLayerIndex );
  }
#endif
  decoder->decode( bitstream, video, outputBitDepth, decoderPath, fileName );
  size_t width      = video.getWidth();
  size_t height     = video.getHeight();
  bool   is444      = video.is444();
  size_t frameIndex = 0;
  for ( auto &image : video ) {
    TRACE_PICTURE( " IdxOutOrderCntVal = %d, ", frameIndex++ );
    TRACE_PICTURE( " MD5checksumChan0 = %s, ", image.computeMD5( 0 ).c_str() );
    TRACE_PICTURE( " MD5checksumChan1 = %s, ", image.computeMD5( 1 ).c_str() );
    TRACE_PICTURE( " MD5checksumChan2 = %s \n", image.computeMD5( 2 ).c_str() );
  }
  TRACE_PICTURE( "Width =  %d, Height = %d \n", video.getWidth(), video.getHeight() );
  printf( "Decoded frame = %zu x %zu %zu bits is444 = %d NumFrames = %zu \n", width, height, outputBitDepth, is444,
          video.getFrameCount() );
  fflush( stdout );
  if ( keepIntermediateFiles ) {
    bitstream.write( binFileName );
    video.write( video.addFormat( fileName + "_rec", outputBitDepth == 8 ? "8" : "10" ), outputBitDepth == 8 ? 1 : 2 );
  }

  // Convert dec video
  std::shared_ptr<PCCVirtualColorConverter<T>> converter;
  std::string                                  configInverseColorSpace;
  if ( colorSpaceConversionPath.empty() || inverseColorSpaceConversionConfig.empty() ) {
    converter               = std::make_shared<PCCInternalColorConverter<T>>();
    configInverseColorSpace = stringFormat( "YUV420ToYUV444_%zu_%zu", outputBitDepth, upsamplingFilter );
  } else {
#ifdef USE_HDRTOOLS
    converter = std::make_shared<PCCHDRToolsLibColorConverter<T>>();
#else
    converter = std::make_shared<PCCHDRToolsAppColorConverter<T>>();
#endif
    configInverseColorSpace = inverseColorSpaceConversionConfig;
  }
  if ( inverseColorSpaceConversionConfig.empty() || is444 ) {
    if ( is444 ) {
      video.setDeprecatedColorFormat( 0 );
    } else {
      video.setDeprecatedColorFormat( 1 );
      video.convertYUV420ToYUV444();
    }
  } else {
    if ( patchColorSubsampling ) {
      PCCVideo<T, 3> video444;
      video444 = video;
      video444.convertYUV420ToYUV444();
      // perform color-upsampling based on patch information
      for ( size_t frNum = 0; frNum < video.getFrameCount(); frNum++ ) {
        // context variable, contains the patch information
        auto &context = contexts[frNum / 2];
        // full resolution image (already filled by previous dilation
        auto &refImage = video444.getFrame( frNum );
        // image that will contain the per-patch chroma sub-sampled image
        auto &destImage = video.getFrame( frNum );
        destImage.resize( width, height, PCCCOLORFORMAT::YUV444 );
        // iterate the patch information and perform chroma down-sampling on each patch individually
        std::vector<PCCPatch> &patches      = context.getTitleFrameContext().getPatches();
        std::vector<size_t>   &blockToPatch = context.getTitleFrameContext().getBlockToPatch();
        for ( int patchIdx = 0; patchIdx <= patches.size(); patchIdx++ ) {
          size_t occupancyResolution;
          size_t patch_left;
          size_t patch_top;
          size_t patch_width;
          size_t patch_height;
          if ( patchIdx == 0 ) {
            // background, does not have a corresponding patch
            auto &patch         = patches[0];
            occupancyResolution = patch.getOccupancyResolution();
            patch_left          = 0;
            patch_top           = 0;
            patch_width         = width;
            patch_height        = height;
          } else {
            auto &patch         = patches[patchIdx - 1];
            occupancyResolution = patch.getOccupancyResolution();
            patch_left          = patch.getU0() * occupancyResolution;
            patch_top           = patch.getV0() * occupancyResolution;
            if ( !( patch.isPatchDimensionSwitched() ) ) {
              patch_width  = patch.getSizeU0() * occupancyResolution;
              patch_height = patch.getSizeV0() * occupancyResolution;
            } else {
              patch_width  = patch.getSizeV0() * occupancyResolution;
              patch_height = patch.getSizeU0() * occupancyResolution;
            }
          }
          // initializing the image container with zeros
          PCCImage<T, 3> tmpImage;
          tmpImage.resize( patch_width, patch_height, PCCCOLORFORMAT::YUV444 );
          // cut out the patch image
          refImage.copyBlock( patch_top, patch_left, patch_width, patch_height, tmpImage );

          // fill in the blocks by extending the edges
          for ( size_t i = 0; i < patch_height / occupancyResolution; i++ ) {
            for ( size_t j = 0; j < patch_width / occupancyResolution; j++ ) {
              if ( blockToPatch[( i + patch_top / occupancyResolution ) * ( width / occupancyResolution ) + j +
                                patch_left / occupancyResolution] == patchIdx ) {
                // do nothing
                continue;
              } else {
                // search for the block that contains attribute information and
                // extend the block edge
                int              direction;
                int              searchIndex;
                std::vector<int> neighborIdx( 4, -1 );
                std::vector<int> neighborDistance( 4, ( std::numeric_limits<int>::max )() );
                // looking for the neighboring block to the left of the
                // current block
                searchIndex = j;
                while ( searchIndex >= 0 ) {
                  if ( blockToPatch[( i + patch_top / occupancyResolution ) * ( width / occupancyResolution ) +
                                    searchIndex + patch_left / occupancyResolution] == patchIdx ) {
                    neighborIdx[0]      = searchIndex;
                    neighborDistance[0] = j - searchIndex;
                    searchIndex         = 0;
                  }
                  searchIndex--;
                }
                // looking for the neighboring block to the right of the
                // current block
                searchIndex = j;
                while ( searchIndex < patch_width / occupancyResolution ) {
                  if ( blockToPatch[( i + patch_top / occupancyResolution ) * ( width / occupancyResolution ) +
                                    searchIndex + patch_left / occupancyResolution] == patchIdx ) {
                    neighborIdx[1]      = searchIndex;
                    neighborDistance[1] = searchIndex - j;
                    searchIndex         = patch_width / occupancyResolution;
                  }
                  searchIndex++;
                }
                // looking for the neighboring block above the current block
                searchIndex = i;
                while ( searchIndex >= 0 ) {
                  if ( blockToPatch[( searchIndex + patch_top / occupancyResolution ) *
                                        ( width / occupancyResolution ) +
                                    j + patch_left / occupancyResolution] == patchIdx ) {
                    neighborIdx[2]      = searchIndex;
                    neighborDistance[2] = i - searchIndex;
                    searchIndex         = 0;
                  }
                  searchIndex--;
                }
                // looking for the neighboring block below the current block
                searchIndex = i;
                while ( searchIndex < patch_height / occupancyResolution ) {
                  if ( blockToPatch[( searchIndex + patch_top / occupancyResolution ) *
                                        ( width / occupancyResolution ) +
                                    j + patch_left / occupancyResolution] == patchIdx ) {
                    neighborIdx[3]      = searchIndex;
                    neighborDistance[3] = searchIndex - i;
                    searchIndex         = patch_height / occupancyResolution;
                  }
                  searchIndex++;
                }
                // check if the candidate was found
                assert( *(std::max)( neighborIdx.begin(), neighborIdx.end() ) > 0 );
                // now fill in the block with the edge value coming from the
                // nearest neighbor
                direction =
                    (std::min_element)( neighborDistance.begin(), neighborDistance.end() ) - neighborDistance.begin();
                if ( direction == 0 ) {
                  // copying from left neighboring block
                  for ( size_t iBlk = 0; iBlk < occupancyResolution; iBlk++ ) {
                    for ( size_t jBlk = 0; jBlk < occupancyResolution; jBlk++ ) {
                      tmpImage.setValue(
                          0, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk,
                          tmpImage.getValue( 0, neighborIdx[0] * occupancyResolution + occupancyResolution - 1,
                                             i * occupancyResolution + iBlk ) );
                      tmpImage.setValue(
                          1, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk,
                          tmpImage.getValue( 1, neighborIdx[0] * occupancyResolution + occupancyResolution - 1,
                                             i * occupancyResolution + iBlk ) );
                      tmpImage.setValue(
                          2, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk,
                          tmpImage.getValue( 2, neighborIdx[0] * occupancyResolution + occupancyResolution - 1,
                                             i * occupancyResolution + iBlk ) );
                    }
                  }
                } else if ( direction == 1 ) {
                  // copying block from right neighboring position
                  for ( size_t iBlk = 0; iBlk < occupancyResolution; iBlk++ ) {
                    for ( size_t jBlk = 0; jBlk < occupancyResolution; jBlk++ ) {
                      tmpImage.setValue( 0, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk,
                                         tmpImage.getValue( 0, neighborIdx[1] * occupancyResolution,
                                                            i * occupancyResolution + iBlk ) );
                      tmpImage.setValue( 1, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk,
                                         tmpImage.getValue( 1, neighborIdx[1] * occupancyResolution,
                                                            i * occupancyResolution + iBlk ) );
                      tmpImage.setValue( 2, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk,
                                         tmpImage.getValue( 2, neighborIdx[1] * occupancyResolution,
                                                            i * occupancyResolution + iBlk ) );
                    }
                  }
                } else if ( direction == 2 ) {
                  // copying block from above
                  for ( size_t iBlk = 0; iBlk < occupancyResolution; iBlk++ ) {
                    for ( size_t jBlk = 0; jBlk < occupancyResolution; jBlk++ ) {
                      tmpImage.setValue(
                          0, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk,
                          tmpImage.getValue( 0, j * occupancyResolution + jBlk,
                                             neighborIdx[2] * occupancyResolution + occupancyResolution - 1 ) );
                      tmpImage.setValue(
                          1, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk,
                          tmpImage.getValue( 1, j * occupancyResolution + jBlk,
                                             neighborIdx[2] * occupancyResolution + occupancyResolution - 1 ) );
                      tmpImage.setValue(
                          2, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk,
                          tmpImage.getValue( 2, j * occupancyResolution + jBlk,
                                             neighborIdx[2] * occupancyResolution + occupancyResolution - 1 ) );
                    }
                  }
                } else if ( direction == 3 ) {
                  // copying block from below
                  for ( size_t iBlk = 0; iBlk < occupancyResolution; iBlk++ ) {
                    for ( size_t jBlk = 0; jBlk < occupancyResolution; jBlk++ ) {
                      tmpImage.setValue( 0, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk,
                                         tmpImage.getValue( 0, j * occupancyResolution + jBlk,
                                                            neighborIdx[3] * occupancyResolution ) );
                      tmpImage.setValue( 1, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk,
                                         tmpImage.getValue( 1, j * occupancyResolution + jBlk,
                                                            neighborIdx[3] * occupancyResolution ) );
                      tmpImage.setValue( 2, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk,
                                         tmpImage.getValue( 2, j * occupancyResolution + jBlk,
                                                            neighborIdx[3] * occupancyResolution ) );
                    }
                  }
                } else {
                  printf( "This condition should never occur, report an error" );
                  return false;
                }
              }
            }
          }
          // perform downsampling
          // const std::string rgbRecFileNamePatch = addVideoFormat( fileName
          // + "_tmp", patch_width, patch_height,
          // false, false ); const std::string yuvRecFileNamePatch =
          // addVideoFormat( fileName + "_tmp", patch_width,
          // patch_height, true, true );
          PCCVideo<T, 3> tmpVideo;
          tmpVideo.resize( 1 );
          tmpVideo[0] = tmpImage;
          converter->convert( configInverseColorSpace, tmpVideo, colorSpaceConversionPath, fileName + "_tmp" );
          tmpImage = tmpVideo[0];
          // substitute the pixels in the output image for compression
          for ( size_t i = 0; i < patch_height; i++ ) {
            for ( size_t j = 0; j < patch_width; j++ ) {
              if ( context.getTitleFrameContext().getBlockToPatch()[( ( i + patch_top ) / occupancyResolution ) *
                                                                        ( width / occupancyResolution ) +
                                                                    ( j + patch_left ) / occupancyResolution] ==
                   patchIdx ) {
                // do nothing
                for ( size_t cc = 0; cc < 3; cc++ ) {
                  destImage.setValue( cc, j + patch_left, i + patch_top, tmpImage.getValue( cc, j, i ) );
                }
              }
            }
          }
        }
      }
    } else {
      converter->convert( configInverseColorSpace, video, colorSpaceConversionPath, fileName + "_rec" );
      video.setDeprecatedColorFormat( colorSpaceConversionPath.empty() ? 1 : 2 );
      if ( keepIntermediateFiles ) { video.write( video.addFormat( fileName + "_rec", "16" ), 2 ); }
    }
  }

  // annex-b: Scale if necessary
  auto &asps                = contexts.getAtlasSequenceParameterSet( 0 );
  auto  expectedFrameWidth  = asps.getFrameWidth();
  auto  expectedFrameHeight = asps.getFrameHeight();

  if ( ( type != "occupancy" && ( type.find( "packedT" ) == std::string::npos ) ) &&
       ( width != expectedFrameWidth || height != expectedFrameHeight ) ) {
    PCCVideo<T, 3>                               scaledVideo;
    std::shared_ptr<PCCVirtualColorConverter<T>> upscaler;
#ifdef USE_HDRTOOLS
    if ( scalerConfigPath != "" ) {
      upscaler = std::make_shared<PCCHDRToolsLibColorConverter<T>>();
#else
    if ( !scalerConfigPath.empty() && !colorSpaceConversionPath.empty() ) {
      upscaler = std::make_shared<PCCHDRToolsAppColorConverter<T>>();
#endif
      upscaler->scale( scalerConfigPath, video, scaledVideo,
                       ( inverseColorSpaceConversionConfig.empty() || is444 ) ? 8 : 16 /*bitdepth*/,
                       3 /*chromaFormat=444*/, 0 /*colorSpace=YUV*/, expectedFrameWidth, expectedFrameHeight,
                       colorSpaceConversionPath, fileName + "_rec_US" );
    } else {
      patchBasedUpsample( video, scaledVideo, expectedFrameWidth, expectedFrameHeight, contexts );
    }
    video = scaledVideo;
    if ( keepIntermediateFiles ) {
      video.write( video.addFormat( fileName + "_rec", outputBitDepth == 8 ? "8" : "10" ),
                   outputBitDepth == 8 ? 1 : 2 );
    }
  }
  return true;
}

template void pcc::PCCVideoDecoder::patchBasedUpsample( PCCVideo<uint8_t, 3> &video,
                                                        PCCVideo<uint8_t, 3> &upsampledVideo,
                                                        const size_t          newWidth,
                                                        const size_t          newHeight,
                                                        PCCContext           &contexts );

template void pcc::PCCVideoDecoder::patchBasedUpsample( PCCVideo<uint16_t, 3> &video,
                                                        PCCVideo<uint16_t, 3> &upsampledVideo,
                                                        const size_t           newWidth,
                                                        const size_t           newHeight,
                                                        PCCContext            &contexts );

template bool pcc::PCCVideoDecoder::decompress<uint8_t>( PCCVideo<uint8_t, 3> &video,
                                                         PCCContext           &contexts,
                                                         const std::string    &path,
                                                         PCCVideoBitstream    &bitstream,
                                                         bool                  byteStreamVideoCoder,
                                                         PCCCodecId            codecId,
                                                         const std::string    &decoderPath,
                                                         size_t                outputBitDepth,
                                                         const std::string    &scalerConfigPath,
                                                         const bool            keepIntermediateFiles,
                                                         const size_t          shvcLayerIndex,
                                                         const bool            patchColorSubsampling,
                                                         const std::string    &inverseColorSpaceConversionConfig,
                                                         const std::string    &colorSpaceConversionPath,
                                                         const size_t          upsamplingFilter );

template bool pcc::PCCVideoDecoder::decompress<uint16_t>( PCCVideo<uint16_t, 3> &video,
                                                          PCCContext            &contexts,
                                                          const std::string     &path,
                                                          PCCVideoBitstream     &bitstream,
                                                          bool                   byteStreamVideoCoder,
                                                          PCCCodecId             codecId,
                                                          const std::string     &decoderPath,
                                                          size_t                 outputBitDepth,
                                                          const std::string     &scalerConfigPath,
                                                          const bool             keepIntermediateFiles,
                                                          const size_t           shvcLayerIndex,
                                                          const bool             patchColorSubsampling,
                                                          const std::string     &inverseColorSpaceConversionConfig,
                                                          const std::string     &colorSpaceConversionPath,
                                                          const size_t           upsamplingFilter );
