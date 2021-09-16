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
#include "PCCVideoEncoder.h"

#include "PCCVideoBitstream.h"
#include "PCCVideo.h"
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCPatch.h"
#include "PCCVirtualVideoEncoder.h"
#include "PCCInternalColorConverter.h"
#ifdef USE_HDRTOOLS
#include "PCCHDRToolsLibColorConverter.h"
#else
#include "PCCHDRToolsAppColorConverter.h"
#endif

using namespace pcc;

PCCVideoEncoder::PCCVideoEncoder() = default;

PCCVideoEncoder::~PCCVideoEncoder() = default;

template <typename T>
void PCCVideoEncoder::patchColorSubsmple( PCCVideo<T, 3>&    video,
                                          PCCContext&        contexts,
                                          const size_t       width,
                                          const size_t       height,
                                          const std::string& configColorSpace,
                                          const std::string& colorSpaceConversionPath,
                                          const std::string& fileName ) {
  printf( "Encoder convert : patchColorSubsampling\n" );

  std::shared_ptr<PCCVirtualColorConverter<T>> converter;
  if ( colorSpaceConversionPath.empty() ) {
    converter = std::make_shared<PCCInternalColorConverter<T>>();
  } else {
#ifdef USE_HDRTOOLS
    converter = std::make_shared<PCCHDRToolsLibColorConverter<T>>();
#else
    converter = std::make_shared<PCCHDRToolsAppColorConverter<T>>();
#endif
  }

  PCCVideo<T, 3> video444;
  // perform color-subsampling based on patch information
  video444.resize( video.getFrameCount() );
  for ( size_t frNum = 0; frNum < video.getFrameCount(); frNum++ ) {
    // context variable, contains the patch information
    auto& context = contexts[(int)( frNum / 2 )];
    // full resolution image (already filled by previous dilation
    auto& refImage = video.getFrame( frNum );
    // image that will contain the per-patch chroma sub-sampled image
    auto& destImage = video444.getFrame( frNum );
    destImage.resize( width, height, PCCCOLORFORMAT::YUV444 );

    // iterate the patch information and perform chroma down-sampling on
    // each patch individually
    std::vector<PCCPatch>& patches      = context.getTitleFrameContext().getPatches();
    std::vector<size_t>&   blockToPatch = context.getTitleFrameContext().getBlockToPatch();
    for ( int patchIdx = 0; patchIdx <= patches.size(); patchIdx++ ) {
      size_t occupancyResolution;
      size_t patch_left;
      size_t patch_top;
      size_t patch_width;
      size_t patch_height;
      if ( patchIdx == 0 ) {
        // background, does not have a corresponding patch
        auto& patch         = patches[0];
        occupancyResolution = patch.getOccupancyResolution();
        patch_left          = 0;
        patch_top           = 0;
        patch_width         = width;
        patch_height        = height;
      } else {
        auto& patch         = patches[patchIdx - 1];
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
            // search for the block that contains attribute information and extend the block edge
            int              direction;
            int              searchIndex;
            std::vector<int> neighborIdx( 4, -1 );
            std::vector<int> neighborDistance( 4, ( std::numeric_limits<int>::max )() );
            // looking for the neighboring block to the left of the current block
            searchIndex = (int)j;
            while ( searchIndex >= 0 ) {
              if ( blockToPatch[( i + patch_top / occupancyResolution ) * ( width / occupancyResolution ) +
                                searchIndex + patch_left / occupancyResolution] == patchIdx ) {
                neighborIdx[0]      = searchIndex;
                neighborDistance[0] = (int)j - searchIndex;
                searchIndex         = 0;
              }
              searchIndex--;
            }
            // looking for the neighboring block to the right of the current block
            searchIndex = (int)j;
            while ( searchIndex < patch_width / occupancyResolution ) {
              if ( blockToPatch[( i + patch_top / occupancyResolution ) * ( width / occupancyResolution ) +
                                searchIndex + patch_left / occupancyResolution] == patchIdx ) {
                neighborIdx[1]      = searchIndex;
                neighborDistance[1] = searchIndex - (int)j;
                searchIndex         = (int)patch_width / occupancyResolution;
              }
              searchIndex++;
            }
            // looking for the neighboring block above the current block
            searchIndex = (int)i;
            while ( searchIndex >= 0 ) {
              if ( blockToPatch[( searchIndex + patch_top / occupancyResolution ) * ( width / occupancyResolution ) +
                                j + patch_left / occupancyResolution] == patchIdx ) {
                neighborIdx[2]      = searchIndex;
                neighborDistance[2] = (int)i - searchIndex;
                searchIndex         = 0;
              }
              searchIndex--;
            }
            // looking for the neighboring block below the current block
            searchIndex = (int)i;
            while ( searchIndex < patch_height / occupancyResolution ) {
              if ( blockToPatch[( searchIndex + patch_top / occupancyResolution ) * ( width / occupancyResolution ) +
                                j + patch_left / occupancyResolution] == patchIdx ) {
                neighborIdx[3]      = searchIndex;
                neighborDistance[3] = searchIndex - (int)i;
                searchIndex         = (int)patch_height / occupancyResolution;
              }
              searchIndex++;
            }
            // check if the candidate was found
            assert( *( std::max )( neighborIdx.begin(), neighborIdx.end() ) > 0 );
            // now fill in the block with the edge value coming from the nearest neighbor
            direction = std::min_element( neighborDistance.begin(), neighborDistance.end() ) - neighborDistance.begin();
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
                  tmpImage.setValue(
                      0, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk,
                      tmpImage.getValue( 0, neighborIdx[1] * occupancyResolution, i * occupancyResolution + iBlk ) );
                  tmpImage.setValue(
                      1, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk,
                      tmpImage.getValue( 1, neighborIdx[1] * occupancyResolution, i * occupancyResolution + iBlk ) );
                  tmpImage.setValue(
                      2, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk,
                      tmpImage.getValue( 2, neighborIdx[1] * occupancyResolution, i * occupancyResolution + iBlk ) );
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
                  tmpImage.setValue(
                      0, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk,
                      tmpImage.getValue( 0, j * occupancyResolution + jBlk, neighborIdx[3] * occupancyResolution ) );
                  tmpImage.setValue(
                      1, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk,
                      tmpImage.getValue( 1, j * occupancyResolution + jBlk, neighborIdx[3] * occupancyResolution ) );
                  tmpImage.setValue(
                      2, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk,
                      tmpImage.getValue( 2, j * occupancyResolution + jBlk, neighborIdx[3] * occupancyResolution ) );
                }
              }
            } else {
              printf( "This condition should never occur, report an error" );
              return;
            }
          }
        }
      }
      // perform downsampling
      PCCVideo<T, 3> tmpVideo;
      tmpVideo.resize( 1 );
      tmpVideo[0] = tmpImage;
      converter->convert( configColorSpace, tmpVideo, colorSpaceConversionPath, fileName + "_tmp" );
      // substitute the pixels in the output image for compression
      for ( size_t i = 0; i < patch_height; i++ ) {
        for ( size_t j = 0; j < patch_width; j++ ) {
          if ( blockToPatch[( ( i + patch_top ) / occupancyResolution ) * ( width / occupancyResolution ) +
                            ( j + patch_left ) / occupancyResolution] == patchIdx ) {
            // do nothing
            for ( size_t cc = 0; cc < 3; cc++ ) {
              destImage.setValue( cc, j + patch_left, i + patch_top, tmpImage.getValue( cc, j, i ) );
            }
          }
        }
      }
    }
  }
  // saving the video
  video444.convertYUV444ToYUV420();
  video = video444;
}

template <typename T>
bool PCCVideoEncoder::compress( PCCVideo<T, 3>&    video,
                                const std::string& path,
                                const int          qp,
                                PCCVideoBitstream& bitstream,
                                const std::string& encoderConfig,
                                const std::string& encoderPath,
                                PCCCodecId         codecId,
                                bool               byteStreamVideoCoder,
                                PCCContext&        contexts,
                                const size_t       nbyte,
                                const bool         use444CodecIo,
                                const bool         use3dmv,
                                const bool         usePccRDO,
                                const size_t       shvcLayerIndex,
                                const size_t       shvcRateX,
                                const size_t       shvcRateY,
                                const size_t       internalBitDepth,
                                const bool         useConversion,
                                const bool         keepIntermediateFiles,
                                const std::string& colorSpaceConversionConfig,
                                const std::string& inverseColorSpaceConversionConfig,
                                const std::string& colorSpaceConversionPath,
                                const size_t       downsamplingFilter,
                                const size_t       upsamplingFilter,
                                const bool         patchColorSubsampling ) {
  auto& frames = video.getFrames();
  if ( frames.empty() || frames[0].getChannelCount() != 3 ) { return false; }
  const size_t      width                = frames[0].getWidth();
  const size_t      height               = frames[0].getHeight();
  const size_t      depth                = nbyte == 1 ? 8 : 10;
  const std::string type                 = bitstream.getExtension();
  const std::string fileName             = path + type;
  const std::string binFileName          = fileName + ".bin";
  const std::string blockToPatchFileName = path + "blockToPatch.txt";
  const std::string occupancyMapFileName = path + "occupancy.txt";
  const std::string patchInfoFileName    = path + "patchInfo.txt";
  const char*       bitdepth             = nbyte == 2 ? "10" : "8";
  const std::string srcYuvFileName =
      addVideoFormat( fileName, width, height, !use444CodecIo, !use444CodecIo, bitdepth );
  const std::string srcRgbFileName = addVideoFormat( fileName, width, height, false, !use444CodecIo, bitdepth );
  const std::string recYuvFileName =
      addVideoFormat( fileName + "_rec", width, height, !use444CodecIo, !use444CodecIo, bitdepth );
  const bool yuvVideo = colorSpaceConversionConfig.empty() || use444CodecIo;

  std::shared_ptr<PCCVirtualColorConverter<T>> converter;
  std::string                                  configInverseColorSpace, configColorSpace;
  if ( colorSpaceConversionPath.empty() ) {
    converter               = std::make_shared<PCCInternalColorConverter<T>>();
    configInverseColorSpace = stringFormat( "YUV420ToYUV444_%zu_%zu", depth, upsamplingFilter );
    configColorSpace        = stringFormat( "RGB444ToYUV420_%zu_%zu", depth, downsamplingFilter );
  } else {
#ifdef USE_HDRTOOLS
    converter = std::make_shared<PCCHDRToolsLibColorConverter<T>>();
#else
    converter = std::make_shared<PCCHDRToolsAppColorConverter<T>>();
#endif
    configInverseColorSpace = inverseColorSpaceConversionConfig;
    configColorSpace        = colorSpaceConversionConfig;
  }

  // Convert src video
  if ( yuvVideo ) {
    if ( !use444CodecIo ) {
      printf( "Encoder convert : write420 without conversion: %s \n", srcYuvFileName.c_str() );
      if ( video.getColorFormat() == PCCCOLORFORMAT::YUV444 ) { video.convertYUV444ToYUV420(); }
    }
  } else {
    if ( keepIntermediateFiles ) { video.write( srcRgbFileName, nbyte ); }
    if ( patchColorSubsampling ) {
      patchColorSubsmple( video, contexts, width, height, configColorSpace, colorSpaceConversionPath, fileName );
    } else {
      converter->convert( configColorSpace, video, colorSpaceConversionPath, fileName + "_rec" );
    }
  }

  if ( keepIntermediateFiles ) { video.write( srcYuvFileName, nbyte ); }

  // Encode video
  PCCVideoEncoderParameters params;
  params.encoderPath_                 = encoderPath;
  params.srcYuvFileName_              = srcYuvFileName;
  params.binFileName_                 = binFileName;
  params.recYuvFileName_              = recYuvFileName;
  params.encoderConfig_               = encoderConfig;
  params.qp_                          = ( std::min )( ( std::max )( qp, -12 ), 51 );
  params.inputBitDepth_               = depth;
  params.internalBitDepth_            = internalBitDepth;
  params.outputBitDepth_              = depth;
  params.use444CodecIo_               = use444CodecIo;
  params.usePccMotionEstimation_      = use3dmv;
  params.blockToPatchFile_            = blockToPatchFileName;
  params.occupancyMapFile_            = occupancyMapFileName;
  params.patchInfoFile_               = patchInfoFileName;
  params.cuTransquantBypassFlagForce_ = false;
  params.transquantBypassEnable_      = false;
  params.inputColourSpaceConvert_     = use444CodecIo;
  params.usePccRDO_                   = usePccRDO;
  params.shvcLayerIndex_              = shvcLayerIndex;
  params.shvcRateX_                   = shvcRateX;
  params.shvcRateY_                   = shvcRateY;
  printf( "Encode: video size = %zu x %zu num frames = %zu \n", video.getWidth(), video.getHeight(),
          video.getFrameCount() );
  fflush( stdout );
  PCCVideo<T, 3> videoRec;
  auto           encoder = PCCVirtualVideoEncoder<T>::create( codecId );
  encoder->encode( video, params, bitstream, videoRec );

  size_t frameIndex = 0;
  for ( auto& image : videoRec ) {
    TRACE_PICTURE( " IdxOutOrderCntVal = %d, ", frameIndex++ );
    TRACE_PICTURE( " MD5checksumChan0 = %s, ", image.computeMD5( 0 ).c_str() );
    TRACE_PICTURE( " MD5checksumChan1 = %s, ", image.computeMD5( 1 ).c_str() );
    TRACE_PICTURE( " MD5checksumChan2 = %s \n", image.computeMD5( 2 ).c_str() );
  }
  TRACE_PICTURE( "Width =  %d, Height = %d \n", videoRec.getWidth(), videoRec.getHeight() );

  if ( keepIntermediateFiles ) {
    bitstream.write( binFileName );
    videoRec.write( recYuvFileName, nbyte );
  }
  // Convert rec video
  if ( yuvVideo ) {
    if ( use444CodecIo ) {
      videoRec.setDeprecatedColorFormat( 0 );
    } else {
      videoRec.convertYUV420ToYUV444();
      videoRec.setDeprecatedColorFormat( 1 );
    }
    video = videoRec;
  } else {
    if ( keepIntermediateFiles ) { videoRec.write( recYuvFileName, nbyte ); }
    converter->convert( configInverseColorSpace, videoRec, video, colorSpaceConversionPath, fileName + "_rec" );
    if ( keepIntermediateFiles ) { video.write( video.addFormat( fileName + "_rec", "16" ), 2 ); }
    video.setDeprecatedColorFormat( 2 );
  }

  if ( byteStreamVideoCoder ) { bitstream.byteStreamToSampleStream(); }
  return true;
}

template bool pcc::PCCVideoEncoder::compress<uint8_t>( PCCVideo<uint8_t, 3>& video,
                                                       const std::string&    path,
                                                       const int             qp,
                                                       PCCVideoBitstream&    bitstream,
                                                       const std::string&    encoderConfig,
                                                       const std::string&    encoderPath,
                                                       PCCCodecId            codecId,
                                                       bool                  byteStreamVideoCoder,
                                                       PCCContext&           contexts,
                                                       const size_t          nbyte,
                                                       const bool            use444CodecIo,
                                                       const bool            use3dmv,
                                                       const bool            usePccRDO,
                                                       const size_t          shvcLayerIndex,
                                                       const size_t          shvcRateX,
                                                       const size_t          shvcRateY,
                                                       const size_t          internalBitDepth,
                                                       const bool            useConversion,
                                                       const bool            keepIntermediateFiles,
                                                       const std::string&    colorSpaceConversionConfig,
                                                       const std::string&    inverseColorSpaceConversionConfig,
                                                       const std::string&    colorSpaceConversionPath,
                                                       const size_t          downsamplingFilter,
                                                       const size_t          upsamplingFilter,
                                                       const bool            patchColorSubsampling );

template bool pcc::PCCVideoEncoder::compress<uint16_t>( PCCVideo<uint16_t, 3>& video,
                                                        const std::string&     path,
                                                        const int              qp,
                                                        PCCVideoBitstream&     bitstream,
                                                        const std::string&     encoderConfig,
                                                        const std::string&     encoderPath,
                                                        PCCCodecId             codecId,
                                                        bool                   byteStreamVideoCoder,
                                                        PCCContext&            contexts,
                                                        const size_t           nbyte,
                                                        const bool             use444CodecIo,
                                                        const bool             use3dmv,
                                                        const bool             usePccRDO,
                                                        const size_t           shvcLayerIndex,
                                                        const size_t           shvcRateX,
                                                        const size_t           shvcRateY,
                                                        const size_t           internalBitDepth,
                                                        const bool             useConversion,
                                                        const bool             keepIntermediateFiles,
                                                        const std::string&     colorSpaceConversionConfig,
                                                        const std::string&     inverseColorSpaceConversionConfig,
                                                        const std::string&     colorSpaceConversionPath,
                                                        const size_t           downsamplingFilter,
                                                        const size_t           upsamplingFilter,
                                                        const bool             patchColorSubsampling );
