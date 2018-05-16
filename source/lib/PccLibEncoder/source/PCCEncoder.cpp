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
#include "PCCPatchSegmenter.h"
#include "PCCVideoEncoder.h"
#include "PCCSystem.h"
#include "PCCGroupOfFrames.h"
#include "PCCPointSet.h"
#include "PCCEncoderParameters.h"

#include <tbb/tbb.h>

#include "PCCEncoder.h"

using namespace std;
using namespace pcc;

void EncodeUInt32(const uint32_t value, const uint32_t bitCount,
                  o3dgc::Arithmetic_Codec &arithmeticEncoder, o3dgc::Static_Bit_Model &bModel0) {
  uint32_t valueToEncode = PCCToLittleEndian<uint32_t>(value);
  for (uint32_t i = 0; i < bitCount; ++i) {
    arithmeticEncoder.encode(valueToEncode & 1, bModel0);
    valueToEncode >>= 1;
  }
}

PCCEncoder::PCCEncoder(){
}
PCCEncoder::~PCCEncoder(){
}

void PCCEncoder::setParameters( PCCEncoderParameters params ) { 
  params_ = params; 
}

int PCCEncoder::compress( const PCCGroupOfFrames& sources, PCCContext &context,
                          PCCBitstream &bitstream, PCCGroupOfFrames& reconstructs ){
  assert( sources.size() < 256);
  if( sources.size() == 0 ) {
    return 0;
  }

  if( params_.nbThread_ > 0 ) {
    tbb::task_scheduler_init init( (int)params_.nbThread_ );
  }
  reconstructs.resize( sources.size() );
  context.resize( sources.size() );
  auto& frames = context.getFrames();
  PCCVideoEncoder videoEncoder;
  const size_t pointCount = sources[ 0 ].getPointCount();
  std::stringstream path;
  path << removeFileExtension( params_.compressedStreamPath_ ) << "_GOF" << context.getIndex() << "_";


  generateGeometryVideo( sources, context );
  resizeGeometryVideo( context );
  dilateGeometryVideo( context );
  auto& width  = context.getWidth ();
  auto& height = context.getHeight();
  width  = (uint16_t)frames[0].getWidth ();
  height = (uint16_t)frames[0].getHeight();
  compressHeader( context, bitstream );

  auto sizeGeometryVideo = bitstream.size();
  auto& videoGeometry = context.getVideoGeometry();
  videoEncoder.compress( videoGeometry, path.str() + "geometry", params_.geometryQP_, bitstream,
                        params_.geometryConfig_, params_.videoEncoderPath_);
  videoGeometry.read420(  addVideoFormat( path.str() + "geometry_rec.yuv", width, height ),
                          width, height,
                          2 * frames.size() );
  sizeGeometryVideo = bitstream.size() - sizeGeometryVideo;

  if( !params_.keepIntermediateFiles_ ){
    removeFiles( path.str() + "geometry.bin" );
    removeFiles( addVideoFormat( path.str() + "geometry.yuv"    , width, height ) );
    removeFiles( addVideoFormat( path.str() + "geometry_rec.yuv", width, height ) );
  }  

  std::cout << "geometry video ->" << sizeGeometryVideo << " B ("
      << (sizeGeometryVideo * 8.0) / (2 * frames.size() * pointCount) << " bpp)"
      << std::endl;

  auto sizeOccupancyMap = bitstream.size();
  compressOccupancyMap( context, bitstream );
  sizeOccupancyMap = bitstream.size() - sizeOccupancyMap;
  std::cout << " occupancy map  ->" << sizeOccupancyMap << " B ("
      << (sizeOccupancyMap * 8.0) / (2 * frames.size() * pointCount) << " bpp)"
      << std::endl;

  GeneratePointCloudParameters generatePointCloudParameters = {
      params_.occupancyResolution_,
      params_.neighborCountSmoothing_,
      params_.radius2Smoothing_,
      params_.radius2BoundaryDetection_,
      params_.thresholdSmoothing_,
      params_.losslessGeo_,
      params_.nbThread_
  };
  generatePointCloud( reconstructs, context, generatePointCloudParameters );

  if( !params_.noAttributes_ ) {
    generateTextureVideo( sources, reconstructs, context );

    auto& videoTexture = context.getVideoTexture();
    const size_t textureFrameCount = 2 * frames.size();
    assert(textureFrameCount == videoTexture.getFrameCount());
    for (size_t f = 0; f < textureFrameCount; ++f) {
      dilate( frames[ f / 2 ], videoTexture.getFrame( f ) );
    }

    auto sizeTextureVideo = bitstream.size();
    videoEncoder.compress( videoTexture,path.str() + "texture", params_.textureQP_,
                           bitstream, params_.textureConfig_,
                           params_.videoEncoderPath_, params_.colorSpaceConversionConfig_,
                           params_.colorSpaceConversionPath_, params_.losslessTexture_);
    const std::string yuvFileName = addVideoFormat( path.str() + "texture_rec"  + ( params_.losslessTexture_ ? ".rgb" : ".yuv" ),
                                                    width, height, params_.losslessTexture_ == 0 );
    if (params_.inverseColorSpaceConversionConfig_.empty() || params_.colorSpaceConversionPath_.empty() ||
        params_.losslessTexture_ ) {
      if ( params_.losslessTexture_ )
        videoTexture.read( yuvFileName, width, height, textureFrameCount);
      else
        videoTexture.read420(yuvFileName, width, height, textureFrameCount);
    } else {
      std::stringstream cmd;
      const std::string rgbFileName = addVideoFormat( path.str() + "texture_rec.rgb", width, height );
      cmd << params_.colorSpaceConversionPath_ << " -f " << params_.inverseColorSpaceConversionConfig_
          << " -p SourceFile=\"" << yuvFileName << "\" -p OutputFile=\"" << rgbFileName
          << "\" -p SourceWidth=" << width << " -p SourceHeight=" << height
          << " -p NumberOfFrames=" << textureFrameCount;
      std::cout << cmd.str() << '\n';
      if (int ret = pcc::system(cmd.str().c_str())) {
        std::cout << "Error: can't run system command!" << std::endl;
        return ret;
      }
      videoTexture.read(rgbFileName, width, height, textureFrameCount);
    }

    sizeTextureVideo = bitstream.size() - sizeTextureVideo;
    std::cout << "texture video  ->" << sizeTextureVideo << " B ("
        << (sizeTextureVideo * 8.0) / pointCount << " bpp)" << std::endl;
    if( !params_.keepIntermediateFiles_ ) {
      removeFiles( path.str() + "texture.bin" );
      removeFiles( addVideoFormat( path.str() + "texture"     + ( params_.losslessTexture_ ? ".rgb" : ".yuv" ),
                                   width, height, params_.losslessTexture_ == 0 ) );
      removeFiles( addVideoFormat( path.str() + "texture_rec" + ( params_.losslessTexture_ ? ".rgb" : ".yuv" ),
                                   width, height, params_.losslessTexture_ == 0 ) );
      if (!params_.colorSpaceConversionConfig_.empty() &&
          !params_.colorSpaceConversionPath_.empty() &&
          !params_.inverseColorSpaceConversionConfig_.empty() &&
          ( params_.losslessTexture_ == 0 )) {
        removeFiles( addVideoFormat( path.str() + "texture.rgb",     width, height ) );
        removeFiles( addVideoFormat( path.str() + "texture_rec.rgb", width, height ) );
      }
    }
  }
  colorPointCloud( reconstructs, context, params_.noAttributes_ != 0, params_.colorTransform_ );
  return 0;
}

void PCCEncoder::printMap(std::vector<bool> img, const size_t sizeU, const size_t sizeV) {
  std::cout << std::endl;
  std::cout << "PrintMap size = " << sizeU << " x " << sizeV << std::endl;
  for (size_t v = 0; v < sizeV; ++v) {
    for (size_t u = 0; u < sizeU; ++u) {
      std::cout << (img[v * sizeU + u] ? 'X' : '.');
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}
void PCCEncoder::pack( PCCFrameContext& frame  ) {
  auto& width   = frame.getWidth(); 
  auto& height  = frame.getHeight(); 
  auto& patches = frame.getPatches();
  if (patches.empty()) {
    return;
  }
  std::sort( patches.begin(), patches.end() );
  size_t occupancySizeU = params_.minimumImageWidth_ / params_.occupancyResolution_;
  size_t occupancySizeV = (std::max)( params_.minimumImageHeight_ / params_.occupancyResolution_, patches[0].getSizeV0());
  for (auto &patch : patches) { occupancySizeU = (std::max)( occupancySizeU, patch.getSizeU0() + 1); }

  width  = occupancySizeU * params_.occupancyResolution_;
  height = occupancySizeV * params_.occupancyResolution_;

  std::vector<bool> occupancyMap;
  occupancyMap.resize( occupancySizeU * occupancySizeV, false );
  for (auto &patch : patches) {
    assert(patch.getSizeU0() <= occupancySizeU);
    assert(patch.getSizeV0() <= occupancySizeV);
    bool locationFound = false;
    auto& occupancy =  patch.getOccupancy();
    while (!locationFound) {
      // print(patch.occupancy, patch.getSizeU0(), patch.getSizeV0());
      // print(occupancyMap, occupancySizeU, occupancySizeV);
      for (size_t v = 0; v <= occupancySizeV - patch.getSizeV0() && !locationFound; ++v) {
        for (size_t u = 0; u <= occupancySizeU - patch.getSizeU0(); ++u) {
          bool canFit = true;
          for (size_t v0 = 0; v0 < patch.getSizeV0() && canFit; ++v0) {
            const size_t y = v + v0;
            for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
              const size_t x = u + u0;

              if (occupancy[v0 * patch.getSizeU0() + u0] && occupancyMap[y * occupancySizeU + x]) {
                canFit = false;
                break;
              }
            }
          }
          if (!canFit) {
            continue;
          }
          locationFound = true;
          patch.getU0() = u;
          patch.getV0() = v;
          break;
        }
      }
      if (!locationFound) {
        occupancySizeV *= 2;
        occupancyMap.resize( occupancySizeU * occupancySizeV );
      }
    }
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      const size_t v = patch.getV0() + v0;
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        const size_t u = patch.getU0() + u0;
        occupancyMap[v * occupancySizeU + u] =
            occupancyMap[v * occupancySizeU + u] || occupancy[v0 * patch.getSizeU0() + u0];
      }
    }

    height = (std::max)( height, (patch.getV0() + patch.getSizeV0()) * patch.getOccupancyResolution() );
    width  = (std::max)( width,  (patch.getU0() + patch.getSizeU0()) * patch.getOccupancyResolution() );
    // print(occupancyMap, occupancySizeU, occupancySizeV);
  }
  printMap( occupancyMap, occupancySizeU, occupancySizeV );
  std::cout << "actualImageSizeU " << width  << std::endl;
  std::cout << "actualImageSizeV " << height << std::endl;
}

bool PCCEncoder::generateGeometryVideo( const PCCPointSet3& source, PCCFrameContext& frame,
                                        const PCCPatchSegmenter3Parameters segmenterParams,
                                        PCCVideo3B &videoGeometry ) {
  if (!source.getPointCount()) {
    return false;
  }
  auto& patches = frame.getPatches();
  patches.reserve(256);
  PCCPatchSegmenter3 segmenter;
  segmenter.setNbThread( params_.nbThread_ );
  segmenter.compute( source, segmenterParams, patches );
  pack( frame );
  return true;
}

void PCCEncoder::generateOccupancyMap( PCCFrameContext& frame ) {
  auto& occupancyMap = frame.getOccupancyMap();
  auto& width   = frame.getWidth();
  auto& height  = frame.getHeight();
  occupancyMap.resize( width * height, 0 );
  const int16_t infiniteDepth = (std::numeric_limits<int16_t>::max)();
  for (const auto &patch : frame.getPatches() ) {
    const size_t v0 = patch.getV0() * patch.getOccupancyResolution();
    const size_t u0 = patch.getU0() * patch.getOccupancyResolution();
    for (size_t v = 0; v < patch.getSizeV(); ++v) {
      for (size_t u = 0; u < patch.getSizeU(); ++u) {
        const size_t p = v * patch.getSizeU() + u;
        const int16_t d = patch.getDepth(0)[p];
        if (d < infiniteDepth) {
          const size_t x = (u0 + u);
          const size_t y = (v0 + v);
          assert(x < width && y < height);
          occupancyMap[x + y * width] = 1;
        }
      }
    }
  }
}

void PCCEncoder::generateIntraImage( PCCFrameContext& frame, const size_t depthIndex, PCCImage3B &image) {
  auto& width  = frame.getWidth();
  auto& height = frame.getHeight();
  image.resize( width, height );
  image.set(0);
  const int16_t infiniteDepth = (std::numeric_limits<int16_t>::max)();
  size_t maxDepth = 0;
  for (const auto &patch : frame.getPatches() ) {
    const size_t v0 = patch.getV0() * patch.getOccupancyResolution();
    const size_t u0 = patch.getU0() * patch.getOccupancyResolution();
    for (size_t v = 0; v < patch.getSizeV(); ++v) {
      for (size_t u = 0; u < patch.getSizeU(); ++u) {
        const size_t p = v * patch.getSizeU() + u;
        const int16_t d = patch.getDepth(depthIndex)[p];
        if (d < infiniteDepth) {
          const size_t x = (u0 + u);
          const size_t y = (v0 + v);
          assert(x < width && y < height);
          image.setValue(0, x, y, uint8_t(d));
          maxDepth = (std::max)(maxDepth, patch.getSizeD());
        }
      }
    }
  }
  std::cout << "maxDepth " << maxDepth << std::endl;
  if (maxDepth > 255) {
    std::cout << "Error: maxDepth > 255" << maxDepth << std::endl;
    exit(-1);
  }
}

bool PCCEncoder::predictGeometryFrame( PCCFrameContext& frame, const PCCImage3B &reference, PCCImage3B &image) {
  assert(reference.getWidth() ==  image.getWidth());
  assert(reference.getHeight() == image.getHeight());
  const size_t refWidth  = reference.getWidth();
  const size_t refHeight = reference.getHeight();
  auto& occupancyMap = frame.getOccupancyMap();
  for (size_t y = 0; y < refHeight; ++y) {
    for (size_t x = 0; x < refWidth; ++x) {
      const size_t pos1 = y * refWidth + x;
      if (occupancyMap[pos1] == 1) {
        for (size_t c = 0; c < 3; ++c) {
          const uint8_t value1 = image.getValue(c, x, y);
          const uint8_t value0 = reference.getValue(c, x, y);
          assert(value0 <= value1);
          const uint8_t delta = value1 - value0;
          assert(delta < 10);
          image.setValue(c, x, y, delta);
        }
      }
    }
  }
  return true;
}


bool PCCEncoder::generateGeometryVideo( const PCCGroupOfFrames& sources, PCCContext &context ) {

  PCCPatchSegmenter3Parameters segmenterParams;
  segmenterParams.nnNormalEstimation                    = params_.nnNormalEstimation_;
  segmenterParams.maxNNCountRefineSegmentation          = params_.maxNNCountRefineSegmentation_;
  segmenterParams.iterationCountRefineSegmentation      = params_.iterationCountRefineSegmentation_;
  segmenterParams.occupancyResolution                   = params_.occupancyResolution_;
  segmenterParams.minPointCountPerCCPatchSegmentation   = params_.minPointCountPerCCPatchSegmentation_;
  segmenterParams.maxNNCountPatchSegmentation           = params_.maxNNCountPatchSegmentation_;
  segmenterParams.surfaceThickness                      = params_.surfaceThickness_;
  segmenterParams.maxAllowedDepth                       = params_.maxAllowedDepth_;
  segmenterParams.maxAllowedDist2MissedPointsDetection  = params_.maxAllowedDist2MissedPointsDetection_;
  segmenterParams.maxAllowedDist2MissedPointsSelection  = params_.maxAllowedDist2MissedPointsSelection_;
  segmenterParams.lambdaRefineSegmentation              = params_.lambdaRefineSegmentation_;
  auto& videoGeometry = context.getVideoGeometry();
  auto & frames = context.getFrames();
  for( size_t i =0; i<frames.size();i++){
    generateGeometryVideo( sources[ i ], frames[ i ], segmenterParams, videoGeometry );
  }
  return true;
}

bool PCCEncoder::resizeGeometryVideo( PCCContext &context ) {
  size_t maxWidth = 0, maxHeight = 0;
  auto& videoGeometry = context.getVideoGeometry();
  for (auto &frame : context.getFrames() ) {
    maxWidth  = (std::max)( maxWidth,  frame.getWidth () );
    maxHeight = (std::max)( maxHeight, frame.getHeight() );
  }
  for (auto &frame : context.getFrames() ) {
    frame.getWidth () = maxWidth;
    frame.getHeight() = maxHeight;
    frame.getOccupancyMap().resize( ( maxWidth  / params_.occupancyResolution_ ) * ( maxHeight / params_.occupancyResolution_ ) );
  }
  return true;
}

bool PCCEncoder::dilateGeometryVideo( PCCContext &context) {
  auto& videoGeometry = context.getVideoGeometry();
  for (auto &frame : context.getFrames() ) {
    generateOccupancyMap( frame );
    const size_t shift = videoGeometry.getFrameCount();
    videoGeometry.resize( shift + 2 );
    for (size_t f = 0; f < 2; ++f) {
      PCCImage3B &frame1 = videoGeometry.getFrame( shift + f );
      generateIntraImage( frame, f, frame1);
      if (f) {
        predictGeometryFrame( frame, videoGeometry.getFrame( shift ), frame1 );
      }
    }
    dilate( frame, videoGeometry.getFrame( shift ) );
  }
  return true;
}

void PCCEncoder::dilate( PCCFrameContext& frame, PCCImage3B &image, const PCCImage3B *reference ) {
  auto occupancyMapTemp = frame.getOccupancyMap();
  const size_t pixelBlockCount = params_.occupancyResolution_ * params_.occupancyResolution_;
  const size_t occupancyMapSizeU = image.getWidth () / params_.occupancyResolution_;
  const size_t occupancyMapSizeV = image.getHeight() / params_.occupancyResolution_;
  const int64_t neighbors[4][2] = {{0, -1}, {-1, 0}, {1, 0}, {0, 1}};
  const size_t MAX_OCCUPANCY_RESOLUTION = 64;
  assert(params_.occupancyResolution_ <= MAX_OCCUPANCY_RESOLUTION);
  size_t count[MAX_OCCUPANCY_RESOLUTION][MAX_OCCUPANCY_RESOLUTION];
  PCCVector3<int32_t> values[MAX_OCCUPANCY_RESOLUTION][MAX_OCCUPANCY_RESOLUTION];

  for (size_t v1 = 0; v1 < occupancyMapSizeV; ++v1) {
    const int64_t v0 = v1 * params_.occupancyResolution_;
    for (size_t u1 = 0; u1 < occupancyMapSizeU; ++u1) {
      const int64_t u0 = u1 * params_.occupancyResolution_;
      size_t nonZeroPixelCount = 0;
      for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
        for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
          const int64_t x0 = u0 + u2;
          const int64_t y0 = v0 + v2;
          assert(x0 < int64_t(image.getWidth()) && y0 < int64_t(image.getHeight()));
          const size_t location0 = y0 * image.getWidth() + x0;
          nonZeroPixelCount += (occupancyMapTemp[location0] == 1);
        }
      }
      if (!nonZeroPixelCount) {
        if (reference) {
          for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
            for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
              const size_t x0 = u0 + u2;
              const size_t y0 = v0 + v2;
              image.setValue(0, x0, y0, reference->getValue(0, x0, y0));
              image.setValue(1, x0, y0, reference->getValue(1, x0, y0));
              image.setValue(2, x0, y0, reference->getValue(2, x0, y0));
            }
          }
        } else if (u1 > 0) {
          for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
            for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
              const size_t x0 = u0 + u2;
              const size_t y0 = v0 + v2;
              assert(x0 > 0);
              const size_t x1 = x0 - 1;
              image.setValue(0, x0, y0, image.getValue(0, x1, y0));
              image.setValue(1, x0, y0, image.getValue(1, x1, y0));
              image.setValue(2, x0, y0, image.getValue(2, x1, y0));
            }
          }
        } else if (v1 > 0) {
          for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
            for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
              const size_t x0 = u0 + u2;
              const size_t y0 = v0 + v2;
              assert(y0 > 0);
              const size_t y1 = y0 - 1;
              image.setValue(0, x0, y0, image.getValue(0, x0, y1));
              image.setValue(1, x0, y0, image.getValue(1, x0, y1));
              image.setValue(2, x0, y0, image.getValue(2, x0, y1));
            }
          }
        }
        continue;
      }
      for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
        for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
          values[v2][u2] = 0;
          count[v2][u2] = 0UL;
        }
      }
      uint32_t iteration = 1;
      while (nonZeroPixelCount < pixelBlockCount) {
        for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
          for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
            const int64_t x0 = u0 + u2;
            const int64_t y0 = v0 + v2;
            assert(x0 < int64_t(image.getWidth()) && y0 < int64_t(image.getHeight()));
            const size_t location0 = y0 * image.getWidth() + x0;
            if (occupancyMapTemp[location0] == iteration) {
              for (size_t n = 0; n < 4; ++n) {
                const int64_t x1 = x0 + neighbors[n][0];
                const int64_t y1 = y0 + neighbors[n][1];
                const size_t location1 = y1 * image.getWidth() + x1;
                if (x1 >= u0 && x1 < int64_t(u0 + params_.occupancyResolution_) && y1 >= v0 &&
                    y1 < int64_t( v0 + params_.occupancyResolution_ ) && occupancyMapTemp[location1] == 0) {
                  const int64_t u3 = u2 + neighbors[n][0];
                  const int64_t v3 = v2 + neighbors[n][1];
                  assert(u3 >= 0 && u3 < int64_t( params_.occupancyResolution_ ));
                  assert(v3 >= 0 && v3 < int64_t( params_.occupancyResolution_ ));
                  for (size_t k = 0; k < 3; ++k) {
                    values[v3][u3][k] += image.getValue(k, x0, y0);
                  }
                  ++count[v3][u3];
                }
              }
            }
          }
        }
        for (size_t v2 = 0; v2 < params_.occupancyResolution_; ++v2) {
          for (size_t u2 = 0; u2 < params_.occupancyResolution_; ++u2) {
            if (count[v2][u2]) {
              ++nonZeroPixelCount;
              const size_t x0 = u0 + u2;
              const size_t y0 = v0 + v2;
              const size_t location0 = y0 * image.getWidth() + x0;
              const size_t c = count[v2][u2];
              const size_t c2 = c / 2;
              occupancyMapTemp[location0] = iteration + 1;
              for (size_t k = 0; k < 3; ++k) {
                image.setValue(k, x0, y0, uint8_t((values[v2][u2][k] + c2) / c));
              }
              values[v2][u2] = 0;
              count[v2][u2] = 0UL;
            }
          }
        }
        ++iteration;
      }
    }
  }
}

bool PCCEncoder::generateTextureVideo( const PCCGroupOfFrames& sources, PCCGroupOfFrames& reconstructs, PCCContext& context ) {
  auto& frames = context.getFrames();
  auto& videoTexture = context.getVideoTexture();
  bool ret = true;
  for( size_t i = 0; i < frames.size(); i++ ) {
    auto& frame = frames[ i ];
    assert( frame.getWidth() == context.getWidth() && frame.getHeight() == context.getHeight() );
    sources[ i ].transfertColors( reconstructs[ i ],
                                  int32_t( params_.bestColorSearchRange_ ),
                                  params_.losslessTexture_ == 1 );
    ret &= generateTextureVideo( reconstructs[ i ], frame, videoTexture, 2 );
  }
  return ret; 
}

bool PCCEncoder::generateTextureVideo( const PCCPointSet3& reconstruct, PCCFrameContext& frame,
                                       PCCVideo3B &video, const size_t frameCount ) {
  auto& pointToPixel = frame.getPointToPixel();
  const size_t pointCount = reconstruct.getPointCount();
  if (!pointCount || !reconstruct.hasColors()) {
    return false;
  }
  const size_t shift = video.getFrameCount();
  video.resize( shift + frameCount );
  for (size_t f = 0; f < frameCount; ++f) {
    auto &image = video.getFrame( f + shift );
    image.resize( frame.getWidth(), frame.getHeight() );
    image.set(0);
  }
  for (size_t i = 0; i < pointCount; ++i) {
    const PCCVector3<size_t> location = pointToPixel[i];
    const PCCColor3B color = reconstruct.getColor(i);
    const size_t u = location[0];
    const size_t v = location[1];
    const size_t f = location[2];
    auto &image = video.getFrame(f + shift);
    image.setValue(0, u, v, color[0]);
    image.setValue(1, u, v, color[1]);
    image.setValue(2, u, v, color[2]);
  }
  return true;
}
int PCCEncoder::compressHeader( PCCContext &context, pcc::PCCBitstream &bitstream ){
  bitstream.write<uint8_t>( (uint8_t)( context.size() ) );
  if (!context.size()) {
    return 0;
  }
  bitstream.write<uint16_t>(uint16_t( context.getWidth () ) );
  bitstream.write<uint16_t>(uint16_t( context.getHeight() ) );
  bitstream.write<uint8_t> (uint8_t(params_.occupancyResolution_));
  bitstream.write<uint8_t> (uint8_t(params_.radius2Smoothing_));
  bitstream.write<uint8_t> (uint8_t(params_.neighborCountSmoothing_));
  bitstream.write<uint8_t> (uint8_t(params_.radius2BoundaryDetection_));
  bitstream.write<uint8_t> (uint8_t(params_.thresholdSmoothing_));
  bitstream.write<uint8_t> (uint8_t(params_.losslessGeo_));
  bitstream.write<uint8_t> (uint8_t(params_.losslessTexture_));
  bitstream.write<uint8_t> (uint8_t(params_.noAttributes_));
  return 1;
}

void PCCEncoder::compressOccupancyMap( PCCContext &context, PCCBitstream& bitstream ){
  for( auto& frame : context.getFrames() ){
    compressOccupancyMap( frame, bitstream );
  }
}
void PCCEncoder::compressOccupancyMap( PCCFrameContext& frame, PCCBitstream &bitstream ) {
  auto& patches = frame.getPatches();
  const size_t patchCount = patches.size();
  bitstream.write<uint32_t>(uint32_t(patchCount));
  bitstream.write<uint8_t> (uint8_t(params_.occupancyPrecision_));
  bitstream.write<uint8_t> (uint8_t(params_.maxCandidateCount_));
  size_t maxU0 = 0;
  size_t maxV0 = 0;
  size_t maxU1 = 0;
  size_t maxV1 = 0;
  size_t maxD1 = 0;
  for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
    const auto &patch = patches[patchIndex];
    maxU0 = (std::max)(maxU0, patch.getU0());
#if PCC_CORRECT_MAX_V0
    maxV0 = (std::max)(maxV0, patch.getV0());
#else
    maxV0 = (std::max)(maxU0, patch.getV0());
#endif
    maxU1 = (std::max)(maxU1, patch.getU1());
    maxV1 = (std::max)(maxV1, patch.getV1());
    maxD1 = (std::max)(maxD1, patch.getD1());
  }
  const uint8_t bitCountU0 =
      uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxU0 + 1)));
  const uint8_t bitCountV0 =
      uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxV0 + 1)));
  const uint8_t bitCountU1 =
      uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxU1 + 1)));
  const uint8_t bitCountV1 =
      uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxV1 + 1)));
  const uint8_t bitCountD1 =
      uint8_t(PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(maxD1 + 1)));

  bitstream.write<uint8_t>(bitCountU0);
  bitstream.write<uint8_t>(bitCountV0);
  bitstream.write<uint8_t>(bitCountU1);
  bitstream.write<uint8_t>(bitCountV1);
  bitstream.write<uint8_t>(bitCountD1);
  PCCBistreamPosition startPosition = bitstream.getPosition();
  bitstream += (uint64_t)4;  // placehoder for bitstream size

  o3dgc::Arithmetic_Codec arithmeticEncoder;
  arithmeticEncoder.set_buffer(uint32_t(bitstream.capacity() - bitstream.size()),
                               bitstream.buffer() + bitstream.size());
  arithmeticEncoder.start_encoder();
  o3dgc::Static_Bit_Model bModel0;
  o3dgc::Adaptive_Bit_Model bModelSizeU0, bModelSizeV0;
  o3dgc::Adaptive_Data_Model orientationModel(4);
  int64_t prevSizeU0 = 0;
  int64_t prevSizeV0 = 0;
  for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
    const auto &patch = patches[patchIndex];
    EncodeUInt32(uint32_t(patch.getU0()), bitCountU0, arithmeticEncoder, bModel0);
    EncodeUInt32(uint32_t(patch.getV0()), bitCountV0, arithmeticEncoder, bModel0);
    EncodeUInt32(uint32_t(patch.getU1()), bitCountU1, arithmeticEncoder, bModel0);
    EncodeUInt32(uint32_t(patch.getV1()), bitCountV1, arithmeticEncoder, bModel0);
    EncodeUInt32(uint32_t(patch.getD1()), bitCountD1, arithmeticEncoder, bModel0);
    const int64_t deltaSizeU0 = static_cast<int64_t>(patch.getSizeU0()) - prevSizeU0;
    const int64_t deltaSizeV0 =
        patchIndex == 0 ? patch.getSizeV0() : prevSizeV0 - static_cast<int64_t>(patch.getSizeV0());
    assert(deltaSizeV0 >= 0);
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(deltaSizeU0)), 0, bModel0,
                                      bModelSizeU0);
    arithmeticEncoder.ExpGolombEncode(int32_t(deltaSizeV0), 0, bModel0, bModelSizeV0);
    prevSizeU0 = patch.getSizeU0();
    prevSizeV0 = patch.getSizeV0();
    arithmeticEncoder.encode(uint32_t(patch.getNormalAxis()), orientationModel);
  }
  const size_t blockToPatchWidth  = frame.getWidth()  / params_.occupancyResolution_;
  const size_t blockToPatchHeight = frame.getHeight() / params_.occupancyResolution_;
  const size_t blockCount = blockToPatchWidth * blockToPatchHeight;
  auto& blockToPatch = frame.getBlockToPatch();
  blockToPatch.resize(0);
  blockToPatch.resize( blockCount, 0 );
  for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
    const auto &patch = patches[patchIndex];
    const auto& occupancy = patch.getOccupancy();
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        if ( occupancy[v0 * patch.getSizeU0() + u0]) {
          blockToPatch[(v0 + patch.getV0()) * blockToPatchWidth + (u0 + patch.getU0())] = patchIndex + 1;
        }
      }
    }
  }
  std::vector<std::vector<size_t>> candidatePatches;
  candidatePatches.resize(blockCount);
  for (int64_t patchIndex = patchCount - 1; patchIndex >= 0; --patchIndex) {  // add actual patches based on their bounding box
    const auto &patch = patches[patchIndex];
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        candidatePatches[(patch.getV0() + v0) * blockToPatchWidth + (patch.getU0() + u0)].push_back( patchIndex + 1 );
      }
    }
  }
  for (auto &candidatePatch : candidatePatches) {  // add empty as potential candidate
    candidatePatch.push_back(0);
  }
  o3dgc::Adaptive_Data_Model candidateIndexModel(uint32_t( params_.maxCandidateCount_ + 2));
  const uint32_t bitCountPatchIndex =
      PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(patchCount + 1));
  for (size_t p = 0; p < blockCount; ++p) {
    const size_t patchIndex = blockToPatch[p];
    const auto &candidates = candidatePatches[p];
    if (candidates.size() == 1) {
      // empty
    } else {
      const uint32_t candidateCount = uint32_t( (std::min)(candidates.size(), params_.maxCandidateCount_));
      bool found = false;
      for (uint32_t i = 0; i < candidateCount; ++i) {
        if (candidates[i] == patchIndex) {
          found = true;
          arithmeticEncoder.encode(i, candidateIndexModel);
          break;
        }
      }
      if (!found) {
        arithmeticEncoder.encode(uint32_t(params_.maxCandidateCount_), candidateIndexModel);
        EncodeUInt32(uint32_t(patchIndex), bitCountPatchIndex, arithmeticEncoder, bModel0);
      }
    }
  }

  const size_t blockSize0 = params_.occupancyResolution_ / params_.occupancyPrecision_;
  const size_t pointCount0 = blockSize0 * blockSize0;
  const size_t traversalOrderCount = 4;
  std::vector<std::vector<std::pair<size_t, size_t>>> traversalOrders;
  traversalOrders.resize(traversalOrderCount);
  for (size_t k = 0; k < traversalOrderCount; ++k) {
    auto &traversalOrder = traversalOrders[k];
    traversalOrder.reserve(pointCount0);
    if (k == 0) {
      for (size_t v1 = 0; v1 < blockSize0; ++v1) {
        for (size_t u1 = 0; u1 < blockSize0; ++u1) {
          traversalOrder.push_back(std::make_pair(u1, v1));
        }
      }
    } else if (k == 1) {
      for (size_t v1 = 0; v1 < blockSize0; ++v1) {
        for (size_t u1 = 0; u1 < blockSize0; ++u1) {
          traversalOrder.push_back(std::make_pair(v1, u1));
        }
      }
    } else if (k == 2) {
      for (int64_t k = 1; k < int64_t(2 * blockSize0); ++k) {
        for (size_t u1 = (std::max)(int64_t(0), k - int64_t(blockSize0));
            int64_t(u1) < (std::min)(k, int64_t(blockSize0)); ++u1) {
          const size_t v1 = k - (u1 + 1);
          traversalOrder.push_back(std::make_pair(u1, v1));
        }
      }
    } else {
      for (int64_t k = 1; k < int64_t(2 * blockSize0); ++k) {
        for (size_t u1 = (std::max)(int64_t(0), k - int64_t(blockSize0));
            int64_t(u1) < (std::min)(k, int64_t(blockSize0)); ++u1) {
          const size_t v1 = k - (u1 + 1);
          traversalOrder.push_back(std::make_pair(blockSize0 - (1 + u1), v1));
        }
      }
    }
  }
  o3dgc::Adaptive_Bit_Model fullBlockModel, occupancyModel;
  o3dgc::Adaptive_Data_Model traversalOrderIndexModel(uint32_t(traversalOrderCount + 1));
  o3dgc::Adaptive_Data_Model runCountModel((uint32_t)(pointCount0));
  o3dgc::Adaptive_Data_Model runLengthModel((uint32_t)(pointCount0));

  std::vector<bool> block0;
  std::vector<size_t> bestRuns;
  std::vector<size_t> runs;
  block0.resize(pointCount0);
  auto& width = frame.getWidth();
  auto& occupancyMap = frame.getOccupancyMap();
  for (size_t v0 = 0; v0 < blockToPatchHeight; ++v0) {
    for (size_t u0 = 0; u0 < blockToPatchWidth; ++u0) {
      const size_t patchIndex = blockToPatch[v0 * blockToPatchWidth + u0];
      if (patchIndex) {
        size_t fullCount = 0;
        for (size_t v1 = 0; v1 < blockSize0; ++v1) {
          const size_t v2 = v0 * params_.occupancyResolution_ + v1 * params_.occupancyPrecision_;
          for (size_t u1 = 0; u1 < blockSize0; ++u1) {
            const size_t u2 = u0 * params_.occupancyResolution_ + u1 * params_.occupancyPrecision_;
            bool isFull = false;
            for (size_t v3 = 0; v3 < params_.occupancyPrecision_ && !isFull; ++v3) {
              for (size_t u3 = 0; u3 < params_.occupancyPrecision_ && !isFull; ++u3) {
                isFull |= occupancyMap[(v2 + v3) * width + u2 + u3] == 1;
              }
            }
            block0[v1 * blockSize0 + u1] = isFull;
            fullCount += isFull;
            for (size_t v3 = 0; v3 < params_.occupancyPrecision_; ++v3) {
              for (size_t u3 = 0; u3 < params_.occupancyPrecision_; ++u3) {
                occupancyMap[(v2 + v3) * width + u2 + u3] = isFull;
              }
            }
          }
        }

        if (fullCount == pointCount0) {
          arithmeticEncoder.encode(true, fullBlockModel);
        } else {
          arithmeticEncoder.encode(false, fullBlockModel);
          bestRuns.clear();
          size_t bestTraversalOrderIndex = 0;
          for (size_t k = 0; k < traversalOrderCount; ++k) {
            auto &traversalOrder = traversalOrders[k];
            const auto &location0 = traversalOrder[0];
            bool occupancy0 = block0[location0.second * blockSize0 + location0.first];
            size_t runLength = 0;
            runs.clear();
            for (size_t p = 1; p < traversalOrder.size(); ++p) {
              const auto &location = traversalOrder[p];
              const bool occupancy1 = block0[location.second * blockSize0 + location.first];
              if (occupancy1 != occupancy0) {
                runs.push_back(runLength);
                occupancy0 = occupancy1;
                runLength = 0;
              } else {
                ++runLength;
              }
            }
            runs.push_back(runLength);
            if (k == 0 || runs.size() < bestRuns.size()) {
              bestRuns = runs;
              bestTraversalOrderIndex = k;
            }
          }

          assert(bestRuns.size() >= 2);
          const uint32_t runCountMinusOne = uint32_t(bestRuns.size() - 1);
          const uint32_t runCountMinusTwo = uint32_t(bestRuns.size() - 2);
          arithmeticEncoder.encode(uint32_t(bestTraversalOrderIndex), traversalOrderIndexModel);
          arithmeticEncoder.encode(runCountMinusTwo, runCountModel);
          const auto &location0 = traversalOrders[bestTraversalOrderIndex][0];
          bool occupancy0 = block0[location0.second * blockSize0 + location0.first];
          arithmeticEncoder.encode(occupancy0, occupancyModel);
          for (size_t r = 0; r < runCountMinusOne; ++r) {
            arithmeticEncoder.encode(uint32_t(bestRuns[r]), runLengthModel);
          }
        }
      }
    }
  }
  uint32_t compressedBitstreamSize = arithmeticEncoder.stop_encoder();
  bitstream += (uint64_t) compressedBitstreamSize;
  bitstream.write<uint32_t>(compressedBitstreamSize, startPosition );
}
