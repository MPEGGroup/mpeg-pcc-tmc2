
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
#include "ArithmeticCodec.h"
#include "PCCPatch.h"
#include "PCCVideoDecoder.h"
#include "PCCGroupOfFrames.h"
#include <tbb/tbb.h>

#include "PCCDecoder.h"

using namespace pcc;
using namespace std;

uint32_t DecodeUInt32(const uint32_t bitCount, o3dgc::Arithmetic_Codec &arithmeticDecoder,
                      o3dgc::Static_Bit_Model &bModel0) {
  uint32_t decodedValue = 0;
  for (uint32_t i = 0; i < bitCount; ++i) {
    decodedValue += (arithmeticDecoder.decode(bModel0) << i);
  }
  return PCCFromLittleEndian<uint32_t>(decodedValue);
}

PCCDecoder::PCCDecoder(){
}
PCCDecoder::~PCCDecoder(){
}

void PCCDecoder::setParameters( PCCDecoderParameters params ) { 
  params_ = params; 
}

int PCCDecoder::decode( PCCBitstream &bitstream, PCCContext &context, PCCGroupOfFrames& reconstructs ){
  if (!decompressHeader( context, bitstream ) ) {
    return 0;
  }
  if( params_.nbThread_ > 0 ) {
    tbb::task_scheduler_init init( (int)params_.nbThread_ );
  }
  reconstructs.resize( context.size() );
  PCCVideoDecoder videoDecoder; 
  std::stringstream path;
  path << removeFileExtension( params_.compressedStreamPath_ ) << "_dec_GOF" << context.getIndex() << "_";

  const size_t nbyteGeo = losslessGeo_ ? 2 : 1;
  if (!absoluteD1_) {
    // Compress D0
    auto sizeGeometryD0Video = bitstream.size();
    videoDecoder.decompress( context.getVideoGeometry(), path.str() + "geometryD0", width_, height_,
                             context.size(), bitstream, params_.videoDecoderPath_,
                             "", "", (losslessGeo_?losslessGeo444_:false), nbyteGeo,
                             params_.keepIntermediateFiles_ );
    sizeGeometryD0Video = bitstream.size() - sizeGeometryD0Video;
    std::cout << "geometry D0 video ->" << sizeGeometryD0Video << " B" << std::endl;

    // Compress D1
    auto sizeGeometryD1Video = bitstream.size();
    videoDecoder.decompress(context.getVideoGeometryD1(), path.str() + "geometryD1", width_, height_,
                            context.size(), bitstream, params_.videoDecoderPath_,
                            "", "", (losslessGeo_?losslessGeo444_:false), nbyteGeo,
                            params_.keepIntermediateFiles_ );
    sizeGeometryD1Video = bitstream.size() - sizeGeometryD1Video;

    std::cout << "geometry D1 video ->" << sizeGeometryD1Video << " B" << std::endl;
    std::cout << "geometry video ->" << sizeGeometryD0Video + sizeGeometryD1Video << " B" << std::endl;
  }
  else {
    auto sizeGeometryVideo = bitstream.size();
    videoDecoder.decompress(context.getVideoGeometry(), path.str() + "geometry", width_, height_,
                            context.size() * 2, bitstream, params_.videoDecoderPath_,
                            "", "", losslessGeo_ & losslessGeo444_, nbyteGeo,
                            params_.keepIntermediateFiles_);
    sizeGeometryVideo = bitstream.size() - sizeGeometryVideo;
    std::cout << "geometry video ->" << sizeGeometryVideo << " B" << std::endl;
  }
  auto sizeOccupancyMap = bitstream.size();
  decompressOccupancyMap( context, bitstream );
  sizeOccupancyMap = bitstream.size() - sizeOccupancyMap;
  std::cout << "occupancy map  ->" << sizeOccupancyMap << " B" << std::endl;

  GeneratePointCloudParameters generatePointCloudParameters = {
      occupancyResolution_,
      neighborCountSmoothing_,
      (double)radius2Smoothing_,
      (double)radius2BoundaryDetection_,
      (double)thresholdSmoothing_,
      losslessGeo_ != 0,
      losslessGeo444_ != 0,
      params_.nbThread_,
      absoluteD1_,
      binArithCoding_
  };
  generatePointCloud( reconstructs, context, generatePointCloudParameters );

  if (!noAttributes_ ) {
    auto sizeTextureVideo = bitstream.size();
	  const size_t nbyteTexture = 1;
    videoDecoder.decompress( context.getVideoTexture(),
                             path.str() + "texture", width_, height_,
                             context.size() * 2, bitstream,
                             params_.videoDecoderPath_,
                             params_.inverseColorSpaceConversionConfig_,
                             params_.colorSpaceConversionPath_,
                             losslessTexture_ != 0, nbyteTexture,
                             params_.keepIntermediateFiles_ );
    sizeTextureVideo = bitstream.size() - sizeTextureVideo;
    std::cout << "texture video  ->" << sizeTextureVideo << " B" << std::endl;
  }
  colorPointCloud( reconstructs, context, noAttributes_ != 0, params_.colorTransform_ );
  return 0;
}

int PCCDecoder::decompressHeader( PCCContext &context, PCCBitstream &bitstream ){
  uint8_t groupOfFramesSize;
  bitstream.read<uint8_t>( groupOfFramesSize );
  if (!groupOfFramesSize) {
    return 0;
  }
  context.resize( groupOfFramesSize );
  bitstream.read<uint16_t>( width_ );
  bitstream.read<uint16_t>( height_ );
  bitstream.read<uint8_t> ( occupancyResolution_ );
  bitstream.read<uint8_t> ( radius2Smoothing_ );
  bitstream.read<uint8_t> ( neighborCountSmoothing_ );
  bitstream.read<uint8_t> ( radius2BoundaryDetection_ );
  bitstream.read<uint8_t> ( thresholdSmoothing_ );
  bitstream.read<uint8_t> ( losslessGeo_ );
  bitstream.read<uint8_t> ( losslessTexture_ );
  bitstream.read<uint8_t> ( noAttributes_ );
  bitstream.read<uint8_t> ( losslessGeo444_);
  uint8_t absD1, binArithCoding;
  bitstream.read<uint8_t> ( absD1 );
  absoluteD1_ = absD1 > 0;
  bitstream.read<uint8_t> ( binArithCoding );
  binArithCoding_ =  binArithCoding > 0;
  context.getWidth()  = width_;
  context.getHeight() = height_; 
  return 1;
}

void PCCDecoder::decompressOccupancyMap( PCCContext &context, PCCBitstream& bitstream ){
  for( auto& frame : context.getFrames() ){
    frame.getWidth () = width_;
    frame.getHeight() = height_;
    decompressOccupancyMap( frame, bitstream);
    auto&  patches = frame.getPatches();
    auto& missedPointsPatch = frame.getMissedPointsPatch();
    if (losslessGeo_) {
      const size_t patchIndex = patches.size();
      PCCPatch &dummyPatch = patches[patchIndex - 1];
      missedPointsPatch.u0 = dummyPatch.getU0();
      missedPointsPatch.v0 = dummyPatch.getV0();
      missedPointsPatch.sizeU0 = dummyPatch.getSizeU0();
      missedPointsPatch.sizeV0 = dummyPatch.getSizeV0();
      missedPointsPatch.occupancyResolution = dummyPatch.getOccupancyResolution();
      patches.pop_back();
    }
  }
}

void PCCDecoder::decompressOccupancyMap( PCCFrameContext& frame, PCCBitstream &bitstream ) {
  uint32_t patchCount = 0;
  auto&  patches = frame.getPatches();
  bitstream.read<uint32_t>( patchCount );
  patches.resize( patchCount );

  size_t occupancyPrecision = 0;
  {
    uint8_t precision = 0;
    bitstream.read<uint8_t>( precision );
    occupancyPrecision = precision;
  }
  size_t maxCandidateCount = 0;
  {
    uint8_t count = 0;
    bitstream.read<uint8_t>( count );
    maxCandidateCount = count;
  }

  uint8_t bitCountU0 = 0;
  uint8_t bitCountV0 = 0;
  uint8_t bitCountU1 = 0;
  uint8_t bitCountV1 = 0;
  uint8_t bitCountD1 = 0;
  bitstream.read<uint8_t>( bitCountU0 );
  bitstream.read<uint8_t>( bitCountV0 );
  bitstream.read<uint8_t>( bitCountU1 );
  bitstream.read<uint8_t>( bitCountV1 );
  bitstream.read<uint8_t>( bitCountD1 );
  uint32_t compressedBitstreamSize;
  bitstream.read<uint32_t>(  compressedBitstreamSize );
  assert(compressedBitstreamSize + bitstream.size() <= bitstream.capacity());
  o3dgc::Arithmetic_Codec arithmeticDecoder;
  arithmeticDecoder.set_buffer(uint32_t(bitstream.capacity() - bitstream.size()),
                               bitstream.buffer() + bitstream.size());

  bool bBinArithCoding = binArithCoding_ && (!losslessGeo_) &&
                         (occupancyResolution_ == 16) && (occupancyPrecision == 4);

  arithmeticDecoder.start_decoder();
  o3dgc::Static_Bit_Model bModel0;
  o3dgc::Adaptive_Bit_Model bModelSizeU0, bModelSizeV0, bModelAbsoluteD1;

  o3dgc::Adaptive_Bit_Model orientationModel2;

  o3dgc::Adaptive_Data_Model orientationModel(4);
  int64_t prevSizeU0 = 0;
  int64_t prevSizeV0 = 0;

  // absoluteD1_ = (bool)arithmeticDecoder.decode( bModelAbsoluteD1 );
  for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
    auto &patch = patches[patchIndex];
    patch.getOccupancyResolution() = occupancyResolution_;

    patch.getU0() = DecodeUInt32(bitCountU0, arithmeticDecoder, bModel0);
    patch.getV0() = DecodeUInt32(bitCountV0, arithmeticDecoder, bModel0);
    patch.getU1() = DecodeUInt32(bitCountU1, arithmeticDecoder, bModel0);
    patch.getV1() = DecodeUInt32(bitCountV1, arithmeticDecoder, bModel0);
    patch.getD1() = DecodeUInt32(bitCountD1, arithmeticDecoder, bModel0);

    const int64_t deltaSizeU0 =
        o3dgc::UIntToInt(arithmeticDecoder.ExpGolombDecode(0, bModel0, bModelSizeU0));
    const int64_t deltaSizeV0 = 
        o3dgc::UIntToInt(arithmeticDecoder.ExpGolombDecode(0, bModel0, bModelSizeV0));

    patch.getSizeU0() = prevSizeU0 + deltaSizeU0;
    patch.getSizeV0() = prevSizeV0 + deltaSizeV0;

    prevSizeU0 = patch.getSizeU0();
    prevSizeV0 = patch.getSizeV0();

    if (bBinArithCoding) {
      size_t bit0 = arithmeticDecoder.decode(orientationModel2);
      if (bit0 == 0) {  // 0
        patch.getNormalAxis() = 0;
      }
      else {
        size_t bit1 = arithmeticDecoder.decode(bModel0);
        if (bit1 == 0) { // 10
          patch.getNormalAxis() = 1;
        }
        else { // 11
          patch.getNormalAxis() = 2;
        }
      }
    }
    else {
      patch.getNormalAxis() = arithmeticDecoder.decode(orientationModel);
    }

    if (patch.getNormalAxis() == 0) {
      patch.getTangentAxis() = 2;
      patch.getBitangentAxis() = 1;
    } else if (patch.getNormalAxis() == 1) {
      patch.getTangentAxis() = 2;
      patch.getBitangentAxis() = 0;
    } else {
      patch.getTangentAxis() = 0;
      patch.getBitangentAxis() = 1;
    }
  }
  const size_t blockToPatchWidth  = width_  / occupancyResolution_;
  const size_t blockToPatchHeight = height_ / occupancyResolution_;
  const size_t blockCount = blockToPatchWidth * blockToPatchHeight;

  std::vector<std::vector<size_t>> candidatePatches;
  candidatePatches.resize(blockCount);
  for (int64_t patchIndex = patchCount - 1; patchIndex >= 0;
      --patchIndex) {  // add actual patches based on their bounding box
    const auto &patch = patches[patchIndex];
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        candidatePatches[(patch.getV0() + v0) * blockToPatchWidth + (patch.getU0() + u0)].push_back(
            patchIndex + 1);
      }
    }
  }
  for (auto &candidatePatch : candidatePatches) {  // add empty as potential candidate
    candidatePatch.push_back(0);
  }

  auto& blockToPatch = frame.getBlockToPatch();
  blockToPatch.resize(0);
  blockToPatch.resize(blockCount, 0);

  o3dgc::Adaptive_Bit_Model candidateIndexModelBit[4];

  o3dgc::Adaptive_Data_Model candidateIndexModel(uint32_t(maxCandidateCount + 2));
  const uint32_t bitCountPatchIndex = PCCGetNumberOfBitsInFixedLengthRepresentation(patchCount + 1);
  for (size_t p = 0; p < blockCount; ++p) {
    const auto &candidates = candidatePatches[p];
    if (candidates.size() == 1) {
      blockToPatch[p] = candidates[0];
    } else {
      size_t candidateIndex;
      if (bBinArithCoding) {
        size_t bit0 = arithmeticDecoder.decode(candidateIndexModelBit[0]);
        if (bit0 == 0) {
          candidateIndex = 0; // Codeword: 0
        } else {
          size_t bit1 = arithmeticDecoder.decode(candidateIndexModelBit[1]);
          if (bit1 == 0) {
            candidateIndex = 1; // Codeword 10
          } else {
            size_t bit2 = arithmeticDecoder.decode(candidateIndexModelBit[2]);
            if (bit2 == 0) {
              candidateIndex = 2; // Codeword 110
            } else {
              size_t bit3 = arithmeticDecoder.decode(candidateIndexModelBit[3]);
              if (bit3 == 0) {
                candidateIndex = 3; // Codeword 1110
              } else {
                candidateIndex = 4; // Codeword 11110
              }
            }
          }
        }
      } else {
        candidateIndex = arithmeticDecoder.decode(candidateIndexModel);
      }

      if (candidateIndex == maxCandidateCount) {
        blockToPatch[p] = DecodeUInt32(bitCountPatchIndex, arithmeticDecoder, bModel0);
      } else {
        blockToPatch[p] = candidates[candidateIndex];
      }
    }
  }

  const size_t blockSize0 = occupancyResolution_ / occupancyPrecision;
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
        for (int64_t u1 = (std::max)(int64_t(0), k - int64_t(blockSize0));
            u1 < (std::min)(k, int64_t(blockSize0)); ++u1) {
          const size_t v1 = k - (u1 + 1);
          traversalOrder.push_back(std::make_pair(u1, v1));
        }
      }
    } else {
      for (int64_t k = 1; k < int64_t(2 * blockSize0); ++k) {
        for (int64_t u1 = (std::max)(int64_t(0), k - int64_t(blockSize0));
            u1 < (std::min)(k, int64_t(blockSize0)); ++u1) {
          const size_t v1 = k - (u1 + 1);
          traversalOrder.push_back(std::make_pair(blockSize0 - (1 + u1), v1));
        }
      }
    }
  }
  o3dgc::Adaptive_Bit_Model fullBlockModel, occupancyModel;

  o3dgc::Adaptive_Bit_Model traversalOrderIndexModel_Bit0;
  o3dgc::Adaptive_Bit_Model traversalOrderIndexModel_Bit1;
  o3dgc::Adaptive_Bit_Model runCountModel2;
  o3dgc::Adaptive_Bit_Model runLengthModel2[4];
  static size_t runLengthInvTable[16] = { 0,  1,  2,  3,  7,  11,  14,  5,  13,  9,  6,  10,  12,  4,  8, 15 };

  o3dgc::Adaptive_Data_Model traversalOrderIndexModel(uint32_t(traversalOrderCount + 1));
  o3dgc::Adaptive_Data_Model runCountModel((uint32_t)(pointCount0));
  o3dgc::Adaptive_Data_Model runLengthModel((uint32_t)(pointCount0));

  std::vector<uint32_t> block0;
  std::vector<size_t> bestRuns;
  std::vector<size_t> runs;
  block0.resize(pointCount0);
  auto& occupancyMap = frame.getOccupancyMap();
  occupancyMap.resize( width_ * height_, 0 );
  for (size_t v0 = 0; v0 < blockToPatchHeight; ++v0) {
    for (size_t u0 = 0; u0 < blockToPatchWidth; ++u0) {
      const size_t patchIndex = blockToPatch[v0 * blockToPatchWidth + u0];
      if (patchIndex) {
        const bool isFull = arithmeticDecoder.decode(fullBlockModel) != 0;
        if (isFull) {
          for (auto &occupancy : block0) {
            occupancy = true;
          }
        } else {
          size_t bestTraversalOrderIndex;
          if (bBinArithCoding) {
            size_t bit1 = arithmeticDecoder.decode(traversalOrderIndexModel_Bit1);
            size_t bit0 = arithmeticDecoder.decode(traversalOrderIndexModel_Bit0);
            bestTraversalOrderIndex = (bit1 << 1) + bit0;
          }
          else {
            bestTraversalOrderIndex = arithmeticDecoder.decode(traversalOrderIndexModel);
          }
          const auto &traversalOrder = traversalOrders[bestTraversalOrderIndex];
          int64_t runCountMinusTwo;
          if (bBinArithCoding) {
            runCountMinusTwo = arithmeticDecoder.ExpGolombDecode(0, bModel0, runCountModel2);
          } else {
            runCountMinusTwo = arithmeticDecoder.decode(runCountModel);
          }

          const size_t runCountMinusOne = runCountMinusTwo + 1;
          size_t i = 0;
          bool occupancy = arithmeticDecoder.decode(occupancyModel) != 0;
          for (size_t r = 0; r < runCountMinusOne; ++r) {
            size_t runLength;
            if (bBinArithCoding) {
              size_t bit3 = arithmeticDecoder.decode(runLengthModel2[3]);
              size_t bit2 = arithmeticDecoder.decode(runLengthModel2[2]);
              size_t bit1 = arithmeticDecoder.decode(runLengthModel2[1]);
              size_t bit0 = arithmeticDecoder.decode(runLengthModel2[0]);
              const size_t runLengthIdx = (bit3 << 3) + (bit2 << 2) + (bit1 << 1) + bit0;
              runLength = runLengthInvTable[runLengthIdx];
            } else {
              runLength = arithmeticDecoder.decode(runLengthModel);
            }

            for (size_t j = 0; j <= runLength; ++j) {
              const auto &location = traversalOrder[i++];
              block0[location.second * blockSize0 + location.first] = occupancy;
            }
            occupancy = !occupancy;
          }
          for (size_t j = i; j < pointCount0; ++j) {
            const auto &location = traversalOrder[j];
            block0[location.second * blockSize0 + location.first] = occupancy;
          }
        }

        for (size_t v1 = 0; v1 < blockSize0; ++v1) {
          const size_t v2 = v0 * occupancyResolution_ + v1 * occupancyPrecision;
          for (size_t u1 = 0; u1 < blockSize0; ++u1) {
            const size_t u2 = u0 * occupancyResolution_ + u1 * occupancyPrecision;
            const bool occupancy = block0[v1 * blockSize0 + u1] != 0;
            for (size_t v3 = 0; v3 < occupancyPrecision; ++v3) {
              for (size_t u3 = 0; u3 < occupancyPrecision; ++u3) {
                occupancyMap[(v2 + v3) * width_ + u2 + u3] = occupancy;
              }
            }
          }
        }
      }
    }
  }
  arithmeticDecoder.stop_decoder();
  bitstream += (uint64_t)compressedBitstreamSize;
}

