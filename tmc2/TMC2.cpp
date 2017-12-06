/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * <OWNER> = Apple Inc.
 * <ORGANIZATION> = Apple Inc.
 * <YEAR> = 2017
 *
 * Copyright (c) 2017, Apple Inc.
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
 *  * Neither the name of the <ORGANIZATION> nor the names of its contributors may
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

#include "TMC2.h"

using namespace std;
using namespace pcc;
using namespace nanoflann;

int main(int argc, char *argv[]) {
  Parameters params;
  Usage();
  if (!ParseParameters(argc, argv, params)) {
    std::cout << "Error: missing parameters!" << std::endl;
    return -1;
  }
  const auto start = std::chrono::high_resolution_clock::now();
  int ret = 0;
  if (params.mode == CODEC_MODE_ENCODE) {
    ret = CompressVideo(params);
  } else {
    ret = DecompressVideo(params);
  }

  const auto end = std::chrono::high_resolution_clock::now();
  std::cout << "Processing time: "
            << std::chrono::duration<double, std::milli>(end - start).count() / 1000.0 << " s"
            << std::endl;
  return ret;
}
void Usage() {
  std::cout << "tmc2 v" << TMC2_VERSION_MAJOR << "." << TMC2_VERSION_MAJOR << std::endl
            << std::endl;

  std::cout << "+ Usage" << std::endl;
  std::cout << "\t Encode example: \n tmc2 --mode 0 --geometryQP 27 --textureQP "
               "43 --uncompressedDataPath ../longdress/longdress_vox10_%i.ply "
               "--compressedStreamPath longdress.bin --startFrameNumber 1051 --groupOfFramesSize "
               "32 --frameCount 32 --videoEncoderPath \"../app/TAppEncoder\" "
               "--colorSpaceConversionPath \"../app/HDRConvert\" "
               "--inverseColorSpaceConversionConfig \"../app/yuv420torgb444.cfg\" "
               "--colorSpaceConversionConfig \"../app/rgb444toyuv420.cfg\" --geometryConfig "
               "\"../app/geometry.cfg\" --textureConfig \"../app/texture.cfg\""
            << std::endl
            << std::endl;
  std::cout << "\t Decode example: \n tmc2 --mode 1 --reconstructedDataPath "
               "../dec/P07S24C04R01/P07S24C04R01_%i.ply --compressedStreamPath longdress.bin "
               "--startFrameNumber 1051  --videoDecoderPath \"../app/TAppDecoder\"  "
               "--colorSpaceConversionPath \"../app/HDRConvert\" "
               "--inverseColorSpaceConversionConfig \"../app/yuv420torgb444.cfg\" "
               "--colorSpaceConversionConfig \"../app/rgb444toyuv420.cfg\""
            << std::endl
            << std::endl;
  std::cout << std::endl;
}

bool ParseParameters(int argc, char *argv[], Parameters &params) {
  for (int i = 1; i < argc; ++i) {
    if (!strcmp(argv[i], "--mode")) {
      if (++i < argc) params.mode = static_cast<CodecMode>(atoi(argv[i]));
    }

    else if (!strcmp(argv[i], "--colorSpaceConversionPath")) {
      if (++i < argc) params.colorSpaceConversionPath = argv[i];
    } else if (!strcmp(argv[i], "--videoEncoderPath")) {
      if (++i < argc) params.videoEncoderPath = argv[i];
    } else if (!strcmp(argv[i], "--videoDecoderPath")) {
      if (++i < argc) params.videoDecoderPath = argv[i];
    } else if (!strcmp(argv[i], "--uncompressedDataPath")) {
      if (++i < argc) params.uncompressedDataPath = argv[i];
    } else if (!strcmp(argv[i], "--compressedStreamPath")) {
      if (++i < argc) params.compressedStreamPath = argv[i];
    } else if (!strcmp(argv[i], "--reconstructedDataPath")) {
      if (++i < argc) params.reconstructedDataPath = argv[i];
    } else if (!strcmp(argv[i], "--frameCount")) {
      if (++i < argc) params.frameCount = atoi(argv[i]);
    } else if (!strcmp(argv[i], "--startFrameNumber")) {
      if (++i < argc) params.startFrameNumber = atoi(argv[i]);
    } else if (!strcmp(argv[i], "--groupOfFramesSize")) {
      if (++i < argc) params.groupOfFramesSize = atoi(argv[i]);
    } else if (!strcmp(argv[i], "--colorTransform")) {
      if (++i < argc) params.colorTransform = static_cast<ColorTransform>(atoi(argv[i]));
    } else if (!strcmp(argv[i], "--nnNormalEstimation")) {
      if (++i < argc) params.nnNormalEstimation = atoi(argv[i]);
    } else if (!strcmp(argv[i], "--maxNNCountRefineSegmentation")) {
      if (++i < argc) params.maxNNCountRefineSegmentation = atoi(argv[i]);
    } else if (!strcmp(argv[i], "--iterationCountRefineSegmentation")) {
      if (++i < argc) params.iterationCountRefineSegmentation = atoi(argv[i]);
    } else if (!strcmp(argv[i], "--occupancyResolution")) {
      if (++i < argc) params.occupancyResolution = atoi(argv[i]);
    } else if (!strcmp(argv[i], "--minPointCountPerCCPatchSegmentation")) {
      if (++i < argc) params.minPointCountPerCCPatchSegmentation = atoi(argv[i]);
    } else if (!strcmp(argv[i], "--maxNNCountPatchSegmentation")) {
      if (++i < argc) params.maxNNCountPatchSegmentation = atoi(argv[i]);
    } else if (!strcmp(argv[i], "--surfaceThickness")) {
      if (++i < argc) params.surfaceThickness = atoi(argv[i]);
    } else if (!strcmp(argv[i], "--maxAllowedDist2MissedPointsDetection")) {
      if (++i < argc) params.maxAllowedDist2MissedPointsDetection = atof(argv[i]);
    } else if (!strcmp(argv[i], "--maxAllowedDist2MissedPointsSelection")) {
      if (++i < argc) params.maxAllowedDist2MissedPointsSelection = atof(argv[i]);
    } else if (!strcmp(argv[i], "--lambdaRefineSegmentation")) {
      if (++i < argc) params.lambdaRefineSegmentation = atof(argv[i]);
    } else if (!strcmp(argv[i], "--minimumImageWidth")) {
      if (++i < argc) params.minimumImageWidth = atoi(argv[i]);
    } else if (!strcmp(argv[i], "--minimumImageHeight")) {
      if (++i < argc) params.minimumImageHeight = atoi(argv[i]);
    } else if (!strcmp(argv[i], "--geometryQP")) {
      if (++i < argc) params.geometryQP = atoi(argv[i]);
    } else if (!strcmp(argv[i], "--textureQP")) {
      if (++i < argc) params.textureQP = atoi(argv[i]);
    } else if (!strcmp(argv[i], "--geometryConfig")) {
      if (++i < argc) params.geometryConfig = argv[i];
    } else if (!strcmp(argv[i], "--textureConfig")) {
      if (++i < argc) params.textureConfig = argv[i];
    } else if (!strcmp(argv[i], "--inverseColorSpaceConversionConfig")) {
      if (++i < argc) params.inverseColorSpaceConversionConfig = argv[i];
    } else if (!strcmp(argv[i], "--colorSpaceConversionConfig")) {
      if (++i < argc) params.colorSpaceConversionConfig = argv[i];
    } else if (!strcmp(argv[i], "--maxCandidateCount")) {
      if (++i < argc) params.maxCandidateCount = atoi(argv[i]);
    } else if (!strcmp(argv[i], "--occupancyPrecision")) {
      if (++i < argc) params.occupancyPrecision = atoi(argv[i]);
    } else if (!strcmp(argv[i], "--neighborCountSmoothing")) {
      if (++i < argc) params.neighborCountSmoothing = atoi(argv[i]);
    } else if (!strcmp(argv[i], "--radius2Smoothing")) {
      if (++i < argc) params.radius2Smoothing = atof(argv[i]);
    } else if (!strcmp(argv[i], "--radius2BoundaryDetection")) {
      if (++i < argc) params.radius2BoundaryDetection = atof(argv[i]);
    } else if (!strcmp(argv[i], "--thresholdSmoothing")) {
      if (++i < argc) params.thresholdSmoothing = atof(argv[i]);
    } else if (!strcmp(argv[i], "--bestColorSearchRange")) {
      if (++i < argc) params.bestColorSearchRange = atoi(argv[i]);
    } else if (!strcmp(argv[i], "--maxAllowedDepth")) {
      if (++i < argc) params.maxAllowedDepth = atoi(argv[i]);
    }
  }

  if (!params.colorSpaceConversionPath.empty() &&
      !params.inverseColorSpaceConversionConfig.empty() &&
      !params.colorSpaceConversionConfig.empty()) {
    std::cout << "Info: Using external color space conversion" << std::endl;
    if (params.colorTransform != COLOR_TRANSFORM_NONE) {
      std::cout << "Warning: Using external color space conversion requires colorTransform = "
                   "COLOR_TRANSFORM_NONE!"
                << std::endl;
      params.colorTransform = COLOR_TRANSFORM_NONE;
    }
  } else {
    std::cout << "Info: Using internal color space conversion" << std::endl;
    params.colorSpaceConversionPath = "";
    params.inverseColorSpaceConversionConfig = "";
    params.colorSpaceConversionConfig = "";
  }

  std::cout << "+ Parameters" << std::endl;
  std::cout << "\t mode                        " << params.mode << std::endl;
  std::cout << "\t uncompressedDataPath        " << params.uncompressedDataPath << std::endl;
  std::cout << "\t compressedStreamPath        " << params.compressedStreamPath << std::endl;
  std::cout << "\t reconstructedDataPath       " << params.reconstructedDataPath << std::endl;
  std::cout << "\t frameCount                  " << params.frameCount << std::endl;
  std::cout << "\t startFrameNumber            " << params.startFrameNumber << std::endl;
  std::cout << "\t groupOfFramesSize           " << params.groupOfFramesSize << std::endl;
  std::cout << "\t colorTransform              " << params.colorTransform << std::endl;

  std::cout << "\t segmentation" << std::endl;
  std::cout << "\t   nnNormalEstimation                          " << params.nnNormalEstimation
            << std::endl;
  std::cout << "\t   maxNNCountRefineSegmentation                "
            << params.maxNNCountRefineSegmentation << std::endl;
  std::cout << "\t   iterationCountRefineSegmentation            "
            << params.iterationCountRefineSegmentation << std::endl;
  std::cout << "\t   occupancyResolution                         " << params.occupancyResolution
            << std::endl;
  std::cout << "\t   minPointCountPerCCPatchSegmentation         "
            << params.minPointCountPerCCPatchSegmentation << std::endl;
  std::cout << "\t   maxNNCountPatchSegmentation                 "
            << params.maxNNCountPatchSegmentation << std::endl;
  std::cout << "\t   surfaceThickness                            " << params.surfaceThickness
            << std::endl;
  std::cout << "\t   maxAllowedDist2MissedPointsDetection        "
            << params.maxAllowedDist2MissedPointsDetection << std::endl;
  std::cout << "\t   maxAllowedDist2MissedPointsSelection        "
            << params.maxAllowedDist2MissedPointsSelection << std::endl;
  std::cout << "\t   lambdaRefineSegmentation                    "
            << params.lambdaRefineSegmentation << std::endl;
  std::cout << "\t   maxAllowedDepth                             " << params.maxAllowedDepth
            << std::endl;

  std::cout << "\t packing" << std::endl;
  std::cout << "\t   minimumImageWidth                           " << params.minimumImageWidth
            << std::endl;
  std::cout << "\t   minimumImageHeight                          " << params.minimumImageHeight
            << std::endl;

  std::cout << "\t video encoding" << std::endl;
  std::cout << "\t   geometryQP                                  " << params.geometryQP
            << std::endl;
  std::cout << "\t   textureQP                                   " << params.textureQP << std::endl;
  std::cout << "\t   colorSpaceConversionPath                    "
            << params.colorSpaceConversionPath << std::endl;
  std::cout << "\t   videoEncoderPath                            " << params.videoEncoderPath
            << std::endl;
  std::cout << "\t   videoDecoderPath                            " << params.videoDecoderPath
            << std::endl;
  std::cout << "\t   geometryConfig                              " << params.geometryConfig
            << std::endl;
  std::cout << "\t   textureConfig                               " << params.textureConfig
            << std::endl;
  std::cout << "\t   colorSpaceConversionConfig                  "
            << params.colorSpaceConversionConfig << std::endl;
  std::cout << "\t   inverseColorSpaceConversionConfig           "
            << params.inverseColorSpaceConversionConfig << std::endl;
  std::cout << "\t occupancy map encoding" << std::endl;
  std::cout << "\t   maxCandidateCount                           " << params.maxCandidateCount
            << std::endl;
  std::cout << "\t   occupancyPrecision                          " << params.occupancyPrecision
            << std::endl;

  std::cout << "\t smoothing" << std::endl;
  std::cout << "\t   neighborCountSmoothing                      " << params.neighborCountSmoothing
            << std::endl;
  std::cout << "\t   radius2Smoothing                            " << params.radius2Smoothing
            << std::endl;
  std::cout << "\t   radius2BoundaryDetection                    "
            << params.radius2BoundaryDetection << std::endl;
  std::cout << "\t   thresholdSmoothing                          " << params.thresholdSmoothing
            << std::endl;

  std::cout << "\t coloring" << std::endl;
  std::cout << "\t   bestColorSearchRange                        " << params.bestColorSearchRange
            << std::endl;
  std::cout << std::endl;

  const bool test1 = params.mode == CODEC_MODE_ENCODE &&
                     (params.uncompressedDataPath.empty() || params.compressedStreamPath.empty());
  const bool test2 = params.mode == CODEC_MODE_DECODE &&
                     (params.reconstructedDataPath.empty() || params.compressedStreamPath.empty());
  if (test1 || test2) {
    return false;
  }
  return true;
}

bool LoadGroupOfFrames(const size_t startFrameNumber, const size_t endFrameNumber,
                       GroupOfFrames &frames, const Parameters &params) {
  char fileName[4096];
  if (endFrameNumber < startFrameNumber) {
    return false;
  }
  const size_t frameCount = endFrameNumber - startFrameNumber;
  frames.resize(frameCount);
  for (size_t frameNumber = startFrameNumber; frameNumber < endFrameNumber; ++frameNumber) {
    sprintf(fileName, params.uncompressedDataPath.c_str(), frameNumber);
    auto &frame = frames[frameNumber - startFrameNumber];
    frame.resize(0);
    if (!frame.read(fileName)) {
      std::cout << "Error: can't open " << fileName << std::endl;
      return false;
    }
    if (params.colorTransform == COLOR_TRANSFORM_RGB_TO_YCBCR) {
      frame.convertRGBToYUV();
    }
  }
  return true;
}

bool GenerateTextureVideo(const PCCPointSet3 &pointCloud,
                          const vector<PCCVector3<size_t>> &pointToPixel, const size_t width,
                          const size_t height, const size_t frameCount, PCCVideo3B &video) {
  const size_t pointCount = pointCloud.getPointCount();
  if (!pointCount || !pointCloud.hasColors()) {
    return false;
  }
  const size_t shift = video.getFrameCount();
  video.resize(shift + frameCount);
  for (size_t f = 0; f < frameCount; ++f) {
    auto &frame = video.getFrame(f + shift);
    frame.resize(width, height);
    frame.set(0);
  }
  for (size_t i = 0; i < pointCount; ++i) {
    const PCCVector3<size_t> location = pointToPixel[i];
    const PCCColor3B color = pointCloud.getColor(i);
    const size_t u = location[0];
    const size_t v = location[1];
    const size_t f = location[2];
    auto &frame = video.getFrame(f + shift);
    frame.setValue(0, u, v, color[0]);
    frame.setValue(1, u, v, color[1]);
    frame.setValue(2, u, v, color[2]);
  }
  return true;
}
void PrintMap(vector<bool> img, const size_t sizeU, const size_t sizeV) {
  std::cout << std::endl;
  for (size_t v = 0; v < sizeV; ++v) {
    for (size_t u = 0; u < sizeU; ++u) {
      std::cout << (img[v * sizeU + u] ? 'X' : '.');
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

void Pack(const Parameters &params, vector<PCCPointCloudPatch> &patches, size_t &actualImageSizeU,
          size_t &actualImageSizeV) {
  actualImageSizeV = 0;
  actualImageSizeU = 0;
  if (patches.empty()) {
    return;
  }

  sort(patches.begin(), patches.end());
  size_t occupancySizeU = params.minimumImageWidth / params.occupancyResolution;
  size_t occupancySizeV =
      max(params.minimumImageHeight / params.occupancyResolution, patches[0].sizeV0);
  for (auto &patch : patches) {
    occupancySizeU = max(occupancySizeU, patch.sizeU0 + 1);
  }

  actualImageSizeV = occupancySizeV * params.occupancyResolution;
  actualImageSizeU = occupancySizeU * params.occupancyResolution;

  vector<bool> occupancyMap;
  occupancyMap.resize(occupancySizeU * occupancySizeV, false);
  for (auto &patch : patches) {
    assert(patch.sizeU0 <= occupancySizeU);
    assert(patch.sizeV0 <= occupancySizeV);
    bool locationFound = false;
    while (!locationFound) {
      // print(patch.occupancy, patch.sizeU0, patch.sizeV0);
      // print(occupancyMap, occupancySizeU, occupancySizeV);

      for (size_t v = 0; v <= occupancySizeV - patch.sizeV0 && !locationFound; ++v) {
        for (size_t u = 0; u <= occupancySizeU - patch.sizeU0; ++u) {
          bool canFit = true;
          for (size_t v0 = 0; v0 < patch.sizeV0 && canFit; ++v0) {
            const size_t y = v + v0;
            for (size_t u0 = 0; u0 < patch.sizeU0; ++u0) {
              const size_t x = u + u0;

              if (patch.occupancy[v0 * patch.sizeU0 + u0] && occupancyMap[y * occupancySizeU + x]) {
                canFit = false;
                break;
              }
            }
          }
          if (!canFit) {
            continue;
          }
          locationFound = true;
          patch.u0 = u;
          patch.v0 = v;
          break;
        }
      }
      if (!locationFound) {
        occupancySizeV *= 2;
        occupancyMap.resize(occupancySizeU * occupancySizeV);
      }
    }
    for (size_t v0 = 0; v0 < patch.sizeV0; ++v0) {
      const size_t v = patch.v0 + v0;
      for (size_t u0 = 0; u0 < patch.sizeU0; ++u0) {
        const size_t u = patch.u0 + u0;
        occupancyMap[v * occupancySizeU + u] =
            occupancyMap[v * occupancySizeU + u] || patch.occupancy[v0 * patch.sizeU0 + u0];
      }
    }
    actualImageSizeV = max(actualImageSizeV, (patch.v0 + patch.sizeV0) * patch.occupancyResolution);
    actualImageSizeU = max(actualImageSizeU, (patch.u0 + patch.sizeU0) * patch.occupancyResolution);
    // print(occupancyMap, occupancySizeU, occupancySizeV);
  }
  PrintMap(occupancyMap, occupancySizeU, occupancySizeV);
  std::cout << "actualImageSizeU " << actualImageSizeU << std::endl;
  std::cout << "actualImageSizeV " << actualImageSizeV << std::endl;
}

void GenerateOccupancyMap(const vector<PCCPointCloudPatch> &patches, const size_t imageSizeU,
                          const size_t imageSizeV, vector<uint32_t> &occupancyMap) {
  occupancyMap.resize(imageSizeU * imageSizeV, 0);
  const int16_t infiniteDepth = numeric_limits<int16_t>::max();
  for (const auto &patch : patches) {
    const size_t v0 = patch.v0 * patch.occupancyResolution;
    const size_t u0 = patch.u0 * patch.occupancyResolution;
    for (size_t v = 0; v < patch.sizeV; ++v) {
      for (size_t u = 0; u < patch.sizeU; ++u) {
        const size_t p = v * patch.sizeU + u;
        const int16_t d = patch.depth[0][p];
        if (d < infiniteDepth) {
          const size_t x = (u0 + u);
          const size_t y = (v0 + v);
          assert(x < imageSizeU && y < imageSizeV);
          occupancyMap[x + y * imageSizeU] = 1;
        }
      }
    }
  }
}

void GenerateIntraImage(const vector<PCCPointCloudPatch> &patches, const size_t imageSizeU,
                        const size_t imageSizeV, const size_t depthIndex, PCCImage3B &image) {
  image.resize(imageSizeU, imageSizeV);
  image.set(0);
  const int16_t infiniteDepth = numeric_limits<int16_t>::max();
  size_t maxDepth = 0;
  for (const auto &patch : patches) {
    const size_t v0 = patch.v0 * patch.occupancyResolution;
    const size_t u0 = patch.u0 * patch.occupancyResolution;
    for (size_t v = 0; v < patch.sizeV; ++v) {
      for (size_t u = 0; u < patch.sizeU; ++u) {
        const size_t p = v * patch.sizeU + u;
        const int16_t d = patch.depth[depthIndex][p];
        if (d < infiniteDepth) {
          const size_t x = (u0 + u);
          const size_t y = (v0 + v);
          assert(x < imageSizeU && y < imageSizeV);
          image.setValue(0, x, y, uint8_t(d));
          maxDepth = max(maxDepth, patch.sizeD);
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
void Dilate(const size_t occupancyResolution, vector<uint32_t> occupancyMap, PCCImage3B &image,
            const PCCImage3B *reference = nullptr) {
  const size_t pixelBlockCount = occupancyResolution * occupancyResolution;
  const size_t occupancyMapSizeU = image.getWidth() / occupancyResolution;
  const size_t occupancyMapSizeV = image.getHeight() / occupancyResolution;
  const int64_t neighbors[4][2] = {{0, -1}, {-1, 0}, {1, 0}, {0, 1}};
  const size_t MAX_OCCUPANCY_RESOLUTION = 64;
  assert(occupancyResolution <= MAX_OCCUPANCY_RESOLUTION);
  size_t count[MAX_OCCUPANCY_RESOLUTION][MAX_OCCUPANCY_RESOLUTION];
  PCCVector3<int32_t> values[MAX_OCCUPANCY_RESOLUTION][MAX_OCCUPANCY_RESOLUTION];

  for (size_t v1 = 0; v1 < occupancyMapSizeV; ++v1) {
    const int64_t v0 = v1 * occupancyResolution;
    for (size_t u1 = 0; u1 < occupancyMapSizeU; ++u1) {
      const int64_t u0 = u1 * occupancyResolution;
      size_t nonZeroPixelCount = 0;
      for (size_t v2 = 0; v2 < occupancyResolution; ++v2) {
        for (size_t u2 = 0; u2 < occupancyResolution; ++u2) {
          const int64_t x0 = u0 + u2;
          const int64_t y0 = v0 + v2;
          assert(x0 < int64_t(image.getWidth()) && y0 < int64_t(image.getHeight()));
          const size_t location0 = y0 * image.getWidth() + x0;
          nonZeroPixelCount += (occupancyMap[location0] == 1);
        }
      }
      if (!nonZeroPixelCount) {
        if (reference) {
          for (size_t v2 = 0; v2 < occupancyResolution; ++v2) {
            for (size_t u2 = 0; u2 < occupancyResolution; ++u2) {
              const size_t x0 = u0 + u2;
              const size_t y0 = v0 + v2;
              image.setValue(0, x0, y0, reference->getValue(0, x0, y0));
              image.setValue(1, x0, y0, reference->getValue(1, x0, y0));
              image.setValue(2, x0, y0, reference->getValue(2, x0, y0));
            }
          }
        } else if (u1 > 0) {
          for (size_t v2 = 0; v2 < occupancyResolution; ++v2) {
            for (size_t u2 = 0; u2 < occupancyResolution; ++u2) {
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
          for (size_t v2 = 0; v2 < occupancyResolution; ++v2) {
            for (size_t u2 = 0; u2 < occupancyResolution; ++u2) {
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
      for (size_t v2 = 0; v2 < occupancyResolution; ++v2) {
        for (size_t u2 = 0; u2 < occupancyResolution; ++u2) {
          values[v2][u2] = 0;
          count[v2][u2] = 0UL;
        }
      }
      uint32_t iteration = 1;
      while (nonZeroPixelCount < pixelBlockCount) {
        for (size_t v2 = 0; v2 < occupancyResolution; ++v2) {
          for (size_t u2 = 0; u2 < occupancyResolution; ++u2) {
            const int64_t x0 = u0 + u2;
            const int64_t y0 = v0 + v2;
            assert(x0 < int64_t(image.getWidth()) && y0 < int64_t(image.getHeight()));
            const size_t location0 = y0 * image.getWidth() + x0;
            if (occupancyMap[location0] == iteration) {
              for (size_t n = 0; n < 4; ++n) {
                const int64_t x1 = x0 + neighbors[n][0];
                const int64_t y1 = y0 + neighbors[n][1];
                const size_t location1 = y1 * image.getWidth() + x1;
                if (x1 >= u0 && x1 < int64_t(u0 + occupancyResolution) && y1 >= v0 &&
                    y1 < int64_t(v0 + occupancyResolution) && occupancyMap[location1] == 0) {
                  const int64_t u3 = u2 + neighbors[n][0];
                  const int64_t v3 = v2 + neighbors[n][1];
                  assert(u3 >= 0 && u3 < int64_t(occupancyResolution));
                  assert(v3 >= 0 && v3 < int64_t(occupancyResolution));
                  for (size_t k = 0; k < 3; ++k) {
                    values[v3][u3][k] += image.getValue(k, x0, y0);
                  }
                  ++count[v3][u3];
                }
              }
            }
          }
        }
        for (size_t v2 = 0; v2 < occupancyResolution; ++v2) {
          for (size_t u2 = 0; u2 < occupancyResolution; ++u2) {
            if (count[v2][u2]) {
              ++nonZeroPixelCount;
              const size_t x0 = u0 + u2;
              const size_t y0 = v0 + v2;
              const size_t location0 = y0 * image.getWidth() + x0;
              const size_t c = count[v2][u2];
              const size_t c2 = c / 2;
              occupancyMap[location0] = iteration + 1;
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
bool PredictGeometryFrame(const PCCImage3B &reference, const vector<uint32_t> &occupancyMap,
                          PCCImage3B &frame) {
  assert(reference.getWidth() == frame.getWidth());
  assert(reference.getHeight() == frame.getHeight());
  const size_t width = reference.getWidth();
  const size_t height = reference.getHeight();
  for (size_t y = 0; y < height; ++y) {
    for (size_t x = 0; x < width; ++x) {
      const size_t pos1 = y * width + x;
      if (occupancyMap[pos1] == 1) {
        for (size_t c = 0; c < 3; ++c) {
          const uint8_t value1 = frame.getValue(c, x, y);
          const uint8_t value0 = reference.getValue(c, x, y);
          assert(value0 <= value1);
          const uint8_t delta = value1 - value0;
          assert(delta < 10);
          frame.setValue(c, x, y, delta);
        }
      }
    }
  }
  return true;
}
bool GenerateGeometryVideo(const PCCPointSet3 &geometry, const Parameters &params,
                           const string &path, PCCVideo3B &videoGeometry,
                           EncodingContext &context) {
  if (!geometry.getPointCount()) {
    return false;
  }
  context.patches.reserve(256);
  PCCPatchSegmenter3 segmenter;
  PCCPatchSegmenter3Parameters segmenterParams;
  segmenterParams.nnNormalEstimation = params.nnNormalEstimation;
  segmenterParams.maxNNCountRefineSegmentation = params.maxNNCountRefineSegmentation;
  segmenterParams.iterationCountRefineSegmentation = params.iterationCountRefineSegmentation;
  segmenterParams.occupancyResolution = params.occupancyResolution;
  segmenterParams.minPointCountPerCCPatchSegmentation = params.minPointCountPerCCPatchSegmentation;
  segmenterParams.maxNNCountPatchSegmentation = params.maxNNCountPatchSegmentation;
  segmenterParams.surfaceThickness = params.surfaceThickness;
  segmenterParams.maxAllowedDepth = params.maxAllowedDepth;
  segmenterParams.maxAllowedDist2MissedPointsDetection =
      params.maxAllowedDist2MissedPointsDetection;
  segmenterParams.maxAllowedDist2MissedPointsSelection =
      params.maxAllowedDist2MissedPointsSelection;
  segmenterParams.lambdaRefineSegmentation = params.lambdaRefineSegmentation;
  segmenter.compute(geometry, segmenterParams, path, context.patches);

  context.width = 0;
  context.height = 0;
  Pack(params, context.patches, context.width, context.height);
  if (context.width != params.minimumImageWidth || context.height != params.minimumImageHeight) {
    std::cout << "width != minimumImageWidth || height != minimumImageHeight" << std::endl;
    std::cout << "width " << context.width << " minimumImageWidth " << params.minimumImageWidth
              << std::endl;
    std::cout << "height " << context.height << " minimumImageHeight " << params.minimumImageHeight
              << std::endl;
    exit(-1);
  }
  GenerateOccupancyMap(context.patches, context.width, context.height, context.occupancyMap);
  const size_t shift = videoGeometry.getFrameCount();
  videoGeometry.resize(shift + 2);
  for (size_t f = 0; f < 2; ++f) {
    PCCImage3B &frame1 = videoGeometry.getFrame(shift + f);
    GenerateIntraImage(context.patches, context.width, context.height, f, frame1);
    if (f) {
      PredictGeometryFrame(videoGeometry.getFrame(shift), context.occupancyMap, frame1);
    }
  }
  Dilate(params.occupancyResolution, context.occupancyMap, videoGeometry.getFrame(shift));
  return true;
}

void EncodeUInt32(const uint32_t value, const uint32_t bitCount,
                  o3dgc::Arithmetic_Codec &arithmeticEncoder, o3dgc::Static_Bit_Model &bModel0) {
  uint32_t valueToEncode = PCCToLittleEndian<uint32_t>(value);
  for (uint32_t i = 0; i < bitCount; ++i) {
    arithmeticEncoder.encode(valueToEncode & 1, bModel0);
    valueToEncode >>= 1;
  }
}

void CompressOccupancyMap(const vector<PCCPointCloudPatch> &patches, const size_t imageWidth,
                          const size_t imageHeight, const Parameters &params,
                          vector<size_t> &blockToPatch, vector<uint32_t> &occupancyMap,
                          PCCBitstream &bitstream) {
  const size_t patchCount = patches.size();
  PCCWriteToBuffer<uint32_t>(uint32_t(patchCount), bitstream.buffer, bitstream.size);
  PCCWriteToBuffer<uint8_t>(uint8_t(params.occupancyPrecision), bitstream.buffer, bitstream.size);
  PCCWriteToBuffer<uint8_t>(uint8_t(params.maxCandidateCount), bitstream.buffer, bitstream.size);
  size_t maxU0 = 0;
  size_t maxV0 = 0;
  size_t maxU1 = 0;
  size_t maxV1 = 0;
  size_t maxD1 = 0;
  for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
    const auto &patch = patches[patchIndex];
    maxU0 = max(maxU0, patch.u0);
    maxV0 = max(maxU0, patch.v0);
    maxU1 = max(maxU1, patch.u1);
    maxV1 = max(maxV1, patch.v1);
    maxD1 = max(maxD1, patch.d1);
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
  PCCWriteToBuffer<uint8_t>(bitCountU0, bitstream.buffer, bitstream.size);
  PCCWriteToBuffer<uint8_t>(bitCountV0, bitstream.buffer, bitstream.size);
  PCCWriteToBuffer<uint8_t>(bitCountU1, bitstream.buffer, bitstream.size);
  PCCWriteToBuffer<uint8_t>(bitCountV1, bitstream.buffer, bitstream.size);
  PCCWriteToBuffer<uint8_t>(bitCountD1, bitstream.buffer, bitstream.size);

  uint64_t startSize = bitstream.size;
  bitstream.size += 4;  // placehoder for bitstream size
  o3dgc::Arithmetic_Codec arithmeticEncoder;
  arithmeticEncoder.set_buffer(uint32_t(bitstream.capacity - bitstream.size),
                               bitstream.buffer + bitstream.size);
  arithmeticEncoder.start_encoder();
  o3dgc::Static_Bit_Model bModel0;
  o3dgc::Adaptive_Bit_Model bModelSizeU0, bModelSizeV0;
  o3dgc::Adaptive_Data_Model orientationModel(4);
  int64_t prevSizeU0 = 0;
  int64_t prevSizeV0 = 0;
  for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
    const auto &patch = patches[patchIndex];
    EncodeUInt32(uint32_t(patch.u0), bitCountU0, arithmeticEncoder, bModel0);
    EncodeUInt32(uint32_t(patch.v0), bitCountV0, arithmeticEncoder, bModel0);
    EncodeUInt32(uint32_t(patch.u1), bitCountU1, arithmeticEncoder, bModel0);
    EncodeUInt32(uint32_t(patch.v1), bitCountV1, arithmeticEncoder, bModel0);
    EncodeUInt32(uint32_t(patch.d1), bitCountD1, arithmeticEncoder, bModel0);
    const int64_t deltaSizeU0 = static_cast<int64_t>(patch.sizeU0) - prevSizeU0;
    const int64_t deltaSizeV0 =
        patchIndex == 0 ? patch.sizeV0 : prevSizeV0 - static_cast<int64_t>(patch.sizeV0);
    assert(deltaSizeV0 >= 0);
    arithmeticEncoder.ExpGolombEncode(o3dgc::IntToUInt(int32_t(deltaSizeU0)), 0, bModel0,
                                      bModelSizeU0);
    arithmeticEncoder.ExpGolombEncode(int32_t(deltaSizeV0), 0, bModel0, bModelSizeV0);
    prevSizeU0 = patch.sizeU0;
    prevSizeV0 = patch.sizeV0;
    arithmeticEncoder.encode(uint32_t(patch.normalAxis), orientationModel);
  }
  const size_t blockToPatchWidth = imageWidth / params.occupancyResolution;
  const size_t blockToPatchHeight = imageHeight / params.occupancyResolution;
  const size_t blockCount = blockToPatchWidth * blockToPatchHeight;
  blockToPatch.resize(0);
  blockToPatch.resize(blockCount, 0);
  for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
    const auto &patch = patches[patchIndex];
    for (size_t v0 = 0; v0 < patch.sizeV0; ++v0) {
      for (size_t u0 = 0; u0 < patch.sizeU0; ++u0) {
        if (patch.occupancy[v0 * patch.sizeU0 + u0]) {
          blockToPatch[(v0 + patch.v0) * blockToPatchWidth + (u0 + patch.u0)] = patchIndex + 1;
        }
      }
    }
  }
  vector<vector<size_t>> candidatePatches;
  candidatePatches.resize(blockCount);
  for (int64_t patchIndex = patchCount - 1; patchIndex >= 0;
       --patchIndex) {  // add actual patches based on their bounding box
    const auto &patch = patches[patchIndex];
    for (size_t v0 = 0; v0 < patch.sizeV0; ++v0) {
      for (size_t u0 = 0; u0 < patch.sizeU0; ++u0) {
        candidatePatches[(patch.v0 + v0) * blockToPatchWidth + (patch.u0 + u0)].push_back(
            patchIndex + 1);
      }
    }
  }
  for (auto &candidatePatch : candidatePatches) {  // add empty as potential candidate
    candidatePatch.push_back(0);
  }
  o3dgc::Adaptive_Data_Model candidateIndexModel(uint32_t(params.maxCandidateCount + 2));
  const uint32_t bitCountPatchIndex =
      PCCGetNumberOfBitsInFixedLengthRepresentation(uint32_t(patchCount + 1));
  for (size_t p = 0; p < blockCount; ++p) {
    const size_t patchIndex = blockToPatch[p];
    const auto &candidates = candidatePatches[p];
    if (candidates.size() == 1) {
      // empty
    } else {
      const uint32_t candidateCount = uint32_t(min(candidates.size(), params.maxCandidateCount));
      bool found = false;
      for (uint32_t i = 0; i < candidateCount; ++i) {
        if (candidates[i] == patchIndex) {
          found = true;
          arithmeticEncoder.encode(i, candidateIndexModel);
          break;
        }
      }
      if (!found) {
        arithmeticEncoder.encode(uint32_t(params.maxCandidateCount), candidateIndexModel);
        EncodeUInt32(uint32_t(patchIndex), bitCountPatchIndex, arithmeticEncoder, bModel0);
      }
    }
  }

  const size_t blockSize0 = params.occupancyResolution / params.occupancyPrecision;
  const size_t pointCount0 = blockSize0 * blockSize0;
  const size_t traversalOrderCount = 4;
  vector<vector<pair<size_t, size_t>>> traversalOrders;
  traversalOrders.resize(traversalOrderCount);
  for (size_t k = 0; k < traversalOrderCount; ++k) {
    auto &traversalOrder = traversalOrders[k];
    traversalOrder.reserve(pointCount0);
    if (k == 0) {
      for (size_t v1 = 0; v1 < blockSize0; ++v1) {
        for (size_t u1 = 0; u1 < blockSize0; ++u1) {
          traversalOrder.push_back(make_pair(u1, v1));
        }
      }
    } else if (k == 1) {
      for (size_t v1 = 0; v1 < blockSize0; ++v1) {
        for (size_t u1 = 0; u1 < blockSize0; ++u1) {
          traversalOrder.push_back(make_pair(v1, u1));
        }
      }
    } else if (k == 2) {
      for (int64_t k = 1; k < int64_t(2 * blockSize0); ++k) {
        for (size_t u1 = max(int64_t(0), k - int64_t(blockSize0));
             int64_t(u1) < min(k, int64_t(blockSize0)); ++u1) {
          const size_t v1 = k - (u1 + 1);
          traversalOrder.push_back(make_pair(u1, v1));
        }
      }
    } else {
      for (int64_t k = 1; k < int64_t(2 * blockSize0); ++k) {
        for (size_t u1 = max(int64_t(0), k - int64_t(blockSize0));
             int64_t(u1) < min(k, int64_t(blockSize0)); ++u1) {
          const size_t v1 = k - (u1 + 1);
          traversalOrder.push_back(make_pair(blockSize0 - (1 + u1), v1));
        }
      }
    }
  }
  o3dgc::Adaptive_Bit_Model fullBlockModel, occupancyModel;
  o3dgc::Adaptive_Data_Model traversalOrderIndexModel(uint32_t(traversalOrderCount + 1));
  o3dgc::Adaptive_Data_Model runCountModel((uint32_t)(pointCount0));
  o3dgc::Adaptive_Data_Model runLengthModel((uint32_t)(pointCount0));

  vector<bool> block0;
  vector<size_t> bestRuns;
  vector<size_t> runs;
  block0.resize(pointCount0);
  for (size_t v0 = 0; v0 < blockToPatchHeight; ++v0) {
    for (size_t u0 = 0; u0 < blockToPatchWidth; ++u0) {
      const size_t patchIndex = blockToPatch[v0 * blockToPatchWidth + u0];
      if (patchIndex) {
        size_t fullCount = 0;
        for (size_t v1 = 0; v1 < blockSize0; ++v1) {
          const size_t v2 = v0 * params.occupancyResolution + v1 * params.occupancyPrecision;
          for (size_t u1 = 0; u1 < blockSize0; ++u1) {
            const size_t u2 = u0 * params.occupancyResolution + u1 * params.occupancyPrecision;
            bool isFull = false;
            for (size_t v3 = 0; v3 < params.occupancyPrecision && !isFull; ++v3) {
              for (size_t u3 = 0; u3 < params.occupancyPrecision && !isFull; ++u3) {
                isFull |= occupancyMap[(v2 + v3) * imageWidth + u2 + u3] == 1;
              }
            }
            block0[v1 * blockSize0 + u1] = isFull;
            fullCount += isFull;
            for (size_t v3 = 0; v3 < params.occupancyPrecision; ++v3) {
              for (size_t u3 = 0; u3 < params.occupancyPrecision; ++u3) {
                occupancyMap[(v2 + v3) * imageWidth + u2 + u3] = isFull;
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
  bitstream.size += compressedBitstreamSize;
  PCCWriteToBuffer<uint32_t>(compressedBitstreamSize, bitstream.buffer, startSize);
}

void GeneratePointCloud(const vector<PCCPointCloudPatch> &patches, const PCCVideo3B &video,
                        const size_t shift, const vector<size_t> &blockToPatch,
                        const vector<uint32_t> &occupancyMap, const size_t occupancyResolution,
                        vector<uint32_t> &partition, vector<PCCVector3<size_t>> &pointToPixel,
                        PCCPointSet3 &pointCloud) {
  partition.resize(0);
  pointToPixel.resize(0);
  pointCloud.clear();
  const size_t layerCount = 2;
  if (video.getFrameCount() < (shift + layerCount)) {
    return;
  }
  const auto &frame0 = video.getFrame(shift);
  const size_t imageWidth = video.getWidth();
  const size_t blockToPatchWidth = imageWidth / occupancyResolution;
  pointCloud.addColors();
  const size_t patchCount = patches.size();
  for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
    const size_t patchIndexPlusOne = patchIndex + 1;
    const auto &patch = patches[patchIndex];
    PCCColor3B color(uint8_t(0));
    while (color[0] == color[1] || color[2] == color[1] || color[2] == color[0]) {
      color[0] = static_cast<uint8_t>(rand() % 32) * 8;
      color[1] = static_cast<uint8_t>(rand() % 32) * 8;
      color[2] = static_cast<uint8_t>(rand() % 32) * 8;
    }
    for (size_t v0 = 0; v0 < patch.sizeV0; ++v0) {
      for (size_t u0 = 0; u0 < patch.sizeU0; ++u0) {
        if (blockToPatch[(v0 + patch.v0) * blockToPatchWidth + u0 + patch.u0] ==
            patchIndexPlusOne) {
          for (size_t v1 = 0; v1 < patch.occupancyResolution; ++v1) {
            const size_t v = v0 * patch.occupancyResolution + v1;
            for (size_t u1 = 0; u1 < patch.occupancyResolution; ++u1) {
              const size_t u = u0 * patch.occupancyResolution + u1;
              const size_t x = patch.u0 * patch.occupancyResolution + u;
              const size_t y = patch.v0 * patch.occupancyResolution + v;
              const bool occupancy = occupancyMap[y * imageWidth + x] != 0;
              if (!occupancy) {
                continue;
              }

              PCCVector3D point0;
              point0[patch.normalAxis] = double(frame0.getValue(0, x, y) + patch.d1);
              point0[patch.tangentAxis] = double(u) + patch.u1;
              point0[patch.bitangentAxis] = double(v) + patch.v1;
              for (size_t f = 0; f < layerCount; ++f) {
                PCCVector3D point1(point0);
                if (f > 0) {
                  const auto &frame1 = video.getFrame(f + shift);
                  point1[patch.normalAxis] += frame1.getValue(0, x, y);
                }
                const size_t pointIndex1 = pointCloud.addPoint(point1);
                pointCloud.setColor(pointIndex1, color);
                partition.push_back(uint32_t(patchIndex));
                pointToPixel.push_back(PCCVector3<size_t>(x, y, f));
              }
            }
          }
        }
      }
    }
  }
}

void SmoothPointCloud(const vector<uint32_t> &partition, PCCPointSet3 &pointCloud,
                      const double radius2Smoothing, const size_t neighborCountSmoothing,
                      const double radius2BoundaryDetection, const double thresholdSmoothing) {
  const size_t pointCount = pointCloud.getPointCount();
  PCCStaticKdTree3 kdtree;
  kdtree.build(pointCloud);
  PCCPointSet3 temp;
  temp.resize(pointCount);
  tbb::parallel_for(size_t(0), pointCount, [&](const size_t i) {
    //  for (size_t i = 0; i < pointCount; ++i) {
    const size_t maxNeighborCount = 512;
    PCCPointDistInfo nNeighbor[maxNeighborCount];
    PCCNNResult result = {nNeighbor, 0};
    const double maxDist = ceil(sqrt(radius2Smoothing));
    PCCNNQuery3 query = {PCCVector3D(0.0), maxDist, min(maxNeighborCount, neighborCountSmoothing)};

    const size_t clusterIndex = partition[i];
    query.point = pointCloud[i];
    kdtree.findNearestNeighbors(query, result);
    assert(result.resultCount);
    PCCVector3D centroid(0.0);
    bool otherClusterPointCount = false;
    size_t neighborCount = 0;
    for (size_t r = 0; r < result.resultCount; ++r) {
      const double dist2 = result.neighbors[r].dist2;
      if (dist2 > radius2Smoothing) {
        break;
      }
      ++neighborCount;
      const size_t index = result.neighbors[r].index;
      centroid += pointCloud[index];
      otherClusterPointCount |=
          (dist2 <= radius2BoundaryDetection) && (partition[index] != clusterIndex);
    }

    if (otherClusterPointCount) {
      const auto sclaedPoint = double(neighborCount) * query.point;
      const double distToCentroid2 =
          int64_t((centroid - sclaedPoint).getNorm2() + (neighborCount / 2.0)) /
          double(neighborCount);
      for (size_t k = 0; k < 3; ++k) {
        centroid[k] = double(int64_t(centroid[k] + (neighborCount / 2)) / neighborCount);
      }
      if (distToCentroid2 >= thresholdSmoothing) {
        temp[i] = centroid;
        pointCloud.setColor(i, PCCColor3B(255, 0, 0));
      } else {
        temp[i] = query.point;
      }

    } else {
      temp[i] = query.point;
    }
  });
  tbb::parallel_for(size_t(0), pointCount, [&](const size_t i) {
    // for (size_t i = 0; i < pointCount; ++i) {
    pointCloud[i] = temp[i];
  });
}
bool ColorPointCloud(const PCCVideo3B &video, const vector<PCCVector3<size_t>> pointToPixel,
                     const size_t shift, PCCPointSet3 &pointCloud) {
  const size_t pointCount = pointCloud.getPointCount();
  if (!pointCount || !pointCloud.hasColors()) {
    return false;
  }
  for (size_t i = 0; i < pointCount; ++i) {
    const PCCVector3<size_t> location = pointToPixel[i];
    const size_t x = location[0];
    const size_t y = location[1];
    const size_t f = location[2];
    const auto &frame = video.getFrame(shift + f);
    PCCColor3B color;
    for (size_t c = 0; c < 3; ++c) {
      color[c] = frame.getValue(c, x, y);
    }
    pointCloud.setColor(i, color);
  }
  return true;
}

int CompressGroupOfFrames(const GroupOfFrames &groupOfFrames, PCCBitstream &bitstream,
                          const Parameters &params, const size_t groupOfFramesIndex) {
  assert(groupOfFrames.size() < 256);
  const uint8_t groupOfFramesSize = uint8_t(groupOfFrames.size());
  PCCWriteToBuffer<uint8_t>(groupOfFramesSize, bitstream.buffer, bitstream.size);

  if (!groupOfFramesSize) {
    return 0;
  }
  const size_t pointCount = groupOfFrames[0].getPointCount();
  stringstream path;
  path << params.compressedStreamPath << "_GOF" << groupOfFramesIndex << "_";

  vector<EncodingContext> contexts;
  contexts.resize(groupOfFramesSize);

  PCCVideo3B videoGeometry;
  for (size_t f = 0; f < groupOfFramesSize; ++f) {
    stringstream path1;
    path1 << path.str() << "frame" << f << "_";
    auto &context = contexts[f];
    GenerateGeometryVideo(groupOfFrames[f], params, path1.str(), videoGeometry, context);
  }
  const size_t width = contexts[0].width;
  const size_t height = contexts[0].height;

  PCCWriteToBuffer<uint16_t>(uint16_t(width), bitstream.buffer, bitstream.size);
  PCCWriteToBuffer<uint16_t>(uint16_t(height), bitstream.buffer, bitstream.size);
  PCCWriteToBuffer<uint8_t>(uint8_t(params.occupancyResolution), bitstream.buffer, bitstream.size);
  PCCWriteToBuffer<uint8_t>(uint8_t(params.radius2Smoothing), bitstream.buffer, bitstream.size);
  PCCWriteToBuffer<uint8_t>(uint8_t(params.neighborCountSmoothing), bitstream.buffer,
                            bitstream.size);
  PCCWriteToBuffer<uint8_t>(uint8_t(params.radius2BoundaryDetection), bitstream.buffer,
                            bitstream.size);
  PCCWriteToBuffer<uint8_t>(uint8_t(params.thresholdSmoothing), bitstream.buffer, bitstream.size);

  auto sizeGeometryVideo = bitstream.size;
  videoGeometry.compress(path.str() + "geometry", params.geometryQP, bitstream,
                         params.geometryConfig, params.videoEncoderPath);
  videoGeometry.read420(path.str() + "geometry_rec.yuv", contexts[0].width, contexts[0].height,
                        2 * groupOfFramesSize);
  sizeGeometryVideo = bitstream.size - sizeGeometryVideo;

#ifndef POINT_CLOUD_CODEC_DUMP_INTERMEDIARY_RESULTS
  remove((path.str() + "geometry.bin").c_str());
  remove((path.str() + "geometry.yuv").c_str());
  remove((path.str() + "geometry_rec.yuv").c_str());
#endif  // POINT_CLOUD_CODEC_DUMP_INTERMEDIARY_RESULTS

  std::cout << "geometry video ->" << sizeGeometryVideo << " B ("
            << (sizeGeometryVideo * 8.0) / (2 * groupOfFramesSize * pointCount) << " bpp)"
            << std::endl;

  auto sizeOccupancyMap = bitstream.size;
  for (size_t f = 0; f < groupOfFramesSize; ++f) {
    auto &context = contexts[f];
    CompressOccupancyMap(context.patches, context.width, context.height, params,
                         context.blockToPatch, context.occupancyMap, bitstream);
  }
  sizeOccupancyMap = bitstream.size - sizeOccupancyMap;
  std::cout << " occupancy map  ->" << sizeOccupancyMap << " B ("
            << (sizeOccupancyMap * 8.0) / (2 * groupOfFramesSize * pointCount) << " bpp)"
            << std::endl;

  for (size_t f = 0; f < groupOfFramesSize; ++f) {
    auto &context = contexts[f];
    vector<uint32_t> partition;
    GeneratePointCloud(context.patches, videoGeometry, 2 * f, context.blockToPatch,
                       context.occupancyMap, params.occupancyResolution, partition,
                       context.pointToPixel, context.frame0);
    SmoothPointCloud(partition, context.frame0, params.radius2Smoothing,
                     params.neighborCountSmoothing, params.radius2BoundaryDetection,
                     params.thresholdSmoothing);
  }

  PCCVideo3B videoTexture;
  for (size_t f = 0; f < groupOfFramesSize; ++f) {
    assert(contexts[f].width == width && contexts[f].height == height);
    auto &context = contexts[f];
    PCCTransfertColors(groupOfFrames[f], int32_t(params.bestColorSearchRange), context.frame0);
    GenerateTextureVideo(context.frame0, context.pointToPixel, width, height, 2, videoTexture);
  }

  const size_t textureFrameCount = 2 * groupOfFramesSize;
  assert(textureFrameCount == videoTexture.getFrameCount());
  for (size_t f = 0; f < textureFrameCount; ++f) {
    Dilate(params.occupancyResolution, contexts[f / 2].occupancyMap, videoTexture.getFrame(f));
  }

  auto sizeTextureVideo = bitstream.size;
  videoTexture.compress(path.str() + "texture", params.textureQP, bitstream, params.textureConfig,
                        params.videoEncoderPath, params.colorSpaceConversionConfig,
                        params.colorSpaceConversionPath);
  if (params.inverseColorSpaceConversionConfig.empty() || params.colorSpaceConversionPath.empty()) {
    videoTexture.read420(path.str() + "texture_rec.yuv", width, height, textureFrameCount);
  } else {
    std::stringstream cmd;
    const std::string yuvFileName = path.str() + "texture_rec.yuv";
    const std::string rgbFileName = path.str() + "texture_rec.rgb";
    cmd << params.colorSpaceConversionPath << " -f " << params.inverseColorSpaceConversionConfig
        << " -p SourceFile=\"" << yuvFileName << "\" -p OutputFile=\"" << rgbFileName
        << "\" -p SourceWidth=" << width << " -p SourceHeight=" << height
        << " -p NumberOfFrames=" << textureFrameCount << std::endl;
    std::cout << cmd.str();
    if (int ret = system(cmd.str().c_str())) {
      std::cout << "Error: can't run system command!" << std::endl;
      return ret;
    }
    videoTexture.read(rgbFileName, width, height, textureFrameCount);
  }

  sizeTextureVideo = bitstream.size - sizeTextureVideo;
  std::cout << "texture video  ->" << sizeTextureVideo << " B ("
            << (sizeTextureVideo * 8.0) / pointCount << " bpp)" << std::endl;

#ifndef POINT_CLOUD_CODEC_DUMP_INTERMEDIARY_RESULTS
  remove((path.str() + "texture.bin").c_str());
  remove((path.str() + "texture.yuv").c_str());
  remove((path.str() + "texture_rec.yuv").c_str());
  if (!params.colorSpaceConversionConfig.empty() && !params.colorSpaceConversionPath.empty() &&
      !params.inverseColorSpaceConversionConfig.empty()) {
    remove((path.str() + "texture.rgb").c_str());
    remove((path.str() + "texture_rec.rgb").c_str());
  }
#endif  // POINT_CLOUD_CODEC_DUMP_INTERMEDIARY_RESULTS
  return 0;
}

int CompressVideo(const Parameters &params) {
  const size_t startFrameNumber0 = params.startFrameNumber;
  const size_t endFrameNumber0 = params.startFrameNumber + params.frameCount;
  const size_t groupOfFramesSize0 = (std::max)(size_t(1), params.groupOfFramesSize);
  size_t startFrameNumber = startFrameNumber0;
  PCCBitstream bitstream = {};
  std::unique_ptr<uint8_t> buffer;
  uint64_t totalSizeIterator = 0;
  size_t groupOfFramesIndex = 0;
  while (startFrameNumber < endFrameNumber0) {
    const size_t endFrameNumber = min(startFrameNumber + groupOfFramesSize0, endFrameNumber0);
    GroupOfFrames groupOfFrames;
    if (!LoadGroupOfFrames(startFrameNumber, endFrameNumber, groupOfFrames, params)) {
      return -1;
    }
    if (startFrameNumber == startFrameNumber0) {
      const size_t predictedBitstreamSize =
          10000 + 8 * params.frameCount * groupOfFrames[0].getPointCount();
      buffer.reset(new uint8_t[predictedBitstreamSize]);
      bitstream.buffer = buffer.get();
      bitstream.capacity = predictedBitstreamSize;
      bitstream.size = 0;

      PCCWriteToBuffer<uint32_t>(PCCTMC2ContainerMagicNumber, bitstream.buffer, bitstream.size);
      PCCWriteToBuffer<uint32_t>(PCCTMC2ContainerVersion, bitstream.buffer, bitstream.size);
      totalSizeIterator = bitstream.size;
      PCCWriteToBuffer<uint64_t>(0, bitstream.buffer, bitstream.size);  // reserved
    }

    std::cout << "Compressing group of frames " << groupOfFramesIndex << ": " << startFrameNumber
              << " -> " << endFrameNumber << "..." << std::endl;

    if (int ret = CompressGroupOfFrames(groupOfFrames, bitstream, params, groupOfFramesIndex++)) {
      return ret;
    }
    startFrameNumber = endFrameNumber;
  }

  assert(bitstream.size <= bitstream.capacity);
  PCCWriteToBuffer<uint64_t>(bitstream.size, bitstream.buffer, totalSizeIterator);
  std::cout << "Total bitstream size " << bitstream.size << " B" << std::endl;
  ofstream fout(params.compressedStreamPath, ios::binary);
  if (!fout.is_open()) {
    return -1;
  }
  fout.write(reinterpret_cast<const char *>(bitstream.buffer), bitstream.size);
  fout.close();
  return 0;
}
uint32_t DecodeUInt32(const uint32_t bitCount, o3dgc::Arithmetic_Codec &arithmeticDecoder,
                      o3dgc::Static_Bit_Model &bModel0) {
  uint32_t decodedValue = 0;
  for (uint32_t i = 0; i < bitCount; ++i) {
    decodedValue += (arithmeticDecoder.decode(bModel0) << i);
  }
  return PCCFromLittleEndian<uint32_t>(decodedValue);
}

void DecompressOccupancyMap(const size_t width, const size_t height,
                            const size_t &occupancyResolution, vector<PCCPointCloudPatch> &patches,
                            vector<size_t> &blockToPatch, vector<uint32_t> &occupancyMap,
                            PCCBitstream &bitstream) {
  uint32_t patchCount = 0;
  PCCReadFromBuffer<uint32_t>(bitstream.buffer, patchCount, bitstream.size);
  patches.resize(patchCount);

  size_t occupancyPrecision = 0;
  {
    uint8_t precision = 0;
    PCCReadFromBuffer<uint8_t>(bitstream.buffer, precision, bitstream.size);
    occupancyPrecision = precision;
  }
  size_t maxCandidateCount = 0;
  {
    uint8_t count = 0;
    PCCReadFromBuffer<uint8_t>(bitstream.buffer, count, bitstream.size);
    maxCandidateCount = count;
  }

  uint8_t bitCountU0 = 0;
  uint8_t bitCountV0 = 0;
  uint8_t bitCountU1 = 0;
  uint8_t bitCountV1 = 0;
  uint8_t bitCountD1 = 0;
  PCCReadFromBuffer<uint8_t>(bitstream.buffer, bitCountU0, bitstream.size);
  PCCReadFromBuffer<uint8_t>(bitstream.buffer, bitCountV0, bitstream.size);
  PCCReadFromBuffer<uint8_t>(bitstream.buffer, bitCountU1, bitstream.size);
  PCCReadFromBuffer<uint8_t>(bitstream.buffer, bitCountV1, bitstream.size);
  PCCReadFromBuffer<uint8_t>(bitstream.buffer, bitCountD1, bitstream.size);

  uint32_t compressedBitstreamSize;
  PCCReadFromBuffer<uint32_t>(bitstream.buffer, compressedBitstreamSize, bitstream.size);
  assert(compressedBitstreamSize + bitstream.size <= bitstream.capacity);
  o3dgc::Arithmetic_Codec arithmeticDecoder;
  arithmeticDecoder.set_buffer(uint32_t(bitstream.capacity - bitstream.size),
                               bitstream.buffer + bitstream.size);
  arithmeticDecoder.start_decoder();
  o3dgc::Static_Bit_Model bModel0;
  o3dgc::Adaptive_Bit_Model bModelSizeU0, bModelSizeV0;
  o3dgc::Adaptive_Data_Model orientationModel(4);
  int64_t prevSizeU0 = 0;
  int64_t prevSizeV0 = 0;

  for (size_t patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
    auto &patch = patches[patchIndex];
    patch.occupancyResolution = occupancyResolution;

    patch.u0 = DecodeUInt32(bitCountU0, arithmeticDecoder, bModel0);
    patch.v0 = DecodeUInt32(bitCountV0, arithmeticDecoder, bModel0);
    patch.u1 = DecodeUInt32(bitCountU1, arithmeticDecoder, bModel0);
    patch.v1 = DecodeUInt32(bitCountV1, arithmeticDecoder, bModel0);
    patch.d1 = DecodeUInt32(bitCountD1, arithmeticDecoder, bModel0);

    const int64_t deltaSizeU0 =
        o3dgc::UIntToInt(arithmeticDecoder.ExpGolombDecode(0, bModel0, bModelSizeU0));
    const int64_t deltaSizeV0 = arithmeticDecoder.ExpGolombDecode(0, bModel0, bModelSizeV0);

    patch.sizeU0 = prevSizeU0 + deltaSizeU0;
    patch.sizeV0 = patchIndex == 0 ? deltaSizeV0 : prevSizeV0 - deltaSizeV0;

    prevSizeU0 = patch.sizeU0;
    prevSizeV0 = patch.sizeV0;

    patch.normalAxis = arithmeticDecoder.decode(orientationModel);
    if (patch.normalAxis == 0) {
      patch.tangentAxis = 2;
      patch.bitangentAxis = 1;
    } else if (patch.normalAxis == 1) {
      patch.tangentAxis = 2;
      patch.bitangentAxis = 0;
    } else {
      patch.tangentAxis = 0;
      patch.bitangentAxis = 1;
    }
  }
  const size_t blockToPatchWidth = width / occupancyResolution;
  const size_t blockToPatchHeight = height / occupancyResolution;
  const size_t blockCount = blockToPatchWidth * blockToPatchHeight;

  vector<vector<size_t>> candidatePatches;
  candidatePatches.resize(blockCount);
  for (int64_t patchIndex = patchCount - 1; patchIndex >= 0;
       --patchIndex) {  // add actual patches based on their bounding box
    const auto &patch = patches[patchIndex];
    for (size_t v0 = 0; v0 < patch.sizeV0; ++v0) {
      for (size_t u0 = 0; u0 < patch.sizeU0; ++u0) {
        candidatePatches[(patch.v0 + v0) * blockToPatchWidth + (patch.u0 + u0)].push_back(
            patchIndex + 1);
      }
    }
  }
  for (auto &candidatePatch : candidatePatches) {  // add empty as potential candidate
    candidatePatch.push_back(0);
  }

  blockToPatch.resize(0);
  blockToPatch.resize(blockCount, 0);
  o3dgc::Adaptive_Data_Model candidateIndexModel(uint32_t(maxCandidateCount + 2));
  const uint32_t bitCountPatchIndex = PCCGetNumberOfBitsInFixedLengthRepresentation(patchCount + 1);
  for (size_t p = 0; p < blockCount; ++p) {
    const auto &candidates = candidatePatches[p];
    if (candidates.size() == 1) {
      blockToPatch[p] = candidates[0];
    } else {
      const size_t candidateIndex = arithmeticDecoder.decode(candidateIndexModel);
      if (candidateIndex == maxCandidateCount) {
        blockToPatch[p] = DecodeUInt32(bitCountPatchIndex, arithmeticDecoder, bModel0);
      } else {
        blockToPatch[p] = candidates[candidateIndex];
      }
    }
  }

  const size_t blockSize0 = occupancyResolution / occupancyPrecision;
  const size_t pointCount0 = blockSize0 * blockSize0;
  const size_t traversalOrderCount = 4;
  vector<vector<pair<size_t, size_t>>> traversalOrders;
  traversalOrders.resize(traversalOrderCount);
  for (size_t k = 0; k < traversalOrderCount; ++k) {
    auto &traversalOrder = traversalOrders[k];
    traversalOrder.reserve(pointCount0);
    if (k == 0) {
      for (size_t v1 = 0; v1 < blockSize0; ++v1) {
        for (size_t u1 = 0; u1 < blockSize0; ++u1) {
          traversalOrder.push_back(make_pair(u1, v1));
        }
      }
    } else if (k == 1) {
      for (size_t v1 = 0; v1 < blockSize0; ++v1) {
        for (size_t u1 = 0; u1 < blockSize0; ++u1) {
          traversalOrder.push_back(make_pair(v1, u1));
        }
      }
    } else if (k == 2) {
      for (int64_t k = 1; k < int64_t(2 * blockSize0); ++k) {
        for (int64_t u1 = max(int64_t(0), k - int64_t(blockSize0));
             u1 < min(k, int64_t(blockSize0)); ++u1) {
          const size_t v1 = k - (u1 + 1);
          traversalOrder.push_back(make_pair(u1, v1));
        }
      }
    } else {
      for (int64_t k = 1; k < int64_t(2 * blockSize0); ++k) {
        for (int64_t u1 = max(int64_t(0), k - int64_t(blockSize0));
             u1 < min(k, int64_t(blockSize0)); ++u1) {
          const size_t v1 = k - (u1 + 1);
          traversalOrder.push_back(make_pair(blockSize0 - (1 + u1), v1));
        }
      }
    }
  }
  o3dgc::Adaptive_Bit_Model fullBlockModel, occupancyModel;
  o3dgc::Adaptive_Data_Model traversalOrderIndexModel(uint32_t(traversalOrderCount + 1));
  o3dgc::Adaptive_Data_Model runCountModel((uint32_t)(pointCount0));
  o3dgc::Adaptive_Data_Model runLengthModel((uint32_t)(pointCount0));

  vector<uint32_t> block0;
  vector<size_t> bestRuns;
  vector<size_t> runs;
  block0.resize(pointCount0);
  occupancyMap.resize(width * height, 0);
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
          const size_t bestTraversalOrderIndex = arithmeticDecoder.decode(traversalOrderIndexModel);
          const auto &traversalOrder = traversalOrders[bestTraversalOrderIndex];
          const size_t runCountMinusTwo = arithmeticDecoder.decode(runCountModel);
          const size_t runCountMinusOne = runCountMinusTwo + 1;
          size_t i = 0;
          bool occupancy = arithmeticDecoder.decode(occupancyModel) != 0;
          for (size_t r = 0; r < runCountMinusOne; ++r) {
            const size_t runLength = arithmeticDecoder.decode(runLengthModel);
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
          const size_t v2 = v0 * occupancyResolution + v1 * occupancyPrecision;
          for (size_t u1 = 0; u1 < blockSize0; ++u1) {
            const size_t u2 = u0 * occupancyResolution + u1 * occupancyPrecision;
            const bool occupancy = block0[v1 * blockSize0 + u1] != 0;
            for (size_t v3 = 0; v3 < occupancyPrecision; ++v3) {
              for (size_t u3 = 0; u3 < occupancyPrecision; ++u3) {
                occupancyMap[(v2 + v3) * width + u2 + u3] = occupancy;
              }
            }
          }
        }
      }
    }
  }
  arithmeticDecoder.stop_decoder();
  bitstream.size += compressedBitstreamSize;
}

int DecompressGroupOfFrames(const size_t groupOfFramesIndex, const Parameters &params,
                            vector<PCCPointSet3> &groupOfFrames, PCCBitstream &bitstream) {
  uint8_t groupOfFramesSize = 0;
  PCCReadFromBuffer<uint8_t>(bitstream.buffer, groupOfFramesSize, bitstream.size);
  if (!groupOfFramesSize) {
    return 0;
  }

  uint16_t width = 0;
  uint16_t height = 0;
  PCCReadFromBuffer<uint16_t>(bitstream.buffer, width, bitstream.size);
  PCCReadFromBuffer<uint16_t>(bitstream.buffer, height, bitstream.size);
  uint8_t occupancyResolution = 0;
  PCCReadFromBuffer<uint8_t>(bitstream.buffer, occupancyResolution, bitstream.size);
  uint8_t radius2Smoothing = 0;
  PCCReadFromBuffer<uint8_t>(bitstream.buffer, radius2Smoothing, bitstream.size);
  uint8_t neighborCountSmoothing = 0;
  PCCReadFromBuffer<uint8_t>(bitstream.buffer, neighborCountSmoothing, bitstream.size);
  uint8_t radius2BoundaryDetection = 0;
  PCCReadFromBuffer<uint8_t>(bitstream.buffer, radius2BoundaryDetection, bitstream.size);
  uint8_t thresholdSmoothing = 0;
  PCCReadFromBuffer<uint8_t>(bitstream.buffer, thresholdSmoothing, bitstream.size);

  stringstream path;
  path << params.compressedStreamPath << "_dec_GOF" << groupOfFramesIndex << "_";

  vector<EncodingContext> contexts;
  contexts.resize(groupOfFramesSize);
  PCCVideo3B videoGeometry;
  auto sizeGeometryVideo = bitstream.size;
  videoGeometry.decompress(path.str() + "geometry", width, height, groupOfFramesSize * 2, bitstream,
                           params.videoDecoderPath);
  sizeGeometryVideo = bitstream.size - sizeGeometryVideo;

#ifndef POINT_CLOUD_CODEC_DUMP_INTERMEDIARY_RESULTS
  remove((path.str() + "geometry.bin").c_str());
  remove((path.str() + "geometry_rec.yuv").c_str());
#endif  // POINT_CLOUD_CODEC_DUMP_INTERMEDIARY_RESULTS

  std::cout << "geometry video ->" << sizeGeometryVideo << " B" << std::endl;

  groupOfFrames.resize(groupOfFramesSize);
  auto sizeOccupancyMap = bitstream.size;
  for (size_t f = 0; f < groupOfFramesSize; ++f) {
    auto &context = contexts[f];
    DecompressOccupancyMap(width, height, occupancyResolution, context.patches,
                           context.blockToPatch, context.occupancyMap, bitstream);
  }
  sizeOccupancyMap = bitstream.size - sizeOccupancyMap;
  std::cout << "occupancy map  ->" << sizeOccupancyMap << " B" << std::endl;

  for (size_t f = 0; f < groupOfFramesSize; ++f) {
    auto &context = contexts[f];
    vector<uint32_t> partition;
    GeneratePointCloud(context.patches, videoGeometry, 2 * f, context.blockToPatch,
                       context.occupancyMap, occupancyResolution, partition, context.pointToPixel,
                       context.frame0);
    SmoothPointCloud(partition, context.frame0, radius2Smoothing, neighborCountSmoothing,
                     radius2BoundaryDetection, thresholdSmoothing);
  }

  PCCVideo3B videoTexture;
  auto sizeTextureVideo = bitstream.size;
  videoTexture.decompress(path.str() + "texture", width, height, groupOfFramesSize * 2, bitstream,
                          params.videoDecoderPath, params.inverseColorSpaceConversionConfig,
                          params.colorSpaceConversionPath);
  sizeTextureVideo = bitstream.size - sizeTextureVideo;
  std::cout << "texture video  ->" << sizeTextureVideo << " B" << std::endl;

#ifndef POINT_CLOUD_CODEC_DUMP_INTERMEDIARY_RESULTS
  remove((path.str() + "texture.bin").c_str());
  remove((path.str() + "texture_rec.yuv").c_str());
  if (!params.colorSpaceConversionConfig.empty() && !params.colorSpaceConversionPath.empty() &&
      !params.inverseColorSpaceConversionConfig.empty()) {
    remove((path.str() + "texture_rec.rgb").c_str());
  }
#endif  // POINT_CLOUD_CODEC_DUMP_INTERMEDIARY_RESULTS

  for (size_t f = 0; f < groupOfFramesSize; ++f) {
    groupOfFrames[f] = contexts[f].frame0;
    ColorPointCloud(videoTexture, contexts[f].pointToPixel, 2 * f, groupOfFrames[f]);
    if (params.colorTransform == COLOR_TRANSFORM_RGB_TO_YCBCR) {
      groupOfFrames[f].convertYUVToRGB();
    }
  }

  return 0;
}

int DecompressVideo(const Parameters &params) {
  PCCBitstream bitstream = {};
  ifstream fin(params.compressedStreamPath, ios::binary);
  if (!fin.is_open()) {
    return -1;
  }
  fin.seekg(0, std::ios::end);
  uint64_t bitStreamSize = fin.tellg();
  fin.seekg(0, std::ios::beg);
  unique_ptr<uint8_t[]> buffer(new uint8_t[bitStreamSize]);
  bitstream.buffer = buffer.get();
  bitstream.capacity = bitStreamSize;
  bitstream.size = 0;
  fin.read(reinterpret_cast<char *>(bitstream.buffer), bitStreamSize);
  if (!fin) {
    return -1;
  }
  fin.close();

  uint32_t containerMagicNumber = 0;
  uint32_t containerVersion = 0;
  uint64_t totalSize = 0;
  PCCReadFromBuffer<uint32_t>(bitstream.buffer, containerMagicNumber, bitstream.size);
  if (containerMagicNumber != PCCTMC2ContainerMagicNumber) {
    return -1;
  }
  PCCReadFromBuffer<uint32_t>(bitstream.buffer, containerVersion, bitstream.size);
  if (containerVersion != PCCTMC2ContainerVersion) {
    return -1;
  }
  PCCReadFromBuffer<uint64_t>(bitstream.buffer, totalSize, bitstream.size);
  assert(bitStreamSize == totalSize);

  size_t frameNumber = params.startFrameNumber;
  size_t groupOfFramesIndex = 0;
  while (bitstream.size < bitstream.capacity) {
    vector<PCCPointSet3> groupOfFrames;
    if (int ret = DecompressGroupOfFrames(groupOfFramesIndex++, params, groupOfFrames, bitstream)) {
      return ret;
    }
    const size_t frameCount = groupOfFrames.size();
    char fileName[4096];
    for (size_t f = 0; f < frameCount; ++f) {
      sprintf(fileName, params.reconstructedDataPath.c_str(), frameNumber++);
      groupOfFrames[f].write(fileName);
    }
  }
  return 0;
}
