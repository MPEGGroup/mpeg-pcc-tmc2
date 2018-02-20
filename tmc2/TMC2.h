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

#ifndef TMC2_h
#define TMC2_h

#define _CRT_SECURE_NO_WARNINGS

#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>

#include "PCCPatchSegmenter.h"
#include "PCCPointSet.h"
#include "PCCPointSetProcessing.h"
#include "PCCTMC2Common.h"
#include "PCCVideo.h"

#include "ArithmeticCodec.h"
#include "KDTreeVectorOfVectorsAdaptor.h"
#include "tbb/tbb.h"

#include "TMC2Config.h"

#include "pcc_chrono.h"

enum ColorTransform { COLOR_TRANSFORM_NONE = 0, COLOR_TRANSFORM_RGB_TO_YCBCR = 1 };

enum CodecMode { CODEC_MODE_ENCODE = 0, CODEC_MODE_DECODE = 1 };

typedef std::vector<pcc::PCCPointSet3> GroupOfFrames;

struct Parameters {
  std::string uncompressedDataPath;
  std::string compressedStreamPath;
  std::string reconstructedDataPath;
  CodecMode mode;
  ColorTransform colorTransform;
  size_t frameCount;
  size_t startFrameNumber;
  size_t groupOfFramesSize;

  // segmentation
  size_t nnNormalEstimation;
  size_t maxNNCountRefineSegmentation;
  size_t iterationCountRefineSegmentation;
  size_t occupancyResolution;
  size_t minPointCountPerCCPatchSegmentation;
  size_t maxNNCountPatchSegmentation;
  size_t surfaceThickness;
  size_t maxAllowedDepth;
  double maxAllowedDist2MissedPointsDetection;
  double maxAllowedDist2MissedPointsSelection;
  double lambdaRefineSegmentation;

  // packing
  size_t minimumImageWidth;
  size_t minimumImageHeight;

  // video encoding
  size_t geometryQP;
  size_t textureQP;

  std::string colorSpaceConversionPath;
  std::string videoEncoderPath;
  std::string videoDecoderPath;

  std::string colorSpaceConversionConfig;
  std::string inverseColorSpaceConversionConfig;
  std::string geometryConfig;
  std::string textureConfig;

  // occupancy map encoding
  size_t maxCandidateCount;
  size_t occupancyPrecision;

  // smoothing
  size_t neighborCountSmoothing;
  double radius2Smoothing;
  double radius2BoundaryDetection;
  double thresholdSmoothing;

  // coloring
  size_t bestColorSearchRange;
};

typedef pcc::PCCImage<uint8_t, 3> PCCImage3B;
typedef pcc::PCCVideo<uint8_t, 3> PCCVideo3B;

struct EncodingContext {
  size_t groupOfFramesIndex;
  size_t width;
  size_t height;
  pcc::PCCPointSet3 frame0;
  std::vector<pcc::PCCVector3<size_t>> pointToPixel;
  std::vector<size_t> blockToPatch;
  std::vector<uint32_t> occupancyMap;
  std::vector<pcc::PCCPointCloudPatch> patches;
};

typedef pcc::chrono::Stopwatch<pcc::chrono::utime_inc_children_clock> Stopwatch;

bool ParseParameters(int argc, char *argv[], Parameters &params);
void Usage();
int CompressVideo(const Parameters &params, Stopwatch&);
int DecompressVideo(const Parameters &params, Stopwatch&);

#endif /* TMC2_h */
