
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
#include "PCCImage.h"
#include "PCCPointSet.h"
#include "tbb/tbb.h"
#include "PCCKdTree.h"
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCGroupOfFrames.h"
#include "PCCPatch.h"

#include "PCCCodec.h"

#define SMOOTH_POINT_CLOUD_MAX_SIZE 1024

using namespace pcc;

PCCCodec::PCCCodec() {}
PCCCodec::~PCCCodec() {}

void PCCCodec::addGridCentroid(PCCPoint3D& point, int patchIdx,
                               std::vector<int>& cnt, std::vector<PCCVector3D>& center_grid,
                               std::vector<int> &gpartition, std::vector<bool>& doSmooth, int grid) {

  const int w = SMOOTH_POINT_CLOUD_MAX_SIZE / grid;
  int x = (int)point.x() / grid;
  int y = (int)point.y() / grid;
  int z = (int)point.z() / grid;
  int idx = x + y*w + z*w*w;

  if (cnt[idx] == 0) {
    gpartition[idx] = patchIdx;
    center_grid[idx] = PCCVector3D(0, 0, 0);
    doSmooth[idx] = false;
  }
  else if (!doSmooth[idx] && gpartition[idx] != patchIdx) {
    doSmooth[idx] = true;
  }

  center_grid[idx] += point;
  cnt[idx]++;
}

void PCCCodec::generatePointCloud( PCCGroupOfFrames& reconstructs, PCCContext& context,
                                   const GeneratePointCloudParameters params) {
  if (!params.losslessGeo_) {
    if (params.gridSmoothing_) {
      //reset for each GOF
      const int w = SMOOTH_POINT_CLOUD_MAX_SIZE / ((int)params.occupancyPrecision_ * 2);
      center_grid_.resize(w*w*w);
      gcnt_.resize(w*w*w);
      gpartition_.resize(w*w*w);
      doSmooth_.resize(w*w*w);
    }
  }
  auto& frames = context.getFrames();
  auto &videoGeometry = context.getVideoGeometry();
  auto &videoGeometryD1 = context.getVideoGeometryD1();

  for (size_t i = 0; i < frames.size(); i++) {
    std::vector<uint32_t> partition;
    generatePointCloud(reconstructs[i], frames[i], videoGeometry, videoGeometryD1, params, partition);

    if (!params.losslessGeo_) {
      if (params.gridSmoothing_) {
        int size = (int)gcnt_.size();
        gcnt_.resize(0);
        gcnt_.resize(size, 0);
        for (int j = 0; j < reconstructs[i].getPointCount(); j++) {
          addGridCentroid(reconstructs[i][j], partition[j] + 1, gcnt_, center_grid_, gpartition_, doSmooth_, (int)params.occupancyPrecision_ * 2);
        }
        for (int i = 0; i < gcnt_.size(); i++) {
          if (gcnt_[i]) {
            center_grid_[i] /= gcnt_[i];
          }
        }
        smoothPointCloudGrid(reconstructs[i], partition, params);
      } else {
        smoothPointCloud(reconstructs[i], partition, params);
      }
    }
  }
}

bool PCCCodec::colorPointCloud( PCCGroupOfFrames& reconstructs, PCCContext& context,
                                const bool noAttributes, const ColorTransform colorTransform,
                                const GeneratePointCloudParameters params) {
  auto &video = context.getVideoTexture();
  auto& frames = context.getFrames();
  for (size_t i = 0; i < frames.size(); i++) {
    const size_t frameCount = video.getFrameCount() / frames.size();
    colorPointCloud(reconstructs[i], frames[i], video, noAttributes, params, frameCount);
    if (!params.losslessGeo_ && params.flagColorSmoothing_) {
      smoothPointCloudColor(reconstructs[i], params);
    }
    if (colorTransform == COLOR_TRANSFORM_RGB_TO_YCBCR) {
      reconstructs[i].convertYUVToRGB();
    }
  }
  return true;
}

int PCCCodec::getDeltaNeighbors( const PCCImageGeometry& frame,
                                 const PCCPatch& patch,
                                 const int xOrg,
                                 const int yOrg,
                                 const int neighboring,
                                 const int threshold,
                                 const bool projectionMode,
                                 const double lodScale ) {
  int deltaMax = 0;
  double dOrg = patch.generateNormalCoordinate( frame.getValue(0, xOrg, yOrg), lodScale, false, false );
  for (int x = (std::max)(0, xOrg - neighboring); x <= (std::min)(xOrg + neighboring, (int)frame.getWidth()); x += 1) {
    for (int y = (std::max)(0, yOrg - neighboring); y <= (std::min)(yOrg + neighboring, (int)frame.getHeight()); y += 1) {
      double dLoc = patch.generateNormalCoordinate( frame.getValue(0, x, y), lodScale, false, false );
      int delta = (int)( dLoc - dOrg );
      if( patch.getProjectionMode() == 0 ) {  
        if (delta <= threshold && delta > deltaMax ) {
          deltaMax = delta;
        }
      } else { 
        if( delta >= -(int)threshold && delta < deltaMax) {
          deltaMax = delta;
        }
      }
    }
  }
  deltaMax = deltaMax == 0 ? deltaMax : ( patch.getProjectionMode() == 0 ? deltaMax - 1 : deltaMax + 1 );
  return deltaMax;
}

void PCCCodec::identifyBoundaryPoints( const std::vector<uint32_t>& occupancyMap,
                                       const size_t x,
                                       const size_t y,
                                       const size_t imageWidth,
                                       const size_t imageHeight,
                                       const size_t pointindex_1,
                                       std::vector<uint32_t>& PBflag,
                                       PCCPointSet3& reconstruct ) {
  if (occupancyMap[y * imageWidth + x] != 0) {
    if (y > 0 && y < imageHeight - 1) {
      if (occupancyMap[(y - 1) * imageWidth + x] == 0 ||
          occupancyMap[(y + 1) * imageWidth + x] == 0) {
        PBflag[y * imageWidth + x] = 1;
        reconstruct.setBoundaryPointType(pointindex_1, static_cast<uint16_t>(1));
      }
    }
    if (x > 0 && x < imageWidth - 1) {
      if (occupancyMap[y * imageWidth + (x + 1)] == 0 ||
          occupancyMap[y * imageWidth + (x - 1)] == 0) {
        PBflag[y * imageWidth + x] = 1;
        reconstruct.setBoundaryPointType(pointindex_1, static_cast<uint16_t>(1));
      }
    }
    if (y > 0 && y < imageHeight - 1 && x > 0) {
      if (occupancyMap[(y - 1) * imageWidth + (x - 1)] == 0 ||
          occupancyMap[(y + 1) * imageWidth + (x - 1)] == 0) {
        PBflag[y * imageWidth + x] = 1;
        reconstruct.setBoundaryPointType(pointindex_1, static_cast<uint16_t>(1));
      }
    }
    if (y > 0 && y < imageHeight - 1 && x < imageWidth - 1) {
      if (occupancyMap[(y - 1) * imageWidth + (x + 1)] == 0 ||
          occupancyMap[(y + 1) * imageWidth + (x + 1)] == 0) {
        PBflag[y * imageWidth + x] = 1;
        reconstruct.setBoundaryPointType(pointindex_1, static_cast<uint16_t>(1));
      }
    }
  }

  // 1st Extension boundary region
  if (occupancyMap[y * imageWidth + x] != 0) {
    if (y > 0 && y < imageHeight - 1) {
      if (PBflag[(y - 1) * imageWidth + x] == 1 ||
          PBflag[(y + 1) * imageWidth + x] == 1) {
        reconstruct.setBoundaryPointType(pointindex_1, static_cast<uint16_t>(1));
      }
    }
    if (x > 0 && x < imageWidth - 1) {
      if (PBflag[y * imageWidth + (x + 1)] == 1 ||
          PBflag[y * imageWidth + (x - 1)] == 1) {
        reconstruct.setBoundaryPointType(pointindex_1, static_cast<uint16_t>(1));
      }
    }
    if (y > 0 && y < imageHeight - 1 && x > 0) {
      if (PBflag[(y - 1) * imageWidth + (x - 1)] == 1 ||
          PBflag[(y + 1) * imageWidth + (x - 1)] == 1) {
        reconstruct.setBoundaryPointType(pointindex_1, static_cast<uint16_t>(1));
      }
    }
    if (y > 0 && y < imageHeight - 1 && x < imageWidth - 1) {
      if (PBflag[(y - 1) * imageWidth + (x + 1)] == 1 ||
          PBflag[(y + 1) * imageWidth + (x + 1)] == 1) {
        reconstruct.setBoundaryPointType(pointindex_1, static_cast<uint16_t>(1));
      }
    }
  }
}

std::vector<PCCPoint3D> PCCCodec::generatePoints( const GeneratePointCloudParameters& params,
                                                  PCCFrameContext &frame,
                                                  const PCCVideoGeometry &video,
                                                  const PCCVideoGeometry &videoD1,
                                                  const size_t shift,
                                                  const size_t patchIndex,
                                                  const size_t u,
                                                  const size_t v,
                                                  const size_t x,
                                                  const size_t y,
                                                  const bool   interpolate,
                                                  const bool   filling    ,
                                                  const size_t minD1      ,
                                                  const size_t neighbor   ,
                                                  const double lodScale ) {
  const auto& patch = frame.getPatch( patchIndex );
  bool useMppSepVid = frame.getUseMissedPointsSeparateVideo();
  bool useAdditionalPointsPatch = frame.getUseAdditionalPointsPatch();
  bool lossyMpp = ! params.losslessGeo_ && useAdditionalPointsPatch;
  auto& frame0 = video.getFrame( shift );
  std::vector<PCCPoint3D> createdPoints;
  auto point0 = patch.generatePoint( u, v, frame0.getValue(0, x, y), lodScale, useMppSepVid, lossyMpp);
  createdPoints.push_back( point0 );
  if( params.singleLayerPixelInterleaving_ ) {
    size_t patchIndexPlusOne = patchIndex + 1;
    double depth0, depth1;
    auto& occupancyMap = frame.getOccupancyMap();
    auto& blockToPatch = frame.getBlockToPatch();
    const auto imageWidth  = frame0.getWidth();
    const auto imageHeight = frame0.getHeight();
    const size_t blockToPatchWidth  = frame.getWidth()  / params.occupancyResolution_;
    const size_t blockToPatchHeight = frame.getHeight() / params.occupancyResolution_;
    double DepthNeighbors[4] = { 0 };
    int count = 0;
    double minimumDepth = point0[patch.getNormalAxis()];
    double maximumDepth = point0[patch.getNormalAxis()];
    if (x > 0 && occupancyMap[y * imageWidth + x - 1]) {
      size_t Temp_u0 = (x - 1) / patch.getOccupancyResolution();
      size_t Temp_v0 = (y    ) / patch.getOccupancyResolution();
      if (blockToPatch[(Temp_v0) * blockToPatchWidth + Temp_u0] == patchIndexPlusOne) {
        if( patch.getProjectionMode() == 0 ) {
          DepthNeighbors[0] = double(frame0.getValue(0, x - 1, y) + patch.getD1()) * lodScale;
        } else {
          DepthNeighbors[0] = double(patch.getD1() - frame0.getValue(0, x - 1, y)) * lodScale;
        }
        count++;
        if (DepthNeighbors[0] < minimumDepth) {
          minimumDepth = DepthNeighbors[0];
        }
        if (DepthNeighbors[0] > maximumDepth) {
          maximumDepth = DepthNeighbors[0];
        }
      }
    }
    if (x < (imageWidth - 1) && occupancyMap[y * imageWidth + x + 1]) {
      size_t Temp_u0 = (x + 1) / patch.getOccupancyResolution();
      size_t Temp_v0 = (y    ) / patch.getOccupancyResolution();
      if (blockToPatch[(Temp_v0) * blockToPatchWidth + Temp_u0] == patchIndexPlusOne) {
        if( patch.getProjectionMode() == 0 ) {
          DepthNeighbors[1] = double(frame0.getValue(0, x + 1, y) + patch.getD1()) * lodScale;
        } else {
          DepthNeighbors[1] = double(patch.getD1() - frame0.getValue(0, x + 1, y)) * lodScale;
        }
        count++;
        if (DepthNeighbors[1] < minimumDepth) {
          minimumDepth = DepthNeighbors[1];
        }
        if (DepthNeighbors[1] > maximumDepth) {
          maximumDepth = DepthNeighbors[1];
        }
      }
    }
    if (y > 0 && occupancyMap[(y - 1) * imageWidth + x]) {
      size_t Temp_u0 = (x    ) / patch.getOccupancyResolution();
      size_t Temp_v0 = (y - 1) / patch.getOccupancyResolution();
      if (blockToPatch[(Temp_v0) * blockToPatchWidth + Temp_u0] == patchIndexPlusOne) {
        if( patch.getProjectionMode() == 0 ) {
          DepthNeighbors[2] = double(frame0.getValue(0, x, y - 1) + patch.getD1()) * lodScale;
        } else {
          DepthNeighbors[2] = double(patch.getD1() - frame0.getValue(0, x, y - 1)) * lodScale;
        }
        count++;
        if (DepthNeighbors[2] < minimumDepth) {
          minimumDepth = DepthNeighbors[2];
        }
        if (DepthNeighbors[2] > maximumDepth) {
          maximumDepth = DepthNeighbors[2];
        }
      }
    }
    if (y < (imageHeight - 1) && occupancyMap[(y + 1) * imageWidth + x]) {
      size_t Temp_u0 = (x    ) / patch.getOccupancyResolution();
      size_t Temp_v0 = (y + 1) / patch.getOccupancyResolution();
      if (blockToPatch[(Temp_v0) * blockToPatchWidth + Temp_u0] == patchIndexPlusOne) {
        if( patch.getProjectionMode() == 0 ) {
          DepthNeighbors[3] = double(frame0.getValue(0, x, y + 1) + patch.getD1()) * lodScale;
        } else {
          DepthNeighbors[3] = double(patch.getD1() - frame0.getValue(0, x, y + 1)) * lodScale;
        }
        count++;
        if (DepthNeighbors[3] < minimumDepth) {
          minimumDepth = DepthNeighbors[3];
        }
        if (DepthNeighbors[3] > maximumDepth) {
          maximumDepth = DepthNeighbors[3];
        }
      }
    }
    if ((x + y) % 2 == 1) {
      depth1 = point0[patch.getNormalAxis()];
      PCCPoint3D interpolateD0(point0);
      if( patch.getProjectionMode() == 0 ){
        interpolateD0[patch.getNormalAxis()] = round((std::min)((std::max)(minimumDepth, depth1 - params.surfaceThickness), depth1));
      } else {
        interpolateD0[patch.getNormalAxis()] = round((std::max)((std::min)(maximumDepth, depth1 + params.surfaceThickness), depth1));
      }
      depth0 = interpolateD0[patch.getNormalAxis()];
      createdPoints.push_back( interpolateD0 );
    } else {
      depth0 = point0[patch.getNormalAxis()];
      PCCPoint3D interpolateD1(point0);
      if( patch.getProjectionMode() == 0 ){
        interpolateD1[patch.getNormalAxis()] = round((std::max)((std::min)((DepthNeighbors[0] + DepthNeighbors[1] + DepthNeighbors[2] + DepthNeighbors[3]) / count, depth0 + params.surfaceThickness), depth0));
      } else {
        interpolateD1[patch.getNormalAxis()] = round((std::min)((std::max)((DepthNeighbors[0] + DepthNeighbors[1] + DepthNeighbors[2] + DepthNeighbors[3]) / count, depth0 - params.surfaceThickness), depth0));
      }
      depth1 = interpolateD1[patch.getNormalAxis()];
      createdPoints.push_back( interpolateD1 );
    }
    size_t xmin = (std::min)(depth0, depth1);
    size_t xmax = (std::max)(depth0, depth1);
    for (size_t step = 1; step < (xmax - xmin); ++step) {
      PCCPoint3D fillPoint(point0);
      fillPoint[patch.getNormalAxis()] = xmin + double(step);
      createdPoints.push_back( fillPoint );
    }
  } else {
    if( params.oneLayerMode_ ) {
      int deltaDepth = 0;
      if ( interpolate ) {
        deltaDepth = getDeltaNeighbors( frame0, patch, x, y, neighbor, NeighborThreshold, patch.getProjectionMode(), lodScale );
      }
      if (patch.getProjectionMode() == 0 ) {
        deltaDepth = (std::max)(deltaDepth, (int)minD1);
      } else {
        deltaDepth = (std::min)(deltaDepth, -(int)minD1);
      }
      if (deltaDepth != 0 ) {
        PCCPoint3D point1( point0 );
        point1[patch.getNormalAxis()] += (double)deltaDepth;
        createdPoints.push_back( point1 );
        if( filling ) {
          size_t xmin = (std::min)(point0[patch.getNormalAxis()], point1[patch.getNormalAxis()]);
          size_t xmax = (std::max)(point0[patch.getNormalAxis()], point1[patch.getNormalAxis()]);
          for (size_t value = xmin + 1; value < xmax; ++value) {
            PCCPoint3D pointFill(point0);
            pointFill[patch.getNormalAxis()] = double(value);
            createdPoints.push_back( pointFill );
          }
        }
      }
    } else { //else (oneLayerMode)
      if (!params.absoluteD1_) {
        PCCPoint3D point1( point0 );
        const auto &frame1 = videoD1.getFrame(shift);
        if (patch.getProjectionMode() == 0){
          point1[patch.getNormalAxis()] += frame1.getValue(0, x, y) * lodScale;
        } else{
          point1[patch.getNormalAxis()] -= frame1.getValue(0, x, y) * lodScale;
        }
        createdPoints.push_back( point1 );
      } else {
        const auto &frame1 = video.getFrame( 1 + shift);
        auto point1 = patch.generatePoint( u, v, frame1.getValue(0, x, y), lodScale, useMppSepVid, lossyMpp);
        createdPoints.push_back( point1 );
        if (( filling == 1)) {
          size_t xmin = (std::min)(point0[patch.getNormalAxis()], point1[patch.getNormalAxis()]);
          size_t xmax = (std::max)(point0[patch.getNormalAxis()], point1[patch.getNormalAxis()]);
          for (size_t value = xmin + 1; value < xmax; ++value) {
            PCCPoint3D pointFill(point0);
            pointFill[patch.getNormalAxis()] = double(value);
            createdPoints.push_back( pointFill );
          }
        }
      } // else !abosluteD1
    } //fi (oneLayerMode)
  } //fi (!params.singleLayerPixelInterleaving_ )
  return createdPoints;
}


void PCCCodec::generatePointCloud(PCCPointSet3& reconstruct, PCCFrameContext &frame,
                                  const PCCVideoGeometry &video, const PCCVideoGeometry &videoD1,
                                  const GeneratePointCloudParameters params,
                                  std::vector<uint32_t> &partition) {
  auto& patches = frame.getPatches();
  auto& pointToPixel = frame.getPointToPixel();
  auto& blockToPatch = frame.getBlockToPatch();
  auto& occupancyMap = frame.getOccupancyMap();
  partition.resize(0);
  pointToPixel.resize(0);
  reconstruct.clear();

  bool useMissedPointsSeparateVideo = frame.getUseMissedPointsSeparateVideo();
  bool useAdditionalPointsPatch = frame.getUseAdditionalPointsPatch();
  bool lossyMpp = !params.losslessGeo_ && useAdditionalPointsPatch;
  PCCPointSet3 eddSavedPoints;
  size_t numEddSavedPoints = 0;
  frame.getMissedPointsPatch().numEddSavedPoints = 0;  

  std::vector<size_t> vecEmptyBlocks;
  if (params.enhancedDeltaDepthCode_) {

    //jkei[!!]there should be more intelligent way!
    const size_t b2pWidth  = frame.getWidth() / params.occupancyResolution_;
    const size_t b2pHeight = frame.getHeight() / params.occupancyResolution_;
    
    for (size_t patchIndex = 0; patchIndex < patches.size(); ++patchIndex) {
      auto &patch = patches[patchIndex];
      size_t nonZeroPixel=0;
      for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
        for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
          const size_t blockIndex  = patch.patchBlock2CanvasBlock( u0, v0 ,b2pWidth, b2pHeight );
          nonZeroPixel=0;
          for (size_t v1 = 0; v1 < patch.getOccupancyResolution(); ++v1) {
            const size_t v = v0 * patch.getOccupancyResolution() + v1;
            for (size_t u1 = 0; u1 < patch.getOccupancyResolution(); ++u1) {
              const size_t u = u0 * patch.getOccupancyResolution() + u1;
              size_t x,y;
              nonZeroPixel += (occupancyMap[patch.patch2Canvas(u,v,video.getWidth(),video.getHeight(),x,y)] != 0);
            }//u1
          }//v1
          if(nonZeroPixel==0)
            blockToPatch[blockIndex] =0;
        }//u0
      }//v0
    }//patch
    
    for (size_t i = 0; i < blockToPatch.size(); i++){
      if (blockToPatch[i] == 0){
        vecEmptyBlocks.push_back(i);
      }
    }
  }
  size_t nUsedEmptyBlockCount = 0;
  size_t nPixelInCurrentBlockCount = 0;
  size_t nFrameToStore = 0;
  const size_t nPixelInBlockNum = params.occupancyResolution_ * params.occupancyResolution_;

  size_t shift;
  const size_t layerCount = (params.oneLayerMode_ && params.singleLayerPixelInterleaving_) ? 1 : 2;
  if (!params.absoluteD1_) {
    shift = frame.getIndex();
    if (video.getFrameCount() < (shift + 1)) {
      return;
    }
  }
  else {
    shift = frame.getIndex() * (params.oneLayerMode_ ? 1 : 2);
    if (video.getFrameCount() < (shift + (params.oneLayerMode_ ? 1 : 2))) {
      return;
    }
  }
  const auto &frame0 = video.getFrame(shift);
  const size_t imageWidth = video.getWidth();
  const size_t imageHeight = video.getHeight();
  std::vector<uint32_t> PBflag;
  PBflag.resize(imageWidth * imageHeight, 0);

  const size_t blockToPatchWidth  = frame.getWidth() / params.occupancyResolution_;
  const size_t blockToPatchHeight = frame.getHeight() / params.occupancyResolution_;
  reconstruct.addColors();
  const size_t patchCount = patches.size();
  size_t patchIndex {0};
  for (patchIndex = 0; patchIndex < patchCount; ++patchIndex) {
    const size_t patchIndexPlusOne = patchIndex + 1;
    auto &patch = patches[patchIndex];
    const double lodScale = params.ignoreLod_ ? 1.0 : double(1u << patch.getLod());
    PCCColor3B color(uint8_t(0));
    while (color[0] == color[1] || color[2] == color[1] || color[2] == color[0]) {
      color[0] = static_cast<uint8_t>(rand() % 32) * 8;
      color[1] = static_cast<uint8_t>(rand() % 32) * 8;
      color[2] = static_cast<uint8_t>(rand() % 32) * 8;
    }
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        const size_t blockIndex  = patch.patchBlock2CanvasBlock( u0, v0 ,blockToPatchWidth, blockToPatchHeight );
        if (blockToPatch[blockIndex] == patchIndexPlusOne) {
          for (size_t v1 = 0; v1 < patch.getOccupancyResolution(); ++v1) {
            const size_t v = v0 * patch.getOccupancyResolution() + v1;
            for (size_t u1 = 0; u1 < patch.getOccupancyResolution(); ++u1) {
              const size_t u = u0 * patch.getOccupancyResolution() + u1;
              size_t x,y;
              const bool occupancy = occupancyMap[patch.patch2Canvas(u,v,imageWidth,imageHeight,x,y)] != 0;
              if (!occupancy) {
                continue;
              }
              bool addPoint = true;
              if (params.enhancedDeltaDepthCode_) {
                //D0
                PCCPoint3D point0 = patch.generatePoint( u, v, frame0.getValue(0, x, y), lodScale, useMissedPointsSeparateVideo, lossyMpp);
                const size_t pointIndex0 = reconstruct.addPoint(point0);
                reconstruct.setColor(pointIndex0, color);
                if (PCC_SAVE_POINT_TYPE == 1) {
                  reconstruct.setType(pointIndex0, PointType::D0);
                }
                partition.push_back(uint32_t(patchIndex));
                pointToPixel.push_back(PCCVector3<size_t>(x, y, 0));

                uint16_t eddCode = 0;
                if (!params.absoluteD1_) {
				  if(!params.improveEDD_) {
                  const auto &frame1 = videoD1.getFrame(shift);
                  eddCode = frame1.getValue(0, x, y);
                } else {
					std::cout << "EDD improvement is not implemented for absoluteD1=0" << std::endl;
				  }
                } else {
                  const auto &frame0 = video.getFrame(shift);
                  const auto &frame1 = video.getFrame(shift + 1);
                  //eddCode = frame1.getValue(0, x, y) - frame0.getValue(0, x, y);
				  if (!params.improveEDD_)
                  eddCode = (frame1.getValue(0, x, y) > frame0.getValue(0, x, y)) ? (frame1.getValue(0, x, y) - frame0.getValue(0, x, y)) : 0;
				  else {
					uint16_t diff = frame1.getValue(0, x, y) - frame0.getValue(0, x, y);
					assert(diff >= 0);
					// Convert occupancy map to eddCode
					if (diff == 0) {
					  eddCode = 0;
                }
					else if (diff == 1) {
					  eddCode = 1;
					}
					else {
					  uint16_t bits = diff - 1;
					  uint16_t symbol = (1 << bits) - occupancyMap[patch.patch2Canvas(u,v,imageWidth,imageHeight,x,y)];
					  eddCode = symbol | (1 << bits);
					}
				  }
                }
                PCCPoint3D  point1(point0);
                if (eddCode == 0) {
                  if( !params.removeDuplicatePoints_ ){
                    const size_t pointIndex1 = reconstruct.addPoint(point1);
                    reconstruct.setColor(pointIndex1, color);
                    if (PCC_SAVE_POINT_TYPE == 1) {
                      reconstruct.setType(pointIndex1, PointType::D1);
                    }
                    partition.push_back(uint32_t(patchIndex));
                    pointToPixel.push_back(PCCVector3<size_t>(x, y, 1));
                  }
                } else { //eddCode != 0
                  uint16_t addedPointCount = 0;
                  size_t   pointIndex1 = 0;
                  //for (uint16_t i = 0; i < surfaceThickness; i++) 
                  for (uint16_t i = 0; i < 10; i++) {
                    if (eddCode & (1 << i)) {
                      uint8_t deltaDCur = (i + 1);
                      if (patch.getProjectionMode() == 0) {
                        point1[patch.getNormalAxis()] = (double)(point0[patch.getNormalAxis()] + deltaDCur);
                      } else {
                        point1[patch.getNormalAxis()] = (double)(point0[patch.getNormalAxis()] - deltaDCur);
                      }
                      if (!useMissedPointsSeparateVideo) {
                        pointIndex1 = reconstruct.addPoint(point1);
                        reconstruct.setColor(pointIndex1, color);
                        if (PCC_SAVE_POINT_TYPE == 1) {
                          reconstruct.setType(pointIndex1, PointType::InBetween);
                        }
                        partition.push_back(uint32_t(patchIndex));
                      }
                      if (addedPointCount == 0) {
                        if (useMissedPointsSeparateVideo) {
                          pointIndex1 = reconstruct.addPoint(point1);
                          reconstruct.setColor(pointIndex1, color);
                          if (PCC_SAVE_POINT_TYPE == 1) {
                            reconstruct.setType(pointIndex1, PointType::InBetween);
                          }
                          partition.push_back(uint32_t(patchIndex));
                        }
                        pointToPixel.push_back(PCCVector3<size_t>(x, y, 1));
                      } else {
                        if (useMissedPointsSeparateVideo) {
                          numEddSavedPoints++;
                          eddSavedPoints.addPoint(point1);
                        } else {
                          size_t uBlock = vecEmptyBlocks[nUsedEmptyBlockCount] % blockToPatchWidth;
                          size_t vBlock = vecEmptyBlocks[nUsedEmptyBlockCount] / blockToPatchWidth;
                          size_t uu = uBlock * params.occupancyResolution_ + nPixelInCurrentBlockCount % params.occupancyResolution_;
                          size_t vv = vBlock * params.occupancyResolution_ + nPixelInCurrentBlockCount / params.occupancyResolution_;
                          pointToPixel.push_back(PCCVector3<size_t>(uu, vv, nFrameToStore));
                          occupancyMap[vv * imageWidth + uu] = 1; //occupied

                          ++nPixelInCurrentBlockCount;
                          if (nPixelInCurrentBlockCount >= nPixelInBlockNum) {
                            nUsedEmptyBlockCount++;
                            nPixelInCurrentBlockCount = 0;
                            if (nUsedEmptyBlockCount >= vecEmptyBlocks.size()) {
                              if (nFrameToStore == 0) {
                                nFrameToStore = 1;
                                nUsedEmptyBlockCount = 0;
                              } else {
                                std::cout << "Enhanced delta depth: for packing the color of in-between points, all empty blocks are used up. Need to enlarge image size. Not implemented... Exit for the moment...\n";
                                exit(-1);
                              }
                            }
                          }
                        }//useMissedPointsSeparateVideo
                      }
                      addedPointCount++;
                    }
                  } //for each bit of EDD code
                  if (PCC_SAVE_POINT_TYPE == 1) {
                    reconstruct.setType(pointIndex1, PointType::D1);
                  }
                  //Without "Identify boundary points" & "1st Extension boundary region" as EDD code is only for lossless coding now
                } // if (eddCode == 0) 
              } else {//not params.enhancedDeltaDepthCode_
                auto createdPoints = generatePoints( params, frame, video, videoD1, shift,
                                                     patchIndex,
                                                     u, v,
                                                     x, y,
                                                     frame.getInterpolate()[blockIndex],
                                                     frame.getFilling    ()[blockIndex],
                                                     frame.getMinD1      ()[blockIndex],
                                                     frame.getNeighbor   ()[blockIndex],
                                                     lodScale );
                if ( createdPoints.size() > 0 ) {
                  for(size_t i = 0; i<createdPoints.size();i++) {
                    if( ( !params.removeDuplicatePoints_ ) ||
                        ( ( i == 0 ) || ( createdPoints[i] != createdPoints[0] ) ) ) {
                      const size_t pointindex_1 = reconstruct.addPoint(createdPoints[i]);
                      reconstruct.setColor(pointindex_1, color);
                      if (PCC_SAVE_POINT_TYPE == 1) {
                        if (params.singleLayerPixelInterleaving_){
                          size_t flag;
                          flag = (i == 0) ? (x + y) % 2 : (i == 1) ? (x + y + 1) % 2 : IntermediateLayerIndex;
                          reconstruct.setType(pointindex_1, flag == 0 ? PointType::D0 : flag == 1 ? PointType::D1 : PointType::DF);
                        } else {
                          reconstruct.setType(pointindex_1, i == 0 ? PointType::D0 :
                              i == 1 ? PointType::D1 : PointType::DF );
                        }
                      }
                      partition.push_back(uint32_t(patchIndex));
                      if (params.singleLayerPixelInterleaving_){
                        pointToPixel.push_back(PCCVector3<size_t>(x, y, i == 0 ? ((size_t)( x + y     ) % 2 ) :
                            i == 1 ? ((size_t)( x + y + 1 ) % 2 ) : IntermediateLayerIndex ) );
                      } else if(params.oneLayerMode_) {
                        pointToPixel.push_back(PCCVector3<size_t>(x, y, i == 0 ? 0 : i == 1 ? IntermediateLayerIndex : IntermediateLayerIndex + 1 ) );
                      } else {
                        pointToPixel.push_back(PCCVector3<size_t>(x, y, i < 2 ? i : IntermediateLayerIndex + 1 ));
                      }
                      identifyBoundaryPoints( occupancyMap, x, y, imageWidth, imageHeight, pointindex_1, PBflag, reconstruct );
                    }
                  }
                }
              } //fi (params.enhancedDeltaDepthCode_)
            }
          }
        }
      }
    }
  }

  if (params.useAdditionalPointsPatch_){
    if (useMissedPointsSeparateVideo) {
      PCCColor3B missedPointsColor(uint8_t(0));
      missedPointsColor[0] = 0;
      missedPointsColor[1] = 255;
      missedPointsColor[2] = 255;
      if (frame.getEnhancedDeltaDepth()) {
        //add Edd to reconstruct
        frame.getMissedPointsPatch().numEddSavedPoints = numEddSavedPoints;
        assert(eddSavedPoints.getPointCount() == numEddSavedPoints);
        size_t pointIndex1;
        for (size_t eddPoint = 0; eddPoint < numEddSavedPoints; eddPoint++) {
          PCCPoint3D  point1;
          point1 = eddSavedPoints[eddPoint];
          pointIndex1 = reconstruct.addPoint(point1);
          reconstruct.setColor(pointIndex1, missedPointsColor);
        }
      } //enhanced

      //Add point GPS from missedPointsPatch without inserting to pointToPixel
      auto& missedPointsPatch = frame.getMissedPointsPatch();
      if (params.losslessGeo444_) {
        for (int i = 0; i < missedPointsPatch.size(); i++) {
          PCCPoint3D point0;
          point0[0] = missedPointsPatch.x[i];
          point0[1] = missedPointsPatch.y[i];
          point0[2] = missedPointsPatch.z[i];
          const size_t pointIndex = reconstruct.addPoint(point0);
          reconstruct.setColor(pointIndex, missedPointsColor);
          partition.push_back(uint32_t(patchIndex)); 
        }
      } else { //else losslessGeo444_
        assert(missedPointsPatch.size() % 3 == 0);
        size_t sizeofMPs = (missedPointsPatch.size()) / 3;
        for (int i = 0; i < sizeofMPs; i++) {
          PCCPoint3D point0;
          point0[0] = missedPointsPatch.x[i];
          point0[1] = missedPointsPatch.x[i + sizeofMPs];
          point0[2] = missedPointsPatch.x[i + 2 * sizeofMPs];
          const size_t pointIndex = reconstruct.addPoint(point0);
          reconstruct.setColor(pointIndex, missedPointsColor);
          partition.push_back(uint32_t(patchIndex)); 
        }
      } //fi losslessGeo444_

    if (frame.getLosslessAtt() || lossyMpp) {
      //secure the size of rgb in missedpointpatch
      auto& missedPointsPatch = frame.getMissedPointsPatch();
      size_t numofMPcolor = (params.losslessGeo444_) ? missedPointsPatch.size() : missedPointsPatch.size() / 3;
      size_t numofEddSaved = frame.getEnhancedDeltaDepth() ? missedPointsPatch.numEddSavedPoints : 0;
      missedPointsPatch.resizecolor(numofMPcolor + numofEddSaved);
      missedPointsPatch.setMPnumbercolor(numofMPcolor + numofEddSaved);
      assert(numEddSavedPoints == numofEddSaved);
    }
  } else { //else useMissedPointsSeparateVideo
    // Add points from missedPointsPatch
    auto& missedPointsPatch = frame.getMissedPointsPatch();

    PCCColor3B missedPointsColor(uint8_t(0));
    missedPointsColor[0] = 0;
    missedPointsColor[1] = 255;
    missedPointsColor[2] = 255;
    size_t numMissedPts = missedPointsPatch.numMissedPts;
    size_t occupancySizeU = blockToPatchWidth;
    size_t ores = missedPointsPatch.occupancyResolution;
    size_t factor = params.losslessGeo444_ ? 1 : 3;
    size_t missedPointsPatchBlocks =
      static_cast<size_t>(ceil(double(numMissedPts * factor) / (ores * ores)));
    missedPointsPatch.sizeV0 =
      static_cast<size_t>(ceil(double(missedPointsPatchBlocks) / occupancySizeU));
    missedPointsPatch.sizeU0 =
      static_cast<size_t>(ceil(double(missedPointsPatchBlocks) / missedPointsPatch.sizeV0));
    missedPointsPatch.sizeV = missedPointsPatch.sizeV0 * ores;
    missedPointsPatch.sizeU =
      static_cast<size_t>(ceil(double(numMissedPts * factor) / missedPointsPatch.sizeV));

    if (params.losslessGeo_ && params.losslessGeo444_) {
      for (size_t v0 = 0; v0 < missedPointsPatch.sizeV0; ++v0) {
        for (size_t u0 = 0; u0 < missedPointsPatch.sizeU0; ++u0) {
          for (size_t v1 = 0; v1 < missedPointsPatch.occupancyResolution; ++v1) {
            const size_t v = v0 * missedPointsPatch.occupancyResolution + v1;
            for (size_t u1 = 0; u1 < missedPointsPatch.occupancyResolution; ++u1) {
              const size_t u = u0 * missedPointsPatch.occupancyResolution + u1;
              const size_t x = missedPointsPatch.u0 * missedPointsPatch.occupancyResolution + u;
              const size_t y = missedPointsPatch.v0 * missedPointsPatch.occupancyResolution + v;
              const bool occupancy = occupancyMap[y * imageWidth + x] != 0;
              if (!occupancy) {
                continue;
              }
              PCCPoint3D point0;
              point0[0] = double(frame0.getValue(0, x, y));
              point0[1] = double(frame0.getValue(1, x, y));
              point0[2] = double(frame0.getValue(2, x, y));
              const size_t pointIndex = reconstruct.addPoint(point0);
              reconstruct.setColor(pointIndex, missedPointsColor);
              for (size_t f = 0; f < layerCount; ++f) {
                partition.push_back(uint32_t(patchIndex));
                pointToPixel.push_back(PCCVector3<size_t>(x, y, f));
              }
            }
          }
        }
      }
    } else if (params.losslessGeo_ || lossyMpp) {
      std::vector<PCCPoint3D> missedPoints;
      missedPoints.resize(numMissedPts);
      size_t numMissedPointsAdded{ 0 };
      const size_t v0 = missedPointsPatch.v0 * missedPointsPatch.occupancyResolution;
      const size_t u0 = missedPointsPatch.u0 * missedPointsPatch.occupancyResolution;
      for (size_t v = 0; v < missedPointsPatch.sizeV; ++v) {
        for (size_t u = 0; u < missedPointsPatch.sizeU; ++u) {
          const size_t x = (u0 + u);
          const size_t y = (v0 + v);
            if (numMissedPointsAdded < numMissedPts) {
              missedPoints[numMissedPointsAdded][0] = double(frame0.getValue(0, x, y));
            }
            else if (numMissedPts <= numMissedPointsAdded && numMissedPointsAdded < 2 * numMissedPts) {
              missedPoints[numMissedPointsAdded - numMissedPts][1] = double(frame0.getValue(0, x, y));
            }
            else if (2 * numMissedPts <= numMissedPointsAdded && numMissedPointsAdded < 3 * numMissedPts) {
              missedPoints[numMissedPointsAdded - 2 * numMissedPts][2] = double(frame0.getValue(0, x, y));
            }
            else {
            }
            numMissedPointsAdded++;
          }//u
        } //v
        size_t counter{ 0 };
        for (size_t v = 0; v < missedPointsPatch.sizeV; ++v) {
          for (size_t u = 0; u < missedPointsPatch.sizeU; ++u) {
            const size_t x = (u0 + u);
            const size_t y = (v0 + v);
            if (counter < numMissedPts) {
              const size_t pointIndex = reconstruct.addPoint(missedPoints[counter]);
              reconstruct.setColor(pointIndex, missedPointsColor);
              for (size_t f = 0; f < layerCount; ++f) {
                partition.push_back(uint32_t(patchIndex));
                pointToPixel.push_back(PCCVector3<size_t>(x, y, f));
              }
              counter++;
            }
          }
        }
      }
    }//fi :useMissedPointsSeparateVideo
  } // fi : useAdditionalPointsPatch
}

bool PCCCodec::gridFiltering( const std::vector<uint32_t> &partition, PCCPointSet3 &pointCloud,
                              PCCPoint3D& curPoint, PCCVector3D& centroid, int &cnt,
                              std::vector<int>& gcnt, std::vector<PCCVector3D>& center_grid, std::vector<bool>& doSmooth, int grid ) {
  const int w = SMOOTH_POINT_CLOUD_MAX_SIZE / grid;
  bool otherClusterPointCount = false;

  int x = curPoint.x();
  int y = curPoint.y();
  int z = curPoint.z();

  int x2 = x / grid;
  int y2 = y / grid;
  int z2 = z / grid;

  int curIdx = x2 + y2*w + z2*w*w;

  int x3 = x % grid;
  int y3 = y % grid;
  int z3 = z % grid;

  int sx = x2 + ((x3 < grid / 2) ? -1 : 0);
  int sy = y2 + ((y3 < grid / 2) ? -1 : 0);
  int sz = z2 + ((z3 < grid / 2) ? -1 : 0);

  int sx2 = sx * grid;
  int sy2 = sy * grid;
  int sz2 = sz * grid;

  int wx = (x - sx2 - grid / 2) * 2 + 1;
  int wy = (y - sy2 - grid / 2) * 2 + 1;
  int wz = (z - sz2 - grid / 2) * 2 + 1;

  int idx[2][2][2];
  for (int dz = 0; dz < 2; dz++) {
    for (int dy = 0; dy < 2; dy++) {
      for (int dx = 0; dx < 2; dx++) {
        int x3 = sx + dx;
        int y3 = sy + dy;
        int z3 = sz + dz;
        int tmp = x3 + y3*w + z3*w*w;
        idx[dz][dy][dx] = tmp;
        if (doSmooth[tmp] && gcnt[tmp]) {
          otherClusterPointCount = true;
        }
      }
    }
  }

  if (!otherClusterPointCount) {
    return otherClusterPointCount;
  }
  PCCVector3D centroid3[2][2][2] = {};
  PCCVector3D curVector(x,y,z);
  int grid2 = grid * 2;

  if (gcnt[idx[0][0][0]] > 0) centroid3[0][0][0] = center_grid[idx[0][0][0]]; else centroid3[0][0][0] = curVector;
  if (gcnt[idx[0][0][1]] > 0) centroid3[0][0][1] = center_grid[idx[0][0][1]]; else centroid3[0][0][1] = curVector;
  if (gcnt[idx[0][1][0]] > 0) centroid3[0][1][0] = center_grid[idx[0][1][0]]; else centroid3[0][1][0] = curVector;
  if (gcnt[idx[0][1][1]] > 0) centroid3[0][1][1] = center_grid[idx[0][1][1]]; else centroid3[0][1][1] = curVector;
  if (gcnt[idx[1][0][0]] > 0) centroid3[1][0][0] = center_grid[idx[1][0][0]]; else centroid3[1][0][0] = curVector;
  if (gcnt[idx[1][0][1]] > 0) centroid3[1][0][1] = center_grid[idx[1][0][1]]; else centroid3[1][0][1] = curVector;
  if (gcnt[idx[1][1][0]] > 0) centroid3[1][1][0] = center_grid[idx[1][1][0]]; else centroid3[1][1][0] = curVector;
  if (gcnt[idx[1][1][1]] > 0) centroid3[1][1][1] = center_grid[idx[1][1][1]]; else centroid3[1][1][1] = curVector;

  centroid3[0][0][0] = (grid2 - wx)*(grid2 - wy)*(grid2 - wz)*centroid3[0][0][0];
  centroid3[0][0][1] = (wx)*(grid2 - wy)*(grid2 - wz)*centroid3[0][0][1];
  centroid3[0][1][0] = (grid2 - wx)*(wy)*(grid2 - wz)*centroid3[0][1][0];
  centroid3[0][1][1] = (wx)*(wy)*(grid2 - wz)*centroid3[0][1][1];
  centroid3[1][0][0] = (grid2 - wx)*(grid2 - wy)*(wz)*centroid3[1][0][0];
  centroid3[1][0][1] = (wx)*(grid2 - wy)*(wz)*centroid3[1][0][1];
  centroid3[1][1][0] = (grid2 - wx)*(wy)*(wz)*centroid3[1][1][0];
  centroid3[1][1][1] = (wx)*(wy)*(wz)*centroid3[1][1][1];

  PCCVector3D centroid4;
  centroid4 = centroid3[0][0][0] + centroid3[0][0][1] + centroid3[0][1][0] + centroid3[0][1][1] +
      centroid3[1][0][0] + centroid3[1][0][1] + centroid3[1][1][0] + centroid3[1][1][1];
  centroid4 /= grid2 * grid2 * grid2;

  cnt = 0;
  cnt += (grid2 - wx)*(grid2 - wy)*(grid2 - wz)*gcnt[idx[0][0][0]];
  cnt += (wx)*(grid2 - wy)*(grid2 - wz)*gcnt[idx[0][0][1]];
  cnt += (grid2 - wx)*(wy)*(grid2 - wz)*gcnt[idx[0][1][0]];
  cnt += (wx)*(wy)*(grid2 - wz)*gcnt[idx[0][1][1]];
  cnt += (grid2 - wx)*(grid2 - wy)*(wz)*gcnt[idx[1][0][0]];
  cnt += (wx)*(grid2 - wy)*(wz)*gcnt[idx[1][0][1]];
  cnt += (grid2 - wx)*(wy)*(wz)*gcnt[idx[1][1][0]];
  cnt += (wx)*(wy)*(wz)*gcnt[idx[1][1][1]];
  cnt /= grid2*grid2*grid2;

  centroid = centroid4*cnt;
  return otherClusterPointCount;
}

void PCCCodec::smoothPointCloudGrid(PCCPointSet3& reconstruct,
                                    const std::vector<uint32_t> &partition,
                                    const GeneratePointCloudParameters params) {
  const size_t pointCount = reconstruct.getPointCount();
  const int disth = (int)params.occupancyPrecision_;
  const int grid = (int)params.occupancyPrecision_ * 2;
  for (int c = 0; c < pointCount; c++) {
    PCCPoint3D curPoint = reconstruct[c];
    int x = (int)curPoint.x();
    int y = (int)curPoint.y();
    int z = (int)curPoint.z();
    if ( x < disth || y < disth || z < disth ||
        SMOOTH_POINT_CLOUD_MAX_SIZE <= x + disth || 
        SMOOTH_POINT_CLOUD_MAX_SIZE <= y + disth || 
        SMOOTH_POINT_CLOUD_MAX_SIZE <= z + disth) {
      continue;
    }
    PCCVector3D centroid(0.0), curVector(x,y,z);
    int cnt = 0;
    bool otherClusterPointCount = false;
    if (reconstruct.getBoundaryPointType(c) == 1)  {
      otherClusterPointCount = gridFiltering(partition, reconstruct, curPoint, centroid, cnt, gcnt_, center_grid_, doSmooth_, grid);
    }
    if (otherClusterPointCount) {
      double dist2 = ((curVector*cnt - centroid).getNorm2() + (double)cnt / 2.0) / (double)cnt;
      if (dist2 >= (std::max)((int)params.thresholdSmoothing_, (int)cnt) * 2) {
        centroid = (centroid + (double)cnt / 2.0) / (double)cnt;
        for (size_t k = 0; k < 3; ++k) {
          centroid[k] = double(int64_t(centroid[k]));
        }
        // reconstruct[c] = centroid;
        reconstruct[c][0] = centroid[0];
        reconstruct[c][1] = centroid[1];
        reconstruct[c][2] = centroid[2];
      }
    }
  }
}

void PCCCodec::smoothPointCloud(PCCPointSet3& reconstruct,
                                const std::vector<uint32_t> &partition,
                                const GeneratePointCloudParameters params) {
  const size_t pointCount = reconstruct.getPointCount();
  PCCKdTree kdtree( reconstruct );
  PCCPointSet3 temp;
  temp.resize(pointCount);
  tbb::task_arena limited((int)params.nbThread_);
  limited.execute([&] {
    tbb::parallel_for(size_t(0), pointCount, [&](const size_t i) {
      //  for (size_t i = 0; i < pointCount; ++i) {
      const size_t clusterindex_ = partition[i];
      PCCNNResult result;
      kdtree.searchRadius( reconstruct[i], params.neighborCountSmoothing_, params.radius2Smoothing_, result );
      PCCVector3D centroid(0.0);
      bool otherClusterPointCount = false;
      size_t neighborCount = 0;
      for (size_t r = 0; r < result.count(); ++r) {
         const double& dist2 = result.dist(r);
        ++neighborCount;
        const size_t pointindex_ = result.indices(r);
        centroid += reconstruct[pointindex_];
        otherClusterPointCount |=
            (dist2 <= params.radius2BoundaryDetection_) && (partition[pointindex_] != clusterindex_);
      }

      if (otherClusterPointCount) {
        if (reconstruct.getBoundaryPointType(i) == 1) {
          reconstruct.setBoundaryPointType(i, static_cast<uint16_t>(2));
        }
        const PCCVector3D sclaedPoint = double(neighborCount) * PCCVector3D( reconstruct[i][0], reconstruct[i][1], reconstruct[i][2] );
        const double distToCentroid2 =
            int64_t((centroid - sclaedPoint).getNorm2() + (neighborCount / 2.0)) /
            double(neighborCount);
        for (size_t k = 0; k < 3; ++k) {
          centroid[k] = double(int64_t( ( centroid[k] + (neighborCount / 2)) / neighborCount));
        }
        if (distToCentroid2 >= params.thresholdSmoothing_) {
          temp[i][0] = centroid[0];
          temp[i][1] = centroid[1];
          temp[i][2] = centroid[2];
          reconstruct.setColor(i, PCCColor3B(255, 0, 0));
          if (PCC_SAVE_POINT_TYPE == 1) {
            reconstruct.setType(i, PointType::Smooth);
          }
        } else {
          temp[i] = reconstruct[i];
        }
      } else {
        temp[i] = reconstruct[i];
      }
    });
  });
  limited.execute([&] {
    tbb::parallel_for(size_t(0), pointCount, [&](const size_t i) {
      // for (size_t i = 0; i < pointCount; ++i) {
      reconstruct[i] = temp[i];
    });
  });
}

void PCCCodec::smoothPointCloudColor(PCCPointSet3 &reconstruct,
                                     const GeneratePointCloudParameters params) {
  const size_t pointCount = reconstruct.getPointCount();
  PCCKdTree kdtree( reconstruct );
  PCCPointSet3 temp;
  temp.resize(pointCount);
  tbb::task_arena limited((int)params.nbThread_);
  limited.execute([&] {
    tbb::parallel_for(size_t(0), pointCount, [&](const size_t i) {
      //  for (size_t i = 0; i < pointCount; ++i) {

      PCCNNResult result;
      if (reconstruct.getBoundaryPointType(i) == 2) {
        kdtree.searchRadius( reconstruct[i], params.neighborCountColorSmoothing_, params.radius2ColorSmoothing_, result );
        PCCVector3D centroid(0.0);

        size_t neighborCount = 0;
        std::vector<uint8_t> Lum;

        for (size_t r = 0; r < result.count(); ++r) {
          ++neighborCount;
          const size_t index = result.indices(r);
          PCCColor3B color = reconstruct.getColor(index);
          centroid[0] += double(color[0]);
          centroid[1] += double(color[1]);
          centroid[2] += double(color[2]);

          double Y =
              0.2126 * double(color[0]) + 0.7152 * double(color[1]) + 0.0722 * double(color[2]);
          Lum.push_back(uint8_t(Y));
        }

        PCCColor3B color;
        if (neighborCount) {
          for (size_t k = 0; k < 3; ++k) {
            centroid[k] = double(int64_t(centroid[k] + (neighborCount / 2)) / neighborCount);
          }

          // Texture characterization
          double H = entropy(Lum, int(neighborCount));
          PCCColor3B colorQP = reconstruct.getColor(i);
          double distToCentroid2 = 0;
          for (size_t k = 0; k < 3; ++k) {
            distToCentroid2 += abs(centroid[k] - double(colorQP[k]));
          }
          if (distToCentroid2 >= double(params.thresholdColorSmoothing_) &&
              H < double(params.thresholdLocalEntropy_)) {
            color[0] = uint8_t(centroid[0]);
            color[1] = uint8_t(centroid[1]);
            color[2] = uint8_t(centroid[2]);
            reconstruct.setColor(i, color);
          }
        }
      }
    });
  });
}

void PCCCodec::createSpecificLayerReconstruct(const PCCPointSet3&                 reconstruct,
                                              const std::vector<uint32_t>&        partition,
                                              PCCFrameContext&                    frame,
                                              const GeneratePointCloudParameters& params,
                                              const size_t                        frameCount,
                                              PCCPointSet3&                       subReconstruct,
                                              std::vector<uint32_t>&              subPartition,
                                              std::vector<size_t>&                subReconstructIndex) {
  subReconstruct.clear();
  subPartition.clear();
  subReconstructIndex.clear();

  auto& pointToPixel = frame.getPointToPixel();
  const size_t pointCount = reconstruct.getPointCount();
  if (!pointCount || !reconstruct.hasColors()) {
    return;
  }
  for (size_t i = 0; i < pointCount; ++i) {
    const PCCVector3<size_t> location = pointToPixel[i];
    const size_t x = location[0];
    const size_t y = location[1];
    const size_t f = location[2];
    if (f == frameCount) {
      int index = subReconstruct.addPoint(reconstruct[i]);
      subPartition.push_back(partition[i]);
      subReconstructIndex.push_back(i);

    }
  }
  subReconstruct.addColors();
}

void PCCCodec::createSubReconstruct(const PCCPointSet3&                 reconstruct,
                                    const std::vector<uint32_t>&        partition,
                                    PCCFrameContext&                    frame,
                                    const GeneratePointCloudParameters& params,
                                    const size_t                        frameCount,
                                    PCCPointSet3&                       subReconstruct,
                                    std::vector<uint32_t>&              subPartition,
                                    std::vector<size_t>&                subReconstructIndex) {
  subReconstruct.clear();
  subPartition.clear();
  subReconstructIndex.clear();
  auto& pointToPixel = frame.getPointToPixel();
  const size_t pointCount = reconstruct.getPointCount();
  if (!pointCount || !reconstruct.hasColors()) {
    return;
  }
  for (size_t i = 0; i < pointCount; ++i) {
    const PCCVector3<size_t> location = pointToPixel[i];
    const size_t x = location[0];
    const size_t y = location[1];
    const size_t f = location[2];
    if (f < frameCount) {
      int index = subReconstruct.addPoint(reconstruct[i]);
      subReconstruct.setType(frameCount, PointType::Unset);
      subPartition.push_back(partition[i]);
      subReconstructIndex.push_back(i);
    }
  }
  subReconstruct.addColors();
}


void PCCCodec::updateReconstruct(PCCPointSet3&              reconstruct,
                                 const PCCPointSet3&        subReconstruct,
                                 const std::vector<size_t>& subReconstructIndex) {
  if (subReconstruct.getPointCount() > 0) {
    for (size_t i = 0; i < subReconstruct.getPointCount(); ++i) {
      reconstruct[subReconstructIndex[i]] = subReconstruct[i];
    }
  }
}
bool PCCCodec::colorPointCloud(PCCPointSet3& reconstruct, PCCFrameContext& frame,
                               const PCCVideoTexture &video, const bool noAttributes,
                               const GeneratePointCloudParameters& params,
                               const size_t frameCount) {

  printf("Start colorPointCloud \n"); fflush(stdout);
  if (noAttributes) {
    for (auto& color : reconstruct.getColors()) {
      for (size_t c = 0; c < 3; ++c) {
        color[c] = static_cast<uint8_t>(127);
      }
    }
  } else {
    auto& pointToPixel = frame.getPointToPixel();
    auto& color = reconstruct.getColors();
    bool useMissedPointsSeparateVideo = frame.getUseMissedPointsSeparateVideo();
    bool losslessAtt = frame.getLosslessAtt();
    bool losslessGeo = frame.getLosslessGeo();
    bool useAdditionalPointsPatch = frame.getUseAdditionalPointsPatch();
    bool lossyMissedPointsPatch = ! losslessGeo && useAdditionalPointsPatch; 
    auto& missedPointsPatch = frame.getMissedPointsPatch();
    size_t numOfMPColor = missedPointsPatch.getMPnumbercolor();
    size_t numOfMPGeos = missedPointsPatch.getMPnumber();
    size_t numEddSavedPoints = missedPointsPatch.numEddSavedPoints;
    size_t pointCount = reconstruct.getPointCount();
    if ((losslessAtt || lossyMissedPointsPatch) && useMissedPointsSeparateVideo) {
      pointCount = reconstruct.getPointCount() - numOfMPGeos - numEddSavedPoints;
      assert(numOfMPColor == (numEddSavedPoints + numOfMPGeos));
      missedPointsPatch.resizecolor(numOfMPColor);
    }

    if (!pointCount || !reconstruct.hasColors()) {
      return false;
    }
    PCCPointSet3   target;
    PCCPointSet3   source;
    std::vector<size_t> targetIndex;
    targetIndex.resize(0);
    target.clear();
    source.clear();
    target.addColors();
    source.addColors();
    const size_t shift = frame.getIndex() * frameCount;
    for (size_t i = 0; i < pointCount; ++i) {
      const PCCVector3<size_t> location = pointToPixel[i];
      const size_t x = location[0];
      const size_t y = location[1];
      const size_t f = location[2];
      if (params.oneLayerMode_ && params.singleLayerPixelInterleaving_) {
        if ((f == 0 && (x + y) % 2 == 0) | (f == 1 && (x + y) % 2 == 1)) {
          const auto &frame = video.getFrame(shift);
          for (size_t c = 0; c < 3; ++c) {
            color[i][c] = frame.getValue(c, x, y);
          }
          int index = source.addPoint(reconstruct[i]);
          source.setColor(index, color[i]);
        } else {
          int index = target.addPoint(reconstruct[i]);
          targetIndex.push_back(i);
        }
      } else {
        if ((f < frameCount) ||
            (!params.oneLayerMode_  && f == IntermediateLayerIndex)) {
          const auto &frame = video.getFrame(shift + ((!params.oneLayerMode_  && f == IntermediateLayerIndex) ? 1 : f));
          for (size_t c = 0; c < 3; ++c) {
            color[i][c] = frame.getValue(c, x, y);
          }
          int index = source.addPoint(reconstruct[i]);
          source.setColor(index, color[i]);
        } else {
          int index = target.addPoint(reconstruct[i]);
          targetIndex.push_back(i);
        }
      }
    }

    if (target.getPointCount() > 0) {
      source.transfertColorWeight(target);
      for (size_t i = 0; i < target.getPointCount(); ++i) {
        reconstruct.setColor(targetIndex[i], target.getColor(i));
      }
    }

    if ((losslessAtt || lossyMissedPointsPatch) && useMissedPointsSeparateVideo) {
      if (frame.getEnhancedDeltaDepth()) {
        for (size_t i = 0; i < numEddSavedPoints; ++i) {
          color[pointCount + i][0] = (uint8_t)missedPointsPatch.r[i];
          color[pointCount + i][1] = (uint8_t)missedPointsPatch.g[i];
          color[pointCount + i][2] = (uint8_t)missedPointsPatch.b[i];
        }
      }
      //miseed points
      for (size_t i = 0; i < numOfMPGeos; ++i) {
        color[numEddSavedPoints + pointCount + i][0] = (uint8_t)missedPointsPatch.r[numEddSavedPoints + i];
        color[numEddSavedPoints + pointCount + i][1] = (uint8_t)missedPointsPatch.g[numEddSavedPoints + i];
        color[numEddSavedPoints + pointCount + i][2] = (uint8_t)missedPointsPatch.b[numEddSavedPoints + i];
      }
    }
  } //noAtt
  return true;
}

void PCCCodec::generateMissedPointsGeometryfromVideo(PCCContext& context, PCCGroupOfFrames& reconstructs) {
  const size_t gofSize = context.getGofSize();
  auto& videoMPsGeometry = context.getVideoMPsGeometry();
  videoMPsGeometry.resize(gofSize);
  for (auto &framecontext : context.getFrames()) {
    const size_t shift = framecontext.getIndex();
    framecontext.setLosslessGeo(context.getLosslessGeo());
    framecontext.setLosslessGeo444(context.getLosslessGeo444());
    generateMPsGeometryfromImage(context, framecontext, reconstructs, shift);
    std::cout << "generate Missed Points (Geometry) : frame " << shift << ", # of Missed Points Geometry : " << framecontext.getMissedPointsPatch().size() << std::endl;
  }
  std::cout << "MissedPoints Geometry [done]" << std::endl;
}

void PCCCodec::generateMPsGeometryfromImage(PCCContext& context, PCCFrameContext& frame, PCCGroupOfFrames& reconstructs, size_t frameIndex) {
  auto& videoMPsGeometry = context.getVideoMPsGeometry();
  auto &image = videoMPsGeometry.getFrame(frameIndex);
  auto& missedPointsPatch = frame.getMissedPointsPatch();
  size_t width = image.getWidth();
  bool losslessGeo444 = frame.getLosslessGeo444();
  size_t numofMPs = missedPointsPatch.getMPnumber();
  if (losslessGeo444) {
    missedPointsPatch.resize(numofMPs);
  }
  else {
    missedPointsPatch.resize(numofMPs * 3);
  }

  for (size_t i = 0; i<numofMPs; i++) {
    if (frame.getLosslessGeo444()) {
      missedPointsPatch.x[i] = image.getValue(0, i%width, i / width);
      missedPointsPatch.y[i] = image.getValue(0, i%width, i / width);
      missedPointsPatch.z[i] = image.getValue(0, i%width, i / width);
    }
    else {
      missedPointsPatch.x[i] = image.getValue(0, i%width, i / width);
      missedPointsPatch.x[numofMPs + i] = image.getValue(0, (numofMPs + i) % width, (numofMPs + i) / width);
      missedPointsPatch.x[2 * numofMPs + i] = image.getValue(0, (2 * numofMPs + i) % width, (2 * numofMPs + i) / width);
    }
  }
}

void PCCCodec::generateMissedPointsTexturefromVideo(PCCContext& context, PCCGroupOfFrames& reconstructs) {
  const size_t gofSize = context.getGofSize();
  auto& videoMPsTexture = context.getVideoMPsTexture();
  videoMPsTexture.resize(gofSize);
  for (auto &framecontext : context.getFrames()) {
    const size_t shift = framecontext.getIndex(); //
    framecontext.setLosslessAtt(context.getLosslessAtt());
    generateMPsTexturefromImage(context, framecontext, reconstructs, shift);
    std::cout << "generate Missed Points (Texture) : frame " << shift << ", # of Missed Points Texture : " << framecontext.getMissedPointsPatch().size() << std::endl;
  }
  std::cout << "MissedPoints Texture [done]" << std::endl;
}

void PCCCodec::generateMPsTexturefromImage(PCCContext& context, PCCFrameContext& frame, PCCGroupOfFrames& reconstructs, size_t frameIndex) {
  auto& videoMPsTexture = context.getVideoMPsTexture();
  auto &image = videoMPsTexture.getFrame(frameIndex);
  auto& missedPointsPatch = frame.getMissedPointsPatch();
  size_t width = image.getWidth();
  size_t numofMPs = missedPointsPatch.getMPnumbercolor();
  for (size_t i = 0; i<numofMPs; i++) {
    assert(i / width<image.getHeight());
    missedPointsPatch.r[i] = image.getValue(0, i%width, i / width);
    missedPointsPatch.g[i] = image.getValue(1, i%width, i / width);
    missedPointsPatch.b[i] = image.getValue(2, i%width, i / width);
  }
}
