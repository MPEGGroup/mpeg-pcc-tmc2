
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
#include "PCCPointSet.h"
#include "tbb/tbb.h"
#include "PCCKdTree.h"
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCGroupOfFrames.h"
#include "PCCPatch.h"

#include "PCCCodec.h"

using namespace pcc;

PCCCodec::PCCCodec() {}
PCCCodec::~PCCCodec() {}

void PCCCodec::generatePointCloud( PCCGroupOfFrames& reconstructs, PCCContext& context,
                                   const GeneratePointCloudParameters params ) {
  auto& frames = context.getFrames();
  auto &videoGeometry = context.getVideoGeometry();
  auto &videoGeometryD1 = context.getVideoGeometryD1();
    for( size_t i = 0; i < frames.size(); i++ ) {
    std::vector<uint32_t> partition;
    generatePointCloud( reconstructs[i], frames[i], videoGeometry, videoGeometryD1, params, partition );
    if (!params.losslessGeo_ ) {
      smoothPointCloud( reconstructs[i], partition, params );
    }
  }
}

bool PCCCodec::colorPointCloud( PCCGroupOfFrames& reconstructs, PCCContext& context,
                               const bool noAttributes, const ColorTransform colorTransform,
                               const GeneratePointCloudParameters params) {
  auto &video = context.getVideoTexture();
  auto& frames = context.getFrames();
  for( size_t i = 0; i < frames.size(); i++ ) {
    colorPointCloud( reconstructs[i], frames[i],  video, noAttributes );
    if (!params.losslessGeo_ && params.flagColorSmoothing_) {
      smoothPointCloudColor(reconstructs[i], params);
    }
    if ( colorTransform == COLOR_TRANSFORM_RGB_TO_YCBCR ) {
      reconstructs[i].convertYUVToRGB();
    }
  }
  return true;
}

void PCCCodec::generatePointCloud( PCCPointSet3& reconstruct, PCCFrameContext &frame,
                                   const PCCVideoGeometry &video, const PCCVideoGeometry &videoD1,
                                   const GeneratePointCloudParameters params,
                                   std::vector<uint32_t> &partition ) {
  auto& patches      = frame.getPatches();
  auto& pointToPixel = frame.getPointToPixel();
  auto& blockToPatch = frame.getBlockToPatch();
  auto& occupancyMap = frame.getOccupancyMap();
  partition.resize(0);
  pointToPixel.resize(0);
  reconstruct.clear();

  bool useMissedPointsSeparateVideo=frame.getUseMissedPointsSeparateVideo();
  PCCPointSet3 eddSavedPoints;
  size_t numEddSavedPoints=0;
  
//EDD code
  std::vector<size_t> vecEmptyBlocks;
  if (params.enhancedDeltaDepthCode_) {
    for (size_t i = 0; i < blockToPatch.size(); i++)
      if (blockToPatch[i] == 0)
        vecEmptyBlocks.push_back(i);
  }
  size_t nUsedEmptyBlockCount = 0;
  size_t nPixelInCurrentBlockCount = 0;
  size_t nFrameToStore = 0;
  const size_t nPixelInBlockNum = params.occupancyResolution_ * params.occupancyResolution_;

  size_t shift;
  const size_t layerCount = 2;
  if (!params.absoluteD1_) {
    shift = frame.getIndex();
    if (video.getFrameCount() < (shift + 1)) {
      return;
    }
  } else {
    shift = frame.getIndex() * 2;
    if (video.getFrameCount() < (shift + layerCount)) {
      return;
    }
  }
  const auto &frame0 = video.getFrame(shift);
  const size_t imageWidth = video.getWidth();
  const size_t imageHeight = video.getHeight();
  std::vector<uint32_t> PBflag;
  PBflag.resize(imageWidth * imageHeight, 0);

  const size_t blockToPatchWidth = frame.getWidth() / params.occupancyResolution_;
  reconstruct.addColors();
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
    for (size_t v0 = 0; v0 < patch.getSizeV0(); ++v0) {
      for (size_t u0 = 0; u0 < patch.getSizeU0(); ++u0) {
        if (blockToPatch[(v0 + patch.getV0()) * blockToPatchWidth + u0 + patch.getU0()] ==
            patchIndexPlusOne) {
          for (size_t v1 = 0; v1 < patch.getOccupancyResolution(); ++v1) {
            const size_t v = v0 * patch.getOccupancyResolution() + v1;
            for (size_t u1 = 0; u1 < patch.getOccupancyResolution(); ++u1) {
              const size_t u = u0 * patch.getOccupancyResolution() + u1;
              const size_t x = patch.getU0() * patch.getOccupancyResolution() + u;
              const size_t y = patch.getV0() * patch.getOccupancyResolution() + v;
              const bool occupancy = occupancyMap[y * imageWidth + x] != 0;
              if (!occupancy) {
                continue;
              }

              PCCVector3D point0;
              //point0[patch.getNormalAxis()] = double(frame0.getValue(0, x, y) + patch.getD1());
              //point0[patch.getTangentAxis()] = double(u) + patch.getU1();
              //point0[patch.getBitangentAxis()] = double(v) + patch.getV1();
              const double lodScale = params.ignoreLod_ ? 1.0 : double(1u << patch.getLod());
              //yo- const double lodScale = 1.0;
              point0[patch.getNormalAxis()] = double(frame0.getValue(0, x, y) + patch.getD1()) * lodScale;
              point0[patch.getTangentAxis()] = (double(u) + patch.getU1()) * lodScale;
              point0[patch.getBitangentAxis()] = (double(v) + patch.getV1()) * lodScale;
              //EDD code
              if (params.enhancedDeltaDepthCode_) {
                //D0
                const size_t pointIndex0 = reconstruct.addPoint(point0);
                reconstruct.setColor(pointIndex0, color);
                if (PCC_SAVE_POINT_TYPE == 1) {
                  reconstruct.setType(pointIndex0, PointType::D0);
                }
                partition.push_back(uint32_t(patchIndex));
                pointToPixel.push_back(PCCVector3<size_t>(x, y, 0));

                //EDD code
                uint16_t eddCode = 0;
                if (!params.absoluteD1_) {
                  const auto &frame1 = videoD1.getFrame(shift);
                  eddCode = frame1.getValue(0, x, y);
                }
                else {
                  const auto &frame0 = video.getFrame(shift);
                  const auto &frame1 = video.getFrame(shift + 1);
                  //eddCode = frame1.getValue(0, x, y) - frame0.getValue(0, x, y);
                  eddCode = (frame1.getValue(0, x, y) > frame0.getValue(0, x, y)) ? (frame1.getValue(0, x, y) - frame0.getValue(0, x, y)) : 0;
                }
                PCCVector3D  point1(point0);
                if (eddCode == 0)
                {
                  const size_t pointIndex1 = reconstruct.addPoint(point1);
                  reconstruct.setColor(pointIndex1, color);
                  if (PCC_SAVE_POINT_TYPE == 1) {
                    reconstruct.setType(pointIndex1, PointType::D1);
                  }
                  partition.push_back(uint32_t(patchIndex));
                  pointToPixel.push_back(PCCVector3<size_t>(x, y, 1));
                }
                else
                { //eddCode != 0
                  uint16_t addedPointCount = 0;
                  size_t   pointIndex1 = 0;
                  //for (uint16_t i = 0; i < surfaceThickness; i++) 
                  for (uint16_t i = 0; i < 10; i++)
                  {
                    if (eddCode & (1 << i))
                    {
                      uint8_t deltaDCur = (i + 1);
                      if (patch.getProjectionMode() == 0)
                        point1[patch.getNormalAxis()] = (double)(point0[patch.getNormalAxis()] + deltaDCur);
                      else
                        point1[patch.getNormalAxis()] = (double)(point0[patch.getNormalAxis()] - deltaDCur);
                      if(!useMissedPointsSeparateVideo)
                      {
                      pointIndex1 = reconstruct.addPoint(point1);
                      reconstruct.setColor(pointIndex1, color);
                      if (PCC_SAVE_POINT_TYPE == 1) {
                        reconstruct.setType(pointIndex1, PointType::InBetween);
                      }
                      partition.push_back(uint32_t(patchIndex));
                      }
                      
                      if (addedPointCount == 0)
                      {
                        if(useMissedPointsSeparateVideo)
                        {
                        pointIndex1 = reconstruct.addPoint(point1);
                        reconstruct.setColor(pointIndex1, color);
                        if (PCC_SAVE_POINT_TYPE == 1) {
                          reconstruct.setType(pointIndex1, PointType::InBetween);
                        }
                        partition.push_back(uint32_t(patchIndex));
                        }

                        pointToPixel.push_back(PCCVector3<size_t>(x, y, 1));
                        
                      }
                      else
                      {
                        if(useMissedPointsSeparateVideo)
                        {
                        numEddSavedPoints++;
                        eddSavedPoints.addPoint(point1);
                        }
                        else
                        {
                        size_t uBlock = vecEmptyBlocks[nUsedEmptyBlockCount] % blockToPatchWidth;
                        size_t vBlock = vecEmptyBlocks[nUsedEmptyBlockCount] / blockToPatchWidth;
                        size_t uu = uBlock * params.occupancyResolution_ + nPixelInCurrentBlockCount % params.occupancyResolution_;
                        size_t vv = vBlock * params.occupancyResolution_ + nPixelInCurrentBlockCount / params.occupancyResolution_;
                        pointToPixel.push_back(PCCVector3<size_t>(uu, vv, nFrameToStore));
                        occupancyMap[vv * imageWidth + uu] = 1; //occupied

                        ++nPixelInCurrentBlockCount;
                        if (nPixelInCurrentBlockCount >= nPixelInBlockNum)
                        {
                          nUsedEmptyBlockCount++;
                          nPixelInCurrentBlockCount = 0;
                          if (nUsedEmptyBlockCount >= vecEmptyBlocks.size())
                          {
                            if (nFrameToStore == 0)
                            {
                              nFrameToStore = 1;
                              nUsedEmptyBlockCount = 0;
                            }
                            else
                            {
                              std::cout << "Enchanded delta depth: for packing the color of in-between points, all emptry blocks are used up. Need to enlarge image size. Not implemented... Exit for the moment...\n";
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
              }
              else {//not params.enhancedDeltaDepthCode_
                for (size_t f = 0; f < layerCount; ++f) {
                  PCCVector3D point1(point0);
                  if (f > 0) {
                    if( !params.absoluteD1_ ) {
                      const auto &frame1 = videoD1.getFrame(shift);
                      //point1[patch.getNormalAxis()] += frame1.getValue(0, x, y);
                      if (patch.getProjectionMode() == 0)
                        //point1[patch.getNormalAxis()] += (std::min)(frame1.getValue(0, x, y), uint16_t(9));
                        point1[patch.getNormalAxis()] += frame1.getValue(0, x, y) * lodScale;
                      else
                        //point1[patch.getNormalAxis()] -= (std::min)(frame1.getValue(0, x, y), uint16_t(9));
                        point1[patch.getNormalAxis()] -= frame1.getValue(0, x, y) * lodScale;
                    } else {
                      const auto &frame1 = video.getFrame(f + shift);
                      //point1[patch.getNormalAxis()] = double( frame1.getValue(0, x, y) + patch.getD1());
                      point1[patch.getNormalAxis()] = double( frame1.getValue(0, x, y) + patch.getD1()) * lodScale;
                    }
                  }
                  const size_t pointindex_1 = reconstruct.addPoint(point1);

                  // Identify boundary points
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

                  reconstruct.setColor(pointindex_1, color);
                  if( PCC_SAVE_POINT_TYPE == 1 ) {
                    reconstruct.setType( pointindex_1, f == 0 ? PointType::D0 : PointType::D1 );
                  }
                  partition.push_back(uint32_t(patchIndex));
                  pointToPixel.push_back(PCCVector3<size_t>(x, y, f));
                }
              } //else (params.enhancedDeltaDepthCode_)
            }
          }
        }
      }
    }
  }

  if(useMissedPointsSeparateVideo)
  {
  if (params.losslessGeo_)
  {
    PCCColor3B missedPointsColor(uint8_t(0));
    missedPointsColor[0] = 0;
    missedPointsColor[1] = 255;
    missedPointsColor[2] = 255;

    if(frame.getEnhancedDeltaDepth())
    {
      //add Edd to reconstruct
      frame.getMissedPointsPatch().numEddSavedPoints = numEddSavedPoints;
      assert(eddSavedPoints.getPointCount()==numEddSavedPoints);
      size_t pointIndex1;
      for(size_t eddPoint=0;eddPoint<numEddSavedPoints;eddPoint++)
      {
        PCCVector3D  point1;
        point1 = eddSavedPoints[eddPoint];
        pointIndex1 = reconstruct.addPoint(point1);
        reconstruct.setColor(pointIndex1, missedPointsColor);
      }
    } //enhanced

    //Add point GPS from missedPointsPatch without inserting to pointToPixel
    auto& missedPointsPatch = frame.getMissedPointsPatch();
    if(params.losslessGeo444_)
    {
      for(int i=0;i<missedPointsPatch.size();i++)
      {
        PCCVector3D point0;
        point0[0] = missedPointsPatch.x[i];
        point0[1] = missedPointsPatch.y[i];
        point0[2] = missedPointsPatch.z[i];
        const size_t pointIndex = reconstruct.addPoint(point0);
        reconstruct.setColor(pointIndex, missedPointsColor);
      }
    }//mp_444
    else
    {
      assert(missedPointsPatch.size()%3==0);
      size_t sizeofMPs=(missedPointsPatch.size())/3;
      for(int i=0;i<sizeofMPs;i++)
      {
        PCCVector3D point0;
        point0[0] = missedPointsPatch.x[i];
        point0[1] = missedPointsPatch.x[i+sizeofMPs];
        point0[2] = missedPointsPatch.x[i+2*sizeofMPs];
        const size_t pointIndex = reconstruct.addPoint(point0);
        reconstruct.setColor(pointIndex, missedPointsColor);
      }
    } //mp_420
    
  } //lossless
  
  if (frame.getLosslessAtt()) 
  {
    //secure the size of rgb in missedpointpatch
    auto& missedPointsPatch = frame.getMissedPointsPatch();
    size_t numofMPcolor = (params.losslessGeo444_)? missedPointsPatch.size() : missedPointsPatch.size()/3;
    size_t numofEddSaved = frame.getEnhancedDeltaDepth()?missedPointsPatch.numEddSavedPoints:0;
    missedPointsPatch.resizecolor(numofMPcolor+numofEddSaved);
    missedPointsPatch.setMPnumbercolor(numofMPcolor+numofEddSaved);
    assert(numEddSavedPoints == numofEddSaved);
  }
  }//useMissedPointsSeparateVideo
  else
  {
  // Add points from missedPointsPatch
  auto& missedPointsPatch = frame.getMissedPointsPatch();
    
  PCCColor3B missedPointsColor(uint8_t(0));
  missedPointsColor[0] = 0;
  missedPointsColor[1] = 255;
  missedPointsColor[2] = 255;
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
            PCCVector3D point0;
            point0[0] = double(frame0.getValue(0, x, y));
            point0[1] = double(frame0.getValue(1, x, y));
            point0[2] = double(frame0.getValue(2, x, y));
            const size_t pointIndex = reconstruct.addPoint(point0);
            reconstruct.setColor(pointIndex, missedPointsColor);
            for (size_t f = 0; f < layerCount; ++f)
            {
              pointToPixel.push_back(PCCVector3<size_t>(x, y, f));
            }
          }
        }
      }
    }
  } else if (params.losslessGeo_) {
    size_t numMissedPts{ 0 };
    for (size_t v0 = 0; v0 < missedPointsPatch.sizeV0; ++v0) {
      for (size_t u0 = 0; u0 < missedPointsPatch.sizeU0; ++u0) {
        for (size_t v1 = 0; v1 < missedPointsPatch.occupancyResolution; ++v1) {
          const size_t v = v0 * missedPointsPatch.occupancyResolution + v1;
          for (size_t u1 = 0; u1 < missedPointsPatch.occupancyResolution; ++u1) {
            const size_t u = u0 * missedPointsPatch.occupancyResolution + u1;
            const size_t x = missedPointsPatch.u0 * missedPointsPatch.occupancyResolution + u;
            const size_t y = missedPointsPatch.v0 * missedPointsPatch.occupancyResolution + v;
            if (occupancyMap[y * imageWidth + x] != 0)
              numMissedPts++;
          }
        }
      }
    }
    numMissedPts /= 3;
    std::vector<PCCVector3D> missedPoints;
    missedPoints.resize(numMissedPts);
    size_t numMissedPointsAdded{ 0 };
    size_t missedPointsPatchWidth = missedPointsPatch.sizeU0 * missedPointsPatch.occupancyResolution;
    size_t missedPointsPatchHeight = missedPointsPatch.sizeV0 * missedPointsPatch.occupancyResolution;
    const size_t v0 = missedPointsPatch.v0 * missedPointsPatch.occupancyResolution;
    const size_t u0 = missedPointsPatch.u0 * missedPointsPatch.occupancyResolution;
    for (size_t v = 0; v < missedPointsPatchHeight; ++v) {
      for (size_t u = 0; u < missedPointsPatchWidth; ++u) {
        const size_t x = (u0 + u);
        const size_t y = (v0 + v);
        const bool occupancy = occupancyMap[y * imageWidth + x] != 0;
        if (!occupancy) {
          continue;
        }
        if (numMissedPointsAdded < numMissedPts) {
          missedPoints[numMissedPointsAdded][0] = double(frame0.getValue(0, x, y));
        }
        else if (numMissedPts <= numMissedPointsAdded && numMissedPointsAdded < 2 * numMissedPts) {
          missedPoints[numMissedPointsAdded - numMissedPts][1] = double(frame0.getValue(0, x, y));
        }
        else if (2 * numMissedPts <= numMissedPointsAdded && numMissedPointsAdded < 3 * numMissedPts) {
          missedPoints[numMissedPointsAdded - 2 * numMissedPts][2] = double(frame0.getValue(0, x, y));
          const size_t pointIndex = reconstruct.addPoint(missedPoints[numMissedPointsAdded - 2 * numMissedPts]);
          reconstruct.setColor(pointIndex, missedPointsColor);
          for (size_t f = 0; f < layerCount; ++f) {
            pointToPixel.push_back(PCCVector3<size_t>(x, y, f));
          }
        }
        numMissedPointsAdded++;
      }//u
    } //v

  }
    
  }//else :useMissedPointsSeparateVideo

   
}

void PCCCodec::smoothPointCloud( PCCPointSet3& reconstruct,
                                 const std::vector<uint32_t> &partition,
                                 const GeneratePointCloudParameters params ) {
  const size_t pointCount = reconstruct.getPointCount();
  PCCStaticKdTree3 kdtree;
  kdtree.build(reconstruct);
  PCCPointSet3 temp;
  temp.resize(pointCount);
  tbb::task_arena limited( (int)params.nbThread_ );
  limited.execute([&]{ 
    tbb::parallel_for(size_t( 0 ), pointCount, [&](const size_t i) {
      //  for (size_t i = 0; i < pointCount; ++i) {
      const size_t maxNeighborCount = 512;
      PCCPointDistInfo nNeighbor[maxNeighborCount];
      PCCNNResult result = {nNeighbor, 0};
      const double maxDist = ceil( sqrt( params.radius2Smoothing_ ) );
      PCCNNQuery3 query = {PCCVector3D(0.0), maxDist, (std::min)(maxNeighborCount, params.neighborCountSmoothing_)};

      const size_t clusterindex_ = partition[i];
      query.point = reconstruct[i];
      kdtree.findNearestNeighbors(query, result);
      assert(result.resultCount);
      PCCVector3D centroid(0.0);
      bool otherClusterPointCount = false;
      size_t neighborCount = 0;
      for (size_t r = 0; r < result.resultCount; ++r) {
        const double dist2 = result.neighbors[r].dist2;
        if (dist2 > params.radius2Smoothing_) {
          break;
        }
        ++neighborCount;
        const size_t pointindex_ = result.neighbors[r].index;
        centroid += reconstruct[pointindex_];
        otherClusterPointCount |=
            (dist2 <= params.radius2BoundaryDetection_) && (partition[pointindex_] != clusterindex_);
      }

      if (otherClusterPointCount) {
        if (reconstruct.getBoundaryPointType(i) == 1) {
          reconstruct.setBoundaryPointType(i, static_cast<uint16_t>(2));
        }

        const auto sclaedPoint = double(neighborCount) * query.point;
        const double distToCentroid2 =
            int64_t((centroid - sclaedPoint).getNorm2() + (neighborCount / 2.0)) /
            double(neighborCount);
        for (size_t k = 0; k < 3; ++k) {
          centroid[k] = double(int64_t(centroid[k] + (neighborCount / 2)) / neighborCount);
        }
        if (distToCentroid2 >= params.thresholdSmoothing_) {
          temp[i] = centroid;
          reconstruct.setColor(i, PCCColor3B(255, 0, 0));
          if( PCC_SAVE_POINT_TYPE == 1 ) {
            reconstruct.setType( i, PointType::Smooth );
          }
        } else {
          temp[i] = query.point;
        }
      } else {
        temp[i] = query.point;
      }
    });
  }); 
  limited.execute([&]{ 
    tbb::parallel_for(size_t( 0 ), pointCount, [&](const size_t i) {
      // for (size_t i = 0; i < pointCount; ++i) {
      reconstruct[i] = temp[i];
    });
  });
}

inline double Entropy(std::vector<uint8_t> &Data, int N) {
  std::vector<size_t> Count;
  Count.resize(256, 0);

  for (size_t i = 0; i < N; ++i) {
    ++Count[size_t(Data[i])];
  }

  double s = 0;
  for (size_t i = 0; i < 256; ++i) {
    if (Count[i]) {
      double p = double(Count[i]) / double(N);
      s += -p * std::log2(p);
    }
  }
  return s;
}

void PCCCodec::smoothPointCloudColor(PCCPointSet3 &reconstruct,
                                     const GeneratePointCloudParameters params) {
  const size_t pointCount = reconstruct.getPointCount();
  PCCStaticKdTree3 kdtree;
  kdtree.build(reconstruct);
  PCCPointSet3 temp;
  temp.resize(pointCount);
  tbb::task_arena limited((int)params.nbThread_);
  limited.execute([&] {
    tbb::parallel_for(size_t(0), pointCount, [&](const size_t i) {
      //  for (size_t i = 0; i < pointCount; ++i) {
      const size_t maxNeighborCount = 512;
      PCCPointDistInfo nNeighbor[maxNeighborCount];
      PCCNNResult result = {nNeighbor, 0};
      const double maxDist = ceil(sqrt(params.radius2ColorSmoothing_));
      PCCNNQuery3 query = {PCCVector3D(0.0), maxDist,
                           (std::min)(maxNeighborCount, params.neighborCountColorSmoothing_)};

      if (reconstruct.getBoundaryPointType(i) == 2) {
        query.point = reconstruct[i];
        kdtree.findNearestNeighbors(query, result);
        assert(result.resultCount);
        PCCVector3D centroid(0.0);

        size_t neighborCount = 0;
        std::vector<uint8_t> Lum;

        for (size_t r = 0; r < result.resultCount; ++r) {
          const double dist2 = result.neighbors[r].dist2;
          if (dist2 > params.radius2ColorSmoothing_) {
            break;
          }
          ++neighborCount;
          const size_t index = result.neighbors[r].index;
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
          double H = Entropy(Lum, int(neighborCount));
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
bool PCCCodec::colorPointCloud( PCCPointSet3& reconstruct, PCCFrameContext& frame,
                                const PCCVideoTexture &video, const bool noAttributes ) {
  if( noAttributes ) {
    for (auto& color : reconstruct.getColors() ) {
      for (size_t c = 0; c < 3; ++c) {
        color[c] = static_cast<uint8_t>(127);
      }
    }
  }
  else
  {
    auto& pointToPixel = frame.getPointToPixel();
    auto& color        = reconstruct.getColors();

    bool useMissedPointsSeparateVideo = frame.getUseMissedPointsSeparateVideo();
    bool losslessAtt = frame.getLosslessAtt();
    auto& missedPointsPatch=frame.getMissedPointsPatch();
    size_t numOfMPColor = missedPointsPatch.getMPnumbercolor();
    size_t numOfMPGeos =missedPointsPatch.getMPnumber();
    size_t numEddSavedPoints = missedPointsPatch.numEddSavedPoints;
    size_t pointCount = reconstruct.getPointCount();
    if(useMissedPointsSeparateVideo && losslessAtt)
    {
      pointCount = reconstruct.getPointCount() - numOfMPGeos - numEddSavedPoints;
    assert( numOfMPColor == (numEddSavedPoints+numOfMPGeos));
    missedPointsPatch.resizecolor(numOfMPColor);
    }
//    const size_t pointCount = reconstruct.getPointCount();

    
    if (!pointCount || !reconstruct.hasColors()) {
      return false;
    }
    const size_t shift = frame.getIndex() * 2;
    for (size_t i = 0; i < pointCount; ++i) {
      const PCCVector3<size_t> location = pointToPixel[i];
      const size_t x = location[0];
      const size_t y = location[1];
      const size_t f = location[2];
      const auto &frame = video.getFrame( shift + f );
      for (size_t c = 0; c < 3; ++c) {
        color[i][c] = frame.getValue(c, x, y);
      }
    }

    if(losslessAtt && useMissedPointsSeparateVideo)
    {
      if(frame.getEnhancedDeltaDepth())
      {
        for (size_t i = 0; i < numEddSavedPoints; ++i) {
          color[pointCount+i][0]=missedPointsPatch.r[i];
          color[pointCount+i][1]=missedPointsPatch.g[i];
          color[pointCount+i][2]=missedPointsPatch.b[i];
        }

      }
      
      //miseed points
      for (size_t i = 0; i < numOfMPGeos; ++i) {
        color[numEddSavedPoints+pointCount+i][0]=missedPointsPatch.r[numEddSavedPoints+i] ;
        color[numEddSavedPoints+pointCount+i][1]=missedPointsPatch.g[numEddSavedPoints+i] ;
        color[numEddSavedPoints+pointCount+i][2]=missedPointsPatch.b[numEddSavedPoints+i] ;
        
      }

    }
  } //noAtt
  

  return true;
}


