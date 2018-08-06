
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
#ifndef PCCVideoEncoder_h
#define PCCVideoEncoder_h

#include "PCCCommon.h"
#include "PCCBitstream.h"
#include "PCCSystem.h"
#include "PCCVideo.h"
#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCPatch.h"

namespace pcc {

class PCCBitstream;
class PCCContext;
class PCCFrameContext;

class PCCVideoEncoder {
 public:
  PCCVideoEncoder();
  ~PCCVideoEncoder();
  template <typename T>
  bool compress(PCCVideo<T, 3> &video,                                            
                const std::string &fileName,
                const size_t qp,
                PCCBitstream &bitstream,
                const std::string &encoderConfig,
                const std::string &encoderPath,
				PCCContext& contexts,
                const std::string &colorSpaceConversionConfig = "",
                const std::string &inverseColorSpaceConversionConfig = "",
                const std::string &colorSpaceConversionPath = "",
                const bool use444CodecIo = false,
				const bool patchColorSubsampling = false,
                const bool flagColorSmoothing = false,
                const size_t nbyte = 1,
                const bool keepIntermediateFiles = false ) {


    auto& frames = video.getFrames();
    if (frames.empty()) {
      return false;
    }
    const size_t width = frames[0].getWidth();
    const size_t height = frames[0].getHeight();
    const size_t depth = nbyte == 1 ? 8 : 10;
    const size_t frameCount = video.getFrameCount();
    if (frames[0].getChannelCount() != 3) {
      return false;
    }


    const std::string format = use444CodecIo ? "444" : "420";
    const std::string binFileName = fileName + ".bin";
    const std::string srcYuvFileName = addVideoFormat(fileName + (use444CodecIo ? ".rgb" : ".yuv"), width, height, !use444CodecIo);
    const std::string srcRgbFileName = addVideoFormat(fileName + ".rgb", width, height);
    const std::string recYuvFileName = addVideoFormat(fileName + "_rec" + (use444CodecIo ? ".rgb" : ".yuv"), width, height, !use444CodecIo);
    const std::string recRgbFileName = addVideoFormat(fileName + "_rec" + ".rgb", width, height, !use444CodecIo);

    const bool yuvVideo = colorSpaceConversionConfig.empty() || colorSpaceConversionPath.empty() || use444CodecIo;
    // todo: should use444CodecIo allow conversion to happen?
    if( yuvVideo ) {
      if (use444CodecIo) {
        if (!video.write( srcYuvFileName, nbyte)) {
          return false;
        }
      } else {
        if (!video.write420( srcYuvFileName, nbyte)) {
          return false;
        }
      }
    } else {
      if (patchColorSubsampling)
      {
        PCCVideo<T, 3> video420;
        //perform color-subsampling based on patch information
        video420.resize(video.getFrameCount());
        for (size_t frNum = 0; frNum < video.getFrameCount(); frNum++) {
          //context variable, contains the patch information
          auto& context = contexts[frNum / 2];
          //full resolution image (already filled by previous dilation
          auto &refImage = video.getFrame(frNum);
          //image that will contain the per-patch chroma sub-sampled image
          auto &destImage = video420.getFrame(frNum);
          destImage.resize(width, height);

          //iterate the patch information and perform chroma down-sampling on each patch individually
          std::vector<PCCPatch> patches = context.getPatches();
          std::vector<size_t> blockToPatch = context.getBlockToPatch();
          for (int patchIdx = 0; patchIdx <=  patches.size(); patchIdx++)
          {
            size_t occupancyResolution;
            size_t patch_left;
            size_t patch_top;
            size_t patch_width;
            size_t patch_height;
            if (patchIdx == 0)
            {
              //background, does not have a corresponding patch
              auto& patch = patches[0];
              occupancyResolution = patch.getOccupancyResolution();
              patch_left = 0;
              patch_top = 0;
              patch_width = width;
              patch_height = height;
            }
            else
            {
              auto& patch = patches[patchIdx-1];
              occupancyResolution = patch.getOccupancyResolution();
              patch_left = patch.getU0() * occupancyResolution;
              patch_top = patch.getV0() * occupancyResolution;
              patch_width = patch.getSizeU0() * occupancyResolution;
              patch_height = patch.getSizeV0() * occupancyResolution;
            }
            //initializing the image container with zeros
            PCCImage<T, 3> tmpImage;
            tmpImage.resize(patch_width, patch_height);
            //cut out the patch image
            refImage.copy_block(patch_top, patch_left, patch_width, patch_height, tmpImage);

            //fill in the blocks by extending the edges
            for (size_t i = 0; i < patch_height/occupancyResolution; i++)
            {
              for (size_t j = 0; j < patch_width/occupancyResolution; j++)
              {
                if (context.getBlockToPatch()[(i + patch_top / occupancyResolution)*(width / occupancyResolution) + j + patch_left / occupancyResolution] == patchIdx)
                {
                  //do nothing
                  continue;
                }
                else
                {
                  //search for the block that contains texture information and extend the block edge 
                  int direction;
                  int searchIndex;
                  std::vector<int> neighborIdx(4,-1);
                  std::vector<int> neighborDistance(4,std::numeric_limits<int>::max());
                  //looking for the neighboring block to the left of the current block
                  searchIndex = j;
                  while (searchIndex >= 0)
                  {
                    if (context.getBlockToPatch()[(i + patch_top / occupancyResolution)*(width / occupancyResolution) + searchIndex + patch_left / occupancyResolution] == patchIdx)
                    {
                      neighborIdx[0] = searchIndex;
                      neighborDistance[0] = j - searchIndex;
                      searchIndex = 0;
                    }
                    searchIndex--;
                  }
                  //looking for the neighboring block to the right of the current block
                  searchIndex = j;
                  while (searchIndex < patch_width/occupancyResolution)
                  {
                    if (context.getBlockToPatch()[(i + patch_top / occupancyResolution)*(width / occupancyResolution) + searchIndex + patch_left / occupancyResolution] == patchIdx)
                    {
                      neighborIdx[1] = searchIndex;
                      neighborDistance[1] = searchIndex - j;
                      searchIndex = patch_width / occupancyResolution;
                    }
                    searchIndex++;
                  }
                  //looking for the neighboring block above the current block
                  searchIndex = i;
                  while (searchIndex >= 0)
                  {
                    if (context.getBlockToPatch()[(searchIndex + patch_top / occupancyResolution)*(width / occupancyResolution) + j + patch_left / occupancyResolution] == patchIdx)
                    {
                      neighborIdx[2] = searchIndex;
                      neighborDistance[2] = i - searchIndex;
                      searchIndex = 0;
                    }
                    searchIndex--;
                  }
                  //looking for the neighboring block below the current block
                  searchIndex = i;
                  while (searchIndex < patch_height / occupancyResolution)
                  {
                    if (context.getBlockToPatch()[(searchIndex + patch_top / occupancyResolution)*(width / occupancyResolution) + j + patch_left / occupancyResolution] == patchIdx)
                    {
                      neighborIdx[3] = searchIndex;
                      neighborDistance[3] = searchIndex - i ;
                      searchIndex = patch_height / occupancyResolution;
                    }
                    searchIndex++;
                  }
                  //check if the candidate was found
                  assert(*std::max(neighborIdx.begin(),neighborIdx.end()) > 0);
                  //now fill in the block with the edge value coming from the nearest neighbor
                  direction = std::min_element(neighborDistance.begin(), neighborDistance.end()) - neighborDistance.begin();
                  if (direction == 0)
                  {
                    //copying from left neighboring block
                    for (size_t iBlk = 0; iBlk < occupancyResolution; iBlk++)
                    {
                      for (size_t jBlk = 0; jBlk < occupancyResolution; jBlk++)
                      {
                        tmpImage.setValue(0, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(0, neighborIdx[0] * occupancyResolution + occupancyResolution - 1, i * occupancyResolution + iBlk));
                        tmpImage.setValue(1, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(1, neighborIdx[0] * occupancyResolution + occupancyResolution - 1, i * occupancyResolution + iBlk));
                        tmpImage.setValue(2, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(2, neighborIdx[0] * occupancyResolution + occupancyResolution - 1, i * occupancyResolution + iBlk));
                      }
                    }
                  }
                  else if (direction == 1) {
                    //copying block from right neighboring position
                    for (size_t iBlk = 0; iBlk < occupancyResolution; iBlk++)
                    {
                      for (size_t jBlk = 0; jBlk < occupancyResolution; jBlk++)
                      {
                        tmpImage.setValue(0, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(0, neighborIdx[1] * occupancyResolution, i * occupancyResolution + iBlk));
                        tmpImage.setValue(1, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(1, neighborIdx[1] * occupancyResolution, i * occupancyResolution + iBlk));
                        tmpImage.setValue(2, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(2, neighborIdx[1] * occupancyResolution, i * occupancyResolution + iBlk));
                      }
                    }
                  }
                  else if (direction == 2) {
                    //copying block from above
                    for (size_t iBlk = 0; iBlk < occupancyResolution; iBlk++)
                    {
                      for (size_t jBlk = 0; jBlk < occupancyResolution; jBlk++)
                      {
                        tmpImage.setValue(0, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(0, j * occupancyResolution + jBlk, neighborIdx[2] * occupancyResolution + occupancyResolution - 1));
                        tmpImage.setValue(1, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(1, j * occupancyResolution + jBlk, neighborIdx[2] * occupancyResolution + occupancyResolution - 1));
                        tmpImage.setValue(2, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(2, j * occupancyResolution + jBlk, neighborIdx[2] * occupancyResolution + occupancyResolution - 1));
                      }
                    }
                  }
                  else if (direction == 3) {
                    //copying block from below
                    for (size_t iBlk = 0; iBlk < occupancyResolution; iBlk++)
                    {
                      for (size_t jBlk = 0; jBlk < occupancyResolution; jBlk++)
                      {
                        tmpImage.setValue(0, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(0, j * occupancyResolution + jBlk, neighborIdx[3] * occupancyResolution));
                        tmpImage.setValue(1, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(1, j * occupancyResolution + jBlk, neighborIdx[3] * occupancyResolution));
                        tmpImage.setValue(2, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(2, j * occupancyResolution + jBlk, neighborIdx[3] * occupancyResolution));
                      }
                    }
                  }
                  else
                  {
                    printf("This condition should never occur, report an error");
                    return false;
                  }
                }

              }
            }

            //perform downsampling
            const std::string rgbFileNameTmp = addVideoFormat(fileName + "_tmp.rgb", patch_width, patch_height);
            const std::string yuvFileNameTmp = addVideoFormat(fileName + "_tmp.yuv", patch_width, patch_height, true);
            if (!tmpImage.write(rgbFileNameTmp,nbyte)) {
              return false;
            }
            std::stringstream cmd;
            cmd << colorSpaceConversionPath << " -f " << colorSpaceConversionConfig << " -p SourceFile=\""
														  
              << rgbFileNameTmp << "\" -p OutputFile=\"" << yuvFileNameTmp << "\" -p SourceWidth=" << patch_width
										 
              << " -p SourceHeight=" << patch_height << " -p NumberOfFrames=" << video.getFrameCount();
												 
            std::cout << cmd.str() << '\n';
            if (pcc::system(cmd.str().c_str())) {
              std::cout << "Error: can't run system command!" << std::endl;
              return false;
            }
            tmpImage.read420(yuvFileNameTmp, patch_width, patch_height, nbyte);
            //removing intermediate files
            remove(rgbFileNameTmp.c_str());
            remove(yuvFileNameTmp.c_str());
            //substitute the pixels in the output image for compression
            for (size_t i = 0; i < patch_height; i++)
            {
              for (size_t j = 0; j < patch_width; j++)
              {
                if (context.getBlockToPatch()[((i + patch_top) / occupancyResolution)*(width / occupancyResolution) + (j + patch_left) / occupancyResolution] == patchIdx)
                {
                  //do nothing
                  for (size_t cc = 0; cc < 3; cc++)
                  {
                    destImage.setValue(cc, j + patch_left, i + patch_top, tmpImage.getValue(cc, j, i));
                  }
                }
              }
            }

          }
        }
        //saving the video
        video420.write420(srcYuvFileName,nbyte);
      }
      else{
        if (!video.write( srcRgbFileName, nbyte )) {
          return false;
        }
        std::stringstream cmd;
        cmd << colorSpaceConversionPath << " -f " << colorSpaceConversionConfig
            << " -p SourceFile=\"" << srcRgbFileName << "\""
            << " -p OutputFile=\"" << srcYuvFileName << "\""
            << " -p SourceWidth="  << width
            << " -p SourceHeight=" << height
            << " -p NumberOfFrames=" << frameCount;
        std::cout << cmd.str() << '\n';
        if (pcc::system(cmd.str().c_str())) {
          std::cout << "Error: can't run system command!" << std::endl;
          return false;
        }
    }
  }

    std::stringstream cmd;
    cmd << encoderPath
        << " -c " << encoderConfig
        << " -i " << srcYuvFileName
        << " --InputBitDepth=" << depth
        << " --InternalBitDepth=" << depth
        << " --InputChromaFormat=" << format
        << " --FrameRate=30 "
        << " --FrameSkip=0 "
        << " --SourceWidth=" << width
        << " --SourceHeight=" << height
        << " --FramesToBeEncoded=" << frameCount
        << " --BitstreamFile=" << binFileName
        << " --ReconFile=" << recYuvFileName
        << " --QP=" << qp;
    std::cout << cmd.str() << '\n';
    if (pcc::system(cmd.str().c_str())) {
      std::cout << "Error: can't run system command!" << std::endl;
      return false;
    }

    std::ifstream file(binFileName, std::ios::binary | std::ios::ate);
    if (!file.good()) {
      return false;
    }
    const uint64_t fileSize = file.tellg();
    bitstream.write<uint32_t>(uint32_t(fileSize));
    assert(bitstream.size() + fileSize < bitstream.capacity());
    file.clear();
    file.seekg(0);
    file.read(reinterpret_cast<char *>(bitstream.buffer()) + bitstream.size(), fileSize);
    bitstream += fileSize;
    file.close();

    if ( yuvVideo ) {
      if ( use444CodecIo ) {
        video.read( recYuvFileName, width, height, frameCount, nbyte );
      } else {
        video.read420( recYuvFileName, width, height, frameCount, nbyte );
      }
    } else {
      std::stringstream cmd;
      cmd << colorSpaceConversionPath
          << " -f " << inverseColorSpaceConversionConfig
          << " -p SourceFile=\"" << recYuvFileName << "\""
          << " -p OutputFile=\"" << recRgbFileName << "\""
          << " -p SourceWidth=" << width
          << " -p SourceHeight=" << height
          << " -p NumberOfFrames=" << frameCount;
      std::cout << cmd.str() << '\n';
      if (int ret = pcc::system(cmd.str().c_str())) {
        std::cout << "Error: can't run system command!" << std::endl;
        return ret;
      }
      video.read( recRgbFileName, width, height, frameCount, nbyte);
    }
    if( !keepIntermediateFiles ) {
      removeFile( binFileName    );
      removeFile( srcYuvFileName );
      removeFile( srcRgbFileName );
      removeFile( recYuvFileName );
      removeFile( recRgbFileName );
    }
    return true;
  }
 private:

};



}; //~namespace

#endif /* PCCVideoEncoder_h */
