
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

#ifndef PCCVideoDecoder_h
#define PCCVideoDecoder_h

#include "PCCCommon.h"
#include "PCCVideoBitstream.h"
#include "PCCSystem.h"
#include "PCCVideo.h"

#include "PCCVideoDecoder.h"

#include "PCCContext.h"
#include "PCCFrameContext.h"
#include "PCCPatch.h"


namespace pcc {

class PCCBitstream;

class PCCVideoDecoder {
 public:
  PCCVideoDecoder();
  ~PCCVideoDecoder();

  template <typename T>
  bool decompress(PCCVideo<T, 3> &video,
                  const std::string &fileName,
                  const size_t width,
                  const size_t height,
                  const size_t frameCount,
                  PCCVideoBitstream &bitstream,
                  const std::string &decoderPath,
                  PCCContext& contexts,
                  const std::string &inverseColorSpaceConversionConfig = "",
                  const std::string &colorSpaceConversionPath = "",
                  const bool use444CodecIo = false,
                  const bool patchColorSubsampling = false,
                  const size_t nbyte = 1,
                  const bool keepIntermediateFiles = false ) {
    const std::string binFileName = fileName + ".bin";
    const std::string yuvRecFileName = addVideoFormat(fileName + "_rec" + (use444CodecIo ? ".rgb" : ".yuv"),
                                                      width, height, !use444CodecIo, nbyte==2?"10":"8");
    const std::string rgbRecFileName = addVideoFormat(fileName + "_rec.rgb", width, height, true, nbyte==2?"10":"8");
    std::ofstream file(binFileName, std::ios::binary);
    const std::string format = use444CodecIo ? "444" : "420";
    if (!file.good()) {
      return false;
    }
    file.write(reinterpret_cast<char *>(bitstream.buffer()), bitstream.size());
    file.close();
    std::stringstream cmd;

    if(use444CodecIo) {
      cmd << decoderPath << " --OutputColourSpaceConvert=GBRtoRGB"
          << " --BitstreamFile=" << binFileName << " --ReconFile=" << yuvRecFileName;
    } else {
      cmd << decoderPath << " --BitstreamFile=" << binFileName << " --ReconFile=" << yuvRecFileName;
      // if nbyte == 1 ensure output bitdepth as 8bit. This is to cater for case if 10bit encoding was used for lossy cases.
      if (nbyte == 1) {
        cmd << " --OutputBitDepth=8 --OutputBitDepthC=8";
      }
    }

    std::cout << cmd.str() << '\n';
    if (pcc::system(cmd.str().c_str())) {
      std::cout << "Error: can't run system command!" << std::endl;
      return false;
    }
    // todo: should use444CodecIo allow conversion to happen?
    if (inverseColorSpaceConversionConfig.empty() || colorSpaceConversionPath.empty() ||
        use444CodecIo) {
      if (use444CodecIo) {
        if (!video.read(yuvRecFileName, width, height, frameCount, nbyte)) {
          return false;
        }
      } else {
        if (!video.read420(yuvRecFileName, width, height, frameCount, nbyte)) {
          return false;
        }
      }
    } else {
      if (patchColorSubsampling) {
        PCCVideo<T, 3> video420;
        if (!video420.read420(yuvRecFileName, width, height, frameCount, nbyte)) {
          return false;
        }
        //allocate the output
        video.resize(frameCount);
        //perform color-upsampling based on patch information
        for (size_t frNum = 0; frNum < video.getFrameCount(); frNum++) {
          //context variable, contains the patch information
          auto& context = contexts[frNum / 2];
          //full resolution image (already filled by previous dilation
          auto &refImage = video420.getFrame(frNum);
          //image that will contain the per-patch chroma sub-sampled image
          auto &destImage = video.getFrame(frNum);
          destImage.resize(width, height);

          //iterate the patch information and perform chroma down-sampling on each patch individually
          std::vector<PCCPatch> patches = context.getPatches();
          std::vector<size_t> blockToPatch = context.getBlockToPatch();
          for (int patchIdx = 0; patchIdx <= patches.size(); patchIdx++) {
            size_t occupancyResolution;
            size_t patch_left;
            size_t patch_top;
            size_t patch_width;
            size_t patch_height;
            if (patchIdx == 0) {
              //background, does not have a corresponding patch
              auto& patch = patches[0];
              occupancyResolution = patch.getOccupancyResolution();
              patch_left = 0;
              patch_top = 0;
              patch_width = width;
              patch_height = height;
            } else {
              auto& patch = patches[patchIdx - 1];
              occupancyResolution = patch.getOccupancyResolution();
              patch_left = patch.getU0() * occupancyResolution;
              patch_top = patch.getV0() * occupancyResolution;
              if (!(patch.isPatchDimensionSwitched())){
                patch_width = patch.getSizeU0() * occupancyResolution;
                patch_height = patch.getSizeV0() * occupancyResolution;
              } else {
                patch_width = patch.getSizeV0() * occupancyResolution;
                patch_height = patch.getSizeU0() * occupancyResolution;
              }
            }
            //initializing the image container with zeros
            PCCImage<T, 3> tmpImage;
            tmpImage.resize(patch_width, patch_height);
            //cut out the patch image
            refImage.copyBlock(patch_top, patch_left, patch_width, patch_height, tmpImage);

            //fill in the blocks by extending the edges
            for (size_t i = 0; i < patch_height / occupancyResolution; i++) {
              for (size_t j = 0; j < patch_width / occupancyResolution; j++) {
                if (context.getBlockToPatch()[(i + patch_top / occupancyResolution)*(width / occupancyResolution) + j + patch_left / occupancyResolution] == patchIdx) {
                  //do nothing
                  continue;
                } else {
                  //search for the block that contains texture information and extend the block edge 
                  int direction;
                  int searchIndex;
                  std::vector<int> neighborIdx(4, -1);
                  std::vector<int> neighborDistance(4, (std::numeric_limits<int>::max)());
                  //looking for the neighboring block to the left of the current block
                  searchIndex = j;
                  while (searchIndex >= 0) {
                    if (context.getBlockToPatch()[(i + patch_top / occupancyResolution)*(width / occupancyResolution) + searchIndex + patch_left / occupancyResolution] == patchIdx) {
                      neighborIdx[0] = searchIndex;
                      neighborDistance[0] = j - searchIndex;
                      searchIndex = 0;
                    }
                    searchIndex--;
                  }
                  //looking for the neighboring block to the right of the current block
                  searchIndex = j;
                  while (searchIndex < patch_width / occupancyResolution) {
                    if (context.getBlockToPatch()[(i + patch_top / occupancyResolution)*(width / occupancyResolution) + searchIndex + patch_left / occupancyResolution] == patchIdx) {
                      neighborIdx[1] = searchIndex;
                      neighborDistance[1] = searchIndex - j;
                      searchIndex = patch_width / occupancyResolution;
                    }
                    searchIndex++;
                  }
                  //looking for the neighboring block above the current block
                  searchIndex = i;
                  while (searchIndex >= 0) {
                    if (context.getBlockToPatch()[(searchIndex + patch_top / occupancyResolution)*(width / occupancyResolution) + j + patch_left / occupancyResolution] == patchIdx) {
                      neighborIdx[2] = searchIndex;
                      neighborDistance[2] = i - searchIndex;
                      searchIndex = 0;
                    }
                    searchIndex--;
                  }
                  //looking for the neighboring block below the current block
                  searchIndex = i;
                  while (searchIndex < patch_height / occupancyResolution) {
                    if (context.getBlockToPatch()[(searchIndex + patch_top / occupancyResolution)*(width / occupancyResolution) + j + patch_left / occupancyResolution] == patchIdx) {
                      neighborIdx[3] = searchIndex;
                      neighborDistance[3] = searchIndex - i;
                      searchIndex = patch_height / occupancyResolution;
                    }
                    searchIndex++;
                  }
                  //check if the candidate was found
                  assert(*(std::max)(neighborIdx.begin(),neighborIdx.end()) > 0);
                  //now fill in the block with the edge value coming from the nearest neighbor
                  direction = (std::min_element)(neighborDistance.begin(), neighborDistance.end()) - neighborDistance.begin();
                  if (direction == 0) {
                    //copying from left neighboring block
                    for (size_t iBlk = 0; iBlk < occupancyResolution; iBlk++) {
                      for (size_t jBlk = 0; jBlk < occupancyResolution; jBlk++) {
                        tmpImage.setValue(0, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(0, neighborIdx[0] * occupancyResolution + occupancyResolution - 1, i * occupancyResolution + iBlk));
                        tmpImage.setValue(1, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(1, neighborIdx[0] * occupancyResolution + occupancyResolution - 1, i * occupancyResolution + iBlk));
                        tmpImage.setValue(2, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(2, neighborIdx[0] * occupancyResolution + occupancyResolution - 1, i * occupancyResolution + iBlk));
                      }
                    }
                  } else if (direction == 1) {
                    //copying block from right neighboring position
                    for (size_t iBlk = 0; iBlk < occupancyResolution; iBlk++) {
                      for (size_t jBlk = 0; jBlk < occupancyResolution; jBlk++) {
                        tmpImage.setValue(0, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(0, neighborIdx[1] * occupancyResolution, i * occupancyResolution + iBlk));
                        tmpImage.setValue(1, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(1, neighborIdx[1] * occupancyResolution, i * occupancyResolution + iBlk));
                        tmpImage.setValue(2, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(2, neighborIdx[1] * occupancyResolution, i * occupancyResolution + iBlk));
                      }
                    }
                  } else if (direction == 2) {
                    //copying block from above
                    for (size_t iBlk = 0; iBlk < occupancyResolution; iBlk++) {
                      for (size_t jBlk = 0; jBlk < occupancyResolution; jBlk++) {
                        tmpImage.setValue(0, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(0, j * occupancyResolution + jBlk, neighborIdx[2] * occupancyResolution + occupancyResolution - 1));
                        tmpImage.setValue(1, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(1, j * occupancyResolution + jBlk, neighborIdx[2] * occupancyResolution + occupancyResolution - 1));
                        tmpImage.setValue(2, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(2, j * occupancyResolution + jBlk, neighborIdx[2] * occupancyResolution + occupancyResolution - 1));
                      }
                    }
                  } else if (direction == 3) {
                    //copying block from below
                    for (size_t iBlk = 0; iBlk < occupancyResolution; iBlk++) {
                      for (size_t jBlk = 0; jBlk < occupancyResolution; jBlk++) {
                        tmpImage.setValue(0, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(0, j * occupancyResolution + jBlk, neighborIdx[3] * occupancyResolution));
                        tmpImage.setValue(1, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(1, j * occupancyResolution + jBlk, neighborIdx[3] * occupancyResolution));
                        tmpImage.setValue(2, j * occupancyResolution + jBlk, i * occupancyResolution + iBlk, tmpImage.getValue(2, j * occupancyResolution + jBlk, neighborIdx[3] * occupancyResolution));
                      }
                    }
                  } else {
                    printf("This condition should never occur, report an error");
                    return false;
                  }
                }
              }
            }

            //perform downsampling
            const std::string rgbRecFileNamePatch = addVideoFormat(fileName + "_tmp.rgb", patch_width, patch_height);
            const std::string yuvRecFileNamePatch = addVideoFormat(fileName + "_tmp.yuv", patch_width, patch_height, true);
            if (!tmpImage.write420(yuvRecFileNamePatch, nbyte)) {
              return false;
            }
            std::stringstream cmd;
            cmd << colorSpaceConversionPath << " -f " << inverseColorSpaceConversionConfig
                << " -p SourceFile=\"" << yuvRecFileNamePatch << "\" -p OutputFile=\"" << rgbRecFileNamePatch
                << "\" -p SourceWidth=" << patch_width << " -p SourceHeight=" << patch_height
                << " -p NumberOfFrames=1";
            std::cout << cmd.str() << '\n';
            if (pcc::system(cmd.str().c_str())) {
              std::cout << "Error: can't run system command!" << std::endl;
              return false;
            }
            tmpImage.read(rgbRecFileNamePatch, patch_width, patch_height,nbyte);
            //removing intermediate files
            remove(rgbRecFileNamePatch.c_str());
            remove(yuvRecFileNamePatch.c_str());
            //substitute the pixels in the output image for compression
            for (size_t i = 0; i < patch_height ; i++) {
              for (size_t j = 0; j < patch_width ; j++) {
                if (context.getBlockToPatch()[((i + patch_top) / occupancyResolution)*(width / occupancyResolution) + (j + patch_left) / occupancyResolution] == patchIdx) {
                  //do nothing
                  for (size_t cc = 0; cc < 3; cc++) {
                    destImage.setValue(cc,j+patch_left,i+patch_top, tmpImage.getValue(cc,j,i));
                  }
                }
              }
            }
          }
        }
      } else {
        std::stringstream cmd;
        cmd << colorSpaceConversionPath << " -f " << inverseColorSpaceConversionConfig
            << " -p SourceFile=\"" << yuvRecFileName << "\" -p OutputFile=\"" << rgbRecFileName
            << "\" -p SourceWidth=" << width << " -p SourceHeight=" << height
            << " -p NumberOfFrames=" << frameCount;
        std::cout << cmd.str() << '\n';
        if (pcc::system(cmd.str().c_str())) {
          std::cout << "Error: can't run system command!" << std::endl;
          return false;
        }
        if (!video.read(rgbRecFileName, width, height, frameCount, nbyte)) {
          return false;
        }
      }
    }
    if (!keepIntermediateFiles) {
      removeFile( binFileName    );
      removeFile( yuvRecFileName );
      removeFile( rgbRecFileName );
    }
    return true;
  }
 private:

};


}; //~namespace

#endif /* PCCVideoDecoder_h */
