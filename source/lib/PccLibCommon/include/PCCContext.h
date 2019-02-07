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
#ifndef PCCContext_h
#define PCCContext_h

#include "PCCCommon.h"
#include "PCCVideo.h"
#include "PCCMetadata.h"
#include "PCCVideoBitstream.h"
#include <map>

namespace pcc {

class PCCGroupOfFrames;
class PCCFrameContext;
typedef pcc::PCCVideo<uint8_t, 3>  PCCVideoTexture;
typedef pcc::PCCVideo<uint16_t, 3> PCCVideoGeometry;
typedef pcc::PCCVideo<uint8_t, 3>  PCCVideoOccupancyMap;

class PCCPatch;
typedef std::map<size_t, PCCPatch> unionPatch;                      // unionPatch ------ [TrackIndex, PatchUnion];
typedef std::pair<size_t, size_t>  GlobalPatch;                     // GlobalPatch ----- [FrameIndex, PatchIndex];
typedef std::map<size_t, std::vector<GlobalPatch> > GlobalPatches;  // GlobalPatches --- [TrackIndex, <GlobalPatch>];
typedef std::pair<size_t, size_t>  SubContext;                      // SubContext ------ [start, end);

class PCCContext {
 public:
  PCCContext();
  ~PCCContext();

  std::vector<PCCFrameContext>::iterator begin() { return frames_.begin(); }
  std::vector<PCCFrameContext>::iterator end  () { return frames_.end  (); }

  void resize( size_t size );

  const size_t size      () { return frames_.size(); }
  std::vector<PCCFrameContext>& getFrames() { return frames_; }
  PCCFrameContext&      operator[]     ( int i ) { return frames_[i]; }

  PCCVideoGeometry&     getVideoGeometry    () { return videoGeometry_; }
  PCCVideoGeometry&     getVideoGeometryD1  () { return videoGeometryD1_; }
  PCCVideoTexture&      getVideoTexture     () { return videoTexture_; }
  PCCVideoOccupancyMap& getVideoOccupancyMap() { return videoOccupancyMap_; }
  PCCVideoGeometry&     getVideoMPsGeometry () { return videoMPsGeometry_; }
  PCCVideoTexture&      getVideoMPsTexture  () { return videoMPsTexture_; }
  

  bool&              getLosslessGeo444              () { return losslessGeo444_;               }
  bool&              getLosslessGeo                 () { return losslessGeo_;                  }
  bool&              getLosslessTexture             () { return losslessTexture_;              }
  uint8_t&           getOccupancyResolution         () { return occupancyResolution_;          }
  uint8_t&           getOccupancyPrecision          () { return occupancyPrecision_;           }
  uint8_t&           getFlagGeometrySmoothing       () { return flagGeometrySmoothing_;        }
  bool&              getGridSmoothing               () { return gridSmoothing_;                }
  uint8_t&           getGridSize                    () { return gridSize_;                     }
  uint8_t&           getRadius2Smoothing            () { return radius2Smoothing_;             }
  uint8_t&           getNeighborCountSmoothing      () { return neighborCountSmoothing_;       }
  uint8_t&           getRadius2BoundaryDetection    () { return radius2BoundaryDetection_;     }
  uint8_t&           getThresholdSmoothing          () { return thresholdSmoothing_;           }
  uint8_t&           getNoAttributes                () { return noAttributes_;                 }
  bool&              getAbsoluteD1                  () { return absoluteD1_;                   }
  bool&              getBinArithCoding              () { return binArithCoding_;               }
  float&             getModelScale                  () { return modelScale_;                   }
  PCCVector3<float>& getModelOrigin                 () { return modelOrigin_;                  }
  uint8_t&           getThresholdColorSmoothing     () { return thresholdColorSmoothing_;      }
  double &           getThresholdLocalEntropy       () { return thresholdLocalEntropy_;        }
  uint8_t&           getRadius2ColorSmoothing       () { return radius2ColorSmoothing_;        }
  uint8_t&           getNeighborCountColorSmoothing () { return neighborCountColorSmoothing_;  }
  uint8_t&           getFlagColorSmoothing          () { return flagColorSmoothing_;           }
  bool&              getEnhancedDeltaDepthCode      () { return enhancedDeltaDepthCode_;       }
  bool&              getImproveEDD                  () { return improveEDD_;                   }
  bool&              getDeltaCoding                 () { return deltaCoding_;                  }
  bool&              getSixDirectionMode            () { return sixDirectionMode_;             }
  bool&              getRemoveDuplicatePoints       () { return removeDuplicatePoints_;        }
  bool&              getOneLayerMode                () { return oneLayerMode_;                 }
  bool&              getSingleLayerPixelInterleaving() { return singleLayerPixelInterleaving_; }
  bool&              getUseAdditionalPointsPatch    () { return useAdditionalPointsPatch_;     }
  uint8_t&           getMinLevel                    () { return minLevel_;                     }
  bool&              getGlobalPatchAllocation       () { return globalPatchAllocation_;        }
  bool&              getUse3dmc                     () { return use3dmc_;                      }

  const size_t       getMPGeoWidth                  () { return MPGeoWidth_;                   }
  const size_t       getMPGeoHeight                 () { return MPGeoHeight_;                  }
  const size_t       getMPAttWidth                  () { return MPAttWidth_;                   }
  const size_t       getMPAttHeight                 () { return MPAttHeight_;                  }
  bool&              getUseMissedPointsSeparateVideo() { return useMissedPointsSeparateVideo_; }
  bool&              getEnhancedDeltaDepth          () { return enhancedDeltaDepth_;           }
  size_t             getIndex                       () { return index_;                        }
  size_t&            getWidth                       () { return width_;                        }
  size_t&            getHeight                      () { return height_;                       }

  void setLosslessGeo444              ( bool losslessGeo444  ) { losslessGeo444_              = losslessGeo444;  }
  void setLossless                    ( bool losslessGeo     ) { losslessGeo_                 = losslessGeo;     }
  void setLosslessTexture             ( bool losslessTexture ) { losslessTexture_             = losslessTexture; }
  void setMPGeoWidth                  ( size_t width         ) { MPGeoWidth_                  = width;           }
  void setMPGeoHeight                 ( size_t height        ) { MPGeoHeight_                 = height;          }
  void setMPAttWidth                  ( size_t width         ) { MPAttWidth_                  = width;           }
  void setMPAttHeight                 ( size_t height        ) { MPAttHeight_                 = height;          }
  void setIndex                       ( size_t index         ) { index_                       = index;           }
  void setUseMissedPointsSeparateVideo( bool code            ) { useMissedPointsSeparateVideo_= code;            }
  void setEnhancedDeltaDepth          ( bool code            ) { enhancedDeltaDepth_          = code;            }
 
  PCCMetadata&      getGOFLevelMetadata() { return gofLevelMetadata_; }

  std::vector<SubContext>&  getSubContexts() { return subContexts_; }
  std::vector<unionPatch>&  getunionPatch() { return unionPatch_; }

  PCCVideoBitstream& createVideoBitstream( PCCVideoType type ){
    videoBitstream_.push_back( PCCVideoBitstream( type ) );
    return videoBitstream_.back();
  }
  size_t getVideoBitstreamCount(){ return videoBitstream_.size();   }
  PCCVideoBitstream& getVideoBitstream( size_t index ){ return videoBitstream_[index]; }
  PCCVideoBitstream& getVideoBitstream( PCCVideoType type  ) {
    for( auto& value: videoBitstream_ ) {
      if( value.type() == type ){
        return value;
      }
    }
    printf("ERROR: can't get video bitstream of type %s \n",toString( type ).c_str() ); fflush(stdout);
    exit(-1);
  }

  void allocOneLayerData( const size_t occupancyResolution );
  void printVideoBitstream();
  void printBlockToPatch( const size_t occupancyResolution );

 private:
  size_t                         index_;
  size_t                         width_;
  size_t                         height_;
  std::vector<PCCFrameContext>   frames_;
  PCCVideoGeometry               videoGeometry_;
  PCCVideoGeometry               videoGeometryD1_;
  PCCVideoTexture                videoTexture_;
  PCCVideoOccupancyMap           videoOccupancyMap_;

  PCCVideoGeometry               videoMPsGeometry_;
  PCCVideoTexture                videoMPsTexture_;
  bool                           losslessGeo444_;
  bool                           losslessGeo_;
  bool                           losslessTexture_;
  size_t                         MPGeoWidth_;
  size_t                         MPGeoHeight_;
  size_t                         MPAttWidth_;
  size_t                         MPAttHeight_;
  bool                           useMissedPointsSeparateVideo_;
  bool                           enhancedDeltaDepth_;
  PCCMetadata                    gofLevelMetadata_;


  std::vector<PCCVideoBitstream> videoBitstream_;
  std::vector<SubContext>        subContexts_;
  std::vector<unionPatch>        unionPatch_;

  uint8_t                        occupancyResolution_;
  uint8_t                        occupancyPrecision_;
  uint8_t                        flagGeometrySmoothing_;
  bool                           gridSmoothing_;
  uint8_t                        gridSize_;
  uint8_t                        radius2Smoothing_;
  uint8_t                        neighborCountSmoothing_;
  uint8_t                        radius2BoundaryDetection_;
  uint8_t                        thresholdSmoothing_;
  uint8_t                        noAttributes_;
  bool                           absoluteD1_;
  bool                           binArithCoding_;
  float                          modelScale_;
  PCCVector3<float>              modelOrigin_;
  uint8_t                        thresholdColorSmoothing_;
  double                         thresholdLocalEntropy_;
  uint8_t                        radius2ColorSmoothing_;
  uint8_t                        neighborCountColorSmoothing_;
  uint8_t                        flagColorSmoothing_;
  bool                           enhancedDeltaDepthCode_;
  bool                           improveEDD_;
  bool                           deltaCoding_;
  bool                           sixDirectionMode_;
  bool                           removeDuplicatePoints_;
  bool                           oneLayerMode_;
  bool                           singleLayerPixelInterleaving_;
  bool                           useAdditionalPointsPatch_;
  uint8_t                        minLevel_;
  bool                           globalPatchAllocation_;
  bool                           use3dmc_;
};
}; //~namespace

#endif /* PCCContext_h */
