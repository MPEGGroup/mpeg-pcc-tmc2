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

#ifndef PCCMetadata_h
#define PCCMetadata_h

#include "PCCCommon.h"
#include "PCCMath.h"

namespace pcc {

  typedef PCCVector3<uint32_t> PCCVector3U;
  typedef PCCVector3<int32_t>  PCCVector3I;
  enum    PointShape { Circle = 0, Square, Diamond };

  class PCCMetadataEnabledFlags {
  public:
    PCCMetadataEnabledFlags() :
      metadataEnabled_            (false),
      scaleEnabled_               (false),
      offsetEnabled_              (false),
      rotationEnabled_            (false),
      pointSizeEnabled_           (false), 
      pointShapeEnabled_          (false)
    {};
    ~PCCMetadataEnabledFlags() {};

    bool& getMetadataEnabled()          { return metadataEnabled_;    }
    bool& getScaleEnabled()             { return scaleEnabled_;       }
    bool& getOffsetEnabled()            { return offsetEnabled_;      }
    bool& getRotationEnabled()          { return rotationEnabled_;    }
    bool& getPointSizeEnabled()         { return pointSizeEnabled_;   }
    bool& getPointShapeEnabled()        { return pointShapeEnabled_;  }

    bool  getMetadataEnabled()    const { return metadataEnabled_;    }
    bool  getScaleEnabled()       const { return scaleEnabled_;       }
    bool  getOffsetEnabled()      const { return offsetEnabled_;      }
    bool  getRotationEnabled()    const { return rotationEnabled_;    }
    bool  getPointSizeEnabled()   const { return pointSizeEnabled_;   }
    bool  getPointShapeEnabled()  const { return pointShapeEnabled_;  }

    void  setMetadataEnabled(bool flag           ) { metadataEnabled_=flag;    }

  private:
    bool metadataEnabled_;
    bool scaleEnabled_;
    bool offsetEnabled_;
    bool rotationEnabled_;
    bool pointSizeEnabled_;
    bool pointShapeEnabled_;
  };

  class PCCMetadata {
  public:
    PCCMetadata() :
      metadataPresent_                (false),
      scalePresent_                   (false),
      scale_                          (1),
      offsetPresent_                  (false),
      offset_                         (0),
      rotationPresent_                (false),
      rotation_                       (0),
      pointSizePresent_               (false),
      pointSize_                      (1),
      pointShapePresent_              (false),
      pointShape_                     (Circle),
      metadataType_                   (METADATA_GOF)
    {};
    PCCMetadata(const PCCMetadata& pccMetadata) {
      scalePresent_                   = pccMetadata.getScalePresent();
      scale_                          = pccMetadata.getScale();
      offsetPresent_                  = pccMetadata.getOffsetPresent();
      offset_                         = pccMetadata.getOffset();
      rotationPresent_                = pccMetadata.getRotationPresent();
      rotation_                       = pccMetadata.getRotation();
      pointSizePresent_               = pccMetadata.getPointSizePresent();
      pointSize_                      = pccMetadata.getPointSize();
      pointShapePresent_              = pccMetadata.getPointShapePresent();
      pointShape_                     = pccMetadata.getPointShape();
      metadataEnabledFlags_           = pccMetadata.getMetadataEnabledFlags();
      lowerLevelMetadataEnabledFlags_ = pccMetadata.getLowerLevelMetadataEnabledFlags();
      metadataType_                   = pccMetadata.getMetadataType();
    }
    ~PCCMetadata() {};
    
    bool&                             getMetadataPresent()                      { return metadataPresent_;                }
    void                              setMetadataPresent(bool flag)             { metadataPresent_=flag;}
    bool&                             getScalePresent()                         { return scalePresent_;                   }
    PCCVector3U&                      getScale()                                { return scale_;                          }
    bool&                             getOffsetPresent()                        { return offsetPresent_;                  }
    PCCVector3I&                      getOffset()                               { return offset_;                         }
    bool&                             getRotationPresent()                      { return rotationPresent_;                }
    PCCVector3I&                      getRotation()                             { return rotation_;                       }
    bool&                             getPointSizePresent()                     { return pointSizePresent_;               }
    uint16_t&                         getPointSize()                            { return pointSize_;                      }
    bool&                             getPointShapePresent()                    { return pointShapePresent_;              }
    PointShape&                       getPointShape()                           { return pointShape_;                     }
    PCCMetadataEnabledFlags&          getMetadataEnabledFlags()                 { return metadataEnabledFlags_;           }
    PCCMetadataEnabledFlags&          getLowerLevelMetadataEnabledFlags()       { return lowerLevelMetadataEnabledFlags_; }

    bool                              getMetadataPresent()                const { return metadataPresent_;                }
    bool                              getScalePresent()                   const { return scalePresent_;                   }
    const PCCVector3U&                getScale()                          const { return scale_;                          }
    bool                              getOffsetPresent()                  const { return offsetPresent_;                  }
    const PCCVector3I&                getOffset()                         const { return offset_;                         }
    bool                              getRotationPresent()                const { return rotationPresent_;                }
    const PCCVector3I&                getRotation()                       const { return rotation_;                       }
    bool                              getPointSizePresent()               const { return pointSizePresent_;               }
    uint16_t                          getPointSize()                      const { return pointSize_;                      }
    bool                              getPointShapePresent()              const { return pointShapePresent_;              }
    PointShape                        getPointShape()                     const { return pointShape_;                     }

    const PCCMetadataEnabledFlags&    getMetadataEnabledFlags()           const { return metadataEnabledFlags_;           }
    const PCCMetadataEnabledFlags&    getLowerLevelMetadataEnabledFlags() const { return lowerLevelMetadataEnabledFlags_; }

    PCCMetadataType                   getMetadataType() const {return metadataType_;}
    void                              setMetadataType(PCCMetadataType type){metadataType_=type;}
    size_t                            getIndex()              const { return index_;    }
    void                              setIndex(size_t index)        {  index_=index;    }

  private:
    bool                              metadataPresent_;
    bool                              scalePresent_;
    PCCVector3U                       scale_;
    bool                              offsetPresent_;
    PCCVector3I                       offset_;
    bool                              rotationPresent_;
    PCCVector3I                       rotation_;
    bool                              pointSizePresent_;
    uint16_t                          pointSize_;
    bool                              pointShapePresent_;
    PointShape                        pointShape_;
    PCCMetadataEnabledFlags           metadataEnabledFlags_;
    PCCMetadataEnabledFlags           lowerLevelMetadataEnabledFlags_;
    size_t                            index_;
    PCCMetadataType                   metadataType_;

  };

}; //~namespace

#endif

