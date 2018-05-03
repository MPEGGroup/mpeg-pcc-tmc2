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

#ifndef PCCPatch_h
#define PCCPatch_h

#include "PCCCommon.h"

namespace pcc {


class PCCPatch {
 public:
  PCCPatch(){};
  ~PCCPatch(){
    depth_[0].clear();
    depth_[1].clear();
    occupancy_.clear();
  };
  size_t&                     getIndex()                     { return index_;                 }
  size_t&                     getU1()                        { return u1_;                    }
  size_t&                     getV1()                        { return v1_;                    }
  size_t&                     getD1()                        { return d1_;                    }
  size_t&                     getSizeD()                     { return sizeD_;                 }
  size_t&                     getSizeU()                     { return sizeU_;                 }
  size_t&                     getSizeV()                     { return sizeV_;                 }
  size_t&                     getU0()                        { return u0_;                    }
  size_t&                     getV0()                        { return v0_;                    }
  size_t&                     getSizeU0()                    { return sizeU0_;                }
  size_t&                     getSizeV0()                    { return sizeV0_;                }
  size_t&                     setViewId()                    { return viewId_;                }
  size_t&                     setBestMatchIdx()              { return bestMatchIdx_;          }
  size_t&                     getOccupancyResolution()       { return occupancyResolution_;   }
  size_t&                     getNormalAxis()                { return normalAxis_;            }
  size_t&                     getTangentAxis()               { return tangentAxis_;           }
  size_t&                     getBitangentAxis()             { return bitangentAxis_;         }
  std::vector<int16_t>&       getDepth( int i )              { return depth_[i];              }
  std::vector<bool>&          getOccupancy()                 { return occupancy_;             }

  size_t                      getIndex()               const { return index_;                 }
  size_t                      getU1()                  const { return u1_;                    }
  size_t                      getV1()                  const { return v1_;                    }
  size_t                      getD1()                  const { return d1_;                    }
  size_t                      getSizeD()               const { return sizeD_;                 }
  size_t                      getSizeU()               const { return sizeU_;                 }
  size_t                      getSizeV()               const { return sizeV_;                 }
  size_t                      getU0()                  const { return u0_;                    }
  size_t                      getV0()                  const { return v0_;                    }
  size_t                      getSizeU0()              const { return sizeU0_;                }
  size_t                      getSizeV0()              const { return sizeV0_;                }
  size_t                      getOccupancyResolution() const { return occupancyResolution_;   }
  size_t                      getNormalAxis()          const { return normalAxis_;            }
  size_t                      getTangentAxis()         const { return tangentAxis_;           }
  size_t                      getBitangentAxis()       const { return bitangentAxis_;         }
  size_t                      getViewId()              const { return viewId_;                }
  size_t                      getBestMatchIdx()        const { return bestMatchIdx_;          }
  const std::vector<int16_t>& getDepth( int i )        const { return depth_[i];              }
  const std::vector<bool>&    getOccupancy()           const { return occupancy_;             }

  void print() const {
    printf("Patch[%3zu] uv0 = %4zu %4zu / %4zu %4zu uvd1 = %4zu %4zu %4zu / %4zu %4zu %4zu \n",
           index_, u0_, v0_, sizeU0_, sizeV0_, u1_, v1_, d1_, sizeU_, sizeV_, sizeD_ );
  }
  friend bool operator<(const PCCPatch &lhs, const PCCPatch &rhs) {
    return lhs.sizeV_ != rhs.sizeV_
        ? lhs.sizeV_ > rhs.sizeV_
            : (lhs.sizeU_ != rhs.sizeU_ ? lhs.sizeU_ > rhs.sizeU_ : lhs.index_ < rhs.index_);
  }
 private:
  size_t index_;                   // patch index
  size_t u1_;                      // tangential shift
  size_t v1_;                      // bitangential shift
  size_t d1_;                      // depth shift
  size_t sizeD_;                   // size for depth
  size_t sizeU_;                   // size for depth
  size_t sizeV_;                   // size for depth
  size_t u0_;                      // location in packed image
  size_t v0_;                      // location in packed image
  size_t sizeU0_;                  // size of occupancy map
  size_t sizeV0_;                  // size of occupancy map
  size_t occupancyResolution_;     // ocupacy map resolution

  size_t normalAxis_;              // x
  size_t tangentAxis_;             // y
  size_t bitangentAxis_;           // z
  std::vector<int16_t> depth_[2];  // depth
  std::vector<bool> occupancy_;    // occupancy map

  size_t viewId_;                  //viewId in [0,1,2,3,4,5]
  size_t bestMatchIdx_;            //index of matched patch from pre-frame patch.
};
}

#endif /* PCCPatch_h */
