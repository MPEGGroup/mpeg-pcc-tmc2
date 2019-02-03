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
#include "PCCMetadata.h"

namespace pcc {


struct GPAPatchData
{
	bool isMatched;
	bool isGlobalPatch;
	int  globalPatchIndex;
	
	size_t sizeU0;
	size_t sizeV0;
	std::vector<bool> occupancy;

	size_t u0;
	size_t v0;
	size_t patchOrientation;

	void initialize() {
		isMatched        = false;
		isGlobalPatch    = false;
		globalPatchIndex = -1;

		sizeU0 = 0;
		sizeV0 = 0;
		occupancy.clear();

		u0 = -1;
		v0 = -1;
		patchOrientation = -1;
	}

	bool isPatchDimensionSwitched()
	{
		if ((patchOrientation == PatchOrientation::DEFAULT) || (patchOrientation == PatchOrientation::ROT180) || (patchOrientation == PatchOrientation::MIRROR) || (patchOrientation == PatchOrientation::MROT180))
			return false;
		else
			return true;
	}

};


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
  int32_t&                    setBestMatchIdx()              { return bestMatchIdx_;          }
  size_t&                     getOccupancyResolution()       { return occupancyResolution_;   }
  size_t&                     getProjectionMode()            { return projectionMode_; }
  size_t&                     getFrameProjectionMode()       { return frameProjectionMode_; }
  size_t&                     getNormalAxis()                { return normalAxis_;            }
  size_t&                     getTangentAxis()               { return tangentAxis_;           }
  size_t&                     getBitangentAxis()             { return bitangentAxis_;         }
  std::vector<int16_t>&       getDepth( int i )              { return depth_[i];              }
  std::vector<bool>&          getOccupancy()                 { return occupancy_;             }
  size_t&                     getLod()                       { return levelOfDetail_;         }
  PCCMetadata&                getPatchLevelMetadata()        { return patchLevelMetadata_;    }

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
  size_t                      getProjectionMode()      const { return projectionMode_;        }
  size_t                      getFrameProjectionMode() const { return frameProjectionMode_;   }
  size_t                      getNormalAxis()          const { return normalAxis_;            }
  size_t                      getTangentAxis()         const { return tangentAxis_;           }
  size_t                      getBitangentAxis()       const { return bitangentAxis_;         }
  size_t                      getViewId()              const { return viewId_;                }
  int32_t                     getBestMatchIdx()        const { return bestMatchIdx_;          }
  const std::vector<int16_t>& getDepth( int i )        const { return depth_[i];              }
  const std::vector<bool>&    getOccupancy()           const { return occupancy_;             }
  size_t                      getLod()                 const { return levelOfDetail_;         }
  std::vector<int16_t>&       getDepthEnhancedDeltaD()       { return depthEnhancedDeltaD_;   }
  const std::vector<int16_t>& getDepthEnhancedDeltaD() const { return depthEnhancedDeltaD_;   }
  const PCCMetadata&          getPatchLevelMetadata()  const { return patchLevelMetadata_;    }

  const std::vector<int64_t>& getdepth0pccidx()        const { return depth0PCidx_;           }
  std::vector<int64_t>&       getdepth0pccidx()              { return depth0PCidx_;           }
  //Flexible Patch Orientation
  size_t&                     getPatchOrientation()          { return patchOrientation_;      }
  size_t                      getPatchOrientation()    const { return patchOrientation_;      }
  bool&						  getIsGlobalPatch() { return isGlobalPatch_; }
  bool						  getIsGlobalPatch() const { return isGlobalPatch_; }
  inline double generateNormalCoordinate( const uint16_t depth, const double lodScale, const bool useMppSepVid, const bool lossyMpp ) const {
    double coord = 0;
    if (lossyMpp && ! useMppSepVid){ // support lossy missed points patch in same video frame, re-shift depth values to store in 10-bit video frame
      if (projectionMode_ == 0) {
        coord = ((double)(depth >> 2) + (double)d1_) * lodScale;   
      }
      else {
        double tmp_depth = double(d1_) - double(depth >> 2);
        if (tmp_depth > 0) {
          coord = tmp_depth * lodScale;
        }
      }
    }else{
      if (projectionMode_ == 0) {
        coord = ((double)depth + (double)d1_) * lodScale;
      }
      else {
        double tmp_depth = double(d1_) - double(depth);
        if (tmp_depth > 0) {
          coord = tmp_depth * lodScale;
        }
      }
    }
    return coord;
  }

  PCCPoint3D generatePoint( const size_t u, const size_t v, const uint16_t depth, const double lodScale, const bool useMppSepVid, const bool lossyMpp ) const {
    PCCPoint3D point0;
    point0[normalAxis_   ] = generateNormalCoordinate( depth, lodScale, useMppSepVid, lossyMpp );
    point0[tangentAxis_  ] = (double(u) + u1_) * lodScale;
    point0[bitangentAxis_] = (double(v) + v1_) * lodScale;
    return point0;
  }

  size_t patch2Canvas(const size_t u, const size_t v, size_t canvasStride, size_t canvasHeight, size_t &x, size_t &y) {
    switch( patchOrientation_ ) {
	case PatchOrientation::DEFAULT:
        x = u + u0_ * occupancyResolution_;
        y = v + v0_ * occupancyResolution_;
        break;
	case PatchOrientation::ROT90:
		x = (sizeV0_ * occupancyResolution_ - 1 - v) + u0_ * occupancyResolution_;
		y = u + v0_ * occupancyResolution_;
		break;
	case  PatchOrientation::ROT180:
        x = (sizeU0_ * occupancyResolution_ - 1 - u) + u0_ * occupancyResolution_;
        y = (sizeV0_ * occupancyResolution_ - 1 - v) + v0_ * occupancyResolution_;
        break;
	case  PatchOrientation::ROT270:
        x = v + u0_ * occupancyResolution_;
        y = (sizeU0_ * occupancyResolution_ - 1 - u) + v0_ * occupancyResolution_;
        break;
	case  PatchOrientation::MIRROR:
        x = (sizeU0_ * occupancyResolution_ - 1 - u) + u0_ * occupancyResolution_;
        y = v + v0_ * occupancyResolution_;
        break;
	case  PatchOrientation::MROT90:
        x = (sizeV0_ * occupancyResolution_ - 1 - v) + u0_ * occupancyResolution_;
        y = (sizeU0_ * occupancyResolution_ - 1 - u) + v0_ * occupancyResolution_;
        break;
	case  PatchOrientation::MROT180:
        x = u + u0_ * occupancyResolution_;
        y = (sizeV0_ * occupancyResolution_ - 1 - v) + v0_ * occupancyResolution_;
        break;
	case PatchOrientation::MROT270:
		x = v + u0_ * occupancyResolution_;
		y = u + v0_ * occupancyResolution_;
		break;
	case PatchOrientation::SWAP://swapAxis
		x = v + u0_ * occupancyResolution_;
		y = u + v0_ * occupancyResolution_;
		break;
	default : assert( 0 ); break;
    }
    //checking the results are within canvas boundary (missing y check)
    assert(x >= 0);
    assert(y >= 0);
    assert(x < canvasStride);
    assert(y < canvasHeight);
    return (x + canvasStride * y);
  }

  int patchBlock2CanvasBlock(const size_t uBlk, const size_t vBlk, size_t canvasStrideBlk, size_t canvasHeightBlk) const {
    size_t x, y;
    switch( patchOrientation_ ) {
	case PatchOrientation::DEFAULT:
		x = uBlk + u0_;
		y = vBlk + v0_;
		break;
	case   PatchOrientation::ROT90:
		x = (sizeV0_ - 1 - vBlk) + u0_;
		y = uBlk + v0_;
		break;
	case   PatchOrientation::ROT180:
		x = (sizeU0_ - 1 - uBlk) + u0_;
		y = (sizeV0_ - 1 - vBlk) + v0_;
		break;
	case   PatchOrientation::ROT270:
		x = vBlk + u0_;
		y = (sizeU0_ - 1 - uBlk) + v0_;
		break;
	case  PatchOrientation::MIRROR:
		x = (sizeU0_ - 1 - uBlk) + u0_;
		y = vBlk + v0_;
		break;
	case  PatchOrientation::MROT90:
		x = (sizeV0_ - 1 - vBlk) + u0_;
		y = (sizeU0_ - 1 - uBlk) + v0_;
		break;
	case   PatchOrientation::MROT180:
		x = uBlk + u0_;
		y = (sizeV0_ - 1 - vBlk) + v0_;
		break;
	case   PatchOrientation::MROT270:
		x = vBlk + u0_;
		y = uBlk + v0_;
		break;
    case   PatchOrientation::SWAP://swapAxis
		x = vBlk + u0_;
		y = uBlk + v0_;
		break;
	 default : return -1; break;
    }
    //checking the results are within canvas boundary (missing y check)
    if (x < 0) return -1;
    if (y < 0) return -1;
    if (x >= canvasStrideBlk) return -1;
    if (y >= canvasHeightBlk) return -1;
    return (x + canvasStrideBlk * y);
  }

  bool checkFitPatchCanvas(std::vector<bool> canvas, size_t canvasStrideBlk, size_t canvasHeightBlk, int safeguard = 0){
    for (size_t v0 = 0; v0 < getSizeV0(); ++v0) {
      for (size_t u0 = 0; u0 < getSizeU0(); ++u0) {
        for (int deltaY = -safeguard; deltaY < safeguard + 1; deltaY++) {
          for (int deltaX = -safeguard; deltaX < safeguard + 1; deltaX++) {
            int pos = patchBlock2CanvasBlock(u0 + deltaX, v0 + deltaY, canvasStrideBlk, canvasHeightBlk);
            if (pos < 0) {
              return false;
            }
            else if (canvas[pos])
            {
              return false;
            }
          }
        }
      }
    }
    return true;
  }

  bool gt(const PCCPatch &rhs) {
    size_t maxDim, rhsMaxDim;
    size_t minDim, rhsMinDim;
    //setting the largest dimension
    if (sizeU0_ > sizeV0_) {
      maxDim = sizeU0_;
      minDim = sizeV0_;
    } else {
      maxDim = sizeV0_;
      minDim = sizeU0_;
    }
    //getting the largest dimensions for the block to be tested
    if (rhs.getSizeU0() > rhs.getSizeV0()) {
      rhsMaxDim = rhs.getSizeU0();
      rhsMinDim = rhs.getSizeV0();
    } else {
      rhsMaxDim = rhs.getSizeV0();
      rhsMinDim = rhs.getSizeU0();
    }
    //if the dimensions are the same, decide by the index
    return maxDim != rhsMaxDim ? (maxDim > rhsMaxDim) : (minDim != rhsMinDim) ? (minDim > rhsMinDim) : (index_ < rhs.index_);
  }   
  
  void print() const {
    printf("Patch[%3zu] uv0 = %4zu %4zu / %4zu %4zu uvd1 = %4zu %4zu %4zu / %4zu %4zu %4zu \n",
           index_, u0_, v0_, sizeU0_, sizeV0_, u1_, v1_, d1_, sizeU_, sizeV_, sizeD_ );
  }

  void print_decoder() const {
    printf("Patch uv0 = %4zu %4zu / %4zu %4zu uvd1 = %4zu %4zu %4zu orientation = %4zu  \n",
           u0_, v0_, sizeU0_, sizeV0_, u1_, v1_, d1_, patchOrientation_);
  }

  friend bool operator<(const PCCPatch &lhs, const PCCPatch &rhs) {
    return lhs.sizeV_ != rhs.sizeV_
        ? lhs.sizeV_ > rhs.sizeV_
            : (lhs.sizeU_ != rhs.sizeU_ ? lhs.sizeU_ > rhs.sizeU_ : lhs.index_ < rhs.index_);
  }

  void get_patch_horizons(std::vector<int> &top_horizon, std::vector<int> &bottom_horizon, std::vector<int> &right_horizon, std::vector<int> &left_horizon)
  {
	  top_horizon.resize(getSizeU0(), 0);
	  bottom_horizon.resize(getSizeU0(), 0);
	  right_horizon.resize(getSizeV0(), 0);
	  left_horizon.resize(getSizeV0(), 0);

	  if (printDetailedInfo)
		  std::cout << "Top Horizon :[";
	  for (int i = 0; i < getSizeU0(); i++) {
		  while (!occupancy_[(getSizeV0() - 1 - top_horizon[i]) * getSizeU0() + i] && (top_horizon[i] < getSizeV0() - 1))
			  top_horizon[i]++;
		  if (printDetailedInfo)
			  std::cout << top_horizon[i] << ",";
	  }
	  if (printDetailedInfo)
		  std::cout << "]" << std::endl;

	  if (printDetailedInfo)
		  std::cout << "Bottom Horizon :[";
	  for (int i = 0; i < getSizeU0(); i++) {
		  while (!occupancy_[bottom_horizon[i] * getSizeU0() + i] && (bottom_horizon[i] < getSizeV0() - 1))
			  bottom_horizon[i]++;
		  if (printDetailedInfo)
			  std::cout << bottom_horizon[i] << ",";
	  }
	  if (printDetailedInfo)
		  std::cout << "]" << std::endl;

	  if (printDetailedInfo)
		  std::cout << "Right Horizon :[";
	  for (int i = 0; i < getSizeV0(); i++) {
		  while (!occupancy_[i * getSizeU0() + getSizeU0() - 1 - right_horizon[i]] && (right_horizon[i] < getSizeU0() - 1))
			  right_horizon[i]++;
		  if (printDetailedInfo)
			  std::cout << right_horizon[i] << ",";
	  }
	  if (printDetailedInfo)
		  std::cout << "]" << std::endl;

	  if (printDetailedInfo)
		  std::cout << "Left Horizon :[";
	  for (int i = 0; i < getSizeV0(); i++) {
		  while (!occupancy_[i * getSizeU0() + left_horizon[i]] && (left_horizon[i] < getSizeU0() - 1))
			  left_horizon[i]++;
		  if (printDetailedInfo)
			  std::cout << left_horizon[i] << ",";
	  }
	  if (printDetailedInfo)
		  std::cout << "]" << std::endl;
  }

  int calculate_wasted_space(std::vector<int> &horizon, std::vector<int> &top_horizon, std::vector<int> &bottom_horizon, std::vector<int> &right_horizon, std::vector<int> &left_horizon)
  {
	  int wasted_space = 0;
	  int wasted_space_external = 0;
	  int wasted_space_internal = 0;
	  int lambda = 100; //--> bias towards the upper part of the canvas

	  if (getPatchOrientation() == PatchOrientation::DEFAULT)
	  {
		  for (int idx = 0; idx < getSizeU0(); idx++) {
			  wasted_space_external += getV0() + bottom_horizon[idx] - horizon[getU0() + idx];
			  //calculating internal wasted space --> because of new block2patch restriction, this area only contains locations for the local patch
			  for (int idx2 = bottom_horizon[idx] + 1; idx2 < getSizeV0() - top_horizon[idx]; idx2++) {
				  if (!occupancy_[idx2 * getSizeU0() + idx])
					  wasted_space_internal++;
			  }
		  }
		  wasted_space = lambda * getV0() + wasted_space_external + wasted_space_internal;
	  }
	  else if (getPatchOrientation() == PatchOrientation::ROT90)
	  {
		  for (int idx = 0; idx < getSizeV0(); idx++) {
			  wasted_space_external += getV0() + left_horizon[getSizeV0() - 1 - idx] - horizon[getU0() + idx];
			  //calculating internal wasted space
			  for (int idx2 = getSizeU0() - 1 - right_horizon[idx]; idx2 >= left_horizon[idx]; idx2--) {
				  if (!occupancy_[idx * getSizeU0() + idx2])
					  wasted_space_internal++;
			  }
		  }
		  wasted_space = lambda * getV0() + wasted_space_external + wasted_space_internal;
	  }
	  else if (getPatchOrientation() == PatchOrientation::ROT180)
	  {
		  for (int idx = 0; idx < getSizeU0(); idx++) {
			  wasted_space_external += getV0() + top_horizon[getSizeU0() - 1 - idx] - horizon[getU0() + idx];
			  //calculating internal wasted space
			  for (int idx2 = getSizeV0() - 1 - top_horizon[idx]; idx2 >= bottom_horizon[idx]; idx2--) {
				  if (!occupancy_[idx2 * getSizeU0() + idx])
					  wasted_space_internal++;
			  }
		  }
		  wasted_space = lambda * getV0() + wasted_space_external + wasted_space_internal;
	  }
	  else if (getPatchOrientation() == PatchOrientation::ROT270)
	  {
		  for (int idx = 0; idx < getSizeV0(); idx++) {
			  wasted_space_external += getV0() + right_horizon[idx] - horizon[getU0() + idx];
			  //calculating internal wasted space
			  for (int idx2 = left_horizon[idx] + 1; idx2 < getSizeU0() - right_horizon[idx]; idx2++) {
				  if (!occupancy_[idx * getSizeU0() + idx2])
					  wasted_space_internal++;
			  }
		  }
		  wasted_space = lambda * getV0() + wasted_space_external + wasted_space_internal;
	  }
	  else if (getPatchOrientation() == PatchOrientation::MIRROR)
	  {
		  for (int idx = 0; idx < getSizeU0(); idx++) {
			  wasted_space_external += getV0() + bottom_horizon[getSizeU0() - 1 - idx] - horizon[getU0() + idx];
			  for (int idx2 = bottom_horizon[idx] + 1; idx2 < getSizeV0() - top_horizon[idx]; idx2++) {
				  if (!occupancy_[idx2 * getSizeU0() + idx])
					  wasted_space_internal++;
			  }
		  }
		  wasted_space = lambda * getV0() + wasted_space_external + wasted_space_internal;
	  }
	  else if (getPatchOrientation() == PatchOrientation::MROT90)
	  {
		  for (int idx = 0; idx < getSizeV0(); idx++) {
			  wasted_space_external += getV0() + right_horizon[getSizeV0() - 1 - idx] - horizon[getU0() + idx];
			  //calculating internal wasted space
			  for (int idx2 = left_horizon[idx] + 1; idx2 < getSizeU0() - right_horizon[idx]; idx2++) {
				  if (!occupancy_[idx * getSizeU0() + idx2])
					  wasted_space_internal++;
			  }
		  }
		  wasted_space = lambda * getV0() + wasted_space_external + wasted_space_internal;
	  }
	  else if (getPatchOrientation() == PatchOrientation::MROT180)
	  {
		  for (int idx = 0; idx < getSizeU0(); idx++) {
			  wasted_space_external += getV0() + top_horizon[idx] - horizon[getU0() + idx];
			  //calculating internal wasted space
			  for (int idx2 = getSizeV0() - 1 - top_horizon[idx]; idx2 >= bottom_horizon[idx]; idx2--) {
				  if (!occupancy_[idx2 * getSizeU0() + idx])
					  wasted_space_internal++;
			  }
		  }
		  wasted_space = lambda * getV0() + wasted_space_external + wasted_space_internal;
	  }
	  else if (getPatchOrientation() == PatchOrientation::MROT270)
	  {
		  for (int idx = 0; idx < getSizeV0(); idx++) {
			  wasted_space_external += getV0() + left_horizon[idx] - horizon[getU0() + idx];
			  for (int idx2 = getSizeU0() - 1 - left_horizon[idx]; idx2 >= right_horizon[idx]; idx2--) {
				  if (!occupancy_[idx * getSizeU0() + idx2])
					  wasted_space_internal++;
			  }
		  }
		  wasted_space = lambda * getV0() + wasted_space_external + wasted_space_internal;
	  }
	  else if (getPatchOrientation() == PatchOrientation::SWAP)
	  {
		  for (int idx = 0; idx < getSizeV0(); idx++) {
			  wasted_space_external += getV0() + left_horizon[idx] - horizon[getU0() + idx];
			  for (int idx2 = getSizeU0() - 1 - left_horizon[idx]; idx2 >= right_horizon[idx]; idx2--) {
				  if (!occupancy_[idx * getSizeU0() + idx2])
					  wasted_space_internal++;
			  }
		  }
		  wasted_space = lambda * getV0() + wasted_space_external + wasted_space_internal;
	  }

	  return wasted_space;
  }

  bool isPatchLocationAboveHorizon(std::vector<int> &horizon, std::vector<int> &top_horizon, std::vector<int> &bottom_horizon, std::vector<int> &right_horizon, std::vector<int> &left_horizon)
  {
	  if (getPatchOrientation() == PatchOrientation::DEFAULT)
	  {
		  for (int idx = 0; idx < getSizeU0(); idx++) {
			  if (getV0() + bottom_horizon[idx] < horizon[getU0() + idx])
				  return false;
		  }
	  }
	  else if (getPatchOrientation() == PatchOrientation::ROT90)
	  {
		  for (int idx = 0; idx < getSizeV0(); idx++) {
			  if (getV0() + left_horizon[getSizeV0() - 1 - idx] < horizon[getU0() + idx])
				  return false;
		  }
	  }
	  else if (getPatchOrientation() == PatchOrientation::ROT180)
	  {
		  for (int idx = 0; idx < getSizeU0(); idx++) {
			  if (getV0() + top_horizon[getSizeU0() - 1 - idx]  < horizon[getU0() + idx])
				  return false;
		  }
	  }
	  else if (getPatchOrientation() == PatchOrientation::ROT270)
	  {
		  for (int idx = 0; idx < getSizeV0(); idx++) {
			  if (getV0() + right_horizon[idx] < horizon[getU0() + idx])
				  return false;
		  }
	  }
	  else if (getPatchOrientation() == PatchOrientation::MIRROR)
	  {
		  for (int idx = 0; idx < getSizeU0(); idx++) {
			  if (getV0() + bottom_horizon[getSizeU0() - 1 - idx] < horizon[getU0() + idx])
				  return false;
		  }
	  }
	  else if (getPatchOrientation() == PatchOrientation::MROT90)
	  {
		  for (int idx = 0; idx < getSizeV0(); idx++) {
			  if (getV0() + right_horizon[getSizeV0() - 1 - idx] < horizon[getU0() + idx])
				  return false;
		  }
	  }
	  else if (getPatchOrientation() == PatchOrientation::MROT180)
	  {
		  for (int idx = 0; idx < getSizeU0(); idx++) {
			  if (getV0() + top_horizon[idx] < horizon[getU0() + idx])
				  return false;
		  }
	  }
	  else if (getPatchOrientation() == PatchOrientation::MROT270)
	  {
		  for (int idx = 0; idx < getSizeV0(); idx++) {
			  if (getV0() + left_horizon[idx] < horizon[getU0() + idx])
				  return false;
		  }
	  }
	  else if (getPatchOrientation() == PatchOrientation::SWAP)
	  {
		  for (int idx = 0; idx < getSizeV0(); idx++) {
			  if (getV0() + left_horizon[idx] < horizon[getU0() + idx])
				  return false;
		  }
	  }
	  return true;
  }

  bool isPatchDimensionSwitched()
  {
	  if ((getPatchOrientation() == PatchOrientation::DEFAULT) || (getPatchOrientation() == PatchOrientation::ROT180) || (getPatchOrientation() == PatchOrientation::MIRROR) || (getPatchOrientation() == PatchOrientation::MROT180))
		  return false;
	  else
		  return true;
  }

  void update_horizon(std::vector<int> &horizon, std::vector<int> &top_horizon, std::vector<int> &bottom_horizon, std::vector<int> &right_horizon, std::vector<int> &left_horizon) {
	  size_t best_u = getU0();
	  size_t best_v = getV0();
	  size_t best_orientation = getPatchOrientation();

	  int newVal;
	  if (best_orientation == PatchOrientation::DEFAULT)
	  {
		  for (int idx = 0; idx < getSizeU0(); idx++)
		  {
			  newVal = best_v + getSizeV0() - 1 - top_horizon[idx];
			  if (newVal > horizon[best_u + idx])
				  horizon[best_u + idx] = newVal;
		  }
	  }
	  else if (best_orientation == PatchOrientation::ROT90)
	  {
		  for (int idx = 0; idx < getSizeV0(); idx++) {
			  newVal = best_v + getSizeU0() - 1 - right_horizon[getSizeV0() - 1 - idx];
			  if (newVal > horizon[best_u + idx])
				  horizon[best_u + idx] = newVal;
		  }
	  }
	  else if (best_orientation == PatchOrientation::ROT180)
	  {
		  for (int idx = 0; idx < getSizeU0(); idx++) {
			  newVal = best_v + getSizeV0() - 1 - bottom_horizon[getSizeU0() - 1 - idx];
			  if (newVal > horizon[best_u + idx])
				  horizon[best_u + idx] = newVal;
		  }
	  }
	  else if (best_orientation == PatchOrientation::ROT270)
	  {
		  for (int idx = 0; idx < getSizeV0(); idx++) {
			  newVal = best_v + getSizeU0() - 1 - left_horizon[idx];
			  if (newVal > horizon[best_u + idx])
				  horizon[best_u + idx] = newVal;
		  }
	  }
	  else if (best_orientation == PatchOrientation::MIRROR)
	  {
		  for (int idx = 0; idx < getSizeU0(); idx++) {
			  newVal = best_v + getSizeV0() - 1 - top_horizon[getSizeU0() - 1 - idx];
			  if (newVal > horizon[best_u + idx])
				  horizon[best_u + idx] = newVal;
		  }
	  }
	  else if (best_orientation == PatchOrientation::MROT90)
	  {
		  for (int idx = 0; idx < getSizeV0(); idx++) {
			  newVal = best_v + getSizeU0() - 1 - left_horizon[getSizeV0() - 1 - idx];
			  if (newVal > horizon[best_u + idx])
				  horizon[best_u + idx] = newVal;
		  }
	  }
	  else if (best_orientation == PatchOrientation::MROT180)
	  {
		  for (int idx = 0; idx < getSizeU0(); idx++) {
			  newVal = best_v + getSizeV0() - 1 - bottom_horizon[idx];
			  if (newVal > horizon[best_u + idx])
				  horizon[best_u + idx] = newVal;
		  }
	  }
	  else if (best_orientation == PatchOrientation::MROT270)
	  {
		  for (int idx = 0; idx < getSizeV0(); idx++) {
			  newVal = best_v + getSizeU0() - 1 - right_horizon[idx];
			  if (newVal > horizon[best_u + idx])
				  horizon[best_u + idx] = newVal;
		  }
	  }
	  else if (best_orientation == PatchOrientation::SWAP)
	  {
		  for (int idx = 0; idx < getSizeV0(); idx++) {
			  newVal = best_v + getSizeU0() - 1 - right_horizon[idx];
			  if (newVal > horizon[best_u + idx])
				  horizon[best_u + idx] = newVal;
		  }
	  }

  }

  GPAPatchData& getPreGPAPatchData()       { return preGPAPatchData_; }
  GPAPatchData  getPreGPAPatchData() const { return preGPAPatchData_; }

  GPAPatchData& getCurGPAPatchData()        { return curGPAPatchData_;  }
  GPAPatchData  getCurGPAPatchData()  const { return curGPAPatchData_;  }

  int patchBlock2CanvasBlockForGPA(const size_t uBlk, const size_t vBlk, size_t canvasStrideBlk, size_t canvasHeightBlk) const {
	  size_t x, y;
	  switch (curGPAPatchData_.patchOrientation) {
	  case PatchOrientation::DEFAULT:
		  x = uBlk + curGPAPatchData_.u0;
		  y = vBlk + curGPAPatchData_.v0;
		  break;
	  case   PatchOrientation::ROT90:
		  x = (curGPAPatchData_.sizeV0 - 1 - vBlk) + curGPAPatchData_.u0;
		  y = uBlk + curGPAPatchData_.v0;
		  break;
	  case   PatchOrientation::ROT180:
		  x = (curGPAPatchData_.sizeU0 - 1 - uBlk) + curGPAPatchData_.u0;
		  y = (curGPAPatchData_.sizeV0 - 1 - vBlk) + curGPAPatchData_.v0;
		  break;
	  case   PatchOrientation::ROT270:
		  x = vBlk + curGPAPatchData_.u0;
		  y = (curGPAPatchData_.sizeU0 - 1 - uBlk) + curGPAPatchData_.v0;
		  break;
	  case  PatchOrientation::MIRROR:
		  x = (curGPAPatchData_.sizeU0 - 1 - uBlk) + curGPAPatchData_.u0;
		  y = vBlk + curGPAPatchData_.v0;
		  break;
	  case  PatchOrientation::MROT90:
		  x = (curGPAPatchData_.sizeV0 - 1 - vBlk) + curGPAPatchData_.u0;
		  y = (curGPAPatchData_.sizeU0 - 1 - uBlk) + curGPAPatchData_.v0;
		  break;
	  case   PatchOrientation::MROT180:
		  x = uBlk + curGPAPatchData_.u0;
		  y = (curGPAPatchData_.sizeV0 - 1 - vBlk) + curGPAPatchData_.v0;
		  break;
	  case   PatchOrientation::MROT270:
		  x = vBlk + curGPAPatchData_.u0;
		  y = uBlk + curGPAPatchData_.v0;
		  break;
	  case   PatchOrientation::SWAP://swapAxis
		  x = vBlk + curGPAPatchData_.u0;
		  y = uBlk + curGPAPatchData_.v0;
		  break;
	  default: return -1; break;
	  }
	  //checking the results are within canvas boundary (missing y check)
	  if (x < 0) return -1;
	  if (y < 0) return -1;
	  if (x >= canvasStrideBlk) return -1;
	  if (y >= canvasHeightBlk) return -1;
	  return (x + canvasStrideBlk * y);
  }

  bool checkFitPatchCanvasForGPA(std::vector<bool> canvas, size_t canvasStrideBlk, size_t canvasHeightBlk, int safeguard = 0) {
	  for (size_t v0 = 0; v0 < curGPAPatchData_.sizeV0; ++v0) {
		  for (size_t u0 = 0; u0 < curGPAPatchData_.sizeU0; ++u0) {
			  for (int deltaY = -safeguard; deltaY < safeguard + 1; deltaY++) {
				  for (int deltaX = -safeguard; deltaX < safeguard + 1; deltaX++) {
					  int pos = patchBlock2CanvasBlockForGPA(u0 + deltaX, v0 + deltaY, canvasStrideBlk, canvasHeightBlk);
					  if (pos < 0) {
						  return false;
					  }
					  else if (canvas[pos]) {
						  return false;
					  }
				  }
			  }
		  }
	  }

	  return true;
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
  size_t occupancyResolution_;     // occupancy map resolution
  size_t projectionMode_;         // 0: related to the min depth value; 1: related to the max value
  size_t frameProjectionMode_;    // 0: fixed projection mode, where all patches use projection mode 0; 1: variable projection mode, where each patch can use a different projection mode. 
  size_t levelOfDetail_;           // level of detail, i.e., patch sampling resolution
  PCCMetadata patchLevelMetadata_;

  size_t normalAxis_;              // x
  size_t tangentAxis_;             // y
  size_t bitangentAxis_;           // z
  std::vector<int16_t> depth_[2];  // depth
  std::vector<bool> occupancy_;    // occupancy map

  size_t viewId_;                  //viewId in [0,1,2,3,4,5]
  int32_t bestMatchIdx_;            //index of matched patch from pre-frame patch.

  std::vector<int16_t> depthEnhancedDeltaD_;
  std::vector<int64_t> depth0PCidx_;       // for Surface separation
  size_t patchOrientation_;        // patch orientation in canvas atlas


  GPAPatchData curGPAPatchData_;
  GPAPatchData preGPAPatchData_;
  bool	isGlobalPatch_;
};

struct PCCMissedPointsPatch {
  size_t sizeU;
  size_t sizeV;
  size_t u0;
  size_t v0;
  size_t sizeV0;
  size_t sizeU0;
  size_t numMissedPts;
  size_t occupancyResolution;
  std::vector<bool> occupancy;
  std::vector<uint16_t> x;
  std::vector<uint16_t> y;
  std::vector<uint16_t> z;

  std::vector<uint16_t> r;
  std::vector<uint16_t> g;
  std::vector<uint16_t> b;
  size_t numEddSavedPoints;
  size_t MPnumber;
  size_t MPnumbercolor;
	
  //GPA.
  size_t pre_v0;
  size_t temp_v0;

  void resize(const size_t size) {
    x.resize(size);
    y.resize(size);
    z.resize(size);
  }
  void resize(const size_t size, const uint16_t val) {
    x.resize(size, val);
    y.resize(size, val);
    z.resize(size, val);
  }

  const size_t size() { return x.size(); }

  const size_t sizeofcolor() { return r.size();}
  void         setMPnumber(size_t numofMPs){MPnumber=numofMPs;}
  void         setMPnumbercolor(size_t numofMPs){MPnumbercolor=numofMPs;}
  const size_t getMPnumber() {return MPnumber;}
  const size_t getMPnumbercolor() {return MPnumbercolor;}
  void resizecolor(const size_t size) {
    r.resize(size);
    g.resize(size);
    b.resize(size);
  }
  void resizecolor(const size_t size, const uint16_t val) {
    r.resize(size, val);
    g.resize(size, val);
    b.resize(size, val);
  }
};
}

#endif /* PCCPatch_h */
