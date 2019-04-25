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
#ifndef PCCEncoder_h
#define PCCEncoder_h

#include "PCCCommon.h"
#include "PCCEncoderParameters.h"
#include "PCCCodec.h"
#include "PCCMetadata.h"

#include <map>

namespace pcc {

class PCCPointSet3; 
class PCCGroupOfFrames;
class PCCBitstream;
class PCCContext;
class PCCFrameContext;
class GeometryFrameParameterSet;
class GeometryPatchParameterSet;
class PointLocalReconstruction;

template <typename T, size_t N>
class PCCVideo;
typedef pcc::PCCVideo<uint8_t,  3> PCCVideoTexture;
typedef pcc::PCCVideo<uint16_t, 3> PCCVideoGeometry;
typedef pcc::PCCVideo<uint8_t,  3> PCCVideoOccupancyMap;
template <typename T, size_t N>
class PCCImage;
typedef pcc::PCCImage<uint8_t,  3> PCCImageTexture;
typedef pcc::PCCImage<uint16_t, 3> PCCImageGeometry;
typedef pcc::PCCImage<uint8_t,  3> PCCImageOccupancyMap;
struct PCCPatchSegmenter3Parameters;
class PCCPatch;
struct PCCBistreamPosition;
  
  struct SparseMatrixCoefficient {
    int32_t _index;
    double _value;
  };
  
  struct SparseMatrixRow {
    SparseMatrixRow() {
      _coefficientCount = 0;
    }
    void addCofficient(const int32_t j, const double value) {
      assert(_coefficientCount < 5);
      auto & coeff= _coefficients[_coefficientCount++];
      coeff._index = j;
      coeff._value = value;
    }
    int32_t _coefficientCount;
    SparseMatrixCoefficient _coefficients[5];
  };
  
  struct SparseMatrix {
    const SparseMatrixRow & getRow(const int32_t i) const {
      assert(i < _rows.size());
      return _rows[i];
    }
    SparseMatrixRow & getRow(const int32_t i) {
      assert(i < _rows.size());
      return _rows[i];
    }
    void resize(const int32_t rowCount) {
      _rows.resize(rowCount);
    }
    std::vector<SparseMatrixRow> _rows;
  };
  struct PaddingContext{
    std::vector<int32_t> _mapping;
    std::vector<int32_t> _invMapping;
    SparseMatrix _A;
    std::vector<double> _b[3];
    std::vector<double> _x, _p, _r, _q;
  };


typedef std::map<size_t, PCCPatch> unionPatch;                      // unionPatch ------ [TrackIndex, UnionPatch];
typedef std::pair<size_t, size_t>  GlobalPatch;                     // GlobalPatch ----- [FrameIndex, PatchIndex];
typedef std::map<size_t, std::vector<GlobalPatch> > GlobalPatches;  // GlobalPatches --- [TrackIndex, <GlobalPatch>];
typedef std::pair<size_t, size_t>  SubContext;                      // SubContext ------ [start, end);

#define BAD_HEIGHT_THRESHOLD    1.10
#define BAD_CONDITION_THRESHOLD 2


class PCCEncoder : public PCCCodec {
public:
  PCCEncoder();
  ~PCCEncoder();
  void setParameters( PCCEncoderParameters params );

  int encode( const PCCGroupOfFrames& sources, PCCContext &context,
              PCCBitstream &bitstream, PCCGroupOfFrames& reconstructs );


  // adaptor methods (JR: move for test )
  void createPatchFrameDataStructure( PCCContext&   context );

  void createPatchFrameDataStructure( PCCContext&      context,
                                      PCCFrameContext& frame,
                                      PCCFrameContext& preFrame,
                                      size_t           frameIndex );

private:
  int encode( const PCCGroupOfFrames& sources, PCCContext &context, PCCGroupOfFrames& reconstructs );

  bool generateOccupancyMapVideo( const PCCGroupOfFrames& sources, PCCContext& context );
  bool generateOccupancyMapVideo(const size_t imageWidth, const size_t imageHeight,
                                 std::vector<uint32_t> &occupancyMap,
                                 PCCImageOccupancyMap &videoFrameOccupancyMap);

  template <typename T>
  T limit(T x, T minVal, T maxVal);
  void preFilterOccupancyMap(PCCImageOccupancyMap &image, size_t kwidth, size_t kheight);
  bool modifyOccupancyMap(const PCCGroupOfFrames& sources, PCCContext& context);
  bool modifyOccupancyMap(const size_t imageWidth, const size_t imageHeight,
    std::vector<uint32_t> &occupancyMap, PCCImageOccupancyMap &videoFrameOccupancyMap, std::ofstream& ofile);

  bool generateGeometryVideo( const PCCGroupOfFrames& sources, PCCContext &context );
  bool resizeGeometryVideo( PCCContext &context );
  bool dilateGeometryVideo( PCCContext &context );

  bool generateTextureVideo( const PCCGroupOfFrames& sources,
                             PCCGroupOfFrames& reconstruct,
                             PCCContext& context,
                             const PCCEncoderParameters params);


  void generateMissedPointsGeometryVideo(PCCContext& context, PCCGroupOfFrames& reconstructs);
  void generateMissedPointsTextureVideo(PCCContext& context, PCCGroupOfFrames& reconstructs );
  
  void generateMPsGeometryImage(PCCContext& context, PCCFrameContext& frame, PCCImageGeometry &image);
  void generateMPsTextureImage(PCCContext& context,
                               PCCFrameContext& frame,
                               PCCImageTexture &image, size_t shift,
                               const PCCPointSet3& reconstruct);
  
  template <typename T>
  void dilate( PCCFrameContext &frame, PCCImage<T, 3> &image, const PCCImage<T, 3> *reference = nullptr );
  //Push-pull background filling
  template <typename T>
  int mean4w(T p1, unsigned char w1, T p2, unsigned char w2, T p3, unsigned char w3, T p4, unsigned char w4);
  template <typename T>
  void pushPullMip(const PCCImage<T, 3> &image, PCCImage<T, 3> &mip,
                   const std::vector<uint32_t>& occupancyMap,
                   std::vector<uint32_t> &mipOccupancyMap);
  template <typename T>
  void pushPullFill( PCCImage<T, 3> &image, const PCCImage<T, 3> &mip, 
                     const std::vector<uint32_t>& occupancyMap, int numIters );
  template <typename T>
  void dilateSmoothedPushPull(PCCFrameContext& frame, PCCImage<T, 3> &image);
  template <typename T>
  void dilateHarmonicBackgroundFill(PCCFrameContext& frame, PCCImage<T, 3> &image);
  template <typename T>
  void CreateCoarseLayer(PCCImage<T, 3> &image, PCCImage<T, 3> &mip, std::vector<uint32_t>& occupancyMap, std::vector<uint32_t> &mipOccupancyMap);
  template <typename T>
  void regionFill(PCCImage<T, 3> &image, std::vector<uint32_t>& occupancyMap, PCCImage<T, 3> &imageLowRes);
  void pack( PCCFrameContext& frame, int safeguard = 0  );
  void packFlexible(PCCFrameContext& frame, int safeguard = 0);
  void packTetris(PCCFrameContext & frame, int safeguard = 0);
  void packMissedPointsPatch( PCCFrameContext& frame, const std::vector<bool> &occupancyMap, size_t &width, 
                              size_t &height, size_t occupancySizeU, size_t occupancySizeV, size_t maxOccupancyRow);
  void spatialConsistencyPack(PCCFrameContext& frame, PCCFrameContext &prevFrame, int safeguard = 0 );
  void spatialConsistencyPackFlexible(PCCFrameContext& frame, PCCFrameContext &prevFrame, int safeguard = 0);
  void spatialConsistencyPackTetris(PCCFrameContext & frame, PCCFrameContext & prevFrame, int safeguard = 0);
  void generateOccupancyMap( PCCFrameContext& frameContext );
  void modifyOccupancyMap(PCCFrameContext& frame, const PCCImageGeometry &imageRef, const PCCImageGeometry &image);
  void refineOccupancyMap( PCCFrameContext &frame );
  void printMap(std::vector<bool> img, const size_t sizeU, const size_t sizeV);
  void printMapTetris(std::vector<bool> img, const size_t sizeU, const size_t sizeV, std::vector<int> horizon);
  void generateIntraImage( PCCFrameContext& frameContext, const size_t depthIndex, PCCImageGeometry &image);
  bool predictGeometryFrame( PCCFrameContext& frameContext, const PCCImageGeometry &reference, PCCImageGeometry &image);
  void generateMissedPointsPatch( const PCCPointSet3& source,
                                  PCCFrameContext& frameContext,
                                  bool useEnhancedDeltaDepthCode);

  void sortMissedPointsPatch(PCCFrameContext& frameContext);
  bool generateGeometryVideo( const PCCPointSet3& source, PCCFrameContext& frameContext,
                             const PCCPatchSegmenter3Parameters segmenterParams,
                             PCCVideoGeometry &videoGeometry, PCCFrameContext &prevFrame,
                             size_t frameIndex, float& distanceSrcRec );

  bool generateTextureVideo( const PCCPointSet3& reconstruct, PCCFrameContext& frameContext,
                             PCCVideoTexture &video, const size_t frameCount );

  void generateIntraEnhancedDeltaDepthImage( PCCFrameContext& frame,
                                             const PCCImageGeometry &imageRef,
                                             PCCImageGeometry &image);


  void geometryGroupDilation( PCCContext& context );

  void create3DMotionEstimationFiles( PCCContext& context, std::string path );
  void remove3DMotionEstimationFiles( std::string path );

  void reconstuctionOptimization( PCCContext& context,
                                  const GeneratePointCloudParameters params );
  void reconstuctionOptimization( PCCFrameContext &frame,
                                  const PCCVideoGeometry &video,
                                  const PCCVideoGeometry &videoD1,
                                  const GeneratePointCloudParameters params);
  void presmoothPointCloudColor(PCCPointSet3 &reconstruct, const PCCEncoderParameters params);

  void calculate_weight_normal(const PCCPointSet3& source, PCCFrameContext& frameContext);

  // perform data-adaptive GPA method;
  void   performDataAdaptiveGPAMethod(PCCContext& context);
  // start a subContext;
  void   initializeSubContext(PCCFrameContext& frameContext,
                              SubContext& subContext,
                              GlobalPatches& globalPatchTracks,
                              unionPatch& unionPatch,
                              size_t frameIndex);
  // generate globalPatches;
  void   generateGlobalPatches(PCCContext& context,
                               size_t frameIndex,
                               GlobalPatches& globalPatchTracks,
                               size_t preIndex);
  // patch unions generation and packing; return the height of the unionsPackingImage;
  size_t unionPatchGenerationAndPacking(const GlobalPatches& globalPatchTracks,
                                        PCCContext& context,
                                        unionPatch& unionPatch,
                                        size_t refFrameIdx,
                                        int safeguard = 0,
                                        bool useRefFrame = false);
  // update patch information;
  void   updateGPAPatchInformation(PCCContext& context, SubContext& subContext, unionPatch& unionPatch);
  // perform data-adaptive gpa packing;
  void   performGPAPacking(const SubContext& subContext,
                           unionPatch& unionPatch,
                           PCCContext& context,
                           bool& badGPAPacking,
                           size_t unionsHeight,
                           int safeguard = 0,
                           bool useRefFrame = false);

  void clearCurrentGPAPatchDataInfor(PCCContext& context, SubContext& subContext);

  void packingFirstFrame(PCCContext& context, size_t frameIndex, bool packingStrategy, int safeguard, bool hasRefFrame);
  void updatePatchInformation(PCCContext& context, SubContext& subContext);
  void packingWithoutRefForFirstFrameNoglobalPatch( PCCPatch& patch,
                                                    size_t i,
                                                    size_t icount,
                                                    size_t& occupancySizeU,
                                                    size_t& occupancySizeV,
                                                    const size_t safeguard,
                                                    std::vector<bool> & occupancyMap,
                                                    size_t& heightGPA,
                                                    size_t& widthGPA,
                                                    size_t maxOccupancyRow);

  void packingWithRefForFirstFrameNoglobalPatch( PCCPatch& patch,
                                                 const std::vector<PCCPatch> prePatches,
                                                 size_t startFrameIndex,
                                                 size_t i,
                                                 size_t icount,
                                                 size_t& occupancySizeU,
                                                 size_t& occupancySizeV,
                                                 const size_t safeguard,
                                                 std::vector<bool> & occupancyMap,
                                                 size_t& heightGPA,
                                                 size_t& widthGPA,
                                                 size_t maxOccupancyRow);

  void setGeometryFrameParameterSet( PCCMetadata& metadata, GeometryFrameParameterSet& gfps );
  void setGeometryPatchParameterSet( PCCMetadata& metadata, GeometryPatchParameterSet& gpps );

  void setPointLocalReconstruction( PCCFrameContext& frame, const PCCPatch& patch, PointLocalReconstruction& plr, size_t occupancyPackingBlockSize );


  PCCEncoderParameters params_;
};

}; //~namespace

#endif /* PCCEncoder_h */
