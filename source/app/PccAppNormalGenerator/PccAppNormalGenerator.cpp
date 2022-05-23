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
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#include "PCCCommon.h"
#include "PCCMath.h"
#include "PCCKdTree.h"
#include "PCCGroupOfFrames.h"
#include "PCCNormalsGenerator.h"
#include <program_options_lite.h>
#if defined( ENABLE_TBB )
#include <tbb/tbb.h>
#endif

using namespace std;
using namespace pcc;

//---------------------------------------------------------------------------
// :: Command line / config parsing helpers

namespace pcc {
static std::istream& operator>>( std::istream& in, PCCNormalsGeneratorOrientation& val ) {
  unsigned int tmp;
  in >> tmp;
  val = PCCNormalsGeneratorOrientation( tmp );
  return in;
}
}  // namespace pcc

//---------------------------------------------------------------------------
// :: Command line / config parsing

bool parseParameters( int                             argc,
                      char*                           argv[],
                      std::string&                    srcPlyPath,
                      std::string&                    dstPlyPath,
                      size_t&                         startFrame,
                      size_t&                         numFrames,
                      size_t&                         numThread,
                      PCCNormalsGenerator3Parameters& normalParams ) {
  namespace po    = df::program_options_lite;
  bool print_help = false;

  // The definition of the program/config options, along with default values.
  //
  // NB: when updating the following tables:
  //      (a) please keep to 80-columns for easier reading at a glance,
  //      (b) do not vertically align values -- it breaks quickly
  //
  // clang-format off
  po::Options opts;
  std::string configurationFolder;
  std::string uncompressedDataFolder;
  std::string uncompressedDataPath;
  size_t tmp = 0;
  opts.addOptions()
    ( "help", print_help, false,"This help text" )
    ( "c,config", po::parseConfigFile, "Configuration file name" )
    ( "configurationFolder",
      configurationFolder,
      configurationFolder, 
      "Folder where the configuration files are stored,use for cfg relative paths." )
    ( "uncompressedDataFolder",
      uncompressedDataFolder,
      uncompressedDataFolder,
      "Folder where the uncompress input data are stored, use for cfg relative paths." )
    ( "uncompressedDataPath",
      uncompressedDataPath,
      uncompressedDataPath,
      "Input pointcloud to encode. Multi-frame sequences may be represented by %04i" )
    //parameters not used, but in the configuration files, so we add them here
    ( "geometry3dCoordinatesBitdepth",tmp,tmp,"UNUSED" )
    ( "geometryNominal2dBitdepth",tmp,tmp,"UNUSED" )
    ( "groupOfFramesSize",tmp,tmp,"UNUSED" )
    ( "iterationCountRefineSegmentation",tmp,tmp,"UNUSED" )
    ( "minNormSumOfInvDist4MPSelection",tmp,tmp,"UNUSED" )
    ( "partialAdditionalProjectionPlane",tmp,tmp,"UNUSED" )
    ( "maxPatchSize",tmp,tmp,"UNUSED" )
    ( "roiBoundingBoxMinX",tmp,tmp,"UNUSED" )
    ( "roiBoundingBoxMaxX",tmp,tmp,"UNUSED" )
    ( "roiBoundingBoxMinY",tmp,tmp,"UNUSED" )
    ( "roiBoundingBoxMaxY",tmp,tmp,"UNUSED" )
    ( "roiBoundingBoxMinZ",tmp,tmp,"UNUSED" )
    ( "roiBoundingBoxMaxZ",tmp,tmp,"UNUSED" )
    ( "numTilesHor",tmp,tmp,"UNUSED" )
    ( "tileHeightToWidthRatio",tmp,tmp,"UNUSED" )
    ( "numCutsAlong1stLongestAxis",tmp,tmp,"UNUSED" )
    ( "numCutsAlong2ndLongestAxis",tmp,tmp,"UNUSED" )
    ( "numCutsAlong3rdLongestAxis",tmp,tmp,"UNUSED" )
    //added for queen sequence
    ( "voxelDimensionRefineSegmentation",tmp,tmp,"UNUSED" )
    ( "minimumImageHeight",tmp,tmp,"UNUSED" )
    ( "flagColorPreSmoothing",tmp,tmp,"UNUSED" )
    ( "surfaceSeparation",tmp,tmp,"UNUSED" )
    ( "enhancedProjectionPlane",tmp,tmp,"UNUSED" )
    ( "skipAvgIfIdenticalSourcePointPresentBwd",tmp,tmp,"UNUSED" )
    // i/o
    ( "srcPlyPath",
      srcPlyPath,
      srcPlyPath,
      "Input pointcloud to encode. Multi-frame sequences may be represented by %04i" )
    ( "dstPlyPath",
      dstPlyPath,
      dstPlyPath,
      "Output decoded pointcloud. Multi-frame sequences may be represented by %04i" )
    // sequence configuration
    ( "startFrameNumber",
      startFrame,
      startFrame,
      "First frame number in sequence to encode/decode" )
    ( "frameCount",
      numFrames,
      numFrames,
      "Number of frames to encode" )
    // etc
    ( "nbThread",
      numThread,
      numThread,
      "Number of thread used for parallel processing" )
    // Normal generation parameters
    ( "viewPointX",
      normalParams.viewPoint_[0],
      
      normalParams.viewPoint_[0],"View Point X" )
    ( "viewPointY",
      normalParams.viewPoint_[1],
      normalParams.viewPoint_[1],
      "View Point Y" )
    ( "viewPointZ",
      normalParams.viewPoint_[2],
      normalParams.viewPoint_[2],
      "View Point Z" )
    ( "radiusNormalSmoothing",normalParams.radiusNormalSmoothing_,
      normalParams.radiusNormalSmoothing_,
      "Radius Normal Smoothing (default:MAX_VAL)" )
    ( "radiusNormalEstimation",
      normalParams.radiusNormalEstimation_,
      normalParams.radiusNormalEstimation_,
      "Radius Normal Estimation (default:MAX_VAL)" )
    ( "radiusNormalOrientation",
      normalParams.radiusNormalOrientation_,
      normalParams.radiusNormalOrientation_,
      "Radius Normal Orientation (default:MAX_VAL)" )
    ( "weightNormalSmoothing",
      normalParams.weightNormalSmoothing_,
      normalParams.weightNormalSmoothing_,
      "Weight Normal Smoothing (default:MAX_VAL)" )
    ( "numberOfNearestNeighborsInNormalSmoothing",
      normalParams.numberOfNearestNeighborsInNormalSmoothing_,
      normalParams.numberOfNearestNeighborsInNormalSmoothing_,
      "Number Of Nearest Neighbors In Normal Smoothing (default:16)" )
    ( "numberOfNearestNeighborsInNormalEstimation",
      normalParams.numberOfNearestNeighborsInNormalEstimation_,
      normalParams.numberOfNearestNeighborsInNormalEstimation_,
      "Number Of Nearest Neighbors In Normal Estimation (default:16)" )
    ( "numberOfNearestNeighborsInNormalOrientation",
      normalParams.numberOfNearestNeighborsInNormalOrientation_,
      normalParams.numberOfNearestNeighborsInNormalOrientation_,
      "Number Of Nearest Neighbors In Normal Orientation (default:16)" )
    ( "numberOfIterationsInNormalSmoothing",
      normalParams.numberOfIterationsInNormalSmoothing_,
      normalParams.numberOfIterationsInNormalSmoothing_,
      "Number Of Iterations In Normal Smoothing (default:0)" )
    ( "orientationStrategy",
      normalParams.orientationStrategy_,
      normalParams.orientationStrategy_,
      "(0)NONE, (1)SPANNING TREE, (2)VIEWPOINT, (3)CUBEMAP PROJECTION" )
    ( "storeEigenvalues",normalParams.storeEigenvalues_,
      normalParams.storeEigenvalues_,
      "Store Eigenvalues (0)false/(1)true" )
    ( "storeNumberOfNearestNeighborsInNormalEstimation",
      normalParams.storeNumberOfNearestNeighborsInNormalEstimation_,
      normalParams.storeNumberOfNearestNeighborsInNormalEstimation_,
      "Store Number Of Nearest Neighbors In Normal Estimation (0)false/(1)true" )
    ( "storeCentroids",
      normalParams.storeCentroids_,
      normalParams.storeCentroids_,
      "Store Centroids (0)false/(1)true" )
    ;
  opts.addOptions();
  // clang-format on
  po::setDefaults( opts );
  po::ErrorReporter        err;
  const list<const char*>& argv_unhandled = po::scanArgv( opts, argc, (const char**)argv, err );
  for ( const auto arg : argv_unhandled ) { printf( "Unhandled argument ignored: %s \n", arg ); }

  if ( argc == 1 || print_help ) {
    po::doHelp( std::cout, opts, 78 );
    return false;
  }

  if ( srcPlyPath.empty() ) { srcPlyPath = uncompressedDataFolder + uncompressedDataPath; }
  if ( dstPlyPath.empty() ) {
    dstPlyPath = uncompressedDataFolder + uncompressedDataPath.substr( 0, uncompressedDataPath.size() - 4 ) + "_n.ply";
  }

  printf( "parseParameters : \n" );
  printf( "  srcPlyPath   = %s \n", srcPlyPath.c_str() );
  printf( "  dstPlyPath   = %s \n", dstPlyPath.c_str() );
  printf( "  startFrame   = %zu \n", startFrame );
  printf( "  frameCount   = %zu \n", numFrames );
  printf( "  numThread    = %zu \n", numThread );
  printf( "  normalParameters: \n" );
  printf( "    viewPoint                                       = [%f,%f,%f] \n", normalParams.viewPoint_[0],
          normalParams.viewPoint_[1], normalParams.viewPoint_[2] );
  printf( "    radiusNormalSmoothing                           = %f \n", normalParams.radiusNormalSmoothing_ );
  printf( "    radiusNormalEstimation                          = %f \n", normalParams.radiusNormalEstimation_ );
  printf( "    radiusNormalOrientation                         = %f \n", normalParams.radiusNormalOrientation_ );
  printf( "    weightNormalSmoothing                           = %f \n", normalParams.weightNormalSmoothing_ );
  printf( "    numberOfNearestNeighborsInNormalSmoothing       = %zu \n",
          normalParams.numberOfNearestNeighborsInNormalSmoothing_ );
  printf( "    numberOfNearestNeighborsInNormalEstimation      = %zu \n",
          normalParams.numberOfNearestNeighborsInNormalEstimation_ );
  printf( "    numberOfNearestNeighborsInNormalOrientation     = %zu \n",
          normalParams.numberOfNearestNeighborsInNormalOrientation_ );
  printf( "    numberOfIterationsInNormalSmoothing             = %zu \n",
          normalParams.numberOfIterationsInNormalSmoothing_ );
  printf( "    orientationStrategy                             = %u \n", normalParams.orientationStrategy_ );
  printf( "    storeEigenvalues                                = %u \n", normalParams.storeEigenvalues_ );
  printf( "    storeNumberOfNearestNeighborsInNormalEstimation = %u \n",
          normalParams.storeNumberOfNearestNeighborsInNormalEstimation_ );
  printf( "    storeCentroids                                  = %u \n", normalParams.storeCentroids_ );

  // report the current configuration (only in the absence of errors so
  // that errors/warnings are more obvious and in the same place).
  if ( err.is_errored ) { return false; }

  return true;
}

int generateNormal( const std::string&                    uncompressedDataPath,
                    const std::string&                    reconstructedDataPath,
                    const size_t                          startFrameNumber,
                    const size_t                          frameCount,
                    const size_t                          nbThread,
                    const PCCNormalsGenerator3Parameters& normalParams ) {
  PCCGroupOfFrames sources;
  // reading the input ply
  std::cout << std::endl << "============= INPUT: " << uncompressedDataPath << " ============= " << std::endl;
  if ( !sources.load( uncompressedDataPath, startFrameNumber, startFrameNumber + frameCount, COLOR_TRANSFORM_NONE ) ) {
    return -1;
  }
  // calculating the normal for each frame
  for ( int frIdx = startFrameNumber; frIdx < startFrameNumber + frameCount; frIdx++ ) {
    std::cout << std::endl << "============= FRAME " << frIdx << " ============= " << std::endl;
    std::cout << "  Computing normals for original point cloud... ";
    PCCPointSet3&        geometry = sources.getFrames()[frIdx];
    PCCKdTree            kdtree( geometry );
    PCCNNResult          result;
    PCCNormalsGenerator3 normalsGen;
    normalsGen.compute( geometry, kdtree, normalParams, nbThread );
    geometry.addNormals();
    for ( int ptIdx = 0; ptIdx < geometry.getPointCount(); ptIdx++ ) {
      geometry.setNormal( ptIdx, normalsGen.getNormal( ptIdx ) );
    }
    std::cout << "[done]" << std::endl;
  }
  // saving the normal
  std::cout << std::endl << "============= OUTPUT: " << reconstructedDataPath << " ============= " << std::endl;
  size_t startFrame = startFrameNumber;
  if ( !reconstructedDataPath.empty() ) { sources.write( reconstructedDataPath, startFrame ); }
  sources.clear();
  return 0;
}

int main( int argc, char* argv[] ) {
  std::cout << "PccAppNormalGenerator v" << TMC2_VERSION_MAJOR << "." << TMC2_VERSION_MINOR << std::endl << std::endl;
  std::string                    uncompressedDataPath;
  std::string                    reconstructedDataPath;
  size_t                         startFrameNumber;
  size_t                         frameCount;
  size_t                         nbThread     = 0;
  PCCNormalsGenerator3Parameters normalParams = {PCCVector3D( 0.0 ),
                                                 ( std::numeric_limits<double>::max )(),
                                                 ( std::numeric_limits<double>::max )(),
                                                 ( std::numeric_limits<double>::max )(),
                                                 ( std::numeric_limits<double>::max )(),
                                                 16,
                                                 16,
                                                 16,
                                                 0,
                                                 PCC_NORMALS_GENERATOR_ORIENTATION_SPANNING_TREE,
                                                 false,
                                                 false,
                                                 false};  // default values
  if ( !parseParameters( argc, argv, uncompressedDataPath, reconstructedDataPath, startFrameNumber, frameCount,
                         nbThread, normalParams ) ) {
    return -1;
  }
#if defined( ENABLE_TBB )
  if ( nbThread > 0 ) { tbb::task_scheduler_init init( static_cast<int>( nbThread ) ); }
#endif
  int ret = generateNormal( uncompressedDataPath, reconstructedDataPath, startFrameNumber, frameCount, nbThread,
                            normalParams );
  return ret;
}