/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2026, ISO/IEC
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

#include "PCCAdaptation.h"
#include "PCCSymEigen.h"

#include <algorithm>
#include <cmath>
#include <set>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace pcc {

//============================================================================
// PCCAdaptationParameters - Config I/O
//============================================================================

bool PCCAdaptationParameters::writeConfig( const std::string &filename ) const {
  std::ofstream ofs( filename );
  if ( !ofs.is_open() ) { return false; }

  ofs << std::fixed << std::setprecision( 6 );
  ofs << "{\n";

  // Bits section (matching Python format)
  ofs << "  \"bits\": {\n";
  ofs << "    \"pos\": " << positionBitDepth_ << ",\n";
  ofs << "    \"sh\": " << shBitDepth_ << ",\n";
  ofs << "    \"opacity\": " << opacityBitDepth_ << ",\n";
  ofs << "    \"scale\": " << scaleBitDepth_ << ",\n";
  ofs << "    \"rot\": " << rotationBitDepth_ << "\n";
  ofs << "  },\n";

  // Limits section (matching Python format)
  ofs << "  \"limits\": {\n";

  // Position limits
  ofs << "    \"pos\": {\n";
  ofs << "      \"min\": [" << computedPositionMin_[0] << ", " << computedPositionMin_[1] << ", "
      << computedPositionMin_[2] << "],\n";
  ofs << "      \"max\": [" << computedPositionMax_[0] << ", " << computedPositionMax_[1] << ", "
      << computedPositionMax_[2] << "]\n";
  ofs << "    },\n";

  // SH limits (48 values: 16 coefficients x 3 channels, flattened)
  ofs << "    \"sh\": {\n";
  ofs << "      \"min\": [";
  for ( size_t i = 0; i < 16; ++i ) {
    for ( size_t c = 0; c < 3; ++c ) {
      ofs << computedShMin_[i][c];
      if ( i < 15 || c < 2 ) ofs << ", ";
    }
  }
  ofs << "],\n";
  ofs << "      \"max\": [";
  for ( size_t i = 0; i < 16; ++i ) {
    for ( size_t c = 0; c < 3; ++c ) {
      ofs << computedShMax_[i][c];
      if ( i < 15 || c < 2 ) ofs << ", ";
    }
  }
  ofs << "]\n";
  ofs << "    },\n";

  // Opacity limits
  ofs << "    \"opacity\": {\n";
  ofs << "      \"min\": " << computedOpacityMin_ << ",\n";
  ofs << "      \"max\": " << computedOpacityMax_ << "\n";
  ofs << "    },\n";

  // Scale limits
  ofs << "    \"scale\": {\n";
  ofs << "      \"min\": [" << computedScaleMin_[0] << ", " << computedScaleMin_[1] << ", " << computedScaleMin_[2]
      << "],\n";
  ofs << "      \"max\": [" << computedScaleMax_[0] << ", " << computedScaleMax_[1] << ", " << computedScaleMax_[2]
      << "]\n";
  ofs << "    },\n";

  // Rotation limits
  ofs << "    \"rot\": {\n";
  ofs << "      \"min\": [" << computedRotationMin_[0] << ", " << computedRotationMin_[1] << ", "
      << computedRotationMin_[2] << ", " << computedRotationMin_[3] << "],\n";
  ofs << "      \"max\": [" << computedRotationMax_[0] << ", " << computedRotationMax_[1] << ", "
      << computedRotationMax_[2] << ", " << computedRotationMax_[3] << "]\n";
  ofs << "    }";

  // Scale non-uniform quantization
  if ( enableScaleNonUniformQuantization_ ) {
    ofs << ",\n";
    ofs << "    \"scale_quant_regions\": {\n";
    ofs << "      \"num_regions_minus_1\": [";
    for ( size_t i = 0; i < scaleNumRegionsMinus1_.size(); ++i ) {
      ofs << scaleNumRegionsMinus1_[i] << ( i < scaleNumRegionsMinus1_.size() - 1 ? ", " : "" );
    }
    ofs << "],\n";
    ofs << "      \"transition_points\": [";
    for ( size_t i = 0; i < scaleTransitionPoints_.size(); ++i ) {
      ofs << scaleTransitionPoints_[i] << ( i < scaleTransitionPoints_.size() - 1 ? ", " : "" );
    }
    ofs << "],\n";
    ofs << "      \"step_sizes\": [";
    for ( size_t i = 0; i < scaleStepSizes_.size(); ++i ) {
      ofs << scaleStepSizes_[i] << ( i < scaleStepSizes_.size() - 1 ? ", " : "" );
    }
    ofs << "],\n";
    ofs << "      \"num_bits_for_transition_point\": [";
    for ( size_t i = 0; i < scaleNumBitsForTransitionPoint_.size(); ++i ) {
      ofs << scaleNumBitsForTransitionPoint_[i] << ( i < scaleNumBitsForTransitionPoint_.size() - 1 ? ", " : "" );
    }
    ofs << "]\n";
    ofs << "    }";
  }

  if ( enableShNonUniformQuantization_ ) {
    ofs << ",\n";
    ofs << "    \"sh_quant_regions\": {\n";
    ofs << "      \"num_regions_minus_1\": [";
    for ( size_t i = 0; i < shNumRegionsMinus1_.size(); ++i ) {
      ofs << shNumRegionsMinus1_[i] << ( i < shNumRegionsMinus1_.size() - 1 ? ", " : "" );
    }
    ofs << "],\n";
    ofs << "      \"transition_points\": [";
    for ( size_t i = 0; i < shTransitionPoints_.size(); ++i ) {
      ofs << shTransitionPoints_[i] << ( i < shTransitionPoints_.size() - 1 ? ", " : "" );
    }
    ofs << "],\n";
    ofs << "      \"step_sizes\": [";
    for ( size_t i = 0; i < shStepSizes_.size(); ++i ) {
      ofs << shStepSizes_[i] << ( i < shStepSizes_.size() - 1 ? ", " : "" );
    }
    ofs << "],\n";
    ofs << "      \"num_bits_for_transition_point\": [";
    for ( size_t i = 0; i < shNumBitsForTransitionPoint_.size(); ++i ) {
      ofs << shNumBitsForTransitionPoint_[i] << ( i < shNumBitsForTransitionPoint_.size() - 1 ? ", " : "" );
    }
    ofs << "]\n";
    ofs << "    }";
  }

  if ( gsdTransform2dPresentFlag_ ) {
    ofs << ",\n";
    ofs << "    \"shTransforms\": {\n";
    ofs << "      \"trTypeCol\": [";
    for ( size_t i = 0; i < shTrTypeCol_.size(); ++i ) {
      ofs << shTrTypeCol_[i] << ( i < shTrTypeCol_.size() - 1 ? ", " : "" );
    }
    ofs << "],\n";
    ofs << "      \"trGrpRow\": [";
    for ( size_t i = 0; i < shTrGrpRow_.size(); ++i ) {
      ofs << "[";
      for ( size_t j = 0; j < shTrGrpRow_[i].size(); ++j ) {
        ofs << shTrGrpRow_[i][j] << ( j < shTrGrpRow_[i].size() - 1 ? ", " : "" );
      }
      ofs << "]" << ( i < shTrGrpRow_.size() - 1 ? ", " : "" );
    }
    ofs << "],\n";
    ofs << "      \"trTypeRow\": [";
    for ( size_t i = 0; i < shTrTypeRow_.size(); ++i ) {
      ofs << shTrTypeRow_[i] << ( i < shTrTypeRow_.size() - 1 ? ", " : "" );
    }
    ofs << "],\n";
    ofs << "      \"kltFlag\": [";
    for ( size_t i = 0; i < shKltFlag_.size(); ++i ) {
      ofs << shKltFlag_[i] << ( i < shKltFlag_.size() - 1 ? ", " : "" );
    }
    ofs << "]\n";
    ofs << "    }\n";
  } else {
    ofs << "\n";
  }
  ofs << "  }";

  if ( enableAdaptiveShQuantization_ ) {
    ofs << ",\n";
    ofs << "  \"adaptive_bitshifts\": {\n";
    ofs << "    \"sh\": [";
    for ( size_t i = 0; i < adaptiveShBitshifts_.size(); ++i ) {
      ofs << adaptiveShBitshifts_[i] << ( i < adaptiveShBitshifts_.size() - 1 ? ", " : "" );
    }
    ofs << "]\n";
    ofs << "  }\n";
  } else {
    ofs << "\n";
  }

  ofs << "}\n";
  ofs.close();
  return true;
}

bool PCCAdaptationParameters::readConfig( const std::string &filename ) {
  std::ifstream ifs( filename );
  if ( !ifs.is_open() ) { return false; }

  std::string content( ( std::istreambuf_iterator<char>( ifs ) ), std::istreambuf_iterator<char>() );
  ifs.close();

  // Helper to find JSON value after key
  auto findValue = [&content]( const std::string &path, size_t startPos = 0 ) -> size_t {
    size_t pos = content.find( "\"" + path + "\"", startPos );
    if ( pos == std::string::npos ) return std::string::npos;
    pos = content.find( ":", pos );
    if ( pos == std::string::npos ) return std::string::npos;
    return pos + 1;
  };

  // Helper to parse integer
  auto parseInt = [&content]( size_t pos ) -> int {
    if ( pos == std::string::npos ) return 0;
    // Skip whitespace
    while ( pos < content.size() &&
            ( content[pos] == ' ' || content[pos] == '\n' || content[pos] == '\t' || content[pos] == '\r' ) )
      ++pos;
    return std::stoi( content.substr( pos ) );
  };

  // Helper to parse double
  auto parseFloat = [&content]( size_t pos ) -> double {
    if ( pos == std::string::npos ) return 0.0;
    // Skip whitespace
    while ( pos < content.size() &&
            ( content[pos] == ' ' || content[pos] == '\n' || content[pos] == '\t' || content[pos] == '\r' ) )
      ++pos;
    return std::stod( content.substr( pos ) );
  };

  // Helper to parse array of doubles
  auto parseFloatArray = [&content]( size_t pos, double *output, size_t count ) {
    if ( pos == std::string::npos ) return;
    // Find opening bracket
    pos = content.find( "[", pos );
    if ( pos == std::string::npos ) return;
    ++pos;

    for ( size_t i = 0; i < count; ++i ) {
      // Skip whitespace
      while ( pos < content.size() &&
              ( content[pos] == ' ' || content[pos] == '\n' || content[pos] == '\t' || content[pos] == '\r' ) )
        ++pos;
      output[i] = std::stod( content.substr( pos ) );
      // Find next comma or closing bracket
      pos = content.find_first_of( ",]", pos );
      if ( pos == std::string::npos || content[pos] == ']' ) break;
      ++pos;
    }
  };

  // Helper to parse array of integers into vector
  auto parseIntVector = [&content]( size_t pos, std::vector<int> &output ) {
    if ( pos == std::string::npos ) return;
    pos = content.find( "[", pos );
    if ( pos == std::string::npos ) return;
    ++pos;
    output.clear();
    while ( pos < content.size() ) {
      while ( pos < content.size() &&
              ( content[pos] == ' ' || content[pos] == '\n' || content[pos] == '\t' || content[pos] == '\r' ) )
        ++pos;
      if ( content[pos] == ']' ) break;
      output.push_back( std::stoi( content.substr( pos ) ) );
      pos = content.find_first_of( ",]", pos );
      if ( pos == std::string::npos || content[pos] == ']' ) break;
      ++pos;
    }
  };

  // Helper to parse array of doubles into vector
  auto parseFloatVector = [&content]( size_t pos, std::vector<double> &output ) {
    if ( pos == std::string::npos ) return;
    pos = content.find( "[", pos );
    if ( pos == std::string::npos ) return;
    ++pos;
    output.clear();
    while ( pos < content.size() ) {
      while ( pos < content.size() &&
              ( content[pos] == ' ' || content[pos] == '\n' || content[pos] == '\t' || content[pos] == '\r' ) )
        ++pos;
      if ( content[pos] == ']' ) break;
      output.push_back( std::stod( content.substr( pos ) ) );
      pos = content.find_first_of( ",]", pos );
      if ( pos == std::string::npos || content[pos] == ']' ) break;
      ++pos;
    }
  };

  // Parse bits section (nested under "bits")
  size_t bitsPos = content.find( "\"bits\"" );
  if ( bitsPos != std::string::npos ) {
    positionBitDepth_ = parseInt( findValue( "pos", bitsPos ) );
    shBitDepth_       = parseInt( findValue( "sh", bitsPos ) );
    opacityBitDepth_  = parseInt( findValue( "opacity", bitsPos ) );
    scaleBitDepth_    = parseInt( findValue( "scale", bitsPos ) );
    rotationBitDepth_ = parseInt( findValue( "rot", bitsPos ) );
  }

  // Parse limits section
  size_t limitsPos = content.find( "\"limits\"" );
  if ( limitsPos != std::string::npos ) {
    // Position limits
    size_t posMinPos = content.find( "\"pos\"", limitsPos );
    if ( posMinPos != std::string::npos ) {
      posMinPos = content.find( "\"min\"", posMinPos );
      parseFloatArray( posMinPos, computedPositionMin_.data(), 3 );
      size_t posMaxPos = content.find( "\"max\"", posMinPos );
      parseFloatArray( posMaxPos, computedPositionMax_.data(), 3 );
    }

    // SH limits (48 values flattened)
    size_t shMinPos = content.find( "\"sh\"", limitsPos );
    if ( shMinPos != std::string::npos ) {
      shMinPos = content.find( "\"min\"", shMinPos );
      double shMinFlat[48];
      parseFloatArray( shMinPos, shMinFlat, 48 );
      // Reshape from flat array to [16][3]
      for ( size_t i = 0; i < 16; ++i ) {
        for ( size_t c = 0; c < 3; ++c ) { computedShMin_[i][c] = shMinFlat[i * 3 + c]; }
      }

      size_t shMaxPos = content.find( "\"max\"", shMinPos );
      double shMaxFlat[48];
      parseFloatArray( shMaxPos, shMaxFlat, 48 );
      // Reshape from flat array to [16][3]
      for ( size_t i = 0; i < 16; ++i ) {
        for ( size_t c = 0; c < 3; ++c ) { computedShMax_[i][c] = shMaxFlat[i * 3 + c]; }
      }
    }

    // Opacity limits
    size_t opacityMinPos = content.find( "\"opacity\"", limitsPos );
    if ( opacityMinPos != std::string::npos ) {
      opacityMinPos        = content.find( "\"min\"", opacityMinPos );
      computedOpacityMin_  = parseFloat( opacityMinPos );
      size_t opacityMaxPos = content.find( "\"max\"", opacityMinPos );
      computedOpacityMax_  = parseFloat( opacityMaxPos );
    }

    // Scale limits
    size_t scaleMinPos = content.find( "\"scale\"", limitsPos );
    if ( scaleMinPos != std::string::npos ) {
      scaleMinPos = content.find( "\"min\"", scaleMinPos );
      parseFloatArray( scaleMinPos, computedScaleMin_.data(), 3 );
      size_t scaleMaxPos = content.find( "\"max\"", scaleMinPos );
      parseFloatArray( scaleMaxPos, computedScaleMax_.data(), 3 );
    }

    // Rotation limits
    size_t rotMinPos = content.find( "\"rot\"", limitsPos );
    if ( rotMinPos != std::string::npos ) {
      rotMinPos = content.find( "\"min\"", rotMinPos );
      parseFloatArray( rotMinPos, computedRotationMin_.data(), 4 );
      size_t rotMaxPos = content.find( "\"max\"", rotMinPos );
      parseFloatArray( rotMaxPos, computedRotationMax_.data(), 4 );
    }

    // Scale non-uniform quantization
    size_t scaleQuantRegionsPos = content.find( "\"scale_quant_regions\"", limitsPos );
    if ( scaleQuantRegionsPos != std::string::npos ) {
      enableScaleNonUniformQuantization_ = true;
      parseIntVector( findValue( "num_regions_minus_1", scaleQuantRegionsPos ), scaleNumRegionsMinus1_ );
      parseIntVector( findValue( "transition_points", scaleQuantRegionsPos ), scaleTransitionPoints_ );
      parseFloatVector( findValue( "step_sizes", scaleQuantRegionsPos ), scaleStepSizes_ );
      parseIntVector( findValue( "num_bits_for_transition_point", scaleQuantRegionsPos ),
                      scaleNumBitsForTransitionPoint_ );
    }

    // SH non-uniform quantization
    size_t shQuantRegionsPos = content.find( "\"sh_quant_regions\"", limitsPos );
    if ( shQuantRegionsPos != std::string::npos ) {
      enableShNonUniformQuantization_ = true;
      parseIntVector( findValue( "num_regions_minus_1", shQuantRegionsPos ), shNumRegionsMinus1_ );
      parseIntVector( findValue( "transition_points", shQuantRegionsPos ), shTransitionPoints_ );
      parseFloatVector( findValue( "step_sizes", shQuantRegionsPos ), shStepSizes_ );
      parseIntVector( findValue( "num_bits_for_transition_point", shQuantRegionsPos ), shNumBitsForTransitionPoint_ );
    }

    // SH 2D transform
    size_t shTransformsPos = content.find( "\"shTransforms\"", limitsPos );
    if ( shTransformsPos != std::string::npos ) {
      gsdTransform2dPresentFlag_ = true;
      parseIntVector( findValue( "trTypeCol", shTransformsPos ), shTrTypeCol_ );
      parseIntVector( findValue( "trTypeRow", shTransformsPos ), shTrTypeRow_ );
      parseIntVector( findValue( "kltFlag", shTransformsPos ), shKltFlag_ );

      // Parse trGrpRow (vector of vectors)
      size_t grpPos = content.find( "\"trGrpRow\"", shTransformsPos );
      if ( grpPos != std::string::npos ) {
        grpPos = content.find( "[", grpPos );
        if ( grpPos != std::string::npos ) {
          shTrGrpRow_.clear();
          size_t p = grpPos + 1;
          while ( p < content.size() ) {
            while ( p < content.size() &&
                    ( content[p] == ' ' || content[p] == '\n' || content[p] == '\t' || content[p] == '\r' ) )
              ++p;
            if ( content[p] == '[' ) {
              std::vector<int> group;
              p++;
              while ( p < content.size() && content[p] != ']' ) {
                while ( p < content.size() && ( content[p] == ' ' || content[p] == '\n' || content[p] == '\t' ||
                                                content[p] == '\r' || content[p] == ',' ) )
                  ++p;
                if ( isdigit( content[p] ) || content[p] == '-' ) {
                  group.push_back( std::stoi( content.substr( p ) ) );
                  while ( p < content.size() && ( isdigit( content[p] ) || content[p] == '-' ) ) ++p;
                } else if ( content[p] == ']' ) {
                  break;
                } else {
                  p++;
                }
              }
              shTrGrpRow_.push_back( group );
              if ( p < content.size() ) p++;
            } else if ( content[p] == ']' ) {
              break;
            } else {
              p++;
            }
          }
        }
      }
    }
  }

  // Parse adaptive quantization bitshifts
  size_t adaptiveQuantizationPos = content.find( "\"adaptive_bitshifts\"" );
  if ( adaptiveQuantizationPos != std::string::npos ) {
    enableAdaptiveShQuantization_ = true;
    parseIntVector( findValue( "sh", adaptiveQuantizationPos ), adaptiveShBitshifts_ );
  }

  return true;
}

//============================================================================
// PCCAdaptation - Constructor/Destructor
//============================================================================

PCCAdaptation::PCCAdaptation() {}

PCCAdaptation::~PCCAdaptation() {}

//============================================================================
// PCCAdaptation - Forward Declarations
//============================================================================

namespace {

template <typename PointSetT>
size_t getAdaptiveBitDepth( const uint32_t maxBitDepth, const PointSetT &input, size_t pointIndex );

}  // namespace

//============================================================================
// PCCAdaptation - Main Adaptation Functions
//============================================================================

bool PCCAdaptation::adapt( const PCCPointSetGS     &inputPointCloud,
                           PCCPointSet3            &outputPointCloud,
                           PCCAdaptationParameters &params ) {
  const size_t pointCount = inputPointCloud.getPointCount();
  if ( pointCount == 0 ) {
    std::cerr << "PCCAdaptation::adapt - Empty input point cloud" << std::endl;
    return false;
  }

  // Initialize output point cloud
  outputPointCloud.resize( pointCount );

  // Set attribute flags based on input
  if ( inputPointCloud.hasColors() ) { outputPointCloud.addColors(); }
  if ( inputPointCloud.hasRotations() ) { outputPointCloud.addRotations(); }
  if ( inputPointCloud.hasScales() ) { outputPointCloud.addScales(); }
  if ( inputPointCloud.hasOpacities() ) { outputPointCloud.addOpacities(); }
  if ( inputPointCloud.hasSphericalHarmonics() ) { outputPointCloud.addSphericalHarmonics(); }

  // Apply position quantization
  if ( params.enablePositionQuantization_ ) {
    if ( !quantizePositions( inputPointCloud, outputPointCloud, params ) ) {
      std::cerr << "PCCAdaptation::adapt - Position quantization failed" << std::endl;
      return false;
    }
  }

  // Apply canonical orientation
  if ( !applyCanonicalOrientation( const_cast<PCCPointSetGS &>( inputPointCloud ) ) ) {
    std::cerr << "PCCAdaptation::adapt - Canonical orientation optimization failed" << std::endl;
    return false;
  }

  // Apply rotation quantization
  if ( params.enableRotationQuantization_ && inputPointCloud.hasRotations() ) {
    if ( !quantizeRotations( inputPointCloud, outputPointCloud, params ) ) {
      std::cerr << "PCCAdaptation::adapt - Rotation quantization failed" << std::endl;
      return false;
    }
  }

  // Apply scale quantization
  if ( params.enableScaleQuantization_ && inputPointCloud.hasScales() ) {
    extractScaleQuantRegions( inputPointCloud, params );
    if ( !quantizeScales( inputPointCloud, outputPointCloud, params ) ) {
      std::cerr << "PCCAdaptation::adapt - Scale quantization failed" << std::endl;
      return false;
    }
  }

  // Apply opacity quantization
  if ( params.enableOpacityQuantization_ && inputPointCloud.hasOpacities() ) {
    if ( params.encodeLinearOpacity_ ) logitToLinearOpacity( const_cast<PCCPointSetGS &>( inputPointCloud ) );
    extractOpacityQuantRegions( inputPointCloud, params );  // m76122
    if ( !quantizeOpacities( inputPointCloud, outputPointCloud, params ) ) {
      std::cerr << "PCCAdaptation::adapt - Opacity quantization failed" << std::endl;
      return false;
    }
  }

  // Apply spherical harmonics quantization
  if ( params.enableShQuantization_ && inputPointCloud.hasSphericalHarmonics() ) {
    if ( params.shMappingMode_ != 2 )
      reorderSphericalHarmonicsAC( const_cast<PCCPointSetGS &>( inputPointCloud ), true );
    initializeShTransformParams( params );
    if ( params.shConvertToYUV_ || params.gsdTransform2dPresentFlag_ ) {
      sphericalHarmonicsTransform( const_cast<PCCPointSetGS &>( inputPointCloud ), params );
    }

    if ( params.shMappingMode_ == 2 ) {
      const size_t pointCount  = inputPointCloud.getPointCount();
      const size_t shCoefCount = inputPointCloud.getSphericalHarmonicsCount();
      const size_t numACCoeff  = shCoefCount - 1;  // 15
      const auto  &shData      = inputPointCloud.getSphericalHarmonics();

      std::array<double, 45> mean;
      mean.fill( 0.0 );
      for ( size_t p = 0; p < pointCount; ++p ) {
        for ( size_t l = 1; l < shCoefCount; ++l ) {
          for ( int ch = 0; ch < 3; ++ch ) { mean[ch * numACCoeff + ( l - 1 )] += shData[p][l][ch]; }
        }
      }
      for ( int i = 0; i < 45; ++i ) { mean[i] /= pointCount; }

      std::array<double, 45> stddev;
      stddev.fill( 0.0 );
      for ( size_t p = 0; p < pointCount; ++p ) {
        for ( size_t l = 1; l < shCoefCount; ++l ) {
          for ( int ch = 0; ch < 3; ++ch ) {
            int    idx = ch * numACCoeff + ( l - 1 );
            double d   = shData[p][l][ch] - mean[idx];
            stddev[idx] += d * d;
          }
        }
      }
      for ( int i = 0; i < 45; ++i ) { stddev[i] = std::sqrt( stddev[i] / pointCount ); }

      std::array<int, 45> indices;
      for ( int i = 0; i < 45; ++i ) { indices[i] = i; }
      std::sort( indices.begin(), indices.end(), [&stddev]( int a, int b ) { return stddev[a] > stddev[b]; } );

      params.shMappingTable_.fill( 0 );
      for ( int j = 0; j < 45; ++j ) { params.shMappingTable_[j] = indices[j]; }
    }

    extractSHQuantRegions( inputPointCloud, params );
    if ( !quantizeSphericalHarmonics( inputPointCloud, outputPointCloud, params ) ) {
      std::cerr << "PCCAdaptation::adapt - SH quantization failed" << std::endl;
      return false;
    }
    if ( params.shMappingMode_ == 1 )
      reorderSphericalHarmonicsAC( const_cast<PCCPointSet3 &>( outputPointCloud ), false );
    else if ( params.shMappingMode_ == 2 )
      reorderSphericalHarmonicsAdaptive( const_cast<PCCPointSet3 &>( outputPointCloud ), params, true );

    if ( params.enableAdaptiveShQuantization_ ) {
      outputPointCloud.getQuantizationGroupIds()            = inputPointCloud.getQuantizationGroupIds();
      outputPointCloud.getQuantizationGroupBitDepthDeltas() = inputPointCloud.getQuantizationGroupBitDepthDeltas();
    }
  }

  // Copy colors directly (already uint8_t)
  if ( inputPointCloud.hasColors() ) {
    for ( size_t i = 0; i < pointCount; ++i ) { outputPointCloud.setColor( i, inputPointCloud.getColors()[i] ); }
  }

  return true;
}

bool PCCAdaptation::inverseAdapt( const PCCPointSet3            &inputPointCloud,
                                  PCCPointSetGS                 &outputPointCloud,
                                  const PCCAdaptationParameters &params ) {
  const size_t pointCount = inputPointCloud.getPointCount();
  if ( pointCount == 0 ) {
    std::cerr << "PCCAdaptation::inverseAdapt - Empty input point cloud" << std::endl;
    return false;
  }

  // Initialize output point cloud
  outputPointCloud.resize( pointCount );

  // Set attribute flags based on input
  if ( inputPointCloud.hasColors() ) { outputPointCloud.addColors(); }
  if ( inputPointCloud.hasRotations() ) { outputPointCloud.addRotations(); }
  if ( inputPointCloud.hasScales() ) { outputPointCloud.addScales(); }
  if ( inputPointCloud.hasOpacities() ) { outputPointCloud.addOpacities(); }
  if ( inputPointCloud.hasSphericalHarmonics() ) { outputPointCloud.addSphericalHarmonics(); }

  // Apply position dequantization
  if ( params.enablePositionQuantization_ ) {
    if ( !dequantizePositions( inputPointCloud, outputPointCloud, params ) ) { return false; }
  }

  // Apply rotation dequantization
  if ( params.enableRotationQuantization_ && inputPointCloud.hasRotations() ) {
    if ( !dequantizeRotations( inputPointCloud, outputPointCloud, params ) ) { return false; }
  }

  // Apply scale dequantization
  if ( params.enableScaleQuantization_ && inputPointCloud.hasScales() ) {
    if ( !dequantizeScales( inputPointCloud, outputPointCloud, params ) ) { return false; }
  }

  // Apply opacity dequantization
  if ( params.enableOpacityQuantization_ && inputPointCloud.hasOpacities() ) {
    if ( !dequantizeOpacities( inputPointCloud, outputPointCloud, params ) ) { return false; }
    if ( params.encodeLinearOpacity_ ) linearToLogitOpacity( const_cast<PCCPointSetGS &>( outputPointCloud ) );
  }

  // Apply spherical harmonics dequantization
  if ( params.enableShQuantization_ && inputPointCloud.hasSphericalHarmonics() ) {
    if ( params.shMappingMode_ == 1 )
      reorderSphericalHarmonicsAC( const_cast<PCCPointSet3 &>( inputPointCloud ), true );
    else if ( params.shMappingMode_ == 2 )
      reorderSphericalHarmonicsAdaptive( const_cast<PCCPointSet3 &>( inputPointCloud ), params, false );
    if ( !dequantizeSphericalHarmonics( inputPointCloud, outputPointCloud, params ) ) { return false; }
    if ( params.shConvertToYUV_ || params.gsdTransform2dPresentFlag_ ) {
      sphericalHarmonicsInvTransform( outputPointCloud, params );
    }
    if ( params.shMappingMode_ != 2 ) reorderSphericalHarmonicsAC( outputPointCloud, false );

    if ( params.enableAdaptiveShQuantization_ ) {
      outputPointCloud.getQuantizationGroupIds()            = inputPointCloud.getQuantizationGroupIds();
      outputPointCloud.getQuantizationGroupBitDepthDeltas() = inputPointCloud.getQuantizationGroupBitDepthDeltas();
    }
  }

  // Copy colors directly
  if ( inputPointCloud.hasColors() ) {
    for ( size_t i = 0; i < pointCount; ++i ) { outputPointCloud.getColors()[i] = inputPointCloud.getColor( i ); }
  }

  return true;
}

//============================================================================
// PCCAdaptation - Position Quantization
//============================================================================

bool PCCAdaptation::quantizePositions( const PCCPointSetGS     &input,
                                       PCCPointSet3            &output,
                                       PCCAdaptationParameters &params ) {
  const size_t pointCount = input.getPointCount();
  if ( pointCount == 0 ) return false;

  // Compute bounding box if auto-compute is enabled
  if ( params.autoComputePositionLimits_ ) {
    computeBoundingBox( input, params.computedPositionMin_, params.computedPositionMax_ );

    // Use bounding box min as start, and add fixed size (256) as in Python script, only if the range is smaller than
    // 256 to avoid excessive quantization error
    double range = 0.0;
    for ( int k = 0; k < 3; ++k ) {
      if ( range < ( params.computedPositionMax_[k] - params.computedPositionMin_[k] ) ) {
        range = params.computedPositionMax_[k] - params.computedPositionMin_[k];
      }
    }
    if ( range >= 256.0 ) {
      std::cerr << "PCCAdaptation::quantizePositions - Warning: Computed position range is " << range
                << ", which is larger than 256. Using actual bounding box limits for quantization." << std::endl;
      for ( int k = 0; k < 3; ++k ) { params.computedPositionMax_[k] = params.computedPositionMin_[k] + range; }
    } else {
      std::cerr << "PCCAdaptation::quantizePositions - Warning: Computed position range is " << range
                << ", which is smaller than 256. Using fixed size 256 for quantization." << std::endl;
      for ( int k = 0; k < 3; ++k ) { params.computedPositionMax_[k] = params.computedPositionMin_[k] + 256.0; }
    }
  } else {
    // Use provided limits
    params.computedPositionMin_ = params.positionLimitMin_;
    params.computedPositionMax_ = params.positionLimitMax_;
  }

  const uint32_t bitDepth = params.positionBitDepth_;

  // Quantize each position
  for ( size_t i = 0; i < pointCount; ++i ) {
    const PCCPointGS &pos  = input[i];
    PCCPoint3D       &qPos = output[i];

    for ( int k = 0; k < 3; ++k ) {
      double normalized = normalizeToUnit( pos[k], params.computedPositionMin_[k], params.computedPositionMax_[k] );
      qPos[k]           = static_cast<int32_t>( quantizeValue( normalized, bitDepth ) );
    }
  }

  return true;
}

bool PCCAdaptation::dequantizePositions( const PCCPointSet3            &input,
                                         PCCPointSetGS                 &output,
                                         const PCCAdaptationParameters &params ) {
  const size_t   pointCount = input.getPointCount();
  const uint32_t bitDepth   = params.positionBitDepth_;

  for ( size_t i = 0; i < pointCount; ++i ) {
    const PCCPoint3D &qPos = input[i];
    PCCPointGS       &pos  = output[i];

    for ( int k = 0; k < 3; ++k ) {
      double normalized = dequantizeValue( static_cast<uint32_t>( qPos[k] ), bitDepth );
      pos[k] = denormalizeFromUnit( normalized, params.computedPositionMin_[k], params.computedPositionMax_[k] );
    }
  }

  return true;
}

//============================================================================
// PCCAdaptation - Rotation Quantization
//============================================================================

bool PCCAdaptation::quantizeRotations( const PCCPointSetGS     &input,
                                       PCCPointSet3            &output,
                                       PCCAdaptationParameters &params ) {
  const size_t pointCount = input.getPointCount();
  if ( pointCount == 0 || !input.hasRotations() ) return false;

  // Store limits used
  params.computedRotationMin_ = params.rotationLimitMin_;
  params.computedRotationMax_ = params.rotationLimitMax_;

  const uint32_t bitDepth = params.rotationBitDepth_;
  for ( size_t i = 0; i < pointCount; ++i ) {
    const GSRotation &rot = input.getRotations()[i];

    // 3-component tangent space representation [x/w, y/w, z/w]
    float w = rot.w, x = rot.x, y = rot.y, z = rot.z;
    if ( w < 1e-6f ) w = 1e-6f;

    std::vector<PCCAttribute16bit> qRot( 2 );
    double                         v[3] = { (double)x / w, (double)y / w, (double)z / w };

    for ( int k = 0; k < 3; ++k ) {
      double normalized =
          normalizeToUnit( v[k], params.computedRotationMin_[k + 1], params.computedRotationMax_[k + 1] );
      qRot[0][k] = static_cast<uint16_t>( quantizeValue( normalized, bitDepth ) );
      qRot[1][k] = 0;
    }
    qRot[1][0] = static_cast<uint16_t>( quantizeValue(
        normalizeToUnit( 0, params.computedRotationMin_[0], params.computedRotationMax_[0] ), bitDepth ) );
    output.setRotation( i, qRot );
  }

  return true;
}

bool PCCAdaptation::dequantizeRotations( const PCCPointSet3            &input,
                                         PCCPointSetGS                 &output,
                                         const PCCAdaptationParameters &params ) {
  const size_t   pointCount = input.getPointCount();
  const uint32_t bitDepth   = params.rotationBitDepth_;
  for ( size_t i = 0; i < pointCount; ++i ) {
    const auto &qRot = input.getRotations()[i];

    // 3-component tangent space representation [x/w, y/w, z/w]
    double rx = denormalizeFromUnit( dequantizeValue( qRot[0][0], bitDepth ), params.computedRotationMin_[0],
                                     params.computedRotationMax_[0] );
    double ry = denormalizeFromUnit( dequantizeValue( qRot[0][1], bitDepth ), params.computedRotationMin_[1],
                                     params.computedRotationMax_[1] );
    double rz = denormalizeFromUnit( dequantizeValue( qRot[0][2], bitDepth ), params.computedRotationMin_[2],
                                     params.computedRotationMax_[2] );

    GSRotation rot( 1.0f, static_cast<float>( rx ), static_cast<float>( ry ), static_cast<float>( rz ) );
    // Re-canonicalize after dequantization
    if ( params.enableRotationCanonicalization_ ) { canonicalizeQuaternion( rot.w, rot.x, rot.y, rot.z ); }
    output.getRotations()[i] = rot;
  }
  return true;
}

bool PCCAdaptation::applyCanonicalOrientation( PCCPointSetGS &pointCloud ) {
  if ( !pointCloud.hasRotations() || !pointCloud.hasScales() ) return false;
  const size_t pointCount = pointCloud.getPointCount();
  auto        &rotations  = pointCloud.getRotations();
  auto        &scales     = pointCloud.getScales();

  const int    perms[6][3] = { { 0, 1, 2 }, { 0, 2, 1 }, { 1, 0, 2 }, { 1, 2, 0 }, { 2, 0, 1 }, { 2, 1, 0 } };
  const double flips[4][3] = { { 1, 1, 1 }, { 1, -1, -1 }, { -1, 1, -1 }, { -1, -1, 1 } };

  for ( size_t i = 0; i < pointCount; ++i ) {
    GSRotation rot = rotations[i];
    canonicalizeQuaternion( rot.w, rot.x, rot.y, rot.z );

    GSRotation bestQ    = rot;
    GSScale    bestS    = scales[i];
    double     bestWAbs = -1.0;

    PCCMatrix3D R_orig;
    quatToRotMat( rot, R_orig );

    for ( int p = 0; p < 6; ++p ) {
      GSScale currS( scales[i][perms[p][0]], scales[i][perms[p][1]], scales[i][perms[p][2]] );

      PCCMatrix3D R_perm;
      for ( int r = 0; r < 3; ++r ) {
        for ( int c = 0; c < 3; ++c ) { R_perm[r][c] = R_orig[r][perms[p][c]]; }
      }

      double det = R_perm[0][0] * ( R_perm[1][1] * R_perm[2][2] - R_perm[1][2] * R_perm[2][1] ) -
                   R_perm[0][1] * ( R_perm[1][0] * R_perm[2][2] - R_perm[1][2] * R_perm[2][0] ) +
                   R_perm[0][2] * ( R_perm[1][0] * R_perm[2][1] - R_perm[1][1] * R_perm[2][0] );

      for ( int f = 0; f < 4; ++f ) {
        PCCMatrix3D R_cand = R_perm;
        double      f0     = flips[f][0];
        double      f1     = flips[f][1];
        double      f2     = flips[f][2] * ( det > 0 ? 1.0 : -1.0 );
        for ( int r = 0; r < 3; ++r ) {
          R_cand[r][0] *= f0;
          R_cand[r][1] *= f1;
          R_cand[r][2] *= f2;
        }

        GSRotation q_cand;
        rotMatToQuat( R_cand, q_cand );
        double absW = std::abs( q_cand.w );
        if ( absW > bestWAbs ) {
          bestWAbs = absW;
          bestQ    = q_cand;
          bestS    = currS;
        }
      }
    }
    if ( bestQ.w < 0 ) {
      bestQ.w = -bestQ.w;
      bestQ.x = -bestQ.x;
      bestQ.y = -bestQ.y;
      bestQ.z = -bestQ.z;
    }
    bestQ.normalize();
    rotations[i] = bestQ;
    scales[i]    = bestS;
  }
  return true;
}

// ── R-D optimal power search (port of Python find_optimal_power) ─────────────

static void metricsForPower( double                     power,
                             uint32_t                   numBins,
                             double                     Q,
                             double                     valRange,
                             double                     valMin,
                             size_t                     N,
                             double                     sumWeights,
                             double                     log10MaxVal,
                             const std::vector<float>  &xSorted,
                             const std::vector<double> &prefixW1,
                             const std::vector<double> &prefixW2,
                             const std::vector<double> &prefixW3,
                             double                    &outPsnr,
                             double                    &outEntropy ) {
  const double        invP = 1.0 / power;
  std::vector<size_t> splitIdx( numBins - 1 );
  for ( uint32_t k = 1; k < numBins; ++k ) {
    double edgeVal  = std::pow( ( k - 0.5 ) / Q, invP );
    splitIdx[k - 1] = static_cast<size_t>(
        std::lower_bound( xSorted.begin(), xSorted.end(), static_cast<float>( edgeVal ) ) - xSorted.begin() );
  }
  std::vector<size_t> starts( numBins ), ends( numBins );
  starts[0]         = 0;
  ends[numBins - 1] = N;
  for ( uint32_t k = 1; k < numBins; ++k ) {
    starts[k]   = splitIdx[k - 1];
    ends[k - 1] = splitIdx[k - 1];
  }
  double entropy = 0.0;
  for ( uint32_t k = 0; k < numBins; ++k ) {
    size_t count = ends[k] - starts[k];
    if ( count > 0 ) {
      double prob = static_cast<double>( count ) / static_cast<double>( N );
      entropy -= prob * std::log2( std::max( prob, 1e-12 ) );
    }
  }
  double totalWeightedError = 0.0;
  for ( uint32_t q = 0; q < numBins; ++q ) {
    size_t l = starts[q], r = ends[q];
    if ( l >= r ) continue;
    double s1    = prefixW1[r] - prefixW1[l];
    double s2    = prefixW2[r] - prefixW2[l];
    double s3    = prefixW3[r] - prefixW3[l];
    double qNorm = std::max( std::min( static_cast<double>( q ) / Q, 1.0 ), 1e-7 );
    double rec   = std::exp( std::pow( qNorm, invP ) * valRange + valMin );
    totalWeightedError += s3 - 2.0 * rec * s2 + rec * rec * s1;
  }
  double weightedMse = totalWeightedError / sumWeights;
  outPsnr            = 20.0 * ( log10MaxVal - 0.5 * std::log10( std::max( weightedMse, 1e-12 ) ) );
  outEntropy         = entropy;
}

static double findBestPowerFromGrid( const std::vector<float>  &powerGrid,
                                     double                     baselineRate,
                                     double                     baselinePsnr,
                                     uint32_t                   numBins,
                                     double                     Q,
                                     double                     valRange,
                                     double                     valMin,
                                     size_t                     N,
                                     double                     sumWeights,
                                     double                     log10MaxVal,
                                     const std::vector<float>  &xSorted,
                                     const std::vector<double> &prefixW1,
                                     const std::vector<double> &prefixW2,
                                     const std::vector<double> &prefixW3,
                                     double                     rateMin,
                                     double                     ratePtp,
                                     double                     psnrMin,
                                     double                     psnrPtp,
                                     bool                       useNormParams,
                                     double                    &outRateMin,
                                     double                    &outRatePtp,
                                     double                    &outPsnrMin,
                                     double                    &outPsnrPtp ) {
  std::vector<double> psnrVals( powerGrid.size() ), rateVals( powerGrid.size() );
  for ( size_t i = 0; i < powerGrid.size(); ++i ) {
    metricsForPower( powerGrid[i], numBins, Q, valRange, valMin, N, sumWeights, log10MaxVal, xSorted, prefixW1,
                     prefixW2, prefixW3, psnrVals[i], rateVals[i] );
  }
  std::vector<double> rateGain, psnrDrop;
  std::vector<float>  validPowers;
  for ( size_t i = 0; i < powerGrid.size(); ++i ) {
    double rg = baselineRate - rateVals[i];
    if ( rg > 0.0 ) {
      rateGain.push_back( rg );
      psnrDrop.push_back( baselinePsnr - psnrVals[i] );
      validPowers.push_back( powerGrid[i] );
    }
  }
  if ( validPowers.empty() ) return -1.0;
  if ( !useNormParams ) {
    outRateMin  = *std::min_element( rateGain.begin(), rateGain.end() );
    double rMax = *std::max_element( rateGain.begin(), rateGain.end() );
    outRatePtp  = rMax - outRateMin + 1e-12;
    outPsnrMin  = *std::min_element( psnrDrop.begin(), psnrDrop.end() );
    double pMax = *std::max_element( psnrDrop.begin(), psnrDrop.end() );
    outPsnrPtp  = pMax - outPsnrMin + 1e-12;
  } else {
    outRateMin = rateMin;
    outRatePtp = ratePtp;
    outPsnrMin = psnrMin;
    outPsnrPtp = psnrPtp;
  }
  double bestScore = -1e18;
  float  bestPower = validPowers[0];
  for ( size_t i = 0; i < validPowers.size(); ++i ) {
    double score = ( rateGain[i] - outRateMin ) / outRatePtp - ( psnrDrop[i] - outPsnrMin ) / outPsnrPtp;
    if ( score > bestScore ) {
      bestScore = score;
      bestPower = validPowers[i];
    }
  }
  return static_cast<double>( bestPower );
}

static double findOptimalScalePower( const std::vector<double> &normData,
                                     uint32_t                   bitDepth,
                                     double                     valMin,
                                     double                     valMax ) {
  if ( normData.empty() ) return 4.0;
  const uint32_t     numBins  = 1u << bitDepth;
  const double       Q        = static_cast<double>( numBins - 1 );
  const double       valRange = valMax - valMin;
  const size_t       N        = normData.size();
  std::vector<float> xSorted( N );
  for ( size_t i = 0; i < N; ++i ) xSorted[i] = static_cast<float>( normData[i] );
  std::sort( xSorted.begin(), xSorted.end() );
  std::vector<double> wSorted( N );
  for ( size_t i = 0; i < N; ++i ) wSorted[i] = std::exp( xSorted[i] * valRange + valMin );
  double sumWeights = 0.0;
  for ( double w : wSorted ) sumWeights += w;
  double              log10MaxVal = std::log10( wSorted.back() );
  std::vector<double> prefixW1( N + 1, 0.0 ), prefixW2( N + 1, 0.0 ), prefixW3( N + 1, 0.0 );
  for ( size_t i = 0; i < N; ++i ) {
    double w        = wSorted[i];
    prefixW1[i + 1] = prefixW1[i] + w;
    prefixW2[i + 1] = prefixW2[i] + w * w;
    prefixW3[i + 1] = prefixW3[i] + w * w * w;
  }
  double baselinePsnr, baselineRate;
  metricsForPower( 1.0, numBins, Q, valRange, valMin, N, sumWeights, log10MaxVal, xSorted, prefixW1, prefixW2, prefixW3,
                   baselinePsnr, baselineRate );
  constexpr int      COARSE_N = 40;
  std::vector<float> coarseGrid( COARSE_N );
  for ( int i = 0; i < COARSE_N; ++i ) coarseGrid[i] = static_cast<float>( 3.5 + ( 4.5 / ( COARSE_N - 1 ) ) * i );
  double cRateMin, cRatePtp, cPsnrMin, cPsnrPtp;
  double coarsePower = findBestPowerFromGrid( coarseGrid, baselineRate, baselinePsnr, numBins, Q, valRange, valMin, N,
                                              sumWeights, log10MaxVal, xSorted, prefixW1, prefixW2, prefixW3, 0, 0, 0,
                                              0, false, cRateMin, cRatePtp, cPsnrMin, cPsnrPtp );
  if ( coarsePower < 0.0 ) return 3.5;
  constexpr int      FINE_N = 20;
  double             fLo    = std::max( 3.5, coarsePower - 0.3 );
  double             fHi    = std::min( 8.0, coarsePower + 0.3 );
  std::vector<float> fineGrid( FINE_N );
  for ( int i = 0; i < FINE_N; ++i ) fineGrid[i] = static_cast<float>( fLo + ( fHi - fLo ) / ( FINE_N - 1 ) * i );
  double fRateMin, fRatePtp, fPsnrMin, fPsnrPtp;
  double finePower = findBestPowerFromGrid(
      fineGrid, baselineRate, baselinePsnr, numBins, Q, valRange, valMin, N, sumWeights, log10MaxVal, xSorted, prefixW1,
      prefixW2, prefixW3, cRateMin, cRatePtp, cPsnrMin, cPsnrPtp, true, fRateMin, fRatePtp, fPsnrMin, fPsnrPtp );
  double optimal = ( finePower < 0.0 ) ? coarsePower : finePower;
  return std::round( optimal * 100.0 ) / 100.0;
}

// Helper: compute 2-segment PLS nodes from power and bit depth (S_THRESH = 0.60)
static void computePlsNodesFromPower( double   power,
                                      uint32_t bitDepth,
                                      int     &n1,
                                      double  &X1,
                                      double  &step1,
                                      double  &step2 ) {
  constexpr double S_THRESH       = 0.60;
  constexpr double MAX_STEP_RATIO = 8.0;
  const uint32_t   N              = 1u << bitDepth;
  X1                              = std::max( 0.005, std::min( 0.49, std::pow( S_THRESH, power ) ) );
  double dx1                      = X1;
  double dx2                      = 1.0 - X1;
  n1     = static_cast<int>( std::max( 1.0, std::ceil( dx1 * N / ( dx1 + MAX_STEP_RATIO * dx2 ) ) ) );
  int n2 = static_cast<int>( N ) - n1;
  step1  = dx1 / n1;
  step2  = dx2 / std::max( 1, n2 );
}

// ─────────────────────────────────────────────────────────────────────────────

//============================================================================
// PCCAdaptation - Scale Quantization
//============================================================================

bool PCCAdaptation::quantizeScales( const PCCPointSetGS     &input,
                                    PCCPointSet3            &output,
                                    PCCAdaptationParameters &params ) {
  const size_t pointCount = input.getPointCount();
  if ( pointCount == 0 || !input.hasScales() ) return false;

  // Auto-compute limits if enabled
  if ( params.autoComputeScaleLimits_ ) {
    for ( int k = 0; k < 3; ++k ) {
      params.computedScaleMin_[k] = std::numeric_limits<double>::max();
      params.computedScaleMax_[k] = std::numeric_limits<double>::lowest();
    }
    for ( size_t i = 0; i < pointCount; ++i ) {
      const GSScale &scale = input.getScales()[i];
      for ( int k = 0; k < 3; ++k ) {
        params.computedScaleMin_[k] = std::min( params.computedScaleMin_[k], static_cast<double>( scale[k] ) );
        params.computedScaleMax_[k] = std::max( params.computedScaleMax_[k], static_cast<double>( scale[k] ) );
      }
    }
  } else {
    params.computedScaleMin_ = params.scaleLimitMin_;
    params.computedScaleMax_ = params.scaleLimitMax_;
  }

  const uint32_t bitDepth      = params.scaleBitDepth_;
  const uint32_t max_quant_val = ( 1u << bitDepth ) - 1;

  if ( params.enableScalePowerTransform_ ) {
    {
      std::vector<double> normAll;
      normAll.reserve( pointCount * 3 );
      for ( size_t i = 0; i < pointCount; ++i ) {
        const GSScale &scale = input.getScales()[i];
        for ( int k = 0; k < 3; ++k ) {
          double n = normalizeToUnit( scale[k], params.computedScaleMin_[k], params.computedScaleMax_[k] );
          normAll.push_back( std::max( 0.0, std::min( 1.0, n ) ) );
        }
      }
      double optimalPower =
          findOptimalScalePower( normAll, bitDepth, params.computedScaleMin_[0], params.computedScaleMax_[0] );
      for ( int k = 0; k < 3; ++k ) params.computedScalePower_[k] = optimalPower;
    }
    for ( size_t i = 0; i < pointCount; ++i ) {
      const GSScale    &scale = input.getScales()[i];
      PCCAttribute16bit qScale;
      for ( int k = 0; k < 3; ++k ) {
        double power   = params.computedScalePower_[k];
        double scaling = params.computedScaleScalingFactor_[k];
        double norm    = std::max( 0.0, std::min( 1.0, normalizeToUnit( scale[k], params.computedScaleMin_[k],
                                                                        params.computedScaleMax_[k] ) ) );
        double pwr     = std::pow( norm, power ) * scaling;
        pwr            = std::max( 0.0, std::min( 1.0, pwr ) );
        int q;
        if ( params.enableScaleNonUniformQuantization_ ) {
          int    n1;
          double X1, step1, step2;
          computePlsNodesFromPower( power, bitDepth, n1, X1, step1, step2 );
          if ( pwr < X1 ) {
            q = static_cast<int>( std::round( pwr / step1 ) );
            q = std::max( 0, std::min( n1 - 1, q ) );
          } else {
            q = n1 + static_cast<int>( std::round( ( pwr - X1 ) / step2 ) );
            q = std::max( n1, std::min( static_cast<int>( max_quant_val ), q ) );
          }
        } else {
          q = static_cast<int>( quantizeValue( pwr, bitDepth ) );
        }
        qScale[k] = static_cast<uint16_t>( q );
      }
      output.setScale( i, qScale );
    }
    return true;
  }

  struct ScaleQuantRegion {
    std::vector<double> region_start_real;
    std::vector<double> region_end_real;
    std::vector<int>    start_int;
    std::vector<int>    end_int;
    std::vector<double> step_size;
  };
  std::array<ScaleQuantRegion, 3> scaleRegions;

  if ( params.enableScaleNonUniformQuantization_ ) {
    for ( size_t ch = 0; ch < 3; ++ch ) {
      const int num_regions_minus1 = params.scaleNumRegionsMinus1_[ch];
      for ( size_t r = 0; r < num_regions_minus1; ++r ) {
        double step_size = params.scaleStepSizes_[ch * ( num_regions_minus1 + 1 ) + r];

        if ( r == 0 ) {
          scaleRegions[ch].start_int.push_back( 0 );
          scaleRegions[ch].region_start_real.push_back( params.computedScaleMin_[ch] );
          scaleRegions[ch].step_size.push_back( step_size );
        }

        int current_region_offset = params.scaleTransitionPoints_[ch * num_regions_minus1 + r];
        scaleRegions[ch].start_int.push_back( current_region_offset );
        scaleRegions[ch].end_int.push_back( current_region_offset - 1 );

        if ( r > 0 ) current_region_offset -= params.scaleTransitionPoints_[ch * num_regions_minus1 + r - 1];

        double next_region_start_real =
            scaleRegions[ch].region_start_real[scaleRegions[ch].region_start_real.size() - 1];
        next_region_start_real += ( step_size * current_region_offset );
        scaleRegions[ch].region_start_real.push_back( next_region_start_real );
        scaleRegions[ch].region_end_real.push_back( next_region_start_real );

        double step_size_next = params.scaleStepSizes_[ch * ( num_regions_minus1 + 1 ) + r + 1];
        scaleRegions[ch].step_size.push_back( step_size_next );

        if ( r == ( num_regions_minus1 - 1 ) ) {
          scaleRegions[ch].region_end_real.push_back( params.computedScaleMax_[ch] );
          scaleRegions[ch].end_int.push_back( max_quant_val );
        }
      }
    }
  }

  for ( size_t i = 0; i < pointCount; ++i ) {
    const GSScale    &scale = input.getScales()[i];
    PCCAttribute16bit qScale;

    for ( int k = 0; k < 3; ++k ) {
      if ( params.enableScaleNonUniformQuantization_ ) {
        int num_regions_minus1 = params.scaleNumRegionsMinus1_[k];
        if ( num_regions_minus1 == 0 ) {
          double normalized = normalizeToUnit( scale[k], params.computedScaleMin_[k], params.computedScaleMax_[k] );
          qScale[k]         = static_cast<uint16_t>( quantizeValue( normalized, bitDepth ) );
        } else {
          const auto &region     = scaleRegions[k];
          double      q_val      = scale[k];
          int         region_idx = 0;
          while ( region_idx < num_regions_minus1 && q_val >= region.region_end_real[region_idx] ) { region_idx++; }
          int    previouds_idx = region.start_int[region_idx];
          double step_size     = region.step_size[region_idx];
          double base_real     = region.region_start_real[region_idx];
          q_val                = std::round( ( q_val - base_real ) / step_size + previouds_idx );
          qScale[k] = static_cast<uint16_t>( std::max( 0.0, std::min( static_cast<double>( max_quant_val ), q_val ) ) );
        }
      } else {
        double normalized = normalizeToUnit( scale[k], params.computedScaleMin_[k], params.computedScaleMax_[k] );
        qScale[k]         = static_cast<uint16_t>( quantizeValue( normalized, bitDepth ) );
      }
    }

    output.setScale( i, qScale );
  }

  return true;
}

bool PCCAdaptation::dequantizeScales( const PCCPointSet3            &input,
                                      PCCPointSetGS                 &output,
                                      const PCCAdaptationParameters &params ) {
  const size_t   pointCount = input.getPointCount();
  const uint32_t bitDepth   = params.scaleBitDepth_;

  if ( params.enableScalePowerTransform_ ) {
    for ( size_t i = 0; i < pointCount; ++i ) {
      const auto &qScale = input.getScale( i );
      GSScale    &scale  = output.getScales()[i];
      for ( int k = 0; k < 3; ++k ) {
        double power   = params.computedScalePower_[k];
        double scaling = params.computedScaleScalingFactor_[k];
        int    q       = static_cast<int>( qScale[k] );
        double pwr;
        if ( params.enableScaleNonUniformQuantization_ ) {
          int    n1;
          double X1, step1, step2;
          computePlsNodesFromPower( power, bitDepth, n1, X1, step1, step2 );
          pwr = ( q < n1 ) ? ( q * step1 ) : ( ( q - n1 ) * step2 + X1 );
        } else {
          pwr = dequantizeValue( static_cast<uint32_t>( q ), bitDepth );
        }
        pwr         = pwr / std::max( scaling, 1e-10 );
        pwr         = std::max( 1e-10, std::min( 1.0, pwr ) );
        double norm = std::pow( pwr, 1.0 / power );
        scale[k]    = denormalizeFromUnit( norm, params.computedScaleMin_[k], params.computedScaleMax_[k] );
      }
    }
    return true;
  }

  struct ScaleQuantRegion {
    std::vector<double> region_start_real;
    std::vector<double> region_end_real;
    std::vector<int>    start_int;
    std::vector<int>    end_int;
    std::vector<double> step_size;
  };
  std::array<ScaleQuantRegion, 3> scaleRegions;
  const uint32_t                  max_quant_val = ( 1u << bitDepth ) - 1;

  if ( params.enableScaleNonUniformQuantization_ ) {
    for ( size_t ch = 0; ch < 3; ++ch ) {
      const int num_regions_minus1 = params.scaleNumRegionsMinus1_[ch];
      for ( size_t r = 0; r < num_regions_minus1; ++r ) {
        double step_size = params.scaleStepSizes_[ch * ( num_regions_minus1 + 1 ) + r];

        if ( r == 0 ) {
          scaleRegions[ch].start_int.push_back( 0 );
          scaleRegions[ch].region_start_real.push_back( params.computedScaleMin_[ch] );
          scaleRegions[ch].step_size.push_back( step_size );
        }

        int current_region_offset = params.scaleTransitionPoints_[ch * num_regions_minus1 + r];
        scaleRegions[ch].start_int.push_back( current_region_offset );
        scaleRegions[ch].end_int.push_back( current_region_offset - 1 );

        if ( r > 0 ) current_region_offset -= params.scaleTransitionPoints_[ch * num_regions_minus1 + r - 1];

        double next_region_start_real =
            scaleRegions[ch].region_start_real[scaleRegions[ch].region_start_real.size() - 1];
        next_region_start_real += ( step_size * current_region_offset );
        scaleRegions[ch].region_start_real.push_back( next_region_start_real );
        scaleRegions[ch].region_end_real.push_back( next_region_start_real );

        double step_size_next = params.scaleStepSizes_[ch * ( num_regions_minus1 + 1 ) + r + 1];
        scaleRegions[ch].step_size.push_back( step_size_next );

        if ( r == ( num_regions_minus1 - 1 ) ) {
          scaleRegions[ch].region_end_real.push_back( params.computedScaleMax_[ch] );
          scaleRegions[ch].end_int.push_back( max_quant_val );
        }
      }
    }
  }

  for ( size_t i = 0; i < pointCount; ++i ) {
    const auto &qScale = input.getScale( i );
    GSScale    &scale  = output.getScales()[i];

    for ( int k = 0; k < 3; ++k ) {
      if ( params.enableScaleNonUniformQuantization_ ) {
        int num_regions_minus1 = params.scaleNumRegionsMinus1_[k];
        if ( num_regions_minus1 == 0 ) {
          double normalized = normalizeToUnit( qScale[k], params.computedScaleMin_[k], params.computedScaleMax_[k] );
          scale[k]          = static_cast<uint16_t>( quantizeValue( normalized, bitDepth ) );
        } else {
          const auto &region     = scaleRegions[k];
          int         region_idx = 0;
          double      q_val      = qScale[k];
          while ( region_idx < num_regions_minus1 && q_val > region.end_int[region_idx] ) { region_idx++; }
          int    previous_idx = region.start_int[region_idx];
          double step_size    = region.step_size[region_idx];
          double base_real    = region.region_start_real[region_idx];
          scale[k]            = ( q_val - previous_idx ) * step_size + base_real;
        }
      } else {
        double normalized = dequantizeValue( qScale[k], bitDepth );
        scale[k]          = denormalizeFromUnit( normalized, params.computedScaleMin_[k], params.computedScaleMax_[k] );
      }
    }
  }

  return true;
}

//============================================================================
// PCCAdaptation - Opacity Quantization
//============================================================================

bool PCCAdaptation::quantizeOpacities( const PCCPointSetGS     &input,
                                       PCCPointSet3            &output,
                                       PCCAdaptationParameters &params ) {
  const size_t pointCount = input.getPointCount();
  if ( pointCount == 0 || !input.hasOpacities() ) return false;

  // Auto-compute limits if enabled
  if ( !params.enableOpacityNonUniformQuantization_ ) {
    if ( params.autoComputeOpacityLimits_ ) {
      params.computedOpacityMin_ = std::numeric_limits<double>::max();
      params.computedOpacityMax_ = std::numeric_limits<double>::lowest();
      for ( size_t i = 0; i < pointCount; ++i ) {
        double opacity             = input.getOpacities()[i];
        params.computedOpacityMin_ = std::min( params.computedOpacityMin_, opacity );
        params.computedOpacityMax_ = std::max( params.computedOpacityMax_, opacity );
      }
    } else {
      params.computedOpacityMin_ = params.opacityLimitMin_;
      params.computedOpacityMax_ = params.opacityLimitMax_;
    }
  }
  const uint32_t bitDepth = params.opacityBitDepth_;

  const uint32_t max_val = ( 1u << bitDepth ) - 1;  // m76122
  struct OpacityQuantRegion {
    std::vector<double> region_start_real;
    std::vector<int>    tp_int;
  };
  OpacityQuantRegion opRegion;
  if ( params.enableOpacityNonUniformQuantization_ ) {
    int    M      = params.opacityNumRegionsMinus1_;
    double minLim = params.computedOpacityMin_;

    if ( M > 0 ) {
      opRegion.tp_int.resize( M );
      opRegion.region_start_real.resize( M + 1 );
      opRegion.region_start_real[0] = minLim;
      for ( int m = 0; m < M; ++m ) {
        int tp_val         = params.opacityTransitionPoints_[m];
        opRegion.tp_int[m] = tp_val;
        double prev_tp     = ( m == 0 ) ? 0.0 : opRegion.tp_int[m - 1];
        opRegion.region_start_real[m + 1] =
            opRegion.region_start_real[m] + ( tp_val - prev_tp ) * params.opacityStepSizes_[m];
      }
    }
  }

  for ( size_t i = 0; i < pointCount; ++i ) {
    double   opacity   = input.getOpacities()[i];
    uint32_t quantized = 0;
    if ( params.enableOpacityNonUniformQuantization_ ) {
      int M = params.opacityNumRegionsMinus1_;
      if ( M == 0 ) {
        double normalized = normalizeToUnit( opacity, params.computedOpacityMin_, params.computedOpacityMax_ );
        quantized         = quantizeValue( normalized, bitDepth );
      } else {
        int region_idx = 0;
        while ( region_idx < M && opacity > opRegion.region_start_real[region_idx + 1] ) { region_idx++; }
        double prev_idx  = ( region_idx > 0 ) ? opRegion.tp_int[region_idx - 1] : 0.0;
        double step_vec  = params.opacityStepSizes_[region_idx];
        double base_real = opRegion.region_start_real[region_idx];
        double q_vals    = std::round( ( opacity - base_real ) / step_vec + prev_idx );
        quantized        = static_cast<uint32_t>( std::max( 0.0, std::min( static_cast<double>( max_val ), q_vals ) ) );
      }
    } else {
      double normalized = normalizeToUnit( opacity, params.computedOpacityMin_, params.computedOpacityMax_ );
      quantized         = quantizeValue( normalized, bitDepth );
    }
    // Store opacity in the first component of PCCAttribute16bit
    PCCAttribute16bit qOpacity;
    qOpacity[0] = static_cast<uint16_t>( quantized );
    qOpacity[1] = 0;
    qOpacity[2] = 0;

    output.setOpacity( i, qOpacity );
  }

  return true;
}

bool PCCAdaptation::dequantizeOpacities( const PCCPointSet3            &input,
                                         PCCPointSetGS                 &output,
                                         const PCCAdaptationParameters &params ) {
  const size_t   pointCount = input.getPointCount();
  const uint32_t bitDepth   = params.opacityBitDepth_;

  struct OpacityQuantRegion {
    std::vector<double> region_start_real;
    std::vector<int>    tp_int;
  };
  OpacityQuantRegion opRegion;
  if ( params.enableOpacityNonUniformQuantization_ ) {
    int    M      = params.opacityNumRegionsMinus1_;
    double minLim = params.computedOpacityMin_;
    if ( M > 0 ) {
      opRegion.tp_int.resize( M );
      opRegion.region_start_real.resize( M + 1 );
      opRegion.region_start_real[0] = minLim;
      for ( int m = 0; m < M; ++m ) {
        int tp_val         = params.opacityTransitionPoints_[m];
        opRegion.tp_int[m] = tp_val;
        double prev_tp     = ( m == 0 ) ? 0.0 : opRegion.tp_int[m - 1];
        opRegion.region_start_real[m + 1] =
            opRegion.region_start_real[m] + ( tp_val - prev_tp ) * params.opacityStepSizes_[m];
      }
    }
  }

  for ( size_t i = 0; i < pointCount; ++i ) {
    const auto &qOpacity = input.getOpacity( i );
    if ( params.enableOpacityNonUniformQuantization_ ) {
      int M = params.opacityNumRegionsMinus1_;
      if ( M == 0 ) {
        double normalized = dequantizeValue( qOpacity[0], bitDepth );
        output.getOpacities()[i] =
            denormalizeFromUnit( normalized, params.computedOpacityMin_, params.computedOpacityMax_ );
      } else {
        int q_val      = qOpacity[0];
        int region_idx = 0;
        while ( region_idx < M && q_val >= opRegion.tp_int[region_idx] ) { region_idx++; }

        double prev_idx  = ( region_idx > 0 ) ? opRegion.tp_int[region_idx - 1] : 0.0;
        double base_real = opRegion.region_start_real[region_idx];
        double step_vec  = params.opacityStepSizes_[region_idx];
        double r_opacity = base_real + ( q_val - prev_idx ) * step_vec;
        output.getOpacities()[i] =
            std::max( params.computedOpacityMin_, std::min( params.computedOpacityMax_, r_opacity ) );
      }
    } else {
      double normalized = dequantizeValue( qOpacity[0], bitDepth );
      output.getOpacities()[i] =
          denormalizeFromUnit( normalized, params.computedOpacityMin_, params.computedOpacityMax_ );
    }
  }

  return true;
}

void PCCAdaptation::logitToLinearOpacity( PCCPointSetGS &pointCloud ) {
  if ( !pointCloud.hasOpacities() ) return;
  auto &opacities = pointCloud.getOpacities();
  for ( auto &opacity : opacities ) {
    // Convert logit to linear: linear = 1 / (1 + exp(-logit))
    opacity = 1.0f / ( 1.0f + std::exp( -opacity ) );
  }
}

void PCCAdaptation::linearToLogitOpacity( PCCPointSetGS &pointCloud ) {
  if ( !pointCloud.hasOpacities() ) return;
  auto &opacities = pointCloud.getOpacities();
  for ( auto &opacity : opacities ) {
    // Convert linear to logit: logit = log(linear / (1 - linear))
    opacity = std::log( opacity / ( 1.0f - opacity ) );
  }
}

//============================================================================
// PCCAdaptation - Spherical Harmonics Quantization
//============================================================================

bool PCCAdaptation::quantizeSphericalHarmonics( const PCCPointSetGS     &input,
                                                PCCPointSet3            &output,
                                                PCCAdaptationParameters &params ) {
  const size_t pointCount = input.getPointCount();
  if ( pointCount == 0 || !input.hasSphericalHarmonics() ) return false;

  // Store SH coefficient count
  params.shCoefficientCount_ = input.getSphericalHarmonicsCount();
  if ( params.shCoefficientCount_ == 0 ) params.shCoefficientCount_ = 16;

  // Store limits used
  for ( size_t c = 0; c < params.shCoefficientCount_; ++c ) {
    params.computedShMin_[c] = params.shLimitMin_[c];
    params.computedShMax_[c] = params.shLimitMax_[c];
  }

  const uint32_t bitDepth  = params.shBitDepth_;
  const uint32_t max_index = 1u << bitDepth;
  const uint32_t max_val   = max_index - 1;

  struct ShQuantRegion {
    std::vector<double> th_real;
    std::vector<double> region_start_real;
    std::vector<int>    tp_int;
    double              min_offset;
  };
  std::array<ShQuantRegion, 48> shRegions;
  std::array<int, 48>           ssBaseOffsets;

  if ( params.enableShNonUniformQuantization_ ) {
    int tpOffset = 0;
    int ssOffset = 0;
    for ( int l = 0; l < 16; ++l ) {
      for ( int ch = 0; ch < 3; ++ch ) {
        int    idx         = l * 3 + ch;
        int    M           = params.shNumRegionsMinus1_[idx];
        double minLim      = params.computedShMin_[l][ch];
        double maxLim      = params.computedShMax_[l][ch];
        ssBaseOffsets[idx] = ssOffset;

        if ( M > 0 ) {
          shRegions[idx].th_real.resize( M );
          shRegions[idx].tp_int.resize( M );
          shRegions[idx].region_start_real.resize( M + 1 );
          shRegions[idx].region_start_real[0] = minLim;

          for ( int m = 0; m < M; ++m ) {
            int tp_val                = params.shTransitionPoints_[tpOffset + m];
            shRegions[idx].tp_int[m]  = tp_val;
            shRegions[idx].th_real[m] = tp_val * ( maxLim - minLim ) / static_cast<double>( max_index ) + minLim;
            shRegions[idx].region_start_real[m + 1] = shRegions[idx].th_real[m];
          }

          shRegions[idx].min_offset = shRegions[idx].tp_int[0] -
                                      ( shRegions[idx].region_start_real[1] - minLim ) / params.shStepSizes_[ssOffset];
        }

        tpOffset += M;
        ssOffset += M + 1;
      }
    }
  }

  for ( size_t i = 0; i < pointCount; ++i ) {
    const GSSphericalHarmonics    &sh = input.getSphericalHarmonics()[i];
    std::vector<PCCAttribute16bit> qSh( 16 );
    uint32_t                       pointBitDepth = bitDepth;
    if ( params.enableAdaptiveShQuantization_ ) { pointBitDepth = getAdaptiveBitDepth( bitDepth, input, i ); }

    for ( size_t l = 0; l < params.shCoefficientCount_; ++l ) {
      for ( int ch = 0; ch < 3; ++ch ) {
        if ( params.enableShNonUniformQuantization_ ) {
          int idx = l * 3 + ch;
          int M   = params.shNumRegionsMinus1_[idx];
          if ( M == 0 ) {
            double norm = normalizeToUnit( sh[l][ch], params.computedShMin_[l][ch], params.computedShMax_[l][ch] );
            qSh[l][ch]  = static_cast<uint16_t>( quantizeValue( norm, pointBitDepth ) );
          } else {
            const auto &reg        = shRegions[idx];
            int         region_idx = 0;
            while ( region_idx < M && sh[l][ch] > reg.th_real[region_idx] ) { region_idx++; }

            double prev_idx  = ( region_idx > 0 ) ? reg.tp_int[region_idx - 1] : reg.min_offset;
            double step_vec  = params.shStepSizes_[ssBaseOffsets[idx] + region_idx];
            double base_real = reg.region_start_real[region_idx];
            double q_vals    = ( sh[l][ch] - base_real ) / step_vec + prev_idx;
            if ( params.enableAdaptiveShQuantization_ ) { q_vals = q_vals / exp2( bitDepth - pointBitDepth ); }
            q_vals     = std::round( q_vals );
            qSh[l][ch] = static_cast<uint16_t>( std::max( 0.0, std::min( static_cast<double>( max_val ), q_vals ) ) );
          }
        } else {
          double norm = normalizeToUnit( sh[l][ch], params.computedShMin_[l][ch], params.computedShMax_[l][ch] );
          qSh[l][ch]  = static_cast<uint16_t>( quantizeValue( norm, pointBitDepth ) );
        }
      }
    }

    output.setSphericalHarmonic( i, qSh );
  }

  return true;
}

bool PCCAdaptation::dequantizeSphericalHarmonics( const PCCPointSet3            &input,
                                                  PCCPointSetGS                 &output,
                                                  const PCCAdaptationParameters &params ) {
  const size_t   pointCount = input.getPointCount();
  const uint32_t bitDepth   = params.shBitDepth_;
  const uint32_t max_index  = 1u << bitDepth;

  output.setSphericalHarmonicsCount( params.shCoefficientCount_ );

  struct ShQuantRegion {
    std::vector<double> region_start_real;
    std::vector<int>    tp_int;
    double              min_offset;
  };
  std::array<ShQuantRegion, 48> shRegions;
  std::array<int, 48>           ssBaseOffsets;

  if ( params.enableShNonUniformQuantization_ ) {
    int tpOffset = 0;
    int ssOffset = 0;
    for ( int l = 0; l < params.shCoefficientCount_; ++l ) {
      for ( int ch = 0; ch < 3; ++ch ) {
        int    idx         = l * 3 + ch;
        int    M           = params.shNumRegionsMinus1_[idx];
        double minLim      = params.computedShMin_[l][ch];
        double maxLim      = params.computedShMax_[l][ch];
        ssBaseOffsets[idx] = ssOffset;

        if ( M > 0 ) {
          shRegions[idx].tp_int.resize( M );
          shRegions[idx].region_start_real.resize( M + 1 );
          shRegions[idx].region_start_real[0] = minLim;

          for ( int m = 0; m < M; ++m ) {
            int tp_val               = params.shTransitionPoints_[tpOffset + m];
            shRegions[idx].tp_int[m] = tp_val;
            shRegions[idx].region_start_real[m + 1] =
                tp_val * ( maxLim - minLim ) / static_cast<double>( max_index ) + minLim;
          }

          shRegions[idx].min_offset = shRegions[idx].tp_int[0] -
                                      ( shRegions[idx].region_start_real[1] - minLim ) / params.shStepSizes_[ssOffset];

          tpOffset += M;
          ssOffset += M + 1;
        }
      }
    }
  }

  for ( size_t i = 0; i < pointCount; ++i ) {
    const auto           &qSh           = input.getSphericalHarmonics()[i];
    GSSphericalHarmonics &sh            = output.getSphericalHarmonics()[i];
    uint32_t              pointBitDepth = bitDepth;
    if ( params.enableAdaptiveShQuantization_ ) { pointBitDepth = getAdaptiveBitDepth( bitDepth, input, i ); }

    for ( size_t l = 0; l < params.shCoefficientCount_; ++l ) {
      for ( int ch = 0; ch < 3; ++ch ) {
        if ( params.enableShNonUniformQuantization_ ) {
          int idx = l * 3 + ch;
          int M   = params.shNumRegionsMinus1_[idx];
          if ( M == 0 ) {
            sh[l][ch] = denormalizeFromUnit( dequantizeValue( qSh[l][ch], pointBitDepth ), params.computedShMin_[l][ch],
                                             params.computedShMax_[l][ch] );
          } else {
            const auto &reg   = shRegions[idx];
            int         q_val = qSh[l][ch];
            if ( params.enableAdaptiveShQuantization_ ) { q_val = q_val * exp2( bitDepth - pointBitDepth ); }
            int region_idx = 0;
            while ( region_idx < M && q_val > reg.tp_int[region_idx] ) { region_idx++; }

            double prev_idx  = ( region_idx > 0 ) ? reg.tp_int[region_idx - 1] : reg.min_offset;
            double step_vec  = params.shStepSizes_[ssBaseOffsets[idx] + region_idx];
            double base_real = reg.region_start_real[region_idx];
            sh[l][ch]        = base_real + ( q_val - prev_idx ) * step_vec;
          }
        } else {
          sh[l][ch] = denormalizeFromUnit( dequantizeValue( qSh[l][ch], pointBitDepth ), params.computedShMin_[l][ch],
                                           params.computedShMax_[l][ch] );
        }
      }
    }
  }

  return true;
}

void PCCAdaptation::initializeShTransformParams( PCCAdaptationParameters &params ) {
  if ( !params.gsdTransform2dPresentFlag_ ) {
    params.shTrTypeCol_.clear();
    params.shTrGrpRow_.clear();
    params.shTrTypeRow_.clear();
    params.shKltFlag_.clear();
  } else if ( params.dataAdaptiveShTransformEnabled_ ) {
    // parameter for group PCA m75790
    params.shTrTypeCol_.assign( 16, 3 );  // changed to BT.709
    params.shTrGrpRow_ = { { 1, 5, 11 }, { 3, 7, 13 }, { 4, 10 }, { 8, 14 }, { 2, 6, 12 }, { 9, 15 } };
    params.shTrTypeRow_.assign( 6, 0 );
    params.shKltFlag_.assign( 6, 1 );
    // m76858: add DC PCA group
    if ( params.gsdPcaDcFlag_ ) {
      params.shTrGrpRow_.push_back( { 0 } );
      params.shTrTypeRow_.push_back( 0 );
      params.shKltFlag_.push_back( 1 );
    }
  } else {
    // parameter for 2D transform m75038
    params.shTrTypeCol_.assign( 16, 2 );
    params.shTrTypeCol_[0] = 1;
    params.shTrGrpRow_     = { { 1, 5, 11 }, { 4, 10 }, { 8, 14 } };
    params.shTrTypeRow_.assign( 3, 2 );
    params.shKltFlag_.assign( 3, 0 );
  }
}

bool PCCAdaptation::sphericalHarmonicsTransform( PCCPointSetGS &pointCloud, PCCAdaptationParameters &params ) {
  if ( !pointCloud.hasSphericalHarmonics() ) return false;
  if ( params.gsdTransform2dPresentFlag_ ) {
    bool anyKlt = false;
    for ( int flag : params.shKltFlag_ ) {
      if ( flag == 1 ) {
        anyKlt = true;
        break;
      }
    }

    if ( anyKlt ) {
      const size_t pointCount  = pointCloud.getPointCount();
      const size_t shCoefCount = pointCloud.getSphericalHarmonicsCount();
      if ( shCoefCount > 1 && pointCount > 0 ) {
        double totalStd = 0.0;
        int    count    = 0;
        for ( size_t l = 1; l < shCoefCount; ++l ) {
          for ( int ch = 0; ch < 3; ++ch ) {
            double sum   = 0.0;
            double sumSq = 0.0;
            for ( size_t p = 0; p < pointCount; ++p ) {
              double val = pointCloud.getSphericalHarmonics()[p][l][ch];
              sum += val;
              sumSq += val * val;
            }
            double mean = sum / pointCount;
            double var  = ( sumSq / pointCount ) - ( mean * mean );
            totalStd += std::sqrt( std::max( 0.0, var ) );
            count++;
          }
        }
        double meanStd = totalStd / count;
        if ( meanStd < 0.05 ) {
          std::vector<std::vector<int>> newGroups;
          std::vector<int>              newTrTypeRow;
          std::vector<int>              newKltFlag;
          for ( size_t i = 0; i < params.shTrGrpRow_.size(); ++i ) {
            bool isDcGroup = ( params.shTrGrpRow_[i].size() == 1 && params.shTrGrpRow_[i][0] == 0 );
            if ( params.shKltFlag_[i] == 0 || isDcGroup ) {
              newGroups.push_back( params.shTrGrpRow_[i] );
              newTrTypeRow.push_back( params.shTrTypeRow_[i] );
              newKltFlag.push_back( params.shKltFlag_[i] );
            }
          }
          params.shTrGrpRow_  = newGroups;
          params.shTrTypeRow_ = newTrTypeRow;
          params.shKltFlag_.assign( newTrTypeRow.size(), 0 );
          params.shKltFlag_ = newKltFlag;
        } else if ( params.adaptivePcaEnabled_ ) {
          // Select global PCA vs group PCA based on inter-group correlation
          double groupCorr = computeGroupCorr( pointCloud );
          if ( groupCorr < 0.5 ) {
            // low inter-group correlation → global PCA (all AC SH in one group)
            std::vector<int> allAc;
            for ( size_t l = 1; l < shCoefCount; ++l ) { allAc.push_back( static_cast<int>( l ) ); }
            params.shTrGrpRow_  = { allAc };
            params.shTrTypeRow_ = { 0 };
            params.shKltFlag_   = { 1 };
            if ( params.gsdPcaDcFlag_ ) {
              params.shTrGrpRow_.push_back( { 0 } );
              params.shTrTypeRow_.push_back( 0 );
              params.shKltFlag_.push_back( 1 );
            }
          }
          // else: keep default group PCA (params already set by initializeShTransformParams)
        }
      }
    }
    return sphericalHarmonicsTransform2D( pointCloud, params );
  }

  const size_t pointCount  = pointCloud.getPointCount();
  const size_t shCoefCount = pointCloud.getSphericalHarmonicsCount();

  for ( size_t p = 0; p < pointCount; ++p ) {
    auto &sh = pointCloud.getSphericalHarmonics()[p];
    for ( size_t l = 0; l < shCoefCount; ++l ) {
      double v0, v1, v2;
      rgbToYuv( sh[l][0], sh[l][1], sh[l][2], v0, v1, v2 );
      sh[l][0] = static_cast<float>( v0 );
      sh[l][1] = static_cast<float>( v1 );
      sh[l][2] = static_cast<float>( v2 );
    }
  }
  return true;
}

bool PCCAdaptation::sphericalHarmonicsInvTransform( PCCPointSetGS &pointCloud, const PCCAdaptationParameters &params ) {
  if ( params.gsdTransform2dPresentFlag_ ) { return sphericalHarmonicsInvTransform2D( pointCloud, params ); }

  if ( !pointCloud.hasSphericalHarmonics() ) return false;
  const size_t pointCount  = pointCloud.getPointCount();
  const size_t shCoefCount = pointCloud.getSphericalHarmonicsCount();

  for ( size_t p = 0; p < pointCount; ++p ) {
    auto &sh = pointCloud.getSphericalHarmonics()[p];
    for ( size_t l = 0; l < shCoefCount; ++l ) {
      double v0, v1, v2;
      yuvToRgb( sh[l][0], sh[l][1], sh[l][2], v0, v1, v2 );
      sh[l][0] = static_cast<float>( v0 );
      sh[l][1] = static_cast<float>( v1 );
      sh[l][2] = static_cast<float>( v2 );
    }
  }
  return true;
}

// Compute mean absolute correlation across default SH groups
double PCCAdaptation::computeGroupCorr( const PCCPointSetGS &pointCloud ) {
  const auto  &shData      = pointCloud.getSphericalHarmonics();
  const size_t pointCount  = pointCloud.getPointCount();
  const size_t shCoefCount = pointCloud.getSphericalHarmonicsCount();
  if ( pointCount == 0 || shCoefCount <= 1 ) return 0.0;

  const std::vector<std::vector<int>> defaultGroups = { { 1, 5, 11 }, { 3, 7, 13 }, { 4, 10 },
                                                        { 8, 14 },    { 2, 6, 12 }, { 9, 15 } };

  double groupCorrSum = 0.0;
  int    groupCount   = 0;

  for ( const auto &group : defaultGroups ) {
    std::vector<int> validIdx;
    for ( int g : group ) {
      if ( static_cast<size_t>( g ) < shCoefCount ) validIdx.push_back( g );
    }
    if ( validIdx.size() < 2 ) continue;

    auto cols = validIdx.size() * 3;

    std::vector<double> mean( cols, 0.0 );
    for ( size_t p = 0; p < pointCount; ++p ) {
      for ( size_t j = 0; j < cols; ++j ) { mean[j] += shData[p][validIdx[j / 3]][j % 3]; }
    }
    for ( size_t j = 0; j < cols; ++j ) { mean[j] /= static_cast<double>( pointCount ); }

    std::vector<double> cov( cols * cols, 0.0 );
    std::vector<double> var( cols, 0.0 );
    for ( size_t p = 0; p < pointCount; ++p ) {
      std::vector<double> x( cols );
      for ( size_t j = 0; j < cols; ++j ) { x[j] = shData[p][validIdx[j / 3]][j % 3] - mean[j]; }
      for ( size_t r = 0; r < cols; ++r ) {
        var[r] += x[r] * x[r];
        for ( size_t c = r + 1; c < cols; ++c ) { cov[r * cols + c] += x[r] * x[c]; }
      }
    }
    double N = static_cast<double>( pointCount );
    for ( size_t j = 0; j < cols; ++j ) { var[j] /= N; }
    for ( size_t r = 0; r < cols; ++r ) {
      for ( size_t c = r + 1; c < cols; ++c ) { cov[r * cols + c] /= N; }
    }

    double corrSum   = 0.0;
    int    pairCount = 0;
    for ( size_t r = 0; r < cols; ++r ) {
      for ( size_t c = r + 1; c < cols; ++c ) {
        double denom = std::sqrt( var[r] * var[c] );
        if ( denom > 1e-10 ) {
          corrSum += std::abs( cov[r * cols + c] / denom );
          ++pairCount;
        }
      }
    }
    if ( pairCount > 0 ) {
      groupCorrSum += corrSum / static_cast<double>( pairCount );
      ++groupCount;
    }
  }

  return ( groupCount > 0 ) ? groupCorrSum / static_cast<double>( groupCount ) : 0.0;
}

bool PCCAdaptation::computeShKlt( const PCCPointSetGS                       &pointCloud,
                                  const std::vector<int>                    &groupidx,
                                  PCCAdaptationParameters::PCCKltParameters &klt ) {
  const size_t pointCount = pointCloud.getPointCount();
  if ( pointCount == 0 ) return false;
  const size_t size     = groupidx.size();
  const size_t fullSize = size * 3;
  const auto  &shData   = pointCloud.getSphericalHarmonics();

  klt.eigvec.assign( fullSize, std::vector<double>( fullSize, 0.0 ) );
  klt.mean.assign( fullSize, 0.0 );
  klt.sigma.assign( fullSize, 1.0 );

  for ( size_t p = 0; p < pointCount; ++p ) {
    for ( int i = 0; i < size; ++i ) {
      for ( int ch = 0; ch < 3; ++ch ) { klt.mean[i * 3 + ch] += shData[p][groupidx[i]][ch]; }
    }
  }
  for ( int j = 0; j < fullSize; ++j ) { klt.mean[j] /= (double)pointCount; }

  std::vector<double> sigma( fullSize, 0.0 );

  for ( size_t p = 0; p < pointCount; ++p ) {
    std::vector<double> centered( fullSize );
    for ( int j = 0; j < fullSize; ++j ) {
      centered[j] = (double)shData[p][groupidx[j / 3]][j % 3] - klt.mean[j];
      sigma[j] += centered[j] * centered[j];
    }
  }

  for ( int j = 0; j < fullSize; ++j ) {
    sigma[j] = std::sqrt( sigma[j] / (double)pointCount );
    if ( sigma[j] < 1e-10 ) sigma[j] = 1.0;
    klt.sigma[j] = sigma[j];
  }

  std::vector<double> cov( fullSize * fullSize, 0.0 );
  for ( size_t p = 0; p < pointCount; ++p ) {
    std::vector<double> normalized( fullSize );
    for ( int j = 0; j < fullSize; ++j ) {
      normalized[j] = ( (double)shData[p][groupidx[j / 3]][j % 3] - klt.mean[j] ) / klt.sigma[j];
    }
    for ( int r = 0; r < fullSize; ++r ) {
      for ( int c = 0; c < fullSize; ++c ) { cov[r + c * fullSize] += normalized[r] * normalized[c]; }
    }
  }

  double div = double( pointCount > 1 ? pointCount - 1 : 1 );
  for ( int i = 0; i < fullSize * fullSize; ++i ) { cov[i] /= div; }
  // Symmetric eigendecomposition (in-tree Jacobi solver).
  SymEigenResult eig;
  if ( !symEigenDecomposeJacobi( cov, fullSize, eig ) ) {
    std::cerr << "Jacobi eigendecomposition failed to converge" << std::endl;
    return false;
  }

  // Sort in descending order (solver returns ascending).
  std::vector<std::pair<double, std::size_t>> eigPairs( fullSize );
  for ( std::size_t i = 0; i < fullSize; ++i ) { eigPairs[i] = { eig.eigenvalues[i], i }; }
  std::sort( eigPairs.rbegin(), eigPairs.rend() );

  // Eigenvectors are columns of eig.eigenvectors (column-major).
  for ( std::size_t j = 0; j < fullSize; ++j ) {
    const std::size_t src = eigPairs[j].second;
    for ( std::size_t i = 0; i < fullSize; ++i ) { klt.eigvec[j][i] = eig.eigenvectors[i + src * fullSize]; }
  }
  return true;
}

double PCCAdaptation::computeShAcTop10Correlation( const PCCPointSetGS &pointCloud ) {
  const size_t pointCount  = pointCloud.getPointCount();
  const size_t shCoefCount = pointCloud.getSphericalHarmonicsCount();
  if ( pointCount == 0 || shCoefCount <= 1 ) return 0.0;

  const size_t acCount = shCoefCount - 1;  // indices 1..15
  const auto  &shData  = pointCloud.getSphericalHarmonics();

  std::vector<double> acMeanSum( acCount, 0.0 );
  for ( size_t p = 0; p < pointCount; ++p ) {
    for ( size_t l = 0; l < acCount; ++l ) {
      acMeanSum[l] += ( (double)shData[p][l + 1][0] + (double)shData[p][l + 1][1] + (double)shData[p][l + 1][2] ) / 3.0;
    }
  }
  std::vector<double> acMean( acMeanSum );
  for ( size_t l = 0; l < acCount; ++l ) { acMean[l] = acMeanSum[l] / (double)pointCount; }

  std::vector<double> var( acCount, 0.0 );
  std::vector<double> cov( acCount * acCount, 0.0 );
  for ( size_t p = 0; p < pointCount; ++p ) {
    std::vector<double> centered( acCount );
    for ( size_t l = 0; l < acCount; ++l ) {
      centered[l] =
          ( (double)shData[p][l + 1][0] + (double)shData[p][l + 1][1] + (double)shData[p][l + 1][2] ) / 3.0 - acMean[l];
    }
    for ( size_t r = 0; r < acCount; ++r ) {
      var[r] += centered[r] * centered[r];
      for ( size_t c = r + 1; c < acCount; ++c ) { cov[r * acCount + c] += centered[r] * centered[c]; }
    }
  }
  for ( size_t r = 0; r < acCount; ++r ) {
    var[r] /= (double)( pointCount > 1 ? pointCount - 1 : 1 );
    for ( size_t c = r + 1; c < acCount; ++c ) {
      cov[r * acCount + c] /= (double)( pointCount > 1 ? pointCount - 1 : 1 );
    }
  }

  std::vector<double> absCorrs;
  for ( size_t r = 0; r < acCount; ++r ) {
    for ( size_t c = r + 1; c < acCount; ++c ) {
      double denom = std::sqrt( std::max( 1e-20, var[r] * var[c] ) );
      absCorrs.push_back( std::abs( cov[r * acCount + c] / denom ) );
    }
  }

  std::sort( absCorrs.rbegin(), absCorrs.rend() );
  int    topN = std::min( 10, (int)absCorrs.size() );
  double sum  = 0.0;
  for ( int i = 0; i < topN; ++i ) { sum += absCorrs[i]; }
  return sum / topN;
}

bool PCCAdaptation::computeShDcPca( const PCCPointSetGS                       &pointCloud,
                                    double                                     top10corr,
                                    PCCAdaptationParameters::PCCKltParameters &klt ) {
  const size_t pointCount = pointCloud.getPointCount();
  if ( pointCount == 0 ) return false;

  const int   fullSize = 3;
  const auto &shData   = pointCloud.getSphericalHarmonics();

  klt.eigvec.assign( fullSize, std::vector<double>( fullSize, 0.0 ) );
  klt.mean.assign( fullSize, 0.0 );
  klt.sigma.assign( fullSize, 1.0 );

  for ( size_t p = 0; p < pointCount; ++p ) {
    for ( int ch = 0; ch < fullSize; ++ch ) { klt.mean[ch] += (double)shData[p][0][ch]; }
  }
  for ( int j = 0; j < fullSize; ++j ) { klt.mean[j] /= (double)pointCount; }

  std::vector<double> cov( static_cast<size_t>( fullSize ) * fullSize, 0.0 );
  for ( size_t p = 0; p < pointCount; ++p ) {
    std::vector<double> centered( fullSize );
    for ( int j = 0; j < fullSize; ++j ) { centered[j] = (double)shData[p][0][j] - klt.mean[j]; }
    for ( int r = 0; r < fullSize; ++r ) {
      for ( int c = 0; c < fullSize; ++c ) { cov[r + c * fullSize] += centered[r] * centered[c]; }
    }
  }
  double div = double( pointCount > 1 ? pointCount - 1 : 1 );
  for ( int i = 0; i < fullSize * fullSize; ++i ) { cov[i] /= div; }

  SymEigenResult eig;
  if ( !symEigenDecomposeJacobi( cov, static_cast<std::size_t>( fullSize ), eig ) ) {
    std::cerr << "Jacobi eigendecomposition failed in computeShDcPca" << std::endl;
    return false;
  }

  // Sort in descending order (solver returns ascending).
  std::vector<std::pair<double, std::size_t>> eigPairs( fullSize );
  for ( int i = 0; i < fullSize; ++i ) { eigPairs[i] = { eig.eigenvalues[i], static_cast<std::size_t>( i ) }; }
  std::sort( eigPairs.rbegin(), eigPairs.rend() );

  // Eigenvectors are columns of eig.eigenvectors (column-major).
  for ( int j = 0; j < fullSize; ++j ) {
    const std::size_t src = eigPairs[j].second;
    for ( int i = 0; i < fullSize; ++i ) { klt.eigvec[j][i] = eig.eigenvectors[i + src * fullSize]; }
  }

  double scaleFactor = ( top10corr > 0.5 ) ? 1.25 : 2.25;
  for ( int j = 0; j < fullSize; ++j ) {
    for ( int i = 0; i < fullSize; ++i ) { klt.eigvec[j][i] *= scaleFactor; }
  }
  klt.scaleFactor = scaleFactor;

  return true;
}

bool PCCAdaptation::sphericalHarmonicsTransform2D( PCCPointSetGS &pointCloud, PCCAdaptationParameters &params ) {
  if ( !pointCloud.hasSphericalHarmonics() ) return false;
  const size_t pointCount = pointCloud.getPointCount();

  const double tr3x3[8][3][3] = {
      { { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 } },  // Identity matrix
      { { 0.29900, 0.58700, 0.11400 }, { -0.16900, -0.33100, 0.50000 }, { 0.50000, -0.41900, -0.08100 } },  // BT.601
      { { 0.33333, 0.33333, 0.33333 }, { 0.40825, 0.00000, -0.40825 }, { 0.23570, -0.47140, 0.23570 } },    // DCT-II
      { { 0.2126, 0.7152, 0.0722 }, { -0.114572, -0.385428, 0.5000 }, { 0.5000, -0.454153, -0.045847 } },   // BT.709
      { { 0.25, 0.5, 0.25 }, { 0.5, 0.0, -0.5 }, { -0.25, 0.5, -0.25 } },                                   // YCoCg
      { { -1.0, -1.0, -1.0 }, { -1.0, -1.0, -1.0 }, { -1.0, -1.0, -1.0 } },                                 // reserved
      { { -1.0, -1.0, -1.0 }, { -1.0, -1.0, -1.0 }, { -1.0, -1.0, -1.0 } },                                 // reserved
      { { -1.0, -1.0, -1.0 }, { -1.0, -1.0, -1.0 }, { -1.0, -1.0, -1.0 } } };                               // reserved

  const double tr2x2[3][2][2] = {
      { { 1.0, 0.0 }, { 0.0, 1.0 } }, { { 1.0, 0.0 }, { 0.0, 1.0 } }, { { 0.5, 0.5 }, { 0.5, -0.5 } } };

  std::vector<int> colTr( 16, 1 );
  for ( size_t gi = 0; gi < params.shTrGrpRow_.size(); ++gi ) {
    if ( params.shKltFlag_[gi] == 1 ) {
      for ( int idx : params.shTrGrpRow_[gi] ) { colTr[idx] = 0; }
    }
  }

  auto &shData = pointCloud.getSphericalHarmonics();

  // Column transform
  for ( size_t p = 0; p < pointCount; ++p ) {
    auto &sh = shData[p];
    for ( size_t i = 0; i < 16; ++i ) {
      if ( colTr[i] == 1 ) {
        int    type = params.shTrTypeCol_[i];
        double r = sh[i][0], g = sh[i][1], b = sh[i][2];
        sh[i][0] = tr3x3[type][0][0] * r + tr3x3[type][0][1] * g + tr3x3[type][0][2] * b;
        sh[i][1] = tr3x3[type][1][0] * r + tr3x3[type][1][1] * g + tr3x3[type][1][2] * b;
        sh[i][2] = tr3x3[type][2][0] * r + tr3x3[type][2][1] * g + tr3x3[type][2][2] * b;
      }
    }
  }

  // Row transform
  params.shKltParams_.clear();
  for ( size_t gi = 0; gi < params.shTrGrpRow_.size(); ++gi ) {
    const auto &groupidx = params.shTrGrpRow_[gi];
    int         size     = groupidx.size();
    if ( params.shKltFlag_[gi] == 1 ) {
      bool isDcGroup = ( groupidx.size() == 1 && groupidx[0] == 0 );

      if ( isDcGroup ) {
        // DC PCA path (m76858)
        PCCAdaptationParameters::PCCKltParameters klt;
        double                                    top10corr = computeShAcTop10Correlation( pointCloud );
        if ( !computeShDcPca( pointCloud, top10corr, klt ) ) continue;

        const int fullSize = 3;

        for ( size_t p = 0; p < pointCount; ++p ) {
          auto               &sh = shData[p];
          std::vector<double> X( fullSize );
          for ( int j = 0; j < fullSize; j++ ) { X[j] = (double)sh[0][j] - klt.mean[j]; }

          for ( int j = 0; j < fullSize; j++ ) {
            double y = 0.0;
            for ( int i = 0; i < fullSize; i++ ) { y += klt.eigvec[j][i] * X[i]; }
            sh[0][j] = static_cast<float>( y / ( klt.scaleFactor * klt.scaleFactor ) );
          }
        }

        params.shKltParams_.push_back( klt );
      } else {
        PCCAdaptationParameters::PCCKltParameters klt;
        if ( !computeShKlt( pointCloud, groupidx, klt ) ) continue;

        int fullSize = size * 3;
        for ( size_t p = 0; p < pointCount; ++p ) {
          auto               &sh = shData[p];
          std::vector<double> X( fullSize );
          for ( int j = 0; j < fullSize; j++ ) {
            X[j] = ( (double)sh[groupidx[j / 3]][j % 3] - klt.mean[j] ) / klt.sigma[j];
          }

          for ( int j = 0; j < fullSize; j++ ) {
            double y = 0.0;
            for ( int i = 0; i < fullSize; i++ ) { y += klt.eigvec[j][i] * X[i]; }
            sh[groupidx[j / 3]][j % 3] = static_cast<float>( y / 17.5 );
          }
        }

        params.shKltParams_.push_back( klt );
      }
      for ( int idx : groupidx ) { params.shTrTypeCol_[idx] = -1; }
    } else {
      int type = params.shTrTypeRow_[gi];
      if ( groupidx.size() == 3 ) {
        for ( size_t p = 0; p < pointCount; ++p ) {
          auto &sh = shData[p];
          for ( int c = 0; c < 3; ++c ) {
            double v0 = sh[groupidx[0]][c], v1 = sh[groupidx[1]][c], v2 = sh[groupidx[2]][c];
            sh[groupidx[0]][c] = tr3x3[type][0][0] * v0 + tr3x3[type][0][1] * v1 + tr3x3[type][0][2] * v2;
            sh[groupidx[1]][c] = tr3x3[type][1][0] * v0 + tr3x3[type][1][1] * v1 + tr3x3[type][1][2] * v2;
            sh[groupidx[2]][c] = tr3x3[type][2][0] * v0 + tr3x3[type][2][1] * v1 + tr3x3[type][2][2] * v2;
          }
        }
      } else if ( groupidx.size() == 2 ) {
        for ( size_t p = 0; p < pointCount; ++p ) {
          auto &sh = shData[p];
          for ( int c = 0; c < 3; ++c ) {
            double v0 = sh[groupidx[0]][c], v1 = sh[groupidx[1]][c];
            sh[groupidx[0]][c] = tr2x2[type][0][0] * v0 + tr2x2[type][0][1] * v1;
            sh[groupidx[1]][c] = tr2x2[type][1][0] * v0 + tr2x2[type][1][1] * v1;
          }
        }
      }
    }
  }
  return true;
}

bool PCCAdaptation::sphericalHarmonicsInvTransform2D( PCCPointSetGS                 &pointCloud,
                                                      const PCCAdaptationParameters &params ) {
  if ( !pointCloud.hasSphericalHarmonics() ) return false;
  const size_t pointCount = pointCloud.getPointCount();

  const double itr3x3[8][3][3] = {
      { { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 } },  // Identity matrix
      { { 1.00000, 0.00000, 1.40200 }, { 1.00000, -0.34000, -0.71400 }, { 1.00000, 1.77200, 0.00000 } },  // BT.601
      { { 1.00000, 1.22474, 0.70711 }, { 1.00000, 0.00000, -1.41421 }, { 1.00000, -1.22474, 0.70711 } },  // DCT-II
      { { 1.00000, 0.00000, 1.57480 }, { 1.00000, -0.18733, -0.46813 }, { 1.0000, 1.85563, 0.000000 } },  // BT.709
      { { 1.00000, 1.00000, -1.0000 }, { 1.00000, 0.000000, 1.000000 }, { 1.0000, -1.0000, -1.00000 } },  // YCoCg
      { { -1.0, -1.0, -1.0 }, { -1.0, -1.0, -1.0 }, { -1.0, -1.0, -1.0 } },                               // reserved
      { { -1.0, -1.0, -1.0 }, { -1.0, -1.0, -1.0 }, { -1.0, -1.0, -1.0 } },                               // reserved
      { { -1.0, -1.0, -1.0 }, { -1.0, -1.0, -1.0 }, { -1.0, -1.0, -1.0 } } };                             // reserved

  const double itr2x2[3][2][2] = {
      { { 1.0, 0.0 }, { 0.0, 1.0 } }, { { 1.0, 0.0 }, { 0.0, 1.0 } }, { { 1.0, 1.0 }, { 1.0, -1.0 } } };

  std::vector<int> colTr( 16, 1 );
  for ( size_t i = 0; i < params.shTrGrpRow_.size(); ++i ) {
    if ( params.shKltFlag_[i] == 1 ) {
      for ( int idx : params.shTrGrpRow_[i] ) { colTr[idx] = 0; }
    }
  }

  auto &shData = pointCloud.getSphericalHarmonics();
  int   ki     = 0;

  for ( size_t i = 0; i < params.shTrGrpRow_.size(); ++i ) {
    const auto &groupidx = params.shTrGrpRow_[i];
    int         size     = groupidx.size();
    if ( params.shKltFlag_[i] == 1 ) {
      const auto &klt       = params.shKltParams_[ki++];
      int         fullSize  = size * 3;
      bool        isDcGroup = ( groupidx.size() == 1 && groupidx[0] == 0 );

      if ( isDcGroup ) {
        // DC PCA inverse path (m76858)
        const int dcSize = 3;
        for ( size_t p = 0; p < pointCount; ++p ) {
          auto               &sh = shData[p];
          std::vector<double> Y( dcSize );
          for ( int j = 0; j < dcSize; j++ ) { Y[j] = sh[0][j]; }

          for ( int i_ = 0; i_ < dcSize; i_++ ) {
            double x_norm = 0.0;
            for ( int j = 0; j < dcSize; j++ ) { x_norm += klt.eigvec[j][i_] * Y[j]; }
            sh[0][i_] = static_cast<float>( x_norm + klt.mean[i_] );
          }
        }
      } else {
        for ( size_t p = 0; p < pointCount; ++p ) {
          auto               &sh = shData[p];
          std::vector<double> Y( fullSize );
          for ( int j = 0; j < fullSize; j++ ) { Y[j] = sh[groupidx[j / 3]][j % 3]; }

          for ( int i_ = 0; i_ < fullSize; i_++ ) {
            double x_norm = 0.0;
            for ( int j = 0; j < fullSize; j++ ) { x_norm += klt.eigvec[j][i_] * Y[j] * 17.5; }
            sh[groupidx[i_ / 3]][i_ % 3] = static_cast<float>( x_norm * klt.sigma[i_] + klt.mean[i_] );
          }
        }
      }
    } else {
      int type = params.shTrTypeRow_[i];
      if ( groupidx.size() == 3 ) {
        for ( size_t p = 0; p < pointCount; ++p ) {
          auto &sh = shData[p];
          for ( int c = 0; c < 3; ++c ) {
            double v0 = sh[groupidx[0]][c], v1 = sh[groupidx[1]][c], v2 = sh[groupidx[2]][c];
            sh[groupidx[0]][c] = itr3x3[type][0][0] * v0 + itr3x3[type][0][1] * v1 + itr3x3[type][0][2] * v2;
            sh[groupidx[1]][c] = itr3x3[type][1][0] * v0 + itr3x3[type][1][1] * v1 + itr3x3[type][1][2] * v2;
            sh[groupidx[2]][c] = itr3x3[type][2][0] * v0 + itr3x3[type][2][1] * v1 + itr3x3[type][2][2] * v2;
          }
        }
      } else if ( groupidx.size() == 2 ) {
        for ( size_t p = 0; p < pointCount; ++p ) {
          auto &sh = shData[p];
          for ( int c = 0; c < 3; ++c ) {
            double v0 = sh[groupidx[0]][c], v1 = sh[groupidx[1]][c];
            sh[groupidx[0]][c] = itr2x2[type][0][0] * v0 + itr2x2[type][0][1] * v1;
            sh[groupidx[1]][c] = itr2x2[type][1][0] * v0 + itr2x2[type][1][1] * v1;
          }
        }
      }
    }
  }

  // Inverse Column transform
  for ( size_t p = 0; p < pointCount; ++p ) {
    auto &sh = shData[p];
    for ( size_t i = 0; i < 16; ++i ) {
      if ( colTr[i] == 1 ) {
        int    type = params.shTrTypeCol_[i];
        double y = sh[i][0], u = sh[i][1], v = sh[i][2];
        sh[i][0] = itr3x3[type][0][0] * y + itr3x3[type][0][1] * u + itr3x3[type][0][2] * v;
        sh[i][1] = itr3x3[type][1][0] * y + itr3x3[type][1][1] * u + itr3x3[type][1][2] * v;
        sh[i][2] = itr3x3[type][2][0] * y + itr3x3[type][2][1] * u + itr3x3[type][2][2] * v;
      }
    }
  }
  return true;
}

bool PCCAdaptation::extractSHQuantRegions( const PCCPointSetGS &input, PCCAdaptationParameters &params ) {
  if ( !params.enableShNonUniformQuantization_ ) {
    params.shNumRegionsMinus1_.assign( size_t{ 16 } * 3, 0 );
    params.shTransitionPoints_.clear();
    params.shStepSizes_.clear();
    params.shNumBitsForTransitionPoint_.assign( size_t{ 16 } * 3, 10 );
    return true;
  }

  const size_t pointCount = input.getPointCount();
  if ( pointCount == 0 || !input.hasSphericalHarmonics() ) return false;

  const size_t   shCoefCount   = input.getSphericalHarmonicsCount();
  const uint32_t bitDepth      = params.shBitDepth_;
  const uint32_t max_quant_val = ( 1u << bitDepth ) - 1;

  params.shNumRegionsMinus1_.assign( size_t{ 16 } * 3, 2 );
  params.shTransitionPoints_.clear();
  params.shStepSizes_.assign( size_t{ 16 } * 9, 0.0 );
  for ( size_t i = 0; i < 16; ++i ) {
    params.shStepSizes_[i * 9 + 0] = 0.015625;
    params.shStepSizes_[i * 9 + 1] = 0.0078125;
    params.shStepSizes_[i * 9 + 2] = 0.015625;
    params.shStepSizes_[i * 9 + 3] = 0.015625;
    params.shStepSizes_[i * 9 + 4] = 0.0078125;
    params.shStepSizes_[i * 9 + 5] = 0.015625;
    params.shStepSizes_[i * 9 + 6] = 0.015625;
    params.shStepSizes_[i * 9 + 7] = 0.0078125;
    params.shStepSizes_[i * 9 + 8] = 0.015625;
  }
  params.shNumBitsForTransitionPoint_.assign( size_t{ 16 } * 3, 10 );

  double       sum1 = 0.0, sumSq1 = 0.0;
  double       sum3 = 0.0, sumSq3 = 0.0;
  const int    idx3_arr[3] = { 11, 13, 12 };
  const size_t count1      = pointCount * 9 * 3;
  const size_t count3      = pointCount * 3 * 3;

  for ( size_t p = 0; p < pointCount; ++p ) {
    for ( size_t l = 1; l <= 9; ++l ) {
      for ( int ch = 0; ch < 3; ++ch ) {
        double val = input.getSphericalHarmonics()[p][l][ch];
        sum1 += val;
        sumSq1 += val * val;
      }
    }
    for ( int i = 0; i < 3; ++i ) {
      size_t l = idx3_arr[i];
      for ( int ch = 0; ch < 3; ++ch ) {
        double val = input.getSphericalHarmonics()[p][l][ch];
        sum3 += val;
        sumSq3 += val * val;
      }
    }
  }

  double mean1 = sum1 / count1;
  double mean3 = sum3 / count3;
  double var1  = ( sumSq1 / count1 ) - ( mean1 * mean1 );
  double var3  = ( sumSq3 / count3 ) - ( mean3 * mean3 );
  double std1  = std::sqrt( std::max( 0.0, var1 ) );
  double std3  = std::sqrt( std::max( 0.0, var3 ) );

  int    thresh1, thresh2, thresh3, thresh4, thresh5;
  double stdRatio = std3 > 1e-10 ? std1 / std3 : 0.0;
  if ( stdRatio > 2.0 && stdRatio < 10.0 ) {
    thresh1 = 10;
    thresh2 = 15;
    thresh3 = 20;
    thresh4 = 25;
    thresh5 = 30;
  } else {
    thresh1 = 15;
    thresh2 = 30;
    thresh3 = 40;
    thresh4 = 5;
    thresh5 = 10;
  }

  const int idx2_arr[3] = { 10, 14, 15 };

  for ( size_t l = 0; l < params.shCoefficientCount_; ++l ) {
    for ( int ch = 0; ch < 3; ++ch ) {
      double perc_low, perc_high;
      if ( l == 0 ) {
        if ( ch == 0 ) {
          perc_low  = thresh4;
          perc_high = 100 - thresh4;
        } else {
          perc_low  = thresh5;
          perc_high = 100 - thresh5;
        }
      } else if ( l >= 1 && l <= 9 ) {
        perc_low  = thresh1;
        perc_high = 100 - thresh1;
      } else if ( l == static_cast<size_t>( idx2_arr[0] ) || l == static_cast<size_t>( idx2_arr[1] ) ||
                  l == static_cast<size_t>( idx2_arr[2] ) ) {
        perc_low  = thresh2;
        perc_high = 100 - thresh2;
      } else if ( l == static_cast<size_t>( idx3_arr[0] ) || l == static_cast<size_t>( idx3_arr[1] ) ||
                  l == static_cast<size_t>( idx3_arr[2] ) ) {
        perc_low  = thresh3;
        perc_high = 100 - thresh3;
      } else {
        perc_low  = 25.0;
        perc_high = 75.0;
      }

      std::vector<double> values;
      values.reserve( pointCount );
      for ( size_t p = 0; p < pointCount; ++p ) { values.push_back( input.getSphericalHarmonics()[p][l][ch] ); }

      size_t lowIdx  = static_cast<size_t>( std::round( perc_low / 100.0 * pointCount ) );
      size_t highIdx = static_cast<size_t>( std::round( perc_high / 100.0 * pointCount ) );
      lowIdx         = std::min( lowIdx, pointCount > 0 ? pointCount - 1 : 0 );
      highIdx        = std::min( highIdx, pointCount > 0 ? pointCount - 1 : 0 );

      std::nth_element( values.begin(), values.begin() + lowIdx, values.end() );
      double tp_low = values[lowIdx];

      std::nth_element( values.begin(), values.begin() + highIdx, values.end() );
      double tp_high = values[highIdx];

      double minLim = params.shLimitMin_[l][ch];
      double maxLim = params.shLimitMax_[l][ch];

      int tp_low_quant  = static_cast<int>( std::round( ( tp_low - minLim ) / ( maxLim - minLim ) * max_quant_val ) );
      int tp_high_quant = static_cast<int>( std::round( ( tp_high - minLim ) / ( maxLim - minLim ) * max_quant_val ) );

      params.shTransitionPoints_.push_back(
          std::max( 0, std::min( static_cast<int>( max_quant_val ), tp_low_quant ) ) );
      params.shTransitionPoints_.push_back(
          std::max( 0, std::min( static_cast<int>( max_quant_val ), tp_high_quant ) ) );
    }
  }

  return true;
}

bool PCCAdaptation::extractOpacityQuantRegions( const PCCPointSetGS &input, PCCAdaptationParameters &params ) {
  if ( !params.enableOpacityNonUniformQuantization_ ) {
    params.opacityNumRegionsMinus1_ = 0;
    params.opacityTransitionPoints_.clear();
    params.opacityStepSizes_.clear();
    params.opacityNumBitsForTransitionPoint_ = 0;
    return true;
  }

  const size_t pointCount = input.getPointCount();
  if ( pointCount == 0 || !input.hasOpacities() ) return false;

  // Auto-compute limits if enabled (Skip if non-uniform quantization is enabled, as it's already computed safely in
  // extractOpacityQuantRegions)

  if ( params.autoComputeOpacityLimits_ ) {
    params.computedOpacityMin_ = std::numeric_limits<double>::max();
    params.computedOpacityMax_ = std::numeric_limits<double>::lowest();
    for ( size_t i = 0; i < pointCount; ++i ) {
      double opacity             = input.getOpacities()[i];
      params.computedOpacityMin_ = std::min( params.computedOpacityMin_, opacity );
      params.computedOpacityMax_ = std::max( params.computedOpacityMax_, opacity );
    }
  } else {
    params.computedOpacityMin_ = params.opacityLimitMin_;
    params.computedOpacityMax_ = params.opacityLimitMax_;
  }

  params.opacityNumRegionsMinus1_ = 2;
  params.opacityTransitionPoints_.clear();
  params.opacityStepSizes_.assign( 3, 0.0 );
  params.opacityNumBitsForTransitionPoint_ = 10;

  double t1       = 0.45;
  double t2       = 0.55;
  double region_1 = std::log( t1 / ( 1.0 - t1 ) );
  double region_2 = std::log( t2 / ( 1.0 - t2 ) );

  const uint32_t bitDepth     = params.opacityBitDepth_;
  const uint32_t total_levels = 1u << bitDepth;
  uint32_t       n1           = total_levels / 8;
  uint32_t       n3           = n1;
  uint32_t       n2           = total_levels - n1 - n3;

  params.opacityStepSizes_[0] = static_cast<float>( std::max( 1e-9, ( region_1 - params.computedOpacityMin_ ) / n1 ) );
  params.opacityStepSizes_[1] = static_cast<float>( std::max( 1e-9, ( region_2 - region_1 ) / n2 ) );
  params.opacityStepSizes_[2] = static_cast<float>( std::max( 1e-9, ( params.computedOpacityMax_ - region_2 ) / n3 ) );

  params.opacityTransitionPoints_.push_back( n1 );
  params.opacityTransitionPoints_.push_back( n1 + n2 );

  return true;
}

// Extract the non-uniform quantization params for scale
// Default config: 3 regions, [min, -10] step_size_base * 8, [-10, -7] step_size_base * 2, [-7, max] step_size_base
bool PCCAdaptation::extractScaleQuantRegions( const PCCPointSetGS &input, PCCAdaptationParameters &params ) {
  if ( !params.enableScaleNonUniformQuantization_ ) {
    params.scaleNumRegionsMinus1_.assign( 3, 0 );
    params.scaleTransitionPoints_.clear();
    params.scaleStepSizes_.clear();
    params.scaleNumBitsForTransitionPoint_.assign( 3, 10 );
    return true;
  }

  const size_t pointCount = input.getPointCount();
  if ( pointCount == 0 || !input.hasScales() ) return false;

  const uint32_t bitDepth      = params.scaleBitDepth_;
  const uint32_t max_quant_val = ( 1u << bitDepth ) - 1;

  params.scaleNumRegionsMinus1_.assign( 3, 2 );
  params.scaleTransitionPoints_.clear();
  params.scaleStepSizes_.assign( size_t{ 3 } * 3, 0.0 );
  for ( size_t ch = 0; ch < 3; ++ch ) {
    double step_size_base              = ( params.scaleLimitMax_[ch] - params.scaleLimitMin_[ch] ) / max_quant_val;
    params.scaleStepSizes_[ch * 3 + 0] = step_size_base * 8;
    params.scaleStepSizes_[ch * 3 + 1] = step_size_base * 2;
    params.scaleStepSizes_[ch * 3 + 2] = step_size_base;
  }
  params.scaleNumBitsForTransitionPoint_.assign( 3, 10 );

  for ( size_t ch = 0; ch < 3; ++ch ) {
    std::vector<double> ranges            = { params.scaleLimitMin_[ch], -10, -7, params.scaleLimitMax_[ch] };
    int                 cumulative_offset = 0;
    for ( size_t r = 0; r < 2; ++r ) {
      cumulative_offset +=
          static_cast<int>( std::round( ( ranges[r + 1] - ranges[r] ) / params.scaleStepSizes_[ch * 3 + r] ) );
      params.scaleTransitionPoints_.push_back( cumulative_offset );
    }
  }

  return true;
}

//============================================================================
// PCCAdaptation - Helper Functions
//============================================================================

double PCCAdaptation::normalizeToUnit( double value, double min, double max ) {
  if ( max <= min ) return 0.0;
  double normalized = ( value - min ) / ( max - min );
  return std::max( 0.0, std::min( 1.0, normalized ) );  // Clamp to [0, 1]
}

double PCCAdaptation::denormalizeFromUnit( double value, double min, double max ) {
  return value * ( max - min ) + min;
}

uint32_t PCCAdaptation::quantizeValue( double normalizedValue, uint32_t bitDepth ) {
  uint32_t maxVal    = ( 1u << bitDepth ) - 1;
  double   quantized = std::round( normalizedValue * static_cast<double>( maxVal ) );
  return static_cast<uint32_t>( std::max( 0.0, std::min( static_cast<double>( maxVal ), quantized ) ) );
}

double PCCAdaptation::dequantizeValue( uint32_t quantizedValue, uint32_t bitDepth ) {
  uint32_t maxVal = ( 1u << bitDepth ) - 1;
  return static_cast<double>( quantizedValue ) / static_cast<double>( maxVal );
}

void PCCAdaptation::canonicalizeQuaternion( float &w, float &x, float &y, float &z ) {
  float norm = std::sqrt( w * w + x * x + y * y + z * z );
  if ( norm < 1e-10f ) {
    // Degenerate quaternion - set to identity
    w = 1.0f;
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
    return;
  }

  // Normalize
  w /= norm;
  x /= norm;
  y /= norm;
  z /= norm;

  // Ensure w >= 0 (canonical form)
  if ( w < 0.0f ) {
    w = -w;
    x = -x;
    y = -y;
    z = -z;
  }
}

void PCCAdaptation::quatToRotMat( const GSRotation &q, PCCMatrix3D &R ) {
  double w = q.w, x = q.x, y = q.y, z = q.z;
  R[0][0] = 1.0 - 2.0 * ( y * y + z * z );
  R[0][1] = 2.0 * ( x * y - w * z );
  R[0][2] = 2.0 * ( x * z + w * y );
  R[1][0] = 2.0 * ( x * y + w * z );
  R[1][1] = 1.0 - 2.0 * ( x * x + z * z );
  R[1][2] = 2.0 * ( y * z - w * x );
  R[2][0] = 2.0 * ( x * z - w * y );
  R[2][1] = 2.0 * ( y * z + w * x );
  R[2][2] = 1.0 - 2.0 * ( x * x + y * y );
}

void PCCAdaptation::rotMatToQuat( const PCCMatrix3D &R, GSRotation &q ) {
  double t = R[0][0] + R[1][1] + R[2][2];
  if ( t > 0.0 ) {
    double s = std::sqrt( t + 1.0 ) * 2.0;
    q.w      = 0.25 * s;
    q.x      = ( R[2][1] - R[1][2] ) / s;
    q.y      = ( R[0][2] - R[2][0] ) / s;
    q.z      = ( R[1][0] - R[0][1] ) / s;
  } else if ( ( R[0][0] > R[1][1] ) && ( R[0][0] > R[2][2] ) ) {
    double s = std::sqrt( 1.0 + R[0][0] - R[1][1] - R[2][2] ) * 2.0;
    q.w      = ( R[2][1] - R[1][2] ) / s;
    q.x      = 0.25 * s;
    q.y      = ( R[0][1] + R[1][0] ) / s;
    q.z      = ( R[0][2] + R[2][0] ) / s;
  } else if ( R[1][1] > R[2][2] ) {
    double s = std::sqrt( 1.0 + R[1][1] - R[0][0] - R[2][2] ) * 2.0;
    q.w      = ( R[0][2] - R[2][0] ) / s;
    q.x      = ( R[0][1] + R[1][0] ) / s;
    q.y      = 0.25 * s;
    q.z      = ( R[1][2] + R[2][1] ) / s;
  } else {
    double s = std::sqrt( 1.0 + R[2][2] - R[0][0] - R[1][1] ) * 2.0;
    q.w      = ( R[1][0] - R[0][1] ) / s;
    q.x      = ( R[0][2] + R[2][0] ) / s;
    q.y      = ( R[1][2] + R[2][1] ) / s;
    q.z      = 0.25 * s;
  }
  q.normalize();
}

void PCCAdaptation::rgbToYuv( double r, double g, double b, double &y, double &u, double &v ) {
  // BT.601 conversion (matches Python gs_quantize.py)
  y = 0.29900 * r + 0.58700 * g + 0.11400 * b;
  u = -0.14713 * r - 0.28886 * g + 0.43600 * b;
  v = 0.61500 * r - 0.51498 * g - 0.10001 * b;
}

void PCCAdaptation::yuvToRgb( double y, double u, double v, double &r, double &g, double &b ) {
  // BT.601 inverse conversion (matches Python gs_quantize.py)
  r = y + 1.13983 * v;
  g = y - 0.39465 * u - 0.58060 * v;
  b = y + 2.03211 * u;
}

void PCCAdaptation::computeBoundingBox( const PCCPointSetGS   &pointCloud,
                                        std::array<double, 3> &minPoint,
                                        std::array<double, 3> &maxPoint ) {
  const size_t pointCount = pointCloud.getPointCount();
  if ( pointCount == 0 ) {
    minPoint = { 0.0, 0.0, 0.0 };
    maxPoint = { 0.0, 0.0, 0.0 };
    return;
  }

  for ( int k = 0; k < 3; ++k ) {
    minPoint[k] = std::numeric_limits<double>::max();
    maxPoint[k] = std::numeric_limits<double>::lowest();
  }

  for ( size_t i = 0; i < pointCount; ++i ) {
    const PCCPointGS &pt = pointCloud[i];
    for ( int k = 0; k < 3; ++k ) {
      minPoint[k] = std::min( minPoint[k], static_cast<double>( pt[k] ) );
      maxPoint[k] = std::max( maxPoint[k], static_cast<double>( pt[k] ) );
    }
  }
}

void PCCAdaptation::computeRange( const std::vector<double> &values, double &minVal, double &maxVal ) {
  if ( values.empty() ) {
    minVal = 0.0;
    maxVal = 0.0;
    return;
  }

  minVal = std::numeric_limits<double>::max();
  maxVal = std::numeric_limits<double>::lowest();

  for ( double v : values ) {
    minVal = std::min( minVal, v );
    maxVal = std::max( maxVal, v );
  }
}

//============================================================================
// Spherical Harmonics Reordering
//============================================================================

bool PCCAdaptation::reorderSphericalHarmonicsAC( PCCPointSetGS &pointCloud, bool toInterleaved ) {
  if ( !pointCloud.hasSphericalHarmonics() ) {
    std::cerr << "Error: Point cloud has no spherical harmonics to reorder." << std::endl;
    return false;
  }

  const size_t pointCount  = pointCloud.getPointCount();
  const size_t shCoefCount = pointCloud.getSphericalHarmonicsCount();

  if ( shCoefCount <= 1 ) {
    // Only DC component, no AC coefficients to reorder
    return true;
  }

  auto        &sphericalHarmonics = pointCloud.getSphericalHarmonics();
  const size_t acCount            = shCoefCount - 1;  // Number of AC coefficients (excluding DC)

  // Temporary storage for reordering
  GSSphericalHarmonics temp;

  for ( size_t p = 0; p < pointCount; ++p ) {
    auto &sh = sphericalHarmonics[p];

    // Clear temp
    for ( size_t i = 0; i < GSSphericalHarmonics::MAX_SH_COEFFS; ++i ) {
      temp[i][0] = 0.0f;
      temp[i][1] = 0.0f;
      temp[i][2] = 0.0f;
    }

    // DC always stays at position 0
    temp[0][0] = sh[0][0];  // DC_R
    temp[0][1] = sh[0][1];  // DC_G
    temp[0][2] = sh[0][2];  // DC_B

    if ( toInterleaved ) {
      for ( size_t j = 0; j < acCount; ++j ) {  // For each AC coefficient index
        for ( size_t c = 0; c < 3; ++c ) {      // For each color channel
          auto   n = j * 3 + c;
          size_t a = n % ( acCount ) + 1;
          size_t b = n / ( acCount );

          temp[a][b] = sh[j + 1][c];
        }
      }
    } else {
      for ( size_t j = 0; j < acCount; ++j ) {  // For each AC coefficient index
        for ( size_t c = 0; c < 3; ++c ) {      // For each color channel

          size_t a = j / 3 + 1 + c * 5;
          size_t b = j % 3;

          temp[a][b] = sh[j + 1][c];
        }
      }
    }

    // Copy back
    for ( size_t i = 0; i < shCoefCount; ++i ) {
      sh[i][0] = temp[i][0];
      sh[i][1] = temp[i][1];
      sh[i][2] = temp[i][2];
    }
  }

  return true;
}

bool PCCAdaptation::reorderSphericalHarmonicsAC( PCCPointSet3 &pointCloud, bool toInterleaved ) {
  if ( !pointCloud.hasSphericalHarmonics() ) {
    std::cerr << "Error: Point cloud has no spherical harmonics to reorder." << std::endl;
    return false;
  }

  const size_t pointCount  = pointCloud.getPointCount();
  const size_t shCoefCount = 16;  // Assuming 16 coefficients as default, can be made dynamic if needed

  if ( shCoefCount <= 1 ) {
    // Only DC component, no AC coefficients to reorder
    return true;
  }

  auto        &sphericalHarmonics = pointCloud.getSphericalHarmonics();
  const size_t acCount            = shCoefCount - 1;  // Number of AC coefficients (excluding DC)

  // Temporary storage for reordering
  GSSphericalHarmonics temp;

  for ( size_t p = 0; p < pointCount; ++p ) {
    auto &sh = sphericalHarmonics[p];

    // Clear temp
    for ( size_t i = 0; i < GSSphericalHarmonics::MAX_SH_COEFFS; ++i ) {
      temp[i][0] = 0.0f;
      temp[i][1] = 0.0f;
      temp[i][2] = 0.0f;
    }

    // DC always stays at position 0
    temp[0][0] = sh[0][0];  // DC_R
    temp[0][1] = sh[0][1];  // DC_G
    temp[0][2] = sh[0][2];  // DC_B

    if ( toInterleaved ) {
      for ( size_t j = 0; j < acCount; ++j ) {  // For each AC coefficient index
        for ( size_t c = 0; c < 3; ++c ) {      // For each color channel
          auto   n = j * 3 + c;
          size_t a = n % ( acCount ) + 1;
          size_t b = n / ( acCount );

          temp[a][b] = sh[j + 1][c];
        }
      }
    } else {
      for ( size_t j = 0; j < acCount; ++j ) {  // For each AC coefficient index
        for ( size_t c = 0; c < 3; ++c ) {      // For each color channel

          size_t a = j / 3 + 1 + c * 5;
          size_t b = j % 3;

          temp[a][b] = sh[j + 1][c];
        }
      }
    }

    // Copy back
    for ( size_t i = 0; i < shCoefCount; ++i ) {
      sh[i][0] = temp[i][0];
      sh[i][1] = temp[i][1];
      sh[i][2] = temp[i][2];
    }
  }

  return true;
}

//============================================================================
// Spherical Harmonics Adaptive Reordering (Mode 2)
//============================================================================

bool PCCAdaptation::reorderSphericalHarmonicsAdaptive( PCCPointSet3                  &pointCloud,
                                                       const PCCAdaptationParameters &params,
                                                       bool                           toInterleaved ) {
  if ( !pointCloud.hasSphericalHarmonics() ) return false;

  const size_t pointCount  = pointCloud.getPointCount();
  const size_t shCoefCount = 16;
  const size_t numACCoeff  = 15;

  auto                &sphericalHarmonics = pointCloud.getSphericalHarmonics();
  GSSphericalHarmonics temp;

  std::array<int, 45> invMappingTable;
  invMappingTable.fill( 0 );
  for ( int i = 0; i < 45; ++i ) { invMappingTable[params.shMappingTable_[i]] = i; }

  for ( size_t p = 0; p < pointCount; ++p ) {
    auto &sh = sphericalHarmonics[p];

    for ( size_t i = 0; i < GSSphericalHarmonics::MAX_SH_COEFFS; ++i ) {
      temp[i][0] = 0.0f;
      temp[i][1] = 0.0f;
      temp[i][2] = 0.0f;
    }

    temp[0][0] = sh[0][0];
    temp[0][1] = sh[0][1];
    temp[0][2] = sh[0][2];

    if ( toInterleaved ) {
      for ( int p = 1; p < 16; ++p ) {
        for ( int c = 0; c < 3; ++c ) {
          int bitstreamPos = ( p - 1 ) * 3 + c;
          int planarPos    = params.shMappingTable_[bitstreamPos];
          int level        = 1 + ( planarPos % numACCoeff );
          int channel      = planarPos / numACCoeff;
          temp[p][c]       = sh[level][channel];
        }
      }
    } else {
      for ( int l = 1; l < 16; ++l ) {
        for ( int ch = 0; ch < 3; ++ch ) {
          int planarPos     = ch * numACCoeff + ( l - 1 );
          int bitstreamPos  = invMappingTable[planarPos];
          int partition     = bitstreamPos / 3 + 1;
          int channelInPart = bitstreamPos % 3;
          temp[l][ch]       = sh[partition][channelInPart];
        }
      }
    }

    for ( size_t i = 0; i < shCoefCount; ++i ) {
      sh[i][0] = temp[i][0];
      sh[i][1] = temp[i][1];
      sh[i][2] = temp[i][2];
    }
  }

  return true;
}

bool PCCAdaptation::reorderSphericalHarmonicsAdaptive( PCCPointSetGS                 &pointCloud,
                                                       const PCCAdaptationParameters &params,
                                                       bool                           toInterleaved ) {
  if ( !pointCloud.hasSphericalHarmonics() ) return false;

  const size_t pointCount  = pointCloud.getPointCount();
  const size_t shCoefCount = pointCloud.getSphericalHarmonicsCount();
  const size_t numACCoeff  = shCoefCount > 0 ? shCoefCount - 1 : 15;
  const int    numAC       = static_cast<int>( numACCoeff );

  auto                &sphericalHarmonics = pointCloud.getSphericalHarmonics();
  GSSphericalHarmonics temp;

  std::array<int, 45> invMappingTable;
  invMappingTable.fill( 0 );
  for ( int i = 0; i < 45; ++i ) { invMappingTable[params.shMappingTable_[i]] = i; }

  for ( size_t p = 0; p < pointCount; ++p ) {
    auto &sh = sphericalHarmonics[p];

    for ( size_t i = 0; i < GSSphericalHarmonics::MAX_SH_COEFFS; ++i ) {
      temp[i][0] = 0.0f;
      temp[i][1] = 0.0f;
      temp[i][2] = 0.0f;
    }

    temp[0][0] = sh[0][0];
    temp[0][1] = sh[0][1];
    temp[0][2] = sh[0][2];

    if ( toInterleaved ) {
      for ( int pIdx = 1; pIdx < static_cast<int>( shCoefCount ); ++pIdx ) {
        for ( int c = 0; c < 3; ++c ) {
          int bitstreamPos = ( pIdx - 1 ) * 3 + c;
          int planarPos    = params.shMappingTable_[bitstreamPos];
          int level        = 1 + ( planarPos % numAC );
          int channel      = planarPos / numAC;
          temp[pIdx][c]    = sh[level][channel];
        }
      }
    } else {
      for ( int l = 1; l < static_cast<int>( shCoefCount ); ++l ) {
        for ( int ch = 0; ch < 3; ++ch ) {
          int planarPos     = ch * numAC + ( l - 1 );
          int bitstreamPos  = invMappingTable[planarPos];
          int partition     = bitstreamPos / 3 + 1;
          int channelInPart = bitstreamPos % 3;
          temp[l][ch]       = sh[partition][channelInPart];
        }
      }
    }

    for ( size_t i = 0; i < shCoefCount; ++i ) {
      sh[i][0] = temp[i][0];
      sh[i][1] = temp[i][1];
      sh[i][2] = temp[i][2];
    }
  }

  return true;
}

//============================================================================
// Adaptive SH Quantization
//============================================================================

bool computeGroupIdsFromImportanceScores( const std::string &importanceScoresPath,
                                          const size_t       pointCount,
                                          std::vector<int>  &groupIds,
                                          uint32_t           frameNumber ) {
  char fileName[512];
  snprintf( fileName, sizeof( fileName ), importanceScoresPath.c_str(), frameNumber );
  std::cout << "Computing quantization groups from importance score file: " << fileName << std::endl;
  std::ifstream ifs( fileName );
  if ( !ifs.is_open() ) {
    std::cerr << "PCCAdaptation::computeGroupIdsFromImportanceScores - Cannot open file: " << fileName << std::endl;
    return false;
  }
  std::vector<double> impScores;
  impScores.reserve( pointCount );

  std::string line;
  while ( std::getline( ifs, line ) ) {
    if ( line.find_first_not_of( " \t\r\n" ) == std::string::npos ) { continue; }
    impScores.push_back( std::stod( line ) );
  }

  if ( impScores.size() != pointCount ) {
    std::cerr << "PCCAdaptation::computeGroupIdsFromImportanceScores - Score count mismatch. " << pointCount
              << " != " << impScores.size() << std::endl;
    return false;
  }

  const double eps = 1e-18;
  for ( auto &score : impScores ) {
    if ( score < eps ) { score = eps; }
  }
  const double maxScore = *std::max_element( impScores.begin(), impScores.end() );
  const double c        = 0.5 * std::log( maxScore );

  groupIds.resize( impScores.size() );
  for ( size_t i = 0; i < impScores.size(); ++i ) {
    const double group = c - 0.5 * std::log( impScores[i] );
    groupIds[i]        = std::max( 0, static_cast<int>( std::round( group ) ) );
  }

  return true;
}

bool PCCAdaptation::pruneAndGroupAdaptiveQuantization( const PCCPointSetGS     &inputPointCloud,
                                                       PCCPointSetGS           &outputPointCloud,
                                                       PCCAdaptationParameters &params,
                                                       uint32_t                 frameNumber ) const {
  if ( !params.enableAdaptiveShQuantization_ ) {
    outputPointCloud = inputPointCloud;
    return true;
  }

  const size_t inputPointCount = inputPointCloud.getPointCount();

  if ( params.importanceScoresPath_.empty() ) {
    std::cerr << "Error - Missing importanceScoresPath for adaptive SH quantization" << std::endl;
    return false;
  }
  if ( params.adaptiveShBitshifts_.empty() ) {
    std::cerr << "Error - Empty adaptiveShBitshifts mapping" << std::endl;
    return false;
  }

  std::vector<int> groupIds;
  if ( !computeGroupIdsFromImportanceScores( params.importanceScoresPath_, inputPointCount, groupIds, frameNumber ) ) {
    std::cerr << "Error - Failed to compute quantization groups from importance score file: "
              << params.importanceScoresPath_ << std::endl;
    return false;
  }

  // Remove empty groups and re-index
  std::vector<int> shBitshifts( params.adaptiveShBitshifts_.begin(), params.adaptiveShBitshifts_.end() );

  const int     maxGrp = *std::max_element( groupIds.begin(), groupIds.end() );
  std::set<int> usedGrps( groupIds.begin(), groupIds.end() );

  std::set<int> allGrps;
  for ( int g = 0; g <= maxGrp; ++g ) { allGrps.insert( g ); }

  std::vector<int> emptyGrps;
  for ( int g : allGrps ) {
    if ( usedGrps.find( g ) == usedGrps.end() ) { emptyGrps.push_back( g ); }
  }

  std::sort( emptyGrps.rbegin(), emptyGrps.rend() );
  for ( int g : emptyGrps ) {
    if ( static_cast<int>( shBitshifts.size() ) > g + 1 ) { shBitshifts.erase( shBitshifts.begin() + g ); }
    for ( auto &gid : groupIds ) {
      if ( gid > g ) { gid -= 1; }
    }
  }

  if ( shBitshifts.empty() ) {
    std::cerr << "Error - Empty SH bitshift mapping after removing empty groups" << std::endl;
    return false;
  }

  for ( auto &gid : groupIds ) { gid = std::max( 0, std::min( gid, static_cast<int>( shBitshifts.size() - 1 ) ) ); }

  // Prune gaussians with 0 SH bits
  std::vector<size_t> keepIndices;
  keepIndices.reserve( inputPointCount );

  const size_t          maxShBitDepth = params.shBitDepth_;
  std::vector<uint8_t>  isPrunedGroup( shBitshifts.size(), 0 );
  std::vector<uint16_t> groupIdsKept;
  groupIdsKept.reserve( inputPointCount );
  for ( size_t i = 0; i < inputPointCount; ++i ) {
    const int gid = groupIds[i];
    if ( isPrunedGroup[gid] ) { continue; }

    const int bitshift = shBitshifts[gid];
    int       bits     = static_cast<int>( maxShBitDepth ) - bitshift;
    bits               = std::max( 0, std::min( static_cast<int>( maxShBitDepth ), bits ) );

    if ( bits > 0 || params.keepZeroShBitGaussians_ ) {
      keepIndices.push_back( i );
      groupIdsKept.push_back( static_cast<uint16_t>( gid ) );
    } else {
      isPrunedGroup[gid] = 1;
    }
  }

  if ( keepIndices.empty() ) {
    std::cerr << "Error - Adaptive SH pruning removed all gaussians" << std::endl;
    return false;
  }

  // Merge same bitshifts and quantization groups and ignore pruned groups
  std::vector<int> mergedShBitshifts;
  std::vector<int> remapGroupIds( shBitshifts.size(), -1 );
  mergedShBitshifts.reserve( shBitshifts.size() );

  for ( size_t g = 0; g < shBitshifts.size(); ++g ) {
    if ( isPrunedGroup[g] ) { continue; }

    const int bitshift = shBitshifts[g];
    auto      it       = std::find( mergedShBitshifts.begin(), mergedShBitshifts.end(), bitshift );
    if ( it == mergedShBitshifts.end() ) {
      remapGroupIds[g] = static_cast<int>( mergedShBitshifts.size() );
      mergedShBitshifts.push_back( bitshift );
    } else {
      remapGroupIds[g] = static_cast<int>( std::distance( mergedShBitshifts.begin(), it ) );
    }
  }

  for ( auto &gid : groupIdsKept ) { gid = static_cast<uint16_t>( remapGroupIds[gid] ); }
  shBitshifts = mergedShBitshifts;
  // Update adaptive SH bitshift mapping
  outputPointCloud.getQuantizationGroupBitDepthDeltas() = shBitshifts;

  outputPointCloud.clear();
  if ( inputPointCloud.hasColors() ) { outputPointCloud.addColors(); }
  if ( inputPointCloud.hasRotations() ) { outputPointCloud.addRotations(); }
  if ( inputPointCloud.hasScales() ) { outputPointCloud.addScales(); }
  if ( inputPointCloud.hasOpacities() ) { outputPointCloud.addOpacities(); }
  if ( inputPointCloud.hasSphericalHarmonics() ) {
    outputPointCloud.addSphericalHarmonics();
    outputPointCloud.setSphericalHarmonicsCount( inputPointCloud.getSphericalHarmonicsCount() );
  }
  outputPointCloud.resize( keepIndices.size() );

  for ( size_t outIdx = 0; outIdx < keepIndices.size(); ++outIdx ) {
    const size_t inIdx       = keepIndices[outIdx];
    outputPointCloud[outIdx] = inputPointCloud[inIdx];

    if ( inputPointCloud.hasColors() ) { outputPointCloud.setColor( outIdx, inputPointCloud.getColor( inIdx ) ); }
    if ( inputPointCloud.hasRotations() ) {
      outputPointCloud.setRotation( outIdx, inputPointCloud.getRotation( inIdx ) );
    }
    if ( inputPointCloud.hasScales() ) { outputPointCloud.setScale( outIdx, inputPointCloud.getScale( inIdx ) ); }
    if ( inputPointCloud.hasOpacities() ) {
      outputPointCloud.setOpacity( outIdx, inputPointCloud.getOpacity( inIdx ) );
    }
    if ( inputPointCloud.hasSphericalHarmonics() ) {
      outputPointCloud.setSphericalHarmonic( outIdx, inputPointCloud.getSphericalHarmonic( inIdx ) );
    }
  }

  outputPointCloud.getQuantizationGroupIds() = groupIdsKept;

  return true;
}

namespace {

template <typename PointSetT>
size_t getAdaptiveBitDepth( const uint32_t maxBitDepth, const PointSetT &input, size_t pointIndex ) {
  if ( !input.hasQuantizationGroupIds() ) {
    std::cerr << "PCCAdaptation::getAdaptiveBitDepth - missing quantization group IDs" << std::endl;
    return 0;
  }
  if ( input.getQuantizationGroupBitDepthDeltas().empty() ) {
    std::cerr << "PCCAdaptation::getAdaptiveBitDepth - empty adaptive bitshift mapping" << std::endl;
    return 0;
  }

  size_t groupId = input.getQuantizationGroupId( pointIndex );
  if ( groupId >= input.getQuantizationGroupBitDepthDeltas().size() ) {
    std::cerr << "PCCAdaptation::getAdaptiveBitDepth - invalid group id " << groupId << " at gaussian index "
              << pointIndex << std::endl;
    return 0;
  }
  int bits = static_cast<int>( maxBitDepth ) - input.getQuantizationGroupBitDepthDelta( groupId );
  bits     = std::max( 0, std::min( static_cast<int>( maxBitDepth ), bits ) );
  return static_cast<uint32_t>( bits );
}

}  // namespace

}  // namespace pcc
