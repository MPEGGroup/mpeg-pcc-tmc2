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

#include "PCCPointSetGS.h"
#include "PCCPointSet.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace pcc {

namespace {

void getTokens( const char *str, const char *sep, std::vector<std::string> &tokens ) {
  tokens.clear();
  std::string buf;
  size_t      i      = 0;
  size_t      length = strlen( str );
  while ( i < length ) {
    if ( strchr( sep, str[i] ) != nullptr ) {
      if ( !buf.empty() ) {
        tokens.push_back( buf );
        buf.clear();
      }
    } else {
      buf += str[i];
    }
    ++i;
  }
  if ( !buf.empty() ) { tokens.push_back( buf ); }
}

template <class T>
inline T readValue( std::ifstream &ifs ) {
  T value;
  ifs.read( reinterpret_cast<char *>( &value ), sizeof( T ) );
  return value;
}

}  // namespace

PCCPointGS PCCPointSetGS::computeCentroid() const {
  PCCPointGSD  centroid( 0.0, 0.0, 0.0 );
  const size_t pointCount = getPointCount();
  if ( pointCount == 0 ) return PCCPointGS();

  for ( const auto &position : positions_ ) {
    centroid.x += position.x;
    centroid.y += position.y;
    centroid.z += position.z;
  }
  centroid.x /= static_cast<double>( pointCount );
  centroid.y /= static_cast<double>( pointCount );
  centroid.z /= static_cast<double>( pointCount );
  return centroid.toFloat();
}

PCCBoxGS PCCPointSetGS::computeBoundingBox() const {
  PCCBoxGS box;
  for ( const auto &position : positions_ ) { box.update( position ); }
  return box;
}

bool PCCPointSetGS::read( const std::string &fileName ) {
  std::ifstream ifs( fileName, std::ifstream::in );
  if ( !ifs.is_open() ) {
    std::cerr << "Error: Cannot open file " << fileName << std::endl;
    return false;
  }

  enum AttributeType {
    ATTR_FLOAT64 = 0,
    ATTR_FLOAT32 = 1,
    ATTR_UINT64  = 2,
    ATTR_UINT32  = 3,
    ATTR_UINT16  = 4,
    ATTR_UINT8   = 5,
    ATTR_INT64   = 6,
    ATTR_INT32   = 7,
    ATTR_INT16   = 8,
    ATTR_INT8    = 9,
  };

  struct AttributeInfo {
    std::string   name;
    AttributeType type;
    size_t        byteCount;
  };

  std::vector<AttributeInfo> attributesInfo;
  attributesInfo.reserve( 64 );

  const size_t             MAX_BUFFER_SIZE = 100000;
  char                     tmp[MAX_BUFFER_SIZE];
  const char              *sep = " \t\r";
  std::vector<std::string> tokens;

  // Read PLY header
  ifs.getline( tmp, MAX_BUFFER_SIZE );
  getTokens( tmp, sep, tokens );
  if ( tokens.empty() || tokens[0] != "ply" ) {
    std::cerr << "Error: Not a valid PLY file!" << std::endl;
    return false;
  }

  bool                     isAscii          = false;
  double                   version          = 1.0;
  size_t                   pointCount       = 0;
  bool                     isVertexProperty = true;
  std::vector<std::string> cameraInfoHeader;

  while ( true ) {
    if ( ifs.eof() ) {
      std::cerr << "Error: Corrupted PLY header!" << std::endl;
      return false;
    }
    ifs.getline( tmp, MAX_BUFFER_SIZE );
    getTokens( tmp, sep, tokens );
    if ( tokens.empty() ) continue;

    if ( tokens[0] == "format" ) {
      if ( tokens.size() != 3 ) {
        std::cerr << "Error: Corrupted format info!" << std::endl;
        return false;
      }
      isAscii = ( tokens[1] == "ascii" );
      version = atof( tokens[2].c_str() );
    } else if ( tokens[0] == "element" ) {
      if ( tokens.size() != 3 ) {
        std::cerr << "Error: Corrupted element info!" << std::endl;
        return false;
      }
      if ( tokens[1] == "vertex" ) {
        pointCount = atoi( tokens[2].c_str() );
      } else {
        isVertexProperty = false;
      }
    } else if ( tokens[0] == "property" && isVertexProperty ) {
      if ( tokens.size() != 3 ) {
        std::cerr << "Error: Corrupted property info!" << std::endl;
        return false;
      }
      const std::string &propertyType = tokens[1];
      const std::string &propertyName = tokens[2];

      AttributeInfo info;
      info.name = propertyName;

      if ( propertyType == "double" || propertyType == "float64" ) {
        info.type      = ATTR_FLOAT64;
        info.byteCount = 8;
      } else if ( propertyType == "float" || propertyType == "float32" ) {
        info.type      = ATTR_FLOAT32;
        info.byteCount = 4;
      } else if ( propertyType == "uint64" ) {
        info.type      = ATTR_UINT64;
        info.byteCount = 8;
      } else if ( propertyType == "uint32" ) {
        info.type      = ATTR_UINT32;
        info.byteCount = 4;
      } else if ( propertyType == "uint16" ) {
        info.type      = ATTR_UINT16;
        info.byteCount = 2;
      } else if ( propertyType == "uchar" || propertyType == "uint8" ) {
        info.type      = ATTR_UINT8;
        info.byteCount = 1;
      } else if ( propertyType == "int64" ) {
        info.type      = ATTR_INT64;
        info.byteCount = 8;
      } else if ( propertyType == "int32" || propertyType == "int" ) {
        info.type      = ATTR_INT32;
        info.byteCount = 4;
      } else if ( propertyType == "int16" ) {
        info.type      = ATTR_INT16;
        info.byteCount = 2;
      } else if ( propertyType == "char" || propertyType == "int8" ) {
        info.type      = ATTR_INT8;
        info.byteCount = 1;
      } else {
        info.type      = ATTR_UINT8;
        info.byteCount = 1;
      }
      attributesInfo.push_back( info );
    } else if ( tokens[0] == "end_header" ) {
      break;
    }
  }

  if ( version != 1.0 ) {
    std::cerr << "Error: Non-supported PLY version!" << std::endl;
    return false;
  }

  // Map attribute indices
  const size_t        UNDEFINED    = std::numeric_limits<size_t>::max();
  size_t              indexX       = UNDEFINED;
  size_t              indexY       = UNDEFINED;
  size_t              indexZ       = UNDEFINED;
  size_t              indexR       = UNDEFINED;
  size_t              indexG       = UNDEFINED;
  size_t              indexB       = UNDEFINED;
  size_t              indexOpacity = UNDEFINED;
  std::vector<size_t> indexRot( 4, UNDEFINED );
  std::vector<size_t> indexScale( 3, UNDEFINED );
  std::vector<size_t> indexSH;
  indexSH.reserve( 48 );

  const size_t attributeCount = attributesInfo.size();
  for ( size_t a = 0; a < attributeCount; ++a ) {
    const auto &info = attributesInfo[a];
    if ( info.name == "x" && ( info.byteCount == 4 || info.byteCount == 8 ) ) {
      indexX = a;
    } else if ( info.name == "y" && ( info.byteCount == 4 || info.byteCount == 8 ) ) {
      indexY = a;
    } else if ( info.name == "z" && ( info.byteCount == 4 || info.byteCount == 8 ) ) {
      indexZ = a;
    } else if ( info.name == "red" && info.byteCount == 1 ) {
      indexR = a;
    } else if ( info.name == "green" && info.byteCount == 1 ) {
      indexG = a;
    } else if ( info.name == "blue" && info.byteCount == 1 ) {
      indexB = a;
    } else if ( info.name == "opacity" ) {
      indexOpacity = a;
    } else if ( info.name.compare( 0, 5, "f_dc_" ) == 0 ) {
      int i = std::stoi( info.name.substr( 5 ) );
      if ( indexSH.size() <= static_cast<size_t>( i ) ) indexSH.resize( i + 1, UNDEFINED );
      indexSH[i] = a;
    } else if ( info.name.compare( 0, 7, "f_rest_" ) == 0 ) {
      int i = std::stoi( info.name.substr( 7 ) ) + 3;
      if ( indexSH.size() <= static_cast<size_t>( i ) ) indexSH.resize( i + 1, UNDEFINED );
      indexSH[i] = a;
    } else if ( info.name.compare( 0, 6, "scale_" ) == 0 ) {
      int i = std::stoi( info.name.substr( 6 ) );
      if ( i >= 0 && i < 3 ) indexScale[i] = a;
    } else if ( info.name.compare( 0, 4, "rot_" ) == 0 ) {
      int i = std::stoi( info.name.substr( 4 ) );
      if ( i >= 0 && i < 4 ) indexRot[i] = a;
    }
  }

  if ( indexX == UNDEFINED || indexY == UNDEFINED || indexZ == UNDEFINED ) {
    std::cerr << "Error: Missing position coordinates in PLY!" << std::endl;
    return false;
  }

  // Set attribute flags
  withColors_    = ( indexR != UNDEFINED && indexG != UNDEFINED && indexB != UNDEFINED );
  withOpacities_ = ( indexOpacity != UNDEFINED );
  withScales_    = ( indexScale[0] != UNDEFINED && indexScale[1] != UNDEFINED && indexScale[2] != UNDEFINED );
  withRotations_ =
      ( indexRot[0] != UNDEFINED && indexRot[1] != UNDEFINED && indexRot[2] != UNDEFINED && indexRot[3] != UNDEFINED );
  withSphericalHarmonics_ =
      !indexSH.empty() && std::all_of( indexSH.begin(), indexSH.end(), [&]( size_t i ) { return i != UNDEFINED; } );

  if ( withSphericalHarmonics_ ) { shCoefficientCount_ = indexSH.size() / 3; }

  // Allocate storage
  resize( pointCount );

  // Lambda to read float value from any numeric type
  auto readFloatValue = [&]( std::ifstream &stream, const AttributeInfo &info ) -> float {
    switch ( info.type ) {
      case ATTR_FLOAT64: return static_cast<float>( readValue<double>( stream ) );
      case ATTR_FLOAT32: return readValue<float>( stream );
      case ATTR_UINT64: return static_cast<float>( readValue<uint64_t>( stream ) );
      case ATTR_INT64: return static_cast<float>( readValue<int64_t>( stream ) );
      case ATTR_UINT32: return static_cast<float>( readValue<uint32_t>( stream ) );
      case ATTR_INT32: return static_cast<float>( readValue<int32_t>( stream ) );
      case ATTR_UINT16: return static_cast<float>( readValue<uint16_t>( stream ) );
      case ATTR_INT16: return static_cast<float>( readValue<int16_t>( stream ) );
      case ATTR_UINT8: return static_cast<float>( readValue<uint8_t>( stream ) );
      case ATTR_INT8: return static_cast<float>( readValue<int8_t>( stream ) );
      default: return 0.0f;
    }
  };

  if ( isAscii ) {
    // ASCII format
    size_t pointCounter = 0;
    while ( !ifs.eof() && pointCounter < pointCount ) {
      ifs.getline( tmp, MAX_BUFFER_SIZE );
      getTokens( tmp, sep, tokens );
      if ( tokens.empty() ) continue;
      if ( tokens.size() < attributeCount ) {
        std::cerr << "Error: Insufficient tokens in ASCII PLY line!" << std::endl;
        return false;
      }

      auto &position = positions_[pointCounter];
      position.x     = static_cast<float>( atof( tokens[indexX].c_str() ) );
      position.y     = static_cast<float>( atof( tokens[indexY].c_str() ) );
      position.z     = static_cast<float>( atof( tokens[indexZ].c_str() ) );

      if ( hasColors() ) {
        auto &color = colors_[pointCounter];
        color[0]    = static_cast<uint8_t>( atoi( tokens[indexR].c_str() ) );
        color[1]    = static_cast<uint8_t>( atoi( tokens[indexG].c_str() ) );
        color[2]    = static_cast<uint8_t>( atoi( tokens[indexB].c_str() ) );
      }

      if ( hasOpacities() ) { opacities_[pointCounter] = static_cast<float>( atof( tokens[indexOpacity].c_str() ) ); }

      if ( hasScales() ) {
        auto &scale = scales_[pointCounter];
        scale.x     = static_cast<float>( atof( tokens[indexScale[0]].c_str() ) );
        scale.y     = static_cast<float>( atof( tokens[indexScale[1]].c_str() ) );
        scale.z     = static_cast<float>( atof( tokens[indexScale[2]].c_str() ) );
      }

      if ( hasRotations() ) {
        auto &rotation = rotations_[pointCounter];
        rotation.w     = static_cast<float>( atof( tokens[indexRot[0]].c_str() ) );
        rotation.x     = static_cast<float>( atof( tokens[indexRot[1]].c_str() ) );
        rotation.y     = static_cast<float>( atof( tokens[indexRot[2]].c_str() ) );
        rotation.z     = static_cast<float>( atof( tokens[indexRot[3]].c_str() ) );
      }

      if ( hasSphericalHarmonics() ) {
        auto &sh = sphericalHarmonics_[pointCounter];
        for ( size_t i = 0; i < indexSH.size(); ++i ) {
          sh[i / 3][i % 3] = static_cast<float>( atof( tokens[indexSH[i]].c_str() ) );
        }
      }

      ++pointCounter;
    }
  } else {
    // Binary format
    ifs.close();
    ifs.open( fileName, std::ifstream::binary | std::ifstream::in );

    // Find header end
    ifs.read( tmp, MAX_BUFFER_SIZE );
    char *str       = strstr( tmp, "end_header" );
    str             = strstr( str, "\n" );
    int headerCount = static_cast<int>( str - tmp + 1 );

    ifs.close();
    ifs.open( fileName, std::ifstream::binary | std::ifstream::in );
    ifs.read( tmp, headerCount );

    for ( size_t pointCounter = 0; pointCounter < pointCount && !ifs.eof(); ++pointCounter ) {
      auto &position = positions_[pointCounter];

      for ( size_t a = 0; a < attributeCount && !ifs.eof(); ++a ) {
        const auto &info = attributesInfo[a];

        if ( a == indexX ) {
          position.x = readFloatValue( ifs, info );
        } else if ( a == indexY ) {
          position.y = readFloatValue( ifs, info );
        } else if ( a == indexZ ) {
          position.z = readFloatValue( ifs, info );
        } else if ( a == indexR && info.byteCount == 1 ) {
          ifs.read( reinterpret_cast<char *>( &colors_[pointCounter][0] ), 1 );
        } else if ( a == indexG && info.byteCount == 1 ) {
          ifs.read( reinterpret_cast<char *>( &colors_[pointCounter][1] ), 1 );
        } else if ( a == indexB && info.byteCount == 1 ) {
          ifs.read( reinterpret_cast<char *>( &colors_[pointCounter][2] ), 1 );
        } else if ( a == indexOpacity ) {
          opacities_[pointCounter] = readFloatValue( ifs, info );
        } else {
          // Check SH coefficients
          auto shIt = std::find( indexSH.begin(), indexSH.end(), a );
          if ( shIt != indexSH.end() ) {
            size_t i                                        = std::distance( indexSH.begin(), shIt );
            sphericalHarmonics_[pointCounter][i / 3][i % 3] = readFloatValue( ifs, info );
            continue;
          }

          // Check scale
          auto scaleIt = std::find( indexScale.begin(), indexScale.end(), a );
          if ( scaleIt != indexScale.end() ) {
            size_t i                 = std::distance( indexScale.begin(), scaleIt );
            scales_[pointCounter][i] = readFloatValue( ifs, info );
            continue;
          }

          // Check rotation
          auto rotIt = std::find( indexRot.begin(), indexRot.end(), a );
          if ( rotIt != indexRot.end() ) {
            size_t i                    = std::distance( indexRot.begin(), rotIt );
            rotations_[pointCounter][i] = readFloatValue( ifs, info );
            continue;
          }

          // Skip unknown attribute
          char skipBuf[8];
          ifs.read( skipBuf, info.byteCount );
        }
      }
    }
  }

  ifs.close();
  return true;
}

bool PCCPointSetGS::write( const std::string &fileName, bool asAscii ) const {
  std::ofstream fout( fileName, asAscii ? std::ios::out : ( std::ios::out | std::ios::binary ) );
  if ( !fout.is_open() ) {
    std::cerr << "Error: Cannot open file " << fileName << " for writing!" << std::endl;
    return false;
  }

  const size_t pointCount = getPointCount();

  // Write header
  fout << "ply" << std::endl;
  if ( asAscii ) {
    fout << "format ascii 1.0" << std::endl;
  } else {
    fout << "format binary_little_endian 1.0" << std::endl;
  }

  fout << "element vertex " << pointCount << std::endl;
  fout << "property float x" << std::endl;
  fout << "property float y" << std::endl;
  fout << "property float z" << std::endl;

  if ( hasColors() ) {
    fout << "property uchar red" << std::endl;
    fout << "property uchar green" << std::endl;
    fout << "property uchar blue" << std::endl;
  }

  if ( hasOpacities() ) { fout << "property float opacity" << std::endl; }

  if ( hasScales() ) {
    fout << "property float scale_0" << std::endl;
    fout << "property float scale_1" << std::endl;
    fout << "property float scale_2" << std::endl;
  }

  if ( hasRotations() ) {
    fout << "property float rot_0" << std::endl;
    fout << "property float rot_1" << std::endl;
    fout << "property float rot_2" << std::endl;
    fout << "property float rot_3" << std::endl;
  }

  if ( hasSphericalHarmonics() ) {
    // DC components
    fout << "property float f_dc_0" << std::endl;
    fout << "property float f_dc_1" << std::endl;
    fout << "property float f_dc_2" << std::endl;
    // Rest components
    for ( size_t i = 3; i < shCoefficientCount_ * 3; ++i ) {
      fout << "property float f_rest_" << ( i - 3 ) << std::endl;
    }
  }

  fout << "end_header" << std::endl;

  // Write data
  if ( asAscii ) {
    fout << std::fixed << std::setprecision( 6 );
    for ( size_t i = 0; i < pointCount; ++i ) {
      const auto &pos = positions_[i];
      fout << pos.x << " " << pos.y << " " << pos.z;

      if ( hasColors() ) {
        const auto &col = colors_[i];
        fout << " " << static_cast<int>( col[0] ) << " " << static_cast<int>( col[1] ) << " "
             << static_cast<int>( col[2] );
      }

      if ( hasOpacities() ) { fout << " " << opacities_[i]; }

      if ( hasScales() ) {
        const auto &scale = scales_[i];
        fout << " " << scale.x << " " << scale.y << " " << scale.z;
      }

      if ( hasRotations() ) {
        const auto &rot = rotations_[i];
        fout << " " << rot.w << " " << rot.x << " " << rot.y << " " << rot.z;
      }

      if ( hasSphericalHarmonics() ) {
        const auto &sh = sphericalHarmonics_[i];
        for ( size_t j = 0; j < shCoefficientCount_ * 3; ++j ) { fout << " " << sh[j / 3][j % 3]; }
      }

      fout << std::endl;
    }
  } else {
    // Binary format
    for ( size_t i = 0; i < pointCount; ++i ) {
      const auto &pos = positions_[i];
      fout.write( reinterpret_cast<const char *>( &pos.x ), sizeof( float ) );
      fout.write( reinterpret_cast<const char *>( &pos.y ), sizeof( float ) );
      fout.write( reinterpret_cast<const char *>( &pos.z ), sizeof( float ) );

      if ( hasColors() ) {
        const auto &col = colors_[i];
        fout.write( reinterpret_cast<const char *>( &col[0] ), sizeof( uint8_t ) );
        fout.write( reinterpret_cast<const char *>( &col[1] ), sizeof( uint8_t ) );
        fout.write( reinterpret_cast<const char *>( &col[2] ), sizeof( uint8_t ) );
      }

      if ( hasOpacities() ) { fout.write( reinterpret_cast<const char *>( &opacities_[i] ), sizeof( float ) ); }

      if ( hasScales() ) {
        const auto &scale = scales_[i];
        fout.write( reinterpret_cast<const char *>( &scale.x ), sizeof( float ) );
        fout.write( reinterpret_cast<const char *>( &scale.y ), sizeof( float ) );
        fout.write( reinterpret_cast<const char *>( &scale.z ), sizeof( float ) );
      }

      if ( hasRotations() ) {
        const auto &rot = rotations_[i];
        fout.write( reinterpret_cast<const char *>( &rot.w ), sizeof( float ) );
        fout.write( reinterpret_cast<const char *>( &rot.x ), sizeof( float ) );
        fout.write( reinterpret_cast<const char *>( &rot.y ), sizeof( float ) );
        fout.write( reinterpret_cast<const char *>( &rot.z ), sizeof( float ) );
      }

      if ( hasSphericalHarmonics() ) {
        const auto &sh = sphericalHarmonics_[i];
        for ( size_t j = 0; j < shCoefficientCount_ * 3; ++j ) {
          float val = sh[j / 3][j % 3];
          fout.write( reinterpret_cast<const char *>( &val ), sizeof( float ) );
        }
      }
    }
  }

  fout.close();
  return true;
}

}  // namespace pcc
