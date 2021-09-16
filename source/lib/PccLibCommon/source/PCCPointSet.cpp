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
#include "PCCCommon.h"

#include "PCCPointSet.h"
#include "PCCMath.h"
#include "KDTreeVectorOfVectorsAdaptor.h"
#include "PCCKdTree.h"
#include <numeric>

using namespace pcc;

void PCCPointSet3::removeDuplicate() {
  PCCPointSet3 newPointcloud;
  if ( withColors_ ) { newPointcloud.hasColors(); }
  if ( withReflectances_ ) { newPointcloud.addReflectances(); }
  std::map<float, std::map<float, std::map<float, size_t>>> eMapA;
  if ( withColors_ ) {
    for ( size_t i = 0; i < positions_.size(); ++i ) {
      float x = positions_[i][0];
      float y = positions_[i][1];
      float z = positions_[i][2];
      if ( eMapA.find( x ) != eMapA.end() && eMapA[x].find( y ) != eMapA[x].end() &&
           eMapA[x][y].find( z ) != eMapA[x][y].end() ) {
      } else {
        eMapA[x][y][z] = i;
        newPointcloud.addPoint( positions_[i], colors_[i] );
      }
    }
  } else {
    for ( size_t i = 0; i < positions_.size(); ++i ) {
      float x = positions_[i][0];
      float y = positions_[i][1];
      float z = positions_[i][2];
      if ( eMapA.find( x ) != eMapA.end() && eMapA[x].find( y ) != eMapA[x].end() &&
           eMapA[x][y].find( z ) != eMapA[x][y].end() ) {
      } else {
        eMapA[x][y][z] = i;
        newPointcloud.addPoint( positions_[i] );
      }
    }
  }
  positions_.swap( newPointcloud.positions_ );
  colors_.swap( newPointcloud.colors_ );
  reflectances_.swap( newPointcloud.reflectances_ );
  types_.swap( newPointcloud.types_ );
}

void PCCPointSet3::distanceGeo( const PCCPointSet3& pointcloud, float& distPAB, float& distPBA ) const {
  this->distance( pointcloud, distPAB );
  pointcloud.distance( *this, distPBA );
}

void PCCPointSet3::distanceGeoColor( const PCCPointSet3& pointcloud,
                                     float&              distPAB,
                                     float&              distPBA,
                                     float&              distYAB,
                                     float&              distYBA,
                                     float&              distUAB,
                                     float&              distUBA,
                                     float&              distVAB,
                                     float&              distVBA ) const {
  this->distance( pointcloud, distPAB, distYAB, distUAB, distVAB );
  pointcloud.distance( *this, distPBA, distYBA, distUBA, distVBA );
}
void convertRGBtoYUV_BT709( const PCCColor3B& in_rgb, float* out_yuv ) {
  // color space conversion to YUV
  out_yuv[0] = ( 0.2126F * in_rgb[0] + 0.7152F * in_rgb[1] + 0.0722F * in_rgb[2] ) / 255.0F;
  out_yuv[1] = ( -0.1146F * in_rgb[0] - 0.3854F * in_rgb[1] + 0.5000F * in_rgb[2] ) / 255.0F + 0.5000F;
  out_yuv[2] = ( 0.5000F * in_rgb[0] - 0.4542F * in_rgb[1] - 0.0458F * in_rgb[2] ) / 255.0F + 0.5000F;
}
void PCCPointSet3::distance( const PCCPointSet3& pointcloud,
                             float&              distP,
                             float&              distY,
                             float&              distU,
                             float&              distV ) const {
  distP = 0.F;
  distY = 0.F;
  distU = 0.F;
  distV = 0.F;
  PCCKdTree   kdtree( pointcloud );
  PCCNNResult result;

  for ( size_t i = 0; i < positions_.size(); ++i ) {
    kdtree.search( positions_[i], 1, result );
    distP += result.dist( 0 );
    float yuvA[3];
    float yuvB[3];
    convertRGBtoYUV_BT709( colors_[i], yuvA );
    convertRGBtoYUV_BT709( pointcloud.colors_[result.indices( 0 )], yuvB );
    distY += pow( yuvA[0] - yuvB[0], 2.F );
    distU += pow( yuvA[1] - yuvB[1], 2.F );
    distV += pow( yuvA[2] - yuvB[2], 2.F );
  }
  distP /= static_cast<float>( positions_.size() );

  distY /= static_cast<float>( positions_.size() );
  distU /= static_cast<float>( positions_.size() );
  distV /= static_cast<float>( positions_.size() );
}

void PCCPointSet3::distance( const PCCPointSet3& pointcloud, float& distP ) const {
  distP = 0.F;
  PCCKdTree   kdtree( pointcloud );
  PCCNNResult result;
  for ( const auto& position : positions_ ) {
    kdtree.search( position, 1, result );
    distP += result.dist( 0 );
  }
  distP /= static_cast<float>( positions_.size() );
}

PCCPoint3D PCCPointSet3::computeCentroid() const {
  PCCPoint3D   bary( 0.0 );
  const size_t pointCount = getPointCount();
  if ( pointCount != 0u ) {
    for ( size_t i = 0; i < pointCount; ++i ) {
      const PCCPoint3D& pt = ( *this )[i];
      bary += pt;
    }
    bary /= double( pointCount );
  }
  return bary;
}

PCCBox3D PCCPointSet3::computeBoundingBox() const {
  PCCBox3D     bbox       = {( std::numeric_limits<double>::max )(), ( std::numeric_limits<double>::lowest )()};
  const size_t pointCount = getPointCount();
  for ( size_t i = 0; i < pointCount; ++i ) {
    const PCCPoint3D& pt = ( *this )[i];
    for ( int k = 0; k < 3; ++k ) {
      if ( pt[k] > bbox.max_[k] ) { bbox.max_[k] = pt[k]; }
      if ( pt[k] < bbox.min_[k] ) { bbox.min_[k] = pt[k]; }
    }
  }
  return bbox;
}

void PCCPointSet3::removeDuplicate( PCCPointSet3& newPointcloud, size_t dropDuplicates ) const {
  if ( withColors_ ) { newPointcloud.hasColors(); }
  if ( withReflectances_ ) { newPointcloud.addReflectances(); }
  if ( withNormals_ ) {
    std::cerr << "Normaled objects can't be modified or reordered \n" << std::endl;
    exit( -1 );
  }
  std::map<float, std::map<float, std::map<float, std::vector<size_t>>>> map;
  for ( size_t i = 0; i < positions_.size(); ++i ) {
    float x = positions_[i][0];
    float y = positions_[i][1];
    float z = positions_[i][2];
    map[x][y][z].push_back( i );
  }
  if ( withColors_ ) {
    for ( auto& itX : map ) {
      for ( auto& itY : itX.second ) {
        for ( auto& itZ : itY.second ) {
          auto& listIndex = itZ.second;
          if ( listIndex.size() == 1 || dropDuplicates == 1 ) {
            newPointcloud.addPoint( positions_[listIndex[0]], colors_[listIndex[0]] );
          } else {
            PCCColor3B average;
            size_t     r = 0;
            size_t     g = 0;
            size_t     b = 0;
            for ( auto& index : listIndex ) {
              r += colors_[index][0];
              g += colors_[index][1];
              b += colors_[index][2];
            }
            average[0] = r / listIndex.size();
            average[1] = g / listIndex.size();
            average[2] = b / listIndex.size();
            newPointcloud.addPoint( positions_[listIndex[0]], average );
          }
        }
      }
    }
  } else {
    for ( auto& itX : map ) {
      for ( auto& itY : itX.second ) {
        for ( auto& itZ : itY.second ) {
          auto& listIndex = itZ.second;
          newPointcloud.addPoint( positions_[listIndex[0]] );
        }
      }
    }
  }
}

using UInt = unsigned int;
#include "MD5.h"
std::vector<uint8_t> PCCPointSet3::computeChecksum( bool reorderPoints ) {
  if ( reorderPoints ) {
    PCCPointSet3 reorderPointCloud;
    if ( withColors_ ) { reorderPointCloud.hasColors(); }
    if ( withReflectances_ ) { reorderPointCloud.addReflectances(); }
    reorder( reorderPointCloud, true );
    return reorderPointCloud.computeMd5();
  }
  return computeMd5();
}
std::vector<uint8_t> PCCPointSet3::computeMd5() {
  std::vector<uint8_t> digest;
  MD5                  md5;
  md5.update( reinterpret_cast<uint8_t*>( positions_.data() ), positions_.size() * sizeof( PCCPoint3D ) );
  if ( withColors_ ) {
    md5.update( reinterpret_cast<uint8_t*>( colors_.data() ), colors_.size() * sizeof( PCCColor3B ) );
  }
  if ( withReflectances_ ) {
    md5.update( reinterpret_cast<uint8_t*>( reflectances_.data() ), reflectances_.size() * sizeof( uint16_t ) );
  }
  digest.resize( MD5_DIGEST_STRING_LENGTH );
  md5.finalize( digest.data() );
  return digest;
}

void PCCPointSet3::sortColor( std::vector<size_t>& list ) {
  for ( size_t i = 0; i < list.size(); ++i ) {
    size_t indexMin = i;
    for ( size_t j = i + 1; j < list.size(); ++j ) {
      if ( colors_[list[j]] < colors_[list[indexMin]] ) { indexMin = j; }
    }
    if ( i != indexMin ) {
      size_t tmp     = list[indexMin];
      list[indexMin] = list[i];
      list[i]        = tmp;
    }
  }
}

void PCCPointSet3::reorder( PCCPointSet3& newPointcloud, bool dropDuplicates ) {
  std::map<float, std::map<float, std::map<float, std::vector<size_t>>>> map;
  for ( size_t i = 0; i < positions_.size(); ++i ) {
    float x = positions_[i][0];
    float y = positions_[i][1];
    float z = positions_[i][2];
    map[x][y][z].push_back( i );
  }
  if ( withColors_ ) {
    for ( auto& itX : map ) {
      for ( auto& itY : itX.second ) {
        for ( auto& itZ : itY.second ) {
          auto& listIndex = itZ.second;
          if ( listIndex.size() > 1 ) { sortColor( listIndex ); }
          if ( dropDuplicates ) {
            PCCColor3B average;
            size_t     r = 0;
            size_t     g = 0;
            size_t     b = 0;
            for ( auto& index : listIndex ) {
              r += colors_[index][0];
              g += colors_[index][1];
              b += colors_[index][2];
            }
            average[0] = r / listIndex.size();
            average[1] = g / listIndex.size();
            average[2] = b / listIndex.size();
            newPointcloud.addPoint( positions_[listIndex[0]], average );
          } else {
            for ( auto& index : listIndex ) { newPointcloud.addPoint( positions_[index], colors_[index] ); }
          }
        }
      }
    }
  } else {
    for ( auto& itX : map ) {
      for ( auto& itY : itX.second ) {
        for ( auto& itZ : itY.second ) {
          auto& listIndex = itZ.second;
          for ( auto& index : listIndex ) { newPointcloud.addPoint( positions_[index] ); }
        }
      }
    }
  }
}

void PCCPointSet3::reorder() {
  PCCPointSet3 newPointcloud;
  if ( withColors_ ) { newPointcloud.hasColors(); }
  if ( withReflectances_ ) { newPointcloud.addReflectances(); }
  reorder( newPointcloud, false );
  swap( newPointcloud );
}
void PCCPointSet3::swap( PCCPointSet3& newPointcloud ) {
  positions_.swap( newPointcloud.positions_ );
  colors_.swap( newPointcloud.colors_ );
  reflectances_.swap( newPointcloud.reflectances_ );
  types_.swap( newPointcloud.types_ );
  normals_.swap( newPointcloud.normals_ );
}
bool PCCPointSet3::isBboxEmpty( PCCBox3D bbox ) const {
  const size_t pointCount = getPointCount();
  for ( size_t i = 0; i < pointCount; ++i ) {
    const PCCPoint3D& pt = ( *this )[i];
    if ( ( pt[0] >= bbox.min_[0] && pt[0] <= bbox.max_[0] ) && ( pt[1] >= bbox.min_[1] && pt[1] <= bbox.max_[1] ) &&
         ( pt[2] >= bbox.min_[2] && pt[2] <= bbox.max_[2] ) ) {
      return false;
    }
  }
  return true;
}

int PCCPointSet3::fillRawPointsBbox( std::vector<size_t>  rawPoints,
                                     PCCBox3D             bbox,
                                     std::vector<size_t>& bboxRawPoints ) const {
  const size_t rawPointsCount = rawPoints.size();
  for ( size_t i = 0; i < rawPointsCount; ++i ) {
    const PCCPoint3D& pt = ( *this )[rawPoints[i]];
    if ( ( pt[0] >= bbox.min_[0] && pt[0] <= bbox.max_[0] ) && ( pt[1] >= bbox.min_[1] && pt[1] <= bbox.max_[1] ) &&
         ( pt[2] >= bbox.min_[2] && pt[2] <= bbox.max_[2] ) ) {
      bboxRawPoints.push_back( rawPoints[i] );
    }
  }
  return bboxRawPoints.size();
}

bool PCCPointSet3::isRawPointsBboxEmpty( std::vector<size_t> rawPoints, PCCBox3D bbox ) const {
  const size_t rawPointsCount = rawPoints.size();
  for ( size_t i = 0; i < rawPointsCount; ++i ) {
    const PCCPoint3D& pt = ( *this )[rawPoints[i]];
    if ( ( pt[0] >= bbox.min_[0] && pt[0] <= bbox.max_[0] ) && ( pt[1] >= bbox.min_[1] && pt[1] <= bbox.max_[1] ) &&
         ( pt[2] >= bbox.min_[2] && pt[2] <= bbox.max_[2] ) ) {
      return false;
    }
  }
  return true;
}

bool PCCPointSet3::write( const std::string& fileName, const bool asAscii ) {
  std::ofstream fout( fileName, std::ofstream::out );
  if ( !fout.is_open() ) { return false; }
  const size_t pointCount = getPointCount();
  fout << "ply" << std::endl;

  if ( asAscii ) {
    fout << "format ascii 1.0" << std::endl;
  } else {
    PCCEndianness endianess = PCCSystemEndianness();
    if ( endianess == PCC_BIG_ENDIAN ) {
      fout << "format binary_big_endian 1.0" << std::endl;
    } else {
      fout << "format binary_little_endian 1.0" << std::endl;
    }
  }
  fout << "element vertex " << pointCount << std::endl;
  if ( asAscii ) {
    fout << "property float x" << std::endl;
    fout << "property float y" << std::endl;
    fout << "property float z" << std::endl;
  } else {
    // fout << "property int16 x" << std::endl;
    // fout << "property int16 y" << std::endl;
    // fout << "property int16 z" << std::endl;
    fout << "property float x" << std::endl;
    fout << "property float y" << std::endl;
    fout << "property float z" << std::endl;
  }
  if ( hasNormals() ) {
    fout << "property float nx" << std::endl;
    fout << "property float ny" << std::endl;
    fout << "property float nz" << std::endl;
  }
  if ( hasColors() ) {
    fout << "property uchar red" << std::endl;
    fout << "property uchar green" << std::endl;
    fout << "property uchar blue" << std::endl;
  }
  if ( hasReflectances() ) { fout << "property uint16 refc" << std::endl; }
  if ( PCC_SAVE_POINT_TYPE != 0u ) {
    fout << "property uchar type" << std::endl;
    switch ( PCC_SAVE_POINT_TYPE ) {
      case 1: fout << "comment POINT_TYPE: Unset D0 D1 Filling Smooth InBetween" << std::endl; break;
      case 2: fout << "comment POINT_TYPE: type0 type1 type2  " << std::endl; break;
      default: break;
    }
  }
  fout << "element face 0" << std::endl;
  fout << "property list uint8 int32 vertex_index" << std::endl;
  fout << "end_header" << std::endl;
  if ( asAscii ) {
    fout << std::setprecision( std::numeric_limits<double>::max_digits10 );
    for ( size_t i = 0; i < pointCount; ++i ) {
      const PCCPoint3D& position = ( *this )[i];
      fout << position.x() << " " << position.y() << " " << position.z();
      if ( hasNormals() ) {
        const PCCNormal3D& normal = getNormals()[i];
        fout << " " << static_cast<float>( normal[0] ) << " " << static_cast<float>( normal[1] ) << " "
             << static_cast<float>( normal[2] );
      }
      if ( hasColors() ) {
        const PCCColor3B& color = getColor( i );
        fout << " " << static_cast<int>( color[0] ) << " " << static_cast<int>( color[1] ) << " "
             << static_cast<int>( color[2] );
      }
      if ( hasReflectances() ) { fout << " " << static_cast<int>( getReflectance( i ) ); }
      if ( PCC_SAVE_POINT_TYPE != 0u ) { fout << " " << static_cast<int>( types_[i] ); }
      fout << std::endl;
    }
  } else {
    fout.clear();
    fout.close();
    fout.open( fileName, std::ofstream::binary | std::ofstream::out | std::ofstream::app );
    for ( size_t i = 0; i < pointCount; ++i ) {
      const PCCPoint3D& position = ( *this )[i];
      // fout.write( reinterpret_cast<const char* const>( &position ), sizeof( PCCType ) * 3 );
      float value[3];
      value[0] = position[0];
      value[1] = position[1];
      value[2] = position[2];
      fout.write( reinterpret_cast<const char*>( &value ), sizeof( float ) * 3 );
      if ( hasNormals() ) {
        const PCCNormal3D& normal = getNormals()[i];
        value[0]                  = normal[0];
        value[1]                  = normal[1];
        value[2]                  = normal[2];
        fout.write( reinterpret_cast<const char*>( &value ), sizeof( float ) * 3 );
      }
      if ( hasColors() ) {
        const PCCColor3B& color = getColor( i );
        fout.write( reinterpret_cast<const char*>( &color ), sizeof( uint8_t ) * 3 );
      }
      if ( hasReflectances() ) {
        const uint16_t& reflectance = getReflectance( i );
        fout.write( reinterpret_cast<const char*>( &reflectance ), sizeof( uint16_t ) );
      }
      if ( PCC_SAVE_POINT_TYPE != 0u ) {
        fout.write( reinterpret_cast<const char* const>( &types_[i] ), sizeof( uint8_t ) );
      }
    }
  }
  fout.close();
  return true;
}
bool PCCPointSet3::read( const std::string& fileName, const bool readNormals ) {
  std::ifstream ifs( fileName, std::ifstream::in );
  if ( !ifs.is_open() ) { return false; }
  enum AttributeType {
    ATTRIBUTE_TYPE_FLOAT64 = 0,
    ATTRIBUTE_TYPE_FLOAT32 = 1,
    ATTRIBUTE_TYPE_UINT64  = 2,
    ATTRIBUTE_TYPE_UINT32  = 3,
    ATTRIBUTE_TYPE_UINT16  = 4,
    ATTRIBUTE_TYPE_UINT8   = 5,
    ATTRIBUTE_TYPE_INT64   = 6,
    ATTRIBUTE_TYPE_INT32   = 7,
    ATTRIBUTE_TYPE_INT16   = 8,
    ATTRIBUTE_TYPE_INT8    = 9,
  };
  struct AttributeInfo {
    std::string   name;
    AttributeType type;
    size_t        byteCount;
  };

  std::vector<AttributeInfo> attributesInfo;
  attributesInfo.reserve( 16 );
  const size_t             MAX_BUFFER_SIZE = 4096;
  char                     tmp[MAX_BUFFER_SIZE];
  const char*              sep = " \t\r";
  std::vector<std::string> tokens;

  ifs.getline( tmp, MAX_BUFFER_SIZE );
  getTokens( tmp, sep, tokens );
  if ( tokens.empty() || tokens[0] != "ply" ) {
    std::cout << "Error: corrupted file!" << std::endl;
    return false;
  }
  bool   isAscii          = false;
  double version          = 1.0;
  size_t pointCount       = 0;
  bool   isVertexProperty = true;
  while ( true ) {
    if ( ifs.eof() ) {
      std::cout << "Error: corrupted header!" << std::endl;
      return false;
    }
    ifs.getline( tmp, MAX_BUFFER_SIZE );
    getTokens( tmp, sep, tokens );
    if ( tokens.empty() || tokens[0] == "comment" ) { continue; }
    if ( tokens[0] == "format" ) {
      if ( tokens.size() != 3 ) {
        std::cout << "Error: corrupted format info!" << std::endl;
        return false;
      }
      isAscii = tokens[1] == "ascii";
      version = atof( tokens[2].c_str() );
    } else if ( tokens[0] == "element" ) {
      if ( tokens.size() != 3 ) {
        std::cout << "Error: corrupted element info!" << std::endl;
        return false;
      }
      if ( tokens[1] == "vertex" ) {
        pointCount = atoi( tokens[2].c_str() );
      } else {
        isVertexProperty = false;
      }
    } else if ( tokens[0] == "property" && isVertexProperty ) {
      if ( tokens.size() != 3 ) {
        std::cout << "Error: corrupted property info!" << std::endl;
        return false;
      }
      const std::string& propertyType   = tokens[1];
      const std::string& propertyName   = tokens[2];
      const size_t       attributeIndex = attributesInfo.size();
      attributesInfo.resize( attributeIndex + 1 );
      AttributeInfo& attributeInfo = attributesInfo[attributeIndex];
      attributeInfo.name           = propertyName;
      if ( propertyType == "float64" ) {
        attributeInfo.type      = ATTRIBUTE_TYPE_FLOAT64;
        attributeInfo.byteCount = 8;
      } else if ( propertyType == "float" || propertyType == "float32" ) {
        attributeInfo.type      = ATTRIBUTE_TYPE_FLOAT32;
        attributeInfo.byteCount = 4;
      } else if ( propertyType == "uint64" ) {
        attributeInfo.type      = ATTRIBUTE_TYPE_UINT64;
        attributeInfo.byteCount = 8;
      } else if ( propertyType == "uint32" ) {
        attributeInfo.type      = ATTRIBUTE_TYPE_UINT32;
        attributeInfo.byteCount = 4;
      } else if ( propertyType == "uint16" ) {
        attributeInfo.type      = ATTRIBUTE_TYPE_UINT16;
        attributeInfo.byteCount = 2;
      } else if ( propertyType == "uchar" || propertyType == "uint8" ) {
        attributeInfo.type      = ATTRIBUTE_TYPE_UINT8;
        attributeInfo.byteCount = 1;
      } else if ( propertyType == "int64" ) {
        attributeInfo.type      = ATTRIBUTE_TYPE_INT64;
        attributeInfo.byteCount = 8;
      } else if ( propertyType == "int32" || propertyType == "int" ) {
        attributeInfo.type      = ATTRIBUTE_TYPE_INT32;
        attributeInfo.byteCount = 4;
      } else if ( propertyType == "int16" ) {
        attributeInfo.type      = ATTRIBUTE_TYPE_INT16;
        attributeInfo.byteCount = 2;
      } else if ( propertyType == "char" || propertyType == "int8" ) {
        attributeInfo.type      = ATTRIBUTE_TYPE_INT8;
        attributeInfo.byteCount = 1;
      }
    } else if ( tokens[0] == "end_header" ) {
      break;
    }
  }
  if ( version != 1.0 ) {
    std::cout << "Error: non-supported version!" << std::endl;
    return false;
  }

  size_t       indexX           = g_undefined_index;
  size_t       indexY           = g_undefined_index;
  size_t       indexZ           = g_undefined_index;
  size_t       indexR           = g_undefined_index;
  size_t       indexG           = g_undefined_index;
  size_t       indexB           = g_undefined_index;
  size_t       indexReflectance = g_undefined_index;
  size_t       indexNX          = g_undefined_index;
  size_t       indexNY          = g_undefined_index;
  size_t       indexNZ          = g_undefined_index;
  const size_t attributeCount   = attributesInfo.size();
  for ( size_t a = 0; a < attributeCount; ++a ) {
    const auto& attributeInfo = attributesInfo[a];
    if ( attributeInfo.name == "x" &&
         ( attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4 || attributeInfo.byteCount == 2 ) ) {
      indexX = a;
    } else if ( attributeInfo.name == "y" &&
                ( attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4 || attributeInfo.byteCount == 2 ) ) {
      indexY = a;
    } else if ( attributeInfo.name == "z" &&
                ( attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4 || attributeInfo.byteCount == 2 ) ) {
      indexZ = a;
    } else if ( attributeInfo.name == "red" && attributeInfo.byteCount == 1 ) {
      indexR = a;
    } else if ( attributeInfo.name == "green" && attributeInfo.byteCount == 1 ) {
      indexG = a;
    } else if ( attributeInfo.name == "blue" && attributeInfo.byteCount == 1 ) {
      indexB = a;
    } else if ( attributeInfo.name == "nx" && attributeInfo.byteCount == 4 && readNormals ) {
      indexNX = a;
    } else if ( attributeInfo.name == "ny" && attributeInfo.byteCount == 4 && readNormals ) {
      indexNY = a;
    } else if ( attributeInfo.name == "nz" && attributeInfo.byteCount == 4 && readNormals ) {
      indexNZ = a;
    } else if ( ( attributeInfo.name == "reflectance" || attributeInfo.name == "refc" ) &&
                attributeInfo.byteCount <= 2 ) {
      indexReflectance = a;
    }
  }
  if ( indexX == g_undefined_index || indexY == g_undefined_index || indexZ == g_undefined_index ) {
    std::cout << "Error: missing coordinates!" << std::endl;
    return false;
  }
  withColors_       = indexR != g_undefined_index && indexG != g_undefined_index && indexB != g_undefined_index;
  withReflectances_ = indexReflectance != g_undefined_index;
  withNormals_      = indexNX != g_undefined_index && indexNY != g_undefined_index && indexNZ != g_undefined_index;
  resize( pointCount );
  if ( isAscii ) {
    size_t pointCounter = 0;
    while ( !ifs.eof() && pointCounter < pointCount ) {
      ifs.getline( tmp, MAX_BUFFER_SIZE );
      getTokens( tmp, sep, tokens );
      if ( tokens.empty() ) { continue; }
      if ( tokens.size() < attributeCount ) { return false; }
      auto& position = positions_[pointCounter];
      position[0]    = atof( tokens[indexX].c_str() );
      position[1]    = atof( tokens[indexY].c_str() );
      position[2]    = atof( tokens[indexZ].c_str() );
      if ( hasColors() ) {
        auto& color = colors_[pointCounter];
        color[0]    = atoi( tokens[indexR].c_str() );
        color[1]    = atoi( tokens[indexG].c_str() );
        color[2]    = atoi( tokens[indexB].c_str() );
      }
      if ( hasReflectances() ) { reflectances_[pointCounter] = uint16_t( atoi( tokens[indexReflectance].c_str() ) ); }
      ++pointCounter;
    }
  } else {
    ifs.close();
    ifs.open( fileName, std::ifstream::binary | std::ifstream::in );
    ifs.read( tmp, MAX_BUFFER_SIZE );
    char* str       = strstr( tmp, "end_header" );
    str             = strstr( str, "\n" );
    int headerCount = str - tmp + 1;
    ifs.close();
    ifs.open( fileName, std::ifstream::binary | std::ifstream::in );
    ifs.read( tmp, headerCount );
    for ( size_t pointCounter = 0; pointCounter < pointCount && !ifs.eof(); ++pointCounter ) {
      auto& position = positions_[pointCounter];
      for ( size_t a = 0; a < attributeCount && !ifs.eof(); ++a ) {
        const auto& attributeInfo = attributesInfo[a];
        if ( a == indexX ) {
          if ( attributeInfo.byteCount == 2 ) {
            uint16_t x;
            ifs.read( reinterpret_cast<char*>( &x ), sizeof( uint16_t ) );
            position[0] = x;
          } else if ( attributeInfo.byteCount == 4 ) {
            float x;
            ifs.read( reinterpret_cast<char*>( &x ), sizeof( float ) );
            position[0] = x;
          } else {
            double x;
            ifs.read( reinterpret_cast<char*>( &x ), sizeof( double ) );
            position[0] = x;
          }
        } else if ( a == indexY ) {
          if ( attributeInfo.byteCount == 2 ) {
            uint16_t y;
            ifs.read( reinterpret_cast<char*>( &y ), sizeof( uint16_t ) );
            position[1] = y;
          } else if ( attributeInfo.byteCount == 4 ) {
            float y;
            ifs.read( reinterpret_cast<char*>( &y ), sizeof( float ) );
            position[1] = y;
          } else {
            double y;
            ifs.read( reinterpret_cast<char*>( &y ), sizeof( double ) );
            position[1] = y;
          }
        } else if ( a == indexZ ) {
          if ( attributeInfo.byteCount == 2 ) {
            uint16_t z;
            ifs.read( reinterpret_cast<char*>( &z ), sizeof( uint16_t ) );
            position[2] = z;
          } else if ( attributeInfo.byteCount == 4 ) {
            float z;
            ifs.read( reinterpret_cast<char*>( &z ), sizeof( float ) );
            position[2] = z;
          } else {
            double z;
            ifs.read( reinterpret_cast<char*>( &z ), sizeof( double ) );
            position[2] = z;
          }
        } else if ( a == indexR && attributeInfo.byteCount == 1 ) {
          auto& color = colors_[pointCounter];
          ifs.read( reinterpret_cast<char*>( &color[0] ), sizeof( uint8_t ) );
        } else if ( a == indexG && attributeInfo.byteCount == 1 ) {
          auto& color = colors_[pointCounter];
          ifs.read( reinterpret_cast<char*>( &color[1] ), sizeof( uint8_t ) );
        } else if ( a == indexB && attributeInfo.byteCount == 1 ) {
          auto& color = colors_[pointCounter];
          ifs.read( reinterpret_cast<char*>( &color[2] ), sizeof( uint8_t ) );
        } else if ( a == indexNX ) {
          if ( attributeInfo.byteCount == 4 ) {
            float nx;
            ifs.read( reinterpret_cast<char*>( &nx ), sizeof( float ) );
            normals_[pointCounter][0] = nx;
          } else {
            double nx;
            ifs.read( reinterpret_cast<char*>( &nx ), sizeof( double ) );
            normals_[pointCounter][0] = nx;
          }
        } else if ( a == indexNY ) {
          if ( attributeInfo.byteCount == 4 ) {
            float ny;
            ifs.read( reinterpret_cast<char*>( &ny ), sizeof( float ) );
            normals_[pointCounter][1] = ny;
          } else {
            double ny;
            ifs.read( reinterpret_cast<char*>( &ny ), sizeof( double ) );
            normals_[pointCounter][1] = ny;
          }
        } else if ( a == indexNZ ) {
          if ( attributeInfo.byteCount == 4 ) {
            float nz;
            ifs.read( reinterpret_cast<char*>( &nz ), sizeof( float ) );
            normals_[pointCounter][2] = nz;
          } else {
            double nz;
            ifs.read( reinterpret_cast<char*>( &nz ), sizeof( double ) );
            normals_[pointCounter][2] = nz;
          }
        } else if ( a == indexReflectance && attributeInfo.byteCount <= 2 ) {
          if ( indexReflectance == 1 ) {
            uint8_t reflectance;
            ifs.read( reinterpret_cast<char*>( &reflectance ), sizeof( uint8_t ) );
            reflectances_[pointCounter] = reflectance;
          } else {
            auto& reflectance = reflectances_[pointCounter];
            ifs.read( reinterpret_cast<char*>( reflectance ), sizeof( uint16_t ) );
          }
        } else {
          char buffer[128];
          ifs.read( buffer, attributeInfo.byteCount );
        }
      }
    }
  }
  return true;
}

void PCCPointSet3::convertRGBToYUV() {  // BT709
  for ( auto& color : colors_ ) {
    const uint8_t r = color[0];
    const uint8_t g = color[1];
    const uint8_t b = color[2];
    const double  y = std::round( 0.212600 * r + 0.715200 * g + 0.072200 * b );
#if POSTSMOOTHING_RGB2YUV
    const double u = std::round( -0.1146 * r - 0.3854 * g + 0.500000 * b + 128.0 );
    const double v = std::round( 0.500000 * r - 0.4542 * g - 0.0458 * b + 128.0 );
#else
    const double u = std::round( -0.114572 * r - 0.385428 * g + 0.500000 * b + 128.0 );
    const double v = std::round( 0.500000 * r - 0.454153 * g - 0.045847 * b + 128.0 );
#endif
    assert( y >= 0.0 && y <= 255.0 && u >= 0.0 && u <= 255.0 && v >= 0.0 && v <= 255.0 );
    color[0] = static_cast<uint8_t>( y );
    color[1] = static_cast<uint8_t>( u );
    color[2] = static_cast<uint8_t>( v );
  }
}

void PCCPointSet3::convertRGBToYUVClosedLoop() {  // BT709
  for ( auto& color : colors_ ) {
    const uint8_t r = color[0];
    const uint8_t g = color[1];
    const uint8_t b = color[2];
    const double  y = std::round( 0.212600 * r + 0.715200 * g + 0.072200 * b );
    const double  u = std::round( ( b - y ) / 1.8556 + 128.0 );
    const double  v = std::round( ( r - y ) / 1.5748 + 128.0 );
    assert( y >= 0.0 && y <= 255.0 && u >= 0.0 && u <= 255.0 && v >= 0.0 && v <= 255.0 );
    color[0] = static_cast<uint8_t>( y );
    color[1] = static_cast<uint8_t>( u );
    color[2] = static_cast<uint8_t>( v );
  }
}
void PCCPointSet3::convertYUVToRGB() {  // BT709
  for ( auto& color : colors_ ) {
    const double y1 = color[0];
    const double u1 = color[1] - 128.0;
    const double v1 = color[2] - 128.0;
    const double r  = PCCClip( round( y1 /*- 0.00000 * u1*/ + 1.57480 * v1 ), 0.0, 255.0 );
    const double g  = PCCClip( round( y1 - 0.18733 * u1 - 0.46813 * v1 ), 0.0, 255.0 );
    const double b  = PCCClip( round( y1 + 1.85563 * u1 /*+ 0.00000 * v1*/ ), 0.0, 255.0 );
    color[0]        = static_cast<uint8_t>( r );
    color[1]        = static_cast<uint8_t>( g );
    color[2]        = static_cast<uint8_t>( b );
  }
}

bool PCCPointSet3::transferColors( PCCPointSet3& target,
                                   const int32_t searchRange,
                                   const bool    losslessAttribute,
                                   const int     numNeighborsColorTransferFwd,
                                   const int     numNeighborsColorTransferBwd,
                                   const bool    useDistWeightedAverageFwd,
                                   const bool    useDistWeightedAverageBwd,
                                   const bool    skipAvgIfIdenticalSourcePointPresentFwd,
                                   const bool    skipAvgIfIdenticalSourcePointPresentBwd,
                                   const double  distOffsetFwd,
                                   const double  distOffsetBwd,
                                   double        maxGeometryDist2Fwd,
                                   double        maxGeometryDist2Bwd,
                                   double        maxColorDist2Fwd,
                                   double        maxColorDist2Bwd,
                                   const bool    excludeColorOutlier,
                                   const double  thresholdColorOutlierDist ) const {
  printf( "transferColors \n" );
  const auto&  source           = *this;
  const size_t pointCountSource = source.getPointCount();
  const size_t pointCountTarget = target.getPointCount();
  if ( ( pointCountSource == 0u ) || ( pointCountTarget == 0u ) || !source.hasColors() ) { return false; }
  PCCKdTree kdtreeTarget( target );
  PCCKdTree kdtreeSource( source );
  target.addColors();
  std::vector<PCCColor3B> refinedColors1;
  refinedColors1.resize( pointCountTarget );
  maxGeometryDist2Fwd = ( maxGeometryDist2Fwd < 512 ) ? maxGeometryDist2Fwd : std::numeric_limits<double>::max();
  maxGeometryDist2Bwd = ( maxGeometryDist2Bwd < 512 ) ? maxGeometryDist2Bwd : std::numeric_limits<double>::max();
  maxColorDist2Fwd    = ( maxColorDist2Fwd < 512 ) ? maxColorDist2Fwd : std::numeric_limits<double>::max();
  maxColorDist2Bwd    = ( maxColorDist2Bwd < 512 ) ? maxColorDist2Bwd : std::numeric_limits<double>::max();

  // ==========================================================================================
  //                                     Forward direction
  // ==========================================================================================
  // for each target point indexed by index, derive the refined color as
  // refinedColors1[index]
  PCCNNResult result;
  for ( size_t index = 0; index < pointCountTarget; ++index ) {
    kdtreeSource.search( target[index], numNeighborsColorTransferFwd, result );
    // keep the points that satisfy geometry dist threshold
    while ( true ) {
      if ( result.size() == 1 ) { break; }
      if ( result.dist( int( result.size() ) - 1 ) <= maxGeometryDist2Fwd ) { break; }
      result.popBack();
    }
    bool isDone = false;
    if ( skipAvgIfIdenticalSourcePointPresentFwd ) {
      if ( result.dist( 0 ) < 0.0001 ) {
        refinedColors1[index] = source.getColor( result.indices( 0 ) );
        isDone                = true;
      }
    }
    if ( !isDone ) {
      int nNN = static_cast<int>( result.size() );
      while ( nNN > 0 && !isDone ) {
        if ( nNN == 1 ) {
          refinedColors1[index] = source.getColor( result.indices( 0 ) );
          isDone                = true;
        }
        if ( !isDone ) {
          std::vector<PCCVector3D> colors;
          colors.resize( 0 );
          colors.resize( nNN );
          for ( int i = 0; i < nNN; ++i ) {
            for ( int k = 0; k < 3; ++k ) { colors[i][k] = double( source.getColor( result.indices( i ) )[k] ); }
          }
          double maxColorDist2 = std::numeric_limits<double>::min();
          for ( int i = 0; i < nNN; ++i ) {
            for ( int j = i + 1; j < nNN; ++j ) {
              const double dist2 = ( colors[i] - colors[j] ).getNorm2();
              if ( dist2 > maxColorDist2 ) { maxColorDist2 = dist2; }
            }
          }
          if ( maxColorDist2 <= maxColorDist2Fwd ) {
            PCCVector3D refinedColor( 0.0 );
            if ( useDistWeightedAverageFwd ) {
              double sumWeights{0.0};
              for ( int i = 0; i < nNN; ++i ) {
                const double weight = 1 / ( result.dist( i ) + distOffsetFwd );
                for ( int k = 0; k < 3; ++k ) { refinedColor[k] += source.getColor( result.indices( i ) )[k] * weight; }
                sumWeights += weight;
              }
              refinedColor /= sumWeights;
              if ( excludeColorOutlier ) {
                PCCVector3D excludeOutlierRefinedColor( 0.0 );
                size_t      excludeCount = 0;
                sumWeights               = 0.0;
                for ( int i = 0; i < nNN; ++i ) {
                  double      dist     = 0.0;
                  PCCColor3B  tmpColor = source.getColor( result.indices( i ) );
                  PCCVector3D sourceColor( tmpColor[0], tmpColor[1], tmpColor[2] );
                  dist = ( sourceColor - refinedColor ).getNorm2();
                  if ( dist > thresholdColorOutlierDist * thresholdColorOutlierDist ) {
                    excludeCount += 1;
                    continue;
                  }
                  const double weight = 1 / ( result.dist( i ) + distOffsetFwd );
                  for ( int k = 0; k < 3; ++k ) {
                    excludeOutlierRefinedColor[k] += source.getColor( result.indices( i ) )[k] * weight;
                  }
                  sumWeights += weight;
                }

                if ( excludeCount != nNN && excludeCount != 0 ) {
                  refinedColor = excludeOutlierRefinedColor / sumWeights;
                }
              }
            } else {
              for ( int i = 0; i < nNN; ++i ) {
                for ( int k = 0; k < 3; ++k ) { refinedColor[k] += source.getColor( result.indices( i ) )[k]; }
              }
              refinedColor /= nNN;
            }
            for ( int k = 0; k < 3; ++k ) {
              refinedColors1[index][k] = uint8_t( PCCClip( round( refinedColor[k] ), 0.0, 255.0 ) );
            }
            isDone = true;
          } else {
            --nNN;
          }
        }
      }
    }
  }
  // ==========================================================================================
  //                                  Backward direction
  // ==========================================================================================
  // for each target point, derive a vector of source candidate points as
  // colorsDists2.
  // colorsDists2 is iteratively refined (by removing the farthest points) until
  // the
  // std of remaining colors in it is smaller than a threshold.
  std::vector<std::vector<DistColor8Bit>> refinedColorsDists2;
  refinedColorsDists2.resize( pointCountTarget );
  // populate refinedColorsDists2
  for ( size_t index = 0; index < pointCountSource; ++index ) {
    const PCCColor3B color = source.getColor( index );
    kdtreeTarget.search( source[index], numNeighborsColorTransferBwd, result );
    // keep the points that satisfy geometry dist threshold
    for ( int i = 0; i < result.size(); ++i ) {
      if ( result.dist( i ) <= maxGeometryDist2Bwd ) {
        refinedColorsDists2[result.indices( i )].push_back( DistColor8Bit{result.dist( i ), color} );
      }
    }
  }
  // sort refinedColorsDists2 according to distance
  for ( size_t index = 0; index < pointCountTarget; ++index ) {
    std::sort( refinedColorsDists2[index].begin(), refinedColorsDists2[index].end(),
               []( DistColor8Bit& dc1, DistColor8Bit& dc2 ) { return dc1.dist < dc2.dist; } );
  }
  // compute centroid2
  for ( size_t index = 0; index < pointCountTarget; ++index ) {
    const PCCColor3B color1       = refinedColors1[index];       // refined color derived in forward direction
    auto&            colorsDists2 = refinedColorsDists2[index];  // set of candidate points
                                                                 // derived in backward
                                                                 // direction
    if ( colorsDists2.empty() || losslessAttribute ) {
      target.setColor( index, color1 );
    } else {
      bool              isDone = false;
      const PCCVector3D centroid1( color1[0], color1[1], color1[2] );
      PCCVector3D       centroid2( 0.0 );
      if ( skipAvgIfIdenticalSourcePointPresentBwd ) {
        if ( colorsDists2[0].dist < 0.0001 ) {
          auto temp = colorsDists2[0];
          colorsDists2.clear();
          colorsDists2.push_back( temp );
          for ( int k = 0; k < 3; ++k ) { centroid2[k] = colorsDists2[0].color[k]; }
          isDone = true;
        }
      }
      if ( !isDone ) {
        int nNN = static_cast<int>( colorsDists2.size() );
        while ( nNN > 0 && !isDone ) {
          nNN = static_cast<int>( colorsDists2.size() );
          if ( nNN == 1 ) {
            auto temp = colorsDists2[0];
            colorsDists2.clear();
            colorsDists2.push_back( temp );
            for ( int k = 0; k < 3; ++k ) { centroid2[k] = colorsDists2[0].color[k]; }
            isDone = true;
          }
          if ( !isDone ) {
            std::vector<PCCVector3D> colors;
            colors.resize( 0 );
            colors.resize( nNN );
            for ( int i = 0; i < nNN; ++i ) {
              for ( int k = 0; k < 3; ++k ) { colors[i][k] = double( colorsDists2[i].color[k] ); }
            }
            double maxColorDist2 = std::numeric_limits<double>::min();
            for ( int i = 0; i < nNN; ++i ) {
              for ( int j = i + 1; j < nNN; ++j ) {
                const double dist2 = ( colors[i] - colors[j] ).getNorm2();
                if ( dist2 > maxColorDist2 ) { maxColorDist2 = dist2; }
              }
            }
            if ( maxColorDist2 <= maxColorDist2Bwd ) {
              for ( size_t k = 0; k < 3; ++k ) { centroid2[k] = 0; }
              if ( useDistWeightedAverageBwd ) {
                double sumWeights{0.0};
                for ( auto& i : colorsDists2 ) {
                  const double weight = 1 / ( sqrt( i.dist ) + distOffsetBwd );
                  for ( size_t k = 0; k < 3; ++k ) { centroid2[k] += ( i.color[k] * weight ); }
                  sumWeights += weight;
                }
                centroid2 /= sumWeights;
                if ( excludeColorOutlier ) {
                  PCCVector3D excludeOutlierCentroid2( 0.0 );
                  size_t      excludeCount = 0;
                  sumWeights               = 0.0;
                  for ( auto& i : colorsDists2 ) {
                    PCCVector3D sourceColor( i.color[0], i.color[1], i.color[2] );
                    double      dist = ( sourceColor - centroid2 ).getNorm2();
                    if ( dist > thresholdColorOutlierDist * thresholdColorOutlierDist ) {
                      excludeCount += 1;
                      continue;
                    }
                    const double weight = 1 / ( sqrt( i.dist ) + distOffsetBwd );
                    for ( size_t k = 0; k < 3; ++k ) { excludeOutlierCentroid2[k] += ( i.color[k] * weight ); }
                    sumWeights += weight;
                  }

                  if ( excludeCount != nNN && excludeCount != 0 ) { centroid2 = excludeOutlierCentroid2 / sumWeights; }
                }
              } else {
                for ( auto& coldist : colorsDists2 ) {
                  for ( int k = 0; k < 3; ++k ) { centroid2[k] += coldist.color[k]; }
                }
                centroid2 /= colorsDists2.size();
              }
              isDone = true;
            } else {
              colorsDists2.pop_back();
            }
          }
        }
      }
      auto   H  = double( colorsDists2.size() );
      double D2 = 0.0;
      for ( const auto& color2dist : colorsDists2 ) {
        auto color2 = color2dist.color;
        for ( size_t k = 0; k < 3; ++k ) {
          const double d2 = centroid2[k] - color2[k];
          D2 += d2 * d2;
        }
      }
      const double r      = double( pointCountTarget ) / double( pointCountSource );
      const double delta2 = ( centroid2 - centroid1 ).getNorm2();
      const double eps    = 0.000001;

      const bool fixWeight = true;        // m42538
      if ( fixWeight || delta2 > eps ) {  // centroid2 != centroid1
        double w = 0.0;

        if ( !fixWeight ) {
          const double alpha = D2 / delta2;
          const double a     = H * r - 1.0;
          const double c     = alpha * r - 1.0;
          if ( fabs( a ) < eps ) {
            w = -0.5 * c;
          } else {
            const double delta = 1.0 - a * c;
            if ( delta >= 0.0 ) { w = ( -1.0 + sqrt( delta ) ) / a; }
          }
        }
        const double oneMinusW = 1.0 - w;
        PCCVector3D  color0;
        for ( size_t k = 0; k < 3; ++k ) {
          color0[k] = PCCClip( round( w * centroid1[k] + oneMinusW * centroid2[k] ), 0.0, 255.0 );
        }
        const double rSource  = 1.0 / double( pointCountSource );
        const double rTarget  = 1.0 / double( pointCountTarget );
        const double maxValue = std::numeric_limits<uint8_t>::max();
        double       minError = std::numeric_limits<double>::max();
        PCCVector3D  bestColor( color0 );
        PCCVector3D  color;
        for ( int32_t s1 = -searchRange; s1 <= searchRange; ++s1 ) {
          color[0] = PCCClip( color0[0] + s1, 0.0, maxValue );
          for ( int32_t s2 = -searchRange; s2 <= searchRange; ++s2 ) {
            color[1] = PCCClip( color0[1] + s2, 0.0, maxValue );
            for ( int32_t s3 = -searchRange; s3 <= searchRange; ++s3 ) {
              color[2] = PCCClip( color0[2] + s3, 0.0, maxValue );

              double e1 = 0.0;
              for ( size_t k = 0; k < 3; ++k ) {
                const double d = color[k] - color1[k];
                e1 += d * d;
              }
              e1 *= rTarget;

              double e2 = 0.0;
              for ( const auto& color2dist : colorsDists2 ) {
                auto color2 = color2dist.color;
                for ( size_t k = 0; k < 3; ++k ) {
                  const double d = color[k] - color2[k];
                  e2 += d * d;
                }
              }
              e2 *= rSource;

              const double error = std::max( e1, e2 );
              if ( error < minError ) {
                minError  = error;
                bestColor = color;
              }
            }
          }
        }
        target.setColor( index,
                         PCCColor3B( uint8_t( bestColor[0] ), uint8_t( bestColor[1] ), uint8_t( bestColor[2] ) ) );
      } else {  // centroid2 == centroid1
        target.setColor( index, color1 );
      }
    }
  }
  return true;
}

bool PCCPointSet3::transferColors16bitBP( PCCPointSet3& target,
                                          const int     filterType,
                                          const int32_t searchRange,
                                          const bool    losslessAttribute,
                                          const int     numNeighborsColorTransferFwd,
                                          const int     numNeighborsColorTransferBwd,
                                          const bool    useDistWeightedAverageFwd,
                                          const bool    useDistWeightedAverageBwd,
                                          const bool    skipAvgIfIdenticalSourcePointPresentFwd,
                                          const bool    skipAvgIfIdenticalSourcePointPresentBwd,
                                          const double  distOffsetFwd,
                                          const double  distOffsetBwd,
                                          double        maxGeometryDist2Fwd,
                                          double        maxGeometryDist2Bwd,
                                          double        maxColorDist2Fwd,
                                          double        maxColorDist2Bwd,
                                          const bool    excludeColorOutlier,
                                          const double  thresholdColorOutlierDist ) const {
  const auto&  source           = *this;
  const size_t pointCountSource = source.getPointCount();
  const size_t pointCountTarget = target.getPointCount();
  if ( ( pointCountSource == 0u ) || ( pointCountTarget == 0u ) || !source.hasColors() ) { return false; }
  PCCKdTree kdtreeTarget( target );
  PCCKdTree kdtreeSource( source );
  target.addColors16bit();
  std::vector<PCCColor16bit> refinedColors1;
  refinedColors1.resize( pointCountTarget );
  maxGeometryDist2Fwd = ( maxGeometryDist2Fwd < 512 ) ? maxGeometryDist2Fwd : std::numeric_limits<double>::max();
  maxGeometryDist2Bwd = ( maxGeometryDist2Bwd < 512 ) ? maxGeometryDist2Bwd : std::numeric_limits<double>::max();
  maxColorDist2Fwd    = ( maxColorDist2Fwd < 131072 ) ? maxColorDist2Fwd : std::numeric_limits<double>::max();
  maxColorDist2Bwd    = ( maxColorDist2Bwd < 131072 ) ? maxColorDist2Bwd : std::numeric_limits<double>::max();
  PCCPointSet3 partSource;
  partSource.addColors();
  // ==========================================================================================
  //                                     Forward direction
  // ==========================================================================================
  // for each target point indexed by index, derive the refined color as
  // refinedColors1[index]
  PCCNNResult result;
  for ( size_t index = 0; index < pointCountTarget; ++index ) {
    PCCColor16bit colorT16bit = target.getColor16bit( index );
    for ( int k = 0; k < 3; ++k ) { refinedColors1[index][k] = colorT16bit[k]; }
    if ( target.getBoundaryPointType( index ) == 3 ) {
      kdtreeSource.search( target[index], numNeighborsColorTransferFwd, result );
      if ( filterType == 1 ) {
        for ( size_t rI = 0; rI < result.size(); ++rI ) {
          auto indexInSource = result.indices( rI );
          auto partIndex2    = partSource.addPoint( source[indexInSource] );
          partSource.setColor( partIndex2, source.getColor( indexInSource ) );
          partSource.setColor16bit( partIndex2, source.getColor16bit( indexInSource ) );
          partSource.setParentPointIndex( partIndex2, indexInSource );
        }
      }
      // keep the points that satisfy geometry dist threshold
      while ( true ) {
        if ( result.size() == 1 ) { break; }
        if ( result.dist( int( result.size() ) - 1 ) <= maxGeometryDist2Fwd ) { break; }
        result.popBack();
      }
      bool isDone = false;
      if ( skipAvgIfIdenticalSourcePointPresentFwd ) {
        if ( result.dist( 0 ) < 0.0001 ) {
          refinedColors1[index] = source.getColor16bit( result.indices( 0 ) );
          isDone                = true;
        }
      }
      if ( !isDone ) {
        int nNN = static_cast<int>( result.count() );
        while ( nNN > 0 && !isDone ) {
          if ( nNN == 1 ) {
            refinedColors1[index] = source.getColor16bit( result.indices( 0 ) );
            isDone                = true;
          }
          if ( !isDone ) {
            std::vector<PCCVector3D> colors;
            colors.resize( 0 );
            colors.resize( nNN );
            for ( int i = 0; i < nNN; ++i ) {
              for ( int k = 0; k < 3; ++k ) { colors[i][k] = double( source.getColor16bit( result.indices( i ) )[k] ); }
            }
            double maxColorDist2 = std::numeric_limits<double>::min();
            for ( int i = 0; i < nNN; ++i ) {
              for ( int j = i + 1; j < nNN; ++j ) {
                const double dist2 = ( colors[i] - colors[j] ).getNorm2();
                if ( dist2 > maxColorDist2 ) { maxColorDist2 = dist2; }
              }
            }
            if ( maxColorDist2 <= maxColorDist2Fwd ) {
              PCCVector3D refinedColor( 0.0 );
              if ( useDistWeightedAverageFwd ) {
                double sumWeights{0.0};
                for ( int i = 0; i < nNN; ++i ) {
                  const double weight = 1 / ( result.dist( i ) + distOffsetFwd );
                  for ( int k = 0; k < 3; ++k ) {
                    refinedColor[k] += source.getColor16bit( result.indices( i ) )[k] * weight;
                  }
                  sumWeights += weight;
                }
                refinedColor /= sumWeights;
                if ( excludeColorOutlier ) {
                  PCCVector3D excludeOutlierRefinedColor( 0.0 );
                  size_t      excludeCount = 0;
                  sumWeights               = 0.0;
                  for ( int i = 0; i < nNN; ++i ) {
                    PCCColor16bit tmpColor = source.getColor16bit( result.indices( i ) );
                    PCCVector3D   sourceColor( tmpColor[0], tmpColor[1], tmpColor[2] );
                    double        dist = ( sourceColor - refinedColor ).getNorm2();
                    if ( dist > thresholdColorOutlierDist * thresholdColorOutlierDist * 256.0 * 256.0 ) {
                      excludeCount += 1;
                      continue;
                    }
                    const double weight = 1 / ( result.dist( i ) + distOffsetFwd );
                    for ( int k = 0; k < 3; ++k ) {
                      excludeOutlierRefinedColor[k] += source.getColor16bit( result.indices( i ) )[k] * weight;
                    }
                    sumWeights += weight;
                  }

                  if ( excludeCount != nNN && excludeCount != 0 ) {
                    refinedColor = excludeOutlierRefinedColor / sumWeights;
                  }
                }
              } else {
                for ( int i = 0; i < nNN; ++i ) {
                  for ( int k = 0; k < 3; ++k ) { refinedColor[k] += source.getColor16bit( result.indices( i ) )[k]; }
                }
                refinedColor /= nNN;
              }
              for ( int k = 0; k < 3; ++k ) {
                refinedColors1[index][k] = uint16_t( PCCClip( round( refinedColor[k] ), 0.0, 65535.0 ) );
              }
              isDone = true;
            } else {
              --nNN;
            }
          }
        }
      }
    }
  }
  // ==========================================================================================
  //                                  Backward direction
  // ==========================================================================================
  // for each target point, derive a vector of source candidate points as
  // colorsDists2.
  // colorsDists2 is iteratively refined (by removing the farthest points) until
  // the
  // std of remaining colors in it is smaller than a threshold.
  std::vector<std::vector<DistColor>> refinedColorsDists2;
  if ( filterType == 1 ) {
    refinedColorsDists2.resize( pointCountTarget );
    // populate refinedColorsDists2
    auto sampleSetPointCount = partSource.getPointCount();
    for ( size_t index = 0; index < sampleSetPointCount; ++index ) {
      const PCCColor16bit color = partSource.getColor16bit( index );
      kdtreeTarget.search( partSource[index], numNeighborsColorTransferBwd, result );
      // keep the points that satisfy geometry dist threshold
      for ( int i = 0; i < result.size(); ++i ) {
        if ( result.dist( i ) <= maxGeometryDist2Bwd ) {
          if ( std::abs( color[0] - target.getColor16bit()[result.indices( i )][0] ) < 40 &&
               std::abs( color[1] - target.getColor16bit()[result.indices( i )][1] ) < 40 &&
               std::abs( color[2] - target.getColor16bit()[result.indices( i )][2] ) < 40 )
            refinedColorsDists2[result.indices( i )].push_back( DistColor{
                result.dist( i ), color, target[result.indices( i )], partSource.getParentPointIndex( index ), index} );
        }
      }
    }

    // sort refinedColorsDists2 according to distance
    for ( size_t index = 0; index < pointCountTarget; ++index ) {
      std::sort( refinedColorsDists2[index].begin(), refinedColorsDists2[index].end(),
                 []( DistColor& dc1, DistColor& dc2 ) { return dc1.dist < dc2.dist; } );
    }
  } else {
    // populate refinedColorsDists2
    refinedColorsDists2.resize( pointCountTarget );
    for ( size_t index = 0; index < pointCountSource; ++index ) {
      const PCCColor16bit color = source.getColor16bit( index );
      kdtreeTarget.search( source[index], numNeighborsColorTransferBwd, result );
      // keep the points that satisfy geometry dist threshold
      for ( int i = 0; i < result.size(); ++i ) {
        if ( result.dist( i ) <= maxGeometryDist2Bwd ) {
          refinedColorsDists2[result.indices( i )].push_back( DistColor{result.dist( i ), color} );
        }
      }
    }
    // sort refinedColorsDists2 according to distance
    for ( size_t index = 0; index < pointCountTarget; ++index ) {
      std::sort( refinedColorsDists2[index].begin(), refinedColorsDists2[index].end(),
                 []( DistColor& dc1, DistColor& dc2 ) { return dc1.dist < dc2.dist; } );
    }
  }
  // compute centroid2
  for ( size_t index = 0; index < pointCountTarget; ++index ) {
    if ( filterType == 1 && target.getBoundaryPointType( index ) != 3 ) continue;
    const PCCColor16bit color1       = refinedColors1[index];       // refined color derived in forward direction
    auto&               colorsDists2 = refinedColorsDists2[index];  // set of candidate points
                                                                    // derived in backward
                                                                    // direction
    if ( colorsDists2.empty() || losslessAttribute ) {
      target.setColor16bit( index, color1 );
    } else {
      bool              isDone = false;
      const PCCVector3D centroid1( color1[0], color1[1], color1[2] );
      PCCVector3D       centroid2( 0.0 );
      if ( skipAvgIfIdenticalSourcePointPresentBwd ) {
        if ( colorsDists2[0].dist < 0.0001 ) {
          auto temp = colorsDists2[0];
          colorsDists2.clear();
          colorsDists2.push_back( temp );
          for ( int k = 0; k < 3; ++k ) { centroid2[k] = colorsDists2[0].color[k]; }
          isDone = true;
        }
      }
      if ( !isDone ) {
        int nNN = static_cast<int>( colorsDists2.size() );
        while ( nNN > 0 && !isDone ) {
          nNN = static_cast<int>( colorsDists2.size() );
          if ( nNN == 1 ) {
            auto temp = colorsDists2[0];
            colorsDists2.clear();
            colorsDists2.push_back( temp );
            for ( int k = 0; k < 3; ++k ) { centroid2[k] = colorsDists2[0].color[k]; }
            isDone = true;
          }
          if ( !isDone ) {
            std::vector<PCCVector3D> colors;
            colors.resize( 0 );
            colors.resize( nNN );
            for ( int i = 0; i < nNN; ++i ) {
              for ( int k = 0; k < 3; ++k ) { colors[i][k] = double( colorsDists2[i].color[k] ); }
            }
            double maxColorDist2 = std::numeric_limits<double>::min();
            for ( int i = 0; i < nNN; ++i ) {
              for ( int j = i + 1; j < nNN; ++j ) {
                const double dist2 = ( colors[i] - colors[j] ).getNorm2();
                if ( dist2 > maxColorDist2 ) { maxColorDist2 = dist2; }
              }
            }
            if ( maxColorDist2 <= maxColorDist2Bwd ) {
              for ( size_t k = 0; k < 3; ++k ) { centroid2[k] = 0; }
              if ( useDistWeightedAverageBwd ) {
                double sumWeights{0.0};
                for ( auto& i : colorsDists2 ) {
                  const double weight = 1 / ( sqrt( i.dist ) + distOffsetBwd );
                  for ( size_t k = 0; k < 3; ++k ) { centroid2[k] += ( i.color[k] * weight ); }
                  sumWeights += weight;
                }
                centroid2 /= sumWeights;
                if ( excludeColorOutlier ) {
                  PCCVector3D excludeOutlierCentroid2( 0.0 );
                  size_t      excludeCount = 0;
                  sumWeights               = 0.0;
                  for ( auto& i : colorsDists2 ) {
                    PCCVector3D sourceColor( i.color[0], i.color[1], i.color[2] );
                    double      dist = ( sourceColor - centroid2 ).getNorm2();
                    if ( dist > thresholdColorOutlierDist * thresholdColorOutlierDist * 256.0 * 256.0 ) {
                      excludeCount += 1;
                      continue;
                    }
                    const double weight = 1 / ( sqrt( i.dist ) + distOffsetBwd );
                    for ( size_t k = 0; k < 3; ++k ) { excludeOutlierCentroid2[k] += ( i.color[k] * weight ); }
                    sumWeights += weight;
                  }

                  if ( excludeCount != nNN && excludeCount != 0 ) { centroid2 = excludeOutlierCentroid2 / sumWeights; }
                }
              } else {
                for ( auto& coldist : colorsDists2 ) {
                  for ( int k = 0; k < 3; ++k ) { centroid2[k] += coldist.color[k]; }
                }
                centroid2 /= colorsDists2.size();
              }
              isDone = true;
            } else {
              colorsDists2.pop_back();
            }
          }
        }
      }
      auto   H  = double( colorsDists2.size() );
      double D2 = 0.0;
      for ( const auto& color2dist : colorsDists2 ) {
        auto color2 = color2dist.color;
        for ( size_t k = 0; k < 3; ++k ) {
          const double d2 = centroid2[k] - color2[k];
          D2 += d2 * d2;
        }
      }
      const double r      = double( pointCountTarget ) / double( pointCountSource );
      const double delta2 = ( centroid2 - centroid1 ).getNorm2();
      const double eps    = 0.000001;

      const bool fixWeight = true;        // m42538
      if ( fixWeight || delta2 > eps ) {  // centroid2 != centroid1
        double w = 0.0;

        if ( !fixWeight ) {
          const double alpha = D2 / delta2;
          const double a     = H * r - 1.0;
          const double c     = alpha * r - 1.0;
          if ( fabs( a ) < eps ) {
            w = -0.5 * c;
          } else {
            const double delta = 1.0 - a * c;
            if ( delta >= 0.0 ) { w = ( -1.0 + sqrt( delta ) ) / a; }
          }
        }
        const double oneMinusW = 1.0 - w;
        PCCVector3D  color0;
        for ( size_t k = 0; k < 3; ++k ) {
          color0[k] = PCCClip( round( w * centroid1[k] + oneMinusW * centroid2[k] ), 0.0, 65535.0 );
        }
        const double rSource  = 1.0 / double( pointCountSource );
        const double rTarget  = 1.0 / double( pointCountTarget );
        const double maxValue = std::numeric_limits<uint16_t>::max();
        double       minError = std::numeric_limits<double>::max();
        PCCVector3D  bestColor( color0 );
        PCCVector3D  color;
        for ( int32_t s1 = -searchRange; s1 <= searchRange; ++s1 ) {
          color[0] = PCCClip( color0[0] + s1, 0.0, maxValue );
          for ( int32_t s2 = -searchRange; s2 <= searchRange; ++s2 ) {
            color[1] = PCCClip( color0[1] + s2, 0.0, maxValue );
            for ( int32_t s3 = -searchRange; s3 <= searchRange; ++s3 ) {
              color[2] = PCCClip( color0[2] + s3, 0.0, maxValue );

              double e1 = 0.0;
              for ( size_t k = 0; k < 3; ++k ) {
                const double d = color[k] - color1[k];
                e1 += d * d;
              }
              e1 *= rTarget;

              double e2 = 0.0;
              for ( const auto& color2dist : colorsDists2 ) {
                auto color2 = color2dist.color;
                for ( size_t k = 0; k < 3; ++k ) {
                  const double d = color[k] - color2[k];
                  e2 += d * d;
                }
              }
              e2 *= rSource;

              const double error = std::max( e1, e2 );
              if ( error < minError ) {
                minError  = error;
                bestColor = color;
              }
            }
          }
        }
        target.setColor16bit(
            index, PCCColor16bit( uint16_t( bestColor[0] ), uint16_t( bestColor[1] ), uint16_t( bestColor[2] ) ) );
      } else {  // centroid2 == centroid1
        target.setColor16bit( index, color1 );
      }
    }
  }
  return true;
}

bool PCCPointSet3::transferColorsBackward16bitBP( PCCPointSet3& target,
                                                  const int     filterType,
                                                  const int32_t searchRange,
                                                  const bool    losslessAttribute,
                                                  const int     numNeighborsColorTransferFwd,
                                                  const int     numNeighborsColorTransferBwd,
                                                  const bool    useDistWeightedAverageFwd,
                                                  const bool    useDistWeightedAverageBwd,
                                                  const bool    skipAvgIfIdenticalSourcePointPresentFwd,
                                                  const bool    skipAvgIfIdenticalSourcePointPresentBwd,
                                                  const double  distOffsetFwd,
                                                  const double  distOffsetBwd,
                                                  double        maxGeometryDist2Fwd,
                                                  double        maxGeometryDist2Bwd,
                                                  double        maxColorDist2Fwd,
                                                  double        maxColorDist2Bwd,
                                                  const bool    excludeColorOutlier,
                                                  const double  thresholdColorOutlierDist ) const {
  const auto&  source           = *this;
  const size_t pointCountSource = source.getPointCount();
  const size_t pointCountTarget = target.getPointCount();
  if ( ( pointCountSource == 0u ) || ( pointCountTarget == 0u ) || !source.hasColors() ) { return false; }
  PCCKdTree kdtreeTarget( target );
  PCCKdTree kdtreeSource( source );
  target.addColors16bit();
  maxGeometryDist2Fwd = ( maxGeometryDist2Fwd < 512 ) ? maxGeometryDist2Fwd : std::numeric_limits<double>::max();
  maxGeometryDist2Bwd = ( maxGeometryDist2Bwd < 512 ) ? maxGeometryDist2Bwd : std::numeric_limits<double>::max();
  maxColorDist2Fwd    = ( maxColorDist2Fwd < 131072 ) ? maxColorDist2Fwd : std::numeric_limits<double>::max();
  maxColorDist2Bwd    = ( maxColorDist2Bwd < 131072 ) ? maxColorDist2Bwd : std::numeric_limits<double>::max();

  // ==========================================================================================
  // backward search first
  // ==========================================================================================
  PCCNNResult       result;
  std::vector<bool> newValueDecided;
  newValueDecided.resize( pointCountTarget, false );
  std::vector<PCCColor16bit> refinedColors1;
  refinedColors1.resize( pointCountTarget );

  PCCPointSet3 partTarget;
  partTarget.addColors();
  if ( filterType == 9 ) {
    for ( size_t index = 0; index < pointCountTarget; ++index ) {
      if ( target.getBoundaryPointType( index ) == 3 ) {
        partTarget.getPositions().push_back( target[index] );
        partTarget.getColor().push_back( target.getColor( index ) );
        partTarget.getColor16bit().push_back( target.getColor16bit( index ) );
        partTarget.getParentPointIndex().push_back( index );
      }
    }
  }

  PCCKdTree kdtreePartTarget( partTarget );

  //////
  std::vector<std::vector<DistColor>> refinedColorsDists2;
  // populate refinedColorsDists2
  refinedColorsDists2.resize( pointCountTarget );
  for ( size_t index = 0; index < pointCountSource; ++index ) {
    const PCCColor16bit color = source.getColor16bit( index );
    if ( target.getBoundaryPointType( index ) != 3 ) continue;
    if ( filterType == 9 ) {
      kdtreePartTarget.search( source[index], numNeighborsColorTransferBwd, result );
      for ( int i = 0; i < result.count(); ++i ) {
        if ( result.dist( i ) <= maxGeometryDist2Bwd &&
             ( std::abs( color[0] - partTarget.getColor16bit()[result.indices( i )][0] ) < 40 &&
               std::abs( color[1] - partTarget.getColor16bit()[result.indices( i )][1] ) < 40 &&
               std::abs( color[2] - partTarget.getColor16bit()[result.indices( i )][2] ) < 40 ) ) {
          auto indexInTarget = partTarget.getParentPointIndex( result.indices( i ) );
          if ( target.getBoundaryPointType( indexInTarget ) != 3 ) {
            printf( "something wrong!!\n" );
            assert( 0 );
            exit( 0 );
          }
          refinedColorsDists2[indexInTarget].push_back(
              DistColor{result.dist( i ), color, partTarget[result.indices( i )], indexInTarget, result.indices( i )} );
        }
      }
    } else {
      kdtreeTarget.search( source[index], numNeighborsColorTransferBwd, result );
      // keep the points that satisfy geometry dist threshold
      for ( int i = 0; i < result.count(); ++i ) {
        if ( result.dist( i ) <= maxGeometryDist2Bwd ) {
          refinedColorsDists2[result.indices( i )].push_back( DistColor{result.dist( i ), color} );
        }
      }
    }
  }

  for ( size_t index = 0; index < pointCountTarget; ++index ) {
    std::sort( refinedColorsDists2[index].begin(), refinedColorsDists2[index].end(),
               []( DistColor& dc1, DistColor& dc2 ) { return dc1.dist < dc2.dist; } );
  }

  // compute centroid2
  for ( size_t index = 0; index < pointCountTarget; ++index ) {
    if ( target.getBoundaryPointType( index ) != 3 ) {
      newValueDecided[index] = true;
      continue;
    }
    if ( refinedColorsDists2[index].empty() ) { continue; }
    auto&       colorsDists2 = refinedColorsDists2[index];
    bool        isDone       = false;
    PCCVector3D centroid2( 0.0 );
    if ( skipAvgIfIdenticalSourcePointPresentBwd ) {
      if ( colorsDists2[0].dist < 0.0001 ) {
        auto temp = colorsDists2[0];
        colorsDists2.clear();
        colorsDists2.push_back( temp );
        for ( int k = 0; k < 3; ++k ) { centroid2[k] = colorsDists2[0].color[k]; }
        isDone = true;
      }
    }
    if ( !isDone ) {
      int nNN = static_cast<int>( colorsDists2.size() );
      while ( nNN > 0 && !isDone ) {
        nNN = static_cast<int>( colorsDists2.size() );
        if ( nNN == 1 ) {
          auto temp = colorsDists2[0];
          colorsDists2.clear();
          colorsDists2.push_back( temp );
          for ( int k = 0; k < 3; ++k ) { centroid2[k] = colorsDists2[0].color[k]; }
          isDone = true;
        }
        if ( !isDone ) {
          std::vector<PCCVector3D> colors;
          colors.resize( 0 );
          colors.resize( nNN );
          for ( int i = 0; i < nNN; ++i ) {
            for ( int k = 0; k < 3; ++k ) { colors[i][k] = double( colorsDists2[i].color[k] ); }
          }
          double maxColorDist2 = std::numeric_limits<double>::min();
          for ( int i = 0; i < nNN; ++i ) {
            for ( int j = i + 1; j < nNN; ++j ) {
              const double dist2 = ( colors[i] - colors[j] ).getNorm2();
              if ( dist2 > maxColorDist2 ) { maxColorDist2 = dist2; }
            }
          }
          if ( maxColorDist2 <= maxColorDist2Bwd ) {
            for ( size_t k = 0; k < 3; ++k ) { centroid2[k] = 0; }
            if ( useDistWeightedAverageBwd ) {
              double sumWeights{0.0};
              for ( auto& i : colorsDists2 ) {
                const double weight = 1 / ( sqrt( i.dist ) + distOffsetBwd );
                for ( size_t k = 0; k < 3; ++k ) { centroid2[k] += ( i.color[k] * weight ); }
                sumWeights += weight;
              }
              centroid2 /= sumWeights;
              if ( excludeColorOutlier ) {
                PCCVector3D excludeOutlierCentroid2( 0.0 );
                size_t      excludeCount = 0;
                sumWeights               = 0.0;
                for ( auto& i : colorsDists2 ) {
                  PCCVector3D sourceColor( i.color[0], i.color[1], i.color[2] );
                  double      dist = ( sourceColor - centroid2 ).getNorm2();
                  if ( dist > thresholdColorOutlierDist * thresholdColorOutlierDist * 256.0 * 256.0 ) {
                    excludeCount += 1;
                    continue;
                  }
                  const double weight = 1 / ( sqrt( i.dist ) + distOffsetBwd );
                  for ( size_t k = 0; k < 3; ++k ) { excludeOutlierCentroid2[k] += ( i.color[k] * weight ); }
                  sumWeights += weight;
                }

                if ( excludeCount != nNN && excludeCount != 0 ) { centroid2 = excludeOutlierCentroid2 / sumWeights; }
              }
            } else {
              for ( auto& coldist : colorsDists2 ) {
                for ( int k = 0; k < 3; ++k ) { centroid2[k] += coldist.color[k]; }
              }
              centroid2 /= colorsDists2.size();
            }
            isDone = true;
          } else {
            colorsDists2.pop_back();
          }
        }
      }
    }

    PCCVector3D color0;
    for ( size_t k = 0; k < 3; ++k ) { color0[k] = PCCClip( round( centroid2[k] ), 0.0, 65535.0 ); }
    target.setColor16bit( index, PCCColor16bit( uint16_t( color0[0] ), uint16_t( color0[1] ), uint16_t( color0[2] ) ) );
    newValueDecided[index] = true;
  }

  // ==========================================================================================
  //                                     Forward direction
  // ==========================================================================================
  // for each target point indexed by index, derive the refined color as
  // refinedColors1[index]

  for ( size_t index = 0; index < pointCountTarget; ++index ) {
    PCCColor16bit colorT16bit = target.getColor16bit( index );
    for ( int k = 0; k < 3; ++k ) { refinedColors1[index][k] = colorT16bit[k]; }
    if ( target.getBoundaryPointType( index ) == 3 && newValueDecided[index] == false ) {
      kdtreeSource.search( target[index], numNeighborsColorTransferFwd, result );
      // keep the points that satisfy geometry dist threshold
      while ( true ) {
        if ( result.count() == 1 ) { break; }
        if ( result.dist( int( result.size() ) - 1 ) <= maxGeometryDist2Fwd ) { break; }
        result.popBack();
      }
      bool isDone = false;
      if ( skipAvgIfIdenticalSourcePointPresentFwd ) {
        if ( result.dist( 0 ) < 0.0001 ) {
          refinedColors1[index] = source.getColor16bit( result.indices( 0 ) );
          isDone                = true;
        }
      }
      if ( !isDone ) {
        int nNN = static_cast<int>( result.count() );
        while ( nNN > 0 && !isDone ) {
          if ( nNN == 1 ) {
            refinedColors1[index] = source.getColor16bit( result.indices( 0 ) );
            isDone                = true;
          }
          if ( !isDone ) {
            std::vector<PCCVector3D> colors;
            colors.resize( 0 );
            colors.resize( nNN );
            for ( int i = 0; i < nNN; ++i ) {
              for ( int k = 0; k < 3; ++k ) { colors[i][k] = double( source.getColor16bit( result.indices( i ) )[k] ); }
            }
            double maxColorDist2 = std::numeric_limits<double>::min();
            for ( int i = 0; i < nNN; ++i ) {
              for ( int j = i + 1; j < nNN; ++j ) {
                const double dist2 = ( colors[i] - colors[j] ).getNorm2();
                if ( dist2 > maxColorDist2 ) { maxColorDist2 = dist2; }
              }
            }
            if ( maxColorDist2 <= maxColorDist2Fwd ) {
              PCCVector3D refinedColor( 0.0 );
              if ( useDistWeightedAverageFwd ) {
                double sumWeights{0.0};
                for ( int i = 0; i < nNN; ++i ) {
                  const double weight = 1 / ( result.dist( i ) + distOffsetFwd );
                  for ( int k = 0; k < 3; ++k ) {
                    refinedColor[k] += source.getColor16bit( result.indices( i ) )[k] * weight;
                  }
                  sumWeights += weight;
                }
                refinedColor /= sumWeights;
                if ( excludeColorOutlier ) {
                  PCCVector3D excludeOutlierRefinedColor( 0.0 );
                  size_t      excludeCount = 0;
                  sumWeights               = 0.0;
                  for ( int i = 0; i < nNN; ++i ) {
                    PCCColor16bit tmpColor = source.getColor16bit( result.indices( i ) );
                    PCCVector3D   sourceColor( tmpColor[0], tmpColor[1], tmpColor[2] );
                    double        dist = ( sourceColor - refinedColor ).getNorm2();
                    if ( dist > thresholdColorOutlierDist * thresholdColorOutlierDist * 256.0 * 256.0 ) {
                      excludeCount += 1;
                      continue;
                    }
                    const double weight = 1 / ( result.dist( i ) + distOffsetFwd );
                    for ( int k = 0; k < 3; ++k ) {
                      excludeOutlierRefinedColor[k] += source.getColor16bit( result.indices( i ) )[k] * weight;
                    }
                    sumWeights += weight;
                  }

                  if ( excludeCount != nNN && excludeCount != 0 ) {
                    refinedColor = excludeOutlierRefinedColor / sumWeights;
                  }
                }
              } else {
                for ( int i = 0; i < nNN; ++i ) {
                  for ( int k = 0; k < 3; ++k ) { refinedColor[k] += source.getColor16bit( result.indices( i ) )[k]; }
                }
                refinedColor /= nNN;
              }
              for ( int k = 0; k < 3; ++k ) {
                refinedColors1[index][k] = uint16_t( PCCClip( round( refinedColor[k] ), 0.0, 65535.0 ) );
              }
              isDone = true;
            } else {
              --nNN;
            }
          }
        }  // while
      }    //! isDone

      target.setColor16bit( index,
                            PCCColor16bit( uint16_t( refinedColors1[index][0] ), uint16_t( refinedColors1[index][1] ),
                                           uint16_t( refinedColors1[index][2] ) ) );
    }  // if ( target.getBoundaryPointType( index ) == 3 && newValueDecided[ index ] == false )
  }    // index

  return true;
}
bool PCCPointSet3::transferColors16bit( PCCPointSet3& target,
                                        const int32_t searchRange,
                                        const bool    losslessAttribute,
                                        const int     numNeighborsColorTransferFwd,
                                        const int     numNeighborsColorTransferBwd,
                                        const bool    useDistWeightedAverageFwd,
                                        const bool    useDistWeightedAverageBwd,
                                        const bool    skipAvgIfIdenticalSourcePointPresentFwd,
                                        const bool    skipAvgIfIdenticalSourcePointPresentBwd,
                                        const double  distOffsetFwd,
                                        const double  distOffsetBwd,
                                        double        maxGeometryDist2Fwd,
                                        double        maxGeometryDist2Bwd,
                                        double        maxColorDist2Fwd,
                                        double        maxColorDist2Bwd,
                                        const bool    excludeColorOutlier,
                                        const double  thresholdColorOutlierDist ) const {
  printf( "transferColors16bit \n" );
  const auto&  source           = *this;
  const size_t pointCountSource = source.getPointCount();
  const size_t pointCountTarget = target.getPointCount();
  if ( ( pointCountSource == 0u ) || ( pointCountTarget == 0u ) || !source.hasColors() ) { return false; }
  PCCKdTree kdtreeTarget( target );
  PCCKdTree kdtreeSource( source );
  target.addColors16bit();
  std::vector<PCCColor16bit> refinedColors1;
  refinedColors1.resize( pointCountTarget );
  maxGeometryDist2Fwd = ( maxGeometryDist2Fwd < 512 ) ? maxGeometryDist2Fwd : std::numeric_limits<double>::max();
  maxGeometryDist2Bwd = ( maxGeometryDist2Bwd < 512 ) ? maxGeometryDist2Bwd : std::numeric_limits<double>::max();
  maxColorDist2Fwd    = ( maxColorDist2Fwd < 131072 ) ? maxColorDist2Fwd : std::numeric_limits<double>::max();
  maxColorDist2Bwd    = ( maxColorDist2Bwd < 131072 ) ? maxColorDist2Bwd : std::numeric_limits<double>::max();

  // ==========================================================================================
  //                                     Forward direction
  // ==========================================================================================
  // for each target point indexed by index, derive the refined color as
  // refinedColors1[index]
  PCCNNResult result;
  for ( size_t index = 0; index < pointCountTarget; ++index ) {
    kdtreeSource.search( target[index], numNeighborsColorTransferFwd, result );
    // keep the points that satisfy geometry dist threshold
    while ( true ) {
      if ( result.size() == 1 ) { break; }
      if ( result.dist( int( result.size() ) - 1 ) <= maxGeometryDist2Fwd ) { break; }
      result.popBack();
    }
    bool isDone = false;
    if ( skipAvgIfIdenticalSourcePointPresentFwd ) {
      if ( result.dist( 0 ) < 0.0001 ) {
        refinedColors1[index] = source.getColor16bit( result.indices( 0 ) );
        isDone                = true;
      }
    }
    if ( !isDone ) {
      int nNN = static_cast<int>( result.size() );
      while ( nNN > 0 && !isDone ) {
        if ( nNN == 1 ) {
          refinedColors1[index] = source.getColor16bit( result.indices( 0 ) );
          isDone                = true;
        }
        if ( !isDone ) {
          std::vector<PCCVector3D> colors;
          colors.resize( 0 );
          colors.resize( nNN );
          for ( int i = 0; i < nNN; ++i ) {
            for ( int k = 0; k < 3; ++k ) { colors[i][k] = double( source.getColor16bit( result.indices( i ) )[k] ); }
          }
          double maxColorDist2 = std::numeric_limits<double>::min();
          for ( int i = 0; i < nNN; ++i ) {
            for ( int j = i + 1; j < nNN; ++j ) {
              const double dist2 = ( colors[i] - colors[j] ).getNorm2();
              if ( dist2 > maxColorDist2 ) { maxColorDist2 = dist2; }
            }
          }
          if ( maxColorDist2 <= maxColorDist2Fwd ) {
            PCCVector3D refinedColor( 0.0 );
            if ( useDistWeightedAverageFwd ) {
              double sumWeights{0.0};
              for ( int i = 0; i < nNN; ++i ) {
                const double weight = 1 / ( result.dist( i ) + distOffsetFwd );
                for ( int k = 0; k < 3; ++k ) {
                  refinedColor[k] += source.getColor16bit( result.indices( i ) )[k] * weight;
                }
                sumWeights += weight;
              }
              refinedColor /= sumWeights;
              if ( excludeColorOutlier ) {
                PCCVector3D excludeOutlierRefinedColor( 0.0 );
                size_t      excludeCount = 0;
                sumWeights               = 0.0;
                for ( int i = 0; i < nNN; ++i ) {
                  PCCColor16bit tmpColor = source.getColor16bit( result.indices( i ) );
                  PCCVector3D   sourceColor( tmpColor[0], tmpColor[1], tmpColor[2] );
                  double        dist = ( sourceColor - refinedColor ).getNorm2();
                  if ( dist > thresholdColorOutlierDist * thresholdColorOutlierDist * 256.0 * 256.0 ) {
                    excludeCount += 1;
                    continue;
                  }
                  const double weight = 1 / ( result.dist( i ) + distOffsetFwd );
                  for ( int k = 0; k < 3; ++k ) {
                    excludeOutlierRefinedColor[k] += source.getColor16bit( result.indices( i ) )[k] * weight;
                  }
                  sumWeights += weight;
                }

                if ( excludeCount != nNN && excludeCount != 0 ) {
                  refinedColor = excludeOutlierRefinedColor / sumWeights;
                }
              }
            } else {
              for ( int i = 0; i < nNN; ++i ) {
                for ( int k = 0; k < 3; ++k ) { refinedColor[k] += source.getColor16bit( result.indices( i ) )[k]; }
              }
              refinedColor /= nNN;
            }
            for ( int k = 0; k < 3; ++k ) {
              refinedColors1[index][k] = uint16_t( PCCClip( round( refinedColor[k] ), 0.0, 65535.0 ) );
            }
            isDone = true;
          } else {
            --nNN;
          }
        }
      }
    }
  }
  // ==========================================================================================
  //                                  Backward direction
  // ==========================================================================================
  // for each target point, derive a vector of source candidate points as
  // colorsDists2.
  // colorsDists2 is iteratively refined (by removing the farthest points) until
  // the
  // std of remaining colors in it is smaller than a threshold.
  std::vector<std::vector<DistColor>> refinedColorsDists2;
  refinedColorsDists2.resize( pointCountTarget );
  // populate refinedColorsDists2
  for ( size_t index = 0; index < pointCountSource; ++index ) {
    const PCCColor16bit color = source.getColor16bit( index );
    kdtreeTarget.search( source[index], numNeighborsColorTransferBwd, result );
    // keep the points that satisfy geometry dist threshold
    for ( int i = 0; i < result.size(); ++i ) {
      if ( result.dist( i ) <= maxGeometryDist2Bwd ) {
        refinedColorsDists2[result.indices( i )].push_back( DistColor{result.dist( i ), color} );
      }
    }
  }
  // sort refinedColorsDists2 according to distance
  for ( size_t index = 0; index < pointCountTarget; ++index ) {
    std::sort( refinedColorsDists2[index].begin(), refinedColorsDists2[index].end(),
               []( DistColor& dc1, DistColor& dc2 ) { return dc1.dist < dc2.dist; } );
  }
  // compute centroid2
  for ( size_t index = 0; index < pointCountTarget; ++index ) {
    const PCCColor16bit color1       = refinedColors1[index];       // refined color derived in forward direction
    auto&               colorsDists2 = refinedColorsDists2[index];  // set of candidate points
                                                                    // derived in backward
                                                                    // direction
    if ( colorsDists2.empty() || losslessAttribute ) {
      target.setColor16bit( index, color1 );
    } else {
      bool              isDone = false;
      const PCCVector3D centroid1( color1[0], color1[1], color1[2] );
      PCCVector3D       centroid2( 0.0 );
      if ( skipAvgIfIdenticalSourcePointPresentBwd ) {
        if ( colorsDists2[0].dist < 0.0001 ) {
          auto temp = colorsDists2[0];
          colorsDists2.clear();
          colorsDists2.push_back( temp );
          for ( int k = 0; k < 3; ++k ) { centroid2[k] = colorsDists2[0].color[k]; }
          isDone = true;
        }
      }
      if ( !isDone ) {
        int nNN = static_cast<int>( colorsDists2.size() );
        while ( nNN > 0 && !isDone ) {
          nNN = static_cast<int>( colorsDists2.size() );
          if ( nNN == 1 ) {
            auto temp = colorsDists2[0];
            colorsDists2.clear();
            colorsDists2.push_back( temp );
            for ( int k = 0; k < 3; ++k ) { centroid2[k] = colorsDists2[0].color[k]; }
            isDone = true;
          }
          if ( !isDone ) {
            std::vector<PCCVector3D> colors;
            colors.resize( 0 );
            colors.resize( nNN );
            for ( int i = 0; i < nNN; ++i ) {
              for ( int k = 0; k < 3; ++k ) { colors[i][k] = double( colorsDists2[i].color[k] ); }
            }
            double maxColorDist2 = std::numeric_limits<double>::min();
            for ( int i = 0; i < nNN; ++i ) {
              for ( int j = i + 1; j < nNN; ++j ) {
                const double dist2 = ( colors[i] - colors[j] ).getNorm2();
                if ( dist2 > maxColorDist2 ) { maxColorDist2 = dist2; }
              }
            }
            if ( maxColorDist2 <= maxColorDist2Bwd ) {
              for ( size_t k = 0; k < 3; ++k ) { centroid2[k] = 0; }
              if ( useDistWeightedAverageBwd ) {
                double sumWeights{0.0};
                for ( auto& i : colorsDists2 ) {
                  const double weight = 1 / ( sqrt( i.dist ) + distOffsetBwd );
                  for ( size_t k = 0; k < 3; ++k ) { centroid2[k] += ( i.color[k] * weight ); }
                  sumWeights += weight;
                }
                centroid2 /= sumWeights;
                if ( excludeColorOutlier ) {
                  PCCVector3D excludeOutlierCentroid2( 0.0 );
                  size_t      excludeCount = 0;
                  sumWeights               = 0.0;
                  for ( auto& i : colorsDists2 ) {
                    PCCVector3D sourceColor( i.color[0], i.color[1], i.color[2] );
                    double      dist = ( sourceColor - centroid2 ).getNorm2();
                    if ( dist > thresholdColorOutlierDist * thresholdColorOutlierDist * 256.0 * 256.0 ) {
                      excludeCount += 1;
                      continue;
                    }
                    const double weight = 1 / ( sqrt( i.dist ) + distOffsetBwd );
                    for ( size_t k = 0; k < 3; ++k ) { excludeOutlierCentroid2[k] += ( i.color[k] * weight ); }
                    sumWeights += weight;
                  }

                  if ( excludeCount != nNN && excludeCount != 0 ) { centroid2 = excludeOutlierCentroid2 / sumWeights; }
                }
              } else {
                for ( auto& coldist : colorsDists2 ) {
                  for ( int k = 0; k < 3; ++k ) { centroid2[k] += coldist.color[k]; }
                }
                centroid2 /= colorsDists2.size();
              }
              isDone = true;
            } else {
              colorsDists2.pop_back();
            }
          }
        }
      }
      auto   H  = double( colorsDists2.size() );
      double D2 = 0.0;
      for ( const auto& color2dist : colorsDists2 ) {
        auto color2 = color2dist.color;
        for ( size_t k = 0; k < 3; ++k ) {
          const double d2 = centroid2[k] - color2[k];
          D2 += d2 * d2;
        }
      }
      const double r      = double( pointCountTarget ) / double( pointCountSource );
      const double delta2 = ( centroid2 - centroid1 ).getNorm2();
      const double eps    = 0.000001;

      const bool fixWeight = true;        // m42538
      if ( fixWeight || delta2 > eps ) {  // centroid2 != centroid1
        double w = 0.0;

        if ( !fixWeight ) {
          const double alpha = D2 / delta2;
          const double a     = H * r - 1.0;
          const double c     = alpha * r - 1.0;
          if ( fabs( a ) < eps ) {
            w = -0.5 * c;
          } else {
            const double delta = 1.0 - a * c;
            if ( delta >= 0.0 ) { w = ( -1.0 + sqrt( delta ) ) / a; }
          }
        }
        const double oneMinusW = 1.0 - w;
        PCCVector3D  color0;
        for ( size_t k = 0; k < 3; ++k ) {
          color0[k] = PCCClip( round( w * centroid1[k] + oneMinusW * centroid2[k] ), 0.0, 65535.0 );
        }
        const double rSource  = 1.0 / double( pointCountSource );
        const double rTarget  = 1.0 / double( pointCountTarget );
        const double maxValue = std::numeric_limits<uint16_t>::max();
        double       minError = std::numeric_limits<double>::max();
        PCCVector3D  bestColor( color0 );
        PCCVector3D  color;
        for ( int32_t s1 = -searchRange; s1 <= searchRange; ++s1 ) {
          color[0] = PCCClip( color0[0] + s1, 0.0, maxValue );
          for ( int32_t s2 = -searchRange; s2 <= searchRange; ++s2 ) {
            color[1] = PCCClip( color0[1] + s2, 0.0, maxValue );
            for ( int32_t s3 = -searchRange; s3 <= searchRange; ++s3 ) {
              color[2] = PCCClip( color0[2] + s3, 0.0, maxValue );

              double e1 = 0.0;
              for ( size_t k = 0; k < 3; ++k ) {
                const double d = color[k] - color1[k];
                e1 += d * d;
              }
              e1 *= rTarget;

              double e2 = 0.0;
              for ( const auto& color2dist : colorsDists2 ) {
                auto color2 = color2dist.color;
                for ( size_t k = 0; k < 3; ++k ) {
                  const double d = color[k] - color2[k];
                  e2 += d * d;
                }
              }
              e2 *= rSource;

              const double error = std::max( e1, e2 );
              if ( error < minError ) {
                minError  = error;
                bestColor = color;
              }
            }
          }
        }
        target.setColor16bit(
            index, PCCColor16bit( uint16_t( bestColor[0] ), uint16_t( bestColor[1] ), uint16_t( bestColor[2] ) ) );
      } else {  // centroid2 == centroid1
        target.setColor16bit( index, color1 );
      }
    }
  }
  return true;
}
bool PCCPointSet3::transferColorsFilter3( PCCPointSet3& target,
                                          const int32_t searchRange,
                                          const bool    losslessAttribute ) const {
  printf( "transferColorsFilter3 \n" );
  const auto&  source           = *this;
  const size_t pointCountSource = source.getPointCount();
  const size_t pointCountTarget = target.getPointCount();
  if ( ( pointCountSource == 0u ) || ( pointCountTarget == 0u ) || !source.hasColors() ) { return false; }

  PCCKdTree kdtreeTarget( target );
  PCCKdTree kdtreeSource( source );
  target.addColors();
  std::vector<PCCColor3B>              refinedColors1;
  std::vector<std::vector<PCCColor3B>> refinedColors2;
  refinedColors1.resize( pointCountTarget );
  refinedColors2.resize( pointCountTarget );
  const size_t num_results = 1;
  PCCNNResult  result;
  //  Find THE closest point in reconstruction to each source point
  for ( size_t index = 0; index < pointCountTarget; ++index ) {
    kdtreeSource.search( target[index], num_results, result );
    refinedColors1[index] = source.getColor( result.indices( 0 ) );
  }
  //  Find points in source that are closest to point in reconstruction
  for ( size_t index = 0; index < pointCountSource; ++index ) {
    const PCCColor3B color = source.getColor( index );
    kdtreeTarget.search( source[index], num_results, result );
    refinedColors2[result.indices( 0 )].push_back( color );
  }

  for ( size_t index = 0; index < pointCountTarget; ++index ) {
    const PCCColor3B               color1  = refinedColors1[index];
    const std::vector<PCCColor3B>& colors2 = refinedColors2[index];
    if ( colors2.empty() || losslessAttribute ) {
      target.setColor( index, color1 );
    } else {
      const auto        H = double( colors2.size() );
      const PCCVector3D centroid1( color1[0], color1[1], color1[2] );
      PCCVector3D       centroid2( 0.0 );
      for ( const auto& color2 : colors2 ) {
        for ( size_t k = 0; k < 3; ++k ) { centroid2[k] += color2[k]; }
      }
      centroid2 /= H;

      double D2 = 0.0;
      for ( const auto& color2 : colors2 ) {
        for ( size_t k = 0; k < 3; ++k ) {
          const double d2 = centroid2[k] - color2[k];
          D2 += d2 * d2;
        }
      }
      //      const double r = double(pointCountTarget) /
      // double(pointCountSource);
      const double delta2 = ( centroid2 - centroid1 ).getNorm2();
      const double eps    = 0.000001;

      const bool fixWeight = true;        // m42538
      if ( fixWeight || delta2 > eps ) {  // centroid2 != centroid1
        double w = 0.0;

        const double oneMinusW = 1.0 - w;
        PCCVector3D  color0;
        for ( size_t k = 0; k < 3; ++k ) {
          color0[k] = PCCClip( round( w * centroid1[k] + oneMinusW * centroid2[k] ), 0.0, 65535.0 );
        }
        PCCVector3D bestColor( color0 );
        target.setColor( index,
                         PCCColor3B( uint8_t( bestColor[0] ), uint8_t( bestColor[1] ), uint8_t( bestColor[2] ) ) );
      } else {  // centroid2 == centroid1
        target.setColor( index, color1 );
      }
    }
  }
  return true;
}

bool PCCPointSet3::transferColorSimple( PCCPointSet3& target, const double bestColorSearchStep ) {
  printf( "transferColorSimple \n" );
  const auto&  source           = *this;
  const size_t pointCountSource = source.getPointCount();
  const size_t pointCountTarget = target.getPointCount();
  if ( ( pointCountSource == 0u ) || ( pointCountTarget == 0u ) || !source.hasColors() ) { return false; }
  target.addColors();

  PCCKdTree kdtreeSource( source );
  PCCKdTree kdtreeTarget( target );

  std::vector<PCCColor3B>              refinedColors1;
  std::vector<std::vector<PCCColor3B>> refinedColors2;
  refinedColors1.resize( pointCountTarget );
  refinedColors2.resize( pointCountTarget );
  const size_t num_results = 1;
  PCCNNResult  result;
  for ( size_t index = 0; index < pointCountTarget; ++index ) {
    kdtreeSource.search( target[index], num_results, result );
    refinedColors1[index] = source.getColor( result.indices( 0 ) );
  }
  for ( size_t index = 0; index < pointCountSource; ++index ) {
    const PCCColor3B color = source.getColor( index );
    kdtreeTarget.search( source[index], num_results, result );
    refinedColors2[result.indices( 0 )].push_back( color );
  }
  for ( size_t index = 0; index < pointCountTarget; ++index ) {
    const PCCColor3B              color1  = refinedColors1[index];
    const std::vector<PCCColor3B> colors2 = refinedColors2[index];
    if ( colors2.empty() ) {
      target.setColor( index, color1 );
    } else {
      double      s        = 1.0 / colors2.size();
      double      r1       = 1.0 / pointCountTarget;
      double      r2       = 1.0 / ( pointCountSource * colors2.size() );
      double      w1       = 0.0;
      double      minError = std::numeric_limits<double>::max();
      PCCVector3D bestColor;
      while ( w1 <= 1.0 ) {
        const double w2 = 1.0 - w1;
        PCCVector3D  color( 0.0 );
        for ( const auto& color2 : colors2 ) {
          for ( size_t k = 0; k < 3; ++k ) { color[k] += color2[k]; }
        }
        for ( size_t k = 0; k < 3; ++k ) {
          color[k] = ( std::min )( round( w2 * s * color[k] + w1 * color1[k] ), 255.0 );
        }

        double e1 = 0.0;
        for ( size_t k = 0; k < 3; ++k ) {
          const double d = color[k] - color1[k];
          e1 += d * d;
        }
        e1 *= r1;

        double e2 = 0.0;
        for ( const auto& color2 : colors2 ) {
          for ( size_t k = 0; k < 3; ++k ) {
            const double d = color[k] - color2[k];
            e2 += d * d;
          }
        }
        e2 *= r2;

        const double e = ( std::max )( e1, e2 );
        if ( e < minError ) {
          bestColor = color;
          minError  = e;
        }
        w1 += bestColorSearchStep;
      }
      target.setColor( index, PCCColor3B( uint8_t( bestColor[0] ), uint8_t( bestColor[1] ), uint8_t( bestColor[2] ) ) );
    }
  }
  return true;
}

bool PCCPointSet3::transferColorWeight( PCCPointSet3& target, const double bestColorSearchStep ) {
  const auto&  source           = *this;
  const size_t pointCountSource = source.getPointCount();
  const size_t pointCountTarget = target.getPointCount();
  if ( ( pointCountSource == 0u ) || ( pointCountTarget == 0u ) || !source.hasColors() ) { return false; }
  target.addColors16bit();
  PCCKdTree    kdtreeSource( source );
  PCCNNResult  result;
  const size_t num_results = 5;
  for ( size_t index = 0; index < pointCountTarget; ++index ) {
    kdtreeSource.search( target[index], num_results, result );
    PCCVector3D color16bit( 0.0 );
    if ( result.size() > 1 && result.dist( 0 ) > 0.0001 ) {
      double sum = 0;
      for ( size_t i = 0; i < result.size(); ++i ) {
        const double w     = 1.0 / pow( result.dist( i ), 2.0 );
        auto         found = source.getColor16bit( result.indices( i ) );
        PCCVector3D  scaled;
        scaled = found;
        color16bit += scaled * w;
        sum += w;
      }
      color16bit /= sum;
    } else {
      const auto& found = source.getColor16bit( result.indices( 0 ) );
      color16bit        = found;
    }
    target.getColor16bit( index ) = color16bit;
  }
  return true;
}

void PCCPointSet3::copyNormals( const PCCPointSet3& sourceWithNormal ) {
  if ( !sourceWithNormal.withNormals_ ) {
    std::cerr << "Normal object don't have normals \n" << std::endl;
    exit( -1 );
  }
  if ( sourceWithNormal.getPointCount() != getPointCount() ) {
    std::cerr << "Normal object and current object must have the same number "
                 "of points \n"
              << std::endl;
    exit( -1 );
  }
  addNormals();

  std::map<double, std::map<double, std::map<double, size_t>>> map;
  for ( size_t i = 0; i < sourceWithNormal.positions_.size(); ++i ) {
    float x      = sourceWithNormal.positions_[i][0];
    float y      = sourceWithNormal.positions_[i][1];
    float z      = sourceWithNormal.positions_[i][2];
    map[x][y][z] = i;
  }
  for ( size_t i = 0; i < positions_.size(); ++i ) {
    float x = positions_[i][0];
    float y = positions_[i][1];
    float z = positions_[i][2];
    if ( map.find( x ) != map.end() && map[x].find( y ) != map[x].end() && map[x][y].find( z ) != map[x][y].end() ) {
      size_t index   = map[x][y][z];
      normals_[i][0] = sourceWithNormal.normals_[index][0];
      normals_[i][1] = sourceWithNormal.normals_[index][1];
      normals_[i][2] = sourceWithNormal.normals_[index][2];
    } else {
      std::cerr << "Error point " << i
                << " of the current points cloud is not "
                   "present in the normal point cloud. "
                   "\n"
                << std::endl;
      exit( -1 );
    }
  }
}

void PCCPointSet3::scaleNormals( const PCCPointSet3& sourceWithNormal ) {
  if ( !sourceWithNormal.withNormals_ ) {
    std::cerr << "Normal object don't have normals \n" << std::endl;
    exit( -1 );
  }
  addNormals();
  std::vector<size_t> count;
  count.resize( getPointCount(), 0 );
  const size_t num_results_max  = 30;
  const size_t num_results_incr = 5;
  PCCKdTree    kdtreeSrc( sourceWithNormal );
  PCCKdTree    kdtreeDst( *this );
  PCCNNResult  result;
  for ( size_t i = 0; i < sourceWithNormal.getPointCount(); ++i ) {
    // For point 'i' in A, find its nearest neighbor in B. store it in 'j'
    size_t num_results = 0;
    do {
      num_results += num_results_incr;
      kdtreeDst.search( sourceWithNormal.positions_[i], num_results, result );
    } while ( result.dist( 0 ) == result.dist( num_results - 1 ) && num_results + num_results_incr <= num_results_max );
    for ( size_t j = 0; j < result.size(); ++j ) {
      if ( result.dist( 0 ) == result.dist( j ) ) {
        size_t index = result.indices( j );
        normals_[index][0] += sourceWithNormal.normals_[i][0];
        normals_[index][1] += sourceWithNormal.normals_[i][1];
        normals_[index][2] += sourceWithNormal.normals_[i][2];
        count[index]++;
      }
    }
  }

  for ( long i = 0; i < getPointCount(); ++i ) {
    if ( count[i] > 0 ) {
      normals_[i][0] /= count[i];
      normals_[i][1] /= count[i];
      normals_[i][2] /= count[i];
    } else {
      size_t num_results = 0;
      do {
        num_results += num_results_incr;
        kdtreeSrc.search( positions_[i], num_results, result );
      } while ( result.dist( 0 ) == result.dist( num_results - 1 ) &&
                num_results + num_results_incr <= num_results_max );
      size_t num = 0;
      for ( size_t j = 0; j < num_results; ++j ) {
        if ( result.dist( 0 ) == result.dist( j ) ) {
          size_t index = result.indices( j );
          normals_[i][0] += sourceWithNormal.normals_[index][0];
          normals_[i][1] += sourceWithNormal.normals_[index][1];
          normals_[i][2] += sourceWithNormal.normals_[index][2];
          num++;
        }
      }
      normals_[i][0] /= num;
      normals_[i][1] /= num;
      normals_[i][2] /= num;
    }
  }
}
