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

using namespace pcc;

PCCPoint3D PCCPointSet3::computeCentroid() const {
  PCCPoint3D bary(0.0);
  const size_t pointCount = getPointCount();
  if (pointCount) {
    for (size_t i = 0; i < pointCount; ++i) {
      const PCCPoint3D &pt = (*this)[i];
      bary += pt;
    }
    bary /= double(pointCount);
  }
  return bary;
}

PCCBox3D PCCPointSet3::computeBoundingBox() const {
  PCCBox3D bbox = { (std::numeric_limits<double>::max)(),
                    (std::numeric_limits<double>::lowest)()};
  const size_t pointCount = getPointCount();
  for (size_t i = 0; i < pointCount; ++i) {
    const PCCPoint3D &pt = (*this)[i];
    for (int k = 0; k < 3; ++k) {
      if (pt[k] > bbox.max_[k]) {
        bbox.max_[k] = pt[k];
      }
      if (pt[k] < bbox.min_[k]) {
        bbox.min_[k] = pt[k];
      }
    }
  }
  return bbox;
}

bool PCCPointSet3::write( const std::string &fileName, const bool asAscii ) {
  std::ofstream fout(fileName, std::ofstream::out);
  if (!fout.is_open()) {
    return false;
  }
  const size_t pointCount = getPointCount();
  fout << "ply" << std::endl;

  if (asAscii) {
    fout << "format ascii 1.0" << std::endl;
  } else {
    PCCEndianness endianess = PCCSystemEndianness();
    if (endianess == PCC_BIG_ENDIAN) {
      fout << "format binary_big_endian 1.0" << std::endl;
    } else {
      fout << "format binary_little_endian 1.0" << std::endl;
    }
  }
  fout << "element vertex " << pointCount << std::endl;
  fout << "property float64 x" << std::endl;
  fout << "property float64 y" << std::endl;
  fout << "property float64 z" << std::endl;
  if (hasColors()) {
    fout << "property uchar red" << std::endl;
    fout << "property uchar green" << std::endl;
    fout << "property uchar blue" << std::endl;
  }
  if (hasReflectances()) {
    fout << "property uint16 refc" << std::endl;
  }
  if( PCC_SAVE_POINT_TYPE ) {
    fout << "property uchar type" << std::endl;
    switch( PCC_SAVE_POINT_TYPE ){
      case 1 : fout << "comment POINT_TYPE: Unset D0 D1 Smooth " << std::endl; break;
      case 2 : fout << "comment POINT_TYPE: type0 type1 type2  " << std::endl; break;
      default:  break;
    }
  }
  fout << "element face 0" << std::endl;
  fout << "property list uint8 int32 vertex_index" << std::endl;
  fout << "end_header" << std::endl;
  if (asAscii) {
    fout << std::setprecision(std::numeric_limits<double>::max_digits10);
    for (size_t i = 0; i < pointCount; ++i) {
      const PCCPoint3D &position = (*this)[i];
      fout << position.x() << " " << position.y() << " " << position.z();
      if (hasColors()) {
        const PCCColor3B &color = getColor(i);
        fout << " " << static_cast<int>(color[0]) << " " << static_cast<int>(color[1]) << " "
            << static_cast<int>(color[2]);
      }
      if (hasReflectances()) {
        fout << " " << static_cast<int>(getReflectance(i));
      }
      if( PCC_SAVE_POINT_TYPE ) {
        fout << " " << types_[ i ];
      }
      fout << std::endl;
    }
  } else {
    fout.clear();
    fout.close();
    fout.open(fileName, std::ofstream::binary | std::ofstream::app);
    for (size_t i = 0; i < pointCount; ++i) {
      const PCCPoint3D &position = (*this)[i];
      fout.write(reinterpret_cast<const char *const>(&position), sizeof(double) * 3);
      if (hasColors()) {
        const PCCColor3B &color = getColor(i);
        fout.write(reinterpret_cast<const char *>(&color), sizeof(uint8_t) * 3);
      }
      if (hasReflectances()) {
        const uint16_t &reflectance = getReflectance(i);
        fout.write(reinterpret_cast<const char *>(&reflectance), sizeof(uint16_t));
      }
      if( PCC_SAVE_POINT_TYPE ) {
        fout.write( reinterpret_cast<const char *const>( &types_[ i ] ), sizeof( uint8_t ) );
      }
    }
  }
  fout.close();
  return true;
}
bool PCCPointSet3::read(const std::string &fileName) {
  std::ifstream ifs(fileName, std::ifstream::in);
  if (!ifs.is_open()) {
    return false;
  }
  enum AttributeType {
    ATTRIBUTE_TYPE_FLOAT64 = 0,
    ATTRIBUTE_TYPE_FLOAT32 = 1,
    ATTRIBUTE_TYPE_UINT64 = 2,
    ATTRIBUTE_TYPE_UINT32 = 3,
    ATTRIBUTE_TYPE_UINT16 = 4,
    ATTRIBUTE_TYPE_UINT8 = 5,
    ATTRIBUTE_TYPE_INT64 = 6,
    ATTRIBUTE_TYPE_INT32 = 7,
    ATTRIBUTE_TYPE_INT16 = 8,
    ATTRIBUTE_TYPE_INT8 = 9,
  };
  struct AttributeInfo {
    std::string name;
    AttributeType type;
    size_t byteCount;
  };

  std::vector<AttributeInfo> attributesInfo;
  attributesInfo.reserve(16);
  const size_t MAX_BUFFER_SIZE = 4096;
  char tmp[MAX_BUFFER_SIZE];
  const char *sep = " \t\r";
  std::vector<std::string> tokens;

  ifs.getline(tmp, MAX_BUFFER_SIZE);
  getTokens(tmp, sep, tokens);
  if (tokens.empty() || tokens[0] != "ply") {
    std::cout << "Error: corrupted file!" << std::endl;
    return false;
  }
  bool isAscii = false;
  double version = 1.0;
  size_t pointCount = 0;
  bool isVertexProperty = true;
  while (1) {
    if (ifs.eof()) {
      std::cout << "Error: corrupted header!" << std::endl;
      return false;
    }
    ifs.getline(tmp, MAX_BUFFER_SIZE);
    getTokens(tmp, sep, tokens);
    if (tokens.empty() || tokens[0] == "comment") {
      continue;
    }
    if (tokens[0] == "format") {
      if (tokens.size() != 3) {
        std::cout << "Error: corrupted format info!" << std::endl;
        return false;
      }
      isAscii = tokens[1] == "ascii";
      version = atof(tokens[2].c_str());
    } else if (tokens[0] == "element") {
      if (tokens.size() != 3) {
        std::cout << "Error: corrupted element info!" << std::endl;
        return false;
      }
      if (tokens[1] == "vertex") {
        pointCount = atoi(tokens[2].c_str());
      } else {
        isVertexProperty = false;
      }
    } else if (tokens[0] == "property" && isVertexProperty) {
      if (tokens.size() != 3) {
        std::cout << "Error: corrupted property info!" << std::endl;
        return false;
      }
      const std::string &propertyType = tokens[1];
      const std::string &propertyName = tokens[2];
      const size_t attributeIndex = attributesInfo.size();
      attributesInfo.resize(attributeIndex + 1);
      AttributeInfo &attributeInfo = attributesInfo[attributeIndex];
      attributeInfo.name = propertyName;
      if (propertyType == "float64") {
        attributeInfo.type = ATTRIBUTE_TYPE_FLOAT64;
        attributeInfo.byteCount = 8;
      } else if (propertyType == "float" || propertyType == "float32") {
        attributeInfo.type = ATTRIBUTE_TYPE_FLOAT32;
        attributeInfo.byteCount = 4;
      } else if (propertyType == "uint64") {
        attributeInfo.type = ATTRIBUTE_TYPE_UINT64;
        attributeInfo.byteCount = 8;
      } else if (propertyType == "uint32") {
        attributeInfo.type = ATTRIBUTE_TYPE_UINT32;
        attributeInfo.byteCount = 4;
      } else if (propertyType == "uint16") {
        attributeInfo.type = ATTRIBUTE_TYPE_UINT16;
        attributeInfo.byteCount = 2;
      } else if (propertyType == "uchar" || propertyType == "uint8") {
        attributeInfo.type = ATTRIBUTE_TYPE_UINT8;
        attributeInfo.byteCount = 1;
      } else if (propertyType == "int64") {
        attributeInfo.type = ATTRIBUTE_TYPE_INT64;
        attributeInfo.byteCount = 8;
      } else if (propertyType == "int32") {
        attributeInfo.type = ATTRIBUTE_TYPE_INT32;
        attributeInfo.byteCount = 4;
      } else if (propertyType == "int16") {
        attributeInfo.type = ATTRIBUTE_TYPE_INT16;
        attributeInfo.byteCount = 2;
      } else if (propertyType == "char" || propertyType == "int8") {
        attributeInfo.type = ATTRIBUTE_TYPE_INT8;
        attributeInfo.byteCount = 1;
      }
    } else if (tokens[0] == "end_header") {
      break;
    }
  }
  if (version != 1.0) {
    std::cout << "Error: non-supported version!" << std::endl;
    return false;
  }

  size_t indexX = PCC_UNDEFINED_INDEX;
  size_t indexY = PCC_UNDEFINED_INDEX;
  size_t indexZ = PCC_UNDEFINED_INDEX;
  size_t indexR = PCC_UNDEFINED_INDEX;
  size_t indexG = PCC_UNDEFINED_INDEX;
  size_t indexB = PCC_UNDEFINED_INDEX;
  size_t indexReflectance = PCC_UNDEFINED_INDEX;
  const size_t attributeCount = attributesInfo.size();
  for (size_t a = 0; a < attributeCount; ++a) {
    const auto &attributeInfo = attributesInfo[a];
    if (attributeInfo.name == "x" &&
        (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4)) {
      indexX = a;
    } else if (attributeInfo.name == "y" &&
        (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4)) {
      indexY = a;
    } else if (attributeInfo.name == "z" &&
        (attributeInfo.byteCount == 8 || attributeInfo.byteCount == 4)) {
      indexZ = a;
    } else if (attributeInfo.name == "red" && attributeInfo.byteCount == 1) {
      indexR = a;
    } else if (attributeInfo.name == "green" && attributeInfo.byteCount == 1) {
      indexG = a;
    } else if (attributeInfo.name == "blue" && attributeInfo.byteCount == 1) {
      indexB = a;
    } else if ((attributeInfo.name == "reflectance" || attributeInfo.name == "refc") &&
        attributeInfo.byteCount <= 2) {
      indexReflectance = a;
    }
  }
  if (indexX == PCC_UNDEFINED_INDEX || indexY == PCC_UNDEFINED_INDEX ||
      indexZ == PCC_UNDEFINED_INDEX) {
    std::cout << "Error: missing coordinates!" << std::endl;
    return false;
  }
  withColors_ = indexR != PCC_UNDEFINED_INDEX && indexG != PCC_UNDEFINED_INDEX &&
      indexB != PCC_UNDEFINED_INDEX;
  withReflectances_ = indexReflectance != PCC_UNDEFINED_INDEX;
  resize(pointCount);
  if (isAscii) {
    size_t pointCounter = 0;
    while (!ifs.eof() && pointCounter < pointCount) {
      ifs.getline(tmp, MAX_BUFFER_SIZE);
      getTokens(tmp, sep, tokens);
      if (tokens.empty()) {
        continue;
      }
      if (tokens.size() < attributeCount) {
        return false;
      }
      auto &position = positions_[pointCounter];
      position[0] = atof(tokens[indexX].c_str());
      position[1] = atof(tokens[indexY].c_str());
      position[2] = atof(tokens[indexZ].c_str());
      if (hasColors()) {
        auto &color = colors_[pointCounter];
        color[0] = atoi(tokens[indexR].c_str());
        color[1] = atoi(tokens[indexG].c_str());
        color[2] = atoi(tokens[indexB].c_str());
      }
      if (hasReflectances()) {
        reflectances_[pointCounter] = uint16_t(atoi(tokens[indexReflectance].c_str()));
      }
      ++pointCounter;
    }
  } else {
    for (size_t pointCounter = 0; pointCounter < pointCount && !ifs.eof(); ++pointCounter) {
      auto &position = positions_[pointCounter];
      for (size_t a = 0; a < attributeCount && !ifs.eof(); ++a) {
        const auto &attributeInfo = attributesInfo[a];
        if (a == indexX) {
          if (attributeInfo.byteCount == 4) {
            float x;
            ifs.read(reinterpret_cast<char *>(&x), sizeof(float));
            position[0] = x;
          } else {
            double x;
            ifs.read(reinterpret_cast<char *>(&x), sizeof(double));
            position[0] = x;
          }
        } else if (a == indexY) {
          if (attributeInfo.byteCount == 4) {
            float y;
            ifs.read(reinterpret_cast<char *>(&y), sizeof(float));
            position[1] = y;
          } else {
            double y;
            ifs.read(reinterpret_cast<char *>(&y), sizeof(double));
            position[1] = y;
          }
        } else if (a == indexZ) {
          if (attributeInfo.byteCount == 4) {
            float z;
            ifs.read(reinterpret_cast<char *>(&z), sizeof(float));
            position[2] = z;
          } else {
            double z;
            ifs.read(reinterpret_cast<char *>(&z), sizeof(double));
            position[2] = z;
          }
        } else if (a == indexR && attributeInfo.byteCount == 1) {
          auto &color = colors_[pointCounter];
          ifs.read(reinterpret_cast<char *>(&color[0]), sizeof(uint8_t));
        } else if (a == indexG && attributeInfo.byteCount == 1) {
          auto &color = colors_[pointCounter];
          ifs.read(reinterpret_cast<char *>(&color[1]), sizeof(uint8_t));
        } else if (a == indexB && attributeInfo.byteCount == 1) {
          auto &color = colors_[pointCounter];
          ifs.read(reinterpret_cast<char *>(&color[2]), sizeof(uint8_t));
        } else if (a == indexReflectance && attributeInfo.byteCount <= 2) {
          if (indexReflectance == 1) {
            uint8_t reflectance;
            ifs.read(reinterpret_cast<char *>(&reflectance), sizeof(uint8_t));
            reflectances_[pointCounter] = reflectance;
          } else {
            auto &reflectance = reflectances_[pointCounter];
            ifs.read(reinterpret_cast<char *>(reflectance), sizeof(uint16_t));
          }
        } else {
          char buffer[128];
          ifs.read(buffer, attributeInfo.byteCount);
        }
      }
    }
  }
  return true;
}

void PCCPointSet3::convertRGBToYUV() {  // BT709
  for (auto &color : colors_) {
    const uint8_t r = color[0];
    const uint8_t g = color[1];
    const uint8_t b = color[2];
    const double y = std::round(0.212600 * r + 0.715200 * g + 0.072200 * b);
    const double u = std::round(-0.114572 * r - 0.385428 * g + 0.500000 * b + 128.0);
    const double v = std::round(0.500000 * r - 0.454153 * g - 0.045847 * b + 128.0);
    assert(y >= 0.0 && y <= 255.0 && u >= 0.0 && u <= 255.0 && v >= 0.0 && v <= 255.0);
    color[0] = static_cast<uint8_t>(y);
    color[1] = static_cast<uint8_t>(u);
    color[2] = static_cast<uint8_t>(v);
  }
}

void PCCPointSet3::convertRGBToYUVClosedLoop() {  // BT709
  for (auto &color : colors_) {
    const uint8_t r = color[0];
    const uint8_t g = color[1];
    const uint8_t b = color[2];
    const double y = std::round(0.212600 * r + 0.715200 * g + 0.072200 * b);
    const double u = std::round((b - y) / 1.8556 + 128.0);
    const double v = std::round((r - y) / 1.5748 + 128.0);
    assert(y >= 0.0 && y <= 255.0 && u >= 0.0 && u <= 255.0 && v >= 0.0 && v <= 255.0);
    color[0] = static_cast<uint8_t>(y);
    color[1] = static_cast<uint8_t>(u);
    color[2] = static_cast<uint8_t>(v);
  }
}
void PCCPointSet3::convertYUVToRGB() {  // BT709
  for (auto &color : colors_) {
    const double y1 = color[0];
    const double u1 = color[1] - 128.0;
    const double v1 = color[2] - 128.0;
    const double r = PCCClip(round(y1 /*- 0.00000 * u1*/ + 1.57480 * v1), 0.0, 255.0);
    const double g = PCCClip(round(y1 - 0.18733 * u1 - 0.46813 * v1), 0.0, 255.0);
    const double b = PCCClip(round(y1 + 1.85563 * u1 /*+ 0.00000 * v1*/), 0.0, 255.0);
    color[0] = static_cast<uint8_t>(r);
    color[1] = static_cast<uint8_t>(g);
    color[2] = static_cast<uint8_t>(b);
  }
}

bool PCCPointSet3::transfertColors( PCCPointSet3 &target, const int32_t searchRange, const bool losslessTexture ) const {
  const auto& source = *this;
  const size_t pointCountSource = source.getPointCount();
  const size_t pointCountTarget = target.getPointCount();
  if (!pointCountSource || !pointCountTarget || !source.hasColors()) {
    return false;
  }
  KDTreeVectorOfVectorsAdaptor<PCCPointSet3, double> kdtreeTarget(3, target, 10);
  KDTreeVectorOfVectorsAdaptor<PCCPointSet3, double> kdtreeSource(3, source, 10);

  target.addColors();
  std::vector<PCCColor3B> refinedColors1;
  std::vector<std::vector<PCCColor3B>> refinedColors2;
  refinedColors1.resize(pointCountTarget);
  refinedColors2.resize(pointCountTarget);
  const size_t num_results = 1;
  std::vector<size_t> indices(num_results);
  std::vector<double> sqrDist(num_results);
  nanoflann::KNNResultSet<double> resultSet(num_results);
  for (size_t index = 0; index < pointCountTarget; ++index) {
    resultSet.init(&indices[0], &sqrDist[0]);
    kdtreeSource.index->findNeighbors(resultSet, &target[index][0], nanoflann::SearchParams(10));
    refinedColors1[index] = source.getColor(indices[0]);
  }
  for (size_t index = 0; index < pointCountSource; ++index) {
    const PCCColor3B color = source.getColor(index);
    resultSet.init(&indices[0], &sqrDist[0]);
    kdtreeTarget.index->findNeighbors(resultSet, &source[index][0], nanoflann::SearchParams(10));
    refinedColors2[indices[0]].push_back(color);
  }
  for (size_t index = 0; index < pointCountTarget; ++index) {
    const PCCColor3B color1 = refinedColors1[index];
    const std::vector<PCCColor3B> &colors2 = refinedColors2[index];
    if (colors2.empty() || losslessTexture) {
      target.setColor(index, color1);
    } else {
      const double H = double(colors2.size());
      const PCCVector3D centroid1(color1[0], color1[1], color1[2]);
      PCCVector3D centroid2(0.0);
      for (const auto color2 : colors2) {
        for (size_t k = 0; k < 3; ++k) {
          centroid2[k] += color2[k];
        }
      }
      centroid2 /= H;

      double D2 = 0.0;
      for (const auto color2 : colors2) {
        for (size_t k = 0; k < 3; ++k) {
          const double d2 = centroid2[k] - color2[k];
          D2 += d2 * d2;
        }
      }
      const double r = double(pointCountTarget) / double(pointCountSource);
      const double delta2 = (centroid2 - centroid1).getNorm2();
      const double eps = 0.000001;

      const bool fixWeight = 1; // m42538
      if (fixWeight || delta2 > eps) {  // centroid2 != centroid1
        double w = 0.0;

      if (!fixWeight){        
        const double alpha = D2 / delta2;
        const double a = H * r - 1.0;
        const double c = alpha * r - 1.0;
        if (fabs(a) < eps) {
          w = -0.5 * c;
        } else {
          const double delta = 1.0 - a * c;
          if (delta >= 0.0) {
            w = (-1.0 + sqrt(delta)) / a;
          }
        }
      }
      const double oneMinusW = 1.0 - w;
      PCCVector3D color0;
      for (size_t k = 0; k < 3; ++k) {
        color0[k] = PCCClip(round(w * centroid1[k] + oneMinusW * centroid2[k]), 0.0, 255.0);
      }
      const double rSource = 1.0 / double(pointCountSource);
      const double rTarget = 1.0 / double(pointCountTarget);
      const double maxValue = std::numeric_limits<uint8_t>::max();
      double minError = std::numeric_limits<double>::max();
      PCCVector3D bestColor(color0);
      PCCVector3D color;
      for (int32_t s1 = -searchRange; s1 <= searchRange; ++s1) {
        color[0] = PCCClip(color0[0] + s1, 0.0, maxValue);
        for (int32_t s2 = -searchRange; s2 <= searchRange; ++s2) {
          color[1] = PCCClip(color0[1] + s2, 0.0, maxValue);
          for (int32_t s3 = -searchRange; s3 <= searchRange; ++s3) {
            color[2] = PCCClip(color0[2] + s3, 0.0, maxValue);

            double e1 = 0.0;
            for (size_t k = 0; k < 3; ++k) {
              const double d = color[k] - color1[k];
              e1 += d * d;
            }
            e1 *= rTarget;

            double e2 = 0.0;
            for (const auto color2 : colors2) {
              for (size_t k = 0; k < 3; ++k) {
                const double d = color[k] - color2[k];
                e2 += d * d;
              }
            }
            e2 *= rSource;

            const double error = std::max(e1, e2);
            if (error < minError) {
              minError = error;
              bestColor = color;
            }
          }
        }
      }
      target.setColor(
          index, PCCColor3B(uint8_t(bestColor[0]), uint8_t(bestColor[1]), uint8_t(bestColor[2])));
      } else {  // centroid2 == centroid1
        target.setColor(index, color1);
      }
    }
  }
  return true;
}
