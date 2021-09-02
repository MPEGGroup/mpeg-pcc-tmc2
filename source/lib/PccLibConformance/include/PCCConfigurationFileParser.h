/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
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
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
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

#ifndef PCCConfigurationFleParser_h
#define PCCConfigurationFleParser_h

#include "PCCCommon.h"

namespace pcc {

const std::vector<std::string> bitStrMD5Keys = {"BITSTRMD5"};

const std::vector<std::string> hlsKeys = {"AtlasFrameIndex", "HLSMD5",   "SEI11MD5", "SEI19MD5",
                                          "SEI65MD5",        "SEI66MD5", "SEI67MD5"};

const std::vector<std::string> atlasKeys = {"AtlasFrameIndex",
                                            "AtlasFrameOrderCntVal",
                                            "AtlasFrameWidthMax",
                                            "AtlasFrameHeightMax",
                                            "AtlasID",
                                            "ASPSFrameSize",
                                            "VPSMapCount",
                                            "AttributeCount",
                                            "AttributeDimension",
                                            "NumTilesAtlasFrame",
                                            "AtlasTotalNumProjPatches",
                                            "AtlasTotalNumRawPatches",
                                            "AtlasTotalNumEomPatches",
                                            "AtlasMD5",
                                            "AtlasB2PMD5"};

const std::vector<std::string> tileKeys = {"AtlasFrameIndex", "TileID",      "AtlasFrameOrderCntVal",
                                           "TileType",        "TileOffsetX", "TileOffsetY",
                                           "TileWidth",       "TileHeight",  "TileMD5",
                                           "TileB2PMD5"};

const std::vector<std::string> pcframeKeys = {
    "AtlasFrameIndex", "PointCloudFrameOrderCntVal", "NumProjPoints", "NumRawPoints", "NumEomPoints", "MD5checksum"};

const std::vector<std::string> recPcframeKeys = {"AtlasFrameIndex", "PointCloudFrameOrderCntVal", "MD5checksum"};

const std::vector<std::string> pictureKeys = {"Occupancy",
                                              "Geometry",
                                              "Attribute",
                                              "MapIdx",
                                              "AuxiliaryVideoFlag",
                                              "PicOrderCntVal",
                                              "IdxOutOrderCntVal",
                                              "AttrIdx",
                                              "AttrPartIdx",
                                              "AttrTypeID",
                                              "Width",
                                              "Height",
                                              "MD5checksumChan0",
                                              "MD5checksumChan1",
                                              "MD5checksumChan2"};

const size_t V3CLevelTable[6][9] = {  // Table A-5
    {30000000, 1500000, 1500000, 1000000, 50000, 50000, 2, 1, 3},
    {60000000, 3000000, 3000000, 2000000, 100000, 100000, 2, 3, 3},
    {120000000, 6000000, 6000000, 4000000, 200000, 200000, 4, 4, 3},
    {240000000, 12000000, 12000000, 8000000, 400000, 400000, 4, 8, 4},
    {480000000, 24000000, 24000000, 16000000, 800000, 800000, 8, 16, 5},
    {960000000, 48000000, 48000000, 32000000, 1600000, 1600000, 8, 24, 6}};

const size_t ASPSLevelTable[6][10] = {  // Table A-6
    {2048, 32, 32, 15000, 15000, 50, 2228224, 65536, 1024, 1024},
    {4096, 64, 32, 30000, 30000, 50, 2228224, 131072, 2048, 1024},
    {16384, 128, 64, 120000, 120000, 200, 8912896, 524288, 4096, 2048},
    {32384, 128, 64, 240000, 240000, 200, 8912896, 1036288, 4096, 2048},
    {65536, 512, 128, 480000, 480000, 500, 35651584, 2097152, 16384, 4096},
    {65536, 512, 512, 480000, 480000, 500, 35651584, 4194304, 32768, 32768}};

enum V3CLimitType {  // Table A-5
  MaxNumProjPointsPerSec = 0,
  MaxNumEomPointsPerSec,
  MaxNumRawPointsPerSec,
  MaxNumProjPoints,
  MaxNumEomPoints,
  MaxNumRawPoints,
  LevelMapCount,
  MaxNumAttributeCount,
  MaxNumAttributeDims
};

enum ASPSLimitType {  // Table A-6
  MaxNumProjPatches = 0,
  MaxNumRawPatches,
  MaxNumEomPatches,
  MaxCABSize,
  MaxAtlasBR,
  MaxNumTiles,
  MaxAtlasSize,
  MaxProjPatchesPerSec,
  MaxRawPatchesPerSec,
  MaxEomPatchesPerSec
};

struct PCCErrorMessage {
  PCCErrorMessage( bool flg = false ) : is_errored_( flg ){};
  PCCErrorMessage( const PCCErrorMessage& rep ) : is_errored_( rep.is_errored_ ){};
  virtual ~PCCErrorMessage() {}
  virtual std::ostream& error( const std::string& errMessage, const std::string& where = "" );
  virtual std::ostream& warn( const std::string& errMessage, const std::string& where = "" );
  bool                  is_errored_;
};

struct PCCDynamicData {
  PCCDynamicData() { data_[0] = data_[1] = data_[2] = 0; }
  PCCDynamicData( size_t proj, size_t raw, size_t eom ) {
    data_[0] = proj, data_[1] = raw;
    data_[2] = eom;
  }

  PCCDynamicData& operator+=( const PCCDynamicData& rhs ) {
    data_[0] += rhs.data_[0];
    data_[1] += rhs.data_[1];
    data_[2] += rhs.data_[2];
    return *this;
  }

  PCCDynamicData& operator-=( const PCCDynamicData& rhs ) {
    data_[0] -= rhs.data_[0];
    data_[1] -= rhs.data_[1];
    data_[2] -= rhs.data_[2];
    return *this;
  }
  size_t data_[3];  // data[0] -> numProj, data[1] -> numRaw, data[2] -> numEom  ;
};

typedef std::vector<std::map<std::string, std::string>> KeyValMaps;
typedef std::map<std::string, std::string>              StringStringMap;

class PCCConfigurationFileParser : PCCErrorMessage {
 public:
  PCCConfigurationFileParser( const std::vector<std::string>& keys ) :
      PCCErrorMessage(),
      name_( "" ),
      linenum_( 0 ),
      keyList_( keys ){};

  const std::string where() {
    std::ostringstream os;
    os << "File Name: " << name_;
    if ( linenum_ >= 0 ) os << "\nLine Number: " << linenum_;
    return os.str();
  }

  bool parseFile( std::string& fileName, KeyValMaps& key_val_maps );
  void scanLine( std::string& line, KeyValMaps& key_val_maps );
  void scanStream( std::istream& in, KeyValMaps& key_Val_maps );
  bool validKey( std::string& key );

 private:
  std::string name_;
  int         linenum_;

  const std::vector<std::string>& keyList_;
};

}  // namespace pcc

#endif  //~PCCConfigurationFleParser_h
