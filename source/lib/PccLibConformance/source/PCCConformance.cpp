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
#include "PCCConformance.h"
#include "PCCConfigurationFileParser.h"
#include "PCCConformanceParameters.h"

using namespace std;
using namespace pcc;

PCCConformance::PCCConformance() : conformanceCount_( 0 ), checkFileCount_(0), checkFileTestsMatch_(true), conformanceTestsMatch_( true ) {}
PCCConformance::~PCCConformance() {}

void PCCConformance::check( const PCCConformanceParameters& params ) {
  PCCErrorMessage errMsg;
  bool            atlasFlag;
  double          aR = 1. / (double)params.fps_;
  KeyValMaps      key_val_decA, key_val_decB;
  checkFileCount_ = 0;
  checkFileTestsMatch_    = true;
  cout << "\n MPEG PCC Conformance v" << TMC2_VERSION_MAJOR << "." << TMC2_VERSION_MINOR << endl;
  std::string fileDecA = params.path_ + "enc_hls_log.txt";
  std::string fileDecB = params.path_ + "dec_hls_log.txt";
  cout << "\n ^^^^^^Checking High Level Syntax Log Files^^^^^^\n\n";

  if ( !checkFiles( fileDecA, fileDecB, hlsKeys, key_val_decA, key_val_decB ) ) {
    std::string tmp = fileDecA + "  " + fileDecB;
    errMsg.warn( "\n ******* Files Do Not Have Equal Lines  ******* \n", tmp );
  }
  fileDecA = params.path_ + "enc_atlas_log.txt";
  fileDecB = params.path_ + "dec_atlas_log.txt";
  cout << "\n ^^^^^^Checking Atlas Log Files^^^^^^\n\n";
  if ( !checkFiles( fileDecA, fileDecB, atlasKeys, key_val_decA, key_val_decB ) ) {
    std::string tmp = fileDecA + "  " + fileDecB;
    errMsg.warn( "\n ******* Files Do Not Have Equal Lines  ******* \n", tmp );
  }
  conformanceCount_      = 0;
  conformanceTestsMatch_ = true;
  checkConformance( params.levelIdc_, aR, key_val_decB, true );
  key_val_decA.clear();
  key_val_decB.clear();
  cout << "\n ^^^^^^Checking Tile Log Files ^^^^^^\n\n";
  fileDecA = params.path_ + "enc_tile_log.txt";
  fileDecB = params.path_ + "dec_tile_log.txt";
  if ( !checkFiles( fileDecA, fileDecB, tileKeys, key_val_decA, key_val_decB ) ) {
    std::string tmp = fileDecA + "  " + fileDecB;
    errMsg.warn( "\n ******* Files Do Not Have Equal Lines ******* \n", tmp );
  }
  key_val_decA.clear();
  key_val_decB.clear();
  cout << "\n ^^^^^^Checking Point Cloud Frame Log Files ^^^^^^\n\n";
  fileDecA = params.path_ + "enc_pcframe_log.txt";
  fileDecB = params.path_ + "dec_pcframe_log.txt";
  if ( !checkFiles( fileDecA, fileDecB, pcframeKeys, key_val_decA, key_val_decB ) ) {
    std::string tmp = fileDecA + "  " + fileDecB;
    errMsg.warn( "\n ******* Files Do Not Have Equal Lines  ******* \n", tmp );
  }
  checkConformance( params.levelIdc_, aR, key_val_decB, false );
  key_val_decA.clear();
  key_val_decB.clear();
  cout << "\n ^^^^^^Checking Picture Log Files ^^^^^^\n\n";
  fileDecA = params.path_ + "enc_picture_log.txt";
  fileDecB = params.path_ + "dec_picture_log.txt";
  if ( !checkFiles( fileDecA, fileDecB, pictureKeys, key_val_decA, key_val_decB ) ) {
    std::string tmp = fileDecA + "  " + fileDecB;
    errMsg.warn( "\n ******* Files Do Not Have Equal Lines  ******* \n", tmp );
  }
  cout << "\n ^^^^^^Matched Check File Tests " << ( checkFileTestsMatch_ ? "MATCH" : "DIFF" ) << " : " << checkFileCount_
       << " tests^^^^^^\n\n";
  cout << "\n ^^^^^^Matched Dynamic Conformance Tests " << ( conformanceTestsMatch_ ? "MATCH" : "DIFF" ) << " : "
       << conformanceCount_ << " tests^^^^^^\n\n";
}

bool PCCConformance::checkFiles( std::string&                    fNameEnc,
                                 std::string&                    fNameDec,
                                 const std::vector<std::string>& keyList,
                                 KeyValMaps&                     key_val_enc,
                                 KeyValMaps&                     key_val_dec ) {
  PCCConfigurationFileParser cfr( keyList );
  cfr.parseFile( fNameEnc, key_val_enc );
  cfr.parseFile( fNameDec, key_val_dec );
  if ( key_val_enc.size() != key_val_dec.size() ) {
    cerr << " Encoder File Has " << key_val_enc.size() << " Lines \n";
    cerr << " Decoder File Has " << key_val_dec.size() << " Lines \n";
    return false;
  }
  size_t index = 0;
  for ( auto& key_val : key_val_enc ) {
    StringStringMap&          dec = key_val_dec[index++];
    StringStringMap::iterator it;
    for ( it = key_val.begin(); it != key_val.end(); ++it ) {
      if ( !dec[it->first].compare( it->second ) ) {
        if ( it->first == "Occupancy" || it->first == "Geometry" || it->first == "Attribute" ) {
          cout << "\n-------" << it->first << "-------\n" << endl;
          continue;
        }
        if ( it->first == "AtlasFrameIndex" ) cout << "\n";
        cout << "(Enc: " << left << setw( 30 ) << it->first << ", " << setw( 32 ) << it->second << " )"
             << " **MATCH** ";
        cout << "(Dec: " << left << setw( 30 ) << it->first << ", " << setw( 32 ) << dec[it->first] << " )" << endl;
      } else {
        cerr << "(Enc: " << left << setw( 30 ) << it->first << ", " << setw( 32 ) << it->second << " )"
             << " **DIFF**  ";
        cerr << "(Dec: " << left << setw( 30 ) << it->first << ", " << setw( 32 ) << dec[it->first] << " )" << endl;
        checkFileTestsMatch_ = false;
      }
      checkFileCount_++; 
    }
  }
  return true;
}

void PCCConformance::checkConformance( uint8_t levelIdc, double aR, KeyValMaps& key_val_map, bool atlasFlag ) {
  std::map<std::string, size_t> maxLevelLimit;
  std::deque<PCCDynamicData>    dataWindow;
  std::vector<std::string>      agrData;
  PCCErrorMessage               error_rep;
  agrData.resize( 3 );

  int64_t clockTick     = -1;
  int64_t frmPerSecMin1 = ( int64_t )( 1 / aR ) - 1;
  uint8_t levelIdx      = ( uint8_t )( 2 * ( levelIdc / 30.0 - 1 ) );
  if ( levelIdx >= 6 ) {
    error_rep.error( " Dynamic Conformance Check Cannot Be Done: level indicator should be in multiples of 30 i.e. 30 - 105 for levels 1 to 3.5 \n" );
    return;
  }

  if ( atlasFlag ) {
    maxLevelLimit.emplace( "VPSMapCount", V3CLevelTable[levelIdx][LevelMapCount] );
    maxLevelLimit.emplace( "AttributeCount", V3CLevelTable[levelIdx][MaxNumAttributeCount] );
    maxLevelLimit.emplace( "AttributeDimension", V3CLevelTable[levelIdx][MaxNumAttributeDims] );
    maxLevelLimit.emplace( "ASPSFrameSize", ASPSLevelTable[levelIdx][MaxAtlasSize] );
    maxLevelLimit.emplace( "NumTilesAtlasFrame", ASPSLevelTable[levelIdx][MaxNumTiles] );
    maxLevelLimit.emplace( "AtlasTotalNumProjPatches", ASPSLevelTable[levelIdx][MaxNumProjPatches] );
    maxLevelLimit.emplace( "AtlasTotalNumRawPatches", ASPSLevelTable[levelIdx][MaxNumRawPatches] );
    maxLevelLimit.emplace( "AtlasTotalNumEomPatches", ASPSLevelTable[levelIdx][MaxNumEomPatches] );
    agrData[0] = "AtlasTotalNumProjPatches";
    agrData[1] = "AtlasTotalNumRawPatches";
    agrData[2] = "AtlasTotalNumEomPatches";
  } else {
    maxLevelLimit.emplace( "NumProjPoints", V3CLevelTable[levelIdx][MaxNumProjPoints] );
    maxLevelLimit.emplace( "NumEomPoints", V3CLevelTable[levelIdx][MaxNumEomPoints] );
    maxLevelLimit.emplace( "NumRawPoints", V3CLevelTable[levelIdx][MaxNumRawPoints] );
    agrData[0] = "NumProjPoints";
    agrData[1] = "NumRawPoints";
    agrData[2] = "NumEomPoints";
  }

  // check general tier level limits A.6.1 & A.6.2
  size_t         frmIdx, value, maxValue;
  PCCDynamicData totalPerSec{};
  for ( auto& key_val_pair : key_val_map ) {
    PCCDynamicData tmp;
    bool           skipLineFlag = false;
    for ( auto& kvp : key_val_pair ) {
      if ( kvp.first == "AtlasFrameIndex" ) {
        convertString( kvp.second, frmIdx );
        skipLineFlag = true;
        clockTick++;
      }
      if ( !maxLevelLimit.count( kvp.first ) ) continue;
      if ( !checkLimit( kvp.second, maxLevelLimit[kvp.first], value ) ) {
        cerr << "\n" << kvp.first << " Value : " << value << " Exceeds Max. Limit " << maxLevelLimit[kvp.first] << endl;
        conformanceTestsMatch_ = false;
      }
      if ( kvp.first == agrData[0] || kvp.first == agrData[1] || kvp.first == agrData[2] ) {
        for ( int n = 0; n < 3; n++ ) {
          if ( kvp.first == agrData[n] ) {
            convertString( kvp.second, tmp.data_[n] );
            break;
          }
        }
      }
    }
    if ( !skipLineFlag ) {
      totalPerSec += tmp;
      dataWindow.push_back( tmp );
      if ( clockTick >= frmPerSecMin1 ) {
        // cout << " \nAggregate Data @ clockTick = " << clockTick << "\n";
        for ( int n = 0; n < 3; n++ ) {
          // cout << totalPerSec.data_[n] << ", " << endl;
          maxValue = maxLevelLimit[agrData[n]];
          if ( totalPerSec.data_[n] > maxValue ) {
            printf( " %s MaxPerSec %zu Exceeds Table A-6 Specified Limit %zu \n", agrData[n].c_str(), totalPerSec.data_[n],
                    maxValue );
            conformanceTestsMatch_ = false;
          }
        }
        totalPerSec -= dataWindow.front();
        dataWindow.pop_front();
      }
    }
  }
  conformanceCount_++;
}

template <typename T>
inline bool PCCConformance::checkLimit( const std::string& keyValue, T& maxVal, T& val ) {
  bool               error = true;
  std::istringstream keyVal_ss( keyValue, std::istringstream::in );
  keyVal_ss.exceptions( std::ios::failbit );
  keyVal_ss >> val;
  if ( val > maxVal ) { error = false; }
  return error;
}

template <typename T>
inline void PCCConformance::convertString( const std::string& keyValue, T& val ) {
  std::istringstream keyVal_ss( keyValue, std::istringstream::in );
  keyVal_ss.exceptions( std::ios::failbit );
  keyVal_ss >> val;
  return;
}
