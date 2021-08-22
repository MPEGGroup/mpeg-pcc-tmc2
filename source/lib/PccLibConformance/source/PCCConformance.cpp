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

PCCConformance::PCCConformance() :
    levelLimitsCount_( 0 ),
    levelLimitsExceedCount_( 0 ),
    logFilesCount_( 0 ),
    logFilesMatchCount_( 0 ),
    logFileTestsMatch_( true ),
    levelLimitTestsMatch_( true ) {}
PCCConformance::~PCCConformance() {}

void PCCConformance::check( const PCCConformanceParameters& params ) {
  PCCErrorMessage errMsg;
  double          aR = 1. / (double)params.fps_;
  KeyValMaps      key_val_decA, key_val_decB;
  logFilesCount_      = 0;
  logFilesMatchCount_ = 0;
  levelLimitsCount_   = 0;
  cout << "\n MPEG PCC Conformance v" << TMC2_VERSION_MAJOR << "." << TMC2_VERSION_MINOR << endl;

  std::string fileDecA = params.path_ + "enc_bitstream_md5.txt";
  std::string fileDecB = params.path_ + "dec_bitstream_md5.txt";
  cout << "\n ^^^^^^Checking Bitstream MD5^^^^^^ \n";
  logFileTestsMatch_ = true;
  if ( !compareLogFiles( fileDecA, fileDecB, bitStrMD5Keys, key_val_decA, key_val_decB ) ) {
    std::string tmp = fileDecA + " with " + fileDecB;
    errMsg.warn( "\n ******* Please Check ******* \n", tmp );
  }
  cout << "^^^^^^ BitStream MD5 : " << ( logFileTestsMatch_ ? "MATCH" : "DIFF" ) << std::endl;

  fileDecA = params.path_ + "enc_hls_md5.txt";
  fileDecB = params.path_ + "dec_hls_md5.txt";
  key_val_decA.clear();
  key_val_decB.clear();

  cout << "\n ^^^^^^Checking High Level Syntax MD5^^^^^^ \n";
  logFileTestsMatch_ = true;
  if ( !compareLogFiles( fileDecA, fileDecB, hlsKeys, key_val_decA, key_val_decB ) ) {
    std::string tmp = fileDecA + " with " + fileDecB;
    errMsg.warn( "\n ******* Please Check ******* \n", tmp );
  }
  cout << "^^^^^^ High Level Syntax MD5 : " << ( logFileTestsMatch_ ? "MATCH" : "DIFF" ) << std::endl;

  fileDecA = params.path_ + "enc_atlas_log.txt";
  fileDecB = params.path_ + "dec_atlas_log.txt";
  key_val_decA.clear();
  key_val_decB.clear();
  cout << "\n ^^^^^^ Atlas Log Files Check ^^^^^^ \n";
  logFileTestsMatch_ = true;
  if ( !compareLogFiles( fileDecA, fileDecB, atlasKeys, key_val_decA, key_val_decB ) ) {
    std::string tmp = fileDecA + " with " + fileDecB;
    errMsg.warn( "\n ******* Please Check ******* \n", tmp );
  }
  cout << "^^^^^^ Atlas Log Files " << ( logFileTestsMatch_ ? "MATCH" : "DIFF" ) << std::endl;

  cout << "\n ^^^^^^ Atlas Level Limits Test  ^^^^^^ \n";
  levelLimitTestsMatch_ = true;
  checkLevelLimits( params.levelIdc_, aR, key_val_decB, true );
  cout << "^^^^^^ Atlas Level Limits: " << ( levelLimitTestsMatch_ ? "MATCH" : "DIFF" ) << std::endl;

  key_val_decA.clear();
  key_val_decB.clear();
  cout << "\n ^^^^^^ Tile Log Files Check ^^^^^^\n";
  logFileTestsMatch_ = true;
  fileDecA           = params.path_ + "enc_tile_log.txt";
  fileDecB           = params.path_ + "dec_tile_log.txt";
  if ( !compareLogFiles( fileDecA, fileDecB, tileKeys, key_val_decA, key_val_decB ) ) {
    std::string tmp = fileDecA + " with " + fileDecB;
    errMsg.warn( "\n ******* Please Check ******* \n", tmp );
  }
  cout << "^^^^^^ Tile Logs: " << ( logFileTestsMatch_ ? "MATCH" : "DIFF" ) << std::endl;

  key_val_decA.clear();
  key_val_decB.clear();
  cout << "\n ^^^^^^ Point Cloud Frame Log Files Check ^^^^^^ \n";
  logFileTestsMatch_ = true;
  fileDecA           = params.path_ + "enc_pcframe_log.txt";
  fileDecB           = params.path_ + "dec_pcframe_log.txt";
  logFileTestsMatch_ = true;
  if ( !compareLogFiles( fileDecA, fileDecB, pcframeKeys, key_val_decA, key_val_decB ) ) {
    std::string tmp = fileDecA + " with " + fileDecB;
    errMsg.warn( "\n ******* Please Check  ******* \n", tmp );
  }
  cout << "^^^^^^ Point Cloud Frame Log Files : " << ( logFileTestsMatch_ ? "MATCH" : "DIFF" ) << std::endl;

  cout << "\n ^^^^^^ Point Cloud Frame Level Limits Test  ^^^^^^\n";
  levelLimitTestsMatch_ = true;
  checkLevelLimits( params.levelIdc_, aR, key_val_decB, false );
  cout << "^^^^^^ Point Cloud Frame Level Limits: " << ( levelLimitTestsMatch_ ? "MATCH" : "DIFF" ) << std::endl;

  key_val_decA.clear();
  key_val_decB.clear();
  logFileTestsMatch_ = true;
  cout << "\n ^^^^^^ Post Reconstruction Point Cloud Frame Log Files Check ^^^^^^ \n";
  fileDecA = params.path_ + "enc_rec_pcframe_log.txt";
  fileDecB = params.path_ + "dec_rec_pcframe_log.txt";
  if ( !compareLogFiles( fileDecA, fileDecB, recPcframeKeys, key_val_decA, key_val_decB ) ) {
    std::string tmp = fileDecA + " with " + fileDecB;
    errMsg.warn( "\n ******* Please Check ******* \n", tmp );
  }
  cout << "^^^^^^ Post Reconstruction Point Cloud Frame Log Files: " << ( logFileTestsMatch_ ? "MATCH" : "DIFF" )
       << std::endl;

  key_val_decA.clear();
  key_val_decB.clear();
  logFileTestsMatch_ = true;
  cout << "\n ^^^^^^ Picture Log Files Check ^^^^^^";
  fileDecA = params.path_ + "enc_picture_log.txt";
  fileDecB = params.path_ + "dec_picture_log.txt";
  if ( !compareLogFiles( fileDecA, fileDecB, pictureKeys, key_val_decA, key_val_decB ) ) {
    std::string tmp = fileDecA + " with " + fileDecB;
    errMsg.warn( "\n ******* Please Check ******* \n", tmp );
  }
  cout << "^^^^^^  Picture Log Files: " << ( logFileTestsMatch_ ? "MATCH" : "DIFF" ) << std::endl;

  cout << "\n File Check Tests (matched / total): " << logFilesMatchCount_ << " / " << logFilesCount_ << std::endl;
  cout << "\n Level Limits Tests ( passed / total): " << ( levelLimitsCount_ - levelLimitsExceedCount_ ) << " / "
       << levelLimitsCount_ << std::endl;
}

bool PCCConformance::compareLogFiles( std::string&                    fNameEnc,
                                      std::string&                    fNameDec,
                                      const std::vector<std::string>& keyList,
                                      KeyValMaps&                     key_val_enc,
                                      KeyValMaps&                     key_val_dec ) {
  PCCConfigurationFileParser cfr( keyList );
  if ( !cfr.parseFile( fNameEnc, key_val_enc ) ) {
    cout << " Encoder File " << fNameEnc << " not exist \n";
    logFileTestsMatch_ = false;
    logFilesCount_++;
    return false;
  }
  if ( !cfr.parseFile( fNameDec, key_val_dec ) ) {
    cout << " Decoder File " << fNameDec << " not exist \n";
    logFileTestsMatch_ = false;
    logFilesCount_++;
    return false;
  }
  if ( key_val_enc.size() != key_val_dec.size() ) {
    cout << " Encoder File Has " << key_val_enc.size() << " Lines \n";
    cout << " Decoder File Has " << key_val_dec.size() << " Lines \n";
    logFileTestsMatch_ = false;
    logFilesCount_++;
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
        logFilesMatchCount_++;
      } else {
        cout << "(Enc: " << left << setw( 30 ) << it->first << ", " << setw( 32 ) << it->second << " )"
             << " **DIFF**  ";
        cout << "(Dec: " << left << setw( 30 ) << it->first << ", " << setw( 32 ) << dec[it->first] << " )" << endl;
        logFileTestsMatch_ = false;
      }
      logFilesCount_++;
    }
  }
  return true;
}

void PCCConformance::checkLevelLimits( uint8_t levelIdc, double aR, KeyValMaps& key_val_map, bool atlasFlag ) {
  std::map<std::string, size_t> maxLevelLimit;
  std::map<std::string, size_t> maxLevelLimitPerSecond;
  std::deque<PCCDynamicData>    dataWindow;
  std::vector<std::string>      agrData;
  PCCErrorMessage               error_rep;
  agrData.resize( 3 );

  int64_t clockTick     = -1;
  int64_t frmPerSecMin1 = ( int64_t )( 1 / aR ) - 1;
  uint8_t levelIdx      = ( uint8_t )( 2 * ( levelIdc / 30.0 - 1 ) );
  if ( levelIdx >= 6 ) {
    error_rep.error(
        " Dynamic Conformance Check Cannot Be Done: level indicator should be in multiples of 30 i.e. 30 - 105 for "
        "levels 1 to 3.5 \n" );
    levelLimitTestsMatch_ = false;
    levelLimitsCount_++;
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
    maxLevelLimitPerSecond.emplace( "AtlasTotalNumProjPatches", ASPSLevelTable[levelIdx][MaxProjPatchesPerSec] );
    maxLevelLimitPerSecond.emplace( "AtlasTotalNumRawPatches", ASPSLevelTable[levelIdx][MaxRawPatchesPerSec] );
    maxLevelLimitPerSecond.emplace( "AtlasTotalNumEomPatches", ASPSLevelTable[levelIdx][MaxEomPatchesPerSec] );
    agrData[0] = "AtlasTotalNumProjPatches";
    agrData[1] = "AtlasTotalNumRawPatches";
    agrData[2] = "AtlasTotalNumEomPatches";
  } else {
    maxLevelLimit.emplace( "NumProjPoints", V3CLevelTable[levelIdx][MaxNumProjPoints] );
    maxLevelLimit.emplace( "NumEomPoints", V3CLevelTable[levelIdx][MaxNumEomPoints] );
    maxLevelLimit.emplace( "NumRawPoints", V3CLevelTable[levelIdx][MaxNumRawPoints] );
    maxLevelLimitPerSecond.emplace( "NumProjPoints", V3CLevelTable[levelIdx][MaxNumProjPointsPerSec] );
    maxLevelLimitPerSecond.emplace( "NumEomPoints", V3CLevelTable[levelIdx][MaxNumEomPointsPerSec] );
    maxLevelLimitPerSecond.emplace( "NumRawPoints", V3CLevelTable[levelIdx][MaxNumRawPointsPerSec] );
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
        cout << "\n" << kvp.first << " Value : " << value << " Exceeds Max. Limit " << maxLevelLimit[kvp.first] << endl;
        levelLimitsExceedCount_++;
        levelLimitsCount_++;
        levelLimitTestsMatch_ = false;
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
          maxValue = maxLevelLimitPerSecond[agrData[n]];
          if ( totalPerSec.data_[n] > maxValue ) {
            printf( " %s MaxPerSec %zu Exceeds Table A-6 Specified Limit %zu \n", agrData[n].c_str(),
                    totalPerSec.data_[n], maxValue );
            levelLimitTestsMatch_ = false;
          }
        }
        totalPerSec -= dataWindow.front();
        dataWindow.pop_front();
      }
    }
  }
  levelLimitsCount_++;
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
