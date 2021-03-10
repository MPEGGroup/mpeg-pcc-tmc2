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

#include <iostream>
#include <fstream>
#include <sstream>
#include <deque>

#include "PCCConformanceParser.h"

using namespace pcc;
using namespace std;

bool checkFiles(std::string&, std::string&, const std::vector<std::string>&, KeyValMaps&, KeyValMaps&);
void checkConformance(uint8_t, double, KeyValMaps&, bool);
template<typename T>
inline bool
checkLimit(const std::string& keyVal, T&, T&);
template<typename T>
inline void
convertString(const std::string& keyValue, T& val);

int main(int argc, char* argv[]) {

    ErrorMessage errMsg;
    
    if (argc == 1) {
        errMsg.error("argc Equal 1");
        exit(-1);
    }

    bool atlasFlag;
    uint8_t version = 0, levelIdc, frmPerSec;
    
    std::string rootDir = argv[1];  //Root directory of conformance files + prefix e.g. C:\\Test\\pcc_conformance\\Bin\\S26C03R03_
    levelIdc = atoi(argv[2]); // Level Idc 
    frmPerSec = atoi(argv[3]); // Frame Per Second

    double aR = 1. / frmPerSec;
        
    

    KeyValMaps key_val_decA, key_val_decB;

    cout << "\n MPEG PCC Conformance version " <<  version << endl;
    std::string   fileDecA = rootDir + "enc_atlas_log.txt";
    std::string   fileDecB = rootDir + "dec_atlas_log.txt";
    cout << "\n ^^^^^^Checking Atlas Log Files^^^^^^\n\n";
    if(!checkFiles(fileDecA, fileDecB, atlasKeys, key_val_decA, key_val_decB)) {
    std::string tmp = fileDecA + "  " + fileDecB;
    errMsg.warn("\n ******* Files Do Not Have Equal Lines  ******* \n", tmp);
    }
    checkConformance(levelIdc, aR, key_val_decB, true);
    key_val_decA.clear();
    key_val_decB.clear();
    cout << "\n ^^^^^^Checking Tile Log Files ^^^^^^\n\n";
    fileDecA = rootDir + "enc_tile_log.txt";
    fileDecB = rootDir + "dec_tile_log.txt";
    if(!checkFiles(fileDecA, fileDecB, tileKeys, key_val_decA, key_val_decB)) {
        std::string tmp = fileDecA + "  " + fileDecB;
        errMsg.warn("\n ******* Files Do Not Have Equal Lines ******* \n", tmp);
    }
    key_val_decA.clear();
    key_val_decB.clear();
    cout << "\n ^^^^^^Checking Point Cloud Frame Log Files ^^^^^^\n\n";
    fileDecA = rootDir + "enc_pcframe_log.txt";
    fileDecB = rootDir + "dec_pcframe_log.txt";
    if (!checkFiles(fileDecA, fileDecB, pcframeKeys, key_val_decA, key_val_decB)) {
        std::string tmp = fileDecA + "  " + fileDecB;
        errMsg.warn("\n ******* Files Do Not Have Equal Lines  ******* \n", tmp );
    }
    key_val_decA.clear();
    key_val_decB.clear();
    cout << "\n ^^^^^^Checking Picture Log Files ^^^^^^\n\n";
    fileDecA = rootDir + "enc_picture_log.txt";
    fileDecB = rootDir + "dec_picture_log.txt";
    if (!checkFiles(fileDecA, fileDecB, pictureKeys, key_val_decA, key_val_decB)) {
        std::string tmp = fileDecA + "  " + fileDecB;
        errMsg.warn("\n ******* Files Do Not Have Equal Lines  ******* \n", tmp);
    }
    return 0;
}


bool checkFiles(std::string& fNameEnc, std::string& fNameDec, const std::vector<std::string>& keyList, KeyValMaps& key_val_enc, KeyValMaps& key_val_dec) {

    CfrFileParser cfr(keyList);

    cfr.parseFile(fNameEnc, key_val_enc);
    cfr.parseFile(fNameDec, key_val_dec);
    if (key_val_enc.size() != key_val_dec.size()) {
        cerr << " Encoder File Has " << key_val_enc.size() << " Lines \n";
        cerr << " Decoder File Has " << key_val_dec.size() << " Lines \n";
        return false;
    }
    size_t index = 0;
    for (auto& key_val : key_val_enc) {
        StringStringMap& dec = key_val_dec[index++];
        StringStringMap::iterator it;
        for (it = key_val.begin(); it != key_val.end(); ++it) {
            if (!dec[it->first].compare(it->second)) {
                if (it->first == "Occupancy" || it->first == "Geometry" || it->first == "Attribute") {
                    cout << "\n-------" << it->first << "-------\n" << endl;
                    continue;
                }
                if (it->first == "AtlasFrameIndex") cout << "\n";
                cout << "(Enc: " << it->first << ", " << it->second << ")" << " **MATCH** ";
                cout << "(Dec: " << it->first << ", " << dec[it->first] << ")" << endl;
            }
            else {
                cerr << "Enc(Key, Value): " << "(" << it->first << ", " << it->second << ")" << " **DIFF** ";
                cerr << "Dec(Key, Value): " << "(" << it->first << ", " << dec[it->first] << ")" << endl;;
            }
        }
    }
}

void checkConformance(uint8_t levelIdc, double aR, KeyValMaps& key_val_map, bool atlasFlag) {

    std::map<std::string, size_t> maxLevelLimit;
    std::deque<PCCDynamicData> dataWindow;
    std::vector<std::string> agrData;
    ErrorMessage error_rep;
    agrData.resize(3);

    int64_t clockTick = -1;
    int64_t frmPerSecMin1 = (int64_t)(1 / aR) - 1;
    uint8_t levelIdx = (uint8_t)(2 * (levelIdc / 30.0 - 1));

    if (atlasFlag) {
        maxLevelLimit.emplace("VPSMapCount", V3CLevelTable[levelIdx][LevelMapCount]);
        maxLevelLimit.emplace("AttributeCount", V3CLevelTable[levelIdx][MaxNumAttributeCount]);
        maxLevelLimit.emplace("AttributeDimension", V3CLevelTable[levelIdx][MaxNumAttributeDims]);
        maxLevelLimit.emplace("ASPSFrameSize", ASPSLevelTable[levelIdx][MaxAtlasSize]);
        maxLevelLimit.emplace("NumTilesAtlasFrame", ASPSLevelTable[levelIdx][MaxNumTiles]);
        maxLevelLimit.emplace("AtlasTotalNumProjPatches", ASPSLevelTable[levelIdx][MaxNumProjPatches]);
        maxLevelLimit.emplace("AtlasTotalNumRawPatches", ASPSLevelTable[levelIdx][MaxNumRawPatches]);
        maxLevelLimit.emplace("AtlasTotalNumEomPatches", ASPSLevelTable[levelIdx][MaxNumEomPatches]);
        agrData[0] = "AtlasTotalNumProjPatches";
        agrData[1] = "AtlasTotalNumRawPatches";
        agrData[2] = "AtlasTotalNumEomPatches";
    }
    else {
        maxLevelLimit.emplace("NumProjPoints", V3CLevelTable[levelIdx][MaxNumProjPoints]);
        maxLevelLimit.emplace("NumEomPoints", V3CLevelTable[levelIdx][MaxNumEomPoints]);
        maxLevelLimit.emplace("NumRawPoints", V3CLevelTable[levelIdx][MaxNumRawPoints]);
        agrData[0] = "NumProjPoints";
        agrData[1] = "NumRawPoints";
        agrData[2] = "NumEomPoints";
    }


    // check general tier level limits A.6.1 & A.6.2
    size_t frmIdx, value, maxValue;
    
    PCCDynamicData totalPerSec{};
    for (auto& key_val_pair : key_val_map) {
        PCCDynamicData tmp;
        bool skipLineFlag = false;
        for (auto& kvp : key_val_pair) {
            if (kvp.first == "AtlasFrameIndex") {
                convertString(kvp.second, frmIdx);
                skipLineFlag = true;
                clockTick++;
            }
            if (!maxLevelLimit.count(kvp.first)) continue;
            if (!checkLimit(kvp.second, maxLevelLimit[kvp.first], value)) {
                cerr << "\n" << kvp.first << " Value : " << value << " Exceeds Max. Limit " << maxLevelLimit[kvp.first] << endl;
            }
            if (kvp.first == agrData[0] || kvp.first == agrData[1] || kvp.first == agrData[2]) {
                for (int n = 0; n < 3; n++) {
                    if (kvp.first == agrData[n]) {
                        convertString(kvp.second, tmp.data_[n]);
                        break;
                    }
                }
            }
        }
        if (!skipLineFlag) {
            totalPerSec += tmp;
            dataWindow.push_back(tmp);
            if (clockTick >= frmPerSecMin1) {
                //cout << " \nAggregate Data @ clockTick = " << clockTick << "\n";
                for (int n = 0; n < 3; n++) {
                    //cout << totalPerSec.data_[n] << ", ";
                    //cout << endl;
                    maxValue = maxLevelLimit[agrData[n]];
                    if (totalPerSec.data_[n] > maxValue)
                        printf(" %s MaxPerSec %zu Exceeds Table A-6 Specified Limit %zu \n", agrData[n], totalPerSec.data_[n], maxValue);
                }
                totalPerSec -= dataWindow.front();
                dataWindow.pop_front();
            }
        }
    }
}



template<typename T>
inline bool
checkLimit(const std::string& keyValue, T& maxVal, T& val) {

    bool error = true;
    std::istringstream keyVal_ss(keyValue, std::istringstream::in);
    keyVal_ss.exceptions(std::ios::failbit);
    keyVal_ss >> val;
    if (val > maxVal) error = false;
    return error;
}

template<typename T>
inline void
convertString(const std::string& keyValue, T& val) {
    std::istringstream keyVal_ss(keyValue, std::istringstream::in);
    keyVal_ss.exceptions(std::ios::failbit);
    keyVal_ss >> val;
    return;
}
