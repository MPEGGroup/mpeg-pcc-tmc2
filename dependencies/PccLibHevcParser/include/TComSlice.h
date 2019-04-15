/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.  
 *
 * Copyright (c) 2010-2014, ITU/ISO/IEC
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

/** \file     TComSlice.h
    \brief    slice header and SPS class (header)
*/

#ifndef __TCOMSLICE__
#define __TCOMSLICE__

#include <cstring>
#include <map>
#include <vector>
#include "CommonDef.h"
#include "TComRom.h"
#include "TComList.h"

//! \ingroup TLibCommon
//! \{

class TComPic;
class TComTrQuant;

#if SVC_EXTENSION
class TComPicYuv;
#endif
// ====================================================================================================================
// Constants
// ====================================================================================================================

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// Reference Picture Set class
class TComReferencePictureSet
{
private:
  Int  m_numberOfPictures;
  Int  m_numberOfNegativePictures;
  Int  m_numberOfPositivePictures;
  Int  m_numberOfLongtermPictures;
  Int  m_deltaPOC[MAX_NUM_REF_PICS];
  Int  m_POC[MAX_NUM_REF_PICS];
  Bool m_used[MAX_NUM_REF_PICS];
  Bool m_interRPSPrediction;
  Int  m_deltaRIdxMinus1;   
  Int  m_deltaRPS; 
  Int  m_numRefIdc; 
  Int  m_refIdc[MAX_NUM_REF_PICS+1];
  Bool m_bCheckLTMSB[MAX_NUM_REF_PICS];
  Int  m_pocLSBLT[MAX_NUM_REF_PICS];
  Int  m_deltaPOCMSBCycleLT[MAX_NUM_REF_PICS];
  Bool m_deltaPocMSBPresentFlag[MAX_NUM_REF_PICS];

public:
  TComReferencePictureSet();
  virtual ~TComReferencePictureSet();
  Int   getPocLSBLT(Int i)                       { return m_pocLSBLT[i]; }
  Void  setPocLSBLT(Int i, Int x)                { m_pocLSBLT[i] = x; }
  Int   getDeltaPocMSBCycleLT(Int i)             { return m_deltaPOCMSBCycleLT[i]; }
  Void  setDeltaPocMSBCycleLT(Int i, Int x)      { m_deltaPOCMSBCycleLT[i] = x; }
  Bool  getDeltaPocMSBPresentFlag(Int i)         { return m_deltaPocMSBPresentFlag[i]; }
  Void  setDeltaPocMSBPresentFlag(Int i, Bool x) { m_deltaPocMSBPresentFlag[i] = x;    }
  Void setUsed(Int bufferNum, Bool used);
  Void setDeltaPOC(Int bufferNum, Int deltaPOC);
  Void setPOC(Int bufferNum, Int deltaPOC);
  Void setNumberOfPictures(Int numberOfPictures);
  Void setCheckLTMSBPresent(Int bufferNum, Bool b );
  Bool getCheckLTMSBPresent(Int bufferNum);

  Int  getUsed(Int bufferNum);
  Int  getDeltaPOC(Int bufferNum);
  Int  getPOC(Int bufferNum);
  Int  getNumberOfPictures();

  Void setNumberOfNegativePictures(Int number)  { m_numberOfNegativePictures = number; }
  Int  getNumberOfNegativePictures()            { return m_numberOfNegativePictures; }
  Void setNumberOfPositivePictures(Int number)  { m_numberOfPositivePictures = number; }
  Int  getNumberOfPositivePictures()            { return m_numberOfPositivePictures; }
  Void setNumberOfLongtermPictures(Int number)  { m_numberOfLongtermPictures = number; }
  Int  getNumberOfLongtermPictures()            { return m_numberOfLongtermPictures; }

  Void setInterRPSPrediction(Bool flag)         { m_interRPSPrediction = flag; }
  Bool getInterRPSPrediction()                  { return m_interRPSPrediction; }
  Void setDeltaRIdxMinus1(Int x)                { m_deltaRIdxMinus1 = x; }
  Int  getDeltaRIdxMinus1()                     { return m_deltaRIdxMinus1; }
  Void setDeltaRPS(Int x)                       { m_deltaRPS = x; }
  Int  getDeltaRPS()                            { return m_deltaRPS; }
  Void setNumRefIdc(Int x)                      { m_numRefIdc = x; }
  Int  getNumRefIdc()                           { return m_numRefIdc; }

  Void setRefIdc(Int bufferNum, Int refIdc);
  Int  getRefIdc(Int bufferNum);

  Void sortDeltaPOC();
  Void printDeltaPOC();
};

/// Reference Picture Set set class
class TComRPSList
{
private:
  Int  m_numberOfReferencePictureSets;
  TComReferencePictureSet* m_referencePictureSets;
  
public:
  TComRPSList();
  virtual ~TComRPSList();
  
  Void  create  (Int numberOfEntries);
  Void  destroy();


  TComReferencePictureSet* getReferencePictureSet(Int referencePictureSetNum);
  Int getNumberOfReferencePictureSets();
  Void setNumberOfReferencePictureSets(Int numberOfReferencePictureSets);
};

/// SCALING_LIST class
class TComScalingList
{
public:
  TComScalingList();
  virtual ~TComScalingList();
  Void     setScalingListPresentFlag    (Bool b)                               { m_scalingListPresentFlag = b;    }
  Bool     getScalingListPresentFlag    ()                                     { return m_scalingListPresentFlag; }
  Int*     getScalingListAddress          (UInt sizeId, UInt listId)           { return m_scalingListCoef[sizeId][listId]; } //!< get matrix coefficient
  Bool     checkPredMode                  (UInt sizeId, UInt listId);
  Void     setRefMatrixId                 (UInt sizeId, UInt listId, UInt u)   { m_refMatrixId[sizeId][listId] = u;    }     //!< set reference matrix ID
  UInt     getRefMatrixId                 (UInt sizeId, UInt listId)           { return m_refMatrixId[sizeId][listId]; }     //!< get reference matrix ID
  Int*     getScalingListDefaultAddress   (UInt sizeId, UInt listId);                                                        //!< get default matrix coefficient
  Void     processDefaultMatrix            (UInt sizeId, UInt listId);
  Void     setScalingListDC               (UInt sizeId, UInt listId, UInt u)   { m_scalingListDC[sizeId][listId] = u; }      //!< set DC value

  Int      getScalingListDC               (UInt sizeId, UInt listId)           { return m_scalingListDC[sizeId][listId]; }   //!< get DC value
  Void     checkDcOfMatrix                ();
  Void     processRefMatrix               (UInt sizeId, UInt listId , UInt refListId );
  Bool     xParseScalingList              (Char* pchFile);

private:
  Void     init                    ();
  Void     destroy                 ();
  Int      m_scalingListDC               [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM]; //!< the DC value of the matrix coefficient for 16x16
  Bool     m_useDefaultScalingMatrixFlag [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM]; //!< UseDefaultScalingMatrixFlag
  UInt     m_refMatrixId                 [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM]; //!< RefMatrixID
  Bool     m_scalingListPresentFlag;                                                //!< flag for using default matrix
  UInt     m_predMatrixId                [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM]; //!< reference list index
  Int      *m_scalingListCoef            [SCALING_LIST_SIZE_NUM][SCALING_LIST_NUM]; //!< quantization matrix                                            
};

class ProfileTierLevel
{
  Int     m_profileSpace;
  Bool    m_tierFlag;
  Int     m_profileIdc;
  Bool    m_profileCompatibilityFlag[32];
  Int     m_levelIdc;

  Bool m_progressiveSourceFlag;
  Bool m_interlacedSourceFlag;
  Bool m_nonPackedConstraintFlag;
  Bool m_frameOnlyConstraintFlag;
  
public:
  ProfileTierLevel();

  Int   getProfileSpace() const   { return m_profileSpace; }
  Void  setProfileSpace(Int x)    { m_profileSpace = x; }

  Bool  getTierFlag()     const   { return m_tierFlag; }
  Void  setTierFlag(Bool x)       { m_tierFlag = x; }

  Int   getProfileIdc()   const   { return m_profileIdc; }
  Void  setProfileIdc(Int x)      { m_profileIdc = x; }

  Bool  getProfileCompatibilityFlag(Int i) const    { return m_profileCompatibilityFlag[i]; }
  Void  setProfileCompatibilityFlag(Int i, Bool x)  { m_profileCompatibilityFlag[i] = x; }

  Int   getLevelIdc()   const   { return m_levelIdc; }
  Void  setLevelIdc(Int x)      { m_levelIdc = x; }
  
  Bool getProgressiveSourceFlag() const { return m_progressiveSourceFlag; }
  Void setProgressiveSourceFlag(Bool b) { m_progressiveSourceFlag = b; }
  
  Bool getInterlacedSourceFlag() const { return m_interlacedSourceFlag; }
  Void setInterlacedSourceFlag(Bool b) { m_interlacedSourceFlag = b; }
  
  Bool getNonPackedConstraintFlag() const { return m_nonPackedConstraintFlag; }
  Void setNonPackedConstraintFlag(Bool b) { m_nonPackedConstraintFlag = b; }
  
  Bool getFrameOnlyConstraintFlag() const { return m_frameOnlyConstraintFlag; }
  Void setFrameOnlyConstraintFlag(Bool b) { m_frameOnlyConstraintFlag = b; }
#if VPS_EXTN_PROFILE_INFO
  Void copyProfileInfo(ProfileTierLevel *ptl);
#endif
};


class TComPTL
{
  ProfileTierLevel m_generalPTL;
  ProfileTierLevel m_subLayerPTL    [MAX_TLAYER-1];      // max. value of max_sub_layers_minus1 is MAX_TLAYER-1 (= 6)
  Bool m_subLayerProfilePresentFlag [MAX_TLAYER-1];
  Bool m_subLayerLevelPresentFlag   [MAX_TLAYER-1];

public:
  TComPTL();
  Bool getSubLayerProfilePresentFlag(Int i) const { return m_subLayerProfilePresentFlag[i]; }
  Void setSubLayerProfilePresentFlag(Int i, Bool x) { m_subLayerProfilePresentFlag[i] = x; }
  
  Bool getSubLayerLevelPresentFlag(Int i) const { return m_subLayerLevelPresentFlag[i]; }
  Void setSubLayerLevelPresentFlag(Int i, Bool x) { m_subLayerLevelPresentFlag[i] = x; }

  ProfileTierLevel* getGeneralPTL()  { return &m_generalPTL; }
  ProfileTierLevel* getSubLayerPTL(Int i)  { return &m_subLayerPTL[i]; }
#if VPS_EXTN_PROFILE_INFO
  Void copyProfileInfo(TComPTL *ptl);
#endif
};
/// VPS class

struct HrdSubLayerInfo
{
  Bool fixedPicRateFlag;
  Bool fixedPicRateWithinCvsFlag;
  UInt picDurationInTcMinus1;
  Bool lowDelayHrdFlag;
  UInt cpbCntMinus1;
  UInt bitRateValueMinus1[MAX_CPB_CNT][2];
  UInt cpbSizeValue      [MAX_CPB_CNT][2];
  UInt ducpbSizeValue    [MAX_CPB_CNT][2];
  UInt cbrFlag           [MAX_CPB_CNT][2];
  UInt duBitRateValue    [MAX_CPB_CNT][2];
};

class TComHRD
{
private:
  Bool m_nalHrdParametersPresentFlag;
  Bool m_vclHrdParametersPresentFlag;
  Bool m_subPicCpbParamsPresentFlag;
  UInt m_tickDivisorMinus2;
  UInt m_duCpbRemovalDelayLengthMinus1;
  Bool m_subPicCpbParamsInPicTimingSEIFlag;
  UInt m_dpbOutputDelayDuLengthMinus1;
  UInt m_bitRateScale;
  UInt m_cpbSizeScale;
  UInt m_ducpbSizeScale;
  UInt m_initialCpbRemovalDelayLengthMinus1;
  UInt m_cpbRemovalDelayLengthMinus1;
  UInt m_dpbOutputDelayLengthMinus1;
  UInt m_numDU;
  HrdSubLayerInfo m_HRD[MAX_TLAYER];

public:
  TComHRD()
  :m_nalHrdParametersPresentFlag(0)
  ,m_vclHrdParametersPresentFlag(0)
  ,m_subPicCpbParamsPresentFlag(false)
  ,m_tickDivisorMinus2(0)
  ,m_duCpbRemovalDelayLengthMinus1(0)
  ,m_subPicCpbParamsInPicTimingSEIFlag(false)
  ,m_dpbOutputDelayDuLengthMinus1(0)
  ,m_bitRateScale(0)
  ,m_cpbSizeScale(0)
  ,m_initialCpbRemovalDelayLengthMinus1(0)
  ,m_cpbRemovalDelayLengthMinus1(0)
  ,m_dpbOutputDelayLengthMinus1(0)
  {}

  virtual ~TComHRD() {}

  Void setNalHrdParametersPresentFlag       ( Bool flag )  { m_nalHrdParametersPresentFlag = flag;         }
  Bool getNalHrdParametersPresentFlag       ( )            { return m_nalHrdParametersPresentFlag;         }

  Void setVclHrdParametersPresentFlag       ( Bool flag )  { m_vclHrdParametersPresentFlag = flag;         }
  Bool getVclHrdParametersPresentFlag       ( )            { return m_vclHrdParametersPresentFlag;         }

  Void setSubPicCpbParamsPresentFlag        ( Bool flag )  { m_subPicCpbParamsPresentFlag = flag;          }
  Bool getSubPicCpbParamsPresentFlag        ( )            { return m_subPicCpbParamsPresentFlag;          }
  
  Void setTickDivisorMinus2                 ( UInt value ) { m_tickDivisorMinus2 = value;                  }
  UInt getTickDivisorMinus2                 ( )            { return m_tickDivisorMinus2;                   }

  Void setDuCpbRemovalDelayLengthMinus1     ( UInt value ) { m_duCpbRemovalDelayLengthMinus1 = value;      }
  UInt getDuCpbRemovalDelayLengthMinus1     ( )            { return m_duCpbRemovalDelayLengthMinus1;       }

  Void setSubPicCpbParamsInPicTimingSEIFlag ( Bool flag)   { m_subPicCpbParamsInPicTimingSEIFlag = flag;   }
  Bool getSubPicCpbParamsInPicTimingSEIFlag ()             { return m_subPicCpbParamsInPicTimingSEIFlag;   }

  Void setDpbOutputDelayDuLengthMinus1      (UInt value )  { m_dpbOutputDelayDuLengthMinus1 = value;       }
  UInt getDpbOutputDelayDuLengthMinus1      ()             { return m_dpbOutputDelayDuLengthMinus1;        }

  Void setBitRateScale                      ( UInt value ) { m_bitRateScale = value;                       }
  UInt getBitRateScale                      ( )            { return m_bitRateScale;                        }

  Void setCpbSizeScale                      ( UInt value ) { m_cpbSizeScale = value;                       }
  UInt getCpbSizeScale                      ( )            { return m_cpbSizeScale;                        }
  Void setDuCpbSizeScale                    ( UInt value ) { m_ducpbSizeScale = value;                     }
  UInt getDuCpbSizeScale                    ( )            { return m_ducpbSizeScale;                      }

  Void setInitialCpbRemovalDelayLengthMinus1( UInt value ) { m_initialCpbRemovalDelayLengthMinus1 = value; }
  UInt getInitialCpbRemovalDelayLengthMinus1( )            { return m_initialCpbRemovalDelayLengthMinus1;  }

  Void setCpbRemovalDelayLengthMinus1       ( UInt value ) { m_cpbRemovalDelayLengthMinus1 = value;        }
  UInt getCpbRemovalDelayLengthMinus1       ( )            { return m_cpbRemovalDelayLengthMinus1;         }

  Void setDpbOutputDelayLengthMinus1        ( UInt value ) { m_dpbOutputDelayLengthMinus1 = value;         }
  UInt getDpbOutputDelayLengthMinus1        ( )            { return m_dpbOutputDelayLengthMinus1;          }

  Void setFixedPicRateFlag       ( Int layer, Bool flag )  { m_HRD[layer].fixedPicRateFlag = flag;         }
  Bool getFixedPicRateFlag       ( Int layer            )  { return m_HRD[layer].fixedPicRateFlag;         }

  Void setFixedPicRateWithinCvsFlag       ( Int layer, Bool flag )  { m_HRD[layer].fixedPicRateWithinCvsFlag = flag;         }
  Bool getFixedPicRateWithinCvsFlag       ( Int layer            )  { return m_HRD[layer].fixedPicRateWithinCvsFlag;         }

  Void setPicDurationInTcMinus1  ( Int layer, UInt value ) { m_HRD[layer].picDurationInTcMinus1 = value;   }
  UInt getPicDurationInTcMinus1  ( Int layer             ) { return m_HRD[layer].picDurationInTcMinus1;    }

  Void setLowDelayHrdFlag        ( Int layer, Bool flag )  { m_HRD[layer].lowDelayHrdFlag = flag;          }
  Bool getLowDelayHrdFlag        ( Int layer            )  { return m_HRD[layer].lowDelayHrdFlag;          }

  Void setCpbCntMinus1           ( Int layer, UInt value ) { m_HRD[layer].cpbCntMinus1 = value; }
  UInt getCpbCntMinus1           ( Int layer            )  { return m_HRD[layer].cpbCntMinus1; }

  Void setBitRateValueMinus1     ( Int layer, Int cpbcnt, Int nalOrVcl, UInt value ) { m_HRD[layer].bitRateValueMinus1[cpbcnt][nalOrVcl] = value; }
  UInt getBitRateValueMinus1     ( Int layer, Int cpbcnt, Int nalOrVcl             ) { return m_HRD[layer].bitRateValueMinus1[cpbcnt][nalOrVcl];  }

  Void setCpbSizeValueMinus1     ( Int layer, Int cpbcnt, Int nalOrVcl, UInt value ) { m_HRD[layer].cpbSizeValue[cpbcnt][nalOrVcl] = value;       }
  UInt getCpbSizeValueMinus1     ( Int layer, Int cpbcnt, Int nalOrVcl            )  { return m_HRD[layer].cpbSizeValue[cpbcnt][nalOrVcl];        }
  Void setDuCpbSizeValueMinus1     ( Int layer, Int cpbcnt, Int nalOrVcl, UInt value ) { m_HRD[layer].ducpbSizeValue[cpbcnt][nalOrVcl] = value;       }
  UInt getDuCpbSizeValueMinus1     ( Int layer, Int cpbcnt, Int nalOrVcl            )  { return m_HRD[layer].ducpbSizeValue[cpbcnt][nalOrVcl];        }
  Void setDuBitRateValueMinus1     ( Int layer, Int cpbcnt, Int nalOrVcl, UInt value ) { m_HRD[layer].duBitRateValue[cpbcnt][nalOrVcl] = value;       }
  UInt getDuBitRateValueMinus1     (Int layer, Int cpbcnt, Int nalOrVcl )              { return m_HRD[layer].duBitRateValue[cpbcnt][nalOrVcl];        }
  Void setCbrFlag                ( Int layer, Int cpbcnt, Int nalOrVcl, UInt value ) { m_HRD[layer].cbrFlag[cpbcnt][nalOrVcl] = value;            }
  Bool getCbrFlag                ( Int layer, Int cpbcnt, Int nalOrVcl             ) { return m_HRD[layer].cbrFlag[cpbcnt][nalOrVcl];             }

  Void setNumDU                              ( UInt value ) { m_numDU = value;                            }
  UInt getNumDU                              ( )            { return m_numDU;          }
  Bool getCpbDpbDelaysPresentFlag() { return getNalHrdParametersPresentFlag() || getVclHrdParametersPresentFlag(); }

#if VPS_VUI_BSP_HRD_PARAMS
Void copyCommonInformation( TComHRD *refHrd )
{
  m_nalHrdParametersPresentFlag         = refHrd->getNalHrdParametersPresentFlag();
  m_vclHrdParametersPresentFlag         = refHrd->getVclHrdParametersPresentFlag();
  m_subPicCpbParamsPresentFlag          = refHrd->getSubPicCpbParamsPresentFlag();
  m_tickDivisorMinus2                   = refHrd->getTickDivisorMinus2();
  m_duCpbRemovalDelayLengthMinus1       = refHrd->getDuCpbRemovalDelayLengthMinus1();
  m_subPicCpbParamsInPicTimingSEIFlag   = refHrd->getSubPicCpbParamsInPicTimingSEIFlag();
  m_dpbOutputDelayDuLengthMinus1        = refHrd->getDpbOutputDelayDuLengthMinus1();
  m_bitRateScale                        = refHrd->getBitRateScale();
  m_cpbSizeScale                        = refHrd->getCpbSizeScale();
  m_ducpbSizeScale                      = refHrd->getDuCpbSizeScale();
  m_initialCpbRemovalDelayLengthMinus1  = refHrd->getInitialCpbRemovalDelayLengthMinus1();
  m_cpbRemovalDelayLengthMinus1         = refHrd->getCpbRemovalDelayLengthMinus1();
  m_dpbOutputDelayLengthMinus1          = refHrd->getDpbOutputDelayLengthMinus1();
}
#endif
};

class TimingInfo
{
  Bool m_timingInfoPresentFlag;
  UInt m_numUnitsInTick;
  UInt m_timeScale;
  Bool m_pocProportionalToTimingFlag;
  Int  m_numTicksPocDiffOneMinus1;
public:
  TimingInfo()
  : m_timingInfoPresentFlag(false)
  , m_numUnitsInTick(1001)
  , m_timeScale(60000)
  , m_pocProportionalToTimingFlag(false)
  , m_numTicksPocDiffOneMinus1(0) {}

  Void setTimingInfoPresentFlag             ( Bool flag )  { m_timingInfoPresentFlag = flag;               }
  Bool getTimingInfoPresentFlag             ( )            { return m_timingInfoPresentFlag;               }

  Void setNumUnitsInTick                    ( UInt value ) { m_numUnitsInTick = value;                     }
  UInt getNumUnitsInTick                    ( )            { return m_numUnitsInTick;                      }

  Void setTimeScale                         ( UInt value ) { m_timeScale = value;                          }
  UInt getTimeScale                         ( )            { return m_timeScale;                           }
  
  Bool getPocProportionalToTimingFlag       ( )            { return m_pocProportionalToTimingFlag;         }
  Void setPocProportionalToTimingFlag       (Bool x      ) { m_pocProportionalToTimingFlag = x;            }
  
  Int  getNumTicksPocDiffOneMinus1          ( )            { return m_numTicksPocDiffOneMinus1;            }
  Void setNumTicksPocDiffOneMinus1          (Int x       ) { m_numTicksPocDiffOneMinus1 = x;               }
};

class Window
{
private:
  Bool          m_enabledFlag;
  Int           m_winLeftOffset;
  Int           m_winRightOffset;
  Int           m_winTopOffset;
  Int           m_winBottomOffset;
#if P0312_VERT_PHASE_ADJ
  Bool          m_vertPhasePositionEnableFlag;
#endif
public:
  Window()
  : m_enabledFlag (false)
  , m_winLeftOffset     (0)
  , m_winRightOffset    (0)
  , m_winTopOffset      (0)
  , m_winBottomOffset   (0)
#if P0312_VERT_PHASE_ADJ
  , m_vertPhasePositionEnableFlag(false)  
#endif
  { }

  Bool          getWindowEnabledFlag() const      { return m_enabledFlag; }
#if P0312_VERT_PHASE_ADJ
  Void          resetWindow()                     { m_enabledFlag = false; m_winLeftOffset = m_winRightOffset = m_winTopOffset = m_winBottomOffset = 0; m_vertPhasePositionEnableFlag = false; } 
#else
  Void          resetWindow()                     { m_enabledFlag = false; m_winLeftOffset = m_winRightOffset = m_winTopOffset = m_winBottomOffset = 0;} 
#endif
  Int           getWindowLeftOffset() const       { return m_enabledFlag ? m_winLeftOffset : 0; }
  Void          setWindowLeftOffset(Int val)      { if(val) {m_winLeftOffset = val; m_enabledFlag = true;} }
  Int           getWindowRightOffset() const      { return m_enabledFlag ? m_winRightOffset : 0; }
  Void          setWindowRightOffset(Int val)     { if(val) {m_winRightOffset = val; m_enabledFlag = true;} }
  Int           getWindowTopOffset() const        { return m_enabledFlag ? m_winTopOffset : 0; }
  Void          setWindowTopOffset(Int val)       { if(val) {m_winTopOffset = val; m_enabledFlag = true;} }
  Int           getWindowBottomOffset() const     { return m_enabledFlag ? m_winBottomOffset: 0; }
  Void          setWindowBottomOffset(Int val)    { if(val) {m_winBottomOffset = val; m_enabledFlag = true;} }
#if P0312_VERT_PHASE_ADJ
  Bool          getVertPhasePositionEnableFlag() const     { return m_vertPhasePositionEnableFlag;  }
  Void          setVertPhasePositionEnableFlag(Bool val)    { m_vertPhasePositionEnableFlag = val;  }
#endif
#if REF_REGION_OFFSET && RESAMPLING_FIX
  Bool          hasEqualOffset(const Window& ref) const
  {
    return (    this->getWindowLeftOffset()   == ref.getWindowLeftOffset()
             && this->getWindowTopOffset()    == ref.getWindowTopOffset()
             && this->getWindowRightOffset()  == ref.getWindowRightOffset()
             && this->getWindowBottomOffset() == ref.getWindowBottomOffset() );
  }
#endif

#if P0312_VERT_PHASE_ADJ
  Void          setWindow(Int offsetLeft, Int offsetLRight, Int offsetLTop, Int offsetLBottom, Bool vertPhasePositionEnableFlag = 0)
#else
  Void          setWindow(Int offsetLeft, Int offsetLRight, Int offsetLTop, Int offsetLBottom)
#endif
  {
    if(offsetLeft || offsetLRight || offsetLTop || offsetLBottom) 
    {
      m_enabledFlag       = true;
      m_winLeftOffset     = offsetLeft;
      m_winRightOffset    = offsetLRight;
      m_winTopOffset      = offsetLTop;
      m_winBottomOffset   = offsetLBottom;
    }
#if P0312_VERT_PHASE_ADJ
    m_vertPhasePositionEnableFlag = vertPhasePositionEnableFlag;    
#endif
  }
};

#if REPN_FORMAT_IN_VPS
class RepFormat
{
#if REPN_FORMAT_CONTROL_FLAG
  Bool m_chromaAndBitDepthVpsPresentFlag;
#endif 
#if AUXILIARY_PICTURES
  ChromaFormat m_chromaFormatVpsIdc;
#else
  Int  m_chromaFormatVpsIdc;
#endif
  Bool m_separateColourPlaneVpsFlag;
  Int  m_picWidthVpsInLumaSamples;
  Int  m_picHeightVpsInLumaSamples;
  Int  m_bitDepthVpsLuma;               // coded as minus8
  Int  m_bitDepthVpsChroma;             // coded as minus8

#if R0156_CONF_WINDOW_IN_REP_FORMAT
  Window m_conformanceWindowVps;
#endif

public:
  RepFormat();
#if RESOLUTION_BASED_DPB
  Void init();
  RepFormat& operator= (const RepFormat &);
  static Bool checkSameSubDpb(const RepFormat &x, const RepFormat &y);
#endif
#if REPN_FORMAT_CONTROL_FLAG
  Bool getChromaAndBitDepthVpsPresentFlag() { return m_chromaAndBitDepthVpsPresentFlag; }
  void setChromaAndBitDepthVpsPresentFlag(Bool x) { m_chromaAndBitDepthVpsPresentFlag = x; }
#endif 

#if AUXILIARY_PICTURES
  ChromaFormat getChromaFormatVpsIdc()        { return m_chromaFormatVpsIdc; }
  Void setChromaFormatVpsIdc(ChromaFormat x)  { m_chromaFormatVpsIdc = x;    }
#else
  Int  getChromaFormatVpsIdc()        { return m_chromaFormatVpsIdc; }
  Void setChromaFormatVpsIdc(Int x)   { m_chromaFormatVpsIdc = x;    }
#endif

  Bool getSeparateColourPlaneVpsFlag()        { return m_separateColourPlaneVpsFlag; }
  Void setSeparateColourPlaneVpsFlag(Bool x)  { m_separateColourPlaneVpsFlag = x;    }

  Int  getPicWidthVpsInLumaSamples()        { return m_picWidthVpsInLumaSamples; }
  Void setPicWidthVpsInLumaSamples(Int x)   { m_picWidthVpsInLumaSamples = x;    }

  Int  getPicHeightVpsInLumaSamples()        { return m_picHeightVpsInLumaSamples; }
  Void setPicHeightVpsInLumaSamples(Int x)   { m_picHeightVpsInLumaSamples = x;    }

  Int  getBitDepthVpsLuma()           { return m_bitDepthVpsLuma;   }
  Void setBitDepthVpsLuma(Int x)      { m_bitDepthVpsLuma = x;      }

  Int  getBitDepthVpsChroma()           { return m_bitDepthVpsChroma;   }
  Void setBitDepthVpsChroma(Int x)      { m_bitDepthVpsChroma = x;      }

#if R0156_CONF_WINDOW_IN_REP_FORMAT
  Window& getConformanceWindowVps()                           { return  m_conformanceWindowVps;             }
  Void    setConformanceWindowVps(Window& conformanceWindow ) { m_conformanceWindowVps = conformanceWindow; }
#endif
};
#endif
class TComVPS
{
private:
  Int         m_VPSId;
#if VPS_RESERVED_FLAGS
  Bool        m_baseLayerInternalFlag;
  Bool        m_baseLayerAvailableFlag;
#endif
  UInt        m_uiMaxTLayers;
  UInt        m_uiMaxLayers;
  Bool        m_bTemporalIdNestingFlag;
  
  UInt        m_numReorderPics[MAX_TLAYER];
  UInt        m_uiMaxDecPicBuffering[MAX_TLAYER]; 
  UInt        m_uiMaxLatencyIncrease[MAX_TLAYER]; // Really max latency increase plus 1 (value 0 expresses no limit)

  UInt        m_numHrdParameters;
#if !SVC_EXTENSION
  UInt        m_maxNuhReservedZeroLayerId;
#endif
  TComHRD*    m_hrdParameters;
  UInt*       m_hrdOpSetIdx;
  Bool*       m_cprmsPresentFlag;
#if !SVC_EXTENSION
  UInt        m_numOpSets;
  Bool        m_layerIdIncludedFlag[MAX_VPS_OP_SETS_PLUS1][MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1];
#endif
  TComPTL     m_pcPTL;
  TimingInfo  m_timingInfo;

#if SVC_EXTENSION
#if DERIVE_LAYER_ID_LIST_VARIABLES
#if Q0078_ADD_LAYER_SETS
#if NECESSARY_LAYER_FLAG
  std::vector< std::vector<Int> >     m_layerSetLayerIdList;
  std::vector<Int>                      m_numLayerInIdList;;
#else
  Int         m_layerSetLayerIdList[MAX_VPS_LAYER_SETS_PLUS1 + MAX_NUM_ADD_LAYER_SETS][MAX_VPS_LAYER_ID_PLUS1];
  Int         m_numLayerInIdList[MAX_VPS_LAYER_SETS_PLUS1 + MAX_NUM_ADD_LAYER_SETS];
#endif
#else
  Int         m_layerSetLayerIdList[MAX_VPS_LAYER_SETS_PLUS1][MAX_VPS_LAYER_ID_PLUS1];
  Int         m_numLayerInIdList[MAX_VPS_LAYER_SETS_PLUS1];
#endif
#endif
#if !P0125_REVERT_VPS_EXTN_OFFSET_TO_RESERVED
#if VPS_EXTN_OFFSET
  UInt        m_extensionOffset;
#endif
#endif
  UInt        m_maxLayerId;
  UInt        m_numLayerSets;
#if Q0078_ADD_LAYER_SETS
  UInt        m_vpsNumLayerSetsMinus1;
  Bool        m_layerIdIncludedFlag[MAX_VPS_LAYER_SETS_PLUS1 + MAX_NUM_ADD_LAYER_SETS][MAX_VPS_LAYER_ID_PLUS1];
#else
  Bool        m_layerIdIncludedFlag[MAX_VPS_LAYER_SETS_PLUS1][MAX_VPS_LAYER_ID_PLUS1];
#endif

  // ------------------------------------------
  // Variables related to VPS extensions
  // ------------------------------------------
#if VPS_EXTN_MASK_AND_DIM_INFO
#if VPS_AVC_BL_FLAG_REMOVAL
  Bool       m_nonHEVCBaseLayerFlag; 
#else
  Bool       m_avcBaseLayerFlag;                                // For now, always set to true.
#endif
  Bool       m_splittingFlag;
  Bool       m_scalabilityMask[MAX_VPS_NUM_SCALABILITY_TYPES];
  UInt       m_dimensionIdLen[MAX_VPS_NUM_SCALABILITY_TYPES];
  Bool       m_nuhLayerIdPresentFlag;
  UInt       m_layerIdInNuh[MAX_VPS_LAYER_ID_PLUS1];            // Maps layer ID in the VPS with layer_id_in_nuh
  UInt       m_dimensionId[MAX_VPS_LAYER_ID_PLUS1][MAX_VPS_NUM_SCALABILITY_TYPES];

  // Below are derived variables
  UInt       m_numScalabilityTypes;
  UInt       m_layerIdInVps[MAX_VPS_LAYER_ID_PLUS1];            // Maps layer_id_in_nuh with the layer ID in the VPS
#endif
#if BITRATE_PICRATE_SIGNALLING
#if Q0078_ADD_LAYER_SETS
  UInt       m_maxSLInLayerSetMinus1[MAX_VPS_LAYER_SETS_PLUS1 + MAX_NUM_ADD_LAYER_SETS];
#else
  UInt       m_maxSLInLayerSetMinus1[MAX_VPS_LAYER_SETS_PLUS1];
#endif
#endif
    
  Bool       m_ilpSshSignalingEnabledFlag;
#if VPS_EXTN_PROFILE_INFO
  // Profile-tier-level signalling related
  Bool       m_profilePresentFlag[MAX_VPS_LAYER_SETS_PLUS1];    // The value with index 0 will not be used.
#if !P0048_REMOVE_PROFILE_REF
  UInt       m_profileLayerSetRef[MAX_VPS_LAYER_SETS_PLUS1];    // The value with index 0 will not be used.
#endif
  std::vector<TComPTL>    m_pcPTLForExtn;  
#endif
#if VPS_EXTN_OP_LAYER_SETS
  // .. More declarations here
  // Target output layer signalling related
  UInt       m_numOutputLayerSets;
#if Q0078_ADD_LAYER_SETS
  UInt       m_outputLayerSetIdx[MAX_VPS_LAYER_SETS_PLUS1 + 2*MAX_NUM_ADD_LAYER_SETS];
  Bool       m_outputLayerFlag[MAX_VPS_LAYER_SETS_PLUS1 + 2*MAX_NUM_ADD_LAYER_SETS][MAX_VPS_LAYER_ID_PLUS1];
#else
  UInt       m_outputLayerSetIdx[MAX_VPS_LAYER_SETS_PLUS1];
  Bool       m_outputLayerFlag[MAX_VPS_LAYER_SETS_PLUS1][MAX_VPS_LAYER_ID_PLUS1];
#endif
#endif
#if VPS_EXTN_DIRECT_REF_LAYERS
  Bool       m_directDependencyFlag[MAX_VPS_LAYER_ID_PLUS1][MAX_VPS_LAYER_ID_PLUS1];
  UInt       m_numDirectRefLayers[MAX_VPS_LAYER_ID_PLUS1];
  UInt       m_refLayerId[MAX_VPS_LAYER_ID_PLUS1][MAX_VPS_LAYER_ID_PLUS1];
  UInt       m_directDepTypeLen;
#if O0096_DEFAULT_DEPENDENCY_TYPE
  Bool       m_defaultDirectDependencyTypeFlag;
  UInt       m_defaultDirectDependencyType;
#endif
  UInt       m_directDependencyType[MAX_VPS_LAYER_ID_PLUS1][MAX_VPS_LAYER_ID_PLUS1];
#endif
  UInt       m_numProfileTierLevel;
#if !VPS_EXTN_UEV_CODING
  Bool       m_moreOutputLayerSetsThanDefaultFlag;
#endif
  Int        m_numAddOutputLayerSets;
#if P0295_DEFAULT_OUT_LAYER_IDC
  UInt       m_defaultTargetOutputLayerIdc;
#else
#if O0109_DEFAULT_ONE_OUT_LAYER_IDC
  UInt       m_defaultOneTargetOutputLayerIdc;
#else
  Bool       m_defaultOneTargetOutputLayerFlag;
#endif
#endif
#if PER_LAYER_PTL
  std::vector< std::vector<Int> >  m_profileLevelTierIdx;
#else
  Int        m_profileLevelTierIdx[64];     
#endif
  Bool       m_maxOneActiveRefLayerFlag;
#if O0062_POC_LSB_NOT_PRESENT_FLAG
  Bool       m_pocLsbNotPresentFlag[MAX_VPS_LAYER_ID_PLUS1];
#endif
#if O0223_PICTURE_TYPES_ALIGN_FLAG
  Bool       m_crossLayerPictureTypeAlignFlag;
#endif
  Bool       m_crossLayerIrapAlignFlag;
#if P0068_CROSS_LAYER_ALIGNED_IDR_ONLY_FOR_IRAP_FLAG
  Bool       m_crossLayerAlignedIdrOnlyFlag;
#endif
#if O0225_MAX_TID_FOR_REF_LAYERS
  UInt       m_maxTidIlRefPicsPlus1[MAX_VPS_LAYER_ID_PLUS1 - 1][MAX_VPS_LAYER_ID_PLUS1];
#else
  UInt       m_maxTidIlRefPicsPlus1[MAX_VPS_LAYER_ID_PLUS1 - 1];
#endif
  Bool       m_maxTidRefPresentFlag;
#if VPS_TSLAYERS
  Bool       m_maxTSLayersPresentFlag;
  UInt       m_maxTSLayerMinus1[MAX_LAYERS];
#endif
#if M0040_ADAPTIVE_RESOLUTION_CHANGE
  Bool       m_singleLayerForNonIrapFlag;
#endif
#if HIGHER_LAYER_IRAP_SKIP_FLAG
  Bool       m_higherLayerIrapSkipFlag;
#endif
#if VPS_VUI_TILES_NOT_IN_USE__FLAG
  Bool       m_tilesNotInUseFlag;
  Bool       m_tilesInUseFlag[MAX_VPS_LAYER_ID_PLUS1];
  Bool       m_loopFilterNotAcrossTilesFlag[MAX_VPS_LAYER_ID_PLUS1];
#endif 
  Bool       m_tileBoundariesAlignedFlag[MAX_VPS_LAYER_ID_PLUS1][MAX_VPS_LAYER_ID_PLUS1];
#if VPS_VUI_WPP_NOT_IN_USE__FLAG
  Bool       m_wppNotInUseFlag;
  Bool       m_wppInUseFlag[MAX_VPS_LAYER_ID_PLUS1];
#endif 
#if N0160_VUI_EXT_ILP_REF    
  Bool       m_ilpRestrictedRefLayersFlag;
  Int        m_minSpatialSegmentOffsetPlus1[MAX_VPS_LAYER_ID_PLUS1][MAX_VPS_LAYER_ID_PLUS1];
  Bool       m_ctuBasedOffsetEnabledFlag   [MAX_VPS_LAYER_ID_PLUS1][MAX_VPS_LAYER_ID_PLUS1];
  Int        m_minHorizontalCtuOffsetPlus1 [MAX_VPS_LAYER_ID_PLUS1][MAX_VPS_LAYER_ID_PLUS1];
#endif 
#if VPS_VUI_VIDEO_SIGNAL
  Bool       m_vidSigPresentVpsFlag;
  Int        m_vpsVidSigInfo;
  Int        m_vpsVidSigIdx[MAX_VPS_LAYER_ID_PLUS1];
  Int        m_vpsVidFormat[16];
  Bool       m_vpsFullRangeFlag[16];
  Int        m_vpsColorPrimaries[16];
  Int        m_vpsTransChar[16];
  Int        m_vpsMatCoeff[16];
#endif

  Bool       m_bitRatePresentVpsFlag;
  Bool       m_picRatePresentVpsFlag;
  Bool       m_bitRatePresentFlag  [MAX_VPS_LAYER_SETS_PLUS1][MAX_TLAYER];
  Bool       m_picRatePresentFlag  [MAX_VPS_LAYER_SETS_PLUS1][MAX_TLAYER];
  Int        m_avgBitRate          [MAX_VPS_LAYER_SETS_PLUS1][MAX_TLAYER];
  Int        m_maxBitRate          [MAX_VPS_LAYER_SETS_PLUS1][MAX_TLAYER];
  Int        m_constPicRateIdc     [MAX_VPS_LAYER_SETS_PLUS1][MAX_TLAYER];
  Int        m_avgPicRate          [MAX_VPS_LAYER_SETS_PLUS1][MAX_TLAYER];
  
#if P0312_VERT_PHASE_ADJ
  Bool       m_vpsVuiVertPhaseInUseFlag;
#endif

#if P0300_ALT_OUTPUT_LAYER_FLAG
#if Q0078_ADD_LAYER_SETS
  Bool       m_altOutputLayerFlag[MAX_VPS_LAYER_SETS_PLUS1 + 2*MAX_NUM_ADD_LAYER_SETS];
#else
  Bool       m_altOutputLayerFlag[MAX_VPS_LAYER_SETS_PLUS1];
#endif
#else
#if O0153_ALT_OUTPUT_LAYER_FLAG
  Bool       m_altOutputLayerFlag;
#endif
#endif
#if REPN_FORMAT_IN_VPS
  Bool       m_repFormatIdxPresentFlag;
  Int        m_vpsNumRepFormats;            // coded as minus1
  RepFormat  m_vpsRepFormat[16];
  Int        m_vpsRepFormatIdx[16];
#endif
#if VIEW_ID_RELATED_SIGNALING 
#if O0109_VIEW_ID_LEN
  Int        m_viewIdLen;
#else
  Int        m_viewIdLenMinus1;
#endif
  Int        m_viewIdVal                [MAX_LAYERS];
#endif

#if O0215_PHASE_ALIGNMENT
  Bool       m_phaseAlignFlag;
#endif

#if O0092_0094_DEPENDENCY_CONSTRAINT
  Int        m_numberRefLayers[MAX_NUM_LAYER_IDS];  // number of direct and indirect reference layers of a coding layer
  Bool       m_recursiveRefLayerFlag[MAX_NUM_LAYER_IDS][MAX_NUM_LAYER_IDS];  // flag to indicate if j-th layer is a direct or indirect reference layer of i-th layer
#endif
#if Q0078_ADD_LAYER_SETS
  Int        m_numAddLayerSets;
  UInt       m_highestLayerIdxPlus1[MAX_NUM_ADD_LAYER_SETS][MAX_NUM_LAYER_IDS];
  UInt       m_predictedLayerId[MAX_NUM_LAYER_IDS][MAX_NUM_LAYER_IDS];
  UInt       m_numPredictedLayers[MAX_NUM_LAYER_IDS];
  Int        m_numIndependentLayers;
  Int        m_numLayersInTreePartition[MAX_NUM_LAYER_IDS];
  UInt       m_treePartitionLayerIdList[MAX_NUM_LAYER_IDS][MAX_NUM_LAYER_IDS];
#endif
#if SPS_DPB_PARAMS
  Int        m_TolsIdx;
#endif
#if VPS_DPB_SIZE_TABLE
  Bool       m_subLayerFlagInfoPresentFlag [MAX_VPS_OP_LAYER_SETS_PLUS1];
  Bool       m_subLayerDpbInfoPresentFlag  [MAX_VPS_OP_LAYER_SETS_PLUS1][MAX_LAYERS];
  Int        m_maxVpsDecPicBufferingMinus1 [MAX_VPS_OP_LAYER_SETS_PLUS1][MAX_LAYERS][MAX_TLAYER];
#if RESOLUTION_BASED_DPB
  Int        m_maxVpsLayerDecPicBuffMinus1 [MAX_VPS_OP_LAYER_SETS_PLUS1][MAX_LAYERS][MAX_TLAYER];  
#endif
  Int        m_maxVpsNumReorderPics        [MAX_VPS_OP_LAYER_SETS_PLUS1][MAX_LAYERS];
  Int        m_maxVpsLatencyIncreasePlus1  [MAX_VPS_OP_LAYER_SETS_PLUS1][MAX_LAYERS];
#if CHANGE_NUMSUBDPB_IDX
#if Q0078_ADD_LAYER_SETS
  Int        m_numSubDpbs                  [MAX_VPS_LAYER_SETS_PLUS1 + 2*MAX_NUM_ADD_LAYER_SETS];
#else
  Int        m_numSubDpbs                  [MAX_VPS_LAYER_SETS_PLUS1];
#endif
#else
  Int        m_numSubDpbs                  [MAX_VPS_OP_LAYER_SETS_PLUS1];
#endif
#endif

#if O0109_MOVE_VPS_VUI_FLAG
  Bool       m_vpsVuiPresentFlag;
#endif
  Bool       m_vpsExtensionFlag;

#if O0164_MULTI_LAYER_HRD
  Bool       m_vpsVuiBspHrdPresentFlag;
#if VPS_VUI_BSP_HRD_PARAMS
  Int                 m_vpsNumAddHrdParams;
  std::vector<Bool>   m_cprmsAddPresentFlag;
  std::vector<Int>    m_numSubLayerHrdMinus1;
  std::vector<TComHRD> m_bspHrd;
  Int    m_numSignalledPartitioningSchemes[MAX_VPS_OUTPUT_LAYER_SETS_PLUS1];
  Int    m_numPartitionsInSchemeMinus1    [MAX_VPS_OUTPUT_LAYER_SETS_PLUS1][16];
  Int    m_layerIncludedInPartitionFlag   [MAX_VPS_OUTPUT_LAYER_SETS_PLUS1][16][MAX_LAYERS][MAX_LAYERS];
  Int    m_numBspSchedulesMinus1          [MAX_VPS_OUTPUT_LAYER_SETS_PLUS1][16][MAX_TLAYER];
  Int    m_bspHrdIdx                      [MAX_VPS_OUTPUT_LAYER_SETS_PLUS1][16][MAX_TLAYER][31][MAX_LAYERS];
  Int    m_bspSchedIdx                    [MAX_VPS_OUTPUT_LAYER_SETS_PLUS1][16][MAX_TLAYER][31][MAX_LAYERS];
#else
  UInt       m_vpsNumBspHrdParametersMinus1;
  Bool       m_bspCprmsPresentFlag[MAX_VPS_LAYER_SETS_PLUS1];
  TComHRD    *m_bspHrd;
  UInt       m_numBitstreamPartitions[MAX_VPS_LAYER_SETS_PLUS1];
  Bool       m_layerInBspFlag[MAX_VPS_LAYER_SETS_PLUS1][8][MAX_LAYERS];
  UInt       m_numBspSchedCombinations[MAX_VPS_LAYER_SETS_PLUS1];
  UInt       m_bspCombHrdIdx[MAX_VPS_LAYER_SETS_PLUS1][16][16];
  UInt       m_bspCombSchedIdx[MAX_VPS_LAYER_SETS_PLUS1][16][16];
#endif
#endif

#if P0182_VPS_VUI_PS_FLAG
  UInt       m_SPSId[MAX_LAYERS];
  UInt       m_PPSId[MAX_LAYERS];
  UInt       m_baseLayerPSCompatibilityFlag[MAX_LAYERS];
#endif

#if !P0307_REMOVE_VPS_VUI_OFFSET
#if VPS_VUI_OFFSET
  Int        m_vpsVuiOffset;
#endif
#endif
#if P0307_VPS_NON_VUI_EXTENSION
  Int        m_vpsNonVuiExtLength;
#endif
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  Bool       m_vpsPocLsbAlignedFlag;
#endif
#if RESOLUTION_BASED_DPB
  Int        m_subDpbAssigned            [MAX_VPS_LAYER_SETS_PLUS1][MAX_LAYERS];
#endif
#if NECESSARY_LAYER_FLAG
  std::vector< std::vector<Bool> > m_necessaryLayerFlag;
  std::vector<Int>                 m_numNecessaryLayers;
#endif
#endif //SVC_EXTENSION
public:
  TComVPS();
  virtual ~TComVPS();

  void init();
#if VPS_RESERVED_FLAGS
  Void        setBaseLayerInternalFlag(Bool x) { m_baseLayerInternalFlag = x; }
  Bool        getBaseLayerInternalFlag()         { return m_baseLayerInternalFlag; }
  Void        setBaseLayerAvailableFlag(Bool x) { m_baseLayerAvailableFlag = x; }
  Bool        getBaseLayerAvailableFlag()         { return m_baseLayerAvailableFlag; }
#endif

  Void    createHrdParamBuffer()
  {
    m_hrdParameters    = new TComHRD[ getNumHrdParameters() ];
    m_hrdOpSetIdx      = new UInt   [ getNumHrdParameters() ];
    m_cprmsPresentFlag = new Bool   [ getNumHrdParameters() ];
  }

#if O0164_MULTI_LAYER_HRD
  Void    createBspHrdParamBuffer(UInt numHrds)
  {
#if VPS_VUI_BSP_HRD_PARAMS
    m_bspHrd.resize( numHrds );
    m_cprmsAddPresentFlag.resize( numHrds );
    m_numSubLayerHrdMinus1.resize( numHrds );
#else
    m_bspHrd    = new TComHRD[ numHrds ];
#endif
//    m_hrdOpSetIdx      = new UInt   [ getNumHrdParameters() ];
//    m_cprmsPresentFlag = new Bool   [ getNumHrdParameters() ];
  }
#endif
#if HRD_BPB
  Int getBspHrdParamBufferCpbCntMinus1(UInt i, UInt sl)
  {
#if VPS_VUI_BSP_HRD_PARAMS
    return m_bspHrd[i].getCpbCntMinus1(sl);
#else
    return m_bspHrd->getCpbCntMinus1(sl);
#endif
  }
#endif

  TComHRD* getHrdParameters   ( UInt i )             { return &m_hrdParameters[ i ]; }
  UInt    getHrdOpSetIdx      ( UInt i )             { return m_hrdOpSetIdx[ i ]; }
  Void    setHrdOpSetIdx      ( UInt val, UInt i )   { m_hrdOpSetIdx[ i ] = val;  }
  Bool    getCprmsPresentFlag ( UInt i )             { return m_cprmsPresentFlag[ i ]; }
  Void    setCprmsPresentFlag ( Bool val, UInt i )   { m_cprmsPresentFlag[ i ] = val;  }

  Int     getVPSId       ()                   { return m_VPSId;          }
  Void    setVPSId       (Int i)              { m_VPSId = i;             }

  UInt    getMaxTLayers  ()                   { return m_uiMaxTLayers;   }
  Void    setMaxTLayers  (UInt t)             { m_uiMaxTLayers = t; }
  
  UInt    getMaxLayers   ()                   { return m_uiMaxLayers;   }
  Void    setMaxLayers   (UInt l)             { m_uiMaxLayers = l; }

  Bool    getTemporalNestingFlag   ()         { return m_bTemporalIdNestingFlag;   }
  Void    setTemporalNestingFlag   (Bool t)   { m_bTemporalIdNestingFlag = t; }
  
  Void    setNumReorderPics(UInt v, UInt tLayer)                { m_numReorderPics[tLayer] = v;    }
  UInt    getNumReorderPics(UInt tLayer)                        { return m_numReorderPics[tLayer]; }
  
  Void    setMaxDecPicBuffering(UInt v, UInt tLayer)            { assert(tLayer < MAX_TLAYER); m_uiMaxDecPicBuffering[tLayer] = v;    }
  UInt    getMaxDecPicBuffering(UInt tLayer)                    { return m_uiMaxDecPicBuffering[tLayer]; }
  
  Void    setMaxLatencyIncrease(UInt v, UInt tLayer)            { m_uiMaxLatencyIncrease[tLayer] = v;    }
  UInt    getMaxLatencyIncrease(UInt tLayer)                    { return m_uiMaxLatencyIncrease[tLayer]; }

  UInt    getNumHrdParameters()                                 { return m_numHrdParameters; }
  Void    setNumHrdParameters(UInt v)                           { m_numHrdParameters = v;    }
  
#if !SVC_EXTENSION
  UInt    getMaxNuhReservedZeroLayerId()                        { return m_maxNuhReservedZeroLayerId; }
  Void    setMaxNuhReservedZeroLayerId(UInt v)                  { m_maxNuhReservedZeroLayerId = v;    }

  UInt    getMaxOpSets()                                        { return m_numOpSets; }
  Void    setMaxOpSets(UInt v)                                  { m_numOpSets = v;    }
#endif
  Bool    getLayerIdIncludedFlag(UInt opsIdx, UInt id)          { return m_layerIdIncludedFlag[opsIdx][id]; }
  Void    setLayerIdIncludedFlag(Bool v, UInt opsIdx, UInt id)  { m_layerIdIncludedFlag[opsIdx][id] = v;    }

  TComPTL* getPTL() { return &m_pcPTL; }
  TimingInfo* getTimingInfo() { return &m_timingInfo; }

#if SVC_EXTENSION
#if DERIVE_LAYER_ID_LIST_VARIABLES
  Int     getLayerSetLayerIdList(Int set, Int layerId)          { return m_layerSetLayerIdList[set][layerId]; }
  Void    setLayerSetLayerIdList(Int set, Int layerId, Int x)   { m_layerSetLayerIdList[set][layerId] = x;    }

  Int     getNumLayersInIdList(Int set)                          { return m_numLayerInIdList[set]; }
  Void    setNumLayersInIdList(Int set, Int x)                   { m_numLayerInIdList[set] = x;    }

  Void    deriveLayerIdListVariables();
#endif
#if VPS_DPB_SIZE_TABLE
Void      deriveNumberOfSubDpbs();
#endif

#if O0092_0094_DEPENDENCY_CONSTRAINT
  Void    setRefLayersFlags(Int currLayerId);
  Bool    getRecursiveRefLayerFlag(Int currLayerId, Int refLayerId)              { return m_recursiveRefLayerFlag[currLayerId][refLayerId];}
  Void    setRecursiveRefLayerFlag(Int currLayerId, Int refLayerId, Bool x)      { m_recursiveRefLayerFlag[currLayerId][refLayerId] = x;   }
  Int     getNumRefLayers(Int currLayerId)                                       { return m_numberRefLayers[currLayerId];                  }
  Void    setNumRefLayers();
#endif
#if Q0078_ADD_LAYER_SETS
  void    setLayerIdIncludedFlagsForAddLayerSets();
  UInt    getVpsNumLayerSetsMinus1()                                             { return m_vpsNumLayerSetsMinus1; }
  Void    setVpsNumLayerSetsMinus1(UInt x)                                       { m_vpsNumLayerSetsMinus1 = x; }
  UInt    getNumAddLayerSets()                                                   { return m_numAddLayerSets; }
  Void    setNumAddLayerSets(UInt x)                                             { m_numAddLayerSets = x; }
  UInt    getHighestLayerIdxPlus1(UInt set, UInt idx)                            { return m_highestLayerIdxPlus1[set][idx]; }
  Void    setHighestLayerIdxPlus1(UInt set, UInt idx, UInt layerIdx)             { m_highestLayerIdxPlus1[set][idx] = layerIdx; }
  Void    setPredictedLayerIds();
  UInt    getPredictedLayerId(UInt layerIdx, UInt predIdx)                       { return m_predictedLayerId[layerIdx][predIdx]; }
  Void    setPredictedLayerId(UInt layerIdx, UInt predIdx, UInt x)               { m_predictedLayerId[layerIdx][predIdx] = x; }
  UInt    getNumPredictedLayers(UInt layerId)                                    { return m_numPredictedLayers[layerId]; }
  Void    setNumPredictedLayers(UInt layerId, UInt x)                            { m_numPredictedLayers[layerId] = x; }
  Void    setTreePartitionLayerIdList();
  Int     getNumIndependentLayers()                                              { return m_numIndependentLayers; }
  Void    setNumIndependentLayers(Int x)                                         { m_numIndependentLayers = x; }
  Int     getNumLayersInTreePartition(Int idx)                                   { return m_numLayersInTreePartition[idx]; }
  Void    setNumLayersInTreePartition(Int idx, Int x)                            { m_numLayersInTreePartition[idx] = x; }
  UInt    getTreePartitionLayerId(Int idx, Int layerIdx)                         { return m_treePartitionLayerIdList[idx][layerIdx]; }
  Void    setTreePartitionLayerId(Int idx, Int layerIdx, UInt layerId)           { m_treePartitionLayerIdList[idx][layerIdx] = layerId; }
#endif
  UInt    getMaxLayerId()                                       { return m_maxLayerId;   }
  Void    setMaxLayerId(UInt v)                                 { m_maxLayerId = v;      }
  UInt    getNumLayerSets()                                     { return m_numLayerSets; }
  Void    setNumLayerSets(UInt v)                               { m_numLayerSets = v;    }
#if VPS_EXTN_MASK_AND_DIM_INFO
#if VPS_AVC_BL_FLAG_REMOVAL
  Bool   getNonHEVCBaseLayerFlag()                              { return m_nonHEVCBaseLayerFlag;       }
  Void   setNonHEVCBaseLayerFlag(Bool x)                        { m_nonHEVCBaseLayerFlag = x;          }
#else
  Bool   getAvcBaseLayerFlag()                                  { return m_avcBaseLayerFlag;       }
  Void   setAvcBaseLayerFlag(Bool x)                            { m_avcBaseLayerFlag = x;          }
#endif

  Bool   getSplittingFlag()                                     { return m_splittingFlag;          }
  Void   setSplittingFlag(Bool x)                               { m_splittingFlag = x;             }

  Bool   getScalabilityMask(Int id)                             { return m_scalabilityMask[id];    }
  Void   setScalabilityMask(Int id, Bool x)                     { m_scalabilityMask[id] = x;       }

  UInt   getDimensionIdLen(Int id)                              { return m_dimensionIdLen[id];     }
  Void   setDimensionIdLen(Int id, UInt x)                      { m_dimensionIdLen[id] = x;        }

  Bool   getNuhLayerIdPresentFlag()                             { return m_nuhLayerIdPresentFlag;  }
  Void   setNuhLayerIdPresentFlag(Bool x)                       { m_nuhLayerIdPresentFlag = x;     }

  UInt   getLayerIdInNuh(Int id)                                { return m_layerIdInNuh[id];       }
  Void   setLayerIdInNuh(Int id, UInt x)                        { m_layerIdInNuh[id] = x;          }

  UInt   getDimensionId(Int lyrId, Int id)                      { return m_dimensionId[lyrId][id]; }
  Void   setDimensionId(Int lyrId, Int id, UInt x)              { m_dimensionId[lyrId][id] = x;    }

  UInt   getNumScalabilityTypes()                               { return m_numScalabilityTypes;    }
  Void   setNumScalabilityTypes(UInt x)                         { m_numScalabilityTypes = x;       }

  UInt   getLayerIdInVps(Int id)                                { return m_layerIdInVps[id];       }
  Void   setLayerIdInVps(Int id, UInt x)                        { m_layerIdInVps[id] = x;          }
#endif
#if BITRATE_PICRATE_SIGNALLING
  UInt   getMaxSLayersInLayerSetMinus1(Int ls)                  { return m_maxSLInLayerSetMinus1[ls]; }
  Void   setMaxSLayersInLayerSetMinus1(Int ls, Int x)           { m_maxSLInLayerSetMinus1[ls] = x;    }
#endif
  Bool   getIlpSshSignalingEnabledFlag()                        { return m_ilpSshSignalingEnabledFlag;}
  Void   setIlpSshSignalingEnabledFlag(Bool x)                  { m_ilpSshSignalingEnabledFlag = x;}
#if VPS_EXTN_PROFILE_INFO
  Bool   getProfilePresentFlag(Int id)                          { return m_profilePresentFlag[id]; }
  Void   setProfilePresentFlag(Int id, Bool x)                  { m_profilePresentFlag[id] = x;    }

#if !P0048_REMOVE_PROFILE_REF
  UInt   getProfileLayerSetRef(Int id)                          { return m_profileLayerSetRef[id]; }
  Void   setProfileLayerSetRef(Int id, Bool x)                  { m_profileLayerSetRef[id] = x;    }
#endif

  std::vector<TComPTL>* getPTLForExtnPtr()                      { return &m_pcPTLForExtn;          }
  TComPTL* getPTLForExtn(Int id)                                { return &m_pcPTLForExtn[id];      }
#endif
#if VPS_EXTN_OP_LAYER_SETS
  // Target output layer signalling related
  UInt   getNumOutputLayerSets()                                { return m_numOutputLayerSets;     } 
  Void   setNumOutputLayerSets(Int x)                           { m_numOutputLayerSets = x;        }
  
  UInt   getOutputLayerSetIdx(Int idx)                          { return m_outputLayerSetIdx[idx]; }
  Void   setOutputLayerSetIdx(Int idx, UInt x)                  { m_outputLayerSetIdx[idx] = x;    }

  Bool   getOutputLayerFlag(Int layerSet, Int layerId)          { return m_outputLayerFlag[layerSet][layerId]; }
  Void   setOutputLayerFlag(Int layerSet, Int layerId, Bool x)  { m_outputLayerFlag[layerSet][layerId] = x;    }
#endif
#if VPS_EXTN_DIRECT_REF_LAYERS
  // Direct dependency of layers
  Bool   getDirectDependencyFlag(Int currLayerId, Int refLayerId)               { return m_directDependencyFlag[currLayerId][refLayerId]; }
  Void   setDirectDependencyFlag(Int currLayerId, Int refLayerId, Bool x)       { m_directDependencyFlag[currLayerId][refLayerId] = x;    }
  
  UInt   getNumDirectRefLayers(Int layerId)                                     { return m_numDirectRefLayers[layerId];                   }
  Void   setNumDirectRefLayers(Int layerId, UInt refLayerNum)                   { m_numDirectRefLayers[layerId] = refLayerNum;            }

  UInt   getRefLayerId(Int layerId, Int refLayerIdx)                            { return m_refLayerId[layerId][refLayerIdx];              }
  Void   setRefLayerId(Int layerId, Int refLayerIdx, UInt refLayerId)           { m_refLayerId[layerId][refLayerIdx] = refLayerId;        }

  UInt   getDirectDepTypeLen()                                                  { return m_directDepTypeLen;                              }
  Void   setDirectDepTypeLen(UInt x)                                            { m_directDepTypeLen = x;                                 }
#if O0096_DEFAULT_DEPENDENCY_TYPE
  Bool   getDefaultDirectDependencyTypeFlag()                                   { return m_defaultDirectDependencyTypeFlag;               }
  Void   setDefaultDirectDependecyTypeFlag(Bool x)                              { m_defaultDirectDependencyTypeFlag = x;                  }
  UInt   getDefaultDirectDependencyType()                                       { return m_defaultDirectDependencyType;                   }
  Void   setDefaultDirectDependecyType(UInt x)                                  { m_defaultDirectDependencyType = x;                      }
#endif
  UInt   getDirectDependencyType(Int currLayerId, Int refLayerId)               { return m_directDependencyType[currLayerId][refLayerId]; }
  Void   setDirectDependencyType(Int currLayerId, Int refLayerId, UInt x)       { m_directDependencyType[currLayerId][refLayerId] = x;    }
  Bool   isSamplePredictionType(Int currLayerId, Int refLayerId)                { assert(currLayerId != refLayerId); return ( ( m_directDependencyType[currLayerId][refLayerId] + 1 ) & 1 ) ? true : false; }
  Bool   isMotionPredictionType(Int currLayerId, Int refLayerId)                { assert(currLayerId != refLayerId); return ( ( ( m_directDependencyType[currLayerId][refLayerId] + 1 ) & 2 ) >> 1 ) ? true : false; }
#endif
  UInt   getNumProfileTierLevel()                                { return m_numProfileTierLevel; }
  Void   setNumProfileTierLevel(Int x)                           { m_numProfileTierLevel = x;    }

#if !VPS_EXTN_UEV_CODING
  Bool   getMoreOutputLayerSetsThanDefaultFlag()                 { return m_moreOutputLayerSetsThanDefaultFlag;}
  Void   setMoreOutputLayerSetsThanDefaultFlag(Bool x)           { m_moreOutputLayerSetsThanDefaultFlag = x   ;}
#endif

  Int    getNumAddOutputLayerSets()                              { return m_numAddOutputLayerSets; }
  Void   setNumAddOutputLayerSets(Int x)                         { m_numAddOutputLayerSets = x   ; }

#if P0295_DEFAULT_OUT_LAYER_IDC
  UInt   getDefaultTargetOutputLayerIdc()                 { return m_defaultTargetOutputLayerIdc;}
  Void   setDefaultTargetOutputLayerIdc(UInt x)           { m_defaultTargetOutputLayerIdc = x    ;}
#else
#if O0109_DEFAULT_ONE_OUT_LAYER_IDC
  UInt   getDefaultOneTargetOutputLayerIdc()                 { return m_defaultOneTargetOutputLayerIdc;}
  Void   setDefaultOneTargetOutputLayerIdc(UInt x)           { m_defaultOneTargetOutputLayerIdc= x    ;}
#else
  Bool   getDefaultOneTargetOutputLayerFlag()                 { return m_defaultOneTargetOutputLayerFlag;}
  Void   setDefaultOneTargetOutputLayerFlag(Bool x)           { m_defaultOneTargetOutputLayerFlag= x    ;}
#endif
#endif
#if PER_LAYER_PTL
  Bool getNecessaryLayerFlag(Int const i, Int const j) { return m_necessaryLayerFlag[i][j]; }
  std::vector< std::vector<Int> >* getProfileLevelTierIdx() { return &m_profileLevelTierIdx; }
  std::vector<Int>* getProfileLevelTierIdx(Int const olsIdx) { return &m_profileLevelTierIdx[olsIdx]; }
  Int    getProfileLevelTierIdx(Int const olsIdx, Int const layerIdx)     { return m_profileLevelTierIdx[olsIdx][layerIdx]; }
  Void   setProfileLevelTierIdx(Int const olsIdx, Int const layerIdx, Int const ptlIdx)     { m_profileLevelTierIdx[olsIdx][layerIdx] = ptlIdx; }
  Int calculateLenOfSyntaxElement( Int const numVal );
#else
  Int    getProfileLevelTierIdx(Int i)                        { return m_profileLevelTierIdx[i]; }
  Void   setProfileLevelTierIdx(Int i, Int x)                 { m_profileLevelTierIdx[i] = x   ; }
#endif
  Bool   getMaxOneActiveRefLayerFlag()                                          { return m_maxOneActiveRefLayerFlag;                      }
  Void   setMaxOneActiveRefLayerFlag(Bool x)                                    { m_maxOneActiveRefLayerFlag = x;                         }
#if O0062_POC_LSB_NOT_PRESENT_FLAG
  UInt   getPocLsbNotPresentFlag(Int i)                                         { return m_pocLsbNotPresentFlag[i]; }
  Void   setPocLsbNotPresentFlag(Int i, Bool x)                                 { m_pocLsbNotPresentFlag[i] = x;    }
#endif
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  Bool   getVpsPocLsbAlignedFlag()                                              { return m_vpsPocLsbAlignedFlag; }
  Void   setVpsPocLsbAlignedFlag(Bool x)                                        { m_vpsPocLsbAlignedFlag = x; }
#endif
#if O0223_PICTURE_TYPES_ALIGN_FLAG
  Bool   getCrossLayerPictureTypeAlignFlag()                                    { return m_crossLayerPictureTypeAlignFlag;                      }
  Void   setCrossLayerPictureTypeAlignFlag(Bool x)                              { m_crossLayerPictureTypeAlignFlag = x;                         }
#endif 
#if P0068_CROSS_LAYER_ALIGNED_IDR_ONLY_FOR_IRAP_FLAG
  Bool   getCrossLayerAlignedIdrOnlyFlag()                                    { return m_crossLayerAlignedIdrOnlyFlag;                      }
  Void   setCrossLayerAlignedIdrOnlyFlag(Bool x)                              { m_crossLayerAlignedIdrOnlyFlag = x;                         }
#endif 
  Bool   getCrossLayerIrapAlignFlag()                                           { return m_crossLayerIrapAlignFlag;                      }
  Void   setCrossLayerIrapAlignFlag(Bool x)                                     { m_crossLayerIrapAlignFlag = x;                         }
#if O0225_MAX_TID_FOR_REF_LAYERS
  UInt   getMaxTidIlRefPicsPlus1(Int layerId, Int refLayerId)                     { return m_maxTidIlRefPicsPlus1[layerId][refLayerId];           }
  Void   setMaxTidIlRefPicsPlus1(Int layerId, Int refLayerId, UInt maxSublayer)   { m_maxTidIlRefPicsPlus1[layerId][refLayerId] = maxSublayer;    }
#else
  UInt   getMaxTidIlRefPicsPlus1(Int layerId)                     { return m_maxTidIlRefPicsPlus1[layerId];                   }
  Void   setMaxTidIlRefPicsPlus1(Int layerId, UInt maxSublayer)   { m_maxTidIlRefPicsPlus1[layerId] = maxSublayer;            }
#endif
  Bool   getMaxTidRefPresentFlag()                                  { return m_maxTidRefPresentFlag ;}
  Void   setMaxTidRefPresentFlag(Bool x)                            { m_maxTidRefPresentFlag = x;}
#if VPS_TSLAYERS
  Bool   getMaxTSLayersPresentFlag()                                  { return m_maxTSLayersPresentFlag ;}
  Void   setMaxTSLayersPresentFlag(Bool x)                            { m_maxTSLayersPresentFlag = x;}
  UInt   getMaxTSLayersMinus1(Int layerId)                            { return m_maxTSLayerMinus1[layerId];}
  Void   setMaxTSLayersMinus1(Int layerId, UInt maxTSublayer)         { m_maxTSLayerMinus1[layerId] = maxTSublayer;}
#endif
#if M0040_ADAPTIVE_RESOLUTION_CHANGE
  Bool   getSingleLayerForNonIrapFlag()                             { return m_singleLayerForNonIrapFlag; }
  Void   setSingleLayerForNonIrapFlag(Bool x)                       { m_singleLayerForNonIrapFlag = x;    }
#endif
#if HIGHER_LAYER_IRAP_SKIP_FLAG
  Bool   getHigherLayerIrapSkipFlag()                             { return m_higherLayerIrapSkipFlag; }
  Void   setHigherLayerIrapSkipFlag(Bool x)                       { m_higherLayerIrapSkipFlag = x;    }
#endif
#if VPS_VUI_TILES_NOT_IN_USE__FLAG  
  Bool   getTilesNotInUseFlag()         { return m_tilesNotInUseFlag; }
  Void   setTilesNotInUseFlag(Bool x); 
  Bool   getTilesInUseFlag(Int currLayerId)    { return m_tilesInUseFlag[currLayerId]; }
  Void   setTilesInUseFlag(Int currLayerId, Bool x)    { m_tilesInUseFlag[currLayerId] = x; } 
  Bool   getLoopFilterNotAcrossTilesFlag(Int currLayerId)    { return m_loopFilterNotAcrossTilesFlag[currLayerId]; }
  Void   setLoopFilterNotAcrossTilesFlag(Int currLayerId, Bool x)    { m_loopFilterNotAcrossTilesFlag[currLayerId] = x; } 
#endif 
  Bool   getTileBoundariesAlignedFlag(Int currLayerId, Int refLayerId)           { return m_tileBoundariesAlignedFlag[currLayerId][refLayerId]; }
  Void   setTileBoundariesAlignedFlag(Int currLayerId, Int refLayerId, Bool x)   { m_tileBoundariesAlignedFlag[currLayerId][refLayerId] = x; } 
#if VPS_VUI_WPP_NOT_IN_USE__FLAG  
  Bool   getWppNotInUseFlag()         { return m_wppNotInUseFlag; }
  Void   setWppNotInUseFlag(Bool x); 
  Bool   getWppInUseFlag(Int currLayerId)    { return m_wppInUseFlag[currLayerId]; }
  Void   setWppInUseFlag(Int currLayerId, Bool x)    { m_wppInUseFlag[currLayerId] = x; } 
#endif 
#if N0160_VUI_EXT_ILP_REF  
  Bool   getIlpRestrictedRefLayersFlag   ( )                                        { return m_ilpRestrictedRefLayersFlag        ;}
  Void   setIlpRestrictedRefLayersFlag   ( Int val )                                { m_ilpRestrictedRefLayersFlag         = val;}
  
  Int    getMinSpatialSegmentOffsetPlus1( Int currLayerId, Int refLayerId )          { return m_minSpatialSegmentOffsetPlus1[currLayerId][refLayerId];}
  Void   setMinSpatialSegmentOffsetPlus1( Int currLayerId, Int refLayerId, Int val ) { m_minSpatialSegmentOffsetPlus1[currLayerId][refLayerId] = val;}
  
  Bool   getCtuBasedOffsetEnabledFlag   ( Int currLayerId, Int refLayerId )            { return m_ctuBasedOffsetEnabledFlag[currLayerId][refLayerId];}
  Void   setCtuBasedOffsetEnabledFlag   ( Int currLayerId, Int refLayerId, Bool flag ) { m_ctuBasedOffsetEnabledFlag[currLayerId][refLayerId] = flag;}
  
  Int    getMinHorizontalCtuOffsetPlus1 ( Int currLayerId, Int refLayerId )            { return m_minHorizontalCtuOffsetPlus1[currLayerId][refLayerId];}
  Void   setMinHorizontalCtuOffsetPlus1 ( Int currLayerId, Int refLayerId, Int val )   { m_minHorizontalCtuOffsetPlus1[currLayerId][refLayerId] = val;}  
#endif
#if VPS_VUI_VIDEO_SIGNAL
  Bool   getVideoSigPresentVpsFlag()           { return m_vidSigPresentVpsFlag; }
  Void   setVideoSigPresentVpsFlag(Bool x)     { m_vidSigPresentVpsFlag = x;    }
  Int    getNumVideoSignalInfo()               { return m_vpsVidSigInfo;        }
  Void   setNumVideoSignalInfo(Int x)          { m_vpsVidSigInfo = x;           }
  Int    getVideoSignalInfoIdx(Int idx)        { return m_vpsVidSigIdx[idx];    }
  Void   setVideoSignalInfoIdx(Int idx, Int x) { m_vpsVidSigIdx[idx] = x;       }
  Int    getVideoVPSFormat(Int idx)            { return m_vpsVidFormat[idx];    }
  Void   setVideoVPSFormat(Int idx, Int x)     { m_vpsVidFormat[idx] = x;       }
  Bool   getVideoFullRangeVpsFlag(Int idx)     { return m_vpsFullRangeFlag[idx];}
  Void   setVideoFullRangeVpsFlag(Int idx, Bool x) { m_vpsFullRangeFlag[idx] = x;   }
  Int    getColorPrimaries(Int idx)            { return m_vpsColorPrimaries[idx];   }
  Void   setColorPrimaries(Int idx, Int x)     { m_vpsColorPrimaries[idx] = x;      }
  Int    getTransCharacter(Int idx)            { return m_vpsTransChar[idx];    }
  Void   setTransCharacter(Int idx, Int x)     { m_vpsTransChar[idx] = x;       }
  Int    getMaxtrixCoeff(Int idx)              { return m_vpsMatCoeff[idx];     }
  Void   setMaxtrixCoeff(Int idx, Int x)       { m_vpsMatCoeff[idx] = x;        }
#endif
  Bool   getBitRatePresentVpsFlag()       { return m_bitRatePresentVpsFlag; }
  Void   setBitRatePresentVpsFlag(Bool x) { m_bitRatePresentVpsFlag = x;    }
  Bool   getPicRatePresentVpsFlag()       { return m_picRatePresentVpsFlag; }
  Void   setPicRatePresentVpsFlag(Bool x) { m_picRatePresentVpsFlag = x;    }
         
  Bool   getBitRatePresentFlag(Int i, Int j)          { return m_bitRatePresentFlag[i][j]; }
  Void   setBitRatePresentFlag(Int i, Int j, Bool x)  { m_bitRatePresentFlag[i][j] = x;    }
  Bool   getPicRatePresentFlag(Int i, Int j)          { return m_picRatePresentFlag[i][j]; }
  Void   setPicRatePresentFlag(Int i, Int j, Bool x)  { m_picRatePresentFlag[i][j] = x;    }
         
  Int    getAvgBitRate(Int i, Int j)          { return m_avgBitRate[i][j]; }
  Void   setAvgBitRate(Int i, Int j, Int x)   { m_avgBitRate[i][j] = x;    }
  Int    getMaxBitRate(Int i, Int j)          { return m_maxBitRate[i][j]; }
  Void   setMaxBitRate(Int i, Int j, Int x)   { m_maxBitRate[i][j] = x;    }
         
  Int    getConstPicRateIdc(Int i, Int j)          { return m_constPicRateIdc[i][j]; }
  Void   setConstPicRateIdc(Int i, Int j, Int x)   { m_constPicRateIdc[i][j] = x;    }
  Int    getAvgPicRate(Int i, Int j)          { return m_avgPicRate[i][j]; }
  Void   setAvgPicRate(Int i, Int j, Int x)   { m_avgPicRate[i][j] = x;    }
#if O0164_MULTI_LAYER_HRD
  Bool     getVpsVuiBspHrdPresentFlag()                         { return m_vpsVuiBspHrdPresentFlag;      }
  Void     setVpsVuiBspHrdPresentFlag(Bool x)                   { m_vpsVuiBspHrdPresentFlag = x;         }
#if VPS_VUI_BSP_HRD_PARAMS
  Int      getVpsNumAddHrdParams()                              { return m_vpsNumAddHrdParams; }
  Void     setVpsNumAddHrdParams(Int  i)                   { m_vpsNumAddHrdParams = i;    }

  Bool     getCprmsAddPresentFlag(Int i)                        { return m_cprmsAddPresentFlag[i];       }
  Void     setCprmsAddPresentFlag(Int  i, Bool  val)  { m_cprmsAddPresentFlag[i] = val;        }

  Int      getNumSubLayerHrdMinus1(Int i)           { return m_numSubLayerHrdMinus1[i]; }
  Void     setNumSubLayerHrdMinus1(Int i, Int val)  { m_numSubLayerHrdMinus1[i] = val; }

  TComHRD*  getBspHrd(Int i)           {return &m_bspHrd[i];}

  Int      getNumSignalledPartitioningSchemes(Int  i)                    { return m_numSignalledPartitioningSchemes[i];    }
  Void     setNumSignalledPartitioningSchemes(Int  i, Int  val)          { m_numSignalledPartitioningSchemes[i] = val;     }

  Int      getNumPartitionsInSchemeMinus1(Int  i, Int j)  { return m_numPartitionsInSchemeMinus1[i][j];}
  Void     setNumPartitionsInSchemeMinus1(Int i, Int j, Int val) { m_numPartitionsInSchemeMinus1[i][j] = val; }

  Int      getLayerIncludedInPartitionFlag(Int  i, Int j, Int k, Int l)  { return m_layerIncludedInPartitionFlag[i][j][k][l];}
  Void     setLayerIncludedInPartitionFlag(Int i, Int j, Int k, Int l, Int val) { m_layerIncludedInPartitionFlag[i][j][k][l] = val; }

  Int      getNumBspSchedulesMinus1(Int  i, Int j, Int k)         { return m_numBspSchedulesMinus1[i][j][k];}
  Void     setNumBspSchedulesMinus1(Int i, Int j, Int k, Int val) { m_numBspSchedulesMinus1[i][j][k] = val; }

  Int      getBspSchedIdx(Int  i, Int j, Int k, Int l, Int m)  { return m_bspSchedIdx[i][j][k][l][m];}
  Void     setBspSchedIdx(Int  i, Int j, Int k, Int l, Int m, Int val) { m_bspSchedIdx[i][j][k][l][m] = val; }

  Int      getBspHrdIdx(Int  i, Int j, Int k, Int l, Int m)  { return m_bspHrdIdx[i][j][k][l][m];}
  Void     setBspHrdIdx(Int  i, Int j, Int k, Int l, Int m, Int val) { m_bspHrdIdx[i][j][k][l][m] = val; }
#else
  UInt     getVpsNumBspHrdParametersMinus1()                    { return m_vpsNumBspHrdParametersMinus1; }
  Void     setVpsNumBspHrdParametersMinus1(UInt i)              { m_vpsNumBspHrdParametersMinus1 = i;    }
  Bool     getBspCprmsPresentFlag(UInt i)                       { return m_bspCprmsPresentFlag[i];       }
  Void     setBspCprmsPresentFlag(UInt i, Bool val)             { m_bspCprmsPresentFlag[i] = val;        }
  TComHRD* getBspHrd(UInt i)                                    { return &m_bspHrd[i];                    }
  UInt     getNumBitstreamPartitions(UInt i)                    { return m_numBitstreamPartitions[i];    }
  Void     setNumBitstreamPartitions(UInt i, UInt val)          { m_numBitstreamPartitions[i] = val;     }
  UInt     getLayerInBspFlag(UInt h, UInt i, UInt j)            { return m_layerInBspFlag[h][i][j];      }
  Void     setLayerInBspFlag(UInt h, UInt i, UInt j, UInt val)  { m_layerInBspFlag[h][i][j] = val;       }
  UInt     getNumBspSchedCombinations(UInt i)                   { return m_numBspSchedCombinations[i];   }
  Void     setNumBspSchedCombinations(UInt i, UInt val)         { m_numBspSchedCombinations[i] = val;    }
  UInt     getBspCombHrdIdx(UInt h, UInt i, UInt j)             { return m_bspCombHrdIdx[h][i][j];       }
  Void     setBspCombHrdIdx(UInt h, UInt i, UInt j, UInt val)   { m_bspCombHrdIdx[h][i][j] = val;        }
  UInt     getBspCombSchedIdx(UInt h, UInt i, UInt j)           { return m_bspCombSchedIdx[h][i][j];     }
  Void     setBspCombSchedIdx(UInt h, UInt i, UInt j, UInt val) { m_bspCombSchedIdx[h][i][j] = val;      }
#endif
#endif
#if P0182_VPS_VUI_PS_FLAG
  Int    getSPSId       (Int layer)                   { return m_SPSId[layer];       }
  Void   setSPSId       (Int layer, Int val)          { m_SPSId[layer] = val;        }
  Int    getPPSId       (Int layer)                   { return m_PPSId[layer];       }
  Void   setPPSId       (Int layer, Int val)          { m_PPSId[layer] = val;        }
  Void   setBaseLayerPSCompatibilityFlag (Int layer, int val)        { m_baseLayerPSCompatibilityFlag[layer] = val; }
  Int    getBaseLayerPSCompatibilityFlag (Int layer)   { return m_baseLayerPSCompatibilityFlag[layer];}
#endif

#if P0312_VERT_PHASE_ADJ
  Bool   getVpsVuiVertPhaseInUseFlag()       { return m_vpsVuiVertPhaseInUseFlag; }
  Void   setVpsVuiVertPhaseInUseFlag(Bool x) { m_vpsVuiVertPhaseInUseFlag = x;    }
#endif

#if P0300_ALT_OUTPUT_LAYER_FLAG
  Bool   getAltOuputLayerFlag(Int idx)         { return m_altOutputLayerFlag[idx]; }
  Void   setAltOuputLayerFlag(Int idx, Bool x) { m_altOutputLayerFlag[idx] = x;    }
#else
#if O0153_ALT_OUTPUT_LAYER_FLAG
  Bool   getAltOuputLayerFlag()             { return m_altOutputLayerFlag; }
  Void   setAltOuputLayerFlag(Bool x)       { m_altOutputLayerFlag = x;    }
#endif
#endif
#if REPN_FORMAT_IN_VPS
  Bool   getRepFormatIdxPresentFlag()       { return m_repFormatIdxPresentFlag; }
  Void   setRepFormatIdxPresentFlag(Bool x) { m_repFormatIdxPresentFlag = x;    }

  Int    getVpsNumRepFormats()              { return m_vpsNumRepFormats;        }
  Void   setVpsNumRepFormats(Int x)         { m_vpsNumRepFormats = x;           }

  RepFormat* getVpsRepFormat(Int idx)       { return &m_vpsRepFormat[idx];      }

  Int    getVpsRepFormatIdx(Int idx)        { return m_vpsRepFormatIdx[idx];    }
  Void   setVpsRepFormatIdx(Int idx, Int x) { m_vpsRepFormatIdx[idx] = x;       }         
#endif
#if VIEW_ID_RELATED_SIGNALING
#if O0109_VIEW_ID_LEN
  Void   setViewIdLen( Int  val )                                   { m_viewIdLen = val;  } 
  Int    getViewIdLen(  )                                           { return m_viewIdLen; } 
#else
  Void   setViewIdLenMinus1( Int  val )                             { m_viewIdLenMinus1 = val;  } 
  Int    getViewIdLenMinus1(  )                                     { return m_viewIdLenMinus1; } 
#endif

  Void   setViewIdVal( Int viewOrderIndex, Int  val )               { m_viewIdVal[viewOrderIndex] = val;  } 
  Int    getViewIdVal( Int viewOrderIndex )                         { return m_viewIdVal[viewOrderIndex]; } 
  Int    getScalabilityId(Int, ScalabilityType scalType );

  Int    getViewIndex    ( Int layerIdInNuh )                       { return getScalabilityId( getLayerIdInVps(layerIdInNuh), VIEW_ORDER_INDEX  ); }    

  Int    getNumViews();
  Int    scalTypeToScalIdx( ScalabilityType scalType );
#endif
#if !P0125_REVERT_VPS_EXTN_OFFSET_TO_RESERVED
#if VPS_EXTN_OFFSET
  Int    getExtensionOffset()                 { return m_extensionOffset;   }
  Void   setExtensionOffset( UInt offset )    { m_extensionOffset = offset; }
#endif
#endif
#if O0215_PHASE_ALIGNMENT
  Bool   getPhaseAlignFlag()                             { return m_phaseAlignFlag; }
  Void   setPhaseAlignFlag(Bool x)                       { m_phaseAlignFlag = x;    }
#endif
#if VPS_DPB_SIZE_TABLE
  Bool   getSubLayerFlagInfoPresentFlag(Int i)         {return m_subLayerFlagInfoPresentFlag[i]; }
  Void   setSubLayerFlagInfoPresentFlag(Int i, Bool x) {m_subLayerFlagInfoPresentFlag[i] = x;    }

  Bool   getSubLayerDpbInfoPresentFlag(Int i, Int j)         {return m_subLayerDpbInfoPresentFlag[i][j]; }
  Void   setSubLayerDpbInfoPresentFlag(Int i, Int j, Bool x) {m_subLayerDpbInfoPresentFlag[i][j] = x;    }

  // For the 0-th output layer set, use the date from the active SPS for base layer.
  Int    getMaxVpsDecPicBufferingMinus1(Int i, Int k, Int j)         { assert(i != 0); return m_maxVpsDecPicBufferingMinus1[i][k][j]; }
  Void   setMaxVpsDecPicBufferingMinus1(Int i, Int k, Int j, Int x) { m_maxVpsDecPicBufferingMinus1[i][k][j] = x;    }

#if RESOLUTION_BASED_DPB
  Int    getMaxVpsLayerDecPicBuffMinus1(Int i, Int k, Int j)        { assert(i != 0); return m_maxVpsLayerDecPicBuffMinus1[i][k][j]; }
  Void   setMaxVpsLayerDecPicBuffMinus1(Int i, Int k, Int j, Int x) { m_maxVpsLayerDecPicBuffMinus1[i][k][j] = x;    }
#endif

  Int    getMaxVpsNumReorderPics(Int i, Int j)        { assert(i != 0); return m_maxVpsNumReorderPics[i][j]; }
  Void   setMaxVpsNumReorderPics(Int i, Int j, Int x) { m_maxVpsNumReorderPics[i][j] = x;    }

  Int    getMaxVpsLatencyIncreasePlus1(Int i, Int j)        { assert(i != 0); return m_maxVpsLatencyIncreasePlus1[i][j]; }
  Void   setMaxVpsLatencyIncreasePlus1(Int i, Int j, Int x) { m_maxVpsLatencyIncreasePlus1[i][j] = x;    }

  Int    getNumSubDpbs(Int i)                          { return m_numSubDpbs[i]; }
  Void   setNumSubDpbs(Int i, Int x)                   { m_numSubDpbs[i] = x;    }
  Void   determineSubDpbInfoFlags();
#endif

#if O0109_MOVE_VPS_VUI_FLAG
  Bool   getVpsVuiPresentFlag()                        { return m_vpsVuiPresentFlag; }
  Void   setVpsVuiPresentFlag(Bool x)                  { m_vpsVuiPresentFlag = x;    }
#endif
  Bool   getVpsExtensionFlag()                         { return m_vpsExtensionFlag;  }
  Void   setVpsExtensionFlag(Bool x)                   { m_vpsExtensionFlag = x;     }

#if !P0307_REMOVE_VPS_VUI_OFFSET
#if VPS_VUI_OFFSET
  Int    getVpsVuiOffset()         { return m_vpsVuiOffset; }
  Void   setVpsVuiOffset(Int x)    { m_vpsVuiOffset = x; }
#endif
#endif
#if P0307_VPS_NON_VUI_EXTENSION
  Int    getVpsNonVuiExtLength()         { return m_vpsNonVuiExtLength; }
  Void   setVpsNonVuiExtLength(Int x)    { m_vpsNonVuiExtLength = x; }
#endif
#if RESOLUTION_BASED_DPB
  Void   assignSubDpbIndices();
  Int    getSubDpbAssigned  (Int lsIdx, Int layerIdx) { return m_subDpbAssigned[lsIdx][layerIdx]; }
  Int    findLayerIdxInLayerSet ( Int lsIdx, Int nuhLayerId );
#endif
#if O0164_MULTI_LAYER_HRD
  Void setBspHrdParameters( UInt hrdIdx, UInt frameRate, UInt numDU, UInt bitRate, Bool randomAccess );
#endif
#if NECESSARY_LAYER_FLAG
  Void  deriveNecessaryLayerFlag();
  Void  deriveNecessaryLayerFlag(Int const olsIdx);
  Void  checkNecessaryLayerFlagCondition();
#endif
#if SUB_LAYERS_IN_LAYER_SET
  Void  calculateMaxSLInLayerSets();
#endif
#endif //SVC_EXTENSION
};

class TComVUI
{
private:
  Bool m_aspectRatioInfoPresentFlag;
  Int  m_aspectRatioIdc;
  Int  m_sarWidth;
  Int  m_sarHeight;
  Bool m_overscanInfoPresentFlag;
  Bool m_overscanAppropriateFlag;
  Bool m_videoSignalTypePresentFlag;
  Int  m_videoFormat;
  Bool m_videoFullRangeFlag;
  Bool m_colourDescriptionPresentFlag;
  Int  m_colourPrimaries;
  Int  m_transferCharacteristics;
  Int  m_matrixCoefficients;
  Bool m_chromaLocInfoPresentFlag;
  Int  m_chromaSampleLocTypeTopField;
  Int  m_chromaSampleLocTypeBottomField;
  Bool m_neutralChromaIndicationFlag;
  Bool m_fieldSeqFlag;

  Window m_defaultDisplayWindow;
  Bool m_frameFieldInfoPresentFlag;
  Bool m_hrdParametersPresentFlag;
  Bool m_bitstreamRestrictionFlag;
  Bool m_tilesFixedStructureFlag;
  Bool m_motionVectorsOverPicBoundariesFlag;
  Bool m_restrictedRefPicListsFlag;
  Int  m_minSpatialSegmentationIdc;
  Int  m_maxBytesPerPicDenom;
  Int  m_maxBitsPerMinCuDenom;
  Int  m_log2MaxMvLengthHorizontal;
  Int  m_log2MaxMvLengthVertical;
  TComHRD m_hrdParameters;
  TimingInfo m_timingInfo;

public:
  TComVUI()
    :m_aspectRatioInfoPresentFlag(false)
    ,m_aspectRatioIdc(0)
    ,m_sarWidth(0)
    ,m_sarHeight(0)
    ,m_overscanInfoPresentFlag(false)
    ,m_overscanAppropriateFlag(false)
    ,m_videoSignalTypePresentFlag(false)
    ,m_videoFormat(5)
    ,m_videoFullRangeFlag(false)
    ,m_colourDescriptionPresentFlag(false)
    ,m_colourPrimaries(2)
    ,m_transferCharacteristics(2)
    ,m_matrixCoefficients(2)
    ,m_chromaLocInfoPresentFlag(false)
    ,m_chromaSampleLocTypeTopField(0)
    ,m_chromaSampleLocTypeBottomField(0)
    ,m_neutralChromaIndicationFlag(false)
    ,m_fieldSeqFlag(false)
    ,m_frameFieldInfoPresentFlag(false)
    ,m_hrdParametersPresentFlag(false)
    ,m_bitstreamRestrictionFlag(false)
    ,m_tilesFixedStructureFlag(false)
    ,m_motionVectorsOverPicBoundariesFlag(true)
    ,m_restrictedRefPicListsFlag(1)
    ,m_minSpatialSegmentationIdc(0)
    ,m_maxBytesPerPicDenom(2)
    ,m_maxBitsPerMinCuDenom(1)
    ,m_log2MaxMvLengthHorizontal(15)
    ,m_log2MaxMvLengthVertical(15)
  {}

  virtual ~TComVUI() {}

  Bool getAspectRatioInfoPresentFlag() { return m_aspectRatioInfoPresentFlag; }
  Void setAspectRatioInfoPresentFlag(Bool i) { m_aspectRatioInfoPresentFlag = i; }

  Int getAspectRatioIdc() { return m_aspectRatioIdc; }
  Void setAspectRatioIdc(Int i) { m_aspectRatioIdc = i; }

  Int getSarWidth() { return m_sarWidth; }
  Void setSarWidth(Int i) { m_sarWidth = i; }

  Int getSarHeight() { return m_sarHeight; }
  Void setSarHeight(Int i) { m_sarHeight = i; }

  Bool getOverscanInfoPresentFlag() { return m_overscanInfoPresentFlag; }
  Void setOverscanInfoPresentFlag(Bool i) { m_overscanInfoPresentFlag = i; }

  Bool getOverscanAppropriateFlag() { return m_overscanAppropriateFlag; }
  Void setOverscanAppropriateFlag(Bool i) { m_overscanAppropriateFlag = i; }

  Bool getVideoSignalTypePresentFlag() { return m_videoSignalTypePresentFlag; }
  Void setVideoSignalTypePresentFlag(Bool i) { m_videoSignalTypePresentFlag = i; }

  Int getVideoFormat() { return m_videoFormat; }
  Void setVideoFormat(Int i) { m_videoFormat = i; }

  Bool getVideoFullRangeFlag() { return m_videoFullRangeFlag; }
  Void setVideoFullRangeFlag(Bool i) { m_videoFullRangeFlag = i; }

  Bool getColourDescriptionPresentFlag() { return m_colourDescriptionPresentFlag; }
  Void setColourDescriptionPresentFlag(Bool i) { m_colourDescriptionPresentFlag = i; }

  Int getColourPrimaries() { return m_colourPrimaries; }
  Void setColourPrimaries(Int i) { m_colourPrimaries = i; }

  Int getTransferCharacteristics() { return m_transferCharacteristics; }
  Void setTransferCharacteristics(Int i) { m_transferCharacteristics = i; }

  Int getMatrixCoefficients() { return m_matrixCoefficients; }
  Void setMatrixCoefficients(Int i) { m_matrixCoefficients = i; }

  Bool getChromaLocInfoPresentFlag() { return m_chromaLocInfoPresentFlag; }
  Void setChromaLocInfoPresentFlag(Bool i) { m_chromaLocInfoPresentFlag = i; }

  Int getChromaSampleLocTypeTopField() { return m_chromaSampleLocTypeTopField; }
  Void setChromaSampleLocTypeTopField(Int i) { m_chromaSampleLocTypeTopField = i; }

  Int getChromaSampleLocTypeBottomField() { return m_chromaSampleLocTypeBottomField; }
  Void setChromaSampleLocTypeBottomField(Int i) { m_chromaSampleLocTypeBottomField = i; }

  Bool getNeutralChromaIndicationFlag() { return m_neutralChromaIndicationFlag; }
  Void setNeutralChromaIndicationFlag(Bool i) { m_neutralChromaIndicationFlag = i; }

  Bool getFieldSeqFlag() { return m_fieldSeqFlag; }
  Void setFieldSeqFlag(Bool i) { m_fieldSeqFlag = i; }

  Bool getFrameFieldInfoPresentFlag() { return m_frameFieldInfoPresentFlag; }
  Void setFrameFieldInfoPresentFlag(Bool i) { m_frameFieldInfoPresentFlag = i; }

  Window& getDefaultDisplayWindow()                              { return m_defaultDisplayWindow;                }
  Void    setDefaultDisplayWindow(Window& defaultDisplayWindow ) { m_defaultDisplayWindow = defaultDisplayWindow; }

  Bool getHrdParametersPresentFlag() { return m_hrdParametersPresentFlag; }
  Void setHrdParametersPresentFlag(Bool i) { m_hrdParametersPresentFlag = i; }

  Bool getBitstreamRestrictionFlag() { return m_bitstreamRestrictionFlag; }
  Void setBitstreamRestrictionFlag(Bool i) { m_bitstreamRestrictionFlag = i; }

  Bool getTilesFixedStructureFlag() { return m_tilesFixedStructureFlag; }
  Void setTilesFixedStructureFlag(Bool i) { m_tilesFixedStructureFlag = i; }

  Bool getMotionVectorsOverPicBoundariesFlag() { return m_motionVectorsOverPicBoundariesFlag; }
  Void setMotionVectorsOverPicBoundariesFlag(Bool i) { m_motionVectorsOverPicBoundariesFlag = i; }

  Bool getRestrictedRefPicListsFlag() { return m_restrictedRefPicListsFlag; }
  Void setRestrictedRefPicListsFlag(Bool b) { m_restrictedRefPicListsFlag = b; }

  Int getMinSpatialSegmentationIdc() { return m_minSpatialSegmentationIdc; }
  Void setMinSpatialSegmentationIdc(Int i) { m_minSpatialSegmentationIdc = i; }
  Int getMaxBytesPerPicDenom() { return m_maxBytesPerPicDenom; }
  Void setMaxBytesPerPicDenom(Int i) { m_maxBytesPerPicDenom = i; }

  Int getMaxBitsPerMinCuDenom() { return m_maxBitsPerMinCuDenom; }
  Void setMaxBitsPerMinCuDenom(Int i) { m_maxBitsPerMinCuDenom = i; }

  Int getLog2MaxMvLengthHorizontal() { return m_log2MaxMvLengthHorizontal; }
  Void setLog2MaxMvLengthHorizontal(Int i) { m_log2MaxMvLengthHorizontal = i; }

  Int getLog2MaxMvLengthVertical() { return m_log2MaxMvLengthVertical; }
  Void setLog2MaxMvLengthVertical(Int i) { m_log2MaxMvLengthVertical = i; }

  TComHRD* getHrdParameters                 ()             { return &m_hrdParameters; }
  TimingInfo* getTimingInfo() { return &m_timingInfo; }
};

/// SPS class
class TComSPS
{
private:
  Int         m_SPSId;
  Int         m_VPSId;
#if AUXILIARY_PICTURES
  ChromaFormat m_chromaFormatIdc;
#else
  Int         m_chromaFormatIdc;
#endif

  UInt        m_uiMaxTLayers;           // maximum number of temporal layers

#if R0279_REP_FORMAT_INBL
  Bool        m_bV1CompatibleSPSFlag;
#endif

  // Structure
  UInt        m_picWidthInLumaSamples;
  UInt        m_picHeightInLumaSamples;
  
  Int         m_log2MinCodingBlockSize;
  Int         m_log2DiffMaxMinCodingBlockSize;
  UInt        m_uiMaxCUWidth;
  UInt        m_uiMaxCUHeight;
  UInt        m_uiMaxCUDepth;

  Window      m_conformanceWindow;

  TComRPSList m_RPSList;
  Bool        m_bLongTermRefsPresent;
  Bool        m_TMVPFlagsPresent;
  Int         m_numReorderPics[MAX_TLAYER];
  
  // Tool list
  UInt        m_uiQuadtreeTULog2MaxSize;
  UInt        m_uiQuadtreeTULog2MinSize;
  UInt        m_uiQuadtreeTUMaxDepthInter;
  UInt        m_uiQuadtreeTUMaxDepthIntra;
  Bool        m_usePCM;
  UInt        m_pcmLog2MaxSize;
  UInt        m_uiPCMLog2MinSize;
  Bool        m_useAMP;

  // Parameter
  Int         m_bitDepthY;
  Int         m_bitDepthC;
  Int         m_qpBDOffsetY;
  Int         m_qpBDOffsetC;

  UInt        m_uiPCMBitDepthLuma;
  UInt        m_uiPCMBitDepthChroma;
  Bool        m_bPCMFilterDisableFlag;

  UInt        m_uiBitsForPOC;
  UInt        m_numLongTermRefPicSPS;
  UInt        m_ltRefPicPocLsbSps[33];
  Bool        m_usedByCurrPicLtSPSFlag[33];
  // Max physical transform size
  UInt        m_uiMaxTrSize;
  
  Int m_iAMPAcc[MAX_CU_DEPTH];
  Bool        m_bUseSAO; 

  Bool        m_bTemporalIdNestingFlag; // temporal_id_nesting_flag

  Bool        m_scalingListEnabledFlag;
  Bool        m_scalingListPresentFlag;
  TComScalingList*     m_scalingList;   //!< ScalingList class pointer
  UInt        m_uiMaxDecPicBuffering[MAX_TLAYER]; 
  UInt        m_uiMaxLatencyIncrease[MAX_TLAYER];  // Really max latency increase plus 1 (value 0 expresses no limit)

  Bool        m_useDF;
  Bool        m_useStrongIntraSmoothing;

  Bool        m_vuiParametersPresentFlag;
  TComVUI     m_vuiParameters;

  static const Int   m_winUnitX[MAX_CHROMA_FORMAT_IDC+1];
  static const Int   m_winUnitY[MAX_CHROMA_FORMAT_IDC+1];
  TComPTL     m_pcPTL;

#if SVC_EXTENSION
  UInt        m_layerId;
  Bool        m_extensionFlag;
  UInt        m_numScaledRefLayerOffsets;
#if R0042_PROFILE_INDICATION  
  Int         m_NumDirectRefLayers;
#endif
#if P0312_VERT_PHASE_ADJ
 Bool         m_vertPhasePositionEnableFlag[MAX_LAYERS];
#endif
#if !MOVE_SCALED_OFFSET_TO_PPS
#if O0098_SCALED_REF_LAYER_ID
  UInt        m_scaledRefLayerId[MAX_LAYERS];
#endif
  Window      m_scaledRefLayerWindow[MAX_LAYERS];
#endif
#if REPN_FORMAT_IN_VPS
  Bool m_updateRepFormatFlag;
#if O0096_REP_FORMAT_INDEX
  UInt        m_updateRepFormatIndex;
#endif
#endif
#if SCALINGLIST_INFERRING
  Bool        m_inferScalingListFlag;
  UInt        m_scalingListRefLayerId;
#endif
#endif //SVC_EXTENSION

public:
  TComSPS();
  virtual ~TComSPS();

  Int  getVPSId       ()         { return m_VPSId;          }
  Void setVPSId       (Int i)    { m_VPSId = i;             }
  Int  getSPSId       ()         { return m_SPSId;          }
  Void setSPSId       (Int i)    { m_SPSId = i;             }

#if AUXILIARY_PICTURES
  ChromaFormat getChromaFormatIdc ()         { return m_chromaFormatIdc;       }
  Void setChromaFormatIdc (ChromaFormat i)   { m_chromaFormatIdc = i;          }

  static Int getWinUnitX (Int chromaFormatIdc) { assert (chromaFormatIdc >= 0 && chromaFormatIdc <= MAX_CHROMA_FORMAT_IDC); return m_winUnitX[chromaFormatIdc];      }
  static Int getWinUnitY (Int chromaFormatIdc) { assert (chromaFormatIdc >= 0 && chromaFormatIdc <= MAX_CHROMA_FORMAT_IDC); return m_winUnitY[chromaFormatIdc];      }
#else
  Int  getChromaFormatIdc ()         { return m_chromaFormatIdc;       }
  Void setChromaFormatIdc (Int i)    { m_chromaFormatIdc = i;          }

  static Int getWinUnitX (Int chromaFormatIdc) { assert (chromaFormatIdc > 0 && chromaFormatIdc <= MAX_CHROMA_FORMAT_IDC); return m_winUnitX[chromaFormatIdc];      }
  static Int getWinUnitY (Int chromaFormatIdc) { assert (chromaFormatIdc > 0 && chromaFormatIdc <= MAX_CHROMA_FORMAT_IDC); return m_winUnitY[chromaFormatIdc];      }
#endif

#if R0279_REP_FORMAT_INBL //These two functions shall be used / called when the syntax element sps_ext_or_max_sub_layers_minus1 and V1CompatibleSPSFlag are implemented
  Bool getV1CompatibleSPSFlag()        {return m_bV1CompatibleSPSFlag;}
  Void setV1CompatibleSPSFlag(Bool x)       { m_bV1CompatibleSPSFlag = x;}
#endif

  UInt getOutputWidth () { 
    printf("getOutputWidth = %d - %d - %d  = %d \n",
        (int)m_picWidthInLumaSamples,
        2 * m_conformanceWindow.getWindowLeftOffset(),
        2 * m_conformanceWindow.getWindowRightOffset(),
        (int)m_picWidthInLumaSamples 
            - 2 * m_conformanceWindow.getWindowLeftOffset()
            - 2 * m_conformanceWindow.getWindowRightOffset()); 
    
    return  (int)m_picWidthInLumaSamples 
    - 2 * m_conformanceWindow.getWindowLeftOffset()
    - 2 * m_conformanceWindow.getWindowRightOffset(); 
  }
  UInt getOutputHeight () { 
    printf("getOutputHeight = %d - %d - %d  = %d \n",
        (int)m_picHeightInLumaSamples,
        2 * m_conformanceWindow.getWindowTopOffset(),
        2 * m_conformanceWindow.getWindowBottomOffset(),
        (int)m_picHeightInLumaSamples 
            - 2 * m_conformanceWindow.getWindowTopOffset() 
            - 2 * m_conformanceWindow.getWindowBottomOffset()); 
    return  m_picHeightInLumaSamples 
    - 2 * m_conformanceWindow.getWindowTopOffset()
    - 2 * m_conformanceWindow.getWindowBottomOffset(); 
  }

  // structure
  Void setPicWidthInLumaSamples       ( UInt u ) { m_picWidthInLumaSamples = u;        }
  UInt getPicWidthInLumaSamples       ()         { return  m_picWidthInLumaSamples;    }
  Void setPicHeightInLumaSamples      ( UInt u ) { m_picHeightInLumaSamples = u;       }
  UInt getPicHeightInLumaSamples      ()         { return  m_picHeightInLumaSamples;   }

  Window& getConformanceWindow()                           { return  m_conformanceWindow;             }
  Void    setConformanceWindow(Window& conformanceWindow ) { m_conformanceWindow = conformanceWindow; }

  UInt  getNumLongTermRefPicSPS()             { return m_numLongTermRefPicSPS; }
  Void  setNumLongTermRefPicSPS(UInt val)     { m_numLongTermRefPicSPS = val; }

  UInt  getLtRefPicPocLsbSps(UInt index)             { return m_ltRefPicPocLsbSps[index]; }
  Void  setLtRefPicPocLsbSps(UInt index, UInt val)     { m_ltRefPicPocLsbSps[index] = val; }

  Bool getUsedByCurrPicLtSPSFlag(Int i)        {return m_usedByCurrPicLtSPSFlag[i];}
  Void setUsedByCurrPicLtSPSFlag(Int i, Bool x)      { m_usedByCurrPicLtSPSFlag[i] = x;}

  Int  getLog2MinCodingBlockSize() const           { return m_log2MinCodingBlockSize; }
  Void setLog2MinCodingBlockSize(Int val)          { m_log2MinCodingBlockSize = val; }
  Int  getLog2DiffMaxMinCodingBlockSize() const    { return m_log2DiffMaxMinCodingBlockSize; }
  Void setLog2DiffMaxMinCodingBlockSize(Int val)   { m_log2DiffMaxMinCodingBlockSize = val; }

  Void setMaxCUWidth  ( UInt u ) { m_uiMaxCUWidth = u;      }
  UInt getMaxCUWidth  ()         { return  m_uiMaxCUWidth;  }
  Void setMaxCUHeight ( UInt u ) { m_uiMaxCUHeight = u;     }
  UInt getMaxCUHeight ()         { return  m_uiMaxCUHeight; }
  Void setMaxCUDepth  ( UInt u ) { m_uiMaxCUDepth = u;      }
  UInt getMaxCUDepth  ()         { return  m_uiMaxCUDepth;  }
  Void setUsePCM      ( Bool b ) { m_usePCM = b;           }
  Bool getUsePCM      ()         { return m_usePCM;        }
  Void setPCMLog2MaxSize  ( UInt u ) { m_pcmLog2MaxSize = u;      }
  UInt getPCMLog2MaxSize  ()         { return  m_pcmLog2MaxSize;  }
  Void setPCMLog2MinSize  ( UInt u ) { m_uiPCMLog2MinSize = u;      }
  UInt getPCMLog2MinSize  ()         { return  m_uiPCMLog2MinSize;  }
  Void setBitsForPOC  ( UInt u ) { m_uiBitsForPOC = u;      }
  UInt getBitsForPOC  ()         { return m_uiBitsForPOC;   }
  Bool getUseAMP() { return m_useAMP; }
  Void setUseAMP( Bool b ) { m_useAMP = b; }
  Void setQuadtreeTULog2MaxSize( UInt u ) { m_uiQuadtreeTULog2MaxSize = u;    }
  UInt getQuadtreeTULog2MaxSize()         { return m_uiQuadtreeTULog2MaxSize; }
  Void setQuadtreeTULog2MinSize( UInt u ) { m_uiQuadtreeTULog2MinSize = u;    }
  UInt getQuadtreeTULog2MinSize()         { return m_uiQuadtreeTULog2MinSize; }
  Void setQuadtreeTUMaxDepthInter( UInt u ) { m_uiQuadtreeTUMaxDepthInter = u;    }
  Void setQuadtreeTUMaxDepthIntra( UInt u ) { m_uiQuadtreeTUMaxDepthIntra = u;    }
  UInt getQuadtreeTUMaxDepthInter()         { return m_uiQuadtreeTUMaxDepthInter; }
  UInt getQuadtreeTUMaxDepthIntra()         { return m_uiQuadtreeTUMaxDepthIntra; }
  Void setNumReorderPics(Int i, UInt tlayer)              { m_numReorderPics[tlayer] = i;    }
  Int  getNumReorderPics(UInt tlayer)                     { return m_numReorderPics[tlayer]; }
  Void         createRPSList( Int numRPS );
  TComRPSList* getRPSList()                      { return &m_RPSList;          }
  Bool      getLongTermRefsPresent()         { return m_bLongTermRefsPresent; }
  Void      setLongTermRefsPresent(Bool b)   { m_bLongTermRefsPresent=b;      }
  Bool      getTMVPFlagsPresent()         { return m_TMVPFlagsPresent; }
  Void      setTMVPFlagsPresent(Bool b)   { m_TMVPFlagsPresent=b;      }  
  // physical transform
  Void setMaxTrSize   ( UInt u ) { m_uiMaxTrSize = u;       }
  UInt getMaxTrSize   ()         { return  m_uiMaxTrSize;   }
  
  // AMP accuracy
  Int       getAMPAcc   ( UInt uiDepth ) { return m_iAMPAcc[uiDepth]; }
  Void      setAMPAcc   ( UInt uiDepth, Int iAccu ) { assert( uiDepth < g_uiMaxCUDepth);  m_iAMPAcc[uiDepth] = iAccu; }

  // Bit-depth
  Int      getBitDepthY() { return m_bitDepthY; }
  Void     setBitDepthY(Int u) { m_bitDepthY = u; }
  Int      getBitDepthC() { return m_bitDepthC; }
  Void     setBitDepthC(Int u) { m_bitDepthC = u; }
  Int       getQpBDOffsetY  ()             { return m_qpBDOffsetY;   }
  Void      setQpBDOffsetY  ( Int value  ) { m_qpBDOffsetY = value;  }
  Int       getQpBDOffsetC  ()             { return m_qpBDOffsetC;   }
  Void      setQpBDOffsetC  ( Int value  ) { m_qpBDOffsetC = value;  }
  Void setUseSAO                  (Bool bVal)  {m_bUseSAO = bVal;}
  Bool getUseSAO                  ()           {return m_bUseSAO;}

  UInt      getMaxTLayers()                           { return m_uiMaxTLayers; }
  Void      setMaxTLayers( UInt uiMaxTLayers )        { assert( uiMaxTLayers <= MAX_TLAYER ); m_uiMaxTLayers = uiMaxTLayers; }

  Bool      getTemporalIdNestingFlag()                { return m_bTemporalIdNestingFlag; }
  Void      setTemporalIdNestingFlag( Bool bValue )   { m_bTemporalIdNestingFlag = bValue; }
  UInt      getPCMBitDepthLuma     ()         { return m_uiPCMBitDepthLuma;     }
  Void      setPCMBitDepthLuma     ( UInt u ) { m_uiPCMBitDepthLuma = u;        }
  UInt      getPCMBitDepthChroma   ()         { return m_uiPCMBitDepthChroma;   }
  Void      setPCMBitDepthChroma   ( UInt u ) { m_uiPCMBitDepthChroma = u;      }
  Void      setPCMFilterDisableFlag     ( Bool   bValue  )    { m_bPCMFilterDisableFlag = bValue; }
  Bool      getPCMFilterDisableFlag     ()                    { return m_bPCMFilterDisableFlag;   } 

  Bool getScalingListFlag       ()         { return m_scalingListEnabledFlag;     }
  Void setScalingListFlag       ( Bool b ) { m_scalingListEnabledFlag  = b;       }
  Bool getScalingListPresentFlag()         { return m_scalingListPresentFlag;     }
  Void setScalingListPresentFlag( Bool b ) { m_scalingListPresentFlag  = b;       }

#if SCALINGLIST_INFERRING
  Void setScalingList( TComScalingList *scalingList ) { m_scalingList = scalingList; }
#else
  Void setScalingList      ( TComScalingList *scalingList);
#endif
  TComScalingList* getScalingList ()       { return m_scalingList; }               //!< get ScalingList class pointer in SPS
  UInt getMaxDecPicBuffering  (UInt tlayer)            { return m_uiMaxDecPicBuffering[tlayer]; }
  Void setMaxDecPicBuffering  ( UInt ui, UInt tlayer ) { assert(tlayer < MAX_TLAYER);  m_uiMaxDecPicBuffering[tlayer] = ui;   }
  UInt getMaxLatencyIncrease  (UInt tlayer)            { return m_uiMaxLatencyIncrease[tlayer];   }
  Void setMaxLatencyIncrease  ( UInt ui , UInt tlayer) { m_uiMaxLatencyIncrease[tlayer] = ui;      }

  Void setUseStrongIntraSmoothing (Bool bVal)  {m_useStrongIntraSmoothing = bVal;}
  Bool getUseStrongIntraSmoothing ()           {return m_useStrongIntraSmoothing;}

  Bool getVuiParametersPresentFlag() { return m_vuiParametersPresentFlag; }
  Void setVuiParametersPresentFlag(Bool b) { m_vuiParametersPresentFlag = b; }
  TComVUI* getVuiParameters() { return &m_vuiParameters; }
  Void setHrdParameters( UInt frameRate, UInt numDU, UInt bitRate, Bool randomAccess );

  TComPTL* getPTL()     { return &m_pcPTL; }

#if SVC_EXTENSION
  Void     setLayerId(UInt layerId)            { m_layerId = layerId; }
  UInt     getLayerId()                        { return m_layerId;    }
  Int      getExtensionFlag()                  { return m_extensionFlag;  }
  Void     setExtensionFlag(Int n)             { m_extensionFlag = n;     }
#if R0042_PROFILE_INDICATION
  Int      getNumDirectRefLayers()                  { return  m_NumDirectRefLayers;  }
  Void     setNumDirectRefLayers(Int n)             {  m_NumDirectRefLayers = n;     }
#endif
#if !MOVE_SCALED_OFFSET_TO_PPS
  UInt     getNumScaledRefLayerOffsets()       { return m_numScaledRefLayerOffsets; }
  Void     setNumScaledRefLayerOffsets(Int x)  { m_numScaledRefLayerOffsets = x;    }
#if P0312_VERT_PHASE_ADJ
  Bool     getVertPhasePositionEnableFlag(Int x)          { return m_vertPhasePositionEnableFlag[x]; }
  Void     setVertPhasePositionEnableFlag(Int x, Bool b)  { m_vertPhasePositionEnableFlag[x] = b;    } 
#endif
#if O0098_SCALED_REF_LAYER_ID
  UInt     getScaledRefLayerId(Int x)          { return m_scaledRefLayerId[x]; }
  Void     setScaledRefLayerId(Int x, UInt id) { m_scaledRefLayerId[x] = id;   }
  Window&  getScaledRefLayerWindowForLayer( Int layerId );
#endif
  Window&  getScaledRefLayerWindow( Int x )   { return m_scaledRefLayerWindow[x]; }
#endif
#if REPN_FORMAT_IN_VPS
  Bool     getUpdateRepFormatFlag()       { return m_updateRepFormatFlag; }
  Void     setUpdateRepFormatFlag(Bool x) { m_updateRepFormatFlag = x;    }
#if O0096_REP_FORMAT_INDEX
  Int      getUpdateRepFormatIndex()      { return m_updateRepFormatIndex; }
  Void     setUpdateRepFormatIndex(UInt index)  { m_updateRepFormatIndex = index; }
#endif
#endif
#if SCALINGLIST_INFERRING
  Bool     getInferScalingListFlag()  { return m_inferScalingListFlag;  }
  UInt     getScalingListRefLayerId() { return m_scalingListRefLayerId; }
  Void     setInferScalingListFlag( Bool flag )     { m_inferScalingListFlag = flag;     }
  Void     setScalingListRefLayerId( UInt layerId ) { m_scalingListRefLayerId = layerId; }
#endif
#endif //SVC_EXTENSION
};

/// Reference Picture Lists class
class TComRefPicListModification
{
private:
  UInt      m_bRefPicListModificationFlagL0;  
  UInt      m_bRefPicListModificationFlagL1;  
  UInt      m_RefPicSetIdxL0[32];
  UInt      m_RefPicSetIdxL1[32];
    
public:
  TComRefPicListModification();
  virtual ~TComRefPicListModification();
  
  Void  create                    ();
  Void  destroy                   ();

  Bool       getRefPicListModificationFlagL0() { return m_bRefPicListModificationFlagL0; }
  Void       setRefPicListModificationFlagL0(Bool flag) { m_bRefPicListModificationFlagL0 = flag; }
  Bool       getRefPicListModificationFlagL1() { return m_bRefPicListModificationFlagL1; }
  Void       setRefPicListModificationFlagL1(Bool flag) { m_bRefPicListModificationFlagL1 = flag; }
  Void       setRefPicSetIdxL0(UInt idx, UInt refPicSetIdx) { m_RefPicSetIdxL0[idx] = refPicSetIdx; }
  UInt       getRefPicSetIdxL0(UInt idx) { return m_RefPicSetIdxL0[idx]; }
  Void       setRefPicSetIdxL1(UInt idx, UInt refPicSetIdx) { m_RefPicSetIdxL1[idx] = refPicSetIdx; }
  UInt       getRefPicSetIdxL1(UInt idx) { return m_RefPicSetIdxL1[idx]; }
};

/// PPS class
class TComPPS
{
private:
  Int         m_PPSId;                    // pic_parameter_set_id
  Int         m_SPSId;                    // seq_parameter_set_id
  Int         m_picInitQPMinus26;
  Bool        m_useDQP;
  Bool        m_bConstrainedIntraPred;    // constrained_intra_pred_flag
  Bool        m_bSliceChromaQpFlag;       // slicelevel_chroma_qp_flag

  // access channel
  TComSPS*    m_pcSPS;
  UInt        m_uiMaxCuDQPDepth;
  UInt        m_uiMinCuDQPSize;

  Int         m_chromaCbQpOffset;
  Int         m_chromaCrQpOffset;

  UInt        m_numRefIdxL0DefaultActive;
  UInt        m_numRefIdxL1DefaultActive;

  Bool        m_bUseWeightPred;           // Use of Weighting Prediction (P_SLICE)
  Bool        m_useWeightedBiPred;        // Use of Weighting Bi-Prediction (B_SLICE)
  Bool        m_OutputFlagPresentFlag;   // Indicates the presence of output_flag in slice header

  Bool        m_TransquantBypassEnableFlag; // Indicates presence of cu_transquant_bypass_flag in CUs.
  Bool        m_useTransformSkip;
  Bool        m_dependentSliceSegmentsEnabledFlag;     //!< Indicates the presence of dependent slices
  Bool        m_tilesEnabledFlag;              //!< Indicates the presence of tiles
  Bool        m_entropyCodingSyncEnabledFlag;  //!< Indicates the presence of wavefronts
  
  Bool     m_loopFilterAcrossTilesEnabledFlag;
  Bool             m_uniformSpacingFlag;
  Int              m_numTileColumnsMinus1;
  Int              m_numTileRowsMinus1;
  std::vector<Int> m_tileColumnWidth;
  std::vector<Int> m_tileRowHeight;

  Int      m_numSubstreams;

  Int      m_signHideFlag;

  Bool     m_cabacInitPresentFlag;
  UInt     m_encCABACTableIdx;           // Used to transmit table selection across slices

  Bool     m_sliceHeaderExtensionPresentFlag;
  Bool     m_loopFilterAcrossSlicesEnabledFlag;
  Bool     m_deblockingFilterControlPresentFlag;
  Bool     m_deblockingFilterOverrideEnabledFlag;
  Bool     m_picDisableDeblockingFilterFlag;
  Int      m_deblockingFilterBetaOffsetDiv2;    //< beta offset for deblocking filter
  Int      m_deblockingFilterTcOffsetDiv2;      //< tc offset for deblocking filter
  Bool     m_scalingListPresentFlag;
  TComScalingList*     m_scalingList;   //!< ScalingList class pointer
  Bool m_listsModificationPresentFlag;
  UInt m_log2ParallelMergeLevelMinus2;
  Int m_numExtraSliceHeaderBits;

#if SVC_EXTENSION
  Bool     m_extensionFlag;
#if SCALINGLIST_INFERRING
  UInt     m_layerId;
  Bool     m_inferScalingListFlag;
  UInt     m_scalingListRefLayerId;
#endif
#if POC_RESET_IDC
  Bool     m_pocResetInfoPresentFlag;
#endif
#if MOVE_SCALED_OFFSET_TO_PPS
  UInt     m_numScaledRefLayerOffsets;
#if O0098_SCALED_REF_LAYER_ID
  UInt     m_scaledRefLayerId[MAX_LAYERS];
#endif
  Window   m_scaledRefLayerWindow[MAX_LAYERS];
#if REF_REGION_OFFSET
  Window   m_refLayerWindow[MAX_LAYERS];
  Bool     m_scaledRefLayerOffsetPresentFlag[MAX_LAYERS];
  Bool     m_refRegionOffsetPresentFlag[MAX_LAYERS];
#endif
#if R0209_GENERIC_PHASE
  Int      m_phaseHorLuma[MAX_LAYERS];
  Int      m_phaseVerLuma[MAX_LAYERS];
  Int      m_phaseHorChroma[MAX_LAYERS];
  Int      m_phaseVerChroma[MAX_LAYERS];
  Bool     m_resamplePhaseSetPresentFlag[MAX_LAYERS];
#endif
#endif
#if Q0048_CGS_3D_ASYMLUT
  Int      m_nCGSFlag;
  Int      m_nCGSOutputBitDepthY; // not for syntax
  Int      m_nCGSOutputBitDepthC; // not for syntax
#endif
#endif

public:
  TComPPS();
  virtual ~TComPPS();
  
  Int       getPPSId ()      { return m_PPSId; }
  Void      setPPSId (Int i) { m_PPSId = i; }
  Int       getSPSId ()      { return m_SPSId; }
  Void      setSPSId (Int i) { m_SPSId = i; }
  
  Int       getPicInitQPMinus26 ()         { return  m_picInitQPMinus26; }
  Void      setPicInitQPMinus26 ( Int i )  { m_picInitQPMinus26 = i;     }
  Bool      getUseDQP ()                   { return m_useDQP;        }
  Void      setUseDQP ( Bool b )           { m_useDQP   = b;         }
  Bool      getConstrainedIntraPred ()         { return  m_bConstrainedIntraPred; }
  Void      setConstrainedIntraPred ( Bool b ) { m_bConstrainedIntraPred = b;     }
  Bool      getSliceChromaQpFlag ()         { return  m_bSliceChromaQpFlag; }
  Void      setSliceChromaQpFlag ( Bool b ) { m_bSliceChromaQpFlag = b;     }

  Void      setSPS              ( TComSPS* pcSPS ) { m_pcSPS = pcSPS; }
  TComSPS*  getSPS              ()         { return m_pcSPS;          }
  Void      setMaxCuDQPDepth    ( UInt u ) { m_uiMaxCuDQPDepth = u;   }
  UInt      getMaxCuDQPDepth    ()         { return m_uiMaxCuDQPDepth;}
  Void      setMinCuDQPSize     ( UInt u ) { m_uiMinCuDQPSize = u;    }
  UInt      getMinCuDQPSize     ()         { return m_uiMinCuDQPSize; }

  Void      setChromaCbQpOffset( Int i ) { m_chromaCbQpOffset = i;    }
  Int       getChromaCbQpOffset()        { return m_chromaCbQpOffset; }
  Void      setChromaCrQpOffset( Int i ) { m_chromaCrQpOffset = i;    }
  Int       getChromaCrQpOffset()        { return m_chromaCrQpOffset; }

  Void      setNumRefIdxL0DefaultActive(UInt ui)    { m_numRefIdxL0DefaultActive=ui;     }
  UInt      getNumRefIdxL0DefaultActive()           { return m_numRefIdxL0DefaultActive; }
  Void      setNumRefIdxL1DefaultActive(UInt ui)    { m_numRefIdxL1DefaultActive=ui;     }
  UInt      getNumRefIdxL1DefaultActive()           { return m_numRefIdxL1DefaultActive; }

  Bool getUseWP                     ()          { return m_bUseWeightPred;  }
  Bool getWPBiPred                  ()          { return m_useWeightedBiPred;     }
  Void setUseWP                     ( Bool b )  { m_bUseWeightPred = b;     }
  Void setWPBiPred                  ( Bool b )  { m_useWeightedBiPred = b;  }
  Void      setOutputFlagPresentFlag( Bool b )  { m_OutputFlagPresentFlag = b;    }
  Bool      getOutputFlagPresentFlag()          { return m_OutputFlagPresentFlag; }
  Void      setTransquantBypassEnableFlag( Bool b ) { m_TransquantBypassEnableFlag = b; }
  Bool      getTransquantBypassEnableFlag()         { return m_TransquantBypassEnableFlag; }

  Bool      getUseTransformSkip       ()         { return m_useTransformSkip;     }
  Void      setUseTransformSkip       ( Bool b ) { m_useTransformSkip  = b;       }

  Void    setLoopFilterAcrossTilesEnabledFlag  (Bool b)    { m_loopFilterAcrossTilesEnabledFlag = b; }
  Bool    getLoopFilterAcrossTilesEnabledFlag  ()          { return m_loopFilterAcrossTilesEnabledFlag;   }
  Bool    getDependentSliceSegmentsEnabledFlag() const     { return m_dependentSliceSegmentsEnabledFlag; }
  Void    setDependentSliceSegmentsEnabledFlag(Bool val)   { m_dependentSliceSegmentsEnabledFlag = val; }
  Bool    getEntropyCodingSyncEnabledFlag() const          { return m_entropyCodingSyncEnabledFlag; }
  Void    setEntropyCodingSyncEnabledFlag(Bool val)        { m_entropyCodingSyncEnabledFlag = val; }

  Void     setTilesEnabledFlag       (Bool val)                             { m_tilesEnabledFlag = val; }
  Bool     getTilesEnabledFlag       () const                               { return m_tilesEnabledFlag; }
  Void     setTileUniformSpacingFlag (Bool b)                               { m_uniformSpacingFlag = b; }
  Bool     getTileUniformSpacingFlag () const                               { return m_uniformSpacingFlag; }
  Void     setNumTileColumnsMinus1   (Int i)                                { m_numTileColumnsMinus1 = i; }
  Int      getNumTileColumnsMinus1   () const                               { return m_numTileColumnsMinus1; }
  Void     setTileColumnWidth        (const std::vector<Int>& columnWidth ) { m_tileColumnWidth = columnWidth; }
  UInt     getTileColumnWidth        (UInt columnIdx) const                 { return  m_tileColumnWidth[columnIdx]; }
  Void     setNumTileRowsMinus1      (Int i)                                { m_numTileRowsMinus1 = i; }
  Int      getTileNumRowsMinus1      () const                               { return m_numTileRowsMinus1; }
  Void     setTileRowHeight          (const std::vector<Int>& rowHeight)    { m_tileRowHeight = rowHeight;  }
  UInt     getTileRowHeight          (UInt rowIdx) const                    { return m_tileRowHeight[rowIdx]; }

  Void     setNumSubstreams    (Int numSubstreams)                     { m_numSubstreams = numSubstreams; }
  Int      getNumSubstreams    ()                                      { return m_numSubstreams; }

  Void      setSignHideFlag( Int signHideFlag ) { m_signHideFlag = signHideFlag; }
  Int       getSignHideFlag()                    { return m_signHideFlag; }

  Void     setCabacInitPresentFlag( Bool flag )     { m_cabacInitPresentFlag = flag;    }
  Void     setEncCABACTableIdx( Int idx )           { m_encCABACTableIdx = idx;         }
  Bool     getCabacInitPresentFlag()                { return m_cabacInitPresentFlag;    }
  UInt     getEncCABACTableIdx()                    { return m_encCABACTableIdx;        }
  Void     setDeblockingFilterControlPresentFlag( Bool val )  { m_deblockingFilterControlPresentFlag = val; }
  Bool     getDeblockingFilterControlPresentFlag()            { return m_deblockingFilterControlPresentFlag; }
  Void     setDeblockingFilterOverrideEnabledFlag( Bool val ) { m_deblockingFilterOverrideEnabledFlag = val; }
  Bool     getDeblockingFilterOverrideEnabledFlag()           { return m_deblockingFilterOverrideEnabledFlag; }
  Void     setPicDisableDeblockingFilterFlag(Bool val)        { m_picDisableDeblockingFilterFlag = val; }       //!< set offset for deblocking filter disabled
  Bool     getPicDisableDeblockingFilterFlag()                { return m_picDisableDeblockingFilterFlag; }      //!< get offset for deblocking filter disabled
  Void     setDeblockingFilterBetaOffsetDiv2(Int val)         { m_deblockingFilterBetaOffsetDiv2 = val; }       //!< set beta offset for deblocking filter
  Int      getDeblockingFilterBetaOffsetDiv2()                { return m_deblockingFilterBetaOffsetDiv2; }      //!< get beta offset for deblocking filter
  Void     setDeblockingFilterTcOffsetDiv2(Int val)           { m_deblockingFilterTcOffsetDiv2 = val; }               //!< set tc offset for deblocking filter
  Int      getDeblockingFilterTcOffsetDiv2()                  { return m_deblockingFilterTcOffsetDiv2; }              //!< get tc offset for deblocking filter
  Bool     getScalingListPresentFlag()         { return m_scalingListPresentFlag;     }
  Void     setScalingListPresentFlag( Bool b ) { m_scalingListPresentFlag  = b;       }

#if SCALINGLIST_INFERRING
  Void     setScalingList( TComScalingList *scalingList ) { m_scalingList = scalingList; }
#else
  Void     setScalingList      ( TComScalingList *scalingList);
#endif
  TComScalingList* getScalingList ()          { return m_scalingList; }         //!< get ScalingList class pointer in PPS
  Bool getListsModificationPresentFlag ()          { return m_listsModificationPresentFlag; }
  Void setListsModificationPresentFlag ( Bool b )  { m_listsModificationPresentFlag = b;    }
  UInt getLog2ParallelMergeLevelMinus2      ()                    { return m_log2ParallelMergeLevelMinus2; }
  Void setLog2ParallelMergeLevelMinus2      (UInt mrgLevel)       { m_log2ParallelMergeLevelMinus2 = mrgLevel; }
  Int getNumExtraSliceHeaderBits() { return m_numExtraSliceHeaderBits; }
  Void setNumExtraSliceHeaderBits(Int i) { m_numExtraSliceHeaderBits = i; }
  Void      setLoopFilterAcrossSlicesEnabledFlag ( Bool   bValue  )    { m_loopFilterAcrossSlicesEnabledFlag = bValue; }
  Bool      getLoopFilterAcrossSlicesEnabledFlag ()                    { return m_loopFilterAcrossSlicesEnabledFlag;   } 
  Bool getSliceHeaderExtensionPresentFlag   ()                    { return m_sliceHeaderExtensionPresentFlag; }
  Void setSliceHeaderExtensionPresentFlag   (Bool val)            { m_sliceHeaderExtensionPresentFlag = val; }
#if SVC_EXTENSION
  Int     getExtensionFlag()                { return m_extensionFlag;  }
  Void    setExtensionFlag(Int n)           { m_extensionFlag = n;     }
#if SCALINGLIST_INFERRING
  UInt     getLayerId() { return m_layerId; }
  Void     setLayerId( UInt layerId ) { m_layerId = layerId;            }
  Bool     getInferScalingListFlag()  { return m_inferScalingListFlag;  }
  UInt     getScalingListRefLayerId() { return m_scalingListRefLayerId; }
  Void     setInferScalingListFlag( Bool flag )     { m_inferScalingListFlag = flag;     }
  Void     setScalingListRefLayerId( UInt layerId ) { m_scalingListRefLayerId = layerId; }
#endif
#if POC_RESET_IDC
  Bool getPocResetInfoPresentFlag   ()                    { return m_pocResetInfoPresentFlag; }
  Void setPocResetInfoPresentFlag   (const Bool val)      { m_pocResetInfoPresentFlag = val; }
#endif
#if MOVE_SCALED_OFFSET_TO_PPS
  UInt     getNumScaledRefLayerOffsets()       { return m_numScaledRefLayerOffsets; }
  Void     setNumScaledRefLayerOffsets(Int x)  { m_numScaledRefLayerOffsets = x;    }
#if O0098_SCALED_REF_LAYER_ID
  UInt     getScaledRefLayerId(Int x)          { return m_scaledRefLayerId[x]; }
  Void     setScaledRefLayerId(Int x, UInt id) { m_scaledRefLayerId[x] = id;   }
  Window&  getScaledRefLayerWindowForLayer( Int layerId );
#endif
  Window&  getScaledRefLayerWindow( Int x )   { return m_scaledRefLayerWindow[x]; }
#if REF_REGION_OFFSET
  Window&  getRefLayerWindowForLayer( Int layerId );
  Window&  getRefLayerWindow( Int x )   { return m_refLayerWindow[x]; }
  Bool getScaledRefLayerOffsetPresentFlag(Int x) { return m_scaledRefLayerOffsetPresentFlag[x]; }
  Void setScaledRefLayerOffsetPresentFlag(Int x, Bool b) { m_scaledRefLayerOffsetPresentFlag[x] = b; }
  Bool getRefRegionOffsetPresentFlag(Int x) { return m_refRegionOffsetPresentFlag[x]; }
  Void setRefRegionOffsetPresentFlag(Int x, Bool b) { m_refRegionOffsetPresentFlag[x] = b; }
#endif
#if R0209_GENERIC_PHASE
  Int getPhaseHorLuma(Int x) { return m_phaseHorLuma[x]; }
  Int getPhaseVerLuma(Int x) { return m_phaseVerLuma[x]; }
  Int getPhaseHorChroma(Int x) { return m_phaseHorChroma[x]; }
  Int getPhaseVerChroma(Int x) { return m_phaseVerChroma[x]; }
  Void setPhaseHorLuma(Int x, Int val) { m_phaseHorLuma[x] = val; }
  Void setPhaseVerLuma(Int x, Int val) { m_phaseVerLuma[x] = val; }
  Void setPhaseHorChroma(Int x, Int val) { m_phaseHorChroma[x] = val; }
  Void setPhaseVerChroma(Int x, Int val) { m_phaseVerChroma[x] = val; }
  Bool getResamplePhaseSetPresentFlag(Int x) { return m_resamplePhaseSetPresentFlag[x]; }
  Void setResamplePhaseSetPresentFlag(Int x, Bool b) { m_resamplePhaseSetPresentFlag[x] = b; }
#if RESAMPLING_FIX
  Bool hasZeroResamplingPhase(Int layerId);
#endif
#endif
#endif
#if Q0048_CGS_3D_ASYMLUT
  Int     getCGSFlag()                { return m_nCGSFlag;  }
  Void    setCGSFlag(Int n)           { m_nCGSFlag = n;     }
  Int     getCGSOutputBitDepthY()     { return m_nCGSOutputBitDepthY;  }
  Void    setCGSOutputBitDepthY(Int n){ m_nCGSOutputBitDepthY = n;     }
  Int     getCGSOutputBitDepthC()     { return m_nCGSOutputBitDepthC;  }
  Void    setCGSOutputBitDepthC(Int n){ m_nCGSOutputBitDepthC = n;     }
#endif
#endif //SVC_EXTENSION
};

typedef struct
{
  // Explicit weighted prediction parameters parsed in slice header,
  // or Implicit weighted prediction parameters (8 bits depth values).
  Bool        bPresentFlag;
  UInt        uiLog2WeightDenom;
  Int         iWeight;
  Int         iOffset;

  // Weighted prediction scaling values built from above parameters (bitdepth scaled):
  Int         w, o, offset, shift, round;
} wpScalingParam;

typedef struct
{
  Int64 iAC;
  Int64 iDC;
#if O0194_WEIGHTED_PREDICTION_CGS
  Int iSamples;
#endif
} wpACDCParam;



//
// * TCH-JR
// *
/// slice header class
class TComSlice
{
  
private:
  //  Bitstream writing
  Bool       m_saoEnabledFlag;
  Bool       m_saoEnabledFlagChroma;      ///< SAO Cb&Cr enabled flag
  Int         m_iPPSId;               ///< picture parameter set ID
  Bool        m_PicOutputFlag;        ///< pic_output_flag 
  Int         m_iPOC;
  Int         m_iLastIDR;
  Int         m_iAssociatedIRAP;
#if POC_RESET_IDC_ENCODER
  Int         m_associatedIrapPocBeforeReset;
#endif
  NalUnitType m_iAssociatedIRAPType;
  static Int  m_prevTid0POC;
  TComReferencePictureSet *m_pcRPS;
  TComReferencePictureSet m_LocalRPS;
  Int         m_iBDidx; 
  TComRefPicListModification m_RefPicListModification;
  NalUnitType m_eNalUnitType;         ///< Nal unit type for the slice
  SliceType   m_eSliceType;
  Int         m_iSliceQp;
  Bool        m_dependentSliceSegmentFlag;
#if ADAPTIVE_QP_SELECTION
  Int         m_iSliceQpBase;
#endif
  Bool        m_deblockingFilterDisable;
  Bool        m_deblockingFilterOverrideFlag;      //< offsets for deblocking filter inherit from PPS
  Int         m_deblockingFilterBetaOffsetDiv2;    //< beta offset for deblocking filter
  Int         m_deblockingFilterTcOffsetDiv2;      //< tc offset for deblocking filter
  Int         m_list1IdxToList0Idx[MAX_NUM_REF];
  Int         m_aiNumRefIdx   [2];    //  for multiple reference of current slice

  Bool        m_bCheckLDC;

  //  Data
  Int         m_iSliceQpDelta;
  Int         m_iSliceQpDeltaCb;
  Int         m_iSliceQpDeltaCr;
  TComPic*    m_apcRefPicList [2][MAX_NUM_REF+1];
  Int         m_aiRefPOCList  [2][MAX_NUM_REF+1];
  Bool        m_bIsUsedAsLongTerm[2][MAX_NUM_REF+1];
  Int         m_iDepth;
  
  // referenced slice?
  Bool        m_bRefenced;

  // access channel
  TComVPS*    m_pcVPS;
  TComSPS*    m_pcSPS;
  TComPPS*    m_pcPPS;
  TComPic*    m_pcPic;
#if ADAPTIVE_QP_SELECTION
  TComTrQuant* m_pcTrQuant;
#endif  
  UInt        m_colFromL0Flag;  // collocated picture from List0 flag
  
#if SETTING_NO_OUT_PIC_PRIOR
  Bool        m_noOutputPriorPicsFlag;
  Bool        m_noRaslOutputFlag;
  Bool        m_handleCraAsBlaFlag;
#endif
  
  UInt        m_colRefIdx;
  UInt        m_maxNumMergeCand;

  Double      m_lambdas[3];

  Bool        m_abEqualRef  [2][MAX_NUM_REF][MAX_NUM_REF];
  UInt        m_uiTLayer;
  Bool        m_bTLayerSwitchingFlag;

  UInt        m_sliceMode;
  UInt        m_sliceArgument;
  UInt        m_sliceCurStartCUAddr;
  UInt        m_sliceCurEndCUAddr;
  UInt        m_sliceIdx;
  UInt        m_sliceSegmentMode;
  UInt        m_sliceSegmentArgument;
  UInt        m_sliceSegmentCurStartCUAddr;
  UInt        m_sliceSegmentCurEndCUAddr;
  Bool        m_nextSlice;
  Bool        m_nextSliceSegment;
  UInt        m_sliceBits;
  UInt        m_sliceSegmentBits;
  Bool        m_bFinalized;

  wpScalingParam  m_weightPredTable[2][MAX_NUM_REF][3]; // [REF_PIC_LIST_0 or REF_PIC_LIST_1][refIdx][0:Y, 1:U, 2:V]
  wpACDCParam    m_weightACDCParam[3];                 // [0:Y, 1:U, 2:V]

  std::vector<UInt> m_tileByteLocation;
  UInt        m_uiTileOffstForMultES;

  UInt*       m_puiSubstreamSizes;
  TComScalingList*     m_scalingList;                 //!< pointer of quantization matrix
  Bool        m_cabacInitFlag; 

  Bool       m_bLMvdL1Zero;
  Int         m_numEntryPointOffsets;
  Bool       m_temporalLayerNonReferenceFlag;
  Bool       m_LFCrossSliceBoundaryFlag;

  Bool       m_enableTMVPFlag;
#if R0226_SLICE_TMVP
  Bool       m_availableForTMVPRefFlag;
#endif

#if SVC_EXTENSION
  UInt        m_layerId;
  TComPic*    m_pcBaseColPic[MAX_LAYERS];
  TComPicYuv* m_pcFullPelBaseRec[MAX_LAYERS];
  Int         m_numMotionPredRefLayers;
#if REF_IDX_MFM
  Bool        m_bMFMEnabledFlag;
  Int         m_colRefLayerIdx;
  Bool        m_altColIndicationFlag;
  TComPic*    m_pcIlpPic;
#endif

  Bool        m_interLayerPredEnabledFlag;
  Int         m_activeNumILRRefIdx;        //< Active inter-layer reference pictures
  Int         m_interLayerPredLayerIdc  [MAX_VPS_LAYER_ID_PLUS1];
#if P0312_VERT_PHASE_ADJ
  Bool        m_vertPhasePositionFlag[MAX_VPS_LAYER_ID_PLUS1];
#endif
#if POC_RESET_FLAG
  Bool        m_bPocResetFlag;
  Int         m_pocValueBeforeReset;
#endif  
  Bool        m_bDiscardableFlag;
#if O0149_CROSS_LAYER_BLA_FLAG
  Bool        m_bCrossLayerBLAFlag;
#endif
#if POC_RESET_IDC_SIGNALLING
  Int         m_pocResetIdc;
  Int         m_pocResetPeriodId;
  Bool        m_fullPocResetFlag;
  Int         m_pocLsbVal;
  Int         m_pocMsbVal;
  Bool        m_pocMsbValRequiredFlag;
  Bool        m_pocMsbValPresentFlag;
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  Bool        m_pocMsbValNeeded;
  Int         m_pocResetDeltaPoc;
#endif
#endif
#if POC_RESET_IDC_ENCODER
  Int         m_pocValueBeforeReset;
#endif
#if POC_RESET_IDC_DECODER || POC_RESET_IDC_ENCODER
  Int         m_picOrderCntLsb;
#endif
#if Q0048_CGS_3D_ASYMLUT
  Int        m_nCGSOverWritePPS;  // for optimization, not output to bitstream
#endif
#endif //SVC_EXTENSION

public:
  TComSlice();
  virtual ~TComSlice(); 
#if SVC_EXTENSION
  Void      initSlice       ( UInt layerId );
#else
  Void      initSlice       ();
#endif

  Void      setVPS          ( TComVPS* pcVPS ) { m_pcVPS = pcVPS; }
  TComVPS*  getVPS          () { return m_pcVPS; }
  Void      setSPS          ( TComSPS* pcSPS ) { m_pcSPS = pcSPS; }
  TComSPS*  getSPS          () { return m_pcSPS; }
  
  Void      setPPS          ( TComPPS* pcPPS )         { assert(pcPPS!=NULL); m_pcPPS = pcPPS; m_iPPSId = pcPPS->getPPSId(); }
  TComPPS*  getPPS          () { return m_pcPPS; }

#if ADAPTIVE_QP_SELECTION
  Void          setTrQuant          ( TComTrQuant* pcTrQuant ) { m_pcTrQuant = pcTrQuant; }
  TComTrQuant*  getTrQuant          () { return m_pcTrQuant; }
#endif

  Void      setPPSId        ( Int PPSId )         { m_iPPSId = PPSId; }
  Int       getPPSId        () { return m_iPPSId; }
  Void      setPicOutputFlag( Bool b )         { m_PicOutputFlag = b;    }
  Bool      getPicOutputFlag()                 { return m_PicOutputFlag; }
  Void      setSaoEnabledFlag(Bool s) {m_saoEnabledFlag =s; }
  Bool      getSaoEnabledFlag() { return m_saoEnabledFlag; }
  Void      setSaoEnabledFlagChroma(Bool s) {m_saoEnabledFlagChroma =s; }       //!< set SAO Cb&Cr enabled flag
  Bool      getSaoEnabledFlagChroma() { return m_saoEnabledFlagChroma; }        //!< get SAO Cb&Cr enabled flag
  Void      setRPS          ( TComReferencePictureSet *pcRPS ) { m_pcRPS = pcRPS; }
  TComReferencePictureSet*  getRPS          () { return m_pcRPS; }
  TComReferencePictureSet*  getLocalRPS     () { return &m_LocalRPS; }

  Void      setRPSidx          ( Int iBDidx ) { m_iBDidx = iBDidx; }
  Int       getRPSidx          () { return m_iBDidx; }
  Int       getPrevTid0POC      ()                        { return  m_prevTid0POC;       }
#if PREVTID0_POC_RESET
    Void     adjustPrevTid0POC      (Int adj)             { m_prevTid0POC=m_prevTid0POC-adj; }
#endif
#if POC_RESET_IDC_DECODER
  Void      setPrevTid0POC( Int x ) { m_prevTid0POC = x; }
#endif

  TComRefPicListModification* getRefPicListModification() { return &m_RefPicListModification; }
  Void      setLastIDR(Int iIDRPOC)                       { m_iLastIDR = iIDRPOC; }
  Int       getLastIDR()                                  { return m_iLastIDR; }
  Void      setAssociatedIRAPPOC(Int iAssociatedIRAPPOC)             { m_iAssociatedIRAP = iAssociatedIRAPPOC; }
  Int       getAssociatedIRAPPOC()                        { return m_iAssociatedIRAP; }
#if POC_RESET_IDC_ENCODER
  Void setAssociatedIrapPocBeforeReset(Int x) { m_associatedIrapPocBeforeReset = x; }
  Int  getAssociatedIrapPocBeforeReset(     ) { return m_associatedIrapPocBeforeReset; }
#endif

  Void      setAssociatedIRAPType(NalUnitType associatedIRAPType)    { m_iAssociatedIRAPType = associatedIRAPType; }
  NalUnitType getAssociatedIRAPType()                        { return m_iAssociatedIRAPType; }
  SliceType getSliceType    ()                          { return  m_eSliceType;         }
  Int       getPOC          ()                          { return  m_iPOC;           }
  Int       getSliceQp      ()                          { return  m_iSliceQp;           }
  Bool      getDependentSliceSegmentFlag() const        { return m_dependentSliceSegmentFlag; }
  void      setDependentSliceSegmentFlag(Bool val)      { m_dependentSliceSegmentFlag = val; }
#if ADAPTIVE_QP_SELECTION
  Int       getSliceQpBase  ()                          { return  m_iSliceQpBase;       }
#endif
  Int       getSliceQpDelta ()                          { return  m_iSliceQpDelta;      }
  Int       getSliceQpDeltaCb ()                          { return  m_iSliceQpDeltaCb;      }
  Int       getSliceQpDeltaCr ()                          { return  m_iSliceQpDeltaCr;      }
  Bool      getDeblockingFilterDisable()                { return  m_deblockingFilterDisable; }
  Bool      getDeblockingFilterOverrideFlag()           { return  m_deblockingFilterOverrideFlag; }
  Int       getDeblockingFilterBetaOffsetDiv2()         { return  m_deblockingFilterBetaOffsetDiv2; }
  Int       getDeblockingFilterTcOffsetDiv2()           { return  m_deblockingFilterTcOffsetDiv2; }

  Int       getNumRefIdx        ( RefPicList e )                { return  m_aiNumRefIdx[e];             }
  TComPic*  getPic              ()                              { return  m_pcPic;                      }
  TComPic*  getRefPic           ( RefPicList e, Int iRefIdx)    { return  m_apcRefPicList[e][iRefIdx];  }
  Int       getRefPOC           ( RefPicList e, Int iRefIdx)    { return  m_aiRefPOCList[e][iRefIdx];   }
  Int       getDepth            ()                              { return  m_iDepth;                     }
  UInt      getColFromL0Flag    ()                              { return  m_colFromL0Flag;              }
  UInt      getColRefIdx        ()                              { return  m_colRefIdx;                  }
  Void      checkColRefIdx      (UInt curSliceIdx, TComPic* pic);
  Bool      getIsUsedAsLongTerm (Int i, Int j)                  { return m_bIsUsedAsLongTerm[i][j]; }
  Void      setIsUsedAsLongTerm (Int i, Int j, Bool value)      { m_bIsUsedAsLongTerm[i][j] = value; }
  Bool      getCheckLDC     ()                                  { return m_bCheckLDC; }
  Bool      getMvdL1ZeroFlag ()                                  { return m_bLMvdL1Zero;    }
  Int       getNumRpsCurrTempList();
  Int       getList1IdxToList0Idx ( Int list1Idx )               { return m_list1IdxToList0Idx[list1Idx]; }
  Void      setReferenced(Bool b)                               { m_bRefenced = b; }
  Bool      isReferenced()                                      { return m_bRefenced; }
  Bool      isReferenceNalu()                                   { return ((getNalUnitType() <= NAL_UNIT_RESERVED_VCL_R15) && (getNalUnitType()%2 != 0)) || ((getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP) && (getNalUnitType() <= NAL_UNIT_RESERVED_IRAP_VCL23) ); }
  Void      setPOC              ( Int i )                       { m_iPOC              = i; if ((getTLayer()==0) && (isReferenceNalu() && (getNalUnitType()!=NAL_UNIT_CODED_SLICE_RASL_R)&& (getNalUnitType()!=NAL_UNIT_CODED_SLICE_RADL_R))) {m_prevTid0POC=i;} }
  Void      setNalUnitType      ( NalUnitType e )               { m_eNalUnitType      = e;      }
  NalUnitType getNalUnitType    () const                        { return m_eNalUnitType;        }
  Bool      getRapPicFlag       ();  
  Bool      getIdrPicFlag       ()                              { return getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP; }
  Bool      isIRAP              () const                        { return (getNalUnitType() >= 16) && (getNalUnitType() <= 23); }  
  Void      checkCRA(TComReferencePictureSet *pReferencePictureSet, Int& pocCRA, NalUnitType& associatedIRAPType, TComList<TComPic *>& rcListPic);
#if NO_CLRAS_OUTPUT_FLAG
  Void      decodingRefreshMarking( TComList<TComPic*>& rcListPic, Bool noClrasOutputFlag );
  Void      decodingRefreshMarking(Int& pocCRA, Bool& bRefreshPending, TComList<TComPic*>& rcListPic, Bool noClrasOutputFlag);
#else
  Void      decodingRefreshMarking(Int& pocCRA, Bool& bRefreshPending, TComList<TComPic*>& rcListPic);
#endif
  Void      setSliceType        ( SliceType e )                 { m_eSliceType        = e;      }
  Void      setSliceQp          ( Int i )                       { m_iSliceQp          = i;      }
#if ADAPTIVE_QP_SELECTION
  Void      setSliceQpBase      ( Int i )                       { m_iSliceQpBase      = i;      }
#endif
  Void      setSliceQpDelta     ( Int i )                       { m_iSliceQpDelta     = i;      }
  Void      setSliceQpDeltaCb   ( Int i )                       { m_iSliceQpDeltaCb   = i;      }
  Void      setSliceQpDeltaCr   ( Int i )                       { m_iSliceQpDeltaCr   = i;      }
  Void      setDeblockingFilterDisable( Bool b )                { m_deblockingFilterDisable= b;      }
  Void      setDeblockingFilterOverrideFlag( Bool b )           { m_deblockingFilterOverrideFlag = b; }
  Void      setDeblockingFilterBetaOffsetDiv2( Int i )          { m_deblockingFilterBetaOffsetDiv2 = i; }
  Void      setDeblockingFilterTcOffsetDiv2( Int i )            { m_deblockingFilterTcOffsetDiv2 = i; }
  
  Void      setRefPic           ( TComPic* p, RefPicList e, Int iRefIdx ) { m_apcRefPicList[e][iRefIdx] = p; }
  Void      setRefPOC           ( Int i, RefPicList e, Int iRefIdx ) { m_aiRefPOCList[e][iRefIdx] = i; }
  Void      setNumRefIdx        ( RefPicList e, Int i )         { m_aiNumRefIdx[e]    = i;      }
  Void      setPic              ( TComPic* p )                  { m_pcPic             = p;      }
  Void      setDepth            ( Int iDepth )                  { m_iDepth            = iDepth; }

#if SVC_EXTENSION
  Void      setRefPicList       ( TComList<TComPic*>& rcListPic, Bool checkNumPocTotalCurr = false, TComPic** ilpPic = NULL );
#if Q0048_CGS_3D_ASYMLUT
  Int       getCGSOverWritePPS()              { return m_nCGSOverWritePPS;    }
  Void      setCGSOverWritePPS(Int n)         { m_nCGSOverWritePPS = n;       }
#endif
#else
  Void      setRefPicList       ( TComList<TComPic*>& rcListPic, Bool checkNumPocTotalCurr = false );
#endif
  Void      setRefPOCList       ();
  Void      setColFromL0Flag    ( UInt colFromL0 ) { m_colFromL0Flag = colFromL0; }
  Void      setColRefIdx        ( UInt refIdx) { m_colRefIdx = refIdx; }
  Void      setCheckLDC         ( Bool b )                      { m_bCheckLDC = b; }
  Void      setMvdL1ZeroFlag     ( Bool b)                       { m_bLMvdL1Zero = b; }

  Bool      isIntra         ()                          { return  m_eSliceType == I_SLICE;  }
  Bool      isInterB        ()                          { return  m_eSliceType == B_SLICE;  }
  Bool      isInterP        ()                          { return  m_eSliceType == P_SLICE;  }
  
  Void      setLambdas ( const Double lambdas[3] ) { for (Int component = 0; component < 3; component++) m_lambdas[component] = lambdas[component]; }
  const Double* getLambdas() const { return m_lambdas; }
  
  Void      initEqualRef();
  Bool      isEqualRef  ( RefPicList e, Int iRefIdx1, Int iRefIdx2 )
  {
    if (iRefIdx1 < 0 || iRefIdx2 < 0) return false;
    return m_abEqualRef[e][iRefIdx1][iRefIdx2];
  }
  
  Void setEqualRef( RefPicList e, Int iRefIdx1, Int iRefIdx2, Bool b)
  {
    m_abEqualRef[e][iRefIdx1][iRefIdx2] = m_abEqualRef[e][iRefIdx2][iRefIdx1] = b;
  }
  
  static Void      sortPicList         ( TComList<TComPic*>& rcListPic );
  Void setList1IdxToList0Idx();

  UInt getTLayer             ()                            { return m_uiTLayer;                      }
  Void setTLayer             ( UInt uiTLayer )             { m_uiTLayer = uiTLayer;                  }

  Void setTLayerInfo( UInt uiTLayer );
  Void decodingMarking( TComList<TComPic*>& rcListPic, Int iGOPSIze, Int& iMaxRefPicNum ); 
#if POC_RESET_IDC_ENCODER
  Void checkLeadingPictureRestrictions(TComList<TComPic*>& rcListPic, Bool usePocBeforeReset = false);
#else
  Void checkLeadingPictureRestrictions( TComList<TComPic*>& rcListPic );
#endif
  Void applyReferencePictureSet( TComList<TComPic*>& rcListPic, TComReferencePictureSet *RPSList);
  Bool isTemporalLayerSwitchingPoint( TComList<TComPic*>& rcListPic );
  Bool isStepwiseTemporalLayerSwitchingPointCandidate( TComList<TComPic*>& rcListPic );
#if ALLOW_RECOVERY_POINT_AS_RAP
  Int  checkThatAllRefPicsAreAvailable( TComList<TComPic*>& rcListPic, TComReferencePictureSet *pReferencePictureSet, Bool printErrors, Int pocRandomAccess = 0, Bool bUseRecoveryPoint = false);
  Void createExplicitReferencePictureSetFromReference( TComList<TComPic*>& rcListPic, TComReferencePictureSet *pReferencePictureSet, Bool isRAP, Int pocRandomAccess = 0, Bool bUseRecoveryPoint = false);
#else
  Int  checkThatAllRefPicsAreAvailable( TComList<TComPic*>& rcListPic, TComReferencePictureSet *pReferencePictureSet, Bool printErrors, Int pocRandomAccess = 0);
  Void createExplicitReferencePictureSetFromReference( TComList<TComPic*>& rcListPic, TComReferencePictureSet *pReferencePictureSet, Bool isRAP);
#endif

  Void setMaxNumMergeCand               (UInt val )         { m_maxNumMergeCand = val;                    }
  UInt getMaxNumMergeCand               ()                  { return m_maxNumMergeCand;                   }

#if SETTING_NO_OUT_PIC_PRIOR
  Void setNoOutputPriorPicsFlag         ( Bool val )        { m_noOutputPriorPicsFlag = val;                    }
  Bool getNoOutputPriorPicsFlag         ()                  { return m_noOutputPriorPicsFlag;                   }

  Void setNoRaslOutputFlag              ( Bool val )        { m_noRaslOutputFlag = val;                    }
  Bool getNoRaslOutputFlag              ()                  { return m_noRaslOutputFlag;                   }

  Void setHandleCraAsBlaFlag            ( Bool val )        { m_handleCraAsBlaFlag = val;                    }
  Bool getHandleCraAsBlaFlag            ()                  { return m_handleCraAsBlaFlag;                   }
#endif

  Void setSliceMode                     ( UInt uiMode )     { m_sliceMode = uiMode;                     }
  UInt getSliceMode                     ()                  { return m_sliceMode;                       }
  Void setSliceArgument                 ( UInt uiArgument ) { m_sliceArgument = uiArgument;             }
  UInt getSliceArgument                 ()                  { return m_sliceArgument;                   }
  Void setSliceCurStartCUAddr           ( UInt uiAddr )     { m_sliceCurStartCUAddr = uiAddr;           }
  UInt getSliceCurStartCUAddr           ()                  { return m_sliceCurStartCUAddr;             }
  Void setSliceCurEndCUAddr             ( UInt uiAddr )     { m_sliceCurEndCUAddr = uiAddr;             }
  UInt getSliceCurEndCUAddr             ()                  { return m_sliceCurEndCUAddr;               }
  Void setSliceIdx                      ( UInt i)           { m_sliceIdx = i;                           }
  UInt getSliceIdx                      ()                  { return  m_sliceIdx;                       }
  Void copySliceInfo                    (TComSlice *pcSliceSrc);
  Void setSliceSegmentMode              ( UInt uiMode )     { m_sliceSegmentMode = uiMode;              }
  UInt getSliceSegmentMode              ()                  { return m_sliceSegmentMode;                }
  Void setSliceSegmentArgument          ( UInt uiArgument ) { m_sliceSegmentArgument = uiArgument;      }
  UInt getSliceSegmentArgument          ()                  { return m_sliceSegmentArgument;            }
  Void setSliceSegmentCurStartCUAddr    ( UInt uiAddr )     { m_sliceSegmentCurStartCUAddr = uiAddr;    }
  UInt getSliceSegmentCurStartCUAddr    ()                  { return m_sliceSegmentCurStartCUAddr;      }
  Void setSliceSegmentCurEndCUAddr      ( UInt uiAddr )     { m_sliceSegmentCurEndCUAddr = uiAddr;      }
  UInt getSliceSegmentCurEndCUAddr      ()                  { return m_sliceSegmentCurEndCUAddr;        }
  Void setNextSlice                     ( Bool b )          { m_nextSlice = b;                           }
  Bool isNextSlice                      ()                  { return m_nextSlice;                        }
  Void setNextSliceSegment              ( Bool b )          { m_nextSliceSegment = b;                    }
  Bool isNextSliceSegment               ()                  { return m_nextSliceSegment;                 }
  Void setSliceBits                     ( UInt uiVal )      { m_sliceBits = uiVal;                      }
  UInt getSliceBits                     ()                  { return m_sliceBits;                       }  
  Void setSliceSegmentBits              ( UInt uiVal )      { m_sliceSegmentBits = uiVal;            }
  UInt getSliceSegmentBits              ()                  { return m_sliceSegmentBits;             }
  Void setFinalized                     ( Bool uiVal )      { m_bFinalized = uiVal;                       }
  Bool getFinalized                     ()                  { return m_bFinalized;                        }
  Void  setWpScaling    ( wpScalingParam  wp[2][MAX_NUM_REF][3] ) { memcpy(m_weightPredTable, wp, sizeof(wpScalingParam)*2*MAX_NUM_REF*3); }
  Void  getWpScaling    ( RefPicList e, Int iRefIdx, wpScalingParam *&wp);

  Void  resetWpScaling  ();
  Void  initWpScaling   ();
  inline Bool applyWP   () { return( (m_eSliceType==P_SLICE && m_pcPPS->getUseWP()) || (m_eSliceType==B_SLICE && m_pcPPS->getWPBiPred()) ); }

  Void  setWpAcDcParam  ( wpACDCParam wp[3] ) { memcpy(m_weightACDCParam, wp, sizeof(wpACDCParam)*3); }
  Void  getWpAcDcParam  ( wpACDCParam *&wp );
  Void  initWpAcDcParam ();
  
  Void setTileLocationCount             ( UInt cnt )               { return m_tileByteLocation.resize(cnt);    }
  UInt getTileLocationCount             ()                         { return (UInt) m_tileByteLocation.size();  }
  Void setTileLocation                  ( Int idx, UInt location ) { assert (idx<(Int)m_tileByteLocation.size());
                                                                     m_tileByteLocation[idx] = location;       }
  Void addTileLocation                  ( UInt location )          { m_tileByteLocation.push_back(location);   }
  UInt getTileLocation                  ( Int idx )                { return m_tileByteLocation[idx];           }

  Void setTileOffstForMultES            (UInt uiOffset )      { m_uiTileOffstForMultES = uiOffset;        }
  UInt getTileOffstForMultES            ()                    { return m_uiTileOffstForMultES;            }
  Void allocSubstreamSizes              ( UInt uiNumSubstreams );
  UInt* getSubstreamSizes               ()                  { return m_puiSubstreamSizes; }
  Void  setScalingList              ( TComScalingList* scalingList ) { m_scalingList = scalingList; }
  TComScalingList*   getScalingList ()                               { return m_scalingList; }
  Void  setDefaultScalingList       ();
  Bool  checkDefaultScalingList     ();
  Void      setCabacInitFlag  ( Bool val ) { m_cabacInitFlag = val;      }  //!< set CABAC initial flag 
  Bool      getCabacInitFlag  ()           { return m_cabacInitFlag;     }  //!< get CABAC initial flag 
  Void      setNumEntryPointOffsets(Int val)  { m_numEntryPointOffsets = val;     }
  Int       getNumEntryPointOffsets()         { return m_numEntryPointOffsets;    }
  Bool      getTemporalLayerNonReferenceFlag()       { return m_temporalLayerNonReferenceFlag;}
  Void      setTemporalLayerNonReferenceFlag(Bool x) { m_temporalLayerNonReferenceFlag = x;}
  Void      setLFCrossSliceBoundaryFlag     ( Bool   val )    { m_LFCrossSliceBoundaryFlag = val; }
  Bool      getLFCrossSliceBoundaryFlag     ()                { return m_LFCrossSliceBoundaryFlag;} 

  Void      setEnableTMVPFlag     ( Bool   b )    { m_enableTMVPFlag = b; }
  Bool      getEnableTMVPFlag     ()              { return m_enableTMVPFlag;}

#if R0226_SLICE_TMVP
  Void      setAvailableForTMVPRefFlag     ( Bool   b )    { m_availableForTMVPRefFlag = b; }
  Bool      getAvailableForTMVPRefFlag     ()              { return m_availableForTMVPRefFlag;}
#endif

#if SVC_EXTENSION
  Bool      setBaseColPic       ( TComList<TComPic*>& rcListPic , UInt refLayerIdc );
  Void      setBaseColPic       (UInt refLayerIdc, TComPic* p)     { m_pcBaseColPic[refLayerIdc] = p; }
  TComPic*  getBaseColPic       (UInt refLayerIdc)                { return m_pcBaseColPic[refLayerIdc]; }
  TComPic** getBaseColPic       ()                { return &m_pcBaseColPic[0]; }
#if MFM_ENCCONSTRAINT
  TComPic*  getBaseColPic( TComList<TComPic*>& rcListPic );
#endif

  Void      setLayerId (UInt layerId)   { m_layerId = layerId; }
  UInt      getLayerId ()               { return m_layerId;    }

  Void        setFullPelBaseRec   (UInt refLayerIdc, TComPicYuv* p) { m_pcFullPelBaseRec[refLayerIdc] = p; }
  TComPicYuv* getFullPelBaseRec   (UInt refLayerIdc)               { return  m_pcFullPelBaseRec[refLayerIdc];  }

  Void      setRefPicListModificationSvc();
  Int       getNumILRRefIdx     ( )                     { return  m_pcVPS->getNumDirectRefLayers( m_layerId ); }

  Int       getActiveNumILRRefIdx     ( )               { return  m_activeNumILRRefIdx; }
  Void      setActiveNumILRRefIdx     ( Int i )         { m_activeNumILRRefIdx = i;     }  

  Int       getInterLayerPredLayerIdc (UInt layerIdx)                        { return  m_interLayerPredLayerIdc[layerIdx];}
  Void      setInterLayerPredLayerIdc (UInt refLayerIdc, UInt layerIdx)      { m_interLayerPredLayerIdc[layerIdx] = refLayerIdc;  }

  Void      setInterLayerPredEnabledFlag     ( Bool   val )    { m_interLayerPredEnabledFlag = val; }
  Bool      getInterLayerPredEnabledFlag     ()                { return m_interLayerPredEnabledFlag;}

#if P0312_VERT_PHASE_ADJ
  Int       getVertPhasePositionFlag (UInt layerIdx)              { return   m_vertPhasePositionFlag[layerIdx];}
  Void      setVertPhasePositionFlag (Bool b, UInt layerIdx)      {  m_vertPhasePositionFlag[layerIdx] = b;  }
#endif

  Void      setNumMotionPredRefLayers(int i)            { m_numMotionPredRefLayers = i; }
  Int       getNumMotionPredRefLayers()                 { return m_numMotionPredRefLayers; }
#if REF_IDX_MFM
  Void      setMFMEnabledFlag(Bool flag)                { m_bMFMEnabledFlag = flag; }
  Bool      getMFMEnabledFlag()                         { return m_bMFMEnabledFlag; }
#endif

  TComPic* getRefPic(TComList<TComPic*>& rcListPic, Int poc) { return xGetRefPic( rcListPic, poc ); } 

  Bool     isRADL()   { return (m_eNalUnitType == NAL_UNIT_CODED_SLICE_RADL_N || m_eNalUnitType == NAL_UNIT_CODED_SLICE_RADL_R); }
  Bool     isRASL()   { return (m_eNalUnitType == NAL_UNIT_CODED_SLICE_RASL_N || m_eNalUnitType == NAL_UNIT_CODED_SLICE_RASL_R); }

#if POC_RESET_RESTRICTIONS
  Bool     isIDR()    { return (m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL
                                  || m_eNalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP); }
  Bool     isCRA()    { return m_eNalUnitType == NAL_UNIT_CODED_SLICE_CRA; }
  Bool     isBLA()    { return (m_eNalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_LP
                                  || m_eNalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_RADL
                                  || m_eNalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP ); }
  Bool     isSLNR()   { return (m_eNalUnitType == NAL_UNIT_CODED_SLICE_TRAIL_N
                                  || m_eNalUnitType == NAL_UNIT_CODED_SLICE_TSA_N
                                  || m_eNalUnitType == NAL_UNIT_CODED_SLICE_STSA_N
                                  || m_eNalUnitType == NAL_UNIT_CODED_SLICE_RADL_N
                                  || m_eNalUnitType == NAL_UNIT_CODED_SLICE_RASL_N
                                  || m_eNalUnitType == NAL_UNIT_RESERVED_VCL_N10
                                  || m_eNalUnitType == NAL_UNIT_RESERVED_VCL_N12
                                  || m_eNalUnitType == NAL_UNIT_RESERVED_VCL_N14 ); }
#endif
#if POC_RESET_FLAG
  Bool      getPocResetFlag  ()                              { return m_bPocResetFlag;       }
  Void      setPocResetFlag  (Bool b)                        { m_bPocResetFlag = b;          }
  Int       getPocValueBeforeReset ()                        { return m_pocValueBeforeReset; }
  Void      setPocValueBeforeReset (Int x)                   { m_pocValueBeforeReset = x ;   }
#endif
  Bool      getDiscardableFlag  ()                           { return m_bDiscardableFlag;    }
  Void      setDiscardableFlag  (Bool b)                     { m_bDiscardableFlag = b;       }
#if O0149_CROSS_LAYER_BLA_FLAG
  Bool      getCrossLayerBLAFlag  ()                         { return m_bCrossLayerBLAFlag;  }
  Void      setCrossLayerBLAFlag  (Bool b)                   { m_bCrossLayerBLAFlag = b;     }
#endif

#if RPL_INIT_N0316_N0082
  Int       getNumNegativeRpsCurrTempList();
#endif

#if REPN_FORMAT_IN_VPS
  UInt getPicWidthInLumaSamples();
  UInt getPicHeightInLumaSamples();
#if AUXILIARY_PICTURES
  ChromaFormat getChromaFormatIdc();
#else
  UInt getChromaFormatIdc();
#endif
  UInt getBitDepthY();
  UInt getBitDepthC();
  Int  getQpBDOffsetY();
  Int  getQpBDOffsetC();

#if R0156_CONF_WINDOW_IN_REP_FORMAT
  Window& getConformanceWindow();
#endif
#endif

  Void setILRPic(TComPic **pcIlpPic);
#if POC_RESET_IDC_SIGNALLING
  Int       getPocResetIdc       ()                              { return m_pocResetIdc;       }
  Void      setPocResetIdc       (Int b)                         { m_pocResetIdc = b;          }
  Int       getPocResetPeriodId  ()                              { return m_pocResetPeriodId;       }
  Void      setPocResetPeriodId  (Int b)                         { m_pocResetPeriodId = b;          }
  Bool      getFullPocResetFlag  ()                              { return m_fullPocResetFlag;       }
  Void      setFullPocResetFlag  (Bool b)                        { m_fullPocResetFlag = b;          }
  Int       getPocLsbVal         ()                              { return m_pocLsbVal;       }
  Void      setPocLsbVal       (Int b)                           { m_pocLsbVal = b;          }
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  Void      setPocMsbNeeded      (Bool x)                        { m_pocMsbValNeeded = x; }
  Bool      getPocMsbNeeded      ()                              { return m_pocMsbValNeeded; }
  Int       getPocResetDeltaPoc  ()                              { return m_pocResetDeltaPoc; }
  Void      setPocResetDeltaPoc  (Int x)                         { m_pocResetDeltaPoc = x; }
#endif
  Int       getPocMsbVal         ()                              { return m_pocMsbVal;       }
  Void      setPocMsbVal       (Int b)                           { m_pocMsbVal = b;          }
  Bool      getPocMsbValPresentFlag ()                           { return m_pocMsbValPresentFlag; }
  Void      setPocMsbValPresentFlag (Bool x)                     { m_pocMsbValPresentFlag = x; }
  Bool      getPocMsbValRequiredFlag ()                           { return m_pocMsbValRequiredFlag; }
  Void      setPocMsbValRequiredFlag (Bool x)                     { m_pocMsbValRequiredFlag = x; }
#endif

#if NO_OUTPUT_OF_PRIOR_PICS
  Bool      getBlaPicFlag       ();
  Bool      getCraPicFlag       ();
#endif
#if POC_RESET_IDC_DECODER
  Bool      getRaslPicFlag      ();
  Bool      getRadlPicFlag      ();
#endif
#if POC_RESET_IDC_DECODER || POC_RESET_IDC_ENCODER
  Int       getPicOrderCntLsb() { return m_picOrderCntLsb; }
  Void      setPicOrderCntLsb(Int x) { m_picOrderCntLsb = x; }
#endif

#if POC_RESET_IDC_ENCODER
  Int       getPocValueBeforeReset ()                        { return m_pocValueBeforeReset; }
  Void      setPocValueBeforeReset (Int x)                   { m_pocValueBeforeReset = x ;   }
  Void      decrementRefPocValues(Int const decrementValue);
  Int       getCurrMsb( Int currLsb, Int prevLsb, Int prevMsb, Int maxLsbVal );
#endif
  Int       getReferenceLayerIdc( UInt refLayerId );

#endif //SVC_EXTENSION
protected:
  TComPic*  xGetRefPic  (TComList<TComPic*>& rcListPic,
                         Int                 poc);
  TComPic*  xGetLongTermRefPic(TComList<TComPic*>& rcListPic, Int poc, Bool pocHasMsb);
};// END CLASS DEFINITION TComSlice

// JR */

template <class T> class ParameterSetMap
{
public:
  ParameterSetMap(Int maxId)
  :m_maxId (maxId)
  {}

  ~ParameterSetMap()
  {
    for (typename std::map<Int,T *>::iterator i = m_paramsetMap.begin(); i!= m_paramsetMap.end(); i++)
    {
      delete (*i).second;
    }
  }

  Void storePS(Int psId, T *ps)
  {
    assert ( psId < m_maxId );
    if ( m_paramsetMap.find(psId) != m_paramsetMap.end() )
    {
      delete m_paramsetMap[psId];
    }
    m_paramsetMap[psId] = ps; 
  }

  Void mergePSList(ParameterSetMap<T> &rPsList)
  {
    for (typename std::map<Int,T *>::iterator i = rPsList.m_paramsetMap.begin(); i!= rPsList.m_paramsetMap.end(); i++)
    {
      storePS(i->first, i->second);
    }
    rPsList.m_paramsetMap.clear();
  }


  T* getPS(Int psId)
  {
    return ( m_paramsetMap.find(psId) == m_paramsetMap.end() ) ? NULL : m_paramsetMap[psId];
  }

  T* getFirstPS()
  {
    return (m_paramsetMap.begin() == m_paramsetMap.end() ) ? NULL : m_paramsetMap.begin()->second;
  }

private:
  std::map<Int,T *> m_paramsetMap;
  Int               m_maxId;
};

class ParameterSetManager
{
public:
  ParameterSetManager();
  virtual ~ParameterSetManager();

  //! store sequence parameter set and take ownership of it 
  Void storeVPS(TComVPS *vps) { m_vpsMap.storePS( vps->getVPSId(), vps); };
  //! get pointer to existing video parameter set  
  TComVPS* getVPS(Int vpsId)  { return m_vpsMap.getPS(vpsId); };
  TComVPS* getFirstVPS()      { return m_vpsMap.getFirstPS(); };
  
  //! store sequence parameter set and take ownership of it 
  Void storeSPS(TComSPS *sps) { m_spsMap.storePS( sps->getSPSId(), sps); };
  //! get pointer to existing sequence parameter set  
  TComSPS* getSPS(Int spsId)  { return m_spsMap.getPS(spsId); };
  TComSPS* getFirstSPS()      { return m_spsMap.getFirstPS(); };

  //! store picture parameter set and take ownership of it 
  Void storePPS(TComPPS *pps) { m_ppsMap.storePS( pps->getPPSId(), pps); };
  //! get pointer to existing picture parameter set  
  TComPPS* getPPS(Int ppsId)  { return m_ppsMap.getPS(ppsId); };
  TComPPS* getFirstPPS()      { return m_ppsMap.getFirstPS(); };

  //! activate a SPS from a active parameter sets SEI message
  //! \returns true, if activation is successful
  Bool activateSPSWithSEI(Int SPSId);

  //! activate a PPS and depending on isIDR parameter also SPS and VPS
  //! \returns true, if activation is successful
  Bool activatePPS(Int ppsId, Bool isIRAP);

  TComVPS* getActiveVPS(){ return m_vpsMap.getPS(m_activeVPSId); };
  TComSPS* getActiveSPS(){ return m_spsMap.getPS(m_activeSPSId); };
  TComPPS* getActivePPS(){ return m_ppsMap.getPS(m_activePPSId); };

protected:

#if SVC_EXTENSION
  static ParameterSetMap<TComVPS> m_vpsMap;
  static ParameterSetMap<TComSPS> m_spsMap; 
  static ParameterSetMap<TComPPS> m_ppsMap;
#else
  ParameterSetMap<TComVPS> m_vpsMap;
  ParameterSetMap<TComSPS> m_spsMap; 
  ParameterSetMap<TComPPS> m_ppsMap;
#endif

#if SVC_EXTENSION
  static Int m_activeVPSId;
#else
  Int m_activeVPSId;
#endif
  Int m_activeSPSId;
  Int m_activePPSId;
};


//! \}

#endif // __TCOMSLICE__

