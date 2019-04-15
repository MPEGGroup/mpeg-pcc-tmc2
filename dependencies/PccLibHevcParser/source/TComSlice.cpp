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

//  \file     TComSlice.cpp
//    \brief    slice header and SPS class

#include <numeric>
#include "CommonDef.h"
#include "TComSlice.h"
//TCH_JR #include "TComPic.h"
//TCH_JR #include "TLibEncoder/TEncSbac.h"
//TCH_JR #include "TLibDecoder/TDecSbac.h"

//! \ingroup TLibCommon
//! \{

#if SVC_EXTENSION
ParameterSetMap<TComVPS> ParameterSetManager::m_vpsMap(MAX_NUM_VPS);
ParameterSetMap<TComSPS> ParameterSetManager::m_spsMap(MAX_NUM_SPS);
ParameterSetMap<TComPPS> ParameterSetManager::m_ppsMap(MAX_NUM_PPS);
Int ParameterSetManager::m_activeVPSId = -1;
#endif

TComSlice::TComSlice()
: m_iPPSId                        ( -1 )
, m_iPOC                          ( 0 )
, m_iLastIDR                      ( 0 )
, m_eNalUnitType                  ( NAL_UNIT_CODED_SLICE_IDR_W_RADL )
, m_eSliceType                    ( I_SLICE )
, m_iSliceQp                      ( 0 )
, m_dependentSliceSegmentFlag            ( false )
#if ADAPTIVE_QP_SELECTION
, m_iSliceQpBase                  ( 0 )
#endif
, m_deblockingFilterDisable        ( false )
, m_deblockingFilterOverrideFlag   ( false )
, m_deblockingFilterBetaOffsetDiv2 ( 0 )
, m_deblockingFilterTcOffsetDiv2   ( 0 )
, m_bCheckLDC                     ( false )
, m_iSliceQpDelta                 ( 0 )
, m_iSliceQpDeltaCb               ( 0 )
, m_iSliceQpDeltaCr               ( 0 )
, m_iDepth                        ( 0 )
, m_bRefenced                     ( false )
, m_pcSPS                         ( NULL )
, m_pcPPS                         ( NULL )
, m_pcPic                         ( NULL )
, m_colFromL0Flag                 ( 1 )
#if SETTING_NO_OUT_PIC_PRIOR
, m_noOutputPriorPicsFlag         ( false )
, m_noRaslOutputFlag              ( false )
, m_handleCraAsBlaFlag            ( false )
#endif
, m_colRefIdx                     ( 0 )
, m_uiTLayer                      ( 0 )
, m_bTLayerSwitchingFlag          ( false )
, m_sliceMode                   ( 0 )
, m_sliceArgument               ( 0 )
, m_sliceCurStartCUAddr         ( 0 )
, m_sliceCurEndCUAddr           ( 0 )
, m_sliceIdx                    ( 0 )
, m_sliceSegmentMode            ( 0 )
, m_sliceSegmentArgument        ( 0 )
, m_sliceSegmentCurStartCUAddr  ( 0 )
, m_sliceSegmentCurEndCUAddr    ( 0 )
, m_nextSlice                    ( false )
, m_nextSliceSegment             ( false )
, m_sliceBits                   ( 0 )
, m_sliceSegmentBits         ( 0 )
, m_bFinalized                    ( false )
, m_uiTileOffstForMultES          ( 0 )
, m_puiSubstreamSizes             ( NULL )
, m_cabacInitFlag                 ( false )
, m_bLMvdL1Zero                   ( false )
, m_numEntryPointOffsets          ( 0 )
, m_temporalLayerNonReferenceFlag ( false )
, m_enableTMVPFlag                ( true )
#if R0226_SLICE_TMVP
, m_availableForTMVPRefFlag       ( true )
#endif
#if SVC_EXTENSION
, m_layerId                     ( 0 )
#if REF_IDX_MFM
, m_bMFMEnabledFlag               ( false )
#endif
#if POC_RESET_FLAG
, m_bPocResetFlag                 ( false )
#endif
, m_bDiscardableFlag              ( false )
#if O0149_CROSS_LAYER_BLA_FLAG
, m_bCrossLayerBLAFlag            ( false )
#endif
#if POC_RESET_IDC_SIGNALLING
, m_pocResetIdc                   ( 0 )
, m_pocResetPeriodId              ( 0 )
, m_fullPocResetFlag              ( false )
, m_pocLsbVal                     ( 0 )
, m_pocMsbVal                     ( 0 )
, m_pocMsbValRequiredFlag         ( false )
, m_pocMsbValPresentFlag          ( false )
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
, m_pocMsbValNeeded               ( false )
#endif
#endif
#if POC_RESET_IDC_DECODER || POC_RESET_IDC_ENCODER
, m_picOrderCntLsb (0)
#endif
#endif //SVC_EXTENSION
{
  m_aiNumRefIdx[0] = m_aiNumRefIdx[1] = 0;

#if SVC_EXTENSION
  memset( m_pcBaseColPic, 0, sizeof( m_pcBaseColPic ) );
  m_activeNumILRRefIdx        = 0; 
  m_interLayerPredEnabledFlag = 0;
  ::memset( m_interLayerPredLayerIdc, 0, sizeof(m_interLayerPredLayerIdc) );
#if P0312_VERT_PHASE_ADJ
  ::memset( m_vertPhasePositionFlag, 0, sizeof(m_vertPhasePositionFlag) );
#endif
#endif //SVC_EXTENSION

  initEqualRef();
  
  for (Int component = 0; component < 3; component++)
  {
    m_lambdas[component] = 0.0;
  }
  
  for ( Int idx = 0; idx < MAX_NUM_REF; idx++ )
  {
    m_list1IdxToList0Idx[idx] = -1;
  }
  for(Int iNumCount = 0; iNumCount < MAX_NUM_REF; iNumCount++)
  {
    m_apcRefPicList [0][iNumCount] = NULL;
    m_apcRefPicList [1][iNumCount] = NULL;
    m_aiRefPOCList  [0][iNumCount] = 0;
    m_aiRefPOCList  [1][iNumCount] = 0;
  }
  resetWpScaling();
  initWpAcDcParam();
  m_saoEnabledFlag = false;
  m_saoEnabledFlagChroma = false;
}

TComSlice::~TComSlice()
{
  delete[] m_puiSubstreamSizes;
  m_puiSubstreamSizes = NULL;
}


#if SVC_EXTENSION
Void TComSlice::initSlice( UInt layerId )
#else
Void TComSlice::initSlice()
#endif
{
#if SVC_EXTENSION
  m_layerId = layerId;
  m_activeNumILRRefIdx        = 0;
  m_interLayerPredEnabledFlag = 0;
#endif
  m_aiNumRefIdx[0]      = 0;
  m_aiNumRefIdx[1]      = 0;
  
  m_colFromL0Flag = 1;
  
  m_colRefIdx = 0;
  initEqualRef();
  m_bCheckLDC = false;
  m_iSliceQpDeltaCb = 0;
  m_iSliceQpDeltaCr = 0;

  m_maxNumMergeCand = MRG_MAX_NUM_CANDS;

  m_bFinalized=false;

  m_tileByteLocation.clear();
  m_cabacInitFlag        = false;
  m_numEntryPointOffsets = 0;
  m_enableTMVPFlag = true;
#if POC_RESET_IDC_SIGNALLING
  m_pocResetIdc                   = 0;
  m_pocResetPeriodId              = 0;
  m_fullPocResetFlag              = false;
  m_pocLsbVal                     = 0;
  m_pocMsbVal                     = 0;
  m_pocMsbValRequiredFlag         = false;
  m_pocMsbValPresentFlag          = false;
#endif
#if POC_RESET_IDC_DECODER || POC_RESET_IDC_ENCODER
  m_picOrderCntLsb = 0;
#endif
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  m_pocMsbValNeeded  = false;
  m_pocResetDeltaPoc = 0;
#endif
}

Bool TComSlice::getRapPicFlag()
{
  return getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA;
}
#if NO_OUTPUT_OF_PRIOR_PICS
Bool TComSlice::getBlaPicFlag       ()
{
    return  getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
    || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
    || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP;
}
Bool TComSlice::getCraPicFlag       ()
{
    return getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA;
}
#endif
#if POC_RESET_IDC_DECODER
Bool TComSlice::getRaslPicFlag      ()
{
  return  getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_R
  || getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N;
}
Bool TComSlice::getRadlPicFlag      ()
{
  return  getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_R
  || getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N;
}
#endif
#if POC_RESET_IDC_ENCODER
Void TComSlice::decrementRefPocValues(Int const decrementValue)
{
  for(Int listNum = 0; listNum < 2; listNum++)
  {
    RefPicList dpbPicSliceList = (listNum == 1) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;
    for(Int listIdx = 0; listIdx < getNumRefIdx( dpbPicSliceList ); listIdx++)
    {
      setRefPOC( getRefPOC(dpbPicSliceList, listIdx) - decrementValue, 
                  dpbPicSliceList,
                  listIdx
                  );
    }
  }
}

Int TComSlice::getCurrMsb( Int currLsb, Int prevLsb, Int prevMsb, Int maxLsbVal )
{
  if( prevLsb - currLsb >= (maxLsbVal >> 1) )
  {
    return prevMsb + maxLsbVal;
  }
  else if( currLsb - prevLsb > (maxLsbVal >> 1) )
  {
    return prevMsb - maxLsbVal;
  }
  else
  {
    return prevMsb;
  }
}
#endif

//
// - allocate table to contain substream sizes to be written to the slice header.
// .
// \param uiNumSubstreams Number of substreams -- the allocation will be this value - 1.
//
Void  TComSlice::allocSubstreamSizes(UInt uiNumSubstreams)
{
  delete[] m_puiSubstreamSizes;
  m_puiSubstreamSizes = new UInt[uiNumSubstreams > 0 ? uiNumSubstreams-1 : 0];
}

/* TCH
Void  TComSlice::sortPicList(TComList<TComPic*>& rcListPic)
{
  TComPic*    pcPicExtract;
  TComPic*    pcPicInsert;
  
  TComList<TComPic*>::iterator    iterPicExtract;
  TComList<TComPic*>::iterator    iterPicExtract_1;
  TComList<TComPic*>::iterator    iterPicInsert;
  
  for (Int i = 1; i < (Int)(rcListPic.size()); i++)
  {
    iterPicExtract = rcListPic.begin();
    for (Int j = 0; j < i; j++) iterPicExtract++;
    pcPicExtract = *(iterPicExtract);
    pcPicExtract->setCurrSliceIdx(0);
    
    iterPicInsert = rcListPic.begin();
    while (iterPicInsert != iterPicExtract)
    {
      pcPicInsert = *(iterPicInsert);
      pcPicInsert->setCurrSliceIdx(0);
      if (pcPicInsert->getPOC() >= pcPicExtract->getPOC())
      {
        break;
      }
      
      iterPicInsert++;
    }
    
    iterPicExtract_1 = iterPicExtract;    iterPicExtract_1++;
    
    //  swap iterPicExtract and iterPicInsert, iterPicExtract = curr. / iterPicInsert = insertion position
    rcListPic.insert (iterPicInsert, iterPicExtract, iterPicExtract_1);
    rcListPic.erase  (iterPicExtract);
  }
}

TComPic* TComSlice::xGetRefPic (TComList<TComPic*>& rcListPic,
                                Int                 poc)
{
  TComList<TComPic*>::iterator  iterPic = rcListPic.begin();  
  TComPic*                      pcPic = *(iterPic);
  while ( iterPic != rcListPic.end() )
  {
#if POC_RESET_IDC_ENCODER
    if( (pcPic->getPOC() == poc) && (pcPic->getSlice(0)->isReferenced()) )
#else
    if(pcPic->getPOC() == poc)
#endif
    {
      break;
    }
    iterPic++;
#if SVC_EXTENSION
    // return NULL, if picture with requested POC is not in the list, otherwise iterator goes outside of the list
    if( iterPic == rcListPic.end() )
    {
      return NULL;
    }
#endif
    pcPic = *(iterPic);
  }
#if POC_RESET_FLAG || POC_RESET_IDC_DECODER
  assert( pcPic->getSlice(0)->isReferenced() );
#endif
  return  pcPic;
}


TComPic* TComSlice::xGetLongTermRefPic(TComList<TComPic*>& rcListPic, Int poc, Bool pocHasMsb)
{
  TComList<TComPic*>::iterator  iterPic = rcListPic.begin();  
  TComPic*                      pcPic = *(iterPic);
  TComPic*                      pcStPic = pcPic;
  
  Int pocCycle = 1 << getSPS()->getBitsForPOC();
  if (!pocHasMsb)
  {
    poc = poc & (pocCycle - 1);
  }
  
  while ( iterPic != rcListPic.end() )
  {
    pcPic = *(iterPic);
    if (pcPic && pcPic->getPOC()!=this->getPOC() && pcPic->getSlice( 0 )->isReferenced())
    {
      Int picPoc = pcPic->getPOC();
      if (!pocHasMsb)
      {
        picPoc = picPoc & (pocCycle - 1);
      }
      
#if POC_RESET_RPS
      if( ((!pocHasMsb) && ((poc & (pocCycle-1)) == picPoc)) || ( pocHasMsb && (poc == picPoc)) )
#else
      if (poc == picPoc)
#endif
      {
       if (pcPic->getIsLongTerm())
      {
        return pcPic;
      }
      else
      {
        pcStPic = pcPic;
      }
      break;
    }
    }

    iterPic++;
  }
  
  return  pcStPic;
}

Void TComSlice::setRefPOCList()
{
  for (Int iDir = 0; iDir < 2; iDir++)
  {
    for (Int iNumRefIdx = 0; iNumRefIdx < m_aiNumRefIdx[iDir]; iNumRefIdx++)
    {
      m_aiRefPOCList[iDir][iNumRefIdx] = m_apcRefPicList[iDir][iNumRefIdx]->getPOC();
    }
  }

}

Void TComSlice::setList1IdxToList0Idx()
{
  Int idxL0, idxL1;
  for ( idxL1 = 0; idxL1 < getNumRefIdx( REF_PIC_LIST_1 ); idxL1++ )
  {
    m_list1IdxToList0Idx[idxL1] = -1;
    for ( idxL0 = 0; idxL0 < getNumRefIdx( REF_PIC_LIST_0 ); idxL0++ )
    {
      if ( m_apcRefPicList[REF_PIC_LIST_0][idxL0]->getPOC() == m_apcRefPicList[REF_PIC_LIST_1][idxL1]->getPOC() )
      {
        m_list1IdxToList0Idx[idxL1] = idxL0;
        break;
      }
    }
  }
}

#if SVC_EXTENSION
Void TComSlice::setRefPicList( TComList<TComPic*>& rcListPic, Bool checkNumPocTotalCurr, TComPic** ilpPic)
#else
Void TComSlice::setRefPicList( TComList<TComPic*>& rcListPic, Bool checkNumPocTotalCurr )
#endif
{
  if (!checkNumPocTotalCurr)
  {
    if (m_eSliceType == I_SLICE)
    {
      ::memset( m_apcRefPicList, 0, sizeof (m_apcRefPicList));
      ::memset( m_aiNumRefIdx,   0, sizeof ( m_aiNumRefIdx ));
      
      return;
    }
  
    m_aiNumRefIdx[0] = getNumRefIdx(REF_PIC_LIST_0);
    m_aiNumRefIdx[1] = getNumRefIdx(REF_PIC_LIST_1);
  }

  TComPic*  pcRefPic= NULL;
  TComPic*  RefPicSetStCurr0[16];
  TComPic*  RefPicSetStCurr1[16];
  TComPic*  RefPicSetLtCurr[16];
  UInt NumPocStCurr0 = 0;
  UInt NumPocStCurr1 = 0;
  UInt NumPocLtCurr = 0;
  Int i;

#if SVC_EXTENSION
  if( m_layerId == 0 || ( m_layerId > 0 && ( m_activeNumILRRefIdx == 0 || !((getNalUnitType() >= NAL_UNIT_CODED_SLICE_BLA_W_LP) && (getNalUnitType() <= NAL_UNIT_CODED_SLICE_CRA)) ) ) )
  {
#endif
  for(i=0; i < m_pcRPS->getNumberOfNegativePictures(); i++)
  {
    if(m_pcRPS->getUsed(i))
    {
      pcRefPic = xGetRefPic(rcListPic, getPOC()+m_pcRPS->getDeltaPOC(i));
      pcRefPic->setIsLongTerm(0);
      pcRefPic->getPicYuvRec()->extendPicBorder();
      RefPicSetStCurr0[NumPocStCurr0] = pcRefPic;
      NumPocStCurr0++;
      pcRefPic->setCheckLTMSBPresent(false);  
    }
  }
  
  for(; i < m_pcRPS->getNumberOfNegativePictures()+m_pcRPS->getNumberOfPositivePictures(); i++)
  {
    if(m_pcRPS->getUsed(i))
    {
      pcRefPic = xGetRefPic(rcListPic, getPOC()+m_pcRPS->getDeltaPOC(i));
      pcRefPic->setIsLongTerm(0);
      pcRefPic->getPicYuvRec()->extendPicBorder();
      RefPicSetStCurr1[NumPocStCurr1] = pcRefPic;
      NumPocStCurr1++;
      pcRefPic->setCheckLTMSBPresent(false);  
    }
  }
  
  for(i = m_pcRPS->getNumberOfNegativePictures()+m_pcRPS->getNumberOfPositivePictures()+m_pcRPS->getNumberOfLongtermPictures()-1; i > m_pcRPS->getNumberOfNegativePictures()+m_pcRPS->getNumberOfPositivePictures()-1 ; i--)
  {
    if(m_pcRPS->getUsed(i))
    {
      pcRefPic = xGetLongTermRefPic(rcListPic, m_pcRPS->getPOC(i), m_pcRPS->getCheckLTMSBPresent(i));
      pcRefPic->setIsLongTerm(1);
      pcRefPic->getPicYuvRec()->extendPicBorder();
      RefPicSetLtCurr[NumPocLtCurr] = pcRefPic;
      NumPocLtCurr++;
    }
    if(pcRefPic==NULL) 
    {
      pcRefPic = xGetLongTermRefPic(rcListPic, m_pcRPS->getPOC(i), m_pcRPS->getCheckLTMSBPresent(i));
    }
    pcRefPic->setCheckLTMSBPresent(m_pcRPS->getCheckLTMSBPresent(i));  
  }
#if SVC_EXTENSION
  }
#endif

  // ref_pic_list_init
  TComPic*  rpsCurrList0[MAX_NUM_REF+1];
  TComPic*  rpsCurrList1[MAX_NUM_REF+1];
#if SVC_EXTENSION
  Int numInterLayerRPSPics = 0;
#if M0040_ADAPTIVE_RESOLUTION_CHANGE
  if( m_layerId > 0 && m_activeNumILRRefIdx > 0 )
#else
  if( m_layerId > 0 )
#endif
  {
    for( i=0; i < m_pcVPS->getNumDirectRefLayers( m_layerId ); i++ )
    {
#if O0225_MAX_TID_FOR_REF_LAYERS
      Int maxTidIlRefPicsPlus1 = getVPS()->getMaxTidIlRefPicsPlus1(ilpPic[i]->getSlice(0)->getLayerId(),m_layerId);
#else
      Int maxTidIlRefPicsPlus1 = getVPS()->getMaxTidIlRefPicsPlus1(ilpPic[i]->getSlice(0)->getLayerId());
#endif 
      if( ((Int)(ilpPic[i]->getSlice(0)->getTLayer())<= maxTidIlRefPicsPlus1-1) || (maxTidIlRefPicsPlus1==0 && ilpPic[i]->getSlice(0)->getRapPicFlag() ) )
      {
        numInterLayerRPSPics++;
#if DISCARDABLE_PIC_RPS
        assert( ilpPic[i]->getSlice(0)->getDiscardableFlag() == 0 );    // Inter-layer RPS shall not contain picture with discardable_flag = 1.
#endif
      }
    }
    if (numInterLayerRPSPics < m_activeNumILRRefIdx)
    {
      m_activeNumILRRefIdx = numInterLayerRPSPics;
    }
#if MAX_ONE_RESAMPLING_DIRECT_LAYERS
    if( m_pcVPS->getScalabilityMask( SCALABILITY_ID ) )
    {
      Int numResampler = 0;
#if MOTION_RESAMPLING_CONSTRAINT
      Int numMotionResamplers = 0;
      Int refResamplingLayer[MAX_LAYERS];
      memset( refResamplingLayer, 0, sizeof( refResamplingLayer ) );
#endif
#if !RESAMPLING_CONSTRAINT_BUG_FIX
      const Window &scalEL = getSPS()->getScaledRefLayerWindow(m_interLayerPredLayerIdc[i]);
      Int scalingOffset = ((scalEL.getWindowLeftOffset()   == 0 ) && 
                           (scalEL.getWindowRightOffset()  == 0 ) && 
                           (scalEL.getWindowTopOffset()    == 0 ) && 
                           (scalEL.getWindowBottomOffset() == 0 ) 
                          );
#endif

      for( i=0; i < m_activeNumILRRefIdx; i++ )
      {
        UInt refLayerIdc = m_interLayerPredLayerIdc[i];
        UInt refLayerId = m_pcVPS->getRefLayerId( m_layerId, refLayerIdc );
#if RESAMPLING_CONSTRAINT_BUG_FIX
#if MOVE_SCALED_OFFSET_TO_PPS
#if O0098_SCALED_REF_LAYER_ID
        const Window &scalEL = getPPS()->getScaledRefLayerWindowForLayer(refLayerId);
#else
        const Window &scalEL = getPPS()->getScaledRefLayerWindow(m_interLayerPredLayerIdc[i]);
#endif
#else
#if O0098_SCALED_REF_LAYER_ID
        const Window &scalEL = getSPS()->getScaledRefLayerWindowForLayer(refLayerId);
#else
        const Window &scalEL = getSPS()->getScaledRefLayerWindow(m_interLayerPredLayerIdc[i]);
#endif
#endif
        Int scalingOffset = ((scalEL.getWindowLeftOffset()   == 0 ) && 
                             (scalEL.getWindowRightOffset()  == 0 ) && 
                             (scalEL.getWindowTopOffset()    == 0 ) && 
                             (scalEL.getWindowBottomOffset() == 0 ) 
                            );
#endif
#if O0194_DIFFERENT_BITDEPTH_EL_BL
        Bool sameBitDepths = ( g_bitDepthYLayer[m_layerId] == g_bitDepthYLayer[refLayerId] ) && ( g_bitDepthCLayer[m_layerId] == g_bitDepthCLayer[refLayerId] );

        if( !( g_posScalingFactor[refLayerIdc][0] == 65536 && g_posScalingFactor[refLayerIdc][1] == 65536 ) || !scalingOffset || !sameBitDepths 
#if Q0048_CGS_3D_ASYMLUT
          || getPPS()->getCGSFlag()
#endif
          ) // ratio 1x
#else
        if(!( g_posScalingFactor[refLayerIdc][0] == 65536 && g_posScalingFactor[refLayerIdc][1] == 65536 ) || (!scalingOffset)) // ratio 1x
#endif
        {
#if MOTION_RESAMPLING_CONSTRAINT
          UInt predType = m_pcVPS->getDirectDependencyType( m_layerId, refLayerId ) + 1;

          if( predType & 0x1 )
          {
            numResampler++;
            refResamplingLayer[i] = refLayerIdc + 1;
          }

          if( predType & 0x2 )
          {
            numMotionResamplers++;
            refResamplingLayer[i] -= refLayerIdc + 1;
          }
#else
          numResampler++;
#endif
        }
      }
      
      // When both picture sample values and picture motion field resampling processes are invoked for decoding of a particular picture, they shall be applied to the same reference layer picture.
      if( m_activeNumILRRefIdx > 1 && numResampler > 0 )
      {
        for( i=0; i < m_activeNumILRRefIdx; i++ )
        {
          assert( refResamplingLayer[i] >= 0 && "Motion and sample inter-layer prediction shall be from the same layer" );
        }
      }

      // Bitstream constraint for SHVC: The picture resampling process as specified in subclause G.8.1.4.1 shall not be invoked more than once for decoding of each particular picture.
      assert(numResampler <= 1);
#if MOTION_RESAMPLING_CONSTRAINT
      assert( numMotionResamplers <= 1  && "Up to 1 motion resampling is allowed" );
#endif
    }
#endif
  }
  Int numPocTotalCurr = NumPocStCurr0 + NumPocStCurr1 + NumPocLtCurr + m_activeNumILRRefIdx;
#else //SVC_EXTENSION
  Int numPocTotalCurr = NumPocStCurr0 + NumPocStCurr1 + NumPocLtCurr;
#endif //SVC_EXTENSION

  if (checkNumPocTotalCurr)
  {
    // The variable NumPocTotalCurr is derived as specified in subclause 7.4.7.2. It is a requirement of bitstream conformance that the following applies to the value of NumPocTotalCurr:
#if SVC_EXTENSION    // inter-layer prediction is allowed for BLA, CRA pictures of nuh_layer_id>0
    // - If the current picture is a BLA or CRA picture with nuh_layer_id equal to 0, the value of NumPocTotalCurr shall be equal to 0.
    // - Otherwise, when the current picture contains a P or B slice, the value of NumPocTotalCurr shall not be equal to 0.
    if (getRapPicFlag() && getLayerId()==0)
#else
    // - If the current picture is a BLA or CRA picture, the value of NumPocTotalCurr shall be equal to 0.
    // - Otherwise, when the current picture contains a P or B slice, the value of NumPocTotalCurr shall not be equal to 0.
    if (getRapPicFlag())
#endif
    {
      assert(numPocTotalCurr == 0);
    }

    if (m_eSliceType == I_SLICE)
    {
      ::memset( m_apcRefPicList, 0, sizeof (m_apcRefPicList));
      ::memset( m_aiNumRefIdx,   0, sizeof ( m_aiNumRefIdx ));

      return;
    }

    assert(numPocTotalCurr > 0);

    m_aiNumRefIdx[0] = getNumRefIdx(REF_PIC_LIST_0);
    m_aiNumRefIdx[1] = getNumRefIdx(REF_PIC_LIST_1);
  }

    Int cIdx = 0;
    for ( i=0; i<NumPocStCurr0; i++, cIdx++)
    {
      rpsCurrList0[cIdx] = RefPicSetStCurr0[i];
    }
#if SVC_EXTENSION
#if RPL_INIT_N0316_N0082
    if( m_layerId > 0 )
    {      
      for( i = 0; i < m_activeNumILRRefIdx && cIdx < numPocTotalCurr; cIdx ++, i ++)      
      {
        Int refLayerIdc = m_interLayerPredLayerIdc[i];
#if O0225_MAX_TID_FOR_REF_LAYERS
        Int maxTidIlRefPicsPlus1 = getVPS()->getMaxTidIlRefPicsPlus1(ilpPic[refLayerIdc]->getSlice(0)->getLayerId(),m_layerId);
#else
        Int maxTidIlRefPicsPlus1 = getVPS()->getMaxTidIlRefPicsPlus1(ilpPic[refLayerIdc]->getSlice(0)->getLayerId());
#endif
        if( ((Int)(ilpPic[refLayerIdc]->getSlice(0)->getTLayer())<=maxTidIlRefPicsPlus1-1) || (maxTidIlRefPicsPlus1==0 && ilpPic[refLayerIdc]->getSlice(0)->getRapPicFlag()) )
        {
          rpsCurrList0[cIdx] = ilpPic[refLayerIdc];
        }
      }
    }
#endif
#endif //SVC_EXTENSION
    for ( i=0; i<NumPocStCurr1; i++, cIdx++)
    {
      rpsCurrList0[cIdx] = RefPicSetStCurr1[i];
    }
    for ( i=0; i<NumPocLtCurr;  i++, cIdx++)
    {
      rpsCurrList0[cIdx] = RefPicSetLtCurr[i];
    }    
#if !RPL_INIT_N0316_N0082
#if SVC_EXTENSION
    if( m_layerId > 0 )
    {
      for( i = 0; i < m_activeNumILRRefIdx && cIdx < numPocTotalCurr; cIdx ++, i ++)      
      {
        Int refLayerIdc = m_interLayerPredLayerIdc[i];
#if O0225_MAX_TID_FOR_REF_LAYERS
        Int maxTidIlRefPicsPlus1 = getVPS()->getMaxTidIlRefPicsPlus1(ilpPic[refLayerIdc]->getSlice(0)->getLayerId(),m_layerId);
#else
        Int maxTidIlRefPicsPlus1 = getVPS()->getMaxTidIlRefPicsPlus1(ilpPic[refLayerIdc]->getSlice(0)->getLayerId());
#endif 
        if( ((Int)(ilpPic[refLayerIdc]->getSlice(0)->getTLayer())<=maxTidIlRefPicsPlus1-1) || (maxTidIlRefPicsPlus1==0 && ilpPic[refLayerIdc]->getSlice(0)->getRapPicFlag()) )
        {
          rpsCurrList0[cIdx] = ilpPic[refLayerIdc];
        }
      }
    }
#endif
#endif
  assert(cIdx == numPocTotalCurr);

  if (m_eSliceType==B_SLICE)
  {
    cIdx = 0;
    for ( i=0; i<NumPocStCurr1; i++, cIdx++)
    {
      rpsCurrList1[cIdx] = RefPicSetStCurr1[i];
    }
    for ( i=0; i<NumPocStCurr0; i++, cIdx++)
    {
      rpsCurrList1[cIdx] = RefPicSetStCurr0[i];
    }
    for ( i=0; i<NumPocLtCurr;  i++, cIdx++)
    {
      rpsCurrList1[cIdx] = RefPicSetLtCurr[i];
    }    

#if SVC_EXTENSION
    if( m_layerId > 0 )
    {
      for( i = 0; i < m_activeNumILRRefIdx && cIdx < numPocTotalCurr; cIdx ++, i ++)
      {
        Int refLayerIdc = m_interLayerPredLayerIdc[i];
#if O0225_MAX_TID_FOR_REF_LAYERS
        Int maxTidIlRefPicsPlus1 = getVPS()->getMaxTidIlRefPicsPlus1(ilpPic[refLayerIdc]->getSlice(0)->getLayerId(),m_layerId);
#else
        Int maxTidIlRefPicsPlus1 = getVPS()->getMaxTidIlRefPicsPlus1(ilpPic[refLayerIdc]->getSlice(0)->getLayerId());
#endif 
        if( ((Int)(ilpPic[refLayerIdc]->getSlice(0)->getTLayer())<=maxTidIlRefPicsPlus1-1) || (maxTidIlRefPicsPlus1==0 && ilpPic[refLayerIdc]->getSlice(0)->getRapPicFlag()) )
        {
          rpsCurrList1[cIdx] = ilpPic[refLayerIdc];
        }
      }
    }
#endif //SVC_EXTENSION

    assert(cIdx == numPocTotalCurr);
  }

  ::memset(m_bIsUsedAsLongTerm, 0, sizeof(m_bIsUsedAsLongTerm));

  for (Int rIdx = 0; rIdx < m_aiNumRefIdx[0]; rIdx ++)
  {
    cIdx = m_RefPicListModification.getRefPicListModificationFlagL0() ? m_RefPicListModification.getRefPicSetIdxL0(rIdx) : rIdx % numPocTotalCurr;
    assert(cIdx >= 0 && cIdx < numPocTotalCurr);
    m_apcRefPicList[0][rIdx] = rpsCurrList0[ cIdx ];
#if RPL_INIT_N0316_N0082
    m_bIsUsedAsLongTerm[0][rIdx] = ( cIdx >= NumPocStCurr0 && cIdx < NumPocStCurr0 + m_activeNumILRRefIdx ) || ( cIdx >= NumPocStCurr0 + NumPocStCurr1 + m_activeNumILRRefIdx );
#else
    m_bIsUsedAsLongTerm[0][rIdx] = ( cIdx >= NumPocStCurr0 + NumPocStCurr1 );
#endif
  }
  if ( m_eSliceType != B_SLICE )
  {
    m_aiNumRefIdx[1] = 0;
    ::memset( m_apcRefPicList[1], 0, sizeof(m_apcRefPicList[1]));
  }
  else
  {
    for (Int rIdx = 0; rIdx < m_aiNumRefIdx[1]; rIdx ++)
    {
      cIdx = m_RefPicListModification.getRefPicListModificationFlagL1() ? m_RefPicListModification.getRefPicSetIdxL1(rIdx) : rIdx % numPocTotalCurr;
      assert(cIdx >= 0 && cIdx < numPocTotalCurr);
      m_apcRefPicList[1][rIdx] = rpsCurrList1[ cIdx ];
      m_bIsUsedAsLongTerm[1][rIdx] = ( cIdx >= NumPocStCurr0 + NumPocStCurr1 );
    }
  }
}

#if SVC_EXTENSION
Void TComSlice::setRefPicListModificationSvc()
{
  if( !m_pcPPS->getListsModificationPresentFlag()) 
  {
    return;
  }

  if(m_eNalUnitType >= NAL_UNIT_CODED_SLICE_BLA_W_LP && m_eNalUnitType <= NAL_UNIT_CODED_SLICE_CRA)
  {
    return;
  }

  TComRefPicListModification* refPicListModification = &m_RefPicListModification;
  Int numberOfRpsCurrTempList = this->getNumRpsCurrTempList();  // total number of ref pics in listTemp0 including inter-layer ref pics
#if RPL_INIT_N0316_N0082
  Int numberOfPocBeforeCurr = this->getNumNegativeRpsCurrTempList();  // number of negative temporal ref pics 
#endif

  assert(m_aiNumRefIdx[REF_PIC_LIST_0] > 0);
  assert(m_aiNumRefIdx[REF_PIC_LIST_1] > 0);

  //set L0 inter-layer reference picture modification
#if RPL_INIT_N0316_N0082
  Bool hasModification = (m_aiNumRefIdx[REF_PIC_LIST_0] == (numberOfPocBeforeCurr + m_activeNumILRRefIdx)) ? false : true;

  if( m_activeNumILRRefIdx > 1 )
  {
    hasModification = (m_aiNumRefIdx[REF_PIC_LIST_0] >= (numberOfPocBeforeCurr + m_activeNumILRRefIdx)) ? false : true;
  }
#else
  Bool hasModification = (m_aiNumRefIdx[REF_PIC_LIST_0] == numberOfRpsCurrTempList) ? false : true;
#endif
  hasModification = hasModification && ( m_aiNumRefIdx[REF_PIC_LIST_0] > 1 );
  refPicListModification->setRefPicListModificationFlagL0(hasModification);
  if(hasModification)
  { 
    for(Int i = 0; i < min(m_aiNumRefIdx[REF_PIC_LIST_0], numberOfRpsCurrTempList); i++)
    {
      refPicListModification->setRefPicSetIdxL0(i, i);
    }

    if(m_aiNumRefIdx[REF_PIC_LIST_0] > numberOfRpsCurrTempList)
    {
      // repeat last ref pic when the number of active ref idx are more than RPS entries
      for (Int i = numberOfRpsCurrTempList; i < m_aiNumRefIdx[REF_PIC_LIST_0]; i ++)
      {
        refPicListModification->setRefPicSetIdxL0(i, numberOfRpsCurrTempList - 1);
      }
    }
    else
    {
      // number of ILRPs included into the reference picture list with the list modification
      Int includeNumILRP = min( max(1, m_aiNumRefIdx[REF_PIC_LIST_0]-numberOfPocBeforeCurr), m_activeNumILRRefIdx);

      for(Int i = includeNumILRP; i > 0; i-- )
      {
#if RPL_INIT_N0316_N0082
        if((numberOfPocBeforeCurr) >= m_aiNumRefIdx[REF_PIC_LIST_0])
        {
          refPicListModification->setRefPicSetIdxL0(m_aiNumRefIdx[REF_PIC_LIST_0] - i, numberOfPocBeforeCurr + includeNumILRP - i);
        }
        else
        {
          refPicListModification->setRefPicSetIdxL0(m_aiNumRefIdx[REF_PIC_LIST_0] - i, numberOfPocBeforeCurr + includeNumILRP - i);
          for (Int j = numberOfPocBeforeCurr; j < (m_aiNumRefIdx[REF_PIC_LIST_0] - i); j++)
          {
            assert( j + includeNumILRP < numberOfRpsCurrTempList );
            refPicListModification->setRefPicSetIdxL0(j, j + includeNumILRP);
          }
        }
#else
        refPicListModification->setRefPicSetIdxL0(m_aiNumRefIdx[REF_PIC_LIST_0] - i, numberOfRpsCurrTempList - i);
#endif
      }
    }
  }

  //set L1 inter-layer reference picture modification
  hasModification = (m_aiNumRefIdx[REF_PIC_LIST_1] >= numberOfRpsCurrTempList) ? false : true;
  hasModification = hasModification && ( m_aiNumRefIdx[REF_PIC_LIST_1] > 1 );

  refPicListModification->setRefPicListModificationFlagL1(hasModification);
  if(hasModification)
  { 
    for(Int i = 0; i < min(m_aiNumRefIdx[REF_PIC_LIST_1], numberOfRpsCurrTempList); i++)
    {
      refPicListModification->setRefPicSetIdxL1(i, i);
    }
    if(m_aiNumRefIdx[REF_PIC_LIST_1] > numberOfRpsCurrTempList)
    {
      for (Int i = numberOfRpsCurrTempList; i < m_aiNumRefIdx[REF_PIC_LIST_1]; i ++)
      {
        // repeat last ref pic when the number of active ref idx are more than RPS entries
        refPicListModification->setRefPicSetIdxL1(i, numberOfRpsCurrTempList - 1);  
      }
    }
    else
    {
      Int includeNumILRP = min(m_aiNumRefIdx[REF_PIC_LIST_1], m_activeNumILRRefIdx);

      for(Int i = includeNumILRP; i > 0; i-- )
      {
        refPicListModification->setRefPicSetIdxL1(m_aiNumRefIdx[REF_PIC_LIST_1] - i, numberOfRpsCurrTempList - i);
      }
    }
  }
  return;
}
#endif
#if RPL_INIT_N0316_N0082
Int TComSlice::getNumNegativeRpsCurrTempList()
{
  if( m_eSliceType == I_SLICE ) 
  {
    return 0;
  }

  Int numPocBeforeCurr = 0;
  for( UInt i = 0; i < m_pcRPS->getNumberOfNegativePictures(); i++ )
  {
    if(m_pcRPS->getUsed(i))
    {
      numPocBeforeCurr++;
    }
  }

  return numPocBeforeCurr;
}
#endif
TCH */


Int TComSlice::getNumRpsCurrTempList()
{
  Int numRpsCurrTempList = 0;

#if SVC_EXTENSION
  if( m_eSliceType == I_SLICE || ( m_layerId && 
    (m_eNalUnitType >= NAL_UNIT_CODED_SLICE_BLA_W_LP) &&
    (m_eNalUnitType <= NAL_UNIT_CODED_SLICE_CRA) ) )
#else
  if (m_eSliceType == I_SLICE) 
#endif 
  {
#if SVC_EXTENSION
    return m_activeNumILRRefIdx;
#else
    return 0;
#endif
  }
  for(UInt i=0; i < (UInt)(m_pcRPS->getNumberOfNegativePictures()+ m_pcRPS->getNumberOfPositivePictures() + m_pcRPS->getNumberOfLongtermPictures()); i++)
  {
    if(m_pcRPS->getUsed(i))
    {
      numRpsCurrTempList++;
    }
  }
#if SVC_EXTENSION
  if( m_layerId > 0 )
  {
    numRpsCurrTempList += m_activeNumILRRefIdx;
  }
#endif

  return numRpsCurrTempList;
}

Void TComSlice::initEqualRef()
{
  for (Int iDir = 0; iDir < 2; iDir++)
  {
    for (Int iRefIdx1 = 0; iRefIdx1 < MAX_NUM_REF; iRefIdx1++)
    {
      for (Int iRefIdx2 = iRefIdx1; iRefIdx2 < MAX_NUM_REF; iRefIdx2++)
      {
        m_abEqualRef[iDir][iRefIdx1][iRefIdx2] = m_abEqualRef[iDir][iRefIdx2][iRefIdx1] = (iRefIdx1 == iRefIdx2? true : false);
      }
    }
  }
}
/*TCH
Void TComSlice::checkColRefIdx(UInt curSliceIdx, TComPic* pic)
{
  Int i;
  TComSlice* curSlice = pic->getSlice(curSliceIdx);
  Int currColRefPOC =  curSlice->getRefPOC( RefPicList(1-curSlice->getColFromL0Flag()), curSlice->getColRefIdx());
  TComSlice* preSlice;
  Int preColRefPOC;
  for(i=curSliceIdx-1; i>=0; i--)
  {
    preSlice = pic->getSlice(i);
    if(preSlice->getSliceType() != I_SLICE)
    {
      preColRefPOC  = preSlice->getRefPOC( RefPicList(1-preSlice->getColFromL0Flag()), preSlice->getColRefIdx());
      if(currColRefPOC != preColRefPOC)
      {
        printf("Collocated_ref_idx shall always be the same for all slices of a coded picture!\n");
        exit(EXIT_FAILURE);
      }
      else
      {
        break;
      }
    }
  }
}

Void TComSlice::checkCRA(TComReferencePictureSet *pReferencePictureSet, Int& pocCRA, NalUnitType& associatedIRAPType, TComList<TComPic *>& rcListPic)
{
  for(Int i = 0; i < pReferencePictureSet->getNumberOfNegativePictures()+pReferencePictureSet->getNumberOfPositivePictures(); i++)
  {
    if(pocCRA < MAX_UINT && getPOC() > pocCRA)
    {
      assert(getPOC()+pReferencePictureSet->getDeltaPOC(i) >= pocCRA);
    }
  }
  for(Int i = pReferencePictureSet->getNumberOfNegativePictures()+pReferencePictureSet->getNumberOfPositivePictures(); i < pReferencePictureSet->getNumberOfPictures(); i++)
  {
    if(pocCRA < MAX_UINT && getPOC() > pocCRA)
    {
      if (!pReferencePictureSet->getCheckLTMSBPresent(i))
      {
        assert(xGetLongTermRefPic(rcListPic, pReferencePictureSet->getPOC(i), false)->getPOC() >= pocCRA);
      }
      else
      {
      assert(pReferencePictureSet->getPOC(i) >= pocCRA);
    }
  }
  }
  if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP ) // IDR picture found
  {
    pocCRA = getPOC();
    associatedIRAPType = getNalUnitType();
  }
  else if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA ) // CRA picture found
  {
    pocCRA = getPOC();
    associatedIRAPType = getNalUnitType();
  }
  else if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
         || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
         || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP ) // BLA picture found
  {
    pocCRA = getPOC();
    associatedIRAPType = getNalUnitType();
  }
}

// * Function for marking the reference pictures when an IDR/CRA/CRANT/BLA/BLANT is encountered.
// * \param pocCRA POC of the CRA/CRANT/BLA/BLANT picture
// * \param bRefreshPending flag indicating if a deferred decoding refresh is pending
// * \param rcListPic reference to the reference picture list
// * This function marks the reference pictures as "unused for reference" in the following conditions.
// * If the nal_unit_type is IDR/BLA/BLANT, all pictures in the reference picture list
// * are marked as "unused for reference"
// *    If the nal_unit_type is BLA/BLANT, set the pocCRA to the temporal reference of the current picture.
// * Otherwise
// *    If the bRefreshPending flag is true (a deferred decoding refresh is pending) and the current
// *    temporal reference is greater than the temporal reference of the latest CRA/CRANT/BLA/BLANT picture (pocCRA),
// *    mark all reference pictures except the latest CRA/CRANT/BLA/BLANT picture as "unused for reference" and set
// *    the bRefreshPending flag to false.
// *    If the nal_unit_type is CRA/CRANT, set the bRefreshPending flag to true and pocCRA to the temporal
// *    reference of the current picture.
// * Note that the current picture is already placed in the reference list and its marking is not changed.
// * If the current picture has a nal_ref_idc that is not 0, it will remain marked as "used for reference".
//
#if NO_CLRAS_OUTPUT_FLAG
Void TComSlice::decodingRefreshMarking( TComList<TComPic*>& rcListPic, Bool noClrasOutputFlag )
{
  if( !isIRAP() )
  {
    return;
  }

  Int pocCurr = getPOC();
  TComPic* rpcPic = NULL;

  // When the current picture is an IRAP picture with nuh_layer_id equal to 0 and NoClrasOutputFlag is equal to 1, 
  // all reference pictures with any value of nuh_layer_id currently in the DPB (if any) are marked as "unused for reference".
  if( m_layerId == 0 && noClrasOutputFlag )
  {
    // mark all pictures for all layers as not used for reference
    TComList<TComPic*>::iterator iterPic = rcListPic.begin();
    while( iterPic != rcListPic.end() )
    {
      rpcPic = *(iterPic);
      if( rpcPic->getPOC() != pocCurr )
      {
        rpcPic->getSlice(0)->setReferenced(false);
      }
      iterPic++;
    }
  }

  // When the current picture is an IRAP picture with NoRaslOutputFlag equal to 1, 
  // all reference pictures with nuh_layer_id equal to currPicLayerId currently in the DPB (if any) are marked as "unused for reference".
  if( m_noRaslOutputFlag )
  {
    // mark all pictures of a current layer as not used for reference
    TComList<TComPic*>::iterator iterPic = rcListPic.begin();
    while( iterPic != rcListPic.end() )
    {
      rpcPic = *(iterPic);
      if( rpcPic->getPOC() != pocCurr && rpcPic->getLayerId() == m_layerId )
      {
        rpcPic->getSlice(0)->setReferenced(false);
      }
      iterPic++;
    }
  }
}

Void TComSlice::decodingRefreshMarking(Int& pocCRA, Bool& bRefreshPending, TComList<TComPic*>& rcListPic, Bool noClrasOutputFlag)
#else
Void TComSlice::decodingRefreshMarking(Int& pocCRA, Bool& bRefreshPending, TComList<TComPic*>& rcListPic)
#endif
{
  TComPic*                 rpcPic;
#if !FIX1172
  setAssociatedIRAPPOC(pocCRA);
#endif
  Int pocCurr = getPOC(); 

  if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
    || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
    || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
    || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
    || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP )  // IDR or BLA picture
  {
    // mark all pictures as not used for reference
    TComList<TComPic*>::iterator        iterPic       = rcListPic.begin();
    while (iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic);
      rpcPic->setCurrSliceIdx(0);
#if NO_CLRAS_OUTPUT_FLAG
#if POC_RESET_IDC_ENCODER
      if (noClrasOutputFlag)
      {
        rpcPic->getSlice(0)->setReferenced(false);  // all layers // TODO. This does not mark all layers
      }
      else
      {
        if (rpcPic->getLayerId() == m_layerId) rpcPic->getSlice(0)->setReferenced(false);  // only current layer
      }
#else
      if (noClrasOutputFlag)
      {
        if (rpcPic->getPOC() != pocCurr) rpcPic->getSlice(0)->setReferenced(false);  // all layers
      }
      else
      {
        if (rpcPic->getPOC() != pocCurr && rpcPic->getLayerId() == m_layerId) rpcPic->getSlice(0)->setReferenced(false);  // only current layer
      }
#endif
#else
      if (rpcPic->getPOC() != pocCurr) rpcPic->getSlice(0)->setReferenced(false);
#endif
      iterPic++;
    }
#if POC_RESET_IDC_ENCODER
    this->getPic()->getSlice(0)->setReferenced(true);   // Mark the current picture back as refererced.
#endif
    if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP )
    {
      pocCRA = pocCurr;
    }
#if EFFICIENT_FIELD_IRAP
    bRefreshPending = true;
#endif
  }
  else // CRA or No DR
  {
#if EFFICIENT_FIELD_IRAP
    if(getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_N_LP || getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL)
    {
      if (bRefreshPending==true && pocCurr > m_iLastIDR) // IDR reference marking pending 
      {
        TComList<TComPic*>::iterator        iterPic       = rcListPic.begin();
        while (iterPic != rcListPic.end())
        {
          rpcPic = *(iterPic);
          if (rpcPic->getPOC() != pocCurr && rpcPic->getPOC() != m_iLastIDR)
          {
            rpcPic->getSlice(0)->setReferenced(false);
          }
          iterPic++;
        }
        bRefreshPending = false; 
      }
    }
    else
    {
#endif
    if (bRefreshPending==true && pocCurr > pocCRA) // CRA reference marking pending 
    {
      TComList<TComPic*>::iterator        iterPic       = rcListPic.begin();
      while (iterPic != rcListPic.end())
      {
        rpcPic = *(iterPic);
        if (rpcPic->getPOC() != pocCurr && rpcPic->getPOC() != pocCRA)
        {
          rpcPic->getSlice(0)->setReferenced(false);
        }
        iterPic++;
      }
      bRefreshPending = false; 
    }
#if EFFICIENT_FIELD_IRAP
    }
#endif
    if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA ) // CRA picture found
    {
      bRefreshPending = true; 
      pocCRA = pocCurr;
    }
  }
}

TCH */
Void TComSlice::copySliceInfo(TComSlice *pSrc)
{
  assert( pSrc != NULL );

  Int i, j, k;

  m_iPOC                 = pSrc->m_iPOC;
  m_eNalUnitType         = pSrc->m_eNalUnitType;
  m_eSliceType           = pSrc->m_eSliceType;
  m_iSliceQp             = pSrc->m_iSliceQp;
#if ADAPTIVE_QP_SELECTION
  m_iSliceQpBase         = pSrc->m_iSliceQpBase;
#endif
  m_deblockingFilterDisable   = pSrc->m_deblockingFilterDisable;
  m_deblockingFilterOverrideFlag = pSrc->m_deblockingFilterOverrideFlag;
  m_deblockingFilterBetaOffsetDiv2 = pSrc->m_deblockingFilterBetaOffsetDiv2;
  m_deblockingFilterTcOffsetDiv2 = pSrc->m_deblockingFilterTcOffsetDiv2;
  
  for (i = 0; i < 2; i++)
  {
    m_aiNumRefIdx[i]     = pSrc->m_aiNumRefIdx[i];
  }

  for (i = 0; i < MAX_NUM_REF; i++)
  {
    m_list1IdxToList0Idx[i] = pSrc->m_list1IdxToList0Idx[i];
  } 
  m_bCheckLDC             = pSrc->m_bCheckLDC;
  m_iSliceQpDelta        = pSrc->m_iSliceQpDelta;
  m_iSliceQpDeltaCb      = pSrc->m_iSliceQpDeltaCb;
  m_iSliceQpDeltaCr      = pSrc->m_iSliceQpDeltaCr;
  for (i = 0; i < 2; i++)
  {
    for (j = 0; j < MAX_NUM_REF; j++)
    {
      m_apcRefPicList[i][j]  = pSrc->m_apcRefPicList[i][j];
      m_aiRefPOCList[i][j]   = pSrc->m_aiRefPOCList[i][j];
    }
  }
  for (i = 0; i < 2; i++)
  {
    for (j = 0; j < MAX_NUM_REF + 1; j++)
    {
      m_bIsUsedAsLongTerm[i][j] = pSrc->m_bIsUsedAsLongTerm[i][j];
    }
  }
  m_iDepth               = pSrc->m_iDepth;

  // referenced slice
  m_bRefenced            = pSrc->m_bRefenced;

  // access channel
#if SVC_EXTENSION
  m_pcVPS                = pSrc->m_pcVPS;
  m_layerId              = pSrc->m_layerId;
  m_activeNumILRRefIdx         = pSrc->m_activeNumILRRefIdx;
  m_interLayerPredEnabledFlag  = pSrc->m_interLayerPredEnabledFlag;
  memcpy( m_interLayerPredLayerIdc, pSrc->m_interLayerPredLayerIdc, sizeof( m_interLayerPredLayerIdc ) );
#if P0312_VERT_PHASE_ADJ
  memcpy( m_vertPhasePositionFlag, pSrc->m_vertPhasePositionFlag, sizeof( m_vertPhasePositionFlag ) );
#endif
#endif
  m_pcSPS                = pSrc->m_pcSPS;
  m_pcPPS                = pSrc->m_pcPPS;
  m_pcRPS                = pSrc->m_pcRPS;
  m_iLastIDR             = pSrc->m_iLastIDR;

  m_pcPic                = pSrc->m_pcPic;

  m_colFromL0Flag        = pSrc->m_colFromL0Flag;
  m_colRefIdx            = pSrc->m_colRefIdx;
  setLambdas(pSrc->getLambdas());
  for (i = 0; i < 2; i++)
  {
    for (j = 0; j < MAX_NUM_REF; j++)
    {
      for (k =0; k < MAX_NUM_REF; k++)
      {
        m_abEqualRef[i][j][k] = pSrc->m_abEqualRef[i][j][k];
      }
    }
  }

  m_uiTLayer                      = pSrc->m_uiTLayer;
  m_bTLayerSwitchingFlag          = pSrc->m_bTLayerSwitchingFlag;

  m_sliceMode                   = pSrc->m_sliceMode;
  m_sliceArgument               = pSrc->m_sliceArgument;
  m_sliceCurStartCUAddr         = pSrc->m_sliceCurStartCUAddr;
  m_sliceCurEndCUAddr           = pSrc->m_sliceCurEndCUAddr;
  m_sliceIdx                    = pSrc->m_sliceIdx;
  m_sliceSegmentMode            = pSrc->m_sliceSegmentMode;
  m_sliceSegmentArgument        = pSrc->m_sliceSegmentArgument; 
  m_sliceSegmentCurStartCUAddr  = pSrc->m_sliceSegmentCurStartCUAddr;
  m_sliceSegmentCurEndCUAddr    = pSrc->m_sliceSegmentCurEndCUAddr;
  m_nextSlice                    = pSrc->m_nextSlice;
  m_nextSliceSegment             = pSrc->m_nextSliceSegment;
  for ( Int e=0 ; e<2 ; e++ )
  {
    for ( Int n=0 ; n<MAX_NUM_REF ; n++ )
    {
      memcpy(m_weightPredTable[e][n], pSrc->m_weightPredTable[e][n], sizeof(wpScalingParam)*3 );
    }
  }
  m_saoEnabledFlag = pSrc->m_saoEnabledFlag; 
  m_saoEnabledFlagChroma = pSrc->m_saoEnabledFlagChroma;
  m_cabacInitFlag                = pSrc->m_cabacInitFlag;
  m_numEntryPointOffsets  = pSrc->m_numEntryPointOffsets;

  m_bLMvdL1Zero = pSrc->m_bLMvdL1Zero;
  m_LFCrossSliceBoundaryFlag = pSrc->m_LFCrossSliceBoundaryFlag;
  m_enableTMVPFlag                = pSrc->m_enableTMVPFlag;
  m_maxNumMergeCand               = pSrc->m_maxNumMergeCand;
}

Int TComSlice::m_prevTid0POC = 0;

// * Function for setting the slice's temporal layer ID and corresponding temporal_layer_switching_point_flag.
// * \param uiTLayer Temporal layer ID of the current slice
// * The decoder calls this function to set temporal_layer_switching_point_flag for each temporal layer based on
// * the SPS's temporal_id_nesting_flag and the parsed PPS.  Then, current slice's temporal layer ID and
// * temporal_layer_switching_point_flag is set accordingly.
//
Void TComSlice::setTLayerInfo( UInt uiTLayer )
{
  m_uiTLayer = uiTLayer;
}

// * Function for checking if this is a switching-point
//


/* TCH
Bool TComSlice::isTemporalLayerSwitchingPoint( TComList<TComPic*>& rcListPic )
{
  TComPic* rpcPic;
  // loop through all pictures in the reference picture buffer
  TComList<TComPic*>::iterator iterPic = rcListPic.begin();
  while ( iterPic != rcListPic.end())
  {
    rpcPic = *(iterPic++);
    if(rpcPic->getSlice(0)->isReferenced() && rpcPic->getPOC() != getPOC())
    {
      if(rpcPic->getTLayer() >= getTLayer())
      {
        return false;
      }
    }
  }
  return true;
}

//  Function for checking if this is a STSA candidate
Bool TComSlice::isStepwiseTemporalLayerSwitchingPointCandidate( TComList<TComPic*>& rcListPic )
{
    TComPic* rpcPic;
    
    TComList<TComPic*>::iterator iterPic = rcListPic.begin();
    while ( iterPic != rcListPic.end())
    {
        rpcPic = *(iterPic++);
        if(rpcPic->getSlice(0)->isReferenced() &&  (rpcPic->getUsedByCurr()==true) && rpcPic->getPOC() != getPOC())
        {
            if(rpcPic->getTLayer() >= getTLayer())
            {
                return false;
            }
        }
    }
    return true;
}

#if POC_RESET_IDC_ENCODER
Void TComSlice::checkLeadingPictureRestrictions(TComList<TComPic*>& rcListPic, Bool usePocBeforeReset)
#else
Void TComSlice::checkLeadingPictureRestrictions(TComList<TComPic*>& rcListPic)
#endif
{
  TComPic* rpcPic;

  Int nalUnitType = this->getNalUnitType();

  // When a picture is a leading picture, it shall be a RADL or RASL picture.
  if(this->getAssociatedIRAPPOC() > this->getPOC())
  {
    // Do not check IRAP pictures since they may get a POC lower than their associated IRAP
    if(nalUnitType < NAL_UNIT_CODED_SLICE_BLA_W_LP ||
       nalUnitType > NAL_UNIT_RESERVED_IRAP_VCL23)
    {
      assert(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||
             nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R ||
             nalUnitType == NAL_UNIT_CODED_SLICE_RADL_N ||
             nalUnitType == NAL_UNIT_CODED_SLICE_RADL_R);
    }
  }

  // When a picture is a trailing picture, it shall not be a RADL or RASL picture.
  if(this->getAssociatedIRAPPOC() < this->getPOC())
  {
    assert(nalUnitType != NAL_UNIT_CODED_SLICE_RASL_N &&
           nalUnitType != NAL_UNIT_CODED_SLICE_RASL_R &&
           nalUnitType != NAL_UNIT_CODED_SLICE_RADL_N &&
           nalUnitType != NAL_UNIT_CODED_SLICE_RADL_R);
  }

  // No RASL pictures shall be present in the bitstream that are associated
  // with a BLA picture having nal_unit_type equal to BLA_W_RADL or BLA_N_LP.
  if(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||
     nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R)
  {
    assert(this->getAssociatedIRAPType() != NAL_UNIT_CODED_SLICE_BLA_W_RADL &&
           this->getAssociatedIRAPType() != NAL_UNIT_CODED_SLICE_BLA_N_LP);
  }

  // No RASL pictures shall be present in the bitstream that are associated with
  // an IDR picture.
  if(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||
     nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R)
  {
    assert(this->getAssociatedIRAPType() != NAL_UNIT_CODED_SLICE_IDR_N_LP   &&
           this->getAssociatedIRAPType() != NAL_UNIT_CODED_SLICE_IDR_W_RADL);
  }

  // No RADL pictures shall be present in the bitstream that are associated with
  // a BLA picture having nal_unit_type equal to BLA_N_LP or that are associated
  // with an IDR picture having nal_unit_type equal to IDR_N_LP.
  if(nalUnitType == NAL_UNIT_CODED_SLICE_RADL_N ||
     nalUnitType == NAL_UNIT_CODED_SLICE_RADL_R)
  {
    assert(this->getAssociatedIRAPType() != NAL_UNIT_CODED_SLICE_BLA_N_LP   &&
           this->getAssociatedIRAPType() != NAL_UNIT_CODED_SLICE_IDR_N_LP);
  }

  // loop through all pictures in the reference picture buffer
  TComList<TComPic*>::iterator iterPic = rcListPic.begin();
  while ( iterPic != rcListPic.end())
  {
    rpcPic = *(iterPic++);
#if BUGFIX_INTRAPERIOD
    if(!rpcPic->getReconMark())
    {
      continue;
    }
#endif
    if (rpcPic->getPOC() == this->getPOC())
    {
      continue;
    }

    // Any picture that has PicOutputFlag equal to 1 that precedes an IRAP picture
    // in decoding order shall precede the IRAP picture in output order.
    // (Note that any picture following in output order would be present in the DPB)
#if !SETTING_NO_OUT_PIC_PRIOR
    if(rpcPic->getSlice(0)->getPicOutputFlag() == 1)
#else
    if(rpcPic->getSlice(0)->getPicOutputFlag() == 1 && !this->getNoOutputPriorPicsFlag())
#endif
    {
      if(nalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP    ||
         nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_LP    ||
         nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_RADL  ||
         nalUnitType == NAL_UNIT_CODED_SLICE_CRA         ||
         nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP    ||
         nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL)
      {
#if POC_RESET_IDC_ENCODER
        if( usePocBeforeReset )
        {
          assert(rpcPic->getSlice(0)->getPocValueBeforeReset() < this->getPocValueBeforeReset());
        }
        else
        {
          assert(rpcPic->getPOC() < this->getPOC());
        }
#else
        assert(rpcPic->getPOC() < this->getPOC());
#endif
      }
    }

    // Any picture that has PicOutputFlag equal to 1 that precedes an IRAP picture
    // in decoding order shall precede any RADL picture associated with the IRAP
    // picture in output order.
    if(rpcPic->getSlice(0)->getPicOutputFlag() == 1)
    {
      if((nalUnitType == NAL_UNIT_CODED_SLICE_RADL_N ||
          nalUnitType == NAL_UNIT_CODED_SLICE_RADL_R))
      {
        // rpcPic precedes the IRAP in decoding order
        if(this->getAssociatedIRAPPOC() > rpcPic->getSlice(0)->getAssociatedIRAPPOC())
        {
          // rpcPic must not be the IRAP picture
          if(this->getAssociatedIRAPPOC() != rpcPic->getPOC())
          {
#if POC_RESET_IDC_ENCODER
            if( usePocBeforeReset )
            {
              assert(rpcPic->getSlice(0)->getPocValueBeforeReset() < this->getPocValueBeforeReset());
            }
            else
            {
              assert(rpcPic->getPOC() < this->getPOC());
            }
#else
            assert(rpcPic->getPOC() < this->getPOC());
#endif
          }
        }
      }
    }

    // When a picture is a leading picture, it shall precede, in decoding order,
    // all trailing pictures that are associated with the same IRAP picture.
    if(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||
       nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R ||
       nalUnitType == NAL_UNIT_CODED_SLICE_RADL_N ||
       nalUnitType == NAL_UNIT_CODED_SLICE_RADL_R)
    {
      if(rpcPic->getSlice(0)->getAssociatedIRAPPOC() == this->getAssociatedIRAPPOC())
      {
        // rpcPic is a picture that preceded the leading in decoding order since it exist in the DPB
        // rpcPic would violate the constraint if it was a trailing picture
#if POC_RESET_IDC_ENCODER
        if( usePocBeforeReset )
        {
          assert(rpcPic->getPOC() <= this->getAssociatedIrapPocBeforeReset());
        }
        else
        {
          assert(rpcPic->getPOC() <= this->getAssociatedIRAPPOC());
        }
#else
        assert(rpcPic->getPOC() <= this->getAssociatedIRAPPOC());
#endif
      }
    }

    // Any RASL picture associated with a CRA or BLA picture shall precede any
    // RADL picture associated with the CRA or BLA picture in output order
    if(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||
       nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R)
    { 
      if((this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_BLA_N_LP   ||
          this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_BLA_W_LP   ||
          this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL ||
          this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_CRA)       &&
          this->getAssociatedIRAPPOC() == rpcPic->getSlice(0)->getAssociatedIRAPPOC())
      {
        if(rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N ||
           rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_R)
        {
          assert(rpcPic->getPOC() > this->getPOC());
        }
      }
    }

    // Any RASL picture associated with a CRA picture shall follow, in output
    // order, any IRAP picture that precedes the CRA picture in decoding order.
    if(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||
       nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R)
    {
      if(this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_CRA)
      {
        if(rpcPic->getSlice(0)->getPOC() < this->getAssociatedIRAPPOC() &&
           (rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP   ||
            rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP   ||
            rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL ||
            rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP   ||
            rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
            rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA))
        {
          assert(this->getPOC() > rpcPic->getSlice(0)->getPOC());
        }
      }
    }
  }
}



// Function for applying picture marking based on the Reference Picture Set in pReferencePictureSet.
Void TComSlice::applyReferencePictureSet( TComList<TComPic*>& rcListPic, TComReferencePictureSet *pReferencePictureSet)
{
  TComPic* rpcPic;
  Int i, isReference;

#if !ALIGNED_BUMPING
  checkLeadingPictureRestrictions(rcListPic);
#endif

  // loop through all pictures in the reference picture buffer
  TComList<TComPic*>::iterator iterPic = rcListPic.begin();
  while ( iterPic != rcListPic.end())
  {
    rpcPic = *(iterPic++);

    if(!rpcPic->getSlice( 0 )->isReferenced())
    {
      continue;
    }

    isReference = 0;
    // loop through all pictures in the Reference Picture Set
    // to see if the picture should be kept as reference picture
    for(i=0;i<pReferencePictureSet->getNumberOfPositivePictures()+pReferencePictureSet->getNumberOfNegativePictures();i++)
    {
      if(!rpcPic->getIsLongTerm() && rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getPOC() + pReferencePictureSet->getDeltaPOC(i))
      {
        isReference = 1;
        rpcPic->setUsedByCurr(pReferencePictureSet->getUsed(i));
        rpcPic->setIsLongTerm(0);
      }
    }
    for(;i<pReferencePictureSet->getNumberOfPictures();i++)
    {
      if(pReferencePictureSet->getCheckLTMSBPresent(i)==true)
      {
        if(rpcPic->getIsLongTerm() && (rpcPic->getPicSym()->getSlice(0)->getPOC()) == pReferencePictureSet->getPOC(i))
        {
          isReference = 1;
          rpcPic->setUsedByCurr(pReferencePictureSet->getUsed(i));
        }
      }
      else 
      {
        Int pocCycle = 1<<rpcPic->getPicSym()->getSlice(0)->getSPS()->getBitsForPOC();
        Int curPoc = rpcPic->getPicSym()->getSlice(0)->getPOC() & (pocCycle-1);
        Int refPoc = pReferencePictureSet->getPOC(i) & (pocCycle-1);
        if(rpcPic->getIsLongTerm() && curPoc == refPoc)
        {
          isReference = 1;
          rpcPic->setUsedByCurr(pReferencePictureSet->getUsed(i));
        }
      }

    }
#if DISCARDABLE_PIC_RPS
    if( isReference ) // Current picture is in the temporal RPS
    {
      assert( rpcPic->getSlice(0)->getDiscardableFlag() == 0 ); // Temporal RPS shall not contain picture with discardable_flag equal to 1
    }
#endif
    // mark the picture as "unused for reference" if it is not in
    // the Reference Picture Set
    if(rpcPic->getPicSym()->getSlice(0)->getPOC() != this->getPOC() && isReference == 0)    
    {            
      rpcPic->getSlice( 0 )->setReferenced( false );   
      rpcPic->setUsedByCurr(0);
      rpcPic->setIsLongTerm(0);
    }
    //check that pictures of higher temporal layers are not used
    assert(rpcPic->getSlice( 0 )->isReferenced()==0||rpcPic->getUsedByCurr()==0||rpcPic->getTLayer()<=this->getTLayer());
    //check that pictures of higher or equal temporal layer are not in the RPS if the current picture is a TSA picture
    if(this->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_R || this->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_N)
    {
      assert(rpcPic->getSlice( 0 )->isReferenced()==0||rpcPic->getTLayer()<this->getTLayer());
    }
    //check that pictures marked as temporal layer non-reference pictures are not used for reference
    if(rpcPic->getPicSym()->getSlice(0)->getPOC() != this->getPOC() && rpcPic->getTLayer()==this->getTLayer())
    {
      assert(rpcPic->getSlice( 0 )->isReferenced()==0||rpcPic->getUsedByCurr()==0||rpcPic->getSlice( 0 )->getTemporalLayerNonReferenceFlag()==false);
    }
  }
}

//  Function for applying picture marking based on the Reference Picture Set in pReferencePictureSet.
#if ALLOW_RECOVERY_POINT_AS_RAP
Int TComSlice::checkThatAllRefPicsAreAvailable( TComList<TComPic*>& rcListPic, TComReferencePictureSet *pReferencePictureSet, Bool printErrors, Int pocRandomAccess, Bool bUseRecoveryPoint)
#else
Int TComSlice::checkThatAllRefPicsAreAvailable( TComList<TComPic*>& rcListPic, TComReferencePictureSet *pReferencePictureSet, Bool printErrors, Int pocRandomAccess)
#endif
{
#if ALLOW_RECOVERY_POINT_AS_RAP
  Int atLeastOneUnabledByRecoveryPoint = 0;
  Int atLeastOneFlushedByPreviousIDR = 0;
#endif
  TComPic* rpcPic;
  Int i, isAvailable;
  Int atLeastOneLost = 0;
  Int atLeastOneRemoved = 0;
  Int iPocLost = 0;

  // loop through all long-term pictures in the Reference Picture Set
  // to see if the picture should be kept as reference picture
  for(i=pReferencePictureSet->getNumberOfNegativePictures()+pReferencePictureSet->getNumberOfPositivePictures();i<pReferencePictureSet->getNumberOfPictures();i++)
  {
    isAvailable = 0;
    // loop through all pictures in the reference picture buffer
    TComList<TComPic*>::iterator iterPic = rcListPic.begin();
    while ( iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic++);
      if(pReferencePictureSet->getCheckLTMSBPresent(i)==true)
      {
        if(rpcPic->getIsLongTerm() && (rpcPic->getPicSym()->getSlice(0)->getPOC()) == pReferencePictureSet->getPOC(i) && rpcPic->getSlice(0)->isReferenced())
        {
#if ALLOW_RECOVERY_POINT_AS_RAP
          if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess)
          {
            isAvailable = 0;
          }
          else
          {
          isAvailable = 1;
        }
#else
          isAvailable = 1;
#endif
        }
      }
      else 
      {
        Int pocCycle = 1<<rpcPic->getPicSym()->getSlice(0)->getSPS()->getBitsForPOC();
        Int curPoc = rpcPic->getPicSym()->getSlice(0)->getPOC() & (pocCycle-1);
        Int refPoc = pReferencePictureSet->getPOC(i) & (pocCycle-1);
        if(rpcPic->getIsLongTerm() && curPoc == refPoc && rpcPic->getSlice(0)->isReferenced())
        {
#if ALLOW_RECOVERY_POINT_AS_RAP
          if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess)
          {
            isAvailable = 0;
          }
          else
          {
          isAvailable = 1;
        }
#else
          isAvailable = 1;
#endif
        }
      }
    }
    // if there was no such long-term check the short terms
    if(!isAvailable)
    {
      iterPic = rcListPic.begin();
      while ( iterPic != rcListPic.end())
      {
        rpcPic = *(iterPic++);

        Int pocCycle = 1 << rpcPic->getPicSym()->getSlice(0)->getSPS()->getBitsForPOC();
        Int curPoc = rpcPic->getPicSym()->getSlice(0)->getPOC();
        Int refPoc = pReferencePictureSet->getPOC(i);
        if (!pReferencePictureSet->getCheckLTMSBPresent(i))
        {
          curPoc = curPoc & (pocCycle - 1);
          refPoc = refPoc & (pocCycle - 1);
        }
        
        if (rpcPic->getSlice(0)->isReferenced() && curPoc == refPoc)
        {
#if ALLOW_RECOVERY_POINT_AS_RAP
          if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess)
          {
            isAvailable = 0;
          }
          else
          {
          isAvailable = 1;
          rpcPic->setIsLongTerm(1);
          break;
        }
#else
          isAvailable = 1;
          rpcPic->setIsLongTerm(1);
          break;
#endif
        }
      }
    }
    // report that a picture is lost if it is in the Reference Picture Set
    // but not available as reference picture
    if(isAvailable == 0)    
    {            
      if (this->getPOC() + pReferencePictureSet->getDeltaPOC(i) >= pocRandomAccess)
      {
        if(!pReferencePictureSet->getUsed(i) )
        {
          if(printErrors)
          {
            printf("\nLong-term reference picture with POC = %3d seems to have been removed or not correctly decoded.", this->getPOC() + pReferencePictureSet->getDeltaPOC(i));
          }
          atLeastOneRemoved = 1;
        }
        else
        {
          if(printErrors)
          {
            printf("\nLong-term reference picture with POC = %3d is lost or not correctly decoded!", this->getPOC() + pReferencePictureSet->getDeltaPOC(i));
          }
          atLeastOneLost = 1;
          iPocLost=this->getPOC() + pReferencePictureSet->getDeltaPOC(i);
        }
      }
#if ALLOW_RECOVERY_POINT_AS_RAP
      else if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess)
      {
        atLeastOneUnabledByRecoveryPoint = 1;
      }
      else if(bUseRecoveryPoint && (this->getAssociatedIRAPType()==NAL_UNIT_CODED_SLICE_IDR_N_LP || this->getAssociatedIRAPType()==NAL_UNIT_CODED_SLICE_IDR_W_RADL))
      {
        atLeastOneFlushedByPreviousIDR = 1;
      }
#endif
    }
  }  
  // loop through all short-term pictures in the Reference Picture Set
  // to see if the picture should be kept as reference picture
  for(i=0;i<pReferencePictureSet->getNumberOfNegativePictures()+pReferencePictureSet->getNumberOfPositivePictures();i++)
  {
    isAvailable = 0;
    // loop through all pictures in the reference picture buffer
    TComList<TComPic*>::iterator iterPic = rcListPic.begin();
    while ( iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic++);

      if(!rpcPic->getIsLongTerm() && rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getPOC() + pReferencePictureSet->getDeltaPOC(i) && rpcPic->getSlice(0)->isReferenced())
      {
#if ALLOW_RECOVERY_POINT_AS_RAP
        if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess)
        {
          isAvailable = 0;
        }
        else
        {
        isAvailable = 1;
      }
#else
        isAvailable = 1;
#endif
      }
    }
    // report that a picture is lost if it is in the Reference Picture Set
    // but not available as reference picture
    if(isAvailable == 0)    
    {            
#if !UNAVAILABLE_PIC_BUGFIX
      if (this->getPOC() + pReferencePictureSet->getDeltaPOC(i) >= pocRandomAccess)
#endif
      {
        if(!pReferencePictureSet->getUsed(i) )
        {
          if(printErrors)
          {
            printf("\nShort-term reference picture with POC = %3d seems to have been removed or not correctly decoded.", this->getPOC() + pReferencePictureSet->getDeltaPOC(i));
          }
          atLeastOneRemoved = 1;
        }
        else
        {
          if(printErrors)
          {
            printf("\nShort-term reference picture with POC = %3d is lost or not correctly decoded!", this->getPOC() + pReferencePictureSet->getDeltaPOC(i));
          }
          atLeastOneLost = 1;
          iPocLost=this->getPOC() + pReferencePictureSet->getDeltaPOC(i);
        }
      }
#if ALLOW_RECOVERY_POINT_AS_RAP
#if UNAVAILABLE_PIC_BUGFIX
      if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess)
#else
      else if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess)
#endif
      {
        atLeastOneUnabledByRecoveryPoint = 1;
      }
      else if(bUseRecoveryPoint && (this->getAssociatedIRAPType()==NAL_UNIT_CODED_SLICE_IDR_N_LP || this->getAssociatedIRAPType()==NAL_UNIT_CODED_SLICE_IDR_W_RADL))
      {
        atLeastOneFlushedByPreviousIDR = 1;
      }
#endif
    }
    }
#if ALLOW_RECOVERY_POINT_AS_RAP
  if(atLeastOneUnabledByRecoveryPoint || atLeastOneFlushedByPreviousIDR)
  {
    return -1;
  }    
#endif
  if(atLeastOneLost)
  {
    return iPocLost+1;
  }
  if(atLeastOneRemoved)
  {
    return -2;
  }
  else
  {
    return 0;
  }
}

//  Function for constructing an explicit Reference Picture Set out of the available pictures in a referenced Reference Picture Set
#if ALLOW_RECOVERY_POINT_AS_RAP
Void TComSlice::createExplicitReferencePictureSetFromReference( TComList<TComPic*>& rcListPic, TComReferencePictureSet *pReferencePictureSet, Bool isRAP, Int pocRandomAccess, Bool bUseRecoveryPoint)
#else
Void TComSlice::createExplicitReferencePictureSetFromReference( TComList<TComPic*>& rcListPic, TComReferencePictureSet *pReferencePictureSet, Bool isRAP)
#endif
{
  TComPic* rpcPic;
  Int i, j;
  Int k = 0;
  Int nrOfNegativePictures = 0;
  Int nrOfPositivePictures = 0;
  TComReferencePictureSet* pcRPS = this->getLocalRPS();
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  Bool pocsAdjusted = false;
#endif

  // loop through all pictures in the Reference Picture Set
  for(i=0;i<pReferencePictureSet->getNumberOfPictures();i++)
  {
    j = 0;
    // loop through all pictures in the reference picture buffer
    TComList<TComPic*>::iterator iterPic = rcListPic.begin();
    while ( iterPic != rcListPic.end())
    {
      j++;
      rpcPic = *(iterPic++);

#if P0297_VPS_POC_LSB_ALIGNED_FLAG
      // poc adjustement by poc reset needs to be taken into account here
      Int deltaPOC = pReferencePictureSet->getDeltaPOC(i) - rpcPic->getPicSym()->getSlice(0)->getPocResetDeltaPoc();
      if (rpcPic->getPicSym()->getSlice(0)->getPocResetDeltaPoc() != 0)
      {
        pocsAdjusted = true;
      }

      if (rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getPOC() + deltaPOC && rpcPic->getSlice(0)->isReferenced())
#else
      if(rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getPOC() + pReferencePictureSet->getDeltaPOC(i) && rpcPic->getSlice(0)->isReferenced())
#endif
      {
        // This picture exists as a reference picture
        // and should be added to the explicit Reference Picture Set
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
        pcRPS->setDeltaPOC(k, deltaPOC);
#else
        pcRPS->setDeltaPOC(k, pReferencePictureSet->getDeltaPOC(i));
#endif
        pcRPS->setUsed(k, pReferencePictureSet->getUsed(i) && (!isRAP));
#if ALLOW_RECOVERY_POINT_AS_RAP
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
        pcRPS->setUsed(k, pcRPS->getUsed(k) && !(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + deltaPOC < pocRandomAccess) ); 
#else
        pcRPS->setUsed(k, pcRPS->getUsed(k) && !(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess) ); 
#endif
#endif
        if(pcRPS->getDeltaPOC(k) < 0)
        {
          nrOfNegativePictures++;
        }
        else
        {
          nrOfPositivePictures++;
        }
        k++;
      }
    }
  }
#if EFFICIENT_FIELD_IRAP
  Bool useNewRPS = false;
  // if current picture is complimentary field associated to IRAP, add the IRAP to its RPS. 
  if(m_pcPic->isField())
  {
    TComList<TComPic*>::iterator iterPic = rcListPic.begin();
    while ( iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic++);
      if(rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getAssociatedIRAPPOC() && this->getAssociatedIRAPPOC() == this->getPOC()+1)
      {
        pcRPS->setDeltaPOC(k, 1);
        pcRPS->setUsed(k, true);
        nrOfPositivePictures++;
        k ++;
        useNewRPS = true;
      }
    }
  }
#endif
  pcRPS->setNumberOfNegativePictures(nrOfNegativePictures);
  pcRPS->setNumberOfPositivePictures(nrOfPositivePictures);
  pcRPS->setNumberOfPictures(nrOfNegativePictures+nrOfPositivePictures);
  // This is a simplistic inter rps example. A smarter encoder will look for a better reference RPS to do the 
  // inter RPS prediction with.  Here we just use the reference used by pReferencePictureSet.
  // If pReferencePictureSet is not inter_RPS_predicted, then inter_RPS_prediction is for the current RPS also disabled.
  if (!pReferencePictureSet->getInterRPSPrediction()
#if EFFICIENT_FIELD_IRAP
    || useNewRPS
#endif
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
    || pocsAdjusted  // inter RPS prediction does not work if POCs have been adjusted
#endif
    )
  {
    pcRPS->setInterRPSPrediction(false);
    pcRPS->setNumRefIdc(0);
  }
  else
  {
    Int rIdx =  this->getRPSidx() - pReferencePictureSet->getDeltaRIdxMinus1() - 1;
    Int deltaRPS = pReferencePictureSet->getDeltaRPS();
    TComReferencePictureSet* pcRefRPS = this->getSPS()->getRPSList()->getReferencePictureSet(rIdx);
    Int iRefPics = pcRefRPS->getNumberOfPictures();
    Int iNewIdc=0;
    for(i=0; i<= iRefPics; i++) 
    {
      Int deltaPOC = ((i != iRefPics)? pcRefRPS->getDeltaPOC(i) : 0);  // check if the reference abs POC is >= 0
      Int iRefIdc = 0;
      for (j=0; j < pcRPS->getNumberOfPictures(); j++) // loop through the  pictures in the new RPS
      {
        if ( (deltaPOC + deltaRPS) == pcRPS->getDeltaPOC(j))
        {
          if (pcRPS->getUsed(j))
          {
            iRefIdc = 1;
          }
          else
          {
            iRefIdc = 2;
          }
        }
      }
      pcRPS->setRefIdc(i, iRefIdc);
      iNewIdc++;
    }
    pcRPS->setInterRPSPrediction(true);
    pcRPS->setNumRefIdc(iNewIdc);
    pcRPS->setDeltaRPS(deltaRPS); 
    pcRPS->setDeltaRIdxMinus1(pReferencePictureSet->getDeltaRIdxMinus1() + this->getSPS()->getRPSList()->getNumberOfReferencePictureSets() - this->getRPSidx());
  }

  this->setRPS(pcRPS);
  this->setRPSidx(-1);
}
*/
//  get AC and DC values for weighted pred
// * \param *wp
// * \returns Void
//
Void  TComSlice::getWpAcDcParam(wpACDCParam *&wp)
{
  wp = m_weightACDCParam;
}

//  init AC and DC values for weighted pred
// \returns Void

Void  TComSlice::initWpAcDcParam()
{
  for(Int iComp = 0; iComp < 3; iComp++ )
  {
    m_weightACDCParam[iComp].iAC = 0;
    m_weightACDCParam[iComp].iDC = 0;
  }
}

//  get WP tables for weighted pred
// * \param RefPicList
// * \param iRefIdx
// * \param *&wpScalingParam
// * \returns Void
//
Void  TComSlice::getWpScaling( RefPicList e, Int iRefIdx, wpScalingParam *&wp )
{
  wp = m_weightPredTable[e][iRefIdx];
}

//  reset Default WP tables settings : no weight.
// * \param wpScalingParam
// * \returns Void
Void  TComSlice::resetWpScaling()
{
  for ( Int e=0 ; e<2 ; e++ )
  {
    for ( Int i=0 ; i<MAX_NUM_REF ; i++ )
    {
      for ( Int yuv=0 ; yuv<3 ; yuv++ )
      {
        wpScalingParam  *pwp = &(m_weightPredTable[e][i][yuv]);
        pwp->bPresentFlag      = false;
        pwp->uiLog2WeightDenom = 0;
        pwp->uiLog2WeightDenom = 0;
        pwp->iWeight           = 1;
        pwp->iOffset           = 0;
      }
    }
  }
}

//  init WP table
// \returns Void
Void  TComSlice::initWpScaling()
{
  for ( Int e=0 ; e<2 ; e++ )
  {
    for ( Int i=0 ; i<MAX_NUM_REF ; i++ )
    {
      for ( Int yuv=0 ; yuv<3 ; yuv++ )
      {
        wpScalingParam  *pwp = &(m_weightPredTable[e][i][yuv]);
        if ( !pwp->bPresentFlag ) 
        {
          // Inferring values not present :
          pwp->iWeight = (1 << pwp->uiLog2WeightDenom);
          pwp->iOffset = 0;
        }

        pwp->w      = pwp->iWeight;
        Int bitDepth = yuv ? g_bitDepthC : g_bitDepthY;
        pwp->o      = pwp->iOffset << (bitDepth-8);
        pwp->shift  = pwp->uiLog2WeightDenom;
        pwp->round  = (pwp->uiLog2WeightDenom>=1) ? (1 << (pwp->uiLog2WeightDenom-1)) : (0);
      }
    }
  }
}

#if REPN_FORMAT_IN_VPS
UInt TComSlice::getPicWidthInLumaSamples()
{
  TComSPS *sps = getSPS();
  TComVPS *vps = getVPS();
  UInt retVal, layerId = getLayerId();
#if O0096_REP_FORMAT_INDEX
#if R0279_REP_FORMAT_INBL
  if ( layerId == 0 || sps->getV1CompatibleSPSFlag() == 1 )
  {
#if VPS_AVC_BL_FLAG_REMOVAL
    if( layerId == 0 && vps->getNonHEVCBaseLayerFlag() )
#else
    if( layerId == 0 && vps->getAvcBaseLayerFlag() )
#endif
#else
  if ( layerId == 0 )
  {
    if( vps->getAvcBaseLayerFlag() )
#endif
    {
      retVal = vps->getVpsRepFormat(layerId)->getPicWidthVpsInLumaSamples();
    }
    else
    {
      retVal = sps->getPicWidthInLumaSamples();
    }
  }
  else
  {
    retVal = vps->getVpsRepFormat( vps->getVpsRepFormatIdx(sps->getUpdateRepFormatFlag() ? sps->getUpdateRepFormatIndex() : layerId) )->getPicWidthVpsInLumaSamples();
  }
#else
  if( ( layerId == 0 ) || sps->getUpdateRepFormatFlag() )
  {
    retVal = sps->getPicWidthInLumaSamples();
  }
  else
  {
    retVal = vps->getVpsRepFormat( vps->getVpsRepFormatIdx(layerId) )->getPicWidthVpsInLumaSamples();
  }
#endif
  return retVal;
}
UInt TComSlice::getPicHeightInLumaSamples()
{
  TComSPS *sps = getSPS();
  TComVPS *vps = getVPS();
  UInt retVal, layerId = getLayerId();
#if O0096_REP_FORMAT_INDEX
#if R0279_REP_FORMAT_INBL
  if ( layerId == 0 || sps->getV1CompatibleSPSFlag() == 1 )
  {
#if VPS_AVC_BL_FLAG_REMOVAL
    if( layerId == 0 && vps->getNonHEVCBaseLayerFlag() )
#else
    if( layerId == 0 && vps->getAvcBaseLayerFlag() )
#endif
#else
  if ( layerId == 0 )
  {
    if( vps->getAvcBaseLayerFlag() )
#endif
    {
      retVal = vps->getVpsRepFormat(layerId)->getPicHeightVpsInLumaSamples();
    }
    else
    {
      retVal = sps->getPicHeightInLumaSamples();
    }
  }
  else
  {
    retVal = vps->getVpsRepFormat( vps->getVpsRepFormatIdx(sps->getUpdateRepFormatFlag() ? sps->getUpdateRepFormatIndex() : layerId) )->getPicHeightVpsInLumaSamples();
  }
#else
  if( ( layerId == 0 ) || sps->getUpdateRepFormatFlag() )
  {
    retVal = sps->getPicHeightInLumaSamples();
  }
  else
  {
    retVal = vps->getVpsRepFormat( vps->getVpsRepFormatIdx(layerId) )->getPicHeightVpsInLumaSamples();
  }
#endif
  return retVal;
}
#if AUXILIARY_PICTURES
ChromaFormat TComSlice::getChromaFormatIdc()
#else
UInt TComSlice::getChromaFormatIdc()
#endif
{
  TComSPS *sps = getSPS();
  TComVPS *vps = getVPS();
#if AUXILIARY_PICTURES
  ChromaFormat retVal;
  UInt layerId = getLayerId();
#else
  UInt retVal, layerId = getLayerId();
#endif
#if O0096_REP_FORMAT_INDEX
#if R0279_REP_FORMAT_INBL
  if ( layerId == 0 || sps->getV1CompatibleSPSFlag() == 1 )
  {
#if VPS_AVC_BL_FLAG_REMOVAL
    if( layerId == 0 && vps->getNonHEVCBaseLayerFlag() )
#else
    if( layerId == 0 && vps->getAvcBaseLayerFlag() )
#endif
#else
  if ( layerId == 0 )
  {
    if( vps->getAvcBaseLayerFlag() )
#endif
    {
      retVal = vps->getVpsRepFormat(layerId)->getChromaFormatVpsIdc();
    }
    else
    {
      retVal = sps->getChromaFormatIdc();
    }
  }
  else
  {
    retVal = vps->getVpsRepFormat( vps->getVpsRepFormatIdx(sps->getUpdateRepFormatFlag() ? sps->getUpdateRepFormatIndex() : layerId) )->getChromaFormatVpsIdc();
  }
#else
  if( ( layerId == 0 ) || sps->getUpdateRepFormatFlag() )
  {
    retVal = sps->getChromaFormatIdc();
  }
  else
  {
    retVal = vps->getVpsRepFormat( vps->getVpsRepFormatIdx(layerId) )->getChromaFormatVpsIdc();
  }
#endif
  return retVal;
}
UInt TComSlice::getBitDepthY()
{
  TComSPS *sps = getSPS();
  TComVPS *vps = getVPS();
  UInt retVal, layerId = getLayerId();
#if O0096_REP_FORMAT_INDEX
#if R0279_REP_FORMAT_INBL
  if ( layerId == 0 || sps->getV1CompatibleSPSFlag() == 1 )
#else
  if ( layerId == 0 )
#endif
  {
    retVal = sps->getBitDepthY();
  }
  else
  {
    retVal = vps->getVpsRepFormat( vps->getVpsRepFormatIdx(sps->getUpdateRepFormatFlag() ? sps->getUpdateRepFormatIndex() : layerId) )->getBitDepthVpsLuma();
  }
#else
  if( ( layerId == 0 ) || sps->getUpdateRepFormatFlag() )
  {
    retVal = sps->getBitDepthY();
  }
  else
  {
    retVal = vps->getVpsRepFormat( vps->getVpsRepFormatIdx(layerId) )->getBitDepthVpsLuma();
  }
#endif
  return retVal;
}
UInt TComSlice::getBitDepthC()
{
  TComSPS *sps = getSPS();
  TComVPS *vps = getVPS();
  UInt retVal, layerId = getLayerId();
#if O0096_REP_FORMAT_INDEX
#if R0279_REP_FORMAT_INBL
  if ( layerId == 0 || sps->getV1CompatibleSPSFlag() == 1 )
#else
  if ( layerId == 0 )
#endif
  {
    retVal = sps->getBitDepthC();
  }
  else
  {
    retVal = vps->getVpsRepFormat( vps->getVpsRepFormatIdx(sps->getUpdateRepFormatFlag() ? sps->getUpdateRepFormatIndex() : layerId) )->getBitDepthVpsChroma();
  }
#else
  if( ( layerId == 0 ) || sps->getUpdateRepFormatFlag() )
  {
    retVal = sps->getBitDepthC();
  }
  else
  {
    retVal = vps->getVpsRepFormat( vps->getVpsRepFormatIdx(layerId) )->getBitDepthVpsChroma();
  }
#endif
  return retVal;
}
Int TComSlice::getQpBDOffsetY()
{
  return (getBitDepthY() - 8) * 6;
}
Int TComSlice::getQpBDOffsetC()
{
  return (getBitDepthC() - 8) * 6;
}

#if R0156_CONF_WINDOW_IN_REP_FORMAT
Window& TComSlice::getConformanceWindow()
{
  TComSPS *sps = getSPS();
  TComVPS *vps = getVPS();
  UInt layerId = getLayerId();
#if O0096_REP_FORMAT_INDEX
#if R0279_REP_FORMAT_INBL
  if ( layerId == 0 || sps->getV1CompatibleSPSFlag() == 1 )
  {
#if VPS_AVC_BL_FLAG_REMOVAL
    if( layerId == 0 && vps->getNonHEVCBaseLayerFlag() )
#else
    if( layerId == 0 && vps->getAvcBaseLayerFlag() )
#endif
#else
  if ( layerId == 0 )
  {
    if( vps->getAvcBaseLayerFlag() )
#endif
    {
      return vps->getVpsRepFormat(layerId)->getConformanceWindowVps();
    }
    else
    {
      return sps->getConformanceWindow();
    }
  }
  else
  {
    return vps->getVpsRepFormat( vps->getVpsRepFormatIdx(sps->getUpdateRepFormatFlag() ? sps->getUpdateRepFormatIndex() : layerId) )->getConformanceWindowVps();
  }
#else
  if( ( layerId == 0 ) || sps->getUpdateRepFormatFlag() )
  {
    return sps->getConformanceWindow();
  }
  else
  {
    return vps->getVpsRepFormat( vps->getVpsRepFormatIdx(layerId) )->getConformanceWindowVps();
  }
#endif
}
#endif

#endif

// TCH */

RepFormat::RepFormat()
#if AUXILIARY_PICTURES
: m_chromaFormatVpsIdc          (CHROMA_420)
#else
: m_chromaFormatVpsIdc          (0)
#endif
, m_separateColourPlaneVpsFlag  (false)
, m_picWidthVpsInLumaSamples    (0)
, m_picHeightVpsInLumaSamples   (0)
, m_bitDepthVpsLuma             (0)
, m_bitDepthVpsChroma           (0)
{}
#if RESOLUTION_BASED_DPB
Void RepFormat::init()
{
  m_chromaFormatVpsIdc          = CHROMA_420;
  m_separateColourPlaneVpsFlag  = false;
  m_picWidthVpsInLumaSamples    = 0;
  m_picHeightVpsInLumaSamples   = 0;
  m_bitDepthVpsLuma             = 0;
  m_bitDepthVpsChroma           = 0;
#if R0156_CONF_WINDOW_IN_REP_FORMAT
  m_conformanceWindowVps.resetWindow();
#endif
}
#endif

// ------------------------------------------------------------------------------------------------
// Video parameter set (VPS)
// ------------------------------------------------------------------------------------------------
#if SVC_EXTENSION
TComVPS::TComVPS()
: m_VPSId                     (  0)
#if VPS_RESERVED_FLAGS
, m_baseLayerInternalFlag     (true)
, m_baseLayerAvailableFlag    (true)
#endif
, m_uiMaxTLayers              (  1)
, m_uiMaxLayers               (  1)
, m_bTemporalIdNestingFlag    (false)
, m_numHrdParameters          (  0)
, m_hrdParameters             (NULL)
, m_hrdOpSetIdx               (NULL)
, m_cprmsPresentFlag          (NULL)
, m_maxLayerId                (0)
, m_numLayerSets              (0)
#if VPS_EXTN_OP_LAYER_SETS
, m_numOutputLayerSets        (0)  
#endif
, m_numProfileTierLevel       (0)
#if !VPS_EXTN_UEV_CODING
, m_moreOutputLayerSetsThanDefaultFlag (false)
#endif
, m_numAddOutputLayerSets     (0)
#if P0295_DEFAULT_OUT_LAYER_IDC
, m_defaultTargetOutputLayerIdc     (0)
#else
#if O0109_DEFAULT_ONE_OUT_LAYER_IDC
, m_defaultOneTargetOutputLayerIdc     (0)
#else
, m_defaultOneTargetOutputLayerFlag    (false)
#endif
#endif
, m_bitRatePresentVpsFlag     (false)
, m_picRatePresentVpsFlag     (false)
#if REPN_FORMAT_IN_VPS
#if Q0195_REP_FORMAT_CLEANUP
, m_repFormatIdxPresentFlag   (false)
#else
, m_repFormatIdxPresentFlag   (true)
#endif
, m_vpsNumRepFormats          (1)
#endif
#if VIEW_ID_RELATED_SIGNALING 
#if O0109_VIEW_ID_LEN
, m_viewIdLen                (0)
#else
, m_viewIdLenMinus1           (0)
#endif
#endif
#if !P0307_REMOVE_VPS_VUI_OFFSET
#if VPS_VUI_OFFSET
, m_vpsVuiOffset (0)
#endif
#endif
#if P0307_VPS_NON_VUI_EXTENSION
, m_vpsNonVuiExtLength (0)
#endif
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
, m_vpsPocLsbAlignedFlag(false)
#endif
{
  for( Int i = 0; i < MAX_TLAYER; i++)
  {
    m_numReorderPics[i] = 0;
    m_uiMaxDecPicBuffering[i] = 1; 
    m_uiMaxLatencyIncrease[i] = 0;
  }
#if VPS_EXTN_MASK_AND_DIM_INFO
#if VPS_AVC_BL_FLAG_REMOVAL
  m_nonHEVCBaseLayerFlag = false;
#else
  m_avcBaseLayerFlag = false;
#endif
  m_splittingFlag = false;
  ::memset(m_scalabilityMask, 0, sizeof(m_scalabilityMask));
  ::memset(m_dimensionIdLen, 0, sizeof(m_dimensionIdLen));
  m_nuhLayerIdPresentFlag = false;
  ::memset(m_layerIdInNuh, 0, sizeof(m_layerIdInNuh));
  ::memset(m_dimensionId, 0, sizeof(m_dimensionId));

  m_numScalabilityTypes = 0;
  ::memset(m_layerIdInVps, 0, sizeof(m_layerIdInVps));
#endif
#if VPS_EXTN_PROFILE_INFO
  ::memset(m_profilePresentFlag, 0, sizeof(m_profilePresentFlag));
#if !P0048_REMOVE_PROFILE_REF
  ::memset(m_profileLayerSetRef, 0, sizeof(m_profileLayerSetRef));
#endif
#endif
#if VPS_EXTN_OP_LAYER_SETS
  ::memset(m_layerIdIncludedFlag, 0, sizeof(m_layerIdIncludedFlag));
  // Consider dynamic allocation for outputLayerSetIdx and outputLayerFlag
  ::memset(m_outputLayerSetIdx, 0, sizeof(m_outputLayerSetIdx));
  ::memset(m_outputLayerFlag, 0, sizeof(m_outputLayerFlag));
#endif
#if VPS_EXTN_DIRECT_REF_LAYERS
  ::memset(m_directDependencyFlag, 0, sizeof(m_directDependencyFlag));
  ::memset(m_numDirectRefLayers,   0, sizeof(m_numDirectRefLayers  ));
  ::memset(m_refLayerId,           0, sizeof(m_refLayerId          ));
  m_directDepTypeLen = 2;
  ::memset(m_directDependencyType, 0, sizeof(m_directDependencyType));
#endif
#if !NECESSARY_LAYER_FLAG
#if DERIVE_LAYER_ID_LIST_VARIABLES
  ::memset(m_layerSetLayerIdList,  0, sizeof(m_layerSetLayerIdList));
  ::memset(m_numLayerInIdList,     0, sizeof(m_numLayerInIdList   )); 
#endif
#endif
#if !PER_LAYER_PTL
  ::memset(m_profileLevelTierIdx,  0, sizeof(m_profileLevelTierIdx));
#endif
  m_maxOneActiveRefLayerFlag = true;
#if O0062_POC_LSB_NOT_PRESENT_FLAG
  ::memset(m_pocLsbNotPresentFlag, 0, sizeof(m_pocLsbNotPresentFlag));
#endif
#if O0223_PICTURE_TYPES_ALIGN_FLAG
  m_crossLayerPictureTypeAlignFlag = true;
#endif 
  m_crossLayerIrapAlignFlag = true;
#if P0068_CROSS_LAYER_ALIGNED_IDR_ONLY_FOR_IRAP_FLAG
  m_crossLayerAlignedIdrOnlyFlag = false;
#endif
  m_maxTidRefPresentFlag = true;
  for( Int i = 0; i < MAX_VPS_LAYER_ID_PLUS1 - 1; i++)
  {
#if O0225_MAX_TID_FOR_REF_LAYERS
    for( Int j = 0; j < MAX_VPS_LAYER_ID_PLUS1; j++)
    {
      m_maxTidIlRefPicsPlus1[i][j] = m_uiMaxTLayers + 1;
    }
#else
    m_maxTidIlRefPicsPlus1[i] = m_uiMaxTLayers + 1;
#endif 
  }
#if VPS_VUI_TILES_NOT_IN_USE__FLAG
  m_tilesNotInUseFlag = true;
  ::memset(m_tilesInUseFlag,  0, sizeof(m_tilesInUseFlag));
  ::memset(m_loopFilterNotAcrossTilesFlag,  0, sizeof(m_loopFilterNotAcrossTilesFlag));
#endif
  ::memset(m_tileBoundariesAlignedFlag,  0, sizeof(m_tileBoundariesAlignedFlag));
#if VPS_VUI_WPP_NOT_IN_USE__FLAG
  m_wppNotInUseFlag = true;
  ::memset(m_wppInUseFlag,  0, sizeof(m_wppInUseFlag));
#endif
#if N0160_VUI_EXT_ILP_REF
  m_ilpRestrictedRefLayersFlag = false;
  ::memset(m_minSpatialSegmentOffsetPlus1,  0, sizeof(m_minSpatialSegmentOffsetPlus1));
  ::memset(m_ctuBasedOffsetEnabledFlag,     0, sizeof(m_ctuBasedOffsetEnabledFlag));
  ::memset(m_minHorizontalCtuOffsetPlus1,   0, sizeof(m_minHorizontalCtuOffsetPlus1));
#endif
#if VPS_VUI_VIDEO_SIGNAL
  m_vidSigPresentVpsFlag=true;
  m_vpsVidSigInfo=1;
  ::memset( m_vpsVidSigIdx, 0, sizeof(m_vpsVidSigIdx) );
  m_vpsVidSigIdx[0]=0;
  for (Int i=0; i < 16; i++)
  {
    m_vpsVidFormat[i] = 5;
    m_vpsFullRangeFlag[i] = false;
    m_vpsColorPrimaries[i] = 2;
    m_vpsTransChar[i] = 2;
    m_vpsMatCoeff[i] = 2;
  }
#endif
  ::memset(m_bitRatePresentFlag, 0, sizeof(m_bitRatePresentFlag));
  ::memset(m_picRatePresentFlag, 0, sizeof(m_picRatePresentFlag));
  ::memset(m_avgBitRate        , 0, sizeof(m_avgBitRate)        );
  ::memset(m_maxBitRate        , 0, sizeof(m_maxBitRate)        );
  ::memset(m_constPicRateIdc   , 0, sizeof(m_constPicRateIdc)   );
  ::memset(m_avgPicRate        , 0, sizeof(m_avgPicRate)        );
#if REPN_FORMAT_IN_VPS
  ::memset( m_vpsRepFormatIdx, 0, sizeof(m_vpsRepFormatIdx) );
#endif
#if VIEW_ID_RELATED_SIGNALING 
  ::memset(m_viewIdVal, 0, sizeof(m_viewIdVal));
#endif
#if O0092_0094_DEPENDENCY_CONSTRAINT
  for (Int i = 0; i < MAX_NUM_LAYER_IDS; i++)
  {
    m_numberRefLayers[i] = 0;
    for (Int j = 0; j < MAX_NUM_LAYER_IDS; j++)
    {
      m_recursiveRefLayerFlag[i][j] = 0;
    }
  }
#endif
#if VPS_DPB_SIZE_TABLE
  ::memset( m_subLayerFlagInfoPresentFlag,  0, sizeof(m_subLayerFlagInfoPresentFlag ) );
  ::memset( m_subLayerDpbInfoPresentFlag,   0, sizeof(m_subLayerDpbInfoPresentFlag )  );
  ::memset( m_maxVpsDecPicBufferingMinus1,  0, sizeof(m_maxVpsDecPicBufferingMinus1 ) );
#if RESOLUTION_BASED_DPB
  ::memset( m_maxVpsLayerDecPicBuffMinus1,  0, sizeof(m_maxVpsLayerDecPicBuffMinus1 ) );
#endif
  ::memset( m_maxVpsNumReorderPics,         0, sizeof(m_maxVpsNumReorderPics )        );
  ::memset( m_maxVpsLatencyIncreasePlus1,   0, sizeof(m_maxVpsLatencyIncreasePlus1 )  );
  ::memset( m_numSubDpbs                ,   0, sizeof(m_numSubDpbs)                   );
#endif
}
#else
TComVPS::TComVPS()
: m_VPSId                     (  0)
, m_uiMaxTLayers              (  1)
, m_uiMaxLayers               (  1)
, m_bTemporalIdNestingFlag    (false)
, m_numHrdParameters          (  0)
, m_maxNuhReservedZeroLayerId (  0)
, m_hrdParameters             (NULL)
, m_hrdOpSetIdx               (NULL)
, m_cprmsPresentFlag          (NULL)
{
  for( Int i = 0; i < MAX_TLAYER; i++)
  {
    m_numReorderPics[i] = 0;
    m_uiMaxDecPicBuffering[i] = 1; 
    m_uiMaxLatencyIncrease[i] = 0;
  }
}
#endif //SVC_EXTENSION

TComVPS::~TComVPS()
{
  if( m_hrdParameters    != NULL )     delete[] m_hrdParameters;
  if( m_hrdOpSetIdx      != NULL )     delete[] m_hrdOpSetIdx;
  if( m_cprmsPresentFlag != NULL )     delete[] m_cprmsPresentFlag;
}


void TComVPS::init()
{
  m_VPSId                     = 0 ;
#if VPS_RESERVED_FLAGS
  m_baseLayerInternalFlag     = true;
  m_baseLayerAvailableFlag    = true;
#endif
  m_uiMaxTLayers              =   1;
  m_uiMaxLayers               =   1;
  m_bTemporalIdNestingFlag    = false;
  m_numHrdParameters          =   0;
  m_hrdParameters             = NULL;
  m_hrdOpSetIdx               = NULL;
  m_cprmsPresentFlag          = NULL;
  m_maxLayerId                = 0;
  m_numLayerSets              = 0;
#if VPS_EXTN_OP_LAYER_SETS
  m_numOutputLayerSets        = 0;
#endif
  m_numProfileTierLevel       = 0;
#if !VPS_EXTN_UEV_CODING
  m_moreOutputLayerSetsThanDefaultFlag = false;
#endif
  m_numAddOutputLayerSets     = 0;
#if P0295_DEFAULT_OUT_LAYER_IDC
  m_defaultTargetOutputLayerIdc     = 0;
#else
#if O0109_DEFAULT_ONE_OUT_LAYER_IDC
  m_defaultOneTargetOutputLayerIdc     = 0;
#else
  m_defaultOneTargetOutputLayerFlag    = false;
#endif
#endif
  m_bitRatePresentVpsFlag     = false;
  m_picRatePresentVpsFlag     = false;
#if REPN_FORMAT_IN_VPS
#if Q0195_REP_FORMAT_CLEANUP
  m_repFormatIdxPresentFlag   = false;
#else
  m_repFormatIdxPresentFlag   = true;
#endif
  m_vpsNumRepFormats          = 1;
#endif
#if VIEW_ID_RELATED_SIGNALING
#if O0109_VIEW_ID_LEN
  m_viewIdLen                = 0;
#else
  m_viewIdLenMinus1           = 0;
#endif
#endif
#if !P0307_REMOVE_VPS_VUI_OFFSET
#if VPS_VUI_OFFSET
  m_vpsVuiOffset = 0;
#endif
#endif
#if P0307_VPS_NON_VUI_EXTENSION
  m_vpsNonVuiExtLength = 0;
#endif
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  m_vpsPocLsbAlignedFlag = false;
#endif
  for( Int i = 0; i < MAX_TLAYER; i++)
  {
    m_numReorderPics[i] = 0;
    m_uiMaxDecPicBuffering[i] = 1;
    m_uiMaxLatencyIncrease[i] = 0;
  }
#if VPS_EXTN_MASK_AND_DIM_INFO
#if VPS_AVC_BL_FLAG_REMOVAL
  m_nonHEVCBaseLayerFlag = false;
#else
  m_avcBaseLayerFlag = false;
#endif
  m_splittingFlag = false;
  ::memset(m_scalabilityMask, 0, sizeof(m_scalabilityMask));
  ::memset(m_dimensionIdLen, 0, sizeof(m_dimensionIdLen));
  m_nuhLayerIdPresentFlag = false;
  ::memset(m_layerIdInNuh, 0, sizeof(m_layerIdInNuh));
  ::memset(m_dimensionId, 0, sizeof(m_dimensionId));

  m_numScalabilityTypes = 0;
  ::memset(m_layerIdInVps, 0, sizeof(m_layerIdInVps));
#endif
#if VPS_EXTN_PROFILE_INFO
  ::memset(m_profilePresentFlag, 0, sizeof(m_profilePresentFlag));
#if !P0048_REMOVE_PROFILE_REF
  ::memset(m_profileLayerSetRef, 0, sizeof(m_profileLayerSetRef));
#endif
#endif
#if VPS_EXTN_OP_LAYER_SETS
  ::memset(m_layerIdIncludedFlag, 0, sizeof(m_layerIdIncludedFlag));
  // Consider dynamic allocation for outputLayerSetIdx and outputLayerFlag
  ::memset(m_outputLayerSetIdx, 0, sizeof(m_outputLayerSetIdx));
  ::memset(m_outputLayerFlag, 0, sizeof(m_outputLayerFlag));
#endif
#if VPS_EXTN_DIRECT_REF_LAYERS
  ::memset(m_directDependencyFlag, 0, sizeof(m_directDependencyFlag));
  ::memset(m_numDirectRefLayers,   0, sizeof(m_numDirectRefLayers  ));
  ::memset(m_refLayerId,           0, sizeof(m_refLayerId          ));
  m_directDepTypeLen = 2;
  ::memset(m_directDependencyType, 0, sizeof(m_directDependencyType));
#endif
#if !NECESSARY_LAYER_FLAG
#if DERIVE_LAYER_ID_LIST_VARIABLES
  ::memset(m_layerSetLayerIdList,  0, sizeof(m_layerSetLayerIdList));
  ::memset(m_numLayerInIdList,     0, sizeof(m_numLayerInIdList   ));
#endif
#endif
#if !PER_LAYER_PTL
  ::memset(m_profileLevelTierIdx,  0, sizeof(m_profileLevelTierIdx));
#endif
  m_maxOneActiveRefLayerFlag = true;
#if O0062_POC_LSB_NOT_PRESENT_FLAG
  ::memset(m_pocLsbNotPresentFlag, 0, sizeof(m_pocLsbNotPresentFlag));
#endif
#if O0223_PICTURE_TYPES_ALIGN_FLAG
  m_crossLayerPictureTypeAlignFlag = true;
#endif
  m_crossLayerIrapAlignFlag = true;
#if P0068_CROSS_LAYER_ALIGNED_IDR_ONLY_FOR_IRAP_FLAG
  m_crossLayerAlignedIdrOnlyFlag = false;
#endif
  m_maxTidRefPresentFlag = true;
  for( Int i = 0; i < MAX_VPS_LAYER_ID_PLUS1 - 1; i++)
  {
#if O0225_MAX_TID_FOR_REF_LAYERS
    for( Int j = 0; j < MAX_VPS_LAYER_ID_PLUS1; j++)
    {
      m_maxTidIlRefPicsPlus1[i][j] = m_uiMaxTLayers + 1;
    }
#else
    m_maxTidIlRefPicsPlus1[i] = m_uiMaxTLayers + 1;
#endif
  }
#if VPS_VUI_TILES_NOT_IN_USE__FLAG
  m_tilesNotInUseFlag = true;
  ::memset(m_tilesInUseFlag,  0, sizeof(m_tilesInUseFlag));
  ::memset(m_loopFilterNotAcrossTilesFlag,  0, sizeof(m_loopFilterNotAcrossTilesFlag));
#endif
  ::memset(m_tileBoundariesAlignedFlag,  0, sizeof(m_tileBoundariesAlignedFlag));
#if VPS_VUI_WPP_NOT_IN_USE__FLAG
  m_wppNotInUseFlag = true;
  ::memset(m_wppInUseFlag,  0, sizeof(m_wppInUseFlag));
#endif
#if N0160_VUI_EXT_ILP_REF
  m_ilpRestrictedRefLayersFlag = false;
  ::memset(m_minSpatialSegmentOffsetPlus1,  0, sizeof(m_minSpatialSegmentOffsetPlus1));
  ::memset(m_ctuBasedOffsetEnabledFlag,     0, sizeof(m_ctuBasedOffsetEnabledFlag));
  ::memset(m_minHorizontalCtuOffsetPlus1,   0, sizeof(m_minHorizontalCtuOffsetPlus1));
#endif
#if VPS_VUI_VIDEO_SIGNAL
  m_vidSigPresentVpsFlag=true;
  m_vpsVidSigInfo=1;
  ::memset( m_vpsVidSigIdx, 0, sizeof(m_vpsVidSigIdx) );
  m_vpsVidSigIdx[0]=0;
  for (Int i=0; i < 16; i++)
  {
    m_vpsVidFormat[i] = 5;
    m_vpsFullRangeFlag[i] = false;
    m_vpsColorPrimaries[i] = 2;
    m_vpsTransChar[i] = 2;
    m_vpsMatCoeff[i] = 2;
  }
#endif
  ::memset(m_bitRatePresentFlag, 0, sizeof(m_bitRatePresentFlag));
  ::memset(m_picRatePresentFlag, 0, sizeof(m_picRatePresentFlag));
  ::memset(m_avgBitRate        , 0, sizeof(m_avgBitRate)        );
  ::memset(m_maxBitRate        , 0, sizeof(m_maxBitRate)        );
  ::memset(m_constPicRateIdc   , 0, sizeof(m_constPicRateIdc)   );
  ::memset(m_avgPicRate        , 0, sizeof(m_avgPicRate)        );
#if REPN_FORMAT_IN_VPS
  ::memset( m_vpsRepFormatIdx, 0, sizeof(m_vpsRepFormatIdx) );
#endif
#if VIEW_ID_RELATED_SIGNALING
  ::memset(m_viewIdVal, 0, sizeof(m_viewIdVal));
#endif
#if O0092_0094_DEPENDENCY_CONSTRAINT
  for (Int i = 0; i < MAX_NUM_LAYER_IDS; i++)
  {
    m_numberRefLayers[i] = 0;
    for (Int j = 0; j < MAX_NUM_LAYER_IDS; j++)
    {
      m_recursiveRefLayerFlag[i][j] = 0;
    }
  }
#endif
#if VPS_DPB_SIZE_TABLE
  ::memset( m_subLayerFlagInfoPresentFlag,  0, sizeof(m_subLayerFlagInfoPresentFlag ) );
  ::memset( m_subLayerDpbInfoPresentFlag,   0, sizeof(m_subLayerDpbInfoPresentFlag )  );
  ::memset( m_maxVpsDecPicBufferingMinus1,  0, sizeof(m_maxVpsDecPicBufferingMinus1 ) );
#if RESOLUTION_BASED_DPB
  ::memset( m_maxVpsLayerDecPicBuffMinus1,  0, sizeof(m_maxVpsLayerDecPicBuffMinus1 ) );
#endif
  ::memset( m_maxVpsNumReorderPics,         0, sizeof(m_maxVpsNumReorderPics )        );
  ::memset( m_maxVpsLatencyIncreasePlus1,   0, sizeof(m_maxVpsLatencyIncreasePlus1 )  );
  ::memset( m_numSubDpbs                ,   0, sizeof(m_numSubDpbs)                   );
#endif
 m_necessaryLayerFlag.clear();
 m_numNecessaryLayers.clear();
}

#if DERIVE_LAYER_ID_LIST_VARIABLES
#if NECESSARY_LAYER_FLAG
Void TComVPS::deriveLayerIdListVariables()
{
  // For layer 0
  m_numLayerInIdList.push_back(1);
  m_layerSetLayerIdList.resize(m_numLayerSets);
  m_layerSetLayerIdList[0].push_back(0);
  
  // For other layers
  for( Int i = 1; i < (Int)m_numLayerSets; i++ )
  {
    for( Int m = 0; m <= (Int)m_maxLayerId; m++)
    {
      if( m_layerIdIncludedFlag[i][m] )
      {
        m_layerSetLayerIdList[i].push_back(m);
      }
    }
    m_numLayerInIdList.push_back(m_layerSetLayerIdList[i].size());
  }
}
#else
Void TComVPS::deriveLayerIdListVariables()
{
  // For layer 0
  m_numLayerInIdList[0] = 1;
  m_layerSetLayerIdList[0][0] = 0;
  
  // For other layers
  Int i, m, n;
  for( i = 1; i < (Int)m_numLayerSets; i++ )
  {
    n = 0;
    for( m = 0; m <= (Int)m_maxLayerId; m++)
    {
      if( m_layerIdIncludedFlag[i][m] )
      {
        m_layerSetLayerIdList[i][n++] = m;
      }
    }
    m_numLayerInIdList[i] = n;
  }
}
#endif
#endif
#if !RESOLUTION_BASED_DPB
#if VPS_DPB_SIZE_TABLE
Void TComVPS::deriveNumberOfSubDpbs()
{
  // Derive number of sub-DPBs
#if CHANGE_NUMSUBDPB_IDX
  // For layer set 0
  setNumSubDpbs(0, 1);
  // For other layer sets
  for( Int i = 1; i < (Int)getNumLayerSets(); i++)
  {
    setNumSubDpbs( i, getNumLayersInIdList( i ) );
  }
#else
  // For output layer set 0
  setNumSubDpbs(0, 1);
  // For other output layer sets
  for( Int i = 1; i < (Int)getNumOutputLayerSets(); i++)
  {
    setNumSubDpbs( i, getNumLayersInIdList( getOutputLayerSetIdx(i)) );
  }
#endif
}
#endif
#endif
#if VPS_VUI_TILES_NOT_IN_USE__FLAG
Void TComVPS::setTilesNotInUseFlag(Bool x)
{
  m_tilesNotInUseFlag = x;
  if (m_tilesNotInUseFlag)
  {
    for (int i = 0; i < (Int)getMaxLayers(); i++)
    {
      m_tilesInUseFlag[i] = m_loopFilterNotAcrossTilesFlag[i] = m_tilesNotInUseFlag;
    }
  }

  if (m_tilesNotInUseFlag)
  {
    for (int i = 1; i < (Int)getMaxLayers(); i++)
    {
      for(int j = 0; j < (Int)getNumDirectRefLayers(getLayerIdInNuh(i)); j++)
      {
        setTileBoundariesAlignedFlag(i, j, m_tilesNotInUseFlag);
      }
    }
  }
}
#endif

#if VPS_VUI_WPP_NOT_IN_USE__FLAG
Void TComVPS::setWppNotInUseFlag(Bool x)
{
  m_wppNotInUseFlag = x;
  if (m_wppNotInUseFlag)
  {
    for (int i = 0; i < (Int)getMaxLayers(); i++)
    {
      m_wppInUseFlag[i] = m_wppNotInUseFlag;
    }
  }
}
#endif
#if O0092_0094_DEPENDENCY_CONSTRAINT
Void TComVPS::setRefLayersFlags(Int currLayerId)
{
  for (Int i = 0; i < (Int)m_numDirectRefLayers[currLayerId]; i++)
  {
    UInt refLayerId = getRefLayerId(currLayerId, i);
    m_recursiveRefLayerFlag[currLayerId][refLayerId] = true;
    for (Int k = 0; k < MAX_NUM_LAYER_IDS; k++)
    {
      m_recursiveRefLayerFlag[currLayerId][k] = m_recursiveRefLayerFlag[currLayerId][k] | m_recursiveRefLayerFlag[refLayerId][k];
    }
  }
}

Void TComVPS::setNumRefLayers()
{
  memset( m_numberRefLayers, 0, sizeof( m_numberRefLayers ) );

  for (Int i = 0; i < (Int)m_uiMaxLayers; i++)
  {
    UInt iNuhLId = m_layerIdInNuh[i];
    setRefLayersFlags(iNuhLId);
    for (UInt j = 0; j < MAX_NUM_LAYER_IDS; j++)
    {
      m_numberRefLayers[iNuhLId] += (m_recursiveRefLayerFlag[iNuhLId][j] == true ? 1 : 0);
    }
  }
}
#endif

#if Q0078_ADD_LAYER_SETS
Void TComVPS::setPredictedLayerIds()
{
  for (UInt i = 0; i < m_uiMaxLayers - 1; i++)
  {
    UInt iNuhLId = m_layerIdInNuh[i];
    UInt predIdx = 0;
    for (UInt j = iNuhLId + 1; j < MAX_NUM_LAYER_IDS; j++)
    {
      if( m_recursiveRefLayerFlag[j][iNuhLId] )
      {
        m_predictedLayerId[iNuhLId][predIdx] = j;
        predIdx++;
      }
    }
    m_numPredictedLayers[iNuhLId] = predIdx;
  }
  m_numPredictedLayers[m_layerIdInNuh[m_uiMaxLayers-1]] = 0;
}

Void TComVPS::setTreePartitionLayerIdList()
{
  Bool countedLayerIdxFlag[MAX_NUM_LAYER_IDS];
  memset( countedLayerIdxFlag, 0, sizeof(countedLayerIdxFlag) );

  Int numIndependentLayers = 0;

  for (UInt i = 0; i < m_uiMaxLayers; i++)
  {
    UInt iNuhLId = m_layerIdInNuh[i];
    if( m_numDirectRefLayers[iNuhLId] == 0 )
    {
      m_treePartitionLayerIdList[numIndependentLayers][0] = iNuhLId;
      m_numLayersInTreePartition[numIndependentLayers] = 1;
      for( UInt j = 0; j < m_numPredictedLayers[iNuhLId]; j++ )
      {
        if( !countedLayerIdxFlag[m_layerIdInVps[iNuhLId]] )
        {
          m_treePartitionLayerIdList[numIndependentLayers][m_numLayersInTreePartition[numIndependentLayers]] = m_predictedLayerId[iNuhLId][j];
          m_numLayersInTreePartition[numIndependentLayers] = m_numLayersInTreePartition[numIndependentLayers] + 1;
          countedLayerIdxFlag[m_layerIdInVps[m_predictedLayerId[iNuhLId][j]]] = true;
        }
      }
      numIndependentLayers++;
    }
  }

  m_numIndependentLayers = numIndependentLayers;
}

void TComVPS::setLayerIdIncludedFlagsForAddLayerSets()
{
  for (UInt i = 0; i < (UInt)m_numAddLayerSets; i++)
  {
    for (UInt j = 1; j < (UInt)m_numIndependentLayers; j++)
    {
      Int layerNum = 0;
      Int lsIdx = m_vpsNumLayerSetsMinus1 + 1 + i;
      for (Int layerId = 0; layerId < MAX_VPS_LAYER_ID_PLUS1; layerId++)
      {
        m_layerIdIncludedFlag[lsIdx][layerId] = false;
      }
      for (Int treeIdx = 1; treeIdx < (Int)m_numIndependentLayers; treeIdx++)
      {
        for (Int layerCnt = 0; layerCnt < (Int)m_highestLayerIdxPlus1[i][j]; layerCnt++)
        {
          m_layerSetLayerIdList[lsIdx][layerNum] = m_treePartitionLayerIdList[treeIdx][layerCnt];
          m_layerIdIncludedFlag[lsIdx][m_treePartitionLayerIdList[treeIdx][layerCnt]] = true;
          layerNum++;
        }
      }
      m_numLayerInIdList[lsIdx] = layerNum;
    }
  }
}

#endif

#if VIEW_ID_RELATED_SIGNALING 
Int TComVPS::getNumViews()
{
  Int numViews = 1; 
  for( Int i = 0; i <= (Int)getMaxLayers() - 1; i++ )
  {
    Int lId = getLayerIdInNuh( i ); 
    if ( i > 0 && ( getViewIndex( lId ) != getScalabilityId( i - 1, VIEW_ORDER_INDEX ) ) )
    {
      numViews++; 
    }    
  }

  return numViews;
}
Int TComVPS::getScalabilityId( Int layerIdInVps, ScalabilityType scalType )
{
  return getScalabilityMask( scalType ) ? getDimensionId( layerIdInVps, scalTypeToScalIdx( scalType ) ) : 0;
}  
Int TComVPS::scalTypeToScalIdx( ScalabilityType scalType )
{
  assert( scalType >= 0 && scalType <= MAX_VPS_NUM_SCALABILITY_TYPES ); 
  assert( scalType == MAX_VPS_NUM_SCALABILITY_TYPES || getScalabilityMask( scalType ) );
  Int scalIdx = 0; 
  for( Int curScalType = 0; curScalType < scalType; curScalType++ )
  {
    scalIdx += ( getScalabilityMask( curScalType ) ? 1 : 0 );

  }

  return scalIdx; 
}
#endif
#if VPS_DPB_SIZE_TABLE
Void TComVPS::determineSubDpbInfoFlags()
{
  for(Int i = 1; i < (Int)getNumOutputLayerSets(); i++)
  {
    Int layerSetIdxForOutputLayerSet = getOutputLayerSetIdx( i );
    // For each output layer set, set the DPB size for each layer and the reorder/latency value the maximum for all layers
    Bool checkFlagOuter = false;      // Used to calculate sub_layer_flag_info_present_flag
    Bool checkFlagInner[MAX_TLAYER];  // Used to calculate sub_layer_dpb_info_present_flag

    for(Int j = 0; j < (Int)getMaxTLayers(); j++)
    {
      // --------------------------------------------------------
      // To determine value of m_subLayerDpbInfoPresentFlag
      // --------------------------------------------------------
      if( j == 0 )  // checkFlagInner[0] is always 1
      {
        checkFlagInner[j] = true;     // Always signal sub-layer DPB information for the first sub-layer
      }
      else
      {
        checkFlagInner[j] = false;    // Initialize to be false. If the values of the current sub-layers matches with the earlier sub-layer, 
                                      // then will be continue to be false - i.e. the j-th sub-layer DPB info is not signaled
        checkFlagInner[j] |= ( getMaxVpsNumReorderPics(i, j) != getMaxVpsNumReorderPics(i, j - 1) );
#if CHANGE_NUMSUBDPB_IDX
        for(Int subDpbIdx = 0; subDpbIdx < getNumSubDpbs(layerSetIdxForOutputLayerSet) && !checkFlagInner[j]; subDpbIdx++)  // If checkFlagInner[j] is true, break and signal the values
#else
        for(Int k = 0; k < getNumSubDpbs(i) && !checkFlagInner[j]; k++)  // If checkFlagInner[j] is true, break and signal the values
#endif
        {
          checkFlagInner[j] |= ( getMaxVpsDecPicBufferingMinus1(i, subDpbIdx, j - 1) != getMaxVpsDecPicBufferingMinus1(i, subDpbIdx, j) );
        }
#if RESOLUTION_BASED_DPB
        for(Int layerIdx = 0; layerIdx < this->getNumLayersInIdList(layerSetIdxForOutputLayerSet) && !checkFlagInner[j]; layerIdx++)  // If checkFlagInner[j] is true, break and signal the values
        {
          checkFlagInner[j] |= ( getMaxVpsLayerDecPicBuffMinus1(i, layerIdx, j - 1) != getMaxVpsLayerDecPicBuffMinus1(i, layerIdx, j) );
        }
#endif
      }
      // If checkFlagInner[j] = true, then some value needs to be signalled for the j-th sub-layer
      setSubLayerDpbInfoPresentFlag( i, j, checkFlagInner[j] );
    }

    // --------------------------------------------------------
    // To determine value of m_subLayerFlagInfoPresentFlag
    // --------------------------------------------------------

    for(Int j = 1; j < (Int)getMaxTLayers(); j++) // Check if DPB info of any of non-zero sub-layers is signaled. If so set flag to one
    {
      if( getSubLayerDpbInfoPresentFlag(i, j) )
      {
        checkFlagOuter = true;
        break;
      }
    }
    setSubLayerFlagInfoPresentFlag( i, checkFlagOuter );
  }
}
#endif
#if RESOLUTION_BASED_DPB
Void TComVPS::assignSubDpbIndices()
{
  RepFormat layerRepFormat  [MAX_LAYERS];
  RepFormat subDpbRepFormat [MAX_LAYERS];

  for(Int lsIdx = 0; lsIdx < (Int)this->getNumLayerSets(); lsIdx++)
  {
    for(Int j = 0; j < MAX_LAYERS; j++)
    {
      layerRepFormat [j].init();
      subDpbRepFormat[j].init();
    }

    // Assign resolution, bit-depth, colour format for each layer in the layer set
    for(Int i = 0; i < this->getNumLayersInIdList( lsIdx ); i++)
    {
      Int layerIdxInVps = this->getLayerIdInVps( this->getLayerSetLayerIdList(lsIdx, i) );
      Int repFormatIdx  = this->getVpsRepFormatIdx( layerIdxInVps );
      RepFormat* repFormat = this->getVpsRepFormat( repFormatIdx );

      // Assign the rep_format() to the layer
      layerRepFormat[i] = *repFormat;
    }

    // ----------------------------------------
    // Sub-DPB assignment
    // ----------------------------------------
    // For the base layer 
    m_subDpbAssigned[lsIdx][0] = 0;
    subDpbRepFormat[0] = layerRepFormat[0];

    // Sub-DPB counter
    Int subDpbCtr = 1;

    for(Int i = 1; i < this->getNumLayersInIdList( lsIdx ); i++)
    {
      Bool newSubDpbFlag = true;
      for(Int j = 0; (j < subDpbCtr) && (newSubDpbFlag); j++)
      {
        if( RepFormat::checkSameSubDpb( layerRepFormat[i], subDpbRepFormat[j] ) )
        {
          // Belong to i-th sub-DPB
          m_subDpbAssigned[lsIdx][i] = j;
          newSubDpbFlag = false;
        }
      }
      if( newSubDpbFlag )
      {
        // New sub-DPB
        subDpbRepFormat[subDpbCtr] = layerRepFormat[i];
        m_subDpbAssigned[lsIdx][i] = subDpbCtr;
        subDpbCtr++;                                    // Increment # subDpbs
      }
    }
    m_numSubDpbs[lsIdx] = subDpbCtr;
  }
}
Int  TComVPS::findLayerIdxInLayerSet ( Int lsIdx, Int nuhLayerId )
{
  for(Int i = 0; i < this->getNumLayersInIdList(lsIdx); i++)
  {
    if( this->getLayerSetLayerIdList( lsIdx, i) == nuhLayerId )
    {
      return i;
    }
  }
  return -1;  // Layer not found
}
#endif
#if O0164_MULTI_LAYER_HRD
Void TComVPS::setBspHrdParameters( UInt hrdIdx, UInt frameRate, UInt numDU, UInt bitRate, Bool randomAccess )
{
  if( !getVpsVuiBspHrdPresentFlag() )
  {
    return;
  }

  TComHRD *hrd = getBspHrd(hrdIdx);

  Bool rateCnt = ( bitRate > 0 );
  hrd->setNalHrdParametersPresentFlag( rateCnt );
  hrd->setVclHrdParametersPresentFlag( rateCnt );

  hrd->setSubPicCpbParamsPresentFlag( ( numDU > 1 ) );

  if( hrd->getSubPicCpbParamsPresentFlag() )
  {
    hrd->setTickDivisorMinus2( 100 - 2 );                          // 
    hrd->setDuCpbRemovalDelayLengthMinus1( 7 );                    // 8-bit precision ( plus 1 for last DU in AU )
    hrd->setSubPicCpbParamsInPicTimingSEIFlag( true );
    hrd->setDpbOutputDelayDuLengthMinus1( 5 + 7 );                 // With sub-clock tick factor of 100, at least 7 bits to have the same value as AU dpb delay
  }
  else
  {
    hrd->setSubPicCpbParamsInPicTimingSEIFlag( false );  
  }

  hrd->setBitRateScale( 4 );                                       // in units of 2~( 6 + 4 ) = 1,024 bps
  hrd->setCpbSizeScale( 6 );                                       // in units of 2~( 4 + 4 ) = 1,024 bit
  hrd->setDuCpbSizeScale( 6 );                                       // in units of 2~( 4 + 4 ) = 1,024 bit

  hrd->setInitialCpbRemovalDelayLengthMinus1(15);                  // assuming 0.5 sec, log2( 90,000 * 0.5 ) = 16-bit
  if( randomAccess )
  {
    hrd->setCpbRemovalDelayLengthMinus1(5);                        // 32 = 2^5 (plus 1)
    hrd->setDpbOutputDelayLengthMinus1 (5);                        // 32 + 3 = 2^6
  }
  else
  {
    hrd->setCpbRemovalDelayLengthMinus1(9);                        // max. 2^10
    hrd->setDpbOutputDelayLengthMinus1 (9);                        // max. 2^10
  }
  // Note: only the case of "vps_max_temporal_layers_minus1 = 0" is supported.

  Int i, j;
  UInt birateValue, cpbSizeValue;
  UInt ducpbSizeValue;
  UInt duBitRateValue = 0;

  for( i = 0; i < MAX_TLAYER; i ++ )
  {
    hrd->setFixedPicRateFlag( i, 1 );
    hrd->setPicDurationInTcMinus1( i, 0 );
    hrd->setLowDelayHrdFlag( i, 0 );
    hrd->setCpbCntMinus1( i, 0 );

    birateValue  = bitRate;
    cpbSizeValue = bitRate;                                     // 1 second
    ducpbSizeValue = bitRate/numDU;
    duBitRateValue = bitRate;
    for( j = 0; j < ( hrd->getCpbCntMinus1( i ) + 1 ); j ++ )
    {
      hrd->setBitRateValueMinus1( i, j, 0, ( birateValue  - 1 ) );
      hrd->setCpbSizeValueMinus1( i, j, 0, ( cpbSizeValue - 1 ) );
      hrd->setDuCpbSizeValueMinus1( i, j, 0, ( ducpbSizeValue - 1 ) );
      hrd->setCbrFlag( i, j, 0, ( j == 0 ) );

      hrd->setBitRateValueMinus1( i, j, 1, ( birateValue  - 1) );
      hrd->setCpbSizeValueMinus1( i, j, 1, ( cpbSizeValue - 1 ) );
      hrd->setDuCpbSizeValueMinus1( i, j, 1, ( ducpbSizeValue - 1 ) );
      hrd->setDuBitRateValueMinus1( i, j, 1, ( duBitRateValue - 1 ) );
      hrd->setCbrFlag( i, j, 1, ( j == 0 ) );
    }
  }
}
#endif
#if NECESSARY_LAYER_FLAG
Void TComVPS::deriveNecessaryLayerFlag()
{
  m_necessaryLayerFlag.empty();
  m_numNecessaryLayers.empty();

  // Assumed that output layer sets and variables RecursiveRefLayer are already derived
  for( Int olsIdx = 0; olsIdx < (Int)getNumOutputLayerSets(); olsIdx++)
  {
    deriveNecessaryLayerFlag(olsIdx);
  }
}
Void TComVPS::deriveNecessaryLayerFlag(Int const olsIdx)
{
  Int lsIdx = this->getOutputLayerSetIdx( olsIdx );
  Int numLayersInLs = this->getNumLayersInIdList( lsIdx );
  printf("JR: m_necessaryLayerFlag.size() = %d  olsIdx = %d  \n",(Int)m_necessaryLayerFlag.size(), olsIdx);
  assert( (Int)m_necessaryLayerFlag.size() == olsIdx );   // Function should be called in the correct order.
  m_necessaryLayerFlag.push_back( std::vector<Bool>( numLayersInLs, false ) ); // Initialize to false
  for( Int lsLayerIdx = 0; lsLayerIdx < numLayersInLs; lsLayerIdx++ )
  {
    if( this->m_outputLayerFlag[olsIdx][lsLayerIdx] )
    {
      m_necessaryLayerFlag[olsIdx][lsLayerIdx] = true;
      Int currNuhLayerId = this->m_layerSetLayerIdList[lsIdx][lsLayerIdx];
      for( Int rLsLayerIdx = 0; rLsLayerIdx < lsLayerIdx; rLsLayerIdx++ )
      {
        Int refNuhLayerId = this->m_layerSetLayerIdList[lsIdx][rLsLayerIdx];
        if( this->m_recursiveRefLayerFlag[currNuhLayerId][refNuhLayerId] )
        {
          m_necessaryLayerFlag[olsIdx][rLsLayerIdx] = true;
        }
      }
    }
  }
  m_numNecessaryLayers.push_back(std::accumulate(m_necessaryLayerFlag[olsIdx].begin(), m_necessaryLayerFlag[olsIdx].end(), 0));
}
Void TComVPS::checkNecessaryLayerFlagCondition()
{
  // It is a requirement of bitstream conformance that for each layer index layerIdx in the range of
  // ( vps_base_layer_internal_flag ? 0 : 1 ) to MaxLayersMinus1, inclusive, there shall be at least one OLS with index olsIdx such that
  // NecessaryLayerFlag[ olsIdx ][ lsLayerIdx ] is equal to 1 for the value of lsLayerIdx
  // for which LayerSetLayerIdList[ OlsIdxToLsIdx[ olsIdx ] ][ lsLayerIdx ] is equal to layer_id_in_nuh[ layerIdx ].
  for(Int layerIdx = this->getBaseLayerInternalFlag() ? 0 : 1; layerIdx < (Int)this->getMaxLayers(); layerIdx++)
  {
    Bool layerFoundNecessaryLayerFlag = false;
    for(Int olsIdx = 0; olsIdx < (Int)this->getNumOutputLayerSets(); olsIdx++)
    {
      Int lsIdx = this->getOutputLayerSetIdx( olsIdx );
      Int currNuhLayerId = this->getLayerIdInNuh( layerIdx );
      std::vector<Int>::iterator iter = std::find( m_layerSetLayerIdList[lsIdx].begin(), m_layerSetLayerIdList[lsIdx].end(), currNuhLayerId );
      if( iter != m_layerSetLayerIdList[lsIdx].end() ) // Layer present in layer set
      {
        size_t positionLayer = iter - m_layerSetLayerIdList[lsIdx].begin();
        if( *(m_necessaryLayerFlag[olsIdx].begin() + positionLayer) == true )
        {
          layerFoundNecessaryLayerFlag = true;
          break;
        }
      }
    }
    assert( layerFoundNecessaryLayerFlag );
  }
}
#endif
#if PER_LAYER_PTL
Int TComVPS::calculateLenOfSyntaxElement( Int const numVal )
{
  Int numBits = 1;
  while((1 << numBits) < numVal)
  {
    numBits++;
  }
  return numBits;
}
#endif
#if SUB_LAYERS_IN_LAYER_SET
Void TComVPS::calculateMaxSLInLayerSets()
{
  for(Int lsIdx = 0; lsIdx < (Int)getNumLayerSets(); lsIdx++)
  {
    UInt maxSLMinus1 = 0;
    for(Int k = 0; k < getNumLayersInIdList(lsIdx); k++ ) {
      Int  lId = getLayerSetLayerIdList(lsIdx, k);
      maxSLMinus1 = std::max(maxSLMinus1, getMaxTSLayersMinus1(getLayerIdInVps(lId)));
    }
    setMaxSLayersInLayerSetMinus1(lsIdx,maxSLMinus1);
  }
}
#endif

#if RESOLUTION_BASED_DPB
// RepFormat Assignment operator
RepFormat& RepFormat::operator= (const RepFormat &other)
{
  if( this != &other)
  {
    m_chromaAndBitDepthVpsPresentFlag = other.m_chromaAndBitDepthVpsPresentFlag;
    m_chromaFormatVpsIdc              = other.m_chromaFormatVpsIdc;
    m_separateColourPlaneVpsFlag      = other.m_separateColourPlaneVpsFlag;
    m_picWidthVpsInLumaSamples        = other.m_picWidthVpsInLumaSamples;
    m_picHeightVpsInLumaSamples       = other.m_picHeightVpsInLumaSamples;
    m_bitDepthVpsLuma                 = other.m_bitDepthVpsLuma;
    m_bitDepthVpsChroma               = other.m_bitDepthVpsChroma;
#if R0156_CONF_WINDOW_IN_REP_FORMAT
    m_conformanceWindowVps            = other.m_conformanceWindowVps;
#endif
  }
  return *this;
}

// Check whether x and y share the same resolution, chroma format and bit-depth.
Bool RepFormat::checkSameSubDpb(const RepFormat &x, const RepFormat &y)
{
  return (    (x.m_chromaFormatVpsIdc              == y.m_chromaFormatVpsIdc)
          &&  (x.m_picWidthVpsInLumaSamples        == y.m_picWidthVpsInLumaSamples)
          &&  (x.m_picHeightVpsInLumaSamples       == y.m_picHeightVpsInLumaSamples)
          &&  (x.m_bitDepthVpsLuma                 == y.m_bitDepthVpsLuma)
          &&  (x.m_bitDepthVpsChroma               == y.m_bitDepthVpsChroma)
          );
}
#endif
// ------------------------------------------------------------------------------------------------
// Sequence parameter set (SPS)
// ------------------------------------------------------------------------------------------------

TComSPS::TComSPS()
: m_SPSId                     (  0)
, m_VPSId                     (  0)
, m_chromaFormatIdc           (CHROMA_420)
, m_uiMaxTLayers              (  1)
#if R0279_REP_FORMAT_INBL
, m_bV1CompatibleSPSFlag      (  0)
#endif
// Structure
, m_picWidthInLumaSamples     (352)
, m_picHeightInLumaSamples    (288)
, m_log2MinCodingBlockSize    (  0)
, m_log2DiffMaxMinCodingBlockSize (0)
, m_uiMaxCUWidth              ( 32)
, m_uiMaxCUHeight             ( 32)
, m_uiMaxCUDepth              (  3)
, m_bLongTermRefsPresent      (false)
, m_uiQuadtreeTULog2MaxSize   (  0)
, m_uiQuadtreeTULog2MinSize   (  0)
, m_uiQuadtreeTUMaxDepthInter (  0)
, m_uiQuadtreeTUMaxDepthIntra (  0)
// Tool list
, m_usePCM                   (false)
, m_pcmLog2MaxSize            (  5)
, m_uiPCMLog2MinSize          (  7)
, m_bitDepthY                 (  8)
, m_bitDepthC                 (  8)
, m_qpBDOffsetY               (  0)
, m_qpBDOffsetC               (  0)
, m_uiPCMBitDepthLuma         (  8)
, m_uiPCMBitDepthChroma       (  8)
, m_bPCMFilterDisableFlag     (false)
, m_uiBitsForPOC              (  8)
, m_numLongTermRefPicSPS    (  0)  
, m_uiMaxTrSize               ( 32)
, m_bUseSAO                   (false) 
, m_bTemporalIdNestingFlag    (false)
, m_scalingListEnabledFlag    (false)
, m_useStrongIntraSmoothing   (false)
, m_vuiParametersPresentFlag  (false)
, m_vuiParameters             ()
#if SVC_EXTENSION
, m_layerId                   ( 0 )
, m_extensionFlag             ( false )
#if !MOVE_SCALED_OFFSET_TO_PPS
, m_numScaledRefLayerOffsets  ( 0 )
#endif
#if REPN_FORMAT_IN_VPS
, m_updateRepFormatFlag       (false)
#if O0096_REP_FORMAT_INDEX
, m_updateRepFormatIndex      (0)
#endif
#endif
#if SCALINGLIST_INFERRING
, m_inferScalingListFlag ( false )
, m_scalingListRefLayerId ( 0 )
#endif
#endif //SVC_EXTENSION
{
  for ( Int i = 0; i < MAX_TLAYER; i++ )
  {
    m_uiMaxLatencyIncrease[i] = 0;
    m_uiMaxDecPicBuffering[i] = 1;
    m_numReorderPics[i]       = 0;
  }
  m_scalingList = new TComScalingList;
  ::memset(m_ltRefPicPocLsbSps, 0, sizeof(m_ltRefPicPocLsbSps));
  ::memset(m_usedByCurrPicLtSPSFlag, 0, sizeof(m_usedByCurrPicLtSPSFlag));

#if !MOVE_SCALED_OFFSET_TO_PPS
#if P0312_VERT_PHASE_ADJ 
  ::memset(m_vertPhasePositionEnableFlag, 0, sizeof(m_vertPhasePositionEnableFlag));
#endif
#endif
}

TComSPS::~TComSPS()
{
#if SCALINGLIST_INFERRING
  if( !m_inferScalingListFlag )
#endif
  delete m_scalingList;
  m_RPSList.destroy();
}

Void  TComSPS::createRPSList( Int numRPS )
{ 
  m_RPSList.destroy();
  m_RPSList.create(numRPS);
}

Void TComSPS::setHrdParameters( UInt frameRate, UInt numDU, UInt bitRate, Bool randomAccess )
{
  if( !getVuiParametersPresentFlag() )
  {
    return;
  }

  TComVUI *vui = getVuiParameters();
  TComHRD *hrd = vui->getHrdParameters();

  TimingInfo *timingInfo = vui->getTimingInfo();
#if SVC_EXTENSION
  if( m_layerId > 0 )
  {
    timingInfo->setTimingInfoPresentFlag( false );
    return;
  }
#endif

  timingInfo->setTimingInfoPresentFlag( true );
  switch( frameRate )
  {
  case 24:
    timingInfo->setNumUnitsInTick( 1125000 );    timingInfo->setTimeScale    ( 27000000 );
    break;
  case 25:
    timingInfo->setNumUnitsInTick( 1080000 );    timingInfo->setTimeScale    ( 27000000 );
    break;
  case 30:
    timingInfo->setNumUnitsInTick( 900900 );     timingInfo->setTimeScale    ( 27000000 );
    break;
  case 50:
    timingInfo->setNumUnitsInTick( 540000 );     timingInfo->setTimeScale    ( 27000000 );
    break;
  case 60:
    timingInfo->setNumUnitsInTick( 450450 );     timingInfo->setTimeScale    ( 27000000 );
    break;
  default:
    timingInfo->setNumUnitsInTick( 1001 );       timingInfo->setTimeScale    ( 60000 );
    break;
  }

  Bool rateCnt = ( bitRate > 0 );
  hrd->setNalHrdParametersPresentFlag( rateCnt );
  hrd->setVclHrdParametersPresentFlag( rateCnt );

  hrd->setSubPicCpbParamsPresentFlag( ( numDU > 1 ) );

  if( hrd->getSubPicCpbParamsPresentFlag() )
  {
    hrd->setTickDivisorMinus2( 100 - 2 );                          // 
    hrd->setDuCpbRemovalDelayLengthMinus1( 7 );                    // 8-bit precision ( plus 1 for last DU in AU )
    hrd->setSubPicCpbParamsInPicTimingSEIFlag( true );
    hrd->setDpbOutputDelayDuLengthMinus1( 5 + 7 );                 // With sub-clock tick factor of 100, at least 7 bits to have the same value as AU dpb delay
  }
  else
  {
    hrd->setSubPicCpbParamsInPicTimingSEIFlag( false );  
  }

  hrd->setBitRateScale( 4 );                                       // in units of 2~( 6 + 4 ) = 1,024 bps
  hrd->setCpbSizeScale( 6 );                                       // in units of 2~( 4 + 4 ) = 1,024 bit
  hrd->setDuCpbSizeScale( 6 );                                       // in units of 2~( 4 + 4 ) = 1,024 bit

  hrd->setInitialCpbRemovalDelayLengthMinus1(15);                  // assuming 0.5 sec, log2( 90,000 * 0.5 ) = 16-bit
  if( randomAccess )
  {
    hrd->setCpbRemovalDelayLengthMinus1(5);                        // 32 = 2^5 (plus 1)
    hrd->setDpbOutputDelayLengthMinus1 (5);                        // 32 + 3 = 2^6
  }
  else
  {
    hrd->setCpbRemovalDelayLengthMinus1(9);                        // max. 2^10
    hrd->setDpbOutputDelayLengthMinus1 (9);                        // max. 2^10
  }


  //Note: only the case of "vps_max_temporal_layers_minus1 = 0" is supported.

  Int i, j;
  UInt birateValue, cpbSizeValue;
  UInt ducpbSizeValue;
  UInt duBitRateValue = 0;

  for( i = 0; i < MAX_TLAYER; i ++ )
  {
    hrd->setFixedPicRateFlag( i, 1 );
    hrd->setPicDurationInTcMinus1( i, 0 );
    hrd->setLowDelayHrdFlag( i, 0 );
    hrd->setCpbCntMinus1( i, 0 );

    birateValue  = bitRate;
    cpbSizeValue = bitRate;                                     // 1 second
    ducpbSizeValue = bitRate/numDU;
    duBitRateValue = bitRate;
    for( j = 0; j < ( hrd->getCpbCntMinus1( i ) + 1 ); j ++ )
    {
      hrd->setBitRateValueMinus1( i, j, 0, ( birateValue  - 1 ) );
      hrd->setCpbSizeValueMinus1( i, j, 0, ( cpbSizeValue - 1 ) );
      hrd->setDuCpbSizeValueMinus1( i, j, 0, ( ducpbSizeValue - 1 ) );
      hrd->setCbrFlag( i, j, 0, ( j == 0 ) );

      hrd->setBitRateValueMinus1( i, j, 1, ( birateValue  - 1) );
      hrd->setCpbSizeValueMinus1( i, j, 1, ( cpbSizeValue - 1 ) );
      hrd->setDuCpbSizeValueMinus1( i, j, 1, ( ducpbSizeValue - 1 ) );
      hrd->setDuBitRateValueMinus1( i, j, 1, ( duBitRateValue - 1 ) );
      hrd->setCbrFlag( i, j, 1, ( j == 0 ) );
    }
  }
}
const Int TComSPS::m_winUnitX[]={1,2,2,1};
const Int TComSPS::m_winUnitY[]={1,2,1,1};

#if !MOVE_SCALED_OFFSET_TO_PPS
#if O0098_SCALED_REF_LAYER_ID
Window& TComSPS::getScaledRefLayerWindowForLayer(Int layerId)
{
  static Window win;

  for (Int i = 0; i < m_numScaledRefLayerOffsets; i++)
  {
    if (layerId == m_scaledRefLayerId[i])
    {
      return m_scaledRefLayerWindow[i];
    }
  }

  win.resetWindow();  // scaled reference layer offsets are inferred to be zero when not present
  return win;
}
#endif
#if REF_REGION_OFFSET
Window& TComSPS::getRefLayerWindowForLayer(Int layerId)
{
  static Window win;

  for (Int i = 0; i < m_numRefLayerOffsets; i++)
  {
    if (layerId == m_refLayerId[i])
    {
      return m_refLayerWindow[i];
    }
  }

  win.resetWindow();  // reference region offsets are inferred to be zero when not present
  return win;
}
#endif
#endif

TComPPS::TComPPS()
: m_PPSId                       (0)
, m_SPSId                       (0)
, m_picInitQPMinus26            (0)
, m_useDQP                      (false)
, m_bConstrainedIntraPred       (false)
, m_bSliceChromaQpFlag          (false)
, m_pcSPS                       (NULL)
, m_uiMaxCuDQPDepth             (0)
, m_uiMinCuDQPSize              (0)
, m_chromaCbQpOffset            (0)
, m_chromaCrQpOffset            (0)
, m_numRefIdxL0DefaultActive    (1)
, m_numRefIdxL1DefaultActive    (1)
, m_TransquantBypassEnableFlag  (false)
, m_useTransformSkip             (false)
, m_dependentSliceSegmentsEnabledFlag    (false)
, m_tilesEnabledFlag               (false)
, m_entropyCodingSyncEnabledFlag   (false)
, m_loopFilterAcrossTilesEnabledFlag  (true)
, m_uniformSpacingFlag           (false)
, m_numTileColumnsMinus1         (0)
, m_numTileRowsMinus1            (0)
, m_numSubstreams               (1)
, m_signHideFlag(0)
, m_cabacInitPresentFlag        (false)
, m_encCABACTableIdx            (I_SLICE)
, m_sliceHeaderExtensionPresentFlag    (false)
, m_loopFilterAcrossSlicesEnabledFlag (false)
, m_listsModificationPresentFlag(  0)
, m_numExtraSliceHeaderBits(0)
#if SVC_EXTENSION
, m_extensionFlag(false)
#if SCALINGLIST_INFERRING
, m_layerId(0)
, m_inferScalingListFlag ( false )
, m_scalingListRefLayerId ( 0 )
#endif
#if POC_RESET_IDC
, m_pocResetInfoPresentFlag   (false)
#endif
#if MOVE_SCALED_OFFSET_TO_PPS
, m_numScaledRefLayerOffsets  ( 0 )
#endif
#if Q0048_CGS_3D_ASYMLUT
, m_nCGSFlag(0)
, m_nCGSOutputBitDepthY(0)
, m_nCGSOutputBitDepthC(0)
#endif
#endif //SVC_EXTENSION
{
  m_scalingList = new TComScalingList;
#if REF_REGION_OFFSET
  ::memset(m_scaledRefLayerOffsetPresentFlag,   0, sizeof(m_scaledRefLayerOffsetPresentFlag));
  ::memset(m_refRegionOffsetPresentFlag,   0, sizeof(m_refRegionOffsetPresentFlag));
#endif
#if R0209_GENERIC_PHASE
  ::memset(m_resamplePhaseSetPresentFlag,   0, sizeof(m_resamplePhaseSetPresentFlag));
  ::memset(m_phaseHorLuma,   0, sizeof(m_phaseHorLuma));
  ::memset(m_phaseVerLuma,   0, sizeof(m_phaseVerLuma));
  ::memset(m_phaseHorChroma, 0, sizeof(m_phaseHorChroma));
  ::memset(m_phaseVerChroma, 0, sizeof(m_phaseVerChroma));
#endif
}

TComPPS::~TComPPS()
{
#if SCALINGLIST_INFERRING
  if( !m_inferScalingListFlag )
#endif
  delete m_scalingList;
}

#if MOVE_SCALED_OFFSET_TO_PPS
#if O0098_SCALED_REF_LAYER_ID
Window& TComPPS::getScaledRefLayerWindowForLayer(Int layerId)
{
  static Window win;

  for (Int i = 0; i < (Int)m_numScaledRefLayerOffsets; i++)
  {
    if (layerId == (Int)m_scaledRefLayerId[i])
    {
      return m_scaledRefLayerWindow[i];
    }
  }

  win.resetWindow();  // scaled reference layer offsets are inferred to be zero when not present
  return win;
}
#endif
#if REF_REGION_OFFSET
Window& TComPPS::getRefLayerWindowForLayer(Int layerId)
{
  static Window win;

  for (Int i = 0; i < (Int)m_numScaledRefLayerOffsets; i++)
  {
    if (layerId == (Int)m_scaledRefLayerId[i])
    {
      return m_refLayerWindow[i];
    }
  }

  win.resetWindow();  // reference region offsets are inferred to be zero when not present
  return win;
}
#endif

#if RESAMPLING_FIX
#if R0209_GENERIC_PHASE
Bool TComPPS::hasZeroResamplingPhase(Int refLayerIdc)
{
  Int phaseHorLuma   = this->getPhaseHorLuma(refLayerIdc);
  Int phaseVerLuma   = this->getPhaseVerLuma(refLayerIdc);
  Int phaseHorChroma = this->getPhaseHorChroma(refLayerIdc);
  Int phaseVerChroma = this->getPhaseVerChroma(refLayerIdc);
  return ( phaseHorLuma == 0 && phaseHorChroma == 0 && phaseVerLuma == 0 && phaseVerChroma == 0);
}
#endif
#endif

#endif

TComReferencePictureSet::TComReferencePictureSet()
: m_numberOfPictures (0)
, m_numberOfNegativePictures (0)
, m_numberOfPositivePictures (0)
, m_numberOfLongtermPictures (0)
, m_interRPSPrediction (0) 
, m_deltaRIdxMinus1 (0)   
, m_deltaRPS (0) 
, m_numRefIdc (0) 
{
  ::memset( m_deltaPOC, 0, sizeof(m_deltaPOC) );
  ::memset( m_POC, 0, sizeof(m_POC) );
  ::memset( m_used, 0, sizeof(m_used) );
  ::memset( m_refIdc, 0, sizeof(m_refIdc) );
}

TComReferencePictureSet::~TComReferencePictureSet()
{
}

Void TComReferencePictureSet::setUsed(Int bufferNum, Bool used)
{
  m_used[bufferNum] = used;
}

Void TComReferencePictureSet::setDeltaPOC(Int bufferNum, Int deltaPOC)
{
  m_deltaPOC[bufferNum] = deltaPOC;
}

Void TComReferencePictureSet::setNumberOfPictures(Int numberOfPictures)
{
  m_numberOfPictures = numberOfPictures;
}

Int TComReferencePictureSet::getUsed(Int bufferNum)
{
  return m_used[bufferNum];
}

Int TComReferencePictureSet::getDeltaPOC(Int bufferNum)
{
  return m_deltaPOC[bufferNum];
}

Int TComReferencePictureSet::getNumberOfPictures()
{
  return m_numberOfPictures;
}

Int TComReferencePictureSet::getPOC(Int bufferNum)
{
  return m_POC[bufferNum];
}

Void TComReferencePictureSet::setPOC(Int bufferNum, Int POC)
{
  m_POC[bufferNum] = POC;
}

Bool TComReferencePictureSet::getCheckLTMSBPresent(Int bufferNum)
{
  return m_bCheckLTMSB[bufferNum];
}

Void TComReferencePictureSet::setCheckLTMSBPresent(Int bufferNum, Bool b)
{
  m_bCheckLTMSB[bufferNum] = b;
}

//  set the reference idc value at uiBufferNum entry to the value of iRefIdc
// * \param uiBufferNum
// * \param iRefIdc
// * \returns Void
//
Void TComReferencePictureSet::setRefIdc(Int bufferNum, Int refIdc)
{
  m_refIdc[bufferNum] = refIdc;
}

//  get the reference idc value at uiBufferNum
// * \param uiBufferNum
// * \returns Int
Int  TComReferencePictureSet::getRefIdc(Int bufferNum)
{
  return m_refIdc[bufferNum];
}

//  Sorts the deltaPOC and Used by current values in the RPS based on the deltaPOC values.
// *  deltaPOC values are sorted with -ve values before the +ve values.  -ve values are in decreasing order.
// *  +ve values are in increasing order.
// * \returns Void
Void TComReferencePictureSet::sortDeltaPOC()
{
  // sort in increasing order (smallest first)
  for(Int j=1; j < getNumberOfPictures(); j++)
  { 
    Int deltaPOC = getDeltaPOC(j);
    Bool used = getUsed(j);
    for (Int k=j-1; k >= 0; k--)
    {
      Int temp = getDeltaPOC(k);
      if (deltaPOC < temp)
      {
        setDeltaPOC(k+1, temp);
        setUsed(k+1, getUsed(k));
        setDeltaPOC(k, deltaPOC);
        setUsed(k, used);
      }
    }
  }
  // flip the negative values to largest first
  Int numNegPics = getNumberOfNegativePictures();
  for(Int j=0, k=numNegPics-1; j < numNegPics>>1; j++, k--)
  { 
    Int deltaPOC = getDeltaPOC(j);
    Bool used = getUsed(j);
    setDeltaPOC(j, getDeltaPOC(k));
    setUsed(j, getUsed(k));
    setDeltaPOC(k, deltaPOC);
    setUsed(k, used);
  }
}

//  Prints the deltaPOC and RefIdc (if available) values in the RPS.
// *  A "*" is added to the deltaPOC value if it is Used bu current.
// * \returns Void
Void TComReferencePictureSet::printDeltaPOC()
{
  printf("DeltaPOC = { ");
  for(Int j=0; j < getNumberOfPictures(); j++)
  {
    printf("%d%s ", getDeltaPOC(j), (getUsed(j)==1)?"*":"");
  } 
  if (getInterRPSPrediction()) 
  {
    printf("}, RefIdc = { ");
    for(Int j=0; j < getNumRefIdc(); j++)
    {
      printf("%d ", getRefIdc(j));
    } 
  }
  printf("}\n");
}

TComRPSList::TComRPSList()
:m_referencePictureSets (NULL)
{
}

TComRPSList::~TComRPSList()
{
}

Void TComRPSList::create( Int numberOfReferencePictureSets)
{
  m_numberOfReferencePictureSets = numberOfReferencePictureSets;
  m_referencePictureSets = new TComReferencePictureSet[numberOfReferencePictureSets];
}

Void TComRPSList::destroy()
{
  if (m_referencePictureSets)
  {
    delete [] m_referencePictureSets;
  }
  m_numberOfReferencePictureSets = 0;
  m_referencePictureSets = NULL;
}



TComReferencePictureSet* TComRPSList::getReferencePictureSet(Int referencePictureSetNum)
{
  return &m_referencePictureSets[referencePictureSetNum];
}

Int TComRPSList::getNumberOfReferencePictureSets()
{
  return m_numberOfReferencePictureSets;
}

Void TComRPSList::setNumberOfReferencePictureSets(Int numberOfReferencePictureSets)
{
  m_numberOfReferencePictureSets = numberOfReferencePictureSets;
}

TComRefPicListModification::TComRefPicListModification()
: m_bRefPicListModificationFlagL0 (false)
, m_bRefPicListModificationFlagL1 (false)
{
  ::memset( m_RefPicSetIdxL0, 0, sizeof(m_RefPicSetIdxL0) );
  ::memset( m_RefPicSetIdxL1, 0, sizeof(m_RefPicSetIdxL1) );
}

TComRefPicListModification::~TComRefPicListModification()
{
}

TComScalingList::TComScalingList()
{
  init();
}

TComScalingList::~TComScalingList()
{
  destroy();
}

/*
//  set default quantization matrix to array
Void TComSlice::setDefaultScalingList()
{
  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId=0;listId<g_scalingListNum[sizeId];listId++)
    {
      getScalingList()->processDefaultMatrix(sizeId, listId);
    }
  }
}
//  check if use default quantization matrix
// \returns true if use default quantization matrix in all size
Bool TComSlice::checkDefaultScalingList()
{
  UInt defaultCounter=0;

  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId=0;listId<g_scalingListNum[sizeId];listId++)
    {
      if( !memcmp(getScalingList()->getScalingListAddress(sizeId,listId), getScalingList()->getScalingListDefaultAddress(sizeId, listId),sizeof(Int)*min(MAX_MATRIX_COEF_NUM,(Int)g_scalingListSize[sizeId])) // check value of matrix
     && ((sizeId < SCALING_LIST_16x16) || (getScalingList()->getScalingListDC(sizeId,listId) == 16))) // check DC value
      {
        defaultCounter++;
      }
    }
  }
  return (defaultCounter == (SCALING_LIST_NUM * SCALING_LIST_SIZE_NUM - 4)) ? false : true; // -4 for 32x32
}
*/

//  get scaling matrix from RefMatrixID
// * \param sizeId size index
// * \param Index of input matrix
// * \param Index of reference matrix
Void TComScalingList::processRefMatrix( UInt sizeId, UInt listId , UInt refListId )
{
  ::memcpy(getScalingListAddress(sizeId, listId),((listId == refListId)? getScalingListDefaultAddress(sizeId, refListId): getScalingListAddress(sizeId, refListId)),sizeof(Int)*min(MAX_MATRIX_COEF_NUM,(Int)g_scalingListSize[sizeId]));
}

//  parse syntax infomation
// *  \param pchFile syntax infomation
// *  \returns false if successful
Bool TComScalingList::xParseScalingList(Char* pchFile)
{
  FILE *fp;
  Char line[1024];
  UInt sizeIdc,listIdc;
  UInt i,size = 0;
  Int *src=0,data;
  Char *ret;
  UInt  retval;

  if((fp = fopen(pchFile,"r")) == (FILE*)NULL)
  {
    printf("can't open file %s :: set Default Matrix\n",pchFile);
    return true;
  }

  for(sizeIdc = 0; sizeIdc < SCALING_LIST_SIZE_NUM; sizeIdc++)
  {
    size = min(MAX_MATRIX_COEF_NUM,(Int)g_scalingListSize[sizeIdc]);
    for(listIdc = 0; listIdc < g_scalingListNum[sizeIdc]; listIdc++)
    {
      src = getScalingListAddress(sizeIdc, listIdc);

      fseek(fp,0,0);
      do 
      {
        ret = fgets(line, 1024, fp);
        if ((ret==NULL)||(strstr(line, MatrixType[sizeIdc][listIdc])==NULL && feof(fp)))
        {
          printf("Error: can't read Matrix :: set Default Matrix\n");
          return true;
        }
      }
      while (strstr(line, MatrixType[sizeIdc][listIdc]) == NULL);
      for (i=0; i<size; i++)
      {
        retval = fscanf(fp, "%d,", &data);
        if (retval!=1)
        {
          printf("Error: can't read Matrix :: set Default Matrix\n");
          return true;
        }
        src[i] = data;
      }
      //set DC value for default matrix check
      setScalingListDC(sizeIdc,listIdc,src[0]);

      if(sizeIdc > SCALING_LIST_8x8)
      {
        fseek(fp,0,0);
        do 
        {
          ret = fgets(line, 1024, fp);
          if ((ret==NULL)||(strstr(line, MatrixType_DC[sizeIdc][listIdc])==NULL && feof(fp)))
          {
            printf("Error: can't read DC :: set Default Matrix\n");
            return true;
          }
        }
        while (strstr(line, MatrixType_DC[sizeIdc][listIdc]) == NULL);
        retval = fscanf(fp, "%d,", &data);
        if (retval!=1)
        {
          printf("Error: can't read Matrix :: set Default Matrix\n");
          return true;
        }
        //overwrite DC value when size of matrix is larger than 16x16
        setScalingListDC(sizeIdc,listIdc,data);
      }
    }
  }
  fclose(fp);
  return false;
}

//  initialization process of quantization matrix array
Void TComScalingList::init()
{
  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId = 0; listId < g_scalingListNum[sizeId]; listId++)
    {
      m_scalingListCoef[sizeId][listId] = new Int [min(MAX_MATRIX_COEF_NUM,(Int)g_scalingListSize[sizeId])];
    }
  }
  m_scalingListCoef[SCALING_LIST_32x32][3] = m_scalingListCoef[SCALING_LIST_32x32][1]; // copy address for 32x32
}

//  destroy quantization matrix array
Void TComScalingList::destroy()
{
  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId = 0; listId < g_scalingListNum[sizeId]; listId++)
    {
      if(m_scalingListCoef[sizeId][listId]) delete [] m_scalingListCoef[sizeId][listId];
    }
  }
}

//  get default address of quantization matrix
// * \param sizeId size index
// * \param listId list index
// * \returns pointer of quantization matrix
Int* TComScalingList::getScalingListDefaultAddress(UInt sizeId, UInt listId)
{
  Int *src = 0;
  switch(sizeId)
  {
    case SCALING_LIST_4x4:
      src = g_quantTSDefault4x4;
      break;
    case SCALING_LIST_8x8:
      src = (listId<3) ? g_quantIntraDefault8x8 : g_quantInterDefault8x8;
      break;
    case SCALING_LIST_16x16:
      src = (listId<3) ? g_quantIntraDefault8x8 : g_quantInterDefault8x8;
      break;
    case SCALING_LIST_32x32:
      src = (listId<1) ? g_quantIntraDefault8x8 : g_quantInterDefault8x8;
      break;
    default:
      assert(0);
      src = NULL;
      break;
  }
  return src;
}

//  process of default matrix
// * \param sizeId size index
// * \param Index of input matrix

Void TComScalingList::processDefaultMatrix(UInt sizeId, UInt listId)
{
  ::memcpy(getScalingListAddress(sizeId, listId),getScalingListDefaultAddress(sizeId,listId),sizeof(Int)*min(MAX_MATRIX_COEF_NUM,(Int)g_scalingListSize[sizeId]));
  setScalingListDC(sizeId,listId,SCALING_LIST_DC);
}

//  check DC value of matrix for default matrix signaling
Void TComScalingList::checkDcOfMatrix()
{
  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId = 0; listId < g_scalingListNum[sizeId]; listId++)
    {
      //check default matrix?
      if(getScalingListDC(sizeId,listId) == 0)
      {
        processDefaultMatrix(sizeId, listId);
      }
    }
  }
}

ParameterSetManager::ParameterSetManager()
#if SVC_EXTENSION
: m_activeSPSId(-1)
, m_activePPSId(-1)
#else
: m_vpsMap(MAX_NUM_VPS)
, m_spsMap(MAX_NUM_SPS)
, m_ppsMap(MAX_NUM_PPS)
, m_activeVPSId(-1)
, m_activeSPSId(-1)
, m_activePPSId(-1)
#endif
{
}


ParameterSetManager::~ParameterSetManager()
{
}

//! activate a SPS from a active parameter sets SEI message
//! \returns true, if activation is successful
Bool ParameterSetManager::activateSPSWithSEI(Int spsId)
{
  TComSPS *sps = m_spsMap.getPS(spsId);
  if (sps)
  {
    Int vpsId = sps->getVPSId();
    if (m_vpsMap.getPS(vpsId))
    {
      m_activeVPSId = vpsId;
      m_activeSPSId = spsId;
      return true;
    }
    else
    {
      printf("Warning: tried to activate SPS using an Active parameter sets SEI message. Referenced VPS does not exist.");
    }
  }
  else
  {
    printf("Warning: tried to activate non-existing SPS using an Active parameter sets SEI message.");
  }
  return false;
}

//! activate a PPS and depending on isIDR parameter also SPS and VPS
//! \returns true, if activation is successful
Bool ParameterSetManager::activatePPS(Int ppsId, Bool isIRAP)
{
  TComPPS *pps = m_ppsMap.getPS(ppsId);
  if (pps)
  {
    Int spsId = pps->getSPSId();
    if (!isIRAP && (spsId != m_activeSPSId))
    {
      printf("Warning: tried to activate PPS referring to a inactive SPS at non-IRAP.");
      return false;
    }
    TComSPS *sps = m_spsMap.getPS(spsId);
    if (sps)
    {
      Int vpsId = sps->getVPSId();
      if (!isIRAP && (vpsId != m_activeVPSId))
      {
        printf("Warning: tried to activate PPS referring to a inactive VPS at non-IRAP.");
        return false;
      }
      if (m_vpsMap.getPS(vpsId))
      {
        m_activePPSId = ppsId;
        m_activeVPSId = vpsId;
        m_activeSPSId = spsId;

        return true;
      }
      else
      {
        printf("Warning: tried to activate PPS that refers to a non-existing VPS.");
      }
    }
    else
    {
      printf("Warning: tried to activate a PPS that refers to a non-existing SPS.");
    }
  }
  else
  {
    printf("Warning: tried to activate non-existing PPS.");
  }
  return false;
}

ProfileTierLevel::ProfileTierLevel()
  : m_profileSpace    (0)
  , m_tierFlag        (false)
  , m_profileIdc      (0)
  , m_levelIdc        (0)
, m_progressiveSourceFlag  (false)
, m_interlacedSourceFlag   (false)
, m_nonPackedConstraintFlag(false)
, m_frameOnlyConstraintFlag(false)
{
  ::memset(m_profileCompatibilityFlag, 0, sizeof(m_profileCompatibilityFlag));
}
#if VPS_EXTN_PROFILE_INFO
Void ProfileTierLevel::copyProfileInfo(ProfileTierLevel *ptl)
{
  this->setProfileSpace          ( ptl->getProfileSpace()      );
  this->setTierFlag              ( ptl->getTierFlag()          );
  this->setProfileIdc            ( ptl->getProfileIdc()        );
  for(Int j = 0; j < 32; j++)
  {
    this->setProfileCompatibilityFlag(j, ptl->getProfileCompatibilityFlag(j));
  }
  this->setProgressiveSourceFlag  ( ptl->getProgressiveSourceFlag()  );
  this->setInterlacedSourceFlag   ( ptl->getInterlacedSourceFlag()   );
  this->setNonPackedConstraintFlag( ptl->getNonPackedConstraintFlag());
  this->setFrameOnlyConstraintFlag( ptl->getFrameOnlyConstraintFlag());  
}
#endif

TComPTL::TComPTL()
{
  ::memset(m_subLayerProfilePresentFlag, 0, sizeof(m_subLayerProfilePresentFlag));
  ::memset(m_subLayerLevelPresentFlag,   0, sizeof(m_subLayerLevelPresentFlag  ));
}
#if VPS_EXTN_PROFILE_INFO
Void TComPTL::copyProfileInfo(TComPTL *ptl)
{
  // Copy all information related to general profile
  this->getGeneralPTL()->copyProfileInfo(ptl->getGeneralPTL());
}
#endif

/*

#if SVC_EXTENSION
Bool TComSlice::setBaseColPic(  TComList<TComPic*>& rcListPic, UInt refLayerIdc )
{  
  if(m_layerId == 0)
  {
    memset( m_pcBaseColPic, 0, sizeof( m_pcBaseColPic ) );
    return false;
  }        
#if POC_RESET_FLAG || POC_RESET_IDC_DECODER
#if POC_RESET_IDC_DECODER
  TComPic* pic = xGetRefPic( rcListPic, getPOC() );
#else
  TComPic* pic = xGetRefPic( rcListPic, m_bPocResetFlag ? 0 : m_iPOC );
#endif

  if( pic )
  {
    setBaseColPic(refLayerIdc, pic );
  }
  else
  {
    return false;
  }
  
  return true;
#else
  setBaseColPic(refLayerIdc, xGetRefPic(rcListPic, getPOC()));
  return true;
#endif
}

#if MFM_ENCCONSTRAINT
TComPic* TComSlice::getBaseColPic(  TComList<TComPic*>& rcListPic )
{
#if POC_RESET_FLAG
  return xGetRefPic( rcListPic, m_bPocResetFlag ? 0 : m_iPOC );
#else
  return xGetRefPic( rcListPic, m_iPOC );
#endif
}
#endif

Void TComSlice::setILRPic(TComPic **pcIlpPic)
{
  for( Int i = 0; i < m_activeNumILRRefIdx; i++ )
  {
    Int refLayerIdc = m_interLayerPredLayerIdc[i];

    if( pcIlpPic[refLayerIdc] )
    {
      TComPic* pcRefPicBL = m_pcBaseColPic[refLayerIdc];

      // copy scalability ratio, it is needed to get the correct location for the motion field of the corresponding reference layer block
      pcIlpPic[refLayerIdc]->setSpatialEnhLayerFlag( refLayerIdc, m_pcPic->isSpatialEnhLayer(refLayerIdc) );

      pcIlpPic[refLayerIdc]->copyUpsampledPictureYuv( m_pcPic->getFullPelBaseRec( refLayerIdc ), pcIlpPic[refLayerIdc]->getPicYuvRec() );      
      pcIlpPic[refLayerIdc]->getSlice(0)->setBaseColPic( refLayerIdc, pcRefPicBL );

      //set reference picture POC of each ILP reference 
      pcIlpPic[refLayerIdc]->getSlice(0)->setPOC( m_iPOC );

      //set temporal Id 
      pcIlpPic[refLayerIdc]->getSlice(0)->setTLayer( m_uiTLayer );

      //copy layer id from the reference layer 
      pcIlpPic[refLayerIdc]->setLayerId( pcRefPicBL->getLayerId() );

      pcIlpPic[refLayerIdc]->getPicYuvRec()->setBorderExtension( false );
      pcIlpPic[refLayerIdc]->getPicYuvRec()->extendPicBorder();
      for (Int j=0; j<pcIlpPic[refLayerIdc]->getPicSym()->getNumberOfCUsInFrame(); j++)    // set reference CU layerId
      {
        pcIlpPic[refLayerIdc]->getPicSym()->getCU(j)->setLayerId( pcIlpPic[refLayerIdc]->getLayerId() );
      }
      pcIlpPic[refLayerIdc]->setIsLongTerm(1);

#if REF_IDX_MFM
      if( m_bMFMEnabledFlag && !(m_eNalUnitType >= NAL_UNIT_CODED_SLICE_BLA_W_LP && m_eNalUnitType <= NAL_UNIT_CODED_SLICE_CRA) )
      {
        //set reference picture POC of each ILP reference 
        assert( pcIlpPic[refLayerIdc]->getPOC() == pcRefPicBL->getPOC() );

        //copy slice type from the reference layer
        pcIlpPic[refLayerIdc]->getSlice(0)->setSliceType( pcRefPicBL->getSlice(0)->getSliceType() );

        //copy "used for reference"
        pcIlpPic[refLayerIdc]->getSlice(0)->setReferenced( pcRefPicBL->getSlice(0)->isReferenced() );

        for( Int refList = 0; refList < 2; refList++ )
        {
          RefPicList refPicList = RefPicList( refList );

          //set reference POC of ILP
          pcIlpPic[refLayerIdc]->getSlice(0)->setNumRefIdx(refPicList, pcRefPicBL->getSlice(0)->getNumRefIdx(refPicList));
          assert(pcIlpPic[refLayerIdc]->getSlice(0)->getNumRefIdx(refPicList) >= 0);
          assert(pcIlpPic[refLayerIdc]->getSlice(0)->getNumRefIdx(refPicList) <= MAX_NUM_REF);

          //initialize reference POC of ILP
          for(Int refIdx = 0; refIdx < pcRefPicBL->getSlice(0)->getNumRefIdx(refPicList); refIdx++)
          {
            pcIlpPic[refLayerIdc]->getSlice(0)->setRefPOC(pcRefPicBL->getSlice(0)->getRefPOC(refPicList, refIdx), refPicList, refIdx);
            pcIlpPic[refLayerIdc]->getSlice(0)->setRefPic(pcRefPicBL->getSlice(0)->getRefPic(refPicList, refIdx), refPicList, refIdx);
          }

          for(Int refIdx = pcRefPicBL->getSlice(0)->getNumRefIdx(refPicList); refIdx < MAX_NUM_REF; refIdx++) 
          { 
            pcIlpPic[refLayerIdc]->getSlice(0)->setRefPOC(0, refPicList, refIdx); 
            pcIlpPic[refLayerIdc]->getSlice(0)->setRefPic(NULL, refPicList, refIdx); 
          }

          //copy reference pictures' marking from the reference layer
          for(Int j = 0; j < MAX_NUM_REF + 1; j++)
          {
            pcIlpPic[refLayerIdc]->getSlice(0)->setIsUsedAsLongTerm(refList, j, pcRefPicBL->getSlice(0)->getIsUsedAsLongTerm(refList, j));
          }
        }

        pcIlpPic[refLayerIdc]->copyUpsampledMvField( refLayerIdc, m_pcBaseColPic[refLayerIdc] );
      }
      else
      {
        pcIlpPic[refLayerIdc]->initUpsampledMvField();
      }
#endif

#if O0225_MAX_TID_FOR_REF_LAYERS
      Int maxTidIlRefPicsPlus1 = m_pcVPS->getMaxTidIlRefPicsPlus1( pcIlpPic[refLayerIdc]->getSlice(0)->getLayerId(), m_layerId );
#else
      Int maxTidIlRefPicsPlus1 = m_pcVPS->getMaxTidIlRefPicsPlus1( pcIlpPic[refLayerIdc]->getSlice(0)->getLayerId() );
#endif
      assert( (Int)pcIlpPic[refLayerIdc]->getSlice(0)->getTLayer() < maxTidIlRefPicsPlus1 || ( !maxTidIlRefPicsPlus1 && pcIlpPic[refLayerIdc]->getSlice(0)->getRapPicFlag() ) );

    }
  }
}

Int TComSlice::getReferenceLayerIdc( UInt refLayerId )
{ 
  for( Int i = 0; i < m_activeNumILRRefIdx; i++ )
  {
    if( m_pcVPS->getRefLayerId(m_layerId, m_interLayerPredLayerIdc[i]) == refLayerId )
    {
      return m_interLayerPredLayerIdc[i];
    }
  }

  return -1;
}
#endif //SVC_EXTENSION
*/
//! \}
