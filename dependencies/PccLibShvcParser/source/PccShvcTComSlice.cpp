/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2016, ITU/ISO/IEC
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

/** \file     TComSlice.cpp
    \brief    slice header and SPS class
*/

#include "PccShvcCommonDef.h"
#include "PccShvcTComSlice.h"
#include "PccShvcTComRom.h"
// #include "TComPic.h"
// #include "TLibEncoder/TEncSbac.h"
// #include "TLibDecoder/TDecSbac.h"


using namespace pcc_shvc;

//! \ingroup TLibCommon
//! \{

#if SVC_EXTENSION
#include <numeric>
ParameterSetMap<TComVPS> ParameterSetManager::m_vpsMap(MAX_NUM_VPS);
ParameterSetMap<TComSPS> ParameterSetManager::m_spsMap(MAX_NUM_SPS);
ParameterSetMap<TComPPS> ParameterSetManager::m_ppsMap(MAX_NUM_PPS);
Int ParameterSetManager::m_activeVPSId = -1;
#endif

TComSlice::TComSlice()
: m_iPPSId                        ( -1 )
, m_PicOutputFlag                 ( true )
, m_iPOC                          ( 0 )
, m_iLastIDR                      ( 0 )
, m_iAssociatedIRAP               ( 0 )
, m_iAssociatedIRAPType           ( NAL_UNIT_INVALID )
, m_pRPS                          ( 0 )
, m_localRPS                      ( )
, m_rpsIdx                        ( 0 )
, m_RefPicListModification        ( )
, m_eNalUnitType                  ( NAL_UNIT_CODED_SLICE_IDR_W_RADL )
, m_eSliceType                    ( I_SLICE )
, m_iSliceQp                      ( 0 )
, m_dependentSliceSegmentFlag     ( false )
#if ADAPTIVE_QP_SELECTION
, m_iSliceQpBase                  ( 0 )
#endif
, m_ChromaQpAdjEnabled            ( false )
, m_deblockingFilterDisable       ( false )
, m_deblockingFilterOverrideFlag  ( false )
, m_deblockingFilterBetaOffsetDiv2( 0 )
, m_deblockingFilterTcOffsetDiv2  ( 0 )
, m_bCheckLDC                     ( false )
, m_iSliceQpDelta                 ( 0 )
, m_iDepth                        ( 0 )
, m_bRefenced                     ( false )
, m_pcVPS                         ( NULL )
, m_pcSPS                         ( NULL )
, m_pcPPS                         ( NULL )
, m_pcPic                         ( NULL )
, m_colFromL0Flag                 ( true )
, m_noOutputPriorPicsFlag         ( false )
, m_noRaslOutputFlag              ( false )
, m_handleCraAsBlaFlag            ( false )
, m_colRefIdx                     ( 0 )
, m_maxNumMergeCand               ( 0 )
, m_uiTLayer                      ( 0 )
, m_bTLayerSwitchingFlag          ( false )
, m_sliceMode                     ( NO_SLICES )
, m_sliceArgument                 ( 0 )
, m_sliceCurStartCtuTsAddr        ( 0 )
, m_sliceCurEndCtuTsAddr          ( 0 )
, m_sliceIdx                      ( 0 )
, m_sliceSegmentMode              ( NO_SLICES )
, m_sliceSegmentArgument          ( 0 )
, m_sliceSegmentCurStartCtuTsAddr ( 0 )
, m_sliceSegmentCurEndCtuTsAddr   ( 0 )
, m_nextSlice                     ( false )
, m_nextSliceSegment              ( false )
, m_sliceBits                     ( 0 )
, m_sliceSegmentBits              ( 0 )
, m_bFinalized                    ( false )
, m_bTestWeightPred               ( false )
, m_bTestWeightBiPred             ( false )
, m_substreamSizes                ( )
, m_cabacInitFlag                 ( false )
, m_bLMvdL1Zero                   ( false )
, m_temporalLayerNonReferenceFlag ( false )
, m_LFCrossSliceBoundaryFlag      ( false )
, m_enableTMVPFlag                ( true )
, m_encCABACTableIdx              (I_SLICE)
#if SVC_EXTENSION
, m_firstSliceInPic               ( false )
, m_availableForTMVPRefFlag       ( true )
, m_layerId                       ( 0 )
, m_bMFMEnabledFlag               ( false )
, m_bDiscardableFlag              ( false )
, m_bCrossLayerBLAFlag            ( false )
, m_pocResetIdc                   ( 0 )
, m_pocResetPeriodId              ( 0 )
, m_fullPocResetFlag              ( false )
, m_pocLsbVal                     ( 0 )
, m_pocMsbVal                     ( 0 )
, m_pocMsbValRequiredFlag         ( false )
, m_pocMsbValPresentFlag          ( false )
, m_pocMsbValNeeded               ( false )
, m_picOrderCntLsb (0)
#endif //SVC_EXTENSION
{
  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_aiNumRefIdx[i] = 0;
  }

#if SVC_EXTENSION
  memset( m_pcBaseColPic, 0, sizeof( m_pcBaseColPic ) );
  m_activeNumILRRefIdx        = 0; 
  m_interLayerPredEnabledFlag = 0;
  ::memset( m_interLayerPredLayerIdc, 0, sizeof(m_interLayerPredLayerIdc) );
#endif //SVC_EXTENSION

  for (UInt component = 0; component < MAX_NUM_COMPONENT; component++)
  {
    m_lambdas            [component] = 0.0;
    m_iSliceChromaQpDelta[component] = 0;
  }

  initEqualRef();

  for ( Int idx = 0; idx < MAX_NUM_REF; idx++ )
  {
    m_list1IdxToList0Idx[idx] = -1;
  }

  for(Int iNumCount = 0; iNumCount < MAX_NUM_REF; iNumCount++)
  {
    for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
    {
      m_apcRefPicList [i][iNumCount] = NULL;
      m_aiRefPOCList  [i][iNumCount] = 0;
    }
  }

  resetWpScaling();
  initWpAcDcParam();

  for(Int ch=0; ch < MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_saoEnabledFlag[ch] = false;
  }
}

TComSlice::~TComSlice()
{
}


#if SVC_EXTENSION
Void TComSlice::initSlice( UInt layerId )
#else
Void TComSlice::initSlice()
#endif
{
#if SVC_EXTENSION
  m_layerId                   = layerId;
  m_activeNumILRRefIdx        = 0;
  m_interLayerPredEnabledFlag = 0;
  m_picOrderCntLsb            = 0;
  m_pocResetIdc               = 0;
  m_pocResetPeriodId          = 0;
  m_fullPocResetFlag          = false;
  m_pocLsbVal                 = 0;
  m_pocMsbVal                 = 0;
  m_pocMsbValRequiredFlag     = false;
  m_pocMsbValPresentFlag      = false;
  m_pocMsbValNeeded           = false;
  m_pocResetDeltaPoc          = 0;
#endif

  for(UInt i=0; i<NUM_REF_PIC_LIST_01; i++)
  {
    m_aiNumRefIdx[i]      = 0;
  }
  m_colFromL0Flag = true;

  m_colRefIdx = 0;
  initEqualRef();

  m_bCheckLDC = false;

  for (UInt component = 0; component < MAX_NUM_COMPONENT; component++)
  {
    m_iSliceChromaQpDelta[component] = 0;
  }

  m_maxNumMergeCand = MRG_MAX_NUM_CANDS;

  m_bFinalized=false;

  m_substreamSizes.clear();
  m_cabacInitFlag        = false;
  m_enableTMVPFlag = true;
}

Bool TComSlice::getRapPicFlag() const
{
  return getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA;
}

/*
Void  TComSlice::sortPicList        (TComList<TComPic*>& rcListPic)
{
  TComPic*    pcPicExtract;
  TComPic*    pcPicInsert;

  TComList<TComPic*>::iterator    iterPicExtract;
  TComList<TComPic*>::iterator    iterPicExtract_1;
  TComList<TComPic*>::iterator    iterPicInsert;

  for (Int i = 1; i < (Int)(rcListPic.size()); i++)
  {
    iterPicExtract = rcListPic.begin();
    for (Int j = 0; j < i; j++)
    {
      iterPicExtract++;
    }
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

TComPic* TComSlice::xGetRefPic (TComList<TComPic*>& rcListPic, Int poc)
{
  TComList<TComPic*>::iterator  iterPic = rcListPic.begin();
  TComPic*                      pcPic = *(iterPic);
  while ( iterPic != rcListPic.end() )
  {
#if SVC_EXTENSION
    if( pcPic->getPOC() == poc && pcPic->getSlice(0)->isReferenced() )
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
#if SVC_EXTENSION
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

      if (poc == picPoc)
      {
        if(pcPic->getIsLongTerm())
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

Void TComSlice::setRefPOCList       ()
{
  for (Int iDir = 0; iDir < NUM_REF_PIC_LIST_01; iDir++)
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
  if ( m_eSliceType == I_SLICE)
  {
    ::memset( m_apcRefPicList, 0, sizeof (m_apcRefPicList));
    ::memset( m_aiNumRefIdx,   0, sizeof ( m_aiNumRefIdx ));

    if (!checkNumPocTotalCurr)
    {
      return;
    }
  }

  TComPic*  pcRefPic= NULL;
  static const UInt MAX_NUM_NEGATIVE_PICTURES=16;
  TComPic*  RefPicSetStCurr0[MAX_NUM_NEGATIVE_PICTURES];
  TComPic*  RefPicSetStCurr1[MAX_NUM_NEGATIVE_PICTURES];
  TComPic*  RefPicSetLtCurr[MAX_NUM_NEGATIVE_PICTURES];
  UInt NumPicStCurr0 = 0;
  UInt NumPicStCurr1 = 0;
  UInt NumPicLtCurr = 0;
  Int i;

#if SVC_EXTENSION
  if( m_layerId == 0 || ( m_layerId > 0 && ( m_activeNumILRRefIdx == 0 || !(m_eNalUnitType >= NAL_UNIT_CODED_SLICE_BLA_W_LP && m_eNalUnitType <= NAL_UNIT_CODED_SLICE_CRA) ) ) )
  {
#endif
  for(i=0; i < m_pRPS->getNumberOfNegativePictures(); i++)
  {
    if(m_pRPS->getUsed(i))
    {
      pcRefPic = xGetRefPic(rcListPic, getPOC()+m_pRPS->getDeltaPOC(i));
      pcRefPic->setIsLongTerm(0);
      pcRefPic->getPicYuvRec()->extendPicBorder();
      RefPicSetStCurr0[NumPicStCurr0] = pcRefPic;
      NumPicStCurr0++;
      pcRefPic->setCheckLTMSBPresent(false);
    }
  }

  for(; i < m_pRPS->getNumberOfNegativePictures()+m_pRPS->getNumberOfPositivePictures(); i++)
  {
    if(m_pRPS->getUsed(i))
    {
      pcRefPic = xGetRefPic(rcListPic, getPOC()+m_pRPS->getDeltaPOC(i));
      pcRefPic->setIsLongTerm(0);
      pcRefPic->getPicYuvRec()->extendPicBorder();
      RefPicSetStCurr1[NumPicStCurr1] = pcRefPic;
      NumPicStCurr1++;
      pcRefPic->setCheckLTMSBPresent(false);
    }
  }

  for(i = m_pRPS->getNumberOfNegativePictures()+m_pRPS->getNumberOfPositivePictures()+m_pRPS->getNumberOfLongtermPictures()-1; i > m_pRPS->getNumberOfNegativePictures()+m_pRPS->getNumberOfPositivePictures()-1 ; i--)
  {
    if(m_pRPS->getUsed(i))
    {
      pcRefPic = xGetLongTermRefPic(rcListPic, m_pRPS->getPOC(i), m_pRPS->getCheckLTMSBPresent(i));
      pcRefPic->setIsLongTerm(1);
      pcRefPic->getPicYuvRec()->extendPicBorder();
      RefPicSetLtCurr[NumPicLtCurr] = pcRefPic;
      NumPicLtCurr++;
    }
    if(pcRefPic==NULL)
    {
      pcRefPic = xGetLongTermRefPic(rcListPic, m_pRPS->getPOC(i), m_pRPS->getCheckLTMSBPresent(i));
    }
    pcRefPic->setCheckLTMSBPresent(m_pRPS->getCheckLTMSBPresent(i));
  }
#if SVC_EXTENSION
  }
#endif

  // ref_pic_list_init
  TComPic*  rpsCurrList0[MAX_NUM_REF+1];
  TComPic*  rpsCurrList1[MAX_NUM_REF+1];
#if SVC_EXTENSION
  if( m_layerId > 0 && m_activeNumILRRefIdx > 0 )
  {
    // max one resampling direct layer
    if( m_pcVPS->getScalabilityMask( SCALABILITY_ID ) )
    {
      Int numResampler = 0;

      // motion resampling constraint
      Int numMotionResamplers = 0;
      Int refResamplingLayer[MAX_LAYERS];
      memset( refResamplingLayer, 0, sizeof( refResamplingLayer ) );

      for( i=0; i < m_activeNumILRRefIdx; i++ )
      {
        UInt refLayerIdc = m_interLayerPredLayerIdc[i];
        UInt refLayerId = m_pcVPS->getRefLayerId( m_layerId, refLayerIdc );

        const Window &scalEL = getPPS()->getScaledRefLayerWindowForLayer(refLayerId);

        Int scalingOffset = ((scalEL.getWindowLeftOffset()   == 0 ) && 
                             (scalEL.getWindowRightOffset()  == 0 ) && 
                             (scalEL.getWindowTopOffset()    == 0 ) && 
                             (scalEL.getWindowBottomOffset() == 0 ) 
                            );

        Bool sameBitDepths = ( m_pcSPS->getBitDepth(CHANNEL_TYPE_LUMA) == ilpPic[refLayerIdc]->getSlice(0)->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) ) && ( m_pcSPS->getBitDepth(CHANNEL_TYPE_CHROMA) == ilpPic[refLayerIdc]->getSlice(0)->getSPS()->getBitDepth(CHANNEL_TYPE_CHROMA) );

        // motion resampling constraint
        // Allow maximum of one motion resampling process for direct reference layers, and use motion inter-layer prediction from the same layer as texture inter-layer prediction
        if( !( m_pcPic->getPosScalingFactor(refLayerIdc, 0) == POS_SCALING_FACTOR_1X && m_pcPic->getPosScalingFactor(refLayerIdc, 1) == POS_SCALING_FACTOR_1X ) || !scalingOffset || !sameBitDepths 
#if CGS_3D_ASYMLUT
          || getPPS()->getCGSFlag()
#endif
          ) // ratio 1x
        {
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
      assert( numMotionResamplers <= 1  && "Up to 1 motion resampling is allowed" );
    }
  }
  Int numPicTotalCurr = NumPicStCurr0 + NumPicStCurr1 + NumPicLtCurr + m_activeNumILRRefIdx;
#else //SVC_EXTENSION
  Int numPicTotalCurr = NumPicStCurr0 + NumPicStCurr1 + NumPicLtCurr;
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
      assert(numPicTotalCurr == 0);
    }

    if (m_eSliceType == I_SLICE)
    {
      return;
    }

    assert(numPicTotalCurr > 0);
    // general tier and level limit:
    assert(numPicTotalCurr <= 8);
  }

  Int cIdx = 0;
  for ( i=0; i<NumPicStCurr0; i++, cIdx++)
  {
    rpsCurrList0[cIdx] = RefPicSetStCurr0[i];
  }

#if SVC_EXTENSION
#if VIEW_SCALABILITY 
  std::vector<TComPic*> refPicSetInterLayer0;
  std::vector<TComPic*> refPicSetInterLayer1;
  const TComVPS* cVPS=getVPS();

  Int viewIdCurrLayerId  = cVPS->getViewIdVal(cVPS->getViewIndex(m_layerId) );
  Int viewId0            = cVPS->getViewIdVal( 0 );
  for( i=0; i< m_activeNumILRRefIdx ;i++)
  {
      Int refLayerIdc = m_interLayerPredLayerIdc[i];       
      Int maxTidIlRefPicsPlus1 = getVPS()->getMaxTidIlRefPicsPlus1(ilpPic[refLayerIdc]->getSlice(0)->getLayerIdx(), getLayerIdx());
      if( ((Int)(ilpPic[refLayerIdc]->getSlice(0)->getTLayer())<=maxTidIlRefPicsPlus1-1) || (maxTidIlRefPicsPlus1==0 && ilpPic[refLayerIdc]->getSlice(0)->getRapPicFlag()) )
      {
          Int viewIdRefPicLayerIdi = cVPS->getViewIdVal( ilpPic[refLayerIdc]->getSlice(0)->getLayerIdx() );
          Bool refPicSet0Flag =
              ( ( viewIdCurrLayerId <=  viewId0  &&  viewIdCurrLayerId <=  viewIdRefPicLayerIdi )  ||
              ( viewIdCurrLayerId >=  viewId0  &&  viewIdCurrLayerId >=  viewIdRefPicLayerIdi ) );

          if ( refPicSet0Flag )
          {
              refPicSetInterLayer0.push_back( ilpPic[refLayerIdc] );
          }
          else
          {
              refPicSetInterLayer1.push_back( ilpPic[refLayerIdc] );
          }

      }
  }
 

  if( m_layerId > 0 )
  {
      for( i = 0; i < refPicSetInterLayer0.size() && cIdx < numPicTotalCurr; cIdx ++, i ++)      
      {
          rpsCurrList0[cIdx] = refPicSetInterLayer0[i];
      }
  }
#else
    // initial reference picture list construction
    if( m_layerId > 0 )
    {      
      for( i = 0; i < m_activeNumILRRefIdx && cIdx < numPicTotalCurr; cIdx++, i++ )      
      {
        Int refLayerIdc = m_interLayerPredLayerIdc[i];
        Int maxTidIlRefPicsPlus1 = m_pcVPS->getMaxTidIlRefPicsPlus1(ilpPic[refLayerIdc]->getSlice(0)->getLayerIdx(), getLayerIdx());

        // SHM: ILP is not valid due to temporal layer restriction, bitstream is not conformant
        assert( Int(ilpPic[refLayerIdc]->getSlice(0)->getTLayer()) < maxTidIlRefPicsPlus1 || (maxTidIlRefPicsPlus1==0 && ilpPic[refLayerIdc]->getSlice(0)->getRapPicFlag()) );

        // SHM: Inter-layer RPS shall not contain picture with discardable_flag = 1.
        assert( ilpPic[refLayerIdc]->getSlice(0)->getDiscardableFlag() == 0 );

        rpsCurrList0[cIdx] = ilpPic[refLayerIdc];
      }
    }
#endif
#endif //SVC_EXTENSION

  for ( i=0; i<NumPicStCurr1; i++, cIdx++)
  {
    rpsCurrList0[cIdx] = RefPicSetStCurr1[i];
  }
  for ( i=0; i<NumPicLtCurr;  i++, cIdx++)
  {
    rpsCurrList0[cIdx] = RefPicSetLtCurr[i];
  }
#if VIEW_SCALABILITY
  if( m_layerId > 0 )
  {
      for( i = 0; i < refPicSetInterLayer1.size(); cIdx++, i++ )      
      {
          rpsCurrList0[cIdx] = refPicSetInterLayer1[i];
      }
  }
#else
  assert(cIdx == numPicTotalCurr);
#endif

  if (m_eSliceType==B_SLICE)
  {
    cIdx = 0;
    for ( i=0; i<NumPicStCurr1; i++, cIdx++)
    {
      rpsCurrList1[cIdx] = RefPicSetStCurr1[i];
    }
#if VIEW_SCALABILITY 
    if( m_layerId > 0 )
    {
        for( i = 0; i < refPicSetInterLayer1.size(); cIdx ++, i ++)      
        {
            rpsCurrList1[cIdx] = refPicSetInterLayer1[i];
        }
    }
#endif
    for ( i=0; i<NumPicStCurr0; i++, cIdx++)
    {
      rpsCurrList1[cIdx] = RefPicSetStCurr0[i];
    }
    for ( i=0; i<NumPicLtCurr;  i++, cIdx++)
    {
      rpsCurrList1[cIdx] = RefPicSetLtCurr[i];
    }    

#if VIEW_SCALABILITY 
    if( m_layerId > 0 )
    {
      for( i = 0; i < refPicSetInterLayer0.size(); cIdx ++, i ++)
      {
        rpsCurrList1[cIdx] = refPicSetInterLayer0[i];
      }
    }
#else
#if SVC_EXTENSION
    if( m_layerId > 0 )
    {
      for( i = 0; i < m_activeNumILRRefIdx && cIdx < numPicTotalCurr; cIdx++, i++ )
      {
        Int refLayerIdc = m_interLayerPredLayerIdc[i];
        Int maxTidIlRefPicsPlus1 = m_pcVPS->getMaxTidIlRefPicsPlus1( ilpPic[refLayerIdc]->getSlice(0)->getLayerIdx(), getLayerIdx() );

        // SHM: ILP is not valid due to temporal layer restriction, bitstream is not conformant
        assert( Int(ilpPic[refLayerIdc]->getSlice(0)->getTLayer()) < maxTidIlRefPicsPlus1 || (maxTidIlRefPicsPlus1==0 && ilpPic[refLayerIdc]->getSlice(0)->getRapPicFlag()) );

        // SHM: Inter-layer RPS shall not contain picture with discardable_flag = 1.
        assert( ilpPic[refLayerIdc]->getSlice(0)->getDiscardableFlag() == 0 );

        rpsCurrList1[cIdx] = ilpPic[refLayerIdc];
      }
    }
#endif //SVC_EXTENSION
    assert(cIdx == numPicTotalCurr);
#endif
  }

  ::memset(m_bIsUsedAsLongTerm, 0, sizeof(m_bIsUsedAsLongTerm));

  for (Int rIdx = 0; rIdx < m_aiNumRefIdx[REF_PIC_LIST_0]; rIdx ++)
  {
    cIdx = m_RefPicListModification.getRefPicListModificationFlagL0() ? m_RefPicListModification.getRefPicSetIdxL0(rIdx) : rIdx % numPicTotalCurr;
    assert(cIdx >= 0 && cIdx < numPicTotalCurr);
    m_apcRefPicList[REF_PIC_LIST_0][rIdx] = rpsCurrList0[ cIdx ];

#if SVC_EXTENSION
#if VIEW_SCALABILITY 
    m_bIsUsedAsLongTerm[REF_PIC_LIST_0][rIdx] = (cIdx<NumPicStCurr0 || (cIdx>=NumPicStCurr0+refPicSetInterLayer0.size() && cIdx<NumPicStCurr0+refPicSetInterLayer0.size()+NumPicStCurr1)) ? false : true;
#else
    m_bIsUsedAsLongTerm[0][rIdx] = ( cIdx >= NumPicStCurr0 && cIdx < NumPicStCurr0 + m_activeNumILRRefIdx ) || ( cIdx >= NumPicStCurr0 + NumPicStCurr1 + m_activeNumILRRefIdx );
#endif
#else
    m_bIsUsedAsLongTerm[REF_PIC_LIST_0][rIdx] = ( cIdx >= NumPicStCurr0 + NumPicStCurr1 );
#endif
  }
  if ( m_eSliceType != B_SLICE )
  {
    m_aiNumRefIdx[REF_PIC_LIST_1] = 0;
    ::memset( m_apcRefPicList[REF_PIC_LIST_1], 0, sizeof(m_apcRefPicList[REF_PIC_LIST_1]));
  }
  else
  {
    for (Int rIdx = 0; rIdx < m_aiNumRefIdx[REF_PIC_LIST_1]; rIdx ++)
    {
      cIdx = m_RefPicListModification.getRefPicListModificationFlagL1() ? m_RefPicListModification.getRefPicSetIdxL1(rIdx) : rIdx % numPicTotalCurr;
      assert(cIdx >= 0 && cIdx < numPicTotalCurr);
      m_apcRefPicList[REF_PIC_LIST_1][rIdx] = rpsCurrList1[ cIdx ];
#if VIEW_SCALABILITY 
      m_bIsUsedAsLongTerm[REF_PIC_LIST_1][rIdx] = (cIdx<NumPicStCurr1 || (cIdx>=NumPicStCurr1+refPicSetInterLayer1.size() && cIdx<NumPicStCurr1+refPicSetInterLayer1.size()+NumPicStCurr0)) ? false : true;
#else
      m_bIsUsedAsLongTerm[REF_PIC_LIST_1][rIdx] = ( cIdx >= NumPicStCurr0 + NumPicStCurr1 );
#endif
    }
  }
}
*/

Int TComSlice::getNumRpsCurrTempList() const
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
  for(UInt i=0; i < m_pRPS->getNumberOfNegativePictures()+ m_pRPS->getNumberOfPositivePictures() + m_pRPS->getNumberOfLongtermPictures(); i++)
  {
    if(m_pRPS->getUsed(i))
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

#if VIEW_SCALABILITY 
Int TComSlice::getNumRpsInterLayerX(Int li, TComPic** ilpPic)
{
  assert(li==0 || li==1);

  Int viewIdCurrLayerId  = m_pcVPS->getViewIdVal(m_pcVPS->getViewIndex(m_layerId) );
  Int viewId0            = m_pcVPS->getViewIdVal( 0 );
  Int numInterLayer0 = 0;
  Int numInterLayer1 = 0;

  // for( Int i=0; i < m_activeNumILRRefIdx; i++ )
  // {
  //   Int refLayerIdc = m_interLayerPredLayerIdc[i];       
  //   Int maxTidIlRefPicsPlus1 = m_pcVPS->getMaxTidIlRefPicsPlus1(ilpPic[refLayerIdc]->getSlice(0)->getLayerIdx(), getLayerIdx());

  //   if( ((Int)(ilpPic[refLayerIdc]->getSlice(0)->getTLayer())<=maxTidIlRefPicsPlus1-1) || (maxTidIlRefPicsPlus1==0 && ilpPic[refLayerIdc]->getSlice(0)->getRapPicFlag()) )
  //   {
  //     Int viewIdRefPicLayerIdi = m_pcVPS->getViewIdVal( ilpPic[refLayerIdc]->getSlice(0)->getLayerIdx() );
  //     Bool refPicSet0Flag = ( ( viewIdCurrLayerId <= viewId0 && viewIdCurrLayerId <= viewIdRefPicLayerIdi ) || ( viewIdCurrLayerId >= viewId0 && viewIdCurrLayerId >= viewIdRefPicLayerIdi ) );

  //     if( refPicSet0Flag )
  //     {
  //       numInterLayer0++;
  //     }
  //     else
  //     {
  //       numInterLayer1++;
  //     }
  //   }
  // }

  if( li==0 )
  {
    return numInterLayer0;
  }
  else
  {
    return numInterLayer1;
  }
}
#endif

Void TComSlice::initEqualRef()
{
  for (Int iDir = 0; iDir < NUM_REF_PIC_LIST_01; iDir++)
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

Void TComSlice::checkColRefIdx(UInt curSliceIdx, TComPic* pic)
{

  // Int i;
  // TComSlice* curSlice = pic->getSlice(curSliceIdx);
  // Int currColRefPOC =  curSlice->getRefPOC( RefPicList(1 - curSlice->getColFromL0Flag()), curSlice->getColRefIdx());
  // TComSlice* preSlice;
  // Int preColRefPOC;
  // for(i=curSliceIdx-1; i>=0; i--)
  // {
  //   preSlice = pic->getSlice(i);
  //   if(preSlice->getSliceType() != I_SLICE)
  //   {
  //     preColRefPOC  = preSlice->getRefPOC( RefPicList(1 - preSlice->getColFromL0Flag()), preSlice->getColRefIdx());
  //     if(currColRefPOC != preColRefPOC)
  //     {
  //       printf("Collocated_ref_idx shall always be the same for all slices of a coded picture!\n");
  //       exit(EXIT_FAILURE);
  //     }
  //     else
  //     {
  //       break;
  //     }
  //   }
  // }
}

Void TComSlice::checkCRA(const TComReferencePictureSet *pReferencePictureSet, Int& pocCRA, NalUnitType& associatedIRAPType, TComList<TComPic *>& rcListPic)
{
  // for(Int i = 0; i < pReferencePictureSet->getNumberOfNegativePictures()+pReferencePictureSet->getNumberOfPositivePictures(); i++)
  // {
  //   if(pocCRA < MAX_UINT && getPOC() > pocCRA)
  //   {
  //     assert(getPOC()+pReferencePictureSet->getDeltaPOC(i) >= pocCRA);
  //   }
  // }
  // for(Int i = pReferencePictureSet->getNumberOfNegativePictures()+pReferencePictureSet->getNumberOfPositivePictures(); i < pReferencePictureSet->getNumberOfPictures(); i++)
  // {
  //   if(pocCRA < MAX_UINT && getPOC() > pocCRA)
  //   {
  //     if (!pReferencePictureSet->getCheckLTMSBPresent(i))
  //     {
  //       assert(xGetLongTermRefPic(rcListPic, pReferencePictureSet->getPOC(i), false)->getPOC() >= pocCRA);
  //     }
  //     else
  //     {
  //       assert(pReferencePictureSet->getPOC(i) >= pocCRA);
  //     }
  //   }
  // }
  // if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP ) // IDR picture found
  // {
  //   pocCRA = getPOC();
  //   associatedIRAPType = getNalUnitType();
  // }
  // else if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA ) // CRA picture found
  // {
  //   pocCRA = getPOC();
  //   associatedIRAPType = getNalUnitType();
  // }
  // else if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
  //        || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
  //        || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP ) // BLA picture found
  // {
  //   pocCRA = getPOC();
  //   associatedIRAPType = getNalUnitType();
  // }
}

// /** Function for marking the reference pictures when an IDR/CRA/CRANT/BLA/BLANT is encountered.
//  * \param pocCRA POC of the CRA/CRANT/BLA/BLANT picture
//  * \param bRefreshPending flag indicating if a deferred decoding refresh is pending
//  * \param rcListPic reference to the reference picture list
//  * This function marks the reference pictures as "unused for reference" in the following conditions.
//  * If the nal_unit_type is IDR/BLA/BLANT, all pictures in the reference picture list
//  * are marked as "unused for reference"
//  *    If the nal_unit_type is BLA/BLANT, set the pocCRA to the temporal reference of the current picture.
//  * Otherwise
//  *    If the bRefreshPending flag is true (a deferred decoding refresh is pending) and the current
//  *    temporal reference is greater than the temporal reference of the latest CRA/CRANT/BLA/BLANT picture (pocCRA),
//  *    mark all reference pictures except the latest CRA/CRANT/BLA/BLANT picture as "unused for reference" and set
//  *    the bRefreshPending flag to false.
//  *    If the nal_unit_type is CRA/CRANT, set the bRefreshPending flag to true and pocCRA to the temporal
//  *    reference of the current picture.
//  * Note that the current picture is already placed in the reference list and its marking is not changed.
//  * If the current picture has a nal_ref_idc that is not 0, it will remain marked as "used for reference".
//  
#if NO_CLRAS_OUTPUT_FLAG
Void TComSlice::decodingRefreshMarking( TComList<TComPic*>& rcListPic, Bool noClrasOutputFlag, UInt smallestLayerId )
{
  // /*
  // if( !isIRAP() )
  // {
  //   return;
  // }

  // Int pocCurr = getPOC();
  // TComPic* rpcPic = NULL;

  // // When the current picture is an IRAP picture with nuh_layer_id equal to 0 and NoClrasOutputFlag is equal to 1, 
  // // all reference pictures with any value of nuh_layer_id currently in the DPB (if any) are marked as "unused for reference".
  // if (m_layerId == smallestLayerId && noClrasOutputFlag)
  // {
  //   // mark all pictures for all layers as not used for reference
  //   TComList<TComPic*>::iterator iterPic = rcListPic.begin();
  //   while( iterPic != rcListPic.end() )
  //   {
  //     rpcPic = *(iterPic);
  //     if( rpcPic->getPOC() != pocCurr )
  //     {
  //       rpcPic->getSlice(0)->setReferenced(false);
  //     }
  //     iterPic++;
  //   }
  // }

  // // When the current picture is an IRAP picture with NoRaslOutputFlag equal to 1, 
  // // all reference pictures with nuh_layer_id equal to currPicLayerId currently in the DPB (if any) are marked as "unused for reference".
  // if( m_noRaslOutputFlag )
  // {
  //   // mark all pictures of a current layer as not used for reference
  //   TComList<TComPic*>::iterator iterPic = rcListPic.begin();
  //   while( iterPic != rcListPic.end() )
  //   {
  //     rpcPic = *(iterPic);
  //     if( rpcPic->getPOC() != pocCurr && rpcPic->getLayerId() == m_layerId )
  //     {
  //       rpcPic->getSlice(0)->setReferenced(false);
  //     }
  //     iterPic++;
  //   }
  // }
}

Void TComSlice::decodingRefreshMarking(Int& pocCRA, Bool& bRefreshPending, TComList<TComPic*>& rcListPic, Bool noClrasOutputFlag, const bool bEfficientFieldIRAPEnabled)
#else
Void TComSlice::decodingRefreshMarking(Int& pocCRA, Bool& bRefreshPending, TComList<TComPic*>& rcListPic, const bool bEfficientFieldIRAPEnabled)
#endif
{
//   TComPic* rpcPic;
//   Int      pocCurr = getPOC();

//   if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
//     || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
//     || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
//     || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL
//     || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP )  // IDR or BLA picture
//   {
//     // mark all pictures as not used for reference
//     TComList<TComPic*>::iterator        iterPic       = rcListPic.begin();
//     while (iterPic != rcListPic.end())
//     {
//       rpcPic = *(iterPic);
//       rpcPic->setCurrSliceIdx(0);
// #if NO_CLRAS_OUTPUT_FLAG
//       if( noClrasOutputFlag )
//       {
//         rpcPic->getSlice(0)->setReferenced(false);  // all layers // TODO. This does not mark all layers
//       }
//       else
//       {
//         if( rpcPic->getLayerId() == m_layerId )
//         {
//           rpcPic->getSlice(0)->setReferenced(false);  // only current layer
//         }
//       }
// #else
//       if (rpcPic->getPOC() != pocCurr)
//       {
//         rpcPic->getSlice(0)->setReferenced(false);
//       }
// #endif
//       iterPic++;
//     }

// #if SVC_EXTENSION
//     m_pcPic->getSlice(0)->setReferenced(true);   // Mark the current picture back as refererced.
// #endif

//     if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
//       || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
//       || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP )
//     {
//       pocCRA = pocCurr;
//     }
//     if (bEfficientFieldIRAPEnabled)
//     {
//       bRefreshPending = true;
//     }
//   }
//   else // CRA or No DR
//   {
//     if(bEfficientFieldIRAPEnabled && (getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_N_LP || getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL))
//     {
//       if (bRefreshPending==true && pocCurr > m_iLastIDR) // IDR reference marking pending 
//       {
//         TComList<TComPic*>::iterator        iterPic       = rcListPic.begin();
//         while (iterPic != rcListPic.end())
//         {
//           rpcPic = *(iterPic);
//           if (rpcPic->getPOC() != pocCurr && rpcPic->getPOC() != m_iLastIDR)
//           {
//             rpcPic->getSlice(0)->setReferenced(false);
//           }
//           iterPic++;
//         }
//         bRefreshPending = false; 
//       }
//     }
//     else
//     {
//       if (bRefreshPending==true && pocCurr > pocCRA) // CRA reference marking pending
//       {
//         TComList<TComPic*>::iterator iterPic = rcListPic.begin();
//         while (iterPic != rcListPic.end())
//         {
//           rpcPic = *(iterPic);
//           if (rpcPic->getPOC() != pocCurr && rpcPic->getPOC() != pocCRA)
//           {
//             rpcPic->getSlice(0)->setReferenced(false);
//           }
//           iterPic++;
//         }
//         bRefreshPending = false;
//       }
//     }
//     if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA ) // CRA picture found
//     {
//       bRefreshPending = true;
//       pocCRA = pocCurr;
//     }
//   }
}

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
  m_ChromaQpAdjEnabled = pSrc->m_ChromaQpAdjEnabled;
  m_deblockingFilterDisable   = pSrc->m_deblockingFilterDisable;
  m_deblockingFilterOverrideFlag = pSrc->m_deblockingFilterOverrideFlag;
  m_deblockingFilterBetaOffsetDiv2 = pSrc->m_deblockingFilterBetaOffsetDiv2;
  m_deblockingFilterTcOffsetDiv2 = pSrc->m_deblockingFilterTcOffsetDiv2;

  for (i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    m_aiNumRefIdx[i]     = pSrc->m_aiNumRefIdx[i];
  }

  for (i = 0; i < MAX_NUM_REF; i++)
  {
    m_list1IdxToList0Idx[i] = pSrc->m_list1IdxToList0Idx[i];
  }

  m_bCheckLDC             = pSrc->m_bCheckLDC;
  m_iSliceQpDelta        = pSrc->m_iSliceQpDelta;
  for (UInt component = 0; component < MAX_NUM_COMPONENT; component++)
  {
    m_iSliceChromaQpDelta[component] = pSrc->m_iSliceChromaQpDelta[component];
  }
  for (i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    for (j = 0; j < MAX_NUM_REF; j++)
    {
      m_apcRefPicList[i][j]  = pSrc->m_apcRefPicList[i][j];
      m_aiRefPOCList[i][j]   = pSrc->m_aiRefPOCList[i][j];
      m_bIsUsedAsLongTerm[i][j] = pSrc->m_bIsUsedAsLongTerm[i][j];
    }
    m_bIsUsedAsLongTerm[i][MAX_NUM_REF] = pSrc->m_bIsUsedAsLongTerm[i][MAX_NUM_REF];
  }
  m_iDepth               = pSrc->m_iDepth;

  // referenced slice
  m_bRefenced            = pSrc->m_bRefenced;

  // access channel
#if SVC_EXTENSION
  m_pcVPS                      = pSrc->m_pcVPS;
  m_layerId                    = pSrc->m_layerId;

  // SHM: copy inter-layer prediction settings and list modification information from the previous slice. Actually, it can vary on a slice basis, but it is not supported by the current encoder.
  m_activeNumILRRefIdx         = pSrc->m_activeNumILRRefIdx;
  m_interLayerPredEnabledFlag  = pSrc->m_interLayerPredEnabledFlag;
  memcpy( m_interLayerPredLayerIdc, pSrc->m_interLayerPredLayerIdc, sizeof( m_interLayerPredLayerIdc ) );
  m_RefPicListModification     = pSrc->m_RefPicListModification;
#endif
  m_pRPS                = pSrc->m_pRPS;
  m_iLastIDR             = pSrc->m_iLastIDR;

  m_pcPic                = pSrc->m_pcPic;

  m_colFromL0Flag        = pSrc->m_colFromL0Flag;
  m_colRefIdx            = pSrc->m_colRefIdx;

  setLambdas(pSrc->getLambdas());

  for (i = 0; i < NUM_REF_PIC_LIST_01; i++)
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

  m_sliceMode                     = pSrc->m_sliceMode;
  m_sliceArgument                 = pSrc->m_sliceArgument;
  m_sliceCurStartCtuTsAddr        = pSrc->m_sliceCurStartCtuTsAddr;
  m_sliceCurEndCtuTsAddr          = pSrc->m_sliceCurEndCtuTsAddr;
  m_sliceIdx                      = pSrc->m_sliceIdx;
  m_sliceSegmentMode              = pSrc->m_sliceSegmentMode;
  m_sliceSegmentArgument          = pSrc->m_sliceSegmentArgument;
  m_sliceSegmentCurStartCtuTsAddr = pSrc->m_sliceSegmentCurStartCtuTsAddr;
  m_sliceSegmentCurEndCtuTsAddr   = pSrc->m_sliceSegmentCurEndCtuTsAddr;
  m_nextSlice                     = pSrc->m_nextSlice;
  m_nextSliceSegment              = pSrc->m_nextSliceSegment;

  for ( UInt e=0 ; e<NUM_REF_PIC_LIST_01 ; e++ )
  {
    for ( UInt n=0 ; n<MAX_NUM_REF ; n++ )
    {
      memcpy(m_weightPredTable[e][n], pSrc->m_weightPredTable[e][n], sizeof(WPScalingParam)*MAX_NUM_COMPONENT );
    }
  }

  for( UInt ch = 0 ; ch < MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_saoEnabledFlag[ch] = pSrc->m_saoEnabledFlag[ch];
  }

  m_cabacInitFlag                 = pSrc->m_cabacInitFlag;

  m_bLMvdL1Zero                   = pSrc->m_bLMvdL1Zero;
  m_LFCrossSliceBoundaryFlag      = pSrc->m_LFCrossSliceBoundaryFlag;
  m_enableTMVPFlag                = pSrc->m_enableTMVPFlag;
  m_maxNumMergeCand               = pSrc->m_maxNumMergeCand;
  m_encCABACTableIdx              = pSrc->m_encCABACTableIdx;
}


//  Function for setting the slice's temporal layer ID and corresponding temporal_layer_switching_point_flag.
// \param uiTLayer Temporal layer ID of the current slice
// The decoder calls this function to set temporal_layer_switching_point_flag for each temporal layer based on
// the SPS's temporal_id_nesting_flag and the parsed PPS.  Then, current slice's temporal layer ID and
// temporal_layer_switching_point_flag is set accordingly.
 
Void TComSlice::setTLayerInfo( UInt uiTLayer )
{
  m_uiTLayer = uiTLayer;
}

// Function for checking if this is a switching-point
Bool TComSlice::isTemporalLayerSwitchingPoint(TComList<TComPic*>& rcListPic)
{
  // TComPic* rpcPic;
  // // loop through all pictures in the reference picture buffer
  // TComList<TComPic*>::iterator iterPic = rcListPic.begin();
  // while ( iterPic != rcListPic.end())
  // {
  //   rpcPic = *(iterPic++);
  //   if(rpcPic->getSlice(0)->isReferenced() && rpcPic->getPOC() != getPOC())
  //   {
  //     if(rpcPic->getTLayer() >= getTLayer())
  //     {
  //       return false;
  //     }
  //   }
  // }
  return true;
}

// Function for checking if this is a STSA candidate
Bool TComSlice::isStepwiseTemporalLayerSwitchingPointCandidate(TComList<TComPic*>& rcListPic)
{
  // TComPic* rpcPic;

  // TComList<TComPic*>::iterator iterPic = rcListPic.begin();
  // while ( iterPic != rcListPic.end())
  // {
  //   rpcPic = *(iterPic++);
  //   if(rpcPic->getSlice(0)->isReferenced() &&  (rpcPic->getUsedByCurr()==true) && rpcPic->getPOC() != getPOC())
  //   {
  //     if(rpcPic->getTLayer() >= getTLayer())
  //     {
  //       return false;
  //     }
  //   }
  // }
  return true;
}

#if SVC_POC
Void TComSlice::checkLeadingPictureRestrictions(TComList<TComPic*>& rcListPic, Bool usePocBeforeReset)
#else
Void TComSlice::checkLeadingPictureRestrictions(TComList<TComPic*>& rcListPic)
#endif
{
//   TComPic* rpcPic;

//   Int nalUnitType = this->getNalUnitType();

//   // When a picture is a leading picture, it shall be a RADL or RASL picture.
//   if(this->getAssociatedIRAPPOC() > this->getPOC())
//   {
//     // Do not check IRAP pictures since they may get a POC lower than their associated IRAP
//     if(nalUnitType < NAL_UNIT_CODED_SLICE_BLA_W_LP ||
//        nalUnitType > NAL_UNIT_RESERVED_IRAP_VCL23)
//     {
//       assert(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||
//              nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R ||
//              nalUnitType == NAL_UNIT_CODED_SLICE_RADL_N ||
//              nalUnitType == NAL_UNIT_CODED_SLICE_RADL_R);
//     }
//   }

//   // When a picture is a trailing picture, it shall not be a RADL or RASL picture.
//   if(this->getAssociatedIRAPPOC() < this->getPOC())
//   {
//     assert(nalUnitType != NAL_UNIT_CODED_SLICE_RASL_N &&
//            nalUnitType != NAL_UNIT_CODED_SLICE_RASL_R &&
//            nalUnitType != NAL_UNIT_CODED_SLICE_RADL_N &&
//            nalUnitType != NAL_UNIT_CODED_SLICE_RADL_R);
//   }

//   // No RASL pictures shall be present in the bitstream that are associated
//   // with a BLA picture having nal_unit_type equal to BLA_W_RADL or BLA_N_LP.
//   if(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||
//      nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R)
//   {
//     assert(this->getAssociatedIRAPType() != NAL_UNIT_CODED_SLICE_BLA_W_RADL &&
//            this->getAssociatedIRAPType() != NAL_UNIT_CODED_SLICE_BLA_N_LP);
//   }

//   // No RASL pictures shall be present in the bitstream that are associated with
//   // an IDR picture.
//   if(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||
//      nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R)
//   {
//     assert(this->getAssociatedIRAPType() != NAL_UNIT_CODED_SLICE_IDR_N_LP   &&
//            this->getAssociatedIRAPType() != NAL_UNIT_CODED_SLICE_IDR_W_RADL);
//   }

//   // No RADL pictures shall be present in the bitstream that are associated with
//   // a BLA picture having nal_unit_type equal to BLA_N_LP or that are associated
//   // with an IDR picture having nal_unit_type equal to IDR_N_LP.
//   if(nalUnitType == NAL_UNIT_CODED_SLICE_RADL_N ||
//      nalUnitType == NAL_UNIT_CODED_SLICE_RADL_R)
//   {
//     assert(this->getAssociatedIRAPType() != NAL_UNIT_CODED_SLICE_BLA_N_LP   &&
//            this->getAssociatedIRAPType() != NAL_UNIT_CODED_SLICE_IDR_N_LP);
//   }

//   // loop through all pictures in the reference picture buffer
//   TComList<TComPic*>::iterator iterPic = rcListPic.begin();
//   while ( iterPic != rcListPic.end())
//   {
//     rpcPic = *(iterPic++);
//     if(!rpcPic->getReconMark())
//     {
//       continue;
//     }
//     if (rpcPic->getPOC() == this->getPOC())
//     {
//       continue;
//     }

//     // Any picture that has PicOutputFlag equal to 1 that precedes an IRAP picture
//     // in decoding order shall precede the IRAP picture in output order.
//     // (Note that any picture following in output order would be present in the DPB)
//     if(rpcPic->getSlice(0)->getPicOutputFlag() == 1 && !this->getNoOutputPriorPicsFlag())
//     {
//       if(nalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP    ||
//          nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_LP    ||
//          nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_RADL  ||
//          nalUnitType == NAL_UNIT_CODED_SLICE_CRA         ||
//          nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP    ||
//          nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL)
//       {
// #if SVC_POC
//         if( usePocBeforeReset )
//         {
//           assert(rpcPic->getSlice(0)->getPocValueBeforeReset() < this->getPocValueBeforeReset());
//         }
//         else
//         {
//           assert(rpcPic->getPOC() < this->getPOC());
//         }
// #else
//         assert(rpcPic->getPOC() < this->getPOC());
// #endif
//       }
//     }

//     // Any picture that has PicOutputFlag equal to 1 that precedes an IRAP picture
//     // in decoding order shall precede any RADL picture associated with the IRAP
//     // picture in output order.
//     if(rpcPic->getSlice(0)->getPicOutputFlag() == 1)
//     {
//       if((nalUnitType == NAL_UNIT_CODED_SLICE_RADL_N ||
//           nalUnitType == NAL_UNIT_CODED_SLICE_RADL_R))
//       {
//         // rpcPic precedes the IRAP in decoding order
//         if(this->getAssociatedIRAPPOC() > rpcPic->getSlice(0)->getAssociatedIRAPPOC())
//         {
//           // rpcPic must not be the IRAP picture
//           if(this->getAssociatedIRAPPOC() != rpcPic->getPOC())
//           {
// #if SVC_POC
//             if( usePocBeforeReset )
//             {
//               assert(rpcPic->getSlice(0)->getPocValueBeforeReset() < this->getPocValueBeforeReset());
//             }
//             else
//             {
//               assert(rpcPic->getPOC() < this->getPOC());
//             }
// #else
//             assert(rpcPic->getPOC() < this->getPOC());
// #endif
//           }
//         }
//       }
//     }

//     // When a picture is a leading picture, it shall precede, in decoding order,
//     // all trailing pictures that are associated with the same IRAP picture.
//       if(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||
//          nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R ||
//          nalUnitType == NAL_UNIT_CODED_SLICE_RADL_N ||
//          nalUnitType == NAL_UNIT_CODED_SLICE_RADL_R)
//       {
//         if(rpcPic->getSlice(0)->getAssociatedIRAPPOC() == this->getAssociatedIRAPPOC())
//         {
//           // rpcPic is a picture that preceded the leading in decoding order since it exist in the DPB
//           // rpcPic would violate the constraint if it was a trailing picture
// #if SVC_POC
//           if( usePocBeforeReset )
//           {
//             assert(rpcPic->getPOC() <= this->getAssociatedIrapPocBeforeReset());
//           }
//           else
//           {
//             assert(rpcPic->getPOC() <= this->getAssociatedIRAPPOC());
//           }
// #else
//           assert(rpcPic->getPOC() <= this->getAssociatedIRAPPOC());
// #endif
//         }
//       }

//     // Any RASL picture associated with a CRA or BLA picture shall precede any
//     // RADL picture associated with the CRA or BLA picture in output order
//     if(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||
//        nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R)
//     {
//       if((this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_BLA_N_LP   ||
//           this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_BLA_W_LP   ||
//           this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL ||
//           this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_CRA)       &&
//           this->getAssociatedIRAPPOC() == rpcPic->getSlice(0)->getAssociatedIRAPPOC())
//       {
//         if(rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N ||
//            rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_R)
//         {
//           assert(rpcPic->getPOC() > this->getPOC());
//         }
//       }
//     }

//     // Any RASL picture associated with a CRA picture shall follow, in output
//     // order, any IRAP picture that precedes the CRA picture in decoding order.
//     if(nalUnitType == NAL_UNIT_CODED_SLICE_RASL_N ||
//        nalUnitType == NAL_UNIT_CODED_SLICE_RASL_R)
//     {
//       if(this->getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_CRA)
//       {
//         if(rpcPic->getSlice(0)->getPOC() < this->getAssociatedIRAPPOC() &&
//            (rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP   ||
//             rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP   ||
//             rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL ||
//             rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP   ||
//             rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
//             rpcPic->getSlice(0)->getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA))
//         {
//           assert(this->getPOC() > rpcPic->getSlice(0)->getPOC());
//         }
//       }
//     }
//   }
}



// Function for applying picture marking based on the Reference Picture Set in pReferencePictureSet.
Void TComSlice::applyReferencePictureSet( TComList<TComPic*>& rcListPic, const TComReferencePictureSet *pReferencePictureSet)
{
//   TComPic* rpcPic;
//   Int i, isReference;

// #if !ALIGNED_BUMPING
//   checkLeadingPictureRestrictions(rcListPic);
// #endif

//   // loop through all pictures in the reference picture buffer
//   TComList<TComPic*>::iterator iterPic = rcListPic.begin();
//   while ( iterPic != rcListPic.end())
//   {
//     rpcPic = *(iterPic++);

//     if(!rpcPic->getSlice( 0 )->isReferenced())
//     {
//       continue;
//     }

//     isReference = 0;
//     // loop through all pictures in the Reference Picture Set
//     // to see if the picture should be kept as reference picture
//     for(i=0;i<pReferencePictureSet->getNumberOfPositivePictures()+pReferencePictureSet->getNumberOfNegativePictures();i++)
//     {
//       if(!rpcPic->getIsLongTerm() && rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getPOC() + pReferencePictureSet->getDeltaPOC(i))
//       {
//         isReference = 1;
//         rpcPic->setUsedByCurr(pReferencePictureSet->getUsed(i));
//         rpcPic->setIsLongTerm(0);
//       }
//     }
//     for(;i<pReferencePictureSet->getNumberOfPictures();i++)
//     {
//       if(pReferencePictureSet->getCheckLTMSBPresent(i)==true)
//       {
//         if(rpcPic->getIsLongTerm() && (rpcPic->getPicSym()->getSlice(0)->getPOC()) == pReferencePictureSet->getPOC(i))
//         {
//           isReference = 1;
//           rpcPic->setUsedByCurr(pReferencePictureSet->getUsed(i));
//         }
//       }
//       else
//       {
//         Int pocCycle = 1<<rpcPic->getPicSym()->getSlice(0)->getSPS()->getBitsForPOC();
//         Int curPoc = rpcPic->getPicSym()->getSlice(0)->getPOC() & (pocCycle-1);
//         Int refPoc = pReferencePictureSet->getPOC(i) & (pocCycle-1);
//         if(rpcPic->getIsLongTerm() && curPoc == refPoc)
//         {
//           isReference = 1;
//           rpcPic->setUsedByCurr(pReferencePictureSet->getUsed(i));
//         }
//       }

//     }

// #if SVC_EXTENSION
//     if( isReference ) // Current picture is in the temporal RPS
//     {
//       assert( rpcPic->getSlice(0)->getDiscardableFlag() == 0 ); // Temporal RPS shall not contain picture with discardable_flag equal to 1
//     }
// #endif

//     // mark the picture as "unused for reference" if it is not in
//     // the Reference Picture Set
//     if(rpcPic->getPicSym()->getSlice(0)->getPOC() != this->getPOC() && isReference == 0)
//     {
//       rpcPic->getSlice( 0 )->setReferenced( false );
//       rpcPic->setUsedByCurr(0);
//       rpcPic->setIsLongTerm(0);
//     }
//     //check that pictures of higher temporal layers are not used
//     assert(rpcPic->getSlice( 0 )->isReferenced()==0||rpcPic->getUsedByCurr()==0||rpcPic->getTLayer()<=this->getTLayer());
//     //check that pictures of higher or equal temporal layer are not in the RPS if the current picture is a TSA picture
//     if(this->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_R || this->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_N)
//     {
//       assert(rpcPic->getSlice( 0 )->isReferenced()==0||rpcPic->getTLayer()<this->getTLayer());
//     }
//     //check that pictures marked as temporal layer non-reference pictures are not used for reference
//     if(rpcPic->getPicSym()->getSlice(0)->getPOC() != this->getPOC() && rpcPic->getTLayer()==this->getTLayer())
//     {
//       assert(rpcPic->getSlice( 0 )->isReferenced()==0||rpcPic->getUsedByCurr()==0||rpcPic->getSlice( 0 )->getTemporalLayerNonReferenceFlag()==false);
//     }
//   }
}

// Function for applying picture marking based on the Reference Picture Set in pReferencePictureSet.
Int TComSlice::checkThatAllRefPicsAreAvailable( TComList<TComPic*>& rcListPic, const TComReferencePictureSet *pReferencePictureSet, Bool printErrors, Int pocRandomAccess, Bool bUseRecoveryPoint)
{
//   Int atLeastOneUnabledByRecoveryPoint = 0;
//   Int atLeastOneFlushedByPreviousIDR = 0;
//   TComPic* rpcPic;
//   Int i, isAvailable;
//   Int atLeastOneLost = 0;
//   Int atLeastOneRemoved = 0;
//   Int iPocLost = 0;

//   // loop through all long-term pictures in the Reference Picture Set
//   // to see if the picture should be kept as reference picture
//   for(i=pReferencePictureSet->getNumberOfNegativePictures()+pReferencePictureSet->getNumberOfPositivePictures();i<pReferencePictureSet->getNumberOfPictures();i++)
//   {
//     isAvailable = 0;
//     // loop through all pictures in the reference picture buffer
//     TComList<TComPic*>::iterator iterPic = rcListPic.begin();
//     while ( iterPic != rcListPic.end())
//     {
//       rpcPic = *(iterPic++);
//       if(pReferencePictureSet->getCheckLTMSBPresent(i)==true)
//       {
//         if(rpcPic->getIsLongTerm() && (rpcPic->getPicSym()->getSlice(0)->getPOC()) == pReferencePictureSet->getPOC(i) && rpcPic->getSlice(0)->isReferenced())
//         {
//           if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess)
//           {
//             isAvailable = 0;
//           }
//           else
//           {
//             isAvailable = 1;
//           }
//         }
//       }
//       else
//       {
//         Int pocCycle = 1<<rpcPic->getPicSym()->getSlice(0)->getSPS()->getBitsForPOC();
//         Int curPoc = rpcPic->getPicSym()->getSlice(0)->getPOC() & (pocCycle-1);
//         Int refPoc = pReferencePictureSet->getPOC(i) & (pocCycle-1);
//         if(rpcPic->getIsLongTerm() && curPoc == refPoc && rpcPic->getSlice(0)->isReferenced())
//         {
//           if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess)
//           {
//             isAvailable = 0;
//           }
//           else
//           {
//             isAvailable = 1;
//           }
//         }
//       }
//     }
//     // if there was no such long-term check the short terms
//     if(!isAvailable)
//     {
//       iterPic = rcListPic.begin();
//       while ( iterPic != rcListPic.end())
//       {
//         rpcPic = *(iterPic++);

//         Int pocCycle = 1 << rpcPic->getPicSym()->getSlice(0)->getSPS()->getBitsForPOC();
//         Int curPoc = rpcPic->getPicSym()->getSlice(0)->getPOC();
//         Int refPoc = pReferencePictureSet->getPOC(i);
//         if (!pReferencePictureSet->getCheckLTMSBPresent(i))
//         {
//           curPoc = curPoc & (pocCycle - 1);
//           refPoc = refPoc & (pocCycle - 1);
//         }

//         if (rpcPic->getSlice(0)->isReferenced() && curPoc == refPoc)
//         {
//           if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess)
//           {
//             isAvailable = 0;
//           }
//           else
//           {
//             isAvailable = 1;
//             rpcPic->setIsLongTerm(1);
//             break;
//           }
//         }
//       }
//     }
//     // report that a picture is lost if it is in the Reference Picture Set
//     // but not available as reference picture
//     if(isAvailable == 0)
//     {
//       if (this->getPOC() + pReferencePictureSet->getDeltaPOC(i) >= pocRandomAccess)
//       {
//         if(!pReferencePictureSet->getUsed(i) )
//         {
//           if(printErrors)
//           {
//             printf("\nLong-term reference picture with POC = %3d seems to have been removed or not correctly decoded.", this->getPOC() + pReferencePictureSet->getDeltaPOC(i));
//           }
//           atLeastOneRemoved = 1;
//         }
//         else
//         {
//           if(printErrors)
//           {
//             printf("\nLong-term reference picture with POC = %3d is lost or not correctly decoded!", this->getPOC() + pReferencePictureSet->getDeltaPOC(i));
//           }
//           atLeastOneLost = 1;
//           iPocLost=this->getPOC() + pReferencePictureSet->getDeltaPOC(i);
//         }
//       }
//       else if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess)
//       {
//         atLeastOneUnabledByRecoveryPoint = 1;
//       }
//       else if(bUseRecoveryPoint && (this->getAssociatedIRAPType()==NAL_UNIT_CODED_SLICE_IDR_N_LP || this->getAssociatedIRAPType()==NAL_UNIT_CODED_SLICE_IDR_W_RADL))
//       {
//         atLeastOneFlushedByPreviousIDR = 1;
//       }
//     }
//   }
//   // loop through all short-term pictures in the Reference Picture Set
//   // to see if the picture should be kept as reference picture
//   for(i=0;i<pReferencePictureSet->getNumberOfNegativePictures()+pReferencePictureSet->getNumberOfPositivePictures();i++)
//   {
//     isAvailable = 0;
//     // loop through all pictures in the reference picture buffer
//     TComList<TComPic*>::iterator iterPic = rcListPic.begin();
//     while ( iterPic != rcListPic.end())
//     {
//       rpcPic = *(iterPic++);

//       if(!rpcPic->getIsLongTerm() && rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getPOC() + pReferencePictureSet->getDeltaPOC(i) && rpcPic->getSlice(0)->isReferenced())
//       {
//         if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess)
//         {
//           isAvailable = 0;
//         }
//         else
//         {
//           isAvailable = 1;
//         }
//       }
//     }
//     // report that a picture is lost if it is in the Reference Picture Set
//     // but not available as reference picture
//     if(isAvailable == 0)
//     {
// #if !UNAVAILABLE_PIC_BUGFIX
//       if (this->getPOC() + pReferencePictureSet->getDeltaPOC(i) >= pocRandomAccess)
// #endif
//       {
//         if(!pReferencePictureSet->getUsed(i) )
//         {
//           if(printErrors)
//           {
//             printf("\nShort-term reference picture with POC = %3d seems to have been removed or not correctly decoded.", this->getPOC() + pReferencePictureSet->getDeltaPOC(i));
//           }
//           atLeastOneRemoved = 1;
//         }
//         else
//         {
//           if(printErrors)
//           {
//             printf("\nShort-term reference picture with POC = %3d is lost or not correctly decoded!", this->getPOC() + pReferencePictureSet->getDeltaPOC(i));
//           }
//           atLeastOneLost = 1;
//           iPocLost=this->getPOC() + pReferencePictureSet->getDeltaPOC(i);
//         }
//       }
// #if UNAVAILABLE_PIC_BUGFIX
//       if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess)
// #else
//       else if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess)
// #endif
//       {
//         atLeastOneUnabledByRecoveryPoint = 1;
//       }
//       else if(bUseRecoveryPoint && (this->getAssociatedIRAPType()==NAL_UNIT_CODED_SLICE_IDR_N_LP || this->getAssociatedIRAPType()==NAL_UNIT_CODED_SLICE_IDR_W_RADL))
//       {
//         atLeastOneFlushedByPreviousIDR = 1;
//       }
//     }
//   }

//   if(atLeastOneUnabledByRecoveryPoint || atLeastOneFlushedByPreviousIDR)
//   {
//     return -1;
//   }    
//   if(atLeastOneLost)
//   {
//     return iPocLost+1;
//   }
//   if(atLeastOneRemoved)
//   {
//     return -2;
//   }
//   else
//   {
//     return 0;
//   }
return 0;
}

// Function for constructing an explicit Reference Picture Set out of the available pictures in a referenced Reference Picture Set
Void TComSlice::createExplicitReferencePictureSetFromReference( TComList<TComPic*>& rcListPic, const TComReferencePictureSet *pReferencePictureSet, Bool isRAP, Int pocRandomAccess, Bool bUseRecoveryPoint, const Bool bEfficientFieldIRAPEnabled)
{
//   TComPic* rpcPic;
//   Int i, j;
//   Int k = 0;
//   Int nrOfNegativePictures = 0;
//   Int nrOfPositivePictures = 0;
//   TComReferencePictureSet* pLocalRPS = this->getLocalRPS();
//   (*pLocalRPS)=TComReferencePictureSet();

//   Bool irapIsInRPS = false; // Used when bEfficientFieldIRAPEnabled==true
// #if SVC_POC
//   Bool pocsAdjusted = false;
// #endif

//   // loop through all pictures in the Reference Picture Set
//   for(i=0;i<pReferencePictureSet->getNumberOfPictures();i++)
//   {
//     j = 0;
//     // loop through all pictures in the reference picture buffer
//     TComList<TComPic*>::iterator iterPic = rcListPic.begin();
//     while ( iterPic != rcListPic.end())
//     {
//       j++;
//       rpcPic = *(iterPic++);

// #if SVC_POC
//       // poc adjustement by poc reset needs to be taken into account here
//       Int deltaPOC = pReferencePictureSet->getDeltaPOC(i) - rpcPic->getPicSym()->getSlice(0)->getPocResetDeltaPoc();
//       if (rpcPic->getPicSym()->getSlice(0)->getPocResetDeltaPoc() != 0)
//       {
//         pocsAdjusted = true;
//       }

//       if (rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getPOC() + deltaPOC && rpcPic->getSlice(0)->isReferenced())
// #else
//       if(rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getPOC() + pReferencePictureSet->getDeltaPOC(i) && rpcPic->getSlice(0)->isReferenced())
// #endif
//       {
//         // This picture exists as a reference picture
//         // and should be added to the explicit Reference Picture Set
// #if SVC_POC
//         pLocalRPS->setDeltaPOC(k, deltaPOC);
// #else
//         pLocalRPS->setDeltaPOC(k, pReferencePictureSet->getDeltaPOC(i));
// #endif
//         pLocalRPS->setUsed(k, pReferencePictureSet->getUsed(i) && (!isRAP));
//         if (bEfficientFieldIRAPEnabled)
//         {
// #if SVC_POC
//           pLocalRPS->setUsed(k, pLocalRPS->getUsed(k) && !(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + deltaPOC < pocRandomAccess) );
// #else
//           pLocalRPS->setUsed(k, pLocalRPS->getUsed(k) && !(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess) );
// #endif
//         }

//         if(pLocalRPS->getDeltaPOC(k) < 0)
//         {
//           nrOfNegativePictures++;
//         }
//         else
//         {
//           if(bEfficientFieldIRAPEnabled && rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getAssociatedIRAPPOC() && this->getAssociatedIRAPPOC() == this->getPOC()+1)
//           {
//             irapIsInRPS = true;
//           }
//           nrOfPositivePictures++;
//         }
//         k++;
//       }
//     }
//   }

//   Bool useNewRPS = false;
//   // if current picture is complimentary field associated to IRAP, add the IRAP to its RPS. 
//   if(bEfficientFieldIRAPEnabled && m_pcPic->isField() && !irapIsInRPS)
//   {
//     TComList<TComPic*>::iterator iterPic = rcListPic.begin();
//     while ( iterPic != rcListPic.end())
//     {
//       rpcPic = *(iterPic++);
//       if(rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getAssociatedIRAPPOC() && this->getAssociatedIRAPPOC() == this->getPOC()+1)
//       {
//         pLocalRPS->setDeltaPOC(k, 1);
//         pLocalRPS->setUsed(k, true);
//         nrOfPositivePictures++;
//         k ++;
//         useNewRPS = true;
//       }
//     }
//   }
//   pLocalRPS->setNumberOfNegativePictures(nrOfNegativePictures);
//   pLocalRPS->setNumberOfPositivePictures(nrOfPositivePictures);
//   pLocalRPS->setNumberOfPictures(nrOfNegativePictures+nrOfPositivePictures);
//   // This is a simplistic inter rps example. A smarter encoder will look for a better reference RPS to do the
//   // inter RPS prediction with.  Here we just use the reference used by pReferencePictureSet.
//   // If pReferencePictureSet is not inter_RPS_predicted, then inter_RPS_prediction is for the current RPS also disabled.
//   if (!pReferencePictureSet->getInterRPSPrediction() || useNewRPS
// #if SVC_POC
//     || pocsAdjusted  // inter RPS prediction does not work if POCs have been adjusted
// #endif
//     )
//   {
//     pLocalRPS->setInterRPSPrediction(false);
//     pLocalRPS->setNumRefIdc(0);
//   }
//   else
//   {
//     Int rIdx =  this->getRPSidx() - pReferencePictureSet->getDeltaRIdxMinus1() - 1;
//     Int deltaRPS = pReferencePictureSet->getDeltaRPS();
//     const TComReferencePictureSet* pcRefRPS = this->getSPS()->getRPSList()->getReferencePictureSet(rIdx);
//     Int iRefPics = pcRefRPS->getNumberOfPictures();
//     Int iNewIdc=0;
//     for(i=0; i<= iRefPics; i++)
//     {
//       Int deltaPOC = ((i != iRefPics)? pcRefRPS->getDeltaPOC(i) : 0);  // check if the reference abs POC is >= 0
//       Int iRefIdc = 0;
//       for (j=0; j < pLocalRPS->getNumberOfPictures(); j++) // loop through the  pictures in the new RPS
//       {
//         if ( (deltaPOC + deltaRPS) == pLocalRPS->getDeltaPOC(j))
//         {
//           if (pLocalRPS->getUsed(j))
//           {
//             iRefIdc = 1;
//           }
//           else
//           {
//             iRefIdc = 2;
//           }
//         }
//       }
//       pLocalRPS->setRefIdc(i, iRefIdc);
//       iNewIdc++;
//     }
//     pLocalRPS->setInterRPSPrediction(true);
//     pLocalRPS->setNumRefIdc(iNewIdc);
//     pLocalRPS->setDeltaRPS(deltaRPS);
//     pLocalRPS->setDeltaRIdxMinus1(pReferencePictureSet->getDeltaRIdxMinus1() + this->getSPS()->getRPSList()->getNumberOfReferencePictureSets() - this->getRPSidx());
//   }

//   this->setRPS(pLocalRPS);
//   this->setRPSidx(-1);
}

//! get AC and DC values for weighted pred
Void  TComSlice::getWpAcDcParam(WPACDCParam *&wp)
{
  wp = m_weightACDCParam;
}

//! init AC and DC values for weighted pred
Void  TComSlice::initWpAcDcParam()
{
  for(Int iComp = 0; iComp < MAX_NUM_COMPONENT; iComp++ )
  {
    m_weightACDCParam[iComp].iAC = 0;
    m_weightACDCParam[iComp].iDC = 0;
  }
}


//! get tables for weighted prediction
Void  TComSlice::getWpScaling( RefPicList e, Int iRefIdx, WPScalingParam *&wp )
{
  assert (e<NUM_REF_PIC_LIST_01);
  wp = m_weightPredTable[e][iRefIdx];
}

//! reset Default WP tables settings : no weight.
Void  TComSlice::resetWpScaling()
{
  for ( Int e=0 ; e<NUM_REF_PIC_LIST_01 ; e++ )
  {
    for ( Int i=0 ; i<MAX_NUM_REF ; i++ )
    {
      for ( Int yuv=0 ; yuv<MAX_NUM_COMPONENT ; yuv++ )
      {
        WPScalingParam  *pwp = &(m_weightPredTable[e][i][yuv]);
        pwp->bPresentFlag      = false;
        pwp->uiLog2WeightDenom = 0;
        pwp->uiLog2WeightDenom = 0;
        pwp->iWeight           = 1;
        pwp->iOffset           = 0;
      }
    }
  }
}

//! init WP table
Void  TComSlice::initWpScaling(const TComSPS *sps)
{
  const Bool bUseHighPrecisionPredictionWeighting = sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag();
  for ( Int e=0 ; e<NUM_REF_PIC_LIST_01 ; e++ )
  {
    for ( Int i=0 ; i<MAX_NUM_REF ; i++ )
    {
      for ( Int yuv=0 ; yuv<MAX_NUM_COMPONENT ; yuv++ )
      {
        WPScalingParam  *pwp = &(m_weightPredTable[e][i][yuv]);
        if ( !pwp->bPresentFlag )
        {
          // Inferring values not present :
          pwp->iWeight = (1 << pwp->uiLog2WeightDenom);
          pwp->iOffset = 0;
        }

        const Int offsetScalingFactor = bUseHighPrecisionPredictionWeighting ? 1 : (1 << (sps->getBitDepth(toChannelType(ComponentID(yuv)))-8));

        pwp->w      = pwp->iWeight;
        pwp->o      = pwp->iOffset * offsetScalingFactor; //NOTE: This value of the ".o" variable is never used - .o is set immediately before it gets used
        pwp->shift  = pwp->uiLog2WeightDenom;
        pwp->round  = (pwp->uiLog2WeightDenom>=1) ? (1 << (pwp->uiLog2WeightDenom-1)) : (0);
      }
    }
  }
}


// ------------------------------------------------------------------------------------------------
// Video parameter set (VPS)
// ------------------------------------------------------------------------------------------------
#if SVC_EXTENSION
TComVPS::TComVPS()
: m_VPSId                     (  0)
, m_uiMaxTLayers              (  1)
, m_uiMaxLayers               (  1)
, m_bTemporalIdNestingFlag    (false)
, m_numHrdParameters          (  0)
, m_hrdParameters             ()
, m_hrdOpSetIdx               ()
, m_cprmsPresentFlag          ()
, m_baseLayerInternalFlag     (true)
, m_baseLayerAvailableFlag    (true)
, m_maxLayerId                (0)
, m_numLayerSets              (0)
, m_numOutputLayerSets        (0)  
, m_numProfileTierLevel       (0)
, m_numAddOutputLayerSets     (0)
, m_defaultTargetOutputLayerIdc(0)
, m_bitRatePresentVpsFlag     (false)
, m_picRatePresentVpsFlag     (false)
, m_repFormatIdxPresentFlag   (false)
, m_vpsNumRepFormats          (1)
, m_viewIdLen                (0)
, m_vpsNonVuiExtLength (0)
, m_vpsPocLsbAlignedFlag(false)
{
  for( Int i = 0; i < MAX_TLAYER; i++)
  {
    m_numReorderPics[i] = 0;
    m_uiMaxDecPicBuffering[i] = 1;
    m_uiMaxLatencyIncrease[i] = 0;
  }

  m_nonHEVCBaseLayerFlag = false;
  m_splittingFlag = false;
  m_nuhLayerIdPresentFlag = false;
  ::memset(m_scalabilityMask, 0, sizeof(m_scalabilityMask));
  ::memset(m_dimensionIdLen, 0, sizeof(m_dimensionIdLen));
  ::memset(m_layerIdInNuh, 0, sizeof(m_layerIdInNuh));
  ::memset(m_dimensionId, 0, sizeof(m_dimensionId));

  m_numScalabilityTypes = 0;
  ::memset(m_layerIdxInVps, 0, sizeof(m_layerIdxInVps));
  ::memset(m_profilePresentFlag, 0, sizeof(m_profilePresentFlag));

  ::memset(m_layerIdIncludedFlag, 0, sizeof(m_layerIdIncludedFlag));
  // Consider dynamic allocation for outputLayerSetIdx and outputLayerFlag
  ::memset(m_outputLayerSetIdx, 0, sizeof(m_outputLayerSetIdx));
  ::memset(m_outputLayerFlag, 0, sizeof(m_outputLayerFlag));

  ::memset(m_directDependencyFlag, false, sizeof(m_directDependencyFlag));
  ::memset(m_numDirectRefLayers,   0, sizeof(m_numDirectRefLayers  ));
  ::memset(m_refLayerId,           0, sizeof(m_refLayerId          ));
  m_directDepTypeLen = 2;
  ::memset(m_directDependencyType, 0, sizeof(m_directDependencyType));

  m_maxOneActiveRefLayerFlag = true;
  ::memset(m_pocLsbNotPresentFlag, 0, sizeof(m_pocLsbNotPresentFlag));
  m_crossLayerPictureTypeAlignFlag = true;
  m_crossLayerIrapAlignFlag = true;
  m_crossLayerAlignedIdrOnlyFlag = false;
  m_maxTidRefPresentFlag = true;
  for( Int i = 0; i < MAX_VPS_LAYER_IDX_PLUS1 - 1; i++)
  {
    for( Int j = 0; j < MAX_VPS_LAYER_IDX_PLUS1; j++)
    {
      m_maxTidIlRefPicsPlus1[i][j] = m_uiMaxTLayers + 1;
    }
  }

  m_tilesNotInUseFlag = true;
  m_wppNotInUseFlag = true;
  ::memset(m_tilesInUseFlag,  0, sizeof(m_tilesInUseFlag));
  ::memset(m_loopFilterNotAcrossTilesFlag,  0, sizeof(m_loopFilterNotAcrossTilesFlag));
  ::memset(m_tileBoundariesAlignedFlag,  0, sizeof(m_tileBoundariesAlignedFlag));
  ::memset(m_wppInUseFlag,  0, sizeof(m_wppInUseFlag));

  m_ilpRestrictedRefLayersFlag = false;
  ::memset(m_minSpatialSegmentOffsetPlus1,  0, sizeof(m_minSpatialSegmentOffsetPlus1));
  ::memset(m_ctuBasedOffsetEnabledFlag,     0, sizeof(m_ctuBasedOffsetEnabledFlag));
  ::memset(m_minHorizontalCtuOffsetPlus1,   0, sizeof(m_minHorizontalCtuOffsetPlus1));

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

  ::memset( m_bitRatePresentFlag, 0, sizeof(m_bitRatePresentFlag) );
  ::memset( m_picRatePresentFlag, 0, sizeof(m_picRatePresentFlag) );
  ::memset( m_avgBitRate        , 0, sizeof(m_avgBitRate)         );
  ::memset( m_maxBitRate        , 0, sizeof(m_maxBitRate)         );
  ::memset( m_constPicRateIdc   , 0, sizeof(m_constPicRateIdc)    );
  ::memset( m_avgPicRate        , 0, sizeof(m_avgPicRate)         );
  ::memset( m_vpsRepFormatIdx   , 0, sizeof(m_vpsRepFormatIdx)    );
  ::memset( m_viewIdVal         , 0, sizeof(m_viewIdVal))         ;

  for( Int i = 0; i < MAX_NUM_LAYER_IDS; i++ )
  {
    m_numberRefLayers[i] = 0;
    for( Int j = 0; j < MAX_NUM_LAYER_IDS; j++ )
    {
      m_recursiveRefLayerFlag[i][j] = 0;
    }
  }

  ::memset( m_subLayerFlagInfoPresentFlag,  0, sizeof(m_subLayerFlagInfoPresentFlag ) );
  ::memset( m_subLayerDpbInfoPresentFlag,   0, sizeof(m_subLayerDpbInfoPresentFlag )  );
  ::memset( m_maxVpsDecPicBufferingMinus1,  0, sizeof(m_maxVpsDecPicBufferingMinus1 ) );
  ::memset( m_maxVpsNumReorderPics,         0, sizeof(m_maxVpsNumReorderPics )        );
  ::memset( m_maxVpsLatencyIncreasePlus1,   0, sizeof(m_maxVpsLatencyIncreasePlus1 )  );
  ::memset( m_numSubDpbs                ,   0, sizeof(m_numSubDpbs)                   );
  ::memset( m_baseLayerPSCompatibilityFlag, 0, sizeof(m_baseLayerPSCompatibilityFlag) );
}
#else
TComVPS::TComVPS()
: m_VPSId                     (  0)
, m_uiMaxTLayers              (  1)
, m_uiMaxLayers               (  1)
, m_bTemporalIdNestingFlag    (false)
, m_numHrdParameters          (  0)
, m_maxNuhReservedZeroLayerId (  0)
, m_hrdParameters             ()
, m_hrdOpSetIdx               ()
, m_cprmsPresentFlag          ()
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
}

// ------------------------------------------------------------------------------------------------
// Sequence parameter set (SPS)
// ------------------------------------------------------------------------------------------------

TComSPSRExt::TComSPSRExt()
 : m_transformSkipRotationEnabledFlag   (false)
 , m_transformSkipContextEnabledFlag    (false)
// m_rdpcmEnabledFlag initialized below
 , m_extendedPrecisionProcessingFlag    (false)
 , m_intraSmoothingDisabledFlag         (false)
 , m_highPrecisionOffsetsEnabledFlag    (false)
 , m_persistentRiceAdaptationEnabledFlag(false)
 , m_cabacBypassAlignmentEnabledFlag    (false)
{
  for (UInt signallingModeIndex = 0; signallingModeIndex < NUMBER_OF_RDPCM_SIGNALLING_MODES; signallingModeIndex++)
  {
    m_rdpcmEnabledFlag[signallingModeIndex] = false;
  }
}

TComSPS::TComSPS()
: m_SPSId                     (  0)
, m_VPSId                     (  0)
, m_chromaFormatIdc           (CHROMA_420)
, m_uiMaxTLayers              (  1)
// Structure
, m_picWidthInLumaSamples     (352)
, m_picHeightInLumaSamples    (288)
, m_log2MinCodingBlockSize    (  0)
, m_log2DiffMaxMinCodingBlockSize(0)
, m_uiMaxCUWidth              ( 32)
, m_uiMaxCUHeight             ( 32)
, m_uiMaxTotalCUDepth         (  3)
, m_bLongTermRefsPresent      (false)
, m_uiQuadtreeTULog2MaxSize   (  0)
, m_uiQuadtreeTULog2MinSize   (  0)
, m_uiQuadtreeTUMaxDepthInter (  0)
, m_uiQuadtreeTUMaxDepthIntra (  0)
// Tool list
, m_usePCM                    (false)
, m_pcmLog2MaxSize            (  5)
, m_uiPCMLog2MinSize          (  7)
, m_bPCMFilterDisableFlag     (false)
, m_uiBitsForPOC              (  8)
, m_numLongTermRefPicSPS      (  0)
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
, m_bV1CompatibleSPSFlag      (  0)
, m_updateRepFormatFlag       (false)
, m_updateRepFormatIndex      (0)
, m_inferScalingListFlag      ( false )
, m_scalingListRefLayerId     ( 0 )
#endif //SVC_EXTENSION
{
  for(Int ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_bitDepths.recon[ch] = 8;
#if O0043_BEST_EFFORT_DECODING
    m_bitDepths.stream[ch] = 8;
#endif
    m_pcmBitDepths[ch] = 8;
    m_qpBDOffset   [ch] = 0;
  }

  for ( Int i = 0; i < MAX_TLAYER; i++ )
  {
    m_uiMaxLatencyIncreasePlus1[i] = 0;
    m_uiMaxDecPicBuffering[i] = 1;
    m_numReorderPics[i]       = 0;
  }

  ::memset(m_ltRefPicPocLsbSps, 0, sizeof(m_ltRefPicPocLsbSps));
  ::memset(m_usedByCurrPicLtSPSFlag, 0, sizeof(m_usedByCurrPicLtSPSFlag));
}

TComSPS::~TComSPS()
{
  m_RPSList.destroy();
}

Void  TComSPS::createRPSList( Int numRPS )
{
  m_RPSList.destroy();
  m_RPSList.create(numRPS);
}


const Int TComSPS::m_winUnitX[]={1,2,2,1};
const Int TComSPS::m_winUnitY[]={1,2,1,1};

TComPPSRExt::TComPPSRExt()
: m_log2MaxTransformSkipBlockSize      (2)
, m_crossComponentPredictionEnabledFlag(false)
, m_diffCuChromaQpOffsetDepth          (0)
, m_chromaQpOffsetListLen              (0)
// m_ChromaQpAdjTableIncludingNullEntry initialized below
// m_log2SaoOffsetScale initialized below
{
  m_ChromaQpAdjTableIncludingNullEntry[0].u.comp.CbOffset = 0; // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0. This is initialised here and never subsequently changed.
  m_ChromaQpAdjTableIncludingNullEntry[0].u.comp.CrOffset = 0;
  for(Int ch=0; ch<MAX_NUM_CHANNEL_TYPE; ch++)
  {
    m_log2SaoOffsetScale[ch] = 0;
  }
}

TComPPS::TComPPS()
: m_PPSId                            (0)
, m_SPSId                            (0)
, m_picInitQPMinus26                 (0)
, m_useDQP                           (false)
, m_bConstrainedIntraPred            (false)
, m_bSliceChromaQpFlag               (false)
, m_uiMaxCuDQPDepth                  (0)
, m_chromaCbQpOffset                 (0)
, m_chromaCrQpOffset                 (0)
, m_numRefIdxL0DefaultActive         (1)
, m_numRefIdxL1DefaultActive         (1)
, m_TransquantBypassEnabledFlag      (false)
, m_useTransformSkip                 (false)
, m_dependentSliceSegmentsEnabledFlag(false)
, m_tilesEnabledFlag                 (false)
, m_entropyCodingSyncEnabledFlag     (false)
, m_loopFilterAcrossTilesEnabledFlag (true)
, m_uniformSpacingFlag               (false)
, m_numTileColumnsMinus1             (0)
, m_numTileRowsMinus1                (0)
, m_signDataHidingEnabledFlag        (false)
, m_cabacInitPresentFlag             (false)
, m_sliceHeaderExtensionPresentFlag  (false)
, m_loopFilterAcrossSlicesEnabledFlag(false)
, m_listsModificationPresentFlag     (0)
, m_numExtraSliceHeaderBits          (0)

#if SVC_EXTENSION
, m_extensionFlag(false)
, m_layerId(0)
, m_inferScalingListFlag ( false )
, m_scalingListRefLayerId ( 0 )
, m_pocResetInfoPresentFlag   (false)
, m_numRefLayerLocationOffsets  ( 0 )
#if CGS_3D_ASYMLUT
, m_nCGSFlag(0)
, m_nCGSOutputBitDepthY(0)
, m_nCGSOutputBitDepthC(0)
#endif
#endif //SVC_EXTENSION
{
#if SVC_EXTENSION
  ::memset(m_scaledRefLayerOffsetPresentFlag,   0, sizeof(m_scaledRefLayerOffsetPresentFlag));
  ::memset(m_refRegionOffsetPresentFlag,   0, sizeof(m_refRegionOffsetPresentFlag));
#endif //SVC_EXTENSION
}

TComPPS::~TComPPS()
{
}

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
  ::memset( m_bCheckLTMSB, 0, sizeof(m_bCheckLTMSB) );
  ::memset( m_pocLSBLT, 0, sizeof(m_pocLSBLT) );
  ::memset( m_deltaPOCMSBCycleLT, 0, sizeof(m_deltaPOCMSBCycleLT) );
  ::memset( m_deltaPocMSBPresentFlag, 0, sizeof(m_deltaPocMSBPresentFlag) );
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

Int TComReferencePictureSet::getUsed(Int bufferNum) const
{
  return m_used[bufferNum];
}

Int TComReferencePictureSet::getDeltaPOC(Int bufferNum) const
{
  return m_deltaPOC[bufferNum];
}

Int TComReferencePictureSet::getNumberOfPictures() const
{
  return m_numberOfPictures;
}

Int TComReferencePictureSet::getPOC(Int bufferNum) const
{
  return m_POC[bufferNum];
}

Void TComReferencePictureSet::setPOC(Int bufferNum, Int POC)
{
  m_POC[bufferNum] = POC;
}

Bool TComReferencePictureSet::getCheckLTMSBPresent(Int bufferNum) const
{
  return m_bCheckLTMSB[bufferNum];
}

Void TComReferencePictureSet::setCheckLTMSBPresent(Int bufferNum, Bool b)
{
  m_bCheckLTMSB[bufferNum] = b;
}

//! set the reference idc value at uiBufferNum entry to the value of iRefIdc
Void TComReferencePictureSet::setRefIdc(Int bufferNum, Int refIdc)
{
  m_refIdc[bufferNum] = refIdc;
}

//! get the reference idc value at uiBufferNum
Int  TComReferencePictureSet::getRefIdc(Int bufferNum) const
{
  return m_refIdc[bufferNum];
}

// Sorts the deltaPOC and Used by current values in the RPS based on the deltaPOC values.
// deltaPOC values are sorted with -ve values before the +ve values.  -ve values are in decreasing order.
// +ve values are in increasing order.
//\returns Void 
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

// Prints the deltaPOC and RefIdc (if available) values in the RPS.
// A "*" is added to the deltaPOC value if it is Used bu current.
// \returns Void
Void TComReferencePictureSet::printDeltaPOC() const
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

TComRefPicListModification::TComRefPicListModification()
: m_refPicListModificationFlagL0 (false)
, m_refPicListModificationFlagL1 (false)
{
  ::memset( m_RefPicSetIdxL0, 0, sizeof(m_RefPicSetIdxL0) );
  ::memset( m_RefPicSetIdxL1, 0, sizeof(m_RefPicSetIdxL1) );
}

TComRefPicListModification::~TComRefPicListModification()
{
}

TComScalingList::TComScalingList()
{
  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId = 0; listId < SCALING_LIST_NUM; listId++)
    {
      m_scalingListCoef[sizeId][listId].resize(min<Int>(MAX_MATRIX_COEF_NUM,(Int)gPccShvc_scalingListSize[sizeId]));
    }
  }
}

// set default quantization matrix to array
Void TComScalingList::setDefaultScalingList()
{
  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId=0;listId<SCALING_LIST_NUM;listId++)
    {
      processDefaultMatrix(sizeId, listId);
    }
  }
}
// check if use default quantization matrix
// \returns true if use default quantization matrix in all size
Bool TComScalingList::checkDefaultScalingList()
{
  UInt defaultCounter=0;

  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId=0;listId<SCALING_LIST_NUM;listId++)
    {
      if( !memcmp(getScalingListAddress(sizeId,listId), getScalingListDefaultAddress(sizeId, listId),sizeof(Int)*min(MAX_MATRIX_COEF_NUM,(Int)gPccShvc_scalingListSize[sizeId])) // check value of matrix
     && ((sizeId < SCALING_LIST_16x16) || (getScalingListDC(sizeId,listId) == 16))) // check DC value
      {
        defaultCounter++;
      }
    }
  }

  return (defaultCounter == (SCALING_LIST_NUM * SCALING_LIST_SIZE_NUM )) ? false : true;
}

// get scaling matrix from RefMatrixID
// \param sizeId    size index
// \param listId    index of input matrix
// \param refListId index of reference matrix
 
Void TComScalingList::processRefMatrix( UInt sizeId, UInt listId , UInt refListId )
{
  ::memcpy(getScalingListAddress(sizeId, listId),((listId == refListId)? getScalingListDefaultAddress(sizeId, refListId): getScalingListAddress(sizeId, refListId)),sizeof(Int)*min(MAX_MATRIX_COEF_NUM,(Int)gPccShvc_scalingListSize[sizeId]));
}

Void TComScalingList::checkPredMode(UInt sizeId, UInt listId)
{
  Int predListStep = (sizeId == SCALING_LIST_32x32? (SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) : 1); // if 32x32, skip over chroma entries.

  for(Int predListIdx = (Int)listId ; predListIdx >= 0; predListIdx-=predListStep)
  {
    if( !memcmp(getScalingListAddress(sizeId,listId),((listId == predListIdx) ?
      getScalingListDefaultAddress(sizeId, predListIdx): getScalingListAddress(sizeId, predListIdx)),sizeof(Int)*min(MAX_MATRIX_COEF_NUM,(Int)gPccShvc_scalingListSize[sizeId])) // check value of matrix
     && ((sizeId < SCALING_LIST_16x16) || (getScalingListDC(sizeId,listId) == getScalingListDC(sizeId,predListIdx)))) // check DC value
    {
      setRefMatrixId(sizeId, listId, predListIdx);
      setScalingListPredModeFlag(sizeId, listId, false);
      return;
    }
  }
  setScalingListPredModeFlag(sizeId, listId, true);
}

static Void outputScalingListHelp(std::ostream &os)
{
  os << "The scaling list file specifies all matrices and their DC values; none can be missing,\n"
         "but their order is arbitrary.\n\n"
         "The matrices are specified by:\n"
         "<matrix name><unchecked data>\n"
         "  <value>,<value>,<value>,....\n\n"
         "  Line-feeds can be added arbitrarily between values, and the number of values needs to be\n"
         "  at least the number of entries for the matrix (superfluous entries are ignored).\n"
         "  The <unchecked data> is text on the same line as the matrix that is not checked\n"
         "  except to ensure that the matrix name token is unique. It is recommended that it is ' ='\n"
         "  The values in the matrices are the absolute values (0-255), not the delta values as\n"
         "  exchanged between the encoder and decoder\n\n"
         "The DC values (for matrix sizes larger than 8x8) are specified by:\n"
         "<matrix name>_DC<unchecked data>\n"
         "  <value>\n";

  os << "The permitted matrix names are:\n";
  for(UInt sizeIdc = 0; sizeIdc < SCALING_LIST_SIZE_NUM; sizeIdc++)
  {
    for(UInt listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++)
    {
      if ((sizeIdc!=SCALING_LIST_32x32) || (listIdc%(SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) == 0))
      {
        os << "  " << gPccShvc_MatrixType[sizeIdc][listIdc] << '\n';
      }
    }
  }
}

Void TComScalingList::outputScalingLists(std::ostream &os) const
{
  for(UInt sizeIdc = 0; sizeIdc < SCALING_LIST_SIZE_NUM; sizeIdc++)
  {
    const UInt size = min(8,4<<(sizeIdc));
    for(UInt listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++)
    {
      if ((sizeIdc!=SCALING_LIST_32x32) || (listIdc%(SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) == 0))
      {
        const Int *src = getScalingListAddress(sizeIdc, listIdc);
        os << (gPccShvc_MatrixType[sizeIdc][listIdc]) << " =\n  ";
        for(UInt y=0; y<size; y++)
        {
          for(UInt x=0; x<size; x++, src++)
          {
            os << std::setw(3) << (*src) << ", ";
          }
          os << (y+1<size?"\n  ":"\n");
        }
        if(sizeIdc > SCALING_LIST_8x8)
        {
          os << gPccShvc_MatrixType_DC[sizeIdc][listIdc] << " = \n  " << std::setw(3) << getScalingListDC(sizeIdc, listIdc) << "\n";
        }
        os << "\n";
      }
    }
  }
}

Bool TComScalingList::xParseScalingList(const std::string &fileName)
{
  static const Int LINE_SIZE=1024;
  FILE *fp = NULL;
  TChar line[LINE_SIZE];

  if (fileName.empty())
  {
    fprintf(stderr, "Error: no scaling list file specified. Help on scaling lists being output\n");
    outputScalingListHelp(std::cout);
    std::cout << "\n\nExample scaling list file using default values:\n\n";
    outputScalingLists(std::cout);
    exit (1);
    return true;
  }
  else if ((fp = fopen(fileName.c_str(),"r")) == (FILE*)NULL)
  {
    fprintf(stderr, "Error: cannot open scaling list file %s for reading\n", fileName.c_str());
    return true;
  }

  for(UInt sizeIdc = 0; sizeIdc < SCALING_LIST_SIZE_NUM; sizeIdc++)
  {
    const UInt size = min(MAX_MATRIX_COEF_NUM,(Int)gPccShvc_scalingListSize[sizeIdc]);

    for(UInt listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++)
    {
      Int * const src = getScalingListAddress(sizeIdc, listIdc);

      if ((sizeIdc==SCALING_LIST_32x32) && (listIdc%(SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) != 0)) // derive chroma32x32 from chroma16x16
      {
        const Int *srcNextSmallerSize = getScalingListAddress(sizeIdc-1, listIdc);
        for(UInt i=0; i<size; i++)
        {
          src[i] = srcNextSmallerSize[i];
        }
        setScalingListDC(sizeIdc,listIdc,(sizeIdc > SCALING_LIST_8x8) ? getScalingListDC(sizeIdc-1, listIdc) : src[0]);
      }
      else
      {
        {
          fseek(fp, 0, SEEK_SET);
          Bool bFound=false;
          while ((!feof(fp)) && (!bFound))
          {
            TChar *ret = fgets(line, LINE_SIZE, fp);
            TChar *findNamePosition= ret==NULL ? NULL : strstr(line, gPccShvc_MatrixType[sizeIdc][listIdc]);
            // This could be a match against the DC string as well, so verify it isn't
            if (findNamePosition!= NULL && (gPccShvc_MatrixType_DC[sizeIdc][listIdc]==NULL || strstr(line, gPccShvc_MatrixType_DC[sizeIdc][listIdc])==NULL))
            {
              bFound=true;
            }
          }
          if (!bFound)
          {
            fprintf(stderr, "Error: cannot find Matrix %s from scaling list file %s\n", gPccShvc_MatrixType[sizeIdc][listIdc], fileName.c_str());
            return true;
          }
        }
        for (UInt i=0; i<size; i++)
        {
          Int data;
          if (fscanf(fp, "%d,", &data)!=1)
          {
            fprintf(stderr, "Error: cannot read value #%d for Matrix %s from scaling list file %s at file position %ld\n", i, gPccShvc_MatrixType[sizeIdc][listIdc], fileName.c_str(), ftell(fp));
            return true;
          }
          if (data<0 || data>255)
          {
            fprintf(stderr, "Error: QMatrix entry #%d of value %d for Matrix %s from scaling list file %s at file position %ld is out of range (0 to 255)\n", i, data, gPccShvc_MatrixType[sizeIdc][listIdc], fileName.c_str(), ftell(fp));
            return true;
          }
          src[i] = data;
        }

        //set DC value for default matrix check
        setScalingListDC(sizeIdc,listIdc,src[0]);

        if(sizeIdc > SCALING_LIST_8x8)
        {
          {
            fseek(fp, 0, SEEK_SET);
            Bool bFound=false;
            while ((!feof(fp)) && (!bFound))
            {
              TChar *ret = fgets(line, LINE_SIZE, fp);
              TChar *findNamePosition= ret==NULL ? NULL : strstr(line, gPccShvc_MatrixType_DC[sizeIdc][listIdc]);
              if (findNamePosition!= NULL)
              {
                // This won't be a match against the non-DC string.
                bFound=true;
              }
            }
            if (!bFound)
            {
              fprintf(stderr, "Error: cannot find DC Matrix %s from scaling list file %s\n", gPccShvc_MatrixType_DC[sizeIdc][listIdc], fileName.c_str());
              return true;
            }
          }
          Int data;
          if (fscanf(fp, "%d,", &data)!=1)
          {
            fprintf(stderr, "Error: cannot read DC %s from scaling list file %s at file position %ld\n", gPccShvc_MatrixType_DC[sizeIdc][listIdc], fileName.c_str(), ftell(fp));
            return true;
          }
          if (data<0 || data>255)
          {
            fprintf(stderr, "Error: DC value %d for Matrix %s from scaling list file %s at file position %ld is out of range (0 to 255)\n", data, gPccShvc_MatrixType[sizeIdc][listIdc], fileName.c_str(), ftell(fp));
            return true;
          }
          //overwrite DC value when size of matrix is larger than 16x16
          setScalingListDC(sizeIdc,listIdc,data);
        }
      }
    }
  }
//  std::cout << "\n\nRead scaling lists of:\n\n";
//  outputScalingLists(std::cout);

  fclose(fp);
  return false;
}


// get default address of quantization matrix
// \param sizeId size index
// \param listId list index
// \returns pointer of quantization matrix
const Int* TComScalingList::getScalingListDefaultAddress(UInt sizeId, UInt listId)
{
  const Int *src = 0;
  switch(sizeId)
  {
    case SCALING_LIST_4x4:
      src = gPccShvc_quantTSDefault4x4;
      break;
    case SCALING_LIST_8x8:
    case SCALING_LIST_16x16:
    case SCALING_LIST_32x32:
      src = (listId < (SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) ) ? gPccShvc_quantIntraDefault8x8 : gPccShvc_quantInterDefault8x8;
      break;
    default:
      assert(0);
      src = NULL;
      break;
  }
  return src;
}

// process of default matrix
// \param sizeId size index
// \param listId index of input matrix
Void TComScalingList::processDefaultMatrix(UInt sizeId, UInt listId)
{
  ::memcpy(getScalingListAddress(sizeId, listId),getScalingListDefaultAddress(sizeId,listId),sizeof(Int)*min(MAX_MATRIX_COEF_NUM,(Int)gPccShvc_scalingListSize[sizeId]));
  setScalingListDC(sizeId,listId,SCALING_LIST_DC);
}

// check DC value of matrix for default matrix signaling
Void TComScalingList::checkDcOfMatrix()
{
  for(UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(UInt listId = 0; listId < SCALING_LIST_NUM; listId++)
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
: m_activePPSId(-1)
#else
: m_vpsMap(MAX_NUM_VPS)
, m_spsMap(MAX_NUM_SPS)
, m_ppsMap(MAX_NUM_PPS)
, m_activeVPSId(-1)
#endif
, m_activeSPSId(-1)
{
}


ParameterSetManager::~ParameterSetManager()
{
}

//! activate a SPS from a active parameter sets SEI message
//! \returns true, if activation is successful
//Bool ParameterSetManager::activateSPSWithSEI(Int spsId)
//{
//  TComSPS *sps = m_spsMap.getPS(spsId);
//  if (sps)
//  {
//    Int vpsId = sps->getVPSId();
//    TComVPS *vps = m_vpsMap.getPS(vpsId);
//    if (vps)
//    {
//      m_activeVPS = *(vps);
//      m_activeSPS = *(sps);
//      return true;
//    }
//    else
//    {
//      printf("Warning: tried to activate SPS using an Active parameter sets SEI message. Referenced VPS does not exist.");
//    }
//  }
//  else
//  {
//    printf("Warning: tried to activate non-existing SPS using an Active parameter sets SEI message.");
//  }
//  return false;
//}

//! activate a PPS and depending on isIDR parameter also SPS and VPS
//! \returns true, if activation is successful
Bool ParameterSetManager::activatePPS(Int ppsId, Bool isIRAP)
{
  TComPPS *pps = m_ppsMap.getPS(ppsId);
  if (pps)
  {
#if SVC_EXTENSION
    m_activePPSId = ppsId;
#endif
    Int spsId = pps->getSPSId();
    if (!isIRAP && (spsId != m_activeSPSId ))
    {
      printf("Warning: tried to activate PPS referring to a inactive SPS at non-IDR.");
    }
    else
    {
      TComSPS *sps = m_spsMap.getPS(spsId);
      if (sps)
      {
        Int vpsId = sps->getVPSId();
        if (!isIRAP && (vpsId != m_activeVPSId ))
        {
          printf("Warning: tried to activate PPS referring to a inactive VPS at non-IDR.");
        }
        else
        {
          TComVPS *vps =m_vpsMap.getPS(vpsId);
          if (vps)
          {
            m_activeVPSId = vpsId;
            m_activeSPSId = spsId;
            return true;
          }
          else
          {
            printf("Warning: tried to activate PPS that refers to a non-existing VPS.");
          }
        }
      }
      else
      {
        printf("Warning: tried to activate a PPS that refers to a non-existing SPS.");
      }
    }
  }
  else
  {
    printf("Warning: tried to activate non-existing PPS.");
  }

  // Failed to activate if reach here.
  m_activeSPSId=-1;
  m_activeVPSId=-1;
  return false;
}

ProfileTierLevel::ProfileTierLevel()
  : m_profileSpace    (0)
  , m_tierFlag        (Level::MAIN)
  , m_profileIdc      (Profile::NONE)
  , m_levelIdc        (Level::NONE)
  , m_progressiveSourceFlag  (false)
  , m_interlacedSourceFlag   (false)
  , m_nonPackedConstraintFlag(false)
  , m_frameOnlyConstraintFlag(false)
{
  ::memset(m_profileCompatibilityFlag, 0, sizeof(m_profileCompatibilityFlag));
}

TComPTL::TComPTL()
{
  ::memset(m_subLayerProfilePresentFlag, 0, sizeof(m_subLayerProfilePresentFlag));
  ::memset(m_subLayerLevelPresentFlag,   0, sizeof(m_subLayerLevelPresentFlag  ));
}

Void calculateParameterSetChangedFlag(Bool &bChanged, const std::vector<UChar> *pOldData, const std::vector<UChar> &newData)
{
  if (!bChanged)
  {
    if ((pOldData==0 && pOldData!=0) || (pOldData!=0 && pOldData==0))
    {
      bChanged=true;
    }
    else if (pOldData!=0 && pOldData!=0)
    {
      // compare the two
      if (pOldData->size() != pOldData->size())
      {
        bChanged=true;
      }
      else
      {
        const UChar *pNewDataArray=&(newData)[0];
        const UChar *pOldDataArray=&(*pOldData)[0];
        if (memcmp(pOldDataArray, pNewDataArray, pOldData->size()))
        {
          bChanged=true;
        }
      }
    }
  }
}

#if SVC_EXTENSION
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

const Window& TComPPS::getScaledRefLayerWindowForLayer(Int layerId) const
{
  static const Window win;

  for( Int i = m_numRefLayerLocationOffsets-1; i >= 0; i-- )
  {
    if( layerId == m_refLocationOffsetLayerId[i] )
    {
      return m_scaledRefLayerWindow[i];
    }
  }
  
  return win;
}

const Window& TComPPS::getRefLayerWindowForLayer(Int layerId) const
{
  static const Window win;

  for( Int i = m_numRefLayerLocationOffsets-1; i >= 0; i-- )
  {
    if( layerId == m_refLocationOffsetLayerId[i] )
    {
      return m_refLayerWindow[i];
    }
  }

  return win;
}

Bool TComPPS::hasZeroResamplingPhase(Int refLayerId) const
{
  const ResamplingPhase &resamplingPhase = getResamplingPhase(refLayerId);

  return ( resamplingPhase.phaseHorLuma == 0 && resamplingPhase.phaseHorChroma == 0 && resamplingPhase.phaseVerLuma == 0 && resamplingPhase.phaseVerChroma == 0 );
}

const ResamplingPhase& TComPPS::getResamplingPhase(Int refLayerId) const
{
  static const ResamplingPhase resamplingPhase;

  for( Int i = m_numRefLayerLocationOffsets-1; i >= 0; i-- )
  {
    if( refLayerId == m_refLocationOffsetLayerId[i] )
    {
      return m_resamplingPhase[i];
    }
  }

  return resamplingPhase;
}

Void TComVPS::deriveLayerIdListVariables()
{
  // For layer 0
  m_numLayerInIdList.push_back(1);
  m_layerSetLayerIdList.resize(m_vpsNumLayerSetsMinus1 + 1);
  m_layerSetLayerIdList[0].push_back(0);
  
  // For other layers
  for (Int i = 1; i <= m_vpsNumLayerSetsMinus1; i++)
  {
    for( Int m = 0; m <= m_maxLayerId; m++)
    {
      if( m_layerIdIncludedFlag[i][m] )
      {
        m_layerSetLayerIdList[i].push_back(m);
      }
    }
    m_numLayerInIdList.push_back((Int)m_layerSetLayerIdList[i].size());
  }
}

Void TComVPS::deriveNumberOfSubDpbs()
{
  // Derive number of sub-DPBs
  // For layer set 0
  m_numSubDpbs[0] = 1;
  // For other layer sets
  for( Int i = 1; i < m_numLayerSets; i++)
  {
    m_numSubDpbs[i] = m_numLayerInIdList[i];
  }
}

Void TComVPS::setTilesNotInUseFlag(Bool x)
{
  m_tilesNotInUseFlag = x;

  if( m_tilesNotInUseFlag )
  {
    for( Int i = 0; i < getMaxLayers(); i++ )
    {
      m_tilesInUseFlag[i] = m_loopFilterNotAcrossTilesFlag[i] = m_tilesNotInUseFlag;
    }
  
    for( Int i = 1; i < getMaxLayers(); i++ )
    {
      for( Int j = 0; j < getNumDirectRefLayers(getLayerIdInNuh(i)); j++)
      {
        setTileBoundariesAlignedFlag(i, j, m_tilesNotInUseFlag);
      }
    }
  }
}

Void TComVPS::setWppNotInUseFlag(Bool x)
{
  m_wppNotInUseFlag = x;

  if( m_wppNotInUseFlag )
  {
    for( Int i = 0; i < getMaxLayers(); i++ )
    {
      m_wppInUseFlag[i] = m_wppNotInUseFlag;
    }
  }
}

Void TComVPS::setRefLayersFlags(Int currLayerId)
{
  for (Int i = 0; i < m_numDirectRefLayers[currLayerId]; i++)
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

  for (Int i = 0; i < m_uiMaxLayers; i++)
  {
    UInt iNuhLId = m_layerIdInNuh[i];
    setRefLayersFlags(iNuhLId);
    for (UInt j = 0; j < MAX_NUM_LAYER_IDS; j++)
    {
      m_numberRefLayers[iNuhLId] += (m_recursiveRefLayerFlag[iNuhLId][j] == true ? 1 : 0);
    }
  }
}

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
        if( !countedLayerIdxFlag[m_layerIdxInVps[iNuhLId]] )
        {
          m_treePartitionLayerIdList[numIndependentLayers][m_numLayersInTreePartition[numIndependentLayers]] = m_predictedLayerId[iNuhLId][j];
          m_numLayersInTreePartition[numIndependentLayers] = m_numLayersInTreePartition[numIndependentLayers] + 1;
          countedLayerIdxFlag[m_layerIdxInVps[m_predictedLayerId[iNuhLId][j]]] = true;
        }
      }
      numIndependentLayers++;
    }
  }

  m_numIndependentLayers = numIndependentLayers;
}

Void TComVPS::deriveLayerIdListVariablesForAddLayerSets()
{
  m_layerSetLayerIdList.resize(m_vpsNumLayerSetsMinus1 + 1 + m_numAddLayerSets);

  for (UInt i = 0; i < m_numAddLayerSets; i++)
  {
    Int layerNum = 0;
    Int lsIdx = m_vpsNumLayerSetsMinus1 + 1 + i;
    for (Int treeIdx = 1; treeIdx < m_numIndependentLayers; treeIdx++)
    {
      for (Int layerCnt = 0; layerCnt < m_highestLayerIdxPlus1[i][treeIdx]; layerCnt++)
      {
        m_layerSetLayerIdList[lsIdx].push_back(m_treePartitionLayerIdList[treeIdx][layerCnt]);
        layerNum++;
      }
    }
    m_numLayerInIdList.push_back(layerNum);
  }
}

Int TComVPS::getNumViews() const
{
  Int numViews = 1; 
  for( Int i = 0; i < m_uiMaxLayers; i++ )
  {
    Int lId = m_layerIdInNuh[i]; 
    if( i > 0 )
    {
      UChar newViewFlag = 1;

      for( Int j = 0; j < i; j++ )
      {
        if( getViewIndex(lId) == getScalabilityId( m_layerIdInNuh[j], VIEW_ORDER_INDEX ) )
        {
          newViewFlag = 0;
          break;
        }
      }

      numViews += newViewFlag; 
    }
  }

  return numViews;
}

Int TComVPS::getScalabilityId( Int layerIdInVps, ScalabilityType scalType ) const
{
  return getScalabilityMask( scalType ) ? getDimensionId( layerIdInVps, scalTypeToScalIdx( scalType ) ) : 0;
}

Int TComVPS::scalTypeToScalIdx( ScalabilityType scalType ) const
{
  assert( (Int)scalType >= 0 && (Int)scalType <= MAX_VPS_NUM_SCALABILITY_TYPES ); 
  assert( (Int)scalType == MAX_VPS_NUM_SCALABILITY_TYPES || getScalabilityMask( scalType ) );
  Int scalIdx = 0; 
  for( Int curScalType = 0; curScalType < scalType; curScalType++ )
  {
    scalIdx += ( getScalabilityMask( curScalType ) ? 1 : 0 );

  }

  return scalIdx; 
}

Int TComVPS::getLayerIdcForOls( Int olsIdx, Int layerId ) const
{
  Int layerIdc = -1;
  UInt lsIdx = m_outputLayerSetIdx[olsIdx];

  std::vector<Int>::const_iterator it = std::find( m_layerSetLayerIdList[lsIdx].begin(), m_layerSetLayerIdList[lsIdx].end(), layerId );

  if( it != m_layerSetLayerIdList[lsIdx].end() )
  {
    layerIdc = (Int)std::distance( m_layerSetLayerIdList[lsIdx].begin(), it );
  }

  assert( layerIdc >= 0 );

  return layerIdc;
}

Void TComVPS::determineSubDpbInfoFlags()
{
  for(Int i = 1; i < getNumOutputLayerSets(); i++)
  {
    Int layerSetIdxForOutputLayerSet = getOutputLayerSetIdx( i );
    // For each output layer set, set the DPB size for each layer and the reorder/latency value the maximum for all layers
    Bool checkFlagOuter = false;      // Used to calculate sub_layer_flag_info_present_flag
    Bool checkFlagInner[MAX_TLAYER];  // Used to calculate sub_layer_dpb_info_present_flag

    for(Int j = 0; j < getMaxTLayers(); j++)
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

        for(Int subDpbIdx = 0; subDpbIdx < getNumSubDpbs(layerSetIdxForOutputLayerSet) && !checkFlagInner[j]; subDpbIdx++)  // If checkFlagInner[j] is true, break and signal the values
        {
          checkFlagInner[j] |= ( getMaxVpsDecPicBufferingMinus1(i, subDpbIdx, j - 1) != getMaxVpsDecPicBufferingMinus1(i, subDpbIdx, j) );
        }
      }
      // If checkFlagInner[j] = true, then some value needs to be signalled for the j-th sub-layer
      setSubLayerDpbInfoPresentFlag( i, j, checkFlagInner[j] );
    }

    // --------------------------------------------------------
    // To determine value of m_subLayerFlagInfoPresentFlag
    // --------------------------------------------------------

    for(Int j = 1; j < getMaxTLayers(); j++) // Check if DPB info of any of non-zero sub-layers is signaled. If so set flag to one
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
#endif

Void TComVPS::deriveNecessaryLayerFlag()
{
  m_necessaryLayerFlag.empty();
  m_numNecessaryLayers.empty();
  // Assumed that output layer sets and variables RecursiveRefLayer are already derived
  for( Int olsIdx = 0; olsIdx < m_numOutputLayerSets; olsIdx++)
  {
    deriveNecessaryLayerFlag(olsIdx);
  }
}

Void TComVPS::deriveNecessaryLayerFlag(const Int olsIdx)
{
  Int lsIdx = m_outputLayerSetIdx[olsIdx];
  Int numLayersInLs = m_numLayerInIdList[lsIdx];
  assert( m_necessaryLayerFlag.size() == olsIdx );   // Function should be called in the correct order.
  m_necessaryLayerFlag.push_back( std::vector<Bool>( numLayersInLs, false ) ); // Initialize to false
  for( Int lsLayerIdx = 0; lsLayerIdx < numLayersInLs; lsLayerIdx++ )
  {
    if( m_outputLayerFlag[olsIdx][lsLayerIdx] )
    {
      m_necessaryLayerFlag[olsIdx][lsLayerIdx] = true;
      Int currNuhLayerId = m_layerSetLayerIdList[lsIdx][lsLayerIdx];
      for( Int rLsLayerIdx = 0; rLsLayerIdx < lsLayerIdx; rLsLayerIdx++ )
      {
        Int refNuhLayerId = m_layerSetLayerIdList[lsIdx][rLsLayerIdx];
        if( m_recursiveRefLayerFlag[currNuhLayerId][refNuhLayerId] )
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
  for(Int layerIdx = m_baseLayerInternalFlag ? 0 : 1; layerIdx < m_uiMaxLayers; layerIdx++)
  {
    Bool layerFoundNecessaryLayerFlag = false;
    for(Int olsIdx = 0; olsIdx < m_numOutputLayerSets; olsIdx++)
    {
      Int lsIdx = m_outputLayerSetIdx[olsIdx];
      Int currNuhLayerId = m_layerIdInNuh[layerIdx];
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

Int TComVPS::calculateLenOfSyntaxElement( const Int numVal ) const
{
  Int numBits = 1;
  while((1 << numBits) < numVal)
  {
    numBits++;
  }
  return numBits;
}

Void TComVPS::calculateMaxSLInLayerSets()
{
  for(Int lsIdx = 0; lsIdx < getNumLayerSets(); lsIdx++)
  {
    UChar maxSLMinus1 = 0;
    for(Int k = 0; k < getNumLayersInIdList(lsIdx); k++ ) {
      Int  lId = getLayerSetLayerIdList(lsIdx, k);
      maxSLMinus1 = std::max(maxSLMinus1, getMaxTSLayersMinus1(getLayerIdxInVps(lId)));
    }
    setMaxSLayersInLayerSetMinus1(lsIdx,maxSLMinus1);
  }
}

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

Void TComPTL::copyProfileInfo(TComPTL *ptl)
{
  // Copy all information related to general profile
  this->getGeneralPTL()->copyProfileInfo(ptl->getGeneralPTL());
}
/*

Bool TComSlice::setBaseColPic(  TComList<TComPic*>& rcListPic, UInt refLayerIdc )
{  
  if(m_layerId == 0)
  {
    memset( m_pcBaseColPic, 0, sizeof( m_pcBaseColPic ) );
    return false;
  }        

  TComPic* pic = xGetRefPic( rcListPic, getPOC() );

  if( pic )
  {
    setBaseColPic(refLayerIdc, pic );
  }
  else
  {
    return false;
  }
  
  return true;
}

TComPic* TComSlice::getBaseColPic(  TComList<TComPic*>& rcListPic )
{
  return xGetRefPic( rcListPic, m_iPOC );
}

Void TComSlice::setILRPic(TComPic **pcIlpPic)
{
  for( Int i = 0; i < m_activeNumILRRefIdx; i++ )
  {
    Int refLayerIdc = m_interLayerPredLayerIdc[i];

    if( pcIlpPic[refLayerIdc] )
    {
      TComPic* pcRefPicBL = m_pcBaseColPic[refLayerIdc];

      // copy scalability ratio, it is needed to get the correct location for the motion field of the corresponding reference layer block
      pcIlpPic[refLayerIdc]->setRequireResamplingFlag( refLayerIdc, m_pcPic->requireResampling(refLayerIdc) );

      pcIlpPic[refLayerIdc]->copyUpsampledPictureYuv( m_pcPic->getFullPelBaseRec( refLayerIdc ), pcIlpPic[refLayerIdc]->getPicYuvRec() );      
      pcIlpPic[refLayerIdc]->getSlice(0)->setBaseColPic( refLayerIdc, pcRefPicBL );

      //set reference picture POC of each ILP reference 
      pcIlpPic[refLayerIdc]->getSlice(0)->setPOC( m_iPOC );

      //set temporal Id 
      pcIlpPic[refLayerIdc]->getSlice(0)->setTLayer( m_uiTLayer );

      //copy layer id from the reference layer 
      pcIlpPic[refLayerIdc]->setLayerId( pcRefPicBL->getLayerId() );
      pcIlpPic[refLayerIdc]->getSlice(0)->setLayerId(pcRefPicBL->getLayerId());

      pcIlpPic[refLayerIdc]->getPicYuvRec()->setBorderExtension( false );
      pcIlpPic[refLayerIdc]->getPicYuvRec()->extendPicBorder();
      pcIlpPic[refLayerIdc]->setIsLongTerm(true);

      // assign PPS to ILRP to be used for reference location offsets
      pcIlpPic[refLayerIdc]->getSlice(0)->setPPS( m_pcPic->getSlice(0)->getPPS() );

      // assign SPS to ILRP to be used for obtaining bit depth
      pcIlpPic[refLayerIdc]->getSlice(0)->setSPS( m_pcPic->getSlice(0)->getSPS() );

      // assing VPS to ILRP to be used for deriving layerIdx
      pcIlpPic[refLayerIdc]->getSlice(0)->setVPS( m_pcPic->getSlice(0)->getVPS() );

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

        pcIlpPic[refLayerIdc]->copyUpsampledMvField( refLayerIdc, m_pcPic->getMvScalingFactor(), m_pcPic->getPosScalingFactor() );
      }
      else
      {
        pcIlpPic[refLayerIdc]->initUpsampledMvField();
      }

      Int maxTidIlRefPicsPlus1 = m_pcVPS->getMaxTidIlRefPicsPlus1( pcIlpPic[refLayerIdc]->getSlice(0)->getLayerIdx(), getLayerIdx() );
      assert( (Int)pcIlpPic[refLayerIdc]->getSlice(0)->getTLayer() < maxTidIlRefPicsPlus1 || ( !maxTidIlRefPicsPlus1 && pcIlpPic[refLayerIdc]->getSlice(0)->getRapPicFlag() ) );

    }
  }
}

*/

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

Bool TComSlice::getBlaPicFlag()
{
    return  getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP
    || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
    || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP;
}

Bool TComSlice::getCraPicFlag()
{
    return getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA;
}

Bool TComSlice::getRaslPicFlag()
{
  return  getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_R
  || getNalUnitType() == NAL_UNIT_CODED_SLICE_RASL_N;
}

Bool TComSlice::getRadlPicFlag()
{
  return  getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_R
  || getNalUnitType() == NAL_UNIT_CODED_SLICE_RADL_N;
}

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

#if VIEW_SCALABILITY
Void TComSlice::setRefPicListModificationSvc( TComPic** ilpPic )
#else
Void TComSlice::setRefPicListModificationSvc()
#endif
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
  Int numberOfPocBeforeCurr = getNumNegativeRpsCurrTempList();  // number of negative temporal ref pics 

  assert(m_aiNumRefIdx[REF_PIC_LIST_0] > 0);
  assert(m_aiNumRefIdx[REF_PIC_LIST_1] > 0);

  //set L0 inter-layer reference picture modification
#if VIEW_SCALABILITY
  Int numberOfPocAfterCurr = 0;
  Int numberOfInterLayer0 = 0;
  Int numberOfInterLayer1 = 0;
#endif

  Bool hasModification = false;

  if( m_pcVPS->getScalabilityMask( SCALABILITY_ID ) ) //scalable layer
  {
    hasModification = (m_aiNumRefIdx[REF_PIC_LIST_0] == (numberOfPocBeforeCurr + m_activeNumILRRefIdx)) ? false : true;

    if( m_activeNumILRRefIdx > 1 )
    {
      hasModification = (m_aiNumRefIdx[REF_PIC_LIST_0] >= (numberOfPocBeforeCurr + m_activeNumILRRefIdx)) ? false : true;
    }
  }
#if VIEW_SCALABILITY
  else if( m_pcVPS->getScalabilityMask( VIEW_ORDER_INDEX ) ) //multi-view
  {
    numberOfPocAfterCurr = getNumfPositiveRpsCurrTempList();
    numberOfInterLayer0 = getNumRpsInterLayerX(0, ilpPic);
    numberOfInterLayer1 = getNumRpsInterLayerX(1, ilpPic);

    if( (numberOfInterLayer1 > 0 && m_aiNumRefIdx[REF_PIC_LIST_0] < numberOfRpsCurrTempList) || (numberOfInterLayer1 == 0 && m_aiNumRefIdx[REF_PIC_LIST_0] < numberOfPocBeforeCurr + numberOfInterLayer0 ) )
    {
      hasModification = true;
    }
    else
    {
      hasModification = false;
    }
  }  
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
    else if( m_pcVPS->getScalabilityMask( SCALABILITY_ID ) ) //scalable layer
    {
      // number of ILRPs included into the reference picture list with the list modification
      Int includeNumILRP = min( max(1, m_aiNumRefIdx[REF_PIC_LIST_0]-numberOfPocBeforeCurr), m_activeNumILRRefIdx);

      for(Int i = includeNumILRP; i > 0; i-- )
      {
        if( numberOfPocBeforeCurr >= m_aiNumRefIdx[REF_PIC_LIST_0] )
        {
          refPicListModification->setRefPicSetIdxL0(m_aiNumRefIdx[REF_PIC_LIST_0] - i, numberOfPocBeforeCurr + includeNumILRP - i);
        }
        else
        {
          refPicListModification->setRefPicSetIdxL0(m_aiNumRefIdx[REF_PIC_LIST_0] - i, numberOfPocBeforeCurr + includeNumILRP - i);
          for( Int j = numberOfPocBeforeCurr; j < (m_aiNumRefIdx[REF_PIC_LIST_0] - i); j++ )
          {
            assert( j + includeNumILRP < numberOfRpsCurrTempList );
            refPicListModification->setRefPicSetIdxL0(j, j + includeNumILRP);
          }
        }
      }
    }
#if VIEW_SCALABILITY
    else if( m_pcVPS->getScalabilityMask( VIEW_ORDER_INDEX ) ) //multi-view
    {
      Int includeNumILRP = m_activeNumILRRefIdx;
      Int iNumILRP = includeNumILRP;

      if( numberOfInterLayer1 > 0 )
      {
        if( m_aiNumRefIdx[REF_PIC_LIST_0] < numberOfPocBeforeCurr + m_activeNumILRRefIdx )
        {
          for( Int i = 0; i < numberOfInterLayer0 && iNumILRP > 0; i++, iNumILRP-- )
          {
            refPicListModification->setRefPicSetIdxL0( m_aiNumRefIdx[REF_PIC_LIST_0] - includeNumILRP + i, numberOfPocBeforeCurr + i );
          }

          for( Int i = 0; i < numberOfInterLayer1 && iNumILRP > 0; i++, iNumILRP-- )
          {
            refPicListModification->setRefPicSetIdxL0( m_aiNumRefIdx[REF_PIC_LIST_0] - includeNumILRP +numberOfInterLayer0+ i, numberOfRpsCurrTempList-  numberOfInterLayer1 + i );
          }
        }
        else
        {
          iNumILRP -= numberOfInterLayer0;

          for( Int i = numberOfInterLayer1; i >0 && iNumILRP > 0; i--, iNumILRP-- )
          {
            refPicListModification->setRefPicSetIdxL0( m_aiNumRefIdx[REF_PIC_LIST_0] - i, numberOfRpsCurrTempList - i );
          }
        }
      }
      else
      {
        for( Int i = numberOfInterLayer0; i >0 && iNumILRP >0 ; i--, iNumILRP-- )
        {
          refPicListModification->setRefPicSetIdxL0( m_aiNumRefIdx[REF_PIC_LIST_0] - i, numberOfPocBeforeCurr + numberOfInterLayer0 - i );
        }
      }
    }
#endif
  }

  //set L1 inter-layer reference picture modification
  if( m_pcVPS->getScalabilityMask( SCALABILITY_ID ) ) //scalable layer
  {
    hasModification = (m_aiNumRefIdx[REF_PIC_LIST_1] >= numberOfRpsCurrTempList) ? false : true;
  }
#if VIEW_SCALABILITY
  else if( m_pcVPS->getScalabilityMask( VIEW_ORDER_INDEX ) ) //multi-view
  {
    if( (numberOfInterLayer0 > 0 && m_aiNumRefIdx[REF_PIC_LIST_1] < numberOfRpsCurrTempList) || (numberOfInterLayer0 == 0 && m_aiNumRefIdx[REF_PIC_LIST_1] < numberOfPocAfterCurr + numberOfInterLayer1) )
    {
      hasModification = true;
    }
    else
    {
      hasModification = false;
    }
  }
#endif
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
    else if( m_pcVPS->getScalabilityMask( SCALABILITY_ID ) ) //scalable layer
    {
      Int includeNumILRP = min(m_aiNumRefIdx[REF_PIC_LIST_1], m_activeNumILRRefIdx);

      for(Int i = includeNumILRP; i > 0; i-- )
      {
        refPicListModification->setRefPicSetIdxL1(m_aiNumRefIdx[REF_PIC_LIST_1] - i, numberOfRpsCurrTempList - i);
      }
    }
#if VIEW_SCALABILITY
    else if( m_pcVPS->getScalabilityMask( VIEW_ORDER_INDEX ) ) //multi-view
    {
      Int includeNumILRP = m_activeNumILRRefIdx;
      Int iNumILRP=includeNumILRP;

      if( numberOfInterLayer0 > 0 )
      {
        if( m_aiNumRefIdx[REF_PIC_LIST_1] < numberOfPocAfterCurr + m_activeNumILRRefIdx )
        {
          for( Int i = 0; i < numberOfInterLayer1 && iNumILRP>0; i++,iNumILRP-- )
          {
            refPicListModification->setRefPicSetIdxL1(m_aiNumRefIdx[REF_PIC_LIST_1] - includeNumILRP + i, numberOfPocAfterCurr + i);
          }

          for(Int i = 0; i < numberOfInterLayer0 && iNumILRP>0; i++,iNumILRP-- )
          {
            refPicListModification->setRefPicSetIdxL1(m_aiNumRefIdx[REF_PIC_LIST_1] - includeNumILRP +numberOfInterLayer1+ i, numberOfRpsCurrTempList-numberOfInterLayer0+i);
          }
        }
        else
        {
          iNumILRP -= numberOfInterLayer1;
          for( Int i = numberOfInterLayer0; i > 0 && iNumILRP > 0; i--,iNumILRP-- )
          {
            refPicListModification->setRefPicSetIdxL1(m_aiNumRefIdx[REF_PIC_LIST_1] - i, numberOfRpsCurrTempList - i);
          }
        }
      }
      else
      {
        for( Int i = numberOfInterLayer1; i >0 && iNumILRP>0; i--, iNumILRP-- )
        {
          refPicListModification->setRefPicSetIdxL1(m_aiNumRefIdx[REF_PIC_LIST_1] - i, numberOfPocAfterCurr + numberOfInterLayer1 - i);
        }
      }
    }
#endif
  }
  return;
}

Int TComSlice::getNumNegativeRpsCurrTempList()
{
  if( m_eSliceType == I_SLICE ) 
  {
    return 0;
  }

  Int numPocBeforeCurr = 0;

  for( UInt i = 0; i < m_pRPS->getNumberOfNegativePictures(); i++ )
  {
    if( m_pRPS->getUsed(i) )
    {
      numPocBeforeCurr++;
    }
  }

  return numPocBeforeCurr;
}

Void TComSPS::inferSPS( const UInt layerId, TComVPS* vps )
{
  printf("inferSPS: layerId = %d m_bV1CompatibleSPSFlag = %d \n",layerId,m_bV1CompatibleSPSFlag);
  RepFormat* repFormat = NULL;

  if( layerId == 0 || m_bV1CompatibleSPSFlag == 1 )
  {
    if( layerId == 0 && vps->getNonHEVCBaseLayerFlag() )
    {
      // infered from vps_rep_format_idx[ 0 ]-th rep_format( ) syntax structure in the active VPS
      repFormat = vps->getVpsRepFormat(0);
    }
  }
  else
  {
  printf("inferSPS: m_updateRepFormatFlag  = %d \n",m_updateRepFormatFlag);
  printf("inferSPS: m_updateRepFormatIndex = %d \n",m_updateRepFormatIndex);
    repFormat = vps->getVpsRepFormat(m_updateRepFormatFlag ? m_updateRepFormatIndex : vps->getVpsRepFormatIdx(vps->getLayerIdxInVps(layerId)));
  }

  printf("inferSPS: repFormat = %p \n",repFormat);
  if( repFormat )
  {
    m_chromaFormatIdc = repFormat->getChromaFormatVpsIdc();
    m_picWidthInLumaSamples = repFormat->getPicWidthVpsInLumaSamples();
    m_picHeightInLumaSamples = repFormat->getPicHeightVpsInLumaSamples();
    printf("inferSPS => %d %d \n", m_picWidthInLumaSamples , m_picHeightInLumaSamples);

    m_bitDepths.recon[CHANNEL_TYPE_LUMA] = repFormat->getBitDepthVps(CHANNEL_TYPE_LUMA);
    m_bitDepths.recon[CHANNEL_TYPE_CHROMA] = repFormat->getBitDepthVps(CHANNEL_TYPE_CHROMA);

    m_qpBDOffset[CHANNEL_TYPE_LUMA] = (m_bitDepths.recon[CHANNEL_TYPE_LUMA] - 8) * 6;
    m_qpBDOffset[CHANNEL_TYPE_CHROMA] = (m_bitDepths.recon[CHANNEL_TYPE_CHROMA] - 8) * 6;

    m_conformanceWindow = repFormat->getConformanceWindowVps();
  }
}

#if VIEW_SCALABILITY 
Int TComSlice::getNumfPositiveRpsCurrTempList()
{
  if( m_eSliceType == I_SLICE ) 
  {
    return 0;
  }

  Int numPocAfterCurr = 0;

  for( UInt i = 0; i < m_pRPS->getNumberOfPositivePictures(); i++ )
  {
    if( m_pRPS->getUsed(i) )
    {
      numPocAfterCurr++;
    }
  }

  return numPocAfterCurr;
}
#endif

#endif //SVC_EXTENSION


//! \}
