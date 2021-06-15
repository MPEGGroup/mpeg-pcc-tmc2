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

/** \file     TComSlice.cpp
    \brief    slice header and SPS class
*/

#include "PccAvcCommonDef.h"
#include "PccAvcTComSlice_avc.h"
// #include "TComPic.h"
// #include "TLibEncoder/TEncSbac.h"
// #include "TLibDecoder/TDecSbac.h"

//! \ingroup TLibCommon
//! \{

TComSlice_avc::TComSlice_avc() :
    m_iPPSId( -1 ),
    m_PicOutputFlag( true ),
    m_iPOC( 0 ),
    m_iLastIDR( 0 ),
    m_iAssociatedIRAP( 0 ),
    m_iAssociatedIRAPType( NAL_UNIT_INVALID ),
    //m_pRPS( 0 ),
    //m_localRPS(),
    m_rpsIdx( 0 ),
    //m_RefPicListModification(),
    m_eNalUnitType( NALU_TYPE_IDR ),
    m_eSliceType( I_SLICE ),
    m_iSliceQp( 0 ),
    m_dependentSliceSegmentFlag( false )
#if ADAPTIVE_QP_SELECTION
    ,
    m_iSliceQpBase( 0 )
#endif
    ,
    //m_ChromaQpAdjEnabled( false ),
    m_deblockingFilterDisable( false ),
    m_deblockingFilterOverrideFlag( false ),
    m_deblockingFilterBetaOffsetDiv2( 0 ),
    m_deblockingFilterTcOffsetDiv2( 0 ),
    m_bCheckLDC( false ),
    m_iSliceQpDelta( 0 ),
    m_iDepth( 0 ),
    m_bRefenced( false ),
    //m_pcVPS( NULL ),
    m_pcSPS( NULL ),
    //m_pcPPS( NULL ),
    m_pcPic( NULL ),
    m_colFromL0Flag( true ),
    m_noOutputPriorPicsFlag( false ),
    m_noRaslOutputFlag( false ),
    m_handleCraAsBlaFlag( false ),
    m_colRefIdx( 0 ),
    m_maxNumMergeCand( 0 ),
    m_uiTLayer( 0 ),
    m_bTLayerSwitchingFlag( false ),
    m_sliceMode( NO_SLICES ),
    m_sliceArgument( 0 ),
    m_sliceCurStartCtuTsAddr( 0 ),
    m_sliceCurEndCtuTsAddr( 0 ),
    m_sliceIdx( 0 ),
    m_sliceSegmentMode( NO_SLICES ),
    m_sliceSegmentArgument( 0 ),
    m_sliceSegmentCurStartCtuTsAddr( 0 ),
    m_sliceSegmentCurEndCtuTsAddr( 0 ),
    m_nextSlice( false ),
    m_nextSliceSegment( false ),
    m_sliceBits( 0 ),
    m_sliceSegmentBits( 0 ),
    m_bFinalized( false ),
    m_bTestWeightPred( false ),
    m_bTestWeightBiPred( false ),
    m_substreamSizes(),
    m_cabacInitFlag( false ),
    m_bLMvdL1Zero( false ),
    m_temporalLayerNonReferenceFlag( false ),
    m_LFCrossSliceBoundaryFlag( false ),
    m_enableTMVPFlag( true ),
    m_encCABACTableIdx( I_SLICE ),
    m_useIntegerMv( false ),
    m_pcLastEncPic( NULL ),
    m_pcCurPicLongTerm( NULL ),
    m_pcTwoVersionsOfCurrDecPicFlag( false ) {
  for ( UInt i = 0; i < NUM_REF_PIC_LIST_01; i++ ) { m_aiNumRefIdx[i] = 0; }

  for ( UInt component = 0; component < MAX_NUM_COMPONENT; component++ ) {
    m_lambdas[component]             = 0.0;
    m_iSliceChromaQpDelta[component] = 0;
    m_iSliceACTQpDelta[component]    = 0;
  }

  initEqualRef();

  for ( Int idx = 0; idx < MAX_NUM_REF; idx++ ) { m_list1IdxToList0Idx[idx] = -1; }

  for ( Int iNumCount = 0; iNumCount < MAX_NUM_REF; iNumCount++ ) {
    for ( UInt i = 0; i < NUM_REF_PIC_LIST_01; i++ ) {
      m_apcRefPicList[i][iNumCount] = NULL;
      m_aiRefPOCList[i][iNumCount]  = 0;
    }
  }

  //resetWpScaling();
  //initWpAcDcParam();

  for ( Int ch = 0; ch < MAX_NUM_CHANNEL_TYPE; ch++ ) { m_saoEnabledFlag[ch] = false; }
}

TComSlice_avc::~TComSlice_avc() {}

Void TComSlice_avc::initSlice() {
  for ( UInt i = 0; i < NUM_REF_PIC_LIST_01; i++ ) { m_aiNumRefIdx[i] = 0; }
  m_colFromL0Flag = true;

  m_colRefIdx = 0;
  initEqualRef();

  m_bCheckLDC = false;

  for ( UInt component = 0; component < MAX_NUM_COMPONENT; component++ ) {
    m_iSliceChromaQpDelta[component] = 0;
    m_iSliceACTQpDelta[component]    = 0;
  }

  m_maxNumMergeCand = MRG_MAX_NUM_CANDS;

  m_bFinalized = false;

  m_substreamSizes.clear();
  m_cabacInitFlag  = false;
  m_enableTMVPFlag = true;
}

/* TCH
Bool TComSlice_avc::getRapPicFlag() const {
  return getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
         getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL ||
         getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP || getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA;
}
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
    if(pcPic->getPOC() == poc)
    {
      break;
    }
    iterPic++;
    pcPic = *(iterPic);
  }
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

Void TComSlice::setRefPicList( TComList<TComPic*>& rcListPic, Bool checkNumPocTotalCurr )
{
  if ( m_eSliceType == I_SLICE)
  {
    ::memset( m_apcRefPicList, 0, sizeof (m_apcRefPicList));
    ::memset( m_aiNumRefIdx,   0, sizeof ( m_aiNumRefIdx ));

    if (!checkNumPocTotalCurr)
    {
      if( m_pRPS->getNumberOfPictures() == 0 )
      {
        TComPic *pPrevPic = xGetRefPic(rcListPic, max(0, getPOC()-1));
        if ( pPrevPic->getSlice( 0 )->getPOC() != max( 0, getPOC()-1 ) )
        {
          pPrevPic = xGetRefPic( rcListPic, getPOC() );
        }
        setLastEncPic(pPrevPic);
      }
      return;
    }
  }

  if ( !checkNumPocTotalCurr && m_pRPS->getNumberOfPictures() == 0 )
  {
    TComPic *pPrevPic = xGetRefPic( rcListPic, max( 0, getPOC() - 1 ) );
    if ( pPrevPic->getSlice( 0 )->getPOC() != max( 0, getPOC() - 1 ) )
    {
      pPrevPic = xGetRefPic( rcListPic, getPOC() );
    }
    setLastEncPic( pPrevPic );
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

  for(i =
m_pRPS->getNumberOfNegativePictures()+m_pRPS->getNumberOfPositivePictures()+m_pRPS->getNumberOfLongtermPictures()-1; i >
m_pRPS->getNumberOfNegativePictures()+m_pRPS->getNumberOfPositivePictures()-1 ; i--)
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

  // ref_pic_list_init
  TComPic*  rpsCurrList0[MAX_NUM_REF+1];
  TComPic*  rpsCurrList1[MAX_NUM_REF+1];
  Int numPicTotalCurr = NumPicStCurr0 + NumPicStCurr1 + NumPicLtCurr;

  if ( getPPS()->getPpsScreenExtension().getUseIntraBlockCopy() )
  {
    numPicTotalCurr++;
  }

  if (checkNumPocTotalCurr)
  {
    // The variable NumPocTotalCurr is derived as specified in subclause 7.4.7.2. It is a requirement of bitstream
conformance that the following applies to the value of NumPocTotalCurr:
    // - If the current picture is a BLA or CRA picture, the value of NumPocTotalCurr shall be equal to 0.
    // - Otherwise, when the current picture contains a P or B slice, the value of NumPocTotalCurr shall not be equal to
0. if (getRapPicFlag())
    {
      if ( getPPS()->getPpsScreenExtension().getUseIntraBlockCopy() )
      {
        assert(numPicTotalCurr == 1);
      }
      else
      {
        assert(numPicTotalCurr == 0);
      }
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
  for ( i=0; i<NumPicStCurr1; i++, cIdx++)
  {
    rpsCurrList0[cIdx] = RefPicSetStCurr1[i];
  }
  for ( i=0; i<NumPicLtCurr;  i++, cIdx++)
  {
    rpsCurrList0[cIdx] = RefPicSetLtCurr[i];
  }
  if ( getPPS()->getPpsScreenExtension().getUseIntraBlockCopy() )
  {
    rpsCurrList0[cIdx++] = getCurPicLongTerm();
  }
  assert(cIdx == numPicTotalCurr);

  if (m_eSliceType==B_SLICE)
  {
    cIdx = 0;
    for ( i=0; i<NumPicStCurr1; i++, cIdx++)
    {
      rpsCurrList1[cIdx] = RefPicSetStCurr1[i];
    }
    for ( i=0; i<NumPicStCurr0; i++, cIdx++)
    {
      rpsCurrList1[cIdx] = RefPicSetStCurr0[i];
    }
    for ( i=0; i<NumPicLtCurr;  i++, cIdx++)
    {
      rpsCurrList1[cIdx] = RefPicSetLtCurr[i];
    }
    if ( getPPS()->getPpsScreenExtension().getUseIntraBlockCopy() )
    {
      rpsCurrList1[cIdx++] = getCurPicLongTerm();
    }
    assert(cIdx == numPicTotalCurr);
  }

  ::memset(m_bIsUsedAsLongTerm, 0, sizeof(m_bIsUsedAsLongTerm));

  for (Int rIdx = 0; rIdx < m_aiNumRefIdx[REF_PIC_LIST_0]; rIdx ++)
  {
    cIdx = m_RefPicListModification.getRefPicListModificationFlagL0() ? m_RefPicListModification.getRefPicSetIdxL0(rIdx)
: rIdx % numPicTotalCurr; assert(cIdx >= 0 && cIdx < numPicTotalCurr); m_apcRefPicList[REF_PIC_LIST_0][rIdx] =
rpsCurrList0[ cIdx ]; m_bIsUsedAsLongTerm[REF_PIC_LIST_0][rIdx] = ( cIdx >= NumPicStCurr0 + NumPicStCurr1 );
  }

  if( getPPS()->getPpsScreenExtension().getUseIntraBlockCopy() &&
      !m_RefPicListModification.getRefPicListModificationFlagL0() && numPicTotalCurr > m_aiNumRefIdx[REF_PIC_LIST_0] )
  {
     m_apcRefPicList[REF_PIC_LIST_0][m_aiNumRefIdx[REF_PIC_LIST_0] - 1] = getCurPicLongTerm();
     m_bIsUsedAsLongTerm[REF_PIC_LIST_0][m_aiNumRefIdx[REF_PIC_LIST_0] - 1] = true;
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
      cIdx = m_RefPicListModification.getRefPicListModificationFlagL1() ?
m_RefPicListModification.getRefPicSetIdxL1(rIdx) : rIdx % numPicTotalCurr; assert(cIdx >= 0 && cIdx < numPicTotalCurr);
      m_apcRefPicList[REF_PIC_LIST_1][rIdx] = rpsCurrList1[ cIdx ];
      m_bIsUsedAsLongTerm[REF_PIC_LIST_1][rIdx] = ( cIdx >= NumPicStCurr0 + NumPicStCurr1 );
    }
  }
}

Bool TComSlice::isOnlyCurrentPictureAsReference()
{
  if ( m_eSliceType == I_SLICE )
  {
    return true;
  }

  for ( Int i=0; i < getNumRefIdx( REF_PIC_LIST_0 ); i++ )
  {
    if ( getRefPic( REF_PIC_LIST_0, i )->getPOC() != getPOC() )
    {
      return false;
    }
  }

  for ( Int i=0; i < getNumRefIdx( REF_PIC_LIST_1 ); i++ )
  {
    if ( getRefPic( REF_PIC_LIST_1, i )->getPOC() != getPOC() )
    {
      return false;
    }
  }

  return true;
}
TCH */
/*
Void TComSlice::setRefPOCListSliceHeader() {
  assert( m_eSliceType != I_SLICE );

  static const UInt MAX_NUM_NEGATIVE_PICTURES = 16;
  Int               RefPicPOCSetStCurr0[MAX_NUM_NEGATIVE_PICTURES];
  Int               RefPicPOCSetStCurr1[MAX_NUM_NEGATIVE_PICTURES];
  Int               RefPicPOCSetLtCurr[MAX_NUM_NEGATIVE_PICTURES];
  UInt              NumPicStCurr0 = 0;
  UInt              NumPicStCurr1 = 0;
  UInt              NumPicLtCurr  = 0;
  Int               i;

  for ( i = 0; i < m_pRPS->getNumberOfNegativePictures(); i++ ) {
    if ( m_pRPS->getUsed( i ) ) {
      RefPicPOCSetStCurr0[NumPicStCurr0] = getPOC() + m_pRPS->getDeltaPOC( i );
      NumPicStCurr0++;
    }
  }

  for ( ; i < m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures(); i++ ) {
    if ( m_pRPS->getUsed( i ) ) {
      RefPicPOCSetStCurr1[NumPicStCurr1] = getPOC() + m_pRPS->getDeltaPOC( i );
      NumPicStCurr1++;
    }
  }

  for ( i = m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures() +
            m_pRPS->getNumberOfLongtermPictures() - 1;
        i > m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures() - 1; i-- ) {
    if ( m_pRPS->getUsed( i ) ) {
      RefPicPOCSetLtCurr[NumPicLtCurr] = m_pRPS->getPOC( i );
      NumPicLtCurr++;
    }
  }

  // ref_pic_list_init
  Int rpsPOCCurrList0[MAX_NUM_REF + 1];
  Int rpsPOCCurrList1[MAX_NUM_REF + 1];
  Int numPicTotalCurr = NumPicStCurr0 + NumPicStCurr1 + NumPicLtCurr;

  if ( getPPS()->getPpsScreenExtension().getUseIntraBlockCopy() ) { numPicTotalCurr++; }

  if ( getRapPicFlag() ) {
    if ( getPPS()->getPpsScreenExtension().getUseIntraBlockCopy() ) {
      assert( numPicTotalCurr == 1 );
    } else {
      assert( numPicTotalCurr == 0 );
    }
  }

  assert( numPicTotalCurr > 0 );
  assert( numPicTotalCurr <= 8 );

  m_aiNumRefIdx[0] = getNumRefIdx( REF_PIC_LIST_0 );
  m_aiNumRefIdx[1] = getNumRefIdx( REF_PIC_LIST_1 );

  Int cIdx = 0;
  for ( i = 0; i < NumPicStCurr0; i++, cIdx++ ) { rpsPOCCurrList0[cIdx] = RefPicPOCSetStCurr0[i]; }
  for ( i = 0; i < NumPicStCurr1; i++, cIdx++ ) { rpsPOCCurrList0[cIdx] = RefPicPOCSetStCurr1[i]; }
  for ( i = 0; i < NumPicLtCurr; i++, cIdx++ ) { rpsPOCCurrList0[cIdx] = RefPicPOCSetLtCurr[i]; }
  if ( getPPS()->getPpsScreenExtension().getUseIntraBlockCopy() ) { rpsPOCCurrList0[cIdx++] = getPOC(); }
  assert( cIdx == numPicTotalCurr );

  if ( m_eSliceType == B_SLICE ) {
    cIdx = 0;
    for ( i = 0; i < NumPicStCurr1; i++, cIdx++ ) { rpsPOCCurrList1[cIdx] = RefPicPOCSetStCurr1[i]; }
    for ( i = 0; i < NumPicStCurr0; i++, cIdx++ ) { rpsPOCCurrList1[cIdx] = RefPicPOCSetStCurr0[i]; }
    for ( i = 0; i < NumPicLtCurr; i++, cIdx++ ) { rpsPOCCurrList1[cIdx] = RefPicPOCSetLtCurr[i]; }
    if ( getPPS()->getPpsScreenExtension().getUseIntraBlockCopy() ) { rpsPOCCurrList1[cIdx++] = getPOC(); }
    assert( cIdx == numPicTotalCurr );
  }

  for ( Int rIdx = 0; rIdx < m_aiNumRefIdx[REF_PIC_LIST_0]; rIdx++ ) {
    cIdx = m_RefPicListModification.getRefPicListModificationFlagL0()
               ? m_RefPicListModification.getRefPicSetIdxL0( rIdx )
               : rIdx % numPicTotalCurr;
    assert( cIdx >= 0 && cIdx < numPicTotalCurr );
    m_aiRefPOCList[REF_PIC_LIST_0][rIdx] = rpsPOCCurrList0[cIdx];
  }
  if ( m_eSliceType != B_SLICE ) {
    m_aiNumRefIdx[REF_PIC_LIST_1] = 0;
  } else {
    for ( Int rIdx = 0; rIdx < m_aiNumRefIdx[REF_PIC_LIST_1]; rIdx++ ) {
      cIdx = m_RefPicListModification.getRefPicListModificationFlagL1()
                 ? m_RefPicListModification.getRefPicSetIdxL1( rIdx )
                 : rIdx % numPicTotalCurr;
      assert( cIdx >= 0 && cIdx < numPicTotalCurr );
      m_aiRefPOCList[REF_PIC_LIST_1][rIdx] = rpsPOCCurrList1[cIdx];
    }
  }
  if ( getPPS()->getPpsScreenExtension().getUseIntraBlockCopy() &&
       !m_RefPicListModification.getRefPicListModificationFlagL0() &&
       numPicTotalCurr > m_aiNumRefIdx[REF_PIC_LIST_0] ) {
    m_aiRefPOCList[REF_PIC_LIST_0][m_aiNumRefIdx[REF_PIC_LIST_0] - 1] = getPOC();
  }
}
*/
/*
Int TComSlice::getNumRpsCurrTempList() const {
  Int numRpsCurrTempList = 0;

  if ( m_eSliceType == I_SLICE ) { return 0; }
  for ( UInt i = 0; i < m_pRPS->getNumberOfNegativePictures() + m_pRPS->getNumberOfPositivePictures() +
                            m_pRPS->getNumberOfLongtermPictures();
        i++ ) {
    if ( m_pRPS->getUsed( i ) ) { numRpsCurrTempList++; }
  }
  if ( getPPS()->getPpsScreenExtension().getUseIntraBlockCopy() ) { numRpsCurrTempList++; }

  return numRpsCurrTempList;
}
*/
Void TComSlice_avc::initEqualRef() {
  for ( Int iDir = 0; iDir < NUM_REF_PIC_LIST_01; iDir++ ) {
    for ( Int iRefIdx1 = 0; iRefIdx1 < MAX_NUM_REF; iRefIdx1++ ) {
      for ( Int iRefIdx2 = iRefIdx1; iRefIdx2 < MAX_NUM_REF; iRefIdx2++ ) {
        m_abEqualRef[iDir][iRefIdx1][iRefIdx2] = m_abEqualRef[iDir][iRefIdx2][iRefIdx1] =
            ( iRefIdx1 == iRefIdx2 ? true : false );
      }
    }
  }
}

/* TCH

Void TComSlice::checkColRefIdx(UInt curSliceIdx, TComPic* pic)
{
  Int i;
  TComSlice* curSlice = pic->getSlice(curSliceIdx);
  Int currColRefPOC =  curSlice->getRefPOC( RefPicList(1 - curSlice->getColFromL0Flag()), curSlice->getColRefIdx());
  TComSlice* preSlice;
  Int preColRefPOC;
  for(i=curSliceIdx-1; i>=0; i--)
  {
    preSlice = pic->getSlice(i);
    if(preSlice->getSliceType() != I_SLICE)
    {
      preColRefPOC  = preSlice->getRefPOC( RefPicList(1 - preSlice->getColFromL0Flag()), preSlice->getColRefIdx());
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

Void TComSlice::checkCRA(const TComReferencePictureSet *pReferencePictureSet, Int& pocCRA, NalUnitType&
associatedIRAPType, TComList<TComPic *>& rcListPic)
{
  for(Int i = 0; i <
pReferencePictureSet->getNumberOfNegativePictures()+pReferencePictureSet->getNumberOfPositivePictures(); i++)
  {
    if(pocCRA < MAX_UINT && getPOC() > pocCRA)
    {
      assert(getPOC()+pReferencePictureSet->getDeltaPOC(i) >= pocCRA);
    }
  }
  for(Int i = pReferencePictureSet->getNumberOfNegativePictures()+pReferencePictureSet->getNumberOfPositivePictures(); i
< pReferencePictureSet->getNumberOfPictures(); i++)
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
  if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL || getNalUnitType() == NAL_UNIT_CODED_SLICE_IDR_N_LP ) // IDR
picture found
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
TCH */

/** Function for marking the reference pictures when an IDR/CRA/CRANT/BLA/BLANT is encountered.
 * \param pocCRA POC of the CRA/CRANT/BLA/BLANT picture
 * \param bRefreshPending flag indicating if a deferred decoding refresh is pending
 * \param rcListPic reference to the reference picture list
 * This function marks the reference pictures as "unused for reference" in the following conditions.
 * If the nal_unit_type is IDR/BLA/BLANT, all pictures in the reference picture list
 * are marked as "unused for reference"
 *    If the nal_unit_type is BLA/BLANT, set the pocCRA to the temporal reference of the current picture.
 * Otherwise
 *    If the bRefreshPending flag is true (a deferred decoding refresh is pending) and the current
 *    temporal reference is greater than the temporal reference of the latest CRA/CRANT/BLA/BLANT picture (pocCRA),
 *    mark all reference pictures except the latest CRA/CRANT/BLA/BLANT picture as "unused for reference" and set
 *    the bRefreshPending flag to false.
 *    If the nal_unit_type is CRA/CRANT, set the bRefreshPending flag to true and pocCRA to the temporal
 *    reference of the current picture.
 * Note that the current picture is already placed in the reference list and its marking is not changed.
 * If the current picture has a nal_ref_idc that is not 0, it will remain marked as "used for reference".
 */
/* TCH
Void TComSlice::decodingRefreshMarking(Int& pocCRA, Bool& bRefreshPending, TComList<TComPic*>& rcListPic, const bool
bEfficientFieldIRAPEnabled)
{
  TComPic* rpcPic;
  Int      pocCurr = getPOC();

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
      if (rpcPic->getPOC() != pocCurr)
      {
        rpcPic->getSlice(0)->setReferenced(false);
        rpcPic->getHashMap()->clearAll();
      }
      iterPic++;
    }
    if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
      || getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP )
    {
      pocCRA = pocCurr;
    }
    if (bEfficientFieldIRAPEnabled)
    {
      bRefreshPending = true;
    }
  }
  else // CRA or No DR
  {
    if(bEfficientFieldIRAPEnabled && (getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
getAssociatedIRAPType() == NAL_UNIT_CODED_SLICE_IDR_W_RADL))
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
            rpcPic->getHashMap()->clearAll();
          }
          iterPic++;
        }
        bRefreshPending = false;
      }
    }
    else
    {
      if (bRefreshPending==true && pocCurr > pocCRA) // CRA reference marking pending
      {
        TComList<TComPic*>::iterator iterPic = rcListPic.begin();
        while (iterPic != rcListPic.end())
        {
          rpcPic = *(iterPic);
          if (rpcPic->getPOC() != pocCurr && rpcPic->getPOC() != pocCRA)
          {
            rpcPic->getSlice(0)->setReferenced(false);
            rpcPic->getHashMap()->clearAll();
          }
          iterPic++;
        }
        bRefreshPending = false;
      }
    }
    if ( getNalUnitType() == NAL_UNIT_CODED_SLICE_CRA ) // CRA picture found
    {
      bRefreshPending = true;
      pocCRA = pocCurr;
    }
  }
}
TCH */

Void TComSlice_avc::copySliceInfo( TComSlice_avc* pSrc ) {
  assert( pSrc != NULL );

  Int i, j, k;

  m_iPOC         = pSrc->m_iPOC;
  m_eNalUnitType = pSrc->m_eNalUnitType;
  m_eSliceType   = pSrc->m_eSliceType;
  m_iSliceQp     = pSrc->m_iSliceQp;
#if ADAPTIVE_QP_SELECTION
  m_iSliceQpBase = pSrc->m_iSliceQpBase;
#endif
  //m_ChromaQpAdjEnabled             = pSrc->m_ChromaQpAdjEnabled;
  m_deblockingFilterDisable        = pSrc->m_deblockingFilterDisable;
  m_deblockingFilterOverrideFlag   = pSrc->m_deblockingFilterOverrideFlag;
  m_deblockingFilterBetaOffsetDiv2 = pSrc->m_deblockingFilterBetaOffsetDiv2;
  m_deblockingFilterTcOffsetDiv2   = pSrc->m_deblockingFilterTcOffsetDiv2;

  for ( i = 0; i < NUM_REF_PIC_LIST_01; i++ ) { m_aiNumRefIdx[i] = pSrc->m_aiNumRefIdx[i]; }

  for ( i = 0; i < MAX_NUM_REF; i++ ) { m_list1IdxToList0Idx[i] = pSrc->m_list1IdxToList0Idx[i]; }

  m_bCheckLDC     = pSrc->m_bCheckLDC;
  m_iSliceQpDelta = pSrc->m_iSliceQpDelta;
  for ( UInt component = 0; component < MAX_NUM_COMPONENT; component++ ) {
    m_iSliceChromaQpDelta[component] = pSrc->m_iSliceChromaQpDelta[component];
  }
  for ( i = 0; i < NUM_REF_PIC_LIST_01; i++ ) {
    for ( j = 0; j < MAX_NUM_REF; j++ ) {
      m_apcRefPicList[i][j]     = pSrc->m_apcRefPicList[i][j];
      m_aiRefPOCList[i][j]      = pSrc->m_aiRefPOCList[i][j];
      m_bIsUsedAsLongTerm[i][j] = pSrc->m_bIsUsedAsLongTerm[i][j];
    }
    m_bIsUsedAsLongTerm[i][MAX_NUM_REF] = pSrc->m_bIsUsedAsLongTerm[i][MAX_NUM_REF];
  }
  m_iDepth = pSrc->m_iDepth;

  // referenced slice
  m_bRefenced = pSrc->m_bRefenced;

  // access channel
  //m_pRPS     = pSrc->m_pRPS;
  m_iLastIDR = pSrc->m_iLastIDR;

  m_pcPic = pSrc->m_pcPic;

  m_colFromL0Flag = pSrc->m_colFromL0Flag;
  m_colRefIdx     = pSrc->m_colRefIdx;

  setLambdas( pSrc->getLambdas() );

  for ( i = 0; i < NUM_REF_PIC_LIST_01; i++ ) {
    for ( j = 0; j < MAX_NUM_REF; j++ ) {
      for ( k = 0; k < MAX_NUM_REF; k++ ) { m_abEqualRef[i][j][k] = pSrc->m_abEqualRef[i][j][k]; }
    }
  }

  m_uiTLayer             = pSrc->m_uiTLayer;
  m_bTLayerSwitchingFlag = pSrc->m_bTLayerSwitchingFlag;

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

  //for ( UInt e = 0; e < NUM_REF_PIC_LIST_01; e++ ) {
  //  for ( UInt n = 0; n < MAX_NUM_REF; n++ ) {
  //    memcpy( m_weightPredTable[e][n], pSrc->m_weightPredTable[e][n], sizeof( WPScalingParam ) * MAX_NUM_COMPONENT );
  //  }
 // }

  for ( UInt ch = 0; ch < MAX_NUM_CHANNEL_TYPE; ch++ ) { m_saoEnabledFlag[ch] = pSrc->m_saoEnabledFlag[ch]; }

  m_cabacInitFlag = pSrc->m_cabacInitFlag;

  m_bLMvdL1Zero              = pSrc->m_bLMvdL1Zero;
  m_LFCrossSliceBoundaryFlag = pSrc->m_LFCrossSliceBoundaryFlag;
  m_enableTMVPFlag           = pSrc->m_enableTMVPFlag;
  m_maxNumMergeCand          = pSrc->m_maxNumMergeCand;
  m_encCABACTableIdx         = pSrc->m_encCABACTableIdx;
  //m_RefPicListModification   = pSrc->m_RefPicListModification;
}

/** Function for setting the slice's temporal layer ID and corresponding temporal_layer_switching_point_flag.
 * \param uiTLayer Temporal layer ID of the current slice
 * The decoder calls this function to set temporal_layer_switching_point_flag for each temporal layer based on
 * the SPS's temporal_id_nesting_flag and the parsed PPS.  Then, current slice's temporal layer ID and
 * temporal_layer_switching_point_flag is set accordingly.
 */
Void TComSlice_avc::setTLayerInfo( UInt uiTLayer ) { m_uiTLayer = uiTLayer; }

/** Function for checking if this is a switching-point
 */
/* TCH
Bool TComSlice::isTemporalLayerSwitchingPoint(TComList<TComPic*>& rcListPic)
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
TCH */

/** Function for checking if this is a STSA candidate
 */
/* TCH
Bool TComSlice::isStepwiseTemporalLayerSwitchingPointCandidate(TComList<TComPic*>& rcListPic)
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
TCH */

/* TCH
Void TComSlice::checkLeadingPictureRestrictions(TComList<TComPic*>& rcListPic)
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
    if(!rpcPic->getReconMark())
    {
      continue;
    }
    if (rpcPic->getPOC() == this->getPOC())
    {
      continue;
    }

    // Any picture that has PicOutputFlag equal to 1 that precedes an IRAP picture
    // in decoding order shall precede the IRAP picture in output order.
    // (Note that any picture following in output order would be present in the DPB)
    if(rpcPic->getSlice(0)->getPicOutputFlag() == 1 && !this->getNoOutputPriorPicsFlag())
    {
      if(nalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP    ||
         nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_LP    ||
         nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_RADL  ||
         nalUnitType == NAL_UNIT_CODED_SLICE_CRA         ||
         nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP    ||
         nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL)
      {
        assert(rpcPic->getPOC() < this->getPOC());
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
            assert(rpcPic->getPOC() < this->getPOC());
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
          assert(rpcPic->getPOC() <= this->getAssociatedIRAPPOC());
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
TCH */

/** Function for applying picture marking based on the Reference Picture Set in pReferencePictureSet.
 */
/* TCH
Void TComSlice::applyReferencePictureSet( TComList<TComPic*>& rcListPic, const TComReferencePictureSet
*pReferencePictureSet)
{
  TComPic* rpcPic;
  Int i, isReference;

  checkLeadingPictureRestrictions(rcListPic);

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
      if(!rpcPic->getIsLongTerm() && rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getPOC() +
pReferencePictureSet->getDeltaPOC(i))
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
    // mark the picture as "unused for reference" if it is not in
    // the Reference Picture Set
    if(rpcPic->getPicSym()->getSlice(0)->getPOC() != this->getPOC() && isReference == 0)
    {
      rpcPic->getSlice( 0 )->setReferenced( false );
      rpcPic->getHashMap()->clearAll();
      rpcPic->setUsedByCurr(0);
      rpcPic->setIsLongTerm(0);
    }
    //check that pictures of higher temporal layers are not used
    assert(rpcPic->getSlice( 0
)->isReferenced()==0||rpcPic->getUsedByCurr()==0||rpcPic->getTLayer()<=this->getTLayer());
    //check that pictures of higher or equal temporal layer are not in the RPS if the current picture is a TSA picture
    if(this->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_R || this->getNalUnitType() == NAL_UNIT_CODED_SLICE_TSA_N)
    {
      assert(rpcPic->getSlice( 0 )->isReferenced()==0||rpcPic->getTLayer()<this->getTLayer());
    }
    //check that pictures marked as temporal layer non-reference pictures are not used for reference
    if(rpcPic->getPicSym()->getSlice(0)->getPOC() != this->getPOC() && rpcPic->getTLayer()==this->getTLayer())
    {
      assert(rpcPic->getSlice( 0 )->isReferenced()==0||rpcPic->getUsedByCurr()==0||rpcPic->getSlice( 0
)->getTemporalLayerNonReferenceFlag()==false);
    }
  }
}
TCH */

/** Function for applying picture marking based on the Reference Picture Set in pReferencePictureSet.
 */
/* TCH
Int TComSlice::checkThatAllRefPicsAreAvailable( TComList<TComPic*>& rcListPic, const TComReferencePictureSet
*pReferencePictureSet, Bool printErrors, Int pocRandomAccess, Bool bUseRecoveryPoint)
{
  Int atLeastOneUnabledByRecoveryPoint = 0;
  Int atLeastOneFlushedByPreviousIDR = 0;
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
        if(rpcPic->getIsLongTerm() && (rpcPic->getPicSym()->getSlice(0)->getPOC()) == pReferencePictureSet->getPOC(i) &&
rpcPic->getSlice(0)->isReferenced())
        {
          if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() +
pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess)
          {
            isAvailable = 0;
          }
          else
          {
            isAvailable = 1;
          }
        }
      }
      else
      {
        Int pocCycle = 1<<rpcPic->getPicSym()->getSlice(0)->getSPS()->getBitsForPOC();
        Int curPoc = rpcPic->getPicSym()->getSlice(0)->getPOC() & (pocCycle-1);
        Int refPoc = pReferencePictureSet->getPOC(i) & (pocCycle-1);
        if(rpcPic->getIsLongTerm() && curPoc == refPoc && rpcPic->getSlice(0)->isReferenced())
        {
          if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() +
pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess)
          {
            isAvailable = 0;
          }
          else
          {
            isAvailable = 1;
          }
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
          if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() +
pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess)
          {
            isAvailable = 0;
          }
          else
          {
            isAvailable = 1;
            rpcPic->setIsLongTerm(1);
            break;
          }
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
            printf("\nLong-term reference picture with POC = %3d seems to have been removed or not correctly decoded.",
this->getPOC() + pReferencePictureSet->getDeltaPOC(i));
          }
          atLeastOneRemoved = 1;
        }
        else
        {
          if(printErrors)
          {
            printf("\nLong-term reference picture with POC = %3d is lost or not correctly decoded!", this->getPOC() +
pReferencePictureSet->getDeltaPOC(i));
          }
          atLeastOneLost = 1;
          iPocLost=this->getPOC() + pReferencePictureSet->getDeltaPOC(i);
        }
      }
      else if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess)
      {
        atLeastOneUnabledByRecoveryPoint = 1;
      }
      else if(bUseRecoveryPoint && (this->getAssociatedIRAPType()==NAL_UNIT_CODED_SLICE_IDR_N_LP ||
this->getAssociatedIRAPType()==NAL_UNIT_CODED_SLICE_IDR_W_RADL))
      {
        atLeastOneFlushedByPreviousIDR = 1;
      }
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

      if(!rpcPic->getIsLongTerm() && rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getPOC() +
pReferencePictureSet->getDeltaPOC(i) && rpcPic->getSlice(0)->isReferenced())
      {
        if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess && this->getPOC() +
pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess)
        {
          isAvailable = 0;
        }
        else
        {
          isAvailable = 1;
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
            printf("\nShort-term reference picture with POC = %3d seems to have been removed or not correctly decoded.",
this->getPOC() + pReferencePictureSet->getDeltaPOC(i));
          }
          atLeastOneRemoved = 1;
        }
        else
        {
          if(printErrors)
          {
            printf("\nShort-term reference picture with POC = %3d is lost or not correctly decoded!", this->getPOC() +
pReferencePictureSet->getDeltaPOC(i));
          }
          atLeastOneLost = 1;
          iPocLost=this->getPOC() + pReferencePictureSet->getDeltaPOC(i);
        }
      }
      else if(bUseRecoveryPoint && this->getPOC() > pocRandomAccess)
      {
        atLeastOneUnabledByRecoveryPoint = 1;
      }
      else if(bUseRecoveryPoint && (this->getAssociatedIRAPType()==NAL_UNIT_CODED_SLICE_IDR_N_LP ||
this->getAssociatedIRAPType()==NAL_UNIT_CODED_SLICE_IDR_W_RADL))
      {
        atLeastOneFlushedByPreviousIDR = 1;
      }
    }
  }

  if(atLeastOneUnabledByRecoveryPoint || atLeastOneFlushedByPreviousIDR)
  {
    return -1;
  }
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
TCH */

/** Function for constructing an explicit Reference Picture Set out of the available pictures in a referenced Reference
 * Picture Set
 */
/* TCH
Void TComSlice::createExplicitReferencePictureSetFromReference( TComList<TComPic*>& rcListPic, const
TComReferencePictureSet *pReferencePictureSet, Bool isRAP, Int pocRandomAccess, Bool bUseRecoveryPoint, const Bool
bEfficientFieldIRAPEnabled)
{
  TComPic* rpcPic;
  Int i, j;
  Int k = 0;
  Int nrOfNegativePictures = 0;
  Int nrOfPositivePictures = 0;
  TComReferencePictureSet* pLocalRPS = this->getLocalRPS();
  (*pLocalRPS)=TComReferencePictureSet();

  Bool irapIsInRPS = false; // Used when bEfficientFieldIRAPEnabled==true

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

      if(rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getPOC() + pReferencePictureSet->getDeltaPOC(i) &&
rpcPic->getSlice(0)->isReferenced())
      {
        // This picture exists as a reference picture
        // and should be added to the explicit Reference Picture Set
        pLocalRPS->setDeltaPOC(k, pReferencePictureSet->getDeltaPOC(i));
        pLocalRPS->setUsed(k, pReferencePictureSet->getUsed(i) && (!isRAP));
        if (bEfficientFieldIRAPEnabled)
        {
          pLocalRPS->setUsed(k, pLocalRPS->getUsed(k) && !(bUseRecoveryPoint && this->getPOC() > pocRandomAccess &&
this->getPOC() + pReferencePictureSet->getDeltaPOC(i) < pocRandomAccess) );
        }

        if(pLocalRPS->getDeltaPOC(k) < 0)
        {
          nrOfNegativePictures++;
        }
        else
        {
          if(bEfficientFieldIRAPEnabled && rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getAssociatedIRAPPOC() &&
this->getAssociatedIRAPPOC() == this->getPOC()+1)
          {
            irapIsInRPS = true;
          }
          nrOfPositivePictures++;
        }
        k++;
      }
    }
  }

  Bool useNewRPS = false;
  // if current picture is complimentary field associated to IRAP, add the IRAP to its RPS.
  if(bEfficientFieldIRAPEnabled && m_pcPic->isField() && !irapIsInRPS)
  {
    TComList<TComPic*>::iterator iterPic = rcListPic.begin();
    while ( iterPic != rcListPic.end())
    {
      rpcPic = *(iterPic++);
      if(rpcPic->getPicSym()->getSlice(0)->getPOC() == this->getAssociatedIRAPPOC() && this->getAssociatedIRAPPOC() ==
this->getPOC()+1)
      {
        pLocalRPS->setDeltaPOC(k, 1);
        pLocalRPS->setUsed(k, true);
        nrOfPositivePictures++;
        k ++;
        useNewRPS = true;
      }
    }
  }
  pLocalRPS->setNumberOfNegativePictures(nrOfNegativePictures);
  pLocalRPS->setNumberOfPositivePictures(nrOfPositivePictures);
  pLocalRPS->setNumberOfPictures(nrOfNegativePictures+nrOfPositivePictures);
  // This is a simplistic inter rps example. A smarter encoder will look for a better reference RPS to do the
  // inter RPS prediction with.  Here we just use the reference used by pReferencePictureSet.
  // If pReferencePictureSet is not inter_RPS_predicted, then inter_RPS_prediction is for the current RPS also disabled.
  if (!pReferencePictureSet->getInterRPSPrediction() || useNewRPS )
  {
    pLocalRPS->setInterRPSPrediction(false);
    pLocalRPS->setNumRefIdc(0);
  }
  else
  {
    Int rIdx =  this->getRPSidx() - pReferencePictureSet->getDeltaRIdxMinus1() - 1;
    Int deltaRPS = pReferencePictureSet->getDeltaRPS();
    const TComReferencePictureSet* pcRefRPS = this->getSPS()->getRPSList()->getReferencePictureSet(rIdx);
    Int iRefPics = pcRefRPS->getNumberOfPictures();
    Int iNewIdc=0;
    for(i=0; i<= iRefPics; i++)
    {
      Int deltaPOC = ((i != iRefPics)? pcRefRPS->getDeltaPOC(i) : 0);  // check if the reference abs POC is >= 0
      Int iRefIdc = 0;
      for (j=0; j < pLocalRPS->getNumberOfPictures(); j++) // loop through the  pictures in the new RPS
      {
        if ( (deltaPOC + deltaRPS) == pLocalRPS->getDeltaPOC(j))
        {
          if (pLocalRPS->getUsed(j))
          {
            iRefIdc = 1;
          }
          else
          {
            iRefIdc = 2;
          }
        }
      }
      pLocalRPS->setRefIdc(i, iRefIdc);
      iNewIdc++;
    }
    pLocalRPS->setInterRPSPrediction(true);
    pLocalRPS->setNumRefIdc(iNewIdc);
    pLocalRPS->setDeltaRPS(deltaRPS);
    pLocalRPS->setDeltaRIdxMinus1(pReferencePictureSet->getDeltaRIdxMinus1() +
this->getSPS()->getRPSList()->getNumberOfReferencePictureSets() - this->getRPSidx());
  }

  this->setRPS(pLocalRPS);
  this->setRPSidx(-1);
}
TCH */

//! get AC and DC values for weighted pred
/*
Void  TComSlice::getWpAcDcParam(WPACDCParam *&wp)
{
  wp = m_weightACDCParam;
}
*/
//! init AC and DC values for weighted pred
/*
Void  TComSlice::initWpAcDcParam()
{
  for(Int iComp = 0; iComp < MAX_NUM_COMPONENT; iComp++ )
  {
    m_weightACDCParam[iComp].iAC = 0;
    m_weightACDCParam[iComp].iDC = 0;
  }
}
*/

//! get tables for weighted prediction
/*
Void  TComSlice::getWpScaling( RefPicList e, Int iRefIdx, WPScalingParam *&wp )
{
  assert (e<NUM_REF_PIC_LIST_01);
  wp = m_weightPredTable[e][iRefIdx];
}
*/
//! reset Default WP tables settings : no weight.
/*
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
*/
//! init WP table
/*
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

        const Int offsetScalingFactor = bUseHighPrecisionPredictionWeighting ? 1 : (1 <<
(sps->getBitDepth(toChannelType(ComponentID(yuv)))-8));

        pwp->w      = pwp->iWeight;
        pwp->o      = pwp->iOffset * offsetScalingFactor; //NOTE: This value of the ".o" variable is never used - .o is
set immediately before it gets used pwp->shift  = pwp->uiLog2WeightDenom; pwp->round  = (pwp->uiLog2WeightDenom>=1) ? (1
<< (pwp->uiLog2WeightDenom-1)) : (0);
      }
    }
  }
}
*/
// ------------------------------------------------------------------------------------------------
// Video parameter set (VPS)
// ------------------------------------------------------------------------------------------------
/*
TComVPS::TComVPS() :
    m_VPSId( 0 ),
    m_uiMaxTLayers( 1 ),
    m_uiMaxLayers( 1 ),
    m_bTemporalIdNestingFlag( false ),
    m_numHrdParameters( 0 ),
    m_maxNuhReservedZeroLayerId( 0 ),
    m_hrdParameters(),
    m_hrdOpSetIdx(),
    m_cprmsPresentFlag() {
  for ( Int i = 0; i < MAX_TLAYER; i++ ) {
    m_numReorderPics[i]       = 0;
    m_uiMaxDecPicBuffering[i] = 1;
    m_uiMaxLatencyIncrease[i] = 0;
  }
}

TComVPS::~TComVPS() {}
*/
// ------------------------------------------------------------------------------------------------
// Sequence parameter set (SPS)
// ------------------------------------------------------------------------------------------------
/*
TComSPSRExt::TComSPSRExt() :
    m_transformSkipRotationEnabledFlag( false ),
    m_transformSkipContextEnabledFlag( false )
    // m_rdpcmEnabledFlag initialized below
    ,
    m_extendedPrecisionProcessingFlag( false ),
    m_intraSmoothingDisabledFlag( false ),
    m_highPrecisionOffsetsEnabledFlag( false ),
    m_persistentRiceAdaptationEnabledFlag( false ),
    m_cabacBypassAlignmentEnabledFlag( false ) {
  for ( UInt signallingModeIndex = 0; signallingModeIndex < NUMBER_OF_RDPCM_SIGNALLING_MODES; signallingModeIndex++ ) {
    m_rdpcmEnabledFlag[signallingModeIndex] = false;
  }
}
*/
/*
TComSPSSCC::TComSPSSCC() :
    m_useIntraBlockCopy( false ),
    m_usePaletteMode( false ),
    m_paletteMaxSize( 31 ),
    m_paletteMaxPredSize( 64 ),
    m_motionVectorResolutionControlIdc( 0 ),
    m_disableIntraBoundaryFilter( false ),
    m_numPalettePred( 0 ) {}
*/
TComSPS_avc::TComSPS_avc() :
    profile_idc( 0 ),
    constrained_set0_flag( false ),
    constrained_set1_flag( false ),
    constrained_set2_flag( false ),
    constrained_set4_flag( false ),
    constrained_set5_flag( false ),
    reserved_zero( 0 ),
    level_idc( 0 ),
    seq_parameter_set_id( 32 ),
    chroma_format_idc( 32 ),
    bit_depth_luma_minus8( 3 ),
    bit_depth_chroma_minus8( false ),
    lossless_qpprime_flag( 0 ),
    separate_colour_plane_flag( 0 ),
    seq_scaling_matrix_present_flag( false ),
    //seq_scaling_list_present_flag( 0 ),
    //ScalingList4x4( 0 ),
    //ScalingList8x8( 0 ),
    //UseDefaultScalingMatrix4x4Flag( 0 ),
    //UseDefaultScalingMatrix8x8Flag( 0 ),
    delta_pic_order_always_zero_flag( 0 ),
    offset_for_non_ref_pic( 0 ),
    offset_for_top_to_bottom_field( 0 ),
    num_ref_frames_in_pic_order_cnt_cycle( 0 ),
    //offset_for_ref_frame( 0 ),
    log2_max_frame_num_minus4( 0 ),
    pic_order_cnt_type( 0 ),
    log2_max_pic_order_cnt_lsb_minus4( 0 ),
    num_ref_frames( 0 ),
    gaps_in_frame_num_value_allowed_flag( false ),
    pic_width_in_mbs_minus1( 0 ),
    pic_height_in_map_units_minus1( 0 ),
    frame_mbs_only_flag( false )
{}

TComSPS_avc::~TComSPS_avc() {
//	m_RPSList.destroy(); 
}

/*
Void TComSPS::createRPSList( Int numRPS ) {
  m_RPSList.destroy();
  m_RPSList.create( numRPS );
}

const Int TComSPS::m_winUnitX[] = {1, 2, 2, 1};
const Int TComSPS::m_winUnitY[] = {1, 2, 1, 1};
*/
/*
TComPPSRExt::TComPPSRExt() :
    m_log2MaxTransformSkipBlockSize( 2 ),
    m_crossComponentPredictionEnabledFlag( false ),
    m_diffCuChromaQpOffsetDepth( 0 ),
    m_chromaQpOffsetListLen( 0 )
// m_ChromaQpAdjTableIncludingNullEntry initialized below
// m_log2SaoOffsetScale initialized below
{
  m_ChromaQpAdjTableIncludingNullEntry[0].u.comp.CbOffset =
      0;  // Array includes entry [0] for the null offset used when cu_chroma_qp_offset_flag=0. This is initialised here
          // and never subsequently changed.
  m_ChromaQpAdjTableIncludingNullEntry[0].u.comp.CrOffset = 0;
  for ( Int ch = 0; ch < MAX_NUM_CHANNEL_TYPE; ch++ ) { m_log2SaoOffsetScale[ch] = 0; }
}
*/
/*
TComPPSSCC::TComPPSSCC() :
    m_useColourTrans( false ),
    m_useSliceACTOffset( false ),
    m_actYQpOffset( -5 ),
    m_actCbQpOffset( -5 ),
    m_actCrQpOffset( -3 ),
    m_numPalettePred( 0 )  // Implies palette pred in PPS deactivated
    ,
    m_usePalettePredictor( false ),
    m_useIntraBlockCopyPps( false ) {
  for ( Int ch = 0; ch < MAX_NUM_CHANNEL_TYPE; ch++ ) { m_palettePredictorBitDepth[ch] = 8; }
}
*/
/*
TComPPS::TComPPS() :
    m_PPSId( 0 ),
    m_SPSId( 0 ),
    m_picInitQPMinus26( 0 ),
    m_useDQP( false ),
    m_bConstrainedIntraPred( false ),
    m_bSliceChromaQpFlag( false ),
    m_uiMaxCuDQPDepth( 0 ),
    m_chromaCbQpOffset( 0 ),
    m_chromaCrQpOffset( 0 ),
    m_numRefIdxL0DefaultActive( 1 ),
    m_numRefIdxL1DefaultActive( 1 ),
    m_TransquantBypassEnabledFlag( false ),
    m_useTransformSkip( false ),
    m_dependentSliceSegmentsEnabledFlag( false ),
    m_tilesEnabledFlag( false ),
    m_entropyCodingSyncEnabledFlag( false ),
    m_loopFilterAcrossTilesEnabledFlag( true ),
    m_uniformSpacingFlag( false ),
    m_numTileColumnsMinus1( 0 ),
    m_numTileRowsMinus1( 0 ),
    m_signDataHidingEnabledFlag( false ),
    m_cabacInitPresentFlag( false ),
    m_sliceHeaderExtensionPresentFlag( false ),
    m_loopFilterAcrossSlicesEnabledFlag( false ),
    m_listsModificationPresentFlag( 0 ),
    m_numExtraSliceHeaderBits( 0 ) {}

TComPPS::~TComPPS() {}
*/
/*
TComReferencePictureSet::TComReferencePictureSet() :
    m_numberOfPictures( 0 ),
    m_numberOfNegativePictures( 0 ),
    m_numberOfPositivePictures( 0 ),
    m_numberOfLongtermPictures( 0 ),
    m_interRPSPrediction( 0 ),
    m_deltaRIdxMinus1( 0 ),
    m_deltaRPS( 0 ),
    m_numRefIdc( 0 ) {
  ::memset( m_deltaPOC, 0, sizeof( m_deltaPOC ) );
  ::memset( m_POC, 0, sizeof( m_POC ) );
  ::memset( m_used, 0, sizeof( m_used ) );
  ::memset( m_refIdc, 0, sizeof( m_refIdc ) );
  ::memset( m_bCheckLTMSB, 0, sizeof( m_bCheckLTMSB ) );
  ::memset( m_pocLSBLT, 0, sizeof( m_pocLSBLT ) );
  ::memset( m_deltaPOCMSBCycleLT, 0, sizeof( m_deltaPOCMSBCycleLT ) );
  ::memset( m_deltaPocMSBPresentFlag, 0, sizeof( m_deltaPocMSBPresentFlag ) );
}

TComReferencePictureSet::~TComReferencePictureSet() {}

Void TComReferencePictureSet::setUsed( Int bufferNum, Bool used ) { m_used[bufferNum] = used; }

Void TComReferencePictureSet::setDeltaPOC( Int bufferNum, Int deltaPOC ) { m_deltaPOC[bufferNum] = deltaPOC; }

Void TComReferencePictureSet::setNumberOfPictures( Int numberOfPictures ) { m_numberOfPictures = numberOfPictures; }

Int TComReferencePictureSet::getUsed( Int bufferNum ) const { return m_used[bufferNum]; }

Int TComReferencePictureSet::getDeltaPOC( Int bufferNum ) const { return m_deltaPOC[bufferNum]; }

Int TComReferencePictureSet::getNumberOfPictures() const { return m_numberOfPictures; }

Int TComReferencePictureSet::getPOC( Int bufferNum ) const { return m_POC[bufferNum]; }

Void TComReferencePictureSet::setPOC( Int bufferNum, Int POC ) { m_POC[bufferNum] = POC; }

Bool TComReferencePictureSet::getCheckLTMSBPresent( Int bufferNum ) const { return m_bCheckLTMSB[bufferNum]; }

Void TComReferencePictureSet::setCheckLTMSBPresent( Int bufferNum, Bool b ) { m_bCheckLTMSB[bufferNum] = b; }

//! set the reference idc value at uiBufferNum entry to the value of iRefIdc
Void TComReferencePictureSet::setRefIdc( Int bufferNum, Int refIdc ) { m_refIdc[bufferNum] = refIdc; }

//! get the reference idc value at uiBufferNum
Int TComReferencePictureSet::getRefIdc( Int bufferNum ) const { return m_refIdc[bufferNum]; }

// Sorts the deltaPOC and Used by current values in the RPS based on the deltaPOC values.
// *  deltaPOC values are sorted with -ve values before the +ve values.  -ve values are in decreasing order.
// *  +ve values are in increasing order.
// * \returns Void
 
Void TComReferencePictureSet::sortDeltaPOC() {
  // sort in increasing order (smallest first)
  for ( Int j = 1; j < getNumberOfPictures(); j++ ) {
    Int  deltaPOC = getDeltaPOC( j );
    Bool used     = getUsed( j );
    for ( Int k = j - 1; k >= 0; k-- ) {
      Int temp = getDeltaPOC( k );
      if ( deltaPOC < temp ) {
        setDeltaPOC( k + 1, temp );
        setUsed( k + 1, getUsed( k ) );
        setDeltaPOC( k, deltaPOC );
        setUsed( k, used );
      }
    }
  }
  // flip the negative values to largest first
  Int numNegPics = getNumberOfNegativePictures();
  for ( Int j = 0, k = numNegPics - 1; j<numNegPics>> 1; j++, k-- ) {
    Int  deltaPOC = getDeltaPOC( j );
    Bool used     = getUsed( j );
    setDeltaPOC( j, getDeltaPOC( k ) );
    setUsed( j, getUsed( k ) );
    setDeltaPOC( k, deltaPOC );
    setUsed( k, used );
  }
}

// Prints the deltaPOC and RefIdc (if available) values in the RPS.
//  A "*" is added to the deltaPOC value if it is Used bu current.
// \returns Void
 
Void TComReferencePictureSet::printDeltaPOC() const {
  printf( "DeltaPOC = { " );
  for ( Int j = 0; j < getNumberOfPictures(); j++ ) {
    printf( "%d%s ", getDeltaPOC( j ), ( getUsed( j ) == 1 ) ? "*" : "" );
  }
  if ( getInterRPSPrediction() ) {
    printf( "}, RefIdc = { " );
    for ( Int j = 0; j < getNumRefIdc(); j++ ) { printf( "%d ", getRefIdc( j ) ); }
  }
  printf( "}\n" );
}
*/
/*
TComRefPicListModification::TComRefPicListModification() :
    m_refPicListModificationFlagL0( false ),
    m_refPicListModificationFlagL1( false ) {
  ::memset( m_RefPicSetIdxL0, 0, sizeof( m_RefPicSetIdxL0 ) );
  ::memset( m_RefPicSetIdxL1, 0, sizeof( m_RefPicSetIdxL1 ) );
}

TComRefPicListModification::~TComRefPicListModification() {}
*/
/*
TComScalingList::TComScalingList() {
  for ( UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++ ) {
    for ( UInt listId = 0; listId < SCALING_LIST_NUM; listId++ ) {
      m_scalingListCoef[sizeId][listId].resize( min<Int>( MAX_MATRIX_COEF_NUM, (Int)g_scalingListSize[sizeId] ) );
    }
  }
}
*/
/** set default quantization matrix to array
 */
/*
Void TComScalingList::setDefaultScalingList() {
  for ( UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++ ) {
    for ( UInt listId = 0; listId < SCALING_LIST_NUM; listId++ ) { processDefaultMatrix( sizeId, listId ); }
  }
}
*/
/** check if use default quantization matrix
 * \returns true if use default quantization matrix in all size
 */
/*
Bool TComScalingList::checkDefaultScalingList() {
  UInt defaultCounter = 0;

  for ( UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++ ) {
    for ( UInt listId = 0; listId < SCALING_LIST_NUM; listId++ ) {
      if ( !memcmp(
               getScalingListAddress( sizeId, listId ), getScalingListDefaultAddress( sizeId, listId ),
               sizeof( Int ) * min( MAX_MATRIX_COEF_NUM, (Int)g_scalingListSize[sizeId] ) )  // check value of matrix
           && ( ( sizeId < SCALING_LIST_16x16 ) || ( getScalingListDC( sizeId, listId ) == 16 ) ) )  // check DC value
      {
        defaultCounter++;
      }
    }
  }

  return ( defaultCounter == ( SCALING_LIST_NUM * SCALING_LIST_SIZE_NUM ) ) ? false : true;
}
*/
/** get scaling matrix from RefMatrixID
 * \param sizeId    size index
 * \param listId    index of input matrix
 * \param refListId index of reference matrix
 */
/*
Void TComScalingList::processRefMatrix( UInt sizeId, UInt listId, UInt refListId ) {
  ::memcpy( getScalingListAddress( sizeId, listId ),
            ( ( listId == refListId ) ? getScalingListDefaultAddress( sizeId, refListId )
                                      : getScalingListAddress( sizeId, refListId ) ),
            sizeof( Int ) * min( MAX_MATRIX_COEF_NUM, (Int)g_scalingListSize[sizeId] ) );
}
*/
/*
Void TComScalingList::checkPredMode( UInt sizeId, UInt listId ) {
  Int predListStep = ( sizeId == SCALING_LIST_32x32 ? ( SCALING_LIST_NUM / NUMBER_OF_PREDICTION_MODES )
                                                    : 1 );  // if 32x32, skip over chroma entries.

  for ( Int predListIdx = (Int)listId; predListIdx >= 0; predListIdx -= predListStep ) {
    if ( !memcmp( getScalingListAddress( sizeId, listId ),
                  ( ( listId == predListIdx ) ? getScalingListDefaultAddress( sizeId, predListIdx )
                                              : getScalingListAddress( sizeId, predListIdx ) ),
                  sizeof( Int ) * min( MAX_MATRIX_COEF_NUM, (Int)g_scalingListSize[sizeId] ) )  // check value of matrix
         && ( ( sizeId < SCALING_LIST_16x16 ) ||
              ( getScalingListDC( sizeId, listId ) == getScalingListDC( sizeId, predListIdx ) ) ) )  // check DC value
    {
      setRefMatrixId( sizeId, listId, predListIdx );
      setScalingListPredModeFlag( sizeId, listId, false );
      return;
    }
  }
  setScalingListPredModeFlag( sizeId, listId, true );
}
*/
static Void outputScalingListHelp( std::ostream& os ) {
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
  for ( UInt sizeIdc = 0; sizeIdc < SCALING_LIST_SIZE_NUM; sizeIdc++ ) {
    for ( UInt listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++ ) {
      if ( ( sizeIdc != SCALING_LIST_32x32 ) || ( listIdc % ( SCALING_LIST_NUM / NUMBER_OF_PREDICTION_MODES ) == 0 ) ) {
        os << "  " << MatrixType[sizeIdc][listIdc] << '\n';
      }
    }
  }
}
/*
Void TComScalingList::outputScalingLists( std::ostream& os ) const {
  for ( UInt sizeIdc = 0; sizeIdc < SCALING_LIST_SIZE_NUM; sizeIdc++ ) {
    const UInt size = min( 8, 4 << ( sizeIdc ) );
    for ( UInt listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++ ) {
      if ( ( sizeIdc != SCALING_LIST_32x32 ) || ( listIdc % ( SCALING_LIST_NUM / NUMBER_OF_PREDICTION_MODES ) == 0 ) ) {
        const Int* src = getScalingListAddress( sizeIdc, listIdc );
        os << ( MatrixType[sizeIdc][listIdc] ) << " =\n  ";
        for ( UInt y = 0; y < size; y++ ) {
          for ( UInt x = 0; x < size; x++, src++ ) { os << std::setw( 3 ) << ( *src ) << ", "; }
          os << ( y + 1 < size ? "\n  " : "\n" );
        }
        if ( sizeIdc > SCALING_LIST_8x8 ) {
          os << MatrixType_DC[sizeIdc][listIdc] << " = \n  " << std::setw( 3 ) << getScalingListDC( sizeIdc, listIdc )
             << "\n";
        }
        os << "\n";
      }
    }
  }
}
*/
/*
Bool TComScalingList::xParseScalingList( const std::string& fileName ) {
  static const Int LINE_SIZE = 1024;
  FILE*            fp        = NULL;
  TChar            line[LINE_SIZE];

  if ( fileName.empty() ) {
    fprintf( stderr, "Error: no scaling list file specified. Help on scaling lists being output\n" );
    outputScalingListHelp( std::cout );
    std::cout << "\n\nExample scaling list file using default values:\n\n";
    outputScalingLists( std::cout );
    exit( 1 );
    return true;
  } else if ( ( fp = fopen( fileName.c_str(), "r" ) ) == (FILE*)NULL ) {
    fprintf( stderr, "Error: cannot open scaling list file %s for reading\n", fileName.c_str() );
    return true;
  }

  for ( UInt sizeIdc = 0; sizeIdc < SCALING_LIST_SIZE_NUM; sizeIdc++ ) {
    const UInt size = min( MAX_MATRIX_COEF_NUM, (Int)g_scalingListSize[sizeIdc] );

    for ( UInt listIdc = 0; listIdc < SCALING_LIST_NUM; listIdc++ ) {
      Int* const src = getScalingListAddress( sizeIdc, listIdc );

      if ( ( sizeIdc == SCALING_LIST_32x32 ) && ( listIdc % ( SCALING_LIST_NUM / NUMBER_OF_PREDICTION_MODES ) !=
                                                  0 ) )  // derive chroma32x32 from chroma16x16
      {
        const Int* srcNextSmallerSize = getScalingListAddress( sizeIdc - 1, listIdc );
        for ( UInt i = 0; i < size; i++ ) { src[i] = srcNextSmallerSize[i]; }
        setScalingListDC( sizeIdc, listIdc,
                          ( sizeIdc > SCALING_LIST_8x8 ) ? getScalingListDC( sizeIdc - 1, listIdc ) : src[0] );
      } else {
        {
          fseek( fp, 0, SEEK_SET );
          Bool bFound = false;
          while ( ( !feof( fp ) ) && ( !bFound ) ) {
            TChar* ret              = fgets( line, LINE_SIZE, fp );
            TChar* findNamePosition = ret == NULL ? NULL : strstr( line, MatrixType[sizeIdc][listIdc] );
            // This could be a match against the DC string as well, so verify it isn't
            if ( findNamePosition != NULL && ( MatrixType_DC[sizeIdc][listIdc] == NULL ||
                                               strstr( line, MatrixType_DC[sizeIdc][listIdc] ) == NULL ) ) {
              bFound = true;
            }
          }
          if ( !bFound ) {
            fprintf( stderr, "Error: cannot find Matrix %s from scaling list file %s\n", MatrixType[sizeIdc][listIdc],
                     fileName.c_str() );
            return true;
          }
        }
        for ( UInt i = 0; i < size; i++ ) {
          Int data;
          if ( fscanf( fp, "%d,", &data ) != 1 ) {
            fprintf( stderr,
                     "Error: cannot read value #%d for Matrix %s from scaling list file %s at file position %ld\n", i,
                     MatrixType[sizeIdc][listIdc], fileName.c_str(), ftell( fp ) );
            return true;
          }
          if ( data < 0 || data > 255 ) {
            fprintf( stderr,
                     "Error: QMatrix entry #%d of value %d for Matrix %s from scaling list file %s at file position "
                     "%ld is out of range (0 to 255)\n",
                     i, data, MatrixType[sizeIdc][listIdc], fileName.c_str(), ftell( fp ) );
            return true;
          }
          src[i] = data;
        }

        // set DC value for default matrix check
        setScalingListDC( sizeIdc, listIdc, src[0] );

        if ( sizeIdc > SCALING_LIST_8x8 ) {
          {
            fseek( fp, 0, SEEK_SET );
            Bool bFound = false;
            while ( ( !feof( fp ) ) && ( !bFound ) ) {
              TChar* ret              = fgets( line, LINE_SIZE, fp );
              TChar* findNamePosition = ret == NULL ? NULL : strstr( line, MatrixType_DC[sizeIdc][listIdc] );
              if ( findNamePosition != NULL ) {
                // This won't be a match against the non-DC string.
                bFound = true;
              }
            }
            if ( !bFound ) {
              fprintf( stderr, "Error: cannot find DC Matrix %s from scaling list file %s\n",
                       MatrixType_DC[sizeIdc][listIdc], fileName.c_str() );
              return true;
            }
          }
          Int data;
          if ( fscanf( fp, "%d,", &data ) != 1 ) {
            fprintf( stderr, "Error: cannot read DC %s from scaling list file %s at file position %ld\n",
                     MatrixType_DC[sizeIdc][listIdc], fileName.c_str(), ftell( fp ) );
            return true;
          }
          if ( data < 0 || data > 255 ) {
            fprintf( stderr,
                     "Error: DC value %d for Matrix %s from scaling list file %s at file position %ld is out of range "
                     "(0 to 255)\n",
                     data, MatrixType[sizeIdc][listIdc], fileName.c_str(), ftell( fp ) );
            return true;
          }
          // overwrite DC value when size of matrix is larger than 16x16
          setScalingListDC( sizeIdc, listIdc, data );
        }
      }
    }
  }
  //  std::cout << "\n\nRead scaling lists of:\n\n";
  //  outputScalingLists(std::cout);

  fclose( fp );
  return false;
}
*/
/** get default address of quantization matrix
 * \param sizeId size index
 * \param listId list index
 * \returns pointer of quantization matrix
 */
/*
const Int* TComScalingList::getScalingListDefaultAddress( UInt sizeId, UInt listId ) {
  const Int* src = 0;
  switch ( sizeId ) {
    case SCALING_LIST_4x4: src = g_quantTSDefault4x4; break;
    case SCALING_LIST_8x8:
    case SCALING_LIST_16x16:
    case SCALING_LIST_32x32:
      src = ( listId < ( SCALING_LIST_NUM / NUMBER_OF_PREDICTION_MODES ) ) ? g_quantIntraDefault8x8
                                                                           : g_quantInterDefault8x8;
      break;
    default:
      assert( 0 );
      src = NULL;
      break;
  }
  return src;
}
*/
/** process of default matrix
 * \param sizeId size index
 * \param listId index of input matrix
 */
/*
Void TComScalingList::processDefaultMatrix( UInt sizeId, UInt listId ) {
  ::memcpy( getScalingListAddress( sizeId, listId ), getScalingListDefaultAddress( sizeId, listId ),
            sizeof( Int ) * min( MAX_MATRIX_COEF_NUM, (Int)g_scalingListSize[sizeId] ) );
  setScalingListDC( sizeId, listId, SCALING_LIST_DC );
}
*/
/** check DC value of matrix for default matrix signaling
 */
/*
Void TComScalingList::checkDcOfMatrix() {
  for ( UInt sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++ ) {
    for ( UInt listId = 0; listId < SCALING_LIST_NUM; listId++ ) {
      // check default matrix?
      if ( getScalingListDC( sizeId, listId ) == 0 ) { processDefaultMatrix( sizeId, listId ); }
    }
  }
}
*/
/*
ParameterSetManager::ParameterSetManager() :
    m_vpsMap( MAX_NUM_VPS ),
    m_spsMap( MAX_NUM_SPS ),
    m_ppsMap( MAX_NUM_PPS ),
    m_activeVPSId( -1 ),
    m_activeSPSId( -1 ) {}

ParameterSetManager::~ParameterSetManager() {}
*/
//! activate a SPS from a active parameter sets SEI message
//! \returns true, if activation is successful
// Bool ParameterSetManager::activateSPSWithSEI(Int spsId)
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
//      printf("Warning: tried to activate SPS using an Active parameter sets SEI message. Referenced VPS does not
//      exist.");
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
/*
Bool ParameterSetManager::activatePPS( Int ppsId, Bool isIRAP ) {
  TComPPS* pps = m_ppsMap.getPS( ppsId );
  if ( pps ) {
    Int spsId = pps->getSPSId();
    if ( !isIRAP && ( spsId != m_activeSPSId ) ) {
      printf( "Warning: tried to activate PPS referring to a inactive SPS at non-IDR." );
    } else {
      TComSPS* sps = m_spsMap.getPS( spsId );
      if ( sps ) {
        Int vpsId = sps->getVPSId();
        if ( !isIRAP && ( vpsId != m_activeVPSId ) ) {
          printf( "Warning: tried to activate PPS referring to a inactive VPS at non-IDR." );
        } else {
          TComVPS* vps = m_vpsMap.getPS( vpsId );
          if ( vps ) {
            m_activeVPSId = vpsId;
            m_activeSPSId = spsId;
            return true;
          } else {
            printf( "Warning: tried to activate PPS that refers to a non-existing VPS." );
          }
        }
      } else {
        printf( "Warning: tried to activate a PPS that refers to a non-existing SPS." );
      }
    }
  } else {
    printf( "Warning: tried to activate non-existing PPS." );
  }

  // Failed to activate if reach here.
  m_activeSPSId = -1;
  m_activeVPSId = -1;
  return false;
}
*/
/*
template <>
Void ParameterSetMap<TComPPS>::setID( TComPPS* parameterSet, const Int psId ) {
  parameterSet->setPPSId( psId );
}
*/
/*
template <>
Void ParameterSetMap<TComSPS>::setID( TComSPS* parameterSet, const Int psId ) {
  parameterSet->setSPSId( psId );
}
*/
/*
ProfileTierLevel::ProfileTierLevel() :
    m_profileSpace( 0 ),
    m_tierFlag( Level::MAIN ),
    m_profileIdc( Profile::NONE ),
    m_levelIdc( Level::NONE ),
    m_progressiveSourceFlag( false ),
    m_interlacedSourceFlag( false ),
    m_nonPackedConstraintFlag( false ),
    m_frameOnlyConstraintFlag( false ) {
  ::memset( m_profileCompatibilityFlag, 0, sizeof( m_profileCompatibilityFlag ) );
}
*/
/*
TComPTL::TComPTL() {
  ::memset( m_subLayerProfilePresentFlag, 0, sizeof( m_subLayerProfilePresentFlag ) );
  ::memset( m_subLayerLevelPresentFlag, 0, sizeof( m_subLayerLevelPresentFlag ) );
}
*/
/*
Void calculateParameterSetChangedFlag( Bool&                     bChanged,
                                       const std::vector<UChar>* pOldData,
                                       const std::vector<UChar>* pNewData ) {
  if ( !bChanged ) {
    if ( ( pOldData == 0 && pNewData != 0 ) || ( pOldData != 0 && pNewData == 0 ) ) {
      bChanged = true;
    } else if ( pOldData != 0 && pNewData != 0 ) {
      // compare the two
      if ( pOldData->size() != pNewData->size() ) {
        bChanged = true;
      } else {
        const UChar* pNewDataArray = &( *pNewData )[0];
        const UChar* pOldDataArray = &( *pOldData )[0];
        if ( memcmp( pOldDataArray, pNewDataArray, pOldData->size() ) ) { bChanged = true; }
      }
    }
  }
}
*/
//! \}
