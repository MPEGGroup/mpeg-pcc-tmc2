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

/** \file     TDecCAVLC.cpp
\brief    CAVLC decoder class
*/

 #include "TDecCAVLC.h"
// #include "SEIread.h"
// #include "TDecSlice.h"
#if Q0048_CGS_3D_ASYMLUT
#include "TCom3DAsymLUT.h"
#endif


//! \ingroup TLibDecoder
//! \{

#if ENC_DEC_TRACE

Void  xTraceSPSHeader (TComSPS *pSPS)
{
  fprintf( g_hTrace, "=========== Sequence Parameter Set ID: %d ===========\n", pSPS->getSPSId() );
}

Void  xTracePPSHeader (TComPPS *pPPS)
{
  fprintf( g_hTrace, "=========== Picture Parameter Set ID: %d ===========\n", pPPS->getPPSId() );
}

Void  xTraceSliceHeader (TComSlice *pSlice)
{
  fprintf( g_hTrace, "=========== Slice ===========\n");
}

#endif

#if SVC_EXTENSION
  ParameterSetMap<TComVPS> ParameterSetManagerDecoder::m_vpsBuffer(MAX_NUM_VPS);
#endif

ParameterSetManagerDecoder::ParameterSetManagerDecoder()
#if SVC_EXTENSION
: m_spsBuffer(MAX_NUM_SPS)
, m_ppsBuffer(MAX_NUM_PPS)
#else
: m_vpsBuffer(MAX_NUM_VPS)
, m_spsBuffer(MAX_NUM_SPS)
, m_ppsBuffer(MAX_NUM_PPS)
#endif
{
}

void TDecCavlc::parseVps()                  { parseVPS(); }
void TDecCavlc::parseSps( int iLayerIndex ) { parseSPS( m_pSPS + iLayerIndex ); }
void TDecCavlc::parsePps( int iLayerIndex ) { parsePPS( &m_c3DAsymLUTPPS, iLayerIndex); }

int  TDecCavlc::parseSliceHeader( int iLayerIndex, NalUnitType iNaluType, int iTemporalIndex )
{
  m_apcSlicePilot->initSlice( iLayerIndex );
  m_apcSlicePilot->setNalUnitType( iNaluType);
#if POC_RESET_RESTRICTIONS
  m_apcSlicePilot->setTLayer( iTemporalIndex );
#endif
  parseSliceHeader( m_apcSlicePilot, &m_parameterSetManagerDecoder );
  return m_apcSlicePilot->getPOC();
}

ParameterSetManagerDecoder::~ParameterSetManagerDecoder()
{
}

TComVPS* ParameterSetManagerDecoder::getPrefetchedVPS  (Int vpsId)
{
  if (m_vpsBuffer.getPS(vpsId) != NULL )
  {
    return m_vpsBuffer.getPS(vpsId);
  }
  else
  {
    return getVPS(vpsId);
  }
}


TComSPS* ParameterSetManagerDecoder::getPrefetchedSPS  (Int spsId)
{
  if (m_spsBuffer.getPS(spsId) != NULL )
  {
    return m_spsBuffer.getPS(spsId);
  }
  else
  {
    return getSPS(spsId);
  }
}

TComPPS* ParameterSetManagerDecoder::getPrefetchedPPS  (Int ppsId)
{
  if (m_ppsBuffer.getPS(ppsId) != NULL )
  {
    return m_ppsBuffer.getPS(ppsId);
  }
  else
  {
    return getPPS(ppsId);
  }
}

Void     ParameterSetManagerDecoder::applyPrefetchedPS()
{
  m_vpsMap.mergePSList(m_vpsBuffer);
  m_ppsMap.mergePSList(m_ppsBuffer);
  m_spsMap.mergePSList(m_spsBuffer);
}

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TDecCavlc::TDecCavlc()
{
  m_apcSlicePilot = new TComSlice;
}

TDecCavlc::~TDecCavlc()
{
  delete m_apcSlicePilot;
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void TDecCavlc::parseShortTermRefPicSet( TComSPS* sps, TComReferencePictureSet* rps, Int idx )
{
  UInt code;
  UInt interRPSPred;
  if (idx > 0)
  {
    READ_FLAG(interRPSPred, "inter_ref_pic_set_prediction_flag");  rps->setInterRPSPrediction(interRPSPred);
  }
  else
  {
    interRPSPred = false;
    rps->setInterRPSPrediction(false);
  }

  if (interRPSPred)
  {
    UInt bit;
    if(idx == sps->getRPSList()->getNumberOfReferencePictureSets())
    {
      READ_UVLC(code, "delta_idx_minus1" ); // delta index of the Reference Picture Set used for prediction minus 1
    }
    else
    {
      code = 0;
    }
    assert((Int)code <= idx-1); // delta_idx_minus1 shall not be larger than idx-1, otherwise we will predict from a negative row position that does not exist. When idx equals 0 there is no legal value and interRPSPred must be zero. See J0185-r2
    Int rIdx =  idx - 1 - code;
    assert (rIdx <= idx-1 && rIdx >= 0); // Made assert tighter; if rIdx = idx then prediction is done from itself. rIdx must belong to range 0, idx-1, inclusive, see J0185-r2
    TComReferencePictureSet*   rpsRef = sps->getRPSList()->getReferencePictureSet(rIdx);
    Int k = 0, k0 = 0, k1 = 0;
    READ_CODE(1, bit, "delta_rps_sign"); // delta_RPS_sign
    READ_UVLC(code, "abs_delta_rps_minus1");  // absolute delta RPS minus 1
    Int deltaRPS = (1 - 2 * bit) * (code + 1); // delta_RPS
    for(Int j=0 ; j <= rpsRef->getNumberOfPictures(); j++)
    {
      READ_CODE(1, bit, "used_by_curr_pic_flag" ); //first bit is "1" if Idc is 1
      Int refIdc = bit;
      if (refIdc == 0)
      {
        READ_CODE(1, bit, "use_delta_flag" ); //second bit is "1" if Idc is 2, "0" otherwise.
        refIdc = bit<<1; //second bit is "1" if refIdc is 2, "0" if refIdc = 0.
      }
      if (refIdc == 1 || refIdc == 2)
      {
        Int deltaPOC = deltaRPS + ((j < rpsRef->getNumberOfPictures())? rpsRef->getDeltaPOC(j) : 0);
        rps->setDeltaPOC(k, deltaPOC);
        rps->setUsed(k, (refIdc == 1));

        if (deltaPOC < 0)
        {
          k0++;
        }
        else
        {
          k1++;
        }
        k++;
      }
      rps->setRefIdc(j,refIdc);
    }
    rps->setNumRefIdc(rpsRef->getNumberOfPictures()+1);
    rps->setNumberOfPictures(k);
    rps->setNumberOfNegativePictures(k0);
    rps->setNumberOfPositivePictures(k1);
    rps->sortDeltaPOC();
  }
  else
  {
    READ_UVLC(code, "num_negative_pics");           rps->setNumberOfNegativePictures(code);
    READ_UVLC(code, "num_positive_pics");           rps->setNumberOfPositivePictures(code);
    Int prev = 0;
    Int poc;
    for(Int j=0 ; j < rps->getNumberOfNegativePictures(); j++)
    {
      READ_UVLC(code, "delta_poc_s0_minus1");
      poc = prev-code-1;
      prev = poc;
      rps->setDeltaPOC(j,poc);
      READ_FLAG(code, "used_by_curr_pic_s0_flag");  rps->setUsed(j,code);
    }
    prev = 0;
    for(Int j=rps->getNumberOfNegativePictures(); j < rps->getNumberOfNegativePictures()+rps->getNumberOfPositivePictures(); j++)
    {
      READ_UVLC(code, "delta_poc_s1_minus1");
      poc = prev+code+1;
      prev = poc;
      rps->setDeltaPOC(j,poc);
      READ_FLAG(code, "used_by_curr_pic_s1_flag");  rps->setUsed(j,code);
    }
    rps->setNumberOfPictures(rps->getNumberOfNegativePictures()+rps->getNumberOfPositivePictures());
  }
#if PRINT_RPS_INFO
  rps->printDeltaPOC();
#endif
}

Void TDecCavlc::parsePPS(
#if Q0048_CGS_3D_ASYMLUT
  TCom3DAsymLUT * pc3DAsymLUT , Int nLayerID
#endif
  )
{
#if TRACE_DEBUG
  printf("PARSE PPS \n");
#endif
#if ENC_DEC_TRACE
  xTracePPSHeader (pcPPS);
#endif
  UInt  uiCode;

  Int   iCode;

  READ_UVLC( uiCode, "pps_pic_parameter_set_id");
  TComPPS* pcPPS = m_pPPS + uiCode;
  assert(uiCode <= 63);
  pcPPS->setPPSId (uiCode);

  READ_UVLC( uiCode, "pps_seq_parameter_set_id");
  assert(uiCode <= 15);
  pcPPS->setSPSId (uiCode);

  READ_FLAG( uiCode, "dependent_slice_segments_enabled_flag"    );    pcPPS->setDependentSliceSegmentsEnabledFlag   ( uiCode == 1 );
  READ_FLAG( uiCode, "output_flag_present_flag" );                    pcPPS->setOutputFlagPresentFlag( uiCode==1 );

  READ_CODE(3, uiCode, "num_extra_slice_header_bits");                pcPPS->setNumExtraSliceHeaderBits(uiCode);
  READ_FLAG ( uiCode, "sign_data_hiding_flag" ); pcPPS->setSignHideFlag( uiCode );

  READ_FLAG( uiCode,   "cabac_init_present_flag" );            pcPPS->setCabacInitPresentFlag( uiCode ? true : false );

  READ_UVLC(uiCode, "num_ref_idx_l0_default_active_minus1");
  assert(uiCode <= 14);
  pcPPS->setNumRefIdxL0DefaultActive(uiCode+1);

  READ_UVLC(uiCode, "num_ref_idx_l1_default_active_minus1");
  assert(uiCode <= 14);
  pcPPS->setNumRefIdxL1DefaultActive(uiCode+1);

  READ_SVLC(iCode, "init_qp_minus26" );                            pcPPS->setPicInitQPMinus26(iCode);
  READ_FLAG( uiCode, "constrained_intra_pred_flag" );              pcPPS->setConstrainedIntraPred( uiCode ? true : false );
  READ_FLAG( uiCode, "transform_skip_enabled_flag" );
  pcPPS->setUseTransformSkip ( uiCode ? true : false );

  READ_FLAG( uiCode, "cu_qp_delta_enabled_flag" );            pcPPS->setUseDQP( uiCode ? true : false );
  if( pcPPS->getUseDQP() )
  {
    READ_UVLC( uiCode, "diff_cu_qp_delta_depth" );
    pcPPS->setMaxCuDQPDepth( uiCode );
  }
  else
  {
    pcPPS->setMaxCuDQPDepth( 0 );
  }
  READ_SVLC( iCode, "pps_cb_qp_offset");
  pcPPS->setChromaCbQpOffset(iCode);
  assert( pcPPS->getChromaCbQpOffset() >= -12 );
  assert( pcPPS->getChromaCbQpOffset() <=  12 );

  READ_SVLC( iCode, "pps_cr_qp_offset");
  pcPPS->setChromaCrQpOffset(iCode);
  assert( pcPPS->getChromaCrQpOffset() >= -12 );
  assert( pcPPS->getChromaCrQpOffset() <=  12 );

  READ_FLAG( uiCode, "pps_slice_chroma_qp_offsets_present_flag" );
  pcPPS->setSliceChromaQpFlag( uiCode ? true : false );

  READ_FLAG( uiCode, "weighted_pred_flag" );          // Use of Weighting Prediction (P_SLICE)
  pcPPS->setUseWP( uiCode==1 );
  READ_FLAG( uiCode, "weighted_bipred_flag" );         // Use of Bi-Directional Weighting Prediction (B_SLICE)
  pcPPS->setWPBiPred( uiCode==1 );

  READ_FLAG( uiCode, "transquant_bypass_enable_flag");
  pcPPS->setTransquantBypassEnableFlag(uiCode ? true : false);
  READ_FLAG( uiCode, "tiles_enabled_flag"               );    pcPPS->setTilesEnabledFlag            ( uiCode == 1 );
  READ_FLAG( uiCode, "entropy_coding_sync_enabled_flag" );    pcPPS->setEntropyCodingSyncEnabledFlag( uiCode == 1 );

  if( pcPPS->getTilesEnabledFlag() )
  {
    READ_UVLC ( uiCode, "num_tile_columns_minus1" );                pcPPS->setNumTileColumnsMinus1( uiCode );  
    READ_UVLC ( uiCode, "num_tile_rows_minus1" );                   pcPPS->setNumTileRowsMinus1( uiCode );  
    READ_FLAG ( uiCode, "uniform_spacing_flag" );                   pcPPS->setTileUniformSpacingFlag( uiCode == 1 );

    if( !pcPPS->getTileUniformSpacingFlag())
    {
      std::vector<Int> columnWidth(pcPPS->getNumTileColumnsMinus1());
      for(UInt i=0; i< (UInt)pcPPS->getNumTileColumnsMinus1(); i++)
      {
        READ_UVLC( uiCode, "column_width_minus1" );
        columnWidth[i] = uiCode+1;
      }
      pcPPS->setTileColumnWidth(columnWidth);

      std::vector<Int> rowHeight (pcPPS->getTileNumRowsMinus1());
      for(UInt i=0; i< (UInt)pcPPS->getTileNumRowsMinus1(); i++)
      {
        READ_UVLC( uiCode, "row_height_minus1" );
        rowHeight[i] = uiCode + 1;
      }
      pcPPS->setTileRowHeight(rowHeight);
    }

    if(pcPPS->getNumTileColumnsMinus1() !=0 || pcPPS->getTileNumRowsMinus1() !=0)
    {
      READ_FLAG ( uiCode, "loop_filter_across_tiles_enabled_flag" );   pcPPS->setLoopFilterAcrossTilesEnabledFlag( uiCode ? true : false );
    }
  }
  READ_FLAG( uiCode, "loop_filter_across_slices_enabled_flag" );       pcPPS->setLoopFilterAcrossSlicesEnabledFlag( uiCode ? true : false );
  READ_FLAG( uiCode, "deblocking_filter_control_present_flag" );       pcPPS->setDeblockingFilterControlPresentFlag( uiCode ? true : false );
  if(pcPPS->getDeblockingFilterControlPresentFlag())
  {
    READ_FLAG( uiCode, "deblocking_filter_override_enabled_flag" );    pcPPS->setDeblockingFilterOverrideEnabledFlag( uiCode ? true : false );
    READ_FLAG( uiCode, "pps_disable_deblocking_filter_flag" );         pcPPS->setPicDisableDeblockingFilterFlag(uiCode ? true : false );
    if(!pcPPS->getPicDisableDeblockingFilterFlag())
    {
      READ_SVLC ( iCode, "pps_beta_offset_div2" );                     pcPPS->setDeblockingFilterBetaOffsetDiv2( iCode );
      READ_SVLC ( iCode, "pps_tc_offset_div2" );                       pcPPS->setDeblockingFilterTcOffsetDiv2( iCode );
    }
  }
#if !R0042_PROFILE_INDICATION
#if SCALINGLIST_INFERRING
  if( pcPPS->getLayerId() > 0 )
  {
    READ_FLAG( uiCode, "pps_infer_scaling_list_flag" );
    pcPPS->setInferScalingListFlag( uiCode );
  }

  if( pcPPS->getInferScalingListFlag() )
  {
    READ_UVLC( uiCode, "pps_scaling_list_ref_layer_id" ); pcPPS->setScalingListRefLayerId( uiCode );

    // The value of pps_scaling_list_ref_layer_id shall be in the range of 0 to 62, inclusive
    assert( pcPPS->getScalingListRefLayerId() <= 62 );

    pcPPS->setScalingListPresentFlag( false );
  }
  else
  {
#endif
#endif

    READ_FLAG( uiCode, "pps_scaling_list_data_present_flag" );           pcPPS->setScalingListPresentFlag( uiCode ? true : false );

    if(pcPPS->getScalingListPresentFlag ())
    {
      parseScalingList( pcPPS->getScalingList() );
    }
#if !R0042_PROFILE_INDICATION
#if SCALINGLIST_INFERRING
  }
#endif
#endif

  READ_FLAG( uiCode, "lists_modification_present_flag");
  pcPPS->setListsModificationPresentFlag(uiCode);

  READ_UVLC( uiCode, "log2_parallel_merge_level_minus2");
  pcPPS->setLog2ParallelMergeLevelMinus2 (uiCode);

  READ_FLAG( uiCode, "slice_segment_header_extension_present_flag");
  pcPPS->setSliceHeaderExtensionPresentFlag(uiCode);

#if !R0042_PROFILE_INDICATION
  READ_FLAG( uiCode, "pps_extension_flag");
#else
  READ_FLAG( uiCode, "pps_extension_present_flag");
#endif

#if !R0042_PROFILE_INDICATION
#if POC_RESET_INFO_INFERENCE
  pcPPS->setExtensionFlag( uiCode ? true : false );

  if( pcPPS->getExtensionFlag() )
#else
  if (uiCode)
#endif  
  {
#if P0166_MODIFIED_PPS_EXTENSION
    UInt ppsExtensionTypeFlag[8];
    for (UInt i = 0; i < 8; i++)
    {
      READ_FLAG( ppsExtensionTypeFlag[i], "pps_extension_type_flag" );
    }
#if !POC_RESET_IDC
    if (ppsExtensionTypeFlag[1])
    {
#else
    if( ppsExtensionTypeFlag[0] )
    {
      READ_FLAG( uiCode, "poc_reset_info_present_flag" );
      pcPPS->setPocResetInfoPresentFlag(uiCode ? true : false);
#if REF_REGION_OFFSET
      READ_UVLC( uiCode,      "num_scaled_ref_layer_offsets" ); pcPPS->setNumScaledRefLayerOffsets(uiCode);
      for(Int i = 0; i < pcPPS->getNumScaledRefLayerOffsets(); i++)
      {
        READ_CODE( 6, uiCode,  "scaled_ref_layer_id" );  pcPPS->setScaledRefLayerId( i, uiCode );
        READ_FLAG( uiCode, "scaled_ref_layer_offset_present_flag" );   pcPPS->setScaledRefLayerOffsetPresentFlag( i, uiCode );
        if (uiCode)
        {
          Window& scaledWindow = pcPPS->getScaledRefLayerWindow(i);
          READ_SVLC( iCode, "scaled_ref_layer_left_offset" );    scaledWindow.setWindowLeftOffset  (iCode << 1);
          READ_SVLC( iCode, "scaled_ref_layer_top_offset" );     scaledWindow.setWindowTopOffset   (iCode << 1);
          READ_SVLC( iCode, "scaled_ref_layer_right_offset" );   scaledWindow.setWindowRightOffset (iCode << 1);
          READ_SVLC( iCode, "scaled_ref_layer_bottom_offset" );  scaledWindow.setWindowBottomOffset(iCode << 1);
#if P0312_VERT_PHASE_ADJ
          READ_FLAG( uiCode, "vert_phase_position_enable_flag" ); scaledWindow.setVertPhasePositionEnableFlag(uiCode);  pcPPS->setVertPhasePositionEnableFlag( pcPPS->getScaledRefLayerId(i), uiCode);
#endif
        }
        READ_FLAG( uiCode, "ref_region_offset_present_flag" );   pcPPS->setRefRegionOffsetPresentFlag( i, uiCode );
        if (uiCode)
        {
          Window& refWindow = pcPPS->getRefLayerWindow(i);
          READ_SVLC( iCode, "ref_region_left_offset" );    refWindow.setWindowLeftOffset  (iCode << 1);
          READ_SVLC( iCode, "ref_region_top_offset" );     refWindow.setWindowTopOffset   (iCode << 1);
          READ_SVLC( iCode, "ref_region_right_offset" );   refWindow.setWindowRightOffset (iCode << 1);
          READ_SVLC( iCode, "ref_region_bottom_offset" );  refWindow.setWindowBottomOffset(iCode << 1);
        }
#if R0209_GENERIC_PHASE
        READ_FLAG( uiCode, "resample_phase_set_present_flag" );   pcPPS->setResamplePhaseSetPresentFlag( i, uiCode );
        if (uiCode)
        {
          READ_UVLC( uiCode, "phase_hor_luma" );    pcPPS->setPhaseHorLuma ( i, uiCode );
          READ_UVLC( uiCode, "phase_ver_luma" );    pcPPS->setPhaseVerLuma ( i, uiCode );
          READ_UVLC( uiCode, "phase_hor_chroma_plus8" );  pcPPS->setPhaseHorChroma (i, uiCode - 8);
          READ_UVLC( uiCode, "phase_ver_chroma_plus8" );  pcPPS->setPhaseVerChroma (i, uiCode - 8);
        }
#endif
      }
#else
#if MOVE_SCALED_OFFSET_TO_PPS
      READ_UVLC( uiCode,      "num_scaled_ref_layer_offsets" ); pcPPS->setNumScaledRefLayerOffsets(uiCode);
      for(Int i = 0; i < pcPPS->getNumScaledRefLayerOffsets(); i++)
      {
        Window& scaledWindow = pcPPS->getScaledRefLayerWindow(i);
#if O0098_SCALED_REF_LAYER_ID
        READ_CODE( 6,  uiCode,  "scaled_ref_layer_id" );       pcPPS->setScaledRefLayerId( i, uiCode );
#endif
        READ_SVLC( iCode, "scaled_ref_layer_left_offset" );    scaledWindow.setWindowLeftOffset  (iCode << 1);
        READ_SVLC( iCode, "scaled_ref_layer_top_offset" );     scaledWindow.setWindowTopOffset   (iCode << 1);
        READ_SVLC( iCode, "scaled_ref_layer_right_offset" );   scaledWindow.setWindowRightOffset (iCode << 1);
        READ_SVLC( iCode, "scaled_ref_layer_bottom_offset" );  scaledWindow.setWindowBottomOffset(iCode << 1);
#if P0312_VERT_PHASE_ADJ
        READ_FLAG( uiCode, "vert_phase_position_enable_flag" ); scaledWindow.setVertPhasePositionEnableFlag(uiCode);  pcPPS->setVertPhasePositionEnableFlag( pcPPS->getScaledRefLayerId(i), uiCode);
#endif
      }
#endif
#endif
#if Q0048_CGS_3D_ASYMLUT
      READ_FLAG( uiCode , "colour_mapping_enabled_flag" ); 
      pcPPS->setCGSFlag( uiCode );
      if( pcPPS->getCGSFlag() )
      {
        xParse3DAsymLUT( pc3DAsymLUT );
        pcPPS->setCGSOutputBitDepthY( pc3DAsymLUT->getOutputBitDepthY() );
        pcPPS->setCGSOutputBitDepthC( pc3DAsymLUT->getOutputBitDepthC() );
      }
#endif
#endif
    }
#if POC_RESET_INFO_INFERENCE
    else  // Extension type 0 absent
    {
      pcPPS->setPocResetInfoPresentFlag( false );
    }
#endif
    if (ppsExtensionTypeFlag[7])
    {
#endif

      while ( xMoreRbspData() )
      {
        READ_FLAG( uiCode, "pps_extension_data_flag");
      }
#if P0166_MODIFIED_PPS_EXTENSION
    }
#endif
  }
#if POC_RESET_INFO_INFERENCE
  if( !pcPPS->getExtensionFlag() )
  {
    pcPPS->setPocResetInfoPresentFlag( false );
  }
#endif
#else
  pcPPS->setExtensionFlag( uiCode ? true : false );
  if( pcPPS->getExtensionFlag() )
  {
    READ_FLAG( uiCode, "pps_range_extension_flag" );
    assert(uiCode == 0);
    READ_FLAG( uiCode, "pps_multilayer_extension_flag" );
    assert(uiCode == 1);
    READ_CODE(6, uiCode, "pps_extension_6bits");
    assert(uiCode == 0);

    READ_FLAG( uiCode, "poc_reset_info_present_flag" );
    pcPPS->setPocResetInfoPresentFlag(uiCode ? true : false);

#if SCALINGLIST_INFERRING
    READ_FLAG( uiCode, "pps_infer_scaling_list_flag" );
    pcPPS->setInferScalingListFlag( uiCode );

    if( pcPPS->getInferScalingListFlag() )
    {
      READ_UVLC( uiCode, "pps_scaling_list_ref_layer_id" ); 
      pcPPS->setScalingListRefLayerId( uiCode );
      // The value of pps_scaling_list_ref_layer_id shall be in the range of 0 to 62, inclusive
      assert( pcPPS->getScalingListRefLayerId() <= 62 );
      pcPPS->setScalingListPresentFlag( false );
    }
#endif

#if REF_REGION_OFFSET
    READ_UVLC( uiCode,      "num_ref_loc_offsets" ); pcPPS->setNumScaledRefLayerOffsets(uiCode);
    for(Int i = 0; i < (Int)pcPPS->getNumScaledRefLayerOffsets(); i++)
    {
      READ_CODE( 6, uiCode,  "ref_loc_offset_layer_id" );  pcPPS->setScaledRefLayerId( i, uiCode );
      READ_FLAG( uiCode, "scaled_ref_layer_offset_present_flag" );   pcPPS->setScaledRefLayerOffsetPresentFlag( i, uiCode );
      if (uiCode)
      {
        Window& scaledWindow = pcPPS->getScaledRefLayerWindow(i);
        READ_SVLC( iCode, "scaled_ref_layer_left_offset" );    scaledWindow.setWindowLeftOffset  (iCode << 1);
        READ_SVLC( iCode, "scaled_ref_layer_top_offset" );     scaledWindow.setWindowTopOffset   (iCode << 1);
        READ_SVLC( iCode, "scaled_ref_layer_right_offset" );   scaledWindow.setWindowRightOffset (iCode << 1);
        READ_SVLC( iCode, "scaled_ref_layer_bottom_offset" );  scaledWindow.setWindowBottomOffset(iCode << 1);
#if P0312_VERT_PHASE_ADJ
        READ_FLAG( uiCode, "vert_phase_position_enable_flag" ); scaledWindow.setVertPhasePositionEnableFlag(uiCode);  pcPPS->setVertPhasePositionEnableFlag( pcPPS->getScaledRefLayerId(i), uiCode);
#endif
      }
      READ_FLAG( uiCode, "ref_region_offset_present_flag" );   pcPPS->setRefRegionOffsetPresentFlag( i, uiCode );
      if (uiCode)
      {
        Window& refWindow = pcPPS->getRefLayerWindow(i);
        READ_SVLC( iCode, "ref_region_left_offset" );    refWindow.setWindowLeftOffset  (iCode << 1);
        READ_SVLC( iCode, "ref_region_top_offset" );     refWindow.setWindowTopOffset   (iCode << 1);
        READ_SVLC( iCode, "ref_region_right_offset" );   refWindow.setWindowRightOffset (iCode << 1);
        READ_SVLC( iCode, "ref_region_bottom_offset" );  refWindow.setWindowBottomOffset(iCode << 1);
      }
#if R0209_GENERIC_PHASE
      READ_FLAG( uiCode, "resample_phase_set_present_flag" );   pcPPS->setResamplePhaseSetPresentFlag( i, uiCode );
      if (uiCode)
      {
        READ_UVLC( uiCode, "phase_hor_luma" );    pcPPS->setPhaseHorLuma ( i, uiCode );
        READ_UVLC( uiCode, "phase_ver_luma" );    pcPPS->setPhaseVerLuma ( i, uiCode );
        READ_UVLC( uiCode, "phase_hor_chroma_plus8" );  pcPPS->setPhaseHorChroma (i, uiCode - 8);
        READ_UVLC( uiCode, "phase_ver_chroma_plus8" );  pcPPS->setPhaseVerChroma (i, uiCode - 8);
      }
#endif
    }
#else
#if MOVE_SCALED_OFFSET_TO_PPS
      READ_UVLC( uiCode,      "num_scaled_ref_layer_offsets" ); pcPPS->setNumScaledRefLayerOffsets(uiCode);
      for(Int i = 0; i < pcPPS->getNumScaledRefLayerOffsets(); i++)
      {
        Window& scaledWindow = pcPPS->getScaledRefLayerWindow(i);
#if O0098_SCALED_REF_LAYER_ID
        READ_CODE( 6,  uiCode,  "scaled_ref_layer_id" );       pcPPS->setScaledRefLayerId( i, uiCode );
#endif
        READ_SVLC( iCode, "scaled_ref_layer_left_offset" );    scaledWindow.setWindowLeftOffset  (iCode << 1);
        READ_SVLC( iCode, "scaled_ref_layer_top_offset" );     scaledWindow.setWindowTopOffset   (iCode << 1);
        READ_SVLC( iCode, "scaled_ref_layer_right_offset" );   scaledWindow.setWindowRightOffset (iCode << 1);
        READ_SVLC( iCode, "scaled_ref_layer_bottom_offset" );  scaledWindow.setWindowBottomOffset(iCode << 1);
#if P0312_VERT_PHASE_ADJ
        READ_FLAG( uiCode, "vert_phase_position_enable_flag" ); scaledWindow.setVertPhasePositionEnableFlag(uiCode);  pcPPS->setVertPhasePositionEnableFlag( pcPPS->getScaledRefLayerId(i), uiCode);
#endif
      }
#endif
#endif
#if Q0048_CGS_3D_ASYMLUT
      READ_FLAG( uiCode , "colour_mapping_enabled_flag" ); 
      pcPPS->setCGSFlag( uiCode );
      if( pcPPS->getCGSFlag() )
      {
        xParse3DAsymLUT( pc3DAsymLUT );
        pcPPS->setCGSOutputBitDepthY( pc3DAsymLUT->getOutputBitDepthY() );
        pcPPS->setCGSOutputBitDepthC( pc3DAsymLUT->getOutputBitDepthC() );
      }
#endif
  }
#endif

}

Void  TDecCavlc::parseVUI(TComVUI* pcVUI, TComSPS *pcSPS)
{
#if ENC_DEC_TRACE
  fprintf( g_hTrace, "----------- vui_parameters -----------\n");
#endif
  UInt  uiCode;

  READ_FLAG(     uiCode, "aspect_ratio_info_present_flag");           pcVUI->setAspectRatioInfoPresentFlag(uiCode);
  if (pcVUI->getAspectRatioInfoPresentFlag())
  {
    READ_CODE(8, uiCode, "aspect_ratio_idc");                         pcVUI->setAspectRatioIdc(uiCode);
    if (pcVUI->getAspectRatioIdc() == 255)
    {
      READ_CODE(16, uiCode, "sar_width");                             pcVUI->setSarWidth(uiCode);
      READ_CODE(16, uiCode, "sar_height");                            pcVUI->setSarHeight(uiCode);
    }
  }

  READ_FLAG(     uiCode, "overscan_info_present_flag");               pcVUI->setOverscanInfoPresentFlag(uiCode);
  if (pcVUI->getOverscanInfoPresentFlag())
  {
    READ_FLAG(   uiCode, "overscan_appropriate_flag");                pcVUI->setOverscanAppropriateFlag(uiCode);
  }

  READ_FLAG(     uiCode, "video_signal_type_present_flag");           pcVUI->setVideoSignalTypePresentFlag(uiCode);
  if (pcVUI->getVideoSignalTypePresentFlag())
  {
    READ_CODE(3, uiCode, "video_format");                             pcVUI->setVideoFormat(uiCode);
    READ_FLAG(   uiCode, "video_full_range_flag");                    pcVUI->setVideoFullRangeFlag(uiCode);
    READ_FLAG(   uiCode, "colour_description_present_flag");          pcVUI->setColourDescriptionPresentFlag(uiCode);
    if (pcVUI->getColourDescriptionPresentFlag())
    {
      READ_CODE(8, uiCode, "colour_primaries");                       pcVUI->setColourPrimaries(uiCode);
      READ_CODE(8, uiCode, "transfer_characteristics");               pcVUI->setTransferCharacteristics(uiCode);
      READ_CODE(8, uiCode, "matrix_coefficients");                    pcVUI->setMatrixCoefficients(uiCode);
    }
  }

  READ_FLAG(     uiCode, "chroma_loc_info_present_flag");             pcVUI->setChromaLocInfoPresentFlag(uiCode);
  if (pcVUI->getChromaLocInfoPresentFlag())
  {
    READ_UVLC(   uiCode, "chroma_sample_loc_type_top_field" );        pcVUI->setChromaSampleLocTypeTopField(uiCode);
    READ_UVLC(   uiCode, "chroma_sample_loc_type_bottom_field" );     pcVUI->setChromaSampleLocTypeBottomField(uiCode);
  }

  READ_FLAG(     uiCode, "neutral_chroma_indication_flag");           pcVUI->setNeutralChromaIndicationFlag(uiCode);

  READ_FLAG(     uiCode, "field_seq_flag");                           pcVUI->setFieldSeqFlag(uiCode);

  READ_FLAG(uiCode, "frame_field_info_present_flag");                 pcVUI->setFrameFieldInfoPresentFlag(uiCode);

  READ_FLAG(     uiCode, "default_display_window_flag");
  if (uiCode != 0)
  {
    Window &defDisp = pcVUI->getDefaultDisplayWindow();
    READ_UVLC(   uiCode, "def_disp_win_left_offset" );                defDisp.setWindowLeftOffset  ( uiCode * TComSPS::getWinUnitX( pcSPS->getChromaFormatIdc()) );
    READ_UVLC(   uiCode, "def_disp_win_right_offset" );               defDisp.setWindowRightOffset ( uiCode * TComSPS::getWinUnitX( pcSPS->getChromaFormatIdc()) );
    READ_UVLC(   uiCode, "def_disp_win_top_offset" );                 defDisp.setWindowTopOffset   ( uiCode * TComSPS::getWinUnitY( pcSPS->getChromaFormatIdc()) );
    READ_UVLC(   uiCode, "def_disp_win_bottom_offset" );              defDisp.setWindowBottomOffset( uiCode * TComSPS::getWinUnitY( pcSPS->getChromaFormatIdc()) );
  }
  TimingInfo *timingInfo = pcVUI->getTimingInfo();
  READ_FLAG(       uiCode, "vui_timing_info_present_flag");         timingInfo->setTimingInfoPresentFlag      (uiCode ? true : false);
#if SVC_EXTENSION
  if( pcSPS->getLayerId() > 0 )
  {
    assert( timingInfo->getTimingInfoPresentFlag() == false );
  }
#endif
  if(timingInfo->getTimingInfoPresentFlag())
  {
    READ_CODE( 32, uiCode, "vui_num_units_in_tick");                timingInfo->setNumUnitsInTick             (uiCode);
    READ_CODE( 32, uiCode, "vui_time_scale");                       timingInfo->setTimeScale                  (uiCode);
    READ_FLAG(     uiCode, "vui_poc_proportional_to_timing_flag");  timingInfo->setPocProportionalToTimingFlag(uiCode ? true : false);
    if(timingInfo->getPocProportionalToTimingFlag())
    {
      READ_UVLC(   uiCode, "vui_num_ticks_poc_diff_one_minus1");    timingInfo->setNumTicksPocDiffOneMinus1   (uiCode);
    }
    READ_FLAG(     uiCode, "hrd_parameters_present_flag");              pcVUI->setHrdParametersPresentFlag(uiCode);
    if( pcVUI->getHrdParametersPresentFlag() )
    {
      parseHrdParameters( pcVUI->getHrdParameters(), 1, pcSPS->getMaxTLayers() - 1 );
    }
  }
  READ_FLAG(     uiCode, "bitstream_restriction_flag");               pcVUI->setBitstreamRestrictionFlag(uiCode);
  if (pcVUI->getBitstreamRestrictionFlag())
  {
    READ_FLAG(   uiCode, "tiles_fixed_structure_flag");               pcVUI->setTilesFixedStructureFlag(uiCode);
    READ_FLAG(   uiCode, "motion_vectors_over_pic_boundaries_flag");  pcVUI->setMotionVectorsOverPicBoundariesFlag(uiCode);
    READ_FLAG(   uiCode, "restricted_ref_pic_lists_flag");            pcVUI->setRestrictedRefPicListsFlag(uiCode);
    READ_UVLC( uiCode, "min_spatial_segmentation_idc");            pcVUI->setMinSpatialSegmentationIdc(uiCode);
    assert(uiCode < 4096);
    READ_UVLC(   uiCode, "max_bytes_per_pic_denom" );                 pcVUI->setMaxBytesPerPicDenom(uiCode);
    READ_UVLC(   uiCode, "max_bits_per_mincu_denom" );                pcVUI->setMaxBitsPerMinCuDenom(uiCode);
    READ_UVLC(   uiCode, "log2_max_mv_length_horizontal" );           pcVUI->setLog2MaxMvLengthHorizontal(uiCode);
    READ_UVLC(   uiCode, "log2_max_mv_length_vertical" );             pcVUI->setLog2MaxMvLengthVertical(uiCode);
  }
}

Void TDecCavlc::parseHrdParameters(TComHRD *hrd, Bool commonInfPresentFlag, UInt maxNumSubLayersMinus1)
{
  UInt  uiCode;
  if( commonInfPresentFlag )
  {
    READ_FLAG( uiCode, "nal_hrd_parameters_present_flag" );           hrd->setNalHrdParametersPresentFlag( uiCode == 1 ? true : false );
    READ_FLAG( uiCode, "vcl_hrd_parameters_present_flag" );           hrd->setVclHrdParametersPresentFlag( uiCode == 1 ? true : false );
    if( hrd->getNalHrdParametersPresentFlag() || hrd->getVclHrdParametersPresentFlag() )
    {
      READ_FLAG( uiCode, "sub_pic_cpb_params_present_flag" );         hrd->setSubPicCpbParamsPresentFlag( uiCode == 1 ? true : false );
      if( hrd->getSubPicCpbParamsPresentFlag() )
      {
        READ_CODE( 8, uiCode, "tick_divisor_minus2" );                hrd->setTickDivisorMinus2( uiCode );
        READ_CODE( 5, uiCode, "du_cpb_removal_delay_length_minus1" ); hrd->setDuCpbRemovalDelayLengthMinus1( uiCode );
        READ_FLAG( uiCode, "sub_pic_cpb_params_in_pic_timing_sei_flag" ); hrd->setSubPicCpbParamsInPicTimingSEIFlag( uiCode == 1 ? true : false );
        READ_CODE( 5, uiCode, "dpb_output_delay_du_length_minus1"  ); hrd->setDpbOutputDelayDuLengthMinus1( uiCode );
      }
      READ_CODE( 4, uiCode, "bit_rate_scale" );                       hrd->setBitRateScale( uiCode );
      READ_CODE( 4, uiCode, "cpb_size_scale" );                       hrd->setCpbSizeScale( uiCode );
      if( hrd->getSubPicCpbParamsPresentFlag() )
      {
        READ_CODE( 4, uiCode, "cpb_size_du_scale" );                  hrd->setDuCpbSizeScale( uiCode );
      }
      READ_CODE( 5, uiCode, "initial_cpb_removal_delay_length_minus1" ); hrd->setInitialCpbRemovalDelayLengthMinus1( uiCode );
      READ_CODE( 5, uiCode, "au_cpb_removal_delay_length_minus1" );      hrd->setCpbRemovalDelayLengthMinus1( uiCode );
      READ_CODE( 5, uiCode, "dpb_output_delay_length_minus1" );       hrd->setDpbOutputDelayLengthMinus1( uiCode );
    }
#if VPS_VUI_BSP_HRD_PARAMS
    else
    {
      hrd->setInitialCpbRemovalDelayLengthMinus1( 23 );
      // Add inferred values for other syntax elements here.
    }
#endif
  }
  Int i, j, nalOrVcl;
  for( i = 0; i <= (Int)maxNumSubLayersMinus1; i ++ )
  {
    READ_FLAG( uiCode, "fixed_pic_rate_general_flag" );                     hrd->setFixedPicRateFlag( i, uiCode == 1 ? true : false  );
    if( !hrd->getFixedPicRateFlag( i ) )
    {
      READ_FLAG( uiCode, "fixed_pic_rate_within_cvs_flag" );                hrd->setFixedPicRateWithinCvsFlag( i, uiCode == 1 ? true : false  );
    }
    else
    {
      hrd->setFixedPicRateWithinCvsFlag( i, true );
    }
    hrd->setLowDelayHrdFlag( i, 0 ); // Infered to be 0 when not present
    hrd->setCpbCntMinus1   ( i, 0 ); // Infered to be 0 when not present
    if( hrd->getFixedPicRateWithinCvsFlag( i ) )
    {
      READ_UVLC( uiCode, "elemental_duration_in_tc_minus1" );             hrd->setPicDurationInTcMinus1( i, uiCode );
    }
    else
    {
      READ_FLAG( uiCode, "low_delay_hrd_flag" );                      hrd->setLowDelayHrdFlag( i, uiCode == 1 ? true : false  );
    }
    if (!hrd->getLowDelayHrdFlag( i ))
    {
      READ_UVLC( uiCode, "cpb_cnt_minus1" );                          hrd->setCpbCntMinus1( i, uiCode );
    }
    for( nalOrVcl = 0; nalOrVcl < 2; nalOrVcl ++ )
    {
      if( ( ( nalOrVcl == 0 ) && ( hrd->getNalHrdParametersPresentFlag() ) ) ||
        ( ( nalOrVcl == 1 ) && ( hrd->getVclHrdParametersPresentFlag() ) ) )
      {
        for( j = 0; j <= ( hrd->getCpbCntMinus1( i ) ); j ++ )
        {
          READ_UVLC( uiCode, "bit_rate_value_minus1" );             hrd->setBitRateValueMinus1( i, j, nalOrVcl, uiCode );
          READ_UVLC( uiCode, "cpb_size_value_minus1" );             hrd->setCpbSizeValueMinus1( i, j, nalOrVcl, uiCode );
          if( hrd->getSubPicCpbParamsPresentFlag() )
          {
            READ_UVLC( uiCode, "cpb_size_du_value_minus1" );       hrd->setDuCpbSizeValueMinus1( i, j, nalOrVcl, uiCode );
            READ_UVLC( uiCode, "bit_rate_du_value_minus1" );       hrd->setDuBitRateValueMinus1( i, j, nalOrVcl, uiCode );
          }
          READ_FLAG( uiCode, "cbr_flag" );                          hrd->setCbrFlag( i, j, nalOrVcl, uiCode == 1 ? true : false  );
        }
      }
    }
  }
}

#if SVC_EXTENSION && !SPS_DPB_PARAMS
Void TDecCavlc::parseSPS( TComSPS* pcSPS, ParameterSetManagerDecoder *parameterSetManager)
#else
Void TDecCavlc::parseSPS(TComSPS* pcSPS)
#endif
{
#if TRACE_DEBUG
  printf("PARSE SPS \n");
#endif
#if ENC_DEC_TRACE
  xTraceSPSHeader (pcSPS);
#endif

#if R0042_PROFILE_INDICATION
  UInt uiTmp = 0;
  Bool bMultiLayerExtSpsFlag;
#endif
  UInt  uiCode;
  READ_CODE( 4,  uiCode, "sps_video_parameter_set_id");          pcSPS->setVPSId        ( uiCode );
#if SVC_EXTENSION
  if(pcSPS->getLayerId() == 0)
  {
#endif
    READ_CODE( 3,  uiCode, "sps_max_sub_layers_minus1" );          pcSPS->setMaxTLayers   ( uiCode+1 );
    assert(uiCode <= 6);
#if SVC_EXTENSION
  }
#if R0042_PROFILE_INDICATION
  else
  {
    READ_CODE( 3,  uiCode, "sps_ext_or_max_sub_layers_minus1" );     uiTmp = uiCode;
  }
#endif
#if !SPS_DPB_PARAMS
  if(pcSPS->getLayerId() != 0)
  {
    pcSPS->setMaxTLayers           ( parameterSetManager->getPrefetchedVPS(pcSPS->getVPSId())->getMaxTLayers()          );
  }
#endif
#endif

#if SVC_EXTENSION
#if R0042_PROFILE_INDICATION
  bMultiLayerExtSpsFlag = ( pcSPS->getLayerId() != 0 && uiTmp == 7 );
#endif
#endif

#if SVC_EXTENSION
#if !R0042_PROFILE_INDICATION
  if(pcSPS->getLayerId() == 0)
#else
  if(!bMultiLayerExtSpsFlag)
#endif
  {
#endif
    READ_FLAG( uiCode, "sps_temporal_id_nesting_flag" );               pcSPS->setTemporalIdNestingFlag ( uiCode > 0 ? true : false );
#if SVC_EXTENSION
  }
#if !SPS_DPB_PARAMS
  else
  {
    pcSPS->setTemporalIdNestingFlag( parameterSetManager->getPrefetchedVPS(pcSPS->getVPSId())->getTemporalNestingFlag() );
  }
#endif
#endif

#if !Q0177_SPS_TEMP_NESTING_FIX   //This part is not needed anymore as it is already covered by implementation in TDecTop::xActivateParameterSets()
  if ( pcSPS->getMaxTLayers() == 1 )
  {
    // sps_temporal_id_nesting_flag must be 1 when sps_max_sub_layers_minus1 is 0
#if SVC_EXTENSION
#if !SPS_DPB_PARAMS
    assert( pcSPS->getTemporalIdNestingFlag() == true );
#endif
#else
    assert( uiCode == 1 );
#endif
  }
#endif

#ifdef SPS_PTL_FIX
#if !R0042_PROFILE_INDICATION
  if ( pcSPS->getLayerId() == 0)
#else
  if(!bMultiLayerExtSpsFlag)
#endif
  {
    parsePTL(pcSPS->getPTL(), 1, pcSPS->getMaxTLayers() - 1);
  }
#else
  parsePTL(pcSPS->getPTL(), 1, pcSPS->getMaxTLayers() - 1);
#endif

  READ_UVLC(     uiCode, "sps_seq_parameter_set_id" );           pcSPS->setSPSId( uiCode );
  assert(uiCode <= 15);

#if REPN_FORMAT_IN_VPS
#if !R0042_PROFILE_INDICATION
  if( pcSPS->getLayerId() > 0 )
#else
  if( bMultiLayerExtSpsFlag)
#endif
  {
    READ_FLAG( uiCode, "update_rep_format_flag" );
    pcSPS->setUpdateRepFormatFlag( uiCode ? true : false );
#if R0042_PROFILE_INDICATION    
    if( bMultiLayerExtSpsFlag && uiCode)
    {
      READ_CODE(8, uiCode, "sps_rep_format_idx");
      pcSPS->setUpdateRepFormatIndex(uiCode);
    }
#endif
  }
  else
  {
#if REP_FORMAT_FIX
    pcSPS->setUpdateRepFormatFlag( false );
#else
    pcSPS->setUpdateRepFormatFlag( true );
#endif
  }

#if R0042_PROFILE_INDICATION  
  if( !bMultiLayerExtSpsFlag )
  {
#else
#if O0096_REP_FORMAT_INDEX
  if( pcSPS->getLayerId() == 0 )
#else
  if( pcSPS->getLayerId() == 0 || pcSPS->getUpdateRepFormatFlag() )
#endif
#endif
  {
#endif
#if AUXILIARY_PICTURES
    READ_UVLC(     uiCode, "chroma_format_idc" );                  pcSPS->setChromaFormatIdc( ChromaFormat(uiCode) );
#else
    READ_UVLC(     uiCode, "chroma_format_idc" );                  pcSPS->setChromaFormatIdc( uiCode );
#endif
    assert(uiCode <= 3);
    // in the first version we only support chroma_format_idc equal to 1 (4:2:0), so separate_colour_plane_flag cannot appear in the bitstream
    assert (uiCode == 1);
    if( uiCode == 3 )
    {
      READ_FLAG(     uiCode, "separate_colour_plane_flag");        assert(uiCode == 0);
    }

    READ_UVLC (    uiCode, "pic_width_in_luma_samples" );          pcSPS->setPicWidthInLumaSamples ( uiCode    );
    READ_UVLC (    uiCode, "pic_height_in_luma_samples" );         pcSPS->setPicHeightInLumaSamples( uiCode    );
#if REPN_FORMAT_IN_VPS
  }
#if O0096_REP_FORMAT_INDEX
#if !R0042_PROFILE_INDICATION
  else if ( pcSPS->getUpdateRepFormatFlag() )
  {
    READ_CODE(8, uiCode, "update_rep_format_index");
    pcSPS->setUpdateRepFormatIndex(uiCode);
  }
#endif
#endif
#endif

#if R0156_CONF_WINDOW_IN_REP_FORMAT
#if REPN_FORMAT_IN_VPS
#if !R0042_PROFILE_INDICATION  
#if O0096_REP_FORMAT_INDEX
  if( pcSPS->getLayerId() == 0 )
#else
  if(  pcSPS->getLayerId() == 0 || pcSPS->getUpdateRepFormatFlag() )
#endif
#endif
  {
#endif
#endif
    READ_FLAG(     uiCode, "conformance_window_flag");
    if (uiCode != 0)
    {
      Window &conf = pcSPS->getConformanceWindow();
#if REPN_FORMAT_IN_VPS
      READ_UVLC(   uiCode, "conf_win_left_offset" );               conf.setWindowLeftOffset  ( uiCode );
      READ_UVLC(   uiCode, "conf_win_right_offset" );              conf.setWindowRightOffset ( uiCode );
      READ_UVLC(   uiCode, "conf_win_top_offset" );                conf.setWindowTopOffset   ( uiCode );
      READ_UVLC(   uiCode, "conf_win_bottom_offset" );             conf.setWindowBottomOffset( uiCode );
#else
      READ_UVLC(   uiCode, "conf_win_left_offset" );               conf.setWindowLeftOffset  ( uiCode * TComSPS::getWinUnitX( pcSPS->getChromaFormatIdc() ) );
      READ_UVLC(   uiCode, "conf_win_right_offset" );              conf.setWindowRightOffset ( uiCode * TComSPS::getWinUnitX( pcSPS->getChromaFormatIdc() ) );
      READ_UVLC(   uiCode, "conf_win_top_offset" );                conf.setWindowTopOffset   ( uiCode * TComSPS::getWinUnitY( pcSPS->getChromaFormatIdc() ) );
      READ_UVLC(   uiCode, "conf_win_bottom_offset" );             conf.setWindowBottomOffset( uiCode * TComSPS::getWinUnitY( pcSPS->getChromaFormatIdc() ) );
#endif
    }
#if R0156_CONF_WINDOW_IN_REP_FORMAT
#if REPN_FORMAT_IN_VPS
  }
#endif
#endif

#if REPN_FORMAT_IN_VPS
#if !R0042_PROFILE_INDICATION  
#if O0096_REP_FORMAT_INDEX
  if( pcSPS->getLayerId() == 0 )
#else
  if(  pcSPS->getLayerId() == 0 || pcSPS->getUpdateRepFormatFlag() )
#endif
#endif
  {
#endif
    READ_UVLC(     uiCode, "bit_depth_luma_minus8" );

    assert(uiCode <= 6);
    pcSPS->setBitDepthY( uiCode + 8 );
    pcSPS->setQpBDOffsetY( (Int) (6*uiCode) );

    READ_UVLC( uiCode,    "bit_depth_chroma_minus8" );
    assert(uiCode <= 6);
    pcSPS->setBitDepthC( uiCode + 8 );
    pcSPS->setQpBDOffsetC( (Int) (6*uiCode) );
#if REPN_FORMAT_IN_VPS
  }
#endif
#if R0042_PROFILE_INDICATION  
  }
#endif

  READ_UVLC( uiCode,    "log2_max_pic_order_cnt_lsb_minus4" );   pcSPS->setBitsForPOC( 4 + uiCode );
  assert(uiCode <= 12);

#if SPS_DPB_PARAMS
#if !R0042_PROFILE_INDICATION
  if( pcSPS->getLayerId() == 0 )  
  {
#else
  if( !bMultiLayerExtSpsFlag )  
  {
#endif
#endif
    UInt subLayerOrderingInfoPresentFlag;
    READ_FLAG(subLayerOrderingInfoPresentFlag, "sps_sub_layer_ordering_info_present_flag");

    for(UInt i=0; i <= pcSPS->getMaxTLayers()-1; i++)
    {
      READ_UVLC ( uiCode, "sps_max_dec_pic_buffering_minus1[i]");
      pcSPS->setMaxDecPicBuffering( uiCode + 1, i);
      READ_UVLC ( uiCode, "sps_num_reorder_pics[i]" );
      pcSPS->setNumReorderPics(uiCode, i);
      READ_UVLC ( uiCode, "sps_max_latency_increase_plus1[i]");
      pcSPS->setMaxLatencyIncrease( uiCode, i );

      if (!subLayerOrderingInfoPresentFlag)
      {
        for (i++; i <= pcSPS->getMaxTLayers()-1; i++)
        {
          pcSPS->setMaxDecPicBuffering(pcSPS->getMaxDecPicBuffering(0), i);
          pcSPS->setNumReorderPics(pcSPS->getNumReorderPics(0), i);
          pcSPS->setMaxLatencyIncrease(pcSPS->getMaxLatencyIncrease(0), i);
        }
        break;
      }
    }
#if SPS_DPB_PARAMS
  }
#endif
  READ_UVLC( uiCode, "log2_min_coding_block_size_minus3" );
  Int log2MinCUSize = uiCode + 3;
  pcSPS->setLog2MinCodingBlockSize(log2MinCUSize);
  READ_UVLC( uiCode, "log2_diff_max_min_coding_block_size" );
  pcSPS->setLog2DiffMaxMinCodingBlockSize(uiCode);

  if (pcSPS->getPTL()->getGeneralPTL()->getLevelIdc() >= Level::LEVEL5)
  {
    assert(log2MinCUSize + pcSPS->getLog2DiffMaxMinCodingBlockSize() >= 5);
  }

  Int maxCUDepthDelta = uiCode;
  pcSPS->setMaxCUWidth  ( 1<<(log2MinCUSize + maxCUDepthDelta) );
  pcSPS->setMaxCUHeight ( 1<<(log2MinCUSize + maxCUDepthDelta) );
  READ_UVLC( uiCode, "log2_min_transform_block_size_minus2" );   pcSPS->setQuadtreeTULog2MinSize( uiCode + 2 );

  READ_UVLC( uiCode, "log2_diff_max_min_transform_block_size" ); pcSPS->setQuadtreeTULog2MaxSize( uiCode + pcSPS->getQuadtreeTULog2MinSize() );
  pcSPS->setMaxTrSize( 1<<(uiCode + pcSPS->getQuadtreeTULog2MinSize()) );

  READ_UVLC( uiCode, "max_transform_hierarchy_depth_inter" );    pcSPS->setQuadtreeTUMaxDepthInter( uiCode+1 );
  READ_UVLC( uiCode, "max_transform_hierarchy_depth_intra" );    pcSPS->setQuadtreeTUMaxDepthIntra( uiCode+1 );

  Int addCuDepth = max (0, log2MinCUSize - (Int)pcSPS->getQuadtreeTULog2MinSize() );
  pcSPS->setMaxCUDepth( maxCUDepthDelta + addCuDepth );
  READ_FLAG( uiCode, "scaling_list_enabled_flag" );                 pcSPS->setScalingListFlag ( uiCode );

  if(pcSPS->getScalingListFlag())
  {
#if SCALINGLIST_INFERRING
#if !R0042_PROFILE_INDICATION
    if( pcSPS->getLayerId() > 0 )
#else
    if( bMultiLayerExtSpsFlag )
#endif
    {
      READ_FLAG( uiCode, "sps_infer_scaling_list_flag" ); pcSPS->setInferScalingListFlag( uiCode );
    }

    if( pcSPS->getInferScalingListFlag() )
    {
      READ_UVLC( uiCode, "sps_scaling_list_ref_layer_id" ); pcSPS->setScalingListRefLayerId( uiCode );

      // The value of pps_scaling_list_ref_layer_id shall be in the range of 0 to 62, inclusive
      assert( pcSPS->getScalingListRefLayerId() <= 62 );

      pcSPS->setScalingListPresentFlag( false );
    }
    else
    {
#endif
      READ_FLAG( uiCode, "sps_scaling_list_data_present_flag" );                 pcSPS->setScalingListPresentFlag ( uiCode );
      if(pcSPS->getScalingListPresentFlag ())
      {
        parseScalingList( pcSPS->getScalingList() );
      }
#if SCALINGLIST_INFERRING
    }
#endif
  }
  READ_FLAG( uiCode, "amp_enabled_flag" );                          pcSPS->setUseAMP( uiCode );
  READ_FLAG( uiCode, "sample_adaptive_offset_enabled_flag" );       pcSPS->setUseSAO ( uiCode ? true : false );

  READ_FLAG( uiCode, "pcm_enabled_flag" ); pcSPS->setUsePCM( uiCode ? true : false );
  if( pcSPS->getUsePCM() )
  {
    READ_CODE( 4, uiCode, "pcm_sample_bit_depth_luma_minus1" );          pcSPS->setPCMBitDepthLuma   ( 1 + uiCode );
    READ_CODE( 4, uiCode, "pcm_sample_bit_depth_chroma_minus1" );        pcSPS->setPCMBitDepthChroma ( 1 + uiCode );
    READ_UVLC( uiCode, "log2_min_pcm_luma_coding_block_size_minus3" );   pcSPS->setPCMLog2MinSize (uiCode+3);
    READ_UVLC( uiCode, "log2_diff_max_min_pcm_luma_coding_block_size" ); pcSPS->setPCMLog2MaxSize ( uiCode+pcSPS->getPCMLog2MinSize() );
    READ_FLAG( uiCode, "pcm_loop_filter_disable_flag" );                 pcSPS->setPCMFilterDisableFlag ( uiCode ? true : false );
  }

  READ_UVLC( uiCode, "num_short_term_ref_pic_sets" );
  assert(uiCode <= 64);
  pcSPS->createRPSList(uiCode);

  TComRPSList* rpsList = pcSPS->getRPSList();
  TComReferencePictureSet* rps;

  for(UInt i=0; i< (UInt)rpsList->getNumberOfReferencePictureSets(); i++)
  {
    rps = rpsList->getReferencePictureSet(i);
    parseShortTermRefPicSet(pcSPS,rps,i);
  }
  READ_FLAG( uiCode, "long_term_ref_pics_present_flag" );          pcSPS->setLongTermRefsPresent(uiCode);
  if (pcSPS->getLongTermRefsPresent())
  {
    READ_UVLC( uiCode, "num_long_term_ref_pic_sps" );
    pcSPS->setNumLongTermRefPicSPS(uiCode);
    for (UInt k = 0; k < pcSPS->getNumLongTermRefPicSPS(); k++)
    {
      READ_CODE( pcSPS->getBitsForPOC(), uiCode, "lt_ref_pic_poc_lsb_sps" );
      pcSPS->setLtRefPicPocLsbSps(k, uiCode);
      READ_FLAG( uiCode,  "used_by_curr_pic_lt_sps_flag[i]");
      pcSPS->setUsedByCurrPicLtSPSFlag(k, uiCode?1:0);
    }
  }
  READ_FLAG( uiCode, "sps_temporal_mvp_enable_flag" );            pcSPS->setTMVPFlagsPresent(uiCode);
  READ_FLAG( uiCode, "sps_strong_intra_smoothing_enable_flag" );  pcSPS->setUseStrongIntraSmoothing(uiCode);

  READ_FLAG( uiCode, "vui_parameters_present_flag" );             pcSPS->setVuiParametersPresentFlag(uiCode);

  if (pcSPS->getVuiParametersPresentFlag())
  {
    parseVUI(pcSPS->getVuiParameters(), pcSPS);
  }

  READ_FLAG( uiCode, "sps_extension_flag");

#if SVC_EXTENSION
  pcSPS->setExtensionFlag( uiCode ? true : false );

  if( pcSPS->getExtensionFlag() )
  {
#if !R0042_PROFILE_INDICATION
#if O0142_CONDITIONAL_SPS_EXTENSION
    UInt spsExtensionTypeFlag[8];
    for (UInt i = 0; i < 8; i++)
    {
      READ_FLAG( spsExtensionTypeFlag[i], "sps_extension_type_flag" );
    }
    if (spsExtensionTypeFlag[1])
    {
      parseSPSExtension( pcSPS );
    }
    if (spsExtensionTypeFlag[7])
    {
#else
    parseSPSExtension( pcSPS );
    READ_FLAG( uiCode, "sps_extension2_flag");
    if(uiCode)
    {
#endif
      while ( xMoreRbspData() )
      {
        READ_FLAG( uiCode, "sps_extension_data_flag");
      }
    }
    }
#else
    READ_FLAG( uiCode, "sps_range_extension_flag" );
    assert(uiCode == 0);
    READ_FLAG( uiCode, "sps_multilayer_extension_flag" );
    assert(uiCode == 1);
    READ_CODE(6, uiCode, "sps_extension_6bits");
    assert(uiCode == 0);
    parseSPSExtension( pcSPS );
  }
#endif
#else
  if (uiCode)
  {
    while ( xMoreRbspData() )
    {
      READ_FLAG( uiCode, "sps_extension_data_flag");
    }
  }
#endif
}

#if SVC_EXTENSION
Void TDecCavlc::parseSPSExtension( TComSPS* pcSPS )
{
  UInt uiCode;
  // more syntax elements to be parsed here

  READ_FLAG( uiCode, "inter_view_mv_vert_constraint_flag" );
  // Vertical MV component restriction is not used in SHVC CTC
  assert( uiCode == 0 );

#if !MOVE_SCALED_OFFSET_TO_PPS
  if( pcSPS->getLayerId() > 0 )
  {
    Int iCode;
    READ_UVLC( uiCode,      "num_scaled_ref_layer_offsets" ); pcSPS->setNumScaledRefLayerOffsets(uiCode);
    for(Int i = 0; i < pcSPS->getNumScaledRefLayerOffsets(); i++)
    {
      Window& scaledWindow = pcSPS->getScaledRefLayerWindow(i);
#if O0098_SCALED_REF_LAYER_ID
      READ_CODE( 6,  uiCode,  "scaled_ref_layer_id" );       pcSPS->setScaledRefLayerId( i, uiCode );
#endif
      READ_SVLC( iCode, "scaled_ref_layer_left_offset" );    scaledWindow.setWindowLeftOffset  (iCode << 1);
      READ_SVLC( iCode, "scaled_ref_layer_top_offset" );     scaledWindow.setWindowTopOffset   (iCode << 1);
      READ_SVLC( iCode, "scaled_ref_layer_right_offset" );   scaledWindow.setWindowRightOffset (iCode << 1);
      READ_SVLC( iCode, "scaled_ref_layer_bottom_offset" );  scaledWindow.setWindowBottomOffset(iCode << 1);
#if P0312_VERT_PHASE_ADJ
      READ_FLAG( uiCode, "vert_phase_position_enable_flag" ); scaledWindow.setVertPhasePositionEnableFlag(uiCode);  pcSPS->setVertPhasePositionEnableFlag( pcSPS->getScaledRefLayerId(i), uiCode);    
#endif
    }
  }
#endif
}
#endif

Void TDecCavlc::parseVPS()
{
#if TRACE_DEBUG
  printf("PARSE VPS \n");
#endif
  UInt  uiCode;

  READ_CODE( 4,  uiCode,  "vps_video_parameter_set_id" );
  TComVPS* pcVPS = m_pVPS + uiCode ;
  pcVPS->init();
  pcVPS->setVPSId( uiCode );

#if VPS_RESERVED_FLAGS
  READ_FLAG( uiCode, "vps_base_layer_internal_flag");             pcVPS->setBaseLayerInternalFlag( uiCode ? true : false );
  READ_FLAG( uiCode, "vps_base_layer_available_flag");            pcVPS->setBaseLayerAvailableFlag( uiCode ? true : false );
#if VPS_AVC_BL_FLAG_REMOVAL
  pcVPS->setNonHEVCBaseLayerFlag( (pcVPS->getBaseLayerAvailableFlag() && !pcVPS->getBaseLayerInternalFlag()) ? true : false);
#endif
#else
  READ_CODE( 2,  uiCode,  "vps_reserved_three_2bits" );           assert(uiCode == 3);
#endif
#if SVC_EXTENSION
#if O0137_MAX_LAYERID
  READ_CODE( 6,  uiCode,  "vps_max_layers_minus1" );              pcVPS->setMaxLayers( min( 62u, uiCode) + 1 );
#else
  READ_CODE( 6,  uiCode,  "vps_max_layers_minus1" );              pcVPS->setMaxLayers( uiCode + 1 );
#endif
#else
  READ_CODE( 6,  uiCode,  "vps_reserved_zero_6bits" );            assert(uiCode == 0);
#endif
  READ_CODE( 3,  uiCode,  "vps_max_sub_layers_minus1" );          pcVPS->setMaxTLayers( uiCode + 1 ); assert(uiCode+1 <= MAX_TLAYER);
  READ_FLAG(     uiCode,  "vps_temporal_id_nesting_flag" );       pcVPS->setTemporalNestingFlag( uiCode ? true:false );
  assert (pcVPS->getMaxTLayers()>1||pcVPS->getTemporalNestingFlag());
#if !P0125_REVERT_VPS_EXTN_OFFSET_TO_RESERVED
#if VPS_EXTN_OFFSET
  READ_CODE( 16, uiCode,  "vps_extension_offset" );               pcVPS->setExtensionOffset( uiCode );
#else
  READ_CODE( 16, uiCode,  "vps_reserved_ffff_16bits" );           assert(uiCode == 0xffff);
#endif
#else
  READ_CODE( 16, uiCode,  "vps_reserved_ffff_16bits" );           assert(uiCode == 0xffff);
#endif
  parsePTL ( pcVPS->getPTL(), true, pcVPS->getMaxTLayers()-1);
  UInt subLayerOrderingInfoPresentFlag;
  READ_FLAG(subLayerOrderingInfoPresentFlag, "vps_sub_layer_ordering_info_present_flag");
  for(UInt i = 0; i <= pcVPS->getMaxTLayers()-1; i++)
  {
    READ_UVLC( uiCode,  "vps_max_dec_pic_buffering_minus1[i]" );     pcVPS->setMaxDecPicBuffering( uiCode + 1, i );
    READ_UVLC( uiCode,  "vps_num_reorder_pics[i]" );          pcVPS->setNumReorderPics( uiCode, i );
    READ_UVLC( uiCode,  "vps_max_latency_increase_plus1[i]" );      pcVPS->setMaxLatencyIncrease( uiCode, i );

    if (!subLayerOrderingInfoPresentFlag)
    {
      for (i++; i <= pcVPS->getMaxTLayers()-1; i++)
      {
        pcVPS->setMaxDecPicBuffering(pcVPS->getMaxDecPicBuffering(0), i);
        pcVPS->setNumReorderPics(pcVPS->getNumReorderPics(0), i);
        pcVPS->setMaxLatencyIncrease(pcVPS->getMaxLatencyIncrease(0), i);
      }
      break;
    }
  }

#if SVC_EXTENSION
  assert( pcVPS->getNumHrdParameters() < MAX_VPS_LAYER_SETS_PLUS1 );
  assert( pcVPS->getMaxLayerId()       < MAX_VPS_LAYER_ID_PLUS1 );
  READ_CODE( 6, uiCode, "vps_max_layer_id" );           pcVPS->setMaxLayerId( uiCode );
#if Q0078_ADD_LAYER_SETS
  READ_UVLC(uiCode, "vps_num_layer_sets_minus1");  pcVPS->setVpsNumLayerSetsMinus1(uiCode);
  pcVPS->setNumLayerSets(pcVPS->getVpsNumLayerSetsMinus1() + 1);
  for (UInt opsIdx = 1; opsIdx <= pcVPS->getVpsNumLayerSetsMinus1(); opsIdx++)
#else
  READ_UVLC(    uiCode, "vps_num_layer_sets_minus1" );  pcVPS->setNumLayerSets( uiCode + 1 );
  for( UInt opsIdx = 1; opsIdx <= ( pcVPS->getNumLayerSets() - 1 ); opsIdx ++ )
#endif
  {
    // Operation point set
    for( UInt i = 0; i <= pcVPS->getMaxLayerId(); i ++ )
#else
  assert( pcVPS->getNumHrdParameters() < MAX_VPS_OP_SETS_PLUS1 );
  assert( pcVPS->getMaxNuhReservedZeroLayerId() < MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1 );
  READ_CODE( 6, uiCode, "vps_max_nuh_reserved_zero_layer_id" );   pcVPS->setMaxNuhReservedZeroLayerId( uiCode );
  READ_UVLC(    uiCode, "vps_max_op_sets_minus1" );               pcVPS->setMaxOpSets( uiCode + 1 );
  for( UInt opsIdx = 1; opsIdx <= ( pcVPS->getMaxOpSets() - 1 ); opsIdx ++ )
  {
    // Operation point set
    for( UInt i = 0; i <= (UInt)pcVPS->getMaxNuhReservedZeroLayerId(); i ++ )
#endif
    {
      READ_FLAG( uiCode, "layer_id_included_flag[opsIdx][i]" );   pcVPS->setLayerIdIncludedFlag( uiCode == 1 ? true : false, opsIdx, i );
    }
  }
#if DERIVE_LAYER_ID_LIST_VARIABLES
  pcVPS->deriveLayerIdListVariables();
#endif
  TimingInfo *timingInfo = pcVPS->getTimingInfo();
  READ_FLAG(       uiCode, "vps_timing_info_present_flag");         timingInfo->setTimingInfoPresentFlag      (uiCode ? true : false);
  if(timingInfo->getTimingInfoPresentFlag())
  {
    READ_CODE( 32, uiCode, "vps_num_units_in_tick");                timingInfo->setNumUnitsInTick             (uiCode);
    READ_CODE( 32, uiCode, "vps_time_scale");                       timingInfo->setTimeScale                  (uiCode);
    READ_FLAG(     uiCode, "vps_poc_proportional_to_timing_flag");  timingInfo->setPocProportionalToTimingFlag(uiCode ? true : false);
    if(timingInfo->getPocProportionalToTimingFlag())
    {
      READ_UVLC(   uiCode, "vps_num_ticks_poc_diff_one_minus1");    timingInfo->setNumTicksPocDiffOneMinus1   (uiCode);
    }
    READ_UVLC( uiCode, "vps_num_hrd_parameters" );                  pcVPS->setNumHrdParameters( uiCode );

    if( pcVPS->getNumHrdParameters() > 0 )
    {
      pcVPS->createHrdParamBuffer();
    }
    for( UInt i = 0; i < pcVPS->getNumHrdParameters(); i ++ )
    {
      READ_UVLC( uiCode, "hrd_op_set_idx" );                       pcVPS->setHrdOpSetIdx( uiCode, i );
      if( i > 0 )
      {
        READ_FLAG( uiCode, "cprms_present_flag[i]" );               pcVPS->setCprmsPresentFlag( uiCode == 1 ? true : false, i );
      }
      else
      {
        pcVPS->setCprmsPresentFlag( true, i );
      }

      parseHrdParameters(pcVPS->getHrdParameters(i), pcVPS->getCprmsPresentFlag( i ), pcVPS->getMaxTLayers() - 1);
    }
  }

#if SVC_EXTENSION
  READ_FLAG( uiCode,  "vps_extension_flag" );      pcVPS->setVpsExtensionFlag( uiCode ? true : false );

  // When MaxLayersMinus1 is greater than 0, vps_extension_flag shall be equal to 1.
  if( pcVPS->getMaxLayers() > 1 )
  {
    assert( pcVPS->getVpsExtensionFlag() == true );
  }



  if( pcVPS->getVpsExtensionFlag()  )
  {
    while ( m_eReader.getNumBitsRead() % 8 != 0 )
    {
      READ_FLAG( uiCode, "vps_extension_alignment_bit_equal_to_one"); assert(uiCode == 1);
    }
    parseVPSExtension(pcVPS);
    READ_FLAG( uiCode, "vps_entension2_flag" );
    if(uiCode)
    {
      while ( xMoreRbspData() )
      {
        READ_FLAG( uiCode, "vps_extension_data_flag");
      }
    }
  }
  else
  {
    // set default parameters when syntax elements are not present
    defaultVPSExtension(pcVPS);    
  }
#else
  READ_FLAG( uiCode,  "vps_extension_flag" );
  if (uiCode)
  {
    while ( xMoreRbspData() )
    {
      READ_FLAG( uiCode, "vps_extension_data_flag");
    }
  }
#endif

  return;
}

#if SVC_EXTENSION
Void TDecCavlc::parseVPSExtension(TComVPS *vps)
{
  UInt uiCode;
  // ... More syntax elements to be parsed here
#if P0300_ALT_OUTPUT_LAYER_FLAG
  Int NumOutputLayersInOutputLayerSet[MAX_VPS_LAYER_SETS_PLUS1];
  Int OlsHighestOutputLayerId[MAX_VPS_LAYER_SETS_PLUS1];
#endif
#if LIST_OF_PTL
  if( vps->getMaxLayers() > 1 && vps->getBaseLayerInternalFlag() )
  {
    vps->setProfilePresentFlag(1, false);
    vps->getPTLForExtnPtr()->empty();
    vps->getPTLForExtnPtr()->resize(2);
    vps->getPTLForExtn(1)->copyProfileInfo( vps->getPTL() );
    parsePTL( vps->getPTLForExtn(1), vps->getProfilePresentFlag(1), vps->getMaxTLayers() - 1 );
  }
#endif
#if VPS_EXTN_MASK_AND_DIM_INFO
  UInt numScalabilityTypes = 0, i = 0, j = 0;

#if !VPS_AVC_BL_FLAG_REMOVAL
  READ_FLAG( uiCode, "avc_base_layer_flag" ); vps->setAvcBaseLayerFlag(uiCode ? true : false);
#endif

#if !P0307_REMOVE_VPS_VUI_OFFSET
#if O0109_MOVE_VPS_VUI_FLAG
  READ_FLAG( uiCode, "vps_vui_present_flag"); vps->setVpsVuiPresentFlag(uiCode ? true : false);
  if ( uiCode )
  {
#endif
#if VPS_VUI_OFFSET
    READ_CODE( 16, uiCode, "vps_vui_offset" );  vps->setVpsVuiOffset( uiCode );
#endif
#if O0109_MOVE_VPS_VUI_FLAG
  }
#endif
#endif
  READ_FLAG( uiCode, "splitting_flag" ); vps->setSplittingFlag(uiCode ? true : false);

  for(i = 0; i < MAX_VPS_NUM_SCALABILITY_TYPES; i++)
  {
    READ_FLAG( uiCode, "scalability_mask[i]" ); vps->setScalabilityMask(i, uiCode ? true : false);
    numScalabilityTypes += uiCode;
  }
  vps->setNumScalabilityTypes(numScalabilityTypes);

  for(j = 0; j < numScalabilityTypes - vps->getSplittingFlag(); j++)
  {
    READ_CODE( 3, uiCode, "dimension_id_len_minus1[j]" ); vps->setDimensionIdLen(j, uiCode + 1);
  }

  // The value of dimBitOffset[ NumScalabilityTypes ] is set equal to 6.
  if(vps->getSplittingFlag())
  {
    UInt numBits = 0;
    for(j = 0; j < numScalabilityTypes - 1; j++)
    {
      numBits += vps->getDimensionIdLen(j);
    }
    assert( numBits < 6 );
    vps->setDimensionIdLen(numScalabilityTypes-1, 6 - numBits);
    numBits = 6;
  }

  READ_FLAG( uiCode, "vps_nuh_layer_id_present_flag" ); vps->setNuhLayerIdPresentFlag(uiCode ? true : false);
  vps->setLayerIdInNuh(0, 0);
  vps->setLayerIdInVps(0, 0);
  for(i = 1; i < vps->getMaxLayers(); i++)
  {
    if( vps->getNuhLayerIdPresentFlag() )
    {
      READ_CODE( 6, uiCode, "layer_id_in_nuh[i]" ); vps->setLayerIdInNuh(i, uiCode);
      assert( uiCode > vps->getLayerIdInNuh(i-1) );
    }
    else
    {
      vps->setLayerIdInNuh(i, i);
    }
    vps->setLayerIdInVps(vps->getLayerIdInNuh(i), i);

    if( !vps->getSplittingFlag() )
    {
      for(j = 0; j < numScalabilityTypes; j++)
      {
        READ_CODE( vps->getDimensionIdLen(j), uiCode, "dimension_id[i][j]" ); vps->setDimensionId(i, j, uiCode);
#if !AUXILIARY_PICTURES
        assert( uiCode <= vps->getMaxLayerId() );
#endif
      }
    }
  }
#endif
#if VIEW_ID_RELATED_SIGNALING
  // if ( pcVPS->getNumViews() > 1 )
  //   However, this is a bug in the text since, view_id_len_minus1 is needed to parse view_id_val.
  {
#if O0109_VIEW_ID_LEN
    READ_CODE( 4, uiCode, "view_id_len" ); vps->setViewIdLen( uiCode );
#else
    READ_CODE( 4, uiCode, "view_id_len_minus1" ); vps->setViewIdLenMinus1( uiCode );
#endif
  }

#if O0109_VIEW_ID_LEN
  if ( vps->getViewIdLen() > 0 )
  {
    for(  i = 0; i < (UInt)vps->getNumViews(); i++ )
    {
      READ_CODE( vps->getViewIdLen( ), uiCode, "view_id_val[i]" ); vps->setViewIdVal( i, uiCode );
    }
  }
#else
  for(  i = 0; i < (UInt)vps->getNumViews(); i++ )
  {
    READ_CODE( vps->getViewIdLenMinus1( ) + 1, uiCode, "view_id_val[i]" ); vps->setViewIdVal( i, uiCode );
  }
#endif
#endif // view id related signaling
#if VPS_EXTN_DIRECT_REF_LAYERS
  // For layer 0
  vps->setNumDirectRefLayers(0, 0);
  // For other layers
  for( Int layerCtr = 1; layerCtr <= (Int)vps->getMaxLayers() - 1; layerCtr++)
  {
    UInt numDirectRefLayers = 0;
    for( Int refLayerCtr = 0; refLayerCtr < layerCtr; refLayerCtr++)
    {
      READ_FLAG(uiCode, "direct_dependency_flag[i][j]" ); vps->setDirectDependencyFlag(layerCtr, refLayerCtr, uiCode? true : false);
      if(uiCode)
      {
        vps->setRefLayerId(layerCtr, numDirectRefLayers, refLayerCtr);
        numDirectRefLayers++;
      }
    }
    vps->setNumDirectRefLayers(layerCtr, numDirectRefLayers);
  }
#endif
#if Q0078_ADD_LAYER_SETS
#if O0092_0094_DEPENDENCY_CONSTRAINT // Moved here
  vps->setNumRefLayers();

  if (vps->getMaxLayers() > MAX_REF_LAYERS)
  {
    for (i = 1; i < vps->getMaxLayers(); i++)
    {
      assert(vps->getNumRefLayers(vps->getLayerIdInNuh(i)) <= MAX_REF_LAYERS);
    }
  }
#endif
  vps->setPredictedLayerIds();
  vps->setTreePartitionLayerIdList();
#endif
#if MOVE_ADDN_LS_SIGNALLING
#if Q0078_ADD_LAYER_SETS
  if (vps->getNumIndependentLayers() > 1)
  {
    READ_UVLC(uiCode, "num_add_layer_sets"); vps->setNumAddLayerSets(uiCode);
    for (i = 0; i < (UInt)vps->getNumAddLayerSets(); i++)
    {
      for (j = 1; j < (UInt)vps->getNumIndependentLayers(); j++)
      {
        int len = 1;
        while ((1 << len) < (vps->getNumLayersInTreePartition(j) + 1))
        {
          len++;
        }
        READ_CODE(len, uiCode, "highest_layer_idx_plus1[i][j]"); vps->setHighestLayerIdxPlus1(i, j, uiCode);
      }
    }
    vps->setNumLayerSets(vps->getNumLayerSets() + vps->getNumAddLayerSets());
    vps->setLayerIdIncludedFlagsForAddLayerSets();
  }
#endif
#endif
#if VPS_TSLAYERS
  READ_FLAG( uiCode, "vps_sub_layers_max_minus1_present_flag"); vps->setMaxTSLayersPresentFlag(uiCode ? true : false);

  if (vps->getMaxTSLayersPresentFlag())
  {
    for(i = 0; i < vps->getMaxLayers(); i++)
    {
      READ_CODE( 3, uiCode, "sub_layers_vps_max_minus1[i]" ); vps->setMaxTSLayersMinus1(i, uiCode);
    }
  }
  else
  {
    for( i = 0; i < vps->getMaxLayers(); i++)
    {
      vps->setMaxTSLayersMinus1(i, vps->getMaxTLayers()-1);
    }
  }
#endif
  READ_FLAG( uiCode, "max_tid_ref_present_flag"); vps->setMaxTidRefPresentFlag(uiCode ? true : false);
  if (vps->getMaxTidRefPresentFlag())
  {
    for(i = 0; i < vps->getMaxLayers() - 1; i++)
    {
#if O0225_MAX_TID_FOR_REF_LAYERS
      for( j = i+1; j <= vps->getMaxLayers() - 1; j++)
      {
        if(vps->getDirectDependencyFlag(j, i))
        {
          READ_CODE( 3, uiCode, "max_tid_il_ref_pics_plus1[i][j]" ); vps->setMaxTidIlRefPicsPlus1(i, j, uiCode);          
        }
      }
#else
      READ_CODE( 3, uiCode, "max_tid_il_ref_pics_plus1[i]" ); vps->setMaxTidIlRefPicsPlus1(i, uiCode);
      assert( uiCode <= vps->getMaxTLayers());
#endif 
    }
  }
  else
  {
    for(i = 0; i < vps->getMaxLayers() - 1; i++)
    {
#if O0225_MAX_TID_FOR_REF_LAYERS
      for( j = i+1; j <= vps->getMaxLayers() - 1; j++)
      {
        vps->setMaxTidIlRefPicsPlus1(i, j, 7);
      }
#else
      vps->setMaxTidIlRefPicsPlus1(i, 7);
#endif
    }
  }
  READ_FLAG( uiCode, "all_ref_layers_active_flag" ); vps->setIlpSshSignalingEnabledFlag(uiCode ? true : false);
#if VPS_EXTN_PROFILE_INFO
  // Profile-tier-level signalling
#if !VPS_EXTN_UEV_CODING
  READ_CODE( 10, uiCode, "vps_number_layer_sets_minus1" );     assert( uiCode == (vps->getNumLayerSets() - 1) );
  READ_CODE(  6, uiCode, "vps_num_profile_tier_level_minus1"); vps->setNumProfileTierLevel( uiCode + 1 );
#else
  READ_UVLC(  uiCode, "vps_num_profile_tier_level_minus1"); vps->setNumProfileTierLevel( uiCode + 1 );
#endif
#if PER_LAYER_PTL
  Int const numBitsForPtlIdx = vps->calculateLenOfSyntaxElement( vps->getNumProfileTierLevel() );
#endif
  vps->getPTLForExtnPtr()->resize(vps->getNumProfileTierLevel());
#if LIST_OF_PTL
  for(Int idx = vps->getBaseLayerInternalFlag() ? 2 : 1; idx <= (Int)vps->getNumProfileTierLevel() - 1; idx++)
#else
  for(Int idx = 1; idx <= vps->getNumProfileTierLevel() - 1; idx++)
#endif
  {
    READ_FLAG( uiCode, "vps_profile_present_flag[i]" ); vps->setProfilePresentFlag(idx, uiCode ? true : false);
    if( !vps->getProfilePresentFlag(idx) )
    {
#if P0048_REMOVE_PROFILE_REF
      // Copy profile information from previous one
      vps->getPTLForExtn(idx)->copyProfileInfo( (idx==1) ? vps->getPTL() : vps->getPTLForExtn( idx - 1 ) );
#else
      READ_CODE( 6, uiCode, "profile_ref_minus1[i]" ); vps->setProfileLayerSetRef(idx, uiCode + 1);
#if O0109_PROF_REF_MINUS1
      assert( vps->getProfileLayerSetRef(idx) <= idx );
#else
      assert( vps->getProfileLayerSetRef(idx) < idx );
#endif
      // Copy profile information as indicated
      vps->getPTLForExtn(idx)->copyProfileInfo( vps->getPTLForExtn( vps->getProfileLayerSetRef(idx) ) );
#endif
    }
    parsePTL( vps->getPTLForExtn(idx), vps->getProfilePresentFlag(idx), vps->getMaxTLayers() - 1 );
  }
#endif

#if !MOVE_ADDN_LS_SIGNALLING
#if Q0078_ADD_LAYER_SETS
  if (vps->getNumIndependentLayers() > 1)
  {
    READ_UVLC(uiCode, "num_add_layer_sets"); vps->setNumAddLayerSets(uiCode);
    for (i = 0; i < vps->getNumAddLayerSets(); i++)
    {
      for (j = 1; j < vps->getNumIndependentLayers(); j++)
      {
        int len = 1;
        while ((1 << len) < (vps->getNumLayersInTreePartition(j) + 1))
        {
          len++;
        }
        READ_CODE(len, uiCode, "highest_layer_idx_plus1[i][j]"); vps->setHighestLayerIdxPlus1(i, j, uiCode);
      }
    }
    vps->setNumLayerSets(vps->getNumLayerSets() + vps->getNumAddLayerSets());
    vps->setLayerIdIncludedFlagsForAddLayerSets();
  }
#endif
#endif

#if !VPS_EXTN_UEV_CODING
  READ_FLAG( uiCode, "more_output_layer_sets_than_default_flag" ); vps->setMoreOutputLayerSetsThanDefaultFlag( uiCode ? true : false );
  Int numOutputLayerSets = 0;
  if(! vps->getMoreOutputLayerSetsThanDefaultFlag() )
  {
    numOutputLayerSets = vps->getNumLayerSets();
  }
  else
  {
    READ_CODE( 10, uiCode, "num_add_output_layer_sets" );          vps->setNumAddOutputLayerSets( uiCode );
    numOutputLayerSets = vps->getNumLayerSets() + vps->getNumAddOutputLayerSets();
  }
#else

#if Q0165_NUM_ADD_OUTPUT_LAYER_SETS
  if( vps->getNumLayerSets() > 1 )
  {
    READ_UVLC( uiCode, "num_add_olss" );            vps->setNumAddOutputLayerSets( uiCode );
    READ_CODE( 2, uiCode, "default_output_layer_idc" );   vps->setDefaultTargetOutputLayerIdc( uiCode );
  }
  else
  {
    vps->setNumAddOutputLayerSets( 0 );
  }
#else
  READ_UVLC( uiCode, "num_add_output_layer_sets" );          vps->setNumAddOutputLayerSets( uiCode );
#endif

  // The value of num_add_olss shall be in the range of 0 to 1023, inclusive.
  assert( vps->getNumAddOutputLayerSets() >= 0 && vps->getNumAddOutputLayerSets() < 1024 );

  Int numOutputLayerSets = vps->getNumLayerSets() + vps->getNumAddOutputLayerSets();
#endif

#if P0295_DEFAULT_OUT_LAYER_IDC
#if !Q0165_NUM_ADD_OUTPUT_LAYER_SETS
  if( numOutputLayerSets > 1 )
  {
    READ_CODE( 2, uiCode, "default_target_output_layer_idc" );   vps->setDefaultTargetOutputLayerIdc( uiCode );
  }
#endif
  vps->setNumOutputLayerSets( numOutputLayerSets );
#if NECESSARY_LAYER_FLAG
  // Default output layer set
  vps->setOutputLayerSetIdx(0, 0);
  vps->setOutputLayerFlag(0, 0, true);

  vps->deriveNecessaryLayerFlag(0);
#if PER_LAYER_PTL
  vps->getProfileLevelTierIdx()->resize(numOutputLayerSets);
  vps->getProfileLevelTierIdx(0)->push_back( vps->getBaseLayerInternalFlag() && vps->getMaxLayers() > 1 ? 1 : 0);
#endif
#endif
  for(i = 1; i < (UInt)numOutputLayerSets; i++)
  {
    if( i > (vps->getNumLayerSets() - 1) )
    {
      Int numBits = 1;
      while ((1 << numBits) < (vps->getNumLayerSets() - 1))
      {
        numBits++;
      }
      READ_CODE( numBits, uiCode, "layer_set_idx_for_ols_minus1");   vps->setOutputLayerSetIdx( i, uiCode + 1);
    }
    else
    {
      vps->setOutputLayerSetIdx( i, i );
    }
    Int layerSetIdxForOutputLayerSet = vps->getOutputLayerSetIdx(i);
#if Q0078_ADD_LAYER_SETS
    if ( i > vps->getVpsNumLayerSetsMinus1() || vps->getDefaultTargetOutputLayerIdc() >= 2 )
#else
    if ( i > (vps->getNumLayerSets() - 1) || vps->getDefaultTargetOutputLayerIdc() >= 2 )
#endif
    {
#if NUM_OL_FLAGS
      for(j = 0; j < (UInt)vps->getNumLayersInIdList(layerSetIdxForOutputLayerSet); j++)
#else
      for(j = 0; j < vps->getNumLayersInIdList(lsIdx) - 1; j++)
#endif
      {
        READ_FLAG( uiCode, "output_layer_flag[i][j]"); vps->setOutputLayerFlag(i, j, uiCode);
      }
    }
    else
    {
      // i <= (vps->getNumLayerSets() - 1)
      // Assign OutputLayerFlag depending on default_one_target_output_layer_flag
      if( vps->getDefaultTargetOutputLayerIdc() == 1 )
      {
        for(j = 0; j < (UInt)vps->getNumLayersInIdList(layerSetIdxForOutputLayerSet); j++)
        {
#if DEF_OPT_LAYER_IDC
          vps->setOutputLayerFlag(i, j, (j == (vps->getNumLayersInIdList(layerSetIdxForOutputLayerSet)-1))  );

#else
          vps->setOutputLayerFlag(i, j, (j == (vps->getNumLayersInIdList(layerSetIdxForOutputLayerSet)-1)) && (vps->getDimensionId(j,1) == 0) );
#endif
        }
      }
      else if ( vps->getDefaultTargetOutputLayerIdc() == 0 )
      {
        for(j = 0; j < (UInt)vps->getNumLayersInIdList(layerSetIdxForOutputLayerSet); j++)
        {
          vps->setOutputLayerFlag(i, j, 1);
        }
      }
    }
#if NECESSARY_LAYER_FLAG
    vps->deriveNecessaryLayerFlag(i);  
#endif
#if PER_LAYER_PTL
    vps->getProfileLevelTierIdx(i)->assign(vps->getNumLayersInIdList(layerSetIdxForOutputLayerSet), -1);
    for(j = 0; j < (UInt)vps->getNumLayersInIdList(layerSetIdxForOutputLayerSet) ; j++)
    {
      if( vps->getNecessaryLayerFlag(i, j) )
      {
        READ_CODE( numBitsForPtlIdx, uiCode, "profile_level_tier_idx[i]" ); 
        vps->setProfileLevelTierIdx(i, j, uiCode );
      }
    }
#else
    Int numBits = 1;
    while ((1 << numBits) < (vps->getNumProfileTierLevel()))
    {
      numBits++;
    }
    READ_CODE( numBits, uiCode, "profile_level_tier_idx[i]" );     vps->setProfileLevelTierIdx(i, uiCode);
#endif
#if P0300_ALT_OUTPUT_LAYER_FLAG
    NumOutputLayersInOutputLayerSet[i] = 0;
    for (j = 0; j < (UInt)vps->getNumLayersInIdList(layerSetIdxForOutputLayerSet); j++)
    {
      NumOutputLayersInOutputLayerSet[i] += vps->getOutputLayerFlag(i, j);
      if (vps->getOutputLayerFlag(i, j))
      {
        OlsHighestOutputLayerId[i] = vps->getLayerSetLayerIdList(layerSetIdxForOutputLayerSet, j);
      }
    }
    if (NumOutputLayersInOutputLayerSet[i] == 1 && vps->getNumDirectRefLayers(OlsHighestOutputLayerId[i]) > 0)
    {
      READ_FLAG(uiCode, "alt_output_layer_flag[i]");
      vps->setAltOuputLayerFlag(i, uiCode ? true : false);
    }
#if ALT_OPT_LAYER_FLAG
    else
    {
          uiCode=0;
          vps->setAltOuputLayerFlag(i, uiCode ? true : false);
    }
#endif
#if Q0165_OUTPUT_LAYER_SET
    assert( NumOutputLayersInOutputLayerSet[i]>0 );
#endif

#endif
  }
#if NECESSARY_LAYER_FLAG
  vps->checkNecessaryLayerFlagCondition();  
#endif
#else
  if( numOutputLayerSets > 1 )
  {
#if O0109_DEFAULT_ONE_OUT_LAYER_IDC
    READ_CODE( 2, uiCode, "default_one_target_output_layer_idc" );   vps->setDefaultOneTargetOutputLayerIdc( uiCode );
#else
    READ_FLAG( uiCode, "default_one_target_output_layer_flag" );   vps->setDefaultOneTargetOutputLayerFlag( uiCode ? true : false );
#endif
  }
  vps->setNumOutputLayerSets( numOutputLayerSets );

  for(i = 1; i < numOutputLayerSets; i++)
  {
    if( i > (vps->getNumLayerSets() - 1) )
    {
      Int numBits = 1;
      while ((1 << numBits) < (vps->getNumLayerSets() - 1))
      {
        numBits++;
      }
      READ_CODE( numBits, uiCode, "output_layer_set_idx_minus1");   vps->setOutputLayerSetIdx( i, uiCode + 1);
      Int lsIdx = vps->getOutputLayerSetIdx(i);
#if NUM_OL_FLAGS
      for(j = 0; j < vps->getNumLayersInIdList(lsIdx) ; j++)
#else
      for(j = 0; j < vps->getNumLayersInIdList(lsIdx) - 1; j++)
#endif
      {
        READ_FLAG( uiCode, "output_layer_flag[i][j]"); vps->setOutputLayerFlag(i, j, uiCode);
      }
    }
    else
    {
#if VPS_DPB_SIZE_TABLE 
      vps->setOutputLayerSetIdx( i, i );
#endif
      // i <= (vps->getNumLayerSets() - 1)
      // Assign OutputLayerFlag depending on default_one_target_output_layer_flag
      Int lsIdx = i;
#if O0109_DEFAULT_ONE_OUT_LAYER_IDC
      if( vps->getDefaultOneTargetOutputLayerIdc() == 1 )
      {
        for(j = 0; j < vps->getNumLayersInIdList(lsIdx); j++)
        {
#if O0135_DEFAULT_ONE_OUT_SEMANTIC
#if DEF_OPT_LAYER_IDC
        vps->setOutputLayerFlag(i, j, (j == (vps->getNumLayersInIdList(lsIdx)-1)) );
#else
          vps->setOutputLayerFlag(i, j, (j == (vps->getNumLayersInIdList(lsIdx)-1)) && (vps->getDimensionId(j,1)==0) );
#endif
#else
          vps->setOutputLayerFlag(i, j, (j == (vps->getNumLayersInIdList(lsIdx)-1)));
#endif
        }
      }
      else if ( vps->getDefaultOneTargetOutputLayerIdc() == 0 )
      {
        for(j = 0; j < vps->getNumLayersInIdList(lsIdx); j++)
        {
          vps->setOutputLayerFlag(i, j, 1);
        }
      }
      else
      {
        // Other values of default_one_target_output_layer_idc than 0 and 1 are reserved for future use.
      }
#else
      if( vps->getDefaultOneTargetOutputLayerFlag() )
      {
        for(j = 0; j < vps->getNumLayersInIdList(lsIdx); j++)
        {
          vps->setOutputLayerFlag(i, j, (j == (vps->getNumLayersInIdList(lsIdx)-1)));
        }
      }
      else
      {
        for(j = 0; j < vps->getNumLayersInIdList(lsIdx); j++)
        {
          vps->setOutputLayerFlag(i, j, 1);
        }
      }
#endif
    }
    Int numBits = 1;
    while ((1 << numBits) < (vps->getNumProfileTierLevel()))
    {
      numBits++;
    }
    READ_CODE( numBits, uiCode, "profile_level_tier_idx[i]" );     vps->setProfileLevelTierIdx(i, uiCode);
  }
#endif

#if !P0300_ALT_OUTPUT_LAYER_FLAG
#if O0153_ALT_OUTPUT_LAYER_FLAG
  if( vps->getMaxLayers() > 1 )
  {
    READ_FLAG( uiCode, "alt_output_layer_flag");
    vps->setAltOuputLayerFlag( uiCode ? true : false );
  }
#endif
#endif

#if REPN_FORMAT_IN_VPS
#if Q0195_REP_FORMAT_CLEANUP
  READ_UVLC( uiCode, "vps_num_rep_formats_minus1" );
  vps->setVpsNumRepFormats( uiCode + 1 );

  // The value of vps_num_rep_formats_minus1 shall be in the range of 0 to 255, inclusive.
  assert( vps->getVpsNumRepFormats() > 0 && vps->getVpsNumRepFormats() <= 256 );

  for(i = 0; i < (UInt)vps->getVpsNumRepFormats(); i++)
  {
    // Read rep_format_structures
    parseRepFormat( vps->getVpsRepFormat(i), i > 0 ? vps->getVpsRepFormat(i-1) : 0 );
  }

  // Default assignment for layer 0
  vps->setVpsRepFormatIdx( 0, 0 );

  if( vps->getVpsNumRepFormats() > 1 )
  {
    READ_FLAG( uiCode, "rep_format_idx_present_flag");
    vps->setRepFormatIdxPresentFlag( uiCode ? true : false );
  }
  else
  {
    // When not present, the value of rep_format_idx_present_flag is inferred to be equal to 0
    vps->setRepFormatIdxPresentFlag( false );
  }

  if( vps->getRepFormatIdxPresentFlag() )
  {
    for(i = 1; i < vps->getMaxLayers(); i++)
    {
      Int numBits = 1;
      while ((1 << numBits) < (vps->getVpsNumRepFormats()))
      {
        numBits++;
      }
      READ_CODE( numBits, uiCode, "vps_rep_format_idx[i]" );
      vps->setVpsRepFormatIdx( i, uiCode );
    }
  }
  else
  {
    // When not present, the value of vps_rep_format_idx[ i ] is inferred to be equal to Min (i, vps_num_rep_formats_minus1)
    for(i = 1; i < vps->getMaxLayers(); i++)
    {
      vps->setVpsRepFormatIdx( i, min( (Int)i, vps->getVpsNumRepFormats()-1 ) );
    }
  }
#else
  READ_FLAG( uiCode, "rep_format_idx_present_flag");
  vps->setRepFormatIdxPresentFlag( uiCode ? true : false );

  if( vps->getRepFormatIdxPresentFlag() )
  {
#if O0096_REP_FORMAT_INDEX
#if !VPS_EXTN_UEV_CODING
    READ_CODE( 8, uiCode, "vps_num_rep_formats_minus1" );
#else
    READ_UVLC( uiCode, "vps_num_rep_formats_minus1" );
#endif
#else
    READ_CODE( 4, uiCode, "vps_num_rep_formats_minus1" );
#endif
    vps->setVpsNumRepFormats( uiCode + 1 );
  }
  else
  {
    // default assignment
    assert (vps->getMaxLayers() <= 16);       // If max_layers_is more than 15, num_rep_formats has to be signaled
    vps->setVpsNumRepFormats( vps->getMaxLayers() );
  }

  // The value of vps_num_rep_formats_minus1 shall be in the range of 0 to 255, inclusive.
  assert( vps->getVpsNumRepFormats() > 0 && vps->getVpsNumRepFormats() <= 256 );

  for(i = 0; i < vps->getVpsNumRepFormats(); i++)
  {
    // Read rep_format_structures
    parseRepFormat( vps->getVpsRepFormat(i), i > 0 ? vps->getVpsRepFormat(i-1) : 0 );
  }

  // Default assignment for layer 0
  vps->setVpsRepFormatIdx( 0, 0 );
  if( vps->getRepFormatIdxPresentFlag() )
  {
    for(i = 1; i < vps->getMaxLayers(); i++)
    {
      if( vps->getVpsNumRepFormats() > 1 )
      {
#if O0096_REP_FORMAT_INDEX
#if !VPS_EXTN_UEV_CODING
        READ_CODE( 8, uiCode, "vps_rep_format_idx[i]" );
#else
        Int numBits = 1;
        while ((1 << numBits) < (vps->getVpsNumRepFormats()))
        {
          numBits++;
        }
        READ_CODE( numBits, uiCode, "vps_rep_format_idx[i]" );
#endif
#else
        READ_CODE( 4, uiCode, "vps_rep_format_idx[i]" );
#endif
        vps->setVpsRepFormatIdx( i, uiCode );
      }
      else
      {
        // default assignment - only one rep_format() structure
        vps->setVpsRepFormatIdx( i, 0 );
      }
    }
  }
  else
  {
    // default assignment - each layer assigned each rep_format() structure in the order signaled
    for(i = 1; i < vps->getMaxLayers(); i++)
    {
      vps->setVpsRepFormatIdx( i, i );
    }
  }
#endif
#endif
#if RESOLUTION_BASED_DPB
  vps->assignSubDpbIndices();
#endif
  READ_FLAG(uiCode, "max_one_active_ref_layer_flag" );
  vps->setMaxOneActiveRefLayerFlag(uiCode);
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  READ_FLAG(uiCode, "vps_poc_lsb_aligned_flag");
  vps->setVpsPocLsbAlignedFlag(uiCode);
#endif
#if O0062_POC_LSB_NOT_PRESENT_FLAG
  for(i = 1; i< vps->getMaxLayers(); i++)
  {
    if( vps->getNumDirectRefLayers( vps->getLayerIdInNuh(i) ) == 0  )
    {
      READ_FLAG(uiCode, "poc_lsb_not_present_flag[i]");
      vps->setPocLsbNotPresentFlag(i, uiCode);
    }
  }
#endif
#if O0215_PHASE_ALIGNMENT
  READ_FLAG( uiCode, "cross_layer_phase_alignment_flag"); vps->setPhaseAlignFlag( uiCode == 1 ? true : false );
#endif

#if !IRAP_ALIGN_FLAG_IN_VPS_VUI
  READ_FLAG(uiCode, "cross_layer_irap_aligned_flag" );
  vps->setCrossLayerIrapAlignFlag(uiCode);
#endif

#if VPS_DPB_SIZE_TABLE
  parseVpsDpbSizeTable(vps);
#endif

#if VPS_EXTN_DIRECT_REF_LAYERS
  READ_UVLC( uiCode,           "direct_dep_type_len_minus2"); vps->setDirectDepTypeLen(uiCode+2);
#if O0096_DEFAULT_DEPENDENCY_TYPE
  READ_FLAG(uiCode, "default_direct_dependency_type_flag"); 
  vps->setDefaultDirectDependecyTypeFlag(uiCode == 1? true : false);
  if (vps->getDefaultDirectDependencyTypeFlag())
  {
    READ_CODE( vps->getDirectDepTypeLen(), uiCode, "default_direct_dependency_type" ); 
    vps->setDefaultDirectDependecyType(uiCode);
  }
#endif
  for(i = 1; i < vps->getMaxLayers(); i++)
  {
    for(j = 0; j < i; j++)
    {
      if (vps->getDirectDependencyFlag(i, j))
      {
#if O0096_DEFAULT_DEPENDENCY_TYPE
        if (vps->getDefaultDirectDependencyTypeFlag())
        {
          vps->setDirectDependencyType(i, j, vps->getDefaultDirectDependencyType());
        }
        else
        {
          READ_CODE( vps->getDirectDepTypeLen(), uiCode, "direct_dependency_type[i][j]" ); 
          vps->setDirectDependencyType(i, j, uiCode);
        }
#else
        READ_CODE( vps->getDirectDepTypeLen(), uiCode, "direct_dependency_type[i][j]" ); 
        vps->setDirectDependencyType(i, j, uiCode);
#endif
      }
    }
  }
#endif
#if !Q0078_ADD_LAYER_SETS
#if O0092_0094_DEPENDENCY_CONSTRAINT // Moved up
  vps->setNumRefLayers();

  if(vps->getMaxLayers() > MAX_REF_LAYERS)
  {
    for(i = 1;i < vps->getMaxLayers(); i++)
    {
      assert( vps->getNumRefLayers(vps->getLayerIdInNuh(i)) <= MAX_REF_LAYERS);
    }
  }
#endif
#endif

#if P0307_VPS_NON_VUI_EXTENSION
  READ_UVLC( uiCode,           "vps_non_vui_extension_length"); vps->setVpsNonVuiExtLength((Int)uiCode);

  // The value of vps_non_vui_extension_length shall be in the range of 0 to 4096, inclusive.
  assert( vps->getVpsNonVuiExtLength() >= 0 && vps->getVpsNonVuiExtLength() <= 4096 );

#if P0307_VPS_NON_VUI_EXT_UPDATE
  Int nonVuiExtByte = uiCode;
  for (i = 1; i <= (UInt)nonVuiExtByte; i++)
  {
    READ_CODE( 8, uiCode, "vps_non_vui_extension_data_byte" ); //just parse and discard for now.
  }
#else
  if ( vps->getVpsNonVuiExtLength() > 0 )
  {
    printf("\n\nUp to the current spec, the value of vps_non_vui_extension_length is supposed to be 0\n");
  }
#endif
#endif

#if !O0109_O0199_FLAGS_TO_VUI
#if M0040_ADAPTIVE_RESOLUTION_CHANGE
  READ_FLAG(uiCode, "single_layer_for_non_irap_flag" ); vps->setSingleLayerForNonIrapFlag(uiCode == 1 ? true : false);
#endif
#if HIGHER_LAYER_IRAP_SKIP_FLAG
  READ_FLAG(uiCode, "higher_layer_irap_skip_flag" ); vps->setHigherLayerIrapSkipFlag(uiCode == 1 ? true : false);
#endif
#endif

#if P0307_REMOVE_VPS_VUI_OFFSET
  READ_FLAG( uiCode, "vps_vui_present_flag"); vps->setVpsVuiPresentFlag(uiCode ? true : false);
#endif

#if O0109_MOVE_VPS_VUI_FLAG
  if ( vps->getVpsVuiPresentFlag() )
#else
  READ_FLAG( uiCode,  "vps_vui_present_flag" );
  if (uiCode)
#endif
  {
    while ( m_eReader.getNumBitsRead() % 8 != 0 )
    {
      READ_FLAG( uiCode, "vps_vui_alignment_bit_equal_to_one"); assert(uiCode == 1);
    }
    parseVPSVUI(vps);
  }
  else
  {
    // set default values for VPS VUI
    defaultVPSVUI( vps );
  }
}

Void TDecCavlc::defaultVPSExtension( TComVPS* vps )
{
  // set default parameters when they are not present
  Int i, j;

  // When layer_id_in_nuh[ i ] is not present, the value is inferred to be equal to i.
  for(i = 0; i < (Int)vps->getMaxLayers(); i++)
  {
    vps->setLayerIdInNuh(i, i);
    vps->setLayerIdInVps(vps->getLayerIdInNuh(i), i);
  }

  // When not present, sub_layers_vps_max_minus1[ i ] is inferred to be equal to vps_max_sub_layers_minus1.
  for( i = 0; i < (Int)vps->getMaxLayers(); i++)
  {
    vps->setMaxTSLayersMinus1(i, vps->getMaxTLayers()-1);
  }

  // When not present, max_tid_il_ref_pics_plus1[ i ][ j ] is inferred to be equal to 7.
  for( i = 0; i < (Int)vps->getMaxLayers() - 1; i++ )
  {
#if O0225_MAX_TID_FOR_REF_LAYERS
    for( j = i + 1; j < (Int)vps->getMaxLayers(); j++ )
    {
      vps->setMaxTidIlRefPicsPlus1(i, j, 7);
    }
#else
    vps->setMaxTidIlRefPicsPlus1(i, 7);
#endif
  }

  // When not present, the value of num_add_olss is inferred to be equal to 0.
  // NumOutputLayerSets = num_add_olss + NumLayerSets
  vps->setNumOutputLayerSets( vps->getNumLayerSets() );

  // For i in the range of 0 to NumOutputLayerSets-1, inclusive, the variable LayerSetIdxForOutputLayerSet[ i ] is derived as specified in the following: 
  // LayerSetIdxForOutputLayerSet[ i ] = ( i <= vps_number_layer_sets_minus1 ) ? i : layer_set_idx_for_ols_minus1[ i ] + 1
  for( i = 1; i < (Int)vps->getNumOutputLayerSets(); i++ )
  {
    vps->setOutputLayerSetIdx( i, i );
    Int lsIdx = vps->getOutputLayerSetIdx(i);

    for( j = 0; j < (Int)vps->getNumLayersInIdList(lsIdx); j++ )
    {
      vps->setOutputLayerFlag(i, j, 1);
    }
  }

  // The value of sub_layer_dpb_info_present_flag[ i ][ 0 ] for any possible value of i is inferred to be equal to 1
  // When not present, the value of sub_layer_dpb_info_present_flag[ i ][ j ] for j greater than 0 and any possible value of i, is inferred to be equal to be equal to 0.
  for( i = 1; i < (Int)vps->getNumOutputLayerSets(); i++ )
  {
    vps->setSubLayerDpbInfoPresentFlag( i, 0, true );
  }

  // When not present, the value of vps_num_rep_formats_minus1 is inferred to be equal to MaxLayersMinus1.
  vps->setVpsNumRepFormats( vps->getMaxLayers() );

  // When not present, the value of rep_format_idx_present_flag is inferred to be equal to 0
  vps->setRepFormatIdxPresentFlag( false );

  if( !vps->getRepFormatIdxPresentFlag() )
  {
    // When not present, the value of vps_rep_format_idx[ i ] is inferred to be equal to Min(i, vps_num_rep_formats_minus1). 
    for(i = 1; i < (Int)vps->getMaxLayers(); i++)
    {
      vps->setVpsRepFormatIdx( (Int)i, min( (Int)i, (Int)(vps->getVpsNumRepFormats() - 1) ) );
    }
  }

#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  vps->setVpsPocLsbAlignedFlag(false);
#endif

#if O0062_POC_LSB_NOT_PRESENT_FLAG
  // When not present, poc_lsb_not_present_flag[ i ] is inferred to be equal to 0.
  for(i = 1; i< (Int)vps->getMaxLayers(); i++)
  {
    vps->setPocLsbNotPresentFlag(i, 0);
  }
#endif

  // set default values for VPS VUI
  defaultVPSVUI( vps );
}

Void TDecCavlc::defaultVPSVUI( TComVPS* vps )
{
  // When not present, the value of all_layers_idr_aligned_flag is inferred to be equal to 0.
  vps->setCrossLayerIrapAlignFlag( false );

#if M0040_ADAPTIVE_RESOLUTION_CHANGE
  // When single_layer_for_non_irap_flag is not present, it is inferred to be equal to 0.
  vps->setSingleLayerForNonIrapFlag( false );
#endif

#if HIGHER_LAYER_IRAP_SKIP_FLAG
  // When higher_layer_irap_skip_flag is not present it is inferred to be equal to 0
  vps->setHigherLayerIrapSkipFlag( false );
#endif
}

#if REPN_FORMAT_IN_VPS
Void  TDecCavlc::parseRepFormat( RepFormat *repFormat, RepFormat *repFormatPrev )
{
  UInt uiCode;
#if REPN_FORMAT_CONTROL_FLAG  
  READ_CODE( 16, uiCode, "pic_width_vps_in_luma_samples" );        repFormat->setPicWidthVpsInLumaSamples ( uiCode );
  READ_CODE( 16, uiCode, "pic_height_vps_in_luma_samples" );       repFormat->setPicHeightVpsInLumaSamples( uiCode );
  READ_FLAG( uiCode, "chroma_and_bit_depth_vps_present_flag" );    repFormat->setChromaAndBitDepthVpsPresentFlag( uiCode ? true : false ); 

  if( !repFormatPrev )
  {
    // The value of chroma_and_bit_depth_vps_present_flag of the first rep_format( ) syntax structure in the VPS shall be equal to 1
    assert( repFormat->getChromaAndBitDepthVpsPresentFlag() );
  }

  if( repFormat->getChromaAndBitDepthVpsPresentFlag() )
  {
    READ_CODE( 2, uiCode, "chroma_format_vps_idc" );
#if AUXILIARY_PICTURES
    repFormat->setChromaFormatVpsIdc( ChromaFormat(uiCode) );
#else
    repFormat->setChromaFormatVpsIdc( uiCode );
#endif

    if( repFormat->getChromaFormatVpsIdc() == 3 )
    {
      READ_FLAG( uiCode, "separate_colour_plane_vps_flag" );       repFormat->setSeparateColourPlaneVpsFlag( uiCode ? true : false );
    }

    READ_CODE( 4, uiCode, "bit_depth_vps_luma_minus8" );           repFormat->setBitDepthVpsLuma  ( uiCode + 8 );
    READ_CODE( 4, uiCode, "bit_depth_vps_chroma_minus8" );         repFormat->setBitDepthVpsChroma( uiCode + 8 );
  }
  else if( repFormatPrev )
  {
    // chroma_and_bit_depth_vps_present_flag equal to 0 specifies that the syntax elements, chroma_format_vps_idc, separate_colour_plane_vps_flag, bit_depth_vps_luma_minus8, and 
    // bit_depth_vps_chroma_minus8 are not present and inferred from the previous rep_format( ) syntax structure in the VPS.

    repFormat->setChromaFormatVpsIdc        ( repFormatPrev->getChromaFormatVpsIdc() );
    repFormat->setSeparateColourPlaneVpsFlag( repFormatPrev->getSeparateColourPlaneVpsFlag() );
    repFormat->setBitDepthVpsLuma           ( repFormatPrev->getBitDepthVpsLuma() );
    repFormat->setBitDepthVpsChroma         ( repFormatPrev->getBitDepthVpsChroma() );
  }

#else 
#if AUXILIARY_PICTURES
  READ_CODE( 2, uiCode, "chroma_format_idc" );               repFormat->setChromaFormatVpsIdc( ChromaFormat(uiCode) );
#else
  READ_CODE( 2, uiCode, "chroma_format_idc" );               repFormat->setChromaFormatVpsIdc( uiCode );
#endif

  if( repFormat->getChromaFormatVpsIdc() == 3 )
  {
    READ_FLAG( uiCode, "separate_colour_plane_flag");        repFormat->setSeparateColourPlaneVpsFlag(uiCode ? true : false);
  }

  READ_CODE ( 16, uiCode, "pic_width_in_luma_samples" );     repFormat->setPicWidthVpsInLumaSamples ( uiCode );
  READ_CODE ( 16, uiCode, "pic_height_in_luma_samples" );    repFormat->setPicHeightVpsInLumaSamples( uiCode );

  READ_CODE( 4, uiCode, "bit_depth_luma_minus8" );           repFormat->setBitDepthVpsLuma  ( uiCode + 8 );
  printf(" bit_depth_luma_minus8   = %lu \n",uiCode);
  READ_CODE( 4, uiCode, "bit_depth_chroma_minus8" );         repFormat->setBitDepthVpsChroma( uiCode + 8 );
  printf(" bit_depth_chroma_minus8 = %lu \n",uiCode);
#endif 

#if R0156_CONF_WINDOW_IN_REP_FORMAT
  READ_FLAG( uiCode, "conformance_window_vps_flag" );
  if( uiCode != 0) 
  {
    Window &conf = repFormat->getConformanceWindowVps();
    READ_UVLC( uiCode, "conf_win_vps_left_offset" );         conf.setWindowLeftOffset  ( uiCode );
    READ_UVLC( uiCode, "conf_win_vps_right_offset" );        conf.setWindowRightOffset ( uiCode );
    READ_UVLC( uiCode, "conf_win_vps_top_offset" );          conf.setWindowTopOffset   ( uiCode );
    READ_UVLC( uiCode, "conf_win_vps_bottom_offset" );       conf.setWindowBottomOffset( uiCode );
  }
#endif
}
#endif
#if VPS_DPB_SIZE_TABLE
Void TDecCavlc::parseVpsDpbSizeTable( TComVPS *vps )
{
  UInt uiCode;
#if SUB_LAYERS_IN_LAYER_SET
  vps->calculateMaxSLInLayerSets();
#else
#if DPB_PARAMS_MAXTLAYERS
#if BITRATE_PICRATE_SIGNALLING
  Int * MaxSubLayersInLayerSetMinus1 = new Int[vps->getNumLayerSets()];
  for(Int i = 0; i < vps->getNumLayerSets(); i++)
#else
  Int * MaxSubLayersInLayerSetMinus1 = new Int[vps->getNumOutputLayerSets()];
  for(Int i = 1; i < vps->getNumOutputLayerSets(); i++)
#endif
  {
    UInt maxSLMinus1 = 0;
#if CHANGE_NUMSUBDPB_IDX
    Int optLsIdx = vps->getOutputLayerSetIdx( i );
#else
    Int optLsIdx = i;
#endif
#if BITRATE_PICRATE_SIGNALLING
    optLsIdx = i;
#endif
    for(Int k = 0; k < vps->getNumLayersInIdList(optLsIdx); k++ ) {
      Int  lId = vps->getLayerSetLayerIdList(optLsIdx, k);
      maxSLMinus1 = max(maxSLMinus1, vps->getMaxTSLayersMinus1(vps->getLayerIdInVps(lId)));
    }
    MaxSubLayersInLayerSetMinus1[ i ] = maxSLMinus1;
#if BITRATE_PICRATE_SIGNALLING
    vps->setMaxSLayersInLayerSetMinus1(i,MaxSubLayersInLayerSetMinus1[ i ]);
#endif
  }
#endif
#endif

#if !RESOLUTION_BASED_DPB
  vps->deriveNumberOfSubDpbs();
#endif
  for(Int i = 1; i < (Int)vps->getNumOutputLayerSets(); i++)
  {
#if CHANGE_NUMSUBDPB_IDX
    Int layerSetIdxForOutputLayerSet = vps->getOutputLayerSetIdx( i );
#endif
    READ_FLAG( uiCode, "sub_layer_flag_info_present_flag[i]");  vps->setSubLayerFlagInfoPresentFlag( i, uiCode ? true : false );
#if SUB_LAYERS_IN_LAYER_SET
    for(Int j = 0; j <= (Int)vps->getMaxSLayersInLayerSetMinus1( layerSetIdxForOutputLayerSet ); j++)
#else
#if DPB_PARAMS_MAXTLAYERS
#if BITRATE_PICRATE_SIGNALLING
    for(Int j = 0; j <= (Int)MaxSubLayersInLayerSetMinus1[ vps->getOutputLayerSetIdx( i ) ]; j++)
#else
    for(Int j = 0; j <= (Int)MaxSubLayersInLayerSetMinus1[ i ]; j++)
#endif
#else
    for(Int j = 0; j <= (Int)vps->getMaxTLayers(); j++)
#endif
#endif
    {
      if( j > 0 && vps->getSubLayerFlagInfoPresentFlag(i) )
      {
        READ_FLAG( uiCode, "sub_layer_dpb_info_present_flag[i]");  vps->setSubLayerDpbInfoPresentFlag( i, j, uiCode ? true : false);
      }
      else
      {
        if( j == 0 )  // Always signal for the first sub-layer
        {
          vps->setSubLayerDpbInfoPresentFlag( i, j, true );
        }
        else // if (j != 0) && !vps->getSubLayerFlagInfoPresentFlag(i)
        {
          vps->setSubLayerDpbInfoPresentFlag( i, j, false );
        }
      }
      if( vps->getSubLayerDpbInfoPresentFlag(i, j) )  // If sub-layer DPB information is present
      {
#if CHANGE_NUMSUBDPB_IDX
        for(Int k = 0; k < vps->getNumSubDpbs(layerSetIdxForOutputLayerSet); k++)
#else
        for(Int k = 0; k < vps->getNumSubDpbs(i); k++)
#endif
        {
#if DPB_INTERNAL_BL_SIG
            uiCode=0;
        if(vps->getBaseLayerInternalFlag()  || ( vps->getLayerSetLayerIdList(layerSetIdxForOutputLayerSet, k)   !=  0 ) )
#endif
          READ_UVLC( uiCode, "max_vps_dec_pic_buffering_minus1[i][k][j]" ); vps->setMaxVpsDecPicBufferingMinus1( i, k, j, uiCode );
        }
        READ_UVLC( uiCode, "max_vps_num_reorder_pics[i][j]" );              vps->setMaxVpsNumReorderPics( i, j, uiCode);
#if RESOLUTION_BASED_DPB
        if( vps->getNumSubDpbs(layerSetIdxForOutputLayerSet) != vps->getNumLayersInIdList( layerSetIdxForOutputLayerSet ) )  
        {
          for(Int k = 0; k < vps->getNumLayersInIdList( layerSetIdxForOutputLayerSet ); k++)
          {
            READ_UVLC( uiCode, "max_vps_layer_dec_pic_buff_minus1[i][k][j]" ); vps->setMaxVpsLayerDecPicBuffMinus1( i, k, j, uiCode);
          }
        }
        else  // vps->getNumSubDpbs(layerSetIdxForOutputLayerSet) == vps->getNumLayersInIdList( layerSetIdxForOutputLayerSet )
        {          
          for(Int k = 0; k < vps->getNumLayersInIdList( layerSetIdxForOutputLayerSet ); k++)
          {
            vps->setMaxVpsLayerDecPicBuffMinus1( i, k, j, vps->getMaxVpsDecPicBufferingMinus1( i, k, j));
          }
        }
#endif
        READ_UVLC( uiCode, "max_vps_latency_increase_plus1[i][j]" );        vps->setMaxVpsLatencyIncreasePlus1( i, j, uiCode);
      }
    }
    for(Int j = vps->getMaxTLayers(); j < MAX_TLAYER; j++)
    {
      vps->setSubLayerDpbInfoPresentFlag( i, j, false );
    }
  }

#if !SUB_LAYERS_IN_LAYER_SET
#if BITRATE_PICRATE_SIGNALLING
  if( MaxSubLayersInLayerSetMinus1 )
  {
    delete [] MaxSubLayersInLayerSetMinus1;
  }
#endif
#endif

  // Infer values when not signalled
  for(Int i = 1; i < (Int)vps->getNumOutputLayerSets(); i++)
  {
    Int layerSetIdxForOutputLayerSet = vps->getOutputLayerSetIdx( i );
    for(Int j = 0; j < MAX_TLAYER; j++)
    {
      if( !vps->getSubLayerDpbInfoPresentFlag(i, j) )  // If sub-layer DPB information is NOT present
      {
#if RESOLUTION_BASED_DPB
        for(Int k = 0; k < (Int)vps->getNumSubDpbs(layerSetIdxForOutputLayerSet); k++)
#else
        for(Int k = 0; k < (Int)vps->getNumLayersInIdList( layerSetIdxForOutputLayerSet ); k++)
#endif
        {
          vps->setMaxVpsDecPicBufferingMinus1( i, k, j, vps->getMaxVpsDecPicBufferingMinus1( i, k, j - 1 ) );
        }
        vps->setMaxVpsNumReorderPics( i, j, vps->getMaxVpsNumReorderPics( i, j - 1) );
#if RESOLUTION_BASED_DPB
        for(Int k = 0; k < (Int)vps->getNumLayersInIdList( layerSetIdxForOutputLayerSet ); k++)
        {
          vps->setMaxVpsLayerDecPicBuffMinus1( i, k, j, vps->getMaxVpsLayerDecPicBuffMinus1( i, k, j - 1));
        }
#endif
        vps->setMaxVpsLatencyIncreasePlus1( i, j, vps->getMaxVpsLatencyIncreasePlus1( i, j - 1 ) );
      }
    }
  }
}
#endif

Void TDecCavlc::parseVPSVUI(TComVPS *vps)
{
  UInt i,j;
  UInt uiCode;
#if O0223_PICTURE_TYPES_ALIGN_FLAG
  READ_FLAG(uiCode, "cross_layer_pic_type_aligned_flag" );
  vps->setCrossLayerPictureTypeAlignFlag(uiCode);
  if (!uiCode) 
  {
#endif
#if IRAP_ALIGN_FLAG_IN_VPS_VUI
    READ_FLAG(uiCode, "cross_layer_irap_aligned_flag" );
    vps->setCrossLayerIrapAlignFlag(uiCode);
#if P0068_CROSS_LAYER_ALIGNED_IDR_ONLY_FOR_IRAP_FLAG
    if( uiCode )
    {
      READ_FLAG( uiCode, "all_layers_idr_aligned_flag" );
      vps->setCrossLayerAlignedIdrOnlyFlag(uiCode);
    }
#endif
#endif
#if O0223_PICTURE_TYPES_ALIGN_FLAG
  }
  else
  {
    vps->setCrossLayerIrapAlignFlag(true);
  }
#endif

  READ_FLAG( uiCode,        "bit_rate_present_vps_flag" );  vps->setBitRatePresentVpsFlag( uiCode ? true : false );
  READ_FLAG( uiCode,        "pic_rate_present_vps_flag" );  vps->setPicRatePresentVpsFlag( uiCode ? true : false );

#if SIGNALLING_BITRATE_PICRATE_FIX
  if ( vps->getBitRatePresentVpsFlag() || vps->getPicRatePresentVpsFlag() )
  {
    for( i = vps->getBaseLayerInternalFlag() ? 0 : 1; i < vps->getNumLayerSets(); i++ )
    {
      for( j = 0; j <= vps->getMaxSLayersInLayerSetMinus1( i ); j++ ) 
      {
        if( vps->getBitRatePresentVpsFlag() )
        {
          READ_FLAG( uiCode, "bit_rate_present_flag[i][j]" ); vps->setBitRatePresentFlag( i, j, uiCode ? true : false );            
        }
        if( vps->getPicRatePresentVpsFlag( )  )
        {
          READ_FLAG( uiCode, "pic_rate_present_flag[i][j]" ); vps->setPicRatePresentFlag( i, j, uiCode ? true : false );
        }
        if( vps->getBitRatePresentFlag( i, j ) )
        {
          READ_CODE( 16, uiCode, "avg_bit_rate" ); vps->setAvgBitRate( i, j, uiCode );
          READ_CODE( 16, uiCode, "max_bit_rate" ); vps->setMaxBitRate( i, j, uiCode );
        }
        else
        {
          vps->setAvgBitRate( i, j, 0 );
          vps->setMaxBitRate( i, j, 0 );
        }
        if( vps->getPicRatePresentFlag( i, j ) )
        {
          READ_CODE( 2,  uiCode, "constant_pic_rate_idc" ); vps->setConstPicRateIdc( i, j, uiCode );
          READ_CODE( 16, uiCode, "avg_pic_rate" );          vps->setAvgPicRate( i, j, uiCode );
        }
        else
        {
          vps->setConstPicRateIdc( i, j, 0 );
          vps->setAvgPicRate( i, j, 0 );
        }
      }
    }
  }
#else
  Bool parseFlag = vps->getBitRatePresentVpsFlag() || vps->getPicRatePresentVpsFlag();

#if Q0078_ADD_LAYER_SETS
#if R0227_BR_PR_ADD_LAYER_SET
  for( i = 0; i < vps->getNumLayerSets(); i++ )
#else
  for( i = 0; i <= vps->getVpsNumLayerSetsMinus1(); i++ )
#endif
#else
  for( i = 0; i < vps->getNumLayerSets(); i++ )
#endif
  {
#if BITRATE_PICRATE_SIGNALLING
    for( j = 0; j <= vps->getMaxSLayersInLayerSetMinus1(i); j++ )
#else
    for( j = 0; j < vps->getMaxTLayers(); j++ )
#endif
    {
      if( parseFlag && vps->getBitRatePresentVpsFlag() )
      {
        READ_FLAG( uiCode,        "bit_rate_present_flag[i][j]" );  vps->setBitRatePresentFlag( i, j, uiCode ? true : false );
      }
      else
      {
        vps->setBitRatePresentFlag( i, j, false );
      }
      if( parseFlag && vps->getPicRatePresentVpsFlag() )
      {
        READ_FLAG( uiCode,        "pic_rate_present_flag[i][j]" );  vps->setPicRatePresentFlag( i, j, uiCode ? true : false );
      }
      else
      {
        vps->setPicRatePresentFlag( i, j, false );
      }
      if( parseFlag && vps->getBitRatePresentFlag(i, j) )
      {
        READ_CODE( 16, uiCode,    "avg_bit_rate[i][j]" ); vps->setAvgBitRate( i, j, uiCode );
        READ_CODE( 16, uiCode,    "max_bit_rate[i][j]" ); vps->setMaxBitRate( i, j, uiCode );
      }
      else
      {
        vps->setAvgBitRate( i, j, 0 );
        vps->setMaxBitRate( i, j, 0 );
      }
      if( parseFlag && vps->getPicRatePresentFlag(i, j) )
      {
        READ_CODE( 2 , uiCode,    "constant_pic_rate_idc[i][j]" ); vps->setConstPicRateIdc( i, j, uiCode );
        READ_CODE( 16, uiCode,    "avg_pic_rate[i][j]"          ); vps->setAvgPicRate( i, j, uiCode );
      }
      else
      {
        vps->setConstPicRateIdc( i, j, 0 );
        vps->setAvgPicRate     ( i, j, 0 );
      }
    }
  }
#endif
#if VPS_VUI_VIDEO_SIGNAL_MOVE
  READ_FLAG( uiCode, "video_signal_info_idx_present_flag" ); vps->setVideoSigPresentVpsFlag( uiCode == 1 );
  if (vps->getVideoSigPresentVpsFlag())
  {
    READ_CODE(4, uiCode, "vps_num_video_signal_info_minus1" ); vps->setNumVideoSignalInfo(uiCode + 1);
  }
  else
  {
#if VPS_VUI_VST_PARAMS
    vps->setNumVideoSignalInfo(vps->getMaxLayers() - vps->getBaseLayerInternalFlag() ? 0 : 1);
#else
    vps->setNumVideoSignalInfo(vps->getMaxLayers());
#endif
  }

  for(i = 0; i < (UInt)vps->getNumVideoSignalInfo(); i++)
  {
    READ_CODE(3, uiCode, "video_vps_format" ); vps->setVideoVPSFormat(i,uiCode);
    READ_FLAG(uiCode, "video_full_range_vps_flag" ); vps->setVideoFullRangeVpsFlag(i,uiCode);
    READ_CODE(8, uiCode, "color_primaries_vps" ); vps->setColorPrimaries(i,uiCode);
    READ_CODE(8, uiCode, "transfer_characteristics_vps" ); vps->setTransCharacter(i,uiCode);
    READ_CODE(8, uiCode, "matrix_coeffs_vps" );vps->setMaxtrixCoeff(i,uiCode);
  }
#if VPS_VUI_VST_PARAMS
  if( vps->getVideoSigPresentVpsFlag() && vps->getNumVideoSignalInfo() > 1 )
  {
    for(i = vps->getBaseLayerInternalFlag() ? 0 : 1; i < vps->getMaxLayers(); i++)
    {
      READ_CODE(4, uiCode, "vps_video_signal_info_idx" ); vps->setVideoSignalInfoIdx(i, uiCode);
    }
  }
  else if ( !vps->getVideoSigPresentVpsFlag() )
  {
    for(i = vps->getBaseLayerInternalFlag() ? 0 : 1; i < vps->getMaxLayers(); i++)
    {
      vps->setVideoSignalInfoIdx( i, i );
    }
  }
  else // ( vps->getNumVideoSignalInfo() = 0 )
  {
    for(i = vps->getBaseLayerInternalFlag() ? 0 : 1; i < vps->getMaxLayers(); i++)
    {
      vps->setVideoSignalInfoIdx( i, 0 );
    }
  }
#else
  if(!vps->getVideoSigPresentVpsFlag())
  {
    for (i=0; i < vps->getMaxLayers(); i++)
    {
      vps->setVideoSignalInfoIdx(i,i);
    }
  }
  else {
    vps->setVideoSignalInfoIdx(0,0);
    if (vps->getNumVideoSignalInfo() > 1 )
    {
      for (i=1; i < vps->getMaxLayers(); i++)
        READ_CODE(4, uiCode, "vps_video_signal_info_idx" ); vps->setVideoSignalInfoIdx(i, uiCode);
    }
    else {
      for (i=1; i < vps->getMaxLayers(); i++)
      {
        vps->setVideoSignalInfoIdx(i,0);
      }
    }
  }
#endif
#endif 
#if VPS_VUI_TILES_NOT_IN_USE__FLAG
  UInt layerIdx;
  READ_FLAG( uiCode, "tiles_not_in_use_flag" ); vps->setTilesNotInUseFlag(uiCode == 1);
  if (!uiCode)
  {
    for(i = 0; i < vps->getMaxLayers(); i++)
    {
      READ_FLAG( uiCode, "tiles_in_use_flag[ i ]" ); vps->setTilesInUseFlag(i, (uiCode == 1));
      if (uiCode)
      {
        READ_FLAG( uiCode, "loop_filter_not_across_tiles_flag[ i ]" ); vps->setLoopFilterNotAcrossTilesFlag(i, (uiCode == 1));
      }
      else
      {
        vps->setLoopFilterNotAcrossTilesFlag(i, false);
      }
    }
#endif

    for(i = 1; i < vps->getMaxLayers(); i++)
    {
      for(j = 0; j < vps->getNumDirectRefLayers(vps->getLayerIdInNuh(i)); j++)
      {
#if VPS_VUI_TILES_NOT_IN_USE__FLAG
        layerIdx = vps->getLayerIdInVps(vps->getRefLayerId(vps->getLayerIdInNuh(i), j));
        if (vps->getTilesInUseFlag(i) && vps->getTilesInUseFlag(layerIdx)) {
          READ_FLAG( uiCode, "tile_boundaries_aligned_flag[i][j]" ); vps->setTileBoundariesAlignedFlag(i,j,(uiCode == 1));
        }
#else
        READ_FLAG( uiCode, "tile_boundaries_aligned_flag[i][j]" ); vps->setTileBoundariesAlignedFlag(i,j,(uiCode == 1));
#endif
      }
    }
#if VPS_VUI_TILES_NOT_IN_USE__FLAG
  }
#endif
#if VPS_VUI_WPP_NOT_IN_USE__FLAG
  READ_FLAG( uiCode, "wpp_not_in_use_flag" ); vps->setWppNotInUseFlag(uiCode == 1);
  if (!uiCode)
  {
    for(i = 0; i < vps->getMaxLayers(); i++)
    {
      READ_FLAG( uiCode, "wpp_in_use_flag[ i ]" ); vps->setWppInUseFlag(i, (uiCode == 1));
    }
  }
#endif

#if O0109_O0199_FLAGS_TO_VUI
#if M0040_ADAPTIVE_RESOLUTION_CHANGE
  READ_FLAG(uiCode, "single_layer_for_non_irap_flag" ); vps->setSingleLayerForNonIrapFlag(uiCode == 1 ? true : false);
#endif
#if HIGHER_LAYER_IRAP_SKIP_FLAG
  READ_FLAG(uiCode, "higher_layer_irap_skip_flag" ); vps->setHigherLayerIrapSkipFlag(uiCode == 1 ? true : false);

  // When single_layer_for_non_irap_flag is equal to 0, higher_layer_irap_skip_flag shall be equal to 0
  if( !vps->getSingleLayerForNonIrapFlag() )
  {
    assert( !vps->getHigherLayerIrapSkipFlag() );
  }
#endif
#endif
#if P0312_VERT_PHASE_ADJ
  READ_FLAG( uiCode, "vps_vui_vert_phase_in_use_flag" ); vps->setVpsVuiVertPhaseInUseFlag(uiCode);
#endif
#if N0160_VUI_EXT_ILP_REF
  READ_FLAG( uiCode, "ilp_restricted_ref_layers_flag" ); vps->setIlpRestrictedRefLayersFlag( uiCode == 1 );
  if( vps->getIlpRestrictedRefLayersFlag())
  {
    for(i = 1; i < vps->getMaxLayers(); i++)
    {
      for(j = 0; j < vps->getNumDirectRefLayers(vps->getLayerIdInNuh(i)); j++)
      {
        READ_UVLC( uiCode, "min_spatial_segment_offset_plus1[i][j]" ); vps->setMinSpatialSegmentOffsetPlus1( i, j, uiCode );
        if( vps->getMinSpatialSegmentOffsetPlus1(i,j ) > 0 )
        {
          READ_FLAG( uiCode, "ctu_based_offset_enabled_flag[i][j]"); vps->setCtuBasedOffsetEnabledFlag(i, j, uiCode == 1 );
          if(vps->getCtuBasedOffsetEnabledFlag(i,j))
          {
            READ_UVLC( uiCode, "min_horizontal_ctu_offset_plus1[i][j]"); vps->setMinHorizontalCtuOffsetPlus1( i,j, uiCode );
          }
        }
      }
    }
  }
#endif
#if VPS_VUI_VIDEO_SIGNAL
#if VPS_VUI_VIDEO_SIGNAL_MOVE
#else
  READ_FLAG( uiCode, "video_signal_info_idx_present_flag" ); vps->setVideoSigPresentVpsFlag( uiCode == 1 );
  if (vps->getVideoSigPresentVpsFlag())
  {
    READ_CODE(4, uiCode, "vps_num_video_signal_info_minus1" ); vps->setNumVideoSignalInfo(uiCode + 1);
  }
  else
  {
    vps->setNumVideoSignalInfo(vps->getMaxLayers());
  }


  for(i = 0; i < vps->getNumVideoSignalInfo(); i++)
  {
    READ_CODE(3, uiCode, "video_vps_format" ); vps->setVideoVPSFormat(i,uiCode);
    READ_FLAG(uiCode, "video_full_range_vps_flag" ); vps->setVideoFullRangeVpsFlag(i,uiCode);
    READ_CODE(8, uiCode, "color_primaries_vps" ); vps->setColorPrimaries(i,uiCode);
    READ_CODE(8, uiCode, "transfer_characteristics_vps" ); vps->setTransCharacter(i,uiCode);
    READ_CODE(8, uiCode, "matrix_coeffs_vps" );vps->setMaxtrixCoeff(i,uiCode);
  }
  if(!vps->getVideoSigPresentVpsFlag())
  {
    for (i=0; i < vps->getMaxLayers(); i++)
    {
      vps->setVideoSignalInfoIdx(i,i);
    }
  }
  else {
    vps->setVideoSignalInfoIdx(0,0);
    if (vps->getNumVideoSignalInfo() > 1 )
    {
      for (i=1; i < vps->getMaxLayers(); i++)
        READ_CODE(4, uiCode, "vps_video_signal_info_idx" ); vps->setVideoSignalInfoIdx(i, uiCode);
    }
    else {
      for (i=1; i < vps->getMaxLayers(); i++)
      {
        vps->setVideoSignalInfoIdx(i,0);
      }
    }
  }
#endif 
#endif

#if O0164_MULTI_LAYER_HRD
  READ_FLAG(uiCode, "vps_vui_bsp_hrd_present_flag" ); vps->setVpsVuiBspHrdPresentFlag(uiCode);
  if (vps->getVpsVuiBspHrdPresentFlag())
  {
#if VPS_VUI_BSP_HRD_PARAMS
    parseVpsVuiBspHrdParams(vps);
#else
#if R0227_VUI_BSP_HRD_FLAG
    assert (vps->getTimingInfo()->getTimingInfoPresentFlag() == 1);
#endif
    READ_UVLC( uiCode, "vps_num_bsp_hrd_parameters_minus1" ); vps->setVpsNumBspHrdParametersMinus1(uiCode);
    vps->createBspHrdParamBuffer(vps->getVpsNumBspHrdParametersMinus1() + 1);
    for( i = 0; i <= vps->getVpsNumBspHrdParametersMinus1(); i++ )
    {
      if( i > 0 )
      {
        READ_FLAG( uiCode, "bsp_cprms_present_flag[i]" ); vps->setBspCprmsPresentFlag(i, uiCode);
      }
      parseHrdParameters(vps->getBspHrd(i), i==0 ? 1 : vps->getBspCprmsPresentFlag(i), vps->getMaxTLayers()-1);
    }
#if Q0078_ADD_LAYER_SETS
    for (UInt h = 1; h <= vps->getVpsNumLayerSetsMinus1(); h++)
#else
    for( UInt h = 1; h <= (vps->getNumLayerSets()-1); h++ )
#endif
    {
      READ_UVLC( uiCode, "num_bitstream_partitions[i]"); vps->setNumBitstreamPartitions(h, uiCode);
#if HRD_BPB
      Int chkPart=0;
#endif
      for( i = 0; i < vps->getNumBitstreamPartitions(h); i++ )
      {
        for( j = 0; j <= (vps->getMaxLayers()-1); j++ )
        {
          if( vps->getLayerIdIncludedFlag(h, j) )
          {
            READ_FLAG( uiCode, "layer_in_bsp_flag[h][i][j]" ); vps->setLayerInBspFlag(h, i, j, uiCode);
          }
        }
#if HRD_BPB
        chkPart+=vps->getLayerInBspFlag(h, i, j);
#endif
      }
#if HRD_BPB
      assert(chkPart<=1);
#endif
#if HRD_BPB
      if(vps->getNumBitstreamPartitions(h)==1)
      {
        Int chkPartition1=0; Int chkPartition2=0;
        for( j = 0; j <= (vps->getMaxLayers()-1); j++ )
        {
          if( vps->getLayerIdIncludedFlag(h, j) )
          {
            chkPartition1+=vps->getLayerInBspFlag(h, 0, j);
            chkPartition2++;
          }
        }
        assert(chkPartition1!=chkPartition2);
      }
#endif
      if (vps->getNumBitstreamPartitions(h))
      {
#if Q0182_MULTI_LAYER_HRD_UPDATE
        READ_UVLC( uiCode, "num_bsp_sched_combinations_minus1[h]"); vps->setNumBspSchedCombinations(h, uiCode + 1);
#else
        READ_UVLC( uiCode, "num_bsp_sched_combinations[h]"); vps->setNumBspSchedCombinations(h, uiCode);
#endif
        for( i = 0; i < vps->getNumBspSchedCombinations(h); i++ )
        {
          for( j = 0; j < vps->getNumBitstreamPartitions(h); j++ )
          {
            READ_UVLC( uiCode, "bsp_comb_hrd_idx[h][i][j]"); vps->setBspCombHrdIdx(h, i, j, uiCode);
#if HRD_BPB
            assert(uiCode <= vps->getVpsNumBspHrdParametersMinus1());
#endif

            READ_UVLC( uiCode, "bsp_comb_sched_idx[h][i][j]"); vps->setBspCombSchedIdx(h, i, j, uiCode);
#if HRD_BPB
            assert(uiCode <= vps->getBspHrdParamBufferCpbCntMinus1(uiCode,vps->getMaxTLayers()-1));
#endif
          }
        }
      }
    }
#endif
  }
#endif
#if P0182_VPS_VUI_PS_FLAG
  for(i = 1; i < vps->getMaxLayers(); i++)
  {
    if (vps->getNumRefLayers(vps->getLayerIdInNuh(i)) == 0)
    {
      READ_FLAG( uiCode, "base_layer_parameter_set_compatibility_flag" ); 
      vps->setBaseLayerPSCompatibilityFlag( i, uiCode );
    }
    else
    {
      vps->setBaseLayerPSCompatibilityFlag( i, 0 );
    }
  }
#endif
}

#endif //SVC_EXTENSION

Void TDecCavlc::parseSliceHeader ( TComSlice*& rpcSlice, ParameterSetManagerDecoder *parameterSetManager)
{
  UInt  uiCode;
  Int   iCode;
#if TRACE_DEBUG
  printf("PARSE SLICE HEADER\n");
#endif
#if ENC_DEC_TRACE
  xTraceSliceHeader(rpcSlice);
#endif
  TComPPS* pps = NULL;
  TComSPS* sps = NULL;

  UInt firstSliceSegmentInPic;
  READ_FLAG( firstSliceSegmentInPic, "first_slice_segment_in_pic_flag" );
  if( rpcSlice->getRapPicFlag())
  {
    READ_FLAG( uiCode, "no_output_of_prior_pics_flag" );  //ignored -- updated already
#if SETTING_NO_OUT_PIC_PRIOR
    rpcSlice->setNoOutputPriorPicsFlag(uiCode ? true : false);
#else
    rpcSlice->setNoOutputPicPrior( false );
#endif
  }
  READ_UVLC (    uiCode, "slice_pic_parameter_set_id" );  rpcSlice->setPPSId(uiCode);
  // pps = parameterSetManager->getPrefetchedPPS(uiCode);

  pps = m_pPPS + uiCode;
  //!KS: need to add error handling code here, if PPS is not available
  assert(pps!=0);
  // sps = parameterSetManager->getPrefetchedSPS(pps->getSPSId());
  sps = m_pSPS + pps->getSPSId();
  //!KS: need to add error handling code here, if SPS is not available
  assert(sps!=0);
  rpcSlice->setSPS(sps);
  rpcSlice->setPPS(pps);


#if R0227_REP_FORMAT_CONSTRAINT //Conformance checking for rep format -- rep format of current picture of current layer shall never be greater rep format defined in VPS for the current layer
  TComVPS* vps = NULL;
  // vps = parameterSetManager->getPrefetchedVPS(sps->getVPSId());
  vps = m_pVPS + sps->getVPSId();
  rpcSlice->setVPS(vps);

#if R0279_REP_FORMAT_INBL
  if ( vps->getVpsExtensionFlag() == 1 && (rpcSlice->getLayerId() == 0 || sps->getV1CompatibleSPSFlag() == 1) )
  {
    assert( (Int)sps->getPicWidthInLumaSamples() <= vps->getVpsRepFormat( vps->getVpsRepFormatIdx(rpcSlice->getLayerId()) )->getPicWidthVpsInLumaSamples() );
    assert( (Int)sps->getPicHeightInLumaSamples() <= vps->getVpsRepFormat( vps->getVpsRepFormatIdx(rpcSlice->getLayerId()) )->getPicHeightVpsInLumaSamples() );
    assert( (Int)sps->getChromaFormatIdc() <= vps->getVpsRepFormat( vps->getVpsRepFormatIdx(rpcSlice->getLayerId()) )->getChromaFormatVpsIdc() );
    assert( (Int)sps->getBitDepthY() <= vps->getVpsRepFormat( vps->getVpsRepFormatIdx(rpcSlice->getLayerId()) )->getBitDepthVpsLuma() );
    assert( (Int)sps->getBitDepthC() <= vps->getVpsRepFormat( vps->getVpsRepFormatIdx(rpcSlice->getLayerId()) )->getBitDepthVpsChroma() );
#else
  if ( rpcSlice->getLayerId() == 0 && vps->getVpsExtensionFlag() == 1 )
  {
    assert( sps->getPicWidthInLumaSamples() <= vps->getVpsRepFormat( vps->getVpsRepFormatIdx(0) )->getPicWidthVpsInLumaSamples() );
    assert( sps->getPicHeightInLumaSamples() <= vps->getVpsRepFormat( vps->getVpsRepFormatIdx(0) )->getPicHeightVpsInLumaSamples() );
    assert( sps->getChromaFormatIdc() <= vps->getVpsRepFormat( vps->getVpsRepFormatIdx(0) )->getChromaFormatVpsIdc() );
    assert( sps->getBitDepthY() <= vps->getVpsRepFormat( vps->getVpsRepFormatIdx(0) )->getBitDepthVpsLuma() );
    assert( sps->getBitDepthC() <= vps->getVpsRepFormat( vps->getVpsRepFormatIdx(0) )->getBitDepthVpsChroma() );
#endif
  }
  else if ( vps->getVpsExtensionFlag() == 1 )
  {
    assert(vps->getVpsRepFormat( vps->getVpsRepFormatIdx(sps->getUpdateRepFormatFlag() ? sps->getUpdateRepFormatIndex() : rpcSlice->getLayerId()))->getPicWidthVpsInLumaSamples() <= vps->getVpsRepFormat( vps->getVpsRepFormatIdx(rpcSlice->getLayerId()))->getPicWidthVpsInLumaSamples());
    assert(vps->getVpsRepFormat( vps->getVpsRepFormatIdx(sps->getUpdateRepFormatFlag() ? sps->getUpdateRepFormatIndex() : rpcSlice->getLayerId()))->getPicHeightVpsInLumaSamples() <= vps->getVpsRepFormat( vps->getVpsRepFormatIdx(rpcSlice->getLayerId()))->getPicHeightVpsInLumaSamples());
    assert(vps->getVpsRepFormat( vps->getVpsRepFormatIdx(sps->getUpdateRepFormatFlag() ? sps->getUpdateRepFormatIndex() : rpcSlice->getLayerId()))->getChromaFormatVpsIdc() <= vps->getVpsRepFormat( vps->getVpsRepFormatIdx(rpcSlice->getLayerId()))->getChromaFormatVpsIdc());
    assert(vps->getVpsRepFormat( vps->getVpsRepFormatIdx(sps->getUpdateRepFormatFlag() ? sps->getUpdateRepFormatIndex() : rpcSlice->getLayerId()))->getBitDepthVpsLuma() <= vps->getVpsRepFormat( vps->getVpsRepFormatIdx(rpcSlice->getLayerId()))->getBitDepthVpsLuma());
    assert(vps->getVpsRepFormat( vps->getVpsRepFormatIdx(sps->getUpdateRepFormatFlag() ? sps->getUpdateRepFormatIndex() : rpcSlice->getLayerId()))->getBitDepthVpsChroma() <= vps->getVpsRepFormat( vps->getVpsRepFormatIdx(rpcSlice->getLayerId()))->getBitDepthVpsChroma());
  }
#endif

  if( pps->getDependentSliceSegmentsEnabledFlag() && ( !firstSliceSegmentInPic ))
  {
    READ_FLAG( uiCode, "dependent_slice_segment_flag" );       rpcSlice->setDependentSliceSegmentFlag(uiCode ? true : false);
  }
  else
  {
    rpcSlice->setDependentSliceSegmentFlag(false);
  }
#if REPN_FORMAT_IN_VPS
   Int numCTUs = ((rpcSlice->getPicWidthInLumaSamples()+sps->getMaxCUWidth()-1)/sps->getMaxCUWidth())*((rpcSlice->getPicHeightInLumaSamples()+sps->getMaxCUHeight()-1)/sps->getMaxCUHeight());
#else
  Int numCTUs = ((sps->getPicWidthInLumaSamples()+sps->getMaxCUWidth()-1)/sps->getMaxCUWidth())*((sps->getPicHeightInLumaSamples()+sps->getMaxCUHeight()-1)/sps->getMaxCUHeight());
#endif
  Int maxParts = (1<<(sps->getMaxCUDepth()<<1));
  UInt sliceSegmentAddress = 0;
  Int bitsSliceSegmentAddress = 0;
  while(numCTUs>(1<<bitsSliceSegmentAddress))
  {
    bitsSliceSegmentAddress++;
  }

  if(!firstSliceSegmentInPic)
  {
    READ_CODE( bitsSliceSegmentAddress, sliceSegmentAddress, "slice_segment_address" );
  }
  //set uiCode to equal slice start address (or dependent slice start address)
  Int startCuAddress = maxParts*sliceSegmentAddress;
  rpcSlice->setSliceSegmentCurStartCUAddr( startCuAddress );
  rpcSlice->setSliceSegmentCurEndCUAddr(numCTUs*maxParts);

  if (rpcSlice->getDependentSliceSegmentFlag())
  {
    rpcSlice->setNextSlice          ( false );
    rpcSlice->setNextSliceSegment ( true  );
  }
  else
  {
    rpcSlice->setNextSlice          ( true  );
    rpcSlice->setNextSliceSegment ( false );

    rpcSlice->setSliceCurStartCUAddr(startCuAddress);
    rpcSlice->setSliceCurEndCUAddr(numCTUs*maxParts);
  }

#if Q0142_POC_LSB_NOT_PRESENT
#if SHM_FIX7
  Int iPOClsb = 0;
#endif
#endif

  if(!rpcSlice->getDependentSliceSegmentFlag())
  {
#if SVC_EXTENSION
#if POC_RESET_FLAG
    Int iBits = 0;
    if(rpcSlice->getPPS()->getNumExtraSliceHeaderBits() > iBits)
    {
      READ_FLAG(uiCode, "poc_reset_flag");      rpcSlice->setPocResetFlag( uiCode ? true : false );
      iBits++;
    }
    if(rpcSlice->getPPS()->getNumExtraSliceHeaderBits() > iBits)
    {
#if DISCARDABLE_PIC_RPS
      READ_FLAG(uiCode, "discardable_flag"); rpcSlice->setDiscardableFlag( uiCode ? true : false );
#else
      READ_FLAG(uiCode, "discardable_flag"); // ignored
#endif
      iBits++;
    }
#if O0149_CROSS_LAYER_BLA_FLAG
    if(rpcSlice->getPPS()->getNumExtraSliceHeaderBits() > iBits)
    {
      READ_FLAG(uiCode, "cross_layer_bla_flag");  rpcSlice->setCrossLayerBLAFlag( uiCode ? true : false );
      iBits++;
    }
#endif
    for (; iBits < rpcSlice->getPPS()->getNumExtraSliceHeaderBits(); iBits++)
    {
      READ_FLAG(uiCode, "slice_reserved_undetermined_flag[]"); // ignored
    }
#else
#if CROSS_LAYER_BLA_FLAG_FIX
    Int iBits = 0;
    if(rpcSlice->getPPS()->getNumExtraSliceHeaderBits() > iBits)
#else
    if(rpcSlice->getPPS()->getNumExtraSliceHeaderBits()>0)
#endif
    {
      READ_FLAG(uiCode, "discardable_flag"); // ignored
#if NON_REF_NAL_TYPE_DISCARDABLE
      rpcSlice->setDiscardableFlag( uiCode ? true : false );
      if (uiCode)
      {
        assert(rpcSlice->getNalUnitType() != NAL_UNIT_CODED_SLICE_TRAIL_R &&
          rpcSlice->getNalUnitType() != NAL_UNIT_CODED_SLICE_TSA_R &&
          rpcSlice->getNalUnitType() != NAL_UNIT_CODED_SLICE_STSA_R &&
          rpcSlice->getNalUnitType() != NAL_UNIT_CODED_SLICE_RADL_R &&
          rpcSlice->getNalUnitType() != NAL_UNIT_CODED_SLICE_RASL_R);
      }
#endif
#if CROSS_LAYER_BLA_FLAG_FIX
      iBits++;
#endif
    }
#if CROSS_LAYER_BLA_FLAG_FIX
    if(rpcSlice->getPPS()->getNumExtraSliceHeaderBits() > iBits)
    {
      READ_FLAG(uiCode, "cross_layer_bla_flag");  rpcSlice->setCrossLayerBLAFlag( uiCode ? true : false );
      iBits++;
    }
    for ( ; iBits < rpcSlice->getPPS()->getNumExtraSliceHeaderBits(); iBits++)
#else
    for (Int i = 1; i < rpcSlice->getPPS()->getNumExtraSliceHeaderBits(); i++)
#endif
    {
      READ_FLAG(uiCode, "slice_reserved_undetermined_flag[]"); // ignored
    }
#endif
#else //SVC_EXTENSION
    for (Int i = 0; i < rpcSlice->getPPS()->getNumExtraSliceHeaderBits(); i++)
    {
      READ_FLAG(uiCode, "slice_reserved_undetermined_flag[]"); // ignored
    }
#endif //SVC_EXTENSION

    READ_UVLC (    uiCode, "slice_type" );            rpcSlice->setSliceType((SliceType)uiCode);
    if( pps->getOutputFlagPresentFlag() )
    {
      READ_FLAG( uiCode, "pic_output_flag" );    rpcSlice->setPicOutputFlag( uiCode ? true : false );
    }
    else
    {
      rpcSlice->setPicOutputFlag( true );
    }
    // in the first version chroma_format_idc is equal to one, thus colour_plane_id will not be present
    assert (sps->getChromaFormatIdc() == 1 );
    // if( separate_colour_plane_flag  ==  1 )
    //   colour_plane_id                                      u(2)

    if( rpcSlice->getIdrPicFlag() )
    {
      rpcSlice->setPOC(0);
      TComReferencePictureSet* rps = rpcSlice->getLocalRPS();
      rps->setNumberOfNegativePictures(0);
      rps->setNumberOfPositivePictures(0);
      rps->setNumberOfLongtermPictures(0);
      rps->setNumberOfPictures(0);
      rpcSlice->setRPS(rps);
    }
#if N0065_LAYER_POC_ALIGNMENT
#if !Q0142_POC_LSB_NOT_PRESENT
#if SHM_FIX7
    Int iPOClsb = 0;
#endif
#endif
#if O0062_POC_LSB_NOT_PRESENT_FLAG
    if( ( rpcSlice->getLayerId() > 0 && !rpcSlice->getVPS()->getPocLsbNotPresentFlag( rpcSlice->getVPS()->getLayerIdInVps(rpcSlice->getLayerId())) ) || !rpcSlice->getIdrPicFlag())
#else
    if( rpcSlice->getLayerId() > 0 || !rpcSlice->getIdrPicFlag() )
#endif
#else
    else
#endif
    {
      READ_CODE(sps->getBitsForPOC(), uiCode, "pic_order_cnt_lsb");
#if POC_RESET_IDC_DECODER
      rpcSlice->setPicOrderCntLsb( uiCode );
#endif
#if SHM_FIX7
      iPOClsb = uiCode;
#else
      Int iPOClsb = uiCode;
#endif
      Int iPrevPOC = rpcSlice->getPrevTid0POC();
      Int iMaxPOClsb = 1<< sps->getBitsForPOC();
      Int iPrevPOClsb = iPrevPOC & (iMaxPOClsb - 1);
      Int iPrevPOCmsb = iPrevPOC-iPrevPOClsb;
      Int iPOCmsb;
      if( ( iPOClsb  <  iPrevPOClsb ) && ( ( iPrevPOClsb - iPOClsb )  >=  ( iMaxPOClsb / 2 ) ) )
      {
        iPOCmsb = iPrevPOCmsb + iMaxPOClsb;
      }
      else if( (iPOClsb  >  iPrevPOClsb )  && ( (iPOClsb - iPrevPOClsb )  >  ( iMaxPOClsb / 2 ) ) )
      {
        iPOCmsb = iPrevPOCmsb - iMaxPOClsb;
      }
      else
      {
        iPOCmsb = iPrevPOCmsb;
      }
      if ( rpcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
        || rpcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
        || rpcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP )
      {
        // For BLA picture types, POCmsb is set to 0.
        iPOCmsb = 0;
      }
      rpcSlice->setPOC              (iPOCmsb+iPOClsb);

#if N0065_LAYER_POC_ALIGNMENT
#if SHM_FIX7
    }
#endif
#if POC_RESET_IDC_DECODER
  else
  {
    rpcSlice->setPicOrderCntLsb( 0 );
  }
#endif
  return;  // return after POC TCH-JR

  if( !rpcSlice->getIdrPicFlag() )
  {
#endif
    TComReferencePictureSet* rps;
    rps = rpcSlice->getLocalRPS();
    rpcSlice->setRPS(rps);
    READ_FLAG( uiCode, "short_term_ref_pic_set_sps_flag" );
    if(uiCode == 0) // use short-term reference picture set explicitly signalled in slice header
    {
      parseShortTermRefPicSet(sps,rps, sps->getRPSList()->getNumberOfReferencePictureSets());
    }
    else // use reference to short-term reference picture set in PPS
    {
      Int numBits = 0;
      while ((1 << numBits) < rpcSlice->getSPS()->getRPSList()->getNumberOfReferencePictureSets())
      {
        numBits++;
      }
      if (numBits > 0)
      {
        READ_CODE( numBits, uiCode, "short_term_ref_pic_set_idx");
      }
      else
      {
        uiCode = 0;       
      }
      *rps = *(sps->getRPSList()->getReferencePictureSet(uiCode));
    }
    if(sps->getLongTermRefsPresent())
    {
      Int offset = rps->getNumberOfNegativePictures()+rps->getNumberOfPositivePictures();
      UInt numOfLtrp = 0;
      UInt numLtrpInSPS = 0;
      if (rpcSlice->getSPS()->getNumLongTermRefPicSPS() > 0)
      {
        READ_UVLC( uiCode, "num_long_term_sps");
        numLtrpInSPS = uiCode;
        numOfLtrp += numLtrpInSPS;
        rps->setNumberOfLongtermPictures(numOfLtrp);
      }
      Int bitsForLtrpInSPS = 0;
      while (rpcSlice->getSPS()->getNumLongTermRefPicSPS() > (1 << bitsForLtrpInSPS))
      {
        bitsForLtrpInSPS++;
      }
      READ_UVLC( uiCode, "num_long_term_pics");             rps->setNumberOfLongtermPictures(uiCode);
      numOfLtrp += uiCode;
      rps->setNumberOfLongtermPictures(numOfLtrp);
      Int maxPicOrderCntLSB = 1 << rpcSlice->getSPS()->getBitsForPOC();
      Int prevDeltaMSB = 0, deltaPocMSBCycleLT = 0;;
      for(Int j= offset+rps->getNumberOfLongtermPictures()-1, k = 0; k < (Int)numOfLtrp; j--, k++)
      {
        Int pocLsbLt;
        if (k < (Int)numLtrpInSPS)
        {
          uiCode = 0;
          if (bitsForLtrpInSPS > 0)
          {
            READ_CODE(bitsForLtrpInSPS, uiCode, "lt_idx_sps[i]");
          }
          Int usedByCurrFromSPS=rpcSlice->getSPS()->getUsedByCurrPicLtSPSFlag(uiCode);

          pocLsbLt = rpcSlice->getSPS()->getLtRefPicPocLsbSps(uiCode);
          rps->setUsed(j,usedByCurrFromSPS);
        }
        else
        {
          READ_CODE(rpcSlice->getSPS()->getBitsForPOC(), uiCode, "poc_lsb_lt"); pocLsbLt= uiCode;
          READ_FLAG( uiCode, "used_by_curr_pic_lt_flag");     rps->setUsed(j,uiCode);
        }
        READ_FLAG(uiCode,"delta_poc_msb_present_flag");
        Bool mSBPresentFlag = uiCode ? true : false;
        if(mSBPresentFlag)
        {
          READ_UVLC( uiCode, "delta_poc_msb_cycle_lt[i]" );
          Bool deltaFlag = false;
          //            First LTRP                               || First LTRP from SH
          if( (j == offset+rps->getNumberOfLongtermPictures()-1) || (j == offset+(Int)(numOfLtrp-numLtrpInSPS)-1) )
          {
            deltaFlag = true;
          }
          if(deltaFlag)
          {
            deltaPocMSBCycleLT = uiCode;
          }
          else
          {
            deltaPocMSBCycleLT = uiCode + prevDeltaMSB;
          }

          Int pocLTCurr = rpcSlice->getPOC() - deltaPocMSBCycleLT * maxPicOrderCntLSB
            - iPOClsb + pocLsbLt;
          rps->setPOC     (j, pocLTCurr);
          rps->setDeltaPOC(j, - rpcSlice->getPOC() + pocLTCurr);
          rps->setCheckLTMSBPresent(j,true);
        }
        else
        {
          rps->setPOC     (j, pocLsbLt);
          rps->setDeltaPOC(j, - rpcSlice->getPOC() + pocLsbLt);
          rps->setCheckLTMSBPresent(j,false);

          // reset deltaPocMSBCycleLT for first LTRP from slice header if MSB not present
          if( j == offset+(Int)(numOfLtrp-numLtrpInSPS)-1 )
          {
            deltaPocMSBCycleLT = 0;
          }
        }
        prevDeltaMSB = deltaPocMSBCycleLT;
      }
      offset += rps->getNumberOfLongtermPictures();
      rps->setNumberOfPictures(offset);
    }
#if DPB_CONSTRAINTS
    if(rpcSlice->getVPS()->getVpsExtensionFlag()==1)
    {
#if Q0078_ADD_LAYER_SETS
      for (Int ii = 1; ii < (rpcSlice->getVPS()->getVpsNumLayerSetsMinus1() + 1); ii++)  // prevent assert error when num_add_layer_sets > 0
#else
      for (Int ii=1; ii< rpcSlice->getVPS()->getNumOutputLayerSets(); ii++ )
#endif
      {
        Int layerSetIdxForOutputLayerSet = rpcSlice->getVPS()->getOutputLayerSetIdx( ii );
        Int chkAssert=0;
        for(Int kk = 0; kk < rpcSlice->getVPS()->getNumLayersInIdList(layerSetIdxForOutputLayerSet); kk++)
        {
          if((Int)rpcSlice->getLayerId()==rpcSlice->getVPS()->getLayerSetLayerIdList(layerSetIdxForOutputLayerSet, kk))
          {
            chkAssert=1;
          }
        }
        if(chkAssert)
        {
          // There may be something wrong here (layer id assumed to be layer idx?)
          assert(rps->getNumberOfNegativePictures() <= rpcSlice->getVPS()->getMaxVpsDecPicBufferingMinus1(ii , rpcSlice->getLayerId() , rpcSlice->getVPS()->getMaxSLayersInLayerSetMinus1(ii)));
          assert(rps->getNumberOfPositivePictures() <= rpcSlice->getVPS()->getMaxVpsDecPicBufferingMinus1(ii , rpcSlice->getLayerId() , rpcSlice->getVPS()->getMaxSLayersInLayerSetMinus1(ii)) - rps->getNumberOfNegativePictures());
          assert((rps->getNumberOfPositivePictures() + rps->getNumberOfNegativePictures() + rps->getNumberOfLongtermPictures()) <= rpcSlice->getVPS()->getMaxVpsDecPicBufferingMinus1(ii , rpcSlice->getLayerId() , rpcSlice->getVPS()->getMaxSLayersInLayerSetMinus1(ii)));
        }
      }


    }
    if(rpcSlice->getLayerId() == 0)
    {
      assert((Int)rps->getNumberOfNegativePictures() <= (Int)rpcSlice->getSPS()->getMaxDecPicBuffering(rpcSlice->getSPS()->getMaxTLayers()-1) );
      assert((Int)rps->getNumberOfPositivePictures() <= (Int)rpcSlice->getSPS()->getMaxDecPicBuffering(rpcSlice->getSPS()->getMaxTLayers()-1) - (Int)rps->getNumberOfNegativePictures());
      assert((Int)(rps->getNumberOfPositivePictures() + rps->getNumberOfNegativePictures() + rps->getNumberOfLongtermPictures()) <= rpcSlice->getSPS()->getMaxDecPicBuffering(rpcSlice->getSPS()->getMaxTLayers()-1));
    }
#endif
    if ( rpcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
      || rpcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
      || rpcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP )
    {
      // In the case of BLA picture types, rps data is read from slice header but ignored
      rps = rpcSlice->getLocalRPS();
      rps->setNumberOfNegativePictures(0);
      rps->setNumberOfPositivePictures(0);
      rps->setNumberOfLongtermPictures(0);
      rps->setNumberOfPictures(0);
      rpcSlice->setRPS(rps);
    }
    if (rpcSlice->getSPS()->getTMVPFlagsPresent())
    {
#if R0226_SLICE_TMVP
      READ_FLAG( uiCode, "slice_temporal_mvp_enabled_flag" );
#else
      READ_FLAG( uiCode, "slice_temporal_mvp_enable_flag" );
#endif
      rpcSlice->setEnableTMVPFlag( uiCode == 1 ? true : false );
    }
    else
    {
      rpcSlice->setEnableTMVPFlag(false);
    }
#if N0065_LAYER_POC_ALIGNMENT && !SHM_FIX7
  }
#endif
  }

#if SVC_EXTENSION
  rpcSlice->setActiveNumILRRefIdx(0);
  if((rpcSlice->getLayerId() > 0) && !(rpcSlice->getVPS()->getIlpSshSignalingEnabledFlag()) && (rpcSlice->getNumILRRefIdx() > 0) )
  {
    READ_FLAG(uiCode,"inter_layer_pred_enabled_flag");
    rpcSlice->setInterLayerPredEnabledFlag(uiCode);
    if( rpcSlice->getInterLayerPredEnabledFlag())
    {
      if(rpcSlice->getNumILRRefIdx() > 1)
      {
        Int numBits = 1;
        while ((1 << numBits) < rpcSlice->getNumILRRefIdx())
        {
          numBits++;
        }
        if( !rpcSlice->getVPS()->getMaxOneActiveRefLayerFlag())
        {
          READ_CODE( numBits, uiCode,"num_inter_layer_ref_pics_minus1" );
          rpcSlice->setActiveNumILRRefIdx(uiCode + 1);
        }
        else
        {
#if P0079_DERIVE_NUMACTIVE_REF_PICS
          for( Int i = 0; i < rpcSlice->getNumILRRefIdx(); i++ ) 
          {
#if Q0060_MAX_TID_REF_EQUAL_TO_ZERO
            if((rpcSlice->getVPS()->getMaxTidIlRefPicsPlus1(rpcSlice->getVPS()->getLayerIdInVps(i),rpcSlice->getLayerId()) >  rpcSlice->getTLayer() || rpcSlice->getTLayer()==0) &&
              (rpcSlice->getVPS()->getMaxTSLayersMinus1(rpcSlice->getVPS()->getLayerIdInVps(i)) >=  rpcSlice->getTLayer()) )
#else 
            if(rpcSlice->getVPS()->getMaxTidIlRefPicsPlus1(rpcSlice->getVPS()->getLayerIdInVps(i),rpcSlice->getLayerId()) >  rpcSlice->getTLayer() &&
              (rpcSlice->getVPS()->getMaxTSLayersMinus1(rpcSlice->getVPS()->getLayerIdInVps(i)) >=  rpcSlice->getTLayer()) )
#endif 
            {          
              rpcSlice->setActiveNumILRRefIdx(1);
              break;
            }
          }
#else
          rpcSlice->setActiveNumILRRefIdx(1);
#endif 
        }

        if( rpcSlice->getActiveNumILRRefIdx() == rpcSlice->getNumILRRefIdx() )
        {
          for( Int i = 0; i < rpcSlice->getActiveNumILRRefIdx(); i++ )
          {
            rpcSlice->setInterLayerPredLayerIdc(i,i);
          }
        }
        else
        {
          for(Int i = 0; i < rpcSlice->getActiveNumILRRefIdx(); i++ )
          {
            READ_CODE( numBits,uiCode,"inter_layer_pred_layer_idc[i]" );
            rpcSlice->setInterLayerPredLayerIdc(uiCode,i);
          }
        }
      }
      else
      {
#if O0225_TID_BASED_IL_RPS_DERIV && TSLAYERS_IL_RPS
#if Q0060_MAX_TID_REF_EQUAL_TO_ZERO
        if((rpcSlice->getVPS()->getMaxTidIlRefPicsPlus1(0,rpcSlice->getLayerId()) >  rpcSlice->getTLayer() || rpcSlice->getTLayer()==0) &&
          (rpcSlice->getVPS()->getMaxTSLayersMinus1(0) >=  rpcSlice->getTLayer()) )
#else
        if( (rpcSlice->getVPS()->getMaxTidIlRefPicsPlus1(0,rpcSlice->getLayerId()) >  rpcSlice->getTLayer()) &&
          (rpcSlice->getVPS()->getMaxTSLayersMinus1(0) >=  rpcSlice->getTLayer()) )
#endif 
        {
#endif
          rpcSlice->setActiveNumILRRefIdx(1);
          rpcSlice->setInterLayerPredLayerIdc(0,0);
#if O0225_TID_BASED_IL_RPS_DERIV && TSLAYERS_IL_RPS
        }
#endif
      }
    }
  }
  else if( rpcSlice->getVPS()->getIlpSshSignalingEnabledFlag() == true &&  (rpcSlice->getLayerId() > 0 ))
  {
    rpcSlice->setInterLayerPredEnabledFlag(true);

#if O0225_TID_BASED_IL_RPS_DERIV && TSLAYERS_IL_RPS
    Int   numRefLayerPics = 0;
    Int   i = 0;
    Int   refLayerPicIdc  [MAX_VPS_LAYER_ID_PLUS1];
    for(i = 0, numRefLayerPics = 0;  i < rpcSlice->getNumILRRefIdx(); i++ ) 
    {
#if Q0060_MAX_TID_REF_EQUAL_TO_ZERO
      if((rpcSlice->getVPS()->getMaxTidIlRefPicsPlus1(rpcSlice->getVPS()->getLayerIdInVps(i),rpcSlice->getLayerId()) >  rpcSlice->getTLayer() || rpcSlice->getTLayer()==0) &&
        (rpcSlice->getVPS()->getMaxTSLayersMinus1(rpcSlice->getVPS()->getLayerIdInVps(i)) >=  rpcSlice->getTLayer()) )
#else 
      if(rpcSlice->getVPS()->getMaxTidIlRefPicsPlus1(rpcSlice->getVPS()->getLayerIdInVps(i),rpcSlice->getLayerId()) >  rpcSlice->getTLayer() &&
        (rpcSlice->getVPS()->getMaxTSLayersMinus1(rpcSlice->getVPS()->getLayerIdInVps(i)) >=  rpcSlice->getTLayer()) )
#endif 
      {          
        refLayerPicIdc[ numRefLayerPics++ ] = i;
      }
    }
    rpcSlice->setActiveNumILRRefIdx(numRefLayerPics);
    for( i = 0; i < rpcSlice->getActiveNumILRRefIdx(); i++ )
    {
      rpcSlice->setInterLayerPredLayerIdc(refLayerPicIdc[i],i);
    }      
#else
    rpcSlice->setActiveNumILRRefIdx(rpcSlice->getNumILRRefIdx());
    for( Int i = 0; i < rpcSlice->getActiveNumILRRefIdx(); i++ )
    {
      rpcSlice->setInterLayerPredLayerIdc(i,i);
    }
#endif 
  }
#if P0312_VERT_PHASE_ADJ
    for(Int i = 0; i < rpcSlice->getActiveNumILRRefIdx(); i++ ) 
    {
      UInt refLayerIdc = rpcSlice->getInterLayerPredLayerIdc(i);
#if !MOVE_SCALED_OFFSET_TO_PPS
      if( rpcSlice->getSPS()->getVertPhasePositionEnableFlag(refLayerIdc) )
#else
      if( rpcSlice->getPPS()->getVertPhasePositionEnableFlag(refLayerIdc) )
#endif
      {
        READ_FLAG( uiCode, "vert_phase_position_flag" ); rpcSlice->setVertPhasePositionFlag( uiCode? true : false, refLayerIdc );
      }
  }
#endif
#endif //SVC_EXTENSION

  if(sps->getUseSAO())
  {
    READ_FLAG(uiCode, "slice_sao_luma_flag");  rpcSlice->setSaoEnabledFlag((Bool)uiCode);
#if AUXILIARY_PICTURES
    ChromaFormat format;
#if REPN_FORMAT_IN_VPS
#if O0096_REP_FORMAT_INDEX
    if( sps->getLayerId() == 0 )
    {
      format = sps->getChromaFormatIdc();
    }
    else
    {
      format = rpcSlice->getVPS()->getVpsRepFormat( sps->getUpdateRepFormatFlag() ? sps->getUpdateRepFormatIndex() : rpcSlice->getVPS()->getVpsRepFormatIdx(sps->getLayerId()) )->getChromaFormatVpsIdc();
#if Q0195_REP_FORMAT_CLEANUP
      assert( (sps->getUpdateRepFormatFlag()==false && rpcSlice->getVPS()->getVpsNumRepFormats()==1) || rpcSlice->getVPS()->getVpsNumRepFormats() > 1 ); //conformance check
#endif
    }
#else
    if( ( sps->getLayerId() == 0 ) || sps->getUpdateRepFormatFlag() )
    {
      format = sps->getChromaFormatIdc();
    }
    else
    {
      format = rpcSlice->getVPS()->getVpsRepFormat( rpcSlice->getVPS()->getVpsRepFormatIdx(sps->getLayerId()) )->getChromaFormatVpsIdc();
    }
#endif
#else
    format = sps->getChromaFormatIdc();
#endif
    if (format != CHROMA_400)
    {
#endif
      READ_FLAG(uiCode, "slice_sao_chroma_flag");  rpcSlice->setSaoEnabledFlagChroma((Bool)uiCode);
#if AUXILIARY_PICTURES
    }
    else
    {
      rpcSlice->setSaoEnabledFlagChroma(false);
    }
#endif
  }

  if (rpcSlice->getIdrPicFlag())
  {
    rpcSlice->setEnableTMVPFlag(false);
  }
  if (!rpcSlice->isIntra())
  {

    READ_FLAG( uiCode, "num_ref_idx_active_override_flag");
    if (uiCode)
    {
      READ_UVLC (uiCode, "num_ref_idx_l0_active_minus1" );  rpcSlice->setNumRefIdx( REF_PIC_LIST_0, uiCode + 1 );
      if (rpcSlice->isInterB())
      {
        READ_UVLC (uiCode, "num_ref_idx_l1_active_minus1" );  rpcSlice->setNumRefIdx( REF_PIC_LIST_1, uiCode + 1 );
      }
      else
      {
        rpcSlice->setNumRefIdx(REF_PIC_LIST_1, 0);
      }
    }
    else
    {
      rpcSlice->setNumRefIdx(REF_PIC_LIST_0, rpcSlice->getPPS()->getNumRefIdxL0DefaultActive());
      if (rpcSlice->isInterB())
      {
        rpcSlice->setNumRefIdx(REF_PIC_LIST_1, rpcSlice->getPPS()->getNumRefIdxL1DefaultActive());
      }
      else
      {
        rpcSlice->setNumRefIdx(REF_PIC_LIST_1,0);
      }
    }
  }
  // }
  TComRefPicListModification* refPicListModification = rpcSlice->getRefPicListModification();
  if(!rpcSlice->isIntra())
  {
    if( !rpcSlice->getPPS()->getListsModificationPresentFlag() || rpcSlice->getNumRpsCurrTempList() <= 1 )
    {
      refPicListModification->setRefPicListModificationFlagL0( 0 );
    }
    else
    {
      READ_FLAG( uiCode, "ref_pic_list_modification_flag_l0" ); refPicListModification->setRefPicListModificationFlagL0( uiCode ? 1 : 0 );
    }

    if(refPicListModification->getRefPicListModificationFlagL0())
    {
      uiCode = 0;
      Int i = 0;
      Int numRpsCurrTempList0 = rpcSlice->getNumRpsCurrTempList();
      if ( numRpsCurrTempList0 > 1 )
      {
        Int length = 1;
        numRpsCurrTempList0 --;
        while ( numRpsCurrTempList0 >>= 1)
        {
          length ++;
        }
        for (i = 0; i < rpcSlice->getNumRefIdx(REF_PIC_LIST_0); i ++)
        {
          READ_CODE( length, uiCode, "list_entry_l0" );
          refPicListModification->setRefPicSetIdxL0(i, uiCode );
        }
      }
      else
      {
        for (i = 0; i < rpcSlice->getNumRefIdx(REF_PIC_LIST_0); i ++)
        {
          refPicListModification->setRefPicSetIdxL0(i, 0 );
        }
      }
    }
  }
  else
  {
    refPicListModification->setRefPicListModificationFlagL0(0);
  }
  if(rpcSlice->isInterB())
  {
    if( !rpcSlice->getPPS()->getListsModificationPresentFlag() || rpcSlice->getNumRpsCurrTempList() <= 1 )
    {
      refPicListModification->setRefPicListModificationFlagL1( 0 );
    }
    else
    {
      READ_FLAG( uiCode, "ref_pic_list_modification_flag_l1" ); refPicListModification->setRefPicListModificationFlagL1( uiCode ? 1 : 0 );
    }
    if(refPicListModification->getRefPicListModificationFlagL1())
    {
      uiCode = 0;
      Int i = 0;
      Int numRpsCurrTempList1 = rpcSlice->getNumRpsCurrTempList();
      if ( numRpsCurrTempList1 > 1 )
      {
        Int length = 1;
        numRpsCurrTempList1 --;
        while ( numRpsCurrTempList1 >>= 1)
        {
          length ++;
        }
        for (i = 0; i < rpcSlice->getNumRefIdx(REF_PIC_LIST_1); i ++)
        {
          READ_CODE( length, uiCode, "list_entry_l1" );
          refPicListModification->setRefPicSetIdxL1(i, uiCode );
        }
      }
      else
      {
        for (i = 0; i < rpcSlice->getNumRefIdx(REF_PIC_LIST_1); i ++)
        {
          refPicListModification->setRefPicSetIdxL1(i, 0 );
        }
      }
    }
  }
  else
  {
    refPicListModification->setRefPicListModificationFlagL1(0);
  }
  if (rpcSlice->isInterB())
  {
    READ_FLAG( uiCode, "mvd_l1_zero_flag" );       rpcSlice->setMvdL1ZeroFlag( (uiCode ? true : false) );
  }

  rpcSlice->setCabacInitFlag( false ); // default
  if(pps->getCabacInitPresentFlag() && !rpcSlice->isIntra())
  {
    READ_FLAG(uiCode, "cabac_init_flag");
    rpcSlice->setCabacInitFlag( uiCode ? true : false );
  }

  if ( rpcSlice->getEnableTMVPFlag() )
  {
#if SVC_EXTENSION && REF_IDX_MFM
    // set motion mapping flag
    rpcSlice->setMFMEnabledFlag( ( rpcSlice->getNumMotionPredRefLayers() > 0 && rpcSlice->getActiveNumILRRefIdx() && !rpcSlice->isIntra() ) ? true : false );
#endif
    if ( rpcSlice->getSliceType() == B_SLICE )
    {
      READ_FLAG( uiCode, "collocated_from_l0_flag" );
      rpcSlice->setColFromL0Flag(uiCode);
    }
    else
    {
      rpcSlice->setColFromL0Flag( 1 );
    }

    if ( rpcSlice->getSliceType() != I_SLICE &&
      ((rpcSlice->getColFromL0Flag() == 1 && rpcSlice->getNumRefIdx(REF_PIC_LIST_0) > 1)||
      (rpcSlice->getColFromL0Flag() == 0 && rpcSlice->getNumRefIdx(REF_PIC_LIST_1) > 1)))
    {
      READ_UVLC( uiCode, "collocated_ref_idx" );
      rpcSlice->setColRefIdx(uiCode);
    }
    else
    {
      rpcSlice->setColRefIdx(0);
    }
  }
  if ( (pps->getUseWP() && rpcSlice->getSliceType()==P_SLICE) || (pps->getWPBiPred() && rpcSlice->getSliceType()==B_SLICE) )
  {
    xParsePredWeightTable(rpcSlice);
    rpcSlice->initWpScaling();
  }
  if (!rpcSlice->isIntra())
  {
    READ_UVLC( uiCode, "five_minus_max_num_merge_cand");
    rpcSlice->setMaxNumMergeCand(MRG_MAX_NUM_CANDS - uiCode);
  }

  READ_SVLC( iCode, "slice_qp_delta" );
  rpcSlice->setSliceQp (26 + pps->getPicInitQPMinus26() + iCode);

#if REPN_FORMAT_IN_VPS
#if O0194_DIFFERENT_BITDEPTH_EL_BL
  g_bitDepthYLayer[rpcSlice->getLayerId()] = rpcSlice->getBitDepthY();
  g_bitDepthCLayer[rpcSlice->getLayerId()] = rpcSlice->getBitDepthC();
#endif
  assert( rpcSlice->getSliceQp() >= -rpcSlice->getQpBDOffsetY() );
#else
  assert( rpcSlice->getSliceQp() >= -sps->getQpBDOffsetY() );
#endif
  assert( rpcSlice->getSliceQp() <=  51 );

  if (rpcSlice->getPPS()->getSliceChromaQpFlag())
  {
    READ_SVLC( iCode, "slice_qp_delta_cb" );
    rpcSlice->setSliceQpDeltaCb( iCode );
    assert( rpcSlice->getSliceQpDeltaCb() >= -12 );
    assert( rpcSlice->getSliceQpDeltaCb() <=  12 );
    assert( (rpcSlice->getPPS()->getChromaCbQpOffset() + rpcSlice->getSliceQpDeltaCb()) >= -12 );
    assert( (rpcSlice->getPPS()->getChromaCbQpOffset() + rpcSlice->getSliceQpDeltaCb()) <=  12 );

    READ_SVLC( iCode, "slice_qp_delta_cr" );
    rpcSlice->setSliceQpDeltaCr( iCode );
    assert( rpcSlice->getSliceQpDeltaCr() >= -12 );
    assert( rpcSlice->getSliceQpDeltaCr() <=  12 );
    assert( (rpcSlice->getPPS()->getChromaCrQpOffset() + rpcSlice->getSliceQpDeltaCr()) >= -12 );
    assert( (rpcSlice->getPPS()->getChromaCrQpOffset() + rpcSlice->getSliceQpDeltaCr()) <=  12 );
  }

  if (rpcSlice->getPPS()->getDeblockingFilterControlPresentFlag())
  {
    if(rpcSlice->getPPS()->getDeblockingFilterOverrideEnabledFlag())
    {
      READ_FLAG ( uiCode, "deblocking_filter_override_flag" );        rpcSlice->setDeblockingFilterOverrideFlag(uiCode ? true : false);
    }
    else
    {
      rpcSlice->setDeblockingFilterOverrideFlag(0);
    }
    if(rpcSlice->getDeblockingFilterOverrideFlag())
    {
      READ_FLAG ( uiCode, "slice_disable_deblocking_filter_flag" );   rpcSlice->setDeblockingFilterDisable(uiCode ? 1 : 0);
      if(!rpcSlice->getDeblockingFilterDisable())
      {
        READ_SVLC( iCode, "slice_beta_offset_div2" );                       rpcSlice->setDeblockingFilterBetaOffsetDiv2(iCode);
        assert(rpcSlice->getDeblockingFilterBetaOffsetDiv2() >= -6 &&
          rpcSlice->getDeblockingFilterBetaOffsetDiv2() <=  6);
        READ_SVLC( iCode, "slice_tc_offset_div2" );                         rpcSlice->setDeblockingFilterTcOffsetDiv2(iCode);
        assert(rpcSlice->getDeblockingFilterTcOffsetDiv2() >= -6 &&
          rpcSlice->getDeblockingFilterTcOffsetDiv2() <=  6);
      }
    }
    else
    {
      rpcSlice->setDeblockingFilterDisable   ( rpcSlice->getPPS()->getPicDisableDeblockingFilterFlag() );
      rpcSlice->setDeblockingFilterBetaOffsetDiv2( rpcSlice->getPPS()->getDeblockingFilterBetaOffsetDiv2() );
      rpcSlice->setDeblockingFilterTcOffsetDiv2  ( rpcSlice->getPPS()->getDeblockingFilterTcOffsetDiv2() );
    }
  }
  else
  {
    rpcSlice->setDeblockingFilterDisable       ( false );
    rpcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
    rpcSlice->setDeblockingFilterTcOffsetDiv2  ( 0 );
  }

  Bool isSAOEnabled = (!rpcSlice->getSPS()->getUseSAO())?(false):(rpcSlice->getSaoEnabledFlag()||rpcSlice->getSaoEnabledFlagChroma());
  Bool isDBFEnabled = (!rpcSlice->getDeblockingFilterDisable());

  if(rpcSlice->getPPS()->getLoopFilterAcrossSlicesEnabledFlag() && ( isSAOEnabled || isDBFEnabled ))
  {
    READ_FLAG( uiCode, "slice_loop_filter_across_slices_enabled_flag");
  }
  else
  {
    uiCode = rpcSlice->getPPS()->getLoopFilterAcrossSlicesEnabledFlag()?1:0;
  }
  rpcSlice->setLFCrossSliceBoundaryFlag( (uiCode==1)?true:false);

}

UInt *entryPointOffset          = NULL;
UInt numEntryPointOffsets, offsetLenMinus1;
if( pps->getTilesEnabledFlag() || pps->getEntropyCodingSyncEnabledFlag() )
{
  READ_UVLC(numEntryPointOffsets, "num_entry_point_offsets"); rpcSlice->setNumEntryPointOffsets ( numEntryPointOffsets );
  if (numEntryPointOffsets>0)
  {
    READ_UVLC(offsetLenMinus1, "offset_len_minus1");
  }
  entryPointOffset = new UInt[numEntryPointOffsets];
  for (UInt idx=0; idx<numEntryPointOffsets; idx++)
  {
    READ_CODE(offsetLenMinus1+1, uiCode, "entry_point_offset_minus1");
    entryPointOffset[ idx ] = uiCode + 1;
  }
}
else
{
  rpcSlice->setNumEntryPointOffsets ( 0 );
}

#if POC_RESET_IDC_SIGNALLING
Int sliceHeaderExtensionLength = 0;
if(pps->getSliceHeaderExtensionPresentFlag())
{
  READ_UVLC( uiCode, "slice_header_extension_length"); sliceHeaderExtensionLength = uiCode;
}
else
{
  sliceHeaderExtensionLength = 0;
#if INFERENCE_POC_MSB_VAL_PRESENT
  rpcSlice->setPocMsbValPresentFlag( false );
#endif
}
UInt startBits = m_eReader.getNumBitsRead();     // Start counter of # SH Extn bits
if( sliceHeaderExtensionLength > 0 )
{
  if( rpcSlice->getPPS()->getPocResetInfoPresentFlag() )
  {
    READ_CODE( 2, uiCode,       "poc_reset_idc"); rpcSlice->setPocResetIdc(uiCode);
#if POC_RESET_RESTRICTIONS
    /* The value of poc_reset_idc shall not be equal to 1 or 2 for a RASL picture, a RADL picture, 
       a sub-layer non-reference picture, or a picture that has TemporalId greater than 0, 
       or a picture that has discardable_flag equal to 1. */
    if( rpcSlice->getPocResetIdc() == 1 || rpcSlice->getPocResetIdc() == 2 )
    {
      assert( !rpcSlice->isRASL() );
      assert( !rpcSlice->isRADL() );
      assert( !rpcSlice->isSLNR() );
      assert( rpcSlice->getTLayer() == 0 );
      assert( rpcSlice->getDiscardableFlag() == 0 );
    }

    // The value of poc_reset_idc of a CRA or BLA picture shall be less than 3.
    if( rpcSlice->getPocResetIdc() == 3)
    {
      assert( ! ( rpcSlice->isCRA() || rpcSlice->isBLA() ) );
    }
#endif
  }
  else
  {
    rpcSlice->setPocResetIdc( 0 );
  }
#if Q0142_POC_LSB_NOT_PRESENT
  if ( rpcSlice->getVPS()->getPocLsbNotPresentFlag(rpcSlice->getLayerId()) && iPOClsb > 0 )
  {
    assert( rpcSlice->getPocResetIdc() != 2 );
  }
#endif
  if( rpcSlice->getPocResetIdc() > 0 )
  {
    READ_CODE(6, uiCode,      "poc_reset_period_id"); rpcSlice->setPocResetPeriodId(uiCode);
  }
  else
  {

    rpcSlice->setPocResetPeriodId( 0 );
  }

  if (rpcSlice->getPocResetIdc() == 3)
  {
    READ_FLAG( uiCode,        "full_poc_reset_flag"); rpcSlice->setFullPocResetFlag((uiCode == 1) ? true : false);
    READ_CODE(rpcSlice->getSPS()->getBitsForPOC(), uiCode,"poc_lsb_val"); rpcSlice->setPocLsbVal(uiCode);
#if Q0142_POC_LSB_NOT_PRESENT
    if ( rpcSlice->getVPS()->getPocLsbNotPresentFlag(rpcSlice->getLayerId()) && rpcSlice->getFullPocResetFlag() )
    {
      assert( rpcSlice->getPocLsbVal() == 0 );
    }
#endif
  }

  // Derive the value of PocMsbValRequiredFlag
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  rpcSlice->setPocMsbValRequiredFlag( (rpcSlice->getCraPicFlag() || rpcSlice->getBlaPicFlag())
                                      && (!rpcSlice->getVPS()->getVpsPocLsbAlignedFlag() ||
                                         (rpcSlice->getVPS()->getVpsPocLsbAlignedFlag() && rpcSlice->getVPS()->getNumDirectRefLayers(rpcSlice->getLayerId()) == 0))
                                    );
#else
  rpcSlice->setPocMsbValRequiredFlag( rpcSlice->getCraPicFlag() || rpcSlice->getBlaPicFlag() );
#endif

#if P0297_VPS_POC_LSB_ALIGNED_FLAG
  if (!rpcSlice->getPocMsbValRequiredFlag() && rpcSlice->getVPS()->getVpsPocLsbAlignedFlag())
#else
  if (!rpcSlice->getPocMsbValRequiredFlag() /* vps_poc_lsb_aligned_flag */)
#endif
  {
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
    READ_FLAG(uiCode, "poc_msb_cycle_val_present_flag"); rpcSlice->setPocMsbValPresentFlag(uiCode ? true : false);
#else
    READ_FLAG(uiCode, "poc_msb_val_present_flag"); rpcSlice->setPocMsbValPresentFlag(uiCode ? true : false);
#endif
  }
  else
  {
#if POC_MSB_VAL_PRESENT_FLAG_SEM
    if( sliceHeaderExtensionLength == 0 )
    {
      rpcSlice->setPocMsbValPresentFlag( false );
    }
    else if( rpcSlice->getPocMsbValRequiredFlag() )
#else
    if( rpcSlice->getPocMsbValRequiredFlag() )
#endif
    {
      rpcSlice->setPocMsbValPresentFlag( true );
    }
    else
    {
      rpcSlice->setPocMsbValPresentFlag( false );
    }
  }

#if !POC_RESET_IDC_DECODER
  Int maxPocLsb  = 1 << rpcSlice->getSPS()->getBitsForPOC();
#endif
  if( rpcSlice->getPocMsbValPresentFlag() )
  {
#if P0297_VPS_POC_LSB_ALIGNED_FLAG
    READ_UVLC( uiCode,    "poc_msb_cycle_val");             rpcSlice->setPocMsbVal( uiCode );
#else
    READ_UVLC( uiCode,    "poc_msb_val");             rpcSlice->setPocMsbVal( uiCode );
#endif

#if !POC_RESET_IDC_DECODER
    // Update POC of the slice based on this MSB val
    Int pocLsb     = rpcSlice->getPOC() % maxPocLsb;
    rpcSlice->setPOC((rpcSlice->getPocMsbVal() * maxPocLsb) + pocLsb);
  }
  else
  {
    rpcSlice->setPocMsbVal( rpcSlice->getPOC() / maxPocLsb );
#endif
  }

  // Read remaining bits in the slice header extension.
  UInt endBits = m_eReader.getNumBitsRead();
  Int counter = (endBits - startBits) % 8;
  if( counter )
  {
    counter = 8 - counter;
  }

  while( counter )
  {
#if Q0146_SSH_EXT_DATA_BIT 
    READ_FLAG( uiCode, "slice_segment_header_extension_data_bit" );
#else
    READ_FLAG( uiCode, "slice_segment_header_extension_reserved_bit" ); assert( uiCode == 1 );
#endif
    counter--;
  }
}
#else
if(pps->getSliceHeaderExtensionPresentFlag())
{
  READ_UVLC(uiCode,"slice_header_extension_length");
  for(Int i=0; i<uiCode; i++)
  {
    UInt ignore;
    READ_CODE(8,ignore,"slice_header_extension_data_byte");
  }
}
#endif
/* TCH
 m_pcBitstream->readByteAlignment();

if( pps->getTilesEnabledFlag() || pps->getEntropyCodingSyncEnabledFlag() )
{
  Int endOfSliceHeaderLocation = m_pcBitstream->getByteLocation();

  // Adjust endOfSliceHeaderLocation to account for emulation prevention bytes in the slice segment header
  for ( UInt curByteIdx  = 0; curByteIdx<m_pcBitstream->numEmulationPreventionBytesRead(); curByteIdx++ )
  {
    if ( m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) < endOfSliceHeaderLocation )
    {
      endOfSliceHeaderLocation++;
    }
  }

  Int  curEntryPointOffset     = 0;
  Int  prevEntryPointOffset    = 0;
  for (UInt idx=0; idx<numEntryPointOffsets; idx++)
  {
    curEntryPointOffset += entryPointOffset[ idx ];

    Int emulationPreventionByteCount = 0;
    for ( UInt curByteIdx  = 0; curByteIdx<m_pcBitstream->numEmulationPreventionBytesRead(); curByteIdx++ )
    {
      if ( m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) >= ( prevEntryPointOffset + endOfSliceHeaderLocation ) &&
        m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) <  ( curEntryPointOffset  + endOfSliceHeaderLocation ) )
      {
        emulationPreventionByteCount++;
      }
    }

    entryPointOffset[ idx ] -= emulationPreventionByteCount;
    prevEntryPointOffset = curEntryPointOffset;
  }

  if ( pps->getTilesEnabledFlag() )
  {
    rpcSlice->setTileLocationCount( numEntryPointOffsets );

    UInt prevPos = 0;
    for (Int idx=0; idx<rpcSlice->getTileLocationCount(); idx++)
    {
      rpcSlice->setTileLocation( idx, prevPos + entryPointOffset [ idx ] );
      prevPos += entryPointOffset[ idx ];
    }
  }
  else if ( pps->getEntropyCodingSyncEnabledFlag() )
  {
    Int numSubstreams = rpcSlice->getNumEntryPointOffsets()+1;
    rpcSlice->allocSubstreamSizes(numSubstreams);
    UInt *pSubstreamSizes       = rpcSlice->getSubstreamSizes();
    for (Int idx=0; idx<numSubstreams-1; idx++)
    {
      if ( idx < numEntryPointOffsets )
      {
        pSubstreamSizes[ idx ] = ( entryPointOffset[ idx ] << 3 ) ;
      }
      else
      {
        pSubstreamSizes[ idx ] = 0;
      }
    }
  }

  if (entryPointOffset)
  {
    delete [] entryPointOffset;
  }
}
TCH */

return;
}

Void TDecCavlc::parsePTL( TComPTL *rpcPTL, Bool profilePresentFlag, Int maxNumSubLayersMinus1 )
{
  UInt uiCode;
  if(profilePresentFlag)
  {
    parseProfileTier(rpcPTL->getGeneralPTL());
  }
  READ_CODE( 8, uiCode, "general_level_idc" );    rpcPTL->getGeneralPTL()->setLevelIdc(uiCode);

  for (Int i = 0; i < maxNumSubLayersMinus1; i++)
  {
    if(profilePresentFlag)
    {
      READ_FLAG( uiCode, "sub_layer_profile_present_flag[i]" ); rpcPTL->setSubLayerProfilePresentFlag(i, uiCode);
    }
    READ_FLAG( uiCode, "sub_layer_level_present_flag[i]"   ); rpcPTL->setSubLayerLevelPresentFlag  (i, uiCode);
  }

  if (maxNumSubLayersMinus1 > 0)
  {
    for (Int i = maxNumSubLayersMinus1; i < 8; i++)
    {
      READ_CODE(2, uiCode, "reserved_zero_2bits");
      assert(uiCode == 0);
    }
  }

  for(Int i = 0; i < maxNumSubLayersMinus1; i++)
  {
    if( profilePresentFlag && rpcPTL->getSubLayerProfilePresentFlag(i) )
    {
      parseProfileTier(rpcPTL->getSubLayerPTL(i));
    }
    if(rpcPTL->getSubLayerLevelPresentFlag(i))
    {
      READ_CODE( 8, uiCode, "sub_layer_level_idc[i]" );   rpcPTL->getSubLayerPTL(i)->setLevelIdc(uiCode);
    }
  }
}

Void TDecCavlc::parseProfileTier(ProfileTierLevel *ptl)
{
  UInt uiCode;
  READ_CODE(2 , uiCode, "XXX_profile_space[]");   ptl->setProfileSpace(uiCode);
  READ_FLAG(    uiCode, "XXX_tier_flag[]"    );   ptl->setTierFlag    (uiCode ? 1 : 0);
  READ_CODE(5 , uiCode, "XXX_profile_idc[]"  );   ptl->setProfileIdc  (uiCode);
  for(Int j = 0; j < 32; j++)
  {
    READ_FLAG(  uiCode, "XXX_profile_compatibility_flag[][j]");   ptl->setProfileCompatibilityFlag(j, uiCode ? 1 : 0);
  }
  READ_FLAG(uiCode, "general_progressive_source_flag");
  ptl->setProgressiveSourceFlag(uiCode ? true : false);

  READ_FLAG(uiCode, "general_interlaced_source_flag");
  ptl->setInterlacedSourceFlag(uiCode ? true : false);

  READ_FLAG(uiCode, "general_non_packed_constraint_flag");
  ptl->setNonPackedConstraintFlag(uiCode ? true : false);

  READ_FLAG(uiCode, "general_frame_only_constraint_flag");
  ptl->setFrameOnlyConstraintFlag(uiCode ? true : false);

  READ_CODE(16, uiCode, "XXX_reserved_zero_44bits[0..15]");
  READ_CODE(16, uiCode, "XXX_reserved_zero_44bits[16..31]");
  READ_CODE(12, uiCode, "XXX_reserved_zero_44bits[32..43]");
}

Void TDecCavlc::parseTerminatingBit( UInt& ruiBit )
{
//  ruiBit = false;
//  Int iBitsLeft = m_pcBitstream->getNumBitsLeft();
//  if(iBitsLeft <= 8)
//  {
//    UInt uiPeekValue = m_pcBitstream->peekBits(iBitsLeft);
//    if (uiPeekValue == (1<<(iBitsLeft-1)))
//    {
//      ruiBit = true;
//    }
//  }
}


// TCH
//Void TDecCavlc::parseSkipFlag( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
//{
//  assert(0);
//}
//
//Void TDecCavlc::parseCUTransquantBypassFlag( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
//{
//  assert(0);
//}
//
//Void TDecCavlc::parseMVPIdx( Int& /*riMVPIdx*/ )
//{
//  assert(0);
//}
//
//Void TDecCavlc::parseSplitFlag     ( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
//{
//  assert(0);
//}
//
//Void TDecCavlc::parsePartSize( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
//{
//  assert(0);
//}
//
//Void TDecCavlc::parsePredMode( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
//{
//  assert(0);
//}
//
///** Parse I_PCM information.
//* \param pcCU pointer to CU
//* \param uiAbsPartIdx CU index
//* \param uiDepth CU depth
//* \returns Void
//*
//* If I_PCM flag indicates that the CU is I_PCM, parse its PCM alignment bits and codes.
//*/
//Void TDecCavlc::parseIPCMInfo( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
//{
//  assert(0);
//}
//
//Void TDecCavlc::parseIntraDirLumaAng  ( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
//{
//  assert(0);
//}
//
//Void TDecCavlc::parseIntraDirChroma( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
//{
//  assert(0);
//}
//
//Void TDecCavlc::parseInterDir( TComDataCU* /*pcCU*/, UInt& /*ruiInterDir*/, UInt /*uiAbsPartIdx*/ )
//{
//  assert(0);
//}
//
//Void TDecCavlc::parseRefFrmIdx( TComDataCU* /*pcCU*/, Int& /*riRefFrmIdx*/, RefPicList /*eRefList*/ )
//{
//  assert(0);
//}
//
//Void TDecCavlc::parseMvd( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiPartIdx*/, UInt /*uiDepth*/, RefPicList /*eRefList*/ )
//{
//  assert(0);
//}
//
//Void TDecCavlc::parseDeltaQP( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
//{
//  Int qp;
//  Int  iDQp;
//
//  xReadSvlc( iDQp );
//
//#if REPN_FORMAT_IN_VPS
//  Int qpBdOffsetY = pcCU->getSlice()->getQpBDOffsetY();
//#else
//  Int qpBdOffsetY = pcCU->getSlice()->getSPS()->getQpBDOffsetY();
//#endif
//  qp = (((Int) pcCU->getRefQP( uiAbsPartIdx ) + iDQp + 52 + 2*qpBdOffsetY )%(52+ qpBdOffsetY)) -  qpBdOffsetY;
//
//  UInt uiAbsQpCUPartIdx = (uiAbsPartIdx>>((g_uiMaxCUDepth - pcCU->getSlice()->getPPS()->getMaxCuDQPDepth())<<1))<<((g_uiMaxCUDepth - pcCU->getSlice()->getPPS()->getMaxCuDQPDepth())<<1) ;
//  UInt uiQpCUDepth =   min(uiDepth,pcCU->getSlice()->getPPS()->getMaxCuDQPDepth()) ;
//
//  pcCU->setQPSubParts( qp, uiAbsQpCUPartIdx, uiQpCUDepth );
//}
//
//Void TDecCavlc::parseCoeffNxN( TComDataCU* /*pcCU*/, TCoeff* /*pcCoef*/, UInt /*uiAbsPartIdx*/, UInt /*uiWidth*/, UInt /*uiHeight*/, UInt /*uiDepth*/, TextType /*eTType*/ )
//{
//  assert(0);
//}
//
//Void TDecCavlc::parseTransformSubdivFlag( UInt& /*ruiSubdivFlag*/, UInt /*uiLog2TransformBlockSize*/ )
//{
//  assert(0);
//}
//
//Void TDecCavlc::parseQtCbf( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, TextType /*eType*/, UInt /*uiTrDepth*/, UInt /*uiDepth*/ )
//{
//  assert(0);
//}
//
//Void TDecCavlc::parseQtRootCbf( UInt /*uiAbsPartIdx*/, UInt& /*uiQtRootCbf*/ )
//{
//  assert(0);
//}
//
//Void TDecCavlc::parseTransformSkipFlags (TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*width*/, UInt /*height*/, UInt /*uiDepth*/, TextType /*eTType*/)
//{
//  assert(0);
//}
//
//Void TDecCavlc::parseMergeFlag ( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/, UInt /*uiPUIdx*/ )
//{
//  assert(0);
//}
//
//Void TDecCavlc::parseMergeIndex ( TComDataCU* /*pcCU*/, UInt& /*ruiMergeIndex*/ )
//{
//  assert(0);
//}
// TCH

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

/** parse explicit wp tables
* \param TComSlice* pcSlice
* \returns Void
*/
Void TDecCavlc::xParsePredWeightTable( TComSlice* pcSlice )
{
  wpScalingParam  *wp;
  Bool            bChroma     = true; // color always present in HEVC ?
  SliceType       eSliceType  = pcSlice->getSliceType();
  Int             iNbRef       = (eSliceType == B_SLICE ) ? (2) : (1);
#if SVC_EXTENSION
  UInt            uiLog2WeightDenomLuma = 0, uiLog2WeightDenomChroma = 0;
#else
  UInt            uiLog2WeightDenomLuma, uiLog2WeightDenomChroma;
#endif
  UInt            uiTotalSignalledWeightFlags = 0;

  Int iDeltaDenom;
#if AUXILIARY_PICTURES
  if (pcSlice->getChromaFormatIdc() == CHROMA_400)
  {
    bChroma = false;
  }
#endif
  // decode delta_luma_log2_weight_denom :
  READ_UVLC( uiLog2WeightDenomLuma, "luma_log2_weight_denom" );     // ue(v): luma_log2_weight_denom
  assert( uiLog2WeightDenomLuma <= 7 );
  if( bChroma )
  {
    READ_SVLC( iDeltaDenom, "delta_chroma_log2_weight_denom" );     // se(v): delta_chroma_log2_weight_denom
    assert((iDeltaDenom + (Int)uiLog2WeightDenomLuma)>=0);
    assert((iDeltaDenom + (Int)uiLog2WeightDenomLuma)<=7);
    uiLog2WeightDenomChroma = (UInt)(iDeltaDenom + uiLog2WeightDenomLuma);
  }

  for ( Int iNumRef=0 ; iNumRef<iNbRef ; iNumRef++ )
  {
    RefPicList  eRefPicList = ( iNumRef ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for ( Int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
    {
      pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);

      wp[0].uiLog2WeightDenom = uiLog2WeightDenomLuma;
#if AUXILIARY_PICTURES
      if (!bChroma)
      {
        wp[1].uiLog2WeightDenom = 0;
        wp[2].uiLog2WeightDenom = 0;
      }
      else
      {
#endif
        wp[1].uiLog2WeightDenom = uiLog2WeightDenomChroma;
        wp[2].uiLog2WeightDenom = uiLog2WeightDenomChroma;
#if AUXILIARY_PICTURES
      }
#endif

      UInt  uiCode;
      READ_FLAG( uiCode, "luma_weight_lX_flag" );           // u(1): luma_weight_l0_flag
      wp[0].bPresentFlag = ( uiCode == 1 );
      uiTotalSignalledWeightFlags += wp[0].bPresentFlag;
    }
    if ( bChroma )
    {
      UInt  uiCode;
      for ( Int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
      {
        pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);
        READ_FLAG( uiCode, "chroma_weight_lX_flag" );      // u(1): chroma_weight_l0_flag
        wp[1].bPresentFlag = ( uiCode == 1 );
        wp[2].bPresentFlag = ( uiCode == 1 );
        uiTotalSignalledWeightFlags += 2*wp[1].bPresentFlag;
      }
    }
    for ( Int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
    {
      pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);
      if ( wp[0].bPresentFlag )
      {
        Int iDeltaWeight;
        READ_SVLC( iDeltaWeight, "delta_luma_weight_lX" );  // se(v): delta_luma_weight_l0[i]
        assert( iDeltaWeight >= -128 );
        assert( iDeltaWeight <=  127 );
        wp[0].iWeight = (iDeltaWeight + (1<<wp[0].uiLog2WeightDenom));
        READ_SVLC( wp[0].iOffset, "luma_offset_lX" );       // se(v): luma_offset_l0[i]
        assert( wp[0].iOffset >= -128 );
        assert( wp[0].iOffset <=  127 );
      }
      else
      {
        wp[0].iWeight = (1 << wp[0].uiLog2WeightDenom);
        wp[0].iOffset = 0;
      }
      if ( bChroma )
      {
        if ( wp[1].bPresentFlag )
        {
          for ( Int j=1 ; j<3 ; j++ )
          {
            Int iDeltaWeight;
            READ_SVLC( iDeltaWeight, "delta_chroma_weight_lX" );  // se(v): chroma_weight_l0[i][j]
            assert( iDeltaWeight >= -128 );
            assert( iDeltaWeight <=  127 );
            wp[j].iWeight = (iDeltaWeight + (1<<wp[1].uiLog2WeightDenom));

            Int iDeltaChroma;
            READ_SVLC( iDeltaChroma, "delta_chroma_offset_lX" );  // se(v): delta_chroma_offset_l0[i][j]
            assert( iDeltaChroma >= -512 );
            assert( iDeltaChroma <=  511 );
            Int pred = ( 128 - ( ( 128*wp[j].iWeight)>>(wp[j].uiLog2WeightDenom) ) );
            wp[j].iOffset = Clip3(-128, 127, (iDeltaChroma + pred) );
          }
        }
        else
        {
          for ( Int j=1 ; j<3 ; j++ )
          {
            wp[j].iWeight = (1 << wp[j].uiLog2WeightDenom);
            wp[j].iOffset = 0;
          }
        }
      }
    }

    for ( Int iRefIdx=pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx<MAX_NUM_REF ; iRefIdx++ )
    {
      pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);

      wp[0].bPresentFlag = false;
      wp[1].bPresentFlag = false;
      wp[2].bPresentFlag = false;
    }
  }
  assert(uiTotalSignalledWeightFlags<=24);
}

/** decode quantization matrix
* \param scalingList quantization matrix information
*/
Void TDecCavlc::parseScalingList(TComScalingList* scalingList)
{
  UInt  code, sizeId, listId;
  Bool scalingListPredModeFlag;
  //for each size
  for(sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++)
  {
    for(listId = 0; listId <  g_scalingListNum[sizeId]; listId++)
    {
      READ_FLAG( code, "scaling_list_pred_mode_flag");
      scalingListPredModeFlag = (code) ? true : false;
      if(!scalingListPredModeFlag) //Copy Mode
      {
        READ_UVLC( code, "scaling_list_pred_matrix_id_delta");
        scalingList->setRefMatrixId (sizeId,listId,(UInt)((Int)(listId)-(code)));
        if( sizeId > SCALING_LIST_8x8 )
        {
          scalingList->setScalingListDC(sizeId,listId,((listId == scalingList->getRefMatrixId (sizeId,listId))? 16 :scalingList->getScalingListDC(sizeId, scalingList->getRefMatrixId (sizeId,listId))));
        }
        scalingList->processRefMatrix( sizeId, listId, scalingList->getRefMatrixId (sizeId,listId));

      }
      else //DPCM Mode
      {
        xDecodeScalingList(scalingList, sizeId, listId);
      }
    }
  }

  return;
}
/** decode DPCM
* \param scalingList  quantization matrix information
* \param sizeId size index
* \param listId list index
*/
Void TDecCavlc::xDecodeScalingList(TComScalingList *scalingList, UInt sizeId, UInt listId)
{
  Int i,coefNum = min(MAX_MATRIX_COEF_NUM,(Int)g_scalingListSize[sizeId]);
  Int data;
  Int scalingListDcCoefMinus8 = 0;
  Int nextCoef = SCALING_LIST_START_VALUE;
  UInt* scan  = (sizeId == 0) ? g_auiSigLastScan [ SCAN_DIAG ] [ 1 ] :  g_sigLastScanCG32x32;
  Int *dst = scalingList->getScalingListAddress(sizeId, listId);

  if( sizeId > SCALING_LIST_8x8 )
  {
    READ_SVLC( scalingListDcCoefMinus8, "scaling_list_dc_coef_minus8");
    scalingList->setScalingListDC(sizeId,listId,scalingListDcCoefMinus8 + 8);
    nextCoef = scalingList->getScalingListDC(sizeId,listId);
  }

  for(i = 0; i < coefNum; i++)
  {
    READ_SVLC( data, "scaling_list_delta_coef");
    nextCoef = (nextCoef + data + 256 ) % 256;
    dst[scan[i]] = nextCoef;
  }
}

Bool TDecCavlc::xMoreRbspData()
{
  /*
}
  Int bitsLeft = m_pcBitstream->getNumBitsLeft();

  // if there are more than 8 bits, it cannot be rbsp_trailing_bits
  if (bitsLeft > 8)
  {
    return true;
  }

  UChar lastByte = m_pcBitstream->peekBits(bitsLeft);
  Int cnt = bitsLeft;

  // remove trailing bits equal to zero
  while ((cnt>0) && ((lastByte & 1) == 0))
  {
    lastByte >>= 1;
    cnt--;
  }
  // remove bit equal to one
  cnt--;

  // we should not have a negative number of bits
  assert (cnt>=0);

  // we have more data, if cnt is not zero
  return (cnt>0);
  */
  return 0;
}

#if Q0048_CGS_3D_ASYMLUT
Void TDecCavlc::xParse3DAsymLUT( TCom3DAsymLUT * pc3DAsymLUT )
{
#if R0150_CGS_SIGNAL_CONSTRAINTS
  UInt uiNumRefLayersM1;
  READ_UVLC( uiNumRefLayersM1 , "num_cm_ref_layers_minus1" );
  assert( uiNumRefLayersM1 <= 61 );
  for( UInt i = 0 ; i <= uiNumRefLayersM1 ; i++ )
  {
    UInt uiRefLayerId;
    READ_CODE( 6 , uiRefLayerId , "cm_ref_layer_id" );
    pc3DAsymLUT->addRefLayerId( uiRefLayerId );
  }
#endif
  UInt uiCurOctantDepth , uiCurPartNumLog2 , uiInputBitDepthM8 , uiOutputBitDepthM8 , uiResQaunBit;
#if R0300_CGS_RES_COEFF_CODING
  UInt uiDeltaBits; 
#endif 
  READ_CODE( 2 , uiCurOctantDepth , "cm_octant_depth" ); 
  READ_CODE( 2 , uiCurPartNumLog2 , "cm_y_part_num_log2" );     
#if R0150_CGS_SIGNAL_CONSTRAINTS
  UInt uiChromaInputBitDepthM8 , uiChromaOutputBitDepthM8;
  READ_UVLC( uiInputBitDepthM8 , "cm_input_luma_bit_depth_minus8" );         
  READ_UVLC( uiChromaInputBitDepthM8 , "cm_input_chroma_bit_depth_minus8" );
  READ_UVLC( uiOutputBitDepthM8 , "cm_output_luma_bit_depth_minus8" );
  READ_UVLC( uiChromaOutputBitDepthM8 , "cm_output_chroma_bit_depth_minus8" );
  
  //  printf("cm_input_luma_bit_depth_minus8= %lu \n",uiInputBitDepthM8);
  //  printf("cm_input_chroma_bit_depth_minus8= %lu \n",uiChromaInputBitDepthM8);
  //  printf("cm_output_luma_bit_depth_minus8= %lu \n",uiOutputBitDepthM8);
  //  printf("cm_output_chroma_bit_depth_minus8= %lu \n",uiChromaOutputBitDepthM8);
#else
  READ_CODE( 3 , uiInputBitDepthM8 , "cm_input_bit_depth_minus8" );
  Int iInputBitDepthCDelta;
  READ_SVLC(iInputBitDepthCDelta, "cm_input_bit_depth_chroma delta");
  READ_CODE( 3 , uiOutputBitDepthM8 , "cm_output_bit_depth_minus8" ); 
  Int iOutputBitDepthCDelta;
  READ_SVLC(iOutputBitDepthCDelta, "cm_output_bit_depth_chroma_delta");
#endif
  READ_CODE( 2 , uiResQaunBit , "cm_res_quant_bit" );
#if R0300_CGS_RES_COEFF_CODING
  READ_CODE( 2 , uiDeltaBits , "cm_flc_bits" );
  pc3DAsymLUT->setDeltaBits(uiDeltaBits + 1);
#endif

#if R0151_CGS_3D_ASYMLUT_IMPROVE
#if R0150_CGS_SIGNAL_CONSTRAINTS
  Int nAdaptCThresholdU = 1 << ( uiChromaInputBitDepthM8 + 8 - 1 );
  Int nAdaptCThresholdV = 1 << ( uiChromaInputBitDepthM8 + 8 - 1 );
#else
  Int nAdaptCThresholdU = 1 << ( uiInputBitDepthM8 + 8 + iInputBitDepthCDelta - 1 );
  Int nAdaptCThresholdV = 1 << ( uiInputBitDepthM8 + 8 + iInputBitDepthCDelta - 1 );
#endif
  if( uiCurOctantDepth == 1 )
  {
    Int delta = 0;
    READ_SVLC( delta , "cm_adapt_threshold_u_delta" );
    nAdaptCThresholdU += delta;
    READ_SVLC( delta , "cm_adapt_threshold_v_delta" );
    nAdaptCThresholdV += delta;
  }
#endif
  pc3DAsymLUT->destroy();
  pc3DAsymLUT->create( uiCurOctantDepth , uiInputBitDepthM8 + 8 ,  
#if R0150_CGS_SIGNAL_CONSTRAINTS
    uiChromaInputBitDepthM8 + 8 ,
#else
    uiInputBitDepthM8 + 8 + iInputBitDepthCDelta, 
#endif
    uiOutputBitDepthM8 + 8 , 
#if R0150_CGS_SIGNAL_CONSTRAINTS
    uiChromaOutputBitDepthM8 + 8 ,
#else
    uiOutputBitDepthM8 + 8 + iOutputBitDepthCDelta ,
#endif
    uiCurPartNumLog2 
#if R0151_CGS_3D_ASYMLUT_IMPROVE
    , nAdaptCThresholdU , nAdaptCThresholdV 
#endif    
    );
  pc3DAsymLUT->setResQuantBit( uiResQaunBit );

#if R0164_CGS_LUT_BUGFIX_CHECK
  pc3DAsymLUT->xInitCuboids();
#endif
  xParse3DAsymLUTOctant( pc3DAsymLUT , 0 , 0 , 0 , 0 , 1 << pc3DAsymLUT->getCurOctantDepth() );
#if R0164_CGS_LUT_BUGFIX
#if R0164_CGS_LUT_BUGFIX_CHECK
  printf("============= Before 'xCuboidsFilledCheck()': ================\n");
  pc3DAsymLUT->display();
  pc3DAsymLUT->xCuboidsFilledCheck( false );
  printf("============= After 'xCuboidsFilledCheck()': =================\n");
  pc3DAsymLUT->display();
#endif
#endif
}

Void TDecCavlc::xParse3DAsymLUTOctant( TCom3DAsymLUT * pc3DAsymLUT , Int nDepth , Int yIdx , Int uIdx , Int vIdx , Int nLength )
{
  UInt uiOctantSplit = nDepth < pc3DAsymLUT->getCurOctantDepth();
  if( nDepth < pc3DAsymLUT->getCurOctantDepth() )
    READ_FLAG( uiOctantSplit , "split_octant_flag" );
  Int nYPartNum = 1 << pc3DAsymLUT->getCurYPartNumLog2();
  if( uiOctantSplit )
  {
    Int nHalfLength = nLength >> 1;
    for( Int l = 0 ; l < 2 ; l++ )
    {
      for( Int m = 0 ; m < 2 ; m++ )
      {
        for( Int n = 0 ; n < 2 ; n++ )
        {
          xParse3DAsymLUTOctant( pc3DAsymLUT , nDepth + 1 , yIdx + l * nHalfLength * nYPartNum , uIdx + m * nHalfLength , vIdx + n * nHalfLength , nHalfLength );
        }
      }
    }
  }
  else
  {
#if R0300_CGS_RES_COEFF_CODING
    Int nFLCbits = pc3DAsymLUT->getMappingShift()-pc3DAsymLUT->getResQuantBit()-pc3DAsymLUT->getDeltaBits() ; 
    nFLCbits = nFLCbits >= 0 ? nFLCbits:0;
#endif
    for( Int l = 0 ; l < nYPartNum ; l++ )
    {
#if R0164_CGS_LUT_BUGFIX
      Int shift = pc3DAsymLUT->getCurOctantDepth() - nDepth ;
#endif
      for( Int nVertexIdx = 0 ; nVertexIdx < 4 ; nVertexIdx++ )
      {
        UInt uiCodeVertex = 0;
        Int deltaY = 0 , deltaU = 0 , deltaV = 0;
        READ_FLAG( uiCodeVertex , "coded_vertex_flag" );
        if( uiCodeVertex )
        {
#if R0151_CGS_3D_ASYMLUT_IMPROVE
#if R0300_CGS_RES_COEFF_CODING
          xReadParam( deltaY, nFLCbits );
          xReadParam( deltaU, nFLCbits );
          xReadParam( deltaV, nFLCbits );
#else
          xReadParam( deltaY );
          xReadParam( deltaU );
          xReadParam( deltaV );
#endif
#else
          READ_SVLC( deltaY , "resY" );
          READ_SVLC( deltaU , "resU" );
          READ_SVLC( deltaV , "resV" );
#endif
        }
#if R0164_CGS_LUT_BUGFIX
        pc3DAsymLUT->setCuboidVertexResTree( yIdx + (l<<shift) , uIdx , vIdx , nVertexIdx , deltaY , deltaU , deltaV );
        for (Int m = 1; m < (1<<shift); m++) {
          pc3DAsymLUT->setCuboidVertexResTree( yIdx + (l<<shift) + m , uIdx , vIdx , nVertexIdx , 0 , 0 , 0 );
#if R0164_CGS_LUT_BUGFIX_CHECK
          pc3DAsymLUT->xSetFilled( yIdx + (l<<shift) + m , uIdx , vIdx );
#endif
        }
#else
        pc3DAsymLUT->setCuboidVertexResTree( yIdx + l , uIdx , vIdx , nVertexIdx , deltaY , deltaU , deltaV );
#endif
      }
#if R0164_CGS_LUT_BUGFIX_CHECK
      pc3DAsymLUT->xSetExplicit( yIdx + (l<<shift) , uIdx , vIdx );
#endif
    }
#if R0164_CGS_LUT_BUGFIX
    for ( Int u=0 ; u<nLength ; u++ ) {
      for ( Int v=0 ; v<nLength ; v++ ) {
        if ( u!=0 || v!=0 ) {
          for ( Int y=0 ; y<nLength*nYPartNum ; y++ ) {
            for( Int nVertexIdx = 0 ; nVertexIdx < 4 ; nVertexIdx++ )
            {
              pc3DAsymLUT->setCuboidVertexResTree( yIdx + y , uIdx + u , vIdx + v , nVertexIdx , 0 , 0 , 0 );
#if R0164_CGS_LUT_BUGFIX_CHECK
              pc3DAsymLUT->xSetFilled( yIdx + y , uIdx + u , vIdx + v );
#endif
            }
          }
        }
      }
    }
#endif
  }
}

#if R0151_CGS_3D_ASYMLUT_IMPROVE
#if R0300_CGS_RES_COEFF_CODING
Void TDecCavlc::xReadParam( Int& param, Int rParam )
#else
Void TDecCavlc::xReadParam( Int& param )
#endif
{
#if !R0300_CGS_RES_COEFF_CODING
  const UInt rParam = 7;
#endif
  UInt prefix;
  UInt codeWord ;
  UInt rSymbol;
  UInt sign;

  READ_UVLC( prefix, "quotient")  ;
  READ_CODE (rParam, codeWord, "remainder");
  rSymbol = (prefix<<rParam) + codeWord;

  if(rSymbol)
  {
    READ_FLAG(sign, "sign");
    param = sign ? -(Int)(rSymbol) : (Int)(rSymbol);
  }
  else param = 0;
}
#endif
#if VPS_VUI_BSP_HRD_PARAMS
Void TDecCavlc::parseVpsVuiBspHrdParams( TComVPS *vps )
{
  UInt uiCode;
  assert (vps->getTimingInfo()->getTimingInfoPresentFlag() == 1);
  READ_UVLC( uiCode, "vps_num_add_hrd_params" ); vps->setVpsNumAddHrdParams(uiCode);
  vps->createBspHrdParamBuffer(vps->getVpsNumAddHrdParams()); // Also allocates m_cprmsAddPresentFlag and m_numSubLayerHrdMinus

  for( Int i = vps->getNumHrdParameters(), j = 0; i < (Int)vps->getNumHrdParameters() + vps->getVpsNumAddHrdParams(); i++, j++ ) // j = i - vps->getNumHrdParameters()
  {
    if( i > 0 )
    {
      READ_FLAG( uiCode, "cprms_add_present_flag[i]" );   vps->setCprmsAddPresentFlag(j, uiCode ? true : false);
    }
    else
    {
      // i == 0
      if( vps->getNumHrdParameters() == 0 )
      {
        vps->setCprmsAddPresentFlag(0, true);
      }
    }
    READ_UVLC( uiCode, "num_sub_layer_hrd_minus1[i]" ); vps->setNumSubLayerHrdMinus1(j, uiCode );
    assert( uiCode <= vps->getMaxTLayers() - 1 );
    
    parseHrdParameters( vps->getBspHrd(j), vps->getCprmsAddPresentFlag(j), vps->getNumSubLayerHrdMinus1(j) );
    if( i > 0 && !vps->getCprmsAddPresentFlag(i) )
    {
      // Copy common information parameters
      if( i == (Int)vps->getNumHrdParameters() )
      {
        vps->getBspHrd(j)->copyCommonInformation( vps->getHrdParameters( vps->getNumHrdParameters() - 1 ) );
      }
      else
      {
        vps->getBspHrd(j)->copyCommonInformation( vps->getBspHrd( j - 1 ) );
      }
    }
  }
  for (Int h = 1; h < (Int)vps->getNumOutputLayerSets(); h++)
  {
    Int lsIdx = vps->getOutputLayerSetIdx( h );
    READ_UVLC( uiCode, "num_signalled_partitioning_schemes[h]"); vps->setNumSignalledPartitioningSchemes(h, uiCode);
    for( Int j = 0; j < vps->getNumSignalledPartitioningSchemes(h); j++ )
    {
      READ_UVLC( uiCode, "num_partitions_in_scheme_minus1[h][j]" ); vps->setNumPartitionsInSchemeMinus1(h, j, uiCode);
      for( Int k = 0; k <= vps->getNumPartitionsInSchemeMinus1(h, j); k++ )
      {
        for( Int r = 0; r < vps->getNumLayersInIdList( lsIdx ); r++ )
        {
          READ_FLAG( uiCode, "layer_included_in_partition_flag[h][j][k][r]" ); vps->setLayerIncludedInPartitionFlag(h, j, k, r, uiCode ? true : false);
        }
      }
    }
    for( Int i = 0; i < vps->getNumSignalledPartitioningSchemes(h) + 1; i++ )
    {
      for( Int t = 0; t <= (Int)vps->getMaxSLayersInLayerSetMinus1(lsIdx); t++ )
      {
        READ_UVLC( uiCode, "num_bsp_schedules_minus1[h][i][t]");              vps->setNumBspSchedulesMinus1(h, i, t, uiCode);
        for( Int j = 0; j <= vps->getNumBspSchedulesMinus1(h, i, t); j++ )
        {
          for( Int k = 0; k < vps->getNumPartitionsInSchemeMinus1(h, i); k++ )
          {
            READ_UVLC( uiCode, "bsp_comb_hrd_idx[h][i][t][j][k]");      vps->setBspHrdIdx(h, i, t, j, k, uiCode);
            READ_UVLC( uiCode, "bsp_comb_sched_idx[h][i][t][j][k]");    vps->setBspSchedIdx(h, i, t, j, k, uiCode);
          }
        }
      }
    }

    // To be done: Check each layer included in not more than one BSP in every partitioning scheme,
    // and other related checks associated with layers in bitstream partitions.

  }
}
#endif
#endif
//! \}

