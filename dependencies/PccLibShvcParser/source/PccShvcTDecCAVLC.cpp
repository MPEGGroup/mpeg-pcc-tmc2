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

/** \file     TDecCAVLC.cpp
\brief    CAVLC decoder class
*/

#include "PccShvcTDecCAVLC.h"
// #include "SEIread.h"
// #include "TDecSlice.h"
#include "PccShvcTComChromaFormat.h"
#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "TLibCommon/TComCodingStatistics.h"
#endif
#if CGS_3D_ASYMLUT
// #include "../TLibCommon/TCom3DAsymLUT.h"
#endif

//! \ingroup TLibDecoder
//! \{


using namespace pcc_shvc;

#if ENC_DEC_TRACE

Void  xTraceVPSHeader ()
{
  fprintf( g_hTrace, "=========== Video Parameter Set     ===========\n" );
}

Void  xTraceSPSHeader ()
{
  fprintf( g_hTrace, "=========== Sequence Parameter Set  ===========\n" );
}

Void  xTracePPSHeader ()
{
  fprintf( g_hTrace, "=========== Picture Parameter Set  ===========\n");
}

Void  xTraceSliceHeader ()
{
  fprintf( g_hTrace, "=========== Slice ===========\n");
}

#endif

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TDecCavlc::TDecCavlc()
{
}

TDecCavlc::~TDecCavlc()
{

}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

Void TDecCavlc::parseShortTermRefPicSet( TComSPS* sps, TComReferencePictureSet* rps, Int idx )
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
    assert(code <= idx-1); // delta_idx_minus1 shall not be larger than idx-1, otherwise we will predict from a negative row position that does not exist. When idx equals 0 there is no legal value and interRPSPred must be zero. See J0185-r2
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

#if CGS_3D_ASYMLUT
Void TDecCavlc::parsePPS(TComPPS* pcPPS, TCom3DAsymLUT * pc3DAsymLUT, Int nLayerID)
#else
Void TDecCavlc::parsePPS(TComPPS* pcPPS)
#endif

{
#if ENC_DEC_TRACE
  xTracePPSHeader ();
#endif
  UInt  uiCode;

  Int   iCode;

  READ_UVLC( uiCode, "pps_pic_parameter_set_id");
  assert(uiCode <= 63);
  pcPPS->setPPSId (uiCode);

  READ_UVLC( uiCode, "pps_seq_parameter_set_id");
  assert(uiCode <= 15);
  pcPPS->setSPSId (uiCode);

  READ_FLAG( uiCode, "dependent_slice_segments_enabled_flag"    );    pcPPS->setDependentSliceSegmentsEnabledFlag   ( uiCode == 1 );

  READ_FLAG( uiCode, "output_flag_present_flag" );                    pcPPS->setOutputFlagPresentFlag( uiCode==1 );

  READ_CODE(3, uiCode, "num_extra_slice_header_bits");                pcPPS->setNumExtraSliceHeaderBits(uiCode);

  READ_FLAG ( uiCode, "sign_data_hiding_enabled_flag" );              pcPPS->setSignDataHidingEnabledFlag( uiCode );

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
  pcPPS->setQpOffset(COMPONENT_Cb, iCode);
  assert( pcPPS->getQpOffset(COMPONENT_Cb) >= -12 );
  assert( pcPPS->getQpOffset(COMPONENT_Cb) <=  12 );

  READ_SVLC( iCode, "pps_cr_qp_offset");
  pcPPS->setQpOffset(COMPONENT_Cr, iCode);
  assert( pcPPS->getQpOffset(COMPONENT_Cr) >= -12 );
  assert( pcPPS->getQpOffset(COMPONENT_Cr) <=  12 );

  assert(MAX_NUM_COMPONENT<=3);

  READ_FLAG( uiCode, "pps_slice_chroma_qp_offsets_present_flag" );
  pcPPS->setSliceChromaQpFlag( uiCode ? true : false );

  READ_FLAG( uiCode, "weighted_pred_flag" );          // Use of Weighting Prediction (P_SLICE)
  pcPPS->setUseWP( uiCode==1 );
  READ_FLAG( uiCode, "weighted_bipred_flag" );         // Use of Bi-Directional Weighting Prediction (B_SLICE)
  pcPPS->setWPBiPred( uiCode==1 );

  READ_FLAG( uiCode, "transquant_bypass_enabled_flag");
  pcPPS->setTransquantBypassEnabledFlag(uiCode ? true : false);
  READ_FLAG( uiCode, "tiles_enabled_flag"               );    pcPPS->setTilesEnabledFlag            ( uiCode == 1 );
  READ_FLAG( uiCode, "entropy_coding_sync_enabled_flag" );    pcPPS->setEntropyCodingSyncEnabledFlag( uiCode == 1 );

  if( pcPPS->getTilesEnabledFlag() )
  {
    READ_UVLC ( uiCode, "num_tile_columns_minus1" );                pcPPS->setNumTileColumnsMinus1( uiCode );  
    READ_UVLC ( uiCode, "num_tile_rows_minus1" );                   pcPPS->setNumTileRowsMinus1( uiCode );  
    READ_FLAG ( uiCode, "uniform_spacing_flag" );                   pcPPS->setTileUniformSpacingFlag( uiCode == 1 );

    const UInt tileColumnsMinus1 = pcPPS->getNumTileColumnsMinus1();
    const UInt tileRowsMinus1    = pcPPS->getNumTileRowsMinus1();
 
    if ( !pcPPS->getTileUniformSpacingFlag())
    {
      if (tileColumnsMinus1 > 0)
      {
        std::vector<Int> columnWidth(tileColumnsMinus1);
        for(UInt i = 0; i < tileColumnsMinus1; i++)
        { 
          READ_UVLC( uiCode, "column_width_minus1" );  
          columnWidth[i] = uiCode+1;
        }
        pcPPS->setTileColumnWidth(columnWidth);
      }

      if (tileRowsMinus1 > 0)
      {
        std::vector<Int> rowHeight (tileRowsMinus1);
        for(UInt i = 0; i < tileRowsMinus1; i++)
        {
          READ_UVLC( uiCode, "row_height_minus1" );
          rowHeight[i] = uiCode + 1;
        }
        pcPPS->setTileRowHeight(rowHeight);
      }
    }
    assert ((tileColumnsMinus1 + tileRowsMinus1) != 0);
    READ_FLAG ( uiCode, "loop_filter_across_tiles_enabled_flag" );     pcPPS->setLoopFilterAcrossTilesEnabledFlag( uiCode ? true : false );
  }
  READ_FLAG( uiCode, "pps_loop_filter_across_slices_enabled_flag" );   pcPPS->setLoopFilterAcrossSlicesEnabledFlag( uiCode ? true : false );
  READ_FLAG( uiCode, "deblocking_filter_control_present_flag" );       pcPPS->setDeblockingFilterControlPresentFlag( uiCode ? true : false );
  if(pcPPS->getDeblockingFilterControlPresentFlag())
  {
    READ_FLAG( uiCode, "deblocking_filter_override_enabled_flag" );    pcPPS->setDeblockingFilterOverrideEnabledFlag( uiCode ? true : false );
    READ_FLAG( uiCode, "pps_deblocking_filter_disabled_flag" );        pcPPS->setPPSDeblockingFilterDisabledFlag(uiCode ? true : false );
    if(!pcPPS->getPPSDeblockingFilterDisabledFlag())
    {
      READ_SVLC ( iCode, "pps_beta_offset_div2" );                     pcPPS->setDeblockingFilterBetaOffsetDiv2( iCode );
      READ_SVLC ( iCode, "pps_tc_offset_div2" );                       pcPPS->setDeblockingFilterTcOffsetDiv2( iCode );
    }
  }
  READ_FLAG( uiCode, "pps_scaling_list_data_present_flag" );           pcPPS->setScalingListPresentFlag( uiCode ? true : false );
  if(pcPPS->getScalingListPresentFlag ())
  {
    parseScalingList( &(pcPPS->getScalingList()) );
  }

  READ_FLAG( uiCode, "lists_modification_present_flag");
  pcPPS->setListsModificationPresentFlag(uiCode);

  READ_UVLC( uiCode, "log2_parallel_merge_level_minus2");
  pcPPS->setLog2ParallelMergeLevelMinus2 (uiCode);

  READ_FLAG( uiCode, "slice_segment_header_extension_present_flag");
  pcPPS->setSliceHeaderExtensionPresentFlag(uiCode);

  READ_FLAG( uiCode, "pps_extension_present_flag");

#if SVC_EXTENSION
  pcPPS->setExtensionFlag( uiCode ? true : false );
  if( pcPPS->getExtensionFlag() )
#else
  if (uiCode)
#endif
  {
#if ENC_DEC_TRACE || RExt__DECODER_DEBUG_BIT_STATISTICS
    static const TChar *syntaxStrings[]={ "pps_range_extension_flag",
                                          "pps_multilayer_extension_flag",
                                          "pps_extension_6bits[0]",
                                          "pps_extension_6bits[1]",
                                          "pps_extension_6bits[2]",
                                          "pps_extension_6bits[3]",
                                          "pps_extension_6bits[4]",
                                          "pps_extension_6bits[5]" };
#endif

    Bool pps_extension_flags[NUM_PPS_EXTENSION_FLAGS];
    for(Int i=0; i<NUM_PPS_EXTENSION_FLAGS; i++)
    {
      READ_FLAG( uiCode, syntaxStrings[i] );
      pps_extension_flags[i] = uiCode!=0;
    }

    Bool bSkipTrailingExtensionBits=false;
    for(Int i=0; i<NUM_PPS_EXTENSION_FLAGS; i++) // loop used so that the order is determined by the enum.
    {
      if (pps_extension_flags[i])
      {
        switch (PPSExtensionFlagIndex(i))
        {
          case PPS_EXT__REXT:
            {
              TComPPSRExt &ppsRangeExtension = pcPPS->getPpsRangeExtension();
              assert(!bSkipTrailingExtensionBits);

              if (pcPPS->getUseTransformSkip())
              {
                READ_UVLC( uiCode, "log2_max_transform_skip_block_size_minus2");
                ppsRangeExtension.setLog2MaxTransformSkipBlockSize(uiCode+2);
              }

              READ_FLAG( uiCode, "cross_component_prediction_enabled_flag");
              ppsRangeExtension.setCrossComponentPredictionEnabledFlag(uiCode != 0);

              READ_FLAG( uiCode, "chroma_qp_offset_list_enabled_flag");
              if (uiCode == 0)
              {
                ppsRangeExtension.clearChromaQpOffsetList();
                ppsRangeExtension.setDiffCuChromaQpOffsetDepth(0);
              }
              else
              {
                READ_UVLC(uiCode, "diff_cu_chroma_qp_offset_depth"); ppsRangeExtension.setDiffCuChromaQpOffsetDepth(uiCode);
                UInt tableSizeMinus1 = 0;
                READ_UVLC(tableSizeMinus1, "chroma_qp_offset_list_len_minus1");
                assert(tableSizeMinus1 < MAX_QP_OFFSET_LIST_SIZE);

                for (Int cuChromaQpOffsetIdx = 0; cuChromaQpOffsetIdx <= (tableSizeMinus1); cuChromaQpOffsetIdx++)
                {
                  Int cbOffset;
                  Int crOffset;
                  READ_SVLC(cbOffset, "cb_qp_offset_list[i]");
                  assert(cbOffset >= -12 && cbOffset <= 12);
                  READ_SVLC(crOffset, "cr_qp_offset_list[i]");
                  assert(crOffset >= -12 && crOffset <= 12);
                  // table uses +1 for index (see comment inside the function)
                  ppsRangeExtension.setChromaQpOffsetListEntry(cuChromaQpOffsetIdx+1, cbOffset, crOffset);
                }
                assert(ppsRangeExtension.getChromaQpOffsetListLen() == tableSizeMinus1 + 1);
              }

              READ_UVLC( uiCode, "log2_sao_offset_scale_luma");
              ppsRangeExtension.setLog2SaoOffsetScale(CHANNEL_TYPE_LUMA, uiCode);
              READ_UVLC( uiCode, "log2_sao_offset_scale_chroma");
              ppsRangeExtension.setLog2SaoOffsetScale(CHANNEL_TYPE_CHROMA, uiCode);
            }
            break;

#if SVC_EXTENSION
          case PPS_EXT__MLAYER:
            READ_FLAG( uiCode, "poc_reset_info_present_flag" );
            pcPPS->setPocResetInfoPresentFlag(uiCode ? true : false);

            READ_FLAG( uiCode, "pps_infer_scaling_list_flag" );
            pcPPS->setInferScalingListFlag( uiCode );

            if( pcPPS->getInferScalingListFlag() )
            {
              READ_CODE( 6, uiCode, "pps_scaling_list_ref_layer_id" ); 
              pcPPS->setScalingListRefLayerId( uiCode );
              // The value of pps_scaling_list_ref_layer_id shall be in the range of 0 to 62, inclusive
              assert( pcPPS->getScalingListRefLayerId() <= 62 );
              pcPPS->setScalingListPresentFlag( false );
            }

            READ_UVLC( uiCode,      "num_ref_loc_offsets" ); pcPPS->setNumRefLayerLocationOffsets(uiCode);
            for(Int k = 0; k < pcPPS->getNumRefLayerLocationOffsets(); k++)
            {
              READ_CODE( 6, uiCode,  "ref_loc_offset_layer_id" );  pcPPS->setRefLocationOffsetLayerId( k, uiCode );
              READ_FLAG( uiCode, "scaled_ref_layer_offset_present_flag" );   pcPPS->setScaledRefLayerOffsetPresentFlag( k, uiCode );
              if (uiCode)
              {
                Window& scaledWindow = pcPPS->getScaledRefLayerWindow(k);
                READ_SVLC( iCode, "scaled_ref_layer_left_offset" );    scaledWindow.setWindowLeftOffset  (iCode << 1);
                READ_SVLC( iCode, "scaled_ref_layer_top_offset" );     scaledWindow.setWindowTopOffset   (iCode << 1);
                READ_SVLC( iCode, "scaled_ref_layer_right_offset" );   scaledWindow.setWindowRightOffset (iCode << 1);
                READ_SVLC( iCode, "scaled_ref_layer_bottom_offset" );  scaledWindow.setWindowBottomOffset(iCode << 1);
              }
              READ_FLAG( uiCode, "ref_region_offset_present_flag" );   pcPPS->setRefRegionOffsetPresentFlag( k, uiCode );
              if (uiCode)
              {
                Window& refWindow = pcPPS->getRefLayerWindow(k);
                READ_SVLC( iCode, "ref_region_left_offset" );    refWindow.setWindowLeftOffset  (iCode << 1);
                READ_SVLC( iCode, "ref_region_top_offset" );     refWindow.setWindowTopOffset   (iCode << 1);
                READ_SVLC( iCode, "ref_region_right_offset" );   refWindow.setWindowRightOffset (iCode << 1);
                READ_SVLC( iCode, "ref_region_bottom_offset" );  refWindow.setWindowBottomOffset(iCode << 1);
              }
              READ_FLAG( uiCode, "resample_phase_set_present_flag" );   pcPPS->setResamplePhaseSetPresentFlag( k, uiCode );
              if (uiCode)
              {
                READ_UVLC( uiCode, "phase_hor_luma" );    pcPPS->setPhaseHorLuma ( k, uiCode );
                READ_UVLC( uiCode, "phase_ver_luma" );    pcPPS->setPhaseVerLuma ( k, uiCode );
                READ_UVLC( uiCode, "phase_hor_chroma_plus8" );  pcPPS->setPhaseHorChroma (k, uiCode - 8);
                READ_UVLC( uiCode, "phase_ver_chroma_plus8" );  pcPPS->setPhaseVerChroma (k, uiCode - 8);
              }
            }
#if CGS_3D_ASYMLUT
            READ_FLAG( uiCode , "colour_mapping_enabled_flag" ); 
            pcPPS->setCGSFlag( uiCode );
            if( pcPPS->getCGSFlag() )
            {
              // when pps_pic_parameter_set_id greater than or equal to 8, colour_mapping_enabled_flag shall be equal to 0
              assert( pcPPS->getPPSId() < 8 );

              // xParse3DAsymLUT( pc3DAsymLUT );
              // pcPPS->setCGSOutputBitDepthY( pc3DAsymLUT->getOutputBitDepthY() );
              // pcPPS->setCGSOutputBitDepthC( pc3DAsymLUT->getOutputBitDepthC() );
            }
#endif
            break;
#endif
          default:
            bSkipTrailingExtensionBits=true;
            break;
        }
      }
    }
    if (bSkipTrailingExtensionBits)
    {
      while ( xMoreRbspData() )
      {
        READ_FLAG( uiCode, "pps_extension_data_flag");
      }
    }
  }
  xReadRbspTrailingBits();
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
      READ_CODE(8, uiCode, "matrix_coeffs");                          pcVUI->setMatrixCoefficients(uiCode);
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

    READ_FLAG(     uiCode, "vui_hrd_parameters_present_flag");        pcVUI->setHrdParametersPresentFlag(uiCode);
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
    READ_UVLC(   uiCode, "min_spatial_segmentation_idc");             pcVUI->setMinSpatialSegmentationIdc(uiCode);
    assert(uiCode < 4096);
    READ_UVLC(   uiCode, "max_bytes_per_pic_denom" );                 pcVUI->setMaxBytesPerPicDenom(uiCode);
    READ_UVLC(   uiCode, "max_bits_per_min_cu_denom" );               pcVUI->setMaxBitsPerMinCuDenom(uiCode);
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
      READ_FLAG( uiCode, "sub_pic_hrd_params_present_flag" );         hrd->setSubPicCpbParamsPresentFlag( uiCode == 1 ? true : false );
      if( hrd->getSubPicCpbParamsPresentFlag() )
      {
        READ_CODE( 8, uiCode, "tick_divisor_minus2" );                hrd->setTickDivisorMinus2( uiCode );
        READ_CODE( 5, uiCode, "du_cpb_removal_delay_increment_length_minus1" ); hrd->setDuCpbRemovalDelayLengthMinus1( uiCode );
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
#if SVC_EXTENSION
    else
    {
      hrd->setInitialCpbRemovalDelayLengthMinus1( 23 );
      // Add inferred values for other syntax elements here.
    }
#endif
  }
  Int i, j, nalOrVcl;
  for( i = 0; i <= maxNumSubLayersMinus1; i ++ )
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

Void TDecCavlc::parseSPS(TComSPS* pcSPS)
{
#if ENC_DEC_TRACE
  xTraceSPSHeader ();
#endif

  printf("Parse SPS : pcSPS->getLayerId() = %d \n",pcSPS->getLayerId());
  UInt  uiCode;
  READ_CODE( 4,  uiCode, "sps_video_parameter_set_id");          pcSPS->setVPSId        ( uiCode );
#if SVC_EXTENSION
  UInt uiTmp = 0;
  
  if(pcSPS->getLayerId() == 0)
  {
#endif
  READ_CODE( 3,  uiCode, "sps_max_sub_layers_minus1" );          pcSPS->setMaxTLayers   ( uiCode+1 );
  assert(uiCode <= 6);
#if SVC_EXTENSION
  }
  else
  {
    READ_CODE( 3,  uiCode, "sps_ext_or_max_sub_layers_minus1" );     uiTmp = uiCode;

    if( uiTmp != 7 )
    {
      pcSPS->setMaxTLayers(uiTmp + 1);
    }
  }

  pcSPS->setMultiLayerExtSpsFlag( pcSPS->getLayerId() != 0 && uiTmp == 7 );

  if( !pcSPS->getMultiLayerExtSpsFlag() )
  {
#endif
  READ_FLAG( uiCode, "sps_temporal_id_nesting_flag" );           pcSPS->setTemporalIdNestingFlag ( uiCode > 0 ? true : false );
#if SVC_EXTENSION
   parsePTL(pcSPS->getPTL(), 1, pcSPS->getMaxTLayers() - 1);
  }
#else
  if ( pcSPS->getMaxTLayers() == 1 )
  {
    // sps_temporal_id_nesting_flag must be 1 when sps_max_sub_layers_minus1 is 0
    assert( uiCode == 1 );
  }

  parsePTL(pcSPS->getPTL(), 1, pcSPS->getMaxTLayers() - 1);
#endif

  
  READ_UVLC(     uiCode, "sps_seq_parameter_set_id" );           pcSPS->setSPSId( uiCode );
  assert(uiCode <= 15);

#if SVC_EXTENSION
  if( pcSPS->getMultiLayerExtSpsFlag() )
  {
    READ_FLAG( uiCode, "update_rep_format_flag" );
    pcSPS->setUpdateRepFormatFlag( uiCode ? true : false );
    
    if( pcSPS->getUpdateRepFormatFlag() )
    {
      READ_CODE(8, uiCode, "sps_rep_format_idx");
      pcSPS->setUpdateRepFormatIndex(uiCode);
    }
  }
  else
  {
    pcSPS->setUpdateRepFormatFlag( false );
#endif
  READ_UVLC(     uiCode, "chroma_format_idc" );                  pcSPS->setChromaFormatIdc( ChromaFormat(uiCode) );
  assert(uiCode <= 3);

  if( pcSPS->getChromaFormatIdc() == CHROMA_444 )
  {
    READ_FLAG(     uiCode, "separate_colour_plane_flag");        assert(uiCode == 0);
  }

  READ_UVLC (    uiCode, "pic_width_in_luma_samples" );          pcSPS->setPicWidthInLumaSamples ( uiCode    );
  READ_UVLC (    uiCode, "pic_height_in_luma_samples" );         pcSPS->setPicHeightInLumaSamples( uiCode    );
  READ_FLAG(     uiCode, "conformance_window_flag");
  if (uiCode != 0)
  {
    Window &conf = pcSPS->getConformanceWindow();
#if SVC_EXTENSION
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

  READ_UVLC(     uiCode, "bit_depth_luma_minus8" );
#if O0043_BEST_EFFORT_DECODING
  pcSPS->setStreamBitDepth(CHANNEL_TYPE_LUMA, 8 + uiCode);
  const UInt forceDecodeBitDepth = pcSPS->getForceDecodeBitDepth();
  if (forceDecodeBitDepth != 0)
  {
    uiCode = forceDecodeBitDepth - 8;
  }
#endif
  assert(uiCode <= 8);
  pcSPS->setBitDepth(CHANNEL_TYPE_LUMA, 8 + uiCode);

#if O0043_BEST_EFFORT_DECODING
  pcSPS->setQpBDOffset(CHANNEL_TYPE_LUMA, (Int) (6*(pcSPS->getStreamBitDepth(CHANNEL_TYPE_LUMA)-8)) );
#else
  pcSPS->setQpBDOffset(CHANNEL_TYPE_LUMA, (Int) (6*uiCode) );
#endif

  READ_UVLC( uiCode,    "bit_depth_chroma_minus8" );
#if O0043_BEST_EFFORT_DECODING
  pcSPS->setStreamBitDepth(CHANNEL_TYPE_CHROMA, 8 + uiCode);
  if (forceDecodeBitDepth != 0)
  {
    uiCode = forceDecodeBitDepth - 8;
  }
#endif
  assert(uiCode <= 8);
  pcSPS->setBitDepth(CHANNEL_TYPE_CHROMA, 8 + uiCode);
#if O0043_BEST_EFFORT_DECODING
  pcSPS->setQpBDOffset(CHANNEL_TYPE_CHROMA,  (Int) (6*(pcSPS->getStreamBitDepth(CHANNEL_TYPE_CHROMA)-8)) );
#else
  pcSPS->setQpBDOffset(CHANNEL_TYPE_CHROMA,  (Int) (6*uiCode) );
#endif

#if SVC_EXTENSION
  }
#endif

 return ; 
/*
  READ_UVLC( uiCode,    "log2_max_pic_order_cnt_lsb_minus4" );   pcSPS->setBitsForPOC( 4 + uiCode );
  assert(uiCode <= 12);

#if SVC_EXTENSION
  if( !pcSPS->getMultiLayerExtSpsFlag() )  
  {
#endif
  UInt subLayerOrderingInfoPresentFlag;
  READ_FLAG(subLayerOrderingInfoPresentFlag, "sps_sub_layer_ordering_info_present_flag");

  for(UInt i=0; i <= pcSPS->getMaxTLayers()-1; i++)
  {
    READ_UVLC ( uiCode, "sps_max_dec_pic_buffering_minus1[i]");
    pcSPS->setMaxDecPicBuffering( uiCode + 1, i);
    READ_UVLC ( uiCode, "sps_max_num_reorder_pics[i]" );
    pcSPS->setNumReorderPics(uiCode, i);
    READ_UVLC ( uiCode, "sps_max_latency_increase_plus1[i]");
    pcSPS->setMaxLatencyIncreasePlus1( uiCode, i );

    if (!subLayerOrderingInfoPresentFlag)
    {
      for (i++; i <= pcSPS->getMaxTLayers()-1; i++)
      {
        pcSPS->setMaxDecPicBuffering(pcSPS->getMaxDecPicBuffering(0), i);
        pcSPS->setNumReorderPics(pcSPS->getNumReorderPics(0), i);
        pcSPS->setMaxLatencyIncreasePlus1(pcSPS->getMaxLatencyIncreasePlus1(0), i);
      }
      break;
    }

#if SVC_EXTENSION
    if( i > 0 )
    {
      // When i is greater than 0, sps_max_dec_pic_buffering_minus1[ i ] shall be greater than or equal to sps_max_dec_pic_buffering_minus1[ i - 1 ]. 
      assert( pcSPS->getMaxDecPicBuffering(i) >= pcSPS->getMaxDecPicBuffering(i-1) );
    }
#endif

  }
#if SVC_EXTENSION
  }
#endif
  READ_UVLC( uiCode, "log2_min_luma_coding_block_size_minus3" );
  Int log2MinCUSize = uiCode + 3;
  pcSPS->setLog2MinCodingBlockSize(log2MinCUSize);
  READ_UVLC( uiCode, "log2_diff_max_min_luma_coding_block_size" );
  pcSPS->setLog2DiffMaxMinCodingBlockSize(uiCode);
  
  if (pcSPS->getPTL()->getGeneralPTL()->getLevelIdc() >= Level::LEVEL5)
  {
    assert(log2MinCUSize + pcSPS->getLog2DiffMaxMinCodingBlockSize() >= 5);
  }
  
  Int maxCUDepthDelta = uiCode;
  pcSPS->setMaxCUWidth  ( 1<<(log2MinCUSize + maxCUDepthDelta) );
  pcSPS->setMaxCUHeight ( 1<<(log2MinCUSize + maxCUDepthDelta) );
  READ_UVLC( uiCode, "log2_min_luma_transform_block_size_minus2" );   pcSPS->setQuadtreeTULog2MinSize( uiCode + 2 );

  READ_UVLC( uiCode, "log2_diff_max_min_luma_transform_block_size" ); pcSPS->setQuadtreeTULog2MaxSize( uiCode + pcSPS->getQuadtreeTULog2MinSize() );
  pcSPS->setMaxTrSize( 1<<(uiCode + pcSPS->getQuadtreeTULog2MinSize()) );

  READ_UVLC( uiCode, "max_transform_hierarchy_depth_inter" );    pcSPS->setQuadtreeTUMaxDepthInter( uiCode+1 );
  READ_UVLC( uiCode, "max_transform_hierarchy_depth_intra" );    pcSPS->setQuadtreeTUMaxDepthIntra( uiCode+1 );

  Int addCuDepth = max (0, log2MinCUSize - (Int)pcSPS->getQuadtreeTULog2MinSize() );
  pcSPS->setMaxTotalCUDepth( maxCUDepthDelta + addCuDepth  + getMaxCUDepthOffset(pcSPS->getChromaFormatIdc(), pcSPS->getQuadtreeTULog2MinSize()) );

  READ_FLAG( uiCode, "scaling_list_enabled_flag" );                 pcSPS->setScalingListFlag ( uiCode );
  if(pcSPS->getScalingListFlag())
  {
#if SVC_EXTENSION
    if( pcSPS->getMultiLayerExtSpsFlag() )
    {
      READ_FLAG( uiCode, "sps_infer_scaling_list_flag" ); pcSPS->setInferScalingListFlag( uiCode );
    }

    if( pcSPS->getInferScalingListFlag() )
    {
      READ_CODE( 6, uiCode, "sps_scaling_list_ref_layer_id" ); pcSPS->setScalingListRefLayerId( uiCode );

      // The value of sps_scaling_list_ref_layer_id shall be in the range of 0 to 62, inclusive
      assert( pcSPS->getScalingListRefLayerId() <= 62 );

      pcSPS->setScalingListPresentFlag( false );
    }
    else
    {
#endif
    READ_FLAG( uiCode, "sps_scaling_list_data_present_flag" );                 pcSPS->setScalingListPresentFlag ( uiCode );
    if(pcSPS->getScalingListPresentFlag ())
    {
      parseScalingList( &(pcSPS->getScalingList()) );
    }
#if SVC_EXTENSION
    }
#endif
  }
  READ_FLAG( uiCode, "amp_enabled_flag" );                          pcSPS->setUseAMP( uiCode );
  READ_FLAG( uiCode, "sample_adaptive_offset_enabled_flag" );       pcSPS->setUseSAO ( uiCode ? true : false );

  READ_FLAG( uiCode, "pcm_enabled_flag" ); pcSPS->setUsePCM( uiCode ? true : false );
  if( pcSPS->getUsePCM() )
  {
    READ_CODE( 4, uiCode, "pcm_sample_bit_depth_luma_minus1" );          pcSPS->setPCMBitDepth    ( CHANNEL_TYPE_LUMA, 1 + uiCode );
    READ_CODE( 4, uiCode, "pcm_sample_bit_depth_chroma_minus1" );        pcSPS->setPCMBitDepth    ( CHANNEL_TYPE_CHROMA, 1 + uiCode );
    READ_UVLC( uiCode, "log2_min_pcm_luma_coding_block_size_minus3" );   pcSPS->setPCMLog2MinSize (uiCode+3);
    READ_UVLC( uiCode, "log2_diff_max_min_pcm_luma_coding_block_size" ); pcSPS->setPCMLog2MaxSize ( uiCode+pcSPS->getPCMLog2MinSize() );
    READ_FLAG( uiCode, "pcm_loop_filter_disable_flag" );                 pcSPS->setPCMFilterDisableFlag ( uiCode ? true : false );
  }

  READ_UVLC( uiCode, "num_short_term_ref_pic_sets" );
  assert(uiCode <= 64);
  pcSPS->createRPSList(uiCode);

  TComRPSList* rpsList = pcSPS->getRPSList();
  TComReferencePictureSet* rps;

  for(UInt i=0; i< rpsList->getNumberOfReferencePictureSets(); i++)
  {
    rps = rpsList->getReferencePictureSet(i);
    parseShortTermRefPicSet(pcSPS,rps,i);
  }
  READ_FLAG( uiCode, "long_term_ref_pics_present_flag" );          pcSPS->setLongTermRefsPresent(uiCode);
  if (pcSPS->getLongTermRefsPresent())
  {
    READ_UVLC( uiCode, "num_long_term_ref_pics_sps" );
    pcSPS->setNumLongTermRefPicSPS(uiCode);
    for (UInt k = 0; k < pcSPS->getNumLongTermRefPicSPS(); k++)
    {
      READ_CODE( pcSPS->getBitsForPOC(), uiCode, "lt_ref_pic_poc_lsb_sps" );
      pcSPS->setLtRefPicPocLsbSps(k, uiCode);
      READ_FLAG( uiCode,  "used_by_curr_pic_lt_sps_flag[i]");
      pcSPS->setUsedByCurrPicLtSPSFlag(k, uiCode?1:0);
    }
  }
  READ_FLAG( uiCode, "sps_temporal_mvp_enabled_flag" );           pcSPS->setSPSTemporalMVPEnabledFlag(uiCode);

  READ_FLAG( uiCode, "strong_intra_smoothing_enable_flag" );      pcSPS->setUseStrongIntraSmoothing(uiCode);

  READ_FLAG( uiCode, "vui_parameters_present_flag" );             pcSPS->setVuiParametersPresentFlag(uiCode);

  if (pcSPS->getVuiParametersPresentFlag())
  {
    parseVUI(pcSPS->getVuiParameters(), pcSPS);
  }

  READ_FLAG( uiCode, "sps_extension_present_flag");

#if SVC_EXTENSION
  pcSPS->setExtensionFlag( uiCode ? true : false );

  if( pcSPS->getExtensionFlag() )
#else
  if (uiCode)
#endif
  {
#if ENC_DEC_TRACE || RExt__DECODER_DEBUG_BIT_STATISTICS
    static const TChar *syntaxStrings[]={ "sps_range_extension_flag",
                                          "sps_multilayer_extension_flag",
                                          "sps_extension_6bits[0]",
                                          "sps_extension_6bits[1]",
                                          "sps_extension_6bits[2]",
                                          "sps_extension_6bits[3]",
                                          "sps_extension_6bits[4]",
                                          "sps_extension_6bits[5]" };
#endif
    Bool sps_extension_flags[NUM_SPS_EXTENSION_FLAGS];

    for(Int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++)
    {
      READ_FLAG( uiCode, syntaxStrings[i] );
      sps_extension_flags[i] = uiCode!=0;
    }

    Bool bSkipTrailingExtensionBits=false;
    for(Int i=0; i<NUM_SPS_EXTENSION_FLAGS; i++) // loop used so that the order is determined by the enum.
    {
      if (sps_extension_flags[i])
      {
        switch (SPSExtensionFlagIndex(i))
        {
          case SPS_EXT__REXT:
            assert(!bSkipTrailingExtensionBits);
            {
              TComSPSRExt &spsRangeExtension = pcSPS->getSpsRangeExtension();
              READ_FLAG( uiCode, "transform_skip_rotation_enabled_flag");     spsRangeExtension.setTransformSkipRotationEnabledFlag(uiCode != 0);
              READ_FLAG( uiCode, "transform_skip_context_enabled_flag");      spsRangeExtension.setTransformSkipContextEnabledFlag (uiCode != 0);
              READ_FLAG( uiCode, "implicit_rdpcm_enabled_flag");              spsRangeExtension.setRdpcmEnabledFlag(RDPCM_SIGNAL_IMPLICIT, (uiCode != 0));
              READ_FLAG( uiCode, "explicit_rdpcm_enabled_flag");              spsRangeExtension.setRdpcmEnabledFlag(RDPCM_SIGNAL_EXPLICIT, (uiCode != 0));
              READ_FLAG( uiCode, "extended_precision_processing_flag");       spsRangeExtension.setExtendedPrecisionProcessingFlag (uiCode != 0);
              READ_FLAG( uiCode, "intra_smoothing_disabled_flag");            spsRangeExtension.setIntraSmoothingDisabledFlag      (uiCode != 0);
              READ_FLAG( uiCode, "high_precision_offsets_enabled_flag");      spsRangeExtension.setHighPrecisionOffsetsEnabledFlag (uiCode != 0);
              READ_FLAG( uiCode, "persistent_rice_adaptation_enabled_flag");  spsRangeExtension.setPersistentRiceAdaptationEnabledFlag (uiCode != 0);
              READ_FLAG( uiCode, "cabac_bypass_alignment_enabled_flag");      spsRangeExtension.setCabacBypassAlignmentEnabledFlag  (uiCode != 0);
            }
            break;
#if SVC_EXTENSION
          case SPS_EXT__MLAYER:
            parseSPSExtension( pcSPS );
            break;
#endif
          default:
            bSkipTrailingExtensionBits=true;
            break;
        }
      }
    }
    if (bSkipTrailingExtensionBits)
    {
      while ( xMoreRbspData() )
      {
        READ_FLAG( uiCode, "sps_extension_data_flag");
      }
    }
  }

  xReadRbspTrailingBits();
  */
}

Void TDecCavlc::parseVPS(TComVPS* pcVPS)
{
#if ENC_DEC_TRACE
  xTraceVPSHeader ();
#endif
  UInt  uiCode;
printf("parseVPS \n");
  READ_CODE( 4,  uiCode,  "vps_video_parameter_set_id" );         pcVPS->setVPSId( uiCode );
#if SVC_EXTENSION
  READ_FLAG( uiCode, "vps_base_layer_internal_flag");             pcVPS->setBaseLayerInternalFlag( uiCode ? true : false );
  READ_FLAG( uiCode, "vps_base_layer_available_flag");            pcVPS->setBaseLayerAvailableFlag( uiCode ? true : false );
  pcVPS->setNonHEVCBaseLayerFlag( (pcVPS->getBaseLayerAvailableFlag() && !pcVPS->getBaseLayerInternalFlag()) ? true : false);

  READ_CODE( 6,  uiCode,  "vps_max_layers_minus1" );              pcVPS->setMaxLayers( min( 62u, uiCode) + 1 );
  assert( pcVPS->getBaseLayerInternalFlag() || pcVPS->getMaxLayers() > 1 );
#else
  READ_FLAG( uiCode,      "vps_base_layer_internal_flag" );       assert(uiCode == 1);
  READ_FLAG( uiCode,      "vps_base_layer_available_flag" );      assert(uiCode == 1);
  READ_CODE( 6,  uiCode,  "vps_max_layers_minus1" );
#endif
  READ_CODE( 3,  uiCode,  "vps_max_sub_layers_minus1" );          pcVPS->setMaxTLayers( uiCode + 1 );    assert(uiCode+1 <= MAX_TLAYER);
  READ_FLAG(     uiCode,  "vps_temporal_id_nesting_flag" );       pcVPS->setTemporalNestingFlag( uiCode ? true:false );
  assert (pcVPS->getMaxTLayers()>1||pcVPS->getTemporalNestingFlag());
  READ_CODE( 16, uiCode,  "vps_reserved_0xffff_16bits" );         assert(uiCode == 0xffff);
  parsePTL ( pcVPS->getPTL(), true, pcVPS->getMaxTLayers()-1);
  UInt subLayerOrderingInfoPresentFlag;
  READ_FLAG(subLayerOrderingInfoPresentFlag, "vps_sub_layer_ordering_info_present_flag");
  for(UInt i = 0; i <= pcVPS->getMaxTLayers()-1; i++)
  {
    READ_UVLC( uiCode,  "vps_max_dec_pic_buffering_minus1[i]" );    pcVPS->setMaxDecPicBuffering( uiCode + 1, i );
    READ_UVLC( uiCode,  "vps_max_num_reorder_pics[i]" );            pcVPS->setNumReorderPics( uiCode, i );
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
  assert( pcVPS->getMaxLayerId()       < MAX_NUM_LAYER_IDS );
  READ_CODE( 6, uiCode, "vps_max_layer_id" );           pcVPS->setMaxLayerId( uiCode );
  READ_UVLC(uiCode, "vps_num_layer_sets_minus1");  pcVPS->setVpsNumLayerSetsMinus1(uiCode);
  pcVPS->setNumLayerSets(pcVPS->getVpsNumLayerSetsMinus1() + 1);

  for (UInt opsIdx = 1; opsIdx <= pcVPS->getVpsNumLayerSetsMinus1(); opsIdx++)
  {
    // Operation point set
    for( UInt i = 0; i <= pcVPS->getMaxLayerId(); i ++ )
#else
  assert( pcVPS->getNumHrdParameters() < MAX_VPS_OP_SETS_PLUS1 );
  assert( pcVPS->getMaxNuhReservedZeroLayerId() < MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1 );
  READ_CODE( 6, uiCode, "vps_max_layer_id" );                        pcVPS->setMaxNuhReservedZeroLayerId( uiCode );
  READ_UVLC(    uiCode, "vps_num_layer_sets_minus1" );               pcVPS->setMaxOpSets( uiCode + 1 );
  for( UInt opsIdx = 1; opsIdx <= ( pcVPS->getMaxOpSets() - 1 ); opsIdx ++ )
  {
    // Operation point set
    for( UInt i = 0; i <= pcVPS->getMaxNuhReservedZeroLayerId(); i ++ )
#endif
    {
      READ_FLAG( uiCode, "layer_id_included_flag[opsIdx][i]" );   pcVPS->setLayerIdIncludedFlag( uiCode == 1 ? true : false, opsIdx, i );
    }
  }

#if SVC_EXTENSION
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
      READ_UVLC( uiCode, "hrd_layer_set_idx[i]" );                  pcVPS->setHrdOpSetIdx( uiCode, i );
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
    while ( m_pcBitstream.getNumBitsRead() % 8 != 0 )
    {
      READ_FLAG( uiCode, "vps_extension_alignment_bit_equal_to_one"); assert(uiCode == 1);
    }
    parseVPSExtension(pcVPS);
    READ_FLAG( uiCode, "vps_extension2_flag" );
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

  xReadRbspTrailingBits();
}

Void TDecCavlc::parseSliceHeader (TComSlice* pcSlice, ParameterSetManager *parameterSetManager, const Int prevTid0POC)
{
  UInt  uiCode;
  Int   iCode;

#if ENC_DEC_TRACE
  xTraceSliceHeader();
#endif
  TComPPS* pps = NULL;
  TComSPS* sps = NULL;

  UInt firstSliceSegmentInPic;
  READ_FLAG( firstSliceSegmentInPic, "first_slice_segment_in_pic_flag" );

#if SVC_EXTENSION
  pcSlice->setFirstSliceInPic( firstSliceSegmentInPic );
#endif

  if( pcSlice->getRapPicFlag())
  {
    READ_FLAG( uiCode, "no_output_of_prior_pics_flag" );  //ignored -- updated already
    pcSlice->setNoOutputPriorPicsFlag(uiCode ? true : false);
  }
  READ_UVLC (    uiCode, "slice_pic_parameter_set_id" );  pcSlice->setPPSId(uiCode);
  pps = parameterSetManager->getPPS(uiCode);
  //!KS: need to add error handling code here, if PPS is not available
  assert(pps!=0);
  sps = parameterSetManager->getSPS(pps->getSPSId());
  //!KS: need to add error handling code here, if SPS is not available
  assert(sps!=0);

#if SVC_EXTENSION
  pcSlice->setSPS(sps);
  pcSlice->setPPS(pps);

  TComVPS* vps = parameterSetManager->getVPS(sps->getVPSId());
  pcSlice->setVPS(vps);

  Int iPOClsb = 0;
#endif

  const ChromaFormat chFmt = sps->getChromaFormatIdc();
  const UInt numValidComp=getNumberValidComponents(chFmt);
  const Bool bChroma=(chFmt!=CHROMA_400);

  if( pps->getDependentSliceSegmentsEnabledFlag() && ( !firstSliceSegmentInPic ))
  {
    READ_FLAG( uiCode, "dependent_slice_segment_flag" );       pcSlice->setDependentSliceSegmentFlag(uiCode ? true : false);
  }
  else
  {
    pcSlice->setDependentSliceSegmentFlag(false);
  }
  Int numCTUs = ((sps->getPicWidthInLumaSamples()+sps->getMaxCUWidth()-1)/sps->getMaxCUWidth())*((sps->getPicHeightInLumaSamples()+sps->getMaxCUHeight()-1)/sps->getMaxCUHeight());
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
  pcSlice->setSliceSegmentCurStartCtuTsAddr( sliceSegmentAddress );// this is actually a Raster-Scan (RS) address, but we do not have the RS->TS conversion table defined yet.
  pcSlice->setSliceSegmentCurEndCtuTsAddr(numCTUs);                // Set end as the last CTU of the picture.

  if (!pcSlice->getDependentSliceSegmentFlag())
  {
    pcSlice->setSliceCurStartCtuTsAddr(sliceSegmentAddress); // this is actually a Raster-Scan (RS) address, but we do not have the RS->TS conversion table defined yet.
    pcSlice->setSliceCurEndCtuTsAddr(numCTUs);
  }

  if(!pcSlice->getDependentSliceSegmentFlag())
  {
#if SVC_EXTENSION
    Int iBits = 0;
    if(pps->getNumExtraSliceHeaderBits() > iBits)
    {
      READ_FLAG(uiCode, "discardable_flag");
      pcSlice->setDiscardableFlag( uiCode ? true : false );

      if( uiCode )
      {
        assert(pcSlice->getNalUnitType() != NAL_UNIT_CODED_SLICE_TRAIL_R &&
          pcSlice->getNalUnitType() != NAL_UNIT_CODED_SLICE_TSA_R &&
          pcSlice->getNalUnitType() != NAL_UNIT_CODED_SLICE_STSA_R &&
          pcSlice->getNalUnitType() != NAL_UNIT_CODED_SLICE_RADL_R &&
          pcSlice->getNalUnitType() != NAL_UNIT_CODED_SLICE_RASL_R);
      }

      iBits++;
    }

    if(pps->getNumExtraSliceHeaderBits() > iBits)
    {
      READ_FLAG(uiCode, "cross_layer_bla_flag");  pcSlice->setCrossLayerBLAFlag( uiCode ? true : false );
      iBits++;
    }

    for ( ; iBits < pps->getNumExtraSliceHeaderBits(); iBits++)
    {
      READ_FLAG(uiCode, "slice_reserved_undetermined_flag[]"); // ignored
    }
#else //SVC_EXTENSION
    for (Int i = 0; i < pps->getNumExtraSliceHeaderBits(); i++)
    {
      READ_FLAG(uiCode, "slice_reserved_flag[]"); // ignored
    }
#endif //SVC_EXTENSION

    READ_UVLC (    uiCode, "slice_type" );            pcSlice->setSliceType((SliceType)uiCode);
    if( pps->getOutputFlagPresentFlag() )
    {
      READ_FLAG( uiCode, "pic_output_flag" );    pcSlice->setPicOutputFlag( uiCode ? true : false );
    }
    else
    {
      pcSlice->setPicOutputFlag( true );
    }

    // if (separate_colour_plane_flag == 1)
    //   read colour_plane_id
    //   (separate_colour_plane_flag == 1) is not supported in this version of the standard.

    if( pcSlice->getIdrPicFlag() )
    {
      pcSlice->setPOC(0);
      TComReferencePictureSet* rps = pcSlice->getLocalRPS();
      (*rps)=TComReferencePictureSet();
      pcSlice->setRPS(rps);
    }

#if SVC_EXTENSION
    if( ( pcSlice->getLayerId() > 0 && !vps->getPocLsbNotPresentFlag( vps->getLayerIdxInVps(pcSlice->getLayerId())) ) || !pcSlice->getIdrPicFlag() )
#else
    else
#endif
    {
      READ_CODE(sps->getBitsForPOC(), uiCode, "slice_pic_order_cnt_lsb");
#if SVC_EXTENSION
      pcSlice->setPicOrderCntLsb( uiCode );

      iPOClsb = uiCode;
#else
      Int iPOClsb = uiCode;
#endif
      Int iPrevPOC = prevTid0POC;
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
      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP )
      {
        // For BLA picture types, POCmsb is set to 0.
        iPOCmsb = 0;
      }
      pcSlice->setPOC              (iPOCmsb+iPOClsb);

#if SVC_EXTENSION
    }
    else
    {
      pcSlice->setPicOrderCntLsb( 0 );
    }

    if( !pcSlice->getIdrPicFlag() )
    {
#endif
      TComReferencePictureSet* rps;
      rps = pcSlice->getLocalRPS();
      (*rps)=TComReferencePictureSet();

      pcSlice->setRPS(rps);
      READ_FLAG( uiCode, "short_term_ref_pic_set_sps_flag" );
      if(uiCode == 0) // use short-term reference picture set explicitly signalled in slice header
      {
        parseShortTermRefPicSet(sps,rps, sps->getRPSList()->getNumberOfReferencePictureSets());
      }
      else // use reference to short-term reference picture set in PPS
      {
        Int numBits = 0;
        while ((1 << numBits) < sps->getRPSList()->getNumberOfReferencePictureSets())
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
        if (sps->getNumLongTermRefPicSPS() > 0)
        {
          READ_UVLC( uiCode, "num_long_term_sps");
          numLtrpInSPS = uiCode;
          numOfLtrp += numLtrpInSPS;
          rps->setNumberOfLongtermPictures(numOfLtrp);
        }
        Int bitsForLtrpInSPS = 0;
        while (sps->getNumLongTermRefPicSPS() > (1 << bitsForLtrpInSPS))
        {
          bitsForLtrpInSPS++;
        }
        READ_UVLC( uiCode, "num_long_term_pics");             rps->setNumberOfLongtermPictures(uiCode);
        numOfLtrp += uiCode;
        rps->setNumberOfLongtermPictures(numOfLtrp);
        Int maxPicOrderCntLSB = 1 << sps->getBitsForPOC();
        Int prevDeltaMSB = 0, deltaPocMSBCycleLT = 0;
        for(Int j=offset+rps->getNumberOfLongtermPictures()-1, k = 0; k < numOfLtrp; j--, k++)
        {
          Int pocLsbLt;
          if (k < numLtrpInSPS)
          {
            uiCode = 0;
            if (bitsForLtrpInSPS > 0)
            {
              READ_CODE(bitsForLtrpInSPS, uiCode, "lt_idx_sps[i]");
            }
            Bool usedByCurrFromSPS=sps->getUsedByCurrPicLtSPSFlag(uiCode);

            pocLsbLt = sps->getLtRefPicPocLsbSps(uiCode);
            rps->setUsed(j,usedByCurrFromSPS);
          }
          else
          {
            READ_CODE(sps->getBitsForPOC(), uiCode, "poc_lsb_lt"); pocLsbLt= uiCode;
            READ_FLAG( uiCode, "used_by_curr_pic_lt_flag");     rps->setUsed(j,uiCode);
          }
          READ_FLAG(uiCode,"delta_poc_msb_present_flag");
          Bool mSBPresentFlag = uiCode ? true : false;
          if(mSBPresentFlag)
          {
            READ_UVLC( uiCode, "delta_poc_msb_cycle_lt[i]" );
            Bool deltaFlag = false;
            //            First LTRP                               || First LTRP from SH
            if( (j == offset+rps->getNumberOfLongtermPictures()-1) || (j == offset+(numOfLtrp-numLtrpInSPS)-1) )
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

            Int pocLTCurr = pcSlice->getPOC() - deltaPocMSBCycleLT * maxPicOrderCntLSB
                                        - iPOClsb + pocLsbLt;
            rps->setPOC     (j, pocLTCurr);
            rps->setDeltaPOC(j, - pcSlice->getPOC() + pocLTCurr);
            rps->setCheckLTMSBPresent(j,true);
          }
          else
          {
            rps->setPOC     (j, pocLsbLt);
            rps->setDeltaPOC(j, - pcSlice->getPOC() + pocLsbLt);
            rps->setCheckLTMSBPresent(j,false);

            // reset deltaPocMSBCycleLT for first LTRP from slice header if MSB not present
            if( j == offset+(numOfLtrp-numLtrpInSPS)-1 )
            {
              deltaPocMSBCycleLT = 0;
            }
          }
          prevDeltaMSB = deltaPocMSBCycleLT;
        }
        offset += rps->getNumberOfLongtermPictures();
        rps->setNumberOfPictures(offset);
      }

#if SVC_EXTENSION
      // DPB constraints
      if( pcSlice->getVPS()->getVpsExtensionFlag() == 1 )
      {
        for( Int ii = 1; ii < (pcSlice->getVPS()->getVpsNumLayerSetsMinus1() + 1); ii++ )  // prevent assert error when num_add_layer_sets > 0
        {
          Int layerSetIdxForOutputLayerSet = pcSlice->getVPS()->getOutputLayerSetIdx( ii );
          Int chkAssert=0;
          for(Int kk = 0; kk < pcSlice->getVPS()->getNumLayersInIdList(layerSetIdxForOutputLayerSet); kk++)
          {
            if( pcSlice->getVPS()->getNecessaryLayerFlag(ii, kk) && pcSlice->getLayerId() == pcSlice->getVPS()->getLayerSetLayerIdList(layerSetIdxForOutputLayerSet, kk) )
            {
              chkAssert=1;
            }
          }

          if( chkAssert )
          {
            UInt layerIdc = pcSlice->getVPS()->getLayerIdcForOls( ii, pcSlice->getLayerId() );
            assert(rps->getNumberOfNegativePictures() <= pcSlice->getVPS()->getMaxVpsDecPicBufferingMinus1(ii, layerIdc, pcSlice->getVPS()->getMaxSLayersInLayerSetMinus1(ii)));
            assert(rps->getNumberOfPositivePictures() <= pcSlice->getVPS()->getMaxVpsDecPicBufferingMinus1(ii, layerIdc, pcSlice->getVPS()->getMaxSLayersInLayerSetMinus1(ii)) - rps->getNumberOfNegativePictures());
            assert((rps->getNumberOfPositivePictures() + rps->getNumberOfNegativePictures() + rps->getNumberOfLongtermPictures()) <= pcSlice->getVPS()->getMaxVpsDecPicBufferingMinus1(ii, layerIdc, pcSlice->getVPS()->getMaxSLayersInLayerSetMinus1(ii)));
          }
        }
      }

      if(pcSlice->getLayerId() == 0)
      {
        assert(rps->getNumberOfNegativePictures() <= pcSlice->getSPS()->getMaxDecPicBuffering(pcSlice->getSPS()->getMaxTLayers()-1) );
        assert(rps->getNumberOfPositivePictures() <= pcSlice->getSPS()->getMaxDecPicBuffering(pcSlice->getSPS()->getMaxTLayers()-1) -rps->getNumberOfNegativePictures());
        assert((rps->getNumberOfPositivePictures() + rps->getNumberOfNegativePictures() + rps->getNumberOfLongtermPictures()) <= pcSlice->getSPS()->getMaxDecPicBuffering(pcSlice->getSPS()->getMaxTLayers()-1));
      }
#endif

      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL
        || pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP )
      {
        // In the case of BLA picture types, rps data is read from slice header but ignored
        rps = pcSlice->getLocalRPS();
        (*rps)=TComReferencePictureSet();
        pcSlice->setRPS(rps);
      }
      if (sps->getSPSTemporalMVPEnabledFlag())
      {
        READ_FLAG( uiCode, "slice_temporal_mvp_enabled_flag" );
        pcSlice->setEnableTMVPFlag( uiCode == 1 ? true : false );
      }
      else
      {
        pcSlice->setEnableTMVPFlag(false);
      }
    }

#if SVC_EXTENSION
    pcSlice->setActiveNumILRRefIdx(0);
    if((pcSlice->getLayerId() > 0) && !(vps->getDefaultRefLayersActiveFlag()) && (pcSlice->getNumILRRefIdx() > 0) )
    {
      READ_FLAG(uiCode,"inter_layer_pred_enabled_flag");
      pcSlice->setInterLayerPredEnabledFlag(uiCode);
      if( pcSlice->getInterLayerPredEnabledFlag())
      {
        if(pcSlice->getNumILRRefIdx() > 1)
        {
          Int numBits = 1;
          while ((1 << numBits) < pcSlice->getNumILRRefIdx())
          {
            numBits++;
          }
          if( !vps->getMaxOneActiveRefLayerFlag())
          {
            READ_CODE( numBits, uiCode,"num_inter_layer_ref_pics_minus1" );
            pcSlice->setActiveNumILRRefIdx(uiCode + 1);
          }
          else
          {
            for( Int i = 0; i < pcSlice->getNumILRRefIdx(); i++ ) 
            {
              if( ( vps->getMaxTidIlRefPicsPlus1(vps->getLayerIdxInVps(i), pcSlice->getLayerIdx()) > pcSlice->getTLayer() || pcSlice->getTLayer()==0 ) &&
                    vps->getMaxTSLayersMinus1(vps->getLayerIdxInVps(i)) >=  pcSlice->getTLayer() )
              {          
                pcSlice->setActiveNumILRRefIdx(1);
                break;
              }
            }
          }

          if( pcSlice->getActiveNumILRRefIdx() == pcSlice->getNumILRRefIdx() )
          {
            for( Int i = 0; i < pcSlice->getActiveNumILRRefIdx(); i++ )
            {
              pcSlice->setInterLayerPredLayerIdc(i,i);
            }
          }
          else
          {
            for(Int i = 0; i < pcSlice->getActiveNumILRRefIdx(); i++ )
            {
              READ_CODE( numBits,uiCode,"inter_layer_pred_layer_idc[i]" );
              pcSlice->setInterLayerPredLayerIdc(uiCode, i);
            }
          }
        }
        else
        {
          Int refLayerId = vps->getRefLayerId(pcSlice->getLayerId(), 0);
          Int refLayerIdx = vps->getLayerIdxInVps(refLayerId);

          if( ( vps->getMaxTidIlRefPicsPlus1(refLayerIdx, pcSlice->getLayerIdx()) > pcSlice->getTLayer() || pcSlice->getTLayer()==0 ) &&
                vps->getMaxTSLayersMinus1(refLayerIdx) >=  pcSlice->getTLayer() )
          {
            pcSlice->setActiveNumILRRefIdx(1);
            pcSlice->setInterLayerPredLayerIdc(0, 0);
          }
        }
      }
    }
    else if( vps->getDefaultRefLayersActiveFlag() == true &&  (pcSlice->getLayerId() > 0 ))
    {
      pcSlice->setInterLayerPredEnabledFlag(true);

      Int   numRefLayerPics = 0;
      Int   i = 0;
      Int   refLayerPicIdc[MAX_VPS_LAYER_IDX_PLUS1];
      for(i = 0, numRefLayerPics = 0;  i < pcSlice->getNumILRRefIdx(); i++ ) 
      {
        if( ( vps->getMaxTidIlRefPicsPlus1(vps->getLayerIdxInVps(i), pcSlice->getLayerIdx()) > pcSlice->getTLayer() || pcSlice->getTLayer()==0 ) &&
              vps->getMaxTSLayersMinus1(vps->getLayerIdxInVps(i)) >= pcSlice->getTLayer() )
        {
          refLayerPicIdc[ numRefLayerPics++ ] = i;
        }
      }
      pcSlice->setActiveNumILRRefIdx(numRefLayerPics);
      for( i = 0; i < pcSlice->getActiveNumILRRefIdx(); i++ )
      {
        pcSlice->setInterLayerPredLayerIdc(refLayerPicIdc[i], i);
      }
    }
#endif //SVC_EXTENSION

    if(sps->getUseSAO())
    {
      READ_FLAG(uiCode, "slice_sao_luma_flag");  pcSlice->setSaoEnabledFlag(CHANNEL_TYPE_LUMA, (Bool)uiCode);
#if SVC_EXTENSION
      ChromaFormat format;
      if( sps->getLayerId() == 0 )
      {
        format = sps->getChromaFormatIdc();
      }
      else
      {
        format = vps->getVpsRepFormat( sps->getUpdateRepFormatFlag() ? sps->getUpdateRepFormatIndex() : vps->getVpsRepFormatIdx( vps->getLayerIdxInVps(sps->getLayerId()) ) )->getChromaFormatVpsIdc();

        // conformance check
        assert( (sps->getUpdateRepFormatFlag()==false && vps->getVpsNumRepFormats()==1) || vps->getVpsNumRepFormats() > 1 ); 
      }
      if (format != CHROMA_400)
#else
      if (bChroma)
#endif
      {
        READ_FLAG(uiCode, "slice_sao_chroma_flag");  pcSlice->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, (Bool)uiCode);
      }
#if SVC_EXTENSION
      else
      {
        pcSlice->setSaoEnabledFlag(CHANNEL_TYPE_CHROMA, false);
      }
#endif
    }

    if (pcSlice->getIdrPicFlag())
    {
      pcSlice->setEnableTMVPFlag(false);
    }
    if (!pcSlice->isIntra())
    {

      READ_FLAG( uiCode, "num_ref_idx_active_override_flag");
      if (uiCode)
      {
        READ_UVLC (uiCode, "num_ref_idx_l0_active_minus1" );  pcSlice->setNumRefIdx( REF_PIC_LIST_0, uiCode + 1 );
        if (pcSlice->isInterB())
        {
          READ_UVLC (uiCode, "num_ref_idx_l1_active_minus1" );  pcSlice->setNumRefIdx( REF_PIC_LIST_1, uiCode + 1 );
        }
        else
        {
          pcSlice->setNumRefIdx(REF_PIC_LIST_1, 0);
        }
      }
      else
      {
        pcSlice->setNumRefIdx(REF_PIC_LIST_0, pps->getNumRefIdxL0DefaultActive());
        if (pcSlice->isInterB())
        {
          pcSlice->setNumRefIdx(REF_PIC_LIST_1, pps->getNumRefIdxL1DefaultActive());
        }
        else
        {
          pcSlice->setNumRefIdx(REF_PIC_LIST_1,0);
        }
      }
    }
    // }
    TComRefPicListModification* refPicListModification = pcSlice->getRefPicListModification();
    if(!pcSlice->isIntra())
    {
      if( !pps->getListsModificationPresentFlag() || pcSlice->getNumRpsCurrTempList() <= 1 )
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
        Int numRpsCurrTempList0 = pcSlice->getNumRpsCurrTempList();
        if ( numRpsCurrTempList0 > 1 )
        {
          Int length = 1;
          numRpsCurrTempList0 --;
          while ( numRpsCurrTempList0 >>= 1)
          {
            length ++;
          }
          for (i = 0; i < pcSlice->getNumRefIdx(REF_PIC_LIST_0); i ++)
          {
            READ_CODE( length, uiCode, "list_entry_l0" );
            refPicListModification->setRefPicSetIdxL0(i, uiCode );
          }
        }
        else
        {
          for (i = 0; i < pcSlice->getNumRefIdx(REF_PIC_LIST_0); i ++)
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
    if(pcSlice->isInterB())
    {
      if( !pps->getListsModificationPresentFlag() || pcSlice->getNumRpsCurrTempList() <= 1 )
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
        Int numRpsCurrTempList1 = pcSlice->getNumRpsCurrTempList();
        if ( numRpsCurrTempList1 > 1 )
        {
          Int length = 1;
          numRpsCurrTempList1 --;
          while ( numRpsCurrTempList1 >>= 1)
          {
            length ++;
          }
          for (i = 0; i < pcSlice->getNumRefIdx(REF_PIC_LIST_1); i ++)
          {
            READ_CODE( length, uiCode, "list_entry_l1" );
            refPicListModification->setRefPicSetIdxL1(i, uiCode );
          }
        }
        else
        {
          for (i = 0; i < pcSlice->getNumRefIdx(REF_PIC_LIST_1); i ++)
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
    if (pcSlice->isInterB())
    {
      READ_FLAG( uiCode, "mvd_l1_zero_flag" );       pcSlice->setMvdL1ZeroFlag( (uiCode ? true : false) );
    }

    pcSlice->setCabacInitFlag( false ); // default
    if(pps->getCabacInitPresentFlag() && !pcSlice->isIntra())
    {
      READ_FLAG(uiCode, "cabac_init_flag");
      pcSlice->setCabacInitFlag( uiCode ? true : false );
    }

    if ( pcSlice->getEnableTMVPFlag() )
    {
      if ( pcSlice->getSliceType() == B_SLICE )
      {
        READ_FLAG( uiCode, "collocated_from_l0_flag" );
        pcSlice->setColFromL0Flag(uiCode);
      }
      else
      {
        pcSlice->setColFromL0Flag( 1 );
      }

      if ( pcSlice->getSliceType() != I_SLICE &&
          ((pcSlice->getColFromL0Flag() == 1 && pcSlice->getNumRefIdx(REF_PIC_LIST_0) > 1)||
           (pcSlice->getColFromL0Flag() == 0 && pcSlice->getNumRefIdx(REF_PIC_LIST_1) > 1)))
      {
        READ_UVLC( uiCode, "collocated_ref_idx" );
        pcSlice->setColRefIdx(uiCode);
      }
      else
      {
        pcSlice->setColRefIdx(0);
      }
    }
    if ( (pps->getUseWP() && pcSlice->getSliceType()==P_SLICE) || (pps->getWPBiPred() && pcSlice->getSliceType()==B_SLICE) )
    {
      xParsePredWeightTable(pcSlice, sps);
      pcSlice->initWpScaling(sps);
    }
    if (!pcSlice->isIntra())
    {
      READ_UVLC( uiCode, "five_minus_max_num_merge_cand");
      pcSlice->setMaxNumMergeCand(MRG_MAX_NUM_CANDS - uiCode);
    }

    READ_SVLC( iCode, "slice_qp_delta" );
    pcSlice->setSliceQp (26 + pps->getPicInitQPMinus26() + iCode);

    assert( pcSlice->getSliceQp() >= -sps->getQpBDOffset(CHANNEL_TYPE_LUMA) );
    assert( pcSlice->getSliceQp() <=  51 );

    if (pps->getSliceChromaQpFlag())
    {
      if (numValidComp>COMPONENT_Cb)
      {
        READ_SVLC( iCode, "slice_cb_qp_offset" );
        pcSlice->setSliceChromaQpDelta(COMPONENT_Cb, iCode );
        assert( pcSlice->getSliceChromaQpDelta(COMPONENT_Cb) >= -12 );
        assert( pcSlice->getSliceChromaQpDelta(COMPONENT_Cb) <=  12 );
        assert( (pps->getQpOffset(COMPONENT_Cb) + pcSlice->getSliceChromaQpDelta(COMPONENT_Cb)) >= -12 );
        assert( (pps->getQpOffset(COMPONENT_Cb) + pcSlice->getSliceChromaQpDelta(COMPONENT_Cb)) <=  12 );
      }

      if (numValidComp>COMPONENT_Cr)
      {
        READ_SVLC( iCode, "slice_cr_qp_offset" );
        pcSlice->setSliceChromaQpDelta(COMPONENT_Cr, iCode );
        assert( pcSlice->getSliceChromaQpDelta(COMPONENT_Cr) >= -12 );
        assert( pcSlice->getSliceChromaQpDelta(COMPONENT_Cr) <=  12 );
        assert( (pps->getQpOffset(COMPONENT_Cr) + pcSlice->getSliceChromaQpDelta(COMPONENT_Cr)) >= -12 );
        assert( (pps->getQpOffset(COMPONENT_Cr) + pcSlice->getSliceChromaQpDelta(COMPONENT_Cr)) <=  12 );
      }
    }

    if (pps->getPpsRangeExtension().getChromaQpOffsetListEnabledFlag())
    {
      READ_FLAG(uiCode, "cu_chroma_qp_offset_enabled_flag"); pcSlice->setUseChromaQpAdj(uiCode != 0);
    }
    else
    {
      pcSlice->setUseChromaQpAdj(false);
    }

    if (pps->getDeblockingFilterControlPresentFlag())
    {
      if(pps->getDeblockingFilterOverrideEnabledFlag())
      {
        READ_FLAG ( uiCode, "deblocking_filter_override_flag" );        pcSlice->setDeblockingFilterOverrideFlag(uiCode ? true : false);
      }
      else
      {
        pcSlice->setDeblockingFilterOverrideFlag(0);
      }
      if(pcSlice->getDeblockingFilterOverrideFlag())
      {
        READ_FLAG ( uiCode, "slice_deblocking_filter_disabled_flag" );   pcSlice->setDeblockingFilterDisable(uiCode ? 1 : 0);
        if(!pcSlice->getDeblockingFilterDisable())
        {
          READ_SVLC( iCode, "slice_beta_offset_div2" );                       pcSlice->setDeblockingFilterBetaOffsetDiv2(iCode);
          assert(pcSlice->getDeblockingFilterBetaOffsetDiv2() >= -6 &&
                 pcSlice->getDeblockingFilterBetaOffsetDiv2() <=  6);
          READ_SVLC( iCode, "slice_tc_offset_div2" );                         pcSlice->setDeblockingFilterTcOffsetDiv2(iCode);
          assert(pcSlice->getDeblockingFilterTcOffsetDiv2() >= -6 &&
                 pcSlice->getDeblockingFilterTcOffsetDiv2() <=  6);
        }
      }
      else
      {
        pcSlice->setDeblockingFilterDisable       ( pps->getPPSDeblockingFilterDisabledFlag() );
        pcSlice->setDeblockingFilterBetaOffsetDiv2( pps->getDeblockingFilterBetaOffsetDiv2() );
        pcSlice->setDeblockingFilterTcOffsetDiv2  ( pps->getDeblockingFilterTcOffsetDiv2() );
      }
    }
    else
    {
      pcSlice->setDeblockingFilterDisable       ( false );
      pcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
      pcSlice->setDeblockingFilterTcOffsetDiv2  ( 0 );
    }

    Bool isSAOEnabled = sps->getUseSAO() && (pcSlice->getSaoEnabledFlag(CHANNEL_TYPE_LUMA) || (bChroma && pcSlice->getSaoEnabledFlag(CHANNEL_TYPE_CHROMA)));
    Bool isDBFEnabled = (!pcSlice->getDeblockingFilterDisable());

    if(pps->getLoopFilterAcrossSlicesEnabledFlag() && ( isSAOEnabled || isDBFEnabled ))
    {
      READ_FLAG( uiCode, "slice_loop_filter_across_slices_enabled_flag");
    }
    else
    {
      uiCode = pps->getLoopFilterAcrossSlicesEnabledFlag()?1:0;
    }
    pcSlice->setLFCrossSliceBoundaryFlag( (uiCode==1)?true:false);

  }

  std::vector<UInt> entryPointOffset;
  if( pps->getTilesEnabledFlag() || pps->getEntropyCodingSyncEnabledFlag() )
  {
    UInt numEntryPointOffsets;
    UInt offsetLenMinus1;
    READ_UVLC(numEntryPointOffsets, "num_entry_point_offsets");
    if (numEntryPointOffsets>0)
    {
      READ_UVLC(offsetLenMinus1, "offset_len_minus1");
      entryPointOffset.resize(numEntryPointOffsets);
      for (UInt idx=0; idx<numEntryPointOffsets; idx++)
      {
        READ_CODE(offsetLenMinus1+1, uiCode, "entry_point_offset_minus1");
        entryPointOffset[ idx ] = uiCode + 1;
      }
    }
  }

#if SVC_EXTENSION
  Int sliceHeaderExtensionLength = 0;
  if(pps->getSliceHeaderExtensionPresentFlag())
  {
    READ_UVLC( uiCode, "slice_segment_header_extension_length"); sliceHeaderExtensionLength = uiCode;
  }
  else
  {
    sliceHeaderExtensionLength = 0;
    pcSlice->setPocMsbValPresentFlag( false );
  }

  UInt startBits = m_pcBitstream.getNumBitsRead();     // Start counter of # SH Extn bits
  if( sliceHeaderExtensionLength > 0 )
  {
    if( pcSlice->getPPS()->getPocResetInfoPresentFlag() )
    {
      READ_CODE( 2, uiCode,       "poc_reset_idc"); pcSlice->setPocResetIdc(uiCode);

      /* The value of poc_reset_idc shall not be equal to 1 or 2 for a RASL picture, a RADL picture, 
      a sub-layer non-reference picture, or a picture that has TemporalId greater than 0, 
      or a picture that has discardable_flag equal to 1. */
      if( pcSlice->getPocResetIdc() == 1 || pcSlice->getPocResetIdc() == 2 )
      {
        assert( !pcSlice->isRASL() );
        assert( !pcSlice->isRADL() );
        assert( !pcSlice->isSLNR() );
        assert( pcSlice->getTLayer() == 0 );
        assert( pcSlice->getDiscardableFlag() == 0 );
      }

      // The value of poc_reset_idc of a CRA or BLA picture shall be less than 3.
      if( pcSlice->getPocResetIdc() == 3)
      {
        assert( ! ( pcSlice->isCRA() || pcSlice->isBLA() ) );
      }
    }
    else
    {
      pcSlice->setPocResetIdc( 0 );
    }

    if( pcSlice->getVPS()->getPocLsbNotPresentFlag( pcSlice->getVPS()->getLayerIdxInVps(pcSlice->getLayerId()) ) && iPOClsb > 0 )
    {
      assert( pcSlice->getPocResetIdc() != 2 );
    }

    if( pcSlice->getPocResetIdc() > 0 )
    {
      READ_CODE(6, uiCode,      "poc_reset_period_id"); pcSlice->setPocResetPeriodId(uiCode);
    }
    else
    {
      pcSlice->setPocResetPeriodId( 0 );
    }

    if( pcSlice->getPocResetIdc() == 3 )
    {
      READ_FLAG( uiCode,        "full_poc_reset_flag"); pcSlice->setFullPocResetFlag((uiCode == 1) ? true : false);
      READ_CODE(pcSlice->getSPS()->getBitsForPOC(), uiCode,"poc_lsb_val"); pcSlice->setPocLsbVal(uiCode);

      if( pcSlice->getVPS()->getPocLsbNotPresentFlag( pcSlice->getVPS()->getLayerIdxInVps(pcSlice->getLayerId()) ) && pcSlice->getFullPocResetFlag() )
      {
        assert( pcSlice->getPocLsbVal() == 0 );
      }
    }

    // Derive the value of PocMsbValRequiredFlag
    pcSlice->setPocMsbValRequiredFlag( (pcSlice->getCraPicFlag() || pcSlice->getBlaPicFlag())
      && (!pcSlice->getVPS()->getVpsPocLsbAlignedFlag() ||
      (pcSlice->getVPS()->getVpsPocLsbAlignedFlag() && pcSlice->getVPS()->getNumDirectRefLayers(pcSlice->getLayerId()) == 0))
      );

    if( !pcSlice->getPocMsbValRequiredFlag() && pcSlice->getVPS()->getVpsPocLsbAlignedFlag() )
    {
      READ_FLAG(uiCode, "poc_msb_cycle_val_present_flag"); pcSlice->setPocMsbValPresentFlag(uiCode ? true : false);
    }
    else
    {
      if( pcSlice->getPocMsbValRequiredFlag() )
      {
        pcSlice->setPocMsbValPresentFlag( true );
      }
      else
      {
        pcSlice->setPocMsbValPresentFlag( false );
      }
    }

    if( pcSlice->getPocMsbValPresentFlag() )
    {
      READ_UVLC( uiCode,    "poc_msb_cycle_val");             pcSlice->setPocMsbVal( uiCode );
    }

    // Read remaining bits in the slice header extension.
    Int bitsNum = sliceHeaderExtensionLength * 8 + startBits - m_pcBitstream.getNumBitsRead();

    while( bitsNum )
    {
      READ_FLAG( uiCode, "slice_segment_header_extension_data_bit" );
      bitsNum--;
    }
  }
#else
  if(pps->getSliceHeaderExtensionPresentFlag())
  {
    READ_UVLC(uiCode,"slice_segment_header_extension_length");
    for(Int i=0; i<uiCode; i++)
    {
      UInt ignore;
      READ_CODE(8,ignore,"slice_segment_header_extension_data_byte");
    }
  }
#endif

/*
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  TComCodingStatistics::IncrementStatisticEP(STATS__BYTE_ALIGNMENT_BITS,m_pcBitstream->readByteAlignment(),0);
#else
  m_pcBitstream.readByteAlignment();
#endif

  pcSlice->clearSubstreamSizes();

  if( pps->getTilesEnabledFlag() || pps->getEntropyCodingSyncEnabledFlag() )
  {
    Int endOfSliceHeaderLocation = m_pcBitstream->getByteLocation();

    // Adjust endOfSliceHeaderLocation to account for emulation prevention bytes in the slice segment header
    for ( UInt curByteIdx  = 0; curByteIdx<m_pcBitstream.numEmulationPreventionBytesRead(); curByteIdx++ )
    {
      if ( m_pcBitstream.getEmulationPreventionByteLocation( curByteIdx ) < endOfSliceHeaderLocation )
      {
        endOfSliceHeaderLocation++;
      }
    }

    Int  curEntryPointOffset     = 0;
    Int  prevEntryPointOffset    = 0;
    for (UInt idx=0; idx<entryPointOffset.size(); idx++)
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
      pcSlice->addSubstreamSize(entryPointOffset [ idx ] );
    }
  }
*/
  return;
}

Void TDecCavlc::parsePTL( TComPTL *rpcPTL, Bool profilePresentFlag, Int maxNumSubLayersMinus1 )
{
  UInt uiCode;
  if(profilePresentFlag)
  {
    parseProfileTier(rpcPTL->getGeneralPTL(), false);
  }
  READ_CODE( 8, uiCode, "general_level_idc" );    rpcPTL->getGeneralPTL()->setLevelIdc(Level::Name(uiCode));

  for (Int i = 0; i < maxNumSubLayersMinus1; i++)
  {
    READ_FLAG( uiCode, "sub_layer_profile_present_flag[i]" ); rpcPTL->setSubLayerProfilePresentFlag(i, uiCode);
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
    if( rpcPTL->getSubLayerProfilePresentFlag(i) )
    {
      parseProfileTier(rpcPTL->getSubLayerPTL(i), true);
    }
    if(rpcPTL->getSubLayerLevelPresentFlag(i))
    {
      READ_CODE( 8, uiCode, "sub_layer_level_idc[i]" );   rpcPTL->getSubLayerPTL(i)->setLevelIdc(Level::Name(uiCode));
    }
  }
}

#if ENC_DEC_TRACE || RExt__DECODER_DEBUG_BIT_STATISTICS
Void TDecCavlc::parseProfileTier(ProfileTierLevel *ptl, const Bool bIsSubLayer)
#define PTL_TRACE_TEXT(txt) bIsSubLayer?("sub_layer_" txt) : ("general_" txt)
#else
Void TDecCavlc::parseProfileTier(ProfileTierLevel *ptl, const Bool /*bIsSubLayer*/)
#define PTL_TRACE_TEXT(txt) txt
#endif
{
  UInt uiCode;
  READ_CODE(2 , uiCode,   PTL_TRACE_TEXT("profile_space"                   )); ptl->setProfileSpace(uiCode);
  READ_FLAG(    uiCode,   PTL_TRACE_TEXT("tier_flag"                       )); ptl->setTierFlag    (uiCode ? Level::HIGH : Level::MAIN);
  READ_CODE(5 , uiCode,   PTL_TRACE_TEXT("profile_idc"                     )); ptl->setProfileIdc  (Profile::Name(uiCode));
  for(Int j = 0; j < 32; j++)
  {
    READ_FLAG(  uiCode,   PTL_TRACE_TEXT("profile_compatibility_flag[][j]" )); ptl->setProfileCompatibilityFlag(j, uiCode ? 1 : 0);
  }
  READ_FLAG(uiCode,       PTL_TRACE_TEXT("progressive_source_flag"         )); ptl->setProgressiveSourceFlag(uiCode ? true : false);

  READ_FLAG(uiCode,       PTL_TRACE_TEXT("interlaced_source_flag"          )); ptl->setInterlacedSourceFlag(uiCode ? true : false);

  READ_FLAG(uiCode,       PTL_TRACE_TEXT("non_packed_constraint_flag"      )); ptl->setNonPackedConstraintFlag(uiCode ? true : false);

  READ_FLAG(uiCode,       PTL_TRACE_TEXT("frame_only_constraint_flag"      )); ptl->setFrameOnlyConstraintFlag(uiCode ? true : false);

  if (ptl->getProfileIdc() == Profile::MAINREXT           || ptl->getProfileCompatibilityFlag(Profile::MAINREXT) ||
      ptl->getProfileIdc() == Profile::HIGHTHROUGHPUTREXT || ptl->getProfileCompatibilityFlag(Profile::HIGHTHROUGHPUTREXT)
#if SCALABLE_REXT
      || ptl->getProfileIdc() == Profile::SCALABLEREXT
#endif
      )
  {
    UInt maxBitDepth=16;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_12bit_constraint_flag"       )); if (uiCode) maxBitDepth=12;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_10bit_constraint_flag"       )); if (uiCode) maxBitDepth=10;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_8bit_constraint_flag"        )); if (uiCode) maxBitDepth=8;
    ptl->setBitDepthConstraint(maxBitDepth);
    ChromaFormat chromaFmtConstraint=CHROMA_444;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_422chroma_constraint_flag"   )); if (uiCode) chromaFmtConstraint=CHROMA_422;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_420chroma_constraint_flag"   )); if (uiCode) chromaFmtConstraint=CHROMA_420;
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("max_monochrome_constraint_flag"  )); if (uiCode) chromaFmtConstraint=CHROMA_400;
    ptl->setChromaFormatConstraint(chromaFmtConstraint);
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("intra_constraint_flag"           )); ptl->setIntraConstraintFlag(uiCode != 0);
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("one_picture_only_constraint_flag")); ptl->setOnePictureOnlyConstraintFlag(uiCode != 0);
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("lower_bit_rate_constraint_flag"  )); ptl->setLowerBitRateConstraintFlag(uiCode != 0);
#if SVC_EXTENSION
    READ_CODE(32, uiCode, "reserved_zero_34bits");  READ_CODE(2, uiCode, "reserved_zero_34bits");
  }
  else if( ptl->getProfileIdc() == Profile::SCALABLEMAIN || ptl->getProfileCompatibilityFlag(Profile::SCALABLEMAIN) )
  {
    READ_FLAG(    uiCode, "max_12bit_constraint_flag" ); assert (uiCode == 1);
    READ_FLAG(    uiCode, "max_10bit_constraint_flag" ); assert (uiCode == 1);
    READ_FLAG(    uiCode, "max_8bit_constraint_flag"  ); ptl->setProfileIdc  ((uiCode) ? Profile::SCALABLEMAIN : Profile::SCALABLEMAIN10);
    READ_FLAG(    uiCode, "max_422chroma_constraint_flag"  ); assert (uiCode == 1);
    READ_FLAG(    uiCode, "max_420chroma_constraint_flag"  ); assert (uiCode == 1);
    READ_FLAG(    uiCode, "max_monochrome_constraint_flag" ); assert (uiCode == 0);
    READ_FLAG(    uiCode, "intra_constraint_flag"); assert (uiCode == 0);
    READ_FLAG(    uiCode, "one_picture_only_constraint_flag"); assert (uiCode == 0);
    READ_FLAG(    uiCode, "lower_bit_rate_constraint_flag"); assert (uiCode == 1);
    READ_CODE(32, uiCode, "reserved_zero_34bits");  READ_CODE(2, uiCode, "reserved_zero_34bits");
  }
#if VIEW_SCALABILITY
  else if( ptl->getProfileIdc() == Profile::MULTIVIEWMAIN )
  {
    READ_FLAG(    uiCode, "general_max_12bit_constraint_flag" ); assert (uiCode == 1);
    READ_FLAG(    uiCode, "general_max_10bit_constraint_flag" ); assert (uiCode == 1);
    READ_FLAG(    uiCode, "general_max_8bit_constraint_flag"  ); assert (uiCode == 1);
    READ_FLAG(    uiCode, "general_max_422chroma_constraint_flag"  ); assert (uiCode == 1);
    READ_FLAG(    uiCode, "general_max_420chroma_constraint_flag"  ); assert (uiCode == 1);
    READ_FLAG(    uiCode, "general_max_monochrome_constraint_flag" ); assert (uiCode == 0);
    READ_FLAG(    uiCode, "general_intra_constraint_flag"); assert (uiCode == 0);
    READ_FLAG(    uiCode, "general_one_picture_only_constraint_flag"); assert (uiCode == 0);
    READ_FLAG(    uiCode, "general_lower_bit_rate_constraint_flag"); assert (uiCode == 1);
    READ_CODE(32, uiCode, "general_reserved_zero_34bits");  READ_CODE(2, uiCode, "general_reserved_zero_34bits");
  }
#endif
  else
  {
    ptl->setBitDepthConstraint((ptl->getProfileIdc() == Profile::MAIN10)?10:8);
    ptl->setChromaFormatConstraint(CHROMA_420);
    ptl->setIntraConstraintFlag(false);
    ptl->setLowerBitRateConstraintFlag(true);
    READ_CODE(32,  uiCode, "reserved_zero_43bits");  READ_CODE(11,  uiCode, "reserved_zero_43bits");
  }
#else
    READ_CODE(16, uiCode, PTL_TRACE_TEXT("reserved_zero_34bits[0..15]"     ));
    READ_CODE(16, uiCode, PTL_TRACE_TEXT("reserved_zero_34bits[16..31]"    ));
    READ_CODE(2,  uiCode, PTL_TRACE_TEXT("reserved_zero_34bits[32..33]"    ));
  }
  else
  {
    ptl->setBitDepthConstraint((ptl->getProfileIdc() == Profile::MAIN10)?10:8);
    ptl->setChromaFormatConstraint(CHROMA_420);
    ptl->setIntraConstraintFlag(false);
    ptl->setLowerBitRateConstraintFlag(true);
    READ_CODE(16, uiCode, PTL_TRACE_TEXT("reserved_zero_43bits[0..15]"     ));
    READ_CODE(16, uiCode, PTL_TRACE_TEXT("reserved_zero_43bits[16..31]"    ));
    READ_CODE(11, uiCode, PTL_TRACE_TEXT("reserved_zero_43bits[32..42]"    ));
  }
#endif

  if ((ptl->getProfileIdc() >= Profile::MAIN && ptl->getProfileIdc() <= Profile::HIGHTHROUGHPUTREXT) ||
       ptl->getProfileCompatibilityFlag(Profile::MAIN) ||
       ptl->getProfileCompatibilityFlag(Profile::MAIN10) ||
       ptl->getProfileCompatibilityFlag(Profile::MAINSTILLPICTURE) ||
       ptl->getProfileCompatibilityFlag(Profile::MAINREXT) ||
       ptl->getProfileCompatibilityFlag(Profile::HIGHTHROUGHPUTREXT) )
  {
#if SVC_EXTENSION
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("inbld_flag"                      ));
#else
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("inbld_flag"                      )); assert(uiCode == 0);
#endif
  }
  else
  {
#if SVC_EXTENSION
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("inbld_flag"                      ));
#else
    READ_FLAG(    uiCode, PTL_TRACE_TEXT("reserved_zero_bit"               ));
#endif
  }
#undef PTL_TRACE_TEXT
}

// Void TDecCavlc::parseTerminatingBit( UInt& ruiBit )
// {
//   ruiBit = false;
//   Int iBitsLeft = m_pcBitstream->getNumBitsLeft();
//   if(iBitsLeft <= 8)
//   {
//     UInt uiPeekValue = m_pcBitstream->peekBits(iBitsLeft);
//     if (uiPeekValue == (1<<(iBitsLeft-1)))
//     {
//       ruiBit = true;
//     }
//   }
// }

// Void TDecCavlc::parseRemainingBytes( Bool noTrailingBytesExpected )
// {
//   if (noTrailingBytesExpected)
//   {
//     const UInt numberOfRemainingSubstreamBytes=m_pcBitstream->getNumBitsLeft();
//     assert (numberOfRemainingSubstreamBytes == 0);
//   }
//   else
//   {
//     while (m_pcBitstream->getNumBitsLeft())
//     {
//       UInt trailingNullByte=m_pcBitstream->readByte();
//       if (trailingNullByte!=0)
//       {
//         printf("Trailing byte should be 0, but has value %02x\n", trailingNullByte);
//         assert(trailingNullByte==0);
//       }
//     }
//   }
// }

// Void TDecCavlc::parseSkipFlag( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parseCUTransquantBypassFlag( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parseMVPIdx( Int& /*riMVPIdx*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parseSplitFlag     ( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parsePartSize( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parsePredMode( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
// {
//   assert(0);
// }

// /** Parse I_PCM information.
// * \param pcCU pointer to CU
// * \param uiAbsPartIdx CU index
// * \param uiDepth CU depth
// * \returns Void
// *
// * If I_PCM flag indicates that the CU is I_PCM, parse its PCM alignment bits and codes.
// */
// Void TDecCavlc::parseIPCMInfo( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parseIntraDirLumaAng  ( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parseIntraDirChroma( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parseInterDir( TComDataCU* /*pcCU*/, UInt& /*ruiInterDir*/, UInt /*uiAbsPartIdx*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parseRefFrmIdx( TComDataCU* /*pcCU*/, Int& /*riRefFrmIdx*/, RefPicList /*eRefList*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parseMvd( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiPartIdx*/, UInt /*uiDepth*/, RefPicList /*eRefList*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parseCrossComponentPrediction( class TComTU& /*rTu*/, ComponentID /*compID*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parseDeltaQP( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth )
// {
//   Int  iDQp;

// #if RExt__DECODER_DEBUG_BIT_STATISTICS
//   READ_SVLC(iDQp, "delta_qp");
// #else
//   xReadSvlc( iDQp );
// #endif

//   Int qpBdOffsetY = pcCU->getSlice()->getSPS()->getQpBDOffset(CHANNEL_TYPE_LUMA);
//   const Int qp = (((Int) pcCU->getRefQP( uiAbsPartIdx ) + iDQp + 52 + 2*qpBdOffsetY )%(52+ qpBdOffsetY)) -  qpBdOffsetY;

//   const UInt maxCUDepth        = pcCU->getSlice()->getSPS()->getMaxTotalCUDepth();
//   const UInt maxCuDQPDepth     = pcCU->getSlice()->getPPS()->getMaxCuDQPDepth();
//   const UInt doubleDepthDifference = ((maxCUDepth - maxCuDQPDepth)<<1);
//   const UInt uiAbsQpCUPartIdx = (uiAbsPartIdx>>doubleDepthDifference)<<doubleDepthDifference ;
//   const UInt uiQpCUDepth =   min(uiDepth,pcCU->getSlice()->getPPS()->getMaxCuDQPDepth()) ;

//   pcCU->setQPSubParts( qp, uiAbsQpCUPartIdx, uiQpCUDepth );
// }

// Void TDecCavlc::parseChromaQpAdjustment( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parseCoeffNxN( TComTU &/*rTu*/, ComponentID /*compID*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parseTransformSubdivFlag( UInt& /*ruiSubdivFlag*/, UInt /*uiLog2TransformBlockSize*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parseQtCbf( TComTU &/*rTu*/, const ComponentID /*compID*/, const Bool /*lowestLevel*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parseQtRootCbf( UInt /*uiAbsPartIdx*/, UInt& /*uiQtRootCbf*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parseTransformSkipFlags (TComTU &/*rTu*/, ComponentID /*component*/)
// {
//   assert(0);
// }

// Void TDecCavlc::parseMergeFlag ( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/, UInt /*uiPUIdx*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parseMergeIndex ( TComDataCU* /*pcCU*/, UInt& /*ruiMergeIndex*/ )
// {
//   assert(0);
// }

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

//! parse explicit wp tables
Void TDecCavlc::xParsePredWeightTable( TComSlice* pcSlice, const TComSPS *sps )
{
        WPScalingParam *wp;
  const ChromaFormat    chFmt        = sps->getChromaFormatIdc();
  const Int             numValidComp = Int(getNumberValidComponents(chFmt));
  const Bool            bChroma      = (chFmt!=CHROMA_400);
  const SliceType       eSliceType   = pcSlice->getSliceType();
  const Int             iNbRef       = (eSliceType == B_SLICE ) ? (2) : (1);
        UInt            uiLog2WeightDenomLuma=0, uiLog2WeightDenomChroma=0;
        UInt            uiTotalSignalledWeightFlags = 0;

  Int iDeltaDenom;
  // decode delta_luma_log2_weight_denom :
  READ_UVLC( uiLog2WeightDenomLuma, "luma_log2_weight_denom" );
  assert( uiLog2WeightDenomLuma <= 7 );
  if( bChroma )
  {
    READ_SVLC( iDeltaDenom, "delta_chroma_log2_weight_denom" );
    assert((iDeltaDenom + (Int)uiLog2WeightDenomLuma)>=0);
    assert((iDeltaDenom + (Int)uiLog2WeightDenomLuma)<=7);
    uiLog2WeightDenomChroma = (UInt)(iDeltaDenom + uiLog2WeightDenomLuma);
  }

  for ( Int iNumRef=0 ; iNumRef<iNbRef ; iNumRef++ ) // loop over l0 and l1 syntax elements
  {
    RefPicList  eRefPicList = ( iNumRef ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for ( Int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
    {
      pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);

      wp[COMPONENT_Y].uiLog2WeightDenom = uiLog2WeightDenomLuma;
      for(Int j=1; j<numValidComp; j++)
      {
        wp[j].uiLog2WeightDenom = uiLog2WeightDenomChroma;
      }

      UInt  uiCode;
      READ_FLAG( uiCode, iNumRef==0?"luma_weight_l0_flag[i]":"luma_weight_l1_flag[i]" );
      wp[COMPONENT_Y].bPresentFlag = ( uiCode == 1 );
      uiTotalSignalledWeightFlags += wp[COMPONENT_Y].bPresentFlag;
    }
    if ( bChroma )
    {
      UInt  uiCode;
      for ( Int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
      {
        pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);
        READ_FLAG( uiCode, iNumRef==0?"chroma_weight_l0_flag[i]":"chroma_weight_l1_flag[i]" );
        for(Int j=1; j<numValidComp; j++)
        {
          wp[j].bPresentFlag = ( uiCode == 1 );
        }
        uiTotalSignalledWeightFlags += 2*wp[COMPONENT_Cb].bPresentFlag;
      }
    }
    for ( Int iRefIdx=0 ; iRefIdx<pcSlice->getNumRefIdx(eRefPicList) ; iRefIdx++ )
    {
      pcSlice->getWpScaling(eRefPicList, iRefIdx, wp);
      if ( wp[COMPONENT_Y].bPresentFlag )
      {
        Int iDeltaWeight;
        READ_SVLC( iDeltaWeight, iNumRef==0?"delta_luma_weight_l0[i]":"delta_luma_weight_l1[i]" );
        assert( iDeltaWeight >= -128 );
        assert( iDeltaWeight <=  127 );
        wp[COMPONENT_Y].iWeight = (iDeltaWeight + (1<<wp[COMPONENT_Y].uiLog2WeightDenom));
        READ_SVLC( wp[COMPONENT_Y].iOffset, iNumRef==0?"luma_offset_l0[i]":"luma_offset_l1[i]" );
        Int range=sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag() ? (1<<sps->getBitDepth(CHANNEL_TYPE_LUMA))/2 : 128;
        assert( wp[0].iOffset >= -range );
        assert( wp[0].iOffset <   range );
      }
      else
      {
        wp[COMPONENT_Y].iWeight = (1 << wp[COMPONENT_Y].uiLog2WeightDenom);
        wp[COMPONENT_Y].iOffset = 0;
      }
      if ( bChroma )
      {
        if ( wp[COMPONENT_Cb].bPresentFlag )
        {
          Int range=sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag() ? (1<<sps->getBitDepth(CHANNEL_TYPE_CHROMA))/2 : 128;
          for ( Int j=1 ; j<numValidComp ; j++ )
          {
            Int iDeltaWeight;
            READ_SVLC( iDeltaWeight, iNumRef==0?"delta_chroma_weight_l0[i]":"delta_chroma_weight_l1[i]" );
            assert( iDeltaWeight >= -128 );
            assert( iDeltaWeight <=  127 );
            wp[j].iWeight = (iDeltaWeight + (1<<wp[j].uiLog2WeightDenom));

            Int iDeltaChroma;
            READ_SVLC( iDeltaChroma, iNumRef==0?"delta_chroma_offset_l0[i]":"delta_chroma_offset_l1[i]" );
            assert( iDeltaChroma >= -4*range);
            assert( iDeltaChroma <   4*range);
            Int pred = ( range - ( ( range*wp[j].iWeight)>>(wp[j].uiLog2WeightDenom) ) );
            wp[j].iOffset = Clip3(-range, range-1, (iDeltaChroma + pred) );
          }
        }
        else
        {
          for ( Int j=1 ; j<numValidComp ; j++ )
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
    for(listId = 0; listId <  SCALING_LIST_NUM; listId++)
    {
      if ((sizeId==SCALING_LIST_32x32) && (listId%(SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES) != 0))
      {
        Int *src = scalingList->getScalingListAddress(sizeId, listId);
        const Int size = min(MAX_MATRIX_COEF_NUM,(Int)gPccShvc_scalingListSize[sizeId]);
        const Int *srcNextSmallerSize = scalingList->getScalingListAddress(sizeId-1, listId);
        for(Int i=0; i<size; i++)
        {
          src[i] = srcNextSmallerSize[i];
        }
        scalingList->setScalingListDC(sizeId,listId,(sizeId > SCALING_LIST_8x8) ? scalingList->getScalingListDC(sizeId-1, listId) : src[0]);
      }
      else
      {
        READ_FLAG( code, "scaling_list_pred_mode_flag");
        scalingListPredModeFlag = (code) ? true : false;
        scalingList->setScalingListPredModeFlag(sizeId, listId, scalingListPredModeFlag);
        if(!scalingListPredModeFlag) //Copy Mode
        {
          READ_UVLC( code, "scaling_list_pred_matrix_id_delta");

          if (sizeId==SCALING_LIST_32x32)
          {
            code*=(SCALING_LIST_NUM/NUMBER_OF_PREDICTION_MODES); // Adjust the decoded code for this size, to cope with the missing 32x32 chroma entries.
          }

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
  Int i,coefNum = min(MAX_MATRIX_COEF_NUM,(Int)gPccShvc_scalingListSize[sizeId]);
  Int data;
  Int scalingListDcCoefMinus8 = 0;
  Int nextCoef = SCALING_LIST_START_VALUE;
  UInt* scan  = gPccShvc_scanOrder[SCAN_UNGROUPED][SCAN_DIAG][sizeId==0 ? 2 : 3][sizeId==0 ? 2 : 3];
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
  return true;
  /*
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
}

Void TDecCavlc::parseExplicitRdpcmMode( TComTU& /*rTu*/, ComponentID /*compID*/ )
{
  assert(0);
}

#if SVC_EXTENSION
Void TDecCavlc::parseVPSExtension(TComVPS *vps)
{
  UInt uiCode;
  Int NumOutputLayersInOutputLayerSet[MAX_VPS_LAYER_SETS_PLUS1];
  Int OlsHighestOutputLayerId[MAX_VPS_LAYER_SETS_PLUS1];

  if( vps->getMaxLayers() > 1 && vps->getBaseLayerInternalFlag() )
  {
    vps->setProfilePresentFlag(1, false);
    parsePTL( vps->getPTL(1), vps->getProfilePresentFlag(1), vps->getMaxTLayers() - 1 );
  }

  UInt numScalabilityTypes = 0, i = 0, j = 0;

  READ_FLAG( uiCode, "splitting_flag" ); vps->setSplittingFlag(uiCode ? true : false);

  for(i = 0; i < MAX_VPS_NUM_SCALABILITY_TYPES; i++)
  {
    READ_FLAG( uiCode, "scalability_mask_flag[i]" ); vps->setScalabilityMask(i, uiCode ? true : false);
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
  vps->setLayerIdxInVps(0, 0);
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
    vps->setLayerIdxInVps(vps->getLayerIdInNuh(i), i);

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

  READ_CODE( 4, uiCode, "view_id_len" ); vps->setViewIdLen( uiCode );

  if ( vps->getViewIdLen() > 0 )
  {
    for( i = 0; i < vps->getNumViews(); i++ )
    {
      READ_CODE( vps->getViewIdLen( ), uiCode, "view_id_val[i]" ); vps->setViewIdVal( i, uiCode );
    }
  }

  // For layer 0
  vps->setNumDirectRefLayers(0, 0);
  // For other layers
  for( Int layerCtr = 1; layerCtr < vps->getMaxLayers(); layerCtr++)
  {
    UInt layerId = vps->getLayerIdInNuh(layerCtr); 
    UInt numDirectRefLayers = 0;
    for( Int refLayerCtr = 0; refLayerCtr < layerCtr; refLayerCtr++)
    {
      READ_FLAG(uiCode, "direct_dependency_flag[i][j]" ); vps->setDirectDependencyFlag(layerCtr, refLayerCtr, uiCode? true : false);
      if(uiCode)
      {
        vps->setRefLayerId(layerId, numDirectRefLayers, vps->getLayerIdInNuh(refLayerCtr));
        numDirectRefLayers++;
      }
    }
    vps->setNumDirectRefLayers(layerId, numDirectRefLayers);
  }

  // dependency constraint
  vps->setNumRefLayers();

  if (vps->getMaxLayers() > MAX_REF_LAYERS)
  {
    for (i = 1; i < vps->getMaxLayers(); i++)
    {
      assert(vps->getNumRefLayers(vps->getLayerIdInNuh(i)) <= MAX_REF_LAYERS);
    }
  }

  vps->setPredictedLayerIds();
  vps->setTreePartitionLayerIdList();

  if( vps->getNumIndependentLayers() > 1 )
  {
    READ_UVLC(uiCode, "num_add_layer_sets"); vps->setNumAddLayerSets(uiCode);

    for( i = 0; i < vps->getNumAddLayerSets(); i++ )
    {
      for( j = 1; j < vps->getNumIndependentLayers(); j++ )
      {
        Int len = 1;
        while( (1 << len) < (vps->getNumLayersInTreePartition(j) + 1) )
        {
          len++;
        }

        READ_CODE(len, uiCode, "highest_layer_idx_plus1[i][j]"); vps->setHighestLayerIdxPlus1(i, j, uiCode);
      }
    }
    vps->setNumLayerSets(vps->getNumLayerSets() + vps->getNumAddLayerSets());
    vps->deriveLayerIdListVariablesForAddLayerSets();
  }
  else
  {
    vps->setNumAddLayerSets(0);
  }

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

  READ_FLAG( uiCode, "max_tid_ref_present_flag"); vps->setMaxTidRefPresentFlag(uiCode ? true : false);
  if( vps->getMaxTidRefPresentFlag() )
  {
    for( i = 0; i < vps->getMaxLayers() - 1; i++ )
    {
      for( j = i+1; j < vps->getMaxLayers(); j++ )
      {
        if( vps->getDirectDependencyFlag(j, i) )
        {
          READ_CODE( 3, uiCode, "max_tid_il_ref_pics_plus1[i][j]" ); vps->setMaxTidIlRefPicsPlus1(i, j, uiCode);          
        }
      }
    }
  }
  else
  {
    for(i = 0; i < vps->getMaxLayers() - 1; i++)
    {
      for( j = i+1; j < vps->getMaxLayers(); j++)
      {
        vps->setMaxTidIlRefPicsPlus1(i, j, 7);
      }
    }
  }
  READ_FLAG( uiCode, "default_ref_layers_active_flag" ); vps->setDefaultRefLayersActiveFlag(uiCode ? true : false);

  // Profile-tier-level signalling
  READ_UVLC(  uiCode, "vps_num_profile_tier_level_minus1"); vps->setNumProfileTierLevel( uiCode + 1 );

  Int const numBitsForPtlIdx = vps->calculateLenOfSyntaxElement( vps->getNumProfileTierLevel() );

  for( Int idx = vps->getBaseLayerInternalFlag() ? 2 : 1; idx < vps->getNumProfileTierLevel(); idx++ )
  {
    READ_FLAG( uiCode, "vps_profile_present_flag[i]" ); vps->setProfilePresentFlag(idx, uiCode ? true : false);

    if( !vps->getProfilePresentFlag(idx) )
    {
      // Copy profile information from previous one
      vps->getPTL(idx)->copyProfileInfo( vps->getPTL( idx - 1 ) );
    }

    parsePTL( vps->getPTL(idx), vps->getProfilePresentFlag(idx), vps->getMaxTLayers() - 1 );
  }

  if( vps->getNumLayerSets() > 1 )
  {
    READ_UVLC( uiCode, "num_add_olss" );                  vps->setNumAddOutputLayerSets( uiCode );
    READ_CODE( 2, uiCode, "default_output_layer_idc" );   vps->setDefaultTargetOutputLayerIdc( uiCode );
  }
  else
  {
    vps->setNumAddOutputLayerSets( 0 );
  }

  // The value of num_add_olss shall be in the range of 0 to 1023, inclusive.
  assert( vps->getNumAddOutputLayerSets() >= 0 && vps->getNumAddOutputLayerSets() < 1024 );

  Int numOutputLayerSets = vps->getNumLayerSets() + vps->getNumAddOutputLayerSets();

  vps->setNumOutputLayerSets( numOutputLayerSets );

  // Default output layer set
  vps->setOutputLayerSetIdx(0, 0);
  vps->setOutputLayerFlag(0, 0, true);
  vps->deriveNecessaryLayerFlag(0);
  vps->getProfileLevelTierIdx()->resize(numOutputLayerSets);
  vps->getProfileLevelTierIdx(0)->push_back( vps->getBaseLayerInternalFlag() && vps->getMaxLayers() > 1 ? 1 : 0);

  std::vector<Profile::Name> profiles;

  if( vps->getScalabilityMask( VIEW_ORDER_INDEX ) )
  {
    profiles.push_back( Profile::MULTIVIEWMAIN );
  }

  if( vps->getScalabilityMask( SCALABILITY_ID ) )
  {
    profiles.push_back( Profile::SCALABLEMAIN );
  }

  for(i = 1; i < numOutputLayerSets; i++)
  {
    if( vps->getNumLayerSets() > 2 && i >= vps->getNumLayerSets() )
    {
      Int numBits = 1;
      while ((1 << numBits) < (vps->getNumLayerSets() - 1))
      {
        numBits++;
      }
      READ_CODE( numBits, uiCode, "layer_set_idx_for_ols_minus1");
    }
    else
    {
      uiCode = 0;
    }

    vps->setOutputLayerSetIdx( i, i < vps->getNumLayerSets() ? i : (uiCode + 1) );

    Int layerSetIdxForOutputLayerSet = vps->getOutputLayerSetIdx(i);

    if( i > vps->getVpsNumLayerSetsMinus1() || vps->getDefaultTargetOutputLayerIdc() == 2 )
    {
      for( j = 0; j < vps->getNumLayersInIdList(layerSetIdxForOutputLayerSet); j++ )
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
        for( j = 0; j < vps->getNumLayersInIdList(layerSetIdxForOutputLayerSet); j++ )
        {
          vps->setOutputLayerFlag(i, j, (j == (vps->getNumLayersInIdList(layerSetIdxForOutputLayerSet)-1))  );
        }
      }
      else if( vps->getDefaultTargetOutputLayerIdc() == 0 )
      {
        for( j = 0; j < vps->getNumLayersInIdList(layerSetIdxForOutputLayerSet); j++ )
        {
          vps->setOutputLayerFlag(i, j, 1);
        }
      }
    }

    vps->deriveNecessaryLayerFlag(i);  

    vps->getProfileLevelTierIdx(i)->assign(vps->getNumLayersInIdList(layerSetIdxForOutputLayerSet), -1);

    std::vector<UInt> passedCheck(profiles.size(), 0);
    UInt numIncludedLayers = 0;

    for( j = 0; j < vps->getNumLayersInIdList(layerSetIdxForOutputLayerSet); j++ )
    {
      if( vps->getNecessaryLayerFlag(i, j) && (vps->getNumProfileTierLevel()-1) > 0 )
      {
        READ_CODE( numBitsForPtlIdx, uiCode, "profile_tier_level_idx[i]" ); 
        vps->setProfileLevelTierIdx(i, j, uiCode );

        //For conformance checking
        //Conformance of a layer in an output operation point associated with an OLS in a bitstream to the Scalable Main profile is indicated as follows:
        //If OpTid of the output operation point is equal to vps_max_sub_layer_minus1, the conformance is indicated by general_profile_idc being equal to 7 or general_profile_compatibility_flag[ 7 ] being equal to 1
        //Conformance of a layer in an output operation point associated with an OLS in a bitstream to the Scalable Main 10 profile is indicated as follows:
        //If OpTid of the output operation point is equal to vps_max_sub_layer_minus1, the conformance is indicated by general_profile_idc being equal to 7 or general_profile_compatibility_flag[ 7 ] being equal to 1
        //The following assert may be updated / upgraded to take care of general_profile_compatibility_flag.

        if( layerSetIdxForOutputLayerSet <= vps->getVpsNumLayerSetsMinus1() )
        {
          if( vps->getLayerSetLayerIdList(layerSetIdxForOutputLayerSet, j) != 0 && vps->getNecessaryLayerFlag(i, j) )
          {
            ProfileTierLevel* ptl = vps->getPTL(vps->getProfileLevelTierIdx(i, j))->getGeneralPTL();
            numIncludedLayers++;
#if SCALABLE_REXT
            // if scalable profile is Scalable Main 10 or Scalable Rext, dimension_id = 2
            const Profile::Name profileName = ( ptl->getProfileIdc() == Profile::SCALABLEMAIN10 || ptl->getProfileIdc() == Profile::SCALABLEREXT ) ? Profile::SCALABLEMAIN : ptl->getProfileIdc();
#else
            const Profile::Name profileName = ptl->getProfileIdc() == Profile::SCALABLEMAIN10 ? Profile::SCALABLEMAIN : ptl->getProfileIdc();
#endif

            for( Int p = 0; p < profiles.size(); p++ )
            {
              passedCheck[p] += profileName == profiles[p] || ptl->getProfileCompatibilityFlag(Int(profiles[p]));
            }
          }
        }
      }
    }

    // check whether all layers are compatible to at least one extension profile
    assert( std::find(passedCheck.begin(), passedCheck.end(), numIncludedLayers) != passedCheck.end() );

    NumOutputLayersInOutputLayerSet[i] = 0;

    for( j = 0; j < vps->getNumLayersInIdList(layerSetIdxForOutputLayerSet); j++ )
    {
      NumOutputLayersInOutputLayerSet[i] += vps->getOutputLayerFlag(i, j);
      if( vps->getOutputLayerFlag(i, j) )
      {
        OlsHighestOutputLayerId[i] = vps->getLayerSetLayerIdList(layerSetIdxForOutputLayerSet, j);
      }
    }

    if( NumOutputLayersInOutputLayerSet[i] == 1 && vps->getNumDirectRefLayers(OlsHighestOutputLayerId[i]) > 0 )
    {
      READ_FLAG(uiCode, "alt_output_layer_flag[i]");
      vps->setAltOuputLayerFlag(i, uiCode ? true : false);
    }
    else
    {
      vps->setAltOuputLayerFlag(i, false);
    }

    assert( NumOutputLayersInOutputLayerSet[i] > 0 );
  }

  vps->checkNecessaryLayerFlagCondition();  

  READ_UVLC( uiCode, "vps_num_rep_formats_minus1" );
  vps->setVpsNumRepFormats( uiCode + 1 );

  // The value of vps_num_rep_formats_minus1 shall be in the range of 0 to 255, inclusive.
  assert( vps->getVpsNumRepFormats() > 0 && vps->getVpsNumRepFormats() <= 256 );

  for(i = 0; i < vps->getVpsNumRepFormats(); i++)
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
    for( i = vps->getBaseLayerInternalFlag() ? 1 : 0; i < vps->getMaxLayers(); i++ )
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

  READ_FLAG(uiCode, "max_one_active_ref_layer_flag" );
  vps->setMaxOneActiveRefLayerFlag(uiCode);

  READ_FLAG(uiCode, "vps_poc_lsb_aligned_flag");
  vps->setVpsPocLsbAlignedFlag(uiCode);

  for( i = 1; i< vps->getMaxLayers(); i++ )
  {
    if( vps->getNumDirectRefLayers( vps->getLayerIdInNuh(i) ) == 0  )
    {
      READ_FLAG(uiCode, "poc_lsb_not_present_flag[i]");
      vps->setPocLsbNotPresentFlag(i, uiCode);
    }
  }

  parseVpsDpbSizeTable(vps);

  READ_UVLC( uiCode,           "direct_dep_type_len_minus2"); vps->setDirectDepTypeLen(uiCode+2);

  READ_FLAG(uiCode, "direct_dependency_all_layers_flag"); 
  vps->setDefaultDirectDependecyTypeFlag(uiCode == 1? true : false);

  if( vps->getDefaultDirectDependencyTypeFlag() )
  {
    READ_CODE( vps->getDirectDepTypeLen(), uiCode, "direct_dependency_all_layers_type" ); 
    vps->setDefaultDirectDependecyType(uiCode);
  }

  for( i = vps->getBaseLayerInternalFlag() ? 1 : 2; i < vps->getMaxLayers(); i++ )
  {
    for( j = vps->getBaseLayerInternalFlag() ? 0 : 1; j < i; j++ )
    {
      if( vps->getDirectDependencyFlag(i, j) )
      {
        if (vps->getDefaultDirectDependencyTypeFlag())
        {
          vps->setDirectDependencyType(i, j, vps->getDefaultDirectDependencyType());
        }
        else
        {
          READ_CODE( vps->getDirectDepTypeLen(), uiCode, "direct_dependency_type[i][j]" ); 
          vps->setDirectDependencyType(i, j, uiCode);
        }
      }
    }
  }

  READ_UVLC( uiCode,           "vps_non_vui_extension_length"); vps->setVpsNonVuiExtLength((Int)uiCode);

  // The value of vps_non_vui_extension_length shall be in the range of 0 to 4096, inclusive.
  assert( vps->getVpsNonVuiExtLength() >= 0 && vps->getVpsNonVuiExtLength() <= 4096 );

  Int nonVuiExtByte = uiCode;
  for (i = 1; i <= nonVuiExtByte; i++)
  {
    READ_CODE( 8, uiCode, "vps_non_vui_extension_data_byte" ); //just parse and discard for now.
  }

  READ_FLAG( uiCode, "vps_vui_present_flag"); vps->setVpsVuiPresentFlag(uiCode ? true : false);

  if ( vps->getVpsVuiPresentFlag() )
  {
    while ( m_pcBitstream.getNumBitsRead() % 8 != 0 )
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
  for(i = 0; i < vps->getMaxLayers(); i++)
  {
    vps->setLayerIdInNuh(i, i);
    vps->setLayerIdxInVps(vps->getLayerIdInNuh(i), i);
  }

  // When not present, sub_layers_vps_max_minus1[ i ] is inferred to be equal to vps_max_sub_layers_minus1.
  for( i = 0; i < vps->getMaxLayers(); i++)
  {
    vps->setMaxTSLayersMinus1(i, vps->getMaxTLayers()-1);
  }

  // When not present, max_tid_il_ref_pics_plus1[ i ][ j ] is inferred to be equal to 7.
  for( i = 0; i < vps->getMaxLayers() - 1; i++ )
  {
    for( j = i + 1; j < vps->getMaxLayers(); j++ )
    {
      vps->setMaxTidIlRefPicsPlus1(i, j, 7);
    }
  }

  // When not present, the value of num_add_olss is inferred to be equal to 0.
  // NumOutputLayerSets = num_add_olss + NumLayerSets
  vps->setNumOutputLayerSets( vps->getNumLayerSets() );

  // For i in the range of 0 to NumOutputLayerSets-1, inclusive, the variable LayerSetIdxForOutputLayerSet[ i ] is derived as specified in the following: 
  // LayerSetIdxForOutputLayerSet[ i ] = ( i <= vps_number_layer_sets_minus1 ) ? i : layer_set_idx_for_ols_minus1[ i ] + 1
  for( i = 1; i < vps->getNumOutputLayerSets(); i++ )
  {
    vps->setOutputLayerSetIdx( i, i < vps->getNumLayerSets() ? i : 1 );

    Int lsIdx = vps->getOutputLayerSetIdx(i);

    for( j = 0; j < vps->getNumLayersInIdList(lsIdx); j++ )
    {
      vps->setOutputLayerFlag(i, j, 1);
    }
  }

  // Default output layer set
  // The value of NumLayersInIdList[ 0 ] is set equal to 1 and the value of LayerSetLayerIdList[ 0 ][ 0 ] is set equal to 0.
  vps->setOutputLayerSetIdx(0, 0);

  // The value of output_layer_flag[ 0 ][ 0 ] is inferred to be equal to 1.
  vps->setOutputLayerFlag(0, 0, true);

  vps->deriveNecessaryLayerFlag(0);

  // The value of sub_layer_dpb_info_present_flag[ i ][ 0 ] for any possible value of i is inferred to be equal to 1
  // When not present, the value of sub_layer_dpb_info_present_flag[ i ][ j ] for j greater than 0 and any possible value of i, is inferred to be equal to be equal to 0.
  for( i = 1; i < vps->getNumOutputLayerSets(); i++ )
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
    for(i = 1; i < vps->getMaxLayers(); i++)
    {
      vps->setVpsRepFormatIdx( i, min( (Int)i, vps->getVpsNumRepFormats() - 1 ) );
    }
  }

  vps->setVpsPocLsbAlignedFlag(false);

  // When not present, poc_lsb_not_present_flag[ i ] is inferred to be equal to 0.
  for( i = 1; i< vps->getMaxLayers(); i++ )
  {
    vps->setPocLsbNotPresentFlag(i, 0);
  }

  // set default values for VPS VUI
  defaultVPSVUI( vps );
}

Void TDecCavlc::defaultVPSVUI( TComVPS* vps )
{
  // When not present, the value of all_layers_idr_aligned_flag is inferred to be equal to 0.
  vps->setCrossLayerIrapAlignFlag( false );

  // When single_layer_for_non_irap_flag is not present, it is inferred to be equal to 0.
  vps->setSingleLayerForNonIrapFlag( false );

  // When higher_layer_irap_skip_flag is not present it is inferred to be equal to 0
  vps->setHigherLayerIrapSkipFlag( false );
}

Void  TDecCavlc::parseRepFormat( RepFormat *repFormat, RepFormat *repFormatPrev )
{
  UInt uiCode;
  READ_CODE( 16, uiCode, "pic_width_vps_in_luma_samples" );        repFormat->setPicWidthVpsInLumaSamples ( uiCode );
  READ_CODE( 16, uiCode, "pic_height_vps_in_luma_samples" );       repFormat->setPicHeightVpsInLumaSamples( uiCode );
  READ_FLAG( uiCode, "chroma_and_bit_depth_vps_present_flag" );    repFormat->setChromaAndBitDepthVpsPresentFlag( uiCode ? true : false ); 

  printf("parseRepFormat => %d %d \n", repFormat->getPicWidthVpsInLumaSamples (), repFormat->getPicHeightVpsInLumaSamples () );
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

    if( repFormat->getChromaFormatVpsIdc() == CHROMA_444 )
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

  READ_FLAG( uiCode, "conformance_window_vps_flag" );
  if( uiCode != 0) 
  {
    Window &conf = repFormat->getConformanceWindowVps();
    READ_UVLC( uiCode, "conf_win_vps_left_offset" );         conf.setWindowLeftOffset  ( uiCode );
    READ_UVLC( uiCode, "conf_win_vps_right_offset" );        conf.setWindowRightOffset ( uiCode );
    READ_UVLC( uiCode, "conf_win_vps_top_offset" );          conf.setWindowTopOffset   ( uiCode );
    READ_UVLC( uiCode, "conf_win_vps_bottom_offset" );       conf.setWindowBottomOffset( uiCode );
  }
}

Void TDecCavlc::parseVpsDpbSizeTable( TComVPS *vps )
{
  UInt uiCode;

  vps->calculateMaxSLInLayerSets();
  vps->deriveNumberOfSubDpbs();

  for(Int i = 1; i < vps->getNumOutputLayerSets(); i++)
  {
    Int layerSetIdxForOutputLayerSet = vps->getOutputLayerSetIdx( i );

    READ_FLAG( uiCode, "sub_layer_flag_info_present_flag[i]");  vps->setSubLayerFlagInfoPresentFlag( i, uiCode ? true : false );

    for(Int j = 0; j <= vps->getMaxSLayersInLayerSetMinus1( layerSetIdxForOutputLayerSet ); j++)
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
        for(Int k = 0; k < vps->getNumSubDpbs(layerSetIdxForOutputLayerSet); k++)
        {
          uiCode=0;

          if( vps->getNecessaryLayerFlag(i, k) && ( vps->getBaseLayerInternalFlag() || vps->getLayerSetLayerIdList(layerSetIdxForOutputLayerSet, k) ) )
          {
            READ_UVLC( uiCode, "max_vps_dec_pic_buffering_minus1[i][k][j]" ); vps->setMaxVpsDecPicBufferingMinus1( i, k, j, uiCode );
          }
        }
        READ_UVLC( uiCode, "max_vps_num_reorder_pics[i][j]" );              vps->setMaxVpsNumReorderPics( i, j, uiCode);

        READ_UVLC( uiCode, "max_vps_latency_increase_plus1[i][j]" );        vps->setMaxVpsLatencyIncreasePlus1( i, j, uiCode);
      }
    }
    for(Int j = vps->getMaxTLayers(); j < MAX_TLAYER; j++)
    {
      vps->setSubLayerDpbInfoPresentFlag( i, j, false );
    }
  }

  // Infer values when not signalled
  for(Int i = 1; i < vps->getNumOutputLayerSets(); i++)
  {
    Int layerSetIdxForOutputLayerSet = vps->getOutputLayerSetIdx( i );
    for(Int j = 0; j < MAX_TLAYER; j++)
    {
      if( !vps->getSubLayerDpbInfoPresentFlag(i, j) )  // If sub-layer DPB information is NOT present
      {
        for(Int k = 0; k < vps->getNumLayersInIdList( layerSetIdxForOutputLayerSet ); k++)
        {
          vps->setMaxVpsDecPicBufferingMinus1( i, k, j, vps->getMaxVpsDecPicBufferingMinus1( i, k, j - 1 ) );
        }
        vps->setMaxVpsNumReorderPics( i, j, vps->getMaxVpsNumReorderPics( i, j - 1) );
        vps->setMaxVpsLatencyIncreasePlus1( i, j, vps->getMaxVpsLatencyIncreasePlus1( i, j - 1 ) );
      }
    }
  }
}

Void TDecCavlc::parseVPSVUI(TComVPS *vps)
{
  UInt i,j;
  UInt uiCode;
  READ_FLAG(uiCode, "cross_layer_pic_type_aligned_flag" );
  vps->setCrossLayerPictureTypeAlignFlag(uiCode);

  if( !uiCode ) 
  {
    READ_FLAG(uiCode, "cross_layer_irap_aligned_flag" );
    vps->setCrossLayerIrapAlignFlag(uiCode);
  }
  else
  {
    vps->setCrossLayerIrapAlignFlag(true);
  }

  if( vps->getCrossLayerIrapAlignFlag() )
  {
    READ_FLAG( uiCode, "all_layers_idr_aligned_flag" );
    vps->setCrossLayerAlignedIdrOnlyFlag(uiCode);
  }

  READ_FLAG( uiCode,        "bit_rate_present_vps_flag" );  vps->setBitRatePresentVpsFlag( uiCode ? true : false );
  READ_FLAG( uiCode,        "pic_rate_present_vps_flag" );  vps->setPicRatePresentVpsFlag( uiCode ? true : false );

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

  READ_FLAG( uiCode, "video_signal_info_idx_present_flag" ); vps->setVideoSigPresentVpsFlag( uiCode == 1 );
  if (vps->getVideoSigPresentVpsFlag())
  {
    READ_CODE(4, uiCode, "vps_num_video_signal_info_minus1" ); vps->setNumVideoSignalInfo(uiCode + 1);
  }
  else
  {
    vps->setNumVideoSignalInfo(vps->getMaxLayers() - (vps->getBaseLayerInternalFlag() ? 0 : 1));
  }

  for(i = 0; i < vps->getNumVideoSignalInfo(); i++)
  {
    READ_CODE(3, uiCode, "video_vps_format" ); vps->setVideoVPSFormat(i,uiCode);
    READ_FLAG(uiCode, "video_full_range_vps_flag" ); vps->setVideoFullRangeVpsFlag(i,uiCode);
    READ_CODE(8, uiCode, "colour_primaries_vps" ); vps->setColorPrimaries(i,uiCode);
    READ_CODE(8, uiCode, "transfer_characteristics_vps" ); vps->setTransCharacter(i,uiCode);
    READ_CODE(8, uiCode, "matrix_coeffs_vps" );vps->setMaxtrixCoeff(i,uiCode);
  }

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

  READ_FLAG( uiCode, "tiles_not_in_use_flag" ); vps->setTilesNotInUseFlag(uiCode == 1);

  if( !uiCode )
  {
    for( i = vps->getBaseLayerInternalFlag() ? 0 : 1; i < vps->getMaxLayers(); i++ )
    {
      READ_FLAG( uiCode, "tiles_in_use_flag[ i ]" ); vps->setTilesInUseFlag(i, (uiCode == 1));

      if( uiCode )
      {
        READ_FLAG( uiCode, "loop_filter_not_across_tiles_flag[ i ]" ); vps->setLoopFilterNotAcrossTilesFlag(i, (uiCode == 1));
      }
      else
      {
        vps->setLoopFilterNotAcrossTilesFlag(i, false);
      }
    }

    for( i = vps->getBaseLayerInternalFlag() ? 1 : 2; i < vps->getMaxLayers(); i++ )
    {
      for( j = 0; j < vps->getNumDirectRefLayers(vps->getLayerIdInNuh(i)); j++ )
      {
        UInt layerIdx = vps->getLayerIdxInVps(vps->getRefLayerId(vps->getLayerIdInNuh(i), j));

        if( vps->getTilesInUseFlag(i) && vps->getTilesInUseFlag(layerIdx) )
        {
          READ_FLAG( uiCode, "tile_boundaries_aligned_flag[i][j]" ); vps->setTileBoundariesAlignedFlag(i,j,(uiCode == 1));
        }
      }
    }
  }

  READ_FLAG( uiCode, "wpp_not_in_use_flag" ); vps->setWppNotInUseFlag(uiCode == 1);
  if( !uiCode )
  {
    for( i = vps->getBaseLayerInternalFlag() ? 0 : 1; i < vps->getMaxLayers(); i++ )
    {
      READ_FLAG( uiCode, "wpp_in_use_flag[ i ]" ); vps->setWppInUseFlag(i, (uiCode == 1));
    }
  }

  READ_FLAG(uiCode, "single_layer_for_non_irap_flag" ); vps->setSingleLayerForNonIrapFlag(uiCode == 1 ? true : false);

  READ_FLAG(uiCode, "higher_layer_irap_skip_flag" ); vps->setHigherLayerIrapSkipFlag(uiCode == 1 ? true : false);

  // When single_layer_for_non_irap_flag is equal to 0, higher_layer_irap_skip_flag shall be equal to 0
  if( !vps->getSingleLayerForNonIrapFlag() )
  {
    assert( !vps->getHigherLayerIrapSkipFlag() );
  }

  READ_FLAG( uiCode, "ilp_restricted_ref_layers_flag" ); vps->setIlpRestrictedRefLayersFlag( uiCode == 1 );
  if( vps->getIlpRestrictedRefLayersFlag())
  {
    for(i = 1; i < vps->getMaxLayers(); i++)
    {
      for(j = 0; j < vps->getNumDirectRefLayers(vps->getLayerIdInNuh(i)); j++)
      {
        if( vps->getBaseLayerInternalFlag() || vps->getRefLayerId(vps->getLayerIdInNuh(i), j) )
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
  }

#if O0164_MULTI_LAYER_HRD
  READ_FLAG(uiCode, "vps_vui_bsp_hrd_present_flag" ); vps->setVpsVuiBspHrdPresentFlag(uiCode);

  if( vps->getVpsVuiBspHrdPresentFlag() )
  {
    parseVpsVuiBspHrdParams(vps);
  }
#endif

  for( i = 1; i < vps->getMaxLayers(); i++ )
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
}

Void TDecCavlc::parseSPSExtension( TComSPS* pcSPS )
{
  UInt uiCode;
  // more syntax elements to be parsed here

  READ_FLAG( uiCode, "inter_view_mv_vert_constraint_flag" );
  
#if VIEW_SCALABILITY 
  pcSPS->setInterViewMvVertConstraintFlag(uiCode == 1 ? true : false);
#else
  // Vertical MV component restriction is not used in SHVC CTC
  assert( uiCode == 0 );
#endif
}

#if CGS_3D_ASYMLUT
/*
Void TDecCavlc::xParse3DAsymLUT( TCom3DAsymLUT * pc3DAsymLUT )
{
  UInt uiNumRefLayersM1;
  READ_UVLC( uiNumRefLayersM1, "num_cm_ref_layers_minus1" );
  assert( uiNumRefLayersM1 <= 61 );
  for( UInt i = 0 ; i <= uiNumRefLayersM1 ; i++ )
  {
    UInt uiRefLayerId;
    READ_CODE( 6, uiRefLayerId, "cm_ref_layer_id" );
    pc3DAsymLUT->addRefLayerId( uiRefLayerId );
  }

  UInt uiCurOctantDepth, uiCurPartNumLog2, uiInputBitDepthM8, uiOutputBitDepthM8, uiResQaunBit, uiDeltaBits;;
  
  READ_CODE( 2, uiCurOctantDepth, "cm_octant_depth" ); 
  READ_CODE( 2, uiCurPartNumLog2, "cm_y_part_num_log2" );     

  UInt uiChromaInputBitDepthM8, uiChromaOutputBitDepthM8;

  READ_UVLC( uiInputBitDepthM8, "cm_input_luma_bit_depth_minus8" );
  READ_UVLC( uiChromaInputBitDepthM8 , "cm_input_chroma_bit_depth_minus8" );
  READ_UVLC( uiOutputBitDepthM8, "cm_output_luma_bit_depth_minus8" );
  READ_UVLC( uiChromaOutputBitDepthM8,  "cm_output_chroma_bit_depth_minus8" );
  READ_CODE( 2, uiResQaunBit, "cm_res_quant_bit" );

  READ_CODE( 2, uiDeltaBits, "cm_flc_bits" );
  pc3DAsymLUT->setDeltaBits(uiDeltaBits + 1);

  Int nAdaptCThresholdU = 1 << ( uiChromaInputBitDepthM8 + 8 - 1 );
  Int nAdaptCThresholdV = 1 << ( uiChromaInputBitDepthM8 + 8 - 1 );

  if( uiCurOctantDepth == 1 )
  {
    Int delta = 0;
    READ_SVLC( delta, "cm_adapt_threshold_u_delta" );
    nAdaptCThresholdU += delta;
    READ_SVLC( delta, "cm_adapt_threshold_v_delta" );
    nAdaptCThresholdV += delta;
  }

  pc3DAsymLUT->destroy();
  pc3DAsymLUT->create( uiCurOctantDepth, uiInputBitDepthM8 + 8, uiChromaInputBitDepthM8 + 8, uiOutputBitDepthM8 + 8, uiChromaOutputBitDepthM8 + 8, uiCurPartNumLog2, nAdaptCThresholdU, nAdaptCThresholdV );
  pc3DAsymLUT->setResQuantBit( uiResQaunBit );

#if R0164_CGS_LUT_BUGFIX_CHECK
  pc3DAsymLUT->xInitCuboids();
#endif
  xParse3DAsymLUTOctant( pc3DAsymLUT, 0, 0, 0, 0, 1 << pc3DAsymLUT->getCurOctantDepth() );
#if R0164_CGS_LUT_BUGFIX_CHECK
  printf("============= Before 'xCuboidsFilledCheck()': ================\n");
  pc3DAsymLUT->display();
  pc3DAsymLUT->xCuboidsFilledCheck( false );
  printf("============= After 'xCuboidsFilledCheck()': =================\n");
  pc3DAsymLUT->display();
#endif
}

Void TDecCavlc::xParse3DAsymLUTOctant( TCom3DAsymLUT * pc3DAsymLUT, Int nDepth, Int yIdx, Int uIdx, Int vIdx, Int nLength )
{
  UInt uiOctantSplit = nDepth < pc3DAsymLUT->getCurOctantDepth();
  if( nDepth < pc3DAsymLUT->getCurOctantDepth() )
  {
    READ_FLAG( uiOctantSplit, "split_octant_flag" );
  }
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
          xParse3DAsymLUTOctant( pc3DAsymLUT, nDepth + 1, yIdx + l * nHalfLength * nYPartNum, uIdx + m * nHalfLength, vIdx + n * nHalfLength, nHalfLength );
        }
      }
    }
  }
  else
  {
    Int nFLCbits = pc3DAsymLUT->getMappingShift()-pc3DAsymLUT->getResQuantBit()-pc3DAsymLUT->getDeltaBits() ; 
    nFLCbits = nFLCbits >= 0 ? nFLCbits:0;

    for( Int l = 0; l < nYPartNum; l++ )
    {
      Int shift = pc3DAsymLUT->getCurOctantDepth() - nDepth;

      for( Int nVertexIdx = 0; nVertexIdx < 4; nVertexIdx++ )
      {
        UInt uiCodeVertex = 0;
        Int deltaY = 0, deltaU = 0, deltaV = 0;

        READ_FLAG( uiCodeVertex, "coded_res_flag" );

        if( uiCodeVertex )
        {
          xReadParam( deltaY, nFLCbits );
          xReadParam( deltaU, nFLCbits );
          xReadParam( deltaV, nFLCbits );
        }

        pc3DAsymLUT->setCuboidVertexResTree( yIdx + (l<<shift), uIdx, vIdx, nVertexIdx, deltaY, deltaU, deltaV );

        for( Int m = 1; m < (1<<shift); m++ )
        {
          pc3DAsymLUT->setCuboidVertexResTree( yIdx + (l<<shift) + m, uIdx, vIdx, nVertexIdx, 0, 0, 0 );
#if R0164_CGS_LUT_BUGFIX_CHECK
          pc3DAsymLUT->xSetFilled( yIdx + (l<<shift) + m , uIdx , vIdx );
#endif
        }
      }
#if R0164_CGS_LUT_BUGFIX_CHECK
      pc3DAsymLUT->xSetExplicit( yIdx + (l<<shift) , uIdx , vIdx );
#endif
    }

    for( Int u=0; u<nLength; u++ )
    {
      for( Int v=0; v<nLength; v++ )
      {
        if( u!=0 || v!=0 )
        {
          for( Int y=0; y<nLength*nYPartNum; y++ )
          {
            for( Int nVertexIdx = 0; nVertexIdx < 4; nVertexIdx++ )
            {
              pc3DAsymLUT->setCuboidVertexResTree( yIdx + y, uIdx + u, vIdx + v, nVertexIdx, 0, 0, 0 );
#if R0164_CGS_LUT_BUGFIX_CHECK
              pc3DAsymLUT->xSetFilled( yIdx + y, uIdx + u, vIdx + v );
#endif
            }
          }
        }
      }
    }
  }
}
*/

Void TDecCavlc::xReadParam( Int& param, Int rParam )
{
  UInt prefix;
  UInt codeWord ;
  UInt rSymbol;
  UInt sign;

  READ_UVLC( prefix, "res_coeff_q");
  READ_CODE (rParam, codeWord, "res_coeff_r");
  rSymbol = (prefix<<rParam) + codeWord;

  if(rSymbol)
  {
    READ_FLAG(sign, "res_coeff_s");
    param = sign ? -(Int)(rSymbol) : (Int)(rSymbol);
  }
  else param = 0;
}
#endif

Void TDecCavlc::parseVpsVuiBspHrdParams( TComVPS *vps )
{
  UInt uiCode;
  assert (vps->getTimingInfo()->getTimingInfoPresentFlag() == 1);
  READ_UVLC( uiCode, "vps_num_add_hrd_params" ); vps->setVpsNumAddHrdParams(uiCode);
  vps->createBspHrdParamBuffer(vps->getVpsNumAddHrdParams()); // Also allocates m_cprmsAddPresentFlag and m_numSubLayerHrdMinus

  for( Int i = vps->getNumHrdParameters(), j = 0; i < vps->getNumHrdParameters() + vps->getVpsNumAddHrdParams(); i++, j++ ) // j = i - vps->getNumHrdParameters()
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
      if( i == vps->getNumHrdParameters() )
      {
        vps->getBspHrd(j)->copyCommonInformation( vps->getHrdParameters( vps->getNumHrdParameters() - 1 ) );
      }
      else
      {
        vps->getBspHrd(j)->copyCommonInformation( vps->getBspHrd( j - 1 ) );
      }
    }
  }

  if( vps->getNumHrdParameters() + vps->getVpsNumAddHrdParams() > 0 )
  {
    // When vps_base_layer_internal_flag is equal to 1, the value of num_partitions_in_scheme_minus1[ 0 ][ 0 ] is inferred to be equal to 0.
    if( vps->getBaseLayerInternalFlag() )
    {
      vps->setNumPartitionsInSchemeMinus1(0, 0, 0);
    }

    for (Int h = 1; h < vps->getNumOutputLayerSets(); h++)
    {
      Int lsIdx = vps->getOutputLayerSetIdx(h);
      READ_UVLC(uiCode, "num_signalled_partitioning_schemes[h]"); vps->setNumSignalledPartitioningSchemes(h, uiCode);

      // When vps_base_layer_internal_flag is equal to 1, the value of num_partitions_in_scheme_minus1[ h ][ 0 ] is inferred to be equal to NumLayersInIdList[ h ] - 1.
      if( vps->getBaseLayerInternalFlag() )
      {
        vps->setNumPartitionsInSchemeMinus1(h, 0, vps->getNumLayersInIdList(h) - 1);
      }

      for (Int j = 1; j < vps->getNumSignalledPartitioningSchemes(h) + 1; j++)
      {
        READ_UVLC(uiCode, "num_partitions_in_scheme_minus1[h][j]"); vps->setNumPartitionsInSchemeMinus1(h, j, uiCode);

        for( Int k = 0; k <= vps->getNumPartitionsInSchemeMinus1(h, j); k++ )
        {
          for( Int r = 0; r < vps->getNumLayersInIdList(lsIdx); r++ )
          {
            READ_FLAG(uiCode, "layer_included_in_partition_flag[h][j][k][r]"); vps->setLayerIncludedInPartitionFlag(h, j, k, r, uiCode ? true : false);
          }
        }
      }

      for( Int i = 0; i < vps->getNumSignalledPartitioningSchemes(h) + 1; i++ )
      {
        for( Int t = 0; t <= vps->getMaxSLayersInLayerSetMinus1(lsIdx); t++ )
        {
          READ_UVLC(uiCode, "num_bsp_schedules_minus1[h][i][t]");              vps->setNumBspSchedulesMinus1(h, i, t, uiCode);

          for( Int j = 0; j <= vps->getNumBspSchedulesMinus1(h, i, t); j++)
          {
            for( Int k = 0; k <= vps->getNumPartitionsInSchemeMinus1(h, i); k++ )
            {
              if( vps->getNumHrdParameters() + vps->getVpsNumAddHrdParams() > 1 )
              {
                Int numBits = 1;

                while( (1 << numBits) < (vps->getNumHrdParameters() + vps->getVpsNumAddHrdParams()) )
                {
                  numBits++;
                }

                READ_CODE(numBits, uiCode, "bsp_hrd_idx[h][i][t][j][k]");      vps->setBspHrdIdx(h, i, t, j, k, uiCode);
              }

              READ_UVLC(uiCode, "bsp_sched_idx[h][i][t][j][k]");    vps->setBspSchedIdx(h, i, t, j, k, uiCode);
            }
          }
        }
      }

      // To be done: Check each layer included in not more than one BSP in every partitioning scheme,
      // and other related checks associated with layers in bitstream partitions.

    }
  }
}
#endif //SVC_EXTENSION
//! \}

