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

/** \file     TDecCAVLC.cpp
\brief    CAVLC decoder class
*/

#include "PccAvcTDecCAVLC_avc.h"
//#include "SEIread.h"
//#include "TDecSlice.h"
#include "PccAvcTComChromaFormat.h"
#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "TLibCommon/TComCodingStatistics.h"
#endif
//#include "TDecConformance.h"

//! \ingroup TLibDecoder
//! \{

#if ENC_DEC_TRACE

Void xTraceVPSHeader() { fprintf( g_hTrace, "=========== Video Parameter Set     ===========\n" ); }

Void xTraceSPSHeader() { fprintf( g_hTrace, "=========== Sequence Parameter Set  ===========\n" ); }

Void xTracePPSHeader() { fprintf( g_hTrace, "=========== Picture Parameter Set  ===========\n" ); }

Void xTraceSliceHeader() { fprintf( g_hTrace, "=========== Slice ===========\n" ); }

#endif

// ====================================================================================================================
// Constructor / destructor / create / destroy
// ====================================================================================================================

TDecCavlc_avc::TDecCavlc_avc() { }

TDecCavlc_avc::~TDecCavlc_avc() { }

// ====================================================================================================================
// Public member functions
// ====================================================================================================================
/*
Void TDecCavlc::parseShortTermRefPicSet( TComSPS* sps, TComReferencePictureSet* rps, Int idx ) {
  UInt code;
  UInt interRPSPred;
  if ( idx > 0 ) {
    READ_FLAG( interRPSPred, "inter_ref_pic_set_prediction_flag" );
    rps->setInterRPSPrediction( interRPSPred );
  } else {
    interRPSPred = false;
    rps->setInterRPSPrediction( false );
  }

  if ( interRPSPred ) {
    UInt bit;
    if ( idx == sps->getRPSList()->getNumberOfReferencePictureSets() ) {
      READ_UVLC( code, "delta_idx_minus1" );  // delta index of the Reference Picture Set used for prediction minus 1
    } else {
      code = 0;
    }
    assert( code <= idx - 1 );  // delta_idx_minus1 shall not be larger than idx-1, otherwise we will predict from a
                                // negative row position that does not exist. When idx equals 0 there is no legal value
                                // and interRPSPred must be zero. See J0185-r2
    Int rIdx = idx - 1 - code;
    assert( rIdx <= idx - 1 && rIdx >= 0 );  // Made assert tighter; if rIdx = idx then prediction is done from itself.
                                             // rIdx must belong to range 0, idx-1, inclusive, see J0185-r2
    TComReferencePictureSet* rpsRef = sps->getRPSList()->getReferencePictureSet( rIdx );
    Int                      k = 0, k0 = 0, k1 = 0;
    READ_CODE( 1, bit, "delta_rps_sign" );          // delta_RPS_sign
    READ_UVLC( code, "abs_delta_rps_minus1" );      // absolute delta RPS minus 1
    Int deltaRPS = ( 1 - 2 * bit ) * ( code + 1 );  // delta_RPS
    for ( Int j = 0; j <= rpsRef->getNumberOfPictures(); j++ ) {
      READ_CODE( 1, bit, "used_by_curr_pic_flag" );  // first bit is "1" if Idc is 1
      Int refIdc = bit;
      if ( refIdc == 0 ) {
        READ_CODE( 1, bit, "use_delta_flag" );  // second bit is "1" if Idc is 2, "0" otherwise.
        refIdc = bit << 1;                      // second bit is "1" if refIdc is 2, "0" if refIdc = 0.
      }
      if ( refIdc == 1 || refIdc == 2 ) {
        Int deltaPOC = deltaRPS + ( ( j < rpsRef->getNumberOfPictures() ) ? rpsRef->getDeltaPOC( j ) : 0 );
        rps->setDeltaPOC( k, deltaPOC );
        rps->setUsed( k, ( refIdc == 1 ) );

        if ( deltaPOC < 0 ) {
          k0++;
        } else {
          k1++;
        }
        k++;
      }
      rps->setRefIdc( j, refIdc );
    }
    rps->setNumRefIdc( rpsRef->getNumberOfPictures() + 1 );
    rps->setNumberOfPictures( k );
    rps->setNumberOfNegativePictures( k0 );
    rps->setNumberOfPositivePictures( k1 );
    rps->sortDeltaPOC();
  } else {
    READ_UVLC( code, "num_negative_pics" );
    rps->setNumberOfNegativePictures( code );
    READ_UVLC( code, "num_positive_pics" );
    rps->setNumberOfPositivePictures( code );
    Int prev = 0;
    Int poc;
    for ( Int j = 0; j < rps->getNumberOfNegativePictures(); j++ ) {
      READ_UVLC( code, "delta_poc_s0_minus1" );
      poc  = prev - code - 1;
      prev = poc;
      rps->setDeltaPOC( j, poc );
      READ_FLAG( code, "used_by_curr_pic_s0_flag" );
      rps->setUsed( j, code );
    }
    prev = 0;
    for ( Int j = rps->getNumberOfNegativePictures();
          j < rps->getNumberOfNegativePictures() + rps->getNumberOfPositivePictures(); j++ ) {
      READ_UVLC( code, "delta_poc_s1_minus1" );
      poc  = prev + code + 1;
      prev = poc;
      rps->setDeltaPOC( j, poc );
      READ_FLAG( code, "used_by_curr_pic_s1_flag" );
      rps->setUsed( j, code );
    }
    rps->setNumberOfPictures( rps->getNumberOfNegativePictures() + rps->getNumberOfPositivePictures() );
  }
#if PRINT_RPS_INFO
  rps->printDeltaPOC();
#endif
}

Void TDecCavlc::parsePPS( TComPPS* pcPPS ) {
#if ENC_DEC_TRACE
  xTracePPSHeader();
#endif
  UInt uiCode;

  Int iCode;

  READ_UVLC_CHK( uiCode, "pps_pic_parameter_set_id", 0, 63 );
  assert( uiCode <= 63 );
  pcPPS->setPPSId( uiCode );

  READ_UVLC_CHK( uiCode, "pps_seq_parameter_set_id", 0, 15 );
  assert( uiCode <= 15 );
  pcPPS->setSPSId( uiCode );

  READ_FLAG( uiCode, "dependent_slice_segments_enabled_flag" );
  pcPPS->setDependentSliceSegmentsEnabledFlag( uiCode == 1 );

  READ_FLAG( uiCode, "output_flag_present_flag" );
  pcPPS->setOutputFlagPresentFlag( uiCode == 1 );

  READ_CODE( 3, uiCode, "num_extra_slice_header_bits" );
  pcPPS->setNumExtraSliceHeaderBits( uiCode );

  READ_FLAG( uiCode, "sign_data_hiding_enabled_flag" );
  pcPPS->setSignDataHidingEnabledFlag( uiCode );

  READ_FLAG( uiCode, "cabac_init_present_flag" );
  pcPPS->setCabacInitPresentFlag( uiCode ? true : false );

  READ_UVLC_CHK( uiCode, "num_ref_idx_l0_default_active_minus1", 0, 14 );
  assert( uiCode <= 14 );
  pcPPS->setNumRefIdxL0DefaultActive( uiCode + 1 );

  READ_UVLC_CHK( uiCode, "num_ref_idx_l1_default_active_minus1", 0, 14 );
  assert( uiCode <= 14 );
  pcPPS->setNumRefIdxL1DefaultActive( uiCode + 1 );

  READ_SVLC_CHK( iCode, "init_qp_minus26", std::numeric_limits<Int>::min(), 25 );
  pcPPS->setPicInitQPMinus26( iCode );
  READ_FLAG( uiCode, "constrained_intra_pred_flag" );
  pcPPS->setConstrainedIntraPred( uiCode ? true : false );
  READ_FLAG( uiCode, "transform_skip_enabled_flag" );
  pcPPS->setUseTransformSkip( uiCode ? true : false );

  READ_FLAG( uiCode, "cu_qp_delta_enabled_flag" );
  pcPPS->setUseDQP( uiCode ? true : false );
  if ( pcPPS->getUseDQP() ) {
    READ_UVLC( uiCode, "diff_cu_qp_delta_depth" );
    pcPPS->setMaxCuDQPDepth( uiCode );
  } else {
    pcPPS->setMaxCuDQPDepth( 0 );
  }
  READ_SVLC_CHK( iCode, "pps_cb_qp_offset", -12, 12 );
  pcPPS->setQpOffset( COMPONENT_Cb, iCode );
  assert( pcPPS->getQpOffset( COMPONENT_Cb ) >= -12 );
  assert( pcPPS->getQpOffset( COMPONENT_Cb ) <= 12 );

  READ_SVLC_CHK( iCode, "pps_cr_qp_offset", -12, 12 );
  pcPPS->setQpOffset( COMPONENT_Cr, iCode );
  assert( pcPPS->getQpOffset( COMPONENT_Cr ) >= -12 );
  assert( pcPPS->getQpOffset( COMPONENT_Cr ) <= 12 );

  assert( MAX_NUM_COMPONENT <= 3 );

  READ_FLAG( uiCode, "pps_slice_chroma_qp_offsets_present_flag" );
  pcPPS->setSliceChromaQpFlag( uiCode ? true : false );

  READ_FLAG( uiCode, "weighted_pred_flag" );  // Use of Weighting Prediction (P_SLICE)
  pcPPS->setUseWP( uiCode == 1 );
  READ_FLAG( uiCode, "weighted_bipred_flag" );  // Use of Bi-Directional Weighting Prediction (B_SLICE)
  pcPPS->setWPBiPred( uiCode == 1 );

  READ_FLAG( uiCode, "transquant_bypass_enabled_flag" );
  pcPPS->setTransquantBypassEnabledFlag( uiCode ? true : false );
  READ_FLAG( uiCode, "tiles_enabled_flag" );
  pcPPS->setTilesEnabledFlag( uiCode == 1 );
  READ_FLAG( uiCode, "entropy_coding_sync_enabled_flag" );
  pcPPS->setEntropyCodingSyncEnabledFlag( uiCode == 1 );

  if ( pcPPS->getTilesEnabledFlag() ) {
    READ_UVLC( uiCode, "num_tile_columns_minus1" );
    pcPPS->setNumTileColumnsMinus1( uiCode );
    READ_UVLC( uiCode, "num_tile_rows_minus1" );
    pcPPS->setNumTileRowsMinus1( uiCode );
    READ_FLAG( uiCode, "uniform_spacing_flag" );
    pcPPS->setTileUniformSpacingFlag( uiCode == 1 );

    const UInt tileColumnsMinus1 = pcPPS->getNumTileColumnsMinus1();
    const UInt tileRowsMinus1    = pcPPS->getNumTileRowsMinus1();

    if ( !pcPPS->getTileUniformSpacingFlag() ) {
      if ( tileColumnsMinus1 > 0 ) {
        std::vector<Int> columnWidth( tileColumnsMinus1 );
        for ( UInt i = 0; i < tileColumnsMinus1; i++ ) {
          READ_UVLC( uiCode, "column_width_minus1" );
          columnWidth[i] = uiCode + 1;
        }
        pcPPS->setTileColumnWidth( columnWidth );
      }

      if ( tileRowsMinus1 > 0 ) {
        std::vector<Int> rowHeight( tileRowsMinus1 );
        for ( UInt i = 0; i < tileRowsMinus1; i++ ) {
          READ_UVLC( uiCode, "row_height_minus1" );
          rowHeight[i] = uiCode + 1;
        }
        pcPPS->setTileRowHeight( rowHeight );
      }
    }
    assert( ( tileColumnsMinus1 + tileRowsMinus1 ) != 0 );
    READ_FLAG( uiCode, "loop_filter_across_tiles_enabled_flag" );
    pcPPS->setLoopFilterAcrossTilesEnabledFlag( uiCode ? true : false );
  }
  READ_FLAG( uiCode, "pps_loop_filter_across_slices_enabled_flag" );
  pcPPS->setLoopFilterAcrossSlicesEnabledFlag( uiCode ? true : false );
  READ_FLAG( uiCode, "deblocking_filter_control_present_flag" );
  pcPPS->setDeblockingFilterControlPresentFlag( uiCode ? true : false );
  if ( pcPPS->getDeblockingFilterControlPresentFlag() ) {
    READ_FLAG( uiCode, "deblocking_filter_override_enabled_flag" );
    pcPPS->setDeblockingFilterOverrideEnabledFlag( uiCode ? true : false );
    READ_FLAG( uiCode, "pps_deblocking_filter_disabled_flag" );
    pcPPS->setPPSDeblockingFilterDisabledFlag( uiCode ? true : false );
    if ( !pcPPS->getPPSDeblockingFilterDisabledFlag() ) {
      READ_SVLC( iCode, "pps_beta_offset_div2" );
      pcPPS->setDeblockingFilterBetaOffsetDiv2( iCode );
      READ_SVLC( iCode, "pps_tc_offset_div2" );
      pcPPS->setDeblockingFilterTcOffsetDiv2( iCode );
    }
  }
  READ_FLAG( uiCode, "pps_scaling_list_data_present_flag" );
  pcPPS->setScalingListPresentFlag( uiCode ? true : false );
  if ( pcPPS->getScalingListPresentFlag() ) { parseScalingList( &( pcPPS->getScalingList() ) ); }

  READ_FLAG( uiCode, "lists_modification_present_flag" );
  pcPPS->setListsModificationPresentFlag( uiCode );

  READ_UVLC( uiCode, "log2_parallel_merge_level_minus2" );
  pcPPS->setLog2ParallelMergeLevelMinus2( uiCode );

  READ_FLAG( uiCode, "slice_segment_header_extension_present_flag" );
  pcPPS->setSliceHeaderExtensionPresentFlag( uiCode );

  READ_FLAG( uiCode, "pps_extension_present_flag" );
  if ( uiCode ) {
#if ENC_DEC_TRACE || RExt__DECODER_DEBUG_BIT_STATISTICS
    static const TChar* syntaxStrings[] = {"pps_range_extension_flag", "pps_multilayer_extension_flag",
                                           "pps_extension_6bits[0]",   "pps_extension_6bits[1]",
                                           "pps_extension_6bits[2]",   "pps_extension_6bits[3]",
                                           "pps_extension_6bits[4]",   "pps_extension_6bits[5]"};
#endif

    Bool pps_extension_flags[NUM_PPS_EXTENSION_FLAGS];
    for ( Int i = 0; i < NUM_PPS_EXTENSION_FLAGS; i++ ) {
      READ_FLAG( uiCode, syntaxStrings[i] );
      pps_extension_flags[i] = uiCode != 0;
    }

    Bool bSkipTrailingExtensionBits = false;
    for ( Int i = 0; i < NUM_PPS_EXTENSION_FLAGS; i++ )  // loop used so that the order is determined by the enum.
    {
      if ( pps_extension_flags[i] ) {
        switch ( PPSExtensionFlagIndex( i ) ) {
          case PPS_EXT__REXT: {
            TComPPSRExt& ppsRangeExtension = pcPPS->getPpsRangeExtension();
            assert( !bSkipTrailingExtensionBits );

            if ( pcPPS->getUseTransformSkip() ) {
              READ_UVLC( uiCode, "log2_max_transform_skip_block_size_minus2" );
              ppsRangeExtension.setLog2MaxTransformSkipBlockSize( uiCode + 2 );
            }

            READ_FLAG( uiCode, "cross_component_prediction_enabled_flag" );
            ppsRangeExtension.setCrossComponentPredictionEnabledFlag( uiCode != 0 );

            READ_FLAG( uiCode, "chroma_qp_offset_list_enabled_flag" );
            if ( uiCode == 0 ) {
              ppsRangeExtension.clearChromaQpOffsetList();
              ppsRangeExtension.setDiffCuChromaQpOffsetDepth( 0 );
            } else {
              READ_UVLC( uiCode, "diff_cu_chroma_qp_offset_depth" );
              ppsRangeExtension.setDiffCuChromaQpOffsetDepth( uiCode );
              UInt tableSizeMinus1 = 0;
              READ_UVLC_CHK( tableSizeMinus1, "chroma_qp_offset_list_len_minus1", 0, MAX_QP_OFFSET_LIST_SIZE - 1 );
              assert( tableSizeMinus1 < MAX_QP_OFFSET_LIST_SIZE );

              for ( Int cuChromaQpOffsetIdx = 0; cuChromaQpOffsetIdx <= ( tableSizeMinus1 ); cuChromaQpOffsetIdx++ ) {
                Int cbOffset;
                Int crOffset;
                READ_SVLC_CHK( cbOffset, "cb_qp_offset_list[i]", -12, 12 );
                assert( cbOffset >= -12 && cbOffset <= 12 );
                READ_SVLC_CHK( crOffset, "cr_qp_offset_list[i]", -12, 12 );
                assert( crOffset >= -12 && crOffset <= 12 );
                // table uses +1 for index (see comment inside the function)
                ppsRangeExtension.setChromaQpOffsetListEntry( cuChromaQpOffsetIdx + 1, cbOffset, crOffset );
              }
              assert( ppsRangeExtension.getChromaQpOffsetListLen() == tableSizeMinus1 + 1 );
            }

            READ_UVLC( uiCode, "log2_sao_offset_scale_luma" );
            ppsRangeExtension.setLog2SaoOffsetScale( CHANNEL_TYPE_LUMA, uiCode );
            READ_UVLC( uiCode, "log2_sao_offset_scale_chroma" );
            ppsRangeExtension.setLog2SaoOffsetScale( CHANNEL_TYPE_CHROMA, uiCode );
          } break;
          case PPS_EXT__SCC: {
            TComPPSSCC& ppsScreenExtension = pcPPS->getPpsScreenExtension();
            READ_FLAG( uiCode, "curr_pic_as_ref_enabled_pps_flag" );
            pcPPS->getPpsScreenExtension().setUseIntraBlockCopy( uiCode != 0 );
            READ_FLAG( uiCode, "adaptive_colour_trans_flag" );
            ppsScreenExtension.setUseColourTrans( uiCode != 0 );
            if ( ppsScreenExtension.getUseColourTrans() ) {
              READ_FLAG( uiCode, "pps_slice_act_qp_offset_present_flag" );
              ppsScreenExtension.setUseSliceACTOffset( uiCode != 0 );

              Int actQpOffset;
              READ_SVLC( actQpOffset, "pps_act_y_qp_offset_plus5" );
              ppsScreenExtension.setActQpOffset( COMPONENT_Y, actQpOffset - 5 );
              READ_SVLC( actQpOffset, "pps_act_cb_qp_offset_plus5" );
              ppsScreenExtension.setActQpOffset( COMPONENT_Cb, actQpOffset - 5 );
              READ_SVLC( actQpOffset, "pps_act_cr_qp_offset_plus3" );
              ppsScreenExtension.setActQpOffset( COMPONENT_Cr, actQpOffset - 3 );
            } else {
              ppsScreenExtension.setUseSliceACTOffset( false );
              ppsScreenExtension.setActQpOffset( COMPONENT_Y, -5 );
              ppsScreenExtension.setActQpOffset( COMPONENT_Cb, -5 );
              ppsScreenExtension.setActQpOffset( COMPONENT_Cr, -3 );
            }
            READ_FLAG( uiCode, "palette_predictor_initializer_flag" );
            ppsScreenExtension.setUsePalettePredictor( uiCode );
            if ( uiCode ) {
              READ_UVLC( uiCode, "pps_num_palette_entries" );
              ppsScreenExtension.setNumPalettePred( uiCode );
              if ( uiCode ) {
                READ_FLAG( uiCode, "monochrome_palette_flag" );
                ppsScreenExtension.setMonochromePaletteFlag( uiCode != 0 );
                READ_UVLC( uiCode, "luma_bit_depth_entry_minus8" );
                ppsScreenExtension.setPalettePredictorBitDepth( CHANNEL_TYPE_LUMA, uiCode + 8 );
                if ( !ppsScreenExtension.getMonochromePaletteFlag() ) {
                  READ_UVLC( uiCode, "chroma_bit_depth_entry_minus8" );
                  ppsScreenExtension.setPalettePredictorBitDepth( CHANNEL_TYPE_CHROMA, uiCode + 8 );
                }
                for ( Int k = 0; k < ( ppsScreenExtension.getMonochromePaletteFlag() ? 1 : 3 ); k++ ) {
                  for ( Int j = 0; j < ppsScreenExtension.getNumPalettePred(); j++ ) {
#if RExt__DECODER_DEBUG_BIT_STATISTICS
                    READ_CODE( ppsScreenExtension.getPalettePredictorBitDepth( toChannelType( ComponentID( k ) ) ),
                               uiCode, "palette_predictor_initializers" );
#else
                    READ_CODE( ppsScreenExtension.getPalettePredictorBitDepth( toChannelType( ComponentID( k ) ) ),
                               uiCode, "palette_predictor_initializers" );
#endif
                    ppsScreenExtension.getPalettePred( k )[j] = uiCode;
                  }
                }
              }
            } else {
              ppsScreenExtension.setNumPalettePred( 0 );
            }
          } break;
          default: bSkipTrailingExtensionBits = true; break;
        }
      }
    }
    if ( bSkipTrailingExtensionBits ) {
      while ( xMoreRbspData() ) { READ_FLAG( uiCode, "pps_extension_data_flag" ); }
    }
  }
  xReadRbspTrailingBits();
}

Void TDecCavlc::parseVUI( TComVUI* pcVUI, TComSPS* pcSPS ) {
#if ENC_DEC_TRACE
  fprintf( g_hTrace, "----------- vui_parameters -----------\n" );
#endif
  UInt uiCode;

  READ_FLAG( uiCode, "aspect_ratio_info_present_flag" );
  pcVUI->setAspectRatioInfoPresentFlag( uiCode );
  if ( pcVUI->getAspectRatioInfoPresentFlag() ) {
    READ_CODE( 8, uiCode, "aspect_ratio_idc" );
    pcVUI->setAspectRatioIdc( uiCode );
    if ( pcVUI->getAspectRatioIdc() == 255 ) {
      READ_CODE( 16, uiCode, "sar_width" );
      pcVUI->setSarWidth( uiCode );
      READ_CODE( 16, uiCode, "sar_height" );
      pcVUI->setSarHeight( uiCode );
    }
  }

  READ_FLAG( uiCode, "overscan_info_present_flag" );
  pcVUI->setOverscanInfoPresentFlag( uiCode );
  if ( pcVUI->getOverscanInfoPresentFlag() ) {
    READ_FLAG( uiCode, "overscan_appropriate_flag" );
    pcVUI->setOverscanAppropriateFlag( uiCode );
  }

  READ_FLAG( uiCode, "video_signal_type_present_flag" );
  pcVUI->setVideoSignalTypePresentFlag( uiCode );
  if ( pcVUI->getVideoSignalTypePresentFlag() ) {
    READ_CODE( 3, uiCode, "video_format" );
    pcVUI->setVideoFormat( uiCode );
    READ_FLAG( uiCode, "video_full_range_flag" );
    pcVUI->setVideoFullRangeFlag( uiCode );
    READ_FLAG( uiCode, "colour_description_present_flag" );
    pcVUI->setColourDescriptionPresentFlag( uiCode );
    if ( pcVUI->getColourDescriptionPresentFlag() ) {
      READ_CODE( 8, uiCode, "colour_primaries" );
      pcVUI->setColourPrimaries( uiCode );
      READ_CODE( 8, uiCode, "transfer_characteristics" );
      pcVUI->setTransferCharacteristics( uiCode );
      READ_CODE( 8, uiCode, "matrix_coeffs" );
      pcVUI->setMatrixCoefficients( uiCode );
    }
  }

  READ_FLAG( uiCode, "chroma_loc_info_present_flag" );
  pcVUI->setChromaLocInfoPresentFlag( uiCode );
  if ( pcVUI->getChromaLocInfoPresentFlag() ) {
    READ_UVLC( uiCode, "chroma_sample_loc_type_top_field" );
    pcVUI->setChromaSampleLocTypeTopField( uiCode );
    READ_UVLC( uiCode, "chroma_sample_loc_type_bottom_field" );
    pcVUI->setChromaSampleLocTypeBottomField( uiCode );
  }

  READ_FLAG( uiCode, "neutral_chroma_indication_flag" );
  pcVUI->setNeutralChromaIndicationFlag( uiCode );

  READ_FLAG( uiCode, "field_seq_flag" );
  pcVUI->setFieldSeqFlag( uiCode );

  READ_FLAG( uiCode, "frame_field_info_present_flag" );
  pcVUI->setFrameFieldInfoPresentFlag( uiCode );

  READ_FLAG( uiCode, "default_display_window_flag" );
  if ( uiCode != 0 ) {
    Window& defDisp = pcVUI->getDefaultDisplayWindow();
    READ_UVLC( uiCode, "def_disp_win_left_offset" );
    defDisp.setWindowLeftOffset( uiCode * TComSPS::getWinUnitX( pcSPS->getChromaFormatIdc() ) );
    READ_UVLC( uiCode, "def_disp_win_right_offset" );
    defDisp.setWindowRightOffset( uiCode * TComSPS::getWinUnitX( pcSPS->getChromaFormatIdc() ) );
    READ_UVLC( uiCode, "def_disp_win_top_offset" );
    defDisp.setWindowTopOffset( uiCode * TComSPS::getWinUnitY( pcSPS->getChromaFormatIdc() ) );
    READ_UVLC( uiCode, "def_disp_win_bottom_offset" );
    defDisp.setWindowBottomOffset( uiCode * TComSPS::getWinUnitY( pcSPS->getChromaFormatIdc() ) );
  }

  TimingInfo* timingInfo = pcVUI->getTimingInfo();
  READ_FLAG( uiCode, "vui_timing_info_present_flag" );
  timingInfo->setTimingInfoPresentFlag( uiCode ? true : false );
  if ( timingInfo->getTimingInfoPresentFlag() ) {
    READ_CODE( 32, uiCode, "vui_num_units_in_tick" );
    timingInfo->setNumUnitsInTick( uiCode );
    READ_CODE( 32, uiCode, "vui_time_scale" );
    timingInfo->setTimeScale( uiCode );
    READ_FLAG( uiCode, "vui_poc_proportional_to_timing_flag" );
    timingInfo->setPocProportionalToTimingFlag( uiCode ? true : false );
    if ( timingInfo->getPocProportionalToTimingFlag() ) {
      READ_UVLC( uiCode, "vui_num_ticks_poc_diff_one_minus1" );
      timingInfo->setNumTicksPocDiffOneMinus1( uiCode );
    }

    READ_FLAG( uiCode, "vui_hrd_parameters_present_flag" );
    pcVUI->setHrdParametersPresentFlag( uiCode );
    if ( pcVUI->getHrdParametersPresentFlag() ) {
      parseHrdParameters( pcVUI->getHrdParameters(), 1, pcSPS->getMaxTLayers() - 1 );
    }
  }

  READ_FLAG( uiCode, "bitstream_restriction_flag" );
  pcVUI->setBitstreamRestrictionFlag( uiCode );
  if ( pcVUI->getBitstreamRestrictionFlag() ) {
    READ_FLAG( uiCode, "tiles_fixed_structure_flag" );
    pcVUI->setTilesFixedStructureFlag( uiCode );
    READ_FLAG( uiCode, "motion_vectors_over_pic_boundaries_flag" );
    pcVUI->setMotionVectorsOverPicBoundariesFlag( uiCode );
    READ_FLAG( uiCode, "restricted_ref_pic_lists_flag" );
    pcVUI->setRestrictedRefPicListsFlag( uiCode );
    READ_UVLC( uiCode, "min_spatial_segmentation_idc" );
    pcVUI->setMinSpatialSegmentationIdc( uiCode );
    assert( uiCode < 4096 );
    READ_UVLC( uiCode, "max_bytes_per_pic_denom" );
    pcVUI->setMaxBytesPerPicDenom( uiCode );
    READ_UVLC( uiCode, "max_bits_per_min_cu_denom" );
    pcVUI->setMaxBitsPerMinCuDenom( uiCode );
    READ_UVLC( uiCode, "log2_max_mv_length_horizontal" );
    pcVUI->setLog2MaxMvLengthHorizontal( uiCode );
    READ_UVLC( uiCode, "log2_max_mv_length_vertical" );
    pcVUI->setLog2MaxMvLengthVertical( uiCode );
  }
}

Void TDecCavlc::parseHrdParameters( TComHRD* hrd, Bool commonInfPresentFlag, UInt maxNumSubLayersMinus1 ) {
  UInt uiCode;
  if ( commonInfPresentFlag ) {
    READ_FLAG( uiCode, "nal_hrd_parameters_present_flag" );
    hrd->setNalHrdParametersPresentFlag( uiCode == 1 ? true : false );
    READ_FLAG( uiCode, "vcl_hrd_parameters_present_flag" );
    hrd->setVclHrdParametersPresentFlag( uiCode == 1 ? true : false );
    if ( hrd->getNalHrdParametersPresentFlag() || hrd->getVclHrdParametersPresentFlag() ) {
      READ_FLAG( uiCode, "sub_pic_hrd_params_present_flag" );
      hrd->setSubPicCpbParamsPresentFlag( uiCode == 1 ? true : false );
      if ( hrd->getSubPicCpbParamsPresentFlag() ) {
        READ_CODE( 8, uiCode, "tick_divisor_minus2" );
        hrd->setTickDivisorMinus2( uiCode );
        READ_CODE( 5, uiCode, "du_cpb_removal_delay_increment_length_minus1" );
        hrd->setDuCpbRemovalDelayLengthMinus1( uiCode );
        READ_FLAG( uiCode, "sub_pic_cpb_params_in_pic_timing_sei_flag" );
        hrd->setSubPicCpbParamsInPicTimingSEIFlag( uiCode == 1 ? true : false );
        READ_CODE( 5, uiCode, "dpb_output_delay_du_length_minus1" );
        hrd->setDpbOutputDelayDuLengthMinus1( uiCode );
      }
      READ_CODE( 4, uiCode, "bit_rate_scale" );
      hrd->setBitRateScale( uiCode );
      READ_CODE( 4, uiCode, "cpb_size_scale" );
      hrd->setCpbSizeScale( uiCode );
      if ( hrd->getSubPicCpbParamsPresentFlag() ) {
        READ_CODE( 4, uiCode, "cpb_size_du_scale" );
        hrd->setDuCpbSizeScale( uiCode );
      }
      READ_CODE( 5, uiCode, "initial_cpb_removal_delay_length_minus1" );
      hrd->setInitialCpbRemovalDelayLengthMinus1( uiCode );
      READ_CODE( 5, uiCode, "au_cpb_removal_delay_length_minus1" );
      hrd->setCpbRemovalDelayLengthMinus1( uiCode );
      READ_CODE( 5, uiCode, "dpb_output_delay_length_minus1" );
      hrd->setDpbOutputDelayLengthMinus1( uiCode );
    }
  }
  Int i, j, nalOrVcl;
  for ( i = 0; i <= maxNumSubLayersMinus1; i++ ) {
    READ_FLAG( uiCode, "fixed_pic_rate_general_flag" );
    hrd->setFixedPicRateFlag( i, uiCode == 1 ? true : false );
    if ( !hrd->getFixedPicRateFlag( i ) ) {
      READ_FLAG( uiCode, "fixed_pic_rate_within_cvs_flag" );
      hrd->setFixedPicRateWithinCvsFlag( i, uiCode == 1 ? true : false );
    } else {
      hrd->setFixedPicRateWithinCvsFlag( i, true );
    }

    hrd->setLowDelayHrdFlag( i, 0 );  // Infered to be 0 when not present
    hrd->setCpbCntMinus1( i, 0 );     // Infered to be 0 when not present

    if ( hrd->getFixedPicRateWithinCvsFlag( i ) ) {
      READ_UVLC( uiCode, "elemental_duration_in_tc_minus1" );
      hrd->setPicDurationInTcMinus1( i, uiCode );
    } else {
      READ_FLAG( uiCode, "low_delay_hrd_flag" );
      hrd->setLowDelayHrdFlag( i, uiCode == 1 ? true : false );
    }
    if ( !hrd->getLowDelayHrdFlag( i ) ) {
      READ_UVLC( uiCode, "cpb_cnt_minus1" );
      hrd->setCpbCntMinus1( i, uiCode );
    }

    for ( nalOrVcl = 0; nalOrVcl < 2; nalOrVcl++ ) {
      if ( ( ( nalOrVcl == 0 ) && ( hrd->getNalHrdParametersPresentFlag() ) ) ||
           ( ( nalOrVcl == 1 ) && ( hrd->getVclHrdParametersPresentFlag() ) ) ) {
        for ( j = 0; j <= ( hrd->getCpbCntMinus1( i ) ); j++ ) {
          READ_UVLC( uiCode, "bit_rate_value_minus1" );
          hrd->setBitRateValueMinus1( i, j, nalOrVcl, uiCode );
          READ_UVLC( uiCode, "cpb_size_value_minus1" );
          hrd->setCpbSizeValueMinus1( i, j, nalOrVcl, uiCode );
          if ( hrd->getSubPicCpbParamsPresentFlag() ) {
            READ_UVLC( uiCode, "cpb_size_du_value_minus1" );
            hrd->setDuCpbSizeValueMinus1( i, j, nalOrVcl, uiCode );
            READ_UVLC( uiCode, "bit_rate_du_value_minus1" );
            hrd->setDuBitRateValueMinus1( i, j, nalOrVcl, uiCode );
          }
          READ_FLAG( uiCode, "cbr_flag" );
          hrd->setCbrFlag( i, j, nalOrVcl, uiCode == 1 ? true : false );
        }
      }
    }
  }
}
*/
// syntax for scaling list matrix values
Void TDecCavlc_avc::Scaling_List( int indexA, int sizeOfScalingList, TComSPS_avc* pcSPS ) {
  int  j, scanj;
  UInt delta_scale, lastScale, nextScale;

  lastScale = 8;
  nextScale = 8;

  for ( j = 0; j < sizeOfScalingList; j++ ) {
    scanj = ( sizeOfScalingList == 16 ) ? ZZ_SCAN[j] : ZZ_SCAN8[j];

    if ( nextScale != 0 ) {
      READ_UVLC( delta_scale, "   : delta_sl   " );  // pcSPS->setChroma_format_idc( delta_scale );
      printf( "SPS: delta_scale=%d. \n", pcSPS->getChroma_format_idc() );  // YTT
      // delta_scale              = read_se_v( "   : delta_sl   ", s, &p_Dec->UsedBits );
      nextScale = ( lastScale + delta_scale + 256 ) % 256;
      if ( sizeOfScalingList == 16 )
        pcSPS->setUseDefaultScalingMatrix4x4Flag( indexA, ( Bool )( scanj == 0 && nextScale == 0 ) );
      if ( sizeOfScalingList == 64 )
        pcSPS->setUseDefaultScalingMatrix8x8Flag( indexA, ( Bool )( scanj == 0 && nextScale == 0 ) );
    }

    if ( sizeOfScalingList == 16 ) {
      pcSPS->setScalingList4x4( indexA, scanj, ( nextScale == 0 ) ? lastScale : nextScale );
      lastScale = pcSPS->getScalingList4x4( indexA, scanj );
    }
    if ( sizeOfScalingList == 64 ) {
      pcSPS->setScalingList8x8( indexA, scanj, ( nextScale == 0 ) ? lastScale : nextScale );
      lastScale = pcSPS->getScalingList8x8( indexA, scanj );
    }
  }
}
// fill sps with content of p
Void TDecCavlc_avc::parseSPS( TComSPS_avc* pcSPS ) {
#if ENC_DEC_TRACE
  xTraceSPSHeader();
#endif
  unsigned n_ScalingList;
  UInt     uiCode;
  Int      iCode;

  //printf( "HELLO!!!parseSPS!!!\n" );

  READ_CODE( 8, uiCode, "profile_idc" ); pcSPS->setProfile_idc( uiCode );
  //printf( "uiCode=%d\n", uiCode );
  //printf( "SPS: profile_idc=%d. \n", int( pcSPS->getProfile_idc() ) );  // YTT

  if ( ( pcSPS->getProfile_idc() != BASELINE ) && ( pcSPS->getProfile_idc() != MAIN ) &&
       ( pcSPS->getProfile_idc() != EXTENDED ) && ( pcSPS->getProfile_idc() != FREXT_HP ) &&
       ( pcSPS->getProfile_idc() != FREXT_Hi10P ) && ( pcSPS->getProfile_idc() != FREXT_Hi422 ) &&
       ( pcSPS->getProfile_idc() != FREXT_Hi444 ) && ( pcSPS->getProfile_idc() != FREXT_CAVLC444 )
#if ( MVC_EXTENSION_ENABLE )
       && ( pcSPS->getProfile_idc() != MVC_HIGH ) && ( pcSPS->getProfile_idc() != STEREO_HIGH )
#endif
  ) {
    printf( "Invalid Profile IDC (%d) encountered. \n", pcSPS->getProfile_idc() );
    // return p_Dec->UsedBits;
  }

  READ_FLAG( uiCode, "constrained_set0_flag" );  pcSPS->setConstrained_set0_flag( uiCode > 0 ? true : false );
  //printf( "SPS: constrained_set0_flag=%d. \n", pcSPS->getConstrained_set0_flag() );  // YTT
  READ_FLAG( uiCode, "constrained_set1_flag" );  pcSPS->setConstrained_set1_flag( uiCode > 0 ? true : false );
  //printf( "SPS: constrained_set1_flag=%d. \n", pcSPS->getConstrained_set1_flag() );  // YTT
  READ_FLAG( uiCode, "constrained_set2_flag" );  pcSPS->setConstrained_set2_flag( uiCode > 0 ? true : false );
  //printf( "SPS: constrained_set2_flag=%d. \n", pcSPS->getConstrained_set2_flag() );  // YTT
  READ_FLAG( uiCode, "constrained_set3_flag" );  pcSPS->setConstrained_set3_flag( uiCode > 0 ? true : false );
  //printf( "SPS: constrained_set3_flag=%d. \n", pcSPS->getConstrained_set3_flag() );  // YTT
#if ( MVC_EXTENSION_ENABLE )
  READ_FLAG( uiCode, "constrained_set4_flag" );  pcSPS->setConstrained_set4_flag( uiCode > 0 ? true : false );
  //printf( "SPS: constrained_set4_flag=%d. \n", pcSPS->getConstrained_set4_flag() );  // YTT
  READ_FLAG( uiCode, "constrained_set5_flag" );  pcSPS->setConstrained_set5_flag( uiCode > 0 ? true : false );
  //printf( "SPS: constrained_set5_flag=%d. \n", pcSPS->getConstrained_set5_flag() );  // YTT
  READ_CODE( 2, uiCode, "reserved_zero" );  pcSPS->setReserved_zero( uiCode );
  //printf( "SPS: reserved_zero=%d. \n", pcSPS->getReserved_zero() );  // YTT
#else
  READ_CODE( 4, uiCode, "reserved_zero_4bits" );
  pcSPS->setReserved_zero_4bits( uiCode );
  printf( "SPS: reserved_zero_4bits=%d. \n", pcSPS->getReserved_zero_4bits() );  // YTT

#endif
  if ( pcSPS->getReserved_zero() != 0 ) {
    printf( "Warning, reserved_zero flag not equal to 0. Possibly new constrained_setX flag introduced.\n" );
  }
  READ_CODE( 8, uiCode, "level_idc" ); pcSPS->setLevel_idc( uiCode );
  //printf( "SPS: level_idc=%d. \n", pcSPS->getLevel_idc() );  // YTT
  READ_UVLC( uiCode, "seq_parameter_set_id" );  pcSPS->setSeq_parameter_set_id( uiCode );
  //printf( "SPS: seq_parameter_set_id=%d. \n", pcSPS->getSeq_parameter_set_id() );  // YTT

  // Fidelity Range Extensions stuff
  pcSPS->setChroma_format_idc( 1 );
  //printf( "SPS: chroma_format_idc=%d. \n", pcSPS->getChroma_format_idc() );  // YTT
  pcSPS->setBit_depth_luma_minus8( 0 );
  //printf( "SPS: bit_depth_luma_minus8=%d. \n", pcSPS->getBit_depth_luma_minus8() );  // YTT
  pcSPS->setBit_depth_chroma_minus8( 0 );
  //printf( "SPS: bit_depth_chroma_minus8=%d. \n", pcSPS->getBit_depth_chroma_minus8() );  // YTT
  pcSPS->setLossless_qpprime_flag( 0 );
  //printf( "SPS: lossless_qpprime_flag=%d. \n", pcSPS->getLossless_qpprime_flag() );  // YTT
  pcSPS->setSeparate_colour_plane_flag( 0 );
  //printf( "SPS: separate_colour_plane_flag=%d. \n", pcSPS->getSeparate_colour_plane_flag() );  // YTT

  if ( ( pcSPS->getProfile_idc() == FREXT_HP ) || ( pcSPS->getProfile_idc() == FREXT_Hi10P ) ||
       ( pcSPS->getProfile_idc() == FREXT_Hi422 ) || ( pcSPS->getProfile_idc() == FREXT_Hi444 ) ||
       ( pcSPS->getProfile_idc() == FREXT_CAVLC444 )
#if ( MVC_EXTENSION_ENABLE )
       || ( pcSPS->getProfile_idc() == MVC_HIGH ) || ( pcSPS->getProfile_idc() == STEREO_HIGH )
#endif
  ) {
    READ_UVLC( uiCode, "chroma_format_idc" );    pcSPS->setChroma_format_idc( uiCode );
    //printf( "SPS: chroma_format_idc=%d. \n", pcSPS->getChroma_format_idc() );  // YTT
    if ( pcSPS->getChroma_format_idc() == YUV444 ) {
      READ_FLAG( uiCode, "separate_colour_plane_flag" ); pcSPS->setSeparate_colour_plane_flag( uiCode );
     // printf( "SPS: separate_colour_plane_flag=%d. \n", pcSPS->getSeparate_colour_plane_flag() );  // YTT
    }
    READ_UVLC( uiCode, "bit_depth_luma_minus8" ); pcSPS->setBit_depth_luma_minus8( uiCode );
    //printf( "SPS: bit_depth_luma_minus8=%d. \n", pcSPS->getBit_depth_luma_minus8() );  // YTT
    READ_UVLC( uiCode, "bit_depth_chroma_minus8" ); pcSPS->setBit_depth_chroma_minus8( uiCode );
    //printf( "SPS: bit_depth_chroma_minus8=%d. \n", pcSPS->getBit_depth_chroma_minus8() );  // YTT
    // checking;
    if ( ( pcSPS->getBit_depth_luma_minus8() + 8 > sizeof( imgpel ) * 8 ) ||
         ( pcSPS->getBit_depth_chroma_minus8() + 8 > sizeof( imgpel ) * 8 ) )
      printf(
          "Source picture has higher bit depth than imgpel data type. \nPlease recompile with larger data type for "
          "imgpel.\n" );

    READ_FLAG( uiCode, "lossless_qpprime_flag" );  pcSPS->setLossless_qpprime_flag( uiCode );
    //printf( "SPS: lossless_qpprime_flag=%d. \n", pcSPS->getLossless_qpprime_flag() );  // YTT
    READ_FLAG( uiCode, "seq_scaling_matrix_present_flag" ); pcSPS->setSeq_scaling_matrix_present_flag( uiCode > 0 ? true : false );
    //printf( "SPS: seq_scaling_matrix_present_flag=%d. \n", pcSPS->getSeq_scaling_matrix_present_flag() );  // YTT

    if ( pcSPS->getSeq_scaling_matrix_present_flag() ) {
      n_ScalingList = ( pcSPS->getChroma_format_idc() != YUV444 ) ? 8 : 12;
      for ( int i = 0; i < n_ScalingList; i++ ) {
        READ_FLAG( uiCode, "seq_scaling_list_present_flag" ); pcSPS->setSeq_scaling_list_present_flag( i, uiCode );
        //printf( "SPS: seq_scaling_list_present_flag=%d. \n", pcSPS->getSeq_scaling_list_present_flag( i ) );  // YTT
        if ( pcSPS->getSeq_scaling_list_present_flag( i ) ) {
          if ( i < 6 )
            Scaling_List( i, 16, pcSPS );
          else
            Scaling_List( i - 6, 64, pcSPS );
        }
      }
    }
  }
  READ_UVLC( uiCode, "log2_max_frame_num_minus4" );  pcSPS->setLog2_max_frame_num_minus4( uiCode );
  //printf( "SPS: log2_max_frame_num_minus4=%d. \n", pcSPS->getLog2_max_frame_num_minus4() );  // YTT
  READ_UVLC( uiCode, "pic_order_cnt_type" );  pcSPS->setPic_order_cnt_type( uiCode );
  //printf( "SPS: pic_order_cnt_type=%d. \n", pcSPS->getPic_order_cnt_type() );  // YTT
  if ( pcSPS->getPic_order_cnt_type() == 0 ) {
    READ_UVLC( uiCode, "log2_max_pic_order_cnt_lsb_minus4" ); pcSPS->setLog2_max_pic_order_cnt_lsb_minus4( uiCode );
    //printf( "SPS: log2_max_pic_order_cnt_lsb_minus4=%d. \n", pcSPS->getLog2_max_pic_order_cnt_lsb_minus4() );  // YTT
  } else if ( pcSPS->getPic_order_cnt_type() == 1 ) {
    READ_FLAG( uiCode, "delta_pic_order_always_zero_flag" ); pcSPS->setDelta_pic_order_always_zero_flag( uiCode );
    //printf( "SPS: delta_pic_order_always_zero_flag=%d. \n", pcSPS->getDelta_pic_order_always_zero_flag() );  // YTT
    READ_SVLC( iCode, "offset_for_non_ref_pic" ); pcSPS->setOffset_for_non_ref_pic( iCode );
    //printf( "SPS: offset_for_non_ref_pic=%d. \n", pcSPS->getOffset_for_non_ref_pic() );  // YTT
    READ_SVLC( iCode, "offset_for_top_to_bottom_field" ); pcSPS->setOffset_for_top_to_bottom_field( iCode );
    //printf( "SPS: offset_for_top_to_bottom_field=%d. \n", pcSPS->getOffset_for_top_to_bottom_field() );  // YTT
    READ_UVLC( uiCode, "num_ref_frames_in_pic_order_cnt_cycle" ); pcSPS->setNum_ref_frames_in_pic_order_cnt_cycle( uiCode );
    //printf( "SPS: num_ref_frames_in_pic_order_cnt_cycle=%d. \n", pcSPS->getNum_ref_frames_in_pic_order_cnt_cycle() );  // YTT
    for ( int i = 0; i < pcSPS->getNum_ref_frames_in_pic_order_cnt_cycle(); i++ ) {
      READ_SVLC( iCode, "offset_for_ref_frame" );  pcSPS->setOffset_for_ref_frame( i, iCode );
      //printf( "SPS: offset_for_ref_frame=%d. \n", pcSPS->getOffset_for_ref_frame(i) );  // YTT
    }
  }
  READ_UVLC( uiCode, "num_ref_frames" ); pcSPS->setNum_ref_frames( uiCode );
 // printf( "SPS: num_ref_frames=%d. \n", pcSPS->getNum_ref_frames() );  // YTT
  READ_FLAG( uiCode, "gaps_in_frame_num_value_allowed_flag" ); pcSPS->setGaps_in_frame_num_value_allowed_flag( uiCode );
  //printf( "SPS: gaps_in_frame_num_value_allowed_flag=%d. \n", pcSPS->getGaps_in_frame_num_value_allowed_flag() );  // YTT
  READ_UVLC( uiCode, "pic_width_in_mbs_minus1" );  pcSPS->setPic_width_in_mbs_minus1( uiCode );
  //printf( "SPS: pic_width_in_mbs_minus1=%d. \n", pcSPS->getPic_width_in_mbs_minus1() );  // YTT
  READ_UVLC( uiCode, "pic_height_in_map_units_minus1" ); pcSPS->setPic_height_in_map_units_minus1( uiCode );
  //printf( "SPS: pic_height_in_map_units_minus1=%d. \n", pcSPS->getPic_height_in_map_units_minus1() );  // YTT
  READ_FLAG( uiCode, "frame_mbs_only_flag" ); pcSPS->setFrame_mbs_only_flag( uiCode );
  //printf( "SPS: frame_mbs_only_flag=%d. \n", pcSPS->getFrame_mbs_only_flag() );  // YTT

  /*
  READ_CODE( 4,  uiCode, "sps_video_parameter_set_id");             pcSPS->setVPSId        ( uiCode );
  READ_CODE_CHK( 3,  uiCode, "sps_max_sub_layers_minus1", 0, 6 );   pcSPS->setMaxTLayers   ( uiCode+1 );
  assert(uiCode <= 6);

  READ_FLAG( uiCode, "sps_temporal_id_nesting_flag" );           pcSPS->setTemporalIdNestingFlag ( uiCode > 0 ? true :
false ); if ( pcSPS->getMaxTLayers() == 1 )
  {
    // sps_temporal_id_nesting_flag must be 1 when sps_max_sub_layers_minus1 is 0
    // TDecConformanceCheck::checkRange(uiCode, "sps_temporal_id_nesting_flag", 1U, 1U);
    assert( uiCode == 1 );
  }

  parsePTL(pcSPS->getPTL(), 1, pcSPS->getMaxTLayers() - 1);
  READ_UVLC_CHK( uiCode, "sps_seq_parameter_set_id", 0, 15 );           pcSPS->setSPSId( uiCode );
  assert(uiCode <= 15);

  READ_UVLC_CHK(     uiCode, "chroma_format_idc", 0, 3 );               pcSPS->setChromaFormatIdc( ChromaFormat(uiCode)
); assert(uiCode <= 3);

  if( pcSPS->getChromaFormatIdc() == CHROMA_444 )
  {
    READ_FLAG_CHK(     uiCode, "separate_colour_plane_flag", 0, 0);
    assert(uiCode == 0);
  }

  // pic_width_in_luma_samples and pic_height_in_luma_samples needs conformance checking - multiples of MinCbSizeY
  READ_UVLC_CHK (    uiCode, "pic_width_in_luma_samples", 1, std::numeric_limits<UInt>::max()  );
pcSPS->setPicWidthInLumaSamples ( uiCode    ); READ_UVLC_CHK (    uiCode, "pic_height_in_luma_samples", 1,
std::numeric_limits<UInt>::max() );  pcSPS->setPicHeightInLumaSamples( uiCode    ); READ_FLAG(     uiCode,
"conformance_window_flag"); if (uiCode != 0)
  {
    Window &conf = pcSPS->getConformanceWindow();
    const UInt subWidthC  = TComSPS::getWinUnitX( pcSPS->getChromaFormatIdc() );
    const UInt subHeightC = TComSPS::getWinUnitY( pcSPS->getChromaFormatIdc() );
    READ_UVLC(   uiCode, "conf_win_left_offset" );               conf.setWindowLeftOffset  ( uiCode * subWidthC );
    READ_UVLC(   uiCode, "conf_win_right_offset" );              conf.setWindowRightOffset ( uiCode * subWidthC );
    READ_UVLC(   uiCode, "conf_win_top_offset" );                conf.setWindowTopOffset   ( uiCode * subHeightC );
    READ_UVLC(   uiCode, "conf_win_bottom_offset" );             conf.setWindowBottomOffset( uiCode * subHeightC );
    // TDecConformanceCheck::checkRange<UInt>(conf.getWindowLeftOffset()+conf.getWindowRightOffset(), "conformance
window width in pixels", 0, pcSPS->getPicWidthInLumaSamples()-1);
  }

  READ_UVLC_CHK(     uiCode, "bit_depth_luma_minus8", 0, 8 );
  assert(uiCode <= 8);
#if O0043_BEST_EFFORT_DECODING
  pcSPS->setStreamBitDepth(CHANNEL_TYPE_LUMA, 8 + uiCode);
  const UInt forceDecodeBitDepth = pcSPS->getForceDecodeBitDepth();
  if (forceDecodeBitDepth != 0)
  {
    uiCode = forceDecodeBitDepth - 8;
  }
  assert(uiCode <= 8);
#endif
  pcSPS->setBitDepth(CHANNEL_TYPE_LUMA, 8 + uiCode);

#if O0043_BEST_EFFORT_DECODING
  pcSPS->setQpBDOffset(CHANNEL_TYPE_LUMA, (Int) (6*(pcSPS->getStreamBitDepth(CHANNEL_TYPE_LUMA)-8)) );
#else
  pcSPS->setQpBDOffset(CHANNEL_TYPE_LUMA, (Int) (6*uiCode) );
#endif

  READ_UVLC_CHK( uiCode,    "bit_depth_chroma_minus8", 0, 8 );
  assert(uiCode <= 8);
#if O0043_BEST_EFFORT_DECODING
  pcSPS->setStreamBitDepth(CHANNEL_TYPE_CHROMA, 8 + uiCode);
  if (forceDecodeBitDepth != 0)
  {
    uiCode = forceDecodeBitDepth - 8;
  }
  assert(uiCode <= 8);
#endif
  pcSPS->setBitDepth(CHANNEL_TYPE_CHROMA, 8 + uiCode);
#if O0043_BEST_EFFORT_DECODING
  pcSPS->setQpBDOffset(CHANNEL_TYPE_CHROMA,  (Int) (6*(pcSPS->getStreamBitDepth(CHANNEL_TYPE_CHROMA)-8)) );
#else
  pcSPS->setQpBDOffset(CHANNEL_TYPE_CHROMA,  (Int) (6*uiCode) );
#endif

  READ_UVLC_CHK( uiCode,    "log2_max_pic_order_cnt_lsb_minus4", 0, 12 );   pcSPS->setBitsForPOC( 4 + uiCode );

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
  }

  const UInt maxLog2CtbSize = MAX_CU_DEPTH; // TDecConformanceCheck::getMaxLog2CtbSize(*(pcSPS->getPTL()));
  const UInt minLog2CtbSize = MAX_CU_DEPTH; // TDecConformanceCheck::getMinLog2CtbSize(*(pcSPS->getPTL()));
  READ_UVLC_CHK( uiCode, "log2_min_luma_coding_block_size_minus3", 0, maxLog2CtbSize-3 );
  assert(uiCode <= maxLog2CtbSize-3);
  Int minCbLog2SizeY = uiCode + 3;
  pcSPS->setLog2MinCodingBlockSize(minCbLog2SizeY);

  // Difference + log2MinCUSize must be <= maxLog2CtbSize
  // Difference + log2MinCUSize must be >= minLog2CtbSize
  const UInt minLog2DiffMaxMinLumaCodingBlockSize = minLog2CtbSize < minCbLog2SizeY ? 0 : minLog2CtbSize -
minCbLog2SizeY; const UInt maxLog2DiffMaxMinLumaCodingBlockSize = maxLog2CtbSize - minCbLog2SizeY;

  READ_UVLC_CHK( uiCode, "log2_diff_max_min_luma_coding_block_size", minLog2DiffMaxMinLumaCodingBlockSize,
maxLog2DiffMaxMinLumaCodingBlockSize); assert(uiCode >= minLog2DiffMaxMinLumaCodingBlockSize && uiCode <=
maxLog2DiffMaxMinLumaCodingBlockSize); pcSPS->setLog2DiffMaxMinCodingBlockSize(uiCode);
  



  const Int maxCUDepthDelta = uiCode;
  const Int ctbLog2SizeY = minCbLog2SizeY + maxCUDepthDelta;
  pcSPS->setMaxCUWidth  ( 1<<ctbLog2SizeY );
  pcSPS->setMaxCUHeight ( 1<<ctbLog2SizeY );
  READ_UVLC_CHK( uiCode, "log2_min_luma_transform_block_size_minus2", 0, minCbLog2SizeY-1-2 );
  const UInt minTbLog2SizeY = uiCode + 2;
  pcSPS->setQuadtreeTULog2MinSize( minTbLog2SizeY );

  //  log2_diff <= Min(CtbLog2SizeY, 5) - minTbLog2SizeY
  READ_UVLC_CHK( uiCode, "log2_diff_max_min_luma_transform_block_size", 0, min<UInt>(5U, ctbLog2SizeY) - minTbLog2SizeY
); pcSPS->setQuadtreeTULog2MaxSize( uiCode + pcSPS->getQuadtreeTULog2MinSize() ); pcSPS->setMaxTrSize( 1<<(uiCode +
pcSPS->getQuadtreeTULog2MinSize()) );

  READ_UVLC_CHK( uiCode, "max_transform_hierarchy_depth_inter", 0, ctbLog2SizeY - minTbLog2SizeY);
pcSPS->setQuadtreeTUMaxDepthInter( uiCode+1 ); READ_UVLC_CHK( uiCode, "max_transform_hierarchy_depth_intra", 0,
ctbLog2SizeY - minTbLog2SizeY);    pcSPS->setQuadtreeTUMaxDepthIntra( uiCode+1 );

  Int addCuDepth = max (0, minCbLog2SizeY - (Int)pcSPS->getQuadtreeTULog2MinSize() );
  pcSPS->setMaxTotalCUDepth( maxCUDepthDelta + addCuDepth  + getMaxCUDepthOffset(pcSPS->getChromaFormatIdc(),
pcSPS->getQuadtreeTULog2MinSize()) );

  READ_FLAG( uiCode, "scaling_list_enabled_flag" );                 pcSPS->setScalingListFlag ( uiCode );
  if(pcSPS->getScalingListFlag())
  {
    READ_FLAG( uiCode, "sps_scaling_list_data_present_flag" );                 pcSPS->setScalingListPresentFlag ( uiCode
); if(pcSPS->getScalingListPresentFlag ())
    {
      parseScalingList( &(pcSPS->getScalingList()) );
    }
  }
  READ_FLAG( uiCode, "amp_enabled_flag" );                          pcSPS->setUseAMP( uiCode );
  READ_FLAG( uiCode, "sample_adaptive_offset_enabled_flag" );       pcSPS->setUseSAO ( uiCode ? true : false );

  READ_FLAG( uiCode, "pcm_enabled_flag" ); pcSPS->setUsePCM( uiCode ? true : false );
  if( pcSPS->getUsePCM() )
  {
#if O0043_BEST_EFFORT_DECODING
    READ_CODE_CHK( 4, uiCode, "pcm_sample_bit_depth_luma_minus1",   0, pcSPS->getStreamBitDepth(CHANNEL_TYPE_LUMA) );
pcSPS->setPCMBitDepth    ( CHANNEL_TYPE_LUMA, 1 + uiCode ); READ_CODE_CHK( 4, uiCode,
"pcm_sample_bit_depth_chroma_minus1", 0, pcSPS->getStreamBitDepth(CHANNEL_TYPE_LUMA) );    pcSPS->setPCMBitDepth    (
CHANNEL_TYPE_CHROMA, 1 + uiCode ); #else READ_CODE_CHK( 4, uiCode, "pcm_sample_bit_depth_luma_minus1",   0,
pcSPS->getBitDepth(CHANNEL_TYPE_LUMA) );          pcSPS->setPCMBitDepth    ( CHANNEL_TYPE_LUMA, 1 + uiCode );
    READ_CODE_CHK( 4, uiCode, "pcm_sample_bit_depth_chroma_minus1", 0, pcSPS->getBitDepth(CHANNEL_TYPE_CHROMA) );
pcSPS->setPCMBitDepth    ( CHANNEL_TYPE_CHROMA, 1 + uiCode ); #endif READ_UVLC_CHK( uiCode,
"log2_min_pcm_luma_coding_block_size_minus3", std::min<UInt>(minCbLog2SizeY, 5 )-3, std::min<UInt>(ctbLog2SizeY, 5)-3);
    const UInt log2MinIpcmCbSizeY = uiCode+3;
    pcSPS->setPCMLog2MinSize (log2MinIpcmCbSizeY);
    READ_UVLC_CHK( uiCode, "log2_diff_max_min_pcm_luma_coding_block_size", 0, (std::min<UInt>(ctbLog2SizeY,5) -
log2MinIpcmCbSizeY) ); pcSPS->setPCMLog2MaxSize ( uiCode+pcSPS->getPCMLog2MinSize() ); READ_FLAG( uiCode,
"pcm_loop_filter_disable_flag" );                 pcSPS->setPCMFilterDisableFlag ( uiCode ? true : false );
  }

  READ_UVLC_CHK( uiCode, "num_short_term_ref_pic_sets", 0, 64 );
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
    READ_UVLC_CHK( uiCode, "num_long_term_ref_pics_sps", 0, 32 );
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
  if (uiCode)
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
              READ_FLAG( uiCode, "transform_skip_rotation_enabled_flag");
spsRangeExtension.setTransformSkipRotationEnabledFlag(uiCode != 0); READ_FLAG( uiCode,
"transform_skip_context_enabled_flag");      spsRangeExtension.setTransformSkipContextEnabledFlag (uiCode != 0);
              READ_FLAG( uiCode, "implicit_rdpcm_enabled_flag");
spsRangeExtension.setRdpcmEnabledFlag(RDPCM_SIGNAL_IMPLICIT, (uiCode != 0)); READ_FLAG( uiCode,
"explicit_rdpcm_enabled_flag");              spsRangeExtension.setRdpcmEnabledFlag(RDPCM_SIGNAL_EXPLICIT, (uiCode !=
0)); READ_FLAG( uiCode, "extended_precision_processing_flag"); spsRangeExtension.setExtendedPrecisionProcessingFlag
(uiCode != 0); READ_FLAG( uiCode, "intra_smoothing_disabled_flag"); spsRangeExtension.setIntraSmoothingDisabledFlag
(uiCode != 0); READ_FLAG( uiCode, "high_precision_offsets_enabled_flag");
spsRangeExtension.setHighPrecisionOffsetsEnabledFlag (uiCode != 0); READ_FLAG( uiCode,
"persistent_rice_adaptation_enabled_flag");  spsRangeExtension.setPersistentRiceAdaptationEnabledFlag (uiCode != 0);
              READ_FLAG( uiCode, "cabac_bypass_alignment_enabled_flag");
spsRangeExtension.setCabacBypassAlignmentEnabledFlag  (uiCode != 0);
            }
            break;
          case SPS_EXT__SCC:
            {
              TComSPSSCC &screenExtension = pcSPS->getSpsScreenExtension();
              READ_FLAG( uiCode, "intra_block_copy_enabled_flag" );           screenExtension.setUseIntraBlockCopy(
uiCode != 0 ); READ_FLAG( uiCode, "palette_mode_enabled_flag" );               screenExtension.setUsePaletteMode( uiCode
!= 0 );

              UInt MaxDPBSize = 0;
              if (!screenExtension.getUseIntraBlockCopy())
              {
                MaxDPBSize = 6;
              }
              else
              {
                MaxDPBSize = 7;
              }

              UInt uiPicSizeInSamplesY = pcSPS->getPicWidthInLumaSamples()*pcSPS->getPicHeightInLumaSamples();
              UInt uiMaxLumaPs = 36864;
              Level::Name lLevel = pcSPS->getPTL()->getGeneralPTL()->getLevelIdc();
              UInt uiMaxDPBSize = MaxDPBSize; // from Annex A
              switch (lLevel)
              {
                case Level::LEVEL1:
                  uiMaxLumaPs = 36864;             break;
                case Level::LEVEL2:
                  uiMaxLumaPs = 122880;            break;
                case Level::LEVEL2_1:
                  uiMaxLumaPs = 245760;            break;
                case Level::LEVEL3:
                  uiMaxLumaPs = 552960;            break;
                case Level::LEVEL3_1:
                  uiMaxLumaPs = 983040;            break;
                case Level::LEVEL4:
                  uiMaxLumaPs = 2228224;           break;
                case Level::LEVEL4_1:
                  uiMaxLumaPs = 2228224;           break;
                case Level::LEVEL5:
                  uiMaxLumaPs = 8912896;           break;
                case Level::LEVEL5_1:
                  uiMaxLumaPs = 8912896;           break;
                case Level::LEVEL5_2:
                  uiMaxLumaPs = 8912896;           break;
                case Level::LEVEL6:
                  uiMaxLumaPs = 35651584;          break;
                case Level::LEVEL6_1:
                  uiMaxLumaPs = 35651584;          break;
                case Level::LEVEL6_2:
                  uiMaxLumaPs = 35651584;          break;
                default:
                  uiMaxLumaPs = 35651584;          break;
              }
              if ( uiPicSizeInSamplesY <= (uiMaxLumaPs >> 2) )
              {
                uiMaxDPBSize = (4 * MaxDPBSize < 16) ? 4 * MaxDPBSize : 16; // from Annex A
              }
              else if ( uiPicSizeInSamplesY <= (uiMaxLumaPs >> 1) )
              {
                uiMaxDPBSize = (2 * MaxDPBSize < 16) ? 2 * MaxDPBSize : 16;
              }
              else if ( uiPicSizeInSamplesY <= ((3 * uiMaxLumaPs) >> 2) )
              {
                uiMaxDPBSize = ((4 * MaxDPBSize) / 3 < 16) ? (4 * MaxDPBSize) / 3 : 16;
              }
              else
              {
                uiMaxDPBSize = MaxDPBSize;
              }

              for(UInt ij=0; ij <= pcSPS->getMaxTLayers()-1; ij++)
              {
                if (pcSPS->getMaxDecPicBuffering(ij) > uiMaxDPBSize)
                {
                  std::cerr <<"Bitstream compliance Error m_uiMaxDecPicBuffering[" << ij << "]" <<
pcSPS->getMaxDecPicBuffering(ij) << "shall not be bigger than MaxDPBSize -1 and smaller than 0" << std::endl;
                  assert(false);
                  exit(1);
                }
              }

              if ( screenExtension.getUsePaletteMode() )//decode only when palette mode is enabled
              {
                READ_UVLC( uiCode, "palette_max_size" );                      screenExtension.setPaletteMaxSize( uiCode
); READ_UVLC( uiCode, "delta_palette_max_predictor_size" );      screenExtension.setPaletteMaxPredSize(
uiCode+screenExtension.getPaletteMaxSize() ); assert( screenExtension.getPaletteMaxPredSize() <= 128 );
                assert(screenExtension.getPaletteMaxSize() != 0 || screenExtension.getPaletteMaxPredSize() == 0);

                READ_FLAG( uiCode, "sps_palette_predictor_initializer_flag" );
                screenExtension.setUsePalettePredictor(uiCode);
                assert(screenExtension.getPaletteMaxSize() != 0 || screenExtension.getUsePalettePredictor() == false);
                assert(screenExtension.getUsePaletteMode() != 0 || screenExtension.getUsePalettePredictor() == false);

                if( uiCode )
                {
                  READ_UVLC( uiCode, "sps_num_palette_entries_minus1" ); uiCode++;
                  screenExtension.setNumPalettePred(uiCode);
                  for ( Int k=0; k < (pcSPS->getChromaFormatIdc() == CHROMA_400 ? 1 : 3); k++ )
                  {
                    for ( Int j=0; j< screenExtension.getNumPalettePred(); j++ )
                    {
#if RExt__DECODER_DEBUG_BIT_STATISTICS
                      xReadCode(  pcSPS->getBitDepth( toChannelType( ComponentID( k ) ) ), uiCode,
"palette_predictor_initializers" ); #else xReadCode(  pcSPS->getBitDepth( toChannelType( ComponentID( k ) ) ), uiCode );
#endif
                      screenExtension.getPalettePred( k )[j] = uiCode;
                    }
                  }
                }
                else
                {
                  screenExtension.setNumPalettePred(0);
                }
              }
              READ_CODE( 2, uiCode, "motion_vector_resolution_control_idc" );
screenExtension.setMotionVectorResolutionControlIdc( uiCode ); READ_FLAG( uiCode, "intra_boundary_filter_disabled_flag"
);     screenExtension.setDisableIntraBoundaryFilter( uiCode != 0 );
            }
            break;
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
  */
  xReadRbspTrailingBits();
}
/*
Void TDecCavlc::parseVPS( TComVPS* pcVPS ) {
#if ENC_DEC_TRACE
  xTraceVPSHeader();
#endif
  UInt uiCode;

  READ_CODE( 4, uiCode, "vps_video_parameter_set_id" );
  pcVPS->setVPSId( uiCode );
  READ_FLAG( uiCode, "vps_base_layer_internal_flag" );
  assert( uiCode == 1 );
  READ_FLAG( uiCode, "vps_base_layer_available_flag" );
  assert( uiCode == 1 );
  READ_CODE( 6, uiCode, "vps_max_layers_minus1" );
  READ_CODE( 3, uiCode, "vps_max_sub_layers_minus1" );
  pcVPS->setMaxTLayers( uiCode + 1 );
  assert( uiCode + 1 <= MAX_TLAYER );
  READ_FLAG( uiCode, "vps_temporal_id_nesting_flag" );
  pcVPS->setTemporalNestingFlag( uiCode ? true : false );
  assert( pcVPS->getMaxTLayers() > 1 || pcVPS->getTemporalNestingFlag() );
  READ_CODE( 16, uiCode, "vps_reserved_0xffff_16bits" );
  assert( uiCode == 0xffff );
  parsePTL( pcVPS->getPTL(), true, pcVPS->getMaxTLayers() - 1 );
  UInt subLayerOrderingInfoPresentFlag;
  READ_FLAG( subLayerOrderingInfoPresentFlag, "vps_sub_layer_ordering_info_present_flag" );
  for ( UInt i = 0; i <= pcVPS->getMaxTLayers() - 1; i++ ) {
    READ_UVLC( uiCode, "vps_max_dec_pic_buffering_minus1[i]" );
    pcVPS->setMaxDecPicBuffering( uiCode + 1, i );
    READ_UVLC( uiCode, "vps_max_num_reorder_pics[i]" );
    pcVPS->setNumReorderPics( uiCode, i );
    READ_UVLC( uiCode, "vps_max_latency_increase_plus1[i]" );
    pcVPS->setMaxLatencyIncrease( uiCode, i );

    if ( !subLayerOrderingInfoPresentFlag ) {
      for ( i++; i <= pcVPS->getMaxTLayers() - 1; i++ ) {
        pcVPS->setMaxDecPicBuffering( pcVPS->getMaxDecPicBuffering( 0 ), i );
        pcVPS->setNumReorderPics( pcVPS->getNumReorderPics( 0 ), i );
        pcVPS->setMaxLatencyIncrease( pcVPS->getMaxLatencyIncrease( 0 ), i );
      }
      break;
    }
  }

  assert( pcVPS->getNumHrdParameters() < MAX_VPS_OP_SETS_PLUS1 );
  assert( pcVPS->getMaxNuhReservedZeroLayerId() < MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1 );
  READ_CODE( 6, uiCode, "vps_max_layer_id" );
  pcVPS->setMaxNuhReservedZeroLayerId( uiCode );
  READ_UVLC( uiCode, "vps_num_layer_sets_minus1" );
  pcVPS->setMaxOpSets( uiCode + 1 );
  for ( UInt opsIdx = 1; opsIdx <= ( pcVPS->getMaxOpSets() - 1 ); opsIdx++ ) {
    // Operation point set
    for ( UInt i = 0; i <= pcVPS->getMaxNuhReservedZeroLayerId(); i++ ) {
      READ_FLAG( uiCode, "layer_id_included_flag[opsIdx][i]" );
      pcVPS->setLayerIdIncludedFlag( uiCode == 1 ? true : false, opsIdx, i );
    }
  }

  TimingInfo* timingInfo = pcVPS->getTimingInfo();
  READ_FLAG( uiCode, "vps_timing_info_present_flag" );
  timingInfo->setTimingInfoPresentFlag( uiCode ? true : false );
  if ( timingInfo->getTimingInfoPresentFlag() ) {
    READ_CODE( 32, uiCode, "vps_num_units_in_tick" );
    timingInfo->setNumUnitsInTick( uiCode );
    READ_CODE( 32, uiCode, "vps_time_scale" );
    timingInfo->setTimeScale( uiCode );
    READ_FLAG( uiCode, "vps_poc_proportional_to_timing_flag" );
    timingInfo->setPocProportionalToTimingFlag( uiCode ? true : false );
    if ( timingInfo->getPocProportionalToTimingFlag() ) {
      READ_UVLC( uiCode, "vps_num_ticks_poc_diff_one_minus1" );
      timingInfo->setNumTicksPocDiffOneMinus1( uiCode );
    }

    READ_UVLC( uiCode, "vps_num_hrd_parameters" );
    pcVPS->setNumHrdParameters( uiCode );

    if ( pcVPS->getNumHrdParameters() > 0 ) { pcVPS->createHrdParamBuffer(); }
    for ( UInt i = 0; i < pcVPS->getNumHrdParameters(); i++ ) {
      READ_UVLC( uiCode, "hrd_layer_set_idx[i]" );
      pcVPS->setHrdOpSetIdx( uiCode, i );
      if ( i > 0 ) {
        READ_FLAG( uiCode, "cprms_present_flag[i]" );
        pcVPS->setCprmsPresentFlag( uiCode == 1 ? true : false, i );
      } else {
        pcVPS->setCprmsPresentFlag( true, i );
      }

      parseHrdParameters( pcVPS->getHrdParameters( i ), pcVPS->getCprmsPresentFlag( i ), pcVPS->getMaxTLayers() - 1 );
    }
  }

  READ_FLAG( uiCode, "vps_extension_flag" );
  if ( uiCode ) {
    while ( xMoreRbspData() ) { READ_FLAG( uiCode, "vps_extension_data_flag" ); }
  }

  xReadRbspTrailingBits();
}
*/
/*
Void TDecCavlc::parseSliceHeader( TComSlice*           pcSlice,
                                  ParameterSetManager* parameterSetManager,
                                  const Int            prevTid0POC ) {
  UInt uiCode;
  Int  iCode;

#if ENC_DEC_TRACE
  xTraceSliceHeader();
#endif
  TComPPS* pps = NULL;
  TComSPS* sps = NULL;

  UInt firstSliceSegmentInPic;
  READ_FLAG( firstSliceSegmentInPic, "first_slice_segment_in_pic_flag" );
  if ( pcSlice->getRapPicFlag() ) {
    READ_FLAG( uiCode, "no_output_of_prior_pics_flag" );  // ignored -- updated already
    pcSlice->setNoOutputPriorPicsFlag( uiCode ? true : false );
  }
  READ_UVLC( uiCode, "slice_pic_parameter_set_id" );
  pcSlice->setPPSId( uiCode );
  pps = parameterSetManager->getPPS( uiCode );
  //! KS: need to add error handling code here, if PPS is not available
  assert( pps != 0 );
  sps = parameterSetManager->getSPS( pps->getSPSId() );
  //! KS: need to add error handling code here, if SPS is not available
  assert( sps != 0 );

  const ChromaFormat chFmt        = sps->getChromaFormatIdc();
  const UInt         numValidComp = getNumberValidComponents( chFmt );
  const Bool         bChroma      = ( chFmt != CHROMA_400 );

  if ( pps->getDependentSliceSegmentsEnabledFlag() && ( !firstSliceSegmentInPic ) ) {
    READ_FLAG( uiCode, "dependent_slice_segment_flag" );
    pcSlice->setDependentSliceSegmentFlag( uiCode ? true : false );
  } else {
    pcSlice->setDependentSliceSegmentFlag( false );
  }
  Int numCTUs = ( ( sps->getPicWidthInLumaSamples() + sps->getMaxCUWidth() - 1 ) / sps->getMaxCUWidth() ) *
                ( ( sps->getPicHeightInLumaSamples() + sps->getMaxCUHeight() - 1 ) / sps->getMaxCUHeight() );
  UInt sliceSegmentAddress     = 0;
  Int  bitsSliceSegmentAddress = 0;
  while ( numCTUs > ( 1 << bitsSliceSegmentAddress ) ) { bitsSliceSegmentAddress++; }

  if ( !firstSliceSegmentInPic ) { READ_CODE( bitsSliceSegmentAddress, sliceSegmentAddress, "slice_segment_address" ); }
  // set uiCode to equal slice start address (or dependent slice start address)
  pcSlice->setSliceSegmentCurStartCtuTsAddr(
      sliceSegmentAddress );  // this is actually a Raster-Scan (RS) address, but we do not have the RS->TS conversion
                              // table defined yet.
  pcSlice->setSliceSegmentCurEndCtuTsAddr( numCTUs );  // Set end as the last CTU of the picture.

  if ( !pcSlice->getDependentSliceSegmentFlag() ) {
    pcSlice->setSliceCurStartCtuTsAddr( sliceSegmentAddress );  // this is actually a Raster-Scan (RS) address, but we
                                                                // do not have the RS->TS conversion table defined yet.
    pcSlice->setSliceCurEndCtuTsAddr( numCTUs );
  }

  if ( !pcSlice->getDependentSliceSegmentFlag() ) {
    Bool bTwoVersionsOfCurrDecPicFlag = ( pps->getPpsScreenExtension().getUseIntraBlockCopy() &&
                                          ( sps->getUseSAO() || !pps->getPPSDeblockingFilterDisabledFlag() ||
                                            pps->getDeblockingFilterOverrideEnabledFlag() ) );
    for ( Int i = 0; i < pps->getNumExtraSliceHeaderBits(); i++ ) {
      READ_FLAG( uiCode, "slice_reserved_flag[]" );  // ignored
    }

    READ_UVLC( uiCode, "slice_type" );
    pcSlice->setSliceType( (SliceType)uiCode );
    if ( sps->getMaxDecPicBuffering( pcSlice->getTLayer() ) == 1 ) { assert( bTwoVersionsOfCurrDecPicFlag == 0 ); }

    if ( pps->getOutputFlagPresentFlag() ) {
      READ_FLAG( uiCode, "pic_output_flag" );
      pcSlice->setPicOutputFlag( uiCode ? true : false );
    } else {
      pcSlice->setPicOutputFlag( true );
    }

    // if (separate_colour_plane_flag == 1)
    //   read colour_plane_id
    //   (separate_colour_plane_flag == 1) is not supported in this version of the standard.

    if ( pcSlice->getIdrPicFlag() ) {
      pcSlice->setPOC( 0 );
      TComReferencePictureSet* rps = pcSlice->getLocalRPS();
      ( *rps )                     = TComReferencePictureSet();
      pcSlice->setRPS( rps );
    } else {
      READ_CODE( sps->getBitsForPOC(), uiCode, "slice_pic_order_cnt_lsb" );
      Int iPOClsb     = uiCode;
      Int iPrevPOC    = prevTid0POC;
      Int iMaxPOClsb  = 1 << sps->getBitsForPOC();
      Int iPrevPOClsb = iPrevPOC & ( iMaxPOClsb - 1 );
      Int iPrevPOCmsb = iPrevPOC - iPrevPOClsb;
      Int iPOCmsb;
      if ( ( iPOClsb < iPrevPOClsb ) && ( ( iPrevPOClsb - iPOClsb ) >= ( iMaxPOClsb / 2 ) ) ) {
        iPOCmsb = iPrevPOCmsb + iMaxPOClsb;
      } else if ( ( iPOClsb > iPrevPOClsb ) && ( ( iPOClsb - iPrevPOClsb ) > ( iMaxPOClsb / 2 ) ) ) {
        iPOCmsb = iPrevPOCmsb - iMaxPOClsb;
      } else {
        iPOCmsb = iPrevPOCmsb;
      }
      TComReferencePictureSet* rps1 = pcSlice->getLocalRPS();
      ( *rps1 )                     = TComReferencePictureSet();
      assert( rps1->getNumberOfPictures() + bTwoVersionsOfCurrDecPicFlag <=
              sps->getMaxDecPicBuffering( sps->getMaxTLayers() - 1 ) - 1 );

      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP ||
           pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL ||
           pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP ) {
        // For BLA picture types, POCmsb is set to 0.
        iPOCmsb = 0;
      }
      pcSlice->setPOC( iPOCmsb + iPOClsb );

      TComReferencePictureSet* rps;
      rps      = pcSlice->getLocalRPS();
      ( *rps ) = TComReferencePictureSet();

      pcSlice->setRPS( rps );
      READ_FLAG( uiCode, "short_term_ref_pic_set_sps_flag" );
      if ( uiCode == 0 )  // use short-term reference picture set explicitly signalled in slice header
      {
        parseShortTermRefPicSet( sps, rps, sps->getRPSList()->getNumberOfReferencePictureSets() );
      } else  // use reference to short-term reference picture set in PPS
      {
        Int numBits = 0;
        while ( ( 1 << numBits ) < sps->getRPSList()->getNumberOfReferencePictureSets() ) { numBits++; }
        if ( numBits > 0 ) {
          READ_CODE( numBits, uiCode, "short_term_ref_pic_set_idx" );
        } else {
          uiCode = 0;
        }
        *rps = *( sps->getRPSList()->getReferencePictureSet( uiCode ) );
      }
      if ( sps->getLongTermRefsPresent() ) {
        Int  offset       = rps->getNumberOfNegativePictures() + rps->getNumberOfPositivePictures();
        UInt numOfLtrp    = 0;
        UInt numLtrpInSPS = 0;
        if ( sps->getNumLongTermRefPicSPS() > 0 ) {
          READ_UVLC( uiCode, "num_long_term_sps" );
          numLtrpInSPS = uiCode;
          numOfLtrp += numLtrpInSPS;
          rps->setNumberOfLongtermPictures( numOfLtrp );
        }
        Int bitsForLtrpInSPS = 0;
        while ( sps->getNumLongTermRefPicSPS() > ( 1 << bitsForLtrpInSPS ) ) { bitsForLtrpInSPS++; }
        READ_UVLC( uiCode, "num_long_term_pics" );
        rps->setNumberOfLongtermPictures( uiCode );
        numOfLtrp += uiCode;
        rps->setNumberOfLongtermPictures( numOfLtrp );
        Int maxPicOrderCntLSB = 1 << sps->getBitsForPOC();
        Int prevDeltaMSB = 0, deltaPocMSBCycleLT = 0;
        for ( Int j = offset + rps->getNumberOfLongtermPictures() - 1, k = 0; k < numOfLtrp; j--, k++ ) {
          Int pocLsbLt;
          if ( k < numLtrpInSPS ) {
            uiCode = 0;
            if ( bitsForLtrpInSPS > 0 ) { READ_CODE( bitsForLtrpInSPS, uiCode, "lt_idx_sps[i]" ); }
            Bool usedByCurrFromSPS = sps->getUsedByCurrPicLtSPSFlag( uiCode );

            pocLsbLt = sps->getLtRefPicPocLsbSps( uiCode );
            rps->setUsed( j, usedByCurrFromSPS );
          } else {
            READ_CODE( sps->getBitsForPOC(), uiCode, "poc_lsb_lt" );
            pocLsbLt = uiCode;
            READ_FLAG( uiCode, "used_by_curr_pic_lt_flag" );
            rps->setUsed( j, uiCode );
          }
          READ_FLAG( uiCode, "delta_poc_msb_present_flag" );
          Bool mSBPresentFlag = uiCode ? true : false;
          if ( mSBPresentFlag ) {
            READ_UVLC( uiCode, "delta_poc_msb_cycle_lt[i]" );
            Bool deltaFlag = false;
            //            First LTRP                               || First LTRP from SH
            if ( ( j == offset + rps->getNumberOfLongtermPictures() - 1 ) ||
                 ( j == offset + ( numOfLtrp - numLtrpInSPS ) - 1 ) ) {
              deltaFlag = true;
            }
            if ( deltaFlag ) {
              deltaPocMSBCycleLT = uiCode;
            } else {
              deltaPocMSBCycleLT = uiCode + prevDeltaMSB;
            }

            Int pocLTCurr = pcSlice->getPOC() - deltaPocMSBCycleLT * maxPicOrderCntLSB - iPOClsb + pocLsbLt;
            rps->setPOC( j, pocLTCurr );
            rps->setDeltaPOC( j, -pcSlice->getPOC() + pocLTCurr );
            rps->setCheckLTMSBPresent( j, true );
          } else {
            rps->setPOC( j, pocLsbLt );
            rps->setDeltaPOC( j, -pcSlice->getPOC() + pocLsbLt );
            rps->setCheckLTMSBPresent( j, false );

            // reset deltaPocMSBCycleLT for first LTRP from slice header if MSB not present
            if ( j == offset + ( numOfLtrp - numLtrpInSPS ) - 1 ) { deltaPocMSBCycleLT = 0; }
          }
          prevDeltaMSB = deltaPocMSBCycleLT;
        }
        offset += rps->getNumberOfLongtermPictures();
        rps->setNumberOfPictures( offset );
      }
      if ( pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_LP ||
           pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_W_RADL ||
           pcSlice->getNalUnitType() == NAL_UNIT_CODED_SLICE_BLA_N_LP ) {
        // In the case of BLA picture types, rps data is read from slice header but ignored
        rps      = pcSlice->getLocalRPS();
        ( *rps ) = TComReferencePictureSet();
        pcSlice->setRPS( rps );
      }
      if ( sps->getSPSTemporalMVPEnabledFlag() ) {
        READ_FLAG( uiCode, "slice_temporal_mvp_enabled_flag" );
        pcSlice->setEnableTMVPFlag( uiCode == 1 ? true : false );
      } else {
        pcSlice->setEnableTMVPFlag( false );
      }
    }
    if ( sps->getUseSAO() ) {
      READ_FLAG( uiCode, "slice_sao_luma_flag" );
      pcSlice->setSaoEnabledFlag( CHANNEL_TYPE_LUMA, (Bool)uiCode );

      if ( bChroma ) {
        READ_FLAG( uiCode, "slice_sao_chroma_flag" );
        pcSlice->setSaoEnabledFlag( CHANNEL_TYPE_CHROMA, (Bool)uiCode );
      }
    }

    if ( pcSlice->getIdrPicFlag() ) { pcSlice->setEnableTMVPFlag( false ); }
    if ( !pcSlice->isIntra() ) {
      READ_FLAG( uiCode, "num_ref_idx_active_override_flag" );
      if ( uiCode ) {
        READ_UVLC( uiCode, "num_ref_idx_l0_active_minus1" );
        pcSlice->setNumRefIdx( REF_PIC_LIST_0, uiCode + 1 );
        if ( pcSlice->isInterB() ) {
          READ_UVLC( uiCode, "num_ref_idx_l1_active_minus1" );
          pcSlice->setNumRefIdx( REF_PIC_LIST_1, uiCode + 1 );
        } else {
          pcSlice->setNumRefIdx( REF_PIC_LIST_1, 0 );
        }
      } else {
        pcSlice->setNumRefIdx( REF_PIC_LIST_0, pps->getNumRefIdxL0DefaultActive() );
        if ( pcSlice->isInterB() ) {
          pcSlice->setNumRefIdx( REF_PIC_LIST_1, pps->getNumRefIdxL1DefaultActive() );
        } else {
          pcSlice->setNumRefIdx( REF_PIC_LIST_1, 0 );
        }
      }
    }
    // }
    pcSlice->setSPS( sps );
    pcSlice->setPPS( pps );
    TComRefPicListModification* refPicListModification = pcSlice->getRefPicListModification();
    if ( !pcSlice->isIntra() ) {
      if ( !pps->getListsModificationPresentFlag() || pcSlice->getNumRpsCurrTempList() <= 1 ) {
        refPicListModification->setRefPicListModificationFlagL0( 0 );
      } else {
        READ_FLAG( uiCode, "ref_pic_list_modification_flag_l0" );
        refPicListModification->setRefPicListModificationFlagL0( uiCode ? 1 : 0 );
      }

      if ( refPicListModification->getRefPicListModificationFlagL0() ) {
        uiCode                  = 0;
        Int i                   = 0;
        Int numRpsCurrTempList0 = pcSlice->getNumRpsCurrTempList();
        if ( numRpsCurrTempList0 > 1 ) {
          Int length = 1;
          numRpsCurrTempList0--;
          while ( numRpsCurrTempList0 >>= 1 ) { length++; }
          for ( i = 0; i < pcSlice->getNumRefIdx( REF_PIC_LIST_0 ); i++ ) {
            READ_CODE( length, uiCode, "list_entry_l0" );
            refPicListModification->setRefPicSetIdxL0( i, uiCode );
          }
        } else {
          for ( i = 0; i < pcSlice->getNumRefIdx( REF_PIC_LIST_0 ); i++ ) {
            refPicListModification->setRefPicSetIdxL0( i, 0 );
          }
        }
      }
    } else {
      refPicListModification->setRefPicListModificationFlagL0( 0 );
    }
    if ( pcSlice->isInterB() ) {
      if ( !pps->getListsModificationPresentFlag() || pcSlice->getNumRpsCurrTempList() <= 1 ) {
        refPicListModification->setRefPicListModificationFlagL1( 0 );
      } else {
        READ_FLAG( uiCode, "ref_pic_list_modification_flag_l1" );
        refPicListModification->setRefPicListModificationFlagL1( uiCode ? 1 : 0 );
      }
      if ( refPicListModification->getRefPicListModificationFlagL1() ) {
        uiCode                  = 0;
        Int i                   = 0;
        Int numRpsCurrTempList1 = pcSlice->getNumRpsCurrTempList();
        if ( numRpsCurrTempList1 > 1 ) {
          Int length = 1;
          numRpsCurrTempList1--;
          while ( numRpsCurrTempList1 >>= 1 ) { length++; }
          for ( i = 0; i < pcSlice->getNumRefIdx( REF_PIC_LIST_1 ); i++ ) {
            READ_CODE( length, uiCode, "list_entry_l1" );
            refPicListModification->setRefPicSetIdxL1( i, uiCode );
          }
        } else {
          for ( i = 0; i < pcSlice->getNumRefIdx( REF_PIC_LIST_1 ); i++ ) {
            refPicListModification->setRefPicSetIdxL1( i, 0 );
          }
        }
      }
    } else {
      refPicListModification->setRefPicListModificationFlagL1( 0 );
    }
    if ( pcSlice->isInterB() ) {
      READ_FLAG( uiCode, "mvd_l1_zero_flag" );
      pcSlice->setMvdL1ZeroFlag( ( uiCode ? true : false ) );
    }

    pcSlice->setCabacInitFlag( false );  // default
    if ( pps->getCabacInitPresentFlag() && !pcSlice->isIntra() ) {
      READ_FLAG( uiCode, "cabac_init_flag" );
      pcSlice->setCabacInitFlag( uiCode ? true : false );
    }

    if ( pcSlice->getEnableTMVPFlag() ) {
      if ( pcSlice->getSliceType() == B_SLICE ) {
        READ_FLAG( uiCode, "collocated_from_l0_flag" );
        pcSlice->setColFromL0Flag( uiCode );
      } else {
        pcSlice->setColFromL0Flag( 1 );
      }

      if ( pcSlice->getSliceType() != I_SLICE &&
           ( ( pcSlice->getColFromL0Flag() == 1 && pcSlice->getNumRefIdx( REF_PIC_LIST_0 ) > 1 ) ||
             ( pcSlice->getColFromL0Flag() == 0 && pcSlice->getNumRefIdx( REF_PIC_LIST_1 ) > 1 ) ) ) {
        READ_UVLC( uiCode, "collocated_ref_idx" );
        pcSlice->setColRefIdx( uiCode );
      } else {
        pcSlice->setColRefIdx( 0 );
      }
    }
    if ( ( pps->getUseWP() && pcSlice->getSliceType() == P_SLICE ) ||
         ( pps->getWPBiPred() && pcSlice->getSliceType() == B_SLICE ) ) {
      pcSlice->setRefPOCListSliceHeader();
      xParsePredWeightTable( pcSlice, sps );
      pcSlice->initWpScaling( sps );
    }
    if ( !pcSlice->isIntra() ) {
      READ_UVLC( uiCode, "five_minus_max_num_merge_cand" );
      pcSlice->setMaxNumMergeCand( MRG_MAX_NUM_CANDS - uiCode );

      if ( sps->getSpsScreenExtension().getMotionVectorResolutionControlIdc() == 2 ) {
        READ_FLAG( uiCode, "use_integer_mv_flag" );
        pcSlice->setUseIntegerMv( uiCode != 0 );
      } else {
        pcSlice->setUseIntegerMv( sps->getSpsScreenExtension().getMotionVectorResolutionControlIdc() == 0 ? false
                                                                                                          : true );
      }
    }

    READ_SVLC( iCode, "slice_qp_delta" );
    pcSlice->setSliceQp( 26 + pps->getPicInitQPMinus26() + iCode );

    assert( pcSlice->getSliceQp() >= -sps->getQpBDOffset( CHANNEL_TYPE_LUMA ) );
    assert( pcSlice->getSliceQp() <= 51 );

    if ( pps->getSliceChromaQpFlag() ) {
      if ( numValidComp > COMPONENT_Cb ) {
        READ_SVLC( iCode, "slice_cb_qp_offset" );
        pcSlice->setSliceChromaQpDelta( COMPONENT_Cb, iCode );
        assert( pcSlice->getSliceChromaQpDelta( COMPONENT_Cb ) >= -12 );
        assert( pcSlice->getSliceChromaQpDelta( COMPONENT_Cb ) <= 12 );
        assert( ( pps->getQpOffset( COMPONENT_Cb ) + pcSlice->getSliceChromaQpDelta( COMPONENT_Cb ) ) >= -12 );
        assert( ( pps->getQpOffset( COMPONENT_Cb ) + pcSlice->getSliceChromaQpDelta( COMPONENT_Cb ) ) <= 12 );
      }

      if ( numValidComp > COMPONENT_Cr ) {
        READ_SVLC( iCode, "slice_cr_qp_offset" );
        pcSlice->setSliceChromaQpDelta( COMPONENT_Cr, iCode );
        assert( pcSlice->getSliceChromaQpDelta( COMPONENT_Cr ) >= -12 );
        assert( pcSlice->getSliceChromaQpDelta( COMPONENT_Cr ) <= 12 );
        assert( ( pps->getQpOffset( COMPONENT_Cr ) + pcSlice->getSliceChromaQpDelta( COMPONENT_Cr ) ) >= -12 );
        assert( ( pps->getQpOffset( COMPONENT_Cr ) + pcSlice->getSliceChromaQpDelta( COMPONENT_Cr ) ) <= 12 );
      }
    }

    if ( pps->getPpsScreenExtension().getUseSliceACTOffset() ) {
      READ_SVLC( iCode, "slice_act_y_qp_offset" );
      pcSlice->setSliceActQpDelta( COMPONENT_Y, iCode );
      assert( pcSlice->getSliceActQpDelta( COMPONENT_Y ) >= -12 );
      assert( pcSlice->getSliceActQpDelta( COMPONENT_Y ) <= 12 );
      assert( ( pps->getPpsScreenExtension().getActQpOffset( COMPONENT_Y ) +
                pcSlice->getSliceActQpDelta( COMPONENT_Y ) ) >= -12 );
      assert( ( pps->getPpsScreenExtension().getActQpOffset( COMPONENT_Y ) +
                pcSlice->getSliceActQpDelta( COMPONENT_Y ) ) <= 12 );

      READ_SVLC( iCode, "slice_act_cb_qp_offset" );
      pcSlice->setSliceActQpDelta( COMPONENT_Cb, iCode );
      assert( pcSlice->getSliceActQpDelta( COMPONENT_Cb ) >= -12 );
      assert( pcSlice->getSliceActQpDelta( COMPONENT_Cb ) <= 12 );
      assert( ( pps->getPpsScreenExtension().getActQpOffset( COMPONENT_Cb ) +
                pcSlice->getSliceActQpDelta( COMPONENT_Cb ) ) >= -12 );
      assert( ( pps->getPpsScreenExtension().getActQpOffset( COMPONENT_Cb ) +
                pcSlice->getSliceActQpDelta( COMPONENT_Cb ) ) <= 12 );

      READ_SVLC( iCode, "slice_act_cr_qp_offset" );
      pcSlice->setSliceActQpDelta( COMPONENT_Cr, iCode );
      assert( pcSlice->getSliceActQpDelta( COMPONENT_Cr ) >= -12 );
      assert( pcSlice->getSliceActQpDelta( COMPONENT_Cr ) <= 12 );
      assert( ( pps->getPpsScreenExtension().getActQpOffset( COMPONENT_Cr ) +
                pcSlice->getSliceActQpDelta( COMPONENT_Cr ) ) >= -12 );
      assert( ( pps->getPpsScreenExtension().getActQpOffset( COMPONENT_Cr ) +
                pcSlice->getSliceActQpDelta( COMPONENT_Cr ) ) <= 12 );
    } else {
      iCode = 0;

      pcSlice->setSliceActQpDelta( COMPONENT_Y, iCode );
      assert( pcSlice->getSliceActQpDelta( COMPONENT_Y ) >= -12 );
      assert( pcSlice->getSliceActQpDelta( COMPONENT_Y ) <= 12 );
      assert( ( pps->getPpsScreenExtension().getActQpOffset( COMPONENT_Y ) +
                pcSlice->getSliceActQpDelta( COMPONENT_Y ) ) >= -12 );
      assert( ( pps->getPpsScreenExtension().getActQpOffset( COMPONENT_Y ) +
                pcSlice->getSliceActQpDelta( COMPONENT_Y ) ) <= 12 );

      pcSlice->setSliceActQpDelta( COMPONENT_Cb, iCode );
      assert( pcSlice->getSliceActQpDelta( COMPONENT_Cb ) >= -12 );
      assert( pcSlice->getSliceActQpDelta( COMPONENT_Cb ) <= 12 );
      assert( ( pps->getPpsScreenExtension().getActQpOffset( COMPONENT_Cb ) +
                pcSlice->getSliceActQpDelta( COMPONENT_Cb ) ) >= -12 );
      assert( ( pps->getPpsScreenExtension().getActQpOffset( COMPONENT_Cb ) +
                pcSlice->getSliceActQpDelta( COMPONENT_Cb ) ) <= 12 );

      pcSlice->setSliceActQpDelta( COMPONENT_Cr, iCode );
      assert( pcSlice->getSliceActQpDelta( COMPONENT_Cr ) >= -12 );
      assert( pcSlice->getSliceActQpDelta( COMPONENT_Cr ) <= 12 );
      assert( ( pps->getPpsScreenExtension().getActQpOffset( COMPONENT_Cr ) +
                pcSlice->getSliceActQpDelta( COMPONENT_Cr ) ) >= -12 );
      assert( ( pps->getPpsScreenExtension().getActQpOffset( COMPONENT_Cr ) +
                pcSlice->getSliceActQpDelta( COMPONENT_Cr ) ) <= 12 );
    }

    if ( pps->getPpsRangeExtension().getChromaQpOffsetListEnabledFlag() ) {
      READ_FLAG( uiCode, "cu_chroma_qp_offset_enabled_flag" );
      pcSlice->setUseChromaQpAdj( uiCode != 0 );
    } else {
      pcSlice->setUseChromaQpAdj( false );
    }

    if ( pps->getDeblockingFilterControlPresentFlag() ) {
      if ( pps->getDeblockingFilterOverrideEnabledFlag() ) {
        READ_FLAG( uiCode, "deblocking_filter_override_flag" );
        pcSlice->setDeblockingFilterOverrideFlag( uiCode ? true : false );
      } else {
        pcSlice->setDeblockingFilterOverrideFlag( 0 );
      }
      if ( pcSlice->getDeblockingFilterOverrideFlag() ) {
        READ_FLAG( uiCode, "slice_deblocking_filter_disabled_flag" );
        pcSlice->setDeblockingFilterDisable( uiCode ? 1 : 0 );
        if ( !pcSlice->getDeblockingFilterDisable() ) {
          READ_SVLC( iCode, "slice_beta_offset_div2" );
          pcSlice->setDeblockingFilterBetaOffsetDiv2( iCode );
          assert( pcSlice->getDeblockingFilterBetaOffsetDiv2() >= -6 &&
                  pcSlice->getDeblockingFilterBetaOffsetDiv2() <= 6 );
          READ_SVLC( iCode, "slice_tc_offset_div2" );
          pcSlice->setDeblockingFilterTcOffsetDiv2( iCode );
          assert( pcSlice->getDeblockingFilterTcOffsetDiv2() >= -6 && pcSlice->getDeblockingFilterTcOffsetDiv2() <= 6 );
        }
      } else {
        pcSlice->setDeblockingFilterDisable( pps->getPPSDeblockingFilterDisabledFlag() );
        pcSlice->setDeblockingFilterBetaOffsetDiv2( pps->getDeblockingFilterBetaOffsetDiv2() );
        pcSlice->setDeblockingFilterTcOffsetDiv2( pps->getDeblockingFilterTcOffsetDiv2() );
      }
    } else {
      pcSlice->setDeblockingFilterDisable( false );
      pcSlice->setDeblockingFilterBetaOffsetDiv2( 0 );
      pcSlice->setDeblockingFilterTcOffsetDiv2( 0 );
    }

    Bool isSAOEnabled = sps->getUseSAO() && ( pcSlice->getSaoEnabledFlag( CHANNEL_TYPE_LUMA ) ||
                                              ( bChroma && pcSlice->getSaoEnabledFlag( CHANNEL_TYPE_CHROMA ) ) );
    Bool isDBFEnabled = ( !pcSlice->getDeblockingFilterDisable() );

    if ( pps->getLoopFilterAcrossSlicesEnabledFlag() && ( isSAOEnabled || isDBFEnabled ) ) {
      READ_FLAG( uiCode, "slice_loop_filter_across_slices_enabled_flag" );
    } else {
      uiCode = pps->getLoopFilterAcrossSlicesEnabledFlag() ? 1 : 0;
    }
    pcSlice->setLFCrossSliceBoundaryFlag( ( uiCode == 1 ) ? true : false );
  }

  std::vector<UInt> entryPointOffset;
  if ( pps->getTilesEnabledFlag() || pps->getEntropyCodingSyncEnabledFlag() ) {
    UInt numEntryPointOffsets;
    UInt offsetLenMinus1;
    READ_UVLC( numEntryPointOffsets, "num_entry_point_offsets" );
    if ( numEntryPointOffsets > 0 ) {
      READ_UVLC( offsetLenMinus1, "offset_len_minus1" );
      entryPointOffset.resize( numEntryPointOffsets );
      for ( UInt idx = 0; idx < numEntryPointOffsets; idx++ ) {
        READ_CODE( offsetLenMinus1 + 1, uiCode, "entry_point_offset_minus1" );
        entryPointOffset[idx] = uiCode + 1;
      }
    }
  }

  if ( pps->getSliceHeaderExtensionPresentFlag() ) {
    READ_UVLC( uiCode, "slice_segment_header_extension_length" );
    for ( Int i = 0; i < uiCode; i++ ) {
      UInt ignore;
      READ_CODE( 8, ignore, "slice_segment_header_extension_data_byte" );
    }
  }
  m_pcBitstream.align();
  

//#if RExt__DECODER_DEBUG_BIT_STATISTICS
//  TComCodingStatistics::IncrementStatisticEP(STATS__BYTE_ALIGNMENT_BITS,m_pcBitstream->readByteAlignment(),0);
//#else
//  m_pcBitstream->readByteAlignment();
//#endif

//  pcSlice->clearSubstreamSizes();

//  if( pps->getTilesEnabledFlag() || pps->getEntropyCodingSyncEnabledFlag() )
//  {
//    Int endOfSliceHeaderLocation = m_pcBitstream->getByteLocation();

    // Adjust endOfSliceHeaderLocation to account for emulation prevention bytes in the slice segment header
//    for ( UInt curByteIdx  = 0; curByteIdx<m_pcBitstream->numEmulationPreventionBytesRead(); curByteIdx++ )
//    {
//      if ( m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) < endOfSliceHeaderLocation )
//      {
//        endOfSliceHeaderLocation++;
//      }
//    }

//    Int  curEntryPointOffset     = 0;
//    Int  prevEntryPointOffset    = 0;
//    for (UInt idx=0; idx<entryPointOffset.size(); idx++)
//    {
//      curEntryPointOffset += entryPointOffset[ idx ];

//      Int emulationPreventionByteCount = 0;
//      for ( UInt curByteIdx  = 0; curByteIdx<m_pcBitstream->numEmulationPreventionBytesRead(); curByteIdx++ )
//      {
//        if ( m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) >= ( prevEntryPointOffset +
//endOfSliceHeaderLocation ) && m_pcBitstream->getEmulationPreventionByteLocation( curByteIdx ) <  ( curEntryPointOffset
//+ endOfSliceHeaderLocation ) )
//       {
//          emulationPreventionByteCount++;
//        }
//      }

//      entryPointOffset[ idx ] -= emulationPreventionByteCount;
//      prevEntryPointOffset = curEntryPointOffset;
//      pcSlice->addSubstreamSize(entryPointOffset [ idx ] );
//    }
//  }

  return;
}
*/
/*
Void TDecCavlc::parsePTL( TComPTL* rpcPTL, Bool profilePresentFlag, Int maxNumSubLayersMinus1 ) {
  UInt uiCode;
  if ( profilePresentFlag ) { parseProfileTier( rpcPTL->getGeneralPTL(), false ); }
  READ_CODE( 8, uiCode, "general_level_idc" );
  rpcPTL->getGeneralPTL()->setLevelIdc( Level::Name( uiCode ) );

  for ( Int i = 0; i < maxNumSubLayersMinus1; i++ ) {
    READ_FLAG( uiCode, "sub_layer_profile_present_flag[i]" );
    rpcPTL->setSubLayerProfilePresentFlag( i, uiCode );
    READ_FLAG( uiCode, "sub_layer_level_present_flag[i]" );
    rpcPTL->setSubLayerLevelPresentFlag( i, uiCode );
  }

  if ( maxNumSubLayersMinus1 > 0 ) {
    for ( Int i = maxNumSubLayersMinus1; i < 8; i++ ) {
      READ_CODE( 2, uiCode, "reserved_zero_2bits" );
      assert( uiCode == 0 );
    }
  }

  for ( Int i = 0; i < maxNumSubLayersMinus1; i++ ) {
    if ( rpcPTL->getSubLayerProfilePresentFlag( i ) ) { parseProfileTier( rpcPTL->getSubLayerPTL( i ), true ); }
    if ( rpcPTL->getSubLayerLevelPresentFlag( i ) ) {
      READ_CODE( 8, uiCode, "sub_layer_level_idc[i]" );
      rpcPTL->getSubLayerPTL( i )->setLevelIdc( Level::Name( uiCode ) );
    }
  }
}
*/
/*
#if ENC_DEC_TRACE || RExt__DECODER_DEBUG_BIT_STATISTICS
Void TDecCavlc::parseProfileTier( ProfileTierLevel* ptl, const Bool bIsSubLayer )
#define PTL_TRACE_TEXT( txt ) bIsSubLayer ? ( "sub_layer_" txt ) : ( "general_" txt )
#else
Void TDecCavlc::parseProfileTier( ProfileTierLevel* ptl, const Bool
//bIsSubLayer
)
#define PTL_TRACE_TEXT( txt ) txt
#endif
{
  UInt uiCode;
  READ_CODE( 2, uiCode, PTL_TRACE_TEXT( "profile_space" ) );
  ptl->setProfileSpace( uiCode );
  READ_FLAG( uiCode, PTL_TRACE_TEXT( "tier_flag" ) );
  ptl->setTierFlag( uiCode ? Level::HIGH : Level::MAIN );
  READ_CODE( 5, uiCode, PTL_TRACE_TEXT( "profile_idc" ) );
  ptl->setProfileIdc( Profile::Name( uiCode ) );
  for ( Int j = 0; j < 32; j++ ) {
    READ_FLAG( uiCode, PTL_TRACE_TEXT( "profile_compatibility_flag[][j]" ) );
    ptl->setProfileCompatibilityFlag( j, uiCode ? 1 : 0 );
  }
  READ_FLAG( uiCode, PTL_TRACE_TEXT( "progressive_source_flag" ) );
  ptl->setProgressiveSourceFlag( uiCode ? true : false );

  READ_FLAG( uiCode, PTL_TRACE_TEXT( "interlaced_source_flag" ) );
  ptl->setInterlacedSourceFlag( uiCode ? true : false );

  READ_FLAG( uiCode, PTL_TRACE_TEXT( "non_packed_constraint_flag" ) );
  ptl->setNonPackedConstraintFlag( uiCode ? true : false );

  READ_FLAG( uiCode, PTL_TRACE_TEXT( "frame_only_constraint_flag" ) );
  ptl->setFrameOnlyConstraintFlag( uiCode ? true : false );

  if ( ptl->getProfileIdc() == Profile::MAINREXT || ptl->getProfileCompatibilityFlag( Profile::MAINREXT ) ||
       ptl->getProfileIdc() == Profile::HIGHTHROUGHPUTREXT ||
       ptl->getProfileCompatibilityFlag( Profile::HIGHTHROUGHPUTREXT ) || ptl->getProfileIdc() == Profile::MAINSCC ||
       ptl->getProfileCompatibilityFlag( Profile::MAINSCC ) ) {
    UInt maxBitDepth = 16;
    READ_FLAG( uiCode, PTL_TRACE_TEXT( "max_12bit_constraint_flag" ) );
    if ( uiCode ) maxBitDepth = 12;
    READ_FLAG( uiCode, PTL_TRACE_TEXT( "max_10bit_constraint_flag" ) );
    if ( uiCode ) maxBitDepth = 10;
    READ_FLAG( uiCode, PTL_TRACE_TEXT( "max_8bit_constraint_flag" ) );
    if ( uiCode ) maxBitDepth = 8;
    ptl->setBitDepthConstraint( maxBitDepth );
    ChromaFormat chromaFmtConstraint = CHROMA_444;
    READ_FLAG( uiCode, PTL_TRACE_TEXT( "max_422chroma_constraint_flag" ) );
    if ( uiCode ) chromaFmtConstraint = CHROMA_422;
    READ_FLAG( uiCode, PTL_TRACE_TEXT( "max_420chroma_constraint_flag" ) );
    if ( uiCode ) chromaFmtConstraint = CHROMA_420;
    READ_FLAG( uiCode, PTL_TRACE_TEXT( "max_monochrome_constraint_flag" ) );
    if ( uiCode ) chromaFmtConstraint = CHROMA_400;
    ptl->setChromaFormatConstraint( chromaFmtConstraint );
    READ_FLAG( uiCode, PTL_TRACE_TEXT( "intra_constraint_flag" ) );
    ptl->setIntraConstraintFlag( uiCode != 0 );
    READ_FLAG( uiCode, PTL_TRACE_TEXT( "one_picture_only_constraint_flag" ) );
    ptl->setOnePictureOnlyConstraintFlag( uiCode != 0 );
    READ_FLAG( uiCode, PTL_TRACE_TEXT( "lower_bit_rate_constraint_flag" ) );
    ptl->setLowerBitRateConstraintFlag( uiCode != 0 );
    READ_CODE( 16, uiCode, PTL_TRACE_TEXT( "reserved_zero_34bits[0..15]" ) );
    READ_CODE( 16, uiCode, PTL_TRACE_TEXT( "reserved_zero_34bits[16..31]" ) );
    READ_CODE( 2, uiCode, PTL_TRACE_TEXT( "reserved_zero_34bits[32..33]" ) );
  } else {
    ptl->setBitDepthConstraint( ( ptl->getProfileIdc() == Profile::MAIN10 ) ? 10 : 8 );
    ptl->setChromaFormatConstraint( CHROMA_420 );
    ptl->setIntraConstraintFlag( false );
    ptl->setLowerBitRateConstraintFlag( true );
    if ( ptl->getProfileIdc() == Profile::MAIN10 || ptl->getProfileCompatibilityFlag( Profile::MAIN10 ) ) {
      READ_CODE( 7, uiCode, PTL_TRACE_TEXT( "reserved_zero_7bits" ) );
      READ_FLAG( uiCode, PTL_TRACE_TEXT( "one_picture_only_constraint_flag" ) );
      ptl->setOnePictureOnlyConstraintFlag( uiCode != 0 );
      READ_CODE( 16, uiCode, PTL_TRACE_TEXT( "reserved_zero_35bits[0..15]" ) );
      READ_CODE( 16, uiCode, PTL_TRACE_TEXT( "reserved_zero_35bits[16..31]" ) );
      READ_CODE( 3, uiCode, PTL_TRACE_TEXT( "reserved_zero_35bits[32..34]" ) );
    } else {
      READ_CODE( 16, uiCode, PTL_TRACE_TEXT( "reserved_zero_43bits[0..15]" ) );
      READ_CODE( 16, uiCode, PTL_TRACE_TEXT( "reserved_zero_43bits[16..31]" ) );
      READ_CODE( 11, uiCode, PTL_TRACE_TEXT( "reserved_zero_43bits[32..42]" ) );
    }
  }

  if ( ( ptl->getProfileIdc() >= Profile::MAIN && ptl->getProfileIdc() <= Profile::HIGHTHROUGHPUTREXT ) ||
       ptl->getProfileIdc() == Profile::MAINSCC || ptl->getProfileCompatibilityFlag( Profile::MAIN ) ||
       ptl->getProfileCompatibilityFlag( Profile::MAIN10 ) ||
       ptl->getProfileCompatibilityFlag( Profile::MAINSTILLPICTURE ) ||
       ptl->getProfileCompatibilityFlag( Profile::MAINREXT ) ||
       ptl->getProfileCompatibilityFlag( Profile::HIGHTHROUGHPUTREXT ) ||
       ptl->getProfileCompatibilityFlag( Profile::MAINSCC ) ) {
    READ_FLAG( uiCode, PTL_TRACE_TEXT( "inbld_flag" ) );
    assert( uiCode == 0 );
  } else {
    READ_FLAG( uiCode, PTL_TRACE_TEXT( "reserved_zero_bit" ) );
  }
#undef PTL_TRACE_TEXT
}
*/
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

// /* Parse I_PCM information.
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

// Void TDecCavlc::parseMvd( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiPartIdx*/, UInt /*uiDepth*/,
// RefPicList /*eRefList*/ )
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
//   const Int qp = (((Int) pcCU->getRefQP( uiAbsPartIdx ) + iDQp + 52 + 2*qpBdOffsetY )%(52+ qpBdOffsetY)) -
//   qpBdOffsetY;

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
/*
//! parse explicit wp tables
Void TDecCavlc::xParsePredWeightTable( TComSlice* pcSlice, const TComSPS* sps ) {
  WPScalingParam*    wp;
  const ChromaFormat chFmt                 = sps->getChromaFormatIdc();
  const Int          numValidComp          = Int( getNumberValidComponents( chFmt ) );
  const Bool         bChroma               = ( chFmt != CHROMA_400 );
  const SliceType    eSliceType            = pcSlice->getSliceType();
  const Int          iNbRef                = ( eSliceType == B_SLICE ) ? ( 2 ) : ( 1 );
  UInt               uiLog2WeightDenomLuma = 0, uiLog2WeightDenomChroma = 0;
  UInt               uiTotalSignalledWeightFlags = 0;

  Int iDeltaDenom;
  // decode delta_luma_log2_weight_denom :
  READ_UVLC( uiLog2WeightDenomLuma, "luma_log2_weight_denom" );
  assert( uiLog2WeightDenomLuma <= 7 );
  if ( bChroma ) {
    READ_SVLC( iDeltaDenom, "delta_chroma_log2_weight_denom" );
    assert( ( iDeltaDenom + (Int)uiLog2WeightDenomLuma ) >= 0 );
    assert( ( iDeltaDenom + (Int)uiLog2WeightDenomLuma ) <= 7 );
    uiLog2WeightDenomChroma = ( UInt )( iDeltaDenom + uiLog2WeightDenomLuma );
  }

  for ( Int iNumRef = 0; iNumRef < iNbRef; iNumRef++ )  // loop over l0 and l1 syntax elements
  {
    RefPicList eRefPicList = ( iNumRef ? REF_PIC_LIST_1 : REF_PIC_LIST_0 );
    for ( Int iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx( eRefPicList ); iRefIdx++ ) {
      pcSlice->getWpScaling( eRefPicList, iRefIdx, wp );

      wp[COMPONENT_Y].uiLog2WeightDenom = uiLog2WeightDenomLuma;
      for ( Int j = 1; j < numValidComp; j++ ) { wp[j].uiLog2WeightDenom = uiLog2WeightDenomChroma; }

      UInt uiCode;
      if ( pcSlice->getRefPOC( eRefPicList, iRefIdx ) == pcSlice->getPOC() ) {
        uiCode = 0;
      } else {
        READ_FLAG( uiCode, iNumRef == 0 ? "luma_weight_l0_flag[i]" : "luma_weight_l1_flag[i]" );
      }
      wp[COMPONENT_Y].bPresentFlag = ( uiCode == 1 );
      uiTotalSignalledWeightFlags += wp[COMPONENT_Y].bPresentFlag;
    }
    if ( bChroma ) {
      UInt uiCode;
      for ( Int iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx( eRefPicList ); iRefIdx++ ) {
        pcSlice->getWpScaling( eRefPicList, iRefIdx, wp );
        if ( pcSlice->getRefPOC( eRefPicList, iRefIdx ) == pcSlice->getPOC() ) {
          uiCode = 0;
        } else {
          READ_FLAG( uiCode, iNumRef == 0 ? "chroma_weight_l0_flag[i]" : "chroma_weight_l1_flag[i]" );
        }
        for ( Int j = 1; j < numValidComp; j++ ) { wp[j].bPresentFlag = ( uiCode == 1 ); }
        uiTotalSignalledWeightFlags += 2 * wp[COMPONENT_Cb].bPresentFlag;
      }
    }
    for ( Int iRefIdx = 0; iRefIdx < pcSlice->getNumRefIdx( eRefPicList ); iRefIdx++ ) {
      pcSlice->getWpScaling( eRefPicList, iRefIdx, wp );
      if ( wp[COMPONENT_Y].bPresentFlag ) {
        Int iDeltaWeight;
        READ_SVLC( iDeltaWeight, iNumRef == 0 ? "delta_luma_weight_l0[i]" : "delta_luma_weight_l1[i]" );
        assert( iDeltaWeight >= -128 );
        assert( iDeltaWeight <= 127 );
        wp[COMPONENT_Y].iWeight = ( iDeltaWeight + ( 1 << wp[COMPONENT_Y].uiLog2WeightDenom ) );
        READ_SVLC( wp[COMPONENT_Y].iOffset, iNumRef == 0 ? "luma_offset_l0[i]" : "luma_offset_l1[i]" );
        Int range = sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag()
                        ? ( 1 << sps->getBitDepth( CHANNEL_TYPE_LUMA ) ) / 2
                        : 128;
        assert( wp[0].iOffset >= -range );
        assert( wp[0].iOffset < range );
      } else {
        wp[COMPONENT_Y].iWeight = ( 1 << wp[COMPONENT_Y].uiLog2WeightDenom );
        wp[COMPONENT_Y].iOffset = 0;
      }
      if ( bChroma ) {
        if ( wp[COMPONENT_Cb].bPresentFlag ) {
          Int range = sps->getSpsRangeExtension().getHighPrecisionOffsetsEnabledFlag()
                          ? ( 1 << sps->getBitDepth( CHANNEL_TYPE_CHROMA ) ) / 2
                          : 128;
          for ( Int j = 1; j < numValidComp; j++ ) {
            Int iDeltaWeight;
            READ_SVLC( iDeltaWeight, iNumRef == 0 ? "delta_chroma_weight_l0[i]" : "delta_chroma_weight_l1[i]" );
            assert( iDeltaWeight >= -128 );
            assert( iDeltaWeight <= 127 );
            wp[j].iWeight = ( iDeltaWeight + ( 1 << wp[j].uiLog2WeightDenom ) );

            Int iDeltaChroma;
            READ_SVLC( iDeltaChroma, iNumRef == 0 ? "delta_chroma_offset_l0[i]" : "delta_chroma_offset_l1[i]" );
            assert( iDeltaChroma >= -4 * range );
            assert( iDeltaChroma < 4 * range );
            Int pred      = ( range - ( ( range * wp[j].iWeight ) >> ( wp[j].uiLog2WeightDenom ) ) );
            wp[j].iOffset = Clip3( -range, range - 1, ( iDeltaChroma + pred ) );
          }
        } else {
          for ( Int j = 1; j < numValidComp; j++ ) {
            wp[j].iWeight = ( 1 << wp[j].uiLog2WeightDenom );
            wp[j].iOffset = 0;
          }
        }
      }
    }

    for ( Int iRefIdx = pcSlice->getNumRefIdx( eRefPicList ); iRefIdx < MAX_NUM_REF; iRefIdx++ ) {
      pcSlice->getWpScaling( eRefPicList, iRefIdx, wp );

      wp[0].bPresentFlag = false;
      wp[1].bPresentFlag = false;
      wp[2].bPresentFlag = false;
    }
  }
  assert( uiTotalSignalledWeightFlags <= 24 );
}
*/
/** decode quantization matrix
 * \param scalingList quantization matrix information
 */
/*
Void TDecCavlc::parseScalingList( TComScalingList* scalingList ) {
  UInt code, sizeId, listId;
  Bool scalingListPredModeFlag;
  // for each size
  for ( sizeId = 0; sizeId < SCALING_LIST_SIZE_NUM; sizeId++ ) {
    for ( listId = 0; listId < SCALING_LIST_NUM; listId++ ) {
      if ( ( sizeId == SCALING_LIST_32x32 ) && ( listId % ( SCALING_LIST_NUM / NUMBER_OF_PREDICTION_MODES ) != 0 ) ) {
        Int*       src                = scalingList->getScalingListAddress( sizeId, listId );
        const Int  size               = min( MAX_MATRIX_COEF_NUM, (Int)g_scalingListSize[sizeId] );
        const Int* srcNextSmallerSize = scalingList->getScalingListAddress( sizeId - 1, listId );
        for ( Int i = 0; i < size; i++ ) { src[i] = srcNextSmallerSize[i]; }
        scalingList->setScalingListDC(
            sizeId, listId,
            ( sizeId > SCALING_LIST_8x8 ) ? scalingList->getScalingListDC( sizeId - 1, listId ) : src[0] );
      } else {
        READ_FLAG( code, "scaling_list_pred_mode_flag" );
        scalingListPredModeFlag = ( code ) ? true : false;
        scalingList->setScalingListPredModeFlag( sizeId, listId, scalingListPredModeFlag );
        if ( !scalingListPredModeFlag )  // Copy Mode
        {
          READ_UVLC( code, "scaling_list_pred_matrix_id_delta" );

          if ( sizeId == SCALING_LIST_32x32 ) {
            code *= ( SCALING_LIST_NUM / NUMBER_OF_PREDICTION_MODES );  // Adjust the decoded code for this size, to
                                                                        // cope with the missing 32x32 chroma entries.
          }

          scalingList->setRefMatrixId( sizeId, listId, ( UInt )( ( Int )( listId ) - ( code ) ) );
          if ( sizeId > SCALING_LIST_8x8 ) {
            scalingList->setScalingListDC(
                sizeId, listId,
                ( ( listId == scalingList->getRefMatrixId( sizeId, listId ) )
                      ? 16
                      : scalingList->getScalingListDC( sizeId, scalingList->getRefMatrixId( sizeId, listId ) ) ) );
          }
          scalingList->processRefMatrix( sizeId, listId, scalingList->getRefMatrixId( sizeId, listId ) );

        } else  // DPCM Mode
        {
          xDecodeScalingList( scalingList, sizeId, listId );
        }
      }
    }
  }

  return;
}
*/
/** decode DPCM
 * \param scalingList  quantization matrix information
 * \param sizeId size index
 * \param listId list index
 */
/*
Void TDecCavlc::xDecodeScalingList( TComScalingList* scalingList, UInt sizeId, UInt listId ) {
  Int   i, coefNum = min( MAX_MATRIX_COEF_NUM, (Int)g_scalingListSize[sizeId] );
  Int   data;
  Int   scalingListDcCoefMinus8 = 0;
  Int   nextCoef                = SCALING_LIST_START_VALUE;
  UInt* scan                    = g_scanOrder[SCAN_UNGROUPED][SCAN_DIAG][sizeId == 0 ? 2 : 3][sizeId == 0 ? 2 : 3];
  Int*  dst                     = scalingList->getScalingListAddress( sizeId, listId );

  if ( sizeId > SCALING_LIST_8x8 ) {
    READ_SVLC( scalingListDcCoefMinus8, "scaling_list_dc_coef_minus8" );
    scalingList->setScalingListDC( sizeId, listId, scalingListDcCoefMinus8 + 8 );
    nextCoef = scalingList->getScalingListDC( sizeId, listId );
  }

  for ( i = 0; i < coefNum; i++ ) {
    READ_SVLC( data, "scaling_list_delta_coef" );
    nextCoef     = ( nextCoef + data + 256 ) % 256;
    dst[scan[i]] = nextCoef;
  }
}
*/
/*
Bool TDecCavlc::xMoreRbspData() {
  return true;
  // Int bitsLeft = m_pcBitstream->getNumBitsLeft();

  // // if there are more than 8 bits, it cannot be Rbsp_trailing_bits
  // if (bitsLeft > 8)
  // {
  //   return true;
  // }

  // UChar lastByte = m_pcBitstream->peekBits(bitsLeft);
  // Int cnt = bitsLeft;

  // // remove trailing bits equal to zero
  // while ((cnt>0) && ((lastByte & 1) == 0))
  // {
  //   lastByte >>= 1;
  //   cnt--;
  // }
  // // remove bit equal to one
  // cnt--;

  // // we should not have a negative number of bits
  // assert (cnt>=0);

  // // we have more data, if cnt is not zero
  // return (cnt>0);
}
*/

// Void TDecCavlc::parseExplicitRdpcmMode( TComTU& /*rTu*/, ComponentID /*compID*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parsePaletteModeSyntax( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/, UInt
// /*uiNumComp*/, Bool& /*bCodeDQP*/, Bool& /*codeChromaQpAdj*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parsePaletteModeFlag( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
// {
//   assert(0);
// }

// Void  TDecCavlc::parseColourTransformFlag( Bool& /*flag*/ )
// {
//   assert(0);
// }

// Void TDecCavlc::parseScanRotationModeFlag( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
// {
//   assert(0);
// }
// Void TDecCavlc::parseScanTraverseModeFlag( TComDataCU* /*pcCU*/, UInt /*uiAbsPartIdx*/, UInt /*uiDepth*/ )
// {
//   assert(0);
// }

//! \}
