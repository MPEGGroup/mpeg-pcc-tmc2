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

/** \file     TypeDef.h
    \brief    Define basic types, new types and enumerations
*/

#ifndef _TYPEDEF__
#define _TYPEDEF__

#define TRACE_DEBUG                      0    // TCH TRACE

#define SVC_EXTENSION                    1

#if SVC_EXTENSION
#define MAX_LAYERS                       8      ///< max number of layers the codec is supposed to handle

#define SIGNALLING_BITRATE_PICRATE_FIX   1      ///< Fix for signalling of bitrate and picture rate info in VPS VUI to be more aligned to JCTVC-R1008
#define INFERENCE_POC_MSB_VAL_PRESENT    1      ///< JCTVC-Q0146 -- poc_msb_val_present_flag shall be equal to 0 when slice_header_extension_length is (inferred to be ) equal to 0
#define CROSS_LAYER_BLA_FLAG_FIX         1      ///< Fix for earlier implementation mistake that omit the signalling of cross_layer_bla_flag
#define NON_REF_NAL_TYPE_DISCARDABLE     1      ///< JCTVC-P0041 -- If discardable picture is a non-IRAP, it must be a non-referenced sub-layer picture
#define VPS_AVC_BL_FLAG_REMOVAL          1      ///< remove avc_base_layer_flag and direct the function its offer to vps_base_layer_internal_flag and vps_base_layer_available_flag
#define VPS_RESERVED_FLAGS               1      ///< vps_base_layer_internal_flag and vps_base_layer_available_flag
#define VPS_VUI_VST_PARAMS               1      ///< JCTVC-R0227: Related to signalling of VST parameters of the base layer.
#define VPS_VUI_OFFSET                   1      ///< N0085: Signal VPS VUI offset in the VPS extension 
#define REMOVE_BSP_HRD_SEI               1      ///< R0010: Remove bitstream partition HRD SEI message.
#define NESTING_SEI_EXTENSIBILITY        1      ///< R0221: Signalling the number of SEI messages in bitstream partition nesting SEI message
#define POC_RESET_RESTRICTIONS           1      ///< Restrictions on semantics of POC reset-related syntax elements, including one item from R0223
#define POC_RESET_VALUE_RESTRICTION      1      ///< R0223: Restriction on the value of full_poc_reset_flag
#define OUTPUT_LAYER_SETS_CONFIG         1
#define MOVE_ADDN_LS_SIGNALLING          1      ///< JCTVC-R0238: (Ed.) Move additional layer set signalling in VPS extn.
#define PTL_SIGNALLING                   1      ///< Overall macro for all PTL-related signalling
#if PTL_SIGNALLING
#define LIST_OF_PTL                      1      ///< JCTVC-R0272: Signalling the PTL for the 0-th OLS
#define NECESSARY_LAYER_FLAG             1      ////< Derivation of NecessaryLayerFlag
#define PER_LAYER_PTL                    1      ///< Signal profile-tier-level information for each layer.
#endif
#define BSP_INIT_ARRIVAL_SEI             1      ///< JCTVC-R0231: Make signalling of vcl_initial_arrival_delay independent of NalHrdBpPresentFlag
#define SUB_LAYERS_IN_LAYER_SET          1      ///< Move calculation of MaxSubLayerInLayerSets to a separate function
#define VPS_VUI_BSP_HRD_PARAMS           1      ///< JCTVC-R0231: Define the VPS VUI BSP hrd_params() as a separate function, and apply changes adopted.
#define O0137_MAX_LAYERID                1      ///< JCTVC-O0137, JCTVC-O0200, JCTVC-O0223: restrict nuh_layer_id and vps_max_layers_minus1

#define R0226_CONSTRAINT_TMVP_SEI        1      ///< JCTVC-R0226, Modification to semantics in temporal motion vector prediction constraints SEI message
#define R0226_SLICE_TMVP                 1      ///< JCTVC-R0226, Regarding slice_temporal_mvp_enabled_flag
#define R0279_REP_FORMAT_INBL            1      ///< JCTVC-R0279, For any independent non-base layer the used representation format is the one that is signalled in the active SPS for the layer
#define R0227_VUI_BSP_HRD_FLAG           1      ///< JCTVC-R0227, Conformance checking such that VPS VUI HRD only present if VPS timing info is signalled
#define R0227_REP_FORMAT_CONSTRAINT      1      ///< JCTVC-R0227, Conformance checking such that representation format of a particular layer shall not be greater than the one defined in VPS for that layer
#define R0227_BR_PR_ADD_LAYER_SET        1      ///< JCTVC-R0227, Signalling of bit-rate and picture rate for additional layer set
#define R0042_PROFILE_INDICATION         1      ///< JCTVC-R0042, Profile indication for additional layer sets

#define Q0108_TSA_STSA                   1      ///< JCTVC-Q0108, Remove cross-layer alignment constraints of TSA and STSA pictures, enable to have different prediction structures in different layers
#define Q0177_SPS_TEMP_NESTING_FIX       1      ///< JCTVC-Q0177; Fix the inference value of sps_temporal_id_nesting_flag when it is not present
#define Q0177_EOS_CHECKS                 1      ///< JCTVC-Q0177; Put checks on handling EOS
#define Q0142_POC_LSB_NOT_PRESENT        1      ///< JCTVC-Q0142; Add constraint checking on the value of poc_reset_idc and poc_lsb_val
#define Q0146_SSH_EXT_DATA_BIT           1      ///< JCTVC-Q0146; Bug-fix -- the SSH_EXT_DATA_BIT can have any value -- not required to be 1

#define P0130_EOB                        1      ///< JCTVC-P0130, set layer Id of EOB NALU to be fixed to 0
#define T_ID_EOB_BUG_FIX                 1      ///< Bug fix for the value of temporal id of EOB NALU. It must be set to 0
#define P0307_REMOVE_VPS_VUI_OFFSET      1      ///< JCTVC-P0307, remove implementation related to VPS VUI offset signalling
#define P0307_VPS_NON_VUI_EXTENSION      1      ///< JCTVC-P0307, implementation related to NON VUI VPS Extension signalling
#define P0307_VPS_NON_VUI_EXT_UPDATE     1      ///< JCTVC-P0307, implementation related to NON VUI VPS Extension signalling

#define DISCARDABLE_PIC_RPS              1      ///< JCTVC-P0130: Inter-layer RPS and temporal RPS should not contain picture with discardable_flag equal to 1
#define VPS_EXTN_UEV_CODING              1      ///< JCTVC-P0306: Code some syntax elements as ue(v), and remove some syntax elements that duplicate behaviour
#define CHANGE_NUMSUBDPB_IDX             1      ///< Change index of NumSubDpb from output layer set to layer set, to be more aligned with the Spec
#define RESOLUTION_BASED_DPB             0      ///< JCTVC-Q0154 - remove sharing of sub-DPB across layers
                                                ///< JCTVC-P0192: Assign layers to sub-DPBs based on the rep_format() signaled in the VPS
#define ALIGNED_BUMPING                  1      ///< JCTVC-P0192: Align bumping of pictures in an AU
#define O0109_O0199_FLAGS_TO_VUI         1      ///< JCTVC-O0109, O0199: move single_layer_for_non_irap_flag and higher_layer_flag to vps_vui
#define O0109_VIEW_ID_LEN                1      ///< JCTVC-O0109: view_id_len_minus1 to view_id_len, and add constraint (1<<view_id_len) is greater than or equal to NumViews

#define P0048_REMOVE_PROFILE_REF         1      ///< JCTVC-P0048: remove profile_ref_minus1
#if !P0048_REMOVE_PROFILE_REF
#define O0109_PROF_REF_MINUS1            1      ///< JCTVC-O0109: constraint that profile_ref_minus1[i] shall be less than or equal to i
#endif

#define P0295_DEFAULT_OUT_LAYER_IDC      1      ///< JCTVC-P0295: modifify default_one_target_output_layer_idc to default_output_layer_idc
#define O0109_DEFAULT_ONE_OUT_LAYER_IDC  1      ///< JCTVC-O0109: default_one_target_output_layer_flag to default_one_target_output_layer_idc
#define O0109_MOVE_VPS_VUI_FLAG          1      ///< JCTVC-O0109: move vps_vui_present_flag before vps_vui_offset

#define O0135_DEFAULT_ONE_OUT_SEMANTIC   1      ///< JCTVC-O0135: semantics change of default_one_target_output_layer_idc for auxiliary pictures

#define O0164_MULTI_LAYER_HRD            1      ///< JCTVC-O0164: Multi-layer HRD operation
#define Q0182_MULTI_LAYER_HRD_UPDATE     1      ///< JCTVC-Q0182: On bitstream partition buffering

#define O0194_DIFFERENT_BITDEPTH_EL_BL   1      ///< JCTVC-O0194: Support for different bitdepth values for BL and EL, add required configuration parameters (and Some bugfixes when REPN_FORMAT_IN_VPS (JCTVC-N0092) is enabled)
#if O0194_DIFFERENT_BITDEPTH_EL_BL
#define O0194_JOINT_US_BITSHIFT          1      ///< JCTVC-O0194: Joint Upsampling and bit-shift
#endif
#define Q0048_CGS_3D_ASYMLUT             1      ///< JCTVC-Q0048: Colour gamut scalability with look-up table
#if Q0048_CGS_3D_ASYMLUT
#define CGS_GCC_NO_VECTORIZATION         1
#define R0150_CGS_SIGNAL_CONSTRAINTS     1      ///< JCTVC-R0150: CGS signaling improvement and constraints
#define R0151_CGS_3D_ASYMLUT_IMPROVE     1      ///< JCTVC-R0151: Non-uniform chroma partitioning and improved LUT coefficient coding
#define R0164_CGS_LUT_BUGFIX             1      ///< JCTVC-R0164: Bug fix with LUT syntax
#define R0164_CGS_LUT_BUGFIX_CHECK       0      ///< JCTVC-R0164: Add traces explicitly/non-explicitly encoded vertices and check if 3DLUT is correctly filled
#define R0179_CGS_SIZE_8x1x1             1      ///< JCTVC-R0179: allow CGS LUT size to be 8x1x1 as well 
#define R0300_CGS_RES_COEFF_CODING       1      ///< JCTVC-R0300: improved residual coefficient coding for R0151
#define R0179_ENC_OPT_3DLUT_SIZE         0      ///< JCTVC-R0179: RD decision based LUT size selection 
#endif
#define O0194_WEIGHTED_PREDICTION_CGS    1      ///< JCTVC-O0194: Weighted prediction for colour gamut scalability
#define POC_RESET_FLAG                   0      ///< JCTVC-N0244: POC reset flag for  layer pictures.
#define POC_RESET_IDC                    1      ///< JCTVC-P0041: Include poc_reset_idc and related derivation - eventually will replace POC_RESET_FLAG
#if POC_RESET_IDC
#define POC_RESET_IDC_SIGNALLING         1      ///< JCTVC-P0041: Include signalling for poc_reset related syntax elements
#define POC_RESET_IDC_ENCODER            1      ///< JCTVC-P0041: Include support of enabling POC reset at the encoder
#define POC_RESET_IDC_DECODER            1      ///< JCTVC-P0041: Include support of enabling POC reset at the decoder
#define ALIGN_IRAP_BUGFIX                1
#define UNAVAILABLE_PIC_BUGFIX           1
#endif
#if INFERENCE_POC_MSB_VAL_PRESENT
#define POC_MSB_VAL_PRESENT_FLAG_SEM     0      ///< JCTVC-Q0146: Inference of poc_msb_val_present_flag
#else
#define POC_MSB_VAL_PRESENT_FLAG_SEM     1      ///< JCTVC-Q0146: Inference of poc_msb_val_present_flag
#endif
#define POC_RESET_INFO_INFERENCE         1      ///< JCTVC-Q0146: Infer the value of poc_reset_info_present_flag when not present
#define NO_OUTPUT_OF_PRIOR_PICS          1      ///< Use no_output_of_prior_pics_flag
#define REPN_FORMAT_IN_VPS               1      ///< JCTVC-N0092: Signal represenation format (spatial resolution, bit depth, colour format) in the VPS
#if REPN_FORMAT_IN_VPS
#define REPN_FORMAT_CONTROL_FLAG         1      ///< JCTVC-O0179: Add control flag in representation format to control sending of chroma and bitdepth parameters
#endif 
#define RPL_INIT_N0316_N0082             1      ///< JCTVC-N0316, JCTVC-N0082: initial reference picture list construction 

#define SCALINGLIST_INFERRING            1      ///< JCTVC-N0371: inter-layer scaling list
#define O0142_CONDITIONAL_SPS_EXTENSION  1      ///< JCTVC-O0142: Conditional SPS extension
#if POC_RESET_FLAG
#define PREVTID0_POC_RESET               1      ///< JCTVC-O0117 Modification of the PicOrderCntVal of prevTid0Pic
#define POC_RESET_RPS                    1      ///< JCTVC-O0117 Modification to the decoding process for rps
#endif

#define P0297_VPS_POC_LSB_ALIGNED_FLAG   1      ///< JCTVC-P0297: vps_poc_lsb_aligned_flag for cross-layer POC anchor picture derivation

#define VPS_EXTN_MASK_AND_DIM_INFO       1      ///< Include avc_base_layer_flag, splitting_flag, scalability mask and dimension related info
#if VPS_EXTN_MASK_AND_DIM_INFO
#define MAX_VPS_NUM_SCALABILITY_TYPES    16
#endif
#define VPS_EXTN_OP_LAYER_SETS           1      ///< Include output layer sets in VPS extension
#define VPS_EXTN_PROFILE_INFO            1      ///< Include profile information for layer sets in VPS extension
#define VPS_EXTN_DIRECT_REF_LAYERS       1      ///< Include indication of direct dependency of layers in VPS extension
#define M0040_ADAPTIVE_RESOLUTION_CHANGE 1

#define VPS_VUI_TILES_NOT_IN_USE__FLAG   1      ///< JCTVC-O0226: VPS VUI flag to indicate tile not in use
#define VPS_VUI_WPP_NOT_IN_USE__FLAG     1      ///< JCTVC-O0226: VPS VUI flag to indicate tile not in use
#define N0160_VUI_EXT_ILP_REF            1      ///< VUI extension inter-layer dependency offset signalling
#if M0040_ADAPTIVE_RESOLUTION_CHANGE
#define HIGHER_LAYER_IRAP_SKIP_FLAG      1      ///< JCTVC-O0199: Indication that higher layer IRAP picture uses skip blocks only
#endif
#define VPS_VUI_VIDEO_SIGNAL             1      ///< JCTVC-O0118 video signal information
#if VPS_VUI_VIDEO_SIGNAL
#define VPS_VUI_VIDEO_SIGNAL_MOVE        1      ///< JCTVC-P0076 Move video signal information syntax structure earlier in the VPS VUI
#endif 
#define P0182_VPS_VUI_PS_FLAG            1      ///< JCTVC-P0182, add base_layer_parameter_set_compatibility_flag

#define P0125_REVERT_VPS_EXTN_OFFSET_TO_RESERVED        1   ///< JCTVC-P0125 -- Keep it as a reserved FFFF value --- The following two macros (VPS_EXTN_OFFSET & VPS_EXTN_OFFSET_CALC) will have no effect when this macro is set to 1.
#define VPS_EXTN_OFFSET                  1      ///< implementation of vps_extension_offset syntax element
#define VPS_EXTN_OFFSET_CALC             1      ///< Calculation of VPS extension offset
#define SPS_PTL_FIX                      1      ///< remove profile_tier_level from enhancement layer SPS

#define DERIVE_LAYER_ID_LIST_VARIABLES   1      ///< Derived variables based on the variables in VPS - for use in syntax table parsing

#define AVC_BASE                         1      ///< YUV BL reading for AVC base SVC

#define REF_IDX_MFM                      1      ///< JCTVC-L0336: motion vector mapping of inter-layer reference picture
#define MAX_ONE_RESAMPLING_DIRECT_LAYERS 1      ///< Allow maximum of one resampling process for direct reference layers
#define MOTION_RESAMPLING_CONSTRAINT     1      ///< JCTVC-N0108: Allow maximum of one motion resampling process for direct reference layers, and use motion inter-layer prediction from the same layer as texture inter-layer prediction.
#define EARLY_REF_PIC_MARKING            0      ///< Valencia meeting - remove early marking of inter-layer reference pictures
                                                ///< Decoded picture marking of sub-layer non-reference pictures
#define O0225_MAX_TID_FOR_REF_LAYERS     1
#define O0225_TID_BASED_IL_RPS_DERIV     1
#define VPS_TSLAYERS                     1      ///< JCTVC-O0120 signal max temporal sub-layers for each layer
#define TSLAYERS_IL_RPS                  1      ///< JCTVC-O0120 IL RPS based on max temporal sub-layers
#define P0079_DERIVE_NUMACTIVE_REF_PICS  1      ///< JCTVC-P0079 Modification of derivation of variable NumActiveRefLayerPics
#define Q0060_MAX_TID_REF_EQUAL_TO_ZERO  1      ///< JCTVC-Q0060 handling the case max_tid_il_ref_pics_plus1 is equal to 0.

#define O0223_PICTURE_TYPES_ALIGN_FLAG   1      ///< a flag to indicatate whether picture types are aligned across layers.

#define P0068_CROSS_LAYER_ALIGNED_IDR_ONLY_FOR_IRAP_FLAG   1  ///< a flag to indicatate whether picture types for IRAP are IDR across layers.

#define IRAP_ALIGN_FLAG_IN_VPS_VUI       1      ///< Move IRAP align flag to VPS VUI 

#define VIEW_ID_RELATED_SIGNALING        1      ///< Introduce syntax elements view_id and view_id_val
#define N0065_LAYER_POC_ALIGNMENT        1

#define O0215_PHASE_ALIGNMENT            1      ///< JCTVC_O0215: signal a flag to specify phase alignment case, 0: zero-position-aligned, 1: central-position-aligned,
#define AUXILIARY_PICTURES               1      ///< JCTVC-O0041: auxiliary picture layers

#define O0062_POC_LSB_NOT_PRESENT_FLAG   1      ///< JCTVC-O0062: signal poc_lsb_not_present_flag for each layer in VPS extension
#define SHM_FIX7                         1      ///< fix for SHVC WD ticket #7

#define O0092_0094_DEPENDENCY_CONSTRAINT 1      ///< JCTVC-O0092: constraint on the layer_id of SPS/PPS
#if O0092_0094_DEPENDENCY_CONSTRAINT
#define MAX_REF_LAYERS                   7
#endif
#define O0096_REP_FORMAT_INDEX           1      ///< JCTVC-O0096: identify SPS rep_format() with an index into the lists of formats in VPS extension.
#define O0096_DEFAULT_DEPENDENCY_TYPE    1      ///< JCTVC-O0096: specify default dependency type for all direct reference layers

#define Q0195_REP_FORMAT_CLEANUP         1      ///< JCTVC-Q0195: restructureing of rep_format() signaling
#define REP_FORMAT_FIX                   1      ///< update_rep_format_flag should be inferred to be equal to 0

#define R0156_CONF_WINDOW_IN_REP_FORMAT  O0096_REP_FORMAT_INDEX ///< JCTVC-R0156: add conformance window cropping offsets to rep_format()

#define RESAMPLING_CONSTRAINT_BUG_FIX    1
#define O0098_SCALED_REF_LAYER_ID        1      ///< JCTVC-O0098: signal scaled reference id

#define O0153_ALT_OUTPUT_LAYER_FLAG      1      ///< JCTVC-O0153: alt output layer flag
#define P0300_ALT_OUTPUT_LAYER_FLAG      1      ///< JCTVC-P0300: alt output layer flag

#define Q0165_OUTPUT_LAYER_SET           1      ///< JCTVC-Q0165: add a constraint to disallow an empty output layer set
#define Q0165_NUM_ADD_OUTPUT_LAYER_SETS  1      ///< JCTVC-Q0165: signal num_add_olss and default_target_output_layer_idc when vps_num_layer_sets_minus1 is greater than 0

#define Q0078_ADD_LAYER_SETS             1      ///< JCTVC-Q0078: additional layer sets and layer set config
#if Q0078_ADD_LAYER_SETS
#define MAX_NUM_ADD_LAYER_SETS           1023
#endif

#define VPS_DPB_SIZE_TABLE               1      ///< JCTVC-O0217: DPB operations: signaling DPB-related parameters
#if VPS_DPB_SIZE_TABLE 
#define OUTPUT_LAYER_SET_INDEX           1      ///< JCTVC-O0217: DPB operations: Inference/input of output layer set index
#if OUTPUT_LAYER_SET_INDEX
#define USE_DPB_SIZE_TABLE               1      ///< JCTVC-O0217: DPB operations: Use signaled DPB-size table parameters in the decoder
#endif
#endif
#define SPS_DPB_PARAMS                   1      ///< JCTVC-P0155 signaling & inferrence for sps dpb parameters for nuh_layer_id > 0
#define DPB_PARAMS_MAXTLAYERS            1      ///< JCTVC-P0156 DPB parameters up to maximum temporal sub-layers in the layer set
#define NUM_OL_FLAGS                     1      ///< JCTVC-P0156 output_layer_flag[ i ][ j ] is signalled for j equal to 0 to NumLayersInIdList[ lsIdx ] inclusive
#define NO_CLRAS_OUTPUT_FLAG             1
#define O0149_CROSS_LAYER_BLA_FLAG       1      ///< JCTVC-O0149: signal cross_layer_bla_flag in slice header

#define P0138_USE_ALT_CPB_PARAMS_FLAG    1      ///< JCTVC-P0138: use_alt_cpb_params_flag syntax in buffering period SEI message extension
#define P0166_MODIFIED_PPS_EXTENSION     1      ///< JCTVC-P0166: add pps_extension_type_flag
#define BITRATE_PICRATE_SIGNALLING       1      ///< JCTVC-Q0102 Proposal 3 signal bitrate, picrate only up to the maximum temporal sub-layers in the corresponding layer set
#define LAYER_DECPICBUFF_PARAM           1      ///< JCTVC-Q0102 Proposal 2 infer value from layer DPB param
#define HRD_BPB                          1      ///< JCTVC-Q0101 Bitstream Partition Buffering Proposals
#define DPB_CONSTRAINTS                  1      ///< JCTVC-Q0100 RPS DPB constraints
#define DPB_INTERNAL_BL_SIG              1      ///< JCTVC-R0153: external base layer
#define ALT_OPT_LAYER_FLAG               1      ///< JCTVC-R0154: proposal1 - alt_output_layer_flag[i] inference
#define DEF_OPT_LAYER_IDC                1      ///< JCTVC-R0154: proposal2 - default_output_layer_idc and output_layer_flag[i][j]
#define ISLICE_TYPE_NUMDIR               1      ///< JCTVC-R0155: Proposal 1 I slice_type
#define OLS_IDX_CHK                      1      ///< JCTVC-R0155: Proposal 2 valid range for output_layer_set_idx_to_vps[i]
#define R0340_RESAMPLING_MODIFICATION    1      ///< JCTVC-R0340: set of changes regarding resampling (as listed below)
#if R0340_RESAMPLING_MODIFICATION
#define MOVE_SCALED_OFFSET_TO_PPS        1      ///< JCTVC-R0013: move scaled reference layer offset from SPS to PPS
#if MOVE_SCALED_OFFSET_TO_PPS
#define REF_REGION_OFFSET                1      ///< JCTVC-Q0159/R0220: reference region offset
#define R0209_GENERIC_PHASE              1      ///< JCTVC-R0209: resampling with generic phase
#define R0220_REMOVE_EL_CLIP             1      ///< JCTVC-R0220: remove clip to scaled ref window in resampling process
#define RESAMPLING_FIX                   1      ///< Resampling fix -- equal offset check and conformance check
#endif
#else
#define Q0200_CONFORMANCE_BL_SIZE        1      ///< JCTVC-Q0200; use conformance picture size in re-sampling processs
#define P0312_VERT_PHASE_ADJ             1      ///< JCTVC-P0312: vertical phase adjustment in re-sampling process (BoG report)
#if P0312_VERT_PHASE_ADJ
#define Q0120_PHASE_CALCULATION          1      ///< JCTVC-Q0120 phase offset derivation for combination of spatial scalibility and field coding.
#endif
#endif

/// scalability types
enum ScalabilityType
{
  VIEW_ORDER_INDEX  = 1,
  SCALABILITY_ID    = 2,
  AUX_ID            = 3,
};

#define WPP_FIX                          1

/// normative encoder constraints --------
#define MFM_ENCCONSTRAINT                1      ///< JCTVC-O0216: Encoder constraint for motion field mapping
#define REF_IDX_ME_ZEROMV                1      ///< JCTVC-L0051: use zero motion for inter-layer reference picture (without fractional ME)

/// encoder settings ---------------------
#define FAST_INTRA_SHVC                  1      ///< JCTVC-M0115: reduction number of intra modes in the EL (encoder only)
#if FAST_INTRA_SHVC
#define NB_REMAIN_MODES                  2      ///< JCTVC-M0115: nb of remaining modes
#endif
#define RC_SHVC_HARMONIZATION            1      ///< JCTVC-M0037: rate control for SHVC
#define JCTVC_M0259_LAMBDAREFINEMENT     1      ///< JCTVC-M0259: lambda refinement (encoder only optimization)
#define ENCODER_FAST_MODE                1      ///< JCTVC-L0174: enable encoder fast mode. TestMethod 1 is enabled by setting to 1 and TestMethod 2 is enable by setting to 2. By default it is set to 1.
#define LAYER_CTB                        0      ///< enable layer-specific CTB structure

/// SEI messages -------------------------
#define P0050_KNEE_FUNCTION_SEI          1      ///< JCTVC-P0050: Knee function SEI
#define SUB_BITSTREAM_PROPERTY_SEI       1      ///< JCTVC-P0204: Sub-bitstream property SEI message
#if SUB_BITSTREAM_PROPERTY_SEI
#define MAX_SUB_STREAMS                  1024
#endif
#define LAYERS_NOT_PRESENT_SEI           1      ///< JCTVC-M0043: add layers not present SEI.
#define N0383_IL_CONSTRAINED_TILE_SETS_SEI  1
#define Q0189_TMVP_CONSTRAINTS           1      ///< JCTVC-Q0189: indicate constraints on TMVP
#define Q0247_FRAME_FIELD_INFO           1      ///< JCTVC-Q0247: field_frame_info SEI message
#define R0247_SEI_ACTIVE                 1      ///< JCTVC-R0247: active parameter sets SEI message

#endif // SVC_EXTENSION
#define Q0074_COLOUR_REMAPPING_SEI       1      ///< JCTVC-Q0074, JCTVC-R0344: SEI Colour Remapping Information


//! \ingroup TLibCommon
//! \{
#define HARMONIZE_GOP_FIRST_FIELD_COUPLE  1
#define FIX_FIELD_DEPTH                 1
#define EFFICIENT_FIELD_IRAP            1
#define ALLOW_RECOVERY_POINT_AS_RAP     1
#define BUGFIX_INTRAPERIOD 1
#define SAO_ENCODE_ALLOW_USE_PREDEBLOCK 1

#define SAO_SGN_FUNC 1

#define TILE_SIZE_CHECK 1

#define FIX1172 1 ///< fix ticket #1172

#define SETTING_PIC_OUTPUT_MARK     1
#define SETTING_NO_OUT_PIC_PRIOR    1
#define FIX_EMPTY_PAYLOAD_NAL       1
#define FIX_WRITING_OUTPUT          1
#define FIX_OUTPUT_EOS              1

#define FIX_POC_CRA_NORASL_OUTPUT   1

#define MAX_NUM_PICS_IN_SOP           1024

#define MAX_NESTING_NUM_OPS         1024
#define MAX_NESTING_NUM_LAYER       64
#if NESTING_SEI_EXTENSIBILITY
#define MAX_SEIS_IN_BSP_NESTING     64
#endif
#if SVC_EXTENSION
#define MAX_VPS_OP_LAYER_SETS_PLUS1               (MAX_LAYERS+1)
#define MAX_VPS_LAYER_SETS_PLUS1                  1024
#define MAX_VPS_OUTPUT_LAYER_SETS_PLUS1           1024
#define MAX_VPS_LAYER_ID_PLUS1                    MAX_LAYERS
#else
#define MAX_VPS_NUM_HRD_PARAMETERS                1
#define MAX_VPS_OP_SETS_PLUS1                     1024
#define MAX_VPS_NUH_RESERVED_ZERO_LAYER_ID_PLUS1  1
#endif

#define MAX_CPB_CNT                     32  ///< Upper bound of (cpb_cnt_minus1 + 1)
#if O0137_MAX_LAYERID
#define MAX_NUM_LAYER_IDS                63
#else
#define MAX_NUM_LAYER_IDS                64
#endif
#define COEF_REMAIN_BIN_REDUCTION        3 ///< indicates the level at which the VLC 
                                           ///< transitions from Golomb-Rice to TU+EG(k)

#define CU_DQP_TU_CMAX 5                   ///< max number bins for truncated unary
#define CU_DQP_EG_k 0                      ///< expgolomb order

#define SBH_THRESHOLD                    4  ///< I0156: value of the fixed SBH controlling threshold
  
#define SEQUENCE_LEVEL_LOSSLESS           0  ///< H0530: used only for sequence or frame-level lossless coding

#define DISABLING_CLIP_FOR_BIPREDME         1  ///< Ticket #175
  
#define C1FLAG_NUMBER               8 // maximum number of largerThan1 flag coded in one chunk :  16 in HM5
#define C2FLAG_NUMBER               1 // maximum number of largerThan2 flag coded in one chunk:  16 in HM5 
#define SAO_ENCODING_CHOICE              1  ///< I0184: picture early termination
#if SAO_ENCODING_CHOICE
#define SAO_ENCODING_RATE                0.75
#define SAO_ENCODING_CHOICE_CHROMA       1 ///< J0044: picture early termination Luma and Chroma are handled separately
#if SAO_ENCODING_CHOICE_CHROMA
#define SAO_ENCODING_RATE_CHROMA         0.5
#endif
#endif

#define MAX_NUM_VPS                8
#define MAX_NUM_SPS                16
#define MAX_NUM_PPS                64

#define RDOQ_CHROMA_LAMBDA          1   ///< F386: weighting of chroma for RDOQ

#define MIN_SCAN_POS_CROSS          4

#define FAST_BIT_EST                1   ///< G763: Table-based bit estimation for CABAC

#define MLS_GRP_NUM                         64     ///< G644 : Max number of coefficient groups, max(16, 64)
#define MLS_CG_SIZE                         4      ///< G644 : Coefficient group size of 4x4

#define ADAPTIVE_QP_SELECTION               1      ///< G382: Adaptive reconstruction levels, non-normative part for adaptive QP selection
#if ADAPTIVE_QP_SELECTION
#define ARL_C_PRECISION                     7      ///< G382: 7-bit arithmetic precision
#define LEVEL_RANGE                         30     ///< G382: max coefficient level in statistics collection
#endif

#define HHI_RQT_INTRA_SPEEDUP             1           ///< tests one best mode with full rqt
#define HHI_RQT_INTRA_SPEEDUP_MOD         0           ///< tests two best modes with full rqt

#if HHI_RQT_INTRA_SPEEDUP_MOD && !HHI_RQT_INTRA_SPEEDUP
#error
#endif

#define VERBOSE_RATE 0 ///< Print additional rate information in encoder

#define AMVP_DECIMATION_FACTOR            4

#define SCAN_SET_SIZE                     16
#define LOG2_SCAN_SET_SIZE                4

#define FAST_UDI_MAX_RDMODE_NUM               35          ///< maximum number of RD comparison in fast-UDI estimation loop 

#define ZERO_MVD_EST                          0           ///< Zero Mvd Estimation in normal mode

#define NUM_INTRA_MODE 36
#if !REMOVE_LM_CHROMA
#define LM_CHROMA_IDX  35
#endif

#define WRITE_BACK                      1           ///< Enable/disable the encoder to replace the deltaPOC and Used by current from the config file with the values derived by the refIdc parameter.
#define AUTO_INTER_RPS                  1           ///< Enable/disable the automatic generation of refIdc from the deltaPOC and Used by current from the config file.
#define PRINT_RPS_INFO                  0           ///< Enable/disable the printing of bits used to send the RPS.
                                                    // using one nearest frame as reference frame, and the other frames are high quality (POC%4==0) frames (1+X)
                                                    // this should be done with encoder only decision
                                                    // but because of the absence of reference frame management, the related code was hard coded currently

#define RVM_VCEGAM10_M 4

#define PLANAR_IDX             0
#define VER_IDX                26                    // index for intra VERTICAL   mode
#define HOR_IDX                10                    // index for intra HORIZONTAL mode
#define DC_IDX                 1                     // index for intra DC mode
#define NUM_CHROMA_MODE        5                     // total number of chroma modes
#define DM_CHROMA_IDX          36                    // chroma mode index for derived from luma intra mode


#define FAST_UDI_USE_MPM 1

#define RDO_WITHOUT_DQP_BITS              0           ///< Disable counting dQP bits in RDO-based mode decision

#define FULL_NBIT 0 ///< When enabled, compute costs using full sample bitdepth.  When disabled, compute costs as if it is 8-bit source video.
#if FULL_NBIT
# define DISTORTION_PRECISION_ADJUSTMENT(x) 0
#else
# define DISTORTION_PRECISION_ADJUSTMENT(x) (x)
#endif

#define LOG2_MAX_NUM_COLUMNS_MINUS1        7
#define LOG2_MAX_NUM_ROWS_MINUS1           7
#define LOG2_MAX_COLUMN_WIDTH              13
#define LOG2_MAX_ROW_HEIGHT                13

#define MATRIX_MULT                             0   // Brute force matrix multiplication instead of partial butterfly

#define REG_DCT 65535

#define AMP_SAD                               1           ///< dedicated SAD functions for AMP
#define AMP_ENC_SPEEDUP                       1           ///< encoder only speed-up by AMP mode skipping
#if AMP_ENC_SPEEDUP
#define AMP_MRG                               1           ///< encoder only force merge for AMP partition (no motion search for AMP)
#endif

#define CABAC_INIT_PRESENT_FLAG     1

// ====================================================================================================================
// Basic type redefinition
// ====================================================================================================================

typedef       void                Void;
typedef       bool                Bool;

#ifdef __arm__
typedef       signed char         Char;
#else
typedef       char                Char;
#endif
typedef       unsigned char       UChar;
typedef       short               Short;
typedef       unsigned short      UShort;
typedef       int                 Int;
typedef       unsigned int        UInt;
typedef       double              Double;
typedef       float               Float;
// ====================================================================================================================
// 64-bit integer type
// ====================================================================================================================

#ifdef _MSC_VER
typedef       __int64             Int64;

#if _MSC_VER <= 1200 // MS VC6
typedef       __int64             UInt64;   // MS VC6 does not support unsigned __int64 to double conversion
#else
typedef       unsigned __int64    UInt64;
#endif

#else

typedef       long long           Int64;
typedef       unsigned long long  UInt64;

#endif

// ====================================================================================================================
// Type definition
// ====================================================================================================================

typedef       UChar           Pxl;        ///< 8-bit pixel type
typedef       Short           Pel;        ///< 16-bit pixel type
typedef       Int             TCoeff;     ///< transform coefficient

/// parameters for adaptive loop filter
class TComPicSym;

// Slice / Slice segment encoding modes
enum SliceConstraint
{
  NO_SLICES              = 0,          ///< don't use slices / slice segments
  FIXED_NUMBER_OF_LCU    = 1,          ///< Limit maximum number of largest coding tree blocks in a slice / slice segments
  FIXED_NUMBER_OF_BYTES  = 2,          ///< Limit maximum number of bytes in a slice / slice segment
  FIXED_NUMBER_OF_TILES  = 3,          ///< slices / slice segments span an integer number of tiles
};

enum SAOComponentIdx
{
  SAO_Y =0,
  SAO_Cb,
  SAO_Cr,
  NUM_SAO_COMPONENTS
};

enum SAOMode //mode
{
  SAO_MODE_OFF = 0,
  SAO_MODE_NEW,
  SAO_MODE_MERGE,
  NUM_SAO_MODES
};

enum SAOModeMergeTypes 
{
  SAO_MERGE_LEFT =0,
  SAO_MERGE_ABOVE,
  NUM_SAO_MERGE_TYPES
};


enum SAOModeNewTypes 
{
  SAO_TYPE_START_EO =0,
  SAO_TYPE_EO_0 = SAO_TYPE_START_EO,
  SAO_TYPE_EO_90,
  SAO_TYPE_EO_135,
  SAO_TYPE_EO_45,
  
  SAO_TYPE_START_BO,
  SAO_TYPE_BO = SAO_TYPE_START_BO,

  NUM_SAO_NEW_TYPES
};
#define NUM_SAO_EO_TYPES_LOG2 2

enum SAOEOClasses 
{
  SAO_CLASS_EO_FULL_VALLEY = 0,
  SAO_CLASS_EO_HALF_VALLEY = 1,
  SAO_CLASS_EO_PLAIN       = 2,
  SAO_CLASS_EO_HALF_PEAK   = 3,
  SAO_CLASS_EO_FULL_PEAK   = 4,
  NUM_SAO_EO_CLASSES,
};


#define NUM_SAO_BO_CLASSES_LOG2  5
enum SAOBOClasses
{
  //SAO_CLASS_BO_BAND0 = 0,
  //SAO_CLASS_BO_BAND1,
  //SAO_CLASS_BO_BAND2,
  //...
  //SAO_CLASS_BO_BAND31,

  NUM_SAO_BO_CLASSES = (1<<NUM_SAO_BO_CLASSES_LOG2),
};
#define MAX_NUM_SAO_CLASSES  32  //(NUM_SAO_EO_GROUPS > NUM_SAO_BO_GROUPS)?NUM_SAO_EO_GROUPS:NUM_SAO_BO_GROUPS

struct SAOOffset
{
  Int modeIdc; //NEW, MERGE, OFF
  Int typeIdc; //NEW: EO_0, EO_90, EO_135, EO_45, BO. MERGE: left, above
  Int typeAuxInfo; //BO: starting band index
  Int offset[MAX_NUM_SAO_CLASSES];

  SAOOffset();
  ~SAOOffset();
  Void reset();

  const SAOOffset& operator= (const SAOOffset& src);
};

struct SAOBlkParam
{

  SAOBlkParam();
  ~SAOBlkParam();
  Void reset();
  const SAOBlkParam& operator= (const SAOBlkParam& src);
  SAOOffset& operator[](Int compIdx){ return offsetParam[compIdx];}
private:
  SAOOffset offsetParam[NUM_SAO_COMPONENTS];

};

/// parameters for deblocking filter
typedef struct _LFCUParam
{
  Bool bInternalEdge;                     ///< indicates internal edge
  Bool bLeftEdge;                         ///< indicates left edge
  Bool bTopEdge;                          ///< indicates top edge
} LFCUParam;

// ====================================================================================================================
// Enumeration
// ====================================================================================================================

/// supported slice type
enum SliceType
{
  B_SLICE,
  P_SLICE,
  I_SLICE
};

/// chroma formats (according to semantics of chroma_format_idc)
enum ChromaFormat
{
  CHROMA_400  = 0,
  CHROMA_420  = 1,
  CHROMA_422  = 2,
  CHROMA_444  = 3
#if AUXILIARY_PICTURES
  ,NUM_CHROMA_FORMAT = 4
#endif
};

/// supported partition shape
enum PartSize
{
  SIZE_2Nx2N,           ///< symmetric motion partition,  2Nx2N
  SIZE_2NxN,            ///< symmetric motion partition,  2Nx N
  SIZE_Nx2N,            ///< symmetric motion partition,   Nx2N
  SIZE_NxN,             ///< symmetric motion partition,   Nx N
  SIZE_2NxnU,           ///< asymmetric motion partition, 2Nx( N/2) + 2Nx(3N/2)
  SIZE_2NxnD,           ///< asymmetric motion partition, 2Nx(3N/2) + 2Nx( N/2)
  SIZE_nLx2N,           ///< asymmetric motion partition, ( N/2)x2N + (3N/2)x2N
  SIZE_nRx2N,           ///< asymmetric motion partition, (3N/2)x2N + ( N/2)x2N
  SIZE_NONE = 15
};

/// supported prediction type
enum PredMode
{
  MODE_INTER,           ///< inter-prediction mode
  MODE_INTRA,           ///< intra-prediction mode
  MODE_NONE = 15
};

/// texture component type
enum TextType
{
  TEXT_LUMA,            ///< luma
  TEXT_CHROMA,          ///< chroma (U+V)
  TEXT_CHROMA_U,        ///< chroma U
  TEXT_CHROMA_V,        ///< chroma V
  TEXT_ALL,             ///< Y+U+V
  TEXT_NONE = 15
};

/// reference list index
enum RefPicList
{
  REF_PIC_LIST_0 = 0,   ///< reference list 0
  REF_PIC_LIST_1 = 1,   ///< reference list 1
  REF_PIC_LIST_X = 100  ///< special mark
};

/// distortion function index
enum DFunc
{
  DF_DEFAULT  = 0,
  DF_SSE      = 1,      ///< general size SSE
  DF_SSE4     = 2,      ///<   4xM SSE
  DF_SSE8     = 3,      ///<   8xM SSE
  DF_SSE16    = 4,      ///<  16xM SSE
  DF_SSE32    = 5,      ///<  32xM SSE
  DF_SSE64    = 6,      ///<  64xM SSE
  DF_SSE16N   = 7,      ///< 16NxM SSE
  
  DF_SAD      = 8,      ///< general size SAD
  DF_SAD4     = 9,      ///<   4xM SAD
  DF_SAD8     = 10,     ///<   8xM SAD
  DF_SAD16    = 11,     ///<  16xM SAD
  DF_SAD32    = 12,     ///<  32xM SAD
  DF_SAD64    = 13,     ///<  64xM SAD
  DF_SAD16N   = 14,     ///< 16NxM SAD
  
  DF_SADS     = 15,     ///< general size SAD with step
  DF_SADS4    = 16,     ///<   4xM SAD with step
  DF_SADS8    = 17,     ///<   8xM SAD with step
  DF_SADS16   = 18,     ///<  16xM SAD with step
  DF_SADS32   = 19,     ///<  32xM SAD with step
  DF_SADS64   = 20,     ///<  64xM SAD with step
  DF_SADS16N  = 21,     ///< 16NxM SAD with step
  
  DF_HADS     = 22,     ///< general size Hadamard with step
  DF_HADS4    = 23,     ///<   4xM HAD with step
  DF_HADS8    = 24,     ///<   8xM HAD with step
  DF_HADS16   = 25,     ///<  16xM HAD with step
  DF_HADS32   = 26,     ///<  32xM HAD with step
  DF_HADS64   = 27,     ///<  64xM HAD with step
  DF_HADS16N  = 28,     ///< 16NxM HAD with step
  
#if AMP_SAD
  DF_SAD12    = 43,
  DF_SAD24    = 44,
  DF_SAD48    = 45,

  DF_SADS12   = 46,
  DF_SADS24   = 47,
  DF_SADS48   = 48,

  DF_SSE_FRAME = 50     ///< Frame-based SSE
#else
  DF_SSE_FRAME = 33     ///< Frame-based SSE
#endif
};

/// index for SBAC based RD optimization
enum CI_IDX
{
  CI_CURR_BEST = 0,     ///< best mode index
  CI_NEXT_BEST,         ///< next best index
  CI_TEMP_BEST,         ///< temporal index
  CI_CHROMA_INTRA,      ///< chroma intra index
  CI_QT_TRAFO_TEST,
  CI_QT_TRAFO_ROOT,
  CI_NUM,               ///< total number
};

/// motion vector predictor direction used in AMVP
enum MVP_DIR
{
  MD_LEFT = 0,          ///< MVP of left block
  MD_ABOVE,             ///< MVP of above block
  MD_ABOVE_RIGHT,       ///< MVP of above right block
  MD_BELOW_LEFT,        ///< MVP of below left block
  MD_ABOVE_LEFT         ///< MVP of above left block
};

/// coefficient scanning type used in ACS
enum COEFF_SCAN_TYPE
{
  SCAN_DIAG = 0,         ///< up-right diagonal scan
  SCAN_HOR,              ///< horizontal first scan
  SCAN_VER               ///< vertical first scan
};

namespace Profile
{
  enum Name
  {
    NONE = 0,
    MAIN = 1,
    MAIN10 = 2,
    MAINSTILLPICTURE = 3,
  };
}

namespace Level
{
  enum Tier
  {
    MAIN = 0,
    HIGH = 1,
  };

  enum Name
  {
    NONE     = 0,
    LEVEL1   = 30,
    LEVEL2   = 60,
    LEVEL2_1 = 63,
    LEVEL3   = 90,
    LEVEL3_1 = 93,
    LEVEL4   = 120,
    LEVEL4_1 = 123,
    LEVEL5   = 150,
    LEVEL5_1 = 153,
    LEVEL5_2 = 156,
    LEVEL6   = 180,
    LEVEL6_1 = 183,
    LEVEL6_2 = 186,
  };
}
//! \}

#endif
