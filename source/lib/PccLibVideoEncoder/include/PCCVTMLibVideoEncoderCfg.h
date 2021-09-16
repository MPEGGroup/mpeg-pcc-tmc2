/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ISO/IEC
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

/** \file     EncAppCfg.h
    \brief    Handle encoder configuration parameters (header)
*/

#ifndef PCCVTMLibVideoEncoderCfg_h
#define PCCVTMLibVideoEncoderCfg_h

#include "PCCCommon.h"

#ifdef USE_VTMLIB_VIDEO_CODEC

#include "CommonLib/CommonDef.h"
#include "EncoderLib/EncCfgParam.h"

#include <map>
template <class T1, class T2>
static inline std::istream& operator>>( std::istream& in, std::map<T1, T2>& map );

#include "Utilities/program_options_lite.h"

#include "EncoderLib/EncCfg.h"
#if EXTENSION_360_VIDEO
#include "AppEncHelper360/TExt360AppEncCfg.h"
#endif

#if JVET_O0756_CALCULATE_HDRMETRICS
#include "HDRLib/inc/DistortionMetric.H"
#endif
namespace po = df::program_options_lite;

#include <sstream>
#include <vector>
//! \ingroup EncoderApp
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// encoder configuration class
class PCCVTMLibVideoEncoderCfg {
#if QP_SWITCHING_FOR_PARALLEL
 public:
  template <class T>
  struct OptionalValue {
    bool bPresent;
    T    value;
    OptionalValue() : bPresent( false ), value() {}
  };
#endif

 protected:
  // file I/O
  std::string m_inputFileName;      ///< source file name
  std::string m_bitstreamFileName;  ///< output bitstream file
  std::string m_reconFileName;      ///< output reconstruction file

#if PCC_ME_EXT
  bool        m_usePCCExt;
  std::string m_blockToPatchFileName;
  std::string m_occupancyMapFileName;
  std::string m_patchInfoFileName;
#endif
  // Lambda modifiers
  double              m_adLambdaModifier[MAX_TLAYER];  ///< Lambda modifier array for each temporal layer
  std::vector<double> m_adIntraLambdaModifier;  ///< Lambda modifier for Intra pictures, one for each temporal layer. If
                                                ///< size>temporalLayer, then use [temporalLayer], else if size>0, use
                                                ///< [size()-1], else use m_adLambdaModifier.
  double m_dIntraQpFactor;  ///< Intra Q Factor. If negative, use a default equation: 0.57*(1.0 - Clip3( 0.0, 0.5,
                            ///< 0.05*(double)(isField ? (GopSize-1)/2 : GopSize-1) ))

  // source specification
  int      m_iFrameRate;              ///< source frame-rates (Hz)
  uint32_t m_FrameSkip;               ///< number of skipped frames from the beginning
  uint32_t m_temporalSubsampleRatio;  ///< temporal subsample ratio, 2 means code every two frames
  int      m_sourceWidth;             ///< source width in pixel
  int      m_sourceHeight;            ///< source height in pixel (when interlaced = field height)
#if EXTENSION_360_VIDEO
  int m_inputFileWidth;   ///< width of image in input file  (this is equivalent to sourceWidth,  if sourceWidth  is not
                          ///< subsequently altered due to padding)
  int m_inputFileHeight;  ///< height of image in input file (this is equivalent to sourceHeight, if sourceHeight is not
                          ///< subsequently altered due to padding)
#endif
  int m_sourceHeightOrg;  ///< original source height in pixel (when interlaced = frame height)

  bool m_isField;  ///< enable field coding
  bool m_isTopFieldFirst;
  bool m_efficientFieldIRAPEnabled;  ///< enable an efficient field IRAP structure.
  bool m_harmonizeGopFirstFieldCoupleEnabled;

  int                        m_conformanceWindowMode;
  int                        m_confWinLeft;
  int                        m_confWinRight;
  int                        m_confWinTop;
  int                        m_confWinBottom;
  int                        m_sourcePadding[2];                  ///< number of padded pixels for width and height
  int                        m_framesToBeEncoded;                 ///< number of encoded frames
  bool                       m_AccessUnitDelimiter;               ///< add Access Unit Delimiter NAL units
  bool                       m_enablePictureHeaderInSliceHeader;  ///< Enable Picture Header in Slice Header
  InputColourSpaceConversion m_inputColourSpaceConvert;           ///< colour space conversion to apply to input video
  bool m_snrInternalColourSpace;     ///< if true, then no colour space conversion is applied for snr calculation,
                                     ///< otherwise inverse of input is applied.
  bool m_outputInternalColourSpace;  ///< if true, then no colour space conversion is applied for reconstructed video,
                                     ///< otherwise inverse of input is applied.
  ChromaFormat m_InputChromaFormatIDC;

  bool m_printMSEBasedSequencePSNR;
  bool m_printHexPsnr;
  bool m_printFrameMSE;
  bool m_printSequenceMSE;
  bool m_printMSSSIM;
  bool m_printWPSNR;
  bool m_cabacZeroWordPaddingEnabled;
  bool m_bClipInputVideoToRec709Range;
  bool m_bClipOutputVideoToRec709Range;
  bool m_packedYUVMode;  ///< If true, output 10-bit and 12-bit YUV data as 5-byte and 3-byte (respectively) packed YUV
                         ///< data

  bool     m_gciPresentFlag;
  bool     m_bIntraOnlyConstraintFlag;
  uint32_t m_maxBitDepthConstraintIdc;
  int      m_maxChromaFormatConstraintIdc;
  bool     m_allLayersIndependentConstraintFlag;
  bool     m_noMrlConstraintFlag;
  bool     m_noIspConstraintFlag;
  bool     m_noMipConstraintFlag;
  bool     m_noLfnstConstraintFlag;
  bool     m_noMmvdConstraintFlag;
  bool     m_noSmvdConstraintFlag;
  bool     m_noProfConstraintFlag;
  bool     m_noPaletteConstraintFlag;
  bool     m_noActConstraintFlag;
  bool     m_noLmcsConstraintFlag;
  bool     m_noExplicitScaleListConstraintFlag;
  bool     m_noVirtualBoundaryConstraintFlag;
  bool     m_noMttConstraintFlag;
  bool     m_noChromaQpOffsetConstraintFlag;
  bool     m_noQtbttDualTreeIntraConstraintFlag;
  int      m_maxLog2CtuSizeConstraintIdc;
  bool     m_noPartitionConstraintsOverrideConstraintFlag;
  bool     m_noSaoConstraintFlag;
  bool     m_noAlfConstraintFlag;
  bool     m_noCCAlfConstraintFlag;
  bool     m_noWeightedPredictionConstraintFlag;
  bool     m_noRefWraparoundConstraintFlag;
  bool     m_noTemporalMvpConstraintFlag;
  bool     m_noSbtmvpConstraintFlag;
  bool     m_noAmvrConstraintFlag;
  bool     m_noBdofConstraintFlag;
  bool     m_noDmvrConstraintFlag;
  bool     m_noCclmConstraintFlag;
  bool     m_noMtsConstraintFlag;
  bool     m_noSbtConstraintFlag;
  bool     m_noAffineMotionConstraintFlag;
  bool     m_noBcwConstraintFlag;
  bool     m_noIbcConstraintFlag;
  bool     m_noCiipConstraintFlag;
  bool     m_noGeoConstraintFlag;
  bool     m_noLadfConstraintFlag;
  bool     m_noTransformSkipConstraintFlag;
  bool     m_noLumaTransformSize64ConstraintFlag;
  bool     m_noBDPCMConstraintFlag;
  bool     m_noJointCbCrConstraintFlag;
  bool     m_noCuQpDeltaConstraintFlag;
  bool     m_noDepQuantConstraintFlag;
  bool     m_noSignDataHidingConstraintFlag;
  bool     m_noTrailConstraintFlag;
  bool     m_noStsaConstraintFlag;
  bool     m_noRaslConstraintFlag;
  bool     m_noRadlConstraintFlag;
  bool     m_noIdrConstraintFlag;
  bool     m_noCraConstraintFlag;
  bool     m_noGdrConstraintFlag;
  bool     m_noApsConstraintFlag;

  // profile/level
  Profile::Name         m_profile;
  Level::Tier           m_levelTier;
  Level::Name           m_level;
  bool                  m_frameOnlyConstraintFlag;
  bool                  m_multiLayerEnabledFlag;
  std::vector<uint32_t> m_subProfile;
  uint8_t               m_numSubProfile;

  uint32_t     m_bitDepthConstraint;
  ChromaFormat m_chromaFormatConstraint;
  bool         m_onePictureOnlyConstraintFlag;
  bool         m_intraOnlyConstraintFlag;
  bool         m_nonPackedConstraintFlag;
  bool         m_nonProjectedConstraintFlag;
  bool         m_noRprConstraintFlag;
  bool         m_noResChangeInClvsConstraintFlag;
  bool         m_oneTilePerPicConstraintFlag;
  bool         m_picHeaderInSliceHeaderConstraintFlag;
  bool         m_oneSlicePerPicConstraintFlag;
  bool         m_noIdrRplConstraintFlag;
  bool         m_noRectSliceConstraintFlag;
  bool         m_oneSlicePerSubpicConstraintFlag;
  bool         m_noSubpicInfoConstraintFlag;
  // coding structure
  int m_iIntraPeriod;  ///< period of I-slice (random access period)
#if GDR_ENABLED
  bool m_gdrEnabled;
  int  m_gdrPocStart;
  int  m_gdrPeriod;
  int  m_gdrInterval;
  bool m_gdrNoHash;
#endif
  int      m_iDecodingRefreshType;  ///< random access type
  int      m_iGOPSize;              ///< GOP size of hierarchical structure
  int      m_drapPeriod;            ///< period of dependent RAP pictures
  bool     m_rewriteParamSets;      ///< Flag to enable rewriting of parameter sets at random access points
  RPLEntry m_RPLList0[MAX_GOP];     ///< the RPL entries from the config file
  RPLEntry m_RPLList1[MAX_GOP];     ///< the RPL entries from the config file
  bool m_idrRefParamList;  ///< indicates if reference picture list syntax elements are present in slice headers of IDR
                           ///< pictures
  GOPEntry m_GOPList[MAX_GOP];                    ///< the coding structure entries from the config file
  int      m_maxNumReorderPics[MAX_TLAYER];       ///< total number of reorder pictures
  int      m_maxDecPicBuffering[MAX_TLAYER];      ///< total number of pictures in the decoded picture buffer
  bool     m_reconBasedCrossCPredictionEstimate;  ///< causes the alpha calculation in encoder search to be based on the
                                              ///< decoded residual rather than the pre-transform encoder-side residual
  bool     m_useTransformSkip;      ///< flag for enabling intra transform skipping
  bool     m_useTransformSkipFast;  ///< flag for enabling fast intra transform skipping
  bool     m_useBDPCM;
  uint32_t m_log2MaxTransformSkipBlockSize;     ///< transform-skip maximum size (minimum of 2)
  bool     m_transformSkipRotationEnabledFlag;  ///< control flag for transform-skip/transquant-bypass residual rotation
  bool m_transformSkipContextEnabledFlag;  ///< control flag for transform-skip/transquant-bypass single significance
                                           ///< map context
#if JVET_V0106_RRC_RICE
  bool m_rrcRiceExtensionEnableFlag;  ///< control flag for enabling extension of the Golomb-Rice parameter derivation
                                      ///< for RRC
#endif
  bool m_persistentRiceAdaptationEnabledFlag;  ///< control flag for Golomb-Rice parameter adaptation over each slice
  bool m_cabacBypassAlignmentEnabledFlag;
  bool m_ISP;
  bool m_useFastISP;  ///< flag for enabling fast methods for ISP

  // coding quality
#if QP_SWITCHING_FOR_PARALLEL
  OptionalValue<uint32_t> m_qpIncrementAtSourceFrame;  ///< Optional source frame number at which all subsequent frames
                                                       ///< are to use an increased internal QP.
#else
  double m_fQP;                     ///< QP value of key-picture (floating point)
#endif
  int                        m_iQP;  ///< QP value of key-picture (integer)
  bool                       m_useIdentityTableForNon420Chroma;
  ChromaQpMappingTableParams m_chromaQpMappingTableParams;
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  int  m_intraQPOffset;       ///< QP offset for intra slice (integer)
  bool m_lambdaFromQPEnable;  ///< enable flag for QP:lambda fix
#endif
  std::string              m_dQPFileName;              ///< QP offset for each slice (initialized from external file)
  int*                     m_aidQP;                    ///< array of slice QP values
  int                      m_iMaxDeltaQP;              ///< max. |delta QP|
  uint32_t                 m_uiDeltaQpRD;              ///< dQP range for multi-pass slice QP optimization
  int                      m_cuQpDeltaSubdiv;          ///< Maximum subdiv for CU luma Qp adjustment (0:default)
  int                      m_cuChromaQpOffsetSubdiv;   ///< If negative, then do not apply chroma qp offsets.
  std::vector<ChromaQpAdj> m_cuChromaQpOffsetList;     ///< Local chroma QP offsets list (to be signalled in PPS)
  bool                     m_cuChromaQpOffsetEnabled;  ///< Enable local chroma QP offsets (slice level flag)
  bool                     m_bFastDeltaQP;             ///< Fast Delta QP (false:default)

  int m_cbQpOffset;            ///< Chroma Cb QP Offset (0:default)
  int m_crQpOffset;            ///< Chroma Cr QP Offset (0:default)
  int m_cbQpOffsetDualTree;    ///< Chroma Cb QP Offset for dual tree (overwrite m_cbQpOffset for dual tree)
  int m_crQpOffsetDualTree;    ///< Chroma Cr QP Offset for dual tree (overwrite m_crQpOffset for dual tree)
  int m_cbCrQpOffset;          ///< QP Offset for joint Cb-Cr mode
  int m_cbCrQpOffsetDualTree;  ///< QP Offset for joint Cb-Cr mode (overwrite m_cbCrQpOffset for dual tree)
#if ER_CHROMA_QP_WCG_PPS
  WCGChromaQPControl m_wcgChromaQpControl;  ///< Wide-colour-gamut chroma QP control.
#endif
#if W0038_CQP_ADJ
  uint32_t m_sliceChromaQpOffsetPeriodicity;  ///< Used in conjunction with Slice Cb/Cr QpOffsetIntraOrPeriodic. Use 0
                                              ///< (default) to disable periodic nature.
  int m_sliceChromaQpOffsetIntraOrPeriodic
      [2 /*Cb,Cr*/];  ///< Chroma Cb QP Offset at slice level for I slice or for periodic inter slices as defined by
                      ///< SliceChromaQPOffsetPeriodicity. Replaces offset in the GOP table.
#endif
#if SHARP_LUMA_DELTA_QP
  LumaLevelToDeltaQPMapping m_lumaLevelToDeltaQPMapping;  ///< mapping from luma level to Delta QP.
#endif
  SEIMasteringDisplay m_masteringDisplay;
#if JVET_V0078
  bool   m_smoothQPReductionEnable;
  double m_smoothQPReductionThreshold;
  double m_smoothQPReductionModelScale;
  double m_smoothQPReductionModelOffset;
  int    m_smoothQPReductionLimit;
  int    m_smoothQPReductionPeriodicity;
#endif

  bool m_bUseAdaptiveQP;      ///< Flag for enabling QP adaptation based on a psycho-visual model
  int  m_iQPAdaptationRange;  ///< dQP range by QP adaptation
#if ENABLE_QPA
  bool m_bUsePerceptQPA;  ///< Flag to enable perceptually motivated input-adaptive QP modification
  bool m_bUseWPSNR;       ///< Flag to output perceptually weighted peak SNR (WPSNR) instead of PSNR
#endif
  int m_maxTempLayer;  ///< Max temporal layer

  // coding unit (CU) definition
  unsigned              m_uiCTUSize;
  bool                  m_subPicInfoPresentFlag;
  unsigned              m_numSubPics;
  bool                  m_subPicSameSizeFlag;
  std::vector<uint32_t> m_subPicCtuTopLeftX;
  std::vector<uint32_t> m_subPicCtuTopLeftY;
  std::vector<uint32_t> m_subPicWidth;
  std::vector<uint32_t> m_subPicHeight;
  std::vector<bool>     m_subPicTreatedAsPicFlag;
  std::vector<bool>     m_loopFilterAcrossSubpicEnabledFlag;
  bool                  m_subPicIdMappingExplicitlySignalledFlag;
  bool                  m_subPicIdMappingInSpsFlag;
  unsigned              m_subPicIdLen;
  std::vector<uint16_t> m_subPicId;
  bool                  m_SplitConsOverrideEnabledFlag;
  unsigned              m_uiMinQT[3];  // 0: I slice luma; 1: P/B slice; 2: I slice chroma
  unsigned              m_uiMaxMTTHierarchyDepth;
  unsigned              m_uiMaxMTTHierarchyDepthI;
  unsigned              m_uiMaxMTTHierarchyDepthIChroma;
  unsigned              m_uiMaxBT[3];
  unsigned              m_uiMaxTT[3];
  bool                  m_dualTree;
  bool                  m_LFNST;
  bool                  m_useFastLFNST;
  bool                  m_sbTmvpEnableFlag;
  bool                  m_Affine;
  bool                  m_AffineType;
  bool                  m_PROF;
  bool                  m_BIO;
  int                   m_LMChroma;
  bool                  m_horCollocatedChromaFlag;
  bool                  m_verCollocatedChromaFlag;
  int                   m_MTS;              ///< XZ: Multiple Transform Set
  int                   m_MTSIntraMaxCand;  ///< XZ: Number of additional candidates to test
  int                   m_MTSInterMaxCand;  ///< XZ: Number of additional candidates to test
  int                   m_MTSImplicit;
  bool                  m_SBT;  ///< Sub-Block Transform for inter blocks
  int                   m_SBTFast64WidthTh;
  bool                  m_SMVD;
  bool                  m_compositeRefEnabled;
  bool                  m_bcw;
  bool                  m_BcwFast;
#if LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET
  bool             m_LadfEnabed;
  int              m_LadfNumIntervals;
  std::vector<int> m_LadfQpOffset;
  int              m_LadfIntervalLowerBound[MAX_LADF_INTERVALS];
#endif

  bool     m_ciip;
  bool     m_Geo;
  bool     m_HashME;
  bool     m_allowDisFracMMVD;
  bool     m_AffineAmvr;
  bool     m_AffineAmvrEncOpt;
  bool     m_DMVR;
  bool     m_MMVD;
  int      m_MmvdDisNum;
  bool     m_rgbFormat;
  bool     m_useColorTrans;
  unsigned m_PLTMode;
  bool     m_JointCbCrMode;
  bool     m_useChromaTS;
  unsigned m_IBCMode;
  unsigned m_IBCLocalSearchRangeX;
  unsigned m_IBCLocalSearchRangeY;
  unsigned m_IBCHashSearch;
  unsigned m_IBCHashSearchMaxCand;
  unsigned m_IBCHashSearchRange4SmallBlk;
  unsigned m_IBCFastMethod;

  bool     m_wrapAround;
  unsigned m_wrapAroundOffset;

  // ADD_NEW_TOOL : (encoder app) add tool enabling flags and associated parameters here
  bool                  m_virtualBoundariesEnabledFlag;
  bool                  m_virtualBoundariesPresentFlag;
  unsigned              m_numVerVirtualBoundaries;
  unsigned              m_numHorVirtualBoundaries;
  std::vector<unsigned> m_virtualBoundariesPosX;
  std::vector<unsigned> m_virtualBoundariesPosY;
  bool                  m_lmcsEnabled;
  uint32_t              m_reshapeSignalType;
  uint32_t              m_intraCMD;
  ReshapeCW             m_reshapeCW;
  int                   m_updateCtrl;
  int                   m_adpOption;
  uint32_t              m_initialCW;
  int                   m_CSoffset;
  bool                  m_encDbOpt;
  unsigned              m_uiMaxCUWidth;   ///< max. CU width in pixel
  unsigned              m_uiMaxCUHeight;  ///< max. CU height in pixel
  unsigned              m_log2MinCuSize;  ///< min. CU size log2

  bool     m_useFastLCTU;
  bool     m_usePbIntraFast;
  bool     m_useAMaxBT;
  bool     m_useFastMrg;
  bool     m_e0023FastEnc;
  bool     m_contentBasedFastQtbt;
  bool     m_useNonLinearAlfLuma;
  bool     m_useNonLinearAlfChroma;
  unsigned m_maxNumAlfAlternativesChroma;
  bool     m_MRL;
  bool     m_MIP;
  bool     m_useFastMIP;
  int      m_fastLocalDualTreeMode;

  int m_log2MaxTbSize;
  // coding tools (bit-depth)
  int m_inputBitDepth[MAX_NUM_CHANNEL_TYPE];        ///< bit-depth of input file
  int m_outputBitDepth[MAX_NUM_CHANNEL_TYPE];       ///< bit-depth of output file
  int m_MSBExtendedBitDepth[MAX_NUM_CHANNEL_TYPE];  ///< bit-depth of input samples after MSB extension
  int m_internalBitDepth[MAX_NUM_CHANNEL_TYPE];  ///< bit-depth codec operates at (input/output files will be converted)
  bool m_extendedPrecisionProcessingFlag;
#if JVET_V0054_TSRC_RICE
  bool m_tsrcRicePresentFlag;
#endif
  bool m_highPrecisionOffsetsEnabledFlag;

  // coding tools (chroma format)
  ChromaFormat m_chromaFormatIDC;

  // coding tool (SAO)
  bool   m_bUseSAO;
  bool   m_bTestSAODisableAtPictureLevel;
  double m_saoEncodingRate;        ///< When >0 SAO early picture termination is enabled for luma and chroma
  double m_saoEncodingRateChroma;  ///< The SAO early picture termination rate to use for chroma (when m_SaoEncodingRate
                                   ///< is >0). If <=0, use results for luma.
  int  m_maxNumOffsetsPerPic;      ///< SAO maximun number of offset per picture
  bool m_saoCtuBoundary;     ///< SAO parameter estimation using non-deblocked pixels for CTU bottom and right boundary
                             ///< areas
  bool m_saoGreedyMergeEnc;  ///< SAO greedy merge encoding algorithm
  // coding tools (loop filter)
  bool m_deblockingFilterDisable;           ///< flag for using deblocking filter
  bool m_deblockingFilterOffsetInPPS;       ///< offset for deblocking filter in 0 = slice header, 1 = PPS
  int  m_deblockingFilterBetaOffsetDiv2;    ///< beta offset for deblocking filter
  int  m_deblockingFilterTcOffsetDiv2;      ///< tc offset for deblocking filter
  int  m_deblockingFilterCbBetaOffsetDiv2;  ///< beta offset for Cb deblocking filter
  int  m_deblockingFilterCbTcOffsetDiv2;    ///< tc offset for Cb deblocking filter
  int  m_deblockingFilterCrBetaOffsetDiv2;  ///< beta offset for Cr deblocking filter
  int  m_deblockingFilterCrTcOffsetDiv2;    ///< tc offset for Cr deblocking filter
#if W0038_DB_OPT
  int m_deblockingFilterMetric;  ///< blockiness metric in encoder
#else
  bool   m_DeblockingFilterMetric;  ///< blockiness metric in encoder
#endif
  bool m_enableIntraReferenceSmoothing;  ///< flag for enabling(default)/disabling intra reference smoothing/filtering

  // coding tools (encoder-only parameters)
  bool m_bUseASR;    ///< flag for using adaptive motion search range
  bool m_bUseHADME;  ///< flag for using HAD in sub-pel ME
  bool m_useRDOQ;    ///< flag for using RD optimized quantization
  bool m_useRDOQTS;  ///< flag for using RD optimized quantization for transform skip
#if T0196_SELECTIVE_RDOQ
  bool m_useSelectiveRDOQ;  ///< flag for using selective RDOQ
#endif
  int m_rdPenalty;  ///< RD-penalty for 32x32 TU for intra in non-intra slices (0: no RD-penalty, 1: RD-penalty, 2:
                    ///< maximum RD-penalty)
  bool                m_bDisableIntraPUsInInterSlices;  ///< Flag for disabling intra predicted PUs in inter slices.
  MESearchMethod      m_motionEstimationSearchMethod;
  bool                m_bRestrictMESampling;               ///< Restrict sampling for the Selective ME
  int                 m_iSearchRange;                      ///< ME search range
  int                 m_bipredSearchRange;                 ///< ME search range for bipred refinement
  int                 m_minSearchWindow;                   ///< ME minimum search window size for the Adaptive Window ME
  bool                m_bClipForBiPredMeEnabled;           ///< Enables clipping for Bi-Pred ME.
  bool                m_bFastMEAssumingSmootherMVEnabled;  ///< Enables fast ME assuming a smoother MV.
  FastInterSearchMode m_fastInterSearchMode;               ///< Parameter that controls fast encoder settings
  bool                m_bUseEarlyCU;                       ///< flag for using Early CU setting
  bool                m_useFastDecisionForMerge;           ///< flag for using Fast Decision Merge RD-Cost
  bool                m_useEarlySkipDetection;             ///< flag for using Early SKIP Detection
  bool m_picPartitionFlag;  ///< enable picture partitioning (0: single tile, single slice, 1: multiple tiles/slices can
                            ///< be used)
  bool                  m_mixedLossyLossless;  ///< enable mixed lossy/lossless coding
  std::vector<uint16_t> m_sliceLosslessArray;  ///< Slice lossless array
  std::vector<uint32_t> m_tileColumnWidth;  ///< tile column widths in units of CTUs (last column width will be repeated
                                            ///< uniformly to cover any remaining picture width)
  std::vector<uint32_t> m_tileRowHeight;    ///< tile row heights in units of CTUs (last row height will be repeated
                                            ///< uniformly to cover any remaining picture height)
  bool m_rasterSliceFlag;  ///< indicates if using raster-scan or rectangular slices (0: rectangular, 1: raster-scan)
  std::vector<uint32_t> m_rectSlicePos;  ///< rectangular slice positions (pairs of top-left CTU address followed by
                                         ///< bottom-right CTU address)
  int m_rectSliceFixedWidth;   ///< fixed rectangular slice width in units of tiles (0: disable this feature and use
                               ///< RectSlicePositions instead)
  int m_rectSliceFixedHeight;  ///< fixed rectangular slice height in units of tiles (0: disable this feature and use
                               ///< RectSlicePositions instead)
  std::vector<uint32_t> m_rasterSliceSize;  ///< raster-scan slice sizes in units of tiles (last size will be repeated
                                            ///< uniformly to cover any remaining tiles in the picture)
  bool m_disableLFCrossTileBoundaryFlag;  ///< 0: filter across tile boundaries  1: do not filter across tile boundaries
  bool m_disableLFCrossSliceBoundaryFlag;  ///< 0: filter across slice boundaries 1: do not filter across slice
                                           ///< boundaries
  uint32_t m_numSlicesInPic;  ///< derived number of rectangular slices in the picture (raster-scan slice specified at
                              ///< slice level)
  bool                   m_tileIdxDeltaPresentFlag;  ///< derived tile index delta present flag
  std::vector<RectSlice> m_rectSlices;               ///< derived list of rectangular slice signalling parameters
  uint32_t               m_numTileCols;              ///< derived number of tile columns
  uint32_t               m_numTileRows;              ///< derived number of tile rows
  bool                   m_singleSlicePerSubPicFlag;
  bool                   m_entropyCodingSyncEnabledFlag;
  bool                   m_entryPointPresentFlag;  ///< flag for the presence of entry points

  bool m_bFastUDIUseMPMEnabled;
  bool m_bFastMEForGenBLowDelayEnabled;
  bool m_bUseBLambdaForNonKeyLowDelayPictures;

  HashType m_decodedPictureHashSEIType;  ///< Checksum mode for decoded picture hash SEI message
  HashType m_subpicDecodedPictureHashType;
  bool     m_bufferingPeriodSEIEnabled;
  bool     m_pictureTimingSEIEnabled;
  bool     m_bpDeltasGOPStructure;
  bool     m_decodingUnitInfoSEIEnabled;
  bool     m_scalableNestingSEIEnabled;
  bool     m_frameFieldInfoSEIEnabled;
  bool     m_framePackingSEIEnabled;
  int      m_framePackingSEIType;
  int      m_framePackingSEIId;
  int      m_framePackingSEIQuincunx;
  int      m_framePackingSEIInterpretation;
  bool     m_parameterSetsInclusionIndicationSEIEnabled;
  int      m_selfContainedClvsFlag;
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
  int m_preferredTransferCharacteristics;
#endif
  // film grain characterstics sei
  bool     m_fgcSEIEnabled;
  bool     m_fgcSEICancelFlag;
  bool     m_fgcSEIPersistenceFlag;
  uint32_t m_fgcSEIModelID;
  bool     m_fgcSEISepColourDescPresentFlag;
  uint32_t m_fgcSEIBlendingModeID;
  uint32_t m_fgcSEILog2ScaleFactor;
  bool     m_fgcSEICompModelPresent[MAX_NUM_COMPONENT];
  // content light level SEI
  bool     m_cllSEIEnabled;
  uint32_t m_cllSEIMaxContentLevel;
  uint32_t m_cllSEIMaxPicAvgLevel;
  // ambient viewing environment sei
  bool     m_aveSEIEnabled;
  uint32_t m_aveSEIAmbientIlluminance;
  uint32_t m_aveSEIAmbientLightX;
  uint32_t m_aveSEIAmbientLightY;
  // content colour volume sei
  bool   m_ccvSEIEnabled;
  bool   m_ccvSEICancelFlag;
  bool   m_ccvSEIPersistenceFlag;
  bool   m_ccvSEIPrimariesPresentFlag;
  bool   m_ccvSEIMinLuminanceValuePresentFlag;
  bool   m_ccvSEIMaxLuminanceValuePresentFlag;
  bool   m_ccvSEIAvgLuminanceValuePresentFlag;
  double m_ccvSEIPrimariesX[MAX_NUM_COMPONENT];
  double m_ccvSEIPrimariesY[MAX_NUM_COMPONENT];
  double m_ccvSEIMinLuminanceValue;
  double m_ccvSEIMaxLuminanceValue;
  double m_ccvSEIAvgLuminanceValue;

  bool     m_erpSEIEnabled;
  bool     m_erpSEICancelFlag;
  bool     m_erpSEIPersistenceFlag;
  bool     m_erpSEIGuardBandFlag;
  uint32_t m_erpSEIGuardBandType;
  uint32_t m_erpSEILeftGuardBandWidth;
  uint32_t m_erpSEIRightGuardBandWidth;

  bool m_sphereRotationSEIEnabled;
  bool m_sphereRotationSEICancelFlag;
  bool m_sphereRotationSEIPersistenceFlag;
  int  m_sphereRotationSEIYaw;
  int  m_sphereRotationSEIPitch;
  int  m_sphereRotationSEIRoll;

  bool                  m_omniViewportSEIEnabled;
  uint32_t              m_omniViewportSEIId;
  bool                  m_omniViewportSEICancelFlag;
  bool                  m_omniViewportSEIPersistenceFlag;
  uint32_t              m_omniViewportSEICntMinus1;
  std::vector<int>      m_omniViewportSEIAzimuthCentre;
  std::vector<int>      m_omniViewportSEIElevationCentre;
  std::vector<int>      m_omniViewportSEITiltCentre;
  std::vector<uint32_t> m_omniViewportSEIHorRange;
  std::vector<uint32_t> m_omniViewportSEIVerRange;
  std::string           m_arSEIFileRoot;  // Annotated region SEI - initialized from external file
  bool                  m_rwpSEIEnabled;
  bool                  m_rwpSEIRwpCancelFlag;
  bool                  m_rwpSEIRwpPersistenceFlag;
  bool                  m_rwpSEIConstituentPictureMatchingFlag;
  int                   m_rwpSEINumPackedRegions;
  int                   m_rwpSEIProjPictureWidth;
  int                   m_rwpSEIProjPictureHeight;
  int                   m_rwpSEIPackedPictureWidth;
  int                   m_rwpSEIPackedPictureHeight;
  std::vector<uint8_t>  m_rwpSEIRwpTransformType;
  std::vector<bool>     m_rwpSEIRwpGuardBandFlag;
  std::vector<uint32_t> m_rwpSEIProjRegionWidth;
  std::vector<uint32_t> m_rwpSEIProjRegionHeight;
  std::vector<uint32_t> m_rwpSEIRwpSEIProjRegionTop;
  std::vector<uint32_t> m_rwpSEIProjRegionLeft;
  std::vector<uint16_t> m_rwpSEIPackedRegionWidth;
  std::vector<uint16_t> m_rwpSEIPackedRegionHeight;
  std::vector<uint16_t> m_rwpSEIPackedRegionTop;
  std::vector<uint16_t> m_rwpSEIPackedRegionLeft;
  std::vector<uint8_t>  m_rwpSEIRwpLeftGuardBandWidth;
  std::vector<uint8_t>  m_rwpSEIRwpRightGuardBandWidth;
  std::vector<uint8_t>  m_rwpSEIRwpTopGuardBandHeight;
  std::vector<uint8_t>  m_rwpSEIRwpBottomGuardBandHeight;
  std::vector<bool>     m_rwpSEIRwpGuardBandNotUsedForPredFlag;
  std::vector<uint8_t>  m_rwpSEIRwpGuardBandType;

  bool                 m_gcmpSEIEnabled;
  bool                 m_gcmpSEICancelFlag;
  bool                 m_gcmpSEIPersistenceFlag;
  uint32_t             m_gcmpSEIPackingType;
  uint32_t             m_gcmpSEIMappingFunctionType;
  std::vector<uint8_t> m_gcmpSEIFaceIndex;
  std::vector<uint8_t> m_gcmpSEIFaceRotation;
  std::vector<double>  m_gcmpSEIFunctionCoeffU;
  std::vector<bool>    m_gcmpSEIFunctionUAffectedByVFlag;
  std::vector<double>  m_gcmpSEIFunctionCoeffV;
  std::vector<bool>    m_gcmpSEIFunctionVAffectedByUFlag;
  bool                 m_gcmpSEIGuardBandFlag;
  uint32_t             m_gcmpSEIGuardBandType;
  bool                 m_gcmpSEIGuardBandBoundaryExteriorFlag;
  uint32_t             m_gcmpSEIGuardBandSamplesMinus1;

  CfgSEISubpictureLevel m_cfgSubpictureLevelInfoSEI;

  bool m_sampleAspectRatioInfoSEIEnabled;
  bool m_sariCancelFlag;
  bool m_sariPersistenceFlag;
  int  m_sariAspectRatioIdc;
  int  m_sariSarWidth;
  int  m_sariSarHeight;

  bool m_MCTSEncConstraint;

  // weighted prediction
  bool                     m_useWeightedPred;    ///< Use of weighted prediction in P slices
  bool                     m_useWeightedBiPred;  ///< Use of bi-directional weighted prediction in B slices
  WeightedPredictionMethod m_weightedPredictionMethod;

  uint32_t m_log2ParallelMergeLevel;  ///< Parallel merge estimation region
  uint32_t m_maxNumMergeCand;         ///< Max number of merge candidates
  uint32_t m_maxNumAffineMergeCand;   ///< Max number of affine merge candidates
  uint32_t m_maxNumGeoCand;
  uint32_t m_maxNumIBCMergeCand;  ///< Max number of IBC merge candidates

  bool m_sliceLevelRpl;      ///< code reference picture lists in slice headers rather than picture header
  bool m_sliceLevelDblk;     ///< code deblocking filter parameters in slice headers rather than picture header
  bool m_sliceLevelSao;      ///< code SAO parameters in slice headers rather than picture header
  bool m_sliceLevelAlf;      ///< code ALF parameters in slice headers rather than picture header
  bool m_sliceLevelWp;       ///< code weighted prediction parameters in slice headers rather than picture header
  bool m_sliceLevelDeltaQp;  ///< code delta in slice headers rather than picture header

  int  m_TMVPModeId;
  bool m_depQuantEnabledFlag;
  bool m_signDataHidingEnabledFlag;
  bool m_RCEnableRateControl;    ///< enable rate control or not
  int  m_RCTargetBitrate;        ///< target bitrate when rate control is enabled
  int  m_RCKeepHierarchicalBit;  ///< 0: equal bit allocation; 1: fixed ratio bit allocation; 2: adaptive ratio bit
                                 ///< allocation
  bool m_RCLCULevelRC;  ///< true: LCU level rate control; false: picture level rate control NOTE: code-tidy - rename to
                        ///< m_RCCtuLevelRC
  bool m_RCUseLCUSeparateModel;  ///< use separate R-lambda model at LCU level                        NOTE: code-tidy -
                                 ///< rename to m_RCUseCtuSeparateModel
  int  m_RCInitialQP;            ///< inital QP for rate control
  bool m_RCForceIntraQP;         ///< force all intra picture to use initial QP or not
#if U0132_TARGET_BITS_SATURATION
  bool     m_RCCpbSaturationEnabled;  ///< enable target bits saturation to avoid CPB overflow and underflow
  uint32_t m_RCCpbSize;               ///< CPB size
  double   m_RCInitialCpbFullness;    ///< initial CPB fullness
#endif
  ScalingListMode m_useScalingListId;     ///< using quantization matrix
  std::string     m_scalingListFileName;  ///< quantization matrix file name
  bool            m_disableScalingMatrixForLfnstBlks;
  bool            m_disableScalingMatrixForAlternativeColourSpace;
  bool            m_scalingMatrixDesignatedColourSpace;
  CostMode        m_costMode;       ///< Cost mode to use
  bool            m_TSRCdisableLL;  ///< disable TSRC for lossless

  bool m_recalculateQPAccordingToLambda;  ///< recalculate QP value according to the lambda value

  bool m_DCIEnabled;                    ///< enable Decoding Capability Information (DCI)
  bool m_hrdParametersPresentFlag;      ///< enable generation of HRD parameters
  bool m_vuiParametersPresentFlag;      ///< enable generation of VUI parameters
  bool m_samePicTimingInAllOLS;         ///< same picture timing SEI message is used in all OLS
  bool m_aspectRatioInfoPresentFlag;    ///< Signals whether aspect_ratio_idc is present
  int  m_aspectRatioIdc;                ///< aspect_ratio_idc
  int  m_sarWidth;                      ///< horizontal size of the sample aspect ratio
  int  m_sarHeight;                     ///< vertical size of the sample aspect ratio
  bool m_colourDescriptionPresentFlag;  ///< Signals whether colour_primaries, transfer_characteristics and
                                        ///< matrix_coefficients are present
  int  m_colourPrimaries;               ///< Indicates chromaticity coordinates of the source primaries
  int  m_transferCharacteristics;       ///< Indicates the opto-electronic transfer characteristics of the source
  int  m_matrixCoefficients;  ///< Describes the matrix coefficients used in deriving luma and chroma from RGB primaries
  bool m_progressiveSourceFlag;           ///< Indicates if the content is progressive
  bool m_interlacedSourceFlag;            ///< Indicates if the content is interlaced
  bool m_chromaLocInfoPresentFlag;        ///< Signals whether chroma_sample_loc_type_top_field and
                                          ///< chroma_sample_loc_type_bottom_field are present
  int  m_chromaSampleLocTypeTopField;     ///< Specifies the location of chroma samples for top field
  int  m_chromaSampleLocTypeBottomField;  ///< Specifies the location of chroma samples for bottom field
  int  m_chromaSampleLocType;             ///< Specifies the location of chroma samples for progressive content
  bool m_overscanInfoPresentFlag;         ///< Signals whether overscan_appropriate_flag is present
  bool m_overscanAppropriateFlag;  ///< Indicates whether conformant decoded pictures are suitable for display using
                                   ///< overscan
  bool m_videoFullRangeFlag;       ///< Indicates the black level and range of luma and chroma signals
  int  m_ImvMode;                  ///< imv mode
  int  m_Imv4PelFast;              ///< imv 4-Pel fast mode

  std::string m_summaryOutFilename;      ///< filename to use for producing summary output file.
  std::string m_summaryPicFilenameBase;  ///< Base filename to use for producing summary picture output files. The
                                         ///< actual filenames used will have I.txt, P.txt and B.txt appended.
  uint32_t m_summaryVerboseness;         ///< Specifies the level of the verboseness of the text output.

  int m_verbosity;

  std::string m_decodeBitstreams[2];  ///< filename for decode bitstreams.
  int         m_debugCTU;
  int         m_switchPOC;  ///< dbg poc.
  int         m_switchDQP;  ///< switch DQP.
  int m_fastForwardToPOC;   ///< get to encoding the specified POC as soon as possible by skipping temporal layers
                            ///< irrelevant for the specified POC
  bool m_stopAfterFFtoPOC;
  bool m_bs2ModPOCAndType;
  bool m_forceDecodeBitstream1;

  bool m_alf;  ///< Adaptive Loop Filter
#if JVET_V0095_ALF_SAO_TRUE_ORG
  bool m_alfSaoTrueOrg;
#endif
  double m_alfStrengthLuma;
  bool   m_alfAllowPredefinedFilters;
  double m_ccalfStrength;
  double m_alfStrengthChroma;
  double m_alfStrengthTargetLuma;
  double m_alfStrengthTargetChroma;
  double m_ccalfStrengthTarget;
  bool   m_ccalf;
  int    m_ccalfQpThreshold;

  bool   m_rprEnabledFlag;
  double m_scalingRatioHor;
  double m_scalingRatioVer;
  bool   m_resChangeInClvsEnabled;
  double m_fractionOfFrames;  ///< encode a fraction of the frames as specified in FramesToBeEncoded
  int    m_switchPocPeriod;
  int m_upscaledOutput;  ////< Output upscaled (2), decoded cropped but in full resolution buffer (1) or decoded cropped
                         ///(0, default) picture for RPR.
  bool m_avoidIntraInDepLayer;

  bool m_gopBasedTemporalFilterEnabled;          ///< GOP-based Temporal Filter enable/disable
  bool m_gopBasedTemporalFilterFutureReference;  ///< Enable/disable future frame references in the GOP-based Temporal
                                                 ///< Filter
  std::map<int, double>
      m_gopBasedTemporalFilterStrengths;  ///< Filter strength per frame for the GOP-based Temporal Filter

  int         m_maxLayers;
  int         m_targetOlsIdx;
  bool        m_OPIEnabled;  ///< enable Operating Point Information (OPI)
  int         m_maxTemporalLayer;
  int         m_layerId[MAX_VPS_LAYERS];
  int         m_layerIdx;
  int         m_maxSublayers;
  bool        m_defaultPtlDpbHrdMaxTidFlag;
  bool        m_allIndependentLayersFlag;
  std::string m_predDirectionArray;

  int         m_numRefLayers[MAX_VPS_LAYERS];
  std::string m_refLayerIdxStr[MAX_VPS_LAYERS];
  bool        m_eachLayerIsAnOlsFlag;
  int         m_olsModeIdc;
  int         m_numOutputLayerSets;
  std::string m_olsOutputLayerStr[MAX_VPS_LAYERS];
  std::string m_maxTidILRefPicsPlus1Str[MAX_VPS_LAYERS];

  int m_numPtlsInVps;

  CfgVPSParameters m_cfgVPSParameters;
  Level::Name      m_levelPtl[MAX_NUM_OLSS];
  int              m_olsPtlIdx[MAX_NUM_OLSS];

#if EXTENSION_360_VIDEO
  TExt360AppEncCfg m_ext360;
  friend class TExt360AppEncCfg;
  friend class TExt360AppEncTop;
#endif

#if JVET_O0756_CONFIG_HDRMETRICS || JVET_O0756_CALCULATE_HDRMETRICS
#if JVET_O0756_CALCULATE_HDRMETRICS
  double m_whitePointDeltaE[hdrtoolslib::NB_REF_WHITE];
#else
  double m_whitePointDeltaE[3];
#endif
  double m_maxSampleValue;
  int    m_sampleRange;
  int    m_colorPrimaries;
  bool   m_enableTFunctionLUT;
  int    m_chromaLocation;
  int    m_chromaUPFilter;
  int    m_cropOffsetLeft;
  int    m_cropOffsetTop;
  int    m_cropOffsetRight;
  int    m_cropOffsetBottom;
  bool   m_calculateHdrMetrics;
#endif

  // internal member functions
  bool xCheckParameter();        ///< check validity of configuration values
  void xPrintParameter();        ///< print configuration values
  void xPrintUsage();            ///< print usage
  bool xHasNonZeroTemporalID();  ///< check presence of constant temporal ID in GOP structure
  bool xHasLeadingPicture();     ///< check presence of leading pictures in GOP structure
  int xAutoDetermineProfile();  ///< auto determine the profile to use given the other configuration settings. Returns 1
                                ///< if erred. Can select profile 'NONE'
 public:
  PCCVTMLibVideoEncoderCfg();
  virtual ~PCCVTMLibVideoEncoderCfg();

 public:
  void create();                            ///< create option handling class
  void destroy();                           ///< destroy option handling class
  bool parseCfg( int argc, char* argv[] );  ///< parse configuration file to fill member variables

};  // END CLASS DEFINITION EncAppCfg

//! \}
#endif

#endif  // __ENCAPPCFG__
