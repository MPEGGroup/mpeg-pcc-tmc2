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
#ifndef PCCHMLibVideoEncoderCfg_h
#define PCCHMLibVideoEncoderCfg_h

#include "PCCCommon.h"

#ifdef USE_HMLIB_VIDEO_CODEC

#include "TLibCommon/CommonDef.h"

#include "TLibEncoder/TEncCfg.h"
#if EXTENSION_360_VIDEO
#include "TAppEncHelper360/TExt360AppEncCfg.h"
#endif
#include <sstream>
#include <vector>
//! \ingroup TAppEncoder
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================
using namespace pcc_hm;

/// encoder configuration class
class PCCHMLibVideoEncoderCfg {
#if JVET_E0059_FLOATING_POINT_QP_FIX
 public:
  template <class T>
  struct OptionalValue {
    Bool bPresent;
    T    value;
    OptionalValue() : bPresent( false ), value() {}
  };
#endif

 protected:
  // file I/O
  std::string m_inputFileName;      ///< source file name
  std::string m_bitstreamFileName;  ///< output bitstream file
  std::string m_reconFileName;      ///< output reconstruction file

#if defined( PCC_ME_EXT ) && PCC_ME_EXT
  Bool        m_usePCCExt;
  std::string m_blockToPatchFileName;
  std::string m_occupancyMapFileName;
  std::string m_patchInfoFileName;
#endif
#if defined( PCC_RDO_EXT ) && PCC_RDO_EXT
  Bool m_usePCCRDO;
#if !PCC_ME_EXT
  std::string m_occupancyMapFileName;
#endif
#endif
  // Lambda modifiers
  Double m_adLambdaModifier[MAX_TLAYER];        ///< Lambda modifier array for each
                                                /// temporal layer
  std::vector<Double> m_adIntraLambdaModifier;  ///< Lambda modifier for Intra
                                                /// pictures, one for each
                                                /// temporal layer. If
                                                /// size>temporalLayer, then use
                                                ///[temporalLayer], else if
                                                /// size>0, use [size()-1], else
                                                /// use m_adLambdaModifier.
  Double m_dIntraQpFactor;                      ///< Intra Q Factor. If negative, use a default
                                                /// equation: 0.57*(1.0 - Clip3( 0.0, 0.5,
                                                /// 0.05*(Double)(isField ? (GopSize-1)/2 :
                                                /// GopSize-1) ))

  // source specification
  Int  m_iFrameRate;              ///< source frame-rates (Hz)
  UInt m_FrameSkip;               ///< number of skipped frames from the beginning
  UInt m_temporalSubsampleRatio;  ///< temporal subsample ratio, 2 means code
                                  /// every two frames
  Int m_iSourceWidth;             ///< source width in pixel
  Int m_iSourceHeight;            ///< source height in pixel (when interlaced = field
                                  /// height)
  Int m_inputFileWidth;           ///< width of image in input file  (this is equivalent
                                  /// to sourceWidth,  if sourceWidth  is not subsequently
                                  /// altered due to padding)
  Int m_inputFileHeight;          ///< height of image in input file (this is equivalent
                                  /// to sourceHeight, if sourceHeight is not
                                  /// subsequently altered due to padding)

  Int m_iSourceHeightOrg;  ///< original source height in pixel (when interlaced
                           ///= frame height)

  Bool m_isField;  ///< enable field coding
  Bool m_isTopFieldFirst;
  Bool m_bEfficientFieldIRAPEnabled;  ///< enable an efficient field IRAP
                                      /// structure.
  Bool m_bHarmonizeGopFirstFieldCoupleEnabled;

  Int                        m_conformanceWindowMode;
  Int                        m_confWinLeft;
  Int                        m_confWinRight;
  Int                        m_confWinTop;
  Int                        m_confWinBottom;
  Int                        m_framesToBeEncoded;        ///< number of encoded frames
  Int                        m_aiPad[2];                 ///< number of padded pixels for width and height
  Bool                       m_AccessUnitDelimiter;      ///< add Access Unit Delimiter NAL units
  InputColourSpaceConversion m_inputColourSpaceConvert;  ///< colour space
                                                         /// conversion to apply
                                                         /// to input video
  Bool m_snrInternalColourSpace;                         ///< if true, then no colour space conversion
                                                         /// is applied for snr calculation, otherwise
                                                         /// inverse of input is applied.
  Bool m_outputInternalColourSpace;                      ///< if true, then no colour space
                                                         /// conversion is applied for reconstructed
                                                         /// video, otherwise inverse of input is
                                                         /// applied.
  ChromaFormat m_InputChromaFormatIDC;

  Bool m_printMSEBasedSequencePSNR;
  Bool m_printHexPsnr;
  Bool m_printFrameMSE;
  Bool m_printSequenceMSE;
#if JVET_F0064_MSSSIM
  Bool m_printMSSSIM;
#endif

#if JCTVC_Y0037_XPSNR
  Bool   m_bXPSNREnableFlag;                 ///< xPSNR enable flag
  Double m_dXPSNRWeight[MAX_NUM_COMPONENT];  ///< xPSNR per component weights
#endif

  Bool m_cabacZeroWordPaddingEnabled;
  Bool m_bClipInputVideoToRec709Range;
  Bool m_bClipOutputVideoToRec709Range;

  // profile/level
  Profile::Name m_profile;
  Level::Tier   m_levelTier;
  Level::Name   m_level;
  UInt          m_bitDepthConstraint;
  ChromaFormat  m_chromaFormatConstraint;
  Bool          m_intraConstraintFlag;
  Bool          m_onePictureOnlyConstraintFlag;
  Bool          m_lowerBitRateConstraintFlag;
  Bool          m_progressiveSourceFlag;
  Bool          m_interlacedSourceFlag;
  Bool          m_nonPackedConstraintFlag;
  Bool          m_frameOnlyConstraintFlag;
  Bool          m_sccHighThroughputFlag;

  // coding structure
  Int m_iIntraPeriod;          ///< period of I-slice (random access period)
  Int m_iDecodingRefreshType;  ///< random access type
  Int m_iGOPSize;              ///< GOP size of hierarchical structure
#if JCTVC_Y0038_PARAMS
  Bool m_bReWriteParamSetsFlag;  ///< Flag to enable rewriting of parameter sets
                                 /// at random access points
#endif
  Int      m_extraRPSs;                                       ///< extra RPSs added to handle CRA
  GOPEntry m_GOPList[MAX_GOP];                                ///< the coding structure entries from the
                                                              /// config file
  Int m_numReorderPics[MAX_TLAYER];                           ///< total number of reorder pictures
  Int m_maxDecPicBuffering[MAX_TLAYER];                       ///< total number of pictures in the
                                                              /// decoded picture buffer
  Bool m_crossComponentPredictionEnabledFlag;                 ///< flag enabling the use of
                                                              /// cross-component prediction
  Bool m_reconBasedCrossCPredictionEstimate;                  ///< causes the alpha calculation
                                                              /// in encoder search to be based
                                                              /// on the decoded residual rather
                                                              /// than the pre-transform
                                                              /// encoder-side residual
  UInt m_log2SaoOffsetScale[MAX_NUM_CHANNEL_TYPE];            ///< number of bits for the
                                                              /// upward bit shift
                                                              /// operation on the decoded
                                                              /// SAO offsets
  Bool m_useTransformSkip;                                    ///< flag for enabling intra transform skipping
  Bool m_useTransformSkipFast;                                ///< flag for enabling fast intra transform
                                                              /// skipping
  UInt m_log2MaxTransformSkipBlockSize;                       ///< transform-skip maximum size
                                                              ///(minimum of 2)
  Bool m_transformSkipRotationEnabledFlag;                    ///< control flag for
                                                              /// transform-skip/transquant-bypass
                                                              /// residual rotation
  Bool m_transformSkipContextEnabledFlag;                     ///< control flag for
                                                              /// transform-skip/transquant-bypass
                                                              /// single significance map context
  Bool m_rdpcmEnabledFlag[NUMBER_OF_RDPCM_SIGNALLING_MODES];  ///< control flags
                                                              /// for residual
                                                              /// DPCM
  Bool m_enableAMP;
  Bool m_persistentRiceAdaptationEnabledFlag;  ///< control flag for Golomb-Rice
                                               /// parameter adaptation over each
                                               /// slice
  Bool m_cabacBypassAlignmentEnabledFlag;

// coding quality
#if JVET_E0059_FLOATING_POINT_QP_FIX
  OptionalValue<UInt> m_qpIncrementAtSourceFrame;  ///< Optional source frame
                                                   /// number at which all
                                                   /// subsequent frames are to
                                                   /// use an increased internal
                                                   /// QP.
#else
  Double m_fQP;                            ///< QP value of key-picture (floating point)
#endif
  Int m_iQP;  ///< QP value of key-picture (integer)
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  Int  m_intraQPOffset;       ///< QP offset for intra slice (integer)
  Bool m_lambdaFromQPEnable;  ///< enable flag for QP:lambda fix
#endif
  std::string m_dQPFileName;         ///< QP offset for each slice (initialized from
                                     /// external file)
  Int* m_aidQP;                      ///< array of slice QP values
  Int  m_iMaxDeltaQP;                ///< max. |delta QP|
  UInt m_uiDeltaQpRD;                ///< dQP range for multi-pass slice QP optimization
  Int  m_iMaxCuDQPDepth;             ///< Max. depth for a minimum CuDQPSize (0:default)
  Int  m_diffCuChromaQpOffsetDepth;  ///< If negative, then do not apply chroma
                                     /// qp offsets.
  Bool m_bFastDeltaQP;               ///< Fast Delta QP (false:default)

  Int                m_cbQpOffset;                        ///< Chroma Cb QP Offset (0:default)
  Int                m_crQpOffset;                        ///< Chroma Cr QP Offset (0:default)
  WCGChromaQPControl m_wcgChromaQpControl;                ///< Wide-colour-gamut chroma QP
                                                          /// control.
  UInt m_sliceChromaQpOffsetPeriodicity;                  ///< Used in conjunction with Slice
                                                          /// Cb/Cr QpOffsetIntraOrPeriodic. Use
                                                          /// 0 (default) to disable periodic
                                                          /// nature.
  Int m_sliceChromaQpOffsetIntraOrPeriodic[2 /*Cb,Cr*/];  ///< Chroma Cb QP Offset at slice level for I slice or for
                                                          /// periodic inter slices as defined by
                                                          /// SliceChromaQPOffsetPeriodicity. Replaces offset in the
                                                          /// GOP table.
  LumaLevelToDeltaQPMapping m_lumaLevelToDeltaQPMapping;  ///< mapping from luma
                                                          /// level to Delta QP.
#if ADAPTIVE_QP_SELECTION
  Bool m_bUseAdaptQpSelect;
#endif
  TComSEIMasteringDisplay m_masteringDisplay;

  Bool m_bUseAdaptiveQP;     ///< Flag for enabling QP adaptation based on a
                             /// psycho-visual model
  Int m_iQPAdaptationRange;  ///< dQP range by QP adaptation

  Int m_maxTempLayer;  ///< Max temporal layer

  // coding unit (CU) definition
  // TODO: Remove MaxCUWidth/MaxCUHeight and replace with MaxCUSize.
  UInt m_uiMaxCUWidth;                     ///< max. CU width in pixel
  UInt m_uiMaxCUHeight;                    ///< max. CU height in pixel
  UInt m_uiMaxCUDepth;                     ///< max. CU depth (as specified by command line)
  UInt m_uiMaxTotalCUDepth;                ///< max. total CU depth - includes depth of
                                           /// transform-block structure
  UInt m_uiLog2DiffMaxMinCodingBlockSize;  ///< difference between largest and
                                           /// smallest CU depth

  // transfom unit (TU) definition
  UInt m_uiQuadtreeTULog2MaxSize;
  UInt m_uiQuadtreeTULog2MinSize;

  UInt m_uiQuadtreeTUMaxDepthInter;
  UInt m_uiQuadtreeTUMaxDepthIntra;

  // coding tools (bit-depth)
  Int m_inputBitDepth[MAX_NUM_CHANNEL_TYPE];        ///< bit-depth of input file
  Int m_outputBitDepth[MAX_NUM_CHANNEL_TYPE];       ///< bit-depth of output file
  Int m_MSBExtendedBitDepth[MAX_NUM_CHANNEL_TYPE];  ///< bit-depth of input
                                                    /// samples after MSB
                                                    /// extension
  Int m_internalBitDepth[MAX_NUM_CHANNEL_TYPE];     ///< bit-depth codec operates
                                                    /// at (input/output files will
                                                    /// be converted)
  Bool m_extendedPrecisionProcessingFlag;
  Bool m_highPrecisionOffsetsEnabledFlag;

  // coding tools (chroma format)
  ChromaFormat m_chromaFormatIDC;

  // coding tools (PCM bit-depth)
  Bool m_bPCMInputBitDepthFlag;  ///< 0: PCM bit-depth is internal bit-depth. 1:
                                 /// PCM bit-depth is input bit-depth.

  // coding tool (SAO)
  Bool   m_bUseSAO;
  Bool   m_bTestSAODisableAtPictureLevel;
  Double m_saoEncodingRate;        ///< When >0 SAO early picture termination is
                                   /// enabled for luma and chroma
  Double m_saoEncodingRateChroma;  ///< The SAO early picture termination rate
                                   /// to use for chroma (when m_SaoEncodingRate
                                   /// is >0). If <=0, use results for luma.
  Int  m_maxNumOffsetsPerPic;      ///< SAO maximun number of offset per picture
  Bool m_saoCtuBoundary;           ///< SAO parameter estimation using non-deblocked
                                   /// pixels for CTU bottom and right boundary areas
#if ADD_RESET_ENCODER_DECISIONS_AFTER_IRAP
  Bool m_resetEncoderStateAfterIRAP;  ///< When true, encoder state will be
                                      /// reset following an IRAP.
#else
  Bool   m_saoResetEncoderStateAfterIRAP;  ///< When true, SAO encoder state will
                                           /// be reset following an IRAP.
#endif
  // coding tools (loop filter)
  Bool m_bLoopFilterDisable;       ///< flag for using deblocking filter
  Bool m_loopFilterOffsetInPPS;    ///< offset for deblocking filter in 0 = slice
                                   /// header, 1 = PPS
  Int m_loopFilterBetaOffsetDiv2;  ///< beta offset for deblocking filter
  Int m_loopFilterTcOffsetDiv2;    ///< tc offset for deblocking filter
  Int m_deblockingFilterMetric;    ///< blockiness metric in encoder
  // coding tools (PCM)
  Bool m_usePCM;                         ///< flag for using IPCM
  UInt m_pcmLog2MaxSize;                 ///< log2 of maximum PCM block size
  UInt m_uiPCMLog2MinSize;               ///< log2 of minimum PCM block size
  Bool m_bPCMFilterDisableFlag;          ///< PCM filter disable flag
  Bool m_enableIntraReferenceSmoothing;  ///< flag for
                                         /// enabling(default)/disabling intra
                                         /// reference smoothing/filtering
  // coding tools (encoder-only parameters)
  Bool m_bUseASR;                        ///< flag for using adaptive motion search range
  Bool m_bUseHADME;                      ///< flag for using HAD in sub-pel ME
  Bool m_useRDOQ;                        ///< flag for using RD optimized quantization
  Bool m_useRDOQTS;                      ///< flag for using RD optimized quantization for transform
                                         /// skip
  Bool m_useSelectiveRDOQ;               ///< flag for using selective RDOQ
  Int  m_rdPenalty;                      ///< RD-penalty for 32x32 TU for intra in non-intra slices
                                         ///(0: no RD-penalty, 1: RD-penalty, 2: maximum RD-penalty)
  Bool m_bDisableIntraPUsInInterSlices;  ///< Flag for disabling intra predicted
                                         /// PUs in inter slices.
  MESearchMethod m_motionEstimationSearchMethod;
  Bool           m_bRestrictMESampling;       ///< Restrict sampling for the Selective ME
  Int            m_iSearchRange;              ///< ME search range
  Int            m_bipredSearchRange;         ///< ME search range for bipred refinement
  Int            m_minSearchWindow;           ///< ME minimum search window size for the Adaptive
                                              /// Window ME
  Bool m_bClipForBiPredMeEnabled;             ///< Enables clipping for Bi-Pred ME.
  Bool m_bFastMEAssumingSmootherMVEnabled;    ///< Enables fast ME assuming a
                                              /// smoother MV.
  FastInterSearchMode m_fastInterSearchMode;  ///< Parameter that controls fast
                                              /// encoder settings
  Bool m_bUseEarlyCU;                         ///< flag for using Early CU setting
  Bool m_useFastDecisionForMerge;             ///< flag for using Fast Decision Merge
                                              /// RD-Cost
  Bool            m_bUseCbfFastMode;          ///< flag for using Cbf Fast PU Mode Decision
  Bool            m_useEarlySkipDetection;    ///< flag for using Early SKIP Detection
  SliceConstraint m_sliceMode;
  Int             m_sliceArgument;  ///< argument according to selected slice mode
  SliceConstraint m_sliceSegmentMode;
  Int             m_sliceSegmentArgument;  ///< argument according to selected slice segment
                                           /// mode

  Bool m_bLFCrossSliceBoundaryFlag;  ///< 1: filter across slice boundaries 0:
                                     /// do not filter across slice boundaries
  Bool m_bLFCrossTileBoundaryFlag;   ///< 1: filter across tile boundaries  0: do
                                     /// not filter across tile boundaries
  Bool             m_tileUniformSpacingFlag;
  Int              m_numTileColumnsMinus1;
  Int              m_numTileRowsMinus1;
  std::vector<Int> m_tileColumnWidth;
  std::vector<Int> m_tileRowHeight;
  Bool             m_entropyCodingSyncEnabledFlag;

  Bool m_bUseConstrainedIntraPred;  ///< flag for using constrained intra
                                    /// prediction
  Bool m_bFastUDIUseMPMEnabled;
  Bool m_bFastMEForGenBLowDelayEnabled;
  Bool m_bUseBLambdaForNonKeyLowDelayPictures;

  HashType m_decodedPictureHashSEIType;  ///< Checksum mode for decoded picture
                                         /// hash SEI message
  Bool m_recoveryPointSEIEnabled;
  Bool m_bufferingPeriodSEIEnabled;
  Bool m_pictureTimingSEIEnabled;
  Bool m_toneMappingInfoSEIEnabled;
  Bool m_chromaResamplingFilterSEIenabled;
  Int  m_chromaResamplingHorFilterIdc;
  Int  m_chromaResamplingVerFilterIdc;
  Int  m_toneMapId;
  Bool m_toneMapCancelFlag;
  Bool m_toneMapPersistenceFlag;
  Int  m_toneMapCodedDataBitDepth;
  Int  m_toneMapTargetBitDepth;
  Int  m_toneMapModelId;
  Int  m_toneMapMinValue;
  Int  m_toneMapMaxValue;
  Int  m_sigmoidMidpoint;
  Int  m_sigmoidWidth;
  Int  m_numPivots;
  Int  m_cameraIsoSpeedIdc;
  Int  m_cameraIsoSpeedValue;
  Int  m_exposureIndexIdc;
  Int  m_exposureIndexValue;
  Bool m_exposureCompensationValueSignFlag;
  Int  m_exposureCompensationValueNumerator;
  Int  m_exposureCompensationValueDenomIdc;
  Int  m_refScreenLuminanceWhite;
  Int  m_extendedRangeWhiteLevel;
  Int  m_nominalBlackLevelLumaCodeValue;
  Int  m_nominalWhiteLevelLumaCodeValue;
  Int  m_extendedWhiteLevelLumaCodeValue;
  Int* m_startOfCodedInterval;
  Int* m_codedPivotValue;
  Int* m_targetPivotValue;
  Bool m_framePackingSEIEnabled;
  Int  m_framePackingSEIType;
  Int  m_framePackingSEIId;
  Int  m_framePackingSEIQuincunx;
  Int  m_framePackingSEIInterpretation;
  Bool m_segmentedRectFramePackingSEIEnabled;
  Bool m_segmentedRectFramePackingSEICancel;
  Int  m_segmentedRectFramePackingSEIType;
  Bool m_segmentedRectFramePackingSEIPersistence;
  Int  m_displayOrientationSEIAngle;
  Bool m_temporalLevel0IndexSEIEnabled;
  Bool m_gradualDecodingRefreshInfoEnabled;
  Int  m_noDisplaySEITLayer;
  Bool m_decodingUnitInfoSEIEnabled;
  Bool m_SOPDescriptionSEIEnabled;
  Bool m_scalableNestingSEIEnabled;
  Bool m_tmctsSEIEnabled;
#if MCTS_ENC_CHECK
  Bool m_tmctsSEITileConstraint;
#endif
#if MCTS_EXTRACTION
  Bool m_tmctsExtractionSEIEnabled;
#endif
  Bool                                    m_timeCodeSEIEnabled;
  Int                                     m_timeCodeSEINumTs;
  TComSEITimeSet                          m_timeSetArray[MAX_TIMECODE_SEI_SETS];
  Bool                                    m_kneeSEIEnabled;
  TEncCfg::TEncSEIKneeFunctionInformation m_kneeFunctionInformationSEI;
  Int                                     m_preferredTransferCharacteristics;
  UInt                                    m_greenMetadataType;
  UInt                                    m_xsdMetricType;
#if CCV_SEI_MESSAGE
  Bool   m_ccvSEIEnabled;
  Bool   m_ccvSEICancelFlag;
  Bool   m_ccvSEIPersistenceFlag;
  Bool   m_ccvSEIPrimariesPresentFlag;
  Bool   m_ccvSEIMinLuminanceValuePresentFlag;
  Bool   m_ccvSEIMaxLuminanceValuePresentFlag;
  Bool   m_ccvSEIAvgLuminanceValuePresentFlag;
  Double m_ccvSEIPrimariesX[MAX_NUM_COMPONENT];  // Int(32)
  Double m_ccvSEIPrimariesY[MAX_NUM_COMPONENT];  // Int(32)
  Double m_ccvSEIMinLuminanceValue;              // UInt(32)
  Double m_ccvSEIMaxLuminanceValue;              // UInt(32)
  Double m_ccvSEIAvgLuminanceValue;              // UInt(32)
#endif
#if ERP_SR_OV_SEI_MESSAGE
  Bool m_erpSEIEnabled;
  Bool m_erpSEICancelFlag;
  Bool m_erpSEIPersistenceFlag;
  Bool m_erpSEIGuardBandFlag;
  UInt m_erpSEIGuardBandType;
  UInt m_erpSEILeftGuardBandWidth;
  UInt m_erpSEIRightGuardBandWidth;

  Bool m_sphereRotationSEIEnabled;
  Bool m_sphereRotationSEICancelFlag;
  Bool m_sphereRotationSEIPersistenceFlag;
  Int  m_sphereRotationSEIYaw;
  Int  m_sphereRotationSEIPitch;
  Int  m_sphereRotationSEIRoll;

  Bool              m_omniViewportSEIEnabled;
  UInt              m_omniViewportSEIId;
  Bool              m_omniViewportSEICancelFlag;
  Bool              m_omniViewportSEIPersistenceFlag;
  UInt              m_omniViewportSEICntMinus1;
  std::vector<Int>  m_omniViewportSEIAzimuthCentre;
  std::vector<Int>  m_omniViewportSEIElevationCentre;
  std::vector<Int>  m_omniViewportSEITiltCentre;
  std::vector<UInt> m_omniViewportSEIHorRange;
  std::vector<UInt> m_omniViewportSEIVerRange;
#endif
#if CMP_SEI_MESSAGE
  Bool m_cmpSEIEnabled;
  Bool m_cmpSEICmpCancelFlag;
  Bool m_cmpSEICmpPersistenceFlag;
#endif
#if RWP_SEI_MESSAGE
  Bool                m_rwpSEIEnabled;
  Bool                m_rwpSEIRwpCancelFlag;
  Bool                m_rwpSEIRwpPersistenceFlag;
  Bool                m_rwpSEIConstituentPictureMatchingFlag;
  Int                 m_rwpSEINumPackedRegions;
  Int                 m_rwpSEIProjPictureWidth;
  Int                 m_rwpSEIProjPictureHeight;
  Int                 m_rwpSEIPackedPictureWidth;
  Int                 m_rwpSEIPackedPictureHeight;
  std::vector<UChar>  m_rwpSEIRwpTransformType;
  std::vector<Bool>   m_rwpSEIRwpGuardBandFlag;
  std::vector<UInt>   m_rwpSEIProjRegionWidth;
  std::vector<UInt>   m_rwpSEIProjRegionHeight;
  std::vector<UInt>   m_rwpSEIRwpSEIProjRegionTop;
  std::vector<UInt>   m_rwpSEIProjRegionLeft;
  std::vector<UShort> m_rwpSEIPackedRegionWidth;
  std::vector<UShort> m_rwpSEIPackedRegionHeight;
  std::vector<UShort> m_rwpSEIPackedRegionTop;
  std::vector<UShort> m_rwpSEIPackedRegionLeft;
  std::vector<UChar>  m_rwpSEIRwpLeftGuardBandWidth;
  std::vector<UChar>  m_rwpSEIRwpRightGuardBandWidth;
  std::vector<UChar>  m_rwpSEIRwpTopGuardBandHeight;
  std::vector<UChar>  m_rwpSEIRwpBottomGuardBandHeight;
  std::vector<Bool>   m_rwpSEIRwpGuardBandNotUsedForPredFlag;
  std::vector<UChar>  m_rwpSEIRwpGuardBandType;
#endif
  // weighted prediction
  Bool m_useWeightedPred;    ///< Use of weighted prediction in P slices
  Bool m_useWeightedBiPred;  ///< Use of bi-directional weighted prediction in B
                             /// slices
  WeightedPredictionMethod m_weightedPredictionMethod;

  UInt m_log2ParallelMergeLevel;  ///< Parallel merge estimation region
  UInt m_maxNumMergeCand;         ///< Max number of merge candidates

  Int  m_TMVPModeId;
  Bool m_signDataHidingEnabledFlag;
  Bool m_RCEnableRateControl;              ///< enable rate control or not
  Int  m_RCTargetBitrate;                  ///< target bitrate when rate control is enabled
  Int  m_RCKeepHierarchicalBit;            ///< 0: equal bit allocation; 1: fixed ratio bit
                                           /// allocation; 2: adaptive ratio bit allocation
  Bool m_RCLCULevelRC;                     ///< true: LCU level rate control; false: picture level
                                           /// rate control NOTE: code-tidy - rename to
                                           /// m_RCCtuLevelRC
  Bool m_RCUseLCUSeparateModel;            ///< use separate R-lambda model at LCU level
                                           /// NOTE: code-tidy - rename to
                                           /// m_RCUseCtuSeparateModel
  Int  m_RCInitialQP;                      ///< inital QP for rate control
  Bool m_RCForceIntraQP;                   ///< force all intra picture to use initial QP or not
  Bool m_RCCpbSaturationEnabled;           ///< enable target bits saturation to avoid
                                           /// CPB overflow and underflow
  UInt            m_RCCpbSize;             ///< CPB size
  Double          m_RCInitialCpbFullness;  ///< initial CPB fullness
  ScalingListMode m_useScalingListId;      ///< using quantization matrix
  std::string     m_scalingListFileName;   ///< quantization matrix file name

  Bool m_TransquantBypassEnabledFlag;  ///< transquant_bypass_enabled_flag
                                       /// setting in PPS.
  Bool m_CUTransquantBypassFlagForce;  ///< if transquant_bypass_enabled_flag,
                                       /// then, if true, all CU transquant
                                       /// bypass flags will be set to true.
  CostMode m_costMode;                 ///< Cost mode to use

  Bool m_recalculateQPAccordingToLambda;  ///< recalculate QP value according to
                                          /// the lambda value
  Bool m_useStrongIntraSmoothing;         ///< enable strong intra smoothing for 32x32
                                          /// blocks where the reference samples are
                                          /// flat
  Int m_activeParameterSetsSEIEnabled;

  Bool m_vuiParametersPresentFlag;            ///< enable generation of VUI parameters
  Bool m_aspectRatioInfoPresentFlag;          ///< Signals whether aspect_ratio_idc is
                                              /// present
  Int  m_aspectRatioIdc;                      ///< aspect_ratio_idc
  Int  m_sarWidth;                            ///< horizontal size of the sample aspect ratio
  Int  m_sarHeight;                           ///< vertical size of the sample aspect ratio
  Bool m_overscanInfoPresentFlag;             ///< Signals whether
                                              /// overscan_appropriate_flag is present
  Bool m_overscanAppropriateFlag;             ///< Indicates whether conformant decoded
                                              /// pictures are suitable for display using
                                              /// overscan
  Bool m_videoSignalTypePresentFlag;          ///< Signals whether video_format,
                                              /// video_full_range_flag, and
                                              /// colour_description_present_flag are
                                              /// present
  Int  m_videoFormat;                         ///< Indicates representation of pictures
  Bool m_videoFullRangeFlag;                  ///< Indicates the black level and range of luma
                                              /// and chroma signals
  Bool m_colourDescriptionPresentFlag;        ///< Signals whether colour_primaries,
                                              /// transfer_characteristics and
                                              /// matrix_coefficients are present
  Int m_colourPrimaries;                      ///< Indicates chromaticity coordinates of the source
                                              /// primaries
  Int m_transferCharacteristics;              ///< Indicates the opto-electronic transfer
                                              /// characteristics of the source
  Int m_matrixCoefficients;                   ///< Describes the matrix coefficients used in
                                              /// deriving luma and chroma from RGB primaries
  Bool m_chromaLocInfoPresentFlag;            ///< Signals whether
                                              /// chroma_sample_loc_type_top_field and
                                              /// chroma_sample_loc_type_bottom_field are
                                              /// present
  Int m_chromaSampleLocTypeTopField;          ///< Specifies the location of chroma
                                              /// samples for top field
  Int m_chromaSampleLocTypeBottomField;       ///< Specifies the location of chroma
                                              /// samples for bottom field
  Bool m_neutralChromaIndicationFlag;         ///< Indicates that the value of all
                                              /// decoded chroma samples is equal to
                                              /// 1<<(BitDepthCr-1)
  Bool m_defaultDisplayWindowFlag;            ///< Indicates the presence of the default
                                              /// window parameters
  Int m_defDispWinLeftOffset;                 ///< Specifies the left offset from the
                                              /// conformance window of the default window
  Int m_defDispWinRightOffset;                ///< Specifies the right offset from the
                                              /// conformance window of the default window
  Int m_defDispWinTopOffset;                  ///< Specifies the top offset from the conformance
                                              /// window of the default window
  Int m_defDispWinBottomOffset;               ///< Specifies the bottom offset from the
                                              /// conformance window of the default window
  Bool m_frameFieldInfoPresentFlag;           ///< Indicates that pic_struct values are
                                              /// present in picture timing SEI messages
  Bool m_pocProportionalToTimingFlag;         ///< Indicates that the POC value is
                                              /// proportional to the output time w.r.t.
                                              /// first picture in CVS
  Int m_numTicksPocDiffOneMinus1;             ///< Number of ticks minus 1 that for a POC
                                              /// difference of one
  Bool m_bitstreamRestrictionFlag;            ///< Signals whether bitstream restriction
                                              /// parameters are present
  Bool m_tilesFixedStructureFlag;             ///< Indicates that each active picture
                                              /// parameter set has the same values of the
                                              /// syntax elements related to tiles
  Bool m_motionVectorsOverPicBoundariesFlag;  ///< Indicates that no samples
                                              /// outside the picture boundaries
                                              /// are used for inter prediction
  Int m_minSpatialSegmentationIdc;            ///< Indicates the maximum size of the
                                              /// spatial segments in the pictures in the
                                              /// coded video sequence
  Int m_maxBytesPerPicDenom;                  ///< Indicates a number of bytes not exceeded by
                                              /// the sum of the sizes of the VCL NAL units
                                              /// associated with any coded picture
  Int m_maxBitsPerMinCuDenom;                 ///< Indicates an upper bound for the number of
                                              /// bits of coding_unit() data
  Int m_log2MaxMvLengthHorizontal;            ///< Indicate the maximum absolute value of
                                              /// a decoded horizontal MV component in
                                              /// quarter-pel luma units
  Int m_log2MaxMvLengthVertical;              ///< Indicate the maximum absolute value of a
                                              /// decoded vertical MV component in
                                              /// quarter-pel luma units
  std::string m_colourRemapSEIFileRoot;
#if RNSEI
  std::string m_regionalNestingSEIFileRoot;
#endif

  std::string m_summaryOutFilename;      ///< filename to use for producing summary
                                         /// output file.
  std::string m_summaryPicFilenameBase;  ///< Base filename to use for producing
                                         /// summary picture output files. The
                                         /// actual filenames used will have
                                         /// I.txt, P.txt and B.txt appended.
  UInt m_summaryVerboseness;             ///< Specifies the level of the verboseness of the
                                         /// text output.

  // SCM new added variables
  Bool m_printClippedPSNR;
  Bool m_bRGBformat;
  Bool m_useColourTrans;
  Int  m_actYQpOffset;
  Int  m_actCbQpOffset;
  Int  m_actCrQpOffset;
  Bool m_useLL;
  Bool m_usePaletteMode;
  UInt m_paletteMaxSize;
  UInt m_paletteMaxPredSize;
  Int  m_motionVectorResolutionControlIdc;
  Bool m_palettePredInSPSEnabled;
  Bool m_palettePredInPPSEnabled;
  Bool m_useIntraBlockCopy;
  Bool m_intraBlockCopyFastSearch;                ///< Use a restricted search range for intra
                                                  /// block-copy motion vectors to reduce the
                                                  /// encoding time
  Bool m_useHashBasedIntraBlockCopySearch;        ///< Enable the use of hash based
                                                  /// search for intra block copying on
                                                  /// 8x8 blocks
  Int m_intraBlockCopySearchWidthInCTUs;          ///< Search range for IBC hash search
                                                  /// method (-1: full frame search)
  UInt m_intraBlockCopyNonHashSearchWidthInCTUs;  ///< Search range for IBC
                                                  /// non-hash search method
                                                  ///(i.e., fast/full search)
  Bool m_useHashBasedME;                          ///< flag for using hash based inter search
  Bool m_disableIntraBoundaryFilter;              ///  flag for enabling(default)/disabling
                                                  /// intra boundary filtering
  Bool m_bTransquantBypassInferTUSplit;           ///< Infer TU splitting for transquant
                                                  /// bypass CUs
  Bool m_bNoTUSplitIntraACTEnabled;

#if EXTENSION_360_VIDEO
  TExt360AppEncCfg m_ext360;
  friend class TExt360AppEncCfg;
  friend class TExt360AppEncTop;
#endif

  // internal member functions
  Void xCheckParameter();  ///< check validity of configuration values
  Void xPrintParameter();  ///< print configuration values
  Void xPrintUsage();      ///< print usage
 public:
  PCCHMLibVideoEncoderCfg();
  virtual ~PCCHMLibVideoEncoderCfg();

 public:
  Void create();                             ///< create option handling class
  Void destroy();                            ///< destroy option handling class
  Bool parseCfg( Int argc, TChar* argv[] );  ///< parse configuration file to fill
                                             /// member variables
#if PCC_CF_EXT

#endif

};  // END CLASS DEFINITION PCCHMLibVideoEncoderCfg

//! \}
#endif

#endif  // __TAPPENCCFG__
