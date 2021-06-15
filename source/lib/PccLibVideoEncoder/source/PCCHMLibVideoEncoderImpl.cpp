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

#include "PCCCommon.h"
#include "PCCLogger.h"

#ifdef USE_HMLIB_VIDEO_CODEC

#include "PCCHMLibVideoEncoderImpl.h"

using namespace pcc;
using namespace pcc_hm;

/// encoder application class

template <typename T>
PCCHMLibVideoEncoderImpl<T>::PCCHMLibVideoEncoderImpl() {
  m_iFrameRcvd     = 0;
  m_totalBytes     = 0;
  m_essentialBytes = 0;
}

template <typename T>
PCCHMLibVideoEncoderImpl<T>::~PCCHMLibVideoEncoderImpl() {}

template <typename T>
Void PCCHMLibVideoEncoderImpl<T>::encode( PCCVideo<T, 3>&    videoSrc,
                                          std::string        arguments,
                                          PCCVideoBitstream& bitstream,
                                          PCCVideo<T, 3>&    videoRec ) {
  std::ostringstream oss( ostringstream::binary | ostringstream::out );
  std::ostream&      bitstreamFile = oss;
  std::istringstream iss( arguments );
  std::string        token;
  std::vector<char*> args;
  while ( iss >> token ) {
    char* arg = new char[token.size() + 1];
    copy( token.begin(), token.end(), arg );
    arg[token.size()] = '\0';
    args.push_back( arg );
  }
  create();
  // parse configuration
  try {
    if ( !parseCfg( args.size(), &args[0] ) ) {
      destroy();
#if ENVIRONMENT_VARIABLE_DEBUG_AND_TEST
      EnvVar::printEnvVar();
#endif
      return;
    }
  } catch ( df::program_options_lite::ParseFailure& e ) {
    std::cerr << "Error parsing option \"" << e.arg << "\" with argument \"" << e.val << "\"." << std::endl;
    return;
  }
  for ( size_t i = 0; i < args.size(); i++ ) { delete[] args[i]; }
  m_framesToBeEncoded     = std::min( m_framesToBeEncoded, (int)videoSrc.getFrameCount() );
  TComPicYuv* pcPicYuvOrg = new TComPicYuv;
  TComPicYuv* pcPicYuvRec = NULL;
  // initialize internal class & member variables
  xInitLibCfg();
  m_cTEncTop.create();
  m_cTEncTop.init( m_isField );
  videoRec.clear();

#if PCC_ME_EXT
  if ( m_usePCCExt ) {
    // Note JR: must be given from the function parameters
    printf( "\nReading the aux info files\n" );
    FILE* patchFile = NULL;
    patchFile       = fopen( m_patchInfoFileName.c_str(), "rb" );
    for ( Int i = 0; i < PCC_ME_EXT_MAX_NUM_FRAMES; i++ ) {
      long long readSize = fread( &g_numPatches[i], sizeof( long long ), 1, patchFile );
      if ( readSize != 1 && readSize != 0 ) { printf( "error: Wrong Patch data group file" ); }
      for ( Int patchIdx = 0; patchIdx < g_numPatches[i]; patchIdx++ ) {
        readSize = fread( &g_projectionIndex[i][patchIdx], sizeof( long long ), 1, patchFile );
        if ( readSize != 1 ) { printf( "error: Wrong Auxiliary data format" ); }
        readSize = fread( g_patch2DInfo[i][patchIdx], sizeof( long long ), 4, patchFile );
        if ( readSize != 4 ) { printf( "error: Wrong Auxiliary data format" ); }
        readSize = fread( g_patch3DInfo[i][patchIdx], sizeof( long long ), 3, patchFile );
        if ( readSize != 3 ) { printf( "error: Wrong Auxiliary data format" ); }
      }
    }
    fclose( patchFile );
  }
#endif
  printChromaFormat();
  // main encoder loop
  Int                              iNumEncoded = 0;
  Bool                             bEos        = false;
  const InputColourSpaceConversion ipCSC       = m_inputColourSpaceConvert;
  const InputColourSpaceConversion snrCSC =
      ( !m_snrInternalColourSpace ) ? m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;
  list<AccessUnit> outputAccessUnits;
  TComPicYuv       cPicYuvTrueOrg;
  // allocate original YUV buffer
  if ( m_isField ) {
    pcPicYuvOrg->create( m_iSourceWidth, m_iSourceHeightOrg, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight,
                         m_uiMaxTotalCUDepth, true );
    cPicYuvTrueOrg.create( m_iSourceWidth, m_iSourceHeightOrg, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight,
                           m_uiMaxTotalCUDepth, true );
  } else {
    pcPicYuvOrg->create( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight,
                         m_uiMaxTotalCUDepth, true );
    cPicYuvTrueOrg.create( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight,
                           m_uiMaxTotalCUDepth, true );
  }
#if EXTENSION_360_VIDEO
  TExt360AppEncTop ext360( *this, m_cTEncTop.getGOPEncoder()->getExt360Data(), *( m_cTEncTop.getGOPEncoder() ),
                           *pcPicYuvOrg );
#endif

  while ( !bEos ) {
    xGetBuffer( pcPicYuvRec );                            // get buffers
    xReadPicture( pcPicYuvOrg, videoSrc, m_iFrameRcvd );  // read input YUV file
    // increase number of received frames
    m_iFrameRcvd++;
    bEos = ( m_isField && ( m_iFrameRcvd == ( m_framesToBeEncoded >> 1 ) ) ) ||
           ( !m_isField && ( m_iFrameRcvd == m_framesToBeEncoded ) );
    // call encoding function for one frame
    if ( m_isField ) {
      m_cTEncTop.encode( bEos, pcPicYuvOrg, &cPicYuvTrueOrg, snrCSC, m_cListPicYuvRec, outputAccessUnits, iNumEncoded,
                         m_isTopFieldFirst );
    } else {
      m_cTEncTop.encode( bEos, pcPicYuvOrg, &cPicYuvTrueOrg, snrCSC, m_cListPicYuvRec, outputAccessUnits, iNumEncoded );
    }
    // write bistream to file if necessary
    if ( iNumEncoded > 0 ) {
      xWriteOutput( bitstreamFile, iNumEncoded, outputAccessUnits, videoRec );
      outputAccessUnits.clear();
    }
  }
  m_cTEncTop.printSummary( m_isField );
  // delete original YUV buffer
  pcPicYuvOrg->destroy();
  delete pcPicYuvOrg;
  pcPicYuvOrg = NULL;
  // delete used buffers in encoder class
  m_cTEncTop.deletePicBuffer();
  cPicYuvTrueOrg.destroy();
  // delete buffers & classes
  xDeleteBuffer();
  m_cTEncTop.destroy();
  printRateSummary();
  auto buffer = oss.str();
  bitstream.resize( buffer.size() );
  std::copy( buffer.data(), buffer.data() + buffer.size(), bitstream.vector().begin() );
  return;
}

template <typename T>
Void PCCHMLibVideoEncoderImpl<T>::xInitLibCfg() {
  TComVPS vps;
  vps.setMaxTLayers( m_maxTempLayer );
  if ( m_maxTempLayer == 1 ) { vps.setTemporalNestingFlag( true ); }
  vps.setMaxLayers( 1 );
  for ( Int i = 0; i < MAX_TLAYER; i++ ) {
    vps.setNumReorderPics( m_numReorderPics[i], i );
    vps.setMaxDecPicBuffering( m_maxDecPicBuffering[i], i );
  }
  m_cTEncTop.setVPS( &vps );
#if defined( PCC_ME_EXT ) & PCC_ME_EXT
  m_cTEncTop.setUsePCCExt( m_usePCCExt );
  if ( m_usePCCExt ) {
    m_cTEncTop.setBlockToPatchFileName( m_blockToPatchFileName );
    m_cTEncTop.setOccupancyMapFileName( m_occupancyMapFileName );
  }
#endif
#if defined( PCC_ME_EXT ) & PCC_RDO_EXT
  m_cTEncTop.setUsePCCRDOExt( m_usePCCRDO );
  if ( m_usePCCRDO ) { m_cTEncTop.setOccupancyMapFileName( m_occupancyMapFileName ); }
#endif

  m_cTEncTop.setProfile( m_profile );
  m_cTEncTop.setLevel( m_levelTier, m_level );
  m_cTEncTop.setProgressiveSourceFlag( m_progressiveSourceFlag );
  m_cTEncTop.setInterlacedSourceFlag( m_interlacedSourceFlag );
  m_cTEncTop.setNonPackedConstraintFlag( m_nonPackedConstraintFlag );
  m_cTEncTop.setFrameOnlyConstraintFlag( m_frameOnlyConstraintFlag );
  m_cTEncTop.setBitDepthConstraintValue( m_bitDepthConstraint );
  m_cTEncTop.setChromaFormatConstraintValue( m_chromaFormatConstraint );
  m_cTEncTop.setIntraConstraintFlag( m_intraConstraintFlag );
  m_cTEncTop.setOnePictureOnlyConstraintFlag( m_onePictureOnlyConstraintFlag );
  m_cTEncTop.setLowerBitRateConstraintFlag( m_lowerBitRateConstraintFlag );
  m_cTEncTop.setPrintMSEBasedSequencePSNR( m_printMSEBasedSequencePSNR );
  m_cTEncTop.setPrintHexPsnr( m_printHexPsnr );
  m_cTEncTop.setPrintFrameMSE( m_printFrameMSE );
  m_cTEncTop.setPrintSequenceMSE( m_printSequenceMSE );
  m_cTEncTop.setPrintClippedPSNR( m_printClippedPSNR );
#if JVET_F0064_MSSSIM
  m_cTEncTop.setPrintMSSSIM( m_printMSSSIM );
#endif
#if JCTVC_Y0037_XPSNR
  m_cTEncTop.setXPSNREnableFlag( m_bXPSNREnableFlag );
  for ( Int id = 0; id < MAX_NUM_COMPONENT; id++ ) {
    m_cTEncTop.setXPSNRWeight( m_dXPSNRWeight[id], ComponentID( id ) );
  }
#endif
  m_cTEncTop.setCabacZeroWordPaddingEnabled( m_cabacZeroWordPaddingEnabled );
  m_cTEncTop.setFrameRate( m_iFrameRate );
  m_cTEncTop.setFrameSkip( m_FrameSkip );
  m_cTEncTop.setTemporalSubsampleRatio( m_temporalSubsampleRatio );
  m_cTEncTop.setSourceWidth( m_iSourceWidth );
  m_cTEncTop.setSourceHeight( m_iSourceHeight );
  m_cTEncTop.setConformanceWindow( m_confWinLeft, m_confWinRight, m_confWinTop, m_confWinBottom );
  m_cTEncTop.setFramesToBeEncoded( m_framesToBeEncoded );

  //====== Coding Structure ========
  m_cTEncTop.setIntraPeriod( m_iIntraPeriod );
  m_cTEncTop.setDecodingRefreshType( m_iDecodingRefreshType );
  m_cTEncTop.setGOPSize( m_iGOPSize );
#if JCTVC_Y0038_PARAMS
  m_cTEncTop.setReWriteParamSetsFlag( m_bReWriteParamSetsFlag );
#endif
  m_cTEncTop.setGopList( m_GOPList );
  m_cTEncTop.setExtraRPSs( m_extraRPSs );
  for ( Int i = 0; i < MAX_TLAYER; i++ ) {
    m_cTEncTop.setNumReorderPics( m_numReorderPics[i], i );
    m_cTEncTop.setMaxDecPicBuffering( m_maxDecPicBuffering[i], i );
  }
  for ( UInt uiLoop = 0; uiLoop < MAX_TLAYER; ++uiLoop ) {
    m_cTEncTop.setLambdaModifier( uiLoop, m_adLambdaModifier[uiLoop] );
  }
  m_cTEncTop.setIntraLambdaModifier( m_adIntraLambdaModifier );
  m_cTEncTop.setIntraQpFactor( m_dIntraQpFactor );
  m_cTEncTop.setQP( m_iQP );
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  m_cTEncTop.setIntraQPOffset( m_intraQPOffset );
  m_cTEncTop.setLambdaFromQPEnable( m_lambdaFromQPEnable );
#endif
  m_cTEncTop.setPad( m_aiPad );
  m_cTEncTop.setAccessUnitDelimiter( m_AccessUnitDelimiter );
  m_cTEncTop.setMaxTempLayer( m_maxTempLayer );
  m_cTEncTop.setUseAMP( m_enableAMP );

  //===== Slice ========
  //====== Loop/Deblock Filter ========
  m_cTEncTop.setLoopFilterDisable( m_bLoopFilterDisable );
  m_cTEncTop.setLoopFilterOffsetInPPS( m_loopFilterOffsetInPPS );
  m_cTEncTop.setLoopFilterBetaOffset( m_loopFilterBetaOffsetDiv2 );
  m_cTEncTop.setLoopFilterTcOffset( m_loopFilterTcOffsetDiv2 );
  m_cTEncTop.setDeblockingFilterMetric( m_deblockingFilterMetric );

  //====== Motion search ========
  m_cTEncTop.setDisableIntraPUsInInterSlices( m_bDisableIntraPUsInInterSlices );
  m_cTEncTop.setMotionEstimationSearchMethod( m_motionEstimationSearchMethod );
  m_cTEncTop.setSearchRange( m_iSearchRange );
  m_cTEncTop.setBipredSearchRange( m_bipredSearchRange );
  m_cTEncTop.setClipForBiPredMeEnabled( m_bClipForBiPredMeEnabled );
  m_cTEncTop.setFastMEAssumingSmootherMVEnabled( m_bFastMEAssumingSmootherMVEnabled );
  m_cTEncTop.setMinSearchWindow( m_minSearchWindow );
  m_cTEncTop.setRestrictMESampling( m_bRestrictMESampling );

  //====== Quality control ========
  m_cTEncTop.setMaxDeltaQP( m_iMaxDeltaQP );
  m_cTEncTop.setMaxCuDQPDepth( m_iMaxCuDQPDepth );
  m_cTEncTop.setDiffCuChromaQpOffsetDepth( m_diffCuChromaQpOffsetDepth );
  m_cTEncTop.setChromaCbQpOffset( m_cbQpOffset );
  m_cTEncTop.setChromaCrQpOffset( m_crQpOffset );
  m_cTEncTop.setWCGChromaQpControl( m_wcgChromaQpControl );
  m_cTEncTop.setSliceChromaOffsetQpIntraOrPeriodic( m_sliceChromaQpOffsetPeriodicity,
                                                    m_sliceChromaQpOffsetIntraOrPeriodic );
  m_cTEncTop.setChromaFormatIdc( m_chromaFormatIDC );
#if ADAPTIVE_QP_SELECTION
  m_cTEncTop.setUseAdaptQpSelect( m_bUseAdaptQpSelect );
#endif
  m_cTEncTop.setUseAdaptiveQP( m_bUseAdaptiveQP );
  m_cTEncTop.setQPAdaptationRange( m_iQPAdaptationRange );
  m_cTEncTop.setExtendedPrecisionProcessingFlag( m_extendedPrecisionProcessingFlag );
  m_cTEncTop.setHighPrecisionOffsetsEnabledFlag( m_highPrecisionOffsetsEnabledFlag );
  m_cTEncTop.setWeightedPredictionMethod( m_weightedPredictionMethod );

  //====== Tool list ========
  m_cTEncTop.setLumaLevelToDeltaQPControls( m_lumaLevelToDeltaQPMapping );
#if X0038_LAMBDA_FROM_QP_CAPABILITY
  m_cTEncTop.setDeltaQpRD( ( m_costMode == COST_LOSSLESS_CODING ) ? 0 : m_uiDeltaQpRD );
#else
  m_cTEncTop.setDeltaQpRD( m_uiDeltaQpRD );
#endif
  m_cTEncTop.setFastDeltaQp( m_bFastDeltaQP );
  m_cTEncTop.setUseASR( m_bUseASR );
  m_cTEncTop.setUseHADME( m_bUseHADME );
  m_cTEncTop.setdQPs( m_aidQP );
  m_cTEncTop.setUseRDOQ( m_useRDOQ );
  m_cTEncTop.setUseRDOQTS( m_useRDOQTS );
  m_cTEncTop.setUseSelectiveRDOQ( m_useSelectiveRDOQ );
  m_cTEncTop.setRDpenalty( m_rdPenalty );
  m_cTEncTop.setMaxCUWidth( m_uiMaxCUWidth );
  m_cTEncTop.setMaxCUHeight( m_uiMaxCUHeight );
  m_cTEncTop.setMaxTotalCUDepth( m_uiMaxTotalCUDepth );
  m_cTEncTop.setLog2DiffMaxMinCodingBlockSize( m_uiLog2DiffMaxMinCodingBlockSize );
  m_cTEncTop.setQuadtreeTULog2MaxSize( m_uiQuadtreeTULog2MaxSize );
  m_cTEncTop.setQuadtreeTULog2MinSize( m_uiQuadtreeTULog2MinSize );
  m_cTEncTop.setQuadtreeTUMaxDepthInter( m_uiQuadtreeTUMaxDepthInter );
  m_cTEncTop.setQuadtreeTUMaxDepthIntra( m_uiQuadtreeTUMaxDepthIntra );
  m_cTEncTop.setFastInterSearchMode( m_fastInterSearchMode );
  m_cTEncTop.setUseEarlyCU( m_bUseEarlyCU );
  m_cTEncTop.setUseFastDecisionForMerge( m_useFastDecisionForMerge );
  m_cTEncTop.setUseCbfFastMode( m_bUseCbfFastMode );
  m_cTEncTop.setUseEarlySkipDetection( m_useEarlySkipDetection );
  m_cTEncTop.setCrossComponentPredictionEnabledFlag( m_crossComponentPredictionEnabledFlag );
  m_cTEncTop.setUseReconBasedCrossCPredictionEstimate( m_reconBasedCrossCPredictionEstimate );
  m_cTEncTop.setLog2SaoOffsetScale( CHANNEL_TYPE_LUMA, m_log2SaoOffsetScale[CHANNEL_TYPE_LUMA] );
  m_cTEncTop.setLog2SaoOffsetScale( CHANNEL_TYPE_CHROMA, m_log2SaoOffsetScale[CHANNEL_TYPE_CHROMA] );
  m_cTEncTop.setUseTransformSkip( m_useTransformSkip );
  m_cTEncTop.setUseTransformSkipFast( m_useTransformSkipFast );
  m_cTEncTop.setTransformSkipRotationEnabledFlag( m_transformSkipRotationEnabledFlag );
  m_cTEncTop.setTransformSkipContextEnabledFlag( m_transformSkipContextEnabledFlag );
  m_cTEncTop.setPersistentRiceAdaptationEnabledFlag( m_persistentRiceAdaptationEnabledFlag );
  m_cTEncTop.setCabacBypassAlignmentEnabledFlag( m_cabacBypassAlignmentEnabledFlag );
  m_cTEncTop.setLog2MaxTransformSkipBlockSize( m_log2MaxTransformSkipBlockSize );
  for ( UInt signallingModeIndex = 0; signallingModeIndex < NUMBER_OF_RDPCM_SIGNALLING_MODES; signallingModeIndex++ ) {
    m_cTEncTop.setRdpcmEnabledFlag( RDPCMSignallingMode( signallingModeIndex ),
                                    m_rdpcmEnabledFlag[signallingModeIndex] );
  }
  m_cTEncTop.setUseConstrainedIntraPred( m_bUseConstrainedIntraPred );
  m_cTEncTop.setFastUDIUseMPMEnabled( m_bFastUDIUseMPMEnabled );
  m_cTEncTop.setFastMEForGenBLowDelayEnabled( m_bFastMEForGenBLowDelayEnabled );
  m_cTEncTop.setUseBLambdaForNonKeyLowDelayPictures( m_bUseBLambdaForNonKeyLowDelayPictures );
  m_cTEncTop.setPCMLog2MinSize( m_uiPCMLog2MinSize );
  m_cTEncTop.setUsePCM( m_usePCM );

  // set internal bit-depth and constants
  for ( UInt channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++ ) {
    m_cTEncTop.setBitDepth( (ChannelType)channelType, m_internalBitDepth[channelType] );
    m_cTEncTop.setPCMBitDepth( (ChannelType)channelType, m_bPCMInputBitDepthFlag ? m_MSBExtendedBitDepth[channelType]
                                                                                 : m_internalBitDepth[channelType] );
  }
  m_cTEncTop.setPCMLog2MaxSize( m_pcmLog2MaxSize );
  m_cTEncTop.setMaxNumMergeCand( m_maxNumMergeCand );

  //====== Weighted Prediction ========
  m_cTEncTop.setUseWP( m_useWeightedPred );
  m_cTEncTop.setWPBiPred( m_useWeightedBiPred );

  //====== Parallel Merge Estimation ========
  m_cTEncTop.setLog2ParallelMergeLevelMinus2( m_log2ParallelMergeLevel - 2 );

  //====== Slice ========
  m_cTEncTop.setSliceMode( m_sliceMode );
  m_cTEncTop.setSliceArgument( m_sliceArgument );

  //====== Dependent Slice ========
  m_cTEncTop.setSliceSegmentMode( m_sliceSegmentMode );
  m_cTEncTop.setSliceSegmentArgument( m_sliceSegmentArgument );

  if ( m_sliceMode == NO_SLICES ) { m_bLFCrossSliceBoundaryFlag = true; }
  m_cTEncTop.setLFCrossSliceBoundaryFlag( m_bLFCrossSliceBoundaryFlag );
  m_cTEncTop.setUseSAO( m_bUseSAO );
  m_cTEncTop.setTestSAODisableAtPictureLevel( m_bTestSAODisableAtPictureLevel );
  m_cTEncTop.setSaoEncodingRate( m_saoEncodingRate );
  m_cTEncTop.setSaoEncodingRateChroma( m_saoEncodingRateChroma );
  m_cTEncTop.setMaxNumOffsetsPerPic( m_maxNumOffsetsPerPic );
  m_cTEncTop.setSaoCtuBoundary( m_saoCtuBoundary );
#if ADD_RESET_ENCODER_DECISIONS_AFTER_IRAP
  m_cTEncTop.setResetEncoderStateAfterIRAP( m_resetEncoderStateAfterIRAP );
#else
  m_cTEncTop.setSaoResetEncoderStateAfterIRAP( m_saoResetEncoderStateAfterIRAP );
#endif
  m_cTEncTop.setPCMInputBitDepthFlag( m_bPCMInputBitDepthFlag );
  m_cTEncTop.setPCMFilterDisableFlag( m_bPCMFilterDisableFlag );
  m_cTEncTop.setIntraSmoothingDisabledFlag( !m_enableIntraReferenceSmoothing );
  m_cTEncTop.setDecodedPictureHashSEIType( m_decodedPictureHashSEIType );
  m_cTEncTop.setRecoveryPointSEIEnabled( m_recoveryPointSEIEnabled );
  m_cTEncTop.setBufferingPeriodSEIEnabled( m_bufferingPeriodSEIEnabled );
  m_cTEncTop.setPictureTimingSEIEnabled( m_pictureTimingSEIEnabled );
  m_cTEncTop.setToneMappingInfoSEIEnabled( m_toneMappingInfoSEIEnabled );
  m_cTEncTop.setTMISEIToneMapId( m_toneMapId );
  m_cTEncTop.setTMISEIToneMapCancelFlag( m_toneMapCancelFlag );
  m_cTEncTop.setTMISEIToneMapPersistenceFlag( m_toneMapPersistenceFlag );
  m_cTEncTop.setTMISEICodedDataBitDepth( m_toneMapCodedDataBitDepth );
  m_cTEncTop.setTMISEITargetBitDepth( m_toneMapTargetBitDepth );
  m_cTEncTop.setTMISEIModelID( m_toneMapModelId );
  m_cTEncTop.setTMISEIMinValue( m_toneMapMinValue );
  m_cTEncTop.setTMISEIMaxValue( m_toneMapMaxValue );
  m_cTEncTop.setTMISEISigmoidMidpoint( m_sigmoidMidpoint );
  m_cTEncTop.setTMISEISigmoidWidth( m_sigmoidWidth );
  m_cTEncTop.setTMISEIStartOfCodedInterva( m_startOfCodedInterval );
  m_cTEncTop.setTMISEINumPivots( m_numPivots );
  m_cTEncTop.setTMISEICodedPivotValue( m_codedPivotValue );
  m_cTEncTop.setTMISEITargetPivotValue( m_targetPivotValue );
  m_cTEncTop.setTMISEICameraIsoSpeedIdc( m_cameraIsoSpeedIdc );
  m_cTEncTop.setTMISEICameraIsoSpeedValue( m_cameraIsoSpeedValue );
  m_cTEncTop.setTMISEIExposureIndexIdc( m_exposureIndexIdc );
  m_cTEncTop.setTMISEIExposureIndexValue( m_exposureIndexValue );
  m_cTEncTop.setTMISEIExposureCompensationValueSignFlag( m_exposureCompensationValueSignFlag );
  m_cTEncTop.setTMISEIExposureCompensationValueNumerator( m_exposureCompensationValueNumerator );
  m_cTEncTop.setTMISEIExposureCompensationValueDenomIdc( m_exposureCompensationValueDenomIdc );
  m_cTEncTop.setTMISEIRefScreenLuminanceWhite( m_refScreenLuminanceWhite );
  m_cTEncTop.setTMISEIExtendedRangeWhiteLevel( m_extendedRangeWhiteLevel );
  m_cTEncTop.setTMISEINominalBlackLevelLumaCodeValue( m_nominalBlackLevelLumaCodeValue );
  m_cTEncTop.setTMISEINominalWhiteLevelLumaCodeValue( m_nominalWhiteLevelLumaCodeValue );
  m_cTEncTop.setTMISEIExtendedWhiteLevelLumaCodeValue( m_extendedWhiteLevelLumaCodeValue );
  m_cTEncTop.setChromaResamplingFilterHintEnabled( m_chromaResamplingFilterSEIenabled );
  m_cTEncTop.setChromaResamplingHorFilterIdc( m_chromaResamplingHorFilterIdc );
  m_cTEncTop.setChromaResamplingVerFilterIdc( m_chromaResamplingVerFilterIdc );
  m_cTEncTop.setFramePackingArrangementSEIEnabled( m_framePackingSEIEnabled );
  m_cTEncTop.setFramePackingArrangementSEIType( m_framePackingSEIType );
  m_cTEncTop.setFramePackingArrangementSEIId( m_framePackingSEIId );
  m_cTEncTop.setFramePackingArrangementSEIQuincunx( m_framePackingSEIQuincunx );
  m_cTEncTop.setFramePackingArrangementSEIInterpretation( m_framePackingSEIInterpretation );
  m_cTEncTop.setSegmentedRectFramePackingArrangementSEIEnabled( m_segmentedRectFramePackingSEIEnabled );
  m_cTEncTop.setSegmentedRectFramePackingArrangementSEICancel( m_segmentedRectFramePackingSEICancel );
  m_cTEncTop.setSegmentedRectFramePackingArrangementSEIType( m_segmentedRectFramePackingSEIType );
  m_cTEncTop.setSegmentedRectFramePackingArrangementSEIPersistence( m_segmentedRectFramePackingSEIPersistence );
  m_cTEncTop.setDisplayOrientationSEIAngle( m_displayOrientationSEIAngle );
  m_cTEncTop.setTemporalLevel0IndexSEIEnabled( m_temporalLevel0IndexSEIEnabled );
  m_cTEncTop.setGradualDecodingRefreshInfoEnabled( m_gradualDecodingRefreshInfoEnabled );
  m_cTEncTop.setNoDisplaySEITLayer( m_noDisplaySEITLayer );
  m_cTEncTop.setDecodingUnitInfoSEIEnabled( m_decodingUnitInfoSEIEnabled );
  m_cTEncTop.setSOPDescriptionSEIEnabled( m_SOPDescriptionSEIEnabled );
  m_cTEncTop.setScalableNestingSEIEnabled( m_scalableNestingSEIEnabled );
  m_cTEncTop.setTMCTSSEIEnabled( m_tmctsSEIEnabled );
#if MCTS_ENC_CHECK
  m_cTEncTop.setTMCTSSEITileConstraint( m_tmctsSEITileConstraint );
#endif
#if MCTS_EXTRACTION
  m_cTEncTop.setTMCTSExtractionSEIEnabled( m_tmctsExtractionSEIEnabled );
#endif
  m_cTEncTop.setTimeCodeSEIEnabled( m_timeCodeSEIEnabled );
  m_cTEncTop.setNumberOfTimeSets( m_timeCodeSEINumTs );
  for ( Int i = 0; i < m_timeCodeSEINumTs; i++ ) { m_cTEncTop.setTimeSet( m_timeSetArray[i], i ); }
  m_cTEncTop.setKneeSEIEnabled( m_kneeSEIEnabled );
  m_cTEncTop.setKneeFunctionInformationSEI( m_kneeFunctionInformationSEI );
#if CCV_SEI_MESSAGE
  m_cTEncTop.setCcvSEIEnabled( m_ccvSEIEnabled );
  m_cTEncTop.setCcvSEICancelFlag( m_ccvSEICancelFlag );
  m_cTEncTop.setCcvSEIPersistenceFlag( m_ccvSEIPersistenceFlag );
  m_cTEncTop.setCcvSEIEnabled( m_ccvSEIEnabled );
  m_cTEncTop.setCcvSEICancelFlag( m_ccvSEICancelFlag );
  m_cTEncTop.setCcvSEIPersistenceFlag( m_ccvSEIPersistenceFlag );
  m_cTEncTop.setCcvSEIPrimariesPresentFlag( m_ccvSEIPrimariesPresentFlag );
  m_cTEncTop.setCcvSEIMinLuminanceValuePresentFlag( m_ccvSEIMinLuminanceValuePresentFlag );
  m_cTEncTop.setCcvSEIMaxLuminanceValuePresentFlag( m_ccvSEIMaxLuminanceValuePresentFlag );
  m_cTEncTop.setCcvSEIAvgLuminanceValuePresentFlag( m_ccvSEIAvgLuminanceValuePresentFlag );
  for ( Int i = 0; i < MAX_NUM_COMPONENT; i++ ) {
    m_cTEncTop.setCcvSEIPrimariesX( m_ccvSEIPrimariesX[i], i );
    m_cTEncTop.setCcvSEIPrimariesY( m_ccvSEIPrimariesY[i], i );
  }
  m_cTEncTop.setCcvSEIMinLuminanceValue( m_ccvSEIMinLuminanceValue );
  m_cTEncTop.setCcvSEIMaxLuminanceValue( m_ccvSEIMaxLuminanceValue );
  m_cTEncTop.setCcvSEIAvgLuminanceValue( m_ccvSEIAvgLuminanceValue );
#endif
#if ERP_SR_OV_SEI_MESSAGE
  m_cTEncTop.setErpSEIEnabled( m_erpSEIEnabled );
  m_cTEncTop.setErpSEICancelFlag( m_erpSEICancelFlag );
  m_cTEncTop.setErpSEIPersistenceFlag( m_erpSEIPersistenceFlag );
  m_cTEncTop.setErpSEIGuardBandFlag( m_erpSEIGuardBandFlag );
  m_cTEncTop.setErpSEIGuardBandType( m_erpSEIGuardBandType );
  m_cTEncTop.setErpSEILeftGuardBandWidth( m_erpSEILeftGuardBandWidth );
  m_cTEncTop.setErpSEIRightGuardBandWidth( m_erpSEIRightGuardBandWidth );
  m_cTEncTop.setSphereRotationSEIEnabled( m_sphereRotationSEIEnabled );
  m_cTEncTop.setSphereRotationSEICancelFlag( m_sphereRotationSEICancelFlag );
  m_cTEncTop.setSphereRotationSEIPersistenceFlag( m_sphereRotationSEIPersistenceFlag );
  m_cTEncTop.setSphereRotationSEIYaw( m_sphereRotationSEIYaw );
  m_cTEncTop.setSphereRotationSEIPitch( m_sphereRotationSEIPitch );
  m_cTEncTop.setSphereRotationSEIRoll( m_sphereRotationSEIRoll );
  m_cTEncTop.setOmniViewportSEIEnabled( m_omniViewportSEIEnabled );
  m_cTEncTop.setOmniViewportSEIId( m_omniViewportSEIId );
  m_cTEncTop.setOmniViewportSEICancelFlag( m_omniViewportSEICancelFlag );
  m_cTEncTop.setOmniViewportSEIPersistenceFlag( m_omniViewportSEIPersistenceFlag );
  m_cTEncTop.setOmniViewportSEICntMinus1( m_omniViewportSEICntMinus1 );
  m_cTEncTop.setOmniViewportSEIAzimuthCentre( m_omniViewportSEIAzimuthCentre );
  m_cTEncTop.setOmniViewportSEIElevationCentre( m_omniViewportSEIElevationCentre );
  m_cTEncTop.setOmniViewportSEITiltCentre( m_omniViewportSEITiltCentre );
  m_cTEncTop.setOmniViewportSEIHorRange( m_omniViewportSEIHorRange );
  m_cTEncTop.setOmniViewportSEIVerRange( m_omniViewportSEIVerRange );
#endif
#if CMP_SEI_MESSAGE
  m_cTEncTop.setCmpSEIEnabled( m_cmpSEIEnabled );
  m_cTEncTop.setCmpSEICmpCancelFlag( m_cmpSEICmpCancelFlag );
  m_cTEncTop.setCmpSEICmpPersistenceFlag( m_cmpSEICmpPersistenceFlag );
#endif
#if RWP_SEI_MESSAGE
  m_cTEncTop.setRwpSEIEnabled( m_rwpSEIEnabled );
  m_cTEncTop.setRwpSEIRwpCancelFlag( m_rwpSEIRwpCancelFlag );
  m_cTEncTop.setRwpSEIRwpPersistenceFlag( m_rwpSEIRwpPersistenceFlag );
  m_cTEncTop.setRwpSEIConstituentPictureMatchingFlag( m_rwpSEIConstituentPictureMatchingFlag );
  m_cTEncTop.setRwpSEINumPackedRegions( m_rwpSEINumPackedRegions );
  m_cTEncTop.setRwpSEIProjPictureWidth( m_rwpSEIProjPictureWidth );
  m_cTEncTop.setRwpSEIProjPictureHeight( m_rwpSEIProjPictureHeight );
  m_cTEncTop.setRwpSEIPackedPictureWidth( m_rwpSEIPackedPictureWidth );
  m_cTEncTop.setRwpSEIPackedPictureHeight( m_rwpSEIPackedPictureHeight );
  m_cTEncTop.setRwpSEIRwpTransformType( m_rwpSEIRwpTransformType );
  m_cTEncTop.setRwpSEIRwpGuardBandFlag( m_rwpSEIRwpGuardBandFlag );
  m_cTEncTop.setRwpSEIProjRegionWidth( m_rwpSEIProjRegionWidth );
  m_cTEncTop.setRwpSEIProjRegionHeight( m_rwpSEIProjRegionHeight );
  m_cTEncTop.setRwpSEIRwpSEIProjRegionTop( m_rwpSEIRwpSEIProjRegionTop );
  m_cTEncTop.setRwpSEIProjRegionLeft( m_rwpSEIProjRegionLeft );
  m_cTEncTop.setRwpSEIPackedRegionWidth( m_rwpSEIPackedRegionWidth );
  m_cTEncTop.setRwpSEIPackedRegionHeight( m_rwpSEIPackedRegionHeight );
  m_cTEncTop.setRwpSEIPackedRegionTop( m_rwpSEIPackedRegionTop );
  m_cTEncTop.setRwpSEIPackedRegionLeft( m_rwpSEIPackedRegionLeft );
  m_cTEncTop.setRwpSEIRwpLeftGuardBandWidth( m_rwpSEIRwpLeftGuardBandWidth );
  m_cTEncTop.setRwpSEIRwpRightGuardBandWidth( m_rwpSEIRwpRightGuardBandWidth );
  m_cTEncTop.setRwpSEIRwpTopGuardBandHeight( m_rwpSEIRwpTopGuardBandHeight );
  m_cTEncTop.setRwpSEIRwpBottomGuardBandHeight( m_rwpSEIRwpBottomGuardBandHeight );
  m_cTEncTop.setRwpSEIRwpGuardBandNotUsedForPredFlag( m_rwpSEIRwpGuardBandNotUsedForPredFlag );
  m_cTEncTop.setRwpSEIRwpGuardBandType( m_rwpSEIRwpGuardBandType );
#endif
  m_cTEncTop.setColourRemapInfoSEIFileRoot( m_colourRemapSEIFileRoot );
  m_cTEncTop.setMasteringDisplaySEI( m_masteringDisplay );
  m_cTEncTop.setSEIAlternativeTransferCharacteristicsSEIEnable( m_preferredTransferCharacteristics >= 0 );
  m_cTEncTop.setSEIPreferredTransferCharacteristics( UChar( m_preferredTransferCharacteristics ) );
  m_cTEncTop.setSEIGreenMetadataInfoSEIEnable( m_greenMetadataType > 0 );
  m_cTEncTop.setSEIGreenMetadataType( UChar( m_greenMetadataType ) );
  m_cTEncTop.setSEIXSDMetricType( UChar( m_xsdMetricType ) );
#if RNSEI
  m_cTEncTop.setRegionalNestingSEIFileRoot( m_regionalNestingSEIFileRoot );
#endif
  m_cTEncTop.setTileUniformSpacingFlag( m_tileUniformSpacingFlag );
  m_cTEncTop.setNumColumnsMinus1( m_numTileColumnsMinus1 );
  m_cTEncTop.setNumRowsMinus1( m_numTileRowsMinus1 );
  if ( !m_tileUniformSpacingFlag ) {
    m_cTEncTop.setColumnWidth( m_tileColumnWidth );
    m_cTEncTop.setRowHeight( m_tileRowHeight );
  }
  m_cTEncTop.xCheckGSParameters();
  Int uiTilesCount = ( m_numTileRowsMinus1 + 1 ) * ( m_numTileColumnsMinus1 + 1 );
  if ( uiTilesCount == 1 ) { m_bLFCrossTileBoundaryFlag = true; }
  m_cTEncTop.setLFCrossTileBoundaryFlag( m_bLFCrossTileBoundaryFlag );
  m_cTEncTop.setEntropyCodingSyncEnabledFlag( m_entropyCodingSyncEnabledFlag );
  m_cTEncTop.setTMVPModeId( m_TMVPModeId );
  m_cTEncTop.setUseScalingListId( m_useScalingListId );
  m_cTEncTop.setScalingListFileName( m_scalingListFileName );
  m_cTEncTop.setSignDataHidingEnabledFlag( m_signDataHidingEnabledFlag );
  m_cTEncTop.setUseRateCtrl( m_RCEnableRateControl );
  m_cTEncTop.setTargetBitrate( m_RCTargetBitrate );
  m_cTEncTop.setKeepHierBit( m_RCKeepHierarchicalBit );
  m_cTEncTop.setLCULevelRC( m_RCLCULevelRC );
  m_cTEncTop.setUseLCUSeparateModel( m_RCUseLCUSeparateModel );
  m_cTEncTop.setInitialQP( m_RCInitialQP );
  m_cTEncTop.setForceIntraQP( m_RCForceIntraQP );
  m_cTEncTop.setCpbSaturationEnabled( m_RCCpbSaturationEnabled );
  m_cTEncTop.setCpbSize( m_RCCpbSize );
  m_cTEncTop.setInitialCpbFullness( m_RCInitialCpbFullness );
  m_cTEncTop.setTransquantBypassEnabledFlag( m_TransquantBypassEnabledFlag );
  m_cTEncTop.setCUTransquantBypassFlagForceValue( m_CUTransquantBypassFlagForce );
  m_cTEncTop.setCostMode( m_costMode );
  m_cTEncTop.setUseRecalculateQPAccordingToLambda( m_recalculateQPAccordingToLambda );
  m_cTEncTop.setUseStrongIntraSmoothing( m_useStrongIntraSmoothing );
  m_cTEncTop.setActiveParameterSetsSEIEnabled( m_activeParameterSetsSEIEnabled );
  m_cTEncTop.setVuiParametersPresentFlag( m_vuiParametersPresentFlag );
  m_cTEncTop.setAspectRatioInfoPresentFlag( m_aspectRatioInfoPresentFlag );
  m_cTEncTop.setAspectRatioIdc( m_aspectRatioIdc );
  m_cTEncTop.setSarWidth( m_sarWidth );
  m_cTEncTop.setSarHeight( m_sarHeight );
  m_cTEncTop.setOverscanInfoPresentFlag( m_overscanInfoPresentFlag );
  m_cTEncTop.setOverscanAppropriateFlag( m_overscanAppropriateFlag );
  m_cTEncTop.setVideoSignalTypePresentFlag( m_videoSignalTypePresentFlag );
  m_cTEncTop.setVideoFormat( m_videoFormat );
  m_cTEncTop.setVideoFullRangeFlag( m_videoFullRangeFlag );
  m_cTEncTop.setColourDescriptionPresentFlag( m_colourDescriptionPresentFlag );
  m_cTEncTop.setColourPrimaries( m_colourPrimaries );
  m_cTEncTop.setTransferCharacteristics( m_transferCharacteristics );
  m_cTEncTop.setMatrixCoefficients( m_matrixCoefficients );
  m_cTEncTop.setChromaLocInfoPresentFlag( m_chromaLocInfoPresentFlag );
  m_cTEncTop.setChromaSampleLocTypeTopField( m_chromaSampleLocTypeTopField );
  m_cTEncTop.setChromaSampleLocTypeBottomField( m_chromaSampleLocTypeBottomField );
  m_cTEncTop.setNeutralChromaIndicationFlag( m_neutralChromaIndicationFlag );
  m_cTEncTop.setDefaultDisplayWindow( m_defDispWinLeftOffset, m_defDispWinRightOffset, m_defDispWinTopOffset,
                                      m_defDispWinBottomOffset );
  m_cTEncTop.setFrameFieldInfoPresentFlag( m_frameFieldInfoPresentFlag );
  m_cTEncTop.setPocProportionalToTimingFlag( m_pocProportionalToTimingFlag );
  m_cTEncTop.setNumTicksPocDiffOneMinus1( m_numTicksPocDiffOneMinus1 );
  m_cTEncTop.setBitstreamRestrictionFlag( m_bitstreamRestrictionFlag );
  m_cTEncTop.setTilesFixedStructureFlag( m_tilesFixedStructureFlag );
  m_cTEncTop.setMotionVectorsOverPicBoundariesFlag( m_motionVectorsOverPicBoundariesFlag );
  m_cTEncTop.setMinSpatialSegmentationIdc( m_minSpatialSegmentationIdc );
  m_cTEncTop.setMaxBytesPerPicDenom( m_maxBytesPerPicDenom );
  m_cTEncTop.setMaxBitsPerMinCuDenom( m_maxBitsPerMinCuDenom );
  m_cTEncTop.setLog2MaxMvLengthHorizontal( m_log2MaxMvLengthHorizontal );
  m_cTEncTop.setLog2MaxMvLengthVertical( m_log2MaxMvLengthVertical );
  m_cTEncTop.setEfficientFieldIRAPEnabled( m_bEfficientFieldIRAPEnabled );
  m_cTEncTop.setHarmonizeGopFirstFieldCoupleEnabled( m_bHarmonizeGopFirstFieldCoupleEnabled );

  m_cTEncTop.setSummaryOutFilename( m_summaryOutFilename );
  m_cTEncTop.setSummaryPicFilenameBase( m_summaryPicFilenameBase );
  m_cTEncTop.setSummaryVerboseness( m_summaryVerboseness );

  // SCM new added variables
  m_cTEncTop.setUseIntraBlockCopy( m_useIntraBlockCopy );
  m_cTEncTop.setUseIntraBlockCopyFastSearch( m_intraBlockCopyFastSearch );
  m_cTEncTop.setUseHashBasedIntraBCSearch( m_useHashBasedIntraBlockCopySearch );
  m_cTEncTop.setIntraBCSearchWidthInCTUs( m_intraBlockCopySearchWidthInCTUs );
  m_cTEncTop.setIntraBCNonHashSearchWidthInCTUs( m_intraBlockCopyNonHashSearchWidthInCTUs );
  m_cTEncTop.setUseHashBasedME( m_useHashBasedME );
  m_cTEncTop.setDisableIntraBoundaryFilter( m_disableIntraBoundaryFilter );
  m_cTEncTop.setUseColourTrans( m_useColourTrans );
  m_cTEncTop.setRGBFormatFlag( m_bRGBformat );
  m_cTEncTop.setUseLossless( m_useLL );
  m_cTEncTop.setActQpYOffset( m_actYQpOffset );
  m_cTEncTop.setActQpCbOffset( m_actCbQpOffset );
  m_cTEncTop.setActQpCrOffset( m_actCrQpOffset );
  m_cTEncTop.setTransquantBypassInferTUSplit( m_bTransquantBypassInferTUSplit );
  m_cTEncTop.setNoTUSplitIntraACTEnabled( m_bNoTUSplitIntraACTEnabled );
  m_cTEncTop.setMotionVectorResolutionControlIdc( m_motionVectorResolutionControlIdc );
  m_cTEncTop.setUsePaletteMode( m_usePaletteMode );
  m_cTEncTop.setPaletteMaxSize( m_paletteMaxSize );
  m_cTEncTop.setPaletteMaxPredSize( m_paletteMaxPredSize );
  m_cTEncTop.setPalettePredInSPSEnabled( m_palettePredInSPSEnabled );
  m_cTEncTop.setPalettePredInPPSEnabled( m_palettePredInPPSEnabled );
}

template <typename T>
Void PCCHMLibVideoEncoderImpl<T>::xGetBuffer( TComPicYuv*& rpcPicYuvRec ) {
  assert( m_iGOPSize > 0 );
  if ( m_cListPicYuvRec.size() >= (UInt)m_iGOPSize ) {
    rpcPicYuvRec = m_cListPicYuvRec.popFront();
  } else {
    rpcPicYuvRec = new TComPicYuv;
    rpcPicYuvRec->create( m_iSourceWidth, m_iSourceHeight, m_chromaFormatIDC, m_uiMaxCUWidth, m_uiMaxCUHeight,
                          m_uiMaxTotalCUDepth, true );
  }
  m_cListPicYuvRec.pushBack( rpcPicYuvRec );
}

template <typename T>
Void PCCHMLibVideoEncoderImpl<T>::xDeleteBuffer() {
  TComList<TComPicYuv*>::iterator iterPicYuvRec = m_cListPicYuvRec.begin();
  Int                             iSize         = Int( m_cListPicYuvRec.size() );
  for ( Int i = 0; i < iSize; i++ ) {
    TComPicYuv* pcPicYuvRec = *( iterPicYuvRec++ );
    pcPicYuvRec->destroy();
    delete pcPicYuvRec;
    pcPicYuvRec = NULL;
  }
}

/**
  Write access units to output file.
  \param bitstreamFile  target bitstream file
  \param iNumEncoded    number of encoded frames
  \param accessUnits    list of access units to be written
 */

template <typename T>
Void PCCHMLibVideoEncoderImpl<T>::xWriteOutput( std::ostream&                bitstreamFile,
                                                Int                          iNumEncoded,
                                                const std::list<AccessUnit>& accessUnits,
                                                PCCVideo<T, 3>&              videoRec ) {
  const InputColourSpaceConversion ipCSC =
      ( !m_outputInternalColourSpace ) ? m_inputColourSpaceConvert : IPCOLOURSPACE_UNCHANGED;
  if ( m_isField ) {
    // Reinterlace fields
    Int                              i;
    TComList<TComPicYuv*>::iterator  iterPicYuvRec = m_cListPicYuvRec.end();
    list<AccessUnit>::const_iterator iterBitstream = accessUnits.begin();
    for ( i = 0; i < iNumEncoded; i++ ) { --iterPicYuvRec; }
    for ( i = 0; i < iNumEncoded / 2; i++ ) {
      TComPicYuv* pcPicYuvRecTop = *( iterPicYuvRec++ );
      iterPicYuvRec++;
      xWritePicture( pcPicYuvRecTop, videoRec );
      const AccessUnit&   auTop    = *( iterBitstream++ );
      const vector<UInt>& statsTop = writeAnnexB( bitstreamFile, auTop );
      rateStatsAccum( auTop, statsTop );
      const AccessUnit&   auBottom    = *( iterBitstream++ );
      const vector<UInt>& statsBottom = writeAnnexB( bitstreamFile, auBottom );
      rateStatsAccum( auBottom, statsBottom );
    }
  } else {
    Int                              i;
    TComList<TComPicYuv*>::iterator  iterPicYuvRec = m_cListPicYuvRec.end();
    list<AccessUnit>::const_iterator iterBitstream = accessUnits.begin();
    for ( i = 0; i < iNumEncoded; i++ ) { --iterPicYuvRec; }
    for ( i = 0; i < iNumEncoded; i++ ) {
      TComPicYuv* pcPicYuvRec = *( iterPicYuvRec++ );
      xWritePicture( pcPicYuvRec, videoRec );
      const AccessUnit&   au    = *( iterBitstream++ );
      const vector<UInt>& stats = writeAnnexB( bitstreamFile, au );
      rateStatsAccum( au, stats );
    }
  }
}

template <typename T>
Void PCCHMLibVideoEncoderImpl<T>::rateStatsAccum( const AccessUnit& au, const std::vector<UInt>& annexBsizes ) {
  AccessUnit::const_iterator   it_au    = au.begin();
  vector<UInt>::const_iterator it_stats = annexBsizes.begin();
  for ( ; it_au != au.end(); it_au++, it_stats++ ) {
    switch ( ( *it_au )->m_nalUnitType ) {
      case NAL_UNIT_CODED_SLICE_TRAIL_R:
      case NAL_UNIT_CODED_SLICE_TRAIL_N:
      case NAL_UNIT_CODED_SLICE_TSA_R:
      case NAL_UNIT_CODED_SLICE_TSA_N:
      case NAL_UNIT_CODED_SLICE_STSA_R:
      case NAL_UNIT_CODED_SLICE_STSA_N:
      case NAL_UNIT_CODED_SLICE_BLA_W_LP:
      case NAL_UNIT_CODED_SLICE_BLA_W_RADL:
      case NAL_UNIT_CODED_SLICE_BLA_N_LP:
      case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
      case NAL_UNIT_CODED_SLICE_IDR_N_LP:
      case NAL_UNIT_CODED_SLICE_CRA:
      case NAL_UNIT_CODED_SLICE_RADL_N:
      case NAL_UNIT_CODED_SLICE_RADL_R:
      case NAL_UNIT_CODED_SLICE_RASL_N:
      case NAL_UNIT_CODED_SLICE_RASL_R:
      case NAL_UNIT_VPS:
      case NAL_UNIT_SPS:
      case NAL_UNIT_PPS: m_essentialBytes += *it_stats; break;
      default: break;
    }
    m_totalBytes += *it_stats;
  }
}

template <typename T>
Void PCCHMLibVideoEncoderImpl<T>::printRateSummary() {
  Double time = (Double)m_iFrameRcvd / m_iFrameRate * m_temporalSubsampleRatio;
  printf( "Bytes written to file: %u (%.3f kbps)\n", m_totalBytes, 0.008 * m_totalBytes / time );
  if ( m_summaryVerboseness > 0 ) {
    printf( "Bytes for SPS/PPS/Slice (Incl. Annex B): %u (%.3f kbps)\n", m_essentialBytes,
            0.008 * m_essentialBytes / time );
  }
}

template <typename T>
Void PCCHMLibVideoEncoderImpl<T>::printChromaFormat() {
  std::cout << std::setw( 43 ) << "Input ChromaFormatIDC = ";
  switch ( m_InputChromaFormatIDC ) {
    case CHROMA_400: std::cout << "  4:0:0"; break;
    case CHROMA_420: std::cout << "  4:2:0"; break;
    case CHROMA_422: std::cout << "  4:2:2"; break;
    case CHROMA_444: std::cout << "  4:4:4"; break;
    default: std::cerr << "Invalid"; exit( 1 );
  }
  std::cout << std::endl;

  std::cout << std::setw( 43 ) << "Output (internal) ChromaFormatIDC = ";
  switch ( m_cTEncTop.getChromaFormatIdc() ) {
    case CHROMA_400: std::cout << "  4:0:0"; break;
    case CHROMA_420: std::cout << "  4:2:0"; break;
    case CHROMA_422: std::cout << "  4:2:2"; break;
    case CHROMA_444: std::cout << "  4:4:4"; break;
    default: std::cerr << "Invalid"; exit( 1 );
  }
  std::cout << "\n" << std::endl;
}

template <typename T>
Void PCCHMLibVideoEncoderImpl<T>::xWritePicture( const TComPicYuv* pic, PCCVideo<T, 3>& video ) {
  video.resize( video.getFrameCount() + 1 );
  auto&          image           = video.getFrames().back();
  int            chromaSubsample = pic->getWidth( COMPONENT_Y ) / pic->getWidth( COMPONENT_Cb );
  int            width           = m_iSourceWidth - m_confWinLeft - m_confWinRight;
  int            height          = m_iSourceHeight - m_confWinTop - m_confWinBottom;
  PCCCOLORFORMAT format          = m_cTEncTop.getChromaFormatIdc() == CHROMA_420
                              ? PCCCOLORFORMAT::YUV420
                              : m_bRGBformat ? PCCCOLORFORMAT::RGB444 : PCCCOLORFORMAT::YUV444;
  image.set( pic->getAddr( COMPONENT_Y ), pic->getAddr( COMPONENT_Cb ), pic->getAddr( COMPONENT_Cr ), width, height,
             pic->getStride( COMPONENT_Y ), width / chromaSubsample, height / chromaSubsample,
             pic->getStride( COMPONENT_Cb ), m_internalBitDepth[0] - m_outputBitDepth[0], format,
             m_inputColourSpaceConvert == IPCOLOURSPACE_RGBtoGBR );
}

template <typename T>
Void PCCHMLibVideoEncoderImpl<T>::xReadPicture( TComPicYuv* pic, PCCVideo<T, 3>& video, int frameIndex ) {
  auto& image = video.getFrame( frameIndex );
  image.get( pic->getAddr( COMPONENT_Y ), pic->getAddr( COMPONENT_Cb ), pic->getAddr( COMPONENT_Cr ),
             pic->getWidth( COMPONENT_Y ), pic->getHeight( COMPONENT_Y ), pic->getStride( COMPONENT_Y ),
             pic->getWidth( COMPONENT_Cb ), pic->getHeight( COMPONENT_Cb ), pic->getStride( COMPONENT_Cb ),
             m_internalBitDepth[0] - m_outputBitDepth[0], m_inputColourSpaceConvert == IPCOLOURSPACE_RGBtoGBR );
}

template class pcc::PCCHMLibVideoEncoderImpl<uint8_t>;
template class pcc::PCCHMLibVideoEncoderImpl<uint16_t>;

#endif
