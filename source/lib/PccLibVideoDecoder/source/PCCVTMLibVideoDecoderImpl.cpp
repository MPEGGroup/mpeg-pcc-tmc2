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
#include "PCCCommon.h"

#ifdef USE_VTMLIB_VIDEO_CODEC

#include "PCCVTMLibVideoDecoderImpl.h"

using namespace pcc;

template <typename T>
PCCVTMLibVideoDecoderImpl<T>::PCCVTMLibVideoDecoderImpl() : m_iPOCLastDisplay( -MAX_INT ) {
  for ( int i = 0; i < MAX_NUM_LAYER_IDS; i++ ) { m_newCLVS[i] = true; }
}

template <typename T>
PCCVTMLibVideoDecoderImpl<T>::~PCCVTMLibVideoDecoderImpl() {}

template <typename T>
uint32_t PCCVTMLibVideoDecoderImpl<T>::decode( PCCVideoBitstream& bitstream,
                                               size_t             outputBitDepth,
                                               PCCVideo<T, 3>&    video ) {
  std::string        s( reinterpret_cast<char*>( bitstream.buffer() ), bitstream.size() );
  std::istringstream iss( s );
  std::istream&      bitstreamFile = iss;
  int                poc;
  PicList*           pcListPic = NULL;

  InputByteStream bytestream( bitstreamFile );
  if ( outputBitDepth ) {
    m_outputBitDepth[CHANNEL_TYPE_LUMA]   = outputBitDepth;
    m_outputBitDepth[CHANNEL_TYPE_CHROMA] = outputBitDepth;
  }
  video.clear();

  // create & initialize internal classes
  m_targetSubPicIdx = 0;
  xCreateDecLib();

  m_iPOCLastDisplay += m_iSkipFrame;  // set the last displayed POC correctly for skip forward.

  // clear contents of colour-remap-information-SEI output file
  if ( !m_colourRemapSEIFileName.empty() ) {
    std::ofstream ofile( m_colourRemapSEIFileName.c_str() );
    if ( !ofile.good() || !ofile.is_open() ) {
      EXIT( "Unable to open file " << m_colourRemapSEIFileName.c_str()
                                   << " for writing colour-remap-information-SEI video" );
    }
  }

  // clear contents of annotated-Regions-SEI output file
  if ( !m_annotatedRegionsSEIFileName.empty() ) {
    std::ofstream ofile( m_annotatedRegionsSEIFileName.c_str() );
    if ( !ofile.good() || !ofile.is_open() ) {
      fprintf( stderr, "\nUnable to open file '%s' for writing annotated-Regions-SEI\n",
               m_annotatedRegionsSEIFileName.c_str() );
      exit( EXIT_FAILURE );
    }
  }

  // main decoder loop
  bool loopFiltered[MAX_VPS_LAYERS] = {false};
  bool bPicSkipped                  = false;

  bool isEosPresentInPu     = false;
  bool isEosPresentInLastPu = false;
#if JVET_S0078_NOOUTPUTPRIORPICFLAG
  bool firstSliceInAU = true;
#endif

  bool outputPicturePresentInBitstream = false;
  auto setOutputPicturePresentInStream = [&]() {
    if ( !outputPicturePresentInBitstream ) {
      PicList::iterator iterPic = pcListPic->begin();
      while ( !outputPicturePresentInBitstream && iterPic != pcListPic->end() ) {
        Picture* pcPic = *( iterPic++ );
        if ( pcPic->neededForOutput ) { outputPicturePresentInBitstream = true; }
      }
    }
  };

  m_cDecLib.setHTidExternalSetFlag( m_mTidExternalSet );
  m_cDecLib.setTOlsIdxExternalFlag( m_tOlsIdxTidExternalSet );

  while ( !!bitstreamFile ) {
    InputNALUnit nalu;
    nalu.m_nalUnitType = NAL_UNIT_INVALID;

    // determine if next NAL unit will be the first one from a new picture
    bool bNewPicture    = isNewPicture( &bitstreamFile, &bytestream );
    bool bNewAccessUnit = bNewPicture && isNewAccessUnit( bNewPicture, &bitstreamFile, &bytestream );
    if ( !bNewPicture ) {
      AnnexBStats stats = AnnexBStats();

      // find next NAL unit in stream
      byteStreamNALUnit( bytestream, nalu.getBitstream().getFifo(), stats );
      if ( nalu.getBitstream().getFifo().empty() ) {
        /* this can happen if the following occur:
         *  - empty input file
         *  - two back-to-back start_code_prefixes
         *  - start_code_prefix immediately followed by EOF
         */
        msg( VTM_ERROR, "Warning: Attempt to decode an empty NAL unit\n" );
      } else {
        // read NAL unit header
        read( nalu );

        // flush output for first slice of an IDR picture
        if ( m_cDecLib.getFirstSliceInPicture() && ( nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
                                                     nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ) ) {
          m_newCLVS[nalu.m_nuhLayerId] = true;  // An IDR picture starts a new CLVS
#if !JVET_S0078_NOOUTPUTPRIORPICFLAG
          xFlushOutput( pcListPic, video );
#endif
        } else if ( m_cDecLib.getFirstSliceInPicture() && nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA &&
                    isEosPresentInLastPu ) {
          // A CRA that is immediately preceded by an EOS is a CLVSS
          m_newCLVS[nalu.m_nuhLayerId] = true;
#if !JVET_S0078_NOOUTPUTPRIORPICFLAG
          xFlushOutput( pcListPic, video );
#endif
        } else if ( m_cDecLib.getFirstSliceInPicture() && nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA &&
                    !isEosPresentInLastPu ) {
          // A CRA that is not immediately precede by an EOS is not a CLVSS
          m_newCLVS[nalu.m_nuhLayerId] = false;
        } else if ( m_cDecLib.getFirstSliceInPicture() && !isEosPresentInLastPu ) {
          m_newCLVS[nalu.m_nuhLayerId] = false;
        }

        // parse NAL unit syntax if within target decoding layer
        if ( ( m_iMaxTemporalLayer < 0 || nalu.m_temporalId <= m_iMaxTemporalLayer ) &&
             xIsNaluWithinTargetDecLayerIdSet( &nalu ) ) {
          CHECK( nalu.m_temporalId > m_iMaxTemporalLayer,
                 "bitstream shall not include any NAL unit with TemporalId greater than HighestTid" );
          if ( m_targetDecLayerIdSet.size() ) {
            CHECK( std::find( m_targetDecLayerIdSet.begin(), m_targetDecLayerIdSet.end(), nalu.m_nuhLayerId ) ==
                       m_targetDecLayerIdSet.end(),
                   "bitstream shall not contain any other layers than included in the OLS with OlsIdx" );
          }
          if ( bPicSkipped ) {
            if ( ( nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_TRAIL ) ||
                 ( nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_STSA ) ||
                 ( nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_RASL ) ||
                 ( nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_RADL ) ||
                 ( nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ) ||
                 ( nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ) ||
                 ( nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA ) ||
                 ( nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_GDR ) ) {
              if ( m_cDecLib.isSliceNaluFirstInAU( true, nalu ) ) {
                m_cDecLib.resetAccessUnitNals();
                m_cDecLib.resetAccessUnitApsNals();
                m_cDecLib.resetAccessUnitPicInfo();
              }
              bPicSkipped = false;
            }
          }
          m_cDecLib.decode( nalu, m_iSkipFrame, m_iPOCLastDisplay, m_targetOlsIdx );
          if ( nalu.m_nalUnitType == NAL_UNIT_OPI ) {
            if ( !m_cDecLib.getHTidExternalSetFlag() && m_cDecLib.getOPI()->getHtidInfoPresentFlag() ) {
              m_iMaxTemporalLayer = m_cDecLib.getOPI()->getOpiHtidPlus1() - 1;
            }
            m_cDecLib.setHTidOpiSetFlag( m_cDecLib.getOPI()->getHtidInfoPresentFlag() );
          }
          if ( nalu.m_nalUnitType == NAL_UNIT_VPS ) {
            m_cDecLib.deriveTargetOutputLayerSet( m_cDecLib.getVPS()->m_targetOlsIdx );
            m_targetDecLayerIdSet    = m_cDecLib.getVPS()->m_targetLayerIdSet;
            m_targetOutputLayerIdSet = m_cDecLib.getVPS()->m_targetOutputLayerIdSet;
          }
        } else {
          bPicSkipped = true;
        }
      }
      // once an EOS NAL unit appears in the current PU, mark the variable isEosPresentInPu as true
      if ( nalu.m_nalUnitType == NAL_UNIT_EOS ) {
        isEosPresentInPu = true;
        m_newCLVS[nalu.m_nuhLayerId] =
            true;  // The presence of EOS means that the next picture is the beginning of new CLVS
      }
      // within the current PU, only EOS and EOB are allowed to be sent after an EOS nal unit
      if ( isEosPresentInPu ) {
        CHECK( nalu.m_nalUnitType != NAL_UNIT_EOS && nalu.m_nalUnitType != NAL_UNIT_EOB,
               "When an EOS NAL unit is present in a PU, it shall be the last NAL unit among all NAL units within the "
               "PU other than other EOS NAL units or an EOB NAL unit" );
      }
    }

    if ( ( bNewPicture || !bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS ) &&
         !m_cDecLib.getFirstSliceInSequence( nalu.m_nuhLayerId ) && !bPicSkipped ) {
      if ( !loopFiltered[nalu.m_nuhLayerId] || bitstreamFile ) {
        m_cDecLib.executeLoopFilters();
        m_cDecLib.finishPicture( poc, pcListPic, VTM_INFO, m_newCLVS[nalu.m_nuhLayerId] );
      }
      loopFiltered[nalu.m_nuhLayerId] = ( nalu.m_nalUnitType == NAL_UNIT_EOS );
      if ( nalu.m_nalUnitType == NAL_UNIT_EOS ) { m_cDecLib.setFirstSliceInSequence( true, nalu.m_nuhLayerId ); }

      m_cDecLib.updateAssociatedIRAP();
      m_cDecLib.updatePrevGDRInSameLayer();
      m_cDecLib.updatePrevIRAPAndGDRSubpic();
    } else if ( ( bNewPicture || !bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS ) &&
                m_cDecLib.getFirstSliceInSequence( nalu.m_nuhLayerId ) ) {
      m_cDecLib.setFirstSliceInPicture( true );
    }

    if ( pcListPic ) {
      //      if( !m_reconFileName.empty() && !m_cVideoIOYuvReconFile[nalu.m_nuhLayerId].isOpen() )
      //      {
      //        const BitDepths &bitDepths=pcListPic->front()->cs->sps->getBitDepths(); // use bit depths of first
      //        reconstructed picture. for( uint32_t channelType = 0; channelType < MAX_NUM_CHANNEL_TYPE; channelType++
      //        )
      //        {
      //            if( m_outputBitDepth[channelType] == 0 )
      //            {
      //                m_outputBitDepth[channelType] = bitDepths.recon[channelType];
      //            }
      //        }
      //
      //        if (m_packedYUVMode && (m_outputBitDepth[CH_L] != 10 && m_outputBitDepth[CH_L] != 12))
      //        {
      //          EXIT ("Invalid output bit-depth for packed YUV output, aborting\n");
      //        }
      //
      //        std::string reconFileName = m_reconFileName;
      //        if( m_reconFileName.compare( "/dev/null" ) && m_cDecLib.getVPS() != nullptr &&
      //        m_cDecLib.getVPS()->getMaxLayers() > 1 && xIsNaluWithinTargetOutputLayerIdSet( &nalu ) )
      //        {
      //          size_t pos = reconFileName.find_last_of('.');
      //          if (pos != std::string::npos)
      //          {
      //            reconFileName.insert( pos, std::to_string( nalu.m_nuhLayerId ) );
      //          }
      //          else
      //          {
      //            reconFileName.append( std::to_string( nalu.m_nuhLayerId ) );
      //          }
      //        }
      //        if( ( m_cDecLib.getVPS() != nullptr && ( m_cDecLib.getVPS()->getMaxLayers() == 1 ||
      //        xIsNaluWithinTargetOutputLayerIdSet( &nalu ) ) ) || m_cDecLib.getVPS() == nullptr )
      //        {
      //          m_cVideoIOYuvReconFile[nalu.m_nuhLayerId].open( reconFileName, true, m_outputBitDepth,
      //          m_outputBitDepth, bitDepths.recon ); // write mode
      //        }
      //      }
      // write reconstruction to file
      if ( bNewPicture ) {
        setOutputPicturePresentInStream();
        setVideoSize( pcListPic->front()->cs->sps );
        xWriteOutput( pcListPic, nalu.m_temporalId, video );
      }
      if ( nalu.m_nalUnitType == NAL_UNIT_EOS ) {
        if ( !m_annotatedRegionsSEIFileName.empty() && bNewPicture ) { xOutputAnnotatedRegions( pcListPic ); }
        setOutputPicturePresentInStream();
        setVideoSize( pcListPic->front()->cs->sps );
        xWriteOutput( pcListPic, nalu.m_temporalId, video );
        m_cDecLib.setFirstSliceInPicture( false );
      }
      // write reconstruction to file -- for additional bumping as defined in C.5.2.3
      if ( !bNewPicture && ( ( nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_TRAIL &&
                               nalu.m_nalUnitType <= NAL_UNIT_RESERVED_IRAP_VCL_11 ) ||
                             ( nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL &&
                               nalu.m_nalUnitType <= NAL_UNIT_CODED_SLICE_GDR ) ) ) {
        setOutputPicturePresentInStream();
#if JVET_S0078_NOOUTPUTPRIORPICFLAG
        bool firstPicInCVSAUThatIsNotAU0 = false;
        if ( firstSliceInAU ) {
          if ( m_targetDecLayerIdSet.size() > 0 ) {
            if ( m_cDecLib.getAudIrapOrGdrAuFlag() ) { firstPicInCVSAUThatIsNotAU0 = true; }
          } else {
            if ( nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
                 nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
                 ( nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA && isEosPresentInLastPu ) ) {
              firstPicInCVSAUThatIsNotAU0 = true;
            }
          }
        }
        if ( firstPicInCVSAUThatIsNotAU0 ) {
          setVideoSize( pcListPic->front()->cs->sps );
          xFlushOutput( pcListPic, video, m_cDecLib.getNoOutputPriorPicsFlag() );
        } else {
          setVideoSize( pcListPic->front()->cs->sps );
          xWriteOutput( pcListPic, video );
        }
#else
        setVideoSize( pcListPic->front()->cs->sps );
        xWriteOutput( pcListPic, nalu.m_temporalId, video );
#endif
      }
    }
#if JVET_S0078_NOOUTPUTPRIORPICFLAG
    if ( ( nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_TRAIL && nalu.m_nalUnitType <= NAL_UNIT_OPI ) ||
         ( nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_IDR_W_RADL && nalu.m_nalUnitType <= NAL_UNIT_CODED_SLICE_GDR ) ) {
      firstSliceInAU = false;
    }
#endif
    if ( bNewPicture ) {
      m_cDecLib.checkSeiInPictureUnit();
      m_cDecLib.resetPictureSeiNalus();
      // reset the EOS present status for the next PU check
      isEosPresentInLastPu = isEosPresentInPu;
      isEosPresentInPu     = false;
    }
    if ( bNewPicture || !bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS ) {
      m_cDecLib.checkAPSInPictureUnit();
      m_cDecLib.resetPictureUnitNals();
    }
    if ( bNewAccessUnit || !bitstreamFile ) {
      m_cDecLib.CheckNoOutputPriorPicFlagsInAccessUnit();
      m_cDecLib.resetAccessUnitNoOutputPriorPicFlags();
      m_cDecLib.checkLayerIdIncludedInCvss();
      m_cDecLib.resetAccessUnitEos();
      m_cDecLib.resetAudIrapOrGdrAuFlag();
    }
    if ( bNewAccessUnit ) {
      m_cDecLib.checkTidLayerIdInAccessUnit();
      m_cDecLib.resetAccessUnitSeiTids();
      m_cDecLib.checkSEIInAccessUnit();
      m_cDecLib.resetAccessUnitSeiPayLoadTypes();
      m_cDecLib.resetAccessUnitNals();
      m_cDecLib.resetAccessUnitApsNals();
      m_cDecLib.resetAccessUnitPicInfo();
#if JVET_S0078_NOOUTPUTPRIORPICFLAG
      firstSliceInAU = true;
#endif
    }
  }
  if ( !m_annotatedRegionsSEIFileName.empty() ) { xOutputAnnotatedRegions( pcListPic ); }
  // May need to check again one more time as in case one the bitstream has only one picture, the first check may miss
  // it
  setOutputPicturePresentInStream();
  CHECK( !outputPicturePresentInBitstream,
         "It is required that there shall be at least one picture with PictureOutputFlag equal to 1 in the bitstream" )

  setVideoSize( pcListPic->front()->cs->sps );
  xFlushOutput( pcListPic, video );

  // get the number of checksum errors
  uint32_t nRet = m_cDecLib.getNumberOfChecksumErrorsDetected();

  // delete buffers
  m_cDecLib.deletePicBuffer();
  // destroy internal classes
  xDestroyDecLib();

#if RExt__DECODER_DEBUG_STATISTICS
  CodingStatistics::DestroyInstance();
#endif

  destroyROM();

  return nRet;
}

template <typename T>
void PCCVTMLibVideoDecoderImpl<T>::xCreateDecLib() {
  initROM();

  // create decoder class
  m_cDecLib.create();

  // initialize decoder class
  m_cDecLib.init(
#if JVET_J0090_MEMORY_BANDWITH_MEASURE
      m_cacheCfgFile
#endif
  );
  m_cDecLib.setDecodedPictureHashSEIEnabled( m_decodedPictureHashSEIEnabled );

  if ( !m_outputDecodedSEIMessagesFilename.empty() ) {
    std::ostream& os = m_seiMessageFileStream.is_open() ? m_seiMessageFileStream : std::cout;
    m_cDecLib.setDecodedSEIMessageOutputStream( &os );
  }
#if JVET_S0257_DUMP_360SEI_MESSAGE
  if ( !m_outputDecoded360SEIMessagesFilename.empty() ) {
    m_cDecLib.setDecoded360SEIMessageFileName( m_outputDecoded360SEIMessagesFilename );
  }
#endif
  m_cDecLib.m_targetSubPicIdx = this->m_targetSubPicIdx;
  m_cDecLib.initScalingList();
}

template <typename T>
void PCCVTMLibVideoDecoderImpl<T>::xDestroyDecLib() {
  //  if( !m_reconFileName.empty() )
  //  {
  //    for( auto & recFile : m_cVideoIOYuvReconFile )
  //    {
  //      recFile.second.close();
  //    }
  //  }

  // destroy decoder class
  m_cDecLib.destroy();
}

template <typename T>
void PCCVTMLibVideoDecoderImpl<T>::setVideoSize( const SPS* sps ) {
  auto& window        = sps->getConformanceWindow();
  int   width         = sps->getMaxPicWidthInLumaSamples();
  int   height        = sps->getMaxPicHeightInLumaSamples();
  m_outputWidth       = width - window.getWindowLeftOffset() - window.getWindowRightOffset();
  m_outputHeight      = height - window.getWindowTopOffset() - window.getWindowBottomOffset();
  m_internalBitDepths = sps->getBitDepths().recon[CHANNEL_TYPE_LUMA];
  m_bRGB2GBR          = sps->getChromaFormatIdc() == CHROMA_444;
}

template <typename T>
void PCCVTMLibVideoDecoderImpl<T>::xWriteOutput( PicList* pcListPic, uint32_t tId, PCCVideo<T, 3>& video ) {
  if ( pcListPic->empty() ) { return; }

  PicList::iterator iterPic                = pcListPic->begin();
  int               numPicsNotYetDisplayed = 0;
  int               dpbFullness            = 0;
  const SPS*        activeSPS              = ( pcListPic->front()->cs->sps );
  uint32_t          maxNumReorderPicsHighestTid;
  uint32_t          maxDecPicBufferingHighestTid;
  uint32_t          maxNrSublayers = activeSPS->getMaxTLayers();

  const VPS* referredVPS = pcListPic->front()->cs->vps;
  const int  temporalId =
      ( m_iMaxTemporalLayer == -1 || m_iMaxTemporalLayer >= maxNrSublayers ) ? maxNrSublayers - 1 : m_iMaxTemporalLayer;

  if ( referredVPS == nullptr || referredVPS->m_numLayersInOls[referredVPS->m_targetOlsIdx] == 1 ) {
    maxNumReorderPicsHighestTid  = activeSPS->getMaxNumReorderPics( temporalId );
    maxDecPicBufferingHighestTid = activeSPS->getMaxDecPicBuffering( temporalId );
  } else {
    maxNumReorderPicsHighestTid  = referredVPS->getMaxNumReorderPics( temporalId );
    maxDecPicBufferingHighestTid = referredVPS->getMaxDecPicBuffering( temporalId );
  }

  while ( iterPic != pcListPic->end() ) {
    Picture* pcPic = *( iterPic );
    if ( pcPic->neededForOutput && pcPic->getPOC() > m_iPOCLastDisplay ) {
      numPicsNotYetDisplayed++;
      dpbFullness++;
    } else if ( pcPic->referenced ) {
      dpbFullness++;
    }
    iterPic++;
  }

  iterPic = pcListPic->begin();

  if ( numPicsNotYetDisplayed > 2 ) { iterPic++; }

  Picture* pcPic = *( iterPic );
  if ( numPicsNotYetDisplayed > 2 && pcPic->fieldPic )  // Field Decoding
  {
    PicList::iterator endPic = pcListPic->end();
    endPic--;
    iterPic = pcListPic->begin();
    while ( iterPic != endPic ) {
      Picture* pcPicTop = *( iterPic );
      iterPic++;
      Picture* pcPicBottom = *( iterPic );

      if ( pcPicTop->neededForOutput && pcPicBottom->neededForOutput &&
           ( numPicsNotYetDisplayed > maxNumReorderPicsHighestTid || dpbFullness > maxDecPicBufferingHighestTid ) &&
           ( !( pcPicTop->getPOC() % 2 ) && pcPicBottom->getPOC() == pcPicTop->getPOC() + 1 ) &&
           ( pcPicTop->getPOC() == m_iPOCLastDisplay + 1 || m_iPOCLastDisplay < 0 ) ) {
        // write to file
        numPicsNotYetDisplayed = numPicsNotYetDisplayed - 2;
        xWritePicture( pcPic, video );

        // update POC of display order
        m_iPOCLastDisplay = pcPicBottom->getPOC();

        // erase non-referenced picture in the reference picture list after display
        if ( !pcPicTop->referenced && pcPicTop->reconstructed ) { pcPicTop->reconstructed = false; }
        if ( !pcPicBottom->referenced && pcPicBottom->reconstructed ) { pcPicBottom->reconstructed = false; }
        pcPicTop->neededForOutput    = false;
        pcPicBottom->neededForOutput = false;
      }
    }
  } else if ( !pcPic->fieldPic )  // Frame Decoding
  {
    iterPic = pcListPic->begin();

    while ( iterPic != pcListPic->end() ) {
      pcPic = *( iterPic );

      if ( pcPic->neededForOutput && pcPic->getPOC() > m_iPOCLastDisplay &&
           ( numPicsNotYetDisplayed > maxNumReorderPicsHighestTid || dpbFullness > maxDecPicBufferingHighestTid ) ) {
        // write to file
        numPicsNotYetDisplayed--;
        if ( !pcPic->referenced ) { dpbFullness--; }
        xWritePicture( pcPic, video );

        // update POC of display order
        m_iPOCLastDisplay = pcPic->getPOC();

        // erase non-referenced picture in the reference picture list after display
        if ( !pcPic->referenced && pcPic->reconstructed ) { pcPic->reconstructed = false; }
        pcPic->neededForOutput = false;
      }

      iterPic++;
    }
  }
}

#if JVET_S0078_NOOUTPUTPRIORPICFLAG
template <typename T>
void PCCVTMLibVideoDecoderImpl<T>::xFlushOutput( PicList*        pcListPic,
                                                 PCCVideo<T, 3>& video,
                                                 const int       layerId,
                                                 bool            noOutputOfPriorPicsFlag ) {
#else
template <typename T>
void PCCVTMLibVideoDecoderImpl<T>::xFlushOutput( PicList* pcListPic, PCCVideo<T, 3>& video, const int layerId ) {
#endif
  if ( !pcListPic || pcListPic->empty() ) { return; }
  PicList::iterator iterPic = pcListPic->begin();

  iterPic        = pcListPic->begin();
  Picture* pcPic = *( iterPic );

  if ( pcPic->fieldPic )  // Field Decoding
  {
    PicList::iterator endPic = pcListPic->end();
    endPic--;
    Picture *pcPicTop, *pcPicBottom = NULL;
    while ( iterPic != endPic ) {
      pcPicTop = *( iterPic );
      iterPic++;
      pcPicBottom = *( iterPic );

      if ( pcPicTop->layerId != layerId && layerId != NOT_VALID ) { continue; }

      if ( pcPicTop->neededForOutput && pcPicBottom->neededForOutput && !( pcPicTop->getPOC() % 2 ) &&
           ( pcPicBottom->getPOC() == pcPicTop->getPOC() + 1 ) ) {
#if JVET_S0078_NOOUTPUTPRIORPICFLAG
        if ( !noOutputOfPriorPicsFlag ) {
#endif
          // write to file
          xWritePicture( pcPic, video );
#if JVET_S0078_NOOUTPUTPRIORPICFLAG
        }
#endif
        // update POC of display order
        m_iPOCLastDisplay = pcPicBottom->getPOC();

        // erase non-referenced picture in the reference picture list after display
        if ( !pcPicTop->referenced && pcPicTop->reconstructed ) { pcPicTop->reconstructed = false; }
        if ( !pcPicBottom->referenced && pcPicBottom->reconstructed ) { pcPicBottom->reconstructed = false; }
        pcPicTop->neededForOutput    = false;
        pcPicBottom->neededForOutput = false;
        if ( pcPicTop ) {
          pcPicTop->destroy();
          delete pcPicTop;
          pcPicTop = NULL;
        }
      }
    }
    if ( pcPicBottom ) {
      pcPicBottom->destroy();
      delete pcPicBottom;
      pcPicBottom = NULL;
    }
  } else  // Frame decoding
  {
    while ( iterPic != pcListPic->end() ) {
      pcPic = *( iterPic );

      if ( pcPic->layerId != layerId && layerId != NOT_VALID ) {
        iterPic++;
        continue;
      }

      if ( pcPic->neededForOutput ) {
#if JVET_S0078_NOOUTPUTPRIORPICFLAG
        if ( !noOutputOfPriorPicsFlag ) {
#endif
          // write to file
          xWritePicture( pcPic, video );

#if JVET_S0078_NOOUTPUTPRIORPICFLAG
        }
#endif
        // update POC of display order
        m_iPOCLastDisplay = pcPic->getPOC();

        // erase non-referenced picture in the reference picture list after display
        if ( !pcPic->referenced && pcPic->reconstructed ) { pcPic->reconstructed = false; }
        pcPic->neededForOutput = false;
      }
      if ( pcPic != NULL ) {
        pcPic->destroy();
        delete pcPic;
        pcPic    = NULL;
        *iterPic = nullptr;
      }
      iterPic++;
    }
  }
  if ( layerId != NOT_VALID ) {
    pcListPic->remove_if( []( Picture* p ) { return p == nullptr; } );
  } else {
    pcListPic->clear();
  }
  m_iPOCLastDisplay = -MAX_INT;
}

template <typename T>
void PCCVTMLibVideoDecoderImpl<T>::xWritePicture( const Picture* pic, PCCVideo<T, 3>& video ) {
  int chromaSubsample = pic->getPicWidthInLumaSamples() / pic->chromaSize().width;
  printf( "write output frame %zu size = %d %d bit depth = %d - %d = %d in %zd bytes buffers ( ChromaSub. = %d )\n",
          video.size(), pic->getPicWidthInLumaSamples(), pic->getPicHeightInLumaSamples(), m_internalBitDepths,
          m_outputBitDepth[0], m_internalBitDepths - m_outputBitDepth[0], sizeof( T ), chromaSubsample );
  fflush( stdout );
  video.resize( video.getFrameCount() + 1 );
  auto&          image = video.getFrames().back();
  PCCCOLORFORMAT format =
      chromaSubsample > 1 ? PCCCOLORFORMAT::YUV420 : m_bRGB2GBR ? PCCCOLORFORMAT::RGB444 : PCCCOLORFORMAT::YUV444;
  image.set( pic->getBuf( COMPONENT_Y, PIC_RECONSTRUCTION ).bufAt( 0, 0 ),
             pic->getBuf( COMPONENT_Cb, PIC_RECONSTRUCTION ).bufAt( 0, 0 ),
             pic->getBuf( COMPONENT_Cr, PIC_RECONSTRUCTION ).bufAt( 0, 0 ), m_outputWidth, m_outputHeight,
             pic->getBuf( COMPONENT_Y, PIC_RECONSTRUCTION ).stride, m_outputWidth / chromaSubsample,
             m_outputHeight / chromaSubsample, pic->getBuf( COMPONENT_Cb, PIC_RECONSTRUCTION ).stride,
             m_internalBitDepths - m_outputBitDepth[0], format, m_bRGB2GBR );
}

template <typename T>
bool PCCVTMLibVideoDecoderImpl<T>::xIsNaluWithinTargetDecLayerIdSet( const InputNALUnit* nalu ) const {
  if ( !m_targetDecLayerIdSet.size() )  // By default, the set is empty, meaning all LayerIds are allowed
  {
    return true;
  }

  return std::find( m_targetDecLayerIdSet.begin(), m_targetDecLayerIdSet.end(), nalu->m_nuhLayerId ) !=
         m_targetDecLayerIdSet.end();
}

/** \param pcListPic list of pictures to be written to file
 */
template <typename T>
void PCCVTMLibVideoDecoderImpl<T>::xOutputAnnotatedRegions( PicList* pcListPic ) {
  if ( !pcListPic || pcListPic->empty() ) { return; }
  PicList::iterator iterPic = pcListPic->begin();

  while ( iterPic != pcListPic->end() ) {
    Picture* pcPic = *( iterPic );
    if ( pcPic->neededForOutput ) {
      // Check if any annotated region SEI has arrived
      SEIMessages annotatedRegionSEIs = getSeisByType( pcPic->SEIs, SEI::ANNOTATED_REGIONS );
      for ( auto it = annotatedRegionSEIs.begin(); it != annotatedRegionSEIs.end(); it++ ) {
        const SEIAnnotatedRegions& seiAnnotatedRegions = *(SEIAnnotatedRegions*)( *it );

        if ( seiAnnotatedRegions.m_hdr.m_cancelFlag ) {
          m_arObjects.clear();
          m_arLabels.clear();
        } else {
          if ( m_arHeader.m_receivedSettingsOnce ) {
            // validate those settings that must stay constant are constant.
            assert( m_arHeader.m_occludedObjectFlag == seiAnnotatedRegions.m_hdr.m_occludedObjectFlag );
            assert( m_arHeader.m_partialObjectFlagPresentFlag ==
                    seiAnnotatedRegions.m_hdr.m_partialObjectFlagPresentFlag );
            assert( m_arHeader.m_objectConfidenceInfoPresentFlag ==
                    seiAnnotatedRegions.m_hdr.m_objectConfidenceInfoPresentFlag );
            assert( ( !m_arHeader.m_objectConfidenceInfoPresentFlag ) ||
                    m_arHeader.m_objectConfidenceLength == seiAnnotatedRegions.m_hdr.m_objectConfidenceLength );
          } else {
            m_arHeader.m_receivedSettingsOnce = true;
            m_arHeader                        = seiAnnotatedRegions.m_hdr;  // copy the settings.
          }
          // Process label updates
          if ( seiAnnotatedRegions.m_hdr.m_objectLabelPresentFlag ) {
            for ( auto srcIt = seiAnnotatedRegions.m_annotatedLabels.begin();
                  srcIt != seiAnnotatedRegions.m_annotatedLabels.end(); srcIt++ ) {
              const uint32_t labIdx = srcIt->first;
              if ( srcIt->second.labelValid ) {
                m_arLabels[labIdx] = srcIt->second.label;
              } else {
                m_arLabels.erase( labIdx );
              }
            }
          }

          // Process object updates
          for ( auto srcIt = seiAnnotatedRegions.m_annotatedRegions.begin();
                srcIt != seiAnnotatedRegions.m_annotatedRegions.end(); srcIt++ ) {
            uint32_t                                          objIdx = srcIt->first;
            const SEIAnnotatedRegions::AnnotatedRegionObject& src    = srcIt->second;

            if ( src.objectCancelFlag ) {
              m_arObjects.erase( objIdx );
            } else {
              auto destIt = m_arObjects.find( objIdx );

              if ( destIt == m_arObjects.end() ) {
                // New object arrived, needs to be appended to the map of tracked objects
                m_arObjects[objIdx] = src;
              } else  // Existing object, modifications to be done
              {
                SEIAnnotatedRegions::AnnotatedRegionObject& dst = destIt->second;

                if ( seiAnnotatedRegions.m_hdr.m_objectLabelPresentFlag && src.objectLabelValid ) {
                  dst.objectLabelValid = true;
                  dst.objLabelIdx      = src.objLabelIdx;
                }
                if ( src.boundingBoxValid ) {
                  dst.boundingBoxTop    = src.boundingBoxTop;
                  dst.boundingBoxLeft   = src.boundingBoxLeft;
                  dst.boundingBoxWidth  = src.boundingBoxWidth;
                  dst.boundingBoxHeight = src.boundingBoxHeight;
                  if ( seiAnnotatedRegions.m_hdr.m_partialObjectFlagPresentFlag ) {
                    dst.partialObjectFlag = src.partialObjectFlag;
                  }
                  if ( seiAnnotatedRegions.m_hdr.m_objectConfidenceInfoPresentFlag ) {
                    dst.objectConfidence = src.objectConfidence;
                  }
                }
              }
            }
          }
        }
      }

      if ( !m_arObjects.empty() ) {
        FILE* fpPersist = fopen( m_annotatedRegionsSEIFileName.c_str(), "ab" );
        if ( fpPersist == NULL ) {
          std::cout << "Not able to open file for writing persist SEI messages" << std::endl;
        } else {
          fprintf( fpPersist, "\n" );
          fprintf( fpPersist, "Number of objects = %d\n", (int)m_arObjects.size() );
          for ( auto it = m_arObjects.begin(); it != m_arObjects.end(); ++it ) {
            fprintf( fpPersist, "Object Idx = %d\n", it->first );
            fprintf( fpPersist, "Object Top = %d\n", it->second.boundingBoxTop );
            fprintf( fpPersist, "Object Left = %d\n", it->second.boundingBoxLeft );
            fprintf( fpPersist, "Object Width = %d\n", it->second.boundingBoxWidth );
            fprintf( fpPersist, "Object Height = %d\n", it->second.boundingBoxHeight );
            if ( it->second.objectLabelValid ) {
              auto labelIt = m_arLabels.find( it->second.objLabelIdx );
              fprintf( fpPersist, "Object Label = %s\n",
                       labelIt != m_arLabels.end() ? ( labelIt->second.c_str() ) : "<UNKNOWN>" );
            }
            if ( m_arHeader.m_partialObjectFlagPresentFlag ) {
              fprintf( fpPersist, "Object Partial = %d\n", it->second.partialObjectFlag ? 1 : 0 );
            }
            if ( m_arHeader.m_objectConfidenceInfoPresentFlag ) {
              fprintf( fpPersist, "Object Conf = %d\n", it->second.objectConfidence );
            }
          }
          fclose( fpPersist );
        }
      }
    }
    iterPic++;
  }
}

template <typename T>
bool PCCVTMLibVideoDecoderImpl<T>::xIsNaluWithinTargetOutputLayerIdSet( const InputNALUnit* nalu ) const {
  if ( !m_targetOutputLayerIdSet.size() )  // By default, the set is empty, meaning all LayerIds are allowed
  {
    return true;
  }

  return std::find( m_targetOutputLayerIdSet.begin(), m_targetOutputLayerIdSet.end(), nalu->m_nuhLayerId ) !=
         m_targetOutputLayerIdSet.end();
}

template <typename T>
bool PCCVTMLibVideoDecoderImpl<T>::isNewPicture( std::istream* bitstreamFile, class InputByteStream* bytestream ) {
  bool ret      = false;
  bool finished = false;

  // cannot be a new picture if there haven't been any slices yet
  if ( m_cDecLib.getFirstSliceInPicture() ) { return false; }

  // save stream position for backup
#if RExt__DECODER_DEBUG_STATISTICS
  CodingStatistics::CodingStatisticsData* backupStats =
      new CodingStatistics::CodingStatisticsData( CodingStatistics::GetStatistics() );
  std::streampos location = bitstreamFile->tellg() - std::streampos( bytestream->GetNumBufferedBytes() );
#else
  std::streampos location = bitstreamFile->tellg();
#endif

  // look ahead until picture start location is determined
  while ( !finished && !!( *bitstreamFile ) ) {
    AnnexBStats  stats = AnnexBStats();
    InputNALUnit nalu;
    byteStreamNALUnit( *bytestream, nalu.getBitstream().getFifo(), stats );
    if ( nalu.getBitstream().getFifo().empty() ) {
      msg( VTM_ERROR, "Warning: Attempt to decode an empty NAL unit\n" );
    } else {
      // get next NAL unit type
      read( nalu );
      switch ( nalu.m_nalUnitType ) {
        // NUT that indicate the start of a new picture
        case NAL_UNIT_ACCESS_UNIT_DELIMITER:
        case NAL_UNIT_OPI:
        case NAL_UNIT_DCI:
        case NAL_UNIT_VPS:
        case NAL_UNIT_SPS:
        case NAL_UNIT_PPS:
        case NAL_UNIT_PH:
          ret      = true;
          finished = true;
          break;

          // NUT that may be the start of a new picture - check first bit in slice header
        case NAL_UNIT_CODED_SLICE_TRAIL:
        case NAL_UNIT_CODED_SLICE_STSA:
        case NAL_UNIT_CODED_SLICE_RASL:
        case NAL_UNIT_CODED_SLICE_RADL:
        case NAL_UNIT_RESERVED_VCL_4:
        case NAL_UNIT_RESERVED_VCL_5:
        case NAL_UNIT_RESERVED_VCL_6:
        case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
        case NAL_UNIT_CODED_SLICE_IDR_N_LP:
        case NAL_UNIT_CODED_SLICE_CRA:
        case NAL_UNIT_CODED_SLICE_GDR:
        case NAL_UNIT_RESERVED_IRAP_VCL_11:
          ret      = checkPictureHeaderInSliceHeaderFlag( nalu );
          finished = true;
          break;

        // NUT that are not the start of a new picture
        case NAL_UNIT_EOS:
        case NAL_UNIT_EOB:
        case NAL_UNIT_SUFFIX_APS:
        case NAL_UNIT_SUFFIX_SEI:
        case NAL_UNIT_FD:
          ret      = false;
          finished = true;
          break;

        // NUT that might indicate the start of a new picture - keep looking
        case NAL_UNIT_PREFIX_APS:
        case NAL_UNIT_PREFIX_SEI:
        case NAL_UNIT_RESERVED_NVCL_26:
        case NAL_UNIT_RESERVED_NVCL_27:
        case NAL_UNIT_UNSPECIFIED_28:
        case NAL_UNIT_UNSPECIFIED_29:
        case NAL_UNIT_UNSPECIFIED_30:
        case NAL_UNIT_UNSPECIFIED_31:
        default: break;
      }
    }
  }

  // restore previous stream location - minus 3 due to the need for the annexB parser to read three extra bytes
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  bitstreamFile->clear();
  bitstreamFile->seekg( location );
  bytestream->reset();
  CodingStatistics::SetStatistics( *backupStats );
  delete backupStats;
#else
  bitstreamFile->clear();
  bitstreamFile->seekg( location - std::streamoff( 3 ) );
  bytestream->reset();
#endif

  // return TRUE if next NAL unit is the start of a new picture
  return ret;
}

template <typename T>
bool PCCVTMLibVideoDecoderImpl<T>::isNewAccessUnit( bool                   newPicture,
                                                    std::istream*          bitstreamFile,
                                                    class InputByteStream* bytestream ) {
  bool ret      = false;
  bool finished = false;

  // can only be the start of an AU if this is the start of a new picture
  if ( newPicture == false ) { return false; }

  // save stream position for backup
#if RExt__DECODER_DEBUG_STATISTICS
  CodingStatistics::CodingStatisticsData* backupStats =
      new CodingStatistics::CodingStatisticsData( CodingStatistics::GetStatistics() );
  std::streampos location = bitstreamFile->tellg() - std::streampos( bytestream->GetNumBufferedBytes() );
#else
  std::streampos location = bitstreamFile->tellg();
#endif

  // look ahead until access unit start location is determined
  while ( !finished && !!( *bitstreamFile ) ) {
    AnnexBStats  stats = AnnexBStats();
    InputNALUnit nalu;
    byteStreamNALUnit( *bytestream, nalu.getBitstream().getFifo(), stats );
    if ( nalu.getBitstream().getFifo().empty() ) {
      msg( VTM_ERROR, "Warning: Attempt to decode an empty NAL unit\n" );
    } else {
      // get next NAL unit type
      read( nalu );
      switch ( nalu.m_nalUnitType ) {
        // AUD always indicates the start of a new access unit
        case NAL_UNIT_ACCESS_UNIT_DELIMITER:
          ret      = true;
          finished = true;
          break;

        // slice types - check layer ID and POC
        case NAL_UNIT_CODED_SLICE_TRAIL:
        case NAL_UNIT_CODED_SLICE_STSA:
        case NAL_UNIT_CODED_SLICE_RASL:
        case NAL_UNIT_CODED_SLICE_RADL:
        case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
        case NAL_UNIT_CODED_SLICE_IDR_N_LP:
        case NAL_UNIT_CODED_SLICE_CRA:
        case NAL_UNIT_CODED_SLICE_GDR:
          ret      = m_cDecLib.isSliceNaluFirstInAU( newPicture, nalu );
          finished = true;
          break;

        // NUT that are not the start of a new access unit
        case NAL_UNIT_EOS:
        case NAL_UNIT_EOB:
        case NAL_UNIT_SUFFIX_APS:
        case NAL_UNIT_SUFFIX_SEI:
        case NAL_UNIT_FD:
          ret      = false;
          finished = true;
          break;

        // all other NUT - keep looking to find first VCL
        default: break;
      }
    }
  }

  // restore previous stream location
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  bitstreamFile->clear();
  bitstreamFile->seekg( location );
  bytestream->reset();
  CodingStatistics::SetStatistics( *backupStats );
  delete backupStats;
#else
  bitstreamFile->clear();
  bitstreamFile->seekg( location );
  bytestream->reset();
#endif

  // return TRUE if next NAL unit is the start of a new picture
  return ret;
}

template class pcc::PCCVTMLibVideoDecoderImpl<uint8_t>;
template class pcc::PCCVTMLibVideoDecoderImpl<uint16_t>;

#endif
