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
#include "PCCCommon.h"

#ifdef USE_HM_VIDEO_CODEC

#include "PCCHMLibVideoDecoderImpl.h"

using namespace pcc;

template <typename T>
PCCHMLibVideoDecoderImpl<T>::PCCHMLibVideoDecoderImpl() {}

template <typename T>
PCCHMLibVideoDecoderImpl<T>::~PCCHMLibVideoDecoderImpl() {}

template <typename T>
void PCCHMLibVideoDecoderImpl<T>::decode( PCCVideoBitstream& bitstream,
                                          size_t             outputBitDepth,
                                          bool               RGB2GBR,
                                          PCCVideo<T, 3>&    video ) {
  std::string         s( reinterpret_cast<char*>( bitstream.buffer() ), bitstream.size() );
  std::istringstream  iss( s );
  std::istream&       bitstreamFile = iss;
  Int                 poc;
  TComList<TComPic*>* pcListPic = NULL;
  m_bRGB2GBR                    = RGB2GBR;
  InputByteStream bytestream( bitstreamFile );
  if ( outputBitDepth ) {
    m_outputBitDepth[CHANNEL_TYPE_LUMA]   = outputBitDepth;
    m_outputBitDepth[CHANNEL_TYPE_CHROMA] = outputBitDepth;
  }
  video.clear();
  // create & initialize internal classes
  m_cTDecTop.create();
  m_cTDecTop.init();
  m_cTDecTop.setDecodedPictureHashSEIEnabled( 1 );
  m_iPOCLastDisplay += m_iSkipFrame;  // set the last displayed POC correctly for skip forward.
  // main decoder loop
  Bool openedReconFile = false;  // reconstruction file not yet opened. (must be performed after SPS is seen)
  Bool loopFiltered    = false;
  while ( !!bitstreamFile ) {
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    TComCodingStatistics::TComCodingStatisticsData backupStats( TComCodingStatistics::GetStatistics() );
    streampos location = bitstreamFile.tellg() - streampos( bytestream.GetNumBufferedBytes() );
#else
    streampos location = bitstreamFile.tellg();
#endif
    AnnexBStats  stats = AnnexBStats();
    InputNALUnit nalu;
    byteStreamNALUnit( bytestream, nalu.getBitstream().getFifo(), stats );
    // call actual decoding function
    Bool bNewPicture = false;
    if ( nalu.getBitstream().getFifo().empty() ) {
      fprintf( stderr, "Warning: Attempt to decode an empty NAL unit\n" );
    } else {
      read( nalu );
      bNewPicture = m_cTDecTop.decode( nalu, m_iSkipFrame, m_iPOCLastDisplay );
      if ( bNewPicture ) {
        bitstreamFile.clear();
#if RExt__DECODER_DEBUG_BIT_STATISTICS
        bitstreamFile.seekg( location );
        bytestream.reset();
        TComCodingStatistics::SetStatistics( backupStats );
#else
        bitstreamFile.seekg( location - streamoff( 3 ) );
        bytestream.reset();
#endif
      }
    }

    if ( ( bNewPicture || !bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS ) &&
         !m_cTDecTop.getFirstSliceInSequence() ) {
      if ( !loopFiltered || bitstreamFile ) { m_cTDecTop.executeLoopFilters( poc, pcListPic ); }
      loopFiltered = ( nalu.m_nalUnitType == NAL_UNIT_EOS );
      if ( nalu.m_nalUnitType == NAL_UNIT_EOS ) { m_cTDecTop.setFirstSliceInSequence( true ); }
    } else if ( ( bNewPicture || !bitstreamFile || nalu.m_nalUnitType == NAL_UNIT_EOS ) &&
                m_cTDecTop.getFirstSliceInSequence() ) {
      m_cTDecTop.setFirstSliceInPicture( true );
    }

    if ( pcListPic ) {
      if ( bNewPicture ) {
        if ( m_cTDecTop.getTwoVersionsOfCurrDecPicFlag() ) {
          // remove current picture before ILF
          m_cTDecTop.remCurPicBefILFFromDPBDecDPBFullnessByOne( pcListPic );
          m_cTDecTop.updateCurrentPictureFlag( pcListPic );
        } else if ( m_cTDecTop.isCurrPicAsRef() ) {
          m_cTDecTop.markCurrentPictureAfterILFforShortTermRef( pcListPic );
        }
      }
      // write reconstruction to file
      if ( bNewPicture ) {
        setVideoSize( pcListPic->front()->getPicSym()->getSPS() );
        xWriteOutput( pcListPic, nalu.m_temporalId, video );
      }
      if ( ( bNewPicture || nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_CRA ) &&
           m_cTDecTop.getNoOutputPriorPicsFlag() ) {
        m_cTDecTop.checkNoOutputPriorPics( pcListPic );
        m_cTDecTop.setNoOutputPriorPicsFlag( false );
      }
      if ( bNewPicture && ( nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
                            nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_IDR_N_LP ||
                            nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_N_LP ||
                            nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_RADL ||
                            nalu.m_nalUnitType == NAL_UNIT_CODED_SLICE_BLA_W_LP ) ) {
        setVideoSize( pcListPic->front()->getPicSym()->getSPS() );
        xFlushOutput( pcListPic, video );
      }
      if ( nalu.m_nalUnitType == NAL_UNIT_EOS ) {
        setVideoSize( pcListPic->front()->getPicSym()->getSPS() );
        xWriteOutput( pcListPic, nalu.m_temporalId, video );
        m_cTDecTop.setFirstSliceInPicture( false );
      }
      // write reconstruction to file -- for additional bumping as defined in C.5.2.3
      if ( !bNewPicture && nalu.m_nalUnitType >= NAL_UNIT_CODED_SLICE_TRAIL_N &&
           nalu.m_nalUnitType <= NAL_UNIT_RESERVED_VCL31 ) {
        setVideoSize( pcListPic->front()->getPicSym()->getSPS() );
        xWriteOutput( pcListPic, nalu.m_temporalId, video );
      }
    }
  }

  xFlushOutput( pcListPic, video );
  // delete buffers
  m_cTDecTop.deletePicBuffer();

  // destroy internal classes
  m_cTDecTop.destroy();
}

template <typename T>
void PCCHMLibVideoDecoderImpl<T>::setVideoSize( const TComSPS& sps ) {
  auto& window        = sps.getConformanceWindow();
  int   width         = sps.getPicWidthInLumaSamples();
  int   height        = sps.getPicHeightInLumaSamples();
  m_outputWidth       = width - window.getWindowLeftOffset() - window.getWindowRightOffset();
  m_outputHeight      = height - window.getWindowTopOffset() - window.getWindowBottomOffset();
  m_internalBitDepths = sps.getBitDepths().recon[CHANNEL_TYPE_LUMA];
}

template <typename T>
void PCCHMLibVideoDecoderImpl<T>::xWriteOutput( TComList<TComPic*>* pcListPic, UInt tId, PCCVideo<T, 3>& video ) {
  if ( pcListPic->empty() ) { return; }
  TComList<TComPic*>::iterator iterPic                = pcListPic->begin();
  Int                          numPicsNotYetDisplayed = 0;
  Int                          dpbFullness            = 0;
  const TComSPS*               activeSPS              = &( pcListPic->front()->getPicSym()->getSPS() );
  UInt                         numReorderPicsHighestTid;
  UInt                         maxDecPicBufferingHighestTid;
  UInt                         maxNrSublayers = activeSPS->getMaxTLayers();
  numReorderPicsHighestTid                    = activeSPS->getNumReorderPics( maxNrSublayers - 1 );
  maxDecPicBufferingHighestTid                = activeSPS->getMaxDecPicBuffering( maxNrSublayers - 1 );
  while ( iterPic != pcListPic->end() ) {
    TComPic* pcPic = *( iterPic );
    if ( pcPic->getOutputMark() && pcPic->getPOC() > m_iPOCLastDisplay ) {
      numPicsNotYetDisplayed++;
      dpbFullness++;
    } else if ( pcPic->getSlice( 0 )->isReferenced() ) {
      dpbFullness++;
    }
    iterPic++;
  }
  iterPic = pcListPic->begin();
  if ( numPicsNotYetDisplayed > 2 ) { iterPic++; }
  TComPic* pcPic = *( iterPic );
  if ( numPicsNotYetDisplayed > 2 && pcPic->isField() ) {  // Field Decoding
    TComList<TComPic*>::iterator endPic = pcListPic->end();
    endPic--;
    iterPic = pcListPic->begin();
    while ( iterPic != endPic ) {
      TComPic* pcPicTop = *( iterPic );
      iterPic++;
      TComPic* pcPicBottom = *( iterPic );
      if ( pcPicTop->getOutputMark() && pcPicBottom->getOutputMark() &&
           ( numPicsNotYetDisplayed > numReorderPicsHighestTid ||
             dpbFullness > maxDecPicBufferingHighestTid - m_cTDecTop.getTwoVersionsOfCurrDecPicFlag() ) &&
           ( !( pcPicTop->getPOC() % 2 ) && pcPicBottom->getPOC() == pcPicTop->getPOC() + 1 ) &&
           ( pcPicTop->getPOC() == m_iPOCLastDisplay + 1 || m_iPOCLastDisplay < 0 ) ) {
        // write to file
        numPicsNotYetDisplayed = numPicsNotYetDisplayed - 2;
        xWritePicture( pcPicTop->getPicYuvRec(), video );
        // update POC of display order
        m_iPOCLastDisplay = pcPicBottom->getPOC();
        // erase non-referenced picture in the reference picture list after display
        if ( !pcPicTop->getSlice( 0 )->isReferenced() && pcPicTop->getReconMark() == true ) {
          pcPicTop->setReconMark( false );
          // mark it should be extended later
          pcPicTop->getPicYuvRec()->setBorderExtension( false );
        }
        if ( !pcPicBottom->getSlice( 0 )->isReferenced() && pcPicBottom->getReconMark() == true ) {
          pcPicBottom->setReconMark( false );
          // mark it should be extended later
          pcPicBottom->getPicYuvRec()->setBorderExtension( false );
        }
        pcPicTop->setOutputMark( false );
        pcPicBottom->setOutputMark( false );
      }
    }
  } else if ( !pcPic->isField() ) {  // Frame Decoding
    iterPic = pcListPic->begin();
    while ( iterPic != pcListPic->end() ) {
      pcPic = *( iterPic );

      if ( pcPic->getOutputMark() && pcPic->getPOC() > m_iPOCLastDisplay &&
           ( numPicsNotYetDisplayed > numReorderPicsHighestTid ||
             dpbFullness > maxDecPicBufferingHighestTid - m_cTDecTop.getTwoVersionsOfCurrDecPicFlag() ) ) {
        // write to file
        numPicsNotYetDisplayed--;
        if ( pcPic->getSlice( 0 )->isReferenced() == false ) { dpbFullness--; }
        xWritePicture( pcPic->getPicYuvRec(), video );
        // update POC of display order
        m_iPOCLastDisplay = pcPic->getPOC();
        // erase non-referenced picture in the reference picture list after display
        if ( !pcPic->getSlice( 0 )->isReferenced() && pcPic->getReconMark() == true ) {
          pcPic->setReconMark( false );
          // mark it should be extended later
          pcPic->getPicYuvRec()->setBorderExtension( false );
        }
        pcPic->setOutputMark( false );
      }
      iterPic++;
    }
  }
}

template <typename T>
void PCCHMLibVideoDecoderImpl<T>::xFlushOutput( TComList<TComPic*>* pcListPic, PCCVideo<T, 3>& video ) {
  if ( !pcListPic || pcListPic->empty() ) { return; }
  TComList<TComPic*>::iterator iterPic = pcListPic->begin();
  iterPic                              = pcListPic->begin();
  TComPic* pcPic                       = *( iterPic );
  if ( pcPic->isField() ) {  // Field Decoding
    TComList<TComPic*>::iterator endPic = pcListPic->end();
    endPic--;
    TComPic *pcPicTop, *pcPicBottom = NULL;
    while ( iterPic != endPic ) {
      pcPicTop = *( iterPic );
      iterPic++;
      pcPicBottom = *( iterPic );
      if ( pcPicTop->getOutputMark() && pcPicBottom->getOutputMark() && !( pcPicTop->getPOC() % 2 ) &&
           ( pcPicBottom->getPOC() == pcPicTop->getPOC() + 1 ) ) {
        // write to file
        xWritePicture( pcPic->getPicYuvRec(), video );
        // update POC of display order
        m_iPOCLastDisplay = pcPicBottom->getPOC();
        // erase non-referenced picture in the reference picture list after display
        if ( !pcPicTop->getSlice( 0 )->isReferenced() && pcPicTop->getReconMark() == true ) {
          pcPicTop->setReconMark( false );
          // mark it should be extended later
          pcPicTop->getPicYuvRec()->setBorderExtension( false );
        }
        if ( !pcPicBottom->getSlice( 0 )->isReferenced() && pcPicBottom->getReconMark() == true ) {
          pcPicBottom->setReconMark( false );
          // mark it should be extended later
          pcPicBottom->getPicYuvRec()->setBorderExtension( false );
        }
        pcPicTop->setOutputMark( false );
        pcPicBottom->setOutputMark( false );

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
  } else {  // Frame decoding
    while ( iterPic != pcListPic->end() ) {
      pcPic = *( iterPic );
      if ( pcPic->getOutputMark() ) {
        // write to file
        xWritePicture( pcPic->getPicYuvRec(), video );
        // update POC of display order
        m_iPOCLastDisplay = pcPic->getPOC();
        // erase non-referenced picture in the reference picture list after display
        if ( !pcPic->getSlice( 0 )->isReferenced() && pcPic->getReconMark() == true ) {
          pcPic->setReconMark( false );
          // mark it should be extended later
          pcPic->getPicYuvRec()->setBorderExtension( false );
        }
        pcPic->setOutputMark( false );
      }
      if ( pcPic != NULL ) {
        pcPic->destroy();
        delete pcPic;
        pcPic = NULL;
      }
      iterPic++;
    }
  }
  pcListPic->clear();
  m_iPOCLastDisplay = -MAX_INT;
}

template <typename T>
void PCCHMLibVideoDecoderImpl<T>::xWritePicture( const TComPicYuv* pic, PCCVideo<T, 3>& video ) {
  int chromaSubsample = pic->getWidth( COMPONENT_Y ) / pic->getWidth( COMPONENT_Cb );
  printf( "write output frame %zu size = %d %d bit depth = %d - %d = %d in %zd bytes buffers ( ChromaSub. = %d )\n",
          video.size(), pic->getWidth( COMPONENT_Y ), pic->getHeight( COMPONENT_Y ), m_internalBitDepths,
          m_outputBitDepth[0], m_internalBitDepths - m_outputBitDepth[0], sizeof( T ), chromaSubsample );
  fflush( stdout );
  video.resize( video.getFrameCount() + 1 );
  auto&          image = video.getFrames().back();
  PCCCOLORFORMAT format =
      chromaSubsample > 1 ? PCCCOLORFORMAT::YUV420 : m_bRGB2GBR ? PCCCOLORFORMAT::RGB444 : PCCCOLORFORMAT::YUV444;
  image.set( pic->getAddr( COMPONENT_Y ), pic->getAddr( COMPONENT_Cb ), pic->getAddr( COMPONENT_Cr ), m_outputWidth,
             m_outputHeight, pic->getStride( COMPONENT_Y ), m_outputWidth / chromaSubsample,
             m_outputHeight / chromaSubsample, pic->getStride( COMPONENT_Cb ),
             m_internalBitDepths - m_outputBitDepth[0], format, m_bRGB2GBR );
}

template class PCCHMLibVideoDecoderImpl<uint8_t>;
template class PCCHMLibVideoDecoderImpl<uint16_t>;

#endif
