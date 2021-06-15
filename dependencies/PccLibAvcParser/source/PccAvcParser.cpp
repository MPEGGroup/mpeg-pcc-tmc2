#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <fstream>

#include <list>
#include <algorithm>
#include <vector>
using namespace std;

#include "PccAvcParser.h"
#include "PccAvcTDecCAVLC_avc.h"

PccAvcParser::PccAvcParser() {
  frames_.clear();
  vps_.clear();
  sps_.clear();
  pps_.clear();
}

PccAvcParser::~PccAvcParser() {
  frames_.clear();
  vps_.clear();
  sps_.clear();
  pps_.clear();
}

const char* PccAvcParser::getNaluType( int iNaluType ) {
  switch ( iNaluType ) {
    case NALU_TYPE_SLICE: return "SLICE  "; break;
    case NALU_TYPE_DPA: return "DPA  "; break;
    case NALU_TYPE_DPB: return "DPV  "; break;
    case NALU_TYPE_DPC: return "DPC  "; break;
    case NALU_TYPE_IDR: return "IDR  "; break;
    case NALU_TYPE_SEI: return "SEI  "; break;
    case NALU_TYPE_SPS: return "SPS  "; break;
    case NALU_TYPE_PPS: return "PPS  "; break;
    case NALU_TYPE_AUD: return "AUD  "; break;
    case NALU_TYPE_EOSEQ: return "EOSEQ  "; break;
    case NALU_TYPE_EOSTREAM: return "EOSTREAM  "; break;
    case NALU_TYPE_FILL: return "EOSTREAM  "; break;
    case NALU_TYPE_PREFIX: return "PREFIX  "; break;
    case NALU_TYPE_SUB_SPS: return "SUB_SPS  "; break;
    case NALU_TYPE_SLC_EXT: return "SLC_EXT  "; break;
    case NALU_TYPE_VDRD: return "VDRD  "; break;
    case NAL_UNIT_INVALID: return "INVALID  "; break;
    default: return "ERROR    "; break;
  }
  return "ERROR";
}

void PccAvcParser::getVideoSize( const std::vector<uint8_t>& buffer,
                                 size_t&                     width,
                                 size_t&                     height,
                                 int                         isAnnexB,
                                 size_t&                     bitDepth,
                                 bool&                       is444 ) {
  setBuffer( buffer, width, height, isAnnexB, bitDepth, is444 );
}
void PccAvcParser::display() {
  int    poc = 0;
  size_t sum = 0;
  for ( auto& nalu : vps_ ) {
    printf( "%4d / %4zu : Size = %8zu type = %4zu %9s VPS \n", poc, frames_.size(), nalu.size(), nalu.getNaluType(),
            getNaluType( nalu.getNaluType() ) );
    sum += nalu.size();
  }
  for ( auto& nalu : sps_ ) {
    printf( "%4d / %4zu : Size = %8zu type = %4zu %9s SPS \n", poc, frames_.size(), nalu.size(), nalu.getNaluType(),
            getNaluType( nalu.getNaluType() ) );
    sum += nalu.size();
  }
  for ( auto& nalu : pps_ ) {
    printf( "%4d / %4zu : Size = %8zu type = %4zu %9s PPS \n", poc, frames_.size(), nalu.size(), nalu.getNaluType(),
            getNaluType( nalu.getNaluType() ) );
    sum += nalu.size();
  }
  for ( auto& frame : frames_ ) {
    for ( auto& nalu : frame.getNalu() ) {
      printf( "%4d / %4zu : Size = %8zu type = %4zu %9s \n", poc, frames_.size(), nalu.size(), nalu.getNaluType(),
              getNaluType( nalu.getNaluType() ) );
      sum += nalu.size();
    }
    poc++;
  }
  // printf( " sum = %lu \n", sum );
}

void PccAvcParser::createNalu( const size_t                frameIndex,
                               const std::vector<uint8_t>& buffer,
                               const size_t                pos,
                               const size_t                size ) {
  PccAvcNalu nalu;
  nalu.add( buffer, pos, size );
  switch ( nalu.getNaluType() ) {
    // case NAL_TYPE_VPS : vps_.push_back( nalu ); break;
    case NALU_TYPE_SPS:
      sps_.push_back( nalu );
      break;
      // case NAL_TYPE_PPS : pps_.push_back( nalu ); break;
    default:
      if ( frames_.size() <= frameIndex ) { frames_.resize( frameIndex + 1 ); }
      frames_[frameIndex].add( nalu );
      break;
  }
}

void PccAvcParser::setBuffer( const std::vector<uint8_t>& buffer,
                              size_t&                     width,
                              size_t&                     height,
                              int                         isAnnexB,
                              size_t&                     bitDepth,
                              bool&                       is444 ) {
  const int      size                   = (int)buffer.size();
  const uint8_t* data                   = buffer.data();
  TDecCavlc_avc* decCavlc               = new TDecCavlc_avc();
  int            nalNumber              = 0;
  int            index                  = 0;
  int            startCodeSize          = 4;
  int            frameIndex             = -1;
  int            maxPocFound            = -1;
  int            sequencePoc            = 0;
  int            previousNaluLayerIndex = -1;
  int            currentPoc             = 0;
  int            satrtPos               = 0;

  if ( !isAnnexB ) {
    satrtPos = 0;
  } else {
    satrtPos = startCodeSize;
  }

  for ( Int i = satrtPos; i <= size; i++ ) {
    if ( !isAnnexB ) {
      int orig_nalu_len = 0;
      for ( Int j = startCodeSize; j > 1; j-- ) {
        orig_nalu_len += ( data[i + startCodeSize - j] ) << ( ( j - 1 ) * 8 );
      }
      orig_nalu_len += data[i + startCodeSize - 1];
      std::cout << "nalu size=" << orig_nalu_len << std::endl;
      int iNalType = ( ( data[index + startCodeSize] ) ) & 0x1f;

      decCavlc->setBuffer( (UChar*)( data + index + startCodeSize + 1 ), orig_nalu_len - startCodeSize );
      switch ( iNalType ) {
        case NALU_TYPE_SPS:
          TComSPS_avc* pcSPS = decCavlc->getSPS();
          decCavlc->parseSPS( pcSPS );
          width    = decCavlc->getSPS()->getOutputWidth();
          height   = decCavlc->getSPS()->getOutputHeight();
          bitDepth = decCavlc->getSPS()->getBitDepth();
          is444    = decCavlc->getSPS()->getIs444();
          break;
      }
      createNalu( currentPoc, buffer, index, orig_nalu_len );
      nalNumber++;

      if ( i < size ) {
        index = i;
        i += ( orig_nalu_len - 1 );
      }
      if ( iNalType == NALU_TYPE_SPS ) { i = size; }

    } else {
      if ( i == size || ( ( data[i + 0] == 0x00 ) && ( data[i + 1] == 0x00 ) &&
                          ( ( ( data[i + 2] == 0x00 ) && ( data[i + 3] == 0x01 ) ) || ( data[i + 2] == 0x01 ) ) ) ) {
        int iNalType = ( ( data[index + startCodeSize] ) ) & 0x1f;
        int iPoc     = 0;
        decCavlc->setBuffer( (UChar*)( data + index + startCodeSize + 1 ), ( i - index ) - startCodeSize - 1 );

        switch ( iNalType ) {
          case NALU_TYPE_SPS:
            TComSPS_avc* pcSPS = decCavlc->getSPS();
            decCavlc->parseSPS( pcSPS );
            width    = decCavlc->getSPS()->getOutputWidth();
            height   = decCavlc->getSPS()->getOutputHeight();
            bitDepth = decCavlc->getSPS()->getBitDepth();
            is444    = decCavlc->getSPS()->getIs444();
            break;
        }
        createNalu( currentPoc, buffer, index, i - index );
        nalNumber++;
        if ( i < size ) {
          startCodeSize = data[i + 2] == 0 ? 4 : 3;
          index         = i;
          i += startCodeSize;
        }
      }
    }
  }
  delete decCavlc;
}
