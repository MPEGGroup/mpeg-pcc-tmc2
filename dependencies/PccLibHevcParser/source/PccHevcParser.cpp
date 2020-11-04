#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <fstream>

#include <list>
#include <algorithm>
#include <vector>

#include "PccHevcParser.h"
#include "PccHevcTDecCAVLC.h"

using namespace std;
using namespace pcc_hevc;

const char* getNaluType( int iNaluType ) {
  switch( iNaluType ) {
    case NAL_UNIT_CODED_SLICE_TRAIL_N   : return "TRAIL_N  "; break;
    case NAL_UNIT_CODED_SLICE_TRAIL_R   : return "TRAIL_R  "; break;
    case NAL_UNIT_CODED_SLICE_TSA_N     : return "TSA_N    "; break;
    case NAL_UNIT_CODED_SLICE_TSA_R     : return "TSA_R    "; break;
    case NAL_UNIT_CODED_SLICE_STSA_N    : return "STSA_N   "; break;
    case NAL_UNIT_CODED_SLICE_STSA_R    : return "STSA_R   "; break;
    case NAL_UNIT_CODED_SLICE_RADL_N    : return "RADL_N   "; break;
    case NAL_UNIT_CODED_SLICE_RADL_R    : return "RADL_R   "; break;
    case NAL_UNIT_CODED_SLICE_RASL_N    : return "RASL_N   "; break;
    case NAL_UNIT_CODED_SLICE_RASL_R    : return "RASL_R   "; break;
    case NAL_UNIT_RESERVED_VCL_N10      : return "RRVCL_N10"; break;
    case NAL_UNIT_RESERVED_VCL_R11      : return "RRVCL_R11"; break;
    case NAL_UNIT_RESERVED_VCL_N12      : return "RRVCL_N12"; break;
    case NAL_UNIT_RESERVED_VCL_R13      : return "RRVCL_R13"; break;
    case NAL_UNIT_RESERVED_VCL_N14      : return "RRVCL_N14"; break;
    case NAL_UNIT_RESERVED_VCL_R15      : return "RRVCL_R15"; break;
    case NAL_UNIT_CODED_SLICE_BLA_W_LP  : return "BLA_W_LP "; break;
    case NAL_UNIT_CODED_SLICE_BLA_W_RADL: return "BLA_WRADL"; break;
    case NAL_UNIT_CODED_SLICE_BLA_N_LP  : return "BLA_NLP  "; break;
    case NAL_UNIT_CODED_SLICE_IDR_W_RADL: return "IDR_WRADL"; break;
    case NAL_UNIT_CODED_SLICE_IDR_N_LP  : return "IDR_NLP  "; break;
    case NAL_UNIT_CODED_SLICE_CRA       : return "CRA      "; break;
    case NAL_UNIT_RESERVED_IRAP_VCL22   : return "RRIRAP22 "; break;
    case NAL_UNIT_RESERVED_IRAP_VCL23   : return "RRIRAP23 "; break;
    case NAL_UNIT_RESERVED_VCL24        : return "RRVCL24  "; break;
    case NAL_UNIT_RESERVED_VCL25        : return "RRVCL25  "; break;
    case NAL_UNIT_RESERVED_VCL26        : return "RRVCL26  "; break;
    case NAL_UNIT_RESERVED_VCL27        : return "RRVCL27  "; break;
    case NAL_UNIT_RESERVED_VCL28        : return "RRVCL28  "; break;
    case NAL_UNIT_RESERVED_VCL29        : return "RRVCL29  "; break;
    case NAL_UNIT_RESERVED_VCL30        : return "RRVCL30  "; break;
    case NAL_UNIT_RESERVED_VCL31        : return "RRVCL31  "; break;
    case NAL_UNIT_VPS                   : return "VPS      "; break;
    case NAL_UNIT_SPS                   : return "SPS      "; break;
    case NAL_UNIT_PPS                   : return "PPS      "; break;
    case NAL_UNIT_ACCESS_UNIT_DELIMITER : return "AU_DEL   "; break;
    case NAL_UNIT_EOS                   : return "EOS      "; break;
    case NAL_UNIT_EOB                   : return "EOB      "; break;
    case NAL_UNIT_FILLER_DATA           : return "FILLER   "; break;
    case NAL_UNIT_PREFIX_SEI            : return "PRE_SEI  "; break;
    case NAL_UNIT_SUFFIX_SEI            : return "SUF_SEI  "; break;
    case NAL_UNIT_RESERVED_NVCL41       : return "RRNVCL41 "; break;
    case NAL_UNIT_RESERVED_NVCL42       : return "RRNVCL42 "; break;
    case NAL_UNIT_RESERVED_NVCL43       : return "RRNVCL43 "; break;
    case NAL_UNIT_RESERVED_NVCL44       : return "RRNVCL44 "; break;
    case NAL_UNIT_RESERVED_NVCL45       : return "RRNVCL45 "; break;
    case NAL_UNIT_RESERVED_NVCL46       : return "RRNVCL46 "; break;
    case NAL_UNIT_RESERVED_NVCL47       : return "RRNVCL47 "; break;
    case NAL_UNIT_UNSPECIFIED_48        : return "U48      "; break;
    case NAL_UNIT_UNSPECIFIED_49        : return "U49      "; break;
    case NAL_UNIT_UNSPECIFIED_50        : return "U50      "; break;
    case NAL_UNIT_UNSPECIFIED_51        : return "U51      "; break;
    case NAL_UNIT_UNSPECIFIED_52        : return "U52      "; break;
    case NAL_UNIT_UNSPECIFIED_53        : return "U53      "; break;
    case NAL_UNIT_UNSPECIFIED_54        : return "U54      "; break;
    case NAL_UNIT_UNSPECIFIED_55        : return "U55      "; break;
    case NAL_UNIT_UNSPECIFIED_56        : return "U56      "; break;
    case NAL_UNIT_UNSPECIFIED_57        : return "U57      "; break;
    case NAL_UNIT_UNSPECIFIED_58        : return "U58      "; break;
    case NAL_UNIT_UNSPECIFIED_59        : return "U59      "; break;
    case NAL_UNIT_UNSPECIFIED_60        : return "U60      "; break;
    case NAL_UNIT_UNSPECIFIED_61        : return "U61      "; break;
    case NAL_UNIT_UNSPECIFIED_62        : return "U62      "; break;
    case NAL_UNIT_UNSPECIFIED_63        : return "U63      "; break;
    case NAL_UNIT_INVALID               : return "INVALID  "; break;
    default                             : return "ERROR    "; break;
  }
  return "ERROR";
}

PccHevcParser::PccHevcParser() {
  frames_.clear();
  vps_.clear();
  sps_.clear();
  pps_.clear();
}

PccHevcParser::~PccHevcParser() {
  frames_.clear();
  vps_.clear();
  sps_.clear();
  pps_.clear();
}

void PccHevcParser::getVideoSize( const std::vector<uint8_t>& buffer, size_t& width, size_t& height ) {
  setBuffer( buffer, width, height );
}
void PccHevcParser::display() {
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
  printf( " sum = %zu \n", sum );
}

void PccHevcParser::createNalu( const size_t frameIndex,
                                const std::vector<uint8_t>& buffer,
                                const size_t pos,
                                const size_t size ) {
  PccHevcNalu nalu;
  nalu.add( buffer, pos, size );
  switch( nalu.getNaluType() ) {
    case NAL_UNIT_VPS : vps_.push_back( nalu ); break;
    case NAL_UNIT_SPS : sps_.push_back( nalu ); break;
    case NAL_UNIT_PPS : pps_.push_back( nalu ); break;
    default:
     if( frames_.size() <= frameIndex ) {
       frames_.resize( frameIndex + 1 );
     }
     frames_[frameIndex ].add( nalu );
     break;
  }
}

void PccHevcParser::setBuffer( const std::vector<uint8_t>& buffer, size_t& width, size_t& height ) {
  const int      size                   = (int)buffer.size();
  const uint8_t* data                   = buffer.data();
  TDecCavlc* decCavlc    = new TDecCavlc();
  int            nalNumber              = 0;
  int            index                  = 0;
  int            startCodeSize          = 4;
  int            frameIndex             = -1;
  int            maxPocFound            = -1;
  int            sequencePoc            = 0;
  int            previousNaluLayerIndex = -1;
  int            currentPoc             = 0;
  for ( Int i = startCodeSize; i <= size; i++ ) {
    if ( i == size || ( ( data[i + 0] == 0x00 ) && ( data[i + 1] == 0x00 ) &&
                        ( ( ( data[i + 2] == 0x00 ) && ( data[i + 3] == 0x01 ) ) || ( data[i + 2] == 0x01 ) ) ) ) {
      int iNalType       = (   ( data[ index + startCodeSize     ] ) &  126 )>> 1 ;
      int iLayer         = ( ( ( data[ index + startCodeSize     ] ) &  1 ) << 6 ) +
                           ( ( ( data[ index + startCodeSize + 1 ] ) &  248 ) >> 3 );
      int iTemporalIndex = (   ( data[ index + startCodeSize + 1 ] ) & 7 ) + 1;
      int iPoc = 0;
      if( iLayer != 0 )
        return ;
      decCavlc->setBuffer( (UChar*)( data + index + startCodeSize + 2 ),  ( i - index ) - startCodeSize - 2);

      switch ( iNalType ) {
        case NAL_UNIT_VPS:                                break;
        case NAL_UNIT_SPS: 
          decCavlc->parseSPS( decCavlc->getSPS() ); 
          width    = decCavlc->getSPS()->getOutputWidth();
          height   = decCavlc->getSPS()->getOutputHeight();
          // bitdepth = decCavlc->getSPS(iLayer)->getBitDepthY();
         break;
         /*
        case NAL_UNIT_PPS: decCavlc->parsePPS( decCavlc->getPPS());  break;
        case NAL_UNIT_CODED_SLICE_TRAIL_N:
        case NAL_UNIT_CODED_SLICE_TRAIL_R:
        case NAL_UNIT_CODED_SLICE_TSA_N:
        case NAL_UNIT_CODED_SLICE_TSA_R:
        case NAL_UNIT_CODED_SLICE_STSA_N:
        case NAL_UNIT_CODED_SLICE_STSA_R:
        case NAL_UNIT_CODED_SLICE_RADL_N:
        case NAL_UNIT_CODED_SLICE_RADL_R:
        case NAL_UNIT_CODED_SLICE_RASL_N:
        case NAL_UNIT_CODED_SLICE_RASL_R:
        case NAL_UNIT_CODED_SLICE_BLA_W_LP:
        case NAL_UNIT_CODED_SLICE_BLA_W_RADL:
        case NAL_UNIT_CODED_SLICE_BLA_N_LP:
        case NAL_UNIT_CODED_SLICE_IDR_W_RADL:
        case NAL_UNIT_CODED_SLICE_IDR_N_LP:
        case NAL_UNIT_CODED_SLICE_CRA:
          iPoc = decCavlc->parseSliceHeader( iLayer, (NalUnitType)iNalType, iTemporalIndex );
          if( iNalType == NAL_UNIT_CODED_SLICE_IDR_W_RADL ||
              iNalType == NAL_UNIT_CODED_SLICE_IDR_N_LP   ||
              iNalType == NAL_UNIT_CODED_SLICE_BLA_N_LP   ||
              iNalType == NAL_UNIT_CODED_SLICE_BLA_W_RADL ||
              iNalType == NAL_UNIT_CODED_SLICE_BLA_W_LP   ) {
            sequencePoc += maxPocFound + 1;
            maxPocFound = -1;
          } else {
            if( iPoc >= maxPocFound ) {
              maxPocFound = iPoc;
            }
          }
          if( previousNaluLayerIndex != iLayer && iLayer == 0 ) {
            frameIndex++;
          }
          previousNaluLayerIndex = iLayer;
          currentPoc = sequencePoc + iPoc;
          */
      }
      createNalu( currentPoc, buffer, index, i - index );
      nalNumber++;
      if( i < size ) {
        startCodeSize = data[ i + 2 ] == 0 ? 4 : 3;
        index = i;
        i += startCodeSize;
      }
    }
  }
  delete decCavlc;
}
