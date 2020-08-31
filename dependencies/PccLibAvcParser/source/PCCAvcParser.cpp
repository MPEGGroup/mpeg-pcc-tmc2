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

PCCAvcParser::PCCAvcParser() {
  frames_.clear();
  vps_.clear();
  sps_.clear();
  pps_.clear();
}

PCCAvcParser::~PCCAvcParser() {
  frames_.clear();
  vps_.clear();
  sps_.clear();
  pps_.clear();
}

const char* PCCAvcParser::getNaluType( int iNaluType ) {
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

    case NAL_UNIT_CODED_SLICE_BLA_W_LP: return "SLICE_BLA_W_LP  "; break;
    case NAL_UNIT_CODED_SLICE_BLA_W_RADL: return "SLICE_BLA_W_RADL  "; break;
    case NAL_UNIT_CODED_SLICE_BLA_N_LP: return "SLICE_BLA_N_LP  "; break;
    case NAL_UNIT_CODED_SLICE_IDR_W_RADL: return "SLICE_IDR_W_RADL  "; break;
    case NAL_UNIT_CODED_SLICE_IDR_N_LP: return "SLICE_IDR_N_LP  "; break;
    case NAL_UNIT_CODED_SLICE_CRA: return "SLICE_CRA  "; break;

    default: return "ERROR    "; break;
  }
  return "ERROR";
}

void PCCAvcParser::getVideoSize( const std::vector<uint8_t>& buffer, size_t& width, size_t& height, int CodecId ) {
  //std::cout << "HELLO!!!getVideoSize!!!"<< std::endl;
  setBuffer( buffer, width, height,  CodecId );
  //std::cout << "BYE!!!getVideoSize!!!" << std::endl;
}
void PCCAvcParser::display() {
  int    poc = 0;
  size_t sum = 0;
  for ( auto& nalu : vps_ ) {
    //printf( "%4d / %4lu : Size = %8lu type = %4lu %9s VPS \n", poc, frames_.size(), nalu.size(), nalu.getNaluType(),
    //        getNaluType( nalu.getNaluType() ) );
    sum += nalu.size();
  }
  for ( auto& nalu : sps_ ) {
    //printf( "%4d / %4lu : Size = %8lu type = %4lu %9s SPS \n", poc, frames_.size(), nalu.size(), nalu.getNaluType(),
    //        getNaluType( nalu.getNaluType() ) );
    sum += nalu.size();
  }
  for ( auto& nalu : pps_ ) {
    //printf( "%4d / %4lu : Size = %8lu type = %4lu %9s PPS \n", poc, frames_.size(), nalu.size(), nalu.getNaluType(),
    //        getNaluType( nalu.getNaluType() ) );
    sum += nalu.size();
  }
  for ( auto& frame : frames_ ) {
    for ( auto& nalu : frame.getNalu() ) {
      //printf( "%4d / %4lu : Size = %8lu type = %4lu %9s \n", poc, frames_.size(), nalu.size(), nalu.getNaluType(),
      //        getNaluType( nalu.getNaluType() ) );
      sum += nalu.size();
    }
    poc++;
  }
  //printf( " sum = %lu \n", sum );
}

void PCCAvcParser::createNalu( const size_t                frameIndex,
                               const std::vector<uint8_t>& buffer,
                               const size_t                pos,
                               const size_t                size ) {
  PCCAvcNalu nalu;
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

void PCCAvcParser::setBuffer( const std::vector<uint8_t>& buffer, size_t& width, size_t& height, int CodecId ) {
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

  int             satrtPos = 0;
  //std::cout << "HELLO!!!setBuffer!!!"<< std::endl;
  
  if ( CodecId == 3 ) {
    satrtPos = 0;  
  } 
  else 
  {
    satrtPos = startCodeSize;
  }

   for ( Int i = satrtPos; i <= size; i++ ) {
    if ( CodecId == 3 ) {
        
       //std::cout <<"i = "<<i<< " nalu header: " << (int)data[i + 0] << " " << (int)data[i + 1] << " " << (int)data[i + 2] << " "
        //        << (int)data[i + 3] << std::endl;
       
       int orig_nalu_len = 0;
       for ( Int j = startCodeSize;j > 1; j-- ) {
         
        orig_nalu_len += ( data[i+startCodeSize - j] ) << ( ( j - 1 ) * 8 );
         //std::cout << "data[i+startCodeSize - j]=" << (int)data[i + startCodeSize - j] << " orig_nalu_len=" << orig_nalu_len
          //         << std::endl;
         // printf("%d %d %d\n", i, annex_b->Buf[nalu_haeder_size - i], orig_nalu_len);
       }
       orig_nalu_len += data[i+startCodeSize-1];
       //std::cout << "data[i+startCodeSize-1]=" << (int)data[i+startCodeSize-1]
       //          << " orig_nalu_len=" << orig_nalu_len << std::endl;

       std::cout << "nalu size=" << orig_nalu_len << std::endl;

       int iNalType = ( ( data[index + startCodeSize] ) ) & 0x1f;
       
       //std::cout << "data[index + startCodeSize]=" << (int)data[index + startCodeSize] << "iNalType=" << iNalType
       //          << std::endl;

       decCavlc->setBuffer( (UChar*)( data + index + startCodeSize + 1 ), orig_nalu_len - startCodeSize );
       switch ( iNalType ) {
         case NALU_TYPE_SPS:
           //std::cout << "HELLO!!!NALU_TYPE_SPS!!!" << std::endl;
           TComSPS_avc* pcSPS = decCavlc->getSPS();
           decCavlc->parseSPS( pcSPS );
           width  = decCavlc->getSPS()->getOutputWidth();
           height = decCavlc->getSPS()->getOutputHeight();
           break;
       }
       //std::cout << "HELLO!!!createNalu!!!" << std::endl;
       createNalu( currentPoc, buffer, index, orig_nalu_len );
       //std::cout << "BYE!!!createNalu!!!" << std::endl;
       nalNumber++;
       
       if ( i < size ) {
         //startCodeSize = data[i + 2] == 0 ? 4 : 3;
         index = i;
         i += (orig_nalu_len-1);
         //std::cout << "index" << index << "i=" << i << std::endl;
       }
       if ( iNalType == NALU_TYPE_SPS ) { 
           i = size;
       }

    } else {
    
     if ( i == size || ( ( data[i + 0] == 0x00 ) && ( data[i + 1] == 0x00 ) &&
                        ( ( ( data[i + 2] == 0x00 ) && ( data[i + 3] == 0x01 ) ) || ( data[i + 2] == 0x01 ) ) ) ) {
      // int iNalType = ( ( data[index + startCodeSize] ) & 126 ) >> 1;
      int iNalType = ( ( data[index + startCodeSize] ) ) & 0x1f;
      //std::cout << "iNalType=" << iNalType << std::endl;
      // if ( iNalType == 7 ) printf("iNalType=7!!!\n");
      //if ( iNalType == NALU_TYPE_SPS ) printf( "iNalType=NALU_TYPE_SPS!!!\n" );
      // int iLayer =
      //    ( ( ( data[index + startCodeSize] ) & 1 ) << 6 ) + ( ( ( data[index + startCodeSize + 1] ) & 248 ) >> 3 );
      // int iTemporalIndex = ( ( data[index + startCodeSize + 1] ) & 7 ) + 1;
      int iPoc = 0;
      // if ( iLayer != 0 ) return;
      // decCavlc->setBuffer( (UChar*)( data + index + startCodeSize + 2 ), ( i - index ) - startCodeSize - 2 );
      decCavlc->setBuffer( (UChar*)( data + index + startCodeSize + 1 ), ( i - index ) - startCodeSize - 1 );

      switch ( iNalType ) {
        // case NAL_UNIT_VPS:                                break;
        case NALU_TYPE_SPS:

          //std::cout << "HELLO!!!NALU_TYPE_SPS!!!" << std::endl;
          TComSPS_avc* pcSPS = decCavlc->getSPS();
          decCavlc->parseSPS( pcSPS );
          // decCavlc->parseSPS( decCavlc->getSPS() );
          width  = decCavlc->getSPS()->getOutputWidth();
          height = decCavlc->getSPS()->getOutputHeight();
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
      if ( i < size ) {
        startCodeSize = data[i + 2] == 0 ? 4 : 3;
        index         = i;
        i += startCodeSize;
      }
    }
    }
   }
   delete decCavlc;
   //std::cout << "BYE!!!setBuffer!!!" << std::endl;
  
}
