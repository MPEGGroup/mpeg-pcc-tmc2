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

/** \file     TDecCAVLC.h
    \brief    CAVLC decoder class (header)
*/

#ifndef __TDECCAVLC__
#define __TDECCAVLC__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TypeDef.h"
#include "CommonDef.h"
#include "TComSlice.h"

#if Q0048_CGS_3D_ASYMLUT
#include "TCom3DAsymLUT.h"
#endif

// #include "TDecEntropy.h"
// #include "SyntaxElementParser.h"

#if Q0048_CGS_3D_ASYMLUT
class TCom3DAsymLUT;
#endif
//! \ingroup TLibDecoder
//! \{

class Reader
{
public:
  Reader() : m_iReadBytes( 0 ), m_iReadBits( 0 ) {}
  ~Reader(){}

  void setBuffer( UChar* pBuffer, int iSize  )
  {
    m_iSize      = iSize;
    m_pBuffer    = pBuffer;
    m_iReadBytes = 0;
    m_iReadBits  = 0;
    m_iZeroNum   = 0;
    loadBuffer( m_iBufferA );
    loadBuffer( m_iBufferB );
  }
  Void readCode (UInt uiLength, UInt& ruiCode) { ruiCode = read( uiLength ); }
  Void readFlag (UInt& ruiCode) { ruiCode = read( 1 ); }
  Void readUvlc( UInt& ruiVal)
  {
    UInt uiVal = 0;
    UInt uiCode = 0;
    UInt uiLength;
    uiCode = read( 1 );
    if( 0 == uiCode )
    {
      uiLength = 0;
      while( ! ( uiCode & 1 ))
      {
        uiCode = read( 1 );
        uiLength++;
      }

      uiVal = read( uiLength );
      uiVal += (1 << uiLength)-1;
    }
    ruiVal = uiVal;
  }
  Void readSvlc( Int& riVal)
  {
    UInt uiBits = 0;
    uiBits = read( 1 );
    if( 0 == uiBits )
    {
      UInt uiLength = 0;

      while( ! ( uiBits & 1 ))
      {
        uiBits = read( 1 );
        uiLength++;
      }
      uiBits = read( uiLength );
      uiBits += (1 << uiLength);
      riVal = ( uiBits & 1) ? -(Int)(uiBits>>1) : (Int)(uiBits>>1);
    }
    else
    {
      riVal = 0;
    }
  }
  UInt  getNumBitsRead() { return m_iReadBits; }
private:
  Int read( Int iBits )
  {
    Int iValue = 0;
    m_iReadBits += iBits;
    if( m_iReadBits <= 32 )
    {
      iValue = m_iBufferA >> ( 32 - iBits );
      m_iBufferA <<= iBits;
    }
    else
    {
      m_iReadBits -= 32;
      iValue = m_iBufferA >> ( 32 - iBits ) | m_iBufferB >> ( 32 - m_iReadBits );
      m_iBufferA = m_iBufferB;
      loadBuffer( m_iBufferB );
      m_iBufferA <<= m_iReadBits;
    }
    return iValue;
  }

  UChar loadChar()
  {
    UChar ucChar = m_pBuffer[ m_iReadBytes ]; m_iReadBytes++;
    // printf(" get    %2x Zero = %d \n",ucChar,m_iZeroNum);
    if( ucChar <= 0x03 )
    {
      if( m_iZeroNum == 2 )
      {
        if( ucChar == 0x03 )
        {
          ucChar = m_pBuffer[ m_iReadBytes ]; m_iReadBytes++;
          // printf(" reload %2x Zero = %d \n",ucChar,m_iZeroNum);
        }
        m_iZeroNum = 0;
        if( ucChar <= 0x03 )
        {
          m_iZeroNum++;
        }
      }
      else
      {
        m_iZeroNum++;
      }
    }
    else
    {
      m_iZeroNum = 0;
    }
    // printf(" push   %2x Zero = %d \n",ucChar,m_iZeroNum);
    return ucChar;
  }

  void loadBuffer( UInt &iBuffer )
  {
    if( m_iReadBytes < m_iSize )
    {
      iBuffer  = loadChar(); iBuffer <<= 8;
      iBuffer |= loadChar(); iBuffer <<= 8;
      iBuffer |= loadChar(); iBuffer <<= 8;
      iBuffer |= loadChar();
    }
  }
  UInt m_iBufferA;
  UInt m_iBufferB;
  Int  m_iReadBytes;
  Int  m_iReadBits;
  UChar *m_pBuffer;
  Int  m_iSize;
  Int  m_iZeroNum;
};


class ParameterSetManagerDecoder:public ParameterSetManager
{
public:
  ParameterSetManagerDecoder();
  virtual ~ParameterSetManagerDecoder();
  Void     storePrefetchedVPS(TComVPS *vps)  { m_vpsBuffer.storePS( vps->getVPSId(), vps); };
  TComVPS* getPrefetchedVPS  (Int vpsId);
  Void     storePrefetchedSPS(TComSPS *sps)  { m_spsBuffer.storePS( sps->getSPSId(), sps); };
  TComSPS* getPrefetchedSPS  (Int spsId);
  Void     storePrefetchedPPS(TComPPS *pps)  { m_ppsBuffer.storePS( pps->getPPSId(), pps); };
  TComPPS* getPrefetchedPPS  (Int ppsId);
  Void     applyPrefetchedPS();

private:
#if SVC_EXTENSION
  static ParameterSetMap<TComVPS> m_vpsBuffer;
#else
  ParameterSetMap<TComVPS> m_vpsBuffer;
#endif
  ParameterSetMap<TComSPS> m_spsBuffer;
  ParameterSetMap<TComPPS> m_ppsBuffer;
};


// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// CAVLC decoder class
class TDecCavlc // : public SyntaxElementParser, public TDecEntropyIf
{
public:
  TDecCavlc();
  virtual ~TDecCavlc();
  

  void setBuffer( UChar* pBuffer, int iSize  ) { m_eReader.setBuffer( pBuffer, iSize ); }

  void parseVps();
  void parseSps( int iLayerIndex );
  void parsePps( int iLayerIndex );
  int parseSliceHeader( int iLayerIndex, NalUnitType iNaluType, int iTemporalIndex );

protected:
  void  parseShortTermRefPicSet            (TComSPS* pcSPS, TComReferencePictureSet* pcRPS, Int idx);

#if !TRACE_DEBUG
  #define READ_CODE(length, code, name )  m_eReader.readCode( length, code );
  #define READ_UVLC(        code, name )  m_eReader.readUvlc(         code );
  #define READ_SVLC(        code, name )  m_eReader.readSvlc(         code );
  #define READ_FLAG(        code, name )  m_eReader.readFlag(         code );
#else
  #define READ_CODE(length, code, name )  m_eReader.readCode( length, code ); printf("READ_CODE(%2d) = %6d : %s \n",length,code,name); fflush(stdout);
  #define READ_UVLC(        code, name )  m_eReader.readUvlc(         code ); printf("READ_UVLC()   = %6d : %s \n",       code,name);  fflush(stdout);
  #define READ_SVLC(        code, name )  m_eReader.readSvlc(         code ); printf("READ_SVLC()   = %6d : %s \n",       code,name);  fflush(stdout);
  #define READ_FLAG(        code, name )  m_eReader.readFlag(         code ); printf("READ_FLAG()   = %6d : %s \n",       code,name);  fflush(stdout);
#endif

public:
  /// rest entropy coder by intial QP and IDC in CABAC
  // Void  resetEntropy        ( TComSlice* /*pcSlice*/  )     { assert(0); };
  // Void  setBitstream        ( TComInputBitstream* p )   { m_pcBitstream = p; }
  Void  parseTransformSubdivFlag( UInt& ruiSubdivFlag, UInt uiLog2TransformBlockSize );
  // Void  parseQtCbf          ( TComDataCU* pcCU, UInt uiAbsPartIdx, TextType eType, UInt uiTrDepth, UInt uiDepth );
  // Void  parseQtRootCbf      ( UInt uiAbsPartIdx, UInt& uiQtRootCbf );
  Void  parseVPS            ( );
#if SVC_EXTENSION
  Void  parseVPSExtension   ( TComVPS* pcVPS );
  Void  defaultVPSExtension ( TComVPS* pcVPS );
  Void  parseVPSVUI         ( TComVPS* pcVPS );
  Void  defaultVPSVUI       ( TComVPS* pcVPS );
#if REPN_FORMAT_IN_VPS
  Void  parseRepFormat      ( RepFormat *repFormat, RepFormat *repFormatPrev );
#endif
#if VPS_DPB_SIZE_TABLE
  Void  parseVpsDpbSizeTable( TComVPS *vps );
#endif
#if VPS_VUI_BSP_HRD_PARAMS
  Void  parseVpsVuiBspHrdParams( TComVPS *vps );
#endif
#if SPS_DPB_PARAMS
  Void  parseSPS            ( TComSPS* pcSPS ); // it should be removed after macro clean up
#else
  Void  parseSPS            ( TComSPS* pcSPS, ParameterSetManagerDecoder *parameterSetManager );
#endif
  Void  parseSPSExtension    ( TComSPS* pcSPS );
#else //SVC_EXTENSION
  Void  parseSPS            ( TComSPS* pcSPS );
#endif //SVC_EXTENSION
  Void  parsePPS            (
#if Q0048_CGS_3D_ASYMLUT
    TCom3DAsymLUT * pc3DAsymLUT , Int nLayerID
#endif
    );
  Void  parseVUI            ( TComVUI* pcVUI, TComSPS* pcSPS );
  // Void  parseSEI            ( SEIMessages& );
  Void  parsePTL            ( TComPTL *rpcPTL, Bool profilePresentFlag, Int maxNumSubLayersMinus1 );
  Void  parseProfileTier    ( ProfileTierLevel *ptl);
  Void  parseHrdParameters  ( TComHRD *hrd, Bool cprms_present_flag, UInt tempLevelHigh);

  Void  parseSliceHeader    ( TComSlice*& rpcSlice,  ParameterSetManagerDecoder *parameterSetManager);

  Void  parseTerminatingBit ( UInt& ruiBit );
  
  Void  parseMVPIdx         ( Int& riMVPIdx );
  
  //Void  parseSkipFlag       ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  //Void  parseCUTransquantBypassFlag( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  //Void parseMergeFlag       ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth, UInt uiPUIdx );
  //Void parseMergeIndex      ( TComDataCU* pcCU, UInt& ruiMergeIndex );
  //Void parseSplitFlag       ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  //Void parsePartSize        ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  //Void parsePredMode        ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );

  //  Void parseIntraDirLumaAng ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  //
  //  Void parseIntraDirChroma  ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  //
  //  Void parseInterDir        ( TComDataCU* pcCU, UInt& ruiInterDir, UInt uiAbsPartIdx );
  //  Void parseRefFrmIdx       ( TComDataCU* pcCU, Int& riRefFrmIdx,  RefPicList eRefList );
  //  Void parseMvd             ( TComDataCU* pcCU, UInt uiAbsPartAddr,UInt uiPartIdx,    UInt uiDepth, RefPicList eRefList );
  //
  //  Void parseDeltaQP         ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth );
  //  Void parseCoeffNxN        ( TComDataCU* pcCU, TCoeff* pcCoef, UInt uiAbsPartIdx, UInt uiWidth, UInt uiHeight, UInt uiDepth, TextType eTType );
  //  Void parseTransformSkipFlags( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt width, UInt height, UInt uiDepth, TextType eTType);
  //
  //  Void parseIPCMInfo        ( TComDataCU* pcCU, UInt uiAbsPartIdx, UInt uiDepth);

//  Void updateContextTables  ( SliceType /*eSliceType*/, Int /*iQp*/ ) { return; }

  Void xParsePredWeightTable ( TComSlice* pcSlice );
  Void  parseScalingList     ( TComScalingList* scalingList );
  Void xDecodeScalingList    ( TComScalingList *scalingList, UInt sizeId, UInt listId);

  
 TComPPS* getPPS( int32_t layerIndex ){ return &m_pPPS[ layerIndex ]; }
 TComSPS* getSPS( int32_t layerIndex ){ return &m_pSPS[ layerIndex ]; }
 
protected:
  Bool  xMoreRbspData();

#if Q0048_CGS_3D_ASYMLUT
  Void xParse3DAsymLUT( TCom3DAsymLUT * pc3DAsymLUT );
  Void xParse3DAsymLUTOctant( TCom3DAsymLUT * pc3DAsymLUT , Int nDepth , Int yIdx , Int uIdx , Int vIdx , Int nLength );
#if R0151_CGS_3D_ASYMLUT_IMPROVE
#if R0300_CGS_RES_COEFF_CODING
  Void xReadParam( Int& param, Int flc_bits );
#else
  Void xReadParam( Int& param );
#endif
#endif
#endif


private:

  TComVPS         m_pVPS[MAX_NUM_VPS];
  TComPPS         m_pPPS[MAX_NUM_PPS];
  TComSPS         m_pSPS[MAX_NUM_SPS];

  Reader          m_eReader;
  TCom3DAsymLUT   m_c3DAsymLUTPPS;
  ParameterSetManagerDecoder m_parameterSetManagerDecoder;  // storage for parameter sets
  TComSlice*              m_apcSlicePilot;
};

//! \}

#endif // !defined(AFX_TDECCAVLC_H__9732DD64_59B0_4A41_B29E_1A5B18821EAD__INCLUDED_)

