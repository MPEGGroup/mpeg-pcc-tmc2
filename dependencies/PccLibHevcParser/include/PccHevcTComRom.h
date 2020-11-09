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

/** \file     TComRom.h
    \brief    global variables & functions (header)
*/

#ifndef __PCC_HEVC_TCOMROM__
#define __PCC_HEVC_TCOMROM__

#include "PccHevcCommonDef.h"

#include<stdio.h>
#include<iostream>

extern       unsigned int*  gPccHevc_scanOrder[pcc_hevc::SCAN_NUMBER_OF_GROUP_TYPES][pcc_hevc::SCAN_NUMBER_OF_TYPES][ pcc_hevc::MAX_CU_DEPTH + 1 ][ pcc_hevc::MAX_CU_DEPTH + 1 ];
extern const unsigned int gPccHevc_scalingListSize [pcc_hevc::SCALING_LIST_SIZE_NUM];
extern const unsigned int gPccHevc_scalingListSizeX[pcc_hevc::SCALING_LIST_SIZE_NUM];

extern const char *gPccHevc_MatrixType[pcc_hevc::SCALING_LIST_SIZE_NUM][pcc_hevc::SCALING_LIST_NUM];
extern const char *gPccHevc_MatrixType_DC[pcc_hevc::SCALING_LIST_SIZE_NUM][pcc_hevc::SCALING_LIST_NUM];

extern const int gPccHevc_quantTSDefault4x4[4*4];
extern const int gPccHevc_quantIntraDefault8x8[8*8];
extern const int gPccHevc_quantInterDefault8x8[8*8];

// flexible conversion from relative to absolute index
extern       unsigned int   gPccHevc_auiZscanToRaster[ pcc_hevc::MAX_NUM_PART_IDXS_IN_CTU_WIDTH*pcc_hevc::MAX_NUM_PART_IDXS_IN_CTU_WIDTH ];
extern       unsigned int   gPccHevc_auiRasterToZscan[ pcc_hevc::MAX_NUM_PART_IDXS_IN_CTU_WIDTH*pcc_hevc::MAX_NUM_PART_IDXS_IN_CTU_WIDTH ];
// conversion of partition index to picture pel position
extern       unsigned int   gPccHevc_auiRasterToPelX[ pcc_hevc::MAX_NUM_PART_IDXS_IN_CTU_WIDTH*pcc_hevc::MAX_NUM_PART_IDXS_IN_CTU_WIDTH ];
extern       unsigned int   gPccHevc_auiRasterToPelY[ pcc_hevc::MAX_NUM_PART_IDXS_IN_CTU_WIDTH*pcc_hevc::MAX_NUM_PART_IDXS_IN_CTU_WIDTH ];

extern char gPccHevc_ucMsbP1Idx[256];

extern       signed char   gPccHevc_aucConvertToBit  [ pcc_hevc::MAX_CU_SIZE+1 ];   // from width to log2(width)-2

namespace pcc_hevc {

//! \ingroup TLibCommon
//! \{

// ====================================================================================================================
// Initialize / destroy functions
// ====================================================================================================================

Void         initROMPccHevc();
Void         destroyROMPccHevc();

// ====================================================================================================================
// Data structure related table & variable
// ====================================================================================================================


// Void         initZscanToRasterPccHevc ( Int iMaxDepth, Int iDepth, UInt uiStartVal, UInt*& rpuiCurrIdx );
Void         initRasterToZscanPccHevc ( UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxDepth         );


Void         initRasterToPelXYPccHevc ( UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxDepth );

extern const UInt gPccHevc_auiPUOffset[NUMBER_OF_PART_SIZES];

extern const Int gPccHevc_quantScales[SCALING_LIST_REM_NUM];             // Q(QP%6)
extern const Int gPccHevc_invQuantScales[SCALING_LIST_REM_NUM];          // IQ(QP%6)

#if RExt__HIGH_PRECISION_FORWARD_TRANSFORM
static const Int gPccHevc_transformMatrixShift[TRANSFORM_NUMBER_OF_DIRECTIONS] = { 14, 6 };
#else
static const Int gPccHevc_transformMatrixShift[TRANSFORM_NUMBER_OF_DIRECTIONS] = {  6, 6 };
#endif

extern const TMatrixCoeff gPccHevc_aiT4 [TRANSFORM_NUMBER_OF_DIRECTIONS][4][4];
extern const TMatrixCoeff gPccHevc_aiT8 [TRANSFORM_NUMBER_OF_DIRECTIONS][8][8];
extern const TMatrixCoeff gPccHevc_aiT16[TRANSFORM_NUMBER_OF_DIRECTIONS][16][16];
extern const TMatrixCoeff gPccHevc_aiT32[TRANSFORM_NUMBER_OF_DIRECTIONS][32][32];

// ====================================================================================================================
// Luma QP to Chroma QP mapping
// ====================================================================================================================

static const Int chromaQPMappingTableSize = 58;

extern const UChar  gPccHevc_aucChromaScale[NUM_CHROMA_FORMAT][chromaQPMappingTableSize];


// ====================================================================================================================
// Scanning order & context mapping table
// ====================================================================================================================

extern const UInt   ctxIndMap4x4[4*4];

extern const UInt   gPccHevc_uiGroupIdx[ MAX_TU_SIZE ];
extern const UInt   gPccHevc_uiMinInGroup[ LAST_SIGNIFICANT_GROUPS ];

// ====================================================================================================================
// Intra prediction table
// ====================================================================================================================

extern const UChar  gPccHevc_aucIntraModeNumFast_UseMPM[MAX_CU_DEPTH];
extern const UChar  gPccHevc_aucIntraModeNumFast_NotUseMPM[MAX_CU_DEPTH];

extern const UChar  gPccHevc_chroma422IntraAngleMappingTable[NUM_INTRA_MODE];

// ====================================================================================================================
extern const UChar  gPccHevc_uhPaletteTBC[257];

// Mode-Dependent DST Matrices
// ====================================================================================================================

extern const TMatrixCoeff gPccHevc_as_DST_MAT_4 [TRANSFORM_NUMBER_OF_DIRECTIONS][4][4];

// ====================================================================================================================
// Misc.
// ====================================================================================================================

#if ENC_DEC_TRACE
extern FILE* gPccHevc_hTrace;
extern Bool  gPccHevc_bJustDoIt;
extern const Bool gPccHevc_bEncDecTraceEnable;
extern const Bool gPccHevc_bEncDecTraceDisable;
extern Bool  gPccHevc_HLSTraceEnable;
extern UInt64 gPccHevc_nSymbolCounter;

#define COUNTER_START    1
#define COUNTER_END      0 //( UInt64(1) << 63 )

#define DTRACE_CABAC_F(x)     if ( (gPccHevc_nSymbolCounter >= COUNTER_START && gPccHevc_nSymbolCounter <= COUNTER_END )||gPccHevc_bJustDoIt ) fprintf(gPccHevc_hTrace, "%f", x );
#define DTRACE_CABAC_V(x)     if ( (gPccHevc_nSymbolCounter >= COUNTER_START && gPccHevc_nSymbolCounter <= COUNTER_END )||gPccHevc_bJustDoIt ) fprintf(gPccHevc_hTrace, "%d", x );
#define DTRACE_CABAC_VL(x)    if ( (gPccHevc_nSymbolCounter >= COUNTER_START && gPccHevc_nSymbolCounter <= COUNTER_END )||gPccHevc_bJustDoIt ) fprintf(gPccHevc_hTrace, "%lld", x );
#define DTRACE_CABAC_T(x)     if ( (gPccHevc_nSymbolCounter >= COUNTER_START && gPccHevc_nSymbolCounter <= COUNTER_END )||gPccHevc_bJustDoIt ) fprintf(gPccHevc_hTrace, "%s", x );
#define DTRACE_CABAC_X(x)     if ( (gPccHevc_nSymbolCounter >= COUNTER_START && gPccHevc_nSymbolCounter <= COUNTER_END )||gPccHevc_bJustDoIt ) fprintf(gPccHevc_hTrace, "%x", x );
#define DTRACE_CABAC_R( x,y ) if ( (gPccHevc_nSymbolCounter >= COUNTER_START && gPccHevc_nSymbolCounter <= COUNTER_END )||gPccHevc_bJustDoIt ) fprintf(gPccHevc_hTrace, x,    y );
#define DTRACE_CABAC_N        if ( (gPccHevc_nSymbolCounter >= COUNTER_START && gPccHevc_nSymbolCounter <= COUNTER_END )||gPccHevc_bJustDoIt ) fprintf(gPccHevc_hTrace, "\n"    );

#else

#define DTRACE_CABAC_F(x)
#define DTRACE_CABAC_V(x)
#define DTRACE_CABAC_VL(x)
#define DTRACE_CABAC_T(x)
#define DTRACE_CABAC_X(x)
#define DTRACE_CABAC_R( x,y )
#define DTRACE_CABAC_N

#endif

const TChar* nalUnitTypeToString(NalUnitType type);

extern UChar getMsbP1Idx(UInt uiVal);
//! \}

}; // namespace pcc_hevc

#endif  //__TCOMROM__

