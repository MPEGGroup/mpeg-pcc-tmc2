/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2016, ITU/ISO/IEC
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

#ifndef __PCC_SHVC_TCOMROM__
#define __PCC_SHVC_TCOMROM__

#include "PccShvcCommonDef.h"

#include<stdio.h>
#include<iostream>

//! \ingroup TLibCommon
//! \{

// flexible conversion from relative to absolute index
extern       unsigned int   gPccShvc_auiZscanToRaster[ pcc_shvc::MAX_NUM_PART_IDXS_IN_CTU_WIDTH*pcc_shvc::MAX_NUM_PART_IDXS_IN_CTU_WIDTH ];
extern       unsigned int   gPccShvc_auiRasterToZscan[ pcc_shvc::MAX_NUM_PART_IDXS_IN_CTU_WIDTH*pcc_shvc::MAX_NUM_PART_IDXS_IN_CTU_WIDTH ];
extern       unsigned int*  gPccShvc_scanOrder[pcc_shvc::SCAN_NUMBER_OF_GROUP_TYPES][pcc_shvc::SCAN_NUMBER_OF_TYPES][ pcc_shvc::MAX_CU_DEPTH ][ pcc_shvc::MAX_CU_DEPTH ];

// conversion of partition index to picture pel position
extern       unsigned int   gPccShvc_auiRasterToPelX[ pcc_shvc::MAX_NUM_PART_IDXS_IN_CTU_WIDTH*pcc_shvc::MAX_NUM_PART_IDXS_IN_CTU_WIDTH ];
extern       unsigned int   gPccShvc_auiRasterToPelY[ pcc_shvc::MAX_NUM_PART_IDXS_IN_CTU_WIDTH*pcc_shvc::MAX_NUM_PART_IDXS_IN_CTU_WIDTH ];

extern       signed char    gPccShvc_aucConvertToBit  [ pcc_shvc::MAX_CU_SIZE+1 ];   // from width to log2(width)-2

extern const unsigned int gPccShvc_scalingListSize [pcc_shvc::SCALING_LIST_SIZE_NUM];
extern const unsigned int gPccShvc_scalingListSizeX[pcc_shvc::SCALING_LIST_SIZE_NUM];

extern const char *gPccShvc_MatrixType[pcc_shvc::SCALING_LIST_SIZE_NUM][pcc_shvc::SCALING_LIST_NUM];
extern const char *gPccShvc_MatrixType_DC[pcc_shvc::SCALING_LIST_SIZE_NUM][pcc_shvc::SCALING_LIST_NUM];

extern const int gPccShvc_quantTSDefault4x4[4*4];
extern const int gPccShvc_quantIntraDefault8x8[8*8];
extern const int gPccShvc_quantInterDefault8x8[8*8];

namespace pcc_shvc{

// ====================================================================================================================
// Initialize / destroy functions
// ====================================================================================================================

Void         initROMPccShvc();
Void         destroyROMPccShvc();

// ====================================================================================================================
// Data structure related table & variable
// ====================================================================================================================


Void         initZscanToRasterPccShvc ( Int iMaxDepth, Int iDepth, UInt uiStartVal, UInt*& rpuiCurrIdx );
Void         initRasterToZscanPccShvc ( UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxDepth         );


Void         initRasterToPelXYPccShvc ( UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxDepth );

extern const UInt gPccShvc_auiPUOffset[NUMBER_OF_PART_SIZES];

extern const Int gPccShvc_quantScales[SCALING_LIST_REM_NUM];             // Q(QP%6)
extern const Int gPccShvc_invQuantScales[SCALING_LIST_REM_NUM];          // IQ(QP%6)

#if RExt__HIGH_PRECISION_FORWARD_TRANSFORM
static const Int gPccShvc_transformMatrixShift[TRANSFORM_NUMBER_OF_DIRECTIONS] = { 14, 6 };
#else
static const Int gPccShvc_transformMatrixShift[TRANSFORM_NUMBER_OF_DIRECTIONS] = {  6, 6 };
#endif

extern const TMatrixCoeff gPccShvc_aiT4 [TRANSFORM_NUMBER_OF_DIRECTIONS][4][4];
extern const TMatrixCoeff gPccShvc_aiT8 [TRANSFORM_NUMBER_OF_DIRECTIONS][8][8];
extern const TMatrixCoeff gPccShvc_aiT16[TRANSFORM_NUMBER_OF_DIRECTIONS][16][16];
extern const TMatrixCoeff gPccShvc_aiT32[TRANSFORM_NUMBER_OF_DIRECTIONS][32][32];

// ====================================================================================================================
// Luma QP to Chroma QP mapping
// ====================================================================================================================

static const Int chromaQPMappingTableSize = 58;

extern const UChar  gPccShvc_aucChromaScale[NUM_CHROMA_FORMAT][chromaQPMappingTableSize];


// ====================================================================================================================
// Scanning order & context mapping table
// ====================================================================================================================

extern const UInt   ctxIndMap4x4[4*4];

extern const UInt   gPccShvc_uiGroupIdx[ MAX_TU_SIZE ];
extern const UInt   gPccShvc_uiMinInGroup[ LAST_SIGNIFICANT_GROUPS ];

// ====================================================================================================================
// Intra prediction table
// ====================================================================================================================

extern const UChar  gPccShvc_aucIntraModeNumFast_UseMPM[MAX_CU_DEPTH];
extern const UChar  gPccShvc_aucIntraModeNumFast_NotUseMPM[MAX_CU_DEPTH];

extern const UChar  gPccShvc_chroma422IntraAngleMappingTable[NUM_INTRA_MODE];

// ====================================================================================================================
// Mode-Dependent DST Matrices
// ====================================================================================================================

extern const TMatrixCoeff gPccShvc_as_DST_MAT_4 [TRANSFORM_NUMBER_OF_DIRECTIONS][4][4];

// ====================================================================================================================
// Misc.
// ====================================================================================================================


#if ENC_DEC_TRACE
extern FILE*  gPccShvc_hTrace;
extern Bool   gPccShvc_bJustDoIt;
extern const Bool gPccShvc_bEncDecTraceEnable;
extern const Bool gPccShvc_bEncDecTraceDisable;
extern Bool   gPccShvc_HLSTraceEnable;
extern UInt64 gPccShvc_nSymbolCounter;

#define COUNTER_START    1
#define COUNTER_END      0 //( UInt64(1) << 63 )

#define DTRACE_CABAC_F(x)     if ( ( gPccShvc_nSymbolCounter >= COUNTER_START && gPccShvc_nSymbolCounter <= COUNTER_END )|| gPccShvc_bJustDoIt ) fprintf( gPccShvc_hTrace, "%f", x );
#define DTRACE_CABAC_V(x)     if ( ( gPccShvc_nSymbolCounter >= COUNTER_START && gPccShvc_nSymbolCounter <= COUNTER_END )|| gPccShvc_bJustDoIt ) fprintf( gPccShvc_hTrace, "%d", x );
#define DTRACE_CABAC_VL(x)    if ( ( gPccShvc_nSymbolCounter >= COUNTER_START && gPccShvc_nSymbolCounter <= COUNTER_END )|| gPccShvc_bJustDoIt ) fprintf( gPccShvc_hTrace, "%lld", x );
#define DTRACE_CABAC_T(x)     if ( ( gPccShvc_nSymbolCounter >= COUNTER_START && gPccShvc_nSymbolCounter <= COUNTER_END )|| gPccShvc_bJustDoIt ) fprintf( gPccShvc_hTrace, "%s", x );
#define DTRACE_CABAC_X(x)     if ( ( gPccShvc_nSymbolCounter >= COUNTER_START && gPccShvc_nSymbolCounter <= COUNTER_END )|| gPccShvc_bJustDoIt ) fprintf( gPccShvc_hTrace, "%x", x );
#define DTRACE_CABAC_R( x,y ) if ( ( gPccShvc_nSymbolCounter >= COUNTER_START && gPccShvc_nSymbolCounter <= COUNTER_END )|| gPccShvc_bJustDoIt ) fprintf( gPccShvc_hTrace, x,    y );
#define DTRACE_CABAC_N        if ( ( gPccShvc_nSymbolCounter >= COUNTER_START && gPccShvc_nSymbolCounter <= COUNTER_END )|| gPccShvc_bJustDoIt ) fprintf( gPccShvc_hTrace, "\n"    );

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


#if LAYER_CTB
extern       UInt gPccShvc_auiLayerZscanToRaster[MAX_LAYERS][ MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH ];
extern       UInt gPccShvc_auiLayerRasterToZscan[MAX_LAYERS][ MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH ];
extern       UInt gPccShvc_auiLayerRasterToPelX[MAX_LAYERS][ MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH ];
extern       UInt gPccShvc_auiLayerRasterToPelY[MAX_LAYERS][ MAX_NUM_PART_IDXS_IN_CTU_WIDTH*MAX_NUM_PART_IDXS_IN_CTU_WIDTH ];
#endif

#if SVC_EXTENSION
Void         initRasterToZscan( UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxDepth, UInt* inZscanToRaster, UInt* outRasterToZscan );
Void         initRasterToPelXY( UInt uiMaxCUWidth, UInt uiMaxCUHeight, UInt uiMaxDepth, UInt* outRasterToPelX, UInt* outRasterToPelY );
#endif

//! \}

}; // namespace pcc_shvc

#endif  //__PCC_SHVC_TCOMROM__

