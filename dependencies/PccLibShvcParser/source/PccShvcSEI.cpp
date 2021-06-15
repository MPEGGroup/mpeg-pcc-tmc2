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

/** \file     SEI.cpp
    \brief    helper functions for SEI handling
*/

#include "PccShvcCommonDef.h"
#include "PccShvcSEI.h"

using namespace pcc_shvc;

SEIMessages getSeisByType(SEIMessages &seiList, SEI::PayloadType seiType)
{
  SEIMessages result;

  for (SEIMessages::iterator it=seiList.begin(); it!=seiList.end(); it++)
  {
    if ((*it)->payloadType() == seiType)
    {
      result.push_back(*it);
    }
  }
  return result;
}

SEIMessages extractSeisByType(SEIMessages &seiList, SEI::PayloadType seiType)
{
  SEIMessages result;

  SEIMessages::iterator it=seiList.begin();
  while ( it!=seiList.end() )
  {
    if ((*it)->payloadType() == seiType)
    {
      result.push_back(*it);
      it = seiList.erase(it);
    }
    else
    {
      it++;
    }
  }
  return result;
}


Void deleteSEIs (SEIMessages &seiList)
{
  for (SEIMessages::iterator it=seiList.begin(); it!=seiList.end(); it++)
  {
    delete (*it);
  }
  seiList.clear();
}

void SEIBufferingPeriod::copyTo (SEIBufferingPeriod& target)
{
  target.m_bpSeqParameterSetId = m_bpSeqParameterSetId;
  target.m_rapCpbParamsPresentFlag = m_rapCpbParamsPresentFlag;
  target.m_cpbDelayOffset = m_cpbDelayOffset;
  target.m_dpbDelayOffset = m_dpbDelayOffset;
  target.m_concatenationFlag = m_concatenationFlag;
  target.m_auCpbRemovalDelayDelta = m_auCpbRemovalDelayDelta;
  ::memcpy(target.m_initialCpbRemovalDelay, m_initialCpbRemovalDelay, sizeof(m_initialCpbRemovalDelay));
  ::memcpy(target.m_initialCpbRemovalDelayOffset, m_initialCpbRemovalDelayOffset, sizeof(m_initialCpbRemovalDelayOffset));
  ::memcpy(target.m_initialAltCpbRemovalDelay, m_initialAltCpbRemovalDelay, sizeof(m_initialAltCpbRemovalDelay));
  ::memcpy(target.m_initialAltCpbRemovalDelayOffset, m_initialAltCpbRemovalDelayOffset, sizeof(m_initialAltCpbRemovalDelayOffset));
}

void SEIPictureTiming::copyTo (SEIPictureTiming& target)
{
  target.m_picStruct = m_picStruct;
  target.m_sourceScanType = m_sourceScanType;
  target.m_duplicateFlag = m_duplicateFlag;

  target.m_auCpbRemovalDelay = m_auCpbRemovalDelay;
  target.m_picDpbOutputDelay = m_picDpbOutputDelay;
  target.m_picDpbOutputDuDelay = m_picDpbOutputDuDelay;
  target.m_numDecodingUnitsMinus1 = m_numDecodingUnitsMinus1;
  target.m_duCommonCpbRemovalDelayFlag = m_duCommonCpbRemovalDelayFlag;
  target.m_duCommonCpbRemovalDelayMinus1 = m_duCommonCpbRemovalDelayMinus1;

  target.m_numNalusInDuMinus1 = m_numNalusInDuMinus1;
  target.m_duCpbRemovalDelayMinus1 = m_duCpbRemovalDelayMinus1;
}

// Static member
const TChar *SEI::getSEIMessageString(SEI::PayloadType payloadType)
{
  switch (payloadType)
  {
    case SEI::BUFFERING_PERIOD:                     return "Buffering period";
    case SEI::PICTURE_TIMING:                       return "Picture timing";
    case SEI::PAN_SCAN_RECT:                        return "Pan-scan rectangle";                   // not currently decoded
    case SEI::FILLER_PAYLOAD:                       return "Filler payload";                       // not currently decoded
    case SEI::USER_DATA_REGISTERED_ITU_T_T35:       return "User data registered";                 // not currently decoded
    case SEI::USER_DATA_UNREGISTERED:               return "User data unregistered";
    case SEI::RECOVERY_POINT:                       return "Recovery point";
    case SEI::SCENE_INFO:                           return "Scene information";                    // not currently decoded
    case SEI::FULL_FRAME_SNAPSHOT:                  return "Picture snapshot";                     // not currently decoded
    case SEI::PROGRESSIVE_REFINEMENT_SEGMENT_START: return "Progressive refinement segment start"; // not currently decoded
    case SEI::PROGRESSIVE_REFINEMENT_SEGMENT_END:   return "Progressive refinement segment end";   // not currently decoded
    case SEI::FILM_GRAIN_CHARACTERISTICS:           return "Film grain characteristics";           // not currently decoded
    case SEI::POST_FILTER_HINT:                     return "Post filter hint";                     // not currently decoded
    case SEI::TONE_MAPPING_INFO:                    return "Tone mapping information";
    case SEI::KNEE_FUNCTION_INFO:                   return "Knee function information";
    case SEI::FRAME_PACKING:                        return "Frame packing arrangement";
    case SEI::DISPLAY_ORIENTATION:                  return "Display orientation";
    case SEI::GREEN_METADATA:                       return "Green metadata information";
    case SEI::SOP_DESCRIPTION:                      return "Structure of pictures information";
    case SEI::ACTIVE_PARAMETER_SETS:                return "Active parameter sets";
    case SEI::DECODING_UNIT_INFO:                   return "Decoding unit information";
    case SEI::TEMPORAL_LEVEL0_INDEX:                return "Temporal sub-layer zero index";
    case SEI::DECODED_PICTURE_HASH:                 return "Decoded picture hash";
    case SEI::SCALABLE_NESTING:                     return "Scalable nesting";
    case SEI::REGION_REFRESH_INFO:                  return "Region refresh information";
    case SEI::NO_DISPLAY:                           return "No display";
    case SEI::TIME_CODE:                            return "Time code";
    case SEI::MASTERING_DISPLAY_COLOUR_VOLUME:      return "Mastering display colour volume";
    case SEI::SEGM_RECT_FRAME_PACKING:              return "Segmented rectangular frame packing arrangement";
    case SEI::TEMP_MOTION_CONSTRAINED_TILE_SETS:    return "Temporal motion constrained tile sets";
    case SEI::CHROMA_RESAMPLING_FILTER_HINT:        return "Chroma sampling filter hint";
    case SEI::COLOUR_REMAPPING_INFO:                return "Colour remapping info";
#if U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI
    case SEI::ALTERNATIVE_TRANSFER_CHARACTERISTICS: return "Alternative transfer characteristics";
#endif
#if SVC_EXTENSION
#if LAYERS_NOT_PRESENT_SEI
    case SEI::LAYERS_NOT_PRESENT:                   return "Layers Present";
#endif
#if N0383_IL_CONSTRAINED_TILE_SETS_SEI
    case SEI::INTER_LAYER_CONSTRAINED_TILE_SETS:    return "Inter Layer Constrained Tile Sets";
#endif
#if SUB_BITSTREAM_PROPERTY_SEI
    case SEI::SUB_BITSTREAM_PROPERTY:               return "Sub-bitstream property";
#endif
#if O0164_MULTI_LAYER_HRD
    case SEI::BSP_NESTING:                          return "Bitstream parition nesting";
    case SEI::BSP_INITIAL_ARRIVAL_TIME:             return "Bitstream parition initial arrival time";
#endif
#if Q0096_OVERLAY_SEI
    case SEI::OVERLAY_INFO:                         return "Overlay Information";
#endif
#if P0123_ALPHA_CHANNEL_SEI
    case SEI::ALPHA_CHANNEL_INFO:                   return "Alpha Channel Information";
#endif
#if Q0189_TMVP_CONSTRAINTS
    case SEI::TMVP_CONSTRAINTS:                     return "TMVP constraints";
#endif
#if Q0247_FRAME_FIELD_INFO
    case SEI::FRAME_FIELD_INFO:                     return "Frame field info";
#endif
#endif
    default:                                        return "Unknown";
  }
}

#if SUB_BITSTREAM_PROPERTY_SEI
SEISubBitstreamProperty::SEISubBitstreamProperty()
{
  m_activeVpsId             = -1;
  m_numAdditionalSubStreams = 0;
  ::memset(m_subBitstreamMode       , 0, sizeof(m_subBitstreamMode));
  ::memset(m_outputLayerSetIdxToVps , 0, sizeof(m_outputLayerSetIdxToVps));
  ::memset(m_highestSublayerId      , 0, sizeof(m_highestSublayerId));
  ::memset(m_avgBitRate             , 0, sizeof(m_avgBitRate));
  ::memset(m_maxBitRate             , 0, sizeof(m_maxBitRate));
}
#endif