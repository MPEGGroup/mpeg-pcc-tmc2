/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2026, ISO/IEC
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

#include "PCCGSHlsBuilder.h"
#include "PCCGSEncoderConstant.h"
#include "CameraModelInfo.h"
#include <cmath>
#include <iostream>

using namespace pcc;

namespace {
constexpr size_t ATLAS_COUNT                = 1;
constexpr size_t ATLAS_INDEX                = 0;
constexpr size_t MAP_COUNT_MINUS1           = 0;
constexpr size_t TIER_FLAG                  = 0;
constexpr size_t PROFILE_CODEC_GROUP_IDC    = 2;
constexpr size_t PROFILE_TOOLSET_IDC        = 2;
constexpr size_t PROFILE_RECONSTRUCTION_IDC = 3;
constexpr size_t LEVEL_IDC                  = 180;
}  // namespace

void PCCGSHlsBuilder::initialize( PCCHighLevelSyntax            &hls,
                                  PCCBitstreamStat              &bitstreamStat,
                                  const PCCGSEncoderParameters  &params,
                                  const PCCAdaptationParameters &adaptationParams ) {
  hls.setBitstreamStat( bitstreamStat );
  hls.setActiveVpsId( vpsParameterSetId_ );
  initializeV3CHls( hls, params );
  initializeAtlasHls( hls, params, adaptationParams );
  initialized_ = true;
}

void PCCGSHlsBuilder::initializeV3CHls( PCCHighLevelSyntax &hls, const PCCGSEncoderParameters &params ) {
  hls.addV3CParameterSet();
  auto &vps = hls.getVpsList().back();
  vps.setV3CParameterSetId( vpsParameterSetId_ );
  vps.setAtlasCountMinus1( ATLAS_COUNT - 1 );
  vps.allocateAtlas();

  vps.setMapCountMinus1( ATLAS_INDEX, MAP_COUNT_MINUS1 );
  vps.setMultipleMapStreamsPresentFlag( ATLAS_INDEX, false );
  vps.setAuxiliaryVideoPresentFlag( ATLAS_INDEX, false );
  vps.setOccupancyVideoPresentFlag( ATLAS_INDEX, false );
  vps.setGeometryVideoPresentFlag( ATLAS_INDEX, true );
  vps.setAttributeVideoPresentFlag( ATLAS_INDEX, true );

  for ( size_t i = 0; i < MAP_COUNT_MINUS1 + 1; i++ ) {
    if ( i == 0 ) {
      vps.setMapAbsoluteCodingEnableFlag( ATLAS_INDEX, i, true );
      vps.setMapPredictorIndexDiff( ATLAS_INDEX, i, false );
    } else {
      vps.setMapAbsoluteCodingEnableFlag( ATLAS_INDEX, i, false );
      vps.setMapPredictorIndexDiff( ATLAS_INDEX, i, false );
    }
  }

  // V3C profile/tier/level related parameters
  auto &ptl = vps.getProfileTierLevel();
  ptl.setTierFlag( TIER_FLAG );
  ptl.setProfileCodecGroupIdc( PROFILE_CODEC_GROUP_IDC );
  ptl.setProfileToolsetIdc( PROFILE_TOOLSET_IDC );
  ptl.setProfileReconstructionIdc( PROFILE_RECONSTRUCTION_IDC );
  ptl.setLevelIdc( LEVEL_IDC );
  ptl.setToolConstraintsPresentFlag( false );

  // geometry information
  auto &gi = vps.getGeometryInformation( ATLAS_INDEX );
  gi.setGeometry3dCoordinatesBitdepthMinus1( params.geometryBitDepth3D_ - 1 );
  gi.setGeometry2dBitdepthMinus1( uint8_t( params.geometryBitDepth2D_ - 1 ) );
  gi.setGeometryMSBAlignFlag( false );
  gi.setGeometryCodecId( 0 );

  // Attribute information
  auto                &ai               = vps.getAttributeInformation( ATLAS_INDEX );
  std::vector<uint8_t> attributeTypeIds = { ATTR_TYPE_OPACITY, ATTR_TYPE_SPHERICAL_HARMONIC, ATTR_TYPE_SCALE,
                                            ATTR_TYPE_ROTATION };

  ai.setAttributeCount( 4 );
  ai.allocate();
  for ( size_t i = 0; i < ai.getAttributeCount(); i++ ) {
    ai.setAttributeTypeId( i, attributeTypeIds[i] );
    if ( attributeTypeIds[i] == ATTR_TYPE_COLOR ) {
      ai.setAttributeDimensionMinus1( i, 2 );
    } else if ( attributeTypeIds[i] == ATTR_TYPE_OPACITY ) {
      ai.setAttributeDimensionMinus1( i, 0 );
    } else if ( attributeTypeIds[i] == ATTR_TYPE_SPHERICAL_HARMONIC ) {
      ai.setAttributeDimensionMinus1( i, 47 );
      ai.setAttributeDimensionPartitionsMinus1( i, 15 );
      for ( int j = 0; j < 16; j++ ) { ai.setAttributePartitionChannelsMinus1( i, j, 2 ); }
    } else if ( attributeTypeIds[i] == ATTR_TYPE_SCALE ) {
      ai.setAttributeDimensionMinus1( i, 2 );
      ai.setAttributeDimensionPartitionsMinus1( i, 0 );
      ai.setAttributePartitionChannelsMinus1( i, 0, 2 );
    } else if ( attributeTypeIds[i] == ATTR_TYPE_ROTATION ) {
      ai.setAttributeDimensionMinus1( i, 3 );
      ai.setAttributeDimensionPartitionsMinus1( i, 1 );
      ai.setAttributePartitionChannelsMinus1( i, 0, 2 );
      ai.setAttributePartitionChannelsMinus1( i, 1, 0 );
    }

    ai.setAttribute2dBitdepthMinus1( i, params.attributeBitDepth2D_ - 1 );
    ai.setAttributeMapAbsoluteCodingPersistenceFlag( i, false );
    ai.setAttributeMSBAlignFlag( i, false );
    ai.setAttributeCodecId( i, 0 );
  }
}

void PCCGSHlsBuilder::initializeAtlasHls( PCCHighLevelSyntax            &hls,
                                          const PCCGSEncoderParameters  &params,
                                          const PCCAdaptationParameters &adaptationParams ) {
  hls.allocateAtlasHLS( ATLAS_COUNT );
  hls.setAtlasIndex( 0 );
  hls.getAtlasHighLevelSyntax().setMaxNumRefAtlasFrame( 1 );

  initializeAtlasAspsHls( hls, params );
  initializeAtlasAfpsHls( hls, params );
  initializeAtlasAapsHls( hls, adaptationParams );
}

void PCCGSHlsBuilder::initializeAtlasAspsHls( PCCHighLevelSyntax &hls, const PCCGSEncoderParameters &params ) {
  auto &asps = hls.addAtlasSequenceParameterSet( aspsParameterSetId_);
  hls.getAtlasHighLevelSyntax(ATLAS_INDEX).setActiveASPS( aspsParameterSetId_ );
  asps.setLog2PatchPackingBlockSize( std::log2( params.occupancyResolution_ ) );
  asps.setLog2MaxAtlasFrameOrderCntLsbMinus4( 6 );
  asps.setMaxDecAtlasFrameBufferingMinus1( 0 );
  asps.setNumRefAtlasFrameListsInAsps( 1 );
  asps.addRefListStruct();
  asps.setMapCountMinus1( 0 );
  asps.setLongTermRefAtlasFramesFlag( false );
  asps.setUseEightOrientationsFlag( false );
  asps.setExtendedProjectionEnabledFlag( false );
  asps.setMaxNumberProjectionsMinus1( 5 );
  asps.setNormalAxisLimitsQuantizationEnabledFlag( false );
  asps.setNormalAxisMaxDeltaValueEnabledFlag( false );
  asps.setPixelDeinterleavingFlag( false );
  if ( asps.getPixelDeinterleavingFlag() ) {
    asps.allocatePixelDeinterleavingMapFlag();
    for ( size_t i = 0; i <= asps.getMapCountMinus1(); i++ ) asps.setPixelDeinterleavingMapFlag( i, false );
  }
  asps.setPatchPrecedenceOrderFlag( false );
  asps.setPatchSizeQuantizerPresentFlag( false );
  asps.setEomPatchEnabledFlag( false );
  asps.setPLREnabledFlag( false );
  asps.setVuiParametersPresentFlag( false );
  asps.setExtensionFlag( true );
  asps.setVpccExtensionFlag( true );
  asps.setExtension7Bits( 0 );
  asps.setAuxiliaryVideoEnabledFlag( 0 );
  asps.setRawPatchEnabledFlag( 1 );
  if ( asps.getVpccExtensionFlag() ) {
    auto &ext = asps.getAspsVpccExtension();
    ext.setRemoveDuplicatePointEnableFlag( false );
    if ( asps.getPixelDeinterleavingFlag() || asps.getPLREnabledFlag() ) { ext.setSurfaceThicknessMinus1( 0 ); }
  }
  asps.setEomFixBitCountMinus1( 0 );
  asps.setGeometry2dBitdepthMinus1( uint8_t( params.geometryBitDepth2D_ - 1 ) );
  asps.setGeometry3dBitdepthMinus1( params.geometryBitDepth3D_ - 1 );
}

void PCCGSHlsBuilder::initializeAtlasAfpsHls( PCCHighLevelSyntax &hls, const PCCGSEncoderParameters &params ) {
  auto &afps = hls.addAtlasFrameParameterSet( afpsParameterSetId_ );
  hls.getAtlasHighLevelSyntax(ATLAS_INDEX).setActiveAFPS( afpsParameterSetId_ );
  auto &asps = hls.getAtlasSequenceParameterSet( aspsParameterSetId_ );
  afps.setAtlasSequenceParameterSetId( asps.getAtlasSequenceParameterSetId() );
  afps.setNumRefIdxDefaultActiveMinus1( 0 );
  afps.setAdditionalLtAfocLsbLen( asps.getLongTermRefAtlasFramesFlag() ? 4 : 0 );
  afps.setRaw3dOffsetBitCountExplicitModeFlag( params.raw3dOffsetBitCountExplicitModeFlag_ );
  afps.setExtensionFlag( false );
  afps.setExtension8Bits( 0 );
  AtlasFrameTileInformationRbsp afti;
  afti.setSingleTileInAtlasFrameFlag( true );
  afps.setAtlasFrameTileInformationRbsp( afti );
}

void PCCGSHlsBuilder::initializeAtlasAapsHls( PCCHighLevelSyntax            &hls,
                                              const PCCAdaptationParameters &adaptationParams ) {
  auto &aaps = hls.addAtlasAdaptationParameterSet( aapsParameterSetId_ );
  hls.getAtlasHighLevelSyntax(ATLAS_INDEX).setActiveAAPS( aapsParameterSetId_ );
  aaps.setExtensionFlag( true );
  aaps.setVpccExtensionFlag( false );
  aaps.setVpccGsExtensionFlag( true );
  aaps.setExtension6Bits( 0 );

  auto &ext = aaps.getAapsVpccGsExtension();
  ext.setDequantizationPresentFlag( true );
  ext.setShPropertyPresentFlag( true );
  auto &dequant = ext.getGsDequantization();
  // Position dequantization parameters
  dequant.setDequantizationType( 1 );
  if ( adaptationParams.enablePositionQuantization_ ) {
    for ( size_t i = 0; i < 3; i++ ) {
      dequant.setMinPosition( i, adaptationParams.computedPositionMin_[i] );
      dequant.setMaxPosition( i, adaptationParams.computedPositionMax_[i] );
    }
  }

  if ( adaptationParams.enableOpacityQuantization_ ) {
    dequant.setOpacityPresentFlag( true );
    dequant.setMinOpacity( adaptationParams.computedOpacityMin_ );
    dequant.setMaxOpacity( adaptationParams.computedOpacityMax_ );
  }

  if ( adaptationParams.enableShQuantization_ ) {
    dequant.setShPresentFlag( true );
    dequant.setShNumLvl( adaptationParams.shCoefficientCount_ );
    int tpOffset = 0;
    int ssOffset = 0;
    for ( size_t i = 0; i < adaptationParams.shCoefficientCount_; i++ ) {
      dequant.setShNumChan( i, 3 );
      for ( size_t j = 0; j < 3; j++ ) {
        dequant.setMinSh( i, j, adaptationParams.computedShMin_[i][j] );
        dequant.setMaxSh( i, j, adaptationParams.computedShMax_[i][j] );

        if ( adaptationParams.enableShNonUniformQuantization_ ) {
          int idx = i * 3 + j;
          int M   = adaptationParams.shNumRegionsMinus1_[idx];
          dequant.setNumQuantRegionsMinus1( i, j, M );
          dequant.setNumBitsForTransitionPoint( i, j, adaptationParams.shNumBitsForTransitionPoint_[idx] );
          for ( int m = 0; m < M; m++ ) {
            dequant.setQuantRegionTransitionPoint( i, j, m, adaptationParams.shTransitionPoints_[tpOffset + m] );
          }
          for ( int m = 0; m < M + 1; m++ ) {
            dequant.setRegionStepSize( i, j, m, adaptationParams.shStepSizes_[ssOffset + m] );
          }
          tpOffset += M;
          ssOffset += M + 1;
        }
      }
    }
  }

  if ( adaptationParams.enableScaleQuantization_ ) {
    dequant.setScalePresentFlag( true );
    for ( size_t i = 0; i < 3; i++ ) {
      dequant.setMinScale( i, adaptationParams.computedScaleMin_[i] );
      dequant.setMaxScale( i, adaptationParams.computedScaleMax_[i] );
    }
  }

  if ( adaptationParams.enableRotationQuantization_ ) {
    dequant.setRotationPresentFlag( true );
    for ( size_t i = 0; i < 4; i++ ) {
      dequant.setMinRotation( i, adaptationParams.computedRotationMin_[i] );
      dequant.setMaxRotation( i, adaptationParams.computedRotationMax_[i] );
    }
  }

  auto &shProp = ext.getGsShProperty();
  shProp.setShMappingMethod( adaptationParams.shMappingMode_ );

  if ( adaptationParams.gsdTransform2dPresentFlag_ ) {
    ext.setTransform2DPresentFlag( true );
    auto &tr2d = ext.getGs2DTransform();
    tr2d.setTrRowNumGroups( adaptationParams.shTrGrpRow_.size() );
    int ki = 0;
    for ( size_t i = 0; i < adaptationParams.shTrGrpRow_.size(); ++i ) {
      tr2d.setRowTrSizeMinus1( i, adaptationParams.shTrGrpRow_[i].size() - 1 );
      for ( size_t j = 0; j < adaptationParams.shTrGrpRow_[i].size(); ++j ) {
        tr2d.setTrRowGroupIdx( i, j, adaptationParams.shTrGrpRow_[i][j] );
      }
      tr2d.setKltFlag( i, adaptationParams.shKltFlag_[i] );
      if ( adaptationParams.shKltFlag_[i] == 1 ) {
        const auto &klt  = adaptationParams.shKltParams_[ki++];
        int         size = adaptationParams.shTrGrpRow_[i].size();
        for ( int m = 0; m < 3 * size; m++ ) {
          for ( int n = 0; n < 3 * size; n++ ) { tr2d.setKltCoeff( i, m, n, klt.eigvec[m][n] ); }
          tr2d.setKltStdDev( i, m, klt.sigma[m] );
          tr2d.setKltMean( i, m, klt.mean[m] );
        }
      } else {
        tr2d.setTrRowType( i, adaptationParams.shTrTypeRow_[i] );
      }
    }
    for ( size_t i = 0; i < 16; ++i ) { tr2d.setTrColType( i, adaptationParams.shTrTypeCol_[i] ); }
  }
  return;
}

void PCCGSHlsBuilder::replaceVpsHls( PCCHighLevelSyntax &hls, const PCCGSEncoderParameters           &params ){
  hls.getVpsList().clear();
  vpsParameterSetId_ = (vpsParameterSetId_ + 1)%16;
  hls.setActiveVpsId( vpsParameterSetId_ );
  initializeV3CHls( hls, params );
}

void PCCGSHlsBuilder::replaceAtlasAspsHls( PCCHighLevelSyntax &hls, const PCCGSEncoderParameters           &params ){
  hls.getAtlasSequenceParameterSetList().clear();
  aspsParameterSetId_ = (aspsParameterSetId_ + 1)%64;
  initializeAtlasAspsHls( hls, params );
}

void PCCGSHlsBuilder::replaceAtlasAfpsHls( PCCHighLevelSyntax &hls, const PCCGSEncoderParameters           &params ){
  hls.getAtlasFrameParameterSetList().clear();
  afpsParameterSetId_ = (afpsParameterSetId_ + 1)%64;
  initializeAtlasAfpsHls( hls, params );
}

void PCCGSHlsBuilder::replaceAtlasAapsHls( PCCHighLevelSyntax &hls, const PCCAdaptationParameters &adaptationParams ){
  hls.getAtlasAdaptationParameterSetList().clear();
  aapsParameterSetId_ = (aapsParameterSetId_ + 1)%64;
  initializeAtlasAapsHls( hls, adaptationParams );
}

void PCCGSHlsBuilder::removeAtlasFrames( PCCHighLevelSyntax &hls ) {
  hls.getAtlasTileLayerList().clear();
}

void PCCGSHlsBuilder::addAtlasFrame( PCCHighLevelSyntax                     &hls,
                                     std::vector<GSEncoderPatchInformation> &patches,
                                     const PCCGSEncoderParameters           &params ) {
  int   tileIndex  = 0;
  int   tileId     = 0;
  auto &afps       = hls.getAtlasFrameParameterSet( afpsParameterSetId_ );
  auto &asps       = hls.getAtlasSequenceParameterSet( afps.getAtlasSequenceParameterSetId() );
  auto &af         = hls.addAtlasTileLayer( frameIndex_, tileIndex );
  auto &ath        = af.getHeader();

  ath.setAtlasFrameParameterSetId( afps.getAtlasFrameParameterSetId() );
  ath.setAtlasAdaptationParameterSetId( aapsParameterSetId_ );
  ath.setId( tileId );
  ath.setType( PCCTileType::I_TILE );
  ath.setAtlasOutputFlag( true );
  ath.setAtlasFrmOrderCntLsb( frameIndex_ );
  ath.setPosMinDQuantizer( 0 );
  ath.setPosDeltaMaxDQuantizer( 0 );
  ath.setPatchSizeXinfoQuantizer( 0 );
  ath.setPatchSizeYinfoQuantizer( 0 );
  if ( afps.getRaw3dOffsetBitCountExplicitModeFlag() ) {
    ath.setRaw3dOffsetAxisBitCountMinus1( params.raw3dOffsetAxisBitCountMinus1_ );
  } else {
    ath.setRaw3dOffsetAxisBitCountMinus1( params.geometryBitDepth3D_ - 1 - ( params.geometryBitDepth2D_ - 1 ) - 1 );
  }
  ath.setNumRefIdxActiveOverrideFlag( false );
  ath.setRefAtlasFrameListSpsFlag( true );
  ath.setRefAtlasFrameListIdx( 0 );

  auto &atd = af.getDataUnit();
  atd.setTileOrder( tileId );
  auto &atdu = af.getDataUnit();

  // eq. 25 and 26 in ISO/IEC 23090-5 4th edition
  size_t RawShiftBits = asps.getGeometry2dBitdepthMinus1() + 1;
  if ( afps.getRaw3dOffsetBitCountExplicitModeFlag() ) {
    RawShiftBits = asps.getGeometry3dBitdepthMinus1() - ath.getRaw3dOffsetAxisBitCountMinus1();
  }

  auto RawShift = std::pow( 2, RawShiftBits );

  for ( auto &patchInfo : patches ) {
    auto &pid = atdu.addPatchInformationData( 1 );
    pid.setTileOrder( tileId );
    pid.setPatchIndex( atdu.getPatchInformationData().size() - 1 );
    auto &rpdu = pid.getRawPatchDataUnit();
    rpdu.setPatchIndex( atdu.getPatchInformationData().size() - 1 );
    rpdu.setFrameIndex( frameIndex_ );
    rpdu.setTileOrder( tileId );
    rpdu.setPatchInAuxiliaryVideoFlag( false );
    rpdu.set2dPosX( patchInfo.patch.u0_ );
    rpdu.set2dPosY( patchInfo.patch.v0_ );
    rpdu.set2dSizeXMinus1( patchInfo.patch.sizeU_ - 1 );
    rpdu.set2dSizeYMinus1( patchInfo.patch.sizeV_ - 1 );
    rpdu.set3dOffsetU( patchInfo.patch.u1_ / RawShift );
    rpdu.set3dOffsetV( patchInfo.patch.v1_ / RawShift );
    rpdu.set3dOffsetD( patchInfo.patch.d1_ / RawShift );
    rpdu.setRawPointsMinus1( patchInfo.patch.getNumberOfRawPoints() - 1 );
  }

  // end patch
  atdu.addPatchInformationData( 14 );

  // create SEI message
  if ( !params.gsColmapFolderPath_.empty() ) { createGsCameraModelInfoSei( af, params.gsColmapFolderPath_ ); }

}

void PCCGSHlsBuilder::updateVps( PCCHighLevelSyntax           &hls,
                                 size_t                        mapWidth,
                                 size_t                        mapHeight,
                                 const PCCGSEncoderParameters &params ) {
  auto &vps = hls.getVpsList().front();
  vps.setFrameWidth( ATLAS_INDEX, mapWidth );
  vps.setFrameHeight( ATLAS_INDEX, mapHeight );

  if ( params.packGeometry_ == 1 ) { vps.setGeometryVideoPresentFlag( ATLAS_INDEX, false ); }
  if ( params.packAttributes_ == 1 ) { vps.setAttributeVideoPresentFlag( ATLAS_INDEX, false ); }

  if ( params.packGeometry_ || params.packAttributes_ ) {
    vps.setExtensionPresentFlag( true );
    vps.setExtensionCount( 1 );
    vps.setExtensionType( 0, VPS_EXT_PACKED_2 );
    auto &ai = vps.getAttributeInformation( ATLAS_INDEX );

    // Calculate the number of packed regions per packed map
    auto       packedRegionsPerMap = std::vector<uint8_t>{};
    const auto countRegion = [&packedRegionsPerMap]( uint8_t packedMapIndex, uint8_t dimensionPartitionsCount ) {
      while ( packedMapIndex >= packedRegionsPerMap.size() ) { packedRegionsPerMap.push_back( 0 ); }
      packedRegionsPerMap[packedMapIndex] += dimensionPartitionsCount;
    };
    if ( params.packGeometry_ ) { countRegion( params.geometryPackedMapIndex_, 1 ); }
    if ( params.packAttributes_ ) {
      for ( uint8_t attrIdx = 0; attrIdx < ai.getAttributeCount(); ++attrIdx ) {
        countRegion( params.getPackedMapIndexByAttributeTypeId( ai.getAttributeTypeId( attrIdx ) ),
                     ai.getAttributeDimensionPartitionsMinus1( attrIdx ) + 1 );
      }
    }

    auto &pin = vps.getVpsPackedVideoExtension().getPackingInformation( ATLAS_INDEX );
    pin.setPackedMapCountMinus1( static_cast<uint8_t>( packedRegionsPerMap.size() - 1 ) );

    if ( params.packGeometry_ ) {
      pin.setGeometryPresentFlag( true );
      auto &region = pin.addRegion();
      region.setTypeIdMinus2( V3C_GVD - 2 );
      region.setPackedMapIndex( params.geometryPackedMapIndex_ );
      std::cout << "region: typeIdMinus2=V3C_GVD - 2, packedMapIndex=" << int( region.getPackedMapIndex() ) << "\n";
    }

    if ( params.packAttributes_ ) {
      pin.setAttributePresentFlag( true );

      for ( uint8_t attrIdx = 0; attrIdx < ai.getAttributeCount(); ++attrIdx ) {
        for ( uint8_t partIdx = 0; partIdx <= ai.getAttributeDimensionPartitionsMinus1( attrIdx ); ++partIdx ) {
          auto &region = pin.addRegion();
          region.setTypeIdMinus2( V3C_AVD - 2 );
          region.setPackedMapIndex( params.getPackedMapIndexByAttributeTypeId( ai.getAttributeTypeId( attrIdx ) ) );
          region.setAttrIndex( attrIdx );
          region.setAttrPartitionIndex( partIdx );

          std::cout << "region: typeIdMinus2=V3C_AVD - 2, packedMapIndex=" << int( region.getPackedMapIndex() )
                    << ", attrIndex= " << int( region.getAttrIndex() )
                    << ", attrPartitionIndex=" << int( region.getAttrPartitionIndex() ) << "\n";
        }
      }
    }

    if ( params.packGeometry_ || params.packAttributes_ ) { layoutPackedMapsHls( hls, params ); }
  }
}

void PCCGSHlsBuilder::updateAsps( PCCHighLevelSyntax           &hls,
                                  size_t                        mapWidth,
                                  size_t                        mapHeight,
                                  const PCCGSEncoderParameters &params ) {
  auto activeAfpsId = hls.getAtlasHighLevelSyntax(ATLAS_INDEX).getActiveAFPS();                                    
  auto &afps = hls.getAtlasFrameParameterSetList()[activeAfpsId];
  auto &asps = hls.getAtlasSequenceParameterSetList()[afps.getAtlasSequenceParameterSetId()];

  asps.setFrameWidth( mapWidth );
  asps.setFrameHeight( mapHeight );
}

void PCCGSHlsBuilder::layoutPackedMapsHls( PCCHighLevelSyntax &hls, const PCCGSEncoderParameters &params ) {
  auto      &vps    = hls.getVps();
  const auto width  = vps.getFrameWidth( ATLAS_INDEX );
  const auto height = vps.getFrameHeight( ATLAS_INDEX );
  auto      &pin    = vps.getVpsPackedVideoExtension().getPackingInformation( ATLAS_INDEX );

  if ( params.packedMapColumnsCount_.size() <= pin.getPackedMapCountMinus1() ) {
    std::cerr << "Error: not enough values in packedMapsColumnsCount parameters\n";
    exit( 1 );
  }
  for ( size_t packedMapIndex = 0; packedMapIndex <= pin.getPackedMapCountMinus1(); ++packedMapIndex ) {
    const auto columnsCount = params.packedMapColumnsCount_[packedMapIndex];
    std::clog << "layoutPackedMaps: packedMapIndex=" << packedMapIndex << ", columnsCount=" << columnsCount << "\n";
    auto nextIndex = 0;

    for ( size_t regionIdx = 0; regionIdx <= pin.getRegionsCountMinus1(); ++regionIdx ) {
      auto &region = pin.getRegion( regionIdx );

      if ( region.getPackedMapIndex() == packedMapIndex ) {
        auto index = nextIndex++;

        region.setWidthMinus1( width - 1 );
        region.setHeightMinus1( height - 1 );
        region.setTopLeftY( ( index / columnsCount ) * height );
        region.setTopLeftX( ( index % columnsCount ) * width );

        std::clog << "  region: attribute, packedMapIndex=" << int( packedMapIndex )
                  << ", attrIndex= " << int( region.getAttrIndex() )
                  << ", attrPartitionIndex=" << int( region.getAttrPartitionIndex() )
                  << ", widthMinus1=" << region.getWidthMinus1() << ", heightMinus1=" << region.getHeightMinus1()
                  << ", topLeftY=" << region.getTopLeftY() << ", topLeftX=" << region.getTopLeftX() << "\n";
      }
    }
  }
}

void PCCGSHlsBuilder::createGsCameraModelInfoSei( AtlasTileLayerRbsp &atgl, std::string path ) {
  std::string cameras_bin = path + "/cameras.bin";
  std::string images_bin  = path + "/images.bin";

  std::string cameras_txt = path + "/cameras.txt";
  std::string images_txt  = path + "/images.txt";

  std::string cameras_file;
  std::string images_file;

  if ( exist( cameras_bin ) && exist( images_bin ) ) {
    cameras_file = cameras_bin;
    images_file  = images_bin;
  } else {
    if ( exist( cameras_txt ) && exist( images_txt ) ) {
      cameras_file = cameras_txt;
      images_file  = images_txt;
    } else {
      throw std::runtime_error( "COLMAP files do not exits in folder: " + path );
    }
  }

  CameraModelInfo cam;
  cam.load( cameras_file, images_file );
  size_t numImages  = cam.numImages();
  size_t numCameras = cam.numCameras();
  bool   same       = cam.hasSameIntrinsics();
  auto  &camParams  = cam.getCamPoseParams();

  auto &sei = static_cast<SEIGsCameraModelInfo &>( atgl.getSEI().addSeiPrefix( GS_CAMERA_POSE_INFO, false ) );

  sei.setCameraNum(
      numImages );  // AJT: extrinsics are set based on the number of image poses according to the colmap convention.
  // AJT: I am doing this because of the way the current syntax table is written - propose to modify it?
  sei.setSameIntrinsicsFlag( same );
  sei.allocate();
  for ( size_t i = 0; i < sei.getCameraNum(); i++ ) {
    sei.setCameraId( i, camParams[i].camera_id );  // AJT: shouls we also include imageId - propose to syntax change!
    sei.setCancelFlag( i, false );
    if ( !sei.getCancelFlag( i ) ) {
      sei.setPersistenceFlag( i, false );
      if ( i == 0 || !sei.getSameIntrinsicsFlag() ) {
        sei.setImageWidth( i, static_cast<uint16_t>( camParams[i].width ) );
        sei.setImageHeight( i, static_cast<uint16_t>( camParams[i].height ) );
        sei.setHorFocalLength( i, static_cast<float>( camParams[i].fx ) );
        sei.setVerFocalLength( i, static_cast<float>( camParams[i].fy ) );
        sei.setHorPrincipalPoint( i, static_cast<float>( camParams[i].cx ) );
        sei.setVerPrincipalPoint( i, static_cast<float>( camParams[i].cy ) );
      }
      sei.setQuaternion( i, 0, static_cast<float>( camParams[i].qw ) );
      sei.setQuaternion( i, 1, static_cast<float>( camParams[i].qx ) );
      sei.setQuaternion( i, 2, static_cast<float>( camParams[i].qy ) );
      sei.setQuaternion( i, 3, static_cast<float>( camParams[i].qz ) );

      sei.setTranslation( i, 0, static_cast<float>( camParams[i].tx ) );
      sei.setTranslation( i, 1, static_cast<float>( camParams[i].ty ) );
      sei.setTranslation( i, 2, static_cast<float>( camParams[i].tz ) );
    }
  }
}
