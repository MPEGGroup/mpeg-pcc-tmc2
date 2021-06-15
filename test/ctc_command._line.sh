  
ENCODER="\
  --configurationFolder=./cfg/ \
  --colorSpaceConversionPath=./HDRTools/bin/HDRConvert\
  --videoEncoderPath=./HM-16.20+SCM-8.8/bin/TAppEncoderHighBitDepthStatic \
  --videoEncoderAuxPath=./HM-16.20+SCM-8.8/bin/TAppEncoderHighBitDepthStatic \
  --videoEncoderOccupancyMapPath=./HM-16.20+SCM-8.8/bin/TAppEncoderHighBitDepthStatic \
  --colorTransform=0 \
  --nbThread=1 \
  --uncompressedDataFolder=./mpeg_datasets/CfP/datasets/Dynamic_Objects/People/ ";

DECODER="\
  --videoDecoderPath=./HM-16.20+SCM-8.8/bin/TAppDecoderHighBitDepthStatic \
  --colorSpaceConversionPath=./HDRTools/bin/HDRConvert \
  --inverseColorSpaceConversionConfig=./cfg/hdrconvert/yuv420toyuv444_16bit.cfg \
  --nbThread=1 \
  --colorTransform=0 ";

# CWAI
./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-all-intra-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/queen-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S22CWAI_queen/S22CWAI_queen.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/Technicolor/queen_n/frame_%04d_n.ply \
  --frameCount=250 \
  --resolution=1023 > S22CWAI_queen/S22CWAI_queen_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0000 \
  --compressedStreamPath=S22CWAI_queen/S22CWAI_queen.bin > S22CWAI_queen/S22CWAI_queen_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-all-intra-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/loot_vox10-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S23CWAI_loot/S23CWAI_loot.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/loot_n/loot_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S23CWAI_loot/S23CWAI_loot_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1000 \
  --compressedStreamPath=S23CWAI_loot/S23CWAI_loot.bin  > S23CWAI_loot/S23CWAI_loot_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-all-intra-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/redandblack_vox10-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S24CWAI_redandblack/S24CWAI_redandblack.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/redandblack_n/redandblack_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S24CWAI_redandblack/S24CWAI_redandblack_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1450 \
  --compressedStreamPath=S24CWAI_redandblack/S24CWAI_redandblack.bin  > S24CWAI_redandblack/S24CWAI_redandblack_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-all-intra-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/soldier_vox10-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S25CWAI_soldier/S25CWAI_soldier.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/soldier_n/soldier_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S25CWAI_soldier/S25CWAI_soldier_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0536 \
  --compressedStreamPath=S25CWAI_soldier/S25CWAI_soldier.bin  > S25CWAI_soldier/S25CWAI_soldier_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-all-intra-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/longdress_vox10-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S26CWAI_longdress/S26CWAI_longdress.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/longdress_n/longdress_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S26CWAI_longdress/S26CWAI_longdress_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1051 \
  --compressedStreamPath=S26CWAI_longdress/S26CWAI_longdress.bin  > S26CWAI_longdress/S26CWAI_longdress_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-all-intra-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/basketball_player_vox11-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S27CWAI_basketball/S27CWAI_basketball.bin \
  --normalDataPath=./owlii/Vox11/basketball_player/basketball_player_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S27CWAI_basketball/S27CWAI_basketball_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S27CWAI_basketball/S27CWAI_basketball.bin  > S27CWAI_basketball/S27CWAI_basketball_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-all-intra-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/dancer_vox11-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S28CWAI_dancer/S28CWAI_dancer.bin \
  --normalDataPath=./owlii/Vox11/dancer/dancer_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S28CWAI_dancer/S28CWAI_dancer_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S28CWAI_dancer/S28CWAI_dancer.bin  > S28CWAI_dancer/S28CWAI_dancer_decoder.log 
# CWLD

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-low-delay-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/queen-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S22CWLD_queen/S22CWLD_queen.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/Technicolor/queen_n/frame_%04d_n.ply \
  --frameCount=250 \
  --resolution=1023 > S22CWLD_queen/S22CWLD_queen_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0000 \
  --compressedStreamPath=S22CWLD_queen/S22CWLD_queen.bin  > S22CWLD_queen/S22CWLD_queen_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-low-delay-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/loot_vox10-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S23CWLD_loot/S23CWLD_loot.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/loot_n/loot_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S23CWLD_loot/S23CWLD_loot_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1000 \
  --compressedStreamPath=S23CWLD_loot/S23CWLD_loot.bin  > S23CWLD_loot/S23CWLD_loot_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-low-delay-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/redandblack_vox10-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S24CWLD_redandblack/S24CWLD_redandblack.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/redandblack_n/redandblack_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S24CWLD_redandblack/S24CWLD_redandblack_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1450 \
  --compressedStreamPath=S24CWLD_redandblack/S24CWLD_redandblack.bin  > S24CWLD_redandblack/S24CWLD_redandblack_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-low-delay-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/soldier_vox10-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S25CWLD_soldier/S25CWLD_soldier.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/soldier_n/soldier_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S25CWLD_soldier/S25CWLD_soldier_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0536 \
  --compressedStreamPath=S25CWLD_soldier/S25CWLD_soldier.bin  > S25CWLD_soldier/S25CWLD_soldier_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-low-delay-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/longdress_vox10-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S26CWLD_longdress/S26CWLD_longdress.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/longdress_n/longdress_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S26CWLD_longdress/S26CWLD_longdress_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1051 \
  --compressedStreamPath=S26CWLD_longdress/S26CWLD_longdress.bin  > S26CWLD_longdress/S26CWLD_longdress_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-low-delay-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/basketball_player_vox11-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S27CWLD_basketball/S27CWLD_basketball.bin \
  --normalDataPath=./owlii/Vox11/basketball_player/basketball_player_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S27CWLD_basketball/S27CWLD_basketball_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S27CWLD_basketball/S27CWLD_basketball.bin  > S27CWLD_basketball/S27CWLD_basketball_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common-lossless-geometry-attribute.cfg \
  --config=./cfg/condition/ctc-low-delay-lossless-geometry-attribute.cfg \
  --config=./cfg/sequence/dancer_vox11-lossless.cfg \
  --config=./cfg/rate/ctc-r5-lossless.cfg \
  --compressedStreamPath=S28CWLD_dancer/S28CWLD_dancer.bin \
  --normalDataPath=./owlii/Vox11/dancer/dancer_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S28CWLD_dancer/S28CWLD_dancer_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S28CWLD_dancer/S28CWLD_dancer.bin  > S28CWLD_dancer/S28CWLD_dancer_decoder.log 
# C2AI

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/queen.cfg \
  --config=./cfg/rate/ctc-r1.cfg \
  --compressedStreamPath=S22C2AI_queen/S22C2AIR01_queen.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/Technicolor/queen_n/frame_%04d_n.ply \
  --frameCount=250 \
  --resolution=1023 > S22C2AI_queen/S22C2AIR01_queen_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0000 \
  --compressedStreamPath=S22C2AI_queen/S22C2AIR01_queen.bin \
  --inverseColorSpaceConversionConfig=./cfg/hdrconvert/yuv420toyuv444_16bit.cfg \
  --videoDecoderOccupancyMapPath=./HM-16.20+SCM-8.8/bin/TAppDecoderHighBitDepthStatic \
  --colorTransform=0 > S22C2AI_queen/S22C2AIR01_queen_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/queen.cfg \
  --config=./cfg/rate/ctc-r2.cfg \
  --compressedStreamPath=S22C2AI_queen/S22C2AIR02_queen.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/Technicolor/queen_n/frame_%04d_n.ply \
  --frameCount=250 \
  --resolution=1023 > S22C2AI_queen/S22C2AIR02_queen_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0000 \
  --compressedStreamPath=S22C2AI_queen/S22C2AIR02_queen.bin \
  --inverseColorSpaceConversionConfig=./cfg/hdrconvert/yuv420toyuv444_16bit.cfg \
  --videoDecoderOccupancyMapPath=./HM-16.20+SCM-8.8/bin/TAppDecoderHighBitDepthStatic \
  --colorTransform=0 > S22C2AI_queen/S22C2AIR02_queen_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/queen.cfg \
  --config=./cfg/rate/ctc-r3.cfg \
  --compressedStreamPath=S22C2AI_queen/S22C2AIR03_queen.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/Technicolor/queen_n/frame_%04d_n.ply \
  --frameCount=250 \
  --resolution=1023 > S22C2AI_queen/S22C2AIR03_queen_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0000 \
  --compressedStreamPath=S22C2AI_queen/S22C2AIR03_queen.bin > S22C2AI_queen/S22C2AIR03_queen_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/queen.cfg \
  --config=./cfg/rate/ctc-r4.cfg \
  --compressedStreamPath=S22C2AI_queen/S22C2AIR04_queen.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/Technicolor/queen_n/frame_%04d_n.ply \
  --frameCount=250 \
  --resolution=1023 > S22C2AI_queen/S22C2AIR04_queen_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0000 \
  --compressedStreamPath=S22C2AI_queen/S22C2AIR04_queen.bin > S22C2AI_queen/S22C2AIR04_queen_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/queen.cfg \
  --config=./cfg/rate/ctc-r5.cfg \
  --compressedStreamPath=S22C2AI_queen/S22C2AIR05_queen.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/Technicolor/queen_n/frame_%04d_n.ply \
  --frameCount=250 \
  --resolution=1023 > S22C2AI_queen/S22C2AIR05_queen_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0000 \
  --compressedStreamPath=S22C2AI_queen/S22C2AIR05_queen.bin > S22C2AI_queen/S22C2AIR05_queen_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/loot_vox10.cfg \
  --config=./cfg/rate/ctc-r1.cfg \
  --compressedStreamPath=S23C2AI_loot/S23C2AIR01_loot.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/loot_n/loot_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S23C2AI_loot/S23C2AIR01_loot_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1000 \
  --compressedStreamPath=S23C2AI_loot/S23C2AIR01_loot.bin > S23C2AI_loot/S23C2AIR01_loot_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/loot_vox10.cfg \
  --config=./cfg/rate/ctc-r2.cfg \
  --compressedStreamPath=S23C2AI_loot/S23C2AIR02_loot.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/loot_n/loot_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S23C2AI_loot/S23C2AIR02_loot_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1000 \
  --compressedStreamPath=S23C2AI_loot/S23C2AIR02_loot.bin > S23C2AI_loot/S23C2AIR02_loot_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/loot_vox10.cfg \
  --config=./cfg/rate/ctc-r3.cfg \
  --compressedStreamPath=S23C2AI_loot/S23C2AIR03_loot.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/loot_n/loot_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S23C2AI_loot/S23C2AIR03_loot_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1000 \
  --compressedStreamPath=S23C2AI_loot/S23C2AIR03_loot.bin > S23C2AI_loot/S23C2AIR03_loot_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/loot_vox10.cfg \
  --config=./cfg/rate/ctc-r4.cfg \
  --compressedStreamPath=S23C2AI_loot/S23C2AIR04_loot.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/loot_n/loot_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S23C2AI_loot/S23C2AIR04_loot_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1000 \
  --compressedStreamPath=S23C2AI_loot/S23C2AIR04_loot.bin > S23C2AI_loot/S23C2AIR04_loot_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/loot_vox10.cfg \
  --config=./cfg/rate/ctc-r5.cfg \
  --compressedStreamPath=S23C2AI_loot/S23C2AIR05_loot.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/loot_n/loot_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S23C2AI_loot/S23C2AIR05_loot_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1000 \
  --compressedStreamPath=S23C2AI_loot/S23C2AIR05_loot.bin > S23C2AI_loot/S23C2AIR05_loot_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/redandblack_vox10.cfg \
  --config=./cfg/rate/ctc-r1.cfg \
  --compressedStreamPath=S24C2AI_redandblack/S24C2AIR01_redandblack.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/redandblack_n/redandblack_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S24C2AI_redandblack/S24C2AIR01_redandblack_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1450 \
  --compressedStreamPath=S24C2AI_redandblack/S24C2AIR01_redandblack.bin > S24C2AI_redandblack/S24C2AIR01_redandblack_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/redandblack_vox10.cfg \
  --config=./cfg/rate/ctc-r2.cfg \
  --compressedStreamPath=S24C2AI_redandblack/S24C2AIR02_redandblack.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/redandblack_n/redandblack_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S24C2AI_redandblack/S24C2AIR02_redandblack_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1450 \
  --compressedStreamPath=S24C2AI_redandblack/S24C2AIR02_redandblack.bin > S24C2AI_redandblack/S24C2AIR02_redandblack_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/redandblack_vox10.cfg \
  --config=./cfg/rate/ctc-r3.cfg \
  --compressedStreamPath=S24C2AI_redandblack/S24C2AIR03_redandblack.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/redandblack_n/redandblack_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S24C2AI_redandblack/S24C2AIR03_redandblack_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1450 \
  --compressedStreamPath=S24C2AI_redandblack/S24C2AIR03_redandblack.bin > S24C2AI_redandblack/S24C2AIR03_redandblack_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/redandblack_vox10.cfg \
  --config=./cfg/rate/ctc-r4.cfg \
  --compressedStreamPath=S24C2AI_redandblack/S24C2AIR04_redandblack.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/redandblack_n/redandblack_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S24C2AI_redandblack/S24C2AIR04_redandblack_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1450 \
  --compressedStreamPath=S24C2AI_redandblack/S24C2AIR04_redandblack.bin > S24C2AI_redandblack/S24C2AIR04_redandblack_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/redandblack_vox10.cfg \
  --config=./cfg/rate/ctc-r5.cfg \
  --compressedStreamPath=S24C2AI_redandblack/S24C2AIR05_redandblack.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/redandblack_n/redandblack_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S24C2AI_redandblack/S24C2AIR05_redandblack_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1450 \
  --compressedStreamPath=S24C2AI_redandblack/S24C2AIR05_redandblack.bin > S24C2AI_redandblack/S24C2AIR05_redandblack_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/soldier_vox10.cfg \
  --config=./cfg/rate/ctc-r1.cfg \
  --compressedStreamPath=S25C2AI_soldier/S25C2AIR01_soldier.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/soldier_n/soldier_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S25C2AI_soldier/S25C2AIR01_soldier_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0536 \
  --compressedStreamPath=S25C2AI_soldier/S25C2AIR01_soldier.bin > S25C2AI_soldier/S25C2AIR01_soldier_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/soldier_vox10.cfg \
  --config=./cfg/rate/ctc-r2.cfg \
  --compressedStreamPath=S25C2AI_soldier/S25C2AIR02_soldier.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/soldier_n/soldier_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S25C2AI_soldier/S25C2AIR02_soldier_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0536 \
  --compressedStreamPath=S25C2AI_soldier/S25C2AIR02_soldier.bin > S25C2AI_soldier/S25C2AIR02_soldier_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/soldier_vox10.cfg \
  --config=./cfg/rate/ctc-r3.cfg \
  --compressedStreamPath=S25C2AI_soldier/S25C2AIR03_soldier.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/soldier_n/soldier_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S25C2AI_soldier/S25C2AIR03_soldier_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0536 \
  --compressedStreamPath=S25C2AI_soldier/S25C2AIR03_soldier.bin > S25C2AI_soldier/S25C2AIR03_soldier_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/soldier_vox10.cfg \
  --config=./cfg/rate/ctc-r4.cfg \
  --compressedStreamPath=S25C2AI_soldier/S25C2AIR04_soldier.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/soldier_n/soldier_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S25C2AI_soldier/S25C2AIR04_soldier_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0536 \
  --compressedStreamPath=S25C2AI_soldier/S25C2AIR04_soldier.bin > S25C2AI_soldier/S25C2AIR04_soldier_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/soldier_vox10.cfg \
  --config=./cfg/rate/ctc-r5.cfg \
  --compressedStreamPath=S25C2AI_soldier/S25C2AIR05_soldier.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/soldier_n/soldier_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S25C2AI_soldier/S25C2AIR05_soldier_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0536 \
  --compressedStreamPath=S25C2AI_soldier/S25C2AIR05_soldier.bin > S25C2AI_soldier/S25C2AIR05_soldier_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/longdress_vox10.cfg \
  --config=./cfg/rate/ctc-r1.cfg \
  --compressedStreamPath=S26C2AI_longdress/S26C2AIR01_longdress.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/longdress_n/longdress_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S26C2AI_longdress/S26C2AIR01_longdress_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1051 \
  --compressedStreamPath=S26C2AI_longdress/S26C2AIR01_longdress.bin > S26C2AI_longdress/S26C2AIR01_longdress_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/longdress_vox10.cfg \
  --config=./cfg/rate/ctc-r2.cfg \
  --compressedStreamPath=S26C2AI_longdress/S26C2AIR02_longdress.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/longdress_n/longdress_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S26C2AI_longdress/S26C2AIR02_longdress_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1051 \
  --compressedStreamPath=S26C2AI_longdress/S26C2AIR02_longdress.bin > S26C2AI_longdress/S26C2AIR02_longdress_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/longdress_vox10.cfg \
  --config=./cfg/rate/ctc-r3.cfg \
  --compressedStreamPath=S26C2AI_longdress/S26C2AIR03_longdress.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/longdress_n/longdress_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S26C2AI_longdress/S26C2AIR03_longdress_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1051 \
  --compressedStreamPath=S26C2AI_longdress/S26C2AIR03_longdress.bin > S26C2AI_longdress/S26C2AIR03_longdress_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/longdress_vox10.cfg \
  --config=./cfg/rate/ctc-r4.cfg \
  --compressedStreamPath=S26C2AI_longdress/S26C2AIR04_longdress.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/longdress_n/longdress_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S26C2AI_longdress/S26C2AIR04_longdress_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1051 \
  --compressedStreamPath=S26C2AI_longdress/S26C2AIR04_longdress.bin > S26C2AI_longdress/S26C2AIR04_longdress_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/longdress_vox10.cfg \
  --config=./cfg/rate/ctc-r5.cfg \
  --compressedStreamPath=S26C2AI_longdress/S26C2AIR05_longdress.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/longdress_n/longdress_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S26C2AI_longdress/S26C2AIR05_longdress_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1051 \
  --compressedStreamPath=S26C2AI_longdress/S26C2AIR05_longdress.bin > S26C2AI_longdress/S26C2AIR05_longdress_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/basketball_player_vox11.cfg \
  --config=./cfg/rate/ctc-r1.cfg \
  --compressedStreamPath=S27C2AI_basketball/S27C2AIR01_basketball.bin \
  --normalDataPath=./owlii/Vox11/basketball_player/basketball_player_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S27C2AI_basketball/S27C2AIR01_basketball_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S27C2AI_basketball/S27C2AIR01_basketball.bin > S27C2AI_basketball/S27C2AIR01_basketball_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/basketball_player_vox11.cfg \
  --config=./cfg/rate/ctc-r2.cfg \
  --compressedStreamPath=S27C2AI_basketball/S27C2AIR02_basketball.bin \
  --normalDataPath=./owlii/Vox11/basketball_player/basketball_player_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S27C2AI_basketball/S27C2AIR02_basketball_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S27C2AI_basketball/S27C2AIR02_basketball.bin > S27C2AI_basketball/S27C2AIR02_basketball_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/basketball_player_vox11.cfg \
  --config=./cfg/rate/ctc-r3.cfg \
  --compressedStreamPath=S27C2AI_basketball/S27C2AIR03_basketball.bin \
  --normalDataPath=./owlii/Vox11/basketball_player/basketball_player_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S27C2AI_basketball/S27C2AIR03_basketball_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S27C2AI_basketball/S27C2AIR03_basketball.bin > S27C2AI_basketball/S27C2AIR03_basketball_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/basketball_player_vox11.cfg \
  --config=./cfg/rate/ctc-r4.cfg \
  --compressedStreamPath=S27C2AI_basketball/S27C2AIR04_basketball.bin \
  --normalDataPath=./owlii/Vox11/basketball_player/basketball_player_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S27C2AI_basketball/S27C2AIR04_basketball_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S27C2AI_basketball/S27C2AIR04_basketball.bin > S27C2AI_basketball/S27C2AIR04_basketball_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/basketball_player_vox11.cfg \
  --config=./cfg/rate/ctc-r5.cfg \
  --compressedStreamPath=S27C2AI_basketball/S27C2AIR05_basketball.bin \
  --normalDataPath=./owlii/Vox11/basketball_player/basketball_player_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S27C2AI_basketball/S27C2AIR05_basketball_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S27C2AI_basketball/S27C2AIR05_basketball.bin > S27C2AI_basketball/S27C2AIR05_basketball_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/dancer_vox11.cfg \
  --config=./cfg/rate/ctc-r1.cfg \
  --compressedStreamPath=S28C2AI_dancer/S28C2AIR01_dancer.bin \
  --normalDataPath=./owlii/Vox11/dancer/dancer_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S28C2AI_dancer/S28C2AIR01_dancer_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S28C2AI_dancer/S28C2AIR01_dancer.bin > S28C2AI_dancer/S28C2AIR01_dancer_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/dancer_vox11.cfg \
  --config=./cfg/rate/ctc-r2.cfg \
  --compressedStreamPath=S28C2AI_dancer/S28C2AIR02_dancer.bin \
  --normalDataPath=./owlii/Vox11/dancer/dancer_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S28C2AI_dancer/S28C2AIR02_dancer_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S28C2AI_dancer/S28C2AIR02_dancer.bin > S28C2AI_dancer/S28C2AIR02_dancer_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/dancer_vox11.cfg \
  --config=./cfg/rate/ctc-r3.cfg \
  --compressedStreamPath=S28C2AI_dancer/S28C2AIR03_dancer.bin \
  --normalDataPath=./owlii/Vox11/dancer/dancer_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S28C2AI_dancer/S28C2AIR03_dancer_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S28C2AI_dancer/S28C2AIR03_dancer.bin > S28C2AI_dancer/S28C2AIR03_dancer_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/dancer_vox11.cfg \
  --config=./cfg/rate/ctc-r4.cfg \
  --compressedStreamPath=S28C2AI_dancer/S28C2AIR04_dancer.bin \
  --normalDataPath=./owlii/Vox11/dancer/dancer_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S28C2AI_dancer/S28C2AIR04_dancer_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S28C2AI_dancer/S28C2AIR04_dancer.bin > S28C2AI_dancer/S28C2AIR04_dancer_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-all-intra.cfg \
  --config=./cfg/sequence/dancer_vox11.cfg \
  --config=./cfg/rate/ctc-r5.cfg \
  --compressedStreamPath=S28C2AI_dancer/S28C2AIR05_dancer.bin \
  --normalDataPath=./owlii/Vox11/dancer/dancer_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S28C2AI_dancer/S28C2AIR05_dancer_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S28C2AI_dancer/S28C2AIR05_dancer.bin > S28C2AI_dancer/S28C2AIR05_dancer_decoder.log 
# C2RA

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/queen.cfg \
  --config=./cfg/rate/ctc-r1.cfg \
  --compressedStreamPath=S22C2RA_queen/S22C2RAR01_queen.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/Technicolor/queen_n/frame_%04d_n.ply \
  --frameCount=250 \
  --resolution=1023 > S22C2RA_queen/S22C2RAR01_queen_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0000 \
  --compressedStreamPath=S22C2RA_queen/S22C2RAR01_queen.bin > S22C2RA_queen/S22C2RAR01_queen_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/queen.cfg \
  --config=./cfg/rate/ctc-r2.cfg \
  --compressedStreamPath=S22C2RA_queen/S22C2RAR02_queen.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/Technicolor/queen_n/frame_%04d_n.ply \
  --frameCount=250 \
  --resolution=1023 > S22C2RA_queen/S22C2RAR02_queen_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0000 \
  --compressedStreamPath=S22C2RA_queen/S22C2RAR02_queen.bin > S22C2RA_queen/S22C2RAR02_queen_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/queen.cfg \
  --config=./cfg/rate/ctc-r3.cfg \
  --compressedStreamPath=S22C2RA_queen/S22C2RAR03_queen.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/Technicolor/queen_n/frame_%04d_n.ply \
  --frameCount=250 \
  --resolution=1023 > S22C2RA_queen/S22C2RAR03_queen_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0000 \
  --compressedStreamPath=S22C2RA_queen/S22C2RAR03_queen.bin > S22C2RA_queen/S22C2RAR03_queen_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/queen.cfg \
  --config=./cfg/rate/ctc-r4.cfg \
  --compressedStreamPath=S22C2RA_queen/S22C2RAR04_queen.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/Technicolor/queen_n/frame_%04d_n.ply \
  --frameCount=250 \
  --resolution=1023 > S22C2RA_queen/S22C2RAR04_queen_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0000 \
  --compressedStreamPath=S22C2RA_queen/S22C2RAR04_queen.bin > S22C2RA_queen/S22C2RAR04_queen_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/queen.cfg \
  --config=./cfg/rate/ctc-r5.cfg \
  --compressedStreamPath=S22C2RA_queen/S22C2RAR05_queen.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/Technicolor/queen_n/frame_%04d_n.ply \
  --frameCount=250 \
  --resolution=1023 > S22C2RA_queen/S22C2RAR05_queen_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0000 \
  --compressedStreamPath=S22C2RA_queen/S22C2RAR05_queen.bin > S22C2RA_queen/S22C2RAR05_queen_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/loot_vox10.cfg \
  --config=./cfg/rate/ctc-r1.cfg \
  --compressedStreamPath=S23C2RA_loot/S23C2RAR01_loot.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/loot_n/loot_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S23C2RA_loot/S23C2RAR01_loot_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1000 \
  --compressedStreamPath=S23C2RA_loot/S23C2RAR01_loot.bin > S23C2RA_loot/S23C2RAR01_loot_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/loot_vox10.cfg \
  --config=./cfg/rate/ctc-r2.cfg \
  --compressedStreamPath=S23C2RA_loot/S23C2RAR02_loot.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/loot_n/loot_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S23C2RA_loot/S23C2RAR02_loot_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1000 \
  --compressedStreamPath=S23C2RA_loot/S23C2RAR02_loot.bin > S23C2RA_loot/S23C2RAR02_loot_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/loot_vox10.cfg \
  --config=./cfg/rate/ctc-r3.cfg \
  --compressedStreamPath=S23C2RA_loot/S23C2RAR03_loot.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/loot_n/loot_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S23C2RA_loot/S23C2RAR03_loot_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1000 \
  --compressedStreamPath=S23C2RA_loot/S23C2RAR03_loot.bin > S23C2RA_loot/S23C2RAR03_loot_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/loot_vox10.cfg \
  --config=./cfg/rate/ctc-r4.cfg \
  --compressedStreamPath=S23C2RA_loot/S23C2RAR04_loot.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/loot_n/loot_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S23C2RA_loot/S23C2RAR04_loot_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1000 \
  --compressedStreamPath=S23C2RA_loot/S23C2RAR04_loot.bin > S23C2RA_loot/S23C2RAR04_loot_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/loot_vox10.cfg \
  --config=./cfg/rate/ctc-r5.cfg \
  --compressedStreamPath=S23C2RA_loot/S23C2RAR05_loot.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/loot_n/loot_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S23C2RA_loot/S23C2RAR05_loot_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1000 \
  --compressedStreamPath=S23C2RA_loot/S23C2RAR05_loot.bin > S23C2RA_loot/S23C2RAR05_loot_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/redandblack_vox10.cfg \
  --config=./cfg/rate/ctc-r1.cfg \
  --compressedStreamPath=S24C2RA_redandblack/S24C2RAR01_redandblack.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/redandblack_n/redandblack_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S24C2RA_redandblack/S24C2RAR01_redandblack_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1450 \
  --compressedStreamPath=S24C2RA_redandblack/S24C2RAR01_redandblack.bin > S24C2RA_redandblack/S24C2RAR01_redandblack_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/redandblack_vox10.cfg \
  --config=./cfg/rate/ctc-r2.cfg \
  --compressedStreamPath=S24C2RA_redandblack/S24C2RAR02_redandblack.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/redandblack_n/redandblack_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S24C2RA_redandblack/S24C2RAR02_redandblack_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1450 \
  --compressedStreamPath=S24C2RA_redandblack/S24C2RAR02_redandblack.bin > S24C2RA_redandblack/S24C2RAR02_redandblack_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/redandblack_vox10.cfg \
  --config=./cfg/rate/ctc-r3.cfg \
  --compressedStreamPath=S24C2RA_redandblack/S24C2RAR03_redandblack.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/redandblack_n/redandblack_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S24C2RA_redandblack/S24C2RAR03_redandblack_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1450 \
  --compressedStreamPath=S24C2RA_redandblack/S24C2RAR03_redandblack.bin > S24C2RA_redandblack/S24C2RAR03_redandblack_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/redandblack_vox10.cfg \
  --config=./cfg/rate/ctc-r4.cfg \
  --compressedStreamPath=S24C2RA_redandblack/S24C2RAR04_redandblack.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/redandblack_n/redandblack_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S24C2RA_redandblack/S24C2RAR04_redandblack_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1450 \
  --compressedStreamPath=S24C2RA_redandblack/S24C2RAR04_redandblack.bin > S24C2RA_redandblack/S24C2RAR04_redandblack_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/redandblack_vox10.cfg \
  --config=./cfg/rate/ctc-r5.cfg \
  --compressedStreamPath=S24C2RA_redandblack/S24C2RAR05_redandblack.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/redandblack_n/redandblack_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S24C2RA_redandblack/S24C2RAR05_redandblack_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1450 \
  --compressedStreamPath=S24C2RA_redandblack/S24C2RAR05_redandblack.bin > S24C2RA_redandblack/S24C2RAR05_redandblack_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/soldier_vox10.cfg \
  --config=./cfg/rate/ctc-r1.cfg \
  --compressedStreamPath=S25C2RA_soldier/S25C2RAR01_soldier.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/soldier_n/soldier_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S25C2RA_soldier/S25C2RAR01_soldier_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0536 \
  --compressedStreamPath=S25C2RA_soldier/S25C2RAR01_soldier.bin > S25C2RA_soldier/S25C2RAR01_soldier_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/soldier_vox10.cfg \
  --config=./cfg/rate/ctc-r2.cfg \
  --compressedStreamPath=S25C2RA_soldier/S25C2RAR02_soldier.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/soldier_n/soldier_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S25C2RA_soldier/S25C2RAR02_soldier_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0536 \
  --compressedStreamPath=S25C2RA_soldier/S25C2RAR02_soldier.bin > S25C2RA_soldier/S25C2RAR02_soldier_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/soldier_vox10.cfg \
  --config=./cfg/rate/ctc-r3.cfg \
  --compressedStreamPath=S25C2RA_soldier/S25C2RAR03_soldier.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/soldier_n/soldier_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S25C2RA_soldier/S25C2RAR03_soldier_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0536 \
  --compressedStreamPath=S25C2RA_soldier/S25C2RAR03_soldier.bin > S25C2RA_soldier/S25C2RAR03_soldier_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/soldier_vox10.cfg \
  --config=./cfg/rate/ctc-r4.cfg \
  --compressedStreamPath=S25C2RA_soldier/S25C2RAR04_soldier.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/soldier_n/soldier_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S25C2RA_soldier/S25C2RAR04_soldier_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0536 \
  --compressedStreamPath=S25C2RA_soldier/S25C2RAR04_soldier.bin > S25C2RA_soldier/S25C2RAR04_soldier_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/soldier_vox10.cfg \
  --config=./cfg/rate/ctc-r5.cfg \
  --compressedStreamPath=S25C2RA_soldier/S25C2RAR05_soldier.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/soldier_n/soldier_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S25C2RA_soldier/S25C2RAR05_soldier_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0536 \
  --compressedStreamPath=S25C2RA_soldier/S25C2RAR05_soldier.bin > S25C2RA_soldier/S25C2RAR05_soldier_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/longdress_vox10.cfg \
  --config=./cfg/rate/ctc-r1.cfg \
  --compressedStreamPath=S26C2RA_longdress/S26C2RAR01_longdress.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/longdress_n/longdress_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S26C2RA_longdress/S26C2RAR01_longdress_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1051 \
  --compressedStreamPath=S26C2RA_longdress/S26C2RAR01_longdress.bin > S26C2RA_longdress/S26C2RAR01_longdress_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/longdress_vox10.cfg \
  --config=./cfg/rate/ctc-r2.cfg \
  --compressedStreamPath=S26C2RA_longdress/S26C2RAR02_longdress.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/longdress_n/longdress_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S26C2RA_longdress/S26C2RAR02_longdress_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1051 \
  --compressedStreamPath=S26C2RA_longdress/S26C2RAR02_longdress.bin > S26C2RA_longdress/S26C2RAR02_longdress_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/longdress_vox10.cfg \
  --config=./cfg/rate/ctc-r3.cfg \
  --compressedStreamPath=S26C2RA_longdress/S26C2RAR03_longdress.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/longdress_n/longdress_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S26C2RA_longdress/S26C2RAR03_longdress_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1051 \
  --compressedStreamPath=S26C2RA_longdress/S26C2RAR03_longdress.bin > S26C2RA_longdress/S26C2RAR03_longdress_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/longdress_vox10.cfg \
  --config=./cfg/rate/ctc-r4.cfg \
  --compressedStreamPath=S26C2RA_longdress/S26C2RAR04_longdress.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/longdress_n/longdress_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S26C2RA_longdress/S26C2RAR04_longdress_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1051 \
  --compressedStreamPath=S26C2RA_longdress/S26C2RAR04_longdress.bin > S26C2RA_longdress/S26C2RAR04_longdress_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/longdress_vox10.cfg \
  --config=./cfg/rate/ctc-r5.cfg \
  --compressedStreamPath=S26C2RA_longdress/S26C2RAR05_longdress.bin \
  --normalDataPath=./mpeg_datasets/CfP/normals/allInfo/Dynamic_Objects/People/8i/longdress_n/longdress_vox10_%04d_n.ply \
  --frameCount=300 \
  --resolution=1023 > S26C2RA_longdress/S26C2RAR05_longdress_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=1051 \
  --compressedStreamPath=S26C2RA_longdress/S26C2RAR05_longdress.bin > S26C2RA_longdress/S26C2RAR05_longdress_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/basketball_player_vox11.cfg \
  --config=./cfg/rate/ctc-r1.cfg \
  --compressedStreamPath=S27C2RA_basketball/S27C2RAR01_basketball.bin \
  --normalDataPath=./owlii/Vox11/basketball_player/basketball_player_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S27C2RA_basketball/S27C2RAR01_basketball_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S27C2RA_basketball/S27C2RAR01_basketball.bin > S27C2RA_basketball/S27C2RAR01_basketball_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/basketball_player_vox11.cfg \
  --config=./cfg/rate/ctc-r2.cfg \
  --compressedStreamPath=S27C2RA_basketball/S27C2RAR02_basketball.bin \
  --normalDataPath=./owlii/Vox11/basketball_player/basketball_player_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S27C2RA_basketball/S27C2RAR02_basketball_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S27C2RA_basketball/S27C2RAR02_basketball.bin > S27C2RA_basketball/S27C2RAR02_basketball_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/basketball_player_vox11.cfg \
  --config=./cfg/rate/ctc-r3.cfg \
  --compressedStreamPath=S27C2RA_basketball/S27C2RAR03_basketball.bin \
  --normalDataPath=./owlii/Vox11/basketball_player/basketball_player_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S27C2RA_basketball/S27C2RAR03_basketball_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S27C2RA_basketball/S27C2RAR03_basketball.bin > S27C2RA_basketball/S27C2RAR03_basketball_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/basketball_player_vox11.cfg \
  --config=./cfg/rate/ctc-r4.cfg \
  --compressedStreamPath=S27C2RA_basketball/S27C2RAR04_basketball.bin \
  --normalDataPath=./owlii/Vox11/basketball_player/basketball_player_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S27C2RA_basketball/S27C2RAR04_basketball_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S27C2RA_basketball/S27C2RAR04_basketball.bin > S27C2RA_basketball/S27C2RAR04_basketball_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/basketball_player_vox11.cfg \
  --config=./cfg/rate/ctc-r5.cfg \
  --compressedStreamPath=S27C2RA_basketball/S27C2RAR05_basketball.bin \
  --normalDataPath=./owlii/Vox11/basketball_player/basketball_player_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S27C2RA_basketball/S27C2RAR05_basketball_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S27C2RA_basketball/S27C2RAR05_basketball.bin > S27C2RA_basketball/S27C2RAR05_basketball_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/dancer_vox11.cfg \
  --config=./cfg/rate/ctc-r1.cfg \
  --compressedStreamPath=S28C2RA_dancer/S28C2RAR01_dancer.bin \
  --normalDataPath=./owlii/Vox11/dancer/dancer_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S28C2RA_dancer/S28C2RAR01_dancer_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S28C2RA_dancer/S28C2RAR01_dancer.bin > S28C2RA_dancer/S28C2RAR01_dancer_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/dancer_vox11.cfg \
  --config=./cfg/rate/ctc-r2.cfg \
  --compressedStreamPath=S28C2RA_dancer/S28C2RAR02_dancer.bin \
  --normalDataPath=./owlii/Vox11/dancer/dancer_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S28C2RA_dancer/S28C2RAR02_dancer_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S28C2RA_dancer/S28C2RAR02_dancer.bin > S28C2RA_dancer/S28C2RAR02_dancer_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/dancer_vox11.cfg \
  --config=./cfg/rate/ctc-r3.cfg \
  --compressedStreamPath=S28C2RA_dancer/S28C2RAR03_dancer.bin \
  --normalDataPath=./owlii/Vox11/dancer/dancer_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S28C2RA_dancer/S28C2RAR03_dancer_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S28C2RA_dancer/S28C2RAR03_dancer.bin > S28C2RA_dancer/S28C2RAR03_dancer_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/dancer_vox11.cfg \
  --config=./cfg/rate/ctc-r4.cfg \
  --compressedStreamPath=S28C2RA_dancer/S28C2RAR04_dancer.bin \
  --normalDataPath=./owlii/Vox11/dancer/dancer_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S28C2RA_dancer/S28C2RAR04_dancer_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S28C2RA_dancer/S28C2RAR04_dancer.bin > S28C2RA_dancer/S28C2RAR04_dancer_decoder.log 

./bin/PccAppEncoder \
  $ENCODER \
  --config=./cfg/common/ctc-common.cfg \
  --config=./cfg/condition/ctc-random-access.cfg \
  --config=./cfg/sequence/dancer_vox11.cfg \
  --config=./cfg/rate/ctc-r5.cfg \
  --compressedStreamPath=S28C2RA_dancer/S28C2RAR05_dancer.bin \
  --normalDataPath=./owlii/Vox11/dancer/dancer_vox11_%04d.ply \
  --frameCount=64 \
  --resolution=2047 > S28C2RA_dancer/S28C2RAR05_dancer_encoder.log 

./bin/PccAppDecoder \
  $DECODER\
  --startFrameNumber=0001 \
  --compressedStreamPath=S28C2RA_dancer/S28C2RAR05_dancer.bin > S28C2RA_dancer/S28C2RAR05_dancer_decoder.log 
