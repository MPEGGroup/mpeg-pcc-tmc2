#!/bin/bash

MAINDIR=$( dirname $( cd "$( dirname $0 )" && pwd ) );
EXTERNAL=$( dirname $MAINDIR )/external_pkg


## Input parameters

SOURCE=${MAINDIR}/../source/longdress/longdress_vox10_%04i.ply
FRAMECOUNT=1;
FRAMEINDEX=1051;
BIN=longdress_F${FRAMECOUNT}.bin
THREAD=1;


## External tool paths
ENCODER="set_value";
DECODER="set_value";
HDRCONVERT="set_value";
HMENCODER="set_value";
HMDECODER="set_value";

if [ ! -f $ENCODER    ] ; then ENCODER=${MAINDIR}/bin/PccAppEncoder;             fi
if [ ! -f $ENCODER    ] ; then ENCODER=${MAINDIR}/bin/Release/PccAppEncoder.exe; fi
if [ ! -f $ENCODER    ] ; then ENCODER=${MAINDIR}/bin/Debug/PccAppEncoder.exe;   fi

if [ ! -f $DECODER    ] ; then DECODER=${MAINDIR}/bin/PccAppDecoder;             fi
if [ ! -f $DECODER    ] ; then DECODER=${MAINDIR}/bin/Release/PccAppDecoder.exe; fi
if [ ! -f $DECODER    ] ; then DECODER=${MAINDIR}/bin/Debug/PccAppDecoder.exe;   fi

if [ ! -f $HDRCONVERT ] ; then HDRCONVERT=${EXTERNAL}/HDRTools-0.17-dev/bin/HDRConvert.exe; fi
if [ ! -f $HDRCONVERT ] ; then HDRCONVERT=${EXTERNAL}/HDRTools-0.17-dev/bin/HDRConvert;     fi
if [ ! -f $HDRCONVERT ] ; then HDRCONVERT=${EXTERNAL}/HDRTools/bin/HDRConvert;              fi

if [ ! -f $HMENCODER ] ; then HMENCODER=${EXTERNAL}/HM16.16/bin/vc2015/x64/Release/TAppEncoder.exe; fi
if [ ! -f $HMENCODER ] ; then HMENCODER=${EXTERNAL}/HM16.16/TAppEncoder;                            fi
if [ ! -f $HMENCODER ] ; then HMENCODER=${EXTERNAL}/hm/bin/TAppEncoder;                             fi

if [ ! -f $HMDECODER ] ; then HMDECODER=${EXTERNAL}/HM16.16/bin/vc2015/x64/Release/TAppDecoder.exe; fi
if [ ! -f $HMDECODER ] ; then HMDECODER=${EXTERNAL}/HM16.16/bin/TAppDecoder;                        fi
if [ ! -f $HMDECODER ] ; then HMDECODER=${EXTERNAL}/hm/bin/TAppDecoder;                             fi


## Parameters and pathes check
if [ ! -f $ENCODER    ] ; then echo "Can't find PccAppEncoder, please set.     ($ENCODER )";   exit -1; fi
if [ ! -f $DECODER    ] ; then echo "Can't find PccAppDecoder, please set.     ($DECODER )";   exit -1; fi
if [ ! -f $HDRCONVERT ] ; then echo "Can't find HdrConverts, please set.       ($HDRCONVERT)"; exit -1; fi
if [ ! -f $HMENCODER  ] ; then echo "Can't find TAppEncoderStatic, please set. ($HMENCODER)";  exit -1; fi
if [ ! -f $HMDECODER  ] ; then echo "Can't find TAppDecoderStatic, please set. ($HMDECODER)";  exit -1; fi
for((i=0;i<$FRAMECOUNT;i++))
do
  INDEX=`expr $i + $FRAMEINDEX`;
  PLY=`printf $SOURCE $INDEX`;
  if [ ! -f $PLY  ] ; then echo "Can't find input ply, please set. ($PLY)"; exit -1; fi
done


## Encoder 
if [ ! -f $BIN ] 
then 
  $ENCODER \
    --config=${MAINDIR}/cfg/ctc-common.cfg \
    --config=${MAINDIR}/cfg/per-sequence/longdress_vox10.cfg \
    --config=${MAINDIR}/cfg/ctc-r1.cfg \
    --colorSpaceConversionPath=$HDRCONVERT \
    --colorSpaceConversionConfig=${MAINDIR}/cfg/rgb444toyuv420.cfg \
    --inverseColorSpaceConversionConfig=${MAINDIR}/cfg/yuv420torgb444.cfg \
    --uncompressedDataPath=$SOURCE \
    --startFrameNumber=$FRAMEINDEX \
    --frameCount=$FRAMECOUNT \
    --videoEncoderPath=${HMENCODER} \
    --geometryConfig=${MAINDIR}/cfg/ctc-hm-geometry-ai.cfg \
    --textureConfig=${MAINDIR}/cfg/ctc-hm-texture-ai.cfg \
    --nbThread=$THREAD \
    --keepIntermediateFiles=1 \
    --reconstructedDataPath=${BIN%.???}_rec_%04d.ply \
    --compressedStreamPath=$BIN
fi

## Decoder
$DECODER \
  --startFrameNumber=1051 \
  --compressedStreamPath=$BIN \
  --videoDecoderPath=${HMDECODER} \
  --colorSpaceConversionPath=${HDRCONVERT} \
  --inverseColorSpaceConversionConfig=${MAINDIR}/cfg/yuv420torgb444.cfg \
  --nbThread=$THREAD \
  --reconstructedDataPath=${BIN%.???}_dec_%04d.ply
