#!/bin/bash

MAINDIR=$( dirname $( cd "$( dirname $0 )" && pwd ) );
EXTERNAL=$( dirname $MAINDIR )/external_pkg

## Input parameters
SRCDIR=${MAINDIR}/../ply/ # note: this directory must containt: http://mpegfs.int-evry.fr/MPEG/PCC/DataSets/pointCloud/CfP/datasets/Dynamic_Objects/People                            
CFGDIR=${MAINDIR}/cfg/
RATE=2;
COND=3;
SEQ=25; 
FRAMECOUNT=1;
THREAD=1;

## Set Configuration based on sequence, condition and rate
if [ $COND -lt 3 ] 
then
  case $SEQ in
      22) CFGSEQUENCE="sequence/queen-lossless.cfg";;
      23) CFGSEQUENCE="sequence/loot_vox10-lossless.cfg";;
      24) CFGSEQUENCE="sequence/redandblack_vox10-lossless.cfg";;
      25) CFGSEQUENCE="sequence/soldier_vox10-lossless.cfg";;
      26) CFGSEQUENCE="sequence/longdress_vox10-lossless.cfg";;
      *) echo "sequence not correct ($SEQ)";   exit -1;;
  esac 
  CFGRATE=;
  BIN=S${SEQ}C0${COND}_F${FRAMECOUNT}.bin
else
  case $SEQ in
      22) CFGSEQUENCE="sequence/queen-lossless.cfg";;
      23) CFGSEQUENCE="sequence/loot_vox10-lossless.cfg";;
      24) CFGSEQUENCE="sequence/redandblack_vox10-lossless.cfg";;
      25) CFGSEQUENCE="sequence/soldier_vox10-lossless.cfg";;
      26) CFGSEQUENCE="sequence/longdress_vox10-lossless.cfg";;
      *) echo "sequence not correct ($SEQ)";   exit -1;;
  esac
  case $RATE in
      5) CFGRATE="rate/ctc-r5.cfg";; 
      4) CFGRATE="rate/ctc-r4.cfg";; 
      3) CFGRATE="rate/ctc-r3.cfg";; 
      2) CFGRATE="rate/ctc-r2.cfg";; 
      1) CFGRATE="rate/ctc-r1.cfg";; 
      *) echo "rate not correct ($RATE)";   exit -1;;
  esac
  BIN=S${SEQ}C0${COND}R0${RATE}_F${FRAMECOUNT}.bin
fi

case $COND in
  1) CFGCOMMON="common/ctc-common-lossless-geometry.cfg";          CFGCONDITION="condition/ctc-all-intra-lossless-geometry.cfg";;
  2) CFGCOMMON="common/ctc-common-lossless-geometry-texture.cfg";  CFGCONDITION="condition/ctc-all-intra-lossless-geometry-texture.cfg";;
  3) CFGCOMMON="common/ctc-common.cfg";                            CFGCONDITION="condition/ctc-all-intra.cfg";;
  4) CFGCOMMON="common/ctc-common.cfg";                            CFGCONDITION="condition/ctc-random-access.cfg";;  
  *) echo "Condition not correct ($COND)";   exit -1;;
esac

## Set external tool paths
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

## Encoder 
if [ ! -f $BIN ] 
then 
  $ENCODER \
    --config=${CFGDIR}${CFGCOMMON} \
    --config=${CFGDIR}${CFGSEQUENCE} \
    --config=${CFGDIR}${CFGCONDITION} \
    --config=${CFGDIR}${CFGRATE} \
    --configurationFolder=${CFGDIR} \
    --uncompressedDataFolder=${SRCDIR} \
    --frameCount=$FRAMECOUNT \
    --colorSpaceConversionPath=$HDRCONVERT \
    --videoEncoderPath=$HMENCODER \
    --nbThread=$THREAD \
    --keepIntermediateFiles=1 \
    --reconstructedDataPath=${BIN%.???}_rec_%04d.ply \
    --compressedStreamPath=$BIN
fi

## Decoder
$DECODER \
  --compressedStreamPath=$BIN \
  --videoDecoderPath=${HMDECODER} \
  --colorSpaceConversionPath=${HDRCONVERT} \
  --inverseColorSpaceConversionConfig=${CFGDIR}/hdrconvert/yuv420torgb444.cfg \
  --nbThread=$THREAD \
  --reconstructedDataPath=${BIN%.???}_dec_%04d.ply
