#!/bin/bash

MAINDIR=$( dirname $( cd "$( dirname $0 )" && pwd ) );
EXTERNAL=$( dirname $MAINDIR )/external_pkg
if [ ! -d $EXTERNAL ] ; then EXTERNAL=$( dirname $MAINDIR )/external; fi

## Input parameters
SRCDIR=${MAINDIR}/../ply/ # note: this directory must containt: http://mpegfs.int-evry.fr/MPEG/PCC/DataSets/pointCloud/CfP/datasets/Dynamic_Objects/People                            
CFGDIR=${MAINDIR}/cfg/

SEQ=25;       # in [22;26]
COND=2;       # in [C2AI, C2LD, CWAI, CWRA]
RATE=2;       # in [1;5]
FRAMECOUNT=1;
THREAD=1;

## Set external tool paths
ENCODER="set_value";
DECODER="set_value";
HDRCONVERT="set_value";
HMENCODER="set_value";
HMDECODER="set_value";

if [ ! -f $ENCODER    ] ; then ENCODER=${MAINDIR}/bin/PccAppEncoder;               fi
if [ ! -f $ENCODER    ] ; then ENCODER=${MAINDIR}/bin/Release/PccAppEncoder.exe;   fi
if [ ! -f $ENCODER    ] ; then ENCODER=${MAINDIR}/bin/Debug/PccAppEncoder.exe;     fi
                                                                                   
if [ ! -f $DECODER    ] ; then DECODER=${MAINDIR}/bin/PccAppDecoder;               fi
if [ ! -f $DECODER    ] ; then DECODER=${MAINDIR}/bin/Release/PccAppDecoder.exe;   fi
if [ ! -f $DECODER    ] ; then DECODER=${MAINDIR}/bin/Debug/PccAppDecoder.exe;     fi

if [ ! -f $HDRCONVERT ] ; then HDRCONVERT=${EXTERNAL}/HDRTools/bin/HDRConvert;     fi
if [ ! -f $HDRCONVERT ] ; then HDRCONVERT=${EXTERNAL}/HDRTools/bin/HDRConvert.exe; fi

if [ ! -f $HMENCODER ] ; then HMENCODER=${EXTERNAL}/HM-16.18+SCM-8.7/bin/TAppEncoderStatic;                  fi
if [ ! -f $HMENCODER ] ; then HMENCODER=${EXTERNAL}/HM-16.18+SCM-8.7/bin/vc2015/x64/Release/TAppEncoder.exe; fi
if [ ! -f $HMENCODER ] ; then HMENCODER=${EXTERNAL}/HM-16.18+SCM-8.7/bin/vc2015/x64/Debug/TAppEncoder.exe;   fi
                                                   
if [ ! -f $HMDECODER ] ; then HMDECODER=${EXTERNAL}/HM-16.18+SCM-8.7/bin/TAppDecoderStatic;                  fi
if [ ! -f $HMDECODER ] ; then HMDECODER=${EXTERNAL}/HM-16.18+SCM-8.7/bin/vc2015/x64/Release/TAppDecoder.exe; fi
if [ ! -f $HMDECODER ] ; then HMDECODER=${EXTERNAL}/HM-16.18+SCM-8.7/bin/vc2015/x64/Debug/TAppDecoder.exe;   fi

## Parameters and pathes check
if [ ! -f $ENCODER    ] ; then echo "Can't find PccAppEncoder, please set.     ($ENCODER )";   exit -1; fi
if [ ! -f $DECODER    ] ; then echo "Can't find PccAppDecoder, please set.     ($DECODER )";   exit -1; fi
if [ ! -f $HDRCONVERT ] ; then echo "Can't find HdrConvert, please set.        ($HDRCONVERT)"; exit -1; fi
if [ ! -f $HMENCODER  ] ; then echo "Can't find TAppEncoderStatic, please set. ($HMENCODER)";  exit -1; fi
if [ ! -f $HMDECODER  ] ; then echo "Can't find TAppDecoderStatic, please set. ($HMDECODER)";  exit -1; fi

## Set Configuration based on sequence, condition and rate
if [ $COND == "C2AI" -o $COND == "C2RA" ] 
then
  case $SEQ in
      22) CFGSEQUENCE="sequence/queen.cfg";;
      23) CFGSEQUENCE="sequence/loot_vox10.cfg";;
      24) CFGSEQUENCE="sequence/redandblack_vox10.cfg";;
      25) CFGSEQUENCE="sequence/soldier_vox10.cfg";;
      26) CFGSEQUENCE="sequence/longdress_vox10.cfg";;
      27) CFGSEQUENCE="sequence/basketball_player_vox11";;
      28) CFGSEQUENCE="sequence/dancer_vox11.cfg";;
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
  BIN=S${SEQ}${COND}R0${RATE}_F${FRAMECOUNT}.bin
else
   case $SEQ in
      22) CFGSEQUENCE="sequence/queen-lossless.cfg";;
      23) CFGSEQUENCE="sequence/loot_vox10-lossless.cfg";;
      24) CFGSEQUENCE="sequence/redandblack_vox10-lossless.cfg";;
      25) CFGSEQUENCE="sequence/soldier_vox10-lossless.cfg";;
      26) CFGSEQUENCE="sequence/longdress_vox10-lossless.cfg";;
      27) CFGSEQUENCE="sequence/basketball_player_vox11-lossless";;
      28) CFGSEQUENCE="sequence/dancer_vox11-lossless.cfg";;
      *) echo "sequence not correct ($SEQ)";   exit -1;;
  esac 
  CFGRATE="rate/ctc-r5.cfg"
  BIN=S${SEQ}${COND}_F${FRAMECOUNT}.bin
fi

case $COND in
  CWAI) CFGCOMMON="common/ctc-common-lossless-geometry-attribute.cfg";;
  CWLD) CFGCOMMON="common/ctc-common-lossless-geometry-attribute.cfg";; 
  C2AI) CFGCOMMON="common/ctc-common.cfg";;                           
  C2RA) CFGCOMMON="common/ctc-common.cfg";;           
  *) echo "Condition not correct ($COND)";   exit -1;;
esac

case $COND in
  CWAI) CFGCONDITION="condition/ctc-all-intra-lossless-geometry-attribute.cfg";;
  CWLD) CFGCONDITION="condition/ctc-low-delay-lossless-geometry-attribute.cfg";;
  C2AI) CFGCONDITION="condition/ctc-all-intra.cfg";;
  C2RA) CFGCONDITION="condition/ctc-random-access.cfg";;  
  *) echo "Condition not correct ($COND)";   exit -1;;
esac

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
  --inverseColorSpaceConversionConfig=${CFGDIR}/hdrconvert/yuv420toyuv444_16bit.cfg \
  --nbThread=$THREAD \
  --reconstructedDataPath=${BIN%.???}_dec_%04d.ply
