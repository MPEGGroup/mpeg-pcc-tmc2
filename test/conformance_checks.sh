#!/bin/bash 

CURDIR=`dirname $(readlink -f $0)`;
echo -e "\033[0;32m$0 $CURDIR \033[0m";

# Input parameters
TMC2DEC="";
TMC2CONF="";
TESTDIR=`pwd`;

# Print usage function
function printUsage(){
  echo "$0 usage:"
  echo ""
  echo "  Parameters:"
  echo "   --help             : print help"
  echo "   -d*|--decoder      : TMC2 decoder software path                       (default $TMC2DEC)"
  echo "   -c*|--conformance  : TMC2 conformance software path                   (default $TMC2CONF)"
  echo "   -t*|--dir*         : Test directory where the conformance bitstream   (default $TESTDIR)"
  echo "                        archives are stored in \"zip\/\" subfolder";
  echo ""
  echo "  Example:"
  echo "    $0 -d ./tmc2_r130/bin/PccAppDecoder -c ./tmc2_r130/bin/PccAppConformance -t ./conformance_bitstreams/"
  echo ""   
  echo "Message: "
  echo "  $1"
  exit -1; 
}

# Parse input parameters
while [[ $# -gt 0 ]]
do
  C=$1; 
  if [[ "$C" =~ [=] ]] ; then  V=`echo $1 | sed 's/[-a-zA-Z0-9]*=//'`; else V=$2; if [ "${2:0:1}" != "-" ] ; then shift; fi; fi;  
  case "$C" in            
    -h|--help          ) printUsage "Number of parameters must be 1 ( != $# )."; exit 1;;
    -d*|--decoder*     ) TMC2DEC=$V;;
    -c*|--conformance* ) TMC2CONF=$V;;
    -t*|--testdir*     ) TESTDIR=$V;;
  esac
  shift; 
done

# Input parameters checks
if [ "${TMC2DEC}"  == "" ] || [ ! -f ${TMC2DEC}     ] ; then printUsage "TMC2 decoder software path must be set and must exist"; fi
if [ "${TMC2CONF}" == "" ] || [ ! -f ${TMC2CONF}    ] ; then printUsage "TMC2 conformance software path must be set and must exist"; fi
if [ "`ls ${TESTDIR}/zip/*.zip 2> /dev/null`" == "" ] ; then printUsage "Test director not contain zip files stored in ${TESTDIR}/zip/ subfolder"; fi

# Main loop en the available conformance bitstream archives
for ZIPFILE in ${TESTDIR}/zip/*.zip 
do
  # Set name of the current test
  NAME=`basename ${ZIPFILE%.???}`
  DSTDIR=${TESTDIR}/${NAME}
  
  echo -e "$NAME conformance bitstreams: "
  
  # Unzip archives
  if [ ! -d ${DSTDIR} ] 
  then 
    echo "  Unzip: ${NAME}";
    unzip ${ZIPFILE} -d ${DSTDIR}; 
  else 
    echo "  Archive already unzips: ${NAME}";
  fi 
    
  # Remove sub directory if exist
  for SURDIR in `ls -d ${DSTDIR}/*/ 2> /dev/null | grep -v -e ${DSTDIR}/cfg -e ${DSTDIR}/encoderlog -e ${DSTDIR}/decoderlog -e ${DSTDIR}/conformance`
  do 
    mv ${SURDIR}/* ${DSTDIR}/
    rm -rf ${SURDIR}    
  done 
  
  # Organize data in subfolders: cfg configuration files, encoder log files and decoder log files.
  mkdir -p ${DSTDIR}/cfg/ 
  mkdir -p ${DSTDIR}/encoderlog/ 
  mkdir -p ${DSTDIR}/decoderlog/ 
  mkdir -p ${DSTDIR}/conformance/ 
  mv ${DSTDIR}/*.cfg ${DSTDIR}/cfg/        2> /dev/null
  mv ${DSTDIR}/*.txt ${DSTDIR}/encoderlog/ 2> /dev/null
 
  # Decode bitstream  
  for BINFILE in ${DSTDIR}/*.bin 
  do 
    DECDIR=`dirname ${BINFILE}`/decoderlog/
    DECNAME=`basename ${BINFILE%.???}`
    if [ ! -f ${DECDIR}/${DECNAME}_decoder.log ] || [ "`cat ${DECDIR}/${DECNAME}_decoder.log | grep "Peak memory:" `" == "" ] 
    then 
      echo "  Start decoding process: ${DECNAME}.bin"
      cp ${BINFILE} ${DECDIR}
      ${TMC2DEC} \
        --compressedStreamPath=${DECDIR}/${DECNAME}.bin \
        --reconstructedDataPath=${DECDIR}/${DECNAME}_%04d.ply \
        --inverseColorSpaceConversionConfig=${DSTDIR}/cfg/yuv420toyuv444_16bit.cfg \
       > ${DECDIR}/${DECNAME}_decoder.log 2>&1
    else 
      echo "  Decoding already done and the decoding process is complete: ${DECNAME}_decoder.log"
    fi            
    
    # Compare encoder and decoder log files 
    CONFDIR=`dirname ${BINFILE}`/conformance/
    
    # Copy log files
    MISSINGFILE=0;
    for SUFFIX in bitstream_md5.txt hls_md5.txt atlas_log.txt tile_log.txt pcframe_log.txt pcframe_log.txt picture_log.txt
    do    
      # Copy encoder files
      if [ ! -f ${CONFDIR}/${DECNAME}_enc_${SUFFIX} ]
      then 
        for LOG in `ls ${DSTDIR}/encoderlog/*_enc_${SUFFIX} 2> /dev/null`; do cp -f ${LOG} ${CONFDIR}/${DECNAME}_enc_${SUFFIX}; done 
      fi
      if [ ! -f ${CONFDIR}/${DECNAME}_enc_${SUFFIX} ]
      then 
        for LOG in `ls ${DSTDIR}/encoderlog/*_dec_${SUFFIX} 2> /dev/null`; do cp -f ${LOG} ${CONFDIR}/${DECNAME}_enc_${SUFFIX}; done 
      fi 
      if [ ! -f ${CONFDIR}/${DECNAME}_enc_${SUFFIX} ]
      then  
        for LOG in `ls ${DSTDIR}/encoderlog/*_${SUFFIX} 2> /dev/null`; do cp -f ${LOG} ${CONFDIR}/${DECNAME}_enc_${SUFFIX}; done 
      fi       
      if [ ! -f ${CONFDIR}/${DECNAME}_enc_${SUFFIX} ]
      then 
        echo "  ${CONFDIR}/${DECNAME}_enc_${SUFFIX} can\'t be found in encoderlog folder"
        MISSINGFILE=1;
      fi 
      
      # Copy decoder files
      for LOG in `ls ${DSTDIR}/decoderlog/*_dec_${SUFFIX}`; do cp -f ${LOG} ${CONFDIR}/${DECNAME}_dec_${SUFFIX}; done       
      if [ ! -f ${CONFDIR}/${DECNAME}_dec_${SUFFIX} ]
      then 
        echo "  ${CONFDIR}/${DECNAME}_dec_${SUFFIX} cant be found in encoderlog folder"
        MISSINGFILE=1;
      fi 
    done 
    
    if [ $MISSINGFILE == 1 ]
    then 
        echo -e "  \033[0;31m All log files are not present \033[0m";
    else 
      echo "  Start conformance tests: ${DECNAME}_conformance.log"
      # Start conformance software
      ${TMC2CONF} \
        --checkConformance=1 \
        --path=${CONFDIR}/${DECNAME}_ \
        --level=105 \
        --fps=30 \
      > ${CONFDIR}/${DECNAME}_conformance.log 2>&1
      
      # Extract conformance software results
      NUMKVMATCH=`   cat ${CONFDIR}/${DECNAME}_conformance.log | grep "File Check Tests"   | awk '{print $7}'`; 
      NUMTESTFILES=` cat ${CONFDIR}/${DECNAME}_conformance.log | grep "File Check Tests"   | awk '{print $9}'`; 
      NUMLIMITMATCH=`cat ${CONFDIR}/${DECNAME}_conformance.log | grep "Level Limits Tests" | awk '{print $8}'`;
      NUMTESTLIMITS=`cat ${CONFDIR}/${DECNAME}_conformance.log | grep "Level Limits Tests" | awk '{print $10}'`; 
      DC=`expr $NUMTESTFILES  - $NUMKVMATCH `;
      DL=`expr $NUMTESTLIMITS - $NUMLIMITMATCH`; 
      
      # Display status    
      echo "  Disaply conformance status:"
      if (( "$NUMKVMATCH" > 0))
      then
        echo -e "\033[0;32m  Conformances file tests match for $NUMKVMATCH / $NUMTESTFILES \033[0m"
      fi
      if (( "$DC" > 0))
      then
        echo -e "\033[0;31m  Conformances file tests do not match for $DC / $NUMTESTFILES check file tests \033[0m"
      fi
      if (( "$NUMLIMITMATCH" > 0))
      then
        echo -e "\033[0;32m  Conformances file limit tests pass for $NUMLIMITMATCH / $NUMTESTLIMITS \033[0m"
      fi
      if (( "$DL" > 0))
      then
        echo -e "\033[0;31m  Conformances file limit tests do not pass for $DL / $NUMTESTLIMITS check file tests \033[0m"
      fi
    fi    
  done 
done 


