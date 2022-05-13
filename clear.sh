#!/bin/bash

CURDIR=$( cd "$( dirname "$0" )" && pwd ); 
echo -e "\033[0;32mClean: $CURDIR \033[0m"; 

rm -rf \
  "${CURDIR}/bin/" \
  "${CURDIR}/lib/" \
  "${CURDIR}/build/" \
  "${CURDIR}/dependencies/papi/" \
  "${CURDIR}/CMakeCache.txt" \
  "${CURDIR}/CMakeFiles"/ 

if [ "$#" -gt "0" ] 
then 
  if [ "$1" == "hm" ] || [ "$1" == "all" ]
  then 
    echo -e "\033[0;32mClean: ${CURDIR}/dependencies/HM-16.20+SCM-8.8 \033[0m";
    rm -rf "${CURDIR}/dependencies/HM-16.20+SCM-8.8";
  fi
  if [ "$1" == "shm" ] || [ "$1" == "all" ]
  then 
    echo -e "\033[0;32mClean: ${CURDIR}/dependencies/SHM-12.4 \033[0m";
    rm -rf "${CURDIR}/dependencies/SHM-12.4";
  fi
  if [ "$1" == "jm" ] || [ "$1" == "all" ]
  then 
    echo -e "\033[0;32mClean: ${CURDIR}/dependencies/jm19.0_app \033[0m";
    rm -rf "${CURDIR}/dependencies/jm19.0_app";
    echo -e "\033[0;32mClean: ${CURDIR}/dependencies/jm19.0_lib \033[0m";
    rm -rf "${CURDIR}/dependencies/jm19.0_lib";
  fi
  if [ "$1" == "hdrtools" ] || [ "$1" == "all" ] 
  then 
    echo -e "\033[0;32mClean: ${CURDIR}/dependencies/HDRTools \033[0m";
    rm -rf "${CURDIR}/dependencies/HDRTools";
  fi
  if [ "$1" == "ffmpeg" ] || [ "$1" == "all" ] 
  then 
    echo -e "\033[0;32mClean: ${CURDIR}/dependencies/ffmpeg \033[0m";
    rm -rf "${CURDIR}/dependencies/ffmpeg";
  fi
  if [ "$1" == "vtm" ] || [ "$1" == "all" ] 
  then 
    echo -e "\033[0;32mClean: ${CURDIR}/dependencies/VTM-11.2 \033[0m";
    rm -rf "${CURDIR}/dependencies/VTM-11.2/";
  fi
  if [ "$1" == "vv" ] || [ "$1" == "all" ] 
  then 
    echo -e "\033[0;32mClean: ${CURDIR}/dependencies/vvenc \033[0m";
    rm -rf "${CURDIR}/dependencies/vvenc/";
    echo -e "\033[0;32mClean: ${CURDIR}/dependencies/vvdec \033[0m";
    rm -rf "${CURDIR}/dependencies/vvdec/";
  fi
fi
