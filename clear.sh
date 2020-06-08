#!/bin/bash

CURDIR=$(readlink -f `dirname $0` );
echo -e "\033[0;32mClean: $CURDIR \033[0m";

rm -rf \
  ${CURDIR}/bin/ \
  ${CURDIR}/lib/ \
  ${CURDIR}/build/ \
  ${CURDIR}/dependencies/papi/ \
  ${CURDIR}/CMakeCache.txt \
  ${CURDIR}/CMakeFiles/ 

if [ "$#" -gt "0" ] 
then 
  if [ "$1" == "hm" ] || [ "$1" == "all" ]
  then 
    echo -e "\033[0;32mClean: ${CURDIR}/dependencies/HM-16.20+SCM-8.8 \033[0m";
    rm -rf ${CURDIR}/dependencies/HM-16.20+SCM-8.8;
  fi
  if [ "$1" == "hdrtools" ] || [ "$1" == "all" ] 
  then 
    echo -e "\033[0;32mClean: ${CURDIR}/dependencies/HDRTools \033[0m";
    rm -rf ${CURDIR}/dependencies/HDRTools;
  fi
fi