#!/bin/bash

CURDIR=`dirname $0`;
echo -e "\033[0;32mClean: $(readlink -f $CURDIR) \033[0m";

${CURDIR}/PccVpccBitstream/clear.sh 

rm -rf \
  ${CURDIR}/bin/ \
  ${CURDIR}/lib/ \
  ${CURDIR}/build/ \
  ${CURDIR}/dependencies/papi/ \
  ${CURDIR}/CMakeCache.txt \
  ${CURDIR}/CMakeFiles/ 