#!/bin/bash

CURDIR=`dirname $0`;
echo -e "\033[0;32mClean: $(readlink -f $CURDIR) \033[0m";

rm -rf \
  ${CURDIR}/bin/ \
  ${CURDIR}/lib/ \
  ${CURDIR}/build/ \
  ${CURDIR}/CMakeCache.txt \
  ${CURDIR}/CMakeFiles/ 