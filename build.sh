#!/bin/bash

CURDIR=`dirname $0`;
echo -e "\033[0;32mBuild: $(readlink -f $CURDIR) \033[0m";

MODE=Release;
CMAKE_FLAGS=;

if [ "$(expr substr $(uname -s) 1 5)" == "Linux" -o "$(expr substr $(uname -s) 1 5)" == "MINGW" ] 
then 
  NUMBER_OF_PROCESSORS=`grep -c ^processor /proc/cpuinfo`; 
else 
  NUMBER_OF_PROCESSORS=4;  
  echo "Force configuration for windows: CMAKE_FLAGS = $CMAKE_FLAGS"
fi

for i in "$@"
do
  case "$i" in
    debug|Debug|DEBUG      ) MODE=Debug;;
    release|Release|RELEASE) MODE=Release;;
    *                      ) echo "ERROR: arguments \"$i\" not supported: option = [debug|release]"; exit -1;;
  esac
done

CMAKE_FLAGS="$CMAKE_FLAGS -DCMAKE_BUILD_TYPE=$MODE";

cmake -H${CURDIR} -B${CURDIR}/build/${MODE} -G "Unix Makefiles" ${CMAKE_FLAGS}

make -C ${CURDIR}/build/${MODE} -j ${NUMBER_OF_PROCESSORS} ${CMD} -s 
