#!/bin/bash

CURDIR=$( cd "$( dirname "$0" )" && pwd ); 
echo -e "\033[0;32mBuild: ${CURDIR} \033[0m";

CMAKE=""; 
if [ "$( cmake  --version 2>&1 | grep version | awk '{print $3 }' | awk -F '.' '{print $1}' )" == 3 ] ; then CMAKE=cmake; fi
if [ "$( cmake3 --version 2>&1 | grep version | awk '{print $3 }' | awk -F '.' '{print $1}' )" == 3 ] ; then CMAKE=cmake3; fi
if [ "$CMAKE" == "" ] ; then echo "Can't find cmake > 3.0"; exit; fi

case "$(uname -s)" in
  Linux*)     MACHINE=Linux;;
  Darwin*)    MACHINE=Mac;;
  CYGWIN*)    MACHINE=Cygwin;;
  MINGW*)     MACHINE=MinGw;;
  MSYS*)      MACHINE=MSys;;
  *)          MACHINE="UNKNOWN:$(uname -s)"
esac

MODE=Release;
FORMAT=0;
TIDY=0;
CMAKE_FLAGS=();
if [ "$MACHINE" == "Linux" ] ; then NUMBER_OF_PROCESSORS=$( grep -c ^processor /proc/cpuinfo ); fi

for i in "$@"
do  
  case "$i" in
    doc       ) make -C "${CURDIR}/doc/"; exit 0;;
    debug     ) MODE=Debug; CMAKE_FLAGS+=("-DCMAKE_C_FLAGS=\"-g3\"" "-DCMAKE_CXX_FLAGS=\"-g3\"" );;
    release   ) MODE=Release;;
    profiling ) CMAKE_FLAGS+=("-DENABLE_PAPI_PROFILING=ON");;
    format    ) FORMAT=1;;
    tidy      ) TIDY=1;;
    *         ) echo "ERROR: arguments \"$i\" not supported: option = [debug|release]"; exit 1;;
  esac
done

CMAKE_FLAGS+=( "-DCMAKE_BUILD_TYPE=$MODE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" );

echo -e "\033[0;32mCmake: ${CURDIR} \033[0m";
${CMAKE} -H${CURDIR} -B"${CURDIR}/build/${MODE}" "${CMAKE_FLAGS[@]}"
echo -e "\033[0;32mdone \033[0m";

if [ $FORMAT == 1 ] 
then 
  echo -e "\033[0;32mFormat: ${CURDIR} \033[0m";
  ${CMAKE} --build "${CURDIR}/build/${MODE}" --target clang-format
  echo -e "\033[0;32mdone \033[0m";
  exit 0;
fi 

if [ $TIDY == 1 ] 
then 
  echo -e "\033[0;32mFormat: ${CURDIR} \033[0m";
  ${CMAKE} --build "${CURDIR}/build/${MODE}" --target clang-tidy
  echo -e "\033[0;32mdone \033[0m";
  exit 0;
fi 

echo -e "\033[0;32mBuild: ${CURDIR} \033[0m";
if ! ${CMAKE} --build "${CURDIR}/build/${MODE}" --config ${MODE} --parallel "${NUMBER_OF_PROCESSORS}" ; then exit 1; fi 
echo -e "\033[0;32mdone \033[0m";


