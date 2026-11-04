#!/bin/bash

# MPEG-PCC-TMC2 Build Script
# Usage:
#   ./build.sh                  # Release build with default HM-16.20
#   ./build.sh debug            # Debug build
#   ./build.sh hm=18.0          # Specify HM version (16.20, 18.0, etc.)
#   ./build.sh debug hm=18.0    # Combine options
#   ./build.sh ffmpeg           # Enable FFMPEG (Kvazaar HEVC) video codec
#   ./build.sh format           # Format code
#   ./build.sh tidy             # Run clang-tidy
#
# Available options:
#   debug, release, profiling, format, tidy, ffmpeg, hm=VERSION

CURDIR=$( cd "$( dirname "$0" )" && pwd ); 
echo -e "\033[0;32mBuild: ${CURDIR} \033[0m";

CMAKE=""; 
CMAKE_MAJOR=$( cmake --version 2>&1 | grep version | awk '{print $3}' | awk -F '.' '{print $1}' );
if [ "$CMAKE_MAJOR" -ge 3 ] 2>/dev/null ; then CMAKE=cmake; fi
if [ "$CMAKE" == "" ] ; then 
  CMAKE_MAJOR=$( cmake3 --version 2>&1 | grep version | awk '{print $3}' | awk -F '.' '{print $1}' );
  if [ "$CMAKE_MAJOR" -ge 3 ] 2>/dev/null ; then CMAKE=cmake3; fi
fi
if [ "$CMAKE" == "" ] ; then echo "Can't find cmake >= 3.0"; exit 1; fi

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
HM_VERSION=18.0;
FFMPEG=0;
FFMPEG_PREFIX="${CURDIR}/dependencies/ffmpeg-kvazaar/install";
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
    ffmpeg    ) FFMPEG=1;;
    hm=*      ) HM_VERSION="${i#hm=}";;
    *         ) echo "ERROR: arguments \"$i\" not supported: option = [debug|release|profiling|format|tidy|ffmpeg|hm=VERSION]"; exit 1;;
  esac
done

CMAKE_FLAGS+=( "-DCMAKE_BUILD_TYPE=$MODE" "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON" );

# Map user-friendly version to CMake HM_VERSION string
case "$HM_VERSION" in
  16.20) HM_CMAKE_VERSION="HM-16.20+SCM-8.8" ;;
  18.0)  HM_CMAKE_VERSION="HM-18.0" ;;
  *)     HM_CMAKE_VERSION="HM-${HM_VERSION}" ;;
esac

CMAKE_FLAGS+=( "-DHM_VERSION=${HM_CMAKE_VERSION}" );
echo -e "\033[0;32mConfiguring with HM_VERSION=${HM_CMAKE_VERSION}...\033[0m";

if [ $FFMPEG == 1 ] ; then
  if [ ! -d "${FFMPEG_PREFIX}/include/libavcodec" ] ; then
    echo -e "\033[0;31mFFMPEG requested but ${FFMPEG_PREFIX} not found.\033[0m";
    echo -e "\033[0;31mRun dependencies/ffmpeg-kvazaar/build_ffmpeg_kvazaar.sh first.\033[0m";
    exit 1;
  fi
  CMAKE_FLAGS+=( "-DUSE_FFMPEG_VIDEO_CODEC=ON" "-DFFMPEG_DIR=${FFMPEG_PREFIX}" );
  export PKG_CONFIG_PATH="${FFMPEG_PREFIX}/lib/pkgconfig:${FFMPEG_PREFIX}/lib64/pkgconfig:${PKG_CONFIG_PATH}";
  echo -e "\033[0;32mFFMPEG (Kvazaar) codec enabled: ${FFMPEG_PREFIX}\033[0m";
fi

echo -e "\033[0;32mCmake: ${CURDIR}\033[0m";
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
