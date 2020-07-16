#!/bin/bash

CURDIR=`dirname $0`;
echo -e "\033[0;32mBuild: $(readlink -f $CURDIR) \033[0m";

CMAKE="";
if [ `cmake  --version | grep version | awk '{print $3 }' | awk -F '.' '{print $1}'` == 3 ] ; then CMAKE=cmake; fi
if [ `cmake3 --version | grep version | awk '{print $3 }' | awk -F '.' '{print $1}'` == 3 ] ; then CMAKE=cmake3; fi
if [ "$CMAKE" == "" ] ; then echo "Can't find cmake > 3.0"; exit; fi

case "$(uname -s)" in
  Linux*)     MACHINE=Linux;;
  Darwin*)    MACHINE=Mac;;
  CYGWIN*)    MACHINE=Cygwin;;
  MINGW*)     MACHINE=MinGw;;
  *)          MACHINE="UNKNOWN:${unameOut}"
esac

buildWindows() {
  MSBUILD="/c/Program Files (x86)/Microsoft Visual Studio/2019/Professional/MSBuild/Current/Bin/MSBuild.exe"
  printf -v MSBUILDPATH '%q' "$MSBUILD"
  MSBUILDFOUND=`ls "${MSBUILD[@]}" 2>&1 | grep "No such file or directory"`
  if [ "$MSBUILDFOUND" == "" ] 
  then 
    eval ${MSBUILDPATH[@]} $1 -p:Configuration=$2 -maxcpucount:8
  else
    echo " Could not directly build project in command line because MsBuild not found ($MSBUILD)";
    echo " "
    echo " Please build solution in Microsoft Visual Studio:" 
    echo "   - Open the generated Visual Studio solution: ${CURDIR}/$1";      
    echo "   - Build $1"        
  fi
}

MODE=Release;
FORMAT=0;
TIDY=0;
CMAKE_FLAGS=;
if [ "$MACHINE" == "Linux" ] ; then NUMBER_OF_PROCESSORS=`grep -c ^processor /proc/cpuinfo`; fi

for i in "$@"
do  
  case "$i" in
    doc|Doc|doc|doc\/      ) make -C ${CURDIR}/doc/; exit;;
    debug|Debug|DEBUG      ) MODE=Debug; CMAKE_FLAGS="$CMAKE_FLAGS-DCMAKE_C_FLAGS=\"-g3\" -DCMAKE_CXX_FLAGS=\"-g3\" ";;
    release|Release|RELEASE) MODE=Release;;
    profiling|Profiling|PROFILING) CMAKE_FLAGS="$CMAKE_FLAGS -DENABLE_PAPI_PROFILING=ON ";;
    format                )  FORMAT=1;;
    tidy                     )    TIDY=1;;
    verbose|VERBOSE)       CMD=VERBOSE=1;;
    *                        ) echo "ERROR: arguments \"$i\" not supported: option = [debug|release]"; exit -1;;
  esac
done
CMAKE_FLAGS="$CMAKE_FLAGS -DCMAKE_BUILD_TYPE=$MODE -DCMAKE_EXPORT_COMPILE_COMMANDS=ON";


# Get default cmake configuration: 
CMAKE_GENERATORS=`$CMAKE -G 2>&1 | grep "*" | cut -c2- | awk -F "=" '{print $1}' | sed -e 's/^[ \t]*//;s/[ \t]*$//'`;
if [ "$CMAKE_GENERATORS" == "" ] ; then CMAKE_GENERATORS="Unix Makefiles"; fi

echo "CMAKE      = $CMAKE"
echo "MACHINE    = $MACHINE";
echo "GENERATORS = $CMAKE_GENERATORS";
if [ $MACHINE != "Linux" ] 
then 
  echo "Windows cmake"
  ${CMAKE} -H${CURDIR} -B${CURDIR}/build/${MODE}   -G "${CMAKE_GENERATORS}" -A x64 ${CMAKE_FLAGS}
else
  ${CMAKE} -H${CURDIR} -B${CURDIR}/build/${MODE} -G "${CMAKE_GENERATORS}"  ${CMAKE_FLAGS} 
fi 

if [ $FORMAT == 1 ] 
then 
  echo -e "\033[0;32mFormat: $(readlink -f $CURDIR) \033[0m";
  case "${MACHINE}" in
    Linux) ${CMAKE} --build ${CURDIR}/build/${MODE} --target clang-format;; 
    Mac)   echo "Please, open the generated xcode project and build it ";;
    *)     buildWindows ./build/${MODE}/clang-format.vcxproj  ${MODE};;
  esac 
  echo -e "\033[0;32mdone \033[0m";
  exit;
fi 

if [ $TIDY == 1 ] 
then 
  echo -e "\033[0;32mFormat: $(readlink -f $CURDIR) \033[0m";
  case "${MACHINE}" in
    Linux) ${CMAKE} --build ${CURDIR}/build/${MODE} --target clang-tidy;; 
    Mac)   echo "Please, open the generated xcode project and build it ";;
    *)     buildWindows ./build/${MODE}/clang-tidy.vcxproj ${MODE};;
  esac 
  echo -e "\033[0;32mdone \033[0m";
  exit;
fi 

echo -e "\033[0;32mBuild: $(readlink -f $CURDIR) \033[0m";
case "${MACHINE}" in
  Linux) make -C ${CURDIR}/build/${MODE} -j ${NUMBER_OF_PROCESSORS} ${CMD} -s ;  if [[ $? -ne 0 ]] ; then exit 1; fi ;;
  Mac)   echo "Please, open the generated xcode project and build it ";;
  *)     buildWindows ${CURDIR}/build/${MODE}/TMC2.sln ${MODE};;
esac 
echo -e "\033[0;32mdone \033[0m";

