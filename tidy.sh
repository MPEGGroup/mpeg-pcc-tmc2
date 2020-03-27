#!/bin/bash

echo "$@"


CURDIR=`dirname $0`;

#Checks: "-*,
#performance-*,
#google-readability-casting,
#  modernize-*,-modernize-use-trailing-return-type,
# # readability-*,
# "

for check in "performance-*" "google-readability-casting" "modernize-*,-modernize-use-trailing-return-type" "readability-*"
do 
  for name in `ls  source/lib/*/source/*.cpp source/lib/*/include/*.h source/app/*/*.cpp source/app/*/*.h `
  do
    clang-tidy \
      --checks=-*,$check \
      -p ./build/  \
      --header-filter=source/*.h \
      --system-headers \
      --fix \
      --fix-errors \
      $name \
      -- \
      -I${CURDIR}/dependencies/nanoflann \
      -I${CURDIR}/dependencies/libmd5 \
      -I${CURDIR}/dependencies/tbb/include \
      -I${CURDIR}/dependencies/program-options-lite \
      -I${CURDIR}/source/lib/PccLibBitstreamCommon/include  \
      -I${CURDIR}/source/lib/PccLibBitstreamReader/include  \
      -I${CURDIR}/source/lib/PccLibBitstreamWriter/include  \
      -I${CURDIR}/source/lib/PccLibCommon/include  \
      -I${CURDIR}/source/lib/PccLibDecoder/include \
      -I${CURDIR}/source/lib/PccLibEncoder/include \
      -I${CURDIR}/source/lib/PccLibMetrics/include \
      -I${CURDIR}/source/app/PccAppDecoder/include \
      -I${CURDIR}/source/app/PccAppEncoder/include \
      -I${CURDIR}/source/app/PccAppMetrics/include \
      -I${CURDIR}/source/app/PccAppParser/include  

    git checkout ./dependencies/tbb/
  done
done

  exit ;
  $@ 



  # -I/usr/include/c++/7.4.0 \  -I /usr/include/x86_64-linux-gnu/c++/7/32 \


# ;/srv/wp21/PCC/ricardj/softwares/mpeg-pcc-renderer/include/PccRendererDef.h;/srv/wp21/PCC/ricardj/softwares/mpeg-pcc-renderer/include/PccRendererMath.h;/srv/wp21/PCC/ricardj/softwares/mpeg-pcc-renderer/include/PccRendererPlyReader.h;/srv/wp21/PCC/ricardj/softwares/mpeg-pcc-renderer/include/PccRendererProgram.h;/srv/wp21/PCC/ricardj/softwares/mpeg-pcc-renderer/include/PccRendererShader.h;/srv/wp21/PCC/ricardj/softwares/mpeg-pcc-renderer/include/PccRendererText.h;/srv/wp21/PCC/ricardj/softwares/mpeg-pcc-renderer/include/PccRendererWindow.h;/srv/wp21/PCC/ricardj/softwares/mpeg-pcc-renderer/source/PccRendererCamera.cpp;/srv/wp21/PCC/ricardj/softwares/mpeg-pcc-renderer/source/PccRendererMain.cpp;/srv/wp21/PCC/ricardj/softwares/mpeg-pcc-renderer/source/PccRendererPlyReader.cpp;/srv/wp21/PCC/ricardj/softwares/mpeg-pcc-renderer/source/PccRendererProgram.cpp;/srv/wp21/PCC/ricardj/softwares/mpeg-pcc-renderer/source/PccRendererShader.cpp;/srv/wp21/PCC/ricardj/softwares/mpeg-pcc-renderer/source/PccRendererText.cpp;/srv/wp21/PCC/ricardj/softwares/mpeg-pcc-renderer/source/PccRendererWindow.cpp




  # -system-headers  

#clang-tidy --list-checks -checks='*' | grep "modernize"