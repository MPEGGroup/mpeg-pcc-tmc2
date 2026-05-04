cmake_minimum_required(VERSION 3.2)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
project(${MYNAME} C CXX)

file(GLOB SRC include/*.h source/*.cpp ${HM_LIB_SOURCE_DIR}/TAppCommon/*)

include_directories(
  include ${CMAKE_SOURCE_DIR}/source/lib/PccLibCommon/include/
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include/)

set(LIBS PccLibCommon)
if(USE_HMLIB_VIDEO_CODEC)
  include_directories(${HM_LIB_SOURCE_DIR}/)
  set(LIBS ${LIBS} TLibCommon TLibDecoder)
endif()
if(USE_JMLIB_VIDEO_CODEC)
  include_directories(${JM_LIB_SOURCE_DIR}/lib/lcommon/
                      ${JM_LIB_SOURCE_DIR}/app/ldecod/)
  set(LIBS ${LIBS} ldecod)
endif()
if(USE_FFMPEG_VIDEO_CODEC)
  include_directories(${FFMPEG_INCLUDE})
  if(WIN32
     OR MSVC
     OR MSYS
     OR MINGW)
    link_directories(${FFMPEG_DEV_DIR}/lib)
  endif()
  set(LIBS ${LIBS} ${AVCODEC_LIBRARY} ${AVUTIL_LIBRARY} ${AVFORMAT_LIBRARY})
endif()
if(USE_VTMLIB_VIDEO_CODEC)
  set(LIBS ${LIBS} CommonLib DecoderLib_vtm)
endif()
if(USE_VVLIB_VIDEO_CODEC)
  set(LIBS ${LIBS} vvdec)
endif()
if(USE_HMAPP_VIDEO_CODEC)
  include_directories(${CMAKE_SOURCE_DIR}/dependencies/PccLibHevcParser/include)
  set(LIBS ${LIBS} PccLibHevcParser)
endif()
if(USE_JMAPP_VIDEO_CODEC)
  include_directories(${CMAKE_SOURCE_DIR}/dependencies/PccLibAvcParser/include)
  set(LIBS ${LIBS} PccLibAvcParser)
endif()
if(USE_SHMAPP_VIDEO_CODEC)
  include_directories(${CMAKE_SOURCE_DIR}/dependencies/PccLibShvcParser/include)
  set(LIBS ${LIBS} PccLibShvcParser)
endif()
add_library(${MYNAME} ${LINKER} ${SRC})

if(MSVC)
  target_link_libraries(${MYNAME} ${LIBS} ws2_32)
else()
  target_link_libraries(${MYNAME} ${LIBS})
endif()

set_target_properties(${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)
