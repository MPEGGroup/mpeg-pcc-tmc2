cmake_minimum_required(VERSION 3.5)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
project(${MYNAME} C CXX)

file(GLOB SRC include/*.h source/*.cpp ${HM_LIB_SOURCE_DIR}/TAppCommon/*)

set(LIBS PccLibCommon)
include_directories(
  include ${CMAKE_SOURCE_DIR}/source/lib/PccLibCommon/include/
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include/)

if(USE_HMLIB_VIDEO_CODEC)
  include_directories(${HM_LIB_SOURCE_DIR}/)
  set(LIBS ${LIBS} TLibCommon TLibEncoder)
endif()
if(USE_JMLIB_VIDEO_CODEC)
  include_directories(${JM_LIB_SOURCE_DIR}/lib/lcommon/
                      ${JM_LIB_SOURCE_DIR}/app/lencod/)
  set(LIBS ${LIBS} lencod)
endif()
if(USE_FFMPEG_VIDEO_CODEC)
  include_directories(${FFMPEG_INCLUDE} ${KVAZAAR_INCLUDE})
  if(WIN32
     OR MSVC
     OR MSYS
     OR MINGW)
    link_directories(${FFMPEG_DEV_DIR}/lib)
  endif()
  include_directories(
    ${CMAKE_SOURCE_DIR}/source/lib/PccLibVideoDecoder/include/)
  set(LIBS ${LIBS} ${KVAZAAR_LIBRARY} ${AVCODEC_LIBRARY} ${AVUTIL_LIBRARY}
           ${AVFORMAT_LIBRARY} PccLibVideoDecoder)
endif()
if(USE_VTMLIB_VIDEO_CODEC)
  set(LIBS ${LIBS} CommonLib EncoderLib_vtm DecoderLib_vtm Utilities_vtm)
endif()
if(USE_VVLIB_VIDEO_CODEC)
  include_directories(
    ${CMAKE_SOURCE_DIR}/source/lib/PccLibVideoDecoder/include/)
  set(LIBS ${LIBS} vvenc vvdec PccLibVideoDecoder)
endif()

add_library(${MYNAME} ${LINKER} ${SRC})

# TARGET_LINK_LIBRARIES( ${MYNAME} ${LIBS} )
if(MSVC)
  target_link_libraries(${MYNAME} ${LIBS} ws2_32)
else()
  target_link_libraries(${MYNAME} ${LIBS})
endif()

set_target_properties(${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)
