cmake_minimum_required(VERSION 3.5)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
set(MYNAME ${MYNAME}${CMAKE_DEBUG_POSTFIX})
project(${MYNAME} C CXX)

file(GLOB SRC *.h *.cpp *.c
     ${CMAKE_SOURCE_DIR}/dependencies/program-options-lite/*)

include_directories(
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibCommon/include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibDecoder/include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibMetrics/include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibConformance/include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamReader/include
  ${CMAKE_SOURCE_DIR}/dependencies/program-options-lite
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibConformance/include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibAdaptation/include)

set(LIBS
    PccLibCommon
    PccLibDecoder
    PccLibMetrics
    PccLibConformance
    PccLibBitstreamCommon
    PccLibBitstreamReader
    PccLibAdaptation)
if(ENABLE_TBB)
  include_directories(${CMAKE_SOURCE_DIR}/dependencies/tbb/include)
  set(LIBS ${LIBS} tbb_static)
endif()

add_executable(${MYNAME} ${SRC})

if(USE_FFMPEG_VIDEO_CODEC)
  if(WIN32
     OR MSVC
     OR MSYS
     OR MINGW)
    link_directories(${FFMPEG_DIR}/lib)
  endif()
endif()

target_link_libraries(${MYNAME} ${LIBS})

install(TARGETS ${MYNAME} DESTINATION bin)
