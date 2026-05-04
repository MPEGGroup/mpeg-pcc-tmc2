cmake_minimum_required(VERSION 3.2)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
project(${MYNAME} C CXX)

file(GLOB SRC include/*.h source/*.cpp
     ${HDRTOOLS_DIR}/projects/HDRConvert/inc/ProjectParameters.h
     ${HDRTOOLS_DIR}/projects/HDRConvert/src/ProjectParameters.cpp)

include_directories(
  include ${CMAKE_SOURCE_DIR}/source/lib/PccLibCommon/include/
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include/
  ${HDRTOOLS_DIR}/common/inc ${HDRTOOLS_DIR}/projects/HDRConvert/inc)

set(LIBS PccLibCommon)
if(USE_HDRTOOLS)
  set(LIBS ${LIBS} HDRLib)
endif()
add_library(${MYNAME} ${LINKER} ${SRC})

target_link_libraries(${MYNAME} ${LIBS})

set_target_properties(${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)
