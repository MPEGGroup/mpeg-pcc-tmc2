cmake_minimum_required(VERSION 3.2)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
project(${MYNAME} C CXX)

file(GLOB SRC include/*.h source/*.cpp)

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibCommon/include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamWriter/include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibVideoEncoder/include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibAdaptation/include)
if(ENABLE_TBB)
  include_directories(${CMAKE_SOURCE_DIR}/dependencies/tbb/include)
endif()

set(LIBS PccLibCommon PccLibBitstreamCommon PccLibBitstreamWriter
         PccLibVideoEncoder PccLibAdaptation)

add_library(${MYNAME} ${LINKER} ${SRC})

target_link_libraries(${MYNAME} ${LIBS})

set_target_properties(${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)
