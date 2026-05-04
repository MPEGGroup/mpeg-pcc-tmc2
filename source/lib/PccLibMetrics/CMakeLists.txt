cmake_minimum_required(VERSION 3.2)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
project(${MYNAME} C CXX)

file(GLOB SRC include/*.h source/*.cpp
     ${CMAKE_SOURCE_DIR}/dependencies/nanoflann/*.hpp
     ${CMAKE_SOURCE_DIR}/dependencies/nanoflann/*.h)

include_directories(
  include ${CMAKE_SOURCE_DIR}/source/lib/PccLibCommon/include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include
  ${CMAKE_SOURCE_DIR}/dependencies/nanoflann)

add_library(${MYNAME} ${LINKER} ${SRC})

target_link_libraries(${MYNAME} PccLibCommon)

set_target_properties(${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)
