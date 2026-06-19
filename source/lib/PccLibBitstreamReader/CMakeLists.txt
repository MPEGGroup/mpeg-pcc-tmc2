cmake_minimum_required(VERSION 3.5)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
project(${MYNAME} C CXX)

file(GLOB SRC include/*.h source/*.cpp)

include_directories(
  include ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include)

add_library(${MYNAME} ${LINKER} ${SRC})

target_link_libraries(${MYNAME} PccLibBitstreamCommon)

set_target_properties(${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)
