cmake_minimum_required(VERSION 3.2)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
set(MYNAME ${MYNAME}${CMAKE_DEBUG_POSTFIX})
project(${MYNAME} C CXX)

file(GLOB SRC include/*.h source/*.cpp)

include_directories(
  include ${CMAKE_SOURCE_DIR}/source/lib/PccLibCommon/include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include)

add_library(${MYNAME} ${LINKER} ${SRC})

target_link_libraries(${MYNAME})

set_target_properties(${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)
