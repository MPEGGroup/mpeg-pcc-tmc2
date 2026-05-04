cmake_minimum_required(VERSION 3.2)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
project(${MYNAME} C CXX Fortran)

file(GLOB SRC include/*.h source/*.cpp)

include_directories(
  include ${CMAKE_SOURCE_DIR}/source/lib/PccLibCommon/include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include)
if(ENABLE_TBB)
  include_directories(${CMAKE_SOURCE_DIR}/dependencies/tbb/include)
endif()

set(LIBS PccLibCommon)

add_library(${MYNAME} ${LINKER} ${SRC})

target_link_libraries(${MYNAME} ${LIBS} ${LAPACK_LIBRARIES})

set_target_properties(${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)
