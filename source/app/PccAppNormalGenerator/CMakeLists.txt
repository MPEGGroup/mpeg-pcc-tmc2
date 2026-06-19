cmake_minimum_required(VERSION 3.5)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
set(MYNAME ${MYNAME}${CMAKE_DEBUG_POSTFIX})
project(${MYNAME} C CXX)

file(
  GLOB
  SRC
  *.h
  *.cpp
  *.c
  ${CMAKE_SOURCE_DIR}/dependencies/program-options-lite/*
  ${CMAKE_SOURCE_DIR}/dependencies/nanoflann/*.hpp
  ${CMAKE_SOURCE_DIR}/dependencies/nanoflann/*.h)

include_directories(
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibCommon/include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibEncoder/include
  ${CMAKE_SOURCE_DIR}/dependencies/program-options-lite
  ${CMAKE_SOURCE_DIR}/dependencies/nanoflann)

set(LIBS PccLibCommon PccLibEncoder)
if(ENABLE_TBB)
  include_directories(${CMAKE_SOURCE_DIR}/dependencies/tbb/include)
  set(LIBS ${LIBS} tbb_static)
endif()

add_executable(${MYNAME} ${SRC})

target_link_libraries(${MYNAME} ${LIBS})

install(TARGETS ${MYNAME} DESTINATION bin)
