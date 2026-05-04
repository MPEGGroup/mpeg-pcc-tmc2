cmake_minimum_required(VERSION 3.2)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
set(MYNAME ${MYNAME}${CMAKE_DEBUG_POSTFIX})
project(${MYNAME} C CXX)

file(GLOB SRC *.h *.cpp *.c
     ${CMAKE_SOURCE_DIR}/dependencies/program-options-lite/*)

include_directories(
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibCommon/include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibConformance/include
  ${CMAKE_SOURCE_DIR}/dependencies/program-options-lite)

set(LIBS PccLibCommon PccLibBitstreamCommon PccLibConformance)

add_executable(${MYNAME} ${SRC})

target_link_libraries(${MYNAME} ${LIBS})

install(TARGETS ${MYNAME} DESTINATION bin)
