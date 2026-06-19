cmake_minimum_required(VERSION 3.5)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
set(MYNAME ${MYNAME}${CMAKE_DEBUG_POSTFIX})
project(${MYNAME} C CXX)

file(GLOB SRC *.h *.cpp *.c
     ${CMAKE_SOURCE_DIR}/dependencies/program-options-lite/*)

include_directories(
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibCommon/include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibVideoEncoder/include
  ${CMAKE_SOURCE_DIR}/dependencies/program-options-lite)

add_executable(${MYNAME} ${SRC})

set(LIBS PccLibCommon PccLibBitstreamCommon PccLibVideoEncoder)

target_link_libraries(${MYNAME} ${LIBS})
if(MSVC)
  set_target_properties(${MYNAME} PROPERTIES LINK_FLAGS /STACK:4194304)
endif()

install(TARGETS ${MYNAME} DESTINATION bin)
