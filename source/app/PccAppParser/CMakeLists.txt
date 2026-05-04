cmake_minimum_required(VERSION 3.2)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
set(MYNAME ${MYNAME}${CMAKE_DEBUG_POSTFIX})
project(${MYNAME} C CXX)

file(GLOB SRC *.h *.cpp *.c)

include_directories(
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include
  ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamReader/include)

add_executable(${MYNAME} ${SRC})

set(LIBS PccLibBitstreamCommon PccLibBitstreamReader)

target_link_libraries(${MYNAME} ${LIBS})

install(TARGETS ${MYNAME} DESTINATION bin)
