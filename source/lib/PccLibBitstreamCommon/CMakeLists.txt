cmake_minimum_required(VERSION 3.5)

get_filename_component(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" MYNAME ${MYNAME})
project(${MYNAME} C CXX)

enable_language(C)

include(CheckSymbolExists)
check_symbol_exists(getrusage sys/resource.h HAVE_GETRUSAGE)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/PCCConfig.h.in
               ${CMAKE_CURRENT_SOURCE_DIR}/include/PCCConfig.h)

file(GLOB PROJECT_INC_FILES include/*.h
     ${CMAKE_SOURCE_DIR}/dependencies/libmd5/*.h)

file(GLOB PROJECT_CPP_FILES source/*.cpp
     ${CMAKE_SOURCE_DIR}/dependencies/libmd5/*.c)

file(GLOB PROJECT_IN_FILES *.in)

source_group(inc FILES ${PROJECT_INC_FILES})
source_group(cpp FILES ${PROJECT_CPP_FILES})
source_group(input FILES ${PROJECT_IN_FILES})

include_directories(include ${CMAKE_SOURCE_DIR}/dependencies/libmd5)

add_library(${MYNAME} ${LINKER} ${SRC} ${PROJECT_INC_FILES}
                      ${PROJECT_CPP_FILES} ${PROJECT_IN_FILES})

set_target_properties(${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)
