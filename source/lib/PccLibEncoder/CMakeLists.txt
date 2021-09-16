CMAKE_MINIMUM_REQUIRED(VERSION 3.2)

GET_FILENAME_COMPONENT(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
STRING(REPLACE " " "_" MYNAME ${MYNAME})
PROJECT(${MYNAME} C CXX)

FILE(GLOB SRC include/*.h source/*.cpp 
              ${CMAKE_SOURCE_DIR}/dependencies/nanoflann/*.hpp
              ${CMAKE_SOURCE_DIR}/dependencies/nanoflann/*.h )

INCLUDE_DIRECTORIES( include
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibCommon/include  
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include 
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibVideoEncoder/include 
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibColorConverter/include/
                     ${CMAKE_SOURCE_DIR}/dependencies/tbb/include
                     ${CMAKE_SOURCE_DIR}/dependencies/nanoflann )

SET( LIBS PccLibCommon PccLibBitstreamCommon PccLibVideoEncoder PccLibColorConverter )

ADD_LIBRARY( ${MYNAME} ${LINKER} ${SRC} )

TARGET_LINK_LIBRARIES( ${MYNAME} ${LIBS} )

SET_TARGET_PROPERTIES( ${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)

