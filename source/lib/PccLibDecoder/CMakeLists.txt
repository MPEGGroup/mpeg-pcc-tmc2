CMAKE_MINIMUM_REQUIRED (VERSION 2.8.11)

GET_FILENAME_COMPONENT(MYNAME ${CMAKE_CURRENT_LIST_DIR} NAME)
STRING(REPLACE " " "_" MYNAME ${MYNAME})
PROJECT(${MYNAME} C CXX)

FILE(GLOB SRC include/*.h source/*.cpp
              ${CMAKE_SOURCE_DIR}/dependencies/arithmetic-coding/inc/*.h
              ${CMAKE_SOURCE_DIR}/dependencies/arithmetic-coding/src/*.cpp
              ${CMAKE_SOURCE_DIR}/dependencies/nanoflann/*.hpp
              ${CMAKE_SOURCE_DIR}/dependencies/nanoflann/*.h )

INCLUDE_DIRECTORIES( include 
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibCommon/include  
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibBitstreamCommon/include 
                     ${CMAKE_SOURCE_DIR}/source/lib/PccLibVideoDecoder/include 
                     ${CMAKE_SOURCE_DIR}/dependencies/tbb/include
                     ${CMAKE_SOURCE_DIR}/dependencies/arithmetic-coding/inc
                     ${CMAKE_SOURCE_DIR}/dependencies/nanoflann )
                     

SET( LIBS PccLibCommon PccLibBitstreamCommon )
IF( NOT USE_HM_VIDEO_CODEC )
    INCLUDE_DIRECTORIES( ${CMAKE_SOURCE_DIR}/dependencies/PccLibHevcParser/include )
    SET( LIBS ${LIBS} PccLibHevcParser )
ENDIF()

ADD_LIBRARY( ${MYNAME} ${LINKER} ${SRC} )

IF( USE_HM_VIDEO_CODEC )
     SET( LIBS ${LIBS} PccLibVideoDecoder )
ENDIF()

TARGET_LINK_LIBRARIES(${MYNAME} ${LIBS} )

SET_TARGET_PROPERTIES( ${MYNAME} PROPERTIES LINKER_LANGUAGE CXX)

